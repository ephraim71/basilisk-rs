//! Switch extended Kalman filter (SEKF) for the sun heading in the body frame,
//! driven by coarse-sun-sensor (CSS) cosine readings: port of upstream Basilisk
//! `sunlineSEKF`.
//!
//! The filter carries five states: the sun heading `d` in body components and
//! the two observable components of the rate `ω_S/B` along `ŝ2` and `ŝ3` of the
//! sun frame `S = {ŝ1 = d̂, ŝ2 = (d × b̂)^, ŝ3 = (d × ŝ2)^}`. The frame is built
//! from a body axis `b̂` (`b̂1` after a reset). When the heading comes within
//! `acos(0.866)` (about 30°) of `b̂`, the filter switches to the other body
//! axis (`b̂1` ↔ `b̂2`) and rotates the rate states and the covariance into the
//! new frame.
//!
//! Every update propagates the state and the state transition matrix with the
//! linearized dynamics and adds process noise to the rate states. When CSS
//! data is available it then applies every reading above
//! `sensor_use_thresh`: as an EKF update that moves the reference state, or,
//! while some covariance element exceeds `ekf_switch`, as a classical Kalman
//! filter update of the state error. Both use the Joseph covariance form.
//!
//! # Measurement write time
//!
//! Upstream processes the CSS message when it has been written (`isWritten`)
//! and its write time (`timeWritten`) is not older than the filter time. Rust
//! messages carry no write metadata, so a connected `css_data_in_msg` counts
//! as written and its `time_tag_s` stands for the write time. Upstream's CSS
//! constellation stamps `timeTag` with the write time, so the two agree for
//! that producer; an unconnected input behaves like upstream's never-written
//! message (time propagation only). A producer that leaves `time_tag_s` at
//! zero (upstream `cssComm` does) is only processed around time zero.
//!
//! # Upstream quirks kept
//!
//! * Reset re-reads the CSS configuration, re-initializes the frame axis, the
//!   measurement buffers and the process noise, but leaves the state, state
//!   error and covariance as they are (they are seeded once, by [`SunlineSekf::new`]).
//! * The switch threshold (0.866) is hard-coded: upstream exposes
//!   `switchTresh` but overwrites it in every reset.
//! * The EKF/CKF choice compares the covariance from *before* the time update
//!   with `ekf_switch`.
//! * `num_obs` is [`MAX_N_CSS_MEAS`] after a reset, and the measurement buffers
//!   and post-fit residuals keep stale entries past `num_obs`; both reach
//!   `filt_data_out_msg`.
//! * A single CSS observation yields a zero Kalman gain (upstream `mInverse`
//!   inverts a 1×1 matrix to zero), and no observation at all logs upstream's
//!   singular-matrix error. An innovation covariance with a NaN determinant
//!   (NaN `q_obs_val`, or overflow in a diverged filter) also inverts to zero.

use nalgebra::{
    Const, DMatrix, DVector, Dyn, Matrix2, Matrix3, OMatrix, SMatrix, SVector, Vector3,
};

use crate::messages::{
    CssArraySensorMsg, CssConfigMsg, EKF_N_STATES_SWITCH, Input, MAX_N_CSS_MEAS,
    NavigationAttitudeMsg, Output, SKF_N_STATES, SunlineFilterMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::{is_equal_within, m_inverse, normalize_or_zero};
use crate::utilities::rigid_body_kinematics::tilde;
use crate::{Module, SimulationContext};

const NUM_STATES: usize = EKF_N_STATES_SWITCH;
/// Upstream `switchTresh`, which every reset sets to this value.
const SWITCH_THRESHOLD: f64 = 0.866;
/// `|d × b̂|` below which upstream uses the identity for `[BS]`.
const DEGENERATE_FRAME_NORM: f64 = 1.0e-5;
/// Tolerance of upstream's check whether the frame axis is `b̂1`.
const FRAME_AXIS_TOLERANCE: f64 = 1.0e-10;

type StateVector = SVector<f64, NUM_STATES>;
type StateMatrix = SMatrix<f64, NUM_STATES, NUM_STATES>;
/// `num_obs × 5` measurement sensitivity matrix.
type MeasurementMatrix = OMatrix<f64, Dyn, Const<NUM_STATES>>;
/// `5 × num_obs` Kalman gain.
type GainMatrix = OMatrix<f64, Const<NUM_STATES>, Dyn>;

/// User-settable parameters of [`SunlineSekf`]. [`Default`] gives upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct SunlineSekfConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `qObsVal` \[-\]: variance of the CSS measurement noise.
    pub q_obs_val: f64,
    /// Upstream `qProcVal` \[-\]: variance of the process noise on the two
    /// rate states, read at reset.
    pub q_proc_val: f64,
    /// Upstream `sensorUseThresh` \[-\]: a CSS reading is used only when it
    /// exceeds this value.
    pub sensor_use_thresh: f64,
    /// Upstream `eKFSwitch` \[-\]: largest absolute covariance element above
    /// which the measurement update runs as a classical Kalman filter instead
    /// of an EKF.
    pub ekf_switch: f64,
    /// Initial reference state: upstream `state` as set before the first
    /// reset (heading, then the two `ω_S/B` rates along `ŝ2`, `ŝ3` \[rad/s\]).
    pub initial_state: SVector<f64, EKF_N_STATES_SWITCH>,
    /// Initial state error: upstream `x` as set before the first reset.
    pub initial_state_error: SVector<f64, EKF_N_STATES_SWITCH>,
    /// Initial covariance: upstream `covar` as set before the first reset.
    pub initial_covar: SMatrix<f64, EKF_N_STATES_SWITCH, EKF_N_STATES_SWITCH>,
}

impl Default for SunlineSekfConfig {
    fn default() -> Self {
        Self {
            name: "sunlineSEKF".to_string(),
            q_obs_val: 0.0,
            q_proc_val: 0.0,
            sensor_use_thresh: 0.0,
            ekf_switch: 0.0,
            initial_state: StateVector::zeros(),
            initial_state_error: StateVector::zeros(),
            initial_covar: StateMatrix::zeros(),
        }
    }
}

/// CSS measurement buffers of upstream `sunlineHMatrixYMeas`, sized for
/// [`MAX_N_CSS_MEAS`] observations. Only the first `num_obs` entries are
/// current; as upstream, the others keep values from earlier updates.
#[derive(Clone, Debug, PartialEq)]
pub struct CssMeasurements {
    /// Upstream `numObs` \[-\]: CSS readings used in the last measurement
    /// update; [`MAX_N_CSS_MEAS`] after a reset, as upstream.
    pub num_obs: usize,
    /// Upstream `obs` \[-\]: the readings used.
    pub obs: SVector<f64, MAX_N_CSS_MEAS>,
    /// Upstream `yMeas` \[-\]: residuals `cos - n̂ · d` of the readings used,
    /// against the propagated heading.
    pub y_meas: SVector<f64, MAX_N_CSS_MEAS>,
    /// Upstream `measMat` \[-\]: measurement sensitivity; row `i` holds the
    /// normal of the `i`-th sensor used in its first three columns.
    pub meas_mat: SMatrix<f64, MAX_N_CSS_MEAS, EKF_N_STATES_SWITCH>,
}

impl CssMeasurements {
    /// The buffers as upstream's reset leaves them.
    fn cleared() -> Self {
        Self {
            num_obs: MAX_N_CSS_MEAS,
            obs: SVector::zeros(),
            y_meas: SVector::zeros(),
            meas_mat: SMatrix::zeros(),
        }
    }

    /// Upstream `sunlineHMatrixYMeas`: record every reading above `threshold`,
    /// its residual against the heading in `state` and its sensitivity row.
    fn record(
        &mut self,
        state: &StateVector,
        cos_values: &[f64],
        normals_b: &[Vector3<f64>],
        threshold: f64,
    ) {
        let heading = heading(state);
        self.num_obs = 0;
        for (normal, &cos_value) in normals_b
            .iter()
            .zip(cos_values)
            .filter(|&(_, &cos_value)| cos_value > threshold)
        {
            let row = self.num_obs;
            self.obs[row] = cos_value;
            self.y_meas[row] = cos_value - heading.dot(normal);
            self.meas_mat
                .fixed_view_mut::<1, 3>(row, 0)
                .copy_from(&normal.transpose());
            self.num_obs += 1;
        }
    }

    /// The current rows of `meas_mat`.
    fn sensitivity(&self) -> MeasurementMatrix {
        self.meas_mat.rows(0, self.num_obs).into_owned()
    }

    /// The current entries of `y_meas`.
    fn residuals(&self) -> DVector<f64> {
        self.y_meas.rows(0, self.num_obs).into_owned()
    }
}

/// A state error and its covariance: upstream `xBar`/`covarBar` after a time
/// update, `x`/`covar` after a measurement update.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Estimate {
    state_error: StateVector,
    covar: StateMatrix,
}

/// Sunline switch-EKF: upstream `sunlineSEKF`.
#[derive(Clone, Debug)]
pub struct SunlineSekf {
    /// Filter parameters; `q_proc_val` is read at reset, the others on every
    /// update.
    pub config: SunlineSekfConfig,
    /// Upstream `cssDataInMsg`: CSS readings.
    pub css_data_in_msg: Input<CssArraySensorMsg>,
    /// Upstream `cssConfigInMsg`: CSS normals, read at reset.
    pub css_config_in_msg: Input<CssConfigMsg>,
    /// Upstream `navStateOutMsg`: normalized heading and filter time tag.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `filtDataOutMsg`: filter state, with the rates as `ω_B/N`.
    pub filt_data_out_msg: Output<SunlineFilterMsg>,
    /// Upstream `state`: reference state at `time_tag_s` (heading, then the
    /// `ω_S/B` rates along `ŝ2`, `ŝ3` \[rad/s\]).
    pub state: SVector<f64, EKF_N_STATES_SWITCH>,
    /// Upstream `x`: state error.
    pub state_error: SVector<f64, EKF_N_STATES_SWITCH>,
    /// Upstream `covar`: state covariance.
    pub covar: SMatrix<f64, EKF_N_STATES_SWITCH, EKF_N_STATES_SWITCH>,
    /// Upstream `stateTransition`: state transition matrix of the last time
    /// update.
    pub state_transition: SMatrix<f64, EKF_N_STATES_SWITCH, EKF_N_STATES_SWITCH>,
    /// Upstream `obs`, `yMeas`, `measMat` and `numObs`.
    pub measurements: CssMeasurements,
    /// Upstream `postFits` \[-\]: post-fit residuals `y_meas - H x`; entries
    /// past `num_obs` are stale.
    pub post_fits: SVector<f64, MAX_N_CSS_MEAS>,
    /// Upstream `timeTag` \[s\]: time of validity of the estimate.
    time_tag_s: f64,
    /// Upstream `bVec_B`: body axis the sun frame is built from.
    b_vec_b: Vector3<f64>,
    /// Upstream `procNoise`: `q_proc_val · I₂` as of the last reset.
    proc_noise: Matrix2<f64>,
    /// Upstream `cssNHat_B`, truncated to the configured sensors.
    css_normals_b: Vec<Vector3<f64>>,
}

impl SunlineSekf {
    /// Builds the filter and seeds its estimate from the configuration.
    pub fn new(config: SunlineSekfConfig) -> Self {
        Self {
            state: config.initial_state,
            state_error: config.initial_state_error,
            covar: config.initial_covar,
            config,
            css_data_in_msg: Input::default(),
            css_config_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            state_transition: StateMatrix::identity(),
            measurements: CssMeasurements::cleared(),
            post_fits: SVector::zeros(),
            time_tag_s: 0.0,
            b_vec_b: Vector3::x(),
            proc_noise: Matrix2::zeros(),
            css_normals_b: Vec::new(),
        }
    }

    /// Upstream `Reset_sunlineSEKF`: cache the CSS normals and re-initialize
    /// the filter bookkeeping. The estimate itself is kept, as upstream.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.css_config_in_msg.is_connected() {
            log::error!("sunlineSEKF.css_config_in_msg is not connected");
        }
        if !self.css_data_in_msg.is_connected() {
            log::error!("sunlineSEKF.css_data_in_msg is not connected");
        }

        let css_config = self.css_config_in_msg.read();
        if css_config.num_css > MAX_N_CSS_MEAS {
            // Upstream then reads past its arrays; the port truncates instead.
            log::error!(
                "sunlineSEKF.css_config_in_msg.num_css must not be greater than MAX_N_CSS_MEAS ({MAX_N_CSS_MEAS})"
            );
        }
        self.css_normals_b = css_config
            .active_units()
            .iter()
            .map(|unit| unit.n_hat_b)
            .collect();

        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.b_vec_b = Vector3::x();
        self.measurements = CssMeasurements::cleared();
        self.state_transition = StateMatrix::identity();
        self.proc_noise = Matrix2::identity() * self.config.q_proc_val;
    }

    /// Upstream `Update_sunlineSEKF` up to the output messages.
    fn run_filter(&mut self, call_time_nanos: u64) {
        let css_readings = self.css_data_in_msg.read();

        let heading_hat = normalize_or_zero(heading(&self.state));
        if self.b_vec_b.dot(&heading_hat).abs() > SWITCH_THRESHOLD {
            (self.b_vec_b, self.state, self.covar) =
                switch_frame(self.b_vec_b, &self.state, &self.covar);
        }

        // Upstream: `timeWritten * NANO2SEC >= timeTag && isWritten` (see
        // "Measurement write time" in the module docs).
        let measurement_time_s = css_readings.time_tag_s;
        if self.css_data_in_msg.is_connected() && measurement_time_s >= self.time_tag_s {
            let prior = self.time_update(measurement_time_s);
            let posterior = self.measurement_update(&prior, &css_readings);
            self.state_error = posterior.state_error;
            self.covar = posterior.covar;
        }
        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.time_tag_s {
            let prior = self.time_update(call_time_s);
            self.state_error = prior.state_error;
            self.covar = prior.covar;
        }

        let num_obs = self.measurements.num_obs;
        let fitted = self.measurements.meas_mat.rows(0, num_obs) * self.state_error;
        let post_fits = self.measurements.y_meas.rows(0, num_obs) - fitted;
        self.post_fits.rows_mut(0, num_obs).copy_from(&post_fits);
    }

    /// Upstream `sunlineTimeUpdate`: propagate the reference state and the
    /// state transition matrix to `update_time_s`, and return the propagated
    /// state error and covariance (with process noise).
    fn time_update(&mut self, update_time_s: f64) -> Estimate {
        let dt = update_time_s - self.time_tag_s;

        let dyn_mat = dynamics_matrix(&self.state, self.b_vec_b);
        let (state, state_transition) = propagate(&dyn_mat, self.b_vec_b, dt, &self.state);
        self.state = state;
        self.state_transition = state_transition;

        let gamma = process_noise_mapping(&self.state, self.b_vec_b, dt);
        let covar = state_transition * (self.covar * state_transition.transpose())
            + gamma * (self.proc_noise * gamma.transpose());
        self.time_tag_s = update_time_s;

        Estimate {
            state_error: state_transition * self.state_error,
            covar,
        }
    }

    /// Upstream `sunlineMeasUpdate`: apply the CSS readings to the propagated
    /// estimate, by EKF (moving the reference state) or CKF.
    fn measurement_update(&mut self, prior: &Estimate, readings: &CssArraySensorMsg) -> Estimate {
        let num_css = self.css_normals_b.len();
        self.measurements.record(
            &self.state,
            &readings.cos_values[..num_css],
            &self.css_normals_b,
            self.config.sensor_use_thresh,
        );
        let h = self.measurements.sensitivity();
        let y = self.measurements.residuals();
        let gain = kalman_gain(&prior.covar, &h, self.config.q_obs_val);

        // Upstream quirk: this tests the covariance from before the time update.
        if max_abs(&self.covar) > self.config.ekf_switch {
            ckf_update(prior, &gain, self.config.q_obs_val, &y, &h)
        } else {
            let (state, posterior) = ekf_update(
                &gain,
                &prior.covar,
                self.config.q_obs_val,
                &y,
                &h,
                &self.state,
            );
            self.state = state;
            posterior
        }
    }

    fn write_outputs(&self) {
        self.nav_state_out_msg.write(NavigationAttitudeMsg {
            time_tag_s: self.time_tag_s,
            vehicle_sun_point_body: normalize_or_zero(heading(&self.state)),
            ..Default::default()
        });

        // The output reports the rates as ω_B/N = -ω_S/B.
        let mut state = SVector::<f64, SKF_N_STATES>::zeros();
        state.fixed_rows_mut::<NUM_STATES>(0).copy_from(&self.state);
        state.fixed_rows_mut::<2>(3).neg_mut();
        let mut state_error = SVector::<f64, SKF_N_STATES>::zeros();
        state_error
            .fixed_rows_mut::<NUM_STATES>(0)
            .copy_from(&self.state_error);
        let mut filter_msg = SunlineFilterMsg {
            time_tag_s: self.time_tag_s,
            state,
            state_error,
            post_fit_res: self.post_fits,
            num_obs: self.measurements.num_obs,
            ..Default::default()
        };
        filter_msg.set_packed_covariance(&self.covar);
        self.filt_data_out_msg.write(filter_msg);
    }
}

impl Module for SunlineSekf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
        self.filt_data_out_msg.write(SunlineFilterMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        SunlineSekf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.run_filter(context.current_sim_nanos);
        self.write_outputs();
    }
}

/// The sun-heading part of a filter state.
fn heading(state: &StateVector) -> Vector3<f64> {
    state.fixed_rows::<3>(0).into_owned()
}

/// `ω_B/N` in body components: the state's `ω_S/B` rates along `ŝ2`, `ŝ3`,
/// negated and rotated by `[BS]`.
fn body_rate(state: &StateVector, dcm_bs: &Matrix3<f64>) -> Vector3<f64> {
    dcm_bs * Vector3::new(0.0, -state[3], -state[4])
}

/// Upstream `vMaxAbs`: start from `|m₀₀|` and take each larger element, so a
/// NaN in (0, 0) propagates and NaNs elsewhere are skipped (unlike `amax`).
/// (0, 0) comes first in either storage order.
fn max_abs(m: &StateMatrix) -> f64 {
    m.iter().skip(1).fold(m[(0, 0)].abs(), |max, value| {
        if value.abs() > max { value.abs() } else { max }
    })
}

/// Upstream `sunlineSEKFComputeDCM_BS`: `[BS]`, whose columns are `ŝ1`, `ŝ2`,
/// `ŝ3` in body components, or the identity when `|d × b̂| < 1e-5`.
fn dcm_bs(sun_heading: Vector3<f64>, b_vec: Vector3<f64>) -> Matrix3<f64> {
    let s2 = sun_heading.cross(&b_vec);
    if s2.norm() < DEGENERATE_FRAME_NORM {
        return Matrix3::identity();
    }
    let s1 = normalize_or_zero(sun_heading);
    let s2 = normalize_or_zero(s2);
    let s3 = normalize_or_zero(sun_heading.cross(&s2));
    Matrix3::from_columns(&[s1, s2, s3])
}

/// Upstream `sunlineDynMatrix`: Jacobian of the state dynamics. Only the
/// heading rows are nonzero.
fn dynamics_matrix(state: &StateVector, b_vec: Vector3<f64>) -> StateMatrix {
    let heading = heading(state);
    let dcm = dcm_bs(heading, b_vec);
    let heading_coupling = -tilde(heading) * dcm;

    let mut dyn_mat = StateMatrix::zeros();
    dyn_mat
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&-tilde(body_rate(state, &dcm)));
    dyn_mat
        .fixed_view_mut::<3, 2>(0, 3)
        .copy_from(&heading_coupling.fixed_view::<3, 2>(0, 1));
    dyn_mat
}

/// Upstream `sunlineStateSTMProp`: one Euler step `ḋ = -ω_B/N × d` of the
/// heading (rates constant), and the first-order STM `I + dt A`.
fn propagate(
    dyn_mat: &StateMatrix,
    b_vec: Vector3<f64>,
    dt: f64,
    state: &StateVector,
) -> (StateVector, StateMatrix) {
    let heading = heading(state);
    let omega_bn_b = body_rate(state, &dcm_bs(heading, b_vec));

    let mut propagated = *state;
    propagated
        .fixed_rows_mut::<3>(0)
        .copy_from(&(heading + -dt * omega_bn_b.cross(&heading)));
    (propagated, StateMatrix::identity() + dyn_mat * dt)
}

/// Process-noise mapping Γ of upstream `sunlineTimeUpdate`: the rate noise
/// enters the rates with `dt` and the heading with `dt²/2 [d̃][BS]`.
fn process_noise_mapping(
    state: &StateVector,
    b_vec: Vector3<f64>,
    dt: f64,
) -> SMatrix<f64, NUM_STATES, 2> {
    let heading = heading(state);
    let heading_block = tilde(heading) * dcm_bs(heading, b_vec) * (dt * dt / 2.0);

    let mut gamma = SMatrix::<f64, NUM_STATES, 2>::zeros();
    gamma
        .fixed_view_mut::<3, 2>(0, 0)
        .copy_from(&heading_block.fixed_view::<3, 2>(0, 1));
    gamma[(3, 0)] = dt;
    gamma[(4, 1)] = dt;
    gamma
}

/// Upstream `sunlineKalmanGain`: `K = P Hᵀ (H P Hᵀ + q_obs I)⁻¹`, with
/// upstream's `mInverse` (zero for a singular, NaN or 1×1 matrix).
fn kalman_gain(covar_bar: &StateMatrix, h: &MeasurementMatrix, q_obs: f64) -> GainMatrix {
    let num_obs = h.nrows();
    let h_t = h.transpose();
    let covar_h_t = covar_bar * &h_t;
    let innovation_covar = h * covar_bar * &h_t + DMatrix::identity(num_obs, num_obs) * q_obs;
    covar_h_t * m_inverse(&innovation_covar).unwrap_or_else(|| DMatrix::zeros(num_obs, num_obs))
}

/// Joseph-form covariance update shared by upstream `sunlineCKFUpdate` and
/// `sunlineSEKFUpdate`: `(I - KH) P (I - KH)ᵀ + K R Kᵀ` with `R = q_obs I`.
fn joseph_covariance(
    gain: &GainMatrix,
    h: &MeasurementMatrix,
    covar_bar: &StateMatrix,
    q_obs: f64,
) -> StateMatrix {
    let num_obs = h.nrows();
    let i_minus_kh = StateMatrix::identity() - gain * h;
    let gain_noise = gain * (DMatrix::identity(num_obs, num_obs) * q_obs);
    i_minus_kh * covar_bar * i_minus_kh.transpose() + gain_noise * gain.transpose()
}

/// Upstream `sunlineCKFUpdate`: classical update of the propagated state
/// error, `x = x̄ + K (y - H x̄)`.
fn ckf_update(
    prior: &Estimate,
    gain: &GainMatrix,
    q_obs: f64,
    y: &DVector<f64>,
    h: &MeasurementMatrix,
) -> Estimate {
    let innovation = y - h * prior.state_error;
    Estimate {
        state_error: prior.state_error + gain * innovation,
        covar: joseph_covariance(gain, h, &prior.covar, q_obs),
    }
}

/// Upstream `sunlineSEKFUpdate`: `x = K y`, added to the reference state.
/// Returns the new reference state and the updated estimate (`x` is kept,
/// as upstream, although the state already contains it).
fn ekf_update(
    gain: &GainMatrix,
    covar_bar: &StateMatrix,
    q_obs: f64,
    y: &DVector<f64>,
    h: &MeasurementMatrix,
    state: &StateVector,
) -> (StateVector, Estimate) {
    let state_error = gain * y;
    (
        state + state_error,
        Estimate {
            state_error,
            covar: joseph_covariance(gain, h, covar_bar, q_obs),
        },
    )
}

/// Upstream `sunlineSEKFSwitch`: build the sun frame on the other body axis
/// (`b̂2` if the current one is `b̂1`, else `b̂1`) and rotate the rate states
/// and the covariance into it. Returns the new axis, state and covariance.
fn switch_frame(
    b_vec: Vector3<f64>,
    state: &StateVector,
    covar: &StateMatrix,
) -> (Vector3<f64>, StateVector, StateMatrix) {
    let heading_hat = normalize_or_zero(heading(state));
    let new_b_vec = if is_equal_within(&b_vec, &Vector3::x(), FRAME_AXIS_TOLERANCE) {
        Vector3::y()
    } else {
        Vector3::x()
    };
    let dcm_new_old = dcm_bs(heading_hat, new_b_vec).transpose() * dcm_bs(heading_hat, b_vec);

    let mut switch = StateMatrix::identity();
    switch
        .fixed_view_mut::<2, 2>(3, 3)
        .copy_from(&dcm_new_old.fixed_view::<2, 2>(1, 1));
    (
        new_b_vec,
        switch * state,
        switch * covar * switch.transpose(),
    )
}

#[cfg(test)]
mod tests {
    // Ports upstream `test_SunLineSEKF.py`: `test_all_functions_sekf`
    // (`sunline_individual_test`, `StatePropStatic`, `StatePropVariable`) and
    // all five parametrizations of `test_all_sunline_sekf`
    // (`StateUpdateSunLine`), with upstream's truth and tolerances.
    //
    // As described under "Measurement write time" in the module docs,
    // upstream's never-written CSS message is an unconnected input here, and
    // each reading carries upstream's write time (`CurrentNanos` when the
    // pytest writes it) in `time_tag_s`. Upstream's numpy noise stream
    // (`default_rng(42)`) is replaced by a seeded `StdRng`, so the noisy
    // cases draw different (equally distributed) noise. The plots of
    // `SunLineSEKF_test_utilities.py` are not ported.

    use std::f64::consts::FRAC_1_SQRT_2;

    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix2, Matrix3, SMatrix, SVector, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, Normal};

    use crate::messages::{
        CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, MAX_N_CSS_MEAS, Output, SunlineFilterMsg,
    };
    use crate::utilities::rigid_body_kinematics::tilde;
    use crate::{Module, SimulationContext};

    use super::{
        CssMeasurements, Estimate, GainMatrix, MeasurementMatrix, StateMatrix, StateVector,
        SunlineSekf, SunlineSekfConfig, ckf_update, dcm_bs, dynamics_matrix, ekf_update, heading,
        kalman_gain, propagate, switch_frame,
    };

    const TASK_NANOS: u64 = 500_000_000;
    const DT: f64 = 0.5;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn state(values: [f64; 5]) -> StateVector {
        StateVector::from_row_slice(&values)
    }

    /// Upstream `setupFilterData`.
    fn upstream_filter_config() -> SunlineSekfConfig {
        SunlineSekfConfig {
            sensor_use_thresh: 0.0,
            initial_state: state([0.1, 0.9, 0.1, 0.0, 0.0]),
            initial_state_error: state([1.0, 0.0, 1.0, 0.0, 0.1]),
            initial_covar: StateMatrix::from_diagonal(&state([0.4, 0.4, 0.4, 0.004, 0.004])),
            q_proc_val: 0.1_f64.powi(2),
            q_obs_val: 0.001,
            ekf_switch: (4.0_f64 / 3.0).powi(2),
            ..Default::default()
        }
    }

    // Inputs shared by the `sunline_individual_test` sections.
    const INPUT_STATES: [f64; 5] = [2.0, 1.0, 0.75, 0.1, 0.4];
    const UPDATE_COVAR: [f64; 25] = [
        1., 0., 0., 1., 0., //
        0., 1., 0., 0., 1., //
        0., 0., 1., 0., 0., //
        1., 0., 0., 1., 0., //
        0., 1., 0., 0., 1.,
    ];
    const UPDATE_GAIN: [f64; 15] = [1., 2., 3., 0., 1., 1., 0., 1., 0., 1., 3., 0., 1., 0., 2.];
    const UPDATE_NOISE: f64 = 0.01;

    fn css_cos() -> [f64; 4] {
        [10.0_f64, 25.0, 5.0, 90.0].map(|deg| deg.to_radians().cos())
    }

    fn css_normals() -> [Vector3<f64>; 4] {
        let half_sqrt2 = 1.0 / 2.0_f64.sqrt();
        [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::new(half_sqrt2, half_sqrt2, 0.0),
        ]
    }

    /// The first three rows of upstream's 8-row `h` buffer: `[I₃ 0]`.
    fn identity_sensitivity() -> MeasurementMatrix {
        MeasurementMatrix::from_row_slice(&[
            1., 0., 0., 0., 0., //
            0., 1., 0., 0., 0., //
            0., 0., 1., 0., 0.,
        ])
    }

    /// Upstream's `inputY`: `cos_j - d · n_j` for the first three sensors.
    fn input_y(states: &StateVector) -> DVector<f64> {
        let d = heading(states);
        DVector::from_iterator(
            3,
            css_cos()
                .iter()
                .zip(css_normals())
                .take(3)
                .map(|(cos, normal)| cos - d.dot(&normal)),
        )
    }

    /// The numpy construction of `[BS]` in upstream's truth code.
    fn numpy_dcm_bs(heading: Vector3<f64>, b_vec: Vector3<f64>) -> Matrix3<f64> {
        let s1 = heading / heading.norm();
        let s2 = s1.cross(&b_vec) / s1.cross(&b_vec).norm();
        let s3 = s1.cross(&s2) / s1.cross(&s2).norm();
        Matrix3::from_columns(&[s1, s2, s3])
    }

    /// Upstream's `expDynMat`: `[ω_SB_B~]` and `-[d~][BS]` (last two columns).
    fn expected_dynamics_matrix(
        heading: Vector3<f64>,
        omega_sb_b: Vector3<f64>,
        dcm_bs: &Matrix3<f64>,
    ) -> StateMatrix {
        let d_bs = tilde(heading) * dcm_bs;
        let mut expected = StateMatrix::zeros();
        expected
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&tilde(omega_sb_b));
        expected
            .fixed_view_mut::<3, 2>(0, 3)
            .copy_from(&-d_bs.fixed_view::<3, 2>(0, 1));
        expected
    }

    /// Upstream's Joseph-form truth `(I - KH) P (I - KH)ᵀ + K R Kᵀ`.
    fn expected_joseph(
        gain: &GainMatrix,
        h: &MeasurementMatrix,
        covar: &StateMatrix,
        noise: f64,
    ) -> StateMatrix {
        let i_kh = StateMatrix::identity() - gain * h;
        let r = DMatrix::<f64>::identity(h.nrows(), h.nrows()) * noise;
        i_kh * covar * i_kh.transpose() + gain * (r * gain.transpose())
    }

    fn update_gain() -> GainMatrix {
        GainMatrix::from_row_slice(&UPDATE_GAIN)
    }

    fn history_error<T>(logged: &[T], truth: &[T], difference: impl Fn(&T, &T) -> f64) -> f64 {
        logged
            .iter()
            .zip(truth)
            .map(|(log, truth)| difference(log, truth).powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// `sunline_individual_test`: "Testing dynamics matrix computation".
    #[test]
    fn matches_upstream_dynamics_matrix_truth() {
        let input_states = state(INPUT_STATES);
        let b_vec = Vector3::x();
        // Upstream's truth takes [BS] from the module's own function; the
        // numpy construction is its independent equivalent.
        let dcm = numpy_dcm_bs(heading(&input_states), b_vec);
        let omega_sb_b = dcm * Vector3::new(0.0, 0.1, 0.4);
        let expected = expected_dynamics_matrix(heading(&input_states), omega_sb_b, &dcm);

        let dyn_mat = dynamics_matrix(&input_states, b_vec);
        assert!((expected - dyn_mat).norm() <= 1.0e-10, "{dyn_mat}");
    }

    /// `sunline_individual_test`: "STM and State Test".
    #[test]
    fn matches_upstream_state_and_stm_propagation_truth() {
        let input_states = state(INPUT_STATES);
        let input_omega = Vector3::new(0.0, 0.1, 0.4);
        let b_vec = Vector3::x();
        let dt = 0.5;
        let dcm = numpy_dcm_bs(heading(&input_states), b_vec);
        let exp_dyn_mat = expected_dynamics_matrix(heading(&input_states), dcm * input_omega, &dcm);

        let (states_out, stm_out) = propagate(&exp_dyn_mat, b_vec, dt, &input_states);

        let expected_stm = dt * exp_dyn_mat * StateMatrix::identity() + StateMatrix::identity();
        let d = heading(&input_states);
        let d_expected = d + dt * (dcm * input_omega).cross(&d);
        let expected_states = state([d_expected.x, d_expected.y, d_expected.z, 0.1, 0.4]);
        assert!((expected_stm - stm_out).norm() <= 1.0e-10, "{stm_out}");
        assert!(
            (expected_states - states_out).norm() <= 1.0e-10,
            "{states_out}"
        );
    }

    /// `sunline_individual_test`: "Test the H and yMeas matrix generation as
    /// well as the observation count".
    #[test]
    fn matches_upstream_measurement_model_truth() {
        let input_states = state(INPUT_STATES);
        let mut measurements = CssMeasurements {
            num_obs: 0,
            ..CssMeasurements::cleared()
        };
        measurements.record(
            &input_states,
            &css_cos(),
            &css_normals(),
            50.0_f64.to_radians().cos(),
        );

        let mut expected_h = SMatrix::<f64, MAX_N_CSS_MEAS, 5>::zeros();
        expected_h
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&Matrix3::identity());
        let mut expected_y = SVector::<f64, MAX_N_CSS_MEAS>::zeros();
        expected_y
            .fixed_rows_mut::<3>(0)
            .copy_from(&input_y(&input_states));
        let mut expected_obs = SVector::<f64, MAX_N_CSS_MEAS>::zeros();
        expected_obs
            .fixed_rows_mut::<3>(0)
            .copy_from_slice(&css_cos()[..3]);

        assert!((measurements.meas_mat - expected_h).norm() <= 1.0e-10);
        assert!((measurements.y_meas - expected_y).norm() <= 1.0e-10);
        assert!((measurements.obs - expected_obs).norm() <= 1.0e-10);
        assert_eq!(measurements.num_obs, 3);
    }

    /// `sunline_individual_test`: "Test the Kalman Gain".
    #[test]
    fn matches_upstream_kalman_gain_truth() {
        let h = identity_sensitivity();
        let covar = StateMatrix::from_row_slice(&UPDATE_COVAR);
        let r = DMatrix::<f64>::identity(3, 3) * UPDATE_NOISE;
        let innovation_inverse = (&h * covar * h.transpose() + r)
            .try_inverse()
            .expect("the innovation covariance is invertible");
        let expected = covar * h.transpose() * innovation_inverse;

        let gain = kalman_gain(&covar, &h, UPDATE_NOISE);
        assert!((&gain - expected).norm() <= 1.0e-10, "{gain}");
    }

    /// `sunline_individual_test`: "Test the EKF update".
    #[test]
    fn matches_upstream_ekf_update_truth() {
        let input_states = state(INPUT_STATES);
        let gain = update_gain();
        let h = identity_sensitivity();
        let covar = StateMatrix::from_row_slice(&UPDATE_COVAR);
        let y = input_y(&input_states);

        let (states_out, posterior) =
            ekf_update(&gain, &covar, UPDATE_NOISE, &y, &h, &input_states);

        let expected_states = input_states + &gain * &y;
        let expected_covar = expected_joseph(&gain, &h, &covar, UPDATE_NOISE);
        assert!((states_out - expected_states).norm() <= 1.0e-10);
        assert!((posterior.covar - expected_covar).norm() <= 1.0e-10);
    }

    /// `sunline_individual_test`: "Test the CKF update".
    #[test]
    fn matches_upstream_ckf_update_truth() {
        let input_states = state(INPUT_STATES);
        let x_bar = state([0.1, 0.2, 0.01, 0.005, 0.009]);
        let gain = update_gain();
        let h = identity_sensitivity();
        let covar = StateMatrix::from_row_slice(&UPDATE_COVAR);
        let y = input_y(&input_states);
        let prior = Estimate {
            state_error: x_bar,
            covar,
        };

        let posterior = ckf_update(&prior, &gain, UPDATE_NOISE, &y, &h);

        let expected_state_error = x_bar + &gain * (&y - &h * x_bar);
        let expected_covar = expected_joseph(&gain, &h, &covar, UPDATE_NOISE);
        assert!((posterior.state_error - expected_state_error).norm() <= 1.0e-10);
        assert!((posterior.covar - expected_covar).norm() <= 1.0e-10);
    }

    /// `sunline_individual_test`: "Test the sunlineSEKFComputeDCM_BS method".
    #[test]
    fn matches_upstream_sun_frame_dcm_truth() {
        let sun_heading = Vector3::new(2.0, 1.0, 0.75);
        let b_vec = Vector3::y();
        let expected = numpy_dcm_bs(sun_heading, b_vec);
        assert!((dcm_bs(sun_heading, b_vec) - expected).norm() <= 1.0e-10);
    }

    /// `sunline_individual_test`: "Test the Switching method".
    #[test]
    fn matches_upstream_frame_switch_truth() {
        let input_states = state(INPUT_STATES);
        let b_old = Vector3::y();
        let b_new = Vector3::x();
        let covar = StateMatrix::from_row_slice(&UPDATE_COVAR);

        let d = heading(&input_states);
        let dcm_new_old = numpy_dcm_bs(d, b_new).transpose() * numpy_dcm_bs(d, b_old);
        let mut switch = StateMatrix::identity();
        switch
            .fixed_view_mut::<2, 2>(3, 3)
            .copy_from(&dcm_new_old.fixed_view::<2, 2>(1, 1));

        let (b_vec_out, states_out, covar_out) = switch_frame(b_old, &input_states, &covar);
        assert!((states_out - switch * input_states).norm() <= 1.0e-10);
        assert!((covar_out - switch * covar * switch.transpose()).norm() <= 1.0e-10);
        assert!((b_vec_out - b_new).norm() <= 1.0e-10);
    }

    /// Upstream `StatePropStatic`: with zero rates and no CSS data the state
    /// stays put over 8000 s.
    #[test]
    fn matches_upstream_static_state_propagation() {
        let mut module = SunlineSekf::new(upstream_filter_config());
        // Upstream connects a never-written CSS configuration (no sensors).
        let config_output = Output::new(CssConfigMsg::default());
        config_output.connect_to(&mut module.css_config_in_msg);
        module.init();

        module.update(&context(0));
        let first = module.state;
        for tick in 1..=16_000 {
            module.update(&context(tick * TASK_NANOS));
        }
        for (last, first) in module.state.iter().zip(first.iter()) {
            assert!((last - first).abs() <= 1.0e-10, "{}", module.state);
        }
    }

    /// Upstream `StatePropVariable`: 1000 s of pure propagation with nonzero
    /// rates, against upstream's step-by-step truth.
    #[test]
    fn matches_upstream_variable_state_propagation() {
        const SAMPLES: usize = 2001;
        let mut config = upstream_filter_config();
        config.initial_state += state([0.0, 0.0, 0.0, 0.0001, 0.002]);
        let q_proc = config.q_proc_val;
        let (initial_state, initial_x, initial_covar) = (
            config.initial_state,
            config.initial_state_error,
            config.initial_covar,
        );

        let mut module = SunlineSekf::new(config);
        let config_output = Output::new(CssConfigMsg::default());
        config_output.connect_to(&mut module.css_config_in_msg);
        module.init();
        let (mut states, mut x_log, mut covar_log, mut stm_log) =
            (Vec::new(), Vec::new(), Vec::new(), Vec::new());
        for tick in 0..SAMPLES as u64 {
            module.update(&context(tick * TASK_NANOS));
            states.push(module.state);
            x_log.push(module.state_error);
            covar_log.push(module.covar);
            stm_log.push(module.state_transition);
        }

        // Upstream truth, with b = b1 throughout. Upstream builds [BS] of the
        // first sample with numpy and the others with the module's function;
        // here all use the independent numpy construction.
        let b_vec = Vector3::x();
        let omega_b = |s: &StateVector, dcm: &Matrix3<f64>| dcm * Vector3::new(0.0, s[3], s[4]);
        let mut expected_states = vec![initial_state];
        let mut dcms = vec![numpy_dcm_bs(heading(&initial_state), b_vec)];
        for i in 1..SAMPLES {
            let previous = expected_states[i - 1];
            let d = heading(&previous);
            let d_next = d + DT * omega_b(&previous, &dcms[i - 1]).cross(&d);
            dcms.push(numpy_dcm_bs(d_next, b_vec));
            expected_states.push(state([
                d_next.x,
                d_next.y,
                d_next.z,
                previous[3],
                previous[4],
            ]));
        }
        let dyn_mats: Vec<StateMatrix> = expected_states
            .iter()
            .zip(&dcms)
            .map(|(s, dcm)| expected_dynamics_matrix(heading(s), omega_b(s, dcm), dcm))
            .collect();
        let mut expected_stms = vec![StateMatrix::identity()];
        let mut expected_x_bars = vec![initial_x];
        let mut expected_covars = vec![initial_covar];
        for i in 1..SAMPLES {
            let stm = DT * dyn_mats[i - 1] * StateMatrix::identity() + StateMatrix::identity();
            let s_bs = tilde(heading(&expected_states[i])) * dcms[i];
            let mut gamma = SMatrix::<f64, 5, 2>::zeros();
            gamma
                .fixed_view_mut::<3, 2>(0, 0)
                .copy_from(&(DT.powi(2) / 2.0 * s_bs.fixed_view::<3, 2>(0, 1)));
            gamma
                .fixed_view_mut::<2, 2>(3, 0)
                .copy_from(&(DT * Matrix2::identity()));
            let proc_noise = gamma * (q_proc * Matrix2::identity() * gamma.transpose());
            expected_x_bars.push(stm * expected_x_bars[i - 1]);
            expected_covars.push(stm * (expected_covars[i - 1] * stm.transpose()) + proc_noise);
            expected_stms.push(stm);
        }

        let vector_error = |a: &StateVector, b: &StateVector| (a - b).norm();
        let matrix_error = |a: &StateMatrix, b: &StateMatrix| (a - b).norm();
        let state_error = history_error(&states, &expected_states, vector_error);
        let x_error = history_error(&x_log, &expected_x_bars, vector_error);
        let covar_error = history_error(&covar_log, &expected_covars, matrix_error);
        let stm_error = history_error(&stm_log, &expected_stms, matrix_error);
        assert!(state_error <= 1.0e-10, "state propagation {state_error}");
        assert!(x_error <= 1.0e-4, "state error propagation {x_error}");
        assert!(
            covar_error <= 1.0e-4,
            "covariance propagation {covar_error}"
        );
        assert!(stm_error <= 1.0e-4, "STM propagation {stm_error}");
    }

    struct StateUpdateCase {
        sim_half_length: u64,
        add_meas_noise: bool,
        test_vector1: [f64; 3],
        test_vector2: [f64; 3],
        state_guess: [f64; 5],
    }

    /// Upstream's `test_all_sunline_sekf` parametrization.
    const STATE_UPDATE_CASES: [StateUpdateCase; 5] = [
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 2000,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: false,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: false,
            test_vector1: [0., 1., 0.],
            test_vector2: [1., 0., 0.],
            state_guess: [0.3, 0.0, 0.6, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [0.5, 0.5, 0.],
            test_vector2: [0., 1., 0.],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0],
        },
    ];

    // Upstream's `CSSOrientationList`, bit for bit: its 0.70710678118654757
    // is `FRAC_1_SQRT_2`, and 0.70710678118654746 is one ulp below it.
    const CSS_ORIENTATIONS: [[f64; 3]; 8] = [
        [0.7071067811865475, -0.5, 0.5],
        [0.7071067811865475, -0.5, -0.5],
        [0.7071067811865475, 0.5, -0.5],
        [0.7071067811865475, 0.5, 0.5],
        [-0.7071067811865475, 0.0, FRAC_1_SQRT_2],
        [-0.7071067811865475, FRAC_1_SQRT_2, 0.0],
        [-0.7071067811865475, 0.0, -FRAC_1_SQRT_2],
        [-0.7071067811865475, -FRAC_1_SQRT_2, 0.0],
    ];

    /// Runs one tick, records the filter message and applies upstream's
    /// issue-#1353 check: post-fit residuals equal `y_meas - H x`.
    fn step_and_check_post_fits(
        module: &mut SunlineSekf,
        tick: u64,
        filter_log: &mut Vec<SunlineFilterMsg>,
    ) {
        module.update(&context(tick * TASK_NANOS));
        filter_log.push(module.filt_data_out_msg.read());

        let num_obs = module.measurements.num_obs;
        if num_obs > 0 {
            let h = module.measurements.meas_mat.rows(0, num_obs);
            let expected = module.measurements.y_meas.rows(0, num_obs) - h * module.state_error;
            let residual_error = (module.post_fits.rows(0, num_obs) - expected).norm();
            assert!(
                residual_error <= 1.0e-9,
                "post-fit residual mismatch at tick {tick}"
            );
        }
    }

    fn assert_converged(
        first: &SunlineFilterMsg,
        last: &SunlineFilterMsg,
        target: [f64; 3],
        phase: &str,
    ) {
        let initial_covar = first.packed_covariance::<5>();
        let final_covar = last.packed_covariance::<5>();
        let target = [target[0], target[1], target[2], 0.0, 0.0];
        for (i, target) in target.iter().enumerate() {
            let covar_error = (final_covar[(i, i)] - initial_covar[(i, i)] / 100.0).abs();
            assert!(
                covar_error <= 1.0e-1,
                "covariance update failure {phase}: {i}"
            );
            let state_error = (last.state[i] - target).abs();
            assert!(state_error <= 1.0e-1, "state update failure {phase}: {i}");
        }
    }

    /// Upstream `StateUpdateSunLine`, every parametrization: converge on one
    /// sun heading, then on a second one, with or without CSS noise.
    #[test]
    fn matches_upstream_state_update_sun_line() {
        let units: Vec<CssUnitConfigMsg> = CSS_ORIENTATIONS
            .iter()
            .map(|&n_hat| CssUnitConfigMsg {
                n_hat_b: Vector3::from(n_hat),
                c_bias: 1.0,
            })
            .collect();

        for case in &STATE_UPDATE_CASES {
            let mut config = upstream_filter_config();
            let [x1, y1, z1] = case.test_vector1;
            config.initial_state = state(case.state_guess);
            config.initial_state_error = state([x1, y1, z1, 0.0, 0.0]) - config.initial_state;
            let noise = Normal::new(0.0, config.q_obs_val.sqrt()).expect("valid noise sigma");

            let config_output = Output::new(CssConfigMsg::from_active(&units));
            let css_output = Output::new(CssArraySensorMsg::default());
            let mut module = SunlineSekf::new(config);
            config_output.connect_to(&mut module.css_config_in_msg);
            module.init();

            let mut rng = StdRng::seed_from_u64(42);
            // Upstream writes between ticks, at the time of the last tick run.
            let mut write_readings = |sun: [f64; 3], last_tick: u64| {
                let sun = Vector3::from(sun);
                let cos_values: Vec<f64> = CSS_ORIENTATIONS
                    .iter()
                    .map(|&n_hat| {
                        let cos = Vector3::from(n_hat).dot(&sun);
                        if case.add_meas_noise {
                            cos + noise.sample(&mut rng)
                        } else {
                            cos
                        }
                    })
                    .collect();
                css_output.write(CssArraySensorMsg {
                    time_tag_s: (last_tick * TASK_NANOS) as f64 * 1.0e-9,
                    ..CssArraySensorMsg::from_active(&cos_values)
                });
            };

            let mut filter_log = Vec::new();
            step_and_check_post_fits(&mut module, 0, &mut filter_log);
            for i in 0..case.sim_half_length {
                if i > 20 {
                    write_readings(case.test_vector1, i);
                    // The first write: from now on upstream's message counts
                    // as written.
                    css_output.connect_to(&mut module.css_data_in_msg);
                }
                step_and_check_post_fits(&mut module, i + 1, &mut filter_log);
            }
            assert_converged(
                &filter_log[0],
                filter_log.last().expect("ticks were run"),
                case.test_vector1,
                "after the first half",
            );

            for i in 0..case.sim_half_length {
                let tick = i + case.sim_half_length;
                if i > 20 {
                    write_readings(case.test_vector2, tick);
                }
                step_and_check_post_fits(&mut module, tick + 1, &mut filter_log);
            }
            assert_converged(
                &filter_log[0],
                filter_log.last().expect("ticks were run"),
                case.test_vector2,
                "at the end",
            );
        }
    }

    // The tests below are not upstream.

    fn css_config(normals: &[[f64; 3]]) -> CssConfigMsg {
        let units: Vec<CssUnitConfigMsg> = normals
            .iter()
            .map(|&n_hat| CssUnitConfigMsg {
                n_hat_b: Vector3::from(n_hat),
                c_bias: 1.0,
            })
            .collect();
        CssConfigMsg::from_active(&units)
    }

    /// Noise-free readings `n̂ · sun`, plus `√q sin(0.37 step + 1.1 k)` for
    /// sensor `k` when `q > 0`, stamped with their write time.
    fn deterministic_readings(
        normals: &[[f64; 3]],
        sun: [f64; 3],
        step: u64,
        q: f64,
        write_nanos: u64,
    ) -> CssArraySensorMsg {
        let cos_values: Vec<f64> = normals
            .iter()
            .enumerate()
            .map(|(k, &n_hat)| {
                let noise = if q > 0.0 {
                    q.sqrt() * (0.37 * step as f64 + 1.1 * k as f64).sin()
                } else {
                    0.0
                };
                Vector3::from(n_hat).dot(&Vector3::from(sun)) + noise
            })
            .collect();
        CssArraySensorMsg {
            time_tag_s: write_nanos as f64 * 1.0e-9,
            ..CssArraySensorMsg::from_active(&cos_values)
        }
    }

    fn assert_slice_close(actual: &[f64], expected: &[f64], accuracy: f64, what: &str) {
        for (a, e) in actual.iter().zip(expected) {
            assert!(
                (a - e).abs() <= accuracy,
                "{what}: {actual:?} != {expected:?}"
            );
        }
    }

    // Regression values from upstream sunlineSEKF.c compiled with gcc
    // (-ffp-contract=off) and driven through the same scenario, with the
    // producer writing in the same tick. The port matched it bit for bit on
    // every tick, apart from the sign of some zeros. The sun rotates past b̂1
    // and b̂2 (frame switches), the CKF runs while the covariance exceeds
    // `ekf_switch`, and the filter is reset halfway.
    #[test]
    fn matches_compiled_upstream_on_a_switching_and_resetting_run() {
        let mut initial_covar = StateMatrix::from_diagonal(&state([0.4, 0.4, 0.4, 0.004, 0.004]));
        initial_covar[(0, 3)] = 0.01;
        initial_covar[(3, 0)] = 0.01;
        let mut module = SunlineSekf::new(SunlineSekfConfig {
            sensor_use_thresh: 0.1,
            initial_state: state([0.95, 0.1, 0.05, 0.01, -0.02]),
            initial_state_error: state([0.01, -0.02, 0.03, 0.001, 0.002]),
            initial_covar,
            q_proc_val: 0.0001,
            q_obs_val: 0.002,
            ekf_switch: 0.3,
            ..Default::default()
        });
        let config_output = Output::new(css_config(&CSS_ORIENTATIONS));
        let css_output = Output::new(CssArraySensorMsg::default());
        config_output.connect_to(&mut module.css_config_in_msg);
        css_output.connect_to(&mut module.css_data_in_msg);
        module.init();

        let mut frame_axes = Vec::new();
        for tick in 0..=700 {
            let t = tick * TASK_NANOS;
            let angle = 0.005 * tick as f64;
            let sun = [angle.cos(), angle.sin(), 0.3 * (0.5 * angle).sin()];
            css_output.write(deterministic_readings(
                &CSS_ORIENTATIONS,
                sun,
                tick,
                0.002,
                t,
            ));
            if tick == 350 {
                module.reset(t);
            }
            module.update(&context(t));
            frame_axes.push(module.b_vec_b);
        }

        // The heading starts near b̂1, so the first update already switches.
        assert_eq!(frame_axes[0], Vector3::y());
        assert!(frame_axes.contains(&Vector3::x()));
        assert_slice_close(
            module.state.as_slice(),
            &[
                -0.9774129694193424,
                -0.4086096298139836,
                0.26997251164878505,
                -0.013000812241187463,
                0.015608351243125249,
            ],
            1.0e-12,
            "state",
        );
        assert_slice_close(
            module.state_error.as_slice(),
            &[
                -0.006747017902629347,
                0.01631450320364259,
                -0.000152766806112389,
                0.005532031807899786,
                0.000913459209080379,
            ],
            1.0e-12,
            "state error",
        );
        assert_slice_close(
            module.covar.diagonal().as_slice(),
            &[
                0.00012043291637100894,
                0.00047270693877113004,
                0.0005215831472985475,
                0.00015436538020695598,
                0.00015547236911241322,
            ],
            1.0e-12,
            "covariance diagonal",
        );
        let filter_msg = module.filt_data_out_msg.read();
        assert_eq!(filter_msg.num_obs, 4);
        assert_eq!(filter_msg.time_tag_s, 350.0);
    }

    // Two sensors read two, then one, then no and again two valid readings.
    // With one observation upstream's 1×1 `mInverse` gives a zero gain, so the
    // state follows pure propagation; with none the residuals keep their stale
    // values. Regression values from the compiled upstream C, as above.
    #[test]
    fn matches_compiled_upstream_with_one_and_zero_observations() {
        const NORMALS: [[f64; 3]; 2] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
        let mut module = SunlineSekf::new(SunlineSekfConfig {
            sensor_use_thresh: 0.5,
            initial_state: state([0.6, 0.6, 0.2, 0.001, 0.0]),
            initial_covar: StateMatrix::from_diagonal(&state([0.4, 0.4, 0.4, 0.004, 0.004])),
            q_proc_val: 1e-4,
            q_obs_val: 1e-3,
            ekf_switch: 10.0,
            ..Default::default()
        });
        let config_output = Output::new(css_config(&NORMALS));
        let css_output = Output::new(CssArraySensorMsg::default());
        config_output.connect_to(&mut module.css_config_in_msg);
        css_output.connect_to(&mut module.css_data_in_msg);
        module.init();

        let mut filter_log = Vec::new();
        for tick in 0..=40 {
            let t = tick * TASK_NANOS;
            let sun = match tick {
                0..10 => [0.8, 0.6, 0.0],
                10..20 => [0.9, 0.3, 0.3],
                20..30 => [0.2, 0.2, 0.96],
                _ => [0.7, 0.7, 0.1],
            };
            css_output.write(deterministic_readings(&NORMALS, sun, tick, 0.0, t));
            module.update(&context(t));
            if (10..20).contains(&tick) {
                assert_eq!(module.state_error, StateVector::zeros());
            }
            filter_log.push(module.filt_data_out_msg.read());
        }

        assert_eq!(filter_log[19].num_obs, 1);
        assert_slice_close(
            &filter_log[19].state.as_slice()[..3],
            &[0.8003477732430991, 0.6000061843212636, 0.19786409989983128],
            1.0e-12,
            "state after the single-observation phase",
        );
        assert_eq!(filter_log[25].num_obs, 0);
        assert_slice_close(
            &filter_log[25].post_fit_res.as_slice()[..3],
            &[0.09965222675690089, 6.642561612274645e-06, 0.0],
            1.0e-12,
            "stale post-fit residuals",
        );
        assert_slice_close(
            module.state.as_slice(),
            &[
                0.6956649864519454,
                0.7011900986862432,
                0.22851389355266705,
                -0.0022852944660555685,
                -0.00460535284920225,
            ],
            1.0e-12,
            "final state",
        );
    }

    #[test]
    fn single_observation_gives_a_zero_kalman_gain() {
        let h = MeasurementMatrix::from_row_slice(&[1.0, 0.0, 0.0, 0.0, 0.0]);
        let covar = StateMatrix::from_row_slice(&UPDATE_COVAR);
        assert_eq!(kalman_gain(&covar, &h, UPDATE_NOISE), GainMatrix::zeros(1));
    }

    // A NaN observation noise makes the innovation covariance's determinant
    // NaN. Upstream `mInverse` only inverts when `fabs(det) > DB0_EPS`, so it
    // returns zeros (checked against the compiled C) and the gain is zero,
    // not NaN.
    #[test]
    fn nan_innovation_determinant_gives_a_zero_kalman_gain() {
        let covar = StateMatrix::from_row_slice(&UPDATE_COVAR);
        let gain = kalman_gain(&covar, &identity_sensitivity(), f64::NAN);
        assert_eq!(gain, GainMatrix::zeros(3));
    }

    #[test]
    fn reset_keeps_the_estimate_and_reinitializes_the_bookkeeping() {
        let mut module = SunlineSekf::new(upstream_filter_config());
        let config_output = Output::new(css_config(&CSS_ORIENTATIONS));
        let css_output = Output::new(CssArraySensorMsg::default());
        config_output.connect_to(&mut module.css_config_in_msg);
        css_output.connect_to(&mut module.css_data_in_msg);
        module.init();
        for tick in 0..10 {
            let t = tick * TASK_NANOS;
            let sun = [0.6, 0.0, 0.8];
            css_output.write(deterministic_readings(&CSS_ORIENTATIONS, sun, tick, 0.0, t));
            module.update(&context(t));
        }
        assert!(module.measurements.num_obs < MAX_N_CSS_MEAS);
        let (state, state_error, covar) = (module.state, module.state_error, module.covar);

        let reset_nanos = 10 * TASK_NANOS;
        Module::reset(&mut module, &context(reset_nanos));
        assert_eq!(module.state, state);
        assert_eq!(module.state_error, state_error);
        assert_eq!(module.covar, covar);
        assert_eq!(module.measurements, CssMeasurements::cleared());
        assert_eq!(module.state_transition, StateMatrix::identity());
        assert_eq!(module.b_vec_b, Vector3::x());

        // The stale reading predates the reset and the call time equals it:
        // neither a measurement nor a time update runs.
        module.update(&context(reset_nanos));
        let filter_msg = module.filt_data_out_msg.read();
        assert_eq!(filter_msg.time_tag_s, 5.0);
        assert_eq!(filter_msg.num_obs, MAX_N_CSS_MEAS);
        assert_eq!(
            filter_msg.post_fit_res,
            SVector::<f64, MAX_N_CSS_MEAS>::zeros()
        );
        assert_eq!(module.state, state);
    }

    #[test]
    fn output_messages_report_body_rates_and_the_packed_covariance() {
        let mut config = upstream_filter_config();
        config.initial_state = state([0.0, 3.0, 4.0, 0.25, -0.5]);
        config.initial_covar = StateMatrix::from_fn(|r, c| (5 * r + c) as f64);
        let mut module = SunlineSekf::new(config);
        Module::init(&mut module);
        module.update(&context(0));

        let nav = module.nav_state_out_msg.read();
        // v3Normalize scales by 1/|d|, so 3 * 0.2 rounds to 0.6000000000000001.
        assert_eq!(
            nav.vehicle_sun_point_body,
            Vector3::new(0.0, 3.0, 4.0) * (1.0 / 5.0)
        );
        assert_eq!(nav.time_tag_s, 0.0);
        assert_eq!(nav.sigma_bn, Vector3::zeros());
        assert_eq!(nav.omega_bn_b_radps, Vector3::zeros());

        let filter_msg = module.filt_data_out_msg.read();
        assert_eq!(
            filter_msg.state.as_slice(),
            &[0.0, 3.0, 4.0, -0.25, 0.5, 0.0]
        );
        assert_eq!(filter_msg.packed_covariance::<5>(), module.covar);
        assert_eq!(filter_msg.covar[(4, 0)], 24.0);
        assert_eq!(filter_msg.covar[(4, 1)], 0.0);
    }
}
