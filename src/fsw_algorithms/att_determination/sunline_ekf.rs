//! Sunline extended Kalman filter: estimates the sun heading `d` and its rate
//! `ḋ` in the body frame from the coarse-sun-sensor (CSS) constellation
//! readings. Port of upstream Basilisk `sunlineEKF`
//! (`fswAlgorithms/attDetermination/sunlineEKF`).
//!
//! The six states are the heading and its derivative. Sun sensors cannot see a
//! rotation about the sunline, so every propagation first removes the rate
//! component along the heading. A measurement update uses the readings above
//! `sensor_use_thresh`; it is an extended update (the correction moves the
//! reference state) unless the largest covariance magnitude exceeds
//! `ekf_switch`, in which case a classical update (only the state error moves)
//! brings the covariance down first.
//!
//! # Measurement write time
//!
//! Upstream decides whether a CSS reading is new from the message header: it
//! is processed when it has been written (`isWritten`) no earlier than the
//! filter time (`timeWritten`). The per-sensor [`SunSensorMsg`] inputs carry
//! no header and no time tag, so the readings count as written at the current
//! update when every one of `css_data_in_msgs` is connected — what upstream
//! sees when the CSS producers run before the filter on every tick — and an
//! unconnected input behaves like upstream's never-written message. The
//! header-driven logic itself is unchanged, with the write time as an
//! explicit argument, and the tests drive it with upstream's exact header
//! values.
//!
//! The measurement-model, gain and update helpers are shared with
//! [`super::okeefe_ekf`], whose upstream code is the same algorithm on three
//! states.

use nalgebra::{
    Const, DMatrix, DVector, Dyn, Matrix3, Matrix6, OMatrix, SMatrix, SVector, Vector3, Vector6,
};

use super::CssUnit;
use super::css_inputs::{
    check_css_inputs, configured_css_units, css_inputs, css_readings, css_written_nanos,
};
use crate::messages::{
    Input, MAX_N_CSS_MEAS, NavigationAttitudeMsg, Output, SKF_N_STATES, SunSensorMsg,
    SunlineFilterMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::{m_inverse, normalize_or_zero, sequential_product};
use crate::{Module, SimulationContext};

/// Time step \[s\] at or below which upstream leaves the rate rows of the
/// dynamics matrix as they were.
const MIN_RATE_PROPAGATION_DT_S: f64 = 1.0e-10;

/// User-settable parameters of [`SunlineEkf`]. Upstream zero-initializes all
/// of them.
#[derive(Clone, Debug)]
pub struct SunlineEkfConfig {
    /// Module instance name.
    pub name: String,
    /// Upstream `qObsVal` \[-\]: variance of one CSS reading.
    pub q_obs_val: f64,
    /// Upstream `qProcVal` \[-\]: process-noise variance. Read at Reset.
    pub q_proc_val: f64,
    /// Upstream `sensorUseThresh` \[-\]: readings at or below it are discarded.
    pub sensor_use_thresh: f64,
    /// Upstream `eKFSwitch` \[-\]: while the largest magnitude in the covariance
    /// carried into a step (before its propagation) exceeds it, measurement
    /// updates use the classical instead of the extended form.
    pub ekf_switch: f64,
    /// Upstream `cssConfigInMsg`: the CSS normals and biases, read at Reset,
    /// one entry per input of [`SunlineEkf::css_data_in_msgs`] (which
    /// [`SunlineEkf::new`] creates from it). Upstream uses at most
    /// `MAX_N_CSS_MEAS` sensors.
    pub css_units: Vec<CssUnit>,
}

impl Default for SunlineEkfConfig {
    fn default() -> Self {
        Self {
            name: "sunlineEKF".to_string(),
            q_obs_val: 0.0,
            q_proc_val: 0.0,
            sensor_use_thresh: 0.0,
            ekf_switch: 0.0,
            css_units: Vec::new(),
        }
    }
}

/// CSS-based sunline extended Kalman filter (upstream `sunlineEKF`).
///
/// Seed `state`, `state_error` and `covar` before `init`, as upstream's
/// Python sets `state`, `x` and `covar`; Reset leaves them untouched.
#[derive(Clone, Debug)]
pub struct SunlineEkf {
    /// User-settable parameters.
    pub config: SunlineEkfConfig,
    /// Upstream `cssDataInMsg`, one input per sensor of `config.css_units`:
    /// its cosine reading is `sensed_value`.
    pub css_data_in_msgs: Vec<Input<SunSensorMsg>>,
    /// Upstream `navStateOutMsg`: the unit sun heading and the filter time.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `filtDataOutMsg`: state, state error, covariance and post-fit
    /// residuals.
    pub filt_data_out_msg: Output<SunlineFilterMsg>,
    /// Upstream `state`: reference state `[d, ḋ]` at the filter time.
    pub state: Vector6<f64>,
    /// Upstream `x`: state error relative to `state`.
    pub state_error: Vector6<f64>,
    /// Upstream `covar`: state covariance.
    pub covar: Matrix6<f64>,
    time_tag_s: f64,
    x_bar: Vector6<f64>,
    covar_bar: Matrix6<f64>,
    state_transition: Matrix6<f64>,
    dyn_mat: Matrix6<f64>,
    proc_noise: Matrix3<f64>,
    css_units: Vec<CssUnit>,
    observations: CssObservations<SKF_N_STATES>,
    post_fits: SVector<f64, MAX_N_CSS_MEAS>,
}

impl SunlineEkf {
    /// A filter with one unconnected CSS input per configured sensor and a
    /// zero estimate; seed `state`, `state_error` and `covar` before `init`.
    pub fn new(config: SunlineEkfConfig) -> Self {
        Self {
            css_data_in_msgs: css_inputs(&config.css_units),
            config,
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            state: Vector6::zeros(),
            state_error: Vector6::zeros(),
            covar: Matrix6::zeros(),
            time_tag_s: 0.0,
            x_bar: Vector6::zeros(),
            covar_bar: Matrix6::zeros(),
            state_transition: Matrix6::zeros(),
            dyn_mat: Matrix6::zeros(),
            proc_noise: Matrix3::zeros(),
            css_units: Vec::new(),
            observations: CssObservations::unprocessed(),
            post_fits: SVector::zeros(),
        }
    }

    /// Upstream `Reset_sunlineEKF`: caches the CSS configuration and restarts
    /// the filter bookkeeping at `call_time_nanos`. The estimate (`state`,
    /// `state_error`, `covar`) and the post-fit residuals are kept, as upstream.
    ///
    /// # Panics
    ///
    /// If `config.css_units` no longer has one entry per CSS input.
    pub fn reset(&mut self, call_time_nanos: u64) {
        check_css_inputs(&self.css_data_in_msgs, &self.config.css_units, "sunlineEKF");
        self.css_units = configured_css_units(&self.config.css_units, "sunlineEKF");

        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.observations = CssObservations::unprocessed();
        self.x_bar = Vector6::zeros();
        self.covar_bar = Matrix6::zeros();
        self.state_transition = Matrix6::identity();
        self.dyn_mat = Matrix6::zeros();
        self.proc_noise = self.config.q_proc_val * Matrix3::identity();
    }

    /// Upstream `Update_sunlineEKF`, with the CSS message header explicit:
    /// `readings` holds one value per sensor, and `written_nanos` is upstream
    /// `timeWritten` when the message `isWritten`.
    fn step(&mut self, call_time_nanos: u64, readings: &[f64], written_nanos: Option<u64>) {
        // Upstream: `timeWritten * NANO2SEC >= timeTag && isWritten`.
        let measurement_time_s = written_nanos.map(|nanos| nanos as f64 * NANO2SEC);
        if let Some(measurement_time_s) = measurement_time_s
            && measurement_time_s >= self.time_tag_s
        {
            self.time_update(measurement_time_s);
            self.measurement_update(readings);
        }

        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.time_tag_s {
            self.time_update(call_time_s);
            self.state_error = self.x_bar;
            self.covar = self.covar_bar;
        }

        self.observations
            .write_post_fit_residuals(&self.state_error, &mut self.post_fits);
        self.nav_state_out_msg.write(self.nav_state());
        self.filt_data_out_msg.write(self.filter_data());
    }

    /// Upstream `sunlineTimeUpdate`: propagates the reference state, the
    /// state error and the covariance to `update_time_s`.
    fn time_update(&mut self, update_time_s: f64) {
        let dt = update_time_s - self.time_tag_s;
        update_dynamics_matrix(&mut self.dyn_mat, &self.state, dt);
        self.state_transition = propagate_state(&mut self.state, &self.dyn_mat, dt);
        self.x_bar = self.state_transition * self.state_error;

        let gamma = process_noise_mapping(dt);
        self.covar_bar = self.state_transition * (self.covar * self.state_transition.transpose())
            + gamma * (self.proc_noise * gamma.transpose());
        self.time_tag_s = update_time_s;
    }

    /// Upstream `sunlineMeasUpdate`.
    fn measurement_update(&mut self, cos_values: &[f64]) {
        self.observations = CssObservations::select(
            &self.state.fixed_rows::<3>(0).into_owned(),
            cos_values,
            &self.css_units,
            self.config.sensor_use_thresh,
        );
        let CssObservations {
            y_meas, meas_mat, ..
        } = &self.observations;
        let q_obs_val = self.config.q_obs_val;
        let gain = kalman_gain(&self.covar_bar, meas_mat, q_obs_val);

        // Re-linearizing about a state this uncertain can make the filter
        // diverge, so a large covariance is first brought down classically.
        let classical = needs_classical_update(&self.covar, self.config.ekf_switch);
        (self.state_error, self.covar) = if classical {
            ckf_update(
                &self.x_bar,
                &gain,
                &self.covar_bar,
                q_obs_val,
                y_meas,
                meas_mat,
            )
        } else {
            ekf_update(
                &gain,
                &self.covar_bar,
                q_obs_val,
                y_meas,
                meas_mat,
                &mut self.state,
            )
        };
    }

    fn nav_state(&self) -> NavigationAttitudeMsg {
        NavigationAttitudeMsg {
            time_tag_s: self.time_tag_s,
            vehicle_sun_point_body: normalize_or_zero(self.state.fixed_rows::<3>(0).into_owned()),
            ..Default::default()
        }
    }

    fn filter_data(&self) -> SunlineFilterMsg {
        SunlineFilterMsg {
            time_tag_s: self.time_tag_s,
            covar: self.covar,
            state: self.state,
            state_error: self.state_error,
            post_fit_res: self.post_fits,
            num_obs: self.observations.count(),
        }
    }
}

impl Module for SunlineEkf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
        self.filt_data_out_msg.write(SunlineFilterMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        SunlineEkf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let readings = css_readings(&self.css_data_in_msgs);
        let written_nanos = css_written_nanos(&self.css_data_in_msgs, context);
        self.step(context.current_sim_nanos, &readings, written_nanos);
    }
}

/// Upstream `sunlineDynMatrix`: the Jacobian of the sunline dynamics at
/// `state`.
///
/// The heading rows are always refreshed. The rate rows are refreshed only
/// when `dt > 1e-10`; otherwise they keep whatever the previous call left
/// (upstream's comment says "leave dynMat zeroed", but nothing zeroes it).
fn update_dynamics_matrix(dyn_mat: &mut Matrix6<f64>, state: &Vector6<f64>, dt: f64) {
    let d = state.fixed_rows::<3>(0).into_owned();
    let d_dot = state.fixed_rows::<3>(3).into_owned();
    let d_dot_d = d.dot(&d_dot);
    let norm_d2 = d.norm() * d.norm();
    let d_outer_d = d * d.transpose();

    let first_term = (1.0 / norm_d2) * (d * d_dot.transpose());
    let second_term =
        (d_dot_d / (norm_d2 * norm_d2)) * (norm_d2 * Matrix3::identity() - 2.0 * d_outer_d);
    let df_dd = -(first_term + second_term);
    let df_dd_dot = Matrix3::identity() - (1.0 / norm_d2) * d_outer_d;
    dyn_mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&df_dd);
    dyn_mat.fixed_view_mut::<3, 3>(0, 3).copy_from(&df_dd_dot);

    if dt > MIN_RATE_PROPAGATION_DT_S {
        dyn_mat
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&((1.0 / dt) * df_dd));
        // `(I + P) - I` as upstream rather than `P`: the round trip rounds the
        // diagonal.
        dyn_mat
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&((1.0 / dt) * (df_dd_dot - Matrix3::identity())));
    }
}

/// Upstream `sunlineStateSTMProp`: removes the rate component along the
/// heading, advances the heading by `dt` and returns the state transition
/// matrix `I + dt A`.
fn propagate_state(state: &mut Vector6<f64>, dyn_mat: &Matrix6<f64>, dt: f64) -> Matrix6<f64> {
    let heading = state.fixed_rows::<3>(0).into_owned();
    let heading_hat = normalize_or_zero(heading);
    let rate = state.fixed_rows::<3>(3).into_owned();
    let observable_rate = rate - rate.dot(&heading_hat) * heading_hat;
    state.fixed_rows_mut::<3>(3).copy_from(&observable_rate);
    state
        .fixed_rows_mut::<3>(0)
        .copy_from(&(heading + dt * observable_rate));
    Matrix6::identity() + dt * dyn_mat
}

/// Upstream's process-noise mapping Γ: `(dt / 2) dt I` over `dt I`.
fn process_noise_mapping(dt: f64) -> SMatrix<f64, 6, 3> {
    let mut gamma = SMatrix::<f64, 6, 3>::zeros();
    gamma
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&Matrix3::from_diagonal_element(dt / 2.0 * dt));
    gamma
        .fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&Matrix3::from_diagonal_element(dt));
    gamma
}

/// Measurement matrix of `N`-state sunline filters: one row per observation
/// (upstream `measMat`, `numObs` × `N`).
pub(crate) type MeasurementMatrix<const N: usize> = OMatrix<f64, Dyn, Const<N>>;

/// Kalman gain of `N`-state sunline filters (upstream `kalmanGain`,
/// `N` × `numObs`).
pub(crate) type KalmanGain<const N: usize> = OMatrix<f64, Const<N>, Dyn>;

/// The CSS readings one measurement update uses, and the linearized
/// measurement model about the reference state (upstream `obs`, `yMeas`,
/// `measMat` and `numObs`).
#[derive(Clone, Debug)]
pub(crate) struct CssObservations<const N: usize> {
    /// Upstream `obs`: the readings above the threshold, in sensor order.
    pub obs: DVector<f64>,
    /// Upstream `yMeas`: each reading minus the reading the reference heading
    /// predicts.
    pub y_meas: DVector<f64>,
    /// Upstream `measMat`: each reading's bias-scaled sensor normal in the
    /// heading columns, zero elsewhere.
    pub meas_mat: MeasurementMatrix<N>,
}

impl<const N: usize> CssObservations<N> {
    /// What upstream Reset leaves: `numObs = MAX_N_CSS_MEAS` with zeroed
    /// rows, so until the first measurement update the filter reports 32
    /// observations and zero post-fit residuals.
    pub(crate) fn unprocessed() -> Self {
        Self {
            obs: DVector::zeros(MAX_N_CSS_MEAS),
            y_meas: DVector::zeros(MAX_N_CSS_MEAS),
            meas_mat: MeasurementMatrix::zeros(MAX_N_CSS_MEAS),
        }
    }

    /// Upstream `sunlineHMatrixYMeas`: keeps the readings above
    /// `sensor_use_thresh` and linearizes them about `sun_heading`.
    pub(crate) fn select(
        sun_heading: &Vector3<f64>,
        cos_values: &[f64],
        css_units: &[CssUnit],
        sensor_use_thresh: f64,
    ) -> Self {
        let (readings, normals): (Vec<f64>, Vec<Vector3<f64>>) = css_units
            .iter()
            .zip(cos_values)
            .filter(|&(_, &reading)| reading > sensor_use_thresh)
            .map(|(unit, &reading)| (reading, unit.c_bias * unit.n_hat_b))
            .unzip();
        let y_meas = readings
            .iter()
            .zip(&normals)
            .map(|(reading, normal)| reading - sun_heading.dot(normal))
            .collect::<Vec<_>>();
        let meas_mat = MeasurementMatrix::from_fn(normals.len(), |row, col| {
            if col < 3 { normals[row][col] } else { 0.0 }
        });
        Self {
            obs: DVector::from_vec(readings),
            y_meas: DVector::from_vec(y_meas),
            meas_mat,
        }
    }

    /// Upstream `numObs`.
    pub(crate) fn count(&self) -> usize {
        self.obs.len()
    }

    /// Upstream's post-fit residuals `yMeas - H x`, written into the leading
    /// `numObs` entries of `post_fits`. As upstream, the remaining entries keep
    /// the values of earlier, larger observation sets.
    pub(crate) fn write_post_fit_residuals(
        &self,
        state_error: &SVector<f64, N>,
        post_fits: &mut SVector<f64, MAX_N_CSS_MEAS>,
    ) {
        let residuals = &self.y_meas - sequential_product(&self.meas_mat, state_error);
        post_fits.rows_mut(0, self.count()).copy_from(&residuals);
    }
}

/// Upstream's switch to the classical update, `vMaxAbs(covar) > eKFSwitch`.
///
/// `vMaxAbs` starts from the magnitude of the first entry and takes any later
/// entry that is larger, so a NaN first entry fails the test (an extended
/// update) while later NaNs are skipped. nalgebra's `amax` would skip a leading
/// NaN as well. The scan order of the remaining entries does not matter.
pub(crate) fn needs_classical_update<const N: usize>(
    covar: &SMatrix<f64, N, N>,
    ekf_switch: f64,
) -> bool {
    let first = covar[(0, 0)].abs();
    let max_abs = covar.iter().skip(1).fold(first, |max, entry| {
        let magnitude = entry.abs();
        if magnitude > max { magnitude } else { max }
    });
    max_abs > ekf_switch
}

/// Upstream `sunlineKalmanGain` (and `sunlineKalmanGainOkeefe`):
/// `K = P̄ Hᵀ (H P̄ Hᵀ + qObsVal I)⁻¹`.
///
/// The inverse is upstream `mInverse`, quirks included: with no observations
/// the gain is empty, and a single observation inverts to zero, so its gain is
/// zero.
pub(crate) fn kalman_gain<const N: usize>(
    covar_bar: &SMatrix<f64, N, N>,
    meas_mat: &MeasurementMatrix<N>,
    q_obs_val: f64,
) -> KalmanGain<N> {
    let num_obs = meas_mat.nrows();
    let meas_mat_t = meas_mat.transpose();
    let covar_h_t = sequential_product(covar_bar, &meas_mat_t);
    let h_covar = sequential_product(meas_mat, covar_bar);
    let innovation_covar =
        sequential_product(&h_covar, &meas_mat_t) + q_obs_val * DMatrix::identity(num_obs, num_obs);
    let inverse = m_inverse(&innovation_covar).unwrap_or_else(|| DMatrix::zeros(num_obs, num_obs));
    sequential_product(&covar_h_t, &inverse)
}

/// Upstream `sunlineEKFUpdate` (and `okeefeEKFUpdate`): the state error
/// `K y` moves the reference `state`; returns that error and the updated
/// covariance.
pub(crate) fn ekf_update<const N: usize>(
    gain: &KalmanGain<N>,
    covar_bar: &SMatrix<f64, N, N>,
    q_obs_val: f64,
    y_meas: &DVector<f64>,
    meas_mat: &MeasurementMatrix<N>,
    state: &mut SVector<f64, N>,
) -> (SVector<f64, N>, SMatrix<f64, N, N>) {
    let state_error = sequential_product(gain, y_meas);
    *state += state_error;
    (
        state_error,
        joseph_covariance(gain, meas_mat, covar_bar, q_obs_val),
    )
}

/// Upstream `sunlineCKFUpdate` (and `sunlineCKFUpdateOkeefe`): corrects the
/// propagated state error `x_bar` with the innovation and leaves the reference
/// state alone; returns the new state error and covariance.
pub(crate) fn ckf_update<const N: usize>(
    x_bar: &SVector<f64, N>,
    gain: &KalmanGain<N>,
    covar_bar: &SMatrix<f64, N, N>,
    q_obs_val: f64,
    y_meas: &DVector<f64>,
    meas_mat: &MeasurementMatrix<N>,
) -> (SVector<f64, N>, SMatrix<f64, N, N>) {
    let innovation = y_meas - sequential_product(meas_mat, x_bar);
    let state_error = x_bar + sequential_product(gain, &innovation);
    (
        state_error,
        joseph_covariance(gain, meas_mat, covar_bar, q_obs_val),
    )
}

/// Joseph-form covariance after a measurement update:
/// `(I - K H) P̄ (I - K H)ᵀ + K (qObsVal I) Kᵀ`.
fn joseph_covariance<const N: usize>(
    gain: &KalmanGain<N>,
    meas_mat: &MeasurementMatrix<N>,
    covar_bar: &SMatrix<f64, N, N>,
    q_obs_val: f64,
) -> SMatrix<f64, N, N> {
    let num_obs = gain.ncols();
    let i_minus_kh = SMatrix::<f64, N, N>::identity() - sequential_product(gain, meas_mat);
    let meas_noise = q_obs_val * DMatrix::identity(num_obs, num_obs);
    (i_minus_kh * covar_bar) * i_minus_kh.transpose()
        + sequential_product(&sequential_product(gain, &meas_noise), &gain.transpose())
}

#[cfg(test)]
pub(crate) mod tests {
    //! Ports of upstream `_UnitTest/test_SunLineEKF.py`: `test_all_functions_ekf`
    //! (`sunline_individual_test`, one Rust test per helper it checks,
    //! `StatePropStatic`, `StatePropVariable`) and `test_all_sunline_ekf`
    //! (`StateUpdateSunLine`, all six parametrizations). Truth values are
    //! recomputed here as the Python computes them, with upstream's
    //! tolerances. `SunLineEKF_test_utilities.py` only draws figures and is not
    //! ported.
    //!
    //! Two deviations, both from the test environment rather than the filter:
    //!
    //! * Upstream writes the CSS message with `write(payload, CurrentNanos)`
    //!   from step 21 on, the filter reading the header's write time. The
    //!   ports carry no header (see the module docs), so [`UpstreamTimeline`]
    //!   replays that timeline through the private `step`, with the write time
    //!   upstream reads.
    //! * The measurement noise comes from a seeded `StdRng`, not NumPy's
    //!   `default_rng(42)`: the same distribution, different samples, so the
    //!   noisy cases check the same statistical tolerances on another draw.

    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, SQRT_2};

    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, UnitQuaternion, Vector3, Vector6};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, Normal};

    use super::{
        CssObservations, KalmanGain, MeasurementMatrix, SunlineEkf, SunlineEkfConfig, ckf_update,
        ekf_update, kalman_gain, needs_classical_update, propagate_state, update_dynamics_matrix,
    };
    use crate::fsw_algorithms::att_determination::CssUnit;
    use crate::fsw_algorithms::att_determination::css_inputs::test_support::UpstreamTimeline;
    use crate::messages::{
        Output, SpacecraftStateMsg, SunEphemerisMsg, SunSensorMsg, SunlineFilterMsg,
    };
    use crate::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    /// Upstream `testProcessRate`: 0.5 s.
    pub(crate) const TASK_RATE_NANOS: u64 = 500_000_000;
    const DT_S: f64 = 0.5;

    pub(crate) fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream's eight-sensor test constellation (`CSSOrientationList`).
    /// Its `0.70710678118654746` is one ulp below `1/√2` and equals
    /// `1 / SQRT_2`; its `0.70710678118654757` is `FRAC_1_SQRT_2`.
    pub(crate) fn css_constellation() -> Vec<CssUnit> {
        let a = 1.0 / SQRT_2;
        let b = FRAC_1_SQRT_2;
        [
            [a, -0.5, 0.5],
            [a, -0.5, -0.5],
            [a, 0.5, -0.5],
            [a, 0.5, 0.5],
            [-a, 0.0, b],
            [-a, b, 0.0],
            [-a, 0.0, -b],
            [-a, -b, 0.0],
        ]
        .into_iter()
        .map(|n_hat_b| CssUnit {
            n_hat_b: Vector3::from(n_hat_b),
            c_bias: 1.0,
        })
        .collect()
    }

    /// One constellation reading of `sun_heading` (upstream's `dotList`),
    /// with `N(0, √qObsVal)` noise per sensor when `noise` is given.
    pub(crate) fn css_reading(
        css_units: &[CssUnit],
        sun_heading: &Vector3<f64>,
        noise: Option<(&Normal<f64>, &mut StdRng)>,
    ) -> Vec<f64> {
        let mut cos_values: Vec<f64> = css_units
            .iter()
            .map(|unit| unit.n_hat_b.dot(sun_heading))
            .collect();
        if let Some((normal, rng)) = noise {
            for cos_value in &mut cos_values {
                *cos_value += normal.sample(rng);
            }
        }
        cos_values
    }

    /// One CSS message per reading.
    pub(crate) fn css_outputs(readings: &[f64]) -> Vec<Output<SunSensorMsg>> {
        readings
            .iter()
            .map(|&sensed_value| {
                Output::new(SunSensorMsg {
                    sensed_value,
                    ..Default::default()
                })
            })
            .collect()
    }

    /// The sensors and readings of `sunline_individual_test`'s measurement
    /// checks (`numCSS = 4`, unit biases).
    pub(crate) fn individual_test_css() -> (Vec<CssUnit>, [f64; 4]) {
        let normals = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0 / 2.0_f64.sqrt(), 1.0 / 2.0_f64.sqrt(), 0.0],
        ];
        let units = normals
            .into_iter()
            .map(|n_hat_b| CssUnit {
                n_hat_b: Vector3::from(n_hat_b),
                c_bias: 1.0,
            })
            .collect();
        let cos_values = [10.0_f64, 25.0, 5.0, 90.0].map(|deg| deg.to_radians().cos());
        (units, cos_values)
    }

    /// Upstream `setupFilterData`, with the sensors `css_units` (upstream
    /// subscribes the filter to their configuration message separately).
    fn setup_filter_data(css_units: Vec<CssUnit>) -> SunlineEkf {
        let mut module = SunlineEkf::new(SunlineEkfConfig {
            name: "SunlineEKF".to_string(),
            sensor_use_thresh: 0.0,
            // Upstream `0.1**2`, which is 0.010000000000000002.
            q_proc_val: 0.1 * 0.1,
            q_obs_val: 0.001,
            ekf_switch: 5.0,
            css_units,
        });
        module.state = Vector6::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
        module.state_error = Vector6::new(1.0, 0.0, 1.0, 0.0, 0.1, 0.0);
        module.covar = Matrix6::from_diagonal(&Vector6::new(0.4, 0.4, 0.4, 0.004, 0.004, 0.004));
        module
    }

    /// `expDynMat` as the Python builds it from `states` and `dt`.
    fn expected_dynamics_matrix(states: &Vector6<f64>, dt: f64) -> Matrix6<f64> {
        let d = states.fixed_rows::<3>(0).into_owned();
        let d_dot = states.fixed_rows::<3>(3).into_owned();
        let norm = d.norm();
        let d_outer_d = d * d.transpose();
        let df_dd = -((d * d_dot.transpose()) / norm.powi(2)
            + d_dot.dot(&d) * (norm.powi(2) * Matrix3::identity() - 2.0 * d_outer_d)
                / norm.powi(4));
        let mut dyn_mat = Matrix6::zeros();
        dyn_mat.fixed_view_mut::<3, 3>(0, 0).copy_from(&df_dd);
        dyn_mat
            .fixed_view_mut::<3, 3>(0, 3)
            .copy_from(&(Matrix3::identity() - d_outer_d / norm.powi(2)));
        dyn_mat
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(df_dd / dt));
        dyn_mat
            .fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(-1.0 / dt * (d_outer_d / norm.powi(2))));
        dyn_mat
    }

    /// `d_dot` without its component along `d`, as the Python computes it.
    fn observable_rate(d: &Vector3<f64>, d_dot: &Vector3<f64>) -> Vector3<f64> {
        d_dot - d_dot.dot(d) * d / d.norm().powi(2)
    }

    /// Upstream's flat `h` (8 × 6, row-major) with the identity in the first
    /// three rows; only the first `numObs = 3` rows are used.
    fn individual_test_meas_mat() -> MeasurementMatrix<6> {
        MeasurementMatrix::from_fn(3, |row, col| if row == col { 1.0 } else { 0.0 })
    }

    /// Upstream's `covar` for the gain and update checks.
    fn individual_test_covar() -> Matrix6<f64> {
        let identity = Matrix3::identity();
        let mut covar = Matrix6::zeros();
        for (row, col) in [(0, 0), (0, 3), (3, 0), (3, 3)] {
            covar.fixed_view_mut::<3, 3>(row, col).copy_from(&identity);
        }
        covar
    }

    /// Upstream's `KGain`: its first 18 entries read as 6 × 3 row-major.
    fn individual_test_gain() -> KalmanGain<6> {
        KalmanGain::from_row_slice(&[
            1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 3.0, 0.0, 1.0, 0.0, 2.0,
            0.0,
        ])
    }

    /// Upstream's `inputY`: the first three readings minus the heading's
    /// prediction.
    pub(crate) fn individual_test_y(heading: &Vector3<f64>) -> DVector<f64> {
        let (units, cos_values) = individual_test_css();
        DVector::from_iterator(
            3,
            (0..3).map(|j| cos_values[j] - heading.dot(&units[j].n_hat_b)),
        )
    }

    /// The Joseph-form covariance the Python expects from both updates.
    pub(crate) fn expected_joseph_covariance(
        gain: &DMatrix<f64>,
        meas_mat: &DMatrix<f64>,
        covar: &DMatrix<f64>,
        noise: f64,
    ) -> DMatrix<f64> {
        let (num_states, num_obs) = gain.shape();
        let i_minus_kh = DMatrix::identity(num_states, num_states) - gain * meas_mat;
        &i_minus_kh * covar * i_minus_kh.transpose()
            + gain * (noise * DMatrix::identity(num_obs, num_obs)) * gain.transpose()
    }

    /// `vector` zero-padded to `len`, like upstream's fixed-size output arrays.
    pub(crate) fn padded(vector: &DVector<f64>, len: usize) -> DVector<f64> {
        DVector::from_fn(len, |i, _| vector.get(i).copied().unwrap_or(0.0))
    }

    #[test]
    fn matches_upstream_dynamics_matrix_check() {
        let input_states = Vector6::new(2.0, 1.0, 0.75, 0.1, 0.4, 0.05);
        let mut dyn_mat = Matrix6::zeros();
        update_dynamics_matrix(&mut dyn_mat, &input_states, DT_S);
        assert!((expected_dynamics_matrix(&input_states, DT_S) - dyn_mat).norm() <= 1.0e-10);
    }

    #[test]
    fn dynamics_matrix_keeps_rate_rows_for_a_zero_step() {
        let mut dyn_mat = Matrix6::from_element(7.0);
        let states = Vector6::new(2.0, 1.0, 0.75, 0.1, 0.4, 0.05);
        update_dynamics_matrix(&mut dyn_mat, &states, 0.0);
        let expected = expected_dynamics_matrix(&states, DT_S);
        assert!((dyn_mat.fixed_rows::<3>(0) - expected.fixed_rows::<3>(0)).norm() <= 1.0e-10);
        assert_eq!(
            dyn_mat.fixed_rows::<3>(3),
            Matrix6::from_element(7.0).fixed_rows::<3>(3)
        );
    }

    #[test]
    fn matches_upstream_state_and_stm_propagation_check() {
        // Upstream propagates with the dynamics matrix of the previous check.
        let dyn_mat = expected_dynamics_matrix(&Vector6::new(2.0, 1.0, 0.75, 0.1, 0.4, 0.05), DT_S);
        let input_states = Vector6::new(2.0, 1.0, 0.75, 1.5, 0.5, 0.5);
        let mut states = input_states;
        let stm = propagate_state(&mut states, &dyn_mat, DT_S);

        let d = input_states.fixed_rows::<3>(0).into_owned();
        let rate = observable_rate(&d, &input_states.fixed_rows::<3>(3).into_owned());
        let mut expected_states = Vector6::zeros();
        expected_states
            .fixed_rows_mut::<3>(0)
            .copy_from(&(d + DT_S * rate));
        expected_states.fixed_rows_mut::<3>(3).copy_from(&rate);
        let expected_stm = DT_S * dyn_mat + Matrix6::identity();
        assert!((expected_stm - stm).norm() <= 1.0e-10);
        assert!((expected_states - states).norm() <= 1.0e-10);
    }

    #[test]
    fn matches_upstream_measurement_model_check() {
        let heading = Vector3::new(2.0, 1.0, 0.75);
        let (units, cos_values) = individual_test_css();
        let observations = CssObservations::<6>::select(
            &heading,
            &cos_values,
            &units,
            50.0_f64.to_radians().cos(),
        );

        let mut expected_h = DMatrix::zeros(8, 6);
        let mut expected_y = DVector::zeros(8);
        for j in 0..3 {
            expected_h[(j, j)] = 1.0;
            expected_y[j] = cos_values[j] - heading.dot(&units[j].n_hat_b);
        }
        let mut expected_obs = DVector::zeros(8);
        expected_obs
            .rows_mut(0, 3)
            .copy_from_slice(&cos_values[..3]);
        let h_out = DMatrix::from_fn(8, 6, |row, col| {
            if row < observations.count() {
                observations.meas_mat[(row, col)]
            } else {
                0.0
            }
        });

        assert!((h_out - expected_h).norm() <= 1.0e-10);
        assert!((padded(&observations.y_meas, 8) - expected_y).norm() <= 1.0e-10);
        assert!((padded(&observations.obs, 8) - expected_obs).norm() <= 1.0e-10);
        assert_eq!(observations.count(), 3);
    }

    #[test]
    fn matches_upstream_kalman_gain_check() {
        let meas_mat = individual_test_meas_mat();
        let covar = individual_test_covar();
        let noise = 0.01;
        let gain = kalman_gain(&covar, &meas_mat, noise);

        let h = DMatrix::from_fn(3, 6, |row, col| meas_mat[(row, col)]);
        let p = DMatrix::from_fn(6, 6, |row, col| covar[(row, col)]);
        let innovation = &h * &p * h.transpose() + noise * DMatrix::identity(3, 3);
        let expected = &p
            * h.transpose()
            * innovation
                .try_inverse()
                .expect("the innovation covariance is invertible");
        assert!((DMatrix::from_fn(6, 3, |row, col| gain[(row, col)]) - expected).norm() <= 1.0e-10);
    }

    #[test]
    fn matches_upstream_ekf_update_check() {
        let gain = individual_test_gain();
        let input_states = Vector6::new(2.0, 1.0, 0.75, 0.1, 0.4, 0.05);
        let meas_mat = individual_test_meas_mat();
        let covar = individual_test_covar();
        let noise = 0.01;
        let input_y = individual_test_y(&input_states.fixed_rows::<3>(0).into_owned());

        let mut states = input_states;
        let (_, covar_out) = ekf_update(&gain, &covar, noise, &input_y, &meas_mat, &mut states);

        let k = DMatrix::from_fn(6, 3, |row, col| gain[(row, col)]);
        let h = DMatrix::from_fn(3, 6, |row, col| meas_mat[(row, col)]);
        let p = DMatrix::from_fn(6, 6, |row, col| covar[(row, col)]);
        let expected_states = DVector::from_column_slice(input_states.as_slice()) + &k * &input_y;
        let expected_p = expected_joseph_covariance(&k, &h, &p, noise);
        assert!(
            (DVector::from_column_slice(states.as_slice()) - expected_states).norm() <= 1.0e-10
        );
        assert!((DMatrix::from_fn(6, 6, |r, c| covar_out[(r, c)]) - expected_p).norm() <= 1.0e-10);
    }

    #[test]
    fn matches_upstream_ckf_update_check() {
        let gain = individual_test_gain();
        let input_states = Vector6::new(2.0, 1.0, 0.75, 0.1, 0.4, 0.05);
        let x_bar = Vector6::new(0.1, 0.2, 0.01, 0.005, 0.009, 0.001);
        let meas_mat = individual_test_meas_mat();
        let covar = individual_test_covar();
        let noise = 0.01;
        let input_y = individual_test_y(&input_states.fixed_rows::<3>(0).into_owned());

        let (state_error, covar_out) =
            ckf_update(&x_bar, &gain, &covar, noise, &input_y, &meas_mat);

        let k = DMatrix::from_fn(6, 3, |row, col| gain[(row, col)]);
        let h = DMatrix::from_fn(3, 6, |row, col| meas_mat[(row, col)]);
        let p = DMatrix::from_fn(6, 6, |row, col| covar[(row, col)]);
        let x_bar_d = DVector::from_column_slice(x_bar.as_slice());
        let expected_state_error = &x_bar_d + &k * (&input_y - &h * &x_bar_d);
        let expected_p = expected_joseph_covariance(&k, &h, &p, noise);
        assert!(
            (DVector::from_column_slice(state_error.as_slice()) - expected_state_error).norm()
                <= 1.0e-10
        );
        assert!((DMatrix::from_fn(6, 6, |r, c| covar_out[(r, c)]) - expected_p).norm() <= 1.0e-10);
    }

    #[test]
    fn a_single_observation_gets_upstreams_zero_gain() {
        // Upstream `mInverse` of a 1×1 matrix returns 0 (see `m_inverse`).
        let meas_mat = MeasurementMatrix::<6>::from_row_slice(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let gain = kalman_gain(&individual_test_covar(), &meas_mat, 0.01);
        assert_eq!(gain, KalmanGain::<6>::zeros(1));
    }

    #[test]
    fn classical_update_switch_scans_the_covariance_like_upstream_v_max_abs() {
        let switch = 0.3;
        let mut covar = Matrix6::from_element(0.2);
        assert!(!needs_classical_update(&covar, switch));
        covar[(4, 1)] = -0.5;
        assert!(needs_classical_update(&covar, switch));

        // `vMaxAbs` keeps a NaN first entry and skips later ones.
        let mut nan_first = Matrix6::from_element(0.4);
        nan_first[(0, 0)] = f64::NAN;
        assert!(!needs_classical_update(&nan_first, switch));
        let mut nan_later = Matrix6::from_element(0.4);
        nan_later[(0, 1)] = f64::NAN;
        assert!(needs_classical_update(&nan_later, switch));
    }

    #[test]
    fn matches_upstream_state_prop_static() {
        // Neither input message is ever written: no sensors, no readings.
        let mut module = setup_filter_data(Vec::new());
        module.init();

        // Upstream logs every 5 s over 8000 s; its first and last samples are
        // the t = 0 and t = 8000 s updates.
        module.update(&context(0));
        let first = module.state;
        for step in 1..=16_000 {
            module.update(&context(step * TASK_RATE_NANOS));
        }
        for i in 0..6 {
            assert!((module.state[i] - first[i]).abs() <= 1.0e-10, "state {i}");
        }
    }

    #[test]
    fn matches_upstream_state_prop_variable() {
        // As in the static case, no CSS message is ever written.
        let mut module = setup_filter_data(Vec::new());
        module.state += Vector6::new(0.0, 0.0, 0.0, 0.0001, 0.002, 0.001);
        let initial_state = module.state;
        let initial_x = module.state_error;
        let initial_covar = module.covar;
        let q_proc_val = module.config.q_proc_val;
        module.init();

        let mut log = Vec::new();
        for step in 0..=2000 {
            module.update(&context(step * TASK_RATE_NANOS));
            log.push((
                module.state,
                module.state_error,
                module.covar,
                module.state_transition,
            ));
        }

        let mut gamma = nalgebra::SMatrix::<f64, 6, 3>::zeros();
        gamma
            .fixed_view_mut::<3, 3>(0, 0)
            .copy_from(&(DT_S.powi(2) / 2.0 * Matrix3::identity()));
        gamma
            .fixed_view_mut::<3, 3>(3, 0)
            .copy_from(&(DT_S * Matrix3::identity()));
        let proc_noise_covar = gamma * (q_proc_val * Matrix3::identity()) * gamma.transpose();

        let mut expected_state = initial_state;
        let mut expected_x = initial_x;
        let mut expected_covar = initial_covar;
        for (j, (state, x, covar, stm)) in log.iter().enumerate().skip(1) {
            let expected_stm =
                DT_S * expected_dynamics_matrix(&expected_state, DT_S) + Matrix6::identity();
            let d = expected_state.fixed_rows::<3>(0).into_owned();
            let rate = observable_rate(&d, &expected_state.fixed_rows::<3>(3).into_owned());
            expected_state
                .fixed_rows_mut::<3>(0)
                .copy_from(&(d + DT_S * rate));
            expected_state.fixed_rows_mut::<3>(3).copy_from(&rate);
            expected_x = expected_stm * expected_x;
            expected_covar =
                expected_stm * expected_covar * expected_stm.transpose() + proc_noise_covar;

            for i in 0..6 {
                assert!(
                    (state[i] - expected_state[i]).abs() <= 1.0e-4,
                    "state {i} at {j}"
                );
                assert!(
                    (x[i] - expected_x[i]).abs() <= 1.0e-4,
                    "state error {i} at {j}"
                );
            }
            for i in 0..36 {
                assert!(
                    (covar[i] - expected_covar[i]).abs() <= 1.0e-4,
                    "covar {i} at {j}"
                );
                assert!((stm[i] - expected_stm[i]).abs() <= 1.0e-4, "STM {i} at {j}");
            }
        }
    }

    /// One `test_all_sunline_ekf` parametrization.
    #[derive(Debug)]
    struct StateUpdateCase {
        sim_half_length: usize,
        add_meas_noise: bool,
        test_vector1: [f64; 3],
        test_vector2: [f64; 3],
        state_guess: [f64; 6],
    }

    const STATE_UPDATE_CASES: [StateUpdateCase; 6] = [
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 2000,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: false,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: false,
            test_vector1: [0.0, 0.4, -0.4],
            test_vector2: [0.0, 0.7, 0.2],
            state_guess: [0.3, 0.0, 0.6, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [0.0, 0.4, -0.4],
            test_vector2: [0.4, 0.5, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0, 0.0, 0.0, 0.0],
        },
    ];

    /// Upstream `StateUpdateSunLine`: converge on `test_vector1`, then on
    /// `test_vector2`. Returns upstream's failure messages.
    fn state_update_sun_line(case: &StateUpdateCase) -> Vec<String> {
        let mut failures = Vec::new();
        let mut rng = StdRng::seed_from_u64(42);
        // Upstream: loose under measurement noise, tight without it.
        let state_conv_tol = if case.add_meas_noise { 1.0e-1 } else { 1.0e-2 };

        let css_units = css_constellation();
        let mut module = setup_filter_data(css_units.clone());

        let target =
            |vector: [f64; 3]| Vector6::new(vector[0], vector[1], vector[2], 0.0, 0.0, 0.0);
        let state_target1 = target(case.test_vector1);
        let state_target2 = target(case.test_vector2);
        module.state = Vector6::from_row_slice(&case.state_guess);
        module.state_error = state_target1 - module.state;
        module.init();
        let noise = Normal::new(0.0, module.config.q_obs_val.sqrt())
            .expect("the noise standard deviation is finite and positive");

        let mut timeline = UpstreamTimeline::new(css_units.len());
        let mut data_log: Vec<SunlineFilterMsg> = Vec::new();
        for (half, sun_heading) in [case.test_vector1, case.test_vector2].iter().enumerate() {
            let sun_heading = Vector3::from(*sun_heading);
            for i in 0..case.sim_half_length {
                if i > 20 {
                    let draw = case.add_meas_noise.then_some((&noise, &mut rng));
                    timeline.write(css_reading(&css_units, &sun_heading, draw));
                }
                let stop_nanos = (half * case.sim_half_length + i + 1) as u64 * TASK_RATE_NANOS;
                timeline.execute_until(stop_nanos, |call_nanos, readings, written_nanos| {
                    module.step(call_nanos, readings, written_nanos);
                    data_log.push(module.filt_data_out_msg.read());
                });
            }

            let state_target = if half == 0 {
                state_target1
            } else {
                state_target2
            };
            let (first, last) = (&data_log[0], &data_log[data_log.len() - 1]);
            for i in 0..6 {
                if (last.covar[(i, i)] - first.covar[(i, i)] / 100.0).abs() > 1.0e-2 {
                    failures.push(format!("Covariance update failure (half {half}, {i})"));
                }
                if (last.state[i] - state_target[i]).abs() > state_conv_tol {
                    failures.push(format!("State update failure (half {half}, {i})"));
                }
            }
        }
        failures
    }

    #[test]
    fn matches_upstream_state_update_sun_line_for_all_parametrizations() {
        for case in &STATE_UPDATE_CASES {
            let failures = state_update_sun_line(case);
            assert!(failures.is_empty(), "{case:?}: {failures:?}");
        }
    }

    #[test]
    fn reports_upstreams_reset_observation_count_until_the_first_measurement() {
        let mut module = setup_filter_data(css_constellation());
        module.init();
        // Upstream's subscribed but never written CSS message.
        module.step(TASK_RATE_NANOS, &[0.0; 8], None);

        let filter_data = module.filt_data_out_msg.read();
        assert_eq!(filter_data.num_obs, crate::messages::MAX_N_CSS_MEAS);
        assert!(filter_data.post_fit_res.iter().all(|&res| res == 0.0));
        assert_eq!(filter_data.time_tag_s, 0.5);
        let nav = module.nav_state_out_msg.read();
        assert_eq!(nav.time_tag_s, 0.5);
        assert!((nav.vehicle_sun_point_body.norm() - 1.0).abs() <= 1.0e-15);
    }

    /// A clean constellation reading of `(-0.7, 0.7, 0)`.
    fn clean_reading() -> Vec<f64> {
        css_reading(&css_constellation(), &Vector3::new(-0.7, 0.7, 0.0), None)
    }

    #[test]
    fn reset_restarts_the_filter_clock_and_keeps_the_estimate() {
        let mut module = setup_filter_data(css_constellation());
        module.init();
        module.step(TASK_RATE_NANOS, &clean_reading(), Some(0));
        let estimate = (module.state, module.state_error, module.covar);
        let post_fits = module.post_fits;

        module.reset(10 * TASK_RATE_NANOS);
        assert_eq!((module.state, module.state_error, module.covar), estimate);
        assert_eq!(module.post_fits, post_fits);
        assert_eq!(module.time_tag_s, 5.0);
        assert_eq!(module.observations.count(), crate::messages::MAX_N_CSS_MEAS);
        assert_eq!(module.state_transition, Matrix6::identity());
    }

    /// `setup_filter_data` with `ekf_switch` and `sensor_use_thresh`, after
    /// one update at 0.5 s that processes [`clean_reading`] written at 0 s.
    fn filter_after_first_reading(ekf_switch: f64, sensor_use_thresh: f64) -> SunlineEkf {
        let mut module = setup_filter_data(css_constellation());
        module.config.ekf_switch = ekf_switch;
        module.config.sensor_use_thresh = sensor_use_thresh;
        module.init();
        module.step(TASK_RATE_NANOS, &clean_reading(), Some(0));
        module
    }

    #[test]
    fn processes_a_css_message_only_when_written_no_earlier_than_the_filter_time() {
        let estimate = |module: &SunlineEkf| (module.state, module.state_error, module.covar);

        // The 0 s reading is stale once the filter has moved on to 0.5 s.
        let mut stale = filter_after_first_reading(5.0, 0.0);
        // Processed: three sensors face that sun heading.
        assert_eq!(stale.observations.count(), 3);
        stale.step(2 * TASK_RATE_NANOS, &clean_reading(), Some(0));

        let mut silent = filter_after_first_reading(5.0, 0.0);
        silent.step(2 * TASK_RATE_NANOS, &[0.0; 8], None);
        assert_eq!(estimate(&stale), estimate(&silent));

        // The same reading written at the filter time is processed again.
        let mut fresh = filter_after_first_reading(5.0, 0.0);
        fresh.step(2 * TASK_RATE_NANOS, &clean_reading(), Some(TASK_RATE_NANOS));
        assert_ne!(fresh.state, stale.state);
    }

    #[test]
    fn update_reads_connected_css_inputs_as_written_at_the_call_time() {
        let readings = clean_reading();
        let css_data = css_outputs(&readings);
        let mut through_update = setup_filter_data(css_constellation());
        // Sensor 5 faces the sun, but its input is left unconnected.
        let mut partly_connected = setup_filter_data(css_constellation());
        for (index, output) in css_data.iter().enumerate() {
            output.connect_to(&mut through_update.css_data_in_msgs[index]);
            if index != 5 {
                output.connect_to(&mut partly_connected.css_data_in_msgs[index]);
            }
        }
        let mut through_step = setup_filter_data(css_constellation());
        let mut unwritten_step = setup_filter_data(css_constellation());
        for module in [
            &mut through_update,
            &mut partly_connected,
            &mut through_step,
            &mut unwritten_step,
        ] {
            module.init();
        }
        let mut partial_readings = readings.clone();
        partial_readings[5] = 0.0;

        for tick in 0..20 {
            let now = tick * TASK_RATE_NANOS;
            through_update.update(&context(now));
            through_step.step(now, &readings, Some(now));
            assert_eq!(
                through_update.filt_data_out_msg.read(),
                through_step.filt_data_out_msg.read()
            );

            // One unconnected input: the readings count as never written.
            partly_connected.update(&context(now));
            unwritten_step.step(now, &partial_readings, None);
            assert_eq!(
                partly_connected.filt_data_out_msg.read(),
                unwritten_step.filt_data_out_msg.read()
            );
        }
        // Sensors 4, 5 and 6 face this sun direction.
        assert_eq!(through_update.filt_data_out_msg.read().num_obs, 3);
    }

    #[test]
    fn config_set_after_new_takes_effect_at_init() {
        let run = |mut module: SunlineEkf| {
            module.init();
            for step in 0..4 {
                module.step(
                    (step + 1) * TASK_RATE_NANOS,
                    &clean_reading(),
                    Some(step * TASK_RATE_NANOS),
                );
            }
            module.filt_data_out_msg.read()
        };
        // `new` fixes the sensors, as it makes one input per sensor; the
        // other parameters may change until `init`.
        let tuned = SunlineEkfConfig {
            q_obs_val: 0.01,
            q_proc_val: 0.5,
            sensor_use_thresh: 0.3,
            ekf_switch: 0.0,
            ..setup_filter_data(css_constellation()).config
        };

        let seeded = setup_filter_data(css_constellation());
        let mut configured_in_new = SunlineEkf::new(tuned.clone());
        configured_in_new.state = seeded.state;
        configured_in_new.state_error = seeded.state_error;
        configured_in_new.covar = seeded.covar;
        let mut configured_later = setup_filter_data(css_constellation());
        configured_later.config = tuned;

        let expected = run(configured_in_new);
        assert_eq!(run(configured_later), expected);
        assert_ne!(run(setup_filter_data(css_constellation())), expected);
    }

    #[test]
    fn a_classical_update_leaves_the_reference_state_uncorrected() {
        // Upstream quirk: in CKF mode only the state error absorbs the
        // measurement, so the reference and the published heading stay put.
        let seeded = setup_filter_data(Vec::new());
        let classical = filter_after_first_reading(0.0, 0.0);
        assert_eq!(classical.state, seeded.state);
        assert_ne!(classical.state_error, seeded.state_error);
        assert_eq!(
            classical.nav_state_out_msg.read().vehicle_sun_point_body,
            Vector3::repeat(1.0 / 3.0_f64.sqrt())
        );

        let extended = filter_after_first_reading(5.0, 0.0);
        assert_ne!(extended.state, seeded.state);
    }

    #[test]
    fn an_extended_update_without_observations_zeroes_the_state_error() {
        // Upstream quirk: with no reading above the threshold the gain is
        // empty, so the extended update sets `x = K y` to zero.
        let module = filter_after_first_reading(5.0, 2.0);
        assert_eq!(module.observations.count(), 0);
        assert_eq!(module.state_error, Vector6::zeros());
        assert_eq!(module.filt_data_out_msg.read().num_obs, 0);
    }

    /// Not upstream: one [`CoarseSunSensor`] per normal of upstream's
    /// constellation, scheduled before the filter and wired to its inputs.
    /// From upstream's tuning and initial guess, the estimate converges on
    /// the sun direction in the body frame of a spacecraft at rest.
    #[test]
    fn converges_on_the_sun_heading_measured_by_coarse_sun_sensors() {
        const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;
        let css_units = css_constellation();
        let mut sensors: Vec<CoarseSunSensor> = css_units
            .iter()
            .enumerate()
            .map(|(index, unit)| {
                CoarseSunSensor::new(CoarseSunSensorConfig {
                    name: format!("css{index}"),
                    position_m: Vector3::zeros(),
                    body_to_sensor_quaternion: UnitQuaternion::rotation_between(
                        &unit.n_hat_b,
                        &Vector3::z(),
                    )
                    .expect("no normal points along -z"),
                    fov_half_angle_rad: FRAC_PI_2,
                    scale_factor: 1.0,
                    kelly_factor: 0.0,
                    k_power: 1.0,
                    bias: 0.0,
                    noise_std: 0.0,
                    noise_prop: 0.0,
                    walk_bounds: 0.0,
                    min_output: 0.0,
                    max_output: 2.0,
                })
            })
            .collect();
        let spacecraft_state = SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            omega_radps: Vector3::zeros(),
        };
        let sun_direction_n = Vector3::new(0.3, -0.5, 0.8).normalize();
        let sun_direction_b = spacecraft_state
            .inertial_to_body()
            .transform_vector(&sun_direction_n);
        let state = Output::new(spacecraft_state);
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sun_direction_n * ASTRONOMICAL_UNIT_M,
            sun_velocity_inertial_mps: Vector3::zeros(),
        });

        let mut module = setup_filter_data(css_units.clone());
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        for (sensor, css_input) in sensors.iter_mut().zip(&mut module.css_data_in_msgs) {
            simulation.connect(&state, &mut sensor.input_state_msg);
            simulation.connect(&sun, &mut sensor.input_sun_msg);
            simulation.connect(&sensor.output_sun_sensor_msg, css_input);
        }
        for (index, sensor) in sensors.iter_mut().enumerate() {
            simulation.add_module(format!("css{index}"), sensor, TASK_RATE_NANOS, 10);
        }
        simulation.add_module("sunlineEKF", &mut module, TASK_RATE_NANOS, 0);
        simulation.run_for(200 * TASK_RATE_NANOS);
        drop(simulation);

        let lit_sensors = css_units
            .iter()
            .filter(|unit| unit.n_hat_b.dot(&sun_direction_b) > 0.0)
            .count();
        let filter_data = module.filt_data_out_msg.read();
        assert_eq!(filter_data.num_obs, lit_sensors);
        assert_eq!(filter_data.time_tag_s, 100.0);
        // Noise-free readings pin the direction down to rounding.
        let heading = module.nav_state_out_msg.read().vehicle_sun_point_body;
        let direction_error = (heading - sun_direction_b).norm();
        assert!(
            direction_error <= 1.0e-12,
            "sun heading off by {direction_error:e}"
        );
        // At 1 AU the readings are the cosines themselves, so the heading
        // state converges on the unit sun vector.
        let heading_error = (module.state.fixed_rows::<3>(0) - sun_direction_b).norm();
        assert!(
            heading_error <= 1.0e-9,
            "heading state off by {heading_error:e}"
        );
    }
}
