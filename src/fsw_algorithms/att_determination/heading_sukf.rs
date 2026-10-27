//! Heading switch square-root unscented Kalman filter: port of upstream
//! Basilisk `headingSuKF` (`fswAlgorithms/attDetermination/headingSuKF`).
//!
//! The filter estimates a body-frame heading `d_B` (for optical navigation,
//! the direction from the spacecraft to the planet) together with the two
//! body-rate components perpendicular to it. Those rates are expressed in a
//! "switch" frame `S` whose first axis is
//! `d` and whose second is `d x b` for a body axis `b`; when `d` comes within
//! 30 deg of `b` (`|b . d_hat| > 0.866`) the filter switches between `b_1` and
//! `b_2` and rotates the rate states and covariance into the new frame. The
//! measurement is the unit vector opposite the [`OpNavMsg`] `r_BN_B`, with the
//! OpNav covariance divided by `max(|r_BN_B|, 1)^2`.
//!
//! The published filter state negates the internal rate states, so that it
//! holds the `S`-frame components 2 and 3 of `omega_BN`. The OpNav output
//! carries the heading scaled back to the measured range, in the body frame
//! and, when a camera configuration is connected, in the camera frame.
//!
//! # Message write times
//!
//! Upstream timestamps a measurement with its message's write time and skips
//! messages that were never written. The Rust ports carry neither, so an
//! OpNav message counts as written at the first update that reads a changed
//! payload (`WriteTracker`). This is upstream's write time whenever the
//! producer runs ahead of the filter in the same step. As upstream, a
//! measurement written at the filter's current time is processed again on the
//! next update.

use nalgebra::{DMatrix, Matrix3, SMatrix, SVector, Vector3};

use crate::fsw_algorithms::att_determination::inertial_ukf::{
    SigmaWeights, WriteTracker, to_dynamic, to_dynamic_vector, to_static,
};
use crate::messages::{
    CameraConfigMsg, HEAD_N_STATES_SWITCH, HeadingFilterMsg, Input, OPNAV_MEAS, OpNavMsg, Output,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::{is_equal_within, is_zero_within, normalize_or_zero};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::utilities::ukf_utilities::{
    UkfResultExt, ukf_chol_decomp, ukf_chol_down_date, ukf_l_inv, ukf_qrd_just_r, ukf_u_inv,
};
use crate::{Module, SimulationContext};

/// Number of sigma points, upstream `2 * countHalfSPs + 1`.
const NUM_SIGMA: usize = 2 * HEAD_N_STATES_SWITCH + 1;
/// Upstream `switchTresh`, which `Reset` always sets to this value: the
/// `|b . d_hat|` above which the filter switches body axis.
const SWITCH_THRESHOLD: f64 = 0.866;
/// `|d x b|` below which upstream `headingSuKFComputeDCM_BS` falls back to the
/// identity.
const DEGENERATE_FRAME_NORM: f64 = 1.0e-5;
/// Observations smaller than this are treated as "no measurement this step"
/// when computing the post-fit residuals.
const ZERO_OBSERVATION: f64 = 1.0e-10;

type StateVector = SVector<f64, HEAD_N_STATES_SWITCH>;
type StateMatrix = SMatrix<f64, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH>;

/// User-settable parameters of [`HeadingSukf`] (upstream `HeadingSuKFConfig`
/// fields set from Python). Every default is upstream's zero initialization.
#[derive(Clone, Debug, Default)]
pub struct HeadingSukfConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `alpha` [-]: sigma-point spread.
    pub alpha: f64,
    /// Upstream `beta` [-]: prior-distribution parameter (2 for Gaussian).
    pub beta: f64,
    /// Upstream `kappa` [-]: secondary sigma-point scaling.
    pub kappa: f64,
    /// Upstream `noiseSF` [-]: scale applied to the sigma-point rows of the
    /// measurement square-root covariance (not to the OpNav noise). Upstream
    /// scenarios leave it at zero.
    pub noise_sf: f64,
    /// Upstream `qObsVal` [-]: accepted for parity; upstream never reads it.
    pub q_obs_val: f64,
    /// Upstream `sensorUseThresh` [-]: accepted for parity; upstream never
    /// reads it.
    pub sensor_use_thresh: f64,
    /// Upstream `stateInit` [-]: initial `[d_B, internal S-frame rates]`.
    pub state_init: StateVector,
    /// Upstream `covarInit` [-]: initial covariance.
    pub covar_init: StateMatrix,
    /// Upstream `qNoise` [-]: process noise (not scaled with the time step).
    pub q_noise: StateMatrix,
}

/// Upstream `headingSuKF`: heading and perpendicular-rate estimate from
/// optical-navigation measurements.
#[derive(Clone, Debug)]
pub struct HeadingSukf {
    /// User-settable filter parameters.
    pub config: HeadingSukfConfig,
    /// Heading estimate scaled to the measured range, in the OpNav format.
    pub opnav_data_out_msg: Output<OpNavMsg>,
    /// Filter state, covariance and post-fit residuals.
    pub filt_data_out_msg: Output<HeadingFilterMsg>,
    /// Heading measurements.
    pub opnav_data_in_msg: Input<OpNavMsg>,
    /// Optional: when connected, the OpNav output is also given in the camera
    /// frame.
    pub camera_config_in_msg: Input<CameraConfigMsg>,

    put_in_camera_frame: bool,
    weights: SigmaWeights<NUM_SIGMA>,
    /// Upstream `rNorm`: measured range, at least 1.
    r_norm: f64,
    time_tag_s: f64,
    /// Upstream `bVec_B`: the body axis the switch frame is built from.
    b_vec_b: Vector3<f64>,
    state: StateVector,
    s_bar: StateMatrix,
    covar: StateMatrix,
    x_bar: StateVector,
    obs: Vector3<f64>,
    /// Upstream `yMeas`: one predicted measurement per sigma point (column).
    y_meas: SMatrix<f64, OPNAV_MEAS, NUM_SIGMA>,
    post_fits: Vector3<f64>,
    /// Upstream `SP`: one sigma point per column.
    sigma_points: SMatrix<f64, HEAD_N_STATES_SWITCH, NUM_SIGMA>,
    /// Upstream `sQnoise`: the transposed (upper) Cholesky factor of `qNoise`.
    s_q_noise: StateMatrix,
    opnav_in_buffer: OpNavMsg,
    opnav_writes: WriteTracker<OpNavMsg>,
}

impl HeadingSukf {
    /// Builds the filter with unconnected ports; `init` (or `reset`) starts
    /// it from `config`.
    pub fn new(config: HeadingSukfConfig) -> Self {
        Self {
            config,
            opnav_data_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            opnav_data_in_msg: Input::default(),
            camera_config_in_msg: Input::default(),
            put_in_camera_frame: false,
            weights: SigmaWeights::default(),
            r_norm: 0.0,
            time_tag_s: 0.0,
            b_vec_b: Vector3::zeros(),
            state: StateVector::zeros(),
            s_bar: StateMatrix::zeros(),
            covar: StateMatrix::zeros(),
            x_bar: StateVector::zeros(),
            obs: Vector3::zeros(),
            y_meas: SMatrix::zeros(),
            post_fits: Vector3::zeros(),
            sigma_points: SMatrix::zeros(),
            s_q_noise: StateMatrix::zeros(),
            opnav_in_buffer: OpNavMsg::default(),
            opnav_writes: WriteTracker::default(),
        }
    }

    /// Upstream `Reset_headingSuKF`: checks the inputs and restarts the filter
    /// from the initial state, building the switch frame on `b_1`.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.opnav_data_in_msg.is_connected() {
            log::error!("headingSuKF.opnav_data_in_msg is not connected");
        }
        self.put_in_camera_frame = self.camera_config_in_msg.is_connected();

        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.b_vec_b = Vector3::x();
        self.obs = Vector3::zeros();
        self.sigma_points = SMatrix::zeros();
        self.weights = SigmaWeights::new(self.config.alpha, self.config.beta, self.config.kappa);
        self.state = self.config.state_init;
        self.covar = self.config.covar_init;
        // Upstream ignores both decompositions' status.
        self.s_bar = to_static(&ukf_chol_decomp(&to_dynamic(&self.covar)).into_output());
        self.s_q_noise = to_static::<HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH>(
            &ukf_chol_decomp(&to_dynamic(&self.config.q_noise)).into_output(),
        )
        .transpose();
    }

    fn heading(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(0).into_owned()
    }

    /// Upstream `Update_headingSuKF`.
    fn update_filter(&mut self, call_time_nanos: u64) {
        self.obs = Vector3::zeros();
        self.post_fits = Vector3::zeros();
        self.opnav_in_buffer = self.opnav_data_in_msg.read();
        self.opnav_writes
            .observe(&self.opnav_in_buffer, call_time_nanos);
        let camera_config = if self.put_in_camera_frame {
            self.camera_config_in_msg.read()
        } else {
            CameraConfigMsg::default()
        };

        let heading_hat = normalize_or_zero(self.heading());
        if self.b_vec_b.dot(&heading_hat).abs() > SWITCH_THRESHOLD {
            self.switch_frame();
        }
        self.r_norm = self.opnav_in_buffer.position_bn_b_m.norm();
        if self.r_norm < 1.0 {
            self.r_norm = 1.0;
        }

        if let Some(written_nanos) = self.opnav_writes.written_nanos() {
            let measurement_time_s = written_nanos as f64 * NANO2SEC;
            if measurement_time_s >= self.time_tag_s && self.opnav_in_buffer.valid {
                self.time_update(measurement_time_s);
                self.meas_update();
            }
        }
        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.time_tag_s {
            self.time_update(call_time_s);
        }

        // `y_meas` still holds this step's measurement model when there was a
        // measurement; otherwise `obs` is zero and the residuals stay zero.
        let y_bar = self.mean_measurement();
        if !is_zero_within(&self.obs, ZERO_OBSERVATION) {
            self.post_fits = self.obs - y_bar;
        }

        let mut state_bn = self.state;
        let rates_bn = -state_bn.fixed_rows::<2>(3).into_owned();
        state_bn.fixed_rows_mut::<2>(3).copy_from(&rates_bn);
        self.filt_data_out_msg.write(HeadingFilterMsg {
            time_tag_s: self.time_tag_s,
            covar: self.covar,
            state: state_bn,
            // Upstream leaves `stateError` uninitialized; it is zero here.
            state_error: StateVector::zeros(),
            post_fit_res: self.post_fits,
        });
        self.opnav_data_out_msg
            .write(self.opnav_output(&state_bn, &camera_config));
    }

    /// The heading estimate in the OpNav message format.
    fn opnav_output(&self, state_bn: &StateVector, camera_config: &CameraConfigMsg) -> OpNavMsg {
        // Upstream bug kept: `RECAST3X3` reads the first nine entries of the
        // flat row-major 5x5 covariance, not its leading 3x3 block.
        let covar_row_major = self.covar.transpose();
        let covar_b = Matrix3::from_row_slice(&covar_row_major.as_slice()[..9]);
        let position_bn_b =
            normalize_or_zero(state_bn.fixed_rows::<3>(0).into_owned()) * -self.r_norm;
        let mut output = OpNavMsg {
            time_tag_s: self.opnav_in_buffer.time_tag_s,
            valid: self.opnav_in_buffer.valid,
            covar_b_m2: covar_b,
            position_bn_b_m: position_bn_b,
            ..OpNavMsg::default()
        };
        if self.put_in_camera_frame {
            let dcm_cb = mrp_to_dcm(camera_config.sigma_cb);
            // Quirk kept: upstream maps the position with [CB]^T, while the
            // covariance gets [CB] P [CB]^T.
            output.position_bn_c_m = dcm_cb.transpose() * position_bn_b;
            output.covar_c_m2 = dcm_cb * covar_b * dcm_cb.transpose();
        }
        output
    }

    /// Weighted mean of the predicted measurements (upstream `yBar`).
    fn mean_measurement(&self) -> Vector3<f64> {
        let mut y_bar = Vector3::zeros();
        for (index, predicted) in self.y_meas.column_iter().enumerate() {
            y_bar += predicted * self.weights.w_m[index];
        }
        y_bar
    }

    /// Upstream `headingSuKFSwitch`: moves the switch frame to the other body
    /// axis (`b_1` <-> `b_2`) and rotates the rate states and the covariance
    /// into it. The square-root covariance is left as it was, as upstream.
    fn switch_frame(&mut self) {
        let heading_hat = normalize_or_zero(self.heading());
        let dcm_bs_old = dcm_bs(heading_hat, self.b_vec_b);
        let new_axis = if is_equal_within(&self.b_vec_b, &Vector3::x(), 1.0e-10) {
            Vector3::y()
        } else {
            Vector3::x()
        };
        let dcm_bs_new = dcm_bs(heading_hat, new_axis);
        self.b_vec_b = new_axis;

        let dcm_snew_sold = dcm_bs_new.transpose() * dcm_bs_old;
        let mut switch = StateMatrix::identity();
        switch
            .fixed_view_mut::<2, 2>(3, 3)
            .copy_from(&dcm_snew_sold.fixed_view::<2, 2>(1, 1));
        self.state = switch * self.state;
        self.covar = switch * self.covar * switch.transpose();
    }

    /// Upstream `headingSuKFTimeUpdate`: propagates the sigma points to
    /// `update_time_s` and rebuilds the square-root covariance.
    fn time_update(&mut self, update_time_s: f64) {
        let dt_s = update_time_s - self.time_tag_s;

        let mut center = self.state;
        heading_state_prop(&mut center, self.b_vec_b, dt_s);
        self.sigma_points.set_column(0, &center);
        let SigmaWeights {
            gamma, w_m, w_c, ..
        } = self.weights;
        self.x_bar = center * w_m[0];
        // Upstream visits the +/- pair of each sBar column together, which
        // fixes the summation order of xBar.
        for column in 0..HEAD_N_STATES_SWITCH {
            for (index, spread) in [
                (column + 1, gamma),
                (column + 1 + HEAD_N_STATES_SWITCH, -gamma),
            ] {
                let mut point = self.s_bar.column(column) * spread + self.state;
                heading_state_prop(&mut point, self.b_vec_b, dt_s);
                self.sigma_points.set_column(index, &point);
                self.x_bar = point * w_m[index] + self.x_bar;
            }
        }

        let mut stacked = DMatrix::zeros(
            2 * HEAD_N_STATES_SWITCH + HEAD_N_STATES_SWITCH,
            HEAD_N_STATES_SWITCH,
        );
        for index in 1..NUM_SIGMA {
            let deviation = (self.sigma_points.column(index) - self.x_bar) * w_c[index].sqrt();
            stacked.row_mut(index - 1).copy_from(&deviation.transpose());
        }
        stacked
            .view_mut(
                (2 * HEAD_N_STATES_SWITCH, 0),
                (HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH),
            )
            .copy_from(&self.s_q_noise);
        let s_bar = ukf_qrd_just_r(&stacked).transpose();

        let center_error = center - self.x_bar;
        self.s_bar = to_static(
            &ukf_chol_down_date(&s_bar, &to_dynamic_vector(&center_error), w_c[0]).into_output(),
        );
        self.covar = self.s_bar * self.s_bar.transpose();
        self.state = center;
        self.time_tag_s = update_time_s;
    }

    /// Upstream `headingSuKFMeasModel`: the measurement is the unit vector
    /// opposite `r_BN_B`, predicted by each sigma point's heading.
    fn meas_model(&mut self) {
        self.obs = normalize_or_zero(self.opnav_in_buffer.position_bn_b_m) * -1.0;
        self.y_meas = self.sigma_points.fixed_rows::<OPNAV_MEAS>(0).into_owned();
    }

    /// Upstream `headingSuKFMeasUpdate`: applies the OpNav heading to the
    /// state and square-root covariance. Like upstream, failures of the
    /// square-root routines are ignored and their partial results used.
    fn meas_update(&mut self) {
        self.meas_model();
        let y_bar = self.mean_measurement();
        let w_c = self.weights.w_c;

        let mut stacked = DMatrix::zeros(2 * HEAD_N_STATES_SWITCH + OPNAV_MEAS, OPNAV_MEAS);
        for index in 1..NUM_SIGMA {
            let deviation = (self.y_meas.column(index) - y_bar) * w_c[index].sqrt();
            stacked.row_mut(index - 1).copy_from(&deviation.transpose());
        }
        let q_obs = self.opnav_in_buffer.covar_b_m2 * (1.0 / (self.r_norm * self.r_norm));
        // Upstream stacks the lower Cholesky factor itself (not its
        // transpose), which only matters for a non-diagonal noise.
        let q_chol = ukf_chol_decomp(&to_dynamic(&q_obs)).into_output();
        stacked
            .view_mut((2 * HEAD_N_STATES_SWITCH, 0), (OPNAV_MEAS, OPNAV_MEAS))
            .copy_from(&q_chol);
        stacked
            .rows_mut(0, 2 * HEAD_N_STATES_SWITCH)
            .scale_mut(self.config.noise_sf);
        let sy = ukf_qrd_just_r(&stacked).transpose();
        let center_deviation = self.y_meas.column(0) - y_bar;
        let sy: Matrix3<f64> = to_static(
            &ukf_chol_down_date(&sy, &to_dynamic_vector(&center_deviation), w_c[0]).into_output(),
        );

        let mut p_xy = SMatrix::<f64, HEAD_N_STATES_SWITCH, OPNAV_MEAS>::zeros();
        for index in 0..NUM_SIGMA {
            let measurement_deviation = self.y_meas.column(index) - y_bar;
            let state_deviation = (self.sigma_points.column(index) - self.x_bar) * w_c[index];
            p_xy += state_deviation * measurement_deviation.transpose();
        }

        let sy_t_inv: Matrix3<f64> =
            to_static(&ukf_u_inv(&to_dynamic(&sy.transpose())).into_output());
        let sy_inv: Matrix3<f64> = to_static(&ukf_l_inv(&to_dynamic(&sy)).into_output());
        let gain = p_xy * sy_t_inv * sy_inv;

        let innovation = self.obs - y_bar;
        self.state += gain * innovation;

        let update = gain * sy;
        for column in update.column_iter() {
            self.s_bar = to_static(
                &ukf_chol_down_date(
                    &to_dynamic(&self.s_bar),
                    &to_dynamic_vector(&column.into_owned()),
                    -1.0,
                )
                .into_output(),
            );
        }
        self.covar = self.s_bar * self.s_bar.transpose();
    }
}

/// Upstream `headingSuKFComputeDCM_BS`: `[BS]`, whose columns are the switch
/// frame axes `d_hat`, `(d x b)_hat` and `(d x s_2)_hat`, or the identity when
/// `d` is (nearly) along `b`.
fn dcm_bs(heading: Vector3<f64>, b_vec: Vector3<f64>) -> Matrix3<f64> {
    let s2 = heading.cross(&b_vec);
    if s2.norm() < DEGENERATE_FRAME_NORM {
        return Matrix3::identity();
    }
    let s1 = normalize_or_zero(heading);
    let s2 = normalize_or_zero(s2);
    let s3 = normalize_or_zero(heading.cross(&s2));
    Matrix3::from_columns(&[s1, s2, s3])
}

/// Upstream `headingStateProp`: rotates the heading by the body rate built
/// from the two (negated) switch-frame rate states; the rates are constant.
fn heading_state_prop(state: &mut StateVector, b_vec: Vector3<f64>, dt_s: f64) {
    let heading = state.fixed_rows::<3>(0).into_owned();
    let omega_bn_s = Vector3::new(0.0, -state[3], -state[4]);
    let omega_bn_b = dcm_bs(heading, b_vec) * omega_bn_s;
    let propagated = omega_bn_b.cross(&heading) * -dt_s;
    state
        .fixed_rows_mut::<3>(0)
        .copy_from(&(heading + propagated));
}

impl Module for HeadingSukf {
    fn init(&mut self) {
        self.opnav_data_out_msg.write(OpNavMsg::default());
        self.filt_data_out_msg.write(HeadingFilterMsg::default());
        HeadingSukf::reset(self, 0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        HeadingSukf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.update_filter(context.current_sim_nanos);
    }
}

#[cfg(test)]
mod tests {
    //! Ports `headingSuKF/_UnitTest/test_headingSuKF.py`. `test_all_heading_kf`
    //! runs `StatePropSunLine` and `StateUpdateSunLine`; `test_functions_ukf`
    //! only exercises `ukfUtilities` (QR, LU, inverses, Cholesky) through this
    //! module's SWIG wrapper, and those checks are ported once in
    //! `crate::utilities::ukf_utilities`.
    //!
    //! `StateUpdateSunLine` omits the numpy measurement noise. Upstream's
    //! `inputData.r_BN_B += np.random.normal(0, 0.001, 3)` extends the
    //! three-element list SWIG returns to six elements, which the array setter
    //! rejects ("Size mismatch"), so that line cannot run as written. Upstream
    //! leaves `noiseSF` at zero, and with it the scenario diverges after its
    //! first measurement: the state is NaN by the end, and the checks hold only
    //! because comparisons with NaN are false. The port keeps those checks with
    //! the same semantics (including the `stateTarget[i - 1]` indexing) and
    //! checks the convergence the test description intends with
    //! `noise_sf = 1`.
    //!
    //! Python writes each OpNav sample between ticks, stamped with the previous
    //! tick's time, so upstream applies it once, at that time. Here a sample
    //! counts as written at the tick that first reads it (see the module doc),
    //! which is upstream's behaviour for a producer that runs ahead of the
    //! filter in the same tick: the sample is applied at that tick and again
    //! at the next one. The divergence is pinned to values from upstream
    //! `headingSuKF.c` (built with gcc) driven with those write times, which
    //! this port reproduces bit for bit. With Python's write times upstream
    //! diverges as well (|state| ~ 1e12 at t = 10.5 s, NaN by t = 495 s).

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{HeadingSukf, HeadingSukfConfig, StateMatrix, StateVector, dcm_bs};
    use crate::messages::{CameraConfigMsg, HeadingFilterMsg, OpNavMsg, Output};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    const TASK_NANOS: u64 = 500_000_000;
    /// Upstream `t1`: loop iterations per phase.
    const T1: u64 = 1000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `macros.sec2nano`.
    fn sec2nano(time_s: f64) -> u64 {
        (time_s * 1.0e9 + 0.5) as u64
    }

    /// Upstream `setupFilterData`.
    fn setup_filter_data() -> HeadingSukfConfig {
        let mut covar_init = StateMatrix::zeros();
        let mut q_noise = StateMatrix::zeros();
        for index in 0..3 {
            covar_init[(index, index)] = 0.1;
            q_noise[(index, index)] = 0.01 * 0.01;
        }
        for index in 3..5 {
            covar_init[(index, index)] = 0.01;
            q_noise[(index, index)] = 0.001 * 0.001;
        }
        HeadingSukfConfig {
            name: "headingSuKF".to_string(),
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state_init: StateVector::from_column_slice(&[0.0, 0.0, 1.0, 0.0, 0.0]),
            covar_init,
            q_noise,
            ..HeadingSukfConfig::default()
        }
    }

    /// One Python loop iteration: `ExecuteSimulation` up to
    /// `(iteration + 1) * 0.5 s` (which also runs the t = 0 tick first),
    /// recording the filter message after every update like
    /// `filtDataOutMsg.recorder()`.
    fn execute_iteration(
        module: &mut HeadingSukf,
        iteration: u64,
        log: &mut Vec<HeadingFilterMsg>,
    ) {
        let first_tick = if iteration == 0 { 0 } else { iteration + 1 };
        for tick in first_tick..=iteration + 1 {
            module.update(&context(tick * TASK_NANOS));
            log.push(module.filt_data_out_msg.read());
        }
    }

    /// Upstream's failure test, `difference > tolerance`, which a NaN
    /// difference never meets.
    fn upstream_flags_failure(difference: f64, tolerance: f64) -> bool {
        difference > tolerance
    }

    #[test]
    fn matches_upstream_state_prop_sun_line() {
        let mut module = HeadingSukf::new(setup_filter_data());
        // Upstream writes a zeroed (invalid) OpNav message before the run.
        Output::new(OpNavMsg::default()).connect_to(&mut module.opnav_data_in_msg);
        module.init();
        let mut log = Vec::new();
        for tick in 0..=16_000 {
            module.update(&context(tick * TASK_NANOS));
            log.push(module.filt_data_out_msg.read());
        }
        let (first, last) = (&log[0], &log[log.len() - 1]);
        for index in 0..5 {
            assert!((last.state[index] - first.state[index]).abs() <= 1.0e-10);
        }
    }

    /// Upstream `StateUpdateSunLine` with the given `noise_sf`: returns the
    /// recorded filter messages and the two measured headings.
    fn run_state_update_sun_line(noise_sf: f64) -> (Vec<HeadingFilterMsg>, [Vector3<f64>; 2]) {
        let mut config = setup_filter_data();
        config.state_init = StateVector::from_column_slice(&[1.0, 0.2, 0.1, 0.01, 0.001]);
        config.noise_sf = noise_sf;
        let mut module = HeadingSukf::new(config);
        let opnav = Output::new(OpNavMsg::default());
        opnav.connect_to(&mut module.opnav_data_in_msg);
        module.init();

        let test_vectors = [Vector3::new(0.9, 0.1, 0.02), Vector3::new(0.6, -0.1, 0.2)];
        let mut input = OpNavMsg::default();
        let mut log = Vec::new();
        for iteration in 0..2 * T1 {
            let (phase, step) = (iteration / T1, iteration % T1);
            let write = if phase == 0 {
                step > 0 && step % 20 == 0
            } else {
                step % 20 == 0
            };
            if write {
                let time_s = if phase == 0 {
                    step as f64 * 0.5
                } else {
                    step as f64 * 0.5 + T1 as f64 + 1.0
                };
                // Upstream stores nanoseconds in this seconds field.
                input.time_tag_s = sec2nano(time_s) as f64;
                input.valid = true;
                input.position_bn_b_m = test_vectors[phase as usize];
                input.covar_b_m2 = Matrix3::from_diagonal_element(0.0001 * 0.0001);
                opnav.write(input.clone());
            }
            execute_iteration(&mut module, iteration, &mut log);
        }
        (log, test_vectors)
    }

    /// Upstream `stateTarget`: the unit vector opposite `r_BN_B`, then zero
    /// rates.
    fn state_target(test_vector: Vector3<f64>) -> [f64; 5] {
        let heading = -test_vector / test_vector.norm();
        [heading.x, heading.y, heading.z, 0.0, 0.0]
    }

    #[test]
    fn matches_upstream_state_update_sun_line() {
        let (log, test_vectors) = run_state_update_sun_line(0.0);
        let t1 = T1 as usize;
        for (phase, (covar_sample, state_sample)) in [(t1 - 10, t1), (2 * t1 - 10, 2 * t1)]
            .into_iter()
            .enumerate()
        {
            let target = state_target(test_vectors[phase]);
            for index in 0..5 {
                let covar_change = (log[covar_sample].covar[(index, index)]
                    - log[0].covar[(index, index)] / 10.0)
                    .abs();
                assert!(!upstream_flags_failure(covar_change, 1.0e-1));
                // Upstream compares state[i] with stateTarget[i - 1].
                let state_error = (log[state_sample].state[index] - target[(index + 4) % 5]).abs();
                assert!(!upstream_flags_failure(state_error, 1.0e-1));
            }
        }

        // Compiled upstream, with the write times above: the first measurement
        // (t = 10.5 s) and its repetition on the next step (the write time
        // equals the filter time) make the filter diverge.
        let reference = [
            (
                21,
                [
                    1.018_131_390_496_01,
                    0.110_245_728_490_325_81,
                    0.042_558_831_531_359_52,
                    0.008_443_899_865_456_352,
                    0.005_449_821_562_413_418,
                ],
            ),
            (
                22,
                [
                    -173_714_566_220.805_73,
                    1_555_669_880_318.196_3,
                    -6_995_089_794_431.161,
                    -60_194.176_675_357_24,
                    -270_024.577_242_523_3,
                ],
            ),
        ];
        for (sample, state) in reference {
            for (index, expected) in state.into_iter().enumerate() {
                let actual = log[sample].state[index];
                assert!(
                    (actual - expected).abs() <= 1.0e-9 * expected.abs(),
                    "sample {sample} state[{index}]: {actual:e} differs from {expected:e}"
                );
            }
        }
        let post_fits = [
            -1.908_817_874_669_957_8,
            -0.212_283_667_542_948_36,
            -0.058_402_825_100_749_58,
        ];
        for (actual, expected) in log[21].post_fit_res.iter().zip(post_fits) {
            assert!((actual - expected).abs() <= 1.0e-9 * expected.abs());
        }
        assert!(log[t1].state.iter().all(|value| value.is_nan()));
        assert!(log[2 * t1].state.iter().all(|value| value.is_nan()));
    }

    #[test]
    fn state_update_sun_line_converges_with_a_unit_noise_scale() {
        let (log, test_vectors) = run_state_update_sun_line(1.0);
        let t1 = T1 as usize;
        for (phase, (covar_sample, state_sample)) in [(t1 - 10, t1), (2 * t1 - 10, 2 * t1)]
            .into_iter()
            .enumerate()
        {
            let target = state_target(test_vectors[phase]);
            for (index, expected) in target.into_iter().enumerate() {
                let covar_change = (log[covar_sample].covar[(index, index)]
                    - log[0].covar[(index, index)] / 10.0)
                    .abs();
                assert!(covar_change <= 1.0e-1);
                let actual = log[state_sample].state[index];
                assert!(
                    (actual - expected).abs() <= 1.0e-1,
                    "phase {phase} state[{index}] = {actual} misses {expected}"
                );
            }
        }
    }

    #[test]
    fn switching_frames_preserves_the_body_rate() {
        let mut module = HeadingSukf::new(setup_filter_data());
        module.init();
        module.state = StateVector::from_column_slice(&[0.9, 0.3, -0.2, 0.01, -0.02]);
        module.covar = StateMatrix::from_diagonal(&StateVector::from_column_slice(&[
            0.1, 0.2, 0.3, 0.01, 0.02,
        ]));
        let heading = module.heading();
        let body_rate = |module: &HeadingSukf| {
            dcm_bs(module.heading(), module.b_vec_b)
                * Vector3::new(0.0, -module.state[3], -module.state[4])
        };
        let rate_before = body_rate(&module);
        let heading_covar_before = module.covar.fixed_view::<3, 3>(0, 0).into_owned();

        module.switch_frame();

        assert_eq!(module.b_vec_b, Vector3::y());
        assert_eq!(module.heading(), heading);
        assert!((body_rate(&module) - rate_before).norm() <= 1.0e-15);
        assert_eq!(
            module.covar.fixed_view::<3, 3>(0, 0).into_owned(),
            heading_covar_before
        );
        module.switch_frame();
        assert_eq!(module.b_vec_b, Vector3::x());
        assert!((body_rate(&module) - rate_before).norm() <= 1.0e-15);
    }

    #[test]
    fn opnav_output_keeps_the_upstream_layout_quirks() {
        let mut module = HeadingSukf::new(setup_filter_data());
        let sigma_cb = Vector3::new(0.1, -0.2, 0.05);
        Output::new(CameraConfigMsg {
            sigma_cb,
            ..CameraConfigMsg::default()
        })
        .connect_to(&mut module.camera_config_in_msg);
        Output::new(OpNavMsg::default()).connect_to(&mut module.opnav_data_in_msg);
        module.init();
        module.covar = StateMatrix::from_fn(|row, column| (row * 5 + column) as f64);
        module.r_norm = 2.5;
        let state_bn = StateVector::from_column_slice(&[3.0, 0.0, 4.0, 0.1, 0.2]);
        let camera = CameraConfigMsg {
            sigma_cb,
            ..CameraConfigMsg::default()
        };

        let output = module.opnav_output(&state_bn, &camera);

        // The first nine row-major entries of the 5x5 covariance.
        let covar_b = Matrix3::new(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
        assert_eq!(output.covar_b_m2, covar_b);
        let position_b = Vector3::new(-1.5, 0.0, -2.0);
        assert!((output.position_bn_b_m - position_b).norm() <= 1.0e-15);
        let dcm_cb = mrp_to_dcm(sigma_cb);
        assert!((output.position_bn_c_m - dcm_cb.transpose() * position_b).norm() <= 1.0e-15);
        assert!((output.covar_c_m2 - dcm_cb * covar_b * dcm_cb.transpose()).norm() <= 1.0e-12);
        assert_eq!(output.covar_n_m2, Matrix3::zeros());
    }
}
