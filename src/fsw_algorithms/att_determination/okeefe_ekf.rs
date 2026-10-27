//! O'Keefe sunline extended Kalman filter: estimates the sun heading `d` in
//! the body frame from the coarse-sun-sensor (CSS) constellation readings,
//! without gyros. Port of upstream Basilisk `okeefeEKF`
//! (`fswAlgorithms/attDetermination/okeefeEKF`).
//!
//! The three states are the heading alone. The body rate `ω` that carries the
//! heading forward is not estimated but inferred from the two latest headings
//! (`ω = acos(d̂ₖ · d̂ₖ₋₁) d̂ₖ × d̂ₖ₋₁ / dt`), and the heading propagates as
//! `ḋ = -ω × d`. The measurement update is [`super::sunline_ekf`]'s, on three
//! states: extended unless the largest covariance magnitude exceeds
//! `ekf_switch`, classical otherwise.
//!
//! As in [`super::sunline_ekf`], a connected `css_data_in_msg` counts as
//! written and its `time_tag_s` stands for upstream's header write time, with
//! the same two exceptions (a producer that leaves `time_tag_s` at zero, and
//! a producer that has not written yet at 0 s).

use nalgebra::{Matrix3, SVector, Vector3};

use super::sunline_ekf::{
    CssObservations, ckf_update, configured_css_units, ekf_update, kalman_gain,
    needs_classical_update,
};
use crate::messages::{
    CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, Input, MAX_N_CSS_MEAS,
    NavigationAttitudeMsg, Output, SKF_N_STATES_HALF, SunlineFilterMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::{is_zero_within, normalize_or_zero, safe_acos};
use crate::{Module, SimulationContext};

/// Time step \[s\] below which upstream sets the inferred rate to zero.
const MIN_RATE_DT_S: f64 = 1.0e-10;

/// Magnitude at or below which every component of the previous heading counts
/// as zero, so that no rate can be inferred from it.
const ZERO_HEADING_TOLERANCE: f64 = 1.0e-10;

/// User-settable parameters of [`OkeefeEkf`]. Upstream zero-initializes all
/// of them.
#[derive(Clone, Debug)]
pub struct OkeefeEkfConfig {
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
}

impl Default for OkeefeEkfConfig {
    fn default() -> Self {
        Self {
            name: "okeefeEKF".to_string(),
            q_obs_val: 0.0,
            q_proc_val: 0.0,
            sensor_use_thresh: 0.0,
            ekf_switch: 0.0,
        }
    }
}

/// Gyro-less CSS sunline extended Kalman filter (upstream `okeefeEKF`).
///
/// Seed `state`, `omega`, `state_error` and `covar` before `init`, as
/// upstream's Python sets `state`, `omega`, `x` and `covar`; Reset leaves them
/// untouched.
#[derive(Clone, Debug)]
pub struct OkeefeEkf {
    /// User-settable parameters.
    pub config: OkeefeEkfConfig,
    /// Upstream `cssDataInMsg`: the CSS cosine readings.
    pub css_data_in_msg: Input<CssArraySensorMsg>,
    /// Upstream `cssConfigInMsg`: the CSS normals and biases, read at Reset.
    pub css_config_in_msg: Input<CssConfigMsg>,
    /// Upstream `navStateOutMsg`: the unit sun heading and the filter time.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `filtDataOutMsg`: the three-state estimate packed into the
    /// six-state filter message.
    pub filt_data_out_msg: Output<SunlineFilterMsg>,
    /// Upstream `state`: reference sun heading `d` at the filter time.
    pub state: Vector3<f64>,
    /// Upstream `omega` \[rad/s\]: body rate inferred from the last two
    /// headings, used by the next propagation.
    pub omega: Vector3<f64>,
    /// Upstream `x`: state error relative to `state`.
    pub state_error: Vector3<f64>,
    /// Upstream `covar`: state covariance.
    pub covar: Matrix3<f64>,
    time_tag_s: f64,
    prev_states: Vector3<f64>,
    x_bar: Vector3<f64>,
    covar_bar: Matrix3<f64>,
    state_transition: Matrix3<f64>,
    proc_noise: Matrix3<f64>,
    css_units: Vec<CssUnitConfigMsg>,
    observations: CssObservations<SKF_N_STATES_HALF>,
    post_fits: SVector<f64, MAX_N_CSS_MEAS>,
}

impl OkeefeEkf {
    /// A filter with unconnected inputs and a zero estimate; seed `state`,
    /// `omega`, `state_error` and `covar` before `init`.
    pub fn new(config: OkeefeEkfConfig) -> Self {
        Self {
            config,
            css_data_in_msg: Input::default(),
            css_config_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            state: Vector3::zeros(),
            omega: Vector3::zeros(),
            state_error: Vector3::zeros(),
            covar: Matrix3::zeros(),
            time_tag_s: 0.0,
            prev_states: Vector3::zeros(),
            x_bar: Vector3::zeros(),
            covar_bar: Matrix3::zeros(),
            state_transition: Matrix3::zeros(),
            proc_noise: Matrix3::zeros(),
            css_units: Vec::new(),
            observations: CssObservations::unprocessed(),
            post_fits: SVector::zeros(),
        }
    }

    /// Upstream `Reset_okeefeEKF`: caches the CSS configuration and restarts
    /// the filter bookkeeping at `call_time_nanos`. The estimate (`state`,
    /// `omega`, `state_error`, `covar`) and the post-fit residuals are kept,
    /// as upstream.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.css_config_in_msg.is_connected() {
            log::error!("okeefeEKF.css_config_in_msg is not connected");
        }
        if !self.css_data_in_msg.is_connected() {
            log::error!("okeefeEKF.css_data_in_msg is not connected");
        }
        self.css_units = configured_css_units(&self.css_config_in_msg.read(), "okeefeEKF");

        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.observations = CssObservations::unprocessed();
        self.x_bar = Vector3::zeros();
        self.prev_states = Vector3::zeros();
        self.covar_bar = Matrix3::zeros();
        self.state_transition = Matrix3::identity();
        self.proc_noise = self.config.q_proc_val * Matrix3::identity();
    }

    fn update_filter(&mut self, call_time_nanos: u64) {
        let css_data = self.css_data_in_msg.read();
        // Upstream: `timeWritten * NANO2SEC >= timeTag && isWritten`.
        if self.css_data_in_msg.is_connected() && css_data.time_tag_s >= self.time_tag_s {
            self.time_update(css_data.time_tag_s);
            self.measurement_update(&css_data.cos_values);
        }

        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.time_tag_s {
            self.time_update(call_time_s);
            self.state_error = self.x_bar;
            self.covar = self.covar_bar;
        }

        self.observations
            .write_post_fit_residuals(&self.state_error, &mut self.post_fits);
    }

    /// Upstream `sunlineTimeUpdate` (okeefeEKF): propagates the heading with
    /// the current rate, infers the next rate, then propagates the state error
    /// and the covariance to `update_time_s`.
    fn time_update(&mut self, update_time_s: f64) {
        let dt = update_time_s - self.time_tag_s;
        let dyn_mat = dynamics_matrix(&self.omega);
        self.prev_states = self.state;
        self.state_transition = propagate_state(&mut self.state, &dyn_mat, &self.omega, dt);
        self.omega = heading_rate(&self.state, dt, &self.prev_states);
        self.x_bar = self.state_transition * self.state_error;

        let gamma = Matrix3::from_diagonal_element(dt * dt / 2.0);
        self.covar_bar = self.state_transition * (self.covar * self.state_transition.transpose())
            + gamma * (self.proc_noise * gamma.transpose());
        self.time_tag_s = update_time_s;
    }

    /// Upstream `sunlineMeasUpdate` (okeefeEKF).
    fn measurement_update(&mut self, cos_values: &[f64]) {
        self.observations = CssObservations::select(
            &self.state,
            cos_values,
            &self.css_units,
            self.config.sensor_use_thresh,
        );
        let CssObservations {
            y_meas, meas_mat, ..
        } = &self.observations;
        let q_obs_val = self.config.q_obs_val;
        let gain = kalman_gain(&self.covar_bar, meas_mat, q_obs_val);

        // As in sunlineEKF: a large covariance is first brought down with the
        // classical update.
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
            vehicle_sun_point_body: normalize_or_zero(self.state),
            ..Default::default()
        }
    }

    /// The three-state estimate in the six-state filter message: the leading
    /// entries of `state`/`state_error` and the packed covariance, the rest
    /// zero, as upstream.
    fn filter_data(&self) -> SunlineFilterMsg {
        let mut msg = SunlineFilterMsg {
            time_tag_s: self.time_tag_s,
            post_fit_res: self.post_fits,
            num_obs: self.observations.count(),
            ..Default::default()
        };
        msg.set_packed_covariance(&self.covar);
        msg.state
            .fixed_rows_mut::<SKF_N_STATES_HALF>(0)
            .copy_from(&self.state);
        msg.state_error
            .fixed_rows_mut::<SKF_N_STATES_HALF>(0)
            .copy_from(&self.state_error);
        msg
    }
}

impl Module for OkeefeEkf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
        self.filt_data_out_msg.write(SunlineFilterMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        OkeefeEkf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.update_filter(context.current_sim_nanos);
        self.nav_state_out_msg.write(self.nav_state());
        self.filt_data_out_msg.write(self.filter_data());
    }
}

/// Upstream `sunlineDynMatrixOkeefe`: the Jacobian `-[ω×]` of `ḋ = -ω × d`.
/// (Upstream also takes `dt`, unused.)
fn dynamics_matrix(omega: &Vector3<f64>) -> Matrix3<f64> {
    -omega.cross_matrix()
}

/// Upstream `sunlineStateSTMProp` (okeefeEKF): advances the heading by
/// `-dt ω × d` and returns the state transition matrix `I + dt A`. The caller
/// keeps the previous heading, which upstream copies here.
fn propagate_state(
    state: &mut Vector3<f64>,
    dyn_mat: &Matrix3<f64>,
    omega: &Vector3<f64>,
    dt: f64,
) -> Matrix3<f64> {
    *state += -dt * omega.cross(state);
    Matrix3::identity() + dt * dyn_mat
}

/// Upstream `sunlineRateCompute`: the rate `acos(d̂ₖ · d̂ₖ₋₁) d̂ₖ × d̂ₖ₋₁ / dt`
/// that turned the previous heading into the current one; zero for
/// `dt < 1e-10` or a zero previous heading.
fn heading_rate(states: &Vector3<f64>, dt: f64, prev_states: &Vector3<f64>) -> Vector3<f64> {
    if dt < MIN_RATE_DT_S || is_zero_within(prev_states, ZERO_HEADING_TOLERANCE) {
        return Vector3::zeros();
    }
    let heading_hat = normalize_or_zero(*states);
    let prev_heading_hat = normalize_or_zero(*prev_states);
    // `safe_acos` clamps exactly like upstream's explicit `> 1` / `< -1`
    // branches.
    (1.0 / dt * safe_acos(heading_hat.dot(&prev_heading_hat)))
        * heading_hat.cross(&prev_heading_hat)
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `_UnitTest/test_okeefeEKF.py`: `test_all_functions_oekf`
    //! (`sunline_individual_test`, one Rust test per helper it checks,
    //! `StatePropStatic`, `StatePropVariable`) and `test_all_sunline_oekf`
    //! (`StateUpdateSunLine`, all six parametrizations, including its
    //! issue-#1353 post-fit residual check). Truth values are recomputed here
    //! as the Python computes them, with upstream's tolerances.
    //! `SunLineOEKF_test_utilities.py` only draws figures and is not ported.
    //!
    //! The CSS input and the measurement noise deviate from upstream exactly as
    //! described in `sunline_ekf`'s tests, whose helpers these share.

    use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::Normal;

    use super::{OkeefeEkf, OkeefeEkfConfig, dynamics_matrix, heading_rate, propagate_state};
    use crate::Module;
    use crate::fsw_algorithms::att_determination::sunline_ekf::tests::{
        TASK_RATE_NANOS, context, css_constellation, css_reading, expected_joseph_covariance,
        individual_test_css, individual_test_y, padded, unwritten_css_data,
    };
    use crate::fsw_algorithms::att_determination::sunline_ekf::{
        CssObservations, KalmanGain, MeasurementMatrix, ckf_update, ekf_update, kalman_gain,
    };
    use crate::messages::{CssConfigMsg, MAX_N_CSS_MEAS, Output, SunlineFilterMsg};

    const DT_S: f64 = 0.5;

    /// Upstream `setupFilterData`.
    fn setup_filter_data() -> OkeefeEkf {
        let mut module = OkeefeEkf::new(OkeefeEkfConfig {
            name: "okeefeEKF".to_string(),
            sensor_use_thresh: 0.0,
            // Upstream `0.1**2`, which is 0.010000000000000002.
            q_proc_val: 0.1 * 0.1,
            q_obs_val: 0.001,
            ekf_switch: 5.0,
        });
        module.state = Vector3::new(1.0, 1.0, 1.0);
        module.omega = Vector3::new(0.1, 0.2, 0.1);
        module.state_error = Vector3::new(1.0, 0.0, 1.0);
        module.covar = Matrix3::from_diagonal_element(0.4);
        module
    }

    /// `expDynMat` as the Python writes it out for `omega`.
    fn expected_dynamics_matrix(omega: &Vector3<f64>) -> Matrix3<f64> {
        -Matrix3::new(
            0.0, -omega[2], omega[1], omega[2], 0.0, -omega[0], -omega[1], omega[0], 0.0,
        )
    }

    /// The Python's rate from two headings, `arccos` branches included.
    fn expected_rate(states: &Vector3<f64>, prev_states: &Vector3<f64>, dt: f64) -> Vector3<f64> {
        let norms = states.norm() * prev_states.norm();
        let arg = states.dot(prev_states) / norms;
        let angle = if arg > 1.0 {
            1.0_f64.acos()
        } else if arg < -1.0 {
            (-1.0_f64).acos()
        } else {
            arg.acos()
        };
        1.0 / dt * states.cross(prev_states) / norms * angle
    }

    /// Upstream's flat `h` (8 × 3, row-major) with the identity in the first
    /// three rows; only the first `numObs = 3` rows are used.
    fn individual_test_meas_mat() -> MeasurementMatrix<3> {
        MeasurementMatrix::from_fn(3, |row, col| if row == col { 1.0 } else { 0.0 })
    }

    /// Upstream's `covar` for the gain and update checks.
    fn individual_test_covar() -> Matrix3<f64> {
        Matrix3::new(1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0)
    }

    fn to_dmatrix<const R: usize, const C: usize>(m: &SMatrix<f64, R, C>) -> DMatrix<f64> {
        DMatrix::from_fn(R, C, |row, col| m[(row, col)])
    }

    #[test]
    fn matches_upstream_dynamics_matrix_check() {
        let input_omega = Vector3::new(0.1, 0.2, 0.1);
        let dyn_mat = dynamics_matrix(&input_omega);
        assert!((expected_dynamics_matrix(&input_omega) - dyn_mat).norm() <= 1.0e-12);
    }

    #[test]
    fn matches_upstream_omega_computation_check() {
        let input_states = Vector3::new(2.0, 1.0, 0.75);
        let input_prev_states = Vector3::new(1.0, 0.1, 0.5);
        let omega = heading_rate(&input_states, DT_S, &input_prev_states);
        let expected = expected_rate(&input_states, &input_prev_states, DT_S);
        assert!((expected - omega).norm() <= 1.0e-12);
    }

    #[test]
    fn rate_is_zero_without_a_time_step_or_a_previous_heading() {
        let states = Vector3::new(2.0, 1.0, 0.75);
        let prev_states = Vector3::new(1.0, 0.1, 0.5);
        assert_eq!(heading_rate(&states, 0.0, &prev_states), Vector3::zeros());
        assert_eq!(
            heading_rate(&states, DT_S, &Vector3::new(1.0e-11, 0.0, -1.0e-11)),
            Vector3::zeros()
        );
        // Parallel headings give no rotation.
        assert!(heading_rate(&states, DT_S, &(3.0 * states)).norm() <= 1.0e-12);
    }

    #[test]
    fn matches_upstream_state_and_stm_propagation_check() {
        let input_states = Vector3::new(2.0, 1.0, 0.75);
        let input_omega = Vector3::new(0.1, 0.2, 0.1);
        let exp_dyn_mat = expected_dynamics_matrix(&input_omega);
        let mut states = input_states;
        let stm = propagate_state(&mut states, &exp_dyn_mat, &input_omega, DT_S);

        let expected_stm = DT_S * exp_dyn_mat + Matrix3::identity();
        let expected_states = input_states - DT_S * input_omega.cross(&input_states);
        assert!((expected_stm - stm).norm() <= 1.0e-12);
        assert!((expected_states - states).norm() <= 1.0e-12);
    }

    #[test]
    fn matches_upstream_measurement_model_check() {
        let heading = Vector3::new(2.0, 1.0, 0.75);
        let (units, cos_values) = individual_test_css();
        let observations = CssObservations::<3>::select(
            &heading,
            &cos_values,
            &units,
            50.0_f64.to_radians().cos(),
        );

        let mut expected_h = DMatrix::zeros(8, 3);
        let mut expected_y = DVector::zeros(8);
        for j in 0..3 {
            expected_h[(j, j)] = 1.0;
            expected_y[j] = cos_values[j] - heading.dot(&units[j].n_hat_b);
        }
        let mut expected_obs = DVector::zeros(8);
        expected_obs
            .rows_mut(0, 3)
            .copy_from_slice(&cos_values[..3]);
        let h_out = DMatrix::from_fn(8, 3, |row, col| {
            if row < observations.count() {
                observations.meas_mat[(row, col)]
            } else {
                0.0
            }
        });

        assert!((h_out - expected_h).norm() <= 1.0e-12);
        assert!((padded(&observations.y_meas, 8) - expected_y).norm() <= 1.0e-12);
        assert!((padded(&observations.obs, 8) - expected_obs).norm() <= 1.0e-12);
        assert_eq!(observations.count(), 3);
    }

    #[test]
    fn matches_upstream_kalman_gain_check() {
        let meas_mat = individual_test_meas_mat();
        let covar = individual_test_covar();
        let noise = 0.01;
        let gain = kalman_gain(&covar, &meas_mat, noise);

        let h = DMatrix::from_fn(3, 3, |row, col| meas_mat[(row, col)]);
        let p = to_dmatrix(&covar);
        let innovation = &h * &p * h.transpose() + noise * DMatrix::identity(3, 3);
        let expected = &p
            * h.transpose()
            * innovation
                .try_inverse()
                .expect("the innovation covariance is invertible");
        let gain = DMatrix::from_fn(3, 3, |row, col| gain[(row, col)]);
        assert!((gain - expected).norm() <= 1.0e-12);
    }

    #[test]
    fn matches_upstream_ekf_update_check() {
        let gain = KalmanGain::<3>::from_row_slice(&[1.0, 2.0, 3.0, 0.0, 1.0, 2.0, 1.0, 0.0, 1.0]);
        let input_states = Vector3::new(2.0, 1.0, 0.75);
        let meas_mat = individual_test_meas_mat();
        let covar = individual_test_covar();
        let noise = 0.01;
        let input_y = individual_test_y(&input_states);

        let mut states = input_states;
        let (_, covar_out) = ekf_update(&gain, &covar, noise, &input_y, &meas_mat, &mut states);

        let k = DMatrix::from_fn(3, 3, |row, col| gain[(row, col)]);
        let h = DMatrix::from_fn(3, 3, |row, col| meas_mat[(row, col)]);
        let expected_states = DVector::from_column_slice(input_states.as_slice()) + &k * &input_y;
        let expected_p = expected_joseph_covariance(&k, &h, &to_dmatrix(&covar), noise);
        assert!(
            (DVector::from_column_slice(states.as_slice()) - expected_states).norm() <= 1.0e-12
        );
        assert!((to_dmatrix(&covar_out) - expected_p).norm() <= 1.0e-12);
    }

    #[test]
    fn matches_upstream_ckf_update_check() {
        // Upstream's `KGain = [1, 2, 3]` padded with zeros, read as 3 × 3.
        let gain = KalmanGain::<3>::from_row_slice(&[1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let input_states = Vector3::new(2.0, 1.0, 0.75);
        let x_bar = Vector3::new(0.1, 0.2, 0.01);
        let meas_mat = individual_test_meas_mat();
        let covar = individual_test_covar();
        let noise = 0.01;
        let input_y = individual_test_y(&input_states);

        let (state_error, covar_out) =
            ckf_update(&x_bar, &gain, &covar, noise, &input_y, &meas_mat);

        let k = DMatrix::from_fn(3, 3, |row, col| gain[(row, col)]);
        let h = DMatrix::from_fn(3, 3, |row, col| meas_mat[(row, col)]);
        let x_bar_d = DVector::from_column_slice(x_bar.as_slice());
        let expected_state_error = &x_bar_d + &k * (&input_y - &h * &x_bar_d);
        let expected_p = expected_joseph_covariance(&k, &h, &to_dmatrix(&covar), noise);
        assert!(
            (DVector::from_column_slice(state_error.as_slice()) - expected_state_error).norm()
                <= 1.0e-12
        );
        assert!((to_dmatrix(&covar_out) - expected_p).norm() <= 1.0e-12);
    }

    #[test]
    fn matches_upstream_state_prop_static() {
        let mut module = setup_filter_data();
        module.omega = Vector3::zeros();
        let css_data = unwritten_css_data();
        let css_config = Output::new(CssConfigMsg::default());
        css_data.connect_to(&mut module.css_data_in_msg);
        css_config.connect_to(&mut module.css_config_in_msg);
        module.init();

        // Upstream logs every 5 s over 8000 s; its first and last samples are
        // the t = 0 and t = 8000 s updates.
        module.update(&context(0));
        let first = module.state;
        for step in 1..=16_000 {
            module.update(&context(step * TASK_RATE_NANOS));
        }
        for i in 0..3 {
            assert!((module.state[i] - first[i]).abs() <= 1.0e-10, "state {i}");
        }
    }

    #[test]
    fn matches_upstream_state_prop_variable() {
        let mut module = setup_filter_data();
        let initial_state = module.state;
        let initial_x = module.state_error;
        let initial_covar = module.covar;
        let init_omega = module.omega;
        let q_proc_val = module.config.q_proc_val;
        let css_data = unwritten_css_data();
        let css_config = Output::new(CssConfigMsg::default());
        css_data.connect_to(&mut module.css_data_in_msg);
        css_config.connect_to(&mut module.css_config_in_msg);
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

        let gamma = DT_S.powi(2) / 2.0 * Matrix3::identity();
        let proc_noise_covar = gamma * (q_proc_val * Matrix3::identity()) * gamma.transpose();
        let mut expected_state = initial_state;
        let mut expected_omega = init_omega;
        let mut expected_x = initial_x;
        let mut expected_covar = initial_covar;
        let mut covar_failures = 0;
        for (j, (state, x, covar, stm)) in log.iter().enumerate().skip(1) {
            let exp_dyn_mat = expected_dynamics_matrix(&expected_omega);
            let expected_prev = expected_state;
            expected_state = expected_prev - DT_S * expected_omega.cross(&expected_prev);
            let expected_stm = DT_S * exp_dyn_mat + Matrix3::identity();
            expected_omega = expected_rate(&expected_state, &expected_prev, DT_S);
            expected_x = expected_stm * expected_x;
            expected_covar =
                expected_stm * expected_covar * expected_stm.transpose() + proc_noise_covar;

            for i in 0..3 {
                assert!(
                    (state[i] - expected_state[i]).abs() <= 1.0e-10,
                    "state {i} at {j}"
                );
                assert!(
                    (x[i] - expected_x[i]).abs() <= 1.0e-10,
                    "state error {i} at {j}"
                );
            }
            for i in 0..9 {
                // Upstream counts these as failures without a message.
                if (covar[i] - expected_covar[i]).abs() > 1.0e-8 {
                    covar_failures += 1;
                }
                assert!(
                    (stm[i] - expected_stm[i]).abs() <= 1.0e-10,
                    "STM {i} at {j}"
                );
            }
        }
        assert_eq!(covar_failures, 0, "covariance propagation failures");
    }

    /// One `test_all_sunline_oekf` parametrization.
    #[derive(Debug)]
    struct StateUpdateCase {
        sim_half_length: usize,
        add_meas_noise: bool,
        test_vector1: [f64; 3],
        test_vector2: [f64; 3],
        state_guess: [f64; 3],
    }

    const STATE_UPDATE_CASES: [StateUpdateCase; 6] = [
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 2000,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: false,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: false,
            test_vector1: [0.0, 0.4, -0.4],
            test_vector2: [0.0, 0.7, 0.2],
            state_guess: [0.3, 0.0, 0.6],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [0.0, 0.4, -0.4],
            test_vector2: [0.4, 0.5, 0.0],
            state_guess: [0.7, 0.7, 0.0],
        },
        StateUpdateCase {
            sim_half_length: 200,
            add_meas_noise: true,
            test_vector1: [-0.7, 0.7, 0.0],
            test_vector2: [0.8, 0.9, 0.0],
            state_guess: [0.7, 0.7, 0.0],
        },
    ];

    /// What upstream's module-variable loggers record after each update.
    struct InternalSample {
        observations: CssObservations<3>,
        state_error: Vector3<f64>,
        post_fits: SVector<f64, MAX_N_CSS_MEAS>,
    }

    impl InternalSample {
        fn of(module: &OkeefeEkf) -> Self {
            Self {
                observations: module.observations.clone(),
                state_error: module.state_error,
                post_fits: module.post_fits,
            }
        }
    }

    /// Upstream `StateUpdateSunLine`: converge on `test_vector1`, then on
    /// `test_vector2`. Returns upstream's failure messages.
    fn state_update_sun_line(case: &StateUpdateCase) -> Vec<String> {
        let mut failures = Vec::new();
        let mut rng = StdRng::seed_from_u64(42);
        let covar_tol = 1.0e-1;
        let state_tol = if case.add_meas_noise { 1.0e-1 } else { 1.0e-10 };

        let mut module = setup_filter_data();
        module.omega = Vector3::zeros();
        let css_units = css_constellation();
        let css_config = Output::new(CssConfigMsg::from_active(&css_units));
        let css_data = unwritten_css_data();
        css_config.connect_to(&mut module.css_config_in_msg);
        css_data.connect_to(&mut module.css_data_in_msg);
        module.state = Vector3::from(case.state_guess);
        module.state_error = Vector3::from(case.test_vector1) - module.state;
        module.init();
        let noise = Normal::new(0.0, module.config.q_obs_val.sqrt())
            .expect("the noise standard deviation is finite and positive");

        let mut data_log: Vec<SunlineFilterMsg> = Vec::new();
        let mut internal_log: Vec<InternalSample> = Vec::new();
        let mut now = 0;
        for (half, state_target) in [case.test_vector1, case.test_vector2].iter().enumerate() {
            let state_target = Vector3::from(*state_target);
            for i in 0..case.sim_half_length {
                if i > 20 {
                    let draw = case.add_meas_noise.then_some((&noise, &mut rng));
                    css_data.write(css_reading(&css_units, &state_target, draw, now));
                }
                // Upstream's first `ExecuteSimulation` also runs the t = 0 step.
                if half == 0 && i == 0 {
                    module.update(&context(0));
                    data_log.push(module.filt_data_out_msg.read());
                    internal_log.push(InternalSample::of(&module));
                }
                now = (half * case.sim_half_length + i + 1) as u64 * TASK_RATE_NANOS;
                module.update(&context(now));
                data_log.push(module.filt_data_out_msg.read());
                internal_log.push(InternalSample::of(&module));
            }

            let first_covar = data_log[0].packed_covariance::<3>();
            let last = &data_log[data_log.len() - 1];
            let last_covar = last.packed_covariance::<3>();
            for i in 0..3 {
                if (last_covar[(i, i)] - first_covar[(i, i)] / 100.0).abs() > covar_tol {
                    failures.push(format!("Covariance update failure (half {half}, {i})"));
                }
                if (last.state[i] - state_target[i]).abs() > state_tol {
                    failures.push(format!("State update failure (half {half}, {i})"));
                }
            }
        }

        // Post-fit residuals must equal `yMeas - H x` with the module's own
        // `numObs × 3` H at every step (issue #1353).
        for sample in &internal_log {
            let num_obs = sample.observations.count();
            if num_obs == 0 {
                continue;
            }
            let expected =
                &sample.observations.y_meas - &sample.observations.meas_mat * sample.state_error;
            if (sample.post_fits.rows(0, num_obs) - expected).norm() > 1.0e-9 {
                failures.push("Post-fit residual dimension mismatch (issue #1353)".to_string());
                break;
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
    fn packs_the_three_state_estimate_into_the_filter_message() {
        let mut module = setup_filter_data();
        module.covar = Matrix3::new(0.4, 0.01, 0.02, 0.01, 0.5, 0.03, 0.02, 0.03, 0.6);
        let css_data = unwritten_css_data();
        let css_config = Output::new(CssConfigMsg::from_active(&css_constellation()));
        css_data.connect_to(&mut module.css_data_in_msg);
        css_config.connect_to(&mut module.css_config_in_msg);
        module.init();
        module.update(&context(0));

        let msg = module.filt_data_out_msg.read();
        assert_eq!(msg.packed_covariance::<3>(), module.covar);
        // Upstream's memmove fills the first nine row-major slots of the 6 × 6.
        assert_eq!(msg.covar[(0, 0)], 0.4);
        assert_eq!(msg.covar[(0, 3)], 0.01);
        assert_eq!(msg.covar[(1, 2)], 0.6);
        assert!(msg.covar.fixed_view::<1, 3>(1, 3).iter().all(|&v| v == 0.0));
        assert!(msg.covar.rows(2, 4).iter().all(|&v| v == 0.0));
        assert_eq!(msg.state.fixed_rows::<3>(0), module.state);
        assert!(msg.state.fixed_rows::<3>(3).iter().all(|&v| v == 0.0));
        assert_eq!(msg.state_error.fixed_rows::<3>(0), module.state_error);
        assert_eq!(msg.num_obs, MAX_N_CSS_MEAS);
        let nav = module.nav_state_out_msg.read();
        assert!(
            (nav.vehicle_sun_point_body - Vector3::repeat(1.0 / 3.0_f64.sqrt())).norm() <= 1.0e-15
        );
    }

    /// `setup_filter_data` with `ekf_switch`, after one update that processes
    /// a clean reading of `(-0.7, 0.7, 0)` written at 0 s.
    fn filter_after_first_reading(omega: Vector3<f64>, ekf_switch: f64) -> OkeefeEkf {
        let mut module = setup_filter_data();
        module.omega = omega;
        module.config.ekf_switch = ekf_switch;
        let css_units = css_constellation();
        let css_config = Output::new(CssConfigMsg::from_active(&css_units));
        let css_data = unwritten_css_data();
        css_data.connect_to(&mut module.css_data_in_msg);
        css_config.connect_to(&mut module.css_config_in_msg);
        module.init();
        css_data.write(css_reading(
            &css_units,
            &Vector3::new(-0.7, 0.7, 0.0),
            None,
            0,
        ));
        module.update(&context(TASK_RATE_NANOS));
        module
    }

    #[test]
    fn config_set_after_new_takes_effect_at_init() {
        let run = |mut module: OkeefeEkf| {
            let css_units = css_constellation();
            let css_config = Output::new(CssConfigMsg::from_active(&css_units));
            let css_data = unwritten_css_data();
            css_config.connect_to(&mut module.css_config_in_msg);
            css_data.connect_to(&mut module.css_data_in_msg);
            module.init();
            for step in 0..4 {
                let sun_heading = Vector3::new(-0.7, 0.7, 0.0);
                let reading = css_reading(&css_units, &sun_heading, None, step * TASK_RATE_NANOS);
                css_data.write(reading);
                module.update(&context((step + 1) * TASK_RATE_NANOS));
            }
            module.filt_data_out_msg.read()
        };
        let tuned = OkeefeEkfConfig {
            q_obs_val: 0.01,
            q_proc_val: 0.5,
            sensor_use_thresh: 0.3,
            ekf_switch: 0.0,
            ..setup_filter_data().config
        };

        let seeded = setup_filter_data();
        let mut configured_in_new = OkeefeEkf::new(tuned.clone());
        configured_in_new.state = seeded.state;
        configured_in_new.omega = seeded.omega;
        configured_in_new.state_error = seeded.state_error;
        configured_in_new.covar = seeded.covar;
        let mut configured_later = setup_filter_data();
        configured_later.config = tuned;

        let expected = run(configured_in_new);
        assert_eq!(run(configured_later), expected);
        assert_ne!(run(setup_filter_data()), expected);
    }

    #[test]
    fn a_classical_update_leaves_the_reference_state_uncorrected() {
        // Upstream quirk: in CKF mode only the state error absorbs the
        // measurement; with no rate the reference heading never moves.
        let guess = setup_filter_data().state;
        let classical = filter_after_first_reading(Vector3::zeros(), 0.0);
        assert_eq!(classical.state, guess);
        assert_ne!(classical.state_error, setup_filter_data().state_error);

        let extended = filter_after_first_reading(Vector3::zeros(), 5.0);
        assert_ne!(extended.state, guess);
    }

    #[test]
    fn reset_keeps_the_rate_and_clears_the_previous_heading() {
        let mut module = filter_after_first_reading(Vector3::new(0.1, 0.2, 0.1), 5.0);
        // The measurement was processed at 0 s, so the rate inferred over the
        // zero step is zero; the step to 0.5 s infers it from the heading
        // change, which is zero too as nothing turned the heading.
        assert_eq!(module.omega, Vector3::zeros());
        module.omega = Vector3::new(0.3, -0.1, 0.2);
        let estimate = (module.state, module.omega, module.state_error, module.covar);
        let post_fits = module.post_fits;

        module.reset(10 * TASK_RATE_NANOS);
        assert_eq!(
            (module.state, module.omega, module.state_error, module.covar),
            estimate
        );
        assert_eq!(module.post_fits, post_fits);
        assert_eq!(module.prev_states, Vector3::zeros());
        assert_eq!(module.time_tag_s, 5.0);
        assert_eq!(module.observations.count(), MAX_N_CSS_MEAS);
        assert_eq!(module.state_transition, Matrix3::identity());
    }
}
