//! Inertial attitude square-root unscented Kalman filter: port of upstream
//! Basilisk `inertialUKF` (`fswAlgorithms/attDetermination/InertialUKF`).
//!
//! The six-element state is the MRP attitude `sigma_BN` followed by the body
//! rate `omega_BN_B`. Star-tracker attitudes are the measurements: one
//! [`StarTrackerAttitudeMsg`] input per tracker, processed in time-tag order.
//! The process model integrates the MRP kinematics plus the body acceleration
//! from the reaction to the wheels' spin-up torques, which are estimated from
//! the change in wheel speed. In a frame without any star-tracker measurement
//! the published attitude is instead propagated by composing the rotations in
//! the gyro buffer, and the published rate is the low-pass filtered gyro rate.
//!
//! # Message write times
//!
//! Upstream reads message metadata that the Rust ports do not carry:
//!
//! * A star tracker is *fresh* when its message was rewritten since the last
//!   read (`timeWritten` changed and `isWritten`). Here it is fresh when its
//!   payload differs from the previous read (`WriteTracker`). Star-tracker
//!   producers stamp a new `time_tag_nanos` on every sample, so the two agree.
//! * The wheel acceleration divides the speed change by the time between the
//!   writes of consecutive `RWSpeedMsg`s. Here a connected wheel-speed message
//!   counts as written at the current call time (the wheel states are
//!   published every step ahead of the filter); an unconnected one keeps
//!   upstream's zero write time, which disables the wheel torque.

use nalgebra::{DMatrix, DVector, Matrix3, SMatrix, SVector, Vector3, Vector4};

use crate::messages::{
    AKF_N_STATES, AccelerometerDataMsg, InertialFilterMsg, Input, MAX_ACC_BUF_PKT, MAX_EFF_COUNT,
    MAX_ST_VEH_COUNT, NavigationAttitudeMsg, Output, RwArrayConfigMsg, RwSpeedMsg,
    StarTrackerAttitudeMsg, VehicleConfigMsg,
};
use crate::time::NANO2SEC;
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::m33_inverse;
use crate::utilities::rigid_body_kinematics::{
    add_ep, add_mrp, b_mat_mrp, ep_to_mrp, mrp_shadow, mrp_switch, mrp_to_ep, prv_to_ep,
};
use crate::utilities::signal_condition::LowPassFilter;
use crate::utilities::ukf_utilities::{
    UkfResultExt, ukf_chol_decomp, ukf_chol_down_date, ukf_l_inv, ukf_qrd_just_r, ukf_u_inv,
};
use crate::{Module, SimulationContext};

/// Number of sigma points, upstream `2 * countHalfSPs + 1`.
const NUM_SIGMA: usize = 2 * AKF_N_STATES + 1;
/// Size of a star-tracker measurement, upstream `numObs`.
const NUM_OBS: usize = 3;

type StateVector = SVector<f64, AKF_N_STATES>;
type StateMatrix = SMatrix<f64, AKF_N_STATES, AKF_N_STATES>;

/// User-settable parameters of [`InertialUkf`] (upstream `InertialUKFConfig`
/// fields set from Python). Every default is upstream's zero initialization.
#[derive(Clone, Debug, Default)]
pub struct InertialUkfConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `alpha` [-]: sigma-point spread.
    pub alpha: f64,
    /// Upstream `beta` [-]: prior-distribution parameter (2 for Gaussian).
    pub beta: f64,
    /// Upstream `kappa` [-]: secondary sigma-point scaling.
    pub kappa: f64,
    /// Upstream `switchMag` [-]: MRP norm above which the attitude switches to
    /// its shadow set.
    pub switch_mag: f64,
    /// Upstream `maxTimeJump` \[s\]: largest propagation step taken to reach a
    /// measurement; longer gaps are shortened to it with a warning. Zero
    /// disables the limit.
    pub max_time_jump_s: f64,
    /// Upstream `stateInit` [-]: initial `[sigma_BN, omega_BN_B]`.
    pub state_init: StateVector,
    /// Upstream `covarInit` [-]: initial covariance.
    pub covar_init: StateMatrix,
    /// Upstream `qNoise` [-]: process noise. Only its diagonal is scaled with
    /// the time step (by `dt^2 / 2` for the attitude and `dt` for the rate).
    pub q_noise: StateMatrix,
    /// Upstream `STDatasStruct.STMessages[i].noise` [-]: measurement noise of
    /// star tracker `i`. Its length when the module is built is the number of
    /// trackers (upstream `numST`, at most [`MAX_ST_VEH_COUNT`]); a tracker
    /// without an entry uses zero noise, like upstream's zeroed slot.
    pub st_noise: Vec<Matrix3<f64>>,
    /// Upstream `gyroFilt`: low-pass filters for the three gyro axes. They are
    /// copied into the module at `init`; `reset` keeps their running state,
    /// as upstream `Reset` does.
    pub gyro_filter: [LowPassFilter; 3],
}

/// Stand-in for upstream's message write metadata (`isWritten`,
/// `timeWritten`), which the Rust ports do not carry: a payload counts as
/// written when it differs from the previous read, at the call time it was
/// first seen. A payload holding NaN never compares equal and so counts as
/// new on every read.
#[derive(Clone, Debug, Default)]
pub(crate) struct WriteTracker<T> {
    last_read: T,
    written_nanos: Option<u64>,
}

impl<T: Clone + PartialEq> WriteTracker<T> {
    /// Records a read of `payload` at `now_nanos`; `true` when it is new.
    pub(crate) fn observe(&mut self, payload: &T, now_nanos: u64) -> bool {
        let is_new = *payload != self.last_read;
        if is_new {
            self.last_read = payload.clone();
            self.written_nanos = Some(now_nanos);
        }
        is_new
    }

    /// When the payload last changed; `None` while it never has (upstream
    /// `isWritten == 0`).
    pub(crate) fn written_nanos(&self) -> Option<u64> {
        self.written_nanos
    }
}

/// `m` as the dynamically sized matrix the `ukf_utilities` routines take.
pub(crate) fn to_dynamic<const R: usize, const C: usize>(m: &SMatrix<f64, R, C>) -> DMatrix<f64> {
    DMatrix::from_column_slice(R, C, m.as_slice())
}

/// The leading `R x C` block of a `ukf_utilities` result, as a fixed-size
/// matrix.
pub(crate) fn to_static<const R: usize, const C: usize>(m: &DMatrix<f64>) -> SMatrix<f64, R, C> {
    m.fixed_view::<R, C>(0, 0).into_owned()
}

/// `v` as the dynamically sized vector the `ukf_utilities` routines take.
pub(crate) fn to_dynamic_vector<const N: usize>(v: &SVector<f64, N>) -> DVector<f64> {
    DVector::from_column_slice(v.as_slice())
}

/// Upstream's unscented-transform parameters for `S = 2 n + 1` sigma points
/// of an `n`-state filter: `lambdaVal`, `gamma` and the weights `wM`, `wC`.
/// The default is upstream's zero initialization.
#[derive(Clone, Copy, Debug)]
pub(crate) struct SigmaWeights<const S: usize> {
    pub(crate) lambda: f64,
    pub(crate) gamma: f64,
    pub(crate) w_m: SVector<f64, S>,
    pub(crate) w_c: SVector<f64, S>,
}

impl<const S: usize> Default for SigmaWeights<S> {
    fn default() -> Self {
        Self {
            lambda: 0.0,
            gamma: 0.0,
            w_m: SVector::zeros(),
            w_c: SVector::zeros(),
        }
    }
}

impl<const S: usize> SigmaWeights<S> {
    const NUM_STATES: f64 = ((S - 1) / 2) as f64;

    /// The standard parameters for the filter's `alpha`, `beta` and `kappa`
    /// (upstream `Reset`).
    pub(crate) fn new(alpha: f64, beta: f64, kappa: f64) -> Self {
        let n = Self::NUM_STATES;
        let lambda = alpha * alpha * (n + kappa) - n;
        let mut weights = Self {
            lambda,
            gamma: (n + lambda).sqrt(),
            ..Self::default()
        };
        weights.reset_weights(alpha, beta);
        weights
    }

    /// Recomputes `wM` and `wC` from the stored `lambda`, as upstream's
    /// clean-up after a bad update does (it leaves `lambda` and `gamma`).
    pub(crate) fn reset_weights(&mut self, alpha: f64, beta: f64) {
        let n = Self::NUM_STATES;
        let lambda = self.lambda;
        self.w_m = SVector::repeat(0.5 / (n + lambda));
        self.w_c = self.w_m;
        self.w_m[0] = lambda / (n + lambda);
        self.w_c[0] = lambda / (n + lambda) + (1.0 - alpha * alpha + beta);
    }
}

/// A time or measurement update that upstream reports by returning `-1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct BadUpdate;

/// What the filter keeps from one star tracker's input.
#[derive(Clone, Debug, Default)]
struct TrackerReading {
    /// Upstream `stSensorIn[i]`; the measurement model shadows its MRP in
    /// place.
    measurement: StarTrackerAttitudeMsg,
    /// Upstream `isFreshST[i]`.
    is_fresh: bool,
    /// Upstream `ClockTimeST[i]`; like it, never cleared by `reset`.
    writes: WriteTracker<StarTrackerAttitudeMsg>,
}

impl TrackerReading {
    fn time_tag_s(&self) -> f64 {
        self.measurement.time_tag_nanos as f64 * NANO2SEC
    }
}

/// Upstream `inertialUKF`: inertial attitude and rate estimate from star
/// trackers, reaction-wheel speeds and a gyro buffer.
#[derive(Clone, Debug)]
pub struct InertialUkf {
    /// User-settable filter parameters.
    pub config: InertialUkfConfig,
    /// Published attitude and rate estimate.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
    /// Filter state, covariance and observation count.
    pub filt_data_out_msg: Output<InertialFilterMsg>,
    /// Spacecraft inertia, read every update.
    pub mass_props_in_msg: Input<VehicleConfigMsg>,
    /// Reaction-wheel configuration, read at reset; may be an empty message
    /// when there are no wheels.
    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    /// Reaction-wheel speeds; may be an empty message when there are no
    /// wheels.
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Gyro packet ring buffer, used between star-tracker updates.
    pub gyr_buff_in_msg: Input<AccelerometerDataMsg>,
    /// Upstream `STDatasStruct.STMessages[i].stInMsg`, one per entry of
    /// [`InertialUkfConfig::st_noise`].
    pub st_in_msgs: Vec<Input<StarTrackerAttitudeMsg>>,

    weights: SigmaWeights<NUM_SIGMA>,
    time_tag_s: f64,
    gyr_agg_time_tag_s: f64,
    agg_sigma_b2b1: Vector3<f64>,
    state: StateVector,
    state_prev: StateVector,
    s_bar: StateMatrix,
    s_bar_prev: StateMatrix,
    covar: StateMatrix,
    covar_prev: StateMatrix,
    x_bar: StateVector,
    obs: Vector3<f64>,
    /// Upstream `yMeas`: one predicted measurement per sigma point (column).
    y_meas: SMatrix<f64, NUM_OBS, NUM_SIGMA>,
    /// Upstream `SP`: one sigma point per column.
    sigma_points: SMatrix<f64, AKF_N_STATES, NUM_SIGMA>,
    /// Upstream `sQnoise`: the transposed (upper) Cholesky factor of `qNoise`.
    s_q_noise: StateMatrix,
    i_inv: Matrix3<f64>,
    num_used_gyros: usize,
    first_pass_complete: bool,
    sigma_bn_out: Vector3<f64>,
    omega_bn_b_out: Vector3<f64>,
    time_tag_out_s: f64,
    trackers: Vec<TrackerReading>,
    /// Upstream `stSensorOrder`: tracker indices sorted by time tag. Entries
    /// past the tracker count stay zero, as upstream.
    st_sensor_order: [usize; MAX_ST_VEH_COUNT],
    rw_config_params: RwArrayConfigMsg,
    rw_speeds: RwSpeedMsg,
    rw_speed_prev: RwSpeedMsg,
    speed_dt_s: f64,
    time_wheel_prev_nanos: u64,
    local_config_data: VehicleConfigMsg,
    gyro_filt: [LowPassFilter; 3],
}

impl InertialUkf {
    /// Builds the filter with one star-tracker input per entry of
    /// `config.st_noise`.
    ///
    /// # Panics
    ///
    /// If `config.st_noise` has more than [`MAX_ST_VEH_COUNT`] entries.
    pub fn new(config: InertialUkfConfig) -> Self {
        let num_trackers = config.st_noise.len();
        assert!(
            num_trackers <= MAX_ST_VEH_COUNT,
            "at most {MAX_ST_VEH_COUNT} star trackers are supported"
        );
        Self {
            st_in_msgs: (0..num_trackers).map(|_| Input::default()).collect(),
            trackers: vec![TrackerReading::default(); num_trackers],
            gyro_filt: config.gyro_filter,
            config,
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            mass_props_in_msg: Input::default(),
            rw_params_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            gyr_buff_in_msg: Input::default(),
            weights: SigmaWeights::default(),
            time_tag_s: 0.0,
            gyr_agg_time_tag_s: 0.0,
            agg_sigma_b2b1: Vector3::zeros(),
            state: StateVector::zeros(),
            state_prev: StateVector::zeros(),
            s_bar: StateMatrix::zeros(),
            s_bar_prev: StateMatrix::zeros(),
            covar: StateMatrix::zeros(),
            covar_prev: StateMatrix::zeros(),
            x_bar: StateVector::zeros(),
            obs: Vector3::zeros(),
            y_meas: SMatrix::zeros(),
            sigma_points: SMatrix::zeros(),
            s_q_noise: StateMatrix::zeros(),
            i_inv: Matrix3::zeros(),
            num_used_gyros: 0,
            first_pass_complete: false,
            sigma_bn_out: Vector3::zeros(),
            omega_bn_b_out: Vector3::zeros(),
            time_tag_out_s: 0.0,
            st_sensor_order: [0; MAX_ST_VEH_COUNT],
            rw_config_params: RwArrayConfigMsg::default(),
            rw_speeds: RwSpeedMsg::default(),
            rw_speed_prev: RwSpeedMsg::default(),
            speed_dt_s: 0.0,
            time_wheel_prev_nanos: 0,
            local_config_data: VehicleConfigMsg::default(),
        }
    }

    /// Upstream `Reset_inertialUKF`: checks the inputs, caches the static
    /// configuration and restarts the filter from the initial state.
    pub fn reset(&mut self, call_time_nanos: u64) {
        for (connected, name) in [
            (self.rw_params_in_msg.is_connected(), "rw_params_in_msg"),
            (self.mass_props_in_msg.is_connected(), "mass_props_in_msg"),
            (self.rw_speeds_in_msg.is_connected(), "rw_speeds_in_msg"),
            (self.gyr_buff_in_msg.is_connected(), "gyr_buff_in_msg"),
        ] {
            if !connected {
                log::error!("inertialUKF.{name} is not connected");
            }
        }

        self.rw_config_params = self.rw_params_in_msg.read();
        self.local_config_data = self.mass_props_in_msg.read();

        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.first_pass_complete = false;
        self.speed_dt_s = 0.0;
        self.time_wheel_prev_nanos = 0;
        self.obs = Vector3::zeros();
        self.sigma_points = SMatrix::zeros();
        self.weights = SigmaWeights::new(self.config.alpha, self.config.beta, self.config.kappa);
        self.state = self.config.state_init;

        let mut bad_update = false;
        self.covar = self.config.covar_init;
        self.s_bar = to_static(
            &ukf_chol_decomp(&to_dynamic(&self.config.covar_init))
                .into_output_flagging(&mut bad_update),
        );
        self.s_q_noise = to_static::<AKF_N_STATES, AKF_N_STATES>(
            &ukf_chol_decomp(&to_dynamic(&self.config.q_noise))
                .into_output_flagging(&mut bad_update),
        )
        .transpose();

        self.sigma_bn_out = self.attitude();
        self.omega_bn_b_out = self.rate();
        self.time_tag_out_s = self.time_tag_s;
        self.read_st_messages(call_time_nanos);

        if bad_update {
            log::warn!("inertialUKF: reset contained a bad update");
        }
    }

    fn attitude(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(0).into_owned()
    }

    fn rate(&self) -> Vector3<f64> {
        self.state.fixed_rows::<3>(3).into_owned()
    }

    /// Upstream `Read_STMessages`: reads every tracker, flags the fresh ones
    /// and orders them by time tag.
    fn read_st_messages(&mut self, call_time_nanos: u64) {
        for (reading, input) in self.trackers.iter_mut().zip(&self.st_in_msgs) {
            reading.measurement = input.read();
            reading.is_fresh = reading
                .writes
                .observe(&reading.measurement, call_time_nanos);
        }
        let trackers = &self.trackers;
        let order = &mut self.st_sensor_order[..trackers.len()];
        for (position, slot) in order.iter_mut().enumerate() {
            *slot = position;
        }
        // Upstream's insertion sort swaps on a strict `<`, so it is stable.
        order.sort_by_key(|&tracker| trackers[tracker].measurement.time_tag_nanos);
    }

    /// The latest positive time tag among the fresh trackers, which starts
    /// the filter clock on the first pass.
    fn latest_fresh_time_tag_s(&self) -> Option<f64> {
        self.trackers
            .iter()
            .filter(|reading| reading.is_fresh)
            .map(TrackerReading::time_tag_s)
            .filter(|&time_tag_s| time_tag_s > 0.0)
            .reduce(f64::max)
    }

    /// Upstream `Update_inertialUKF`.
    fn update_filter(&mut self, call_time_nanos: u64) {
        let switch_mag = self.config.switch_mag;
        let sigma = self.attitude();
        if sigma.norm() > switch_mag {
            self.state
                .fixed_rows_mut::<3>(0)
                .copy_from(&mrp_switch(sigma, switch_mag));
        }

        self.local_config_data = self.mass_props_in_msg.read();
        let gyro_buffer = self.gyr_buff_in_msg.read();
        self.rw_speeds = self.rw_speeds_in_msg.read();
        let rw_speeds_written_nanos = if self.rw_speeds_in_msg.is_connected() {
            call_time_nanos
        } else {
            0
        };
        self.read_st_messages(call_time_nanos);
        // Upstream `m33Inverse` logs a singular inertia and writes zeros.
        self.i_inv = m33_inverse(&self.local_config_data.inertia_about_point_b_kg_m2)
            .unwrap_or_else(Matrix3::zeros);

        // Nothing is estimated or published until a star tracker delivers.
        if !self.first_pass_complete {
            self.rw_speed_prev = self.rw_speeds.clone();
            self.time_wheel_prev_nanos = rw_speeds_written_nanos;
            let Some(first_time_tag_s) = self.latest_fresh_time_tag_s() else {
                return;
            };
            self.first_pass_complete = true;
            self.time_tag_s = first_time_tag_s;
        }

        self.speed_dt_s =
            diff_nanos_to_seconds(rw_speeds_written_nanos, self.time_wheel_prev_nanos);
        self.time_wheel_prev_nanos = rw_speeds_written_nanos;

        let (tracker_valid, num_obs) = self.process_star_trackers();
        if tracker_valid < 1 {
            self.propagate_output_on_gyros(&gyro_buffer);
        } else {
            self.sigma_bn_out = self.attitude();
            self.omega_bn_b_out = self.rate();
            self.time_tag_out_s = self.time_tag_s;
        }

        self.nav_state_out_msg.write(NavigationAttitudeMsg {
            time_tag_s: self.time_tag_out_s,
            sigma_bn: self.sigma_bn_out,
            omega_bn_b_radps: self.omega_bn_b_out,
            vehicle_sun_point_body: Vector3::zeros(),
        });
        self.filt_data_out_msg.write(InertialFilterMsg {
            time_tag_s: self.time_tag_s,
            covar: self.covar,
            state: self.state,
            // Upstream's `int` count goes negative when both updates of a
            // tracker fail; the Rust count stops at zero.
            num_obs: usize::try_from(num_obs).unwrap_or(0),
        });
        self.rw_speed_prev = self.rw_speeds.clone();
    }

    /// Runs a time and a measurement update for every fresh tracker whose
    /// time tag has not been passed yet. Returns upstream's `trackerValid`
    /// (1 when the last processed tracker updated cleanly, less after a bad
    /// update, 0 when none was processed) and the observation count.
    fn process_star_trackers(&mut self) -> (i32, i32) {
        let mut tracker_valid = 0;
        let mut num_obs = 0;
        for position in 0..self.trackers.len() {
            let tracker = self.st_sensor_order[position];
            let reading = &self.trackers[tracker];
            let measurement_time_s = reading.time_tag_s();
            let is_due = measurement_time_s >= self.time_tag_s && reading.is_fresh;
            if !is_due {
                continue;
            }
            tracker_valid = 1;
            let max_jump_s = self.config.max_time_jump_s;
            if measurement_time_s - self.time_tag_s > max_jump_s && max_jump_s > 0.0 {
                self.time_tag_s = measurement_time_s - max_jump_s;
                log::warn!("inertialUKF: large jump in state time that was set to max");
            }
            tracker_valid -= i32::from(self.time_update(measurement_time_s).is_err());
            tracker_valid -= i32::from(self.meas_update(tracker).is_err());
            num_obs += tracker_valid;
        }
        (tracker_valid, num_obs)
    }

    /// Propagates the published attitude through the gyro buffer and takes
    /// the published rate from the low-pass filtered gyros.
    fn propagate_output_on_gyros(&mut self, gyro_buffer: &AccelerometerDataMsg) {
        self.aggregate_gyro_data(self.time_tag_out_s, gyro_buffer);
        let switch_mag = self.config.switch_mag;
        let mut sigma_sum = add_mrp(self.sigma_bn_out, self.agg_sigma_b2b1);
        if sigma_sum.norm() > switch_mag {
            sigma_sum = mrp_switch(sigma_sum, switch_mag);
        }
        self.sigma_bn_out = sigma_sum;
        self.omega_bn_b_out = Vector3::from_fn(|axis, _| self.gyro_filt[axis].current_state);
        // Quirk kept: without gyro packets this stays at the previous (or
        // initial zero) aggregation time.
        self.time_tag_out_s = self.gyr_agg_time_tag_s;
    }

    /// Upstream `inertialStateProp`: one Euler step of the MRP kinematics and
    /// of the rate driven by the reaction-wheel spin-up torques.
    fn state_prop(&self, state: &mut StateVector, dt_s: f64) {
        let sigma = state.fixed_rows::<3>(0).into_owned();
        let omega = state.fixed_rows::<3>(3).into_owned();
        let sigma_dot = b_mat_mrp(sigma) * 0.25 * omega;
        state
            .fixed_rows_mut::<3>(0)
            .copy_from(&(sigma + sigma_dot * dt_s));

        let num_rw = self.rw_config_params.num_rw.min(MAX_EFF_COUNT);
        let mut torque_total = Vector3::zeros();
        if self.speed_dt_s != 0.0 {
            for wheel in 0..num_rw {
                let speed_delta = self.rw_speeds.wheel_speeds_radps[wheel]
                    - self.rw_speed_prev.wheel_speeds_radps[wheel];
                let wheel_torque = speed_delta
                    * self.rw_config_params.spin_axis_inertias_kg_m2[wheel]
                    / self.speed_dt_s;
                torque_total -= self.rw_config_params.spin_axes_body[wheel] * wheel_torque;
            }
        }
        let ang_accel = self.i_inv * torque_total * dt_s;
        state.fixed_rows_mut::<3>(3).copy_from(&(omega + ang_accel));
    }

    /// Saves the estimate a bad update falls back to (upstream `statePrev`,
    /// `sBarPrev`, `covarPrev`).
    fn save_previous(&mut self) {
        self.state_prev = self.state;
        self.s_bar_prev = self.s_bar;
        self.covar_prev = self.covar;
    }

    /// Upstream `inertialUKFCleanUpdate`: restores the previous estimate and
    /// the standard weights after a bad update.
    fn clean_update(&mut self) {
        self.obs = Vector3::zeros();
        self.state = self.state_prev;
        self.s_bar = self.s_bar_prev;
        self.covar = self.covar_prev;
        self.weights
            .reset_weights(self.config.alpha, self.config.beta);
    }

    /// Upstream `inertialUKFTimeUpdate`: propagates the sigma points to
    /// `update_time_s` and rebuilds the square-root covariance.
    fn time_update(&mut self, update_time_s: f64) -> Result<(), BadUpdate> {
        let dt_s = update_time_s - self.time_tag_s;
        self.save_previous();

        let mut center = self.state;
        self.state_prop(&mut center, dt_s);
        self.sigma_points.set_column(0, &center);
        let SigmaWeights {
            gamma, w_m, w_c, ..
        } = self.weights;
        self.x_bar = center * w_m[0];
        // Upstream visits the +/- pair of each sBar column together, which
        // fixes the summation order of xBar.
        for column in 0..AKF_N_STATES {
            for (index, spread) in [(column + 1, gamma), (column + 1 + AKF_N_STATES, -gamma)] {
                let mut point = self.s_bar.column(column) * spread + self.state;
                self.state_prop(&mut point, dt_s);
                self.sigma_points.set_column(index, &point);
                self.x_bar = point * w_m[index] + self.x_bar;
            }
        }

        let mut stacked = DMatrix::zeros(2 * AKF_N_STATES + AKF_N_STATES, AKF_N_STATES);
        for index in 1..NUM_SIGMA {
            if w_c[index] <= 0.0 {
                self.clean_update();
                return Err(BadUpdate);
            }
            let deviation = (self.sigma_points.column(index) - self.x_bar) * w_c[index].sqrt();
            stacked.row_mut(index - 1).copy_from(&deviation.transpose());
        }
        let mut proc_noise = self.s_q_noise;
        for axis in 0..3 {
            proc_noise[(axis, axis)] *= dt_s * dt_s / 2.0;
            proc_noise[(axis + 3, axis + 3)] *= dt_s;
        }
        stacked
            .view_mut((2 * AKF_N_STATES, 0), (AKF_N_STATES, AKF_N_STATES))
            .copy_from(&proc_noise);
        self.s_bar = to_static::<AKF_N_STATES, AKF_N_STATES>(&ukf_qrd_just_r(&stacked)).transpose();

        let mut bad_update = false;
        let center_error = center - self.x_bar;
        self.s_bar = to_static(
            &ukf_chol_down_date(
                &to_dynamic(&self.s_bar),
                &to_dynamic_vector(&center_error),
                w_c[0],
            )
            .into_output_flagging(&mut bad_update),
        );
        self.covar = self.s_bar * self.s_bar.transpose();
        self.state = center;

        if bad_update {
            self.clean_update();
            return Err(BadUpdate);
        }
        self.time_tag_s = update_time_s;
        Ok(())
    }

    /// Upstream `inertialUKFMeasModel`: the star tracker measures the MRP
    /// state directly. The measurement is first moved to the shadow set when
    /// that brings it closer to the estimate, so the residual never wraps.
    fn meas_model(&mut self, tracker: usize) {
        let mut state_inverse = mrp_to_ep(self.attitude());
        let vector_part = -state_inverse.fixed_rows::<3>(1).into_owned();
        state_inverse.fixed_rows_mut::<3>(1).copy_from(&vector_part);
        let measurement = &mut self.trackers[tracker].measurement;
        let relative: Vector4<f64> = add_ep(state_inverse, mrp_to_ep(measurement.sigma_bn));
        if ep_to_mrp(relative).norm() > 1.0 {
            measurement.sigma_bn = mrp_shadow(measurement.sigma_bn);
        }
        self.obs = measurement.sigma_bn;
        self.y_meas = self.sigma_points.fixed_rows::<NUM_OBS>(0).into_owned();
    }

    /// Weighted mean of the predicted measurements (upstream `yBar`).
    fn mean_measurement(&self) -> Vector3<f64> {
        let mut y_bar = Vector3::zeros();
        for (index, predicted) in self.y_meas.column_iter().enumerate() {
            y_bar += predicted * self.weights.w_m[index];
        }
        y_bar
    }

    /// Upstream `inertialUKFMeasUpdate`: applies star tracker `tracker`'s
    /// attitude to the state and square-root covariance.
    fn meas_update(&mut self, tracker: usize) -> Result<(), BadUpdate> {
        self.save_previous();
        self.meas_model(tracker);
        let y_bar = self.mean_measurement();
        let w_c = self.weights.w_c;

        let mut stacked = DMatrix::zeros(2 * AKF_N_STATES + NUM_OBS, NUM_OBS);
        for index in 1..NUM_SIGMA {
            // Quirk kept: unlike the time update this tests `< 0` and returns
            // without restoring the previous estimate.
            if w_c[index] < 0.0 {
                return Err(BadUpdate);
            }
            let deviation = (self.y_meas.column(index) - y_bar) * w_c[index].sqrt();
            stacked.row_mut(index - 1).copy_from(&deviation.transpose());
        }

        let mut bad_update = false;
        let noise = self
            .config
            .st_noise
            .get(tracker)
            .copied()
            .unwrap_or_else(Matrix3::zeros);
        // Upstream stacks the lower Cholesky factor itself (not its
        // transpose), which only matters for a non-diagonal noise.
        let noise_chol = ukf_chol_decomp(&to_dynamic(&noise)).into_output_flagging(&mut bad_update);
        stacked
            .view_mut((2 * AKF_N_STATES, 0), (NUM_OBS, NUM_OBS))
            .copy_from(&noise_chol);
        let sy = ukf_qrd_just_r(&stacked).transpose();
        let center_deviation = self.y_meas.column(0) - y_bar;
        let sy: Matrix3<f64> = to_static(
            &ukf_chol_down_date(&sy, &to_dynamic_vector(&center_deviation), w_c[0])
                .into_output_flagging(&mut bad_update),
        );

        let mut p_xy = SMatrix::<f64, AKF_N_STATES, NUM_OBS>::zeros();
        for index in 0..NUM_SIGMA {
            let measurement_deviation = self.y_meas.column(index) - y_bar;
            let state_deviation = (self.sigma_points.column(index) - self.x_bar) * w_c[index];
            p_xy += state_deviation * measurement_deviation.transpose();
        }

        let sy_t_inv: Matrix3<f64> = to_static(
            &ukf_u_inv(&to_dynamic(&sy.transpose())).into_output_flagging(&mut bad_update),
        );
        let sy_inv: Matrix3<f64> =
            to_static(&ukf_l_inv(&to_dynamic(&sy)).into_output_flagging(&mut bad_update));
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
                .into_output_flagging(&mut bad_update),
            );
        }
        self.covar = self.s_bar * self.s_bar.transpose();

        if bad_update {
            self.clean_update();
            return Err(BadUpdate);
        }
        Ok(())
    }

    /// Upstream `inertialUKFAggGyrData`: composes the gyro-buffer rotations
    /// newer than `prev_time_s` into `agg_sigma_b2b1` and feeds each packet's
    /// rate through the low-pass filters. The buffer is a ring: aggregation
    /// starts at the oldest newer packet and stops at the first older one.
    fn aggregate_gyro_data(&mut self, prev_time_s: f64, buffer: &AccelerometerDataMsg) {
        let packet_time_s =
            |index: usize| buffer.accel_packets[index].meas_time_nanos as f64 * NANO2SEC;
        // Upstream's "no packet" sentinel is index 0 at time -1.
        let (mut index, mut next_time_s) = (0..MAX_ACC_BUF_PKT)
            .map(|index| (index, packet_time_s(index)))
            .filter(|&(_, time_s)| time_s > prev_time_s)
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .unwrap_or((0, -1.0));

        let mut ep_bprop_b0 = Vector4::new(1.0, 0.0, 0.0, 0.0);
        let mut count = 0;
        let mut meas_time_s = prev_time_s;
        while next_time_s > prev_time_s && count < MAX_ACC_BUF_PKT {
            let dt_s = next_time_s - meas_time_s;
            let omega = buffer.accel_packets[index].gyro_b_radps;
            // The rate times dt is a small-angle PRV; quaternions compose it.
            ep_bprop_b0 = add_ep(ep_bprop_b0, prv_to_ep(omega * dt_s));
            self.gyr_agg_time_tag_s = next_time_s;
            count += 1;
            meas_time_s = next_time_s;
            index = (index + 1) % MAX_ACC_BUF_PKT;
            next_time_s = packet_time_s(index);
            for (filter, rate) in self.gyro_filt.iter_mut().zip(omega.iter()) {
                filter.filter(*rate);
            }
        }
        self.num_used_gyros = count;
        self.agg_sigma_b2b1 = ep_to_mrp(ep_bprop_b0);
    }
}

impl Module for InertialUkf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
        self.filt_data_out_msg.write(InertialFilterMsg::default());
        self.gyro_filt = self.config.gyro_filter;
        InertialUkf::reset(self, 0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        InertialUkf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.update_filter(context.current_sim_nanos);
    }
}

#[cfg(test)]
mod tests {
    //! Ports `InertialUKF/_UnitTest/test_inertialKF.py`: `test_FilterMethods`,
    //! `test_stateUpdateInertialAttitude` and
    //! `test_stateUpdateRWInertialAttitude`. Upstream logs the module's own
    //! `covar` and `state` variables, which these tests read from the private
    //! fields.
    //!
    //! Upstream disables `BROKENtest_statePropInertialAttitude`,
    //! `BROKENtest_StatePropRateInertialAttitude` and
    //! `BROKENtest_FaultScenarios` (pytest does not collect the `BROKENtest_`
    //! prefix, and the file notes that they "appear to be broken"): some of
    //! their assertions fail against upstream's own filter. The parts that
    //! hold are ported:
    //!
    //! * `StatePropRateInertialAttitude`: its rate check. Its covariance check
    //!   (every variance must grow) fails upstream, because the covariance only
    //!   changes on star-tracker updates and the scenario's single one happens
    //!   at the first tick.
    //! * `FaultScenarios`: everything except its last time update, which is
    //!   expected to fail from a negative `qNoise` but succeeds upstream
    //!   (`qNoise` is only read by `Reset`).
    //! * `statePropInertialAttitude` is not ported: it never writes a star
    //!   tracker, so the filter never starts and its covariance check fails;
    //!   `publishes_nothing_before_the_first_star_tracker_measurement` covers
    //!   that behaviour.
    //!
    //! The "compiled upstream" values come from upstream `inertialUKF.c`,
    //! built with gcc and driven through the same scenario; this port
    //! reproduces them bit for bit.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, SVector, Vector3};

    use super::{InertialUkf, InertialUkfConfig, StateMatrix, StateVector};
    use crate::messages::{
        AccelerometerDataMsg, InertialFilterMsg, MAX_ACC_BUF_PKT, Output, RwArrayConfigMsg,
        RwSpeedMsg, StarTrackerAttitudeMsg, VehicleConfigMsg,
    };
    use crate::utilities::signal_condition::LowPassFilter;
    use crate::{Module, SimulationContext};

    const TASK_NANOS: u64 = 500_000_000;

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

    /// Upstream `setupFilterData` for `num_st` star trackers.
    fn setup_filter_data(num_st: usize) -> InertialUkfConfig {
        let mut covar_init = StateMatrix::zeros();
        let mut q_noise = StateMatrix::zeros();
        for axis in 0..3 {
            covar_init[(axis, axis)] = 0.04;
            covar_init[(axis + 3, axis + 3)] = 0.004;
            q_noise[(axis, axis)] = 0.0017 * 0.0017;
            q_noise[(axis + 3, axis + 3)] = 0.00017 * 0.00017;
        }
        InertialUkfConfig {
            name: "InertialUKF".to_string(),
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            switch_mag: 1.2,
            max_time_jump_s: 0.0,
            state_init: StateVector::from_column_slice(&[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            covar_init,
            q_noise,
            st_noise: vec![Matrix3::from_diagonal_element(0.00017 * 0.00017); num_st],
            gyro_filter: [LowPassFilter::new(0.5, 15.0 / (2.0 * PI)); 3],
        }
    }

    /// The module with every input connected, as in the upstream tests, and
    /// the outputs the tests keep writing.
    struct Fixture {
        module: InertialUkf,
        st_out: Vec<Output<StarTrackerAttitudeMsg>>,
        rw_speeds_out: Output<RwSpeedMsg>,
        gyro_out: Output<AccelerometerDataMsg>,
    }

    impl Fixture {
        fn new(
            config: InertialUkfConfig,
            rw_config: RwArrayConfigMsg,
            rw_speeds: RwSpeedMsg,
        ) -> Self {
            let mut module = InertialUkf::new(config);
            let st_out: Vec<Output<StarTrackerAttitudeMsg>> = (0..module.st_in_msgs.len())
                .map(|_| Output::default())
                .collect();
            for (output, input) in st_out.iter().zip(module.st_in_msgs.iter_mut()) {
                output.connect_to(input);
            }
            Output::new(VehicleConfigMsg {
                inertia_about_point_b_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                    1000.0, 800.0, 800.0,
                )),
                ..Default::default()
            })
            .connect_to(&mut module.mass_props_in_msg);
            Output::new(rw_config).connect_to(&mut module.rw_params_in_msg);
            let rw_speeds_out = Output::new(rw_speeds);
            rw_speeds_out.connect_to(&mut module.rw_speeds_in_msg);
            let gyro_out = Output::new(AccelerometerDataMsg::default());
            gyro_out.connect_to(&mut module.gyr_buff_in_msg);
            Self {
                module,
                st_out,
                rw_speeds_out,
                gyro_out,
            }
        }

        /// One Python loop iteration: `ExecuteSimulation` up to
        /// `(iteration + 1) * 0.5 s`, which also runs the t = 0 tick first.
        fn execute_iteration(&mut self, iteration: u64) {
            let first_tick = if iteration == 0 { 0 } else { iteration + 1 };
            for tick in first_tick..=iteration + 1 {
                self.module.update(&context(tick * TASK_NANOS));
            }
        }
    }

    fn assert_close(actual: f64, expected: f64, relative: f64, what: &str) {
        let tolerance = relative * expected.abs().max(1.0e-12);
        assert!(
            (actual - expected).abs() <= tolerance,
            "{what}: {actual:e} differs from {expected:e}"
        );
    }

    #[test]
    fn matches_upstream_filter_methods() {
        // inertialStateProp on a fresh configuration: no wheels, so the rate
        // is unchanged.
        let module = InertialUkf::new(InertialUkfConfig::default());
        let state_input = StateVector::from_column_slice(&[1.0, 0.0, 0.0, 0.1, 0.1, 0.1]);
        let mut state = state_input;
        module.state_prop(&mut state, 0.5);
        let wheel_speed_delta = [-5.0, 5.0];
        let wheel_inertia = [1.0, 1.0];
        let wheel_speed_dt = 1.0;
        let wheel_torque: Vec<f64> = (0..2)
            .map(|wheel| wheel_speed_delta[wheel] / wheel_speed_dt * wheel_inertia[wheel])
            .collect();
        let ang_accel = -0.5 * (wheel_torque[0] + wheel_torque[1]) * Vector3::x();
        let expected_rate = state_input.fixed_rows::<3>(3) + ang_accel;
        assert!((expected_rate - state.fixed_rows::<3>(3)).norm() <= 1.0e-10);

        // Read_STMessages ordering: three trackers (upstream's fresh
        // STMessage() list, so zero noise) written before the run.
        let mut config = setup_filter_data(3);
        config.st_noise = vec![Matrix3::zeros(); 3];
        let fixture = Fixture::new(config, RwArrayConfigMsg::default(), RwSpeedMsg::default());
        let mut module = fixture.module;
        for (output, (tag_s, sigma)) in fixture.st_out.iter().zip([
            (1.25, Vector3::new(0.1, 0.2, 0.3)),
            (1.0, Vector3::new(0.2, 0.2, 0.3)),
            (0.75, Vector3::new(0.3, 0.2, 0.3)),
        ]) {
            output.write(StarTrackerAttitudeMsg {
                time_tag_nanos: sec2nano(tag_s),
                sigma_bn: sigma,
            });
        }
        module.init();
        module.update(&context(0));
        assert_eq!(module.st_sensor_order, [2, 1, 0, 0]);
    }

    #[derive(Clone, Copy)]
    struct StateUpdateCase {
        /// Star trackers are written from loop iteration `> first_write_after`.
        first_write_after: u64,
        max_time_jump_s: f64,
        with_wheels: bool,
        /// Compiled-upstream state and covariance diagonal at t = 10000 s and
        /// at t = 20000 s.
        reference: [([f64; 6], [f64; 6]); 2],
    }

    fn run_state_update_case(case: StateUpdateCase) {
        let attitude = Vector3::new(0.3, 0.4, 0.5);
        let mut config = setup_filter_data(2);
        config.max_time_jump_s = case.max_time_jump_s;
        let mut rw_config = RwArrayConfigMsg::default();
        let mut rw_speeds = RwSpeedMsg::default();
        if case.with_wheels {
            // Upstream sets only numRW: the spin axes and inertias are zero.
            rw_config.num_rw = 3;
            rw_speeds.wheel_speeds_radps[..3].copy_from_slice(&[0.1, 0.01, 0.1]);
        }
        let mut fixture = Fixture::new(config, rw_config, rw_speeds.clone());
        fixture.module.init();

        let mut covar_t0 = StateMatrix::zeros();
        for iteration in 0..20_000 {
            if iteration > case.first_write_after {
                for output in &fixture.st_out {
                    output.write(StarTrackerAttitudeMsg {
                        time_tag_nanos: iteration * TASK_NANOS,
                        sigma_bn: attitude,
                    });
                }
            }
            if case.with_wheels && iteration == 10_000 {
                rw_speeds.wheel_speeds_radps[..3].copy_from_slice(&[0.5, 0.1, 0.05]);
                fixture.rw_speeds_out.write(rw_speeds.clone());
            }
            fixture.execute_iteration(iteration);
            if iteration == 0 {
                // The 5 s logger's first sample, at t = 0, is still covarInit.
                covar_t0 = fixture.module.covar;
            }
        }
        let accuracy = 1.0e-5;
        for axis in 0..3 {
            assert!(fixture.module.covar[(axis, axis)] <= covar_t0[(axis, axis)]);
            assert!((fixture.module.state[axis] - attitude[axis]).abs() <= accuracy);
        }
        assert_reference(&fixture.module, case.reference[0]);

        let switched_attitude = Vector3::new(1.2, 0.0, 0.0);
        for step in 0..20_000 {
            let iteration = step + 20_000;
            if step > 20 {
                fixture.st_out[0].write(StarTrackerAttitudeMsg {
                    time_tag_nanos: iteration * (TASK_NANOS / 2),
                    sigma_bn: switched_attitude,
                });
                fixture.st_out[1].write(StarTrackerAttitudeMsg {
                    time_tag_nanos: iteration * TASK_NANOS,
                    sigma_bn: switched_attitude,
                });
            }
            fixture.execute_iteration(iteration);
        }
        for axis in 0..3 {
            assert!(fixture.module.covar[(axis, axis)] <= covar_t0[(axis, axis)]);
        }
        assert_reference(&fixture.module, case.reference[1]);
    }

    fn assert_reference(module: &InertialUkf, (state, covar_diagonal): ([f64; 6], [f64; 6])) {
        for index in 0..6 {
            // Absolute floor for the components that are zero up to rounding.
            let state_error = (module.state[index] - state[index]).abs();
            assert!(
                state_error <= 1.0e-9 * state[index].abs() + 1.0e-15,
                "state[{index}]: {:e} differs from {:e}",
                module.state[index],
                state[index]
            );
            assert_close(
                module.covar[(index, index)],
                covar_diagonal[index],
                1.0e-9,
                "covariance diagonal",
            );
        }
    }

    #[test]
    fn matches_upstream_state_update_inertial_attitude() {
        run_state_update_case(StateUpdateCase {
            first_write_after: 21,
            max_time_jump_s: 10.0,
            with_wheels: false,
            reference: [
                (
                    [
                        0.300_000_000_069_134_96,
                        0.400_000_000_092_145_5,
                        0.500_000_000_115_074_3,
                        5.765_257_138_446_672e-10,
                        7.685_427_548_524_695e-10,
                        9.606_980_878_504_156e-10,
                    ],
                    [
                        1.929_348_234_641_349e-8,
                        1.929_348_234_641_350_4e-8,
                        1.929_348_234_644_369_3e-8,
                        7.761_355_054_514_619e-8,
                        7.761_355_054_560_667e-8,
                        7.761_355_054_572_527e-8,
                    ],
                ),
                (
                    [
                        0.938_509_515_071_689_1,
                        -2.346_156_847_542_120_7e-18,
                        1.964_888_164_067_761e-19,
                        7.818_856_822_506_688,
                        1.691_105_060_470_007_8e-18,
                        3.282_168_167_025_233_2e-18,
                    ],
                    [
                        5.711_730_544_228_786_4e-8,
                        7.148_707_690_430_22e-8,
                        7.148_707_690_430_25e-8,
                        1.058_522_721_992_561_2e-7,
                        2.934_351_915_937_001e-7,
                        2.934_351_915_937_003_6e-7,
                    ],
                ),
            ],
        });
    }

    #[test]
    fn matches_upstream_state_update_rw_inertial_attitude() {
        run_state_update_case(StateUpdateCase {
            first_write_after: 20,
            max_time_jump_s: 0.0,
            with_wheels: true,
            reference: [
                (
                    [
                        0.300_000_000_069_153_5,
                        0.400_000_000_092_258_94,
                        0.500_000_000_115_215_7,
                        5.764_478_804_532_529e-10,
                        7.686_335_544_145_198e-10,
                        9.608_386_632_703_053e-10,
                    ],
                    [
                        1.929_348_234_641_349_7e-8,
                        1.929_348_234_641_349_7e-8,
                        1.929_348_234_644_37e-8,
                        7.761_355_054_543_826e-8,
                        7.761_355_054_518_532e-8,
                        7.761_355_054_539_029e-8,
                    ],
                ),
                (
                    [
                        0.938_509_515_075_401_9,
                        7.171_895_842_853_304e-20,
                        2.236_027_424_597_259e-18,
                        7.818_856_822_538_129,
                        1.191_941_847_512_476_3e-18,
                        3.667_617_216_891_465e-20,
                    ],
                    [
                        5.711_730_544_225_505e-8,
                        7.148_707_690_432_359e-8,
                        7.148_707_690_432_359e-8,
                        1.058_522_721_992_422_7e-7,
                        2.934_351_915_945_198_6e-7,
                        2.934_351_915_945_194e-7,
                    ],
                ),
            ],
        });
    }

    #[test]
    fn matches_upstream_fault_scenarios() {
        // Upstream runs these on a configuration that was never reset, so
        // lambda and gamma are zero. Its measurement updates read tracker 1,
        // the zeroed slot behind its single tracker: here a second tracker
        // with zero noise.
        let mut module = InertialUkf::new(InertialUkfConfig {
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            switch_mag: 1.2,
            st_noise: vec![Matrix3::identity(), Matrix3::zeros()],
            ..InertialUkfConfig::default()
        });

        // Cleaning restores the saved estimate.
        module.s_bar_prev = StateMatrix::identity();
        module.covar_prev = StateMatrix::identity() * 2.0;
        module.clean_update();
        assert_eq!(module.covar, module.covar_prev);
        assert_eq!(module.state, module.state_prev);
        assert_eq!(module.s_bar, module.s_bar_prev);

        // Negative sigma-point weights fail the time update, which restores
        // the estimate and the standard weights; with those, the zero tracker
        // noise fails the measurement update.
        module.weights.w_c = SVector::repeat(-1.0);
        module.weights.w_m = SVector::repeat(-1.0);
        assert!(module.time_update(1.0).is_err());
        assert_eq!(module.covar, StateMatrix::identity() * 2.0);
        assert!(module.weights.w_c.iter().all(|&weight| weight > 0.0));
        assert!(module.meas_update(1).is_err());

        // Upstream expects a negative `qNoise` to fail the next time update,
        // but `qNoise` only enters through `Reset`: with unit weights the
        // update succeeds (which is why upstream disabled the test), and the
        // measurement update still fails on the zero noise.
        module.weights.w_c = SVector::repeat(1.0);
        module.weights.w_m = SVector::repeat(1.0);
        module.config.q_noise = -StateMatrix::identity();
        assert!(module.time_update(1.0).is_ok());
        assert!(module.meas_update(1).is_err());
    }

    #[test]
    fn a_negative_weight_fails_the_measurement_update_without_cleaning_up() {
        let state = StateVector::from_column_slice(&[0.1, 0.2, 0.3, 0.01, 0.02, 0.03]);
        let mut module = InertialUkf::new(setup_filter_data(1));
        module.state = state;
        module.weights.w_c = SVector::repeat(-1.0);
        assert!(module.meas_update(0).is_err());
        // Upstream returns before restoring the standard weights.
        assert!(module.weights.w_c.iter().all(|&weight| weight == -1.0));
        assert_eq!(module.state, state);
    }

    #[test]
    fn matches_upstream_state_prop_rate_inertial_attitude() {
        let rate_radps = PI / 18.0;
        let mut config = setup_filter_data(1);
        config.state_init = StateVector::from_column_slice(&[0.0, 0.0, 0.0, rate_radps, 0.0, 0.0]);
        let mut fixture = Fixture::new(config, RwArrayConfigMsg::default(), RwSpeedMsg::default());
        fixture.module.init();
        // Upstream writes the only star-tracker sample after initialization.
        fixture.st_out[0].write(StarTrackerAttitudeMsg {
            time_tag_nanos: 1_000_000_000,
            sigma_bn: Vector3::zeros(),
        });
        let mut covar_t0 = StateMatrix::zeros();
        let mut buffer = AccelerometerDataMsg::default();
        // The 5 s logger's last sample is the t = 3600 s tick, which iteration
        // 7199 runs.
        for iteration in 0..7200 {
            let packet = &mut buffer.accel_packets[iteration as usize % MAX_ACC_BUF_PKT];
            packet.meas_time_nanos = iteration * TASK_NANOS;
            packet.gyro_b_radps = Vector3::new(rate_radps, 0.0, 0.0);
            fixture.gyro_out.write(buffer.clone());
            fixture.execute_iteration(iteration);
            if iteration == 0 {
                covar_t0 = fixture.module.covar;
            }
        }

        let omega = fixture.module.nav_state_out_msg.read().omega_bn_b_radps;
        let accuracy = 1.0e-3;
        for (axis, expected) in [rate_radps, 0.0, 0.0].into_iter().enumerate() {
            assert!((omega[axis] - expected).abs() <= accuracy);
        }
        // Why upstream's covariance check fails: nothing after the single
        // star-tracker update changes the covariance.
        assert_eq!(fixture.module.covar, covar_t0);
    }

    #[test]
    fn publishes_nothing_before_the_first_star_tracker_measurement() {
        let mut fixture = Fixture::new(
            setup_filter_data(2),
            RwArrayConfigMsg::default(),
            RwSpeedMsg::default(),
        );
        fixture.module.init();
        for tick in 0..4 {
            fixture.module.update(&context(tick * TASK_NANOS));
        }
        assert!(!fixture.module.first_pass_complete);
        assert_eq!(
            fixture.module.filt_data_out_msg.read(),
            InertialFilterMsg::default()
        );
        assert_eq!(fixture.module.nav_state_out_msg.read().time_tag_s, 0.0);
    }

    #[test]
    fn propagates_the_published_attitude_on_the_gyros_between_star_tracker_updates() {
        let mut config = setup_filter_data(1);
        config.state_init = StateVector::zeros();
        let mut fixture = Fixture::new(config, RwArrayConfigMsg::default(), RwSpeedMsg::default());
        fixture.module.init();

        // A star tracker at the identity starts the filter at t = 1 s.
        fixture.st_out[0].write(StarTrackerAttitudeMsg {
            time_tag_nanos: 1_000_000_000,
            sigma_bn: Vector3::zeros(),
        });
        fixture.module.update(&context(1_000_000_000));
        let nav = fixture.module.nav_state_out_msg.read();
        assert_eq!(nav.time_tag_s, 1.0);
        assert!(nav.sigma_bn.norm() <= 1.0e-15);

        // Ten gyro packets of 0.1 rad/s about z cover the next second.
        let rate = Vector3::new(0.0, 0.0, 0.1);
        let mut buffer = AccelerometerDataMsg::default();
        for (index, packet) in buffer.accel_packets.iter_mut().take(10).enumerate() {
            packet.meas_time_nanos = 1_100_000_000 + index as u64 * 100_000_000;
            packet.gyro_b_radps = rate;
        }
        fixture.gyro_out.write(buffer);
        fixture.module.update(&context(2_000_000_000));

        let nav = fixture.module.nav_state_out_msg.read();
        assert_eq!(fixture.module.num_used_gyros, 10);
        assert!((nav.time_tag_s - 2.0).abs() <= 1.0e-12);
        let rotation_rad: f64 = 0.1;
        let expected_sigma = Vector3::new(0.0, 0.0, (rotation_rad / 4.0).tan());
        assert!((nav.sigma_bn - expected_sigma).norm() <= 1.0e-12);
        let mut filter = LowPassFilter::new(0.5, 15.0 / (2.0 * PI));
        let filtered_rate = (0..10).map(|_| filter.filter(rate.z)).last();
        assert_eq!(nav.omega_bn_b_radps.x, 0.0);
        assert_eq!(nav.omega_bn_b_radps.y, 0.0);
        assert_eq!(Some(nav.omega_bn_b_radps.z), filtered_rate);
    }

    #[test]
    fn freshness_follows_payload_changes() {
        let mut fixture = Fixture::new(
            setup_filter_data(1),
            RwArrayConfigMsg::default(),
            RwSpeedMsg::default(),
        );
        fixture.module.init();
        let measurement = StarTrackerAttitudeMsg {
            time_tag_nanos: 500_000_000,
            sigma_bn: Vector3::new(0.1, 0.0, 0.0),
        };
        fixture.st_out[0].write(measurement.clone());
        fixture.module.update(&context(TASK_NANOS));
        assert!(fixture.module.trackers[0].is_fresh);
        // Rewriting the same sample is not a new measurement.
        fixture.st_out[0].write(measurement);
        fixture.module.update(&context(2 * TASK_NANOS));
        assert!(!fixture.module.trackers[0].is_fresh);
        assert_eq!(
            fixture.module.trackers[0].writes.written_nanos(),
            Some(TASK_NANOS)
        );
    }
}
