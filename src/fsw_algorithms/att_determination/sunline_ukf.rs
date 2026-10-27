//! Coarse-sun-sensor sunline estimator: a square-root unscented Kalman filter
//! for the body-frame sun heading and its rate. Port of upstream Basilisk
//! `sunlineUKF` (`fswAlgorithms/attDetermination/sunlineUKF`).
//!
//! The six states are the sun heading `d` (body components, scaled like the
//! CSS cosines) and its body-frame rate `d'`. A time update propagates every
//! sigma point with `d += dt (d' - (d'·d̂) d̂)` — the rate along the heading is
//! unobservable with sun sensors and is removed — and adds the process noise
//! `q_noise` once, whatever the step. The measurement model predicts
//! `CBias n̂ · d` for every sensor whose reading exceeds `sensor_use_thresh`,
//! with independent noise of variance `q_obs_val`.
//!
//! Every update, as upstream: if the CSS message is not older than the
//! estimate, propagate to its time and apply it; propagate to the call time if
//! that is later; publish the unit heading on `nav_state_out_msg` and the
//! state, covariance and post-fit residuals on `filt_data_out_msg`.
//!
//! Upstream behaviour kept as is: the estimate is the propagated *zeroth*
//! sigma point, not the sigma-point mean, and a reset refactors the current
//! covariance around the current state instead of re-initializing them.
//!
//! # Message timing
//!
//! Upstream tells a new measurement from the CSS message header (`isWritten`,
//! `timeWritten`). Rust ports carry no header, so a connected
//! `css_data_in_msg` counts as written at the current update — what upstream
//! sees when the CSS producer runs before the filter on every tick. The
//! header-driven logic itself is unchanged, with the header as an explicit
//! argument, and the tests drive it with upstream's exact header values.

use nalgebra::{DMatrix, DVector, Matrix6, SVector, Vector3, Vector6};

use crate::messages::{
    CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, Input, MAX_N_CSS_MEAS,
    NavigationAttitudeMsg, Output, SunlineFilterMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::{Module, SimulationContext};

pub(crate) mod square_root_ukf;

use square_root_ukf::{
    NUM_SIGMA_POINTS, SquareRootUkf, configured_css_units, css_written_nanos, heading,
};

/// User-set parameters of [`SunlineUkf`] (the upstream `SunlineUKFConfig`
/// fields set from Python). The defaults are upstream's zero-initialized
/// values, which do not make a working filter.
#[derive(Clone, Debug, Default)]
pub struct SunlineUkfConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `alpha` \[-\]: sigma-point spread.
    pub alpha: f64,
    /// Upstream `beta` \[-\]: prior-distribution parameter (2 for a Gaussian).
    pub beta: f64,
    /// Upstream `kappa` \[-\]: secondary spread parameter.
    pub kappa: f64,
    /// Upstream `state`: the starting estimate, heading `d` \[-\] then rate
    /// `d'` \[1/s\], taken by [`SunlineUkf::new`] and again by `init` (upstream
    /// initialization reads whatever the user set). As upstream, a reset keeps
    /// the current estimate instead.
    pub state: Vector6<f64>,
    /// Upstream `covar` \[-\]: the starting covariance (row-major upstream),
    /// taken like `state`. As upstream, a reset refactors the current
    /// covariance instead.
    pub covar: Matrix6<f64>,
    /// Upstream `qNoise` \[-\]: process noise covariance, added once per time
    /// update.
    pub q_noise: Matrix6<f64>,
    /// Upstream `qObsVal` \[-\]: variance of each CSS measurement.
    pub q_obs_val: f64,
    /// Upstream `sensorUseThresh` \[-\]: readings at or below this are not
    /// used.
    pub sensor_use_thresh: f64,
}

/// The `sunlineUKF` module.
#[derive(Clone, Debug)]
pub struct SunlineUkf {
    /// User-set parameters.
    pub config: SunlineUkfConfig,
    /// Upstream `cssDataInMsg`: CSS readings.
    pub css_data_in_msg: Input<CssArraySensorMsg>,
    /// Upstream `cssConfigInMsg`: CSS constellation, read at reset.
    pub css_config_in_msg: Input<CssConfigMsg>,
    /// Upstream `navStateOutMsg`: unit sun heading and its time tag.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `filtDataOutMsg`: state, covariance and post-fit residuals.
    pub filt_data_out_msg: Output<SunlineFilterMsg>,
    filter: SquareRootUkf,
    /// Upstream `timeTag` \[s\]: time of the estimate.
    time_tag_s: f64,
    /// Upstream `cssNHat_B` and `CBias` of the `numCSSTotal` sensors.
    css_units: Vec<CssUnitConfigMsg>,
    /// Upstream `cssSensorInBuffer`: the readings of the last update.
    css_readings: CssArraySensorMsg,
    /// Upstream `obs`: the readings the last measurement model used, in its
    /// leading entries; the others keep older values.
    observations: SVector<f64, MAX_N_CSS_MEAS>,
    /// Upstream `yMeas`: predicted reading per used sensor (row) and sigma
    /// point (column). Its row count is upstream `numObs`.
    predictions: DMatrix<f64>,
}

impl SunlineUkf {
    /// A filter at the configured starting estimate, with unconnected ports.
    pub fn new(config: SunlineUkfConfig) -> Self {
        Self {
            filter: SquareRootUkf::new(config.state, config.covar),
            config,
            css_data_in_msg: Input::default(),
            css_config_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            time_tag_s: 0.0,
            css_units: Vec::new(),
            css_readings: CssArraySensorMsg::default(),
            observations: SVector::zeros(),
            predictions: DMatrix::zeros(0, NUM_SIGMA_POINTS),
        }
    }

    /// Upstream `Reset_sunlineUKF`: reads the CSS configuration and rebuilds
    /// the filter matrices around the current estimate and covariance.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.css_config_in_msg.is_connected() {
            log::error!("sunlineUKF.css_config_in_msg is not connected");
        }
        if !self.css_data_in_msg.is_connected() {
            log::error!("sunlineUKF.css_data_in_msg is not connected");
        }
        self.css_units = configured_css_units(&self.css_config_in_msg.read(), "sunlineUKF");
        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.observations = SVector::zeros();
        // sunlineUKF ignores the Cholesky status; a failed factor keeps
        // upstream's partial output.
        self.filter.reset(
            self.config.alpha,
            self.config.beta,
            self.config.kappa,
            &self.config.q_noise,
        );
    }

    /// Upstream `Update_sunlineUKF`, with the CSS message header explicit:
    /// `written_nanos` is upstream `timeWritten` when the message `isWritten`.
    fn step(
        &mut self,
        call_time_nanos: u64,
        readings: CssArraySensorMsg,
        written_nanos: Option<u64>,
    ) {
        self.css_readings = readings;
        let measurement_time_s = written_nanos.map(|nanos| nanos as f64 * NANO2SEC);
        if let Some(measurement_time_s) = measurement_time_s
            && measurement_time_s >= self.time_tag_s
        {
            self.time_update(measurement_time_s);
            self.measurement_update();
        }
        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.time_tag_s {
            self.time_update(call_time_s);
        }

        self.measurement_model();
        let y_bar = self.filter.predicted_mean(&self.predictions);
        // Upstream subtracts all MAX_N_CSS_MEAS entries of yBar but fills only
        // the used ones; the rest is uninitialized stack memory (an upstream
        // bug). The port takes it as zero, so those residuals repeat the stale
        // observations.
        let post_fit_res = SVector::from_fn(|index, _| {
            self.observations[index] - y_bar.get(index).copied().unwrap_or(0.0)
        });

        self.nav_state_out_msg.write(NavigationAttitudeMsg {
            time_tag_s: self.time_tag_s,
            vehicle_sun_point_body: normalize_or_zero(heading(&self.filter.state)),
            ..Default::default()
        });
        self.filt_data_out_msg.write(SunlineFilterMsg {
            time_tag_s: self.time_tag_s,
            covar: self.filter.covar,
            state: self.filter.state,
            post_fit_res,
            num_obs: self.predictions.nrows(),
            ..Default::default()
        });
    }

    /// Upstream `sunlineUKFTimeUpdate`: propagates the estimate to
    /// `update_time_s`.
    fn time_update(&mut self, update_time_s: f64) {
        let dt_s = update_time_s - self.time_tag_s;
        self.filter
            .propagate_sigma_points(|point| propagate(point, dt_s));
        // sunlineUKF ignores a failed down-date.
        self.filter.update_time_covariance();
        self.time_tag_s = update_time_s;
    }

    /// Upstream `sunlineUKFMeasUpdate`: applies the current CSS readings.
    fn measurement_update(&mut self) {
        self.measurement_model();
        let observations =
            DVector::from_column_slice(&self.observations.as_slice()[..self.predictions.nrows()]);
        // sunlineUKF ignores the factorization statuses.
        self.filter
            .measurement_update(&self.predictions, &observations, self.config.q_obs_val);
    }

    /// Upstream `sunlineUKFMeasModel`: selects the readings above the
    /// threshold and predicts them at every sigma point of the last time
    /// update.
    fn measurement_model(&mut self) {
        let threshold = self.config.sensor_use_thresh;
        let used: Vec<(f64, Vector3<f64>)> = self
            .css_units
            .iter()
            .zip(self.css_readings.cos_values)
            .filter(|&(_, reading)| reading > threshold)
            .map(|(unit, reading)| (reading, unit.n_hat_b * unit.c_bias))
            .collect();
        for (observation, (reading, _)) in self.observations.iter_mut().zip(&used) {
            *observation = *reading;
        }
        let sigma_points = &self.filter.sigma_points;
        self.predictions = DMatrix::from_fn(used.len(), NUM_SIGMA_POINTS, |row, column| {
            heading(&sigma_points.column(column)).dot(&used[row].1)
        });
    }
}

/// Upstream `sunlineStateProp`: removes the rate component along the heading
/// and steps the heading by `dt_s`.
fn propagate(point: &mut Vector6<f64>, dt_s: f64) {
    let sun_heading = heading(point);
    let unit_heading = normalize_or_zero(sun_heading);
    let rate =
        point.fixed_rows::<3>(3) - unit_heading * point.fixed_rows::<3>(3).dot(&unit_heading);
    point
        .fixed_rows_mut::<3>(0)
        .copy_from(&(sun_heading + rate * dt_s));
    point.fixed_rows_mut::<3>(3).copy_from(&rate);
}

impl Module for SunlineUkf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
        self.filt_data_out_msg.write(SunlineFilterMsg::default());
        // Upstream initialization starts from the `state` and `covar` the user
        // set, also after construction.
        self.filter.state = self.config.state;
        self.filter.covar = self.config.covar;
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        SunlineUkf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let readings = self.css_data_in_msg.read();
        let written_nanos = css_written_nanos(&self.css_data_in_msg, context);
        self.step(context.current_sim_nanos, readings, written_nanos);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `_UnitTest/test_SunLineUKF.py`: all three
    //! `test_all_sunline_kf` parametrizations (`sunline_utilities_test`,
    //! `checkStatePropSunLine`, `checkStateUpdateSunLine`) with upstream's
    //! truth and tolerances. The plots (`SunLineuKF_test_utilities.py`) are not
    //! ported.
    //!
    //! The pytests feed the filter through the CSS message header: readings
    //! are stamped with the start of a task step and read at its end.
    //! [`UpstreamTimeline`] replays that timeline through the private `step`,
    //! so the filter sees the header values upstream sees.

    use std::cmp::Ordering;

    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix6, Vector3, Vector6};

    use super::square_root_ukf::test_support::{TASK_RATE_NANOS, UpstreamTimeline};
    use super::{SunlineUkf, SunlineUkfConfig, propagate};
    use crate::messages::{
        CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, Output, SunlineFilterMsg,
    };
    use crate::utilities::ukf_utilities::{
        ukf_chol_decomp, ukf_l_inv, ukf_lu_bck_slv, ukf_lud, ukf_mat_inv, ukf_qrd_just_r, ukf_u_inv,
    };
    use crate::{Module, SimulationContext};

    /// Upstream `CSSOrientationList`, verbatim: its `...746` entries are one ulp
    /// below `FRAC_1_SQRT_2`, its `...757` ones equal it.
    #[allow(clippy::approx_constant, clippy::excessive_precision)]
    const CSS_NORMALS: [[f64; 3]; 8] = [
        [0.70710678118654746, -0.5, 0.5],
        [0.70710678118654746, -0.5, -0.5],
        [0.70710678118654746, 0.5, -0.5],
        [0.70710678118654746, 0.5, 0.5],
        [-0.70710678118654746, 0.0, 0.70710678118654757],
        [-0.70710678118654746, 0.70710678118654757, 0.0],
        [-0.70710678118654746, 0.0, -0.70710678118654757],
        [-0.70710678118654746, -0.70710678118654757, 0.0],
    ];

    /// `macros.sec2nano(steps * 0.5)`: the pytests' stop times.
    fn half_seconds(steps: u64) -> u64 {
        steps * 500_000_000
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `setupFilterData`.
    fn upstream_config() -> SunlineUkfConfig {
        let heading_noise = 0.01 * 0.01;
        let rate_noise = 0.001 * 0.001;
        SunlineUkfConfig {
            name: "SunlineUKF".to_string(),
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state: Vector6::new(1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            covar: Matrix6::from_diagonal(&Vector6::new(0.4, 0.4, 0.4, 0.04, 0.04, 0.04)),
            q_noise: Matrix6::from_diagonal(&Vector6::new(
                heading_noise,
                heading_noise,
                heading_noise,
                rate_noise,
                rate_noise,
                rate_noise,
            )),
            q_obs_val: 0.001,
            sensor_use_thresh: 0.0,
        }
    }

    /// Upstream `cssConstelation`: the eight normals with unit `CBias`.
    fn css_constellation() -> CssConfigMsg {
        let units: Vec<CssUnitConfigMsg> = CSS_NORMALS
            .iter()
            .map(|normal| CssUnitConfigMsg {
                n_hat_b: Vector3::from(*normal),
                c_bias: 1.0,
            })
            .collect();
        CssConfigMsg::from_active(&units)
    }

    /// Upstream `dotList`: every normal dotted with `sun_vector`.
    fn css_readings(sun_vector: &Vector3<f64>) -> CssArraySensorMsg {
        let readings: Vec<f64> = CSS_NORMALS
            .iter()
            .map(|normal| Vector3::from(*normal).dot(sun_vector))
            .collect();
        CssArraySensorMsg::from_active(&readings)
    }

    /// The module subscribed to `css_config` and `css_data`, then initialized
    /// (`InitializeSimulation`).
    fn subscribed_filter(
        config: SunlineUkfConfig,
        css_config: CssConfigMsg,
        css_data: &Output<CssArraySensorMsg>,
    ) -> SunlineUkf {
        let mut module = SunlineUkf::new(config);
        Output::new(css_config).connect_to(&mut module.css_config_in_msg);
        css_data.connect_to(&mut module.css_data_in_msg);
        module.init();
        module
    }

    /// `ExecuteSimulation` up to `stop_nanos` with `filtDataOutMsg` recorded
    /// every tick (`dataLog`).
    fn execute_until(
        timeline: &mut UpstreamTimeline,
        module: &mut SunlineUkf,
        log: &mut Vec<SunlineFilterMsg>,
        stop_nanos: u64,
    ) {
        timeline.execute_until(stop_nanos, |call_nanos, readings, written_nanos| {
            module.step(call_nanos, readings, written_nanos);
            log.push(module.filt_data_out_msg.read());
        });
    }

    #[test]
    fn matches_upstream_sunline_utilities_test() {
        // numpy `qr` R, rows flipped to a positive diagonal as the pytest does.
        let numpy_r = |a: &DMatrix<f64>| {
            let mut r = a.clone().qr().r();
            for i in 0..r.nrows() {
                if r[(i, i)] < 0.0 {
                    r.row_mut(i).neg_mut();
                }
            }
            r
        };
        let qr_cases: [(usize, usize, &[f64], f64); 3] = [
            (
                6,
                4,
                &[
                    0.488894, 0.888396, 0.325191, 0.319207, 1.03469, -1.14707, -0.754928, 0.312859,
                    0.726885, -1.06887, 1.3703, -0.86488, -0.303441, -0.809499, -1.71152,
                    -0.0300513, 0.293871, -2.94428, -0.102242, -0.164879, -0.787283, 1.43838,
                    -0.241447, 0.627707,
                ],
                1.0e-15,
            ),
            (
                5,
                4,
                &[
                    1.09327,
                    1.10927,
                    -0.863653,
                    1.32288,
                    -1.21412,
                    -1.1135,
                    -0.00684933,
                    -2.43508,
                    -0.769666,
                    0.371379,
                    -0.225584,
                    -1.76492,
                    -1.08906,
                    0.0325575,
                    0.552527,
                    -1.6256,
                    1.54421,
                    0.0859311,
                    -1.49159,
                    1.59683,
                ],
                1.0e-14,
            ),
            (
                6,
                2,
                &[
                    0.2236, 0.0, 0.0, 0.2236, -0.2236, 0.0, 0.0, -0.2236, 0.0170, 0.0, 0.0, 0.0170,
                ],
                1.0e-15,
            ),
        ];
        for (rows, cols, values, accuracy) in qr_cases {
            let a = DMatrix::from_row_slice(rows, cols, values);
            let error = (ukf_qrd_just_r(&a) - numpy_r(&a)).norm();
            assert!(
                error <= accuracy,
                "QR Decomposition accuracy failure ({rows}x{cols}): {error:e}"
            );
        }

        // LU factors, multiplied back and un-permuted with the pytest's loop.
        let magic = DMatrix::from_row_slice(3, 3, &[8.0, 1.0, 6.0, 3.0, 5.0, 7.0, 4.0, 9.0, 2.0]);
        let lu = ukf_lud(&magic).expect("the magic square is regular");
        let lower = DMatrix::from_fn(3, 3, |i, j| match i.cmp(&j) {
            Ordering::Greater => lu.lu[(i, j)],
            Ordering::Equal => 1.0,
            Ordering::Less => 0.0,
        });
        let mut product = lower * lu.lu.upper_triangle();
        let mut swapped = DMatrix::zeros(3, 3);
        for i in 0..3 {
            let current_row = lu.row_swaps[i];
            swapped.set_row(i, &product.row(current_row));
            let row_i = product.row(i).into_owned();
            product.set_row(current_row, &row_i);
        }
        assert!(
            (swapped - &magic).norm() <= 1.0e-14,
            "LU Decomposition accuracy failure"
        );

        let equations =
            DMatrix::from_row_slice(3, 3, &[2.0, 1.0, 3.0, 2.0, 6.0, 8.0, 6.0, 8.0, 18.0]);
        let lu = ukf_lud(&equations).expect("the system is regular");
        let solution = ukf_lu_bck_slv(&lu, &DVector::from_vec(vec![1.0, 3.0, 5.0]))
            .expect("the system is regular");
        let error: f64 = solution
            .iter()
            .zip([3.0 / 10.0, 4.0 / 10.0, 0.0])
            .map(|(x, expected)| (x - expected).abs())
            .sum();
        assert!(error <= 1.0e-14, "LU Back-Solve accuracy failure");

        let inverse = ukf_mat_inv(&magic).expect("the magic square is regular");
        assert!(
            (inverse * &magic - DMatrix::identity(3, 3)).norm() <= 1.0e-14,
            "LU Matrix Inverse accuracy failure"
        );

        let symmetric =
            DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]);
        let numpy_cholesky = symmetric
            .clone()
            .cholesky()
            .expect("the matrix is positive definite")
            .l();
        let cholesky = ukf_chol_decomp(&symmetric).expect("the matrix is positive definite");
        assert!(
            (cholesky - numpy_cholesky).norm() <= 1.0e-14,
            "Cholesky Matrix Decomposition accuracy failure"
        );

        #[allow(clippy::excessive_precision)]
        let lower = DMatrix::from_row_slice(
            4,
            4,
            &[
                2.1950926119414667,
                0.0,
                0.0,
                0.0,
                1.0974804773131115,
                1.9010439702743847,
                0.0,
                0.0,
                0.0,
                1.2672359635912551,
                1.7923572711881284,
                0.0,
                1.0974804773131113,
                -0.63357997864171967,
                1.7920348101787789,
                0.033997451205364251,
            ],
        );
        let lower_inverse = ukf_l_inv(&lower).expect("the diagonal is non-zero");
        assert!(
            (lower_inverse * &lower - DMatrix::identity(4, 4)).norm() <= 1.0e-12,
            "L Matrix Inverse accuracy failure"
        );
        let upper = lower.transpose();
        let upper_inverse = ukf_u_inv(&upper).expect("the diagonal is non-zero");
        assert!(
            (upper_inverse * &upper - DMatrix::identity(4, 4)).norm() <= 1.0e-12,
            "U Matrix Inverse accuracy failure"
        );
    }

    #[test]
    fn matches_upstream_check_state_prop_sun_line() {
        // Neither input message is ever written.
        let mut module = subscribed_filter(
            upstream_config(),
            CssConfigMsg::default(),
            &Output::default(),
        );
        let mut timeline = UpstreamTimeline::new();
        let mut log = Vec::new();
        execute_until(&mut timeline, &mut module, &mut log, half_seconds(16_000));

        let (first, last) = (&log[0], &log[log.len() - 1]);
        for i in 0..6 {
            let drift = (last.state[i] - first.state[i]).abs();
            assert!(
                drift <= 1.0e-10,
                "State propagation failure: state {i} moved by {drift:e}"
            );
        }
    }

    #[test]
    fn matches_upstream_check_state_update_sun_line() {
        let mut config = upstream_config();
        config.state = Vector6::new(0.7, 0.7, 0.0, 0.0, 0.0, 0.0);
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());
        let mut timeline = UpstreamTimeline::new();
        let mut log = Vec::new();

        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let readings = css_readings(&sun_vector);
        for i in 0..400 {
            if i > 20 {
                timeline.write(readings.clone());
            }
            execute_until(&mut timeline, &mut module, &mut log, half_seconds(i + 1));
        }
        assert_state_update(&log, &sun_vector);

        let sun_vector = Vector3::new(-0.8, -0.9, 0.0);
        let readings = css_readings(&sun_vector);
        for i in 0..400 {
            if i > 20 {
                timeline.write(readings.clone());
            }
            execute_until(&mut timeline, &mut module, &mut log, half_seconds(i + 401));
        }
        assert_state_update(&log, &sun_vector);
    }

    /// Upstream's checks: every variance down to at most 1 % of its first
    /// logged value, and the state within 1e-5 of the sun vector with zero
    /// rate.
    fn assert_state_update(log: &[SunlineFilterMsg], sun_vector: &Vector3<f64>) {
        let (first, last) = (&log[0], &log[log.len() - 1]);
        let target = Vector6::new(sun_vector.x, sun_vector.y, sun_vector.z, 0.0, 0.0, 0.0);
        for i in 0..6 {
            assert!(
                last.covar[(i, i)] <= first.covar[(i, i)] / 100.0,
                "Covariance update failure: variance {i} is {:e}",
                last.covar[(i, i)]
            );
            let error = (last.state[i] - target[i]).abs();
            assert!(
                error <= 1.0e-5,
                "State update failure: state {i} is off by {error:e}"
            );
        }
    }

    #[test]
    fn update_reads_a_connected_css_message_as_written_at_the_call_time() {
        let readings = css_readings(&Vector3::new(-0.7, 0.7, 0.0));
        let css_data = Output::new(readings.clone());
        let mut through_update =
            subscribed_filter(upstream_config(), css_constellation(), &css_data);
        let mut through_step =
            subscribed_filter(upstream_config(), css_constellation(), &Output::default());
        let mut unconnected = SunlineUkf::new(upstream_config());
        let mut unconnected_step = SunlineUkf::new(upstream_config());
        unconnected.init();
        unconnected_step.init();

        for tick in 0..20 {
            let now = tick * TASK_RATE_NANOS;
            through_update.update(&context(now));
            through_step.step(now, readings.clone(), Some(now));
            assert_eq!(
                through_update.filt_data_out_msg.read(),
                through_step.filt_data_out_msg.read()
            );

            unconnected.update(&context(now));
            unconnected_step.step(now, CssArraySensorMsg::default(), None);
            assert_eq!(
                unconnected.filt_data_out_msg.read(),
                unconnected_step.filt_data_out_msg.read()
            );
        }
        // Sensors 4, 5 and 6 face this sun direction.
        assert_eq!(through_update.filt_data_out_msg.read().num_obs, 3);
    }

    #[test]
    fn propagation_removes_the_rate_along_the_heading() {
        let mut point = Vector6::new(2.0, 0.0, 0.0, 0.3, 0.1, -0.2);
        propagate(&mut point, 0.5);
        assert_eq!(point, Vector6::new(2.0, 0.05, -0.1, 0.0, 0.1, -0.2));
    }

    #[test]
    fn only_readings_above_the_threshold_are_observations() {
        let mut config = upstream_config();
        config.sensor_use_thresh = 0.1;
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());

        // Along -x the four aft sensors read cos 45°.
        module.step(0, css_readings(&Vector3::new(-1.0, 0.0, 0.0)), Some(0));
        let output = module.filt_data_out_msg.read();
        assert_eq!(output.num_obs, 4);
        assert!(
            output
                .post_fit_res
                .rows(4, 28)
                .iter()
                .all(|&res| res == 0.0)
        );

        // Along +y three sensors read above the threshold. The fourth residual
        // repeats the older observation: upstream reads an uninitialized yBar
        // entry there, which the port takes as zero.
        let readings = css_readings(&Vector3::new(0.0, 1.0, 0.0));
        module.step(TASK_RATE_NANOS, readings, Some(TASK_RATE_NANOS));
        let output = module.filt_data_out_msg.read();
        assert_eq!(output.num_obs, 3);
        assert_eq!(output.post_fit_res[3], -CSS_NORMALS[7][0]);
        assert!(
            output
                .post_fit_res
                .rows(4, 28)
                .iter()
                .all(|&res| res == 0.0)
        );
    }

    #[test]
    fn configuring_before_init_equals_configuring_at_construction() {
        let mut configured_later = SunlineUkf::new(SunlineUkfConfig::default());
        configured_later.config = upstream_config();
        Output::new(css_constellation()).connect_to(&mut configured_later.css_config_in_msg);
        Output::<CssArraySensorMsg>::default().connect_to(&mut configured_later.css_data_in_msg);
        configured_later.init();
        let mut configured_at_new =
            subscribed_filter(upstream_config(), css_constellation(), &Output::default());

        let readings = css_readings(&Vector3::new(-0.7, 0.7, 0.0));
        for tick in 0..10 {
            let now = tick * TASK_RATE_NANOS;
            configured_later.step(now, readings.clone(), Some(now));
            configured_at_new.step(now, readings.clone(), Some(now));
            assert_eq!(
                configured_later.filt_data_out_msg.read(),
                configured_at_new.filt_data_out_msg.read()
            );
        }
    }

    /// `|actual - golden| <= 1e-12 |golden| + 1e-18` for every element.
    fn assert_matches_golden(what: &str, actual: &[f64], golden: &[f64]) {
        assert_eq!(actual.len(), golden.len(), "{what}: length");
        for (i, (a, g)) in actual.iter().zip(golden).enumerate() {
            assert!(
                (a - g).abs() <= 1.0e-12 * g.abs() + 1.0e-18,
                "{what}[{i}]: {a:e} vs upstream {g:e}"
            );
        }
    }

    /// Golden `filtDataOutMsg` after 60 iterations of the
    /// `checkStateUpdateSunLine` loop (measurements from the 22nd on), from
    /// upstream `sunlineUKF.c` compiled with gcc `-O2 -ffp-contract=off` and
    /// driven through the same message timeline. The port reproduces it bit
    /// for bit; the tolerance only allows for another libm.
    #[test]
    #[allow(clippy::excessive_precision)]
    fn matches_compiled_upstream_on_the_state_update_timeline() {
        let mut config = upstream_config();
        config.state = Vector6::new(0.7, 0.7, 0.0, 0.0, 0.0, 0.0);
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());
        let mut timeline = UpstreamTimeline::new();
        let mut log = Vec::new();
        let readings = css_readings(&Vector3::new(-0.7, 0.7, 0.0));
        for i in 0..60 {
            if i > 20 {
                timeline.write(readings.clone());
            }
            execute_until(&mut timeline, &mut module, &mut log, half_seconds(i + 1));
        }

        let output = module.filt_data_out_msg.read();
        assert_eq!(output.num_obs, 3);
        assert_matches_golden("time tag", &[output.time_tag_s], &[30.000000000000004]);
        assert_matches_golden(
            "state",
            output.state.as_slice(),
            &[
                -0.70001145156739408,
                0.6999828805822077,
                -9.880162382335158e-14,
                -6.5096515857761428e-06,
                -6.5098567483075182e-06,
                -7.842378839306008e-15,
            ],
        );
        let covar: Vec<f64> = output.covar.transpose().iter().copied().collect();
        assert_matches_golden(
            "covar (row-major)",
            &covar,
            &[
                0.00058977149585070967,
                0.0002598549068751086,
                -1.0970291032604218e-18,
                3.9188937605347628e-05,
                3.918502466367415e-05,
                -3.623724198622137e-19,
                0.0002598549068751086,
                0.00099542902888345741,
                -1.6805091093599073e-18,
                5.5943103958876734e-05,
                5.5966868012503731e-05,
                -5.2676476514976094e-19,
                -1.0970291032604218e-18,
                -1.6805091093599073e-18,
                0.00063909729844818981,
                -7.3175808882892057e-18,
                6.618251171731321e-18,
                6.0710157509919617e-05,
                3.9188937605347628e-05,
                5.5943103958876734e-05,
                -7.3175808882892057e-18,
                2.6202487720441708e-05,
                2.5180468382267321e-05,
                -3.5783381552167282e-18,
                3.918502466367415e-05,
                5.5966868012503731e-05,
                6.618251171731321e-18,
                2.5180468382267321e-05,
                2.6205377803552871e-05,
                3.1994984269130345e-18,
                -3.623724198622137e-19,
                -5.2676476514976094e-19,
                6.0710157509919617e-05,
                -3.5783381552167282e-18,
                3.1994984269130345e-18,
                4.7894104624560459e-05,
            ],
        );
        assert_matches_golden(
            "post-fit residuals",
            &output.post_fit_res.as_slice()[..3],
            &[
                1.8978957432302934e-05,
                5.8159735639162768e-05,
                1.8978957347037806e-05,
            ],
        );
        assert_matches_golden(
            "sun heading",
            module
                .nav_state_out_msg
                .read()
                .vehicle_sun_point_body
                .as_slice(),
            &[
                -0.70712121162441022,
                0.70709235045418639,
                -9.9805115747740434e-14,
            ],
        );
    }
}
