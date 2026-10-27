//! Coarse-sun-sensor sunline estimator with a switching frame: a square-root
//! unscented Kalman filter for the sun heading, the observable body rates and
//! the sun intensity. Port of upstream Basilisk `sunlineSuKF`
//! (`fswAlgorithms/attDetermination/sunlineSuKF`).
//!
//! The six states are the unit sun heading `d̂` (body components), the two
//! components of `ω_SB` normal to it in the S frame, and the sun intensity
//! that scales the CSS readings. The S frame has `s1 = d̂`, `s2 = d × b̂`
//! and `s3 = d × s2` (both normalized) for a body axis `b̂`; when `|b̂ · d̂|`
//! exceeds 0.866 the filter switches `b̂` between the body x and y axes and
//! rotates the rate states and the covariance into the new frame.
//!
//! A time update rotates every sigma point's heading with
//! `d += -dt (ω_BN × d)`, `ω_BN = [BS] (0, -ω_2, -ω_3)`, re-normalizes it, and
//! adds `q_noise` once per update. Each CSS is predicted as the cosine
//! `CBias n̂ · d̂` clipped at zero, shaped by its Kelly fit
//! (`1 - exp(-x^p / f)` and a relative scale) when the fit factor is positive,
//! times the intensity and clipped again. A sensor is used when its reading
//! or its prediction from the state exceeds `sensor_use_thresh`.
//!
//! Every update, as upstream: seed the state from the strongest sensor if the
//! filter is not initialized (publishing nothing while no reading reaches the
//! threshold), switch frames if needed, propagate to and apply a CSS message
//! that is not older than the estimate, re-normalize the heading, propagate to
//! the call time if that is later, and publish. A time or measurement update
//! whose factorization fails rolls the estimate back to its value before that
//! update. The published rates are `ω_BN = -ω_SB`.
//!
//! Upstream behaviour kept as is, noted where it happens: a frame switch
//! rotates `covar` but not its square root `sBar`, from which the next time
//! update builds its sigma points; a reset re-initializes the covariance but
//! keeps the current state and the initialized flag; post-fit residuals past
//! this update's observation count keep their previous values.
//!
//! # Message timing
//!
//! Upstream tells a new measurement from the CSS message header (`isWritten`,
//! `timeWritten`). Rust ports carry no header, so a connected
//! `css_data_in_msg` counts as written at the current update — what upstream
//! sees when the CSS producer runs before the filter on every tick. The
//! header-driven logic itself is unchanged, with the header as an explicit
//! argument, and the tests drive it with upstream's exact header values.

use nalgebra::{DMatrix, DVector, Matrix3, Matrix6, SVector, Vector3, Vector6};

use crate::fsw_algorithms::att_determination::sunline_ukf::square_root_ukf::{
    NUM_SIGMA_POINTS, SquareRootUkf, configured_css_units, css_written_nanos, heading,
};
use crate::messages::{
    CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, Input, MAX_N_CSS_MEAS, MAX_NUM_CSS_SENSORS,
    NavigationAttitudeMsg, Output, SunlineFilterMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::{is_equal_within, normalize_or_zero};
use crate::{Module, SimulationContext};

/// Upstream `switchTresh`, which every Reset sets: `|b̂ · d̂|` above which
/// the S frame switches body axis.
const SWITCH_THRESHOLD: f64 = 0.866;

/// Norm of `d × b̂` below which upstream takes the S frame as the body frame.
const DEGENERATE_FRAME_NORM: f64 = 1.0e-5;

/// Kelly-curve fit of one CSS's response (upstream `SunlineSuKFCFit`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CssKellyFit {
    /// Upstream `cssRelScale` \[-\]: relative scale factor of the sensor.
    pub relative_scale: f64,
    /// Upstream `cssKellPow` \[-\]: exponent of the Kelly curve.
    pub kelly_power: f64,
    /// Upstream `cssKellFact` \[-\]: Kelly factor; the fit applies only when it
    /// is positive.
    pub kelly_factor: f64,
}

/// User-set parameters of [`SunlineSukf`] (the upstream `SunlineSuKFConfig`
/// fields set from Python). The defaults are upstream's zero-initialized
/// values, which do not make a working filter.
#[derive(Clone, Debug, Default)]
pub struct SunlineSukfConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `alpha` \[-\]: sigma-point spread.
    pub alpha: f64,
    /// Upstream `beta` \[-\]: prior-distribution parameter (2 for a Gaussian).
    pub beta: f64,
    /// Upstream `kappa` \[-\]: secondary spread parameter.
    pub kappa: f64,
    /// Upstream `state`: the estimate the filter starts from when
    /// `filter_initialized` is set — heading \[-\], S-frame rates \[rad/s\],
    /// intensity \[-\]. Taken by [`SunlineSukf::new`] and again by `init`
    /// (upstream initialization reads whatever the user set); a reset keeps
    /// the current estimate.
    ///
    /// Upstream also exposes `stateInit`, but only as scratch for the
    /// strongest-sensor guess: a value set by the user is never read (Reset
    /// does not copy it into the state), so it is not a parameter here.
    pub state: Vector6<f64>,
    /// Upstream `covarInit` \[-\]: covariance set by every reset.
    pub covar_init: Matrix6<f64>,
    /// Upstream `qNoise` \[-\]: process noise covariance, added once per time
    /// update.
    pub q_noise: Matrix6<f64>,
    /// Upstream `qObsVal` \[-\]: variance of each CSS measurement.
    pub q_obs_val: f64,
    /// Upstream `sensorUseThresh` \[-\]: a sensor is used when its reading or
    /// its prediction exceeds this.
    pub sensor_use_thresh: f64,
    /// Upstream `kellFits`: Kelly fit per CSS, in configuration order.
    pub kell_fits: [CssKellyFit; MAX_NUM_CSS_SENSORS],
    /// Upstream `filterInitialized`, taken like `state`: when false, the first
    /// update with a reading at or above `sensor_use_thresh` seeds the state
    /// from the strongest sensor.
    pub filter_initialized: bool,
}

/// The `sunlineSuKF` module.
#[derive(Clone, Debug)]
pub struct SunlineSukf {
    /// User-set parameters.
    pub config: SunlineSukfConfig,
    /// Upstream `cssDataInMsg`: CSS readings.
    pub css_data_in_msg: Input<CssArraySensorMsg>,
    /// Upstream `cssConfigInMsg`: CSS constellation, read at reset.
    pub css_config_in_msg: Input<CssConfigMsg>,
    /// Upstream `navStateOutMsg`: unit sun heading and its time tag.
    pub nav_state_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `filtDataOutMsg`: state (with `ω_BN` rates), covariance and
    /// post-fit residuals.
    pub filt_data_out_msg: Output<SunlineFilterMsg>,
    filter: SquareRootUkf,
    /// Upstream `statePrev`, `sBarPrev`, `covarPrev`.
    previous: SavedEstimate,
    /// Upstream `filterInitialized`; a reset leaves it alone.
    filter_initialized: bool,
    /// Upstream `timeTag` \[s\]: time of the estimate.
    time_tag_s: f64,
    /// Upstream `bVec_B`: the body axis that defines the S frame.
    b_vec_b: Vector3<f64>,
    /// Upstream `cssNHat_B` and `CBias` of the `numCSSTotal` sensors.
    css_units: Vec<CssUnitConfigMsg>,
    /// Upstream `cssSensorInBuffer`: the readings of the last update.
    css_readings: CssArraySensorMsg,
    /// Upstream `obs`: the readings the last measurement model used, in its
    /// leading `num_obs` entries; the others keep older values.
    observations: SVector<f64, MAX_N_CSS_MEAS>,
    /// Upstream `yMeas`: predicted reading per used sensor (row) and sigma
    /// point (column).
    predictions: DMatrix<f64>,
    /// Upstream `numObs`: sensors used by the last measurement model
    /// (`MAX_N_CSS_MEAS` after a reset).
    num_obs: usize,
    /// Upstream `postFits`: only the leading `num_obs` entries are refreshed.
    post_fits: SVector<f64, MAX_N_CSS_MEAS>,
}

/// The estimate before the current time or measurement update (upstream
/// `statePrev`, `sBarPrev`, `covarPrev`).
#[derive(Clone, Copy, Debug, Default)]
struct SavedEstimate {
    state: Vector6<f64>,
    s_bar: Matrix6<f64>,
    covar: Matrix6<f64>,
}

/// Why a time or measurement update was rejected (upstream returns `-1`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RejectedUpdate {
    /// A non-central covariance weight is negative; the update stopped before
    /// touching the estimate.
    NegativeWeight,
    /// A factorization failed; the estimate was restored to its value before
    /// the update.
    BadFactorization,
}

impl SunlineSukf {
    /// A filter at the configured starting estimate, with unconnected ports.
    pub fn new(config: SunlineSukfConfig) -> Self {
        Self {
            filter: SquareRootUkf::new(config.state, Matrix6::zeros()),
            filter_initialized: config.filter_initialized,
            config,
            css_data_in_msg: Input::default(),
            css_config_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            previous: SavedEstimate::default(),
            time_tag_s: 0.0,
            b_vec_b: Vector3::zeros(),
            css_units: Vec::new(),
            css_readings: CssArraySensorMsg::default(),
            observations: SVector::zeros(),
            predictions: DMatrix::zeros(0, NUM_SIGMA_POINTS),
            num_obs: 0,
            post_fits: SVector::zeros(),
        }
    }

    /// Upstream `Reset_sunlineSuKF`: reads the CSS configuration, resets the
    /// covariance to `covar_init` and rebuilds the filter matrices. The state,
    /// the initialized flag and the post-fit residuals are kept.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.css_config_in_msg.is_connected() {
            log::error!("sunlineSuKF.css_config_in_msg is not connected");
        }
        if !self.css_data_in_msg.is_connected() {
            log::error!("sunlineSuKF.css_data_in_msg is not connected");
        }
        self.css_units = configured_css_units(&self.css_config_in_msg.read(), "sunlineSuKF");
        self.time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.num_obs = MAX_N_CSS_MEAS;
        self.b_vec_b = Vector3::x();
        self.observations = SVector::zeros();
        self.filter.covar = self.config.covar_init;
        self.previous = SavedEstimate::default();
        let bad_update = self.filter.reset(
            self.config.alpha,
            self.config.beta,
            self.config.kappa,
            &self.config.q_noise,
        );
        // Upstream reads the readings only if the message was written; an
        // unwritten one reads as zeros either way.
        self.css_readings = self.css_data_in_msg.read();
        if bad_update {
            log::warn!("sunlineSuKF: Reset method contained bad update");
        }
    }

    /// Upstream `Update_sunlineSuKF`, with the CSS message header explicit:
    /// `written_nanos` is upstream `timeWritten` when the message `isWritten`.
    fn step(
        &mut self,
        call_time_nanos: u64,
        readings: CssArraySensorMsg,
        written_nanos: Option<u64>,
    ) {
        self.css_readings = readings;
        if !self.filter_initialized {
            let Some(state) = self.strongest_sensor_state() else {
                return;
            };
            self.filter.state = state;
            self.filter_initialized = true;
        }

        let unit_heading = normalize_or_zero(heading(&self.filter.state));
        if self.b_vec_b.dot(&unit_heading).abs() > SWITCH_THRESHOLD {
            // Upstream rotates `covar` but not `sBar`, which the next time
            // update spreads its sigma points with.
            switch_frame(
                &mut self.b_vec_b,
                &mut self.filter.state,
                &mut self.filter.covar,
            );
        }

        let measurement_time_s = written_nanos.map(|nanos| nanos as f64 * NANO2SEC);
        if let Some(measurement_time_s) = measurement_time_s
            && measurement_time_s >= self.time_tag_s
        {
            // Upstream ignores both statuses: a rejected update has already
            // kept or restored the estimate.
            let _ = self.time_update(measurement_time_s);
            let _ = self.measurement_update();
        }
        let unit_heading = normalize_or_zero(heading(&self.filter.state));
        self.filter
            .state
            .fixed_rows_mut::<3>(0)
            .copy_from(&unit_heading);
        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.time_tag_s {
            let _ = self.time_update(call_time_s);
        }

        self.measurement_model();
        let y_bar = self.filter.predicted_mean(&self.predictions);
        for (index, mean) in y_bar.iter().enumerate() {
            self.post_fits[index] = self.observations[index] - mean;
        }

        self.nav_state_out_msg.write(NavigationAttitudeMsg {
            time_tag_s: self.time_tag_s,
            vehicle_sun_point_body: normalize_or_zero(heading(&self.filter.state)),
            ..Default::default()
        });
        // The filter estimates ω_SB = -ω_BN; the message carries ω_BN.
        let mut state = self.filter.state;
        state.fixed_rows_mut::<2>(3).neg_mut();
        self.filt_data_out_msg.write(SunlineFilterMsg {
            time_tag_s: self.time_tag_s,
            covar: self.filter.covar,
            state,
            post_fit_res: self.post_fits,
            num_obs: self.num_obs,
            ..Default::default()
        });
    }

    /// Upstream's automatic initialization: the normal of the CSS with the
    /// largest reading as heading, zero rates and that reading as intensity;
    /// `None` while the largest reading is below `sensor_use_thresh`.
    fn strongest_sensor_state(&self) -> Option<Vector6<f64>> {
        let mut state = Vector6::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        let mut max_reading = 0.0;
        for (unit, reading) in self.css_units.iter().zip(self.css_readings.cos_values) {
            if reading > max_reading {
                state.fixed_rows_mut::<3>(0).copy_from(&unit.n_hat_b);
                state[5] = reading;
                max_reading = reading;
            }
        }
        if max_reading < self.config.sensor_use_thresh {
            None
        } else {
            Some(state)
        }
    }

    /// Upstream `sunlineSuKFTimeUpdate`: propagates the estimate to
    /// `update_time_s`.
    fn time_update(&mut self, update_time_s: f64) -> Result<(), RejectedUpdate> {
        self.save_previous_estimate();
        let dt_s = update_time_s - self.time_tag_s;
        let b_vec_b = self.b_vec_b;
        self.filter
            .propagate_sigma_points(|point| propagate(point, &b_vec_b, dt_s));
        if self.filter.has_negative_spread_weight() {
            return Err(RejectedUpdate::NegativeWeight);
        }
        let bad_update = self.filter.update_time_covariance();
        self.time_tag_s = update_time_s;
        self.restore_if(bad_update)
    }

    /// Upstream `sunlineSuKFMeasUpdate`: applies the current CSS readings.
    fn measurement_update(&mut self) -> Result<(), RejectedUpdate> {
        self.save_previous_estimate();
        self.measurement_model();
        if self.filter.has_negative_spread_weight() {
            return Err(RejectedUpdate::NegativeWeight);
        }
        let observations =
            DVector::from_column_slice(&self.observations.as_slice()[..self.num_obs]);
        let bad_update =
            self.filter
                .measurement_update(&self.predictions, &observations, self.config.q_obs_val);
        self.restore_if(bad_update)
    }

    fn save_previous_estimate(&mut self) {
        self.previous = SavedEstimate {
            state: self.filter.state,
            s_bar: self.filter.s_bar,
            covar: self.filter.covar,
        };
    }

    fn restore_if(&mut self, bad_update: bool) -> Result<(), RejectedUpdate> {
        if bad_update {
            self.restore_previous_estimate();
            Err(RejectedUpdate::BadFactorization)
        } else {
            Ok(())
        }
    }

    /// Upstream `sunlineSuKFCleanUpdate`: zeroes the observations of this
    /// update, restores the saved estimate and recomputes the standard
    /// weights.
    fn restore_previous_estimate(&mut self) {
        self.observations.rows_mut(0, self.num_obs).fill(0.0);
        self.filter.state = self.previous.state;
        self.filter.s_bar = self.previous.s_bar;
        self.filter.covar = self.previous.covar;
        self.filter.set_weights(self.config.alpha, self.config.beta);
    }

    /// Upstream `sunlineSuKFMeasModel`: selects the sensors whose reading or
    /// predicted reading exceeds the threshold and predicts them at every
    /// sigma point of the last time update.
    fn measurement_model(&mut self) {
        let threshold = self.config.sensor_use_thresh;
        let unit_heading = normalize_or_zero(heading(&self.filter.state));
        let intensity = self.filter.state[5];
        let used: Vec<(f64, Vector3<f64>, CssKellyFit)> = self
            .css_units
            .iter()
            .zip(self.css_readings.cos_values)
            .zip(self.config.kell_fits)
            .map(|((unit, reading), fit)| (reading, unit.n_hat_b * unit.c_bias, fit))
            .filter(|(reading, normal, fit)| {
                *reading > threshold
                    || expected_reading(&unit_heading, normal, fit, intensity) > threshold
            })
            .collect();
        for (observation, (reading, _, _)) in self.observations.iter_mut().zip(&used) {
            *observation = *reading;
        }
        let sigma_points = &self.filter.sigma_points;
        self.predictions = DMatrix::from_fn(used.len(), NUM_SIGMA_POINTS, |row, column| {
            let point = sigma_points.column(column);
            let (_, normal, fit) = &used[row];
            expected_reading(&normalize_or_zero(heading(&point)), normal, fit, point[5])
        });
        self.num_obs = used.len();
    }
}

/// Upstream's CSS model for a unit heading: the cosine with the scaled
/// normal clipped at zero, shaped by the Kelly fit when its factor is
/// positive, times the sun intensity and clipped again.
fn expected_reading(
    unit_heading: &Vector3<f64>,
    normal: &Vector3<f64>,
    fit: &CssKellyFit,
    intensity: f64,
) -> f64 {
    let mut expected = clip_negative(unit_heading.dot(normal));
    if fit.kelly_factor > 0.0 {
        expected *= 1.0 - (-expected.powf(fit.kelly_power) / fit.kelly_factor).exp();
        expected *= fit.relative_scale;
    }
    clip_negative(expected * intensity)
}

/// Upstream's `x > 0.0 ? x : 0.0`, which also maps NaN to zero.
fn clip_negative(x: f64) -> f64 {
    if x > 0.0 { x } else { 0.0 }
}

/// Upstream `sunlineStateProp` (sunlineSuKF): rotates the heading by the body
/// rate the S-frame rate states imply and re-normalizes it.
fn propagate(point: &mut Vector6<f64>, b_vec_b: &Vector3<f64>, dt_s: f64) {
    let sun_heading = heading(point);
    let omega_bn_s = Vector3::new(0.0, -point[3], -point[4]);
    let omega_bn_b = dcm_bs(&sun_heading, b_vec_b) * omega_bn_s;
    let propagated = normalize_or_zero(sun_heading + omega_bn_b.cross(&sun_heading) * -dt_s);
    point.fixed_rows_mut::<3>(0).copy_from(&propagated);
}

/// Upstream `sunlineSuKFComputeDCM_BS`: `[BS]`, whose columns are the S axes
/// in body components; the identity when `d × b̂` nearly vanishes.
fn dcm_bs(sun_heading: &Vector3<f64>, b_vec_b: &Vector3<f64>) -> Matrix3<f64> {
    let s2 = sun_heading.cross(b_vec_b);
    if s2.norm() < DEGENERATE_FRAME_NORM {
        return Matrix3::identity();
    }
    let s2 = normalize_or_zero(s2);
    let s3 = normalize_or_zero(sun_heading.cross(&s2));
    Matrix3::from_columns(&[normalize_or_zero(*sun_heading), s2, s3])
}

/// Upstream `sunlineSuKFSwitch`: moves the S frame to the other body axis
/// (y if it is on x, x otherwise) and rotates the rate states and the
/// covariance into it.
fn switch_frame(b_vec_b: &mut Vector3<f64>, state: &mut Vector6<f64>, covar: &mut Matrix6<f64>) {
    let unit_heading = normalize_or_zero(heading(state));
    let dcm_bs_old = dcm_bs(&unit_heading, b_vec_b);
    let new_b_vec = if is_equal_within(b_vec_b, &Vector3::x(), 1.0e-10) {
        Vector3::y()
    } else {
        Vector3::x()
    };
    let dcm_new_old = dcm_bs(&unit_heading, &new_b_vec).transpose() * dcm_bs_old;
    *b_vec_b = new_b_vec;

    let mut switch = Matrix6::identity();
    switch
        .fixed_view_mut::<2, 2>(3, 3)
        .copy_from(&dcm_new_old.fixed_view::<2, 2>(1, 1));
    *state = switch * *state;
    *covar = switch * *covar * switch.transpose();
}

impl Module for SunlineSukf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationAttitudeMsg::default());
        self.filt_data_out_msg.write(SunlineFilterMsg::default());
        // Upstream initialization starts from the `state` and
        // `filterInitialized` the user set, also after construction.
        self.filter.state = self.config.state;
        self.filter_initialized = self.config.filter_initialized;
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        SunlineSukf::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let readings = self.css_data_in_msg.read();
        let written_nanos = css_written_nanos(&self.css_data_in_msg, context);
        self.step(context.current_sim_nanos, readings, written_nanos);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `_UnitTest/test_SunLineSuKF.py`. Its
    //! `test_all_sunline_kf` runs `SwitchMethods`, `StatePropSunLine`,
    //! `StateUpdateSunLine(kellyOn)` and `FaultScenarios` for `kellyOn` in
    //! {False, True}; only the state update depends on the parameter, so each
    //! part is its own test and the state update runs both values. Truth and
    //! tolerances are upstream's (the numpy truth of `SwitchMethods` is
    //! recomputed). The plots (`SunLineSuKF_test_utilities.py`) are not
    //! ported.
    //!
    //! The pytests feed the filter through the CSS message header: readings
    //! are stamped with the start of a task step and read at its end.
    //! [`UpstreamTimeline`] replays that timeline through the private `step`,
    //! so the filter sees the header values upstream sees.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};

    use super::{
        CssKellyFit, RejectedUpdate, SavedEstimate, SunlineSukf, SunlineSukfConfig, dcm_bs,
        switch_frame,
    };
    use crate::fsw_algorithms::att_determination::sunline_ukf::square_root_ukf::SigmaWeights;
    use crate::fsw_algorithms::att_determination::sunline_ukf::square_root_ukf::test_support::{
        TASK_RATE_NANOS, UpstreamTimeline,
    };
    use crate::messages::{
        CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, Output, SunlineFilterMsg,
    };
    use crate::utilities::linear_algebra::safe_acos;
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

    /// Upstream `setupFilterData(filterObject, initialized)`. Its
    /// `stateInit = [0, 0, 1, 0, 0, 1]` for an initialized filter has no
    /// effect upstream (see [`SunlineSukfConfig::state`]), so the state
    /// stays zero.
    fn upstream_config(initialized: bool) -> SunlineSukfConfig {
        let rate_noise = 0.001 * 0.001;
        let intensity_noise = 0.0000002 * 0.0000002;
        SunlineSukfConfig {
            name: "sunlineSuKF".to_string(),
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state: Vector6::zeros(),
            covar_init: Matrix6::from_diagonal(&Vector6::new(1.0, 1.0, 1.0, 0.02, 0.02, 1.0e-4)),
            q_noise: Matrix6::from_diagonal(&Vector6::new(
                rate_noise,
                rate_noise,
                rate_noise,
                rate_noise,
                rate_noise,
                intensity_noise,
            )),
            q_obs_val: 0.002,
            sensor_use_thresh: 0.0,
            kell_fits: Default::default(),
            filter_initialized: initialized,
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

    /// Upstream `dotList`: `reading(normal)` for every normal.
    fn css_readings(reading: impl Fn(&Vector3<f64>) -> f64) -> CssArraySensorMsg {
        let readings: Vec<f64> = CSS_NORMALS
            .iter()
            .map(|normal| reading(&Vector3::from(*normal)))
            .collect();
        CssArraySensorMsg::from_active(&readings)
    }

    /// The module subscribed to `css_config` and `css_data`, then initialized
    /// (`InitializeSimulation`).
    fn subscribed_filter(
        config: SunlineSukfConfig,
        css_config: CssConfigMsg,
        css_data: &Output<CssArraySensorMsg>,
    ) -> SunlineSukf {
        let mut module = SunlineSukf::new(config);
        Output::new(css_config).connect_to(&mut module.css_config_in_msg);
        css_data.connect_to(&mut module.css_data_in_msg);
        module.init();
        module
    }

    /// `ExecuteSimulation` up to `stop_nanos` with `filtDataOutMsg` recorded
    /// every tick (`dataLog`).
    fn execute_until(
        timeline: &mut UpstreamTimeline,
        module: &mut SunlineSukf,
        log: &mut Vec<SunlineFilterMsg>,
        stop_nanos: u64,
    ) {
        timeline.execute_until(stop_nanos, |call_nanos, readings, written_nanos| {
            module.step(call_nanos, readings, written_nanos);
            log.push(module.filt_data_out_msg.read());
        });
    }

    /// numpy's `DCM_BS`: the unit heading and the normalized cross products
    /// with `b_vec` and with the second axis, as columns.
    fn numpy_dcm_bs(sun_heading: &Vector3<f64>, b_vec: &Vector3<f64>) -> Matrix3<f64> {
        let s1 = sun_heading / sun_heading.norm();
        let s2 = s1.cross(b_vec) / s1.cross(b_vec).norm();
        let s3 = s1.cross(&s2) / s1.cross(&s2).norm();
        Matrix3::from_columns(&[s1, s2, s3])
    }

    #[test]
    fn matches_upstream_switch_methods() {
        let sun_heading = Vector3::new(2.0, 1.0, 0.75);
        let b1 = Vector3::new(0.0, 1.0, 0.0);
        let error = (dcm_bs(&sun_heading, &b1) - numpy_dcm_bs(&sun_heading, &b1)).norm();
        assert!(
            error <= 1.0e-10,
            "Frame switch failure: DCM error {error:e}"
        );

        let input_states = Vector6::new(2.0, 1.0, 0.75, 0.1, 0.4, 1.0);
        let covar = Matrix6::from_row_slice(&[
            1.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, 0.0, 1.0, //
            1.0, 0.0, 0.0, 1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, 0.0, 1.0, 0.0, //
            0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
        ]);
        let b2 = Vector3::new(1.0, 0.0, 0.0);
        let dcm_new_old =
            numpy_dcm_bs(&sun_heading, &b2).transpose() * numpy_dcm_bs(&sun_heading, &b1);
        let mut switch = Matrix6::identity();
        switch
            .fixed_view_mut::<2, 2>(3, 3)
            .copy_from(&dcm_new_old.fixed_view::<2, 2>(1, 1));

        let (mut b_vec, mut states, mut covar_out) = (b1, input_states, covar);
        switch_frame(&mut b_vec, &mut states, &mut covar_out);
        let errors = [
            (states - switch * input_states).norm(),
            (switch * covar * switch.transpose() - covar_out).norm(),
            (b_vec - b2).norm(),
        ];
        for error in errors {
            assert!(error <= 1.0e-10, "Frame switch failure: error {error:e}");
        }
    }

    #[test]
    fn matches_upstream_state_prop_sun_line() {
        // Neither input message is ever written.
        let mut module = subscribed_filter(
            upstream_config(true),
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
    fn matches_upstream_state_update_sun_line() {
        for kelly_on in [false, true] {
            state_update_sun_line(kelly_on);
        }
    }

    /// Upstream `StateUpdateSunLine(show_plots, kellyOn)`.
    fn state_update_sun_line(kelly_on: bool) {
        let mut config = upstream_config(false);
        if kelly_on {
            config.kell_fits[..CSS_NORMALS.len()].fill(CssKellyFit {
                relative_scale: 1.0,
                kelly_power: 2.0,
                kelly_factor: 0.05,
            });
        }
        // A 1 % error is allowed with the Kelly curves the readings lack.
        let (steps, accuracy) = if kelly_on {
            (1000, 1.0e-2)
        } else {
            (500, 1.0e-3)
        };
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());
        let mut timeline = UpstreamTimeline::new();
        let mut log = Vec::new();

        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let sun_vector: Vector3<f64> = sun_vector / sun_vector.norm();
        let readings =
            css_readings(|normal| normal.dot(&sun_vector) / (normal.norm() * sun_vector.norm()));
        for i in 0..steps {
            timeline.write(readings.clone());
            execute_until(&mut timeline, &mut module, &mut log, half_seconds(i + 1));
        }
        assert_state_update(&log, &sun_vector, accuracy, kelly_on);

        let sun_vector = Vector3::new(-0.7, 0.75, 0.0);
        let sun_vector: Vector3<f64> = sun_vector / sun_vector.norm();
        let readings = css_readings(|normal| normal.dot(&sun_vector));
        for i in 0..steps {
            if i > 20 {
                timeline.write(readings.clone());
            }
            execute_until(
                &mut timeline,
                &mut module,
                &mut log,
                half_seconds(i + steps + 1),
            );
        }
        assert_state_update(&log, &sun_vector, accuracy, kelly_on);
    }

    /// Upstream's checks: no variance above its first logged value, the
    /// heading within `accuracy` \[rad\] of the sun vector, the rates zero and
    /// the intensity one within `accuracy`.
    fn assert_state_update(
        log: &[SunlineFilterMsg],
        sun_vector: &Vector3<f64>,
        accuracy: f64,
        kelly_on: bool,
    ) {
        let (first, last) = (&log[0], &log[log.len() - 1]);
        for i in 0..6 {
            assert!(
                last.covar[(i, i)] <= first.covar[(i, i)],
                "Covariance update failure (kelly {kelly_on}): variance {i} grew to {:e}",
                last.covar[(i, i)]
            );
        }
        let sun_heading = last.state.fixed_rows::<3>(0);
        let angle =
            safe_acos(sun_heading.dot(sun_vector) / (sun_heading.norm() * sun_vector.norm()));
        assert!(
            angle <= accuracy,
            "Pointing update failure (kelly {kelly_on}): {angle:e} rad"
        );
        let rate_error = (last.state.fixed_rows::<3>(3) - Vector3::new(0.0, 0.0, 1.0)).norm();
        assert!(
            rate_error <= accuracy,
            "Rate update failure (kelly {kelly_on}): {rate_error:e}"
        );
        let intensity_error = (last.state[5] - 1.0).abs();
        assert!(
            intensity_error <= accuracy,
            "Sun Intensity update failure (kelly {kelly_on}): {intensity_error:e}"
        );
    }

    #[test]
    fn matches_upstream_fault_scenarios() {
        let mut module = SunlineSukf::new(SunlineSukfConfig::default());
        module.filter.state = Vector6::zeros();
        module.filter.s_bar = Matrix6::zeros();
        module.filter.covar = Matrix6::zeros();
        module.previous = SavedEstimate {
            state: Vector6::zeros(),
            s_bar: Matrix6::identity(),
            covar: Matrix6::identity() * 2.0,
        };
        module.restore_previous_estimate();
        // Upstream compares within 1e10 (a typo for 1e-10, so it cannot fail);
        // the restore is exact.
        assert_eq!(
            module.filter.covar, module.previous.covar,
            "Clean Covar failed"
        );
        assert_eq!(
            module.filter.state, module.previous.state,
            "Clean States failed"
        );
        assert_eq!(
            module.filter.s_bar, module.previous.s_bar,
            "Clean sBar failed"
        );

        module.config.alpha = 0.02;
        module.config.beta = 2.0;
        module.config.kappa = 0.0;
        module.filter.w_c = SigmaWeights::repeat(-1.0);
        module.filter.w_m = SigmaWeights::repeat(-1.0);
        assert_eq!(
            module.time_update(1.0),
            Err(RejectedUpdate::NegativeWeight),
            "Failed to catch bad Update and clean in Time update"
        );
        assert_eq!(
            module.measurement_update(),
            Err(RejectedUpdate::NegativeWeight),
            "Failed to catch bad Update and clean in Meas update"
        );
    }

    #[test]
    fn a_failed_factorization_restores_the_previous_estimate() {
        let mut module = subscribed_filter(
            upstream_config(true),
            css_constellation(),
            &Output::default(),
        );
        module.filter.state = Vector6::new(0.6, 0.8, 0.0, 0.05, -0.02, 1.0);
        module.num_obs = 3;
        module.observations.fill(0.5);
        let (state, s_bar, covar) = (
            module.filter.state,
            module.filter.s_bar,
            module.filter.covar,
        );
        let standard_w_c = module.filter.w_c;

        // A huge negative zeroth weight makes the down-date indefinite.
        module.filter.w_c[0] = -1.0e12;
        assert_eq!(
            module.time_update(10.0),
            Err(RejectedUpdate::BadFactorization)
        );
        assert_eq!(module.filter.state, state);
        assert_eq!(module.filter.s_bar, s_bar);
        assert_eq!(module.filter.covar, covar);
        assert_eq!(module.filter.w_c, standard_w_c);
        // Upstream keeps the new time tag and zeroes this update's observations.
        assert_eq!(module.time_tag_s, 10.0);
        assert!(module.observations.rows(0, 3).iter().all(|&obs| obs == 0.0));
        assert_eq!(module.observations[3], 0.5);
    }

    #[test]
    fn the_first_update_seeds_the_state_from_the_strongest_sensor() {
        let mut module = subscribed_filter(
            upstream_config(false),
            css_constellation(),
            &Output::default(),
        );
        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let readings = css_readings(|normal| normal.dot(&sun_vector));
        let strongest = readings.cos_values[5];
        module.step(0, readings, None);

        let output = module.filt_data_out_msg.read();
        let expected_heading = Vector3::from(CSS_NORMALS[5]);
        assert!((output.state.fixed_rows::<3>(0) - expected_heading).norm() <= 1.0e-15);
        assert_eq!(output.state.fixed_rows::<2>(3), nalgebra::Vector2::zeros());
        assert_eq!(output.state[5], strongest);
    }

    #[test]
    fn nothing_is_published_until_a_reading_reaches_the_threshold() {
        let mut config = upstream_config(false);
        config.sensor_use_thresh = 0.5;
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());

        module.step(0, CssArraySensorMsg::from_active(&[0.2; 8]), Some(0));
        assert_eq!(module.filt_data_out_msg.read(), SunlineFilterMsg::default());

        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let readings = css_readings(|normal| normal.dot(&sun_vector));
        module.step(TASK_RATE_NANOS, readings, Some(TASK_RATE_NANOS));
        let output = module.filt_data_out_msg.read();
        assert_eq!(output.time_tag_s, 0.5);
        assert!(output.num_obs > 0);
    }

    #[test]
    fn the_frame_switches_when_the_heading_nears_its_axis() {
        let mut config = upstream_config(true);
        config.state = Vector6::new(0.95, 0.3122498999199199, 0.0, 0.0, 0.0, 1.0);
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());
        assert_eq!(module.b_vec_b, Vector3::x());
        module.step(TASK_RATE_NANOS, CssArraySensorMsg::default(), None);
        assert_eq!(module.b_vec_b, Vector3::y());
    }

    #[test]
    fn reset_restores_the_covariance_but_keeps_the_state() {
        let mut module = subscribed_filter(
            upstream_config(false),
            css_constellation(),
            &Output::default(),
        );
        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let readings = css_readings(|normal| normal.dot(&sun_vector));
        for tick in 0..10 {
            let now = tick * TASK_RATE_NANOS;
            module.step(now, readings.clone(), Some(now));
        }
        let state = module.filter.state;
        module.reset(10 * TASK_RATE_NANOS);
        assert_eq!(module.filter.covar, module.config.covar_init);
        assert_eq!(module.filter.state, state);
        assert!(module.filter_initialized);
        assert_eq!(module.time_tag_s, 5.0);
    }

    #[test]
    fn update_reads_a_connected_css_message_as_written_at_the_call_time() {
        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let readings = css_readings(|normal| normal.dot(&sun_vector));
        let css_data = Output::new(readings.clone());
        let mut through_update =
            subscribed_filter(upstream_config(false), css_constellation(), &css_data);
        let mut through_step = subscribed_filter(
            upstream_config(false),
            css_constellation(),
            &Output::default(),
        );
        let mut unconnected = SunlineSukf::new(upstream_config(true));
        let mut unconnected_step = SunlineSukf::new(upstream_config(true));
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
    }

    #[test]
    fn configuring_before_init_equals_configuring_at_construction() {
        let mut config = upstream_config(true);
        config.state = Vector6::new(0.6, 0.8, 0.0, 0.01, -0.02, 1.0);
        let mut configured_later = SunlineSukf::new(SunlineSukfConfig::default());
        configured_later.config = config.clone();
        Output::new(css_constellation()).connect_to(&mut configured_later.css_config_in_msg);
        Output::<CssArraySensorMsg>::default().connect_to(&mut configured_later.css_data_in_msg);
        configured_later.init();
        let mut configured_at_new =
            subscribed_filter(config, css_constellation(), &Output::default());

        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let readings = css_readings(|normal| normal.dot(&sun_vector));
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
    /// `StateUpdateSunLine(kellyOn = True)` loop, from upstream
    /// `sunlineSuKF.c` compiled with gcc `-O2 -ffp-contract=off` and driven
    /// through the same message timeline. The port reproduces it bit for bit;
    /// the tolerance only allows for another libm (`exp`, `pow`).
    #[test]
    #[allow(clippy::excessive_precision)]
    fn matches_compiled_upstream_on_the_kelly_state_update_timeline() {
        let mut config = upstream_config(false);
        config.kell_fits[..CSS_NORMALS.len()].fill(CssKellyFit {
            relative_scale: 1.0,
            kelly_power: 2.0,
            kelly_factor: 0.05,
        });
        let mut module = subscribed_filter(config, css_constellation(), &Output::default());
        let mut timeline = UpstreamTimeline::new();
        let mut log = Vec::new();
        let sun_vector = Vector3::new(-0.7, 0.7, 0.0);
        let sun_vector: Vector3<f64> = sun_vector / sun_vector.norm();
        let readings =
            css_readings(|normal| normal.dot(&sun_vector) / (normal.norm() * sun_vector.norm()));
        for i in 0..60 {
            timeline.write(readings.clone());
            execute_until(&mut timeline, &mut module, &mut log, half_seconds(i + 1));
        }

        let output = module.filt_data_out_msg.read();
        assert_eq!(output.num_obs, 4);
        assert_eq!(module.b_vec_b, Vector3::x());
        assert_matches_golden("time tag", &[output.time_tag_s], &[30.000000000000004]);
        assert_matches_golden(
            "state",
            output.state.as_slice(),
            &[
                -0.71158289936981023,
                0.70260214725294889,
                -7.4048417039225785e-14,
                1.7322009344516064e-05,
                -5.4651193867383466e-15,
                1.000894420610267,
            ],
        );
        let covar: Vec<f64> = output.covar.transpose().iter().copied().collect();
        assert_matches_golden(
            "covar (row-major)",
            &covar,
            &[
                0.00031121518804056038,
                0.00031370930364604777,
                -1.2712640667958396e-16,
                6.4596794494471327e-05,
                4.2263472047178722e-17,
                1.6231176949127935e-05,
                0.00031370930364604777,
                0.00031923923869032339,
                -8.5449941408943614e-17,
                6.542247872926593e-05,
                3.5719717616097713e-17,
                1.643864584778711e-05,
                -1.2712640667958396e-16,
                -8.5449941408943614e-17,
                0.00036505201936904672,
                -1.4209895550663573e-17,
                -6.5569646123128425e-05,
                -2.1056694633796319e-16,
                6.4596794494471327e-05,
                6.542247872926593e-05,
                -1.4209895550663573e-17,
                2.7437809894122722e-05,
                5.914581951885451e-18,
                -1.5792173607395262e-08,
                4.2263472047178722e-17,
                3.5719717616097713e-17,
                -6.5569646123128425e-05,
                5.914581951885451e-18,
                2.3271369292356868e-05,
                9.4844095748132935e-17,
                1.6231176949127935e-05,
                1.643864584778711e-05,
                -2.1056694633796319e-16,
                -1.5792173607395262e-08,
                9.4844095748132935e-17,
                2.4658730045816063e-05,
            ],
        );
        assert_matches_golden(
            "post-fit residuals",
            &output.post_fit_res.as_slice()[..4],
            &[
                -1.5804917268269492e-05,
                -0.00037776749974227641,
                -1.5804917239847782e-05,
                -0.00024426971413326628,
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
                -0.71158289936981023,
                0.70260214725294889,
                -7.4048417039225785e-14,
            ],
        );
    }
}
