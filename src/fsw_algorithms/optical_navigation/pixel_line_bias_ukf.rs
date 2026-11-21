//! Relative orbit determination from planet circles, estimating the image
//! processing bias: port of upstream Basilisk `pixelLineBiasUKF`
//! (`fswAlgorithms/opticalNavigation`).
//!
//! Like [`relative_od_ukf`](super::relative_od_ukf), a square-root unscented
//! Kalman filter estimates the spacecraft position and velocity relative to the
//! planet being navigated (two-body dynamics, one RK4 step per sigma point), but
//! it reads the circle found in the image directly ([`OpNavCirclesMsg`]) and
//! folds the pixel/line conversion into its measurement model. Three more
//! states hold the bias \[pixels\] of the measured circle centre (x, y) and
//! radius; they are not propagated and are rounded to whole pixels after each
//! measurement update. The six measurements are the biased centre and radius
//! and the rounded bias between the measured circle and the one predicted by
//! the central sigma point.
//!
//! A measurement is processed when the attitude input is (re)written and the
//! circles message is valid; the camera configuration and attitude are read at
//! every update. The filter works in kilometres internally; its configuration,
//! inputs and outputs are in metres (bias states in pixels), except
//! [`PixelLineBiasUkfConfig::q_noise`].
//!
//! # Deviations from upstream
//!
//! * The crate's message ports carry no write header, so upstream's
//!   `attInMsg.isWritten()` / `timeWritten()` are emulated: the attitude input
//!   counts as written when it is connected, at the first update in which its
//!   payload differs from the previous one. This equals upstream when the
//!   attitude producer runs before the filter in the tick it writes.
//! * Upstream uses an uninitialized gravitational parameter or planet radius
//!   when the circle's planet ID is not 1, 2 or 3 (undefined behaviour); the
//!   port uses NaN and logs an error for every unknown ID, not only for
//!   upstream's ID 0.
//! * Upstream's clean-up after a rejected update recomputes the weights from
//!   the `alpha` and `beta` currently in the configuration (with the `lambda`
//!   of the last reset). The port keeps the tuning of the last reset, so
//!   changing `alpha`, `beta` or `kappa` takes effect at the next reset only.
//! * Upstream's unused `maxTimeJump` configuration field is not ported. Its
//!   only observable effect is through the `postFits` overflow below, where
//!   the port shows its zero default.
//!
//! # Upstream bugs and quirks kept
//!
//! * `postFits` is declared with three entries but receives all six residuals.
//!   The overflow lands in `timeTagOut`, `maxTimeJump` and the circles buffer's
//!   time tag, so after the first measurement the navigation output's time tag
//!   is the fourth residual, and the last three entries of the filter output's
//!   `post_fit_res` keep the residuals of the latest measurement (they start at
//!   the reset time, `maxTimeJump`'s default 0 and 0).
//! * Reset converts only the first 60 row-major entries of `covarInit` to
//!   kilometres (`PIXLINE_DYN_STATES * PIXLINE_N_STATES + PIXLINE_DYN_STATES`),
//!   which covers the dynamic block and row 6's position/velocity terms but not
//!   rows 7 and 8; the output converts the same 60 entries back.
//! * The predicted circle centre uses the integer half resolution
//!   (`resolution / 2` in `int` arithmetic).
//! * Measurements are not propagated to the current time when it equals the
//!   measurement time (`>` instead of `relativeODuKF`'s `>=`), so the post-fit
//!   residuals then come from the pre-update sigma points.
//! * As in `relativeODuKF`, the measurement noise enters the QR step as the rows
//!   of its lower Cholesky factor, and the filter output's `num_obs` and
//!   `state_error` stay zero.

use nalgebra::{Matrix3, SMatrix, SVector, Vector2, Vector3, Vector6};

use super::relative_od_ukf::square_root_ukf::{
    NavigationPlanet, SquareRootUkf, UkfTuning, WriteStamp, norm3, planet_mu_km3ps2, two_body_rk4,
};
use crate::messages::{
    CameraConfigMsg, Input, NavigationAttitudeMsg, NavigationTranslationMsg, OpNavCirclesMsg,
    Output, PIXLINE_DYN_STATES, PIXLINE_N_MEAS, PIXLINE_N_STATES, PixelLineFilterMsg,
};
use crate::time::NANO2SEC;
use crate::utilities::linear_algebra::safe_asin;
use crate::utilities::linear_algebra::sequential_product;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Number of sigma points, `2 PIXLINE_N_STATES + 1`.
const NUM_SIGMA_POINTS: usize = 2 * PIXLINE_N_STATES + 1;

/// Row-major entries of the covariance that upstream converts between metres
/// and kilometres.
const CONVERTED_COVARIANCE_ENTRIES: usize =
    PIXLINE_DYN_STATES * PIXLINE_N_STATES + PIXLINE_DYN_STATES;

type PixelLineFilter = SquareRootUkf<PIXLINE_N_STATES, PIXLINE_N_MEAS, NUM_SIGMA_POINTS>;
type SigmaPoints = SMatrix<f64, PIXLINE_N_STATES, NUM_SIGMA_POINTS>;
type Covariance = SMatrix<f64, PIXLINE_N_STATES, PIXLINE_N_STATES>;
type Measurements = SVector<f64, PIXLINE_N_MEAS>;
type PredictedMeasurements = SMatrix<f64, PIXLINE_N_MEAS, NUM_SIGMA_POINTS>;

/// Fingerprint of an attitude message, for the emulated write time.
type AttitudeFingerprint = (f64, Vector3<f64>, Vector3<f64>, Vector3<f64>);

/// User configuration of [`PixelLineBiasUkf`]. [`Default`] is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct PixelLineBiasUkfConfig {
    /// Module name.
    pub name: String,
    /// Upstream `planetIdInit` [-]: planet navigated about until a measurement
    /// names one (Earth = 1, Mars = 2, Jupiter = 3).
    pub planet_id_init: i32,
    /// Upstream `alpha` [-]: sigma-point spread parameter.
    pub alpha: f64,
    /// Upstream `beta` [-]: prior-distribution parameter (2 for Gaussian).
    pub beta: f64,
    /// Upstream `kappa` [-]: secondary spread parameter.
    pub kappa: f64,
    /// Upstream `stateInit` \[m, m/s, pixels\]: initial planet-relative
    /// position and velocity (inertial components) and the circle centre
    /// (x, y) and radius biases.
    pub state_init: SVector<f64, PIXLINE_N_STATES>,
    /// Upstream `covarInit` \[m^2, m^2/s, m^2/s^2, pixels^2\]: initial state
    /// covariance (see the module doc for which entries are converted).
    pub covar_init: Covariance,
    /// Upstream `qNoise` \[km^2, km^2/s, km^2/s^2, pixels^2\]: process noise
    /// covariance added at every time update. Upstream applies no conversion
    /// to it.
    pub q_noise: Covariance,
}

impl Default for PixelLineBiasUkfConfig {
    fn default() -> Self {
        Self {
            name: "pixelLineBiasUKF".to_string(),
            planet_id_init: 0,
            alpha: 0.0,
            beta: 0.0,
            kappa: 0.0,
            state_init: SVector::zeros(),
            covar_init: SMatrix::zeros(),
            q_noise: SMatrix::zeros(),
        }
    }
}

/// Square-root UKF estimating the planet-relative translational state and the
/// circle-fit bias from planet circles (upstream `pixelLineBiasUKF`).
#[derive(Clone, Debug)]
pub struct PixelLineBiasUkf {
    /// User configuration, applied at reset.
    pub config: PixelLineBiasUkfConfig,
    /// Upstream `circlesInMsg`: circles found in the image (required).
    pub circles_in_msg: Input<OpNavCirclesMsg>,
    /// Upstream `cameraConfigInMsg`: camera field of view, resolution and
    /// mounting (required).
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// Upstream `attInMsg`: spacecraft attitude; its writes trigger the
    /// measurement updates (required).
    pub att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `navStateOutMsg`: estimated position and velocity.
    pub nav_state_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `filtDataOutMsg`: filter state, covariance and residuals.
    pub filt_data_out_msg: Output<PixelLineFilterMsg>,
    filter: PixelLineFilter,
    /// Upstream `planetId`: planet of the last processed measurement.
    planet_id: i32,
    /// Upstream `circlesInBuffer`: last processed circles.
    circles_in_buffer: OpNavCirclesMsg,
    /// Upstream `cameraSpecs`: camera configuration read at the last update.
    camera_specs: CameraConfigMsg,
    /// Upstream `attInfo`: attitude read at the last update.
    att_info: NavigationAttitudeMsg,
    /// Upstream `postFits` \[pixels\] together with the three fields its
    /// overflow writes: entry 3 is upstream `timeTagOut` \[s\], entry 4
    /// `maxTimeJump` and entry 5 the circles buffer's time tag.
    post_fits: Measurements,
    att_written: WriteStamp<AttitudeFingerprint>,
}

impl PixelLineBiasUkf {
    /// A filter with unconnected ports; [`Module::init`] resets it.
    pub fn new(config: PixelLineBiasUkfConfig) -> Self {
        Self {
            config,
            circles_in_msg: Input::default(),
            camera_config_in_msg: Input::default(),
            att_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            filter: PixelLineFilter::default(),
            planet_id: 0,
            circles_in_buffer: OpNavCirclesMsg::default(),
            camera_specs: CameraConfigMsg::default(),
            att_info: NavigationAttitudeMsg::default(),
            post_fits: Measurements::zeros(),
            att_written: WriteStamp::default(),
        }
    }

    /// Upstream `Reset_pixelLineBiasUKF`: re-initializes the estimate from the
    /// configuration (converted to kilometres) at `call_time_nanos`.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.circles_in_msg.is_connected() {
            log::error!("pixelLineBiasUKF.circles_in_msg is not connected");
        }
        if !self.camera_config_in_msg.is_connected() {
            log::error!("pixelLineBiasUKF.camera_config_in_msg is not connected");
        }
        if !self.att_in_msg.is_connected() {
            log::error!("pixelLineBiasUKF.att_in_msg is not connected");
        }

        let time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.planet_id = self.config.planet_id_init;
        let tuning = UkfTuning {
            alpha: self.config.alpha,
            beta: self.config.beta,
            kappa: self.config.kappa,
        };
        let mut state = self.config.state_init;
        state
            .fixed_rows_mut::<PIXLINE_DYN_STATES>(0)
            .scale_mut(1.0e-3);
        let reset = self.filter.reset(
            time_tag_s,
            tuning,
            state,
            scale_converted_entries(&self.config.covar_init, 1.0e-6),
            &self.config.q_noise,
        );
        // Upstream `timeTagOut`, which shares its storage with `postFits[3]`.
        self.post_fits[3] = time_tag_s;
        if reset.is_err() {
            log::warn!("Reset method contained bad update");
        }
    }

    /// Upstream `pixelLineBiasUKFTimeUpdate`: propagates the position and
    /// velocity; the bias states are constant.
    fn time_update(&mut self, update_time_s: f64) {
        if self.planet_id == 0 {
            log::error!("Need a planet to navigate");
        } else if NavigationPlanet::from_id(self.planet_id).is_none() {
            log::error!(
                "pixelLineBiasUKF: unknown planet ID {}; propagating with NaN gravity",
                self.planet_id
            );
        }
        let mu_km3ps2 = planet_mu_km3ps2(self.planet_id);
        // Upstream ignores the status: a rejected update has already restored
        // the previous estimate.
        let _ = self.filter.time_update(update_time_s, |state, dt_s| {
            propagate_dynamic_states(state, mu_km3ps2, dt_s)
        });
    }

    /// Upstream `pixelLineBiasUKFMeasUpdate`, with the buffered circles.
    fn measurement_update(&mut self) {
        let uncertainty = self.circles_in_buffer.uncertainty;
        let mut noise = SMatrix::<f64, PIXLINE_N_MEAS, PIXLINE_N_MEAS>::zeros();
        noise.fixed_view_mut::<3, 3>(3, 3).copy_from(&uncertainty);
        noise.fixed_view_mut::<3, 3>(0, 0).copy_from(&uncertainty);
        let (circles, camera, attitude) =
            (&self.circles_in_buffer, &self.camera_specs, &self.att_info);
        let _ = self.filter.measurement_update(
            |sigma_points| measurement_model(circles, camera, attitude, sigma_points),
            &noise,
            1.0,
            |state| {
                state
                    .fixed_rows_mut::<3>(PIXLINE_DYN_STATES)
                    .apply(|bias| *bias = bias.round())
            },
        );
    }

    /// Post-fit residuals: the observation minus the mean predicted by the
    /// current sigma points.
    fn post_fit_residuals(&self) -> Measurements {
        let (observation, predicted) = measurement_model(
            &self.circles_in_buffer,
            &self.camera_specs,
            &self.att_info,
            &self.filter.sigma_points,
        );
        observation - self.filter.weighted_mean(&predicted)
    }

    fn nav_state(&self) -> NavigationTranslationMsg {
        let state = &self.filter.state;
        NavigationTranslationMsg {
            time_tag_s: self.post_fits[3],
            position_bn_n_m: state.fixed_rows::<3>(0) * 1.0e3,
            velocity_bn_n_mps: state.fixed_rows::<3>(3) * 1.0e3,
            ..Default::default()
        }
    }

    fn filter_data(&self) -> PixelLineFilterMsg {
        let mut state = self.filter.state;
        state
            .fixed_rows_mut::<PIXLINE_DYN_STATES>(0)
            .scale_mut(1.0e3);
        PixelLineFilterMsg {
            time_tag_s: self.filter.time_tag_s,
            covar: scale_converted_entries(&self.filter.covar, 1.0e6),
            state,
            post_fit_res: self.post_fits,
            ..Default::default()
        }
    }
}

impl Module for PixelLineBiasUkf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationTranslationMsg::default());
        self.filt_data_out_msg.write(PixelLineFilterMsg::default());
        PixelLineBiasUkf::reset(self, 0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        PixelLineBiasUkf::reset(self, context.current_sim_nanos);
    }

    /// Upstream `Update_pixelLineBiasUKF`.
    fn update(&mut self, context: &SimulationContext) {
        let call_time_nanos = context.current_sim_nanos;
        // Only the three declared entries are cleared; see `post_fits`.
        self.post_fits.fixed_rows_mut::<3>(0).fill(0.0);

        let circles = self.circles_in_msg.read();
        self.camera_specs = self.camera_config_in_msg.read();
        self.att_info = self.att_in_msg.read();
        let written_nanos = self.att_in_msg.is_connected().then(|| {
            self.att_written
                .observe(attitude_fingerprint(&self.att_info), call_time_nanos)
        });

        let mut processed_measurement = false;
        if let Some(written_nanos) = written_nanos {
            let measurement_time_s = written_nanos as f64 * NANO2SEC;
            if measurement_time_s >= self.filter.time_tag_s && circles.valid {
                // C's `(int)` conversion truncates toward zero.
                self.planet_id = circles.planet_ids[0] as i32;
                self.circles_in_buffer = circles;
                self.time_update(measurement_time_s);
                self.measurement_update();
                processed_measurement = true;
            }
        }

        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s > self.filter.time_tag_s {
            self.time_update(call_time_s);
        }

        if processed_measurement {
            self.post_fits = self.post_fit_residuals();
        }

        self.nav_state_out_msg.write(self.nav_state());
        self.filt_data_out_msg.write(self.filter_data());
    }
}

/// Upstream `relODStateProp` on the leading position/velocity states.
fn propagate_dynamic_states(state: &mut SVector<f64, PIXLINE_N_STATES>, mu_km3ps2: f64, dt_s: f64) {
    let mut dynamic: Vector6<f64> = state.fixed_rows::<PIXLINE_DYN_STATES>(0).into_owned();
    two_body_rk4(&mut dynamic, mu_km3ps2, dt_s);
    state
        .fixed_rows_mut::<PIXLINE_DYN_STATES>(0)
        .copy_from(&dynamic);
}

/// Upstream `pixelLineBiasUKFMeasModel`.
///
/// The observation is the measured circle (centre x, y and radius \[pixels\])
/// followed by the rounded difference between the circle predicted by the
/// central sigma point and the measured one. Sigma point `j` predicts its own
/// circle minus its bias states, followed by the bias states themselves.
fn measurement_model(
    circles: &OpNavCirclesMsg,
    camera: &CameraConfigMsg,
    attitude: &NavigationAttitudeMsg,
    sigma_points: &SigmaPoints,
) -> (Measurements, PredictedMeasurements) {
    let measured_center = circles.circles_centers_px[0];
    let measured_radius = circles.circles_radii_px[0];
    let dcm_cn = sequential_product(&mrp_to_dcm(camera.sigma_cb), &mrp_to_dcm(attitude.sigma_bn));
    let pixel_size = PixelSize::of(camera);
    let planet_radius_km = planet_radius_km(circles.planet_ids[0]);

    let mut observation = Measurements::zeros();
    observation[0] = measured_center.x;
    observation[1] = measured_center.y;
    observation[2] = measured_radius;
    let mut predicted = PredictedMeasurements::zeros();
    for (j, point) in sigma_points.column_iter().enumerate() {
        let position_km = point.fixed_rows::<3>(0).into_owned();
        let (center, radius) =
            pixel_line_circle(&position_km, &dcm_cn, &pixel_size, camera, planet_radius_km);
        if j == 0 {
            observation[3] = (center.x - observation[0]).round();
            observation[4] = (center.y - observation[1]).round();
            observation[5] = (radius - observation[2]).round();
        }
        let bias = point.fixed_rows::<3>(PIXLINE_DYN_STATES);
        let mut column = predicted.column_mut(j);
        column[0] = center.x - bias[0];
        column[1] = center.y - bias[1];
        column[2] = radius - bias[2];
        column.fixed_rows_mut::<3>(3).copy_from(&bias);
    }
    (observation, predicted)
}

/// Size of one pixel on the image plane at unit focal length: upstream `X` and
/// `Y`, the sensor size over the focal length divided by the resolution.
struct PixelSize {
    x: f64,
    y: f64,
}

impl PixelSize {
    fn of(camera: &CameraConfigMsg) -> Self {
        let [width, height] = camera.resolution_px.map(f64::from);
        let fov = camera.field_of_view_rad;
        let sensor_x = 2.0 * (fov * width / height / 2.0).tan();
        let sensor_y = 2.0 * (fov / 2.0).tan();
        Self {
            x: sensor_x / width,
            y: sensor_y / height,
        }
    }
}

/// Circle \[pixels\] the planet would make in the image seen from
/// `position_km`: centre (x, y) and radius.
fn pixel_line_circle(
    position_km: &Vector3<f64>,
    dcm_cn: &Matrix3<f64>,
    pixel_size: &PixelSize,
    camera: &CameraConfigMsg,
    planet_radius_km: f64,
) -> (Vector2<f64>, f64) {
    let range_km = norm3(position_km);
    let r_c = sequential_product(dcm_cn, position_km);
    let r_c = r_c * (-1.0 / r_c[2]);
    // Upstream adds the half resolution in `int` arithmetic.
    let [half_width, half_height] = camera.resolution_px.map(|pixels| f64::from(pixels / 2));
    let center = Vector2::new(
        r_c[0] / pixel_size.x + half_width - 0.5,
        r_c[1] / pixel_size.y + half_height - 0.5,
    );
    let radius = safe_asin(planet_radius_km / range_km).tan() / pixel_size.x;
    (center, radius)
}

/// Equatorial radius \[km\] of the circle's planet (upstream `planetIds[0]`,
/// compared as a `double`); NaN where upstream reads an uninitialized radius.
fn planet_radius_km(planet_id: f64) -> f64 {
    let id = planet_id as i32;
    NavigationPlanet::from_id(id)
        .filter(|_| f64::from(id) == planet_id)
        .map_or(f64::NAN, NavigationPlanet::equatorial_radius_km)
}

/// Upstream converts only the leading [`CONVERTED_COVARIANCE_ENTRIES`]
/// row-major entries of the covariance between metres and kilometres.
fn scale_converted_entries(covariance: &Covariance, factor: f64) -> Covariance {
    Covariance::from_fn(|row, col| {
        let entry = covariance[(row, col)];
        if row * PIXLINE_N_STATES + col < CONVERTED_COVARIANCE_ENTRIES {
            entry * factor
        } else {
            entry
        }
    })
}

fn attitude_fingerprint(attitude: &NavigationAttitudeMsg) -> AttitudeFingerprint {
    (
        attitude.time_tag_s,
        attitude.sigma_bn,
        attitude.omega_bn_b_radps,
        attitude.vehicle_sun_point_body,
    )
}

#[cfg(test)]
mod tests {
    //! Ports `_UnitTest/test_pixelLineBiasUKF.py`: `test_methods_kf`
    //! (`relOD_method_test`: dynamics and measurement model) and
    //! `test_propagation_kf` (`StatePropRelOD` at `dt = 10 s`), with the same
    //! truth (re-implemented numpy code, the RK4 in
    //! `square_root_ukf::test_support`) and tolerances. The plots of
    //! `pixelLineBias_test_utilities.py` are not ported.
    //!
    //! Upstream never runs a measurement update. The remaining tests exercise
    //! it on circles synthesized from a simulated orbit, and pin down the kept
    //! upstream quirks. (The measurement update was also compared bit for bit
    //! with the compiled upstream C during the port.)

    use nalgebra::{Matrix3, SMatrix, SVector, Vector2, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, Normal};

    use super::super::relative_od_ukf::square_root_ukf::planet_mu_km3ps2;
    use super::super::relative_od_ukf::square_root_ukf::test_support::{
        context, initial_orbit, orbital_energy, rk4, time_grid,
    };
    use super::{
        CONVERTED_COVARIANCE_ENTRIES, Covariance, PixelLineBiasUkf, PixelLineBiasUkfConfig,
        PredictedMeasurements, SigmaPoints, measurement_model, propagate_dynamic_states,
    };
    use crate::Module;
    use crate::messages::{
        CameraConfigMsg, NavigationAttitudeMsg, OpNavCirclesMsg, Output, PIXLINE_DYN_STATES,
        PIXLINE_N_STATES, PixelLineFilterMsg,
    };
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm};

    const NANOS_PER_SECOND: u64 = 1_000_000_000;
    /// Upstream test `planetRad`: Mars equatorial radius \[km\].
    const PLANET_RADIUS_KM: f64 = 3396.19;
    /// Bias between the true and the measured circle (centre x, y, radius)
    /// \[pixels\]; `setupFilterData` starts the bias states at it.
    const TRUE_BIAS_PX: [f64; 3] = [1.0, 1.0, -2.0];

    type State = SVector<f64, PIXLINE_N_STATES>;

    fn nanos(seconds: u64) -> u64 {
        seconds * NANOS_PER_SECOND
    }

    /// Upstream `setupFilterData`.
    fn setup_filter_data() -> PixelLineBiasUkfConfig {
        let (r, v) = initial_orbit(8000.0 * 1.0e3);
        let [bx, by, br] = TRUE_BIAS_PX;
        let mut covar_diagonal = State::repeat(1000.0 * 1.0e6);
        covar_diagonal
            .fixed_rows_mut::<3>(3)
            .copy_from(&Vector3::repeat(5.0 * 1.0e6));
        covar_diagonal
            .fixed_rows_mut::<3>(6)
            .copy_from(&Vector3::new(2.0, 2.0, 3.0));
        // numpy scales the blocks in sequence; the bias block is the scaled
        // velocity block scaled again.
        let position_noise = 0.00001 * 0.00001 * 1.0e-6;
        let velocity_noise = 0.0001 * 0.0001 * 1.0e-6;
        let bias_noise = velocity_noise * 0.0001 * 0.0001;
        let mut q_diagonal = State::repeat(position_noise);
        q_diagonal
            .fixed_rows_mut::<3>(3)
            .copy_from(&Vector3::repeat(velocity_noise));
        q_diagonal
            .fixed_rows_mut::<3>(6)
            .copy_from(&Vector3::repeat(bias_noise));
        PixelLineBiasUkfConfig {
            planet_id_init: 2,
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state_init: State::from_column_slice(&[r.x, r.y, r.z, v.x, v.y, v.z, bx, by, br]),
            covar_init: SMatrix::from_diagonal(&covar_diagonal),
            q_noise: SMatrix::from_diagonal(&q_diagonal),
            ..Default::default()
        }
    }

    /// `2 arctan(s/2 / f)` with a 10 mm sensor and focal length `focal_m`.
    fn camera(focal_m: f64, resolution_px: [i32; 2], sigma_cb: Vector3<f64>) -> CameraConfigMsg {
        CameraConfigMsg {
            field_of_view_rad: 2.0 * (10.0 * 1e-3 / 2.0 / focal_m).atan(),
            resolution_px,
            sigma_cb,
            ..Default::default()
        }
    }

    /// The filter's three inputs, each a fresh message.
    struct Inputs {
        circles: Output<OpNavCirclesMsg>,
        camera: Output<CameraConfigMsg>,
        attitude: Output<NavigationAttitudeMsg>,
    }

    fn connected_filter(
        config: PixelLineBiasUkfConfig,
        camera: CameraConfigMsg,
        attitude: NavigationAttitudeMsg,
    ) -> (PixelLineBiasUkf, Inputs) {
        let mut module = PixelLineBiasUkf::new(config);
        let inputs = Inputs {
            circles: Output::default(),
            camera: Output::new(camera),
            attitude: Output::new(attitude),
        };
        inputs.circles.connect_to(&mut module.circles_in_msg);
        inputs.camera.connect_to(&mut module.camera_config_in_msg);
        inputs.attitude.connect_to(&mut module.att_in_msg);
        module.init();
        (module, inputs)
    }

    /// The sigma points of `relOD_method_test`: the state, then the state plus
    /// and minus each column of `10 I`.
    fn method_test_sigma_points(state: &State) -> SigmaPoints {
        SigmaPoints::from_fn(|row, col| {
            let spread = |index: usize| if row == index { 10.0 } else { 0.0 };
            match col {
                0 => state[row],
                1..=PIXLINE_N_STATES => state[row] + spread(col - 1),
                _ => state[row] - spread(col - 1 - PIXLINE_N_STATES),
            }
        })
    }

    #[test]
    fn matches_upstream_methods_test() {
        let state =
            State::from_column_slice(&[250.0, 32000.0, 1000.0, 5.0, 3.0, 2.0, 1.0, 1.0, 1.0]);

        // Dynamics model: one RK4 step about Mars; the biases are constant.
        let dt_s = 10.0;
        let mut propagated = state;
        propagate_dynamic_states(&mut propagated, planet_mu_km3ps2(2), dt_s);
        let truth = rk4(&[0.0, dt_s], state * 1.0e3)[1] * 1.0e-3;
        let relative_error = (propagated - truth).component_div(&truth);
        assert!(relative_error.norm() <= 1.0e-15, "{relative_error}");

        // Measurement model. Upstream checks only the predicted measurements
        // (its `obs[3:5]` truth subtracts the measured centre from itself and
        // is never asserted); the observation is checked here against the
        // C algorithm.
        let mut circles = OpNavCirclesMsg::default();
        circles.circles_centers_px[0] = Vector2::new(100.0, 200.0);
        circles.circles_radii_px[0] = 100.0;
        circles.planet_ids[0] = 2.0;
        let attitude = NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.0, 0.2, -0.1),
            ..Default::default()
        };
        let camera = camera(1.0 * 1e-3, [512, 512], Vector3::new(-0.2, 0.0, 0.3));
        let sigma_points = method_test_sigma_points(&state);
        let (observation, predicted) =
            measurement_model(&circles, &camera, &attitude, &sigma_points);

        let dcm_cn = mrp_to_dcm(camera.sigma_cb) * mrp_to_dcm(attitude.sigma_bn);
        let [width, height] = camera.resolution_px.map(f64::from);
        let fov = camera.field_of_view_rad;
        let x = 2.0 * (fov * width / height / 2.0).tan() / width;
        let y = 2.0 * (fov / 2.0).tan() / height;
        let mut expected = PredictedMeasurements::zeros();
        let mut expected_observation = [100.0, 200.0, 100.0, 0.0, 0.0, 0.0];
        for (i, point) in sigma_points.column_iter().enumerate() {
            let position = point.fixed_rows::<3>(0);
            let r_c = dcm_cn * position;
            let r_c = -1.0 / r_c[2] * r_c;
            let center_x = r_c[0] / x + (width / 2.0 - 0.5);
            let center_y = r_c[1] / y + (height / 2.0 - 0.5);
            let radius = 1.0 / x * (PLANET_RADIUS_KM / position.norm()).asin().tan();
            if i == 0 {
                expected_observation[3] = (center_x - 100.0).round();
                expected_observation[4] = (center_y - 200.0).round();
                expected_observation[5] = (radius - 100.0).round();
            }
            expected[(0, i)] = center_x - point[6];
            expected[(1, i)] = center_y - point[7];
            expected[(2, i)] = radius - point[8];
            expected
                .fixed_view_mut::<3, 1>(3, i)
                .copy_from(&point.fixed_rows::<3>(6));
        }
        let relative_error = (predicted - expected).norm() / expected.column(0).norm();
        assert!(relative_error <= 1.0e-15, "{relative_error:e}");
        assert_eq!(observation.as_slice(), expected_observation.as_slice());
    }

    #[test]
    fn matches_upstream_state_propagation_test() {
        let dt_s = 10;
        let config = setup_filter_data();
        let attitude = NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.6, 1.0, 0.1),
            ..Default::default()
        };
        let camera = camera(0.01, [512, 512], Vector3::new(1.0, 0.3, 0.1));
        let (mut module, _inputs) = connected_filter(config.clone(), camera, attitude);

        let time_sim_s = 60 * 60;
        let steps = time_sim_s / dt_s;
        let state_log: Vec<State> = (0..=steps)
            .map(|step| {
                module.update(&context(nanos(step * dt_s)));
                module.filt_data_out_msg.read().state
            })
            .collect();

        let time = time_grid(time_sim_s as f64, steps as usize + 1);
        let expected = rk4(&time, config.state_init);
        let energy: Vec<f64> = expected.iter().map(orbital_energy).collect();

        let last = expected.len() - 1;
        let relative_error = (state_log[last] - expected[last]).component_div(&expected[last]);
        assert!(relative_error.norm() <= 1.0e-10, "{relative_error}");
        assert!((energy[0] - energy[last]) / energy[0] <= 1.0e-10);
    }

    /// Asserts `actual` equals the upstream C values to 1e-12, relative for
    /// magnitudes above one and absolute below.
    fn assert_matches_c(actual: &[f64], expected: &[f64], what: &str) {
        assert_eq!(actual.len(), expected.len(), "{what}");
        for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
            let tolerance = 1.0e-12 * e.abs().max(1.0);
            assert!((a - e).abs() <= tolerance, "{what}[{i}]: {a} vs {e}");
        }
    }

    /// The covariance diagonal followed by the entries (1, 7) and (7, 1).
    fn covariance_samples(filter: &PixelLineFilterMsg) -> Vec<f64> {
        let mut samples: Vec<f64> = filter.covar.diagonal().iter().copied().collect();
        samples.extend([filter.covar[(1, 7)], filter.covar[(7, 1)]]);
        samples
    }

    #[test]
    fn matches_compiled_upstream_on_a_biased_circle() {
        // Truth from upstream pixelLineBiasUKF.c (gcc -O2 -ffp-contract=off)
        // on the same inputs, for the measurement update that the Python
        // tests never run. The measured circle is off by more than half a
        // pixel from the configured bias, so the update moves the rounded
        // bias states. The values also pin the odd-resolution integer half,
        // the unconverted covariance entry (7, 1), the missing propagation
        // after a measurement at the current time and the postFits overflow.
        let mut covar_init = Covariance::from_diagonal(&State::from_column_slice(&[
            1.0e9, 1.0e9, 1.0e9, 5.0e6, 5.0e6, 5.0e6, 2.0, 2.0, 3.0,
        ]));
        covar_init[(1, 7)] = 3.0;
        covar_init[(7, 1)] = 3.0;
        let config = PixelLineBiasUkfConfig {
            planet_id_init: 2,
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state_init: State::from_column_slice(&[
                6.0e6, 4.0e6, 1.0e6, -1200.0, 1500.0, 300.0, 1.0, 1.0, -2.0,
            ]),
            covar_init,
            q_noise: Covariance::from_diagonal(&State::from_column_slice(&[
                1.0e-16, 1.0e-16, 1.0e-16, 1.0e-14, 1.0e-14, 1.0e-14, 1.0e-22, 1.0e-22, 1.0e-22,
            ])),
            ..Default::default()
        };
        let camera = CameraConfigMsg {
            field_of_view_rad: 0.9272952180016122,
            resolution_px: [513, 511],
            sigma_cb: Vector3::new(0.1, -0.05, 0.2),
            ..Default::default()
        };
        let (mut module, inputs) =
            connected_filter(config, camera, NavigationAttitudeMsg::default());
        module.update(&context(0));

        let attitude = |time_s: f64| NavigationAttitudeMsg {
            time_tag_s: time_s,
            sigma_bn: Vector3::new(0.435759, 0.0779093, -0.337936),
            ..Default::default()
        };
        let mut circles = OpNavCirclesMsg {
            time_tag_nanos: nanos(1),
            valid: true,
            uncertainty: Matrix3::new(0.5, 0.1, 0.0, 0.1, 0.4, 0.05, 0.0, 0.05, 0.6),
            ..Default::default()
        };
        circles.planet_ids[0] = 2.0;
        circles.circles_centers_px[0] = Vector2::new(263.52, 254.18);
        circles.circles_radii_px[0] = 273.3;
        inputs.attitude.write(attitude(1.0));
        inputs.circles.write(circles.clone());
        module.update(&context(nanos(1)));
        let output = module.filt_data_out_msg.read();
        assert_matches_c(
            output.state.as_slice(),
            &[
                5994111.439821724,
                4001715.831023846,
                997669.922962874,
                -1223.991056026039,
                1500.5609504978506,
                286.804400834719,
                2.0,
                1.0,
                -4.0,
            ],
            "state at 1 s",
        );
        let residuals_1s = [
            -0.9988630803459273,
            -0.19779108490212138,
            1.9542783153927417,
            0.9999999999990337,
            -7.389644451905042e-13,
            -1.9999999999973852,
        ];
        assert_matches_c(
            output.post_fit_res.as_slice(),
            &residuals_1s,
            "post-fit residuals at 1 s",
        );
        assert_matches_c(
            &covariance_samples(&output),
            &[
                273899936.75377303,
                217989127.65671125,
                117134642.90105486,
                4981903.775535965,
                4980587.560183427,
                4978024.081790596,
                0.3809041292178955,
                0.3003801959012524,
                0.42040896229880376,
                -1518346.6954417985,
                -1.5183466954417986,
            ],
            "covariance at 1 s",
        );
        assert_eq!(output.time_tag_s, 1.0);
        assert_eq!(
            module.nav_state_out_msg.read().time_tag_s,
            output.post_fit_res[3]
        );

        // The rewritten attitude processes the still-valid circles again.
        inputs.attitude.write(attitude(2.0));
        module.update(&context(nanos(2)));
        let kept = module.filt_data_out_msg.read().post_fit_res;

        inputs.circles.write(OpNavCirclesMsg {
            valid: false,
            ..circles
        });
        module.update(&context(nanos(3)));
        let output = module.filt_data_out_msg.read();
        assert_matches_c(
            output.state.as_slice(),
            &[
                5994708.003939479,
                4004194.205018549,
                998025.2154708883,
                -1106.951967680942,
                1443.119567368706,
                254.7745213550711,
                2.0,
                1.0,
                -4.0,
            ],
            "state at 3 s",
        );
        assert_eq!(output.post_fit_res.fixed_rows::<3>(0), Vector3::zeros());
        assert_eq!(
            output.post_fit_res.fixed_rows::<3>(3),
            kept.fixed_rows::<3>(3)
        );
        assert_matches_c(
            &covariance_samples(&output),
            &[
                181145669.6155315,
                143851807.221233,
                76581214.54239869,
                4885419.356210747,
                4863349.221845484,
                4780002.22748276,
                0.21933161733655782,
                0.16857877731048623,
                0.24367870101342828,
                -1035115.5238446712,
                -1.0351155238446712,
            ],
            "covariance at 3 s",
        );
    }

    /// Attitude that points the camera boresight (`+z_C`) at the planet from
    /// `state` \[m, m/s\], slightly off centre.
    fn pointing_attitude(
        state: &State,
        camera: &CameraConfigMsg,
        time_s: f64,
    ) -> NavigationAttitudeMsg {
        let position = state.fixed_rows::<3>(0);
        let velocity = state.fixed_rows::<3>(3);
        let boresight = (-position.normalize()
            + Vector3::new(
                0.03 * (0.01 * time_s).sin(),
                0.02 * (0.013 * time_s).cos(),
                0.0,
            ))
        .normalize();
        let first = position.cross(&velocity).cross(&boresight).normalize();
        let second = boresight.cross(&first);
        let dcm_cn =
            Matrix3::from_rows(&[first.transpose(), second.transpose(), boresight.transpose()]);
        NavigationAttitudeMsg {
            time_tag_s: time_s,
            sigma_bn: dcm_to_mrp(mrp_to_dcm(camera.sigma_cb).transpose() * dcm_cn),
            ..Default::default()
        }
    }

    /// The image of the planet seen from `state`, in the projection of
    /// upstream's test truth (`-r_C / r_C[2]`), offset by [`TRUE_BIAS_PX`] and
    /// noisy.
    fn simulated_circles(
        state: &State,
        camera: &CameraConfigMsg,
        attitude: &NavigationAttitudeMsg,
        noise: &mut impl FnMut() -> f64,
    ) -> OpNavCirclesMsg {
        let position_km = state.fixed_rows::<3>(0) * 1.0e-3;
        let r_c = mrp_to_dcm(camera.sigma_cb) * mrp_to_dcm(attitude.sigma_bn) * position_km;
        let projected = -r_c / r_c.z;
        let [width, height] = camera.resolution_px.map(f64::from);
        let fov = camera.field_of_view_rad;
        let x = 2.0 * (fov * width / height / 2.0).tan() / width;
        let y = 2.0 * (fov / 2.0).tan() / height;
        let [half_width, half_height] = camera.resolution_px.map(|pixels| f64::from(pixels / 2));
        let [bx, by, br] = TRUE_BIAS_PX;
        let mut circles = OpNavCirclesMsg {
            valid: true,
            uncertainty: Matrix3::from_diagonal_element(0.5),
            ..Default::default()
        };
        circles.planet_ids[0] = 2.0;
        circles.circles_centers_px[0] = Vector2::new(
            projected.x / x + half_width - 0.5 - bx + noise(),
            projected.y / y + half_height - 0.5 - by + noise(),
        );
        circles.circles_radii_px[0] =
            (PLANET_RADIUS_KM / position_km.norm()).asin().tan() / x - br + noise();
        circles
    }

    /// Flies `setupFilterData`'s orbit for `duration_s`, rewriting the
    /// attitude every second and a valid circle every ten. Returns the filter
    /// outputs and the truth, every second.
    fn fly_with_circles(
        config: &PixelLineBiasUkfConfig,
        truth_init: State,
        duration_s: usize,
    ) -> (Vec<PixelLineFilterMsg>, Vec<State>) {
        let camera = camera(0.01, [512, 512], Vector3::new(0.1, -0.05, 0.2));
        let truth = rk4(&time_grid(duration_s as f64, duration_s + 1), truth_init);
        let (mut module, inputs) = connected_filter(
            config.clone(),
            camera.clone(),
            NavigationAttitudeMsg::default(),
        );
        let pixel_noise = Normal::new(0.0, 0.3).expect("valid distribution");
        let mut rng = StdRng::seed_from_u64(9);
        let mut noise = move || pixel_noise.sample(&mut rng);

        let mut log = Vec::new();
        for (t, state) in truth.iter().enumerate() {
            let attitude = pointing_attitude(state, &camera, t as f64);
            inputs.attitude.write(attitude.clone());
            if t > 0 && t % 10 == 0 {
                inputs
                    .circles
                    .write(simulated_circles(state, &camera, &attitude, &mut noise));
            } else if t % 10 == 1 {
                inputs.circles.write(OpNavCirclesMsg::default());
            }
            module.update(&context(nanos(t as u64)));
            log.push(module.filt_data_out_msg.read());
        }
        (log, truth)
    }

    #[test]
    fn tracks_a_simulated_orbit_within_its_own_covariance() {
        // A sub-pixel initial error (one pixel spans about 12 km here). A
        // multi-pixel one is absorbed by the bias states at the first update,
        // whose bias observation comes from the filter's own central sigma
        // point; upstream behaves the same.
        let mut config = setup_filter_data();
        let truth_init = config.state_init;
        let mut initial_position = config.state_init.fixed_rows_mut::<3>(0);
        initial_position += Vector3::new(2000.0, -1000.0, 1500.0);
        let (log, truth) = fly_with_circles(&config, truth_init, 400);

        for (t, (sample, true_state)) in log.iter().zip(&truth).enumerate() {
            for i in 0..PIXLINE_DYN_STATES {
                let error = sample.state[i] - true_state[i];
                assert!(
                    error.abs() <= 3.0 * sample.covar[(i, i)].sqrt(),
                    "state {i} at {t}: error {error}"
                );
            }
            let bias = sample.state.fixed_rows::<3>(PIXLINE_DYN_STATES);
            assert_eq!(bias.as_slice(), TRUE_BIAS_PX.as_slice(), "at {t}");
            if t > 0 && t % 10 == 0 {
                let circle_residuals = sample.post_fit_res.fixed_rows::<3>(0);
                assert!(circle_residuals.amax() < 3.0, "at {t}: {circle_residuals}");
            }
        }
        for i in 0..PIXLINE_N_STATES {
            assert!(
                log[400].covar[(i, i)] < log[0].covar[(i, i)] / 50.0,
                "state {i}"
            );
        }
    }

    #[test]
    fn residual_overflow_retimes_the_navigation_output_and_keeps_the_last_residuals() {
        let config = setup_filter_data();
        let camera = camera(0.01, [512, 512], Vector3::new(0.1, -0.05, 0.2));
        let state = config.state_init;
        let (mut module, inputs) = connected_filter(
            config.clone(),
            camera.clone(),
            NavigationAttitudeMsg::default(),
        );
        module.reset(nanos(3));
        module.update(&context(nanos(3)));
        // Before any measurement the overflow slots hold `timeTagOut` (the
        // reset time), `maxTimeJump` (0) and the circles' time tag (0).
        let residuals = module.filt_data_out_msg.read().post_fit_res;
        assert_eq!(residuals.as_slice(), &[0.0, 0.0, 0.0, 3.0, 0.0, 0.0]);
        assert_eq!(module.nav_state_out_msg.read().time_tag_s, 3.0);

        let attitude = pointing_attitude(&state, &camera, 4.0);
        inputs.attitude.write(attitude.clone());
        inputs
            .circles
            .write(simulated_circles(&state, &camera, &attitude, &mut || 0.25));
        module.update(&context(nanos(4)));
        let measured = module.filt_data_out_msg.read().post_fit_res;
        assert_eq!(module.nav_state_out_msg.read().time_tag_s, measured[3]);

        inputs.circles.write(OpNavCirclesMsg::default());
        module.update(&context(nanos(5)));
        let after = module.filt_data_out_msg.read().post_fit_res;
        assert_eq!(after.fixed_rows::<3>(0), Vector3::zeros());
        assert_eq!(after.fixed_rows::<3>(3), measured.fixed_rows::<3>(3));
        assert_eq!(module.nav_state_out_msg.read().time_tag_s, measured[3]);
    }

    #[test]
    fn reset_converts_only_the_leading_sixty_covariance_entries() {
        let mut config = setup_filter_data();
        for (row, col, value) in [(0, 7, 10.0), (1, 6, 20.0), (4, 8, 2.0)] {
            config.covar_init[(row, col)] = value;
            config.covar_init[(col, row)] = value;
        }
        let camera = camera(0.01, [512, 512], Vector3::zeros());
        let (module, _inputs) =
            connected_filter(config.clone(), camera, NavigationAttitudeMsg::default());

        for row in 0..PIXLINE_N_STATES {
            for col in 0..PIXLINE_N_STATES {
                let scale = if row * PIXLINE_N_STATES + col < CONVERTED_COVARIANCE_ENTRIES {
                    1.0e-6
                } else {
                    1.0
                };
                assert_eq!(
                    module.filter.covar[(row, col)],
                    config.covar_init[(row, col)] * scale,
                    "({row}, {col})"
                );
            }
        }
        // So the stored covariance is no longer symmetric in rows 7 and 8.
        assert_ne!(module.filter.covar[(0, 7)], module.filter.covar[(7, 0)]);
        assert_eq!(module.filter.covar[(1, 6)], module.filter.covar[(6, 1)]);
    }

    #[test]
    fn predicted_centre_uses_the_integer_half_resolution() {
        let state = State::from_column_slice(&[0.0, 0.0, -8000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let sigma_points = SigmaPoints::from_fn(|row, _| state[row]);
        let mut circles = OpNavCirclesMsg::default();
        circles.planet_ids[0] = 2.0;
        // Identity attitudes: the planet lies straight down the boresight.
        let camera = camera(0.01, [513, 511], Vector3::zeros());
        let (_, predicted) = measurement_model(
            &circles,
            &camera,
            &NavigationAttitudeMsg::default(),
            &sigma_points,
        );
        assert_eq!(predicted[(0, 0)], 256.0 - 0.5);
        assert_eq!(predicted[(1, 0)], 255.0 - 0.5);
    }

    #[test]
    fn circles_are_processed_only_when_the_attitude_is_rewritten() {
        let config = setup_filter_data();
        let camera = camera(0.01, [512, 512], Vector3::new(0.1, -0.05, 0.2));
        let state = config.state_init;
        let attitude = pointing_attitude(&state, &camera, 0.0);
        let (mut module, inputs) = connected_filter(config, camera.clone(), attitude.clone());
        let residuals = |module: &PixelLineBiasUkf| {
            module
                .filt_data_out_msg
                .read()
                .post_fit_res
                .fixed_rows::<3>(0)
                .into_owned()
        };
        // Without a measurement the filter only advances its time once it
        // propagates (`>`), so after t = 1 it is past the attitude written at 0.
        module.update(&context(0));
        module.update(&context(nanos(1)));

        inputs
            .circles
            .write(simulated_circles(&state, &camera, &attitude, &mut || 0.0));
        module.update(&context(nanos(2)));
        assert_eq!(residuals(&module), Vector3::zeros());

        inputs.attitude.write(NavigationAttitudeMsg {
            time_tag_s: 3.0,
            ..attitude
        });
        module.update(&context(nanos(3)));
        assert_ne!(residuals(&module), Vector3::zeros());
    }
}
