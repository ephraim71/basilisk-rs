//! Planet position from a circle found in a camera image: port of upstream
//! Basilisk `pixelLineConverter`.
//!
//! The centre of the first circle of an [`OpNavCirclesMsg`] gives the planet
//! direction through the pinhole camera model, and its apparent radius gives
//! the range from the planet's equatorial radius. The circle-fit covariance in
//! `(pixel, line, radius)` is mapped linearly to a position covariance. The
//! position and covariance are published in the camera, body and inertial
//! frames, in metres.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    CameraConfigMsg, Input, NavigationAttitudeMsg, OpNavCirclesMsg, OpNavMsg, Output,
};
use crate::utilities::astro_constants::{REQ_EARTH, REQ_JUPITER, REQ_MARS};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// User parameters of [`PixelLineConverter`]; the defaults are upstream's
/// zero-initialized ones.
#[derive(Clone, Debug)]
pub struct PixelLineConverterConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetTarget` [-]: targeted planet (0 none, 1 Earth, 2 Mars,
    /// 3 Jupiter). Every valid update overwrites it with the first circle's
    /// planet ID, as upstream.
    pub planet_target: i32,
}

impl Default for PixelLineConverterConfig {
    fn default() -> Self {
        Self {
            name: "pixelLineConverter".to_string(),
            planet_target: 0,
        }
    }
}

/// Circle-to-position converter: upstream `pixelLineConverter`.
#[derive(Clone, Debug)]
pub struct PixelLineConverter {
    /// Module configuration.
    pub config: PixelLineConverterConfig,
    /// Upstream `cameraConfigInMsg`: camera intrinsics and mounting.
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// Upstream `attInMsg`: spacecraft attitude.
    pub att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `circlesInMsg`: circles found in the image.
    pub circles_in_msg: Input<OpNavCirclesMsg>,
    /// Upstream `opNavOutMsg`: planet-relative position measurement.
    pub op_nav_out_msg: Output<OpNavMsg>,
}

impl PixelLineConverter {
    /// Builds the module with unconnected inputs.
    pub fn new(config: PixelLineConverterConfig) -> Self {
        Self {
            config,
            camera_config_in_msg: Input::default(),
            att_in_msg: Input::default(),
            circles_in_msg: Input::default(),
            op_nav_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_pixelLineConverter`: check that every input is
    /// connected.
    pub fn reset(&mut self) {
        if !self.camera_config_in_msg.is_connected() {
            log::error!("pixelLineConverter.camera_config_in_msg is not connected");
        }
        if !self.circles_in_msg.is_connected() {
            log::error!("pixelLineConverter.circles_in_msg is not connected");
        }
        if !self.att_in_msg.is_connected() {
            log::error!("pixelLineConverter.att_in_msg is not connected");
        }
    }

    /// Upstream `Update_pixelLineConverter`.
    fn convert(&mut self) -> OpNavMsg {
        let camera = self.camera_config_in_msg.read();
        let circles = self.circles_in_msg.read();
        let navigation = self.att_in_msg.read();

        if !circles.valid {
            return OpNavMsg::default();
        }
        // `as` truncates toward zero like upstream's `(int32_t)` cast (and
        // saturates where the C cast is undefined).
        self.config.planet_target = circles.planet_ids[0] as i32;

        let [width, height] = camera.resolution_px;
        // Upstream halves the integer resolution with integer division.
        let center = circles.circles_centers_px[0];
        let re_centered_x = center.x - f64::from(width / 2) + 0.5;
        let re_centered_y = center.y - f64::from(height / 2) + 0.5;
        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        let dcm_nc = (dcm_cb * mrp_to_dcm(navigation.sigma_bn)).transpose();

        // Pixel pitch over focal length: sensor size / focal length is
        // 2 tan(FOV / 2) per axis.
        let p_x =
            2.0 * (camera.field_of_view_rad * f64::from(width) / f64::from(height) / 2.0).tan();
        let p_y = 2.0 * (camera.field_of_view_rad / 2.0).tan();
        let x = p_x / f64::from(width);
        let y = p_y / f64::from(height);

        // The planet is along +[x, y, 1] in the camera frame, so the
        // spacecraft position relative to it is along the opposite direction.
        let r_hat_bn_c =
            normalize_or_zero(-Vector3::new(re_centered_x * x, re_centered_y * y, 1.0));
        let r_hat_bn_n = dcm_nc * r_hat_bn_c;
        let r_hat_bn_b = dcm_cb.tr_mul(&r_hat_bn_c);

        let mut measurement = OpNavMsg {
            time_tag_s: circles.time_tag_nanos as f64,
            valid: true,
            ..Default::default()
        };
        // Without a known planet the range stays 1 km and the covariances
        // zero (upstream publishes uninitialized covariances there).
        let mut range_km = 1.0;
        let mut covar_c = Matrix3::zeros();
        let mut covar_n = Matrix3::zeros();
        let mut covar_b = Matrix3::zeros();
        if self.config.planet_target > 0 {
            match equatorial_radius_km(self.config.planet_target) {
                Some(planet_radius_km) => {
                    measurement.planet_id = self.config.planet_target;
                    let radius_px = circles.circles_radii_px[0];
                    range_km = planet_radius_km / (x * radius_px).atan().sin();

                    // Linear map from the (pixel, line, radius) covariance.
                    // `powf` like upstream's `pow(.., 2)`: glibc's pow is not
                    // always the correctly rounded square that `x * x` gives.
                    let scaled = (1.0 + (radius_px * x).powf(2.0)).sqrt();
                    let rho_map =
                        planet_radius_km * (x / scaled - 1.0 / x * scaled / radius_px.powf(2.0));
                    let covar_map =
                        Matrix3::from_diagonal(&Vector3::new(range_km * x, range_km * y, rho_map));
                    covar_c = covar_map * circles.uncertainty * covar_map.transpose();
                    covar_n = dcm_nc * covar_c * dcm_nc.transpose();
                    covar_b = dcm_cb.tr_mul(&covar_c) * dcm_cb;
                }
                None => {
                    // Upstream reads an uninitialized radius here; this port
                    // keeps the unit range instead.
                    log::error!(
                        "pixelLineConverter: unsupported planet ID {}",
                        self.config.planet_target
                    );
                }
            }
        }

        // Kilometres to metres.
        measurement.position_bn_n_m = range_km * 1e3 * r_hat_bn_n;
        measurement.position_bn_c_m = range_km * 1e3 * r_hat_bn_c;
        measurement.position_bn_b_m = range_km * 1e3 * r_hat_bn_b;
        measurement.covar_n_m2 = covar_n * 1e6;
        measurement.covar_c_m2 = covar_c * 1e6;
        measurement.covar_b_m2 = covar_b * 1e6;
        measurement
    }
}

impl Module for PixelLineConverter {
    fn init(&mut self) {
        self.op_nav_out_msg.write(OpNavMsg::default());
        PixelLineConverter::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        PixelLineConverter::reset(self);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let measurement = self.convert();
        self.op_nav_out_msg.write(measurement);
    }
}

/// Equatorial radius \[km\] of an upstream planet ID (1 Earth, 2 Mars,
/// 3 Jupiter).
fn equatorial_radius_km(planet_id: i32) -> Option<f64> {
    match planet_id {
        1 => Some(REQ_EARTH),
        2 => Some(REQ_MARS),
        3 => Some(REQ_JUPITER),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, SMatrix, Vector2, Vector3};

    use crate::messages::{
        CameraConfigMsg, MAX_CIRCLE_NUM, NavigationAttitudeMsg, OpNavCirclesMsg, OpNavMsg, Output,
    };
    use crate::utilities::astro_constants::{REQ_EARTH, REQ_JUPITER};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::{PixelLineConverter, PixelLineConverterConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The camera of the upstream pytest.
    fn upstream_camera() -> CameraConfigMsg {
        CameraConfigMsg {
            field_of_view_rad: 2.0 * (10.0e-3 / 2.0 / 1.0e-3_f64).atan(),
            resolution_px: [512, 512],
            sigma_cb: Vector3::new(1.0, 0.3, 0.1),
            ..Default::default()
        }
    }

    /// The circles of the upstream pytest, which sets neither `valid` nor
    /// `planetIds`.
    fn upstream_circles() -> OpNavCirclesMsg {
        let mut circles = OpNavCirclesMsg {
            uncertainty: Matrix3::new(0.5, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0),
            time_tag_nanos: 12345,
            ..Default::default()
        };
        circles.circles_centers_px[0] = Vector2::new(152.0, 251.0);
        circles.circles_radii_px[0] = 75.0;
        circles
    }

    fn upstream_attitude() -> NavigationAttitudeMsg {
        NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.6, 1.0, 0.1),
            ..Default::default()
        }
    }

    /// Runs the module for the pytest's two updates (0 and 0.5 s).
    fn run(
        camera: CameraConfigMsg,
        circles: OpNavCirclesMsg,
        attitude: NavigationAttitudeMsg,
        planet_target: i32,
    ) -> (OpNavMsg, PixelLineConverter) {
        let camera = Output::new(camera);
        let circles = Output::new(circles);
        let attitude = Output::new(attitude);
        let mut module = PixelLineConverter::new(PixelLineConverterConfig {
            planet_target,
            ..Default::default()
        });
        camera.connect_to(&mut module.camera_config_in_msg);
        circles.connect_to(&mut module.circles_in_msg);
        attitude.connect_to(&mut module.att_in_msg);
        module.init();
        module.update(&context(0));
        module.update(&context(500_000_000));
        (module.op_nav_out_msg.read(), module)
    }

    /// Upstream `mapState` with the module's conventions: pixel coordinates
    /// re-centred on the image centre, the position pointing away from the
    /// planet, and kilometres converted to metres. The pytest's own
    /// `mapState` omits all three.
    fn upstream_map_state(
        state: [f64; 3],
        radius_km: f64,
        camera: &CameraConfigMsg,
    ) -> Vector3<f64> {
        let [width, height] = camera.resolution_px.map(f64::from);
        let p_x = 2.0 * (camera.field_of_view_rad * width / height / 2.0).tan();
        let p_y = 2.0 * (camera.field_of_view_rad / 2.0).tan();
        let d_x = p_x / width;
        let d_y = p_y / height;
        let a = 2.0 * (state[2] * d_x).atan();
        let norm = 0.5 * (2.0 * radius_km) / (0.5 * a).sin();
        let x = state[0] - (width / 2.0).floor() + 0.5;
        let y = state[1] - (height / 2.0).floor() + 0.5;
        let vec = Vector3::new(x * d_x, y * d_y, 1.0);
        -norm * vec / vec.norm() * 1e3
    }

    /// Upstream `mapCovar`, converted from km² to m².
    fn upstream_map_covar(
        covar_xyr: Matrix3<f64>,
        rho: f64,
        radius_km: f64,
        camera: &CameraConfigMsg,
    ) -> Matrix3<f64> {
        let [width, height] = camera.resolution_px.map(f64::from);
        let d = 2.0 * radius_km;
        let p_x = 2.0 * (camera.field_of_view_rad * width / height / 2.0).tan();
        let p_y = 2.0 * (camera.field_of_view_rad / 2.0).tan();
        let d_x = p_x / width;
        let d_y = p_y / height;
        let a = 2.0 * (rho * d_x).atan();
        let rho_map = 0.5
            * d
            * (-(1.0 + rho.powi(2) * d_x.powi(2)).sqrt() / (rho.powi(2) * d_x)
                + d_x / (1.0 + rho.powi(2) * d_x.powi(2)).sqrt());
        let x_map = 0.5 * d / (0.5 * a).sin() * d_x;
        let y_map = 0.5 * d / (0.5 * a).sin() * d_y;
        let covar_map = Matrix3::from_diagonal(&Vector3::new(x_map, y_map, rho_map));
        covar_map * covar_xyr * covar_map.transpose() * 1e6
    }

    /// Every element within `tolerance` times the largest expected magnitude.
    fn assert_relative<const R: usize, const C: usize>(
        actual: &SMatrix<f64, R, C>,
        expected: &SMatrix<f64, R, C>,
        tolerance: f64,
        label: &str,
    ) {
        let scale = expected.amax();
        assert!(
            (actual - expected)
                .iter()
                .all(|e| e.abs() <= tolerance * scale),
            "{label}: {actual:?} != {expected:?}"
        );
    }

    /// Port of `test_pixelLineConverter.py` with its exact inputs. The pytest
    /// never sets `valid` (nor `planetIds`), so the module publishes the
    /// zeroed invalid message; the pytest's comparisons are guarded by
    /// `np.isnan(x.any())`, which is always false, so they can never fail.
    /// This asserts what the module actually publishes for those inputs.
    #[test]
    fn matches_upstream_pixel_line_converter_unit_test() {
        let (output, module) = run(
            upstream_camera(),
            upstream_circles(),
            upstream_attitude(),
            2,
        );
        assert_eq!(output, OpNavMsg::default());
        // The invalid image returns before the target is overwritten.
        assert_eq!(module.config.planet_target, 2);
    }

    /// The pytest's scenario with a valid Mars circle, against its `mapState`
    /// and `mapCovar` truth (relative 1e-10; the pytest's absolute 1e-10 is
    /// below the resolution of doubles of this magnitude).
    #[test]
    fn converts_a_valid_circle_like_the_upstream_truth() {
        let camera = upstream_camera();
        let mut circles = upstream_circles();
        circles.valid = true;
        circles.planet_ids[0] = 2.0;
        let (output, module) = run(camera.clone(), circles.clone(), upstream_attitude(), 0);

        let radius_km = 3396.19;
        let state = [152.0, 251.0, 75.0];
        let r_c = upstream_map_state(state, radius_km, &camera);
        let covar_c = upstream_map_covar(circles.uncertainty, state[2], radius_km, &camera);
        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        let dcm_nc = (dcm_cb * mrp_to_dcm(upstream_attitude().sigma_bn)).transpose();

        assert!(output.valid);
        assert_eq!(output.planet_id, 2);
        assert_eq!(module.config.planet_target, 2);
        // Upstream copies the nanosecond time tag into the seconds field.
        assert_eq!(output.time_tag_s, 12345.0);
        assert_relative(&output.position_bn_c_m, &r_c, 1e-10, "r_BN_C");
        assert_relative(&output.position_bn_n_m, &(dcm_nc * r_c), 1e-10, "r_BN_N");
        assert_relative(
            &output.position_bn_b_m,
            &(dcm_cb.transpose() * r_c),
            1e-10,
            "r_BN_B",
        );
        assert_relative(&output.covar_c_m2, &covar_c, 1e-10, "covar_C");
        assert_relative(
            &output.covar_n_m2,
            &(dcm_nc * covar_c * dcm_nc.transpose()),
            1e-10,
            "covar_N",
        );
        assert_relative(
            &output.covar_b_m2,
            &(dcm_cb.transpose() * covar_c * dcm_cb),
            1e-10,
            "covar_B",
        );
    }

    #[test]
    fn scales_the_range_with_the_identified_planet_radius() {
        let camera = upstream_camera();
        for (planet_id, radius_km) in [(1.0, REQ_EARTH), (3.0, REQ_JUPITER)] {
            let mut circles = upstream_circles();
            circles.valid = true;
            circles.planet_ids[0] = planet_id;
            let (output, _) = run(camera.clone(), circles, upstream_attitude(), 2);
            let r_c = upstream_map_state([152.0, 251.0, 75.0], radius_km, &camera);
            assert_eq!(f64::from(output.planet_id), planet_id);
            assert_relative(&output.position_bn_c_m, &r_c, 1e-10, "r_BN_C");
        }
    }

    #[test]
    fn publishes_a_unit_range_direction_without_a_known_planet() {
        let camera = upstream_camera();
        // 0 means no planet; 4 is not a supported ID.
        for planet_id in [0.0, 4.0] {
            let mut circles = upstream_circles();
            circles.valid = true;
            circles.planet_ids = [planet_id; MAX_CIRCLE_NUM];
            let (output, module) = run(camera.clone(), circles, upstream_attitude(), 2);
            let r_c = upstream_map_state([152.0, 251.0, 75.0], 1.0, &camera);
            assert!(output.valid);
            assert_eq!(output.planet_id, 0);
            assert_eq!(f64::from(module.config.planet_target), planet_id);
            assert_relative(
                &output.position_bn_c_m,
                &(r_c / r_c.norm() * 1e3),
                1e-12,
                "unit range",
            );
            assert_eq!(output.covar_c_m2, Matrix3::zeros());
            assert_eq!(output.covar_n_m2, Matrix3::zeros());
            assert_eq!(output.covar_b_m2, Matrix3::zeros());
        }
    }
}
