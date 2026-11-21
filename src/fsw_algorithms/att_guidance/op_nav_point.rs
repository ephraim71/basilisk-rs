//! Attitude guidance that points a camera axis at the planet seen by optical
//! navigation: port of upstream Basilisk `opNavPoint`.
//!
//! The analogue of `sunSafePoint` for an optical-navigation heading: the
//! attitude error is the principal rotation from the planet heading in the
//! camera frame to the camera alignment axis, and the reference rate is a
//! constant spin about that axis. While no measurement is valid, the last
//! heading (held in the inertial frame) keeps the guidance going until
//! `time_out_s` has passed since the last valid image; after that, or when
//! there is no heading at all, the module commands a body-fixed search rate.

use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::messages::{
    AttitudeGuidanceMsg, CameraConfigMsg, Input, NavigationAttitudeMsg, OpNavMsg, Output,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::{is_zero_within, normalize_or_zero, safe_acos};
use crate::utilities::rigid_body_kinematics::{mrp_switch, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// Upstream's `v3IsZero` tolerance for "a previous heading is stored".
const STORED_HEADING_TOLERANCE: f64 = 1e-10;

/// User parameters of [`OpNavPoint`]; the defaults are upstream's
/// zero-initialized ones.
#[derive(Clone, Debug)]
pub struct OpNavPointConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `alignAxis_C` [-]: camera-frame axis to point at the planet.
    /// Reset normalizes it in place, as upstream.
    pub align_axis_c: Vector3<f64>,
    /// Upstream `minUnitMag` [-]: minimum acceptable norm of the heading.
    /// Upstream declares it but never uses it.
    pub min_unit_mag: f64,
    /// Upstream `smallAngle` \[rad\]: angle below which the heading counts as
    /// aligned with (or, near π, opposite to) the camera axis.
    pub small_angle_rad: f64,
    /// Upstream `timeOut` \[s\]: how long after the last valid image the stored
    /// heading is still used.
    pub time_out_s: f64,
    /// Upstream `omega_RN_B` \[rad/s\]: body-frame search rate commanded when
    /// no heading is available.
    pub omega_rn_b_radps: Vector3<f64>,
    /// Upstream `opNavAxisSpinRate` \[rad/s\]: spin rate about the camera axis
    /// while pointing.
    pub op_nav_axis_spin_rate_radps: f64,
}

impl Default for OpNavPointConfig {
    fn default() -> Self {
        Self {
            name: "opNavPoint".to_string(),
            align_axis_c: Vector3::zeros(),
            min_unit_mag: 0.0,
            small_angle_rad: 0.0,
            time_out_s: 0.0,
            omega_rn_b_radps: Vector3::zeros(),
            op_nav_axis_spin_rate_radps: 0.0,
        }
    }
}

/// Optical-navigation pointing guidance: upstream `opNavPoint`.
#[derive(Clone, Debug)]
pub struct OpNavPoint {
    /// Module configuration.
    pub config: OpNavPointConfig,
    /// Upstream `opnavDataInMsg`: planet position measurement.
    pub opnav_data_in_msg: Input<OpNavMsg>,
    /// Upstream `imuInMsg`: spacecraft attitude and rate.
    pub imu_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `cameraConfigInMsg`: camera mounting (`sigma_CB`).
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// Upstream `attGuidanceOutMsg`.
    pub att_guidance_out_msg: Output<AttitudeGuidanceMsg>,
    /// Upstream `opNavAngleErr` \[rad\]: last angle between the heading and the
    /// camera axis.
    pub op_nav_angle_err_rad: f64,
    /// Upstream `opNavMnvrVec` [-]: last principal rotation axis.
    pub op_nav_mnvr_vec: Vector3<f64>,
    /// Upstream `eHat180_B`: rotation axis used when the heading is opposite
    /// to the camera axis.
    e_hat_180: Vector3<f64>,
    /// Upstream `lastTime` \[ns\]; 0 doubles as "no measurement", as upstream.
    last_time_nanos: u64,
    /// Upstream `currentHeading_N`: last measured planet position, inertial
    /// components.
    current_heading_n: Vector3<f64>,
    /// Upstream `attGuidanceOutBuffer`: the guidance message, kept between
    /// updates (only Reset clears it).
    guidance: AttitudeGuidanceMsg,
}

impl OpNavPoint {
    /// Builds the module with unconnected inputs.
    pub fn new(config: OpNavPointConfig) -> Self {
        Self {
            config,
            opnav_data_in_msg: Input::default(),
            imu_in_msg: Input::default(),
            camera_config_in_msg: Input::default(),
            att_guidance_out_msg: Output::default(),
            op_nav_angle_err_rad: 0.0,
            op_nav_mnvr_vec: Vector3::zeros(),
            e_hat_180: Vector3::zeros(),
            last_time_nanos: 0,
            current_heading_n: Vector3::zeros(),
            guidance: AttitudeGuidanceMsg::default(),
        }
    }

    /// Upstream `Reset_opNavPoint`: check the inputs, normalize the camera
    /// axis, pick the half-turn axis and forget the stored heading.
    pub fn reset(&mut self) {
        if !self.opnav_data_in_msg.is_connected() {
            log::error!("opNavPoint.opnav_data_in_msg is not connected");
        }
        if !self.imu_in_msg.is_connected() {
            log::error!("opNavPoint.imu_in_msg is not connected");
        }
        if !self.camera_config_in_msg.is_connected() {
            log::error!("opNavPoint.camera_config_in_msg is not connected");
        }

        let align_axis = self.config.align_axis_c;
        if align_axis.norm() < 0.1 {
            log::error!(
                "opNavPoint.align_axis_c is not set up as a unit vector [{}, {} {}]",
                align_axis[0],
                align_axis[1],
                align_axis[2]
            );
        } else {
            self.config.align_axis_c = normalize_or_zero(align_axis);
            let mut e_hat_180 = self.config.align_axis_c.cross(&Vector3::x());
            if e_hat_180.norm() < 0.1 {
                e_hat_180 = self.config.align_axis_c.cross(&Vector3::y());
            }
            self.e_hat_180 = normalize_or_zero(e_hat_180);
        }
        self.last_time_nanos = 0;
        self.current_heading_n = Vector3::zeros();
        self.guidance = AttitudeGuidanceMsg::default();
    }

    /// Upstream `Update_opNavPoint`: refresh the retained guidance message.
    fn update_guidance(&mut self, call_time_nanos: u64) {
        let op_nav = self.opnav_data_in_msg.read();
        let navigation = self.imu_in_msg.read();
        let camera = self.camera_config_in_msg.read();

        // Upstream's zero-time sentinel: a measurement taken at t = 0 leaves
        // `last_time_nanos` at 0, so the next update also restarts the clock
        // and drops the stored heading.
        if self.last_time_nanos == 0 {
            self.last_time_nanos = call_time_nanos;
            self.current_heading_n = Vector3::zeros();
        }
        let time_without_meas_s = diff_nanos_to_seconds(call_time_nanos, self.last_time_nanos);

        let omega_bn_b = navigation.omega_bn_b_radps;
        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        let dcm_cn = dcm_cb * mrp_to_dcm(navigation.sigma_bn);

        let heading_available =
            op_nav.valid || !is_zero_within(&self.current_heading_n, STORED_HEADING_TOLERANCE);
        if heading_available && time_without_meas_s < self.config.time_out_s {
            let position_c = if op_nav.valid {
                self.last_time_nanos = call_time_nanos;
                self.current_heading_n = dcm_cn.tr_mul(&op_nav.position_bn_c_m);
                op_nav.position_bn_c_m
            } else {
                // Keep guiding on the last heading, rotated into today's
                // camera frame.
                dcm_cn * self.current_heading_n
            };
            // The planet heading is opposite to the spacecraft position.
            // Upstream scales by 1/|h| with no zero guard (not v3Normalize),
            // so a zero position gives NaN, as upstream.
            let heading_c = -position_c;
            let heading_c = heading_c * (1.0 / heading_c.norm());
            self.guidance.sigma_br = self.attitude_error(heading_c);

            let align_axis_b = dcm_cb.tr_mul(&self.config.align_axis_c);
            let omega_rn_b = self.config.op_nav_axis_spin_rate_radps * align_axis_b;
            self.guidance.omega_br_b_radps = omega_bn_b - omega_rn_b;
            self.guidance.omega_rn_b_radps = omega_rn_b;
        } else {
            self.last_time_nanos = 0;
            self.guidance.sigma_br = Vector3::zeros();
            self.guidance.omega_br_b_radps = omega_bn_b - self.config.omega_rn_b_radps;
            self.guidance.omega_rn_b_radps = self.config.omega_rn_b_radps;
        }
    }

    /// MRP attitude error `σ_BR` that rotates the camera axis onto the unit
    /// planet heading `heading_c`.
    fn attitude_error(&mut self, heading_c: Vector3<f64>) -> Vector3<f64> {
        let align_axis_c = self.config.align_axis_c;
        let cos_angle = align_axis_c.dot(&heading_c);
        let cos_angle = if cos_angle.abs() > 1.0 {
            cos_angle / cos_angle.abs()
        } else {
            cos_angle
        };
        self.op_nav_angle_err_rad = safe_acos(cos_angle);

        if self.op_nav_angle_err_rad < self.config.small_angle_rad {
            // Already aligned.
            return Vector3::zeros();
        }
        let e_hat = if PI - self.op_nav_angle_err_rad < self.config.small_angle_rad {
            self.e_hat_180
        } else {
            heading_c.cross(&align_axis_c)
        };
        self.op_nav_mnvr_vec = normalize_or_zero(e_hat);
        mrp_switch(
            (self.op_nav_angle_err_rad * 0.25).tan() * self.op_nav_mnvr_vec,
            1.0,
        )
    }
}

impl Module for OpNavPoint {
    fn init(&mut self) {
        self.att_guidance_out_msg
            .write(AttitudeGuidanceMsg::default());
        OpNavPoint::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        OpNavPoint::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.update_guidance(context.current_sim_nanos);
        self.att_guidance_out_msg.write(self.guidance.clone());
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{CameraConfigMsg, NavigationAttitudeMsg, OpNavMsg, Output};
    use crate::utilities::astro_constants::D2R;
    use crate::{Module, SimulationContext};

    use super::{OpNavPoint, OpNavPointConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The upstream test cases.
    #[derive(Clone, Copy, Debug, PartialEq)]
    enum Case {
        /// 1: the target is visible and off the camera axis.
        VisibleNotAligned,
        /// 2: the target is not visible (search rate set).
        NotVisible,
        /// 3: the target is visible on the camera axis.
        VisibleAligned,
        /// 4: the target is not visible, search.
        NotVisibleSearch,
    }

    /// Port of `test_opNavPoint.py::test_module` for all four cases: three
    /// updates at 0.5 s, every output field checked to 1e-12.
    #[test]
    fn matches_upstream_op_nav_point_unit_test() {
        let accuracy = 1e-12;
        for case in [
            Case::VisibleNotAligned,
            Case::NotVisible,
            Case::VisibleAligned,
            Case::NotVisibleSearch,
        ] {
            let searching = matches!(case, Case::NotVisible | Case::NotVisibleSearch);
            let camera_z = Vector3::new(0.0, 0.0, 1.0);
            let planet_b = Vector3::new(1.0, 1.0, 0.0);
            let omega_bn_b = Vector3::new(0.01, 0.50, -0.2);
            let omega_rn_b_search = Vector3::new(0.0, 0.0, 0.1);

            let mut config = OpNavPointConfig {
                align_axis_c: camera_z,
                min_unit_mag: 0.01,
                small_angle_rad: 0.01 * D2R,
                time_out_s: 100.0,
                ..Default::default()
            };
            if searching {
                config.omega_rn_b_radps = omega_rn_b_search;
            }
            let op_nav = Output::new(OpNavMsg {
                position_bn_c_m: if case == Case::VisibleAligned {
                    Vector3::new(0.0, 0.0, -1.0)
                } else {
                    planet_b
                },
                valid: !searching,
                ..Default::default()
            });
            let imu = Output::new(NavigationAttitudeMsg {
                omega_bn_b_radps: omega_bn_b,
                ..Default::default()
            });
            let camera = Output::new(CameraConfigMsg::default());

            let mut module = OpNavPoint::new(config);
            op_nav.connect_to(&mut module.opnav_data_in_msg);
            imu.connect_to(&mut module.imu_in_msg);
            camera.connect_to(&mut module.camera_config_in_msg);
            module.init();

            let e_hat = (-planet_b).cross(&camera_z);
            let e_hat = e_hat / e_hat.norm();
            let phi = (-planet_b / planet_b.norm()).dot(&camera_z).acos();
            let sigma_true = if case == Case::VisibleNotAligned {
                e_hat * (phi / 4.0).tan()
            } else {
                Vector3::zeros()
            };
            let (omega_br_true, omega_rn_true) = if searching {
                (omega_bn_b - omega_rn_b_search, omega_rn_b_search)
            } else {
                (omega_bn_b, Vector3::zeros())
            };

            for time_nanos in [0, 500_000_000, 1_000_000_000] {
                module.update(&context(time_nanos));
                let guidance = module.att_guidance_out_msg.read();
                let label = format!("{case:?} at {time_nanos} ns");
                assert_close(guidance.sigma_br, sigma_true, accuracy, &label);
                assert_close(guidance.omega_br_b_radps, omega_br_true, accuracy, &label);
                assert_close(guidance.omega_rn_b_radps, omega_rn_true, accuracy, &label);
                assert_close(
                    guidance.domega_rn_b_radps2,
                    Vector3::zeros(),
                    accuracy,
                    &label,
                );
            }
        }
    }

    /// Upstream's truth for a planet seen at `planet_c` by a camera whose
    /// axis is +z: `ê tan(Φ/4)` with `ê = -r × z`.
    fn pointing_error(planet_c: Vector3<f64>) -> Vector3<f64> {
        let camera_z = Vector3::z();
        let e_hat = (-planet_c).cross(&camera_z);
        let e_hat = e_hat / e_hat.norm();
        let phi = (-planet_c / planet_c.norm()).dot(&camera_z).acos();
        e_hat * (phi / 4.0).tan()
    }

    fn pointing_module(
        op_nav: &Output<OpNavMsg>,
        imu: &Output<NavigationAttitudeMsg>,
        camera: &Output<CameraConfigMsg>,
        config: OpNavPointConfig,
    ) -> OpNavPoint {
        let mut module = OpNavPoint::new(config);
        op_nav.connect_to(&mut module.opnav_data_in_msg);
        imu.connect_to(&mut module.imu_in_msg);
        camera.connect_to(&mut module.camera_config_in_msg);
        module.init();
        module
    }

    #[test]
    fn keeps_the_last_heading_until_the_time_out_then_searches() {
        let planet_c = Vector3::new(1.0, 1.0, 0.0);
        let search = Vector3::new(0.0, 0.0, 0.1);
        let op_nav = Output::new(OpNavMsg {
            position_bn_c_m: planet_c,
            valid: true,
            ..Default::default()
        });
        let imu = Output::new(NavigationAttitudeMsg::default());
        let camera = Output::new(CameraConfigMsg::default());
        let mut module = pointing_module(
            &op_nav,
            &imu,
            &camera,
            OpNavPointConfig {
                align_axis_c: Vector3::z(),
                small_angle_rad: 0.01 * D2R,
                time_out_s: 1.5,
                omega_rn_b_radps: search,
                ..Default::default()
            },
        );

        module.update(&context(1_000_000_000));
        assert_close(
            module.att_guidance_out_msg.read().sigma_br,
            pointing_error(planet_c),
            1e-12,
            "valid image",
        );

        // The image is lost and the spacecraft turns 90 degrees about z: the
        // stored inertial heading now appears at [1, -1, 0] in the camera.
        op_nav.write(OpNavMsg::default());
        imu.write(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.0, 0.0, (PI / 8.0).tan()),
            ..Default::default()
        });
        module.update(&context(2_000_000_000));
        let guidance = module.att_guidance_out_msg.read();
        assert_close(
            guidance.sigma_br,
            pointing_error(Vector3::new(1.0, -1.0, 0.0)),
            1e-12,
            "stored heading",
        );
        assert_close(
            guidance.omega_rn_b_radps,
            Vector3::zeros(),
            0.0,
            "stored heading",
        );

        // 2 s without an image exceeds the 1.5 s time out.
        for time_nanos in [3_000_000_000, 4_000_000_000] {
            module.update(&context(time_nanos));
            let guidance = module.att_guidance_out_msg.read();
            assert_close(guidance.sigma_br, Vector3::zeros(), 0.0, "search");
            assert_close(guidance.omega_br_b_radps, -search, 0.0, "search");
            assert_close(guidance.omega_rn_b_radps, search, 0.0, "search");
        }
    }

    // Upstream's zero-time sentinel: an image at t = 0 leaves the stored time
    // at 0, so the next update forgets the stored heading and searches, while
    // an image at t = 1 s (above) is kept until the time out.
    #[test]
    fn drops_a_heading_measured_at_time_zero() {
        let planet_c = Vector3::new(1.0, 1.0, 0.0);
        let search = Vector3::new(0.0, 0.0, 0.1);
        let op_nav = Output::new(OpNavMsg {
            position_bn_c_m: planet_c,
            valid: true,
            ..Default::default()
        });
        let imu = Output::new(NavigationAttitudeMsg::default());
        let camera = Output::new(CameraConfigMsg::default());
        let mut module = pointing_module(
            &op_nav,
            &imu,
            &camera,
            OpNavPointConfig {
                align_axis_c: Vector3::z(),
                small_angle_rad: 0.01 * D2R,
                time_out_s: 100.0,
                omega_rn_b_radps: search,
                ..Default::default()
            },
        );

        module.update(&context(0));
        assert_close(
            module.att_guidance_out_msg.read().sigma_br,
            pointing_error(planet_c),
            1e-12,
            "valid image at t = 0",
        );

        op_nav.write(OpNavMsg::default());
        module.update(&context(500_000_000));
        let guidance = module.att_guidance_out_msg.read();
        assert_close(guidance.sigma_br, Vector3::zeros(), 0.0, "heading dropped");
        assert_close(guidance.omega_rn_b_radps, search, 0.0, "heading dropped");
    }

    #[test]
    fn turns_about_the_half_turn_axis_when_the_heading_is_opposite() {
        let op_nav = Output::new(OpNavMsg {
            // The planet heading -r = -z is opposite to the camera axis.
            position_bn_c_m: Vector3::new(0.0, 0.0, 1.0),
            valid: true,
            ..Default::default()
        });
        let imu = Output::new(NavigationAttitudeMsg::default());
        let camera = Output::new(CameraConfigMsg::default());
        let mut module = pointing_module(
            &op_nav,
            &imu,
            &camera,
            OpNavPointConfig {
                align_axis_c: Vector3::new(0.0, 0.0, 2.0),
                small_angle_rad: 0.01 * D2R,
                time_out_s: 100.0,
                ..Default::default()
            },
        );
        // Reset normalizes the camera axis in place, as upstream.
        assert_eq!(module.config.align_axis_c, Vector3::z());

        module.update(&context(1_000_000_000));
        // ê180 = z × x = y, and tan(π/4) = 1 is not switched by MRPswitch.
        assert_close(
            module.att_guidance_out_msg.read().sigma_br,
            Vector3::y(),
            1e-15,
            "half turn",
        );
        assert!((module.op_nav_angle_err_rad - PI).abs() < 1e-15);
    }

    #[test]
    fn spins_about_the_camera_axis_in_body_components() {
        let op_nav = Output::new(OpNavMsg {
            position_bn_c_m: Vector3::new(0.0, 0.0, -1.0),
            valid: true,
            ..Default::default()
        });
        let omega_bn_b = Vector3::new(0.01, 0.02, 0.03);
        let imu = Output::new(NavigationAttitudeMsg {
            omega_bn_b_radps: omega_bn_b,
            ..Default::default()
        });
        // Camera turned 90 degrees about body x: its +z axis is body -y.
        let camera = Output::new(CameraConfigMsg {
            sigma_cb: Vector3::new((PI / 8.0).tan(), 0.0, 0.0),
            ..Default::default()
        });
        let mut module = pointing_module(
            &op_nav,
            &imu,
            &camera,
            OpNavPointConfig {
                align_axis_c: Vector3::z(),
                small_angle_rad: 0.01 * D2R,
                time_out_s: 100.0,
                op_nav_axis_spin_rate_radps: 0.3,
                ..Default::default()
            },
        );
        module.update(&context(1_000_000_000));
        let guidance = module.att_guidance_out_msg.read();
        let omega_rn_b = Vector3::new(0.0, -0.3, 0.0);
        assert_close(guidance.sigma_br, Vector3::zeros(), 0.0, "aligned");
        assert_close(guidance.omega_rn_b_radps, omega_rn_b, 1e-15, "spin");
        assert_close(
            guidance.omega_br_b_radps,
            omega_bn_b - omega_rn_b,
            1e-15,
            "spin",
        );
    }

    /// Upstream `isArrayEqual`: every component within `accuracy`; NaN fails.
    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64, label: &str) {
        assert!(
            actual
                .iter()
                .zip(expected.iter())
                .all(|(a, e)| (a - e).abs() <= accuracy),
            "{label}: {actual:?} != {expected:?}"
        );
    }
}
