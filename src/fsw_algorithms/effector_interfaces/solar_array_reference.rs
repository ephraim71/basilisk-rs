//! Reference angle and rate for a rotating solar array.
//!
//! Port of upstream Basilisk `solarArrayReference`. The array rotates about
//! the body-fixed drive axis `a1`; `a2` is its power-generating surface normal
//! at zero rotation. In [`PointingMode::PowerGeneration`] the reference angle
//! turns `a2` towards the Sun direction projected onto the plane normal to
//! `a1`, using either the reference attitude or the current body attitude
//! ([`AttitudeFrame`]). In [`PointingMode::MomentumManagement`] the angle is
//! additionally offset by `2 Θ_max / π · atan(σ f^n)` whenever
//! `f = (r̂_S × r_AC) · H_RW` is positive, so that the array's solar radiation
//! pressure torque works against the net wheel momentum `H_RW`. The reference
//! is kept within π of the current angle and its rate is a finite difference
//! of successive outputs.

use std::f64::consts::PI;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeReferenceMsg, HingedRigidBodyMsg, Input, MAX_EFF_COUNT, NavigationAttitudeMsg, Output,
    RwArrayConfigMsg, RwSpeedMsg, VehicleConfigMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream module tolerance for a zero projected Sun direction.
const EPSILON: f64 = 1e-12;

/// Frame whose Sun direction sets the reference angle (upstream
/// `attitudeFrame`); discriminants are the upstream codes.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum AttitudeFrame {
    /// Sun direction mapped into the reference frame R (upstream
    /// `referenceFrame = 0`, the default).
    #[default]
    Reference = 0,
    /// Sun direction in the current body frame B (upstream `bodyFrame = 1`).
    Body = 1,
}

/// What the reference angle optimizes (upstream `pointingMode`);
/// discriminants are the upstream codes.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum PointingMode {
    /// Maximum Sun incidence (upstream `powerGeneration = 0`, the default).
    #[default]
    PowerGeneration = 0,
    /// Sun incidence plus an SRP momentum-dumping offset (upstream
    /// `momentumManagement = 1`).
    MomentumManagement = 1,
}

/// User-settable parameters of [`SolarArrayReference`]; upstream
/// zero-initializes all of them.
#[derive(Clone, Debug)]
pub struct SolarArrayReferenceConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Array drive axis, body frame; normalized internally (upstream `a1Hat_B`).
    pub a1_hat_b: Vector3<f64>,
    /// Array surface normal at zero rotation, body frame; its component
    /// normal to `a1_hat_b` is used (upstream `a2Hat_B`).
    pub a2_hat_b: Vector3<f64>,
    /// Frame of the Sun direction (upstream `attitudeFrame`).
    pub attitude_frame: AttitudeFrame,
    /// Requested pointing mode (upstream `pointingMode`). Reset downgrades it
    /// to [`PointingMode::PowerGeneration`] if the wheel messages are missing.
    pub pointing_mode: PointingMode,
    /// Array center of pressure, body frame (upstream `r_AB_B`) \[m\].
    pub r_ab_b_m: Vector3<f64>,
    /// Maximum momentum-management deflection (upstream `ThetaMax`) \[rad\].
    pub theta_max_rad: f64,
    /// Momentum-management gain (upstream `sigma`) \[-\].
    pub sigma: f64,
    /// Momentum-management exponent; `n > 1` adds a deadband (upstream `n`) \[-\].
    pub n: f64,
}

impl Default for SolarArrayReferenceConfig {
    fn default() -> Self {
        Self {
            name: "solarArrayReference".to_string(),
            a1_hat_b: Vector3::zeros(),
            a2_hat_b: Vector3::zeros(),
            attitude_frame: AttitudeFrame::Reference,
            pointing_mode: PointingMode::PowerGeneration,
            r_ab_b_m: Vector3::zeros(),
            theta_max_rad: 0.0,
            sigma: 0.0,
            n: 0.0,
        }
    }
}

/// Computes the solar-array reference angle and rate.
#[derive(Clone, Debug)]
pub struct SolarArrayReference {
    /// Module configuration.
    pub config: SolarArrayReferenceConfig,
    /// Current attitude and body-frame Sun direction (upstream `attNavInMsg`).
    pub att_nav_in_msg: Input<NavigationAttitudeMsg>,
    /// Reference attitude (upstream `attRefInMsg`).
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Vehicle center of mass, used in momentum management (upstream `vehConfigInMsg`).
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Wheel speeds, optional (upstream `rwSpeedsInMsg`).
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Wheel array configuration, optional (upstream `rwConfigDataInMsg`).
    pub rw_config_data_in_msg: Input<RwArrayConfigMsg>,
    /// Current array angle (upstream `hingedRigidBodyInMsg`).
    pub hinged_rigid_body_in_msg: Input<HingedRigidBodyMsg>,
    /// Reference array angle and rate (upstream `hingedRigidBodyRefOutMsg`).
    pub hinged_rigid_body_ref_out_msg: Output<HingedRigidBodyMsg>,
    rw_config_params: RwArrayConfigMsg,
    prior_time_nanos: u64,
    /// Previous reference angle; `None` until the first update after a reset
    /// (upstream `count == 0`).
    prior_theta_ref_rad: Option<f64>,
}

impl SolarArrayReference {
    /// Creates the module with unconnected ports and no prior reference.
    pub fn new(config: SolarArrayReferenceConfig) -> Self {
        Self {
            config,
            att_nav_in_msg: Input::default(),
            att_ref_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_config_data_in_msg: Input::default(),
            hinged_rigid_body_in_msg: Input::default(),
            hinged_rigid_body_ref_out_msg: Output::default(),
            rw_config_params: RwArrayConfigMsg::default(),
            prior_time_nanos: 0,
            prior_theta_ref_rad: None,
        }
    }

    fn reset_at(&mut self, call_time_nanos: u64) {
        if !self.att_nav_in_msg.is_connected() {
            log::error!("solarArrayReference.att_nav_in_msg is not connected");
        }
        if !self.att_ref_in_msg.is_connected() {
            log::error!("solarArrayReference.att_ref_in_msg is not connected");
        }
        if !self.hinged_rigid_body_in_msg.is_connected() {
            log::error!("solarArrayReference.hinged_rigid_body_in_msg is not connected");
        }
        self.prior_theta_ref_rad = None;

        // Upstream overwrites the configured mode, so a downgrade persists
        // across later resets.
        if self.config.pointing_mode == PointingMode::MomentumManagement {
            if !self.rw_speeds_in_msg.is_connected() {
                self.config.pointing_mode = PointingMode::PowerGeneration;
                log::warn!(
                    "solarArrayReference.rw_speeds_in_msg is not connected; \
                     switching to power generation mode"
                );
            }
            if self.rw_config_data_in_msg.is_connected() {
                self.rw_config_params = self.rw_config_data_in_msg.read();
            } else {
                self.config.pointing_mode = PointingMode::PowerGeneration;
                log::warn!(
                    "solarArrayReference.rw_config_data_in_msg is not connected; \
                     switching to power generation mode"
                );
            }
        }
        self.prior_time_nanos = call_time_nanos;
    }

    fn compute_reference(&mut self, call_time_nanos: u64) -> HingedRigidBodyMsg {
        let navigation = self.att_nav_in_msg.read();
        let reference = self.att_ref_in_msg.read();
        let current = self.hinged_rigid_body_in_msg.read();

        // [RB]; `None` when the body frame itself is the reference.
        let dcm_rb = match self.config.attitude_frame {
            AttitudeFrame::Reference => {
                Some(mrp_to_dcm(reference.sigma_rn) * mrp_to_dcm(navigation.sigma_bn).transpose())
            }
            AttitudeFrame::Body => None,
        };
        let r_hat_sb_r = to_reference_frame(
            dcm_rb.as_ref(),
            normalize_or_zero(navigation.vehicle_sun_point_body),
        );

        let mut theta_ref = self.sun_reference_angle(r_hat_sb_r, current.theta_rad);
        if self.config.pointing_mode == PointingMode::MomentumManagement {
            theta_ref += self.momentum_management_offset(r_hat_sb_r, dcm_rb.as_ref());
        }

        // Keep the reference within π of the current angle.
        let theta_current = current.theta_rad.sin().atan2(current.theta_rad.cos());
        let difference = theta_ref - theta_current;
        let theta_rad = if difference > PI {
            current.theta_rad + theta_ref - theta_current - 2.0 * PI
        } else if difference < -PI {
            current.theta_rad + theta_ref - theta_current + 2.0 * PI
        } else {
            current.theta_rad + theta_ref - theta_current
        };

        let theta_dot_radps = match self.prior_theta_ref_rad {
            None => 0.0,
            Some(prior) => {
                (theta_rad - prior) / diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos)
            }
        };
        self.prior_theta_ref_rad = Some(theta_rad);
        self.prior_time_nanos = call_time_nanos;

        HingedRigidBodyMsg {
            theta_rad,
            theta_dot_radps,
        }
    }

    /// Angle that turns the array normal towards the Sun direction projected
    /// onto the plane normal to the drive axis; `current_theta_rad` when the
    /// Sun lies along the drive axis.
    fn sun_reference_angle(&self, r_hat_sb_r: Vector3<f64>, current_theta_rad: f64) -> f64 {
        let a1_hat_b = normalize_or_zero(self.config.a1_hat_b);
        let a3_hat_b = normalize_or_zero(a1_hat_b.cross(&self.config.a2_hat_b));
        let a2_hat_b = a3_hat_b.cross(&a1_hat_b);

        let a2_hat_r = normalize_or_zero(r_hat_sb_r - a1_hat_b.dot(&r_hat_sb_r) * a1_hat_b);
        let a1_hat_r = a2_hat_b.cross(&a2_hat_r);

        if a2_hat_r.norm() < EPSILON {
            return current_theta_rad;
        }
        // `max`/`min` rather than `clamp`: like fmax/fmin, they discard NaN.
        #[allow(clippy::manual_clamp)]
        let theta = a2_hat_b.dot(&a2_hat_r).max(-1.0).min(1.0).acos();
        if a1_hat_b.dot(&a1_hat_r) < 0.0 {
            -theta
        } else {
            theta
        }
    }

    /// Momentum-dumping deflection added to the Sun reference angle \[rad\].
    fn momentum_management_offset(
        &self,
        r_hat_sb_r: Vector3<f64>,
        dcm_rb: Option<&Matrix3<f64>>,
    ) -> f64 {
        let speeds = self.rw_speeds_in_msg.read();
        // Upstream would read past its arrays for num_rw > MAX_EFF_COUNT.
        let wheels = self.rw_config_params.num_rw.min(MAX_EFF_COUNT);
        let hs_b = (0..wheels).fold(Vector3::zeros(), |sum, i| {
            sum + (self.rw_config_params.spin_axis_inertias_kg_m2[i] * speeds.wheel_speeds_radps[i])
                * self.rw_config_params.spin_axes_body[i]
        });
        let hs_r = to_reference_frame(dcm_rb, hs_b);

        let r_ac_b = self.config.r_ab_b_m - self.veh_config_in_msg.read().center_of_mass_body_m;
        let f = r_hat_sb_r.cross(&r_ac_b).dot(&hs_r);
        if f > 0.0 {
            2.0 * self.config.theta_max_rad / PI
                * (self.config.sigma * f.powf(self.config.n)).atan()
        } else {
            0.0
        }
    }
}

/// Maps body components into the attitude frame the reference is computed in.
fn to_reference_frame(dcm_rb: Option<&Matrix3<f64>>, v_b: Vector3<f64>) -> Vector3<f64> {
    dcm_rb.map_or(v_b, |dcm_rb| dcm_rb * v_b)
}

impl Module for SolarArrayReference {
    fn init(&mut self) {
        self.hinged_rigid_body_ref_out_msg
            .write(HingedRigidBodyMsg::default());
        self.reset_at(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_at(context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let reference = self.compute_reference(context.current_sim_nanos);
        self.hinged_rigid_body_ref_out_msg.write(reference);
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        AttitudeReferenceMsg, HingedRigidBodyMsg, NavigationAttitudeMsg, Output, RwArrayConfigMsg,
        RwSpeedMsg, VehicleConfigMsg,
    };
    use crate::utilities::astro_constants::RPM;
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::{AttitudeFrame, PointingMode, SolarArrayReference, SolarArrayReferenceConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Port of the pytest's `computeSunReferenceAngle`, including its
    /// `theta -= π` wrap (never reached with `theta0 = 0`).
    fn compute_sun_reference_angle(
        sigma_rn: Vector3<f64>,
        r_hat_sb_n: Vector3<f64>,
        a1_hat_b: Vector3<f64>,
        a2_hat_b: Vector3<f64>,
        theta0: f64,
    ) -> f64 {
        let r_s_r = mrp_to_dcm(sigma_rn) * r_hat_sb_n;
        let a2_r = r_s_r - r_s_r.dot(&a1_hat_b) * a1_hat_b;
        let a2_r_norm = a2_r.norm();
        let mut theta = if a2_r_norm > 1e-6 {
            let a2_r = a2_r / a2_r_norm;
            let theta = a2_hat_b.dot(&a2_r).clamp(-1.0, 1.0).acos();
            if a1_hat_b.dot(&a2_hat_b.cross(&a2_r)) < 0.0 {
                -theta
            } else {
                theta
            }
        } else {
            theta0
        };
        if theta - theta0 > PI {
            theta -= PI;
        } else if theta - theta0 < -PI {
            theta += PI;
        }
        theta
    }

    struct Case {
        r_hat_sb_n: Vector3<f64>,
        sigma_bn: Vector3<f64>,
        sigma_rn: Vector3<f64>,
        attitude_frame: AttitudeFrame,
        pointing_mode: PointingMode,
    }

    const R_AB_B: [f64; 3] = [4.5, 0.0, 0.5];
    const THETA_MAX: f64 = PI / 2.0;

    fn wheel_config() -> RwArrayConfigMsg {
        let mut config = RwArrayConfigMsg {
            num_rw: 3,
            ..Default::default()
        };
        config.spin_axes_body[..3].copy_from_slice(&[Vector3::x(), Vector3::y(), Vector3::z()]);
        config.spin_axis_inertias_kg_m2[..3].copy_from_slice(&[0.15; 3]);
        config
    }

    fn wheel_speeds() -> RwSpeedMsg {
        let mut speeds = RwSpeedMsg::default();
        speeds.wheel_speeds_radps[..3].copy_from_slice(&[RPM * 1000.0; 3]);
        speeds
    }

    /// Truth of `solarArrayRotationTestFunction` (`thetaC = 0`, `sigma = n = 1`).
    fn upstream_truth(case: &Case) -> f64 {
        let a1_hat_b = Vector3::x();
        let a2_hat_b = Vector3::y();
        let theta_c = 0.0;
        let r_hat_sb_b = mrp_to_dcm(case.sigma_bn) * case.r_hat_sb_n;
        let theta_sun_r = match case.attitude_frame {
            AttitudeFrame::Reference => compute_sun_reference_angle(
                case.sigma_rn,
                case.r_hat_sb_n,
                a1_hat_b,
                a2_hat_b,
                theta_c,
            ),
            AttitudeFrame::Body => compute_sun_reference_angle(
                case.sigma_bn,
                case.r_hat_sb_n,
                a1_hat_b,
                a2_hat_b,
                theta_c,
            ),
        };

        let mut theta_r = match case.pointing_mode {
            PointingMode::PowerGeneration => theta_sun_r,
            PointingMode::MomentumManagement => {
                let config = wheel_config();
                let speeds = wheel_speeds();
                let mut h_b = Vector3::zeros();
                for i in 0..config.num_rw {
                    for j in 0..3 {
                        h_b[j] += config.spin_axes_body[i][j]
                            * config.spin_axis_inertias_kg_m2[i]
                            * speeds.wheel_speeds_radps[i];
                    }
                }
                let (s_hat, h_r) = match case.attitude_frame {
                    AttitudeFrame::Reference => {
                        let rb = mrp_to_dcm(case.sigma_rn) * mrp_to_dcm(case.sigma_bn).transpose();
                        (rb * r_hat_sb_b, rb * h_b)
                    }
                    AttitudeFrame::Body => (r_hat_sb_b, h_b),
                };
                let f = s_hat.cross(&Vector3::from(R_AB_B)).dot(&h_r);
                if f > 0.0 {
                    theta_sun_r + 2.0 * THETA_MAX / PI * (1.0 * f.powf(1.0)).atan()
                } else {
                    theta_sun_r
                }
            }
        };

        // math.fmod keeps the sign of the dividend, like Rust's `%`.
        let theta_diff = (theta_r - theta_c) % (2.0 * PI);
        if theta_diff > PI {
            theta_r -= 2.0 * PI;
        } else if theta_diff < -PI {
            theta_r += 2.0 * PI;
        }
        theta_r
    }

    /// Port of `test_solarArrayRotation`, every parametrization: 1 s task
    /// rate, stopped at 0.5 s, so the single t = 0 sample is checked.
    #[test]
    fn matches_upstream_solar_array_rotation_unit_test() {
        let accuracy = 1e-12;
        for r_hat_sb_n in [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)] {
            for sigma_bn in [Vector3::new(0.1, 0.2, 0.3), Vector3::new(0.5, 0.4, 0.3)] {
                for sigma_rn in [Vector3::new(0.3, 0.2, 0.1), Vector3::new(0.9, 0.7, 0.8)] {
                    for attitude_frame in [AttitudeFrame::Reference, AttitudeFrame::Body] {
                        for pointing_mode in [
                            PointingMode::PowerGeneration,
                            PointingMode::MomentumManagement,
                        ] {
                            let case = Case {
                                r_hat_sb_n,
                                sigma_bn,
                                sigma_rn,
                                attitude_frame,
                                pointing_mode,
                            };
                            let actual = run_case(&case);
                            let expected = upstream_truth(&case);
                            assert!(
                                (actual.theta_rad - expected).abs() <= accuracy,
                                "rHat_SB_N={r_hat_sb_n:?}, sigma_BN={sigma_bn:?}, \
                                 sigma_RN={sigma_rn:?}, {attitude_frame:?}, {pointing_mode:?}: \
                                 expected {expected}, got {}",
                                actual.theta_rad
                            );
                            assert!(actual.theta_dot_radps.abs() <= accuracy);
                        }
                    }
                }
            }
        }
    }

    fn run_case(case: &Case) -> HingedRigidBodyMsg {
        let navigation = Output::new(NavigationAttitudeMsg {
            sigma_bn: case.sigma_bn,
            vehicle_sun_point_body: mrp_to_dcm(case.sigma_bn) * case.r_hat_sb_n,
            ..Default::default()
        });
        let reference = Output::new(AttitudeReferenceMsg {
            sigma_rn: case.sigma_rn,
            ..Default::default()
        });
        let hinged = Output::new(HingedRigidBodyMsg::default());
        let vehicle = Output::new(VehicleConfigMsg::default());
        let wheels = Output::new(wheel_config());
        let speeds = Output::new(wheel_speeds());

        let mut module = SolarArrayReference::new(SolarArrayReferenceConfig {
            a1_hat_b: Vector3::x(),
            a2_hat_b: Vector3::y(),
            r_ab_b_m: Vector3::from(R_AB_B),
            pointing_mode: case.pointing_mode,
            attitude_frame: case.attitude_frame,
            theta_max_rad: THETA_MAX,
            sigma: 1.0,
            n: 1.0,
            ..Default::default()
        });
        navigation.connect_to(&mut module.att_nav_in_msg);
        reference.connect_to(&mut module.att_ref_in_msg);
        hinged.connect_to(&mut module.hinged_rigid_body_in_msg);
        if case.pointing_mode == PointingMode::MomentumManagement {
            vehicle.connect_to(&mut module.veh_config_in_msg);
            wheels.connect_to(&mut module.rw_config_data_in_msg);
            speeds.connect_to(&mut module.rw_speeds_in_msg);
        }
        module.init();
        module.update(&context(0));
        module.hinged_rigid_body_ref_out_msg.read()
    }

    fn sun_along(axis: Vector3<f64>) -> SolarArrayReference {
        let navigation = Output::new(NavigationAttitudeMsg {
            vehicle_sun_point_body: axis,
            ..Default::default()
        });
        let mut module = SolarArrayReference::new(SolarArrayReferenceConfig {
            a1_hat_b: Vector3::x(),
            a2_hat_b: Vector3::y(),
            attitude_frame: AttitudeFrame::Body,
            ..Default::default()
        });
        navigation.connect_to(&mut module.att_nav_in_msg);
        module
    }

    #[test]
    fn holds_the_current_angle_when_the_sun_is_along_the_drive_axis() {
        let mut module = sun_along(Vector3::new(2.0, 0.0, 0.0));
        let hinged = Output::new(HingedRigidBodyMsg {
            theta_rad: 0.7,
            theta_dot_radps: 0.0,
        });
        hinged.connect_to(&mut module.hinged_rigid_body_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(module.hinged_rigid_body_ref_out_msg.read().theta_rad, 0.7);
    }

    #[test]
    fn keeps_the_reference_within_half_a_turn_of_the_current_angle() {
        // Sun along -z: the Sun angle is -π/2 in [-π, π]; with the array at
        // 3π (wrapped to ≈π) the reference stays within π of it, at 3π + π/2.
        let mut module = sun_along(-Vector3::z());
        let hinged = Output::new(HingedRigidBodyMsg {
            theta_rad: 3.0 * PI,
            theta_dot_radps: 0.0,
        });
        hinged.connect_to(&mut module.hinged_rigid_body_in_msg);
        module.init();
        module.update(&context(0));
        let theta = module.hinged_rigid_body_ref_out_msg.read().theta_rad;
        assert!((theta - 3.5 * PI).abs() <= 1e-12, "got {theta}");
    }

    #[test]
    fn differences_successive_references_for_the_rate() {
        let navigation = Output::new(NavigationAttitudeMsg {
            vehicle_sun_point_body: Vector3::y(),
            ..Default::default()
        });
        let hinged = Output::new(HingedRigidBodyMsg::default());
        let mut module = SolarArrayReference::new(SolarArrayReferenceConfig {
            a1_hat_b: Vector3::x(),
            a2_hat_b: Vector3::y(),
            attitude_frame: AttitudeFrame::Body,
            ..Default::default()
        });
        navigation.connect_to(&mut module.att_nav_in_msg);
        hinged.connect_to(&mut module.hinged_rigid_body_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(module.hinged_rigid_body_ref_out_msg.read().theta_rad, 0.0);

        // Sun moves to +z: reference π/2 reached over 2 s.
        navigation.write(NavigationAttitudeMsg {
            vehicle_sun_point_body: Vector3::z(),
            ..Default::default()
        });
        module.update(&context(2_000_000_000));
        let output = module.hinged_rigid_body_ref_out_msg.read();
        assert!((output.theta_rad - PI / 2.0).abs() <= 1e-15);
        assert!((output.theta_dot_radps - PI / 4.0).abs() <= 1e-15);

        // A reset restarts the finite difference.
        Module::reset(&mut module, &context(3_000_000_000));
        module.update(&context(4_000_000_000));
        assert_eq!(
            module.hinged_rigid_body_ref_out_msg.read().theta_dot_radps,
            0.0
        );
    }

    #[test]
    fn missing_wheel_messages_downgrade_to_power_generation() {
        let mut module = SolarArrayReference::new(SolarArrayReferenceConfig {
            pointing_mode: PointingMode::MomentumManagement,
            ..Default::default()
        });
        let wheels = Output::new(wheel_config());
        wheels.connect_to(&mut module.rw_config_data_in_msg);
        module.init();
        assert_eq!(module.config.pointing_mode, PointingMode::PowerGeneration);
    }
}
