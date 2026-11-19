//! Perfect navigation: port of upstream Basilisk `simpleNav` without its error
//! model.
//!
//! Republishes spacecraft truth in the navigation messages the flight-software
//! modules read: upstream `attOutMsg` ([`NavigationAttitudeMsg`]) and
//! `transOutMsg` ([`NavigationTranslationMsg`]). Upstream adds Gauss-Markov
//! errors to the truth; this port has none, so the outputs are upstream's
//! `trueAttState` / `trueTransState` with upstream's time tags.
//!
//! The accumulated ΔV is copied from the spacecraft state's
//! `TotalAccumDVBdy` ([`SpacecraftStateMsg::accumulated_dv_cn_b_mps`]), as
//! upstream does: the non-gravitational velocity change of the center of mass
//! that [`crate::spacecraft::Spacecraft`] sums in body-frame components.
//!
//! The Sun-point vector stays zero: [`SpacecraftStateMsg`] carries no Sun
//! direction and the optional Sun-state input is not ported.

use nalgebra::Vector3;

use crate::messages::{
    Input, NavigationAttitudeMsg, NavigationTranslationMsg, Output, SpacecraftStateMsg,
};
use crate::{Module, SimulationContext};

/// Noiseless navigation solution: upstream `simpleNav` without errors.
#[derive(Clone, Debug)]
pub struct SimpleNavigation {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `scStateInMsg`: spacecraft truth state.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `attOutMsg`: attitude and body rate.
    pub attitude_out_msg: Output<NavigationAttitudeMsg>,
    /// Upstream `transOutMsg`: inertial position and velocity.
    pub translation_out_msg: Output<NavigationTranslationMsg>,
}

impl SimpleNavigation {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            spacecraft_state_in_msg: Input::default(),
            attitude_out_msg: Output::default(),
            translation_out_msg: Output::default(),
        }
    }
}

impl Module for SimpleNavigation {
    fn init(&mut self) {
        if !self.spacecraft_state_in_msg.is_connected() {
            log::error!(
                "simple navigation '{}' spacecraft_state_in_msg is not connected",
                self.name
            );
        }
        self.attitude_out_msg
            .write(NavigationAttitudeMsg::default());
        self.translation_out_msg
            .write(NavigationTranslationMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.spacecraft_state_in_msg.read();
        // Upstream `(double)Clock * NANO2SEC` on both outputs.
        let time_tag_s = context.current_sim_nanos as f64 * 1.0e-9;

        self.attitude_out_msg.write(NavigationAttitudeMsg {
            time_tag_s,
            sigma_bn: state.sigma_bn,
            omega_bn_b_radps: state.omega_radps,
            vehicle_sun_point_body: Vector3::zeros(),
        });
        self.translation_out_msg.write(NavigationTranslationMsg {
            time_tag_s,
            position_bn_n_m: state.position_m,
            velocity_bn_n_mps: state.velocity_mps,
            // Upstream `v3Copy(inertialState.TotalAccumDVBdy, trueTransState.vehAccumDV)`.
            accumulated_dv_b_mps: state.accumulated_dv_cn_b_mps,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::dynamics::gravity::GravBodyData;
    use crate::fsw_algorithms::att_guidance::hill_point::{
        HillPoint, HillPointConfig, compute_hill_pointing_reference,
    };
    use crate::fsw_algorithms::att_guidance::velocity_point::{VelocityPoint, VelocityPointConfig};
    use crate::messages::{AttitudeReferenceMsg, Output, SpacecraftStateMsg};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::SimpleNavigation;

    fn assert_same_reference(actual: &AttitudeReferenceMsg, expected: &AttitudeReferenceMsg) {
        assert_eq!(actual.sigma_rn, expected.sigma_rn);
        assert_eq!(actual.omega_rn_n_radps, expected.omega_rn_n_radps);
        assert_eq!(actual.domega_rn_n_radps2, expected.domega_rn_n_radps2);
    }

    #[test]
    fn copies_spacecraft_truth_exactly() {
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(7_000_000.0, -1_000.0, 2_000.0),
            velocity_mps: Vector3::new(10.0, 7_500.0, -20.0),
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            omega_radps: Vector3::new(0.01, 0.02, -0.03),
            accumulated_dv_cn_b_mps: Vector3::new(0.5, -1.5, 2.5),
            accumulated_dv_bn_b_mps: Vector3::new(9.0, 9.0, 9.0),
            accumulated_dv_cn_n_mps: Vector3::new(-9.0, -9.0, -9.0),
        };
        let state_out = Output::new(state.clone());
        let mut navigation = SimpleNavigation::new("simple_navigation");
        navigation.spacecraft_state_in_msg.connect(state_out.slot());

        navigation.init();
        navigation.update(&SimulationContext {
            current_sim_nanos: 12_345_678_900,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });

        let attitude = navigation.attitude_out_msg.read();
        assert!((attitude.time_tag_s - 12.345_678_9).abs() < 1.0e-14);
        assert_eq!(attitude.sigma_bn, state.sigma_bn);
        assert_eq!(attitude.omega_bn_b_radps, state.omega_radps);
        assert_eq!(attitude.vehicle_sun_point_body, Vector3::zeros());

        let translation = navigation.translation_out_msg.read();
        assert_eq!(translation.time_tag_s, attitude.time_tag_s);
        assert_eq!(translation.position_bn_n_m, state.position_m);
        assert_eq!(translation.velocity_bn_n_mps, state.velocity_mps);
        assert_eq!(
            translation.accumulated_dv_b_mps, state.accumulated_dv_cn_b_mps,
            "upstream copies TotalAccumDVBdy"
        );
    }

    /// A propagated spacecraft feeds `hillPoint` and `velocityPoint` through
    /// the translation output: each reference is the one computed from the
    /// spacecraft truth, and for the circular orbit the Hill frame turns at the
    /// mean motion with its first axis along the position.
    #[test]
    fn translation_output_drives_orbit_pointing_guidance() {
        const MU_EARTH_M3PS2: f64 = 3.986_004_418e14;
        const STEP_NANOS: u64 = 1_000_000_000;
        let radius_m = 7_000_000.0;
        let inclination = 30.0_f64.to_radians();
        let speed_mps = (MU_EARTH_M3PS2 / radius_m).sqrt();
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: STEP_NANOS,
            initial_position_m: Vector3::new(radius_m, 0.0, 0.0),
            initial_velocity_mps: speed_mps
                * Vector3::new(0.0, inclination.cos(), inclination.sin()),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "earth",
                    MU_EARTH_M3PS2,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid Earth gravity body"),
            )
            .expect("unique central gravity body");
        let mut navigation = SimpleNavigation::new("simple_navigation");
        let mut hill_point = HillPoint::new(HillPointConfig::default());
        let mut velocity_point = VelocityPoint::new(VelocityPointConfig {
            mu_m3ps2: MU_EARTH_M3PS2,
            ..Default::default()
        });

        {
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            crate::connect!(&simulation,
                &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
                &navigation.translation_out_msg => &mut hill_point.trans_nav_in_msg,
                &navigation.translation_out_msg => &mut velocity_point.trans_nav_in_msg,
            );
            crate::schedule! { simulation,
                "spacecraft" => &mut spacecraft, period = STEP_NANOS, priority = 30;
                "navigation" => &mut navigation, period = STEP_NANOS, priority = 20;
                "hill_point" => &mut hill_point, period = STEP_NANOS, priority = 10;
                "velocity_point" => &mut velocity_point, period = STEP_NANOS, priority = 10;
            }
            simulation.run_for(600 * STEP_NANOS);
        }

        let truth = spacecraft.state_out.read();
        let translation = navigation.translation_out_msg.read();
        assert_eq!(translation.time_tag_s, 600.0);
        assert_eq!(translation.position_bn_n_m, truth.position_m);
        assert_eq!(translation.velocity_bn_n_mps, truth.velocity_mps);
        assert!(
            (truth.position_m - Vector3::new(radius_m, 0.0, 0.0)).norm() > 1.0e6,
            "the spacecraft has moved along its orbit"
        );

        let zero = Vector3::zeros();
        let hill = hill_point.att_ref_out_msg.read();
        assert_same_reference(
            &hill,
            &compute_hill_pointing_reference(truth.position_m, truth.velocity_mps, zero, zero),
        );
        assert_same_reference(
            &velocity_point.att_ref_out_msg.read(),
            &velocity_point.compute_velocity_pointing_reference(
                truth.position_m,
                truth.velocity_mps,
                zero,
                zero,
            ),
        );

        let dcm_rn = mrp_to_dcm(hill.sigma_rn);
        let radial = truth.position_m.normalize();
        let orbit_normal = truth.position_m.cross(&truth.velocity_mps).normalize();
        let mean_motion_radps = (MU_EARTH_M3PS2 / radius_m.powi(3)).sqrt();
        assert!((dcm_rn.row(0).transpose() - radial).norm() < 1.0e-12);
        assert!((hill.omega_rn_n_radps - mean_motion_radps * orbit_normal).norm() < 1.0e-9);
    }

    /// A body force on the spacecraft accumulates ΔV in its state message,
    /// which the navigation forwards, so `dvExecuteGuidance` cuts the burn
    /// once the ΔV since its start time exceeds the command: 0.1 m/s² from
    /// 2 s exceeds 0.955 m/s at 11.6 s.
    #[test]
    fn accumulated_dv_ends_a_dv_execute_guidance_burn() {
        use std::any::Any;

        use crate::fsw_algorithms::dv_guidance::dv_execute_guidance::{
            BurnState, DvExecuteGuidance, DvExecuteGuidanceConfig,
        };
        use crate::messages::DvBurnCommandMsg;
        use crate::spacecraft::{DynamicEffector, EffectorOutput};

        struct BodyForce;
        impl DynamicEffector for BodyForce {
            fn name(&self) -> &str {
                "body_force"
            }
            fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
                EffectorOutput {
                    force_inertial_n: state
                        .body_to_inertial()
                        .transform_vector(&Vector3::new(10.0, 0.0, 0.0)),
                    torque_body_nm: Vector3::zeros(),
                }
            }
            fn as_any(&self) -> &dyn Any {
                self
            }
        }

        const STEP_NANOS: u64 = 100_000_000;
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 100.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: STEP_NANOS,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft.add_dynamic_effector(BodyForce);
        let mut navigation = SimpleNavigation::new("simple_navigation");
        let mut guidance = DvExecuteGuidance::new(DvExecuteGuidanceConfig::default());
        let burn = Output::new(DvBurnCommandMsg {
            dv_inertial_cmd_mps: Vector3::new(0.955, 0.0, 0.0),
            burn_start_time_nanos: 2_000_000_000,
            ..Default::default()
        });
        guidance.burn_data_in_msg.connect(burn.slot());

        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        {
            let mut simulation = Simulation::new(epoch, false);
            crate::connect!(&simulation,
                &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
                &navigation.translation_out_msg => &mut guidance.nav_data_in_msg,
            );
            crate::schedule! { simulation,
                "spacecraft" => &mut spacecraft, period = STEP_NANOS, priority = 30;
                "navigation" => &mut navigation, period = STEP_NANOS, priority = 20;
                "guidance" => &mut guidance, period = STEP_NANOS, priority = 10;
            }
            simulation.run_for(11_500_000_000);
        }
        assert_eq!(guidance.burn_state, BurnState::Executing);
        assert!((guidance.dv_init_mps.x - 0.2).abs() < 1.0e-12);
        let translation = navigation.translation_out_msg.read();
        assert!((translation.accumulated_dv_b_mps.x - 1.15).abs() < 1.0e-12);
        assert_eq!(
            translation.accumulated_dv_b_mps,
            spacecraft.state_out.read().accumulated_dv_cn_b_mps
        );

        // One more step: 0.96 m/s since the start exceeds the 0.955 m/s command.
        let next = SimulationContext {
            current_sim_nanos: 11_600_000_000,
            current_epoch: epoch,
        };
        spacecraft.update(&next);
        navigation.update(&next);
        guidance.update(&next);
        assert_eq!(guidance.burn_state, BurnState::Complete);
    }
}
