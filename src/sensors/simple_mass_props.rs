//! Perfect mass-properties sensor: port of upstream Basilisk
//! `simpleMassProps`.
//!
//! Converts the simulation-side spacecraft mass properties (upstream
//! `SCMassPropsMsg`, here [`SpacecraftMassPropsMsg`], published by
//! [`crate::spacecraft::Spacecraft::mass_props_out`]) into the flight-software
//! [`VehicleConfigMsg`]: the mass, the inertia about the body origin `B` in
//! body components (upstream flattens it row-major into `ISCPntB_B[9]`; here
//! both are `Matrix3`), and the center of mass in body components. The ADCS
//! state flag stays zero. No errors are applied.
//!
//! As upstream, a reset also runs an update, so the vehicle configuration is
//! populated before the first task tick. Flight-software modules that read
//! the configuration only on reset, such as
//! [`crate::fsw_algorithms::att_control::mrp_feedback::MrpFeedback`], need
//! this; schedule this module at a higher priority than they are so its reset
//! runs first.

use crate::messages::{Input, Output, SpacecraftMassPropsMsg, VehicleConfigMsg};
use crate::{Module, SimulationContext};

/// Mass properties to vehicle configuration: upstream `SimpleMassProps`.
#[derive(Clone, Debug)]
pub struct SimpleMassProps {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `scMassPropsInMsg`: spacecraft mass properties.
    pub sc_mass_props_in_msg: Input<SpacecraftMassPropsMsg>,
    /// Upstream `vehicleConfigOutMsg`: flight-software vehicle configuration.
    pub vehicle_config_out_msg: Output<VehicleConfigMsg>,
}

impl SimpleMassProps {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            sc_mass_props_in_msg: Input::default(),
            vehicle_config_out_msg: Output::default(),
        }
    }

    fn check_connections(&self) {
        if !self.sc_mass_props_in_msg.is_connected() {
            log::error!(
                "simpleMassProps '{}' sc_mass_props_in_msg is not linked",
                self.name
            );
        }
    }

    /// Upstream `readInputMessages`, `computeMassProperties` and
    /// `writeOutputMessages`.
    fn publish(&mut self) {
        let mass_props = self.sc_mass_props_in_msg.read();
        self.vehicle_config_out_msg.write(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: mass_props.inertia_about_point_b_body_kg_m2,
            center_of_mass_body_m: mass_props.center_of_mass_body_m,
            mass_kg: mass_props.mass_kg,
            ..VehicleConfigMsg::default()
        });
    }
}

impl Module for SimpleMassProps {
    fn init(&mut self) {
        self.check_connections();
        self.publish();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
        self.publish();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.publish();
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `simpleMassProps/_UnitTest/test_simpleMassProps.py`
    //! with its 1e-8 accuracy, and adds an end-to-end case: a spacecraft
    //! feeds this module, which feeds `mrpFeedback` in a closed attitude loop.

    use std::any::Any;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
    use crate::fsw_algorithms::att_guidance::att_tracking_error::{
        AttTrackingError, AttTrackingErrorConfig,
    };
    use crate::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
    use crate::messages::{
        BodyTorqueCommandMsg, Input, Output, SpacecraftMassPropsMsg, SpacecraftStateMsg,
        VehicleConfigMsg,
    };
    use crate::sensors::navigation::SimpleNavigation;
    use crate::simulation::Simulation;
    use crate::spacecraft::{DynamicEffector, EffectorOutput, Spacecraft, SpacecraftConfig};
    use crate::utilities::rigid_body_kinematics::sub_mrp;
    use crate::{Module, SimulationContext};

    use super::SimpleMassProps;

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    fn assert_config(actual: &VehicleConfigMsg, mass: f64, inertia: [f64; 9], com: [f64; 3]) {
        let accuracy = 1.0e-8;
        assert!(
            (actual.mass_kg - mass).abs() < accuracy,
            "mass {}",
            actual.mass_kg
        );
        // Upstream checks the row-major `ISCPntB_B[9]`.
        let expected_inertia = Matrix3::from_row_slice(&inertia);
        assert!(
            (actual.inertia_about_point_b_kg_m2 - expected_inertia).amax() < accuracy,
            "inertia {}",
            actual.inertia_about_point_b_kg_m2
        );
        assert!(
            (actual.center_of_mass_body_m - Vector3::from(com)).amax() < accuracy,
            "center of mass {}",
            actual.center_of_mass_body_m
        );
        assert_eq!(actual.current_adcs_state, 0);
    }

    /// Upstream `test_module`: one step with the first mass properties, then
    /// one with the second; each step's output carries that step's input.
    #[test]
    fn copies_mass_inertia_and_center_of_mass_each_step() {
        let mut module = SimpleMassProps::new("scMassPropsModule");
        let mass_props = Output::new(SpacecraftMassPropsMsg {
            mass_kg: 100.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_about_point_b_body_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                40.0, 50.0, 60.0,
            )),
        });
        mass_props.connect_to(&mut module.sc_mass_props_in_msg);
        module.init();

        let context = |nanos| SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: epoch(),
        };
        module.update(&context(0));
        let first = module.vehicle_config_out_msg.read();

        mass_props.write(SpacecraftMassPropsMsg {
            mass_kg: 500.0,
            center_of_mass_body_m: Vector3::new(1.0, -1.0, 0.0),
            inertia_about_point_b_body_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                200.0, 300.0, 400.0,
            )),
        });
        module.update(&context(1_000_000_000));
        let second = module.vehicle_config_out_msg.read();

        assert_config(
            &first,
            100.0,
            [40.0, 0.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 60.0],
            [0.0, 0.0, 0.0],
        );
        assert_config(
            &second,
            500.0,
            [200.0, 0.0, 0.0, 0.0, 300.0, 0.0, 0.0, 0.0, 400.0],
            [1.0, -1.0, 0.0],
        );
    }

    /// The off-diagonal products of inertia keep their row and column.
    #[test]
    fn keeps_the_inertia_layout() {
        let inertia = Matrix3::new(10.0, -1.0, 2.0, -1.5, 20.0, -3.0, 2.5, -3.5, 30.0);
        let mut module = SimpleMassProps::new("layout");
        let mass_props = Output::new(SpacecraftMassPropsMsg {
            mass_kg: 1.0,
            center_of_mass_body_m: Vector3::new(0.1, 0.2, 0.3),
            inertia_about_point_b_body_kg_m2: inertia,
        });
        mass_props.connect_to(&mut module.sc_mass_props_in_msg);
        module.init();
        assert_eq!(
            module
                .vehicle_config_out_msg
                .read()
                .inertia_about_point_b_kg_m2,
            inertia
        );
    }

    /// A reset publishes the configuration without waiting for an update.
    #[test]
    fn reset_publishes_before_the_first_update() {
        let mut module = SimpleMassProps::new("reset");
        let mass_props = Output::new(SpacecraftMassPropsMsg {
            mass_kg: 42.0,
            ..Default::default()
        });
        mass_props.connect_to(&mut module.sc_mass_props_in_msg);
        module.init();
        assert_eq!(module.vehicle_config_out_msg.read().mass_kg, 42.0);

        mass_props.write(SpacecraftMassPropsMsg {
            mass_kg: 7.0,
            ..Default::default()
        });
        Module::reset(
            &mut module,
            &SimulationContext {
                current_sim_nanos: 0,
                current_epoch: epoch(),
            },
        );
        assert_eq!(module.vehicle_config_out_msg.read().mass_kg, 7.0);
    }

    /// Applies the latest commanded body torque to the hub.
    struct CommandedTorque {
        torque_in_msg: Input<BodyTorqueCommandMsg>,
    }

    impl DynamicEffector for CommandedTorque {
        fn name(&self) -> &str {
            "commanded_torque"
        }
        fn compute_output(&self, _state: &SpacecraftStateMsg) -> EffectorOutput {
            EffectorOutput {
                force_inertial_n: Vector3::zeros(),
                torque_body_nm: self.torque_in_msg.read().torque_request_body_nm,
            }
        }
        fn as_any(&self) -> &dyn Any {
            self
        }
    }

    /// End to end: `Spacecraft` → `simpleMassProps` → `mrpFeedback`. The hub
    /// center of mass is off the body origin, so the inertia about `B` the
    /// controller needs carries the parallel-axis term. `mrpFeedback` reads
    /// the vehicle configuration only on reset, so its first command shows it
    /// received the spacecraft's inertia about `B` before the first tick; the
    /// closed loop then drives the attitude to an inertial target.
    #[test]
    fn spacecraft_mass_properties_drive_mrp_feedback() {
        const STEP_NANOS: u64 = 100_000_000;
        let mass_kg = 100.0;
        let hub_com_m = Vector3::new(0.1, -0.05, 0.2);
        let hub_inertia = Matrix3::new(10.0, 0.2, -0.1, 0.2, 12.0, 0.3, -0.1, 0.3, 14.0);
        let initial_omega_radps = Vector3::new(0.02, -0.01, 0.015);
        let target_sigma_rn = Vector3::new(0.1, 0.2, -0.1);

        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg,
            hub_center_of_mass_body_m: hub_com_m,
            inertia_kg_m2: hub_inertia,
            integration_step_nanos: STEP_NANOS,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps,
            integrator: None,
        });
        let mut mass_props = SimpleMassProps::new("simple_mass_props");
        let mut navigation = SimpleNavigation::new("simple_navigation");
        let mut reference = Inertial3D::new(Inertial3DConfig {
            name: "inertial_3d".to_string(),
            sigma_r0n: target_sigma_rn,
        });
        let mut tracking = AttTrackingError::new(AttTrackingErrorConfig {
            name: "att_tracking_error".to_string(),
            sigma_r0r: Vector3::zeros(),
        });
        let controller_config = MrpFeedbackConfig {
            name: "mrp_feedback".to_string(),
            k: 0.5,
            ki: -1.0,
            p: 2.5,
            integral_limit: 0.0,
            known_torque_body_nm: Vector3::zeros(),
            // `omega_BN_B × [I] omega_BN_B`: the command depends on [I].
            control_law_type: 1,
        };
        let mut controller = MrpFeedback::new(controller_config.clone());
        let mut torque = CommandedTorque {
            torque_in_msg: Input::default(),
        };
        controller
            .cmd_torque_out_msg
            .connect_to(&mut torque.torque_in_msg);
        spacecraft.add_dynamic_effector(torque);
        let mut command_probe = Input::default();
        controller.cmd_torque_out_msg.connect_to(&mut command_probe);

        let first_command: Vector3<f64>;
        {
            let mut simulation = Simulation::new(epoch(), false);
            crate::connect!(&simulation,
                &spacecraft.mass_props_out => &mut mass_props.sc_mass_props_in_msg,
                &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
                &navigation.attitude_out_msg => &mut tracking.att_nav_in_msg,
                &reference.att_ref_out_msg => &mut tracking.att_ref_in_msg,
                &tracking.att_guid_out_msg => &mut controller.guid_in_msg,
                &mass_props.vehicle_config_out_msg => &mut controller.veh_config_in_msg,
            );
            crate::schedule! { simulation,
                "spacecraft" => &mut spacecraft, period = STEP_NANOS, priority = 60;
                "simple_mass_props" => &mut mass_props, period = STEP_NANOS, priority = 50;
                "navigation" => &mut navigation, period = STEP_NANOS, priority = 40;
                "inertial_3d" => &mut reference, period = STEP_NANOS, priority = 30;
                "att_tracking_error" => &mut tracking, period = STEP_NANOS, priority = 20;
                "mrp_feedback" => &mut controller, period = STEP_NANOS, priority = 10;
            }
            simulation.run_for(0);
            first_command = command_probe.read().torque_request_body_nm;
            simulation.run_for(300_000_000_000);
        }

        // The spacecraft's inertia about B: hub inertia plus m [c̃][c̃]ᵀ.
        let com_tilde = crate::utilities::rigid_body_kinematics::tilde(hub_com_m);
        let inertia_b = hub_inertia + mass_kg * com_tilde * com_tilde.transpose();
        let vehicle = mass_props.vehicle_config_out_msg.read();
        assert!((vehicle.inertia_about_point_b_kg_m2 - inertia_b).amax() < 1.0e-12);
        assert!((vehicle.center_of_mass_body_m - hub_com_m).amax() < 1.0e-15);
        assert_eq!(vehicle.mass_kg, mass_kg);

        // First command, at t = 0 with σ_BN = 0: -(K σ_BR + P ω_BR - ω × [I] ω).
        let sigma_br = sub_mrp(Vector3::zeros(), target_sigma_rn);
        let omega = initial_omega_radps;
        let expected = -(controller_config.k * sigma_br + controller_config.p * omega
            - omega.cross(&(inertia_b * omega)));
        assert!(
            (first_command - expected).amax() < 1.0e-14,
            "{first_command} vs {expected}"
        );
        let without_inertia = -(controller_config.k * sigma_br + controller_config.p * omega);
        assert!(
            (first_command - without_inertia).amax() > 1.0e-4,
            "the gyroscopic term is visible in the command"
        );

        // The closed loop settles on the target.
        let state = spacecraft.state_out.read();
        assert!(sub_mrp(state.sigma_bn, target_sigma_rn).norm() < 1.0e-4);
        assert!(state.omega_radps.norm() < 1.0e-5);
    }
}
