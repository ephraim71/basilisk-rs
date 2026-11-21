//! External force and torque: port of upstream Basilisk
//! `simulation/dynamics/extForceTorque`.
//!
//! Applies a constant force and torque set in the configuration, plus the
//! loads commanded on three optional inputs: a force in inertial components, a
//! force in body components and a torque about the body frame origin `B` in
//! body components. Every configured vector and every connected command adds
//! in. As upstream notes, the inertial and the body force are separate vectors,
//! and both apply when both are set.
//!
//! An unconnected command contributes nothing, as upstream skips an unlinked
//! message. Upstream samples the commands in its own `UpdateState` and holds
//! them through the integration; here they are read at each evaluation, which
//! sees the same values, since no module runs while the spacecraft integrates.

use std::any::Any;

use nalgebra::Vector3;

use crate::messages::{
    BodyForceCommandMsg, BodyTorqueCommandMsg, InertialForceCommandMsg, Input, SpacecraftStateMsg,
};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

/// User-settable parameters of [`ExtForceTorque`]: the constant loads.
#[derive(Clone, Debug, Default)]
pub struct ExtForceTorqueConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `extForce_N` \[N\]: constant force, inertial components.
    pub ext_force_inertial_n: Vector3<f64>,
    /// Upstream `extForce_B` \[N\]: constant force, body components.
    pub ext_force_body_n: Vector3<f64>,
    /// Upstream `extTorquePntB_B` \[N m\]: constant torque about `B`, body
    /// components.
    pub ext_torque_body_nm: Vector3<f64>,
}

/// The loads of [`ExtForceTorque`] as upstream reports them, before the
/// body force is rotated into the inertial frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ExternalLoads {
    /// Upstream `forceExternal_N` \[N\]: force, inertial components.
    pub force_inertial_n: Vector3<f64>,
    /// Upstream `forceExternal_B` \[N\]: force, body components.
    pub force_body_n: Vector3<f64>,
    /// Upstream `torqueExternalPntB_B` \[N m\]: torque about `B`, body
    /// components.
    pub torque_body_nm: Vector3<f64>,
}

/// Constant and commanded external loads: upstream `ExtForceTorque`.
#[derive(Clone, Debug)]
pub struct ExtForceTorque {
    /// User-settable parameters.
    pub config: ExtForceTorqueConfig,
    /// Upstream `cmdTorqueInMsg`: commanded torque about `B`.
    pub cmd_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `cmdForceBodyInMsg`: commanded force, body components.
    pub cmd_force_body_in_msg: Input<BodyForceCommandMsg>,
    /// Upstream `cmdForceInertialInMsg`: commanded force, inertial components.
    pub cmd_force_inertial_in_msg: Input<InertialForceCommandMsg>,
}

impl ExtForceTorque {
    /// Builds the effector with its command inputs unconnected.
    pub fn new(config: ExtForceTorqueConfig) -> Self {
        Self {
            config,
            cmd_torque_in_msg: Input::default(),
            cmd_force_body_in_msg: Input::default(),
            cmd_force_inertial_in_msg: Input::default(),
        }
    }

    /// Upstream `computeForceTorque`: the configured loads plus every
    /// connected command.
    pub fn loads(&self) -> ExternalLoads {
        let mut loads = ExternalLoads {
            force_inertial_n: self.config.ext_force_inertial_n,
            force_body_n: self.config.ext_force_body_n,
            torque_body_nm: self.config.ext_torque_body_nm,
        };
        if self.cmd_force_inertial_in_msg.is_connected() {
            loads.force_inertial_n += self
                .cmd_force_inertial_in_msg
                .read()
                .force_request_inertial_n;
        }
        if self.cmd_force_body_in_msg.is_connected() {
            loads.force_body_n += self.cmd_force_body_in_msg.read().force_request_body_n;
        }
        if self.cmd_torque_in_msg.is_connected() {
            loads.torque_body_nm += self.cmd_torque_in_msg.read().torque_request_body_nm;
        }
        loads
    }
}

impl DynamicEffector for ExtForceTorque {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let loads = self.loads();
        EffectorOutput {
            force_inertial_n: loads.force_inertial_n
                + state
                    .body_to_inertial()
                    .transform_vector(&loads.force_body_n),
            torque_body_nm: loads.torque_body_nm,
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `extForceTorque/_UnitTest/` with its tolerances:
    //! `test_extForceTorque.py` (every combination of configured and commanded
    //! loads), `test_extForceTorqueIntegrated.py` (final hub states after ten
    //! minutes of flight) and the parts of `test_extForceTorqueInitialization.py`
    //! that apply here: configured loads alone until a command is connected and
    //! written, then their sum. Its sampling cases (an unscheduled effector
    //! never sampling, held samples cleared by `Reset`) have no counterpart,
    //! since commands are read at each evaluation; nor does its attachment to a
    //! one-DOF spinning-body appendage, which does not host dynamic effectors
    //! here (the hinged panels and the two- and N-DOF spinning bodies do).

    use hifitime::Epoch;
    use nalgebra::Matrix3;

    use super::*;
    use crate::dynamics::gravity::GravBodyData;
    use crate::messages::Output;
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};

    /// Upstream `unitDynamicsModesTestFunction`: each input is 0 (unset), 1
    /// (configured), 2 (commanded) or 3 (both).
    fn modes(torque_input: u8, force_n_input: u8, force_b_input: u8) -> ExternalLoads {
        let torque = Vector3::new(-1.0, 1.0, -1.0);
        let force_n = Vector3::new(-10.0, -5.0, 5.0);
        let force_b = Vector3::new(10.0, 20.0, 30.0);
        let mut config = ExtForceTorqueConfig {
            name: "externalDisturbance".to_string(),
            ..Default::default()
        };
        if torque_input & 1 == 1 {
            config.ext_torque_body_nm = torque;
        }
        if force_n_input & 1 == 1 {
            config.ext_force_inertial_n = force_n;
        }
        if force_b_input & 1 == 1 {
            config.ext_force_body_n = force_b;
        }
        let mut effector = ExtForceTorque::new(config);
        let torque_cmd = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: torque,
            ..Default::default()
        });
        let force_n_cmd = Output::new(InertialForceCommandMsg {
            force_request_inertial_n: force_n,
        });
        let force_b_cmd = Output::new(BodyForceCommandMsg {
            force_request_body_n: force_b,
        });
        if torque_input & 2 == 2 {
            torque_cmd.connect_to(&mut effector.cmd_torque_in_msg);
        }
        if force_n_input & 2 == 2 {
            force_n_cmd.connect_to(&mut effector.cmd_force_inertial_in_msg);
        }
        if force_b_input & 2 == 2 {
            force_b_cmd.connect_to(&mut effector.cmd_force_body_in_msg);
        }
        effector.loads()
    }

    /// Upstream `test_unitDynamicsModes`.
    #[test]
    fn sums_configured_and_commanded_loads_in_every_combination() {
        let accuracy = 1e-12;
        for torque_input in 0..4 {
            for force_n_input in 0..4 {
                for force_b_input in 0..4 {
                    let loads = modes(torque_input, force_n_input, force_b_input);
                    let count = |input: u8| f64::from(input.count_ones());
                    let expected = ExternalLoads {
                        torque_body_nm: count(torque_input) * Vector3::new(-1.0, 1.0, -1.0),
                        force_inertial_n: count(force_n_input) * Vector3::new(-10.0, -5.0, 5.0),
                        force_body_n: count(force_b_input) * Vector3::new(10.0, 20.0, 30.0),
                    };
                    for (actual, truth) in [
                        (loads.torque_body_nm, expected.torque_body_nm),
                        (loads.force_inertial_n, expected.force_inertial_n),
                        (loads.force_body_n, expected.force_body_n),
                    ] {
                        assert!(
                            (actual - truth).abs().max() <= accuracy,
                            "case ({torque_input}, {force_n_input}, {force_b_input}): \
                             {actual:?} vs {truth:?}"
                        );
                    }
                }
            }
        }
    }

    /// Upstream `test_command_defaults_and_scheduling` and
    /// `test_cached_commands_and_repeated_reset`, as they apply here: a
    /// connected but unwritten command reads zero, and a newly written one is
    /// seen at the next evaluation.
    #[test]
    fn applies_configured_loads_until_commands_are_written() {
        let static_force_n = Vector3::new(0.3, -0.2, 0.1);
        let static_force_b = Vector3::new(0.1, 0.2, -0.1);
        let static_torque_b = Vector3::new(0.01, -0.02, 0.03);
        let command_force_n = Vector3::new(1.0, -2.0, 3.0);
        let command_force_b = Vector3::new(-4.0, 5.0, -6.0);
        let command_torque_b = Vector3::new(0.2, -0.3, 0.4);
        let mut effector = ExtForceTorque::new(ExtForceTorqueConfig {
            name: "ext".to_string(),
            ext_force_inertial_n: static_force_n,
            ext_force_body_n: static_force_b,
            ext_torque_body_nm: static_torque_b,
        });
        let force_n = Output::<InertialForceCommandMsg>::default();
        let force_b = Output::<BodyForceCommandMsg>::default();
        let torque_b = Output::<BodyTorqueCommandMsg>::default();
        force_n.connect_to(&mut effector.cmd_force_inertial_in_msg);
        force_b.connect_to(&mut effector.cmd_force_body_in_msg);
        torque_b.connect_to(&mut effector.cmd_torque_in_msg);
        let check = |scale: f64| {
            let loads = effector.loads();
            for (actual, configured, commanded) in [
                (loads.force_inertial_n, static_force_n, command_force_n),
                (loads.force_body_n, static_force_b, command_force_b),
                (loads.torque_body_nm, static_torque_b, command_torque_b),
            ] {
                let expected = configured + scale * commanded;
                assert!(
                    (actual - expected).abs().max() <= 1e-15,
                    "{actual:?} vs {expected:?}"
                );
            }
        };

        check(0.0);
        for scale in [1.0, 2.0] {
            force_n.write(InertialForceCommandMsg {
                force_request_inertial_n: scale * command_force_n,
            });
            force_b.write(BodyForceCommandMsg {
                force_request_body_n: scale * command_force_b,
            });
            torque_b.write(BodyTorqueCommandMsg {
                torque_request_body_nm: scale * command_torque_b,
                ..Default::default()
            });
            check(scale);
            check(scale);
        }
    }

    /// The body force is rotated into the inertial frame and added to the
    /// inertial one; the torque passes through.
    #[test]
    fn rotates_the_body_force_into_the_inertial_frame() {
        let effector = ExtForceTorque::new(ExtForceTorqueConfig {
            name: "ext".to_string(),
            ext_force_inertial_n: Vector3::new(1.0, 0.0, 0.0),
            ext_force_body_n: Vector3::new(0.0, 2.0, 0.0),
            ext_torque_body_nm: Vector3::new(0.0, 0.0, 3.0),
        });
        // 90 degrees about +z: body y is inertial -x.
        let state = SpacecraftStateMsg {
            sigma_bn: Vector3::new(0.0, 0.0, (std::f64::consts::FRAC_PI_4 / 2.0).tan()),
            ..Default::default()
        };
        let output = effector.compute_output(&state);
        assert!((output.force_inertial_n - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-15);
        assert_eq!(output.torque_body_nm, Vector3::new(0.0, 0.0, 3.0));
    }

    /// Upstream `extForceBodyAndTorque` / `extForceInertialAndTorque`: the hub
    /// position and attitude after ten minutes under the loads.
    fn integrated_final_state(config: ExtForceTorqueConfig) -> SpacecraftStateMsg {
        let step_nanos = 100_000_000;
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 750.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            integration_step_nanos: step_nanos,
            initial_position_m: Vector3::new(
                -4020338.690396649,
                7490566.741852513,
                5248299.211589362,
            ),
            initial_velocity_mps: Vector3::new(
                -5199.77710904224,
                -3436.681645356935,
                1041.576797498721,
            ),
            initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "earth_planet_data",
                    0.3986004415E+15,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .unwrap(),
            )
            .unwrap();
        spacecraft.add_dynamic_effector(ExtForceTorque::new(config));
        let state_out = spacecraft.state_out.clone();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("spacecraftBody", &mut spacecraft, step_nanos, 0);
            sim.run_for(600_000_000_000);
        }
        state_out.read()
    }

    /// `unitTestSupport.isArrayEqualRelative`.
    #[track_caller]
    fn assert_relative(actual: Vector3<f64>, truth: Vector3<f64>, accuracy: f64) {
        for (a, t) in actual.iter().zip(truth.iter()) {
            assert!(((a - t) / t).abs() <= accuracy, "{actual:?} vs {truth:?}");
        }
    }

    #[test]
    fn integrated_body_force_and_torque() {
        let state = integrated_final_state(ExtForceTorqueConfig {
            name: "externalDisturbance".to_string(),
            ext_torque_body_nm: Vector3::new(-1.0, 1.0, -1.0),
            ext_force_body_n: Vector3::new(1.0, 2.0, 3.0),
            ..Default::default()
        });
        let accuracy = 1e-8;
        assert_relative(
            state.position_m,
            Vector3::new(-6.78136423e+06, 4.94628599e+06, 5.48655395e+06),
            accuracy,
        );
        assert_relative(
            state.sigma_bn,
            Vector3::new(4.91025978e-01, -4.21586707e-01, 3.61459503e-01),
            accuracy,
        );
    }

    #[test]
    fn integrated_inertial_force_and_torque() {
        let state = integrated_final_state(ExtForceTorqueConfig {
            name: "externalDisturbance".to_string(),
            ext_torque_body_nm: Vector3::new(-1.0, 1.0, -1.0),
            ext_force_inertial_n: Vector3::new(-1.0, -0.5, 0.5),
            ..Default::default()
        });
        let accuracy = 1e-8;
        assert_relative(
            state.position_m,
            Vector3::new(-6.78183900e+06, 4.94674963e+06, 5.48686274e+06),
            accuracy,
        );
        assert_relative(
            state.sigma_bn,
            Vector3::new(4.91025978e-01, -4.21586707e-01, 3.61459503e-01),
            accuracy,
        );
    }
}
