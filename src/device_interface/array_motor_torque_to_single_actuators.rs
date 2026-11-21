//! Array motor torque to one command per actuator: port of upstream Basilisk
//! `simulation/deviceInterface/arrayMotorTorqueToSingleActuators`.
//!
//! Flight software such as
//! [`RwMotorTorque`](crate::fsw_algorithms::effector_interfaces::rw_motor_torque::RwMotorTorque)
//! publishes the motor torques of a whole actuator set in one
//! [`ArrayMotorTorqueMsg`]; actuator models read one command each. This module
//! copies entry `i` of the array to output `i`, every update, for the
//! configured number of actuators.
//!
//! Upstream writes a `SingleActuatorMsg` per actuator. Here each output is a
//! [`MotorTorqueMsg`], the per-wheel command
//! [`ReactionWheel::motor_torque_in_msg`](crate::dynamics::reaction_wheel_state_effector::ReactionWheel::motor_torque_in_msg)
//! reads, so the outputs drive the reaction wheels directly; the outputs are
//! made by [`ArrayMotorTorqueToSingleActuators::new`] from the configured
//! count, as for
//! [`MotorVoltageInterface`](super::motor_voltage_interface::MotorVoltageInterface).

use crate::messages::{ArrayMotorTorqueMsg, Input, MAX_EFF_COUNT, MotorTorqueMsg, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ArrayMotorTorqueToSingleActuators`].
#[derive(Clone, Debug)]
pub struct ArrayMotorTorqueToSingleActuatorsConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `numActuators` (`setNumActuators`): the number of actuator
    /// outputs, at most [`MAX_EFF_COUNT`]. Fixed at construction.
    pub num_actuators: usize,
}

/// Splits an [`ArrayMotorTorqueMsg`] into per-actuator commands: upstream
/// `ArrayMotorTorqueToSingleActuators`.
#[derive(Clone, Debug)]
pub struct ArrayMotorTorqueToSingleActuators {
    /// Module configuration.
    pub config: ArrayMotorTorqueToSingleActuatorsConfig,
    /// Upstream `torqueInMsg`: the motor torques of the actuator set.
    pub torque_in_msg: Input<ArrayMotorTorqueMsg>,
    /// Upstream `actuatorOutMsgs`: the motor torque \[N m\] of each actuator,
    /// in array order.
    pub actuator_out_msgs: Vec<Output<MotorTorqueMsg>>,
}

impl ArrayMotorTorqueToSingleActuators {
    /// Builds the module with `config.num_actuators` outputs.
    ///
    /// # Panics
    ///
    /// If `config.num_actuators` exceeds [`MAX_EFF_COUNT`], the length of the
    /// input array.
    pub fn new(config: ArrayMotorTorqueToSingleActuatorsConfig) -> Self {
        assert!(
            config.num_actuators <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} actuators are supported, got {}",
            config.num_actuators
        );
        let actuator_out_msgs = (0..config.num_actuators)
            .map(|_| Output::default())
            .collect();
        Self {
            config,
            torque_in_msg: Input::default(),
            actuator_out_msgs,
        }
    }

    /// Upstream `Reset`: zeroes every actuator command.
    fn zero_outputs(&self) {
        for output in &self.actuator_out_msgs {
            output.write(MotorTorqueMsg::default());
        }
    }
}

impl Module for ArrayMotorTorqueToSingleActuators {
    fn init(&mut self) {
        if !self.torque_in_msg.is_connected() {
            log::error!(
                "arrayMotorTorqueToSingleActuators '{}' torque_in_msg is not connected",
                self.config.name
            );
        }
        self.zero_outputs();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.zero_outputs();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let torques = self.torque_in_msg.read();
        for (output, torque_nm) in self.actuator_out_msgs.iter().zip(torques.motor_torque_nm) {
            output.write(MotorTorqueMsg {
                motor_torque_nm: torque_nm,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `arrayMotorTorqueToSingleActuators/_UnitTest/test_arrayMotorTorqueToSingleActuators.py`
    //! over its parameters, and adds an end-to-end run from the FSW
    //! `rwMotorTorque` through this module to the reaction wheels.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig};
    use crate::dynamics::reaction_wheel_state_effector::{
        ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
    };
    use crate::fsw_algorithms::effector_interfaces::rw_motor_torque::{
        RwMotorTorque, RwMotorTorqueConfig,
    };
    use crate::messages::{
        ArrayMotorTorqueMsg, BodyTorqueCommandMsg, MAX_EFF_COUNT, Output, ReactionWheelStateMsg,
        RwArrayConfigMsg,
    };
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::{Module, SimulationContext};

    const TASK_PERIOD_NANOS: u64 = 100_000_000;

    fn splitter(num_actuators: usize) -> ArrayMotorTorqueToSingleActuators {
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "arrayMotorTorqueToSingleActuators".to_string(),
            num_actuators,
        })
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `test_arrayMotorTorqueToSingleActuators` for 1, 3 and 8
    /// actuators: torque `0.05 (i + 1)` on actuator `i`, two task ticks.
    #[test]
    fn each_output_carries_its_array_entry() {
        for num_actuators in [1, 3, 8] {
            let torques: Vec<f64> = (0..num_actuators)
                .map(|index| (index + 1) as f64 * 0.05)
                .collect();
            let torque_out = Output::new(ArrayMotorTorqueMsg::from_active(&torques));
            let mut adapter = splitter(num_actuators);
            torque_out.connect_to(&mut adapter.torque_in_msg);

            adapter.init();
            for tick in 0..2 {
                adapter.update(&context(tick * TASK_PERIOD_NANOS));
            }

            assert_eq!(adapter.actuator_out_msgs.len(), num_actuators);
            for (output, truth) in adapter.actuator_out_msgs.iter().zip(&torques) {
                assert!((output.read().motor_torque_nm - truth).abs() <= 1e-14);
            }
        }
    }

    #[test]
    fn reset_zeroes_the_commands() {
        let torque_out = Output::new(ArrayMotorTorqueMsg::from_active(&[0.5, -0.25]));
        let mut adapter = splitter(2);
        torque_out.connect_to(&mut adapter.torque_in_msg);
        adapter.init();
        adapter.update(&context(0));
        assert_eq!(adapter.actuator_out_msgs[1].read().motor_torque_nm, -0.25);

        Module::reset(&mut adapter, &context(0));

        assert!(
            adapter
                .actuator_out_msgs
                .iter()
                .all(|output| output.read().motor_torque_nm == 0.0)
        );
    }

    #[test]
    #[should_panic(expected = "at most 36 actuators are supported")]
    fn rejects_more_actuators_than_the_array_holds() {
        let _ = splitter(MAX_EFF_COUNT + 1);
    }

    /// FSW `rwMotorTorque` maps a body torque request onto three wheels; this
    /// module splits its array command, and each wheel applies its own entry,
    /// turning the hub along the request.
    #[test]
    fn rw_motor_torque_drives_the_reaction_wheels_through_the_splitter() {
        let spin_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
        let wheel_configs: Vec<_> = spin_axes
            .iter()
            .enumerate()
            .map(|(index, axis)| {
                ReactionWheelStateEffectorConfig::balanced(
                    format!("rw{index}"),
                    Vector3::zeros(),
                    *axis,
                    0.2,
                    50.0,
                )
            })
            .collect();
        let mut rw_config = RwArrayConfigMsg {
            num_rw: wheel_configs.len(),
            ..Default::default()
        };
        for (index, wheel) in wheel_configs.iter().enumerate() {
            rw_config.spin_axes_body[index] = wheel.spin_axis_body;
            rw_config.spin_axis_inertias_kg_m2[index] = wheel.js_kg_m2;
            rw_config.max_motor_torques_nm[index] = wheel.max_torque_nm;
        }
        let rw_config_out = Output::new(rw_config);
        let torque_request_body_nm = Vector3::new(0.01, -0.02, 0.03);
        let torque_request_out = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm,
            ..Default::default()
        });

        let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: spin_axes.to_vec(),
        });
        torque_request_out.connect_to(&mut rw_motor_torque.veh_control_in_msg);
        rw_config_out.connect_to(&mut rw_motor_torque.rw_params_in_msg);
        let mut adapter = splitter(wheel_configs.len());
        rw_motor_torque
            .rw_motor_torque_out_msg
            .connect_to(&mut adapter.torque_in_msg);

        let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
        for config in wheel_configs {
            reaction_wheels.add_reaction_wheel(config);
        }
        for (output, wheel) in adapter
            .actuator_out_msgs
            .iter()
            .zip(reaction_wheels.wheels_mut())
        {
            output.connect_to(&mut wheel.motor_torque_in_msg);
        }
        let wheel_states: Vec<Output<ReactionWheelStateMsg>> = reaction_wheels
            .wheels()
            .iter()
            .map(|wheel| wheel.state_out.clone())
            .collect();

        let inertia_kg_m2 = Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0));
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 750.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2,
            integration_step_nanos: TASK_PERIOD_NANOS,
            initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        });
        spacecraft.add_state_effector(reaction_wheels);

        let duration_nanos = 10_000_000_000;
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.add_module("spacecraft", &mut spacecraft, TASK_PERIOD_NANOS, 100);
            sim.add_module("rwMotorTorque", &mut rw_motor_torque, TASK_PERIOD_NANOS, 50);
            sim.add_module("splitter", &mut adapter, TASK_PERIOD_NANOS, 40);
            sim.run_for(duration_nanos);
        }

        // Orthonormal wheels on the control axes: each wheel takes the
        // opposite of the request about its own axis.
        let commands = rw_motor_torque.rw_motor_torque_out_msg.read();
        for (index, axis) in spin_axes.iter().enumerate() {
            let expected_nm = -axis.dot(&torque_request_body_nm);
            assert!((commands.motor_torque_nm[index] - expected_nm).abs() < 1e-15);
            assert_eq!(
                adapter.actuator_out_msgs[index].read().motor_torque_nm,
                commands.motor_torque_nm[index]
            );
            assert_eq!(
                wheel_states[index].read().applied_motor_torque_nm,
                commands.motor_torque_nm[index]
            );
        }

        // The reaction on the hub is the request: the hub spins up along it
        // (the wheels' spin inertia only shades the rate).
        let omega_radps = spacecraft.state_out.read().omega_radps;
        let elapsed_s = (duration_nanos - TASK_PERIOD_NANOS) as f64 * 1e-9;
        let expected_radps =
            inertia_kg_m2.try_inverse().unwrap() * torque_request_body_nm * elapsed_s;
        for axis in 0..3 {
            let ratio = omega_radps[axis] / expected_radps[axis];
            assert!(
                (ratio - 1.0).abs() < 0.05,
                "axis {axis}: hub rate {omega_radps:?} vs {expected_radps:?}"
            );
        }
    }
}
