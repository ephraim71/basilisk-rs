//! basilisk-rs: a Rust port of Basilisk.
//!
//! The current crate layout is an architecture sketch:
//! - one folder per module family
//! - a separate `messages` module for message payloads
//! - type-safe telemetry recorders that can combine arbitrary message types
//! - explicit wiring in Rust code

use hifitime::Epoch;

pub mod clock_sync;
pub mod device_interface;
pub mod dynamics;
pub mod environment;
pub mod fsw_algorithms;
pub mod gauss_markov;
pub mod integrators;
mod kinematics;
pub mod messages;
pub mod power;
pub mod sensors;
pub mod simulation;
pub mod spacecraft;
pub mod telemetry;
mod time;

#[cfg(test)]
mod test_utils;

#[derive(Clone, Debug)]
pub struct SimulationContext {
    pub current_sim_nanos: u64,
    pub current_epoch: Epoch,
}

pub trait Module: Send {
    fn init(&mut self);
    /// Reset retained runtime state without repeating one-time output initialization.
    fn reset(&mut self, _context: &SimulationContext) {
        // Existing modules historically used `init` for both lifecycle events.
        // They retain that behavior unless they provide a distinct reset hook.
        self.init();
    }
    fn update(&mut self, context: &SimulationContext);
}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::dynamics::gravity::{GravBodyData, SphericalHarmonicsGravityModel};
    use crate::dynamics::reaction_wheel_state_effector::{
        ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
    };
    use crate::messages::MotorTorqueMsg;
    use crate::sensors::imu_sensor::{ImuSensor, ImuSensorConfig};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};

    #[test]
    fn example_architecture_wiring_is_reasonable() {
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 12.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::new(0.12, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.18),
            integration_step_nanos: 5_000_000,
            initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::new(0.01, 0.02, 0.03),
            integrator: None,
        });
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    "earth",
                    3.986_004_418e14,
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid Earth gravity body"),
            )
            .expect("unique central gravity body");

        let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
        reaction_wheels.add_reaction_wheel(ReactionWheelStateEffectorConfig::balanced(
            "rw_x",
            Vector3::new(0.1, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            0.02,
            0.4,
        ));
        let rw_command = crate::messages::Output::new(MotorTorqueMsg {
            motor_torque_nm: 0.001,
        });
        let mut imu = ImuSensor::new(ImuSensorConfig {
            name: "imu_1".to_string(),
            ..ImuSensorConfig::default()
        });

        let module_names = {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&spacecraft.state_out, &mut imu.input_state_msg);
            sim.connect(
                &rw_command,
                &mut reaction_wheels.wheels_mut()[0].motor_torque_in_msg,
            );
            spacecraft.add_state_effector(reaction_wheels);
            sim.add_module("spacecraft", &mut spacecraft, 5_000_000, 10);
            sim.add_module("imu", &mut imu, 5_000_000, 0);
            sim.run_for(0);
            sim.module_names()
        };

        assert_eq!(
            imu.output_imu_msg.read().angular_rate_sensor_radps,
            Vector3::new(0.01, 0.02, 0.03)
        );
        assert_eq!(
            spacecraft.state_effectors.len() + spacecraft.dynamic_effectors.len(),
            1
        );
        assert_eq!(spacecraft.gravity.bodies().len(), 1);
        assert_eq!(
            module_names,
            vec!["spacecraft".to_string(), "imu".to_string()]
        );
    }

    #[test]
    fn spherical_harmonics_gravity_has_reasonable_surface_magnitude() {
        let mut model = SphericalHarmonicsGravityModel::from_file("assets/gravity/GGM03S.txt", 20)
            .expect("valid gravity coefficient file");
        let gravity = model
            .acceleration_at_degree_mps2(Vector3::new(0.0, 0.0, 6_378_136.3), 20, true)
            .expect("non-singular gravity position");
        let gravity_magnitude = gravity.norm();

        assert!(
            (gravity_magnitude - 9.8).abs() < 0.2,
            "expected Earth surface gravity near 9.8 m/s^2, got {gravity_magnitude}"
        );
    }
}
