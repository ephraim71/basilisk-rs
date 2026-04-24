//! basilisk-rs: a Rust port of Basilisk.
//!
//! The current crate layout is an architecture sketch:
//! - one folder per module family
//! - a separate `messages` module for message payloads
//! - a generic telemetry recorder for any message type
//! - explicit wiring in Rust code

use hifitime::Epoch;

pub mod actuators;
pub mod environment;
pub mod fsw;
pub mod messages;
pub mod sensors;
pub mod simulation;
pub mod spacecraft;
pub mod telemetry;

#[path = "environment/atmosphere/mod.rs"]
pub mod atmosphere;
#[path = "actuators/drag/mod.rs"]
pub mod drag;
#[path = "environment/eclipse/mod.rs"]
pub mod eclipse;
#[path = "environment/ephemeris/mod.rs"]
pub mod ephemeris;
#[path = "sensors/gps/mod.rs"]
pub mod gps;
#[path = "environment/gravity/mod.rs"]
pub mod gravity;
#[path = "sensors/imu/mod.rs"]
pub mod imu;
#[path = "environment/magnetic_field/mod.rs"]
pub mod magnetic_field;
#[path = "actuators/mtb/mod.rs"]
pub mod mtb;
#[path = "environment/nrlmsise.rs"]
pub mod nrlmsise;
#[path = "actuators/reaction_wheel/mod.rs"]
pub mod reaction_wheel;
#[path = "environment/srp/mod.rs"]
pub mod srp;
#[path = "sensors/star_tracker/mod.rs"]
pub mod star_tracker;
#[path = "environment/sun_ephemeris/mod.rs"]
pub mod sun_ephemeris;
#[path = "sensors/sun_sensor/mod.rs"]
pub mod sun_sensor;
#[path = "sensors/tam/mod.rs"]
pub mod tam;
#[path = "actuators/thruster/mod.rs"]
pub mod thruster;

#[derive(Clone, Debug)]
pub struct SimulationContext {
    pub current_sim_nanos: u64,
    pub current_epoch: Epoch,
}

pub trait Module: Send {
    fn init(&mut self);
    fn update(&mut self, context: &SimulationContext);
}

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    use crate::gravity::{GravBodyData, SphericalHarmonicsGravityModel};
    use crate::imu::{Imu, ImuConfig};
    use crate::messages::ReactionWheelCommandMsg;
    use crate::reaction_wheel::{ReactionWheel, ReactionWheelConfig};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};

    #[test]
    fn example_architecture_wiring_is_reasonable() {
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 12.0,
            inertia_kg_m2: Matrix3::new(0.12, 0.0, 0.0, 0.0, 0.15, 0.0, 0.0, 0.0, 0.18),
            integration_step_nanos: 5_000_000,
            initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
            initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
            initial_attitude_b_to_i: UnitQuaternion::identity(),
            initial_omega_radps: Vector3::new(0.01, 0.02, 0.03),
        });
        spacecraft.add_grav_body(GravBodyData::point_mass(
            "earth",
            3.986_004_418e14,
            true,
            Vector3::zeros(),
            Vector3::zeros(),
        ));

        let mut reaction_wheel = ReactionWheel::new(ReactionWheelConfig::balanced(
            "rw_x",
            Vector3::new(0.1, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            0.02,
            0.4,
        ));
        let rw_command = crate::messages::Output::new(ReactionWheelCommandMsg {
            motor_torque_nm: 0.001,
        });
        let mut imu = Imu::new(ImuConfig {
            name: "imu_1".to_string(),
            position_m: Vector3::new(0.0, 0.0, 0.0),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            rate_noise_std_radps: Vector3::new(0.0, 0.0, 0.0),
        });

        let module_names = {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&spacecraft.state_out, &mut imu.input_state_msg);
            sim.connect(&rw_command, &mut reaction_wheel.command_in);
            spacecraft.add_state_effector(reaction_wheel);
            sim.add_module("spacecraft", &mut spacecraft, 5_000_000, 0);
            sim.add_module("imu", &mut imu, 5_000_000, 10);
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
        assert_eq!(spacecraft.gravity.grav_bodies.len(), 1);
        assert_eq!(
            module_names,
            vec!["spacecraft".to_string(), "imu".to_string()]
        );
    }

    #[test]
    fn spherical_harmonics_gravity_has_reasonable_surface_magnitude() {
        let model = SphericalHarmonicsGravityModel::from_file("assets/gravity/GGM03S.txt", 20);
        let gravity = model.compute_field(Vector3::new(0.0, 0.0, 6_378_136.3), 20, true);
        let gravity_magnitude = gravity.norm();

        assert!(
            (gravity_magnitude - 9.8).abs() < 0.2,
            "expected Earth surface gravity near 9.8 m/s^2, got {gravity_magnitude}"
        );
    }
}
