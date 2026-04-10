use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::Vector3;
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

use crate::messages::{Input, Output, SpacecraftStateMsg, SunEphemerisMsg, SunSensorMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct SunSensorConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    pub body_to_sensor_quaternion: nalgebra::UnitQuaternion<f64>,
    pub alpha_beta_half_fov_rad: f64,
    pub alpha_noise_std_rad: f64,
    pub beta_noise_std_rad: f64,
}

#[derive(Clone, Debug)]
pub struct SunSensor {
    pub config: SunSensorConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub output_sun_sensor_msg: Output<SunSensorMsg>,
    rng: StdRng,
}

impl Module for SunSensor {
    fn init(&mut self) {
        self.output_sun_sensor_msg.write(SunSensorMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.read_state_input_message();
        let sun = self.read_sun_input_message();

        let measurement = self.compute_measurement(&state, &sun);
        self.write_output_message(measurement);
    }
}

impl SunSensor {
    pub fn new(config: SunSensorConfig) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            config,
            input_state_msg: Input::default(),
            input_sun_msg: Input::default(),
            output_sun_sensor_msg: Output::default(),
        }
    }

    fn read_state_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn read_sun_input_message(&self) -> SunEphemerisMsg {
        self.input_sun_msg.read()
    }

    fn compute_measurement(
        &mut self,
        state: &SpacecraftStateMsg,
        sun: &SunEphemerisMsg,
    ) -> SunSensorMsg {
        let spacecraft_to_sun_inertial_m = sun.sun_position_inertial_m - state.position_m;
        if spacecraft_to_sun_inertial_m.norm_squared() == 0.0 {
            return SunSensorMsg::default();
        }

        let sun_direction_inertial = spacecraft_to_sun_inertial_m.normalize();
        let sun_direction_body = state
            .attitude_b_to_i
            .inverse()
            .transform_vector(&sun_direction_inertial);
        let sun_direction_sensor = self
            .config
            .body_to_sensor_quaternion
            .transform_vector(&sun_direction_body);

        if sun_direction_sensor.z <= 0.0 {
            return SunSensorMsg::default();
        }

        let alpha_rad = sun_direction_sensor.x.atan2(sun_direction_sensor.z);
        let beta_rad = sun_direction_sensor.y.atan2(sun_direction_sensor.z);
        let valid = alpha_rad.abs() <= self.config.alpha_beta_half_fov_rad
            && beta_rad.abs() <= self.config.alpha_beta_half_fov_rad;

        if !valid {
            return SunSensorMsg::default();
        }

        SunSensorMsg {
            alpha_rad: alpha_rad + self.sample_gaussian_noise(self.config.alpha_noise_std_rad),
            beta_rad: beta_rad + self.sample_gaussian_noise(self.config.beta_noise_std_rad),
            valid: true,
        }
    }

    fn write_output_message(&mut self, measurement: SunSensorMsg) {
        self.output_sun_sensor_msg.write(measurement);
    }

    fn sample_gaussian_noise(&mut self, std_dev: f64) -> f64 {
        if std_dev == 0.0 {
            return 0.0;
        }

        Normal::new(0.0, std_dev)
            .expect("std_dev must be positive")
            .sample(&mut self.rng)
    }
}

fn seed_from_name(name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}
