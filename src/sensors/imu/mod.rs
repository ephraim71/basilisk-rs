use nalgebra::{UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal};

use crate::messages::{ImuMsg, Input, Output, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct ImuConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    pub body_to_sensor_quaternion: UnitQuaternion<f64>,
    pub rate_noise_std_radps: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct Imu {
    pub config: ImuConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub output_imu_msg: Output<ImuMsg>,
    rng: StdRng,
}

impl Module for Imu {
    fn init(&mut self) {
        self.output_imu_msg.write(ImuMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.read_input_message();

        let true_rates = self.compute_body_to_sensor_rate(&state);
        let sensed_rates = self.apply_rate_noise(true_rates);

        self.write_output_message(sensed_rates);
    }
}

impl Imu {
    pub fn new(config: ImuConfig) -> Self {
        Self {
            config,
            input_state_msg: Input::default(),
            output_imu_msg: Output::default(),
            rng: StdRng::seed_from_u64(0),
        }
    }

    fn read_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn compute_body_to_sensor_rate(&self, state: &SpacecraftStateMsg) -> Vector3<f64> {
        self.config
            .body_to_sensor_quaternion
            .transform_vector(&state.omega_radps)
    }

    fn apply_rate_noise(&mut self, rate_sensor_radps: Vector3<f64>) -> Vector3<f64> {
        Vector3::new(
            rate_sensor_radps.x + self.sample_gaussian_noise(self.config.rate_noise_std_radps.x),
            rate_sensor_radps.y + self.sample_gaussian_noise(self.config.rate_noise_std_radps.y),
            rate_sensor_radps.z + self.sample_gaussian_noise(self.config.rate_noise_std_radps.z),
        )
    }

    fn write_output_message(&mut self, angular_rate_sensor_radps: Vector3<f64>) {
        self.output_imu_msg.write(ImuMsg {
            angular_rate_sensor_radps,
        });
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
