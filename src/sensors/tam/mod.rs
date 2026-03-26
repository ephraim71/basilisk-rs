use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::{Matrix3, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};

use crate::messages::{Input, MagneticFieldMsg, Output, SpacecraftStateMsg, TamMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct TamConfig {
    pub name: String,
    pub body_to_sensor_quaternion: nalgebra::UnitQuaternion<f64>,
    pub bias_t: Vector3<f64>,
    pub p_matrix_sqrt_t: Matrix3<f64>,
    pub a_matrix: Matrix3<f64>,
    pub walk_bounds_t: Vector3<f64>,
    pub scale_factor: f64,
    pub min_output_t: f64,
    pub max_output_t: f64,
}

#[derive(Clone, Debug)]
pub struct Tam {
    pub config: TamConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_magnetic_field_msg: Input<MagneticFieldMsg>,
    pub output_tam_msg: Output<TamMsg>,
    error_state_t: Vector3<f64>,
    rng: StdRng,
}

impl Module for Tam {
    fn init(&mut self) {
        self.output_tam_msg.write(TamMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.read_state_input_message();
        let magnetic_field = self.read_magnetic_field_input_message();

        let true_field_sensor_t = self.compute_true_output(&state, &magnetic_field);
        let sensed_field_sensor_t = self.apply_sensor_errors(true_field_sensor_t);

        self.write_output_message(sensed_field_sensor_t);
    }
}

impl Tam {
    pub fn new(config: TamConfig) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            config,
            input_state_msg: Input::default(),
            input_magnetic_field_msg: Input::default(),
            output_tam_msg: Output::default(),
            error_state_t: Vector3::zeros(),
        }
    }

    fn read_state_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn read_magnetic_field_input_message(&self) -> MagneticFieldMsg {
        self.input_magnetic_field_msg.read()
    }

    fn compute_true_output(
        &self,
        state: &SpacecraftStateMsg,
        magnetic_field: &MagneticFieldMsg,
    ) -> Vector3<f64> {
        let magnetic_field_body_t = state.attitude_b_to_i
            .inverse()
            .transform_vector(&magnetic_field.magnetic_field_inertial_t);

        self.config
            .body_to_sensor_quaternion
            .transform_vector(&magnetic_field_body_t)
    }

    fn apply_sensor_errors(&mut self, true_field_sensor_t: Vector3<f64>) -> Vector3<f64> {
        let random_vector = Vector3::new(
            StandardNormal.sample(&mut self.rng),
            StandardNormal.sample(&mut self.rng),
            StandardNormal.sample(&mut self.rng),
        );
        self.error_state_t =
            self.config.a_matrix * self.error_state_t + self.config.p_matrix_sqrt_t * random_vector;
        self.error_state_t =
            self.error_state_t
                .zip_map(&self.config.walk_bounds_t, |error, bound| {
                    if bound > 0.0 {
                        error.clamp(-bound, bound)
                    } else {
                        error
                    }
                });

        let sensed_field_sensor_t = (true_field_sensor_t + self.error_state_t + self.config.bias_t)
            * self.config.scale_factor;

        sensed_field_sensor_t
            .map(|value| value.clamp(self.config.min_output_t, self.config.max_output_t))
    }

    fn write_output_message(&mut self, magnetic_field_sensor_t: Vector3<f64>) {
        self.output_tam_msg.write(TamMsg {
            magnetic_field_sensor_t,
        });
    }
}

fn seed_from_name(name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}
