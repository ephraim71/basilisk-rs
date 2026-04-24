use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};

use crate::messages::{Input, Output, SpacecraftStateMsg, StarTrackerMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct StarTrackerConfig {
    pub name: String,
    pub body_to_sensor_quaternion: UnitQuaternion<f64>,
    pub p_matrix_sqrt_rad: Matrix3<f64>,
    pub a_matrix: Matrix3<f64>,
    pub walk_bounds_rad: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct StarTracker {
    pub config: StarTrackerConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub output_star_tracker_msg: Output<StarTrackerMsg>,
    error_state_prv_rad: Vector3<f64>,
    rng: StdRng,
}

impl Module for StarTracker {
    fn init(&mut self) {
        self.output_star_tracker_msg
            .write(StarTrackerMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.read_input_message();
        let true_attitude = self.compute_true_output(&state);
        let sensed_attitude = self.apply_sensor_errors(true_attitude);

        self.write_output_message(sensed_attitude);
    }
}

impl StarTracker {
    pub fn new(config: StarTrackerConfig) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            config,
            input_state_msg: Input::default(),
            output_star_tracker_msg: Output::default(),
            error_state_prv_rad: Vector3::zeros(),
        }
    }

    fn read_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn compute_true_output(&self, state: &SpacecraftStateMsg) -> UnitQuaternion<f64> {
        self.config.body_to_sensor_quaternion * state.inertial_to_body()
    }

    fn apply_sensor_errors(&mut self, true_attitude: UnitQuaternion<f64>) -> UnitQuaternion<f64> {
        let random_vector = Vector3::new(
            StandardNormal.sample(&mut self.rng),
            StandardNormal.sample(&mut self.rng),
            StandardNormal.sample(&mut self.rng),
        );
        self.error_state_prv_rad = self.config.a_matrix * self.error_state_prv_rad
            + self.config.p_matrix_sqrt_rad * random_vector;
        self.error_state_prv_rad =
            self.error_state_prv_rad
                .zip_map(&self.config.walk_bounds_rad, |error, bound| {
                    if bound > 0.0 {
                        error.clamp(-bound, bound)
                    } else {
                        error
                    }
                });

        UnitQuaternion::from_scaled_axis(self.error_state_prv_rad) * true_attitude
    }

    fn write_output_message(&mut self, attitude_inertial_to_sensor: UnitQuaternion<f64>) {
        self.output_star_tracker_msg.write(StarTrackerMsg {
            attitude_inertial_to_sensor,
        });
    }
}

fn seed_from_name(name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}
