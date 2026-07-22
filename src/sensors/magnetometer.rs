use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::{Matrix3, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};

use crate::messages::{Input, MagneticFieldMsg, Output, SpacecraftStateMsg, TamSensorMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct MagnetometerConfig {
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

impl MagnetometerConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("scale_factor", self.scale_factor),
            ("min_output_t", self.min_output_t),
            ("max_output_t", self.max_output_t),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite, got {value}"));
            }
        }
        for (label, vector) in [
            ("bias_t", self.bias_t),
            ("walk_bounds_t", self.walk_bounds_t),
        ] {
            if !vector.iter().all(|v| v.is_finite()) {
                return Err(format!("{label} must be finite, got {vector:?}"));
            }
        }
        if !self.p_matrix_sqrt_t.iter().all(|v| v.is_finite()) {
            return Err(format!(
                "p_matrix_sqrt_t must be finite, got {:?}",
                self.p_matrix_sqrt_t
            ));
        }
        if !self.a_matrix.iter().all(|v| v.is_finite()) {
            return Err(format!("a_matrix must be finite, got {:?}", self.a_matrix));
        }
        if self.min_output_t > self.max_output_t {
            return Err(format!(
                "min_output_t ({}) must not exceed max_output_t ({})",
                self.min_output_t, self.max_output_t
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct Magnetometer {
    pub config: MagnetometerConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_magnetic_field_msg: Input<MagneticFieldMsg>,
    pub output_tam_msg: Output<TamSensorMsg>,
    error_state_t: Vector3<f64>,
    rng: StdRng,
}

impl Module for Magnetometer {
    fn init(&mut self) {
        // On reset, clear the accumulated Gauss-Markov walk state.
        self.error_state_t = Vector3::zeros();
        self.output_tam_msg.write(TamSensorMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.read_state_input_message();
        let magnetic_field = self.read_magnetic_field_input_message();

        let true_field_sensor_t = self.compute_true_output(&state, &magnetic_field);
        let sensed_field_sensor_t = self.apply_sensor_errors(true_field_sensor_t);

        self.write_output_message(sensed_field_sensor_t);
    }
}

impl Magnetometer {
    pub fn new(config: MagnetometerConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid MagnetometerConfig: {msg}");
        }
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
        let magnetic_field_body_t = state
            .inertial_to_body()
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
        self.output_tam_msg.write(TamSensorMsg {
            magnetic_field_sensor_t,
        });
    }
}

fn seed_from_name(name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    use crate::messages::{MagneticFieldMsg, Output, SpacecraftStateMsg};
    use crate::{Module, SimulationContext};

    use super::{Magnetometer, MagnetometerConfig};

    fn dummy_context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// A noiseless config: zero process-noise sqrt and zero walk, so the error
    /// state stays at zero and the output is deterministic.
    fn noiseless_config(name: &str) -> MagnetometerConfig {
        MagnetometerConfig {
            name: name.to_string(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            bias_t: Vector3::zeros(),
            p_matrix_sqrt_t: Matrix3::zeros(),
            a_matrix: Matrix3::identity(),
            walk_bounds_t: Vector3::zeros(),
            scale_factor: 1.0,
            min_output_t: -1.0e-4,
            max_output_t: 1.0e-4,
        }
    }

    fn run(
        mut tam: Magnetometer,
        sigma_bn: Vector3<f64>,
        field_inertial_t: Vector3<f64>,
    ) -> Vector3<f64> {
        let state_out = Output::new(SpacecraftStateMsg {
            sigma_bn,
            ..Default::default()
        });
        let field_out = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t: field_inertial_t,
        });
        tam.input_state_msg.connect(state_out.slot());
        tam.input_magnetic_field_msg.connect(field_out.slot());
        tam.init();
        tam.update(&dummy_context());
        tam.output_tam_msg.read().magnetic_field_sensor_t
    }

    #[test]
    fn transform_bias_and_scale_no_noise() {
        let mut config = noiseless_config("tam");
        config.bias_t = Vector3::new(1.0e-6, 1.0e-6, 1.0e-5);
        config.scale_factor = 2.0;

        let field = Vector3::new(1.0e-5, 2.0e-5, 1.5e-5);
        // Identity attitude and identity sensor frame: sensed = (field + bias) * scale.
        let sensed = run(Magnetometer::new(config), Vector3::zeros(), field);
        let expected = (field + Vector3::new(1.0e-6, 1.0e-6, 1.0e-5)) * 2.0;
        assert!(
            (sensed - expected).norm() < 1e-18,
            "got {sensed:?}, want {expected:?}"
        );
    }

    #[test]
    fn attitude_and_sensor_rotation() {
        let mut config = noiseless_config("tam");
        config.body_to_sensor_quaternion = UnitQuaternion::from_euler_angles(0.1, 1.0, 0.7854);
        let field = Vector3::new(1.0e-5, 2.0e-5, 1.5e-5);
        let sigma_bn = Vector3::new(0.3, 0.2, 0.1);

        let sensed = run(Magnetometer::new(config.clone()), sigma_bn, field);

        // Rotate field inertial -> body -> sensor frame.
        let state = SpacecraftStateMsg {
            sigma_bn,
            ..Default::default()
        };
        let field_body = state.inertial_to_body().transform_vector(&field);
        let expected = config
            .body_to_sensor_quaternion
            .transform_vector(&field_body);
        assert!(
            (sensed - expected).norm() < 1e-18,
            "got {sensed:?}, want {expected:?}"
        );
    }

    #[test]
    fn saturation_clamps_each_axis() {
        let mut config = noiseless_config("tam");
        config.scale_factor = 2.0;
        config.min_output_t = -1.0e-5;
        config.max_output_t = 1.0e-5;

        let field = Vector3::new(1.0e-5, 2.0e-5, 1.5e-5);
        // (field)*2 exceeds max on every axis -> clamps to max.
        let sensed = run(Magnetometer::new(config), Vector3::zeros(), field);
        assert!(
            (sensed - Vector3::repeat(1.0e-5)).norm() < 1e-18,
            "got {sensed:?}"
        );
    }

    #[test]
    fn iid_noise_matches_configured_std() {
        let mut config = noiseless_config("tam_noise");
        let std = 3.0e-9;
        config.p_matrix_sqrt_t = Matrix3::from_diagonal(&Vector3::repeat(std));
        config.a_matrix = Matrix3::zeros(); // IID: no propagation of prior error
        config.min_output_t = -1.0;
        config.max_output_t = 1.0;

        let field = Vector3::new(1.0e-5, 2.0e-5, 1.5e-5);
        let state_out = Output::new(SpacecraftStateMsg::default());
        let field_out = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t: field,
        });
        let mut tam = Magnetometer::new(config);
        tam.input_state_msg.connect(state_out.slot());
        tam.input_magnetic_field_msg.connect(field_out.slot());
        tam.init();

        let n = 20_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            tam.update(&dummy_context());
            let noise = tam.output_tam_msg.read().magnetic_field_sensor_t.x - field.x;
            sum += noise;
            sum_sq += noise * noise;
        }
        let mean = sum / n as f64;
        let sample_std = (sum_sq / n as f64 - mean * mean).sqrt();
        assert!(
            (sample_std - std).abs() < 0.05 * std,
            "std {sample_std} vs {std}"
        );
    }

    #[test]
    #[should_panic(expected = "min_output_t")]
    fn rejects_min_above_max() {
        let mut config = noiseless_config("tam");
        config.min_output_t = 1.0;
        config.max_output_t = -1.0;
        let _ = Magnetometer::new(config);
    }
}
