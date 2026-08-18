use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};

use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::gauss_markov::GaussMarkov;
use crate::messages::{ImuMsg, Input, Output, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

/// Rate-gyro measurement model.
///
/// The error chain, in order: rotate (with body bias) -> Gauss-Markov errors
/// + sensor bias -> per-axis scale -> LSB discretization -> saturation. Only
/// the rotational states are modelled; `ImuMsg` carries no accelerometer
/// output.
#[derive(Clone, Debug)]
pub struct ImuSensorConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    pub body_to_sensor_quaternion: UnitQuaternion<f64>,
    /// [rad/s] rate bias in the body frame, added before the body->sensor
    /// rotation.
    pub body_rate_bias_radps: Vector3<f64>,
    /// [rad/s] rate bias in the sensor frame, added together with the error
    /// state, before the scale factor.
    pub sensor_rate_bias_radps: Vector3<f64>,
    /// Gauss-Markov error model: matrix square root of the covariance, in
    /// **3-sigma units** -- see [`crate::gauss_markov`]: a diagonal entry of
    /// `x` yields error steps with standard deviation `x / 3`. A zero matrix
    /// disables the noise.
    pub p_matrix_sqrt_radps: Matrix3<f64>,
    /// Error-state propagation matrix. Zero reduces the model to independent
    /// per-sample noise.
    pub a_matrix: Matrix3<f64>,
    /// [rad/s] per-axis soft walk bounds; the error state is statistically
    /// herded back rather than clamped. A non-positive bound leaves that
    /// axis unbounded.
    pub walk_bounds_radps: Vector3<f64>,
    /// Per-axis scale factors, applied after the errors so bias and noise
    /// are scaled together with the truth.
    pub gyro_scale: Vector3<f64>,
    /// [rad/s] least significant bit for quantization, floor-toward-zero.
    /// Zero disables.
    pub lsb_radps: f64,
    /// [rad/s] symmetric saturation bound.
    pub max_rate_radps: f64,
}

impl Default for ImuSensorConfig {
    /// An error-free sensor: identity mounting, zero biases, no noise, unit
    /// scale, no quantization, and a saturation bound far above any real rate.
    fn default() -> Self {
        Self {
            name: String::new(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            body_rate_bias_radps: Vector3::zeros(),
            sensor_rate_bias_radps: Vector3::zeros(),
            p_matrix_sqrt_radps: Matrix3::zeros(),
            a_matrix: Matrix3::zeros(),
            walk_bounds_radps: Vector3::zeros(),
            gyro_scale: Vector3::repeat(1.0),
            lsb_radps: 0.0,
            max_rate_radps: 1.0e6,
        }
    }
}

impl ImuSensorConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, vector) in [
            ("body_rate_bias_radps", self.body_rate_bias_radps),
            ("sensor_rate_bias_radps", self.sensor_rate_bias_radps),
            ("walk_bounds_radps", self.walk_bounds_radps),
            ("gyro_scale", self.gyro_scale),
        ] {
            if !vector.iter().all(|v| v.is_finite()) {
                return Err(format!("{label} must be finite, got {vector:?}"));
            }
        }
        for (label, matrix) in [
            ("p_matrix_sqrt_radps", self.p_matrix_sqrt_radps),
            ("a_matrix", self.a_matrix),
        ] {
            if !matrix.iter().all(|v| v.is_finite()) {
                return Err(format!("{label} must be finite, got {matrix:?}"));
            }
        }
        if !(self.lsb_radps >= 0.0) {
            return Err(format!(
                "lsb_radps must be non-negative, got {}",
                self.lsb_radps
            ));
        }
        if !(self.max_rate_radps > 0.0) {
            return Err(format!(
                "max_rate_radps must be positive, got {}",
                self.max_rate_radps
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct ImuSensor {
    pub config: ImuSensorConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub output_imu_msg: Output<ImuMsg>,
    error_model: GaussMarkov,
    rng: StdRng,
}

impl Module for ImuSensor {
    fn init(&mut self) {
        // On reset, rebuild the error model from config and clear its walk state.
        self.error_model = GaussMarkov::new(
            self.config.a_matrix,
            self.config.p_matrix_sqrt_radps,
            self.config.walk_bounds_radps,
        );
        self.output_imu_msg.write(ImuMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.input_state_msg.read();

        let true_rate_sensor = self.compute_body_to_sensor_rate(&state);
        let with_errors = self.apply_sensor_errors(true_rate_sensor);
        let scaled = with_errors.component_mul(&self.config.gyro_scale);
        let quantized = self.apply_discretization(scaled);
        let saturated = quantized
            .map(|value| value.clamp(-self.config.max_rate_radps, self.config.max_rate_radps));

        self.output_imu_msg.write(ImuMsg {
            angular_rate_sensor_radps: saturated,
        });
    }
}

impl ImuSensor {
    pub fn new(config: ImuSensorConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid ImuSensorConfig: {msg}");
        }
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            error_model: GaussMarkov::new(
                config.a_matrix,
                config.p_matrix_sqrt_radps,
                config.walk_bounds_radps,
            ),
            config,
            input_state_msg: Input::default(),
            output_imu_msg: Output::default(),
        }
    }

    fn compute_body_to_sensor_rate(&self, state: &SpacecraftStateMsg) -> Vector3<f64> {
        self.config
            .body_to_sensor_quaternion
            .transform_vector(&(state.omega_radps + self.config.body_rate_bias_radps))
    }

    fn apply_sensor_errors(&mut self, rate_sensor_radps: Vector3<f64>) -> Vector3<f64> {
        let error_state_radps = self.error_model.compute_next_state(&mut self.rng);

        rate_sensor_radps + error_state_radps + self.config.sensor_rate_bias_radps
    }

    fn apply_discretization(&self, rate_sensor_radps: Vector3<f64>) -> Vector3<f64> {
        if self.config.lsb_radps <= 0.0 {
            return rate_sensor_radps;
        }
        rate_sensor_radps.map(|value| {
            (value.abs() / self.config.lsb_radps).floor() * self.config.lsb_radps * value.signum()
        })
    }
}

fn seed_from_name(name: &str) -> u64 {
    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    use crate::messages::{Output, SpacecraftStateMsg};
    use crate::{Module, SimulationContext};

    use super::{ImuSensor, ImuSensorConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    /// An error-free configuration: identity rotation, zero biases, no noise,
    /// unit scale, no quantization, generous saturation.
    fn clean_config(name: &str) -> ImuSensorConfig {
        ImuSensorConfig {
            name: name.to_string(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            body_rate_bias_radps: Vector3::zeros(),
            sensor_rate_bias_radps: Vector3::zeros(),
            p_matrix_sqrt_radps: Matrix3::zeros(),
            a_matrix: Matrix3::zeros(),
            walk_bounds_radps: Vector3::zeros(),
            gyro_scale: Vector3::repeat(1.0),
            lsb_radps: 0.0,
            max_rate_radps: 1.0e6,
        }
    }

    fn run_imu(imu: &mut ImuSensor, omega_radps: Vector3<f64>) -> Vector3<f64> {
        let state_out = Output::new(SpacecraftStateMsg {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
            omega_radps,
        });
        imu.input_state_msg.connect(state_out.slot());
        imu.init();
        imu.update(&dummy_context());
        imu.output_imu_msg.read().angular_rate_sensor_radps
    }

    /// omega_body = [0.0, 0.15, 0.1] rad/s, no errors → sensor output = input unchanged.
    #[test]
    fn identity_rotation_passes_through_omega() {
        let omega = Vector3::new(0.0, 0.15, 0.1);
        let mut imu = ImuSensor::new(clean_config("imu"));
        let out = run_imu(&mut imu, omega);
        assert!(
            (out - omega).norm() < 1e-12,
            "expected {omega:?}, got {out:?}"
        );
    }

    /// yaw=0.7854 rad, pitch=1.0 rad, roll=0.1 rad; omega_body = [0.0, 0.15, 0.1] rad/s, no errors.
    /// Expected: sensor output = q.transform(omega_body).
    #[test]
    fn known_rotation_transforms_omega() {
        let omega_body = Vector3::new(0.0, 0.15, 0.1);
        // Euler 3-2-1: yaw=0.7854, pitch=1.0, roll=0.1
        let q = UnitQuaternion::from_euler_angles(0.1, 1.0, 0.7854);
        let expected = q.transform_vector(&omega_body);

        let mut config = clean_config("imu");
        config.body_to_sensor_quaternion = q;
        let mut imu = ImuSensor::new(config);
        let out = run_imu(&mut imu, omega_body);
        assert!(
            (out - expected).norm() < 1e-12,
            "expected {expected:?}, got {out:?}"
        );
    }

    /// Body bias is added before the rotation, sensor bias after it — the two
    /// are distinguishable whenever the mounting rotation is non-identity.
    #[test]
    fn body_and_sensor_biases_apply_on_their_own_sides_of_the_rotation() {
        let omega_body = Vector3::new(0.01, -0.02, 0.03);
        let q = UnitQuaternion::from_euler_angles(0.1, 1.0, 0.7854);
        let body_bias = Vector3::new(1.0e-3, -2.0e-3, 3.0e-3);
        let sensor_bias = Vector3::new(-4.0e-3, 5.0e-3, -6.0e-3);

        let mut config = clean_config("imu");
        config.body_to_sensor_quaternion = q;
        config.body_rate_bias_radps = body_bias;
        config.sensor_rate_bias_radps = sensor_bias;
        let mut imu = ImuSensor::new(config);

        let out = run_imu(&mut imu, omega_body);
        let expected = q.transform_vector(&(omega_body + body_bias)) + sensor_bias;
        assert!(
            (out - expected).norm() < 1e-15,
            "expected {expected:?}, got {out:?}"
        );
    }

    /// The per-axis scale multiplies bias and noise as well as the truth
    /// (errors are applied before the scale).
    #[test]
    fn scale_applies_after_the_sensor_errors() {
        let omega_body = Vector3::new(0.01, 0.02, 0.03);
        let sensor_bias = Vector3::new(1.0e-3, 1.0e-3, 1.0e-3);
        let scale = Vector3::new(2.0, 3.0, 4.0);

        let mut config = clean_config("imu");
        config.sensor_rate_bias_radps = sensor_bias;
        config.gyro_scale = scale;
        let mut imu = ImuSensor::new(config);

        let out = run_imu(&mut imu, omega_body);
        let expected = (omega_body + sensor_bias).component_mul(&scale);
        assert!(
            (out - expected).norm() < 1e-15,
            "expected {expected:?}, got {out:?}"
        );
    }

    /// LSB quantization floors the magnitude toward zero and keeps the sign.
    #[test]
    fn discretization_floors_toward_zero() {
        let lsb = 1.0e-3;
        let mut config = clean_config("imu");
        config.lsb_radps = lsb;
        let mut imu = ImuSensor::new(config);

        let out = run_imu(&mut imu, Vector3::new(0.0123, -0.0456, 0.0));
        let quantize = |v: f64| (v.abs() / lsb).floor() * lsb * v.signum();
        assert!((out.x - quantize(0.0123)).abs() < 1e-15, "{out:?}");
        assert!((out.y - quantize(-0.0456)).abs() < 1e-15, "{out:?}");
        assert_eq!(out.z, 0.0);
    }

    /// Saturation clamps the discretized output to ±max_rate_radps.
    #[test]
    fn saturation_clamps_the_output() {
        let mut config = clean_config("imu");
        config.max_rate_radps = 0.05;
        let mut imu = ImuSensor::new(config);

        let out = run_imu(&mut imu, Vector3::new(0.2, -0.3, 0.01));
        assert_eq!(out.x, 0.05);
        assert_eq!(out.y, -0.05);
        assert!((out.z - 0.01).abs() < 1e-15);
    }

    /// With a zero propagation matrix the Gauss-Markov model reduces to IID
    /// noise whose standard deviation matches the configured sqrt-covariance.
    #[test]
    fn iid_noise_matches_configured_std() {
        let std = 1.0e-4;
        let mut config = clean_config("imu_noise");
        // The P matrix is in 3-sigma units: pass 3x the desired sigma.
        config.p_matrix_sqrt_radps = Matrix3::from_diagonal(&Vector3::repeat(3.0 * std));
        let mut imu = ImuSensor::new(config);

        let state_out = Output::new(SpacecraftStateMsg::default());
        imu.input_state_msg.connect(state_out.slot());
        imu.init();

        let n = 20_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            imu.update(&dummy_context());
            let noise = imu.output_imu_msg.read().angular_rate_sensor_radps.x;
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

    /// Positive walk bounds herd the accumulated error state: the walk stays
    /// in the bound's neighbourhood (transient excursions are expected of a
    /// soft bound) instead of running away.
    #[test]
    fn walk_bounds_herd_the_error_state() {
        let std = 1.0e-3;
        let bound = 5.0e-4;
        let mut config = clean_config("imu_walk");
        config.p_matrix_sqrt_radps = Matrix3::from_diagonal(&Vector3::repeat(std));
        config.a_matrix = Matrix3::identity();
        config.walk_bounds_radps = Vector3::repeat(bound);
        let mut imu = ImuSensor::new(config);

        let state_out = Output::new(SpacecraftStateMsg::default());
        imu.input_state_msg.connect(state_out.slot());
        imu.init();

        for _ in 0..1_000 {
            imu.update(&dummy_context());
            let out = imu.output_imu_msg.read().angular_rate_sensor_radps;
            for axis in 0..3 {
                assert!(
                    out[axis].abs() <= 4.0 * bound,
                    "error escaped the soft walk bound: {out:?}"
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "max_rate_radps")]
    fn rejects_non_positive_saturation() {
        let mut config = clean_config("imu");
        config.max_rate_radps = 0.0;
        let _ = ImuSensor::new(config);
    }
}
