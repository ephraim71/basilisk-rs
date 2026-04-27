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

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{UnitQuaternion, Vector3};

    use crate::messages::{Output, SpacecraftStateMsg};
    use crate::{Module, SimulationContext};

    use super::{Imu, ImuConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    fn run_imu(imu: &mut Imu, omega_radps: Vector3<f64>) -> Vector3<f64> {
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

    /// omega_body = [0.0, 0.15, 0.1] rad/s, zero noise → sensor output = input unchanged.
    #[test]
    fn identity_rotation_passes_through_omega() {
        let omega = Vector3::new(0.0, 0.15, 0.1);
        let mut imu = Imu::new(ImuConfig {
            name: "imu".to_string(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: UnitQuaternion::identity(),
            rate_noise_std_radps: Vector3::zeros(),
        });
        let out = run_imu(&mut imu, omega);
        assert!(
            (out - omega).norm() < 1e-12,
            "expected {omega:?}, got {out:?}"
        );
    }

    /// yaw=0.7854 rad, pitch=1.0 rad, roll=0.1 rad; omega_body = [0.0, 0.15, 0.1] rad/s, zero noise.
    /// Expected: sensor output = q.transform(omega_body).
    #[test]
    fn known_rotation_transforms_omega() {
        let omega_body = Vector3::new(0.0, 0.15, 0.1);
        // Euler 3-2-1: yaw=0.7854, pitch=1.0, roll=0.1 — same as Basilisk setBodyToPlatformDCM
        let q = UnitQuaternion::from_euler_angles(0.1, 1.0, 0.7854);
        let expected = q.transform_vector(&omega_body);

        let mut imu = Imu::new(ImuConfig {
            name: "imu".to_string(),
            position_m: Vector3::zeros(),
            body_to_sensor_quaternion: q,
            rate_noise_std_radps: Vector3::zeros(),
        });
        let out = run_imu(&mut imu, omega_body);
        assert!(
            (out - expected).norm() < 1e-12,
            "expected {expected:?}, got {out:?}"
        );
    }
}
