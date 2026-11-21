//! IMU platform-to-body conversion: port of upstream Basilisk `imuComm`
//! (`IMUSensorData`).
//!
//! Each update rotates the four vectors of a platform-frame [`ImuMsg`]
//! (accumulated delta-velocity, acceleration, accumulated delta-angle and
//! angular rate) into the spacecraft body frame with the configured `[BP]`
//! and publishes them as an [`ImuSensorBodyMsg`].

use nalgebra::Matrix3;

use crate::messages::{ImuMsg, ImuSensorBodyMsg, Input, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ImuComm`].
#[derive(Clone, Debug)]
pub struct ImuCommConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `dcm_BP`: DCM from the sensor platform frame `P` to the body
    /// frame `B` (`v_B = [BP] v_P`). Upstream stores it row-major as
    /// `double[9]`; build it from that array with [`Matrix3::from_row_slice`].
    /// `Default` keeps upstream's zero matrix, so it must be set.
    pub dcm_bp: Matrix3<f64>,
}

impl Default for ImuCommConfig {
    fn default() -> Self {
        Self {
            name: "imuComm".to_string(),
            dcm_bp: Matrix3::zeros(),
        }
    }
}

/// IMU sample rotation from platform to body frame: upstream `imuComm`.
#[derive(Clone, Debug)]
pub struct ImuComm {
    /// User-settable parameters.
    pub config: ImuCommConfig,
    /// Upstream `imuComInMsg`: IMU sample in platform components.
    pub imu_com_in_msg: Input<ImuMsg>,
    /// Upstream `imuSensorOutMsg`: the same sample in body components.
    pub imu_sensor_out_msg: Output<ImuSensorBodyMsg>,
}

impl ImuComm {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: ImuCommConfig) -> Self {
        Self {
            config,
            imu_com_in_msg: Input::default(),
            imu_sensor_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_imuProcessTelem`: the module retains no state.
    fn check_connections(&self) {
        if !self.imu_com_in_msg.is_connected() {
            log::error!("imuComm.imu_com_in_msg is not connected");
        }
    }

    fn body_frame_sample(&self, platform: &ImuMsg) -> ImuSensorBodyMsg {
        let dcm_bp = self.config.dcm_bp;
        ImuSensorBodyMsg {
            delta_velocity_body_mps: dcm_bp * platform.delta_velocity_sensor_mps,
            accel_body_mps2: dcm_bp * platform.accel_sensor_mps2,
            delta_angle_body_rad: dcm_bp * platform.delta_angle_sensor_rad,
            angular_rate_body_radps: dcm_bp * platform.angular_rate_sensor_radps,
        }
    }
}

impl Module for ImuComm {
    fn init(&mut self) {
        self.imu_sensor_out_msg.write(ImuSensorBodyMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let platform = self.imu_com_in_msg.read();
        self.imu_sensor_out_msg
            .write(self.body_frame_sample(&platform));
    }
}

#[cfg(test)]
mod tests {
    // Upstream `imuComm` has no unit test; these follow its documentation,
    // with hand-derived truth.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{ImuComm, ImuCommConfig};
    use crate::messages::{ImuMsg, ImuSensorBodyMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn platform_sample() -> ImuMsg {
        ImuMsg {
            delta_velocity_sensor_mps: Vector3::new(1.0, 2.0, 3.0),
            accel_sensor_mps2: Vector3::new(-0.5, 0.25, 4.0),
            delta_angle_sensor_rad: Vector3::new(1e-3, -2e-3, 5e-4),
            angular_rate_sensor_radps: Vector3::new(0.1, -0.2, 0.3),
        }
    }

    fn assert_body_sample_eq(actual: &ImuSensorBodyMsg, expected: &ImuSensorBodyMsg) {
        assert_eq!(
            actual.delta_velocity_body_mps,
            expected.delta_velocity_body_mps
        );
        assert_eq!(actual.accel_body_mps2, expected.accel_body_mps2);
        assert_eq!(actual.delta_angle_body_rad, expected.delta_angle_body_rad);
        assert_eq!(
            actual.angular_rate_body_radps,
            expected.angular_rate_body_radps
        );
    }

    /// A platform mounted with `P1 = B2`, `P2 = -B1`, `P3 = B3`, given
    /// row-major as upstream's `dcm_BP`: platform `(x, y, z)` reads
    /// `(-y, x, z)` in the body frame.
    #[test]
    fn rotates_every_platform_vector_into_the_body_frame() {
        let dcm_bp = Matrix3::from_row_slice(&[
            0.0, -1.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 0.0, 1.0,
        ]);
        let swap = |v: Vector3<f64>| Vector3::new(-v.y, v.x, v.z);
        let sample = platform_sample();
        let expected = ImuSensorBodyMsg {
            delta_velocity_body_mps: swap(sample.delta_velocity_sensor_mps),
            accel_body_mps2: swap(sample.accel_sensor_mps2),
            delta_angle_body_rad: swap(sample.delta_angle_sensor_rad),
            angular_rate_body_radps: swap(sample.angular_rate_sensor_radps),
        };

        let imu_output = Output::new(sample);
        let mut module = ImuComm::new(ImuCommConfig {
            dcm_bp,
            ..Default::default()
        });
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&imu_output, &mut module.imu_com_in_msg);
        simulation.connect(&module.imu_sensor_out_msg, &mut recorder.input_msg);
        simulation.add_module("imuComm", &mut module, 500_000_000, 0);
        simulation.add_module("recorder", &mut recorder, 500_000_000, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), 3);
        for sample in &recorder.samples {
            assert_body_sample_eq(sample, &expected);
        }
    }

    /// A general DCM, set on the public config after `new`: every output is
    /// `[BP]` times the matching input.
    #[test]
    fn applies_a_dcm_set_after_construction() {
        let angle: f64 = 0.3;
        let (s, c) = angle.sin_cos();
        // Rotation about the first axis.
        let dcm_bp = Matrix3::from_row_slice(&[1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c]);
        let sample = platform_sample();
        let imu_output = Output::new(sample.clone());
        let mut module = ImuComm::new(ImuCommConfig::default());
        module.config.dcm_bp = dcm_bp;
        imu_output.connect_to(&mut module.imu_com_in_msg);
        module.init();
        module.update(&context(0));

        let rotate = |v: Vector3<f64>| Vector3::new(v.x, c * v.y + s * v.z, -s * v.y + c * v.z);
        let output = module.imu_sensor_out_msg.read();
        for (actual, input) in [
            (
                output.delta_velocity_body_mps,
                sample.delta_velocity_sensor_mps,
            ),
            (output.accel_body_mps2, sample.accel_sensor_mps2),
            (output.delta_angle_body_rad, sample.delta_angle_sensor_rad),
            (
                output.angular_rate_body_radps,
                sample.angular_rate_sensor_radps,
            ),
        ] {
            let expected = rotate(input);
            assert!(
                (actual - expected).norm() <= 1e-14,
                "{actual} != {expected}"
            );
        }
    }

    /// Upstream's zero default `dcm_BP` maps everything to zero, and an
    /// unconnected input reads zeros.
    #[test]
    fn default_configuration_and_unconnected_input_give_zero_output() {
        let imu_output = Output::new(platform_sample());
        let mut module = ImuComm::new(ImuCommConfig::default());
        imu_output.connect_to(&mut module.imu_com_in_msg);
        module.init();
        module.update(&context(0));
        assert_body_sample_eq(
            &module.imu_sensor_out_msg.read(),
            &ImuSensorBodyMsg::default(),
        );

        let mut unconnected = ImuComm::new(ImuCommConfig {
            dcm_bp: Matrix3::identity(),
            ..Default::default()
        });
        unconnected.init();
        unconnected.update(&context(0));
        assert_body_sample_eq(
            &unconnected.imu_sensor_out_msg.read(),
            &ImuSensorBodyMsg::default(),
        );
    }
}
