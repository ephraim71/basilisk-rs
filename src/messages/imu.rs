//! IMU output in the sensor platform frame (upstream `IMUSensorMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// One IMU sample in platform (sensor-frame) components: upstream
/// `IMUSensorMsg`.
///
/// "Sensor" in the field names is the upstream platform frame `P`; FSW
/// `imuComm` rotates every field into the body frame, producing
/// [`super::ImuSensorBodyMsg`].
///
/// The Rust [`crate::sensors::imu_sensor::ImuSensor`] models only the rate
/// gyro: it fills `angular_rate_sensor_radps` and leaves the delta-velocity,
/// acceleration and delta-angle fields at zero.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuMsg {
    /// Upstream `DVFramePlatform` [m/s]: velocity change accumulated over the
    /// sample interval, platform-frame components.
    pub delta_velocity_sensor_mps: Vector3<f64>,
    /// Upstream `AccelPlatform` [m/s^2]: apparent (non-gravitational)
    /// acceleration, platform-frame components.
    pub accel_sensor_mps2: Vector3<f64>,
    /// Upstream `DRFramePlatform` \[rad\]: attitude change accumulated over the
    /// sample interval, platform-frame components.
    pub delta_angle_sensor_rad: Vector3<f64>,
    /// Upstream `AngVelPlatform` [rad/s]: angular velocity, platform-frame
    /// components.
    pub angular_rate_sensor_radps: Vector3<f64>,
}

impl_telemetry!(ImuMsg {
    delta_velocity_sensor_mps,
    accel_sensor_mps2,
    delta_angle_sensor_rad,
    angular_rate_sensor_radps,
});
