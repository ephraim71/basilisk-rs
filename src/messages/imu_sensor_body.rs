//! IMU output in the body frame (upstream `IMUSensorBodyMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// One IMU sample in body components, as FSW `imuComm` produces it from a
/// platform-frame [`super::ImuMsg`]: upstream `IMUSensorBodyMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuSensorBodyMsg {
    /// Upstream `DVFrameBody` [m/s]: velocity change accumulated over the
    /// sample interval, body components.
    pub delta_velocity_body_mps: Vector3<f64>,
    /// Upstream `AccelBody` [m/s^2]: apparent (non-gravitational)
    /// acceleration, body components.
    pub accel_body_mps2: Vector3<f64>,
    /// Upstream `DRFrameBody` \[rad\]: attitude change accumulated over the
    /// sample interval, body components.
    pub delta_angle_body_rad: Vector3<f64>,
    /// Upstream `AngVelBody` [rad/s]: angular velocity, body components.
    pub angular_rate_body_radps: Vector3<f64>,
}

impl_telemetry!(ImuSensorBodyMsg {
    delta_velocity_body_mps,
    accel_body_mps2,
    delta_angle_body_rad,
    angular_rate_body_radps,
});
