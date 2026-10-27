//! One accelerometer/gyro packet (upstream `AccPktDataMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// One time-tagged gyro and accelerometer sample: upstream `AccPktDataMsg`,
/// the element type of [`super::AccelerometerDataMsg`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AccelerometerPacket {
    /// Upstream `measTime` \[ns\]: measurement time. Upstream labels it \[Tick\];
    /// its consumers (`dvAccumulation`, `inertialUKF`) treat it as
    /// nanoseconds. Zero marks an empty slot.
    pub meas_time_nanos: u64,
    /// Upstream `gyro_B` [rad/s]: angular rate, body components.
    pub gyro_b_radps: Vector3<f64>,
    /// Upstream `accel_B` [m/s^2]: acceleration, body components.
    pub accel_b_mps2: Vector3<f64>,
}

impl_telemetry!(AccelerometerPacket {
    meas_time_nanos,
    gyro_b_radps,
    accel_b_mps2,
});
