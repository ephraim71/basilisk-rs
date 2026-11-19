//! Boresight angles to a target (upstream `BoreAngleMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Angles from a body-fixed boresight to a target direction: upstream
/// `BoreAngleMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BoreAngleMsg {
    /// Upstream `azimuth` \[rad\]: angle of the boresight about the target
    /// direction, which places the miss in a quadrant.
    pub azimuth_rad: f64,
    /// Upstream `missAngle` \[rad\]: angle between the boresight and the target
    /// direction.
    pub miss_angle_rad: f64,
}

impl_telemetry!(BoreAngleMsg {
    azimuth_rad,
    miss_angle_rad
});
