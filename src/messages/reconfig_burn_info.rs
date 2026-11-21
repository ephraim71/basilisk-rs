//! One formation-reconfiguration burn (upstream `ReconfigBurnInfoMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Progress of a scheduled reconfiguration burn: upstream `flag`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum ReconfigBurnStatus {
    /// Not scheduled yet; upstream `0`, the zeroed default.
    #[default]
    NotScheduled = 0,
    /// Scheduled but not burned yet; upstream `1`.
    Pending = 1,
    /// Already burned; upstream `2`.
    Burned = 2,
    /// Skipped because it was combined with another burn; upstream `3`.
    Skipped = 3,
}

/// Schedule and target attitude of one burn: upstream `ReconfigBurnInfoMsg`,
/// the element type of [`super::ReconfigBurnArrayInfoMsg`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ReconfigBurnInfoMsg {
    /// Upstream `flag` (`int` 0–3): progress of the burn.
    pub status: ReconfigBurnStatus,
    /// Upstream `t` \[s\]: scheduled burn time, simulation time since the module
    /// reset.
    pub time_s: f64,
    /// Upstream `thrustOnTime` \[s\]: thrust duration.
    pub thrust_on_time_s: f64,
    /// Upstream `sigma_RN` [-]: MRP target attitude for the burn.
    pub sigma_rn: Vector3<f64>,
}

impl_telemetry!(enum ReconfigBurnStatus as u8);
impl_telemetry!(ReconfigBurnInfoMsg {
    status,
    time_s,
    thrust_on_time_s,
    sigma_rn,
});
