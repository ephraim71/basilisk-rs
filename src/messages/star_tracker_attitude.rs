//! Star-tracker attitude in the body frame (upstream `STAttMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Star-tracker attitude measurement mapped into the body frame: upstream
/// `STAttMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StarTrackerAttitudeMsg {
    /// Upstream `timeTag` \[ns\]: vehicle time of the measurement.
    pub time_tag_nanos: u64,
    /// Upstream `MRP_BdyInrtl` [-]: MRP attitude of the body relative to
    /// inertial (σ_BN).
    pub sigma_bn: Vector3<f64>,
}

impl_telemetry!(StarTrackerAttitudeMsg {
    time_tag_nanos,
    sigma_bn,
});
