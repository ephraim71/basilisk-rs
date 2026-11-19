//! Desired inertial velocity at a maneuver (upstream
//! `DesiredVelocityMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Inertial velocity a maneuver should reach, and when: upstream
/// `DesiredVelocityMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DesiredVelocityMsg {
    /// Upstream `vDesired_N` [m/s]: desired velocity, inertial components.
    pub v_desired_n_mps: Vector3<f64>,
    /// Upstream `maneuverTime` \[s\]: time of the maneuver.
    pub maneuver_time_s: f64,
}

impl_telemetry!(DesiredVelocityMsg {
    v_desired_n_mps,
    maneuver_time_s,
});
