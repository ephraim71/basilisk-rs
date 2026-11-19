//! States of a set of joints (upstream C++ `JointArrayStateMsgPayload`).

use serde::{Deserialize, Serialize};

/// Joint states and their first two time derivatives, one entry per joint:
/// upstream `JointArrayStateMsg`.
///
/// Upstream's payload holds `std::vector`s, so the length is whatever the
/// producer writes; the message has no telemetry columns for the same
/// reason.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct JointArrayStateMsg {
    /// Upstream `states` \[rad\] or \[m\]: joint states.
    pub states: Vec<f64>,
    /// Upstream `stateDots` \[rad/s\] or \[m/s\]: joint state rates.
    pub state_dots: Vec<f64>,
    /// Upstream `stateDDots` \[rad/s^2\] or \[m/s^2\]: joint state
    /// accelerations.
    pub state_ddots: Vec<f64>,
}
