//! One scalar joint state (upstream `ScalarJointStateMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// State of a single one-degree-of-freedom joint, or its time derivative:
/// upstream `ScalarJointStateMsg`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ScalarJointStateMsg {
    /// Upstream `state` \[rad\] or \[m\] (or their rates): the joint angle of a
    /// hinge, the displacement of a slider.
    pub state: f64,
}

impl_telemetry!(ScalarJointStateMsg { state });
