//! One electric current (upstream `CurrentMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// An electric current: upstream `CurrentMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CurrentMsg {
    /// Upstream `current` \[A\].
    pub current_a: f64,
}

impl_telemetry!(CurrentMsg { current_a });
