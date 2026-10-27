//! Thruster on-time commands (upstream `THRArrayOnTimeCmdMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_EFF_COUNT, filled_prefix};

/// Requested on-time of every thruster: upstream `THRArrayOnTimeCmdMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThrusterArrayOnTimeCommandMsg {
    /// Upstream `OnTimeRequest` \[s\]: requested on-time per thruster, in
    /// thruster order. (The upstream comment calls it a "control request
    /// fraction"; every producer and consumer treats it as seconds.)
    #[serde(with = "super::big_array")]
    pub on_time_request_s: [f64; MAX_EFF_COUNT],
}

impl Default for ThrusterArrayOnTimeCommandMsg {
    fn default() -> Self {
        Self {
            on_time_request_s: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl ThrusterArrayOnTimeCommandMsg {
    /// Builds the message from the on-times of the thrusters in use; the
    /// remaining entries stay zero.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_EFF_COUNT`] values are given.
    pub fn from_active(on_time_request_s: &[f64]) -> Self {
        Self {
            on_time_request_s: filled_prefix(on_time_request_s, "thruster on-time requests"),
        }
    }
}

impl_telemetry!(ThrusterArrayOnTimeCommandMsg { on_time_request_s });
