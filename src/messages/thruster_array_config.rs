//! Thruster cluster configuration (upstream `THRArrayConfigMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_EFF_COUNT, ThrusterConfigMsg, filled_prefix};

/// Configuration of the whole thruster cluster: upstream `THRArrayConfigMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThrusterArrayConfigMsg {
    /// Upstream `numThrusters` [-]: number of thrusters in use, the leading
    /// entries of `thrusters`.
    pub num_thrusters: usize,
    /// Upstream `thrusters` [-]: per-thruster configuration; entries past
    /// `num_thrusters` are unused and zero.
    #[serde(with = "super::big_array")]
    pub thrusters: [ThrusterConfigMsg; MAX_EFF_COUNT],
}

impl Default for ThrusterArrayConfigMsg {
    fn default() -> Self {
        Self {
            num_thrusters: 0,
            thrusters: [ThrusterConfigMsg::default(); MAX_EFF_COUNT],
        }
    }
}

impl ThrusterArrayConfigMsg {
    /// Builds the message from the thrusters in use, setting `num_thrusters`.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_EFF_COUNT`] thrusters are given.
    pub fn from_active(thrusters: &[ThrusterConfigMsg]) -> Self {
        Self {
            num_thrusters: thrusters.len(),
            thrusters: filled_prefix(thrusters, "thruster configurations"),
        }
    }

    /// The first `num_thrusters` entries (at most [`MAX_EFF_COUNT`]).
    pub fn active_thrusters(&self) -> &[ThrusterConfigMsg] {
        &self.thrusters[..self.num_thrusters.min(MAX_EFF_COUNT)]
    }
}

impl_telemetry!(ThrusterArrayConfigMsg {
    num_thrusters,
    thrusters,
});
