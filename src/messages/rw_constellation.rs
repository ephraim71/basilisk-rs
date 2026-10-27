//! Reaction-wheel constellation configuration (upstream
//! `RWConstellationMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_EFF_COUNT, RwConfigElementMsg, filled_prefix};

/// FSW configuration of every reaction wheel: upstream `RWConstellationMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct RwConstellationMsg {
    /// Upstream `numRW` [-]: number of wheels in use, the leading entries of
    /// `reaction_wheels`.
    pub num_rw: usize,
    /// Upstream `reactionWheels` [-]: per-wheel configuration; entries past
    /// `num_rw` are unused and zero.
    #[serde(with = "super::big_array")]
    pub reaction_wheels: [RwConfigElementMsg; MAX_EFF_COUNT],
}

impl Default for RwConstellationMsg {
    fn default() -> Self {
        Self {
            num_rw: 0,
            reaction_wheels: [RwConfigElementMsg::default(); MAX_EFF_COUNT],
        }
    }
}

impl RwConstellationMsg {
    /// Builds the message from the wheels in use, setting `num_rw`.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_EFF_COUNT`] wheels are given.
    pub fn from_active(reaction_wheels: &[RwConfigElementMsg]) -> Self {
        Self {
            num_rw: reaction_wheels.len(),
            reaction_wheels: filled_prefix(reaction_wheels, "reaction-wheel configurations"),
        }
    }

    /// The first `num_rw` entries (at most [`MAX_EFF_COUNT`]).
    pub fn active_wheels(&self) -> &[RwConfigElementMsg] {
        &self.reaction_wheels[..self.num_rw.min(MAX_EFF_COUNT)]
    }
}

impl_telemetry!(RwConstellationMsg {
    num_rw,
    reaction_wheels,
});
