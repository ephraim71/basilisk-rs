//! VSCMG cluster configuration (upstream `VSCMGArrayConfigMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_EFF_COUNT, VscmgConfigElementMsg, filled_prefix};

/// Configuration of the whole VSCMG cluster: upstream `VSCMGArrayConfigMsg`.
///
/// Upstream stores the cluster as parallel arrays (`Gs0Matrix_B`,
/// `Gt0Matrix_B`, `GgMatrix_B`, `JsList`, `JtList`, `JgList`, `IwsList`,
/// `Omega0List`, `gamma0List`, `gammaDot0List`) whose `i`-th entries are the
/// fields of one [`VscmgConfigElementMsg`]; here entry `i` of `vscmgs` holds
/// them together, as upstream's `fswSetupVSCMGs` builds them before
/// flattening.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VscmgArrayConfigMsg {
    /// Upstream `numVSCMG` [-]: number of VSCMGs in use, the leading entries
    /// of `vscmgs`.
    pub num_vscmg: usize,
    /// Per-VSCMG configuration; entries past `num_vscmg` are unused and zero.
    #[serde(with = "super::big_array")]
    pub vscmgs: [VscmgConfigElementMsg; MAX_EFF_COUNT],
}

impl Default for VscmgArrayConfigMsg {
    fn default() -> Self {
        Self {
            num_vscmg: 0,
            vscmgs: [VscmgConfigElementMsg::default(); MAX_EFF_COUNT],
        }
    }
}

impl VscmgArrayConfigMsg {
    /// Builds the message from the VSCMGs in use, setting `num_vscmg`.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_EFF_COUNT`] VSCMGs are given.
    pub fn from_active(vscmgs: &[VscmgConfigElementMsg]) -> Self {
        Self {
            num_vscmg: vscmgs.len(),
            vscmgs: filled_prefix(vscmgs, "VSCMG configurations"),
        }
    }

    /// The first `num_vscmg` entries (at most [`MAX_EFF_COUNT`]).
    pub fn active_vscmgs(&self) -> &[VscmgConfigElementMsg] {
        &self.vscmgs[..self.num_vscmg.min(MAX_EFF_COUNT)]
    }
}

impl_telemetry!(VscmgArrayConfigMsg { num_vscmg, vscmgs });
