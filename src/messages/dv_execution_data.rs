//! Delta-v burn execution status (upstream `DvExecutionDataMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Progress of the current delta-v burn: upstream `DvExecutionDataMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DvExecutionDataMsg {
    /// Upstream `burnExecuting` (`uint32_t` 1 / 0): the burn is in progress.
    pub burn_executing: bool,
    /// Upstream `burnComplete` (`uint32_t` 1 / 0): the burn has finished.
    pub burn_complete: bool,
}

impl_telemetry!(DvExecutionDataMsg {
    burn_executing,
    burn_complete,
});
