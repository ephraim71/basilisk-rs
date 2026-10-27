//! The burns of one formation reconfiguration (upstream
//! `ReconfigBurnArrayInfoMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_BURN_CNT, ReconfigBurnInfoMsg};

/// Every burn `spacecraftReconfig` plans for one reconfiguration: upstream
/// `ReconfigBurnArrayInfoMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ReconfigBurnArrayInfoMsg {
    /// Upstream `burnArray` [-]: the burns, in execution order.
    pub burn_array: [ReconfigBurnInfoMsg; MAX_BURN_CNT],
}

impl_telemetry!(ReconfigBurnArrayInfoMsg { burn_array });
