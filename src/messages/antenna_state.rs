//! Commanded antenna mode (upstream `AntennaStateMsgPayload`).

use serde::{Deserialize, Serialize};

use super::AntennaState;
use super::flatten::impl_telemetry;

impl AntennaState {
    /// Whether the antenna receives (`Rx` or `RxTx`).
    pub fn receives(self) -> bool {
        matches!(self, Self::Rx | Self::RxTx)
    }

    /// Whether the antenna transmits (`Tx` or `RxTx`).
    pub fn transmits(self) -> bool {
        matches!(self, Self::Tx | Self::RxTx)
    }
}

/// Mode an antenna is commanded into: upstream `AntennaStateMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AntennaStateMsg {
    /// Upstream `antennaState` (`uint32_t` 0 / 1 / 2 / 3).
    pub antenna_state: AntennaState,
}

impl_telemetry!(AntennaStateMsg { antenna_state });
