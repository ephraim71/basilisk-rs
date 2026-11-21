//! One space-weather index (upstream `SwDataMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// One space-weather channel value: upstream `SwDataMsg`. The space-weather
/// data module publishes 23 of them, one per NRLMSISE-00 input.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SwDataMsg {
    /// Upstream `dataValue`: the index value, Ap \[-\] or F10.7 \[sfu\]
    /// depending on the channel. (Upstream's unit comment, kg/m^3, is a
    /// copy-paste slip.)
    pub data_value: f64,
}

impl_telemetry!(SwDataMsg { data_value });
