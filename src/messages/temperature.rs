//! A temperature (upstream `TemperatureMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Current temperature of a device: upstream `TemperatureMsg`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct TemperatureMsg {
    /// Upstream `temperature` \[°C\].
    pub temperature_c: f64,
}

impl_telemetry!(TemperatureMsg { temperature_c });
