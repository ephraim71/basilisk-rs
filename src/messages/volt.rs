//! A space object's electric potential (upstream `VoltMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Electric potential of a spacecraft: upstream `VoltMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VoltMsg {
    /// Upstream `voltage` \[V\].
    pub voltage_v: f64,
}

impl_telemetry!(VoltMsg { voltage_v });
