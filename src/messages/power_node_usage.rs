use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct PowerNodeUsageMsg {
    pub net_power_w: f64,
}

impl TelemetryMessage for PowerNodeUsageMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "net_power_w".to_string(),
            value: self.net_power_w,
        }]
    }
}
