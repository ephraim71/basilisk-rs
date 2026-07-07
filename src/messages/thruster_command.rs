use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ThrusterCommandMsg {
    pub on_time_s: f64,
}

impl TelemetryMessage for ThrusterCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "on_time_s".to_string(),
            value: self.on_time_s,
        }]
    }
}
