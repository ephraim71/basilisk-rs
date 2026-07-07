use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SunSensorMsg {
    pub sensed_value: f64,
    pub true_value: f64,
    pub valid: bool,
}

impl TelemetryMessage for SunSensorMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "sensed_value".to_string(),
                value: self.sensed_value,
            },
            TelemetryField {
                path: "true_value".to_string(),
                value: self.true_value,
            },
            TelemetryField {
                path: "valid".to_string(),
                value: if self.valid { 1.0 } else { 0.0 },
            },
        ]
    }
}
