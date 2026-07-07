use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct EclipseMsg {
    pub illumination_factor: f64,
}

impl TelemetryMessage for EclipseMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "illumination_factor".to_string(),
            value: self.illumination_factor,
        }]
    }
}
