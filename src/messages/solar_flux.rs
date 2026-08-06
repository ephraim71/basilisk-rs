use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SolarFluxMsg {
    pub flux_w_per_m2: f64,
}

impl TelemetryMessage for SolarFluxMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "flux_w_per_m2".to_string(),
            value: self.flux_w_per_m2,
        }]
    }
}
