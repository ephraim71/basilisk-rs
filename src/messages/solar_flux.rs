use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
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
