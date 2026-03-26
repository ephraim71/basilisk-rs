use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
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
