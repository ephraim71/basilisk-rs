use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct SunSensorMsg {
    pub alpha_rad: f64,
    pub beta_rad: f64,
    pub valid: bool,
}

impl TelemetryMessage for SunSensorMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "alpha_rad".to_string(),
                value: self.alpha_rad,
            },
            TelemetryField {
                path: "beta_rad".to_string(),
                value: self.beta_rad,
            },
            TelemetryField {
                path: "valid".to_string(),
                value: if self.valid { 1.0 } else { 0.0 },
            },
        ]
    }
}
