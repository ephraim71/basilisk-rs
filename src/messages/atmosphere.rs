use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct AtmosphereMsg {
    pub neutral_density_kgpm3: f64,
    pub local_temp_k: f64,
}

impl TelemetryMessage for AtmosphereMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "neutral_density_kgpm3".to_string(),
                value: self.neutral_density_kgpm3,
            },
            TelemetryField {
                path: "local_temp_k".to_string(),
                value: self.local_temp_k,
            },
        ]
    }
}
