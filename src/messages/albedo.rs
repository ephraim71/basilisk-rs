use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AlbedoMsg {
    /// Albedo-to-solar flux ratio from every visible lit patch, ignoring the instrument's FOV.
    pub albedo_at_instrument_max: f64,
    pub albedo_flux_at_instrument_max_wpm2: f64,
    /// Albedo-to-solar flux ratio inside the FOV, cosine-weighted by the instrument normal.
    pub albedo_at_instrument: f64,
    pub albedo_flux_at_instrument_wpm2: f64,
}

impl TelemetryMessage for AlbedoMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "albedo_at_instrument_max".to_string(),
                value: self.albedo_at_instrument_max,
            },
            TelemetryField {
                path: "albedo_flux_at_instrument_max_wpm2".to_string(),
                value: self.albedo_flux_at_instrument_max_wpm2,
            },
            TelemetryField {
                path: "albedo_at_instrument".to_string(),
                value: self.albedo_at_instrument,
            },
            TelemetryField {
                path: "albedo_flux_at_instrument_wpm2".to_string(),
                value: self.albedo_flux_at_instrument_wpm2,
            },
        ]
    }
}
