use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct SunLineMsg {
    pub sun_vector_body: Vector3<f64>,
    pub valid: bool,
    pub num_active_sensors: u32,
}

impl TelemetryMessage for SunLineMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "sun_vector_body.x".to_string(),
                value: self.sun_vector_body.x,
            },
            TelemetryField {
                path: "sun_vector_body.y".to_string(),
                value: self.sun_vector_body.y,
            },
            TelemetryField {
                path: "sun_vector_body.z".to_string(),
                value: self.sun_vector_body.z,
            },
            TelemetryField {
                path: "valid".to_string(),
                value: if self.valid { 1.0 } else { 0.0 },
            },
            TelemetryField {
                path: "num_active_sensors".to_string(),
                value: self.num_active_sensors as f64,
            },
        ]
    }
}
