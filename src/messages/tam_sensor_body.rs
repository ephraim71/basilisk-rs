use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Three-axis magnetometer measurement expressed in the spacecraft body frame.
#[derive(Clone, Debug, Default)]
pub struct TamSensorBodyMsg {
    pub magnetic_field_body_t: Vector3<f64>,
}

impl TelemetryMessage for TamSensorBodyMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "magnetic_field_body_t.x".to_string(),
                value: self.magnetic_field_body_t.x,
            },
            TelemetryField {
                path: "magnetic_field_body_t.y".to_string(),
                value: self.magnetic_field_body_t.y,
            },
            TelemetryField {
                path: "magnetic_field_body_t.z".to_string(),
                value: self.magnetic_field_body_t.z,
            },
        ]
    }
}
