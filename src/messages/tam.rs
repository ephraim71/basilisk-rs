use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct TamMsg {
    pub magnetic_field_sensor_t: Vector3<f64>,
}

impl TelemetryMessage for TamMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "magnetic_field_sensor_t.x".to_string(),
                value: self.magnetic_field_sensor_t.x,
            },
            TelemetryField {
                path: "magnetic_field_sensor_t.y".to_string(),
                value: self.magnetic_field_sensor_t.y,
            },
            TelemetryField {
                path: "magnetic_field_sensor_t.z".to_string(),
                value: self.magnetic_field_sensor_t.z,
            },
        ]
    }
}
