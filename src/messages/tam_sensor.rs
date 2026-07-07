use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Three-axis magnetometer measurement expressed in the sensor frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TamSensorMsg {
    pub magnetic_field_sensor_t: Vector3<f64>,
}

impl TelemetryMessage for TamSensorMsg {
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
