use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct ImuMsg {
    pub angular_rate_sensor_radps: Vector3<f64>,
}

impl TelemetryMessage for ImuMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "angular_rate_sensor_radps.x".to_string(),
                value: self.angular_rate_sensor_radps.x,
            },
            TelemetryField {
                path: "angular_rate_sensor_radps.y".to_string(),
                value: self.angular_rate_sensor_radps.y,
            },
            TelemetryField {
                path: "angular_rate_sensor_radps.z".to_string(),
                value: self.angular_rate_sensor_radps.z,
            },
        ]
    }
}
