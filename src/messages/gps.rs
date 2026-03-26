use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct GpsMsg {
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    pub gps_week: u32,
    pub gps_seconds_of_week: f64,
}

impl TelemetryMessage for GpsMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "position_m.x".to_string(),
                value: self.position_m.x,
            },
            TelemetryField {
                path: "position_m.y".to_string(),
                value: self.position_m.y,
            },
            TelemetryField {
                path: "position_m.z".to_string(),
                value: self.position_m.z,
            },
            TelemetryField {
                path: "velocity_mps.x".to_string(),
                value: self.velocity_mps.x,
            },
            TelemetryField {
                path: "velocity_mps.y".to_string(),
                value: self.velocity_mps.y,
            },
            TelemetryField {
                path: "velocity_mps.z".to_string(),
                value: self.velocity_mps.z,
            },
            TelemetryField {
                path: "gps_week".to_string(),
                value: self.gps_week as f64,
            },
            TelemetryField {
                path: "gps_seconds_of_week".to_string(),
                value: self.gps_seconds_of_week,
            },
        ]
    }
}
