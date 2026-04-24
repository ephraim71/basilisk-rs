use nalgebra::{Matrix3, Vector3};

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct PlanetStateMsg {
    pub position_inertial_m: Vector3<f64>,
    pub velocity_inertial_mps: Vector3<f64>,
    pub has_orientation: bool,
    pub inertial_to_fixed: Matrix3<f64>,
    pub inertial_to_fixed_dot: Matrix3<f64>,
}

impl TelemetryMessage for PlanetStateMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "position_inertial_m.x".to_string(),
                value: self.position_inertial_m.x,
            },
            TelemetryField {
                path: "position_inertial_m.y".to_string(),
                value: self.position_inertial_m.y,
            },
            TelemetryField {
                path: "position_inertial_m.z".to_string(),
                value: self.position_inertial_m.z,
            },
            TelemetryField {
                path: "velocity_inertial_mps.x".to_string(),
                value: self.velocity_inertial_mps.x,
            },
            TelemetryField {
                path: "velocity_inertial_mps.y".to_string(),
                value: self.velocity_inertial_mps.y,
            },
            TelemetryField {
                path: "velocity_inertial_mps.z".to_string(),
                value: self.velocity_inertial_mps.z,
            },
            TelemetryField {
                path: "has_orientation".to_string(),
                value: if self.has_orientation { 1.0 } else { 0.0 },
            },
        ]
    }
}
