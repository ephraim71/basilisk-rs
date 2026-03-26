use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct SunEphemerisMsg {
    pub sun_position_inertial_m: Vector3<f64>,
    pub sun_velocity_inertial_mps: Vector3<f64>,
}

impl TelemetryMessage for SunEphemerisMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "sun_position_inertial_m.x".to_string(),
                value: self.sun_position_inertial_m.x,
            },
            TelemetryField {
                path: "sun_position_inertial_m.y".to_string(),
                value: self.sun_position_inertial_m.y,
            },
            TelemetryField {
                path: "sun_position_inertial_m.z".to_string(),
                value: self.sun_position_inertial_m.z,
            },
            TelemetryField {
                path: "sun_velocity_inertial_mps.x".to_string(),
                value: self.sun_velocity_inertial_mps.x,
            },
            TelemetryField {
                path: "sun_velocity_inertial_mps.y".to_string(),
                value: self.sun_velocity_inertial_mps.y,
            },
            TelemetryField {
                path: "sun_velocity_inertial_mps.z".to_string(),
                value: self.sun_velocity_inertial_mps.z,
            },
        ]
    }
}
