use nalgebra::{UnitQuaternion, Vector3};

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug)]
pub struct SpacecraftStateMsg {
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    pub attitude_b_to_i: UnitQuaternion<f64>,
    pub omega_radps: Vector3<f64>,
}

impl Default for SpacecraftStateMsg {
    fn default() -> Self {
        Self {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            attitude_b_to_i: UnitQuaternion::identity(),
            omega_radps: Vector3::zeros(),
        }
    }
}

impl TelemetryMessage for SpacecraftStateMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        let q = self.attitude_b_to_i.quaternion();
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
                path: "attitude_b_to_i.x".to_string(),
                value: q.i,
            },
            TelemetryField {
                path: "attitude_b_to_i.y".to_string(),
                value: q.j,
            },
            TelemetryField {
                path: "attitude_b_to_i.z".to_string(),
                value: q.k,
            },
            TelemetryField {
                path: "attitude_b_to_i.w".to_string(),
                value: q.w,
            },
            TelemetryField {
                path: "omega_radps.x".to_string(),
                value: self.omega_radps.x,
            },
            TelemetryField {
                path: "omega_radps.y".to_string(),
                value: self.omega_radps.y,
            },
            TelemetryField {
                path: "omega_radps.z".to_string(),
                value: self.omega_radps.z,
            },
        ]
    }
}
