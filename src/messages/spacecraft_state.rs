use nalgebra::{Quaternion, UnitQuaternion, Vector3};

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug)]
pub struct SpacecraftStateMsg {
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    pub sigma_bn: Vector3<f64>,
    pub omega_radps: Vector3<f64>,
}

impl Default for SpacecraftStateMsg {
    fn default() -> Self {
        Self {
            position_m: Vector3::zeros(),
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
            omega_radps: Vector3::zeros(),
        }
    }
}

impl SpacecraftStateMsg {
    pub fn body_to_inertial(&self) -> UnitQuaternion<f64> {
        let sigma_squared = self.sigma_bn.norm_squared();
        let denom = 1.0 + sigma_squared;
        let q_scalar = (1.0 - sigma_squared) / denom;
        let q_vector = 2.0 * self.sigma_bn / denom;
        UnitQuaternion::new_normalize(Quaternion::new(
            q_scalar, q_vector.x, q_vector.y, q_vector.z,
        ))
        .inverse()
    }

    pub fn inertial_to_body(&self) -> UnitQuaternion<f64> {
        self.body_to_inertial().inverse()
    }
}

impl TelemetryMessage for SpacecraftStateMsg {
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
                path: "sigma_bn.x".to_string(),
                value: self.sigma_bn.x,
            },
            TelemetryField {
                path: "sigma_bn.y".to_string(),
                value: self.sigma_bn.y,
            },
            TelemetryField {
                path: "sigma_bn.z".to_string(),
                value: self.sigma_bn.z,
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
