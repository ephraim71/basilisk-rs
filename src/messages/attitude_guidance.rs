use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct AttitudeGuidanceMsg {
    pub sigma_br: Vector3<f64>,
    pub omega_br_b_radps: Vector3<f64>,
    pub omega_rn_b_radps: Vector3<f64>,
    pub domega_rn_b_radps2: Vector3<f64>,
}

impl TelemetryMessage for AttitudeGuidanceMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "sigma_br.x".to_string(),
                value: self.sigma_br.x,
            },
            TelemetryField {
                path: "sigma_br.y".to_string(),
                value: self.sigma_br.y,
            },
            TelemetryField {
                path: "sigma_br.z".to_string(),
                value: self.sigma_br.z,
            },
            TelemetryField {
                path: "omega_br_b_radps.x".to_string(),
                value: self.omega_br_b_radps.x,
            },
            TelemetryField {
                path: "omega_br_b_radps.y".to_string(),
                value: self.omega_br_b_radps.y,
            },
            TelemetryField {
                path: "omega_br_b_radps.z".to_string(),
                value: self.omega_br_b_radps.z,
            },
            TelemetryField {
                path: "omega_rn_b_radps.x".to_string(),
                value: self.omega_rn_b_radps.x,
            },
            TelemetryField {
                path: "omega_rn_b_radps.y".to_string(),
                value: self.omega_rn_b_radps.y,
            },
            TelemetryField {
                path: "omega_rn_b_radps.z".to_string(),
                value: self.omega_rn_b_radps.z,
            },
            TelemetryField {
                path: "domega_rn_b_radps2.x".to_string(),
                value: self.domega_rn_b_radps2.x,
            },
            TelemetryField {
                path: "domega_rn_b_radps2.y".to_string(),
                value: self.domega_rn_b_radps2.y,
            },
            TelemetryField {
                path: "domega_rn_b_radps2.z".to_string(),
                value: self.domega_rn_b_radps2.z,
            },
        ]
    }
}
