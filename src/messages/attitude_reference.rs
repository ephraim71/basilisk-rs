use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct AttitudeReferenceMsg {
    pub sigma_bn: Vector3<f64>,
    pub omega_bn_n_radps: Vector3<f64>,
}

impl TelemetryMessage for AttitudeReferenceMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
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
                path: "omega_bn_n_radps.x".to_string(),
                value: self.omega_bn_n_radps.x,
            },
            TelemetryField {
                path: "omega_bn_n_radps.y".to_string(),
                value: self.omega_bn_n_radps.y,
            },
            TelemetryField {
                path: "omega_bn_n_radps.z".to_string(),
                value: self.omega_bn_n_radps.z,
            },
        ]
    }
}
