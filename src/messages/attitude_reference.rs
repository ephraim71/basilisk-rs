use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct AttitudeReferenceMsg {
    /// Reference-frame attitude relative to inertial, expressed as MRPs.
    pub sigma_rn: Vector3<f64>,
    /// Reference-frame angular velocity relative to inertial, in inertial components.
    pub omega_rn_n_radps: Vector3<f64>,
    /// Reference-frame inertial angular acceleration, in inertial components.
    pub domega_rn_n_radps2: Vector3<f64>,
}

impl TelemetryMessage for AttitudeReferenceMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "sigma_rn.x".to_string(),
                value: self.sigma_rn.x,
            },
            TelemetryField {
                path: "sigma_rn.y".to_string(),
                value: self.sigma_rn.y,
            },
            TelemetryField {
                path: "sigma_rn.z".to_string(),
                value: self.sigma_rn.z,
            },
            TelemetryField {
                path: "omega_rn_n_radps.x".to_string(),
                value: self.omega_rn_n_radps.x,
            },
            TelemetryField {
                path: "omega_rn_n_radps.y".to_string(),
                value: self.omega_rn_n_radps.y,
            },
            TelemetryField {
                path: "omega_rn_n_radps.z".to_string(),
                value: self.omega_rn_n_radps.z,
            },
            TelemetryField {
                path: "domega_rn_n_radps2.x".to_string(),
                value: self.domega_rn_n_radps2.x,
            },
            TelemetryField {
                path: "domega_rn_n_radps2.y".to_string(),
                value: self.domega_rn_n_radps2.y,
            },
            TelemetryField {
                path: "domega_rn_n_radps2.z".to_string(),
                value: self.domega_rn_n_radps2.z,
            },
        ]
    }
}
