//! Translational reference state (upstream `TransRefMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Reference position, velocity and acceleration of a spacecraft, inertial
/// components: upstream `TransRefMsg`.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct TranslationReferenceMsg {
    /// Upstream `r_RN_N` \[m\]: reference position, inertial components.
    pub position_m: Vector3<f64>,
    /// Upstream `v_RN_N` \[m/s\]: reference velocity, inertial components.
    pub velocity_mps: Vector3<f64>,
    /// Upstream `a_RN_N` \[m/s^2\]: total reference acceleration, inertial
    /// components.
    pub acceleration_mps2: Vector3<f64>,
}

impl TelemetryMessage for TranslationReferenceMsg {
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
                path: "acceleration_mps2.x".to_string(),
                value: self.acceleration_mps2.x,
            },
            TelemetryField {
                path: "acceleration_mps2.y".to_string(),
                value: self.acceleration_mps2.y,
            },
            TelemetryField {
                path: "acceleration_mps2.z".to_string(),
                value: self.acceleration_mps2.z,
            },
        ]
    }
}
