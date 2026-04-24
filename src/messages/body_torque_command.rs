use nalgebra::Vector3;

use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct BodyTorqueCommandMsg {
    pub torque_request_body_nm: Vector3<f64>,
}

impl TelemetryMessage for BodyTorqueCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "torque_request_body_nm.x".to_string(),
                value: self.torque_request_body_nm.x,
            },
            TelemetryField {
                path: "torque_request_body_nm.y".to_string(),
                value: self.torque_request_body_nm.y,
            },
            TelemetryField {
                path: "torque_request_body_nm.z".to_string(),
                value: self.torque_request_body_nm.z,
            },
        ]
    }
}
