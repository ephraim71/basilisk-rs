use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Body-frame torque command (upstream `CmdTorqueBodyMsgPayload`).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct BodyTorqueCommandMsg {
    pub torque_request_body_nm: Vector3<f64>,
    /// Sim time \[ns\] of the update that wrote the request: the stand-in
    /// for upstream's message-header `timeWritten`, which
    /// `thrMomentumDumping` uses to tell a new momentum-change request
    /// (`deltaHInMsg`) from the one it last acted on, even when the two are
    /// equal. `thrMomentumManagement` stamps its requests; producers of a
    /// continuous torque command, whose readers take every write as current,
    /// leave it 0 (upstream's never-written time). Like a header field, it is
    /// not part of the telemetry.
    #[serde(default)]
    pub command_time_nanos: u64,
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
