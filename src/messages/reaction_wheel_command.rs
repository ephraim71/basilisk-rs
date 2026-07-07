use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ReactionWheelCommandMsg {
    pub motor_torque_nm: f64,
}

impl TelemetryMessage for ReactionWheelCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "motor_torque_nm".to_string(),
            value: self.motor_torque_nm,
        }]
    }
}
