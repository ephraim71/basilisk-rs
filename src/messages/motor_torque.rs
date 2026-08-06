use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

/// Effective torque applied by one motor.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MotorTorqueMsg {
    pub motor_torque_nm: f64,
}

impl TelemetryMessage for MotorTorqueMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "motor_torque_nm".to_string(),
            value: self.motor_torque_nm,
        }]
    }
}
