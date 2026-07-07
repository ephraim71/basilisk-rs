use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ArrayMotorTorqueMsg {
    pub motor_torque_nm: Vec<f64>,
}

impl ArrayMotorTorqueMsg {
    pub fn first_torque_nm(&self) -> f64 {
        self.motor_torque_nm.first().copied().unwrap_or(0.0)
    }
}

impl TelemetryMessage for ArrayMotorTorqueMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        self.motor_torque_nm
            .iter()
            .enumerate()
            .map(|(index, value)| TelemetryField {
                path: format!("motor_torque_nm.{index}"),
                value: *value,
            })
            .collect()
    }
}
