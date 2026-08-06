use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayMotorTorqueMsg {
    #[serde(with = "super::big_array")]
    pub motor_torque_nm: [f64; MAX_EFF_COUNT],
}

impl Default for ArrayMotorTorqueMsg {
    fn default() -> Self {
        Self {
            motor_torque_nm: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl ArrayMotorTorqueMsg {
    pub fn from_active(values: &[f64]) -> Self {
        assert!(
            values.len() <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} motor torques are supported"
        );
        let mut message = Self::default();
        message.motor_torque_nm[..values.len()].copy_from_slice(values);
        message
    }

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
