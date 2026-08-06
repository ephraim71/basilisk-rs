use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;

/// Aggregate reaction-wheel motor voltage command.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayMotorVoltageMsg {
    #[serde(with = "super::big_array")]
    pub voltage_v: [f64; MAX_EFF_COUNT],
}

impl Default for ArrayMotorVoltageMsg {
    fn default() -> Self {
        Self {
            voltage_v: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl TelemetryMessage for ArrayMotorVoltageMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        self.voltage_v
            .iter()
            .enumerate()
            .map(|(index, value)| TelemetryField {
                path: format!("voltage_v.{index}"),
                value: *value,
            })
            .collect()
    }
}
