use crate::telemetry::{TelemetryField, TelemetryMessage};

use super::MAX_EFF_COUNT;

/// Aggregate reaction-wheel speed and angle state.
#[derive(Clone, Debug, PartialEq)]
pub struct RwSpeedMsg {
    pub wheel_speeds_radps: [f64; MAX_EFF_COUNT],
    pub wheel_angles_rad: [f64; MAX_EFF_COUNT],
}

impl Default for RwSpeedMsg {
    fn default() -> Self {
        Self {
            wheel_speeds_radps: [0.0; MAX_EFF_COUNT],
            wheel_angles_rad: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl TelemetryMessage for RwSpeedMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        let mut fields =
            Vec::with_capacity(self.wheel_speeds_radps.len() + self.wheel_angles_rad.len());
        fields.extend(
            self.wheel_speeds_radps
                .iter()
                .enumerate()
                .map(|(index, value)| TelemetryField {
                    path: format!("wheel_speeds_radps.{index}"),
                    value: *value,
                }),
        );
        fields.extend(
            self.wheel_angles_rad
                .iter()
                .enumerate()
                .map(|(index, value)| TelemetryField {
                    path: format!("wheel_angles_rad.{index}"),
                    value: *value,
                }),
        );
        fields
    }
}
