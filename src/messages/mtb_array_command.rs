use crate::telemetry::{TelemetryField, TelemetryMessage};

use super::MAX_EFF_COUNT;

/// Aggregate magnetic-torque-bar dipole commands.
#[derive(Clone, Debug, PartialEq)]
pub struct MtbArrayCommandMsg {
    pub dipole_cmds_am2: [f64; MAX_EFF_COUNT],
}

impl Default for MtbArrayCommandMsg {
    fn default() -> Self {
        Self {
            dipole_cmds_am2: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl TelemetryMessage for MtbArrayCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        self.dipole_cmds_am2
            .iter()
            .enumerate()
            .map(|(index, value)| TelemetryField {
                path: format!("dipole_cmds_am2.{index}"),
                value: *value,
            })
            .collect()
    }
}
