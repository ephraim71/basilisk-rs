use crate::telemetry::{TelemetryField, TelemetryMessage};

use super::MAX_EFF_COUNT;

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

impl MtbArrayCommandMsg {
    pub fn from_active(values: &[f64]) -> Self {
        assert!(
            values.len() <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} MTB commands are supported"
        );
        let mut message = Self::default();
        message.dipole_cmds_am2[..values.len()].copy_from_slice(values);
        message
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
