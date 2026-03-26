use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct MtbCommandMsg {
    pub dipole_cmd_am2: f64,
}

impl TelemetryMessage for MtbCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "dipole_cmd_am2".to_string(),
            value: self.dipole_cmd_am2,
        }]
    }
}
