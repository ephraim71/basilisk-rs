use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ThrusterCommandMsg {
    pub on_time_s: f64,
    /// Sim time \[ns\] of the update that issued the command: the stand-in for
    /// upstream's message-header `timeWritten`, which a thruster uses to tell a
    /// new command from a repeat of the last one. Producers stamp every write
    /// with their update time; see `ThrusterDynamicEffector::pre_integration`.
    pub command_time_nanos: u64,
}

impl TelemetryMessage for ThrusterCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "on_time_s".to_string(),
                value: self.on_time_s,
            },
            TelemetryField {
                path: "command_time_nanos".to_string(),
                value: self.command_time_nanos as f64,
            },
        ]
    }
}
