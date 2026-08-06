use crate::telemetry::{TelemetryField, TelemetryMessage};
use serde::{Deserialize, Serialize};

/// Effective magnetic dipole commanded along one configured actuator axis.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MagneticDipoleCommandMsg {
    pub dipole_moment_am2: f64,
}

impl TelemetryMessage for MagneticDipoleCommandMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "dipole_moment_am2".to_string(),
            value: self.dipole_moment_am2,
        }]
    }
}
