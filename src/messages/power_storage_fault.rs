use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug)]
pub struct PowerStorageFaultMsg {
    pub fault_capacity_ratio: f64,
}

impl Default for PowerStorageFaultMsg {
    fn default() -> Self {
        Self {
            fault_capacity_ratio: 1.0,
        }
    }
}

impl TelemetryMessage for PowerStorageFaultMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "fault_capacity_ratio".to_string(),
            value: self.fault_capacity_ratio,
        }]
    }
}
