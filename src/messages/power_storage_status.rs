use crate::telemetry::{TelemetryField, TelemetryMessage};

#[derive(Clone, Debug, Default)]
pub struct PowerStorageStatusMsg {
    pub storage_level_j: f64,
    pub storage_capacity_j: f64,
    pub current_net_power_w: f64,
}

impl TelemetryMessage for PowerStorageStatusMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![
            TelemetryField {
                path: "storage_level_j".to_string(),
                value: self.storage_level_j,
            },
            TelemetryField {
                path: "storage_capacity_j".to_string(),
                value: self.storage_capacity_j,
            },
            TelemetryField {
                path: "current_net_power_w".to_string(),
                value: self.current_net_power_w,
            },
        ]
    }
}
