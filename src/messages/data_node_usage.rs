//! Data produced or consumed by a data node (upstream
//! `DataNodeUsageMsgPayload`).

use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Data a module generates or sinks, under the name of the partition it
/// belongs to: upstream `DataNodeUsageMsg`.
///
/// Upstream's `char dataName[128]` is a `String` here; the modules that
/// produce one reject names of [`super::DATA_NAME_SIZE`] characters or more,
/// as upstream does.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DataNodeUsageMsg {
    /// Upstream `dataName`: name of the data, which a storage unit files it
    /// under. Empty for a node that names no partition.
    pub data_name: String,
    /// Upstream `baudRate` \[bit/s\]: positive for a data generator,
    /// negative for a data sink.
    pub baud_rate_bps: f64,
}

impl TelemetryMessage for DataNodeUsageMsg {
    /// The baud rate; the name is not a column.
    fn flatten(&self) -> Vec<TelemetryField> {
        vec![TelemetryField {
            path: "baud_rate_bps".to_string(),
            value: self.baud_rate_bps,
        }]
    }
}
