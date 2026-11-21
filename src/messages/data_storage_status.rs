//! Storage-unit fill and partitions (upstream `DataStorageStatusMsgPayload`).

use serde::{Deserialize, Serialize};

use crate::telemetry::{TelemetryField, TelemetryMessage};

/// Stored data, capacity and net data rate of a storage unit, with the
/// amount held in each partition: upstream `DataStorageStatusMsg`.
///
/// Upstream's `std::vector`s are `Vec`s; `stored_data_name[i]` names
/// `stored_data_bits[i]`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DataStorageStatusMsg {
    /// Upstream `storageLevel` \[bit\]: data stored in the unit.
    pub storage_level_bits: f64,
    /// Upstream `storageCapacity` \[bit\]: largest amount the unit holds.
    pub storage_capacity_bits: f64,
    /// Upstream `currentNetBaud` \[bit/s\]: net rate at which data is written
    /// to (positive) or removed from (negative) the unit.
    pub current_net_baud_bps: f64,
    /// Upstream `storedDataName`: name of each partition.
    pub stored_data_name: Vec<String>,
    /// Upstream `storedData` \[bit\]: data held in each partition.
    pub stored_data_bits: Vec<f64>,
}

impl TelemetryMessage for DataStorageStatusMsg {
    /// The three scalars, then one column per partition,
    /// `stored_data_bits.0` onwards; the names are not columns.
    fn flatten(&self) -> Vec<TelemetryField> {
        let scalars = [
            ("storage_level_bits", self.storage_level_bits),
            ("storage_capacity_bits", self.storage_capacity_bits),
            ("current_net_baud_bps", self.current_net_baud_bps),
        ];
        scalars
            .into_iter()
            .map(|(path, value)| TelemetryField {
                path: path.to_string(),
                value,
            })
            .chain(
                self.stored_data_bits
                    .iter()
                    .enumerate()
                    .map(|(index, bits)| TelemetryField {
                        path: format!("stored_data_bits.{index}"),
                        value: *bits,
                    }),
            )
            .collect()
    }
}
