//! The onboard data system shared by upstream `scenarioDataDemo` and
//! `scenarioGroundDownlink`: two `simpleInstrument`s named `Instrument 1`
//! and `Instrument 2`, and a transmitter, all feeding a
//! `partitionedStorageUnit` (8e9 bit, one partition per instrument) and a
//! `simpleStorageUnit` (1e5 bit). The transmitter, which differs between the
//! two scenarios, downlinks from the partitioned unit.

#![allow(dead_code)]

use basilisk_rs::messages::{DataNodeUsageMsg, Output};
use basilisk_rs::onboard_data_handling::partitioned_storage_unit::PartitionedStorageUnit;
use basilisk_rs::onboard_data_handling::simple_instrument::SimpleInstrument;
use basilisk_rs::onboard_data_handling::simple_storage_unit::SimpleStorageUnit;
use basilisk_rs::telemetry::CsvSourceConfig;

use super::common::array_columns;

/// Upstream `dataMonitor.storageCapacity` \[bit\] (1 GB).
pub const PARTITIONED_CAPACITY_BITS: i64 = 8_000_000_000;
/// Upstream `dataMonitor2.storageCapacity` \[bit\].
pub const SIMPLE_CAPACITY_BITS: i64 = 100_000;
/// Upstream `nodeDataName` of the two instruments, also the partitions.
pub const PARTITIONS: [&str; 2] = ["Instrument 1", "Instrument 2"];

/// The instruments and storage units.
pub struct DataSystem {
    /// Upstream `instrument` and `instrument2`.
    pub instruments: [SimpleInstrument; 2],
    /// Upstream `dataMonitor`.
    pub partitioned: PartitionedStorageUnit,
    /// Upstream `dataMonitor2`.
    pub simple: SimpleStorageUnit,
}

impl DataSystem {
    /// Instruments generating `instrument_baud_bps` each, with both storage
    /// units taking them and the transmitter `transmitter_out`, in
    /// upstream's `addDataNodeToModel` order.
    pub fn new(instrument_baud_bps: f64, transmitter_out: &Output<DataNodeUsageMsg>) -> Self {
        let instruments = [
            ("instrument1", PARTITIONS[0]),
            ("instrument2", PARTITIONS[1]),
        ]
        .map(|(tag, name)| {
            let mut instrument = SimpleInstrument::new(tag);
            instrument.node_baud_rate_bps = instrument_baud_bps;
            instrument.node_data_name = name.to_string();
            instrument
        });
        let mut partitioned = PartitionedStorageUnit::new("dataMonitor");
        partitioned.storage_capacity_bits = PARTITIONED_CAPACITY_BITS;
        let mut simple = SimpleStorageUnit::new("dataMonitor2");
        simple.storage_capacity_bits = SIMPLE_CAPACITY_BITS;
        for node in instruments
            .iter()
            .map(|instrument| &instrument.node_data_out_msg)
            .chain([transmitter_out])
        {
            node.connect_to(partitioned.add_data_node_to_model());
            node.connect_to(simple.add_data_node_to_model());
        }
        for name in PARTITIONS {
            partitioned.add_partition(name);
        }
        Self {
            instruments,
            partitioned,
            simple,
        }
    }
}

/// `storageLevel_bits`, `currentNetBaud_bps` and `storedData_bits_<i>` of a
/// storage-status message with `partitions` partitions: upstream's
/// `storageLevel`, `currentNetBaud` and `storedData`.
pub fn storage_columns(partitions: usize) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        [
            (
                "storage_level_bits".to_string(),
                "storageLevel_bits".to_string(),
            ),
            (
                "current_net_baud_bps".to_string(),
                "currentNetBaud_bps".to_string(),
            ),
        ]
        .into_iter()
        .chain(array_columns(
            "stored_data_bits",
            "storedData_bits",
            partitions,
        )),
    )
}
