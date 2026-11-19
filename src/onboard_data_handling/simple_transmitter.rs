//! Packetized data sink: port of upstream Basilisk
//! `simulation/onboardDataHandling/transmitter/simpleTransmitter`.
//!
//! The transmitter downlinks, a packet at a time, from the partition holding
//! the most data in the last connected storage unit. It publishes
//! [`DataNodeBase::node_baud_rate_bps`] under that partition's name once the
//! partition holds at least one packet, keeps the partition until the packet
//! is sent, and publishes zero when there is nothing to send. By upstream's
//! convention the baud rate and [`SimpleTransmitterModel::packet_size_bits`]
//! of a sink are negative. See [`super::data_node_base`] for the device
//! command and the shared behaviour.
//!
//! # Differences from upstream
//!
//! * Upstream reads `storageUnitMsgs.back()` even with no storage unit
//!   connected (undefined behaviour); here no storage unit means no data to
//!   send.
//! * Upstream's `numBuffers` and `currentIndex` members are never read and
//!   are not ported.

use super::data_node_base::{DataNodeBase, DataNodeFields, DataNodeModel};
use crate::messages::{DATA_NAME_SIZE, DataNodeUsageMsg, DataStorageStatusMsg, Input};

/// The model of [`SimpleTransmitter`]: upstream `SimpleTransmitter`'s
/// members and `evaluateDataModel`.
#[derive(Clone, Debug, Default)]
pub struct SimpleTransmitterModel {
    /// Upstream `packetSize` \[bit\]: size of a downlink packet, negative by
    /// the sink convention. Downlinking starts only once the partition holds
    /// at least `-packet_size_bits`.
    pub packet_size_bits: f64,
    /// Upstream `storageUnitInMsgs`: storage units the transmitter reads,
    /// added by [`SimpleTransmitter::add_storage_unit_to_transmitter`]. Only
    /// the last is downlinked from, as upstream.
    pub storage_unit_in_msgs: Vec<Input<DataStorageStatusMsg>>,
    /// Upstream `storageUnitMsgs`: the storage statuses read this update.
    storage_unit_msgs: Vec<DataStorageStatusMsg>,
    /// Upstream `packetTransmitted` \[bit\]: amount of the current packet sent
    /// (negative).
    packet_transmitted_bits: f64,
    /// Upstream `previousTime` \[s\]: time of the last evaluation.
    previous_time_s: f64,
}

/// A data sink that downlinks packets from a storage unit: upstream
/// `SimpleTransmitter`.
pub type SimpleTransmitter = DataNodeBase<SimpleTransmitterModel>;

impl SimpleTransmitter {
    /// A transmitter that is on, with a zero baud rate and packet size and no
    /// storage unit.
    pub fn new(name: impl Into<String>) -> Self {
        Self::with_model(name, SimpleTransmitterModel::default())
    }

    /// Upstream `addStorageUnitToTransmitter`: adds a storage-status input
    /// and returns it for connecting.
    pub fn add_storage_unit_to_transmitter(&mut self) -> &mut Input<DataStorageStatusMsg> {
        self.model.storage_unit_in_msgs.push(Input::default());
        self.model
            .storage_unit_in_msgs
            .last_mut()
            .expect("just pushed a storage input")
    }
}

impl SimpleTransmitterModel {
    /// Upstream `getMaxIndex`: the partition of the last storage unit holding
    /// the most data, the first of equals.
    fn max_index(&self) -> Option<usize> {
        let storage = self.storage_unit_msgs.last()?;
        let mut max_value = -1.0;
        let mut max_index = None;
        for (index, &bits) in storage.stored_data_bits.iter().enumerate() {
            if bits > max_value {
                max_value = bits;
                max_index = Some(index);
            }
        }
        max_index
    }
}

impl DataNodeModel for SimpleTransmitterModel {
    /// # Panics
    ///
    /// If the selected partition's name has [`DATA_NAME_SIZE`] characters or
    /// more (upstream `bskError`).
    fn evaluate_data_model(
        &mut self,
        node: DataNodeFields<'_>,
        usage: &mut DataNodeUsageMsg,
        current_time_s: f64,
    ) {
        let timestep_s = current_time_s - self.previous_time_s;
        usage.baud_rate_bps = node.node_baud_rate_bps;
        let sent_this_step = node.node_baud_rate_bps * timestep_s;
        match self.max_index() {
            Some(index) if self.packet_transmitted_bits == 0.0 => {
                // No packet under way: start one from the fullest partition.
                let storage = self
                    .storage_unit_msgs
                    .last()
                    .expect("a partition was found");
                let name = &storage.stored_data_name[index];
                assert!(
                    name.len() < DATA_NAME_SIZE,
                    "SimpleTransmitter: storedDataName is {} characters, but nodeDataName \
                     supports at most {} characters.",
                    name.len(),
                    DATA_NAME_SIZE - 1
                );
                node.node_data_name.clone_from(name);
                usage.data_name.clone_from(name);
                self.packet_transmitted_bits += sent_this_step;
                // Wait until the partition holds a whole packet.
                if storage.stored_data_bits[index] < -self.packet_size_bits {
                    usage.baud_rate_bps = 0.0;
                    self.packet_transmitted_bits = 0.0;
                }
            }
            Some(_) => {
                usage.data_name.clone_from(node.node_data_name);
                self.packet_transmitted_bits += sent_this_step;
                // Both are negative: the packet is complete.
                if self.packet_transmitted_bits <= self.packet_size_bits {
                    self.packet_transmitted_bits = 0.0;
                }
            }
            None => {
                usage.baud_rate_bps = 0.0;
                self.packet_transmitted_bits = 0.0;
            }
        }
        self.previous_time_s = current_time_s;
    }

    /// Upstream `customReadMessages`.
    fn custom_read_messages(&mut self) -> bool {
        self.storage_unit_msgs.clear();
        if self.storage_unit_in_msgs.is_empty() {
            log::info!("SimpleTransmitter: Data storage has no data node messages to read.");
        }
        self.storage_unit_msgs
            .extend(self.storage_unit_in_msgs.iter().map(Input::read));
        true
    }
}

#[cfg(test)]
mod tests {
    //! Ports both cases of upstream
    //! `transmitter/_UnitTest/test_simpleTransmitter.py` with their
    //! tolerance, with the instrument and partitioned storage unit upstream
    //! uses.

    use super::SimpleTransmitter;
    use crate::Module;
    use crate::onboard_data_handling::partitioned_storage_unit::PartitionedStorageUnit;
    use crate::onboard_data_handling::simple_instrument::SimpleInstrument;
    use crate::onboard_data_handling::test_support::{TaskClock, context};

    /// Upstream `testProcessRate`, `macros.sec2nano(0.5)`.
    const STEP_NANOS: u64 = 500_000_000;
    /// Upstream `accuracy`.
    const ACCURACY: f64 = 1e-16;

    /// Upstream's transmitter, instrument and storage chain, run to
    /// `stop_nanos` in upstream's task order; returns the transmitter's baud
    /// rates.
    fn run_chain(
        transmitter: &mut SimpleTransmitter,
        instrument_baud_bps: f64,
        stop_nanos: u64,
    ) -> Vec<f64> {
        let mut instrument = SimpleInstrument::new("instrument1");
        instrument.node_baud_rate_bps = instrument_baud_bps;
        instrument.node_data_name = "Instrument 1".to_string();
        let mut storage = PartitionedStorageUnit::new("dataMonitor");
        storage.storage_capacity_bits = 8_000_000_000;
        instrument
            .node_data_out_msg
            .connect_to(storage.add_data_node_to_model());
        transmitter
            .node_data_out_msg
            .connect_to(storage.add_data_node_to_model());
        storage
            .storage_unit_data_out_msg
            .connect_to(transmitter.add_storage_unit_to_transmitter());

        transmitter.init();
        instrument.init();
        storage.init();
        let mut baud = Vec::new();
        for tick in TaskClock::new(STEP_NANOS).run_until(stop_nanos) {
            transmitter.update(&context(tick));
            instrument.update(&context(tick));
            storage.update(&context(tick));
            baud.push(transmitter.node_data_out_msg.read().baud_rate_bps);
        }
        baud
    }

    /// Upstream `checkDefault`: the transmitter waits three updates for a
    /// packet's worth of data, then transmits.
    #[test]
    fn check_default() {
        let mut transmitter = SimpleTransmitter::new("transmitter");
        transmitter.node_baud_rate_bps = 9600.0;
        transmitter.model.packet_size_bits = -9600.0;
        let baud = run_chain(&mut transmitter, 9600.0, 3_000_000_000);
        let expected = [0.0, 0.0, 0.0, 9600.0, 9600.0, 9600.0, 9600.0];
        assert_eq!(baud.len(), expected.len());
        for (value, expected) in baud.iter().zip(expected) {
            assert!((value - expected).abs() <= ACCURACY, "{baud:?}");
        }
    }

    /// Upstream `checkStatus`: a transmitter that is off publishes zero.
    #[test]
    fn check_status() {
        let mut transmitter = SimpleTransmitter::new("transmitter");
        transmitter.node_baud_rate_bps = 9600.0;
        transmitter.model.packet_size_bits = -9600.0;
        transmitter.data_status = 0;
        let baud = run_chain(&mut transmitter, 1200.0, 1_000_000_000);
        assert_eq!(baud.len(), 3);
        for value in baud {
            assert!(value.abs() <= ACCURACY);
        }
    }

    #[test]
    fn no_storage_unit_means_nothing_to_send() {
        let mut transmitter = SimpleTransmitter::new("transmitter");
        transmitter.node_baud_rate_bps = -9600.0;
        transmitter.init();
        transmitter.update(&context(0));
        transmitter.update(&context(STEP_NANOS));
        assert_eq!(transmitter.node_data_out_msg.read().baud_rate_bps, 0.0);
    }
}
