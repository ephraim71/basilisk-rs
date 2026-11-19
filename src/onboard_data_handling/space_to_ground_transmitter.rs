//! Access-gated packetized downlink: port of upstream Basilisk
//! `simulation/onboardDataHandling/spaceToGroundTransmitter`.
//!
//! Like [`super::simple_transmitter`], the transmitter downlinks a packet at
//! a time from the fullest partition of the last connected storage unit, but
//! only while at least one ground location has access, and it keeps
//! downlinking a partition until the packet is sent or the partition is
//! empty. A step that would send more than the partition holds ends the
//! packet there; the rate stays the full baud rate, and the storage unit
//! clamps the partition at zero.
//! By upstream's convention the baud rate and
//! [`SpaceToGroundTransmitterModel::packet_size_bits`] of a sink are
//! negative. See [`super::data_node_base`] for the device command and the
//! shared behaviour.
//!
//! # Differences from upstream
//!
//! * Upstream reads `storageUnitMsgsBuffer.back()` even with no storage unit
//!   connected (undefined behaviour); here no storage unit means no data to
//!   send.
//! * Upstream's `numBuffers` and `hasAccess` members are never read and are
//!   not ported.

use super::data_node_base::{DataNodeBase, DataNodeFields, DataNodeModel};
use crate::messages::{AccessMsg, DATA_NAME_SIZE, DataNodeUsageMsg, DataStorageStatusMsg, Input};

/// The model of [`SpaceToGroundTransmitter`]: upstream
/// `SpaceToGroundTransmitter`'s members and `evaluateDataModel`.
#[derive(Clone, Debug, Default)]
pub struct SpaceToGroundTransmitterModel {
    /// Upstream `packetSize` \[bit\]: size of a downlink packet, negative by
    /// the sink convention. A partition is started only once it holds at
    /// least `-packet_size_bits`.
    pub packet_size_bits: f64,
    /// Upstream `storageUnitInMsgs`: storage units the transmitter reads,
    /// added by [`SpaceToGroundTransmitter::add_storage_unit_to_transmitter`].
    /// Only the last is downlinked from, as upstream.
    pub storage_unit_in_msgs: Vec<Input<DataStorageStatusMsg>>,
    /// Upstream `groundLocationAccessInMsgs`: access to each ground location,
    /// added by [`SpaceToGroundTransmitter::add_access_msg_to_transmitter`].
    pub ground_location_access_in_msgs: Vec<Input<AccessMsg>>,
    /// Upstream `storageUnitMsgsBuffer`: the storage statuses read this
    /// update.
    storage_unit_msgs: Vec<DataStorageStatusMsg>,
    /// Upstream `groundLocationAccessMsgs`: the accesses read this update.
    ground_location_access_msgs: Vec<AccessMsg>,
    /// Upstream `packetTransmitted` \[bit\]: amount of the current packet sent
    /// (negative).
    packet_transmitted_bits: f64,
    /// Upstream `previousTime` \[s\]: time of the last evaluation.
    previous_time_s: f64,
}

/// A data sink that downlinks packets while a ground location has access:
/// upstream `SpaceToGroundTransmitter`.
pub type SpaceToGroundTransmitter = DataNodeBase<SpaceToGroundTransmitterModel>;

impl SpaceToGroundTransmitter {
    /// A transmitter that is on, with a zero baud rate and packet size, no
    /// storage unit and no ground location.
    pub fn new(name: impl Into<String>) -> Self {
        Self::with_model(name, SpaceToGroundTransmitterModel::default())
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

    /// Upstream `addAccessMsgToTransmitter`: adds a ground-location access
    /// input and returns it for connecting.
    pub fn add_access_msg_to_transmitter(&mut self) -> &mut Input<AccessMsg> {
        self.model
            .ground_location_access_in_msgs
            .push(Input::default());
        self.model
            .ground_location_access_in_msgs
            .last_mut()
            .expect("just pushed an access input")
    }
}

impl DataNodeModel for SpaceToGroundTransmitterModel {
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
        let empty = DataStorageStatusMsg::default();
        let storage = self.storage_unit_msgs.last().unwrap_or(&empty);
        let mut max_value = -1.0;
        let mut max_index = None;

        // Both are negative: the packet is complete.
        if self.packet_transmitted_bits <= self.packet_size_bits {
            self.packet_transmitted_bits = 0.0;
        }
        // A packet under way continues from its partition, unless it is empty.
        if self.packet_transmitted_bits != 0.0 {
            for (index, name) in storage.stored_data_name.iter().enumerate() {
                if name == node.node_data_name {
                    max_value = storage.stored_data_bits[index];
                    max_index = Some(index);
                }
            }
            if max_value <= 0.0 {
                max_index = None;
                self.packet_transmitted_bits = 0.0;
                usage.baud_rate_bps = 0.0;
            }
        }
        // No packet under way: pick the fullest partition.
        if self.packet_transmitted_bits == 0.0 {
            for (index, &bits) in storage.stored_data_bits.iter().enumerate() {
                if bits > max_value {
                    max_value = bits;
                    max_index = Some(index);
                }
            }
        }

        let has_access = self
            .ground_location_access_msgs
            .iter()
            .any(|access| access.has_access);
        match max_index {
            Some(index) if has_access => {
                if self.packet_transmitted_bits == 0.0 {
                    let name = &storage.stored_data_name[index];
                    assert!(
                        name.len() < DATA_NAME_SIZE,
                        "SpaceToGroundTransmitter: storedDataName is {} characters, but \
                         nodeDataName supports at most {} characters.",
                        name.len(),
                        DATA_NAME_SIZE - 1
                    );
                    node.node_data_name.clone_from(name);
                    usage.data_name.clone_from(name);
                    if max_value < -self.packet_size_bits {
                        // Wait until the partition holds a whole packet.
                        usage.baud_rate_bps = 0.0;
                        self.packet_transmitted_bits = 0.0;
                    } else if max_value + sent_this_step < 0.0 {
                        // The step would empty the partition: end the packet.
                        self.packet_transmitted_bits = -max_value;
                    } else {
                        self.packet_transmitted_bits += sent_this_step;
                    }
                } else {
                    usage.data_name.clone_from(node.node_data_name);
                    if max_value + sent_this_step < 0.0 {
                        self.packet_transmitted_bits = -max_value;
                    } else {
                        self.packet_transmitted_bits += sent_this_step;
                    }
                }
            }
            // No data, or no access: nothing can be sent.
            _ => {
                usage.baud_rate_bps = 0.0;
                self.packet_transmitted_bits = 0.0;
            }
        }
        self.previous_time_s = current_time_s;
    }

    /// Upstream `customReadMessages`.
    fn custom_read_messages(&mut self) -> bool {
        self.storage_unit_msgs.clear();
        self.ground_location_access_msgs.clear();
        if self.storage_unit_in_msgs.is_empty() {
            log::info!("SpaceToGroundTransmitter: Data storage has no data node messages to read.");
        }
        self.storage_unit_msgs
            .extend(self.storage_unit_in_msgs.iter().map(Input::read));
        if self.ground_location_access_in_msgs.is_empty() {
            log::info!("SpaceToGroundTransmitter has no ground stations attached.");
        }
        self.ground_location_access_msgs
            .extend(self.ground_location_access_in_msgs.iter().map(Input::read));
        true
    }
}

#[cfg(test)]
mod tests {
    //! Ports every case of upstream
    //! `spaceToGroundTransmitter/_UnitTest/test_spaceToGroundTransmitter.py`
    //! with its expected values, with the instrument and storage units
    //! upstream uses, and adds an end-to-end chain: ground-location access
    //! feeding the transmitter, which drains a storage unit an instrument
    //! fills while the FSW `simpleInstrumentController` switches it.
    //!
    //! The modules are stepped by hand in upstream's task order, so the
    //! access can change between runs as upstream's tests rewrite it.

    use nalgebra::Vector3;

    use super::SpaceToGroundTransmitter;
    use crate::Module;
    use crate::environment::ground_location::{GroundLocation, GroundLocationConfig, SiteLocation};
    use crate::fsw_algorithms::sensor_interfaces::simple_instrument_controller::{
        SimpleInstrumentController, SimpleInstrumentControllerConfig,
    };
    use crate::messages::{AccessMsg, AttitudeGuidanceMsg, Output, SpacecraftStateMsg};
    use crate::onboard_data_handling::partitioned_storage_unit::PartitionedStorageUnit;
    use crate::onboard_data_handling::simple_instrument::SimpleInstrument;
    use crate::onboard_data_handling::simple_storage_unit::SimpleStorageUnit;
    use crate::onboard_data_handling::test_support::{TaskClock, context};
    use crate::utilities::astro_constants::REQ_EARTH;
    use crate::utilities::geodetic_conversion::Lla;

    /// Upstream `testProcessRate`, `macros.sec2nano(0.5)`.
    const STEP_NANOS: u64 = 500_000_000;
    const SECOND_NANOS: u64 = 1_000_000_000;
    /// Upstream's storage capacity, `8E9` bits.
    const CAPACITY_BITS: i64 = 8_000_000_000;

    fn access(has_access: bool) -> Output<AccessMsg> {
        Output::new(AccessMsg {
            has_access,
            ..AccessMsg::default()
        })
    }

    /// Upstream's transmitter at -9600 bit/s with `packet_size_bits`.
    fn transmitter(packet_size_bits: f64) -> SpaceToGroundTransmitter {
        let mut transmitter = SpaceToGroundTransmitter::new("transmitter");
        transmitter.node_baud_rate_bps = -9600.0;
        transmitter.model.packet_size_bits = packet_size_bits;
        transmitter
    }

    /// Upstream `test_baudRate`: the transmitter publishes its baud rate
    /// once the partition holds a packet, only while on and with access.
    #[test]
    fn baud_rate() {
        for (device_status, access_status) in [(0, false), (1, true)] {
            let expected = device_status as f64 * f64::from(u8::from(access_status));
            let access_1 = access(false);
            let access_2 = access(access_status);
            let mut transmitter = transmitter(-9600.0);
            transmitter.data_status = device_status;
            access_1.connect_to(transmitter.add_access_msg_to_transmitter());
            access_2.connect_to(transmitter.add_access_msg_to_transmitter());
            let mut instrument = SimpleInstrument::new("instrument1");
            instrument.node_baud_rate_bps = 9600.0;
            instrument.node_data_name = "Instrument 1".to_string();
            let mut storage = PartitionedStorageUnit::new("dataMonitor");
            storage.storage_capacity_bits = CAPACITY_BITS;
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
            for tick in TaskClock::new(STEP_NANOS).run_until(3 * SECOND_NANOS) {
                transmitter.update(&context(tick));
                instrument.update(&context(tick));
                storage.update(&context(tick));
                baud.push(transmitter.node_data_out_msg.read().baud_rate_bps);
            }

            let true_data = -9600.0;
            let mut expected_baud = vec![0.0; 3];
            expected_baud.extend([expected * true_data; 4]);
            assert_eq!(baud, expected_baud);
        }
    }

    /// Upstream `test_downlink`: packets drained from a simple storage
    /// unit, and data short of a packet left in place.
    #[test]
    fn downlink() {
        let cases = [
            (28800, 9600.0, 0.0, -9600.0),
            (9599, 9599.0, 9599.0, -9600.0),
            (9601, 1.0, 1.0, -9600.0),
            (3600, 3600.0, 3600.0, -9600.0),
            (3600, 0.0, 0.0, 0.0),
        ];
        for (data_0s, data_2s, data_5s, packet_size) in cases {
            let ground = access(true);
            let mut transmitter = transmitter(packet_size);
            ground.connect_to(transmitter.add_access_msg_to_transmitter());
            let mut storage = SimpleStorageUnit::new("dataMonitor");
            storage.storage_capacity_bits = CAPACITY_BITS;
            transmitter
                .node_data_out_msg
                .connect_to(storage.add_data_node_to_model());
            storage
                .storage_unit_data_out_msg
                .connect_to(transmitter.add_storage_unit_to_transmitter());
            storage.set_data_buffer(data_0s);

            transmitter.init();
            storage.init();
            let mut clock = TaskClock::new(STEP_NANOS);
            let mut run_until = |stop_nanos| {
                for tick in clock.run_until(stop_nanos) {
                    transmitter.update(&context(tick));
                    storage.update(&context(tick));
                }
                storage.storage_unit_data_out_msg.read().stored_data_bits
            };
            assert_eq!(run_until(2 * SECOND_NANOS), [data_2s], "at 2 s");
            assert_eq!(run_until(5 * SECOND_NANOS), [data_5s], "at 5 s");
        }
    }

    /// A transmitter with upstream's ground access draining partitions
    /// "P_1" to "P_3" of a partitioned unit.
    struct PartitionChain {
        ground: Output<AccessMsg>,
        transmitter: SpaceToGroundTransmitter,
        storage: PartitionedStorageUnit,
        clock: TaskClock,
    }

    impl PartitionChain {
        fn new(data_0s: [i64; 3], packet_size: f64) -> Self {
            let ground = access(true);
            let mut transmitter = transmitter(packet_size);
            ground.connect_to(transmitter.add_access_msg_to_transmitter());
            let mut storage = PartitionedStorageUnit::new("dataMonitor");
            storage.storage_capacity_bits = CAPACITY_BITS;
            transmitter
                .node_data_out_msg
                .connect_to(storage.add_data_node_to_model());
            storage
                .storage_unit_data_out_msg
                .connect_to(transmitter.add_storage_unit_to_transmitter());
            storage.set_data_buffer(&["P_1", "P_2", "P_3"], &data_0s);
            transmitter.init();
            storage.init();
            Self {
                ground,
                transmitter,
                storage,
                clock: TaskClock::new(STEP_NANOS),
            }
        }

        fn run_until(&mut self, stop_nanos: u64) -> Vec<f64> {
            for tick in self.clock.run_until(stop_nanos) {
                self.transmitter.update(&context(tick));
                self.storage.update(&context(tick));
            }
            self.storage
                .storage_unit_data_out_msg
                .read()
                .stored_data_bits
        }
    }

    /// Upstream `test_downlink_from_partition`.
    #[test]
    fn downlink_from_partition() {
        let cases = [
            (
                [0, 19200, 19200],
                [0.0, 9600.0, 9600.0],
                [0.0, 0.0, 0.0],
                -9600.0,
            ),
            ([0, 9600, 9600], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], -9600.0),
            (
                [0, 0, 9599],
                [0.0, 0.0, 9599.0],
                [0.0, 0.0, 9599.0],
                -9600.0,
            ),
            ([0, 0, 9601], [0.0, 0.0, 1.0], [0.0, 0.0, 1.0], -9600.0),
            (
                [1200, 2400, 3600],
                [1200.0, 2400.0, 3600.0],
                [1200.0, 2400.0, 3600.0],
                -9600.0,
            ),
            ([1200, 2400, 3600], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 0.0),
            ([1200, 2400, 3600], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], -1.0),
        ];
        for (data_0s, data_2s, data_5s, packet_size) in cases {
            let mut chain = PartitionChain::new(data_0s, packet_size);
            assert_eq!(chain.run_until(2 * SECOND_NANOS), data_2s, "at 2 s");
            assert_eq!(chain.run_until(5 * SECOND_NANOS), data_5s, "at 5 s");
        }
    }

    /// Upstream `test_downlink_with_disrupted_connection`: access lost from
    /// 2 s to 3 s abandons the packet under way.
    #[test]
    fn downlink_with_disrupted_connection() {
        let cases = [
            (
                [0, 0, 38400],
                [0.0, 0.0, 19200.0],
                [0.0, 0.0, 19200.0],
                -38400.0,
            ),
            ([0, 0, 38400], [0.0, 0.0, 19200.0], [0.0, 0.0, 0.0], 0.0),
        ];
        for (data_0s, data_2s, data_5s, packet_size) in cases {
            let mut chain = PartitionChain::new(data_0s, packet_size);
            assert_eq!(chain.run_until(2 * SECOND_NANOS), data_2s, "at 2 s");
            chain.ground.write(AccessMsg::default());
            chain.run_until(3 * SECOND_NANOS);
            chain.ground.write(AccessMsg {
                has_access: true,
                ..AccessMsg::default()
            });
            assert_eq!(chain.run_until(5 * SECOND_NANOS), data_5s, "at 5 s");
        }
    }

    /// End to end: a ground target's access lets the FSW
    /// `simpleInstrumentController` switch the camera on for one image,
    /// which fills a partitioned storage unit; a ground station's access
    /// then lets the transmitter drain it packet by packet.
    #[test]
    fn instrument_storage_and_downlink_chain() {
        let orbit_radius_m = REQ_EARTH * 1.0e3 + 500.0e3;
        let spacecraft = Output::new(SpacecraftStateMsg::default());
        let guidance = Output::new(AttitudeGuidanceMsg::default());

        let mut target = GroundLocation::new(GroundLocationConfig {
            name: "target".to_string(),
            ..GroundLocationConfig::default()
        });
        let mut station = GroundLocation::new(GroundLocationConfig {
            name: "station".to_string(),
            location: SiteLocation::Geodetic(Lla {
                latitude_rad: 0.0,
                longitude_rad: std::f64::consts::FRAC_PI_2,
                altitude_m: 0.0,
            }),
            ..GroundLocationConfig::default()
        });
        let target_index = target.add_spacecraft_to_model();
        let station_index = station.add_spacecraft_to_model();
        spacecraft.connect_to(&mut target.sc_state_in_msgs[target_index]);
        spacecraft.connect_to(&mut station.sc_state_in_msgs[station_index]);

        let mut controller = SimpleInstrumentController::new(SimpleInstrumentControllerConfig {
            att_err_tolerance: 0.1,
            ..SimpleInstrumentControllerConfig::default()
        });
        target.access_out_msgs[target_index].connect_to(&mut controller.location_access_in_msg);
        guidance.connect_to(&mut controller.att_guid_in_msg);

        let mut camera = SimpleInstrument::new("camera");
        camera.node_baud_rate_bps = 19200.0;
        camera.node_data_name = "camera".to_string();
        controller
            .device_cmd_out_msg
            .connect_to(&mut camera.node_status_in_msg);

        let mut downlink = transmitter(-9600.0);
        station.access_out_msgs[station_index].connect_to(downlink.add_access_msg_to_transmitter());

        let mut storage = PartitionedStorageUnit::new("storage");
        storage.storage_capacity_bits = CAPACITY_BITS;
        camera
            .node_data_out_msg
            .connect_to(storage.add_data_node_to_model());
        downlink
            .node_data_out_msg
            .connect_to(storage.add_data_node_to_model());
        storage
            .storage_unit_data_out_msg
            .connect_to(downlink.add_storage_unit_to_transmitter());

        // Handles on the outputs the scheduled modules publish.
        let command = controller.device_cmd_out_msg.clone();
        let camera_data = camera.node_data_out_msg.clone();
        let downlink_data = downlink.node_data_out_msg.clone();
        let stored = storage.storage_unit_data_out_msg.clone();

        // Producers before consumers, the transmitter reading the storage
        // status of the previous tick, as in upstream's tests.
        let mut modules: [&mut dyn Module; 6] = [
            &mut target,
            &mut station,
            &mut controller,
            &mut camera,
            &mut downlink,
            &mut storage,
        ];
        for module in modules.iter_mut() {
            module.init();
        }
        let mut clock = TaskClock::new(SECOND_NANOS);
        // Per tick: device command, camera rate, downlink rate, stored bits.
        let mut log: Vec<(u64, f64, f64, f64)> = Vec::new();
        let mut run_until = |modules: &mut [&mut dyn Module; 6], stop_nanos| {
            for tick in clock.run_until(stop_nanos) {
                for module in modules.iter_mut() {
                    module.update(&context(tick));
                }
                log.push((
                    command.read().device_cmd,
                    camera_data.read().baud_rate_bps,
                    downlink_data.read().baud_rate_bps,
                    stored.read().storage_level_bits,
                ));
            }
        };
        let place = |position_m| SpacecraftStateMsg {
            position_m,
            ..SpacecraftStateMsg::default()
        };

        // 0-1 s over the north pole: neither site sees the spacecraft.
        spacecraft.write(place(Vector3::new(0.0, 0.0, orbit_radius_m)));
        run_until(&mut modules, SECOND_NANOS);
        // 2-4 s over the target: one image of 19200 bit, taken at 2 s.
        spacecraft.write(place(Vector3::new(orbit_radius_m, 0.0, 0.0)));
        run_until(&mut modules, 4 * SECOND_NANOS);
        // 5-8 s over the station: two 9600 bit packets drain the camera
        // partition, then the transmitter waits for a full packet.
        spacecraft.write(place(Vector3::new(0.0, orbit_radius_m, 0.0)));
        run_until(&mut modules, 8 * SECOND_NANOS);

        let commands: Vec<u64> = log.iter().map(|sample| sample.0).collect();
        let camera_rates: Vec<f64> = log.iter().map(|sample| sample.1).collect();
        let downlink_rates: Vec<f64> = log.iter().map(|sample| sample.2).collect();
        let levels: Vec<f64> = log.iter().map(|sample| sample.3).collect();
        assert_eq!(commands, [0, 0, 1, 0, 0, 0, 0, 0, 0]);
        assert_eq!(
            camera_rates,
            [0.0, 0.0, 19200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        );
        assert_eq!(
            downlink_rates,
            [0.0, 0.0, 0.0, 0.0, 0.0, -9600.0, -9600.0, 0.0, 0.0]
        );
        assert_eq!(
            levels,
            [0.0, 0.0, 19200.0, 19200.0, 19200.0, 9600.0, 0.0, 0.0, 0.0]
        );
        let last = stored.read();
        assert_eq!(last.stored_data_name, ["camera"]);
        assert_eq!(downlink_data.read().data_name, "camera");
    }
}
