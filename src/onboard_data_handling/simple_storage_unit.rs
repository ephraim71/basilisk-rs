//! Single-partition data storage: port of upstream Basilisk
//! `simulation/onboardDataHandling/storageUnit/simpleStorageUnit`.
//!
//! Every data node's bits go into one "STORED DATA" partition, whatever the
//! node's data name. See [`super::data_storage_unit_base`] for the shared
//! reading, capacity and publishing.
//!
//! As upstream, the partition is created by the first node integrated into
//! an empty unit, and that node's data for that step is not stored; a unit
//! primed with [`SimpleStorageUnit::set_data_buffer`] already has it.

use super::data_storage_unit_base::{DataInstance, DataStorageUnitBase, StoragePartitioning};

/// Upstream `SimpleStorageUnit`'s partition name.
pub const STORED_DATA_PARTITION: &str = "STORED DATA";

/// The partitioning of [`SimpleStorageUnit`]: upstream `SimpleStorageUnit`'s
/// `integrateDataStatus` override.
#[derive(Clone, Copy, Debug, Default)]
pub struct SinglePartition;

/// A storage unit with one partition: upstream `SimpleStorageUnit`.
pub type SimpleStorageUnit = DataStorageUnitBase<SinglePartition>;

impl SimpleStorageUnit {
    /// Upstream `setDataBuffer(data)`: adds `data_bits` (negative to remove)
    /// to the "STORED DATA" partition once; see
    /// [`DataStorageUnitBase`]'s rules for refused data.
    pub fn set_data_buffer(&mut self, data_bits: i64) {
        self.set_partition_data(STORED_DATA_PARTITION, data_bits);
    }
}

impl StoragePartitioning for SinglePartition {
    /// Each node in turn adds its data to the one partition while the total
    /// stays within the capacity; a node that does not generate data
    /// (baud rate at or below zero) may always remove it.
    fn integrate_data_status(unit: &mut DataStorageUnitBase<Self>, current_time_s: f64) {
        let messages = unit.begin_integration(current_time_s);
        for message in &messages {
            let data_delta = unit.compute_data_delta(message.baud_rate_bps);
            if unit.stored_data.is_empty() {
                unit.stored_data.push(DataInstance {
                    name: STORED_DATA_PARTITION.to_string(),
                    sum_bits: 0,
                });
            } else if unit.data_fits_in_capacity(data_delta, true) || message.baud_rate_bps <= 0.0 {
                unit.apply_data_delta(0, data_delta);
            }
            // The next node is checked against the updated amount.
            unit.stored_data_sum_bits = unit.stored_data[0].sum_bits;
            unit.net_baud_bps += message.baud_rate_bps;
        }
        unit.end_integration(messages, current_time_s);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `storageUnit/_UnitTest/test_simpleStorageUnit.py` with
    //! its expected values and tolerances: `test_storage_limits`,
    //! `test_set_data_buffer`, `test_data_removal`,
    //! `test_set_data_buffer_rejects_int64_overflow`,
    //! `test_multiple_data_nodes_respect_storage_capacity`, and the
    //! simple-unit cases of `test_integrated_data_rejects_int64_overflow` and
    //! `test_integrated_data_delta_range_check` (the partitioned cases are in
    //! `partitioned_storage_unit`).
    //!
    //! Not ported: `test_storage_python_inputs_validate_int64_range` and
    //! `test_storage_python_inputs_accept_scientific_notation` check the SWIG
    //! typemaps that convert Python numbers; here the capacity and data are
    //! `i64`, so out-of-range values do not compile.
    //!
    //! The unit is stepped by hand so it can be changed between runs, as the
    //! upstream tests do between `ExecuteSimulation` calls.

    use super::SimpleStorageUnit;
    use crate::Module;
    use crate::messages::{DataNodeUsageMsg, DataStorageStatusMsg, Output};
    use crate::onboard_data_handling::test_support::{TaskClock, context};

    /// Upstream `testProcessRate`, `macros.sec2nano(0.1)`.
    const STEP_NANOS: u64 = 100_000_000;
    const SECOND_NANOS: u64 = 1_000_000_000;

    fn node(name: &str, baud_rate_bps: f64) -> Output<DataNodeUsageMsg> {
        Output::new(DataNodeUsageMsg {
            data_name: name.to_string(),
            baud_rate_bps,
        })
    }

    /// Runs the ticks up to `stop_nanos`, recording the status after each.
    fn run_until(
        unit: &mut SimpleStorageUnit,
        clock: &mut TaskClock,
        stop_nanos: u64,
        log: &mut Vec<DataStorageStatusMsg>,
    ) {
        for tick in clock.run_until(stop_nanos) {
            unit.update(&context(tick));
            log.push(unit.storage_unit_data_out_msg.read());
        }
    }

    /// `np.testing.assert_allclose(actual, desired, atol=atol)` (rtol 1e-7).
    #[track_caller]
    fn assert_allclose(actual: f64, desired: f64, atol: f64) {
        assert!(
            (actual - desired).abs() <= atol + 1e-7 * desired.abs(),
            "{actual} vs {desired}"
        );
    }

    /// The capacity and sign checks every upstream case makes on the log.
    #[track_caller]
    fn assert_within_capacity(log: &[DataStorageStatusMsg]) {
        for status in log {
            let (level, capacity) = (status.storage_level_bits, status.storage_capacity_bits);
            assert!(
                level <= capacity || (level - capacity).abs() <= 1e-8 + 1e-5 * capacity.abs(),
                "stored data exceeded its capacity"
            );
            assert!(level >= 0.0, "stored data was negative");
        }
    }

    /// Upstream `test_storage_limits`.
    #[test]
    fn storage_limits() {
        let cases = [
            (1200.0, 1200.0, 2400, 2400.0),
            (600.0, 1200.0, 3600, 3600.0),
            (600.0, 600.0, 10000, 6000.0),
            (-1000.0, 0.0, 5000, 0.0),
        ];
        for (baud_1, baud_2, capacity, expected) in cases {
            let (node_1, node_2) = (node("node_1_msg", baud_1), node("node_2_msg", baud_2));
            let mut unit = SimpleStorageUnit::new("storage");
            unit.storage_capacity_bits = capacity;
            node_1.connect_to(unit.add_data_node_to_model());
            node_2.connect_to(unit.add_data_node_to_model());

            unit.init();
            let mut log = Vec::new();
            run_until(
                &mut unit,
                &mut TaskClock::new(STEP_NANOS),
                5 * SECOND_NANOS,
                &mut log,
            );

            for status in &log {
                assert_allclose(status.current_net_baud_bps, baud_1 + baud_2, 1e-1);
            }
            assert_allclose(log.last().unwrap().storage_level_bits, expected, 1e-4);
            assert_within_capacity(&log);
            assert_eq!(log[0].stored_data_name, ["STORED DATA"]);
        }
    }

    /// Upstream `test_set_data_buffer`: data added between runs.
    #[test]
    fn set_data_buffer() {
        let cases = [
            (1200.0, 1200.0, 1200, 2400, 2400.0),
            (600.0, 600.0, 0, 10000, 6000.0),
            (600.0, 600.0, 4000, 10000, 10000.0),
            (0.0, 0.0, 1000, 2000, 1000.0),
            (0.0, 0.0, -1000, 2000, 0.0),
            (1000.0, 0.0, -2000, 6000, 3000.0),
            (0.0, 0.0, 3000, 2000, 0.0),
            (600.0, 0.0, 3000, 10000, 6000.0),
            (600.0, 600.0, 3000, 10000, 9000.0),
            (300.0, 600.0, 3000, 10000, 7500.0),
        ];
        for (baud_1, baud_2, add_data, capacity, expected) in cases {
            let (node_1, node_2) = (node("node_1_msg", baud_1), node("node_2_msg", baud_2));
            let mut unit = SimpleStorageUnit::new("storage");
            unit.storage_capacity_bits = capacity;
            node_1.connect_to(unit.add_data_node_to_model());
            node_2.connect_to(unit.add_data_node_to_model());
            unit.set_data_buffer(0);

            unit.init();
            let (mut clock, mut log) = (TaskClock::new(STEP_NANOS), Vec::new());
            run_until(&mut unit, &mut clock, 4 * SECOND_NANOS, &mut log);
            unit.set_data_buffer(add_data);
            run_until(&mut unit, &mut clock, 5 * SECOND_NANOS, &mut log);

            for status in &log {
                assert_allclose(status.current_net_baud_bps, baud_1 + baud_2, 1e-4);
            }
            let last = log.last().unwrap();
            assert_allclose(last.storage_level_bits, expected, 1e-4);
            assert_within_capacity(&log);
            assert_eq!(last.stored_data_bits[0], last.storage_level_bits);
            assert_eq!(log[0].stored_data_name, ["STORED DATA"]);
        }
    }

    /// Upstream `test_data_removal`.
    #[test]
    fn data_removal() {
        let cases = [
            (-400.0, 2000, 2000, 0.0),
            (-800.0, 2000, 2000, 0.0),
            (-2000.0, 0, 2000, 0.0),
        ];
        for (baud, initial_data, capacity, expected) in cases {
            let node_1 = node("node_1_msg", baud);
            let mut unit = SimpleStorageUnit::new("storage");
            unit.storage_capacity_bits = capacity;
            node_1.connect_to(unit.add_data_node_to_model());
            unit.set_data_buffer(initial_data);

            unit.init();
            let mut log = Vec::new();
            run_until(
                &mut unit,
                &mut TaskClock::new(STEP_NANOS),
                5 * SECOND_NANOS,
                &mut log,
            );

            assert_allclose(log.last().unwrap().storage_level_bits, expected, 1e-4);
            assert_within_capacity(&log);
        }
    }

    /// Upstream `test_set_data_buffer_rejects_int64_overflow`.
    #[test]
    fn set_data_buffer_rejects_int64_overflow() {
        let capacity = i64::MAX;
        let initial_data = capacity - 10_000;
        let mut unit = SimpleStorageUnit::new("storage");
        unit.storage_capacity_bits = capacity;
        assert_eq!(unit.storage_capacity_bits, capacity);
        unit.set_data_buffer(initial_data);
        unit.set_data_buffer(20_000);

        unit.init();
        let mut log = Vec::new();
        run_until(&mut unit, &mut TaskClock::new(SECOND_NANOS), 0, &mut log);

        let last = log.last().unwrap();
        assert_eq!(last.storage_level_bits, initial_data as f64);
        assert_eq!(last.stored_data_bits[0], initial_data as f64);
    }

    /// The simple-unit case of upstream
    /// `test_integrated_data_rejects_int64_overflow`.
    #[test]
    fn integrated_data_rejects_int64_overflow() {
        let capacity = i64::MAX;
        let initial_data = capacity - 10_000;
        let mut unit = SimpleStorageUnit::new("storage");
        unit.storage_capacity_bits = capacity;
        unit.set_data_buffer(initial_data);
        let data = node("dataNode", 20_000.0);
        data.connect_to(unit.add_data_node_to_model());

        unit.init();
        let mut log = Vec::new();
        run_until(
            &mut unit,
            &mut TaskClock::new(SECOND_NANOS),
            SECOND_NANOS,
            &mut log,
        );

        let last = log.last().unwrap();
        assert_eq!(last.storage_level_bits, initial_data as f64);
        assert_eq!(last.stored_data_bits[0], initial_data as f64);
    }

    /// Upstream `test_multiple_data_nodes_respect_storage_capacity`: each
    /// node is checked against the level the previous one left.
    #[test]
    fn multiple_data_nodes_respect_storage_capacity() {
        let (capacity, initial_data, baud) = (100, 90, 9.0);
        let mut unit = SimpleStorageUnit::new("storage");
        unit.storage_capacity_bits = capacity;
        unit.set_data_buffer(initial_data);
        let nodes = [node("dataNodeA", baud), node("dataNodeB", baud)];
        for data in &nodes {
            data.connect_to(unit.add_data_node_to_model());
        }

        unit.init();
        let mut log = Vec::new();
        run_until(
            &mut unit,
            &mut TaskClock::new(SECOND_NANOS),
            SECOND_NANOS,
            &mut log,
        );

        let expected = (initial_data + baud as i64) as f64;
        let last = log.last().unwrap();
        assert_eq!(last.storage_level_bits, expected);
        assert_eq!(last.stored_data_bits[0], expected);
        assert!(last.storage_level_bits < capacity as f64);
    }

    /// The simple-unit cases of upstream
    /// `test_integrated_data_delta_range_check`: a data change outside the
    /// `int64_t` range stops the run.
    #[test]
    fn integrated_data_delta_range_check() {
        for baud in [2f64.powi(63), f64::INFINITY] {
            let result = std::panic::catch_unwind(|| {
                let mut unit = SimpleStorageUnit::new("storage");
                unit.storage_capacity_bits = i64::MAX;
                let data = node("dataNode", baud);
                data.connect_to(unit.add_data_node_to_model());
                unit.init();
                let mut log = Vec::new();
                run_until(
                    &mut unit,
                    &mut TaskClock::new(SECOND_NANOS),
                    SECOND_NANOS,
                    &mut log,
                );
            });
            let message = result.expect_err("the run must stop");
            let message = message
                .downcast_ref::<String>()
                .expect("a formatted panic message");
            assert!(
                message.contains("outside the supported int64_t range"),
                "{message}"
            );
        }
    }

    #[test]
    fn the_partition_is_created_on_the_first_update() {
        let data = node("dataNode", 10.0);
        let mut unit = SimpleStorageUnit::new("storage");
        unit.storage_capacity_bits = 1000;
        data.connect_to(unit.add_data_node_to_model());
        unit.init();
        let mut log = Vec::new();
        let mut clock = TaskClock::new(SECOND_NANOS);
        run_until(&mut unit, &mut clock, 3 * SECOND_NANOS, &mut log);
        // Created at t = 0 with nothing; 10 bits at each later second.
        let levels: Vec<f64> = log.iter().map(|status| status.storage_level_bits).collect();
        assert_eq!(levels, [0.0, 10.0, 20.0, 30.0]);
    }
}
