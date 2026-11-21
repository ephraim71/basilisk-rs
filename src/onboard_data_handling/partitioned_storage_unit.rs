//! Data storage with one partition per data name: port of upstream Basilisk
//! `simulation/onboardDataHandling/storageUnit/partitionedStorageUnit`.
//!
//! Each data node's bits are filed under the node's data name, creating the
//! partition on first use; partitions can also be created up front with
//! [`PartitionedStorageUnit::add_partition`]. See
//! [`super::data_storage_unit_base`] for the shared reading, capacity and
//! publishing, and [`DataStorageUnitBase::integrate_by_data_name`] for the
//! filing rule.

use super::data_storage_unit_base::{DataInstance, DataStorageUnitBase, StoragePartitioning};
use crate::messages::DATA_NAME_SIZE;

/// The partitioning of [`PartitionedStorageUnit`]: upstream's base
/// `integrateDataStatus`, one partition per data name.
#[derive(Clone, Copy, Debug, Default)]
pub struct NamedPartitions;

impl StoragePartitioning for NamedPartitions {}

/// A storage unit with one partition per data name: upstream
/// `PartitionedStorageUnit`.
pub type PartitionedStorageUnit = DataStorageUnitBase<NamedPartitions>;

impl PartitionedStorageUnit {
    /// Upstream `addPartition`: adds an empty partition named `data_name`.
    ///
    /// # Panics
    ///
    /// If `data_name` has [`DATA_NAME_SIZE`] characters or more (upstream
    /// `bskError`).
    pub fn add_partition(&mut self, data_name: impl Into<String>) {
        let data_name = data_name.into();
        assert!(
            data_name.len() < DATA_NAME_SIZE,
            "PartitionedStorageUnit: dataName is {} characters, but dataInstanceName supports at \
             most {} characters.",
            data_name.len(),
            DATA_NAME_SIZE - 1
        );
        self.stored_data.push(DataInstance {
            name: data_name,
            sum_bits: 0,
        });
    }

    /// Upstream `setDataBuffer(partitionNames, data)`: adds `data_bits[i]`
    /// (negative to remove) to the partition named `partition_names[i]` once,
    /// in order; see [`DataStorageUnitBase`]'s rules for refused data.
    ///
    /// # Panics
    ///
    /// If the two slices differ in length.
    pub fn set_data_buffer<S: AsRef<str>>(&mut self, partition_names: &[S], data_bits: &[i64]) {
        assert_eq!(
            partition_names.len(),
            data_bits.len(),
            "PartitionedStorageUnit: one data amount per partition name is needed"
        );
        for (name, data) in partition_names.iter().zip(data_bits) {
            self.set_partition_data(name.as_ref(), *data);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `storageUnit/_UnitTest/test_partitionedStorageUnit.py`
    //! with its expected values and tolerances, and the partitioned-unit
    //! cases of `test_simpleStorageUnit.py`'s
    //! `test_integrated_data_rejects_int64_overflow` and
    //! `test_integrated_data_delta_range_check`. The SWIG typemap tests are
    //! not ported; see `simple_storage_unit`.

    use super::PartitionedStorageUnit;
    use crate::Module;
    use crate::messages::{DataNodeUsageMsg, DataStorageStatusMsg, Output};
    use crate::onboard_data_handling::test_support::{TaskClock, context};

    /// Upstream `testProcessRate`, `macros.sec2nano(0.1)`.
    const STEP_NANOS: u64 = 100_000_000;
    const SECOND_NANOS: u64 = 1_000_000_000;
    /// Upstream `sim_time` \[s\] of the set-data cases.
    const SIM_TIME_S: f64 = 5.0;

    /// Two node baud rates, partition names, data added to them and the
    /// capacity.
    type PartitionCase = (f64, f64, &'static [&'static str], &'static [i64], i64);
    /// A [`PartitionCase`] and the expected stored data.
    type SetDataCase = (f64, f64, &'static [&'static str], &'static [i64], i64, f64);

    fn node(name: &str, baud_rate_bps: f64) -> Output<DataNodeUsageMsg> {
        Output::new(DataNodeUsageMsg {
            data_name: name.to_string(),
            baud_rate_bps,
        })
    }

    fn run_until(
        unit: &mut PartitionedStorageUnit,
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

    /// A unit of `capacity` fed by "node_1_msg" at `baud_1` and
    /// "node_2_msg" at `baud_2`, with the node outputs kept alive.
    fn two_node_unit(
        baud_1: f64,
        baud_2: f64,
        capacity: i64,
    ) -> (PartitionedStorageUnit, [Output<DataNodeUsageMsg>; 2]) {
        let nodes = [node("node_1_msg", baud_1), node("node_2_msg", baud_2)];
        let mut unit = PartitionedStorageUnit::new("storage");
        unit.storage_capacity_bits = capacity;
        for data in &nodes {
            data.connect_to(unit.add_data_node_to_model());
        }
        (unit, nodes)
    }

    /// Initializes `unit`, runs to 4 s, adds `add_data` to `names`, and runs
    /// to 5 s, as upstream's set-data cases do after zeroing `names`.
    fn run_set_data_case(
        unit: &mut PartitionedStorageUnit,
        names: &[&str],
        add_data: &[i64],
    ) -> Vec<DataStorageStatusMsg> {
        unit.set_data_buffer(names, &vec![0; names.len()]);
        unit.init();
        let (mut clock, mut log) = (TaskClock::new(STEP_NANOS), Vec::new());
        run_until(unit, &mut clock, 4 * SECOND_NANOS, &mut log);
        unit.set_data_buffer(names, add_data);
        run_until(unit, &mut clock, 5 * SECOND_NANOS, &mut log);
        log
    }

    /// Upstream `test_storage_limits`.
    #[test]
    fn storage_limits() {
        let cases = [
            (1200.0, 1200.0, 2400, 2400.0),
            (600.0, 1200.0, 3600, 3600.0),
            (600.0, 600.0, 10000, 6000.0),
            (-100.0, 0.0, 10000, 0.0),
            (-100.0, -100.0, 10000, 0.0),
        ];
        for (baud_1, baud_2, capacity, expected) in cases {
            let (mut unit, _nodes) = two_node_unit(baud_1, baud_2, capacity);
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
        }
    }

    /// Upstream `test_set_data_buffer`.
    #[test]
    fn set_data_buffer() {
        let cases: [SetDataCase; 11] = [
            (1200.0, 1200.0, &["test"], &[1200], 2400, 2400.0),
            (600.0, 600.0, &["test"], &[0], 10000, 6000.0),
            (600.0, 600.0, &["test"], &[4000], 10000, 10000.0),
            (0.0, 0.0, &["test"], &[1000], 2000, 1000.0),
            (0.0, 0.0, &["test"], &[-1000], 2000, 0.0),
            (1000.0, 0.0, &["node_1_msg"], &[-2000], 6000, 3000.0),
            (0.0, 0.0, &["test"], &[3000], 2000, 0.0),
            (600.0, 0.0, &["node_1_msg"], &[3000], 10000, 6000.0),
            (300.0, 600.0, &["node_1_msg"], &[3000], 10000, 7500.0),
            (
                600.0,
                600.0,
                &["node_1_msg", "node_2_msg"],
                &[1000, 1000],
                10000,
                8000.0,
            ),
            (
                600.0,
                300.0,
                &["test", "node_2_msg"],
                &[1000, 1000],
                10000,
                6500.0,
            ),
        ];
        for (baud_1, baud_2, names, add_data, capacity, expected) in cases {
            let (mut unit, _nodes) = two_node_unit(baud_1, baud_2, capacity);
            let log = run_set_data_case(&mut unit, names, add_data);

            for status in &log {
                assert_allclose(status.current_net_baud_bps, baud_1 + baud_2, 1e-4);
            }
            assert_allclose(log.last().unwrap().storage_level_bits, expected, 1e-4);
            assert_within_capacity(&log);
        }
    }

    /// Upstream `test_set_data_buffer_partition`: existing and new
    /// partitions each hold their added data plus their node's integral.
    #[test]
    fn set_data_buffer_partition() {
        let cases: [PartitionCase; 7] = [
            (600.0, 600.0, &["test"], &[0], 10000),
            (0.0, 0.0, &["test"], &[1000], 2000),
            (1000.0, 0.0, &["node_1_msg"], &[-2000], 6000),
            (600.0, 0.0, &["node_1_msg"], &[3000], 10000),
            (600.0, 600.0, &["node_1_msg"], &[3000], 10000),
            (
                600.0,
                600.0,
                &["node_1_msg", "node_2_msg"],
                &[1000, 1000],
                10000,
            ),
            (
                600.0,
                300.0,
                &["test", "node_1_msg", "node_2_msg"],
                &[1500, 1000, 1000],
                10000,
            ),
        ];
        for (baud_1, baud_2, names, add_data, capacity) in cases {
            let (mut unit, _nodes) = two_node_unit(baud_1, baud_2, capacity);
            let log = run_set_data_case(&mut unit, names, add_data);
            let last = log.last().unwrap();

            for (name, added) in names.iter().zip(add_data) {
                let index = last
                    .stored_data_name
                    .iter()
                    .position(|stored| stored == name)
                    .expect("the partition was added");
                let baud = match *name {
                    "node_1_msg" => baud_1,
                    "node_2_msg" => baud_2,
                    _ => 0.0,
                };
                let desired = *added as f64 + baud * SIM_TIME_S;
                assert_allclose(last.stored_data_bits[index], desired, 0.0);
            }
        }
    }

    /// Upstream `test_data_removal`.
    #[test]
    fn data_removal() {
        let cases = [
            (-1200.0, 0.0, 2400, 4800, 2400.0),
            (-120.0, 0.0, 2400, 4800, 4200.0),
            (-1200.0, -1200.0, 2400, 4800, 0.0),
        ];
        for (baud_1, baud_2, initial_data, capacity, expected) in cases {
            let (mut unit, _nodes) = two_node_unit(baud_1, baud_2, capacity);
            unit.set_data_buffer(&["node_1_msg", "node_2_msg"], &[initial_data; 2]);
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

    /// The partitioned-unit case of upstream
    /// `test_integrated_data_rejects_int64_overflow`.
    #[test]
    fn integrated_data_rejects_int64_overflow() {
        let capacity = i64::MAX;
        let initial_data = capacity - 10_000;
        let mut unit = PartitionedStorageUnit::new("storage");
        unit.storage_capacity_bits = capacity;
        unit.set_data_buffer(&["dataNode"], &[initial_data]);
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

    /// The partitioned-unit cases of upstream
    /// `test_integrated_data_delta_range_check`.
    #[test]
    fn integrated_data_delta_range_check() {
        for baud in [2f64.powi(63), f64::INFINITY] {
            let result = std::panic::catch_unwind(|| {
                let mut unit = PartitionedStorageUnit::new("storage");
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
    fn unnamed_nodes_create_no_partition() {
        let data = node("", 100.0);
        let mut unit = PartitionedStorageUnit::new("storage");
        unit.storage_capacity_bits = 1000;
        unit.add_partition("A");
        data.connect_to(unit.add_data_node_to_model());
        unit.init();
        let mut log = Vec::new();
        run_until(
            &mut unit,
            &mut TaskClock::new(SECOND_NANOS),
            2 * SECOND_NANOS,
            &mut log,
        );
        let last = log.last().unwrap();
        assert_eq!(last.stored_data_name, ["A"]);
        assert_eq!(last.storage_level_bits, 0.0);
        assert_eq!(last.current_net_baud_bps, 100.0);
    }

    /// Upstream's read fails while a node message is unwritten; here an
    /// unconnected node input stops the integration.
    #[test]
    fn unconnected_node_skips_the_integration() {
        let data = node("A", 100.0);
        let mut unit = PartitionedStorageUnit::new("storage");
        unit.storage_capacity_bits = 1000;
        data.connect_to(unit.add_data_node_to_model());
        unit.add_data_node_to_model();
        unit.init();
        let mut log = Vec::new();
        run_until(
            &mut unit,
            &mut TaskClock::new(SECOND_NANOS),
            2 * SECOND_NANOS,
            &mut log,
        );
        let last = log.last().unwrap();
        assert!(last.stored_data_name.is_empty());
        assert_eq!(last.current_net_baud_bps, 0.0);
    }

    #[test]
    #[should_panic(expected = "dataName is 128 characters")]
    fn add_partition_rejects_a_name_that_does_not_fit() {
        PartitionedStorageUnit::new("storage").add_partition("x".repeat(128));
    }
}
