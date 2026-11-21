//! Storage-unit plumbing shared by the data storage units: port of upstream
//! Basilisk
//! `simulation/onboardDataHandling/_GeneralModuleFiles/dataStorageUnitBase`.
//!
//! [`DataStorageUnitBase`] is the module. It reads one [`DataNodeUsageMsg`]
//! per connected data node, integrates each node's baud rate over the step
//! into whole bits, files the bits under a partition, and publishes the
//! partitions in a [`DataStorageStatusMsg`]. How the bits are filed is the
//! [`StoragePartitioning`]: [`super::partitioned_storage_unit`] files each
//! node under its own data name (upstream's base behaviour), and
//! [`super::simple_storage_unit`] keeps one "STORED DATA" partition.
//!
//! Stored data is an `i64` bit count, as upstream. Additions that would
//! overflow are refused by the capacity check, removals clamp each partition
//! at zero, and a node can only add data while the total stays within the
//! capacity.
//!
//! # Differences from upstream
//!
//! * No message headers: a connected data node counts as written at the
//!   current update. A unit with no data node, or with one not connected,
//!   skips the integration, as upstream does when a read fails, and still
//!   publishes its partitions.
//! * Upstream's `bskError`s on a data change, partition or total outside the
//!   `int64_t` range, and on a data name too long for upstream's buffer, are
//!   panics, so the run stops as upstream's does.
//! * The capacity warning both upstream units log at reset lives here.

use std::marker::PhantomData;

use crate::messages::{DATA_NAME_SIZE, DataNodeUsageMsg, DataStorageStatusMsg, Input, Output};
use crate::{Module, SimulationContext};

/// Data stored under one name (upstream `dataInstance`).
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct DataInstance {
    /// Upstream `dataInstanceName`: partition name.
    pub name: String,
    /// Upstream `dataInstanceSum` \[bit\]: data in the partition.
    pub sum_bits: i64,
}

/// How a [`DataStorageUnitBase`] files the integrated node data: upstream's
/// virtual `integrateDataStatus`.
pub trait StoragePartitioning: Send + Sized {
    /// Upstream `integrateDataStatus`: integrates the node messages read
    /// this update into `unit`'s partitions at `current_time_s` \[s\]. The
    /// default is upstream's base version, one partition per data name; see
    /// [`DataStorageUnitBase::integrate_by_data_name`].
    fn integrate_data_status(unit: &mut DataStorageUnitBase<Self>, current_time_s: f64) {
        unit.integrate_by_data_name(current_time_s);
    }
}

/// A data storage unit (upstream `DataStorageUnitBase`), generic over how it
/// partitions its data.
#[derive(Clone, Debug)]
pub struct DataStorageUnitBase<P> {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `nodeDataUseInMsgs`: one input per data node, added by
    /// [`Self::add_data_node_to_model`].
    pub node_data_use_in_msgs: Vec<Input<DataNodeUsageMsg>>,
    /// Upstream `storageUnitDataOutMsg`: stored data and partitions.
    pub storage_unit_data_out_msg: Output<DataStorageStatusMsg>,
    /// Upstream `storageCapacity` \[bit\]: largest amount of data the unit
    /// holds; must be positive (init logs it otherwise).
    pub storage_capacity_bits: i64,
    /// Upstream `nodeBaudMsgs`: the node messages read this update.
    node_baud_msgs: Vec<DataNodeUsageMsg>,
    /// Upstream `storedDataSum` \[bit\]: data in all partitions.
    pub(super) stored_data_sum_bits: i64,
    /// Upstream `storedData`: the partitions.
    pub(super) stored_data: Vec<DataInstance>,
    /// Upstream `previousTime` \[s\]: time of the last integration.
    previous_time_s: f64,
    /// Upstream `currentTimestep` \[s\]: step of the current integration.
    current_timestep_s: f64,
    /// Upstream `netBaud` \[bit/s\]: sum of the node baud rates.
    pub(super) net_baud_bps: f64,
    partitioning: PhantomData<fn() -> P>,
}

/// `left + right`, or `None` where `int64_t` would overflow (upstream
/// `checkedAddInt64`).
fn checked_add(left: i64, right: i64) -> Option<i64> {
    left.checked_add(right)
}

impl<P: StoragePartitioning> DataStorageUnitBase<P> {
    /// An empty unit with zero capacity, no data nodes and no partitions
    /// (upstream's constructors). Set [`Self::storage_capacity_bits`] before
    /// running it.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            node_data_use_in_msgs: Vec::new(),
            storage_unit_data_out_msg: Output::default(),
            storage_capacity_bits: 0,
            node_baud_msgs: Vec::new(),
            stored_data_sum_bits: 0,
            stored_data: Vec::new(),
            previous_time_s: 0.0,
            current_timestep_s: 0.0,
            net_baud_bps: 0.0,
            partitioning: PhantomData,
        }
    }

    /// Upstream `addDataNodeToModel`: adds a data-node input and returns it
    /// for connecting.
    pub fn add_data_node_to_model(&mut self) -> &mut Input<DataNodeUsageMsg> {
        self.node_data_use_in_msgs.push(Input::default());
        self.node_data_use_in_msgs
            .last_mut()
            .expect("just pushed a data node input")
    }

    /// The partitions, in the order they were created.
    pub fn stored_data(&self) -> &[DataInstance] {
        &self.stored_data
    }

    /// Upstream `storedDataSum` \[bit\]: data in all partitions.
    pub fn stored_data_sum_bits(&self) -> i64 {
        self.stored_data_sum_bits
    }

    /// Upstream `messageInStoredData`: index of the partition named `name`
    /// (the last one, should there be several), if any.
    fn partition_index(&self, name: &str) -> Option<usize> {
        self.stored_data
            .iter()
            .rposition(|partition| partition.name == name)
    }

    /// Upstream `sumAllData` \[bit\].
    ///
    /// # Panics
    ///
    /// If the total leaves the `i64` range (upstream `bskError`).
    fn sum_all_data(&self) -> i64 {
        self.stored_data.iter().fold(0, |sum, partition| {
            checked_add(sum, partition.sum_bits).unwrap_or_else(|| {
                panic!(
                    "DataStorageUnitBase: total stored data exceeds the supported int64_t range."
                )
            })
        })
    }

    /// Upstream `computeDataDelta` \[bit\]: `baud_rate_bps` integrated over
    /// the current step and rounded to whole bits.
    ///
    /// # Panics
    ///
    /// If the change is not finite or leaves the `i64` range (upstream
    /// `bskError`).
    pub(super) fn compute_data_delta(&self, baud_rate_bps: f64) -> i64 {
        let rounded = (baud_rate_bps * self.current_timestep_s).round();
        let limit = 2f64.powi(63);
        assert!(
            rounded.is_finite() && rounded >= -limit && rounded < limit,
            "DataStorageUnitBase: baud rate {baud_rate_bps:.17} [bits/s] over timestep {:.17} [s] \
             produces a data change outside the supported int64_t range.",
            self.current_timestep_s
        );
        rounded as i64
    }

    /// Upstream `dataFitsInCapacity`: whether adding `data_delta` keeps the
    /// total within the capacity (`allow_equal` accepts a full unit). A sum
    /// that would overflow fits only when it is a removal.
    pub(super) fn data_fits_in_capacity(&self, data_delta: i64, allow_equal: bool) -> bool {
        match checked_add(self.stored_data_sum_bits, data_delta) {
            None => data_delta < 0,
            Some(sum) if allow_equal => sum <= self.storage_capacity_bits,
            Some(sum) => sum < self.storage_capacity_bits,
        }
    }

    /// Upstream `applyDataDelta`: adds `data_delta` to partition `index`,
    /// clamping it at zero.
    ///
    /// # Panics
    ///
    /// If an addition leaves the `i64` range (upstream `bskError`).
    pub(super) fn apply_data_delta(&mut self, index: usize, data_delta: i64) {
        let partition = &mut self.stored_data[index];
        partition.sum_bits = match checked_add(partition.sum_bits, data_delta) {
            Some(updated) => updated.max(0),
            None if data_delta < 0 => 0,
            None => {
                panic!("DataStorageUnitBase: partition data exceeds the supported int64_t range.")
            }
        };
    }

    /// Upstream base `integrateDataStatus`: files each node's data under its
    /// data name, creating the partition on first use. A node may add data
    /// only while the total stays within the capacity; a sink (negative baud
    /// rate) may always remove it. A node without a data name creates no
    /// partition.
    ///
    /// # Panics
    ///
    /// If a data name has [`DATA_NAME_SIZE`] characters or more, or a data
    /// change leaves the `i64` range (upstream `bskError`s).
    pub fn integrate_by_data_name(&mut self, current_time_s: f64) {
        let messages = self.begin_integration(current_time_s);
        for message in &messages {
            assert!(
                message.data_name.len() < DATA_NAME_SIZE,
                "DataStorageUnitBase: dataName is not null-terminated within {DATA_NAME_SIZE} \
                 characters."
            );
            let index = self.partition_index(&message.data_name);
            let data_delta = self.compute_data_delta(message.baud_rate_bps);
            if self.data_fits_in_capacity(data_delta, true) || message.baud_rate_bps < 0.0 {
                if let Some(index) = index {
                    self.apply_data_delta(index, data_delta);
                } else if !message.data_name.is_empty() {
                    self.stored_data.push(DataInstance {
                        name: message.data_name.clone(),
                        sum_bits: data_delta.max(0),
                    });
                }
            }
            self.net_baud_bps += message.baud_rate_bps;
            self.stored_data_sum_bits = self.sum_all_data();
        }
        self.end_integration(messages, current_time_s);
    }

    /// Starts an integration at `current_time_s` \[s\]: sets the step,
    /// clears the net baud rate and hands out the node messages read this
    /// update.
    pub(super) fn begin_integration(&mut self, current_time_s: f64) -> Vec<DataNodeUsageMsg> {
        self.current_timestep_s = current_time_s - self.previous_time_s;
        self.net_baud_bps = 0.0;
        std::mem::take(&mut self.node_baud_msgs)
    }

    /// Ends an integration started with [`Self::begin_integration`].
    pub(super) fn end_integration(&mut self, messages: Vec<DataNodeUsageMsg>, current_time_s: f64) {
        self.node_baud_msgs = messages;
        self.previous_time_s = current_time_s;
    }

    /// Upstream base `setDataBuffer(partitionName, data)`: adds `data_bits`
    /// (negative to remove) to the partition named `partition_name` once,
    /// creating it if needed. Data that would exceed the capacity is refused
    /// whole, and a removal that would take the partition below zero is
    /// ignored; a new partition starts at no less than zero.
    ///
    /// # Panics
    ///
    /// If a new partition's name has [`DATA_NAME_SIZE`] characters or more,
    /// or an addition leaves the `i64` range (upstream `bskError`s).
    pub(super) fn set_partition_data(&mut self, partition_name: &str, data_bits: i64) {
        let index = self.partition_index(partition_name);
        if self.data_fits_in_capacity(data_bits, true) {
            if let Some(index) = index {
                let partition = &mut self.stored_data[index];
                match checked_add(partition.sum_bits, data_bits) {
                    Some(updated) if updated >= 0 => partition.sum_bits = updated,
                    Some(_) => {}
                    None if data_bits > 0 => panic!(
                        "DataStorageUnitBase: partition data exceeds the supported int64_t range."
                    ),
                    None => {}
                }
            } else if !partition_name.is_empty() {
                assert!(
                    partition_name.len() < DATA_NAME_SIZE,
                    "DataStorageUnitBase: partitionName is {} characters, but dataInstanceName \
                     supports at most {} characters.",
                    partition_name.len(),
                    DATA_NAME_SIZE - 1
                );
                self.stored_data.push(DataInstance {
                    name: partition_name.to_string(),
                    sum_bits: data_bits.max(0),
                });
            }
        }
        self.stored_data_sum_bits = self.sum_all_data();
    }

    /// Upstream `Reset` (with both units' `customReset`).
    fn reset_state(&mut self) {
        self.previous_time_s = 0.0;
        if self.storage_capacity_bits <= 0 {
            log::info!(
                "{}: The storageCapacity variable must be set to a positive value.",
                self.name
            );
        }
    }

    /// Upstream `readMessages`: whether every data node could be read.
    fn read_messages(&mut self) -> bool {
        self.node_baud_msgs.clear();
        if self.node_data_use_in_msgs.is_empty() {
            log::info!(
                "{}: Data storage has no data node messages to read.",
                self.name
            );
            return false;
        }
        self.node_baud_msgs
            .extend(self.node_data_use_in_msgs.iter().map(Input::read));
        self.node_data_use_in_msgs.iter().all(Input::is_connected)
    }

    /// Upstream `writeMessages`.
    fn write_messages(&self) {
        self.storage_unit_data_out_msg.write(DataStorageStatusMsg {
            storage_level_bits: self.stored_data_sum_bits as f64,
            storage_capacity_bits: self.storage_capacity_bits as f64,
            current_net_baud_bps: self.net_baud_bps,
            stored_data_name: self
                .stored_data
                .iter()
                .map(|partition| partition.name.clone())
                .collect(),
            stored_data_bits: self
                .stored_data
                .iter()
                .map(|partition| partition.sum_bits as f64)
                .collect(),
        });
    }
}

impl<P: StoragePartitioning> Module for DataStorageUnitBase<P> {
    fn init(&mut self) {
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    /// Upstream `UpdateState`: integrate when a data node was read, then
    /// publish.
    fn update(&mut self, context: &SimulationContext) {
        if self.read_messages() {
            P::integrate_data_status(self, context.current_sim_nanos as f64 * 1.0e-9);
        }
        self.write_messages();
    }
}
