//! Link quality to reliable downlink throughput: port of upstream Basilisk
//! `simulation/communication/downlinkHandling`.
//!
//! A data node ([`crate::onboard_data_handling::data_node_base`]) that
//! drains a storage unit at the rate a radio link can deliver. Each update:
//!
//! 1. it selects the storage partition holding the most data across the
//!    connected storage units (falling back to a unit's total when it lists
//!    no partitions);
//! 2. it selects the receiver of the [`LinkBudgetMsg`]: the configured one,
//!    or in auto mode the receiving antenna with the larger CNR;
//! 3. from the CNR, the shared bandwidth and the requested bit rate it forms
//!    `C/N0` and `Eb/N0`, the BPSK bit error rate over AWGN
//!    `0.5 erfc(sqrt(Eb/N0))`, and the packet error rate of a packet that
//!    fails on any wrong bit;
//! 4. an ARQ model with at most `max_retransmissions` attempts per packet
//!    gives the drop probability `PER^m`, the expected attempts per packet,
//!    and so the rates at which data is attempted, removed from storage,
//!    delivered and dropped, scaled down so no more than the available data
//!    is removed in a step (and, by default, nothing until a full packet is
//!    stored);
//! 5. it publishes the removal as a negative baud rate under the
//!    partition's name, and the whole budget in a [`DownlinkHandlingMsg`].
//!
//! Removal is published only when it can reach the right partition: with
//! more than one storage unit connected, or a partition without a name, the
//! data-node message would be ambiguous, so the removal is forced to zero
//! (and the reason logged once). [`RemovalPolicy::RemoveDeliveredOnly`]
//! keeps dropped data onboard.
//!
//! While the node is off the diagnostics are zeroed (keeping the
//! configuration and the cumulative totals) and the step timer follows the
//! clock, so re-enabling does not integrate the off time.
//!
//! # Differences from upstream
//!
//! * No message headers: the link budget is used whenever it is connected
//!   ("connected ⇒ written at the current update"); a `LinkBudget` writes its
//!   output at init, as upstream's does at reset.
//! * The setters return `Err` instead of raising `bskError`, keeping the
//!   previous value. Upstream's `addStorageUnitToDownlink` overload taking a
//!   pre-made reader exists for SWIG reference retention and is not ported.

use std::sync::Arc;

use crate::messages::{
    ANTENNA_NAME_SIZE, AntennaState, DATA_NAME_SIZE, DataNodeUsageMsg, DataStorageStatusMsg,
    DownlinkHandlingMsg, Input, LinkBudgetMsg, Output, RemovalPolicy,
};
use crate::onboard_data_handling::data_node_base::{DataNodeBase, DataNodeFields, DataNodeModel};

/// Upstream's default `bitRateRequest` \[bit/s\].
const DEFAULT_BIT_RATE_REQUEST_BPS: f64 = 0.0;
/// Upstream's default `packetSizeBits` \[bit\].
const DEFAULT_PACKET_SIZE_BITS: f64 = 256.0;
/// Upstream's default `maxRetransmissions` \[-\].
const DEFAULT_MAX_RETRANSMISSIONS: u64 = 10;

/// Receiver of the link budget the downlink uses: upstream
/// `receiverAntenna` (0, 1 or 2).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum ReceiverSelection {
    /// Upstream `0`, the default: the receiving antenna with the larger
    /// CNR (antenna 1 on a tie).
    #[default]
    Auto,
    /// Upstream `1`: antenna 1 only.
    Antenna1,
    /// Upstream `2`: antenna 2 only.
    Antenna2,
}

/// Upstream `StorageSelection`: the storage target of a step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct StorageSelection {
    /// Upstream `storageUnitIndex`, `None` for `-1`.
    storage_unit: Option<usize>,
    /// Upstream `partitionIndex`, `None` for `-1` (a unit's total).
    partition: Option<usize>,
    /// Upstream `availableBits` \[bit\].
    available_bits: f64,
}

/// The model of [`DownlinkHandling`]: upstream `DownlinkHandling`'s members
/// and `evaluateDataModel`.
#[derive(Clone, Debug)]
pub struct DownlinkHandlingModel {
    /// Upstream `linkBudgetInMsg`: the link budget of the downlink.
    pub link_budget_in_msg: Input<LinkBudgetMsg>,
    /// Upstream `downlinkOutMsg`: downlink performance.
    pub downlink_out_msg: Output<DownlinkHandlingMsg>,
    /// Upstream `bitRateRequest` \[bit/s\].
    bit_rate_request_bps: f64,
    /// Upstream `packetSizeBits` \[bit\].
    packet_size_bits: f64,
    /// Upstream `maxRetransmissions` \[-\].
    max_retransmissions: u64,
    /// Upstream `receiverAntenna`.
    receiver_antenna: ReceiverSelection,
    /// Upstream `removalPolicy`.
    removal_policy: RemovalPolicy,
    /// Upstream `requireFullPacket`.
    require_full_packet: bool,
    /// Upstream `storageUnitMsgPtrs`: the storage outputs registered, for the
    /// duplicate check.
    storage_unit_sources: Vec<Output<DataStorageStatusMsg>>,
    /// Upstream `storageUnitInMsgs`.
    storage_unit_in_msgs: Vec<Input<DataStorageStatusMsg>>,
    /// Upstream `storageUnitMsgsBuffer`.
    storage_unit_msgs: Vec<DataStorageStatusMsg>,
    /// Upstream `linkBudgetBuffer`.
    link_budget: LinkBudgetMsg,
    /// Upstream `linkBudgetValid`.
    link_budget_valid: bool,
    /// Upstream `downlinkOutBuffer`.
    downlink_out: DownlinkHandlingMsg,
    /// Upstream `previousTime` \[s\].
    previous_time_s: f64,
    /// Upstream `currentTimeStep` \[s\].
    current_time_step_s: f64,
    /// Upstream `availableDataBits` \[bit\].
    available_data_bits: f64,
    /// Upstream `cumulativeAttemptedBits` \[bit\].
    cumulative_attempted_bits: f64,
    /// Upstream `cumulativeRemovedBits` \[bit\].
    cumulative_removed_bits: f64,
    /// Upstream `cumulativeDeliveredBits` \[bit\].
    cumulative_delivered_bits: f64,
    /// Upstream `cumulativeDroppedBits` \[bit\].
    cumulative_dropped_bits: f64,
    /// Upstream `lastBlockedStorageRemovalReason`.
    last_blocked_storage_removal_reason: String,
}

impl Default for DownlinkHandlingModel {
    /// Upstream's constructor defaults.
    fn default() -> Self {
        Self {
            link_budget_in_msg: Input::default(),
            downlink_out_msg: Output::default(),
            bit_rate_request_bps: DEFAULT_BIT_RATE_REQUEST_BPS,
            packet_size_bits: DEFAULT_PACKET_SIZE_BITS,
            max_retransmissions: DEFAULT_MAX_RETRANSMISSIONS,
            receiver_antenna: ReceiverSelection::Auto,
            removal_policy: RemovalPolicy::RemoveAttempted,
            require_full_packet: true,
            storage_unit_sources: Vec::new(),
            storage_unit_in_msgs: Vec::new(),
            storage_unit_msgs: Vec::new(),
            link_budget: LinkBudgetMsg::default(),
            link_budget_valid: false,
            downlink_out: DownlinkHandlingMsg::default(),
            previous_time_s: 0.0,
            current_time_step_s: 0.0,
            available_data_bits: 0.0,
            cumulative_attempted_bits: 0.0,
            cumulative_removed_bits: 0.0,
            cumulative_delivered_bits: 0.0,
            cumulative_dropped_bits: 0.0,
            last_blocked_storage_removal_reason: String::new(),
        }
    }
}

/// Downlink data handling that maps link quality to reliable throughput:
/// upstream `DownlinkHandling`.
pub type DownlinkHandling = DataNodeBase<DownlinkHandlingModel>;

/// Upstream `isFiniteNonNegative`.
fn is_finite_non_negative(value: f64) -> bool {
    value.is_finite() && value >= 0.0
}

/// Upstream `isFinitePositive`.
fn is_finite_positive(value: f64) -> bool {
    value.is_finite() && value > 0.0
}

/// Upstream `clampProbability`: into `[0, 1]`, non-finite to zero.
fn clamp_probability(value: f64) -> f64 {
    if value.is_finite() {
        value.clamp(0.0, 1.0)
    } else {
        0.0
    }
}

/// Upstream `sanitizeNonNegative`: negative or non-finite to zero.
fn sanitize_non_negative(value: f64) -> f64 {
    if value.is_finite() && value >= 0.0 {
        value
    } else {
        0.0
    }
}

/// Upstream `computeBerFromEbN0dB`: BPSK bit error rate over AWGN at
/// `eb_n0_db`.
fn ber_from_eb_n0_db(eb_n0_db: f64) -> f64 {
    if !eb_n0_db.is_finite() {
        return 0.5;
    }
    let eb_n0 = 10f64.powf(eb_n0_db / 10.0);
    if !eb_n0.is_finite() {
        return 0.0;
    }
    if eb_n0 <= 0.0 {
        return 0.5;
    }
    0.5 * libm::erfc(eb_n0.sqrt())
}

impl DownlinkHandling {
    /// A downlink that is on, with upstream's defaults: no requested bit
    /// rate, 256 bit packets, 10 attempts, automatic receiver selection,
    /// removal of attempted data and full-packet gating.
    pub fn new(name: impl Into<String>) -> Self {
        let mut downlink = Self::with_model(name, DownlinkHandlingModel::default());
        downlink.node_data_name = "STORED DATA".to_string();
        downlink
    }

    /// Upstream `addStorageUnitToDownlink`: connects `storage` as a storage
    /// input; returns `false`, with a warning, if it is already connected.
    pub fn add_storage_unit_to_downlink(&mut self, storage: &Output<DataStorageStatusMsg>) -> bool {
        let model = &mut self.model;
        if model
            .storage_unit_sources
            .iter()
            .any(|source| Arc::ptr_eq(&source.slot(), &storage.slot()))
        {
            log::warn!(
                "{}: DownlinkHandling.addStorageUnitToDownlink: duplicate storage message ignored.",
                self.name
            );
            return false;
        }
        let mut input = Input::default();
        storage.connect_to(&mut input);
        model.storage_unit_sources.push(storage.clone());
        model.storage_unit_in_msgs.push(input);
        true
    }

    /// Upstream `setBitRateRequest` \[bit/s\]: the raw channel bit rate.
    ///
    /// # Errors
    ///
    /// Unless finite and non-negative; the previous value is kept.
    pub fn set_bit_rate_request(&mut self, bit_rate_request_bps: f64) -> Result<(), String> {
        if !is_finite_non_negative(bit_rate_request_bps) {
            return Err(
                "DownlinkHandling.setBitRateRequest: value must be finite and >= 0 [bit/s]."
                    .to_string(),
            );
        }
        self.model.bit_rate_request_bps = bit_rate_request_bps;
        Ok(())
    }

    /// Upstream `setPacketSizeBits` \[bit\]: packet size of the BER-to-PER
    /// conversion.
    ///
    /// # Errors
    ///
    /// Unless finite and positive; the previous value is kept.
    pub fn set_packet_size_bits(&mut self, packet_size_bits: f64) -> Result<(), String> {
        if !is_finite_positive(packet_size_bits) {
            return Err(
                "DownlinkHandling.setPacketSizeBits: value must be finite and > 0 [bit]."
                    .to_string(),
            );
        }
        self.model.packet_size_bits = packet_size_bits;
        Ok(())
    }

    /// Upstream `setMaxRetransmissions`: most transmission attempts per
    /// packet in the ARQ model.
    ///
    /// # Errors
    ///
    /// If below 1; the previous value is kept.
    pub fn set_max_retransmissions(&mut self, max_retransmissions: u64) -> Result<(), String> {
        if max_retransmissions < 1 {
            return Err("DownlinkHandling.setMaxRetransmissions: value must be >= 1.".to_string());
        }
        self.model.max_retransmissions = max_retransmissions;
        Ok(())
    }

    /// Upstream `setReceiverAntenna`: which receiver of the link budget to
    /// use.
    pub fn set_receiver_antenna(&mut self, receiver: ReceiverSelection) {
        self.model.receiver_antenna = receiver;
    }

    /// Upstream `setRemovalPolicy`.
    pub fn set_removal_policy(&mut self, removal_policy: RemovalPolicy) {
        self.model.removal_policy = removal_policy;
    }

    /// Upstream `setRequireFullPacket`: wait for at least one full packet in
    /// storage before downlinking.
    pub fn set_require_full_packet(&mut self, require_full_packet: bool) {
        self.model.require_full_packet = require_full_packet;
    }

    /// Upstream `getBitRateRequest` \[bit/s\].
    pub fn bit_rate_request_bps(&self) -> f64 {
        self.model.bit_rate_request_bps
    }

    /// Upstream `getPacketSizeBits` \[bit\].
    pub fn packet_size_bits(&self) -> f64 {
        self.model.packet_size_bits
    }

    /// Upstream `getMaxRetransmissions`.
    pub fn max_retransmissions(&self) -> u64 {
        self.model.max_retransmissions
    }

    /// Upstream `getReceiverAntenna`.
    pub fn receiver_antenna(&self) -> ReceiverSelection {
        self.model.receiver_antenna
    }

    /// Upstream `getRemovalPolicy`.
    pub fn removal_policy(&self) -> RemovalPolicy {
        self.model.removal_policy
    }

    /// Upstream `getRequireFullPacket`.
    pub fn require_full_packet(&self) -> bool {
        self.model.require_full_packet
    }

    /// The downlink diagnostics of the last update (upstream's
    /// `getCurrent*` getters).
    pub fn current(&self) -> &DownlinkHandlingMsg {
        &self.model.downlink_out
    }
}

impl DownlinkHandlingModel {
    /// Upstream `selectStorageSelection`: the partition with the most data
    /// across the storage units, the first of equals; a unit's total for a
    /// unit that lists no partitions.
    fn select_storage(&self) -> StorageSelection {
        let mut selection = StorageSelection::default();
        for (unit, storage) in self.storage_unit_msgs.iter().enumerate() {
            for (partition, &bits) in storage.stored_data_bits.iter().enumerate() {
                let bits = sanitize_non_negative(bits);
                if bits > selection.available_bits {
                    selection = StorageSelection {
                        storage_unit: Some(unit),
                        partition: Some(partition),
                        available_bits: bits,
                    };
                }
            }
            if storage.stored_data_bits.is_empty() {
                let bits = sanitize_non_negative(storage.storage_level_bits);
                if bits > selection.available_bits {
                    selection = StorageSelection {
                        storage_unit: Some(unit),
                        partition: None,
                        available_bits: bits,
                    };
                }
            }
        }
        selection
    }

    /// Upstream `isStorageRemovalRouteSafe`: a data-node message can reach
    /// the selected partition unambiguously.
    fn is_storage_removal_route_safe(&self, selection: &StorageSelection) -> bool {
        selection.storage_unit.is_none()
            || (self.storage_unit_msgs.len() <= 1 && selection.partition.is_some())
    }

    /// Upstream `setDataNameFromStorageSelection`: the selected partition's
    /// name, if it has one.
    ///
    /// # Panics
    ///
    /// If the name has [`DATA_NAME_SIZE`] characters or more (upstream
    /// `bskError`).
    fn selected_data_name(&self, selection: &StorageSelection) -> Option<String> {
        let storage = self.storage_unit_msgs.get(selection.storage_unit?)?;
        let name = storage.stored_data_name.get(selection.partition?)?;
        if name.is_empty() {
            return None;
        }
        assert!(
            name.len() < DATA_NAME_SIZE,
            "DownlinkHandling: selected storage partition name is {} characters, but the \
             destination buffer supports at most {} characters.",
            name.len(),
            DATA_NAME_SIZE - 1
        );
        Some(name.clone())
    }

    /// Upstream `selectReceiver`: the selected receiver (1 or 2) and its
    /// linear CNR, or 0 and zero when none receives.
    fn select_receiver(&self) -> (u32, f64) {
        if !self.link_budget_valid {
            return (0, 0.0);
        }
        let link = &self.link_budget;
        // Upstream `isReceiverState`: Rx or RxTx.
        let usable = |state: AntennaState, cnr: f64| {
            if state.receives() && is_finite_positive(cnr) {
                cnr
            } else {
                0.0
            }
        };
        let cnr_1 = usable(link.antenna_state_1, link.cnr_1);
        let cnr_2 = usable(link.antenna_state_2, link.cnr_2);
        match self.receiver_antenna {
            ReceiverSelection::Antenna1 if cnr_1 > 0.0 => (1, cnr_1),
            ReceiverSelection::Antenna2 if cnr_2 > 0.0 => (2, cnr_2),
            ReceiverSelection::Antenna1 | ReceiverSelection::Antenna2 => (0, 0.0),
            ReceiverSelection::Auto if cnr_1 >= cnr_2 && cnr_1 > 0.0 => (1, cnr_1),
            ReceiverSelection::Auto if cnr_2 > 0.0 => (2, cnr_2),
            ReceiverSelection::Auto => (0, 0.0),
        }
    }

    /// The zeroed diagnostics with the configuration filled in.
    fn configured_output(&self) -> DownlinkHandlingMsg {
        DownlinkHandlingMsg {
            max_retransmissions: self.max_retransmissions,
            removal_policy: self.removal_policy,
            bit_rate_request_bps: self.bit_rate_request_bps,
            packet_size_bits: self.packet_size_bits,
            ..DownlinkHandlingMsg::default()
        }
    }

    /// Copies the cumulative totals into the diagnostics.
    fn publish_cumulative(&mut self) {
        let out = &mut self.downlink_out;
        out.cumulative_attempted_bits = sanitize_non_negative(self.cumulative_attempted_bits);
        out.cumulative_removed_bits = sanitize_non_negative(self.cumulative_removed_bits);
        out.cumulative_delivered_bits = sanitize_non_negative(self.cumulative_delivered_bits);
        out.cumulative_dropped_bits = sanitize_non_negative(self.cumulative_dropped_bits);
    }

    /// Logs why a selected storage target cannot be acted on, once per
    /// reason.
    fn report_blocked_route(
        &mut self,
        node_name: &str,
        selection: &StorageSelection,
        name_valid: bool,
    ) {
        let reason = if !name_valid {
            "selected storage target does not provide an explicit partition name"
        } else if self.storage_unit_msgs.len() > 1 {
            "multiple linked storage units cannot be safely targeted through DataNodeUsageMsgPayload"
        } else if selection.partition.is_none() {
            "selected storage target has no explicit partition name"
        } else {
            "selected storage route is invalid"
        };
        if reason != self.last_blocked_storage_removal_reason {
            log::warn!(
                "{node_name}: DownlinkHandling: storage removal is forced to zero because \
                 {reason}. Conservative behavior: if the linked storage-route mapping is not \
                 uniquely actionable, the module blocks removal for that step to avoid silent \
                 data corruption."
            );
            self.last_blocked_storage_removal_reason = reason.to_string();
        }
    }
}

/// Copies an antenna name of the link budget.
///
/// # Panics
///
/// If it has [`ANTENNA_NAME_SIZE`] characters or more (upstream
/// `bskError`).
fn antenna_name(name: &str) -> String {
    assert!(
        name.len() < ANTENNA_NAME_SIZE,
        "DownlinkHandling: antenna name is not null-terminated within {ANTENNA_NAME_SIZE} \
         characters."
    );
    name.to_string()
}

impl DataNodeModel for DownlinkHandlingModel {
    fn evaluate_data_model(
        &mut self,
        node: DataNodeFields<'_>,
        usage: &mut DataNodeUsageMsg,
        current_time_s: f64,
    ) {
        *usage = DataNodeUsageMsg::default();
        let time_step_s = current_time_s - self.previous_time_s;
        self.current_time_step_s = if is_finite_non_negative(time_step_s) {
            time_step_s
        } else {
            0.0
        };

        let mut out = self.configured_output();
        out.time_step_s = self.current_time_step_s;
        out.bandwidth_hz = sanitize_non_negative(self.link_budget.bandwidth_hz);

        // One storage target for both the diagnostics and, if safe, the
        // removal.
        let selection = self.select_storage();
        self.available_data_bits = selection.available_bits;
        let selected_name = self.selected_data_name(&selection);
        let name_valid = selected_name.is_some();
        *node.node_data_name = selected_name.unwrap_or_default();
        out.data_name.clone_from(node.node_data_name);
        out.available_data_bits = self.available_data_bits;
        let route_safe = name_valid && self.is_storage_removal_route_safe(&selection);
        if !route_safe && selection.storage_unit.is_some() {
            let node_name = node.node_data_name.clone();
            self.report_blocked_route(&node_name, &selection, name_valid);
        } else {
            self.last_blocked_storage_removal_reason.clear();
        }
        if route_safe {
            usage.data_name.clone_from(node.node_data_name);
        }
        // An unnamed removal creates no partition in a storage unit.

        let (receiver, cnr) = self.select_receiver();
        out.receiver_index = receiver;
        out.cnr = sanitize_non_negative(cnr);
        match receiver {
            1 => {
                out.receiver_antenna_name = antenna_name(&self.link_budget.antenna_name_1);
                out.transmitter_antenna_name = antenna_name(&self.link_budget.antenna_name_2);
            }
            2 => {
                out.receiver_antenna_name = antenna_name(&self.link_budget.antenna_name_2);
                out.transmitter_antenna_name = antenna_name(&self.link_budget.antenna_name_1);
            }
            _ => {}
        }

        // A usable RF path needs written link data, a positive CNR, overlap
        // bandwidth, bit rate and packet size.
        let link_active = self.link_budget_valid
            && is_finite_positive(cnr)
            && is_finite_positive(out.bandwidth_hz)
            && is_finite_positive(self.bit_rate_request_bps)
            && is_finite_positive(self.packet_size_bits);
        out.link_active = link_active;

        let max_attempts = self.max_retransmissions as f64;
        let (mut ber, mut per) = (0.0, 0.0);
        let (mut packet_success, mut packet_drop) = (0.0, 0.0);
        let mut expected_attempts = max_attempts;
        let (mut attempted_potential, mut removal_potential) = (0.0, 0.0);
        let (mut delivered_potential, mut dropped_potential) = (0.0, 0.0);
        if link_active {
            // Per-bit energy, so the BER follows the requested bit rate.
            out.cnr_db = 10.0 * cnr.log10();
            out.c_n0_dbhz = out.cnr_db + 10.0 * out.bandwidth_hz.log10();
            out.eb_n0_db = out.c_n0_dbhz - 10.0 * self.bit_rate_request_bps.log10();
            ber = clamp_probability(ber_from_eb_n0_db(out.eb_n0_db));

            // A packet fails on any wrong bit.
            per = if ber >= 1.0 {
                1.0
            } else if ber <= 0.0 {
                0.0
            } else {
                clamp_probability(1.0 - (self.packet_size_bits * (-ber).ln_1p()).exp())
            };

            let success_one_attempt = clamp_probability(1.0 - per);
            packet_drop = clamp_probability(per.powf(max_attempts));
            packet_success = clamp_probability(1.0 - packet_drop);
            // Expected ARQ attempts slow the source-packet rate below the
            // channel rate.
            expected_attempts = if success_one_attempt <= 0.0 {
                max_attempts
            } else {
                (packet_success / success_one_attempt).clamp(1.0, max_attempts)
            };

            attempted_potential = self.bit_rate_request_bps;
            removal_potential = attempted_potential / expected_attempts;
            delivered_potential = removal_potential * packet_success;
            dropped_potential = removal_potential - delivered_potential;
        }
        out.ber = ber;
        out.per = per;
        out.packet_success_prob = packet_success;
        out.packet_drop_prob = packet_drop;
        out.expected_attempts_per_packet = expected_attempts;

        // Packet gating, and never more than the stored data in a step.
        let enough_for_packet =
            !self.require_full_packet || self.available_data_bits >= self.packet_size_bits;
        let mut scale = 0.0;
        if removal_potential > 0.0
            && self.current_time_step_s > 0.0
            && enough_for_packet
            && route_safe
        {
            let available_rate = self.available_data_bits / self.current_time_step_s;
            if is_finite_non_negative(available_rate) {
                scale = (available_rate / removal_potential).clamp(0.0, 1.0);
            }
        }
        let attempted_rate = sanitize_non_negative(attempted_potential * scale);
        let modeled_removal_rate = sanitize_non_negative(removal_potential * scale);
        let delivered_rate = sanitize_non_negative(delivered_potential * scale);
        let dropped_rate = sanitize_non_negative(dropped_potential * scale);

        // Delivered-only removal keeps dropped data onboard for later.
        let removal_rate = match self.removal_policy {
            RemovalPolicy::RemoveDeliveredOnly => delivered_rate,
            RemovalPolicy::RemoveAttempted => modeled_removal_rate,
        };
        usage.baud_rate_bps = -removal_rate;
        out.attempted_data_rate_bps = attempted_rate;
        out.storage_removal_rate_bps = removal_rate;
        out.delivered_data_rate_bps = delivered_rate;
        out.dropped_data_rate_bps = dropped_rate;
        out.estimated_remaining_data_bits = sanitize_non_negative(
            self.available_data_bits - removal_rate * self.current_time_step_s,
        );

        let step = self.current_time_step_s;
        if step > 0.0 {
            self.cumulative_attempted_bits += attempted_rate * step;
            self.cumulative_removed_bits += removal_rate * step;
            self.cumulative_delivered_bits += delivered_rate * step;
            self.cumulative_dropped_bits += dropped_rate * step;
        }
        self.downlink_out = out;
        self.publish_cumulative();

        if current_time_s.is_finite() {
            self.previous_time_s = current_time_s;
        } else {
            self.previous_time_s += self.current_time_step_s;
        }
    }

    /// Upstream `customReset`.
    fn custom_reset(&mut self, _node: DataNodeFields<'_>, current_sim_nanos: u64) {
        self.previous_time_s = current_sim_nanos as f64 * 1.0e-9;
        self.current_time_step_s = 0.0;
        self.cumulative_attempted_bits = 0.0;
        self.cumulative_removed_bits = 0.0;
        self.cumulative_delivered_bits = 0.0;
        self.cumulative_dropped_bits = 0.0;
        self.last_blocked_storage_removal_reason.clear();
        self.downlink_out = DownlinkHandlingMsg::default();
        // The setters keep the configuration valid, so upstream's restoring
        // of defaults for an invalid one cannot trigger here.
        if !self.link_budget_in_msg.is_connected() {
            log::warn!("DownlinkHandling.linkBudgetInMsg is not linked.");
        }
        if self.storage_unit_in_msgs.is_empty() {
            log::warn!("DownlinkHandling has no storageUnitInMsgs linked.");
        }
    }

    /// Upstream `customReadMessages`.
    fn custom_read_messages(&mut self) -> bool {
        self.storage_unit_msgs.clear();
        self.storage_unit_msgs
            .extend(self.storage_unit_in_msgs.iter().map(Input::read));
        self.link_budget_valid = self.link_budget_in_msg.is_connected();
        self.link_budget = if self.link_budget_valid {
            self.link_budget_in_msg.read()
        } else {
            LinkBudgetMsg::default()
        };
        true
    }

    /// Upstream `customWriteMessages`: while the node is off, publish zeroed
    /// diagnostics and keep the step timer on the clock.
    fn custom_write_messages(&mut self, node: DataNodeFields<'_>, current_sim_nanos: u64) {
        if node.data_status == 0 {
            self.previous_time_s = current_sim_nanos as f64 * 1.0e-9;
            self.current_time_step_s = 0.0;
            self.downlink_out = self.configured_output();
            self.publish_cumulative();
        }
        self.downlink_out_msg.write(self.downlink_out.clone());
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `downlinkHandling/_UnitTest/test_downlinkHandling.py`
    //! with its expected values and tolerances, and
    //! `linkBudget/_UnitTest/test_antennaProducerScheduling.py`, which runs
    //! the ground location, antennas, link budget, downlink and storage
    //! together.
    //!
    //! The modules are stepped by hand in upstream's task order. In the
    //! scheduling test a disabled task is one whose modules are not stepped,
    //! the stationary spacecraft is a written state message, and upstream's
    //! write-time checks (`isWritten`, `timesWritten`) have no counterpart.
    //! `test_downlink_setter_validation_rejects_invalid_inputs`'s receiver
    //! 4 and removal policy 2 are not representable here.

    use nalgebra::Vector3;

    use super::{DownlinkHandling, ReceiverSelection};
    use crate::Module;
    use crate::communication::link_budget::LinkBudget;
    use crate::communication::simple_antenna::{SimpleAntenna, SimpleAntennaConfig};
    use crate::environment::ground_location::{GroundLocation, GroundLocationConfig, SiteLocation};
    use crate::messages::{
        AntennaLogMsg, AntennaState, DataNodeUsageMsg, DataStorageStatusMsg, DeviceCommandMsg,
        DownlinkHandlingMsg, LinkBudgetMsg, Output, RemovalPolicy, SpacecraftStateMsg,
    };
    use crate::onboard_data_handling::partitioned_storage_unit::PartitionedStorageUnit;
    use crate::onboard_data_handling::simple_storage_unit::SimpleStorageUnit;
    use crate::onboard_data_handling::test_support::{TaskClock, context};
    use crate::utilities::astro_constants::REQ_EARTH;
    use crate::utilities::geodetic_conversion::Lla;
    use crate::utilities::rigid_body_kinematics::dcm_to_mrp;

    const SECOND_NANOS: u64 = 1_000_000_000;

    /// `pytest.approx(expected, rel=rel, abs=abs)`.
    #[track_caller]
    fn assert_approx(actual: f64, expected: f64, rel: f64, abs: f64) {
        let tolerance = (rel * expected.abs()).max(abs);
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} vs {expected}, tolerance {tolerance}"
        );
    }

    /// Upstream `python_equivalent_from_link`.
    struct Equivalent {
        ber: f64,
        per: f64,
        expected_attempts: f64,
        storage_removal_rate: f64,
        delivered_rate: f64,
        dropped_rate: f64,
    }

    fn python_equivalent_from_link(
        cnr: f64,
        bandwidth: f64,
        bit_rate: f64,
        packet_bits: f64,
        max_retx: f64,
        removal_policy: RemovalPolicy,
    ) -> Equivalent {
        let cnr_db = 10.0 * cnr.log10();
        let c_n0_dbhz = cnr_db + 10.0 * bandwidth.log10();
        let eb_n0_db = c_n0_dbhz - 10.0 * bit_rate.log10();
        let eb_n0 = 10f64.powf(eb_n0_db / 10.0);
        // q(x) = 0.5 erfc(x / sqrt(2)) at x = sqrt(2 Eb/N0).
        let ber = 0.5 * libm::erfc((2.0 * eb_n0).sqrt() / 2f64.sqrt());
        let per = 1.0 - (1.0 - ber).powf(packet_bits);
        let one_try_success = 1.0 - per;
        let packet_drop = per.powf(max_retx);
        let packet_success = 1.0 - packet_drop;
        let expected_attempts = if one_try_success <= 0.0 {
            max_retx
        } else {
            packet_success / one_try_success
        };
        let modeled_removal = bit_rate / expected_attempts;
        let delivered_rate = modeled_removal * packet_success;
        Equivalent {
            ber,
            per,
            expected_attempts,
            storage_removal_rate: match removal_policy {
                RemovalPolicy::RemoveDeliveredOnly => delivered_rate,
                RemovalPolicy::RemoveAttempted => modeled_removal,
            },
            delivered_rate,
            dropped_rate: modeled_removal - delivered_rate,
        }
    }

    /// Upstream `make_storage_status_msg`.
    fn storage_status(level_bits: f64, partitions: &[(&str, f64)]) -> Output<DataStorageStatusMsg> {
        Output::new(DataStorageStatusMsg {
            storage_level_bits: level_bits,
            stored_data_name: partitions
                .iter()
                .map(|(name, _)| name.to_string())
                .collect(),
            stored_data_bits: partitions.iter().map(|(_, bits)| *bits).collect(),
            ..DataStorageStatusMsg::default()
        })
    }

    /// Upstream's link payload: "TX_Ant" and "RX_Ant" at 2.2 GHz.
    fn link_payload(
        cnr_1: f64,
        cnr_2: f64,
        state_1: AntennaState,
        state_2: AntennaState,
        bandwidth_hz: f64,
    ) -> Output<LinkBudgetMsg> {
        Output::new(LinkBudgetMsg {
            antenna_name_1: "TX_Ant".to_string(),
            antenna_name_2: "RX_Ant".to_string(),
            antenna_state_1: state_1,
            antenna_state_2: state_2,
            cnr_1,
            cnr_2,
            bandwidth_hz,
            frequency_hz: 2.2e9,
            ..LinkBudgetMsg::default()
        })
    }

    /// Upstream `run_downlink_case`'s arguments, with its defaults.
    struct Case {
        cnr_1: f64,
        cnr_2: f64,
        state_1: AntennaState,
        state_2: AntennaState,
        bandwidth_hz: f64,
        bit_rate_bps: f64,
        packet_bits: f64,
        max_retx: u64,
        receiver: ReceiverSelection,
        removal_policy: RemovalPolicy,
        initial_bits: f64,
        stop_time_s: u64,
    }

    impl Default for Case {
        fn default() -> Self {
            Self {
                cnr_1: 0.0,
                cnr_2: 0.5,
                state_1: AntennaState::Tx,
                state_2: AntennaState::Rx,
                bandwidth_hz: 1.0e6,
                bit_rate_bps: 1.0e5,
                packet_bits: 256.0,
                max_retx: 10,
                receiver: ReceiverSelection::Antenna2,
                removal_policy: RemovalPolicy::RemoveAttempted,
                initial_bits: 1.0e9,
                stop_time_s: 3,
            }
        }
    }

    /// Per-tick records of upstream's recorders.
    struct Logs {
        node: Vec<DataNodeUsageMsg>,
        storage: Vec<DataStorageStatusMsg>,
        downlink: Vec<DownlinkHandlingMsg>,
    }

    /// Upstream `run_downlink_case`: the downlink draining a simple storage
    /// unit over a written link budget, at a 1 s task rate.
    fn run_downlink_case(case: &Case) -> Logs {
        let mut module = DownlinkHandling::new("downlink");
        module.set_bit_rate_request(case.bit_rate_bps).unwrap();
        module.set_packet_size_bits(case.packet_bits).unwrap();
        module.set_max_retransmissions(case.max_retx).unwrap();
        module.set_receiver_antenna(case.receiver);
        module.set_removal_policy(case.removal_policy);
        module.set_require_full_packet(true);

        let mut storage = SimpleStorageUnit::new("storage");
        storage.storage_capacity_bits =
            (2.0 * case.initial_bits).max(case.initial_bits + 1.0e6) as i64;
        module
            .node_data_out_msg
            .connect_to(storage.add_data_node_to_model());
        assert!(module.add_storage_unit_to_downlink(&storage.storage_unit_data_out_msg));
        storage.set_data_buffer(case.initial_bits as i64);
        let link = link_payload(
            case.cnr_1,
            case.cnr_2,
            case.state_1,
            case.state_2,
            case.bandwidth_hz,
        );
        link.connect_to(&mut module.model.link_budget_in_msg);

        module.init();
        storage.init();
        let mut logs = Logs {
            node: Vec::new(),
            storage: Vec::new(),
            downlink: Vec::new(),
        };
        for tick in TaskClock::new(SECOND_NANOS).run_until(case.stop_time_s * SECOND_NANOS) {
            module.update(&context(tick));
            storage.update(&context(tick));
            logs.node.push(module.node_data_out_msg.read());
            logs.storage.push(storage.storage_unit_data_out_msg.read());
            logs.downlink.push(module.model.downlink_out_msg.read());
        }
        logs
    }

    /// Upstream `test_downlink_matches_python_equivalent`.
    #[test]
    fn downlink_matches_python_equivalent() {
        let logs = run_downlink_case(&Case::default());
        let expected = python_equivalent_from_link(
            0.5,
            1.0e6,
            1.0e5,
            256.0,
            10.0,
            RemovalPolicy::RemoveAttempted,
        );
        let last = logs.downlink.last().unwrap();
        assert!(last.link_active);
        assert_eq!(last.removal_policy, RemovalPolicy::RemoveAttempted);
        assert_approx(last.ber, expected.ber, 1e-12, 1e-15);
        assert_approx(last.per, expected.per, 1e-12, 1e-15);
        assert_approx(
            last.expected_attempts_per_packet,
            expected.expected_attempts,
            1e-12,
            1e-15,
        );
        assert_approx(
            last.delivered_data_rate_bps,
            expected.delivered_rate,
            1e-12,
            1e-9,
        );
        assert_approx(
            last.storage_removal_rate_bps,
            expected.storage_removal_rate,
            1e-12,
            1e-9,
        );
        assert_approx(
            last.dropped_data_rate_bps,
            expected.dropped_rate,
            1e-12,
            1e-9,
        );
        assert_approx(
            logs.node.last().unwrap().baud_rate_bps,
            -expected.storage_removal_rate,
            1e-12,
            1e-9,
        );
    }

    /// Upstream `test_downlink_invalid_link_outputs_zero_flow`.
    #[test]
    fn downlink_invalid_link_outputs_zero_flow() {
        let logs = run_downlink_case(&Case {
            cnr_2: 0.0,
            ..Case::default()
        });
        let last = logs.downlink.last().unwrap();
        assert!(!last.link_active);
        assert_approx(last.storage_removal_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.delivered_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.dropped_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(logs.node.last().unwrap().baud_rate_bps, 0.0, 0.0, 1e-12);
    }

    /// The downlink of upstream's enable/disable tests: 4 attempts,
    /// receiver 2, a simple storage unit of 1e9 bits and a device command.
    struct SwitchedDownlink {
        module: DownlinkHandling,
        storage: SimpleStorageUnit,
        command: Output<DeviceCommandMsg>,
        _link: Output<LinkBudgetMsg>,
        clock: TaskClock,
        node: Vec<DataNodeUsageMsg>,
        downlink: Vec<DownlinkHandlingMsg>,
    }

    impl SwitchedDownlink {
        fn new() -> Self {
            let mut module = DownlinkHandling::new("downlink");
            module.set_bit_rate_request(1.0e5).unwrap();
            module.set_packet_size_bits(256.0).unwrap();
            module.set_max_retransmissions(4).unwrap();
            module.set_receiver_antenna(ReceiverSelection::Antenna2);
            let mut storage = SimpleStorageUnit::new("storage");
            storage.storage_capacity_bits = 2_000_000_000;
            module
                .node_data_out_msg
                .connect_to(storage.add_data_node_to_model());
            module.add_storage_unit_to_downlink(&storage.storage_unit_data_out_msg);
            storage.set_data_buffer(1_000_000_000);
            let link = link_payload(0.0, 0.5, AntennaState::Tx, AntennaState::Rx, 1.0e6);
            link.connect_to(&mut module.model.link_budget_in_msg);
            let command = Output::new(DeviceCommandMsg { device_cmd: 1 });
            command.connect_to(&mut module.node_status_in_msg);
            module.init();
            storage.init();
            Self {
                module,
                storage,
                command,
                _link: link,
                clock: TaskClock::new(SECOND_NANOS),
                node: Vec::new(),
                downlink: Vec::new(),
            }
        }

        fn run_until(&mut self, stop_s: u64) {
            for tick in self.clock.run_until(stop_s * SECOND_NANOS) {
                self.module.update(&context(tick));
                self.storage.update(&context(tick));
                self.node.push(self.module.node_data_out_msg.read());
                self.downlink
                    .push(self.module.model.downlink_out_msg.read());
            }
        }

        fn last(&self) -> &DownlinkHandlingMsg {
            self.downlink.last().unwrap()
        }
    }

    /// Upstream `test_downlink_disabling_node_clears_diagnostics`.
    #[test]
    fn downlink_disabling_node_clears_diagnostics() {
        let mut chain = SwitchedDownlink::new();
        chain.run_until(1);
        assert!(chain.last().link_active);
        assert!(chain.last().storage_removal_rate_bps > 0.0);
        assert!(chain.node.last().unwrap().baud_rate_bps < 0.0);

        chain.command.write(DeviceCommandMsg { device_cmd: 0 });
        chain.run_until(2);
        let last = chain.last();
        assert!(!last.link_active);
        assert_eq!(last.receiver_index, 0);
        assert_approx(last.storage_removal_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.delivered_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.dropped_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.ber, 0.0, 0.0, 1e-12);
        assert_approx(last.per, 0.0, 0.0, 1e-12);
        assert_approx(chain.node.last().unwrap().baud_rate_bps, 0.0, 0.0, 1e-12);
    }

    /// Upstream `test_downlink_reenable_does_not_integrate_disabled_downtime`.
    #[test]
    fn downlink_reenable_does_not_integrate_disabled_downtime() {
        let mut chain = SwitchedDownlink::new();
        chain.run_until(1);
        let baseline_rate = chain.last().storage_removal_rate_bps;
        let baseline_removed = chain.last().cumulative_removed_bits;
        assert_approx(chain.last().time_step_s, 1.0, 0.0, 1e-12);
        assert!(baseline_rate > 0.0);

        chain.command.write(DeviceCommandMsg { device_cmd: 0 });
        chain.run_until(4);
        let removed_after_off = chain.last().cumulative_removed_bits;
        assert_approx(removed_after_off, baseline_removed, 0.0, 1e-9);

        chain.command.write(DeviceCommandMsg { device_cmd: 1 });
        chain.run_until(5);
        assert_approx(chain.last().time_step_s, 1.0, 0.0, 1e-12);
        assert_approx(
            chain.last().storage_removal_rate_bps,
            baseline_rate,
            1e-12,
            1e-9,
        );
        let reenable_removed = chain.last().cumulative_removed_bits - removed_after_off;
        assert_approx(reenable_removed, baseline_rate, 1e-12, 1e-6);
    }

    /// Upstream `test_downlink_retry_limit_changes_storage_draw_not_goodput`.
    #[test]
    fn downlink_retry_limit_changes_storage_draw_not_goodput() {
        let m1 = run_downlink_case(&Case {
            max_retx: 1,
            ..Case::default()
        });
        let m8 = run_downlink_case(&Case {
            max_retx: 8,
            ..Case::default()
        });
        let (m1, m8) = (m1.downlink.last().unwrap(), m8.downlink.last().unwrap());
        assert!(m1.storage_removal_rate_bps > m8.storage_removal_rate_bps);
        assert!(m1.packet_drop_prob > m8.packet_drop_prob);
        assert_approx(
            m1.delivered_data_rate_bps,
            m8.delivered_data_rate_bps,
            1e-12,
            1e-9,
        );
    }

    /// Upstream `test_downlink_remove_delivered_only_retains_undelivered_bits`.
    #[test]
    fn downlink_remove_delivered_only_retains_undelivered_bits() {
        let case = |removal_policy| Case {
            max_retx: 4,
            removal_policy,
            ..Case::default()
        };
        let attempted = run_downlink_case(&case(RemovalPolicy::RemoveAttempted));
        let delivered = run_downlink_case(&case(RemovalPolicy::RemoveDeliveredOnly));
        let (a, d) = (
            attempted.downlink.last().unwrap(),
            delivered.downlink.last().unwrap(),
        );
        assert_eq!(a.removal_policy, RemovalPolicy::RemoveAttempted);
        assert_eq!(d.removal_policy, RemovalPolicy::RemoveDeliveredOnly);
        assert_approx(
            a.attempted_data_rate_bps,
            d.attempted_data_rate_bps,
            1e-12,
            1e-9,
        );
        assert_approx(
            a.delivered_data_rate_bps,
            d.delivered_data_rate_bps,
            1e-12,
            1e-9,
        );
        assert_approx(
            a.dropped_data_rate_bps,
            d.dropped_data_rate_bps,
            1e-12,
            1e-9,
        );
        assert!(a.storage_removal_rate_bps > d.storage_removal_rate_bps);
        assert_approx(
            d.storage_removal_rate_bps,
            d.delivered_data_rate_bps,
            1e-12,
            1e-9,
        );
        assert_approx(
            attempted.node.last().unwrap().baud_rate_bps,
            -a.storage_removal_rate_bps,
            1e-12,
            1e-9,
        );
        assert_approx(
            delivered.node.last().unwrap().baud_rate_bps,
            -d.storage_removal_rate_bps,
            1e-12,
            1e-9,
        );
        assert!(
            delivered.storage.last().unwrap().storage_level_bits
                > attempted.storage.last().unwrap().storage_level_bits
        );
    }

    /// Upstream `test_downlink_storage_limited_case_caps_rate_and_drains_storage`.
    #[test]
    fn downlink_storage_limited_case_caps_rate_and_drains_storage() {
        let initial_bits = 300.0;
        let logs = run_downlink_case(&Case {
            cnr_2: 100.0,
            initial_bits,
            stop_time_s: 1,
            ..Case::default()
        });
        let last = logs.downlink.last().unwrap();
        let expected_cap_rate = initial_bits / last.time_step_s;
        assert_approx(
            last.storage_removal_rate_bps,
            expected_cap_rate,
            1e-12,
            1e-9,
        );
        assert_approx(last.estimated_remaining_data_bits, 0.0, 0.0, 1.0);
        assert_approx(
            logs.storage.last().unwrap().storage_level_bits,
            0.0,
            0.0,
            1.0,
        );
    }

    /// Upstream `test_downlink_auto_receiver_selects_valid_rx_path`.
    #[test]
    fn downlink_auto_receiver_selects_valid_rx_path() {
        let logs = run_downlink_case(&Case {
            cnr_1: 0.8,
            cnr_2: 0.0,
            state_1: AntennaState::Rx,
            state_2: AntennaState::Tx,
            receiver: ReceiverSelection::Auto,
            ..Case::default()
        });
        let last = logs.downlink.last().unwrap();
        assert!(last.link_active);
        assert_eq!(last.receiver_index, 1);
        assert_eq!(last.receiver_antenna_name, "TX_Ant");
        assert_eq!(last.transmitter_antenna_name, "RX_Ant");
    }

    /// Upstream `test_downlink_setter_validation_rejects_invalid_inputs`.
    #[test]
    fn downlink_setter_validation_rejects_invalid_inputs() {
        let mut module = DownlinkHandling::new("downlink");
        module.set_bit_rate_request(1.0e5).unwrap();
        module.set_packet_size_bits(1024.0).unwrap();
        module.set_max_retransmissions(6).unwrap();
        module.set_receiver_antenna(ReceiverSelection::Antenna2);
        module.set_removal_policy(RemovalPolicy::RemoveDeliveredOnly);
        module.set_require_full_packet(true);

        assert!(module.set_bit_rate_request(-1.0).is_err());
        assert!(module.set_bit_rate_request(f64::NAN).is_err());
        assert!(module.set_packet_size_bits(0.0).is_err());
        assert!(module.set_packet_size_bits(f64::INFINITY).is_err());
        assert!(module.set_max_retransmissions(0).is_err());

        assert_approx(module.bit_rate_request_bps(), 1.0e5, 0.0, 1e-12);
        assert_approx(module.packet_size_bits(), 1024.0, 0.0, 1e-12);
        assert_eq!(module.max_retransmissions(), 6);
        assert_eq!(module.receiver_antenna(), ReceiverSelection::Antenna2);
        assert_eq!(module.removal_policy(), RemovalPolicy::RemoveDeliveredOnly);
        assert!(module.require_full_packet());
    }

    /// Upstream `test_downlink_forced_receiver_invalid_path_disables_link`.
    #[test]
    fn downlink_forced_receiver_invalid_path_disables_link() {
        let logs = run_downlink_case(&Case {
            cnr_1: 0.8,
            cnr_2: 0.0,
            state_1: AntennaState::Rx,
            state_2: AntennaState::Tx,
            receiver: ReceiverSelection::Antenna2,
            initial_bits: 1.0e8,
            ..Case::default()
        });
        let last = logs.downlink.last().unwrap();
        assert_eq!(last.receiver_index, 0);
        assert!(!last.link_active);
        assert_approx(last.storage_removal_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(logs.node.last().unwrap().baud_rate_bps, 0.0, 0.0, 1e-12);
    }

    /// Upstream `test_downlink_duplicate_storage_message_is_rejected`.
    #[test]
    fn downlink_duplicate_storage_message_is_rejected() {
        let mut module = DownlinkHandling::new("downlink");
        let storage = storage_status(100.0, &[("DATA", 100.0)]);
        assert!(module.add_storage_unit_to_downlink(&storage));
        assert!(!module.add_storage_unit_to_downlink(&storage));
        assert!(!module.add_storage_unit_to_downlink(&storage.clone()));
    }

    /// Upstream's downlink with 4 attempts, receiver 2 and full-packet
    /// gating over a CNR of 20, run to 1 s over `storages`; returns the last
    /// downlink and node records.
    fn run_over_storages(
        module: &mut DownlinkHandling,
        storages: &mut [&mut dyn Module],
    ) -> (DownlinkHandlingMsg, DataNodeUsageMsg) {
        module.set_bit_rate_request(1.0e5).unwrap();
        module.set_packet_size_bits(256.0).unwrap();
        module.set_max_retransmissions(4).unwrap();
        module.set_receiver_antenna(ReceiverSelection::Antenna2);
        module.set_require_full_packet(true);
        let link = link_payload(0.0, 20.0, AntennaState::Tx, AntennaState::Rx, 1.0e6);
        link.connect_to(&mut module.model.link_budget_in_msg);
        module.init();
        for storage in storages.iter_mut() {
            storage.init();
        }
        for tick in TaskClock::new(SECOND_NANOS).run_until(SECOND_NANOS) {
            module.update(&context(tick));
            for storage in storages.iter_mut() {
                storage.update(&context(tick));
            }
        }
        (
            module.model.downlink_out_msg.read(),
            module.node_data_out_msg.read(),
        )
    }

    /// Upstream `test_downlink_selects_largest_partition_across_storage_messages`.
    #[test]
    fn downlink_selects_largest_partition_across_storage_messages() {
        let mut module = DownlinkHandling::new("downlink");
        let a = storage_status(900.0, &[("A_BIG", 400.0), ("A_SMALL", 100.0)]);
        let b = storage_status(2000.0, &[("B_TOP", 1200.0), ("B_OTHER", 800.0)]);
        assert!(module.add_storage_unit_to_downlink(&a));
        assert!(module.add_storage_unit_to_downlink(&b));
        let (last, _) = run_over_storages(&mut module, &mut []);
        assert_approx(last.available_data_bits, 1200.0, 0.0, 1e-12);
        assert_eq!(last.data_name, "B_TOP");
    }

    /// Upstream `test_downlink_ambiguous_multi_storage_name_blocks_removal`.
    #[test]
    fn downlink_ambiguous_multi_storage_name_blocks_removal() {
        let mut module = DownlinkHandling::new("downlink");
        let a = storage_status(800.0, &[("STORED DATA", 400.0)]);
        let b = storage_status(3000.0, &[("STORED DATA", 1200.0)]);
        assert!(module.add_storage_unit_to_downlink(&a));
        assert!(module.add_storage_unit_to_downlink(&b));
        let (last, node) = run_over_storages(&mut module, &mut []);
        assert!(last.link_active);
        assert_eq!(last.data_name, "STORED DATA");
        assert_approx(last.available_data_bits, 1200.0, 0.0, 1e-12);
        assert_approx(last.storage_removal_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.delivered_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.dropped_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(node.baud_rate_bps, 0.0, 0.0, 1e-12);
    }

    /// Upstream
    /// `test_downlink_multi_storage_units_block_actual_removal_even_with_unique_names`.
    #[test]
    fn downlink_multi_storage_units_block_actual_removal_even_with_unique_names() {
        let mut module = DownlinkHandling::new("downlink");
        let mut storage_a = PartitionedStorageUnit::new("storageA");
        storage_a.storage_capacity_bits = 1_000_000;
        storage_a.add_partition("A_ONLY");
        storage_a.set_data_buffer(&["A_ONLY"], &[400]);
        module
            .node_data_out_msg
            .connect_to(storage_a.add_data_node_to_model());
        let mut storage_b = PartitionedStorageUnit::new("storageB");
        storage_b.storage_capacity_bits = 1_000_000;
        storage_b.add_partition("B_TOP");
        storage_b.set_data_buffer(&["B_TOP"], &[1200]);
        module
            .node_data_out_msg
            .connect_to(storage_b.add_data_node_to_model());
        assert!(module.add_storage_unit_to_downlink(&storage_a.storage_unit_data_out_msg));
        assert!(module.add_storage_unit_to_downlink(&storage_b.storage_unit_data_out_msg));

        let (last, node) = run_over_storages(&mut module, &mut [&mut storage_a, &mut storage_b]);
        let (a, b) = (
            storage_a.storage_unit_data_out_msg.read(),
            storage_b.storage_unit_data_out_msg.read(),
        );
        assert!(last.link_active);
        assert_eq!(last.data_name, "B_TOP");
        assert_approx(last.available_data_bits, 1200.0, 0.0, 1e-12);
        assert_approx(last.storage_removal_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.delivered_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.dropped_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(node.baud_rate_bps, 0.0, 0.0, 1e-12);
        assert_eq!(node.data_name, "");
        assert_eq!(a.stored_data_name, ["A_ONLY"]);
        assert_eq!(b.stored_data_name, ["B_TOP"]);
        assert_approx(a.stored_data_bits[0], 400.0, 0.0, 1e-12);
        assert_approx(b.stored_data_bits[0], 1200.0, 0.0, 1e-12);
    }

    /// Upstream `test_downlink_missing_partition_name_blocks_removal`.
    #[test]
    fn downlink_missing_partition_name_blocks_removal() {
        let mut module = DownlinkHandling::new("downlink");
        let storage = storage_status(1200.0, &[("", 1200.0)]);
        assert!(module.add_storage_unit_to_downlink(&storage));
        let (last, node) = run_over_storages(&mut module, &mut []);
        assert!(last.link_active);
        assert_eq!(last.data_name, "");
        assert_approx(last.available_data_bits, 1200.0, 0.0, 1e-12);
        assert_approx(last.storage_removal_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.delivered_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_approx(last.dropped_data_rate_bps, 0.0, 0.0, 1e-12);
        assert_eq!(node.data_name, "");
        assert_approx(node.baud_rate_bps, 0.0, 0.0, 1e-12);
    }

    // ---------------------------------------------------------------------
    // test_antennaProducerScheduling.py
    // ---------------------------------------------------------------------

    /// Upstream `TASK_PERIOD` \[s\].
    const TASK_PERIOD_NANOS: u64 = SECOND_NANOS;
    const FREQUENCY_HZ: f64 = 20.0e9;
    const BANDWIDTH_HZ: f64 = 8.0e6;
    const INITIAL_ALTITUDE_M: f64 = 1624.0;
    const ALTITUDE_CHANGE_M: f64 = 2000.0;
    const LINK_DISTANCE_M: f64 = 2000.0e3;
    const INITIAL_DATA_BITS: i64 = 10_000_000;

    /// Upstream `configure_antenna`.
    fn scheduled_antenna(name: &str, state: AntennaState) -> SimpleAntenna {
        let mut antenna = SimpleAntenna::new(SimpleAntennaConfig {
            name: name.to_string(),
            frequency_hz: FREQUENCY_HZ,
            bandwidth_hz: BANDWIDTH_HZ,
            directivity_db: 30.0,
            hpbw_ratio: 1.0,
            p_tx_w: 5.0,
            p_rx_w: 5.0,
            equivalent_noise_temperature_k: 120.0,
            radiation_efficiency: 0.65,
            ambient_temperature_k: Some(150.0),
            r_ab_b_m: Vector3::zeros(),
            sigma_ab: Vector3::zeros(),
        });
        antenna.set_antenna_state(state);
        antenna
    }

    /// Per-tick records of upstream's recorders, with the tick times.
    #[derive(Default)]
    struct ChainLogs {
        times: Vec<u64>,
        link: Vec<LinkBudgetMsg>,
        antenna: Vec<AntennaLogMsg>,
        downlink: Vec<DownlinkHandlingMsg>,
        node: Vec<DataNodeUsageMsg>,
        storage_level: Vec<f64>,
        l_atm: Vec<f64>,
        l_point: Vec<f64>,
    }

    /// Upstream `ScheduledLink`: the producer chain and its recorders.
    struct ScheduledLink {
        ground_first: bool,
        spacecraft: Output<SpacecraftStateMsg>,
        ground: GroundLocation,
        transmitter: SimpleAntenna,
        receiver: SimpleAntenna,
        budget: LinkBudget,
        downlink: DownlinkHandling,
        storage: SimpleStorageUnit,
        normal: Vector3<f64>,
        ground_position: Vector3<f64>,
        spacecraft_position: Vector3<f64>,
        clock: TaskClock,
        logs: ChainLogs,
    }

    impl ScheduledLink {
        fn new(ground_first: bool) -> Self {
            let mut chain = Self {
                ground_first,
                spacecraft: Output::default(),
                ground: GroundLocation::new(GroundLocationConfig {
                    name: "ground".to_string(),
                    planet_radius_m: REQ_EARTH * 1.0e3,
                    ..GroundLocationConfig::default()
                }),
                transmitter: scheduled_antenna("space", AntennaState::Tx),
                receiver: scheduled_antenna("ground", AntennaState::Rx),
                budget: LinkBudget::new("linkBudget"),
                downlink: DownlinkHandling::new("downlink"),
                storage: SimpleStorageUnit::new("storage"),
                normal: Vector3::zeros(),
                ground_position: Vector3::zeros(),
                spacecraft_position: Vector3::zeros(),
                clock: TaskClock::new(TASK_PERIOD_NANOS),
                logs: ChainLogs::default(),
            };
            let index = chain.ground.add_spacecraft_to_model();
            chain
                .spacecraft
                .connect_to(&mut chain.ground.sc_state_in_msgs[index]);
            chain
                .spacecraft
                .connect_to(&mut chain.transmitter.sc_state_in_msg);
            chain
                .ground
                .current_ground_state_out_msg
                .connect_to(&mut chain.receiver.ground_state_in_msg);
            chain.budget.atmos_att = true;
            let (first, second) = if ground_first {
                (&chain.receiver, &chain.transmitter)
            } else {
                (&chain.transmitter, &chain.receiver)
            };
            first
                .antenna_out_msg
                .connect_to(&mut chain.budget.antenna_in_msg_1);
            second
                .antenna_out_msg
                .connect_to(&mut chain.budget.antenna_in_msg_2);
            chain.downlink.set_bit_rate_request(100000.0).unwrap();
            chain
                .budget
                .link_budget_out_msg
                .connect_to(&mut chain.downlink.model.link_budget_in_msg);
            chain.storage.storage_capacity_bits = 2 * INITIAL_DATA_BITS;
            chain.storage.set_data_buffer(INITIAL_DATA_BITS);
            chain
                .downlink
                .node_data_out_msg
                .connect_to(chain.storage.add_data_node_to_model());
            chain
                .downlink
                .add_storage_unit_to_downlink(&chain.storage.storage_unit_data_out_msg);
            chain.set_geometry(INITIAL_ALTITUDE_M);
            chain
        }

        /// Upstream `set_geometry`: stationary geometry with the spacecraft
        /// boresight on the station.
        fn set_geometry(&mut self, altitude: f64) {
            let (latitude, longitude, elevation) = (0.35f64, -0.6f64, 0.7f64);
            self.ground.config.location = SiteLocation::Geodetic(Lla {
                latitude_rad: latitude,
                longitude_rad: longitude,
                altitude_m: altitude,
            });
            self.normal = Vector3::new(
                latitude.cos() * longitude.cos(),
                latitude.cos() * longitude.sin(),
                latitude.sin(),
            );
            self.ground_position = (self.ground.config.planet_radius_m + altitude) * self.normal;
            let east = Vector3::new(-longitude.sin(), longitude.cos(), 0.0);
            let ray = elevation.sin() * self.normal + elevation.cos() * east;
            self.spacecraft_position = self.ground_position + LINK_DISTANCE_M * ray;
            let boresight = -ray;
            let axis_y = boresight.cross(&self.normal).normalize();
            let axis_x = axis_y.cross(&boresight);
            let dcm_bn = nalgebra::Matrix3::from_rows(&[
                axis_x.transpose(),
                axis_y.transpose(),
                boresight.transpose(),
            ]);
            self.spacecraft.write(SpacecraftStateMsg {
                position_m: self.spacecraft_position,
                sigma_bn: dcm_to_mrp(dcm_bn),
                ..SpacecraftStateMsg::default()
            });
        }

        fn modules(&mut self) -> [&mut dyn Module; 6] {
            [
                &mut self.ground,
                &mut self.transmitter,
                &mut self.receiver,
                &mut self.budget,
                &mut self.downlink,
                &mut self.storage,
            ]
        }

        /// Upstream `InitializeSimulation`: the first one initializes, a
        /// later one resets the modules, the clock and the recorders.
        fn initialize(&mut self, first: bool) {
            for module in self.modules() {
                if first {
                    module.init();
                } else {
                    module.reset(&context(0));
                }
            }
            self.clock = TaskClock::new(TASK_PERIOD_NANOS);
            self.logs = ChainLogs::default();
        }

        /// Upstream `run_until`, with the task `delayed` (`"ground"` or
        /// `"antennas"`) disabled.
        fn run_until(&mut self, time_s: u64, delayed: Option<&str>) {
            for tick in self.clock.run_until(time_s * TASK_PERIOD_NANOS) {
                let context = context(tick);
                if delayed != Some("ground") {
                    self.ground.update(&context);
                }
                if delayed != Some("antennas") {
                    self.transmitter.update(&context);
                    self.receiver.update(&context);
                }
                self.budget.update(&context);
                self.downlink.update(&context);
                self.storage.update(&context);
                let logs = &mut self.logs;
                logs.times.push(tick);
                logs.link.push(self.budget.link_budget_out_msg.read());
                logs.antenna.push(self.receiver.antenna_out_msg.read());
                logs.downlink
                    .push(self.downlink.model.downlink_out_msg.read());
                logs.node.push(self.downlink.node_data_out_msg.read());
                logs.storage_level.push(
                    self.storage
                        .storage_unit_data_out_msg
                        .read()
                        .storage_level_bits,
                );
                logs.l_atm.push(self.budget.l_atm_db());
                logs.l_point.push(self.budget.l_point_db());
            }
        }

        /// Upstream `assert_reset_output`.
        fn assert_reset_output(&self) {
            let output = self.budget.link_budget_out_msg.read();
            assert_eq!(
                (output.cnr_1, output.cnr_2, output.bandwidth_hz),
                (0.0, 0.0, 0.0)
            );
            assert_eq!(self.budget.l_atm_db(), 0.0);
        }

        /// Upstream `assert_ready`: the scheduled results from
        /// `first_sample` on against an explicitly primed reference link;
        /// returns the reference attenuation.
        fn assert_ready(&self, first_sample: usize) -> f64 {
            let mut ground_payload = self.receiver.antenna_out_msg.read();
            ground_payload.r_an_n_m = self.ground_position;
            ground_payload.r_ap_n_m = self.ground_position;
            ground_payload.n_hat_lp_n = self.normal;
            let mut space_payload = self.transmitter.antenna_out_msg.read();
            space_payload.r_an_n_m = self.spacecraft_position;
            let (ground_msg, space_msg) = (Output::new(ground_payload), Output::new(space_payload));
            let mut reference = LinkBudget::new("reference");
            reference.atmos_att = true;
            space_msg.connect_to(&mut reference.antenna_in_msg_1);
            ground_msg.connect_to(&mut reference.antenna_in_msg_2);
            reference.init();
            reference.update(&context(0));
            let expected_cnr = reference.cnr_2();
            let expected_loss = reference.l_atm_db();
            assert!(expected_cnr > 0.0 && expected_loss > 0.0);
            assert!(self.budget.atmos_att);

            let logs = &self.logs;
            let ready = first_sample..logs.times.len();
            for i in 0..logs.times.len() {
                let (cnr, unused) = if self.ground_first {
                    (logs.link[i].cnr_1, logs.link[i].cnr_2)
                } else {
                    (logs.link[i].cnr_2, logs.link[i].cnr_1)
                };
                assert_eq!(unused, 0.0);
                if !ready.contains(&i) {
                    continue;
                }
                assert!(
                    (cnr - expected_cnr).abs() <= 1.0e-10 * expected_cnr,
                    "{cnr} vs {expected_cnr}"
                );
                assert!((logs.l_atm[i] - expected_loss).abs() <= 1.0e-12);
                assert!(logs.l_point[i].abs() <= 1.0e-12);
                assert!((logs.antenna[i].n_hat_lp_n - self.normal).amax() <= 1.0e-14);
                assert!((logs.antenna[i].r_ap_n_m - self.ground_position).amax() <= 1.0e-8);
                assert_eq!(logs.link[i].bandwidth_hz, BANDWIDTH_HZ);
                assert!(logs.downlink[i].link_active);
                assert_eq!(
                    logs.downlink[i].receiver_index,
                    if self.ground_first { 1 } else { 2 }
                );
                assert!((logs.downlink[i].cnr - expected_cnr).abs() <= 1.0e-10 * expected_cnr);
            }
            let mut previous_nanos = 0;
            for i in 0..logs.times.len() {
                assert_approx(
                    logs.node[i].baud_rate_bps,
                    -logs.downlink[i].storage_removal_rate_bps,
                    1e-7,
                    0.0,
                );
                let elapsed_s = (logs.times[i] - previous_nanos) as f64 * 1.0e-9;
                assert_approx(logs.downlink[i].time_step_s, elapsed_s, 1e-7, 0.0);
                previous_nanos = logs.times[i];
            }
            // At time zero the step is zero; later ready ticks deliver data.
            let flow_start = first_sample.max(1);
            for i in flow_start..logs.times.len() {
                assert!(logs.downlink[i].delivered_data_rate_bps > 0.0);
                assert!(logs.node[i].baud_rate_bps < 0.0);
            }
            let levels = &logs.storage_level;
            assert!(levels.last().unwrap() < &levels[0]);
            let removed: f64 = logs
                .downlink
                .iter()
                .map(|sample| sample.storage_removal_rate_bps * sample.time_step_s)
                .sum();
            assert_approx(
                logs.downlink.last().unwrap().cumulative_removed_bits,
                removed,
                1e-6,
                1e-12,
            );
            // Storage rounds each integrated update to whole bits.
            assert_approx(
                levels[0] - levels.last().unwrap(),
                removed,
                0.0,
                logs.times.len() as f64,
            );
            expected_loss
        }
    }

    /// Upstream `test_scheduled_antenna_initialization`: initialization,
    /// waiting for delayed producers, downstream service, and a reset that
    /// rebuilds the atmosphere at a changed altitude.
    #[test]
    fn scheduled_antenna_initialization() {
        for ground_first in [false, true] {
            for delayed in [None, Some("antennas"), Some("ground")] {
                let mut chain = ScheduledLink::new(ground_first);
                chain.initialize(true);
                // Unwritten: the stand-in for upstream's `isWritten`.
                assert_eq!(chain.budget.antenna_in_msg_1.read().frequency_hz, 0.0);
                assert_eq!(chain.budget.antenna_in_msg_2.read().frequency_hz, 0.0);
                chain.assert_reset_output();
                chain.run_until(1, delayed);
                if delayed.is_some() {
                    let logs = &chain.logs;
                    for i in 0..logs.times.len() {
                        assert_eq!(logs.link[i].cnr_1, 0.0);
                        assert_eq!(logs.link[i].cnr_2, 0.0);
                        assert_eq!(logs.link[i].bandwidth_hz, 0.0);
                        assert_eq!(logs.l_atm[i], 0.0);
                        assert!(!logs.downlink[i].link_active);
                        assert_eq!(logs.downlink[i].delivered_data_rate_bps, 0.0);
                        assert_eq!(logs.downlink[i].cumulative_removed_bits, 0.0);
                        assert_eq!(logs.node[i].baud_rate_bps, 0.0);
                        assert_eq!(logs.storage_level[i], INITIAL_DATA_BITS as f64);
                    }
                }
                chain.run_until(3, None);
                let expected_times: Vec<u64> = (0..4).map(|i| i * TASK_PERIOD_NANOS).collect();
                assert_eq!(chain.logs.times, expected_times);
                let first_loss = chain.assert_ready(if delayed.is_none() { 0 } else { 2 });

                let previous_position = chain.ground_position;
                chain.set_geometry(INITIAL_ALTITUDE_M + ALTITUDE_CHANGE_M);
                chain.initialize(false);
                chain.assert_reset_output();
                // The old antenna publication survives reset; the next
                // ordered tick replaces it.
                assert!(
                    (chain.receiver.antenna_out_msg.read().r_ap_n_m - previous_position).amax()
                        <= 1e-7 * previous_position.amax()
                );
                chain.run_until(2, None);
                assert_eq!(chain.logs.times, expected_times[..3]);
                assert!(chain.assert_ready(0) < first_loss);
            }
        }
    }
}
