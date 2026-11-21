//! Downlink throughput and reliability (upstream
//! `DownlinkHandlingMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// What a downlink removes from onboard storage: upstream
/// `DownlinkHandling::RemovalPolicy`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u32)]
pub enum RemovalPolicy {
    /// Upstream `REMOVE_ATTEMPTED = 0`, the default: remove delivered and
    /// retry-limited (dropped) data.
    #[default]
    RemoveAttempted = 0,
    /// Upstream `REMOVE_DELIVERED_ONLY = 1`: remove only delivered data, and
    /// keep dropped data onboard.
    RemoveDeliveredOnly = 1,
}

/// Link quality, error rates and data rates of a downlink: upstream
/// `DownlinkHandlingMsg`.
///
/// Upstream's `char` name buffers are `String`s here.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DownlinkHandlingMsg {
    /// Upstream `linkActive` (`uint32_t`, 1 / 0): the link-quality inputs
    /// are valid for the downlink calculation.
    pub link_active: bool,
    /// Upstream `receiverIndex` \[-\]: receiving antenna of the link budget
    /// (1 or 2), or 0 for none.
    pub receiver_index: u32,
    /// Upstream `maxRetransmissions` \[-\]: most transmission attempts per
    /// packet in the ARQ model.
    pub max_retransmissions: u64,
    /// Upstream `removalPolicy` (`uint32_t`): storage-removal policy.
    pub removal_policy: RemovalPolicy,
    /// Upstream `transmitterAntennaName`: name of the transmitting antenna.
    pub transmitter_antenna_name: String,
    /// Upstream `receiverAntennaName`: name of the receiving antenna.
    pub receiver_antenna_name: String,
    /// Upstream `dataName`: storage partition being downlinked.
    pub data_name: String,
    /// Upstream `timeStep` \[s\]: integration step of this update.
    pub time_step_s: f64,
    /// Upstream `bandwidth` \[Hz\]: shared link bandwidth.
    pub bandwidth_hz: f64,
    /// Upstream `bitRateRequest` \[bit/s\]: requested channel bit rate.
    pub bit_rate_request_bps: f64,
    /// Upstream `packetSizeBits` \[bit\]: packet size of the BER-to-PER
    /// conversion.
    pub packet_size_bits: f64,
    /// Upstream `cnr` \[-\]: linear carrier-to-noise ratio of the selected
    /// receiver.
    pub cnr: f64,
    /// Upstream `cnr_dB` \[dB\]: the same, in decibels.
    pub cnr_db: f64,
    /// Upstream `cNo_dBHz` \[dBHz\]: carrier-to-noise-density ratio.
    pub c_n0_dbhz: f64,
    /// Upstream `ebN0_dB` \[dB\]: energy-per-bit to noise-density ratio.
    pub eb_n0_db: f64,
    /// Upstream `ber` \[-\]: bit error rate.
    pub ber: f64,
    /// Upstream `per` \[-\]: packet error rate.
    pub per: f64,
    /// Upstream `packetSuccessProb` \[-\]: probability a packet is delivered
    /// within the attempt limit.
    pub packet_success_prob: f64,
    /// Upstream `packetDropProb` \[-\]: probability a packet is dropped at
    /// the attempt limit.
    pub packet_drop_prob: f64,
    /// Upstream `expectedAttemptsPerPacket` \[-\]: expected attempts to
    /// complete one source packet.
    pub expected_attempts_per_packet: f64,
    /// Upstream `attemptedDataRate` \[bit/s\]: channel rate including
    /// retransmissions.
    pub attempted_data_rate_bps: f64,
    /// Upstream `storageRemovalRate` \[bit/s\]: rate data leaves storage
    /// under the removal policy.
    pub storage_removal_rate_bps: f64,
    /// Upstream `deliveredDataRate` \[bit/s\]: rate data reaches the
    /// receiver.
    pub delivered_data_rate_bps: f64,
    /// Upstream `droppedDataRate` \[bit/s\]: rate data is dropped at the
    /// attempt limit.
    pub dropped_data_rate_bps: f64,
    /// Upstream `availableDataBits` \[bit\]: data in the selected partition
    /// at the start of the step.
    pub available_data_bits: f64,
    /// Upstream `estimatedRemainingDataBits` \[bit\]: data estimated to be
    /// left in the partition after the step.
    pub estimated_remaining_data_bits: f64,
    /// Upstream `cumulativeAttemptedBits` \[bit\]: bits attempted so far,
    /// retransmissions included.
    pub cumulative_attempted_bits: f64,
    /// Upstream `cumulativeRemovedBits` \[bit\]: bits removed from storage so
    /// far.
    pub cumulative_removed_bits: f64,
    /// Upstream `cumulativeDeliveredBits` \[bit\]: bits delivered so far.
    pub cumulative_delivered_bits: f64,
    /// Upstream `cumulativeDroppedBits` \[bit\]: bits dropped so far.
    pub cumulative_dropped_bits: f64,
}

impl_telemetry!(enum RemovalPolicy as u32);
impl_telemetry!(DownlinkHandlingMsg {
    link_active,
    receiver_index,
    max_retransmissions,
    removal_policy,
    time_step_s,
    bandwidth_hz,
    bit_rate_request_bps,
    packet_size_bits,
    cnr,
    cnr_db,
    c_n0_dbhz,
    eb_n0_db,
    ber,
    per,
    packet_success_prob,
    packet_drop_prob,
    expected_attempts_per_packet,
    attempted_data_rate_bps,
    storage_removal_rate_bps,
    delivered_data_rate_bps,
    dropped_data_rate_bps,
    available_data_bits,
    estimated_remaining_data_bits,
    cumulative_attempted_bits,
    cumulative_removed_bits,
    cumulative_delivered_bits,
    cumulative_dropped_bits,
});
