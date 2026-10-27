//! Buffer of accelerometer/gyro packets (upstream `AccDataMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{AccelerometerPacket, MAX_ACC_BUF_PKT};

/// The accelerometer/gyro buffer read in one frame: upstream `AccDataMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AccelerometerDataMsg {
    /// Upstream `accPkts` [-]: buffered packets, in no particular order
    /// (consumers sort by `meas_time_nanos`); unused slots are zero.
    #[serde(with = "super::big_array")]
    pub accel_packets: [AccelerometerPacket; MAX_ACC_BUF_PKT],
}

impl Default for AccelerometerDataMsg {
    fn default() -> Self {
        Self {
            accel_packets: [AccelerometerPacket::default(); MAX_ACC_BUF_PKT],
        }
    }
}

impl_telemetry!(AccelerometerDataMsg { accel_packets });
