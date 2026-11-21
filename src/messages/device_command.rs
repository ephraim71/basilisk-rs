//! Device command (upstream `DeviceCmdMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Command changing the state of an instrument: upstream `DeviceCmdMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DeviceCommandMsg {
    /// Upstream `deviceCmd` [-]: 0 commands the device off; values above 0
    /// command other, device-defined states (the instrument controllers send
    /// 1 to take an image).
    pub device_cmd: u64,
}

impl_telemetry!(DeviceCommandMsg { device_cmd });
