//! Device on/off status (upstream `DeviceStatusMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// On/off state of a device: upstream `enum deviceState`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum DeviceState {
    /// Upstream `Off = 0`, the zeroed default.
    #[default]
    Off = 0,
    /// Upstream `On = 1`.
    On = 1,
}

impl DeviceState {
    /// Whether the device is on.
    pub fn is_on(self) -> bool {
        self == Self::On
    }
}

/// Status of a device such as an instrument controller: upstream
/// `DeviceStatusMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DeviceStatusMsg {
    /// Upstream `deviceStatus`: whether the device is on.
    pub device_status: DeviceState,
}

impl_telemetry!(enum DeviceState as u8);
impl_telemetry!(DeviceStatusMsg { device_status });
