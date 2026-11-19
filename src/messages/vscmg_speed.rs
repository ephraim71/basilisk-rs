//! VSCMG wheel and gimbal states (upstream `VSCMGSpeedMsgPayload`).

use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;
use super::flatten::impl_telemetry;

/// Wheel speed, gimbal angle and gimbal rate of every VSCMG, in cluster
/// order: upstream `VSCMGSpeedMsg`, published by the VSCMG state effector
/// for the whole cluster (as [`super::RwSpeedMsg`] is for the wheels).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VscmgSpeedMsg {
    /// Upstream `wheelSpeeds` \[rad/s\]: wheel speed relative to the gimbal.
    #[serde(with = "super::big_array")]
    pub wheel_speeds_radps: [f64; MAX_EFF_COUNT],
    /// Upstream `gimbalAngles` \[rad\]: gimbal angle.
    #[serde(with = "super::big_array")]
    pub gimbal_angles_rad: [f64; MAX_EFF_COUNT],
    /// Upstream `gimbalRates` \[rad/s\]: gimbal rate.
    #[serde(with = "super::big_array")]
    pub gimbal_rates_radps: [f64; MAX_EFF_COUNT],
}

impl Default for VscmgSpeedMsg {
    fn default() -> Self {
        Self {
            wheel_speeds_radps: [0.0; MAX_EFF_COUNT],
            gimbal_angles_rad: [0.0; MAX_EFF_COUNT],
            gimbal_rates_radps: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl_telemetry!(VscmgSpeedMsg {
    wheel_speeds_radps,
    gimbal_angles_rad,
    gimbal_rates_radps,
});
