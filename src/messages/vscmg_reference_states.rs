//! Desired VSCMG wheel accelerations and gimbal rates (upstream
//! `VSCMGRefStatesMsgPayload`).

use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;
use super::flatten::impl_telemetry;

/// Steering-law output for every VSCMG, in cluster order: upstream
/// `VSCMGRefStatesMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VscmgReferenceStatesMsg {
    /// Upstream `wheelAccels` \[rad/s^2\]: desired wheel accelerations.
    #[serde(with = "super::big_array")]
    pub wheel_accels_radps2: [f64; MAX_EFF_COUNT],
    /// Upstream `gimbalRates` \[rad/s\]: desired gimbal rates.
    #[serde(with = "super::big_array")]
    pub gimbal_rates_radps: [f64; MAX_EFF_COUNT],
}

impl Default for VscmgReferenceStatesMsg {
    fn default() -> Self {
        Self {
            wheel_accels_radps2: [0.0; MAX_EFF_COUNT],
            gimbal_rates_radps: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl_telemetry!(VscmgReferenceStatesMsg {
    wheel_accels_radps2,
    gimbal_rates_radps,
});
