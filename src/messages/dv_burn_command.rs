//! Delta-v burn command (upstream `DvBurnCmdMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Configuration of one delta-v burn: upstream `DvBurnCmdMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DvBurnCommandMsg {
    /// Upstream `dvInrtlCmd` [m/s]: commanded delta-v, inertial components.
    pub dv_inertial_cmd_mps: Vector3<f64>,
    /// Upstream `dvRotVecUnit` [-]: unit axis the burn direction rotates
    /// about.
    pub dv_rot_vec_unit: Vector3<f64>,
    /// Upstream `dvRotVecMag` [rad/s]: rotation rate about
    /// `dv_rot_vec_unit`.
    pub dv_rot_vec_mag_radps: f64,
    /// Upstream `burnStartTime` \[ns\]: commanded burn start time.
    pub burn_start_time_nanos: u64,
}

impl_telemetry!(DvBurnCommandMsg {
    dv_inertial_cmd_mps,
    dv_rot_vec_unit,
    dv_rot_vec_mag_radps,
    burn_start_time_nanos,
});
