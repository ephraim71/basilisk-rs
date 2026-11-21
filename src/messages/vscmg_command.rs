//! Motor torque command of one VSCMG (upstream `VSCMGCmdMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Wheel and gimbal motor torques commanded to a single VSCMG: upstream
/// `VSCMGCmdMsg`, the per-device command the VSCMG state effector applies.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VscmgCommandMsg {
    /// Upstream `u_s_cmd` \[N m\]: wheel motor torque.
    pub u_s_cmd_nm: f64,
    /// Upstream `u_g_cmd` \[N m\]: gimbal motor torque.
    pub u_g_cmd_nm: f64,
}

impl_telemetry!(VscmgCommandMsg {
    u_s_cmd_nm,
    u_g_cmd_nm
});
