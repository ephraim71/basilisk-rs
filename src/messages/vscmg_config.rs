//! One VSCMG's simulated state and motor settings (upstream
//! `VSCMGConfigMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// How a VSCMG's wheel couples to the hub: upstream `enum VSCMGModels`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum VscmgModel {
    /// Upstream `vscmgBalancedWheels = 0`: a balanced wheel on a balanced
    /// gimbal, with coincident centers of mass.
    #[default]
    BalancedWheels = 0,
    /// Upstream `vscmgJitterSimple = 1`: the balanced model plus the static
    /// and dynamic wheel imbalance as an external force and torque.
    JitterSimple = 1,
    /// Upstream `vscmgJitterFullyCoupled = 2`: wheel and gimbal offsets and
    /// imbalance coupled into the hub equations of motion.
    JitterFullyCoupled = 2,
}

/// The state and motor settings of one VSCMG as the dynamics publish them:
/// upstream `VSCMGConfigMsg`, one message per VSCMG (`vscmgOutMsgs`).
///
/// Upstream's payload is also the device configuration and carries the
/// effector's intermediate terms; this holds the fields its
/// `WriteOutputMessages` fills, the rest of the payload staying zero there.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VscmgConfigMsg {
    /// Upstream `VSCMGModel`.
    pub model: VscmgModel,
    /// Upstream `u_s_current` \[N m\]: applied wheel motor torque.
    pub u_s_current_nm: f64,
    /// Upstream `u_s_max` \[N m\]: wheel torque limit (negative: none).
    pub u_s_max_nm: f64,
    /// Upstream `u_s_min` \[N m\]: smallest wheel torque applied.
    pub u_s_min_nm: f64,
    /// Upstream `u_s_f` \[N m\]: wheel Coulomb friction torque.
    pub u_s_f_nm: f64,
    /// Upstream `u_g_current` \[N m\]: applied gimbal motor torque.
    pub u_g_current_nm: f64,
    /// Upstream `u_g_max` \[N m\]: gimbal torque limit (negative: none).
    pub u_g_max_nm: f64,
    /// Upstream `u_g_min` \[N m\]: smallest gimbal torque applied.
    pub u_g_min_nm: f64,
    /// Upstream `u_g_f` \[N m\]: gimbal Coulomb friction torque.
    pub u_g_f_nm: f64,
    /// Upstream `theta` \[rad\]: wheel angle (jitter models only, else zero).
    pub theta_rad: f64,
    /// Upstream `Omega` \[rad/s\]: wheel speed relative to the gimbal.
    pub omega_radps: f64,
    /// Upstream `Omega_max` \[rad/s\]: wheel speed limit (negative: none).
    pub omega_max_radps: f64,
    /// Upstream `gamma` \[rad\]: gimbal angle.
    pub gamma_rad: f64,
    /// Upstream `gammaDot` \[rad/s\]: gimbal rate.
    pub gamma_dot_radps: f64,
    /// Upstream `gammaDot_max` \[rad/s\]: gimbal rate scale of the linear
    /// friction band.
    pub gamma_dot_max_radps: f64,
    /// Upstream `IW1` \[kg m^2\]: wheel spin-axis inertia.
    pub iw1_kg_m2: f64,
    /// Upstream `U_s` \[kg m\]: static wheel imbalance.
    pub static_imbalance_kg_m: f64,
    /// Upstream `U_d` \[kg m^2\]: dynamic wheel imbalance.
    pub dynamic_imbalance_kg_m2: f64,
}

impl_telemetry!(enum VscmgModel as u8);

impl_telemetry!(VscmgConfigMsg {
    model,
    u_s_current_nm,
    u_s_max_nm,
    u_s_min_nm,
    u_s_f_nm,
    u_g_current_nm,
    u_g_max_nm,
    u_g_min_nm,
    u_g_f_nm,
    theta_rad,
    omega_radps,
    omega_max_radps,
    gamma_rad,
    gamma_dot_radps,
    gamma_dot_max_radps,
    iw1_kg_m2,
    static_imbalance_kg_m,
    dynamic_imbalance_kg_m2,
});
