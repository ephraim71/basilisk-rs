//! One VSCMG's FSW configuration (upstream `VSCMGConfigElementMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Configuration of a single variable-speed control moment gyroscope (a
/// reaction wheel on a gimbal): upstream `VSCMGConfigElementMsg`, also the
/// element type of [`super::VscmgArrayConfigMsg`].
///
/// The gimbal frame `G` has the spin axis `ĝ_s`, the transverse axis `ĝ_t`
/// and the gimbal axis `ĝ_g`; the first two rotate about `ĝ_g` with the
/// gimbal angle, so they are given at zero gimbal angle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VscmgConfigElementMsg {
    /// Upstream `gsHat0_B` [-]: spin axis at zero gimbal angle, body
    /// components.
    pub gs_hat0_b: Vector3<f64>,
    /// Upstream `gtHat0_B` [-]: transverse axis at zero gimbal angle, body
    /// components.
    pub gt_hat0_b: Vector3<f64>,
    /// Upstream `ggHat_B` [-]: gimbal axis, body components.
    pub gg_hat_b: Vector3<f64>,
    /// Upstream `Js` \[kg m^2\]: gimbal-plus-wheel inertia about `ĝ_s`.
    pub js_kg_m2: f64,
    /// Upstream `Jt` \[kg m^2\]: gimbal-plus-wheel inertia about `ĝ_t`.
    pub jt_kg_m2: f64,
    /// Upstream `Jg` \[kg m^2\]: gimbal-plus-wheel inertia about `ĝ_g`.
    pub jg_kg_m2: f64,
    /// Upstream `Iws` \[kg m^2\]: wheel inertia about its spin axis.
    pub iws_kg_m2: f64,
    /// Upstream `Omega0` \[rad/s\]: initial wheel speed.
    pub wheel_speed0_radps: f64,
    /// Upstream `gamma0` \[rad\]: initial gimbal angle.
    pub gimbal_angle0_rad: f64,
    /// Upstream `gammaDot0` \[rad/s\]: initial gimbal rate.
    pub gimbal_rate0_radps: f64,
}

impl_telemetry!(VscmgConfigElementMsg {
    gs_hat0_b,
    gt_hat0_b,
    gg_hat_b,
    js_kg_m2,
    jt_kg_m2,
    jg_kg_m2,
    iws_kg_m2,
    wheel_speed0_radps,
    gimbal_angle0_rad,
    gimbal_rate0_radps,
});
