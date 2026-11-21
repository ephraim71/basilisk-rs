//! Translational navigation estimate (upstream `NavTransMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Spacecraft translational navigation solution: upstream `NavTransMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NavigationTranslationMsg {
    /// Upstream `timeTag` \[s\]: vehicle time tag of the measurements.
    pub time_tag_s: f64,
    /// Upstream `r_BN_N` \[m\]: inertial position of the spacecraft, inertial
    /// components.
    pub position_bn_n_m: Vector3<f64>,
    /// Upstream `v_BN_N` [m/s]: inertial velocity of the spacecraft, inertial
    /// components.
    pub velocity_bn_n_mps: Vector3<f64>,
    /// Upstream `vehAccumDV` [m/s]: total accumulated delta-v.
    ///
    /// Body-frame components: the upstream payload does not name the frame,
    /// but every producer writes a body-frame sum (`dvAccumulation` copies
    /// `vehAccumDV_B`, `simpleNav` copies `TotalAccumDVBdy`).
    pub accumulated_dv_b_mps: Vector3<f64>,
}

impl_telemetry!(NavigationTranslationMsg {
    time_tag_s,
    position_bn_n_m,
    velocity_bn_n_mps,
    accumulated_dv_b_mps,
});
