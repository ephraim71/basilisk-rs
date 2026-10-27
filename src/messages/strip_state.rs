//! Current target on an imaging strip (upstream `StripStateMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Inertial state of the current target `L` on the central line of a strip,
/// with `P` the planet centre: upstream `StripStateMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StripStateMsg {
    /// Upstream `r_LN_N` \[m\]: target position relative to the inertial origin,
    /// inertial components.
    pub position_ln_n_m: Vector3<f64>,
    /// Upstream `r_LP_N` \[m\]: target position relative to the planet centre,
    /// inertial components.
    pub position_lp_n_m: Vector3<f64>,
    /// Upstream `v_LN_N` [m/s]: target velocity relative to the inertial
    /// origin, inertial components.
    pub velocity_ln_n_mps: Vector3<f64>,
    /// Upstream `v_LP_N` [m/s]: target velocity relative to the planet centre,
    /// inertial components.
    pub velocity_lp_n_mps: Vector3<f64>,
}

impl_telemetry!(StripStateMsg {
    position_ln_n_m,
    position_lp_n_m,
    velocity_ln_n_mps,
    velocity_lp_n_mps,
});
