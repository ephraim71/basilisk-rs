//! Inertial location of a ground location (upstream `GroundStateMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Inertial location of a ground location `L` at the current time, with `P`
/// the planet centre: upstream `GroundStateMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GroundStateMsg {
    /// Upstream `r_LN_N` \[m\]: location relative to the inertial origin,
    /// inertial components.
    pub position_ln_n_m: Vector3<f64>,
    /// Upstream `r_LP_N` \[m\]: location relative to the planet centre,
    /// inertial components.
    pub position_lp_n_m: Vector3<f64>,
    /// Upstream `rHat_LP_N` [-]: surface normal at the location, inertial
    /// components.
    pub r_hat_lp_n: Vector3<f64>,
}

impl_telemetry!(GroundStateMsg {
    position_ln_n_m,
    position_lp_n_m,
    r_hat_lp_n,
});
