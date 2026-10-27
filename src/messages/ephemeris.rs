//! Celestial-body ephemeris state (upstream `EphemerisMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Translational and rotational state of an orbital body: upstream
/// `EphemerisMsg`.
///
/// Here `B` is the body-fixed frame of the celestial body, not the spacecraft.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EphemerisMsg {
    /// Upstream `r_BdyZero_N` \[m\]: position of the body, inertial components.
    pub position_bdy_zero_n_m: Vector3<f64>,
    /// Upstream `v_BdyZero_N` [m/s]: velocity of the body, inertial components.
    pub velocity_bdy_zero_n_mps: Vector3<f64>,
    /// Upstream `sigma_BN` [-]: MRP attitude of the body-fixed frame relative
    /// to inertial.
    pub sigma_bn: Vector3<f64>,
    /// Upstream `omega_BN_B` [rad/s]: angular velocity of the body relative to
    /// inertial, body-fixed components.
    pub omega_bn_b_radps: Vector3<f64>,
    /// Upstream `timeTag` \[s\]: vehicle time tag of the state.
    pub time_tag_s: f64,
}

impl_telemetry!(EphemerisMsg {
    position_bdy_zero_n_m,
    velocity_bdy_zero_n_mps,
    sigma_bn,
    omega_bn_b_radps,
    time_tag_s,
});
