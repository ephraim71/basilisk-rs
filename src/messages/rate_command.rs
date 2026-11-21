//! Body-rate steering command (upstream `RateCmdMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Desired body rate from a steering law: upstream `RateCmdMsg`.
///
/// `B*` ("Bast") is the desired body frame the steering law asks for.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RateCommandMsg {
    /// Upstream `omega_BastR_B` [rad/s]: desired body rate relative to the
    /// reference frame `R`, body components.
    pub omega_bast_r_b_radps: Vector3<f64>,
    /// Upstream `omegap_BastR_B` [rad/s^2]: body-frame time derivative of
    /// `omega_bast_r_b_radps`.
    pub omega_prime_bast_r_b_radps2: Vector3<f64>,
}

impl_telemetry!(RateCommandMsg {
    omega_bast_r_b_radps,
    omega_prime_bast_r_b_radps2,
});
