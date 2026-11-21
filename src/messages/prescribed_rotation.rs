//! Prescribed rotational state of an effector (upstream
//! `PrescribedRotationMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Rotational state of a prescribed-motion body frame `P` relative to its
/// mount frame `M`: upstream `PrescribedRotationMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PrescribedRotationMsg {
    /// Upstream `omega_PM_P` [rad/s]: angular velocity of `P` relative to `M`,
    /// `P` components.
    pub omega_pm_p_radps: Vector3<f64>,
    /// Upstream `omegaPrime_PM_P` [rad/s^2]: `P`-frame time derivative of
    /// `omega_pm_p_radps`.
    pub omega_prime_pm_p_radps2: Vector3<f64>,
    /// Upstream `sigma_PM` [-]: MRP attitude of `P` relative to `M`.
    pub sigma_pm: Vector3<f64>,
}

impl_telemetry!(PrescribedRotationMsg {
    omega_pm_p_radps,
    omega_prime_pm_p_radps2,
    sigma_pm,
});
