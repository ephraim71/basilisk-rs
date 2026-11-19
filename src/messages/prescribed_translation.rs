//! Prescribed translational state of an effector (upstream
//! `PrescribedTranslationMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Translational state of a prescribed-motion body frame `P` relative to its
/// mount frame `M`: upstream `PrescribedTranslationMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PrescribedTranslationMsg {
    /// Upstream `r_PM_M` \[m\]: position of the `P` frame origin relative to
    /// the `M` frame origin, `M` components.
    pub r_pm_m_m: Vector3<f64>,
    /// Upstream `rPrime_PM_M` \[m/s\]: `B`/`M`-frame time derivative of
    /// `r_pm_m_m`.
    pub r_prime_pm_m_mps: Vector3<f64>,
    /// Upstream `rPrimePrime_PM_M` \[m/s^2\]: `B`/`M`-frame time derivative
    /// of `r_prime_pm_m_mps`.
    pub r_prime_prime_pm_m_mps2: Vector3<f64>,
}

impl_telemetry!(PrescribedTranslationMsg {
    r_pm_m_m,
    r_prime_pm_m_mps,
    r_prime_prime_pm_m_mps2,
});
