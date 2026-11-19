//! Displacement of a body along one axis (upstream
//! `LinearTranslationRigidBodyMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Scalar displacement of a translating body and its rate: upstream
/// `LinearTranslationRigidBodyMsg`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LinearTranslationRigidBodyMsg {
    /// Upstream `rho` \[m\]: body displacement.
    pub rho_m: f64,
    /// Upstream `rhoDot` \[m/s\]: body displacement rate.
    pub rho_dot_mps: f64,
}

impl_telemetry!(LinearTranslationRigidBodyMsg { rho_m, rho_dot_mps });
