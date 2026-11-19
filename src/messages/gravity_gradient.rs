//! Gravity-gradient torque (upstream `GravityGradientMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Gravity-gradient torque on a spacecraft: upstream `GravityGradientMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct GravityGradientMsg {
    /// Upstream `gravityGradientTorque_B` \[N m\]: first-order gravity-gradient
    /// torque, summed over the source planets, body components.
    pub gravity_gradient_torque_body_nm: Vector3<f64>,
}

impl_telemetry!(GravityGradientMsg {
    gravity_gradient_torque_body_nm
});
