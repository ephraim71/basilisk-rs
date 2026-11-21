//! Requested net magnetic dipole (upstream `DipoleRequestBodyMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Desired net torque-bar dipole in the body frame: upstream
/// `DipoleRequestBodyMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DipoleRequestBodyMsg {
    /// Upstream `dipole_B` [A m^2]: desired net dipole, body components.
    pub dipole_b_am2: Vector3<f64>,
}

impl_telemetry!(DipoleRequestBodyMsg { dipole_b_am2 });
