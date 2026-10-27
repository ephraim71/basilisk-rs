//! One reaction wheel's FSW configuration (upstream
//! `RWConfigElementMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Configuration of a single reaction wheel: upstream `RWConfigElementMsg`,
/// also the element type of [`super::RwConstellationMsg`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RwConfigElementMsg {
    /// Upstream `gsHat_B` [-]: wheel spin-axis unit vector, body components.
    pub gs_hat_b: Vector3<f64>,
    /// Upstream `Js` [kg m^2]: spin-axis inertia of the wheel.
    pub spin_axis_inertia_kg_m2: f64,
    /// Upstream `uMax` [N m]: maximum motor torque.
    pub max_motor_torque_nm: f64,
}

impl_telemetry!(RwConfigElementMsg {
    gs_hat_b,
    spin_axis_inertia_kg_m2,
    max_motor_torque_nm,
});
