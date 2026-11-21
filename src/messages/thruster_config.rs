//! One thruster's FSW configuration (upstream `THRConfigMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Configuration of a single thruster: upstream `THRConfigMsg`, also the
/// element type of [`super::ThrusterArrayConfigMsg`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ThrusterConfigMsg {
    /// Upstream `rThrust_B` \[m\]: thruster location, body components.
    pub position_thrust_b_m: Vector3<f64>,
    /// Upstream `tHatThrust_B` [-]: unit thrust direction, body components.
    pub t_hat_thrust_b: Vector3<f64>,
    /// Upstream `maxThrust` \[N\]: maximum thrust.
    pub max_thrust_n: f64,
    /// Upstream `swirlTorque` [N m]: swirl torque about the thrust axis.
    pub swirl_torque_nm: f64,
}

impl_telemetry!(ThrusterConfigMsg {
    position_thrust_b_m,
    t_hat_thrust_b,
    max_thrust_n,
    swirl_torque_nm,
});
