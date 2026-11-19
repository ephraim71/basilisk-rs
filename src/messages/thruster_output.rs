//! One thruster's delivered force and torque (upstream
//! `THROutputMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// State of a single thruster as the dynamics deliver it: upstream
/// `THROutputMsg`, one message per thruster.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ThrusterOutputMsg {
    /// Upstream `maxThrust` \[N\]: steady-state thrust.
    pub max_thrust_n: f64,
    /// Upstream `thrustFactor` [-]: current fraction of the steady-state
    /// thrust from the ramp up or down.
    pub thrust_factor: f64,
    /// Upstream `thrustBlowDownFactor` [-]: current thrust fraction from tank
    /// blow-down.
    pub thrust_blow_down_factor: f64,
    /// Upstream `ispBlowDownFactor` [-]: current specific-impulse fraction
    /// from tank blow-down.
    pub isp_blow_down_factor: f64,
    /// Upstream `thrustForce` \[N\]: thrust magnitude.
    pub thrust_force_n: f64,
    /// Upstream `thrustForce_B` \[N\]: thrust force, body components.
    pub thrust_force_b_n: Vector3<f64>,
    /// Upstream `thrustTorquePntB_B` \[N m\]: thrust torque about point B,
    /// body components.
    pub thrust_torque_pnt_b_b_nm: Vector3<f64>,
    /// Upstream `thrusterLocation` \[m\]: thruster position.
    pub thruster_location_m: Vector3<f64>,
    /// Upstream `thrusterDirection` [-]: unit thrust direction.
    pub thruster_direction: Vector3<f64>,
}

impl_telemetry!(ThrusterOutputMsg {
    max_thrust_n,
    thrust_factor,
    thrust_blow_down_factor,
    isp_blow_down_factor,
    thrust_force_n,
    thrust_force_b_n,
    thrust_torque_pnt_b_b_nm,
    thruster_location_m,
    thruster_direction,
});
