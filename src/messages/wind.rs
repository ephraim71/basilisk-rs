//! Local air velocity at a spacecraft (upstream `WindMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Velocity of the air mass at a spacecraft, inertial components: upstream
/// `WindMsg`, published by the wind models.
///
/// With a co-rotating atmosphere, `air_velocity_inertial_mps =
/// omega_planet x r + wind_velocity_inertial_mps`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct WindMsg {
    /// Upstream `v_air_N` \[m/s\]: full local air velocity, the one drag is
    /// computed against.
    pub air_velocity_inertial_mps: Vector3<f64>,
    /// Upstream `v_wind_N` \[m/s\]: the wind perturbation alone, without the
    /// co-rotation of the atmosphere.
    pub wind_velocity_inertial_mps: Vector3<f64>,
}

impl_telemetry!(WindMsg {
    air_velocity_inertial_mps,
    wind_velocity_inertial_mps,
});
