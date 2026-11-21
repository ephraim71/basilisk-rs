//! Loads and violation of a constraint between two spacecraft (upstream
//! `ConstDynEffectorMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Constraint force and torques on two connected spacecraft, the direction
/// constraint violation and the low-pass-filtered load magnitudes: upstream
/// `ConstDynEffectorMsg`, written by
/// [`ConstraintDynamicEffector`](crate::dynamics::constraint_dynamic_effector::ConstraintDynamicEffector).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ConstDynEffectorMsg {
    /// Upstream `Fc_N` \[N\]: constraint force on spacecraft 1, inertial
    /// components. Spacecraft 2 receives its negative.
    pub force_inertial_n: Vector3<f64>,
    /// Upstream `L1_B1` \[N·m\]: constraint torque on spacecraft 1 about its
    /// body origin, B1 components.
    pub torque_1_body_nm: Vector3<f64>,
    /// Upstream `L2_B2` \[N·m\]: constraint torque on spacecraft 2 about its
    /// body origin, B2 components.
    pub torque_2_body_nm: Vector3<f64>,
    /// Upstream `psi_N` \[m\]: direction constraint violation, inertial
    /// components.
    pub direction_violation_inertial_m: Vector3<f64>,
    /// Upstream `Fc_mag_filtered` \[N\]: low-pass-filtered magnitude of the
    /// constraint force.
    pub force_magnitude_filtered_n: f64,
    /// Upstream `L1_mag_filtered` \[N·m\]: low-pass-filtered magnitude of the
    /// torque on spacecraft 1.
    pub torque_1_magnitude_filtered_nm: f64,
    /// Upstream `L2_mag_filtered` \[N·m\]: low-pass-filtered magnitude of the
    /// torque on spacecraft 2.
    pub torque_2_magnitude_filtered_nm: f64,
}

impl_telemetry!(ConstDynEffectorMsg {
    force_inertial_n,
    torque_1_body_nm,
    torque_2_body_nm,
    direction_violation_inertial_m,
    force_magnitude_filtered_n,
    torque_1_magnitude_filtered_nm,
    torque_2_magnitude_filtered_nm,
});
