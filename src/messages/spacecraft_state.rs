//! Spacecraft truth state: the parts of upstream `SCStatesMsgPayload` that
//! basilisk-rs publishes from [`Spacecraft`](crate::spacecraft::Spacecraft).

use nalgebra::{Quaternion, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Spacecraft hub state (upstream `SCStatesMsgPayload`).
///
/// The angular acceleration and non-conservative acceleration upstream also
/// carries are in [`SpacecraftDiagnosticsMsg`](super::SpacecraftDiagnosticsMsg).
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SpacecraftStateMsg {
    /// Upstream `r_BN_N` \[m\]: inertial position of the body frame origin.
    pub position_m: Vector3<f64>,
    /// Upstream `v_BN_N` \[m/s\]: inertial velocity of the body frame origin.
    pub velocity_mps: Vector3<f64>,
    /// Upstream `sigma_BN`: body attitude relative to the inertial frame (MRP).
    pub sigma_bn: Vector3<f64>,
    /// Upstream `omega_BN_B` \[rad/s\]: body rate, body-frame components.
    pub omega_radps: Vector3<f64>,
    /// Upstream `TotalAccumDVBdy` (`dvAccum_CN_B`) \[m/s\]: accumulated
    /// non-gravitational velocity change of the center of mass, each step's
    /// increment in body-frame components at the end of that step.
    #[serde(default)]
    pub accumulated_dv_cn_b_mps: Vector3<f64>,
    /// Upstream `TotalAccumDV_BN_B` (`dvAccum_BN_B`) \[m/s\]: the same for
    /// the body frame origin, which also picks up the velocity of rotation
    /// about an offset center of mass.
    #[serde(default)]
    pub accumulated_dv_bn_b_mps: Vector3<f64>,
    /// Upstream `TotalAccumDV_CN_N` (`dvAccum_CN_N`) \[m/s\]: accumulated
    /// non-gravitational velocity change of the center of mass, inertial
    /// components.
    #[serde(default)]
    pub accumulated_dv_cn_n_mps: Vector3<f64>,
}

impl SpacecraftStateMsg {
    pub fn body_to_inertial(&self) -> UnitQuaternion<f64> {
        let sigma_squared = self.sigma_bn.norm_squared();
        let denom = 1.0 + sigma_squared;
        let q_scalar = (1.0 - sigma_squared) / denom;
        let q_vector = 2.0 * self.sigma_bn / denom;
        UnitQuaternion::new_normalize(Quaternion::new(
            q_scalar, q_vector.x, q_vector.y, q_vector.z,
        ))
    }

    pub fn inertial_to_body(&self) -> UnitQuaternion<f64> {
        self.body_to_inertial().inverse()
    }
}

impl_telemetry!(SpacecraftStateMsg {
    position_m,
    velocity_mps,
    sigma_bn,
    omega_radps,
    accumulated_dv_cn_b_mps,
    accumulated_dv_bn_b_mps,
    accumulated_dv_cn_n_mps,
});
