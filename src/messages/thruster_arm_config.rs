//! Geometry of thrusters mounted on jointed arms (upstream C++
//! `THRArmConfigMsgPayload`).

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

/// Static configuration of spacecraft whose thrusters sit on arms of hinged
/// joints: upstream `THRArmConfigMsg`.
///
/// Arm joints are numbered arm by arm: the joints of arm 0, then those of
/// arm 1, and so on (`arm_joint_count` gives the split); the per-joint
/// vectors below follow that flat order. Each joint `C` is described in the
/// frame of its parent joint `P`, the hub body frame `B` for the first joint
/// of an arm. Upstream flattens the vectors and DCMs column-major into
/// `std::vector`s; here each is one entry of a `Vec`. The message has no
/// telemetry columns.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ThrusterArmConfigMsg {
    /// Upstream `hubMass` \[kg\]: hub mass.
    pub hub_mass_kg: f64,
    /// Upstream `r_BcB_B` \[m\]: hub centre of mass relative to the body frame
    /// origin, body components.
    pub r_bcb_b_m: Vector3<f64>,
    /// Upstream `bodyArmIdx` [-]: arm each arm body belongs to.
    pub body_arm_idx: Vec<usize>,
    /// Upstream `bodyJointIdx` [-]: local index of the closest parent joint
    /// of each arm body.
    pub body_joint_idx: Vec<usize>,
    /// Upstream `bodyMass` \[kg\]: mass of each arm body.
    pub body_mass_kg: Vec<f64>,
    /// Upstream `r_LcP_P` \[m\]: centre of mass of each arm body relative to
    /// its parent joint, parent-joint components.
    pub r_lcp_p_m: Vec<Vector3<f64>>,
    /// Upstream `armJointCount` [-]: number of hinged joints of each arm.
    pub arm_joint_count: Vec<usize>,
    /// Upstream `r_CP_P` \[m\]: each joint relative to its parent joint,
    /// parent-joint components.
    pub r_cp_p_m: Vec<Vector3<f64>>,
    /// Upstream `shat_P` [-]: spin axis of each joint, parent-joint
    /// components.
    pub s_hat_p: Vec<Vector3<f64>>,
    /// Upstream `dcm_C0P` [-]: DCM from each joint's parent frame to the
    /// joint frame at zero joint angle.
    pub dcm_c0p: Vec<Matrix3<f64>>,
    /// Upstream `armTreeIdx` [-]: kinematic tree (spacecraft) of each arm.
    pub arm_tree_idx: Vec<usize>,
    /// Upstream `thrArmIdx` [-]: arm each thruster is mounted on.
    pub thr_arm_idx: Vec<usize>,
    /// Upstream `thrArmJointIdx` [-]: local index of the closest parent joint
    /// of each thruster.
    pub thr_arm_joint_idx: Vec<usize>,
    /// Upstream `r_TP_P` \[m\]: each thruster relative to its parent joint,
    /// parent-joint components.
    pub r_tp_p_m: Vec<Vector3<f64>>,
    /// Upstream `fhat_P` [-]: force direction of each thruster, parent-joint
    /// components.
    pub f_hat_p: Vec<Vector3<f64>>,
}
