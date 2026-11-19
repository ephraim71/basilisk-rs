//! Generalized forces on the joints of a multibody scene (upstream C++
//! `MJJointReactionsMsgPayload`).

use serde::{Deserialize, Serialize};

/// Kind of a joint of a multibody (MuJoCo) scene: upstream's `jointTypes`
/// codes.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum MjJointType {
    /// Six-degree-of-freedom joint of a free-floating body; upstream `0`.
    #[default]
    Free = 0,
    /// Three-degree-of-freedom rotational joint; upstream `1`.
    Ball = 1,
    /// One-degree-of-freedom translational joint; upstream `2`.
    Slide = 2,
    /// One-degree-of-freedom rotational joint; upstream `3`.
    Hinge = 3,
}

/// Joint topology and the generalized forces acting on every degree of
/// freedom of the scene: upstream `MJJointReactionsMsg`.
///
/// Joints are listed in scene order; each kinematic tree (one spacecraft)
/// starts with its free joint. The force vectors have one entry per degree
/// of freedom, in the order of the generalized coordinates. Upstream's
/// payload holds `std::vector`s, so the message has no telemetry columns.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MjJointReactionsMsg {
    /// Upstream `jointTreeIdx` [-]: kinematic tree of each joint.
    pub joint_tree_idx: Vec<usize>,
    /// Upstream `jointParentBodyIdx` [-]: parent body of each joint.
    pub joint_parent_body_idx: Vec<usize>,
    /// Upstream `jointTypes` [-]: kind of each joint.
    pub joint_types: Vec<MjJointType>,
    /// Upstream `jointDOFStart` [-]: first degree of freedom of each joint.
    pub joint_dof_start: Vec<usize>,
    /// Upstream `biasForces` [-]: bias (Coriolis, centrifugal and
    /// gravitational) generalized forces.
    pub bias_forces: Vec<f64>,
    /// Upstream `passiveForces` [-]: passive (spring, damper) generalized
    /// forces.
    pub passive_forces: Vec<f64>,
    /// Upstream `constraintForces` [-]: constraint generalized forces.
    pub constraint_forces: Vec<f64>,
    /// Upstream `appliedForces` [-]: externally applied generalized forces.
    pub applied_forces: Vec<f64>,
    /// Upstream `actuatorForces` [-]: actuator generalized forces.
    pub actuator_forces: Vec<f64>,
}
