//! Bodies, sites and joints of a scene: ports of upstream Basilisk
//! `MJBody`, `MJSite` and `MJJoint` (`MJScalarJoint`, `MJBallJoint`,
//! `MJFreeJoint`) from `simulation/mujocoDynamics/_GeneralModuleFiles`.
//!
//! These hold the names, MuJoCo ids and message ports of the objects. Their
//! joint states live in the scene's integrated
//! [`MjSceneState`](super::MjSceneState), so the state setters upstream puts
//! on the objects are on the [`BodyMut`](super::BodyMut) view the scene hands
//! out.

use crate::messages::SpacecraftStateMsg;
use crate::messages::{Input, Output, ScalarJointStateMsg, SpacecraftMassPropsMsg};

/// Kind of a MuJoCo joint (upstream `mjtJoint`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MjJointKind {
    /// Six degrees of freedom: position and a scalar-first attitude
    /// quaternion (`mjJNT_FREE`).
    Free,
    /// Three rotational degrees of freedom: a scalar-first quaternion
    /// (`mjJNT_BALL`).
    Ball,
    /// One translational degree of freedom (`mjJNT_SLIDE`).
    Slide,
    /// One rotational degree of freedom (`mjJNT_HINGE`).
    Hinge,
}

impl MjJointKind {
    /// Entries of the joint in `qpos`.
    pub fn qpos_len(self) -> usize {
        match self {
            Self::Free => 7,
            Self::Ball => 4,
            Self::Slide | Self::Hinge => 1,
        }
    }

    /// Entries of the joint in `qvel`.
    pub fn qvel_len(self) -> usize {
        match self {
            Self::Free => 6,
            Self::Ball => 3,
            Self::Slide | Self::Hinge => 1,
        }
    }

    /// Offset of the attitude quaternion within the joint's `qpos`, if it
    /// has one.
    pub(crate) fn quaternion_offset(self) -> Option<usize> {
        match self {
            Self::Free => Some(3),
            Self::Ball => Some(0),
            Self::Slide | Self::Hinge => None,
        }
    }
}

/// Where a joint's coordinates are in `qpos` and `qvel`, in MuJoCo joint
/// order (upstream `jnt_qposadr`, `jnt_dofadr`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct JointSlot {
    pub(crate) kind: MjJointKind,
    pub(crate) qpos_adr: usize,
    pub(crate) qvel_adr: usize,
}

/// A site: a frame fixed to a body (upstream `MJSite`).
///
/// After every forward-kinematics pass the scene writes the site frame
/// state to [`state_out_msg`](Self::state_out_msg).
#[derive(Clone, Debug)]
pub struct MjSite {
    pub(crate) name: String,
    pub(crate) id: usize,
    /// Upstream `stateOutMsg`: the site frame `S` relative to the inertial
    /// frame (`r_BN_N`, `v_BN_N`, `sigma_BN`, `omega_BN_B` of the site; the
    /// accumulated-ΔV fields stay zero).
    pub state_out_msg: Output<SpacecraftStateMsg>,
}

impl MjSite {
    pub(crate) fn new(name: String) -> Self {
        Self {
            name,
            id: 0,
            state_out_msg: Output::default(),
        }
    }

    /// Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// MuJoCo site id (upstream `getId`).
    pub fn id(&self) -> usize {
        self.id
    }
}

/// A hinge or slide joint (upstream `MJScalarJoint`).
#[derive(Clone, Debug)]
pub struct MjScalarJoint {
    pub(crate) name: String,
    pub(crate) kind: MjJointKind,
    pub(crate) id: usize,
    pub(crate) qpos_adr: usize,
    pub(crate) qvel_adr: usize,
    pub(crate) axis: [f64; 3],
    pub(crate) constrained_equality: String,
    pub(crate) constrained_tendon: String,
    pub(crate) constrained_tendon_id: usize,
    /// Upstream `stateOutMsg`: the joint angle \[rad\] or displacement \[m\].
    pub state_out_msg: Output<ScalarJointStateMsg>,
    /// Upstream `stateDotOutMsg`: the joint rate \[rad/s\] or \[m/s\].
    pub state_dot_out_msg: Output<ScalarJointStateMsg>,
    /// Upstream `constrainedStateInMsg`: while connected, the joint is held
    /// at the commanded value by its constrained equality.
    pub constrained_state_in_msg: Input<ScalarJointStateMsg>,
}

impl MjScalarJoint {
    /// Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Upstream `isHinge`: a rotational (rather than translational) joint.
    pub fn is_hinge(&self) -> bool {
        self.kind == MjJointKind::Hinge
    }

    /// Upstream `getAxis`: the joint axis in the body frame.
    pub fn axis(&self) -> [f64; 3] {
        self.axis
    }

    /// Upstream `getQposAdr`.
    pub fn qpos_adr(&self) -> usize {
        self.qpos_adr
    }

    /// Upstream `getQvelAdr`.
    pub fn qvel_adr(&self) -> usize {
        self.qvel_adr
    }

    /// Upstream `getConstrainedEquality`: name of the equality that holds
    /// the joint at [`constrained_state_in_msg`](Self::constrained_state_in_msg)
    /// (`_basilisk_constrainedEquality_<joint>`).
    pub fn constrained_equality(&self) -> &str {
        &self.constrained_equality
    }
}

/// A ball joint (upstream `MJBallJoint`).
#[derive(Clone, Debug)]
pub struct MjBallJoint {
    pub(crate) name: String,
    pub(crate) id: usize,
    pub(crate) qpos_adr: usize,
    pub(crate) qvel_adr: usize,
}

impl MjBallJoint {
    /// Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Upstream `getQposAdr`: first entry of the quaternion in `qpos`.
    pub fn qpos_adr(&self) -> usize {
        self.qpos_adr
    }

    /// Upstream `getQvelAdr`: first entry of the body rate in `qvel`.
    pub fn qvel_adr(&self) -> usize {
        self.qvel_adr
    }
}

/// A free joint (upstream `MJFreeJoint`): position, then a scalar-first
/// body-to-world quaternion in `qpos`; inertial velocity, then the body
/// rate in body components in `qvel`.
#[derive(Clone, Debug)]
pub struct MjFreeJoint {
    pub(crate) name: String,
    pub(crate) id: usize,
    pub(crate) qpos_adr: usize,
    pub(crate) qvel_adr: usize,
}

impl MjFreeJoint {
    /// Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Upstream `getQposAdr`.
    pub fn qpos_adr(&self) -> usize {
        self.qpos_adr
    }

    /// Upstream `getQvelAdr`.
    pub fn qvel_adr(&self) -> usize {
        self.qvel_adr
    }
}

/// A body of the scene (upstream `MJBody`).
///
/// Every body has a `<name>_com` site at its center of mass (oriented like
/// the body frame) and a `<name>_origin` site at its frame origin; the scene
/// adds them when the XML does not.
#[derive(Clone, Debug)]
pub struct MjBody {
    pub(crate) name: String,
    pub(crate) id: usize,
    pub(crate) sites: Vec<MjSite>,
    pub(crate) scalar_joints: Vec<MjScalarJoint>,
    pub(crate) ball_joint: Option<MjBallJoint>,
    pub(crate) free_joint: Option<MjFreeJoint>,
    /// Upstream `massPropertiesOutMsg`: the body mass \[kg\] in `mass_kg`
    /// (the other fields stay zero, as upstream fills only `massSC`).
    pub mass_properties_out_msg: Output<SpacecraftMassPropsMsg>,
    /// Upstream `derivativeMassPropertiesInMsg`: while connected, its
    /// `mass_kg` is the body's mass rate \[kg/s\] (upstream reads `massSC`
    /// of an `SCMassPropsMsg`).
    pub derivative_mass_properties_in_msg: Input<SpacecraftMassPropsMsg>,
}

impl MjBody {
    /// Upstream `getName`.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// MuJoCo body id (upstream `getId`).
    pub fn id(&self) -> usize {
        self.id
    }

    /// Upstream `hasSite`.
    pub fn has_site(&self, name: &str) -> bool {
        self.sites.iter().any(|site| site.name == name)
    }

    /// The body's sites, in specification order.
    pub fn sites(&self) -> &[MjSite] {
        &self.sites
    }

    /// Upstream `getSite`.
    ///
    /// # Panics
    ///
    /// If the body has no such site (upstream raises `BasiliskError`).
    pub fn site(&self, name: &str) -> &MjSite {
        self.sites
            .iter()
            .find(|site| site.name == name)
            .unwrap_or_else(|| panic!("Unknown site '{name}' in body '{}'", self.name))
    }

    /// Upstream `getCenterOfMass`: the `<body>_com` site.
    pub fn center_of_mass(&self) -> &MjSite {
        self.site(&format!("{}_com", self.name))
    }

    /// Upstream `getOrigin`: the `<body>_origin` site.
    pub fn origin(&self) -> &MjSite {
        self.site(&format!("{}_origin", self.name))
    }

    /// The hinge and slide joints, in specification order.
    pub fn scalar_joints(&self) -> &[MjScalarJoint] {
        &self.scalar_joints
    }

    /// Upstream `getScalarJoint`.
    ///
    /// # Panics
    ///
    /// If the body has no such scalar joint.
    pub fn scalar_joint(&self, name: &str) -> &MjScalarJoint {
        self.scalar_joints
            .iter()
            .find(|joint| joint.name == name)
            .unwrap_or_else(|| panic!("Unknown scalar joint '{name}' in body '{}'", self.name))
    }

    /// Mutable form of [`scalar_joint`](Self::scalar_joint), to connect its
    /// constrained-state input.
    pub fn scalar_joint_mut(&mut self, name: &str) -> &mut MjScalarJoint {
        let body = self.name.clone();
        self.scalar_joints
            .iter_mut()
            .find(|joint| joint.name == name)
            .unwrap_or_else(|| panic!("Unknown scalar joint '{name}' in body '{body}'"))
    }

    /// Upstream `hasBallJoint`.
    pub fn has_ball_joint(&self) -> bool {
        self.ball_joint.is_some()
    }

    /// Upstream `getBallJoint`.
    ///
    /// # Panics
    ///
    /// If the body has no ball joint.
    pub fn ball_joint(&self) -> &MjBallJoint {
        self.ball_joint.as_ref().unwrap_or_else(|| {
            panic!(
                "Tried to get a ball joint for a body without ball joints: {}",
                self.name
            )
        })
    }

    /// Upstream `isFree`.
    pub fn is_free(&self) -> bool {
        self.free_joint.is_some()
    }

    /// Upstream `getFreeJoint`.
    ///
    /// # Panics
    ///
    /// If the body has no free joint.
    pub fn free_joint(&self) -> &MjFreeJoint {
        self.free_joint.as_ref().unwrap_or_else(|| {
            panic!(
                "Tried to get a free joint for a body without free joints: {}",
                self.name
            )
        })
    }

    /// Upstream `areJoints3DTranslation`: the first three scalar joints are
    /// slides along the body x, y and z axes, in that order.
    pub(crate) fn has_3d_translation(&self) -> bool {
        self.scalar_joints.len() >= 3
            && self.scalar_joints[..3]
                .iter()
                .enumerate()
                .all(|(axis, joint)| !joint.is_hinge() && (joint.axis[axis] - 1.0).abs() <= 1e-10)
    }
}
