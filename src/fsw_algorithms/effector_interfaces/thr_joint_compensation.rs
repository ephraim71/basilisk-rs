//! Joint motor torques that hold thruster arms still while their thrusters
//! fire: port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/thrJointCompensation`.
//!
//! Every spacecraft of the multibody scene is a six-degree-of-freedom hub
//! carrying arms of hinged joints, with pure-force thrusters mounted on the
//! arms. Each update
//!
//! 1. walks every arm from the hub outward, composing the pose of each
//!    joint frame `C` in the hub body frame `B` from its parent's: with the
//!    joint angle `φ` about the spin axis `ŝ`,
//!    `[CC0] = cos φ [I] - sin φ [ŝ×] + (1 - cos φ) ŝ ŝ^T`,
//!    `[CB] = [CC0] [C0P] [PB]` and `r_C = r_P + [PB]^T r_CP`;
//! 2. sums each thruster's force `f` and its torque `r_T × f` about the
//!    body origin onto its spacecraft's hub, and its torque
//!    `ŝ · ((r_T - r_J) × f)` onto every joint `J` between the hub and the
//!    thruster;
//! 3. computes, for each spacecraft, the hub acceleration
//!    `a = [M_bb]^-1 (F_thr + τ_b)` from the thruster load and the
//!    non-actuator generalized forces `τ` (passive + constraint + applied -
//!    bias) and the joint motor torques that keep the joints from
//!    accelerating with it,
//!
//!    ```text
//!    u = [M_θb] a - τ_θ - τ_thr
//!    ```
//!
//!    optionally clamped to `±u_max`.
//!
//! The module has one joint-angle input per hinged joint and one force
//! input per thruster, made by [`ThrJointCompensation::new`] from
//! [`ThrJointCompensationConfig::num_hinged_joints`] and
//! [`ThrJointCompensationConfig::num_thrusters`], and one motor-torque
//! output per joint, registered with
//! [`ThrJointCompensation::add_motor_torque_output`] (upstream's
//! `addHingedJoint` and `addThruster` add them). Joints and thrusters are in
//! scene order; arm joints are matched to the scene's hinges tree by tree,
//! in arm order. The scene messages come from a multibody (MuJoCo)
//! simulation.

use std::collections::BTreeMap;
use std::fmt;

use nalgebra::{DVector, Matrix3, Vector3};

use super::joint_trees::{
    KinematicTrees, SceneError, check_mass_matrix, kinematic_trees, non_actuator_forces, saturate,
    solve_symmetric_vector, sub_matrix, sub_vector, tree_dofs,
};
use crate::messages::{
    Input, MjJointReactionsMsg, MjSystemMassMatrixMsg, Output, ScalarJointStateMsg,
    SingleActuatorMsg, ThrusterArmConfigMsg,
};
use crate::utilities::rigid_body_kinematics::tilde;
use crate::{Module, SimulationContext};

/// Degrees of freedom of a free joint: hub translation, then rotation.
const BASE_DOFS: usize = 6;

/// User-settable parameters of [`ThrJointCompensation`].
#[derive(Clone, Debug)]
pub struct ThrJointCompensationConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Number of hinged joints in the scene (upstream calls
    /// `addHingedJoint` once per joint); [`ThrJointCompensation::new`] makes
    /// the joint-angle inputs from it.
    pub num_hinged_joints: usize,
    /// Number of thrusters (upstream calls `addThruster` once per
    /// thruster); [`ThrJointCompensation::new`] makes the thruster-force
    /// inputs from it.
    pub num_thrusters: usize,
    /// Upstream `uMax` \[N m\]: optional torque limit of each joint (upstream
    /// leaves the vector empty for none).
    pub u_max_nm: Option<Vec<f64>>,
}

impl Default for ThrJointCompensationConfig {
    fn default() -> Self {
        Self {
            name: "thrJointCompensation".to_string(),
            num_hinged_joints: 0,
            num_thrusters: 0,
            u_max_nm: None,
        }
    }
}

/// Why an update publishes nothing. Upstream raises a `BasiliskError` (or
/// reads out of bounds, for the shape errors) before writing the outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UpdateError {
    Scene(SceneError),
    /// `arm_tree_idx` and `arm_joint_count` differ in length.
    ArmListLengths,
    /// An arm's tree is not in the scene.
    ArmTreeNotFound {
        arm: usize,
        tree: usize,
    },
    /// An arm needs more hinges than its tree has left.
    ArmHingesUnavailable {
        arm: usize,
        tree: usize,
        needed: usize,
        available: usize,
    },
    /// The per-joint arm vectors hold fewer entries than there are arm
    /// joints.
    ArmJointConfigLength,
    /// The per-thruster vectors differ in length, or there are more
    /// thrusters than thruster inputs.
    ThrusterConfigLength,
    /// A thruster is mounted past the end of the arms or of its arm.
    ThrusterMountOutOfRange {
        thruster: usize,
    },
    /// `u_max_nm` does not hold one limit per joint.
    TorqueLimitLength,
}

impl From<SceneError> for UpdateError {
    fn from(error: SceneError) -> Self {
        Self::Scene(error)
    }
}

impl fmt::Display for UpdateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "thrJointCompensation: ")?;
        match self {
            Self::Scene(error) => write!(f, "{error}"),
            Self::ArmListLengths => {
                write!(f, "arm_tree_idx and arm_joint_count differ in size")
            }
            Self::ArmTreeNotFound { arm, tree } => write!(
                f,
                "tree {tree} for arm {arm} was not found in joint reaction tree data"
            ),
            Self::ArmHingesUnavailable {
                arm,
                tree,
                needed,
                available,
            } => write!(
                f,
                "arm {arm} expects {needed} hinge joints in tree {tree}, but only {available} \
                 are left"
            ),
            Self::ArmJointConfigLength => write!(
                f,
                "r_cp_p_m, s_hat_p and dcm_c0p need one entry per arm joint"
            ),
            Self::ThrusterConfigLength => write!(
                f,
                "thr_arm_idx, thr_arm_joint_idx, r_tp_p_m and f_hat_p need one entry per \
                 thruster, and at most num_thrusters"
            ),
            Self::ThrusterMountOutOfRange { thruster } => {
                write!(f, "thruster {thruster} is mounted past the end of its arm")
            }
            Self::TorqueLimitLength => {
                write!(f, "size of u_max_nm does not match num_hinged_joints")
            }
        }
    }
}

/// How the arm joints map onto the scene, found on the first update after a
/// reset (upstream's `treeInfoInitialized` block).
#[derive(Clone, Debug)]
struct ArmLayout {
    /// Upstream `treeMap`.
    trees: KinematicTrees,
    /// Upstream `hingeToTree`: the tree of each scene hinge.
    hinge_to_tree: Vec<usize>,
    /// Upstream `armJointStart`: flat index of each arm's first joint.
    arm_joint_start: Vec<usize>,
    /// Upstream `armHingeGlobalIdx`: scene hinge of each flat arm joint.
    arm_hinge_global: Vec<usize>,
}

/// Pose of a joint frame `C` in the hub body frame (upstream
/// `JointPoseCache`).
#[derive(Clone, Copy, Debug)]
struct JointPose {
    dcm_cb: Matrix3<f64>,
    r_cb_b: Vector3<f64>,
}

/// Motor torques that cancel the thruster loads on jointed arms: upstream
/// `ThrJointCompensation`.
#[derive(Clone, Debug)]
pub struct ThrJointCompensation {
    /// Module configuration.
    pub config: ThrJointCompensationConfig,
    /// Upstream `armConfigInMsg`: arm and thruster geometry, read at reset.
    pub arm_config_in_msg: Input<ThrusterArmConfigMsg>,
    /// Upstream `massMatrixInMsg`: scene mass matrix.
    pub mass_matrix_in_msg: Input<MjSystemMassMatrixMsg>,
    /// Upstream `reactionForcesInMsg`: scene joints and generalized forces.
    pub reaction_forces_in_msg: Input<MjJointReactionsMsg>,
    /// Upstream `jointStatesInMsgs`: angle of each hinged joint, one input
    /// per joint in scene order.
    pub joint_states_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `thrForcesInMsgs`: force \[N\] of each thruster, one input
    /// per thruster.
    pub thr_forces_in_msgs: Vec<Input<SingleActuatorMsg>>,
    /// Upstream `motorTorquesOutMsgs`: motor torque \[N m\] of each joint,
    /// one output per joint in scene order, registered with
    /// [`ThrJointCompensation::add_motor_torque_output`].
    pub motor_torques_out_msgs: Vec<Output<SingleActuatorMsg>>,
    /// Upstream `kinCfg`: the arm configuration read at reset.
    arm_config: ThrusterArmConfigMsg,
    layout: Option<ArmLayout>,
}

impl ThrJointCompensation {
    /// Builds the module with one unconnected angle input per configured
    /// joint, one unconnected force input per configured thruster and no
    /// outputs.
    pub fn new(config: ThrJointCompensationConfig) -> Self {
        // Not `vec![_; n]`: cloning a port aliases it.
        let joint_states_in_msgs = (0..config.num_hinged_joints)
            .map(|_| Input::default())
            .collect();
        let thr_forces_in_msgs = (0..config.num_thrusters)
            .map(|_| Input::default())
            .collect();
        Self {
            config,
            arm_config_in_msg: Input::default(),
            mass_matrix_in_msg: Input::default(),
            reaction_forces_in_msg: Input::default(),
            joint_states_in_msgs,
            thr_forces_in_msgs,
            motor_torques_out_msgs: Vec::new(),
            arm_config: ThrusterArmConfigMsg::default(),
            layout: None,
        }
    }

    /// Registers one independently connectable motor-torque output. The
    /// outputs follow the scene's joint order.
    pub fn add_motor_torque_output(&mut self) -> &Output<SingleActuatorMsg> {
        self.motor_torques_out_msgs.push(Output::default());
        self.motor_torques_out_msgs
            .last()
            .expect("registered motor torque output should be available")
    }

    /// Upstream `Reset`: reports missing inputs, reads the arm configuration
    /// and forgets the arm layout.
    ///
    /// # Panics
    ///
    /// If the number of joint or thruster inputs no longer matches the
    /// configuration: the inputs are made from it in `new`.
    fn reset_module(&mut self) {
        assert!(
            self.joint_states_in_msgs.len() == self.config.num_hinged_joints
                && self.thr_forces_in_msgs.len() == self.config.num_thrusters,
            "thrJointCompensation needs one joint_states_in_msgs input per \
             config.num_hinged_joints joint and one thr_forces_in_msgs input per \
             config.num_thrusters thruster"
        );
        for (connected, port) in [
            (self.arm_config_in_msg.is_connected(), "arm_config_in_msg"),
            (self.mass_matrix_in_msg.is_connected(), "mass_matrix_in_msg"),
            (
                self.reaction_forces_in_msg.is_connected(),
                "reaction_forces_in_msg",
            ),
        ] {
            if !connected {
                log::error!("thrJointCompensation.{port} is not connected");
            }
        }
        if self.joint_states_in_msgs.is_empty() {
            log::error!("thrJointCompensation has no hinged joints");
        }
        if self.thr_forces_in_msgs.is_empty() {
            log::error!("thrJointCompensation has no thrusters");
        }
        for (index, input) in self.joint_states_in_msgs.iter().enumerate() {
            if !input.is_connected() {
                log::error!("thrJointCompensation.joint_states_in_msgs[{index}] is not connected");
            }
        }
        for (index, input) in self.thr_forces_in_msgs.iter().enumerate() {
            if !input.is_connected() {
                log::error!("thrJointCompensation.thr_forces_in_msgs[{index}] is not connected");
            }
        }
        if self.motor_torques_out_msgs.len() != self.config.num_hinged_joints {
            log::error!(
                "thrJointCompensation has {} hinged joints but {} motor torque outputs",
                self.config.num_hinged_joints,
                self.motor_torques_out_msgs.len()
            );
        }
        self.arm_config = self.arm_config_in_msg.read();
        self.layout = None;
    }

    /// The kinematic trees and the scene hinge of every arm joint: the arms
    /// take their tree's hinges in order.
    fn arm_layout(&self, reactions: &MjJointReactionsMsg) -> Result<ArmLayout, UpdateError> {
        let arms = &self.arm_config;
        let trees = kinematic_trees(reactions, self.config.num_hinged_joints)?;
        let mut hinge_to_tree = vec![0; self.config.num_hinged_joints];
        for (&tree_index, tree) in &trees {
            for &hinge in &tree.hinge_global {
                hinge_to_tree[hinge] = tree_index;
            }
        }
        if arms.arm_tree_idx.len() != arms.arm_joint_count.len() {
            return Err(UpdateError::ArmListLengths);
        }
        let mut arm_joint_start = Vec::with_capacity(arms.arm_joint_count.len());
        let mut num_arm_joints = 0;
        for &count in &arms.arm_joint_count {
            arm_joint_start.push(num_arm_joints);
            num_arm_joints += count;
        }
        let mut arm_hinge_global = Vec::with_capacity(num_arm_joints);
        let mut tree_cursor: BTreeMap<usize, usize> = BTreeMap::new();
        for (arm, (&tree_index, &count)) in arms
            .arm_tree_idx
            .iter()
            .zip(&arms.arm_joint_count)
            .enumerate()
        {
            let tree = trees.get(&tree_index).ok_or(UpdateError::ArmTreeNotFound {
                arm,
                tree: tree_index,
            })?;
            let cursor = tree_cursor.entry(tree_index).or_insert(0);
            let hinges = &tree.hinge_global;
            if *cursor + count > hinges.len() {
                return Err(UpdateError::ArmHingesUnavailable {
                    arm,
                    tree: tree_index,
                    needed: count,
                    available: hinges.len() - *cursor,
                });
            }
            arm_hinge_global.extend_from_slice(&hinges[*cursor..*cursor + count]);
            *cursor += count;
        }
        Ok(ArmLayout {
            trees,
            hinge_to_tree,
            arm_joint_start,
            arm_hinge_global,
        })
    }

    /// Upstream `UpdateState`: the motor torque of each joint, in scene
    /// order.
    fn compute_torques(&mut self) -> Result<DVector<f64>, UpdateError> {
        let mass = self.mass_matrix_in_msg.read();
        let reactions = self.reaction_forces_in_msg.read();
        let layout = match self.layout.take() {
            Some(layout) => layout,
            None => self.arm_layout(&reactions)?,
        };
        let layout = &*self.layout.insert(layout);
        let config = &self.config;
        let arms = &self.arm_config;
        let n = config.num_hinged_joints;

        let joint_angles: Vec<f64> = self
            .joint_states_in_msgs
            .iter()
            .map(|input| input.read().state)
            .collect();
        let thruster_forces: Vec<f64> = self
            .thr_forces_in_msgs
            .iter()
            .map(|input| input.read().input)
            .collect();

        // Joint poses, arm by arm from the hub outward.
        let num_arm_joints = layout.arm_hinge_global.len();
        if [arms.r_cp_p_m.len(), arms.s_hat_p.len(), arms.dcm_c0p.len()]
            .iter()
            .any(|&len| len < num_arm_joints)
        {
            return Err(UpdateError::ArmJointConfigLength);
        }
        let mut poses = Vec::with_capacity(num_arm_joints);
        for (&start, &count) in layout.arm_joint_start.iter().zip(&arms.arm_joint_count) {
            let mut parent = JointPose {
                dcm_cb: Matrix3::identity(),
                r_cb_b: Vector3::zeros(),
            };
            for k in start..start + count {
                let s_hat_p = arms.s_hat_p[k];
                let phi = joint_angles[layout.arm_hinge_global[k]];
                let dcm_cc0 = Matrix3::identity() * phi.cos() - phi.sin() * tilde(s_hat_p)
                    + (1.0 - phi.cos()) * (s_hat_p * s_hat_p.transpose());
                let dcm_cp = dcm_cc0 * arms.dcm_c0p[k];
                let pose = JointPose {
                    dcm_cb: dcm_cp * parent.dcm_cb,
                    r_cb_b: parent.r_cb_b + parent.dcm_cb.transpose() * arms.r_cp_p_m[k],
                };
                poses.push(pose);
                parent = pose;
            }
        }

        // Thruster loads on the hubs and on the joints.
        let num_mounted = arms.thr_arm_idx.len();
        if arms.thr_arm_joint_idx.len() != num_mounted
            || arms.r_tp_p_m.len() < num_mounted
            || arms.f_hat_p.len() < num_mounted
            || thruster_forces.len() < num_mounted
        {
            return Err(UpdateError::ThrusterConfigLength);
        }
        // Per tree: thruster force and torque about the body origin, body
        // components.
        let mut tree_loads: BTreeMap<usize, (Vector3<f64>, Vector3<f64>)> = layout
            .trees
            .keys()
            .map(|&tree| (tree, (Vector3::zeros(), Vector3::zeros())))
            .collect();
        let mut tau_joint = DVector::zeros(n);
        for (thruster, ((&arm, &joint), &thrust_n)) in arms
            .thr_arm_idx
            .iter()
            .zip(&arms.thr_arm_joint_idx)
            .zip(&thruster_forces)
            .enumerate()
        {
            if arm >= arms.arm_joint_count.len() || joint >= arms.arm_joint_count[arm] {
                return Err(UpdateError::ThrusterMountOutOfRange { thruster });
            }
            let start = layout.arm_joint_start[arm];
            let mount = poses[start + joint];
            let f_b = mount.dcm_cb.transpose() * (arms.f_hat_p[thruster] * thrust_n);
            let r_tb_b = mount.r_cb_b + mount.dcm_cb.transpose() * arms.r_tp_p_m[thruster];
            let tree = layout.hinge_to_tree[layout.arm_hinge_global[start + joint]];
            let load = tree_loads
                .get_mut(&tree)
                .expect("every hinge belongs to a tree of the scene");
            load.0 += f_b;
            load.1 += r_tb_b.cross(&f_b);

            for k in start..=start + joint {
                let s_b = poses[k].dcm_cb.transpose() * arms.s_hat_p[k];
                let r_tj_b = r_tb_b - poses[k].r_cb_b;
                tau_joint[layout.arm_hinge_global[k]] += s_b.dot(&r_tj_b.cross(&f_b));
            }
        }

        // Motor torques that keep the joints from accelerating with the hub.
        let forces = non_actuator_forces(&reactions)?;
        let dofs = forces.len();
        check_mass_matrix(&mass.mass_matrix, dofs)?;
        let mut torques = DVector::zeros(n);
        for (tree_index, tree) in &layout.trees {
            let nh = tree.hinge_joints.len();
            if nh == 0 {
                continue;
            }
            let tree_dof_indices = tree_dofs(&reactions, tree, BASE_DOFS, dofs)?;
            let m_full = sub_matrix(&mass.mass_matrix, &tree_dof_indices);
            let m_base = m_full.view((0, 0), (BASE_DOFS, BASE_DOFS)).clone_owned();
            let m_theta_base = m_full.view((BASE_DOFS, 0), (nh, BASE_DOFS));
            let base_bias = sub_vector(&forces, &tree_dof_indices[..BASE_DOFS]);
            let (force_b, torque_b) = tree_loads[tree_index];
            let base_thr =
                DVector::from_iterator(BASE_DOFS, force_b.iter().chain(torque_b.iter()).copied());
            let base_accel = solve_symmetric_vector(&m_base, &(base_thr + base_bias))?;
            let joint_bias = sub_vector(&forces, &tree_dof_indices[BASE_DOFS..]);
            let tau_thr_tree = sub_vector(&tau_joint, &tree.hinge_global);
            let u_tree = m_theta_base * base_accel - joint_bias - tau_thr_tree;
            for (&j, u) in tree.hinge_global.iter().zip(u_tree.iter()) {
                torques[j] = *u;
            }
        }

        if let Some(u_max) = &config.u_max_nm {
            if u_max.len() != n {
                return Err(UpdateError::TorqueLimitLength);
            }
            saturate(&mut torques, u_max);
        }
        Ok(torques)
    }
}

impl Module for ThrJointCompensation {
    fn init(&mut self) {
        for output in &self.motor_torques_out_msgs {
            output.write(SingleActuatorMsg::default());
        }
        self.reset_module();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.compute_torques() {
            Ok(torques) => {
                for (output, &input) in self.motor_torques_out_msgs.iter().zip(torques.iter()) {
                    output.write(SingleActuatorMsg { input });
                }
            }
            // Upstream throws before writing: the outputs keep their previous
            // values.
            Err(error) => log::error!("{error}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix3, Vector3};

    use super::{ThrJointCompensation, ThrJointCompensationConfig, UpdateError};
    use crate::fsw_algorithms::effector_interfaces::joint_trees::test_support::{
        assert_allclose, scene_mass_matrix, scene_reactions,
    };
    use crate::messages::{
        MjJointReactionsMsg, MjSystemMassMatrixMsg, Output, ScalarJointStateMsg, SingleActuatorMsg,
        ThrusterArmConfigMsg,
    };
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's joint angles, the same on every arm.
    fn joint_angles(num_joints: usize) -> &'static [f64] {
        if num_joints == 1 {
            &[0.1]
        } else {
            &[0.1, -0.5, 0.3, 0.7]
        }
    }

    /// The pytest's arm configuration: on every arm one thruster at the last
    /// joint, 0.1 m out along x and firing along -x; the joints spin about
    /// x, y, z, x and sit at 0.5 m, 0, 1 m, 0 along x; the first joint of a
    /// second arm is turned half a revolution about z.
    fn upstream_arm_config(
        num_spacecraft: usize,
        num_arms: usize,
        num_joints: usize,
    ) -> ThrusterArmConfigMsg {
        let axis_pattern = [Vector3::x(), Vector3::y(), Vector3::z(), Vector3::x()];
        let link_pattern = [
            Vector3::new(0.5, 0.0, 0.0),
            Vector3::zeros(),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::zeros(),
        ];
        let mut config = ThrusterArmConfigMsg::default();
        for sc in 0..num_spacecraft {
            for arm in 0..num_arms {
                config.thr_arm_idx.push(sc * num_arms + arm);
                config.thr_arm_joint_idx.push(num_joints - 1);
                config.arm_tree_idx.push(sc);
                config.arm_joint_count.push(num_joints);
                config.r_tp_p_m.push(Vector3::new(0.1, 0.0, 0.0));
                config.f_hat_p.push(Vector3::new(-1.0, 0.0, 0.0));
                for j in 0..num_joints {
                    config.r_cp_p_m.push(link_pattern[j]);
                    config.s_hat_p.push(axis_pattern[j]);
                    config.dcm_c0p.push(if arm == 1 && j == 0 {
                        Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0))
                    } else {
                        Matrix3::identity()
                    });
                }
            }
        }
        config
    }

    fn skew(v: Vector3<f64>) -> Matrix3<f64> {
        Matrix3::new(0.0, -v[2], v[1], v[2], 0.0, -v[0], -v[1], v[0], 0.0)
    }

    /// The pytest's `PRV2C`.
    fn prv2c(ehat: Vector3<f64>, phi: f64) -> Matrix3<f64> {
        Matrix3::identity() * phi.cos() - phi.sin() * skew(ehat)
            + (1.0 - phi.cos()) * (ehat * ehat.transpose())
    }

    /// The pytest's `jointTorques`, with NumPy's LU solve.
    #[allow(clippy::too_many_arguments)]
    fn upstream_truth(
        n_sc: usize,
        n_arms: usize,
        n_joints: usize,
        m: &DMatrix<f64>,
        theta: &[f64],
        bias: &[f64],
        thrust: &[f64],
        arm_cfg: &ThrusterArmConfigMsg,
        max_torque: Option<&[f64]>,
    ) -> Vec<f64> {
        let bias = DVector::from_iterator(bias.len(), bias.iter().map(|b| -b));
        let n_h = n_sc * n_arms * n_joints;
        let n_base = 6;
        let mut dcm_cb = vec![Matrix3::zeros(); n_h];
        let mut r_cb_b = vec![Vector3::zeros(); n_h];
        for sc in 0..n_sc {
            for arm in 0..n_arms {
                let mut dcm_pb = Matrix3::identity();
                let mut r_pb_b = Vector3::zeros();
                for (j, &phi) in theta.iter().enumerate().take(n_joints) {
                    let h = sc * (n_arms * n_joints) + arm * n_joints + j;
                    let c_cp = prv2c(arm_cfg.s_hat_p[h], phi) * arm_cfg.dcm_c0p[h];
                    r_cb_b[h] = r_pb_b + dcm_pb.transpose() * arm_cfg.r_cp_p_m[h];
                    dcm_cb[h] = c_cp * dcm_pb;
                    r_pb_b = r_cb_b[h];
                    dcm_pb = dcm_cb[h];
                }
            }
        }

        let mut f_base = vec![Vector3::zeros(); n_sc];
        let mut l_base = vec![Vector3::zeros(); n_sc];
        let mut tau_joint = vec![0.0; n_h];
        for sc in 0..n_sc {
            for arm in 0..n_arms {
                let thr_idx = sc * n_arms + arm;
                let j_local = n_joints - 1;
                let h_attach = sc * (n_arms * n_joints) + arm * n_joints + j_local;
                let c_cb = dcm_cb[h_attach];
                let f_b = c_cb.transpose() * (arm_cfg.f_hat_p[thr_idx] * thrust[thr_idx]);
                let r_tb_b = r_cb_b[h_attach] + c_cb.transpose() * arm_cfg.r_tp_p_m[thr_idx];
                f_base[sc] += f_b;
                l_base[sc] += r_tb_b.cross(&f_b);
                for j in 0..=j_local {
                    let h_j = sc * (n_arms * n_joints) + arm * n_joints + j;
                    let shat_b = dcm_cb[h_j].transpose() * arm_cfg.s_hat_p[h_j];
                    let r_tj_b = r_tb_b - r_cb_b[h_j];
                    tau_joint[h_j] += shat_b.dot(&r_tj_b.cross(&f_b));
                }
            }
        }

        let mut u_h = vec![0.0; n_h];
        let per_sc = n_arms * n_joints;
        for sc in 0..n_sc {
            let sc0 = sc * (n_base + per_sc);
            let size = n_base + per_sc;
            let m_sc = m.view((sc0, sc0), (size, size));
            let m_base = m_sc.view((0, 0), (n_base, n_base)).clone_owned();
            let m_thb = m_sc.view((n_base, 0), (per_sc, n_base));
            let mut base_thr = DVector::zeros(n_base);
            base_thr.rows_mut(0, 3).copy_from(&f_base[sc]);
            base_thr.rows_mut(3, 3).copy_from(&l_base[sc]);
            let base_bias = bias.rows(sc0, n_base);
            let base_accel = m_base
                .lu()
                .solve(&(base_thr + base_bias))
                .expect("a regular base mass matrix");
            let joint_bias = bias.rows(sc0 + n_base, per_sc);
            let tau_thr_sc = DVector::from_column_slice(&tau_joint[sc * per_sc..(sc + 1) * per_sc]);
            let u_sc = m_thb * base_accel - joint_bias - tau_thr_sc;
            u_h[sc * per_sc..(sc + 1) * per_sc].copy_from_slice(u_sc.as_slice());
        }
        if let Some(max_torque) = max_torque {
            for (u, limit) in u_h.iter_mut().zip(max_torque) {
                *u = u.clamp(-limit, *limit);
            }
        }
        u_h
    }

    struct Fixture {
        module: ThrJointCompensation,
        arm_config_out: Output<ThrusterArmConfigMsg>,
        mass_out: Output<MjSystemMassMatrixMsg>,
        _reactions_out: Output<MjJointReactionsMsg>,
        thrust_outs: Vec<Output<SingleActuatorMsg>>,
        _angle_outs: Vec<Output<ScalarJointStateMsg>>,
        bias_forces: Vec<f64>,
        thrust: Vec<f64>,
    }

    /// The pytest's set-up.
    fn fixture(
        non_act_forces: bool,
        max_torque: bool,
        num_joints: usize,
        num_arms: usize,
        num_spacecraft: usize,
    ) -> Fixture {
        let n = num_joints * num_arms * num_spacecraft;
        let num_thrusters = num_arms * num_spacecraft;
        let mut module = ThrJointCompensation::new(ThrJointCompensationConfig {
            num_hinged_joints: n,
            num_thrusters,
            u_max_nm: max_torque.then(|| vec![1e-6; n]),
            ..Default::default()
        });
        for _ in 0..n {
            module.add_motor_torque_output();
        }
        let dofs = (6 + num_joints * num_arms) * num_spacecraft;
        let bias_forces: Vec<f64> = if non_act_forces {
            (1..=dofs).map(|k| -0.2 * k as f64).collect()
        } else {
            vec![0.0; dofs]
        };
        let thrust: Vec<f64> = (0..num_thrusters).map(|k| 0.25 + 0.1 * k as f64).collect();

        let arm_config_out = Output::new(upstream_arm_config(num_spacecraft, num_arms, num_joints));
        let mass_out = Output::new(MjSystemMassMatrixMsg {
            mass_matrix: scene_mass_matrix(num_spacecraft, num_joints * num_arms),
            ..Default::default()
        });
        let reactions_out = Output::new(scene_reactions(
            num_spacecraft,
            num_joints * num_arms,
            bias_forces.clone(),
        ));
        arm_config_out.connect_to(&mut module.arm_config_in_msg);
        mass_out.connect_to(&mut module.mass_matrix_in_msg);
        reactions_out.connect_to(&mut module.reaction_forces_in_msg);
        let thrust_outs: Vec<_> = thrust
            .iter()
            .map(|&input| Output::new(SingleActuatorMsg { input }))
            .collect();
        for (output, input) in thrust_outs.iter().zip(&mut module.thr_forces_in_msgs) {
            output.connect_to(input);
        }
        let angle_outs: Vec<_> = (0..n)
            .map(|i| {
                Output::new(ScalarJointStateMsg {
                    state: joint_angles(num_joints)[i % num_joints],
                })
            })
            .collect();
        for (output, input) in angle_outs.iter().zip(&mut module.joint_states_in_msgs) {
            output.connect_to(input);
        }
        module.init();
        Fixture {
            module,
            arm_config_out,
            mass_out,
            _reactions_out: reactions_out,
            thrust_outs,
            _angle_outs: angle_outs,
            bias_forces,
            thrust,
        }
    }

    fn torques(module: &ThrJointCompensation) -> Vec<f64> {
        module
            .motor_torques_out_msgs
            .iter()
            .map(|output| output.read().input)
            .collect()
    }

    /// Upstream `test_thrJointCompensation`, all thirty-two `(nonActForces,
    /// maxTorque, numJoints, numArms, numSpacecraft)` parametrizations: a
    /// 0.1 s task run to 0.1 s, the motor torques checked against the truth
    /// with `atol = 1e-8`. The mass matrix is drawn from a seeded generator,
    /// not NumPy's `default_rng(12345)` (see `scene_mass_matrix`).
    #[test]
    fn matches_upstream_thr_joint_compensation_unit_test() {
        for non_act_forces in [false, true] {
            for max_torque in [false, true] {
                for num_joints in [1, 4] {
                    for num_arms in [1, 2] {
                        for num_spacecraft in [1, 2] {
                            let mut fixture = fixture(
                                non_act_forces,
                                max_torque,
                                num_joints,
                                num_arms,
                                num_spacecraft,
                            );
                            for nanos in [0, 100_000_000] {
                                fixture.module.update(&context(nanos));
                            }
                            let expected = upstream_truth(
                                num_spacecraft,
                                num_arms,
                                num_joints,
                                &fixture.mass_out.read().mass_matrix,
                                joint_angles(num_joints),
                                &fixture.bias_forces,
                                &fixture.thrust,
                                &fixture.arm_config_out.read(),
                                fixture.module.config.u_max_nm.as_deref(),
                            );
                            assert_allclose(
                                &torques(&fixture.module),
                                &expected,
                                0.0,
                                1e-8,
                                &format!(
                                    "nonAct {non_act_forces}, max {max_torque}, {num_joints} \
                                     joints, {num_arms} arms, {num_spacecraft} spacecraft"
                                ),
                            );
                        }
                    }
                }
            }
        }
    }

    /// Not upstream: with the thrusters off and no generalized forces the
    /// joints need no torque.
    #[test]
    fn idle_thrusters_need_no_torque() {
        let mut fixture = fixture(false, false, 4, 2, 2);
        for output in &fixture.thrust_outs {
            output.write(SingleActuatorMsg::default());
        }
        fixture.module.update(&context(0));
        assert!(torques(&fixture.module).iter().all(|&u| u == 0.0));
    }

    #[test]
    fn rejects_arms_that_do_not_fit_the_scene() {
        let mut fixture = fixture(false, false, 1, 2, 1);
        fixture.module.update(&context(0));
        let before = torques(&fixture.module);

        // Three joints on an arm of a tree with two hinges.
        let mut arm_config = upstream_arm_config(1, 2, 1);
        arm_config.arm_joint_count[1] = 2;
        fixture.arm_config_out.write(arm_config);
        fixture.module.reset(&context(0));
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::ArmHingesUnavailable {
                arm: 1,
                tree: 0,
                needed: 2,
                available: 1
            })
        );
        fixture.module.update(&context(0));
        assert_eq!(torques(&fixture.module), before);

        let mut arm_config = upstream_arm_config(1, 2, 1);
        arm_config.arm_tree_idx[0] = 3;
        fixture.arm_config_out.write(arm_config);
        fixture.module.reset(&context(0));
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::ArmTreeNotFound { arm: 0, tree: 3 })
        );

        let mut arm_config = upstream_arm_config(1, 2, 1);
        arm_config.thr_arm_joint_idx[1] = 1;
        fixture.arm_config_out.write(arm_config);
        fixture.module.reset(&context(0));
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::ThrusterMountOutOfRange { thruster: 1 })
        );

        let mut arm_config = upstream_arm_config(1, 2, 1);
        arm_config.arm_joint_count.pop();
        fixture.arm_config_out.write(arm_config);
        fixture.module.reset(&context(0));
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::ArmListLengths)
        );
    }

    #[test]
    fn makes_one_input_per_joint_and_per_thruster() {
        let module = ThrJointCompensation::new(ThrJointCompensationConfig {
            num_hinged_joints: 6,
            num_thrusters: 2,
            ..Default::default()
        });
        assert_eq!(module.joint_states_in_msgs.len(), 6);
        assert_eq!(module.thr_forces_in_msgs.len(), 2);
        assert!(module.motor_torques_out_msgs.is_empty());
    }
}
