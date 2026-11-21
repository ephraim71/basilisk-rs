//! Hub torques that cancel the reaction to moving joints: port of upstream
//! Basilisk `fswAlgorithms/effectorInterfaces/jointMotionCompensator`.
//!
//! Every spacecraft of the multibody scene is a six-degree-of-freedom hub
//! carrying hinged joints. Given the joint motor torques `u` (for example
//! from [`super::hinged_joint_array_motor`]), the mass matrix and the
//! non-actuator generalized forces `τ` (passive + constraint + applied -
//! bias), the module solves the hub-translation and joint rows of the
//! equations of motion with the hub rotation held, by block elimination,
//!
//! ```text
//! θ̈ = ([M_θθ] - [M_θt] [M_tt]^-1 [M_tθ])^-1 (τ_θ + u - [M_θt] [M_tt]^-1 τ_t)
//! r̈ = [M_tt]^-1 (τ_t - [M_tθ] θ̈)
//! ```
//!
//! and publishes the hub torque that keeps the hub from rotating,
//!
//! ```text
//! L = -τ_ω + [M_ωt] r̈ + [M_ωθ] θ̈
//! ```
//!
//! optionally clamped to `±u_max` per axis.
//!
//! The module has one joint-torque input per hinged joint (scene order),
//! made by [`JointMotionCompensator::new`] from
//! [`JointMotionCompensatorConfig::num_hinged_joints`], and three hub-torque
//! outputs per spacecraft, one per hub axis, registered with
//! [`JointMotionCompensator::add_hub_torque_output`] (upstream's
//! `addSpacecraft` adds three, `addHingedJoint` one input). The scene
//! messages come from a multibody (MuJoCo) simulation.

use std::fmt;

use nalgebra::DVector;

use super::joint_trees::{
    KinematicTrees, SceneError, check_mass_matrix, kinematic_trees, non_actuator_forces, saturate,
    solve_symmetric, solve_symmetric_vector, sub_matrix, sub_vector, tree_dofs,
};
use crate::messages::{
    Input, MjJointReactionsMsg, MjSystemMassMatrixMsg, Output, SingleActuatorMsg,
};
use crate::{Module, SimulationContext};

/// Degrees of freedom of a free joint: hub translation, then rotation.
const BASE_DOFS: usize = 6;

/// User-settable parameters of [`JointMotionCompensator`].
#[derive(Clone, Debug)]
pub struct JointMotionCompensatorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `numSpacecraft`: number of spacecraft (kinematic trees) in
    /// the scene.
    pub num_spacecraft: usize,
    /// Number of hinged joints in the scene (upstream calls
    /// `addHingedJoint` once per joint); [`JointMotionCompensator::new`]
    /// makes the joint-torque inputs from it.
    pub num_hinged_joints: usize,
    /// Upstream `uMax` \[N m\]: optional limit of each hub torque, three per
    /// spacecraft (upstream leaves the vector empty for none); must not be
    /// negative.
    pub u_max_nm: Option<Vec<f64>>,
}

impl Default for JointMotionCompensatorConfig {
    fn default() -> Self {
        Self {
            name: "jointMotionCompensator".to_string(),
            num_spacecraft: 0,
            num_hinged_joints: 0,
            u_max_nm: None,
        }
    }
}

/// Why an update publishes nothing. Upstream raises a `BasiliskError` (or
/// reads out of bounds, for the shape errors) before writing the outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UpdateError {
    Scene(SceneError),
    /// The scene has a different number of kinematic trees than
    /// `num_spacecraft`.
    SpacecraftCountMismatch {
        configured: usize,
        found: usize,
    },
    /// A tree index is not below `num_spacecraft`.
    TreeOutOfRange {
        tree: usize,
    },
    /// `u_max_nm` does not hold three limits per spacecraft.
    TorqueLimitLength,
}

impl From<SceneError> for UpdateError {
    fn from(error: SceneError) -> Self {
        Self::Scene(error)
    }
}

impl fmt::Display for UpdateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Scene(error) => write!(f, "jointMotionCompensator: {error}"),
            Self::SpacecraftCountMismatch { configured, found } => write!(
                f,
                "jointMotionCompensator: num_spacecraft ({configured}) does not match the number \
                 of kinematic trees found in the system ({found})"
            ),
            Self::TreeOutOfRange { tree } => write!(
                f,
                "jointMotionCompensator: kinematic tree {tree} is not a spacecraft index"
            ),
            Self::TorqueLimitLength => write!(
                f,
                "jointMotionCompensator: size of u_max_nm does not match 3 x number of spacecraft"
            ),
        }
    }
}

/// Hub torques that negate the joint reaction torques: upstream
/// `JointMotionCompensator`.
#[derive(Clone, Debug)]
pub struct JointMotionCompensator {
    /// Module configuration.
    pub config: JointMotionCompensatorConfig,
    /// Upstream `massMatrixInMsg`: scene mass matrix.
    pub mass_matrix_in_msg: Input<MjSystemMassMatrixMsg>,
    /// Upstream `reactionForcesInMsg`: scene joints and generalized forces.
    pub reaction_forces_in_msg: Input<MjJointReactionsMsg>,
    /// Upstream `jointTorqueInMsgs`: motor torque \[N m\] of each hinged
    /// joint, one input per joint in scene order.
    pub joint_torque_in_msgs: Vec<Input<SingleActuatorMsg>>,
    /// Upstream `hubTorqueOutMsgs`: hub torque \[N m\] about each hub axis,
    /// three outputs per spacecraft (x, y, z of spacecraft 0, then of
    /// spacecraft 1, ...), registered with
    /// [`JointMotionCompensator::add_hub_torque_output`].
    pub hub_torque_out_msgs: Vec<Output<SingleActuatorMsg>>,
    /// Upstream `treeMap`, found on the first update after a reset
    /// (upstream `treeInfoInitialized`).
    trees: Option<KinematicTrees>,
}

impl JointMotionCompensator {
    /// Builds the module with one unconnected joint-torque input per
    /// configured joint and no outputs.
    pub fn new(config: JointMotionCompensatorConfig) -> Self {
        // Not `vec![_; n]`: cloning a port aliases it.
        let joint_torque_in_msgs = (0..config.num_hinged_joints)
            .map(|_| Input::default())
            .collect();
        Self {
            config,
            mass_matrix_in_msg: Input::default(),
            reaction_forces_in_msg: Input::default(),
            joint_torque_in_msgs,
            hub_torque_out_msgs: Vec::new(),
            trees: None,
        }
    }

    /// Registers one independently connectable hub-torque output, for the
    /// next hub axis in spacecraft order.
    pub fn add_hub_torque_output(&mut self) -> &Output<SingleActuatorMsg> {
        self.hub_torque_out_msgs.push(Output::default());
        self.hub_torque_out_msgs
            .last()
            .expect("registered hub torque output should be available")
    }

    /// Upstream `Reset` (and the check of upstream's `setUMax`): reports
    /// missing inputs and negative limits, and forgets the kinematic trees.
    ///
    /// # Panics
    ///
    /// If the number of joint inputs no longer matches
    /// `config.num_hinged_joints`: the inputs are made from it in `new`.
    fn reset_module(&mut self) {
        assert_eq!(
            self.joint_torque_in_msgs.len(),
            self.config.num_hinged_joints,
            "jointMotionCompensator needs one joint_torque_in_msgs input per \
             config.num_hinged_joints joint"
        );
        for (connected, port) in [
            (self.mass_matrix_in_msg.is_connected(), "mass_matrix_in_msg"),
            (
                self.reaction_forces_in_msg.is_connected(),
                "reaction_forces_in_msg",
            ),
        ] {
            if !connected {
                log::error!("jointMotionCompensator.{port} is not connected");
            }
        }
        if self.joint_torque_in_msgs.is_empty() {
            log::error!("jointMotionCompensator has no hinged joints");
        }
        for (index, input) in self.joint_torque_in_msgs.iter().enumerate() {
            if !input.is_connected() {
                log::error!(
                    "jointMotionCompensator.joint_torque_in_msgs[{index}] is not connected"
                );
            }
        }
        if self.hub_torque_out_msgs.len() != 3 * self.config.num_spacecraft {
            log::error!(
                "jointMotionCompensator has {} spacecraft but {} hub torque outputs",
                self.config.num_spacecraft,
                self.hub_torque_out_msgs.len()
            );
        }
        for (index, limit) in self.config.u_max_nm.iter().flatten().enumerate() {
            if *limit < 0.0 {
                log::error!("jointMotionCompensator: u_max_nm[{index}] is negative");
            }
        }
        self.trees = None;
    }

    /// Upstream `UpdateState`: the hub torques, three per spacecraft.
    fn compute_hub_torques(&mut self) -> Result<DVector<f64>, UpdateError> {
        let config = &self.config;
        let mass = self.mass_matrix_in_msg.read();
        let reactions = self.reaction_forces_in_msg.read();

        let trees = match self.trees.take() {
            Some(trees) => trees,
            None => {
                let trees = kinematic_trees(&reactions, config.num_hinged_joints)?;
                if trees.len() != config.num_spacecraft {
                    return Err(UpdateError::SpacecraftCountMismatch {
                        configured: config.num_spacecraft,
                        found: trees.len(),
                    });
                }
                trees
            }
        };
        let trees = self.trees.insert(trees);

        let joint_torques = DVector::from_iterator(
            config.num_hinged_joints,
            self.joint_torque_in_msgs
                .iter()
                .map(|input| input.read().input),
        );
        let forces = non_actuator_forces(&reactions)?;
        let dofs = forces.len();
        check_mass_matrix(&mass.mass_matrix, dofs)?;

        let mut hub_torques = DVector::zeros(3 * config.num_spacecraft);
        for (&tree_index, tree) in trees.iter() {
            let nh = tree.hinge_joints.len();
            if nh == 0 {
                continue;
            }
            if tree_index >= config.num_spacecraft {
                return Err(UpdateError::TreeOutOfRange { tree: tree_index });
            }
            let tree_dof_indices = tree_dofs(&reactions, tree, BASE_DOFS, dofs)?;
            let m_full = sub_matrix(&mass.mass_matrix, &tree_dof_indices);
            let m_tt = m_full.view((0, 0), (3, 3)).clone_owned();
            let m_rt = m_full.view((3, 0), (3, 3));
            let m_t_theta = m_full.view((0, 6), (3, nh)).clone_owned();
            let m_r_theta = m_full.view((3, 6), (3, nh));
            let m_theta_t = m_full.view((6, 0), (nh, 3));
            let m_theta_theta = m_full.view((6, 6), (nh, nh));

            let base_trans_bias = sub_vector(&forces, &tree_dof_indices[..3]);
            let base_rot_bias = sub_vector(&forces, &tree_dof_indices[3..6]);
            let joint_bias = sub_vector(&forces, &tree_dof_indices[6..]);
            let tree_motor_torque = sub_vector(&joint_torques, &tree.hinge_global);

            let temp1 = solve_symmetric_vector(&m_tt, &base_trans_bias)?;
            let temp2 = solve_symmetric(&m_tt, &m_t_theta)?;
            let temp3 = m_theta_theta - m_theta_t * &temp2;
            let theta_ddot = solve_symmetric_vector(
                &temp3,
                &(joint_bias + tree_motor_torque - m_theta_t * &temp1),
            )?;
            let r_ddot = temp1 - &temp2 * &theta_ddot;
            let tree_hub_torque = -base_rot_bias + m_rt * r_ddot + m_r_theta * theta_ddot;
            hub_torques
                .rows_mut(3 * tree_index, 3)
                .copy_from(&tree_hub_torque);
        }

        if let Some(u_max) = &config.u_max_nm {
            if u_max.len() != 3 * config.num_spacecraft {
                return Err(UpdateError::TorqueLimitLength);
            }
            saturate(&mut hub_torques, u_max);
        }
        Ok(hub_torques)
    }
}

impl Module for JointMotionCompensator {
    fn init(&mut self) {
        for output in &self.hub_torque_out_msgs {
            output.write(SingleActuatorMsg::default());
        }
        self.reset_module();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.compute_hub_torques() {
            Ok(torques) => {
                for (output, &input) in self.hub_torque_out_msgs.iter().zip(torques.iter()) {
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
    use nalgebra::{DMatrix, DVector};

    use super::{JointMotionCompensator, JointMotionCompensatorConfig, UpdateError};
    use crate::fsw_algorithms::effector_interfaces::joint_trees::test_support::{
        assert_allclose, scene_mass_matrix, scene_reactions,
    };
    use crate::messages::{MjJointReactionsMsg, MjSystemMassMatrixMsg, Output, SingleActuatorMsg};
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's `compensationTorque` for one spacecraft, with NumPy's
    /// LU solves.
    fn upstream_truth(
        m: &DMatrix<f64>,
        non_act_force: &DVector<f64>,
        joint_torque: &[f64],
        max_torque: Option<&[f64]>,
    ) -> Vec<f64> {
        let nj = joint_torque.len();
        let base_trans_bias = non_act_force.rows(0, 3);
        let base_rot_bias = non_act_force.rows(3, 3);
        let joint_bias = non_act_force.rows(6, nj);
        let u_h = DVector::from_column_slice(joint_torque);
        let mtt = m.view((0, 0), (3, 3)).clone_owned();
        let mrt = m.view((3, 0), (3, 3));
        let mrth = m.view((3, 6), (3, nj));
        let mtth = m.view((0, 6), (3, nj));
        let mtht = m.view((6, 0), (nj, 3));
        let mthth = m.view((6, 6), (nj, nj));
        let lu = mtt.lu();
        let temp1 = lu.solve(&base_trans_bias.clone_owned()).unwrap();
        let temp2 = lu.solve(&mtth.clone_owned()).unwrap();
        let temp3 = mthth - mtht * &temp2;
        let theta_ddot = temp3
            .lu()
            .solve(&(joint_bias + u_h - mtht * &temp1))
            .unwrap();
        let r_ddot = temp1 - &temp2 * &theta_ddot;
        let mut tau = -base_rot_bias + mrt * r_ddot + mrth * theta_ddot;
        if let Some(max_torque) = max_torque {
            for i in 0..3 {
                if tau[i] > max_torque[i] {
                    tau[i] = max_torque[i];
                } else if tau[i] < -max_torque[i] {
                    tau[i] = -max_torque[i];
                }
            }
        }
        tau.as_slice().to_vec()
    }

    struct Fixture {
        module: JointMotionCompensator,
        mass_out: Output<MjSystemMassMatrixMsg>,
        reactions_out: Output<MjJointReactionsMsg>,
        _torque_outs: Vec<Output<SingleActuatorMsg>>,
    }

    fn motor_torques(num_joints: usize) -> &'static [f64] {
        if num_joints == 1 {
            &[1.0]
        } else {
            &[1.0, 0.5, -0.5, -1.0]
        }
    }

    /// The pytest's set-up for `num_joints` joints on each of
    /// `num_spacecraft` spacecraft.
    fn fixture(
        non_act_forces: bool,
        max_torque: bool,
        num_joints: usize,
        num_spacecraft: usize,
    ) -> (Fixture, Vec<f64>) {
        let n = num_joints * num_spacecraft;
        let mut module = JointMotionCompensator::new(JointMotionCompensatorConfig {
            num_spacecraft,
            num_hinged_joints: n,
            u_max_nm: max_torque.then(|| vec![0.03; 3 * num_spacecraft]),
            ..Default::default()
        });
        for _ in 0..3 * num_spacecraft {
            module.add_hub_torque_output();
        }
        let dofs = (6 + num_joints) * num_spacecraft;
        let bias_forces = vec![if non_act_forces { -2.0 } else { 0.0 }; dofs];
        let mass_out = Output::new(MjSystemMassMatrixMsg {
            mass_matrix: scene_mass_matrix(num_spacecraft, num_joints),
            ..Default::default()
        });
        let reactions_out = Output::new(scene_reactions(
            num_spacecraft,
            num_joints,
            bias_forces.clone(),
        ));
        mass_out.connect_to(&mut module.mass_matrix_in_msg);
        reactions_out.connect_to(&mut module.reaction_forces_in_msg);
        let torque_outs: Vec<_> = (0..n)
            .map(|i| {
                Output::new(SingleActuatorMsg {
                    input: motor_torques(num_joints)[i % num_joints],
                })
            })
            .collect();
        for (output, input) in torque_outs.iter().zip(&mut module.joint_torque_in_msgs) {
            output.connect_to(input);
        }
        module.init();
        (
            Fixture {
                module,
                mass_out,
                reactions_out,
                _torque_outs: torque_outs,
            },
            bias_forces,
        )
    }

    fn hub_torques(module: &JointMotionCompensator) -> Vec<f64> {
        module
            .hub_torque_out_msgs
            .iter()
            .map(|output| output.read().input)
            .collect()
    }

    /// Upstream `test_jointMotionCompensator`, all sixteen `(nonActForces,
    /// maxTorque, numJoints, numSpacecraft)` parametrizations: a 0.1 s task
    /// run to 0.1 s, the hub torques checked against the truth with
    /// `rtol = 1e-8`. The mass matrix is drawn from a seeded generator, not
    /// NumPy's `default_rng(12345)` (see `scene_mass_matrix`).
    #[test]
    fn matches_upstream_joint_motion_compensator_unit_test() {
        for non_act_forces in [false, true] {
            for max_torque in [false, true] {
                for num_joints in [1, 4] {
                    for num_spacecraft in [1, 2] {
                        let (mut fixture, bias_forces) =
                            fixture(non_act_forces, max_torque, num_joints, num_spacecraft);
                        for nanos in [0, 100_000_000] {
                            fixture.module.update(&context(nanos));
                        }
                        let actual = hub_torques(&fixture.module);

                        let mass = fixture.mass_out.read().mass_matrix;
                        let reaction_forces = DVector::from_vec(bias_forces);
                        let mut expected = Vec::new();
                        for sc in 0..num_spacecraft {
                            let start = sc * (6 + num_joints);
                            let size = 6 + num_joints;
                            let m_sc = mass.view((start, start), (size, size)).clone_owned();
                            let forces_sc = -reaction_forces.rows(start, size).clone_owned();
                            expected.extend(upstream_truth(
                                &m_sc,
                                &forces_sc,
                                motor_torques(num_joints),
                                max_torque.then_some(&[0.03, 0.03, 0.03][..]),
                            ));
                        }
                        assert_allclose(
                            &actual,
                            &expected,
                            1e-8,
                            0.0,
                            &format!(
                                "nonAct {non_act_forces}, max {max_torque}, {num_joints} joints, \
                                 {num_spacecraft} spacecraft"
                            ),
                        );
                    }
                }
            }
        }
    }

    /// Not upstream: with no joint torque and no generalized forces nothing
    /// moves, so no hub torque is needed.
    #[test]
    fn needs_no_hub_torque_for_idle_joints() {
        let (mut fixture, _) = fixture(false, false, 4, 2);
        for input in &mut fixture.module.joint_torque_in_msgs {
            Output::new(SingleActuatorMsg::default()).connect_to(input);
        }
        fixture.module.update(&context(0));
        assert!(hub_torques(&fixture.module).iter().all(|&t| t == 0.0));
    }

    #[test]
    fn a_scene_that_does_not_match_stops_the_update() {
        let (mut fixture, _) = fixture(true, false, 1, 2);
        fixture.module.update(&context(0));
        let before = hub_torques(&fixture.module);

        // Two hinges, but on a single spacecraft.
        fixture
            .reactions_out
            .write(scene_reactions(1, 2, vec![0.0; 8]));
        fixture.module.reset(&context(0));
        assert_eq!(
            fixture.module.compute_hub_torques(),
            Err(UpdateError::SpacecraftCountMismatch {
                configured: 2,
                found: 1
            })
        );
        fixture.module.update(&context(0));
        assert_eq!(hub_torques(&fixture.module), before);

        fixture
            .reactions_out
            .write(scene_reactions(2, 1, vec![0.0; 14]));
        fixture.module.config.u_max_nm = Some(vec![1.0; 3]);
        assert_eq!(
            fixture.module.compute_hub_torques(),
            Err(UpdateError::TorqueLimitLength)
        );
    }

    #[test]
    fn makes_one_joint_torque_input_per_joint() {
        let module = JointMotionCompensator::new(JointMotionCompensatorConfig {
            num_spacecraft: 2,
            num_hinged_joints: 5,
            ..Default::default()
        });
        assert_eq!(module.joint_torque_in_msgs.len(), 5);
        assert!(module.hub_torque_out_msgs.is_empty());
    }
}
