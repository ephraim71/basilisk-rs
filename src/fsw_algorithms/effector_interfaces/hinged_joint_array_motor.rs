//! Tracking control of an array of hinged joints on free-floating hubs:
//! port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/hingedJointArrayMotor`.
//!
//! Every spacecraft of the multibody scene is a six-degree-of-freedom hub
//! carrying hinged joints. For each spacecraft, with joint angles `θ`, the
//! desired joint acceleration is a PD tracking law on the desired joint
//! states,
//!
//! ```text
//! θ̈_d = θ̈* - [K_θ] e - [P_θ] ė,    e = wrap(θ - θ*),  ė = θ̇ - θ̇*
//! ```
//!
//! and the joint motor torques realize it given the mass matrix and the
//! non-actuator generalized forces `τ` (passive + constraint + applied -
//! bias), eliminating the hub translation `r̈` through its block of the
//! equations of motion:
//!
//! ```text
//! λ = [M_tt]^-1 (τ_t - [M_tθ] θ̈_d)
//! u = [M_θθ] θ̈_d + [M_θt] λ - τ_θ
//! ```
//!
//! The hub rotation is left out: the reaction torques on the hub from the
//! joint motion are assumed to be cancelled (see
//! [`super::joint_motion_compensator`]). Torques are optionally clamped to
//! `±u_max`.
//!
//! The joints are numbered across the scene in joint order; the module has
//! one joint-angle and one joint-rate input per joint, made by
//! [`HingedJointArrayMotor::new`] from
//! [`HingedJointArrayMotorConfig::num_hinged_joints`], and one motor-torque
//! output per joint, registered with
//! [`HingedJointArrayMotor::add_motor_torque_output`] (upstream's
//! `addHingedJoint` adds all three). The scene messages come from a
//! multibody (MuJoCo) simulation.

use std::fmt;

use nalgebra::{DMatrix, DVector};

use super::joint_trees::{
    KinematicTrees, SceneError, check_mass_matrix, kinematic_trees, non_actuator_forces, saturate,
    solve_symmetric_vector, sub_matrix, sub_vector, tree_dofs,
};
use crate::messages::{
    Input, JointArrayStateMsg, MjJointReactionsMsg, MjSystemMassMatrixMsg, Output,
    ScalarJointStateMsg, SingleActuatorMsg,
};
use crate::{Module, SimulationContext};

/// Hub translational degrees of freedom of a free joint.
const TRANSLATION_DOFS: usize = 3;

/// User-settable parameters of [`HingedJointArrayMotor`].
#[derive(Clone, Debug)]
pub struct HingedJointArrayMotorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Number of hinged joints in the scene (upstream calls
    /// `addHingedJoint` once per joint); [`HingedJointArrayMotor::new`]
    /// makes the joint inputs from it.
    pub num_hinged_joints: usize,
    /// Upstream `Ktheta` \[1/s^2\]: angle-error gain, `num_hinged_joints`
    /// square. Upstream's `setKtheta` takes it row-major; build it from that
    /// vector with [`DMatrix::from_row_slice`].
    pub k_theta: DMatrix<f64>,
    /// Upstream `Ptheta` \[1/s\]: rate-error gain, `num_hinged_joints`
    /// square.
    pub p_theta: DMatrix<f64>,
    /// Upstream `uMax` \[N m\]: optional torque limit of each joint (upstream
    /// leaves the vector empty for none).
    pub u_max_nm: Option<Vec<f64>>,
}

impl Default for HingedJointArrayMotorConfig {
    fn default() -> Self {
        Self {
            name: "hingedJointArrayMotor".to_string(),
            num_hinged_joints: 0,
            k_theta: DMatrix::zeros(0, 0),
            p_theta: DMatrix::zeros(0, 0),
            u_max_nm: None,
        }
    }
}

/// Why an update publishes nothing. Upstream raises a `BasiliskError` (or
/// reads out of bounds, for the shape errors) before writing the outputs.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum UpdateError {
    Scene(SceneError),
    /// A gain matrix is not `num_hinged_joints` square.
    GainShape(&'static str),
    /// The desired joint states hold fewer entries than there are joints.
    DesiredStatesLength,
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
        match self {
            Self::Scene(error) => write!(f, "hingedJointArrayMotor: {error}"),
            Self::GainShape(gain) => write!(
                f,
                "hingedJointArrayMotor: {gain} is not num_hinged_joints x num_hinged_joints"
            ),
            Self::DesiredStatesLength => write!(
                f,
                "hingedJointArrayMotor: des_joint_states_in_msg holds fewer states than joints"
            ),
            Self::TorqueLimitLength => write!(
                f,
                "hingedJointArrayMotor: size of u_max_nm does not match num_hinged_joints"
            ),
        }
    }
}

/// Motor torques of an array of hinged joints from their desired states:
/// upstream `HingedJointArrayMotor`.
#[derive(Clone, Debug)]
pub struct HingedJointArrayMotor {
    /// Module configuration.
    pub config: HingedJointArrayMotorConfig,
    /// Upstream `massMatrixInMsg`: scene mass matrix.
    pub mass_matrix_in_msg: Input<MjSystemMassMatrixMsg>,
    /// Upstream `reactionForcesInMsg`: scene joints and generalized forces.
    pub reaction_forces_in_msg: Input<MjJointReactionsMsg>,
    /// Upstream `desJointStatesInMsg`: desired angle, rate and acceleration
    /// of each joint, in scene order.
    pub des_joint_states_in_msg: Input<JointArrayStateMsg>,
    /// Upstream `jointStatesInMsgs`: angle of each joint, one input per
    /// joint.
    pub joint_states_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `jointStateDotsInMsgs`: rate of each joint, one input per
    /// joint.
    pub joint_state_dots_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `motorTorquesOutMsgs`: motor torque \[N m\] of each joint,
    /// one output per joint in scene order, registered with
    /// [`HingedJointArrayMotor::add_motor_torque_output`].
    pub motor_torques_out_msgs: Vec<Output<SingleActuatorMsg>>,
    /// Upstream `treeMap`, found on the first update after a reset
    /// (upstream `treeInfoInitialized`).
    trees: Option<KinematicTrees>,
}

impl HingedJointArrayMotor {
    /// Builds the module with one unconnected angle and rate input per
    /// configured joint and no outputs.
    pub fn new(config: HingedJointArrayMotorConfig) -> Self {
        // Not `vec![_; n]`: cloning a port aliases it.
        let joint_inputs = || {
            (0..config.num_hinged_joints)
                .map(|_| Input::default())
                .collect()
        };
        Self {
            joint_states_in_msgs: joint_inputs(),
            joint_state_dots_in_msgs: joint_inputs(),
            config,
            mass_matrix_in_msg: Input::default(),
            reaction_forces_in_msg: Input::default(),
            des_joint_states_in_msg: Input::default(),
            motor_torques_out_msgs: Vec::new(),
            trees: None,
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

    /// Upstream `Reset`: reports missing inputs and misshapen gains, and
    /// forgets the kinematic trees.
    ///
    /// # Panics
    ///
    /// If the number of joint inputs no longer matches
    /// `config.num_hinged_joints`: the inputs are made from it in `new`.
    fn reset_module(&mut self) {
        let n = self.config.num_hinged_joints;
        assert!(
            self.joint_states_in_msgs.len() == n && self.joint_state_dots_in_msgs.len() == n,
            "hingedJointArrayMotor needs one joint_states_in_msgs and one \
             joint_state_dots_in_msgs input per config.num_hinged_joints joint"
        );
        for (connected, port) in [
            (self.mass_matrix_in_msg.is_connected(), "mass_matrix_in_msg"),
            (
                self.reaction_forces_in_msg.is_connected(),
                "reaction_forces_in_msg",
            ),
            (
                self.des_joint_states_in_msg.is_connected(),
                "des_joint_states_in_msg",
            ),
        ] {
            if !connected {
                log::error!("hingedJointArrayMotor.{port} is not connected");
            }
        }
        if n == 0 {
            log::error!("hingedJointArrayMotor has no hinged joints");
        }
        for (name, inputs) in [
            ("joint_states_in_msgs", &self.joint_states_in_msgs),
            ("joint_state_dots_in_msgs", &self.joint_state_dots_in_msgs),
        ] {
            for (index, input) in inputs.iter().enumerate() {
                if !input.is_connected() {
                    log::error!("hingedJointArrayMotor.{name}[{index}] is not connected");
                }
            }
        }
        if self.motor_torques_out_msgs.len() != n {
            log::error!(
                "hingedJointArrayMotor has {n} hinged joints but {} motor torque outputs",
                self.motor_torques_out_msgs.len()
            );
        }
        for (gain, name) in [
            (&self.config.k_theta, "k_theta"),
            (&self.config.p_theta, "p_theta"),
        ] {
            if gain.shape() != (n, n) {
                log::error!("{}", UpdateError::GainShape(name));
            }
        }
        self.trees = None;
    }

    /// Upstream `UpdateState`: the motor torque of each joint, in scene
    /// order.
    fn compute_torques(&mut self) -> Result<DVector<f64>, UpdateError> {
        let config = &self.config;
        let n = config.num_hinged_joints;
        let mass = self.mass_matrix_in_msg.read();
        let reactions = self.reaction_forces_in_msg.read();
        let desired = self.des_joint_states_in_msg.read();

        let trees = match self.trees.take() {
            Some(trees) => trees,
            None => kinematic_trees(&reactions, n)?,
        };
        let trees = self.trees.insert(trees);
        for (gain, name) in [(&config.k_theta, "k_theta"), (&config.p_theta, "p_theta")] {
            if gain.shape() != (n, n) {
                return Err(UpdateError::GainShape(name));
            }
        }
        if [&desired.states, &desired.state_dots, &desired.state_ddots]
            .iter()
            .any(|states| states.len() < n)
        {
            return Err(UpdateError::DesiredStatesLength);
        }

        let theta: Vec<f64> = self
            .joint_states_in_msgs
            .iter()
            .map(|input| input.read().state)
            .collect();
        let theta_dot: Vec<f64> = self
            .joint_state_dots_in_msgs
            .iter()
            .map(|input| input.read().state)
            .collect();

        let forces = non_actuator_forces(&reactions)?;
        let dofs = forces.len();
        check_mass_matrix(&mass.mass_matrix, dofs)?;

        let mut torques = DVector::zeros(n);
        for tree in trees.values() {
            let nh = tree.hinge_joints.len();
            if nh == 0 {
                continue;
            }
            let tree_dof_indices = tree_dofs(&reactions, tree, TRANSLATION_DOFS, dofs)?;
            let m_full = sub_matrix(&mass.mass_matrix, &tree_dof_indices);
            let m_tt = m_full.view((0, 0), (3, 3)).clone_owned();
            let m_t_theta = m_full.view((0, 3), (3, nh));
            let m_theta_t = m_full.view((3, 0), (nh, 3));
            let m_theta_theta = m_full.view((3, 3), (nh, nh));
            let base_trans_bias = sub_vector(&forces, &tree_dof_indices[..3]);
            let joint_bias = sub_vector(&forces, &tree_dof_indices[3..]);

            let global = &tree.hinge_global;
            let k_tree = sub_matrix(&config.k_theta, global);
            let p_tree = sub_matrix(&config.p_theta, global);
            let wrap = |angle: f64| angle.sin().atan2(angle.cos());
            let error = DVector::from_iterator(
                nh,
                global.iter().map(|&j| wrap(theta[j] - desired.states[j])),
            );
            let error_dot = DVector::from_iterator(
                nh,
                global.iter().map(|&j| theta_dot[j] - desired.state_dots[j]),
            );
            let theta_ddot_d =
                DVector::from_iterator(nh, global.iter().map(|&j| desired.state_ddots[j]));
            let theta_ddot_des = theta_ddot_d - &k_tree * error - &p_tree * error_dot;

            // Schur complement of the hub-translation block.
            let rhs = -(m_t_theta * &theta_ddot_des) + base_trans_bias;
            let lambda = solve_symmetric_vector(&m_tt, &rhs)?;
            let u_tree = m_theta_theta * &theta_ddot_des + m_theta_t * lambda - joint_bias;
            for (&j, u) in global.iter().zip(u_tree.iter()) {
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

impl Module for HingedJointArrayMotor {
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
    use nalgebra::{DMatrix, DVector};

    use super::{HingedJointArrayMotor, HingedJointArrayMotorConfig, UpdateError};
    use crate::fsw_algorithms::effector_interfaces::joint_trees::SceneError;
    use crate::fsw_algorithms::effector_interfaces::joint_trees::test_support::{
        assert_allclose, scene_mass_matrix, scene_reactions,
    };
    use crate::messages::{
        JointArrayStateMsg, MjJointReactionsMsg, MjSystemMassMatrixMsg, Output, ScalarJointStateMsg,
    };
    use crate::{Module, SimulationContext};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's `jointTorques` for one spacecraft.
    #[allow(clippy::too_many_arguments)]
    fn upstream_truth(
        m: &DMatrix<f64>,
        k: &DMatrix<f64>,
        p: &DMatrix<f64>,
        theta: &[f64],
        theta_dot: &[f64],
        des_theta: &[f64],
        des_theta_dot: &[f64],
        des_theta_ddot: &[f64],
        bias: &DVector<f64>,
        max_torque: Option<&[f64]>,
    ) -> Vec<f64> {
        let nj = theta.len();
        let e = DVector::from_iterator(
            nj,
            (0..nj).map(|i| {
                (theta[i] - des_theta[i])
                    .sin()
                    .atan2((theta[i] - des_theta[i]).cos())
            }),
        );
        let edot = DVector::from_iterator(nj, (0..nj).map(|i| theta_dot[i] - des_theta_dot[i]));
        let theta_ddot_des = DVector::from_column_slice(des_theta_ddot) - k * e - p * edot;
        let mtt = m.view((0, 0), (3, 3));
        let mtth = m.view((0, 6), (3, nj));
        let mtht = m.view((6, 0), (nj, 3));
        let mthth = m.view((6, 6), (nj, nj));
        let trans_bias = bias.rows(0, 3);
        let joint_bias = bias.rows(6, nj);
        let rhs = -(mtth * &theta_ddot_des) + trans_bias;
        // `np.linalg.solve`: an LU solve.
        let x = mtt.clone_owned().lu().solve(&rhs).expect("a regular M_tt");
        let mut u_h = mthth * &theta_ddot_des + mtht * x - joint_bias;
        if let Some(max_torque) = max_torque {
            for i in 0..nj {
                if u_h[i] > max_torque[i] {
                    u_h[i] = max_torque[i];
                } else if u_h[i] < -max_torque[i] {
                    u_h[i] = -max_torque[i];
                }
            }
        }
        u_h.as_slice().to_vec()
    }

    struct Fixture {
        module: HingedJointArrayMotor,
        mass_out: Output<MjSystemMassMatrixMsg>,
        reactions_out: Output<MjJointReactionsMsg>,
        _desired_out: Output<JointArrayStateMsg>,
        _joint_outs: Vec<Output<ScalarJointStateMsg>>,
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
        let config = HingedJointArrayMotorConfig {
            num_hinged_joints: n,
            k_theta: 10.0 * DMatrix::identity(n, n),
            p_theta: 2.0 * 10.0_f64.sqrt() * DMatrix::identity(n, n),
            u_max_nm: max_torque.then(|| vec![0.03; n]),
            ..Default::default()
        };
        let mut module = HingedJointArrayMotor::new(config);
        for _ in 0..n {
            module.add_motor_torque_output();
        }

        let mass_matrix = scene_mass_matrix(num_spacecraft, num_joints);
        let (theta, theta_dot): (&[f64], &[f64]) = if num_joints == 1 {
            (&[0.1], &[0.5])
        } else {
            (&[0.1, -0.5, 0.3, 0.7], &[0.5, -0.2, 0.1, -0.3])
        };
        let desired = JointArrayStateMsg {
            states: vec![0.12; n],
            state_dots: vec![0.05; n],
            state_ddots: vec![0.002; n],
        };
        let dofs = (6 + num_joints) * num_spacecraft;
        let bias_forces = vec![if non_act_forces { -2.0 } else { 0.0 }; dofs];

        let mass_out = Output::new(MjSystemMassMatrixMsg {
            mass_matrix,
            ..Default::default()
        });
        let reactions_out = Output::new(scene_reactions(
            num_spacecraft,
            num_joints,
            bias_forces.clone(),
        ));
        let desired_out = Output::new(desired);
        mass_out.connect_to(&mut module.mass_matrix_in_msg);
        reactions_out.connect_to(&mut module.reaction_forces_in_msg);
        desired_out.connect_to(&mut module.des_joint_states_in_msg);
        let mut joint_outs = Vec::new();
        for i in 0..n {
            let j = i % num_joints;
            let angle = Output::new(ScalarJointStateMsg { state: theta[j] });
            let rate = Output::new(ScalarJointStateMsg {
                state: theta_dot[j],
            });
            angle.connect_to(&mut module.joint_states_in_msgs[i]);
            rate.connect_to(&mut module.joint_state_dots_in_msgs[i]);
            joint_outs.extend([angle, rate]);
        }
        module.init();
        (
            Fixture {
                module,
                mass_out,
                reactions_out,
                _desired_out: desired_out,
                _joint_outs: joint_outs,
            },
            bias_forces,
        )
    }

    fn torques(module: &HingedJointArrayMotor) -> Vec<f64> {
        module
            .motor_torques_out_msgs
            .iter()
            .map(|output| output.read().input)
            .collect()
    }

    /// Upstream `test_hingedJointArrayMotor`, all sixteen `(nonActForces,
    /// maxTorque, numJoints, numSpacecraft)` parametrizations: a 0.1 s task
    /// run to 0.1 s, the motor torques checked against the truth with
    /// `rtol = 1e-8`. The mass matrix is drawn from a seeded generator, not
    /// NumPy's `default_rng(12345)` (see `scene_mass_matrix`). (The pytest's
    /// `getKtheta`/`getPtheta` round trip is the public `config` here.)
    #[test]
    fn matches_upstream_hinged_joint_array_motor_unit_test() {
        for non_act_forces in [false, true] {
            for max_torque in [false, true] {
                for num_joints in [1, 4] {
                    for num_spacecraft in [1, 2] {
                        let (mut fixture, bias_forces) =
                            fixture(non_act_forces, max_torque, num_joints, num_spacecraft);
                        for nanos in [0, 100_000_000] {
                            fixture.module.update(&context(nanos));
                        }
                        let actual = torques(&fixture.module);

                        let config = &fixture.module.config;
                        let mass = fixture.mass_out.read().mass_matrix;
                        let reaction_forces = DVector::from_vec(bias_forces);
                        let (theta, theta_dot): (&[f64], &[f64]) = if num_joints == 1 {
                            (&[0.1], &[0.5])
                        } else {
                            (&[0.1, -0.5, 0.3, 0.7], &[0.5, -0.2, 0.1, -0.3])
                        };
                        let n_total = num_joints * num_spacecraft;
                        let (des_theta, des_theta_dot, des_theta_ddot) = (
                            vec![0.12; n_total],
                            vec![0.05; n_total],
                            vec![0.002; n_total],
                        );
                        let mut expected = Vec::new();
                        for sc in 0..num_spacecraft {
                            let start = sc * (6 + num_joints);
                            let size = 6 + num_joints;
                            let joints = sc * num_joints..(sc + 1) * num_joints;
                            let m_sc = mass.view((start, start), (size, size)).clone_owned();
                            let k_sc = config
                                .k_theta
                                .view((joints.start, joints.start), (num_joints, num_joints))
                                .clone_owned();
                            let p_sc = config
                                .p_theta
                                .view((joints.start, joints.start), (num_joints, num_joints))
                                .clone_owned();
                            let bias_sc = -reaction_forces.rows(start, size).clone_owned();
                            expected.extend(upstream_truth(
                                &m_sc,
                                &k_sc,
                                &p_sc,
                                &theta[..num_joints],
                                &theta_dot[..num_joints],
                                &des_theta[joints.clone()],
                                &des_theta_dot[joints.clone()],
                                &des_theta_ddot[joints],
                                &bias_sc,
                                config.u_max_nm.as_deref(),
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

    /// Not upstream: the angle error is wrapped, so a full turn added to a
    /// joint angle leaves the torque unchanged.
    #[test]
    fn wraps_the_angle_error() {
        let (mut fixture, _) = fixture(true, false, 1, 1);
        fixture.module.update(&context(0));
        let on_target = torques(&fixture.module);
        let wrapped = Output::new(ScalarJointStateMsg {
            state: 0.1 + 2.0 * std::f64::consts::PI,
        });
        wrapped.connect_to(&mut fixture.module.joint_states_in_msgs[0]);
        fixture.module.update(&context(100_000_000));
        assert_allclose(&torques(&fixture.module), &on_target, 1e-12, 0.0, "wrapped");
    }

    #[test]
    fn a_scene_that_does_not_match_stops_the_update() {
        let (mut fixture, _) = fixture(false, false, 4, 1);
        fixture.module.update(&context(0));
        let before = torques(&fixture.module);

        // The scene has a hinge more than the module has joints.
        fixture
            .reactions_out
            .write(scene_reactions(1, 5, vec![0.0; 11]));
        fixture.module.reset(&context(0));
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::Scene(SceneError::HingeCountMismatch {
                configured: 4,
                found: 5
            }))
        );
        fixture.module.update(&context(0));
        assert_eq!(torques(&fixture.module), before);

        // Gains of the wrong size, and torque limits of the wrong length.
        fixture
            .reactions_out
            .write(scene_reactions(1, 4, vec![0.0; 10]));
        fixture.module.config.k_theta = DMatrix::identity(3, 3);
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::GainShape("k_theta"))
        );
        fixture.module.config.k_theta = DMatrix::identity(4, 4);
        fixture.module.config.u_max_nm = Some(vec![1.0; 3]);
        assert_eq!(
            fixture.module.compute_torques(),
            Err(UpdateError::TorqueLimitLength)
        );
        fixture.module.config.u_max_nm = None;
        fixture.mass_out.write(MjSystemMassMatrixMsg::default());
        assert!(matches!(
            fixture.module.compute_torques(),
            Err(UpdateError::Scene(SceneError::MassMatrixShape { .. }))
        ));
    }

    #[test]
    fn makes_one_angle_and_one_rate_input_per_joint() {
        let module = HingedJointArrayMotor::new(HingedJointArrayMotorConfig {
            num_hinged_joints: 3,
            ..Default::default()
        });
        assert_eq!(module.joint_states_in_msgs.len(), 3);
        assert_eq!(module.joint_state_dots_in_msgs.len(), 3);
        assert!(module.motor_torques_out_msgs.is_empty());
    }
}
