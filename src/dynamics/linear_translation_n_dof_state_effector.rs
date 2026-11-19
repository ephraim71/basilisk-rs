//! A chain of rigid bodies, each sliding along one axis fixed in the body
//! before it: port of upstream Basilisk
//! `simulation/dynamics/linearTranslationalBodies/linearTranslationBodiesNDOF`
//! (`LinearTranslationNDOFStateEffector`).
//!
//! Body `i`'s frame Fi is fixed in orientation relative to its parent frame
//! (the hub frame B for the first body, the previous body's frame otherwise)
//! by `parent_to_translating_dcm` (`dcm_FP`). Its origin moves from Fi0, at
//! `origin0_parent_m` (`r_F0P_P`) from the parent's origin, a distance ρi
//! along `axis_parent` (`fHat_P`), and its center of mass lies at
//! `center_of_mass_translating_m` (`r_FcF_F`) from its origin. No body
//! rotates relative to the hub. Each joint carries a linear spring–damper
//! acting on `ρi − ρi,ref`, `ρ̇i − ρ̇i,ref` and an optional motor force.
//! Massless bodies can compose multi-axis joints, as long as the joint mass
//! matrix stays regular.
//!
//! State layout, as upstream registers it: `[ρ1 … ρN, ρ̇1 … ρ̇N]`.
//!
//! Inputs are read in `pre_integration`, before each integration step
//! (upstream reads them in `UpdateState`); a connected input counts as
//! written. Entry `i` of the force and lock arrays drives body `i`.
//!
//! Differences from upstream, as in
//! [`super::linear_translation_one_dof_state_effector`]: `addDynamicEffector`
//! is not ported (the attached-load terms are zero), a locked axis is
//! evaluated with a zero rate during the step, its committed rate zeroed in
//! `modify_states` and, if locked at initialization, in the initial state,
//! and [`LinearTranslationNDofStateEffector::validate`] runs the checks of
//! upstream's `TranslatingBody` setters when the spacecraft initializes.

use nalgebra::{DMatrix, DVector, Matrix3, MatrixXx3, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::dynamics::state_effector_support::{
    KinematicsCache, add_back_sub, all_finite, normalized_axis,
};
use crate::messages::{
    ArrayEffectorLockMsg, ArrayMotorForceMsg, EffectorLock, Input, LinearTranslationRigidBodyMsg,
    Output, SpacecraftStateMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::avs_eigen_support::{
    DEFAULT_TOLERANCE, is_rotation_matrix, is_valid_inertia_matrix,
};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm, tilde};

/// One body of the chain (upstream `TranslatingBody`).
#[derive(Clone, Debug, PartialEq)]
pub struct TranslatingBody {
    /// Upstream `mass` \[kg\] (default 0).
    pub mass_kg: f64,
    /// Upstream `k` \[N/m\]: linear spring constant (default 0).
    pub spring_constant_n_per_m: f64,
    /// Upstream `c` \[N s/m\]: linear damping coefficient (default 0).
    pub damping_n_s_per_m: f64,
    /// Upstream `rhoInit` \[m\] (default 0).
    pub rho_init_m: f64,
    /// Upstream `rhoDotInit` \[m/s\] (default 0).
    pub rho_dot_init_mps: f64,
    /// Upstream `fHat_P`: translation axis in parent components, normalized
    /// when used (default `[1, 0, 0]`).
    pub axis_parent: Vector3<f64>,
    /// Upstream `r_FcF_F` \[m\]: center of mass relative to the body's origin,
    /// in its frame (default zero).
    pub center_of_mass_translating_m: Vector3<f64>,
    /// Upstream `r_F0P_P` \[m\]: the body's origin at ρ = 0 relative to the
    /// parent's origin, in parent components (default zero).
    pub origin0_parent_m: Vector3<f64>,
    /// Upstream `IPntFc_F` \[kg m^2\]: inertia about the center of mass in the
    /// body's frame (default identity).
    pub inertia_about_com_translating_kg_m2: Matrix3<f64>,
    /// Upstream `dcm_FP`: DCM from the parent frame to the body's frame
    /// (default identity).
    pub parent_to_translating_dcm: Matrix3<f64>,
}

impl Default for TranslatingBody {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            spring_constant_n_per_m: 0.0,
            damping_n_s_per_m: 0.0,
            rho_init_m: 0.0,
            rho_dot_init_mps: 0.0,
            axis_parent: Vector3::x(),
            center_of_mass_translating_m: Vector3::zeros(),
            origin0_parent_m: Vector3::zeros(),
            inertia_about_com_translating_kg_m2: Matrix3::identity(),
            parent_to_translating_dcm: Matrix3::identity(),
        }
    }
}

/// Commands of one joint, read from the input messages.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TranslatingJointCommand {
    /// Upstream `u` \[N\].
    pub motor_force_n: f64,
    /// Upstream `isAxisLocked`.
    pub locked: bool,
    /// Upstream `rhoRef` \[m\].
    pub rho_ref_m: f64,
    /// Upstream `rhoDotRef` \[m/s\].
    pub rho_dot_ref_mps: f64,
}

/// A chain of translating bodies (upstream
/// `LinearTranslationNDOFStateEffector`).
#[derive(Debug)]
pub struct LinearTranslationNDofStateEffector {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `motorForceInMsg` (optional): entry `i` is body `i`'s motor
    /// force.
    pub motor_force_in: Input<ArrayMotorForceMsg>,
    /// Upstream `motorLockInMsg` (optional): entry `i` locks body `i`'s axis.
    pub motor_lock_in: Input<ArrayEffectorLockMsg>,
    /// Upstream `translatingBodyRefInMsgs` (optional): reference displacement
    /// and rate of each joint, in chain order.
    pub translating_body_ref_in: Vec<Input<LinearTranslationRigidBodyMsg>>,
    /// Upstream `translatingBodyOutMsgs`: displacement and rate of each joint.
    pub translating_body_out: Vec<Output<LinearTranslationRigidBodyMsg>>,
    /// Upstream `translatingBodyConfigLogOutMsgs`: inertial state of each
    /// body, with its frame as body frame and its center of mass as position.
    pub translating_body_state_out: Vec<Output<SpacecraftStateMsg>>,
    bodies: Vec<TranslatingBody>,
    commands: Vec<TranslatingJointCommand>,
    /// Last loaded `[ρ1 … ρN, ρ̇1 … ρ̇N]`.
    state: Vec<f64>,
    back_sub: RefCell<LinearNDofTerms>,
    kinematics: KinematicsCache<Vec<TranslatingBody>, Vec<TranslatingKinematics>>,
}

impl LinearTranslationNDofStateEffector {
    /// An effector without bodies; add them with
    /// [`add_translating_body`](Self::add_translating_body).
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            motor_force_in: Input::default(),
            motor_lock_in: Input::default(),
            translating_body_ref_in: Vec::new(),
            translating_body_out: Vec::new(),
            translating_body_state_out: Vec::new(),
            bodies: Vec::new(),
            commands: Vec::new(),
            state: Vec::new(),
            back_sub: RefCell::new(LinearNDofTerms::default()),
            kinematics: KinematicsCache::default(),
        }
    }

    /// Appends `body` to the outboard end of the chain, with its input and
    /// output messages (upstream `addTranslatingBody`).
    pub fn add_translating_body(&mut self, body: TranslatingBody) {
        self.bodies.push(body);
        self.commands.push(TranslatingJointCommand::default());
        self.translating_body_ref_in.push(Input::default());
        self.translating_body_out.push(Output::default());
        self.translating_body_state_out.push(Output::default());
        self.state = self.configured_state();
    }

    /// Body `index` of the chain, counting from the hub (upstream
    /// `getTranslatingBody`).
    pub fn translating_body(&self, index: usize) -> &TranslatingBody {
        &self.bodies[index]
    }

    /// Mutable access to body `index` (upstream `getTranslatingBody` returns a
    /// shared pointer).
    pub fn translating_body_mut(&mut self, index: usize) -> &mut TranslatingBody {
        &mut self.bodies[index]
    }

    /// The joint commands read at the start of the last step.
    pub fn commands(&self) -> &[TranslatingJointCommand] {
        &self.commands
    }

    /// `[ρ1 … ρN, ρ̇1 … ρ̇N]` of the last loaded state.
    pub fn state(&self) -> &[f64] {
        &self.state
    }

    /// Upstream `validateConfiguration` (`checkBodyConfiguration`,
    /// `checkJointMassMatrix`) and the checks of upstream's `TranslatingBody`
    /// setters: at least one body, every value finite, masses, spring
    /// constants and dampers non-negative, axis norms above 0.01, every
    /// `parent_to_translating_dcm` a rotation matrix, a valid inertia tensor
    /// for each body with mass, and a regular joint mass matrix.
    pub fn validate(&self) -> Result<(), String> {
        if self.bodies.is_empty() {
            return Err("at least one translating body is required".to_string());
        }
        for (index, body) in self.bodies.iter().enumerate() {
            for (label, value) in [
                ("mass_kg", body.mass_kg),
                ("spring_constant_n_per_m", body.spring_constant_n_per_m),
                ("damping_n_s_per_m", body.damping_n_s_per_m),
                ("rho_init_m", body.rho_init_m),
                ("rho_dot_init_mps", body.rho_dot_init_mps),
            ] {
                if !value.is_finite() {
                    return Err(format!("translating body {index}: {label} must be finite"));
                }
            }
            for (label, finite) in [
                ("axis_parent", all_finite(body.axis_parent.as_slice())),
                (
                    "center_of_mass_translating_m",
                    all_finite(body.center_of_mass_translating_m.as_slice()),
                ),
                (
                    "origin0_parent_m",
                    all_finite(body.origin0_parent_m.as_slice()),
                ),
                (
                    "inertia_about_com_translating_kg_m2",
                    all_finite(body.inertia_about_com_translating_kg_m2.as_slice()),
                ),
            ] {
                if !finite {
                    return Err(format!(
                        "translating body {index}: {label} must contain only finite values"
                    ));
                }
            }
            for (label, value) in [
                ("mass_kg", body.mass_kg),
                ("spring_constant_n_per_m", body.spring_constant_n_per_m),
                ("damping_n_s_per_m", body.damping_n_s_per_m),
            ] {
                if value < 0.0 {
                    return Err(format!(
                        "translating body {index}: {label} must be greater than or equal to 0"
                    ));
                }
            }
            if body.axis_parent.norm() <= 0.01 {
                return Err(format!(
                    "translating body {index}: the norm of axis_parent must exceed 0.01"
                ));
            }
            if !is_rotation_matrix(&body.parent_to_translating_dcm, DEFAULT_TOLERANCE) {
                return Err(format!(
                    "translating body {index}: parent_to_translating_dcm must be an orthogonal, \
                     right-handed rotation matrix"
                ));
            }
            if body.mass_kg > 0.0
                && !is_valid_inertia_matrix(
                    &body.inertia_about_com_translating_kg_m2,
                    DEFAULT_TOLERANCE,
                )
            {
                return Err(format!(
                    "translating body {index}: inertia_about_com_translating_kg_m2 must be a \
                     valid inertia tensor"
                ));
            }
        }
        // The axes are fixed in their parents and the bodies do not rotate, so
        // the joint mass matrix is constant.
        let kinematics = self.kinematics(&self.configured_state(), &[]);
        let m_rho = self.joint_mass_matrix(&kinematics, &[]);
        if !m_rho.iter().all(|value| value.is_finite()) {
            return Err("the joint mass matrix must be finite".to_string());
        }
        // Upstream `Eigen::FullPivLU::rank`, with Eigen's default threshold.
        let n = self.bodies.len();
        let singular_values = m_rho.singular_values();
        let threshold = singular_values.max() * f64::EPSILON * n as f64;
        let rank = singular_values
            .iter()
            .filter(|value| **value > threshold)
            .count();
        if rank < n {
            return Err(
                "the translating body masses and axes leave the joint mass matrix singular \
                 because at least one nonzero combination of joint rates leaves every \
                 mass-bearing body stationary"
                    .to_string(),
            );
        }
        Ok(())
    }

    /// Inertial state of every body (upstream
    /// `computeTranslatingBodyInertialStates`) for the last loaded state and
    /// the given hub state.
    pub fn translating_inertial_states(
        &self,
        hub_state: &SpacecraftStateMsg,
    ) -> Vec<SpacecraftStateMsg> {
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        self.kinematics(&self.state, &self.commands)
            .iter()
            .map(|k| SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_nb * k.r_fcb_b,
                velocity_mps: hub_state.velocity_mps
                    + dcm_nb * (k.r_prime_fcb_b + omega_bn_b.cross(&k.r_fcb_b)),
                sigma_bn: dcm_to_mrp(k.dcm_fb * dcm_bn),
                omega_radps: k.dcm_fb * omega_bn_b,
                ..Default::default()
            })
            .collect()
    }

    fn configured_state(&self) -> Vec<f64> {
        self.bodies
            .iter()
            .map(|body| body.rho_init_m)
            .chain(self.bodies.iter().map(|body| body.rho_dot_init_mps))
            .collect()
    }

    /// Upstream `updateEffectorMassProps`'s per-body geometry; the rate of a
    /// body that `commands` locks is zero.
    fn kinematics(
        &self,
        effector_state: &[f64],
        commands: &[TranslatingJointCommand],
    ) -> Vec<TranslatingKinematics> {
        let n = self.bodies.len();
        assert_eq!(
            effector_state.len(),
            2 * n,
            "linear translation N DOF state length mismatch"
        );
        let mut key = effector_state.to_vec();
        for (rate, command) in key[n..].iter_mut().zip(commands) {
            if command.locked {
                *rate = 0.0;
            }
        }
        self.kinematics
            .get_or_compute(&key, &self.bodies, || self.chain_kinematics(&key))
    }

    fn chain_kinematics(&self, effector_state: &[f64]) -> Vec<TranslatingKinematics> {
        let n = self.bodies.len();
        let (rho, rho_dot) = effector_state.split_at(n);
        let mut chain: Vec<TranslatingKinematics> = Vec::with_capacity(n);
        for (index, body) in self.bodies.iter().enumerate() {
            let rho_dot = rho_dot[index];
            let (dcm_pb, r_pb_b, r_prime_pb_b) = match chain.last() {
                Some(parent) => (parent.dcm_fb, parent.r_fb_b, parent.r_prime_fb_b),
                None => (Matrix3::identity(), Vector3::zeros(), Vector3::zeros()),
            };
            let dcm_fb = body.parent_to_translating_dcm * dcm_pb;
            let f_hat_b = dcm_pb.transpose() * normalized_axis(body.axis_parent);
            let r_fb_b = dcm_pb.transpose() * body.origin0_parent_m + rho[index] * f_hat_b + r_pb_b;
            let r_prime_fb_b = rho_dot * f_hat_b + r_prime_pb_b;
            chain.push(TranslatingKinematics {
                rho: rho[index],
                rho_dot,
                dcm_fb,
                f_hat_b,
                r_fb_b,
                r_prime_fb_b,
                r_fcb_b: dcm_fb.transpose() * body.center_of_mass_translating_m + r_fb_b,
                r_prime_fcb_b: r_prime_fb_b,
                i_pnt_fc_b: dcm_fb.transpose() * body.inertia_about_com_translating_kg_m2 * dcm_fb,
            });
        }
        chain
    }

    /// Upstream `computeMRho`.
    fn joint_mass_matrix(
        &self,
        k: &[TranslatingKinematics],
        commands: &[TranslatingJointCommand],
    ) -> DMatrix<f64> {
        let n = k.len();
        let locked = |index: usize| commands.get(index).is_some_and(|command| command.locked);
        DMatrix::from_fn(n, n, |row, col| {
            if (locked(row) || locked(col)) && row != col {
                return 0.0;
            }
            (row.max(col)..n)
                .map(|j| self.bodies[j].mass_kg * k[row].f_hat_b.dot(&k[col].f_hat_b))
                .sum()
        })
    }

    /// Upstream `updateContributions`, with 0-based body indices.
    fn back_sub_terms(
        &self,
        k: &[TranslatingKinematics],
        omega_bn_b: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> LinearNDofTerms {
        let n = k.len();
        let mass = |index: usize| self.bodies[index].mass_kg;
        let m_rho = self.joint_mass_matrix(k, &self.commands);

        // Upstream `computeARhoStar`, `computeBRhoStar` and `computeCRhoStar`.
        let mut a_rho_star = MatrixXx3::<f64>::zeros(n);
        let mut b_rho_star = MatrixXx3::<f64>::zeros(n);
        let mut c_rho_star = DVector::<f64>::zeros(n);
        for row in 0..n {
            let command = &self.commands[row];
            if command.locked {
                continue;
            }
            let f_hat = k[row].f_hat_b;
            let body = &self.bodies[row];
            let mut a = Vector3::zeros();
            let mut b = Vector3::zeros();
            let mut c = command.motor_force_n
                - body.spring_constant_n_per_m * (k[row].rho - command.rho_ref_m)
                - body.damping_n_s_per_m * (k[row].rho_dot - command.rho_dot_ref_mps);
            for (body_i, k_i) in self.bodies.iter().zip(k).skip(row) {
                let m_i = body_i.mass_kg;
                a -= m_i * f_hat;
                b += m_i * f_hat.cross(&k_i.r_fcb_b);
                c += f_hat.dot(
                    &(m_i * gravity_body_mps2
                        - m_i
                            * (omega_bn_b.cross(&omega_bn_b.cross(&k_i.r_fcb_b))
                                + 2.0 * omega_bn_b.cross(&k_i.r_prime_fcb_b))),
                );
            }
            a_rho_star.set_row(row, &a.transpose());
            b_rho_star.set_row(row, &b.transpose());
            c_rho_star[row] = c;
        }

        let m_rho_inverse = m_rho
            .try_inverse()
            .expect("linear translation N DOF joint mass matrix must be invertible");
        let a_rho: MatrixXx3<f64> = &m_rho_inverse * a_rho_star;
        let b_rho: MatrixXx3<f64> = &m_rho_inverse * b_rho_star;
        let c_rho: DVector<f64> = &m_rho_inverse * c_rho_star;

        // Upstream `computeBackSubContributions`.
        let mut back_sub = BackSubMatrices::default();
        for i in 0..n {
            back_sub.vec_rot -=
                mass(i) * omega_bn_b.cross(&k[i].r_fcb_b.cross(&k[i].r_prime_fcb_b));
            for j in i..n {
                let r_cross_f_hat = k[j].r_fcb_b.cross(&k[i].f_hat_b);
                let trans = mass(j) * k[i].f_hat_b;
                let rot = mass(j) * r_cross_f_hat;
                back_sub.matrix_a += trans * a_rho.row(i);
                back_sub.matrix_b += trans * b_rho.row(i);
                back_sub.vec_trans -= trans * c_rho[i];
                back_sub.matrix_c += rot * a_rho.row(i);
                back_sub.matrix_d += rot * b_rho.row(i);
                back_sub.vec_rot -= rot * c_rho[i];
            }
        }

        LinearNDofTerms {
            back_sub,
            a_rho,
            b_rho,
            c_rho,
        }
    }
}

impl StateEffector for LinearTranslationNDofStateEffector {
    fn name(&self) -> &str {
        &self.name
    }

    fn state_len(&self) -> usize {
        2 * self.bodies.len()
    }

    /// Validates the chain (upstream `registerStates`) and returns the initial
    /// `[ρ1 … ρN, ρ̇1 … ρ̇N]`, with the rates of axes that `motor_lock_in`
    /// already locks zeroed.
    ///
    /// # Panics
    ///
    /// If [`LinearTranslationNDofStateEffector::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.validate() {
            panic!(
                "invalid LinearTranslationNDofStateEffector '{}': {message}",
                self.name
            );
        }
        let mut state = self.configured_state();
        if self.motor_lock_in.is_connected() {
            let lock = self.motor_lock_in.read();
            let n = self.bodies.len();
            for index in 0..n {
                if lock.effector_lock[index] == EffectorLock::Locked {
                    state[n + index] = 0.0;
                }
            }
        }
        state
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "linear translation N DOF state length mismatch"
        );
        self.state.clear();
        self.state.extend_from_slice(state);
    }

    /// Upstream `readInputMessages`.
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        if self.motor_force_in.is_connected() {
            let force = self.motor_force_in.read().motor_force_n;
            for (command, force_n) in self.commands.iter_mut().zip(force) {
                command.motor_force_n = force_n;
            }
        }
        if self.motor_lock_in.is_connected() {
            let lock = self.motor_lock_in.read().effector_lock;
            for (command, lock) in self.commands.iter_mut().zip(lock) {
                command.locked = lock == EffectorLock::Locked;
            }
        }
        for (command, input) in self.commands.iter_mut().zip(&self.translating_body_ref_in) {
            if input.is_connected() {
                let reference = input.read();
                command.rho_ref_m = reference.rho_m;
                command.rho_dot_ref_mps = reference.rho_dot_mps;
            }
        }
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let kinematics = self.kinematics(effector_state, &self.commands);
        let terms = self.back_sub_terms(&kinematics, body_omega_radps, gravity_body_mps2);
        add_back_sub(back_sub, &terms.back_sub);
        self.back_sub.replace(terms);
    }

    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        let kinematics = self.kinematics(effector_state, &self.commands);
        let terms = self.back_sub.borrow();
        let rho_ddot = &terms.a_rho * body_trans_accel_mps2
            + &terms.b_rho * body_omega_dot_radps2
            + &terms.c_rho;
        kinematics
            .iter()
            .map(|k| k.rho_dot)
            .chain(rho_ddot.iter().copied())
            .collect()
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let kinematics = self.kinematics(effector_state, &self.commands);
        let mut props = StateEffectorMassProps::default();
        for (body, k) in self.bodies.iter().zip(&kinematics) {
            let m = body.mass_kg;
            let r_tilde = tilde(k.r_fcb_b);
            let r_prime_tilde = tilde(k.r_prime_fcb_b);
            props.mass_kg += m;
            props.center_of_mass_body_m += m * k.r_fcb_b;
            props.center_of_mass_prime_body_mps += m * k.r_prime_fcb_b;
            props.inertia_about_point_b_body_kg_m2 += k.i_pnt_fc_b - m * r_tilde * r_tilde;
            props.inertia_about_point_b_body_prime_kg_m2ps -=
                m * (r_prime_tilde * r_tilde + r_tilde * r_prime_tilde);
        }
        props.center_of_mass_body_m /= props.mass_kg;
        props.center_of_mass_prime_body_mps /= props.mass_kg;
        props
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B); the
    /// bodies share the hub's rate.
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.bodies
            .iter()
            .zip(self.kinematics(effector_state, &self.commands))
            .map(|(body, k)| {
                let r_dot_fcb_b = k.r_prime_fcb_b + body_omega_radps.cross(&k.r_fcb_b);
                k.i_pnt_fc_b * body_omega_radps + body.mass_kg * k.r_fcb_b.cross(&r_dot_fcb_b)
            })
            .sum()
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the spring energies.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.bodies
            .iter()
            .zip(&self.commands)
            .zip(self.kinematics(effector_state, &self.commands))
            .map(|((body, command), k)| {
                let r_dot_fcb_b = k.r_prime_fcb_b + body_omega_radps.cross(&k.r_fcb_b);
                let deflection_m = k.rho - command.rho_ref_m;
                0.5 * body_omega_radps.dot(&(k.i_pnt_fc_b * body_omega_radps))
                    + 0.5 * body.mass_kg * r_dot_fcb_b.norm_squared()
                    + 0.5 * body.spring_constant_n_per_m * deflection_m * deflection_m
            })
            .sum()
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        let n = self.bodies.len();
        for (index, output) in self.translating_body_out.iter().enumerate() {
            output.write(LinearTranslationRigidBodyMsg {
                rho_m: self.state[index],
                rho_dot_mps: self.state[n + index],
            });
        }
        for (output, state) in self
            .translating_body_state_out
            .iter()
            .zip(self.translating_inertial_states(hub_state))
        {
            output.write(state);
        }
    }

    /// Zeroes the rates of locked axes (upstream overwrites the state the same
    /// way).
    fn modify_states(&mut self, _current_sim_nanos: u64, effector_state: &mut [f64]) {
        let n = self.bodies.len();
        for (index, command) in self.commands.iter().enumerate() {
            if command.locked {
                effector_state[n + index] = 0.0;
            }
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

/// Back-substitution contributions and the joint-acceleration terms
/// `ρ̈ = A r̈_B + B ω̇ + C` cached for `compute_derivatives`.
#[derive(Clone, Debug)]
struct LinearNDofTerms {
    back_sub: BackSubMatrices,
    /// Upstream `ARho`.
    a_rho: MatrixXx3<f64>,
    /// Upstream `BRho`.
    b_rho: MatrixXx3<f64>,
    /// Upstream `CRho`.
    c_rho: DVector<f64>,
}

impl Default for LinearNDofTerms {
    fn default() -> Self {
        Self {
            back_sub: BackSubMatrices::default(),
            a_rho: MatrixXx3::zeros(0),
            b_rho: MatrixXx3::zeros(0),
            c_rho: DVector::zeros(0),
        }
    }
}

/// One body's geometry for a given chain state (upstream `TranslatingBody`'s
/// derived members).
#[derive(Clone, Debug)]
struct TranslatingKinematics {
    rho: f64,
    /// ρ̇, zero for a locked axis.
    rho_dot: f64,
    /// Upstream `dcm_FB`.
    dcm_fb: Matrix3<f64>,
    /// Upstream `fHat_B`.
    f_hat_b: Vector3<f64>,
    /// Upstream `r_FB_B`.
    r_fb_b: Vector3<f64>,
    /// Upstream `rPrime_FB_B`.
    r_prime_fb_b: Vector3<f64>,
    /// Upstream `r_FcB_B`.
    r_fcb_b: Vector3<f64>,
    /// Upstream `rPrime_FcB_B`.
    r_prime_fcb_b: Vector3<f64>,
    /// Upstream `IPntFc_B`.
    i_pnt_fc_b: Matrix3<f64>,
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_linearTranslationNDOFStateEffector.py` and
    //! `test_changeParameterslinearTranslatingBodyNDOF.py`.
    //!
    //! The integrated tests draw their parameters from `np.random` without a
    //! seed; here they come from a seeded generator over the same ranges. The
    //! configuration-validation test's `scheduleEffector` parameter has no
    //! counterpart: validation runs once, when the spacecraft initializes.
    //!
    //! Not ported: `test_translatingBodyDynamicEffectorSegmentBounds`, which
    //! exercises `addDynamicEffector` (not ported, see the module docs).

    use super::{LinearTranslationNDofStateEffector, TranslatingBody};
    use crate::dynamics::linear_translation_one_dof_state_effector::{
        LinearTranslationOneDofConfig, LinearTranslationOneDofStateEffector,
    };
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, assert_relative_equal,
        run_logging_every_step, spacecraft_with_effectors,
    };
    use crate::messages::{
        ArrayEffectorLockMsg, ArrayMotorForceMsg, EffectorLock, Output, SpacecraftStateMsg,
    };
    use crate::simulation::Simulation;
    use crate::spacecraft::StateEffector;
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    /// Upstream `randomValidInertia`.
    fn random_valid_inertia(rng: &mut StdRng) -> Matrix3<f64> {
        let second_moments: [f64; 3] = std::array::from_fn(|_| rng.gen_range(2.5..50.0));
        Matrix3::from_diagonal(&Vector3::new(
            second_moments[1] + second_moments[2],
            second_moments[0] + second_moments[2],
            second_moments[0] + second_moments[1],
        ))
    }

    fn random_vector(rng: &mut StdRng) -> Vector3<f64> {
        Vector3::from_fn(|_, _| rng.gen_range(-1.0..1.0))
    }

    /// The four-body chain of the integrated tests.
    fn upstream_chain(seed: u64) -> LinearTranslationNDofStateEffector {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut effector = LinearTranslationNDofStateEffector::new("translatingBodyEffector");
        let tilted = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        let in_plane = Vector3::new(3.0 / 5.0, 4.0 / 5.0, 0.0);
        let bodies = [
            (tilted, in_plane, -5.0..10.0),
            (Matrix3::identity(), in_plane, -5.0..5.0),
            (Matrix3::identity(), in_plane, -5.0..5.0),
            (Matrix3::identity(), Vector3::z(), -5.0..5.0),
        ];
        for (dcm, axis, rho_range) in bodies {
            let mass_kg = rng.gen_range(5.0..50.0);
            let inertia = random_valid_inertia(&mut rng);
            let center_of_mass = random_vector(&mut rng);
            let origin = random_vector(&mut rng);
            let rho_init_m = rng.gen_range(rho_range);
            effector.add_translating_body(TranslatingBody {
                mass_kg,
                spring_constant_n_per_m: rng.r#gen(),
                damping_n_s_per_m: 0.0,
                rho_init_m,
                rho_dot_init_mps: 0.05,
                axis_parent: axis,
                center_of_mass_translating_m: center_of_mass,
                origin0_parent_m: origin,
                inertia_about_com_translating_kg_m2: inertia,
                parent_to_translating_dcm: dcm,
            });
        }
        effector
    }

    /// The integrated tests: 5 s at a 1 ms step, conservation to 1e-13.
    fn integrated_test(
        effector: LinearTranslationNDofStateEffector,
        check_rotational_energy: bool,
    ) {
        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        hub.omega_bn_b_radps = Vector3::new(0.1, -0.1, 0.1);
        let mut spacecraft = spacecraft_with_effectors(&hub, 1_000_000, vec![Box::new(effector)]);
        add_point_mass_earth(&mut spacecraft);

        let log = run_logging_every_step(&mut spacecraft, 5_000, || {});

        assert_conserved(&log[0], &log[log.len() - 1], 1e-13, check_rotational_energy);
    }

    /// Upstream `translatingBodyNoInput`.
    #[test]
    fn translating_body_no_input() {
        integrated_test(upstream_chain(1), true);
    }

    /// Upstream `translatingBodyLockAxis`: bodies 1 and 4 locked.
    #[test]
    fn translating_body_lock_axis() {
        let mut effector = upstream_chain(2);
        let mut lock = ArrayEffectorLockMsg::default();
        lock.effector_lock[0] = EffectorLock::Locked;
        lock.effector_lock[3] = EffectorLock::Locked;
        let lock = Output::new(lock);
        effector.motor_lock_in.connect(lock.slot());
        let rho = [
            effector.translating_body_out[0].clone(),
            effector.translating_body_out[3].clone(),
        ];
        let rho_init = [
            effector.translating_body(0).rho_init_m,
            effector.translating_body(3).rho_init_m,
        ];
        integrated_test(effector, true);
        for (output, rho_init) in rho.iter().zip(rho_init) {
            let state = output.read();
            assert_eq!((state.rho_m, state.rho_dot_mps), (rho_init, 0.0));
        }
    }

    /// Upstream `translatingBodyCommandedForce`.
    #[test]
    fn translating_body_commanded_force() {
        let mut effector = upstream_chain(3);
        let force = Output::new(ArrayMotorForceMsg::from_active(&[0.1, -0.2, 0.3, -0.15]));
        effector.motor_force_in.connect(force.slot());
        integrated_test(effector, false);
    }

    /// Upstream `test_translatingBodyOutputMessagesMatchOneDOF`: a one-body
    /// chain and the equivalent one-DOF effector, each on its own tumbling
    /// spacecraft, agree on every output over ten 10 ms steps.
    #[test]
    fn output_messages_match_one_dof() {
        let body = TranslatingBody {
            mass_kg: 20.0,
            spring_constant_n_per_m: 100.0,
            damping_n_s_per_m: 0.0,
            rho_init_m: 0.4,
            rho_dot_init_mps: 0.05,
            axis_parent: Vector3::new(3.0 / 5.0, 4.0 / 5.0, 0.0),
            center_of_mass_translating_m: Vector3::new(-1.0, 1.0, 0.5),
            origin0_parent_m: Vector3::new(-1.0, 1.0, 0.0),
            inertia_about_com_translating_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                50.0, 80.0, 60.0,
            )),
            parent_to_translating_dcm: Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0),
        };
        let mut n_dof = LinearTranslationNDofStateEffector::new("translatingBodyNDOF");
        n_dof.add_translating_body(body.clone());
        let mut one_dof_config = LinearTranslationOneDofConfig::new("translatingBodyOneDOF");
        one_dof_config.mass_kg = body.mass_kg;
        one_dof_config.spring_constant_n_per_m = body.spring_constant_n_per_m;
        one_dof_config.rho_init_m = body.rho_init_m;
        one_dof_config.rho_dot_init_mps = body.rho_dot_init_mps;
        one_dof_config.axis_body = body.axis_parent;
        one_dof_config.center_of_mass_translating_m = body.center_of_mass_translating_m;
        one_dof_config.origin0_body_m = body.origin0_parent_m;
        one_dof_config.inertia_about_com_translating_kg_m2 =
            body.inertia_about_com_translating_kg_m2;
        one_dof_config.body_to_translating_dcm = body.parent_to_translating_dcm;
        let one_dof = LinearTranslationOneDofStateEffector::new(one_dof_config);

        let n_outputs = (
            n_dof.translating_body_out[0].clone(),
            n_dof.translating_body_state_out[0].clone(),
        );
        let one_outputs = (
            one_dof.translating_body_out.clone(),
            one_dof.translating_body_state_out.clone(),
        );
        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        hub.sigma_bn = Vector3::new(0.1, 0.2, -0.3);
        hub.omega_bn_b_radps = Vector3::new(0.5, -0.4, 0.6);
        let step_nanos = 10_000_000;
        let mut spacecraft_n = spacecraft_with_effectors(&hub, step_nanos, vec![Box::new(n_dof)]);
        let mut spacecraft_one =
            spacecraft_with_effectors(&hub, step_nanos, vec![Box::new(one_dof)]);
        add_point_mass_earth(&mut spacecraft_n);
        add_point_mass_earth(&mut spacecraft_one);

        let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        sim.add_module("spacecraftNDOF", &mut spacecraft_n, step_nanos, 0);
        sim.add_module("spacecraftOneDOF", &mut spacecraft_one, step_nanos, 0);
        let accuracy = 1e-10;
        let close = |a: &[f64], b: &[f64], what: &str| {
            for (a, b) in a.iter().zip(b) {
                assert!(
                    (a - b).abs() <= accuracy + accuracy * b.abs(),
                    "{what}: {a:.17e} vs {b:.17e}"
                );
            }
        };
        let compare = |n: &SpacecraftStateMsg, one: &SpacecraftStateMsg| {
            close(n.position_m.as_slice(), one.position_m.as_slice(), "r_BN_N");
            close(
                n.velocity_mps.as_slice(),
                one.velocity_mps.as_slice(),
                "v_BN_N",
            );
            close(n.sigma_bn.as_slice(), one.sigma_bn.as_slice(), "sigma_BN");
            close(
                n.omega_radps.as_slice(),
                one.omega_radps.as_slice(),
                "omega_BN_B",
            );
        };
        sim.run_for(0);
        for _ in 0..=10 {
            let (n_rho, one_rho) = (n_outputs.0.read(), one_outputs.0.read());
            assert_relative_equal(&[n_rho.rho_m], &[one_rho.rho_m], accuracy, "rho");
            assert_relative_equal(
                &[n_rho.rho_dot_mps],
                &[one_rho.rho_dot_mps],
                accuracy,
                "rhoDot",
            );
            compare(&n_outputs.1.read(), &one_outputs.1.read());
            sim.run_for(step_nanos);
        }
    }

    /// Upstream `test_translatingBodyMassSetterBoundary`.
    #[test]
    fn mass_boundary() {
        // A massless joint stage below a body with mass, as upstream's
        // `MasslessJoint` chain.
        let chain = |mass_kg: f64| {
            let mut effector = LinearTranslationNDofStateEffector::new("chain");
            effector.add_translating_body(TranslatingBody {
                mass_kg,
                inertia_about_com_translating_kg_m2: Matrix3::zeros(),
                ..TranslatingBody::default()
            });
            effector.add_translating_body(TranslatingBody {
                mass_kg: 1.0,
                axis_parent: Vector3::y(),
                ..TranslatingBody::default()
            });
            effector
        };
        assert_eq!(chain(0.0).validate(), Ok(()));
        let negative = chain(-1.0).validate();
        assert!(negative.is_err_and(|message| message.contains("greater than or equal to 0")));
    }

    /// Upstream `test_changeParametersTranslatingBody`.
    #[test]
    fn change_parameters_of_an_added_body() {
        let mut effector = LinearTranslationNDofStateEffector::new("chain");
        effector.add_translating_body(TranslatingBody::default());
        let expected = TranslatingBody {
            mass_kg: 20.0,
            spring_constant_n_per_m: 152.0,
            damping_n_s_per_m: 432.0,
            rho_init_m: 0.0,
            rho_dot_init_mps: 0.0,
            axis_parent: Vector3::x(),
            center_of_mass_translating_m: Vector3::new(0.412, 0.623, 0.456),
            origin0_parent_m: Vector3::new(0.199, 0.624, 0.771),
            inertia_about_com_translating_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                767.0, 912.0, 360.0,
            )),
            parent_to_translating_dcm: Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
        };
        *effector.translating_body_mut(0) = expected.clone();
        assert_eq!(effector.translating_body(0), &expected);
    }

    /// Axis and mass \[kg\] of each body of upstream's `VALIDATION_CHAINS`.
    fn validation_chain(chain: &str) -> Vec<(f64, Vector3<f64>)> {
        let (x, y, z) = (Vector3::x(), Vector3::y(), Vector3::z());
        let xy = Vector3::new(1.0, 1.0, 0.0);
        match chain {
            "MasslessJoint" => vec![(0.0, x), (20.0, y)],
            "MasslessChain" => vec![(0.0, x), (0.0, y), (20.0, z)],
            "MasslessOutermost" => vec![(20.0, x), (0.0, y)],
            "CollinearMassless" => vec![(0.0, x), (20.0, x)],
            "CoplanarMassless" => vec![(0.0, x), (0.0, y), (20.0, xy)],
            "NoBodies" => Vec::new(),
            _ => vec![(20.0, x), (15.0, y)],
        }
    }

    /// Upstream `test_translatingBodyConfigurationValidation`: invalid chains
    /// are rejected, and accepted ones conserve rotational energy and angular
    /// momentum over 1 s at a 1 ms step to 1e-10.
    #[test]
    fn configuration_validation() {
        let cases = [
            ("Valid", false),
            ("MasslessJoint", false),
            ("MasslessChain", false),
            ("MasslessOutermost", true),
            ("CollinearMassless", true),
            ("CoplanarMassless", true),
            ("NoBodies", true),
            ("SkewedDCM", true),
            ("AsymmetricInertia", true),
            ("TriangleInertia", true),
        ];
        for (chain, should_fail) in cases {
            let mut effector = LinearTranslationNDofStateEffector::new(chain);
            for (mass_kg, axis) in validation_chain(chain) {
                let mut body = TranslatingBody {
                    mass_kg,
                    spring_constant_n_per_m: 10.0,
                    rho_init_m: 0.1,
                    rho_dot_init_mps: 0.05,
                    axis_parent: axis,
                    center_of_mass_translating_m: Vector3::new(0.5, 0.2, -0.3),
                    origin0_parent_m: Vector3::new(1.0, 0.5, 0.25),
                    ..TranslatingBody::default()
                };
                if mass_kg == 0.0 {
                    body.inertia_about_com_translating_kg_m2 = Matrix3::zeros();
                }
                match chain {
                    "SkewedDCM" => {
                        body.parent_to_translating_dcm =
                            Matrix3::new(1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
                    }
                    "AsymmetricInertia" => {
                        body.inertia_about_com_translating_kg_m2 =
                            Matrix3::new(50.0, 3.0, 0.0, 0.0, 80.0, 0.0, 0.0, 0.0, 60.0)
                    }
                    "TriangleInertia" => {
                        body.inertia_about_com_translating_kg_m2 =
                            Matrix3::from_diagonal(&Vector3::new(10.0, 10.0, 90.0))
                    }
                    _ => {}
                }
                effector.add_translating_body(body);
            }

            let result = effector.validate();
            if should_fail {
                let message = result.expect_err(chain);
                if matches!(
                    chain,
                    "MasslessOutermost" | "CollinearMassless" | "CoplanarMassless"
                ) {
                    assert!(
                        message.contains(
                            "nonzero combination of joint rates leaves every mass-bearing body \
                             stationary"
                        ),
                        "{chain}: {message}"
                    );
                }
                continue;
            }
            assert_eq!(result, Ok(()), "{chain}");

            let mut hub = HubSetup::hinged_body_hub();
            hub.omega_bn_b_radps = Vector3::new(0.1, -0.2, 0.3);
            let mut spacecraft =
                spacecraft_with_effectors(&hub, 1_000_000, vec![Box::new(effector)]);
            let log = run_logging_every_step(&mut spacecraft, 1_000, || {});
            let accuracy = 1e-10;
            for sample in &log {
                assert_relative_equal(
                    &[sample.rotational_energy_j],
                    &[log[0].rotational_energy_j],
                    accuracy,
                    &format!("{chain}: rotational energy"),
                );
                assert_relative_equal(
                    sample
                        .rotational_angular_momentum_inertial_kg_m2ps
                        .as_slice(),
                    log[0]
                        .rotational_angular_momentum_inertial_kg_m2ps
                        .as_slice(),
                    accuracy,
                    &format!("{chain}: rotational angular momentum"),
                );
            }
        }
    }

    #[test]
    #[should_panic(expected = "at least one translating body")]
    fn spacecraft_initialization_rejects_empty_chain() {
        let _ = LinearTranslationNDofStateEffector::new("chain").initial_state();
    }
}
