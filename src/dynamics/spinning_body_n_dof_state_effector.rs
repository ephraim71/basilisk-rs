//! A chain of rigid bodies, each spinning about one axis relative to the one
//! before it: port of upstream Basilisk
//! `simulation/dynamics/spinningBodies/spinningBodiesNDOF`
//! (`SpinningBodyNDOFStateEffector`).
//!
//! Body `i`'s frame Si is the frame Si0, fixed in its parent frame P (the
//! hub frame B for the first body, the previous body's frame otherwise) by
//! `parent_to_spinner0_dcm` (`dcm_S0P`) at `origin_parent_m` (`r_SP_P`),
//! rotated by θi about its spin axis ŝi. Each joint carries a torsional
//! spring–damper acting on `θi − θi,ref`, `θ̇i − θ̇i,ref` and an optional motor
//! torque. Massless bodies can be used to compose multi-axis joints, but the
//! outermost body must carry mass.
//!
//! State layout, as upstream registers it: `[θ1 … θN, θ̇1 … θ̇N]`.
//!
//! Inputs are read in `pre_integration`, before each integration step
//! (upstream reads them in `UpdateState`); a connected input counts as
//! written. Entry `i` of the torque and lock arrays drives body `i`.
//!
//! Differences from upstream, as in
//! [`super::spinning_body_one_dof_state_effector`]: `addDynamicEffector` is not
//! ported (the attached-load terms are zero), and a locked axis is evaluated
//! with a zero rate during the step, its committed rate zeroed in
//! `modify_states` and, if locked at initialization, in the initial state.
//! Upstream's setters validate `mass`, `k`, `c` and `sHat_S` as they are set;
//! here the fields are public and [`SpinningBodyNDofStateEffector::validate`]
//! checks them when the spacecraft initializes.

use nalgebra::{DMatrix, DVector, Matrix3, MatrixXx3, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::dynamics::state_effector_support::{
    KinematicsCache, add_back_sub, all_finite, normalized_axis,
};
use crate::messages::{
    ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, HingedRigidBodyMsg, Input,
    MAX_EFF_COUNT, Output, SpacecraftStateMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::avs_eigen_support::{
    DEFAULT_TOLERANCE, is_rotation_matrix, is_valid_inertia_matrix,
};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm, prv_to_dcm, tilde};

/// One body of the chain (upstream `SpinningBody`).
#[derive(Clone, Debug, PartialEq)]
pub struct SpinningBody {
    /// Upstream `mass` \[kg\] (default 1; zero is allowed except for the
    /// outermost body).
    pub mass_kg: f64,
    /// Upstream `k` \[N m/rad\]: torsional spring constant (default 0).
    pub spring_constant_nm_per_rad: f64,
    /// Upstream `c` \[N m s/rad\]: rotational damping coefficient (default 0).
    pub damping_nm_s_per_rad: f64,
    /// Upstream `thetaInit` \[rad\] (default 0).
    pub theta_init_rad: f64,
    /// Upstream `thetaDotInit` \[rad/s\] (default 0).
    pub theta_dot_init_radps: f64,
    /// Upstream `r_SP_P` \[m\]: the body's frame origin relative to its
    /// parent's, in parent components (default zero).
    pub origin_parent_m: Vector3<f64>,
    /// Upstream `r_ScS_S` \[m\]: center of mass relative to the body's frame
    /// origin, in its frame (default zero).
    pub center_of_mass_spinner_m: Vector3<f64>,
    /// Upstream `sHat_S`: spin axis in the body's frame, normalized when used
    /// (default `[1, 0, 0]`).
    pub spin_axis_spinner: Vector3<f64>,
    /// Upstream `ISPntSc_S` \[kg m^2\]: inertia about the center of mass in
    /// the body's frame (default identity).
    pub inertia_about_com_spinner_kg_m2: Matrix3<f64>,
    /// Upstream `dcm_S0P`: DCM from the parent frame to the body frame at
    /// θ = 0 (default identity).
    pub parent_to_spinner0_dcm: Matrix3<f64>,
}

impl Default for SpinningBody {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            spring_constant_nm_per_rad: 0.0,
            damping_nm_s_per_rad: 0.0,
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            origin_parent_m: Vector3::zeros(),
            center_of_mass_spinner_m: Vector3::zeros(),
            spin_axis_spinner: Vector3::x(),
            inertia_about_com_spinner_kg_m2: Matrix3::identity(),
            parent_to_spinner0_dcm: Matrix3::identity(),
        }
    }
}

/// Commands of one joint, read from the input messages.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SpinningJointCommand {
    /// Upstream `u` \[N m\].
    pub motor_torque_nm: f64,
    /// Upstream `isAxisLocked`.
    pub locked: bool,
    /// Upstream `thetaRef` \[rad\].
    pub theta_ref_rad: f64,
    /// Upstream `thetaDotRef` \[rad/s\].
    pub theta_dot_ref_radps: f64,
}

/// A chain of spinning bodies (upstream `SpinningBodyNDOFStateEffector`).
#[derive(Debug)]
pub struct SpinningBodyNDofStateEffector {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `motorTorqueInMsg` (optional): entry `i` is body `i`'s motor
    /// torque.
    pub motor_torque_in: Input<ArrayMotorTorqueMsg>,
    /// Upstream `motorLockInMsg` (optional): entry `i` locks body `i`'s axis.
    pub motor_lock_in: Input<ArrayEffectorLockMsg>,
    /// Upstream `spinningBodyRefInMsgs` (optional): reference angle and rate
    /// of each joint, in chain order.
    pub spinning_body_ref_in: Vec<Input<HingedRigidBodyMsg>>,
    /// Upstream `spinningBodyOutMsgs`: angle and rate of each joint.
    pub spinning_body_out: Vec<Output<HingedRigidBodyMsg>>,
    /// Upstream `spinningBodyConfigLogOutMsgs`: inertial state of each body,
    /// with its frame as body frame and its center of mass as position.
    pub spinning_body_state_out: Vec<Output<SpacecraftStateMsg>>,
    bodies: Vec<SpinningBody>,
    commands: Vec<SpinningJointCommand>,
    /// Last loaded `[θ1 … θN, θ̇1 … θ̇N]`.
    state: Vec<f64>,
    back_sub: RefCell<NDofTerms>,
    kinematics: KinematicsCache<Vec<SpinningBody>, Vec<SpinnerKinematics>>,
}

impl SpinningBodyNDofStateEffector {
    /// An effector without bodies; add them with
    /// [`add_spinning_body`](Self::add_spinning_body).
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            motor_torque_in: Input::default(),
            motor_lock_in: Input::default(),
            spinning_body_ref_in: Vec::new(),
            spinning_body_out: Vec::new(),
            spinning_body_state_out: Vec::new(),
            bodies: Vec::new(),
            commands: Vec::new(),
            state: Vec::new(),
            back_sub: RefCell::new(NDofTerms::default()),
            kinematics: KinematicsCache::default(),
        }
    }

    /// Appends `body` to the outboard end of the chain, with its input and
    /// output messages (upstream `addSpinningBody`).
    pub fn add_spinning_body(&mut self, body: SpinningBody) {
        self.bodies.push(body);
        self.commands.push(SpinningJointCommand::default());
        self.spinning_body_ref_in.push(Input::default());
        self.spinning_body_out.push(Output::default());
        self.spinning_body_state_out.push(Output::default());
        self.state = self.configured_state();
    }

    /// Body `index` of the chain, counting from the hub (upstream
    /// `getSpinningBody`).
    pub fn spinning_body(&self, index: usize) -> &SpinningBody {
        &self.bodies[index]
    }

    /// Mutable access to body `index`, to change its parameters after it was
    /// added (upstream `getSpinningBody` returns a shared pointer).
    pub fn spinning_body_mut(&mut self, index: usize) -> &mut SpinningBody {
        &mut self.bodies[index]
    }

    /// The bodies, from the hub outward.
    pub fn spinning_bodies(&self) -> &[SpinningBody] {
        &self.bodies
    }

    /// The joint commands read at the start of the last step.
    pub fn commands(&self) -> &[SpinningJointCommand] {
        &self.commands
    }

    /// `[θ1 … θN, θ̇1 … θ̇N]` of the last loaded state.
    pub fn state(&self) -> &[f64] {
        &self.state
    }

    /// Upstream `validateConfiguration` (with `validateCommandCapacity`) and
    /// the checks of upstream's `SpinningBody` setters: at most
    /// [`MAX_EFF_COUNT`] bodies when a torque or lock array is connected, at
    /// least one body, the outermost with mass, every value finite, masses,
    /// spring constants and dampers non-negative, spin axis norms above 0.01,
    /// every `parent_to_spinner0_dcm` a rotation matrix and, for a body with
    /// mass, a valid inertia tensor.
    pub fn validate(&self) -> Result<(), String> {
        self.validate_command_capacity()?;
        let Some(last) = self.bodies.last() else {
            return Err("at least one spinning body is required".to_string());
        };
        if last.mass_kg.is_nan() || last.mass_kg <= 0.0 {
            return Err("the mass of the last spinning body must be greater than 0".to_string());
        }
        let mut total_mass_kg = 0.0;
        for (index, body) in self.bodies.iter().enumerate() {
            for (label, value) in [
                ("mass_kg", body.mass_kg),
                (
                    "spring_constant_nm_per_rad",
                    body.spring_constant_nm_per_rad,
                ),
                ("damping_nm_s_per_rad", body.damping_nm_s_per_rad),
                ("theta_init_rad", body.theta_init_rad),
                ("theta_dot_init_radps", body.theta_dot_init_radps),
            ] {
                if !value.is_finite() {
                    return Err(format!("spinning body {index}: {label} must be finite"));
                }
            }
            for (label, finite) in [
                (
                    "spin_axis_spinner",
                    all_finite(body.spin_axis_spinner.as_slice()),
                ),
                (
                    "origin_parent_m",
                    all_finite(body.origin_parent_m.as_slice()),
                ),
                (
                    "center_of_mass_spinner_m",
                    all_finite(body.center_of_mass_spinner_m.as_slice()),
                ),
                (
                    "inertia_about_com_spinner_kg_m2",
                    all_finite(body.inertia_about_com_spinner_kg_m2.as_slice()),
                ),
            ] {
                if !finite {
                    return Err(format!(
                        "spinning body {index}: {label} must contain only finite values"
                    ));
                }
            }
            for (label, value) in [
                ("mass_kg", body.mass_kg),
                (
                    "spring_constant_nm_per_rad",
                    body.spring_constant_nm_per_rad,
                ),
                ("damping_nm_s_per_rad", body.damping_nm_s_per_rad),
            ] {
                if value < 0.0 {
                    return Err(format!(
                        "spinning body {index}: {label} must be greater than or equal to 0"
                    ));
                }
            }
            if body.spin_axis_spinner.norm() <= 0.01 {
                return Err(format!(
                    "spinning body {index}: the norm of spin_axis_spinner must exceed 0.01"
                ));
            }
            if !is_rotation_matrix(&body.parent_to_spinner0_dcm, DEFAULT_TOLERANCE) {
                return Err(format!(
                    "spinning body {index}: parent_to_spinner0_dcm must be an orthogonal, \
                     right-handed rotation matrix"
                ));
            }
            if body.mass_kg > 0.0
                && !is_valid_inertia_matrix(
                    &body.inertia_about_com_spinner_kg_m2,
                    DEFAULT_TOLERANCE,
                )
            {
                return Err(format!(
                    "spinning body {index}: inertia_about_com_spinner_kg_m2 must be a valid \
                     inertia tensor"
                ));
            }
            total_mass_kg += body.mass_kg;
        }
        if !total_mass_kg.is_finite() {
            return Err("the combined mass must be finite".to_string());
        }
        Ok(())
    }

    /// Upstream `validateCommandCapacity`.
    fn validate_command_capacity(&self) -> Result<(), String> {
        if self.bodies.len() > MAX_EFF_COUNT
            && (self.motor_lock_in.is_connected() || self.motor_torque_in.is_connected())
        {
            return Err(format!(
                "the number of degrees of freedom ({}) exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}) \
                 with a connected motor_lock_in or motor_torque_in",
                self.bodies.len()
            ));
        }
        Ok(())
    }

    /// Inertial state of every body (upstream
    /// `computeSpinningBodyInertialStates`) for the last loaded state and the
    /// given hub state.
    pub fn spinner_inertial_states(
        &self,
        hub_state: &SpacecraftStateMsg,
    ) -> Vec<SpacecraftStateMsg> {
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        self.kinematics(&self.state)
            .iter()
            .map(|k| SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_nb * k.r_scb_b,
                velocity_mps: hub_state.velocity_mps
                    + dcm_nb * (k.r_prime_scb_b + omega_bn_b.cross(&k.r_scb_b)),
                sigma_bn: dcm_to_mrp(k.dcm_bs.transpose() * dcm_bn),
                omega_radps: k.dcm_bs.transpose() * (k.omega_sb_b + omega_bn_b),
                ..Default::default()
            })
            .collect()
    }

    fn configured_state(&self) -> Vec<f64> {
        self.bodies
            .iter()
            .map(|body| body.theta_init_rad)
            .chain(self.bodies.iter().map(|body| body.theta_dot_init_radps))
            .collect()
    }

    /// Upstream `computeAttitudeProperties`, `computeAngularVelocityProperties`,
    /// `computePositionProperties`, `computeVelocityProperties` and
    /// `computeInertiaProperties` for every body.
    fn kinematics(&self, effector_state: &[f64]) -> Vec<SpinnerKinematics> {
        let n = self.bodies.len();
        assert_eq!(
            effector_state.len(),
            2 * n,
            "spinning body N DOF state length mismatch"
        );
        let mut key = effector_state.to_vec();
        for (rate, command) in key[n..].iter_mut().zip(&self.commands) {
            if command.locked {
                *rate = 0.0;
            }
        }
        self.kinematics
            .get_or_compute(&key, &self.bodies, || self.chain_kinematics(&key))
    }

    fn chain_kinematics(&self, effector_state: &[f64]) -> Vec<SpinnerKinematics> {
        let n = self.bodies.len();
        let (theta, theta_dot) = effector_state.split_at(n);
        let mut chain: Vec<SpinnerKinematics> = Vec::with_capacity(n);
        for (index, body) in self.bodies.iter().enumerate() {
            let theta_dot = theta_dot[index];
            let s_hat_s = normalized_axis(body.spin_axis_spinner);
            let dcm_ps =
                body.parent_to_spinner0_dcm.transpose() * prv_to_dcm(-theta[index] * s_hat_s);
            let (dcm_bp, omega_pb_b, r_pb_b, r_prime_pb_b) = match chain.last() {
                Some(parent) => (
                    parent.dcm_bs,
                    parent.omega_sb_b,
                    parent.r_sb_b,
                    parent.r_prime_sb_b,
                ),
                None => (
                    Matrix3::identity(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Vector3::zeros(),
                ),
            };
            let dcm_bs = dcm_bp * dcm_ps;
            let s_hat_b = dcm_bs * s_hat_s;
            let omega_sp_b = theta_dot * s_hat_b;
            let omega_sb_b = omega_sp_b + omega_pb_b;
            let r_scs_b = dcm_bs * body.center_of_mass_spinner_m;
            let r_sp_b = dcm_bp * body.origin_parent_m;
            let r_sb_b = r_sp_b + r_pb_b;
            let r_prime_scs_b = omega_sb_b.cross(&r_scs_b);
            let r_prime_sb_b = omega_pb_b.cross(&r_sp_b) + r_prime_pb_b;
            let i_pnt_sc_b = dcm_bs * body.inertia_about_com_spinner_kg_m2 * dcm_bs.transpose();
            let omega_tilde_sb_b = tilde(omega_sb_b);
            chain.push(SpinnerKinematics {
                theta: theta[index],
                theta_dot,
                dcm_bs,
                s_hat_b,
                omega_sp_b,
                omega_sb_b,
                r_sb_b,
                r_scb_b: r_scs_b + r_sb_b,
                r_prime_scs_b,
                r_prime_sb_b,
                r_prime_scb_b: r_prime_scs_b + r_prime_sb_b,
                i_pnt_sc_b,
                i_prime_pnt_sc_b: omega_tilde_sb_b * i_pnt_sc_b - i_pnt_sc_b * omega_tilde_sb_b,
            });
        }
        chain
    }

    /// Upstream `updateContributions`, with 0-based body indices.
    fn back_sub_terms(
        &self,
        k: &[SpinnerKinematics],
        omega_bn_b: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> NDofTerms {
        let n = k.len();
        let mass = |index: usize| self.bodies[index].mass_kg;
        let locked = |index: usize| self.commands[index].locked;
        let omega_sn_b: Vec<Vector3<f64>> = k.iter().map(|ki| ki.omega_sb_b + omega_bn_b).collect();
        // Upstream's `linkAcceleration` of body i through joint j < i, and the
        // relative rate `omega_SiSj_B`.
        let link_terms = |i: usize, j: usize| {
            let omega_si_sj = k[i].omega_sb_b - k[j].omega_sb_b;
            let r_sci_sj1 = k[i].r_scb_b - k[j + 1].r_sb_b;
            let r_prime_sci_sj = k[i].r_prime_scb_b - k[j].r_prime_sb_b;
            let link_accel = k[j].omega_sp_b.cross(&r_prime_sci_sj)
                - r_sci_sj1.cross(&k[j].omega_sb_b.cross(&k[j + 1].omega_sp_b));
            (omega_si_sj, link_accel)
        };

        // Upstream `computeMTheta`.
        let mut m_theta = DMatrix::<f64>::zeros(n, n);
        for row in 0..n {
            for col in 0..n {
                if (locked(row) || locked(col)) && row != col {
                    continue;
                }
                for j in row.max(col)..n {
                    let r_scj_srow = k[j].r_scb_b - k[row].r_sb_b;
                    let r_scj_scol = k[j].r_scb_b - k[col].r_sb_b;
                    let contribution = k[j].i_pnt_sc_b * k[col].s_hat_b
                        - mass(j) * r_scj_srow.cross(&r_scj_scol.cross(&k[col].s_hat_b));
                    m_theta[(row, col)] += k[row].s_hat_b.dot(&contribution);
                }
            }
        }

        // Upstream `computeAThetaStar`, `computeBThetaStar` and
        // `computeCThetaStar`.
        let mut a_theta_star = MatrixXx3::<f64>::zeros(n);
        let mut b_theta_star = MatrixXx3::<f64>::zeros(n);
        let mut c_theta_star = DVector::<f64>::zeros(n);
        for row in 0..n {
            if locked(row) {
                continue;
            }
            let s_hat = k[row].s_hat_b;
            let body = &self.bodies[row];
            let command = &self.commands[row];
            let mut c = command.motor_torque_nm
                - body.spring_constant_nm_per_rad * (k[row].theta - command.theta_ref_rad)
                - body.damping_nm_s_per_rad * (k[row].theta_dot - command.theta_dot_ref_radps);
            let mut a = Vector3::zeros();
            let mut b = Vector3::zeros();
            for i in row..n {
                let r_sci_srow = k[i].r_scb_b - k[row].r_sb_b;
                a -= mass(i) * s_hat.cross(&r_sci_srow);
                b -= k[i].i_pnt_sc_b * s_hat
                    - mass(i) * k[i].r_scb_b.cross(&r_sci_srow.cross(&s_hat));
                let coriolis_translation = -gravity_body_mps2
                    + omega_bn_b.cross(&omega_bn_b.cross(&k[i].r_scb_b))
                    + 2.0 * omega_bn_b.cross(&k[i].r_prime_scb_b)
                    + k[i].omega_sp_b.cross(&k[i].r_prime_scs_b);
                let contribution = omega_sn_b[i].cross(&(k[i].i_pnt_sc_b * omega_sn_b[i]))
                    - k[i].i_pnt_sc_b * k[i].omega_sb_b.cross(&omega_bn_b)
                    + mass(i) * r_sci_srow.cross(&coriolis_translation);
                c -= s_hat.dot(&contribution);
                for j in 0..i {
                    let (omega_si_sj, link_accel) = link_terms(i, j);
                    let inner = -k[i].i_pnt_sc_b * omega_si_sj.cross(&k[j].omega_sp_b)
                        + mass(i) * r_sci_srow.cross(&link_accel);
                    c -= s_hat.dot(&inner);
                }
            }
            a_theta_star.set_row(row, &a.transpose());
            b_theta_star.set_row(row, &b.transpose());
            c_theta_star[row] = c;
        }

        let m_theta_inverse = m_theta
            .try_inverse()
            .expect("spinning body N DOF joint mass matrix must be invertible");
        let a_theta: MatrixXx3<f64> = &m_theta_inverse * a_theta_star;
        let b_theta: MatrixXx3<f64> = &m_theta_inverse * b_theta_star;
        let c_theta: DVector<f64> = &m_theta_inverse * c_theta_star;

        // Upstream `computeBackSubMatrices` and `computeBackSubVectors`.
        let mut back_sub = BackSubMatrices::default();
        for i in 0..n {
            back_sub.vec_rot -= omega_sn_b[i].cross(&(k[i].i_pnt_sc_b * k[i].omega_sb_b))
                + mass(i) * omega_bn_b.cross(&k[i].r_scb_b.cross(&k[i].r_prime_scb_b));
            for j in 0..i {
                let (omega_si_sj, link_accel) = link_terms(i, j);
                back_sub.vec_trans -= mass(i) * link_accel;
                back_sub.vec_rot -= -k[i].i_pnt_sc_b * omega_si_sj.cross(&k[j].omega_sp_b)
                    + mass(i) * k[i].r_scb_b.cross(&link_accel);
            }
            let tip_accel = k[i].omega_sp_b.cross(&k[i].r_prime_scs_b);
            back_sub.vec_trans -= mass(i) * tip_accel;
            back_sub.vec_rot -= mass(i) * k[i].r_scb_b.cross(&tip_accel);

            for j in i..n {
                let r_scj_si = k[j].r_scb_b - k[i].r_sb_b;
                let ab = r_scj_si.cross(&k[i].s_hat_b);
                let cd = k[j].i_pnt_sc_b * k[i].s_hat_b
                    - mass(j) * k[j].r_scb_b.cross(&r_scj_si.cross(&k[i].s_hat_b));
                back_sub.matrix_a -= mass(j) * ab * a_theta.row(i);
                back_sub.matrix_b -= mass(j) * ab * b_theta.row(i);
                back_sub.matrix_c += cd * a_theta.row(i);
                back_sub.matrix_d += cd * b_theta.row(i);
                back_sub.vec_trans += mass(j) * ab * c_theta[i];
                back_sub.vec_rot -= cd * c_theta[i];
            }
        }

        NDofTerms {
            back_sub,
            a_theta,
            b_theta,
            c_theta,
        }
    }
}

impl StateEffector for SpinningBodyNDofStateEffector {
    fn name(&self) -> &str {
        &self.name
    }

    fn state_len(&self) -> usize {
        2 * self.bodies.len()
    }

    /// Validates the chain (upstream `registerStates`) and returns the initial
    /// `[θ1 … θN, θ̇1 … θ̇N]`, with the rates of axes that `motor_lock_in`
    /// already locks zeroed.
    ///
    /// # Panics
    ///
    /// If [`SpinningBodyNDofStateEffector::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.validate() {
            panic!(
                "invalid SpinningBodyNDofStateEffector '{}': {message}",
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
            "spinning body N DOF state length mismatch"
        );
        self.state.clear();
        self.state.extend_from_slice(state);
    }

    /// Upstream `readInputMessages`.
    ///
    /// # Panics
    ///
    /// If a torque or lock array was connected after initialization to a chain
    /// longer than [`MAX_EFF_COUNT`].
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        if let Err(message) = self.validate_command_capacity() {
            panic!("SpinningBodyNDofStateEffector '{}': {message}", self.name);
        }
        if self.motor_torque_in.is_connected() {
            let torque = self.motor_torque_in.read().motor_torque_nm;
            for (command, torque_nm) in self.commands.iter_mut().zip(torque) {
                command.motor_torque_nm = torque_nm;
            }
        }
        if self.motor_lock_in.is_connected() {
            let lock = self.motor_lock_in.read().effector_lock;
            for (command, lock) in self.commands.iter_mut().zip(lock) {
                command.locked = lock == EffectorLock::Locked;
            }
        }
        for (command, input) in self.commands.iter_mut().zip(&self.spinning_body_ref_in) {
            if input.is_connected() {
                let reference = input.read();
                command.theta_ref_rad = reference.theta_rad;
                command.theta_dot_ref_radps = reference.theta_dot_radps;
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
        let kinematics = self.kinematics(effector_state);
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
        let kinematics = self.kinematics(effector_state);
        let terms = self.back_sub.borrow();
        let theta_ddot = &terms.a_theta * body_trans_accel_mps2
            + &terms.b_theta * body_omega_dot_radps2
            + &terms.c_theta;
        kinematics
            .iter()
            .map(|k| k.theta_dot)
            .chain(theta_ddot.iter().copied())
            .collect()
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let kinematics = self.kinematics(effector_state);
        let mut props = StateEffectorMassProps::default();
        for (body, k) in self.bodies.iter().zip(&kinematics) {
            let m = body.mass_kg;
            let r_tilde = tilde(k.r_scb_b);
            let r_prime_tilde = tilde(k.r_prime_scb_b);
            props.mass_kg += m;
            props.center_of_mass_body_m += m * k.r_scb_b;
            props.center_of_mass_prime_body_mps += m * k.r_prime_scb_b;
            props.inertia_about_point_b_body_kg_m2 += k.i_pnt_sc_b - m * r_tilde * r_tilde;
            props.inertia_about_point_b_body_prime_kg_m2ps +=
                k.i_prime_pnt_sc_b - m * (r_prime_tilde * r_tilde + r_tilde * r_prime_tilde);
        }
        props.center_of_mass_body_m /= props.mass_kg;
        props.center_of_mass_prime_body_mps /= props.mass_kg;
        props
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.bodies
            .iter()
            .zip(self.kinematics(effector_state))
            .map(|(body, k)| {
                let omega_sn_b = k.omega_sb_b + body_omega_radps;
                let r_dot_scb_b = k.r_prime_scb_b + body_omega_radps.cross(&k.r_scb_b);
                k.i_pnt_sc_b * omega_sn_b + body.mass_kg * k.r_scb_b.cross(&r_dot_scb_b)
            })
            .sum()
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the spring energies.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.bodies
            .iter()
            .zip(&self.commands)
            .zip(self.kinematics(effector_state))
            .map(|((body, command), k)| {
                let omega_sn_b = k.omega_sb_b + body_omega_radps;
                let r_dot_scb_b = k.r_prime_scb_b + body_omega_radps.cross(&k.r_scb_b);
                let deflection_rad = k.theta - command.theta_ref_rad;
                0.5 * omega_sn_b.dot(&(k.i_pnt_sc_b * omega_sn_b))
                    + 0.5 * body.mass_kg * r_dot_scb_b.norm_squared()
                    + 0.5 * body.spring_constant_nm_per_rad * deflection_rad * deflection_rad
            })
            .sum()
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        let n = self.bodies.len();
        for (index, output) in self.spinning_body_out.iter().enumerate() {
            output.write(HingedRigidBodyMsg {
                theta_rad: self.state[index],
                theta_dot_radps: self.state[n + index],
            });
        }
        for (output, state) in self
            .spinning_body_state_out
            .iter()
            .zip(self.spinner_inertial_states(hub_state))
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
/// `θ̈ = A r̈_B + B ω̇ + C` cached for `compute_derivatives`.
#[derive(Clone, Debug)]
struct NDofTerms {
    back_sub: BackSubMatrices,
    /// Upstream `ATheta`.
    a_theta: MatrixXx3<f64>,
    /// Upstream `BTheta`.
    b_theta: MatrixXx3<f64>,
    /// Upstream `CTheta`.
    c_theta: DVector<f64>,
}

impl Default for NDofTerms {
    fn default() -> Self {
        Self {
            back_sub: BackSubMatrices::default(),
            a_theta: MatrixXx3::zeros(0),
            b_theta: MatrixXx3::zeros(0),
            c_theta: DVector::zeros(0),
        }
    }
}

/// One body's geometry for a given chain state (upstream `SpinningBody`'s
/// derived members).
#[derive(Clone, Debug)]
struct SpinnerKinematics {
    theta: f64,
    /// θ̇, zero for a locked axis.
    theta_dot: f64,
    /// Upstream `dcm_BS`.
    dcm_bs: Matrix3<f64>,
    /// Upstream `sHat_B`.
    s_hat_b: Vector3<f64>,
    /// Upstream `omega_SP_B`.
    omega_sp_b: Vector3<f64>,
    /// Upstream `omega_SB_B`.
    omega_sb_b: Vector3<f64>,
    /// Upstream `r_SB_B`.
    r_sb_b: Vector3<f64>,
    /// Upstream `r_ScB_B`.
    r_scb_b: Vector3<f64>,
    /// Upstream `rPrime_ScS_B`.
    r_prime_scs_b: Vector3<f64>,
    /// Upstream `rPrime_SB_B`.
    r_prime_sb_b: Vector3<f64>,
    /// Upstream `rPrime_ScB_B`.
    r_prime_scb_b: Vector3<f64>,
    /// Upstream `ISPntSc_B`.
    i_pnt_sc_b: Matrix3<f64>,
    /// Upstream `IPrimeSPntSc_B`.
    i_prime_pnt_sc_b: Matrix3<f64>,
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_spinningBodyNDOFStateEffector.py`,
    //! `test_spinningBodyNDOFResetChecks.py` and
    //! `test_changeParametersSpinningBodyNDOF.py`.
    //!
    //! The integrated tests draw their masses, inertias, offsets, initial
    //! angles and spring constants from `np.random` without a seed; here they
    //! come from a seeded generator over the same ranges. The reset checks'
    //! "reset" and "attachment" paths are one check here,
    //! [`SpinningBodyNDofStateEffector::validate`].

    use super::{SpinningBody, SpinningBodyNDofStateEffector};
    use crate::dynamics::spinning_body_two_dof_state_effector::{
        SpinningBodyTwoDofConfig, SpinningBodyTwoDofStateEffector,
    };
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, assert_relative_equal,
        run_logging_every_step, spacecraft_with_effectors,
    };
    use crate::messages::{
        ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, MAX_EFF_COUNT, Output,
    };
    use crate::spacecraft::StateEffector;
    use nalgebra::{Matrix3, Vector3};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};
    use std::f64::consts::FRAC_1_SQRT_2;

    const D2R: f64 = std::f64::consts::PI / 180.0;

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

    /// The four-body chain of the integrated tests; `random_rates` draws the
    /// initial rates as the lock and torque cases do.
    fn upstream_chain(seed: u64, random_rates: bool) -> SpinningBodyNDofStateEffector {
        let mut rng = StdRng::seed_from_u64(seed);
        let mut effector = SpinningBodyNDofStateEffector::new("spinningBodyEffector");
        let frames = [
            (
                Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
                Vector3::new(0.0, 0.0, 1.0),
            ),
            (
                Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0),
                Vector3::new(0.0, -1.0, 0.0),
            ),
            (
                Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, -1.0)),
                Vector3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0),
            ),
            (
                Matrix3::new(0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 0.0),
                Vector3::new(FRAC_1_SQRT_2, -FRAC_1_SQRT_2, 0.0),
            ),
        ];
        for (dcm, axis) in frames {
            let mass_kg = rng.gen_range(5.0..50.0);
            let inertia = random_valid_inertia(&mut rng);
            let center_of_mass = random_vector(&mut rng);
            let origin = random_vector(&mut rng);
            let theta_init_rad = rng.gen_range(-10.0..10.0) * D2R;
            let theta_dot_init_radps = if random_rates {
                rng.gen_range(-1.0..1.0) * D2R
            } else {
                0.0
            };
            effector.add_spinning_body(SpinningBody {
                mass_kg,
                spring_constant_nm_per_rad: rng.r#gen(),
                damping_nm_s_per_rad: 0.0,
                theta_init_rad,
                theta_dot_init_radps,
                origin_parent_m: origin,
                center_of_mass_spinner_m: center_of_mass,
                spin_axis_spinner: axis,
                inertia_about_com_spinner_kg_m2: inertia,
                parent_to_spinner0_dcm: dcm,
            });
        }
        effector
    }

    /// The integrated tests: 1 s at a 0.1 ms step, conservation to 1e-13.
    fn integrated_test(effector: SpinningBodyNDofStateEffector, check_rotational_energy: bool) {
        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        hub.omega_bn_b_radps = Vector3::new(0.1, -0.1, 0.1);
        let mut spacecraft = spacecraft_with_effectors(&hub, 100_000, vec![Box::new(effector)]);
        add_point_mass_earth(&mut spacecraft);

        let log = run_logging_every_step(&mut spacecraft, 10_000, || {});

        assert_conserved(&log[0], &log[log.len() - 1], 1e-13, check_rotational_energy);
    }

    /// Upstream `spinningBodyNoInput`.
    #[test]
    fn spinning_body_no_input() {
        integrated_test(upstream_chain(1, false), true);
    }

    /// Upstream `spinningBodyLockAxis`: bodies 1 and 4 locked.
    #[test]
    fn spinning_body_lock_axis() {
        let mut effector = upstream_chain(2, true);
        let mut lock = ArrayEffectorLockMsg::default();
        lock.effector_lock[0] = EffectorLock::Locked;
        lock.effector_lock[3] = EffectorLock::Locked;
        let lock = Output::new(lock);
        effector.motor_lock_in.connect(lock.slot());
        let initial = effector.initial_state();
        assert_eq!((initial[4], initial[7]), (0.0, 0.0));
        assert_ne!(initial[5], 0.0);
        integrated_test(effector, true);
    }

    /// Upstream `spinningBodyCommandedTorque`.
    #[test]
    fn spinning_body_commanded_torque() {
        let mut effector = upstream_chain(3, true);
        let torque = Output::new(ArrayMotorTorqueMsg::from_active(&[0.1, -0.2, 0.3, -0.15]));
        effector.motor_torque_in.connect(torque.slot());
        integrated_test(effector, false);
    }

    /// Upstream `changeParameters`.
    #[test]
    fn change_parameters_of_an_added_body() {
        let mut effector = SpinningBodyNDofStateEffector::new("chain");
        effector.add_spinning_body(SpinningBody::default());
        let expected = SpinningBody {
            mass_kg: 20.0,
            spring_constant_nm_per_rad: 152.0,
            damping_nm_s_per_rad: 432.0,
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            origin_parent_m: Vector3::new(0.199, 0.624, 0.771),
            center_of_mass_spinner_m: Vector3::new(0.412, 0.623, 0.456),
            spin_axis_spinner: Vector3::x(),
            inertia_about_com_spinner_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                767.0, 912.0, 360.0,
            )),
            parent_to_spinner0_dcm: Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
        };
        *effector.spinning_body_mut(0) = expected.clone();
        assert_eq!(effector.spinning_body(0), &expected);
    }

    /// Upstream `_validBody` of the reset checks.
    fn valid_body() -> SpinningBody {
        SpinningBody {
            mass_kg: 50.0,
            inertia_about_com_spinner_kg_m2: Matrix3::from_diagonal(&Vector3::new(
                100.0, 50.0, 50.0,
            )),
            parent_to_spinner0_dcm: Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
            spin_axis_spinner: Vector3::z(),
            ..SpinningBody::default()
        }
    }

    fn chain_of(bodies: Vec<SpinningBody>) -> SpinningBodyNDofStateEffector {
        let mut effector = SpinningBodyNDofStateEffector::new("spinningBodyEffector");
        for body in bodies {
            effector.add_spinning_body(body);
        }
        effector
    }

    /// Upstream `test_spinningBodyNDOF_rejectsInvalidConfiguration`, the
    /// setter checks and `test_spinningBodyNDOF_rejectsEmptyChain`.
    #[test]
    fn rejects_invalid_configuration() {
        type Break = fn(&mut SpinningBody);
        let cases: [(&str, &str, Break); 7] = [
            ("dcm_S0P not orthogonal", "parent_to_spinner0_dcm", |b| {
                b.parent_to_spinner0_dcm = Matrix3::new(1.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
            }),
            ("dcm_S0P left-handed", "parent_to_spinner0_dcm", |b| {
                b.parent_to_spinner0_dcm = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0))
            }),
            (
                "ISPntSc_S not symmetric",
                "inertia_about_com_spinner",
                |b| {
                    b.inertia_about_com_spinner_kg_m2 =
                        Matrix3::new(100.0, 5.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0)
                },
            ),
            (
                "ISPntSc_S not positive definite",
                "inertia_about_com_spinner",
                |b| {
                    b.inertia_about_com_spinner_kg_m2 =
                        Matrix3::from_diagonal(&Vector3::new(-100.0, 50.0, 50.0))
                },
            ),
            ("sHat_S too short", "spin_axis_spinner", |b| {
                b.spin_axis_spinner = Vector3::new(0.0, 0.0, 0.001)
            }),
            ("negative spring constant", "spring_constant", |b| {
                b.spring_constant_nm_per_rad = -1.0
            }),
            ("massless last body", "mass of the last", |b| {
                b.mass_kg = 0.0
            }),
        ];
        for (case, expected, break_it) in cases {
            let mut body = valid_body();
            break_it(&mut body);
            let result = chain_of(vec![body]).validate();
            assert!(
                result
                    .as_ref()
                    .is_err_and(|message| message.contains(expected)),
                "{case}: expected an error naming {expected}, got {result:?}"
            );
        }
        let empty = chain_of(Vec::new()).validate();
        assert!(empty.is_err_and(|message| message.contains("at least one spinning body")));
    }

    #[test]
    #[should_panic(expected = "at least one spinning body")]
    fn spacecraft_initialization_rejects_empty_chain() {
        let _ = chain_of(Vec::new()).initial_state();
    }

    /// Upstream `test_spinningBodyNDOF_resetAcceptsValidConfig` and
    /// `test_spinningBodyNDOF_resetAcceptsMasslessConnector`.
    #[test]
    fn accepts_valid_chain_and_massless_connector() {
        assert_eq!(chain_of(vec![valid_body()]).validate(), Ok(()));
        let connector = SpinningBody {
            mass_kg: 0.0,
            inertia_about_com_spinner_kg_m2: Matrix3::zeros(),
            ..SpinningBody::default()
        };
        assert_eq!(chain_of(vec![connector, valid_body()]).validate(), Ok(()));
    }

    /// Upstream `test_command_array_capacity_validation`.
    #[test]
    fn command_array_capacity_validation() {
        for extra_bodies in [0, 1] {
            for inputs in ["none", "lock", "torque", "both"] {
                let mut effector = chain_of(vec![valid_body(); MAX_EFF_COUNT + extra_bodies]);
                let lock = Output::new(ArrayEffectorLockMsg::default());
                let torque = Output::new(ArrayMotorTorqueMsg::default());
                if matches!(inputs, "lock" | "both") {
                    effector.motor_lock_in.connect(lock.slot());
                }
                if matches!(inputs, "torque" | "both") {
                    effector.motor_torque_in.connect(torque.slot());
                }
                let result = effector.validate();
                if extra_bodies == 1 && inputs != "none" {
                    assert!(
                        result
                            .as_ref()
                            .is_err_and(|message| message.contains("MAX_EFF_COUNT")),
                        "{inputs}: {result:?}"
                    );
                } else {
                    assert_eq!(result, Ok(()), "{extra_bodies} extra, {inputs}");
                }
            }
        }
    }

    /// Upstream `test_command_array_linked_after_initialization`.
    #[test]
    #[should_panic(expected = "MAX_EFF_COUNT")]
    fn command_array_connected_after_initialization() {
        let mut effector = chain_of(vec![valid_body(); MAX_EFF_COUNT + 1]);
        let _ = effector.initial_state();
        let torque = Output::new(ArrayMotorTorqueMsg::default());
        effector.motor_torque_in.connect(torque.slot());
        effector.pre_integration(0, 0.01);
    }

    /// Upstream `test_command_array_last_entry`: the last entries of full
    /// torque and lock arrays drive the last of `MAX_EFF_COUNT` bodies.
    #[test]
    fn command_array_last_entry() {
        for lock_last in [false, true] {
            let mut effector = chain_of(vec![valid_body(); MAX_EFF_COUNT]);
            let mut lock = ArrayEffectorLockMsg {
                effector_lock: [EffectorLock::Locked; MAX_EFF_COUNT],
            };
            if !lock_last {
                lock.effector_lock[MAX_EFF_COUNT - 1] = EffectorLock::Unlocked;
            }
            let lock = Output::new(lock);
            let mut torque = ArrayMotorTorqueMsg::default();
            torque.motor_torque_nm[MAX_EFF_COUNT - 1] = 0.1;
            let torque = Output::new(torque);
            effector.motor_lock_in.connect(lock.slot());
            effector.motor_torque_in.connect(torque.slot());
            let last = effector.spinning_body_out[MAX_EFF_COUNT - 1].clone();

            let mut spacecraft = spacecraft_with_effectors(
                &HubSetup::hinged_body_hub(),
                10_000_000,
                vec![Box::new(effector)],
            );
            run_logging_every_step(&mut spacecraft, 2, || {});

            let last = last.read();
            if lock_last {
                assert_eq!((last.theta_rad, last.theta_dot_radps), (0.0, 0.0));
            } else {
                assert!(
                    last.theta_rad > 0.0 && last.theta_dot_radps > 0.0,
                    "{last:?}"
                );
            }
        }
    }

    /// A two-body chain and the equivalent two-DOF effector agree on the
    /// back-substitution terms and the joint accelerations.
    #[test]
    fn two_body_chain_matches_two_dof_effector() {
        let mut rng = StdRng::seed_from_u64(9);
        let first = SpinningBody {
            mass_kg: 40.0,
            inertia_about_com_spinner_kg_m2: random_valid_inertia(&mut rng),
            parent_to_spinner0_dcm: Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0)),
            spin_axis_spinner: Vector3::new(0.0, 0.0, 1.0),
            center_of_mass_spinner_m: random_vector(&mut rng),
            origin_parent_m: random_vector(&mut rng),
            spring_constant_nm_per_rad: 3.0,
            ..SpinningBody::default()
        };
        let second = SpinningBody {
            mass_kg: 25.0,
            inertia_about_com_spinner_kg_m2: random_valid_inertia(&mut rng),
            parent_to_spinner0_dcm: Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0),
            spin_axis_spinner: Vector3::new(0.0, -1.0, 0.0),
            center_of_mass_spinner_m: random_vector(&mut rng),
            origin_parent_m: random_vector(&mut rng),
            spring_constant_nm_per_rad: 7.0,
            ..SpinningBody::default()
        };
        let mut pair = SpinningBodyTwoDofConfig::new("pair");
        for (config, body) in [(&mut pair.body1, &first), (&mut pair.body2, &second)] {
            config.mass_kg = body.mass_kg;
            config.inertia_about_com_spinner_kg_m2 = body.inertia_about_com_spinner_kg_m2;
            config.spin_axis_spinner = body.spin_axis_spinner;
            config.center_of_mass_spinner_m = body.center_of_mass_spinner_m;
            config.spring_constant_nm_per_rad = body.spring_constant_nm_per_rad;
        }
        pair.body_to_spinner1_dcm = first.parent_to_spinner0_dcm;
        pair.spinner1_to_spinner2_dcm = second.parent_to_spinner0_dcm;
        pair.spinner1_origin_body_m = first.origin_parent_m;
        pair.spinner2_origin_spinner1_m = second.origin_parent_m;
        let two_dof = SpinningBodyTwoDofStateEffector::new(pair);
        let chain = chain_of(vec![first, second]);

        let (theta, theta_dot) = ([0.2, -0.3], [0.05, -0.08]);
        let chain_state = [theta[0], theta[1], theta_dot[0], theta_dot[1]];
        let pair_state = [theta[0], theta_dot[0], theta[1], theta_dot[1]];
        let omega = Vector3::new(0.1, -0.2, 0.15);
        let gravity = Vector3::new(0.3, -0.1, 0.2);
        let mut chain_back_sub = Default::default();
        let mut pair_back_sub = Default::default();
        chain.update_contributions(&chain_state, omega, gravity, &mut chain_back_sub);
        two_dof.update_contributions(&pair_state, omega, gravity, &mut pair_back_sub);
        let accel = Vector3::new(-0.2, 0.4, 0.1);
        let omega_dot = Vector3::new(0.01, 0.03, -0.02);
        let chain_dot = chain.compute_derivatives(&chain_state, accel, omega_dot);
        let pair_dot = two_dof.compute_derivatives(&pair_state, accel, omega_dot);

        let tolerance = 1e-10;
        assert_relative_equal(&[chain_dot[2]], &[pair_dot[1]], tolerance, "theta1 ddot");
        assert_relative_equal(&[chain_dot[3]], &[pair_dot[3]], tolerance, "theta2 ddot");
        for (a, b, what) in [
            (chain_back_sub.matrix_a, pair_back_sub.matrix_a, "A"),
            (chain_back_sub.matrix_b, pair_back_sub.matrix_b, "B"),
            (chain_back_sub.matrix_c, pair_back_sub.matrix_c, "C"),
            (chain_back_sub.matrix_d, pair_back_sub.matrix_d, "D"),
        ] {
            assert!((a - b).norm() <= tolerance * b.norm(), "{what}: {a} vs {b}");
        }
        assert!((chain_back_sub.vec_trans - pair_back_sub.vec_trans).norm() < 1e-9);
        assert!((chain_back_sub.vec_rot - pair_back_sub.vec_rot).norm() < 1e-9);
        let chain_props = chain.mass_properties(&chain_state);
        let pair_props = two_dof.mass_properties(&pair_state);
        assert!(
            (chain_props.inertia_about_point_b_body_prime_kg_m2ps
                - pair_props.inertia_about_point_b_body_prime_kg_m2ps)
                .norm()
                < 1e-10
        );
    }
}
