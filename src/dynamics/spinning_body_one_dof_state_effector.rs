//! A rigid body spinning about one hub-fixed axis: port of upstream Basilisk
//! `simulation/dynamics/spinningBodies/spinningBodiesOneDOF`
//! (`SpinningBodyOneDOFStateEffector`).
//!
//! The spinner frame S is the frame S0, fixed in the body frame B by
//! `body_to_spinner0_dcm` (`dcm_S0B`) at `spinner_origin_body_m` (`r_SB_B`),
//! rotated by the angle θ about the spin axis ŝ (`sHat_S`). The spinner's
//! center of mass Sc lies at `center_of_mass_spinner_m` (`r_ScS_S`) from the S
//! origin. The joint carries a torsional spring–damper acting on `θ − θ_ref`,
//! `θ̇ − θ̇_ref` and an optional motor torque, and the states `[θ, θ̇]` are
//! coupled to the hub by back-substitution (Vaz Carneiro, Allard and Schaub,
//! "Rotating Rigid Body Dynamics Architecture for Spacecraft Simulation
//! Software Implementation").
//!
//! Inputs are read in `pre_integration`, before each integration step
//! (upstream reads them in `UpdateState`). A connected input counts as
//! written, the stand-in for upstream's `isWritten()`.
//!
//! The spinner can also ride on a
//! [`PrescribedMotionStateEffector`](super::prescribed_motion_state_effector::PrescribedMotionStateEffector)
//! (upstream "branching"), whose frame P then plays the part of B.
//!
//! Differences from upstream:
//! - Upstream's `addDynamicEffector` (dynamic effectors attached to the
//!   spinner) is not ported, as in [`super::hinged_rigid_body_state_effector`]:
//!   there is no body-level effector attachment in the spacecraft model, so
//!   the attached-load terms are zero.
//! - Upstream locks the axis by overwriting the θ̇ state with zero whenever it
//!   updates the mass properties. Here a locked axis is evaluated with θ̇ = 0
//!   throughout the step and the committed θ̇ is zeroed in `modify_states`; a
//!   lock already commanded when the spacecraft initializes zeroes the initial
//!   θ̇, as upstream's zero-length first integration does.
//! - Upstream's prescribed-motion coupling adds `−m c r̃_PB r̃_ScS ŝ` to the
//!   rotational contribution; the moment of the joint acceleration's inertial
//!   force about B is `+m c r̃_PB r̃_ScS ŝ`, as upstream's two-DOF spinner
//!   has it, and that sign is used here. Upstream's branching test puts the
//!   spinner's center of mass on its axis, where the term vanishes.

use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::dynamics::prescribed_motion_state_effector::{
    PrescribedFrameMotion, PrescribedMotionAttachment,
};
use crate::dynamics::state_effector_support::{
    KinematicsCache, add_back_sub, all_finite, normalized_axis,
};
use crate::messages::{
    ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, HingedRigidBodyMsg, Input, Output,
    SpacecraftStateMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::avs_eigen_support::{
    DEFAULT_TOLERANCE, is_rotation_matrix, is_valid_inertia_matrix,
};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm, prv_to_dcm, tilde};

/// User-settable parameters of [`SpinningBodyOneDofStateEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct SpinningBodyOneDofConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mass` \[kg\] (default 1; zero is allowed).
    pub mass_kg: f64,
    /// Upstream `k` \[N m/rad\]: torsional spring constant (default 0).
    pub spring_constant_nm_per_rad: f64,
    /// Upstream `c` \[N m s/rad\]: rotational damping coefficient (default 0).
    pub damping_nm_s_per_rad: f64,
    /// Upstream `thetaInit` \[rad\] (default 0).
    pub theta_init_rad: f64,
    /// Upstream `thetaDotInit` \[rad/s\] (default 0).
    pub theta_dot_init_radps: f64,
    /// Upstream `r_SB_B` \[m\]: spinner frame origin S relative to the body
    /// origin (default zero).
    pub spinner_origin_body_m: Vector3<f64>,
    /// Upstream `r_ScS_S` \[m\]: spinner center of mass relative to S, in S
    /// components (default zero).
    pub center_of_mass_spinner_m: Vector3<f64>,
    /// Upstream `sHat_S`: spin axis in S components, normalized when used.
    /// Upstream's constructor leaves it zero, so it must be set (default
    /// zero).
    pub spin_axis_spinner: Vector3<f64>,
    /// Upstream `IPntSc_S` \[kg m^2\]: inertia about the spinner center of
    /// mass in S components (default identity).
    pub inertia_about_com_spinner_kg_m2: Matrix3<f64>,
    /// Upstream `dcm_S0B`: DCM from the body frame to the S frame at θ = 0
    /// (default identity).
    pub body_to_spinner0_dcm: Matrix3<f64>,
}

impl SpinningBodyOneDofConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mass_kg: 1.0,
            spring_constant_nm_per_rad: 0.0,
            damping_nm_s_per_rad: 0.0,
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            spinner_origin_body_m: Vector3::zeros(),
            center_of_mass_spinner_m: Vector3::zeros(),
            spin_axis_spinner: Vector3::zeros(),
            inertia_about_com_spinner_kg_m2: Matrix3::identity(),
            body_to_spinner0_dcm: Matrix3::identity(),
        }
    }

    /// Upstream `validateConfiguration`: every value finite, the spin axis
    /// norm above 0.01, `body_to_spinner0_dcm` a rotation matrix, the mass
    /// non-negative and, for a body with mass, a valid inertia tensor.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("mass_kg", self.mass_kg),
            (
                "spring_constant_nm_per_rad",
                self.spring_constant_nm_per_rad,
            ),
            ("damping_nm_s_per_rad", self.damping_nm_s_per_rad),
            ("theta_init_rad", self.theta_init_rad),
            ("theta_dot_init_radps", self.theta_dot_init_radps),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite"));
            }
        }
        for (label, finite) in [
            (
                "spin_axis_spinner",
                all_finite(self.spin_axis_spinner.as_slice()),
            ),
            (
                "spinner_origin_body_m",
                all_finite(self.spinner_origin_body_m.as_slice()),
            ),
            (
                "center_of_mass_spinner_m",
                all_finite(self.center_of_mass_spinner_m.as_slice()),
            ),
            (
                "inertia_about_com_spinner_kg_m2",
                all_finite(self.inertia_about_com_spinner_kg_m2.as_slice()),
            ),
        ] {
            if !finite {
                return Err(format!("{label} must contain only finite values"));
            }
        }
        if self.spin_axis_spinner.norm() <= 0.01 {
            return Err("the norm of spin_axis_spinner must be greater than 0.01".to_string());
        }
        if !is_rotation_matrix(&self.body_to_spinner0_dcm, DEFAULT_TOLERANCE) {
            return Err(
                "body_to_spinner0_dcm must be an orthogonal, right-handed rotation matrix"
                    .to_string(),
            );
        }
        if self.mass_kg < 0.0 {
            return Err("mass_kg must be greater than or equal to 0".to_string());
        }
        if self.mass_kg > 0.0
            && !is_valid_inertia_matrix(&self.inertia_about_com_spinner_kg_m2, DEFAULT_TOLERANCE)
        {
            return Err(
                "inertia_about_com_spinner_kg_m2 must be a valid inertia tensor".to_string(),
            );
        }
        Ok(())
    }
}

/// A rigid body spinning about one axis (upstream
/// `SpinningBodyOneDOFStateEffector`).
///
/// State layout: `[θ, θ̇]`.
#[derive(Debug)]
pub struct SpinningBodyOneDofStateEffector {
    pub config: SpinningBodyOneDofConfig,
    /// Upstream `motorTorqueInMsg` (optional): entry 0 is the motor torque.
    pub motor_torque_in: Input<ArrayMotorTorqueMsg>,
    /// Upstream `motorLockInMsg` (optional): entry 0 locks the axis.
    pub motor_lock_in: Input<ArrayEffectorLockMsg>,
    /// Upstream `spinningBodyRefInMsg` (optional): reference angle and rate
    /// of the spring–damper.
    pub spinning_body_ref_in: Input<HingedRigidBodyMsg>,
    /// Upstream `spinningBodyOutMsg`: angle and rate of the axis.
    pub spinning_body_out: Output<HingedRigidBodyMsg>,
    /// Upstream `spinningBodyConfigLogOutMsg`: inertial state of the spinner,
    /// with S as body frame and the center of mass Sc as position.
    pub spinning_body_state_out: Output<SpacecraftStateMsg>,
    /// Upstream `u` \[N m\].
    pub motor_torque_nm: f64,
    /// Upstream `lockFlag`.
    pub locked: bool,
    /// Upstream `thetaRef` \[rad\].
    pub theta_ref_rad: f64,
    /// Upstream `thetaDotRef` \[rad/s\].
    pub theta_dot_ref_radps: f64,
    /// Last loaded `[θ, θ̇]`.
    state: [f64; 2],
    back_sub: RefCell<OneDofTerms>,
    kinematics: KinematicsCache<SpinningBodyOneDofConfig, OneDofKinematics>,
}

impl SpinningBodyOneDofStateEffector {
    pub fn new(config: SpinningBodyOneDofConfig) -> Self {
        let state = [config.theta_init_rad, config.theta_dot_init_radps];
        Self {
            config,
            motor_torque_in: Input::default(),
            motor_lock_in: Input::default(),
            spinning_body_ref_in: Input::default(),
            spinning_body_out: Output::default(),
            spinning_body_state_out: Output::default(),
            motor_torque_nm: 0.0,
            locked: false,
            theta_ref_rad: 0.0,
            theta_dot_ref_radps: 0.0,
            state,
            back_sub: RefCell::new(OneDofTerms::default()),
            kinematics: KinematicsCache::default(),
        }
    }

    /// Upstream `thetaState` \[rad\] of the last loaded state.
    pub fn theta_rad(&self) -> f64 {
        self.state[0]
    }

    /// Upstream `thetaDotState` \[rad/s\] of the last loaded state.
    pub fn theta_dot_radps(&self) -> f64 {
        self.state[1]
    }

    /// Inertial state of the spinner (upstream
    /// `computeSpinningBodyInertialStates`) for the last loaded state and the
    /// given state of the frame the spinner is mounted on.
    pub fn spinner_inertial_state(&self, hub_state: &SpacecraftStateMsg) -> SpacecraftStateMsg {
        let k = self.kinematics(&self.state);
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        let r_dot_scb_b = k.r_prime_scs_b + omega_bn_b.cross(&k.r_scb_b);
        SpacecraftStateMsg {
            position_m: hub_state.position_m + dcm_nb * k.r_scb_b,
            velocity_mps: hub_state.velocity_mps + dcm_nb * r_dot_scb_b,
            sigma_bn: dcm_to_mrp(k.dcm_bs.transpose() * dcm_bn),
            omega_radps: k.dcm_bs.transpose() * (k.omega_sb_b + omega_bn_b),
            ..Default::default()
        }
    }

    fn kinematics(&self, effector_state: &[f64]) -> OneDofKinematics {
        assert_eq!(
            effector_state.len(),
            2,
            "spinning body one DOF state length mismatch"
        );
        let key = [
            effector_state[0],
            if self.locked { 0.0 } else { effector_state[1] },
        ];
        self.kinematics.get_or_compute(&key, &self.config, || {
            OneDofKinematics::new(&self.config, key[0], key[1])
        })
    }

    /// Upstream `updateContributions`.
    fn back_sub_terms(
        &self,
        k: &OneDofKinematics,
        omega_bn_b: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> OneDofTerms {
        let m = self.config.mass_kg;
        let omega_sn_b = k.omega_sb_b + omega_bn_b;
        let r_tilde_scs_b = tilde(k.r_scs_b);
        let i_pnt_s_b = k.i_pnt_sc_b - m * r_tilde_scs_b * r_tilde_scs_b;
        let r_scs_cross_s_hat_b = k.r_scs_b.cross(&k.s_hat_b);
        let m_theta = k.s_hat_b.dot(&(i_pnt_s_b * k.s_hat_b));

        let (a_theta, b_theta, c_theta) = if self.locked {
            (Vector3::zeros(), Vector3::zeros(), 0.0)
        } else {
            let r_sb_b = self.config.spinner_origin_body_m;
            let a_theta = m * r_scs_cross_s_hat_b / m_theta;
            let b_theta =
                -(i_pnt_s_b * k.s_hat_b - m * r_sb_b.cross(&r_scs_cross_s_hat_b)) / m_theta;
            let r_dot_sb_b = omega_bn_b.cross(&r_sb_b);
            let gravity_torque_pnt_s_b = k.r_scs_b.cross(&(m * gravity_body_mps2));
            let c_theta = (self.motor_torque_nm
                - self.config.spring_constant_nm_per_rad * (k.theta - self.theta_ref_rad)
                - self.config.damping_nm_s_per_rad * (k.theta_dot - self.theta_dot_ref_radps)
                + k.s_hat_b.dot(
                    &(gravity_torque_pnt_s_b
                        - omega_sn_b.cross(&(i_pnt_s_b * omega_sn_b))
                        - i_pnt_s_b * omega_bn_b.cross(&k.omega_sb_b)
                        - m * k.r_scs_b.cross(&omega_bn_b.cross(&r_dot_sb_b))),
                ))
                / m_theta;
            (a_theta, b_theta, c_theta)
        };

        let rot_theta_factor_b =
            k.i_pnt_sc_b * k.s_hat_b - m * k.r_scb_b.cross(&r_scs_cross_s_hat_b);
        OneDofTerms {
            back_sub: BackSubMatrices {
                matrix_a: -m * r_scs_cross_s_hat_b * a_theta.transpose(),
                matrix_b: -m * r_scs_cross_s_hat_b * b_theta.transpose(),
                matrix_c: rot_theta_factor_b * a_theta.transpose(),
                matrix_d: rot_theta_factor_b * b_theta.transpose(),
                vec_trans: -m * k.omega_sb_b.cross(&k.r_prime_scs_b)
                    + m * r_scs_cross_s_hat_b * c_theta,
                vec_rot: -omega_sn_b.cross(&(k.i_pnt_sc_b * k.omega_sb_b))
                    - m * omega_bn_b.cross(&k.r_scb_b.cross(&k.r_prime_scs_b))
                    - m * k.r_scb_b.cross(&k.omega_sb_b.cross(&k.r_prime_scs_b))
                    - rot_theta_factor_b * c_theta,
            },
            a_theta,
            b_theta,
            c_theta,
        }
    }
}

impl StateEffector for SpinningBodyOneDofStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        2
    }

    /// Validates the configuration (upstream `registerStates`) and returns the
    /// initial `[θ, θ̇]`, with θ̇ zeroed if `motor_lock_in` already locks the
    /// axis.
    ///
    /// # Panics
    ///
    /// If [`SpinningBodyOneDofConfig::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid SpinningBodyOneDofConfig '{}': {message}",
                self.config.name
            );
        }
        let locked = self.motor_lock_in.is_connected()
            && self.motor_lock_in.read().effector_lock[0] == EffectorLock::Locked;
        let theta_dot = if locked {
            0.0
        } else {
            self.config.theta_dot_init_radps
        };
        vec![self.config.theta_init_rad, theta_dot]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            2,
            "spinning body one DOF state length mismatch"
        );
        self.state = [state[0], state[1]];
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        if self.motor_torque_in.is_connected() {
            self.motor_torque_nm = self.motor_torque_in.read().motor_torque_nm[0];
        }
        if self.motor_lock_in.is_connected() {
            self.locked = self.motor_lock_in.read().effector_lock[0] == EffectorLock::Locked;
        }
        if self.spinning_body_ref_in.is_connected() {
            let reference = self.spinning_body_ref_in.read();
            self.theta_ref_rad = reference.theta_rad;
            self.theta_dot_ref_radps = reference.theta_dot_radps;
        }
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let k = self.kinematics(effector_state);
        let terms = self.back_sub_terms(&k, body_omega_radps, gravity_body_mps2);
        add_back_sub(back_sub, &terms.back_sub);
        self.back_sub.replace(terms);
    }

    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        let k = self.kinematics(effector_state);
        let terms = self.back_sub.borrow();
        let theta_ddot = terms.a_theta.dot(&body_trans_accel_mps2)
            + terms.b_theta.dot(&body_omega_dot_radps2)
            + terms.c_theta;
        vec![k.theta_dot, theta_ddot]
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let k = self.kinematics(effector_state);
        let m = self.config.mass_kg;
        let r_tilde = tilde(k.r_scb_b);
        let r_prime_tilde = tilde(k.r_prime_scs_b);
        let omega_tilde = tilde(k.omega_sb_b);
        StateEffectorMassProps {
            mass_kg: m,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m: k.r_scb_b,
            center_of_mass_prime_body_mps: k.r_prime_scs_b,
            inertia_about_point_b_body_kg_m2: k.i_pnt_sc_b - m * r_tilde * r_tilde,
            inertia_about_point_b_body_prime_kg_m2ps: omega_tilde * k.i_pnt_sc_b
                - k.i_pnt_sc_b * omega_tilde
                - m * (r_prime_tilde * r_tilde + r_tilde * r_prime_tilde),
        }
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let k = self.kinematics(effector_state);
        let omega_sn_b = k.omega_sb_b + body_omega_radps;
        let r_dot_scb_b = k.r_prime_scs_b + body_omega_radps.cross(&k.r_scb_b);
        k.i_pnt_sc_b * omega_sn_b + self.config.mass_kg * k.r_scb_b.cross(&r_dot_scb_b)
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the spring energy.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let k = self.kinematics(effector_state);
        let omega_sn_b = k.omega_sb_b + body_omega_radps;
        let r_dot_scb_b = k.r_prime_scs_b + body_omega_radps.cross(&k.r_scb_b);
        let deflection_rad = k.theta - self.theta_ref_rad;
        0.5 * omega_sn_b.dot(&(k.i_pnt_sc_b * omega_sn_b))
            + 0.5 * self.config.mass_kg * r_dot_scb_b.norm_squared()
            + 0.5 * self.config.spring_constant_nm_per_rad * deflection_rad * deflection_rad
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.spinning_body_out.write(HingedRigidBodyMsg {
            theta_rad: self.state[0],
            theta_dot_radps: self.state[1],
        });
        self.spinning_body_state_out
            .write(self.spinner_inertial_state(hub_state));
    }

    /// Zeroes θ̇ of a locked axis (upstream overwrites the state the same way).
    fn modify_states(&mut self, _current_sim_nanos: u64, effector_state: &mut [f64]) {
        if self.locked {
            effector_state[1] = 0.0;
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrescribedMotionAttachment for SpinningBodyOneDofStateEffector {
    /// Upstream `addPrescribedMotionCouplingContributions`. The spinner's B
    /// frame is the prescribed frame P here.
    fn add_prescribed_motion_coupling(
        &self,
        effector_state: &[f64],
        motion: &PrescribedFrameMotion,
        back_sub: &mut BackSubMatrices,
    ) {
        let k = self.kinematics(effector_state);
        let terms = self.back_sub.borrow();
        let (a, b, c) = (terms.a_theta, terms.b_theta, terms.c_theta);
        let m = self.config.mass_kg;
        let f = motion.in_child_frame();

        let s_hat_p = k.s_hat_b;
        let r_tilde_scs_p = tilde(k.r_scs_b);
        let r_prime_scp_p = k.r_prime_scs_b;
        let r_scp_p = k.r_scb_b;
        let r_tilde_scp_p = tilde(r_scp_p);

        back_sub.matrix_b += m * r_tilde_scs_p * s_hat_p * a.transpose() * f.r_tilde_pb_p;

        let term1 = -2.0 * m * f.omega_tilde_pb_p * r_prime_scp_p
            - m * f.omega_prime_tilde_pb_p * r_scp_p
            - m * f.omega_tilde_pb_p * f.omega_tilde_pb_p * r_scp_p
            - m * f.r_prime_prime_pb_p;
        let term2 = a.dot(&f.frame_accel_p);
        let term3 = b.dot(&f.frame_ang_accel_p);
        back_sub.vec_trans += term1 + m * (term2 + term3) * r_tilde_scs_p * s_hat_p;

        back_sub.matrix_c += -m * f.r_tilde_pb_p * r_tilde_scs_p * s_hat_p * a.transpose();

        let i_sc_p = k.i_pnt_sc_b;
        let lever = i_sc_p - m * r_tilde_scp_p * r_tilde_scs_p - m * f.r_tilde_pb_p * r_tilde_scs_p;
        back_sub.matrix_d += -m * f.r_tilde_pb_p * r_tilde_scs_p * s_hat_p * b.transpose()
            - lever * s_hat_p * a.transpose() * f.r_tilde_pb_p;

        let r_tilde_scb_p = tilde(r_scp_p + f.r_pb_p);
        let omega_sp_p = k.omega_sb_b;
        let omega_tilde_sp_p = tilde(omega_sp_p);
        let omega_tilde_sn_p = tilde(omega_sp_p + f.omega_pn_p);
        let omega_tilde_pn_p = tilde(f.omega_pn_p);
        let r_prime_scs_p = k.r_prime_scs_b;
        let w = f.omega_tilde_pb_p;
        let transport = w * r_scp_p + f.r_prime_pb_p;
        back_sub.vec_rot += -i_sc_p * (w * omega_sp_p + f.omega_prime_pb_p)
            - omega_tilde_sn_p * i_sc_p * f.omega_pb_p
            - m * f.r_tilde_pb_p * omega_tilde_sp_p * r_prime_scs_p
            - m * r_tilde_scb_p
                * (2.0 * w * r_prime_scp_p
                    + f.omega_prime_tilde_pb_p * r_scp_p
                    + w * w * r_scp_p
                    + f.r_prime_prime_pb_p)
            + m * w * r_tilde_scp_p * r_prime_scp_p
            - m * omega_tilde_pn_p * f.r_tilde_pb_p * r_prime_scp_p
            + m * w * f.r_tilde_pb_p * r_prime_scp_p
            - m * omega_tilde_pn_p * r_tilde_scb_p * transport
            + m * w * r_tilde_scb_p * transport
            + m * c * f.r_tilde_pb_p * r_tilde_scs_p * s_hat_p
            - (lever * s_hat_p * a.transpose()) * f.frame_accel_p
            - (lever * s_hat_p * b.transpose()) * f.frame_ang_accel_p;
    }
}

/// Back-substitution contributions and the `θ̈ = a·r̈_B + b·ω̇ + c` terms
/// cached for `compute_derivatives`.
#[derive(Clone, Debug, Default)]
struct OneDofTerms {
    back_sub: BackSubMatrices,
    /// Upstream `aTheta`.
    a_theta: Vector3<f64>,
    /// Upstream `bTheta`.
    b_theta: Vector3<f64>,
    /// Upstream `cTheta`.
    c_theta: f64,
}

/// Spinner geometry for one state (upstream `updateEffectorMassProps`).
#[derive(Clone, Debug)]
struct OneDofKinematics {
    theta: f64,
    /// θ̇, zero for a locked axis.
    theta_dot: f64,
    /// Upstream `dcm_BS`.
    dcm_bs: Matrix3<f64>,
    /// Upstream `sHat_B`.
    s_hat_b: Vector3<f64>,
    /// Upstream `r_ScS_B`.
    r_scs_b: Vector3<f64>,
    /// Upstream `r_ScB_B`.
    r_scb_b: Vector3<f64>,
    /// Upstream `rPrime_ScS_B`, which equals `rPrime_ScB_B`.
    r_prime_scs_b: Vector3<f64>,
    /// Upstream `omega_SB_B`.
    omega_sb_b: Vector3<f64>,
    /// Upstream `IPntSc_B`.
    i_pnt_sc_b: Matrix3<f64>,
}

impl OneDofKinematics {
    fn new(config: &SpinningBodyOneDofConfig, theta: f64, theta_dot: f64) -> Self {
        let s_hat_s = normalized_axis(config.spin_axis_spinner);
        let dcm_s0s = prv_to_dcm(-theta * s_hat_s);
        let dcm_bs = config.body_to_spinner0_dcm.transpose() * dcm_s0s;
        let s_hat_b = dcm_bs * s_hat_s;
        let r_scs_b = dcm_bs * config.center_of_mass_spinner_m;
        let omega_sb_b = theta_dot * s_hat_b;
        Self {
            theta,
            theta_dot,
            dcm_bs,
            s_hat_b,
            r_scs_b,
            r_scb_b: r_scs_b + config.spinner_origin_body_m,
            r_prime_scs_b: omega_sb_b.cross(&r_scs_b),
            omega_sb_b,
            i_pnt_sc_b: dcm_bs * config.inertia_about_com_spinner_kg_m2 * dcm_bs.transpose(),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_spinningBodyOneDOFStateEffector.py` and
    //! `test_spinningBodyOneDOFResetChecks.py`.
    //!
    //! The reset checks' "reset" path (calling `Reset` on the effector alone)
    //! and "attachment" path (spacecraft initialization) are the same check
    //! here: [`SpinningBodyOneDofConfig::validate`], which
    //! [`StateEffector::initial_state`] runs.

    use super::{SpinningBodyOneDofConfig, SpinningBodyOneDofStateEffector};
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, assert_relative_equal,
        run_logging_every_step, spacecraft_with_effectors,
    };
    use crate::messages::{
        ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, HingedRigidBodyMsg, Output,
    };
    use crate::spacecraft::StateEffector;
    use nalgebra::{Matrix3, Vector3};

    const D2R: f64 = std::f64::consts::PI / 180.0;

    /// Upstream `spinningBody` of the integrated test.
    fn upstream_spinning_body() -> SpinningBodyOneDofConfig {
        let mut config = SpinningBodyOneDofConfig::new("SpinningBody");
        config.mass_kg = 50.0;
        config.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0));
        config.body_to_spinner0_dcm = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        config.center_of_mass_spinner_m = Vector3::new(1.0, 0.0, -1.0);
        config.spinner_origin_body_m = Vector3::new(0.5, -1.5, -0.5);
        config.spin_axis_spinner = Vector3::new(0.0, -1.0, 0.0);
        config.theta_init_rad = 5.0 * D2R;
        config.theta_dot_init_radps = -D2R;
        config.spring_constant_nm_per_rad = 100.0;
        config
    }

    /// The hub of the spinning body tests: `HubSetup::hinged_body_hub` with
    /// its center of mass 1 m up, tumbling in the "Gravity" orbit.
    pub(crate) fn spinning_body_hub(omega_bn_b_radps: Vector3<f64>) -> HubSetup {
        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        hub.omega_bn_b_radps = omega_bn_b_radps;
        hub
    }

    /// Upstream `test_spinningBody`: 25 s at a 1 ms step, conservation to
    /// 1e-13 and, with a reference angle, convergence to it within 0.01 rad.
    fn spinning_body_test(cmd_torque_nm: f64, lock: bool, theta_ref_rad: f64) {
        let mut config = upstream_spinning_body();
        if theta_ref_rad != 0.0 {
            config.damping_nm_s_per_rad = 50.0;
        }
        if lock {
            config.theta_dot_init_radps = 0.0;
        }
        let mut effector = SpinningBodyOneDofStateEffector::new(config);
        let torque = Output::new(ArrayMotorTorqueMsg::from_active(&[cmd_torque_nm]));
        effector.motor_torque_in.connect(torque.slot());
        let mut lock_msg = ArrayEffectorLockMsg::default();
        lock_msg.effector_lock[0] = EffectorLock::Locked;
        let lock_out = Output::new(lock_msg);
        if lock {
            effector.motor_lock_in.connect(lock_out.slot());
        }
        let reference = Output::new(HingedRigidBodyMsg {
            theta_rad: theta_ref_rad,
            theta_dot_radps: 0.0,
        });
        effector.spinning_body_ref_in.connect(reference.slot());
        let theta_out = effector.spinning_body_out.clone();

        let hub = spinning_body_hub(Vector3::new(0.1, -0.1, 0.1));
        let mut spacecraft = spacecraft_with_effectors(&hub, 1_000_000, vec![Box::new(effector)]);
        add_point_mass_earth(&mut spacecraft);

        let log = run_logging_every_step(&mut spacecraft, 25_000, || {});

        let accuracy = 1e-13;
        let check_energy = cmd_torque_nm == 0.0 && theta_ref_rad == 0.0;
        assert_conserved(&log[0], &log[log.len() - 1], accuracy, check_energy);
        if theta_ref_rad != 0.0 {
            let theta = theta_out.read().theta_rad;
            assert!(
                (theta - theta_ref_rad).abs() <= 0.01,
                "theta {theta} did not converge to {theta_ref_rad}"
            );
        }
        if lock {
            let state = theta_out.read();
            assert_eq!(state.theta_dot_radps, 0.0);
            assert_eq!(state.theta_rad, 5.0 * D2R);
        }
    }

    #[test]
    fn spinning_body_free() {
        spinning_body_test(0.0, false, 0.0);
    }

    #[test]
    fn spinning_body_locked() {
        spinning_body_test(0.0, true, 0.0);
    }

    #[test]
    fn spinning_body_commanded_torque() {
        spinning_body_test(1.0, false, 0.0);
    }

    #[test]
    fn spinning_body_reference_angle() {
        spinning_body_test(0.0, false, 20.0 * D2R);
    }

    /// Upstream `_validSpinningBody` of the reset checks.
    fn valid_spinning_body() -> SpinningBodyOneDofConfig {
        let mut config = SpinningBodyOneDofConfig::new("spinningBody");
        config.mass_kg = 50.0;
        config.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0));
        config.body_to_spinner0_dcm = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        config.center_of_mass_spinner_m = Vector3::new(1.0, 0.0, -1.0);
        config.spinner_origin_body_m = Vector3::new(0.5, -1.5, -0.5);
        config.spin_axis_spinner = Vector3::new(0.0, -1.0, 0.0);
        config
    }

    /// Upstream `test_spinningBodyOneDOF_rejectsInvalidConfiguration`.
    #[test]
    fn rejects_invalid_configuration() {
        type Break = fn(&mut SpinningBodyOneDofConfig);
        let cases: [(&str, &str, Break); 6] = [
            ("dcm_S0B not orthogonal", "body_to_spinner0_dcm", |c| {
                c.body_to_spinner0_dcm = Matrix3::new(1.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
            }),
            ("dcm_S0B left-handed", "body_to_spinner0_dcm", |c| {
                c.body_to_spinner0_dcm = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0))
            }),
            ("IPntSc_S not symmetric", "inertia_about_com_spinner", |c| {
                c.inertia_about_com_spinner_kg_m2 =
                    Matrix3::new(50.0, 5.0, 0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 40.0)
            }),
            (
                "IPntSc_S not positive definite",
                "inertia_about_com_spinner",
                |c| {
                    c.inertia_about_com_spinner_kg_m2 =
                        Matrix3::from_diagonal(&Vector3::new(-50.0, 30.0, 40.0))
                },
            ),
            ("sHat_S not set", "spin_axis_spinner", |c| {
                c.spin_axis_spinner = Vector3::zeros()
            }),
            ("mass negative", "mass_kg", |c| c.mass_kg = -1.0),
        ];
        for (case, expected, break_it) in cases {
            let mut config = valid_spinning_body();
            break_it(&mut config);
            let result = config.validate();
            assert!(
                result
                    .as_ref()
                    .is_err_and(|message| message.contains(expected)),
                "{case}: expected an error naming {expected}, got {result:?}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "spin_axis_spinner")]
    fn spacecraft_initialization_rejects_invalid_configuration() {
        let mut config = valid_spinning_body();
        config.spin_axis_spinner = Vector3::zeros();
        let _ = SpinningBodyOneDofStateEffector::new(config).initial_state();
    }

    /// Upstream `test_spinningBodyOneDOF_resetAcceptsValidConfig` and
    /// `test_spinningBodyOneDOF_resetAcceptsMasslessZeroInertia`.
    #[test]
    fn accepts_valid_and_massless_configurations() {
        assert_eq!(valid_spinning_body().validate(), Ok(()));
        let mut massless = valid_spinning_body();
        massless.mass_kg = 0.0;
        massless.inertia_about_com_spinner_kg_m2 = Matrix3::zeros();
        assert_eq!(massless.validate(), Ok(()));
    }

    #[test]
    fn mass_properties_and_axis_follow_the_angle() {
        let mut config = SpinningBodyOneDofConfig::new("spinner");
        config.mass_kg = 2.0;
        config.spin_axis_spinner = Vector3::new(0.0, 0.0, 5.0);
        config.center_of_mass_spinner_m = Vector3::new(1.0, 0.0, 0.0);
        config.spinner_origin_body_m = Vector3::new(0.0, 0.0, 3.0);
        let effector = SpinningBodyOneDofStateEffector::new(config);

        let props = effector.mass_properties(&[std::f64::consts::FRAC_PI_2, 0.5]);

        // A quarter turn about +z carries the center of mass from +x to +y.
        assert!((props.center_of_mass_body_m - Vector3::new(0.0, 1.0, 3.0)).norm() < 1e-12);
        assert!(
            (props.center_of_mass_prime_body_mps - Vector3::new(-0.5, 0.0, 0.0)).norm() < 1e-12
        );
        let r = props.center_of_mass_body_m;
        let expected = Matrix3::identity()
            + 2.0 * (r.norm_squared() * Matrix3::identity() - r * r.transpose());
        assert!((props.inertia_about_point_b_body_kg_m2 - expected).norm() < 1e-12);
    }

    #[test]
    fn reads_inputs_and_zeroes_a_locked_rate() {
        let mut config = valid_spinning_body();
        config.theta_dot_init_radps = 0.3;
        let mut effector = SpinningBodyOneDofStateEffector::new(config);
        assert_eq!(effector.initial_state(), vec![0.0, 0.3]);

        let torque = Output::new(ArrayMotorTorqueMsg::from_active(&[0.7, 9.0]));
        let mut lock = ArrayEffectorLockMsg::default();
        lock.effector_lock[0] = EffectorLock::Locked;
        let lock = Output::new(lock);
        let reference = Output::new(HingedRigidBodyMsg {
            theta_rad: 0.2,
            theta_dot_radps: -0.1,
        });
        effector.motor_torque_in.connect(torque.slot());
        effector.motor_lock_in.connect(lock.slot());
        effector.spinning_body_ref_in.connect(reference.slot());
        assert_eq!(effector.initial_state(), vec![0.0, 0.0]);

        effector.pre_integration(0, 0.1);
        assert_eq!(effector.motor_torque_nm, 0.7);
        assert!(effector.locked);
        assert_eq!(
            (effector.theta_ref_rad, effector.theta_dot_ref_radps),
            (0.2, -0.1)
        );
        let mut state = [0.1, 0.4];
        assert_eq!(
            effector.compute_derivatives(&state, Vector3::x(), Vector3::y()),
            vec![0.0, 0.0]
        );
        effector.modify_states(0, &mut state);
        assert_eq!(state, [0.1, 0.0]);
        assert_relative_equal(
            &[effector.mass_properties(&state).mass_kg],
            &[50.0],
            0.0,
            "mass",
        );
    }
}
