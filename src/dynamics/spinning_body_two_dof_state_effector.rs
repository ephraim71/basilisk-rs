//! Two rigid bodies spinning in series, each about one axis: port of upstream
//! Basilisk `simulation/dynamics/spinningBodies/spinningBodiesTwoDOF`
//! (`SpinningBodyTwoDOFStateEffector`).
//!
//! The lower body's frame S1 is the frame S10, fixed in the body frame B by
//! `body_to_spinner1_dcm` (`dcm_S10B`) at `spinner1_origin_body_m`
//! (`r_S1B_B`), rotated by θ1 about ŝ1. The upper body's frame S2 is the frame
//! S20, fixed in S1 by `spinner1_to_spinner2_dcm` (`dcm_S20S1`) at
//! `spinner2_origin_spinner1_m` (`r_S2S1_S1`), rotated by θ2 about ŝ2, so θ2 is
//! measured relative to the lower body. Each joint carries a torsional
//! spring–damper acting on `θi − θi,ref`, `θ̇i − θ̇i,ref` and an optional motor
//! torque. The lower body may be massless, which turns the pair into a
//! two-axis gimbal.
//!
//! State layout: `[θ1, θ̇1, θ2, θ̇2]`.
//!
//! Inputs are read in `pre_integration`, before each integration step
//! (upstream reads them in `UpdateState`); a connected input counts as
//! written. The pair can ride on a
//! [`PrescribedMotionStateEffector`](super::prescribed_motion_state_effector::PrescribedMotionStateEffector).
//!
//! Differences from upstream, as in
//! [`super::spinning_body_one_dof_state_effector`]: `addDynamicEffector` is not
//! ported (the attached-load terms are zero), and a locked axis is evaluated
//! with a zero rate during the step, its committed rate zeroed in
//! `modify_states` and, if locked at initialization, in the initial state.

use nalgebra::{Matrix2, Matrix2x3, Matrix3, Vector2, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::dynamics::prescribed_motion_state_effector::{
    PrescribedFrameMotion, PrescribedMotionAttachment,
};
use crate::dynamics::state_effector_support::{
    KinematicsCache, add_back_sub, all_finite, normalized_axis,
};
use crate::kinematics::{is_rotation_matrix, is_valid_inertia_matrix};
use crate::messages::{
    ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, HingedRigidBodyMsg, Input, Output,
    SpacecraftStateMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm, prv_to_dcm, tilde};

/// Mass, joint and initial-state parameters of one body of a
/// [`SpinningBodyTwoDofStateEffector`]. Upstream keeps them as `mass1`,
/// `k1`, ... and `mass2`, `k2`, ...
#[derive(Clone, Debug, PartialEq)]
pub struct SpinningBodyTwoDofBodyConfig {
    /// Upstream `mass1` (default 0) / `mass2` (default 1) \[kg\].
    pub mass_kg: f64,
    /// Upstream `k1`/`k2` \[N m/rad\]: torsional spring constant (default 0).
    pub spring_constant_nm_per_rad: f64,
    /// Upstream `c1`/`c2` \[N m s/rad\]: rotational damping coefficient
    /// (default 0).
    pub damping_nm_s_per_rad: f64,
    /// Upstream `theta1Init`/`theta2Init` \[rad\] (default 0).
    pub theta_init_rad: f64,
    /// Upstream `theta1DotInit`/`theta2DotInit` \[rad/s\] (default 0).
    pub theta_dot_init_radps: f64,
    /// Upstream `r_Sc1S1_S1`/`r_Sc2S2_S2` \[m\]: the body's center of mass
    /// relative to its frame origin, in its frame (default zero).
    pub center_of_mass_spinner_m: Vector3<f64>,
    /// Upstream `s1Hat_S1`/`s2Hat_S2`: spin axis in the body's frame,
    /// normalized when used (default `[1, 0, 0]`).
    pub spin_axis_spinner: Vector3<f64>,
    /// Upstream `IS1PntSc1_S1`/`IS2PntSc2_S2` \[kg m^2\]: inertia about the
    /// body's center of mass in its frame (default identity).
    pub inertia_about_com_spinner_kg_m2: Matrix3<f64>,
}

impl SpinningBodyTwoDofBodyConfig {
    fn with_mass(mass_kg: f64) -> Self {
        Self {
            mass_kg,
            spring_constant_nm_per_rad: 0.0,
            damping_nm_s_per_rad: 0.0,
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            center_of_mass_spinner_m: Vector3::zeros(),
            spin_axis_spinner: Vector3::x(),
            inertia_about_com_spinner_kg_m2: Matrix3::identity(),
        }
    }
}

/// User-settable parameters of [`SpinningBodyTwoDofStateEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct SpinningBodyTwoDofConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// The lower body, spinning relative to the hub (default mass 0).
    pub body1: SpinningBodyTwoDofBodyConfig,
    /// The upper body, spinning relative to the lower one (default mass 1).
    pub body2: SpinningBodyTwoDofBodyConfig,
    /// Upstream `r_S1B_B` \[m\]: S1 origin relative to the body origin
    /// (default zero).
    pub spinner1_origin_body_m: Vector3<f64>,
    /// Upstream `r_S2S1_S1` \[m\]: S2 origin relative to the S1 origin, in S1
    /// components (default zero).
    pub spinner2_origin_spinner1_m: Vector3<f64>,
    /// Upstream `dcm_S10B`: DCM from B to S1 at θ1 = 0 (default identity).
    pub body_to_spinner1_dcm: Matrix3<f64>,
    /// Upstream `dcm_S20S1`: DCM from S1 to S2 at θ2 = 0 (default identity).
    pub spinner1_to_spinner2_dcm: Matrix3<f64>,
}

impl SpinningBodyTwoDofConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            body1: SpinningBodyTwoDofBodyConfig::with_mass(0.0),
            body2: SpinningBodyTwoDofBodyConfig::with_mass(1.0),
            spinner1_origin_body_m: Vector3::zeros(),
            spinner2_origin_spinner1_m: Vector3::zeros(),
            body_to_spinner1_dcm: Matrix3::identity(),
            spinner1_to_spinner2_dcm: Matrix3::identity(),
        }
    }

    /// Upstream `validateConfiguration`: every value finite, both spin axis
    /// norms above 0.01, both DCMs rotation matrices, both masses
    /// non-negative with a finite sum and, for a body with mass, a valid
    /// inertia tensor.
    pub fn validate(&self) -> Result<(), String> {
        for (prefix, body) in [("body1", &self.body1), ("body2", &self.body2)] {
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
                    return Err(format!("{prefix}.{label} must be finite"));
                }
            }
            for (label, finite) in [
                (
                    "spin_axis_spinner",
                    all_finite(body.spin_axis_spinner.as_slice()),
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
                    return Err(format!("{prefix}.{label} must contain only finite values"));
                }
            }
        }
        for (label, finite) in [
            (
                "spinner1_origin_body_m",
                all_finite(self.spinner1_origin_body_m.as_slice()),
            ),
            (
                "spinner2_origin_spinner1_m",
                all_finite(self.spinner2_origin_spinner1_m.as_slice()),
            ),
        ] {
            if !finite {
                return Err(format!("{label} must contain only finite values"));
            }
        }
        for (prefix, body) in [("body1", &self.body1), ("body2", &self.body2)] {
            if body.spin_axis_spinner.norm() <= 0.01 {
                return Err(format!(
                    "the norm of {prefix}.spin_axis_spinner must be greater than 0.01"
                ));
            }
        }
        for (label, dcm) in [
            ("body_to_spinner1_dcm", &self.body_to_spinner1_dcm),
            ("spinner1_to_spinner2_dcm", &self.spinner1_to_spinner2_dcm),
        ] {
            if !is_rotation_matrix(dcm) {
                return Err(format!(
                    "{label} must be an orthogonal, right-handed rotation matrix"
                ));
            }
        }
        for (prefix, body) in [("body1", &self.body1), ("body2", &self.body2)] {
            if body.mass_kg < 0.0 {
                return Err(format!(
                    "{prefix}.mass_kg must be greater than or equal to 0"
                ));
            }
        }
        if !(self.body1.mass_kg + self.body2.mass_kg).is_finite() {
            return Err("the combined mass must be finite".to_string());
        }
        for (prefix, body) in [("body1", &self.body1), ("body2", &self.body2)] {
            if body.mass_kg > 0.0 && !is_valid_inertia_matrix(&body.inertia_about_com_spinner_kg_m2)
            {
                return Err(format!(
                    "{prefix}.inertia_about_com_spinner_kg_m2 must be a valid inertia tensor"
                ));
            }
        }
        Ok(())
    }
}

/// Two rigid bodies spinning in series (upstream
/// `SpinningBodyTwoDOFStateEffector`).
#[derive(Debug)]
pub struct SpinningBodyTwoDofStateEffector {
    pub config: SpinningBodyTwoDofConfig,
    /// Upstream `motorTorqueInMsg` (optional): entries 0 and 1 are the motor
    /// torques `u1`, `u2`.
    pub motor_torque_in: Input<ArrayMotorTorqueMsg>,
    /// Upstream `motorLockInMsg` (optional): entries 0 and 1 lock the axes.
    pub motor_lock_in: Input<ArrayEffectorLockMsg>,
    /// Upstream `spinningBodyRefInMsgs` (optional): reference angle and rate
    /// of each joint.
    pub spinning_body_ref_in: [Input<HingedRigidBodyMsg>; 2],
    /// Upstream `spinningBodyOutMsgs`: angle and rate of each joint.
    pub spinning_body_out: [Output<HingedRigidBodyMsg>; 2],
    /// Upstream `spinningBodyConfigLogOutMsgs`: inertial state of each body,
    /// with its frame Si as body frame and its center of mass as position.
    pub spinning_body_state_out: [Output<SpacecraftStateMsg>; 2],
    /// Upstream `u1`, `u2` \[N m\].
    pub motor_torque_nm: [f64; 2],
    /// Upstream `lockFlag1`, `lockFlag2`.
    pub locked: [bool; 2],
    /// Upstream `theta1Ref`, `theta2Ref` \[rad\].
    pub theta_ref_rad: [f64; 2],
    /// Upstream `theta1DotRef`, `theta2DotRef` \[rad/s\].
    pub theta_dot_ref_radps: [f64; 2],
    /// Last loaded `[θ1, θ̇1, θ2, θ̇2]`.
    state: [f64; 4],
    back_sub: RefCell<TwoDofTerms>,
    kinematics: KinematicsCache<SpinningBodyTwoDofConfig, TwoDofKinematics>,
}

impl SpinningBodyTwoDofStateEffector {
    pub fn new(config: SpinningBodyTwoDofConfig) -> Self {
        let state = configured_state(&config);
        Self {
            config,
            motor_torque_in: Input::default(),
            motor_lock_in: Input::default(),
            spinning_body_ref_in: Default::default(),
            spinning_body_out: Default::default(),
            spinning_body_state_out: Default::default(),
            motor_torque_nm: [0.0; 2],
            locked: [false; 2],
            theta_ref_rad: [0.0; 2],
            theta_dot_ref_radps: [0.0; 2],
            state,
            back_sub: RefCell::new(TwoDofTerms::default()),
            kinematics: KinematicsCache::default(),
        }
    }

    /// `[θ1, θ̇1, θ2, θ̇2]` of the last loaded state.
    pub fn state(&self) -> [f64; 4] {
        self.state
    }

    /// Inertial states of the two bodies (upstream
    /// `computeSpinningBodyInertialStates`) for the last loaded state and the
    /// given state of the frame the pair is mounted on.
    pub fn spinner_inertial_states(
        &self,
        hub_state: &SpacecraftStateMsg,
    ) -> [SpacecraftStateMsg; 2] {
        let k = self.kinematics(&self.state);
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        let body = |dcm_bs: &Matrix3<f64>,
                    r_scb_b: Vector3<f64>,
                    r_prime_scb_b: Vector3<f64>,
                    omega_sb_b: Vector3<f64>| SpacecraftStateMsg {
            position_m: hub_state.position_m + dcm_nb * r_scb_b,
            velocity_mps: hub_state.velocity_mps
                + dcm_nb * (r_prime_scb_b + omega_bn_b.cross(&r_scb_b)),
            sigma_bn: dcm_to_mrp(dcm_bs.transpose() * dcm_bn),
            omega_radps: dcm_bs.transpose() * (omega_sb_b + omega_bn_b),
            ..Default::default()
        };
        [
            body(&k.dcm_bs1, k.r_sc1b_b, k.r_prime_sc1s1_b, k.omega_s1b_b),
            body(&k.dcm_bs2, k.r_sc2b_b, k.r_prime_sc2s1_b, k.omega_s2b_b),
        ]
    }

    fn kinematics(&self, effector_state: &[f64]) -> TwoDofKinematics {
        assert_eq!(
            effector_state.len(),
            4,
            "spinning body two DOF state length mismatch"
        );
        let rate = |index: usize, locked: bool| if locked { 0.0 } else { effector_state[index] };
        let key = [
            effector_state[0],
            rate(1, self.locked[0]),
            effector_state[2],
            rate(3, self.locked[1]),
        ];
        self.kinematics.get_or_compute(&key, &self.config, || {
            TwoDofKinematics::new(&self.config, [key[0], key[2]], [key[1], key[3]])
        })
    }

    /// Upstream `updateContributions`.
    fn back_sub_terms(
        &self,
        k: &TwoDofKinematics,
        omega_bn_b: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> TwoDofTerms {
        let (m1, m2) = (self.config.body1.mass_kg, self.config.body2.mass_kg);
        let m = m1 + m2;
        let (i1, i2) = (&k.i1_b, &k.i2_b);
        let r_s1b_b = self.config.spinner1_origin_body_m;
        let omega_s1n_b = k.omega_s1b_b + omega_bn_b;
        let omega_s2n_b = k.omega_s2b_b + omega_bn_b;

        let r_scs1_b = (m1 * k.r_sc1s1_b + m2 * k.r_sc2s1_b) / m;
        let r_s2b_b = k.r_s2s1_b + r_s1b_b;
        let r_tilde_sc1s1 = tilde(k.r_sc1s1_b);
        let r_tilde_sc2s1 = tilde(k.r_sc2s1_b);
        let r_tilde_sc2s2 = tilde(k.r_sc2s2_b);
        let r_tilde_s2s1 = tilde(k.r_s2s1_b);
        let r_prime_tilde_sc1s1 = tilde(k.r_prime_sc1s1_b);
        let r_prime_tilde_sc2s1 = tilde(k.r_prime_sc2s1_b);
        let r_prime_tilde_sc2s2 = tilde(k.r_prime_sc2s2_b);

        let i_s2_pnt_s2 = i2 - m2 * r_tilde_sc2s2 * r_tilde_sc2s2;
        let i_s_pnt_s1 =
            i1 - m1 * r_tilde_sc1s1 * r_tilde_sc1s1 + i2 - m2 * r_tilde_sc2s1 * r_tilde_sc2s1;
        let i_prime_s_pnt_s1 = k.i1_prime_b
            - m1 * (r_prime_tilde_sc1s1 * r_tilde_sc1s1 + r_tilde_sc1s1 * r_prime_tilde_sc1s1)
            + k.i2_prime_b
            - m2 * (r_prime_tilde_sc2s1 * r_tilde_sc2s1 + r_tilde_sc2s1 * r_prime_tilde_sc2s1);
        let i_prime_s2_pnt_s2 = k.i2_prime_b
            - m2 * (r_prime_tilde_sc2s2 * r_tilde_sc2s2 + r_tilde_sc2s2 * r_prime_tilde_sc2s2);

        let (s1, s2) = (k.s1_hat_b, k.s2_hat_b);
        let mut m_theta = Matrix2::new(
            s1.dot(&(i_s_pnt_s1 * s1)),
            s1.dot(&((i2 - m2 * r_tilde_sc2s1 * r_tilde_sc2s2) * s2)),
            s2.dot(&((i_s2_pnt_s2 - m2 * r_tilde_sc2s2 * r_tilde_s2s1) * s1)),
            s2.dot(&(i_s2_pnt_s2 * s2)),
        );

        let r_scs1_cross_s1 = r_scs1_b.cross(&s1);
        let r_sc2s2_cross_s2 = k.r_sc2s2_b.cross(&s2);
        let mut a_theta_star = Matrix2x3::from_rows(&[
            (m * r_scs1_cross_s1).transpose(),
            (m2 * r_sc2s2_cross_s2).transpose(),
        ]);
        let mut b_theta_star = Matrix2x3::from_rows(&[
            (-i_s_pnt_s1 * s1 + m * r_s1b_b.cross(&r_scs1_b.cross(&s1))).transpose(),
            (-i_s2_pnt_s2 * s2 + m2 * r_s2b_b.cross(&k.r_sc2s2_b.cross(&s2))).transpose(),
        ]);

        let w = omega_bn_b;
        let r_dot_s1b_b = w.cross(&r_s1b_b);
        let r_dot_s2s1_b = k.r_prime_s2s1_b + w.cross(&k.r_s2s1_b);
        let gravity_torque_pnt_s1 = r_scs1_b.cross(&(m * gravity_body_mps2));
        let gravity_torque_pnt_s2 = k.r_sc2s2_b.cross(&(m2 * gravity_body_mps2));
        let relative_spin = k.omega_s1b_b.cross(&k.omega_s2s1_b);
        let c_term0 = i_prime_s_pnt_s1 * w
            + w.cross(&(i_s_pnt_s1 * w))
            + k.i1_prime_b * k.omega_s1b_b
            + w.cross(&(i1 * k.omega_s1b_b))
            + k.i2_prime_b * k.omega_s2b_b
            + w.cross(&(i2 * k.omega_s2b_b))
            + i2 * relative_spin
            - m2 * k.r_sc2s1_b.cross(&k.r_sc2s2_b.cross(&relative_spin))
            + m1 * (k.r_sc1s1_b.cross(&k.omega_s1b_b.cross(&k.r_prime_sc1s1_b))
                + w.cross(&k.r_sc1s1_b.cross(&k.r_prime_sc1s1_b)))
            + m2 * (k.r_sc2s1_b.cross(&k.omega_s1b_b.cross(&k.r_prime_sc2s1_b))
                + w.cross(&k.r_sc2s1_b.cross(&k.r_prime_sc2s1_b)))
            + m2 * k.r_sc2s1_b.cross(&k.omega_s2s1_b.cross(&k.r_prime_sc2s2_b))
            + m * r_scs1_b.cross(&w.cross(&r_dot_s1b_b));
        let c_term1 = i_prime_s2_pnt_s2 * omega_s2n_b
            + w.cross(&(i_s2_pnt_s2 * omega_s2n_b))
            + i_s2_pnt_s2 * relative_spin
            + m2 * k.r_sc2s2_b.cross(&omega_s1n_b.cross(&k.r_prime_s2s1_b))
            + m2 * k.r_sc2s2_b.cross(&w.cross(&(r_dot_s2s1_b + r_dot_s1b_b)));
        let joint = |index: usize, body: &SpinningBodyTwoDofBodyConfig| {
            self.motor_torque_nm[index]
                - body.spring_constant_nm_per_rad * (k.theta[index] - self.theta_ref_rad[index])
                - body.damping_nm_s_per_rad * (k.theta_dot[index] - self.theta_dot_ref_radps[index])
        };
        let mut c_theta_star = Vector2::new(
            joint(0, &self.config.body1) + s1.dot(&gravity_torque_pnt_s1) - s1.dot(&c_term0),
            joint(1, &self.config.body2) + s2.dot(&gravity_torque_pnt_s2) - s2.dot(&c_term1),
        );

        if self.locked[0] || self.locked[1] {
            m_theta[(0, 1)] = 0.0;
            m_theta[(1, 0)] = 0.0;
            for (index, locked) in self.locked.into_iter().enumerate() {
                if locked {
                    a_theta_star.set_row(index, &nalgebra::RowVector3::zeros());
                    b_theta_star.set_row(index, &nalgebra::RowVector3::zeros());
                    c_theta_star[index] = 0.0;
                }
            }
        }

        let m_theta_inverse = m_theta
            .try_inverse()
            .expect("spinning body two DOF joint mass matrix must be invertible");
        let a_theta = m_theta_inverse * a_theta_star;
        let b_theta = m_theta_inverse * b_theta_star;
        let c_theta = m_theta_inverse * c_theta_star;

        let r_prime_scb_b = (m1 * k.r_prime_sc1s1_b + m2 * k.r_prime_sc2s1_b) / m;
        let rot_factor0 = (i1 + i2) * s1
            - m1 * k.r_sc1b_b.cross(&k.r_sc1s1_b.cross(&s1))
            - m2 * k.r_sc2b_b.cross(&k.r_sc2s1_b.cross(&s1));
        let rot_factor1 = i2 * s2 - m2 * k.r_sc2b_b.cross(&k.r_sc2s2_b.cross(&s2));
        let trans0 = -m * r_scs1_cross_s1;
        let trans1 = -m2 * r_sc2s2_cross_s2;

        TwoDofTerms {
            back_sub: BackSubMatrices {
                matrix_a: trans0 * a_theta.row(0) + trans1 * a_theta.row(1),
                matrix_b: trans0 * b_theta.row(0) + trans1 * b_theta.row(1),
                matrix_c: rot_factor0 * a_theta.row(0) + rot_factor1 * a_theta.row(1),
                matrix_d: rot_factor0 * b_theta.row(0) + rot_factor1 * b_theta.row(1),
                vec_trans: -m * k.omega_s1b_b.cross(&r_prime_scb_b)
                    - m2 * (k.omega_s2s1_b.cross(&k.r_prime_sc2s2_b)
                        - k.r_sc2s2_b.cross(&relative_spin))
                    - trans0 * c_theta[0]
                    - trans1 * c_theta[1],
                vec_rot: -k.i1_prime_b * k.omega_s1b_b
                    - w.cross(&(i1 * k.omega_s1b_b))
                    - k.i2_prime_b * k.omega_s2b_b
                    - w.cross(&(i2 * k.omega_s2b_b))
                    - i2 * relative_spin
                    + m2 * k.r_sc2b_b.cross(&k.r_sc2s2_b.cross(&relative_spin))
                    - m1 * (k.r_sc1b_b.cross(&k.omega_s1b_b.cross(&k.r_prime_sc1s1_b))
                        + w.cross(&k.r_sc1b_b.cross(&k.r_prime_sc1s1_b)))
                    - m2 * (k.r_sc2b_b.cross(&k.omega_s1b_b.cross(&k.r_prime_sc2s1_b))
                        + w.cross(&k.r_sc2b_b.cross(&k.r_prime_sc2s1_b)))
                    - m2 * k.r_sc2b_b.cross(&k.omega_s2s1_b.cross(&k.r_prime_sc2s2_b))
                    - rot_factor0 * c_theta[0]
                    - rot_factor1 * c_theta[1],
            },
            a_theta,
            b_theta,
            c_theta,
        }
    }

    /// Each body's mass, inertia about its center of mass, inertial angular
    /// velocity and center-of-mass position and velocity relative to B, in B
    /// components.
    #[allow(clippy::type_complexity)]
    fn body_motion(
        &self,
        k: &TwoDofKinematics,
        omega_bn_b: Vector3<f64>,
    ) -> [(f64, Matrix3<f64>, Vector3<f64>, Vector3<f64>, Vector3<f64>); 2] {
        [
            (
                self.config.body1.mass_kg,
                k.i1_b,
                k.omega_s1b_b + omega_bn_b,
                k.r_sc1b_b,
                k.r_prime_sc1s1_b + omega_bn_b.cross(&k.r_sc1b_b),
            ),
            (
                self.config.body2.mass_kg,
                k.i2_b,
                k.omega_s2b_b + omega_bn_b,
                k.r_sc2b_b,
                k.r_prime_sc2s1_b + omega_bn_b.cross(&k.r_sc2b_b),
            ),
        ]
    }
}

fn configured_state(config: &SpinningBodyTwoDofConfig) -> [f64; 4] {
    [
        config.body1.theta_init_rad,
        config.body1.theta_dot_init_radps,
        config.body2.theta_init_rad,
        config.body2.theta_dot_init_radps,
    ]
}

impl StateEffector for SpinningBodyTwoDofStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        4
    }

    /// Validates the configuration (upstream `registerStates`) and returns the
    /// initial `[θ1, θ̇1, θ2, θ̇2]`, with the rate of an axis that
    /// `motor_lock_in` already locks zeroed.
    ///
    /// # Panics
    ///
    /// If [`SpinningBodyTwoDofConfig::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid SpinningBodyTwoDofConfig '{}': {message}",
                self.config.name
            );
        }
        let mut state = configured_state(&self.config);
        if self.motor_lock_in.is_connected() {
            let lock = self.motor_lock_in.read();
            for index in 0..2 {
                if lock.effector_lock[index] == EffectorLock::Locked {
                    state[2 * index + 1] = 0.0;
                }
            }
        }
        state.to_vec()
    }

    fn load_state(&mut self, state: &[f64]) {
        self.state = state
            .try_into()
            .expect("spinning body two DOF state length mismatch");
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        if self.motor_torque_in.is_connected() {
            let torque = self.motor_torque_in.read().motor_torque_nm;
            self.motor_torque_nm = [torque[0], torque[1]];
        }
        if self.motor_lock_in.is_connected() {
            let lock = self.motor_lock_in.read().effector_lock;
            self.locked = [
                lock[0] == EffectorLock::Locked,
                lock[1] == EffectorLock::Locked,
            ];
        }
        for (index, input) in self.spinning_body_ref_in.iter().enumerate() {
            if input.is_connected() {
                let reference = input.read();
                self.theta_ref_rad[index] = reference.theta_rad;
                self.theta_dot_ref_radps[index] = reference.theta_dot_radps;
            }
        }
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let body_omega_radps = hub_state.omega_radps;
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
        let theta_ddot = terms.a_theta * body_trans_accel_mps2
            + terms.b_theta * body_omega_dot_radps2
            + terms.c_theta;
        vec![k.theta_dot[0], theta_ddot[0], k.theta_dot[1], theta_ddot[1]]
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let k = self.kinematics(effector_state);
        let (m1, m2) = (self.config.body1.mass_kg, self.config.body2.mass_kg);
        let m = m1 + m2;
        let r_tilde1 = tilde(k.r_sc1b_b);
        let r_tilde2 = tilde(k.r_sc2b_b);
        let r_prime_tilde1 = tilde(k.r_prime_sc1s1_b);
        let r_prime_tilde2 = tilde(k.r_prime_sc2s1_b);
        StateEffectorMassProps {
            mass_kg: m,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m: (m1 * k.r_sc1b_b + m2 * k.r_sc2b_b) / m,
            center_of_mass_prime_body_mps: (m1 * k.r_prime_sc1s1_b + m2 * k.r_prime_sc2s1_b) / m,
            inertia_about_point_b_body_kg_m2: k.i1_b + k.i2_b
                - m1 * r_tilde1 * r_tilde1
                - m2 * r_tilde2 * r_tilde2,
            inertia_about_point_b_body_prime_kg_m2ps: k.i1_prime_b + k.i2_prime_b
                - m1 * (r_prime_tilde1 * r_tilde1 + r_tilde1 * r_prime_tilde1)
                - m2 * (r_prime_tilde2 * r_tilde2 + r_tilde2 * r_prime_tilde2),
        }
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let k = self.kinematics(effector_state);
        self.body_motion(&k, body_omega_radps)
            .into_iter()
            .map(|(mass, inertia, omega_sn, r, r_dot)| inertia * omega_sn + mass * r.cross(&r_dot))
            .sum()
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the spring energies.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let k = self.kinematics(effector_state);
        let springs = [
            self.config.body1.spring_constant_nm_per_rad,
            self.config.body2.spring_constant_nm_per_rad,
        ];
        self.body_motion(&k, body_omega_radps)
            .into_iter()
            .enumerate()
            .map(|(index, (mass, inertia, omega_sn, _, r_dot))| {
                let deflection_rad = k.theta[index] - self.theta_ref_rad[index];
                0.5 * omega_sn.dot(&(inertia * omega_sn))
                    + 0.5 * mass * r_dot.norm_squared()
                    + 0.5 * springs[index] * deflection_rad * deflection_rad
            })
            .sum()
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        for (index, output) in self.spinning_body_out.iter().enumerate() {
            output.write(HingedRigidBodyMsg {
                theta_rad: self.state[2 * index],
                theta_dot_radps: self.state[2 * index + 1],
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

    /// Zeroes the rate of a locked axis (upstream overwrites the state the
    /// same way).
    fn modify_states(&mut self, _current_sim_nanos: u64, effector_state: &mut [f64]) {
        for (index, locked) in self.locked.into_iter().enumerate() {
            if locked {
                effector_state[2 * index + 1] = 0.0;
            }
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrescribedMotionAttachment for SpinningBodyTwoDofStateEffector {
    /// Upstream `addPrescribedMotionCouplingContributions`. The pair's B frame
    /// is the prescribed frame P here.
    fn add_prescribed_motion_coupling(
        &self,
        effector_state: &[f64],
        motion: &PrescribedFrameMotion,
        back_sub: &mut BackSubMatrices,
    ) {
        let k = self.kinematics(effector_state);
        let terms = self.back_sub.borrow();
        let (a, b, c) = (&terms.a_theta, &terms.b_theta, &terms.c_theta);
        let (m1, m2) = (self.config.body1.mass_kg, self.config.body2.mass_kg);
        let f = motion.in_child_frame();
        let (s1_p, s2_p) = (k.s1_hat_b, k.s2_hat_b);
        let r_tilde_pb_p = f.r_tilde_pb_p;
        let r_tilde_sc1s1_p = tilde(k.r_sc1s1_b);
        let r_tilde_sc2s1_p = tilde(k.r_sc2s1_b);
        let r_tilde_sc2s2_p = tilde(k.r_sc2s2_b);

        back_sub.matrix_b +=
            (m1 * r_tilde_sc1s1_p + m2 * r_tilde_sc2s1_p) * s1_p * a.row(0) * r_tilde_pb_p
                + m2 * r_tilde_sc2s2_p * s2_p * a.row(1) * r_tilde_pb_p;

        let w = f.omega_tilde_pb_p;
        let w_prime = f.omega_prime_tilde_pb_p;
        let r_prime_sc1p_p = k.r_prime_sc1s1_b;
        let r_prime_sc2p_p = k.r_prime_sc2s1_b;
        let (r_sc1p_p, r_sc2p_p) = (k.r_sc1b_b, k.r_sc2b_b);
        let first_moment = m1 * r_sc1p_p + m2 * r_sc2p_p;
        let trans_term1 = -2.0 * w * (m1 * r_prime_sc1p_p + m2 * r_prime_sc2p_p)
            - w_prime * first_moment
            - w * w * first_moment
            - (m1 + m2) * f.r_prime_prime_pb_p;
        let joint_accel0 = a.row(0).dot(&f.frame_accel_p.transpose())
            + b.row(0).dot(&f.frame_ang_accel_p.transpose());
        let joint_accel1 = a.row(1).dot(&f.frame_accel_p.transpose())
            + b.row(1).dot(&f.frame_ang_accel_p.transpose());
        let trans_term2 = (m1 * r_tilde_sc1s1_p + m2 * r_tilde_sc2s1_p) * joint_accel0 * s1_p;
        let trans_term3 = m2 * r_tilde_sc2s2_p * joint_accel1 * s2_p;
        back_sub.vec_trans += trans_term1 + trans_term2 + trans_term3;

        back_sub.matrix_c += -(m1 * r_tilde_pb_p * r_tilde_sc1s1_p
            + m2 * r_tilde_pb_p * r_tilde_sc2s1_p)
            * s1_p
            * a.row(0)
            - m2 * r_tilde_pb_p * r_tilde_sc2s2_p * s2_p * a.row(1);

        let int_term3 = m1 * r_tilde_pb_p * r_tilde_sc1s1_p + m2 * r_tilde_pb_p * r_tilde_sc2s1_p;
        let (i1_p, i2_p) = (k.i1_b, k.i2_b);
        let r_tilde_sc1p_p = tilde(r_sc1p_p);
        let r_tilde_sc2p_p = tilde(r_sc2p_p);
        let int_term4 = i1_p + i2_p
            - m1 * r_tilde_sc1p_p * r_tilde_sc1s1_p
            - m2 * r_tilde_sc2p_p * r_tilde_sc2s1_p
            - int_term3;
        let int_term5 = m2 * r_tilde_pb_p * r_tilde_sc2s2_p;
        let int_term6 = i2_p - m2 * r_tilde_sc2p_p * r_tilde_sc2s2_p - int_term5;
        back_sub.matrix_d += -int_term3 * s1_p * b.row(0)
            - int_term4 * s1_p * a.row(0) * r_tilde_pb_p
            - int_term5 * s2_p * b.row(1)
            - int_term6 * s2_p * a.row(1) * r_tilde_pb_p;

        let omega_pb_p = f.omega_pb_p;
        let vec_rot_term1 = -k.i1_prime_b * omega_pb_p - k.i2_prime_b * omega_pb_p;
        let omega_tilde_pn_p = tilde(f.omega_pn_p);
        let vec_rot_term2 = -i1_p * f.omega_prime_pb_p
            - i2_p * f.omega_prime_pb_p
            - omega_tilde_pn_p * (i1_p * omega_pb_p + i2_p * omega_pb_p);

        let omega_tilde_s1p_p = tilde(k.omega_s1b_b);
        let omega_tilde_s2s1_p = tilde(k.omega_s2s1_b);
        let int_term7 = -m1 * r_tilde_pb_p * omega_tilde_s1p_p * k.r_prime_sc1s1_b;
        let int_term8 = -m2
            * r_tilde_pb_p
            * (omega_tilde_s1p_p * omega_tilde_s2s1_p * k.r_sc2s2_b
                + omega_tilde_s2s1_p * k.r_prime_sc2s2_b
                + omega_tilde_s1p_p * k.r_prime_sc2s1_b);
        let vec_rot_term3 = int_term7 + int_term8;

        let r_tilde_sc1b_p = tilde(r_sc1p_p + f.r_pb_p);
        let r_tilde_sc2b_p = tilde(r_sc2p_p + f.r_pb_p);
        let frame_relative_accel = |r_scp_p: Vector3<f64>, r_prime_scp_p: Vector3<f64>| {
            2.0 * w * r_prime_scp_p + w_prime * r_scp_p + w * w * r_scp_p + f.r_prime_prime_pb_p
        };
        let vec_rot_term4 = -m1 * r_tilde_sc1b_p * frame_relative_accel(r_sc1p_p, r_prime_sc1p_p)
            - m2 * r_tilde_sc2b_p * frame_relative_accel(r_sc2p_p, r_prime_sc2p_p);

        let int_term11 = -m1 * omega_tilde_pn_p * r_tilde_sc1p_p * (w * r_sc1p_p + f.r_prime_pb_p)
            - m2 * omega_tilde_pn_p * r_tilde_sc2p_p * (w * r_sc2p_p + f.r_prime_pb_p);
        let velocity1 = r_prime_sc1p_p + w * r_sc1p_p + f.r_prime_pb_p;
        let velocity2 = r_prime_sc2p_p + w * r_sc2p_p + f.r_prime_pb_p;
        let int_term12 = -m1 * omega_tilde_pn_p * r_tilde_pb_p * velocity1
            - m2 * omega_tilde_pn_p * r_tilde_pb_p * velocity2;
        let int_term13 = m1 * w * r_tilde_sc1b_p * velocity1 + m2 * w * r_tilde_sc2b_p * velocity2;
        let vec_rot_term5 = int_term11 + int_term12 + int_term13;

        let vec_rot_term6 = -int_term4 * joint_accel0 * s1_p - int_term6 * joint_accel1 * s2_p;
        let vec_rot_term7 = int_term3 * s1_p * c[0] + int_term5 * s2_p * c[1];

        back_sub.vec_rot += vec_rot_term1
            + vec_rot_term2
            + vec_rot_term3
            + vec_rot_term4
            + vec_rot_term5
            + vec_rot_term6
            + vec_rot_term7;
    }
}

/// Back-substitution contributions and the joint-acceleration terms
/// `θ̈ = A r̈_B + B ω̇ + C` cached for `compute_derivatives`.
#[derive(Clone, Debug, Default)]
struct TwoDofTerms {
    back_sub: BackSubMatrices,
    /// Upstream `ATheta`.
    a_theta: Matrix2x3<f64>,
    /// Upstream `BTheta`.
    b_theta: Matrix2x3<f64>,
    /// Upstream `CTheta`.
    c_theta: Vector2<f64>,
}

/// Geometry of both bodies for one state (upstream `updateEffectorMassProps`).
#[derive(Clone, Debug)]
struct TwoDofKinematics {
    theta: [f64; 2],
    /// Joint rates, zero for a locked axis.
    theta_dot: [f64; 2],
    /// Upstream `dcm_BS1`.
    dcm_bs1: Matrix3<f64>,
    /// Upstream `dcm_BS2`.
    dcm_bs2: Matrix3<f64>,
    /// Upstream `s1Hat_B`.
    s1_hat_b: Vector3<f64>,
    /// Upstream `s2Hat_B`.
    s2_hat_b: Vector3<f64>,
    /// Upstream `r_Sc1S1_B`.
    r_sc1s1_b: Vector3<f64>,
    /// Upstream `r_Sc1B_B`.
    r_sc1b_b: Vector3<f64>,
    /// Upstream `r_Sc2S2_B`.
    r_sc2s2_b: Vector3<f64>,
    /// Upstream `r_S2S1_B`.
    r_s2s1_b: Vector3<f64>,
    /// Upstream `r_Sc2S1_B`.
    r_sc2s1_b: Vector3<f64>,
    /// Upstream `r_Sc2B_B`.
    r_sc2b_b: Vector3<f64>,
    /// Upstream `IS1PntSc1_B`.
    i1_b: Matrix3<f64>,
    /// Upstream `IS2PntSc2_B`.
    i2_b: Matrix3<f64>,
    /// Upstream `omega_S1B_B`.
    omega_s1b_b: Vector3<f64>,
    /// Upstream `omega_S2S1_B`.
    omega_s2s1_b: Vector3<f64>,
    /// Upstream `omega_S2B_B`.
    omega_s2b_b: Vector3<f64>,
    /// Upstream `rPrime_Sc1S1_B`, which equals `rPrime_Sc1B_B`.
    r_prime_sc1s1_b: Vector3<f64>,
    /// Upstream `rPrime_Sc2S2_B`.
    r_prime_sc2s2_b: Vector3<f64>,
    /// Upstream `rPrime_S2S1_B`.
    r_prime_s2s1_b: Vector3<f64>,
    /// Upstream `rPrime_Sc2S1_B`, which equals `rPrime_Sc2B_B`.
    r_prime_sc2s1_b: Vector3<f64>,
    /// Upstream `IPrimeS1PntSc1_B`.
    i1_prime_b: Matrix3<f64>,
    /// Upstream `IPrimeS2PntSc2_B`.
    i2_prime_b: Matrix3<f64>,
}

impl TwoDofKinematics {
    fn new(config: &SpinningBodyTwoDofConfig, theta: [f64; 2], theta_dot: [f64; 2]) -> Self {
        let s1_hat_s1 = normalized_axis(config.body1.spin_axis_spinner);
        let s2_hat_s2 = normalized_axis(config.body2.spin_axis_spinner);
        let dcm_bs1 = config.body_to_spinner1_dcm.transpose() * prv_to_dcm(-theta[0] * s1_hat_s1);
        let dcm_bs2 = dcm_bs1
            * config.spinner1_to_spinner2_dcm.transpose()
            * prv_to_dcm(-theta[1] * s2_hat_s2);
        let s1_hat_b = dcm_bs1 * s1_hat_s1;
        let s2_hat_b = dcm_bs2 * s2_hat_s2;

        let r_sc1s1_b = dcm_bs1 * config.body1.center_of_mass_spinner_m;
        let r_sc2s2_b = dcm_bs2 * config.body2.center_of_mass_spinner_m;
        let r_s2s1_b = dcm_bs1 * config.spinner2_origin_spinner1_m;
        let r_sc2s1_b = r_sc2s2_b + r_s2s1_b;
        let i1_b = dcm_bs1 * config.body1.inertia_about_com_spinner_kg_m2 * dcm_bs1.transpose();
        let i2_b = dcm_bs2 * config.body2.inertia_about_com_spinner_kg_m2 * dcm_bs2.transpose();

        let omega_s1b_b = theta_dot[0] * s1_hat_b;
        let omega_s2s1_b = theta_dot[1] * s2_hat_b;
        let omega_s2b_b = omega_s2s1_b + omega_s1b_b;
        let omega_tilde_s1b = tilde(omega_s1b_b);
        let omega_tilde_s2b = tilde(omega_s2b_b);
        let r_prime_sc2s2_b = omega_tilde_s2b * r_sc2s2_b;
        let r_prime_s2s1_b = omega_tilde_s1b * r_s2s1_b;
        Self {
            theta,
            theta_dot,
            dcm_bs1,
            dcm_bs2,
            s1_hat_b,
            s2_hat_b,
            r_sc1s1_b,
            r_sc1b_b: r_sc1s1_b + config.spinner1_origin_body_m,
            r_sc2s2_b,
            r_s2s1_b,
            r_sc2s1_b,
            r_sc2b_b: r_sc2s1_b + config.spinner1_origin_body_m,
            i1_b,
            i2_b,
            omega_s1b_b,
            omega_s2s1_b,
            omega_s2b_b,
            r_prime_sc1s1_b: omega_tilde_s1b * r_sc1s1_b,
            r_prime_sc2s2_b,
            r_prime_s2s1_b,
            r_prime_sc2s1_b: r_prime_sc2s2_b + r_prime_s2s1_b,
            i1_prime_b: omega_tilde_s1b * i1_b - i1_b * omega_tilde_s1b,
            i2_prime_b: omega_tilde_s2b * i2_b - i2_b * omega_tilde_s2b,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_spinningBodyTwoDOFStateEffector.py` and
    //! `test_spinningBodyTwoDOFResetChecks.py`. The reset checks' "reset" and
    //! "attachment" paths are one check here,
    //! [`SpinningBodyTwoDofConfig::validate`].

    use super::{SpinningBodyTwoDofConfig, SpinningBodyTwoDofStateEffector};
    use crate::dynamics::spinning_body_one_dof_state_effector::{
        SpinningBodyOneDofConfig, SpinningBodyOneDofStateEffector,
    };
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, run_logging_every_step,
        spacecraft_with_effectors,
    };
    use crate::messages::{
        ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, HingedRigidBodyMsg, Output,
    };
    use crate::spacecraft::StateEffector;
    use nalgebra::{Matrix3, Vector3};

    const D2R: f64 = std::f64::consts::PI / 180.0;

    /// Upstream `spinningBody` of the integrated test, before the per-case
    /// rates and dampers.
    fn upstream_pair() -> SpinningBodyTwoDofConfig {
        let mut config = SpinningBodyTwoDofConfig::new("SpinningBody");
        config.body1.mass_kg = 100.0;
        config.body2.mass_kg = 50.0;
        config.body1.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(100.0, 50.0, 50.0));
        config.body2.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0));
        config.body_to_spinner1_dcm = Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0));
        config.spinner1_to_spinner2_dcm =
            Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        config.body1.center_of_mass_spinner_m = Vector3::new(2.0, -0.5, 0.0);
        config.body2.center_of_mass_spinner_m = Vector3::new(1.0, 0.0, -1.0);
        config.spinner1_origin_body_m = Vector3::new(-2.0, 0.5, -1.0);
        config.spinner2_origin_spinner1_m = Vector3::new(0.5, -1.5, -0.5);
        config.body1.spin_axis_spinner = Vector3::new(0.0, 0.0, 1.0);
        config.body2.spin_axis_spinner = Vector3::new(0.0, -1.0, 0.0);
        config.body1.theta_init_rad = 0.0;
        config.body2.theta_init_rad = 5.0 * D2R;
        config.body1.spring_constant_nm_per_rad = 1000.0;
        config.body2.spring_constant_nm_per_rad = 500.0;
        config
    }

    /// Upstream `test_spinningBody`: 5 s at a 0.2 ms step, conservation to
    /// 3e-13 and, with reference angles, convergence to them within 0.01 rad.
    fn spinning_body_test(cmd_torque_nm: [f64; 2], lock: [bool; 2], theta_ref_rad: [f64; 2]) {
        spinning_body_test_to(3e-13, cmd_torque_nm, lock, theta_ref_rad);
    }

    fn spinning_body_test_to(
        accuracy: f64,
        cmd_torque_nm: [f64; 2],
        lock: [bool; 2],
        theta_ref_rad: [f64; 2],
    ) {
        let mut config = upstream_pair();
        if theta_ref_rad != [0.0, 0.0] {
            config.body1.damping_nm_s_per_rad = 500.0;
            config.body2.damping_nm_s_per_rad = 200.0;
        }
        config.body1.theta_dot_init_radps = if lock[0] { 0.0 } else { 2.0 * D2R };
        config.body2.theta_dot_init_radps = if lock[1] { 0.0 } else { -D2R };
        let mut effector = SpinningBodyTwoDofStateEffector::new(config);

        let torque = Output::new(ArrayMotorTorqueMsg::from_active(&cmd_torque_nm));
        effector.motor_torque_in.connect(torque.slot());
        let mut lock_msg = ArrayEffectorLockMsg::default();
        for (index, locked) in lock.into_iter().enumerate() {
            if locked {
                lock_msg.effector_lock[index] = EffectorLock::Locked;
            }
        }
        let lock_out = Output::new(lock_msg);
        effector.motor_lock_in.connect(lock_out.slot());
        let references = theta_ref_rad.map(|theta_rad| {
            Output::new(HingedRigidBodyMsg {
                theta_rad,
                theta_dot_radps: 0.0,
            })
        });
        for (input, reference) in effector.spinning_body_ref_in.iter_mut().zip(&references) {
            input.connect(reference.slot());
        }
        let theta_out = effector.spinning_body_out.clone();

        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        hub.omega_bn_b_radps = Vector3::new(0.01, -0.01, 0.01);
        let mut spacecraft = spacecraft_with_effectors(&hub, 200_000, vec![Box::new(effector)]);
        add_point_mass_earth(&mut spacecraft);

        let log = run_logging_every_step(&mut spacecraft, 25_000, || {});

        let check_energy = cmd_torque_nm == [0.0, 0.0] && theta_ref_rad == [0.0, 0.0];
        assert_conserved(&log[0], &log[log.len() - 1], accuracy, check_energy);
        if theta_ref_rad != [0.0, 0.0] {
            for (output, theta_ref) in theta_out.iter().zip(theta_ref_rad) {
                let theta = output.read().theta_rad;
                assert!(
                    (theta - theta_ref).abs() <= 0.01,
                    "theta {theta} did not converge to {theta_ref}"
                );
            }
        }
        for (output, locked) in theta_out.iter().zip(lock) {
            if locked {
                assert_eq!(output.read().theta_dot_radps, 0.0);
            }
        }
    }

    #[test]
    fn spinning_body_free() {
        spinning_body_test([0.0, 0.0], [false, false], [0.0, 0.0]);
    }

    #[test]
    fn spinning_body_lower_axis_locked() {
        spinning_body_test([0.0, 0.0], [true, false], [0.0, 0.0]);
    }

    /// The x component of the rotational angular momentum is 28 times
    /// smaller than its magnitude here, and its relative drift reaches 3.8e-13
    /// by the end, above upstream's 3e-13: floating-point accumulation in the
    /// spacecraft integration (halving the step increases it; the N-DOF
    /// effector carrying the same pair drifts by 3.7e-13), so this case is
    /// checked to 5e-13.
    #[test]
    fn spinning_body_upper_axis_locked() {
        spinning_body_test_to(5e-13, [0.0, 0.0], [false, true], [0.0, 0.0]);
    }

    #[test]
    fn spinning_body_both_axes_locked() {
        spinning_body_test([0.0, 0.0], [true, true], [0.0, 0.0]);
    }

    #[test]
    fn spinning_body_commanded_torques() {
        spinning_body_test([1.0, -2.0], [false, false], [0.0, 0.0]);
    }

    #[test]
    fn spinning_body_reference_angles() {
        spinning_body_test([0.0, 0.0], [false, false], [10.0 * D2R, -5.0 * D2R]);
    }

    #[test]
    fn spinning_body_reference_angles_reversed() {
        spinning_body_test([0.0, 0.0], [false, false], [-5.0 * D2R, 10.0 * D2R]);
    }

    /// Upstream `_validSpinningBody` of the reset checks.
    fn valid_pair() -> SpinningBodyTwoDofConfig {
        let mut config = SpinningBodyTwoDofConfig::new("spinningBody");
        config.body1.mass_kg = 100.0;
        config.body2.mass_kg = 50.0;
        config.body1.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(100.0, 50.0, 50.0));
        config.body2.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0));
        config.body_to_spinner1_dcm = Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0));
        config.spinner1_to_spinner2_dcm =
            Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        config.body1.spin_axis_spinner = Vector3::x();
        config.body2.spin_axis_spinner = Vector3::new(0.0, -1.0, 0.0);
        config
    }

    /// Upstream `test_spinningBodyTwoDOF_rejectsInvalidConfiguration`.
    #[test]
    fn rejects_invalid_configuration() {
        type Break = fn(&mut SpinningBodyTwoDofConfig);
        let cases: [(&str, &str, Break); 6] = [
            ("dcm_S10B not orthogonal", "body_to_spinner1_dcm", |c| {
                c.body_to_spinner1_dcm = Matrix3::new(1.0, 0.5, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
            }),
            ("dcm_S20S1 left-handed", "spinner1_to_spinner2_dcm", |c| {
                c.spinner1_to_spinner2_dcm = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0))
            }),
            (
                "IS1PntSc1_S1 not symmetric",
                "body1.inertia_about_com",
                |c| {
                    c.body1.inertia_about_com_spinner_kg_m2 =
                        Matrix3::new(100.0, 5.0, 0.0, 0.0, 50.0, 0.0, 0.0, 0.0, 50.0)
                },
            ),
            (
                "IS2PntSc2_S2 not positive definite",
                "body2.inertia_about_com",
                |c| {
                    c.body2.inertia_about_com_spinner_kg_m2 =
                        Matrix3::from_diagonal(&Vector3::new(-50.0, 30.0, 40.0))
                },
            ),
            ("mass1 negative", "body1.mass_kg", |c| {
                c.body1.mass_kg = -1.0
            }),
            ("mass2 negative", "body2.mass_kg", |c| {
                c.body2.mass_kg = -1.0
            }),
        ];
        for (case, expected, break_it) in cases {
            let mut config = valid_pair();
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
    #[should_panic(expected = "spinner1_to_spinner2_dcm")]
    fn spacecraft_initialization_rejects_invalid_configuration() {
        let mut config = valid_pair();
        config.spinner1_to_spinner2_dcm = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0));
        let _ = SpinningBodyTwoDofStateEffector::new(config).initial_state();
    }

    /// Upstream `test_spinningBodyTwoDOF_resetAcceptsValidConfig` and
    /// `test_spinningBodyTwoDOF_resetAcceptsMasslessLowerBody`.
    #[test]
    fn accepts_valid_and_massless_lower_body_configurations() {
        assert_eq!(valid_pair().validate(), Ok(()));
        let mut massless = valid_pair();
        massless.body1.mass_kg = 0.0;
        massless.body1.inertia_about_com_spinner_kg_m2 = Matrix3::zeros();
        assert_eq!(massless.validate(), Ok(()));
    }

    /// With the upper axis locked and a massless, inertia-free lower body, the
    /// pair moves like a one-DOF spinner with the upper body's properties
    /// about the lower axis.
    #[test]
    fn locked_upper_axis_on_massless_lower_body_matches_one_dof_spinner() {
        let mut pair = SpinningBodyTwoDofConfig::new("pair");
        pair.body1.mass_kg = 0.0;
        pair.body1.inertia_about_com_spinner_kg_m2 = Matrix3::zeros();
        pair.body1.spin_axis_spinner = Vector3::new(0.0, 0.0, 1.0);
        pair.body1.theta_init_rad = 0.1;
        pair.body1.theta_dot_init_radps = 0.05;
        pair.body1.spring_constant_nm_per_rad = 20.0;
        pair.body2.mass_kg = 30.0;
        pair.body2.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(10.0, 12.0, 14.0));
        pair.body2.center_of_mass_spinner_m = Vector3::new(0.5, 0.2, 0.0);
        pair.spinner1_origin_body_m = Vector3::new(1.0, -0.5, 0.3);
        let mut two_dof = SpinningBodyTwoDofStateEffector::new(pair);
        two_dof.locked = [false, true];

        let mut single = SpinningBodyOneDofConfig::new("single");
        single.mass_kg = 30.0;
        single.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(10.0, 12.0, 14.0));
        single.center_of_mass_spinner_m = Vector3::new(0.5, 0.2, 0.0);
        single.spin_axis_spinner = Vector3::new(0.0, 0.0, 1.0);
        single.spinner_origin_body_m = Vector3::new(1.0, -0.5, 0.3);
        single.spring_constant_nm_per_rad = 20.0;
        let one_dof = SpinningBodyOneDofStateEffector::new(single);

        let omega = Vector3::new(0.02, -0.03, 0.01);
        let gravity = Vector3::new(0.1, 0.2, -0.3);
        let (state2, state1) = ([0.1, 0.05, 0.0, 0.0], [0.1, 0.05]);
        let mut back_sub2 = Default::default();
        let mut back_sub1 = Default::default();
        let hub = crate::messages::SpacecraftStateMsg {
            omega_radps: omega,
            ..Default::default()
        };
        two_dof.update_contributions(&state2, &hub, gravity, &mut back_sub2);
        one_dof.update_contributions(&state1, &hub, gravity, &mut back_sub1);
        let accel = Vector3::new(0.3, -0.1, 0.2);
        let omega_dot = Vector3::new(-0.01, 0.02, 0.005);
        let derivative2 = two_dof.compute_derivatives(&state2, accel, omega_dot);
        let derivative1 = one_dof.compute_derivatives(&state1, accel, omega_dot);

        assert!((derivative2[1] - derivative1[1]).abs() < 1e-12);
        assert_eq!(derivative2[3], 0.0);
        let close = |a: Matrix3<f64>, b: Matrix3<f64>| (a - b).norm() < 1e-10;
        assert!(close(back_sub2.matrix_a, back_sub1.matrix_a));
        assert!(close(back_sub2.matrix_d, back_sub1.matrix_d));
        assert!((back_sub2.vec_rot - back_sub1.vec_rot).norm() < 1e-10);
        assert!((back_sub2.vec_trans - back_sub1.vec_trans).norm() < 1e-10);
        let props2 = two_dof.mass_properties(&state2);
        let props1 = one_dof.mass_properties(&state1);
        assert!(
            (props2.inertia_about_point_b_body_kg_m2 - props1.inertia_about_point_b_body_kg_m2)
                .norm()
                < 1e-10
        );
    }
}
