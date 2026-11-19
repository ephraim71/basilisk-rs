//! A rigid body sliding along one hub-fixed axis: port of upstream Basilisk
//! `simulation/dynamics/linearTranslationalBodies/linearTranslationBodiesOneDOF`
//! (`LinearTranslationOneDOFStateEffector`).
//!
//! The body's frame F is fixed in orientation relative to the body frame B
//! (`body_to_translating_dcm`, upstream `dcm_FB`). Its origin moves from F0,
//! at `origin0_body_m` (`r_F0B_B`), a distance ρ along the axis f̂
//! (`fHat_B`), and its center of mass lies at `center_of_mass_translating_m`
//! (`r_FcF_F`) from the F origin. The joint carries a linear spring–damper
//! acting on `ρ − ρ_ref`, `ρ̇ − ρ̇_ref` and an optional motor force.
//!
//! State layout: `[ρ, ρ̇]`.
//!
//! Inputs are read in `pre_integration`, before each integration step
//! (upstream reads them in `UpdateState`); a connected input counts as
//! written. The body can ride on a
//! [`PrescribedMotionStateEffector`](super::prescribed_motion_state_effector::PrescribedMotionStateEffector).
//!
//! Differences from upstream:
//! - `addDynamicEffector` is not ported, as in
//!   [`super::hinged_rigid_body_state_effector`]: the attached-load terms are
//!   zero.
//! - A locked axis is evaluated with ρ̇ = 0 throughout the step and the
//!   committed ρ̇ is zeroed in `modify_states` (and, if locked at
//!   initialization, in the initial state). Upstream zeroes the ρ̇ state
//!   only after reading it, so its first evaluation after the lock engages
//!   still uses the old rate in the mass-property rates.
//! - Upstream's setters validate the mass, `k`, `c` and `fHat_B` as they are
//!   set; here [`LinearTranslationOneDofConfig::validate`] checks them when
//!   the spacecraft initializes.
//! - Upstream's prescribed-motion coupling computes the rotational term
//!   `−m ω̃_BN r̃_FcB (ω̃_PB r_FcP + r'_PB)` but drops it (a stray semicolon
//!   ends the statement before it); it is included here, as the spinning
//!   bodies include their equivalent term.

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
    ArrayEffectorLockMsg, ArrayMotorForceMsg, EffectorLock, Input, LinearTranslationRigidBodyMsg,
    Output, SpacecraftStateMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::avs_eigen_support::{
    DEFAULT_TOLERANCE, is_rotation_matrix, is_valid_inertia_matrix,
};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm, tilde};

/// User-settable parameters of [`LinearTranslationOneDofStateEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct LinearTranslationOneDofConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mass` \[kg\] (default 1; must be positive).
    pub mass_kg: f64,
    /// Upstream `k` \[N/m\]: linear spring constant (default 0).
    pub spring_constant_n_per_m: f64,
    /// Upstream `c` \[N s/m\]: linear damping coefficient (default 0).
    pub damping_n_s_per_m: f64,
    /// Upstream `rhoInit` \[m\] (default 0).
    pub rho_init_m: f64,
    /// Upstream `rhoDotInit` \[m/s\] (default 0).
    pub rho_dot_init_mps: f64,
    /// Upstream `fHat_B`: translation axis in B components, normalized when
    /// used (default `[1, 0, 0]`).
    pub axis_body: Vector3<f64>,
    /// Upstream `r_FcF_F` \[m\]: center of mass relative to the F origin, in
    /// F components (default zero).
    pub center_of_mass_translating_m: Vector3<f64>,
    /// Upstream `r_F0B_B` \[m\]: F origin at ρ = 0 relative to the body origin
    /// (default zero).
    pub origin0_body_m: Vector3<f64>,
    /// Upstream `IPntFc_F` \[kg m^2\]: inertia about the center of mass in F
    /// components (default identity).
    pub inertia_about_com_translating_kg_m2: Matrix3<f64>,
    /// Upstream `dcm_FB`: DCM from B to F (default identity).
    pub body_to_translating_dcm: Matrix3<f64>,
}

impl LinearTranslationOneDofConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mass_kg: 1.0,
            spring_constant_n_per_m: 0.0,
            damping_n_s_per_m: 0.0,
            rho_init_m: 0.0,
            rho_dot_init_mps: 0.0,
            axis_body: Vector3::x(),
            center_of_mass_translating_m: Vector3::zeros(),
            origin0_body_m: Vector3::zeros(),
            inertia_about_com_translating_kg_m2: Matrix3::identity(),
            body_to_translating_dcm: Matrix3::identity(),
        }
    }

    /// Upstream `validateConfiguration` and the checks of upstream's setters:
    /// every value finite, the mass positive, `k` and `c` non-negative, the
    /// axis norm above 0.01, `body_to_translating_dcm` a rotation matrix and
    /// a valid inertia tensor.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("mass_kg", self.mass_kg),
            ("spring_constant_n_per_m", self.spring_constant_n_per_m),
            ("damping_n_s_per_m", self.damping_n_s_per_m),
            ("rho_init_m", self.rho_init_m),
            ("rho_dot_init_mps", self.rho_dot_init_mps),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite"));
            }
        }
        for (label, finite) in [
            ("axis_body", all_finite(self.axis_body.as_slice())),
            (
                "center_of_mass_translating_m",
                all_finite(self.center_of_mass_translating_m.as_slice()),
            ),
            ("origin0_body_m", all_finite(self.origin0_body_m.as_slice())),
        ] {
            if !finite {
                return Err(format!("{label} must contain only finite values"));
            }
        }
        if self.mass_kg <= 0.0 {
            return Err("mass_kg must be greater than 0".to_string());
        }
        for (label, value) in [
            ("spring_constant_n_per_m", self.spring_constant_n_per_m),
            ("damping_n_s_per_m", self.damping_n_s_per_m),
        ] {
            if value < 0.0 {
                return Err(format!("{label} must be greater than or equal to 0"));
            }
        }
        if self.axis_body.norm() <= 0.01 {
            return Err("the norm of axis_body must exceed 0.01".to_string());
        }
        if !is_rotation_matrix(&self.body_to_translating_dcm, DEFAULT_TOLERANCE) {
            return Err(
                "body_to_translating_dcm must be an orthogonal, right-handed rotation matrix"
                    .to_string(),
            );
        }
        if !is_valid_inertia_matrix(&self.inertia_about_com_translating_kg_m2, DEFAULT_TOLERANCE) {
            return Err(
                "inertia_about_com_translating_kg_m2 must be a valid inertia tensor".to_string(),
            );
        }
        Ok(())
    }
}

/// A rigid body sliding along one axis (upstream
/// `LinearTranslationOneDOFStateEffector`).
#[derive(Debug)]
pub struct LinearTranslationOneDofStateEffector {
    pub config: LinearTranslationOneDofConfig,
    /// Upstream `motorForceInMsg` (optional): entry 0 is the motor force.
    pub motor_force_in: Input<ArrayMotorForceMsg>,
    /// Upstream `motorLockInMsg` (optional): entry 0 locks the axis.
    pub motor_lock_in: Input<ArrayEffectorLockMsg>,
    /// Upstream `translatingBodyRefInMsg` (optional): reference displacement
    /// and rate of the spring–damper.
    pub translating_body_ref_in: Input<LinearTranslationRigidBodyMsg>,
    /// Upstream `translatingBodyOutMsg`: displacement and rate.
    pub translating_body_out: Output<LinearTranslationRigidBodyMsg>,
    /// Upstream `translatingBodyConfigLogOutMsg`: inertial state of the body,
    /// with F as body frame and its center of mass as position.
    pub translating_body_state_out: Output<SpacecraftStateMsg>,
    /// Upstream `motorForce` \[N\].
    pub motor_force_n: f64,
    /// Upstream `isAxisLocked`.
    pub locked: bool,
    /// Upstream `rhoRef` \[m\].
    pub rho_ref_m: f64,
    /// Upstream `rhoDotRef` \[m/s\].
    pub rho_dot_ref_mps: f64,
    /// Last loaded `[ρ, ρ̇]`.
    state: [f64; 2],
    back_sub: RefCell<LinearOneDofTerms>,
    kinematics: KinematicsCache<LinearTranslationOneDofConfig, LinearOneDofKinematics>,
}

impl LinearTranslationOneDofStateEffector {
    pub fn new(config: LinearTranslationOneDofConfig) -> Self {
        let state = [config.rho_init_m, config.rho_dot_init_mps];
        Self {
            config,
            motor_force_in: Input::default(),
            motor_lock_in: Input::default(),
            translating_body_ref_in: Input::default(),
            translating_body_out: Output::default(),
            translating_body_state_out: Output::default(),
            motor_force_n: 0.0,
            locked: false,
            rho_ref_m: 0.0,
            rho_dot_ref_mps: 0.0,
            state,
            back_sub: RefCell::new(LinearOneDofTerms::default()),
            kinematics: KinematicsCache::default(),
        }
    }

    /// `[ρ, ρ̇]` of the last loaded state.
    pub fn state(&self) -> [f64; 2] {
        self.state
    }

    /// Inertial state of the body (upstream
    /// `computeTranslatingBodyInertialStates`) for the last loaded state and
    /// the given state of the frame the body is mounted on.
    pub fn translating_inertial_state(&self, hub_state: &SpacecraftStateMsg) -> SpacecraftStateMsg {
        let k = self.kinematics(&self.state);
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        let dcm_fb = self.config.body_to_translating_dcm;
        SpacecraftStateMsg {
            position_m: hub_state.position_m + dcm_nb * k.r_fcb_b,
            velocity_mps: hub_state.velocity_mps
                + dcm_nb * (k.r_prime_fcb_b + omega_bn_b.cross(&k.r_fcb_b)),
            sigma_bn: dcm_to_mrp(dcm_fb * dcm_bn),
            omega_radps: dcm_fb * omega_bn_b,
            ..Default::default()
        }
    }

    fn kinematics(&self, effector_state: &[f64]) -> LinearOneDofKinematics {
        assert_eq!(
            effector_state.len(),
            2,
            "linear translation one DOF state length mismatch"
        );
        let key = [
            effector_state[0],
            if self.locked { 0.0 } else { effector_state[1] },
        ];
        self.kinematics.get_or_compute(&key, &self.config, || {
            LinearOneDofKinematics::new(&self.config, key[0], key[1])
        })
    }

    /// Upstream `computeBackSubContributions`.
    fn back_sub_terms(
        &self,
        k: &LinearOneDofKinematics,
        omega_bn_b: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> LinearOneDofTerms {
        if self.locked {
            return LinearOneDofTerms::default();
        }
        let m = self.config.mass_kg;
        let f_hat = k.f_hat_b;
        let a_rho = -f_hat;
        let b_rho = f_hat.cross(&k.r_fcb_b);
        let gravity_force = m * gravity_body_mps2;
        let c_rho = (self.motor_force_n
            - self.config.spring_constant_n_per_m * (k.rho - self.rho_ref_m)
            - self.config.damping_n_s_per_m * (k.rho_dot - self.rho_dot_ref_mps)
            + f_hat.dot(
                &(gravity_force
                    - m * (2.0 * omega_bn_b.cross(&k.r_prime_fcb_b)
                        + omega_bn_b.cross(&omega_bn_b.cross(&k.r_fcb_b)))),
            ))
            / m;
        let r_cross_f_hat = k.r_fcb_b.cross(&f_hat);
        LinearOneDofTerms {
            back_sub: BackSubMatrices {
                matrix_a: m * f_hat * a_rho.transpose(),
                matrix_b: m * f_hat * b_rho.transpose(),
                matrix_c: m * r_cross_f_hat * a_rho.transpose(),
                matrix_d: m * r_cross_f_hat * b_rho.transpose(),
                vec_trans: -m * c_rho * f_hat,
                vec_rot: -m * omega_bn_b.cross(&k.r_fcb_b.cross(&k.r_prime_fcb_b))
                    - m * c_rho * r_cross_f_hat,
            },
            a_rho,
            b_rho,
            c_rho,
        }
    }
}

impl StateEffector for LinearTranslationOneDofStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        2
    }

    /// Validates the configuration (upstream `registerStates`) and returns the
    /// initial `[ρ, ρ̇]`, with ρ̇ zeroed if `motor_lock_in` already locks the
    /// axis.
    ///
    /// # Panics
    ///
    /// If [`LinearTranslationOneDofConfig::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid LinearTranslationOneDofConfig '{}': {message}",
                self.config.name
            );
        }
        let locked = self.motor_lock_in.is_connected()
            && self.motor_lock_in.read().effector_lock[0] == EffectorLock::Locked;
        let rho_dot = if locked {
            0.0
        } else {
            self.config.rho_dot_init_mps
        };
        vec![self.config.rho_init_m, rho_dot]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            2,
            "linear translation one DOF state length mismatch"
        );
        self.state = [state[0], state[1]];
    }

    /// Upstream `readInputMessages`.
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {
        if self.motor_force_in.is_connected() {
            self.motor_force_n = self.motor_force_in.read().motor_force_n[0];
        }
        if self.motor_lock_in.is_connected() {
            self.locked = self.motor_lock_in.read().effector_lock[0] == EffectorLock::Locked;
        }
        if self.translating_body_ref_in.is_connected() {
            let reference = self.translating_body_ref_in.read();
            self.rho_ref_m = reference.rho_m;
            self.rho_dot_ref_mps = reference.rho_dot_mps;
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
        let rho_ddot = terms.a_rho.dot(&body_trans_accel_mps2)
            + terms.b_rho.dot(&body_omega_dot_radps2)
            + terms.c_rho;
        vec![k.rho_dot, rho_ddot]
    }

    /// Upstream `updateEffectorMassProps`.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let k = self.kinematics(effector_state);
        let m = self.config.mass_kg;
        let r_tilde = tilde(k.r_fcb_b);
        let r_prime_tilde = tilde(k.r_prime_fcb_b);
        StateEffectorMassProps {
            mass_kg: m,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m: k.r_fcb_b,
            center_of_mass_prime_body_mps: k.r_prime_fcb_b,
            inertia_about_point_b_body_kg_m2: k.i_pnt_fc_b + m * r_tilde * r_tilde.transpose(),
            inertia_about_point_b_body_prime_kg_m2ps: -m
                * (r_prime_tilde * r_tilde + r_tilde * r_prime_tilde),
        }
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B); the
    /// body's frame shares the hub's rate.
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let k = self.kinematics(effector_state);
        let r_dot_fcb_b = k.r_prime_fcb_b + body_omega_radps.cross(&k.r_fcb_b);
        k.i_pnt_fc_b * body_omega_radps + self.config.mass_kg * k.r_fcb_b.cross(&r_dot_fcb_b)
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the spring energy.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let k = self.kinematics(effector_state);
        let r_dot_fcb_b = k.r_prime_fcb_b + body_omega_radps.cross(&k.r_fcb_b);
        let deflection_m = k.rho - self.rho_ref_m;
        0.5 * body_omega_radps.dot(&(k.i_pnt_fc_b * body_omega_radps))
            + 0.5 * self.config.mass_kg * r_dot_fcb_b.norm_squared()
            + 0.5 * self.config.spring_constant_n_per_m * deflection_m * deflection_m
    }

    fn write_outputs(&mut self, _current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.translating_body_out
            .write(LinearTranslationRigidBodyMsg {
                rho_m: self.state[0],
                rho_dot_mps: self.state[1],
            });
        self.translating_body_state_out
            .write(self.translating_inertial_state(hub_state));
    }

    /// Zeroes ρ̇ of a locked axis (upstream overwrites the state the same way).
    fn modify_states(&mut self, _current_sim_nanos: u64, effector_state: &mut [f64]) {
        if self.locked {
            effector_state[1] = 0.0;
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrescribedMotionAttachment for LinearTranslationOneDofStateEffector {
    /// Upstream `addPrescribedMotionCouplingContributions`. The body's B frame
    /// is the prescribed frame P here.
    fn add_prescribed_motion_coupling(
        &self,
        effector_state: &[f64],
        motion: &PrescribedFrameMotion,
        back_sub: &mut BackSubMatrices,
    ) {
        let k = self.kinematics(effector_state);
        let terms = self.back_sub.borrow();
        let (a, b, c) = (terms.a_rho, terms.b_rho, terms.c_rho);
        let m = self.config.mass_kg;
        let f = motion.in_child_frame();
        let f_hat_p = k.f_hat_b;
        let r_prime_fcp_p = k.r_prime_fcb_b;
        let r_fcp_p = k.r_fcb_b;
        let w = f.omega_tilde_pb_p;

        back_sub.matrix_b += -m * f_hat_p * a.transpose() * f.r_tilde_pb_p;

        let term1 = 2.0 * w * r_prime_fcp_p
            + f.omega_prime_tilde_pb_p * r_fcp_p
            + w * w * r_fcp_p
            + f.r_prime_prime_pb_p;
        back_sub.vec_trans += -m * term1
            - m * a.dot(&f.frame_accel_p) * f_hat_p
            - m * b.dot(&f.frame_ang_accel_p) * f_hat_p;

        back_sub.matrix_c += m * f.r_tilde_pb_p * f_hat_p * a.transpose();
        let r_tilde_fcb_p = tilde(r_fcp_p + f.r_pb_p);
        back_sub.matrix_d += m * f.r_tilde_pb_p * f_hat_p * b.transpose()
            - m * r_tilde_fcb_p * f_hat_p * a.transpose() * f.r_tilde_pb_p;

        let i_pnt_fc_p = k.i_pnt_fc_b;
        let omega_tilde_pn_p = tilde(f.omega_pn_p);
        let omega_tilde_bn_p = omega_tilde_pn_p - w;
        let r_tilde_fcp_p = tilde(r_fcp_p);
        let vec_rot_term1 =
            -i_pnt_fc_p * f.omega_prime_pb_p - omega_tilde_pn_p * i_pnt_fc_p * f.omega_pb_p;
        let vec_rot_term2 = -m * r_tilde_fcb_p * term1;
        let vec_rot_term3 =
            -m * (omega_tilde_bn_p * f.r_tilde_pb_p - w * r_tilde_fcp_p) * r_prime_fcp_p
                - m * omega_tilde_bn_p * r_tilde_fcb_p * (w * r_fcp_p + f.r_prime_pb_p);
        let vec_rot_term4 = -m * c * f.r_tilde_pb_p * f_hat_p;
        let vec_rot_term5 = -m * r_tilde_fcb_p * f_hat_p * a.dot(&f.frame_accel_p)
            - m * r_tilde_fcb_p * f_hat_p * b.dot(&f.frame_ang_accel_p);
        back_sub.vec_rot +=
            vec_rot_term1 + vec_rot_term2 + vec_rot_term3 + vec_rot_term4 + vec_rot_term5;
    }
}

/// Back-substitution contributions and the `ρ̈ = a·r̈_B + b·ω̇ + c` terms
/// cached for `compute_derivatives`; all zero for a locked axis.
#[derive(Clone, Debug, Default)]
struct LinearOneDofTerms {
    back_sub: BackSubMatrices,
    /// Upstream `aRho`.
    a_rho: Vector3<f64>,
    /// Upstream `bRho`.
    b_rho: Vector3<f64>,
    /// Upstream `cRho`.
    c_rho: f64,
}

/// The body's geometry for one state (upstream `updateEffectorMassProps`).
#[derive(Clone, Debug)]
struct LinearOneDofKinematics {
    rho: f64,
    /// ρ̇, zero for a locked axis.
    rho_dot: f64,
    /// Upstream `fHat_B`, normalized.
    f_hat_b: Vector3<f64>,
    /// Upstream `r_FcB_B`.
    r_fcb_b: Vector3<f64>,
    /// Upstream `rPrime_FcB_B`.
    r_prime_fcb_b: Vector3<f64>,
    /// Upstream `IPntFc_B`.
    i_pnt_fc_b: Matrix3<f64>,
}

impl LinearOneDofKinematics {
    fn new(config: &LinearTranslationOneDofConfig, rho: f64, rho_dot: f64) -> Self {
        let f_hat_b = normalized_axis(config.axis_body);
        let dcm_bf = config.body_to_translating_dcm.transpose();
        let r_fcf0_b = dcm_bf * config.center_of_mass_translating_m + rho * f_hat_b;
        Self {
            rho,
            rho_dot,
            f_hat_b,
            r_fcb_b: config.origin0_body_m + r_fcf0_b,
            r_prime_fcb_b: rho_dot * f_hat_b,
            i_pnt_fc_b: dcm_bf * config.inertia_about_com_translating_kg_m2 * dcm_bf.transpose(),
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_linearTranslationOneDOFStateEffector.py`. The
    //! configuration-validation test's `scheduleEffector` parameter (whether
    //! upstream's `Reset` also runs) has no counterpart: validation runs once,
    //! when the spacecraft initializes.

    use super::{LinearTranslationOneDofConfig, LinearTranslationOneDofStateEffector};
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, run_logging_every_step,
        spacecraft_with_effectors,
    };
    use crate::messages::{
        ArrayEffectorLockMsg, ArrayMotorForceMsg, EffectorLock, LinearTranslationRigidBodyMsg,
        Output,
    };
    use crate::spacecraft::StateEffector;
    use nalgebra::{Matrix3, Vector3};

    /// Upstream `translatingBody` of the integrated tests.
    pub(crate) fn upstream_translating_body() -> LinearTranslationOneDofConfig {
        let mut config = LinearTranslationOneDofConfig::new("translatingBody");
        config.mass_kg = 20.0;
        config.rho_init_m = 1.0;
        config.rho_dot_init_mps = 0.05;
        config.axis_body = Vector3::new(3.0 / 5.0, 4.0 / 5.0, 0.0);
        config.center_of_mass_translating_m = Vector3::new(-1.0, 1.0, 0.0);
        config.origin0_body_m = Vector3::new(-5.0, 4.0, 3.0);
        config.inertia_about_com_translating_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, 60.0));
        config.body_to_translating_dcm =
            Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        config.spring_constant_n_per_m = 100.0;
        config
    }

    /// The integrated tests: `steps` steps of 0.1 ms, conservation to 1e-13.
    fn integrated_test(
        effector: LinearTranslationOneDofStateEffector,
        steps: usize,
        check_rotational_energy: bool,
    ) {
        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.center_of_mass_body_m = Vector3::new(0.0, 0.0, 1.0);
        hub.omega_bn_b_radps = Vector3::new(0.1, -0.1, 0.1);
        let mut spacecraft = spacecraft_with_effectors(&hub, 100_000, vec![Box::new(effector)]);
        add_point_mass_earth(&mut spacecraft);

        let log = run_logging_every_step(&mut spacecraft, steps, || {});

        assert_conserved(&log[0], &log[log.len() - 1], 1e-13, check_rotational_energy);
    }

    /// Upstream `translatingBodyNoInput`.
    #[test]
    fn translating_body_no_input() {
        let effector = LinearTranslationOneDofStateEffector::new(upstream_translating_body());
        integrated_test(effector, 25_000, true);
    }

    /// Upstream `translatingBodyLockFlag`.
    #[test]
    fn translating_body_lock_flag() {
        let mut config = upstream_translating_body();
        config.rho_dot_init_mps = 0.0;
        let mut effector = LinearTranslationOneDofStateEffector::new(config);
        let mut lock = ArrayEffectorLockMsg::default();
        lock.effector_lock[0] = EffectorLock::Locked;
        let lock = Output::new(lock);
        effector.motor_lock_in.connect(lock.slot());
        let rho = effector.translating_body_out.clone();
        integrated_test(effector, 25_000, true);
        assert_eq!(
            rho.read(),
            LinearTranslationRigidBodyMsg {
                rho_m: 1.0,
                rho_dot_mps: 0.0
            }
        );
    }

    /// Upstream `translatingBodyCommandedForce` with a 1 N force.
    #[test]
    fn translating_body_commanded_force() {
        let mut effector = LinearTranslationOneDofStateEffector::new(upstream_translating_body());
        let force = Output::new(ArrayMotorForceMsg::from_active(&[1.0]));
        effector.motor_force_in.connect(force.slot());
        integrated_test(effector, 25_000, false);
    }

    /// Upstream `translatingBodyRhoReference` with a 0.5 m reference: 3 s,
    /// the displacement settling within 0.01 m of it.
    #[test]
    fn translating_body_rho_reference() {
        let mut config = upstream_translating_body();
        config.damping_n_s_per_m = 30.0;
        let mut effector = LinearTranslationOneDofStateEffector::new(config);
        let reference = Output::new(LinearTranslationRigidBodyMsg {
            rho_m: 0.5,
            rho_dot_mps: 0.0,
        });
        effector.translating_body_ref_in.connect(reference.slot());
        let rho = effector.translating_body_out.clone();
        integrated_test(effector, 30_000, false);
        let final_rho = rho.read().rho_m;
        assert!(
            (final_rho - 0.5).abs() <= 0.01,
            "rho settled at {final_rho}"
        );
    }

    /// Upstream `test_translatingBodyConfigurationValidation`.
    #[test]
    fn configuration_validation() {
        let cases = [
            ("Valid", false),
            ("SkewedDCM", true),
            ("MirroredDCM", true),
            ("AsymmetricInertia", true),
            ("NegativeInertia", true),
            ("TriangleInertia", true),
        ];
        for (configuration, should_fail) in cases {
            let mut config = LinearTranslationOneDofConfig::new("translatingBody");
            config.mass_kg = 20.0;
            config.axis_body = Vector3::x();
            config.origin0_body_m = Vector3::new(1.0, 0.5, 0.25);
            config.body_to_translating_dcm = match configuration {
                "SkewedDCM" => Matrix3::new(1.0, 0.1, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
                "MirroredDCM" => Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0)),
                _ => Matrix3::identity(),
            };
            config.inertia_about_com_translating_kg_m2 = match configuration {
                "AsymmetricInertia" => Matrix3::new(50.0, 3.0, 0.0, 0.0, 80.0, 0.0, 0.0, 0.0, 60.0),
                "NegativeInertia" => Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, -10.0)),
                "TriangleInertia" => Matrix3::from_diagonal(&Vector3::new(10.0, 10.0, 90.0)),
                _ => Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, 60.0)),
            };
            assert_eq!(config.validate().is_err(), should_fail, "{configuration}");
        }
    }

    #[test]
    fn validation_rejects_setter_violations() {
        let mut config = upstream_translating_body();
        config.mass_kg = 0.0;
        assert!(config.validate().unwrap_err().contains("mass_kg"));
        let mut config = upstream_translating_body();
        config.damping_n_s_per_m = -1.0;
        assert!(config.validate().unwrap_err().contains("damping"));
        let mut config = upstream_translating_body();
        config.axis_body = Vector3::new(0.0, 0.005, 0.0);
        assert!(config.validate().unwrap_err().contains("axis_body"));
    }

    #[test]
    #[should_panic(expected = "body_to_translating_dcm")]
    fn spacecraft_initialization_rejects_invalid_configuration() {
        let mut config = upstream_translating_body();
        config.body_to_translating_dcm = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0));
        let _ = LinearTranslationOneDofStateEffector::new(config).initial_state();
    }

    #[test]
    fn mass_properties_slide_along_the_axis() {
        let mut config = LinearTranslationOneDofConfig::new("slider");
        config.mass_kg = 2.0;
        config.axis_body = Vector3::new(0.0, 2.0, 0.0);
        config.origin0_body_m = Vector3::new(1.0, 0.0, 0.0);
        let effector = LinearTranslationOneDofStateEffector::new(config);

        let props = effector.mass_properties(&[0.5, 0.2]);

        assert_eq!(props.center_of_mass_body_m, Vector3::new(1.0, 0.5, 0.0));
        assert_eq!(
            props.center_of_mass_prime_body_mps,
            Vector3::new(0.0, 0.2, 0.0)
        );
        let r = props.center_of_mass_body_m;
        let expected = Matrix3::identity()
            + 2.0 * (r.norm_squared() * Matrix3::identity() - r * r.transpose());
        assert!((props.inertia_about_point_b_body_kg_m2 - expected).norm() < 1e-12);
    }
}
