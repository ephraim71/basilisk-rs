//! A rigid body whose motion relative to the hub is prescribed: port of
//! upstream Basilisk `simulation/dynamics/prescribedMotion`
//! (`PrescribedMotionStateEffector`).
//!
//! The body frame P moves relative to a hub-fixed mount frame M, which sits
//! at `mount_position_body_m` (`r_MB_B`) with attitude `sigma_mb`
//! (`sigma_MB`). The translation `r_PM_M`, `r'_PM_M`, `r''_PM_M` and the
//! rotation `ω_PM_P`, `ω'_PM_P`, `σ_PM` come from the
//! [`PrescribedTranslationMsg`] and [`PrescribedRotationMsg`] inputs, for
//! example from
//! [`PrescribedRot2Dof`](crate::fsw_algorithms::effector_interfaces::prescribed_rot_2dof::PrescribedRot2Dof).
//! The body adds no degrees of freedom: it loads the hub through its mass
//! properties and the inertial forces of the prescribed motion.
//!
//! Inputs are read in `pre_integration`, before each integration step, and
//! taken as the prescribed state at the start of the step; a connected input
//! counts as written. Within the step the translation is extrapolated with
//! constant `r''_PM_M` and the rate with constant `ω'_PM_P`, and `σ_PM` is
//! integrated from the rate, as upstream does from `UpdateState`'s epoch. A
//! profiler should therefore be scheduled after the spacecraft, so that the
//! message it wrote at the previous tick holds the state at the start of the
//! step. Without a connected input the prescribed state continues from the
//! configured initial values with constant acceleration.
//!
//! Other state effectors can ride on the body (upstream "branching",
//! [`add_state_effector`](PrescribedMotionStateEffector::add_state_effector)):
//! their hub frame is P, and they implement [`PrescribedMotionAttachment`] to
//! add the coupling to P's prescribed motion. The spinning bodies with one and
//! two degrees of freedom and the one-degree-of-freedom translating body
//! implement it.
//!
//! The spacecraft hands state effectors neither the integration time nor a
//! way to overwrite their state before a step, so the effector's state is
//! `[σ_PM (3), τ, attached states…]`, where τ is the time since the start of
//! the step (upstream `integTime − currentSimTimeSec`), integrated at rate 1.
//! The reset of `σ_PM` to the commanded attitude (upstream
//! `sigma_PMState->setState`) is applied as a shift of the integrated value
//! during the step and folded into the state, with τ reset to zero, in
//! `modify_states`.
//!
//! Differences from upstream:
//! - Upstream discards the configured `r_PM_M`, `rPrime_PM_M` and
//!   `omega_PM_P` at the first mass-property update when no message has been
//!   read, and restarts the prescribed rates from the last message (or from
//!   zero) at every step. Here the configured values start the motion and,
//!   between messages, the state continues with constant acceleration.
//! - The output messages carry the prescribed state at the end of the step
//!   rather than the message read for the next one.

use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::cell::Cell;
use std::fmt;

use crate::dynamics::state_effector_support::{KinematicsCache, add_back_sub};
use crate::messages::{
    Input, Output, PrescribedRotationMsg, PrescribedTranslationMsg, SpacecraftStateMsg,
};
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::rigid_body_kinematics::{b_mat_mrp, dcm_to_mrp, mrp_to_dcm, tilde};

/// Leading entries of the state vector before the attached effectors':
/// `σ_PM` and the step clock τ.
const OWN_STATE_LEN: usize = 4;

/// Motion of a prescribed body frame P relative to the hub frame B, as an
/// attached state effector needs it for its coupling terms (upstream's
/// prescribed motion properties `r_PB_B`, `rPrime_PB_B`, `rPrimePrime_PB_B`,
/// `sigma_PB`, `omega_PB_P`, `omegaPrime_PB_P` and the hub's `hubOmega`
/// state).
#[derive(Clone, Debug)]
pub struct PrescribedFrameMotion {
    /// Upstream `r_PB_B` \[m\].
    pub r_pb_b_m: Vector3<f64>,
    /// Upstream `rPrime_PB_B` \[m/s\]: B-frame derivative of `r_pb_b_m`.
    pub r_prime_pb_b_mps: Vector3<f64>,
    /// Upstream `rPrimePrime_PB_B` \[m/s^2\].
    pub r_prime_prime_pb_b_mps2: Vector3<f64>,
    /// `[PB]`, the DCM of upstream `sigma_PB`.
    pub dcm_pb: Matrix3<f64>,
    /// Upstream `omega_PB_P` \[rad/s\].
    pub omega_pb_p_radps: Vector3<f64>,
    /// Upstream `omegaPrime_PB_P` \[rad/s^2\].
    pub omega_prime_pb_p_radps2: Vector3<f64>,
    /// The hub rate `omega_BN_B` \[rad/s\].
    pub omega_bn_b_radps: Vector3<f64>,
}

impl PrescribedFrameMotion {
    /// The motion in P components, with the products the coupling terms of
    /// every attached effector share.
    pub(crate) fn in_child_frame(&self) -> ChildFrameMotion {
        let r_pb_p = self.dcm_pb * self.r_pb_b_m;
        let r_prime_pb_p = self.dcm_pb * self.r_prime_pb_b_mps;
        let r_prime_prime_pb_p = self.dcm_pb * self.r_prime_prime_pb_b_mps2;
        let omega_bn_p = self.dcm_pb * self.omega_bn_b_radps;
        let omega_tilde_bn_p = tilde(omega_bn_p);
        let omega_pb_p = self.omega_pb_p_radps;
        ChildFrameMotion {
            r_pb_p,
            r_tilde_pb_p: tilde(r_pb_p),
            r_prime_pb_p,
            r_prime_prime_pb_p,
            omega_pb_p,
            omega_tilde_pb_p: tilde(omega_pb_p),
            omega_prime_pb_p: self.omega_prime_pb_p_radps2,
            omega_prime_tilde_pb_p: tilde(self.omega_prime_pb_p_radps2),
            omega_pn_p: omega_pb_p + omega_bn_p,
            frame_accel_p: r_prime_prime_pb_p
                + 2.0 * omega_tilde_bn_p * r_prime_pb_p
                + omega_tilde_bn_p * omega_tilde_bn_p * r_pb_p,
            frame_ang_accel_p: self.omega_prime_pb_p_radps2 + omega_tilde_bn_p * omega_pb_p,
        }
    }
}

/// [`PrescribedFrameMotion`] in P components.
pub(crate) struct ChildFrameMotion {
    pub r_pb_p: Vector3<f64>,
    pub r_tilde_pb_p: Matrix3<f64>,
    pub r_prime_pb_p: Vector3<f64>,
    pub r_prime_prime_pb_p: Vector3<f64>,
    pub omega_pb_p: Vector3<f64>,
    pub omega_tilde_pb_p: Matrix3<f64>,
    pub omega_prime_pb_p: Vector3<f64>,
    pub omega_prime_tilde_pb_p: Matrix3<f64>,
    /// `omega_PN_P`.
    pub omega_pn_p: Vector3<f64>,
    /// `r''_PB + 2 ω_BN × r'_PB + ω_BN × (ω_BN × r_PB)`: the part of P's
    /// inertial acceleration relative to B's that does not depend on the hub
    /// accelerations.
    pub frame_accel_p: Vector3<f64>,
    /// `ω'_PB + ω_BN × ω_PB`: the same for P's angular acceleration.
    pub frame_ang_accel_p: Vector3<f64>,
}

/// A state effector that can ride on a [`PrescribedMotionStateEffector`]
/// (upstream `StateEffector::addPrescribedMotionCouplingContributions`).
///
/// Mounted on the prescribed body, the effector sees the frame P as its hub
/// frame: its states, mass properties, contributions and outputs are all
/// relative to P, and the prescribed body transforms them to B.
pub trait PrescribedMotionAttachment: StateEffector {
    /// Adds to `back_sub`, in P components, the terms that couple the
    /// effector's equations to P's prescribed motion relative to B. Called
    /// right after [`StateEffector::update_contributions`] with the same
    /// `effector_state`, on contributions that hold only this effector's.
    fn add_prescribed_motion_coupling(
        &self,
        effector_state: &[f64],
        motion: &PrescribedFrameMotion,
        back_sub: &mut BackSubMatrices,
    );
}

/// User-settable parameters of [`PrescribedMotionStateEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct PrescribedMotionConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mass` \[kg\] (default 0; zero is allowed).
    pub mass_kg: f64,
    /// Upstream `IPntPc_P` \[kg m^2\]: inertia about the body's center of
    /// mass in P components (default identity).
    pub inertia_about_com_p_kg_m2: Matrix3<f64>,
    /// Upstream `r_PcP_P` \[m\]: center of mass relative to the P origin, in P
    /// components (default zero).
    pub center_of_mass_p_m: Vector3<f64>,
    /// Upstream `r_MB_B` \[m\]: mount frame origin relative to the body
    /// origin (default zero).
    pub mount_position_body_m: Vector3<f64>,
    /// Upstream `sigma_MB`: attitude of the mount frame relative to B
    /// (default zero).
    pub sigma_mb: Vector3<f64>,
    /// Upstream `r_PM_M` \[m\]: initial position of P relative to M (default
    /// zero).
    pub r_pm_m_m: Vector3<f64>,
    /// Upstream `rPrime_PM_M` \[m/s\] (default zero).
    pub r_prime_pm_m_mps: Vector3<f64>,
    /// Upstream `rPrimePrime_PM_M` \[m/s^2\] (default zero).
    pub r_prime_prime_pm_m_mps2: Vector3<f64>,
    /// Upstream `omega_PM_P` \[rad/s\] (default zero).
    pub omega_pm_p_radps: Vector3<f64>,
    /// Upstream `omegaPrime_PM_P` \[rad/s^2\] (default zero).
    pub omega_prime_pm_p_radps2: Vector3<f64>,
    /// Upstream `sigma_PM`: initial attitude of P relative to M (default
    /// zero).
    pub sigma_pm: Vector3<f64>,
}

impl PrescribedMotionConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mass_kg: 0.0,
            inertia_about_com_p_kg_m2: Matrix3::identity(),
            center_of_mass_p_m: Vector3::zeros(),
            mount_position_body_m: Vector3::zeros(),
            sigma_mb: Vector3::zeros(),
            r_pm_m_m: Vector3::zeros(),
            r_prime_pm_m_mps: Vector3::zeros(),
            r_prime_prime_pm_m_mps2: Vector3::zeros(),
            omega_pm_p_radps: Vector3::zeros(),
            omega_prime_pm_p_radps2: Vector3::zeros(),
            sigma_pm: Vector3::zeros(),
        }
    }

    /// Upstream `validateMass`: the mass finite and non-negative.
    pub fn validate(&self) -> Result<(), String> {
        if !self.mass_kg.is_finite() || self.mass_kg < 0.0 {
            return Err("mass_kg must be finite and non-negative".to_string());
        }
        Ok(())
    }
}

/// The prescribed translation and rate at the start of the current step
/// (upstream `rEpoch_PM_M`, `rPrimeEpoch_PM_M`, `omegaEpoch_PM_P` and the
/// latched accelerations).
#[derive(Clone, Copy, Debug)]
struct PrescribedEpoch {
    r_pm_m: Vector3<f64>,
    r_prime_pm_m: Vector3<f64>,
    r_prime_prime_pm_m: Vector3<f64>,
    omega_pm_p: Vector3<f64>,
    omega_prime_pm_p: Vector3<f64>,
}

impl PrescribedEpoch {
    fn from_config(config: &PrescribedMotionConfig) -> Self {
        Self {
            r_pm_m: config.r_pm_m_m,
            r_prime_pm_m: config.r_prime_pm_m_mps,
            r_prime_prime_pm_m: config.r_prime_prime_pm_m_mps2,
            omega_pm_p: config.omega_pm_p_radps,
            omega_prime_pm_p: config.omega_prime_pm_p_radps2,
        }
    }

    /// The epoch moved `tau_s` along with constant accelerations.
    fn advanced(&self, tau_s: f64) -> Self {
        Self {
            r_pm_m: self.r_pm_m
                + self.r_prime_pm_m * tau_s
                + 0.5 * self.r_prime_prime_pm_m * tau_s * tau_s,
            r_prime_pm_m: self.r_prime_pm_m + self.r_prime_prime_pm_m * tau_s,
            omega_pm_p: self.omega_pm_p + self.omega_prime_pm_p * tau_s,
            ..*self
        }
    }
}

/// A rigid body with prescribed motion relative to the hub (upstream
/// `PrescribedMotionStateEffector`).
pub struct PrescribedMotionStateEffector {
    pub config: PrescribedMotionConfig,
    /// Upstream `prescribedTranslationInMsg` (optional).
    pub prescribed_translation_in: Input<PrescribedTranslationMsg>,
    /// Upstream `prescribedRotationInMsg` (optional).
    pub prescribed_rotation_in: Input<PrescribedRotationMsg>,
    /// Upstream `prescribedTranslationOutMsg`.
    pub prescribed_translation_out: Output<PrescribedTranslationMsg>,
    /// Upstream `prescribedRotationOutMsg`.
    pub prescribed_rotation_out: Output<PrescribedRotationMsg>,
    /// Upstream `prescribedMotionConfigLogOutMsg`: inertial state of the
    /// body, with P as body frame and its center of mass as position.
    pub prescribed_motion_state_out: Output<SpacecraftStateMsg>,
    attached: Vec<Box<dyn PrescribedMotionAttachment>>,
    /// `None` until the first step starts: the configured values then.
    epoch: Option<PrescribedEpoch>,
    /// Commanded `σ_PM` minus the integrated value, during a step.
    sigma_shift: Vector3<f64>,
    /// Last loaded state.
    state: Vec<f64>,
    /// Hub rate of the last `update_contributions`, which upstream caches for
    /// `computeDerivatives`.
    hub_omega_radps: Cell<Vector3<f64>>,
    kinematics: KinematicsCache<PrescribedMotionConfig, PrescribedKinematics>,
}

impl fmt::Debug for PrescribedMotionStateEffector {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter
            .debug_struct("PrescribedMotionStateEffector")
            .field("config", &self.config)
            .field(
                "attached",
                &self
                    .attached
                    .iter()
                    .map(|effector| effector.name())
                    .collect::<Vec<_>>(),
            )
            .field("state", &self.state)
            .finish_non_exhaustive()
    }
}

impl PrescribedMotionStateEffector {
    pub fn new(config: PrescribedMotionConfig) -> Self {
        let mut state = config.sigma_pm.as_slice().to_vec();
        state.push(0.0);
        Self {
            config,
            prescribed_translation_in: Input::default(),
            prescribed_rotation_in: Input::default(),
            prescribed_translation_out: Output::default(),
            prescribed_rotation_out: Output::default(),
            prescribed_motion_state_out: Output::default(),
            attached: Vec::new(),
            epoch: None,
            sigma_shift: Vector3::zeros(),
            state,
            hub_omega_radps: Cell::new(Vector3::zeros()),
            kinematics: KinematicsCache::default(),
        }
    }

    /// Mounts `effector` on the prescribed body (upstream `addStateEffector`).
    /// Its states follow this effector's own in the state vector.
    pub fn add_state_effector<E>(&mut self, effector: E)
    where
        E: PrescribedMotionAttachment + 'static,
    {
        self.state
            .extend(std::iter::repeat_n(0.0, effector.state_len()));
        self.attached.push(Box::new(effector));
    }

    /// The effectors mounted on the body, in the order they were added.
    pub fn attached_state_effectors(&self) -> &[Box<dyn PrescribedMotionAttachment>] {
        &self.attached
    }

    /// Upstream `sigma_PM` of the last loaded state.
    pub fn sigma_pm(&self) -> Vector3<f64> {
        Vector3::new(self.state[0], self.state[1], self.state[2]) + self.sigma_shift
    }

    /// The prescribed translation and rotation of the last loaded state.
    pub fn prescribed_state(&self) -> (PrescribedTranslationMsg, PrescribedRotationMsg) {
        let k = self.kinematics(&self.state);
        (
            PrescribedTranslationMsg {
                r_pm_m_m: k.r_pm_m,
                r_prime_pm_m_mps: k.r_prime_pm_m,
                r_prime_prime_pm_m_mps2: k.r_prime_prime_pm_m,
            },
            PrescribedRotationMsg {
                omega_pm_p_radps: k.omega_pm_p,
                omega_prime_pm_p_radps2: k.omega_prime_pm_p,
                sigma_pm: k.sigma_pm,
            },
        )
    }

    /// The inertial state of the frame P (upstream `r_PN_N`, `v_PN_N`,
    /// `sigma_PN`, `omega_PN_P`), which the attached effectors take as their
    /// hub state, and of the body's center of mass (upstream
    /// `computePrescribedMotionInertialStates`), for the last loaded state.
    pub fn prescribed_inertial_states(
        &self,
        hub_state: &SpacecraftStateMsg,
    ) -> (SpacecraftStateMsg, SpacecraftStateMsg) {
        let k = self.kinematics(&self.state);
        let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
        let dcm_nb = dcm_bn.transpose();
        let omega_bn_b = hub_state.omega_radps;
        let sigma_pn = dcm_to_mrp(k.dcm_bp.transpose() * dcm_bn);
        let omega_pn_p = k.dcm_bp.transpose() * (k.omega_pb_b + omega_bn_b);
        let frame = SpacecraftStateMsg {
            position_m: hub_state.position_m + dcm_nb * k.r_pb_b,
            velocity_mps: hub_state.velocity_mps
                + dcm_nb * (k.r_prime_pm_b + omega_bn_b.cross(&k.r_pb_b)),
            sigma_bn: sigma_pn,
            omega_radps: omega_pn_p,
            ..Default::default()
        };
        let center_of_mass = SpacecraftStateMsg {
            position_m: hub_state.position_m + dcm_nb * k.r_pcb_b,
            velocity_mps: hub_state.velocity_mps
                + dcm_nb * (k.r_prime_pcb_b + omega_bn_b.cross(&k.r_pcb_b)),
            sigma_bn: sigma_pn,
            omega_radps: omega_pn_p,
            ..Default::default()
        };
        (frame, center_of_mass)
    }

    fn current_epoch(&self) -> PrescribedEpoch {
        self.epoch
            .unwrap_or_else(|| PrescribedEpoch::from_config(&self.config))
    }

    fn kinematics(&self, effector_state: &[f64]) -> PrescribedKinematics {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "prescribed motion state length mismatch"
        );
        let sigma_pm = Vector3::new(effector_state[0], effector_state[1], effector_state[2])
            + self.sigma_shift;
        let epoch = self.current_epoch().advanced(effector_state[3]);
        let key: Vec<f64> = [
            sigma_pm,
            epoch.r_pm_m,
            epoch.r_prime_pm_m,
            epoch.r_prime_prime_pm_m,
            epoch.omega_pm_p,
            epoch.omega_prime_pm_p,
        ]
        .iter()
        .flat_map(|vector| vector.iter().copied())
        .collect();
        self.kinematics.get_or_compute(&key, &self.config, || {
            PrescribedKinematics::new(&self.config, &epoch, sigma_pm)
        })
    }

    /// Each attached effector with its slice of `effector_state`.
    fn attached_states<'a>(
        &'a self,
        effector_state: &'a [f64],
    ) -> impl Iterator<Item = (&'a dyn PrescribedMotionAttachment, &'a [f64])> + 'a {
        let mut rest = &effector_state[OWN_STATE_LEN..];
        self.attached.iter().map(move |effector| {
            let (slice, tail) = rest.split_at(effector.state_len());
            rest = tail;
            (effector.as_ref(), slice)
        })
    }
}

impl StateEffector for PrescribedMotionStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        OWN_STATE_LEN
            + self
                .attached
                .iter()
                .map(|effector| effector.state_len())
                .sum::<usize>()
    }

    /// Validates the mass (upstream `registerStates`) and returns
    /// `[σ_PM, 0, attached initial states…]`.
    ///
    /// # Panics
    ///
    /// If [`PrescribedMotionConfig::validate`] or an attached effector's
    /// `initial_state` fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid PrescribedMotionConfig '{}': {message}",
                self.config.name
            );
        }
        let mut state = self.config.sigma_pm.as_slice().to_vec();
        state.push(0.0);
        for effector in &self.attached {
            state.extend(effector.initial_state());
        }
        state
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "prescribed motion state length mismatch"
        );
        let mut rest = &state[OWN_STATE_LEN..];
        for effector in &mut self.attached {
            let (slice, tail) = rest.split_at(effector.state_len());
            effector.load_state(slice);
            rest = tail;
        }
        self.state.clear();
        self.state.extend_from_slice(state);
    }

    /// Upstream `UpdateState`: latches the commanded prescribed state as the
    /// start of this step.
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        let mut epoch = self.current_epoch();
        if self.prescribed_translation_in.is_connected() {
            let translation = self.prescribed_translation_in.read();
            epoch.r_pm_m = translation.r_pm_m_m;
            epoch.r_prime_pm_m = translation.r_prime_pm_m_mps;
            epoch.r_prime_prime_pm_m = translation.r_prime_prime_pm_m_mps2;
        }
        if self.prescribed_rotation_in.is_connected() {
            let rotation = self.prescribed_rotation_in.read();
            epoch.omega_pm_p = rotation.omega_pm_p_radps;
            epoch.omega_prime_pm_p = rotation.omega_prime_pm_p_radps2;
            self.sigma_shift =
                rotation.sigma_pm - Vector3::new(self.state[0], self.state[1], self.state[2]);
        }
        self.epoch = Some(epoch);
        for effector in &mut self.attached {
            effector.pre_integration(current_sim_nanos, dt_seconds);
        }
    }

    /// Hands the stage time to the attached effectors, as the spacecraft does
    /// to its own.
    fn begin_stage(&mut self, stage_nanos: u64) {
        for effector in &mut self.attached {
            effector.begin_stage(stage_nanos);
        }
    }

    /// Upstream `updateContributions`, including the attached effectors'.
    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let k = self.kinematics(effector_state);
        let omega_bn_b = hub_state.omega_radps;
        self.hub_omega_radps.set(omega_bn_b);
        let dcm_pb = k.dcm_bp.transpose();
        if !self.attached.is_empty() {
            let motion = k.frame_motion(omega_bn_b);
            let omega_pn_p = k.omega_pm_p + dcm_pb * omega_bn_b;
            let gravity_p = dcm_pb * gravity_body_mps2;
            // The attached effectors' hub is the platform frame P.
            let dcm_bn = mrp_to_dcm(hub_state.sigma_bn);
            let dcm_nb = dcm_bn.transpose();
            let platform_state = SpacecraftStateMsg {
                position_m: hub_state.position_m + dcm_nb * k.r_pb_b,
                velocity_mps: hub_state.velocity_mps
                    + dcm_nb * (k.r_prime_pm_b + omega_bn_b.cross(&k.r_pb_b)),
                sigma_bn: dcm_to_mrp(dcm_pb * dcm_bn),
                omega_radps: omega_pn_p,
                ..Default::default()
            };
            for (effector, slice) in self.attached_states(effector_state) {
                let mut terms = BackSubMatrices::default();
                effector.update_contributions(slice, &platform_state, gravity_p, &mut terms);
                effector.add_prescribed_motion_coupling(slice, &motion, &mut terms);
                add_back_sub(
                    back_sub,
                    &BackSubMatrices {
                        matrix_a: k.dcm_bp * terms.matrix_a * dcm_pb,
                        matrix_b: k.dcm_bp * terms.matrix_b * dcm_pb,
                        matrix_c: k.dcm_bp * terms.matrix_c * dcm_pb,
                        matrix_d: k.dcm_bp * terms.matrix_d * dcm_pb,
                        vec_trans: k.dcm_bp * terms.vec_trans,
                        vec_rot: k.dcm_bp * terms.vec_rot,
                    },
                );
            }
        }

        let m = self.config.mass_kg;
        let r_prime_prime_pcb_b = k.omega_prime_pb_b.cross(&k.r_pcp_b)
            + k.omega_pb_b.cross(&k.omega_pb_b.cross(&k.r_pcp_b))
            + k.r_prime_prime_pm_b;
        back_sub.vec_trans += -m * r_prime_prime_pcb_b;
        let omega_tilde_pb_b = tilde(k.omega_pb_b);
        let i_prime_pnt_pc_b = omega_tilde_pb_b * k.i_pnt_pc_b - k.i_pnt_pc_b * omega_tilde_pb_b;
        back_sub.vec_rot += -(m * k.r_pcb_b.cross(&r_prime_prime_pcb_b))
            - i_prime_pnt_pc_b * k.omega_pb_b
            - omega_bn_b.cross(&(k.i_pnt_pc_b * k.omega_pb_b))
            - k.i_pnt_pc_b * k.omega_prime_pb_b
            - m * omega_bn_b.cross(&k.r_pcb_b.cross(&k.r_prime_pcb_b));
    }

    /// Upstream `computeDerivatives`: `σ̇_PM = ¼ B(σ_PM) ω_PM_P`, the step
    /// clock, and the attached effectors' derivatives at P's acceleration.
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        let k = self.kinematics(effector_state);
        let sigma_dot = 0.25 * b_mat_mrp(k.sigma_pm) * k.omega_pm_p;
        let mut derivative = sigma_dot.as_slice().to_vec();
        derivative.push(1.0);
        if !self.attached.is_empty() {
            let omega = self.hub_omega_radps.get();
            let dcm_pb = k.dcm_bp.transpose();
            let omega_pm_b = k.omega_pb_b;
            let omega_dot_pn_p =
                dcm_pb * (k.omega_prime_pb_b + omega.cross(&omega_pm_b) + body_omega_dot_radps2);
            let r_ddot_pb_b = k.r_prime_prime_pm_b
                + 2.0 * omega.cross(&k.r_prime_pm_b)
                + body_omega_dot_radps2.cross(&k.r_pb_b)
                + omega.cross(&omega.cross(&k.r_pb_b));
            let r_ddot_pn_p = dcm_pb * (r_ddot_pb_b + body_trans_accel_mps2);
            for (effector, slice) in self.attached_states(effector_state) {
                derivative.extend(effector.compute_derivatives(slice, r_ddot_pn_p, omega_dot_pn_p));
            }
        }
        derivative
    }

    /// Upstream `updateEffectorMassProps`, including the attached effectors'.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        let k = self.kinematics(effector_state);
        let m = self.config.mass_kg;
        let r_tilde = tilde(k.r_pcb_b);
        let r_prime_tilde = tilde(k.r_prime_pcb_b);
        let omega_tilde_pb_b = tilde(k.omega_pb_b);
        let mut props = StateEffectorMassProps {
            mass_kg: m,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m: k.r_pcb_b,
            center_of_mass_prime_body_mps: k.r_prime_pcb_b,
            inertia_about_point_b_body_kg_m2: k.i_pnt_pc_b - m * r_tilde * r_tilde,
            inertia_about_point_b_body_prime_kg_m2ps: omega_tilde_pb_b * k.i_pnt_pc_b
                - k.i_pnt_pc_b * omega_tilde_pb_b
                + m * (r_prime_tilde * r_tilde.transpose() + r_tilde * r_prime_tilde.transpose()),
        };
        if self.attached.is_empty() {
            return props;
        }

        let mut first_moment = m * k.r_pcb_b;
        let mut first_moment_prime = m * k.r_prime_pcb_b;
        for (effector, slice) in self.attached_states(effector_state) {
            let attached = effector.mass_properties(slice);
            assert!(
                attached.mass_dot_kgps == 0.0,
                "PrescribedMotionStateEffector '{}' does not support the variable-mass \
                 effector '{}'",
                self.config.name,
                effector.name()
            );
            let m_e = attached.mass_kg;
            props.mass_kg += m_e;

            let r_ecp_b = k.dcm_bp * attached.center_of_mass_body_m;
            let r_ecb_b = r_ecp_b + k.r_pb_b;
            first_moment += m_e * r_ecb_b;
            let r_prime_ecp_b =
                k.dcm_bp * attached.center_of_mass_prime_body_mps + k.omega_pb_b.cross(&r_ecp_b);
            let r_prime_ecb_b = r_prime_ecp_b + k.r_prime_pm_b;
            first_moment_prime += m_e * r_prime_ecb_b;

            let i_pnt_p_b =
                k.dcm_bp * attached.inertia_about_point_b_body_kg_m2 * k.dcm_bp.transpose();
            let r_tilde_ecb = tilde(r_ecb_b);
            let r_tilde_ecp = tilde(r_ecp_b);
            props.inertia_about_point_b_body_kg_m2 +=
                i_pnt_p_b + m_e * (r_tilde_ecp * r_tilde_ecp - r_tilde_ecb * r_tilde_ecb);

            let r_prime_tilde_ecb = tilde(r_prime_ecb_b);
            let r_prime_tilde_ecp = tilde(r_prime_ecp_b);
            let i_p_prime_pnt_p_b =
                k.dcm_bp * attached.inertia_about_point_b_body_prime_kg_m2ps * k.dcm_bp.transpose();
            let i_prime_pnt_p_b = i_p_prime_pnt_p_b
                + omega_tilde_pb_b * i_pnt_p_b
                + i_pnt_p_b * omega_tilde_pb_b.transpose();
            props.inertia_about_point_b_body_prime_kg_m2ps += i_prime_pnt_p_b
                - m_e * r_prime_tilde_ecp * r_tilde_ecp.transpose()
                - m_e * r_tilde_ecp * r_prime_tilde_ecp.transpose()
                + m_e * r_prime_tilde_ecb * r_tilde_ecb.transpose()
                + m_e * r_tilde_ecb * r_prime_tilde_ecb.transpose();
        }
        if props.mass_kg > 0.0 {
            props.center_of_mass_body_m = first_moment / props.mass_kg;
            props.center_of_mass_prime_body_mps = first_moment_prime / props.mass_kg;
        }
        props
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B),
    /// including the attached effectors'.
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.energy_and_momentum(effector_state, body_omega_radps).1
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B,
    /// including the attached effectors'.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.energy_and_momentum(effector_state, body_omega_radps).0
    }

    fn write_outputs(&mut self, current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        let (translation, rotation) = self.prescribed_state();
        self.prescribed_translation_out.write(translation);
        self.prescribed_rotation_out.write(rotation);
        let (frame, center_of_mass) = self.prescribed_inertial_states(hub_state);
        self.prescribed_motion_state_out.write(center_of_mass);
        for effector in &mut self.attached {
            effector.write_outputs(current_sim_nanos, &frame);
        }
    }

    /// Folds the step's `σ_PM` reset into the state, moves the prescribed
    /// epoch to the end of the step, restarts the step clock, and lets the
    /// attached effectors rewrite their states.
    fn modify_states(&mut self, current_sim_nanos: u64, effector_state: &mut [f64]) {
        self.epoch = Some(self.current_epoch().advanced(effector_state[3]));
        for (value, shift) in effector_state.iter_mut().zip(self.sigma_shift.iter()) {
            *value += shift;
        }
        self.sigma_shift = Vector3::zeros();
        effector_state[3] = 0.0;
        let mut rest = &mut effector_state[OWN_STATE_LEN..];
        for effector in &mut self.attached {
            let (slice, tail) = rest.split_at_mut(effector.state_len());
            effector.modify_states(current_sim_nanos, slice);
            rest = tail;
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl PrescribedMotionStateEffector {
    /// Rotational energy and angular momentum about B (upstream
    /// `updateEnergyMomContributions`).
    fn energy_and_momentum(
        &self,
        effector_state: &[f64],
        omega_bn_b: Vector3<f64>,
    ) -> (f64, Vector3<f64>) {
        let k = self.kinematics(effector_state);
        let omega_pn_b = k.omega_pb_b + omega_bn_b;
        let omega_pn_p = k.dcm_bp.transpose() * omega_pn_b;
        let mut energy_j = 0.0;
        let mut momentum = Vector3::zeros();
        for (effector, slice) in self.attached_states(effector_state) {
            let attached = effector.mass_properties(slice);
            let m_e = attached.mass_kg;
            let r_ecp_b = k.dcm_bp * attached.center_of_mass_body_m;
            let r_dot_pb_b = k.r_prime_pm_b + omega_bn_b.cross(&k.r_pb_b);
            let r_dot_ecp_b =
                k.dcm_bp * attached.center_of_mass_prime_body_mps + omega_pn_b.cross(&r_ecp_b);
            momentum += k.dcm_bp * effector.rotational_angular_momentum_body(slice, omega_pn_p)
                + m_e * r_ecp_b.cross(&r_dot_pb_b)
                + m_e * k.r_pb_b.cross(&r_dot_ecp_b)
                + m_e * k.r_pb_b.cross(&r_dot_pb_b);
            energy_j += effector.rotational_energy_j(slice, omega_pn_p)
                + m_e * r_dot_ecp_b.dot(&r_dot_pb_b)
                + 0.5 * m_e * r_dot_pb_b.norm_squared();
        }
        let m = self.config.mass_kg;
        let r_dot_pcb_b = k.r_prime_pcb_b + omega_bn_b.cross(&k.r_pcb_b);
        momentum += k.i_pnt_pc_b * omega_pn_b + m * k.r_pcb_b.cross(&r_dot_pcb_b);
        energy_j += 0.5 * omega_pn_b.dot(&(k.i_pnt_pc_b * omega_pn_b))
            + 0.5 * m * r_dot_pcb_b.norm_squared();
        (energy_j, momentum)
    }
}

/// The prescribed body's geometry at one state (upstream
/// `updateEffectorMassProps`).
#[derive(Clone, Debug)]
struct PrescribedKinematics {
    r_pm_m: Vector3<f64>,
    r_prime_pm_m: Vector3<f64>,
    r_prime_prime_pm_m: Vector3<f64>,
    omega_pm_p: Vector3<f64>,
    omega_prime_pm_p: Vector3<f64>,
    sigma_pm: Vector3<f64>,
    /// Upstream `dcm_BP`.
    dcm_bp: Matrix3<f64>,
    /// Upstream `omega_PB_B` (= `omega_PM_B`).
    omega_pb_b: Vector3<f64>,
    /// Upstream `omegaPrime_PB_B`.
    omega_prime_pb_b: Vector3<f64>,
    /// Upstream `rPrime_PM_B` (= `rPrime_PB_B`).
    r_prime_pm_b: Vector3<f64>,
    /// Upstream `rPrimePrime_PM_B` (= `rPrimePrime_PB_B`).
    r_prime_prime_pm_b: Vector3<f64>,
    /// Upstream `r_PB_B`.
    r_pb_b: Vector3<f64>,
    /// Upstream `r_PcP_B`.
    r_pcp_b: Vector3<f64>,
    /// Upstream `r_PcB_B`.
    r_pcb_b: Vector3<f64>,
    /// Upstream `rPrime_PcB_B`.
    r_prime_pcb_b: Vector3<f64>,
    /// Upstream `IPntPc_B`.
    i_pnt_pc_b: Matrix3<f64>,
}

impl PrescribedKinematics {
    fn new(
        config: &PrescribedMotionConfig,
        epoch: &PrescribedEpoch,
        sigma_pm: Vector3<f64>,
    ) -> Self {
        let dcm_bm = mrp_to_dcm(config.sigma_mb).transpose();
        let dcm_pm = mrp_to_dcm(sigma_pm);
        let dcm_bp = dcm_bm * dcm_pm.transpose();
        let omega_pb_b = dcm_bp * epoch.omega_pm_p;
        let r_pb_b = dcm_bm * epoch.r_pm_m + config.mount_position_body_m;
        let r_prime_pm_b = dcm_bm * epoch.r_prime_pm_m;
        let r_pcp_b = dcm_bp * config.center_of_mass_p_m;
        Self {
            r_pm_m: epoch.r_pm_m,
            r_prime_pm_m: epoch.r_prime_pm_m,
            r_prime_prime_pm_m: epoch.r_prime_prime_pm_m,
            omega_pm_p: epoch.omega_pm_p,
            omega_prime_pm_p: epoch.omega_prime_pm_p,
            sigma_pm,
            dcm_bp,
            omega_pb_b,
            omega_prime_pb_b: dcm_bp * epoch.omega_prime_pm_p,
            r_prime_pm_b,
            r_prime_prime_pm_b: dcm_bm * epoch.r_prime_prime_pm_m,
            r_pb_b,
            r_pcp_b,
            r_pcb_b: r_pcp_b + r_pb_b,
            r_prime_pcb_b: omega_pb_b.cross(&r_pcp_b) + r_prime_pm_b,
            i_pnt_pc_b: dcm_bp * config.inertia_about_com_p_kg_m2 * dcm_bp.transpose(),
        }
    }

    fn frame_motion(&self, omega_bn_b: Vector3<f64>) -> PrescribedFrameMotion {
        PrescribedFrameMotion {
            r_pb_b_m: self.r_pb_b,
            r_prime_pb_b_mps: self.r_prime_pm_b,
            r_prime_prime_pb_b_mps2: self.r_prime_prime_pm_b,
            dcm_pb: self.dcm_bp.transpose(),
            omega_pb_p_radps: self.omega_pm_p,
            omega_prime_pb_p_radps2: self.omega_prime_pm_p,
            omega_bn_b_radps: omega_bn_b,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_PrescribedMotionStateEffector.py` and
    //! `test_PrescribedMotionStateEffectorBranching.py`, and an end-to-end
    //! test with FSW [`PrescribedRot2Dof`] driving the body.
    //!
    //! Upstream drives the body with its `prescribedRotation1DOF` and
    //! `prescribedLinearTranslation` profilers, which are not ported. They
    //! smooth their acceleration switches, so that latching the profile once
    //! per step, as the effector does, leaves only small jumps in the
    //! prescribed rates. The ports use [`RestToRest`] profiles instead:
    //! bang-bang, from the same initial to the same reference values within
    //! the same maximum accelerations, with the switch and the end on step
    //! boundaries so that the latched acceleration is exact for every step.
    //!
    //! Upstream's `test_prescribed_motion_state_effector` runs 36
    //! parametrizations of 30 s at a 1 ms step; eight of them, which cover
    //! every value of every parameter, are ported
    //! (`prescribed_motion_*`), to keep the suite's run time in check.

    use super::{PrescribedMotionConfig, PrescribedMotionStateEffector};
    use crate::dynamics::linear_translation_one_dof_state_effector::{
        LinearTranslationOneDofConfig, LinearTranslationOneDofStateEffector,
    };
    use crate::dynamics::spinning_body_one_dof_state_effector::{
        SpinningBodyOneDofConfig, SpinningBodyOneDofStateEffector,
    };
    use crate::dynamics::spinning_body_two_dof_state_effector::{
        SpinningBodyTwoDofConfig, SpinningBodyTwoDofStateEffector,
    };
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_relative_equal, spacecraft_with_effectors,
    };
    use crate::fsw_algorithms::effector_interfaces::prescribed_rot_2dof::{
        PrescribedRot2Dof, PrescribedRot2DofConfig,
    };
    use crate::messages::{
        HingedRigidBodyMsg, Output, PrescribedRotationMsg, PrescribedTranslationMsg,
        SpacecraftDiagnosticsMsg,
    };
    use crate::simulation::Simulation;
    use crate::spacecraft::{BackSubMatrices, StateEffector};
    use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, prv_to_mrp};
    use crate::{Module, SimulationContext};
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    const D2R: f64 = std::f64::consts::PI / 180.0;
    const STEP_NANOS: u64 = 1_000_000;
    /// Upstream's maximum angular acceleration of the profiles \[rad/s^2\].
    const ANG_ACCEL_MAX_RADPS2: f64 = 0.5 * D2R;
    /// Upstream's maximum translational acceleration of the profiles \[m/s^2\].
    const TRANS_ACCEL_MAX_MPS2: f64 = 0.005;

    /// A rest-to-rest bang-bang profile from `initial` to `reference`, at most
    /// as fast as `accel_max` allows, accelerating for the first half and
    /// decelerating for the second, with both halves a whole number of
    /// integration steps long.
    #[derive(Clone, Copy, Debug)]
    struct RestToRest {
        initial: f64,
        reference: f64,
        accel: f64,
        half_nanos: u64,
    }

    impl RestToRest {
        fn new(initial: f64, reference: f64, accel_max: f64) -> Self {
            let distance = reference - initial;
            if distance == 0.0 {
                return Self {
                    initial,
                    reference,
                    accel: 0.0,
                    half_nanos: 0,
                };
            }
            let steps = ((distance.abs() / accel_max).sqrt() * 1e9 / STEP_NANOS as f64).ceil();
            let half_nanos = steps as u64 * STEP_NANOS;
            let half_s = half_nanos as f64 * 1e-9;
            Self {
                initial,
                reference,
                accel: distance / (half_s * half_s),
                half_nanos,
            }
        }

        /// Value, rate and acceleration at `nanos`; at a switch, the
        /// acceleration of the interval that starts there.
        fn at(&self, nanos: u64) -> (f64, f64, f64) {
            if nanos >= 2 * self.half_nanos {
                (self.reference, 0.0, 0.0)
            } else if nanos < self.half_nanos {
                let t_s = nanos as f64 * 1e-9;
                (
                    self.initial + 0.5 * self.accel * t_s * t_s,
                    self.accel * t_s,
                    self.accel,
                )
            } else {
                let remaining_s = (2 * self.half_nanos - nanos) as f64 * 1e-9;
                (
                    self.reference - 0.5 * self.accel * remaining_s * remaining_s,
                    self.accel * remaining_s,
                    -self.accel,
                )
            }
        }
    }

    /// Publishes a rotation about and a translation along the mount frame's
    /// x axis, in place of upstream's `prescribedRotation1DOF` and
    /// `prescribedLinearTranslation`.
    struct RestToRestProfiler {
        rotation: RestToRest,
        translation: RestToRest,
        rotation_out: Output<PrescribedRotationMsg>,
        translation_out: Output<PrescribedTranslationMsg>,
    }

    impl RestToRestProfiler {
        fn publish(&self, nanos: u64) {
            let axis = Vector3::x();
            let (theta, theta_dot, theta_ddot) = self.rotation.at(nanos);
            self.rotation_out.write(PrescribedRotationMsg {
                omega_pm_p_radps: theta_dot * axis,
                omega_prime_pm_p_radps2: theta_ddot * axis,
                sigma_pm: prv_to_mrp(theta * axis),
            });
            let (rho, rho_dot, rho_ddot) = self.translation.at(nanos);
            self.translation_out.write(PrescribedTranslationMsg {
                r_pm_m_m: rho * axis,
                r_prime_pm_m_mps: rho_dot * axis,
                r_prime_prime_pm_m_mps2: rho_ddot * axis,
            });
        }
    }

    impl Module for RestToRestProfiler {
        fn init(&mut self) {
            self.rotation_out.write(PrescribedRotationMsg::default());
            self.translation_out
                .write(PrescribedTranslationMsg::default());
        }

        fn update(&mut self, context: &SimulationContext) {
            self.publish(context.current_sim_nanos);
        }
    }

    /// A profiler driving `platform` from (`theta_init`, `position_init`) to
    /// (`theta_ref`, `position_ref`).
    fn rest_to_rest_profiler(
        platform: &mut PrescribedMotionStateEffector,
        theta_init_rad: f64,
        theta_ref_rad: f64,
        position_init_m: f64,
        position_ref_m: f64,
    ) -> RestToRestProfiler {
        let profiler = RestToRestProfiler {
            rotation: RestToRest::new(theta_init_rad, theta_ref_rad, ANG_ACCEL_MAX_RADPS2),
            translation: RestToRest::new(position_init_m, position_ref_m, TRANS_ACCEL_MAX_MPS2),
            rotation_out: Output::default(),
            translation_out: Output::default(),
        };
        profiler
            .rotation_out
            .connect_to(&mut platform.prescribed_rotation_in);
        profiler
            .translation_out
            .connect_to(&mut platform.prescribed_translation_in);
        profiler
    }

    /// Upstream `create_spacecraft_hub`: an 800 kg 1 m cube, slowly tumbling
    /// in the "Gravity" orbit.
    fn cube_hub() -> HubSetup {
        let inertia = 800.0 / 12.0 * 2.0;
        let mut hub = HubSetup::hinged_body_hub().in_gravity_case_orbit();
        hub.mass_kg = 800.0;
        hub.inertia_kg_m2 = Matrix3::from_diagonal_element(inertia);
        hub.omega_bn_b_radps = Vector3::new(0.01, -0.01, 0.01);
        hub
    }

    /// Upstream `create_prescribed_body`: a 100 kg 1 m cube mounted 0.5 m out
    /// along the hub's x axis.
    fn cube_platform(r_pm_m_m: Vector3<f64>, sigma_pm: Vector3<f64>) -> PrescribedMotionConfig {
        let mut config = PrescribedMotionConfig::new("prescribedMotion");
        config.mass_kg = 100.0;
        config.inertia_about_com_p_kg_m2 = Matrix3::from_diagonal_element(100.0 / 12.0 * 2.0);
        config.mount_position_body_m = Vector3::new(0.5, 0.0, 0.0);
        config.center_of_mass_p_m = Vector3::new(0.5, 0.0, 0.0);
        config.r_pm_m_m = r_pm_m_m;
        config.sigma_pm = sigma_pm;
        config
    }

    /// Runs the spacecraft carrying `platform` for `steps` steps, with the
    /// `profilers` scheduled after it, and returns the spacecraft diagnostics
    /// at t = 0 and after every step.
    fn run(
        platform: PrescribedMotionStateEffector,
        profilers: &mut [&mut dyn Module],
        steps: usize,
    ) -> Vec<SpacecraftDiagnosticsMsg> {
        let mut spacecraft =
            spacecraft_with_effectors(&cube_hub(), STEP_NANOS, vec![Box::new(platform)]);
        add_point_mass_earth(&mut spacecraft);
        let diagnostics = spacecraft.diagnostics_out.clone();
        let mut log = Vec::with_capacity(steps + 1);
        let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 1);
        for (index, profiler) in profilers.iter_mut().enumerate() {
            sim.add_module(format!("profiler{index}"), *profiler, STEP_NANOS, 0);
        }
        sim.run_for(0);
        log.push(diagnostics.read());
        for _ in 0..steps {
            sim.run_for(STEP_NANOS);
            log.push(diagnostics.read());
        }
        log
    }

    /// Upstream `unit_test_check_verification`: orbital angular momentum,
    /// orbital energy and rotational angular momentum at `initial` and
    /// `final_sample` agree to a relative 1e-6.
    fn assert_conserved_between(
        initial: &SpacecraftDiagnosticsMsg,
        final_sample: &SpacecraftDiagnosticsMsg,
    ) {
        let accuracy = 1e-6;
        assert_relative_equal(
            final_sample
                .orbital_angular_momentum_inertial_kg_m2ps
                .as_slice(),
            initial.orbital_angular_momentum_inertial_kg_m2ps.as_slice(),
            accuracy,
            "orbital angular momentum",
        );
        assert_relative_equal(
            &[final_sample.orbital_energy_j],
            &[initial.orbital_energy_j],
            accuracy,
            "orbital energy",
        );
        assert_relative_equal(
            final_sample
                .rotational_angular_momentum_inertial_kg_m2ps
                .as_slice(),
            initial
                .rotational_angular_momentum_inertial_kg_m2ps
                .as_slice(),
            accuracy,
            "rotational angular momentum",
        );
    }

    /// Upstream `test_prescribed_motion_state_effector`: the body rotates and
    /// translates relative to the hub, 30 s at a 1 ms step. As upstream, the
    /// configured `r_PM_M` is zero whatever the profile's initial position,
    /// so the check starts at sample 3, after the first message has moved the
    /// body.
    fn prescribed_motion_test(
        theta_init_rad: f64,
        theta_ref_rad: f64,
        position_init_m: f64,
        position_ref_m: f64,
    ) {
        let sigma_p0m = prv_to_mrp(theta_init_rad * Vector3::x());
        let mut platform =
            PrescribedMotionStateEffector::new(cube_platform(Vector3::zeros(), sigma_p0m));
        let mut profiler = rest_to_rest_profiler(
            &mut platform,
            theta_init_rad,
            theta_ref_rad,
            position_init_m,
            position_ref_m,
        );
        let translation_out = platform.prescribed_translation_out.clone();
        let rotation_out = platform.prescribed_rotation_out.clone();

        let log = run(platform, &mut [&mut profiler], 30_000);

        assert_conserved_between(&log[3], &log[log.len() - 1]);
        // Both maneuvers are complete: the body rests at the reference.
        let translation = translation_out.read();
        assert!((translation.r_pm_m_m - position_ref_m * Vector3::x()).norm() < 1e-12);
        assert_eq!(translation.r_prime_pm_m_mps, Vector3::zeros());
        let rotation = rotation_out.read();
        assert!((rotation.sigma_pm - prv_to_mrp(theta_ref_rad * Vector3::x())).norm() < 1e-12);
        assert_eq!(rotation.omega_pm_p_radps, Vector3::zeros());
    }

    /// `(theta_init, theta_ref)` in degrees and `(pos_init, pos_ref)` in
    /// meters, from upstream's parametrization grid.
    macro_rules! prescribed_motion_tests {
        ($($name:ident: $theta_init:expr, $theta_ref:expr, $position_init:expr, $position_ref:expr;)+) => {
            $(
                #[test]
                fn $name() {
                    prescribed_motion_test(
                        $theta_init * D2R,
                        $theta_ref * D2R,
                        $position_init,
                        $position_ref,
                    );
                }
            )+
        };
    }

    prescribed_motion_tests! {
        prescribed_motion_at_rest: 0.0, 0.0, 0.0, 0.0;
        prescribed_motion_translation_only: 0.0, 0.0, 0.1, 0.05;
        prescribed_motion_rotation_only: 0.0, 5.0, 0.0, 0.0;
        prescribed_motion_both_backward: 10.0, 5.0, 0.1, 0.05;
        prescribed_motion_largest: -5.0, 5.0, -0.1, 0.05;
        prescribed_motion_rotate_back_translate_out: 10.0, 0.0, -0.1, 0.0;
        prescribed_motion_rotate_out_translate_out: -5.0, 0.0, 0.0, 0.05;
        prescribed_motion_rotate_out_translate_back: 0.0, 5.0, 0.1, 0.0;
    }

    /// Upstream's branching tests: `child` rides on the platform while it
    /// rotates 10° and translates 0.1 m, 15 s at a 1 ms step.
    fn branching_test<E>(child: E)
    where
        E: super::PrescribedMotionAttachment + 'static,
    {
        let mut platform =
            PrescribedMotionStateEffector::new(cube_platform(Vector3::zeros(), Vector3::zeros()));
        platform.add_state_effector(child);
        let mut profiler = rest_to_rest_profiler(&mut platform, 0.0, 10.0 * D2R, 0.0, 0.1);
        let log = run(platform, &mut [&mut profiler], 15_000);
        assert_conserved_between(&log[0], &log[log.len() - 1]);
    }

    fn branch_spinner() -> SpinningBodyOneDofConfig {
        let mut config = SpinningBodyOneDofConfig::new("spinningBodyOneDOF");
        config.mass_kg = 50.0;
        config.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0));
        config.spinner_origin_body_m = Vector3::new(1.5, 0.0, 0.0);
        config.spin_axis_spinner = Vector3::x();
        config.theta_init_rad = 10.0 * D2R;
        config.spring_constant_nm_per_rad = 100.0;
        config
    }

    fn branch_pair() -> SpinningBodyTwoDofConfig {
        let mut config = SpinningBodyTwoDofConfig::new("spinningBodyTwoDOF");
        config.body1.mass_kg = 100.0;
        config.body2.mass_kg = 50.0;
        config.body1.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(100.0, 50.0, 50.0));
        config.body2.inertia_about_com_spinner_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0));
        config.spinner1_origin_body_m = Vector3::new(1.5, 0.0, 0.0);
        config.spinner2_origin_spinner1_m = Vector3::new(1.0, 0.0, 0.0);
        config.body1.spin_axis_spinner = Vector3::x();
        config.body2.spin_axis_spinner = Vector3::x();
        config.body2.theta_init_rad = 5.0 * D2R;
        config.body1.theta_dot_init_radps = 2.0 * D2R;
        config.body2.theta_dot_init_radps = -D2R;
        config.body1.spring_constant_nm_per_rad = 1000.0;
        config.body2.spring_constant_nm_per_rad = 500.0;
        config
    }

    fn branch_slider() -> LinearTranslationOneDofConfig {
        let mut config = LinearTranslationOneDofConfig::new("translatingBody");
        config.mass_kg = 20.0;
        config.spring_constant_n_per_m = 100.0;
        config.rho_init_m = 1.0;
        config.axis_body = Vector3::x();
        config.origin0_body_m = Vector3::new(1.0, 0.0, 0.0);
        config.inertia_about_com_translating_kg_m2 =
            Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, 60.0));
        config
    }

    /// Upstream `test_prescribed_branching_spinning_body_one_dof`.
    #[test]
    fn prescribed_branching_spinning_body_one_dof() {
        branching_test(SpinningBodyOneDofStateEffector::new(branch_spinner()));
    }

    /// Upstream `test_prescribed_branching_spinning_body_two_dof`.
    #[test]
    fn prescribed_branching_spinning_body_two_dof() {
        branching_test(SpinningBodyTwoDofStateEffector::new(branch_pair()));
    }

    /// Upstream `test_prescribed_branching_linear_translation_one_dof`.
    #[test]
    fn prescribed_branching_linear_translation_one_dof() {
        branching_test(LinearTranslationOneDofStateEffector::new(branch_slider()));
    }

    /// Upstream's one-DOF branching test puts the spinner's center of mass on
    /// its axis, where several coupling terms vanish; here it is off the
    /// axis, which exercises them (and the corrected sign of the
    /// `c r̃_PB r̃_ScS ŝ` term).
    #[test]
    fn prescribed_branching_spinning_body_one_dof_off_axis() {
        let mut config = branch_spinner();
        config.center_of_mass_spinner_m = Vector3::new(0.2, -0.3, 0.4);
        config.body_to_spinner0_dcm = mrp_to_dcm(Vector3::new(0.1, 0.2, -0.3));
        config.spin_axis_spinner = Vector3::new(0.0, 1.0, 1.0);
        branching_test(SpinningBodyOneDofStateEffector::new(config));
    }

    /// End to end: FSW `prescribedRot2DOF` drives the body 10° about x
    /// (`phi_ddot_max` = 0.5°/s²) while it translates 0.1 m. The body ends at
    /// the FSW reference attitude, at rest.
    ///
    /// The FSW profile switches its acceleration between dynamics steps, and
    /// the body latches it once per step, so the body's rate jumps by at most
    /// `2 φ̈ Δt` at the switch and `φ̈ Δt` at the end of the maneuver when the
    /// next message resets it; the hub's angular momentum moves by the body's
    /// inertia times those jumps. It is conserved exactly (to 1e-10) up to
    /// the switch and to within `3 I_xx φ̈ Δt` up to the end. After the end,
    /// `PrescribedRot2Dof` restarts a zero-length maneuver at every update,
    /// as upstream does, publishing `φ̈_max` with a zero rate; the check stops
    /// before the first of those messages is used.
    #[test]
    fn prescribed_rot_2dof_drives_the_prescribed_body() {
        let theta_ref_rad = 10.0 * D2R;
        let mut platform =
            PrescribedMotionStateEffector::new(cube_platform(Vector3::zeros(), Vector3::zeros()));
        // Only the translation of this profiler drives the platform: the FSW
        // output replaces its rotation below.
        let mut translation = rest_to_rest_profiler(&mut platform, 0.0, 0.0, 0.0, 0.1);
        let mut rotation = PrescribedRot2Dof::new(PrescribedRot2DofConfig {
            phi_ddot_max_radps2: ANG_ACCEL_MAX_RADPS2,
            rot_axis1_m: Vector3::x(),
            rot_axis2_p1: Vector3::z(),
            ..Default::default()
        });
        let references = [theta_ref_rad, 0.0].map(|theta_rad| {
            Output::new(HingedRigidBodyMsg {
                theta_rad,
                theta_dot_radps: 0.0,
            })
        });
        references[0].connect_to(&mut rotation.spinning_body_ref1_in_msg);
        references[1].connect_to(&mut rotation.spinning_body_ref2_in_msg);
        rotation
            .prescribed_rotation_out_msg
            .connect_to(&mut platform.prescribed_rotation_in);
        let fsw_rotation = rotation.prescribed_rotation_out_msg.clone();
        let body_rotation = platform.prescribed_rotation_out.clone();

        let step_s = STEP_NANOS as f64 * 1e-9;
        let duration_s = (4.0 * theta_ref_rad / ANG_ACCEL_MAX_RADPS2).sqrt();
        let switch_tick = (0.5 * duration_s / step_s).floor() as usize;
        // The first update after the end reports completion; the step after
        // it still uses that message.
        let completion_tick = (duration_s / step_s).floor() as usize + 1;
        let log = run(
            platform,
            &mut [&mut rotation, &mut translation],
            completion_tick + 1,
        );

        let body = body_rotation.read();
        let fsw = fsw_rotation.read();
        let sigma_ref = prv_to_mrp(theta_ref_rad * Vector3::x());
        assert!(
            (fsw.sigma_pm - sigma_ref).norm() < 1e-12,
            "{:?}",
            fsw.sigma_pm
        );
        assert!(
            (body.sigma_pm - sigma_ref).norm() < 1e-12,
            "{:?}",
            body.sigma_pm
        );
        assert_eq!(body.omega_pm_p_radps, Vector3::zeros());

        let momentum = |index: usize| log[index].rotational_angular_momentum_inertial_kg_m2ps;
        let before_switch = (momentum(switch_tick) - momentum(0)).norm();
        assert!(
            before_switch < 1e-10 * momentum(0).norm(),
            "{before_switch:e}"
        );
        let inertia_xx = 100.0 / 12.0 * 2.0;
        let bound = 3.0 * inertia_xx * ANG_ACCEL_MAX_RADPS2 * step_s;
        let at_completion = (momentum(log.len() - 1) - momentum(0)).norm();
        assert!(at_completion < bound, "{at_completion:e} >= {bound:e}");
        assert_relative_equal(
            &[log[log.len() - 1].orbital_energy_j],
            &[log[0].orbital_energy_j],
            1e-6,
            "orbital energy",
        );
    }

    /// A platform at rest, offset and tilted, carrying `child`, against
    /// `equivalent` mounted on the hub with the platform's offset and tilt
    /// folded into its configuration: mass properties, contributions, state
    /// derivatives, energy and momentum must agree.
    fn assert_static_platform_matches_hub_mount(
        platform: &PrescribedMotionStateEffector,
        equivalent: &dyn StateEffector,
        child_state: &[f64],
    ) {
        let mut platform_state = platform.config.sigma_pm.as_slice().to_vec();
        platform_state.push(0.0);
        platform_state.extend_from_slice(child_state);
        let close = |a: &[f64], b: &[f64], what: &str| {
            let scale = b
                .iter()
                .fold(1.0_f64, |scale, value| scale.max(value.abs()));
            for (a, b) in a.iter().zip(b) {
                assert!(
                    (a - b).abs() <= 1e-11 * scale,
                    "{what}: {a:.15e} vs {b:.15e}"
                );
            }
        };

        let props = platform.mass_properties(&platform_state);
        let expected = equivalent.mass_properties(child_state);
        close(&[props.mass_kg], &[expected.mass_kg], "mass");
        close(
            props.center_of_mass_body_m.as_slice(),
            expected.center_of_mass_body_m.as_slice(),
            "center of mass",
        );
        close(
            props.center_of_mass_prime_body_mps.as_slice(),
            expected.center_of_mass_prime_body_mps.as_slice(),
            "center of mass rate",
        );
        close(
            props.inertia_about_point_b_body_kg_m2.as_slice(),
            expected.inertia_about_point_b_body_kg_m2.as_slice(),
            "inertia",
        );
        close(
            props.inertia_about_point_b_body_prime_kg_m2ps.as_slice(),
            expected.inertia_about_point_b_body_prime_kg_m2ps.as_slice(),
            "inertia rate",
        );

        let omega = Vector3::new(0.2, -0.1, 0.3);
        let gravity = Vector3::new(1.0, -2.0, 0.5);
        let hub = crate::messages::SpacecraftStateMsg {
            omega_radps: omega,
            ..Default::default()
        };
        let mut platform_terms = BackSubMatrices::default();
        let mut expected_terms = BackSubMatrices::default();
        platform.update_contributions(&platform_state, &hub, gravity, &mut platform_terms);
        equivalent.update_contributions(child_state, &hub, gravity, &mut expected_terms);
        for (a, b, what) in [
            (platform_terms.matrix_a, expected_terms.matrix_a, "A"),
            (platform_terms.matrix_b, expected_terms.matrix_b, "B"),
            (platform_terms.matrix_c, expected_terms.matrix_c, "C"),
            (platform_terms.matrix_d, expected_terms.matrix_d, "D"),
        ] {
            close(a.as_slice(), b.as_slice(), what);
        }
        close(
            platform_terms.vec_trans.as_slice(),
            expected_terms.vec_trans.as_slice(),
            "vecTrans",
        );
        close(
            platform_terms.vec_rot.as_slice(),
            expected_terms.vec_rot.as_slice(),
            "vecRot",
        );

        let accel = Vector3::new(0.3, 0.1, -0.2);
        let omega_dot = Vector3::new(-0.02, 0.01, 0.04);
        let derivative = platform.compute_derivatives(&platform_state, accel, omega_dot);
        let expected_derivative = equivalent.compute_derivatives(child_state, accel, omega_dot);
        close(
            &derivative[..4],
            &[0.0, 0.0, 0.0, 1.0],
            "platform derivative",
        );
        close(&derivative[4..], &expected_derivative, "child derivative");

        close(
            &[platform.rotational_energy_j(&platform_state, omega)],
            &[equivalent.rotational_energy_j(child_state, omega)],
            "energy",
        );
        close(
            platform
                .rotational_angular_momentum_body(&platform_state, omega)
                .as_slice(),
            equivalent
                .rotational_angular_momentum_body(child_state, omega)
                .as_slice(),
            "angular momentum",
        );
    }

    /// A massless, inertia-free platform at rest, offset by `r_PB_B` and
    /// tilted by `[BP]`.
    fn static_platform() -> (PrescribedMotionConfig, Vector3<f64>, Matrix3<f64>) {
        let mut config = PrescribedMotionConfig::new("platform");
        config.inertia_about_com_p_kg_m2 = Matrix3::zeros();
        config.mount_position_body_m = Vector3::new(0.3, -0.2, 0.5);
        config.sigma_mb = Vector3::new(0.1, 0.2, -0.1);
        config.r_pm_m_m = Vector3::new(0.1, 0.0, 0.2);
        config.sigma_pm = Vector3::new(-0.05, 0.1, 0.2);
        let dcm_bm = mrp_to_dcm(config.sigma_mb).transpose();
        let dcm_bp = dcm_bm * mrp_to_dcm(config.sigma_pm).transpose();
        let r_pb_b = dcm_bm * config.r_pm_m_m + config.mount_position_body_m;
        (config, r_pb_b, dcm_bp)
    }

    #[test]
    fn static_platform_carries_a_spinning_body_like_the_hub() {
        let (config, r_pb_b, dcm_bp) = static_platform();
        let mut child = branch_spinner();
        child.center_of_mass_spinner_m = Vector3::new(0.2, -0.3, 0.1);
        child.body_to_spinner0_dcm = mrp_to_dcm(Vector3::new(0.3, 0.0, -0.2));
        let mut equivalent = child.clone();
        equivalent.spinner_origin_body_m = r_pb_b + dcm_bp * child.spinner_origin_body_m;
        equivalent.body_to_spinner0_dcm = child.body_to_spinner0_dcm * dcm_bp.transpose();
        let mut platform = PrescribedMotionStateEffector::new(config);
        platform.add_state_effector(SpinningBodyOneDofStateEffector::new(child));

        assert_static_platform_matches_hub_mount(
            &platform,
            &SpinningBodyOneDofStateEffector::new(equivalent),
            &[0.3, -0.1],
        );
    }

    #[test]
    fn static_platform_carries_a_two_dof_spinning_body_like_the_hub() {
        let (config, r_pb_b, dcm_bp) = static_platform();
        let mut child = branch_pair();
        child.body1.center_of_mass_spinner_m = Vector3::new(0.1, 0.2, 0.0);
        child.body2.center_of_mass_spinner_m = Vector3::new(-0.2, 0.0, 0.3);
        child.body2.spin_axis_spinner = Vector3::y();
        child.body_to_spinner1_dcm = mrp_to_dcm(Vector3::new(0.0, 0.2, 0.1));
        let mut equivalent = child.clone();
        equivalent.spinner1_origin_body_m = r_pb_b + dcm_bp * child.spinner1_origin_body_m;
        equivalent.body_to_spinner1_dcm = child.body_to_spinner1_dcm * dcm_bp.transpose();
        let mut platform = PrescribedMotionStateEffector::new(config);
        platform.add_state_effector(SpinningBodyTwoDofStateEffector::new(child));

        assert_static_platform_matches_hub_mount(
            &platform,
            &SpinningBodyTwoDofStateEffector::new(equivalent),
            &[0.2, 0.05, -0.4, 0.1],
        );
    }

    #[test]
    fn static_platform_carries_a_translating_body_like_the_hub() {
        let (config, r_pb_b, dcm_bp) = static_platform();
        let mut child = branch_slider();
        child.axis_body = Vector3::new(0.0, 0.6, 0.8);
        child.center_of_mass_translating_m = Vector3::new(0.1, 0.2, -0.1);
        child.body_to_translating_dcm = mrp_to_dcm(Vector3::new(-0.1, 0.3, 0.0));
        let mut equivalent = child.clone();
        equivalent.origin0_body_m = r_pb_b + dcm_bp * child.origin0_body_m;
        equivalent.axis_body = dcm_bp * child.axis_body;
        equivalent.body_to_translating_dcm = child.body_to_translating_dcm * dcm_bp.transpose();
        let mut platform = PrescribedMotionStateEffector::new(config);
        platform.add_state_effector(LinearTranslationOneDofStateEffector::new(child));

        assert_static_platform_matches_hub_mount(
            &platform,
            &LinearTranslationOneDofStateEffector::new(equivalent),
            &[0.7, 0.02],
        );
    }

    /// A massless platform in motion carrying a locked `child` (spring
    /// constants zero) against a platform with the child's mass properties
    /// as its own: the attached body moves rigidly with P, so mass
    /// properties, contributions, energy and momentum must agree. This checks
    /// the coupling terms that do not involve the joint accelerations.
    fn assert_locked_child_moves_like_platform_mass<E>(child: E, child_state: &[f64])
    where
        E: super::PrescribedMotionAttachment + 'static,
    {
        let mut moving = PrescribedMotionConfig::new("moving");
        moving.inertia_about_com_p_kg_m2 = Matrix3::zeros();
        moving.mount_position_body_m = Vector3::new(0.4, 0.1, -0.3);
        moving.sigma_mb = Vector3::new(0.05, -0.1, 0.2);
        moving.r_pm_m_m = Vector3::new(0.2, -0.1, 0.3);
        moving.r_prime_pm_m_mps = Vector3::new(0.01, 0.03, -0.02);
        moving.r_prime_prime_pm_m_mps2 = Vector3::new(-0.004, 0.002, 0.005);
        moving.omega_pm_p_radps = Vector3::new(0.02, -0.05, 0.03);
        moving.omega_prime_pm_p_radps2 = Vector3::new(0.003, 0.001, -0.002);
        moving.sigma_pm = Vector3::new(0.1, -0.2, 0.05);

        let props = child.mass_properties(child_state);
        let r = props.center_of_mass_body_m;
        let mut rigid = moving.clone();
        rigid.mass_kg = props.mass_kg;
        rigid.center_of_mass_p_m = r;
        rigid.inertia_about_com_p_kg_m2 = props.inertia_about_point_b_body_kg_m2
            - props.mass_kg * (r.norm_squared() * Matrix3::identity() - r * r.transpose());
        let rigid = PrescribedMotionStateEffector::new(rigid);
        let mut platform = PrescribedMotionStateEffector::new(moving);
        platform.add_state_effector(child);

        // Part way into a step, so the prescribed translation and rate have
        // moved from their epoch values.
        let tau_s = 0.3;
        let mut platform_state = platform.config.sigma_pm.as_slice().to_vec();
        platform_state.push(tau_s);
        let rigid_state = platform_state.clone();
        platform_state.extend_from_slice(child_state);
        let close = |a: &[f64], b: &[f64], what: &str| {
            let scale = b
                .iter()
                .fold(1.0_f64, |scale, value| scale.max(value.abs()));
            for (a, b) in a.iter().zip(b) {
                assert!(
                    (a - b).abs() <= 1e-11 * scale,
                    "{what}: {a:.15e} vs {b:.15e}"
                );
            }
        };

        let props = platform.mass_properties(&platform_state);
        let expected = rigid.mass_properties(&rigid_state);
        close(&[props.mass_kg], &[expected.mass_kg], "mass");
        close(
            props.center_of_mass_body_m.as_slice(),
            expected.center_of_mass_body_m.as_slice(),
            "center of mass",
        );
        close(
            props.center_of_mass_prime_body_mps.as_slice(),
            expected.center_of_mass_prime_body_mps.as_slice(),
            "center of mass rate",
        );
        close(
            props.inertia_about_point_b_body_kg_m2.as_slice(),
            expected.inertia_about_point_b_body_kg_m2.as_slice(),
            "inertia",
        );
        close(
            props.inertia_about_point_b_body_prime_kg_m2ps.as_slice(),
            expected.inertia_about_point_b_body_prime_kg_m2ps.as_slice(),
            "inertia rate",
        );

        let omega = Vector3::new(0.2, -0.1, 0.3);
        let gravity = Vector3::new(1.0, -2.0, 0.5);
        let hub = crate::messages::SpacecraftStateMsg {
            omega_radps: omega,
            ..Default::default()
        };
        let mut platform_terms = BackSubMatrices::default();
        let mut expected_terms = BackSubMatrices::default();
        platform.update_contributions(&platform_state, &hub, gravity, &mut platform_terms);
        rigid.update_contributions(&rigid_state, &hub, gravity, &mut expected_terms);
        for matrix in [
            platform_terms.matrix_a,
            platform_terms.matrix_b,
            platform_terms.matrix_c,
            platform_terms.matrix_d,
        ] {
            assert!(
                matrix.norm() < 1e-12,
                "a locked body adds no coupling matrix: {matrix}"
            );
        }
        close(
            platform_terms.vec_trans.as_slice(),
            expected_terms.vec_trans.as_slice(),
            "vecTrans",
        );
        close(
            platform_terms.vec_rot.as_slice(),
            expected_terms.vec_rot.as_slice(),
            "vecRot",
        );
        close(
            &[platform.rotational_energy_j(&platform_state, omega)],
            &[rigid.rotational_energy_j(&rigid_state, omega)],
            "energy",
        );
        close(
            platform
                .rotational_angular_momentum_body(&platform_state, omega)
                .as_slice(),
            rigid
                .rotational_angular_momentum_body(&rigid_state, omega)
                .as_slice(),
            "angular momentum",
        );
    }

    #[test]
    fn locked_spinning_body_moves_rigidly_with_the_platform() {
        let mut config = branch_spinner();
        config.spring_constant_nm_per_rad = 0.0;
        config.center_of_mass_spinner_m = Vector3::new(0.2, -0.3, 0.1);
        config.body_to_spinner0_dcm = mrp_to_dcm(Vector3::new(0.3, 0.0, -0.2));
        let mut child = SpinningBodyOneDofStateEffector::new(config);
        child.locked = true;
        assert_locked_child_moves_like_platform_mass(child, &[0.3, 0.0]);
    }

    #[test]
    fn locked_two_dof_spinning_body_moves_rigidly_with_the_platform() {
        let mut config = branch_pair();
        config.body1.spring_constant_nm_per_rad = 0.0;
        config.body2.spring_constant_nm_per_rad = 0.0;
        config.body1.center_of_mass_spinner_m = Vector3::new(0.1, 0.2, 0.0);
        config.body2.center_of_mass_spinner_m = Vector3::new(-0.2, 0.0, 0.3);
        config.body2.spin_axis_spinner = Vector3::y();
        let mut child = SpinningBodyTwoDofStateEffector::new(config);
        child.locked = [true, true];
        assert_locked_child_moves_like_platform_mass(child, &[0.2, 0.0, -0.4, 0.0]);
    }

    #[test]
    fn locked_translating_body_moves_rigidly_with_the_platform() {
        let mut config = branch_slider();
        config.spring_constant_n_per_m = 0.0;
        config.axis_body = Vector3::new(0.0, 0.6, 0.8);
        config.center_of_mass_translating_m = Vector3::new(0.1, 0.2, -0.1);
        config.body_to_translating_dcm = mrp_to_dcm(Vector3::new(-0.1, 0.3, 0.0));
        let mut child = LinearTranslationOneDofStateEffector::new(config);
        child.locked = true;
        assert_locked_child_moves_like_platform_mass(child, &[0.7, 0.0]);
    }

    #[test]
    fn commanded_attitude_resets_the_integrated_attitude() {
        let mut platform = PrescribedMotionStateEffector::new(PrescribedMotionConfig::new("p"));
        let rotation = Output::new(PrescribedRotationMsg {
            omega_pm_p_radps: Vector3::new(0.0, 0.0, 0.1),
            omega_prime_pm_p_radps2: Vector3::new(0.0, 0.0, 0.01),
            sigma_pm: Vector3::new(0.0, 0.0, 0.2),
        });
        rotation.connect_to(&mut platform.prescribed_rotation_in);
        let translation = Output::new(PrescribedTranslationMsg {
            r_pm_m_m: Vector3::new(1.0, 0.0, 0.0),
            r_prime_pm_m_mps: Vector3::new(0.5, 0.0, 0.0),
            r_prime_prime_pm_m_mps2: Vector3::new(-0.2, 0.0, 0.0),
        });
        translation.connect_to(&mut platform.prescribed_translation_in);
        let initial = platform.initial_state();
        assert_eq!(initial, vec![0.0, 0.0, 0.0, 0.0]);
        platform.load_state(&initial);
        platform.pre_integration(0, 0.5);
        assert_eq!(platform.sigma_pm(), Vector3::new(0.0, 0.0, 0.2));

        // Half a second into the step, then the step completes there.
        let mut state = vec![0.0, 0.0, 0.0, 0.5];
        platform.load_state(&state);
        let (translation, rotation) = platform.prescribed_state();
        assert!((translation.r_pm_m_m.x - (1.0 + 0.25 - 0.025)).abs() < 1e-15);
        assert!((translation.r_prime_pm_m_mps.x - 0.4).abs() < 1e-15);
        assert!((rotation.omega_pm_p_radps.z - 0.105).abs() < 1e-15);
        platform.modify_states(500_000_000, &mut state);
        assert_eq!(state, vec![0.0, 0.0, 0.2, 0.0]);
        platform.load_state(&state);
        assert_eq!(platform.sigma_pm(), Vector3::new(0.0, 0.0, 0.2));
        let (translation, _) = platform.prescribed_state();
        assert!((translation.r_pm_m_m.x - 1.225).abs() < 1e-15);
    }

    #[test]
    fn mass_validation() {
        let mut config = PrescribedMotionConfig::new("p");
        assert_eq!(config.validate(), Ok(()));
        config.mass_kg = -1.0;
        assert!(config.validate().is_err());
        config.mass_kg = f64::NAN;
        assert!(config.validate().is_err());
    }

    #[test]
    #[should_panic(expected = "mass_kg must be finite and non-negative")]
    fn spacecraft_initialization_rejects_negative_mass() {
        let mut config = PrescribedMotionConfig::new("p");
        config.mass_kg = -1.0;
        let _ = PrescribedMotionStateEffector::new(config).initial_state();
    }
}
