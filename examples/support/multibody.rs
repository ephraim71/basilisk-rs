//! Setup shared by the ports of upstream's multibody example scenarios
//! (`scenarioHingedRigidBody`, `scenarioDeployingPanel`, `scenarioFuelSlosh`,
//! `scenarioSpinningBodiesTwoDOF`, ...).
//!
//! * [`set_center_of_mass_initial_state`]: upstream `hub.r_CN_NInit` and
//!   `hub.v_CN_NInit`, the initial state of the spacecraft's center of mass,
//!   which basilisk-rs's [`SpacecraftConfig`](basilisk_rs::spacecraft::SpacecraftConfig)
//!   takes at the body origin B instead.
//! * [`Hosted`]: a state effector as the spacecraft holds it, with changes
//!   staged between runs ([`EffectorEdits`], upstream's setters called
//!   between `ExecuteSimulation`s) and observers of its committed state
//!   (upstream `dynManager.getStateObject(...)` read by a
//!   `pythonVariableLogger`).
//!
//! Include it next to `common.rs`:
//!
//! ```ignore
//! #[path = "support/multibody.rs"]
//! mod multibody;
//! ```

#![allow(dead_code)]

use std::any::Any;
use std::sync::{Arc, Mutex};

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::spacecraft::{BackSubMatrices, Spacecraft, StateEffector, StateEffectorMassProps};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use nalgebra::{Matrix3, Vector3};

/// The spacecraft's center of mass `c_B` and its inertial-frame rate
/// `cDot_B = c'_B + ω × c_B` (body components) at the initial state of its
/// hub and effectors: upstream `Spacecraft::updateSCMassProps` at time zero.
pub fn initial_center_of_mass(spacecraft: &Spacecraft) -> (Vector3<f64>, Vector3<f64>) {
    let hub_mass = spacecraft.config.mass_kg;
    let mut mass = hub_mass;
    let mut mass_rate = 0.0;
    let mut first_moment = hub_mass * spacecraft.config.hub_center_of_mass_body_m;
    let mut first_moment_rate = Vector3::zeros();
    for effector in &spacecraft.state_effectors {
        let props = effector.mass_properties(&effector.initial_state());
        mass += props.mass_kg;
        mass_rate += props.mass_dot_kgps;
        first_moment += props.mass_kg * props.center_of_mass_body_m;
        first_moment_rate += props.mass_kg * props.center_of_mass_prime_body_mps
            + props.mass_dot_kgps * props.center_of_mass_body_m;
    }
    let c_b = first_moment / mass;
    let c_prime_b = first_moment_rate / mass - mass_rate * c_b / mass;
    let c_dot_b = c_prime_b + spacecraft.config.initial_omega_radps.cross(&c_b);
    (c_b, c_dot_b)
}

/// Upstream `hub.r_CN_NInit = r_CN_N`, `hub.v_CN_NInit = v_CN_N`: places the
/// spacecraft so that its center of mass starts at `r_cn_n_m` with velocity
/// `v_cn_n_mps`. Upstream's `initializeDynamics` subtracts `[NB] c_B` and
/// `[NB] cDot_B` from them to get the hub states `r_BN_N`, `v_BN_N`; this
/// does the same with the effectors already added (call it after
/// `add_state_effector`, with the initial attitude and rate set).
pub fn set_center_of_mass_initial_state(
    spacecraft: &mut Spacecraft,
    r_cn_n_m: Vector3<f64>,
    v_cn_n_mps: Vector3<f64>,
) {
    let (c_b, c_dot_b) = initial_center_of_mass(spacecraft);
    let dcm_nb = mrp_to_dcm(spacecraft.config.initial_sigma_bn).transpose();
    spacecraft.config.initial_position_m = r_cn_n_m - dcm_nb * c_b;
    spacecraft.config.initial_velocity_mps = v_cn_n_mps - dcm_nb * c_dot_b;
}

/// Inertia of a uniform rectangular box of `mass` about its center, edges
/// `x`, `y`, `z` along the frame axes.
pub fn box_inertia(mass: f64, x: f64, y: f64, z: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(
        mass / 12.0 * (y * y + z * z),
        mass / 12.0 * (x * x + z * z),
        mass / 12.0 * (x * x + y * y),
    ))
}

type Edit<E> = Box<dyn FnOnce(&mut E) + Send>;
type Observer<E> = Box<dyn FnMut(&E, u64, &SpacecraftStateMsg) + Send>;

/// Changes to a [`Hosted`] effector made between runs: upstream's setters
/// on an effector the spacecraft already integrates. Clones share the queue.
pub struct EffectorEdits<E>(Arc<Mutex<Vec<Edit<E>>>>);

impl<E> Clone for EffectorEdits<E> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<E> Default for EffectorEdits<E> {
    fn default() -> Self {
        Self(Arc::new(Mutex::new(Vec::new())))
    }
}

impl<E> EffectorEdits<E> {
    /// Queues `edit` for the start of the next integration step, the first
    /// one upstream's changed effector takes part in.
    pub fn edit(&self, edit: impl FnOnce(&mut E) + Send + 'static) {
        self.0.lock().expect("effector edits").push(Box::new(edit));
    }
}

/// A state effector as the spacecraft holds it: `inner`, taking the pending
/// [`EffectorEdits`] before each integration step, and calling its observers
/// with its committed state after every step (and at initialization), when
/// the spacecraft writes its outputs.
pub struct Hosted<E> {
    pub inner: E,
    edits: EffectorEdits<E>,
    observers: Vec<Observer<E>>,
}

impl<E: StateEffector> Hosted<E> {
    pub fn new(inner: E) -> Self {
        Self {
            inner,
            edits: EffectorEdits::default(),
            observers: Vec::new(),
        }
    }

    /// The handle the scenario stages its changes through.
    pub fn edits(&self) -> EffectorEdits<E> {
        self.edits.clone()
    }

    /// Calls `observer(effector, time_ns, hub_state)` with the committed
    /// state at every output write.
    pub fn observe(
        mut self,
        observer: impl FnMut(&E, u64, &SpacecraftStateMsg) + Send + 'static,
    ) -> Self {
        self.observers.push(Box::new(observer));
        self
    }
}

impl<E: StateEffector + 'static> StateEffector for Hosted<E> {
    fn name(&self) -> &str {
        self.inner.name()
    }
    fn state_len(&self) -> usize {
        self.inner.state_len()
    }
    fn initial_state(&self) -> Vec<f64> {
        self.inner.initial_state()
    }
    fn load_state(&mut self, state: &[f64]) {
        self.inner.load_state(state);
    }
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        for edit in self.edits.0.lock().expect("effector edits").drain(..) {
            edit(&mut self.inner);
        }
        self.inner.pre_integration(current_sim_nanos, dt_seconds);
    }
    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.inner
            .update_contributions(effector_state, hub_state, gravity_body_mps2, back_sub);
    }
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.inner
            .compute_derivatives(effector_state, body_trans_accel_mps2, body_omega_dot_radps2)
    }
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        self.inner.mass_properties(effector_state)
    }
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.inner
            .rotational_angular_momentum_body(effector_state, body_omega_radps)
    }
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.inner
            .rotational_energy_j(effector_state, body_omega_radps)
    }
    fn write_outputs(&mut self, current_sim_nanos: u64, hub_state: &SpacecraftStateMsg) {
        self.inner.write_outputs(current_sim_nanos, hub_state);
        for observer in &mut self.observers {
            observer(&self.inner, current_sim_nanos, hub_state);
        }
    }
    fn modify_states(&mut self, current_sim_nanos: u64, effector_state: &mut [f64]) {
        self.inner.modify_states(current_sim_nanos, effector_state);
    }
    fn as_any(&self) -> &dyn Any {
        self.inner.as_any()
    }
    fn begin_stage(&mut self, stage_nanos: u64) {
        self.inner.begin_stage(stage_nanos);
    }
    fn noise_source_count(&self) -> usize {
        self.inner.noise_source_count()
    }
    fn compute_diffusion(&self, effector_state: &[f64]) -> Vec<Vec<f64>> {
        self.inner.compute_diffusion(effector_state)
    }
}

/// A shared time history filled by a [`Hosted`] observer: `(time_ns, value)`
/// in recording order. Clones share the samples.
pub struct SharedLog<T>(Arc<Mutex<Vec<(u64, T)>>>);

impl<T> Clone for SharedLog<T> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<T> Default for SharedLog<T> {
    fn default() -> Self {
        Self(Arc::new(Mutex::new(Vec::new())))
    }
}

impl<T: Clone> SharedLog<T> {
    /// Appends `value` at `time_nanos`; a sample at the time of the last one
    /// replaces it (the spacecraft writes its outputs both when it
    /// initializes and at its first update, at time zero).
    pub fn push(&self, time_nanos: u64, value: T) {
        let mut samples = self.0.lock().expect("shared log");
        if samples.last().is_some_and(|(last, _)| *last == time_nanos) {
            samples.pop();
        }
        samples.push((time_nanos, value));
    }

    /// A copy of the samples.
    pub fn samples(&self) -> Vec<(u64, T)> {
        self.0.lock().expect("shared log").clone()
    }
}
