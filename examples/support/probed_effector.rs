//! [`Probed`]: a state effector that also publishes what upstream scenarios
//! read from its states, and takes changes between runs.
//!
//! Upstream scripts reach into an effector the spacecraft owns:
//!
//! ```python
//! scSim.rhoLog = pythonVariableLogger.PythonVariableLogger({
//!     "rho1": lambda _: scObject.dynManager.getStateObject(rho1Name).getState()})
//! effector.setRhoRef(0.2)                    # between ExecuteSimulation calls
//! ```
//!
//! The spacecraft owns its effectors in basilisk-rs, and a running
//! [`Simulation`](basilisk_rs::simulation::Simulation) holds the spacecraft.
//! A [`Probed`] effector forwards every [`StateEffector`] call to the effector
//! it wraps, and after each step's outputs (`write_outputs`, the committed
//! state) runs its probe, which can write the values a scenario logs to
//! [`Output`](basilisk_rs::messages::Output)s the scenario holds. Changes
//! staged through a [`ProbeEdits`] handle are applied before the next
//! integration step, as upstream's setters take effect at the next step.

#![allow(dead_code)]

use std::any::Any;
use std::sync::{Arc, Mutex};

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use nalgebra::Vector3;

type Probe<E> = Box<dyn FnMut(&E, u64) + Send>;
type Edit<E> = Box<dyn FnOnce(&mut E) + Send>;

/// Changes for a [`Probed`] effector, applied before its next step. Clones
/// share the queue.
pub struct ProbeEdits<E>(Arc<Mutex<Vec<Edit<E>>>>);

impl<E> Clone for ProbeEdits<E> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<E> Default for ProbeEdits<E> {
    fn default() -> Self {
        Self(Arc::new(Mutex::new(Vec::new())))
    }
}

impl<E> ProbeEdits<E> {
    pub fn edit(&self, edit: impl FnOnce(&mut E) + Send + 'static) {
        self.0.lock().expect("probe edits").push(Box::new(edit));
    }
}

/// A state effector with a probe run after each step's outputs.
pub struct Probed<E> {
    pub inner: E,
    probe: Probe<E>,
    edits: ProbeEdits<E>,
}

impl<E> Probed<E> {
    /// Wraps `inner`; `probe(effector, time_nanos)` runs after every output
    /// write, `t = 0` included.
    pub fn new(inner: E, probe: impl FnMut(&E, u64) + Send + 'static) -> Self {
        Self {
            inner,
            probe: Box::new(probe),
            edits: ProbeEdits::default(),
        }
    }

    /// The handle through which changes reach the wrapped effector.
    pub fn edits(&self) -> ProbeEdits<E> {
        self.edits.clone()
    }
}

impl<E: StateEffector + 'static> StateEffector for Probed<E> {
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
        for edit in self.edits.0.lock().expect("probe edits").drain(..) {
            edit(&mut self.inner);
        }
        self.inner.pre_integration(current_sim_nanos, dt_seconds);
    }
    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.inner.update_contributions(
            effector_state,
            body_omega_radps,
            gravity_body_mps2,
            back_sub,
        );
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
        (self.probe)(&self.inner, current_sim_nanos);
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
