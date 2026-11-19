use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::structs::BackSubMatrices;
use crate::spacecraft::{EffectorOutput, StateEffectorMassProps};
use nalgebra::Vector3;
use std::any::Any;

pub trait DynamicEffector: Send {
    fn name(&self) -> &str;
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}
    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput;
    fn as_any(&self) -> &dyn Any;
}

pub trait StateEffector: Send {
    fn name(&self) -> &str;
    fn state_len(&self) -> usize;
    fn initial_state(&self) -> Vec<f64>;
    fn load_state(&mut self, state: &[f64]);
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64);
    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    );
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64>;
    fn mass_properties(&self, _effector_state: &[f64]) -> StateEffectorMassProps {
        StateEffectorMassProps::default()
    }
    fn rotational_angular_momentum_body(
        &self,
        _effector_state: &[f64],
        _body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn rotational_energy_j(&self, _effector_state: &[f64], _body_omega_radps: Vector3<f64>) -> f64 {
        0.0
    }
    fn write_outputs(&mut self, _current_sim_nanos: u64, _hub_state: &SpacecraftStateMsg) {}
    /// Rewrites the effector's committed state into a different but equivalent
    /// form once an integration step is complete (upstream
    /// `StateEffector::modifyStates`), for example to move a parameterization
    /// away from a coordinate singularity. The default leaves the state as is.
    fn modify_states(&mut self, _current_sim_nanos: u64, _effector_state: &mut [f64]) {}
    fn as_any(&self) -> &dyn Any;

    /// Number of independent Wiener processes driving this effector's states
    /// (upstream `StateSpec::noiseCount`). The spacecraft numbers its noise
    /// sources effector by effector, in the order the effectors were added.
    ///
    /// The default is `0`: the effector's states have no diffusion.
    fn noise_source_count(&self) -> usize {
        0
    }

    /// Diffusion columns of this effector's states, one per noise source, each
    /// [`state_len`](Self::state_len) long: the state change per unit Wiener
    /// increment, as upstream effectors write with `StateData::setDiffusion`.
    ///
    /// A stochastic integrator calls this at its diffusion stages with the
    /// stage's effector state; deterministic integrators never do. The default
    /// returns no columns, matching the default
    /// [`noise_source_count`](Self::noise_source_count).
    fn compute_diffusion(&self, _effector_state: &[f64]) -> Vec<Vec<f64>> {
        Vec::new()
    }
}
