use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::structs::BackSubMatrices;
use crate::spacecraft::{EffectorOutput, IntegratedState, Spacecraft, StateEffectorMassProps};
use nalgebra::Vector3;
use std::any::Any;

pub trait DynamicEffector: Send {
    fn name(&self) -> &str;
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}
    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput;
    /// Force and torque at a derivative evaluation at simulation time
    /// `current_sim_nanos` (upstream `computeForceTorque(integTime, timeStep)`),
    /// for effectors whose loads depend on the time of the evaluation, such as
    /// a pulse train or an Euler-stepped planet position. A spacecraft that
    /// evaluates its effectors at integrator stages passes the stage time.
    ///
    /// The default ignores the time and returns
    /// [`compute_output`](Self::compute_output).
    fn compute_output_at(
        &self,
        state: &SpacecraftStateMsg,
        _current_sim_nanos: u64,
    ) -> EffectorOutput {
        self.compute_output(state)
    }
    fn as_any(&self) -> &dyn Any;

    /// Called at the start of every derivative evaluation, before any
    /// [`compute_output`](Self::compute_output) of that evaluation, with its
    /// integrator stage time: upstream passes `integTime` to
    /// `computeForceTorque`, where a thruster decides whether it still fires.
    /// The default does nothing.
    fn begin_stage(&mut self, _stage_nanos: u64) {}
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

    /// Called at the start of every derivative evaluation, after the stage
    /// state is loaded ([`load_state`](Self::load_state)) and before any
    /// other call of that evaluation, with its integrator stage time:
    /// upstream passes `integTime` to every state-effector call, where a
    /// thruster decides whether it still fires. The default does nothing.
    fn begin_stage(&mut self, _stage_nanos: u64) {}

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

/// An effector acting between the spacecraft of a
/// [`SpacecraftGroup`](crate::spacecraft::SpacecraftGroup), such as upstream
/// `constraintDynamicEffector`: upstream a `DynamicEffector` added to every
/// spacecraft it acts on, whose integration is synchronized
/// (`syncDynamicsIntegration`).
///
/// At every integrator stage the group hands the effector the stage state of
/// every spacecraft at once, before it evaluates any spacecraft's equations of
/// motion, so the effector sees one consistent stage and computes the loads
/// on all of them together. Upstream instead calls the effector once per
/// spacecraft and the effector alternates which spacecraft it serves.
pub trait CouplingEffector: Send {
    fn name(&self) -> &str;

    /// Called when the group initializes (upstream `Reset` and
    /// `linkInStates`), with the number of spacecraft in the group. Panics on
    /// a configuration it cannot run with.
    fn init(&mut self, _spacecraft_count: usize) {}

    /// Called before each integration step, like
    /// [`DynamicEffector::pre_integration`].
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}

    /// Adds this effector's load on each spacecraft at the stage states
    /// `states` to `loads`: a force in inertial components and a torque about
    /// the body origin in body components, as a [`DynamicEffector`] returns
    /// them. `states[i]` and `loads[i]` belong to group member `i`; the states
    /// are inertial, as [`DynamicEffector::compute_output`] sees them.
    fn add_loads(&mut self, states: &[SpacecraftStateMsg], loads: &mut [EffectorOutput]);

    /// [`add_loads`](Self::add_loads) with the group's spacecraft and their
    /// integrated stage states, for an effector that also acts on the
    /// spacecraft's state effectors (upstream's dynamic effectors attached to
    /// a state effector, "branching"): it reads an effector's stage state
    /// from `members[i].state_effectors[k]` and `stage[i].effector_states[k]`
    /// and hands it its load before the group evaluates `members[i]`. The
    /// group calls this one; the default is `add_loads`.
    fn add_member_loads(
        &mut self,
        _members: &[Spacecraft],
        _stage: &[IntegratedState],
        states: &[SpacecraftStateMsg],
        loads: &mut [EffectorOutput],
    ) {
        self.add_loads(states, loads);
    }

    /// Called once per group update after the spacecraft have integrated and
    /// published, and at `t = 0` after they publish their initial outputs:
    /// upstream the effector's own `UpdateState`, scheduled after the
    /// spacecraft. Reads inputs and writes outputs.
    fn update(&mut self, _current_sim_nanos: u64) {}

    fn as_any(&self) -> &dyn Any;

    fn as_any_mut(&mut self) -> &mut dyn Any;
}
