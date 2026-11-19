//! Spacecraft integrated together: upstream `Spacecraft` objects joined with
//! `syncDynamicsIntegration`, which the primary's integrator advances in one
//! step.
//!
//! A [`SpacecraftGroup`] owns its spacecraft and is the module that the
//! scheduler updates in their place. Each update runs every spacecraft's
//! pre-integration, advances all of them with the group's
//! [`integrator`](SpacecraftGroup::integrator) as one
//! [`Synchronized`] dynamic object, and then commits and publishes each
//! spacecraft exactly as it would on its own. [`CouplingEffector`]s, such as
//! [`ConstraintDynamicEffector`](crate::dynamics::constraint_dynamic_effector::ConstraintDynamicEffector),
//! act between the spacecraft: at every stage they see the stage state of all
//! of them.
//!
//! Upstream the secondary spacecraft stay separate modules whose updates only
//! publish; here their outputs are written by the group's update, in member
//! order. Every member must share one integration step, and the members' own
//! `SpacecraftConfig::integrator` must be unset: upstream rejects an
//! integrator set on a synced secondary, and the group's integrator is the
//! primary's.

use hifitime::Epoch;

use super::{
    CouplingEffector, EffectorOutput, IntegratedState, Spacecraft, StateDerivative, StepStart,
};
use crate::integrators::{Coupling, DynamicObject, Integrator, Rk4, Synchronized};
use crate::messages::SpacecraftStateMsg;
use crate::{Module, SimulationContext};

/// The coupling of a [`SpacecraftGroup`]: its [`CouplingEffector`]s.
#[derive(Default)]
pub struct SpacecraftCoupling {
    effectors: Vec<Box<dyn CouplingEffector>>,
}

impl SpacecraftCoupling {
    /// Adds an effector acting between the group's spacecraft.
    pub fn add_effector<E: CouplingEffector + 'static>(&mut self, effector: E) {
        self.effectors.push(Box::new(effector));
    }

    /// The coupling effectors, in the order they were added.
    pub fn effectors(&self) -> &[Box<dyn CouplingEffector>] {
        &self.effectors
    }

    /// The coupling effectors, in the order they were added.
    pub fn effectors_mut(&mut self) -> &mut [Box<dyn CouplingEffector>] {
        &mut self.effectors
    }
}

impl Coupling<Spacecraft> for SpacecraftCoupling {
    /// Each spacecraft's drift with the loads every coupling effector puts on
    /// it at the stage. Without coupling effectors, each spacecraft's own
    /// equations of motion.
    fn equations_of_motion(
        &mut self,
        members: &mut [Spacecraft],
        states: &[IntegratedState],
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Vec<StateDerivative> {
        if self.effectors.is_empty() {
            return members
                .iter_mut()
                .zip(states)
                .map(|(member, state)| {
                    member.equations_of_motion(state, current_sim_nanos, current_epoch)
                })
                .collect();
        }
        let stage_states: Vec<SpacecraftStateMsg> = members
            .iter()
            .zip(states)
            .map(|(member, state)| member.stage_state_msg(state, current_sim_nanos))
            .collect();
        let mut loads = vec![EffectorOutput::default(); members.len()];
        for effector in &mut self.effectors {
            effector.add_loads(&stage_states, &mut loads);
        }
        members
            .iter_mut()
            .zip(states)
            .zip(&loads)
            .map(|((member, state), load)| {
                member.equations_of_motion_with_load(state, current_sim_nanos, Some(load))
            })
            .collect()
    }
}

/// The dynamics of a [`SpacecraftGroup`]: what its integrator advances.
pub type SpacecraftGroupDynamics = Synchronized<Spacecraft, SpacecraftCoupling>;

/// Spacecraft integrated together, with the effectors acting between them:
/// upstream `syncDynamicsIntegration`, as a module.
pub struct SpacecraftGroup {
    /// The spacecraft and the coupling effectors.
    pub dynamics: SpacecraftGroupDynamics,
    /// The integration scheme advancing every spacecraft at once, like
    /// `SpacecraftConfig::integrator` for one. Defaults to
    /// [`Rk4`] if `None`.
    pub integrator: Option<Box<dyn Integrator<SpacecraftGroupDynamics>>>,
}

impl SpacecraftGroup {
    /// Groups `spacecraft`, in order: upstream the first is the primary and
    /// the others are synced to it. Coupling effectors index the spacecraft
    /// by their position here.
    ///
    /// # Panics
    ///
    /// If `spacecraft` is empty.
    pub fn new(spacecraft: Vec<Spacecraft>) -> Self {
        Self {
            dynamics: Synchronized::new(spacecraft, SpacecraftCoupling::default()),
            integrator: None,
        }
    }

    /// Adds an effector acting between the spacecraft.
    pub fn add_coupling_effector<E: CouplingEffector + 'static>(&mut self, effector: E) {
        self.dynamics.coupling.add_effector(effector);
    }

    /// The spacecraft, in order.
    pub fn spacecraft(&self) -> &[Spacecraft] {
        self.dynamics.members()
    }

    /// The spacecraft, in order.
    pub fn spacecraft_mut(&mut self) -> &mut [Spacecraft] {
        self.dynamics.members_mut()
    }

    /// The coupling effectors, in the order they were added.
    pub fn coupling_effectors(&self) -> &[Box<dyn CouplingEffector>] {
        self.dynamics.coupling.effectors()
    }

    /// The coupling effectors, in the order they were added.
    pub fn coupling_effectors_mut(&mut self) -> &mut [Box<dyn CouplingEffector>] {
        self.dynamics.coupling.effectors_mut()
    }

    /// Dissolves the group.
    pub fn into_spacecraft(self) -> Vec<Spacecraft> {
        self.dynamics.into_members()
    }

    fn update_coupling_effectors(&mut self, current_sim_nanos: u64) {
        for effector in self.dynamics.coupling.effectors_mut() {
            effector.update(current_sim_nanos);
        }
    }
}

impl Module for SpacecraftGroup {
    /// Checks that the spacecraft can share a step, initializes each one,
    /// then the coupling effectors.
    ///
    /// # Panics
    ///
    /// If the spacecraft have different integration steps or one has its own
    /// integrator, or a coupling effector rejects its configuration.
    fn init(&mut self) {
        let spacecraft = self.dynamics.members();
        let step_nanos = spacecraft[0].config.integration_step_nanos;
        for (index, member) in spacecraft.iter().enumerate() {
            assert_eq!(
                member.config.integration_step_nanos, step_nanos,
                "spacecraft {index} of a group has a different integration step than spacecraft 0"
            );
            assert!(
                member.config.integrator.is_none(),
                "spacecraft {index} of a group has its own integrator; set \
                 SpacecraftGroup::integrator instead"
            );
        }
        for member in self.dynamics.members_mut() {
            member.init();
        }
        let spacecraft_count = self.dynamics.members().len();
        for effector in self.dynamics.coupling.effectors_mut() {
            effector.init(spacecraft_count);
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        let starts: Vec<Option<(StepStart, IntegratedState)>> = self
            .dynamics
            .members_mut()
            .iter_mut()
            .map(|member| member.begin_step(context))
            .collect();
        // The members share the integration step, so either all of them
        // integrate or none does (t = 0, or a zero step).
        if starts.iter().all(Option::is_none) {
            self.update_coupling_effectors(context.current_sim_nanos);
            return;
        }
        let (steps, states): (Vec<StepStart>, Vec<IntegratedState>) = starts
            .into_iter()
            .map(|start| start.expect("group members share one integration step"))
            .unzip();
        let (start_nanos, start_epoch, dt_seconds) = (
            steps[0].start_nanos,
            steps[0].start_epoch,
            steps[0].dt_seconds,
        );

        for effector in self.dynamics.coupling.effectors_mut() {
            effector.pre_integration(start_nanos, dt_seconds);
        }
        let default_integrator = Rk4;
        let chosen: &dyn Integrator<SpacecraftGroupDynamics> =
            self.integrator.as_deref().unwrap_or(&default_integrator);
        let (new_states, gravity_delta_velocities) = chosen.propagate(
            &mut self.dynamics,
            &states,
            start_nanos,
            start_epoch,
            dt_seconds,
        );

        for (((member, step), new_state), gravity_delta_velocity) in self
            .dynamics
            .members_mut()
            .iter_mut()
            .zip(&steps)
            .zip(new_states)
            .zip(gravity_delta_velocities)
        {
            member.finish_step(context, step, new_state, gravity_delta_velocity);
        }
        self.update_coupling_effectors(context.current_sim_nanos);
    }
}

#[cfg(test)]
mod tests;
