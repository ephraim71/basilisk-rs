//! Synchronized integration of several dynamic objects: port of upstream
//! `DynamicObject::syncDynamicsIntegration`.
//!
//! Upstream, a primary object's integrator also advances every object synced
//! to it. Each stage writes the stage state of every object before it
//! evaluates any object's equations of motion, so a force that depends on two
//! objects (a constraint between two spacecraft) sees one consistent stage.
//! Here the group is a value, [`Synchronized`], that owns its members and is
//! itself a [`DynamicObject`] whose state is the members' states in member
//! order. Any [`Integrator`](super::Integrator) advances it unchanged
//! (fixed-step, adaptive or stochastic), with one step, one set of stage times
//! and, for an adaptive method, one step-size control for all members.
//!
//! A [`Coupling`] evaluates the members' equations of motion at a stage. It
//! receives every member's stage state at once, so interactions between
//! members belong there. [`Uncoupled`] evaluates each member on its own.
//!
//! Upstream's pointer bookkeeping (rejecting a second primary or a
//! self-connection, refusing a secondary's own integrator, unlinking on
//! destruction) is ownership here: a member belongs to exactly one group, and
//! the group is integrated as one object.
//!
//! Member by member, the group does what each member's own
//! [`DynamicObject`] methods do, so a one-member group integrates exactly like
//! the member alone. Across members:
//! - adaptive error control takes the largest member
//!   [`error_norm`](DynamicObject::error_norm), so a step is accepted only when
//!   every member meets the tolerance (upstream takes the largest relative error
//!   over every state of every object);
//! - noise sources are numbered member by member, as upstream numbers the
//!   registered states of its objects in order, and a member's diffusion
//!   depends only on its own state.

use hifitime::Epoch;

use super::traits::{DiffusionIncrement, DynamicObject};

/// Evaluates the drift of every member of a [`Synchronized`] group at one
/// stage: the group's
/// [`equations_of_motion`](DynamicObject::equations_of_motion).
///
/// Upstream an object reads the other objects' states through pointers into
/// their state managers, which hold the stage states while the integrator
/// evaluates the stage. Here the coupling gets all of them as arguments.
pub trait Coupling<D: DynamicObject> {
    /// The drift of each member at `states`, one per member and in member
    /// order, at the stage time.
    fn equations_of_motion(
        &mut self,
        members: &mut [D],
        states: &[D::State],
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Vec<D::Derivative>;
}

/// The members do not interact: each member's own
/// [`equations_of_motion`](DynamicObject::equations_of_motion).
#[derive(Clone, Copy, Debug, Default)]
pub struct Uncoupled;

impl<D: DynamicObject> Coupling<D> for Uncoupled {
    fn equations_of_motion(
        &mut self,
        members: &mut [D],
        states: &[D::State],
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Vec<D::Derivative> {
        members
            .iter_mut()
            .zip(states)
            .map(|(member, state)| {
                member.equations_of_motion(state, current_sim_nanos, current_epoch)
            })
            .collect()
    }
}

/// Dynamic objects integrated together: the primary object and the objects
/// synced to it with upstream `syncDynamicsIntegration`.
///
/// The state is one `D::State` per member. A derivative holds one
/// `Option<D::Derivative>` per member, where `None` is a zero derivative: the
/// diffusion column of one member's noise source is zero for every other
/// member.
pub struct Synchronized<D, C> {
    members: Vec<D>,
    /// Evaluates the members' drift at each stage.
    pub coupling: C,
}

impl<D, C> Synchronized<D, C> {
    /// Groups `members`, in order, with `coupling`.
    ///
    /// # Panics
    ///
    /// If `members` is empty.
    pub fn new(members: Vec<D>, coupling: C) -> Self {
        assert!(
            !members.is_empty(),
            "a synchronized group needs at least one member"
        );
        Self { members, coupling }
    }

    /// The members, in order.
    pub fn members(&self) -> &[D] {
        &self.members
    }

    /// The members, in order. A member can be reconfigured but not added or
    /// removed.
    pub fn members_mut(&mut self) -> &mut [D] {
        &mut self.members
    }

    /// Dissolves the group.
    pub fn into_members(self) -> Vec<D> {
        self.members
    }

    fn assert_member_count(&self, count: usize, what: &str) {
        assert_eq!(
            count,
            self.members.len(),
            "synchronized group has {} members but {count} {what}",
            self.members.len()
        );
    }
}

/// Member `index`'s terms of a group weight list, skipping its zero ones.
fn member_terms<'a, T>(weighted: &[(&'a Vec<Option<T>>, f64)], index: usize) -> Vec<(&'a T, f64)> {
    weighted
        .iter()
        .filter_map(|&(derivative, weight)| derivative[index].as_ref().map(|term| (term, weight)))
        .collect()
}

/// Member `index`'s share of group diffusion increments. An increment with no
/// term for the member is dropped: it adds nothing to the member's state.
fn member_increments<'a, T>(
    diffusion: &[DiffusionIncrement<'a, Vec<Option<T>>>],
    index: usize,
) -> Vec<DiffusionIncrement<'a, T>> {
    diffusion
        .iter()
        .filter_map(|increment| {
            let tangents = member_terms(&increment.tangents, index);
            (!tangents.is_empty()).then_some(DiffusionIncrement {
                tangents,
                pseudo_step: increment.pseudo_step,
            })
        })
        .collect()
}

/// Splits group weights into one by-value weight list per member, skipping
/// the zero terms.
fn per_member_weights<T>(
    weighted: Vec<(Vec<Option<T>>, f64)>,
    member_count: usize,
) -> Vec<Vec<(T, f64)>> {
    let mut per_member: Vec<Vec<(T, f64)>> = (0..member_count)
        .map(|_| Vec::with_capacity(weighted.len()))
        .collect();
    for (derivative, weight) in weighted {
        assert_eq!(
            derivative.len(),
            member_count,
            "synchronized group derivative has {} terms for {member_count} members",
            derivative.len()
        );
        for (terms, term) in per_member.iter_mut().zip(derivative) {
            if let Some(term) = term {
                terms.push((term, weight));
            }
        }
    }
    per_member
}

impl<D: DynamicObject, C: Coupling<D>> DynamicObject for Synchronized<D, C> {
    type State = Vec<D::State>;
    type Derivative = Vec<Option<D::Derivative>>;
    type StepOutput = Vec<D::StepOutput>;

    /// The coupling's drift of every member at the stage states.
    fn equations_of_motion(
        &mut self,
        state: &Self::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Self::Derivative {
        self.assert_member_count(state.len(), "states");
        let derivatives = self.coupling.equations_of_motion(
            &mut self.members,
            state,
            current_sim_nanos,
            current_epoch,
        );
        self.assert_member_count(derivatives.len(), "derivatives from the coupling");
        derivatives.into_iter().map(Some).collect()
    }

    fn state_with_derivative(
        &self,
        state: &Self::State,
        derivative: &Self::Derivative,
        dt: f64,
    ) -> Self::State {
        self.members
            .iter()
            .zip(state)
            .zip(derivative)
            .map(|((member, member_state), term)| match term {
                Some(term) => member.state_with_derivative(member_state, term, dt),
                None => member_state.clone(),
            })
            .collect()
    }

    fn state_with_derivatives(
        &self,
        state: &Self::State,
        weighted: &[(&Self::Derivative, f64)],
        step: f64,
    ) -> Self::State {
        self.members
            .iter()
            .zip(state)
            .enumerate()
            .map(|(index, (member, member_state))| {
                member.state_with_derivatives(member_state, &member_terms(weighted, index), step)
            })
            .collect()
    }

    fn combine(
        &self,
        state: &Self::State,
        weighted: Vec<(Self::Derivative, f64)>,
        step: f64,
    ) -> (Self::State, Self::StepOutput) {
        self.assert_member_count(state.len(), "states");
        self.members
            .iter()
            .zip(state)
            .zip(per_member_weights(weighted, self.members.len()))
            .map(|((member, member_state), terms)| member.combine(member_state, terms, step))
            .unzip()
    }

    fn merge_step_outputs(
        &self,
        accumulated: Self::StepOutput,
        next: Self::StepOutput,
    ) -> Self::StepOutput {
        self.members
            .iter()
            .zip(accumulated.into_iter().zip(next))
            .map(|(member, (accumulated, next))| member.merge_step_outputs(accumulated, next))
            .collect()
    }

    /// The largest member norm, or `None` when any member opts out of
    /// error control.
    fn error_norm(
        &self,
        high: &Self::State,
        low: &Self::State,
        atol: f64,
        rtol: f64,
    ) -> Option<f64> {
        self.members.iter().zip(high.iter().zip(low)).try_fold(
            0.0_f64,
            |largest, (member, (high, low))| {
                member
                    .error_norm(high, low, atol, rtol)
                    .map(|norm| largest.max(norm))
            },
        )
    }

    fn noise_source_count(&self) -> usize {
        self.members
            .iter()
            .map(DynamicObject::noise_source_count)
            .sum()
    }

    /// Every member's diffusion columns, member by member, each zero for the
    /// other members.
    fn equations_of_motion_diffusion(
        &mut self,
        state: &Self::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> Vec<Self::Derivative> {
        self.assert_member_count(state.len(), "states");
        let member_count = self.members.len();
        let mut columns = Vec::new();
        for (index, (member, member_state)) in self.members.iter_mut().zip(state).enumerate() {
            for column in
                member.equations_of_motion_diffusion(member_state, current_sim_nanos, current_epoch)
            {
                let mut terms: Self::Derivative = (0..member_count).map(|_| None).collect();
                terms[index] = Some(column);
                columns.push(terms);
            }
        }
        columns
    }

    fn state_with_diffusion(
        &self,
        state: &Self::State,
        diffusion: &[DiffusionIncrement<'_, Self::Derivative>],
    ) -> Self::State {
        self.members
            .iter()
            .zip(state)
            .enumerate()
            .map(|(index, (member, member_state))| {
                member.state_with_diffusion(member_state, &member_increments(diffusion, index))
            })
            .collect()
    }

    fn combine_with_diffusion(
        &self,
        state: &Self::State,
        drift: Vec<(Self::Derivative, f64)>,
        step: f64,
        diffusion: &[DiffusionIncrement<'_, Self::Derivative>],
    ) -> (Self::State, Self::StepOutput) {
        self.assert_member_count(state.len(), "states");
        self.members
            .iter()
            .zip(state)
            .zip(per_member_weights(drift, self.members.len()))
            .enumerate()
            .map(|(index, ((member, member_state), terms))| {
                member.combine_with_diffusion(
                    member_state,
                    terms,
                    step,
                    &member_increments(diffusion, index),
                )
            })
            .unzip()
    }
}

#[cfg(test)]
mod tests;
