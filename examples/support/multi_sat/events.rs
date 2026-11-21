//! Upstream's simulation events (`SimulationBaseClass.createNewEvent`) as the
//! MultiSat flight software uses them: one group of events per spacecraft,
//! checked at every FSW tick, that switch its flight-software modes.
//!
//! | upstream | here |
//! |---|---|
//! | `createNewEvent(name, rate, True, conditionFunction, actionFunction)` | [`Events::add`] |
//! | `setEventActivity(name, flag)` | [`EventActivity::set`] |
//! | `setAllButCurrentEventActivity(name, flag, useIndex=True)` | [`EventActivity::set_all_but`] |
//! | the request strings `modeRequest`, `stationKeeping` | [`Request`] |
//!
//! Upstream checks the active events before every step of
//! `ExecuteSimulation`, in creation order, each with its activity as it is
//! when reached: a triggered event deactivates itself, then runs its action,
//! which may re-activate events checked later in the same pass. Upstream's
//! `useIndex` restricts `setAllButCurrentEventActivity` to the events of the
//! same spacecraft (the index after `_` in the name), so the events of one
//! spacecraft never touch another's; here each spacecraft has its own
//! [`Events`], and they are checked in spacecraft order.
//!
//! All MultiSat processes run at one rate, the events' rate, and the
//! conditions read only the requests, so checking at the start of every tick
//! (an [`Events`] module in a process ahead of all others) is upstream's
//! check after the previous tick.
//!
//! Marked for consolidation with `examples/support/bsk_sim/events.rs`, which
//! ports the same machinery for BskSim (with condition times and checks at
//! event rates slower than the tasks).

#![allow(dead_code)]

use std::sync::{Arc, Mutex};

use basilisk_rs::{Module, SimulationContext};

/// A request a scenario sets between runs and the events read: upstream's
/// `FSWModels[i].modeRequest` or `.stationKeeping`. Clones share the value.
#[derive(Debug)]
pub struct Request<T>(Arc<Mutex<T>>);

impl<T> Clone for Request<T> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<T: Copy + PartialEq> Request<T> {
    pub fn new(value: T) -> Self {
        Self(Arc::new(Mutex::new(value)))
    }

    /// Upstream `FSWModels[i].modeRequest = value`.
    pub fn set(&self, value: T) {
        *self
            .0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = value;
    }

    pub fn get(&self) -> T {
        *self
            .0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    pub fn is(&self, value: T) -> bool {
        self.get() == value
    }
}

/// Upstream `conditionFunction`.
pub type Condition = Box<dyn FnMut() -> bool + Send>;
/// Upstream `actionFunction`: gets the activities of the spacecraft's events.
pub type Action = Box<dyn FnMut(&mut EventActivity) + Send>;

/// The `eventActive` flags of one spacecraft's events, by name.
#[derive(Debug, Default)]
pub struct EventActivity {
    names: Vec<String>,
    active: Vec<bool>,
}

impl EventActivity {
    /// Upstream `setEventActivity(name, flag)`.
    pub fn set(&mut self, name: &str, active: bool) {
        match self.names.iter().position(|known| known == name) {
            Some(index) => self.active[index] = active,
            None => log::warn!("You asked me to set the status of an event that I don't have."),
        }
    }

    /// Upstream `setAllButCurrentEventActivity(current, flag, useIndex=True)`
    /// within this spacecraft's events.
    pub fn set_all_but(&mut self, current: &str, active: bool) {
        for (name, flag) in self.names.iter().zip(&mut self.active) {
            if name != current {
                *flag = active;
            }
        }
    }

    pub fn is_active(&self, name: &str) -> bool {
        self.names
            .iter()
            .position(|known| known == name)
            .is_some_and(|index| self.active[index])
    }
}

/// The events of one spacecraft, checked at every update.
#[derive(Default)]
pub struct Events {
    conditions: Vec<Condition>,
    actions: Vec<Action>,
    activity: EventActivity,
}

impl Events {
    /// Upstream `createNewEvent(name, rate, active, condition, action)`; a
    /// second event of the same name is skipped, as upstream.
    pub fn add(
        &mut self,
        name: impl Into<String>,
        active: bool,
        condition: impl FnMut() -> bool + Send + 'static,
        action: impl FnMut(&mut EventActivity) + Send + 'static,
    ) {
        let name = name.into();
        if self.activity.names.contains(&name) {
            log::warn!("Skipping event creation since {name} already exists.");
            return;
        }
        self.activity.names.push(name);
        self.activity.active.push(active);
        self.conditions.push(Box::new(condition));
        self.actions.push(Box::new(action));
    }

    pub fn activity(&self) -> &EventActivity {
        &self.activity
    }

    /// One pass over the events.
    pub fn check(&mut self) {
        for index in 0..self.conditions.len() {
            if self.activity.active[index] && (self.conditions[index])() {
                self.activity.active[index] = false;
                (self.actions[index])(&mut self.activity);
            }
        }
    }
}

impl Module for Events {
    fn init(&mut self) {}

    fn update(&mut self, _context: &SimulationContext) {
        self.check();
    }
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicUsize, Ordering};

    use super::*;

    /// A triggered event deactivates itself; re-activating a later event lets
    /// it trigger in the same pass, an earlier one only in the next.
    #[test]
    fn actions_change_activity_in_order() {
        let fired = Arc::new(AtomicUsize::new(0));
        let mut events = Events::default();
        events.add("early_0", false, || true, |_| {});
        events.add(
            "mode_0",
            true,
            || true,
            |activity| activity.set_all_but("mode_0", true),
        );
        let counter = Arc::clone(&fired);
        events.add(
            "late_0",
            false,
            || true,
            move |_| {
                counter.fetch_add(1, Ordering::Relaxed);
            },
        );
        events.check();
        assert_eq!(fired.load(Ordering::Relaxed), 1);
        assert!(!events.activity().is_active("mode_0"));
        assert!(!events.activity().is_active("late_0"));
        // "early_0" was re-activated behind the pass and fires in the next.
        assert!(events.activity().is_active("early_0"));
        events.check();
        assert!(!events.activity().is_active("early_0"));
    }

    #[test]
    fn a_request_is_shared() {
        let request = Request::new(1);
        let clone = request.clone();
        clone.set(2);
        assert!(request.is(2));
    }
}
