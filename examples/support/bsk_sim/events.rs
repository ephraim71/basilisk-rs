//! Upstream's simulation events (`SimulationBaseClass.createNewEvent`) and
//! FSW tasks (`enableTask`, `fswProc.disableAllTasks()`), which BskSim uses to
//! switch flight-software modes and to inject faults.
//!
//! | upstream | here |
//! |---|---|
//! | `createNewEvent(name, eventRate, eventActive, conditionFunction=..., actionFunction=...)` | [`Events::add`] with [`Check::Every`] |
//! | `createNewEvent(..., conditionTime=t, ...)` | [`Events::add`] with [`Check::At`] |
//! | `setEventActivity(name, flag)` | [`EventActivity::set`] |
//! | `setAllButCurrentEventActivity(name, flag)` | [`EventActivity::set_all_but`] |
//! | `ConfigureStopTime(t)` + `ExecuteSimulation()` | [`Runner::execute`] |
//! | a task of the FSW process | a [`TaskSwitch`] |
//! | `AddModelToTask(task, module)` for each task listing a module | [`InTasks`] |
//!
//! A [`Simulation`] holds its modules mutably borrowed, so an action cannot
//! reach a module the way upstream's lambdas reach `self.FSWModels`. Actions
//! act through handles instead: [`TaskSwitch`]es, message
//! [`Output`](basilisk_rs::messages::Output)s (the FSW gateway messages) and
//! override targets of a [`Registry`](basilisk_rs::overrides::Registry) (the
//! faults), cloned into the closures when the events are built.
//!
//! # When events are checked
//!
//! Upstream's `ExecuteSimulation` loop checks the active events at the current
//! time `C` (the last task tick executed, or 0 before the first), then steps
//! every task tick up to the smallest next check time of an active event (at
//! least the next task tick, at most the stop time). So events are checked
//! once before tick 0, and afterwards between the end of a tick and the start
//! of the next one, at most once per tick, with `TotalSim.CurrentNanos = C`.
//! The checks of a run that starts where another stopped come right at the
//! start: at the stop time of the previous run.
//!
//! [`Events`] is a module that reproduces that loop. Scheduled first in every
//! tick (the highest priority, at the rate of the fastest task), at tick `t`
//! it runs the check upstream runs after the previous tick `C`, when upstream
//! would have stopped there, and gives the conditions and actions the time
//! `C`. Nothing runs between the end of tick `C` and the start of tick `t`,
//! so the messages an action reads, and what it changes, are what upstream's
//! action reads and changes. [`Runner::execute`] tells it the stop time, which
//! upstream's loop also stops at.
//!
//! Within one check the events are visited in the order they were added, each
//! with its activity as it is when it is reached, which is what upstream's
//! active-event cache resumes to after an action changes activities: an
//! event re-activated after the current one in the list is checked in the
//! same pass, one before it in the next. A triggered event deactivates itself
//! before its action runs.

#![allow(dead_code)]

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};

use basilisk_rs::simulation::Simulation;
use basilisk_rs::{Module, SimulationContext};

pub use super::task_switch::TaskSwitch;

/// When an event is checked.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Check {
    /// Upstream `eventRate` with the default `exactRateMatch`: at every check
    /// whose time is a multiple of the rate \[ns\].
    Every(u64),
    /// Upstream `conditionTime` \[ns\]: at every check at or after the time,
    /// with the condition "now >= time".
    At(u64),
}

impl Check {
    fn due(self, now_nanos: u64) -> bool {
        match self {
            Self::Every(rate) => now_nanos.is_multiple_of(rate),
            Self::At(time) => now_nanos >= time,
        }
    }

    /// Upstream `nextCheckTime`.
    fn next(self, now_nanos: u64) -> u64 {
        match self {
            Self::Every(rate) => (now_nanos / rate + 1) * rate,
            Self::At(time) => time,
        }
    }
}

/// Upstream `conditionFunction`: gets `TotalSim.CurrentNanos`.
pub type Condition = Box<dyn FnMut(u64) -> bool + Send>;
/// Upstream `actionFunction`: gets `TotalSim.CurrentNanos` and the event
/// activities.
pub type Action = Box<dyn FnMut(u64, &mut EventActivity) + Send>;

struct Hooks {
    check: Check,
    condition: Condition,
    action: Action,
}

/// The `eventActive` flags of every event, by name (upstream `eventMap`).
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
            None => println!("You asked me to set the status of an event that I don't have."),
        }
    }

    /// Upstream `setAllButCurrentEventActivity(current, flag)`.
    pub fn set_all_but(&mut self, current: &str, active: bool) {
        for (name, flag) in self.names.iter().zip(&mut self.active) {
            if name != current {
                *flag = active;
            }
        }
    }

    /// Upstream `eventMap[name].eventActive`.
    pub fn is_active(&self, name: &str) -> bool {
        self.names
            .iter()
            .position(|known| known == name)
            .is_some_and(|index| self.active[index])
    }
}

/// Upstream's event map and its checks in `ExecuteSimulation`.
pub struct Events {
    period_nanos: u64,
    hooks: Vec<Hooks>,
    activity: EventActivity,
    stop_nanos: Arc<AtomicU64>,
    previous_tick_nanos: Option<u64>,
    next_stop_nanos: u64,
}

impl Events {
    /// Events of a simulation whose fastest task runs every `period_nanos`.
    pub fn new(period_nanos: u64) -> Self {
        Self {
            period_nanos,
            hooks: Vec::new(),
            activity: EventActivity::default(),
            stop_nanos: Arc::new(AtomicU64::new(u64::MAX)),
            previous_tick_nanos: None,
            next_stop_nanos: 0,
        }
    }

    /// Upstream `createNewEvent(name, rate, active, condition, action)`. As
    /// upstream, a second event of the same name is skipped.
    pub fn add(
        &mut self,
        name: impl Into<String>,
        check: Check,
        active: bool,
        condition: impl FnMut(u64) -> bool + Send + 'static,
        action: impl FnMut(u64, &mut EventActivity) + Send + 'static,
    ) {
        let name = name.into();
        if self.activity.names.contains(&name) {
            eprintln!("Skipping event creation since {name} already exists.");
            return;
        }
        self.activity.names.push(name);
        self.activity.active.push(active);
        self.hooks.push(Hooks {
            check,
            condition: Box::new(condition),
            action: Box::new(action),
        });
    }

    /// The handle that runs the simulation to a stop time.
    pub fn runner(&self) -> Runner {
        Runner {
            stop_nanos: Arc::clone(&self.stop_nanos),
        }
    }

    /// The rate the module checks at \[ns\].
    pub fn period_nanos(&self) -> u64 {
        self.period_nanos
    }

    /// The event activities, as upstream's `eventMap` reports them.
    pub fn activity(&self) -> &EventActivity {
        &self.activity
    }

    /// One pass over the events at `now_nanos`.
    fn check_events(&mut self, now_nanos: u64) {
        for index in 0..self.hooks.len() {
            if !self.activity.active[index] || !self.hooks[index].check.due(now_nanos) {
                continue;
            }
            if (self.hooks[index].condition)(now_nanos) {
                self.activity.active[index] = false;
                (self.hooks[index].action)(now_nanos, &mut self.activity);
            }
        }
    }

    /// Upstream's next stop: the earliest next check of an active event, at
    /// least the next task tick, at most the stop time.
    fn next_stop(&self, now_nanos: u64, next_tick_nanos: u64) -> u64 {
        let earliest = self
            .hooks
            .iter()
            .zip(&self.activity.active)
            .filter(|(_, active)| **active)
            .map(|(hooks, _)| hooks.check.next(now_nanos))
            .min()
            .map_or(u64::MAX, |next| next.max(next_tick_nanos));
        earliest.min(self.stop_nanos.load(Ordering::Relaxed))
    }
}

impl Module for Events {
    fn init(&mut self) {
        self.previous_tick_nanos = None;
        self.next_stop_nanos = 0;
    }

    fn update(&mut self, context: &SimulationContext) {
        let tick = context.current_sim_nanos;
        let check_at = match self.previous_tick_nanos {
            // Upstream checks once before the first step.
            None => Some(tick),
            Some(previous) if tick > self.next_stop_nanos => Some(previous),
            Some(_) => None,
        };
        if let Some(now) = check_at {
            self.check_events(now);
            self.next_stop_nanos = self.next_stop(now, tick);
        }
        self.previous_tick_nanos = Some(tick);
    }
}

/// Upstream `ConfigureStopTime(t)` followed by `ExecuteSimulation()`.
#[derive(Clone, Debug)]
pub struct Runner {
    stop_nanos: Arc<AtomicU64>,
}

impl Runner {
    /// Runs `simulation` to the absolute time `stop_nanos`: every task tick
    /// up to and including it.
    pub fn execute(&self, simulation: &mut Simulation<'_>, stop_nanos: u64) {
        self.stop_nanos.store(stop_nanos, Ordering::Relaxed);
        let now = simulation.current_sim_nanos();
        // Before the first run the clock reads 0 and tick 0 is still to come.
        simulation.run_for(stop_nanos.saturating_sub(now));
    }
}

/// A module that runs while any task listing it is enabled: upstream
/// `AddModelToTask(task, module)` for each of `tasks`. Upstream runs such a
/// module once per enabled task; the BskSim modes never enable two tasks of
/// one module together, so it runs at most once per tick there too.
pub struct InTasks<M> {
    pub module: M,
    tasks: Vec<TaskSwitch>,
}

impl<M> InTasks<M> {
    pub fn new(module: M, tasks: &[&TaskSwitch]) -> Self {
        Self {
            module,
            tasks: tasks.iter().map(|task| (*task).clone()).collect(),
        }
    }

    /// Whether a task listing the module is enabled.
    pub fn is_enabled(&self) -> bool {
        self.tasks.iter().any(TaskSwitch::is_enabled)
    }
}

impl<M: Module> Module for InTasks<M> {
    fn init(&mut self) {
        self.module.init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.module.reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        if self.is_enabled() {
            self.module.update(context);
        }
    }
}

/// The requested flight-software mode: upstream's `scenario.modeRequest`,
/// which the mode events compare against. Clones share the request.
#[derive(Debug)]
pub struct ModeRequest<M>(Arc<std::sync::Mutex<M>>);

impl<M> Clone for ModeRequest<M> {
    fn clone(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<M: Copy + PartialEq> ModeRequest<M> {
    pub fn new(mode: M) -> Self {
        Self(Arc::new(std::sync::Mutex::new(mode)))
    }

    /// Upstream `scenario.modeRequest = mode`.
    pub fn set(&self, mode: M) {
        *self
            .0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner()) = mode;
    }

    pub fn get(&self) -> M {
        *self
            .0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    pub fn is(&self, mode: M) -> bool {
        self.get() == mode
    }
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use super::*;

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: hifitime::Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream checks before tick 0 and then after every tick but tick 0.
    #[test]
    fn events_are_checked_where_upstream_checks_them() {
        let seen = Arc::new(Mutex::new(Vec::new()));
        let mut events = Events::new(10);
        let log = Arc::clone(&seen);
        events.add(
            "always",
            Check::Every(10),
            true,
            move |now| {
                log.lock().unwrap().push(now);
                false
            },
            |_, _| {},
        );
        events.init();
        for tick in [0, 10, 20, 30] {
            events.update(&context(tick));
        }
        assert_eq!(*seen.lock().unwrap(), vec![0, 10, 20]);
    }

    /// A triggered event deactivates itself; an action re-activating the
    /// others lets a later event see the change in the same pass.
    #[test]
    fn actions_change_activity_in_order() {
        let mut events = Events::new(10);
        events.add(
            "first",
            Check::Every(10),
            true,
            |_| true,
            |_, activity| activity.set_all_but("first", true),
        );
        events.add("second", Check::Every(10), false, |_| true, |_, _| {});
        events.init();
        events.update(&context(0));
        assert!(!events.activity().is_active("first"));
        // "second" was re-activated by "first" and then triggered itself.
        assert!(!events.activity().is_active("second"));
    }

    /// A condition time triggers at the first check at or after it.
    #[test]
    fn a_condition_time_triggers_once() {
        let fired = Arc::new(Mutex::new(Vec::new()));
        let mut events = Events::new(10);
        let log = Arc::clone(&fired);
        events.add(
            "late",
            Check::At(25),
            true,
            |_| true,
            move |now, _| log.lock().unwrap().push(now),
        );
        events.init();
        for tick in (0..=60).step_by(10) {
            events.update(&context(tick));
        }
        assert_eq!(*fired.lock().unwrap(), vec![30]);
    }
}
