//! Upstream task enabling for the scenarios: `enableTask`, `disableTask`
//! and `fswProcess.disableAllTasks()` between `ExecuteSimulation` calls, or in
//! the actions of `createNewEvent`.
//!
//! A [`TaskSwitch`] is a shared on/off flag, like a message
//! [`Output`](basilisk_rs::messages::Output) a slot the scenario keeps a clone
//! of while the simulation holds the modules. Each module of an upstream task
//! is scheduled wrapped in a [`Switched`] holding that task's switch: while the
//! switch is off the module skips its updates, as the modules of a disabled
//! task do. It keeps its schedule (upstream advances a disabled task's next
//! start time too) and is initialized with the others, as
//! `InitializeSimulation` resets every task.

#![allow(dead_code)]

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use basilisk_rs::{Module, SimulationContext};

/// Shared on/off state of one upstream task. Clones share the flag.
#[derive(Clone, Debug)]
pub struct TaskSwitch(Arc<AtomicBool>);

impl TaskSwitch {
    pub fn new(enabled: bool) -> Self {
        Self(Arc::new(AtomicBool::new(enabled)))
    }

    /// Upstream `enableTask(name)`.
    pub fn enable(&self) {
        self.0.store(true, Ordering::Relaxed);
    }

    /// Upstream `disableTask(name)`.
    pub fn disable(&self) {
        self.0.store(false, Ordering::Relaxed);
    }

    pub fn is_enabled(&self) -> bool {
        self.0.load(Ordering::Relaxed)
    }
}

/// A module of the task `switch` turns on and off.
#[derive(Debug)]
pub struct Switched<M> {
    pub module: M,
    switch: TaskSwitch,
}

impl<M> Switched<M> {
    pub fn new(module: M, switch: &TaskSwitch) -> Self {
        Self {
            module,
            switch: switch.clone(),
        }
    }
}

impl<M: Module> Module for Switched<M> {
    fn init(&mut self) {
        self.module.init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.module.reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        if self.switch.is_enabled() {
            self.module.update(context);
        }
    }
}
