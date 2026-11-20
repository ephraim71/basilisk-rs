//! A module shared between the simulation, which updates it, and the
//! scenario, which changes it between `run_for` calls: the stand-in for
//! upstream scripts reaching a module (`viz.quadMaps`, ...) between
//! `ExecuteSimulation` calls.
//!
//! A [`Simulation`](basilisk_rs::simulation::Simulation) holds its modules
//! mutably borrowed while it exists. Schedule a [`Shared::handle`] instead,
//! and [`Shared::lock`] the module between runs.

#![allow(dead_code)]

use std::sync::{Arc, Mutex, MutexGuard};

use basilisk_rs::{Module, SimulationContext};

/// A module behind a shared lock. Handles share the module.
pub struct Shared<M>(Arc<Mutex<M>>);

impl<M> Shared<M> {
    pub fn new(module: M) -> Self {
        Self(Arc::new(Mutex::new(module)))
    }

    pub fn lock(&self) -> MutexGuard<'_, M> {
        self.0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    /// Another handle on the same module.
    pub fn handle(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<M: Module> Module for Shared<M> {
    fn init(&mut self) {
        self.lock().init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.lock().reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.lock().update(context);
    }
}
