//! Module changes upstream scenarios make between `ExecuteSimulation` calls.
//!
//! Upstream scripts stop the simulation, change a module, and continue:
//!
//! ```python
//! scSim.ConfigureStopTime(T1); scSim.ExecuteSimulation()
//! locPoint.pHat_B = [0, 0, -1]          # set a module variable
//! thrDesatControl.Reset(T1)             # or call its Reset
//! scSim.ConfigureStopTime(T2); scSim.ExecuteSimulation()
//! ```
//!
//! A [`Simulation`](basilisk_rs::simulation::Simulation) holds its modules
//! mutably borrowed while it exists, so a scenario cannot reach them between
//! [`run_for`](basilisk_rs::simulation::Simulation::run_for) calls. Instead the
//! module is scheduled wrapped in a [`Staged`] holding the change and the stop
//! time it follows. The change is applied at the module's first update after
//! that time, which is the first update upstream's changed module runs.

#![allow(dead_code)]

use basilisk_rs::{Module, SimulationContext};

type Change<M> = Box<dyn FnOnce(&mut M, &SimulationContext) + Send>;

/// A module with changes staged for later stop times.
pub struct Staged<M> {
    pub module: M,
    pending: Vec<(u64, Change<M>)>,
    held_until_nanos: Option<u64>,
}

impl<M> Staged<M> {
    pub fn new(module: M) -> Self {
        Self {
            module,
            pending: Vec::new(),
            held_until_nanos: None,
        }
    }

    /// Skips the module's updates up to and including `stop_nanos`, as if
    /// its task were disabled until then; it is still initialized.
    pub fn held_until(mut self, stop_nanos: u64) -> Self {
        self.held_until_nanos = Some(stop_nanos);
        self
    }

    /// Applies `change` before the first update after `stop_nanos`; the
    /// change gets the context of that update.
    pub fn after(
        mut self,
        stop_nanos: u64,
        change: impl FnOnce(&mut M, &SimulationContext) + Send + 'static,
    ) -> Self {
        self.pending.push((stop_nanos, Box::new(change)));
        self
    }
}

impl<M: Module> Staged<M> {
    /// Stages upstream's `module.Reset(stop_nanos)` between runs.
    pub fn reset_after(self, stop_nanos: u64) -> Self {
        self.after(stop_nanos, |module: &mut M, context| module.reset(context))
    }
}

impl<M: Module> Module for Staged<M> {
    fn init(&mut self) {
        self.module.init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.module.reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        if self
            .held_until_nanos
            .is_some_and(|stop| context.current_sim_nanos <= stop)
        {
            return;
        }
        let mut index = 0;
        while index < self.pending.len() {
            if context.current_sim_nanos > self.pending[index].0 {
                let (_, change) = self.pending.remove(index);
                change(&mut self.module, context);
            } else {
                index += 1;
            }
        }
        self.module.update(context);
    }
}

/// A module that also updates once at initialization, at the start epoch.
///
/// Upstream `spiceInterface.Reset` ends with an `UpdateState`, so the planet
/// states are written when the other modules reset (the spacecraft evaluates
/// gravity there). basilisk-rs's ephemeris modules write a zero message at
/// `init`, which places every body at the origin; scheduled wrapped in an
/// `UpdatedAtInit`, they publish the start-epoch states instead.
pub struct UpdatedAtInit<M> {
    pub module: M,
    start: SimulationContext,
}

impl<M> UpdatedAtInit<M> {
    pub fn new(module: M, start_epoch: hifitime::Epoch) -> Self {
        Self {
            module,
            start: SimulationContext {
                current_sim_nanos: 0,
                current_epoch: start_epoch,
            },
        }
    }
}

impl<M: Module> Module for UpdatedAtInit<M> {
    fn init(&mut self) {
        self.module.init();
        self.module.update(&self.start);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.module.reset(context);
        self.module.update(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.module.update(context);
    }
}
