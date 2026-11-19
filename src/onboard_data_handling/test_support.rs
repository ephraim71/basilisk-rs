//! Test scaffolding shared by the data-handling and communication tests.

use hifitime::Epoch;

use crate::SimulationContext;

/// A context at `current_sim_nanos`.
pub(crate) fn context(current_sim_nanos: u64) -> SimulationContext {
    SimulationContext {
        current_sim_nanos,
        current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
    }
}

/// Upstream's single-task scheduler, for tests that step modules by hand so
/// they can change them between runs: one tick every `step_nanos` from zero,
/// and each `ConfigureStopTime` + `ExecuteSimulation` runs the ticks up to
/// and including the stop time.
pub(crate) struct TaskClock {
    step_nanos: u64,
    next_nanos: u64,
}

impl TaskClock {
    pub(crate) fn new(step_nanos: u64) -> Self {
        Self {
            step_nanos,
            next_nanos: 0,
        }
    }

    /// The ticks not yet run, up to and including `stop_nanos`.
    pub(crate) fn run_until(&mut self, stop_nanos: u64) -> Vec<u64> {
        let mut ticks = Vec::new();
        while self.next_nanos <= stop_nanos {
            ticks.push(self.next_nanos);
            self.next_nanos += self.step_nanos;
        }
        ticks
    }
}
