use std::thread;
use std::time::{Duration, Instant};

use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct ClockSync {
    accel_factor: f64,
    accuracy_nanos: u64,
    display_time: bool,
    initialized: bool,
    start_wall: Instant,
    start_sim_nanos: u64,
    overrun_counter: u64,
    last_display_second: u64,
}

impl ClockSync {
    pub fn new(accel_factor: f64, accuracy_nanos: u64, display_time: bool) -> Self {
        Self {
            accel_factor: accel_factor.max(1.0e-9),
            accuracy_nanos,
            display_time,
            initialized: false,
            start_wall: Instant::now(),
            start_sim_nanos: 0,
            overrun_counter: 0,
            last_display_second: u64::MAX,
        }
    }

    fn reset_clock(&mut self, current_sim_nanos: u64) {
        self.initialized = true;
        self.start_wall = Instant::now();
        self.start_sim_nanos = current_sim_nanos;
        self.overrun_counter = 0;
        self.last_display_second = u64::MAX;
    }
}

impl Module for ClockSync {
    fn init(&mut self) {
        self.initialized = false;
        self.overrun_counter = 0;
    }

    fn update(&mut self, context: &SimulationContext) {
        if !self.initialized {
            self.reset_clock(context.current_sim_nanos);
            return;
        }

        let sim_elapsed_nanos = (context
            .current_sim_nanos
            .saturating_sub(self.start_sim_nanos)) as f64
            / self.accel_factor;
        let mut real_elapsed_nanos = self.start_wall.elapsed().as_nanos() as f64;
        let accuracy_nanos = self.accuracy_nanos as f64;

        if sim_elapsed_nanos - real_elapsed_nanos < -accuracy_nanos {
            self.overrun_counter = self.overrun_counter.saturating_add(1);
        }

        while real_elapsed_nanos - sim_elapsed_nanos < -accuracy_nanos {
            let remaining_nanos = ((sim_elapsed_nanos - real_elapsed_nanos) / 2.0).max(1.0);
            let sleep_nanos = remaining_nanos.min(50_000_000.0) as u64;
            thread::sleep(Duration::from_nanos(sleep_nanos));
            real_elapsed_nanos = self.start_wall.elapsed().as_nanos() as f64;
        }

        if self.display_time {
            let second = context.current_sim_nanos / 1_000_000_000;
            if second != self.last_display_second {
                println!(
                    "[clock_sync] sim_elapsed={:.3}s overruns={}",
                    context.current_sim_nanos as f64 * 1.0e-9,
                    self.overrun_counter
                );
                self.last_display_second = second;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use super::ClockSync;
    use crate::{Module, SimulationContext};

    #[test]
    fn first_update_uses_the_current_simulation_time_as_its_baseline() {
        let mut clock = ClockSync::new(1.0, 0, false);
        let context = SimulationContext {
            current_sim_nanos: 10_000_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };

        clock.update(&context);

        assert!(clock.initialized);
        assert_eq!(clock.start_sim_nanos, context.current_sim_nanos);
        assert_eq!(clock.overrun_counter, 0);
    }

    #[test]
    fn init_clears_runtime_state() {
        let mut clock = ClockSync::new(1.0, 0, false);
        clock.initialized = true;
        clock.overrun_counter = 3;

        clock.init();

        assert!(!clock.initialized);
        assert_eq!(clock.overrun_counter, 0);
    }

    #[test]
    fn nonpositive_acceleration_is_clamped_to_a_positive_value() {
        let clock = ClockSync::new(0.0, 0, false);

        assert_eq!(clock.accel_factor, 1.0e-9);
    }
}
