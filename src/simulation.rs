mod macros;

use hifitime::{Duration, Epoch};
use indicatif::{ProgressBar, ProgressStyle};
use std::cmp::Reverse;
use std::time::Instant;

use crate::messages::{Input, Output};
use crate::{Module, SimulationContext};

struct ScheduledModule<'a> {
    name: String,
    priority: i32,
    period_nanos: u64,
    next_run_nanos: u64,
    insertion_order: usize,
    num_updates: u64,
    total_update_nanos: u128,
    module: &'a mut dyn Module,
}

#[derive(Clone, Debug)]
pub struct ModuleTiming {
    pub name: String,
    pub priority: i32,
    pub num_updates: u64,
    pub total_update_nanos: u128,
}

pub struct Simulation<'a> {
    start_epoch: Epoch,
    current_sim_nanos: u64,
    initialized: bool,
    show_progress: bool,
    collect_timings: bool,
    modules: Vec<ScheduledModule<'a>>,
    next_insertion_order: usize,
}

impl<'a> Simulation<'a> {
    pub fn new(start_epoch: Epoch, show_progress: bool) -> Self {
        Self {
            start_epoch,
            current_sim_nanos: 0,
            initialized: false,
            show_progress,
            collect_timings: false,
            modules: Vec::new(),
            next_insertion_order: 0,
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.collect_timings = enabled;
    }

    /// Schedule a module. Higher numeric priorities execute first; equal
    /// priorities retain insertion order.
    pub fn add_module(
        &mut self,
        name: impl Into<String>,
        module: &'a mut dyn Module,
        period_nanos: u64,
        priority: i32,
    ) {
        self.modules.push(ScheduledModule {
            name: name.into(),
            priority,
            period_nanos,
            next_run_nanos: 0,
            insertion_order: self.next_insertion_order,
            num_updates: 0,
            total_update_nanos: 0,
            module,
        });
        self.next_insertion_order += 1;
    }

    pub fn connect<T: Clone>(&self, output: &Output<T>, input: &mut Input<T>) {
        input.connect(output.slot());
    }

    pub fn initialize(&mut self) {
        if self.initialized {
            return;
        }

        self.modules
            .sort_by_key(|scheduled| (Reverse(scheduled.priority), scheduled.insertion_order));

        for scheduled in &mut self.modules {
            scheduled.module.init();
            scheduled.next_run_nanos = 0;
        }

        self.initialized = true;
    }

    /// Runs every module update scheduled at or before the requested stop time.
    ///
    /// A stop time between task ticks does not cause an unscheduled partial
    /// update; the simulation time remains at the most recent executed task
    /// tick.
    pub fn run_for(&mut self, duration_nanos: u64) {
        self.initialize();

        let start_nanos = self.current_sim_nanos;
        let stop_nanos = self.current_sim_nanos + duration_nanos;
        let progress_bar = self.show_progress.then(|| {
            let progress_bar = ProgressBar::new(duration_nanos);
            progress_bar.set_style(
                ProgressStyle::with_template(
                    "[{elapsed_precise}] {bar:40.cyan/blue} {percent:>3}% {msg}",
                )
                .expect("valid progress template")
                .progress_chars("##-"),
            );
            progress_bar.set_message("simulation");
            progress_bar
        });

        loop {
            let context = self.context();
            let current = self.current_sim_nanos;
            // A task executes only at its scheduled tick; stopping between task
            // ticks does not synthesize a partial final update.
            for group in self.modules.chunk_by_mut(|a, b| a.priority == b.priority) {
                group
                    .iter_mut()
                    .filter(|scheduled| scheduled.next_run_nanos == current)
                    .for_each(|scheduled| {
                        let started_at = Instant::now();
                        scheduled.module.update(&context);
                        if self.collect_timings {
                            scheduled.total_update_nanos += started_at.elapsed().as_nanos();
                        }
                        scheduled.num_updates += 1;
                        scheduled.next_run_nanos += scheduled.period_nanos;
                    });
            }

            let next_nanos = self
                .modules
                .iter()
                .map(|scheduled| scheduled.next_run_nanos)
                .min()
                .expect("simulation has no modules");
            if next_nanos > stop_nanos {
                if let Some(progress_bar) = &progress_bar {
                    progress_bar.set_position(duration_nanos);
                }
                break;
            }

            if let Some(progress_bar) = &progress_bar {
                progress_bar.set_position(next_nanos - start_nanos);
            }
            self.current_sim_nanos = next_nanos;
        }

        if let Some(progress_bar) = progress_bar {
            progress_bar.finish_with_message("simulation complete");
        }
    }

    pub fn run_for_chunked_while(
        &mut self,
        duration_nanos: u64,
        chunk_nanos: u64,
        mut should_continue: impl FnMut() -> bool,
    ) -> bool {
        let chunk_nanos = chunk_nanos.max(1).min(duration_nanos.max(1));
        let mut remaining_nanos = duration_nanos;
        while remaining_nanos > 0 && should_continue() {
            let run_nanos = remaining_nanos.min(chunk_nanos);
            self.run_for(run_nanos);
            remaining_nanos = remaining_nanos.saturating_sub(run_nanos);
        }
        remaining_nanos == 0
    }

    pub fn current_sim_nanos(&self) -> u64 {
        self.current_sim_nanos
    }

    pub fn current_epoch(&self) -> Epoch {
        self.start_epoch + Duration::from_total_nanoseconds(self.current_sim_nanos as i128)
    }

    pub fn start_epoch(&self) -> Epoch {
        self.start_epoch
    }

    pub fn context(&self) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: self.current_sim_nanos,
            current_epoch: self.current_epoch(),
        }
    }

    pub fn reset(&mut self) {
        self.current_sim_nanos = 0;
        for m in &mut self.modules {
            m.next_run_nanos = 0;
        }
        if self.initialized {
            let context = SimulationContext {
                current_sim_nanos: 0,
                current_epoch: self.start_epoch,
            };
            for scheduled in &mut self.modules {
                scheduled.module.reset(&context);
            }
        }
    }

    pub fn module_names(&self) -> Vec<String> {
        self.modules
            .iter()
            .map(|scheduled| scheduled.name.clone())
            .collect()
    }

    pub fn module_timings(&self) -> Vec<ModuleTiming> {
        let mut timings: Vec<_> = self
            .modules
            .iter()
            .map(|scheduled| ModuleTiming {
                name: scheduled.name.clone(),
                priority: scheduled.priority,
                num_updates: scheduled.num_updates,
                total_update_nanos: scheduled.total_update_nanos,
            })
            .collect();
        timings.sort_by(|lhs, rhs| rhs.total_update_nanos.cmp(&lhs.total_update_nanos));
        timings
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use crate::messages::{Input, Output};
    use crate::{Module, SimulationContext};

    use super::Simulation;

    #[derive(Default)]
    struct LifecycleProbe {
        init_count: usize,
        reset_count: usize,
        update_count: usize,
    }

    struct Producer {
        output: Output<u32>,
    }

    impl Module for Producer {
        fn init(&mut self) {
            self.output.write(0);
        }

        fn update(&mut self, _context: &SimulationContext) {
            self.output.write(1);
        }
    }

    #[derive(Default)]
    struct Consumer {
        input: Input<u32>,
        observed: u32,
    }

    #[derive(Default)]
    struct UpdateTimes {
        times_nanos: Vec<u64>,
    }

    impl Module for UpdateTimes {
        fn init(&mut self) {}

        fn update(&mut self, context: &SimulationContext) {
            self.times_nanos.push(context.current_sim_nanos);
        }
    }

    impl Module for Consumer {
        fn init(&mut self) {}

        fn update(&mut self, _context: &SimulationContext) {
            self.observed = self.input.read();
        }
    }

    impl Module for LifecycleProbe {
        fn init(&mut self) {
            self.init_count += 1;
        }

        fn reset(&mut self, _context: &SimulationContext) {
            self.reset_count += 1;
        }

        fn update(&mut self, _context: &SimulationContext) {
            self.update_count += 1;
        }
    }

    #[test]
    fn reset_uses_reset_hook_without_repeating_initialization() {
        let mut probe = LifecycleProbe::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.add_module("probe", &mut probe, 1_000_000_000, 0);
        simulation.run_for(0);
        simulation.reset();
        simulation.run_for(0);
        drop(simulation);

        assert_eq!(probe.init_count, 1);
        assert_eq!(probe.reset_count, 1);
        assert_eq!(probe.update_count, 2);
    }

    #[test]
    fn higher_numeric_priority_executes_first_like_upstream() {
        let mut producer = Producer {
            output: Output::default(),
        };
        let mut consumer = Consumer::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&producer.output, &mut consumer.input);
        simulation.add_module("consumer", &mut consumer, 1_000_000_000, 0);
        simulation.add_module("producer", &mut producer, 1_000_000_000, 10);
        simulation.run_for(0);
        drop(simulation);

        assert_eq!(consumer.observed, 1);
    }

    #[test]
    fn equal_priorities_retain_insertion_order() {
        let mut producer = Producer {
            output: Output::default(),
        };
        let mut consumer = Consumer::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&producer.output, &mut consumer.input);
        simulation.add_module("producer", &mut producer, 1_000_000_000, 0);
        simulation.add_module("consumer", &mut consumer, 1_000_000_000, 0);
        simulation.run_for(0);
        drop(simulation);

        assert_eq!(consumer.observed, 1);
    }

    #[test]
    fn stop_between_task_ticks_does_not_execute_a_partial_update() {
        let mut updates = UpdateTimes::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.add_module("updates", &mut updates, 10_000_000_000, 0);
        simulation.run_for(25_000_000_000);

        assert_eq!(simulation.current_sim_nanos(), 20_000_000_000);
        drop(simulation);
        assert_eq!(updates.times_nanos, vec![0, 10_000_000_000, 20_000_000_000]);
    }
}
