use hifitime::{Duration, Epoch};
use indicatif::{ProgressBar, ProgressStyle};
use rayon::prelude::*;
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

    pub fn connect<T>(&self, output: &Output<T>, input: &mut Input<T>) {
        input.connect(output.slot());
    }

    pub fn initialize(&mut self) {
        if self.initialized {
            return;
        }

        self.modules.sort_by_key(|scheduled| (scheduled.priority, scheduled.insertion_order));

        for scheduled in &mut self.modules {
            scheduled.module.init();
            scheduled.next_run_nanos = 0;
        }

        self.initialized = true;
    }

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

        while self.current_sim_nanos <= stop_nanos {
            let context = self.context();
            let current = self.current_sim_nanos;
            // A module fires if it is scheduled at this tick, OR if this is the final
            // stop time and the module is mid-period (partial final step, matches Basilisk
            // behaviour when stop time is not a multiple of the task rate).
            let is_final_tick = current == stop_nanos;
            let should_fire = move |s: &&mut ScheduledModule| {
                s.next_run_nanos == current
                    || (is_final_tick
                        && s.next_run_nanos > current
                        && s.next_run_nanos - s.period_nanos < current)
            };

            let mut group_start = 0;
            while group_start < self.modules.len() {
                let priority = self.modules[group_start].priority;
                let mut group_end = group_start + 1;
                while group_end < self.modules.len() && self.modules[group_end].priority == priority {
                    group_end += 1;
                }

                let group = &mut self.modules[group_start..group_end];
                if self.collect_timings {
                    group
                        .par_iter_mut()
                        .filter(should_fire)
                        .for_each(|scheduled| {
                            let started_at = Instant::now();
                            scheduled.module.update(&context);
                            scheduled.total_update_nanos += started_at.elapsed().as_nanos();
                            scheduled.num_updates += 1;
                            scheduled.next_run_nanos += scheduled.period_nanos;
                        });
                } else {
                    group
                        .par_iter_mut()
                        .filter(should_fire)
                        .for_each(|scheduled| {
                            scheduled.module.update(&context);
                            scheduled.num_updates += 1;
                            scheduled.next_run_nanos += scheduled.period_nanos;
                        });
                }

                group_start = group_end;
            }

            if self.current_sim_nanos == stop_nanos {
                if let Some(progress_bar) = &progress_bar {
                    progress_bar.set_position(duration_nanos);
                }
                break;
            }

            let next_nanos = self
                .modules
                .iter()
                .map(|scheduled| scheduled.next_run_nanos)
                .min()
                .expect("simulation has no modules");
            if let Some(progress_bar) = &progress_bar {
                progress_bar.set_position(next_nanos.min(stop_nanos) - start_nanos);
            }

            // Cap to stop_nanos so the final partial step lands exactly at the target time.
            self.current_sim_nanos = next_nanos.min(stop_nanos);
        }

        if let Some(progress_bar) = progress_bar {
            progress_bar.finish_with_message("simulation complete");
        }
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
            start_epoch: self.start_epoch,
            current_sim_nanos: self.current_sim_nanos,
            current_epoch: self.current_epoch(),
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
