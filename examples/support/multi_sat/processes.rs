//! Upstream's simulation processes, run in lockstep, serially or on threads.
//!
//! | upstream | here |
//! |---|---|
//! | `CreateNewProcess(name, priority)` | [`Processes::create_process`]: a [`Simulation`] of its own |
//! | `TotalSim.resetThreads(n)` | [`Processes::reset_threads`] |
//! | `InitializeSimulation`, `ConfigureStopTime(t)` + `ExecuteSimulation()` | [`Processes::execute_to`] |
//!
//! Upstream runs, at every tick, its processes by priority (equal priorities
//! in creation order), each process its tasks by priority. Here each process
//! is a separate [`Simulation`], and [`Processes::execute_to`] advances them
//! one base step at a time in that order, so a process sees what the
//! processes before it wrote in the same tick, as upstream. Every process is
//! initialized before any of them runs its first tick, as
//! `InitializeSimulation` initializes everything before `ExecuteSimulation`.
//!
//! # Threads
//!
//! Upstream's multi-threading (`resetThreads`) spreads the processes over
//! threads and runs them concurrently, with the warning that its messaging
//! is not thread-safe. basilisk-rs's message ports are (`Arc<RwLock>`), and
//! here a group of processes of *equal* priority runs in parallel on a
//! [`rayon`] pool, the groups still one after the other. Equal-priority
//! processes must not read what the others of the group write in the same
//! tick (upstream's processes of one spacecraft read the environment and
//! their own spacecraft only); then the parallel run gives bit-for-bit the
//! serial one's results.
//!
//! Every process gets a no-op module at the base step, so its clock follows
//! the others' even when its own modules run less often.

#![allow(dead_code)]

use basilisk_rs::simulation::Simulation;
use basilisk_rs::{Module, SimulationContext};
use hifitime::Epoch;
use rayon::prelude::*;

/// A no-op module at the base step that keeps a process's clock with the
/// others.
struct Clock;

impl Module for Clock {
    fn init(&mut self) {}

    fn update(&mut self, _context: &SimulationContext) {}
}

/// One upstream process.
pub struct Process<'a> {
    pub name: String,
    /// Upstream process priority: higher runs first.
    pub priority: i32,
    pub simulation: Simulation<'a>,
    creation_order: usize,
}

/// Upstream's processes of one simulation (`TotalSim`).
pub struct Processes<'a> {
    start_epoch: Epoch,
    step_nanos: u64,
    processes: Vec<Process<'a>>,
    threads: usize,
    pool: Option<rayon::ThreadPool>,
    current_nanos: Option<u64>,
}

impl<'a> Processes<'a> {
    /// Processes starting at `start_epoch` that all run at multiples of
    /// `step_nanos`.
    pub fn new(start_epoch: Epoch, step_nanos: u64) -> Self {
        assert!(step_nanos > 0, "the base step must be positive");
        Self {
            start_epoch,
            step_nanos,
            processes: Vec::new(),
            threads: 1,
            pool: None,
            current_nanos: None,
        }
    }

    /// Upstream `CreateNewProcess(name, priority)`.
    pub fn create_process(
        &mut self,
        name: impl Into<String>,
        priority: i32,
    ) -> &mut Simulation<'a> {
        let simulation = Simulation::new(self.start_epoch, false);
        self.add_process(name, priority, simulation)
    }

    /// A process of the modules already scheduled in `simulation` (started
    /// at this simulation's epoch).
    pub fn add_process(
        &mut self,
        name: impl Into<String>,
        priority: i32,
        mut simulation: Simulation<'a>,
    ) -> &mut Simulation<'a> {
        assert!(
            self.current_nanos.is_none(),
            "processes are created before the simulation runs"
        );
        assert_eq!(
            simulation.start_epoch(),
            self.start_epoch,
            "every process starts at the same epoch"
        );
        // A zero-sized module: leaking it allocates nothing.
        simulation.add_module(
            "clock",
            Box::leak(Box::new(Clock)),
            self.step_nanos,
            i32::MIN,
        );
        let creation_order = self.processes.len();
        self.processes.push(Process {
            name: name.into(),
            priority,
            simulation,
            creation_order,
        });
        &mut self.processes.last_mut().expect("just created").simulation
    }

    /// The simulation start epoch.
    pub fn start_epoch(&self) -> Epoch {
        self.start_epoch
    }

    /// The process named `name`.
    ///
    /// # Panics
    ///
    /// If there is none.
    pub fn process(&mut self, name: &str) -> &mut Simulation<'a> {
        &mut self
            .processes
            .iter_mut()
            .find(|process| process.name == name)
            .unwrap_or_else(|| panic!("no process named {name}"))
            .simulation
    }

    /// Upstream `TotalSim.resetThreads(threads)`: the number of threads the
    /// equal-priority processes are spread over (1 runs everything on the
    /// calling thread).
    pub fn reset_threads(&mut self, threads: usize) {
        self.threads = threads.max(1);
        self.pool = (self.threads > 1).then(|| {
            rayon::ThreadPoolBuilder::new()
                .num_threads(self.threads)
                .build()
                .expect("build the process thread pool")
        });
    }

    pub fn threads(&self) -> usize {
        self.threads
    }

    /// The base step \[ns\].
    pub fn step_nanos(&self) -> u64 {
        self.step_nanos
    }

    /// The time of the last tick run \[ns\], `None` before the first.
    pub fn current_nanos(&self) -> Option<u64> {
        self.current_nanos
    }

    /// Upstream `ConfigureStopTime(stop_nanos)` and `ExecuteSimulation()`:
    /// every tick up to and including `stop_nanos`, the first call
    /// initializing every process first.
    pub fn execute_to(&mut self, stop_nanos: u64) {
        let mut now = match self.current_nanos {
            Some(now) => now,
            None => {
                self.processes.sort_by_key(|process| {
                    (std::cmp::Reverse(process.priority), process.creation_order)
                });
                for process in &mut self.processes {
                    process.simulation.initialize();
                }
                self.run_tick(0);
                self.current_nanos = Some(0);
                0
            }
        };
        while now + self.step_nanos <= stop_nanos {
            self.run_tick(self.step_nanos);
            now += self.step_nanos;
            self.current_nanos = Some(now);
        }
    }

    /// Runs every process over `duration_nanos` (0 for the first tick, else
    /// one step), group by group.
    fn run_tick(&mut self, duration_nanos: u64) {
        let pool = self.pool.as_ref();
        for group in self
            .processes
            .chunk_by_mut(|first, second| first.priority == second.priority)
        {
            match pool {
                Some(pool) if group.len() > 1 => pool.install(|| {
                    group
                        .par_iter_mut()
                        .for_each(|process| process.simulation.run_for(duration_nanos));
                }),
                _ => group
                    .iter_mut()
                    .for_each(|process| process.simulation.run_for(duration_nanos)),
            }
        }
    }
}
