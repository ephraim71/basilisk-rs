//! Upstream `examples/mujoco` thruster-arm framework: `BSK_mujocoMasters.py`
//! (`BSKSim`, `BSKScenario`), `mujocoModels/BSK_MujocoDynamics.py`,
//! `mujocoModels/BSK_MujocoFSW.py` and `thrArmControlSupport/*Config.py`,
//! with the pure-Python modules they use.
//!
//! | upstream | here |
//! |---|---|
//! | `BSKSim(fswRate, dynRate)`, `set_DynModel`, `set_FswModel` | [`ThrArmSim::new`] |
//! | `BSKScenario.configure_initial_conditions` | each scenario's initial-condition function, through [`ThrArmSim::scene`] |
//! | `BSK_MujocoDynamics.BSKMujocoDynamicsModels` | [`dynamics::MujocoDynamics`] |
//! | `BSK_MujocoFSW.BSKMujocoFSWModels` | [`fsw::MujocoFsw`] |
//! | `RegulationConfig`, `TrackingConfig` | [`config::ThrArmConfig::regulation`], [`config::ThrArmConfig::tracking`] |
//! | `createNewEvent`, `enableTask`, `ConfigureStopTime` + `ExecuteSimulation` | [`events`] (the BskSim port's), [`events::Runner::execute`] |
//! | `jointThrAllocation`, `thrFiringRound`, `arcLineHoldRef`, `stateMerge` (Python `SysModel`s) | [`joint_thr_allocation`], [`thr_firing_round`], [`arc_line_hold_ref`], [`state_merge`] |
//!
//! Upstream's model-setup guards (`DynModels` read before `set_DynModel`, a
//! second `set_FswModel`) are ruled out by construction: [`ThrArmSim::new`]
//! builds the dynamics, then the flight software on it.
//!
//! # Processes and tasks
//!
//! basilisk-rs has one schedule ordered by priority. Upstream runs, at every
//! tick, the dynamics process (created first) before the FSW process, each
//! by task priority and module priority, with recorders after the modules of
//! their task. The priorities below keep that order: the events first (see
//! [`events`]), then the dynamics task and its recorders, then the FSW tasks
//! (outer loop, joint motion, thruster firing, recorders).

#![allow(dead_code)]

#[path = "../cli.rs"]
pub mod cli;
#[path = "../common.rs"]
pub mod common;
#[path = "../bsk_sim/events.rs"]
pub mod events;
#[path = "../message_log.rs"]
pub mod message_log;
#[path = "../recording.rs"]
pub mod recording;
#[path = "../shared.rs"]
pub mod shared;
#[path = "../task_switch.rs"]
pub mod task_switch;

pub mod arc_line_hold_ref;
pub mod bounded_sqp;
pub mod config;
pub mod dynamics;
pub mod fsw;
pub mod joint_thr_allocation;
pub mod state_merge;
pub mod thr_firing_round;

use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::simulation::Simulation;
use hifitime::Epoch;

use config::ThrArmConfig;
use dynamics::MujocoDynamics;
use events::{Events, Runner};
use fsw::MujocoFsw;
use shared::Shared;

/// The event checks: first in every tick.
pub const EVENTS_PRIORITY: i32 = 10_000;
/// Base of the dynamics task's modules (upstream priorities 200 ... 197
/// added to it).
pub const DYNAMICS_PRIORITY: i32 = 9_000;
/// Recorders added to the dynamics task: after its modules.
pub const DYNAMICS_RECORDER_PRIORITY: i32 = 8_900;
/// Bases of the FSW tasks, by upstream task priority (100 ... 97); the
/// modules' upstream priorities are added to them.
pub const OUTER_LOOP_PRIORITY: i32 = 8_000;
pub const JOINT_MOTION_PRIORITY: i32 = 7_000;
pub const THR_FIRING_PRIORITY: i32 = 6_000;
/// Recorders added to a FSW task: after the task's modules.
pub const OUTER_LOOP_RECORDER_PRIORITY: i32 = OUTER_LOOP_PRIORITY;
pub const JOINT_MOTION_RECORDER_PRIORITY: i32 = JOINT_MOTION_PRIORITY;
pub const RECORDER_TASK_PRIORITY: i32 = 5_000;

/// Upstream `macros.sec2nano`: `int(seconds * 1e9 + 0.5)`.
pub fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1e9 + 0.5) as u64
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Upstream `BSKSim` with `set_DynModel(BSK_MujocoDynamics, config)` and
/// `set_FswModel(BSK_MujocoFSW, config)`.
pub struct ThrArmSim {
    pub config: ThrArmConfig,
    /// Upstream `eventMap`.
    pub events: Events,
    pub dynamics: MujocoDynamics,
    pub fsw: MujocoFsw,
}

impl ThrArmSim {
    /// Upstream `BSKSim(fswRate, dynRate)`, `set_DynModel` and
    /// `set_FswModel` with the configuration's rates.
    pub fn new(config: ThrArmConfig) -> Self {
        let fsw_rate = sec2nano(config.fsw_time_step_s);
        let dyn_rate = sec2nano(config.dyn_time_step_s);
        let mut events = Events::new(gcd(fsw_rate, dyn_rate));
        let mut dynamics = MujocoDynamics::new(config.dyn_time_step_s, &config);
        let fsw = MujocoFsw::new(config.fsw_time_step_s, &config, &mut dynamics, &mut events);
        Self {
            config,
            events,
            dynamics,
            fsw,
        }
    }

    /// A handle on the scene, to set the initial conditions between
    /// [`Simulation::initialize`] and the first run.
    pub fn scene(&self) -> Shared<MjScene> {
        self.dynamics.scene.handle()
    }

    /// The handle that runs the simulation; take it before scheduling.
    pub fn runner(&self) -> Runner {
        self.events.runner()
    }

    /// Schedules the events, the dynamics and the flight software.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.events.period_nanos();
        simulation.add_module("events", &mut self.events, period, EVENTS_PRIORITY);
        self.dynamics.schedule(simulation);
        self.fsw.schedule(simulation);
    }
}

/// The simulation's start epoch (the scenarios do not use it).
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
}
