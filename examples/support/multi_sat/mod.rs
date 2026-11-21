//! Upstream `examples/MultiSatBskSim`: the `BSKSim` framework of its
//! multi-spacecraft scenarios (`BSK_MultiSatMasters.py`, `modelsMultiSat/`),
//! here as plain structs and builder functions.
//!
//! | upstream | here |
//! |---|---|
//! | `BSKSim(numberSpacecraft, relativeNavigation, fswRate, dynRate, envRate, relNavRate)` | [`MultiSatSim::new`] with [`Rates`] |
//! | `set_EnvModel(BSK_EnvironmentEarth \| BSK_EnvironmentMercury)` | [`environment::Environment`] of a [`environment::Planet`] |
//! | `set_DynModel([BSK_MultiSatDynamics] * n)` | [`dynamics::Dynamics`], one per spacecraft |
//! | `set_FswModel([BSK_MultiSatFsw] * n)` | [`MultiSatSim::with_fsw`]: [`fsw::Fsw`], one per spacecraft |
//! | `add_relativeNavigation` (`formationBarycenter`) | [`MultiSatSim::relative_navigation`] |
//! | `BSKScenario` (`configure_initial_conditions`, `log_outputs`, `pull_outputs`) | each example's `run(config) -> Results`, with [`logs::SpacecraftLogs`] |
//! | `CreateNewProcess`, `TotalSim.resetThreads(n)`, `ExecuteSimulation` | [`processes::Processes`], built by [`MultiSatSim::processes`] |
//! | the run shared by the orbit-only scenarios | [`orbits::run_orbits`] |
//! | `plottingMultiSat/BSK_MultiSatPlotting.py` | `examples/multi_sat_plotting.py` |
//!
//! A heterogeneous formation (upstream's list of different model classes)
//! is [`MultiSatSim::from_models`] with dynamics and flight software built
//! by other functions.
//!
//! # Processes
//!
//! As upstream, the environment, each spacecraft's dynamics and each
//! spacecraft's flight software are separate processes (priorities 300,
//! 200 and 100; the relative navigation 150), run in that order at every
//! tick. Upstream's mode events, checked before every step, run in an
//! `Events` process ahead of all of them. The processes of equal priority,
//! the spacecraft's dynamics and their flight software, run in parallel when
//! threads are requested, upstream's `TotalSim.resetThreads(n)`.
//!
//! ```ignore
//! let mut sim = MultiSatSim::new(Planet::Earth, 3, Rates::uniform_s(10.0), false).with_fsw();
//! sim.dynamics[0].set_initial_state(r, v, sigma, omega);
//! sim.fsw[0].mode_request.set(FswMode::SunPointing);
//! let mut logs: Vec<SpacecraftLogs> = (0..3).map(|i| SpacecraftLogs::new(&sim, i, period, Logged::ATTITUDE)).collect();
//! let mut processes = sim.processes();
//! for (index, log) in logs.iter_mut().enumerate() {
//!     log.schedule(processes.process(&dynamics_process_name(index)));
//! }
//! processes.reset_threads(4);
//! processes.execute_to(hour2nano(1.0));
//! ```

#![allow(dead_code)]

#[path = "../cli.rs"]
pub mod cli;
#[path = "../common.rs"]
pub mod common;
#[path = "../grav_factory.rs"]
pub mod grav_factory;
#[path = "../latch.rs"]
pub mod latch;
#[path = "../message_log.rs"]
pub mod message_log;
#[path = "../rw_factory.rs"]
pub mod rw_factory;
#[path = "../staged.rs"]
pub mod staged;
#[path = "../table_csv.rs"]
pub mod table_csv;
#[path = "../task_switch.rs"]
pub mod task_switch;
#[path = "../thruster_factory.rs"]
pub mod thruster_factory;

pub mod dynamics;
pub mod environment;
pub mod events;
pub mod fsw;
pub mod logs;
pub mod orbits;
pub mod processes;

use basilisk_rs::fsw_algorithms::formation_flying::formation_barycenter::{
    FormationBarycenter, FormationBarycenterConfig,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use nalgebra::Vector3;

use dynamics::Dynamics;
use environment::{Environment, Planet};
use fsw::Fsw;
use processes::Processes;

/// Upstream process priorities.
pub const ENVIRONMENT_PRIORITY: i32 = 300;
pub const DYNAMICS_PRIORITY: i32 = 200;
pub const RELATIVE_NAVIGATION_PRIORITY: i32 = 150;
pub const FSW_PRIORITY: i32 = 100;
/// Upstream checks its events before every step, ahead of every process.
pub const EVENTS_PRIORITY: i32 = i32::MAX;

/// Upstream `DynamicsProcessName[i]`.
pub fn dynamics_process_name(index: usize) -> String {
    format!("DynamicsProcess{index}")
}

/// Upstream `FSWProcessName[i]`.
pub fn fsw_process_name(index: usize) -> String {
    format!("FSWProcess{index}")
}

/// Upstream `EnvProcessName`.
pub const ENVIRONMENT_PROCESS_NAME: &str = "EnvironmentProcess";
/// Upstream's relative-navigation process.
pub const RELATIVE_NAVIGATION_PROCESS_NAME: &str = "RelativeNavigation";

/// Seconds to nanoseconds (upstream `macros.sec2nano`).
pub fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1e9).round() as u64
}

/// Hours to nanoseconds (upstream `macros.hour2nano`).
pub fn hour2nano(hours: f64) -> u64 {
    sec2nano(hours * 3600.0)
}

/// Upstream `BSKSim`'s task rates \[ns\].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rates {
    pub fsw_nanos: u64,
    pub dyn_nanos: u64,
    pub env_nanos: u64,
    pub rel_nav_nanos: u64,
}

impl Rates {
    /// Every rate `seconds` (the scenarios set them all equal).
    pub fn uniform_s(seconds: f64) -> Self {
        let nanos = sec2nano(seconds);
        Self {
            fsw_nanos: nanos,
            dyn_nanos: nanos,
            env_nanos: nanos,
            rel_nav_nanos: nanos,
        }
    }

    /// Upstream's defaults, 0.1 s each.
    pub fn upstream_default() -> Self {
        Self::uniform_s(0.1)
    }

    /// The base step of the processes: the greatest common divisor of the
    /// rates in use.
    fn base_step_nanos(&self, relative_navigation: bool) -> u64 {
        let mut step = gcd(gcd(self.fsw_nanos, self.dyn_nanos), self.env_nanos);
        if relative_navigation {
            step = gcd(step, self.rel_nav_nanos);
        }
        step
    }
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// Upstream `BSKSim`: the environment, the spacecraft and the optional
/// relative navigation.
pub struct MultiSatSim {
    pub rates: Rates,
    pub environment: Environment,
    /// Upstream `DynModels`.
    pub dynamics: Vec<Dynamics>,
    /// Upstream `FSWModels`; empty without flight software.
    pub fsw: Vec<Fsw>,
    /// Upstream `relativeNavigationModule` (`formationBarycenter`, ModelTag
    /// `RelativeNavigation`), with an input pair per spacecraft. Upstream
    /// connects it, and sets its parameters, in the scenario.
    pub relative_navigation: Option<FormationBarycenter>,
}

impl MultiSatSim {
    /// Upstream `BSKSim(numberSpacecraft, relativeNavigation, rates...)`
    /// followed by `set_EnvModel(planet)` and `set_DynModel` with the
    /// MultiSat dynamics for every spacecraft.
    pub fn new(
        planet: Planet,
        number_spacecraft: usize,
        rates: Rates,
        relative_navigation: bool,
    ) -> Self {
        let mut environment = Environment::new(planet, rates.env_nanos);
        let dynamics = (0..number_spacecraft)
            .map(|index| Dynamics::new(&mut environment, rates.dyn_nanos, index))
            .collect();
        Self::from_models(
            environment,
            dynamics,
            Vec::new(),
            relative_navigation,
            rates,
        )
    }

    /// The simulation from models built elsewhere (a heterogeneous
    /// formation): the dynamics in spacecraft order, the flight software
    /// empty or one per spacecraft.
    pub fn from_models(
        environment: Environment,
        dynamics: Vec<Dynamics>,
        fsw: Vec<Fsw>,
        relative_navigation: bool,
        rates: Rates,
    ) -> Self {
        assert!(
            fsw.is_empty() || fsw.len() == dynamics.len(),
            "one flight software model per spacecraft"
        );
        let relative_navigation = relative_navigation.then(|| {
            FormationBarycenter::new(FormationBarycenterConfig {
                name: "RelativeNavigation".to_string(),
                num_spacecraft: dynamics.len(),
                ..Default::default()
            })
        });
        Self {
            rates,
            environment,
            dynamics,
            fsw,
            relative_navigation,
        }
    }

    /// Upstream `set_FswModel([BSK_MultiSatFsw] * n)`.
    pub fn with_fsw(mut self) -> Self {
        assert!(
            self.fsw.is_empty(),
            "Flight software models have already been added"
        );
        self.fsw = self
            .dynamics
            .iter_mut()
            .map(|dynamics| Fsw::new(&self.environment, dynamics, self.rates.fsw_nanos))
            .collect();
        self
    }

    /// Upstream `numberSpacecraft`.
    pub fn number_spacecraft(&self) -> usize {
        self.dynamics.len()
    }

    /// Upstream `configure_initial_conditions` for one spacecraft: the state
    /// of `elements` about the central body, with `sigma_bn` and
    /// `omega_bn_b`. Returns the position and velocity.
    pub fn set_initial_elements(
        &mut self,
        index: usize,
        elements: &ClassicElements,
        sigma_bn: Vector3<f64>,
        omega_bn_b_radps: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let (position, velocity) = elements_to_state(self.environment.mu_m3ps2, elements);
        self.dynamics[index].set_initial_state(position, velocity, sigma_bn, omega_bn_b_radps);
        (position, velocity)
    }

    /// Upstream `relativeNavigationModule.addSpacecraftToModel(nav, mass)`
    /// for every spacecraft, `useOrbitalElements = False`, `mu`, and every
    /// spacecraft's `spacecraftReconfig` chief is the barycenter.
    pub fn use_barycenter_as_chief(&mut self) {
        let barycenter = self
            .relative_navigation
            .as_mut()
            .expect("the simulation has relative navigation");
        barycenter.config.use_orbital_elements = false;
        barycenter.config.mu_m3ps2 = self.environment.mu_m3ps2;
        for (index, dynamics) in self.dynamics.iter().enumerate() {
            dynamics
                .simple_nav
                .translation_out_msg
                .connect_to(&mut barycenter.sc_nav_in_msgs[index]);
            dynamics
                .simple_mass_props
                .vehicle_config_out_msg
                .connect_to(&mut barycenter.sc_payload_in_msgs[index]);
        }
        let chief = barycenter.trans_out_msg.clone();
        for fsw in &mut self.fsw {
            fsw.set_chief(&chief);
        }
    }

    /// Every spacecraft's `spacecraftReconfig` chief is the navigation of
    /// spacecraft `chief`.
    pub fn use_spacecraft_as_chief(&mut self, chief: usize) {
        let chief = self.dynamics[chief].simple_nav.translation_out_msg.clone();
        for fsw in &mut self.fsw {
            fsw.set_chief(&chief);
        }
    }

    /// The processes of this simulation, its modules scheduled: the events,
    /// the environment, each spacecraft's dynamics, the relative navigation
    /// and each spacecraft's flight software. Add recorders to the dynamics
    /// processes, as upstream adds them to the dynamics tasks, before
    /// running.
    pub fn processes(&mut self) -> Processes<'_> {
        let relative_navigation = self.relative_navigation.is_some();
        let mut processes = Processes::new(
            self.environment.epoch(),
            self.rates.base_step_nanos(relative_navigation),
        );
        let epoch = processes.start_epoch();
        let mut events_simulation = Simulation::new(epoch, false);
        let mut fsw_simulations = Vec::new();
        for fsw in &mut self.fsw {
            let mut simulation = Simulation::new(epoch, false);
            let name = fsw_process_name(fsw.index);
            fsw.schedule(&mut events_simulation, &mut simulation);
            fsw_simulations.push((name, simulation));
        }
        processes.add_process("Events", EVENTS_PRIORITY, events_simulation);
        self.environment
            .schedule(processes.create_process(ENVIRONMENT_PROCESS_NAME, ENVIRONMENT_PRIORITY));
        for dynamics in &mut self.dynamics {
            let name = dynamics_process_name(dynamics.index);
            dynamics.schedule(processes.create_process(name, DYNAMICS_PRIORITY));
        }
        if let Some(barycenter) = &mut self.relative_navigation {
            let simulation = processes.create_process(
                RELATIVE_NAVIGATION_PROCESS_NAME,
                RELATIVE_NAVIGATION_PRIORITY,
            );
            simulation.add_module(
                "RelativeNavigation",
                barycenter,
                self.rates.rel_nav_nanos,
                20,
            );
        }
        for (name, simulation) in fsw_simulations {
            processes.add_process(name, FSW_PRIORITY, simulation);
        }
        processes
    }
}
