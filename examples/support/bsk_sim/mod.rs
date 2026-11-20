//! Upstream `examples/BskSim`: the `BSKSim` framework its scenarios are
//! built on, here as plain structs and builder functions.
//!
//! | upstream | here |
//! |---|---|
//! | `BSK_masters.BSKSim(fswRate, dynRate)` | [`BskSim::new`]: the rates, the [`events::Events`] and the fault [`Registry`] |
//! | `set_DynModel(BSK_Dynamics)` | [`dynamics::BskDynamics::new`] |
//! | `set_FswModel(BSK_Fsw)` | [`fsw::BskFsw::new`] |
//! | `BSK_FormationDynamics`, `BSK_FormationFsw` | [`formation_dynamics`], [`formation_fsw`] |
//! | `BSK_Faults` | [`faults`] |
//! | `BSKScenario` (`configure_initial_conditions`, `log_outputs`, `pull_outputs`) | each example's `run(config) -> Results` |
//! | `createNewEvent`, `enableTask`, `modeRequest` | [`events`] |
//! | `scenario.modeRequest = ...`, `ConfigureStopTime` + `ExecuteSimulation` | [`events::ModeRequest::set`], [`events::Runner::execute`] |
//! | `BSK_Plotting` | `examples/bsk_sim_plotting.py` and the `plot_bsk_sim_*.py` scripts |
//!
//! A scenario builds the models, connects its recorders, schedules everything
//! into one [`Simulation`], requests modes and runs:
//!
//! ```ignore
//! let mut base = BskSim::new(0.1, 0.1);
//! let mut dynamics = BskDynamics::new(&mut base, &HubInit::from_elements(...));
//! let mut fsw = BskFsw::new(&mut base, &mut dynamics);
//! let (mode, runner) = (fsw.mode_request(), base.runner());
//! let mut simulation = Simulation::new(dynamics::epoch(), false);
//! base.schedule(&mut simulation);
//! dynamics.schedule(&mut simulation);
//! fsw.schedule(&mut simulation);
//! mode.set(FswMode::HillPoint);
//! runner.execute(&mut simulation, min2nano(10.0));
//! ```
//!
//! # Upstream tests
//!
//! `src/tests/test_bskTestScript.py` runs each BskSim scenario and checks
//! only that it completes; each `bsk_sim_*` example's tests check what its
//! scenario demonstrates instead. `test_bskFaults.py` is ported in
//! `bsk_sim_fault_list`. `test_bskSimModelAccess.py` tests Python's
//! model-setup guards (`DynModels` read before `set_DynModel`, a second
//! `set_FswModel`, a failed constructor): here the types rule those out, as
//! [`BskModels::new`] builds the dynamics before the flight software and
//! returns both.
//!
//! # Processes and tasks
//!
//! basilisk-rs has one schedule ordered by priority. Upstream runs the
//! dynamics process (created first) before the flight-software process at
//! every tick, each by task priority and module priority, and runs the
//! recorders a scenario adds to the dynamics task after that task's modules.
//! The priorities below keep that order: the [`events::Events`] first (see
//! there), the dynamics modules, the dynamics-task recorders, then the
//! flight software. The FSW tasks are [`events::TaskSwitch`]es that the mode
//! events enable and disable; every FSW module is scheduled once, wrapped in
//! an [`events::InTasks`] listing the tasks upstream adds it to.

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
#[path = "../shared.rs"]
pub mod shared;
#[path = "../small_body_simple_nav.rs"]
pub mod simple_nav;
#[path = "../staged.rs"]
pub mod staged;
#[path = "../task_switch.rs"]
pub mod task_switch;
#[path = "../thruster_factory.rs"]
pub mod thruster_factory;

pub mod dynamics;
pub mod events;
pub mod faults;
pub mod formation_dynamics;
pub mod formation_fsw;
pub mod fsw;

use std::path::PathBuf;

use basilisk_rs::overrides::Registry;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use nalgebra::Vector3;

use common::{array_columns, scenario_output_path, vector_columns};
use events::{Events, Runner};

/// The [`events::Events`] module: first in every tick.
pub const EVENTS_PRIORITY: i32 = 10_000;
/// Recorders a scenario adds to upstream's `DynamicsTask`: after its modules,
/// before the flight software.
pub const RECORDER_PRIORITY: i32 = 5_000;
/// Recorders added to the formation's second dynamics task
/// (`DynamicsTask2`): after that task's modules.
pub const RECORDER_TASK2_PRIORITY: i32 = 4_000;

/// Upstream `macros.sec2nano`: `int(seconds * 1e9 + 0.5)`.
pub fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1e9 + 0.5) as u64
}

/// Upstream `macros.min2nano`.
pub fn min2nano(minutes: f64) -> u64 {
    sec2nano(minutes * 60.0)
}

/// Upstream `BSKSim`: the task rates, the event map and, for the faults, the
/// override targets of the models.
pub struct BskSim {
    /// Upstream `fswRate` \[ns\]: the period of every FSW task.
    pub fsw_rate_nanos: u64,
    /// Upstream `dynRate` \[ns\]: the period of the dynamics task(s).
    pub dyn_rate_nanos: u64,
    /// Upstream `eventMap`.
    pub events: Events,
    /// What the faults can be injected into, by `<module>.<port>`.
    pub registry: Registry,
}

impl BskSim {
    /// Upstream `BSKSim(fswRate, dynRate)` \[s\]; upstream's defaults are
    /// `0.1` and `0.1`.
    pub fn new(fsw_rate_s: f64, dyn_rate_s: f64) -> Self {
        let fsw_rate_nanos = sec2nano(fsw_rate_s);
        let dyn_rate_nanos = sec2nano(dyn_rate_s);
        Self {
            fsw_rate_nanos,
            dyn_rate_nanos,
            events: Events::new(gcd(fsw_rate_nanos, dyn_rate_nanos)),
            registry: Registry::new(),
        }
    }

    /// The handle that runs the simulation (`ConfigureStopTime` and
    /// `ExecuteSimulation`); take it before scheduling.
    pub fn runner(&self) -> Runner {
        self.events.runner()
    }

    /// Schedules the event checks.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let period = self.events.period_nanos();
        simulation.add_module("events", &mut self.events, period, EVENTS_PRIORITY);
    }
}

fn gcd(a: u64, b: u64) -> u64 {
    if b == 0 { a } else { gcd(b, a % b) }
}

/// The models of a single-spacecraft scenario: upstream's `BSKSim` with
/// `set_DynModel(BSK_Dynamics)` and `set_FswModel(BSK_Fsw)`.
pub struct BskModels {
    pub base: BskSim,
    pub dynamics: dynamics::BskDynamics,
    pub fsw: fsw::BskFsw,
}

impl BskModels {
    /// Upstream `BSKSim(fswRate, dynRate)` \[s\], `set_DynModel`,
    /// `set_FswModel` and `configure_initial_conditions`.
    pub fn new(fsw_rate_s: f64, dyn_rate_s: f64, initial: &HubInit) -> Self {
        let mut base = BskSim::new(fsw_rate_s, dyn_rate_s);
        let mut dynamics = dynamics::BskDynamics::new(&mut base, initial);
        let fsw = fsw::BskFsw::new(&mut base, &mut dynamics);
        Self {
            base,
            dynamics,
            fsw,
        }
    }

    /// Schedules the events, the dynamics and the flight software.
    pub fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        self.base.schedule(simulation);
        self.dynamics.schedule(simulation);
        self.fsw.schedule(simulation);
    }
}

/// Upstream's `oe` of most BskSim scenarios: `a = 10 000 km`, `i = 33.3
/// deg`, `Omega = 48.2 deg`, `omega = 347.8 deg`, `f = 85.3 deg`, and the
/// scenario's eccentricity, about the Earth, with `sigma_BN` and `omega_BN_B`.
pub fn standard_orbit(
    eccentricity: f64,
    sigma_bn: [f64; 3],
    omega_bn_b_radps: [f64; 3],
) -> HubInit {
    HubInit::from_elements(
        grav_factory::Body::Earth.mu_m3ps2(),
        &elements(10_000_000.0, eccentricity, 33.3, 48.2, 347.8, 85.3),
        sigma_bn,
        omega_bn_b_radps,
    )
}

/// A scenario's `configure_initial_conditions`: the hub's initial state.
#[derive(Clone, Copy, Debug)]
pub struct HubInit {
    /// Upstream `r_CN_NInit` \[m\].
    pub position_m: Vector3<f64>,
    /// Upstream `v_CN_NInit` \[m/s\].
    pub velocity_mps: Vector3<f64>,
    /// Upstream `sigma_BNInit`.
    pub sigma_bn: Vector3<f64>,
    /// Upstream `omega_BN_BInit` \[rad/s\].
    pub omega_bn_b_radps: Vector3<f64>,
}

impl HubInit {
    /// Upstream `rN, vN = orbitalMotion.elem2rv(mu, oe)` and the attitude
    /// and rate.
    pub fn from_elements(
        mu_m3ps2: f64,
        elements: &ClassicElements,
        sigma_bn: [f64; 3],
        omega_bn_b_radps: [f64; 3],
    ) -> Self {
        let (position_m, velocity_mps) = elements_to_state(mu_m3ps2, elements);
        Self {
            position_m,
            velocity_mps,
            sigma_bn: Vector3::from(sigma_bn),
            omega_bn_b_radps: Vector3::from(omega_bn_b_radps),
        }
    }
}

/// Classic elements from upstream's `oe.a`, `oe.e`, `oe.i`, `oe.Omega`,
/// `oe.omega`, `oe.f` (angles in degrees).
pub fn elements(
    a_m: f64,
    e: f64,
    i_deg: f64,
    raan_deg: f64,
    aop_deg: f64,
    f_deg: f64,
) -> ClassicElements {
    ClassicElements {
        semi_major_axis: a_m,
        eccentricity: e,
        inclination_rad: i_deg.to_radians(),
        raan_rad: raan_deg.to_radians(),
        arg_periapsis_rad: aop_deg.to_radians(),
        true_anomaly_rad: f_deg.to_radians(),
        ..ClassicElements::default()
    }
}

/// The initial attitude most BskSim scenarios use: `sigma_BN = [0.1, 0.2,
/// -0.3]`.
pub const SIGMA_BN_INIT: [f64; 3] = [0.1, 0.2, -0.3];
/// The initial rate most BskSim scenarios use \[rad/s\].
pub const OMEGA_BN_B_INIT: [f64; 3] = [0.001, -0.01, 0.03];

/// A scenario-format CSV recorder for `examples/output/scenarios/<stem>.csv`,
/// and the path, when `write_csv`; schedule it only then.
pub fn scenario_recorder(stem: &str, write_csv: bool) -> (CsvRecorder, Option<PathBuf>) {
    let output_path = write_csv.then(|| scenario_output_path(&format!("{stem}.csv")));
    let recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: stem.to_string(),
        output_path: output_path.clone().unwrap_or_default(),
    })
    .with_format(CsvFormat::Scenario);
    (recorder, output_path)
}

/// Upstream `attGuidMsg` recorder fields: `sigma_BR_i`,
/// `omega_BR_B_rad_per_s_i`, prefixed with `prefix` when nonempty.
pub fn guidance_columns(prefix: &str) -> CsvSourceConfig {
    columns(
        prefix,
        vector_columns("sigma_br", "sigma_BR")
            .into_iter()
            .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
    )
}

/// `simpleNav.transOutMsg` recorder fields: `r_BN_N_m_i`, `v_BN_N_m_per_s_i`.
pub fn translation_columns(prefix: &str) -> CsvSourceConfig {
    columns(
        prefix,
        vector_columns("position_bn_n_m", "r_BN_N_m")
            .into_iter()
            .chain(vector_columns("velocity_bn_n_mps", "v_BN_N_m_per_s")),
    )
}

/// `simpleNav.attOutMsg` recorder fields: `sigma_BN_i`,
/// `omega_BN_B_rad_per_s_i`.
pub fn attitude_columns(prefix: &str) -> CsvSourceConfig {
    columns(
        prefix,
        vector_columns("sigma_bn", "sigma_BN")
            .into_iter()
            .chain(vector_columns("omega_bn_b_radps", "omega_BN_B_rad_per_s")),
    )
}

/// `scStateOutMsg` recorder fields: `r_BN_N_m_i`, `v_BN_N_m_per_s_i`,
/// `sigma_BN_i`, `omega_BN_B_rad_per_s_i`.
pub fn spacecraft_state_columns(prefix: &str) -> CsvSourceConfig {
    columns(
        prefix,
        vector_columns("position_m", "r_BN_N_m")
            .into_iter()
            .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
            .chain(vector_columns("sigma_bn", "sigma_BN"))
            .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
    )
}

/// `attRefMsg` recorder fields: `sigma_RN_i`, `omega_RN_N_rad_per_s_i`.
pub fn reference_columns(prefix: &str) -> CsvSourceConfig {
    columns(
        prefix,
        vector_columns("sigma_rn", "sigma_RN")
            .into_iter()
            .chain(vector_columns("omega_rn_n_radps", "omega_RN_N_rad_per_s")),
    )
}

/// `cmdTorqueMsg` recorder fields: `torqueRequestBody_Nm_i`.
pub fn torque_columns(prefix: &str) -> CsvSourceConfig {
    columns(
        prefix,
        vector_columns("torque_request_body_nm", "torqueRequestBody_Nm"),
    )
}

/// `cmdRwMotorMsg` recorder fields of the four wheels: `motorTorque_Nm_i`.
pub fn motor_torque_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(array_columns("motor_torque_nm", "motorTorque_Nm", 4))
}

/// `rwSpeedOutMsg` recorder fields of the four wheels:
/// `wheelSpeeds_rad_per_s_i`.
pub fn wheel_speed_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(array_columns(
        "wheel_speeds_radps",
        "wheelSpeeds_rad_per_s",
        4,
    ))
}

fn columns(prefix: &str, columns: impl IntoIterator<Item = (String, String)>) -> CsvSourceConfig {
    let config = CsvSourceConfig::columns(columns);
    if prefix.is_empty() {
        config
    } else {
        config.with_name(prefix)
    }
}
