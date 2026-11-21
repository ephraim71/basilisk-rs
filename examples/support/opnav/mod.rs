//! Upstream `examples/OpNavScenarios`: the `BSK_OpNav` framework its
//! optical-navigation scenarios are built on, here as plain structs and
//! builder functions on top of the BskSim port's events and models
//! ([`bsk_sim`]).
//!
//! | upstream | here |
//! |---|---|
//! | `BSK_OpNav.BSKSim(fswRate, dynRate)` | [`OpNavSim`] (BskSim's [`BskSim`]): the rates and the [`events`] |
//! | `set_DynModel(BSK_OpNavDynamics)` | [`dynamics::OpNavDynamics::new`] |
//! | `set_FswModel(BSK_OpNavFsw)` | [`fsw::OpNavFsw::new`] |
//! | `BSKScenario` (`configure_initial_conditions`, `log_outputs`, `pull_outputs`) | each example's `run(config) -> Results`, with [`OpNavModels::new`] |
//! | `createNewEvent`, `enableTask`, `modeRequest` | [`fsw::FswMode`], [`fsw::OpNavFsw::mode_request`] and BskSim's [`events`] |
//! | `ConfigureStopTime` + `ExecuteSimulation` | [`events::Runner::execute`] |
//! | `run_vizard`, `vizInterface`, the Vizard application | [`planet_renderer::PlanetRenderer`], a synthetic stand-in (see [`dynamics`]) |
//! | `plottingOpNav/OpNav_Plotting.py` | `examples/opnav_plotting.py` and the `plot_scenario_*.py` scripts |
//! | `scenariosOpNav/OpNavMC/MonteCarlo.py` | [`monte_carlo`] on `basilisk_rs::monte_carlo` |
//!
//! A scenario builds the models, changes what its
//! `configure_initial_conditions` changes, connects its recorders, schedules
//! everything into one [`Simulation`], requests modes and runs:
//!
//! ```ignore
//! let mut models = OpNavModels::new(0.5, 0.5, &initial, cameras);
//! models.fsw.relative_od.config.noise_sf = 5.0;
//! let (mode, runner) = (models.fsw.mode_request(), models.base.runner());
//! let mut simulation = Simulation::new(dynamics::epoch(), false);
//! models.schedule(&mut simulation);
//! mode.set(FswMode::PrepOpNav);
//! runner.execute(&mut simulation, min2nano(3.0));
//! mode.set(FswMode::OpNavAttOd);
//! runner.execute(&mut simulation, min2nano(600.0));
//! ```
//!
//! # The images
//!
//! Upstream's scenarios need the Vizard application to render the camera
//! images (`run_vizard`), and are skipped by upstream's tests without it.
//! Here the images come from [`planet_renderer`], a small ray tracer of a
//! Sun-lit Mars ellipsoid through the camera model that stands in for
//! Vizard. Its images are cleaner than Vizard's (no texture, no terrain on
//! the limb), so the scenarios compare with upstream by outcome
//! (navigation errors, convergence), not by pixel.

#![allow(dead_code)]

#[path = "../bsk_sim/mod.rs"]
pub mod bsk_sim;
#[path = "../multi_sat/environment.rs"]
pub mod multi_sat_environment;
#[path = "../planet_renderer.rs"]
pub mod planet_renderer;
#[path = "../table_csv.rs"]
pub mod table_csv;

pub mod dynamics;
pub mod fsw;
pub mod monte_carlo;
pub mod outputs;

use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::orbital_motion::elements_to_state;
use nalgebra::Vector3;

#[allow(unused_imports)]
pub use bsk_sim::{
    BskSim, HubInit, RECORDER_PRIORITY, cli, common, elements, events, grav_factory, message_log,
    min2nano, sec2nano, staged, thruster_factory,
};

use dynamics::OpNavDynamics;
use fsw::OpNavFsw;

/// Upstream `BSK_OpNav.BSKSim`: BskSim's rates and events.
pub type OpNavSim = BskSim;

/// Upstream `BSKSim(fswRate=0.5, dynRate=0.5)` of every OpNav scenario \[s\].
pub const RATE_S: f64 = 0.5;

/// The camera parameters a scenario changes from upstream's `SetCamera`
/// and `SetCamera2`.
#[derive(Clone, Debug)]
pub struct Cameras {
    pub instrument: basilisk_rs::sensors::camera::CameraConfig,
    pub cam2: basilisk_rs::sensors::camera::CameraConfig,
}

impl Default for Cameras {
    fn default() -> Self {
        Self {
            instrument: dynamics::instrument_camera(),
            cam2: dynamics::second_camera(),
        }
    }
}

/// The models of an OpNav scenario: upstream's `BSKSim` with
/// `set_DynModel(BSK_OpNavDynamics)` and `set_FswModel(BSK_OpNavFsw)`.
pub struct OpNavModels {
    pub base: OpNavSim,
    pub dynamics: OpNavDynamics,
    pub fsw: OpNavFsw,
}

impl OpNavModels {
    /// Upstream `BSKSim(fswRate, dynRate)` \[s\], `set_DynModel`,
    /// `set_FswModel`, and the hub state of `configure_initial_conditions`.
    pub fn new(fsw_rate_s: f64, dyn_rate_s: f64, initial: &HubInit, cameras: Cameras) -> Self {
        let mut base = OpNavSim::new(fsw_rate_s, dyn_rate_s);
        let mut dynamics = OpNavDynamics::new(&base, initial, cameras.instrument, cameras.cam2);
        let fsw = OpNavFsw::new(&mut base, &mut dynamics);
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

/// A scenario's `oe` about Mars (`gravBodies['mars barycenter'].mu`), angles
/// in degrees: the hub's position \[m\] and velocity \[m/s\].
pub fn mars_orbit(
    a_m: f64,
    e: f64,
    i_deg: f64,
    raan_deg: f64,
    aop_deg: f64,
    f_deg: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    elements_to_state(
        grav_factory::Body::MarsBarycenter.mu_m3ps2(),
        &elements(a_m, e, i_deg, raan_deg, aop_deg, f_deg),
    )
}

/// The hub state of a scenario: position and velocity, `sigma_BN`, at rest.
pub fn hub_init(
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    sigma_bn: [f64; 3],
) -> HubInit {
    HubInit {
        position_m,
        velocity_mps,
        sigma_bn: Vector3::from(sigma_bn),
        omega_bn_b_radps: Vector3::zeros(),
    }
}

/// Upstream's mode sequence of a scenario: requests each mode and runs to
/// its stop time \[min\] (`modeRequest = ...`, `ConfigureStopTime`,
/// `ExecuteSimulation`).
pub fn execute_modes(
    simulation: &mut Simulation<'_>,
    mode: &events::ModeRequest<fsw::FswMode>,
    runner: &events::Runner,
    phases: &[(fsw::FswMode, f64)],
) {
    for (requested, stop_min) in phases {
        mode.set(*requested);
        runner.execute(simulation, min2nano(*stop_min));
    }
}

/// The command line of upstream's `run(showPlots, simTime)`: `--sim-time m`
/// sets `simTime` \[min\] (upstream's test runs 10), `--all` is accepted and
/// runs the default, as upstream's test only runs the scenario.
pub fn sim_time_from_cli(example: &str, default_min: f64) -> f64 {
    let mut cli = cli::Cli::from_env(&format!(
        "{example} [--sim-time minutes] [--all]   (upstream run(showPlots, simTime); default {default_min}, the test runs 10)"
    ));
    let _all = cli.switch("all");
    let sim_time_min = cli.value("sim-time", default_min);
    cli.finish();
    sim_time_min
}
