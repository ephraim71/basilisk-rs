//! Earth and Moon gravity on an `MJScene`: port of upstream
//! `examples/mujoco/scenarioMJEarthMoonGravity.py`. Needs the `mujoco`
//! feature, and with SPICE states the ANISE assets (`just
//! fetch-anise-assets`).
//!
//! The CubeSat of `sat_w_wheel.xml` (in `examples/data/mujoco/`), a hub and
//! a wheel, on a 700 km periapsis orbit (e = 0.001, i = 28.5°) for 6 h at
//! 30 s steps, with RKF45 at tolerances 1e-8. Earth (central) and the Moon
//! act on both bodies through one `NBodyGravity` (upstream
//! `gravFactory.addBodiesTo(scene)`), their states from an ephemeris that
//! runs in the scene's dynamics task at priority 75, after the forward
//! kinematics and before the gravity, so every integrator stage reads
//! current planet states; `extraEoMCall` refreshes the dynamics-task outputs
//! at each step's final state for the recorders.
//!
//! | `--spice` / `--no-spice` | ephemeris | CSV |
//! |---|---|---|
//! | `--spice` (default) | upstream `gravFactory.createSpiceInterface` (epoch 2025 November 15 12:00 UTC, `zeroBase` Earth) | `scenarioMJEarthMoonGravity_spice.csv` |
//! | `--no-spice` | `PlanetEphemeris` two-body heliocentric elements of Earth and of the Moon (its geocentric state added to Earth's), `zeroBase` Earth | `scenarioMJEarthMoonGravity_planetEphemeris.csv` |
//!
//! `--all` runs both, as `test_scenarioMujoco.py` does. As in the other
//! basilisk-rs scenarios (see `support/grav_factory.rs`), the SPICE states
//! come from ANISE (DE440s) rather than CSPICE (DE430), one ephemeris module
//! per body.
//!
//! The CSV holds, every step, the hub center-of-mass state and the Earth
//! and Moon ephemeris positions. Upstream raises an error if the recorded
//! radius drops below R_E + 400 km; `run` panics likewise.
//!
//! `test_scenarioMujoco.py` only runs both cases; `tests` checks upstream's
//! altitude bound, the size of the lunar perturbation and that the two
//! ephemerides give nearly the same trajectory. Upstream's Vizard call
//! (`saveFile` commented out) is not ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use basilisk_rs::connect;
use basilisk_rs::environment::planet_ephemeris::{
    PlanetEphemeris, PlanetEphemerisBody, PlanetEphemerisConfig,
};
use basilisk_rs::messages::{Output, PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{AU, MU_SUN};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use cli::Cli;
use common::{seconds, vector_columns};
use grav_factory::Body;
use hifitime::Epoch;
use mj_scenarios::{
    GravitySourceSpec, add_gravity_to_all_bodies, load_scene, rkf45, site_state_columns,
    written_csv,
};
use recording::{History, scenario_csv};
use shared::Shared;

const USAGE: &str = "\
scenario_mj_earth_moon_gravity [--spice | --no-spice]
scenario_mj_earth_moon_gravity --all     run both ephemeris sources";

/// Upstream `TASK_STEP_SEC`.
const STEP_NANOS: u64 = seconds(30);
/// Upstream `SIMULATION_DURATION_SEC` (6 h).
const DURATION_NANOS: u64 = seconds(6 * 3600);
/// Upstream `MINIMUM_SAFE_ALTITUDE_M`.
const MINIMUM_SAFE_ALTITUDE_M: f64 = 400_000.0;
/// Upstream `INITIAL_PERIAPSIS_ALTITUDE_M`.
const INITIAL_PERIAPSIS_ALTITUDE_M: f64 = 700_000.0;
/// Upstream `INITIAL_ECCENTRICITY`.
const INITIAL_ECCENTRICITY: f64 = 0.001;
/// Upstream `EPHEMERIS_PRIORITY`.
const EPHEMERIS_PRIORITY: i32 = 75;

/// Upstream `SPICE_EPOCH` (UTC).
fn spice_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2025, 11, 15, 12, 0, 0, 0)
}

/// Upstream `run(showPlots, useSpice)`.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_spice: bool,
}

impl Config {
    fn file_stem(&self) -> &'static str {
        if self.use_spice {
            "scenarioMJEarthMoonGravity_spice"
        } else {
            "scenarioMJEarthMoonGravity_planetEphemeris"
        }
    }
}

/// `test_scenarioMujoco.py`'s runs.
const TEST_CASES: [Config; 2] = [Config { use_spice: true }, Config { use_spice: false }];

/// Upstream `_create_analytic_ephemeris`: Earth's heliocentric elements and
/// the Moon's, from its geocentric state added to Earth's heliocentric one.
fn analytic_ephemeris() -> PlanetEphemeris {
    let sun_mu = MU_SUN * 1.0e9;
    let earth_elements = ClassicElements {
        semi_major_axis: AU * 1000.0,
        eccentricity: 0.0167086,
        inclination_rad: 0.00005_f64.to_radians(),
        raan_rad: (-11.26064_f64).to_radians(),
        arg_periapsis_rad: 102.94719_f64.to_radians(),
        true_anomaly_rad: 55.0_f64.to_radians(),
        ..Default::default()
    };
    let moon_about_earth = ClassicElements {
        semi_major_axis: 384_400_000.0,
        eccentricity: 0.0549,
        inclination_rad: 5.145_f64.to_radians(),
        raan_rad: 125.08_f64.to_radians(),
        arg_periapsis_rad: 318.15_f64.to_radians(),
        true_anomaly_rad: 35.0_f64.to_radians(),
        ..Default::default()
    };
    let (earth_r, earth_v) = elements_to_state(sun_mu, &earth_elements);
    let (moon_r, moon_v) = elements_to_state(Body::Earth.mu_m3ps2(), &moon_about_earth);
    let moon_elements = state_to_elements(sun_mu, earth_r + moon_r, earth_v + moon_v);
    PlanetEphemeris::new(PlanetEphemerisConfig {
        name: "EarthMoonPlanetEphemeris".to_string(),
        bodies: vec![
            PlanetEphemerisBody {
                name: "Earth".to_string(),
                elements: earth_elements,
                rotation: None,
            },
            PlanetEphemerisBody {
                name: "Moon".to_string(),
                elements: moon_elements,
                rotation: None,
            },
        ],
        zero_base: Some("Earth".to_string()),
    })
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    hub: Vec<(u64, SpacecraftStateMsg)>,
    earth: Vec<(u64, PlanetStateMsg)>,
    moon: Vec<(u64, PlanetStateMsg)>,
}

fn run(config: Config, write_csv: bool) -> Results {
    let mut scene: MjScene = load_scene("sat_w_wheel.xml");
    scene.name = "earthMoonSpacecraft".to_string();
    scene.extra_eom_call = true;
    scene.set_integrator(rkf45(1.0e-8, 1.0e-8));

    // The ephemeris, in the dynamics task ahead of the gravity.
    let (earth_state, moon_state): (Output<PlanetStateMsg>, Output<PlanetStateMsg>) =
        if config.use_spice {
            let earth = Body::Earth.ephemeris(Body::Earth);
            let moon = Body::Moon.ephemeris(Body::Earth);
            let states = (
                earth.output_planet_msg.clone(),
                moon.output_planet_msg.clone(),
            );
            scene.add_module_to_dynamics_task("earthEphemeris", earth, EPHEMERIS_PRIORITY);
            scene.add_module_to_dynamics_task("moonEphemeris", moon, EPHEMERIS_PRIORITY);
            states
        } else {
            let ephemeris = analytic_ephemeris();
            let states = (
                ephemeris.planet_out_msgs[0].clone(),
                ephemeris.planet_out_msgs[1].clone(),
            );
            scene.add_module_to_dynamics_task(
                "EarthMoonPlanetEphemeris",
                ephemeris,
                EPHEMERIS_PRIORITY,
            );
            states
        };
    add_gravity_to_all_bodies(
        &mut scene,
        &[
            GravitySourceSpec {
                name: "earth",
                mu_m3ps2: Body::Earth.mu_m3ps2(),
                is_central_body: true,
                ephemeris: Some(&earth_state),
            },
            GravitySourceSpec {
                name: "moon",
                mu_m3ps2: Body::Moon.mu_m3ps2(),
                is_central_body: false,
                ephemeris: Some(&moon_state),
            },
        ],
    );

    let elements = ClassicElements {
        semi_major_axis: (Body::Earth.radius_equator_m() + INITIAL_PERIAPSIS_ALTITUDE_M)
            / (1.0 - INITIAL_ECCENTRICITY),
        eccentricity: INITIAL_ECCENTRICITY,
        inclination_rad: 28.5_f64.to_radians(),
        raan_rad: 48.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 85.3_f64.to_radians(),
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(Body::Earth.mu_m3ps2(), &elements);

    let hub_state = scene.body("hub").center_of_mass().state_out_msg.clone();
    let mut hub_log = History::<SpacecraftStateMsg>::new();
    let mut earth_log = History::<PlanetStateMsg>::new();
    let mut moon_log = History::<PlanetStateMsg>::new();
    let mut csv = write_csv.then(|| scenario_csv(config.file_stem()));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(spice_epoch(), false);
        connect!(&simulation,
            &hub_state => &mut hub_log.input,
            &earth_state => &mut earth_log.input,
            &moon_state => &mut moon_log.input,
        );
        // Upstream `SCENE_PRIORITY` 0, `RECORDER_PRIORITY` -100.
        simulation.add_module("earthMoonSpacecraft", &mut scene_task, STEP_NANOS, 0);
        simulation.add_module("hubRecorder", &mut hub_log, STEP_NANOS, -100);
        simulation.add_module("earthRecorder", &mut earth_log, STEP_NANOS, -100);
        simulation.add_module("moonRecorder", &mut moon_log, STEP_NANOS, -100);
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
                &earth_state => csv.add_source::<PlanetStateMsg>(CsvSourceConfig::columns(
                    vector_columns("position_inertial_m", "earth_r_N_m"),
                )),
                &moon_state => csv.add_source::<PlanetStateMsg>(CsvSourceConfig::columns(
                    vector_columns("position_inertial_m", "moon_r_N_m"),
                )),
            );
            simulation.add_module("csv", csv, STEP_NANOS, -100);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            let mut hub = scene.body_mut("hub");
            hub.set_position(position_m);
            hub.set_velocity(velocity_mps);
        }
        simulation.run_for(DURATION_NANOS);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    let results = Results {
        hub: hub_log.into_samples(),
        earth: earth_log.into_samples(),
        moon: moon_log.into_samples(),
    };
    // Upstream's run-time check of the altitude.
    let minimum_safe_radius = Body::Earth.radius_equator_m() + MINIMUM_SAFE_ALTITUDE_M;
    let minimum = results.minimum_radius();
    assert!(
        minimum > minimum_safe_radius,
        "The MuJoCo spacecraft crossed the required {:.0} km altitude boundary.",
        MINIMUM_SAFE_ALTITUDE_M / 1000.0
    );
    results
}

impl Results {
    fn minimum_radius(&self) -> f64 {
        self.hub
            .iter()
            .map(|(_, state)| state.position_m.norm())
            .fold(f64::INFINITY, f64::min)
    }

    /// Upstream's `lunarThirdBodyAcceleration` at sample `index`: the Moon's
    /// direct pull on the spacecraft less its pull on Earth \[m/s²\].
    fn lunar_third_body_acceleration(&self, index: usize) -> f64 {
        let moon = self.moon[index].1.position_inertial_m - self.earth[index].1.position_inertial_m;
        let to_spacecraft = moon - self.hub[index].1.position_m;
        let mu = Body::Moon.mu_m3ps2();
        (mu * to_spacecraft / to_spacecraft.norm().powi(3) - mu * moon / moon.norm().powi(3)).norm()
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        TEST_CASES.to_vec()
    } else {
        vec![Config {
            use_spice: cli.flag("spice", true),
        }]
    };
    cli.finish();
    for config in configs {
        let results = run(config, true);
        println!(
            "{}: minimum radius {:.1} km, lunar third-body acceleration {:.3e} m/s^2",
            config.file_stem(),
            results.minimum_radius() / 1000.0,
            results.lunar_third_body_acceleration(0)
        );
        println!("wrote {}", written_csv(config.file_stem()).display());
    }
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` runs both cases; `run` itself checks
    //! upstream's altitude bound. These check what the tutorial shows.

    use super::*;

    #[test]
    fn both_ephemerides_keep_the_orbit_and_perturb_it_slightly() {
        let spice = run(Config { use_spice: true }, false);
        let analytic = run(Config { use_spice: false }, false);
        for results in [&spice, &analytic] {
            assert_eq!(results.hub.len(), 721);
            // Earth is the zero base.
            assert!(
                results
                    .earth
                    .iter()
                    .all(|(_, earth)| earth.position_inertial_m.norm() < 1e-3)
            );
            // The Moon is about 384 000 km away, and pulls the low orbit by
            // about 1e-6 m/s².
            let moon = results.moon[0].1.position_inertial_m.norm();
            assert!(moon > 3.5e8 && moon < 4.1e8, "{moon}");
            let lunar = results.lunar_third_body_acceleration(0);
            assert!(lunar > 1e-7 && lunar < 1e-5, "{lunar}");
            assert!(results.minimum_radius() > Body::Earth.radius_equator_m() + 690_000.0);
        }
        // The Moon is a small perturbation: both ephemerides give nearly
        // the same 6 h trajectory.
        let (_, end_spice) = spice.hub.last().unwrap();
        let (_, end_analytic) = analytic.hub.last().unwrap();
        let difference = (end_spice.position_m - end_analytic.position_m).norm();
        assert!(difference < 200.0, "{difference} m");
    }
}
