//! Port of upstream `examples/scenarioBasicOrbit.py`: a spacecraft on a LEO,
//! GTO or GEO orbit about a point-mass or J2 Earth or Mars.
//!
//! Every parametrization of upstream's `test_scenarioBasicOrbit.py` runs:
//!
//! | `--orbit` | `--spherical-harmonics` | `--planet` | CSV |
//! |---|---|---|---|
//! | LEO | no | Earth | `scenarioBasicOrbitLEO0Earth.csv` |
//! | GTO | no | Earth | `scenarioBasicOrbitGTO0Earth.csv` |
//! | GEO | no | Earth | `scenarioBasicOrbitGEO0Earth.csv` |
//! | LEO | yes | Earth | `scenarioBasicOrbitLEO1Earth.csv` |
//! | LEO | no | Mars | `scenarioBasicOrbitLEO0Mars.csv` |
//! | LEO | yes | Mars | `scenarioBasicOrbitLEO1Mars.csv` |
//!
//! `--all` runs the six. Without spherical harmonics the run lasts 0.75 orbits
//! and is sampled 100 times; with the J2-only `GGM03S-J2-only.txt` (Earth) or
//! `GGM2B-J2-only.txt` (Mars) field (`just fetch-orbit-scenario-data`) it
//! lasts three orbits, sampled 400 times. The J2-only fields are zonal, so, as
//! upstream notes, no planet rotation is needed.
//!
//! Upstream's check, ported in `tests`: the final position differs from the
//! two-body (Kepler) solution by at most 1 m (only computed, as upstream, for
//! the point-mass cases).

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use grav_factory::{Body, GGM2B_J2_ONLY, GGM03S_J2_ONLY};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{kepler_state, nano2sec, orbit_period_s, sec2nano};
use recording::{sampling_time, scenario_csv, state_columns};

const USAGE: &str = "\
scenario_basic_orbit [--orbit LEO|GTO|GEO] [--planet Earth|Mars] [--spherical-harmonics]
scenario_basic_orbit --all     run every upstream test parametrization";

/// Upstream `simulationTimeStep`.
const STEP_NANOS: u64 = 10_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrbitCase {
    Leo,
    Gto,
    Geo,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanetCase {
    Earth,
    Mars,
}

/// Upstream `run(show_plots, orbitCase, useSphericalHarmonics, planetCase)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub orbit: OrbitCase,
    pub spherical_harmonics: bool,
    pub planet: PlanetCase,
}

impl Config {
    /// The upstream file-name suffix, `LEO0Earth`.
    pub fn label(&self) -> String {
        let orbit = match self.orbit {
            OrbitCase::Leo => "LEO",
            OrbitCase::Gto => "GTO",
            OrbitCase::Geo => "GEO",
        };
        let planet = match self.planet {
            PlanetCase::Earth => "Earth",
            PlanetCase::Mars => "Mars",
        };
        format!("{orbit}{}{planet}", u8::from(self.spherical_harmonics))
    }
}

/// The parametrizations of `test_scenarioBasicOrbit.py`.
pub const TEST_CASES: [Config; 6] = [
    Config {
        orbit: OrbitCase::Leo,
        spherical_harmonics: false,
        planet: PlanetCase::Earth,
    },
    Config {
        orbit: OrbitCase::Gto,
        spherical_harmonics: false,
        planet: PlanetCase::Earth,
    },
    Config {
        orbit: OrbitCase::Geo,
        spherical_harmonics: false,
        planet: PlanetCase::Earth,
    },
    Config {
        orbit: OrbitCase::Leo,
        spherical_harmonics: true,
        planet: PlanetCase::Earth,
    },
    Config {
        orbit: OrbitCase::Leo,
        spherical_harmonics: false,
        planet: PlanetCase::Mars,
    },
    Config {
        orbit: OrbitCase::Leo,
        spherical_harmonics: true,
        planet: PlanetCase::Mars,
    },
];

pub struct Results {
    /// The CSV written.
    pub csv_path: std::path::PathBuf,
    /// The recorded spacecraft states.
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream's `oe` after `rv2elem`: the initial elements.
    pub initial_elements: ClassicElements,
    pub mu_m3ps2: f64,
    /// Upstream `finalDiff`: distance of the last sample from the Kepler
    /// solution (m); zero with spherical harmonics, as upstream.
    pub final_difference_m: f64,
}

/// The initial orbit elements of upstream's `orbitCase` (before `rv2elem`).
fn orbit_elements(orbit: OrbitCase) -> ClassicElements {
    let degrees = std::f64::consts::PI / 180.0;
    let r_leo = 7_000.0 * 1_000.0;
    let r_geo = 42_000.0 * 1_000.0;
    let (semi_major_axis, eccentricity, inclination_rad) = match orbit {
        OrbitCase::Geo => (r_geo, 0.00001, 0.0),
        OrbitCase::Gto => {
            let a = (r_leo + r_geo) / 2.0;
            (a, 1.0 - r_leo / a, 0.0)
        }
        OrbitCase::Leo => (r_leo, 0.0001, 33.3 * degrees),
    };
    ClassicElements {
        semi_major_axis,
        eccentricity,
        inclination_rad,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    }
}

pub fn run(config: Config) -> Results {
    let (body, field) = match config.planet {
        PlanetCase::Mars => (Body::MarsBarycenter, GGM2B_J2_ONLY),
        PlanetCase::Earth => (Body::Earth, GGM03S_J2_ONLY),
    };
    let planet = if config.spherical_harmonics {
        body.spherical_harmonics(field, 2, true)
    } else {
        body.point_mass(true)
    };
    let mu = body.mu_m3ps2();

    let (position_m, velocity_mps) = elements_to_state(mu, &orbit_elements(config.orbit));
    // Upstream stores the consistent elements of the state it starts from.
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let period_s = orbit_period_s(mu, elements.semi_major_axis);
    let (duration_nanos, points) = if config.spherical_harmonics {
        (sec2nano(3.0 * period_s), 400)
    } else {
        (sec2nano(0.75 * period_s), 100)
    };
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, points);

    // Upstream leaves the hub at its defaults: 1 kg, identity inertia.
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(planet)
        .expect("one central gravity body");

    let file_stem = format!("scenarioBasicOrbit{}", config.label());
    let mut csv = scenario_csv(&file_stem);
    let csv_path = csv.config.output_path.clone();
    let mut history = MessageLog::<SpacecraftStateMsg>::default();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
        );
        schedule! { simulation,
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "history" => &mut history, period=sample_nanos, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");

    let states = history.into_samples();
    let final_difference_m = if config.spherical_harmonics {
        0.0
    } else {
        let (time_nanos, last) = states.last().expect("recorded states");
        let (kepler_position_m, _) = kepler_state(mu, &elements, nano2sec(*time_nanos));
        (last.position_m - kepler_position_m).norm()
    };
    Results {
        csv_path,
        states,
        initial_elements: elements,
        mu_m3ps2: mu,
        final_difference_m,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let configs = if args.switch("all") {
        TEST_CASES.to_vec()
    } else {
        let orbit = match args.value("orbit", "LEO".to_string()).as_str() {
            "LEO" => OrbitCase::Leo,
            "GTO" => OrbitCase::Gto,
            "GEO" => OrbitCase::Geo,
            other => args.fail(&format!("unknown orbit case {other:?}")),
        };
        let planet = match args.value("planet", "Earth".to_string()).as_str() {
            "Earth" => PlanetCase::Earth,
            "Mars" => PlanetCase::Mars,
            other => args.fail(&format!("unknown planet case {other:?}")),
        };
        vec![Config {
            orbit,
            spherical_harmonics: args.switch("spherical-harmonics"),
            planet,
        }]
    };
    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final Kepler difference {:.3e} m)",
            results.csv_path.display(),
            results.final_difference_m
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{TEST_CASES, run};

    /// `test_scenarioBasicOrbit.py`, every parametrization: `posData`
    /// (`finalDiff`) at most `accuracy = 1.0` m.
    #[test]
    fn final_position_matches_the_kepler_solution() {
        for config in TEST_CASES {
            let results = run(config);
            assert!(
                results.final_difference_m <= 1.0,
                "{}: final difference {} m",
                config.label(),
                results.final_difference_m
            );
            assert!(!results.states.is_empty());
        }
    }
}
