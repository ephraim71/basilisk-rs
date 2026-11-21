//! Three spacecraft orbiting the Earth or Mercury with the MultiSat
//! framework: port of upstream
//! `examples/MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat.py`.
//!
//! The framework is [`multi_sat`] (upstream `BSK_MultiSatMasters` and
//! `modelsMultiSat`): an environment process (the Sun, the planet and, for
//! the Earth, the Moon from ANISE ephemerides; an eclipse; the Boulder
//! ground station) and one dynamics process per spacecraft (750 kg hub,
//! four reaction wheels, a thruster with its tank, navigation, mass
//! properties and power). No flight software. Every task runs at 10 s, for
//! 5 hours. The three orbits are upstream's:
//!
//! | spacecraft | `a` | `e` | `i` | `Ω`, `ω`, `f` |
//! |---|---|---|---|---|
//! | 0 | 1.1 R | 0.01 | 45° | 48.2°, 347.8°, 85.3° |
//! | 1 | 1.2 R | 0.1 | 30° | as 0 |
//! | 2 | 1.3 R | 0.05 | 60° | as 0 |
//!
//! with `sigma_BN = [0.1, 0.2, -0.3]` at rest. Upstream hard-codes three
//! spacecraft, so `numberSpacecraft` must be 3.
//!
//! Upstream `run(show_plots, numberSpacecraft, environment)` maps to
//! `--environment Earth|Mercury` (default Earth, as upstream's `__main__`);
//! `--all` runs both, as `test_scenarioBasicOrbitMultiSat.py` does. Upstream's
//! Vizard set-up (`vizSupport`, only when Vizard is installed) is not ported.
//!
//! Writes `examples/output/scenarios/scenario_BasicOrbitMultiSat<Environment>_sat<i>.csv`
//! (columns: see `multi_sat::logs`); `plot_multi_sat_basic_orbit.py` draws
//! upstream's orbit figure from them.
//!
//! Upstream's test only checks that the scenario runs. `tests` also checks
//! the initial states against upstream's elements, that the attitude and
//! wheels stay at rest without flight software, and that the orbits keep
//! their elements under the third-body perturbations.

#[path = "support/multi_sat/mod.rs"]
mod multi_sat;

use std::path::PathBuf;

use basilisk_rs::utilities::orbital_motion::ClassicElements;
use multi_sat::cli::Cli;
use multi_sat::environment::Planet;
use multi_sat::logs::SpacecraftLogs;
use multi_sat::orbits::{InitialOrbit, OrbitRun, run_orbits};
use multi_sat::{Rates, hour2nano, sec2nano};
use nalgebra::Vector3;

const USAGE: &str = "\
multi_sat_basic_orbit [--environment Earth|Mercury]
multi_sat_basic_orbit --all   both environments (upstream's test)";

/// Upstream `numberSpacecraft` (the scenario's initial conditions are for
/// three).
const NUMBER_SPACECRAFT: usize = 3;
/// Upstream `simulationTime`.
const DURATION_HOURS: f64 = 5.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    environment: Planet,
    write_csv: bool,
}

impl Config {
    fn new(environment: Planet) -> Self {
        Self {
            environment,
            write_csv: true,
        }
    }
}

/// The parametrizations of upstream's test.
const TEST_ENVIRONMENTS: [Planet; 2] = [Planet::Mercury, Planet::Earth];

/// Upstream's `snTransLog`, `snAttLog`, `rwSpeedLog` and `rwLogs`.
// The histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    logs: Vec<SpacecraftLogs>,
    initial: Vec<InitialOrbit>,
    csv_paths: Vec<PathBuf>,
}

/// Upstream `configure_initial_conditions` about a planet of radius
/// `planet_radius_m`.
fn initial_orbits(planet_radius_m: f64) -> Vec<InitialOrbit> {
    let degree = std::f64::consts::PI / 180.0;
    let first = ClassicElements {
        semi_major_axis: 1.1 * planet_radius_m,
        eccentricity: 0.01,
        inclination_rad: 45.0 * degree,
        raan_rad: 48.2 * degree,
        arg_periapsis_rad: 347.8 * degree,
        true_anomaly_rad: 85.3 * degree,
        ..Default::default()
    };
    let elements = [
        first,
        ClassicElements {
            semi_major_axis: 1.2 * planet_radius_m,
            eccentricity: 0.1,
            inclination_rad: 30.0 * degree,
            ..first
        },
        ClassicElements {
            semi_major_axis: 1.3 * planet_radius_m,
            eccentricity: 0.05,
            inclination_rad: 60.0 * degree,
            ..first
        },
    ];
    elements
        .into_iter()
        .map(|elements| InitialOrbit {
            elements,
            sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            omega_bn_b_radps: Vector3::zeros(),
        })
        .collect()
}

fn run(config: Config) -> Results {
    let initial = initial_orbits(config.environment.body().radius_equator_m());
    assert_eq!(initial.len(), NUMBER_SPACECRAFT);
    let logs = run_orbits(
        &OrbitRun {
            planet: config.environment,
            rates: Rates::uniform_s(10.0),
            sampling_nanos: sec2nano(1.0),
            duration_nanos: hour2nano(DURATION_HOURS),
            threads: 1,
        },
        &initial,
    );
    let csv_paths = if config.write_csv {
        logs.iter()
            .map(|log| {
                log.write_csv(&format!(
                    "scenario_BasicOrbitMultiSat{}_sat{}",
                    config.environment.name(),
                    log.index
                ))
            })
            .collect()
    } else {
        Vec::new()
    };
    Results {
        logs,
        initial,
        csv_paths,
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        TEST_ENVIRONMENTS.map(Config::new).to_vec()
    } else {
        let environment: String = cli.value("environment", "Earth".to_string());
        let environment = environment.parse().unwrap_or_else(|error: String| {
            eprintln!("error: {error}\n\n{USAGE}");
            std::process::exit(2)
        });
        vec![Config::new(environment)]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        for path in &results.csv_paths {
            println!("wrote {}", path.display());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::orbital_motion::{elements_to_state, state_to_elements};

    fn check(environment: Planet) {
        let results = run(Config {
            environment,
            write_csv: false,
        });
        let mu = environment.body().mu_m3ps2();
        let samples = (hour2nano(DURATION_HOURS) / sec2nano(10.0) + 1) as usize;
        for (log, initial) in results.logs.iter().zip(&results.initial) {
            assert_eq!(log.trans.values.len(), samples);
            // Upstream's initial state: `elem2rv(mu, oe)`.
            let (r0, v0) = elements_to_state(mu, &initial.elements);
            assert!((log.positions()[0] - r0).norm() < 1e-6);
            assert!((log.velocities()[0] - v0).norm() < 1e-9);
            // No flight software: the attitude, rate and wheels stay at rest.
            for (att, speed) in log.att.values.iter().zip(&log.rw_speed.values) {
                assert!((att.sigma_bn - initial.sigma_bn).norm() < 1e-12);
                assert!(att.omega_bn_b_radps.norm() < 1e-12);
                assert!(speed.wheel_speeds_radps.iter().all(|speed| *speed == 0.0));
            }
            // Point-mass orbits perturbed by the Sun (and the Moon): the
            // osculating elements hardly change in five hours.
            let last = state_to_elements(
                mu,
                *log.positions().last().unwrap(),
                *log.velocities().last().unwrap(),
            );
            let da = (last.semi_major_axis - initial.elements.semi_major_axis).abs()
                / initial.elements.semi_major_axis;
            let de = (last.eccentricity - initial.elements.eccentricity).abs();
            let di = (last.inclination_rad - initial.elements.inclination_rad).abs();
            assert!(
                da < 1e-5,
                "{environment:?} sat {}: da/a = {da:e}",
                log.index
            );
            assert!(de < 1e-5, "{environment:?} sat {}: de = {de:e}", log.index);
            assert!(di < 1e-5, "{environment:?} sat {}: di = {di:e}", log.index);
        }
    }

    #[test]
    fn earth() {
        check(Planet::Earth);
    }

    #[test]
    fn mercury() {
        check(Planet::Mercury);
    }
}
