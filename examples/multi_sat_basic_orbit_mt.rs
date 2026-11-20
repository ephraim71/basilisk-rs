//! A formation of spacecraft simulated on several threads: port of upstream
//! `examples/MultiSatBskSim/scenariosMultiSat/scenario_BasicOrbitMultiSat_MT.py`.
//!
//! The simulation of [`multi_sat_basic_orbit`](multi_sat_basic_orbit.rs)
//! (environment and MultiSat dynamics, no flight software, 10 s tasks) with
//! any number of spacecraft for 24 hours, spacecraft `i` on upstream's orbit
//! `a = 1.1 R + 100 (i + 1) km`, `e = 0.01 + 0.001 i`, `i = 45°`,
//! `Ω = 48.2° + 5° i`, `ω = 347.8°`, `f = 85.3°`, with
//! `sigma_BN = [0.1, 0.2, -0.3]` at rest.
//!
//! # Threads
//!
//! Upstream's `TotalSim.resetThreads(numThreads)` assigns its processes to
//! threads round-robin and runs them concurrently, its messaging not
//! thread-safe. Here the processes of [`multi_sat::processes`] run in
//! lockstep: at every tick the environment process, then all spacecraft's
//! dynamics processes in parallel on a `rayon` pool of `--threads` threads
//! (each reads only the environment and its own spacecraft), so the result
//! is bit-for-bit the serial one's. `main` prints the wall time.
//!
//! Upstream `run(show_plots, numberSpacecraft, environment, numThreads)`
//! maps to `--number-spacecraft`, `--environment` and `--threads`, with the
//! `__main__` defaults 32, Earth and 4; `--all` runs upstream's test case,
//! 16 spacecraft about the Earth on 4 threads.
//!
//! Writes `examples/output/scenarios/scenario_BasicOrbitMultiSat_MT<Environment>_sat<i>.csv`;
//! `plot_multi_sat_basic_orbit_mt.py` draws upstream's orbit figure.
//!
//! Upstream's test only checks that the scenario runs; `tests` runs its case
//! and checks that the threaded run equals the serial one and that the
//! orbits keep their elements.

#[path = "support/multi_sat/mod.rs"]
mod multi_sat;

use std::path::PathBuf;
use std::time::Instant;

use basilisk_rs::utilities::orbital_motion::ClassicElements;
use multi_sat::cli::Cli;
use multi_sat::environment::Planet;
use multi_sat::logs::SpacecraftLogs;
use multi_sat::orbits::{InitialOrbit, OrbitRun, run_orbits};
use multi_sat::{Rates, hour2nano, sec2nano};
use nalgebra::Vector3;

const USAGE: &str = "\
multi_sat_basic_orbit_mt [--number-spacecraft 32] [--environment Earth|Mercury] [--threads 4]
multi_sat_basic_orbit_mt --all   upstream's test case: 16 spacecraft, Earth, 4 threads";

/// Upstream `simulationTime`.
const DURATION_HOURS: f64 = 24.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    number_spacecraft: usize,
    environment: Planet,
    threads: usize,
    write_csv: bool,
}

/// Upstream `test_scenarioBasicOrbitMultiSat_MT.py`'s case.
const TEST_CASE: Config = Config {
    number_spacecraft: 16,
    environment: Planet::Earth,
    threads: 4,
    write_csv: true,
};

// The histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    logs: Vec<SpacecraftLogs>,
    initial: Vec<InitialOrbit>,
    csv_paths: Vec<PathBuf>,
}

/// Upstream `configure_initial_conditions`. Upstream sets `omega_BN_BInit`
/// of the first spacecraft only; the others keep the default, also zero.
fn initial_orbits(number_spacecraft: usize, planet_radius_m: f64) -> Vec<InitialOrbit> {
    let degree = std::f64::consts::PI / 180.0;
    (0..number_spacecraft)
        .map(|index| {
            let i = index as f64;
            InitialOrbit {
                elements: ClassicElements {
                    semi_major_axis: 1.1 * planet_radius_m + 1e5 * (i + 1.0),
                    eccentricity: 0.01 + 0.001 * i,
                    inclination_rad: 45.0 * degree,
                    raan_rad: (48.2 + 5.0 * i) * degree,
                    arg_periapsis_rad: 347.8 * degree,
                    true_anomaly_rad: 85.3 * degree,
                    ..Default::default()
                },
                sigma_bn: Vector3::new(0.1, 0.2, -0.3),
                omega_bn_b_radps: Vector3::zeros(),
            }
        })
        .collect()
}

fn run(config: Config) -> Results {
    let initial = initial_orbits(
        config.number_spacecraft,
        config.environment.body().radius_equator_m(),
    );
    let logs = run_orbits(
        &OrbitRun {
            planet: config.environment,
            rates: Rates::uniform_s(10.0),
            sampling_nanos: sec2nano(1.0),
            duration_nanos: hour2nano(DURATION_HOURS),
            threads: config.threads,
        },
        &initial,
    );
    let csv_paths = if config.write_csv {
        logs.iter()
            .map(|log| {
                log.write_csv(&format!(
                    "scenario_BasicOrbitMultiSat_MT{}_sat{}",
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
    let config = if cli.switch("all") {
        TEST_CASE
    } else {
        let environment: String = cli.value("environment", "Earth".to_string());
        Config {
            number_spacecraft: cli.value("number-spacecraft", 32),
            environment: environment.parse().unwrap_or_else(|error: String| {
                eprintln!("error: {error}\n\n{USAGE}");
                std::process::exit(2)
            }),
            threads: cli.value("threads", 4),
            write_csv: true,
        }
    };
    cli.finish();

    let started = Instant::now();
    let results = run(config);
    println!(
        "{} spacecraft, {} h on {} threads in {:.2} s",
        config.number_spacecraft,
        DURATION_HOURS,
        config.threads,
        started.elapsed().as_secs_f64()
    );
    for path in &results.csv_paths {
        println!("wrote {}", path.display());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    /// Upstream's case on 4 threads equals the serial run, bit for bit, and
    /// every orbit keeps its elements.
    #[test]
    fn threaded_run_equals_the_serial_run() {
        let threaded = run(Config {
            write_csv: false,
            ..TEST_CASE
        });
        let serial = run(Config {
            threads: 1,
            write_csv: false,
            ..TEST_CASE
        });
        assert_eq!(threaded.logs.len(), 16);
        let mu = Planet::Earth.body().mu_m3ps2();
        for ((parallel, serial), initial) in threaded
            .logs
            .iter()
            .zip(&serial.logs)
            .zip(&threaded.initial)
        {
            assert_eq!(parallel.trans.times_nanos, serial.trans.times_nanos);
            assert_eq!(parallel.positions(), serial.positions());
            assert_eq!(parallel.velocities(), serial.velocities());
            let last = state_to_elements(
                mu,
                *parallel.positions().last().unwrap(),
                *parallel.velocities().last().unwrap(),
            );
            let da = (last.semi_major_axis - initial.elements.semi_major_axis).abs()
                / initial.elements.semi_major_axis;
            assert!(da < 1e-5, "sat {}: da/a = {da:e}", parallel.index);
            assert!((last.inclination_rad - initial.elements.inclination_rad).abs() < 1e-5);
        }
    }
}
