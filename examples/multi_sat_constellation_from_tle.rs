//! A constellation initialized from two-line element sets, simulated on
//! several threads: port of upstream
//! `examples/MultiSatBskSim/scenariosMultiSat/scenario_constellationFromTle.py`.
//!
//! The simulation of [`multi_sat_basic_orbit_mt`](multi_sat_basic_orbit_mt.rs)
//! (environment and MultiSat dynamics about the Earth, no flight software,
//! 10 s tasks) for 2 hours, one spacecraft per element set of a TLE file,
//! each from the set's osculating elements, at `sigma_BN = [0.1, 0.2,
//! -0.3]` and at rest. As upstream, the elements start at the environment's
//! epoch (`2021 MAY 04 07:47:48.965 UTC`), not at the sets' epochs.
//!
//! The TLE files are upstream's `tleData`, copied to `examples/data/tle`.
//! [`basilisk_rs::utilities::tle_handling::sat_tle_to_elements`] (upstream
//! `tleHandling.satTle2elem`) reads them: SGP4 (the `sgp4` crate, for
//! upstream's Python `sgp4` package) at each set's epoch, the TEME state
//! rotated to the GCRF, then the osculating elements.
//!
//! Upstream `run(show_plots, tleData, numThreads)` maps to `--tle <file>`
//! and `--threads`, with the `__main__` defaults
//! `examples/data/tle/oneWeb25.tle` and 2; `--all` runs upstream's test
//! case, `spacestations.2le` on 2 threads. Upstream's Vizard set-up is not
//! ported.
//!
//! Writes `examples/output/scenarios/scenario_constellationFromTle_<file stem>_sat<i>.csv`;
//! `plot_multi_sat_constellation_from_tle.py` draws upstream's orbit figure.
//!
//! Upstream's test only checks that the scenario runs. `tests` runs its case
//! and checks the initial states against the elements upstream's
//! `test_readTLE.py` expects of `spacestations.2le`, and that the threaded
//! run equals the serial one.

#[path = "support/multi_sat/mod.rs"]
mod multi_sat;

use std::path::{Path, PathBuf};

use basilisk_rs::utilities::tle_handling::{OnError, TleData, sat_tle_to_elements};
use multi_sat::cli::Cli;
use multi_sat::environment::Planet;
use multi_sat::logs::SpacecraftLogs;
use multi_sat::orbits::{InitialOrbit, OrbitRun, run_orbits};
use multi_sat::{Rates, hour2nano, sec2nano};
use nalgebra::Vector3;

const USAGE: &str = "\
multi_sat_constellation_from_tle [--tle examples/data/tle/oneWeb25.tle] [--threads 2]
multi_sat_constellation_from_tle --all   upstream's test case: spacestations.2le on 2 threads";

/// Upstream `simulationTime`.
const DURATION_HOURS: f64 = 2.0;

/// The TLE file `name` of `examples/data/tle`.
fn tle_path(name: &str) -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("examples/data/tle")
        .join(name)
}

#[derive(Clone, Debug)]
struct Config {
    tle_file: PathBuf,
    threads: usize,
    write_csv: bool,
}

// The histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    tle_data: Vec<TleData>,
    logs: Vec<SpacecraftLogs>,
    csv_paths: Vec<PathBuf>,
}

fn run(config: &Config) -> Results {
    let tle_data = sat_tle_to_elements(&config.tle_file, OnError::Raise)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", config.tle_file.display()));
    // `configure_initial_conditions`.
    let initial: Vec<InitialOrbit> = tle_data
        .iter()
        .map(|data| InitialOrbit {
            elements: data.oe,
            sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            omega_bn_b_radps: Vector3::zeros(),
        })
        .collect();
    let logs = run_orbits(
        &OrbitRun {
            planet: Planet::Earth,
            rates: Rates::uniform_s(10.0),
            sampling_nanos: sec2nano(1.0),
            duration_nanos: hour2nano(DURATION_HOURS),
            threads: config.threads,
        },
        &initial,
    );
    let stem = config
        .tle_file
        .file_stem()
        .map(|stem| stem.to_string_lossy().into_owned())
        .unwrap_or_default();
    let csv_paths = if config.write_csv {
        logs.iter()
            .map(|log| {
                log.write_csv(&format!(
                    "scenario_constellationFromTle_{stem}_sat{}",
                    log.index
                ))
            })
            .collect()
    } else {
        Vec::new()
    };
    Results {
        tle_data,
        logs,
        csv_paths,
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let config = if cli.switch("all") {
        Config {
            tle_file: tle_path("spacestations.2le"),
            threads: 2,
            write_csv: true,
        }
    } else {
        let tle: String = cli.value("tle", String::new());
        Config {
            tle_file: if tle.is_empty() {
                tle_path("oneWeb25.tle")
            } else {
                PathBuf::from(tle)
            },
            threads: cli.value("threads", 2),
            write_csv: true,
        }
    };
    cli.finish();

    let results = run(&config);
    for (data, path) in results.tle_data.iter().zip(&results.csv_paths) {
        println!(
            "{:>12} (NORAD {}): a = {:.1} km, wrote {}",
            data.sat_name,
            data.norad_id,
            data.oe.semi_major_axis / 1e3,
            path.display()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    /// Upstream `test_readTLE.py`'s `EXPECTED_OE_2LE`: `(a, e, i, Omega,
    /// omega, f)` of `spacestations.2le`.
    const EXPECTED_2LE: [[f64; 6]; 3] = [
        [
            6802505.842859207,
            0.0011264735370325686,
            0.903248948599224,
            2.3886009471482503,
            0.8571434002531804,
            5.428341410047342,
        ],
        [
            6770009.6048538275,
            0.0011530055595853114,
            0.7265482390037918,
            1.6261518262462817,
            0.7478010900864719,
            5.535537401708238,
        ],
        [
            6802528.7123854,
            0.001128835473051786,
            0.9032099423215613,
            2.410867356948434,
            0.8567388460152983,
            5.428797206538743,
        ],
    ];

    #[test]
    fn upstream_case() {
        let threaded = run(&Config {
            tle_file: tle_path("spacestations.2le"),
            threads: 2,
            write_csv: false,
        });
        let serial = run(&Config {
            tle_file: tle_path("spacestations.2le"),
            threads: 1,
            write_csv: false,
        });
        assert_eq!(threaded.logs.len(), 3);
        let mu = Planet::Earth.body().mu_m3ps2();
        for ((log, other), expected) in threaded.logs.iter().zip(&serial.logs).zip(EXPECTED_2LE) {
            // The initial state has the elements upstream reads from the
            // TLE (`A_TOL_DIST` and `A_TOL` of `test_readTLE.py`).
            let oe = state_to_elements(mu, log.positions()[0], log.velocities()[0]);
            let actual = [
                oe.semi_major_axis,
                oe.eccentricity,
                oe.inclination_rad,
                oe.raan_rad,
                oe.arg_periapsis_rad,
                oe.true_anomaly_rad,
            ];
            for (index, (actual, wanted)) in actual.iter().zip(expected).enumerate() {
                let tolerance = if index == 0 { 0.1 } else { 1e-5 };
                assert!(
                    (actual - wanted).abs() < tolerance,
                    "spacecraft {} element {index}: {actual} instead of {wanted}",
                    log.index
                );
            }
            assert_eq!(log.positions(), other.positions());
            assert_eq!(log.velocities(), other.velocities());
        }
    }
}
