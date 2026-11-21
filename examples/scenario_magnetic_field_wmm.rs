//! World Magnetic Model field along an orbit: port of upstream
//! `examples/scenarioMagneticFieldWMM.py`.
//!
//! The spacecraft of `scenarioMagneticFieldCenteredDipole` (default hub, one
//! orbit of the `i = 85 deg` orbit with periapsis at `1.1 REQ`, circular or
//! with apoapsis at `3.5 REQ`) about a point-mass Earth, with
//! `magneticFieldWMM` evaluating the field at the epoch
//! `2019 June 27, 10:23:0.0 (UTC)`. In the elliptical case the model is only
//! evaluated between 10 000 km and 20 000 km (`envMinReach`, `envMaxReach`).
//! One 60 s task, histories sampled every `samplingTime(P, 60 s, 100)`.
//!
//! Upstream reads the coefficients from `WMM.COF` and the epoch from an
//! epoch message; basilisk-rs's [`WmmField`](basilisk_rs::environment::wmm_field::WmmField)
//! embeds WMM2025 and evaluates it at the simulation epoch, so the
//! simulation starts at the upstream epoch.
//!
//! Upstream `run(show_plots, orbitCase)` maps to
//! `--orbit-case circular|elliptical`; `--all` runs both parametrizations of
//! upstream's `test_scenarioMagneticFieldWMM.py`.
//!
//! Writes `examples/output/scenarios/scenarioMagneticFieldWMM<orbitCase>.csv`
//! with `r_BN_N_m_*` and `magField_N_T_*`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/magnetic_field.rs"]
mod magnetic_field;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{sampling_period_nanos, scenario_recorder};
use basilisk_rs::messages::{MagneticFieldMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use magnetic_field::{OrbitCase, Planet, ScenarioOrbit, wmm, wmm_epoch};
use message_log::MessageLog;

/// Upstream `simulationTimeStep = macros.sec2nano(60.0)`.
const TASK_PERIOD_NANOS: u64 = 60_000_000_000;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    orbit_case: OrbitCase,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(orbit_case: OrbitCase) -> Self {
        Self {
            orbit_case,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!("scenarioMagneticFieldWMM{}", self.orbit_case.name())
    }
}

/// The parametrizations of upstream `test_scenarioMagneticFieldWMM.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [
    Config::new(OrbitCase::Circular),
    Config::new(OrbitCase::Elliptical),
];

/// The recorded histories (upstream `magLog`, `dataLog`).
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    field: MessageLog<MagneticFieldMsg>,
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let orbit = ScenarioOrbit::new(Planet::Earth, config.orbit_case);
    let mut spacecraft = orbit.spacecraft(TASK_PERIOD_NANOS);
    Planet::Earth.add_to(&mut spacecraft);
    let mut magnetic_field = wmm(config.orbit_case);

    let duration = orbit.duration_nanos();
    let sample_period = sampling_period_nanos(duration, TASK_PERIOD_NANOS, 100);
    let mut field_log = MessageLog::<MagneticFieldMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(wmm_epoch(), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut magnetic_field.input_state_msg,
        &magnetic_field.output_magnetic_field_msg => &mut field_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
        &magnetic_field.output_magnetic_field_msg => recorder.add_source::<MagneticFieldMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_inertial_t", "magField_N_T")),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "wmm" => &mut magnetic_field, period=TASK_PERIOD_NANOS, priority=90;
        "field_log" => &mut field_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(duration);
    drop(simulation);
    drop(recorder);

    Results {
        field: field_log,
        state: state_log,
        csv_path,
    }
}

fn main() {
    let mut cli =
        Cli::from_env("scenario_magnetic_field_wmm [--orbit-case circular|elliptical] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(cli.value("orbit-case", OrbitCase::Elliptical))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} ({} samples)",
            results.csv_path.expect("CSV written").display(),
            results.field.values.len(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioMagneticFieldWMM.py` only checks that each
    //! parametrization runs. These tests run both and check that the field is
    //! zero exactly outside the reach band, and that far from Earth, where
    //! the dipole dominates, the WMM field is close to the centered dipole of
    //! `scenarioMagneticFieldCenteredDipole`.

    use nalgebra::Vector3;

    use super::*;
    use magnetic_field::wmm_reach_m;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    fn dipole_field(r_n: Vector3<f64>) -> Vector3<f64> {
        let config = Planet::Earth.centered_dipole("dipole");
        let d = Vector3::new(config.g11_t, config.h11_t, config.g10_t);
        let r_hat = r_n.normalize();
        (config.planet_radius_m / r_n.norm()).powi(3) * (3.0 * r_hat * r_hat.dot(&d) - d)
    }

    #[test]
    fn every_upstream_parametrization_runs_for_one_orbit() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            let duration = ScenarioOrbit::new(Planet::Earth, config.orbit_case).duration_nanos();
            let period = sampling_period_nanos(duration, TASK_PERIOD_NANOS, 100);
            assert_eq!(results.field.values.len() as u64, duration / period + 1);
        }
    }

    /// At `1.1 REQ` the field magnitude is that of a low Earth orbit, between
    /// the equatorial and polar dipole values, 15 000 nT to 55 000 nT.
    #[test]
    fn the_circular_orbit_sees_the_low_orbit_field() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        for field in &results.field.values {
            let nanotesla = field.magnetic_field_inertial_t.norm() * 1e9;
            assert!((15_000.0..55_000.0).contains(&nanotesla), "{nanotesla}");
        }
    }

    #[test]
    fn the_elliptical_orbit_field_is_zero_outside_the_reach_band() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        let (Some(min_reach), Some(max_reach)) = wmm_reach_m(OrbitCase::Elliptical) else {
            panic!("the elliptical case has a reach band");
        };
        let mut inside = 0;
        for (field, state) in results.field.values.iter().zip(&results.state.values) {
            let radius = state.position_m.norm();
            let field = field.magnetic_field_inertial_t;
            if radius <= min_reach || radius >= max_reach {
                assert_eq!(field, Vector3::zeros(), "r = {radius}");
            } else {
                inside += 1;
                let dipole = dipole_field(state.position_m);
                let magnitude_error = (field.norm() - dipole.norm()).abs() / dipole.norm();
                let angle = field.angle(&dipole).to_degrees();
                assert!(magnitude_error < 0.2, "r = {radius}: {magnitude_error}");
                assert!(angle < 15.0, "r = {radius}: {angle} deg");
            }
        }
        assert!(inside > 20, "{inside}");
    }
}
