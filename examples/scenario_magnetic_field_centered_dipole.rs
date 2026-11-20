//! Centered-dipole planetary magnetic field along an orbit: port of upstream
//! `examples/scenarioMagneticFieldCenteredDipole.py`.
//!
//! A spacecraft (upstream's default hub: 1 kg, identity inertia, no attitude
//! motion) flies one orbit of a polar orbit about a point-mass Earth or
//! Jupiter: periapsis at `1.1 REQ`, circular or with apoapsis at `3.5 REQ`,
//! `i = 85 deg`, `Ω = 48.2 deg`, `ω = 347.8 deg`, `f = 85.3 deg`. A
//! `magneticFieldCenteredDipole` evaluates the field with
//! `simSetPlanetEnvironment.centeredDipoleMagField`'s coefficients (the 2020
//! IGRF dipole for Earth, the NASA fact sheet's tilted dipole for Jupiter).
//! In the elliptical Earth case a second, half-strength dipole covers the
//! orbit below `1.3 REQ`, and the first one only above it. One 10 s task,
//! histories sampled every `samplingTime(P, 10 s, 100)`.
//!
//! Upstream `run(show_plots, orbitCase, planetCase)` maps to
//! `--orbit-case circular|elliptical` and `--planet-case Earth|Jupiter`;
//! `--all` runs the three parametrizations of upstream's
//! `test_scenarioMagneticFieldCenteredDipole.py`.
//!
//! Writes `examples/output/scenarios/scenarioMagneticFieldCenteredDipole<orbitCase><planetCase>.csv`
//! (upstream's figure suffix): `r_BN_N_m_*`, `magField_N_T_*` and, with the
//! second dipole, `magField2_N_T_*`.

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
use hifitime::Epoch;
use magnetic_field::{
    OrbitCase, Planet, ScenarioOrbit, TASK_PERIOD_NANOS, centered_dipole, inner_dipole_reach_m,
    inner_earth_dipole,
};
use message_log::MessageLog;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    orbit_case: OrbitCase,
    planet_case: Planet,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(orbit_case: OrbitCase, planet_case: Planet) -> Self {
        Self {
            orbit_case,
            planet_case,
            write_csv: true,
        }
    }

    /// The elliptical Earth case adds the inner dipole.
    fn has_inner_dipole(&self) -> bool {
        self.orbit_case == OrbitCase::Elliptical && self.planet_case == Planet::Earth
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioMagneticFieldCenteredDipole{}{}",
            self.orbit_case.name(),
            self.planet_case.name()
        )
    }
}

/// The parametrizations of upstream `test_scenarioMagneticFieldCenteredDipole.py`.
const TEST_PARAMETRIZATIONS: [Config; 3] = [
    Config::new(OrbitCase::Circular, Planet::Earth),
    Config::new(OrbitCase::Elliptical, Planet::Earth),
    Config::new(OrbitCase::Elliptical, Planet::Jupiter),
];

/// The recorded histories (upstream `magLog`, `mag2Log`, `dataLog`).
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    field: MessageLog<MagneticFieldMsg>,
    inner_field: Option<MessageLog<MagneticFieldMsg>>,
    state: MessageLog<SpacecraftStateMsg>,
    orbit_period_s: f64,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let orbit = ScenarioOrbit::new(config.planet_case, config.orbit_case);
    let mut spacecraft = orbit.spacecraft(TASK_PERIOD_NANOS);
    config.planet_case.add_to(&mut spacecraft);

    let mut dipole_config = config.planet_case.centered_dipole("CenteredDipole");
    if config.has_inner_dipole() {
        dipole_config.env_min_reach_m = Some(inner_dipole_reach_m());
    }
    let mut dipole = centered_dipole(dipole_config);
    let mut inner_dipole = centered_dipole(inner_earth_dipole("CenteredDipole2"));

    let duration = orbit.duration_nanos();
    let sample_period = sampling_period_nanos(duration, TASK_PERIOD_NANOS, 100);
    let mut field_log = MessageLog::<MagneticFieldMsg>::default();
    let mut inner_field_log = MessageLog::<MagneticFieldMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut dipole.input_state_msg,
        &spacecraft.state_out => &mut inner_dipole.input_state_msg,
        &dipole.output_magnetic_field_msg => &mut field_log.input,
        &inner_dipole.output_magnetic_field_msg => &mut inner_field_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
        &dipole.output_magnetic_field_msg => recorder.add_source::<MagneticFieldMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_inertial_t", "magField_N_T")),
        ),
    );
    if config.has_inner_dipole() {
        connect!(&simulation,
            &inner_dipole.output_magnetic_field_msg => recorder.add_source::<MagneticFieldMsg>(
                CsvSourceConfig::columns(vector_columns(
                    "magnetic_field_inertial_t",
                    "magField2_N_T",
                )),
            ),
        );
    }
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "centered_dipole" => &mut dipole, period=TASK_PERIOD_NANOS, priority=90;
        "field_log" => &mut field_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
    }
    if config.has_inner_dipole() {
        schedule! { simulation,
            "centered_dipole_2" => &mut inner_dipole, period=TASK_PERIOD_NANOS, priority=80;
            "inner_field_log" => &mut inner_field_log, period=sample_period, priority=0;
        }
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(duration);
    drop(simulation);
    drop(recorder);

    Results {
        field: field_log,
        inner_field: config.has_inner_dipole().then_some(inner_field_log),
        state: state_log,
        orbit_period_s: orbit.period_s,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_magnetic_field_centered_dipole [--orbit-case circular|elliptical] \
         [--planet-case Earth|Jupiter] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.value("orbit-case", OrbitCase::Circular),
            cli.value("planet-case", Planet::Earth),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} ({} samples over {:.1} s)",
            results.csv_path.expect("CSV written").display(),
            results.field.values.len(),
            results.orbit_period_s,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioMagneticFieldCenteredDipole.py` only checks that
    //! each parametrization runs. These tests run the same parametrizations
    //! and check the recorded field against the centered-dipole field at the
    //! recorded positions, `B = (R/r)^3 (3 r̂ (r̂·d) - d)`, with the reach
    //! limits upstream's module applies.

    use nalgebra::Vector3;

    use super::*;
    use basilisk_rs::environment::magnetic_field_centered_dipole::MagneticFieldCenteredDipoleConfig;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    fn dipole_field(config: &MagneticFieldCenteredDipoleConfig, r_n: Vector3<f64>) -> Vector3<f64> {
        let radius = r_n.norm();
        if config.env_min_reach_m.is_some_and(|reach| radius <= reach)
            || config.env_max_reach_m.is_some_and(|reach| radius >= reach)
        {
            return Vector3::zeros();
        }
        let d = Vector3::new(config.g11_t, config.h11_t, config.g10_t);
        let r_hat = r_n / radius;
        (config.planet_radius_m / radius).powi(3) * (3.0 * r_hat * r_hat.dot(&d) - d)
    }

    #[track_caller]
    fn assert_field(
        log: &MessageLog<MagneticFieldMsg>,
        states: &MessageLog<SpacecraftStateMsg>,
        config: &MagneticFieldCenteredDipoleConfig,
    ) -> usize {
        let mut evaluated = 0;
        for (field, state) in log.values.iter().zip(&states.values) {
            let truth = dipole_field(config, state.position_m);
            let error = (field.magnetic_field_inertial_t - truth).norm();
            assert!(
                error <= 1.0e-12 * truth.norm().max(1.0e-9),
                "{} vs {truth}",
                field.magnetic_field_inertial_t
            );
            evaluated += usize::from(truth != Vector3::zeros());
        }
        evaluated
    }

    #[test]
    fn every_upstream_parametrization_runs_for_one_orbit() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            let duration =
                ScenarioOrbit::new(config.planet_case, config.orbit_case).duration_nanos();
            let period = sampling_period_nanos(duration, TASK_PERIOD_NANOS, 100);
            assert_eq!(
                results.field.values.len() as u64,
                duration / period + 1,
                "{config:?}"
            );
            // A Keplerian orbit about the point-mass planet: the specific
            // orbital energy is kept over the orbit.
            let mu = config.planet_case.mu_m3ps2();
            let energy = |state: &SpacecraftStateMsg| {
                0.5 * state.velocity_mps.norm_squared() - mu / state.position_m.norm()
            };
            let initial = energy(&results.state.values[0]);
            for state in &results.state.values {
                assert!(
                    ((energy(state) - initial) / initial).abs() < 1.0e-6,
                    "{config:?}"
                );
            }
        }
    }

    #[test]
    fn the_field_is_the_centered_dipole_at_the_spacecraft() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            let mut outer = config.planet_case.centered_dipole("CenteredDipole");
            if config.has_inner_dipole() {
                outer.env_min_reach_m = Some(inner_dipole_reach_m());
            }
            let evaluated = assert_field(&results.field, &results.state, &outer);
            assert!(evaluated > 50, "{config:?}: {evaluated}");
        }
    }

    /// The elliptical Earth case splits the orbit at `1.3 REQ`: exactly one
    /// of the two dipoles is active at each sample, and the inner one gives
    /// half the field of the full-strength one.
    #[test]
    fn the_two_earth_dipoles_split_the_orbit_at_their_reach() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        let inner = results.inner_field.as_ref().expect("inner dipole log");
        let inner_config = inner_earth_dipole("CenteredDipole2");
        let inner_samples = assert_field(inner, &results.state, &inner_config);
        assert!(inner_samples > 0);
        let full = Planet::Earth.centered_dipole("full");
        for ((outer, inner), state) in results
            .field
            .values
            .iter()
            .zip(&inner.values)
            .zip(&results.state.values)
        {
            let outer_on = outer.magnetic_field_inertial_t != Vector3::zeros();
            let inner_on = inner.magnetic_field_inertial_t != Vector3::zeros();
            assert!(outer_on != inner_on);
            if inner_on {
                let half = 0.5 * dipole_field(&full, state.position_m);
                assert!((inner.magnetic_field_inertial_t - half).norm() < 1.0e-12 * half.norm());
            }
        }
    }
}
