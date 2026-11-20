//! Three-axis magnetometer (TAM) along an orbit: port of upstream
//! `examples/scenarioTAM.py`.
//!
//! The orbit of `scenarioMagneticFieldCenteredDipole` (default hub, one orbit
//! of the `i = 85 deg` orbit with periapsis at `1.1 REQ`, circular or with
//! apoapsis at `3.5 REQ`). About Earth the field is WMM at
//! `2019 June 27, 10:23:0.0 (UTC)`, evaluated only between 10 000 km and
//! 20 000 km in the elliptical case; about Jupiter it is the fact-sheet
//! centered dipole. A magnetometer aligned with the body axes measures it
//! with 100 nT white noise, optionally a `[0, 0, -1] uT` bias
//! (`useBias`) and optionally a `±350 uT` saturation (`useBounds`). One 10 s
//! task, histories sampled every `samplingTime(P, 10 s, 100)`.
//!
//! Upstream `run(show_plots, orbitCase, planetCase, useBias, useBounds)`
//! maps to `--orbit-case`, `--planet-case`, `--[no-]bias` and
//! `--[no-]bounds`; `--all` runs the four parametrizations of upstream's
//! `test_scenarioTAM.py`, whose truth the tests check.
//!
//! The noise is basilisk-rs's [`GaussMarkov`](basilisk_rs::gauss_markov),
//! given `3 senNoiseStd` for its 3-sigma convention, so it has upstream's
//! standard deviation but not upstream's random draws.
//!
//! Writes `examples/output/scenarios/scenarioTAM<orbitCase><planetCase><bias><bounds>.csv`
//! with `r_BN_N_m_*`, `magField_N_T_*` and `tam_S_T_*`.

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

use attitude::{bit, sampling_period_nanos, scenario_recorder};
use basilisk_rs::Module;
use basilisk_rs::messages::{MagneticFieldMsg, Output, SpacecraftStateMsg, TamSensorMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use magnetic_field::{
    OrbitCase, Planet, ScenarioOrbit, TASK_PERIOD_NANOS, centered_dipole, tam, wmm, wmm_epoch,
};
use message_log::MessageLog;

/// Upstream `TAM.maxOutput = 3.5e-4`, `TAM.minOutput = -3.5e-4` \[T\].
const TAM_BOUND_T: f64 = 3.5e-4;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    orbit_case: OrbitCase,
    planet_case: Planet,
    use_bias: bool,
    use_bounds: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(
        orbit_case: OrbitCase,
        planet_case: Planet,
        use_bias: bool,
        use_bounds: bool,
    ) -> Self {
        Self {
            orbit_case,
            planet_case,
            use_bias,
            use_bounds,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioTAM{}{}{}{}",
            self.orbit_case.name(),
            self.planet_case.name(),
            bit(self.use_bias),
            bit(self.use_bounds),
        )
    }
}

/// The parametrizations of upstream `test_scenarioTAM.py`.
const TEST_PARAMETRIZATIONS: [Config; 4] = [
    Config::new(OrbitCase::Circular, Planet::Earth, false, false),
    Config::new(OrbitCase::Elliptical, Planet::Earth, true, false),
    Config::new(OrbitCase::Circular, Planet::Jupiter, false, true),
    Config::new(OrbitCase::Elliptical, Planet::Jupiter, false, false),
];

/// The recorded histories (upstream `magLog`, `tamLog`, `dataLog`).
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    field: MessageLog<MagneticFieldMsg>,
    tam: MessageLog<TamSensorMsg>,
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let orbit = ScenarioOrbit::new(config.planet_case, config.orbit_case);
    let mut spacecraft = orbit.spacecraft(TASK_PERIOD_NANOS);
    config.planet_case.add_to(&mut spacecraft);

    // Upstream's `magModule`: WMM about Earth, the centered dipole about
    // Jupiter.
    let mut wmm_field = wmm(config.orbit_case);
    let mut dipole = centered_dipole(Planet::Jupiter.centered_dipole("CenteredDipole"));
    // Ports are shared handles: connecting the clone connects the module.
    let (field_output, mut field_state_input): (Output<MagneticFieldMsg>, _) =
        match config.planet_case {
            Planet::Earth => (
                wmm_field.output_magnetic_field_msg.clone(),
                wmm_field.input_state_msg.clone(),
            ),
            Planet::Jupiter => (
                dipole.output_magnetic_field_msg.clone(),
                dipole.input_state_msg.clone(),
            ),
        };
    let magnetic_field: &mut dyn Module = match config.planet_case {
        Planet::Earth => &mut wmm_field,
        Planet::Jupiter => &mut dipole,
    };
    let mut magnetometer = tam(
        "TAM_sensor",
        config.use_bias,
        config.use_bounds.then_some(TAM_BOUND_T),
    );

    let duration = orbit.duration_nanos();
    let sample_period = sampling_period_nanos(duration, TASK_PERIOD_NANOS, 100);
    let mut field_log = MessageLog::<MagneticFieldMsg>::default();
    let mut tam_log = MessageLog::<TamSensorMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(wmm_epoch(), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut field_state_input,
        &spacecraft.state_out => &mut magnetometer.input_state_msg,
        &field_output => &mut magnetometer.input_magnetic_field_msg,
        &field_output => &mut field_log.input,
        &magnetometer.output_tam_msg => &mut tam_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
        &field_output => recorder.add_source::<MagneticFieldMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_inertial_t", "magField_N_T")),
        ),
        &magnetometer.output_tam_msg => recorder.add_source::<TamSensorMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_sensor_t", "tam_S_T")),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "magnetic_field" => magnetic_field, period=TASK_PERIOD_NANOS, priority=90;
        "tam" => &mut magnetometer, period=TASK_PERIOD_NANOS, priority=80;
        "field_log" => &mut field_log, period=sample_period, priority=0;
        "tam_log" => &mut tam_log, period=sample_period, priority=0;
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
        tam: tam_log,
        state: state_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_tam [--orbit-case circular|elliptical] [--planet-case Earth|Jupiter] \
         [--[no-]bias] [--[no-]bounds] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.value("orbit-case", OrbitCase::Elliptical),
            cli.value("planet-case", Planet::Jupiter),
            cli.flag("bias", false),
            cli.flag("bounds", false),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} ({} samples)",
            results.csv_path.expect("CSV written").display(),
            results.tam.values.len(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioTAM.py`: five TAM samples, evenly spaced over
    //! the orbit (`tamData[::int(len / 4)]`), against hard-coded truth with
    //! `compareArray`'s absolute accuracy of 1e-3 T. Plus the sensor model
    //! itself: the reading is the field (the body and sensor frames are the
    //! inertial frame here) plus the bias, within five sigma of the 100 nT
    //! noise, saturated at the bounds.

    use nalgebra::Vector3;

    use super::*;
    use magnetic_field::{TAM_BIAS_T, TAM_NOISE_STD_T};

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// Upstream `trueMag`, in `TEST_PARAMETRIZATIONS` order.
    const TRUE_MAG_T: [[[f64; 3]; 5]; 4] = [
        [
            [
                -9.8909711711167910e-06,
                -1.5750678254527107e-05,
                -3.9770940590502404e-05,
            ],
            [
                1.8137775834150782e-05,
                1.3753594278249663e-05,
                1.4627065911256415e-05,
            ],
            [
                -1.2438328043293678e-06,
                -2.3225636857449733e-05,
                -3.1154667860968480e-05,
            ],
            [
                2.0220127543110147e-05,
                1.4951978250984546e-05,
                7.0917680337198594e-06,
            ],
            [
                -1.3773943421777615e-05,
                -1.9421019773673322e-05,
                -3.6313467519953167e-05,
            ],
        ],
        [
            [
                -2.9321026358524342e-06,
                -5.8651112860140206e-06,
                -1.3646243717429426e-05,
            ],
            [
                0.0000000000000000e+00,
                0.0000000000000000e+00,
                -9.9999999999999995e-07,
            ],
            [
                0.0000000000000000e+00,
                0.0000000000000000e+00,
                -9.9999999999999995e-07,
            ],
            [
                -1.3642616233976047e-06,
                -2.5553818742773536e-06,
                -1.3674037556015926e-06,
            ],
            [
                0.0000000000000000e+00,
                0.0000000000000000e+00,
                -9.9999999999999995e-07,
            ],
        ],
        [
            [
                -1.6819836450589402e-04,
                -2.2784371297700405e-04,
                -3.5000000000000000e-04,
            ],
            [
                1.9468153473841567e-04,
                2.7464566890817013e-04,
                1.9285361028600469e-04,
            ],
            [
                -2.1437405607225573e-04,
                -2.7189272324396394e-04,
                -3.5000000000000000e-04,
            ],
            [
                2.3757943503534954e-04,
                3.1431522854130238e-04,
                1.2955048838291110e-04,
            ],
            [
                -2.5365042123908573e-04,
                -3.0686495730002382e-04,
                -3.5000000000000000e-04,
            ],
        ],
        [
            [
                -5.4118082036659928e-05,
                -7.3309064488523122e-05,
                -1.7481547130195689e-04,
            ],
            [
                1.2443851358473241e-05,
                1.4960617668135152e-05,
                -1.8639105517436789e-06,
            ],
            [
                4.9056456907319987e-07,
                2.9201251508677668e-06,
                1.0222789452821487e-05,
            ],
            [
                -3.0771050824857636e-05,
                -3.2476159775524664e-05,
                -8.3194247838253931e-06,
            ],
            [
                -1.2553066020901707e-04,
                -1.4753925529969626e-04,
                -1.6322348568320908e-04,
            ],
        ],
    ];

    /// Upstream `compareArray(trueMag, dataMagRed, 1e-3, ...)`.
    #[test]
    fn the_tam_matches_upstream_truth() {
        for (config, truth) in TEST_PARAMETRIZATIONS.iter().zip(TRUE_MAG_T) {
            let results = quiet(*config);
            let skip = results.tam.values.len() / 4;
            let reduced: Vec<Vector3<f64>> = results
                .tam
                .values
                .iter()
                .step_by(skip)
                .map(|sample| sample.magnetic_field_sensor_t)
                .collect();
            assert_eq!(reduced.len(), truth.len(), "{config:?}");
            for (sample, expected) in reduced.iter().zip(truth) {
                let error = (sample - Vector3::from(expected)).abs().max();
                assert!(error < 1e-3, "{config:?}: {sample} vs {expected:?}");
            }
        }
    }

    #[test]
    fn the_tam_reads_the_field_plus_bias_within_the_noise() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            let bias = if config.use_bias {
                Vector3::from(TAM_BIAS_T)
            } else {
                Vector3::zeros()
            };
            let mut saturated = 0;
            for (reading, field) in results.tam.values.iter().zip(&results.field.values) {
                let mut expected = field.magnetic_field_inertial_t + bias;
                if config.use_bounds {
                    saturated += expected
                        .iter()
                        .filter(|value| value.abs() > TAM_BOUND_T)
                        .count();
                    expected = expected.map(|value| value.clamp(-TAM_BOUND_T, TAM_BOUND_T));
                }
                let error = (reading.magnetic_field_sensor_t - expected).abs().max();
                assert!(error < 5.0 * TAM_NOISE_STD_T, "{config:?}: {error}");
            }
            if config.use_bounds {
                assert!(saturated > 0, "{config:?}: the bounds never act");
            }
        }
    }

    /// The elliptical Earth case leaves the WMM reach band: there the field
    /// is zero and the TAM reads its bias plus noise.
    #[test]
    fn outside_the_wmm_reach_the_tam_reads_its_bias() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        let outside: Vec<_> = results
            .tam
            .values
            .iter()
            .zip(&results.field.values)
            .filter(|(_, field)| field.magnetic_field_inertial_t == Vector3::zeros())
            .collect();
        assert!(outside.len() > 10, "{}", outside.len());
        for (reading, _) in outside {
            let error = (reading.magnetic_field_sensor_t - Vector3::from(TAM_BIAS_T))
                .abs()
                .max();
            assert!(error < 5.0 * TAM_NOISE_STD_T, "{error}");
        }
    }
}
