//! Magnetometers on a centered-dipole and a WMM field: port of upstream
//! `examples/scenarioTAMcomparison.py`.
//!
//! The Earth orbit of `scenarioTAM` (default hub, one orbit of the
//! `i = 85 deg` orbit with periapsis at `1.1 REQ`, circular or with apoapsis
//! at `3.5 REQ`) with two field models side by side: the 2020 IGRF centered
//! dipole (`magModule1`), read by `TAM1`, and WMM at
//! `2019 June 27, 10:23:0.0 (UTC)` (`magModule3`), read by `TAM2`. In the
//! elliptical case the dipole only acts above `1.3 REQ`, a half-strength
//! dipole (`magModule2`, read by the dummy `TAM3` with `TAM1`'s settings)
//! below it, and WMM only between 10 000 km and 20 000 km. Every TAM has
//! 100 nT white noise; each of `TAM1` and `TAM2` optionally a
//! `[0, 0, -1] uT` bias and a `±25 uT` saturation. One 10 s task, histories
//! sampled every `samplingTime(P, 10 s, 100)`.
//!
//! Upstream `run(show_plots, orbitCase, useBias1, useBias2, useBounds1,
//! useBounds2)` maps to `--orbit-case`, `--[no-]bias1`, `--[no-]bias2`,
//! `--[no-]bounds1` and `--[no-]bounds2`; `--all` runs the four
//! parametrizations of upstream's `test_scenarioTAMcomparison.py`. As in
//! `scenario_tam`, the noise has upstream's standard deviation, not its
//! draws.
//!
//! Writes `examples/output/scenarios/scenarioTAMcomparison<orbitCase><bias1><bounds1><bias2><bounds2>.csv`
//! (upstream's figure suffix, with `True`/`False`): `r_BN_N_m_*`,
//! `magField1_N_T_*`, `magField3_N_T_*`, `tam1_S_T_*`, `tam2_S_T_*` and, in
//! the elliptical case, `magField2_N_T_*` and `tam3_S_T_*`.

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
use basilisk_rs::messages::{MagneticFieldMsg, Output, SpacecraftStateMsg, TamSensorMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use magnetic_field::{
    OrbitCase, Planet, ScenarioOrbit, TASK_PERIOD_NANOS, centered_dipole, inner_dipole_reach_m,
    inner_earth_dipole, tam, wmm, wmm_epoch,
};
use message_log::MessageLog;

/// Upstream `TAM.maxOutput = 2.5E-5`, `TAM.minOutput = -2.5E-5` \[T\].
const TAM_BOUND_T: f64 = 2.5e-5;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    orbit_case: OrbitCase,
    use_bias1: bool,
    use_bias2: bool,
    use_bounds1: bool,
    use_bounds2: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(
        orbit_case: OrbitCase,
        use_bias1: bool,
        use_bias2: bool,
        use_bounds1: bool,
        use_bounds2: bool,
    ) -> Self {
        Self {
            orbit_case,
            use_bias1,
            use_bias2,
            use_bounds1,
            use_bounds2,
            write_csv: true,
        }
    }

    fn elliptical(&self) -> bool {
        self.orbit_case == OrbitCase::Elliptical
    }

    fn file_stem(&self) -> String {
        let text = |flag: bool| if flag { "True" } else { "False" };
        format!(
            "scenarioTAMcomparison{}{}{}{}{}",
            self.orbit_case.name(),
            text(self.use_bias1),
            text(self.use_bounds1),
            text(self.use_bias2),
            text(self.use_bounds2),
        )
    }
}

/// The parametrizations of upstream `test_scenarioTAMcomparison.py`.
const TEST_PARAMETRIZATIONS: [Config; 4] = [
    Config::new(OrbitCase::Circular, false, false, true, true),
    Config::new(OrbitCase::Elliptical, true, true, false, false),
    Config::new(OrbitCase::Elliptical, false, false, false, false),
    Config::new(OrbitCase::Circular, false, false, true, false),
];

/// One field model and the magnetometer reading it.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Channel {
    field: MessageLog<MagneticFieldMsg>,
    tam: MessageLog<TamSensorMsg>,
    use_bias: bool,
    use_bounds: bool,
}

/// The recorded histories: the dipole (`TAM1`), WMM (`TAM2`) and, in the
/// elliptical case, the inner dipole (`TAM3`).
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    dipole: Channel,
    wmm: Channel,
    inner_dipole: Option<Channel>,
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let orbit = ScenarioOrbit::new(Planet::Earth, config.orbit_case);
    let mut spacecraft = orbit.spacecraft(TASK_PERIOD_NANOS);
    Planet::Earth.add_to(&mut spacecraft);

    let mut dipole_config = Planet::Earth.centered_dipole("CenteredDipole");
    if config.elliptical() {
        dipole_config.env_min_reach_m = Some(inner_dipole_reach_m());
    }
    let mut dipole = centered_dipole(dipole_config);
    let mut inner_dipole = centered_dipole(inner_earth_dipole("CenteredDipole2"));
    let mut wmm_field = wmm(config.orbit_case);
    let bound = |use_bounds: bool| use_bounds.then_some(TAM_BOUND_T);
    let mut tam1 = tam("TAM1_sensor", config.use_bias1, bound(config.use_bounds1));
    let mut tam2 = tam("TAM2_sensor", config.use_bias2, bound(config.use_bounds2));
    let mut tam3 = tam("TAM3_sensor", config.use_bias1, bound(config.use_bounds1));

    let duration = orbit.duration_nanos();
    let sample_period = sampling_period_nanos(duration, TASK_PERIOD_NANOS, 100);
    let mut logs: [(MessageLog<MagneticFieldMsg>, MessageLog<TamSensorMsg>); 3] =
        std::array::from_fn(|_| (MessageLog::default(), MessageLog::default()));
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(wmm_epoch(), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut dipole.input_state_msg,
        &spacecraft.state_out => &mut inner_dipole.input_state_msg,
        &spacecraft.state_out => &mut wmm_field.input_state_msg,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
    );
    let channels: [(&Output<MagneticFieldMsg>, &mut _, &str, &str); 3] = [
        (
            &dipole.output_magnetic_field_msg,
            &mut tam1,
            "magField1_N_T",
            "tam1_S_T",
        ),
        (
            &wmm_field.output_magnetic_field_msg,
            &mut tam2,
            "magField3_N_T",
            "tam2_S_T",
        ),
        (
            &inner_dipole.output_magnetic_field_msg,
            &mut tam3,
            "magField2_N_T",
            "tam3_S_T",
        ),
    ];
    for (index, ((field, magnetometer, field_column, tam_column), (field_log, tam_log))) in
        channels.into_iter().zip(&mut logs).enumerate()
    {
        connect!(&simulation,
            &spacecraft.state_out => &mut magnetometer.input_state_msg,
            field => &mut magnetometer.input_magnetic_field_msg,
            field => &mut field_log.input,
            &magnetometer.output_tam_msg => &mut tam_log.input,
        );
        if index < 2 || config.elliptical() {
            connect!(&simulation,
                field => recorder.add_source::<MagneticFieldMsg>(CsvSourceConfig::columns(
                    vector_columns("magnetic_field_inertial_t", field_column),
                )),
                &magnetometer.output_tam_msg => recorder.add_source::<TamSensorMsg>(
                    CsvSourceConfig::columns(vector_columns("magnetic_field_sensor_t", tam_column)),
                ),
            );
        }
    }
    let [
        (dipole_field_log, dipole_tam_log),
        (wmm_field_log, wmm_tam_log),
        (inner_field_log, inner_tam_log),
    ] = &mut logs;
    // Upstream's task order: the spacecraft, the inner dipole, the dipole,
    // WMM, then TAM1, TAM3 and TAM2.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "centered_dipole" => &mut dipole, period=TASK_PERIOD_NANOS, priority=80;
        "wmm" => &mut wmm_field, period=TASK_PERIOD_NANOS, priority=70;
        "tam1" => &mut tam1, period=TASK_PERIOD_NANOS, priority=60;
        "tam2" => &mut tam2, period=TASK_PERIOD_NANOS, priority=40;
        "dipole_field_log" => dipole_field_log, period=sample_period, priority=0;
        "dipole_tam_log" => dipole_tam_log, period=sample_period, priority=0;
        "wmm_field_log" => wmm_field_log, period=sample_period, priority=0;
        "wmm_tam_log" => wmm_tam_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
    }
    if config.elliptical() {
        schedule! { simulation,
            "centered_dipole_2" => &mut inner_dipole, period=TASK_PERIOD_NANOS, priority=90;
            "tam3" => &mut tam3, period=TASK_PERIOD_NANOS, priority=50;
            "inner_field_log" => inner_field_log, period=sample_period, priority=0;
            "inner_tam_log" => inner_tam_log, period=sample_period, priority=0;
        }
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(duration);
    drop(simulation);
    drop(recorder);

    let [
        (dipole_field, dipole_tam),
        (wmm_field, wmm_tam),
        (inner_field, inner_tam),
    ] = logs;
    Results {
        dipole: Channel {
            field: dipole_field,
            tam: dipole_tam,
            use_bias: config.use_bias1,
            use_bounds: config.use_bounds1,
        },
        wmm: Channel {
            field: wmm_field,
            tam: wmm_tam,
            use_bias: config.use_bias2,
            use_bounds: config.use_bounds2,
        },
        inner_dipole: config.elliptical().then_some(Channel {
            field: inner_field,
            tam: inner_tam,
            use_bias: config.use_bias1,
            use_bounds: config.use_bounds1,
        }),
        state: state_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_tam_comparison [--orbit-case circular|elliptical] [--[no-]bias1] \
         [--[no-]bias2] [--[no-]bounds1] [--[no-]bounds2] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.value("orbit-case", OrbitCase::Circular),
            cli.flag("bias1", false),
            cli.flag("bias2", false),
            cli.flag("bounds1", true),
            cli.flag("bounds2", false),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} ({} samples)",
            results.csv_path.expect("CSV written").display(),
            results.state.values.len(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioTAMcomparison.py` only checks that each
    //! parametrization runs. These tests run the same parametrizations and
    //! check each magnetometer against the field it reads (plus its bias,
    //! within five sigma of the noise, saturated at its bounds), the reach
    //! split of the elliptical case, and that the dipole and WMM fields agree
    //! to the extent a dipole approximates the geomagnetic field.

    use nalgebra::Vector3;

    use super::*;
    use magnetic_field::{TAM_BIAS_T, TAM_NOISE_STD_T};

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[track_caller]
    fn assert_reads_its_field(channel: &Channel) {
        let bias = if channel.use_bias {
            Vector3::from(TAM_BIAS_T)
        } else {
            Vector3::zeros()
        };
        for (reading, field) in channel.tam.values.iter().zip(&channel.field.values) {
            let mut expected = field.magnetic_field_inertial_t + bias;
            if channel.use_bounds {
                expected = expected.map(|value| value.clamp(-TAM_BOUND_T, TAM_BOUND_T));
            }
            let error = (reading.magnetic_field_sensor_t - expected).abs().max();
            assert!(error < 5.0 * TAM_NOISE_STD_T, "{error}");
        }
    }

    #[test]
    fn every_magnetometer_reads_its_field_model() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_reads_its_field(&results.dipole);
            assert_reads_its_field(&results.wmm);
            if let Some(inner) = &results.inner_dipole {
                assert_reads_its_field(inner);
            }
            if config.use_bounds1 {
                let saturated = results.dipole.tam.values.iter().any(|reading| {
                    reading
                        .magnetic_field_sensor_t
                        .iter()
                        .any(|value| value.abs() == TAM_BOUND_T)
                });
                assert!(saturated, "{config:?}: the 25 uT bound never acts");
            }
        }
    }

    #[test]
    fn the_elliptical_dipoles_split_the_orbit_at_1_3_req() {
        let results = quiet(TEST_PARAMETRIZATIONS[2]);
        let inner = results.inner_dipole.as_ref().expect("inner dipole");
        for ((outer, inner), state) in results
            .dipole
            .field
            .values
            .iter()
            .zip(&inner.field.values)
            .zip(&results.state.values)
        {
            let below = state.position_m.norm() < inner_dipole_reach_m();
            assert_eq!(outer.magnetic_field_inertial_t == Vector3::zeros(), below);
            assert_eq!(inner.magnetic_field_inertial_t != Vector3::zeros(), below);
        }
    }

    /// On the circular orbit both models are on all the time. At 640 km the
    /// non-dipole terms still matter (the largest magnitude difference,
    /// about 40%, is over the South Atlantic anomaly), but the dipole
    /// (upstream's coefficients) is within 20% of WMM in magnitude on average
    /// and within 20 deg of it in direction throughout.
    #[test]
    fn the_dipole_approximates_wmm() {
        let results = quiet(TEST_PARAMETRIZATIONS[3]);
        let mut magnitude_errors = Vec::new();
        for (dipole, wmm) in results
            .dipole
            .field
            .values
            .iter()
            .zip(&results.wmm.field.values)
        {
            let dipole = dipole.magnetic_field_inertial_t;
            let wmm = wmm.magnetic_field_inertial_t;
            magnitude_errors.push((dipole.norm() - wmm.norm()).abs() / wmm.norm());
            assert!(dipole.angle(&wmm).to_degrees() < 20.0);
        }
        let mean = magnitude_errors.iter().sum::<f64>() / magnitude_errors.len() as f64;
        assert!(mean < 0.2, "{mean}");
    }
}
