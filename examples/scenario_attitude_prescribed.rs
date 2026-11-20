//! A prescribed spacecraft attitude: port of upstream
//! `examples/scenarioAttitudePrescribed.py`.
//!
//! The 750 kg hub of the attitude scenarios on a `a = 10 000 km`, `e = 0.1`
//! Earth orbit (point-mass Earth), with no attitude control: `hillPoint`'s
//! Hill-frame reference drives the spacecraft's `attRefInMsg`, which sets
//! the hub attitude and rate after every integration step. One 60 s task,
//! 3 hours, the navigation attitude sampled every step.
//!
//! Upstream `run(show_plots, useAltBodyFrame)` maps to `--alt-body-frame`:
//! `attRefCorrection` with `sigma_BcB = [0, 0, tan(π/8)]` (a 90° turn about
//! the third body axis) corrects the reference so that the corrected body
//! frame Bc, not B, follows the Hill frame. The script's own default is
//! `--no-alt-body-frame`; `--all` runs both cases of upstream's
//! `test_scenarioAttitudePrescribed.py`.
//!
//! Writes `examples/output/scenarios/scenarioAttitudePrescribed<alt>.csv`.

#[path = "support/att_scenario.rs"]
mod att_scenario;
#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use att_scenario::rigid_spacecraft;
use attitude::{
    HubState, bit, hill_point, hub_inertia_kg_m2, sampling_period_nanos, scenario_recorder,
};
use basilisk_rs::fsw_algorithms::att_guidance::att_ref_correction::{
    AttRefCorrection, AttRefCorrectionConfig,
};
use basilisk_rs::messages::{
    AttitudeReferenceMsg, NavigationAttitudeMsg, NavigationTranslationMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{elem2rv, vector_columns};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// Upstream `simulationTimeStep = macros.sec2nano(60)`.
const STEP_NANOS: u64 = 60_000_000_000;
/// Upstream `simulationTime = macros.min2nano(3 * 60.)`.
const DURATION_NANOS: u64 = 3 * 3_600_000_000_000;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_alt_body_frame: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_alt_body_frame: bool) -> Self {
        Self {
            use_alt_body_frame,
            write_csv: true,
        }
    }
}

/// The parametrizations of upstream `test_scenarioAttitudePrescribed.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// Upstream `attRefCor.sigma_BcB = [0, 0, tan(π/8)]`.
fn sigma_bcb() -> Vector3<f64> {
    Vector3::new(0.0, 0.0, (std::f64::consts::PI / 8.0).tan())
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    attitude: MessageLog<NavigationAttitudeMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    reference: MessageLog<AttitudeReferenceMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let (position_m, velocity_mps) = elem2rv(
        earth.mu_m3ps2(),
        10_000_000.0,
        0.1,
        33.3_f64.to_radians(),
        48.2_f64.to_radians(),
        347.8_f64.to_radians(),
        85.3_f64.to_radians(),
    );
    let initial = HubState {
        position_m,
        velocity_mps,
        sigma_bn: Vector3::zeros(),
        omega_bn_b_radps: Vector3::zeros(),
    };
    let mut spacecraft = rigid_spacecraft(hub_inertia_kg_m2(), &initial, STEP_NANOS);
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("one central body");

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = hill_point();
    let mut correction = AttRefCorrection::new(AttRefCorrectionConfig {
        name: "attRefCor".to_string(),
        sigma_bcb: sigma_bcb(),
    });

    let sample_period = sampling_period_nanos(DURATION_NANOS, STEP_NANOS, 100);
    let mut attitude_log = MessageLog::<NavigationAttitudeMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let mut reference_log = MessageLog::<AttitudeReferenceMsg>::default();
    let stem = format!(
        "scenarioAttitudePrescribed{}",
        bit(config.use_alt_body_frame)
    );
    let (mut recorder, csv_path) = scenario_recorder(&stem, config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    let reference = if config.use_alt_body_frame {
        connect!(&simulation, &guidance.att_ref_out_msg => &mut correction.att_ref_in_msg);
        correction.att_ref_out_msg.clone()
    } else {
        guidance.att_ref_out_msg.clone()
    };
    connect!(&simulation,
        &reference => &mut spacecraft.att_ref_in,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &reference => &mut reference_log.input,
        &navigation.attitude_out_msg => recorder.add_source::<NavigationAttitudeMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
        ),
        &navigation.translation_out_msg => recorder.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_bn_n_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_bn_n_mps", "v_BN_N_m_per_s")),
            ),
        ),
    );
    // Upstream's task order: spacecraft, navigation, hillPoint, attRefCor.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "navigation" => &mut navigation, period=STEP_NANOS, priority=90;
        "hillPoint" => &mut guidance, period=STEP_NANOS, priority=80;
        "attitude_log" => &mut attitude_log, period=sample_period, priority=0;
        "translation_log" => &mut translation_log, period=sample_period, priority=0;
        "reference_log" => &mut reference_log, period=sample_period, priority=0;
    }
    if config.use_alt_body_frame {
        simulation.add_module("attRefCor", &mut correction, STEP_NANOS, 70);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        attitude: attitude_log,
        translation: translation_log,
        reference: reference_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_prescribed [--[no-]alt-body-frame] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, False)`.
        vec![Config::new(cli.flag("alt-body-frame", false))]
    };
    cli.finish();
    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final sigma_BN = {:.4?})",
            results.csv_path.expect("CSV written").display(),
            results.attitude.last().sigma_bn.as_slice()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudePrescribed.py` only checks that both
    //! parametrizations run. These tests check what the scenario documents:
    //! the (corrected) body frame follows the Hill frame.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
    use nalgebra::Matrix3;

    use super::*;

    /// The Hill frame `[HN]` of an inertial position and velocity: rows
    /// `î_r`, `î_θ`, `î_h`.
    fn hill_dcm(position: Vector3<f64>, velocity: Vector3<f64>) -> Matrix3<f64> {
        let i_r = position.normalize();
        let i_h = position.cross(&velocity).normalize();
        let i_theta = i_h.cross(&i_r);
        Matrix3::from_rows(&[i_r.transpose(), i_theta.transpose(), i_h.transpose()])
    }

    /// The hub takes the reference written at the previous step (the
    /// reference modules run after the spacecraft, as upstream), so after
    /// the first step `[BN](t) = [B Bc] [HN](t - Δt)`, with `[B Bc] = I`
    /// without the correction. The attitude starts at zero.
    #[test]
    fn the_body_frame_follows_the_hill_frame() {
        for config in TEST_PARAMETRIZATIONS {
            let results = run(Config {
                write_csv: false,
                ..config
            });
            assert_eq!(results.attitude.values.len(), 181);
            assert_eq!(results.attitude.values[0].sigma_bn, Vector3::zeros());
            let b_bc = if config.use_alt_body_frame {
                mrp_to_dcm(sigma_bcb()).transpose()
            } else {
                Matrix3::identity()
            };
            for step in 1..results.attitude.values.len() {
                let previous = &results.translation.values[step - 1];
                let expected =
                    b_bc * hill_dcm(previous.position_bn_n_m, previous.velocity_bn_n_mps);
                let actual = mrp_to_dcm(results.attitude.values[step].sigma_bn);
                assert!((actual - expected).norm() < 1e-12, "{config:?} step {step}");
                assert_eq!(
                    results.attitude.values[step].sigma_bn,
                    results.reference.values[step - 1].sigma_rn
                );
            }
        }
    }

    /// The corrected case is the uncorrected one turned by 90° about the
    /// third body axis.
    #[test]
    fn the_correction_turns_the_body_by_ninety_degrees() {
        let plain = run(Config::new(false).quiet());
        let corrected = run(Config::new(true).quiet());
        let last = |results: &Results| mrp_to_dcm(results.attitude.last().sigma_bn);
        let relative = last(&corrected) * last(&plain).transpose();
        // [B Bc]: B is Bc turned by -90° about the third axis.
        let turn = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        assert!((relative - turn).norm() < 1e-12, "{relative}");
    }

    impl Config {
        fn quiet(self) -> Self {
            Self {
                write_csv: false,
                ..self
            }
        }
    }
}
