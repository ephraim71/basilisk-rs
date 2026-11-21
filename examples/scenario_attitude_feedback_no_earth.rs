//! Inertial attitude pointing in deep space: port of upstream
//! `examples/scenarioAttitudeFeedbackNoEarth.py`.
//!
//! [`scenarioAttitudeFeedback`](scenario_attitude_feedback.rs) without the
//! Earth: no gravity body and no orbit, so the spacecraft sits at the inertial
//! origin while `inertial3D`, `attTrackingError` and `mrpFeedback` bring it to
//! the inertial frame through an `extForceTorque` effector. One 0.1 s task,
//! 10 minutes.
//!
//! Upstream `run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque)`
//! maps to `--unmodeled-torque`, `--int-gain` and `--known-torque`; `--all`
//! runs the four parametrizations of upstream's
//! `test_scenarioAttitudeFeedbackNoEarth.py`.
//!
//! Writes
//! `examples/output/scenarios/scenarioAttitudeFeedbackNoEarth<ut><ig><kt>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, UNMODELED_TORQUE_NM, att_tracking_error, bit,
    external_disturbance, hub_inertia_kg_m2, inertial_3d, mrp_feedback, sampling_period_nanos,
    scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Output, SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_unmodeled_torque: bool,
    use_int_gain: bool,
    use_known_torque: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_unmodeled_torque: bool, use_int_gain: bool, use_known_torque: bool) -> Self {
        Self {
            use_unmodeled_torque,
            use_int_gain,
            use_known_torque,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAttitudeFeedbackNoEarth{}{}{}",
            bit(self.use_unmodeled_torque),
            bit(self.use_int_gain),
            bit(self.use_known_torque),
        )
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedbackNoEarth.py`.
const TEST_PARAMETRIZATIONS: [Config; 4] = [
    Config::new(false, false, false),
    Config::new(true, false, false),
    Config::new(true, true, false),
    Config::new(true, false, true),
];

/// The recorded histories, sampled every
/// `samplingTime(simulationTime, simulationTimeStep, 100)`.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    // No gravity body: upstream leaves r_CN_NInit and v_CN_NInit at zero.
    let mut spacecraft = spacecraft(inertia, &HubState::in_deep_space());

    let disturbance_nm = if config.use_unmodeled_torque {
        Vector3::from(UNMODELED_TORQUE_NM)
    } else {
        Vector3::zeros()
    };
    let mut external_torque = external_disturbance(disturbance_nm);
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let known_torque_nm = if config.use_known_torque {
        Vector3::from(UNMODELED_TORQUE_NM)
    } else {
        Vector3::zeros()
    };
    let ki = if config.use_int_gain { 0.0002 } else { -1.0 };
    let mut controller = mrp_feedback(3.5, ki, 30.0, known_torque_nm);
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
    );
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "torque_request_body_nm",
                "torqueRequestBody_Nm",
            )),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial_reference" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        state: state_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_attitude_feedback_no_earth [--[no-]unmodeled-torque] [--[no-]int-gain] \
         [--[no-]known-torque] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("unmodeled-torque", false),
            cli.flag("int-gain", false),
            cli.flag("known-torque", false),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e})",
            results.csv_path.expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeFeedbackNoEarth.py` only checks that
    //! each parametrization runs. These tests run the same parametrizations
    //! and check the behavior the scenario shows.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[test]
    fn every_upstream_parametrization_runs_and_stays_at_the_origin() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_eq!(results.guidance.values.len(), 101, "{config:?}");
            // Without gravity or a force the hub never leaves the origin.
            for state in &results.state.values {
                assert_eq!(state.position_m, Vector3::zeros());
                assert_eq!(state.velocity_mps, Vector3::zeros());
            }
        }
    }

    #[test]
    fn the_attitude_settles_unless_a_torque_is_left_uncompensated() {
        let disturbance = Vector3::from(UNMODELED_TORQUE_NM);
        let settled = quiet(TEST_PARAMETRIZATIONS[0]);
        assert!(settled.guidance.last().sigma_br.norm() < 1.0e-4);

        // Proportional feedback alone holds sigma = L / K against the torque.
        let offset = quiet(TEST_PARAMETRIZATIONS[1]);
        let sigma = offset.guidance.last().sigma_br;
        assert!((sigma - disturbance / 3.5).norm() < 1.0e-4, "{sigma}");

        let integral = quiet(TEST_PARAMETRIZATIONS[2]);
        assert!(integral.guidance.last().sigma_br.norm() < 0.1 * sigma.norm());

        let feed_forward = quiet(TEST_PARAMETRIZATIONS[3]);
        assert!(feed_forward.guidance.last().sigma_br.norm() < 1.0e-4);
        let torque = feed_forward.torque.last().torque_request_body_nm;
        assert!((torque + disturbance).norm() < 1.0e-4, "{torque}");
    }
}
