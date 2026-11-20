//! Inertial attitude pointing with an MRP feedback controller and an external
//! control torque: port of upstream `examples/scenarioAttitudeFeedback.py`.
//!
//! A 750 kg rigid spacecraft on a 10 000 km Earth orbit starts at
//! `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s` and
//! is driven to the inertial frame (`inertial3D`, `attTrackingError`) by
//! `mrpFeedback`, whose torque command an `extForceTorque` effector applies
//! directly. Every module runs on one 0.1 s task for 10 minutes.
//!
//! Upstream `run(show_plots, useUnmodeledTorque, useIntGain, useKnownTorque,
//! useCMsg)` maps to the flags `--unmodeled-torque` (a constant
//! `[0.25, -0.25, 0.1] N m` disturbance), `--int-gain` (`Ki = 0.0002`
//! instead of off), `--known-torque` (the disturbance fed forward) and
//! `--c-msg`; `--all` runs the five parametrizations of upstream's
//! `test_scenarioAttitudeFeedback.py`.
//!
//! `useCMsg` makes upstream route the torque command through a stand-alone C
//! message that `mrpFeedback` writes to (`CmdTorqueBodyMsg_C_addAuthor`).
//! basilisk-rs has one message type, so the counterpart is a stand-alone
//! [`Output`] handed to the controller as its output port: the module writes
//! the message, every reader subscribes to it, and the simulation is the same.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeFeedback<ut><ig><kt>.csv`
//! (upstream's figure suffix, `str(int(flag))` of the first three flags), with
//! `CMsg` appended for the stand-alone message variant.

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
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, UNMODELED_TORQUE_NM, add_point_mass_earth,
    att_tracking_error, bit, external_disturbance, hub_inertia_kg_m2, inertial_3d, mrp_feedback,
    sampling_period_nanos, scenario_recorder, spacecraft, vehicle_config,
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
    use_c_msg: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(
        use_unmodeled_torque: bool,
        use_int_gain: bool,
        use_known_torque: bool,
        use_c_msg: bool,
    ) -> Self {
        Self {
            use_unmodeled_torque,
            use_int_gain,
            use_known_torque,
            use_c_msg,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAttitudeFeedback{}{}{}{}",
            bit(self.use_unmodeled_torque),
            bit(self.use_int_gain),
            bit(self.use_known_torque),
            if self.use_c_msg { "CMsg" } else { "" },
        )
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedback.py`.
const TEST_PARAMETRIZATIONS: [Config; 5] = [
    Config::new(false, false, false, false),
    Config::new(true, false, false, false),
    Config::new(true, true, false, false),
    Config::new(true, false, true, false),
    Config::new(false, false, false, true),
];

/// The recorded histories, sampled as upstream's recorders
/// (`samplingTime(simulationTime, simulationTimeStep, 100)`).
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
    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.01));
    add_point_mass_earth(&mut spacecraft);

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

    // `useCMsg`: a stand-alone torque message the controller is made the
    // author of, and which the effector and the recorder subscribe to.
    let torque_msg = if config.use_c_msg {
        let stand_alone = Output::<BodyTorqueCommandMsg>::default();
        controller.cmd_torque_out_msg = stand_alone.clone();
        stand_alone
    } else {
        controller.cmd_torque_out_msg.clone()
    };

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation, &torque_msg => &mut external_torque.cmd_torque_in_msg);
    spacecraft.add_dynamic_effector(external_torque);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &torque_msg => &mut torque_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &torque_msg => recorder.add_source::<BodyTorqueCommandMsg>(CsvSourceConfig::columns(
            vector_columns("torque_request_body_nm", "torqueRequestBody_Nm"),
        )),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
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
        "scenario_attitude_feedback [--[no-]unmodeled-torque] [--[no-]int-gain] \
         [--[no-]known-torque] [--[no-]c-msg] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("unmodeled-torque", false),
            cli.flag("int-gain", false),
            cli.flag("known-torque", false),
            cli.flag("c-msg", false),
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
    //! Upstream `test_scenarioAttitudeFeedback.py` only checks that each
    //! parametrization runs. These tests run the same parametrizations and
    //! check the behavior the scenario documents.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[test]
    fn every_upstream_parametrization_runs_from_the_initial_attitude() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            // samplingTime(10 min, 0.1 s, 100) = 6 s: samples 0, 6, ..., 600 s.
            assert_eq!(results.guidance.values.len(), 101, "{config:?}");
            assert_eq!(
                *results.guidance.times_nanos.last().unwrap(),
                DURATION_NANOS
            );
            // The reference is the inertial frame, so the first error is
            // sigma_BN(0).
            let first = results.guidance.values[0].sigma_br;
            assert!((first - Vector3::new(0.1, 0.2, -0.3)).norm() < 1.0e-12);
        }
    }

    #[test]
    fn without_a_disturbance_the_attitude_error_settles_to_zero() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-4);
        assert!(results.guidance.last().omega_br_b_radps.norm() < 1.0e-5);
    }

    /// "the orientation error doesn't settle to zero, but rather converges to
    /// a non-zero offset proportional to the un-modeled torque": at rest,
    /// `-K sigma + L = 0`, so `sigma = L / K` and the control torque is `-L`.
    #[test]
    fn an_unmodeled_torque_leaves_an_offset_proportional_to_it() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        let disturbance = Vector3::from(UNMODELED_TORQUE_NM);
        let sigma = results.guidance.last().sigma_br;
        assert!((sigma - disturbance / 3.5).norm() < 1.0e-4, "{sigma}");
        let torque = results.torque.last().torque_request_body_nm;
        assert!((torque + disturbance).norm() < 1.0e-3, "{torque}");
    }

    /// "In this case the orientation error does settle to zero. The integral
    /// term changes the control torque to settle on a value that matches the
    /// un-modeled external torque."
    #[test]
    fn integral_feedback_removes_the_offset() {
        let without = quiet(TEST_PARAMETRIZATIONS[1])
            .guidance
            .last()
            .sigma_br
            .norm();
        let results = quiet(TEST_PARAMETRIZATIONS[2]);
        let sigma = results.guidance.last().sigma_br.norm();
        assert!(sigma < 0.1 * without, "{sigma} vs {without}");
        let torque = results.torque.last().torque_request_body_nm;
        assert!(
            (torque + Vector3::from(UNMODELED_TORQUE_NM)).norm() < 0.05,
            "{torque}"
        );
    }

    /// "the orientation error does settle to zero as the feed-forward term
    /// compensates for the external torque."
    #[test]
    fn the_known_torque_feed_forward_removes_the_offset() {
        let results = quiet(TEST_PARAMETRIZATIONS[3]);
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-4);
        let torque = results.torque.last().torque_request_body_nm;
        assert!(
            (torque + Vector3::from(UNMODELED_TORQUE_NM)).norm() < 1.0e-4,
            "{torque}"
        );
    }

    /// `useCMsg` changes only how the torque message is plumbed.
    #[test]
    fn the_stand_alone_message_variant_is_the_same_simulation() {
        let module_message = quiet(TEST_PARAMETRIZATIONS[0]);
        let stand_alone = quiet(TEST_PARAMETRIZATIONS[4]);
        for (left, right) in module_message
            .torque
            .values
            .iter()
            .zip(&stand_alone.torque.values)
        {
            assert_eq!(left.torque_request_body_nm, right.torque_request_body_nm);
        }
        assert_eq!(
            module_message.state.last().sigma_bn,
            stand_alone.state.last().sigma_bn
        );
    }
}
