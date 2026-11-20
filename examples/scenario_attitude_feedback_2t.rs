//! Inertial attitude pointing with the dynamics and the flight software on
//! separate task rates: port of upstream
//! `examples/scenarioAttitudeFeedback2T.py`.
//!
//! The [`scenarioAttitudeFeedback`](scenario_attitude_feedback.rs) setup
//! split over two processes: the spacecraft, `extForceTorque` and `simpleNav`
//! on a 0.1 s dynamics task, and `inertial3D`, `attTrackingError` and
//! `mrpFeedback` on a 0.5 s flight-software task that runs after the dynamics
//! at shared ticks (upstream's process order; here a lower priority). The
//! control torque is therefore held for 0.5 s between updates. 10 minutes.
//!
//! Upstream `run(show_plots, useUnmodeledTorque, useIntGain)` maps to
//! `--unmodeled-torque` and `--int-gain`; `--all` runs the three
//! parametrizations of upstream's `test_scenarioAttitudeFeedback2T.py`.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeFeedback2T<ut><ig>.csv`.

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

/// Upstream `fswTimeStep = macros.sec2nano(0.5)`.
const FSW_PERIOD_NANOS: u64 = 500_000_000;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_unmodeled_torque: bool,
    use_int_gain: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_unmodeled_torque: bool, use_int_gain: bool) -> Self {
        Self {
            use_unmodeled_torque,
            use_int_gain,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAttitudeFeedback2T{}{}",
            bit(self.use_unmodeled_torque),
            bit(self.use_int_gain),
        )
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedback2T.py`.
const TEST_PARAMETRIZATIONS: [Config; 3] = [
    Config::new(false, false),
    Config::new(true, false),
    Config::new(true, true),
];

/// The recorded histories (every 6 s on both tasks) and the number of
/// updates each task ran.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    state: MessageLog<SpacecraftStateMsg>,
    dynamics_updates: u64,
    fsw_updates: u64,
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
    let ki = if config.use_int_gain { 0.0002 } else { -1.0 };
    let mut controller = mrp_feedback(3.5, ki, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    // Upstream samples the dynamics recorder at samplingTime(T, 0.1 s, 100)
    // and the FSW ones at samplingTime(T, 0.5 s, 100): both 6 s.
    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    assert_eq!(
        sample_period,
        sampling_period_nanos(DURATION_NANOS, FSW_PERIOD_NANOS, 100)
    );
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
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("sigma_bn", "sigma_BN")),
            ),
        ),
    );
    schedule! { simulation,
        // Dynamics process.
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        // Flight-software process, after the dynamics at shared ticks.
        "inertial_reference" => &mut inertial_reference, period=FSW_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=FSW_PERIOD_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=FSW_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    let updates = |name: &str| {
        simulation
            .module_timings()
            .into_iter()
            .find(|timing| timing.name == name)
            .expect("scheduled module")
            .num_updates
    };
    let dynamics_updates = updates("spacecraft");
    let fsw_updates = updates("mrp_feedback");
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        state: state_log,
        dynamics_updates,
        fsw_updates,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_attitude_feedback_2t [--[no-]unmodeled-torque] [--[no-]int-gain] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("unmodeled-torque", false),
            cli.flag("int-gain", false),
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
    //! Upstream `test_scenarioAttitudeFeedback2T.py` only checks that each
    //! parametrization runs. These tests run the same parametrizations and
    //! check the two-rate schedule and the closed-loop behavior.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[test]
    fn the_tasks_run_at_their_own_rates() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_eq!(results.dynamics_updates, 6_001, "{config:?}");
            assert_eq!(results.fsw_updates, 1_201, "{config:?}");
            assert_eq!(results.guidance.values.len(), 101);
            assert_eq!(
                results.state.values[0].sigma_bn,
                Vector3::new(0.1, 0.2, -0.3)
            );
        }
    }

    #[test]
    fn the_attitude_settles_unless_a_torque_is_left_uncompensated() {
        let settled = quiet(TEST_PARAMETRIZATIONS[0]);
        assert!(settled.guidance.last().sigma_br.norm() < 1.0e-4);

        // Proportional feedback alone holds sigma = L / K against the torque.
        let disturbance = Vector3::from(UNMODELED_TORQUE_NM);
        let offset = quiet(TEST_PARAMETRIZATIONS[1]);
        let sigma = offset.guidance.last().sigma_br;
        assert!((sigma - disturbance / 3.5).norm() < 1.0e-4, "{sigma}");
        let torque = offset.torque.last().torque_request_body_nm;
        assert!((torque + disturbance).norm() < 1.0e-3, "{torque}");

        let integral = quiet(TEST_PARAMETRIZATIONS[2]);
        assert!(integral.guidance.last().sigma_br.norm() < 0.1 * sigma.norm());
    }
}
