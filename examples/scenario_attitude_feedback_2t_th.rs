//! Inertial attitude pointing with thrusters, dynamics and flight software on
//! separate task rates: port of upstream
//! `examples/scenarioAttitudeFeedback2T_TH.py`.
//!
//! The spacecraft of [`scenarioAttitudeFeedback2T`](scenario_attitude_feedback_2t.rs),
//! with the constant `[0.25, -0.25, 0.1] N m` disturbance always on, is
//! controlled by thrusters instead of a torque effector: `mrpFeedback`
//! (`K = 35`, `Ki = 0.0002`, `P = 300`) feeds `thrForceMapping`, whose forces
//! `thrFiringSchmitt` (0.002 s minimum fire time, levels 0.75 / 0.25) turns
//! into on-times for one `ThrusterDynamicEffector` per thruster. Dynamics
//! (spacecraft, disturbance, `simpleNav`, thrusters) run every 0.1 s and the
//! flight software every 0.5 s, after the dynamics. 10 minutes.
//!
//! Upstream `run(show_plots, useDVThrusters)` maps to `--dv-thrusters`:
//! off, eight on-pulsing 0.9 N `MOOG_Monarc_1` attitude thrusters controlling
//! all three axes; on, six off-pulsing 22 N `MOOG_Monarc_22_6` orbit
//! thrusters along `+b3`, which can only control the first two axes. `--all`
//! runs both, as upstream's `test_scenarioAttitudeFeedback2T_TH.py` does.
//!
//! basilisk-rs's thrusters take one command each (upstream's
//! `THRArrayOnTimeCmdMsg` is one message for the cluster): the Schmitt
//! trigger gets one output per thruster, in cluster order.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeFeedback2T_TH<dv>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, UNMODELED_TORQUE_NM, add_point_mass_earth,
    att_tracking_error, bit, external_disturbance, hub_inertia_kg_m2, inertial_3d, mrp_feedback,
    sampling_period_nanos, scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, NavigationTranslationMsg,
    Output, ThrusterArrayForceCommandMsg, ThrusterCommandMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use thruster_factory::{ThrusterFactory, thr_firing_schmitt, thr_force_mapping, thruster_factory};

/// Upstream `fswTimeStep = macros.sec2nano(0.5)`.
const FSW_PERIOD_NANOS: u64 = 500_000_000;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_dv_thrusters: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_dv_thrusters: bool) -> Self {
        Self {
            use_dv_thrusters,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAttitudeFeedback2T_TH{}",
            bit(self.use_dv_thrusters)
        )
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedback2T_TH.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// The recorded histories, every `samplingTime(T, 0.5 s, 100)` = 6 s.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    thruster_forces: MessageLog<ThrusterArrayForceCommandMsg>,
    on_times: Vec<MessageLog<ThrusterCommandMsg>>,
    thruster_count: usize,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.01));
    add_point_mass_earth(&mut spacecraft);
    spacecraft.add_dynamic_effector(external_disturbance(Vector3::from(UNMODELED_TORQUE_NM)));
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let factory: ThrusterFactory = thruster_factory(config.use_dv_thrusters, None);
    let thruster_count = factory.thrusters.len();
    let thr_config_msg = Output::new(factory.config_message());
    let mut thrusters: Vec<ThrusterDynamicEffector> = factory
        .thrusters
        .iter()
        .map(|thruster| ThrusterDynamicEffector::new(thruster.dynamic_effector_config()))
        .collect();

    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5 * 10.0, 0.0002, 30.0 * 10.0, Vector3::zeros());
    let mut force_mapping = thr_force_mapping(config.use_dv_thrusters);
    let mut schmitt = thr_firing_schmitt(config.use_dv_thrusters);
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(DURATION_NANOS, FSW_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut force_log = MessageLog::<ThrusterArrayForceCommandMsg>::default();
    let mut on_time_logs: Vec<MessageLog<ThrusterCommandMsg>> =
        (0..thruster_count).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for ((thruster, on_time_log), index) in
        thrusters.iter_mut().zip(on_time_logs.iter_mut()).zip(0..)
    {
        let command = schmitt.add_thruster_command_output().clone();
        connect!(&simulation,
            &command => &mut thruster.command_in,
            &command => &mut on_time_log.input,
            &command => recorder.add_source::<ThrusterCommandMsg>(CsvSourceConfig::columns([(
                "on_time_s".to_string(),
                format!("OnTimeRequest_s_{index}"),
            )])),
        );
    }
    for thruster in thrusters {
        spacecraft.add_dynamic_effector(thruster);
    }
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &controller.cmd_torque_out_msg => &mut force_mapping.cmd_torque_in_msg,
        &thr_config_msg => &mut force_mapping.thr_config_in_msg,
        &vehicle_config_msg => &mut force_mapping.veh_config_in_msg,
        &thr_config_msg => &mut schmitt.thr_conf_in_msg,
        &force_mapping.thr_force_cmd_out_msg => &mut schmitt.thr_force_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &force_mapping.thr_force_cmd_out_msg => &mut force_log.input,
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns(
                "torque_request_body_nm",
                "torqueRequestBody_Nm",
            )),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &navigation.translation_out_msg => recorder.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(vector_columns("position_bn_n_m", "r_BN_N_m")),
        ),
        &navigation.attitude_out_msg => recorder.add_source::<NavigationAttitudeMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
        ),
        &force_mapping.thr_force_cmd_out_msg => recorder.add_source::<ThrusterArrayForceCommandMsg>(
            CsvSourceConfig::columns(array_columns("thrust_force_n", "thrForce_N", thruster_count)),
        ),
    );
    schedule! { simulation,
        // Dynamics process; the thrusters live inside the spacecraft.
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        // Flight-software process, after the dynamics at shared ticks.
        "inertial_reference" => &mut inertial_reference, period=FSW_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=FSW_PERIOD_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=FSW_PERIOD_NANOS, priority=60;
        "thr_force_mapping" => &mut force_mapping, period=FSW_PERIOD_NANOS, priority=50;
        "thr_firing_schmitt" => &mut schmitt, period=FSW_PERIOD_NANOS, priority=40;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "force_log" => &mut force_log, period=sample_period, priority=0;
    }
    for log in &mut on_time_logs {
        simulation.add_module("on_time_log", log, sample_period, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        thruster_forces: force_log,
        on_times: on_time_logs,
        thruster_count,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_feedback_2t_th [--[no-]dv-thrusters] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(cli.flag("dv-thrusters", false))]
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
    //! Upstream `test_scenarioAttitudeFeedback2T_TH.py` only checks that both
    //! thruster sets run. These tests check what the scenario documents for
    //! each of them.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// "every axis is controlled, and the de-tumbling action is perfectly
    /// performed. ... the requested force for each thruster never reaches the
    /// 1 N limit (apart from during the initial transitory)".
    #[test]
    fn the_attitude_thrusters_control_all_three_axes() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert_eq!(results.thruster_count, 8);
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-2);
        assert!(results.guidance.last().omega_br_b_radps.norm() < 1.0e-3);
        // On-pulsing: forces are never negative, and after the first
        // minutes stay below the 0.9 N rating.
        for command in &results.thruster_forces.values {
            assert!(
                command.thrust_force_n[..8]
                    .iter()
                    .all(|force| *force >= 0.0)
            );
        }
        for command in results.thruster_forces.since(120_000_000_000) {
            assert!(command.thrust_force_n[..8].iter().all(|force| *force < 0.9));
        }
        // The Schmitt trigger never asks for more than 1.1 control periods.
        for log in &results.on_times {
            assert!(
                log.values
                    .iter()
                    .all(|command| command.on_time_s <= 0.55 + 1.0e-12)
            );
        }
    }

    /// "it is impossible to control the third axis, so the result is a
    /// spacecraft attitude in which the x, y axes are controlled, but with a
    /// tumbling motion in the z axis. ... the requested thruster force is
    /// always negative".
    #[test]
    fn the_orbit_thrusters_control_only_the_first_two_axes() {
        let results = quiet(TEST_PARAMETRIZATIONS[1]);
        assert_eq!(results.thruster_count, 6);
        for command in &results.thruster_forces.values {
            assert!(
                command.thrust_force_n[..6]
                    .iter()
                    .all(|force| *force <= 0.0)
            );
        }
        // The uncontrolled 0.1 N m disturbance about b3 spins the hub up
        // (0.1 / 600 rad/s^2 over 10 minutes) while b1 and b2 are held.
        let rate = results.guidance.last().omega_br_b_radps;
        assert!(rate.z.abs() > 0.1, "{rate}");
        assert!(
            rate.x.abs().max(rate.y.abs()) < 0.05 * rate.z.abs(),
            "{rate}"
        );
    }
}
