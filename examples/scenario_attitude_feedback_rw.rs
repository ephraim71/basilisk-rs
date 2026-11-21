//! Inertial attitude pointing with reaction wheels: port of upstream
//! `examples/scenarioAttitudeFeedbackRW.py`.
//!
//! The spacecraft of `scenarioAttitudeFeedback` (10 000 km orbit,
//! `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`)
//! with three Honeywell HR16 wheels (50 N m s, along the body axes, at 100,
//! 200 and 300 RPM, the third mounted at `[0.5, 0.5, 0.5] m`).
//! `mrpFeedback` (`K = 3.5`, `P = 30`, no integral feedback, wheel momentum
//! included) feeds `rwMotorTorque`, which maps the torque onto the three
//! wheels. One 0.1 s task, 10 minutes, histories sampled every 6 s.
//!
//! Upstream `run(show_plots, useJitterSimple, useRWVoltageIO)` maps to
//! `--jitter-simple` (the wheels use the simple static/dynamic imbalance
//! jitter model) and `--rw-voltage-io` (`rwMotorVoltage` turns the torques
//! into 0-10 V commands, and a `motorVoltageInterface` with a 0.02 N m/V gain
//! turns those back into wheel torques). The upstream script's own default is
//! `--no-jitter-simple --rw-voltage-io`; `--all` runs the three
//! parametrizations of upstream's `test_scenarioAttitudeFeedbackRW.py`.
//!
//! The wheels take one motor command each, while `rwMotorTorque` publishes
//! one array for the cluster. Without the voltage interface, upstream's
//! direct subscription of the wheels to that array is
//! `arrayMotorTorqueToSingleActuators` here, run right after `rwMotorTorque`
//! so the wheels see the same command upstream's would.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeFeedbackRW<js><vio>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/rw_factory.rs"]
mod rw_factory;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, add_point_mass_earth, att_tracking_error, bit,
    hub_inertia_kg_m2, inertial_3d, mrp_feedback, sampling_period_nanos, scenario_recorder,
    spacecraft, vehicle_config,
};
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::device_interface::motor_voltage_interface::{
    MotorVoltageInterface, MotorVoltageInterfaceConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::rw_motor_voltage::{RwMotorVoltage, RwMotorVoltageConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, ArrayMotorVoltageMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT,
    NavigationTranslationMsg, Output, ReactionWheelStateMsg, RwSpeedMsg,
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
use rw_factory::RwFactory;

const WHEEL_COUNT: usize = 3;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_jitter_simple: bool,
    use_rw_voltage_io: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_jitter_simple: bool, use_rw_voltage_io: bool) -> Self {
        Self {
            use_jitter_simple,
            use_rw_voltage_io,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAttitudeFeedbackRW{}{}",
            bit(self.use_jitter_simple),
            bit(self.use_rw_voltage_io),
        )
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedbackRW.py`.
const TEST_PARAMETRIZATIONS: [Config; 3] = [
    Config::new(false, false),
    Config::new(true, false),
    Config::new(false, true),
];

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    requested_torque: MessageLog<ArrayMotorTorqueMsg>,
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    voltage: MessageLog<ArrayMotorVoltageMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let model = if config.use_jitter_simple {
        ReactionWheelModel::JitterSimple
    } else {
        ReactionWheelModel::BalancedWheels
    };
    let mut rw_factory = RwFactory::default();
    rw_factory.create_honeywell_hr16(Vector3::x(), 50.0, 100.0, Vector3::zeros(), model);
    rw_factory.create_honeywell_hr16(Vector3::y(), 50.0, 200.0, Vector3::zeros(), model);
    rw_factory.create_honeywell_hr16(
        Vector3::z(),
        50.0,
        300.0,
        Vector3::new(0.5, 0.5, 0.5),
        model,
    );
    let rw_config_msg = Output::new(rw_factory.config_message());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for wheel in rw_factory.wheels {
        reaction_wheels.add_reaction_wheel(wheel);
    }
    let wheel_speed_msg = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_msgs: [Output<ReactionWheelStateMsg>; WHEEL_COUNT] =
        std::array::from_fn(|index| reaction_wheels.wheels()[index].state_out.clone());

    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.01));
    add_point_mass_earth(&mut spacecraft);

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    // Used with `useRWVoltageIO`.
    let mut rw_motor_voltage = RwMotorVoltage::new(RwMotorVoltageConfig {
        name: "rwMotorVoltage".to_string(),
        v_min_v: 0.0,
        v_max_v: 10.0,
        k: 0.0,
    });
    let mut voltage_interface = MotorVoltageInterface::new(MotorVoltageInterfaceConfig {
        name: "rwVoltageInterface".to_string(),
        torque_gains_nm_per_v: vec![0.2 / 10.0; WHEEL_COUNT],
    });
    // Used otherwise.
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: WHEEL_COUNT,
        });

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut requested_torque_log = MessageLog::<ArrayMotorTorqueMsg>::default();
    let mut wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let mut voltage_log = MessageLog::<ArrayMotorVoltageMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    let wheel_commands = if config.use_rw_voltage_io {
        &voltage_interface.motor_torque_out_msgs
    } else {
        &torque_splitter.actuator_out_msgs
    };
    for (command, wheel) in wheel_commands.iter().zip(reaction_wheels.wheels_mut()) {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);

    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut rw_motor_voltage.torque_in_msg,
        &rw_config_msg => &mut rw_motor_voltage.rw_params_in_msg,
        &rw_motor_voltage.voltage_out_msg => &mut voltage_interface.voltage_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut requested_torque_log.input,
        &rw_motor_voltage.voltage_out_msg => &mut voltage_log.input,
        &rw_motor_torque.rw_motor_torque_out_msg => recorder.add_source::<ArrayMotorTorqueMsg>(
            CsvSourceConfig::columns(array_columns(
                "motor_torque_nm",
                "motorTorque_Nm",
                MAX_EFF_COUNT,
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
        &wheel_speed_msg => recorder.add_source::<RwSpeedMsg>(
            CsvSourceConfig::columns(array_columns(
                "wheel_speeds_radps",
                "wheelSpeeds_rad_per_s",
                MAX_EFF_COUNT,
            )),
        ),
    );
    for (index, (state, log)) in wheel_state_msgs.iter().zip(&mut wheel_logs).enumerate() {
        connect!(&simulation,
            state => &mut log.input,
            state => recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
        );
    }
    if config.use_rw_voltage_io {
        connect!(&simulation,
            &rw_motor_voltage.voltage_out_msg => recorder.add_source::<ArrayMotorVoltageMsg>(
                CsvSourceConfig::columns(array_columns("voltage_v", "rwVoltage_V", MAX_EFF_COUNT)),
            ),
        );
    }

    // Upstream's task order: wheels and spacecraft, the voltage interface,
    // then navigation and flight software.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "inertial_reference" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "mrp_feedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=50;
        "rw_motor_torque" => &mut rw_motor_torque, period=DYNAMICS_PERIOD_NANOS, priority=40;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "requested_torque_log" => &mut requested_torque_log, period=sample_period, priority=0;
    }
    if config.use_rw_voltage_io {
        schedule! { simulation,
            "voltage_interface" => &mut voltage_interface, period=DYNAMICS_PERIOD_NANOS, priority=90;
            "rw_motor_voltage" => &mut rw_motor_voltage, period=DYNAMICS_PERIOD_NANOS, priority=30;
            "voltage_log" => &mut voltage_log, period=sample_period, priority=0;
        }
    } else {
        simulation.add_module(
            "rw_motor_torque_splitter",
            &mut torque_splitter,
            DYNAMICS_PERIOD_NANOS,
            35,
        );
    }
    for log in &mut wheel_logs {
        simulation.add_module("wheel_log", log, sample_period, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        requested_torque: requested_torque_log,
        wheels: wheel_logs,
        voltage: voltage_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_attitude_feedback_rw [--[no-]jitter-simple] [--[no-]rw-voltage-io] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, False, True)`.
        vec![Config::new(
            cli.flag("jitter-simple", false),
            cli.flag("rw-voltage-io", true),
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
    //! Upstream `test_scenarioAttitudeFeedbackRW.py` only checks that each
    //! parametrization runs. These tests check what the scenario documents.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// "with this maneuver the RW devices are being saturated, and the
    /// attitude still eventually stabilizes."
    #[test]
    fn the_saturated_wheels_still_stabilize_the_attitude() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_eq!(results.guidance.values.len(), 101);
            assert!(
                results.guidance.last().sigma_br.norm() < 1.0e-3,
                "{config:?}"
            );
            assert!(results.guidance.last().omega_br_b_radps.norm() < 1.0e-4);

            let requested_peak = results
                .requested_torque
                .values
                .iter()
                .flat_map(|command| command.motor_torque_nm[..WHEEL_COUNT].to_vec())
                .fold(0.0_f64, |peak, torque| peak.max(torque.abs()));
            assert!(requested_peak > 0.2, "{config:?}: {requested_peak}");
            for wheel in &results.wheels {
                for state in &wheel.values {
                    assert!(state.applied_motor_torque_nm.abs() <= 0.2 + 1.0e-12);
                }
            }
        }
    }

    /// "The impact of the RW jitter is very small, naturally. The plots for
    /// this case look very similar to the balanced RW case. But there is a
    /// distinct numerical difference."
    #[test]
    fn jitter_changes_the_attitude_only_slightly() {
        let balanced = quiet(TEST_PARAMETRIZATIONS[0]);
        let jitter = quiet(TEST_PARAMETRIZATIONS[1]);
        let differences: Vec<f64> = balanced
            .guidance
            .values
            .iter()
            .zip(&jitter.guidance.values)
            .map(|(left, right)| (left.sigma_br - right.sigma_br).norm())
            .collect();
        let largest = differences.iter().copied().fold(0.0_f64, f64::max);
        // Below 1.5 % of the initial |sigma_BR| = 0.374, and gone once the
        // attitude has settled.
        assert!(largest > 1.0e-4, "{largest}");
        assert!(largest < 5.0e-3, "{largest}");
        assert!(*differences.last().unwrap() < 1.0e-4);
    }

    /// `rwMotorVoltage` maps the torques into the 0-10 V range of the
    /// interface, whose 0.02 N m/V gain gives the wheel torques back.
    #[test]
    fn the_voltage_commands_stay_in_range() {
        let results = quiet(TEST_PARAMETRIZATIONS[2]);
        assert_eq!(results.voltage.values.len(), 101);
        let mut saturated = false;
        for voltage in &results.voltage.values {
            for volts in &voltage.voltage_v[..WHEEL_COUNT] {
                assert!(volts.abs() <= 10.0);
                saturated |= volts.abs() == 10.0;
            }
        }
        assert!(saturated);
    }
}
