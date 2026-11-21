//! Reaction-wheel momentum management with magnetic torque bars: port of
//! upstream `examples/scenarioMtbMomentumManagement.py`.
//!
//! A 10 kg spacecraft on a 400 km, 45 deg Earth orbit holds the inertial
//! attitude (`inertial3D`, `attTrackingError`, `mrpFeedback` with `K =
//! 0.0001`, `P = 0.002`, `rwMotorTorque`) with four BCT RWP015 wheels at
//! 52 deg. `mtbMomentumManagement` drives the wheel speeds towards the biases
//! `[800, 600, 400, 200] RPM` with four torque bars (0.1 A m^2), reading the
//! WMM field (`2019 June 27, 10:23:0.0 (UTC)`) through a noiseless
//! magnetometer and `tamComm`, and feeds the wheel torques forward. One 2 s
//! task for 120 minutes, histories sampled every 36 s.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted for
//! symmetry with the other scenarios and runs the same single case.
//!
//! The wheels and torque bars take one command each: `mtbMomentumManagement`
//! publishes one wheel torque and one dipole per device.
//!
//! Writes `examples/output/scenarios/scenarioMtbMomentumManagement.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mtb.rs"]
mod mtb;
#[path = "support/rw_factory.rs"]
mod rw_factory;

use std::path::PathBuf;

use attitude::{att_tracking_error, inertial_3d, sampling_period_nanos, scenario_recorder};
use basilisk_rs::dynamics::mtb_dynamic_effector::{MtbConfig, MtbEffector};
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector;
use basilisk_rs::fsw_algorithms::mtb_momentum_management::{
    MtbMomentumManagement, MtbMomentumManagementConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT, MagneticFieldMsg, MtbArrayCommandMsg,
    Output, ReactionWheelStateMsg, RwSpeedMsg, TamSensorBodyMsg, TamSensorMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, rpm_to_radps, seconds, vector_columns};
use message_log::MessageLog;
use mtb::{DURATION_NANOS, MAX_DIPOLE_AM2};
use nalgebra::Vector3;

const TASK_PERIOD_NANOS: u64 = seconds(2);
/// Upstream `wheelSpeedBiases` \[RPM\].
const WHEEL_SPEED_BIASES_RPM: [f64; 4] = [800.0, 600.0, 400.0, 200.0];

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every 36 s.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    wheel_speeds: MessageLog<RwSpeedMsg>,
    dipoles: MessageLog<MtbArrayCommandMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let rw_factory = mtb::rw_factory();
    let rw_config_output = Output::new(rw_factory.config_message());
    let mtb_axes = mtb::mtb_axes();
    let mtb_config_output = Output::new(mtb::mtb_config());
    let vehicle_config_output = Output::new(mtb::vehicle_config());

    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for wheel in rw_factory.wheels {
        reaction_wheels.add_reaction_wheel(wheel);
    }
    let wheel_speed_output = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_outputs: [Output<ReactionWheelStateMsg>; 4] =
        std::array::from_fn(|index| reaction_wheels.wheels()[index].state_out.clone());

    let mut spacecraft = mtb::spacecraft(TASK_PERIOD_NANOS);
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut magnetic_field = mtb::wmm();
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut mrp_feedback = mtb::controller();
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut tam = mtb::tam();
    let mut tam_comm = mtb::tam_comm();
    let mut wheel_speed_biases_radps = [0.0; MAX_EFF_COUNT];
    for (slot, rpm) in wheel_speed_biases_radps[..4]
        .iter_mut()
        .zip(WHEEL_SPEED_BIASES_RPM)
    {
        *slot = rpm_to_radps(rpm);
    }
    let mut momentum_management = MtbMomentumManagement::new(MtbMomentumManagementConfig {
        name: "mtbMomentumManagement".to_string(),
        wheel_speed_biases_radps,
        c_gain_per_sec: 0.003,
    });
    let mut mtb_effector = MtbEffector::new("MtbEff");

    let sample_period = sampling_period_nanos(DURATION_NANOS, TASK_PERIOD_NANOS, 200);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut wheel_speed_log = MessageLog::<RwSpeedMsg>::default();
    let mut dipole_log = MessageLog::<MtbArrayCommandMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioMtbMomentumManagement", config.write_csv);

    let mut simulation = Simulation::new(mtb::epoch(), false);
    for wheel in reaction_wheels.wheels_mut() {
        simulation.connect(
            momentum_management.add_rw_motor_torque_output(),
            &mut wheel.motor_torque_in_msg,
        );
    }
    for (index, axis) in mtb_axes.iter().enumerate() {
        let mtb = mtb_effector.add_mtb(MtbConfig {
            name: format!("Mtb{index}"),
            dipole_axis_body: *axis,
            max_dipole_am2: MAX_DIPOLE_AM2,
        });
        simulation.connect(
            momentum_management.add_mtb_command_output(),
            &mut mtb.dipole_cmd_in_msg,
        );
    }
    connect!(&simulation,
        &magnetic_field.output_magnetic_field_msg => &mut mtb_effector.mag_in_msg,
    );
    spacecraft.add_state_effector(reaction_wheels);
    spacecraft.add_dynamic_effector(mtb_effector);

    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &spacecraft.state_out => &mut magnetic_field.input_state_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut mrp_feedback.guid_in_msg,
        &vehicle_config_output => &mut mrp_feedback.veh_config_in_msg,
        &rw_config_output => &mut mrp_feedback.rw_params_in_msg,
        &wheel_speed_output => &mut mrp_feedback.rw_speeds_in_msg,
        &mrp_feedback.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_output => &mut rw_motor_torque.rw_params_in_msg,
        &spacecraft.state_out => &mut tam.input_state_msg,
        &magnetic_field.output_magnetic_field_msg => &mut tam.input_magnetic_field_msg,
        &tam.output_tam_msg => &mut tam_comm.tam_in_msg,
        &rw_config_output => &mut momentum_management.rw_params_in_msg,
        &mtb_config_output => &mut momentum_management.mtb_params_in_msg,
        &tam_comm.tam_out_msg => &mut momentum_management.tam_sensor_body_in_msg,
        &wheel_speed_output => &mut momentum_management.rw_speeds_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut momentum_management.rw_motor_torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &wheel_speed_output => &mut wheel_speed_log.input,
        &momentum_management.mtb_cmd_out_msg => &mut dipole_log.input,
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
                    .chain(vector_columns(
                        "omega_br_b_radps",
                        "omega_BR_B_rad_per_s",
                    )),
            ),
        ),
        &wheel_speed_output => recorder.add_source::<RwSpeedMsg>(
            CsvSourceConfig::columns(array_columns(
                "wheel_speeds_radps",
                "wheelSpeeds_rad_per_s",
                MAX_EFF_COUNT,
            )),
        ),
    );
    for (index, state) in wheel_state_outputs.iter().enumerate() {
        connect!(&simulation,
            state => recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
        );
    }
    connect!(&simulation,
        &magnetic_field.output_magnetic_field_msg => recorder.add_source::<MagneticFieldMsg>(
            CsvSourceConfig::columns(vector_columns(
                "magnetic_field_inertial_t",
                "magField_N_T",
            )),
        ),
        &tam.output_tam_msg => recorder.add_source::<TamSensorMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_sensor_t", "tam_S_T")),
        ),
        &tam_comm.tam_out_msg => recorder.add_source::<TamSensorBodyMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_body_t", "tam_B_T")),
        ),
        &momentum_management.mtb_cmd_out_msg => recorder.add_source::<MtbArrayCommandMsg>(
            CsvSourceConfig::columns(array_columns(
                "dipole_cmds_am2",
                "mtbDipoleCmds_A_m2",
                MAX_EFF_COUNT,
            )),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=90;
        "magnetic_field" => &mut magnetic_field, period=TASK_PERIOD_NANOS, priority=80;
        "inertial_reference" => &mut inertial_reference, period=TASK_PERIOD_NANOS, priority=70;
        "tracking_error" => &mut tracking_error, period=TASK_PERIOD_NANOS, priority=60;
        "mrp_feedback" => &mut mrp_feedback, period=TASK_PERIOD_NANOS, priority=50;
        "rw_motor_torque" => &mut rw_motor_torque, period=TASK_PERIOD_NANOS, priority=40;
        "tam" => &mut tam, period=TASK_PERIOD_NANOS, priority=30;
        "tam_comm" => &mut tam_comm, period=TASK_PERIOD_NANOS, priority=20;
        "momentum_management" => &mut momentum_management, period=TASK_PERIOD_NANOS, priority=10;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "wheel_speed_log" => &mut wheel_speed_log, period=sample_period, priority=0;
        "dipole_log" => &mut dipole_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        wheel_speeds: wheel_speed_log,
        dipoles: dipole_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_mtb_momentum_management [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        println!("wrote {}", results.csv_path.expect("CSV written").display());
    }
}

#[cfg(test)]
mod tests {
    //! Upstream only runs this scenario (an entry of `test_scenarioTest.py`).
    //! These tests check what it demonstrates: the attitude settles on the
    //! inertial reference while the torque bars, within their 0.1 A m^2
    //! limit, bring every wheel to its speed bias.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_wheels_reach_their_biases_while_the_attitude_holds() {
        let results = quiet();
        // samplingTime(120 min, 2 s, 200) = 36 s: samples 0, 36, ..., 7200 s.
        assert_eq!(results.guidance.values.len(), 201);

        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1.0e-3, "{}", last.sigma_br);
        assert!(last.omega_br_b_radps.norm() < 1.0e-4);

        let speeds = results.wheel_speeds.last().wheel_speeds_radps;
        for (speed, bias) in speeds.iter().zip(WHEEL_SPEED_BIASES_RPM) {
            let bias = rpm_to_radps(bias);
            assert!((speed - bias).abs() < 0.05 * bias, "{speed} vs {bias}");
        }
        for sample in &results.dipoles.values {
            for dipole in &sample.dipole_cmds_am2[..4] {
                assert!(dipole.abs() <= MAX_DIPOLE_AM2 + 1.0e-15, "{dipole}");
            }
        }
    }
}
