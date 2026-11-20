//! Reaction-wheel momentum management with magnetic torque bars, the
//! modular chain: port of upstream
//! `examples/scenarioMtbMomentumManagementSimple.py`.
//!
//! The spacecraft, wheels, torque bars, field and attitude control of
//! `scenario_mtb_momentum_management`, with the momentum management split
//! into upstream's simpler modules:
//!
//! * `mtbMomentumManagementSimple` (`Kp = 0.003`) asks for the torque that
//!   removes the net wheel momentum;
//! * `torque2Dipole` turns it into a body dipole with the measured field,
//!   and `dipoleMapping` maps that onto the four torque bars;
//! * `mtbFeedforward` adds the torque-bar torque to the attitude control
//!   torque, which `rwMotorTorque` maps onto the wheels;
//! * `rwNullSpace` (`OmegaGain = 3e-7`) adds null-space torques that drive
//!   every wheel towards 1000 RPM.
//!
//! One 0.5 s task for 120 minutes, histories sampled every 36 s.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case.
//!
//! basilisk-rs keeps one command per device: `dipoleMapping` publishes one
//! dipole per torque bar besides its array message (which `mtbFeedforward`
//! reads), and `arrayMotorTorqueToSingleActuators` splits `rwNullSpace`'s
//! wheel-torque array into the wheels' commands. `rwNullSpace` reads the
//! wheel configuration as an `RwArrayConfigMsg` where upstream builds an
//! `RWConstellationMsg` with the same axes, inertias and torque limits.
//!
//! Writes `examples/output/scenarios/scenarioMtbMomentumManagementSimple.csv`
//! with the columns of `scenario_mtb_momentum_management`.

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
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::mtb_dynamic_effector::{MtbConfig, MtbEffector};
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector;
use basilisk_rs::fsw_algorithms::att_control::mtb_feedforward::{
    MtbFeedforward, MtbFeedforwardConfig,
};
use basilisk_rs::fsw_algorithms::att_control::mtb_momentum_management_simple::{
    MtbMomentumManagementSimple, MtbMomentumManagementSimpleConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::dipole_mapping::{
    DipoleMapping, DipoleMappingConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_null_space::{
    RwNullSpace, RwNullSpaceConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::torque2_dipole::{
    Torque2Dipole, Torque2DipoleConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT, MagneticFieldMsg, MtbArrayCommandMsg,
    Output, ReactionWheelStateMsg, RwSpeedMsg, TamSensorBodyMsg, TamSensorMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::RPM;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use message_log::MessageLog;
use mtb::{DURATION_NANOS, MAX_DIPOLE_AM2};
use nalgebra::Vector3;

/// Upstream `simulationTimeStep = macros.sec2nano(0.5)`.
const TASK_PERIOD_NANOS: u64 = 500_000_000;
/// Upstream `desiredOmega = [1000 * macros.rpm2radsec] * numRW` \[RPM\].
const DESIRED_WHEEL_SPEED_RPM: f64 = 1000.0;

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
    let mtb_config_output = Output::new(mtb::mtb_config());
    let vehicle_config_output = Output::new(mtb::vehicle_config());
    let mut desired_speeds = RwSpeedMsg::default();
    desired_speeds.wheel_speeds_radps[..4].fill(DESIRED_WHEEL_SPEED_RPM * RPM);
    let desired_speed_output = Output::new(desired_speeds);

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
    let mut mtb_effector = MtbEffector::new("MtbEff");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut mrp_feedback = mtb::controller();
    let mut tam = mtb::tam();
    let mut tam_comm = mtb::tam_comm();
    let mut momentum_management =
        MtbMomentumManagementSimple::new(MtbMomentumManagementSimpleConfig {
            name: "mtbMomentumManagementSimple".to_string(),
            kp_per_s: 0.003,
        });
    let mut torque_to_dipole = Torque2Dipole::new(Torque2DipoleConfig::default());
    #[allow(clippy::approx_constant)]
    let mut dipole_mapping = DipoleMapping::new(DipoleMappingConfig {
        name: "dipoelMapping".to_string(),
        steering_matrix: vec![
            Vector3::new(0.75, -0.25, 0.0),
            Vector3::new(-0.25, 0.75, 0.0),
            Vector3::new(0.0, 0.0, 1.0),
            Vector3::new(0.353_553_39, 0.353_553_39, 0.0),
        ],
    });
    let mut feedforward = MtbFeedforward::new(MtbFeedforwardConfig::default());
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut null_space = RwNullSpace::new(RwNullSpaceConfig {
        name: "rwNullSpace".to_string(),
        omega_gain: 0.000_000_3,
    });
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: 4,
        });

    let sample_period = sampling_period_nanos(DURATION_NANOS, TASK_PERIOD_NANOS, 200);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut wheel_speed_log = MessageLog::<RwSpeedMsg>::default();
    let mut dipole_log = MessageLog::<MtbArrayCommandMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioMtbMomentumManagementSimple", config.write_csv);

    let mut simulation = Simulation::new(mtb::epoch(), false);
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    for (index, axis) in mtb::mtb_axes().iter().enumerate() {
        let mtb = mtb_effector.add_mtb(MtbConfig {
            name: format!("Mtb{index}"),
            dipole_axis_body: *axis,
            max_dipole_am2: MAX_DIPOLE_AM2,
        });
        simulation.connect(
            dipole_mapping.add_mtb_command_output(),
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
        &spacecraft.state_out => &mut tam.input_state_msg,
        &magnetic_field.output_magnetic_field_msg => &mut tam.input_magnetic_field_msg,
        &tam.output_tam_msg => &mut tam_comm.tam_in_msg,
        &rw_config_output => &mut momentum_management.rw_params_in_msg,
        &wheel_speed_output => &mut momentum_management.rw_speeds_in_msg,
        &momentum_management.tau_mtb_request_out_msg => &mut torque_to_dipole.tau_request_in_msg,
        &tam_comm.tam_out_msg => &mut torque_to_dipole.tam_sensor_body_in_msg,
        &torque_to_dipole.dipole_request_out_msg => &mut dipole_mapping.dipole_request_body_in_msg,
        &mtb_config_output => &mut dipole_mapping.mtb_array_config_params_in_msg,
        &mrp_feedback.cmd_torque_out_msg => &mut feedforward.veh_control_in_msg,
        &dipole_mapping.dipole_request_mtb_out_msg => &mut feedforward.dipole_request_mtb_in_msg,
        &tam_comm.tam_out_msg => &mut feedforward.tam_sensor_body_in_msg,
        &mtb_config_output => &mut feedforward.mtb_array_config_params_in_msg,
        &rw_config_output => &mut rw_motor_torque.rw_params_in_msg,
        &feedforward.veh_control_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut null_space.rw_motor_torque_in_msg,
        &wheel_speed_output => &mut null_space.rw_speeds_in_msg,
        &rw_config_output => &mut null_space.rw_config_in_msg,
        &desired_speed_output => &mut null_space.rw_desired_speeds_in_msg,
        &null_space.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &wheel_speed_output => &mut wheel_speed_log.input,
        &dipole_mapping.dipole_request_mtb_out_msg => &mut dipole_log.input,
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
            CsvSourceConfig::columns(vector_columns("magnetic_field_inertial_t", "magField_N_T")),
        ),
        &tam.output_tam_msg => recorder.add_source::<TamSensorMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_sensor_t", "tam_S_T")),
        ),
        &tam_comm.tam_out_msg => recorder.add_source::<TamSensorBodyMsg>(
            CsvSourceConfig::columns(vector_columns("magnetic_field_body_t", "tam_B_T")),
        ),
        &dipole_mapping.dipole_request_mtb_out_msg => recorder.add_source::<MtbArrayCommandMsg>(
            CsvSourceConfig::columns(array_columns(
                "dipole_cmds_am2",
                "mtbDipoleCmds_A_m2",
                MAX_EFF_COUNT,
            )),
        ),
    );
    // Upstream's task order: the spacecraft and wheels, then the modules as
    // added.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=95;
        "magnetic_field" => &mut magnetic_field, period=TASK_PERIOD_NANOS, priority=90;
        "inertial_reference" => &mut inertial_reference, period=TASK_PERIOD_NANOS, priority=85;
        "tracking_error" => &mut tracking_error, period=TASK_PERIOD_NANOS, priority=80;
        "mrp_feedback" => &mut mrp_feedback, period=TASK_PERIOD_NANOS, priority=75;
        "tam" => &mut tam, period=TASK_PERIOD_NANOS, priority=70;
        "tam_comm" => &mut tam_comm, period=TASK_PERIOD_NANOS, priority=65;
        "momentum_management" => &mut momentum_management, period=TASK_PERIOD_NANOS, priority=60;
        "torque2_dipole" => &mut torque_to_dipole, period=TASK_PERIOD_NANOS, priority=55;
        "dipole_mapping" => &mut dipole_mapping, period=TASK_PERIOD_NANOS, priority=50;
        "mtb_feedforward" => &mut feedforward, period=TASK_PERIOD_NANOS, priority=45;
        "rw_motor_torque" => &mut rw_motor_torque, period=TASK_PERIOD_NANOS, priority=40;
        "rw_null_space" => &mut null_space, period=TASK_PERIOD_NANOS, priority=35;
        "rw_motor_torque_splitter" => &mut torque_splitter, period=TASK_PERIOD_NANOS, priority=30;
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
    let mut cli = Cli::from_env("scenario_mtb_momentum_management_simple [--all]");
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
    //! These tests check what it demonstrates. The attitude settles on the
    //! inertial reference. The torque bars, within their 0.1 A m^2 limit,
    //! remove the net wheel momentum, while the null-space torques bring the
    //! wheel speeds' null-space component (the one that leaves the net
    //! momentum unchanged) to that of 1000 RPM on every wheel. The two goals
    //! share the four wheels, so the speeds settle near, not at, 1000 RPM.

    use nalgebra::Vector4;

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_net_momentum_goes_and_the_null_space_speed_reaches_its_target() {
        let results = quiet();
        // samplingTime(120 min, 0.5 s, 200) = 36 s: samples 0, 36, ..., 7200 s.
        assert_eq!(results.guidance.values.len(), 201);

        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1.0e-3, "{}", last.sigma_br);
        assert!(last.omega_br_b_radps.norm() < 1.0e-4);

        // The wheels share one spin inertia, so the net momentum is
        // proportional to `sum g_i Omega_i`.
        let axes = mtb::wheel_axes();
        let speeds =
            Vector4::from_column_slice(&results.wheel_speeds.last().wheel_speeds_radps[..4]);
        let desired = Vector4::repeat(DESIRED_WHEEL_SPEED_RPM * RPM);
        let net = |omega: &Vector4<f64>| -> Vector3<f64> {
            axes.iter()
                .zip(omega.iter())
                .map(|(axis, speed)| axis * *speed)
                .sum()
        };
        // Net momentum below 1% of what one wheel at the target speed holds.
        assert!(
            net(&speeds).norm() < 0.01 * DESIRED_WHEEL_SPEED_RPM * RPM,
            "{}",
            net(&speeds)
        );
        // The null space of `[g_1 .. g_4]` (beta = 52 deg).
        let beta = 52.0_f64.to_radians();
        let (s, c) = beta.sin_cos();
        let null = Vector4::new(1.0, s / c, 0.5 / (s * c), 0.5 / (s * c));
        assert!(net(&null).norm() < 1.0e-12);
        let error = (null.dot(&speeds) - null.dot(&desired)) / null.dot(&desired);
        assert!(error.abs() < 0.01, "{error}");

        for sample in &results.dipoles.values {
            for dipole in &sample.dipole_cmds_am2[..4] {
                assert!(dipole.abs() <= MAX_DIPOLE_AM2 + 1.0e-15, "{dipole}");
            }
        }
    }
}
