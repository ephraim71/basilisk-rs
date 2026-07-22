//! Reaction-wheel attitude-feedback configuration `01` from
//! `scenarioAttitudeFeedbackRW.py`.
//!
//! Balanced Honeywell HR16 wheels with the flight-software and hardware
//! voltage interfaces enabled. Writes the simulated telemetry to CSV.

#[path = "support/common.rs"]
mod common;

use basilisk_rs::device_interface::motor_voltage_interface::{
    MotorVoltageInterface, MotorVoltageInterfaceConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::fsw_algorithms::att_tracking_error::{AttTrackingError, AttTrackingErrorConfig};
use basilisk_rs::fsw_algorithms::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::rw_motor_voltage::{RwMotorVoltage, RwMotorVoltageConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, ArrayMotorVoltageMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT, Output,
    ReactionWheelStateMsg, RwArrayConfigMsg, RwSpeedMsg, TranslationReferenceMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::{connect, schedule};
use common::{
    MU_EARTH_M3PS2, array_columns, elem2rv, rpm_to_radps, scenario_output_path, seconds,
    vector_columns,
};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

const TASK_PERIOD_NANOS: u64 = 100_000_000;
const SAMPLE_PERIOD_NANOS: u64 = seconds(6);
const DURATION_NANOS: u64 = seconds(600);

fn hr16_config(
    name: &str,
    axis_body: Vector3<f64>,
    position_m: Vector3<f64>,
    initial_rpm: f64,
) -> ReactionWheelStateEffectorConfig {
    let maximum_speed_radps = rpm_to_radps(6_000.0);
    let mut config =
        ReactionWheelStateEffectorConfig::balanced(name, position_m, axis_body, 0.2, 50.0);
    config.js_kg_m2 = 50.0 / maximum_speed_radps;
    config.jt_kg_m2 = 0.5 * config.js_kg_m2;
    config.jg_kg_m2 = config.jt_kg_m2;
    config.max_speed_radps = maximum_speed_radps;
    config.initial_omega_radps = rpm_to_radps(initial_rpm);
    config
}

fn rw_configuration(wheels: &[ReactionWheelStateEffectorConfig; 3]) -> RwArrayConfigMsg {
    let mut config = RwArrayConfigMsg {
        num_rw: wheels.len(),
        ..Default::default()
    };
    for (index, wheel) in wheels.iter().enumerate() {
        config.spin_axes_body[index] = wheel.spin_axis_body;
        config.spin_axis_inertias_kg_m2[index] = wheel.js_kg_m2;
        config.max_motor_torques_nm[index] = wheel.max_torque_nm;
    }
    config
}

fn main() {
    let degrees = std::f64::consts::PI / 180.0;
    let inertia = Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0));
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        10_000_000.0,
        0.01,
        33.3 * degrees,
        48.2 * degrees,
        347.8 * degrees,
        85.3 * degrees,
    );

    let wheel_configs = [
        hr16_config("RW1", Vector3::x(), Vector3::zeros(), 100.0),
        hr16_config("RW2", Vector3::y(), Vector3::zeros(), 200.0),
        hr16_config("RW3", Vector3::z(), Vector3::new(0.5, 0.5, 0.5), 300.0),
    ];
    let rw_config_output = Output::new(rw_configuration(&wheel_configs));
    let vehicle_config_output = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        center_of_mass_body_m: Vector3::zeros(),
        mass_kg: 750.0,
        current_adcs_state: 0,
    });

    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for config in wheel_configs {
        reaction_wheels.add_reaction_wheel(config);
    }
    let wheel_speed_output = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_outputs: [Output<ReactionWheelStateMsg>; 3] =
        std::array::from_fn(|index| reaction_wheels.wheels()[index].state_out.clone());

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::zeros(),
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attErrorInertial3D".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let mut mrp_feedback = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: 3.5,
        ki: -1.0,
        p: 30.0,
        integral_limit: -0.2,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut rw_motor_voltage = RwMotorVoltage::new(RwMotorVoltageConfig {
        name: "rwMotorVoltage".to_string(),
        v_min_v: 0.0,
        v_max_v: 10.0,
        k: 0.0,
    });
    let mut voltage_interface = MotorVoltageInterface::new(MotorVoltageInterfaceConfig {
        name: "rwVoltageInterface".to_string(),
        torque_gains_nm_per_v: vec![0.02; 3],
    });

    let output_path = scenario_output_path("scenarioAttitudeFeedbackRW01.csv");
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioAttitudeFeedbackRW01".to_string(),
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Scenario);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &voltage_interface.motor_torque_out_msg => &mut reaction_wheels.rw_motor_cmd_in_msg,
    );
    spacecraft.add_state_effector(reaction_wheels);

    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut mrp_feedback.guid_in_msg,
        &vehicle_config_output => &mut mrp_feedback.veh_config_in_msg,
        &rw_config_output => &mut mrp_feedback.rw_params_in_msg,
        &wheel_speed_output => &mut mrp_feedback.rw_speeds_in_msg,
        &mrp_feedback.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_output => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut rw_motor_voltage.torque_in_msg,
        &rw_config_output => &mut rw_motor_voltage.rw_params_in_msg,
        &rw_motor_voltage.voltage_out_msg => &mut voltage_interface.voltage_in_msg,
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
        &navigation.translation_out_msg => recorder.add_source::<TranslationReferenceMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
        &wheel_speed_output => recorder.add_source::<RwSpeedMsg>(
            CsvSourceConfig::columns(array_columns(
                "wheel_speeds_radps",
                "wheelSpeeds_rad_per_s",
                MAX_EFF_COUNT,
            )),
        ),
        &wheel_state_outputs[0] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_0")]),
        ),
        &wheel_state_outputs[1] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_1")]),
        ),
        &wheel_state_outputs[2] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_2")]),
        ),
        &rw_motor_voltage.voltage_out_msg => recorder.add_source::<ArrayMotorVoltageMsg>(
            CsvSourceConfig::columns(array_columns(
                "voltage_v",
                "rwVoltage_V",
                MAX_EFF_COUNT,
            )),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, TASK_PERIOD_NANOS, 100;
        "voltage_interface" => &mut voltage_interface, TASK_PERIOD_NANOS, 90;
        "navigation" => &mut navigation, TASK_PERIOD_NANOS, 80;
        "inertial_reference" => &mut inertial_reference, TASK_PERIOD_NANOS, 70;
        "tracking_error" => &mut tracking_error, TASK_PERIOD_NANOS, 60;
        "mrp_feedback" => &mut mrp_feedback, TASK_PERIOD_NANOS, 50;
        "rw_motor_torque" => &mut rw_motor_torque, TASK_PERIOD_NANOS, 40;
        "rw_motor_voltage" => &mut rw_motor_voltage, TASK_PERIOD_NANOS, 30;
        "recorder" => &mut recorder, SAMPLE_PERIOD_NANOS, 0;
    }
    simulation.run_for(DURATION_NANOS);

    println!("wrote {}", output_path.display());
}
