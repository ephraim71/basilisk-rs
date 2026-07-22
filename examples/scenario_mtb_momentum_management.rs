//! Magnetic torque-bar momentum-management configuration from
//! `scenarioMtbMomentumManagement.py`.
//!
//! Four reaction wheels, four magnetic torque bars, WMM2025, and the complete
//! attitude/momentum-management control chain. Writes the telemetry to CSV.

#[path = "support/common.rs"]
mod common;

use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::mtb_dynamic_effector::MtbEffector;
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::environment::wmm_field::{WmmField, WmmFieldConfig};
use basilisk_rs::fsw_algorithms::att_tracking_error::{AttTrackingError, AttTrackingErrorConfig};
use basilisk_rs::fsw_algorithms::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::mtb_momentum_management::{
    MtbMomentumManagement, MtbMomentumManagementConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::tam_comm::{TamComm, TamCommConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, MAX_EFF_COUNT, MagneticFieldMsg, MtbArrayCommandMsg,
    MtbArrayConfigMsg, Output, ReactionWheelStateMsg, RwArrayConfigMsg, RwSpeedMsg,
    TamSensorBodyMsg, TamSensorMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::magnetometer::{Magnetometer, MagnetometerConfig};
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
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

const TASK_PERIOD_NANOS: u64 = seconds(2);
const SAMPLE_PERIOD_NANOS: u64 = seconds(36);
const DURATION_NANOS: u64 = seconds(7_200);

fn bct_wheel(name: &str, spin_axis_body: Vector3<f64>) -> ReactionWheelStateEffectorConfig {
    let maximum_speed_radps = rpm_to_radps(5_000.0);
    let mut config = ReactionWheelStateEffectorConfig::balanced(
        name,
        Vector3::zeros(),
        spin_axis_body,
        0.004,
        0.015,
    );
    config.js_kg_m2 = 0.015 / maximum_speed_radps;
    config.jt_kg_m2 = 0.5 * config.js_kg_m2;
    config.jg_kg_m2 = config.jt_kg_m2;
    config.max_speed_radps = maximum_speed_radps;
    config
}

fn rw_configuration(wheels: &[ReactionWheelStateEffectorConfig; 4]) -> RwArrayConfigMsg {
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
    let beta = 52.0 * degrees;
    let wheel_axes = [
        Vector3::new(0.0, beta.cos(), beta.sin()),
        Vector3::new(0.0, beta.sin(), -beta.cos()),
        Vector3::new(beta.cos(), -beta.sin(), 0.0),
        Vector3::new(-beta.cos(), -beta.sin(), 0.0),
    ];
    let wheel_configs =
        std::array::from_fn(|index| bct_wheel(&format!("RW{}", index + 1), wheel_axes[index]));
    let rw_config_output = Output::new(rw_configuration(&wheel_configs));

    // Preserve the source scenario's deliberately truncated diagonal-axis
    // literal.
    #[allow(clippy::approx_constant)]
    let mtb_axes = [
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Vector3::new(0.707_106_78, 0.707_106_78, 0.0),
    ];
    let mtb_config_output = Output::new(MtbArrayConfigMsg::from_active(&mtb_axes, &[0.1; 4]));

    let inertia = Matrix3::from_diagonal(&Vector3::new(0.02 / 3.0, 0.1256 / 3.0, 0.1256 / 3.0));
    let vehicle_config_output = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        center_of_mass_body_m: Vector3::zeros(),
        mass_kg: 10.0,
        current_adcs_state: 0,
    });
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        6_778_140.0,
        0.0,
        45.0 * degrees,
        60.0 * degrees,
        0.0,
        0.0,
    );

    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for config in wheel_configs {
        reaction_wheels.add_reaction_wheel(config);
    }
    let wheel_speed_output = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_outputs: [Output<ReactionWheelStateMsg>; 4] =
        std::array::from_fn(|index| reaction_wheels.wheels()[index].state_out.clone());

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 10.0,
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
    let mut magnetic_field = WmmField::new(WmmFieldConfig {
        name: "WMM".to_string(),
    });
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
        k: 0.0001,
        ki: -1.0,
        p: 0.002,
        integral_limit: -0.2,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut tam = Magnetometer::new(MagnetometerConfig {
        name: "TAM_sensor".to_string(),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        bias_t: Vector3::zeros(),
        p_matrix_sqrt_t: Matrix3::zeros(),
        a_matrix: Matrix3::identity(),
        walk_bounds_t: Vector3::zeros(),
        scale_factor: 1.0,
        min_output_t: -f64::MAX,
        max_output_t: f64::MAX,
    });
    let mut tam_comm = TamComm::new(TamCommConfig {
        name: "tamComm".to_string(),
        dcm_bs: Matrix3::identity(),
    });
    let mut wheel_speed_biases_radps = [0.0; MAX_EFF_COUNT];
    for (slot, rpm) in wheel_speed_biases_radps[..4]
        .iter_mut()
        .zip([800.0, 600.0, 400.0, 200.0])
    {
        *slot = rpm_to_radps(rpm);
    }
    let mut momentum_management = MtbMomentumManagement::new(MtbMomentumManagementConfig {
        name: "mtbMomentumManagement".to_string(),
        wheel_speed_biases_radps,
        c_gain_per_sec: 0.003,
    });
    let mut mtb_effector = MtbEffector::new("MtbEff");

    let output_path = scenario_output_path("scenarioMtbMomentumManagement.csv");
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioMtbMomentumManagement".to_string(),
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Scenario);

    let start_epoch = Epoch::from_gregorian_utc(2019, 6, 27, 10, 23, 0, 0);
    let mut simulation = Simulation::new(start_epoch, false);
    connect!(&simulation,
        &momentum_management.rw_motor_torque_out_msg => &mut reaction_wheels.rw_motor_cmd_in_msg,
        &momentum_management.mtb_cmd_out_msg => &mut mtb_effector.mtb_cmd_in_msg,
        &mtb_config_output => &mut mtb_effector.mtb_params_in_msg,
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
        &wheel_state_outputs[0] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_0")]),
        ),
        &wheel_state_outputs[1] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_1")]),
        ),
        &wheel_state_outputs[2] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_2")]),
        ),
        &wheel_state_outputs[3] => recorder.add_source::<ReactionWheelStateMsg>(
            CsvSourceConfig::columns([("applied_motor_torque_nm", "rwTorque_Nm_3")]),
        ),
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
        "recorder" => &mut recorder, period=SAMPLE_PERIOD_NANOS, priority=0;
    }
    simulation.run_for(DURATION_NANOS);

    println!("wrote {}", output_path.display());
}
