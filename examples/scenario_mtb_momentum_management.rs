//! Parity case for `scenarioMtbMomentumManagement.py`.
//!
//! Four reaction wheels, four magnetic torque bars, WMM2025, and the complete
//! attitude/momentum-management control chain. The binary only writes CSV.

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
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, Input, MAX_EFF_COUNT, MagneticFieldMsg,
    MtbArrayCommandMsg, MtbArrayConfigMsg, Output, ReactionWheelStateMsg, RwArrayConfigMsg,
    RwSpeedMsg, TamSensorBodyMsg, TamSensorMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::magnetometer::{Magnetometer, MagnetometerConfig};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{
    CsvFormat, CsvRecorder, CsvRecorderConfig, TelemetryField, TelemetryMessage,
};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use common::{
    MU_EARTH_M3PS2, array_fields, elem2rv, rpm_to_radps, scenario_output_path, seconds,
    vector_fields,
};
use hifitime::Epoch;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

const TASK_PERIOD_NANOS: u64 = seconds(2);
const SAMPLE_PERIOD_NANOS: u64 = seconds(36);
const DURATION_NANOS: u64 = seconds(7_200);

#[derive(Clone, Debug)]
struct MtbTelemetry {
    requested_motor_torque_nm: [f64; MAX_EFF_COUNT],
    guidance: AttitudeGuidanceMsg,
    wheel_speeds_radps: [f64; MAX_EFF_COUNT],
    applied_motor_torque_nm: [f64; 4],
    magnetic_field_inertial_t: Vector3<f64>,
    tam_sensor_t: Vector3<f64>,
    tam_body_t: Vector3<f64>,
    dipole_cmds_am2: [f64; MAX_EFF_COUNT],
}

impl Default for MtbTelemetry {
    fn default() -> Self {
        Self {
            requested_motor_torque_nm: [0.0; MAX_EFF_COUNT],
            guidance: AttitudeGuidanceMsg::default(),
            wheel_speeds_radps: [0.0; MAX_EFF_COUNT],
            applied_motor_torque_nm: [0.0; 4],
            magnetic_field_inertial_t: Vector3::zeros(),
            tam_sensor_t: Vector3::zeros(),
            tam_body_t: Vector3::zeros(),
            dipole_cmds_am2: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl TelemetryMessage for MtbTelemetry {
    fn flatten(&self) -> Vec<TelemetryField> {
        let mut fields = array_fields("motorTorque_Nm", &self.requested_motor_torque_nm);
        fields.extend(vector_fields("sigma_BR", self.guidance.sigma_br));
        fields.extend(vector_fields(
            "omega_BR_B_rad_per_s",
            self.guidance.omega_br_b_radps,
        ));
        fields.extend(array_fields(
            "wheelSpeeds_rad_per_s",
            &self.wheel_speeds_radps,
        ));
        fields.extend(array_fields("rwTorque_Nm", &self.applied_motor_torque_nm));
        fields.extend(vector_fields(
            "magField_N_T",
            self.magnetic_field_inertial_t,
        ));
        fields.extend(vector_fields("tam_S_T", self.tam_sensor_t));
        fields.extend(vector_fields("tam_B_T", self.tam_body_t));
        fields.extend(array_fields("mtbDipoleCmds_A_m2", &self.dipole_cmds_am2));
        fields
    }
}

#[derive(Clone, Debug, Default)]
struct MtbCollector {
    requested_torque_in_msg: Input<ArrayMotorTorqueMsg>,
    guidance_in_msg: Input<AttitudeGuidanceMsg>,
    wheel_speeds_in_msg: Input<RwSpeedMsg>,
    wheel_state_in_msgs: [Input<ReactionWheelStateMsg>; 4],
    magnetic_field_in_msg: Input<MagneticFieldMsg>,
    tam_sensor_in_msg: Input<TamSensorMsg>,
    tam_body_in_msg: Input<TamSensorBodyMsg>,
    dipole_command_in_msg: Input<MtbArrayCommandMsg>,
    telemetry_out_msg: Output<MtbTelemetry>,
}

impl Module for MtbCollector {
    fn init(&mut self) {
        self.telemetry_out_msg.write(MtbTelemetry::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.telemetry_out_msg.write(MtbTelemetry {
            requested_motor_torque_nm: self.requested_torque_in_msg.read().motor_torque_nm,
            guidance: self.guidance_in_msg.read(),
            wheel_speeds_radps: self.wheel_speeds_in_msg.read().wheel_speeds_radps,
            applied_motor_torque_nm: self
                .wheel_state_in_msgs
                .each_ref()
                .map(|input| input.read().applied_motor_torque_nm),
            magnetic_field_inertial_t: self.magnetic_field_in_msg.read().magnetic_field_inertial_t,
            tam_sensor_t: self.tam_sensor_in_msg.read().magnetic_field_sensor_t,
            tam_body_t: self.tam_body_in_msg.read().magnetic_field_body_t,
            dipole_cmds_am2: self.dipole_command_in_msg.read().dipole_cmds_am2,
        });
    }
}

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

    // Preserve the reference scenario's deliberately truncated diagonal-axis
    // literal instead of substituting Rust's higher-precision constant.
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
    let mut collector = MtbCollector::default();
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioMtbMomentumManagement".to_string(),
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Reference);

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
        &rw_motor_torque.rw_motor_torque_out_msg => &mut collector.requested_torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut collector.guidance_in_msg,
        &wheel_speed_output => &mut collector.wheel_speeds_in_msg,
        &wheel_state_outputs[0] => &mut collector.wheel_state_in_msgs[0],
        &wheel_state_outputs[1] => &mut collector.wheel_state_in_msgs[1],
        &wheel_state_outputs[2] => &mut collector.wheel_state_in_msgs[2],
        &wheel_state_outputs[3] => &mut collector.wheel_state_in_msgs[3],
        &magnetic_field.output_magnetic_field_msg => &mut collector.magnetic_field_in_msg,
        &tam.output_tam_msg => &mut collector.tam_sensor_in_msg,
        &tam_comm.tam_out_msg => &mut collector.tam_body_in_msg,
        &momentum_management.mtb_cmd_out_msg => &mut collector.dipole_command_in_msg,
        &collector.telemetry_out_msg => &mut recorder.input_msg,
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, TASK_PERIOD_NANOS, 100;
        "navigation" => &mut navigation, TASK_PERIOD_NANOS, 90;
        "magnetic_field" => &mut magnetic_field, TASK_PERIOD_NANOS, 80;
        "inertial_reference" => &mut inertial_reference, TASK_PERIOD_NANOS, 70;
        "tracking_error" => &mut tracking_error, TASK_PERIOD_NANOS, 60;
        "mrp_feedback" => &mut mrp_feedback, TASK_PERIOD_NANOS, 50;
        "rw_motor_torque" => &mut rw_motor_torque, TASK_PERIOD_NANOS, 40;
        "tam" => &mut tam, TASK_PERIOD_NANOS, 30;
        "tam_comm" => &mut tam_comm, TASK_PERIOD_NANOS, 20;
        "momentum_management" => &mut momentum_management, TASK_PERIOD_NANOS, 10;
        "collector" => &mut collector, SAMPLE_PERIOD_NANOS, 5;
        "recorder" => &mut recorder, SAMPLE_PERIOD_NANOS, 0;
    }
    simulation.run_for(DURATION_NANOS);

    println!("wrote {}", output_path.display());
}
