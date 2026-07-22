use std::path::{Path, PathBuf};

use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::fsw_algorithms::css_wls_est::{CssWlsEst, CssWlsEstConfig};
use basilisk_rs::fsw_algorithms::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::sun_safe_point::{SunSafePoint, SunSafePointConfig};
use basilisk_rs::messages::{Output, RwArrayConfigMsg, SunEphemerisMsg, VehicleConfigMsg};
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::sensors::imu_sensor::{ImuSensor, ImuSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvRecorder, CsvRecorderConfig};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use hifitime::Epoch;
use nalgebra::{Matrix3, Rotation3, UnitQuaternion, Vector3};

const STEP_NANOS: u64 = 100_000_000;
const DURATION_NANOS: u64 = 600_000_000_000;

struct ConstantSunEphemeris {
    pub output_sun_msg: Output<SunEphemerisMsg>,
    sun_position_inertial_m: Vector3<f64>,
}

impl ConstantSunEphemeris {
    fn new(sun_position_inertial_m: Vector3<f64>) -> Self {
        Self {
            output_sun_msg: Output::default(),
            sun_position_inertial_m,
        }
    }

    fn current_msg(&self) -> SunEphemerisMsg {
        SunEphemerisMsg {
            sun_position_inertial_m: self.sun_position_inertial_m,
            sun_velocity_inertial_mps: Vector3::zeros(),
        }
    }
}

impl Module for ConstantSunEphemeris {
    fn init(&mut self) {
        self.output_sun_msg.write(self.current_msg());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.output_sun_msg.write(self.current_msg());
    }
}

fn main() {
    let show_progress = std::env::var_os("SHOW_PROGRESS").is_some();
    let profile_sim = std::env::var_os("PROFILE_SIM").is_some();
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let output_dir = repo_root.join("examples/output/sun_pointing");
    let sun_position_inertial_m = Vector3::new(1.0e11, 0.0, 0.0);
    if output_dir.exists() {
        std::fs::remove_dir_all(&output_dir).expect("failed to clear previous sun-pointing output");
    }

    let mut sim = Simulation::new(
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        show_progress,
    );
    sim.set_timing_enabled(profile_sim);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 12.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::new(0.16, 0.0, 0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 0.22),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.0, 0.0),
        integrator: None,
    });

    let wheel_axes = [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)];
    let mut rw_allocator = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: wheel_axes.to_vec(),
    });
    let mut rw_array_config = RwArrayConfigMsg {
        num_rw: wheel_axes.len(),
        ..Default::default()
    };
    rw_array_config.spin_axes_body[..wheel_axes.len()].copy_from_slice(&wheel_axes);
    rw_array_config.spin_axis_inertias_kg_m2[..wheel_axes.len()].fill(0.002);
    rw_array_config.max_motor_torques_nm[..wheel_axes.len()].fill(0.1);
    let rw_array_config_output = Output::new(rw_array_config);
    connect!(&sim,
        &rw_array_config_output => &mut rw_allocator.rw_params_in_msg,
    );

    let mut rw_x_config = ReactionWheelStateEffectorConfig::balanced(
        "rw_x",
        Vector3::zeros(),
        wheel_axes[0],
        0.002,
        0.1,
    );
    rw_x_config.js_kg_m2 = 0.002;
    rw_x_config.max_speed_radps = 500.0;
    let mut rw_y_config = ReactionWheelStateEffectorConfig::balanced(
        "rw_y",
        Vector3::zeros(),
        wheel_axes[1],
        0.002,
        0.1,
    );
    rw_y_config.js_kg_m2 = 0.002;
    rw_y_config.max_speed_radps = 500.0;
    let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
    reaction_wheels.add_reaction_wheel(rw_x_config);
    reaction_wheels.add_reaction_wheel(rw_y_config);
    connect!(&sim,
        &rw_allocator.rw_motor_torque_out_msg => &mut reaction_wheels.rw_motor_cmd_in_msg,
    );
    spacecraft.add_state_effector(reaction_wheels);

    let mut sun_ephemeris = ConstantSunEphemeris::new(sun_position_inertial_m);
    let mut imu = ImuSensor::new(ImuSensorConfig {
        name: "imu".to_string(),
        position_m: Vector3::zeros(),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        rate_noise_std_radps: Vector3::zeros(),
    });

    let css_normals = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.0, 0.0, -1.0),
    ];
    let mut css_px = single_css("css_px", Vector3::zeros(), css_normals[0]);
    let mut css_mx = single_css("css_mx", Vector3::zeros(), css_normals[1]);
    let mut css_py = single_css("css_py", Vector3::zeros(), css_normals[2]);
    let mut css_my = single_css("css_my", Vector3::zeros(), css_normals[3]);
    let mut css_pz = single_css("css_pz", Vector3::zeros(), css_normals[4]);
    let mut css_mz = single_css("css_mz", Vector3::zeros(), css_normals[5]);

    let mut css_wls_est = CssWlsEst::new(CssWlsEstConfig {
        name: "cssWlsEst".to_string(),
        sensor_normals_body: css_normals.to_vec(),
        sensor_use_thresh: 0.001,
        use_weights: true,
    });
    let mut sun_safe_point = SunSafePoint::new(SunSafePointConfig {
        name: "sunSafePoint".to_string(),
        s_hat_bdy_cmd: Vector3::new(0.0, 0.0, 1.0),
        min_unit_mag: 0.1,
        small_angle: 1.0e-10,
        sun_axis_spin_rate_radps: 0.0,
        omega_rn_b_search_radps: Vector3::new(0.0, 0.0, 0.01),
    });
    let mut mrp_feedback = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: 0.08,
        ki: -1.0,
        p: 0.8,
        integral_limit: 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let vehicle_config_output = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: Matrix3::new(0.16, 0.0, 0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 0.22),
        mass_kg: 12.0,
        ..Default::default()
    });
    connect!(&sim,
        &vehicle_config_output => &mut mrp_feedback.veh_config_in_msg,
    );

    let mut spacecraft_recorder = csv_recorder("spacecraft_state", &output_dir);
    let mut imu_recorder = csv_recorder("imu", &output_dir);
    let mut css_px_recorder = csv_recorder("css_px", &output_dir);
    let mut css_mx_recorder = csv_recorder("css_mx", &output_dir);
    let mut css_py_recorder = csv_recorder("css_py", &output_dir);
    let mut css_my_recorder = csv_recorder("css_my", &output_dir);
    let mut css_pz_recorder = csv_recorder("css_pz", &output_dir);
    let mut css_mz_recorder = csv_recorder("css_mz", &output_dir);
    let mut sunline_recorder = csv_recorder("sunline", &output_dir);
    let mut guidance_recorder = csv_recorder("guidance", &output_dir);
    let mut body_torque_recorder = csv_recorder("body_torque", &output_dir);
    let mut rw_cmd_recorder = csv_recorder("rw_commands", &output_dir);

    connect!(&sim,
        &spacecraft.state_out => &mut imu.input_state_msg,
    );
    for css in [
        &mut css_px,
        &mut css_mx,
        &mut css_py,
        &mut css_my,
        &mut css_pz,
        &mut css_mz,
    ] {
        connect!(&sim,
            &spacecraft.state_out => &mut css.input_state_msg,
            &sun_ephemeris.output_sun_msg => &mut css.input_sun_msg,
        );
    }
    for (input, sensor_output) in css_wls_est.css_data_in_msgs.iter_mut().zip([
        &css_px.output_sun_sensor_msg,
        &css_mx.output_sun_sensor_msg,
        &css_py.output_sun_sensor_msg,
        &css_my.output_sun_sensor_msg,
        &css_pz.output_sun_sensor_msg,
        &css_mz.output_sun_sensor_msg,
    ]) {
        connect!(&sim, sensor_output => input);
    }
    connect!(&sim,
        &css_wls_est.nav_state_out_msg => &mut sun_safe_point.sun_direction_in_msg,
        &imu.output_imu_msg => &mut sun_safe_point.imu_in_msg,
        &sun_safe_point.att_guidance_out_msg => &mut mrp_feedback.guid_in_msg,
        &mrp_feedback.cmd_torque_out_msg => &mut rw_allocator.veh_control_in_msg,
    );

    connect!(&sim,
        &spacecraft.state_out => spacecraft_recorder.add_source(""),
        &imu.output_imu_msg => imu_recorder.add_source(""),
        &css_px.output_sun_sensor_msg => css_px_recorder.add_source(""),
        &css_mx.output_sun_sensor_msg => css_mx_recorder.add_source(""),
        &css_py.output_sun_sensor_msg => css_py_recorder.add_source(""),
        &css_my.output_sun_sensor_msg => css_my_recorder.add_source(""),
        &css_pz.output_sun_sensor_msg => css_pz_recorder.add_source(""),
        &css_mz.output_sun_sensor_msg => css_mz_recorder.add_source(""),
        &css_wls_est.nav_state_out_msg => sunline_recorder.add_source(""),
        &sun_safe_point.att_guidance_out_msg => guidance_recorder.add_source(""),
        &mrp_feedback.cmd_torque_out_msg => body_torque_recorder.add_source(""),
        &rw_allocator.rw_motor_torque_out_msg => rw_cmd_recorder.add_source(""),
    );

    const PRIORITY_ENV: i32 = 70;
    const PRIORITY_DYNAMICS: i32 = 60;
    const PRIORITY_SENSORS: i32 = 50;
    const PRIORITY_ESTIMATION: i32 = 40;
    const PRIORITY_GUIDANCE: i32 = 30;
    const PRIORITY_CONTROL: i32 = 20;
    const PRIORITY_ALLOCATION: i32 = 10;
    const PRIORITY_RECORD: i32 = 0;

    schedule! { sim,
        "sun_ephemeris" => &mut sun_ephemeris, STEP_NANOS, PRIORITY_ENV;
        "spacecraft" => &mut spacecraft, STEP_NANOS, PRIORITY_DYNAMICS;
        "imu" => &mut imu, STEP_NANOS, PRIORITY_SENSORS;
        "css_px" => &mut css_px, STEP_NANOS, PRIORITY_SENSORS;
        "css_mx" => &mut css_mx, STEP_NANOS, PRIORITY_SENSORS;
        "css_py" => &mut css_py, STEP_NANOS, PRIORITY_SENSORS;
        "css_my" => &mut css_my, STEP_NANOS, PRIORITY_SENSORS;
        "css_pz" => &mut css_pz, STEP_NANOS, PRIORITY_SENSORS;
        "css_mz" => &mut css_mz, STEP_NANOS, PRIORITY_SENSORS;
        "cssWlsEst" => &mut css_wls_est, STEP_NANOS, PRIORITY_ESTIMATION;
        "sunSafePoint" => &mut sun_safe_point, STEP_NANOS, PRIORITY_GUIDANCE;
        "mrpFeedback" => &mut mrp_feedback, STEP_NANOS, PRIORITY_CONTROL;
        "rwMotorTorque" => &mut rw_allocator, STEP_NANOS, PRIORITY_ALLOCATION;
        "spacecraft_state_recorder" => &mut spacecraft_recorder, STEP_NANOS, PRIORITY_RECORD;
        "imu_recorder" => &mut imu_recorder, STEP_NANOS, PRIORITY_RECORD;
        "css_px_recorder" => &mut css_px_recorder, STEP_NANOS, PRIORITY_RECORD;
        "css_mx_recorder" => &mut css_mx_recorder, STEP_NANOS, PRIORITY_RECORD;
        "css_py_recorder" => &mut css_py_recorder, STEP_NANOS, PRIORITY_RECORD;
        "css_my_recorder" => &mut css_my_recorder, STEP_NANOS, PRIORITY_RECORD;
        "css_pz_recorder" => &mut css_pz_recorder, STEP_NANOS, PRIORITY_RECORD;
        "css_mz_recorder" => &mut css_mz_recorder, STEP_NANOS, PRIORITY_RECORD;
        "sunline_recorder" => &mut sunline_recorder, STEP_NANOS, PRIORITY_RECORD;
        "guidance_recorder" => &mut guidance_recorder, STEP_NANOS, PRIORITY_RECORD;
        "body_torque_recorder" => &mut body_torque_recorder, STEP_NANOS, PRIORITY_RECORD;
        "rw_command_recorder" => &mut rw_cmd_recorder, STEP_NANOS, PRIORITY_RECORD;
    }

    sim.run_for(DURATION_NANOS);
    let module_timings = if profile_sim {
        sim.module_timings()
    } else {
        Vec::new()
    };
    drop(sim);

    let final_state = spacecraft.state_out.read();
    let body_z_inertial = final_state
        .body_to_inertial()
        .transform_vector(&Vector3::new(0.0, 0.0, 1.0));
    let sun_hat_inertial = sun_position_inertial_m.normalize();
    let final_error_rad = body_z_inertial
        .dot(&sun_hat_inertial)
        .clamp(-1.0, 1.0)
        .acos();

    if profile_sim {
        println!("module_timings_ms =");
        for timing in module_timings.into_iter().take(12) {
            println!(
                "  {:>24}  priority={:>2}  calls={:>6}  total_ms={:>10.3}",
                timing.name,
                timing.priority,
                timing.num_updates,
                timing.total_update_nanos as f64 * 1.0e-6,
            );
        }
    }

    println!("output_dir = {}", output_dir.display());
    println!("show_progress = {}", show_progress);
    println!("profile_sim = {}", profile_sim);
    println!(
        "final_pointing_error_deg = {:.6}",
        final_error_rad.to_degrees()
    );
    println!("final_body_rates_radps = {:?}", final_state.omega_radps);
    println!(
        "sunline_estimate = {:?}",
        css_wls_est.nav_state_out_msg.read()
    );
}

fn single_css(
    name: &str,
    position_m: Vector3<f64>,
    boresight_body: Vector3<f64>,
) -> CoarseSunSensor {
    CoarseSunSensor::new(CoarseSunSensorConfig {
        name: name.to_string(),
        position_m,
        body_to_sensor_quaternion: body_to_sensor_for_boresight(boresight_body),
        fov_half_angle_rad: 90.0_f64.to_radians(),
        scale_factor: 1.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 1.0,
        walk_bounds: -1.0,
        min_output: 0.0,
        max_output: 1.0,
    })
}

fn body_to_sensor_for_boresight(boresight_body: Vector3<f64>) -> UnitQuaternion<f64> {
    let sensor_z_body = boresight_body.normalize();
    let reference_body = if sensor_z_body.cross(&Vector3::z()).norm() > 1.0e-12 {
        Vector3::z()
    } else {
        Vector3::y()
    };
    let sensor_x_body = reference_body.cross(&sensor_z_body).normalize();
    let sensor_y_body = sensor_z_body.cross(&sensor_x_body);
    let sensor_to_body = Matrix3::from_columns(&[sensor_x_body, sensor_y_body, sensor_z_body]);
    let body_to_sensor = Rotation3::from_matrix_unchecked(sensor_to_body.transpose());
    UnitQuaternion::from_rotation_matrix(&body_to_sensor)
}

fn csv_recorder(topic: &str, output_dir: &Path) -> CsvRecorder {
    CsvRecorder::new(CsvRecorderConfig {
        topic: topic.to_string(),
        output_path: output_dir.join(format!("{topic}.csv")),
    })
}
