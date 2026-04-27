use std::path::{Path, PathBuf};

use basilisk_rs::fsw::css_wls_est::{CssWlsEst, CssWlsEstConfig};
use basilisk_rs::fsw::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw::sun_safe_point::{SunSafePoint, SunSafePointConfig};
use basilisk_rs::imu::{Imu, ImuConfig};
use basilisk_rs::messages::{Output, SunEphemerisMsg};
use basilisk_rs::reaction_wheel::{ReactionWheel, ReactionWheelConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::sun_sensor::{SunSensor, SunSensorConfig};
use basilisk_rs::telemetry::{CsvRecorder, CsvRecorderConfig};
use basilisk_rs::{Module, SimulationContext};
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
        inertia_kg_m2: Matrix3::new(0.16, 0.0, 0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 0.22),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.0, 0.0),
    });

    let wheel_axes = [Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)];
    let mut rw_allocator = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: wheel_axes.to_vec(),
        wheel_spin_axes_body: wheel_axes.to_vec(),
    });

    let mut rw_x_config =
        ReactionWheelConfig::balanced("rw_x", Vector3::zeros(), wheel_axes[0], 0.002, 0.1);
    rw_x_config.js_kg_m2 = 0.002;
    rw_x_config.max_speed_radps = 500.0;
    let mut rw_x = ReactionWheel::new(rw_x_config);
    let mut rw_y_config =
        ReactionWheelConfig::balanced("rw_y", Vector3::zeros(), wheel_axes[1], 0.002, 0.1);
    rw_y_config.js_kg_m2 = 0.002;
    rw_y_config.max_speed_radps = 500.0;
    let mut rw_y = ReactionWheel::new(rw_y_config);
    sim.connect(
        &rw_allocator.rw_motor_torque_out_msgs[0],
        &mut rw_x.command_in,
    );
    sim.connect(
        &rw_allocator.rw_motor_torque_out_msgs[1],
        &mut rw_y.command_in,
    );
    spacecraft.add_state_effector(rw_x);
    spacecraft.add_state_effector(rw_y);

    let mut sun_ephemeris = ConstantSunEphemeris::new(sun_position_inertial_m);
    let mut imu = Imu::new(ImuConfig {
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
        inertia_kg_m2: Matrix3::new(0.16, 0.0, 0.0, 0.0, 0.18, 0.0, 0.0, 0.0, 0.22),
        k: 0.08,
        ki: -1.0,
        p: 0.8,
        integral_limit: 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });

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
    let mut rw_x_cmd_recorder = csv_recorder("rw_x_cmd", &output_dir);
    let mut rw_y_cmd_recorder = csv_recorder("rw_y_cmd", &output_dir);

    sim.connect(&spacecraft.state_out, &mut imu.input_state_msg);
    for css in [
        &mut css_px,
        &mut css_mx,
        &mut css_py,
        &mut css_my,
        &mut css_pz,
        &mut css_mz,
    ] {
        sim.connect(&spacecraft.state_out, &mut css.input_state_msg);
        sim.connect(&sun_ephemeris.output_sun_msg, &mut css.input_sun_msg);
    }
    for (input, sensor_output) in css_wls_est.css_data_in_msgs.iter_mut().zip([
        &css_px.output_sun_sensor_msg,
        &css_mx.output_sun_sensor_msg,
        &css_py.output_sun_sensor_msg,
        &css_my.output_sun_sensor_msg,
        &css_pz.output_sun_sensor_msg,
        &css_mz.output_sun_sensor_msg,
    ]) {
        sim.connect(sensor_output, input);
    }
    sim.connect(
        &css_wls_est.nav_state_out_msg,
        &mut sun_safe_point.sun_direction_in_msg,
    );
    sim.connect(&imu.output_imu_msg, &mut sun_safe_point.imu_in_msg);
    sim.connect(
        &sun_safe_point.att_guidance_out_msg,
        &mut mrp_feedback.guid_in_msg,
    );
    sim.connect(
        &mrp_feedback.cmd_torque_out_msg,
        &mut rw_allocator.veh_control_in_msg,
    );

    sim.connect(&spacecraft.state_out, &mut spacecraft_recorder.input_msg);
    sim.connect(&imu.output_imu_msg, &mut imu_recorder.input_msg);
    sim.connect(
        &css_px.output_sun_sensor_msg,
        &mut css_px_recorder.input_msg,
    );
    sim.connect(
        &css_mx.output_sun_sensor_msg,
        &mut css_mx_recorder.input_msg,
    );
    sim.connect(
        &css_py.output_sun_sensor_msg,
        &mut css_py_recorder.input_msg,
    );
    sim.connect(
        &css_my.output_sun_sensor_msg,
        &mut css_my_recorder.input_msg,
    );
    sim.connect(
        &css_pz.output_sun_sensor_msg,
        &mut css_pz_recorder.input_msg,
    );
    sim.connect(
        &css_mz.output_sun_sensor_msg,
        &mut css_mz_recorder.input_msg,
    );
    sim.connect(
        &css_wls_est.nav_state_out_msg,
        &mut sunline_recorder.input_msg,
    );
    sim.connect(
        &sun_safe_point.att_guidance_out_msg,
        &mut guidance_recorder.input_msg,
    );
    sim.connect(
        &mrp_feedback.cmd_torque_out_msg,
        &mut body_torque_recorder.input_msg,
    );
    sim.connect(
        &rw_allocator.rw_motor_torque_out_msgs[0],
        &mut rw_x_cmd_recorder.input_msg,
    );
    sim.connect(
        &rw_allocator.rw_motor_torque_out_msgs[1],
        &mut rw_y_cmd_recorder.input_msg,
    );

    const PRIORITY_ENV: i32 = 0;
    const PRIORITY_DYNAMICS: i32 = 10;
    const PRIORITY_SENSORS: i32 = 20;
    const PRIORITY_ESTIMATION: i32 = 30;
    const PRIORITY_GUIDANCE: i32 = 40;
    const PRIORITY_CONTROL: i32 = 50;
    const PRIORITY_ALLOCATION: i32 = 60;
    const PRIORITY_RECORD: i32 = 70;

    sim.add_module(
        "sun_ephemeris",
        &mut sun_ephemeris,
        STEP_NANOS,
        PRIORITY_ENV,
    );
    sim.add_module("spacecraft", &mut spacecraft, STEP_NANOS, PRIORITY_DYNAMICS);
    sim.add_module("imu", &mut imu, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module("css_px", &mut css_px, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module("css_mx", &mut css_mx, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module("css_py", &mut css_py, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module("css_my", &mut css_my, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module("css_pz", &mut css_pz, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module("css_mz", &mut css_mz, STEP_NANOS, PRIORITY_SENSORS);
    sim.add_module(
        "cssWlsEst",
        &mut css_wls_est,
        STEP_NANOS,
        PRIORITY_ESTIMATION,
    );
    sim.add_module(
        "sunSafePoint",
        &mut sun_safe_point,
        STEP_NANOS,
        PRIORITY_GUIDANCE,
    );
    sim.add_module(
        "mrpFeedback",
        &mut mrp_feedback,
        STEP_NANOS,
        PRIORITY_CONTROL,
    );
    sim.add_module(
        "rwMotorTorque",
        &mut rw_allocator,
        STEP_NANOS,
        PRIORITY_ALLOCATION,
    );

    sim.add_module(
        "spacecraft_state_recorder",
        &mut spacecraft_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "imu_recorder",
        &mut imu_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "css_px_recorder",
        &mut css_px_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "css_mx_recorder",
        &mut css_mx_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "css_py_recorder",
        &mut css_py_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "css_my_recorder",
        &mut css_my_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "css_pz_recorder",
        &mut css_pz_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "css_mz_recorder",
        &mut css_mz_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "sunline_recorder",
        &mut sunline_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "guidance_recorder",
        &mut guidance_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "body_torque_recorder",
        &mut body_torque_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "rw_x_cmd_recorder",
        &mut rw_x_cmd_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );
    sim.add_module(
        "rw_y_cmd_recorder",
        &mut rw_y_cmd_recorder,
        STEP_NANOS,
        PRIORITY_RECORD,
    );

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

fn single_css(name: &str, position_m: Vector3<f64>, boresight_body: Vector3<f64>) -> SunSensor {
    SunSensor::new(SunSensorConfig {
        name: name.to_string(),
        position_m,
        body_to_sensor_quaternion: body_to_sensor_for_boresight(boresight_body),
        fov_half_angle_rad: 90.0_f64.to_radians(),
        scale_factor: 1.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
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

fn csv_recorder<T>(topic: &str, output_dir: &Path) -> CsvRecorder<T> {
    CsvRecorder::new(CsvRecorderConfig {
        topic: topic.to_string(),
        output_path: output_dir.join(format!("{topic}.csv")),
    })
}
