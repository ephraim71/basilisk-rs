use std::path::{Path, PathBuf};

use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME, MOON_J2000};
use basilisk_rs::atmosphere::{MsisAtmosphere, MsisAtmosphereConfig};
use basilisk_rs::drag::{Drag, DragConfig};
use basilisk_rs::eclipse::{Eclipse, EclipseConfig};
use basilisk_rs::ephemeris::{AnisePlanetEphemeris, AnisePlanetEphemerisConfig};
use basilisk_rs::gps::{Gps, GpsConfig};
use basilisk_rs::gravity::GravBodyData;
use basilisk_rs::imu::{Imu, ImuConfig};
use basilisk_rs::magnetic_field::{IgrfField, IgrfFieldConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::srp::{SolarRadiationPressure, SolarRadiationPressureConfig};
use basilisk_rs::star_tracker::{StarTracker, StarTrackerConfig};
use basilisk_rs::sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::sun_sensor::{SunSensor, SunSensorConfig};
use basilisk_rs::tam::{Tam, TamConfig};
use basilisk_rs::telemetry::{CsvRecorder, CsvRecorderConfig};
use hifitime::Epoch;
use nalgebra::{Matrix3, Rotation3, SMatrix, SVector, UnitQuaternion, Vector3};

fn main() {
    let simulation_duration_nanos = 300_000_000_000_u64;
    let enable_recording = std::env::var_os("ENABLE_RECORDING").is_some();
    let show_progress = std::env::var_os("SHOW_PROGRESS").is_some();
    let profile_sim = std::env::var_os("PROFILE_SIM").is_some();
    const PRIORITY_ENVIRONMENT: i32 = 0;
    const PRIORITY_SENSORS: i32 = 10;
    const PRIORITY_ACTUATORS: i32 = 20;
    const PRIORITY_RECORDERS: i32 = 30;
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let output_dir = repo_root.join("examples/output/full_satellite_stack");
    if enable_recording && output_dir.exists() {
        std::fs::remove_dir_all(&output_dir).expect("failed to clear previous sensor suite output");
    }

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 18.0,
        inertia_kg_m2: Matrix3::new(0.16, 0.001, 0.0, 0.001, 0.21, 0.0, 0.0, 0.0, 0.28),
        integration_step_nanos: 5_000_000,
        initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
        initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.01, 0.02, 0.015),
    });
    spacecraft.set_timing_enabled(profile_sim);

    spacecraft.add_grav_body(
        GravBodyData::spherical_harmonics_from_file(
            "earth",
            "assets/gravity/GGM03S.txt",
            20,
            true,
            Vector3::zeros(),
            Vector3::zeros(),
        )
        .with_anise_orientation(
            repo_root.join("assets/anise/pck11.pca"),
            &[repo_root.join("assets/anise/earth_latest_high_prec.bpc")],
            EARTH_J2000,
            IAU_EARTH_FRAME,
        ),
    );
    spacecraft.add_grav_body(GravBodyData::point_mass(
        "sun",
        1.327_124_400_18e20,
        false,
        Vector3::zeros(),
        Vector3::zeros(),
    ));
    spacecraft.add_grav_body(GravBodyData::point_mass(
        "moon",
        4.904_869_5e12,
        false,
        Vector3::zeros(),
        Vector3::zeros(),
    ));

    let mut sun_ephemeris = AniseSunEphemeris::new(AniseSunEphemerisConfig {
        name: "sun_ephemeris".to_string(),
        spk_path: repo_root.join("assets/anise/de440s.bsp"),
    });
    sun_ephemeris.set_timing_enabled(profile_sim);
    let mut earth_ephemeris = AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
        name: "earth_ephemeris".to_string(),
        spk_path: repo_root.join("assets/anise/de440s.bsp"),
        additional_kernel_paths: vec![
            repo_root.join("assets/anise/pck11.pca"),
            repo_root.join("assets/anise/earth_latest_high_prec.bpc"),
        ],
        source_frame: EARTH_J2000,
        observer_frame: EARTH_J2000,
        fixed_frame: Some(IAU_EARTH_FRAME),
    });
    earth_ephemeris.set_timing_enabled(profile_sim);
    let mut moon_ephemeris = AnisePlanetEphemeris::new(AnisePlanetEphemerisConfig {
        name: "moon_ephemeris".to_string(),
        spk_path: repo_root.join("assets/anise/de440s.bsp"),
        additional_kernel_paths: vec![],
        source_frame: MOON_J2000,
        observer_frame: EARTH_J2000,
        fixed_frame: None,
    });
    moon_ephemeris.set_timing_enabled(profile_sim);
    let mut atmosphere = MsisAtmosphere::new(MsisAtmosphereConfig {
        name: "earth_atmosphere".to_string(),
        planet_radius_m: 6_378_136.3,
        first_kernel_path: repo_root.join("assets/anise/pck11.pca"),
        additional_kernel_paths: vec![repo_root.join("assets/anise/earth_latest_high_prec.bpc")],
        inertial_frame: EARTH_J2000,
        fixed_frame: IAU_EARTH_FRAME,
        ap_daily: 4.0,
        ap_3hr: 4.0,
        f107_daily: 150.0,
        f107_average: 150.0,
    });
    atmosphere.set_timing_enabled(profile_sim);
    let mut eclipse = Eclipse::new(EclipseConfig {
        name: "earth_eclipse".to_string(),
        occulting_body_radius_m: 6_378_136.3,
    });

    let mut magnetic_field = IgrfField::new(IgrfFieldConfig {
        name: "earth_igrf".to_string(),
        first_kernel_path: repo_root.join("assets/anise/pck11.pca"),
        additional_kernel_paths: vec![repo_root.join("assets/anise/earth_latest_high_prec.bpc")],
        inertial_frame: EARTH_J2000,
        fixed_frame: IAU_EARTH_FRAME,
    });
    magnetic_field.set_timing_enabled(profile_sim);

    let mut imu_1 = Imu::new(ImuConfig {
        name: "imu_1".to_string(),
        position_m: Vector3::new(0.0, 0.0, 0.0),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        rate_noise_std_radps: Vector3::new(1.0e-4, 1.0e-4, 1.0e-4),
    });
    let mut imu_2 = Imu::new(ImuConfig {
        name: "imu_2".to_string(),
        position_m: Vector3::new(0.05, 0.0, 0.0),
        body_to_sensor_quaternion: UnitQuaternion::from_euler_angles(
            0.0,
            0.0,
            std::f64::consts::FRAC_PI_2,
        ),
        rate_noise_std_radps: Vector3::new(1.0e-4, 1.0e-4, 1.0e-4),
    });

    let mut tam_1 = Tam::new(TamConfig {
        name: "tam_1".to_string(),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        bias_t: Vector3::zeros(),
        p_matrix_sqrt_t: Matrix3::from_diagonal(&Vector3::new(1.0e-8, 1.0e-8, 1.0e-8)),
        a_matrix: Matrix3::identity(),
        walk_bounds_t: Vector3::new(1.0e-6, 1.0e-6, 1.0e-6),
        scale_factor: 1.0,
        min_output_t: -1.0,
        max_output_t: 1.0,
    });
    let mut tam_2 = Tam::new(TamConfig {
        name: "tam_2".to_string(),
        body_to_sensor_quaternion: UnitQuaternion::from_euler_angles(
            0.0,
            std::f64::consts::FRAC_PI_2,
            0.0,
        ),
        bias_t: Vector3::zeros(),
        p_matrix_sqrt_t: Matrix3::from_diagonal(&Vector3::new(1.0e-8, 1.0e-8, 1.0e-8)),
        a_matrix: Matrix3::identity(),
        walk_bounds_t: Vector3::new(1.0e-6, 1.0e-6, 1.0e-6),
        scale_factor: 1.0,
        min_output_t: -1.0,
        max_output_t: 1.0,
    });

    let mut sun_sensor_px = single_sun_sensor(
        "sun_sensor_px",
        Vector3::new(0.10, 0.0, 0.0),
        Vector3::new(1.0, 0.0, 0.0),
    );
    let mut sun_sensor_mx = single_sun_sensor(
        "sun_sensor_mx",
        Vector3::new(-0.10, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
    );
    let mut sun_sensor_py = single_sun_sensor(
        "sun_sensor_py",
        Vector3::new(0.0, 0.10, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
    );
    let mut sun_sensor_my = single_sun_sensor(
        "sun_sensor_my",
        Vector3::new(0.0, -0.10, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
    );
    let mut sun_sensor_pz = single_sun_sensor(
        "sun_sensor_pz",
        Vector3::new(0.0, 0.0, 0.10),
        Vector3::new(0.0, 0.0, 1.0),
    );
    let mut sun_sensor_mz = single_sun_sensor(
        "sun_sensor_mz",
        Vector3::new(0.0, 0.0, -0.10),
        Vector3::new(0.0, 0.0, -1.0),
    );

    let mut gps = Gps::new(GpsConfig {
        name: "gps_1".to_string(),
        p_matrix_sqrt: SMatrix::<f64, 6, 6>::from_diagonal(&SVector::<f64, 6>::from_row_slice(&[
            1.0, 1.0, 1.0, 0.01, 0.01, 0.01,
        ])),
        a_matrix: SMatrix::<f64, 6, 6>::identity(),
        walk_bounds: SVector::<f64, 6>::from_row_slice(&[100.0, 100.0, 100.0, 1.0, 1.0, 1.0]),
        cross_trans: true,
    });

    let mut star_tracker_1 = StarTracker::new(StarTrackerConfig {
        name: "st_1".to_string(),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        p_matrix_sqrt_rad: Matrix3::from_diagonal(&Vector3::new(1.0e-5, 1.0e-5, 1.0e-5)),
        a_matrix: Matrix3::identity(),
        walk_bounds_rad: Vector3::new(1.0e-3, 1.0e-3, 1.0e-3),
    });
    let mut star_tracker_2 = StarTracker::new(StarTrackerConfig {
        name: "st_2".to_string(),
        body_to_sensor_quaternion: UnitQuaternion::from_euler_angles(
            0.0,
            std::f64::consts::PI,
            0.0,
        ),
        p_matrix_sqrt_rad: Matrix3::from_diagonal(&Vector3::new(1.0e-5, 1.0e-5, 1.0e-5)),
        a_matrix: Matrix3::identity(),
        walk_bounds_rad: Vector3::new(1.0e-3, 1.0e-3, 1.0e-3),
    });
    let mut drag = Drag::new(DragConfig {
        name: "drag".to_string(),
        projected_area_m2: 0.18,
        drag_coeff: 2.2,
        com_offset_m: Vector3::new(0.01, 0.0, 0.0),
        planet_rotation_rate_radps: Vector3::new(0.0, 0.0, 7.292_115_9e-5),
    });
    let mut srp = SolarRadiationPressure::new(SolarRadiationPressureConfig {
        name: "srp".to_string(),
        area_m2: 0.18,
        coefficient_reflection: 1.2,
    });
    let mut spacecraft_recorder = csv_recorder("spacecraft_state", &output_dir);
    let mut sun_ephemeris_recorder = csv_recorder("sun_ephemeris", &output_dir);
    let mut moon_ephemeris_recorder = csv_recorder("moon_ephemeris", &output_dir);
    let mut atmosphere_recorder = csv_recorder("atmosphere", &output_dir);
    let mut eclipse_recorder = csv_recorder("eclipse", &output_dir);
    let mut magnetic_field_recorder = csv_recorder("magnetic_field", &output_dir);
    let mut imu_1_recorder = csv_recorder("imu_1", &output_dir);
    let mut imu_2_recorder = csv_recorder("imu_2", &output_dir);
    let mut tam_1_recorder = csv_recorder("tam_1", &output_dir);
    let mut tam_2_recorder = csv_recorder("tam_2", &output_dir);
    let mut gps_recorder = csv_recorder("gps", &output_dir);
    let mut st_1_recorder = csv_recorder("star_tracker_1", &output_dir);
    let mut st_2_recorder = csv_recorder("star_tracker_2", &output_dir);
    let mut sun_sensor_px_recorder = csv_recorder("sun_sensor_px", &output_dir);
    let mut sun_sensor_mx_recorder = csv_recorder("sun_sensor_mx", &output_dir);
    let mut sun_sensor_py_recorder = csv_recorder("sun_sensor_py", &output_dir);
    let mut sun_sensor_my_recorder = csv_recorder("sun_sensor_my", &output_dir);
    let mut sun_sensor_pz_recorder = csv_recorder("sun_sensor_pz", &output_dir);
    let mut sun_sensor_mz_recorder = csv_recorder("sun_sensor_mz", &output_dir);

    let mut sim = Simulation::new(
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        show_progress,
    );
    sim.set_timing_enabled(profile_sim);

    sim.connect(
        &earth_ephemeris.output_planet_msg,
        spacecraft
            .grav_body_input_mut("earth")
            .expect("missing earth gravity body input"),
    );
    sim.connect(
        &sun_ephemeris.output_planet_msg,
        spacecraft
            .grav_body_input_mut("sun")
            .expect("missing sun gravity body input"),
    );
    sim.connect(
        &moon_ephemeris.output_planet_msg,
        spacecraft
            .grav_body_input_mut("moon")
            .expect("missing moon gravity body input"),
    );
    sim.connect(&spacecraft.state_out, &mut atmosphere.input_state_msg);
    sim.connect(
        &earth_ephemeris.output_planet_msg,
        &mut atmosphere.input_planet_msg,
    );
    sim.connect(&spacecraft.state_out, &mut eclipse.input_state_msg);
    sim.connect(&sun_ephemeris.output_sun_msg, &mut eclipse.input_sun_msg);
    sim.connect(
        &atmosphere.output_atmosphere_msg,
        &mut drag.input_atmosphere_msg,
    );
    sim.connect(&sun_ephemeris.output_sun_msg, &mut srp.input_sun_msg);
    sim.connect(&eclipse.output_eclipse_msg, &mut srp.input_eclipse_msg);
    spacecraft.add_dynamic_effector(drag);
    spacecraft.add_dynamic_effector(srp);

    sim.connect(&spacecraft.state_out, &mut magnetic_field.input_state_msg);
    sim.connect(
        &earth_ephemeris.output_planet_msg,
        &mut magnetic_field.input_planet_msg,
    );
    sim.connect(&spacecraft.state_out, &mut imu_1.input_state_msg);
    sim.connect(&spacecraft.state_out, &mut imu_2.input_state_msg);
    sim.connect(&spacecraft.state_out, &mut tam_1.input_state_msg);
    sim.connect(&spacecraft.state_out, &mut tam_2.input_state_msg);
    sim.connect(&spacecraft.state_out, &mut gps.input_state_msg);
    sim.connect(&spacecraft.state_out, &mut star_tracker_1.input_state_msg);
    sim.connect(&spacecraft.state_out, &mut star_tracker_2.input_state_msg);
    for sun_sensor in [
        &mut sun_sensor_px,
        &mut sun_sensor_mx,
        &mut sun_sensor_py,
        &mut sun_sensor_my,
        &mut sun_sensor_pz,
        &mut sun_sensor_mz,
    ] {
        sim.connect(&spacecraft.state_out, &mut sun_sensor.input_state_msg);
        sim.connect(&sun_ephemeris.output_sun_msg, &mut sun_sensor.input_sun_msg);
        sim.connect(
            &eclipse.output_eclipse_msg,
            &mut sun_sensor.input_eclipse_msg,
        );
    }
    sim.connect(
        &magnetic_field.output_magnetic_field_msg,
        &mut tam_1.input_magnetic_field_msg,
    );
    sim.connect(
        &magnetic_field.output_magnetic_field_msg,
        &mut tam_2.input_magnetic_field_msg,
    );

    if enable_recording {
        sim.connect(&spacecraft.state_out, &mut spacecraft_recorder.input_msg);
        sim.connect(
            &sun_ephemeris.output_sun_msg,
            &mut sun_ephemeris_recorder.input_msg,
        );
        sim.connect(
            &moon_ephemeris.output_planet_msg,
            &mut moon_ephemeris_recorder.input_msg,
        );
        sim.connect(
            &atmosphere.output_atmosphere_msg,
            &mut atmosphere_recorder.input_msg,
        );
        sim.connect(&eclipse.output_eclipse_msg, &mut eclipse_recorder.input_msg);
        sim.connect(
            &magnetic_field.output_magnetic_field_msg,
            &mut magnetic_field_recorder.input_msg,
        );
        sim.connect(&imu_1.output_imu_msg, &mut imu_1_recorder.input_msg);
        sim.connect(&imu_2.output_imu_msg, &mut imu_2_recorder.input_msg);
        sim.connect(&tam_1.output_tam_msg, &mut tam_1_recorder.input_msg);
        sim.connect(&tam_2.output_tam_msg, &mut tam_2_recorder.input_msg);
        sim.connect(&gps.output_gps_msg, &mut gps_recorder.input_msg);
        sim.connect(
            &star_tracker_1.output_star_tracker_msg,
            &mut st_1_recorder.input_msg,
        );
        sim.connect(
            &star_tracker_2.output_star_tracker_msg,
            &mut st_2_recorder.input_msg,
        );
        sim.connect(
            &sun_sensor_px.output_sun_sensor_msg,
            &mut sun_sensor_px_recorder.input_msg,
        );
        sim.connect(
            &sun_sensor_mx.output_sun_sensor_msg,
            &mut sun_sensor_mx_recorder.input_msg,
        );
        sim.connect(
            &sun_sensor_py.output_sun_sensor_msg,
            &mut sun_sensor_py_recorder.input_msg,
        );
        sim.connect(
            &sun_sensor_my.output_sun_sensor_msg,
            &mut sun_sensor_my_recorder.input_msg,
        );
        sim.connect(
            &sun_sensor_pz.output_sun_sensor_msg,
            &mut sun_sensor_pz_recorder.input_msg,
        );
        sim.connect(
            &sun_sensor_mz.output_sun_sensor_msg,
            &mut sun_sensor_mz_recorder.input_msg,
        );
    }

    let _ = PRIORITY_ACTUATORS;
    sim.add_module(
        "sun_ephemeris",
        &mut sun_ephemeris,
        5_000_000,
        PRIORITY_ENVIRONMENT,
    );
    sim.add_module(
        "earth_ephemeris",
        &mut earth_ephemeris,
        5_000_000,
        PRIORITY_ENVIRONMENT,
    );
    sim.add_module(
        "moon_ephemeris",
        &mut moon_ephemeris,
        5_000_000,
        PRIORITY_ENVIRONMENT,
    );
    sim.add_module(
        "atmosphere",
        &mut atmosphere,
        5_000_000,
        PRIORITY_ENVIRONMENT,
    );
    sim.add_module("eclipse", &mut eclipse, 5_000_000, PRIORITY_ENVIRONMENT);
    sim.add_module(
        "spacecraft",
        &mut spacecraft,
        5_000_000,
        PRIORITY_ENVIRONMENT,
    );
    sim.add_module(
        "magnetic_field",
        &mut magnetic_field,
        5_000_000,
        PRIORITY_ENVIRONMENT,
    );
    sim.add_module("imu_1", &mut imu_1, 5_000_000, PRIORITY_SENSORS);
    sim.add_module("imu_2", &mut imu_2, 5_000_000, PRIORITY_SENSORS);
    sim.add_module("tam_1", &mut tam_1, 5_000_000, PRIORITY_SENSORS);
    sim.add_module("tam_2", &mut tam_2, 5_000_000, PRIORITY_SENSORS);
    sim.add_module(
        "sun_sensor_px",
        &mut sun_sensor_px,
        5_000_000,
        PRIORITY_SENSORS,
    );
    sim.add_module(
        "sun_sensor_mx",
        &mut sun_sensor_mx,
        5_000_000,
        PRIORITY_SENSORS,
    );
    sim.add_module(
        "sun_sensor_py",
        &mut sun_sensor_py,
        5_000_000,
        PRIORITY_SENSORS,
    );
    sim.add_module(
        "sun_sensor_my",
        &mut sun_sensor_my,
        5_000_000,
        PRIORITY_SENSORS,
    );
    sim.add_module(
        "sun_sensor_pz",
        &mut sun_sensor_pz,
        5_000_000,
        PRIORITY_SENSORS,
    );
    sim.add_module(
        "sun_sensor_mz",
        &mut sun_sensor_mz,
        5_000_000,
        PRIORITY_SENSORS,
    );
    sim.add_module("gps", &mut gps, 5_000_000, PRIORITY_SENSORS);
    sim.add_module("st_1", &mut star_tracker_1, 5_000_000, PRIORITY_SENSORS);
    sim.add_module("st_2", &mut star_tracker_2, 5_000_000, PRIORITY_SENSORS);

    if enable_recording {
        sim.add_module(
            "spacecraft_recorder",
            &mut spacecraft_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_ephemeris_recorder",
            &mut sun_ephemeris_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "moon_ephemeris_recorder",
            &mut moon_ephemeris_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "atmosphere_recorder",
            &mut atmosphere_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "eclipse_recorder",
            &mut eclipse_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "magnetic_field_recorder",
            &mut magnetic_field_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "imu_1_recorder",
            &mut imu_1_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "imu_2_recorder",
            &mut imu_2_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "tam_1_recorder",
            &mut tam_1_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "tam_2_recorder",
            &mut tam_2_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "gps_recorder",
            &mut gps_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "st_1_recorder",
            &mut st_1_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "st_2_recorder",
            &mut st_2_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_sensor_px_recorder",
            &mut sun_sensor_px_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_sensor_mx_recorder",
            &mut sun_sensor_mx_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_sensor_py_recorder",
            &mut sun_sensor_py_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_sensor_my_recorder",
            &mut sun_sensor_my_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_sensor_pz_recorder",
            &mut sun_sensor_pz_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
        sim.add_module(
            "sun_sensor_mz_recorder",
            &mut sun_sensor_mz_recorder,
            5_000_000,
            PRIORITY_RECORDERS,
        );
    }
    sim.run_for(simulation_duration_nanos);

    println!(
        "simulation_time_s = {:.1}",
        simulation_duration_nanos as f64 * 1.0e-9
    );
    if profile_sim {
        println!("module_timings_ms =");
        for timing in sim.module_timings().into_iter().take(12) {
            println!(
                "  {:>24}  priority={:>2}  calls={:>6}  total_ms={:>10.3}",
                timing.name,
                timing.priority,
                timing.num_updates,
                timing.total_update_nanos as f64 * 1.0e-6,
            );
        }
        let grav = spacecraft.gravity.timing_stats();
        println!("gravity_breakdown_ms =");
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "update_cache_total",
            grav.update_cache_calls,
            grav.update_cache_total_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "cache_state_read",
            grav.update_cache_state_read_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "cache_orientation_now",
            grav.update_cache_orientation_current_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "cache_orientation_prev",
            grav.update_cache_orientation_previous_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "compute_field_total",
            grav.compute_gravity_field_calls,
            grav.compute_gravity_field_total_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "position_step",
            grav.compute_gravity_position_step_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "accel_eval",
            grav.compute_gravity_accel_eval_nanos as f64 * 1.0e-6,
        );

        let atm = atmosphere.timing_stats();
        println!("atmosphere_breakdown_ms =");
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "update_total",
            atm.update_calls,
            atm.total_update_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "read_state",
            atm.read_state_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "read_planet",
            atm.read_planet_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "anise_rotation",
            atm.anise_rotation_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "geodetic",
            atm.geodetic_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "msis_eval",
            atm.msis_eval_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "output_write",
            atm.output_write_nanos as f64 * 1.0e-6,
        );

        let mag = magnetic_field.timing_stats();
        println!("magnetic_field_breakdown_ms =");
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "update_total",
            mag.update_calls,
            mag.total_update_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "read_input",
            mag.read_input_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "anise_rotation",
            mag.anise_rotation_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "geodetic",
            mag.geodetic_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "igrf_eval",
            mag.igrf_eval_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "frame_transform",
            mag.frame_transform_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "output_write",
            mag.output_write_nanos as f64 * 1.0e-6,
        );

        let sun = sun_ephemeris.timing_stats();
        println!("sun_ephemeris_breakdown_ms =");
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "update_total",
            sun.update_calls,
            sun.total_update_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "translate",
            sun.translate_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "write_sun_msg",
            sun.write_sun_msg_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "write_planet_msg",
            sun.write_planet_msg_nanos as f64 * 1.0e-6,
        );

        let earth = earth_ephemeris.timing_stats();
        println!("earth_ephemeris_breakdown_ms =");
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "update_total",
            earth.update_calls,
            earth.total_update_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "translate",
            earth.translate_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "write_output",
            earth.write_output_nanos as f64 * 1.0e-6,
        );

        let moon = moon_ephemeris.timing_stats();
        println!("moon_ephemeris_breakdown_ms =");
        println!(
            "  {:>24}  calls={:>6}  total_ms={:>10.3}",
            "update_total",
            moon.update_calls,
            moon.total_update_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "translate",
            moon.translate_nanos as f64 * 1.0e-6,
        );
        println!(
            "  {:>24}  total_ms={:>10.3}",
            "write_output",
            moon.write_output_nanos as f64 * 1.0e-6,
        );
    }
    println!("recording_enabled = {}", enable_recording);
    println!("show_progress = {}", show_progress);
    println!("profile_sim = {}", profile_sim);
    if enable_recording {
        println!("output_dir = {}", output_dir.display());
    }
    println!(
        "effectors = {}",
        spacecraft.state_effectors.len() + spacecraft.dynamic_effectors.len()
    );
}

fn single_sun_sensor(
    name: &str,
    position_m: Vector3<f64>,
    boresight_body: Vector3<f64>,
) -> SunSensor {
    SunSensor::new(SunSensorConfig {
        name: name.to_string(),
        position_m,
        body_to_sensor_quaternion: body_to_sensor_for_boresight(boresight_body),
        fov_half_angle_rad: 60.0_f64.to_radians(),
        scale_factor: 1.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.002,
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
