//! Imaging three ground strips in sequence: port of upstream
//! `examples/scenarioStripImaging.py`.
//!
//! A 330 kg spacecraft (inertia `100 I` kg m^2) with three Honeywell HR16
//! reaction wheels (50 N m s, spinning at 500 rpm, 0.5 N m motors) flies a
//! 520 km equatorial orbit about the point-mass, non-rotating Earth (no
//! planet ephemeris, as upstream). A `stripLocation` target moves at 3 km/s
//! along the great circle of each strip, starting 120 s of pre-imaging
//! before the strip's start; `locationPointing` turns the body +z axis to the
//! target and the +y axis across the scanning direction, and
//! `attTrackingError`, `mrpSteering` (`K1 = 10`, `K3 = 3`, 1.8 deg/s
//! maximum rate), `rateServoFullNonlinear` (`Ki = 5`, `P = 500`) and
//! `rwMotorTorque` follow it with the wheels. Three strips of about 500 km,
//! 167 s each after their pre-imaging: parallel to the ground track
//! (5 deg S, 110 to 120 deg E), perpendicular to it (2 to 12 deg N at
//! 137 deg E) and diagonal (1.5 deg S, 152 deg E to 1.5 deg N, 142 deg E).
//! One 0.1 s task, 861 s, histories sampled every 3 s.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case.
//!
//! Mapping to basilisk-rs:
//!
//! * between strips upstream re-specifies the strip ends and clears
//!   `isStartPositionUpdated` and `durationStripImaging`; here those are
//!   [`Staged`] changes of the target's configuration followed by
//!   `begin_new_strip`, applied after 287 s and 574 s;
//! * upstream steps the run 5 s at a time only to update Vizard's strip
//!   tiles and scan line (`addQuadMap`, `changeLocation`); that Vizard
//!   drawing is not ported, so the run is continuous;
//! * as in the reaction-wheel scenarios, `arrayMotorTorqueToSingleActuators`
//!   carries `rwMotorTorque`'s array command to the per-wheel motor inputs;
//!   the flight software's wheel configuration keeps upstream's 0.2 N m
//!   motor limit (`fswSetupRW.create(gsHat_B, Js, 0.2)`).
//!
//! Writes `examples/output/scenarios/scenarioStripImaging.csv` every 3 s
//! with `sigma_BR_<i>` (`attErrLog`), `sigma_RN_<i>` (`attrefLog`), the
//! target's `elevation_rad` (`accessLog`) and `v_LP_N_m_per_s_<i>`
//! (`targetLog`).

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
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::{add_point_mass_earth, att_tracking_error, scenario_recorder, vehicle_config};
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::environment::ground_location::SiteLocation;
use basilisk_rs::environment::strip_location::{StripLocation, StripLocationConfig};
use basilisk_rs::fsw_algorithms::att_control::mrp_steering::{MrpSteering, MrpSteeringConfig};
use basilisk_rs::fsw_algorithms::att_control::rate_servo_full_nonlinear::{
    RateServoFullNonlinear, RateServoFullNonlinearConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    AccessMsg, AttitudeGuidanceMsg, AttitudeReferenceMsg, Output, StripStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::geodetic_conversion::Lla;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use rw_factory::RwFactory;
use staged::Staged;

/// Upstream `simulationTimeStep = macros.sec2nano(0.1)`.
const TASK_PERIOD_NANOS: u64 = 100_000_000;
/// Upstream `samplingTime = macros.sec2nano(3)`.
const SAMPLE_PERIOD_NANOS: u64 = 3_000_000_000;
/// Upstream `pre_imaging_s` \[s\].
const PRE_IMAGING_NANOS: u64 = 120_000_000_000;
/// Upstream `imaging_s` \[s\]: a 500 km strip at 3 km/s.
const IMAGING_NANOS: u64 = 167_000_000_000;
/// One strip's phase: pre-imaging then imaging.
const PHASE_NANOS: u64 = PRE_IMAGING_NANOS + IMAGING_NANOS;
/// Upstream `phase3_end`: three phases.
const DURATION_NANOS: u64 = 3 * PHASE_NANOS;
/// Upstream `acquisitionSpeed` \[m/ns\].
const ACQUISITION_SPEED_M_PER_NS: f64 = 3e-6;
/// Upstream `minimumElevation` \[rad\]: the view-angle constraint.
const MINIMUM_ELEVATION_RAD: f64 = 10.0 * D2R;
/// Upstream `maxPrincipalRotationError` \[deg\]: the attitude constraint.
const MAX_PRINCIPAL_ROTATION_ERROR_DEG: f64 = 5.0;
/// One of upstream's `strip_configs`.
struct Strip {
    label: &'static str,
    /// Start (latitude, longitude) \[deg\].
    start: (f64, f64),
    /// End (latitude, longitude) \[deg\].
    end: (f64, f64),
}

/// Upstream `strip_configs`.
const STRIPS: [Strip; 3] = [
    Strip {
        label: "Parallel",
        start: (-5.0, 110.0),
        end: (-5.0, 120.0),
    },
    Strip {
        label: "Perpendicular",
        start: (2.0, 137.0),
        end: (12.0, 137.0),
    },
    Strip {
        label: "Diagonal",
        start: (-1.5, 152.0),
        end: (1.5, 142.0),
    },
];
const WHEEL_COUNT: usize = 3;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every 3 s.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `attErrLog`.
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// Upstream `attrefLog`.
    reference: MessageLog<AttitudeReferenceMsg>,
    /// Upstream `accessLog`.
    access: MessageLog<AccessMsg>,
    /// Upstream `targetLog`.
    target: MessageLog<StripStateMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream `specifyLocationStart/End(np.radians(lat), np.radians(lon), 0)`.
fn site((latitude_deg, longitude_deg): (f64, f64)) -> SiteLocation {
    SiteLocation::Geodetic(Lla {
        latitude_rad: latitude_deg * D2R,
        longitude_rad: longitude_deg * D2R,
        altitude_m: 0.0,
    })
}

/// Upstream `configure_strip_target(cfg)` for strip `index`.
fn configure_strip(target: &mut StripLocation, index: usize) {
    let strip = &STRIPS[index];
    target.config.start = site(strip.start);
    target.config.end = site(strip.end);
    target.begin_new_strip();
}

fn run(config: Config) -> Results {
    let inertia = Matrix3::from_diagonal_element(100.0);
    let model = ReactionWheelModel::BalancedWheels;
    let mut rw_factory = RwFactory::default();
    for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
        rw_factory
            .create_honeywell_hr16(axis, 50.0, 500.0, Vector3::zeros(), model)
            .max_torque_nm = 0.5;
    }
    // Upstream `fswSetupRW.create(gsHat_B, Js, 0.2)`.
    let mut fsw_rw_config = rw_factory.config_message();
    for torque in fsw_rw_config
        .max_motor_torques_nm
        .iter_mut()
        .take(WHEEL_COUNT)
    {
        *torque = 0.2;
    }
    let rw_config_msg = Output::new(fsw_rw_config);
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let mut reaction_wheels = ReactionWheelStateEffector::new("spacecraftBody");
    for wheel in rw_factory.wheels {
        reaction_wheels.add_reaction_wheel(wheel);
    }
    let wheel_speed_msg = reaction_wheels.rw_speed_out_msg.clone();

    let mu = MU_EARTH * 1e9;
    let elements = ClassicElements {
        semi_major_axis: (6378.0 + 520.0) * 1000.0,
        eccentricity: 0.0,
        inclination_rad: 0.0,
        raan_rad: 110.0 * D2R,
        arg_periapsis_rad: 347.8 * D2R,
        true_anomaly_rad: 0.0,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 330.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.0, -0.5, 0.0),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    add_point_mass_earth(&mut spacecraft);
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let mut strip_target = StripLocation::new(StripLocationConfig {
        name: "Strip_Imaging_Target".to_string(),
        minimum_elevation_rad: MINIMUM_ELEVATION_RAD,
        maximum_range_m: Some(1e9),
        planet_radius_m: REQ_EARTH * 1e3,
        acquisition_speed_m_per_ns: ACQUISITION_SPEED_M_PER_NS,
        pre_imaging_time_nanos: PRE_IMAGING_NANOS,
        ..StripLocationConfig::default()
    });
    configure_strip(&mut strip_target, 0);
    let target_index = strip_target.add_spacecraft_to_model();
    let mut strip_target = Staged::new(strip_target)
        .after(PHASE_NANOS, |target, _| configure_strip(target, 1))
        .after(2 * PHASE_NANOS, |target, _| configure_strip(target, 2));

    let mut pointing = LocationPointing::new(LocationPointingConfig {
        name: "locPoint".to_string(),
        p_hat_b: Vector3::z(),
        c_hat_b: Vector3::y(),
        use_boresight_rate_damping: false,
        ..LocationPointingConfig::default()
    });
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut steering = MrpSteering::new(MrpSteeringConfig {
        name: "MRP_Steering".to_string(),
        k1: 10.0,
        k3: 3.0,
        omega_max_radps: 1.8 * D2R,
        ignore_outer_loop_feedforward: false,
    });
    let servo_ki = 5.0;
    let mut servo = RateServoFullNonlinear::new(RateServoFullNonlinearConfig {
        name: "rate_servo".to_string(),
        p: 500.0,
        ki: servo_ki,
        known_torque_body_nm: Vector3::zeros(),
        integral_limit: 2.0 / servo_ki * 0.5,
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: WHEEL_COUNT,
        });

    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut reference_log = MessageLog::<AttitudeReferenceMsg>::default();
    let mut access_log = MessageLog::<AccessMsg>::default();
    let mut target_log = MessageLog::<StripStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioStripImaging", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);
    let target = &mut strip_target.module;
    let access = target.access_out_msgs[target_index].clone();
    let strip_state = target.current_strip_state_out_msg.clone();
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &spacecraft.state_out => &mut target.sc_state_in_msgs[target_index],
        &navigation.attitude_out_msg => &mut pointing.sc_att_in_msg,
        &navigation.translation_out_msg => &mut pointing.sc_trans_in_msg,
        &strip_state => &mut pointing.locationstrip_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &pointing.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut steering.guid_in_msg,
        &tracking_error.att_guid_out_msg => &mut servo.guid_in_msg,
        &vehicle_config_msg => &mut servo.veh_config_in_msg,
        &rw_config_msg => &mut servo.rw_params_in_msg,
        &wheel_speed_msg => &mut servo.rw_speeds_in_msg,
        &steering.rate_cmd_out_msg => &mut servo.rate_steering_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &servo.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &pointing.att_ref_out_msg => &mut reference_log.input,
        &access => &mut access_log.input,
        &strip_state => &mut target_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
        &pointing.att_ref_out_msg => recorder.add_source::<AttitudeReferenceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_rn", "sigma_RN")),
        ),
        &access => recorder.add_source::<AccessMsg>(
            CsvSourceConfig::columns([("elevation_rad", "elevation_rad")]),
        ),
        &strip_state => recorder.add_source::<StripStateMsg>(
            CsvSourceConfig::columns(vector_columns("velocity_lp_n_mps", "v_LP_N_m_per_s")),
        ),
    );

    // Upstream's task order: the wheels (priority 2) and the spacecraft (1),
    // here one module, then at the default priority navigation, the strip
    // target, the flight software and the recorders.
    schedule! { simulation,
        "spacecraftBody" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "SimpleNavigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=90;
        "Strip_Imaging_Target" => &mut strip_target, period=TASK_PERIOD_NANOS, priority=80;
        "locPoint" => &mut pointing, period=TASK_PERIOD_NANOS, priority=70;
        "attErrorInertial3D" => &mut tracking_error, period=TASK_PERIOD_NANOS, priority=60;
        "MRP_Steering" => &mut steering, period=TASK_PERIOD_NANOS, priority=50;
        "rate_servo" => &mut servo, period=TASK_PERIOD_NANOS, priority=40;
        "rwMotorTorque" => &mut rw_motor_torque, period=TASK_PERIOD_NANOS, priority=30;
        "rwMotorTorqueSplitter" => &mut torque_splitter, period=TASK_PERIOD_NANOS, priority=25;
        "attErrLog" => &mut guidance_log, period=SAMPLE_PERIOD_NANOS, priority=0;
        "attrefLog" => &mut reference_log, period=SAMPLE_PERIOD_NANOS, priority=0;
        "accessLog" => &mut access_log, period=SAMPLE_PERIOD_NANOS, priority=0;
        "targetLog" => &mut target_log, period=SAMPLE_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, SAMPLE_PERIOD_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        reference: reference_log,
        access: access_log,
        target: target_log,
        csv_path,
    }
}

/// Upstream's principal rotation error `4 atan(|sigma_BR|)` \[deg\].
fn principal_rotation_error_deg(guidance: &AttitudeGuidanceMsg) -> f64 {
    4.0 * guidance.sigma_br.norm().atan() / D2R
}

fn main() {
    let mut cli = Cli::from_env("scenario_strip_imaging [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        println!(
            "wrote {}",
            results.csv_path.as_ref().expect("CSV written").display()
        );
        for (index, Strip { label, .. }) in STRIPS.iter().enumerate() {
            let start = index as u64 * PHASE_NANOS + PRE_IMAGING_NANOS;
            let end = (index as u64 + 1) * PHASE_NANOS;
            let in_window = |time: &u64| (start..=end).contains(time);
            let worst_error = results
                .guidance
                .times_nanos
                .iter()
                .zip(&results.guidance.values)
                .filter(|(time, _)| in_window(time))
                .map(|(_, guidance)| principal_rotation_error_deg(guidance))
                .fold(0.0, f64::max);
            let lowest_elevation = results
                .access
                .times_nanos
                .iter()
                .zip(&results.access.values)
                .filter(|(time, _)| in_window(time))
                .map(|(_, access)| access.elevation_rad / D2R)
                .fold(f64::INFINITY, f64::min);
            println!(
                "{label} strip: pointing error up to {worst_error:.3} deg (limit \
                 {MAX_PRINCIPAL_ROTATION_ERROR_DEG}), elevation down to {lowest_elevation:.1} deg \
                 (limit {})",
                MINIMUM_ELEVATION_RAD / D2R
            );
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it runs. These tests check what it demonstrates: during
    //! each strip's imaging window the target moves along the strip at the
    //! acquisition speed and is in view, and the attitude stays within the
    //! 5 deg imaging constraint.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    /// The samples of `log` in strip `index`'s imaging window.
    fn imaging<T>(log: &MessageLog<T>, index: usize) -> impl Iterator<Item = &T> {
        let start = index as u64 * PHASE_NANOS + PRE_IMAGING_NANOS;
        let end = (index as u64 + 1) * PHASE_NANOS;
        log.times_nanos
            .iter()
            .zip(&log.values)
            .filter(move |(time, _)| (start..=end).contains(*time))
            .map(|(_, value)| value)
    }

    #[test]
    fn the_target_moves_along_each_strip_at_the_acquisition_speed() {
        let results = quiet();
        assert_eq!(
            results.target.values.len() as u64,
            DURATION_NANOS / SAMPLE_PERIOD_NANOS + 1
        );
        for index in 0..STRIPS.len() {
            for state in imaging(&results.target, index) {
                let speed = state.velocity_lp_n_mps.norm();
                assert!((speed - 3000.0).abs() < 1e-6, "strip {index}: {speed} m/s");
                assert!((state.position_lp_n_m.norm() - REQ_EARTH * 1e3).abs() < 1e-3);
            }
        }
    }

    #[test]
    fn each_strip_is_imaged_within_the_constraints() {
        let results = quiet();
        for index in 0..STRIPS.len() {
            let worst_error = imaging(&results.guidance, index)
                .map(principal_rotation_error_deg)
                .fold(0.0, f64::max);
            assert!(
                worst_error < MAX_PRINCIPAL_ROTATION_ERROR_DEG,
                "strip {index}: {worst_error} deg"
            );
            let lowest = imaging(&results.access, index)
                .map(|access| access.elevation_rad)
                .fold(f64::INFINITY, f64::min);
            assert!(
                lowest > MINIMUM_ELEVATION_RAD,
                "strip {index}: {lowest} rad"
            );
            assert!(imaging(&results.access, index).all(|access| access.has_access));
        }
    }

    #[test]
    fn the_scan_line_lies_across_the_strip() {
        let results = quiet();
        // In the reference frame the body +y axis (`cHat_B`) is
        // perpendicular to the target's ground velocity, unless that
        // velocity is nearly along the boresight.
        for index in 0..STRIPS.len() {
            for (reference, target) in
                imaging(&results.reference, index).zip(imaging(&results.target, index))
            {
                let v_r = mrp_to_dcm(reference.sigma_rn) * target.velocity_lp_n_mps.normalize();
                let v_perp = Vector3::z().cross(&v_r);
                if v_perp.norm() > 0.1 {
                    assert!(v_r.y.abs() < 1e-6, "strip {index}: {}", v_r.y);
                }
            }
        }
    }
}
