//! Reaction-wheel momentum dumping with thrusters: port of upstream
//! `examples/scenarioMomentumDumping.py`.
//!
//! A 2500 kg spacecraft (hub center of mass 1.28 m along body z, inertia
//! `diag(1700, 1700, 1800) kg m^2`) on a 7000 km Earth orbit, with the Sun
//! as a third body from `2022 JUNE 27 00:00:00.0`, holds the inertial
//! attitude with four Honeywell HR16 wheels (100 N m s, at 4000, 2000, 3500
//! and 0 RPM). `mrpFeedback` (`P = 3 max(I) / 10 s`, `K = P^2 / max(I)`)
//! feeds `rwMotorTorque`. After 10 s, `thrMomentumManagement` is reset and
//! asks for the momentum change that brings the wheels down to 80 N m s;
//! `thrForceMapping` maps it onto eight MOOG Monarc-5 thrusters (5 N) and
//! `thrMomentumDumping` fires them as on-time pulses, one every 100 control
//! periods. Dynamics on a 0.1 s task, flight software on a 1 s task, for 5
//! minutes; histories sampled every dynamics step.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and
//! runs the same single case.
//!
//! Mapping to basilisk-rs:
//!
//! * the wheels and thrusters take one command each:
//!   `arrayMotorTorqueToSingleActuators` splits `rwMotorTorque`'s array right
//!   after it, `thrMomentumDumping` publishes one on-time command per
//!   thruster, and each thruster is its own `ThrusterDynamicEffector`;
//! * upstream's `spiceInterface` (`zeroBase = 'Earth'`) is
//!   [`AniseSunEphemeris`], the Sun relative to Earth from DE440s, on the
//!   flight-software task as upstream, also evaluated at initialization
//!   ([`UpdatedAtInit`]) as upstream's `Reset` is;
//! * upstream's `thrDesatControl.Reset(10 s)` between the two
//!   `ExecuteSimulation` calls is a [`Staged`] reset.
//!
//! # The momentum request before the reset
//!
//! Upstream's `thrMomentumManagement` writes its request once after each
//! reset: at 0 s and, reset, at 11 s, with the same value. `thrMomentumDumping`
//! starts a firing cycle when the request's write time changes and ignores
//! one written at 0 s, so upstream dumps from 11 s. basilisk-rs messages
//! carry no write time, and `thrMomentumDumping` counts the request as
//! rewritten when its value changes, so it cannot see the 11 s rewrite of an
//! unchanged request. The momentum management is therefore held (not
//! updated) until its reset: its first request is the 11 s one, which starts
//! the dumping when upstream's does. The only difference is that the
//! request and the mapped thruster forces read zero, not the request, for
//! the first 11 s.
//!
//! Writes `examples/output/scenarios/scenarioMomentumDumping.csv`.

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
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use std::path::PathBuf;

use attitude::{att_tracking_error, inertial_3d, mrp_feedback, scenario_recorder, vehicle_config};
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::fsw_algorithms::att_control::thr_momentum_management::{
    ThrMomentumManagement, ThrMomentumManagementConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_force_mapping::{
    ThrForceMapping, ThrForceMappingConfig, ThrForceSign,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_momentum_dumping::{
    ThrMomentumDumping, ThrMomentumDumpingConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, Output,
    ReactionWheelStateMsg, RwSpeedMsg, SpacecraftStateMsg, ThrusterArrayForceCommandMsg,
    ThrusterCommandMsg, ThrusterOutputMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, MU_SUN};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use rw_factory::RwFactory;
use staged::{Staged, UpdatedAtInit};
use thruster_factory::{ThrusterFactory, ThrusterModel};

/// Upstream `simulationTimeStepDyn = macros.sec2nano(0.1)`.
const DYNAMICS_PERIOD_NANOS: u64 = 100_000_000;
/// Upstream `simulationTimeStepFsw = macros.sec2nano(1)`.
const FSW_PERIOD_NANOS: u64 = 1_000_000_000;
/// Upstream `simulationTime = macros.min2nano(5)`.
const DURATION_NANOS: u64 = 300_000_000_000;
/// Upstream's first stop, after which `thrDesatControl` is reset.
const RESET_TIME_NANOS: u64 = 10_000_000_000;
/// Upstream `thrDesatControl.hs_min` \[N m s\].
const HS_MIN_NMS: f64 = 80.0;
/// The DE440s planetary SPK (`just fetch-anise-assets`).
const PLANETARY_SPK: &str = "assets/anise/de440s.bsp";
const WHEEL_COUNT: usize = 4;
const THRUSTER_COUNT: usize = 8;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioMomentumDumping.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// Upstream `I = [1700, 1700, 1800]` (diagonal) \[kg m^2\].
fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(1700.0, 1700.0, 1800.0))
}

/// Upstream's wheels: `Honeywell_HR16`, 100 N m s, balanced.
fn wheels() -> RwFactory {
    let c = 0.5_f64.sqrt();
    let mut factory = RwFactory::default();
    for (axis, speed_rpm) in [
        (Vector3::new(c, 0.0, c), 4000.0),
        (Vector3::new(0.0, c, c), 2000.0),
        (Vector3::new(-c, 0.0, c), 3500.0),
        (Vector3::new(0.0, -c, c), 0.0),
    ] {
        factory.create_honeywell_hr16(
            axis,
            100.0,
            speed_rpm,
            Vector3::zeros(),
            ReactionWheelModel::BalancedWheels,
        );
    }
    factory
}

/// Upstream's eight `MOOG_Monarc_5` thrusters with `MaxThrust = 5.0`.
fn thrusters() -> ThrusterFactory {
    let (a, b) = (1.0, 1.28);
    let locations = [
        [-a, -a, b],
        [a, -a, -b],
        [a, -a, b],
        [a, a, -b],
        [a, a, b],
        [-a, a, -b],
        [-a, a, b],
        [-a, -a, -b],
    ];
    let directions = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
    ];
    let mut factory = ThrusterFactory::default();
    for (location, direction) in locations.into_iter().zip(directions) {
        factory
            .create(
                ThrusterModel::MoogMonarc5,
                Vector3::from(location),
                Vector3::from(direction),
                None,
            )
            .max_thrust_n = 5.0;
    }
    factory
}

/// The recorded histories, every dynamics step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    wheel_speeds: MessageLog<RwSpeedMsg>,
    delta_h: MessageLog<BodyTorqueCommandMsg>,
    on_times: Vec<MessageLog<ThrusterCommandMsg>>,
    thrusts: Vec<MessageLog<ThrusterOutputMsg>>,
    /// Upstream `RW.Js` and `RW.gsHat_B` of each wheel.
    wheel_axes: Vec<(f64, Vector3<f64>)>,
    csv_path: Option<PathBuf>,
}

/// Upstream `timeInitString = "2022 JUNE 27 00:00:00.0"`.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2022, 6, 27)
}

fn run(config: Config) -> Results {
    let mu = MU_EARTH * 1e9;
    let elements = ClassicElements {
        semi_major_axis: 7000.0 * 1000.0,
        eccentricity: 0.0001,
        inclination_rad: 33.3 * D2R,
        raan_rad: 148.2 * D2R,
        arg_periapsis_rad: 347.8 * D2R,
        true_anomaly_rad: 335.0 * D2R,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 2500.0,
        hub_center_of_mass_body_m: Vector3::new(0.0, 0.0, 1.28),
        inertia_kg_m2: inertia,
        integration_step_nanos: DYNAMICS_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "sun",
                MU_SUN * 1e9,
                false,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Sun"),
        )
        .expect("unique gravity body");
    let mut sun_ephemeris = UpdatedAtInit::new(
        AniseSunEphemeris::new(AniseSunEphemerisConfig {
            name: "spiceObject".to_string(),
            spk_path: PLANETARY_SPK.into(),
        }),
        epoch(),
    );

    let rw_factory = wheels();
    let rw_config_msg = Output::new(rw_factory.config_message());
    let wheel_axes = rw_factory
        .wheels
        .iter()
        .map(|wheel| (wheel.js_kg_m2, wheel.spin_axis_body))
        .collect();
    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for wheel in rw_factory.wheels {
        reaction_wheels.add_reaction_wheel(wheel);
    }
    let wheel_speed_msg = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_msgs: Vec<Output<ReactionWheelStateMsg>> = reaction_wheels
        .wheels()
        .iter()
        .map(|wheel| wheel.state_out.clone())
        .collect();

    let thruster_factory = thrusters();
    let thr_config_msg = Output::new(thruster_factory.config_message());
    let mut thruster_effectors: Vec<ThrusterDynamicEffector> = thruster_factory
        .thrusters
        .iter()
        .map(|thruster| ThrusterDynamicEffector::new(thruster.dynamic_effector_config()))
        .collect();
    let thruster_outputs: Vec<Output<ThrusterOutputMsg>> = thruster_effectors
        .iter()
        .map(|thruster| thruster.thruster_out_msg.clone())
        .collect();

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    // Upstream: `P = 3 max(I) / decayTime`, `K = (P / xi)^2 / max(I)`.
    let (decay_time_s, xi, max_inertia) = (10.0, 1.0, 1800.0);
    let p = 3.0 * max_inertia / decay_time_s;
    let k = (p / xi) * (p / xi) / max_inertia;
    let mut controller = mrp_feedback(k, -1.0, p, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: WHEEL_COUNT,
        });
    let mut desaturation = Staged::new(ThrMomentumManagement::new(ThrMomentumManagementConfig {
        name: "thrMomentumManagement".to_string(),
        hs_min_nms: HS_MIN_NMS,
    }))
    .held_until(RESET_TIME_NANOS)
    .reset_after(RESET_TIME_NANOS);
    let mut force_mapping = ThrForceMapping::new(ThrForceMappingConfig {
        name: "thrForceMapping".to_string(),
        control_axes_b: [Vector3::x(), Vector3::y(), Vector3::z()],
        thr_force_sign: ThrForceSign::Positive,
        ang_err_thresh_rad: 3.15,
        ..ThrForceMappingConfig::default()
    });
    let mut dumping = ThrMomentumDumping::new(ThrMomentumDumpingConfig {
        name: "thrDump".to_string(),
        max_counter_value: 100,
        thr_min_fire_time_s: 0.02,
    });

    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut wheel_speed_log = MessageLog::<RwSpeedMsg>::default();
    let mut delta_h_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut on_time_logs: Vec<MessageLog<ThrusterCommandMsg>> =
        (0..THRUSTER_COUNT).map(|_| MessageLog::default()).collect();
    let mut thrust_logs: Vec<MessageLog<ThrusterOutputMsg>> =
        (0..THRUSTER_COUNT).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder("scenarioMomentumDumping", config.write_csv);

    let mut simulation = Simulation::new(epoch(), false);
    connect!(&simulation,
        &sun_ephemeris.module.output_planet_msg => spacecraft
            .grav_body_input_mut("sun")
            .expect("the Sun is a gravity body"),
    );
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);
    for ((thruster, on_time_log), thrust_log) in thruster_effectors
        .iter_mut()
        .zip(&mut on_time_logs)
        .zip(&mut thrust_logs)
    {
        let command = dumping.add_thruster_command_output().clone();
        connect!(&simulation,
            &command => &mut thruster.command_in,
            &command => &mut on_time_log.input,
            &thruster.thruster_out_msg => &mut thrust_log.input,
        );
    }
    for thruster in thruster_effectors {
        spacecraft.add_dynamic_effector(thruster);
    }
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &wheel_speed_msg => &mut desaturation.module.rw_speeds_in_msg,
        &rw_config_msg => &mut desaturation.module.rw_config_data_in_msg,
        &desaturation.module.delta_h_out_msg => &mut force_mapping.cmd_torque_in_msg,
        &thr_config_msg => &mut force_mapping.thr_config_in_msg,
        &vehicle_config_msg => &mut force_mapping.veh_config_in_msg,
        &thr_config_msg => &mut dumping.thruster_conf_in_msg,
        &desaturation.module.delta_h_out_msg => &mut dumping.delta_h_in_msg,
        &force_mapping.thr_force_cmd_out_msg => &mut dumping.thruster_impulse_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &wheel_speed_msg => &mut wheel_speed_log.input,
        &desaturation.module.delta_h_out_msg => &mut delta_h_log.input,
        &navigation.attitude_out_msg => recorder.add_source::<NavigationAttitudeMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
        ),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
        &rw_motor_torque.rw_motor_torque_out_msg => recorder.add_source::<ArrayMotorTorqueMsg>(
            CsvSourceConfig::columns(array_columns("motor_torque_nm", "motorTorque_Nm", WHEEL_COUNT)),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &desaturation.module.delta_h_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns("torque_request_body_nm", "deltaH_Nms")),
        ),
        &force_mapping.thr_force_cmd_out_msg => recorder.add_source::<ThrusterArrayForceCommandMsg>(
            CsvSourceConfig::columns(array_columns("thrust_force_n", "thrForce_N", THRUSTER_COUNT)),
        ),
        &wheel_speed_msg => recorder.add_source::<RwSpeedMsg>(
            CsvSourceConfig::columns(array_columns(
                "wheel_speeds_radps",
                "wheelSpeeds_rad_per_s",
                WHEEL_COUNT,
            )),
        ),
    );
    for (index, state) in wheel_state_msgs.iter().enumerate() {
        connect!(&simulation,
            state => recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
        );
    }
    for (index, output) in dumping.thruster_on_time_out_msgs.iter().enumerate() {
        connect!(&simulation,
            output => recorder.add_source::<ThrusterCommandMsg>(CsvSourceConfig::columns([(
                "on_time_s".to_string(),
                format!("OnTimeRequest_s_{index}"),
            )])),
        );
    }
    for (index, output) in thruster_outputs.iter().enumerate() {
        connect!(&simulation,
            output => recorder.add_source::<ThrusterOutputMsg>(CsvSourceConfig::columns([(
                "thrust_force_n".to_string(),
                format!("thrustForce_N_{index}"),
            )])),
        );
    }
    schedule! { simulation,
        // Dynamics task; the wheels and thrusters live inside the spacecraft.
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        // Flight-software task, after the dynamics at shared ticks.
        "spice" => &mut sun_ephemeris, period=FSW_PERIOD_NANOS, priority=85;
        "inertial_reference" => &mut inertial_reference, period=FSW_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=FSW_PERIOD_NANOS, priority=75;
        "mrp_feedback" => &mut controller, period=FSW_PERIOD_NANOS, priority=70;
        "rw_motor_torque" => &mut rw_motor_torque, period=FSW_PERIOD_NANOS, priority=65;
        "rw_motor_torque_splitter" => &mut torque_splitter, period=FSW_PERIOD_NANOS, priority=64;
        "thr_momentum_management" => &mut desaturation, period=FSW_PERIOD_NANOS, priority=60;
        "thr_force_mapping" => &mut force_mapping, period=FSW_PERIOD_NANOS, priority=55;
        "thr_momentum_dumping" => &mut dumping, period=FSW_PERIOD_NANOS, priority=50;
        "guidance_log" => &mut guidance_log, period=DYNAMICS_PERIOD_NANOS, priority=0;
        "wheel_speed_log" => &mut wheel_speed_log, period=DYNAMICS_PERIOD_NANOS, priority=0;
        "delta_h_log" => &mut delta_h_log, period=DYNAMICS_PERIOD_NANOS, priority=0;
    }
    for log in &mut on_time_logs {
        simulation.add_module("on_time_log", log, DYNAMICS_PERIOD_NANOS, 0);
    }
    for log in &mut thrust_logs {
        simulation.add_module("thrust_log", log, DYNAMICS_PERIOD_NANOS, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, DYNAMICS_PERIOD_NANOS, 0);
    }
    simulation.run_for(RESET_TIME_NANOS);
    simulation.run_for(DURATION_NANOS - RESET_TIME_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        wheel_speeds: wheel_speed_log,
        delta_h: delta_h_log,
        on_times: on_time_logs,
        thrusts: thrust_logs,
        wheel_axes,
        csv_path,
    }
}

/// The wheels' net angular momentum \[N m s\], body components.
fn wheel_momentum(speeds: &RwSpeedMsg, axes: &[(f64, Vector3<f64>)]) -> Vector3<f64> {
    axes.iter()
        .zip(speeds.wheel_speeds_radps.iter())
        .map(|((js, axis), speed)| js * speed * axis)
        .sum()
}

fn main() {
    let mut cli = Cli::from_env("scenario_momentum_dumping [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let first = wheel_momentum(&results.wheel_speeds.values[0], &results.wheel_axes);
        let last = wheel_momentum(results.wheel_speeds.last(), &results.wheel_axes);
        println!(
            "wrote {} (wheel momentum {:.1} -> {:.1} N m s)",
            results.csv_path.expect("CSV written").display(),
            first.norm(),
            last.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioMomentumDumping.py` only checks that the
    //! scenario runs. These tests check what it demonstrates: once the
    //! momentum management is reset, it asks for the momentum change that
    //! brings the wheels' net momentum down to `hs_min = 80 N m s`, the
    //! thrusters deliver it in pulses, and the wheels end there while the
    //! attitude holds.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_thrusters_dump_the_wheel_momentum_to_hs_min() {
        let results = quiet();
        assert_eq!(results.guidance.values.len(), 3001);
        let first = wheel_momentum(&results.wheel_speeds.values[0], &results.wheel_axes);
        let last = wheel_momentum(results.wheel_speeds.last(), &results.wheel_axes);
        assert!(first.norm() > 110.0, "{}", first.norm());
        assert!(
            (last.norm() - HS_MIN_NMS).abs() < 0.05 * HS_MIN_NMS,
            "{}",
            last.norm()
        );
        // The momentum is removed, not turned: the direction is kept.
        assert!(last.angle(&first).to_degrees() < 5.0);

        // The request: `-(1 - hs_min / |H|) H` after the reset.
        let request = results
            .delta_h
            .at(11 * FSW_PERIOD_NANOS)
            .torque_request_body_nm;
        let expected = -(1.0 - HS_MIN_NMS / first.norm()) * first;
        assert!(
            (request - expected).norm() < 0.05 * expected.norm(),
            "{request} vs {expected}"
        );

        let fired = results
            .thrusts
            .iter()
            .filter(|log| log.values.iter().any(|sample| sample.thrust_force_n > 0.0))
            .count();
        assert!(fired >= 2, "{fired}");
        for log in &results.on_times {
            assert!(log.values.iter().all(|command| command.on_time_s >= 0.0));
        }
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-2);
    }
}
