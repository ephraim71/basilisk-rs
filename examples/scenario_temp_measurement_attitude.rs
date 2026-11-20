//! Reaction-wheel motor temperatures and their noisy measurement during an
//! attitude maneuver: port of upstream
//! `examples/scenarioTempMeasurementAttitude.py`.
//!
//! The reaction-wheel attitude scenario of `scenarioAttitudeFeedbackRW`
//! (750 kg spacecraft on the 10 000 km orbit, three Honeywell HR16 wheels at
//! 100, 200 and 300 RPM, `mrpFeedback` with `K = 3.5`, `P = 30` and
//! `rwMotorTorque`), with a `motorThermal` model on each wheel (20 °C start
//! and ambient, efficiency 0.7, 5 °C/W to ambient, 50 J/°C) and a
//! `tempMeasurement` sensor on each motor (no bias, 0.5 °C white noise, no
//! faults). One 0.1 s task for 10 minutes, histories sampled every 6 s.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and
//! runs the same single case.
//!
//! Mapping to basilisk-rs:
//!
//! * the wheels take one command each: `arrayMotorTorqueToSingleActuators`
//!   splits `rwMotorTorque`'s array, right after it;
//! * upstream adds the wheel effector and the thermal and sensor models at
//!   priority 2, ahead of the spacecraft: they run first here too, so each
//!   thermal model reads its wheel's state of the previous step, as
//!   upstream's does;
//! * upstream seeds each sensor with the wall clock (`time.time_ns()`); here
//!   the seed comes from the sensor's name, so runs repeat.
//!
//! Writes `examples/output/scenarios/scenarioTempMeasurementAttitude.csv`
//! with `rwTemperature_C_*` and `tempMeasurement_C_*`.

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

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HubState, add_point_mass_earth, att_tracking_error,
    hub_inertia_kg_m2, inertial_3d, mrp_feedback, sampling_period_nanos, scenario_recorder,
    spacecraft, vehicle_config,
};
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::device_interface::temp_measurement::{
    TempFaultState, TempMeasurement, TempMeasurementConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{AttitudeGuidanceMsg, Output, ReactionWheelStateMsg, TemperatureMsg};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::thermal::motor_thermal::{MotorThermal, MotorThermalConfig};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use rw_factory::RwFactory;

const WHEEL_COUNT: usize = 3;
/// Upstream `currentTemperature` and `ambientTemperature` \[°C\].
const INITIAL_TEMPERATURE_C: f64 = 20.0;
/// Upstream `senNoiseStd` \[°C\].
const SENSOR_NOISE_STD_C: f64 = 0.5;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every 6 s.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    temperatures: Vec<MessageLog<TemperatureMsg>>,
    measurements: Vec<MessageLog<TemperatureMsg>>,
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let inertia = hub_inertia_kg_m2();
    let model = ReactionWheelModel::BalancedWheels;
    let mut rw_factory = RwFactory::default();
    rw_factory.create_honeywell_hr16(Vector3::x(), 50.0, 100.0, Vector3::zeros(), model);
    rw_factory.create_honeywell_hr16(Vector3::y(), 50.0, 200.0, Vector3::zeros(), model);
    rw_factory.create_honeywell_hr16(
        Vector3::z(),
        50.0,
        300.0,
        Vector3::new(0.5, 0.5, 0.5),
        model,
    );
    let rw_config_msg = Output::new(rw_factory.config_message());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

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

    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.01));
    add_point_mass_earth(&mut spacecraft);
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = inertial_3d();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: WHEEL_COUNT,
        });
    let mut thermals: Vec<MotorThermal> = (0..WHEEL_COUNT)
        .map(|index| {
            MotorThermal::new(MotorThermalConfig {
                name: format!("rwThermals{index}"),
                initial_temperature_c: INITIAL_TEMPERATURE_C,
                ambient_temperature_c: INITIAL_TEMPERATURE_C,
                ambient_thermal_resistance_c_per_w: 5.0,
                motor_heat_capacity_j_per_c: 50.0,
                efficiency: 0.7,
            })
        })
        .collect();
    let mut sensors: Vec<TempMeasurement> = (0..WHEEL_COUNT)
        .map(|index| {
            TempMeasurement::new(TempMeasurementConfig {
                name: format!("tempMeasurementModel{index}"),
                fault_state: TempFaultState::Nominal,
                sen_bias_c: 0.0,
                sen_noise_std_c: SENSOR_NOISE_STD_C,
                stuck_value_c: 0.0,
                spike_probability: 0.0,
                spike_amount: 0.0,
                ..TempMeasurementConfig::default()
            })
        })
        .collect();

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut temperature_logs: Vec<MessageLog<TemperatureMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let mut measurement_logs: Vec<MessageLog<TemperatureMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let mut wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioTempMeasurementAttitude", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
    );
    for index in 0..WHEEL_COUNT {
        let thermal = &mut thermals[index];
        let sensor = &mut sensors[index];
        connect!(&simulation,
            &wheel_state_msgs[index] => &mut thermal.rw_state_in_msg,
            &thermal.temperature_out_msg => &mut sensor.temp_in_msg,
            &wheel_state_msgs[index] => &mut wheel_logs[index].input,
            &thermal.temperature_out_msg => &mut temperature_logs[index].input,
            &sensor.temp_out_msg => &mut measurement_logs[index].input,
        );
    }
    for (index, thermal) in thermals.iter().enumerate() {
        connect!(&simulation,
            &thermal.temperature_out_msg => recorder.add_source::<TemperatureMsg>(
                CsvSourceConfig::columns([(
                    "temperature_c".to_string(),
                    format!("rwTemperature_C_{index}"),
                )]),
            ),
        );
    }
    for (index, sensor) in sensors.iter().enumerate() {
        connect!(&simulation,
            &sensor.temp_out_msg => recorder.add_source::<TemperatureMsg>(
                CsvSourceConfig::columns([(
                    "temperature_c".to_string(),
                    format!("tempMeasurement_C_{index}"),
                )]),
            ),
        );
    }

    // Upstream's priorities: the thermal and sensor models (2) ahead of the
    // spacecraft (1), then the flight software in the order added.
    for (thermal, sensor) in thermals.iter_mut().zip(&mut sensors) {
        simulation.add_module("rw_thermal", thermal, DYNAMICS_PERIOD_NANOS, 110);
        simulation.add_module("temp_measurement", sensor, DYNAMICS_PERIOD_NANOS, 110);
    }
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial_reference" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrp_feedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "rw_motor_torque" => &mut rw_motor_torque, period=DYNAMICS_PERIOD_NANOS, priority=50;
        "rw_motor_torque_splitter" => &mut torque_splitter, period=DYNAMICS_PERIOD_NANOS, priority=40;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
    }
    for log in temperature_logs.iter_mut().chain(&mut measurement_logs) {
        simulation.add_module("temperature_log", log, sample_period, 0);
    }
    for log in &mut wheel_logs {
        simulation.add_module("wheel_log", log, sample_period, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        temperatures: temperature_logs,
        measurements: measurement_logs,
        wheels: wheel_logs,
        guidance: guidance_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_temp_measurement_attitude [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let peaks: Vec<String> = results
            .temperatures
            .iter()
            .map(|log| {
                let peak = log
                    .values
                    .iter()
                    .fold(f64::MIN, |peak, sample| peak.max(sample.temperature_c));
                format!("{peak:.2}")
            })
            .collect();
        println!(
            "wrote {} (peak motor temperatures {} °C)",
            results.csv_path.expect("CSV written").display(),
            peaks.join(", "),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream only runs this scenario (an entry of `test_scenarioTest.py`).
    //! These tests check what it demonstrates: the motors warm above ambient
    //! while the wheels work and never cool below it (the model only adds
    //! heat and relaxes towards ambient), the attitude settles, and each
    //! sensor reads its motor's temperature with the configured 0.5 °C
    //! noise.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_motors_warm_while_the_wheels_work() {
        let results = quiet();
        assert_eq!(results.guidance.values.len(), 101);
        assert!(results.guidance.last().sigma_br.norm() < 1.0e-3);
        for (log, wheels) in results.temperatures.iter().zip(&results.wheels) {
            assert_eq!(log.values[0].temperature_c, INITIAL_TEMPERATURE_C);
            for sample in &log.values {
                assert!(sample.temperature_c >= INITIAL_TEMPERATURE_C - 1.0e-12);
            }
            let peak = log
                .values
                .iter()
                .fold(f64::MIN, |peak, sample| peak.max(sample.temperature_c));
            // Each wheel does work: its speed changes.
            let speed_change = (wheels.last().omega_radps - wheels.values[0].omega_radps).abs();
            assert!(speed_change > 1.0, "{speed_change}");
            assert!(peak > INITIAL_TEMPERATURE_C + 0.01, "{peak}");
        }
    }

    #[test]
    fn the_sensors_read_the_motor_temperatures_with_their_noise() {
        let results = quiet();
        let errors: Vec<f64> = results
            .measurements
            .iter()
            .zip(&results.temperatures)
            .flat_map(|(measured, truth)| {
                measured
                    .values
                    .iter()
                    .zip(&truth.values)
                    .map(|(measured, truth)| measured.temperature_c - truth.temperature_c)
                    .collect::<Vec<_>>()
            })
            .collect();
        let mean = errors.iter().sum::<f64>() / errors.len() as f64;
        let sigma = (errors
            .iter()
            .map(|error| (error - mean).powi(2))
            .sum::<f64>()
            / errors.len() as f64)
            .sqrt();
        // 303 samples: the sample sigma is within 20% of 0.5 °C.
        assert!(mean.abs() < 0.15, "{mean}");
        assert!((sigma - SENSOR_NOISE_STD_C).abs() < 0.1, "{sigma}");
    }
}
