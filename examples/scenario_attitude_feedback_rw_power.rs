//! Reaction-wheel attitude control with the wheels' electrical power drawn
//! from a battery: port of upstream
//! `examples/scenarioAttitudeFeedbackRWPower.py`.
//!
//! The balanced-wheel case of
//! [`scenarioAttitudeFeedbackRW`](scenario_attitude_feedback_rw.rs) (three
//! HR16 wheels, `mrpFeedback` and `rwMotorTorque` driving the wheels directly,
//! inertial pointing from `sigma_BN = [0.1, 0.2, -0.3]`), plus one
//! `ReactionWheelPower` node per wheel (5 W base load, ideal motor) feeding a
//! `simpleBattery` (300 kJ capacity, starting 80 % full). One 0.1 s task,
//! 10 minutes, histories sampled every 6 s.
//!
//! Upstream `run(show_plots, useRwPowerGeneration)` maps to
//! `--rw-power-generation`: braking a wheel returns half of its mechanical
//! power to the bus (`mechToElecEfficiency = 0.5`) instead of drawing power
//! like spinning it up. The upstream script's own default is on; `--all` runs
//! both cases, as upstream's `test_scenarioAttitudeFeedbackRWPower.py` does.
//!
//! As in the reaction-wheel scenario, `arrayMotorTorqueToSingleActuators`
//! carries `rwMotorTorque`'s array command to the per-wheel motor inputs.
//!
//! Writes
//! `examples/output/scenarios/scenarioAttitudeFeedbackRWPower<True|False>.csv`
//! (upstream's figure suffix `str(useRwPowerGeneration)`).

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
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, Output, PowerNodeUsageMsg, PowerStorageStatusMsg,
    ReactionWheelStateMsg, RwSpeedMsg,
};
use basilisk_rs::power::reaction_wheel_power::{ReactionWheelPower, ReactionWheelPowerModel};
use basilisk_rs::power::simple_battery::{SimpleBattery, SimpleBatteryConfig};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use rw_factory::RwFactory;

const WHEEL_COUNT: usize = 3;
/// Upstream `battery.storageCapacity` \[J\].
const STORAGE_CAPACITY_J: f64 = 300_000.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_rw_power_generation: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_rw_power_generation: bool) -> Self {
        Self {
            use_rw_power_generation,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        let suffix = if self.use_rw_power_generation {
            "True"
        } else {
            "False"
        };
        format!("scenarioAttitudeFeedbackRWPower{suffix}")
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeFeedbackRWPower.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    wheel_power: Vec<MessageLog<PowerNodeUsageMsg>>,
    battery: MessageLog<PowerStorageStatusMsg>,
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

    let mut reaction_wheels = ReactionWheelStateEffector::new("bsk-Sat");
    for wheel in rw_factory.wheels {
        reaction_wheels.add_reaction_wheel(wheel);
    }
    let wheel_speed_msg = reaction_wheels.rw_speed_out_msg.clone();
    let wheel_state_msgs: [Output<ReactionWheelStateMsg>; WHEEL_COUNT] =
        std::array::from_fn(|index| reaction_wheels.wheels()[index].state_out.clone());

    let mut spacecraft = spacecraft(inertia, &HubState::on_orbit(0.01));
    add_point_mass_earth(&mut spacecraft);
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let mut wheel_power: Vec<ReactionWheelPower> = (0..WHEEL_COUNT)
        .map(|index| {
            let mut model = ReactionWheelPowerModel::default();
            model.base_power_need_w = 5.0;
            if config.use_rw_power_generation {
                model.mech_to_elec_efficiency = 0.5;
            }
            ReactionWheelPower::new(format!("bsk-SatRWPower{index}"), model)
        })
        .collect();
    let mut battery = SimpleBattery::new(SimpleBatteryConfig {
        name: "bsk-Sat".to_string(),
        storage_capacity_j: STORAGE_CAPACITY_J,
        stored_charge_init_j: 0.8 * STORAGE_CAPACITY_J,
    });

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

    let sample_period = sampling_period_nanos(DURATION_NANOS, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut power_logs: Vec<MessageLog<PowerNodeUsageMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let mut battery_log = MessageLog::<PowerStorageStatusMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

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
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &battery.bat_power_out_msg => &mut battery_log.input,
        &rw_motor_torque.rw_motor_torque_out_msg => recorder.add_source::<ArrayMotorTorqueMsg>(
            CsvSourceConfig::columns(array_columns(
                "motor_torque_nm",
                "motorTorque_Nm",
                WHEEL_COUNT,
            )),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
        &wheel_speed_msg => recorder.add_source::<RwSpeedMsg>(
            CsvSourceConfig::columns(array_columns(
                "wheel_speeds_radps",
                "wheelSpeeds_rad_per_s",
                WHEEL_COUNT,
            )),
        ),
    );
    for (index, ((state, node), log)) in wheel_state_msgs
        .iter()
        .zip(&mut wheel_power)
        .zip(&mut power_logs)
        .enumerate()
    {
        connect!(&simulation,
            state => &mut node.model.rw_state_in_msg,
            &node.node_power_out_msg => battery.add_power_node_to_model(),
            &node.node_power_out_msg => &mut log.input,
            state => recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
            &node.node_power_out_msg => recorder.add_source::<PowerNodeUsageMsg>(
                CsvSourceConfig::columns([("net_power_w".to_string(), format!("rwPower_W_{index}"))]),
            ),
        );
    }
    connect!(&simulation,
        &battery.bat_power_out_msg => recorder.add_source::<PowerStorageStatusMsg>(
            CsvSourceConfig::columns([("storage_level_j", "storageLevel_Ws")]),
        ),
    );

    // Upstream's task order: wheels and spacecraft, navigation, the power
    // nodes and the battery, then the flight software.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
    }
    for node in &mut wheel_power {
        simulation.add_module("rw_power", node, DYNAMICS_PERIOD_NANOS, 85);
    }
    schedule! { simulation,
        "battery" => &mut battery, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "inertial_reference" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "tracking_error" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "mrp_feedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=50;
        "rw_motor_torque" => &mut rw_motor_torque, period=DYNAMICS_PERIOD_NANOS, priority=40;
        "rw_motor_torque_splitter" => &mut torque_splitter, period=DYNAMICS_PERIOD_NANOS, priority=35;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "battery_log" => &mut battery_log, period=sample_period, priority=0;
    }
    for log in &mut power_logs {
        simulation.add_module("power_log", log, sample_period, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        wheel_power: power_logs,
        battery: battery_log,
        csv_path,
    }
}

fn main() {
    let mut cli =
        Cli::from_env("scenario_attitude_feedback_rw_power [--[no-]rw-power-generation] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, True)`.
        vec![Config::new(cli.flag("rw-power-generation", true))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final battery storage {:.1} J)",
            results.csv_path.expect("CSV written").display(),
            results.battery.last().storage_level_j,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeFeedbackRWPower.py` only checks that
    //! both cases run. These tests check the power accounting and what the
    //! scenario documents.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[test]
    fn the_battery_sums_the_wheel_power_nodes() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_eq!(results.battery.values.len(), 101);
            assert_eq!(
                results.battery.values[0].storage_level_j,
                0.8 * STORAGE_CAPACITY_J
            );
            for (sample, status) in results.battery.values.iter().enumerate() {
                let nodes: f64 = results
                    .wheel_power
                    .iter()
                    .map(|log| log.values[sample].net_power_w)
                    .sum();
                assert!((status.current_net_power_w - nodes).abs() < 1.0e-9);
            }
            // The attitude history is the reaction-wheel scenario's.
            assert!(results.guidance.last().sigma_br.norm() < 1.0e-3);
        }
    }

    /// "The next simulation allows 50% of the breaking power to be returned
    /// to the power system. You can see how this will reduce the overall
    /// maneuver energy requirements."
    #[test]
    fn recovering_braking_power_saves_energy() {
        let drawn = quiet(TEST_PARAMETRIZATIONS[0]);
        let recovered = quiet(TEST_PARAMETRIZATIONS[1]);
        // Without recovery every wheel draws at least its 5 W base load.
        for log in &drawn.wheel_power {
            assert!(log.values.iter().all(|usage| usage.net_power_w <= -5.0));
        }
        // With it, a braking wheel draws less than its base load.
        assert!(
            recovered
                .wheel_power
                .iter()
                .flat_map(|log| &log.values)
                .any(|usage| usage.net_power_w > -5.0)
        );
        let initial = 0.8 * STORAGE_CAPACITY_J;
        let drawn_final = drawn.battery.last().storage_level_j;
        let recovered_final = recovered.battery.last().storage_level_j;
        assert!(
            drawn_final < recovered_final,
            "{drawn_final} vs {recovered_final}"
        );
        assert!(recovered_final < initial);
        // The 3 x 5 W base load alone takes 9 kJ over the 10 minutes.
        assert!(initial - drawn_final > 9_000.0 - 1.0);
    }
}
