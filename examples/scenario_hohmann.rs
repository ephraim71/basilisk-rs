//! Port of upstream `examples/scenarioHohmann.py`: a Hohmann transfer with
//! impulsive burns, the attitude flight software switching between Hill
//! pointing and burn-attitude modes.
//!
//! A 750 kg spacecraft with three Honeywell HR16 wheels (100, 200 and 300 RPM)
//! on a circular 30 degree orbit of radius `--r-first` (default 7000 km). The
//! flight software runs in mode tasks, switched as upstream's
//! `createNewEvent` actions do (`disableAllTasks`, then enable the mode's
//! task and the control task):
//!
//! * `hillPoint`: Hill-frame pointing;
//! * `firstBurn` / `secondBurn`: velocity pointing rotated by -45 / +45
//!   degrees about the first axis (`velocityPoint` into `mrpRotation`);
//! * the control task: `attTrackingError`, `mrpFeedback` (`K = 3.5`,
//!   `P = 30`, no integral term) and `rwMotorTorque`.
//!
//! Mode sequence, as upstream: Hill pointing for 0.4 orbits, first-burn
//! attitude for 0.1 orbits, the transfer burn to `--r-second` (default
//! 42 164 km; upstream's test uses 30 000 km), 0.2 of the transfer in
//! first-burn attitude, 0.6 in Hill pointing, 0.2 in second-burn attitude,
//! the circularization burn, 0.1 of the final orbit in second-burn attitude
//! and 0.2 in Hill pointing. The burns are velocity changes through the
//! spacecraft's [`HubStateHandle`](basilisk_rs::spacecraft::HubStateHandle)
//! (upstream `velRef.setState`). The Earth's state comes from the ANISE
//! ephemeris with the Earth as zero base (upstream `createSpiceInterface`,
//! from 2021 May 4 07:47:48.965 UTC) through `ephemerisConverter`.
//!
//! Upstream's mode tasks all write one attitude-reference message
//! (`AttRefMsg_C_addAuthor`). Here each mode's guidance keeps its own output
//! and a mode switch reconnects the tracking error's reference input to the
//! active one: a reconnected [`Input`](basilisk_rs::messages::Input) clone
//! follows the new producer.
//!
//! Writes `scenarioHohmann.csv` (position, `sigma_BN`, `sigma_BR` and the
//! reference `sigma_RN`, every second). `tests` covers
//! `test_scenarioHohmann.py` (`rFirst = 7e6`, `rSecond = 3e7`), which only
//! runs the scenario, by checking the orbit it reaches and that each mode's
//! attitude is tracked.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/task_switch.rs"]
mod task_switch;

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::att_guidance::mrp_rotation::{MrpRotation, MrpRotationConfig};
use basilisk_rs::fsw_algorithms::att_guidance::velocity_point::{
    VelocityPoint, VelocityPointConfig,
};
use basilisk_rs::fsw_algorithms::att_tracking_error::{AttTrackingError, AttTrackingErrorConfig};
use basilisk_rs::fsw_algorithms::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, NavigationAttitudeMsg, Output, RwArrayConfigMsg,
    SpacecraftStateMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use common::{rpm_to_radps, vector_columns};
use grav_factory::Body;
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, run_until, sec2nano};
use recording::{History, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;
use task_switch::{Switched, TaskSwitch};

const USAGE: &str = "scenario_hohmann [--r-first 7000000] [--r-second 42164000]";

/// Upstream `simulationTimeStep`.
const STEP_NANOS: u64 = 1_000_000_000;
/// Upstream `samplingTime`; the recorders sit in the 1 s control task.
const SAMPLE_NANOS: u64 = 500_000_000;

/// Upstream `rwFactory.create('Honeywell_HR16', axis, maxMomentum=50.,
/// Omega=rpm)`: a balanced wheel at the body origin.
fn hr16(name: &str, axis_body: Vector3<f64>, initial_rpm: f64) -> ReactionWheelStateEffectorConfig {
    let maximum_speed_radps = rpm_to_radps(6_000.0);
    let mut config =
        ReactionWheelStateEffectorConfig::balanced(name, Vector3::zeros(), axis_body, 0.2, 50.0);
    config.js_kg_m2 = 50.0 / maximum_speed_radps;
    config.jt_kg_m2 = 0.5 * config.js_kg_m2;
    config.jg_kg_m2 = config.jt_kg_m2;
    config.max_speed_radps = maximum_speed_radps;
    config.initial_omega_radps = rpm_to_radps(initial_rpm);
    config
}

/// Upstream `scSim.modeRequest`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    HillPoint,
    FirstBurn,
    SecondBurn,
}

/// One phase of the run: its mode and duration.
#[derive(Clone, Copy, Debug)]
pub struct Phase {
    pub mode: Mode,
    pub start_nanos: u64,
    pub end_nanos: u64,
}

pub struct Results {
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub attitude_errors: Vec<(u64, AttitudeGuidanceMsg)>,
    pub phases: Vec<Phase>,
    pub r_second_m: f64,
}

/// Upstream `run(show_plots, rFirst, rSecond)`.
pub fn run(r_first_m: f64, r_second_m: f64) -> Results {
    let inertia = Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0));
    let mu = Body::Earth.mu_m3ps2();
    let (position_m, velocity_mps) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: r_first_m,
            inclination_rad: 30.0_f64.to_radians(),
            ..Default::default()
        },
    );

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central body");
    let hub = spacecraft.hub_state_handle();
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);
    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "ephem data".to_string(),
        body_names: vec!["earth".to_string()],
    });

    let wheel_configs = [
        hr16("RW1", Vector3::x(), 100.0),
        hr16("RW2", Vector3::y(), 200.0),
        hr16("RW3", Vector3::z(), 300.0),
    ];
    let mut rw_config = RwArrayConfigMsg {
        num_rw: wheel_configs.len(),
        ..Default::default()
    };
    for (index, wheel) in wheel_configs.iter().enumerate() {
        rw_config.spin_axes_body[index] = wheel.spin_axis_body;
        rw_config.spin_axis_inertias_kg_m2[index] = wheel.js_kg_m2;
        rw_config.max_motor_torques_nm[index] = wheel.max_torque_nm;
    }
    let rw_config = Output::new(rw_config);
    let mut reaction_wheels = ReactionWheelStateEffector::new("RW_cluster");
    for config in wheel_configs {
        reaction_wheels.add_reaction_wheel(config);
    }
    let wheel_speeds = reaction_wheels.rw_speed_out_msg.clone();
    let mut wheel_commands =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwCommands".to_string(),
            num_actuators: 3,
        });
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let vehicle_config = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        ..VehicleConfigMsg::default()
    });

    // The mode tasks, enabled one at a time, and the control task.
    let first_burn_task = TaskSwitch::new(true);
    let second_burn_task = TaskSwitch::new(true);
    let hill_point_task = TaskSwitch::new(true);
    let mut first_burn = Switched::new(
        VelocityPoint::new(VelocityPointConfig {
            name: "velocityPoint".to_string(),
            mu_m3ps2: mu,
        }),
        &first_burn_task,
    );
    let mut first_burn_rotation = Switched::new(
        MrpRotation::new(MrpRotationConfig {
            sigma_rr0: Vector3::new((-std::f64::consts::PI / 8.0).tan(), 0.0, 0.0),
            ..MrpRotationConfig::default()
        }),
        &first_burn_task,
    );
    let mut second_burn = Switched::new(
        VelocityPoint::new(VelocityPointConfig {
            name: "velocityPoint".to_string(),
            mu_m3ps2: mu,
        }),
        &second_burn_task,
    );
    let mut second_burn_rotation = Switched::new(
        MrpRotation::new(MrpRotationConfig {
            sigma_rr0: Vector3::new((std::f64::consts::PI / 8.0).tan(), 0.0, 0.0),
            ..MrpRotationConfig::default()
        }),
        &second_burn_task,
    );
    let mut hill_point =
        Switched::new(HillPoint::new(HillPointConfig::default()), &hill_point_task);
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attError".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let mut control = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: 3.5,
        ki: -1.0,
        p: 30.0,
        integral_limit: -1.0,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let mut motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });

    // The mode references, and every input of upstream's shared `attRefMsg`.
    let references = [
        (Mode::HillPoint, hill_point.module.att_ref_out_msg.clone()),
        (
            Mode::FirstBurn,
            first_burn_rotation.module.att_ref_out_msg.clone(),
        ),
        (
            Mode::SecondBurn,
            second_burn_rotation.module.att_ref_out_msg.clone(),
        ),
    ];
    let mut csv = scenario_csv("scenarioHohmann");
    let mut state_history = History::<SpacecraftStateMsg>::new();
    let mut error_history = History::<AttitudeGuidanceMsg>::new();
    let mut reference_inputs = vec![tracking_error.att_ref_in_msg.clone()];
    reference_inputs.push(
        csv.add_source::<AttitudeReferenceMsg>(CsvSourceConfig::columns(vector_columns(
            "sigma_rn", "sigma_RN",
        )))
        .clone(),
    );

    let p1 = orbit_period_s(mu, r_first_m);
    let mut phases = Vec::new();
    {
        let mut simulation = Simulation::new(
            Epoch::from_gregorian_utc(2021, 5, 4, 7, 47, 48, 965_000_000),
            false,
        );
        for (output, wheel) in wheel_commands
            .actuator_out_msgs
            .iter()
            .zip(reaction_wheels.wheels_mut())
        {
            simulation.connect(output, &mut wheel.motor_torque_in_msg);
        }
        spacecraft.add_state_effector(reaction_wheels);
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &earth_ephemeris.output_planet_msg => &mut ephemeris_converter.planet_in_msgs[0],
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut first_burn.module.trans_nav_in_msg,
            &ephemeris_converter.ephem_out_msgs[0] => &mut first_burn.module.cel_body_in_msg,
            &first_burn.module.att_ref_out_msg => &mut first_burn_rotation.module.att_ref_in_msg,
            &navigation.translation_out_msg => &mut second_burn.module.trans_nav_in_msg,
            &ephemeris_converter.ephem_out_msgs[0] => &mut second_burn.module.cel_body_in_msg,
            &second_burn.module.att_ref_out_msg => &mut second_burn_rotation.module.att_ref_in_msg,
            &navigation.translation_out_msg => &mut hill_point.module.trans_nav_in_msg,
            &ephemeris_converter.ephem_out_msgs[0] => &mut hill_point.module.cel_body_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &tracking_error.att_guid_out_msg => &mut control.guid_in_msg,
            &vehicle_config => &mut control.veh_config_in_msg,
            &rw_config => &mut control.rw_params_in_msg,
            &wheel_speeds => &mut control.rw_speeds_in_msg,
            &rw_config => &mut motor_torque.rw_params_in_msg,
            &control.cmd_torque_out_msg => &mut motor_torque.veh_control_in_msg,
            &motor_torque.rw_motor_torque_out_msg => &mut wheel_commands.torque_in_msg,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &navigation.attitude_out_msg => csv.add_source::<NavigationAttitudeMsg>(
                CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
            ),
            &tracking_error.att_guid_out_msg => csv.add_source::<AttitudeGuidanceMsg>(
                CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
            ),
            &spacecraft.state_out => &mut state_history.input,
            &tracking_error.att_guid_out_msg => &mut error_history.input,
        );
        // Upstream: the dynamics process (RW effector, spacecraft, navigation,
        // ephemeris), then the mode tasks (priority 5), then the control task.
        schedule! { simulation,
            "spiceObject" => &mut earth_ephemeris, period=STEP_NANOS, priority=120;
            "bsk-Sat" => &mut spacecraft, period=STEP_NANOS, priority=110;
            "ephem data" => &mut ephemeris_converter, period=STEP_NANOS, priority=100;
            "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=90;
            "firstBurn" => &mut first_burn, period=STEP_NANOS, priority=52;
            "firstBurnMRPRotation" => &mut first_burn_rotation, period=STEP_NANOS, priority=51;
            "secondBurn" => &mut second_burn, period=STEP_NANOS, priority=50;
            "secondBurnMRPRotation" => &mut second_burn_rotation, period=STEP_NANOS, priority=49;
            "hillPoint" => &mut hill_point, period=STEP_NANOS, priority=48;
            "attError" => &mut tracking_error, period=STEP_NANOS, priority=40;
            "mrpFeedback" => &mut control, period=STEP_NANOS, priority=39;
            "rwMotorTorque" => &mut motor_torque, period=STEP_NANOS, priority=38;
            "rwCommands" => &mut wheel_commands, period=STEP_NANOS, priority=37;
            "csv" => &mut csv, period=SAMPLE_NANOS, priority=0;
            "scRec" => &mut state_history, period=SAMPLE_NANOS, priority=0;
            "attErrorLog" => &mut error_history, period=SAMPLE_NANOS, priority=0;
        }

        // Upstream's events: `disableAllTasks()`, then enable the mode's task
        // (and the control task, never disabled here).
        let mut set_mode = |simulation: &Simulation<'_>, mode: Mode| {
            first_burn_task.disable();
            second_burn_task.disable();
            hill_point_task.disable();
            match mode {
                Mode::HillPoint => hill_point_task.enable(),
                Mode::FirstBurn => first_burn_task.enable(),
                Mode::SecondBurn => second_burn_task.enable(),
            }
            let (_, reference) = references
                .iter()
                .find(|(reference_mode, _)| *reference_mode == mode)
                .expect("every mode has a reference");
            for input in &mut reference_inputs {
                simulation.connect(reference, input);
            }
        };
        let mut stop_nanos = 0;
        let mut run_phase = |simulation: &mut Simulation<'_>, mode: Mode, duration_s: f64| {
            let start_nanos = stop_nanos;
            stop_nanos += sec2nano(duration_s);
            set_mode(simulation, mode);
            run_until(simulation, stop_nanos);
            phases.push(Phase {
                mode,
                start_nanos,
                end_nanos: stop_nanos,
            });
        };
        // Upstream's burn: the speed of the transfer orbit to `r_second_m`
        // at the current radius, along the velocity.
        let burn = || {
            let (r, v) = (hub.position_m(), hub.velocity_mps());
            let transfer_a = (r.norm() + r_second_m) * 0.5;
            let target_speed = ((2.0 * mu / r.norm()) - (mu / transfer_a)).sqrt();
            hub.set_velocity_mps(v + (target_speed - v.norm()) * v.normalize());
            transfer_a
        };

        run_phase(&mut simulation, Mode::HillPoint, 0.4 * p1);
        run_phase(&mut simulation, Mode::FirstBurn, 0.1 * p1);
        let transfer_a = burn();
        let transfer_s = std::f64::consts::PI * (transfer_a.powi(3) / mu).sqrt();
        run_phase(&mut simulation, Mode::FirstBurn, 0.2 * transfer_s);
        run_phase(&mut simulation, Mode::HillPoint, 0.6 * transfer_s);
        run_phase(&mut simulation, Mode::SecondBurn, 0.2 * transfer_s);
        burn();
        let p2 = orbit_period_s(mu, r_second_m);
        run_phase(&mut simulation, Mode::SecondBurn, 0.1 * p2);
        run_phase(&mut simulation, Mode::HillPoint, 0.2 * p2);
    }
    csv.flush().expect("flush scenario CSV");

    Results {
        states: state_history.into_samples(),
        attitude_errors: error_history.into_samples(),
        phases,
        r_second_m,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let results = run(
        args.value("--r-first", 7_000_000.0),
        args.value("--r-second", 42_164_000.0),
    );
    let (_, last) = results.states.last().expect("samples");
    let elements = basilisk_rs::utilities::orbital_motion::state_to_elements(
        Body::Earth.mu_m3ps2(),
        last.position_m,
        last.velocity_mps,
    );
    println!(
        "wrote scenarioHohmann.csv: final a = {:.1} km (target {:.1} km), e = {:.2e}",
        elements.semi_major_axis / 1e3,
        results.r_second_m / 1e3,
        elements.eccentricity
    );
}

#[cfg(test)]
mod tests {
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    use super::run;
    use crate::grav_factory::Body;

    /// `test_scenarioHohmann.py` (`rFirst = 7e6`, `rSecond = 3e7`) only runs
    /// the scenario. The burns leave a circular orbit of radius `rSecond`, and
    /// at the end of every mode phase the attitude has converged onto that
    /// mode's reference.
    #[test]
    fn transfer_reaches_the_second_orbit_and_modes_are_tracked() {
        let results = run(7e6, 3e7);
        let (_, last) = results.states.last().expect("samples");
        let elements =
            state_to_elements(Body::Earth.mu_m3ps2(), last.position_m, last.velocity_mps);
        assert!(
            (elements.semi_major_axis / 3e7 - 1.0).abs() < 1e-3,
            "a = {} m",
            elements.semi_major_axis
        );
        assert!(
            elements.eccentricity < 1e-3,
            "e = {}",
            elements.eccentricity
        );

        assert_eq!(results.phases.len(), 7);
        for phase in &results.phases {
            let (_, error) = results
                .attitude_errors
                .iter()
                .rev()
                .find(|(time, _)| *time <= phase.end_nanos)
                .expect("samples in every phase");
            assert!(
                error.sigma_br.norm() < 1e-2,
                "{:?} phase ending at {} s: |sigma_BR| = {}",
                phase.mode,
                phase.end_nanos as f64 * 1e-9,
                error.sigma_br.norm()
            );
        }
    }
}
