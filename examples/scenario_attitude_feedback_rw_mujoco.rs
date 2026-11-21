//! The classic reaction-wheel attitude feedback on MuJoCo dynamics: port of
//! upstream `examples/mujoco/scenarioAttitudeFeedbackRWMuJoCo.py`. Needs the
//! `mujoco` feature.
//!
//! A free-floating 750 kg `bus` (inertia diag(900, 800, 600) kg m²) with
//! three 9 kg reaction wheels on hinges (`rw1Spin`..`rw3Spin`, spin axes x,
//! y, z; Honeywell HR16-small inertias, 50 N m s at 6000 rpm), built from
//! upstream's XML string, each spun by a joint motor added with
//! `addJointSingleActuator`. Earth's point-mass gravity acts on all four
//! bodies (upstream `gravFactory.addBodiesTo(scene)`). The FSW stack of
//! `scenarioAttitudeFeedbackRW`: `simpleNav` on the bus center of mass,
//! `inertial3D`, `attTrackingError`, `mrpFeedback` (K 3.5, P 30, no
//! integral term) with the wheel speeds from the joint rates
//! (`scalarJointStatesToRWSpeed`), and `rwMotorTorque`, whose commands
//! `arrayMotorTorqueToSingleActuators` splits per wheel and
//! `saturationSingleActuator` limits to ±0.2 N m before the motors. The bus
//! starts on the classic orbit, at σ = (0.1, 0.2, -0.3) and
//! ω = (0.001, -0.01, 0.03) rad/s, the wheels at 100, 200 and 300 rpm.
//! Default RK4, 0.1 s task step, 10 min.
//!
//! basilisk-rs's `arrayMotorTorqueToSingleActuators` publishes
//! `MotorTorqueMsg`s (for its reaction-wheel effector); a
//! [`MotorTorqueToSingleActuators`] after it gives the `SingleActuatorMsg`s
//! upstream's module publishes.
//!
//! Writes `examples/output/scenarios/scenarioAttitudeFeedbackRWMuJoCo.csv`:
//! the attitude and rate errors, the limited wheel commands and the wheel
//! speeds, at upstream's sampling time (100 points, 6 s).
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks that
//! the controller brings the bus to the inertial reference.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::device_interface::saturation_single_actuator::{
    SaturationSingleActuator, SaturationSingleActuatorConfig,
};
use basilisk_rs::device_interface::scalar_joint_states_to_rw_speed::{
    ScalarJointStatesToRwSpeed, ScalarJointStatesToRwSpeedConfig,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, Output, RwArrayConfigMsg, ScalarJointStateMsg, SingleActuatorMsg,
    VehicleConfigMsg,
};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::MU_EARTH;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{rpm_to_radps, seconds, vector_columns};
use mj_scenarios::{
    GravitySourceSpec, MotorTorqueToSingleActuators, add_gravity_to_all_bodies, epoch,
    scalar_column, written_csv,
};
use nalgebra::{Matrix3, Vector3};
use recording::{History, sampling_time, scenario_csv};
use shared::Shared;

/// Upstream `timeStep`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `simulationTime` (10 min).
const DURATION_NANOS: u64 = seconds(600);
const BUS_MASS_KG: f64 = 750.0;
const BUS_INERTIA_DIAG: [f64; 3] = [900.0, 800.0, 600.0];
const WHEEL_COUNT: usize = 3;
const FILE_STEM: &str = "scenarioAttitudeFeedbackRWMuJoCo";

/// Upstream `getHr16SmallWheelParams`.
struct WheelParams {
    mass_kg: f64,
    u_max_nm: f64,
    js_kg_m2: f64,
    jt_kg_m2: f64,
}

fn hr16_small_wheel() -> WheelParams {
    let max_momentum = 50.0;
    let omega_max = rpm_to_radps(6000.0);
    let js = max_momentum / omega_max;
    WheelParams {
        mass_kg: 9.0,
        u_max_nm: 0.200,
        js_kg_m2: js,
        jt_kg_m2: 0.5 * js,
    }
}

/// Upstream `makeMjXmlString`.
fn xml(wheel: &WheelParams) -> String {
    let [ixx, iyy, izz] = BUS_INERTIA_DIAG;
    let (mass, js, jt) = (wheel.mass_kg, wheel.js_kg_m2, wheel.jt_kg_m2);
    format!(
        r#"
<mujoco model="busWith3Rws">
  <option>
    <flag contact="disable"/>
  </option>

  <worldbody>
    <body name="bus" pos="0 0 0">
      <freejoint name="busFree"/>

      <inertial pos="0 0 0" mass="{BUS_MASS_KG}"
               diaginertia="{ixx} {iyy} {izz}"/>

      <geom type="box" size="0.6 0.5 0.4" rgba="0.6 0.6 0.6 1"/>

      <body name="rw1" pos="0 0 0">
        <joint name="rw1Spin" type="hinge" axis="1 0 0" limited="false"/>
        <inertial pos="0 0 0" mass="{mass}"
                 diaginertia="{js} {jt} {jt}"/>
        <geom type="cylinder" size="0.08 0.03" rgba="0.2 0.2 0.8 1" euler="0 90 0"/>
      </body>

      <body name="rw2" pos="0 0 0">
        <joint name="rw2Spin" type="hinge" axis="0 1 0" limited="false"/>
        <inertial pos="0 0 0" mass="{mass}"
                 diaginertia="{jt} {js} {jt}"/>
        <geom type="cylinder" size="0.08 0.03" rgba="0.2 0.8 0.2 1" euler="90 0 0"/>
      </body>

      <body name="rw3" pos="0.5 0.5 0.5">
        <joint name="rw3Spin" type="hinge" axis="0 0 1" limited="false"/>
        <inertial pos="0 0 0" mass="{mass}"
                 diaginertia="{jt} {jt} {js}"/>
        <geom type="cylinder" size="0.08 0.03" rgba="0.8 0.2 0.2 1"/>
      </body>

    </body>
  </worldbody>
</mujoco>
"#
    )
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: Vec<(u64, AttitudeGuidanceMsg)>,
    commands: Vec<Vec<(u64, SingleActuatorMsg)>>,
    wheel_rates: Vec<Vec<(u64, ScalarJointStateMsg)>>,
    u_max_nm: f64,
}

fn run(write_csv: bool) -> Results {
    let wheel = hr16_small_wheel();
    let mut scene = MjScene::new(&xml(&wheel)).expect("valid bus XML");
    scene.name = "mujocoScene".to_string();

    // A motor on each wheel's spin joint.
    let joints: Vec<String> = (1..=WHEEL_COUNT).map(|i| format!("rw{i}Spin")).collect();
    let actuators: Vec<String> = (1..=WHEEL_COUNT).map(|i| format!("rw{i}Act")).collect();
    for (actuator, joint) in actuators.iter().zip(&joints) {
        scene.add_joint_single_actuator(actuator, joint);
    }

    // Earth on every body.
    let mu_earth = MU_EARTH * 1e9;
    add_gravity_to_all_bodies(
        &mut scene,
        &[GravitySourceSpec {
            name: "earth",
            mu_m3ps2: mu_earth,
            is_central_body: true,
            ephemeris: None,
        }],
    );

    // Navigation and FSW.
    let mut navigation = SimpleNavigation::new("simpleNav");
    scene
        .body("bus")
        .center_of_mass()
        .state_out_msg
        .connect_to(&mut navigation.spacecraft_state_in_msg);
    let mut inertial_guidance = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::zeros(),
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attTrackingError".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let mut rw_config = RwArrayConfigMsg {
        num_rw: WHEEL_COUNT,
        ..Default::default()
    };
    for (index, axis) in [Vector3::x(), Vector3::y(), Vector3::z()]
        .into_iter()
        .enumerate()
    {
        rw_config.spin_axes_body[index] = axis;
        rw_config.spin_axis_inertias_kg_m2[index] = wheel.js_kg_m2;
        rw_config.max_motor_torques_nm[index] = wheel.u_max_nm;
    }
    let rw_config_msg = Output::new(rw_config);
    let mut rw_speeds = ScalarJointStatesToRwSpeed::new(ScalarJointStatesToRwSpeedConfig {
        name: "scalarJointStatesToRWSpeed".to_string(),
        num_joints: WHEEL_COUNT,
    });
    for (joint, input) in joints.iter().zip(rw_speeds.joint_state_in_msgs.iter_mut()) {
        scene
            .scalar_joint(joint)
            .state_dot_out_msg
            .connect_to(input);
    }
    let mut controller = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: 3.5,
        ki: -1.0,
        p: 30.0,
        integral_limit: 0.0,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: Matrix3::from_diagonal(&Vector3::from(BUS_INERTIA_DIAG)),
        ..Default::default()
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "arrayMotorTorqueToSingleActuators".to_string(),
            num_actuators: WHEEL_COUNT,
        });
    let mut bridge = MotorTorqueToSingleActuators::new(WHEEL_COUNT);
    for (output, input) in splitter
        .actuator_out_msgs
        .iter()
        .zip(bridge.motor_torque_in_msgs.iter_mut())
    {
        output.connect_to(input);
    }
    // One limiter per wheel motor.
    let mut limiters: Vec<SaturationSingleActuator> = (1..=WHEEL_COUNT)
        .map(|i| {
            SaturationSingleActuator::new(SaturationSingleActuatorConfig {
                name: format!("rw{i}TorqueLimiter"),
                min_input: -wheel.u_max_nm,
                max_input: wheel.u_max_nm,
                ..Default::default()
            })
        })
        .collect();
    for ((limiter, command), actuator) in limiters
        .iter_mut()
        .zip(&bridge.actuator_out_msgs)
        .zip(&actuators)
    {
        command.connect_to(&mut limiter.actuator_in_msg);
        limiter
            .actuator_out_msg
            .connect_to(&mut scene.single_actuator_mut(actuator).actuator_in_msg);
    }

    let sample_nanos = sampling_time(DURATION_NANOS, STEP_NANOS, 100);
    let wheel_rates: Vec<_> = joints
        .iter()
        .map(|joint| scene.scalar_joint(joint).state_dot_out_msg.clone())
        .collect();
    let commands: Vec<_> = limiters
        .iter()
        .map(|limiter| limiter.actuator_out_msg.clone())
        .collect();
    let mut guidance_log = History::<AttitudeGuidanceMsg>::new();
    let mut command_logs: Vec<History<SingleActuatorMsg>> =
        (0..WHEEL_COUNT).map(|_| History::new()).collect();
    let mut rate_logs: Vec<History<ScalarJointStateMsg>> =
        (0..WHEEL_COUNT).map(|_| History::new()).collect();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    let guidance_out = tracking_error.att_guid_out_msg.clone();

    // Initial conditions, set after initialization: the classic orbit and
    // attitude, and the wheel speeds.
    let (position_m, velocity_mps) = elements_to_state(
        mu_earth,
        &ClassicElements {
            semi_major_axis: 10_000_000.0,
            eccentricity: 0.01,
            inclination_rad: 33.3_f64.to_radians(),
            raan_rad: 48.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
            ..Default::default()
        },
    );
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &inertial_guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
            &rw_config_msg => &mut controller.rw_params_in_msg,
            &rw_speeds.rw_speed_out_msg => &mut controller.rw_speeds_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
            &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
            &rw_motor_torque.rw_motor_torque_out_msg => &mut splitter.torque_in_msg,
            &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        );
        for ((command, log), (rate, rate_log)) in commands
            .iter()
            .zip(command_logs.iter_mut())
            .zip(wheel_rates.iter().zip(rate_logs.iter_mut()))
        {
            connect!(&simulation, command => &mut log.input, rate => &mut rate_log.input);
        }
        // Upstream's single task, in insertion order.
        schedule! { simulation,
            "mujocoScene" => &mut scene_task, period=STEP_NANOS, priority=100;
            "simpleNav" => &mut navigation, period=STEP_NANOS, priority=90;
            "inertial3D" => &mut inertial_guidance, period=STEP_NANOS, priority=80;
            "attTrackingError" => &mut tracking_error, period=STEP_NANOS, priority=70;
            "scalarJointStatesToRWSpeed" => &mut rw_speeds, period=STEP_NANOS, priority=60;
            "mrpFeedback" => &mut controller, period=STEP_NANOS, priority=50;
            "rwMotorTorque" => &mut rw_motor_torque, period=STEP_NANOS, priority=40;
            "arrayMotorTorqueToSingleActuators" => &mut splitter, period=STEP_NANOS, priority=30;
            "singleActuatorBridge" => &mut bridge, period=STEP_NANOS, priority=29;
            "attErrorLog" => &mut guidance_log, period=sample_nanos, priority=0;
        }
        for limiter in limiters.iter_mut() {
            let name = limiter.config.name.clone();
            simulation.add_module(name, limiter, STEP_NANOS, 20);
        }
        for (index, (log, rate_log)) in command_logs
            .iter_mut()
            .zip(rate_logs.iter_mut())
            .enumerate()
        {
            simulation.add_module(format!("rwCmdLog{index}"), log, sample_nanos, 0);
            simulation.add_module(format!("rwOmegaLog{index}"), rate_log, sample_nanos, 0);
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &guidance_out => csv.add_source::<AttitudeGuidanceMsg>(
                    CsvSourceConfig::columns(
                        vector_columns("sigma_br", "sigma_BR")
                            .into_iter()
                            .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
                    ),
                ),
            );
            for (index, (command, rate)) in commands.iter().zip(&wheel_rates).enumerate() {
                connect!(&simulation,
                    command => csv.add_source::<SingleActuatorMsg>(
                        scalar_column("input", &format!("rwMotorTorque_Nm_{index}")),
                    ),
                    rate => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("rwOmega_rad_per_s_{index}")),
                    ),
                );
            }
            simulation.add_module("csv", csv, sample_nanos, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            {
                let mut bus = scene.body_mut("bus");
                bus.set_position(position_m);
                bus.set_velocity(velocity_mps);
                bus.set_attitude(Vector3::new(0.1, 0.2, -0.3));
                bus.set_attitude_rate(Vector3::new(0.001, -0.01, 0.03));
            }
            for (index, rpm) in [100.0, 200.0, 300.0].into_iter().enumerate() {
                scene
                    .body_mut(&format!("rw{}", index + 1))
                    .set_scalar_joint_velocity(&joints[index], rpm_to_radps(rpm));
            }
        }
        simulation.run_for(DURATION_NANOS);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        guidance: guidance_log.into_samples(),
        commands: command_logs
            .into_iter()
            .map(History::into_samples)
            .collect(),
        wheel_rates: rate_logs.into_iter().map(History::into_samples).collect(),
        u_max_nm: wheel.u_max_nm,
    }
}

fn main() {
    Cli::from_env("scenario_attitude_feedback_rw_mujoco").finish();
    let results = run(true);
    let (_, last) = results.guidance.last().expect("recorded guidance");
    println!(
        "final sigma_BR {:.3e}, omega_BR {:.3e} rad/s",
        last.sigma_br.norm(),
        last.omega_br_b_radps.norm()
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! attitude control it demonstrates.

    use super::*;

    #[test]
    fn the_wheels_bring_the_bus_to_the_inertial_reference() {
        let results = run(false);
        assert_eq!(results.guidance.len(), 101);
        let (_, first) = &results.guidance[0];
        let (_, last) = results.guidance.last().unwrap();
        // From |σ| = 0.37 the error decays by orders of magnitude in the
        // 10 min (P = 30 N m s on 600 to 900 kg m²).
        assert!(first.sigma_br.norm() > 0.3);
        assert!(
            last.sigma_br.norm() < 1e-2 * first.sigma_br.norm(),
            "{}",
            last.sigma_br
        );
        assert!(
            last.omega_br_b_radps.norm() < 1e-3,
            "{}",
            last.omega_br_b_radps
        );
        // The limiters hold every command within ±uMax, and they bind
        // early on.
        for log in &results.commands {
            assert!(
                log.iter()
                    .all(|(_, command)| command.input.abs() <= results.u_max_nm)
            );
        }
        assert!(results.commands.iter().any(|log| {
            log.iter()
                .any(|(_, command)| command.input.abs() == results.u_max_nm)
        }));
        // The wheels absorbed the bus momentum: they no longer spin at
        // their initial speeds.
        let initial = results.wheel_rates[2][0].1.state;
        let last_rate = results.wheel_rates[2].last().unwrap().1.state;
        assert!(
            (last_rate - initial).abs() > 1.0,
            "{initial} -> {last_rate}"
        );
    }
}
