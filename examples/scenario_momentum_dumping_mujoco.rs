//! Reaction-wheel momentum dumping with thrusters on MuJoCo dynamics: port
//! of upstream `examples/mujoco/scenarioMomentumDumpingMuJoCo.py`, the
//! MuJoCo translation of `scenarioMomentumDumping`. Needs the `mujoco`
//! feature.
//!
//! A free-floating 2500 kg `hub` (inertia diag(1700, 1700, 1800) kg m²
//! about its center of mass, 1.28 m above the hub origin) with four
//! Honeywell HR16 (100 N m s) wheels on hinges along canted axes
//! (`rw<i>Spin`, motors limited to ±0.2 N m by their `ctrlrange`), and eight
//! 5 N MOOG Monarc-5 thrusters as hub sites with motors along their z axes
//! (`ctrlrange` 0 to 5 N), built from upstream's XML string
//! ([`add_rws_xml`], [`add_thrusters_xml`]). Earth's point-mass gravity on
//! the hub only. The wheels point the hub at a fixed inertial attitude
//! (`simpleNav`, `inertial3D`, `attTrackingError`, `mrpFeedback` for a 10 s
//! decay time, `rwMotorTorque`, then `arrayMotorTorqueToSingleActuators` and
//! one `saturationSingleActuator` per wheel); the thrusters dump the excess
//! wheel momentum: `thrMomentumManagement` (hs_min 80 N m s) requests the
//! momentum change, `thrForceMapping` maps it to thruster impulses,
//! `thrMomentumDumping` (100 control periods between firings, 0.02 s
//! minimum) to on-times, and `thrOnTimeToForce`, in the scene's dynamics
//! task, to forces at every integrator stage. The wheels start at 4000,
//! 2000, 3500 and 0 rpm. After 10 s `thrMomentumManagement` is reset, so
//! its update at 11 s writes a fresh request (the one written at 0 s is not
//! taken as new by `thrMomentumDumping`), and the run continues to 5 min.
//! Default RK4 at the 0.1 s dynamics step, FSW at 1 s.
//!
//! Stand-ins: upstream's `thrMomentumManagement.Reset(10 s)` between runs is
//! a [`Staged`] reset applied before its first update after 10 s;
//! basilisk-rs's `arrayMotorTorqueToSingleActuators` publishes
//! `MotorTorqueMsg`s, so [`MotorTorqueToSingleActuators`] gives the
//! `SingleActuatorMsg`s upstream's module publishes; `thrMomentumDumping`
//! has one on-time output per thruster (basilisk-rs's per-device ports),
//! each read by one `thrOnTimeToForce` input.
//!
//! Writes `examples/output/scenarios/scenarioMomentumDumpingMuJoCo.csv`:
//! the attitude and rate errors, the commanded wheel torques, the
//! momentum-change request, the thruster impulses, on-times and delivered
//! forces, and the wheel speeds, every 0.1 s (upstream's sampling time for
//! 5000 points).
//!
//! `test_scenarioMujoco.py` runs the scenario and checks, in
//! `test_momentum_dumping_thruster_limit`, that a generated thruster
//! delivers exactly its configured maximum when commanded twice that
//! (maxThrust 3 N and 8 N); that check is ported in `tests`, which also
//! checks the dumping the scenario demonstrates.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/rw_factory.rs"]
mod rw_factory;
#[path = "support/shared.rs"]
mod shared;
#[path = "support/staged.rs"]
mod staged;
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::device_interface::saturation_single_actuator::{
    SaturationSingleActuator, SaturationSingleActuatorConfig,
};
use basilisk_rs::device_interface::scalar_joint_states_to_rw_speed::{
    ScalarJointStatesToRwSpeed, ScalarJointStatesToRwSpeedConfig,
};
use basilisk_rs::dynamics::gravity::PointMassGravityModel;
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_control::thr_momentum_management::{
    ThrMomentumManagement, ThrMomentumManagementConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_force_mapping::{
    ThrForceMapping, ThrForceMappingConfig, ThrForceSign,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thr_momentum_dumping::{
    ThrMomentumDumping, ThrMomentumDumpingConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, BodyTorqueCommandMsg, Output, ScalarJointStateMsg,
    SingleActuatorMsg, SpacecraftStateMsg, ThrusterArrayForceCommandMsg, ThrusterCommandMsg,
    VehicleConfigMsg,
};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
use basilisk_rs::mujoco_dynamics::thr_on_time_to_force::{
    ThrOnTimeToForce, ThrOnTimeToForceConfig,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, rpm_to_radps, seconds, vector_columns};
use message_log::MessageLog;
use mj_scenarios::{MotorTorqueToSingleActuators, epoch, scalar_column, written_csv};
use nalgebra::{Matrix3, Vector3};
use recording::{sampling_time, scenario_csv};
use rw_factory::RwFactory;
use shared::Shared;
use staged::Staged;
use thruster_factory::{ThrusterFactory, ThrusterModel, ThrusterSpec};

/// Upstream `simulationTimeStepDyn`.
const DYN_STEP_NANOS: u64 = 100_000_000;
/// Upstream `simulationTimeStepFsw`.
const FSW_STEP_NANOS: u64 = seconds(1);
/// Upstream `simulationTime` (5 min).
const DURATION_NANOS: u64 = seconds(300);
/// When upstream resets `thrMomentumManagement`.
const RESET_NANOS: u64 = seconds(10);
/// Upstream `muEarth` \[m^3/s^2\].
const MU_EARTH: f64 = 0.3986004415e15;
/// Upstream `hs_min` \[N m s\].
const HS_MIN_NMS: f64 = 80.0;
/// Upstream `initialOmegas` \[rpm\].
const INITIAL_RPM: [f64; 4] = [4000.0, 2000.0, 3500.0, 0.0];
const FILE_STEM: &str = "scenarioMomentumDumpingMuJoCo";

/// Upstream `addRWsXML(rwPos, rwAxes, rwFactory, maxMomentum=100,
/// baseIndent=3)`: the wheel bodies and their motors, with each wheel
/// registered in `factory` (`Honeywell_HR16`).
fn add_rws_xml(
    positions: &[Vector3<f64>],
    axes: &[Vector3<f64>],
    factory: &mut RwFactory,
    max_momentum_nms: f64,
) -> (String, String, Vec<ReactionWheelStateEffectorConfig>) {
    let pad = "\t".repeat(3);
    let mut bodies = Vec::new();
    let mut actuators = Vec::new();
    for (index, (position, axis)) in positions.iter().zip(axes).enumerate() {
        let n = index + 1;
        let wheel = factory
            .create_honeywell_hr16(
                *axis,
                max_momentum_nms,
                0.0,
                Vector3::zeros(),
                ReactionWheelModel::BalancedWheels,
            )
            .clone();
        bodies.push(format!(
            "{pad}<body name = \"rw{n}Spin\" pos = \"{} {} {}\" zaxis = \"{} {} {}\">\n\
             {pad}\t<joint name = \"rw{n}Joint\" pos = \"0 0 0\" axis = \"0 0 1\" ref = \"0\"/>\n\
             {pad}\t<inertial pos = \"0 0 0\" mass = \"{}\" diaginertia = \"{} {} {}\"/>\n\
             {pad}\t<geom name = \"rw{n}Geom\" type = \"cylinder\" size = \"0.2 0.05\" contype = \"0\" conaffinity = \"0\"/>\n\
             {pad}</body>",
            position.x,
            position.y,
            position.z,
            axis.x,
            axis.y,
            axis.z,
            wheel.mass_kg,
            wheel.jt_kg_m2,
            wheel.jt_kg_m2,
            wheel.js_kg_m2,
        ));
        actuators.push(format!(
            "\t\t<motor name = \"rw{n}Act\" joint = \"rw{n}Joint\" ctrlrange = \"{} {}\" ctrllimited = \"true\"/>",
            -wheel.max_torque_nm, wheel.max_torque_nm
        ));
    }
    let wheels = factory.wheels.clone();
    (bodies.join("\n"), actuators.join("\n"), wheels)
}

/// Upstream `addThrustersXML(thrustLocs, thrustDirs, thrFactory,
/// maxThrust=5.0, baseIndent=2)`: a site per thruster and a motor along its
/// z axis limited to `[0, maxThrust]`, each thruster registered in
/// `factory` (`MOOG_Monarc_5` with `MaxThrust = maxThrust`).
fn add_thrusters_xml(
    locations: &[Vector3<f64>],
    directions: &[Vector3<f64>],
    factory: &mut ThrusterFactory,
    max_thrust_n: f64,
) -> (String, String, Vec<ThrusterSpec>) {
    let pad = "\t".repeat(2);
    let mut sites = Vec::new();
    let mut actuators = Vec::new();
    for (index, (location, direction)) in locations.iter().zip(directions).enumerate() {
        let n = index + 1;
        let thruster = factory.create(ThrusterModel::MoogMonarc5, *location, *direction, None);
        thruster.max_thrust_n = max_thrust_n;
        sites.push(format!(
            "{pad}\t<site name = \"thrusterSite{n}\" pos = \"{} {} {}\" zaxis = \"{} {} {}\"/>",
            location.x, location.y, location.z, direction.x, direction.y, direction.z
        ));
        actuators.push(format!(
            "{pad}<motor name = \"thruster{n}\" site = \"thrusterSite{n}\" gear = \"0 0 1 0 0 0\" ctrlrange = \"0 {max_thrust_n}\"/>"
        ));
    }
    (
        sites.join("\n"),
        actuators.join("\n"),
        factory.thrusters.clone(),
    )
}

/// Upstream `makeMjXmlString()`: the hub, wheels and thrusters, and the
/// factories that hold the FSW configuration.
fn make_xml() -> (String, RwFactory, ThrusterFactory) {
    let (hub_mass, (ixx, iyy, izz)) = (2500.0, (1700.0, 1700.0, 1800.0));
    let c = 0.5_f64.sqrt();
    let rw_positions = [Vector3::new(0.0, 0.0, 1.28); 4];
    let rw_axes = [
        Vector3::new(c, 0.0, c),
        Vector3::new(0.0, c, c),
        Vector3::new(-c, 0.0, c),
        Vector3::new(0.0, -c, c),
    ];
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
    ]
    .map(Vector3::from);
    let directions = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
    .map(Vector3::from);
    let mut rw_factory = RwFactory::default();
    let mut thr_factory = ThrusterFactory::default();
    let (rw_bodies, rw_acts, _) = add_rws_xml(&rw_positions, &rw_axes, &mut rw_factory, 100.0);
    let (thr_sites, thr_acts, _) =
        add_thrusters_xml(&locations, &directions, &mut thr_factory, 5.0);
    let xml = format!(
        r#"<mujoco model = "busWithRWsAndThrusters">
    <compiler angle = "radian" meshdir = ""/>
    <worldbody>
        <body name = "hub" pos = "0 0 0">
            <freejoint name = "busFree"/>
            <inertial pos = "0 0 {b}" mass = "{hub_mass}" diaginertia = "{ixx} {iyy} {izz}"/>
            <geom name = "hubVisual" type = "box" size = "1 1 1.28" rgba = "1 1 1 1"/>
{thr_sites}
{rw_bodies}
        </body>
    </worldbody>
    <actuator>
{rw_acts}
{thr_acts}
    </actuator>
</mujoco>"#
    );
    (xml, rw_factory, thr_factory)
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: Vec<(u64, AttitudeGuidanceMsg)>,
    delta_h: Vec<(u64, BodyTorqueCommandMsg)>,
    wheel_rates: Vec<Vec<(u64, ScalarJointStateMsg)>>,
    on_times: Vec<Vec<(u64, ThrusterCommandMsg)>>,
    thrusts: Vec<Vec<(u64, SingleActuatorMsg)>>,
    /// `(Js, spin axis)` per wheel.
    wheel_axes: Vec<(f64, Vector3<f64>)>,
    max_thrust_n: Vec<f64>,
}

fn run(write_csv: bool) -> Results {
    let (xml, rw_factory, thr_factory) = make_xml();
    let mut scene = MjScene::new(&xml).expect("valid momentum-dumping scene");
    scene.name = "mujocoScene".to_string();
    scene.extra_eom_call = true;
    let wheel_count = rw_factory.wheels.len();
    let thruster_count = thr_factory.thrusters.len();
    let wheel_joints: Vec<String> = (1..=wheel_count).map(|i| format!("rw{i}Joint")).collect();

    // Earth's gravity on the hub only.
    let mut gravity = NBodyGravity::new("gravity");
    gravity.add_gravity_source(
        "earth",
        PointMassGravityModel::new(MU_EARTH).expect("positive mu"),
        true,
    );
    gravity.add_gravity_target_at_body(&mut scene, "hub", "hub");
    scene.add_model_to_dynamics_task(gravity, -1);

    let (position_m, velocity_mps) = elements_to_state(
        MU_EARTH,
        &ClassicElements {
            semi_major_axis: 7000.0 * 1000.0,
            eccentricity: 0.0001,
            inclination_rad: 33.3_f64.to_radians(),
            raan_rad: 148.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 335.0_f64.to_radians(),
            ..Default::default()
        },
    );

    // Navigation and FSW.
    let mut navigation = SimpleNavigation::new("simpleNav");
    scene
        .body("hub")
        .center_of_mass()
        .state_out_msg
        .connect_to(&mut navigation.spacecraft_state_in_msg);
    let mut inertial_guidance = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::zeros(),
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attErrorInertial3D".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let (decay_time, xi, max_inertia) = (10.0, 1.0, 1800.0);
    let p = 3.0 * max_inertia / decay_time;
    let ki = -1.0;
    let mut controller = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: (p / xi) * (p / xi) / max_inertia,
        ki,
        p,
        integral_limit: 2.0 / ki * 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let control_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: control_axes.to_vec(),
    });
    let mut desaturation = Staged::new(ThrMomentumManagement::new(ThrMomentumManagementConfig {
        name: "thrMomentumManagement".to_string(),
        hs_min_nms: HS_MIN_NMS,
    }))
    .reset_after(RESET_NANOS);
    let mut force_mapping = ThrForceMapping::new(ThrForceMappingConfig {
        name: "thrForceMapping".to_string(),
        control_axes_b: control_axes,
        thr_force_sign: ThrForceSign::Positive,
        ang_err_thresh_rad: 3.15,
        ..ThrForceMappingConfig::default()
    });
    let mut dumping = ThrMomentumDumping::new(ThrMomentumDumpingConfig {
        name: "thrDump".to_string(),
        max_counter_value: 100,
        thr_min_fire_time_s: 0.02,
    });
    let rw_config_msg = Output::new(rw_factory.config_message());
    let thr_config_msg = Output::new(thr_factory.config_message());
    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: Matrix3::from_diagonal(&Vector3::new(1700.0, 1700.0, 1800.0)),
        ..Default::default()
    });

    // Wheel commands: split, bridged, limited, to the wheel motors.
    let u_max = rw_factory.wheels[0].max_torque_nm;
    let mut splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "arrayMotorTorqueToSingleActuators".to_string(),
            num_actuators: wheel_count,
        });
    let mut bridge = MotorTorqueToSingleActuators::new(wheel_count);
    for (output, input) in splitter
        .actuator_out_msgs
        .iter()
        .zip(bridge.motor_torque_in_msgs.iter_mut())
    {
        output.connect_to(input);
    }
    let mut limiters: Vec<SaturationSingleActuator> = (1..=wheel_count)
        .map(|i| {
            SaturationSingleActuator::new(SaturationSingleActuatorConfig {
                name: format!("rw{i}TorqueLimiter"),
                min_input: -u_max,
                max_input: u_max,
                ..Default::default()
            })
        })
        .collect();
    for (index, (limiter, command)) in limiters
        .iter_mut()
        .zip(&bridge.actuator_out_msgs)
        .enumerate()
    {
        command.connect_to(&mut limiter.actuator_in_msg);
        limiter.actuator_out_msg.connect_to(
            &mut scene
                .single_actuator_mut(&format!("rw{}Act", index + 1))
                .actuator_in_msg,
        );
    }

    // Thruster on-times to forces, at every integrator stage.
    let max_thrust_n: Vec<f64> = thr_factory
        .thrusters
        .iter()
        .map(|thruster| thruster.max_thrust_n)
        .collect();
    let mut force_converter = ThrOnTimeToForce::new(ThrOnTimeToForceConfig {
        name: "thrOnTimeToForce".to_string(),
        thr_mag_n: max_thrust_n.clone(),
    });
    let on_time_outputs: Vec<Output<ThrusterCommandMsg>> = (0..thruster_count)
        .map(|_| dumping.add_thruster_command_output().clone())
        .collect();
    for (output, input) in on_time_outputs
        .iter()
        .zip(force_converter.on_time_in_msgs.iter_mut())
    {
        output.connect_to(input);
    }
    let thrust_outputs = force_converter.thruster_force_out_msgs.clone();
    for (index, force) in thrust_outputs.iter().enumerate() {
        force.connect_to(
            &mut scene
                .single_actuator_mut(&format!("thruster{}", index + 1))
                .actuator_in_msg,
        );
    }
    scene.add_module_to_dynamics_task("thrOnTimeToForce", force_converter, -1);

    // Wheel speeds from the joint rates, for the controller and the
    // momentum management.
    let mut rw_speeds = ScalarJointStatesToRwSpeed::new(ScalarJointStatesToRwSpeedConfig {
        name: "scalarJointStatesToRWSpeed".to_string(),
        num_joints: wheel_count,
    });
    let wheel_rates: Vec<_> = wheel_joints
        .iter()
        .map(|joint| scene.scalar_joint(joint).state_dot_out_msg.clone())
        .collect();
    for (rate, input) in wheel_rates
        .iter()
        .zip(rw_speeds.joint_state_in_msgs.iter_mut())
    {
        rate.connect_to(input);
    }

    let sample_nanos = sampling_time(DURATION_NANOS, DYN_STEP_NANOS, 5000).max(DYN_STEP_NANOS);
    let guidance_out = tracking_error.att_guid_out_msg.clone();
    let delta_h_out = desaturation.module.delta_h_out_msg.clone();
    let motor_torque_out = rw_motor_torque.rw_motor_torque_out_msg.clone();
    let impulse_out = force_mapping.thr_force_cmd_out_msg.clone();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut delta_h_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut rate_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        (0..wheel_count).map(|_| MessageLog::default()).collect();
    let mut on_time_logs: Vec<MessageLog<ThrusterCommandMsg>> =
        (0..thruster_count).map(|_| MessageLog::default()).collect();
    let mut thrust_logs: Vec<MessageLog<SingleActuatorMsg>> =
        (0..thruster_count).map(|_| MessageLog::default()).collect();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    let hub_state = scene.body("hub").center_of_mass().state_out_msg.clone();

    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &inertial_guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &guidance_out => &mut controller.guid_in_msg,
            &rw_config_msg => &mut controller.rw_params_in_msg,
            &rw_speeds.rw_speed_out_msg => &mut controller.rw_speeds_in_msg,
            &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
            &vehicle_config_msg => &mut force_mapping.veh_config_in_msg,
            &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
            &rw_motor_torque.rw_motor_torque_out_msg => &mut splitter.torque_in_msg,
            &rw_speeds.rw_speed_out_msg => &mut desaturation.module.rw_speeds_in_msg,
            &rw_config_msg => &mut desaturation.module.rw_config_data_in_msg,
            &thr_config_msg => &mut force_mapping.thr_config_in_msg,
            &delta_h_out => &mut force_mapping.cmd_torque_in_msg,
            &thr_config_msg => &mut dumping.thruster_conf_in_msg,
            &delta_h_out => &mut dumping.delta_h_in_msg,
            &force_mapping.thr_force_cmd_out_msg => &mut dumping.thruster_impulse_in_msg,
            &guidance_out => &mut guidance_log.input,
            &delta_h_out => &mut delta_h_log.input,
        );
        for (rate, log) in wheel_rates.iter().zip(rate_logs.iter_mut()) {
            connect!(&simulation, rate => &mut log.input);
        }
        for ((on_time, log), (thrust, thrust_log)) in on_time_outputs
            .iter()
            .zip(on_time_logs.iter_mut())
            .zip(thrust_outputs.iter().zip(thrust_logs.iter_mut()))
        {
            connect!(&simulation, on_time => &mut log.input, thrust => &mut thrust_log.input);
        }
        // Upstream's tasks: dynamics (3), FSW (2), recording (1).
        schedule! { simulation,
            "mujocoScene" => &mut scene_task, period=DYN_STEP_NANOS, priority=300;
            "simpleNav" => &mut navigation, period=DYN_STEP_NANOS, priority=299;
            "scalarJointStatesToRWSpeed" => &mut rw_speeds, period=FSW_STEP_NANOS, priority=210;
            "inertial3D" => &mut inertial_guidance, period=FSW_STEP_NANOS, priority=209;
            "attErrorInertial3D" => &mut tracking_error, period=FSW_STEP_NANOS, priority=208;
            "mrpFeedback" => &mut controller, period=FSW_STEP_NANOS, priority=207;
            "rwMotorTorque" => &mut rw_motor_torque, period=FSW_STEP_NANOS, priority=206;
            "thrMomentumManagement" => &mut desaturation, period=FSW_STEP_NANOS, priority=205;
            "thrForceMapping" => &mut force_mapping, period=FSW_STEP_NANOS, priority=204;
            "thrDump" => &mut dumping, period=FSW_STEP_NANOS, priority=203;
            "arrayMotorTorqueToSingleActuators" => &mut splitter, period=FSW_STEP_NANOS, priority=202;
            "singleActuatorBridge" => &mut bridge, period=FSW_STEP_NANOS, priority=201;
            "attErrorLog" => &mut guidance_log, period=sample_nanos, priority=100;
            "deltaHLog" => &mut delta_h_log, period=sample_nanos, priority=100;
        }
        for limiter in limiters.iter_mut() {
            let name = limiter.config.name.clone();
            simulation.add_module(name, limiter, FSW_STEP_NANOS, 200);
        }
        for (index, log) in rate_logs.iter_mut().enumerate() {
            simulation.add_module(format!("rwSpeedLog{index}"), log, sample_nanos, 100);
        }
        for (index, (log, thrust_log)) in on_time_logs
            .iter_mut()
            .zip(thrust_logs.iter_mut())
            .enumerate()
        {
            simulation.add_module(format!("onTimeLog{index}"), log, sample_nanos, 100);
            simulation.add_module(format!("thrForceLog{index}"), thrust_log, sample_nanos, 100);
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
                &motor_torque_out => csv.add_source::<ArrayMotorTorqueMsg>(
                    CsvSourceConfig::columns(array_columns("motor_torque_nm", "motorTorque_Nm", wheel_count)),
                ),
                &delta_h_out => csv.add_source::<BodyTorqueCommandMsg>(
                    CsvSourceConfig::columns(vector_columns("torque_request_body_nm", "deltaH_Nms")),
                ),
                &impulse_out => csv.add_source::<ThrusterArrayForceCommandMsg>(
                    CsvSourceConfig::columns(array_columns("thrust_force_n", "thrImpulse_Ns", thruster_count)),
                ),
                &hub_state => csv.add_source::<SpacecraftStateMsg>(
                    CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
                ),
            );
            for (index, (on_time, thrust)) in
                on_time_outputs.iter().zip(&thrust_outputs).enumerate()
            {
                connect!(&simulation,
                    on_time => csv.add_source::<ThrusterCommandMsg>(
                        scalar_column("on_time_s", &format!("OnTimeRequest_s_{index}")),
                    ),
                    thrust => csv.add_source::<SingleActuatorMsg>(
                        scalar_column("input", &format!("thrForce_N_{index}")),
                    ),
                );
            }
            for (index, rate) in wheel_rates.iter().enumerate() {
                connect!(&simulation,
                    rate => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("wheelSpeeds_rad_per_s_{index}")),
                    ),
                );
            }
            simulation.add_module("csv", csv, sample_nanos, 100);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            {
                let mut hub = scene.body_mut("hub");
                hub.set_position(position_m);
                hub.set_velocity(velocity_mps);
            }
            for (index, rpm) in INITIAL_RPM.into_iter().enumerate() {
                scene
                    .body_mut(&format!("rw{}Spin", index + 1))
                    .set_scalar_joint_velocity(&wheel_joints[index], rpm_to_radps(rpm));
            }
        }
        // Coast 10 s; the momentum management is reset before its 11 s
        // update.
        simulation.run_for(RESET_NANOS);
        simulation.run_for(DURATION_NANOS - RESET_NANOS);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        guidance: guidance_log.into_samples(),
        delta_h: delta_h_log.into_samples(),
        wheel_rates: rate_logs
            .into_iter()
            .map(MessageLog::into_samples)
            .collect(),
        on_times: on_time_logs
            .into_iter()
            .map(MessageLog::into_samples)
            .collect(),
        thrusts: thrust_logs
            .into_iter()
            .map(MessageLog::into_samples)
            .collect(),
        wheel_axes: rw_factory
            .wheels
            .iter()
            .map(|wheel| (wheel.js_kg_m2, wheel.spin_axis_body))
            .collect(),
        max_thrust_n,
    }
}

impl Results {
    /// The net wheel momentum `Σ Js Ω ĝ` at sample `index` \[N m s\].
    fn wheel_momentum(&self, index: usize) -> Vector3<f64> {
        self.wheel_axes
            .iter()
            .zip(&self.wheel_rates)
            .map(|((js, axis), rates)| js * rates[index].1.state * axis)
            .sum()
    }
}

fn main() {
    Cli::from_env("scenario_momentum_dumping_mujoco").finish();
    let results = run(true);
    let last = results.wheel_rates[0].len() - 1;
    println!(
        "wheel momentum {:.1} -> {:.1} N m s",
        results.wheel_momentum(0).norm(),
        results.wheel_momentum(last).norm()
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upstream `test_momentum_dumping_thruster_limit(maxThrust)`: one
    /// thruster from [`add_thrusters_xml`] on a 1 kg free body, commanded
    /// twice its maximum for 1 s, delivers exactly its maximum (the
    /// actuator's `ctrlrange`).
    #[test]
    fn momentum_dumping_thruster_limit() {
        for max_thrust in [3.0, 8.0] {
            let mut factory = ThrusterFactory::default();
            let (sites, actuators, thrusters) = add_thrusters_xml(
                &[Vector3::zeros()],
                &[Vector3::z()],
                &mut factory,
                max_thrust,
            );
            assert_eq!(thrusters[0].max_thrust_n, max_thrust);
            let body_mass = 1.0;
            let xml = format!(
                r#"<mujoco>
        <worldbody>
            <body name="hub">
                <freejoint/>
                <inertial pos="0 0 0" mass="{body_mass}" diaginertia="1 1 1"/>
                {sites}
            </body>
        </worldbody>
        <actuator>{actuators}</actuator>
    </mujoco>"#
            );
            let mut scene = MjScene::new(&xml).expect("valid thruster scene");
            let command = Output::new(SingleActuatorMsg {
                input: 2.0 * max_thrust,
            });
            command.connect_to(&mut scene.single_actuator_mut("thruster1").actuator_in_msg);
            let state = scene.body("hub").center_of_mass().state_out_msg.clone();
            let mut log = MessageLog::<SpacecraftStateMsg>::default();
            {
                let mut simulation = Simulation::new(epoch(), false);
                connect!(&simulation, &state => &mut log.input);
                schedule! { simulation,
                    "scene" => &mut scene, period=100_000_000, priority=10;
                    "recorder" => &mut log, period=100_000_000, priority=0;
                }
                simulation.run_for(seconds(1));
            }
            let last = log.last();
            let expected = Vector3::new(0.0, 0.0, max_thrust * 1.0 / body_mass);
            assert!(
                (last.velocity_mps - expected).abs().max() <= 1e-12,
                "{} vs {expected}",
                last.velocity_mps
            );
        }
    }

    /// The scenario: the thrusters dump the wheel momentum to hs_min while
    /// the wheels hold the attitude.
    #[test]
    fn the_thrusters_dump_the_wheel_momentum_to_hs_min() {
        let results = run(false);
        assert_eq!(results.guidance.len(), 3001);
        let first = results.wheel_momentum(0);
        let last = results.wheel_momentum(results.wheel_rates[0].len() - 1);
        assert!(first.norm() > 110.0, "{}", first.norm());
        assert!(
            (last.norm() - HS_MIN_NMS).abs() < 0.05 * HS_MIN_NMS,
            "{}",
            last.norm()
        );
        assert!(last.angle(&first).to_degrees() < 5.0);
        // The request written at 11 s, after the reset: `-(1 - hs_min /
        // |H|) H` of the wheels then; zero is never requested before.
        let request_at = |nanos: u64| {
            results
                .delta_h
                .iter()
                .find(|(time, _)| *time == nanos)
                .map(|(_, request)| request.torque_request_body_nm)
                .unwrap()
        };
        let at_11 = request_at(seconds(11));
        let wheels_at_11 = results.wheel_momentum(110);
        let expected = -(1.0 - HS_MIN_NMS / wheels_at_11.norm()) * wheels_at_11;
        assert!(
            (at_11 - expected).norm() < 0.05 * expected.norm(),
            "{at_11} vs {expected}"
        );
        // The thrusters fire, never above their maximum.
        let fired = results
            .thrusts
            .iter()
            .filter(|log| log.iter().any(|(_, force)| force.input > 0.0))
            .count();
        assert!(fired >= 2, "{fired}");
        for (log, max) in results.thrusts.iter().zip(&results.max_thrust_n) {
            assert!(
                log.iter()
                    .all(|(_, force)| force.input >= 0.0 && force.input <= *max)
            );
        }
        for log in &results.on_times {
            assert!(log.iter().all(|(_, command)| command.on_time_s >= 0.0));
        }
        assert!(results.guidance.last().unwrap().1.sigma_br.norm() < 1.0e-2);
    }
}
