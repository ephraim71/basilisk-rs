//! The Bevo 2 slew simulation shared by upstream's
//! `scenarioAttitudeConstraintViolation` and
//! `scenarioAttitudeConstrainedManeuver`, which differ in the attitude
//! guidance (`inertial3D` or `constrainedAttitudeManeuver`), the task step,
//! the duration and the number of samples.
//!
//! A 4 kg spacecraft (`I = diag(0.02, 0.1256, 0.1256) / 3 kg m²`) on a
//! 7000 km Earth orbit (`e = 0.0001`, `i = 33.3°`, `Ω = 148.2°`,
//! `ω = 347.8°`, `f = 135°`) from 2021 January 15 00:28:30 UTC, the Earth
//! (central) and the Sun as point masses, slews from one of four initial
//! attitudes at rest to the matching target with three custom wheels
//! (`maxMomentum = 0.01 N m s`, 6000 RPM, 1 mN m) along the body axes,
//! `mrpFeedback` (decay time 10 s, `ξ = 1`: `P = 3 max(I) / 10`,
//! `K = P² / max(I)`, no integral feedback) and `rwMotorTorque`.
//! `boreAngCalc` gives the angle between the Sun and the star tracker
//! (body `x`, keep-out cone of `starTrackerFov`) and the sun sensors (body
//! `y`, and body `z` with `use2SunSensors`, keep-in cones of
//! `sunSensorFov`). The Sun's ephemeris relative to the Earth (upstream's
//! `spiceInterface` with `zeroBase = 'Earth'`) comes from ANISE with
//! DE440s, published at initialization too (`staged::UpdatedAtInit`) as
//! `constrainedAttitudeManeuver` plans at its reset.
//!
//! The wheels take one motor command each; upstream's direct subscription
//! of the wheels to `rwMotorTorque`'s array is
//! `arrayMotorTorqueToSingleActuators` here, run right after it.

#![allow(dead_code)]

use std::path::PathBuf;

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::dyn_output::bore_ang_calc::{BoreAngCalc, BoreAngCalcConfig};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::fsw_algorithms::att_guidance::constrained_attitude_maneuver::{
    ConstrainedAttitudeManeuver, ConstrainedAttitudeManeuverConfig, CostFunction, TrajectoryFit,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, AttitudeReferenceMsg, BoreAngleMsg, Output,
    ReactionWheelStateMsg, RwSpeedMsg, SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{Module, connect, schedule};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use super::attitude::{
    HubState, att_tracking_error, bit, mrp_feedback, sampling_period_nanos, scenario_recorder,
    vehicle_config,
};
use super::common::{array_columns, elem2rv, vector_columns};
use super::grav_factory::Body;
use super::message_log::MessageLog;
use super::rw_factory::RwFactory;
use super::staged::UpdatedAtInit;

const WHEEL_COUNT: usize = 3;
const HUB_MASS_KG: f64 = 4.0;

/// Upstream `sigma_BN_start`, by `attitudeSetCase`.
pub const SIGMA_BN_START: [[f64; 3]; 4] = [
    [0.522, -0.065, 0.539],
    [0.314, -0.251, 0.228],
    [-0.378, 0.119, -0.176],
    [-0.412, 0.044, -0.264],
];
/// Upstream `sigma_BN_target`, by `attitudeSetCase`.
pub const SIGMA_BN_TARGET: [[f64; 3]; 4] = [
    [0.342, 0.223, -0.432],
    [0.326, -0.206, -0.823],
    [0.350, 0.220, -0.440],
    [0.350, 0.220, -0.440],
];

/// Upstream's `I`.
pub fn inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(0.02 / 3.0, 0.1256 / 3.0, 0.1256 / 3.0))
}

/// Upstream `timeInitString = "2021 JANUARY 15 00:28:30.0"` (UTC).
pub fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2021, 1, 15, 0, 28, 30, 0)
}

/// Upstream `run(show_plots, use2SunSensors, starTrackerFov, sunSensorFov,
/// attitudeSetCase)` parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Case {
    pub use_2_sun_sensors: bool,
    /// [deg]
    pub star_tracker_fov_deg: u32,
    /// [deg]
    pub sun_sensor_fov_deg: u32,
    pub attitude_set_case: usize,
}

impl Case {
    pub const fn new(
        use_2_sun_sensors: bool,
        star_tracker_fov_deg: u32,
        sun_sensor_fov_deg: u32,
        attitude_set_case: usize,
    ) -> Self {
        Self {
            use_2_sun_sensors,
            star_tracker_fov_deg,
            sun_sensor_fov_deg,
            attitude_set_case,
        }
    }

    /// Upstream's figure-name suffix, `str(int(use2SunSensors)) +
    /// str(starTrackerFov) + str(sunSensorFov) + str(attitudeSetCase)`.
    pub fn label(&self) -> String {
        format!(
            "{}{}{}{}",
            bit(self.use_2_sun_sensors),
            self.star_tracker_fov_deg,
            self.sun_sensor_fov_deg,
            self.attitude_set_case
        )
    }

    pub fn star_tracker_fov_rad(&self) -> f64 {
        f64::from(self.star_tracker_fov_deg).to_radians()
    }

    pub fn sun_sensor_fov_rad(&self) -> f64 {
        f64::from(self.sun_sensor_fov_deg).to_radians()
    }

    /// The body axes of the sun sensors.
    pub fn sun_sensor_axes(&self) -> Vec<Vector3<f64>> {
        if self.use_2_sun_sensors {
            vec![Vector3::y(), Vector3::z()]
        } else {
            vec![Vector3::y()]
        }
    }
}

/// The attitude guidance of the slew.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Guidance {
    /// `inertial3D` to the target: constraint-naive.
    Inertial3D,
    /// `constrainedAttitudeManeuver` (`N = 8`, `avgOmega = 0.04 rad/s`,
    /// interpolating B-spline, minimum-effort cost) to the target, with the
    /// star tracker as keep-out and the sun sensors as keep-in instruments.
    ConstrainedManeuver,
}

/// The scenario's simulation settings.
#[derive(Clone, Debug)]
pub struct Setup {
    /// The upstream scenario name, the CSV file stem.
    pub stem: &'static str,
    pub guidance: Guidance,
    pub step_nanos: u64,
    pub duration_nanos: u64,
    /// Upstream `numDataPoints`.
    pub data_points: u64,
}

/// The recorded histories.
pub struct Results {
    pub guidance: MessageLog<AttitudeGuidanceMsg>,
    pub reference: MessageLog<AttitudeReferenceMsg>,
    pub state: MessageLog<SpacecraftStateMsg>,
    /// Star tracker, then sun sensor `y` (and `z`).
    pub bore_angles: Vec<MessageLog<BoreAngleMsg>>,
    pub requested_torque: MessageLog<ArrayMotorTorqueMsg>,
    pub wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    pub wheel_speeds: MessageLog<RwSpeedMsg>,
    pub csv_path: Option<PathBuf>,
}

impl Results {
    /// The star tracker's Sun angle at each sample [rad].
    pub fn star_tracker_angles(&self) -> Vec<f64> {
        self.bore_angles[0]
            .values
            .iter()
            .map(|angle| angle.miss_angle_rad)
            .collect()
    }

    /// The smallest sun-sensor Sun angle at each sample [rad]: the keep-in
    /// constraint holds while one sensor sees the Sun.
    pub fn sun_sensor_angles(&self) -> Vec<f64> {
        (0..self.bore_angles[1].values.len())
            .map(|sample| {
                self.bore_angles[1..]
                    .iter()
                    .map(|log| log.values[sample].miss_angle_rad)
                    .fold(f64::INFINITY, f64::min)
            })
            .collect()
    }
}

/// The guidance module of the slew, behind one type.
enum GuidanceModule {
    Inertial(Inertial3D),
    Maneuver(Box<ConstrainedAttitudeManeuver>),
}

impl GuidanceModule {
    fn att_ref_out_msg(&self) -> &Output<AttitudeReferenceMsg> {
        match self {
            Self::Inertial(module) => &module.att_ref_out_msg,
            Self::Maneuver(module) => &module.att_ref_out_msg,
        }
    }
}

impl Module for GuidanceModule {
    fn init(&mut self) {
        match self {
            Self::Inertial(module) => Module::init(module),
            Self::Maneuver(module) => Module::init(module.as_mut()),
        }
    }

    fn update(&mut self, context: &basilisk_rs::SimulationContext) {
        match self {
            Self::Inertial(module) => module.update(context),
            Self::Maneuver(module) => module.update(context),
        }
    }
}

/// Runs the slew of `case` and writes
/// `examples/output/scenarios/<stem><label>.csv` when `write_csv`.
pub fn run(setup: &Setup, case: Case, write_csv: bool) -> Results {
    let inertia = inertia_kg_m2();
    let earth = Body::Earth;
    let (position_m, velocity_mps) = elem2rv(
        earth.mu_m3ps2(),
        7_000_000.0,
        0.0001,
        33.3_f64.to_radians(),
        148.2_f64.to_radians(),
        347.8_f64.to_radians(),
        135.0_f64.to_radians(),
    );
    let initial = HubState {
        position_m,
        velocity_mps,
        sigma_bn: Vector3::from(SIGMA_BN_START[case.attitude_set_case]),
        omega_bn_b_radps: Vector3::zeros(),
    };
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: setup.step_nanos,
        initial_position_m: initial.position_m,
        initial_velocity_mps: initial.velocity_mps,
        initial_sigma_bn: initial.sigma_bn,
        initial_omega_radps: initial.omega_bn_b_radps,
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("one central body");
    spacecraft
        .add_grav_body(Body::Sun.point_mass(false))
        .expect("unique gravity bodies");
    let mut sun = UpdatedAtInit::new(Body::Sun.ephemeris(Body::Earth), start_epoch());

    let mut rw_factory = RwFactory::default();
    for axis in [Vector3::x(), Vector3::y(), Vector3::z()] {
        rw_factory.create_custom(
            axis,
            0.0,
            6000.0,
            0.01,
            0.001,
            ReactionWheelModel::BalancedWheels,
        );
    }
    let rw_config_msg = Output::new(rw_factory.config_message());
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

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let target = Vector3::from(SIGMA_BN_TARGET[case.attitude_set_case]);
    let mut guidance = match setup.guidance {
        Guidance::Inertial3D => GuidanceModule::Inertial(Inertial3D::new(Inertial3DConfig {
            name: "inertial3D".to_string(),
            sigma_r0n: target,
        })),
        Guidance::ConstrainedManeuver => {
            let mut config = ConstrainedAttitudeManeuverConfig {
                name: "constrainedAttitudeManeuvering".to_string(),
                n: 8,
                trajectory_fit: TrajectoryFit::Interpolation,
                cost_function: CostFunction::MinimumEffort,
                sigma_bn_goal: target,
                omega_bn_b_goal_radps: Vector3::zeros(),
                avg_omega_radps: 0.04,
                ..ConstrainedAttitudeManeuverConfig::default()
            };
            config.append_keep_out_direction(Vector3::x(), case.star_tracker_fov_rad());
            for axis in case.sun_sensor_axes() {
                config.append_keep_in_direction(axis, case.sun_sensor_fov_rad());
            }
            GuidanceModule::Maneuver(Box::new(ConstrainedAttitudeManeuver::new(config)))
        }
    };
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let max_inertia = inertia.diagonal().max();
    let (decay_time_s, xi) = (10.0, 1.0);
    let p = 3.0 * max_inertia / decay_time_s;
    let mut controller = mrp_feedback((p / xi) * (p / xi) / max_inertia, -1.0, p, Vector3::zeros());
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut torque_splitter =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorTorqueSplitter".to_string(),
            num_actuators: WHEEL_COUNT,
        });
    let bore_angle = |name: &str, axis: Vector3<f64>| {
        BoreAngCalc::new(BoreAngCalcConfig {
            name: name.to_string(),
            bore_vec_body: axis,
            inertial_heading_inertial: Vector3::zeros(),
        })
    };
    let mut bore_angles = vec![bore_angle("starTrackerBoresight", Vector3::x())];
    for axis in case.sun_sensor_axes() {
        bore_angles.push(bore_angle("SunSensorBoresight", axis));
    }
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period =
        sampling_period_nanos(setup.duration_nanos, setup.step_nanos, setup.data_points);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut reference_log = MessageLog::<AttitudeReferenceMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut bore_logs: Vec<MessageLog<BoreAngleMsg>> =
        bore_angles.iter().map(|_| MessageLog::default()).collect();
    let mut requested_torque_log = MessageLog::<ArrayMotorTorqueMsg>::default();
    let mut wheel_logs: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..WHEEL_COUNT).map(|_| MessageLog::default()).collect();
    let mut wheel_speed_log = MessageLog::<RwSpeedMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder(&format!("{}{}", setup.stem, case.label()), write_csv);

    let mut simulation = Simulation::new(start_epoch(), false);
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    spacecraft.add_state_effector(reaction_wheels);
    simulation.connect(
        &sun.module.output_planet_msg,
        spacecraft.grav_body_input_mut("sun").expect("Sun gravity"),
    );
    if let GuidanceModule::Maneuver(maneuver) = &mut guidance {
        connect!(&simulation,
            &spacecraft.state_out => &mut maneuver.sc_state_in_msg,
            &vehicle_config_msg => &mut maneuver.vehicle_config_in_msg,
            &sun.module.output_planet_msg => &mut maneuver.keep_out_cel_body_in_msg,
            &sun.module.output_planet_msg => &mut maneuver.keep_in_cel_body_in_msg,
        );
    }
    let reference_msg = guidance.att_ref_out_msg().clone();
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &reference_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &reference_msg => &mut reference_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut requested_torque_log.input,
        &wheel_speed_msg => &mut wheel_speed_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &rw_motor_torque.rw_motor_torque_out_msg => recorder.add_source::<ArrayMotorTorqueMsg>(
            CsvSourceConfig::columns(array_columns("motor_torque_nm", "motorTorque_Nm", WHEEL_COUNT)),
        ),
        &wheel_speed_msg => recorder.add_source::<RwSpeedMsg>(CsvSourceConfig::columns(
            array_columns("wheel_speeds_radps", "wheelSpeeds_rad_per_s", WHEEL_COUNT),
        )),
    );
    let bore_names = ["starTracker", "sunSensorY", "sunSensorZ"];
    for ((calculator, log), name) in bore_angles.iter_mut().zip(&mut bore_logs).zip(bore_names) {
        connect!(&simulation,
            &spacecraft.state_out => &mut calculator.sc_state_in_msg,
            &sun.module.output_planet_msg => &mut calculator.cel_body_in_msg,
            &calculator.ang_out_msg => &mut log.input,
            &calculator.ang_out_msg => recorder.add_source::<BoreAngleMsg>(CsvSourceConfig::columns([(
                "miss_angle_rad".to_string(),
                format!("{name}MissAngle_rad"),
            )])),
        );
    }
    for (index, (state, log)) in wheel_state_msgs.iter().zip(&mut wheel_logs).enumerate() {
        connect!(&simulation,
            state => &mut log.input,
            state => recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
        );
    }

    let step = setup.step_nanos;
    // Upstream's task order: spiceInterface and the wheels (priority 2), the
    // spacecraft (1), then the flight software and the boresight angles.
    schedule! { simulation,
        "spiceObject" => &mut sun, period=step, priority=120;
        "Bevo2-Sat" => &mut spacecraft, period=step, priority=100;
        "SimpleNavigation" => &mut navigation, period=step, priority=90;
        "guidance" => &mut guidance, period=step, priority=80;
        "attErrorInertial3D" => &mut tracking_error, period=step, priority=70;
        "mrpFeedback" => &mut controller, period=step, priority=60;
        "rwMotorTorque" => &mut rw_motor_torque, period=step, priority=50;
        "rwMotorTorqueSplitter" => &mut torque_splitter, period=step, priority=45;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "reference_log" => &mut reference_log, period=sample_period, priority=0;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "requested_torque_log" => &mut requested_torque_log, period=sample_period, priority=0;
        "wheel_speed_log" => &mut wheel_speed_log, period=sample_period, priority=0;
    }
    for (calculator, log) in bore_angles.iter_mut().zip(&mut bore_logs) {
        simulation.add_module("boreAngCalc", calculator, step, 40);
        simulation.add_module("bore_log", log, sample_period, 0);
    }
    for log in &mut wheel_logs {
        simulation.add_module("wheel_log", log, sample_period, 0);
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(setup.duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        reference: reference_log,
        state: state_log,
        bore_angles: bore_logs,
        requested_torque: requested_torque_log,
        wheels: wheel_logs,
        wheel_speeds: wheel_speed_log,
        csv_path,
    }
}
