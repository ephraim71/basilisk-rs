//! Momentum management of a solar-electric-propulsion spacecraft with a
//! gimballed thruster, solar-array articulation and center-of-mass
//! estimation: port of upstream `examples/scenarioSepMomentumManagement.py`.
//!
//! A 2500 kg spacecraft (hub center of mass `[0.008, -0.010, 1.214] m`, a
//! full inertia tensor) on a 100 million km heliocentric orbit carries four
//! Honeywell HR16 wheels (100 N m s, at rest), two 85 kg rotating solar
//! arrays (`spinningBodyOneDOF`), and a 0.27 N electric thruster (Isp
//! 1600 s, firing for the whole run) on a two-axis gimballed platform
//! (`spinningBodyTwoDOF`). A ten-facet SRP model includes the arrays as
//! articulated facets.
//!
//! Flight software on a 2 s task: `oneAxisSolarArrayPoint` keeps the
//! thrust axis along the platform reference heading and the arrays' drive
//! axis perpendicular to the Sun; `attTrackingError`, `mrpFeedback`
//! (`K = 9`, `P = 275`, `Ki = 1e-5`, control law 1) and `rwMotorTorque` drive
//! the wheels; `solarArrayReference` and PID motors turn each array to the
//! Sun; `thrusterPlatformState` and `thrustCMEstimation` estimate the center
//! of mass from the integral feedback torque; PID motors and a
//! `torqueScheduler` drive the platform to the angles that
//! `thrusterPlatformReference` (on a 1 h task) computes: through the center
//! of mass, or offset from it to dump wheel momentum (`K = 2.5e-4`). Dynamics
//! on a 0.5 s task, 6 hours, histories sampled every 2 s.
//!
//! Upstream `run(swirlTorque, thrMomManagement, saMomManagement,
//! cmEstimation, showPlots)` maps to the flags `--swirl-torque`,
//! `--thr-mom-management`, `--sa-mom-management` (switching the arrays to
//! momentum-management pointing at 3 h) and `--cm-estimation` (the platform
//! and arrays use the estimated center of mass instead of the true one). The
//! upstream script's own default is `0101`; `--all` runs the two
//! parametrizations of upstream's `test_scenarioSepMomentumManagement.py`,
//! `0000` and `0101`.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `spiceInterface` with `zeroBase = 'Sun'` gives the Sun at
//!   the origin: a static Sun message here, which `simpleNav`'s Sun input
//!   (upstream `sunStateInMsg`), the SRP model and the boresight modules
//!   read;
//! * the wheels take one command each: `arrayMotorTorqueToSingleActuators`
//!   splits `rwMotorTorque`'s array right after it;
//! * the thruster's on-time command is a static message: upstream's
//!   one-week `OnTimeRequest`;
//! * `thrustCMEstimation` writes its estimate into the stand-alone vehicle
//!   configuration message that starts at the initial guess, as upstream's
//!   `VehicleConfigMsg_C_addAuthor`;
//! * upstream's array pointing-mode change at 3 h (`saMomManagement`) is a
//!   [`Staged`] change and reset;
//! * upstream logs the SRP force and torque with a variable logger; the SRP
//!   effector is inside the spacecraft here and has no output message, so
//!   the SRP torque figures are not reproduced.
//!
//! Writes `examples/output/scenarios/scenarioSepMomentumManagement<s><t><a><c>.csv`
//! (the four flags).

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

use std::f64::consts::PI;
use std::path::PathBuf;

use attitude::{att_tracking_error, bit, scenario_recorder};
use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::dyn_output::bore_ang_calc::{BoreAngCalc, BoreAngCalcConfig};
use basilisk_rs::dynamics::facet_srp_dynamic_effector::{
    FacetSRPDynamicEffector, FacetSRPDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffector,
};
use basilisk_rs::dynamics::spinning_body_one_dof_state_effector::{
    SpinningBodyOneDofConfig, SpinningBodyOneDofStateEffector,
};
use basilisk_rs::dynamics::spinning_body_two_dof_state_effector::{
    SpinningBodyTwoDofConfig, SpinningBodyTwoDofStateEffector,
};
use basilisk_rs::dynamics::thruster_state_effector::{
    ThrusterStateEffector, ThrusterStateEffectorConfig,
};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::one_axis_solar_array_point::{
    OneAxisSolarArrayPoint, OneAxisSolarArrayPointConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::hinged_rigid_body_pid_motor::{
    HingedRigidBodyPidMotor, HingedRigidBodyPidMotorConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::solar_array_reference::{
    PointingMode, SolarArrayReference, SolarArrayReferenceConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thruster_platform_reference::{
    ThrusterPlatformReference, ThrusterPlatformReferenceConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::thruster_platform_state::{
    ThrusterPlatformState, ThrusterPlatformStateConfig,
};
use basilisk_rs::fsw_algorithms::effector_interfaces::torque_scheduler::{
    LockFlag, TorqueScheduler, TorqueSchedulerConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::state_estimation::thrust_cm_estimation::{
    ThrustCmEstimation, ThrustCmEstimationConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, BoreAngleMsg, CmEstimationDataMsg,
    HingedRigidBodyMsg, NavigationAttitudeMsg, Output, PlanetStateMsg, RwSpeedMsg, SunEphemerisMsg,
    ThrusterCommandMsg, ThrusterConfigMsg, ThrusterOutputMsg, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::sensors::simple_mass_props::SimpleMassProps;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_SUN};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::{euler2, euler3, prv_to_dcm};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use rw_factory::RwFactory;
use staged::Staged;

/// Upstream `simulationTimeStepDyn = macros.sec2nano(0.5)`.
const DYNAMICS_PERIOD_NANOS: u64 = 500_000_000;
/// Upstream `simulationTimeStepFsw = macros.sec2nano(2)`.
const FSW_PERIOD_NANOS: u64 = 2_000_000_000;
/// Upstream `simulationTimeStepPlt = macros.hour2nano(1)`.
const PLATFORM_REFERENCE_PERIOD_NANOS: u64 = 3_600_000_000_000;
/// Upstream `simulationTime = macros.hour2nano(6)`.
const DURATION_NANOS: u64 = 6 * 3_600_000_000_000;
const WHEEL_COUNT: usize = 4;
/// Upstream `maxThrust` \[N\].
const MAX_THRUST_N: f64 = 0.27;
/// Upstream `r_CB_B_0`: the initial center-of-mass guess \[m\].
const INITIAL_CM_GUESS_M: [f64; 3] = [0.04, -0.05, 1.25];

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    swirl_torque: bool,
    thr_mom_management: bool,
    sa_mom_management: bool,
    cm_estimation: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(
        swirl_torque: bool,
        thr_mom_management: bool,
        sa_mom_management: bool,
        cm_estimation: bool,
    ) -> Self {
        Self {
            swirl_torque,
            thr_mom_management,
            sa_mom_management,
            cm_estimation,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioSepMomentumManagement{}{}{}{}",
            bit(self.swirl_torque),
            bit(self.thr_mom_management),
            bit(self.sa_mom_management),
            bit(self.cm_estimation),
        )
    }
}

/// The parametrizations of upstream `test_scenarioSepMomentumManagement.py`
/// (`withFunction` False and True).
const TEST_PARAMETRIZATIONS: [Config; 2] = [
    Config::new(false, false, false, false),
    Config::new(false, true, false, true),
];

/// The recorded histories, every 2 s.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    wheel_speeds: MessageLog<RwSpeedMsg>,
    true_cm: MessageLog<VehicleConfigMsg>,
    cm_estimate: MessageLog<CmEstimationDataMsg>,
    array_angles: Vec<MessageLog<HingedRigidBodyMsg>>,
    array_references: Vec<MessageLog<HingedRigidBodyMsg>>,
    array_boresights: Vec<MessageLog<BoreAngleMsg>>,
    /// Upstream `RW.Js` and `RW.gsHat_B` of each wheel.
    wheel_axes: Vec<(f64, Vector3<f64>)>,
    csv_path: Option<PathBuf>,
}

/// Upstream `I` \[kg m^2\].
fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::new(1725.0, -5.0, -12.0, -5.0, 5525.0, 43.0, -12.0, 43.0, 4810.0)
}

/// Upstream's wheels: `Honeywell_HR16`, 100 N m s, at rest, with the spin
/// axes `Mi(-az, 3) Mi(el, 2) [1, 0, 0]`.
fn wheels() -> RwFactory {
    let mut factory = RwFactory::default();
    let positions = [
        [0.8, 0.8, 1.8],
        [0.8, -0.8, 1.8],
        [-0.8, -0.8, 1.8],
        [-0.8, 0.8, 1.8],
    ];
    for (azimuth_deg, position) in [45.0, 135.0, 225.0, 315.0].into_iter().zip(positions) {
        let axis = euler3(-azimuth_deg * D2R) * euler2(40.0 * D2R) * Vector3::x();
        factory.create_honeywell_hr16(
            axis,
            100.0,
            0.0,
            Vector3::from(position),
            ReactionWheelModel::BalancedWheels,
        );
    }
    factory
}

/// Upstream `RSAList[item]`: a rotating solar array.
fn solar_array(name: &str, index: usize) -> SpinningBodyOneDofStateEffector {
    let side = if index == 0 { 1.0 } else { -1.0 };
    let mut config = SpinningBodyOneDofConfig::new(name);
    config.spinner_origin_body_m = Vector3::new(0.75 * side, 0.0, 0.45);
    config.center_of_mass_spinner_m = Vector3::new(0.0, 3.75, 0.0);
    config.spin_axis_spinner = Vector3::y();
    config.body_to_spinner0_dcm = Matrix3::new(0.0, 0.0, side, side, 0.0, 0.0, 0.0, 1.0, 0.0);
    config.inertia_about_com_spinner_kg_m2 =
        Matrix3::from_diagonal(&Vector3::new(250.0, 250.0, 500.0));
    config.mass_kg = 85.0;
    SpinningBodyOneDofStateEffector::new(config)
}

/// Upstream `platform`: the two-axis thruster gimbal.
fn platform() -> SpinningBodyTwoDofStateEffector {
    let mut config = SpinningBodyTwoDofConfig::new("platform1");
    config.body1.mass_kg = 0.0;
    config.body2.mass_kg = 10.0;
    config.body1.spin_axis_spinner = Vector3::x();
    config.body2.spin_axis_spinner = Vector3::y();
    let inertia = Matrix3::from_diagonal(&Vector3::new(2.0, 3.0, 4.0));
    config.body1.inertia_about_com_spinner_kg_m2 = inertia;
    config.body2.inertia_about_com_spinner_kg_m2 = inertia;
    SpinningBodyTwoDofStateEffector::new(config)
}

/// Upstream's ten SRP facets: the hub box and the two arrays (front and
/// back), the last four articulated with the arrays.
fn srp_model() -> FacetSRPDynamicEffector {
    let (len_x, len_y, len_z) = (1.50, 1.8, 2.86);
    let array_diameter = 7.262;
    let array_area = PI * (0.5 * array_diameter) * (0.5 * array_diameter);
    let areas = [
        len_y * len_z,
        len_x * len_z,
        len_y * len_z,
        len_x * len_z,
        len_x * len_y,
        len_x * len_y,
        array_area,
        array_area,
        array_area,
        array_area,
    ];
    let z = Vector3::z();
    let x = Vector3::x();
    let rotations = [
        -90.0 * z,
        0.0 * z,
        90.0 * z,
        180.0 * z,
        90.0 * x,
        -90.0 * x,
        0.0 * z,
        180.0 * z,
        0.0 * z,
        180.0 * z,
    ];
    let articulation_axes = [
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
        Vector3::zeros(),
        x,
        -x,
        -x,
        x,
    ];
    let locations = [
        [0.5 * len_x, 0.0, 0.5 * len_z],
        [0.0, 0.5 * len_y, 0.5 * len_z],
        [-0.5 * len_x, 0.0, 0.5 * len_z],
        [0.0, -0.5 * len_y, 0.5 * len_z],
        [0.0, 0.0, len_z],
        [0.0, 0.0, 0.0],
        [3.75 + 0.5 * len_x, 0.0, 0.45],
        [3.75 + 0.5 * len_x, 0.0, 0.45],
        [-(3.75 + 0.5 * len_x), 0.0, 0.45],
        [-(3.75 + 0.5 * len_x), 0.0, 0.45],
    ];
    let specular = [
        0.336, 0.336, 0.336, 0.336, 0.336, 0.336, 0.16, 0.00, 0.16, 0.00,
    ];
    let diffuse = [
        0.139, 0.139, 0.139, 0.139, 0.139, 0.139, 0.16, 0.56, 0.16, 0.56,
    ];
    let mut srp = FacetSRPDynamicEffector::new(FacetSRPDynamicEffectorConfig {
        name: "FacetSRP".to_string(),
    });
    for index in 0..areas.len() {
        srp.add_facet(
            areas[index],
            prv_to_dcm(rotations[index] * D2R),
            Vector3::y(),
            articulation_axes[index],
            Vector3::from(locations[index]),
            diffuse[index],
            specular[index],
        );
    }
    srp
}

fn run(config: Config) -> Results {
    // Upstream `timeInitString = "2023 OCTOBER 22 00:00:00.0"`.
    let epoch = Epoch::from_gregorian_utc_at_midnight(2023, 10, 22);
    let mu = MU_SUN * 1e9;
    let elements = ClassicElements {
        semi_major_axis: 100e9,
        eccentricity: 0.001,
        true_anomaly_rad: -110.0 * D2R,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 2500.0,
        hub_center_of_mass_body_m: Vector3::new(0.008, -0.010, 1.214),
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
            GravBodyData::point_mass("sun", mu, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid point-mass Sun"),
        )
        .expect("unique central gravity body");
    // `zeroBase = 'Sun'`: the Sun at the origin.
    let sun_state = Output::new(PlanetStateMsg::default());
    let sun_ephemeris = Output::new(SunEphemerisMsg::default());

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

    let mut arrays = [solar_array("solarArray1", 0), solar_array("solarArray2", 1)];
    let mut gimbal = platform();
    let swirl_torque_nm = if config.swirl_torque {
        1.0e-3 * MAX_THRUST_N
    } else {
        0.0
    };
    // Upstream `THRConfig`: the thruster in the platform frame F.
    let thruster_config_f = Output::new(ThrusterConfigMsg {
        position_thrust_b_m: Vector3::zeros(),
        t_hat_thrust_b: Vector3::z(),
        max_thrust_n: MAX_THRUST_N,
        swirl_torque_nm,
    });
    let mut sep_thruster = ThrusterStateEffector::new("sepThruster");
    let mut thruster_config = ThrusterStateEffectorConfig::new(
        "TH1",
        Vector3::zeros(),
        Vector3::z(),
        MAX_THRUST_N,
        1600.0,
    );
    thruster_config.min_on_time_s = 0.006;
    thruster_config.cutoff_frequency_radps = 5.0;
    thruster_config.max_swirl_torque_nm = swirl_torque_nm;
    let thruster_on_time = Output::new(ThrusterCommandMsg {
        on_time_s: 3600.0 * 24.0 * 7.0,
        ..ThrusterCommandMsg::default()
    });
    let mut srp = srp_model();

    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut mass_props = SimpleMassProps::new("simpleMassProps");
    let mut hub_boresight = BoreAngCalc::new(BoreAngCalcConfig {
        name: "hubBoresight".to_string(),
        bore_vec_body: -Vector3::y(),
        ..BoreAngCalcConfig::default()
    });
    let mut array_boresights: Vec<BoreAngCalc> = (0..2)
        .map(|index| {
            BoreAngCalc::new(BoreAngCalcConfig {
                name: format!("saBoresight{index}"),
                bore_vec_body: Vector3::z(),
                ..BoreAngCalcConfig::default()
            })
        })
        .collect();

    let mut platform_state = ThrusterPlatformState::new(ThrusterPlatformStateConfig {
        name: "thrusterPlatformState".to_string(),
        ..ThrusterPlatformStateConfig::default()
    });
    let mut cm_estimator = ThrustCmEstimation::new(ThrustCmEstimationConfig {
        name: "cmEstimator".to_string(),
        attitude_tol: 1e-6,
        r_cb_b_m: Vector3::from(INITIAL_CM_GUESS_M),
        p0_m2: Vector3::repeat(0.0025),
        r0_n2m2: Vector3::repeat(1e-10),
    });
    // Upstream `vcMsg_CoM`, which the estimator writes into.
    let estimated_cm_msg = Output::new(VehicleConfigMsg {
        center_of_mass_body_m: Vector3::from(INITIAL_CM_GUESS_M),
        ..VehicleConfigMsg::default()
    });
    cm_estimator.veh_config_out_msg = estimated_cm_msg.clone();
    let inertia_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        ..VehicleConfigMsg::default()
    });
    let mut platform_reference = ThrusterPlatformReference::new(ThrusterPlatformReferenceConfig {
        name: "thrusterPlatformReference".to_string(),
        theta1_max_rad: PI / 12.0,
        theta2_max_rad: PI / 12.0,
        k: if config.thr_mom_management {
            2.5e-4
        } else {
            0.0
        },
        ki: 0.0,
        ..ThrusterPlatformReferenceConfig::default()
    });
    let mut platform_controllers: Vec<HingedRigidBodyPidMotor> = (0..2)
        .map(|index| {
            HingedRigidBodyPidMotor::new(HingedRigidBodyPidMotorConfig {
                name: format!("PltMototorGimbal{}", index + 1),
                k: 0.5,
                p: 3.0,
                i: 0.0,
            })
        })
        .collect();
    let mut torque_scheduler = TorqueScheduler::new(TorqueSchedulerConfig {
        name: "TorqueScheduler".to_string(),
        lock_flag: LockFlag::ApplyBoth,
        t_switch_s: 60.0,
    });
    let mut sep_point = OneAxisSolarArrayPoint::new(OneAxisSolarArrayPointConfig {
        name: "sepPointGuidance".to_string(),
        a1_hat_b: Vector3::x(),
        a2_hat_b: Vector3::y(),
        h_hat_n: Vector3::x(),
        ..OneAxisSolarArrayPointConfig::default()
    });
    let array_reference = |index: usize| {
        let side = if index == 0 { 1.0 } else { -1.0 };
        SolarArrayReference::new(SolarArrayReferenceConfig {
            name: format!("SolarArrayReference{}", index + 1),
            a1_hat_b: Vector3::new(side, 0.0, 0.0),
            a2_hat_b: Vector3::y(),
            r_ab_b_m: Vector3::new(side * 0.5 * (1.50 + 7.262), 0.0, 0.45),
            pointing_mode: PointingMode::PowerGeneration,
            n: 2.0,
            sigma: 1e-3,
            theta_max_rad: PI / 3.0,
            ..SolarArrayReferenceConfig::default()
        })
    };
    let mut array_references: Vec<Staged<SolarArrayReference>> = (0..2)
        .map(|index| {
            let reference = Staged::new(array_reference(index));
            if config.sa_mom_management {
                reference.after(DURATION_NANOS / 2, |reference, context| {
                    reference.config.pointing_mode = PointingMode::MomentumManagement;
                    basilisk_rs::Module::reset(reference, context);
                })
            } else {
                reference
            }
        })
        .collect();
    let mut array_controllers: Vec<HingedRigidBodyPidMotor> = (0..2)
        .map(|index| {
            HingedRigidBodyPidMotor::new(HingedRigidBodyPidMotorConfig {
                name: format!("SolarArrayMotor{}", index + 1),
                k: 1.25,
                p: 50.0,
                i: 3e-3,
            })
        })
        .collect();
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: 9.0,
        ki: 1e-5,
        p: 275.0,
        integral_limit: 2.0 / 1e-5 * 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 1,
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
    let mut wheel_speed_log = MessageLog::<RwSpeedMsg>::default();
    let mut true_cm_log = MessageLog::<VehicleConfigMsg>::default();
    let mut cm_estimate_log = MessageLog::<CmEstimationDataMsg>::default();
    let mut array_angle_logs: Vec<MessageLog<HingedRigidBodyMsg>> =
        (0..2).map(|_| MessageLog::default()).collect();
    let mut array_reference_logs: Vec<MessageLog<HingedRigidBodyMsg>> =
        (0..2).map(|_| MessageLog::default()).collect();
    let mut array_boresight_logs: Vec<MessageLog<BoreAngleMsg>> =
        (0..2).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let vehicle_cm_msg = if config.cm_estimation {
        estimated_cm_msg.clone()
    } else {
        mass_props.vehicle_config_out_msg.clone()
    };
    let array_outputs: Vec<Output<HingedRigidBodyMsg>> = arrays
        .iter()
        .map(|array| array.spinning_body_out.clone())
        .collect();
    let array_states: Vec<_> = arrays
        .iter()
        .map(|array| array.spinning_body_state_out.clone())
        .collect();
    let platform_outputs = gimbal.spinning_body_out.clone();

    let mut simulation = Simulation::new(epoch, false);
    for (command, wheel) in torque_splitter
        .actuator_out_msgs
        .iter()
        .zip(reaction_wheels.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    {
        let thruster = sep_thruster.add_thruster(thruster_config);
        connect!(&simulation,
            &thruster_on_time => &mut thruster.command_in_msg,
            &gimbal.spinning_body_state_out[1] => &mut thruster.attached_body_in_msg,
        );
    }
    let thruster_output = sep_thruster.thrusters()[0].thruster_out_msg.clone();
    for (index, (array, controller)) in arrays.iter_mut().zip(&array_controllers).enumerate() {
        connect!(&simulation,
            &controller.motor_torque_out_msg => &mut array.motor_torque_in,
        );
        // Upstream `addArticulatedFacet`: facets 7-10 follow the arrays.
        for facet in [6 + 2 * index, 7 + 2 * index] {
            connect!(&simulation,
                &array.spinning_body_out => &mut srp.facets[facet].articulation_angle_in,
            );
        }
    }
    connect!(&simulation,
        &torque_scheduler.motor_torque_out_msg => &mut gimbal.motor_torque_in,
        &torque_scheduler.effector_lock_out_msg => &mut gimbal.motor_lock_in,
        &sun_ephemeris => &mut srp.input_sun_msg,
    );
    spacecraft.add_state_effector(reaction_wheels);
    let [array1, array2] = arrays;
    spacecraft.add_state_effector(array1);
    spacecraft.add_state_effector(array2);
    spacecraft.add_state_effector(gimbal);
    spacecraft.add_state_effector(sep_thruster);
    spacecraft.add_dynamic_effector(srp);

    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &sun_state => &mut navigation.sun_state_in_msg,
        &spacecraft.mass_props_out => &mut mass_props.sc_mass_props_in_msg,
        &thruster_config_f => &mut platform_state.thruster_config_f_in_msg,
        &platform_outputs[0] => &mut platform_state.hinged_rigid_body1_in_msg,
        &platform_outputs[1] => &mut platform_state.hinged_rigid_body2_in_msg,
        &platform_state.thruster_config_b_out_msg => &mut cm_estimator.thruster_config_b_in_msg,
        &controller.int_feedback_torque_out_msg => &mut cm_estimator.int_feedback_torque_in_msg,
        &tracking_error.att_guid_out_msg => &mut cm_estimator.att_guid_in_msg,
        &mass_props.vehicle_config_out_msg => &mut cm_estimator.veh_config_in_msg,
        &vehicle_cm_msg => &mut platform_reference.veh_config_in_msg,
        &thruster_config_f => &mut platform_reference.thruster_config_f_in_msg,
        &rw_config_msg => &mut platform_reference.rw_config_data_in_msg,
        &wheel_speed_msg => &mut platform_reference.rw_speeds_in_msg,
        &platform_controllers[0].motor_torque_out_msg => &mut torque_scheduler.motor_torque1_in_msg,
        &platform_controllers[1].motor_torque_out_msg => &mut torque_scheduler.motor_torque2_in_msg,
        &navigation.attitude_out_msg => &mut sep_point.att_nav_in_msg,
        &platform_reference.body_heading_out_msg => &mut sep_point.body_heading_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &sep_point.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &inertia_msg => &mut controller.veh_config_in_msg,
        &rw_config_msg => &mut controller.rw_params_in_msg,
        &wheel_speed_msg => &mut controller.rw_speeds_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &controller.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut torque_splitter.torque_in_msg,
        &spacecraft.state_out => &mut hub_boresight.sc_state_in_msg,
        &sun_state => &mut hub_boresight.cel_body_in_msg,
        &platform_outputs[0] => &mut platform_controllers[0].hinged_rigid_body_in_msg,
        &platform_outputs[1] => &mut platform_controllers[1].hinged_rigid_body_in_msg,
        &platform_reference.hinged_rigid_body_ref1_out_msg => &mut platform_controllers[0].hinged_rigid_body_ref_in_msg,
        &platform_reference.hinged_rigid_body_ref2_out_msg => &mut platform_controllers[1].hinged_rigid_body_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &wheel_speed_msg => &mut wheel_speed_log.input,
        &mass_props.vehicle_config_out_msg => &mut true_cm_log.input,
        &cm_estimator.cm_est_data_out_msg => &mut cm_estimate_log.input,
    );
    for index in 0..2 {
        let reference = &mut array_references[index].module;
        connect!(&simulation,
            &navigation.attitude_out_msg => &mut reference.att_nav_in_msg,
            &sep_point.att_ref_out_msg => &mut reference.att_ref_in_msg,
            &array_outputs[index] => &mut reference.hinged_rigid_body_in_msg,
            &wheel_speed_msg => &mut reference.rw_speeds_in_msg,
            &rw_config_msg => &mut reference.rw_config_data_in_msg,
            &vehicle_cm_msg => &mut reference.veh_config_in_msg,
            &array_outputs[index] => &mut array_controllers[index].hinged_rigid_body_in_msg,
            &reference.hinged_rigid_body_ref_out_msg => &mut array_controllers[index].hinged_rigid_body_ref_in_msg,
            &array_states[index] => &mut array_boresights[index].sc_state_in_msg,
            &sun_state => &mut array_boresights[index].cel_body_in_msg,
            &array_outputs[index] => &mut array_angle_logs[index].input,
            &reference.hinged_rigid_body_ref_out_msg => &mut array_reference_logs[index].input,
            &array_boresights[index].ang_out_msg => &mut array_boresight_logs[index].input,
            &array_outputs[index] => recorder.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([("theta_rad".to_string(), format!("saAngle_rad_{index}"))]),
            ),
            &reference.hinged_rigid_body_ref_out_msg => recorder.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([("theta_rad".to_string(), format!("saRefAngle_rad_{index}"))]),
            ),
            &array_boresights[index].ang_out_msg => recorder.add_source::<BoreAngleMsg>(
                CsvSourceConfig::columns([("miss_angle_rad".to_string(), format!("saMissAngle_rad_{index}"))]),
            ),
            &platform_outputs[index] => recorder.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([("theta_rad".to_string(), format!("pltAngle_rad_{index}"))]),
            ),
        );
    }
    connect!(&simulation,
        &platform_reference.hinged_rigid_body_ref1_out_msg => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([("theta_rad", "pltRefAngle_rad_0")]),
        ),
        &platform_reference.hinged_rigid_body_ref2_out_msg => recorder.add_source::<HingedRigidBodyMsg>(
            CsvSourceConfig::columns([("theta_rad", "pltRefAngle_rad_1")]),
        ),
        &navigation.attitude_out_msg => recorder.add_source::<NavigationAttitudeMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_bn", "sigma_BN")),
        ),
        &sep_point.att_ref_out_msg => recorder.add_source::<AttitudeReferenceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_rn", "sigma_RN")),
        ),
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
        &wheel_speed_msg => recorder.add_source::<RwSpeedMsg>(CsvSourceConfig::columns(
            array_columns("wheel_speeds_radps", "wheelSpeeds_rad_per_s", WHEEL_COUNT),
        )),
        &mass_props.vehicle_config_out_msg => recorder.add_source::<VehicleConfigMsg>(
            CsvSourceConfig::columns(vector_columns("center_of_mass_body_m", "CoM_B_m")),
        ),
        &cm_estimator.cm_est_data_out_msg => recorder.add_source::<CmEstimationDataMsg>(
            CsvSourceConfig::columns(
                vector_columns("state_error_m", "stateError_m")
                    .into_iter()
                    .chain(vector_columns("covariance_m", "covariance_m"))
                    .chain(vector_columns("pre_fit_res_nm", "preFitRes_Nm"))
                    .chain(vector_columns("post_fit_res_nm", "postFitRes_Nm")),
            ),
        ),
        &hub_boresight.ang_out_msg => recorder.add_source::<BoreAngleMsg>(
            CsvSourceConfig::columns([("miss_angle_rad", "hubMissAngle_rad")]),
        ),
        &thruster_output => recorder.add_source::<ThrusterOutputMsg>(CsvSourceConfig::columns(
            vector_columns("thruster_location_m", "thrLoc_F_m")
                .into_iter()
                .chain(vector_columns("thruster_direction", "thrDir_F"))
                .chain([("thrust_force_n".to_string(), "thrustForce_N".to_string())]),
        )),
    );
    // Upstream's tasks: dynamics (0.5 s), then the platform reference (1 h),
    // then the flight software (2 s) at shared ticks; within each, upstream's
    // module priorities.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=300;
        "navigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=290;
        "mass_props" => &mut mass_props, period=DYNAMICS_PERIOD_NANOS, priority=280;
        "hub_boresight" => &mut hub_boresight, period=DYNAMICS_PERIOD_NANOS, priority=270;
        "platform_reference" => &mut platform_reference, period=PLATFORM_REFERENCE_PERIOD_NANOS, priority=200;
        "platform_state" => &mut platform_state, period=FSW_PERIOD_NANOS, priority=130;
        "cm_estimator" => &mut cm_estimator, period=FSW_PERIOD_NANOS, priority=129;
        "torque_scheduler" => &mut torque_scheduler, period=FSW_PERIOD_NANOS, priority=126;
        "sep_point" => &mut sep_point, period=FSW_PERIOD_NANOS, priority=125;
        "tracking_error" => &mut tracking_error, period=FSW_PERIOD_NANOS, priority=122;
        "mrp_feedback" => &mut controller, period=FSW_PERIOD_NANOS, priority=121;
        "rw_motor_torque" => &mut rw_motor_torque, period=FSW_PERIOD_NANOS, priority=120;
        "rw_motor_torque_splitter" => &mut torque_splitter, period=FSW_PERIOD_NANOS, priority=119;
        "guidance_log" => &mut guidance_log, period=FSW_PERIOD_NANOS, priority=0;
        "wheel_speed_log" => &mut wheel_speed_log, period=FSW_PERIOD_NANOS, priority=0;
        "true_cm_log" => &mut true_cm_log, period=FSW_PERIOD_NANOS, priority=0;
        "cm_estimate_log" => &mut cm_estimate_log, period=FSW_PERIOD_NANOS, priority=0;
    }
    for boresight in &mut array_boresights {
        simulation.add_module("sa_boresight", boresight, DYNAMICS_PERIOD_NANOS, 260);
    }
    for controller in &mut platform_controllers {
        simulation.add_module("platform_controller", controller, FSW_PERIOD_NANOS, 127);
    }
    for reference in &mut array_references {
        simulation.add_module("sa_reference", reference, FSW_PERIOD_NANOS, 124);
    }
    for controller in &mut array_controllers {
        simulation.add_module("sa_controller", controller, FSW_PERIOD_NANOS, 123);
    }
    for log in array_angle_logs.iter_mut().chain(&mut array_reference_logs) {
        simulation.add_module("array_log", log, FSW_PERIOD_NANOS, 0);
    }
    for log in &mut array_boresight_logs {
        simulation.add_module("array_boresight_log", log, FSW_PERIOD_NANOS, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, FSW_PERIOD_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        wheel_speeds: wheel_speed_log,
        true_cm: true_cm_log,
        cm_estimate: cm_estimate_log,
        array_angles: array_angle_logs,
        array_references: array_reference_logs,
        array_boresights: array_boresight_logs,
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
    let mut cli = Cli::from_env(
        "scenario_sep_momentum_management [--[no-]swirl-torque] [--[no-]thr-mom-management] \
         [--[no-]sa-mom-management] [--[no-]cm-estimation] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("swirl-torque", false),
            cli.flag("thr-mom-management", true),
            cli.flag("sa-mom-management", false),
            cli.flag("cm-estimation", true),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let momentum = wheel_momentum(results.wheel_speeds.last(), &results.wheel_axes);
        println!(
            "wrote {} (final wheel momentum {:.3} N m s, attitude error {:.2e})",
            results.csv_path.expect("CSV written").display(),
            momentum.norm(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioSepMomentumManagement.py` only checks that each
    //! parametrization runs. These tests run both and check what they
    //! demonstrate.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[test]
    fn the_spacecraft_tracks_its_sep_attitude_and_the_arrays_track_the_sun() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            assert_eq!(results.guidance.values.len(), 10801, "{config:?}");
            // After the initial slew (30 min), the attitude error stays small;
            // each hourly platform-reference update starts a small new slew.
            for guidance in results.guidance.since(1_800_000_000_000) {
                assert!(guidance.sigma_br.norm() < 1.0e-2, "{config:?}");
            }
            for (angles, references) in results.array_angles.iter().zip(&results.array_references) {
                let (angle, reference) = (angles.last().theta_rad, references.last().theta_rad);
                assert!(
                    (angle - reference).abs() < 1.0e-3,
                    "{config:?}: {angle} vs {reference}"
                );
            }
            for boresight in &results.array_boresights {
                assert!(
                    boresight.last().miss_angle_rad.to_degrees() < 5.0,
                    "{config:?}"
                );
            }
        }
    }

    /// With the center-of-mass estimate in the loop and the thruster used
    /// for momentum management, the estimate converges on the true center of
    /// mass across the thrust axis (along it, an offset makes no torque and
    /// is unobservable), and the wheels end with less momentum than without
    /// them.
    #[test]
    fn momentum_management_and_cm_estimation_do_their_job() {
        let plain = quiet(TEST_PARAMETRIZATIONS[0]);
        let managed = quiet(TEST_PARAMETRIZATIONS[1]);
        let true_cm = managed.true_cm.last().center_of_mass_body_m;
        let initial_error = Vector3::from(INITIAL_CM_GUESS_M) - true_cm;
        let final_error = managed.cm_estimate.last().state_error_m;
        let across = |error: Vector3<f64>| error.xy().norm();
        assert!(
            across(final_error) < 0.5 * across(initial_error),
            "{final_error} vs {initial_error}"
        );

        let momentum = |results: &Results| {
            wheel_momentum(results.wheel_speeds.last(), &results.wheel_axes).norm()
        };
        assert!(
            momentum(&managed) < momentum(&plain),
            "{} vs {}",
            momentum(&managed),
            momentum(&plain)
        );
    }
}
