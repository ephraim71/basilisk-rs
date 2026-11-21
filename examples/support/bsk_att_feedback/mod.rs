//! The attitude-feedback simulation of upstream BskSim's
//! `scenarios/scenario_AttFeedback.py` (`BSK_Dynamics` and `BSK_Fsw` in the
//! `inertial3D` mode), which upstream's `MonteCarloExamples` disperse and
//! rerun.
//!
//! **Marked for consolidation with `examples/support/bsk_sim`**, upstream's
//! BskSim framework: the Monte Carlo examples should build this run from
//! `bsk_sim::BskModels` in the mode `inertial3D` (with the dispersed hub mass,
//! center of mass and inertia set on `dynamics.spacecraft` and the hub state
//! converted with `multibody::set_center_of_mass_initial_state`), and this
//! module go. That run is not bit-identical to this one: the translation
//! matches exactly, but `sigma_BR` differs in the last digit from the second
//! sample on (the first, dispersed), as BskSim's `NoisySimpleNav` composes
//! its zero attitude noise through an MRP addition where `SimpleNavigation`
//! passes the attitude through. The switch therefore changes the archived
//! campaign and its CSV files (in the last digits), so it waits for the
//! owner's decision. Until then this module is built from the existing
//! helpers and holds only the parts of BskSim that drive this mode.
//!
//! # The simulation
//!
//! * Dynamics (0.1 s): the `bskSat` hub (750 kg, `diag(900, 800, 600)` kg m²
//!   about its center of mass at `r_BcB_B`) on `a = 10 000 km`, `e = 0.01`,
//!   `i = 33.3°`, `Ω = 48.2°`, `ω = 347.8°`, `f = 85.3°`, from
//!   `sigma_BN = [0.1, 0.2, -0.3]` and `omega_BN_B = [0.001, -0.01, 0.03]`
//!   rad/s (the orbit of the center of mass, upstream's `r_CN_NInit`);
//!   point-mass Sun, Earth (central) and Moon from ANISE ephemerides
//!   relative to the Earth (upstream's SPICE with `zeroBase = 'Earth'`) from
//!   `2012 MAY 1 00:28:30.0`; the four `Honeywell_HR16` wheels of the BskSim
//!   pyramid; `simpleNav`.
//! * Flight software (0.1 s): `inertial3D` (`sigma_R0N = [0.2, 0.4, 0.6]`),
//!   `attTrackingError`, `mrpFeedback` on the wheels (`K = 3.5`, `P = 30`,
//!   no integral feedback) with the FSW's vehicle configuration (the
//!   nominal inertia), and `rwMotorTorque` on the three body axes.
//!   Upstream's mode event enables these tasks at the first check, before
//!   tick 0, so they run throughout.
//! * 10 minutes, the guidance and navigation recorded at the FSW rate in
//!   the dynamics task (upstream's `msgRecList`).
//!
//! Of `BSK_Dynamics`, the parts that cannot change this run are left out:
//! the external torque and the eight thrusters (their commands stay zero in
//! this mode), and the eclipse, the coarse Sun sensors, the WMM field and
//! the magnetometer (no module of this mode reads them). As in the other
//! basilisk-rs ports, the ephemerides run before the spacecraft, and the
//! wheels' array command is split per wheel.

#![allow(dead_code)]

#[path = "../common.rs"]
pub mod common;
#[path = "../grav_factory.rs"]
pub mod grav_factory;
#[path = "../message_log.rs"]
pub mod message_log;
#[path = "../multibody.rs"]
pub mod multibody;
#[path = "../rw_factory.rs"]
pub mod rw_factory;
#[path = "../staged.rs"]
pub mod staged;
#[path = "../table_csv.rs"]
pub mod table_csv;

use std::collections::BTreeMap;
use std::path::PathBuf;

use basilisk_rs::device_interface::array_motor_torque_to_single_actuators::{
    ArrayMotorTorqueToSingleActuators, ArrayMotorTorqueToSingleActuatorsConfig,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector;
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::effector_interfaces::rw_motor_torque::{
    RwMotorTorque, RwMotorTorqueConfig,
};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, NavigationTranslationMsg, Output, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use grav_factory::Body;
use message_log::MessageLog;
use staged::UpdatedAtInit;

/// Upstream BskSim `fswRate` and `dynRate` \[ns\].
pub const STEP_NANOS: u64 = 100_000_000;
/// Upstream `runScenario`'s `simulationTime` (10 minutes) \[ns\].
pub const DURATION_NANOS: u64 = 600_000_000_000;
/// Upstream `sNavTransName` and `attGuidName`, the recorder names.
pub const NAV_TRANS_NAME: &str = "sNavTransMsg";
pub const ATT_GUID_NAME: &str = "attGuidMsg";

/// Upstream `timeInitString` of `BSK_Dynamics`: "2012 MAY 1 00:28:30.0".
pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0)
}

/// Upstream `I_sc` \[kg m²\].
pub fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0))
}

/// The parameters upstream's Monte Carlo examples disperse, by their
/// upstream paths (`TaskList[0].TaskModels[0]` is the hub).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct AttFeedbackConfig {
    /// `hub.sigma_BNInit`.
    pub sigma_bn_init: Vector3<f64>,
    /// `hub.omega_BN_BInit` \[rad/s\].
    pub omega_bn_b_init_radps: Vector3<f64>,
    /// `hub.mHub` \[kg\].
    pub hub_mass_kg: f64,
    /// `hub.r_BcB_B` \[m\].
    pub hub_com_offset_m: Vector3<f64>,
    /// `hubref.IHubPntBc_B` \[kg m²\] (upstream defines its dispersion but
    /// never adds it).
    pub hub_inertia_kg_m2: Matrix3<f64>,
}

impl AttFeedbackConfig {
    /// Upstream `scenario_AttFeedback`'s values.
    pub fn nominal() -> Self {
        Self {
            sigma_bn_init: Vector3::new(0.1, 0.2, -0.3),
            omega_bn_b_init_radps: Vector3::new(0.001, -0.01, 0.03),
            hub_mass_kg: 750.0,
            hub_com_offset_m: Vector3::zeros(),
            hub_inertia_kg_m2: hub_inertia_kg_m2(),
        }
    }
}

/// Upstream `msgRecList`: the guidance and navigation histories.
#[derive(Clone, Debug, Default)]
pub struct AttFeedbackHistory {
    pub times_nanos: Vec<u64>,
    /// `attGuidMsg.sigma_BR`.
    pub sigma_br: Vec<Vector3<f64>>,
    /// `attGuidMsg.omega_BR_B` \[rad/s\].
    pub omega_br_b_radps: Vec<Vector3<f64>>,
    /// `sNavTransMsg.r_BN_N` \[m\].
    pub r_bn_n_m: Vec<Vector3<f64>>,
}

/// Upstream `scenario_AttFeedback(...)` followed by `runScenario`: one run
/// of the simulation from `config`.
pub fn simulate(config: &AttFeedbackConfig) -> AttFeedbackHistory {
    // `configure_initial_conditions`.
    let degree = std::f64::consts::PI / 180.0;
    let elements = ClassicElements {
        semi_major_axis: 10_000_000.0,
        eccentricity: 0.01,
        inclination_rad: 33.3 * degree,
        raan_rad: 48.2 * degree,
        arg_periapsis_rad: 347.8 * degree,
        true_anomaly_rad: 85.3 * degree,
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(Body::Earth.mu_m3ps2(), &elements);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: config.hub_mass_kg,
        hub_center_of_mass_body_m: config.hub_com_offset_m,
        inertia_kg_m2: config.hub_inertia_kg_m2,
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: config.sigma_bn_init,
        initial_omega_radps: config.omega_bn_b_init_radps,
        integrator: None,
    });
    // `SetGravityBodies`: point masses, SPICE relative to the Earth.
    let mut ephemerides: Vec<UpdatedAtInit<_>> = [Body::Sun, Body::Earth, Body::Moon]
        .into_iter()
        .map(|body| {
            spacecraft
                .add_grav_body(body.point_mass(body == Body::Earth))
                .expect("one central body");
            let ephemeris = UpdatedAtInit::new(body.ephemeris(Body::Earth), epoch());
            ephemeris.module.output_planet_msg.connect_to(
                spacecraft
                    .grav_body_input_mut(body.name())
                    .expect("the body was just added"),
            );
            ephemeris
        })
        .collect();

    // `SetReactionWheelDynEffector`, and the FSW's `SetRWConfigMsg`
    // (`Js = 50 N m s / 6000 rpm`, 0.2 N m, as the wheels).
    let rw_factory = rw_factory::hr16_pyramid(50.0);
    let rw_config_msg = Output::new(rw_factory.config_message());
    let mut wheels = ReactionWheelStateEffector::new("RWA");
    for wheel in &rw_factory.wheels {
        wheels.add_reaction_wheel(wheel.clone());
    }
    let mut rw_motor_cmd =
        ArrayMotorTorqueToSingleActuators::new(ArrayMotorTorqueToSingleActuatorsConfig {
            name: "rwMotorCmd".to_string(),
            num_actuators: rw_factory.wheels.len(),
        });
    for (command, wheel) in rw_motor_cmd
        .actuator_out_msgs
        .iter()
        .zip(wheels.wheels_mut())
    {
        command.connect_to(&mut wheel.motor_torque_in_msg);
    }
    let wheel_speed_msg = wheels.rw_speed_out_msg.clone();
    spacecraft.add_state_effector(wheels);
    // Upstream's `r_CN_NInit` and `v_CN_NInit` are the center of mass's.
    multibody::set_center_of_mass_initial_state(&mut spacecraft, position_m, velocity_mps);
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    // `BSK_Fsw`: `SetVehicleConfiguration` (the nominal inertia), the
    // `inertial3D` and `mrpFeedbackRWs` tasks.
    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: hub_inertia_kg_m2(),
        ..Default::default()
    });
    let mut inertial_3d = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::new(0.2, 0.4, 0.6),
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "trackingError".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let ki = -1.0;
    let mut mrp_feedback = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedbackRWs".to_string(),
        k: 3.5,
        ki,
        p: 30.0,
        integral_limit: 2.0 / ki * 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });

    // `log_outputs`: at the FSW rate, in the dynamics task.
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();

    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &inertial_3d.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &tracking_error.att_guid_out_msg => &mut mrp_feedback.guid_in_msg,
            &vehicle_config_msg => &mut mrp_feedback.veh_config_in_msg,
            &rw_config_msg => &mut mrp_feedback.rw_params_in_msg,
            &wheel_speed_msg => &mut mrp_feedback.rw_speeds_in_msg,
            &mrp_feedback.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
            &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
            &rw_motor_torque.rw_motor_torque_out_msg => &mut rw_motor_cmd.torque_in_msg,
            &tracking_error.att_guid_out_msg => &mut guidance_log.input,
            &navigation.translation_out_msg => &mut translation_log.input,
        );
        // The dynamics process (the ephemerides, the wheels' command, the
        // spacecraft, the navigation, then the recorders of its task), then
        // the FSW process by task and module priority.
        for ephemeris in &mut ephemerides {
            simulation.add_module("spice", ephemeris, STEP_NANOS, 400);
        }
        schedule! { simulation,
            "rwMotorCmd" => &mut rw_motor_cmd, period=STEP_NANOS, priority=301;
            "bskSat" => &mut spacecraft, period=STEP_NANOS, priority=201;
            "SimpleNavigation" => &mut navigation, period=STEP_NANOS, priority=109;
            "attGuidMsg" => &mut guidance_log, period=STEP_NANOS, priority=100;
            "sNavTransMsg" => &mut translation_log, period=STEP_NANOS, priority=100;
            "inertial3D" => &mut inertial_3d, period=STEP_NANOS, priority=50;
            "trackingError" => &mut tracking_error, period=STEP_NANOS, priority=49;
            "mrpFeedbackRWs" => &mut mrp_feedback, period=STEP_NANOS, priority=48;
            "rwMotorTorque" => &mut rw_motor_torque, period=STEP_NANOS, priority=47;
        }
        simulation.run_for(DURATION_NANOS);
    }

    AttFeedbackHistory {
        times_nanos: guidance_log.times_nanos.clone(),
        sigma_br: guidance_log.values.iter().map(|msg| msg.sigma_br).collect(),
        omega_br_b_radps: guidance_log
            .values
            .iter()
            .map(|msg| msg.omega_br_b_radps)
            .collect(),
        r_bn_n_m: translation_log
            .values
            .iter()
            .map(|msg| msg.position_bn_n_m)
            .collect(),
    }
}

/// What a Monte Carlo run retains (upstream's `RetentionPolicy` message
/// logs): the times and, by `"<recorder>.<field>"` (upstream's
/// `data["messages"]` keys), the logged vectors, downcast to `f32` as
/// upstream's `setVarCast('float')`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct RetainedMessages {
    pub times_nanos: Vec<u64>,
    pub messages: BTreeMap<String, Vec<[f32; 3]>>,
}

impl RetainedMessages {
    /// Upstream `retentionPolicy.addMessageLog(name, [fields])` for the
    /// recorded `fields` of `history`.
    pub fn retain(history: &AttFeedbackHistory, fields: &[&str]) -> Self {
        let cast = |values: &[Vector3<f64>]| -> Vec<[f32; 3]> {
            values
                .iter()
                .map(|value| [value.x as f32, value.y as f32, value.z as f32])
                .collect()
        };
        let messages = fields
            .iter()
            .map(|field| {
                let values = match *field {
                    "attGuidMsg.sigma_BR" => cast(&history.sigma_br),
                    "attGuidMsg.omega_BR_B" => cast(&history.omega_br_b_radps),
                    "sNavTransMsg.r_BN_N" => cast(&history.r_bn_n_m),
                    _ => panic!("no recorded field {field}"),
                };
                (field.to_string(), values)
            })
            .collect();
        Self {
            times_nanos: history.times_nanos.clone(),
            messages,
        }
    }

    /// The history of `field` (upstream `data["messages"][field]` without
    /// its time column).
    pub fn field(&self, field: &str) -> &[[f32; 3]] {
        self.messages
            .get(field)
            .unwrap_or_else(|| panic!("{field} was not retained"))
    }
}

/// Upstream `scenarioBskSimAttFeedbackMC`'s name, its archive directory's.
pub const MC_NAME: &str = "scenarioBskSimAttFeedbackMC";

/// `examples/output/montecarlo/<name>`, the stand-in for upstream's
/// `examples/MonteCarloExamples/<name>`.
pub fn archive_dir(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/output/montecarlo")
        .join(name)
}

/// Upstream `scenarioBskSimAttFeedbackMC`'s `setExecutionCount(4)`.
pub const NUMBER_OF_RUNS: usize = 4;
/// Upstream `scenarioBskSimAttFeedbackMC`'s retention policy.
pub const RETAINED_FIELDS: [&str; 3] = [
    "sNavTransMsg.r_BN_N",
    "attGuidMsg.sigma_BR",
    "attGuidMsg.omega_BR_B",
];

/// Upstream `NormalVectorCartDispersion` of `hub.omega_BN_BInit`: σ =
/// 0.75°/3 per component \[rad/s\].
pub fn omega_std_radps() -> f64 {
    0.75 / 3.0 * std::f64::consts::PI / 180.0
}

/// Upstream `UniformDispersion` bounds of `hub.mHub`: 750 kg ± 5 %.
pub const HUB_MASS_BOUNDS_KG: (f64, f64) = (750.0 - 0.05 * 750.0, 750.0 + 0.05 * 750.0);

/// Upstream `NormalVectorCartDispersion` of `hub.r_BcB_B`: mean and σ \[m\].
pub fn com_offset_mean_and_std_m() -> (Vector3<f64>, Vector3<f64>) {
    (
        Vector3::new(0.0, 0.0, 1.0),
        Vector3::new(0.05 / 3.0, 0.05 / 3.0, 0.1 / 3.0),
    )
}

/// Upstream `scenarioBskSimAttFeedbackMC`'s controller: four runs, seeds
/// dispersed (no module of this simulation draws random numbers, so none is
/// registered), and its four dispersions.
pub fn campaign() -> basilisk_rs::monte_carlo::MonteCarlo<AttFeedbackConfig> {
    use basilisk_rs::monte_carlo::{
        MonteCarlo, NormalVectorCartDispersion, UniformDispersion, UniformEulerAngleMrpDispersion,
    };
    let (com_mean, com_std) = com_offset_mean_and_std_m();
    MonteCarlo::new(AttFeedbackConfig::nominal())
        .execution_count(NUMBER_OF_RUNS)
        .disperse_seeds(true)
        .disperse(
            |config: &mut AttFeedbackConfig| &mut config.sigma_bn_init,
            UniformEulerAngleMrpDispersion::default(),
        )
        .disperse(
            |config: &mut AttFeedbackConfig| &mut config.omega_bn_b_init_radps,
            NormalVectorCartDispersion::isotropic(0.0, omega_std_radps()),
        )
        .disperse(
            |config: &mut AttFeedbackConfig| &mut config.hub_mass_kg,
            UniformDispersion::new(HUB_MASS_BOUNDS_KG.0, HUB_MASS_BOUNDS_KG.1),
        )
        .disperse(
            |config: &mut AttFeedbackConfig| &mut config.hub_com_offset_m,
            NormalVectorCartDispersion::new(com_mean, com_std),
        )
}

/// The campaign's run function (upstream's `scenario_AttFeedback` creation
/// and `runScenario` execution functions), retaining `fields`.
pub fn run_retaining(config: &AttFeedbackConfig, fields: &[&str]) -> RetainedMessages {
    RetainedMessages::retain(&simulate(config), fields)
}
