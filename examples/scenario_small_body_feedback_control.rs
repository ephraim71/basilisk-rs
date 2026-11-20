//! Waypoint-to-waypoint motion about Bennu: port of upstream
//! `examples/scenarioSmallBodyFeedbackControl.py`.
//!
//! A 330 kg spacecraft with three HR16 wheels (100 N m s) starts at rest
//! relative to Bennu, `r_BO_N = [-2000, 1500, 1000] m` from it, tumbling at
//! `omega_BN_B = [0.1, 0.1, 0.1] rad/s`. Bennu (`mu = 4.892 m^3/s^2`) and the
//! Sun are point masses; Bennu follows its December 31st, 2018 heliocentric
//! orbit (`planetEphemeris`, pole at declination 90 deg, 4.297461 h spin) and
//! the Sun sits at the origin of the zeroed Sun messages upstream writes for
//! `radiationPressure` (cannonball, 1 m^2, `Cr = 1.9`) and the controller.
//!
//! `inertial3D` (`sigma_R0N = [0.1, 0, 0]`), `attTrackingError` and
//! `mrpFeedback` (`K = 7`, `P = 30`, no integral feedback) hold an inertial
//! attitude with the wheels (`rwMotorTorque` on all three body axes), while
//! `smallBodyWaypointFeedback` (`K1 = 5e-4 I`, `K2 = I`) drives the
//! spacecraft to the Hill-frame waypoint `x1_ref = [-2000, 0, 0] m`,
//! `x2_ref = 0`, through an `extForceTorque` body force. Both read noisy
//! `simpleNav` measurements (30 m, 1 cm/s, 0.1 deg, 0.05 deg/s) and the
//! controller a noisy `planetNav` Bennu ephemeris (2 deg, 0.3 deg/s attitude
//! and rate errors). One 1 s task, 15 000 s.
//!
//! Upstream `run(show_plots)` has no parameters. Writes
//! `examples/output/scenarios/scenarioSmallBodyFeedbackControl.csv` with the
//! spacecraft truth, Bennu's state, the measurements, the commanded force and
//! the attitude error at every step; `plot_scenario_small_body_feedback_control.py`
//! redraws upstream's five figures from it.
//!
//! # Differences from upstream
//!
//! * basilisk-rs's `SimpleNavigation` has no error model, so the noisy
//!   navigation is [`small_body_simple_nav::NoisySimpleNav`], upstream's
//!   18-state `simpleNav` errors on the crate's `GaussMarkov` (upstream's
//!   model, as `planetNav` here); the noise streams are not upstream's.
//! * The wheels take one command each, so `rwMotorTorque`'s array command
//!   goes through `arrayMotorTorqueToSingleActuators`.
//! * `radiationPressure` reads an eclipse message for its illumination,
//!   which upstream leaves unlinked (full sun); here a constant message with
//!   `illumination_factor = 1` stands in.
//! * The force command reaches the spacecraft through the effector's input
//!   at the next integration, one step earlier than upstream's
//!   `extForceTorque`, which buffers its input in its own update.
//!
//! Upstream's test only runs the scenario (`test_scenarioTest.py`); `tests`
//! checks that the spacecraft reaches and holds the waypoint and the
//! inertial attitude.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/rw_factory.rs"]
mod rw_factory;
#[path = "support/small_body.rs"]
mod small_body;
#[path = "support/small_body_simple_nav.rs"]
mod small_body_simple_nav;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::radiation_pressure::{RadiationPressure, RadiationPressureConfig};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::fsw_algorithms::orbit_control::small_body_waypoint_feedback::{
    SmallBodyWaypointFeedback, SmallBodyWaypointFeedbackConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyForceCommandMsg, EclipseMsg, EphemerisMsg, NavigationAttitudeMsg,
    NavigationTranslationMsg, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
    VehicleConfigMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_SUN};
use basilisk_rs::utilities::orbital_motion::state_to_hill;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use recording::planet_columns;
use small_body::{
    BENNU_MU_M3PS2, BENNU_STEP_NANOS, SPACECRAFT_MASS_KG, bennu_ephemeris, bennu_spacecraft,
    ephemeris_converter, planet_nav, spacecraft_inertia_kg_m2, wheel_cluster,
};
use small_body_simple_nav::{NavSigmas, NoisySimpleNav, grouped};

const STEM: &str = "scenarioSmallBodyFeedbackControl";
/// Upstream `simulationTime_1 = macros.sec2nano(15000.0)`.
const DURATION_NANOS: u64 = 15_000 * BENNU_STEP_NANOS;
/// Upstream `waypointFeedback.x1_ref` \[m\].
const WAYPOINT_M: [f64; 3] = [-2000.0, 0.0, 0.0];

/// Upstream `run(...)` has no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories (upstream's recorders, every step).
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    truth: MessageLog<SpacecraftStateMsg>,
    asteroid: MessageLog<PlanetStateMsg>,
    measured_translation: MessageLog<NavigationTranslationMsg>,
    measured_attitude: MessageLog<NavigationAttitudeMsg>,
    requested_force: MessageLog<BodyForceCommandMsg>,
    attitude_error: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

impl Results {
    /// Upstream `orbitalMotion.rv2hill(rc_N, vc_N, rd_N, vd_N)` of the true
    /// spacecraft state about Bennu at sample `index`: `r_BO_O`, `v_BO_O`.
    fn hill_truth(&self, index: usize) -> (Vector3<f64>, Vector3<f64>) {
        let asteroid = &self.asteroid.values[index];
        let truth = &self.truth.values[index];
        state_to_hill(
            asteroid.position_inertial_m,
            asteroid.velocity_inertial_mps,
            truth.position_m,
            truth.velocity_mps,
        )
    }
}

fn run(config: Config) -> Results {
    let inertia = spacecraft_inertia_kg_m2();
    let mut ephemeris = bennu_ephemeris(0.0, 90.0);
    let mut spacecraft = bennu_spacecraft(Vector3::new(-2000.0, 1500.0, 1000.0), Vector3::zeros());
    // Upstream `gravFactory.createSun()` and `createCustomGravObject("bennu",
    // mu)`, neither central: the spacecraft integrates heliocentric states.
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "sun",
                MU_SUN * 1.0e9,
                false,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Sun"),
        )
        .expect("unique gravity bodies");
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "bennu",
                BENNU_MU_M3PS2,
                false,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Bennu"),
        )
        .expect("unique gravity bodies");

    let mut wheels = wheel_cluster(100.0);
    let rw_config_msg = Output::new(wheels.config.clone());

    // Upstream's zeroed `SpicePlanetStateMsg` and `EphemerisMsg` of the Sun.
    let sun_state_msg = Output::new(SunEphemerisMsg::default());
    let sun_ephemeris_msg = Output::new(EphemerisMsg::default());
    let full_sun_msg = Output::new(EclipseMsg {
        illumination_factor: 1.0,
    });
    let mut srp = RadiationPressure::new(RadiationPressureConfig {
        name: "radiationPressure".to_string(),
        area_m2: 1.0,
        coefficient_reflection: 1.9,
    });

    let mut converter = ephemeris_converter("bennu");
    let mut navigation = NoisySimpleNav::new(
        "SimpleNav",
        NavSigmas {
            position: 30.0,
            velocity: 0.01,
            attitude: 0.1 * D2R,
            rate: 0.05 * D2R,
            sun: 0.0,
            delta_v: 0.0,
        }
        .p_matrix(),
        grouped([10.0, 0.001, 0.005, 0.002, 0.0, 0.0]),
    );
    let mut asteroid_navigation = planet_nav(
        "planetNav",
        [0.0, 0.0, 2.0 * D2R, 0.3 * D2R],
        [0.0, 0.0, 0.005, 0.002],
    );

    let mut inertial_point = Inertial3D::new(Inertial3DConfig {
        name: "inertialPoint".to_string(),
        sigma_r0n: Vector3::new(0.1, 0.0, 0.0),
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "trackingError".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        ..Default::default()
    });
    let mut attitude_control = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedbackControl".to_string(),
        k: 7.0,
        ki: -1.0,
        p: 30.0,
        integral_limit: 2.0 / -1.0 * 0.1,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let mut rw_motor_torque = RwMotorTorque::new(RwMotorTorqueConfig {
        name: "rwMotorTorque".to_string(),
        control_axes_body: vec![Vector3::x(), Vector3::y(), Vector3::z()],
    });
    let mut waypoint_feedback = SmallBodyWaypointFeedback::new(SmallBodyWaypointFeedbackConfig {
        name: "waypointFeedback".to_string(),
        a_sc_m2: 1.0,
        m_sc_kg: SPACECRAFT_MASS_KG,
        mu_ast_m3ps2: BENNU_MU_M3PS2,
        x1_ref_m: Vector3::from(WAYPOINT_M),
        x2_ref_mps: Vector3::zeros(),
        k1: Matrix3::from_diagonal_element(5e-4),
        k2: Matrix3::identity(),
        ..SmallBodyWaypointFeedbackConfig::default()
    });
    let mut ext_force = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "extForceTorque".to_string(),
        ext_force_inertial_n: Vector3::zeros(),
        ext_force_body_n: Vector3::zeros(),
        ext_torque_body_nm: Vector3::zeros(),
    });

    let mut truth_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut asteroid_log = MessageLog::<PlanetStateMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let mut attitude_log = MessageLog::<NavigationAttitudeMsg>::default();
    let mut force_log = MessageLog::<BodyForceCommandMsg>::default();
    let mut error_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut csv, csv_path) = scenario_recorder(STEM, config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (command, wheel) in wheels
        .splitter
        .actuator_out_msgs
        .iter()
        .zip(wheels.effector.wheels_mut())
    {
        simulation.connect(command, &mut wheel.motor_torque_in_msg);
    }
    connect!(&simulation,
        &sun_state_msg => &mut srp.input_sun_msg,
        &full_sun_msg => &mut srp.input_eclipse_msg,
        &waypoint_feedback.force_out_msg => &mut ext_force.cmd_force_body_in_msg,
    );
    spacecraft.add_state_effector(wheels.effector);
    spacecraft.add_dynamic_effector(srp);
    spacecraft.add_dynamic_effector(ext_force);
    connect!(&simulation,
        &ephemeris.planet_out_msgs[0] => spacecraft.grav_body_input_mut("bennu").expect("bennu"),
        &ephemeris.planet_out_msgs[0] => &mut converter.planet_in_msgs[0],
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &converter.ephem_out_msgs[0] => &mut asteroid_navigation.ephemeris_in_msg,
        &inertial_point.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut attitude_control.guid_in_msg,
        &vehicle_config_msg => &mut attitude_control.veh_config_in_msg,
        &attitude_control.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut wheels.splitter.torque_in_msg,
        &asteroid_navigation.ephemeris_out_msg => &mut waypoint_feedback.asteroid_ephemeris_in_msg,
        &sun_ephemeris_msg => &mut waypoint_feedback.sun_ephemeris_in_msg,
        &navigation.attitude_out_msg => &mut waypoint_feedback.nav_att_in_msg,
        &navigation.translation_out_msg => &mut waypoint_feedback.nav_trans_in_msg,
        &spacecraft.state_out => &mut truth_log.input,
        &ephemeris.planet_out_msgs[0] => &mut asteroid_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &navigation.attitude_out_msg => &mut attitude_log.input,
        &waypoint_feedback.force_out_msg => &mut force_log.input,
        &tracking_error.att_guid_out_msg => &mut error_log.input,
        &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(spacecraft_columns()),
        &ephemeris.planet_out_msgs[0] => csv.add_source::<PlanetStateMsg>(planet_columns("bennu")),
        &navigation.translation_out_msg => csv.add_source::<NavigationTranslationMsg>(
            measured_translation_columns(),
        ),
        &navigation.attitude_out_msg => csv.add_source::<NavigationAttitudeMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_bn", "sigma_BN_meas")
                    .into_iter()
                    .chain(vector_columns("omega_bn_b_radps", "omega_BN_B_meas_rad_per_s")),
            ),
        ),
        &waypoint_feedback.force_out_msg => csv.add_source::<BodyForceCommandMsg>(
            CsvSourceConfig::columns(vector_columns("force_request_body_n", "forceRequestBody_N")),
        ),
        &tracking_error.att_guid_out_msg => csv.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
    );
    // Upstream's priorities, with the ephemeris ahead of the spacecraft: the
    // gravity effector takes Bennu's state as written at the current step.
    schedule! { simulation,
        "planetEphemeris" => &mut ephemeris, period=BENNU_STEP_NANOS, priority=210;
        "bskSat" => &mut spacecraft, period=BENNU_STEP_NANOS, priority=200;
        "ephemConverter" => &mut converter, period=BENNU_STEP_NANOS, priority=197;
        "SimpleNav" => &mut navigation, period=BENNU_STEP_NANOS, priority=196;
        "planetNav" => &mut asteroid_navigation, period=BENNU_STEP_NANOS, priority=195;
        "inertialPoint" => &mut inertial_point, period=BENNU_STEP_NANOS, priority=108;
        "trackingError" => &mut tracking_error, period=BENNU_STEP_NANOS, priority=106;
        "mrpFeedbackControl" => &mut attitude_control, period=BENNU_STEP_NANOS, priority=105;
        "rwMotorTorque" => &mut rw_motor_torque, period=BENNU_STEP_NANOS, priority=81;
        "rwMotorTorqueSplitter" => &mut wheels.splitter, period=BENNU_STEP_NANOS, priority=80;
        "waypointFeedback" => &mut waypoint_feedback, period=BENNU_STEP_NANOS, priority=78;
        "sc_truth_recorder" => &mut truth_log, period=BENNU_STEP_NANOS, priority=0;
        "ast_truth_recorder" => &mut asteroid_log, period=BENNU_STEP_NANOS, priority=0;
        "sc_meas_recorder" => &mut translation_log, period=BENNU_STEP_NANOS, priority=0;
        "sc_att_meas_recorder" => &mut attitude_log, period=BENNU_STEP_NANOS, priority=0;
        "requested_control_recorder" => &mut force_log, period=BENNU_STEP_NANOS, priority=0;
        "attitude_error_recorder" => &mut error_log, period=BENNU_STEP_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut csv, BENNU_STEP_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(csv);

    Results {
        truth: truth_log,
        asteroid: asteroid_log,
        measured_translation: translation_log,
        measured_attitude: attitude_log,
        requested_force: force_log,
        attitude_error: error_log,
        csv_path,
    }
}

/// Upstream `sc_truth_recorder`'s `r_BN_N`, `v_BN_N`, `sigma_BN`,
/// `omega_BN_B`.
fn spacecraft_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", "r_BN_N_m")
            .into_iter()
            .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
            .chain(vector_columns("sigma_bn", "sigma_BN"))
            .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
    )
}

/// Upstream `sc_meas_recorder`'s `r_BN_N`, `v_BN_N`.
fn measured_translation_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_bn_n_m", "r_BN_N_meas_m")
            .into_iter()
            .chain(vector_columns("velocity_bn_n_mps", "v_BN_N_meas_m_per_s")),
    )
}

fn main() {
    let cli = Cli::from_env("scenario_small_body_feedback_control");
    cli.finish();
    let results = run(Config { write_csv: true });
    let last = results.truth.values.len() - 1;
    let (r_bo_o, v_bo_o) = results.hill_truth(last);
    println!(
        "wrote {} (final r_BO_O = [{:.1}, {:.1}, {:.1}] m, |v_BO_O| = {:.2e} m/s, |sigma_BR| = {:.2e})",
        results.csv_path.as_ref().expect("CSV written").display(),
        r_bo_o.x,
        r_bo_o.y,
        r_bo_o.z,
        v_bo_o.norm(),
        results.attitude_error.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream's `test_scenarioTest.py` only runs this scenario. These tests
    //! check what it demonstrates: the waypoint is reached and held, and the
    //! inertial attitude is held while the force is applied.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_spacecraft_reaches_and_holds_the_waypoint() {
        let results = quiet();
        assert_eq!(results.truth.values.len(), 15_001);
        let waypoint = Vector3::from(WAYPOINT_M);
        let (start, _) = results.hill_truth(0);
        assert!((start - waypoint).norm() > 1500.0, "{start}");
        // The last hour stays within 10 m of the waypoint (the controller
        // acts on 30 m navigation noise) and at rest relative to Bennu.
        for index in (results.truth.values.len() - 3600)..results.truth.values.len() {
            let (r_bo_o, v_bo_o) = results.hill_truth(index);
            assert!(
                (r_bo_o - waypoint).norm() < 10.0,
                "sample {index}: r_BO_O = {r_bo_o}"
            );
            assert!(v_bo_o.norm() < 1.0e-2, "sample {index}: v_BO_O = {v_bo_o}");
            let force = results.requested_force.values[index].force_request_body_n;
            assert!(force.norm() < 0.1, "sample {index}: force {force} N");
        }
    }

    /// The controller acts on `simpleNav` measurements with upstream's
    /// error settings, bounded random walks about the truth.
    #[test]
    fn the_measurements_carry_the_navigation_errors() {
        let results = quiet();
        let count = results.truth.values.len();
        let (mut position, mut velocity, mut attitude) = (0.0_f64, 0.0_f64, 0.0_f64);
        let (mut position_max, mut velocity_max) = (0.0_f64, 0.0_f64);
        for index in 0..count {
            let truth = &results.truth.values[index];
            let translation = &results.measured_translation.values[index];
            let attitude_meas = &results.measured_attitude.values[index];
            let dr = (translation.position_bn_n_m - truth.position_m).norm();
            let dv = (translation.velocity_bn_n_mps - truth.velocity_mps).norm();
            let dsigma = basilisk_rs::utilities::rigid_body_kinematics::sub_mrp(
                attitude_meas.sigma_bn,
                truth.sigma_bn,
            )
            .norm();
            position += dr * dr;
            velocity += dv * dv;
            attitude += dsigma * dsigma;
            position_max = position_max.max(dr);
            velocity_max = velocity_max.max(dv);
        }
        let rms = |sum: f64| (sum / count as f64).sqrt();
        // Per axis the walks stay near their 10 m, 1 mm/s and 0.005 bounds,
        // pushed past them by the 30 m, 1 cm/s and 0.1 deg steps.
        assert!((5.0..40.0).contains(&rms(position)), "{} m", rms(position));
        assert!(position_max < 100.0, "{position_max} m");
        assert!(
            (1.0e-3..3.0e-2).contains(&rms(velocity)),
            "{} m/s",
            rms(velocity)
        );
        assert!(velocity_max < 0.1, "{velocity_max} m/s");
        assert!(
            (1.0e-3..2.0e-2).contains(&rms(attitude)),
            "{}",
            rms(attitude)
        );
    }

    #[test]
    fn the_wheels_hold_the_inertial_attitude() {
        let results = quiet();
        let reference = Vector3::new(0.1, 0.0, 0.0);
        // Once the initial 0.1 rad/s tumble is removed, the true attitude
        // stays within the navigation errors the controller tracks: their
        // attitude and rate walks are bounded at 0.005 and 0.002 rad/s.
        for state in results.truth.since(2_000 * BENNU_STEP_NANOS) {
            let error =
                basilisk_rs::utilities::rigid_body_kinematics::sub_mrp(state.sigma_bn, reference);
            assert!(error.norm() < 3.0e-2, "sigma_BR = {error}");
            assert!(state.omega_radps.norm() < 5.0e-3, "{}", state.omega_radps);
        }
        assert!(results.truth.values[0].omega_radps.norm() > 0.17);
    }
}
