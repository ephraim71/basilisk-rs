//! Small-body relative navigation about Bennu with `smallBodyNavEKF`: port
//! of upstream `examples/scenarioSmallBodyNav.py`.
//!
//! The 330 kg spacecraft with three HR16 wheels (50 N m s) starts
//! `r_BO_N = [2000, 1500, 1000] m` from Bennu, drifting at
//! `v_BO_N = [1, 1, 1] m/s`. Bennu (`mu = 4.892 m^3/s^2`) follows its
//! December 31st, 2018 heliocentric orbit (`planetEphemeris`, pole at
//! right ascension 86.6388 deg and declination -65.1086 deg, 4.297461 h
//! spin); the Sun is a point mass at the origin of the zeroed Sun messages
//! upstream writes for `radiationPressure` (cannonball, 3 m^2, `Cr = 0.9`)
//! and the filter.
//!
//! The extended Kalman filter estimates the spacecraft position and velocity
//! relative to Bennu in Bennu's Hill frame, and Bennu's attitude and rate,
//! from `simpleNav` (40 m, 5 cm/s) and `planetNav` (1 deg, 0.1 deg/s)
//! measurements, with a zeroed thruster message and the commanded force as
//! inputs. `smallBodyWaypointFeedback` (`K1 = 5e-4 I`, `K2 = I`) steers
//! towards the Hill-frame waypoint `[-2000, 0, 0] m` from the filter's
//! estimate, while `hillPoint`, `attTrackingError`, `mrpFeedback` (`K = 7`,
//! `P = 35`) and `rwMotorTorque` point the spacecraft at Bennu from a second
//! `simpleNav` instance. Three 1 s tasks: dynamics, measurements and flight
//! software. The run lasts 5000 s with the measurement task disabled from
//! 1000 to 2000 s and from 3000 to 4000 s, so the filter only propagates
//! there.
//!
//! Upstream `run(show_plots)` has no parameters. Writes, in
//! `examples/output/scenarios/`, `scenarioSmallBodyNav.csv` (truth, filter
//! state and covariance diagonal every step) and `scenarioSmallBodyNavMeas.csv`
//! (the measurements, recorded on the measurement task as upstream does);
//! `plot_scenario_small_body_nav.py` redraws upstream's eight figures.
//!
//! # Differences from upstream
//!
//! * basilisk-rs's `SimpleNavigation` has no error model, so both
//!   navigation instances are [`small_body_simple_nav::NoisySimpleNav`],
//!   upstream's 18-state `simpleNav` errors on the crate's `GaussMarkov`
//!   (3-sigma units, soft bounds, as `planetNav`); the noise streams are not
//!   upstream's, and the two instances draw different ones.
//! * Upstream's filter updates only on measurements written at the current
//!   time (message headers). The task switch that stands in for upstream's
//!   `disableTask` also tells the filter when the measurement task ran,
//!   through [`MeasurementGatedEkf`] and
//!   `SmallBodyNavEkf::update_with_measurement_status`.
//! * The wheels take one command each (`arrayMotorTorqueToSingleActuators`),
//!   `radiationPressure` reads a constant full-sun eclipse message (upstream
//!   leaves it unlinked), and the force command reaches the spacecraft at the
//!   next integration, one step earlier than upstream's `extForceTorque`.
//!
//! Upstream's test only runs the scenario (`test_scenarioTest.py`); `tests`
//! checks the filter's convergence, its consistency with its covariance,
//! and the covariance growth during the outages.

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
#[path = "support/task_switch.rs"]
mod task_switch;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::dynamics::radiation_pressure::{RadiationPressure, RadiationPressureConfig};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::hill_point::{HillPoint, HillPointConfig};
use basilisk_rs::fsw_algorithms::orbit_control::small_body_waypoint_feedback::{
    SmallBodyWaypointFeedback, SmallBodyWaypointFeedbackConfig,
};
use basilisk_rs::fsw_algorithms::rw_motor_torque::{RwMotorTorque, RwMotorTorqueConfig};
use basilisk_rs::fsw_algorithms::small_body_navigation::small_body_nav_ekf::{
    SmallBodyNavEkf, SmallBodyNavEkfConfig, SmallBodyNavMatrix, SmallBodyNavState,
};
use basilisk_rs::messages::{
    EclipseMsg, EphemerisMsg, NavigationTranslationMsg, Output, PlanetStateMsg, SmallBodyNavMsg,
    SpacecraftStateMsg, SunEphemerisMsg, ThrusterOutputMsg, VehicleConfigMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_SUN};
use basilisk_rs::utilities::orbital_motion::hill_frame;
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::{array_columns, vector_columns};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use recording::planet_columns;
use small_body::{
    BENNU_MU_M3PS2, BENNU_STEP_NANOS, SPACECRAFT_MASS_KG, bennu_ephemeris, bennu_spacecraft,
    ephemeris_converter, planet_nav, spacecraft_inertia_kg_m2, wheel_cluster,
};
use small_body_simple_nav::{NavSigmas, NoisySimpleNav, grouped};
use task_switch::{Switched, TaskSwitch};

const STEM: &str = "scenarioSmallBodyNav";
/// Upstream `simulationTime = macros.sec2nano(1000.0)`: each of the five
/// legs, alternately with and without the measurement task.
const LEG_NANOS: u64 = 1_000 * BENNU_STEP_NANOS;
/// Upstream `simpleNav` position and velocity sigmas \[m, m/s\], also the
/// filter's measurement noise.
const POSITION_SIGMA_M: f64 = 40.0;
const VELOCITY_SIGMA_MPS: f64 = 0.05;
/// Upstream `planetNav` attitude and rate sigmas \[-, rad/s\], also the
/// filter's measurement noise.
const ATTITUDE_SIGMA: f64 = 1.0 * D2R;
const RATE_SIGMA_RADPS: f64 = 0.1 * D2R;

/// Upstream `run(...)` has no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV files (off in the tests).
    write_csv: bool,
}

/// `smallBodyNavEKF` with upstream's header check: the measurements are new
/// when the measurement task ran this step. The navigation attitude comes
/// from the dynamics task, which always runs.
struct MeasurementGatedEkf {
    ekf: SmallBodyNavEkf,
    measurements: TaskSwitch,
}

impl Module for MeasurementGatedEkf {
    fn init(&mut self) {
        self.ekf.init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        Module::reset(&mut self.ekf, context);
    }

    fn update(&mut self, context: &SimulationContext) {
        let connected = self.ekf.nav_trans_in_msg.is_connected()
            && self.ekf.nav_att_in_msg.is_connected()
            && self.ekf.asteroid_ephemeris_in_msg.is_connected();
        self.ekf.update_with_measurement_status(
            context.current_sim_nanos,
            connected && self.measurements.is_enabled(),
        );
    }
}

/// The recorded histories (upstream's recorders, every step of their task).
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    truth: MessageLog<SpacecraftStateMsg>,
    asteroid: MessageLog<PlanetStateMsg>,
    asteroid_attitude: MessageLog<EphemerisMsg>,
    estimate: MessageLog<SmallBodyNavMsg>,
    measured_translation: MessageLog<NavigationTranslationMsg>,
    csv_paths: Vec<PathBuf>,
}

impl Results {
    /// The true filter state at sample `index`: upstream's
    /// `dcm_ON (r_BN_N - r_AN_N)`, `dcm_ON (v_BN_N - v_AN_N)` and Bennu's
    /// `sigma_BN`, `omega_BN_B` from the ephemeris converter.
    fn true_state(&self, index: usize) -> SmallBodyNavState {
        let asteroid = &self.asteroid.values[index];
        let truth = &self.truth.values[index];
        let attitude = &self.asteroid_attitude.values[index];
        let dcm_on = hill_frame(asteroid.position_inertial_m, asteroid.velocity_inertial_mps);
        let mut state = SmallBodyNavState::zeros();
        state
            .fixed_rows_mut::<3>(0)
            .copy_from(&(dcm_on * (truth.position_m - asteroid.position_inertial_m)));
        state
            .fixed_rows_mut::<3>(3)
            .copy_from(&(dcm_on * (truth.velocity_mps - asteroid.velocity_inertial_mps)));
        state.fixed_rows_mut::<3>(6).copy_from(&attitude.sigma_bn);
        state
            .fixed_rows_mut::<3>(9)
            .copy_from(&attitude.omega_bn_b_radps);
        state
    }

    /// Truth minus estimate at sample `index`.
    fn error(&self, index: usize) -> SmallBodyNavState {
        self.true_state(index) - self.estimate.values[index].state
    }

    /// The filter's one-sigma bound of state `component` at sample `index`.
    fn sigma(&self, index: usize, component: usize) -> f64 {
        self.estimate.values[index].covar[(component, component)].sqrt()
    }
}

fn run(config: Config) -> Results {
    let inertia = spacecraft_inertia_kg_m2();
    let measurement_task = TaskSwitch::new(true);
    let mut ephemeris = bennu_ephemeris(86.6388, -65.1086);
    let mut spacecraft = bennu_spacecraft(
        Vector3::new(2000.0, 1500.0, 1000.0),
        Vector3::new(1.0, 1.0, 1.0),
    );
    // Upstream `gravFactory.createSun()` and `createCustomGravObject("bennu",
    // mu, radEquator=565 / 2)`, neither central.
    for body in [
        GravBodyData::point_mass(
            "sun",
            MU_SUN * 1.0e9,
            false,
            Vector3::zeros(),
            Vector3::zeros(),
        ),
        GravBodyData::point_mass(
            "bennu",
            BENNU_MU_M3PS2,
            false,
            Vector3::zeros(),
            Vector3::zeros(),
        ),
    ] {
        spacecraft
            .add_grav_body(body.expect("valid point mass"))
            .expect("unique gravity bodies");
    }

    let mut wheels = wheel_cluster(50.0);
    let rw_config_msg = Output::new(wheels.config.clone());
    // Upstream's zeroed Sun `SpicePlanetStateMsg` and `EphemerisMsg`, and
    // zeroed `THROutputMsg`.
    let sun_state_msg = Output::new(SunEphemerisMsg::default());
    let sun_ephemeris_msg = Output::new(EphemerisMsg::default());
    let thruster_msg = Output::new(ThrusterOutputMsg::default());
    let full_sun_msg = Output::new(EclipseMsg {
        illumination_factor: 1.0,
    });
    let mut srp = RadiationPressure::new(RadiationPressureConfig {
        name: "radiationPressure".to_string(),
        area_m2: 3.0,
        coefficient_reflection: 0.9,
    });
    let mut converter = ephemeris_converter("bennu");

    let nav_sigmas = NavSigmas {
        position: POSITION_SIGMA_M,
        velocity: VELOCITY_SIGMA_MPS,
        ..NavSigmas::default()
    };
    let walk_bounds = grouped([10.0, 0.01, 0.005, 0.002, 0.0, 0.0]);
    let mut measurement_nav = Switched::new(
        NoisySimpleNav::new("SimpleNav", nav_sigmas.p_matrix(), walk_bounds),
        &measurement_task,
    );
    let mut attitude_nav = NoisySimpleNav::new("SimpleNav2", nav_sigmas.p_matrix(), walk_bounds);
    let mut asteroid_navigation = Switched::new(
        planet_nav(
            "planetNav",
            [0.0, 0.0, ATTITUDE_SIGMA, RATE_SIGMA_RADPS],
            [0.0, 0.0, 0.005, 0.002],
        ),
        &measurement_task,
    );

    let mut sun_point = HillPoint::new(HillPointConfig {
        name: "sunPoint".to_string(),
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
        p: 35.0,
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
        x1_ref_m: Vector3::new(-2000.0, 0.0, 0.0),
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

    let mut ekf = SmallBodyNavEkf::new(SmallBodyNavEkfConfig {
        name: "smallBodyNavEKF".to_string(),
        area_sc_m2: 1.0,
        mass_sc_kg: SPACECRAFT_MASS_KG,
        mu_ast_m3ps2: BENNU_MU_M3PS2,
        process_noise: SmallBodyNavMatrix::from_diagonal(&SmallBodyNavState::from_fn(
            |index, _| [1e-7, 1e-6, 1e-6, 1e-7][index / 3],
        )),
        measurement_noise: SmallBodyNavMatrix::from_diagonal(&SmallBodyNavState::from_fn(
            |index, _| {
                [
                    POSITION_SIGMA_M,
                    VELOCITY_SIGMA_MPS,
                    ATTITUDE_SIGMA,
                    RATE_SIGMA_RADPS,
                ][index / 3]
                    .powi(2)
            },
        )),
        ..SmallBodyNavEkfConfig::default()
    });
    ekf.x_hat_k = SmallBodyNavState::from_column_slice(&[
        2458.0, -704.08, 844.275, 1.475, -0.176, 0.894, -0.58, 0.615, 0.125, 0.0, 0.0, 0.0004,
    ]);
    ekf.p_k = 0.1 * SmallBodyNavMatrix::identity();

    let mut truth_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut asteroid_log = MessageLog::<PlanetStateMsg>::default();
    let mut asteroid_attitude_log = MessageLog::<EphemerisMsg>::default();
    let mut estimate_log = MessageLog::<SmallBodyNavMsg>::default();
    let mut translation_log = Switched::new(
        MessageLog::<NavigationTranslationMsg>::default(),
        &measurement_task,
    );
    let (mut csv, csv_path) = scenario_recorder(STEM, config.write_csv);
    let (measurement_csv, measurement_csv_path) =
        scenario_recorder(&format!("{STEM}Meas"), config.write_csv);
    let mut measurement_csv = Switched::new(measurement_csv, &measurement_task);

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
        &spacecraft.state_out => &mut measurement_nav.module.spacecraft_state_in_msg,
        &spacecraft.state_out => &mut attitude_nav.spacecraft_state_in_msg,
        &converter.ephem_out_msgs[0] => &mut asteroid_navigation.module.ephemeris_in_msg,
        &converter.ephem_out_msgs[0] => &mut sun_point.cel_body_in_msg,
        &attitude_nav.translation_out_msg => &mut sun_point.trans_nav_in_msg,
        &sun_point.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &attitude_nav.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &tracking_error.att_guid_out_msg => &mut attitude_control.guid_in_msg,
        &vehicle_config_msg => &mut attitude_control.veh_config_in_msg,
        &attitude_control.cmd_torque_out_msg => &mut rw_motor_torque.veh_control_in_msg,
        &rw_config_msg => &mut rw_motor_torque.rw_params_in_msg,
        &rw_motor_torque.rw_motor_torque_out_msg => &mut wheels.splitter.torque_in_msg,
        &asteroid_navigation.module.ephemeris_out_msg => &mut waypoint_feedback.asteroid_ephemeris_in_msg,
        &sun_ephemeris_msg => &mut waypoint_feedback.sun_ephemeris_in_msg,
        &attitude_nav.attitude_out_msg => &mut waypoint_feedback.nav_att_in_msg,
        &ekf.nav_trans_out_msg => &mut waypoint_feedback.nav_trans_in_msg,
        &measurement_nav.module.translation_out_msg => &mut ekf.nav_trans_in_msg,
        &attitude_nav.attitude_out_msg => &mut ekf.nav_att_in_msg,
        &asteroid_navigation.module.ephemeris_out_msg => &mut ekf.asteroid_ephemeris_in_msg,
        &sun_ephemeris_msg => &mut ekf.sun_ephemeris_in_msg,
        &waypoint_feedback.force_out_msg => &mut ekf.cmd_force_body_in_msg,
        &thruster_msg => ekf.add_thruster_input(),
        &spacecraft.state_out => &mut truth_log.input,
        &ephemeris.planet_out_msgs[0] => &mut asteroid_log.input,
        &converter.ephem_out_msgs[0] => &mut asteroid_attitude_log.input,
        &ekf.small_body_nav_out_msg => &mut estimate_log.input,
        &measurement_nav.module.translation_out_msg => &mut translation_log.module.input,
        &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m")
                .into_iter()
                .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
        )),
        &ephemeris.planet_out_msgs[0] => csv.add_source::<PlanetStateMsg>(planet_columns("bennu")),
        &converter.ephem_out_msgs[0] => csv.add_source::<EphemerisMsg>(CsvSourceConfig::columns(
            vector_columns("sigma_bn", "sigma_AN")
                .into_iter()
                .chain(vector_columns("omega_bn_b_radps", "omega_AN_A_rad_per_s")),
        )),
        &ekf.small_body_nav_out_msg => csv.add_source::<SmallBodyNavMsg>(CsvSourceConfig::columns(
            array_columns("state", "x_hat", 12)
                .into_iter()
                .chain((1..=12).map(|index| {
                    (format!("covar.m{index}_{index}"), format!("P_{}", index - 1))
                })),
        )),
        &measurement_nav.module.translation_out_msg => measurement_csv.module.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_bn_n_m", "r_BN_N_meas_m")
                    .into_iter()
                    .chain(vector_columns("velocity_bn_n_mps", "v_BN_N_meas_m_per_s")),
            ),
        ),
        &asteroid_navigation.module.ephemeris_out_msg => measurement_csv.module.add_source::<EphemerisMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_bn", "sigma_AN_meas")
                    .into_iter()
                    .chain(vector_columns("omega_bn_b_radps", "omega_AN_A_meas_rad_per_s")),
            ),
        ),
    );
    let mut ekf = MeasurementGatedEkf {
        ekf,
        measurements: measurement_task.clone(),
    };
    // Upstream's three tasks in creation order, each with its module
    // priorities, and the ephemeris ahead of the spacecraft: the gravity
    // effector takes Bennu's state as written at the current step.
    schedule! { simulation,
        "planetEphemeris" => &mut ephemeris, period=BENNU_STEP_NANOS, priority=310;
        "bskSat" => &mut spacecraft, period=BENNU_STEP_NANOS, priority=300;
        "ephemConverter" => &mut converter, period=BENNU_STEP_NANOS, priority=298;
        "SimpleNav2" => &mut attitude_nav, period=BENNU_STEP_NANOS, priority=290;
        "sc_truth_recorder" => &mut truth_log, period=BENNU_STEP_NANOS, priority=280;
        "ast_truth_recorder" => &mut asteroid_log, period=BENNU_STEP_NANOS, priority=280;
        "ast_ephemeris_recorder" => &mut asteroid_attitude_log, period=BENNU_STEP_NANOS, priority=280;
        "SimpleNav" => &mut measurement_nav, period=BENNU_STEP_NANOS, priority=197;
        "planetNav" => &mut asteroid_navigation, period=BENNU_STEP_NANOS, priority=196;
        "sc_meas_recorder" => &mut translation_log, period=BENNU_STEP_NANOS, priority=180;
        "sunPoint" => &mut sun_point, period=BENNU_STEP_NANOS, priority=95;
        "trackingError" => &mut tracking_error, period=BENNU_STEP_NANOS, priority=94;
        "mrpFeedbackControl" => &mut attitude_control, period=BENNU_STEP_NANOS, priority=93;
        "rwMotorTorque" => &mut rw_motor_torque, period=BENNU_STEP_NANOS, priority=92;
        "rwMotorTorqueSplitter" => &mut wheels.splitter, period=BENNU_STEP_NANOS, priority=91;
        "smallBodyNavEKF" => &mut ekf, period=BENNU_STEP_NANOS, priority=90;
        "waypointFeedback" => &mut waypoint_feedback, period=BENNU_STEP_NANOS, priority=89;
        "state_recorder" => &mut estimate_log, period=BENNU_STEP_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut csv, BENNU_STEP_NANOS, 0);
        simulation.add_module(
            "measurement_csv",
            &mut measurement_csv,
            BENNU_STEP_NANOS,
            180,
        );
    }

    // Upstream: 1000 s, then `disableTask(measTaskName)` and `enableTask`
    // alternately for four more 1000 s legs.
    simulation.run_for(LEG_NANOS);
    for leg in 1..5 {
        if leg % 2 == 1 {
            measurement_task.disable();
        } else {
            measurement_task.enable();
        }
        simulation.run_for(LEG_NANOS);
    }
    drop(simulation);
    drop(csv);
    drop(measurement_csv);

    Results {
        truth: truth_log,
        asteroid: asteroid_log,
        asteroid_attitude: asteroid_attitude_log,
        estimate: estimate_log,
        measured_translation: translation_log.module,
        csv_paths: csv_path.into_iter().chain(measurement_csv_path).collect(),
    }
}

fn main() {
    let cli = Cli::from_env("scenario_small_body_nav");
    cli.finish();
    let results = run(Config { write_csv: true });
    let last = results.estimate.values.len() - 1;
    let error = results.error(last);
    for path in &results.csv_paths {
        println!("wrote {}", path.display());
    }
    println!(
        "final estimation error: |r| = {:.2} m (3 sigma {:.2} m), |v| = {:.2e} m/s, |sigma_AN| = {:.2e}",
        error.fixed_rows::<3>(0).norm(),
        3.0 * results.sigma(last, 0),
        error.fixed_rows::<3>(3).norm(),
        error.fixed_rows::<3>(6).norm(),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream's `test_scenarioTest.py` only runs this scenario. These tests
    //! check what it demonstrates: the filter converges from its initial
    //! guess, stays consistent with its covariance, and its covariance grows
    //! while the measurement task is off and shrinks again when it resumes.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    /// Sample index at simulation time `seconds` (one sample per second).
    fn at(seconds: usize) -> usize {
        seconds
    }

    #[test]
    fn the_translation_estimate_stays_within_its_covariance() {
        let results = quiet();
        assert_eq!(results.estimate.values.len(), 5_001);
        // The measurement task ran in three 1000 s legs.
        assert_eq!(results.measured_translation.values.len(), 3_001);
        for index in 0..results.estimate.values.len() {
            let error = results.error(index);
            for component in 0..6 {
                assert!(
                    error[component].abs() <= 3.0 * results.sigma(index, component),
                    "t = {index} s, state {component}: error {} beyond 3 sigma {}",
                    error[component],
                    3.0 * results.sigma(index, component)
                );
            }
            // Inside the 40 m and 5 cm/s measurement noise, the velocity
            // after the first minutes well inside it.
            assert!(
                error.fixed_rows::<3>(0).norm() < POSITION_SIGMA_M,
                "t = {index} s: {error}"
            );
            assert!(
                error.fixed_rows::<3>(3).norm() < 5.0e-2,
                "t = {index} s: {error}"
            );
            if index >= at(300) {
                assert!(
                    error.fixed_rows::<3>(3).norm() < 1.0e-2,
                    "t = {index} s: {error}"
                );
            }
        }
    }

    #[test]
    fn the_covariance_grows_during_the_outages() {
        let results = quiet();
        for (outage_start, outage_end) in [(at(1_000), at(2_000)), (at(3_000), at(4_000))] {
            // The filter only propagates: the position and Bennu attitude
            // uncertainties grow, the latter by an order of magnitude.
            assert!(
                results.sigma(outage_end, 0) > 1.2 * results.sigma(outage_start, 0),
                "position sigma {} -> {}",
                results.sigma(outage_start, 0),
                results.sigma(outage_end, 0)
            );
            assert!(
                results.sigma(outage_end, 6) > 10.0 * results.sigma(outage_start, 6),
                "attitude sigma {} -> {}",
                results.sigma(outage_start, 6),
                results.sigma(outage_end, 6)
            );
            // Ten seconds of measurements bring them back down.
            let resumed = outage_end + 10;
            assert!(results.sigma(resumed, 0) < results.sigma(outage_end, 0));
            assert!(results.sigma(resumed, 6) < 0.1 * results.sigma(outage_end, 6));
        }
    }

    #[test]
    fn the_measured_legs_recover_bennus_attitude_and_rate() {
        let results = quiet();
        for leg_end in [at(1_000), at(3_000), at(5_000)] {
            let error = results.error(leg_end);
            // Bounded by the 1 deg and 0.1 deg/s `planetNav` errors and
            // their 0.005 and 0.002 rad/s walk bounds.
            assert!(
                error.fixed_rows::<3>(6).norm() < 6.0e-2,
                "t = {leg_end} s: {error}"
            );
            assert!(
                error.fixed_rows::<3>(9).norm() < 2.0e-3,
                "t = {leg_end} s: {error}"
            );
            assert!(results.sigma(leg_end, 6) < 2.0e-2);
        }
    }
}
