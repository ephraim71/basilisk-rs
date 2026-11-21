//! Port of upstream `examples/scenarioLambertSolver.py`: a Lambert transfer
//! to a target position, planned, solved and validated in flight software,
//! and executed as an impulsive burn.
//!
//! A 10 000 km, nearly circular orbit about a point-mass Earth (10 s dynamics
//! task). The flight-software task (30 s) runs `lambertPlanner`,
//! `lambertSolver` and `lambertValidator` on the navigation solution to reach
//! `r_TN_N = [-(R_E + 200 km), 0, 0]` at half an orbit, with the burn at a
//! quarter orbit. At the maneuver time the validated delta-v is added to the
//! hub velocity through the spacecraft's
//! [`HubStateHandle`](basilisk_rs::spacecraft::HubStateHandle) (upstream
//! `velRef.setState`), the flight-software task is disabled
//! (`fswProcess.disableAllTasks()`, here a [`task_switch::TaskSwitch`]) and the
//! run continues to the final time.
//!
//! The navigation is [`SimpleNavigation`] with upstream's Gauss-Markov
//! errors (1 m and 1 cm/s standard deviations, `PMatrix`, and `walkBounds`
//! of 10 m and 1 m/s); its noise stream is not upstream's.
//!
//! Writes `scenarioLambertSolver.csv` (truth and navigation states every
//! 10 s) and `scenarioLambertSolverFsw.csv` (the flight-software outputs every
//! 30 s up to the maneuver). Upstream has no test for this scenario; `tests`
//! checks that the validator accepts the burn and the spacecraft reaches the
//! target within the validator's `maxDistanceTarget` plus what the
//! navigation errors at the maneuver account for.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/task_switch.rs"]
mod task_switch;

use basilisk_rs::fsw_algorithms::orbit_control::lambert_planner::{
    LambertPlanner, LambertPlannerConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_solver::{
    LambertSolver, LambertSolverConfig,
};
use basilisk_rs::fsw_algorithms::orbit_control::lambert_validator::{
    LambertValidator, LambertValidatorConfig,
};
use basilisk_rs::messages::{
    DvBurnCommandMsg, LambertPerformanceMsg, LambertProblemMsg, LambertSolutionMsg,
    LambertValidatorMsg, NavigationTranslationMsg, SpacecraftStateMsg,
};
use basilisk_rs::sensors::navigation::{SimpleNavMatrix, SimpleNavVector, SimpleNavigation};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use orbit_tools::{orbit_period_s, run_until, sec2nano};
use recording::{scenario_csv, state_columns};
use task_switch::{Switched, TaskSwitch};

/// Upstream `simStep` and `fswStep` (s).
const SIM_STEP_S: f64 = 10.0;
const FSW_STEP_S: f64 = 30.0;
/// Upstream `rEarth` (m).
const R_EARTH_M: f64 = 6378.0 * 1e3;
/// Upstream `maxDistanceTarget` (m).
pub const MAX_DISTANCE_TARGET_M: f64 = 500.0;

pub struct Results {
    pub truth: Vec<(u64, SpacecraftStateMsg)>,
    /// The navigation solution, every 10 s.
    pub navigation: Vec<(u64, NavigationTranslationMsg)>,
    pub dv_commands: Vec<(u64, DvBurnCommandMsg)>,
    pub validator: Vec<(u64, LambertValidatorMsg)>,
    /// Upstream `r_TN_N` (m).
    pub target_m: Vector3<f64>,
    /// Upstream `tm` and `tf` (s).
    pub maneuver_time_s: f64,
    pub final_time_s: f64,
}

pub fn run() -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mu = Body::Earth.mu_m3ps2();
    let elements = ClassicElements {
        semi_major_axis: 10_000.0 * 1e3,
        eccentricity: 0.001,
        inclination_rad: 5.0 * degrees,
        raan_rad: 10.0 * degrees,
        arg_periapsis_rad: 10.0 * degrees,
        true_anomaly_rad: 10.0 * degrees,
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(mu, &elements);
    let tau = orbit_period_s(mu, elements.semi_major_axis);
    let maneuver_time_s = (tau / 4.0 / SIM_STEP_S).round() * SIM_STEP_S;
    let final_time_s = (tau / 2.0 / SIM_STEP_S).round() * SIM_STEP_S;
    let target_m = Vector3::new(-(R_EARTH_M + 200.0 * 1e3), 0.0, 0.0);
    let (pos_sigma, vel_sigma) = (1.0, 0.01);

    let sim_step_nanos = sec2nano(SIM_STEP_S);
    let fsw_step_nanos = sec2nano(FSW_STEP_S);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 330.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: sim_step_nanos,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central body");
    let hub = spacecraft.hub_state_handle();
    // Upstream's `p_matrix_sc` (1-sigma) and `walk_bounds_sc`: position and
    // velocity errors only.
    let mut p_matrix_sc = SimpleNavMatrix::zeros();
    let mut walk_bounds_sc = SimpleNavVector::zeros();
    for axis in 0..3 {
        p_matrix_sc[(axis, axis)] = pos_sigma;
        p_matrix_sc[(axis + 3, axis + 3)] = vel_sigma;
        walk_bounds_sc[axis] = 10.0;
        walk_bounds_sc[axis + 3] = 1.0;
    }
    let mut navigation = SimpleNavigation::with_errors("SimpleNav", p_matrix_sc, walk_bounds_sc);

    let fsw = TaskSwitch::new(true);
    let mut planner = Switched::new(
        LambertPlanner::new(LambertPlannerConfig {
            name: "lambertPlanner".to_string(),
            r_tn_n_m: target_m,
            final_time_s,
            maneuver_time_s,
            mu_m3ps2: mu,
            num_revolutions: 0,
            ..LambertPlannerConfig::default()
        }),
        &fsw,
    );
    let mut solver = Switched::new(LambertSolver::new(LambertSolverConfig::default()), &fsw);
    let mut validator = Switched::new(
        LambertValidator::new(LambertValidatorConfig {
            name: "lambertValidator".to_string(),
            final_time_s,
            maneuver_time_s,
            max_distance_target_m: MAX_DISTANCE_TARGET_M,
            min_orbit_radius_m: R_EARTH_M,
            uncertainty_states: Matrix6::from_diagonal(&Vector6::new(
                pos_sigma, pos_sigma, pos_sigma, vel_sigma, vel_sigma, vel_sigma,
            )),
            uncertainty_dv_mps: 0.1,
            dv_convergence_tolerance_mps: 1.0,
            ..LambertValidatorConfig::default()
        }),
        &fsw,
    );

    let mut truth_csv = scenario_csv("scenarioLambertSolver");
    let mut truth = MessageLog::<SpacecraftStateMsg>::default();
    let mut navigation_history = MessageLog::<NavigationTranslationMsg>::default();
    // Upstream's FSW recorders sit in the FSW task, and stop with it.
    let mut fsw_csv = Switched::new(scenario_csv("scenarioLambertSolverFsw"), &fsw);
    let mut dv_commands = Switched::new(MessageLog::<DvBurnCommandMsg>::default(), &fsw);
    let mut validator_history = Switched::new(MessageLog::<LambertValidatorMsg>::default(), &fsw);
    let dv_command = validator.module.dv_burn_cmd_out_msg.clone();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut planner.module.nav_trans_in_msg,
            &planner.module.lambert_problem_out_msg => &mut solver.module.lambert_problem_in_msg,
            &navigation.translation_out_msg => &mut validator.module.nav_trans_in_msg,
            &planner.module.lambert_problem_out_msg => &mut validator.module.lambert_problem_in_msg,
            &solver.module.lambert_performance_out_msg => &mut validator.module.lambert_performance_in_msg,
            &solver.module.lambert_solution_out_msg => &mut validator.module.lambert_solution_in_msg,
            &spacecraft.state_out => truth_csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &navigation.translation_out_msg => truth_csv.add_source::<NavigationTranslationMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_bn_n_m", "nav_r_BN_N_m")
                        .into_iter()
                        .chain(vector_columns("velocity_bn_n_mps", "nav_v_BN_N_m_per_s")),
                ),
            ),
            &spacecraft.state_out => &mut truth.input,
            &navigation.translation_out_msg => &mut navigation_history.input,
            &planner.module.lambert_problem_out_msg => fsw_csv.module.add_source::<LambertProblemMsg>(
                CsvSourceConfig::columns(vector_columns("r1_n_m", "r1_N_m")),
            ),
            &solver.module.lambert_solution_out_msg => fsw_csv.module.add_source::<LambertSolutionMsg>(
                CsvSourceConfig::columns(vector_columns("first.v1_n_mps", "v1_N_m_per_s")),
            ),
            &solver.module.lambert_performance_out_msg => fsw_csv.module.add_source::<LambertPerformanceMsg>(
                CsvSourceConfig::columns([
                    ("first.x", "x"),
                    ("first.num_iter", "numIter"),
                    ("first.err_x", "errX"),
                ]),
            ),
            &validator.module.dv_burn_cmd_out_msg => fsw_csv.module.add_source::<DvBurnCommandMsg>(
                CsvSourceConfig::columns(vector_columns("dv_inertial_cmd_mps", "dvInrtlCmd_m_per_s")),
            ),
            &validator.module.lambert_validator_out_msg => fsw_csv.module.add_source::<LambertValidatorMsg>(
                CsvSourceConfig::columns([(
                    "failed_dv_solution_convergence",
                    "failedDvSolutionConvergence",
                )]),
            ),
            &validator.module.dv_burn_cmd_out_msg => &mut dv_commands.module.input,
            &validator.module.lambert_validator_out_msg => &mut validator_history.module.input,
        );
        schedule! { simulation,
            "bskSat" => &mut spacecraft, period=sim_step_nanos, priority=100;
            "SimpleNav" => &mut navigation, period=sim_step_nanos, priority=99;
            "lambertPlanner" => &mut planner, period=fsw_step_nanos, priority=98;
            "lambertSolver" => &mut solver, period=fsw_step_nanos, priority=97;
            "lambertValidator" => &mut validator, period=fsw_step_nanos, priority=96;
            "truth_csv" => &mut truth_csv, period=sim_step_nanos, priority=0;
            "truth" => &mut truth, period=sim_step_nanos, priority=0;
            "navigation_history" => &mut navigation_history, period=sim_step_nanos, priority=0;
            "fsw_csv" => &mut fsw_csv, period=fsw_step_nanos, priority=0;
            "dv_commands" => &mut dv_commands, period=fsw_step_nanos, priority=0;
            "validator_history" => &mut validator_history, period=fsw_step_nanos, priority=0;
        }
        run_until(&mut simulation, sec2nano(maneuver_time_s));

        // The burn, upstream `velRef.setState(vm_N + dv_N)` with the last
        // commanded delta-v, then `fswProcess.disableAllTasks()`.
        hub.set_velocity_mps(hub.velocity_mps() + dv_command.read().dv_inertial_cmd_mps);
        fsw.disable();
        run_until(&mut simulation, sec2nano(final_time_s));
    }
    truth_csv.flush().expect("flush scenario CSV");
    fsw_csv.module.flush().expect("flush scenario CSV");

    Results {
        truth: truth.into_samples(),
        navigation: navigation_history.into_samples(),
        dv_commands: dv_commands.module.into_samples(),
        validator: validator_history.module.into_samples(),
        target_m,
        maneuver_time_s,
        final_time_s,
    }
}

fn main() {
    let results = run();
    let (_, last) = results.truth.last().expect("samples");
    let (_, dv) = results.dv_commands.last().expect("FSW samples");
    println!(
        "wrote scenarioLambertSolver.csv: burn of {:.3} m/s at {:.0} s, {:.1} m from the target at {:.0} s",
        dv.dv_inertial_cmd_mps.norm(),
        results.maneuver_time_s,
        (last.position_m - results.target_m).norm(),
        results.final_time_s
    );
}

#[cfg(test)]
mod tests {
    use super::{MAX_DISTANCE_TARGET_M, run};
    use crate::orbit_tools::sec2nano;

    /// No upstream test. The validator accepted the burn (no failure flags
    /// at the last FSW update), and the spacecraft reaches the target as
    /// closely as the navigation errors allow: the burn is planned from the
    /// navigation state, so its error at the maneuver, a velocity error
    /// drifting over the transfer time and a position error, adds to the
    /// validator's `maxDistanceTarget`. The errors stay within upstream's
    /// `walkBounds` (10 m, 1 m/s).
    #[test]
    fn transfer_reaches_the_target() {
        let results = run();
        let (time, last) = results.truth.last().expect("samples");
        assert_eq!(*time, sec2nano(results.final_time_s));

        let maneuver_nanos = sec2nano(results.maneuver_time_s);
        let at_maneuver = |nanos: u64| nanos <= maneuver_nanos;
        let (_, truth) = results
            .truth
            .iter()
            .rfind(|(nanos, _)| at_maneuver(*nanos))
            .expect("truth at the maneuver");
        let (_, navigation) = results
            .navigation
            .iter()
            .rfind(|(nanos, _)| at_maneuver(*nanos))
            .expect("navigation at the maneuver");
        let position_error = (navigation.position_bn_n_m - truth.position_m).norm();
        let velocity_error = (navigation.velocity_bn_n_mps - truth.velocity_mps).norm();
        assert!(position_error > 0.0 && position_error <= 10.0 * 3.0_f64.sqrt());
        assert!(velocity_error > 0.0 && velocity_error <= 3.0_f64.sqrt());

        let transfer_s = results.final_time_s - results.maneuver_time_s;
        let miss = (last.position_m - results.target_m).norm();
        let allowed = MAX_DISTANCE_TARGET_M + 3.0 * (velocity_error * transfer_s + position_error);
        assert!(
            miss < allowed,
            "missed the target by {miss} m (navigation errors {position_error} m, \
             {velocity_error} m/s; allowed {allowed} m)"
        );

        let (_, validator) = results.validator.last().expect("FSW samples");
        assert!(!validator.failed_valid_lambert);
        assert!(!validator.failed_dv_solution_convergence);
        assert!(!validator.failed_distance_target_constraint);
        assert!(!validator.failed_orbit_radius_constraint);
        let (_, dv) = results.dv_commands.last().expect("FSW samples");
        assert!(dv.dv_inertial_cmd_mps.norm() > 1.0);
        // The FSW recorders stop with the FSW task at the maneuver.
        let (last_fsw_time, _) = results.dv_commands.last().unwrap();
        assert!(*last_fsw_time <= sec2nano(results.maneuver_time_s));
    }
}
