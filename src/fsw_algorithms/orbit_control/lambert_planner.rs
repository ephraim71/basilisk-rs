//! Lambert problem setup: port of upstream Basilisk
//! `fswAlgorithms/orbitControl/lambertPlanner`.
//!
//! Every update predicts where the spacecraft will be at the maneuver time, by
//! propagating its navigated state under point-mass gravity with fixed-step
//! RK4 ([`propagate_two_body_rk4`], 10 s steps), and publishes the Lambert
//! problem from there to the configured target: the transfer time is the
//! final time minus the maneuver time, and the revolution count, solver method
//! and transfer direction come from the configuration.
//!
//! Upstream raises `BasiliskError` when the final time precedes the maneuver
//! time (at reset), when `mu` is set negative (in its setter), and when the
//! navigation time tag is past the maneuver time (in the update). Here each is
//! logged with `log::error!`: the first two at reset, which then completes,
//! and the last in the update, which then publishes nothing so the output
//! keeps its previous value. Upstream's unknown transfer-direction integers
//! cannot be expressed in [`LambertTransferDirection`].

use std::fmt;

use nalgebra::{Vector3, Vector6};

use crate::messages::{
    Input, LambertProblemMsg, LambertSolverMethod, LambertTransferDirection,
    NavigationTranslationMsg, Output,
};
use crate::utilities::orbital_motion::propagate_two_body_rk4;
use crate::{Module, SimulationContext};

/// Upstream RK4 step \[s\] of the propagation to the maneuver time.
const PROPAGATION_STEP_S: f64 = 10.0;

/// User-settable parameters of [`LambertPlanner`] (upstream's setters).
#[derive(Clone, Debug)]
pub struct LambertPlannerConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `r_TN_N` \[m\]: target position at the final time, inertial
    /// components.
    pub r_tn_n_m: Vector3<f64>,
    /// Upstream `finalTime` \[s\]: time the target is to be reached.
    pub final_time_s: f64,
    /// Upstream `maneuverTime` \[s\]: time of the first burn.
    pub maneuver_time_s: f64,
    /// Upstream `mu` [m^3/s^2]: gravitational parameter of the central body;
    /// must not be negative.
    pub mu_m3ps2: f64,
    /// Upstream `numRevolutions` [-]: complete revolutions of the transfer.
    pub num_revolutions: usize,
    /// Upstream `solverMethod`, chosen with `useSolverIzzoMethod` (the
    /// default) or `useSolverGoodingMethod`.
    pub solver_method: LambertSolverMethod,
    /// Upstream `transferDirection` (default prograde).
    pub transfer_direction: LambertTransferDirection,
}

impl Default for LambertPlannerConfig {
    fn default() -> Self {
        Self {
            name: "lambertPlanner".to_string(),
            r_tn_n_m: Vector3::zeros(),
            final_time_s: 0.0,
            maneuver_time_s: 0.0,
            mu_m3ps2: 0.0,
            num_revolutions: 0,
            solver_method: LambertSolverMethod::Izzo,
            transfer_direction: LambertTransferDirection::Prograde,
        }
    }
}

/// A problem upstream reports with `BasiliskError`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LambertPlannerError {
    /// The navigation input is not connected (at reset).
    InputNotConnected,
    /// The final time precedes the maneuver time (at reset).
    FinalTimeBeforeManeuver,
    /// `mu` is negative (upstream `setMu`).
    NegativeMu,
    /// The navigation time tag \[s\] is past the maneuver time (in the
    /// update).
    ManeuverTimePassed(f64),
}

impl fmt::Display for LambertPlannerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputNotConnected => {
                write!(f, "lambertPlanner.nav_trans_in_msg is not connected")
            }
            Self::FinalTimeBeforeManeuver => write!(
                f,
                "lambertPlanner: Maneuver start time maneuverTime must be before final time \
                 finalTime."
            ),
            Self::NegativeMu => write!(f, "lambertPlanner: mu must be positive."),
            Self::ManeuverTimePassed(time_tag_s) => write!(
                f,
                "lambertPlanner: current time {time_tag_s} s must be before maneuver time \
                 maneuverTime."
            ),
        }
    }
}

impl std::error::Error for LambertPlannerError {}

/// Poses the Lambert problem from the predicted maneuver state to the target:
/// upstream `LambertPlanner`.
#[derive(Clone, Debug)]
pub struct LambertPlanner {
    /// Module configuration.
    pub config: LambertPlannerConfig,
    /// Upstream `navTransInMsg`: the spacecraft's navigated inertial state.
    pub nav_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `lambertProblemOutMsg`: the problem for `lambertSolver`.
    pub lambert_problem_out_msg: Output<LambertProblemMsg>,
}

impl LambertPlanner {
    /// Builds the module with unconnected ports.
    pub fn new(config: LambertPlannerConfig) -> Self {
        Self {
            config,
            nav_trans_in_msg: Input::default(),
            lambert_problem_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset`'s checks, plus `setMu`'s.
    pub fn configuration_errors(&self) -> Vec<LambertPlannerError> {
        let mut errors = Vec::new();
        if !self.nav_trans_in_msg.is_connected() {
            errors.push(LambertPlannerError::InputNotConnected);
        }
        if self.config.final_time_s - self.config.maneuver_time_s < 0.0 {
            errors.push(LambertPlannerError::FinalTimeBeforeManeuver);
        }
        if self.config.mu_m3ps2 < 0.0 {
            errors.push(LambertPlannerError::NegativeMu);
        }
        errors
    }

    fn report_configuration_errors(&self) {
        for error in self.configuration_errors() {
            log::error!("{error}");
        }
    }

    /// The problem for the current navigation state.
    fn plan(&self) -> Result<LambertProblemMsg, LambertPlannerError> {
        let config = &self.config;
        let navigation = self.nav_trans_in_msg.read();
        if config.maneuver_time_s - navigation.time_tag_s < 0.0 {
            return Err(LambertPlannerError::ManeuverTimePassed(
                navigation.time_tag_s,
            ));
        }
        let mut state = Vector6::zeros();
        state
            .fixed_rows_mut::<3>(0)
            .copy_from(&navigation.position_bn_n_m);
        state
            .fixed_rows_mut::<3>(3)
            .copy_from(&navigation.velocity_bn_n_mps);
        let states = propagate_two_body_rk4(
            config.mu_m3ps2,
            state,
            navigation.time_tag_s,
            config.maneuver_time_s,
            PROPAGATION_STEP_S,
        );
        let maneuver_state = states.last().expect("propagation keeps the initial state");

        Ok(LambertProblemMsg {
            solver_method: config.solver_method,
            r1_n_m: maneuver_state.fixed_rows::<3>(0).into_owned(),
            r2_n_m: config.r_tn_n_m,
            transfer_time_s: config.final_time_s - config.maneuver_time_s,
            mu_m3ps2: config.mu_m3ps2,
            num_revolutions: config.num_revolutions,
            transfer_direction: config.transfer_direction,
        })
    }
}

impl Module for LambertPlanner {
    fn init(&mut self) {
        self.lambert_problem_out_msg
            .write(LambertProblemMsg::default());
        self.report_configuration_errors();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.report_configuration_errors();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.plan() {
            Ok(problem) => self.lambert_problem_out_msg.write(problem),
            // Upstream throws here, before writing: the output keeps its
            // previous value.
            Err(error) => log::error!("{error}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::TAU;

    use nalgebra::Vector3;

    use super::{LambertPlanner, LambertPlannerConfig, LambertPlannerError};
    use crate::Module;
    use crate::fsw_algorithms::orbit_control::lambert_solver::{
        LambertSolver, LambertSolverConfig,
    };
    use crate::fsw_algorithms::orbit_control::test_support::{
        METHODS, conic_state, context, direction_sign, elements,
    };
    use crate::messages::{
        LambertProblemMsg, LambertSolverMethod, LambertTransferDirection, NavigationTranslationMsg,
        Output,
    };
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::orbital_motion::{
        ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly,
        hyperbolic_to_mean_hyperbolic_anomaly, hyperbolic_to_true_anomaly,
        mean_hyperbolic_to_hyperbolic_anomaly, mean_to_eccentric_anomaly,
        true_to_eccentric_anomaly, true_to_hyperbolic_anomaly,
    };

    fn navigation(time_tag_s: f64, r: Vector3<f64>, v: Vector3<f64>) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            time_tag_s,
            position_bn_n_m: r,
            velocity_bn_n_mps: v,
            ..Default::default()
        }
    }

    /// Upstream `test_lambertPlanner`: for zero to four revolutions, maneuver
    /// and final times and four conics, the published problem matches the
    /// inputs, with `r1_N` against Kepler's equation from t0 = 100 s to the
    /// maneuver time (rtol 1e-2). A maneuver after the final time must be
    /// reported at reset, where upstream raises.
    #[test]
    fn matches_upstream_lambert_planner_test() {
        let accuracy = 1e-2;
        let mu = 3.986004418e14;
        let t0 = 100.0;
        let target = Vector3::new(7000.0 * 1000.0, -1000.0 * 1000.0, 2000.0 * 1000.0);
        for revolutions in [0, 1, 4] {
            for tm in [1e3, 2e3] {
                for tf in [1e3, 4e3] {
                    for e in [0.0, 0.05, 1.0, 1.2] {
                        let what = format!("rev={revolutions}, tm={tm}, tf={tf}, e={e}");
                        let radius = 10000.0 * 1000.0;
                        let a = if e < 1.0 { radius } else { -radius };
                        let oe1 = elements(a, e, 10.0 * D2R, 20.0 * D2R, 30.0 * D2R, 40.0 * D2R);
                        let (r0, v0) = conic_state(mu, &oe1);
                        let f1 = oe1.true_anomaly_rad;
                        let f2 = if e < 1.0 {
                            let m1 = eccentric_to_mean_anomaly(true_to_eccentric_anomaly(f1, e), e);
                            let n = (mu / a.powf(3.0)).sqrt();
                            let m2 = m1 + n * (tm - t0);
                            eccentric_to_true_anomaly(mean_to_eccentric_anomaly(m2, e), e)
                        } else if e == 1.0 {
                            let d1 = (f1 / 2.0).tan();
                            let m1 = d1 + 1.0 / 3.0 * d1.powf(3.0);
                            let n = (mu / (2.0 * (-a).powf(3.0))).sqrt();
                            let m2 = m1 + n * (tm - t0);
                            let big_a = 3.0 / 2.0 * m2;
                            let big_b = (big_a + (big_a * big_a + 1.0).sqrt()).powf(1.0 / 3.0);
                            2.0 * (big_b - 1.0 / big_b).atan()
                        } else {
                            let n1 = hyperbolic_to_mean_hyperbolic_anomaly(
                                true_to_hyperbolic_anomaly(f1, e),
                                e,
                            );
                            let n = (mu / (-a).powf(3.0)).sqrt();
                            let n2 = n1 + n * (tm - t0);
                            hyperbolic_to_true_anomaly(
                                mean_hyperbolic_to_hyperbolic_anomaly(n2, e),
                                e,
                            )
                        };
                        let (rm, _) = conic_state(
                            mu,
                            &ClassicElements {
                                true_anomaly_rad: f2,
                                ..oe1
                            },
                        );

                        let mut module = LambertPlanner::new(LambertPlannerConfig {
                            r_tn_n_m: target,
                            final_time_s: tf,
                            maneuver_time_s: tm,
                            mu_m3ps2: mu,
                            num_revolutions: revolutions,
                            solver_method: LambertSolverMethod::Izzo,
                            ..Default::default()
                        });
                        let source = Output::new(navigation(t0, r0, v0));
                        source.connect_to(&mut module.nav_trans_in_msg);
                        let errors = module.configuration_errors();
                        if tm > tf {
                            assert_eq!(errors, [LambertPlannerError::FinalTimeBeforeManeuver]);
                            continue;
                        }
                        assert!(errors.is_empty(), "{what}: {errors:?}");
                        module.init();
                        module.update(&context(0));
                        let output = module.lambert_problem_out_msg.read();

                        assert_eq!(output.solver_method, LambertSolverMethod::Izzo, "{what}");
                        for axis in 0..3 {
                            let close = |actual: f64, truth: f64| {
                                (actual - truth).abs() <= accuracy * truth.abs()
                            };
                            assert!(
                                close(output.r1_n_m[axis], rm[axis]),
                                "{what}: r1_N {output:?} vs {rm}"
                            );
                            assert!(close(output.r2_n_m[axis], target[axis]), "{what}: r2_N");
                        }
                        assert!(
                            (output.transfer_time_s - (tf - tm)).abs() <= accuracy,
                            "{what}"
                        );
                        assert!((output.mu_m3ps2 - mu).abs() <= accuracy, "{what}");
                        assert_eq!(output.num_revolutions, revolutions, "{what}");
                    }
                }
            }
        }
    }

    /// Upstream `test_transfer_direction_reaches_solver`: explicit directions
    /// pass through the planner and the solver to the known circular
    /// transfers. The maneuver is at the navigation time, so no propagation
    /// happens.
    #[test]
    fn transfer_direction_reaches_solver() {
        let mu: f64 = 3.986004360e14;
        let radius: f64 = 8.0e6;
        let speed = (mu / radius).sqrt();
        let period = TAU * (radius * radius * radius / mu).sqrt();
        let r1 = Vector3::new(radius, 0.0, 0.0);
        let r2 = Vector3::new(0.0, radius, 0.0);
        for method in METHODS {
            for revolutions in [0, 1] {
                let source = Output::new(navigation(0.0, r1, Vector3::new(0.0, speed, 0.0)));
                let mut planner = LambertPlanner::new(LambertPlannerConfig {
                    mu_m3ps2: mu,
                    r_tn_n_m: r2,
                    num_revolutions: revolutions,
                    solver_method: method,
                    ..Default::default()
                });
                source.connect_to(&mut planner.nav_trans_in_msg);
                let mut solver = LambertSolver::new(LambertSolverConfig::default());
                planner
                    .lambert_problem_out_msg
                    .connect_to(&mut solver.lambert_problem_in_msg);
                assert_eq!(
                    planner.config.transfer_direction,
                    LambertTransferDirection::Prograde
                );
                for direction in [
                    None,
                    Some(LambertTransferDirection::Retrograde),
                    Some(LambertTransferDirection::Prograde),
                ] {
                    if let Some(direction) = direction {
                        planner.config.transfer_direction = direction;
                    }
                    let expected_direction = direction.unwrap_or_default();
                    let what = format!("{method:?} {revolutions} {expected_direction:?}");
                    let sign = direction_sign(expected_direction);
                    let fraction = match expected_direction {
                        LambertTransferDirection::Retrograde => 0.75,
                        LambertTransferDirection::Prograde => 0.25,
                    };
                    planner.config.final_time_s = (revolutions as f64 + fraction) * period;
                    planner.reset(&context(0));
                    planner.update(&context(0));
                    let problem = planner.lambert_problem_out_msg.read();
                    assert_eq!(problem.transfer_direction, expected_direction, "{what}");
                    assert_eq!(problem.solver_method, method, "{what}");
                    solver.reset(&context(0));
                    solver.update(&context(0));
                    let solution = solver.lambert_solution_out_msg.read();
                    assert!(solution.first.valid, "{what}");
                    assert_eq!(solution.second.valid, revolutions > 0, "{what}");
                    let mut branches = vec![solution.first];
                    if revolutions > 0 {
                        branches.push(solution.second);
                    }
                    for branch in &branches {
                        assert!(sign * r1.cross(&branch.v1_n_mps).z > 0.0, "{what}");
                    }
                    let expected_v1 = Vector3::new(0.0, sign * speed, 0.0);
                    let expected_v2 = Vector3::new(-sign * speed, 0.0, 0.0);
                    let closest = branches
                        .iter()
                        .min_by(|a, b| {
                            (a.v1_n_mps - expected_v1)
                                .norm()
                                .total_cmp(&(b.v1_n_mps - expected_v1).norm())
                        })
                        .expect("at least one branch");
                    assert!((closest.v1_n_mps - expected_v1).amax() <= 1e-5, "{what}");
                    assert!((closest.v2_n_mps - expected_v2).amax() <= 1e-5, "{what}");
                }
            }
        }
    }

    /// gtest `DefaultsToPrograde` (planner half).
    #[test]
    fn defaults_to_izzo_and_prograde() {
        let config = LambertPlannerConfig::default();
        assert_eq!(
            config.transfer_direction,
            LambertTransferDirection::Prograde
        );
        assert_eq!(config.solver_method, LambertSolverMethod::Izzo);
    }

    /// The update-time check upstream raises from `readMessages`: a
    /// navigation time past the maneuver time publishes nothing.
    #[test]
    fn navigation_after_the_maneuver_publishes_nothing() {
        let mut module = LambertPlanner::new(LambertPlannerConfig {
            maneuver_time_s: 50.0,
            final_time_s: 100.0,
            mu_m3ps2: 3.986004418e14,
            ..Default::default()
        });
        let source = Output::new(navigation(
            0.0,
            Vector3::new(7e6, 0.0, 0.0),
            Vector3::new(0.0, 7.5e3, 0.0),
        ));
        source.connect_to(&mut module.nav_trans_in_msg);
        module.init();
        module.update(&context(0));
        let published = module.lambert_problem_out_msg.read();
        assert_ne!(published, LambertProblemMsg::default());

        source.write(navigation(
            60.0,
            Vector3::new(7e6, 0.0, 0.0),
            Vector3::zeros(),
        ));
        assert_eq!(
            module.plan(),
            Err(LambertPlannerError::ManeuverTimePassed(60.0))
        );
        module.update(&context(0));
        assert_eq!(module.lambert_problem_out_msg.read(), published);
    }

    /// Upstream `setMu`'s rejection of a negative `mu`, and `Reset`'s of an
    /// unconnected input, both reported at reset here.
    #[test]
    fn reports_negative_mu_and_a_missing_input() {
        let module = LambertPlanner::new(LambertPlannerConfig {
            mu_m3ps2: -1.0,
            ..Default::default()
        });
        assert_eq!(
            module.configuration_errors(),
            [
                LambertPlannerError::InputNotConnected,
                LambertPlannerError::NegativeMu
            ]
        );
    }

    /// Not upstream: the shared RK4 propagation follows the circular orbit it
    /// starts on, forwards and backwards, landing exactly on the end time
    /// with a shortened last step.
    #[test]
    fn rk4_propagation_follows_a_circular_orbit() {
        use crate::utilities::orbital_motion::propagate_two_body_rk4;
        use nalgebra::Vector6;

        let mu: f64 = 3.986004418e14;
        let radius: f64 = 7.0e6;
        let speed = (mu / radius).sqrt();
        let rate = speed / radius;
        let state = Vector6::new(radius, 0.0, 0.0, 0.0, speed, 0.0);
        for (t0, tf) in [(0.0, 1234.5), (1234.5, 0.0)] {
            let states = propagate_two_body_rk4(mu, state, t0, tf, 10.0);
            // ceil(123.45) = 124 steps after the initial state.
            assert_eq!(states.len(), 124 + 1);
            let angle = rate * (tf - t0);
            let last = states.last().unwrap();
            let expected = Vector3::new(angle.cos(), angle.sin(), 0.0) * radius;
            // RK4 at 10 s steps leaves about a millimetre after 124 steps.
            let error = (last.fixed_rows::<3>(0) - expected).norm();
            assert!(error < 1e-2, "{t0} -> {tf}: {error}");
        }
        assert_eq!(
            propagate_two_body_rk4(mu, state, 5.0, 5.0, 10.0),
            vec![state]
        );
    }
}
