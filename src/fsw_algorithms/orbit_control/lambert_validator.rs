//! Lambert solution check and first delta-v command: port of upstream Basilisk
//! `fswAlgorithms/orbitControl/lambertValidator`.
//!
//! Every update predicts the spacecraft state at the maneuver time by
//! propagating its navigated state under point-mass gravity
//! ([`propagate_two_body_rk4`], 10 s RK4 steps) and takes the delta-v from
//! that velocity to the selected Lambert solution's departure velocity. The
//! delta-v is commanded only if
//!
//! * the Lambert solution is valid, took fewer than 6 root-finder iterations
//!   and a final step below `1e-8`;
//! * `x` and the delta-v have both settled: they changed by less than `1e-2`
//!   and [`LambertValidatorConfig::dv_convergence_tolerance_mps`] since the
//!   previous update;
//! * none of 27 dispersed trajectories violates a constraint. They start from
//!   the maneuver state perturbed by plus and minus each column of the
//!   state-uncertainty square root (Hill frame), each with the delta-v
//!   magnitude shortened and lengthened by its uncertainty, plus the
//!   unperturbed state with the nominal, short and long delta-v; each is
//!   propagated to the final time, must end within
//!   [`LambertValidatorConfig::max_distance_target_m`] of the target and must
//!   stay outside [`LambertValidatorConfig::min_orbit_radius_m`] at every
//!   step.
//!
//! With [`LambertValidatorConfig::ignore_constraint_violations`] a valid
//! solution is commanded regardless, and the dispersions are not propagated.
//! Otherwise the command is zeroed, and [`LambertValidatorMsg`] says which
//! checks failed.
//!
//! As upstream, the module keeps the previous update's `x` and delta-v (for
//! the convergence checks) and the constraint violation counts (which an
//! update with an invalid solution does not recount), and a reset keeps them
//! too. Upstream raises `BasiliskError` when an input is unconnected or the
//! final time precedes the maneuver time (at reset), and when the problem's
//! `mu` is not positive or the navigation time is past the maneuver time (in
//! the update). Here each is logged with `log::error!`; the reset completes,
//! and the update publishes nothing so both outputs keep their previous
//! values. Upstream's `lambertSolutionSpecifier` values other than 1 or 2
//! cannot be expressed in [`LambertSolutionSpecifier`]; its
//! `uncertaintyStates` must be 6x6 and has no default, here it defaults to
//! zero.

use std::fmt;

use nalgebra::{Matrix6, Vector3, Vector6};

use crate::messages::{
    DvBurnCommandMsg, Input, LambertPerformanceMsg, LambertProblemMsg, LambertSolutionMsg,
    LambertSolutionSpecifier, LambertValidatorMsg, NavigationTranslationMsg, Output,
};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::orbital_motion::{hill_frame, propagate_two_body_rk4};
use crate::{Module, SimulationContext};

/// Upstream `timestep` \[s\]: RK4 step of every propagation.
const PROPAGATION_STEP_S: f64 = 10.0;
/// Upstream `maxNumIterLambert`: a solution needing this many root-finder
/// iterations is rejected. The solver usually takes two or three, so this is
/// deliberately tighter than its own limit.
const MAX_NUM_ITER_LAMBERT: usize = 6;
/// Upstream `xToleranceLambert` [-]: largest final root-finder step accepted.
const X_TOLERANCE_LAMBERT: f64 = 1e-8;
/// Upstream `xConvergenceTolerance` [-]: largest change of `x` between
/// updates accepted.
const X_CONVERGENCE_TOLERANCE: f64 = 1e-2;

/// User-settable parameters of [`LambertValidator`] (upstream's setters).
#[derive(Clone, Debug)]
pub struct LambertValidatorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `lambertSolutionSpecifier`: which Lambert solution to use
    /// (default the first).
    pub lambert_solution_specifier: LambertSolutionSpecifier,
    /// Upstream `finalTime` \[s\]: time the target is to be reached.
    pub final_time_s: f64,
    /// Upstream `maneuverTime` \[s\]: time of the burn.
    pub maneuver_time_s: f64,
    /// Upstream `maxDistanceTarget` \[m\]: largest acceptable distance from
    /// the target at the final time.
    pub max_distance_target_m: f64,
    /// Upstream `minOrbitRadius` \[m\]: smallest acceptable orbit radius.
    pub min_orbit_radius_m: f64,
    /// Upstream `uncertaintyStates`: matrix square root of the 6x6 covariance
    /// of the maneuver-time state errors (position \[m\], velocity [m/s]),
    /// Hill-frame components. Each column is one dispersion.
    pub uncertainty_states: Matrix6<f64>,
    /// Upstream `uncertaintyDV` [m/s]: uncertainty of the delta-v magnitude
    /// (default 0.1).
    pub uncertainty_dv_mps: f64,
    /// Upstream `dvConvergenceTolerance` [m/s]: largest change of the delta-v
    /// between updates accepted (default 1e-3).
    pub dv_convergence_tolerance_mps: f64,
    /// Upstream `ignoreConstraintViolations`: command any valid solution.
    pub ignore_constraint_violations: bool,
}

impl Default for LambertValidatorConfig {
    fn default() -> Self {
        Self {
            name: "lambertValidator".to_string(),
            lambert_solution_specifier: LambertSolutionSpecifier::First,
            final_time_s: 0.0,
            maneuver_time_s: 0.0,
            max_distance_target_m: 0.0,
            min_orbit_radius_m: 0.0,
            uncertainty_states: Matrix6::zeros(),
            uncertainty_dv_mps: 0.1,
            dv_convergence_tolerance_mps: 1e-3,
            ignore_constraint_violations: false,
        }
    }
}

/// A problem upstream reports with `BasiliskError`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum LambertValidatorError {
    /// The named input is not connected (at reset).
    InputNotConnected(&'static str),
    /// The final time precedes the maneuver time (at reset).
    FinalTimeBeforeManeuver,
    /// The Lambert problem's `mu` [m^3/s^2] is not positive (in the update).
    NonPositiveMu(f64),
    /// The navigation time tag \[s\] is past the maneuver time (in the
    /// update).
    ManeuverTimePassed(f64),
}

impl fmt::Display for LambertValidatorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputNotConnected(port) => write!(f, "lambertValidator.{port} is not connected"),
            Self::FinalTimeBeforeManeuver => write!(
                f,
                "lambertValidator: Maneuver start time maneuverTime must be before final time \
                 finalTime."
            ),
            Self::NonPositiveMu(mu) => {
                write!(f, "lambertValidator: mu must be positive, not {mu}.")
            }
            Self::ManeuverTimePassed(time_tag_s) => write!(
                f,
                "lambertValidator: current time {time_tag_s} s must be before maneuver time \
                 maneuverTime."
            ),
        }
    }
}

impl std::error::Error for LambertValidatorError {}

/// The delta-v command the Lambert modules publish: `dv_n_mps` starting at
/// `maneuver_time_s`, with the unused rotation set to the arbitrary unit axis
/// x and zero rate, as upstream.
pub(crate) fn lambert_burn_command(
    dv_n_mps: Vector3<f64>,
    maneuver_time_s: f64,
) -> DvBurnCommandMsg {
    DvBurnCommandMsg {
        dv_inertial_cmd_mps: dv_n_mps,
        dv_rot_vec_unit: Vector3::x(),
        dv_rot_vec_mag_radps: 0.0,
        // Upstream `static_cast<uint64_t>(maneuverTime * SEC2NANO)`.
        burn_start_time_nanos: (maneuver_time_s * 1e9) as u64,
    }
}

/// `[r, v]` as one state.
fn state(r: Vector3<f64>, v: Vector3<f64>) -> Vector6<f64> {
    let mut state = Vector6::zeros();
    state.fixed_rows_mut::<3>(0).copy_from(&r);
    state.fixed_rows_mut::<3>(3).copy_from(&v);
    state
}

/// Checks the Lambert solution and commands its delta-v: upstream
/// `LambertValidator`.
#[derive(Clone, Debug)]
pub struct LambertValidator {
    /// Module configuration.
    pub config: LambertValidatorConfig,
    /// Upstream `navTransInMsg`: the spacecraft's navigated inertial state.
    pub nav_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `lambertProblemInMsg`: the problem that was solved (its
    /// target `r2_N` and `mu` are used).
    pub lambert_problem_in_msg: Input<LambertProblemMsg>,
    /// Upstream `lambertSolutionInMsg`: the solver's velocities.
    pub lambert_solution_in_msg: Input<LambertSolutionMsg>,
    /// Upstream `lambertPerformanceInMsg`: the solver's diagnostics.
    pub lambert_performance_in_msg: Input<LambertPerformanceMsg>,
    /// Upstream `dvBurnCmdOutMsg`: the delta-v command, zero unless every
    /// check passed.
    pub dv_burn_cmd_out_msg: Output<DvBurnCommandMsg>,
    /// Upstream `lambertValidatorOutMsg`: which checks failed, and why.
    pub lambert_validator_out_msg: Output<LambertValidatorMsg>,
    /// Upstream `prevLambertSolutionX` [-].
    prev_lambert_solution_x: f64,
    /// Upstream `prevDv_N` [m/s]: the computed delta-v of the previous update
    /// (not the command, which may have been zeroed).
    prev_dv_n_mps: Vector3<f64>,
    /// Upstream `violationsDistanceTarget`.
    violations_distance_target: usize,
    /// Upstream `violationsOrbitRadius`.
    violations_orbit_radius: usize,
}

impl LambertValidator {
    /// Builds the module with unconnected ports and no history.
    pub fn new(config: LambertValidatorConfig) -> Self {
        Self {
            config,
            nav_trans_in_msg: Input::default(),
            lambert_problem_in_msg: Input::default(),
            lambert_solution_in_msg: Input::default(),
            lambert_performance_in_msg: Input::default(),
            dv_burn_cmd_out_msg: Output::default(),
            lambert_validator_out_msg: Output::default(),
            prev_lambert_solution_x: 0.0,
            prev_dv_n_mps: Vector3::zeros(),
            violations_distance_target: 0,
            violations_orbit_radius: 0,
        }
    }

    /// Upstream `Reset`'s checks.
    pub fn configuration_errors(&self) -> Vec<LambertValidatorError> {
        let mut errors: Vec<_> = [
            ("nav_trans_in_msg", self.nav_trans_in_msg.is_connected()),
            (
                "lambert_problem_in_msg",
                self.lambert_problem_in_msg.is_connected(),
            ),
            (
                "lambert_solution_in_msg",
                self.lambert_solution_in_msg.is_connected(),
            ),
            (
                "lambert_performance_in_msg",
                self.lambert_performance_in_msg.is_connected(),
            ),
        ]
        .into_iter()
        .filter(|(_, connected)| !connected)
        .map(|(port, _)| LambertValidatorError::InputNotConnected(port))
        .collect();
        if self.config.final_time_s - self.config.maneuver_time_s < 0.0 {
            errors.push(LambertValidatorError::FinalTimeBeforeManeuver);
        }
        errors
    }

    fn report_configuration_errors(&self) {
        for error in self.configuration_errors() {
            log::error!("{error}");
        }
    }

    /// Both outputs for the current inputs, updating the history; or why
    /// upstream would raise, leaving the history untouched.
    fn validate(
        &mut self,
    ) -> Result<(DvBurnCommandMsg, LambertValidatorMsg), LambertValidatorError> {
        let navigation = self.nav_trans_in_msg.read();
        let problem = self.lambert_problem_in_msg.read();
        let which = self.config.lambert_solution_specifier;
        let solution = *self.lambert_solution_in_msg.read().solution(which);
        let root = *self.lambert_performance_in_msg.read().root(which);

        let mu = problem.mu_m3ps2;
        if mu <= 0.0 {
            return Err(LambertValidatorError::NonPositiveMu(mu));
        }
        let maneuver_time_s = self.config.maneuver_time_s;
        if maneuver_time_s - navigation.time_tag_s < 0.0 {
            return Err(LambertValidatorError::ManeuverTimePassed(
                navigation.time_tag_s,
            ));
        }

        let maneuver_state = *propagate_two_body_rk4(
            mu,
            state(navigation.position_bn_n_m, navigation.velocity_bn_n_mps),
            navigation.time_tag_s,
            maneuver_time_s,
            PROPAGATION_STEP_S,
        )
        .last()
        .expect("propagation keeps the initial state");
        let rm_n: Vector3<f64> = maneuver_state.fixed_rows::<3>(0).into_owned();
        let vm_n: Vector3<f64> = maneuver_state.fixed_rows::<3>(3).into_owned();
        let dv_n = solution.v1_n_mps - vm_n;

        // The dispersions are only worth propagating for a valid solution
        // whose constraints are not ignored anyway.
        if solution.valid && !self.config.ignore_constraint_violations {
            self.count_violations(mu, rm_n, vm_n, dv_n, problem.r2_n_m);
        }

        let x_difference = root.x - self.prev_lambert_solution_x;
        let dv_difference = (dv_n - self.prev_dv_n_mps).norm();
        let violations_distance_target = self.violations_distance_target;
        let violations_orbit_radius = self.violations_orbit_radius;
        let diagnostics = LambertValidatorMsg {
            failed_valid_lambert: !solution.valid,
            failed_num_iterations_lambert: root.num_iter >= MAX_NUM_ITER_LAMBERT,
            failed_x_tolerance_lambert: root.err_x.abs() >= X_TOLERANCE_LAMBERT,
            failed_x_solution_convergence: x_difference.abs() >= X_CONVERGENCE_TOLERANCE,
            failed_dv_solution_convergence: dv_difference.abs()
                >= self.config.dv_convergence_tolerance_mps,
            failed_distance_target_constraint: violations_distance_target != 0,
            failed_orbit_radius_constraint: violations_orbit_radius != 0,
            x_solution_difference: x_difference,
            dv_solution_difference_mps: dv_difference,
            violations_distance_target,
            violations_orbit_radius,
            dv_n_mps: dv_n,
        };

        // Upstream `checkPerformance`: every check written with the
        // complementary strict comparison, so a NaN fails it.
        let converged = root.num_iter < MAX_NUM_ITER_LAMBERT
            && root.err_x.abs() < X_TOLERANCE_LAMBERT
            && x_difference.abs() < X_CONVERGENCE_TOLERANCE
            && dv_difference.abs() < self.config.dv_convergence_tolerance_mps
            && violations_distance_target == 0
            && violations_orbit_radius == 0;
        let command = if solution.valid && (converged || self.config.ignore_constraint_violations) {
            lambert_burn_command(dv_n, maneuver_time_s)
        } else {
            DvBurnCommandMsg::default()
        };

        self.prev_lambert_solution_x = root.x;
        self.prev_dv_n_mps = dv_n;
        Ok((command, diagnostics))
    }

    /// The 27 dispersed maneuver-time states (upstream `getInitialStates`),
    /// inertial: the state-uncertainty columns are applied in the Hill frame of
    /// the predicted maneuver state, and the delta-v magnitude is shortened and
    /// lengthened by its uncertainty along the delta-v direction.
    fn initial_states(
        &self,
        rm_n: Vector3<f64>,
        vm_n: Vector3<f64>,
        dv_n: Vector3<f64>,
    ) -> Vec<Vector6<f64>> {
        let dcm_hn = hill_frame(rm_n, vm_n);
        let dcm_nh = dcm_hn.transpose();
        let nominal_h = state(dcm_hn * rm_n, dcm_hn * vm_n);
        let dv_h = dcm_hn * dv_n;
        let dv_hat_h = normalize_or_zero(dv_h);
        let dv_spread = dv_hat_h * self.config.uncertainty_dv_mps;
        let to_inertial = |x_h: Vector6<f64>| {
            state(
                dcm_nh * x_h.fixed_rows::<3>(0),
                dcm_nh * x_h.fixed_rows::<3>(3),
            )
        };
        let with_burn = |x_h: Vector6<f64>, burn_h: Vector3<f64>| {
            let mut x_h = x_h;
            let mut velocity = x_h.fixed_rows_mut::<3>(3);
            velocity += burn_h;
            to_inertial(x_h)
        };

        // Upstream orders them as short burns, long burns, then the three
        // unperturbed ones; only the counts they produce are used.
        let psqrt = &self.config.uncertainty_states;
        let dispersions = (0..6)
            .flat_map(|column| [1.0, -1.0].map(|sign| nominal_h + psqrt.column(column) * sign));
        dispersions
            .flat_map(|x_h| [dv_h - dv_spread, dv_h + dv_spread].map(|burn| with_burn(x_h, burn)))
            .chain(
                [dv_h, dv_h - dv_spread, dv_h + dv_spread].map(|burn| with_burn(nominal_h, burn)),
            )
            .collect()
    }

    /// Propagates every dispersed state from the maneuver to the final time
    /// and counts the constraint violations (upstream `countViolations` and
    /// `checkConstraintViolations`).
    fn count_violations(
        &mut self,
        mu: f64,
        rm_n: Vector3<f64>,
        vm_n: Vector3<f64>,
        dv_n: Vector3<f64>,
        r_tn_n: Vector3<f64>,
    ) {
        self.violations_distance_target = 0;
        self.violations_orbit_radius = 0;
        for initial in self.initial_states(rm_n, vm_n, dv_n) {
            let trajectory = propagate_two_body_rk4(
                mu,
                initial,
                self.config.maneuver_time_s,
                self.config.final_time_s,
                PROPAGATION_STEP_S,
            );
            let position = |x: &Vector6<f64>| x.fixed_rows::<3>(0).into_owned();
            let final_position = position(trajectory.last().expect("initial state kept"));
            if (final_position - r_tn_n).norm() > self.config.max_distance_target_m {
                self.violations_distance_target += 1;
            }
            if trajectory
                .iter()
                .any(|x| position(x).norm() < self.config.min_orbit_radius_m)
            {
                self.violations_orbit_radius += 1;
            }
        }
    }
}

impl Module for LambertValidator {
    fn init(&mut self) {
        self.dv_burn_cmd_out_msg.write(DvBurnCommandMsg::default());
        self.lambert_validator_out_msg
            .write(LambertValidatorMsg::default());
        self.report_configuration_errors();
    }

    /// Upstream `Reset` only checks; the history is kept.
    fn reset(&mut self, _context: &SimulationContext) {
        self.report_configuration_errors();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.validate() {
            Ok((command, diagnostics)) => {
                self.dv_burn_cmd_out_msg.write(command);
                self.lambert_validator_out_msg.write(diagnostics);
            }
            // Upstream throws here, before writing: the outputs keep their
            // previous values.
            Err(error) => log::error!("{error}"),
        }
    }
}

// Upstream test inputs are copied verbatim, digits included.
#[allow(clippy::excessive_precision)]
#[cfg(test)]
mod tests;
