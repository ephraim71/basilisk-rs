//! Lambert's problem solver: port of upstream Basilisk
//! `fswAlgorithms/orbitControl/lambertSolver`.
//!
//! Every update reads a [`LambertProblemMsg`], solves it with Izzo's or
//! Gooding's algorithm ([`LambertProblem::solve`], see
//! [`crate::utilities::lambert`]) and publishes the endpoint velocities of both
//! solutions and the root finder's diagnostics. A solution that does not
//! exist (the endpoints are too close to aligned, or a multi-revolution flight
//! time is below the minimum) or does not converge is published zeroed and
//! flagged invalid, independently of the other one. Nothing carries over
//! between updates.
//!
//! Upstream raises `BasiliskError` from the update when the problem is
//! malformed: `mu` or the flight time not finite and positive, or an endpoint
//! with a non-finite component or a zero or non-finite norm. Here the error is
//! logged with `log::error!` and nothing is published, so both outputs keep
//! their previous values, as upstream's do after the exception. Negative
//! revolution counts and unknown method or direction codes, which upstream
//! also rejects, cannot be expressed in a [`LambertProblemMsg`].

use std::f64::consts::PI;
use std::fmt;

use crate::messages::{
    Input, LambertPerformanceMsg, LambertProblemMsg, LambertRootPerformance, LambertSolutionMsg,
    LambertTransferVelocities, Output,
};
use crate::utilities::lambert::{LambertBranch, LambertInputError, LambertProblem};
use crate::{Module, SimulationContext};

/// Upstream default `alignmentThreshold` \[deg\].
const DEFAULT_ALIGNMENT_THRESHOLD_DEG: f64 = 1.0;

/// Configuration of [`LambertSolver`]. The alignment threshold, which upstream
/// validates in its setter, is set with
/// [`LambertSolver::set_alignment_threshold_deg`].
#[derive(Clone, Debug)]
pub struct LambertSolverConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for LambertSolverConfig {
    fn default() -> Self {
        Self {
            name: "lambertSolver".to_string(),
        }
    }
}

/// An alignment threshold outside `(0, 90)` degrees, which upstream
/// `setAlignmentThreshold` rejects with `BasiliskError`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InvalidAlignmentThreshold(pub f64);

impl fmt::Display for InvalidAlignmentThreshold {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "lambertSolver: alignmentThreshold must be finite and strictly between 0 and 90 \
             degrees, not {}",
            self.0
        )
    }
}

impl std::error::Error for InvalidAlignmentThreshold {}

/// Solves the Lambert problem it is given: upstream `LambertSolver`.
#[derive(Clone, Debug)]
pub struct LambertSolver {
    /// Module configuration.
    pub config: LambertSolverConfig,
    /// Upstream `lambertProblemInMsg`: the problem to solve.
    pub lambert_problem_in_msg: Input<LambertProblemMsg>,
    /// Upstream `lambertSolutionOutMsg`: velocities of both solutions.
    pub lambert_solution_out_msg: Output<LambertSolutionMsg>,
    /// Upstream `lambertPerformanceOutMsg`: root-finder diagnostics of both
    /// solutions.
    pub lambert_performance_out_msg: Output<LambertPerformanceMsg>,
    /// Upstream `alignmentThreshold` \[deg\].
    alignment_threshold_deg: f64,
}

impl LambertSolver {
    /// Builds the module with unconnected ports and the default alignment
    /// threshold of 1 degree.
    pub fn new(config: LambertSolverConfig) -> Self {
        Self {
            config,
            lambert_problem_in_msg: Input::default(),
            lambert_solution_out_msg: Output::default(),
            lambert_performance_out_msg: Output::default(),
            alignment_threshold_deg: DEFAULT_ALIGNMENT_THRESHOLD_DEG,
        }
    }

    /// Upstream `setAlignmentThreshold` \[deg\]: the smallest angle between
    /// the endpoint directions and parallel or antiparallel for which a
    /// transfer is attempted.
    ///
    /// # Errors
    ///
    /// A value that is not finite and strictly between 0 and 90 degrees is
    /// rejected, and the previous threshold kept.
    pub fn set_alignment_threshold_deg(
        &mut self,
        value_deg: f64,
    ) -> Result<(), InvalidAlignmentThreshold> {
        if !(value_deg.is_finite() && value_deg > 0.0 && value_deg < 90.0) {
            return Err(InvalidAlignmentThreshold(value_deg));
        }
        self.alignment_threshold_deg = value_deg;
        Ok(())
    }

    /// Upstream `getlignmentThreshold` \[deg\].
    pub fn alignment_threshold_deg(&self) -> f64 {
        self.alignment_threshold_deg
    }

    /// Upstream `Reset`: the module keeps no state, so a reset only reports a
    /// missing input.
    fn check_connections(&self) {
        if !self.lambert_problem_in_msg.is_connected() {
            log::error!("lambertSolver.lambert_problem_in_msg is not connected");
        }
    }

    /// Both outputs for the current problem, or why upstream would raise.
    fn solve(&self) -> Result<(LambertSolutionMsg, LambertPerformanceMsg), LambertInputError> {
        let problem = self.lambert_problem_in_msg.read();
        let solution = LambertProblem {
            r1: problem.r1_n_m,
            r2: problem.r2_n_m,
            transfer_time: problem.transfer_time_s,
            mu: problem.mu_m3ps2,
            revolutions: problem.num_revolutions,
            method: problem.solver_method,
            direction: problem.transfer_direction,
        }
        // Upstream converts with `* M_PI / 180.`; kept for the same rounding.
        .solve(self.alignment_threshold_deg * PI / 180.0)?;

        // A missing solution keeps its zeroed slots and its cleared flag.
        let velocities = |branch: Option<LambertBranch>| {
            branch.map_or_else(LambertTransferVelocities::default, |branch| {
                LambertTransferVelocities {
                    v1_n_mps: branch.v1,
                    v2_n_mps: branch.v2,
                    valid: true,
                }
            })
        };
        let performance = |branch: Option<LambertBranch>| {
            branch.map_or_else(LambertRootPerformance::default, |branch| {
                LambertRootPerformance {
                    x: branch.root.x,
                    num_iter: branch.root.iterations,
                    err_x: branch.root.step,
                }
            })
        };
        Ok((
            LambertSolutionMsg {
                first: velocities(solution.first),
                second: velocities(solution.second),
            },
            LambertPerformanceMsg {
                first: performance(solution.first),
                second: performance(solution.second),
            },
        ))
    }
}

impl Module for LambertSolver {
    fn init(&mut self) {
        self.lambert_solution_out_msg
            .write(LambertSolutionMsg::default());
        self.lambert_performance_out_msg
            .write(LambertPerformanceMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.solve() {
            Ok((solution, performance)) => {
                self.lambert_solution_out_msg.write(solution);
                self.lambert_performance_out_msg.write(performance);
            }
            // Upstream throws here, before writing: the outputs keep their
            // previous values.
            Err(error) => log::error!("lambertSolver: {error}"),
        }
    }
}

// Upstream test inputs are copied verbatim, digits included.
#[allow(clippy::excessive_precision)]
#[cfg(test)]
mod tests;
