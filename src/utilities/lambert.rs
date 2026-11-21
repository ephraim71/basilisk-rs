//! Lambert's problem: the conic arc that joins two positions in a given flight
//! time. Port of the solver inside upstream Basilisk
//! `fswAlgorithms/orbitControl/lambertSolver`, which upstream keeps private to
//! that module; here it is shared so any module or scenario can solve a
//! transfer without going through messages.
//!
//! Both algorithms solve Izzo's formulation (*Revisiting Lambert's problem*,
//! 2015) for the free variable `x` of the non-dimensional flight-time curve
//! `T(x)`, and differ only in the starter and the iteration:
//!
//! * [`LambertSolverMethod::Gooding`]: Gooding's bilinear starters (1990,
//!   rescaled to Izzo's time) refined with Halley steps;
//! * [`LambertSolverMethod::Izzo`]: Izzo's starters refined with third-order
//!   Householder steps.
//!
//! Every step is safeguarded inside a bracket of the root, with bisection when a
//! high-order step leaves it or fails to shrink. A root counts only once the
//! step (or the final correction) is below `1e-8` *and* the relative
//! flight-time residual is below `1e-12`.
//!
//! | upstream (`LambertSolver::`) | here |
//! |---|---|
//! | `readMessages` input checks | [`LambertProblem::solve`] returning [`LambertInputError`] |
//! | `problemGeometry` | `TransferGeometry::new` |
//! | `findx` | `TransferGeometry::find_roots` |
//! | `goodingInitialGuess`, `izzoInitialGuess` | the `*_starter(s)` functions |
//! | `x2tof` | `time_of_flight` |
//! | `dTdx` | `time_derivatives` |
//! | `solveRoot` | `solve_root` |
//! | `getTmin` | `minimum_time` |
//! | `hypergeometricF` | `hypergeometric_f` |
//! | `computeVelocities` | `TransferGeometry::velocities` |
//! | `RootResult` (converged) | [`LambertRoot`] |
//!
//! # Conventions
//!
//! Units are whatever the caller uses, as in [`super::orbital_motion`]:
//! positions in the length unit of `mu`, the flight time in its time unit, and
//! velocities come back in length per time unit. The transfer direction is
//! taken about the +z axis of the frame the positions are given in.
//!
//! Failures behave as upstream: malformed input is an error (upstream raises
//! `BasiliskError`), while an unusable transfer plane, an infeasible
//! multi-revolution time or a root search that does not converge is logged as
//! a warning and leaves that solution out.

use std::f64::consts::{LN_2, PI};
use std::fmt;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// Upstream `rootTimeTolerance` [-]: largest relative flight-time residual a
/// root may leave.
const ROOT_TIME_TOLERANCE: f64 = 1e-12;
/// Upstream `xTolerance` in `solveRoot` [-]: largest final step or correction
/// in `x` a root may take.
const ROOT_X_TOLERANCE: f64 = 1e-8;
/// Upstream `iterationLimit` in `solveRoot`.
const ROOT_ITERATION_LIMIT: usize = 80;
/// Upstream `expansionLimit` in `findx`: doublings of the zero-revolution
/// upper bracket.
const BRACKET_EXPANSION_LIMIT: usize = 64;
/// Upstream `tol` in `getTmin` [-]: `|dT/dx|` relative to `d2T/dx2` at which
/// the minimum-time location counts as found.
const MINIMUM_TIME_TOLERANCE: f64 = 1e-13;
/// Upstream `iterMax` in `getTmin`.
const MINIMUM_TIME_ITERATION_LIMIT: usize = 64;
/// Upstream `battin` in `x2tof`: within this distance of `x = 1` the flight
/// time uses Battin's hypergeometric series instead of Lancaster's formula.
const BATTIN_DISTANCE: f64 = 0.01;
/// Upstream threshold in `dTdx`: within this distance of `x = 1` the
/// derivatives take their parabolic limits.
const PARABOLIC_DERIVATIVE_DISTANCE: f64 = 1e-8;
/// Upstream `tol` in `hypergeometricF`.
const HYPERGEOMETRIC_TOLERANCE: f64 = 1e-11;
/// Upstream `c0` … `c42`: coefficients of Gooding's starters.
const GOODING_C0: f64 = 1.7;
const GOODING_C1: f64 = 0.5;
const GOODING_C2: f64 = 0.03;
const GOODING_C3: f64 = 0.15;
const GOODING_C41: f64 = 1.0;
const GOODING_C42: f64 = 0.24;
/// Upstream tolerance factor `8 eps` for roundoff-level comparisons: cross
/// products of the endpoint directions and the minimum flight time.
const ROUNDOFF_FACTOR: f64 = 8.0 * f64::EPSILON;

/// Lambert algorithm: upstream `SolverMethod`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum LambertSolverMethod {
    /// Izzo's starters with Householder iterations; upstream `IZZO = 0`, the
    /// zeroed default.
    #[default]
    Izzo = 0,
    /// Gooding's starters with Halley iterations; upstream `GOODING = 1`.
    Gooding = 1,
}

/// Sense of the transfer about the +z axis of the input frame: upstream
/// `LambertTransferDirection`.
///
/// Of the two transfer angles between the endpoints, the requested direction
/// picks the one whose orbit normal has the matching z sign. When the normal
/// lies in the xy-plane to within roundoff (a polar transfer), prograde keeps
/// `r1 x r2` and retrograde reverses it.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[repr(u8)]
pub enum LambertTransferDirection {
    /// Non-negative z-component of the orbital angular momentum; upstream
    /// `PROGRADE = 0`, the zeroed default.
    #[default]
    Prograde = 0,
    /// Non-positive z-component of the orbital angular momentum; upstream
    /// `RETROGRADE = 1`.
    Retrograde = 1,
}

/// One Lambert problem: the fields of upstream `LambertProblemMsgPayload`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LambertProblem {
    /// Upstream `r1_N`: position at the start of the transfer.
    pub r1: Vector3<f64>,
    /// Upstream `r2_N`: position at the end of the transfer.
    pub r2: Vector3<f64>,
    /// Upstream `transferTime`: flight time from `r1` to `r2`.
    pub transfer_time: f64,
    /// Upstream `mu`: gravitational parameter of the central body.
    pub mu: f64,
    /// Upstream `numRevolutions`: complete revolutions before arrival.
    pub revolutions: usize,
    /// Upstream `solverMethod`.
    pub method: LambertSolverMethod,
    /// Upstream `transferDirection`.
    pub direction: LambertTransferDirection,
}

/// Malformed input that upstream rejects with `BasiliskError` before any
/// geometry is computed.
///
/// Upstream also rejects negative revolution counts and unknown method or
/// direction codes; here the types rule those out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LambertInputError {
    /// `mu` is not finite and positive.
    Mu,
    /// The flight time is not finite and positive.
    TransferTime,
    /// `r1` has a non-finite component, or a zero or non-finite norm.
    R1,
    /// `r2` has a non-finite component, or a zero or non-finite norm.
    R2,
}

impl fmt::Display for LambertInputError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let endpoint = "must have finite components and a finite, nonzero computed norm";
        match self {
            Self::Mu => write!(f, "mu must be finite and positive"),
            Self::TransferTime => write!(f, "transferTime must be finite and positive"),
            Self::R1 => write!(f, "r1_N {endpoint}"),
            Self::R2 => write!(f, "r2_N {endpoint}"),
        }
    }
}

impl std::error::Error for LambertInputError {}

/// A converged root of the flight-time equation (upstream `RootResult` once
/// `converged` is set).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LambertRoot {
    /// Upstream `x` [-]: the free variable of Izzo's formulation.
    pub x: f64,
    /// Upstream `iterations`: refinements taken from the starter.
    pub iterations: usize,
    /// Upstream `error` [-]: magnitude of the final step or of the converged
    /// correction in `x`.
    pub step: f64,
}

/// One solution of a Lambert problem.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LambertBranch {
    /// Velocity at `r1` on the transfer arc.
    pub v1: Vector3<f64>,
    /// Velocity at `r2` on the transfer arc.
    pub v2: Vector3<f64>,
    /// The root the velocities were built from.
    pub root: LambertRoot,
}

/// Both solutions of a Lambert problem; `None` where none was found.
///
/// A zero-revolution problem has only `first`. A multi-revolution problem has
/// two: `first` on the branch of `T(x)` left of its minimum (`x < x_min`) and
/// `second` on the right. Exactly at the minimum flight time both are the same
/// solution.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LambertSolution {
    /// Upstream solution 1 (`v1_N`, `v2_N`, `valid`).
    pub first: Option<LambertBranch>,
    /// Upstream solution 2 (`v1Sol2_N`, `v2Sol2_N`, `validSol2`).
    pub second: Option<LambertBranch>,
}

impl LambertProblem {
    /// Solves the problem, rejecting a transfer plane within
    /// `alignment_threshold_rad` of parallel or antiparallel endpoints
    /// (upstream `alignmentThreshold`, there in degrees).
    ///
    /// # Errors
    ///
    /// [`LambertInputError`] for malformed input, which upstream raises as
    /// `BasiliskError`.
    pub fn solve(
        &self,
        alignment_threshold_rad: f64,
    ) -> Result<LambertSolution, LambertInputError> {
        self.validate()?;
        let Some(geometry) = TransferGeometry::new(self, alignment_threshold_rad) else {
            log::warn!(
                "lambertSolver: r1_N and r2_N do not define a usable transfer plane at the \
                 current alignmentThreshold, so no solution is returned."
            );
            return Ok(LambertSolution::default());
        };
        let (first, second) = geometry.find_roots(self.revolutions, self.method);
        Ok(LambertSolution {
            first: first.and_then(|root| geometry.branch(root, "first")),
            second: second.and_then(|root| geometry.branch(root, "second")),
        })
    }

    fn validate(&self) -> Result<(), LambertInputError> {
        if !(self.mu.is_finite() && self.mu > 0.0) {
            return Err(LambertInputError::Mu);
        }
        if !(self.transfer_time.is_finite() && self.transfer_time > 0.0) {
            return Err(LambertInputError::TransferTime);
        }
        // Components first: the norm of a vector holding a NaN or an infinity
        // says nothing useful, and an overflowed or underflowed norm cannot
        // be divided by either.
        let usable = |position: &Vector3<f64>| {
            position.iter().all(|component| component.is_finite()) && {
                let magnitude = position.norm();
                magnitude.is_finite() && magnitude > 0.0
            }
        };
        if !usable(&self.r1) {
            return Err(LambertInputError::R1);
        }
        if !usable(&self.r2) {
            return Err(LambertInputError::R2);
        }
        Ok(())
    }
}

/// The transfer plane, its orbit frames at both endpoints, and the
/// non-dimensional parameters of the flight-time equation (upstream
/// `problemGeometry`).
struct TransferGeometry {
    mu: f64,
    r1: f64,
    r2: f64,
    /// Chord length `|r2 - r1|`.
    chord: f64,
    /// Semiperimeter `(r1 + r2 + c) / 2`.
    semiperimeter: f64,
    /// Upstream `lambda` [-]: `±sqrt(1 - c/s)`, negative for a transfer angle
    /// above 180 degrees.
    lambda: f64,
    /// Upstream `TOF` [-]: the flight time scaled by `sqrt(2 mu / s^3)`.
    time: f64,
    /// Radial and transverse unit vectors at `r1` (upstream `Oframe1`).
    radial1: Vector3<f64>,
    transverse1: Vector3<f64>,
    /// Radial and transverse unit vectors at `r2` (upstream `Oframe2`).
    radial2: Vector3<f64>,
    transverse2: Vector3<f64>,
}

impl TransferGeometry {
    /// `None` when the endpoints are too close to parallel or antiparallel to
    /// define a plane: within `alignment_threshold_rad`, or with a cross
    /// product of their directions no larger than its roundoff.
    fn new(problem: &LambertProblem, alignment_threshold_rad: f64) -> Option<Self> {
        let r1 = problem.r1.norm();
        let r2 = problem.r2.norm();
        let radial1 = problem.r1 / r1;
        let radial2 = problem.r2 / r2;
        let radial_cross = radial1.cross(&radial2);
        let sin_theta = radial_cross.norm();

        // Normalizing collinear endpoints can leave a nonzero cross product.
        // Bound the cancellation roundoff component by component, so a small
        // but resolvable cross product stays usable when the threshold allows.
        let [a, b] = [radial1, radial2];
        let cross_tolerance = Vector3::new(
            (a.y * b.z).abs() + (a.z * b.y).abs(),
            (a.z * b.x).abs() + (a.x * b.z).abs(),
            (a.x * b.y).abs() + (a.y * b.x).abs(),
        ) * ROUNDOFF_FACTOR;
        let unresolved_plane = radial_cross
            .iter()
            .zip(cross_tolerance.iter())
            .all(|(component, tolerance)| component.abs() <= *tolerance);
        if unresolved_plane || sin_theta == 0.0 || sin_theta < alignment_threshold_rad.sin() {
            return None;
        }

        // Only now: antiparallel endpoints can round 1 - c/s negative.
        let chord = (problem.r2 - problem.r1).norm();
        let semiperimeter = 0.5 * (r1 + r2 + chord);
        let mut lambda = (1.0 - chord / semiperimeter).sqrt();
        let time = (2.0 * problem.mu / semiperimeter.powf(3.0)).sqrt() * problem.transfer_time;

        // Reversing the orbit normal selects the complementary transfer angle,
        // which flips the sign of lambda. A z-component within roundoff of zero
        // is a polar tie: prograde keeps r1 x r2, retrograde reverses it.
        let mut normal = radial_cross / sin_theta;
        let retrograde = problem.direction == LambertTransferDirection::Retrograde;
        if (radial_cross.z < -cross_tolerance.z) != retrograde {
            lambda = -lambda;
            normal = -normal;
        }

        Some(Self {
            mu: problem.mu,
            r1,
            r2,
            chord,
            semiperimeter,
            lambda,
            time,
            radial1,
            transverse1: normal.cross(&radial1),
            radial2,
            transverse2: normal.cross(&radial2),
        })
    }

    /// The roots of both solutions on their branches of `T(x)` (upstream
    /// `findx`).
    fn find_roots(
        &self,
        revolutions: usize,
        method: LambertSolverMethod,
    ) -> (Option<LambertRoot>, Option<LambertRoot>) {
        let (time, lambda) = (self.time, self.lambda);
        if !(time.is_finite() && time > 0.0 && lambda.is_finite() && lambda.abs() < 1.0) {
            log::warn!("lambertSolver: root search requires finite time and geometry.");
            return (None, None);
        }
        if revolutions == 0 {
            (zero_revolution_root(time, lambda, method), None)
        } else {
            multi_revolution_roots(time, lambda, revolutions, method)
        }
    }

    /// The solution built from `root` (upstream `computeVelocities`), or
    /// `None` if its velocities are not finite: a root alone does not
    /// guarantee that when the geometry is numerically unresolvable.
    fn branch(&self, root: LambertRoot, which: &str) -> Option<LambertBranch> {
        let (v1, v2) = self.velocities(root.x);
        let finite = |v: &Vector3<f64>| v.iter().all(|component| component.is_finite());
        if finite(&v1) && finite(&v2) {
            Some(LambertBranch { v1, v2, root })
        } else {
            log::warn!("lambertSolver: {which} solution has nonfinite velocities.");
            None
        }
    }

    /// Velocities at both endpoints of the transfer with free variable `x`.
    fn velocities(&self, x: f64) -> (Vector3<f64>, Vector3<f64>) {
        let lambda = self.lambda;
        let y = (1.0 - lambda * lambda * (1.0 - x * x)).sqrt();
        let gamma = (self.mu * self.semiperimeter / 2.0).sqrt();
        let rho = (self.r1 - self.r2) / self.chord;
        let sigma = (1.0 - rho * rho).sqrt();

        let radial_speed1 = gamma * ((lambda * y - x) - rho * (lambda * y + x)) / self.r1;
        let radial_speed2 = -gamma * ((lambda * y - x) + rho * (lambda * y + x)) / self.r2;
        let transverse_speed1 = gamma * sigma * (y + lambda * x) / self.r1;
        let transverse_speed2 = gamma * sigma * (y + lambda * x) / self.r2;

        (
            self.radial1 * radial_speed1 + self.transverse1 * transverse_speed1,
            self.radial2 * radial_speed2 + self.transverse2 * transverse_speed2,
        )
    }
}

/// Zero-revolution root: `T(x)` falls monotonically from infinity at
/// `x = -1` to zero as `x` grows, so one root exists for any positive time.
fn zero_revolution_root(
    time: f64,
    lambda: f64,
    method: LambertSolverMethod,
) -> Option<LambertRoot> {
    let starter = match method {
        LambertSolverMethod::Gooding => gooding_zero_revolution_starter(lambda, time),
        LambertSolverMethod::Izzo => izzo_zero_revolution_starter(lambda, time),
    };

    // Keep the analytic parabolic solution when it meets the residual
    // tolerance of an iterated root, including roundoff in the geometry.
    let parabolic_time = time_of_flight(1.0, 0, lambda);
    if (time - parabolic_time).abs() <= ROOT_TIME_TOLERANCE * time {
        return Some(LambertRoot {
            x: 1.0,
            iterations: 0,
            step: 0.0,
        });
    }

    // Expand the finite upper bound for hyperbolic transfers; x = -1 is never
    // evaluated.
    let mut x_upper = if starter.is_finite() {
        starter.max(1.0)
    } else {
        1.0
    };
    let mut bracketed = false;
    for _ in 0..BRACKET_EXPANSION_LIMIT {
        let upper_time = time_of_flight(x_upper, 0, lambda);
        if !(upper_time.is_finite() && upper_time > 0.0) {
            break;
        }
        if upper_time <= time {
            bracketed = true;
            break;
        }
        x_upper *= 2.0;
    }
    if !bracketed {
        log::warn!("lambertSolver: could not bracket the zero-revolution root.");
        return None;
    }
    let branch = Branch {
        lambda,
        revolutions: 0,
        method,
        increasing: false,
    };
    branch.solve_root(time, starter, -1.0, x_upper)
}

/// Both multi-revolution roots, one on each side of the minimum of `T(x)`.
fn multi_revolution_roots(
    time: f64,
    lambda: f64,
    revolutions: usize,
    method: LambertSolverMethod,
) -> (Option<LambertRoot>, Option<LambertRoot>) {
    let time_at_zero = zero_x_time(lambda) + revolutions as f64 * PI;
    let Some((x_min, minimum)) = minimum_time(lambda, time_at_zero, revolutions) else {
        return (None, None);
    };
    let minimum_tolerance = ROUNDOFF_FACTOR * minimum;
    if time < minimum - minimum_tolerance {
        log::warn!(
            "lambertSolver: no multi-revolution solution exists for the given time of flight."
        );
        return (None, None);
    }
    let coalesced = |x| {
        // The minimum-time search already checked convergence here.
        let root = Some(LambertRoot {
            x,
            iterations: 0,
            step: 0.0,
        });
        (root, root)
    };
    if (time - minimum).abs() <= minimum_tolerance {
        return coalesced(x_min);
    }

    let (left_starter, right_starter) = match method {
        LambertSolverMethod::Gooding => gooding_multi_revolution_starters(
            lambda,
            time,
            revolutions,
            time_at_zero,
            x_min,
            minimum,
        ),
        LambertSolverMethod::Izzo => izzo_multi_revolution_starters(time, revolutions),
    };
    if left_starter == right_starter {
        return coalesced(left_starter);
    }
    let left = Branch {
        lambda,
        revolutions,
        method,
        increasing: false,
    };
    let right = Branch {
        increasing: true,
        ..left
    };
    (
        left.solve_root(time, left_starter, -1.0, x_min),
        right.solve_root(time, right_starter, x_min, 1.0),
    )
}

/// Upstream `T00`: non-dimensional zero-revolution flight time at `x = 0`.
fn zero_x_time(lambda: f64) -> f64 {
    lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt()
}

/// Gooding's transfer-angle parameter `theta` of his starters. `atan2`
/// already places it above pi for negative lambda.
fn gooding_theta(lambda: f64) -> f64 {
    2.0 * (1.0 - lambda * lambda).atan2(2.0 * lambda)
}

/// Gooding's zero-revolution starter, patched over the time range (upstream
/// `goodingInitialGuess` for `N = 0`).
fn gooding_zero_revolution_starter(lambda: f64, time: f64) -> f64 {
    let time_at_zero = zero_x_time(lambda);
    let theta = gooding_theta(lambda);
    let excess = time - time_at_zero;

    let x01 = -excess / (excess + 4.0);
    let weight = x01 + GOODING_C0 * (2.0 - theta / PI).sqrt();
    let x03 = if weight >= 0.0 {
        x01
    } else {
        let x02 = -(excess / (time + 0.5 * time_at_zero)).sqrt();
        x01 + (-weight).powf(1.0 / 16.0) * (x02 - x01)
    };
    let scale =
        1.0 + GOODING_C1 * x03 * (1.0 + x01) - GOODING_C2 * (x03 * x03) * (1.0 + x01).sqrt();
    scale * x03
}

/// Gooding's two multi-revolution starters from the minimum `(x_min,
/// minimum)` of `T(x)` (upstream `goodingInitialGuess` for `N > 0`).
///
/// Izzo's non-dimensional time is half of Gooding's XLAMB time, so the
/// bilinear starter and its weighting use 2 where XLAMB uses 4.
fn gooding_multi_revolution_starters(
    lambda: f64,
    time: f64,
    revolutions: usize,
    time_at_zero: f64,
    x_min: f64,
    minimum: f64,
) -> (f64, f64) {
    let n = revolutions as f64;
    let theta_fraction = gooding_theta(lambda) / (2.0 * PI);
    let [_, second_derivative, _] = time_derivatives(x_min, minimum, lambda);
    let excess_over_minimum = time - minimum;
    let excess_over_zero = time - time_at_zero;

    let left = if time <= time_at_zero {
        let term = excess_over_minimum
            / (0.5 * second_derivative
                - excess_over_minimum
                    * (0.5 * second_derivative / (time_at_zero - minimum) - 1.0 / (x_min * x_min)));
        x_min - term.sqrt()
    } else {
        let mut x01 = -excess_over_zero / (excess_over_zero + 2.0);
        let weight = x01 + GOODING_C0 * (2.0 * (1.0 - theta_fraction)).sqrt();
        if weight < 0.0 {
            x01 -= (-weight).powf(1.0 / 16.0)
                * (x01 + (excess_over_zero / (excess_over_zero + 1.5 * time_at_zero)).sqrt());
        }
        // The correction applies after either bilinear patch.
        let w2 = 2.0 / (2.0 + excess_over_zero);
        x01 * (1.0
            + (1.0 + n + GOODING_C42 * (theta_fraction - 0.5)) / (1.0 + GOODING_C3 * n)
                * x01
                * (GOODING_C1 * w2 - GOODING_C2 * x01 * w2.sqrt()))
    };

    // The right starter is a positive displacement from the minimum.
    let term = excess_over_minimum
        / (0.5 * second_derivative + excess_over_minimum / ((1.0 - x_min) * (1.0 - x_min)));
    let x01 = term.sqrt();
    let mut weight = x_min + x01;
    weight = weight * 2.0 / (2.0 + excess_over_minimum) + (1.0 - weight) * (1.0 - weight);
    let right = x01
        * (1.0
            - (1.0 + n + GOODING_C41 * (theta_fraction - 0.5)) / (1.0 + GOODING_C3 * n)
                * x01
                * (GOODING_C1 * weight + GOODING_C2 * x01 * weight.sqrt()))
        + x_min;

    (left, right)
}

/// Izzo's zero-revolution starter (upstream `izzoInitialGuess` for `N = 0`).
fn izzo_zero_revolution_starter(lambda: f64, time: f64) -> f64 {
    let time_at_zero = zero_x_time(lambda);
    let parabolic_time = 2.0 / 3.0 * (1.0 - lambda.powf(3.0));
    if time >= time_at_zero {
        (time_at_zero / time).powf(2.0 / 3.0) - 1.0
    } else if time <= parabolic_time {
        5.0 / 2.0 * parabolic_time * (parabolic_time - time) / (time * (1.0 - lambda.powf(5.0)))
            + 1.0
    } else {
        // Interpolate log(1 + x) from 0 at T00 to log(2) at the parabolic time.
        (time_at_zero / time).powf(LN_2 / (time_at_zero / parabolic_time).ln()) - 1.0
    }
}

/// Izzo's two multi-revolution starters (upstream `izzoInitialGuess` for
/// `N > 0`).
fn izzo_multi_revolution_starters(time: f64, revolutions: usize) -> (f64, f64) {
    let n = revolutions as f64;
    let left = ((n * PI + PI) / (8.0 * time)).powf(2.0 / 3.0);
    let right = ((8.0 * time) / (n * PI)).powf(2.0 / 3.0);
    ((left - 1.0) / (left + 1.0), (right - 1.0) / (right + 1.0))
}

/// Non-dimensional flight time `T(x)` for `revolutions` complete revolutions
/// (upstream `x2tof`).
///
/// Lancaster's formula, except within [`BATTIN_DISTANCE`] of `x = 1`, where
/// Battin's hypergeometric form stays smooth arbitrarily close to the
/// parabola. The zero-revolution parabola itself is exact.
fn time_of_flight(x: f64, revolutions: usize, lambda: f64) -> f64 {
    let n = revolutions as f64;
    let u = 1.0 - x * x;
    let y = (1.0 - lambda * lambda * u).sqrt();

    if x == 1.0 && revolutions == 0 {
        2.0 / 3.0 * (1.0 - lambda.powf(3.0))
    } else if (x - 1.0).abs() < BATTIN_DISTANCE {
        // Never flattened to the parabolic value: a constant interval would
        // hide the true flight-time residual.
        let eta = y - lambda * x;
        let s1 = 0.5 * (1.0 - lambda - x * eta);
        let q = 4.0 / 3.0 * hypergeometric_f(s1);
        (eta.powf(3.0) * q + 4.0 * lambda * eta) / 2.0 + n * PI / u.abs().powf(1.5)
    } else {
        let f = (y - lambda * x) * u.abs().sqrt();
        let g = x * y + lambda * u;
        let d = if u > 0.0 {
            f.atan2(g) + n * PI // elliptic
        } else {
            (f / g).atanh() // hyperbolic
        };
        (d / u.abs().sqrt() - x + lambda * y) / u
    }
}

/// First three derivatives of `T(x)` at `x`, where `T(x) = time` (upstream
/// `dTdx`); their parabolic limits within
/// [`PARABOLIC_DERIVATIVE_DISTANCE`] of `x = 1`.
fn time_derivatives(x: f64, time: f64, lambda: f64) -> [f64; 3] {
    if (x - 1.0).abs() < PARABOLIC_DERIVATIVE_DISTANCE {
        return [2.0 / 5.0 * (lambda.powf(5.0) - 1.0), 0.0, 0.0];
    }
    let u = 1.0 - x * x;
    let y = (1.0 - lambda * lambda * u).sqrt();
    let lambda3 = lambda.powf(3.0);
    let one_minus_lambda2 = 1.0 - lambda * lambda;
    let first = 1.0 / u * (3.0 * time * x - 2.0 + 2.0 * lambda3 * x / y);
    let second =
        1.0 / u * (3.0 * time + 5.0 * x * first + 2.0 * one_minus_lambda2 * lambda3 / y.powf(3.0));
    let third = 1.0 / u
        * (7.0 * x * second + 8.0 * first
            - 6.0 * one_minus_lambda2 * lambda.powf(5.0) * x / y.powf(5.0));
    [first, second, third]
}

/// Gauss's hypergeometric function `2F1(3, 1; 5/2; z)` from its series, at
/// most 13 terms (upstream `hypergeometricF`).
fn hypergeometric_f(z: f64) -> f64 {
    let (a, b, c) = (3.0, 1.0, 2.5);
    let mut sum = 1.0;
    let mut term = 1.0;
    for j in 0..=12 {
        let j = f64::from(j);
        term = term * (a + j) * (b + j) / (c + j) * z / (j + 1.0);
        sum += term;
        if term.abs() < HYPERGEOMETRIC_TOLERANCE {
            break;
        }
    }
    sum
}

/// Location and value `(x_min, T_min)` of the minimum of the multi-revolution
/// flight-time curve (upstream `getTmin`), or `None` if the search fails.
///
/// Halley steps on `dT/dx = 0` stay inside a bracket of the derivative's sign
/// change, which starts as `[0, 1)`: `dT/dx = -2` at `x = 0` and grows without
/// bound towards `x = 1`. A step that leaves the bracket or does not halve is
/// replaced by bisection. A small step alone is not enough: the derivative must
/// vanish at a true minimum.
fn minimum_time(lambda: f64, time_at_zero: f64, revolutions: usize) -> Option<(f64, f64)> {
    if revolutions == 0 || !time_at_zero.is_finite() || !lambda.is_finite() || lambda.abs() >= 1.0 {
        log::warn!("lambertSolver: minimum-time search requires finite elliptic geometry.");
        return None;
    }

    let (mut x_lower, mut x_upper) = (0.0, 1.0);
    let mut previous_step: f64 = x_upper - x_lower;
    let mut x = x_lower;
    let mut time = time_at_zero;
    for _ in 0..MINIMUM_TIME_ITERATION_LIMIT {
        let [first, second, third] = time_derivatives(x, time, lambda);
        if !(time.is_finite()
            && time > 0.0
            && first.is_finite()
            && second.is_finite()
            && third.is_finite())
        {
            break;
        }
        if second > 0.0 && first.abs() <= MINIMUM_TIME_TOLERANCE * second {
            return Some((x, time));
        }
        if first < 0.0 {
            x_lower = x;
        } else {
            x_upper = x;
        }

        let mut x_new = 0.5 * (x_lower + x_upper);
        let denominator = second * second - first * third / 2.0;
        if denominator.is_finite() && denominator != 0.0 {
            let step = first * second / denominator;
            let candidate = x - step;
            if candidate.is_finite()
                && candidate > x_lower
                && candidate < x_upper
                && step.abs() <= 0.5 * previous_step
            {
                x_new = candidate;
            }
        }
        previous_step = (x_new - x).abs();
        x = x_new;
        time = time_of_flight(x, revolutions, lambda);
    }
    log::warn!("lambertSolver: minimum-time search failed to converge to a finite minimum.");
    None
}

/// One monotonic branch of `T(x)` for a root search.
#[derive(Clone, Copy)]
struct Branch {
    lambda: f64,
    revolutions: usize,
    method: LambertSolverMethod,
    /// `T(x)` increases with `x`: the right multi-revolution branch.
    increasing: bool,
}

impl Branch {
    /// The root of `T(x) = time` inside `(x_lower, x_upper)` from `starter`,
    /// by safeguarded Halley (Gooding) or Householder (Izzo) steps (upstream
    /// `solveRoot`); `None` if it does not converge.
    ///
    /// A starter outside the bracket is replaced by its midpoint. A finite
    /// zero-revolution upper bound is admissible, and may be the starter
    /// itself; the multi-revolution bounds stay open to exclude the
    /// singularity at `x = 1`. Bisection replaces a step that is not finite,
    /// leaves the bracket, or is more than half the previous high-order step.
    /// Success needs a small step or correction *and* a small relative
    /// residual (or an exact match); numerical failure never counts.
    fn solve_root(
        &self,
        time: f64,
        starter: f64,
        mut x_lower: f64,
        mut x_upper: f64,
    ) -> Option<LambertRoot> {
        let multi_revolution = self.revolutions > 0;
        let mut x = starter;
        if !x.is_finite() || x <= x_lower || x > x_upper || (multi_revolution && x == x_upper) {
            x = x_lower + 0.5 * (x_upper - x_lower);
        }

        let mut step = f64::INFINITY;
        // Empty at the start and after a bisection.
        let mut previous_high_order_step: Option<f64> = None;
        for iterations in 0..=ROOT_ITERATION_LIMIT {
            let flight_time = time_of_flight(x, self.revolutions, self.lambda);
            if !(flight_time.is_finite() && flight_time > 0.0)
                || x <= -1.0
                || (multi_revolution && x >= 1.0)
            {
                break;
            }
            let residual = flight_time - time;
            let residual_small = residual.abs() <= ROOT_TIME_TOLERANCE * time;
            // An exact match needs no correction, however large the last step.
            if residual == 0.0 {
                step = 0.0;
            }
            if step <= ROOT_X_TOLERANCE && residual_small {
                return Some(LambertRoot {
                    x,
                    iterations,
                    step,
                });
            }
            if iterations == ROOT_ITERATION_LIMIT {
                break;
            }

            if (residual < 0.0) == self.increasing {
                x_lower = x;
            } else {
                x_upper = x;
            }
            let mut x_new = x_lower + 0.5 * (x_upper - x_lower);
            let mut high_order = false;
            if let Some(correction) = self.correction(x, flight_time, residual) {
                // The residual can pass right after a large step. Accept a small
                // correction before the strict bracket test rejects a candidate
                // equal to x (now a bound) and sends an accurate root back to
                // the midpoint.
                if correction.abs() <= ROOT_X_TOLERANCE && residual_small {
                    return Some(LambertRoot {
                        x,
                        iterations,
                        step: correction.abs(),
                    });
                }
                let candidate = x - correction;
                // Compare consecutive high-order steps only: after a bisection
                // an admissible step resumes without being measured against
                // the bisection distance, which could force repeated fallback.
                let shrinking = previous_high_order_step
                    .is_none_or(|previous| (candidate - x).abs() <= 0.5 * previous);
                if candidate > x_lower && candidate < x_upper && shrinking {
                    x_new = candidate;
                    high_order = true;
                }
            }
            // A collapsed bracket cannot improve the residual; stagnation is not
            // convergence.
            if x_new == x {
                break;
            }
            step = (x_new - x).abs();
            previous_high_order_step = high_order.then_some(step);
            x = x_new;
        }
        log::warn!("lambertSolver: root search failed to converge within its numerical limits.");
        None
    }

    /// The finite Halley (Gooding) or Householder (Izzo) correction at `x`,
    /// where `T(x) - time = residual`.
    fn correction(&self, x: f64, flight_time: f64, residual: f64) -> Option<f64> {
        let [first, second, third] = time_derivatives(x, flight_time, self.lambda);
        let (numerator, denominator) = match self.method {
            LambertSolverMethod::Gooding => {
                (residual * first, first * first - residual * second / 2.0)
            }
            LambertSolverMethod::Izzo => (
                residual * (first * first - residual * second / 2.0),
                first * (first * first - residual * second) + third * residual * residual / 6.0,
            ),
        };
        if !(numerator.is_finite() && denominator.is_finite() && denominator != 0.0) {
            return None;
        }
        Some(numerator / denominator).filter(|correction| correction.is_finite())
    }
}

#[cfg(test)]
mod tests {
    //! The solver is exercised end to end, against upstream's tests, through
    //! the `lambertSolver` module in
    //! `crate::fsw_algorithms::orbit_control::lambert_solver`. These check the
    //! pieces against independent formulas.

    use super::*;

    /// Lagrange's elliptic flight time, an independent formulation of
    /// [`time_of_flight`] (the same as upstream's `lagrange_time` test helper).
    fn lagrange_time(x: f64, lambda: f64, revolutions: usize) -> f64 {
        let axis = 1.0 / (1.0 - x * x);
        let alpha = 2.0 * x.acos();
        let beta = 2.0 * (lambda / axis.sqrt()).asin();
        axis.powf(1.5) * (alpha - alpha.sin() - beta + beta.sin() + 2.0 * PI * revolutions as f64)
            / 2.0
    }

    #[test]
    fn lancaster_and_battin_times_agree_with_lagrange() {
        for lambda in [-0.9, -0.3, 0.0, 0.4, 0.95] {
            for revolutions in [0, 1, 3] {
                // Both sides of the Battin switch at |x - 1| = 0.01.
                for x in [-0.95, -0.4, 0.0, 0.5, 0.95, 0.985, 0.995, 0.9999] {
                    let expected = lagrange_time(x, lambda, revolutions);
                    let actual = time_of_flight(x, revolutions, lambda);
                    assert!(
                        (actual - expected).abs() <= 1e-9 * expected,
                        "lambda {lambda}, N {revolutions}, x {x}: {actual} vs {expected}"
                    );
                }
            }
        }
    }

    #[test]
    fn parabolic_time_is_exact_and_battin_is_continuous_across_it() {
        for lambda in [-0.7_f64, 0.2, 0.9] {
            let parabolic = 2.0 / 3.0 * (1.0 - lambda.powf(3.0));
            assert_eq!(time_of_flight(1.0, 0, lambda), parabolic);
            for offset in [-1e-6, 1e-6] {
                let nearby = time_of_flight(1.0 + offset, 0, lambda);
                assert!((nearby - parabolic).abs() < 1e-5, "{nearby} vs {parabolic}");
                // T decreases through the parabola.
                assert!((nearby - parabolic) * offset < 0.0);
            }
        }
    }

    #[test]
    fn derivatives_match_central_differences() {
        for lambda in [-0.6, 0.3, 0.8] {
            for revolutions in [0, 2] {
                for x in [-0.5, 0.1, 0.6] {
                    let h = 1e-5;
                    let time = |x| time_of_flight(x, revolutions, lambda);
                    let [first, second, third] = time_derivatives(x, time(x), lambda);
                    let numeric_first = (time(x + h) - time(x - h)) / (2.0 * h);
                    let numeric_second = (time(x + h) - 2.0 * time(x) + time(x - h)) / (h * h);
                    let first_at = |x| time_derivatives(x, time(x), lambda)[0];
                    let numeric_third =
                        (first_at(x + h) - 2.0 * first_at(x) + first_at(x - h)) / (h * h);
                    assert!((first - numeric_first).abs() < 1e-6 * first.abs().max(1.0));
                    assert!((second - numeric_second).abs() < 1e-3 * second.abs().max(1.0));
                    assert!((third - numeric_third).abs() < 1e-3 * third.abs().max(1.0));
                }
            }
        }
    }

    /// The 13-term truncation against the converged series over the small
    /// arguments the Battin branch produces.
    #[test]
    fn hypergeometric_truncation_matches_the_converged_series() {
        for z in [-0.05, -0.01, 0.0, 0.003, 0.01, 0.05] {
            let mut sum = 1.0;
            let mut term = 1.0;
            for j in 0..200 {
                let j = f64::from(j);
                term *= (3.0 + j) * (1.0 + j) / (2.5 + j) * z / (j + 1.0);
                sum += term;
            }
            assert!((hypergeometric_f(z) - sum).abs() < 1e-10, "z {z}");
        }
    }

    #[test]
    fn minimum_time_is_a_stationary_point_of_the_flight_time() {
        for lambda in [-0.9, 0.0, 0.7] {
            for revolutions in [1, 4] {
                let time_at_zero = zero_x_time(lambda) + revolutions as f64 * PI;
                let (x_min, minimum) =
                    minimum_time(lambda, time_at_zero, revolutions).expect("a minimum exists");
                assert!(0.0 < x_min && x_min < 1.0);
                assert_eq!(minimum, time_of_flight(x_min, revolutions, lambda));
                for offset in [-1e-4, 1e-4] {
                    assert!(time_of_flight(x_min + offset, revolutions, lambda) > minimum);
                }
            }
        }
    }

    #[test]
    fn malformed_input_is_rejected_in_upstream_order() {
        let problem = LambertProblem {
            r1: Vector3::new(8e6, 0.0, 0.0),
            r2: Vector3::new(0.0, 8e6, 0.0),
            transfer_time: 2000.0,
            mu: 3.986004360e14,
            revolutions: 0,
            method: LambertSolverMethod::Izzo,
            direction: LambertTransferDirection::Prograde,
        };
        assert!(problem.solve(1e-3).is_ok());
        let everything_wrong = LambertProblem {
            mu: f64::NAN,
            transfer_time: -1.0,
            r1: Vector3::zeros(),
            r2: Vector3::new(f64::INFINITY, 0.0, 0.0),
            ..problem
        };
        assert_eq!(everything_wrong.solve(1e-3), Err(LambertInputError::Mu));
        let wrong = LambertProblem {
            mu: problem.mu,
            ..everything_wrong
        };
        assert_eq!(wrong.solve(1e-3), Err(LambertInputError::TransferTime));
        let wrong = LambertProblem {
            transfer_time: problem.transfer_time,
            ..wrong
        };
        assert_eq!(wrong.solve(1e-3), Err(LambertInputError::R1));
        let wrong = LambertProblem {
            r1: problem.r1,
            ..wrong
        };
        assert_eq!(wrong.solve(1e-3), Err(LambertInputError::R2));
    }
}
