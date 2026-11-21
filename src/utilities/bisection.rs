//! Bisection root finding, ported from upstream Basilisk `bisectionSolve`
//! (`architecture/utilities/avsEigenSupport.cpp`).

use std::fmt;

/// Why [`bisection_solve`] could not search (each a `bskError` upstream).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BisectionError {
    /// The accuracy is not finite and strictly positive.
    InvalidAccuracy(f64),
    /// An interval bound is not finite.
    NonFiniteInterval,
    /// The interval runs from right to left.
    ReversedInterval,
    /// The function has the same sign at both bounds.
    NoSignChange,
}

impl fmt::Display for BisectionError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAccuracy(accuracy) => write!(
                f,
                "bisection accuracy must be finite and strictly positive, got {accuracy}"
            ),
            Self::NonFiniteInterval => write!(f, "bisection interval bounds must be finite"),
            Self::ReversedInterval => write!(f, "bisection interval must be from left to right"),
            Self::NoSignChange => write!(
                f,
                "function values at the bisection interval bounds must have opposite signs"
            ),
        }
    }
}

impl std::error::Error for BisectionError {}

/// Upstream `bisectionSolve`: a root of `f` in `[left, right]`, halving the
/// bracket until it is narrower than `accuracy`.
///
/// As upstream, a bound where `f` is exactly zero is returned as is, the
/// search stops at a midpoint where `f` is exactly zero, and otherwise the
/// result is the last midpoint evaluated (so within `accuracy` of a root).
pub fn bisection_solve(
    interval: [f64; 2],
    accuracy: f64,
    mut f: impl FnMut(f64) -> f64,
) -> Result<f64, BisectionError> {
    let [mut left, mut right] = interval;
    let mut current_estimate = (left + right) / 2.0;
    if !(accuracy.is_finite() && accuracy > 0.0) {
        return Err(BisectionError::InvalidAccuracy(accuracy));
    }
    if !(left.is_finite() && right.is_finite()) {
        return Err(BisectionError::NonFiniteInterval);
    }
    if left > right {
        return Err(BisectionError::ReversedInterval);
    }
    let mut f_left = f(left);
    let f_right = f(right);
    if f_left == 0.0 {
        return Ok(left);
    }
    if f_right == 0.0 {
        return Ok(right);
    }
    if f_left * f_right > 0.0 {
        return Err(BisectionError::NoSignChange);
    }
    while right - left >= accuracy {
        current_estimate = (left + right) / 2.0;
        let f_mid = f(current_estimate);
        if f_mid == 0.0 {
            break;
        } else if (f_left < 0.0 && f_mid > 0.0) || (f_left > 0.0 && f_mid < 0.0) {
            right = current_estimate;
        } else {
            left = current_estimate;
            f_left = f_mid;
        }
    }
    Ok(current_estimate)
}

#[cfg(test)]
mod tests {
    use super::{BisectionError, bisection_solve};

    #[test]
    fn finds_a_root_within_the_accuracy() {
        let root = bisection_solve([0.0, 2.0], 1e-12, |x| x * x - 2.0).unwrap();
        assert!((root - 2.0_f64.sqrt()).abs() < 1e-12, "{root}");
        // A decreasing function is bracketed the same way.
        let root = bisection_solve([-10.0, 10.0], 1e-9, |x| 3.0 - x).unwrap();
        assert!((root - 3.0).abs() < 1e-9, "{root}");
    }

    #[test]
    fn exact_roots_end_the_search() {
        assert_eq!(bisection_solve([1.0, 4.0], 1e-6, |x| x - 1.0), Ok(1.0));
        assert_eq!(bisection_solve([1.0, 4.0], 1e-6, |x| x - 4.0), Ok(4.0));
        assert_eq!(bisection_solve([0.0, 4.0], 1e-6, |x| x - 2.0), Ok(2.0));
    }

    #[test]
    fn invalid_searches_are_errors() {
        assert_eq!(
            bisection_solve([0.0, 1.0], 0.0, |x| x - 0.5),
            Err(BisectionError::InvalidAccuracy(0.0))
        );
        assert_eq!(
            bisection_solve([0.0, f64::INFINITY], 1e-6, |x| x - 0.5),
            Err(BisectionError::NonFiniteInterval)
        );
        assert_eq!(
            bisection_solve([1.0, 0.0], 1e-6, |x| x - 0.5),
            Err(BisectionError::ReversedInterval)
        );
        assert_eq!(
            bisection_solve([1.0, 2.0], 1e-6, |x| x - 0.5),
            Err(BisectionError::NoSignChange)
        );
    }
}
