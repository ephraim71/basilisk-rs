//! Newton–Raphson root finding, ported from upstream Basilisk
//! `newtonRaphsonSolve` (`architecture/utilities/avsEigenSupport.cpp`).

/// Upstream's iteration cap.
const MAX_ITERATIONS: usize = 100;

/// A zero of `f` near `initial_estimate` by Newton–Raphson iteration (upstream
/// `newtonRaphsonSolve`): stops once `|f(x)| < accuracy` or after 100 steps,
/// returning the last estimate either way, as upstream does.
pub fn newton_raphson_solve(
    initial_estimate: f64,
    accuracy: f64,
    f: impl Fn(f64) -> f64,
    f_prime: impl Fn(f64) -> f64,
) -> f64 {
    let mut estimate = initial_estimate;
    for _ in 0..MAX_ITERATIONS {
        let value = f(estimate);
        if value.abs() < accuracy {
            break;
        }
        estimate -= value / f_prime(estimate);
    }
    estimate
}

#[cfg(test)]
mod tests {
    use super::newton_raphson_solve;

    #[test]
    fn finds_a_square_root() {
        let root = newton_raphson_solve(1.0, 1e-14, |x| x * x - 2.0, |x| 2.0 * x);
        assert!((root - 2.0_f64.sqrt()).abs() < 1e-14);
    }

    /// A root the iteration starts on is returned unchanged.
    #[test]
    fn returns_an_initial_root_unchanged() {
        assert_eq!(newton_raphson_solve(0.5, 1e-12, |x| x - 0.5, |_| 1.0), 0.5);
    }

    /// Without convergence the estimate after the 100th step is returned.
    #[test]
    fn stops_after_one_hundred_steps() {
        let estimate = newton_raphson_solve(0.0, 0.0, |_| 1.0, |_| -1.0);
        assert_eq!(estimate, 100.0);
    }
}
