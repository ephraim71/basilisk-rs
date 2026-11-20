//! First-order bounded Gauss-Markov error model, as Basilisk's `GaussMarkov`.
//!
//! Each step is `state = A * state + P * w` with `w ~ N(0, 1)` per state, so a
//! `noise_matrix_sqrt` diagonal entry of `x` gives error steps with a standard
//! deviation of `x`. A state past a positive bound is clamped to that bound.

use nalgebra::{Matrix3, Vector3};
use rand::Rng;
use rand_distr::{Distribution, Normal};

/// Three-state bounded Gauss-Markov process.
#[derive(Clone, Debug)]
pub struct GaussMarkov {
    /// Error-state propagation matrix. Zero reduces the model to independent
    /// per-sample noise; identity is a random walk.
    pub prop_matrix: Matrix3<f64>,
    /// Matrix square root of the noise covariance (1-sigma).
    pub noise_matrix_sqrt: Matrix3<f64>,
    /// Per-state bounds; a non-positive bound leaves that state unbounded.
    pub state_bounds: Vector3<f64>,
    state: Vector3<f64>,
}

impl GaussMarkov {
    pub fn new(
        prop_matrix: Matrix3<f64>,
        noise_matrix_sqrt: Matrix3<f64>,
        state_bounds: Vector3<f64>,
    ) -> Self {
        Self {
            prop_matrix,
            noise_matrix_sqrt,
            state_bounds,
            state: Vector3::zeros(),
        }
    }

    /// Clear the accumulated error state (module reset).
    pub fn reset(&mut self) {
        self.state = Vector3::zeros();
    }

    pub fn current_state(&self) -> Vector3<f64> {
        self.state
    }

    /// Advance one step and return the new error state.
    pub fn compute_next_state<R: Rng>(&mut self, rng: &mut R) -> Vector3<f64> {
        let normal = Normal::new(0.0, 1.0).expect("valid distribution");
        let draws = Vector3::from_fn(|_, _| normal.sample(rng));
        self.state = self.prop_matrix * self.state + self.noise_matrix_sqrt * draws;
        for i in 0..3 {
            let bound = self.state_bounds[i];
            if bound > 0.0 && self.state[i].abs() > bound {
                self.state[i] = bound.copysign(self.state[i]);
            }
        }
        self.state
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    use super::GaussMarkov;

    /// With zero propagation the process is IID and the output standard
    /// deviation equals the noise-matrix entry.
    #[test]
    fn noise_matrix_entry_is_the_step_standard_deviation() {
        let sigma = 1.0e-4;
        let mut model = GaussMarkov::new(
            Matrix3::zeros(),
            Matrix3::from_diagonal(&Vector3::repeat(sigma)),
            Vector3::zeros(),
        );
        let mut rng = StdRng::seed_from_u64(7);

        let n = 50_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            let error = model.compute_next_state(&mut rng).x;
            sum += error;
            sum_sq += error * error;
        }
        let mean = sum / n as f64;
        let std = (sum_sq / n as f64 - mean * mean).sqrt();
        assert!(
            (std - sigma).abs() < 0.05 * sigma,
            "std {std} vs expected {sigma}"
        );
    }

    /// With identity propagation the walk is clamped: it never passes the
    /// bound and, with steps larger than the bound, it often sits on it.
    #[test]
    fn bounds_clamp_the_walk() {
        let bound = 5.0e-4;
        let mut model = GaussMarkov::new(
            Matrix3::identity(),
            Matrix3::from_diagonal(&Vector3::repeat(1.0e-3)),
            Vector3::repeat(bound),
        );
        let mut rng = StdRng::seed_from_u64(11);

        let mut at_bound = 0usize;
        for _ in 0..20_000 {
            let error = model.compute_next_state(&mut rng).x;
            assert!(error.abs() <= bound, "walk passed the bound: {error}");
            if error.abs() == bound {
                at_bound += 1;
            }
        }
        assert!(at_bound > 0, "walk never reached the bound");
    }

    #[test]
    fn reset_clears_the_accumulated_state() {
        let mut model = GaussMarkov::new(
            Matrix3::identity(),
            Matrix3::from_diagonal(&Vector3::repeat(1.0e-3)),
            Vector3::zeros(),
        );
        let mut rng = StdRng::seed_from_u64(3);
        for _ in 0..10 {
            model.compute_next_state(&mut rng);
        }
        assert!(model.current_state().norm() > 0.0);
        model.reset();
        assert_eq!(model.current_state(), Vector3::zeros());
    }
}
