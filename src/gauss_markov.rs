//! First-order bounded Gauss-Markov error model, as Basilisk's `GaussMarkov`.
//!
//! Each step is `state = A * state + P * w` with `w ~ N(0, 1)` per state, so a
//! `noise_matrix_sqrt` diagonal entry of `x` gives error steps with a standard
//! deviation of `x`. A state past a positive bound is clamped to that bound.
//!
//! [`GaussMarkov`] carries `N` error states, three unless a caller names
//! another count (one for a scalar sensor, twelve for a full navigation
//! state); its propagation and noise matrices may couple any pair of them.

use nalgebra::{SMatrix, SVector};
use rand::Rng;
use rand_distr::{Distribution, Normal};

/// `N`-state bounded Gauss-Markov process (three states by default).
#[derive(Clone, Debug)]
pub struct GaussMarkov<const N: usize = 3> {
    /// Error-state propagation matrix. Zero reduces the model to independent
    /// per-sample noise; identity is a random walk.
    pub prop_matrix: SMatrix<f64, N, N>,
    /// Matrix square root of the noise covariance (1-sigma).
    pub noise_matrix_sqrt: SMatrix<f64, N, N>,
    /// Per-state bounds; a non-positive bound leaves that state unbounded.
    pub state_bounds: SVector<f64, N>,
    state: SVector<f64, N>,
}

impl<const N: usize> GaussMarkov<N> {
    pub fn new(
        prop_matrix: SMatrix<f64, N, N>,
        noise_matrix_sqrt: SMatrix<f64, N, N>,
        state_bounds: SVector<f64, N>,
    ) -> Self {
        Self {
            prop_matrix,
            noise_matrix_sqrt,
            state_bounds,
            state: SVector::zeros(),
        }
    }

    /// Clear the accumulated error state (module reset).
    pub fn reset(&mut self) {
        self.state = SVector::zeros();
    }

    pub fn current_state(&self) -> SVector<f64, N> {
        self.state
    }

    /// Advance one step and return the new error state.
    pub fn compute_next_state<R: Rng>(&mut self, rng: &mut R) -> SVector<f64, N> {
        let normal = Normal::new(0.0, 1.0).expect("valid distribution");
        let draws = SVector::<f64, N>::from_fn(|_, _| normal.sample(rng));
        self.state = self.prop_matrix * self.state + self.noise_matrix_sqrt * draws;
        for i in 0..N {
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

    /// A two-state model whose propagation couples the states: noise drives the
    /// second state only, the first integrates it, `x0 += dt * x1` each step.
    #[test]
    fn propagation_couples_states_across_the_model() {
        use nalgebra::{Matrix2, Vector2};

        let dt = 0.1;
        let mut model = GaussMarkov::new(
            Matrix2::new(1.0, dt, 0.0, 1.0),
            Matrix2::new(0.0, 0.0, 0.0, 1.0e-2),
            Vector2::zeros(),
        );
        let mut rng = StdRng::seed_from_u64(5);

        let mut expected_first = 0.0;
        let mut previous_second = 0.0;
        for _ in 0..100 {
            let state = model.compute_next_state(&mut rng);
            expected_first += dt * previous_second;
            previous_second = state[1];
            assert!((state[0] - expected_first).abs() < 1.0e-15);
        }
        assert!(previous_second != 0.0 && expected_first != 0.0);
    }

    /// A one-state model draws from the same 1-sigma distribution.
    #[test]
    fn a_single_state_model_has_the_noise_entry_as_its_step_deviation() {
        use nalgebra::{Matrix1, Vector1};

        let mut model = GaussMarkov::new(Matrix1::zeros(), Matrix1::new(0.1), Vector1::zeros());
        let mut rng = StdRng::seed_from_u64(9);
        let n = 50_000;
        let sum_sq: f64 = (0..n)
            .map(|_| model.compute_next_state(&mut rng)[0].powi(2))
            .sum();
        let std = (sum_sq / n as f64).sqrt();
        assert!((std - 0.1).abs() < 0.005, "std {std}");
    }
}
