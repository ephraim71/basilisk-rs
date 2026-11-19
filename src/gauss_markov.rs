//! First-order bounded Gauss-Markov error model.
//!
//! [`GaussMarkov`] carries `N` error states, three unless a caller names
//! another count (one for a scalar sensor, twelve for a full navigation
//! state); its propagation and noise matrices may couple any pair of them.
//!
//! Two conventions callers must know:
//!
//! * **The noise matrix is in 3-sigma units.** Draws are `N(0, 1/3)`, so a
//!   `noise_matrix_sqrt` diagonal entry of `x` produces error steps with a
//!   standard deviation of `x / 3`. Specs quoted as 1-sigma must be
//!   multiplied by 3 on the way in.
//! * **Walk bounds repel rather than clamp.** As the state nears a bound the
//!   random draw is biased back toward zero by `exp(-((2B-|s|)/|s|)^3)^2`;
//!   the state may briefly exceed the bound and never sits pinned at it,
//!   unlike a hard clamp, which piles probability mass at the boundary.
//!   [`GaussMarkov::clamp_state`] applies upstream's current hard clamp
//!   instead, for a module whose upstream relies on it.

use nalgebra::{SMatrix, SVector};
use rand::Rng;
use rand_distr::{Distribution, Normal};

/// `N`-state bounded Gauss-Markov process (three states by default).
#[derive(Clone, Debug)]
pub struct GaussMarkov<const N: usize = 3> {
    /// Error-state propagation matrix. Zero reduces the model to independent
    /// per-sample noise; identity is a random walk.
    pub prop_matrix: SMatrix<f64, N, N>,
    /// Matrix square root of the noise covariance in **3-sigma units**;
    /// see the module docs.
    pub noise_matrix_sqrt: SMatrix<f64, N, N>,
    /// Per-state soft bounds; a non-positive bound leaves that state
    /// unbounded.
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

    /// Advance one step and return the new error state: propagate, draw
    /// `N(0, 1/3)` per state, apply the bound repulsion to the raw draw,
    /// then scale by the noise matrix and accumulate.
    pub fn compute_next_state<R: Rng>(&mut self, rng: &mut R) -> SVector<f64, N> {
        self.state = self.prop_matrix * self.state;

        let normal = Normal::new(0.0, 1.0 / 3.0).expect("valid distribution");
        let mut draws = SVector::<f64, N>::zeros();
        for i in 0..N {
            draws[i] = normal.sample(rng);
            let bound = self.state_bounds[i];
            if bound > 0.0 {
                draws[i] += bound_repulsion(self.state[i], bound);
            }
        }

        self.state += self.noise_matrix_sqrt * draws;
        self.state
    }

    /// Hard-limit each state whose entry in `bounds` is positive to
    /// `±bound`, and return the result: the bound of upstream's current
    /// `GaussMarkov`, which clips the state after every step instead of
    /// repelling it. A caller wanting it leaves
    /// [`state_bounds`](Self::state_bounds) at zero and calls this after each
    /// [`compute_next_state`](Self::compute_next_state); the clipped state is
    /// the one the next step propagates.
    pub fn clamp_state(&mut self, bounds: &SVector<f64, N>) -> SVector<f64, N> {
        for (state, &bound) in self.state.iter_mut().zip(bounds.iter()) {
            if bound > 0.0 && state.abs() > bound {
                *state = bound.copysign(*state);
            }
        }
        self.state
    }
}

/// Soft-bound repulsion: near-zero states fall back to evaluating at the
/// bound itself, and the repulsion is the *square* of the exponential.
fn bound_repulsion(state: f64, bound: f64) -> f64 {
    let state_calc = if state.abs() > bound * 1.0e-10 {
        state.abs()
    } else {
        bound
    };
    let mut bound_check = (bound * 2.0 - state_calc) / state_calc;
    bound_check = bound_check.max(bound * 1.0e-10);
    bound_check = 1.0 / (bound_check * bound_check * bound_check).exp();
    bound_check * bound_check.copysign(-state)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;

    use super::GaussMarkov;

    /// With zero propagation the process is IID and the output standard
    /// deviation is a third of the noise-matrix entry (3-sigma convention).
    #[test]
    fn three_sigma_convention_scales_draws_by_one_third() {
        let three_sigma = 3.0e-4;
        let mut model = GaussMarkov::new(
            Matrix3::zeros(),
            Matrix3::from_diagonal(&Vector3::repeat(three_sigma)),
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
        let expected = three_sigma / 3.0;
        assert!(
            (std - expected).abs() < 0.05 * expected,
            "std {std} vs expected {expected}"
        );
    }

    /// With identity propagation and bounds set, the walk is herded: it stays
    /// near the bounds without piling up hard at them, and transient
    /// excursions past the bound are permitted (soft, not clamped).
    #[test]
    fn soft_bounds_herd_the_walk_without_clamping() {
        let bound = 5.0e-4;
        let mut model = GaussMarkov::new(
            Matrix3::identity(),
            Matrix3::from_diagonal(&Vector3::repeat(1.0e-3)),
            Vector3::repeat(bound),
        );
        let mut rng = StdRng::seed_from_u64(11);

        let n = 20_000;
        let mut exceedances = 0usize;
        let mut at_bound_exactly = 0usize;
        let mut max_abs: f64 = 0.0;
        for _ in 0..n {
            let error = model.compute_next_state(&mut rng).x;
            max_abs = max_abs.max(error.abs());
            if error.abs() > bound {
                exceedances += 1;
            }
            if (error.abs() - bound).abs() < f64::EPSILON {
                at_bound_exactly += 1;
            }
        }
        // Herded: the walk cannot run far past the bound...
        assert!(
            max_abs < 4.0 * bound,
            "walk escaped the soft bound entirely: {max_abs}"
        );
        // ...but it is not a clamp: no samples sit exactly on the bound, and
        // transient exceedances exist.
        assert_eq!(at_bound_exactly, 0, "soft bound behaved like a clamp");
        assert!(exceedances > 0, "no excursions: this is a hard clamp");
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

    /// An off-diagonal propagation term couples two states: with noise on the
    /// second state only, the first integrates it, `x0 += dt * x1` each step.
    #[test]
    fn propagation_couples_states_across_the_model() {
        use nalgebra::{Matrix2, Vector2};

        let dt = 0.1;
        let mut model = GaussMarkov::new(
            Matrix2::new(1.0, dt, 0.0, 1.0),
            Matrix2::new(0.0, 0.0, 0.0, 3.0e-2),
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

    /// A one-state model draws from the same 3-sigma distribution.
    #[test]
    fn a_single_state_model_keeps_the_three_sigma_convention() {
        use nalgebra::{Matrix1, Vector1};

        let mut model = GaussMarkov::new(Matrix1::zeros(), Matrix1::new(0.3), Vector1::zeros());
        let mut rng = StdRng::seed_from_u64(9);
        let n = 50_000;
        let sum_sq: f64 = (0..n)
            .map(|_| model.compute_next_state(&mut rng)[0].powi(2))
            .sum();
        let std = (sum_sq / n as f64).sqrt();
        assert!((std - 0.1).abs() < 0.005, "std {std}");
    }

    /// The hard clamp pins a random walk at its bound, and the next step
    /// propagates the clipped state.
    #[test]
    fn clamp_state_pins_the_walk_at_the_bound() {
        use nalgebra::{Matrix2, Vector2};

        let bounds = Vector2::new(2.0e-3, 0.0);
        let mut model = GaussMarkov::new(
            Matrix2::identity(),
            Matrix2::from_diagonal(&Vector2::repeat(3.0e-3)),
            Vector2::zeros(),
        );
        let mut rng = StdRng::seed_from_u64(13);
        let mut pinned = 0;
        let mut unbounded_max: f64 = 0.0;
        for _ in 0..5_000 {
            let before = model.current_state();
            let raw = model.compute_next_state(&mut rng);
            let state = model.clamp_state(&bounds);
            assert!(state[0].abs() <= bounds[0]);
            assert_eq!(
                state[1], raw[1],
                "a non-positive bound leaves the state free"
            );
            assert!(
                (raw[0] - before[0]).abs() < 1.0e-2,
                "steps from the clipped state"
            );
            pinned += usize::from(state[0].abs() == bounds[0]);
            unbounded_max = unbounded_max.max(state[1].abs());
            assert_eq!(model.current_state(), state);
        }
        assert!(pinned > 0, "the walk never reached its bound");
        assert!(unbounded_max > bounds[0]);
    }
}
