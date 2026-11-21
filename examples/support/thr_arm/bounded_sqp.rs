//! A bound-constrained quasi-Newton minimizer: the stand-in for the
//! `scipy.optimize.minimize(..., bounds=..., method="SLSQP")` call of
//! upstream's `jointThrAllocation`.
//!
//! With bounds as its only constraints, SLSQP is a sequential quadratic
//! programming method whose subproblem is a box-constrained quadratic
//! program on a BFGS Hessian, followed by a line search on the objective.
//! [`minimize`] is that method written out:
//!
//! 1. start from `x0` clipped into the box, with the identity Hessian `B`;
//! 2. solve `min ½ dᵀ B d + gᵀ d` subject to `l ≤ x + d ≤ u`
//!    ([`box_qp`], a primal active-set method);
//! 3. stop with success when `|gᵀ d| < ftol`, the first-order condition in
//!    the box (SLSQP's `h1 < acc` test);
//! 4. backtrack from the full step until the objective decreases
//!    sufficiently (Armijo, `c = 10⁻⁴`, at most ten halvings, SLSQP's limit
//!    of line-search iterations);
//! 5. stop with success when the step or the change of the objective is
//!    below `ftol` (SLSQP's `|f - f0| < acc` or `‖s‖ < acc`);
//! 6. update `B` with Powell's damped BFGS formula (as SLSQP does) and go
//!    to 2, failing after `max_iterations` iterations or a line search that
//!    finds no decrease.
//!
//! Gradients are forward differences with SciPy's default step,
//! `√ε · max(1, |xᵢ|)` signed like `xᵢ` and reversed at an upper bound.
//!
//! Deviation from upstream: SciPy's Fortran SLSQP differs in its subproblem
//! solver (an `LSQ` on the `LDLᵀ` factors), its merit-function line search
//! and its numerical details, so the two converge to different points of
//! the same local minima, or to different local minima from the same seed.
//! The scenarios' checks do not depend on which allocation is chosen
//! (upstream's tests note that "equivalent optimizer solutions" exist).

#![allow(dead_code)]

use nalgebra::{DMatrix, DVector};

/// Options of [`minimize`]: upstream's `maxiter` and `ftol`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MinimizeOptions {
    pub max_iterations: usize,
    pub ftol: f64,
}

impl Default for MinimizeOptions {
    /// `jointThrAllocation`'s defaults: 3000 iterations, `ftol = 1e-10`.
    fn default() -> Self {
        Self {
            max_iterations: 3000,
            ftol: 1e-10,
        }
    }
}

/// The result of [`minimize`]: SciPy's `OptimizeResult.x`, `.fun`,
/// `.success` and `.nit`.
#[derive(Clone, Debug, PartialEq)]
pub struct Minimum {
    pub x: DVector<f64>,
    pub f: f64,
    pub success: bool,
    pub iterations: usize,
}

/// `x` clipped into `[lower, upper]`.
fn clip(x: &DVector<f64>, lower: &DVector<f64>, upper: &DVector<f64>) -> DVector<f64> {
    DVector::from_iterator(
        x.len(),
        x.iter()
            .zip(lower.iter().zip(upper.iter()))
            .map(|(value, (low, high))| value.max(*low).min(*high)),
    )
}

/// Forward-difference gradient of `f` at `x` (value `fx`), stepping inside
/// the box.
fn gradient(
    f: &mut impl FnMut(&DVector<f64>) -> f64,
    x: &DVector<f64>,
    fx: f64,
    upper: &DVector<f64>,
) -> DVector<f64> {
    let relative_step = f64::EPSILON.sqrt();
    let mut probe = x.clone();
    DVector::from_iterator(
        x.len(),
        (0..x.len()).map(|index| {
            let sign = if x[index] >= 0.0 { 1.0 } else { -1.0 };
            let mut step = relative_step * sign * x[index].abs().max(1.0);
            if x[index] + step > upper[index] {
                step = -step;
            }
            // The step actually taken in floating point.
            let shifted = x[index] + step;
            let step = shifted - x[index];
            probe[index] = shifted;
            let value = f(&probe);
            probe[index] = x[index];
            (value - fx) / step
        }),
    )
}

/// `min ½ dᵀ B d + gᵀ d` subject to `low ≤ d ≤ high` (with `low ≤ 0 ≤
/// high`) for a symmetric positive definite `B`: a primal active-set method
/// started from `d = 0`.
pub fn box_qp(
    b: &DMatrix<f64>,
    g: &DVector<f64>,
    low: &DVector<f64>,
    high: &DVector<f64>,
) -> DVector<f64> {
    let n = g.len();
    let mut d = DVector::zeros(n);
    // Bound each variable starts at, when the gradient pushes it outward.
    let mut active: Vec<Option<bool>> = (0..n)
        .map(|index| {
            if low[index] >= 0.0 && g[index] > 0.0 {
                Some(false)
            } else if high[index] <= 0.0 && g[index] < 0.0 {
                Some(true)
            } else {
                None
            }
        })
        .collect();
    for _ in 0..(10 * n + 10) {
        let free: Vec<usize> = (0..n).filter(|index| active[*index].is_none()).collect();
        // The minimizer over the free variables, the active ones held.
        let mut target = d.clone();
        for (index, bound) in active.iter().enumerate() {
            if let Some(upper) = bound {
                target[index] = if *upper { high[index] } else { low[index] };
            }
        }
        if !free.is_empty() {
            let rhs = DVector::from_iterator(
                free.len(),
                free.iter().map(|&row| {
                    let mut value = -g[row];
                    for (column, bound) in active.iter().enumerate() {
                        if bound.is_some() {
                            value -= b[(row, column)] * target[column];
                        }
                    }
                    value
                }),
            );
            let reduced = DMatrix::from_fn(free.len(), free.len(), |r, c| b[(free[r], free[c])]);
            let solution = match reduced.clone().cholesky() {
                Some(cholesky) => cholesky.solve(&rhs),
                None => reduced
                    .lu()
                    .solve(&rhs)
                    .unwrap_or_else(|| DVector::zeros(free.len())),
            };
            for (slot, &index) in free.iter().enumerate() {
                target[index] = solution[slot];
            }
        }
        // Step from d toward the target until a free variable hits a bound.
        let direction = &target - &d;
        let mut step = 1.0;
        let mut blocking = None;
        for &index in &free {
            let delta = direction[index];
            if delta > 0.0 && d[index] + delta > high[index] {
                let ratio = (high[index] - d[index]) / delta;
                if ratio < step {
                    step = ratio;
                    blocking = Some((index, true));
                }
            } else if delta < 0.0 && d[index] + delta < low[index] {
                let ratio = (low[index] - d[index]) / delta;
                if ratio < step {
                    step = ratio;
                    blocking = Some((index, false));
                }
            }
        }
        d += direction * step;
        if let Some((index, upper)) = blocking {
            d[index] = if upper { high[index] } else { low[index] };
            active[index] = Some(upper);
            continue;
        }
        // At the subspace minimum: release the bound whose multiplier has
        // the wrong sign the most.
        let grad = b * &d + g;
        let mut release = None;
        let mut worst = 0.0;
        for (index, bound) in active.iter().enumerate() {
            let violation = match bound {
                // At a lower bound the gradient must not point inward.
                Some(false) => -grad[index],
                Some(true) => grad[index],
                None => continue,
            };
            if violation > worst {
                worst = violation;
                release = Some(index);
            }
        }
        match release {
            Some(index) => active[index] = None,
            None => break,
        }
    }
    clip(&d, low, high)
}

/// Minimizes `f` over `[lower, upper]` from `x0` (see the module docs).
pub fn minimize(
    mut f: impl FnMut(&DVector<f64>) -> f64,
    x0: &DVector<f64>,
    lower: &DVector<f64>,
    upper: &DVector<f64>,
    options: MinimizeOptions,
) -> Minimum {
    let n = x0.len();
    let mut x = clip(x0, lower, upper);
    let mut fx = f(&x);
    let mut g = gradient(&mut f, &x, fx, upper);
    let mut b = DMatrix::identity(n, n);
    let result = |x: DVector<f64>, f: f64, success: bool, iterations: usize| Minimum {
        x,
        f,
        success,
        iterations,
    };
    if !fx.is_finite() || g.iter().any(|value| !value.is_finite()) {
        return result(x, fx, false, 0);
    }
    for iteration in 1..=options.max_iterations {
        let mut d = box_qp(&b, &g, &(lower - &x), &(upper - &x));
        let mut slope = g.dot(&d);
        if slope.abs() < options.ftol {
            return result(x, fx, true, iteration);
        }
        if slope > 0.0 {
            // Not a descent direction: restart from the identity Hessian.
            b = DMatrix::identity(n, n);
            d = box_qp(&b, &g, &(lower - &x), &(upper - &x));
            slope = g.dot(&d);
            if slope >= 0.0 {
                return result(x, fx, slope.abs() < options.ftol, iteration);
            }
        }
        let mut alpha = 1.0;
        let mut accepted = None;
        for _ in 0..10 {
            let candidate = clip(&(&x + &d * alpha), lower, upper);
            let value = f(&candidate);
            if value.is_finite() && value <= fx + 1e-4 * alpha * slope {
                accepted = Some((candidate, value));
                break;
            }
            alpha *= 0.5;
        }
        let Some((x_new, f_new)) = accepted else {
            // SLSQP's "positive directional derivative for linesearch".
            return result(x, fx, false, iteration);
        };
        let s = &x_new - &x;
        if (f_new - fx).abs() < options.ftol || s.norm() < options.ftol {
            return result(x_new, f_new, true, iteration);
        }
        let g_new = gradient(&mut f, &x_new, f_new, upper);
        if g_new.iter().any(|value| !value.is_finite()) {
            return result(x_new, f_new, false, iteration);
        }
        // Powell-damped BFGS update, which keeps B positive definite.
        let mut y = &g_new - &g;
        let bs = &b * &s;
        let sbs = s.dot(&bs);
        let sy = s.dot(&y);
        if sbs > 0.0 {
            if sy < 0.2 * sbs {
                let theta = 0.8 * sbs / (sbs - sy);
                y = &y * theta + &bs * (1.0 - theta);
            }
            let sy = s.dot(&y);
            b = &b - &bs * bs.transpose() / sbs + &y * y.transpose() / sy;
        }
        x = x_new;
        fx = f_new;
        g = g_new;
    }
    result(x, fx, false, options.max_iterations)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn vector(values: &[f64]) -> DVector<f64> {
        DVector::from_column_slice(values)
    }

    /// An unconstrained quadratic is solved to its stationary point.
    #[test]
    fn interior_quadratic_minimum() {
        let target = vector(&[0.3, -0.7, 1.2]);
        let minimum = minimize(
            |x| (x - &target).norm_squared() + 0.5 * (x[0] - x[1]).powi(2),
            &vector(&[0.0, 0.0, 0.0]),
            &vector(&[-5.0; 3]),
            &vector(&[5.0; 3]),
            MinimizeOptions::default(),
        );
        assert!(minimum.success);
        // Stationary point of the quadratic: solve its normal equations.
        let hessian =
            DMatrix::from_row_slice(3, 3, &[3.0, -1.0, 0.0, -1.0, 3.0, 0.0, 0.0, 0.0, 2.0]);
        let expected = hessian.lu().solve(&(target * 2.0)).unwrap();
        assert!((minimum.x - expected).norm() < 1e-5);
    }

    /// The minimum of a quadratic outside the box lies on the bounds.
    #[test]
    fn bound_constrained_minimum() {
        let minimum = minimize(
            |x| (x[0] - 3.0).powi(2) + (x[1] + 2.0).powi(2) + x[2].powi(2),
            &vector(&[0.5, 0.5, 0.5]),
            &vector(&[0.0, -1.0, -1.0]),
            &vector(&[1.0, 1.0, 1.0]),
            MinimizeOptions::default(),
        );
        assert!(minimum.success);
        assert!((minimum.x - vector(&[1.0, -1.0, 0.0])).norm() < 1e-6);
    }

    /// Rosenbrock's valley, a classic nonlinear test.
    #[test]
    fn rosenbrock() {
        let minimum = minimize(
            |x| 100.0 * (x[1] - x[0] * x[0]).powi(2) + (1.0 - x[0]).powi(2),
            &vector(&[-1.2, 1.0]),
            &vector(&[-2.0, -2.0]),
            &vector(&[2.0, 2.0]),
            MinimizeOptions::default(),
        );
        assert!(minimum.success);
        assert!(
            (&minimum.x - vector(&[1.0, 1.0])).norm() < 1e-3,
            "{}",
            minimum.x
        );
    }

    /// The box QP solves a problem whose optimum has active and free
    /// variables.
    #[test]
    fn box_qp_mixed_active_set() {
        let b = DMatrix::from_row_slice(2, 2, &[2.0, 0.5, 0.5, 1.0]);
        let g = vector(&[-4.0, 1.0]);
        let d = box_qp(&b, &g, &vector(&[-1.0, -1.0]), &vector(&[1.0, 1.0]));
        // d0 at its upper bound, d1 minimizing 0.5 d1² + (1 + 0.5) d1.
        assert!((d - vector(&[1.0, -1.0])).norm() < 1e-12);
    }
}
