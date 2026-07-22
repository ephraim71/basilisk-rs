//! Generic explicit Runge-Kutta integrators, expressed as Butcher tableaus.
//!
//! Every fixed-step method is a value of [`ButcherTableau`] driven by
//! [`propagate_explicit_rk`]. Embedded (adaptive) pairs additionally provide a
//! lower-order weight row and are driven by [`propagate_adaptive_rk`], which
//! sub-steps internally to fill the fixed outer `dt` requested by the caller.
//!
//! Adding a new method is therefore just adding a tableau constant plus a thin
//! `fn` wrapper that matches the `SpacecraftConfig::integrator` pointer shape.

use super::traits::DynamicObject;
use hifitime::{Duration, Epoch};

/// An explicit Runge-Kutta method, described by its Butcher tableau.
///
/// Row `a[i]` holds the stage coefficients `a_{i,j}` for `j < i`, so it has
/// length `i` (row 0 is empty). `c[i]` is the node (fraction of the step) at
/// which stage `i` is evaluated, and `b[i]` are the solution weights.
pub struct ButcherTableau {
    /// Lower-triangular stage coefficients; `a[i]` has length `i`.
    pub a: &'static [&'static [f64]],
    /// Stage nodes, as fractions of the step.
    pub c: &'static [f64],
    /// Primary solution weights.
    pub b: &'static [f64],
    /// Embedded lower-order weights for adaptive error estimation, if the
    /// tableau is an embedded pair. `None` for plain fixed-step methods.
    pub b_embedded: Option<&'static [f64]>,
    /// Order of the error estimate, i.e. `min(p, p̂)` of the embedded pair.
    /// Used as the exponent denominator for step-size scaling.
    pub error_estimate_order: u32,
}

/// Nanosecond offset of a stage node, saturating so pathological steps can't
/// panic on overflow.
fn offset_nanos(base_nanos: u64, seconds: f64) -> u64 {
    base_nanos.saturating_add((seconds * 1.0e9).max(0.0) as u64)
}

/// Evaluates every stage derivative of `tableau` for a step of length `h`
/// starting at relative time `t_rel` from `(base_nanos, base_epoch)`.
fn compute_stages<D: DynamicObject>(
    tableau: &ButcherTableau,
    object: &mut D,
    y: &D::State,
    base_nanos: u64,
    base_epoch: Epoch,
    t_rel: f64,
    h: f64,
) -> Vec<D::Derivative> {
    let mut ks: Vec<D::Derivative> = Vec::with_capacity(tableau.c.len());
    for (i, &c_i) in tableau.c.iter().enumerate() {
        let weighted: Vec<(&D::Derivative, f64)> = tableau.a[i]
            .iter()
            .zip(ks.iter())
            .map(|(&a_ij, k_j)| (k_j, a_ij))
            .collect();
        let trial = object.state_with_derivatives(y, &weighted, h);
        let node_seconds = t_rel + c_i * h;
        ks.push(object.equations_of_motion(
            &trial,
            offset_nanos(base_nanos, node_seconds),
            base_epoch + Duration::from_seconds(node_seconds),
        ));
    }
    ks
}

/// Advances one fixed step using `tableau`'s primary weights.
pub fn propagate_explicit_rk<D: DynamicObject>(
    tableau: &ButcherTableau,
    object: &mut D,
    state: &D::State,
    current_sim_nanos: u64,
    current_epoch: Epoch,
    dt_seconds: f64,
) -> (D::State, D::StepOutput) {
    let ks = compute_stages(
        tableau,
        object,
        state,
        current_sim_nanos,
        current_epoch,
        0.0,
        dt_seconds,
    );
    let weighted = ks
        .into_iter()
        .zip(tableau.b.iter().copied())
        .collect::<Vec<_>>();
    object.combine(state, weighted, dt_seconds)
}

/// Tuning parameters for adaptive (embedded) Runge-Kutta step-size control.
#[derive(Clone, Copy, Debug)]
pub struct AdaptiveController {
    /// Absolute tolerance floor per state component.
    pub atol: f64,
    /// Relative tolerance per state component.
    pub rtol: f64,
    /// Safety factor applied to the ideal step scale (typically `0.9`).
    pub safety: f64,
    /// Smallest allowed step-shrink factor after a rejected step.
    pub min_scale: f64,
    /// Largest allowed step-growth factor after an accepted step.
    pub max_scale: f64,
    /// Hard cap on sub-steps per outer step, guarding against a stuck controller.
    pub max_substeps: u32,
}

impl Default for AdaptiveController {
    fn default() -> Self {
        Self {
            atol: 1.0e-9,
            rtol: 1.0e-9,
            safety: 0.9,
            min_scale: 0.2,
            max_scale: 5.0,
            max_substeps: 10_000,
        }
    }
}

/// Advances across the fixed outer step `dt_seconds` using an embedded pair,
/// sub-stepping internally so each accepted sub-step meets `controller`'s
/// tolerance. The per-sub-step [`StepOutput`](DynamicObject::StepOutput)s are
/// reduced into one via [`merge_step_outputs`](DynamicObject::merge_step_outputs).
///
/// Falls back to a single [`propagate_explicit_rk`] step if the tableau has no
/// embedded row or the object does not implement
/// [`error_norm`](DynamicObject::error_norm).
pub fn propagate_adaptive_rk<D: DynamicObject>(
    tableau: &ButcherTableau,
    controller: &AdaptiveController,
    object: &mut D,
    state: &D::State,
    current_sim_nanos: u64,
    current_epoch: Epoch,
    dt_seconds: f64,
) -> (D::State, D::StepOutput) {
    let Some(b_embedded) = tableau.b_embedded else {
        return propagate_explicit_rk(
            tableau,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        );
    };

    let exponent = 1.0 / (tableau.error_estimate_order as f64 + 1.0);
    let mut y = state.clone();
    let mut t_rel = 0.0f64;
    let mut h = dt_seconds;
    let mut accumulated: Option<D::StepOutput> = None;
    let mut substeps = 0u32;

    while dt_seconds - t_rel > 1.0e-12 * dt_seconds.abs().max(1.0) {
        h = h.min(dt_seconds - t_rel);
        let ks = compute_stages(
            tableau,
            object,
            &y,
            current_sim_nanos,
            current_epoch,
            t_rel,
            h,
        );

        // Error increment `y + h·Σ(bᵢ - b̂ᵢ)·kᵢ`, compared against `y`. Formed
        // before `combine` so the (non-linear) MRP shadow switch inside
        // `combine` cannot distort the estimate near the switching surface.
        let error = {
            let difference: Vec<(&D::Derivative, f64)> = ks
                .iter()
                .zip(tableau.b.iter().zip(b_embedded.iter()))
                .map(|(k, (&b_i, &b_hat_i))| (k, b_i - b_hat_i))
                .collect();
            let candidate = object.state_with_derivatives(&y, &difference, h);
            object.error_norm(&candidate, &y, controller.atol, controller.rtol)
        };

        let Some(error) = error else {
            // Object opts out of error control: take the remaining span as one step.
            let weighted = ks.into_iter().zip(tableau.b.iter().copied()).collect();
            let (next, out) = object.combine(&y, weighted, h);
            y = next;
            accumulated = Some(match accumulated {
                Some(acc) => object.merge_step_outputs(acc, out),
                None => out,
            });
            break;
        };

        if error <= 1.0 {
            let weighted = ks.into_iter().zip(tableau.b.iter().copied()).collect();
            let (next, out) = object.combine(&y, weighted, h);
            y = next;
            accumulated = Some(match accumulated {
                Some(acc) => object.merge_step_outputs(acc, out),
                None => out,
            });
            t_rel += h;
            let scale = if error == 0.0 {
                controller.max_scale
            } else {
                (controller.safety * error.powf(-exponent)).clamp(1.0, controller.max_scale)
            };
            h *= scale;
        } else {
            let scale =
                (controller.safety * error.powf(-exponent)).clamp(controller.min_scale, 1.0);
            h *= scale;
        }

        substeps += 1;
        assert!(
            substeps <= controller.max_substeps,
            "adaptive integrator exceeded {} sub-steps in one step; tolerances may be too tight",
            controller.max_substeps
        );
    }

    (
        y,
        accumulated.expect("adaptive integrator produced no accepted sub-step"),
    )
}

/// Forward (explicit) Euler, order 1. One evaluation per step.
pub const EULER: ButcherTableau = ButcherTableau {
    a: &[&[]],
    c: &[0.0],
    b: &[1.0],
    b_embedded: None,
    error_estimate_order: 1,
};

/// Explicit midpoint method, order 2. Two evaluations per step.
pub const MIDPOINT: ButcherTableau = ButcherTableau {
    a: &[&[], &[0.5]],
    c: &[0.0, 0.5],
    b: &[0.0, 1.0],
    b_embedded: None,
    error_estimate_order: 2,
};

/// Heun's method (explicit trapezoid), order 2. Two evaluations per step.
pub const HEUN: ButcherTableau = ButcherTableau {
    a: &[&[], &[1.0]],
    c: &[0.0, 1.0],
    b: &[0.5, 0.5],
    b_embedded: None,
    error_estimate_order: 2,
};

/// Ralston's method, order 2 with minimal truncation error. Two evaluations.
pub const RALSTON: ButcherTableau = ButcherTableau {
    a: &[&[], &[2.0 / 3.0]],
    c: &[0.0, 2.0 / 3.0],
    b: &[0.25, 0.75],
    b_embedded: None,
    error_estimate_order: 2,
};

/// Classic ("Kutta") RK4, order 4. Four evaluations per step.
pub const RK4: ButcherTableau = ButcherTableau {
    a: &[&[], &[0.5], &[0.0, 0.5], &[0.0, 0.0, 1.0]],
    c: &[0.0, 0.5, 0.5, 1.0],
    b: &[1.0 / 6.0, 2.0 / 6.0, 2.0 / 6.0, 1.0 / 6.0],
    b_embedded: None,
    error_estimate_order: 4,
};

/// The RK4 "3/8 rule", order 4 with smaller error constants than classic RK4.
pub const RK4_THREE_EIGHTHS: ButcherTableau = ButcherTableau {
    a: &[&[], &[1.0 / 3.0], &[-1.0 / 3.0, 1.0], &[1.0, -1.0, 1.0]],
    c: &[0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0],
    b: &[1.0 / 8.0, 3.0 / 8.0, 3.0 / 8.0, 1.0 / 8.0],
    b_embedded: None,
    error_estimate_order: 4,
};

/// Runge-Kutta-Fehlberg 4(5): propagates the 5th-order solution with a 4th-order
/// embedded estimate. Six evaluations per step.
pub const RKF45: ButcherTableau = ButcherTableau {
    a: &[
        &[],
        &[0.25],
        &[3.0 / 32.0, 9.0 / 32.0],
        &[1932.0 / 2197.0, -7200.0 / 2197.0, 7296.0 / 2197.0],
        &[439.0 / 216.0, -8.0, 3680.0 / 513.0, -845.0 / 4104.0],
        &[
            -8.0 / 27.0,
            2.0,
            -3544.0 / 2565.0,
            1859.0 / 4104.0,
            -11.0 / 40.0,
        ],
    ],
    c: &[0.0, 0.25, 3.0 / 8.0, 12.0 / 13.0, 1.0, 0.5],
    b: &[
        16.0 / 135.0,
        0.0,
        6656.0 / 12825.0,
        28561.0 / 56430.0,
        -9.0 / 50.0,
        2.0 / 55.0,
    ],
    b_embedded: Some(&[
        25.0 / 216.0,
        0.0,
        1408.0 / 2565.0,
        2197.0 / 4104.0,
        -1.0 / 5.0,
        0.0,
    ]),
    error_estimate_order: 4,
};

/// Dormand-Prince 5(4), the default of many general-purpose ODE solvers.
/// Propagates the 5th-order solution with a 4th-order embedded estimate.
/// Seven evaluations per step (FSAL, not exploited here).
pub const DOPRI45: ButcherTableau = ButcherTableau {
    a: &[
        &[],
        &[1.0 / 5.0],
        &[3.0 / 40.0, 9.0 / 40.0],
        &[44.0 / 45.0, -56.0 / 15.0, 32.0 / 9.0],
        &[
            19372.0 / 6561.0,
            -25360.0 / 2187.0,
            64448.0 / 6561.0,
            -212.0 / 729.0,
        ],
        &[
            9017.0 / 3168.0,
            -355.0 / 33.0,
            46732.0 / 5247.0,
            49.0 / 176.0,
            -5103.0 / 18656.0,
        ],
        &[
            35.0 / 384.0,
            0.0,
            500.0 / 1113.0,
            125.0 / 192.0,
            -2187.0 / 6784.0,
            11.0 / 84.0,
        ],
    ],
    c: &[0.0, 1.0 / 5.0, 3.0 / 10.0, 4.0 / 5.0, 8.0 / 9.0, 1.0, 1.0],
    b: &[
        35.0 / 384.0,
        0.0,
        500.0 / 1113.0,
        125.0 / 192.0,
        -2187.0 / 6784.0,
        11.0 / 84.0,
        0.0,
    ],
    b_embedded: Some(&[
        5179.0 / 57600.0,
        0.0,
        7571.0 / 16695.0,
        393.0 / 640.0,
        -92097.0 / 339200.0,
        187.0 / 2100.0,
        1.0 / 40.0,
    ]),
    error_estimate_order: 4,
};

#[cfg(test)]
mod tests;
