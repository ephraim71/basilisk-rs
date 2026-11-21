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
#[derive(Clone, Copy, Debug, PartialEq)]
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

impl ButcherTableau {
    /// Checks a (custom) tableau the way upstream's `svIntegratorRungeKutta`
    /// and `svIntegratorAdaptiveRungeKutta` constructors do: matching stage
    /// counts, finite coefficients and an explicit (strictly lower
    /// triangular) stage matrix. Row `a[i]` with more than `i` entries is
    /// accepted only if the extra entries are zero. The drivers do not call
    /// this; it is for validating a tableau once, where it is built.
    pub fn validate(&self) -> Result<(), String> {
        let stages = self.c.len();
        if self.a.len() != stages || self.b.len() != stages {
            return Err(format!(
                "Runge-Kutta tableau dimensions disagree: {} rows of a, {} nodes c, {} weights b",
                self.a.len(),
                stages,
                self.b.len()
            ));
        }
        if let Some(b_embedded) = self.b_embedded {
            if b_embedded.len() != stages {
                return Err(format!(
                    "Runge-Kutta tableau has {stages} stages but {} embedded weights",
                    b_embedded.len()
                ));
            }
            if b_embedded.iter().any(|value| !value.is_finite()) {
                return Err("Runge-Kutta embedded weights must be finite".to_string());
            }
        }
        if self.error_estimate_order == 0 {
            return Err("Runge-Kutta error-estimate order must be finite and positive".to_string());
        }
        for (row_index, row) in self.a.iter().enumerate() {
            if row.len() > stages {
                return Err(format!(
                    "Runge-Kutta tableau row {row_index} has {} entries for {stages} stages",
                    row.len()
                ));
            }
            if row.iter().any(|value| !value.is_finite()) {
                return Err("Runge-Kutta stage coefficients must be finite".to_string());
            }
            if row.iter().skip(row_index).any(|&value| value != 0.0) {
                return Err(
                    "Runge-Kutta stage coefficients must be strictly lower triangular".to_string(),
                );
            }
        }
        if self.b.iter().chain(self.c).any(|value| !value.is_finite()) {
            return Err("Runge-Kutta weights and nodes must be finite".to_string());
        }
        Ok(())
    }
}

/// Nanosecond offset of a stage node, saturating so pathological steps can't
/// panic on overflow.
pub(super) fn offset_nanos(base_nanos: u64, seconds: f64) -> u64 {
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

/// How [`propagate_adaptive_rk`] measures a trial's error, which solution it
/// keeps and how it sizes the next trial.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StepControl {
    /// basilisk-rs's scheme: the error is the weighted RMS norm of
    /// [`DynamicObject::error_norm`] over every scalar, an accepted trial
    /// keeps the `b` (higher-order) solution, and the step scale
    /// `safety · err^(-1/(q+1))` is clamped to `[1, max_scale]` after an
    /// accepted trial and to `[min_scale, 1]` after a rejected one.
    #[default]
    RmsNorm,
    /// Upstream `svIntegratorAdaptiveRungeKutta::integrateImpl`:
    ///
    /// * every outer step starts with the trial step equal to the whole step;
    /// * each trial forms the `b` (upstream `bArray`) and `b_embedded`
    ///   (`bStarArray`) candidates and accepts when the largest error ratio
    ///   over the states, [`DynamicObject::max_relative_error`]
    ///   (`computeFlatMaxRelativeError`), is at most one;
    /// * an accepted trial keeps the `b_embedded` candidate, as upstream keeps
    ///   `bStarArray`'s (for [`RKF45`] and [`RKF78`] the lower-order one);
    /// * the next trial step is `safety · h · (1/err)^(1/p)`, clamped to
    ///   `[min_scale · h, max_scale · h]` and to the rest of the outer step,
    ///   with `p = error_estimate_order + 1` (upstream `methodLargestOrder`).
    ///
    /// Use it with [`AdaptiveController::upstream`]'s factors and tolerances.
    /// An object that does not implement `max_relative_error` takes the
    /// whole step at once.
    Upstream,
}

/// Tuning parameters for adaptive (embedded) Runge-Kutta step-size control.
#[derive(Clone, Copy, Debug)]
pub struct AdaptiveController {
    /// Absolute tolerance floor per state component (upstream `absTol`).
    pub atol: f64,
    /// Relative tolerance per state component (upstream `relTol`).
    pub rtol: f64,
    /// Safety factor applied to the ideal step scale (typically `0.9`;
    /// upstream `safetyFactorForNextStepSize`).
    pub safety: f64,
    /// Smallest allowed step-shrink factor (upstream
    /// `minimumFactorDecreaseForNextStepSize`).
    pub min_scale: f64,
    /// Largest allowed step-growth factor (upstream
    /// `maximumFactorIncreaseForNextStepSize`).
    pub max_scale: f64,
    /// Hard cap on sub-steps per outer step, guarding against a stuck controller.
    pub max_substeps: u32,
    /// The step-size control scheme.
    pub step_control: StepControl,
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
            step_control: StepControl::RmsNorm,
        }
    }
}

impl AdaptiveController {
    /// Upstream `svIntegratorAdaptiveRungeKutta`'s step control with its
    /// defaults: `relTol = 1e-4`, `absTol = 1e-8`, safety 0.9, step factors
    /// in `[0.1, 4]`. Set `rtol` and `atol` for other tolerances.
    pub fn upstream() -> Self {
        Self {
            atol: 1.0e-8,
            rtol: 1.0e-4,
            safety: 0.9,
            min_scale: 0.1,
            max_scale: 4.0,
            max_substeps: 10_000,
            step_control: StepControl::Upstream,
        }
    }
}

/// Upstream's error-to-tolerance ratio of one state: `error / (magnitude ·
/// rel_tol + abs_tol)`, where `error` is the norm of the high-minus-low
/// candidate difference and `magnitude` the norm of the high-order candidate
/// (whole-state control), or the absolute values of one component
/// (per-component control). A zero threshold gives 0 for a zero error and
/// infinity otherwise.
///
/// # Panics
///
/// On a non-finite error or magnitude, as upstream raises.
pub fn relative_error_ratio(error: f64, magnitude: f64, rel_tol: f64, abs_tol: f64) -> f64 {
    assert!(
        error.is_finite() && magnitude.is_finite(),
        "adaptive Runge-Kutta produced a nonfinite state or truncation error"
    );
    let threshold = magnitude * rel_tol + abs_tol;
    if threshold == 0.0 {
        if error == 0.0 { 0.0 } else { f64::INFINITY }
    } else {
        error / threshold
    }
}

/// Advances across the fixed outer step `dt_seconds` using an embedded pair,
/// sub-stepping internally so each accepted sub-step meets `controller`'s
/// tolerance, with the scheme of its [`StepControl`]. The per-sub-step
/// [`StepOutput`](DynamicObject::StepOutput)s are reduced into one via
/// [`merge_step_outputs`](DynamicObject::merge_step_outputs).
///
/// Falls back to a single [`propagate_explicit_rk`] step if the tableau has no
/// embedded row, and takes the rest of the outer step at once if the object
/// does not implement the error measure of the scheme
/// ([`error_norm`](DynamicObject::error_norm) or
/// [`max_relative_error`](DynamicObject::max_relative_error)).
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

    if controller.step_control == StepControl::Upstream {
        return propagate_upstream_adaptive_rk(
            tableau,
            b_embedded,
            controller,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        );
    }

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

/// [`StepControl::Upstream`]: port of upstream
/// `svIntegratorAdaptiveRungeKutta::integrateImpl` (see there).
#[allow(clippy::too_many_arguments)]
fn propagate_upstream_adaptive_rk<D: DynamicObject>(
    tableau: &ButcherTableau,
    b_embedded: &[f64],
    controller: &AdaptiveController,
    object: &mut D,
    state: &D::State,
    current_sim_nanos: u64,
    current_epoch: Epoch,
    dt_seconds: f64,
) -> (D::State, D::StepOutput) {
    // Upstream `validateTolerance`.
    for (tolerance, name) in [(controller.rtol, "relative"), (controller.atol, "absolute")] {
        assert!(
            tolerance.is_finite() && tolerance >= 0.0,
            "adaptive Runge-Kutta {name} tolerance must be finite and nonnegative"
        );
    }
    assert!(
        dt_seconds.is_finite() && dt_seconds >= 0.0,
        "adaptive Runge-Kutta integration requires a finite, nonnegative time step"
    );
    if dt_seconds == 0.0 {
        return propagate_explicit_rk(
            tableau,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        );
    }

    // Upstream `methodLargestOrder`.
    let largest_order = tableau.error_estimate_order as f64 + 1.0;
    let mut base = state.clone();
    let mut elapsed = 0.0_f64;
    let mut step = dt_seconds;
    let mut accumulated: Option<D::StepOutput> = None;
    let mut substeps = 0u32;
    while elapsed < dt_seconds {
        assert!(
            step.is_finite() && step > 0.0 && elapsed + step > elapsed,
            "adaptive Runge-Kutta integration cannot make representable forward progress"
        );
        let ks = compute_stages(
            tableau,
            object,
            &base,
            current_sim_nanos,
            current_epoch,
            elapsed,
            step,
        );
        let max_error = {
            let low: Vec<(&D::Derivative, f64)> =
                ks.iter().zip(tableau.b.iter().copied()).collect();
            let high: Vec<(&D::Derivative, f64)> =
                ks.iter().zip(b_embedded.iter().copied()).collect();
            object.max_relative_error(&base, &low, &high, step, controller.atol, controller.rtol)
        };
        let Some(max_error) = max_error else {
            // The object opts out of error control: take the rest at once.
            let weighted = ks.into_iter().zip(b_embedded.iter().copied()).collect();
            let (next, out) = object.combine(&base, weighted, dt_seconds - elapsed);
            base = next;
            accumulated = Some(match accumulated {
                Some(acc) => object.merge_step_outputs(acc, out),
                None => out,
            });
            break;
        };
        assert!(
            !max_error.is_nan(),
            "adaptive Runge-Kutta produced a nonfinite truncation error"
        );

        if max_error <= 1.0 {
            elapsed += step;
            let weighted = ks.into_iter().zip(b_embedded.iter().copied()).collect();
            let (next, out) = object.combine(&base, weighted, step);
            base = next;
            accumulated = Some(match accumulated {
                Some(acc) => object.merge_step_outputs(acc, out),
                None => out,
            });
        }

        let next_step = (controller.safety * step * (1.0 / max_error).powf(1.0 / largest_order))
            .min(step * controller.max_scale)
            .max(step * controller.min_scale)
            .min(dt_seconds - elapsed);
        if elapsed < dt_seconds {
            assert!(
                next_step.is_finite() && next_step > 0.0 && elapsed + next_step > elapsed,
                "adaptive Runge-Kutta error control reduced the time step below representable \
                 forward progress"
            );
        }
        step = next_step;

        substeps += 1;
        assert!(
            substeps <= controller.max_substeps,
            "adaptive integrator exceeded {} sub-steps in one step; tolerances may be too tight",
            controller.max_substeps
        );
    }

    (
        base,
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

/// Runge-Kutta-Fehlberg 7(8), the tableau of upstream `svIntegratorRKF78`
/// (Fehlberg 1968, NASA TR R-287). Thirteen evaluations per step.
///
/// Like [`RKF45`] this propagates the higher-order (8th) solution and uses the
/// 7th-order one as the embedded estimate; upstream propagates the 7th-order
/// solution (`bStarArray`). The difference of the two, and so the step-size
/// control with its `1/8` exponent, is the same.
pub const RKF78: ButcherTableau = ButcherTableau {
    a: &[
        &[],
        &[2.0 / 27.0],
        &[1.0 / 36.0, 1.0 / 12.0],
        &[1.0 / 24.0, 0.0, 1.0 / 8.0],
        &[5.0 / 12.0, 0.0, -25.0 / 16.0, 25.0 / 16.0],
        &[1.0 / 20.0, 0.0, 0.0, 1.0 / 4.0, 1.0 / 5.0],
        &[
            -25.0 / 108.0,
            0.0,
            0.0,
            125.0 / 108.0,
            -65.0 / 27.0,
            125.0 / 54.0,
        ],
        &[
            31.0 / 300.0,
            0.0,
            0.0,
            0.0,
            61.0 / 225.0,
            -2.0 / 9.0,
            13.0 / 900.0,
        ],
        &[
            2.0,
            0.0,
            0.0,
            -53.0 / 6.0,
            704.0 / 45.0,
            -107.0 / 9.0,
            67.0 / 90.0,
            3.0,
        ],
        &[
            -91.0 / 108.0,
            0.0,
            0.0,
            23.0 / 108.0,
            -976.0 / 135.0,
            311.0 / 54.0,
            -19.0 / 60.0,
            17.0 / 6.0,
            -1.0 / 12.0,
        ],
        &[
            2383.0 / 4100.0,
            0.0,
            0.0,
            -341.0 / 164.0,
            4496.0 / 1025.0,
            -301.0 / 82.0,
            2133.0 / 4100.0,
            45.0 / 82.0,
            45.0 / 164.0,
            18.0 / 41.0,
        ],
        &[
            3.0 / 205.0,
            0.0,
            0.0,
            0.0,
            0.0,
            -6.0 / 41.0,
            -3.0 / 205.0,
            -3.0 / 41.0,
            3.0 / 41.0,
            6.0 / 41.0,
            0.0,
        ],
        &[
            -1777.0 / 4100.0,
            0.0,
            0.0,
            -341.0 / 164.0,
            4496.0 / 1025.0,
            -289.0 / 82.0,
            2193.0 / 4100.0,
            51.0 / 82.0,
            33.0 / 164.0,
            12.0 / 41.0,
            0.0,
            1.0,
        ],
    ],
    c: &[
        0.0,
        2.0 / 27.0,
        1.0 / 9.0,
        1.0 / 6.0,
        5.0 / 12.0,
        1.0 / 2.0,
        5.0 / 6.0,
        1.0 / 6.0,
        2.0 / 3.0,
        1.0 / 3.0,
        1.0,
        0.0,
        1.0,
    ],
    b: &[
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        34.0 / 105.0,
        9.0 / 35.0,
        9.0 / 35.0,
        9.0 / 280.0,
        9.0 / 280.0,
        0.0,
        41.0 / 840.0,
        41.0 / 840.0,
    ],
    b_embedded: Some(&[
        41.0 / 840.0,
        0.0,
        0.0,
        0.0,
        0.0,
        34.0 / 105.0,
        9.0 / 35.0,
        9.0 / 35.0,
        9.0 / 280.0,
        9.0 / 280.0,
        41.0 / 840.0,
        0.0,
        0.0,
    ]),
    error_estimate_order: 7,
};

#[cfg(test)]
mod tests;
