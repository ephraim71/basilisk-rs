//! Physics-independent verification of the integrator machinery.
//!
//! We integrate the harmonic oscillator `x'' = -x` (equivalently the first-order
//! system `[x, v]' = [v, -x]`) from `x(0) = 1, v(0) = 0`, whose exact solution is
//! `x(t) = cos t`, `v(t) = -sin t`. This exercises the Butcher driver, the
//! `state_with_derivatives` chaining and the adaptive controller.

use super::*;
use crate::integrators::traits::DynamicObject;
use hifitime::Epoch;

/// `[position, velocity]` harmonic oscillator with unit angular frequency.
struct Harmonic;

impl DynamicObject for Harmonic {
    type State = [f64; 2];
    type Derivative = [f64; 2];
    type StepOutput = f64;

    fn equations_of_motion(&mut self, state: &[f64; 2], _: u64, _: Epoch) -> [f64; 2] {
        [state[1], -state[0]]
    }

    fn state_with_derivative(&self, state: &[f64; 2], derivative: &[f64; 2], dt: f64) -> [f64; 2] {
        [state[0] + derivative[0] * dt, state[1] + derivative[1] * dt]
    }

    fn combine(
        &self,
        state: &[f64; 2],
        weighted: Vec<([f64; 2], f64)>,
        step: f64,
    ) -> ([f64; 2], f64) {
        let mut increment = [0.0f64; 2];
        for (derivative, weight) in &weighted {
            increment[0] += derivative[0] * weight;
            increment[1] += derivative[1] * weight;
        }
        // Report the sub-step length as the (additive) step output, so we can
        // assert that adaptive sub-steps are summed back to the outer `dt`.
        (
            [
                state[0] + increment[0] * step,
                state[1] + increment[1] * step,
            ],
            step,
        )
    }

    fn merge_step_outputs(&self, accumulated: f64, next: f64) -> f64 {
        accumulated + next
    }

    fn error_norm(&self, high: &[f64; 2], low: &[f64; 2], atol: f64, rtol: f64) -> Option<f64> {
        let mut sum_of_squares = 0.0;
        for axis in 0..2 {
            let scale = atol + rtol * high[axis].abs().max(low[axis].abs());
            let scaled = (high[axis] - low[axis]) / scale;
            sum_of_squares += scaled * scaled;
        }
        Some((sum_of_squares / 2.0).sqrt())
    }
}

fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2020, 1, 1)
}

/// Integrates over `[0, t_final]` in `steps` fixed steps and returns the final state.
fn integrate_fixed(tableau: &ButcherTableau, t_final: f64, steps: usize) -> [f64; 2] {
    let dt = t_final / steps as f64;
    let mut object = Harmonic;
    let mut state = [1.0, 0.0];
    for step in 0..steps {
        let t = step as f64 * dt;
        let nanos = (t * 1.0e9) as u64;
        (state, _) = propagate_explicit_rk(tableau, &mut object, &state, nanos, epoch(), dt);
    }
    state
}

fn final_error(tableau: &ButcherTableau, t_final: f64, steps: usize) -> f64 {
    let state = integrate_fixed(tableau, t_final, steps);
    let exact = [t_final.cos(), -t_final.sin()];
    ((state[0] - exact[0]).powi(2) + (state[1] - exact[1]).powi(2)).sqrt()
}

/// Empirical order of convergence from halving the step size:
/// `error(n) / error(2n) ≈ 2^order`.
fn observed_order(tableau: &ButcherTableau) -> f64 {
    let t_final = 1.0;
    let coarse = final_error(tableau, t_final, 20);
    let fine = final_error(tableau, t_final, 40);
    (coarse / fine).log2()
}

#[test]
fn euler_is_first_order() {
    let order = observed_order(&EULER);
    assert!((order - 1.0).abs() < 0.15, "euler order was {order}");
}

#[test]
fn midpoint_and_heun_and_ralston_are_second_order() {
    for tableau in [&MIDPOINT, &HEUN, &RALSTON] {
        let order = observed_order(tableau);
        assert!((order - 2.0).abs() < 0.2, "order was {order}");
    }
}

#[test]
fn rk4_variants_are_fourth_order() {
    for tableau in [&RK4, &RK4_THREE_EIGHTHS] {
        let order = observed_order(tableau);
        assert!((order - 4.0).abs() < 0.3, "order was {order}");
    }
}

#[test]
fn rk4_matches_reference_solution_closely() {
    // 100 steps over one radian is already accurate to well under 1e-8.
    let error = final_error(&RK4, 1.0, 100);
    assert!(error < 1.0e-8, "rk4 error was {error}");
}

/// The 3/8-rule and classic RK4 are both order 4 but use different tableaus;
/// their solutions should be close yet not bit-identical, confirming the
/// multi-derivative stage assembly actually differs between them.
#[test]
fn three_eighths_rule_differs_from_classic_rk4() {
    let classic = integrate_fixed(&RK4, 1.0, 100);
    let three_eighths = integrate_fixed(&RK4_THREE_EIGHTHS, 1.0, 100);
    let difference =
        ((classic[0] - three_eighths[0]).powi(2) + (classic[1] - three_eighths[1]).powi(2)).sqrt();
    assert!(difference > 0.0, "tableaus produced identical output");
    assert!(
        difference < 1.0e-6,
        "tableaus diverged unexpectedly: {difference}"
    );
}

/// Adaptive integration over a single large outer step must sub-step to hit the
/// requested tolerance, landing far closer than a single fixed RK4 step would.
#[test]
fn adaptive_dopri_meets_tolerance_over_large_step() {
    let controller = AdaptiveController {
        atol: 1.0e-10,
        rtol: 1.0e-10,
        ..Default::default()
    };
    let mut object = Harmonic;
    let dt = 3.0; // deliberately large: a single RK4 step here is very inaccurate.
    let (state, _) = propagate_adaptive_rk(
        &DOPRI45,
        &controller,
        &mut object,
        &[1.0, 0.0],
        0,
        epoch(),
        dt,
    );
    let exact = [dt.cos(), -dt.sin()];
    let error = ((state[0] - exact[0]).powi(2) + (state[1] - exact[1]).powi(2)).sqrt();
    assert!(error < 1.0e-7, "adaptive dopri error was {error}");

    let single_rk4 = integrate_fixed(&RK4, dt, 1);
    let rk4_error =
        ((single_rk4[0] - exact[0]).powi(2) + (single_rk4[1] - exact[1]).powi(2)).sqrt();
    assert!(
        error < rk4_error,
        "adaptive ({error}) should beat one fixed RK4 step ({rk4_error})"
    );
}

#[test]
fn adaptive_rkf45_meets_tolerance() {
    let mut object = Harmonic;
    let dt = 3.0;
    let (state, _) = propagate_adaptive_rk(
        &RKF45,
        &AdaptiveController {
            atol: 1.0e-10,
            rtol: 1.0e-10,
            ..Default::default()
        },
        &mut object,
        &[1.0, 0.0],
        0,
        epoch(),
        dt,
    );
    let exact = [dt.cos(), -dt.sin()];
    let error = ((state[0] - exact[0]).powi(2) + (state[1] - exact[1]).powi(2)).sqrt();
    assert!(error < 1.0e-7, "adaptive rkf45 error was {error}");
}

/// `merge_step_outputs` reduces the per-sub-step outputs; here each sub-step
/// reports its own length, so the merged total must equal the outer `dt`
/// regardless of how many sub-steps the controller took.
#[test]
fn adaptive_step_output_is_summed_across_substeps() {
    let mut object = Harmonic;
    let dt = 7.0; // large enough to force several sub-steps
    let (_, summed_substep_lengths) = propagate_adaptive_rk(
        &DOPRI45,
        &AdaptiveController {
            atol: 1.0e-10,
            rtol: 1.0e-10,
            ..Default::default()
        },
        &mut object,
        &[1.0, 0.0],
        0,
        epoch(),
        dt,
    );
    assert!(
        (summed_substep_lengths - dt).abs() < 1.0e-9,
        "sub-step lengths summed to {summed_substep_lengths}, expected {dt}"
    );
}

/// A tableau with no embedded row falls back to a single fixed step.
#[test]
fn adaptive_falls_back_without_embedded_row() {
    let mut object = Harmonic;
    let adaptive = propagate_adaptive_rk(
        &RK4,
        &AdaptiveController::default(),
        &mut object,
        &[1.0, 0.0],
        0,
        epoch(),
        0.5,
    );
    let fixed = propagate_explicit_rk(&RK4, &mut object, &[1.0, 0.0], 0, epoch(), 0.5);
    assert_eq!(adaptive.0, fixed.0);
}
