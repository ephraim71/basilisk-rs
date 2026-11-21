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
/// their solutions should be close yet not identical, confirming the
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

/// Final error of `steps` fixed steps of `b`-weights `weights` over `[0, 2]`.
fn rkf78_final_error(weights: &'static [f64], steps: usize) -> f64 {
    let tableau = ButcherTableau {
        b: weights,
        b_embedded: None,
        ..RKF78
    };
    final_error(&tableau, 2.0, steps)
}

/// The RKF78 tableau propagates its 8th-order solution and carries the
/// 7th-order one as the embedded row.
#[test]
fn rkf78_weights_have_their_orders() {
    let embedded = RKF78.b_embedded.expect("RKF78 is an embedded pair");
    let propagated = (rkf78_final_error(RKF78.b, 4) / rkf78_final_error(RKF78.b, 8)).log2();
    let lower = (rkf78_final_error(embedded, 4) / rkf78_final_error(embedded, 8)).log2();
    assert!(
        (propagated - 8.0).abs() < 0.5,
        "propagated order was {propagated}"
    );
    assert!((lower - 7.0).abs() < 0.5, "embedded order was {lower}");
}

#[test]
fn adaptive_rkf78_meets_tolerance() {
    let mut object = Harmonic;
    let dt = 3.0;
    let (state, _) = propagate_adaptive_rk(
        &RKF78,
        &AdaptiveController {
            atol: 1.0e-12,
            rtol: 1.0e-12,
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
    assert!(error < 1.0e-10, "adaptive rkf78 error was {error}");
}

/// Every shipped tableau is consistent: it passes validation, its weights sum
/// to one and its nodes are the row sums of `a`.
#[test]
fn shipped_tableaus_are_consistent() {
    for tableau in [
        &EULER,
        &MIDPOINT,
        &HEUN,
        &RALSTON,
        &RK4,
        &RK4_THREE_EIGHTHS,
        &RKF45,
        &DOPRI45,
        &RKF78,
    ] {
        tableau.validate().unwrap();
        assert!((tableau.b.iter().sum::<f64>() - 1.0).abs() < 1.0e-14);
        if let Some(embedded) = tableau.b_embedded {
            assert!((embedded.iter().sum::<f64>() - 1.0).abs() < 1.0e-14);
        }
        for (row, &node) in tableau.a.iter().zip(tableau.c) {
            assert!((row.iter().sum::<f64>() - node).abs() < 1.0e-14);
        }
    }
}

/// Port of upstream `test_custom_integrators_reject_malformed_tableaus` and
/// `test_generated_custom_integrator_constructors_validate_dimensions`
/// (`test_Integrators.py`), for the cases this representation can express:
/// ragged and two-dimensional `b` inputs cannot be written as a
/// [`ButcherTableau`], and the error-estimate order is an integer.
#[test]
fn custom_tableaus_reject_malformed_coefficients() {
    const SECOND_ORDER: ButcherTableau = ButcherTableau {
        a: &[&[0.0, 0.0], &[1.0, 0.0]],
        c: &[0.0, 1.0],
        b: &[0.5, 0.5],
        b_embedded: None,
        error_estimate_order: 2,
    };
    SECOND_ORDER.validate().unwrap();

    let too_wide = ButcherTableau {
        a: &[&[0.0, 0.0, 0.0], &[1.0, 0.0, 0.0]],
        ..SECOND_ORDER
    };
    assert!(too_wide.validate().unwrap_err().contains("entries"));
    let too_few_rows = ButcherTableau {
        a: &[&[0.0, 0.0]],
        ..SECOND_ORDER
    };
    assert!(too_few_rows.validate().unwrap_err().contains("dimensions"));
    let non_finite_a = ButcherTableau {
        a: &[&[0.0, 0.0], &[f64::NAN, 0.0]],
        ..SECOND_ORDER
    };
    assert!(non_finite_a.validate().unwrap_err().contains("finite"));
    let non_finite_c = ButcherTableau {
        c: &[0.0, f64::INFINITY],
        ..SECOND_ORDER
    };
    assert!(non_finite_c.validate().unwrap_err().contains("finite"));
    const UPPER_ENTRY: &[&[f64]] = &[&[0.0, 0.25], &[1.0, 0.0]];
    const DIAGONAL_ENTRY: &[&[f64]] = &[&[0.0, 0.0], &[1.0, 0.25]];
    for a in [UPPER_ENTRY, DIAGONAL_ENTRY] {
        let implicit = ButcherTableau { a, ..SECOND_ORDER };
        assert!(
            implicit
                .validate()
                .unwrap_err()
                .contains("strictly lower triangular")
        );
    }

    let adaptive = ButcherTableau {
        b_embedded: Some(&[1.0, 0.0]),
        ..SECOND_ORDER
    };
    adaptive.validate().unwrap();
    let zero_order = ButcherTableau {
        error_estimate_order: 0,
        ..adaptive
    };
    assert!(
        zero_order
            .validate()
            .unwrap_err()
            .contains("finite and positive")
    );
    let non_finite_embedded = ButcherTableau {
        b_embedded: Some(&[1.0, f64::NAN]),
        ..SECOND_ORDER
    };
    assert!(
        non_finite_embedded
            .validate()
            .unwrap_err()
            .contains("finite")
    );
}

// Port of upstream `test_adaptiveRungeKuttaAccuracy.cpp`. Upstream couples two
// synchronized dynamic objects; synchronized integration is not ported, so the
// two oscillator halves are one object here, which integrates the same system.

/// Stage evaluation times recorded by the accuracy objects \[ns\].
type StageLog = Vec<u64>;

/// Upstream `OscillatorDynamics` pair: `first' = ω second`,
/// `second' = -ω first`, each a 2×3 block of coordinates.
struct CoupledOscillator {
    frequency: f64,
    stages: StageLog,
}

/// Upstream per-component error control (`ErrorControlMode::PerComponent`),
/// as an RMS of the scaled component errors.
fn componentwise_error_norm(high: &[f64], low: &[f64], atol: f64, rtol: f64) -> Option<f64> {
    let sum: f64 = high
        .iter()
        .zip(low)
        .map(|(high, low)| {
            let scale = atol + rtol * high.abs().max(low.abs());
            ((high - low) / scale).powi(2)
        })
        .sum();
    Some((sum / high.len() as f64).sqrt())
}

impl DynamicObject for CoupledOscillator {
    type State = [f64; 12];
    type Derivative = [f64; 12];
    type StepOutput = ();

    fn equations_of_motion(&mut self, state: &[f64; 12], nanos: u64, _: Epoch) -> [f64; 12] {
        self.stages.push(nanos);
        let mut derivative = [0.0; 12];
        for index in 0..6 {
            derivative[index] = self.frequency * state[index + 6];
            derivative[index + 6] = -self.frequency * state[index];
        }
        derivative
    }

    fn state_with_derivative(
        &self,
        state: &[f64; 12],
        derivative: &[f64; 12],
        dt: f64,
    ) -> [f64; 12] {
        std::array::from_fn(|index| state[index] + derivative[index] * dt)
    }

    fn combine(
        &self,
        state: &[f64; 12],
        weighted: Vec<([f64; 12], f64)>,
        step: f64,
    ) -> ([f64; 12], ()) {
        let next = std::array::from_fn(|index| {
            let increment: f64 = weighted
                .iter()
                .map(|(derivative, weight)| derivative[index] * weight)
                .sum();
            state[index] + increment * step
        });
        (next, ())
    }

    fn merge_step_outputs(&self, _: (), _: ()) {}

    fn error_norm(&self, high: &[f64; 12], low: &[f64; 12], atol: f64, rtol: f64) -> Option<f64> {
        componentwise_error_norm(high, low, atol, rtol)
    }
}

/// Upstream `MixedScaleDynamics`: a constant `1e9` component next to a
/// decaying unit one.
struct MixedScale {
    stages: StageLog,
}

impl DynamicObject for MixedScale {
    type State = [f64; 2];
    type Derivative = [f64; 2];
    type StepOutput = ();

    fn equations_of_motion(&mut self, state: &[f64; 2], nanos: u64, _: Epoch) -> [f64; 2] {
        self.stages.push(nanos);
        [0.0, -state[1]]
    }

    fn state_with_derivative(&self, state: &[f64; 2], derivative: &[f64; 2], dt: f64) -> [f64; 2] {
        [state[0] + derivative[0] * dt, state[1] + derivative[1] * dt]
    }

    fn combine(
        &self,
        state: &[f64; 2],
        weighted: Vec<([f64; 2], f64)>,
        step: f64,
    ) -> ([f64; 2], ()) {
        let next = std::array::from_fn(|index| {
            let increment: f64 = weighted
                .iter()
                .map(|(derivative, weight)| derivative[index] * weight)
                .sum();
            state[index] + increment * step
        });
        (next, ())
    }

    fn merge_step_outputs(&self, _: (), _: ()) {}

    fn error_norm(&self, high: &[f64; 2], low: &[f64; 2], atol: f64, rtol: f64) -> Option<f64> {
        componentwise_error_norm(high, low, atol, rtol)
    }

    /// Upstream `ErrorControlMode::PerComponent`.
    fn max_relative_error(
        &self,
        state: &[f64; 2],
        low: &[(&[f64; 2], f64)],
        high: &[(&[f64; 2], f64)],
        step: f64,
        atol: f64,
        rtol: f64,
    ) -> Option<f64> {
        let low = self.state_with_derivatives(state, low, step);
        let high = self.state_with_derivatives(state, high, step);
        Some(
            (0..2)
                .map(|index| {
                    relative_error_ratio(
                        (high[index] - low[index]).abs(),
                        high[index].abs(),
                        rtol,
                        atol,
                    )
                })
                .fold(0.0, f64::max),
        )
    }
}

/// Upstream `expectValidTrials`: the stage times split into trials of one
/// evaluation per stage at `t + c_i h`; a rejected trial restarts at the same
/// time with a smaller step, an accepted one ends where the next begins, the
/// last one ends at the interval end, and both kinds occur. The trial step is
/// read from the tableau's last `c = 1` stage; nanosecond times allow a
/// couple of nanoseconds of truncation.
fn expect_valid_trials(stages: &[u64], tableau: &ButcherTableau, end_nanos: u64) {
    let stage_count = tableau.c.len();
    assert!(!stages.is_empty() && stages.len().is_multiple_of(stage_count));
    let unit_stage = tableau
        .c
        .iter()
        .rposition(|&c| c == 1.0)
        .expect("a c = 1 stage");
    let tolerance = 2.0;
    let trials: Vec<&[u64]> = stages.chunks(stage_count).collect();
    assert_eq!(trials[0][0], 0);
    let (mut rejected, mut accepted) = (0, 0);
    let step_of = |trial: &[u64]| (trial[unit_stage] - trial[0]) as f64;
    for (index, trial) in trials.iter().enumerate() {
        let step = step_of(trial);
        assert!(step > 0.0);
        assert!(trial[0] as f64 + step <= end_nanos as f64 + tolerance);
        for (&time, &c) in trial.iter().zip(tableau.c) {
            assert!((time as f64 - (trial[0] as f64 + c * step)).abs() <= tolerance);
        }
        match trials.get(index + 1) {
            Some(next) if next[0] == trial[0] => {
                rejected += 1;
                assert!(step_of(next) < step);
            }
            Some(next) => {
                accepted += 1;
                assert!((next[0] as f64 - (trial[0] as f64 + step)).abs() <= tolerance);
            }
            None => {
                accepted += 1;
                assert!((trial[0] as f64 + step - end_nanos as f64).abs() <= tolerance);
            }
        }
    }
    assert!(rejected > 0, "no trial was rejected");
    assert!(accepted > 1, "only {accepted} trial was accepted");
}

/// Upstream `CoupledOscillatorConvergesWithTighterTolerance`, for RKF45 and
/// RKF78: tightening the tolerance from `1e-4` to `1e-8` reduces the global
/// error of the eight-second trajectory below `1e-6` and by at least 100×.
#[test]
fn coupled_oscillator_converges_with_tighter_tolerance() {
    let duration = 8.0;
    let frequency = 1.0;
    for tableau in [&RKF45, &RKF78] {
        let mut previous_error = f64::INFINITY;
        for tolerance in [1.0e-4, 1.0e-8] {
            let mut object = CoupledOscillator {
                frequency,
                stages: Vec::new(),
            };
            let mut initial = [0.0; 12];
            initial[..6].fill(1.0);
            let controller = AdaptiveController {
                rtol: tolerance,
                atol: tolerance * 0.01,
                ..Default::default()
            };
            let (state, ()) = propagate_adaptive_rk(
                tableau,
                &controller,
                &mut object,
                &initial,
                0,
                epoch(),
                duration,
            );
            assert!(state.iter().all(|value| value.is_finite()));
            let error = (0..6)
                .map(|index| {
                    let first = (state[index] - (frequency * duration).cos()).abs();
                    let second = (state[index + 6] + (frequency * duration).sin()).abs();
                    first.max(second)
                })
                .fold(0.0, f64::max);
            if tolerance == 1.0e-8 {
                assert!(error < 1.0e-6, "global error {error}");
                assert!(error < previous_error * 0.01, "{error} vs {previous_error}");
            }
            previous_error = error;
            expect_valid_trials(&object.stages, tableau, 8_000_000_000);
        }
    }
}

/// Upstream `ComponentwiseControlPreservesSmallStateAccuracy`: per-component
/// error control keeps the small decaying component accurate next to a
/// `1e9` constant one.
#[test]
fn componentwise_control_preserves_small_state_accuracy() {
    let duration: f64 = 4.0;
    let exact = (-duration).exp();
    for tableau in [&RKF45, &RKF78] {
        let mut object = MixedScale { stages: Vec::new() };
        let controller = AdaptiveController {
            rtol: 1.0e-8,
            atol: 1.0e-10,
            ..Default::default()
        };
        let (state, ()) = propagate_adaptive_rk(
            tableau,
            &controller,
            &mut object,
            &[1.0e9, 1.0],
            0,
            epoch(),
            duration,
        );
        assert_eq!(state[0], 1.0e9);
        assert!((state[1] - exact).abs() <= 1.0e-6 * exact, "{}", state[1]);
        expect_valid_trials(&object.stages, tableau, 4_000_000_000);
    }
}

// Upstream step control (`StepControl::Upstream`): the same accuracy tests
// with upstream's error measure, where each upstream state is compared as a
// whole (the two 2×3 oscillator blocks) or per component (the mixed-scale
// state), and checks of the trial sequence itself.

/// Upstream whole-state ratio of the components `range` of two candidates.
fn whole_state_ratio(
    low: &[f64],
    high: &[f64],
    range: std::ops::Range<usize>,
    atol: f64,
    rtol: f64,
) -> f64 {
    let error = range
        .clone()
        .map(|index| (high[index] - low[index]).powi(2))
        .sum::<f64>()
        .sqrt();
    let magnitude = range.map(|index| high[index].powi(2)).sum::<f64>().sqrt();
    relative_error_ratio(error, magnitude, rtol, atol)
}

impl CoupledOscillator {
    /// The two candidates `state + step Σ w k`.
    fn candidates(
        &self,
        state: &[f64; 12],
        low: &[(&[f64; 12], f64)],
        high: &[(&[f64; 12], f64)],
        step: f64,
    ) -> ([f64; 12], [f64; 12]) {
        (
            self.state_with_derivatives(state, low, step),
            self.state_with_derivatives(state, high, step),
        )
    }
}

/// Upstream's two `OscillatorDynamics` objects, one whole state each.
struct UpstreamOscillator(CoupledOscillator);

impl DynamicObject for UpstreamOscillator {
    type State = [f64; 12];
    type Derivative = [f64; 12];
    type StepOutput = ();

    fn equations_of_motion(&mut self, state: &[f64; 12], nanos: u64, epoch: Epoch) -> [f64; 12] {
        self.0.equations_of_motion(state, nanos, epoch)
    }

    fn state_with_derivative(
        &self,
        state: &[f64; 12],
        derivative: &[f64; 12],
        dt: f64,
    ) -> [f64; 12] {
        self.0.state_with_derivative(state, derivative, dt)
    }

    fn combine(
        &self,
        state: &[f64; 12],
        weighted: Vec<([f64; 12], f64)>,
        step: f64,
    ) -> ([f64; 12], ()) {
        self.0.combine(state, weighted, step)
    }

    fn merge_step_outputs(&self, _: (), _: ()) {}

    fn max_relative_error(
        &self,
        state: &[f64; 12],
        low: &[(&[f64; 12], f64)],
        high: &[(&[f64; 12], f64)],
        step: f64,
        atol: f64,
        rtol: f64,
    ) -> Option<f64> {
        let (low, high) = self.0.candidates(state, low, high, step);
        Some(
            whole_state_ratio(&low, &high, 0..6, atol, rtol).max(whole_state_ratio(
                &low,
                &high,
                6..12,
                atol,
                rtol,
            )),
        )
    }
}

/// Upstream `CoupledOscillatorConvergesWithTighterTolerance` under upstream's
/// step control.
#[test]
fn upstream_control_coupled_oscillator_converges_with_tighter_tolerance() {
    let duration = 8.0;
    let frequency = 1.0;
    for tableau in [&RKF45, &RKF78] {
        let mut previous_error = f64::INFINITY;
        for tolerance in [1.0e-4, 1.0e-8] {
            let mut object = UpstreamOscillator(CoupledOscillator {
                frequency,
                stages: Vec::new(),
            });
            let mut initial = [0.0; 12];
            initial[..6].fill(1.0);
            let controller = AdaptiveController {
                rtol: tolerance,
                atol: tolerance * 0.01,
                ..AdaptiveController::upstream()
            };
            let (state, ()) = propagate_adaptive_rk(
                tableau,
                &controller,
                &mut object,
                &initial,
                0,
                epoch(),
                duration,
            );
            assert!(state.iter().all(|value| value.is_finite()));
            let error = (0..6)
                .map(|index| {
                    let first = (state[index] - (frequency * duration).cos()).abs();
                    let second = (state[index + 6] + (frequency * duration).sin()).abs();
                    first.max(second)
                })
                .fold(0.0, f64::max);
            if tolerance == 1.0e-8 {
                assert!(error < 1.0e-6, "global error {error}");
                assert!(error < previous_error * 0.01, "{error} vs {previous_error}");
            }
            previous_error = error;
            expect_valid_trials(&object.0.stages, tableau, 8_000_000_000);
        }
    }
}

/// Upstream `ComponentwiseControlPreservesSmallStateAccuracy` under
/// upstream's step control.
#[test]
fn upstream_control_componentwise_preserves_small_state_accuracy() {
    let duration: f64 = 4.0;
    let exact = (-duration).exp();
    for tableau in [&RKF45, &RKF78] {
        let mut object = MixedScale { stages: Vec::new() };
        let controller = AdaptiveController {
            rtol: 1.0e-8,
            atol: 1.0e-10,
            ..AdaptiveController::upstream()
        };
        let (state, ()) = propagate_adaptive_rk(
            tableau,
            &controller,
            &mut object,
            &[1.0e9, 1.0],
            0,
            epoch(),
            duration,
        );
        assert_eq!(state[0], 1.0e9);
        assert!((state[1] - exact).abs() <= 1.0e-6 * exact, "{}", state[1]);
        expect_valid_trials(&object.stages, tableau, 4_000_000_000);
    }
}

/// An accepted trial keeps upstream's `bStarArray` candidate, the
/// `b_embedded` row (RKF45's fourth-order one), not the `b` row.
#[test]
fn upstream_control_keeps_the_b_star_candidate() {
    const RKF45_B_STAR: ButcherTableau = ButcherTableau {
        b: match RKF45.b_embedded {
            Some(b_star) => b_star,
            None => RKF45.b,
        },
        b_embedded: None,
        ..RKF45
    };
    let mut object = UpstreamOscillator(CoupledOscillator {
        frequency: 1.0,
        stages: Vec::new(),
    });
    let mut initial = [0.0; 12];
    initial[..6].fill(1.0);
    let loose = AdaptiveController {
        rtol: 1.0,
        atol: 1.0,
        ..AdaptiveController::upstream()
    };
    let (adaptive, ()) =
        propagate_adaptive_rk(&RKF45, &loose, &mut object, &initial, 0, epoch(), 0.1);
    assert_eq!(object.0.stages.len(), RKF45.c.len(), "one accepted trial");
    let (b_star, ()) = propagate_explicit_rk(&RKF45_B_STAR, &mut object, &initial, 0, epoch(), 0.1);
    let (b, ()) = propagate_explicit_rk(&RKF45, &mut object, &initial, 0, epoch(), 0.1);
    assert_eq!(adaptive, b_star);
    assert_ne!(adaptive, b);
}

/// Upstream's trial sizing: a rejected whole-interval trial is followed by
/// one of `0.9 h (1/err)^(1/5)` (RKF45, `methodLargestOrder = 5`), at least
/// `0.1 h`, from the same start; an accepted one by at most `4 h`, capped at
/// the rest of the interval.
#[test]
fn upstream_control_sizes_trials_like_upstream() {
    let dt = 2.0;
    let controller = AdaptiveController {
        rtol: 1.0e-6,
        atol: 1.0e-8,
        ..AdaptiveController::upstream()
    };
    let mut initial = [0.0; 12];
    initial[..6].fill(1.0);
    let mut object = UpstreamOscillator(CoupledOscillator {
        frequency: 1.0,
        stages: Vec::new(),
    });

    // The first trial's error, from its own stages.
    let ks = compute_stages(&RKF45, &mut object, &initial, 0, epoch(), 0.0, dt);
    let low: Vec<_> = ks.iter().zip(RKF45.b.iter().copied()).collect();
    let high: Vec<_> = ks
        .iter()
        .zip(RKF45.b_embedded.unwrap().iter().copied())
        .collect();
    let first_error = object
        .max_relative_error(&initial, &low, &high, dt, controller.atol, controller.rtol)
        .unwrap();
    assert!(first_error > 1.0, "the whole interval is rejected");
    let second_step = (0.9 * dt * (1.0 / first_error).powf(0.2)).clamp(0.1 * dt, 4.0 * dt);

    object.0.stages.clear();
    propagate_adaptive_rk(&RKF45, &controller, &mut object, &initial, 0, epoch(), dt);
    let trials: Vec<&[u64]> = object.0.stages.chunks(RKF45.c.len()).collect();
    // Stage 4 has `c = 1`: it sits one trial step after the trial start.
    let step_of = |trial: &[u64]| (trial[4] - trial[0]) as f64 * 1.0e-9;
    assert_eq!(trials[1][0], 0, "the second trial restarts at the start");
    assert!(
        (step_of(trials[1]) - second_step).abs() < 3.0e-9,
        "{} vs {second_step}",
        step_of(trials[1])
    );
    for pair in trials.windows(2) {
        let (trial, next) = (pair[0], pair[1]);
        if next[0] != trial[0] {
            assert!(step_of(next) <= 4.0 * step_of(trial) + 3.0e-9);
        }
    }
    expect_valid_trials(&object.0.stages, &RKF45, 2_000_000_000);
}

/// An object without `max_relative_error` takes the whole step at once.
#[test]
fn upstream_control_without_the_error_measure_takes_one_step() {
    let mut object = CoupledOscillator {
        frequency: 1.0,
        stages: Vec::new(),
    };
    let mut initial = [0.0; 12];
    initial[..6].fill(1.0);
    propagate_adaptive_rk(
        &RKF45,
        &AdaptiveController::upstream(),
        &mut object,
        &initial,
        0,
        epoch(),
        2.0,
    );
    assert_eq!(object.stages.len(), RKF45.c.len());
}
