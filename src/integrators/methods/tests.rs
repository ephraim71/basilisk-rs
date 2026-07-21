//! Tests for the [`Integrator`] trait objects and the built-in method structs,
//! including a user-defined integrator to guard the extension point.

use super::*;
use crate::integrators::traits::DynamicObject;
use hifitime::Epoch;

/// Constant-velocity 1-D point: `x' = v`, `v' = 0`, so `x(t) = x0 + v0·t`.
/// Every integrator is exact on this, which makes equality assertions safe.
///
/// Its `StepOutput` is `()` — it has no side-channel to report.
#[derive(Clone)]
struct Drifter;

impl DynamicObject for Drifter {
    type State = [f64; 2];
    type Derivative = [f64; 2];
    type StepOutput = ();

    fn equations_of_motion(&mut self, state: &[f64; 2], _: u64, _: Epoch) -> [f64; 2] {
        [state[1], 0.0]
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
        let mut increment = [0.0f64; 2];
        for (derivative, weight) in &weighted {
            increment[0] += derivative[0] * weight;
            increment[1] += derivative[1] * weight;
        }
        (
            [
                state[0] + increment[0] * step,
                state[1] + increment[1] * step,
            ],
            (),
        )
    }

    fn merge_step_outputs(&self, _accumulated: (), _next: ()) {}

    fn error_norm(&self, high: &[f64; 2], low: &[f64; 2], atol: f64, rtol: f64) -> Option<f64> {
        let mut sum = 0.0;
        for axis in 0..2 {
            let scale = atol + rtol * high[axis].abs().max(low[axis].abs());
            let e = (high[axis] - low[axis]) / scale;
            sum += e * e;
        }
        Some((sum / 2.0).sqrt())
    }
}

fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2020, 1, 1)
}

fn step_with(integrator: &dyn Integrator<Drifter>, dt: f64) -> [f64; 2] {
    let mut object = Drifter;
    integrator
        .propagate(&mut object, &[0.0, 2.0], 0, epoch(), dt)
        .0
}

#[test]
fn built_in_methods_advance_the_state() {
    // Exact answer after dt = 3 s: x = 0 + 2·3 = 6, v unchanged.
    let expected = [6.0, 2.0];
    for integrator in [
        Box::new(Euler) as Box<dyn Integrator<Drifter>>,
        Box::new(Midpoint),
        Box::new(Heun),
        Box::new(Ralston),
        Box::new(Rk4),
        Box::new(Rk4ThreeEighths),
        Box::new(Rkf45::default()),
        Box::new(Dopri45::default()),
    ] {
        let result = step_with(integrator.as_ref(), 3.0);
        assert!(
            (result[0] - expected[0]).abs() < 1e-9 && (result[1] - expected[1]).abs() < 1e-9,
            "{integrator:?} gave {result:?}, expected {expected:?}"
        );
    }
}

/// Adaptive structs carry configurable tolerances, unlike the old fn wrappers.
#[test]
fn adaptive_controller_is_configurable_per_instance() {
    let tight = Dopri45::new(AdaptiveController {
        rtol: 1.0e-12,
        atol: 1.0e-12,
        ..Default::default()
    });
    assert_eq!(tight.controller.rtol, 1.0e-12);
    // Still integrates the exact linear drift correctly.
    let result = step_with(&tight, 5.0);
    assert!((result[0] - 10.0).abs() < 1e-9, "got {result:?}");
}

/// Boxed integrators must be cloneable so `SpacecraftConfig` can keep `Clone`.
#[test]
fn boxed_integrator_clones() {
    let original: Box<dyn Integrator<Drifter>> = Box::new(Dopri45::default());
    let cloned = original.clone();
    assert_eq!(
        step_with(original.as_ref(), 4.0),
        step_with(cloned.as_ref(), 4.0)
    );
}

/// A downstream user can define an integrator entirely outside this module by
/// implementing [`Integrator`]; here, a naive forward-Euler written from scratch.
#[derive(Clone, Debug)]
struct UserForwardEuler;

impl<D: DynamicObject + 'static> Integrator<D> for UserForwardEuler {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput) {
        let k = object.equations_of_motion(state, current_sim_nanos, current_epoch);
        object.combine(state, vec![(k, 1.0)], dt_seconds)
    }
}

#[test]
fn user_defined_integrator_plugs_in_and_matches_builtin_euler() {
    let user: Box<dyn Integrator<Drifter>> = Box::new(UserForwardEuler);
    // Cloning through the trait object must work for a user type too.
    let user_clone = user.clone();
    assert_eq!(
        step_with(user.as_ref(), 3.0),
        step_with(&Euler, 3.0),
        "hand-written Euler should match the built-in"
    );
    assert_eq!(step_with(user_clone.as_ref(), 3.0), [6.0, 2.0]);
}
