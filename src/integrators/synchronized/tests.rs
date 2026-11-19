//! Synchronized integration tests.
//!
//! `CoupledOscillatorConvergesWithTighterTolerance` is a port of upstream
//! `test_adaptiveRungeKuttaAccuracy.cpp`, and the noise-numbering test follows
//! `test_flatStochastic.cpp`'s `ManagerLocalSharedIdsDoNotCollideAcrossObjects`.
//! Upstream's other synchronization tests (`test_dynamicObject.cpp`,
//! `test_synchronizedDynamicsLifetime.py`, `test_integratorOwnership.py`, the
//! peer-destruction cases of `test_flatRungeKutta.cpp` and
//! `test_flatStochastic.cpp`) check pointer and Python-proxy lifetimes:
//! connecting an object twice or to itself, a secondary outliving its primary,
//! replacing a secondary's integrator. Ownership rules those cases out here,
//! so they have no counterpart.

use std::sync::{Arc, Mutex};

use hifitime::Epoch;
use nalgebra::Matrix2x3;

use super::{Coupling, Synchronized, Uncoupled};
use crate::integrators::stochastic::{
    EulerMaruyama, GaussianNoiseGenerator, PrescribedGaussianNoiseGenerator,
};
use crate::integrators::{
    AdaptiveController, DynamicObject, Integrator, RKF45, RKF78, Rk4, Rkf45, Rkf78,
};

fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
}

/// Upstream `OscillatorDynamics`: a 2×3 state whose derivative is `rate`
/// times its partner's state, recording every stage time it is evaluated at.
struct Oscillator {
    rate: f64,
    stage_nanos: Vec<u64>,
}

impl Oscillator {
    fn new(rate: f64) -> Self {
        Self {
            rate,
            stage_nanos: Vec::new(),
        }
    }
}

impl DynamicObject for Oscillator {
    type State = Matrix2x3<f64>;
    type StepOutput = ();
    type Derivative = Matrix2x3<f64>;

    fn equations_of_motion(&mut self, _: &Self::State, _: u64, _: Epoch) -> Self::Derivative {
        unreachable!("an oscillator is evaluated through its partner coupling")
    }

    fn state_with_derivative(
        &self,
        state: &Self::State,
        derivative: &Self::Derivative,
        dt: f64,
    ) -> Self::State {
        state + derivative * dt
    }

    fn combine(
        &self,
        state: &Self::State,
        weighted: Vec<(Self::Derivative, f64)>,
        step: f64,
    ) -> (Self::State, ()) {
        let sum = weighted
            .iter()
            .fold(Matrix2x3::zeros(), |sum, (derivative, weight)| {
                sum + derivative * *weight
            });
        (state + sum * step, ())
    }

    fn merge_step_outputs(&self, _: (), _: ()) {}

    fn error_norm(
        &self,
        high: &Self::State,
        low: &Self::State,
        atol: f64,
        rtol: f64,
    ) -> Option<f64> {
        let sum_of_squares: f64 = high
            .iter()
            .zip(low.iter())
            .map(|(high, low)| ((high - low) / (atol + rtol * high.abs().max(low.abs()))).powi(2))
            .sum();
        Some((sum_of_squares / 6.0).sqrt())
    }
}

/// Upstream's partner pointers: each oscillator's derivative reads the other
/// one's stage state.
struct PartnerCoupling;

impl Coupling<Oscillator> for PartnerCoupling {
    fn equations_of_motion(
        &mut self,
        members: &mut [Oscillator],
        states: &[Matrix2x3<f64>],
        current_sim_nanos: u64,
        _: Epoch,
    ) -> Vec<Matrix2x3<f64>> {
        members
            .iter_mut()
            .enumerate()
            .map(|(index, member)| {
                member.stage_nanos.push(current_sim_nanos);
                states[1 - index] * member.rate
            })
            .collect()
    }
}

/// Upstream `expectValidTrials`: the stages come in whole trials, each at
/// `t₀ + cᵢ h`; a rejected trial restarts at its own `t₀` with a smaller `h`
/// and an accepted one is followed by a trial at `t₀ + h`; the last accepted
/// trial ends at `end_nanos`. There must be rejected trials and more than one
/// accepted one. Times are integer nanoseconds here, so they agree to a few
/// nanoseconds of truncation.
fn expect_valid_trials(stage_nanos: &[u64], nodes: &[f64], end_nanos: u64) {
    let stage_count = nodes.len();
    let tolerance_nanos = 4.0;
    assert!(!stage_nanos.is_empty());
    assert_eq!(stage_nanos.len() % stage_count, 0);
    assert_eq!(stage_nanos[0], 0);
    let last_node = nodes
        .iter()
        .position(|&node| node == 1.0)
        .expect("tableau has a stage at the step end");
    let trial_step = |trial: &[u64]| (trial[last_node] - trial[0]) as f64;
    let trials: Vec<&[u64]> = stage_nanos.chunks(stage_count).collect();
    let (mut rejected, mut accepted) = (0, 0);
    for (index, trial) in trials.iter().enumerate() {
        let (start, step) = (trial[0] as f64, trial_step(trial));
        for (&time, &node) in trial.iter().zip(nodes) {
            assert!(
                (time as f64 - (start + node * step)).abs() <= tolerance_nanos,
                "stage at {time} ns is not at node {node} of the trial from {start} ns"
            );
        }
        match trials.get(index + 1) {
            Some(next) if next[0] == trial[0] => {
                rejected += 1;
                assert!(trial_step(next) < step);
            }
            Some(next) => {
                accepted += 1;
                assert!((next[0] as f64 - (start + step)).abs() <= tolerance_nanos);
            }
            None => {
                accepted += 1;
                assert!((start + step - end_nanos as f64).abs() <= tolerance_nanos);
            }
        }
    }
    assert!(rejected > 0);
    assert!(accepted > 1);
}

/// Upstream `CoupledOscillatorConvergesWithTighterTolerance`: `x₁' = ω x₂`,
/// `x₂' = -ω x₁` split across two synchronized objects, integrated over 8 s
/// in one call. At the tighter tolerance the global error is below `1e-6` and
/// a hundredth of the looser tolerance's, and both objects see the same
/// stage times.
#[test]
fn coupled_oscillator_converges_with_tighter_tolerance() {
    let duration_s: f64 = 8.0;
    let duration_nanos = 8_000_000_000;
    let frequency_radps: f64 = 1.0;
    let initial = Matrix2x3::repeat(1.0);
    let exact_first = initial * (frequency_radps * duration_s).cos();
    let exact_second = -initial * (frequency_radps * duration_s).sin();

    type Factory =
        fn(AdaptiveController) -> Box<dyn Integrator<Synchronized<Oscillator, PartnerCoupling>>>;
    let methods: [(&str, Factory, &[f64]); 2] = [
        (
            "RKF45",
            |controller| Box::new(Rkf45::new(controller)),
            RKF45.c,
        ),
        (
            "RKF78",
            |controller| Box::new(Rkf78::new(controller)),
            RKF78.c,
        ),
    ];
    for (name, factory, nodes) in methods {
        let mut previous_error = f64::INFINITY;
        for tolerance in [1.0e-4, 1.0e-8] {
            let integrator = factory(AdaptiveController {
                rtol: tolerance,
                atol: tolerance * 0.01,
                ..AdaptiveController::default()
            });
            let mut group = Synchronized::new(
                vec![
                    Oscillator::new(frequency_radps),
                    Oscillator::new(-frequency_radps),
                ],
                PartnerCoupling,
            );
            let (state, outputs) = integrator.propagate(
                &mut group,
                &vec![initial, Matrix2x3::zeros()],
                0,
                epoch(),
                duration_s,
            );
            assert_eq!(outputs.len(), 2);
            assert!(
                state
                    .iter()
                    .all(|value| value.iter().all(|x| x.is_finite()))
            );
            let error = (state[0] - exact_first)
                .abs()
                .max()
                .max((state[1] - exact_second).abs().max());
            if tolerance == 1.0e-8 {
                assert!(error < 1.0e-6, "{name}: error {error}");
                assert!(
                    error < previous_error * 0.01,
                    "{name}: error {error} vs {previous_error}"
                );
            }
            previous_error = error;
            let [first, second] = &group.members() else {
                unreachable!()
            };
            assert_eq!(first.stage_nanos, second.stage_nanos, "{name}");
            expect_valid_trials(&first.stage_nanos, nodes, duration_nanos);
        }
    }
}

/// `x' = drift`, with one constant diffusion column per entry of `diffusion`.
struct Scalar {
    drift: fn(f64) -> f64,
    diffusion: Vec<f64>,
}

impl DynamicObject for Scalar {
    type State = f64;
    type StepOutput = f64;
    type Derivative = f64;

    fn equations_of_motion(&mut self, state: &f64, _: u64, _: Epoch) -> f64 {
        (self.drift)(*state)
    }

    fn state_with_derivative(&self, state: &f64, derivative: &f64, dt: f64) -> f64 {
        state + derivative * dt
    }

    /// The step output is the drift increment, to check the per-member merge.
    fn combine(&self, state: &f64, weighted: Vec<(f64, f64)>, step: f64) -> (f64, f64) {
        let increment = weighted
            .iter()
            .map(|(derivative, weight)| derivative * weight)
            .sum::<f64>()
            * step;
        (state + increment, increment)
    }

    fn merge_step_outputs(&self, accumulated: f64, next: f64) -> f64 {
        accumulated + next
    }

    fn error_norm(&self, high: &f64, low: &f64, atol: f64, rtol: f64) -> Option<f64> {
        Some(((high - low) / (atol + rtol * high.abs().max(low.abs()))).abs())
    }

    fn noise_source_count(&self) -> usize {
        self.diffusion.len()
    }

    fn equations_of_motion_diffusion(&mut self, _: &f64, _: u64, _: Epoch) -> Vec<f64> {
        self.diffusion.clone()
    }
}

fn decay(x: f64) -> f64 {
    -0.7 * x + 0.1 * x * x
}

fn growth(x: f64) -> f64 {
    0.3 * x.sin() + 1.0
}

/// A replay of `dw`, one step per entry.
fn replay(steps: &[Vec<f64>]) -> Arc<Mutex<dyn GaussianNoiseGenerator>> {
    let mut generator = PrescribedGaussianNoiseGenerator::new();
    for dw in steps {
        generator.push_step(dw, &[]);
    }
    Arc::new(Mutex::new(generator))
}

/// Upstream `ManagerLocalSharedIdsDoNotCollideAcrossObjects`: one
/// Euler-Maruyama step of two synchronized objects draws every Wiener
/// increment at once, the first member's sources first.
#[test]
fn noise_sources_are_numbered_member_by_member() {
    let mut group = Synchronized::new(
        vec![
            Scalar {
                drift: |_| 1.5,
                diffusion: vec![1.0, 10.0],
            },
            Scalar {
                drift: |_| -2.0,
                diffusion: vec![100.0, 1000.0],
            },
        ],
        Uncoupled,
    );
    assert_eq!(group.noise_source_count(), 4);
    let integrator = EulerMaruyama::new().with_noise_generator(replay(&[vec![2.0, 3.0, 5.0, 7.0]]));
    let (state, drift) = integrator.propagate(&mut group, &vec![1.0, 2.0], 0, epoch(), 2.0);

    assert_eq!(
        state,
        vec![1.0 + 3.0 + 2.0 + 30.0, 2.0 - 4.0 + 500.0 + 7000.0]
    );
    assert_eq!(drift, vec![3.0, -4.0]);
}

/// Integrates `steps` steps of `dt` from `x0` with `integrator`.
fn integrate<D: DynamicObject + 'static>(
    integrator: &dyn Integrator<D>,
    object: &mut D,
    x0: D::State,
    dt: f64,
    steps: u64,
) -> (D::State, Vec<D::StepOutput>) {
    let dt_nanos = (dt * 1.0e9).round() as u64;
    let mut state = x0;
    let mut outputs = Vec::new();
    for step in 0..steps {
        let (next, output) = integrator.propagate(object, &state, step * dt_nanos, epoch(), dt);
        state = next;
        outputs.push(output);
    }
    (state, outputs)
}

/// Members that do not interact advance exactly as they would alone under a
/// fixed-step method, stochastic ones included when each draws its own
/// increments; under an adaptive method a one-member group is the member.
#[test]
fn uncoupled_members_integrate_as_they_would_alone() {
    let members = || {
        vec![
            Scalar {
                drift: decay,
                diffusion: vec![0.2],
            },
            Scalar {
                drift: growth,
                diffusion: vec![0.05, -0.3],
            },
        ]
    };
    let x0 = [1.3, -0.4];

    let mut group = Synchronized::new(members(), Uncoupled);
    let (together, outputs) = integrate(&Rk4, &mut group, x0.to_vec(), 0.1, 50);
    for (index, mut member) in members().into_iter().enumerate() {
        let (alone, alone_outputs) = integrate(&Rk4, &mut member, x0[index], 0.1, 50);
        assert_eq!(together[index].to_bits(), alone.to_bits());
        let member_outputs: Vec<f64> = outputs.iter().map(|output| output[index]).collect();
        assert_eq!(member_outputs, alone_outputs);
    }

    let steps: Vec<Vec<f64>> = (0..20)
        .map(|step| {
            let step = step as f64;
            vec![0.1 * step.sin(), 0.2 * step.cos(), -0.05 * step]
        })
        .collect();
    let mut group = Synchronized::new(members(), Uncoupled);
    let (together, _) = integrate(
        &EulerMaruyama::new().with_noise_generator(replay(&steps)),
        &mut group,
        x0.to_vec(),
        0.05,
        20,
    );
    for (index, (mut member, sources)) in members().into_iter().zip([0..1, 1..3]).enumerate() {
        let member_steps: Vec<Vec<f64>> = steps
            .iter()
            .map(|dw| dw[sources.clone()].to_vec())
            .collect();
        let (alone, _) = integrate(
            &EulerMaruyama::new().with_noise_generator(replay(&member_steps)),
            &mut member,
            x0[index],
            0.05,
            20,
        );
        assert_eq!(together[index].to_bits(), alone.to_bits());
    }

    let controller = AdaptiveController {
        rtol: 1.0e-10,
        atol: 1.0e-12,
        ..AdaptiveController::default()
    };
    let mut group = Synchronized::new(members().into_iter().take(1).collect(), Uncoupled);
    let (together, outputs) = integrate(&Rkf45::new(controller), &mut group, vec![x0[0]], 2.0, 3);
    let mut alone = members().remove(0);
    let (alone_state, alone_outputs) =
        integrate(&Rkf45::new(controller), &mut alone, x0[0], 2.0, 3);
    assert_eq!(together[0].to_bits(), alone_state.to_bits());
    let merged: Vec<f64> = outputs.iter().map(|output| output[0]).collect();
    assert_eq!(merged, alone_outputs);
}

/// The group's error norm is its largest member's, and a member without error
/// control opts the whole group out.
#[test]
fn error_norm_is_the_largest_member_norm() {
    let group = Synchronized::new(
        vec![
            Scalar {
                drift: decay,
                diffusion: Vec::new(),
            },
            Scalar {
                drift: growth,
                diffusion: Vec::new(),
            },
        ],
        Uncoupled,
    );
    assert_eq!(
        group.error_norm(&vec![1.0, 2.0], &vec![1.5, 1.0], 1.0, 0.0),
        Some(1.0)
    );
    assert_eq!(
        group.error_norm(&vec![1.0, 2.0], &vec![1.0, 2.25], 1.0, 0.0),
        Some(0.25)
    );

    struct NoErrorControl;
    impl DynamicObject for NoErrorControl {
        type State = f64;
        type StepOutput = ();
        type Derivative = f64;
        fn equations_of_motion(&mut self, _: &f64, _: u64, _: Epoch) -> f64 {
            0.0
        }
        fn state_with_derivative(&self, state: &f64, derivative: &f64, dt: f64) -> f64 {
            state + derivative * dt
        }
        fn combine(&self, state: &f64, _: Vec<(f64, f64)>, _: f64) -> (f64, ()) {
            (*state, ())
        }
        fn merge_step_outputs(&self, _: (), _: ()) {}
    }
    let opted_out = Synchronized::new(vec![NoErrorControl, NoErrorControl], Uncoupled);
    assert_eq!(
        opted_out.error_norm(&vec![1.0, 1.0], &vec![0.0, 0.0], 1.0, 0.0),
        None
    );
}

#[test]
#[should_panic(expected = "at least one member")]
fn an_empty_group_is_rejected() {
    let _ = Synchronized::<Scalar, Uncoupled>::new(Vec::new(), Uncoupled);
}
