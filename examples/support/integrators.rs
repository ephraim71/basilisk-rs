//! Upstream's integrator classes, as the integrator scenarios configure them.
//!
//! | upstream | here |
//! |---|---|
//! | default (`svIntegratorRK4`) | `None` ([`basilisk_rs::integrators::Rk4`]) |
//! | `svIntegratorEuler` | [`basilisk_rs::integrators::Euler`] |
//! | `svIntegratorRK2` | [`basilisk_rs::integrators::Heun`] (upstream's tableau) |
//! | `svIntegratorRungeKutta(sc, a, b, c)` | [`FixedTableau`] of a [`ButcherTableau`] |
//! | `svIntegratorRKF45`, `svIntegratorRKF78` | [`UpstreamAdaptive::rkf45`], [`UpstreamAdaptive::rkf78`] |
//! | `svIntegratorAdaptiveRungeKutta(sc, order, a, b, b*, c)` | [`UpstreamAdaptive::new`] |
//!
//! basilisk-rs's own adaptive methods ([`basilisk_rs::integrators::Rkf45`],
//! [`basilisk_rs::integrators::Rkf78`]) control the step with their own
//! scheme: an RMS norm over every scalar and the higher-order solution
//! propagated. The integrator scenarios compare against upstream's truth,
//! which depends on upstream's step-size control, so [`UpstreamAdaptive`]
//! ports `svIntegratorAdaptiveRungeKutta::integrateImpl` as a custom
//! [`Integrator`] of a [`Spacecraft`], the extension point upstream's
//! `scenarioIntegrators` demonstrates:
//!
//! * every outer step starts with the trial step equal to the whole step;
//! * each trial forms the `bArray` and `bStarArray` candidates and accepts
//!   when the largest error ratio over the states is at most one, each state
//!   (upstream `ErrorControlMode::WholeState`) contributing
//!   `|high - low| / (|high| relTol + absTol)` with matrix norms;
//! * an accepted trial keeps the `bStarArray` candidate;
//! * the next trial step is `0.9 h (1/err)^(1/order)`, clamped to
//!   `[0.1 h, 4 h]` and to the rest of the outer step.
//!
//! The states are the hub's position, velocity, attitude and rate, each
//! state effector's state vector (one block per effector: upstream may split
//! an effector's states further) and upstream's `hubGravVelocity`, the
//! integrated gravity, which restarts from the hub velocity every step.
//! Upstream also integrates `BcGravVelocity`, the same for the center of
//! mass; it matches `hubGravVelocity` while the center of mass is at `B`, as
//! in every scenario that uses this.

#![allow(dead_code)]

use basilisk_rs::integrators::{ButcherTableau, DynamicObject, Integrator, RKF45, RKF78};
use basilisk_rs::spacecraft::{IntegratedState, Spacecraft, StateDerivative};
use hifitime::{Duration, Epoch};
use nalgebra::Vector3;

/// `scenarioIntegrators`' `rk3`: Kutta's third-order method through
/// `svIntegratorRungeKutta`.
pub const RK3: ButcherTableau = ButcherTableau {
    a: &[&[], &[0.5], &[-1.0, 2.0]],
    c: &[0.0, 0.5, 1.0],
    b: &[1.0 / 6.0, 2.0 / 3.0, 1.0 / 6.0],
    b_embedded: None,
    error_estimate_order: 3,
};

/// `scenarioIntegrators`' `bogackiShampine` through
/// `svIntegratorAdaptiveRungeKutta`: `b` is upstream's `b_coefficients`,
/// `b_embedded` its `b_star_coefficients` (the propagated ones).
pub const BOGACKI_SHAMPINE: ButcherTableau = ButcherTableau {
    a: &[
        &[],
        &[0.5],
        &[0.0, 0.75],
        &[2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0],
    ],
    c: &[0.0, 0.5, 0.75, 1.0],
    b: &[7.0 / 24.0, 0.25, 1.0 / 3.0, 0.125],
    b_embedded: Some(&[2.0 / 9.0, 1.0 / 3.0, 4.0 / 9.0, 0.0]),
    error_estimate_order: 2,
};

/// A fixed-step method of a user tableau: upstream `svIntegratorRungeKutta`.
#[derive(Clone, Copy, Debug)]
pub struct FixedTableau(pub ButcherTableau);

impl<D: DynamicObject + 'static> Integrator<D> for FixedTableau {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput) {
        basilisk_rs::integrators::propagate_explicit_rk(
            &self.0,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        )
    }
}

/// Upstream `svIntegratorAdaptiveRungeKutta` for a [`Spacecraft`]; see the
/// module docs.
#[derive(Clone, Copy, Debug)]
pub struct UpstreamAdaptive {
    /// `a`, `c`; `b` is upstream's `bArray`, `b_embedded` its `bStarArray`.
    pub tableau: ButcherTableau,
    /// Upstream `methodLargestOrder`.
    pub largest_order: f64,
    /// Upstream `relTol` (default `1e-4`).
    pub rel_tol: f64,
    /// Upstream `absTol` (default `1e-8`).
    pub abs_tol: f64,
    /// Upstream `safetyFactorForNextStepSize` (default 0.9).
    pub safety: f64,
    /// Upstream `maximumFactorIncreaseForNextStepSize` (default 4).
    pub max_increase: f64,
    /// Upstream `minimumFactorDecreaseForNextStepSize` (default 0.1).
    pub min_decrease: f64,
}

impl UpstreamAdaptive {
    /// Upstream's constructor with its default tolerances and factors.
    pub fn new(tableau: ButcherTableau, largest_order: f64) -> Self {
        assert!(
            tableau.b_embedded.is_some(),
            "an adaptive tableau needs b* weights"
        );
        Self {
            tableau,
            largest_order,
            rel_tol: 1.0e-4,
            abs_tol: 1.0e-8,
            safety: 0.9,
            max_increase: 4.0,
            min_decrease: 0.1,
        }
    }

    /// Upstream `svIntegratorRKF45`: basilisk-rs's [`RKF45`] tableau holds
    /// upstream's `bArray` (fifth order) in `b` and `bStarArray` (fourth
    /// order, propagated) in `b_embedded`; `methodLargestOrder = 5`.
    pub fn rkf45() -> Self {
        Self::new(RKF45, 5.0)
    }

    /// Upstream `svIntegratorRKF78` ([`RKF78`]: `bArray` eighth order,
    /// `bStarArray` seventh order); `methodLargestOrder = 8`.
    pub fn rkf78() -> Self {
        Self::new(RKF78, 8.0)
    }

    /// The same method with upstream's `relTol` and `absTol` set.
    pub fn with_tolerances(mut self, rel_tol: f64, abs_tol: f64) -> Self {
        self.rel_tol = rel_tol;
        self.abs_tol = abs_tol;
        self
    }

    /// Upstream `computeFlatMaxRelativeError` for one trial: the largest
    /// whole-state error ratio of the `b` (`low`) and `b*` (`high`)
    /// candidates and of the integrated gravity.
    fn max_relative_error(
        &self,
        low: &IntegratedState,
        high: &IntegratedState,
        gravity_velocity_low: Vector3<f64>,
        gravity_velocity_high: Vector3<f64>,
    ) -> f64 {
        let ratio = |error: f64, magnitude: f64| {
            let threshold = magnitude * self.rel_tol + self.abs_tol;
            if threshold == 0.0 {
                if error == 0.0 { 0.0 } else { f64::INFINITY }
            } else {
                error / threshold
            }
        };
        let vector =
            |low: Vector3<f64>, high: Vector3<f64>| ratio((high - low).norm(), high.norm());
        let mut largest = vector(
            low.position_wrt_central_body_m,
            high.position_wrt_central_body_m,
        )
        .max(vector(
            low.velocity_wrt_central_body_mps,
            high.velocity_wrt_central_body_mps,
        ))
        .max(vector(low.sigma_bn, high.sigma_bn))
        .max(vector(low.omega_radps, high.omega_radps))
        .max(vector(gravity_velocity_low, gravity_velocity_high));
        for (low, high) in low.effector_states.iter().zip(&high.effector_states) {
            let error = low
                .iter()
                .zip(high)
                .map(|(l, h)| (h - l) * (h - l))
                .sum::<f64>()
                .sqrt();
            let magnitude = high.iter().map(|h| h * h).sum::<f64>().sqrt();
            largest = largest.max(ratio(error, magnitude));
        }
        assert!(
            largest.is_finite() || largest == f64::INFINITY,
            "adaptive Runge-Kutta produced a non-finite truncation error"
        );
        largest
    }
}

fn weighted<'a>(
    derivatives: &'a [StateDerivative],
    weights: &[f64],
) -> Vec<(&'a StateDerivative, f64)> {
    derivatives.iter().zip(weights.iter().copied()).collect()
}

fn gravity_sum(derivatives: &[StateDerivative], weights: &[f64], step: f64) -> Vector3<f64> {
    derivatives
        .iter()
        .zip(weights)
        .fold(Vector3::zeros(), |sum, (k, w)| {
            sum + k.gravity_accel_inertial_mps2 * (w * step)
        })
}

impl Integrator<Spacecraft> for UpstreamAdaptive {
    fn propagate(
        &self,
        spacecraft: &mut Spacecraft,
        state: &IntegratedState,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (IntegratedState, Vector3<f64>) {
        let tableau = &self.tableau;
        let b = tableau.b;
        let b_star = tableau.b_embedded.expect("adaptive tableau");
        let mut base = state.clone();
        let mut elapsed = 0.0_f64;
        let mut step = dt_seconds;
        let mut gravity_delta_velocity = Vector3::zeros();
        while elapsed < dt_seconds {
            assert!(
                step.is_finite() && step > 0.0 && elapsed + step > elapsed,
                "adaptive Runge-Kutta integration cannot make representable forward progress"
            );
            let mut stages: Vec<StateDerivative> = Vec::with_capacity(tableau.c.len());
            for (row, node) in tableau.a.iter().zip(tableau.c) {
                let trial = spacecraft.state_with_derivatives(&base, &weighted(&stages, row), step);
                let stage_seconds = elapsed + node * step;
                stages.push(spacecraft.equations_of_motion(
                    &trial,
                    current_sim_nanos.saturating_add((stage_seconds * 1.0e9).max(0.0) as u64),
                    current_epoch + Duration::from_seconds(stage_seconds),
                ));
            }
            let low = spacecraft.state_with_derivatives(&base, &weighted(&stages, b), step);
            let high = spacecraft.state_with_derivatives(&base, &weighted(&stages, b_star), step);
            let gravity_low = gravity_sum(&stages, b, step);
            let gravity_high = gravity_sum(&stages, b_star, step);
            let max_error = self.max_relative_error(
                &low,
                &high,
                base.velocity_wrt_central_body_mps + gravity_low,
                base.velocity_wrt_central_body_mps + gravity_high,
            );
            if max_error <= 1.0 {
                elapsed += step;
                let (next, gravity) = spacecraft.combine(
                    &base,
                    stages.into_iter().zip(b_star.iter().copied()).collect(),
                    step,
                );
                base = next;
                gravity_delta_velocity += gravity;
            }
            let next_step = (self.safety * step * (1.0 / max_error).powf(1.0 / self.largest_order))
                .min(step * self.max_increase)
                .max(step * self.min_decrease)
                .min(dt_seconds - elapsed);
            if elapsed < dt_seconds {
                assert!(
                    next_step.is_finite() && next_step > 0.0 && elapsed + next_step > elapsed,
                    "adaptive Runge-Kutta error control reduced the time step below representable \
                     forward progress"
                );
            }
            step = next_step;
        }
        (base, gravity_delta_velocity)
    }
}
