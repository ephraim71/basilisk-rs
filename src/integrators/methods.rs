//! Integrators as values: the [`Integrator`] trait plus one struct per built-in
//! method.
//!
//! Each built-in method is a small struct that delegates to the Butcher-tableau
//! drivers in [`super::butcher`]. To add a *custom* integrator, implement
//! [`Integrator`] for your own type and derive `Clone` + `Debug` — it can then
//! be dropped straight into `SpacecraftConfig::integrator`. The step itself can
//! reuse [`propagate_explicit_rk`] / [`propagate_adaptive_rk`] with a custom
//! [`ButcherTableau`](super::butcher::ButcherTableau), or be written from scratch
//! against the [`DynamicObject`] trait.

use super::butcher::{
    AdaptiveController, DOPRI45, EULER, HEUN, MIDPOINT, RALSTON, RK4, RK4_THREE_EIGHTHS, RKF45,
    propagate_adaptive_rk, propagate_explicit_rk,
};
use super::traits::DynamicObject;
use hifitime::Epoch;
use std::fmt;

/// A numerical integration scheme: advances a [`DynamicObject`] by one step of
/// `dt_seconds`, returning the next state and the step's
/// [`StepOutput`](DynamicObject::StepOutput).
///
/// Implement this — and derive `Clone` + `Debug` — to define a custom
/// integrator that plugs into `SpacecraftConfig::integrator`. `Send` is required
/// because a `Spacecraft` is a [`Module`](crate::Module), which must be `Send`.
pub trait Integrator<D: DynamicObject + 'static>: IntegratorClone<D> + fmt::Debug + Send {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput);
}

/// Object-safe cloning support for `Box<dyn Integrator<D>>`.
///
/// Blanket-implemented for every `Clone` integrator, so implementors get it for
/// free just by deriving `Clone`; there is never a reason to implement it by hand.
pub trait IntegratorClone<D: DynamicObject + 'static> {
    fn clone_box(&self) -> Box<dyn Integrator<D>>;
}

impl<D, T> IntegratorClone<D> for T
where
    D: DynamicObject + 'static,
    T: Integrator<D> + Clone + 'static,
{
    fn clone_box(&self) -> Box<dyn Integrator<D>> {
        Box::new(self.clone())
    }
}

impl<D: DynamicObject + 'static> Clone for Box<dyn Integrator<D>> {
    fn clone(&self) -> Self {
        self.clone_box()
    }
}

/// Declares a zero-config fixed-step integrator struct bound to a tableau.
macro_rules! fixed_step_method {
    ($(#[$meta:meta])* $name:ident => $tableau:ident) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name;

        impl<D: DynamicObject + 'static> Integrator<D> for $name {
            fn propagate(
                &self,
                object: &mut D,
                state: &D::State,
                current_sim_nanos: u64,
                current_epoch: Epoch,
                dt_seconds: f64,
            ) -> (D::State, D::StepOutput) {
                propagate_explicit_rk(
                    &$tableau,
                    object,
                    state,
                    current_sim_nanos,
                    current_epoch,
                    dt_seconds,
                )
            }
        }
    };
}

/// Declares an adaptive integrator struct that carries an [`AdaptiveController`].
macro_rules! adaptive_method {
    ($(#[$meta:meta])* $name:ident => $tableau:ident) => {
        $(#[$meta])*
        #[derive(Clone, Copy, Debug, Default)]
        pub struct $name {
            /// Step-size controller: tolerances, safety factor and step bounds.
            pub controller: AdaptiveController,
        }

        impl $name {
            /// Builds the method with an explicit step-size controller. Use
            /// `Default::default()` for the standard tolerances instead.
            pub fn new(controller: AdaptiveController) -> Self {
                Self { controller }
            }
        }

        impl<D: DynamicObject + 'static> Integrator<D> for $name {
            fn propagate(
                &self,
                object: &mut D,
                state: &D::State,
                current_sim_nanos: u64,
                current_epoch: Epoch,
                dt_seconds: f64,
            ) -> (D::State, D::StepOutput) {
                propagate_adaptive_rk(
                    &$tableau,
                    &self.controller,
                    object,
                    state,
                    current_sim_nanos,
                    current_epoch,
                    dt_seconds,
                )
            }
        }
    };
}

fixed_step_method! {
    /// Forward (explicit) Euler, order 1. One evaluation per step.
    Euler => EULER
}
fixed_step_method! {
    /// Explicit midpoint method, order 2. Two evaluations per step.
    Midpoint => MIDPOINT
}
fixed_step_method! {
    /// Heun's method (explicit trapezoid), order 2. Two evaluations per step.
    Heun => HEUN
}
fixed_step_method! {
    /// Ralston's method, order 2 with minimal truncation error.
    Ralston => RALSTON
}
fixed_step_method! {
    /// Classic ("Kutta") Runge-Kutta, order 4. The default integrator.
    Rk4 => RK4
}
fixed_step_method! {
    /// Runge-Kutta "3/8 rule", order 4 with smaller error constants than [`Rk4`].
    Rk4ThreeEighths => RK4_THREE_EIGHTHS
}

adaptive_method! {
    /// Adaptive Runge-Kutta-Fehlberg 4(5). Six evaluations per accepted step.
    Rkf45 => RKF45
}
adaptive_method! {
    /// Adaptive Dormand-Prince 5(4), the workhorse of many ODE suites.
    Dopri45 => DOPRI45
}

#[cfg(test)]
mod tests;
