//! The stochastic integrators as [`Integrator`] values: one struct per
//! upstream `svStochasticIntegrator*` class, plus [`StrongSra`] and
//! [`StrongSri`] for custom tableaus of the generic upstream templates.
//!
//! Every struct owns a [`StochasticNoise`]. `new()` / `Default` seed a
//! [`RandomGaussianNoiseGenerator`](super::RandomGaussianNoiseGenerator) with
//! [`DEFAULT_NOISE_SEED`](super::DEFAULT_NOISE_SEED); `with_seed` and
//! `set_rng_seed` (upstream `setRNGSeed`) reseed it, and
//! `with_noise_generator` / `set_noise_generator` (upstream
//! `setNoiseGenerator`) install any other source.

use std::sync::{Arc, Mutex};

use hifitime::Epoch;

use super::schemes::{self, Dri1Scheme};
use super::tableaus::{
    Dri1Coefficients, RS1, RS2, SIEA, SIEB, SMEA, SMEB, SOSRA, SOSRI, SRA1, SRIW1, SraTableau,
    SriTableau, W2ITO1, W2ITO2,
};
use super::{GaussianNoiseGenerator, StochasticNoise};
use crate::integrators::Integrator;
use crate::integrators::traits::DynamicObject;

/// Noise-source accessors shared by every stochastic integrator struct.
macro_rules! noise_accessors {
    () => {
        /// Builder: reseeds the default generator with `seed`.
        pub fn with_seed(mut self, seed: u64) -> Self {
            self.noise = StochasticNoise::seeded(seed);
            self
        }

        /// Builder: draws from `generator` (upstream `setNoiseGenerator`).
        pub fn with_noise_generator(
            mut self,
            generator: Arc<Mutex<dyn GaussianNoiseGenerator>>,
        ) -> Self {
            self.noise = StochasticNoise::from_generator(generator);
            self
        }

        /// Reseeds the generator (upstream `setRNGSeed`); a custom generator
        /// may ignore it, as a replay does.
        pub fn set_rng_seed(&self, seed: u64) {
            self.noise.set_seed(seed);
        }

        /// Replaces the generator (upstream `setNoiseGenerator`).
        pub fn set_noise_generator(&mut self, generator: Arc<Mutex<dyn GaussianNoiseGenerator>>) {
            self.noise = StochasticNoise::from_generator(generator);
        }

        /// The generator this integrator draws from.
        pub fn noise(&self) -> &StochasticNoise {
            &self.noise
        }
    };
}

/// Declares a stochastic integrator struct whose step is `$scheme` with the
/// given leading arguments (a tableau, or nothing).
macro_rules! stochastic_method {
    ($(#[$meta:meta])* $name:ident => $scheme:path $(, $argument:expr)*) => {
        $(#[$meta])*
        #[derive(Debug, Default)]
        pub struct $name {
            noise: StochasticNoise,
        }

        impl $name {
            /// The method with the default seeded generator.
            pub fn new() -> Self {
                Self::default()
            }

            noise_accessors!();
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
                $scheme(
                    $($argument,)*
                    &self.noise,
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

stochastic_method! {
    /// Euler-Maruyama, strong order 0.5 and weak order 1: upstream
    /// `svStochasticIntegratorMayurama` (sic).
    EulerMaruyama => schemes::euler_maruyama
}
stochastic_method! {
    /// Euler-Heun, strong order 0.5 for Stratonovich SDEs: upstream
    /// `svStochasticIntegratorEulerHeun`.
    EulerHeun => schemes::euler_heun
}
stochastic_method! {
    /// Derivative-free Runge-Kutta Milstein, strong order 1 for diagonal or
    /// scalar Ito noise: upstream `svStochasticIntegratorRKMil`.
    RkMil => schemes::rk_milstein
}
stochastic_method! {
    /// Roessler SRA1, strong order 1.5 and weak order 2 for additive noise:
    /// upstream `svStochasticIntegratorSRA1`.
    Sra1 => schemes::sra, &SRA1
}
stochastic_method! {
    /// Stability-optimized Roessler SRA, strong order 1.5 for additive noise:
    /// upstream `svStochasticIntegratorSOSRA`.
    Sosra => schemes::sra, &SOSRA
}
stochastic_method! {
    /// Roessler SRIW1, strong order 1.5 and weak order 2 for diagonal or
    /// scalar Ito noise: upstream `svStochasticIntegratorSRIW1`.
    Sriw1 => schemes::sri, &SRIW1
}
stochastic_method! {
    /// Stability-optimized Roessler SRI, strong order 1.5 for diagonal or
    /// scalar Ito noise: upstream `svStochasticIntegratorSOSRI`.
    Sosri => schemes::sri, &SOSRI
}
stochastic_method! {
    /// Tang & Xiao W2Ito1, weak order 2 (deterministic order 3) for Ito SDEs
    /// of any noise structure: upstream `svStochasticIntegratorW2Ito1`.
    W2Ito1 => schemes::w2ito, &W2ITO1
}
stochastic_method! {
    /// Tang & Xiao W2Ito2, weak order 2 (deterministic order 4) for Ito SDEs
    /// of any noise structure: upstream `svStochasticIntegratorW2Ito2`.
    W2Ito2 => schemes::w2ito, &W2ITO2
}
stochastic_method! {
    /// Debrabant-Roessler DRI1, weak order 2 (deterministic order 3) for Ito
    /// SDEs of any noise structure: upstream `svStochasticIntegratorDRI1`.
    Dri1 => schemes::dri1, &Dri1Scheme { coefficients: Dri1Coefficients::dri1(), non_mixing: false }
}
stochastic_method! {
    /// DRI1 without the cross-source terms, exact and cheaper for diagonal
    /// noise: upstream `svStochasticIntegratorDRI1NM` (`setNonMixing(true)`).
    Dri1Nm => schemes::dri1, &Dri1Scheme { coefficients: Dri1Coefficients::dri1(), non_mixing: true }
}
stochastic_method! {
    /// Roessler RI1, weak order 2 for Ito SDEs (DRI1's step):
    /// upstream `svStochasticIntegratorRI1`.
    Ri1 => schemes::dri1, &Dri1Scheme { coefficients: Dri1Coefficients::ri1(), non_mixing: false }
}
stochastic_method! {
    /// Roessler RI3, weak order 2 for Ito SDEs (DRI1's step):
    /// upstream `svStochasticIntegratorRI3`.
    Ri3 => schemes::dri1, &Dri1Scheme { coefficients: Dri1Coefficients::ri3(), non_mixing: false }
}
stochastic_method! {
    /// Roessler RI5, weak order 2 for Ito SDEs (DRI1's step):
    /// upstream `svStochasticIntegratorRI5`.
    Ri5 => schemes::dri1, &Dri1Scheme { coefficients: Dri1Coefficients::ri5(), non_mixing: false }
}
stochastic_method! {
    /// Roessler RI6, weak order 2 for Ito SDEs (DRI1's step):
    /// upstream `svStochasticIntegratorRI6`.
    Ri6 => schemes::dri1, &Dri1Scheme { coefficients: Dri1Coefficients::ri6(), non_mixing: false }
}
stochastic_method! {
    /// Roessler RS1, weak order 2 (deterministic order 2) for Stratonovich
    /// SDEs of any noise structure: upstream `svStochasticIntegratorRS1`.
    ///
    /// Follows the published method for several sources, as upstream does,
    /// rather than StochasticDiffEq.jl's aliasing cache.
    Rs1 => schemes::rs, &RS1
}
stochastic_method! {
    /// Roessler RS2, weak order 2 (deterministic order 3) for Stratonovich
    /// SDEs of any noise structure: upstream `svStochasticIntegratorRS2`.
    Rs2 => schemes::rs, &RS2
}
stochastic_method! {
    /// Tocino & Vigo-Aguiar SIEA (stochastic improved Euler), weak order 2
    /// for diagonal or scalar Ito noise: upstream `svStochasticIntegratorSIEA`.
    Siea => schemes::sie_sme, &SIEA
}
stochastic_method! {
    /// Tocino & Vigo-Aguiar SMEA (stochastic modified Euler), weak order 2:
    /// upstream `svStochasticIntegratorSMEA`.
    Smea => schemes::sie_sme, &SMEA
}
stochastic_method! {
    /// Tocino & Vigo-Aguiar SIEB, weak order 2: upstream
    /// `svStochasticIntegratorSIEB`.
    Sieb => schemes::sie_sme, &SIEB
}
stochastic_method! {
    /// Tocino & Vigo-Aguiar SMEB, weak order 2: upstream
    /// `svStochasticIntegratorSMEB`.
    Smeb => schemes::sie_sme, &SMEB
}
stochastic_method! {
    /// Roessler RDI1WM, weak order 1 (deterministic order 2) for diagonal or
    /// scalar noise: upstream `svStochasticIntegratorRDI1WM`.
    Rdi1Wm => schemes::rdi1wm
}

/// Roessler SRA method with a custom `S`-stage tableau: upstream
/// `svIntegratorStrongStochasticRungeKuttaSRA<S>`.
#[derive(Debug)]
pub struct StrongSra<const S: usize> {
    tableau: SraTableau<S>,
    noise: StochasticNoise,
}

impl<const S: usize> StrongSra<S> {
    /// The method for `tableau`, rejected as upstream's constructor rejects
    /// it (see [`SraTableau::validate`]).
    pub fn new(tableau: SraTableau<S>) -> Result<Self, String> {
        tableau.validate()?;
        Ok(Self {
            tableau,
            noise: StochasticNoise::default(),
        })
    }

    /// The method's tableau.
    pub fn tableau(&self) -> &SraTableau<S> {
        &self.tableau
    }

    noise_accessors!();
}

impl<D: DynamicObject + 'static, const S: usize> Integrator<D> for StrongSra<S> {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput) {
        schemes::sra(
            &self.tableau,
            &self.noise,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        )
    }
}

/// Roessler SRI method with a custom `S`-stage tableau: upstream
/// `svIntegratorStrongStochasticRungeKuttaSRI<S>`.
#[derive(Debug)]
pub struct StrongSri<const S: usize> {
    tableau: SriTableau<S>,
    noise: StochasticNoise,
}

impl<const S: usize> StrongSri<S> {
    /// The method for `tableau`, rejected as upstream's constructor rejects
    /// it (see [`SriTableau::validate`]).
    pub fn new(tableau: SriTableau<S>) -> Result<Self, String> {
        tableau.validate()?;
        Ok(Self {
            tableau,
            noise: StochasticNoise::default(),
        })
    }

    /// The method's tableau.
    pub fn tableau(&self) -> &SriTableau<S> {
        &self.tableau
    }

    noise_accessors!();
}

impl<D: DynamicObject + 'static, const S: usize> Integrator<D> for StrongSri<S> {
    fn propagate(
        &self,
        object: &mut D,
        state: &D::State,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> (D::State, D::StepOutput) {
        schemes::sri(
            &self.tableau,
            &self.noise,
            object,
            state,
            current_sim_nanos,
            current_epoch,
            dt_seconds,
        )
    }
}
