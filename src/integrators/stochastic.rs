//! Stochastic integrators for `dx = f(t, x) dt + Σ_k g_k(t, x) dW_k`: port of
//! upstream `stateVecStochasticIntegrator`, `stochasticRKIntegratorBase`,
//! `flatStochasticWorkspace` and the `svStochasticIntegrator*` methods.
//!
//! An object opts into noise through the diffusion half of
//! [`DynamicObject`]: [`noise_source_count`](DynamicObject::noise_source_count)
//! and [`equations_of_motion_diffusion`](DynamicObject::equations_of_motion_diffusion),
//! whose columns are expressed as derivatives. Every integrator here is an
//! [`Integrator`](super::Integrator), so it drops into
//! `SpacecraftConfig::integrator` like the deterministic ones; on an object
//! without noise it reduces to its drift tableau.
//!
//! Upstream binds a flat view of every registered state and keeps packed
//! stage buffers (`FlatStochasticWorkspace`). Here the stages are the
//! object's own derivative values, combined by weight lists exactly as
//! [`super::propagate_explicit_rk`] does: a stage state is
//! `y + h Σ aⱼ fⱼ` followed by one [`DiffusionIncrement`] per noise source,
//! applied in source order as upstream applies them in local order. Because
//! each step returns a new state rather than writing the live one, upstream's
//! rollback of a failed step has nothing to restore here.
//!
//! Each integrator draws its increments from a [`StochasticNoise`]: a seeded
//! [`RandomGaussianNoiseGenerator`] by default, or any
//! [`GaussianNoiseGenerator`] (such as a [`PrescribedGaussianNoiseGenerator`]
//! replay in tests), shared through `Arc<Mutex<_>>` like upstream's
//! `shared_ptr`. The integrator interface takes `&self`, so the lock is what
//! lets a step advance the generator.
//!
//! Stochastic integration is in beta upstream.

mod methods;
mod noise;
mod schemes;
mod tableaus;

use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

use hifitime::{Duration, Epoch};

use super::butcher::offset_nanos;
use super::traits::{DiffusionIncrement, DynamicObject};

pub use methods::{
    Dri1, Dri1Nm, EulerHeun, EulerMaruyama, Rdi1Wm, Ri1, Ri3, Ri5, Ri6, RkMil, Rs1, Rs2, Siea,
    Sieb, Smea, Smeb, Sosra, Sosri, Sra1, Sriw1, StrongSra, StrongSri, W2Ito1, W2Ito2,
};
pub use noise::{
    DEFAULT_NOISE_SEED, GaussianNoiseGenerator, GaussianNoiseSample, NORMAL_ONESIX_QUANTILE,
    PrescribedGaussianNoiseGenerator, RandomGaussianNoiseGenerator, three_point, two_point,
};
pub use tableaus::{
    Dri1Coefficients, RS1, RS2, RsCoefficients, SIEA, SIEB, SMEA, SMEB, SOSRA, SOSRI, SRA1, SRIW1,
    SieSmeCoefficients, SraTableau, SriTableau, W2ITO1, W2ITO2, W2ItoTableau,
};

/// The random-number source of a stochastic integrator: upstream
/// `StochasticRKIntegratorBase`'s generator ownership (`setRNGSeed`,
/// `setNoiseGenerator`).
#[derive(Debug)]
pub struct StochasticNoise {
    generator: Arc<Mutex<dyn GaussianNoiseGenerator>>,
}

impl StochasticNoise {
    /// A [`RandomGaussianNoiseGenerator`] seeded with `seed`.
    pub fn seeded(seed: u64) -> Self {
        Self::from_generator(Arc::new(Mutex::new(RandomGaussianNoiseGenerator::new(
            seed,
        ))))
    }

    /// Draws from `generator`, which the caller may keep a handle to (for
    /// example to check how many prescribed samples remain).
    pub fn from_generator(generator: Arc<Mutex<dyn GaussianNoiseGenerator>>) -> Self {
        Self { generator }
    }

    /// Reseeds the generator (upstream `setRNGSeed`); a replay ignores it.
    pub fn set_seed(&self, seed: u64) {
        self.lock().set_seed(seed);
    }

    /// Shared handle to the generator.
    pub fn generator(&self) -> Arc<Mutex<dyn GaussianNoiseGenerator>> {
        Arc::clone(&self.generator)
    }

    fn lock(&self) -> MutexGuard<'_, dyn GaussianNoiseGenerator + 'static> {
        self.generator
            .lock()
            .unwrap_or_else(PoisonError::into_inner)
    }

    /// One step's Wiener increments and `auxiliary_count` auxiliary ones
    /// (upstream `generateNoise`), checked against the request.
    pub(crate) fn sample(
        &self,
        noise_count: usize,
        auxiliary_count: usize,
        h: f64,
    ) -> GaussianNoiseSample {
        let sample = self
            .lock()
            .generate_with_auxiliary(noise_count, auxiliary_count, h);
        assert!(
            sample.dw.len() == noise_count && sample.dz.len() >= auxiliary_count,
            "GaussianNoiseGenerator output does not match the requested Wiener and auxiliary \
             counts"
        );
        sample
    }

    /// One step's Wiener increments only (upstream `generateWienerNoise`).
    pub(crate) fn wiener(&self, noise_count: usize, h: f64) -> Vec<f64> {
        self.sample(noise_count, 0, h).dw
    }
}

impl Default for StochasticNoise {
    fn default() -> Self {
        Self::seeded(DEFAULT_NOISE_SEED)
    }
}

/// Upstream `validateStochasticTimeStep` plus the `timeStep == 0` early return
/// every method starts with: a zero step draws no noise and leaves the state
/// unchanged, returning the empty combination.
///
/// # Panics
///
/// If `h` is negative or not finite, before any noise is drawn.
fn zero_step<D: DynamicObject>(
    object: &D,
    state: &D::State,
    h: f64,
) -> Option<(D::State, D::StepOutput)> {
    assert!(
        h.is_finite() && h >= 0.0,
        "stochastic integration requires a finite, nonnegative time step, got {h}"
    );
    (h == 0.0).then(|| object.combine(state, Vec::new(), 0.0))
}

/// One stochastic step in progress: evaluates the drift and diffusion at
/// stage states and builds candidates from the step-start state `base`.
/// Upstream splits this between `StateVecIntegrator::evaluateDerivatives` /
/// `evaluateDiffusions` and `StateVecStochasticIntegrator`'s candidate
/// builders.
struct StochasticStep<'a, D: DynamicObject> {
    object: &'a mut D,
    base: &'a D::State,
    base_nanos: u64,
    base_epoch: Epoch,
    /// Step length `h` \[s\].
    h: f64,
    noise_count: usize,
}

impl<'a, D: DynamicObject> StochasticStep<'a, D> {
    fn new(
        object: &'a mut D,
        base: &'a D::State,
        base_nanos: u64,
        base_epoch: Epoch,
        h: f64,
    ) -> Self {
        let noise_count = object.noise_source_count();
        Self {
            object,
            base,
            base_nanos,
            base_epoch,
            h,
            noise_count,
        }
    }

    /// Simulation time and epoch of the node `t + node·h`.
    fn time(&self, node: f64) -> (u64, Epoch) {
        let seconds = node * self.h;
        (
            offset_nanos(self.base_nanos, seconds),
            self.base_epoch + Duration::from_seconds(seconds),
        )
    }

    /// Drift `f(t + node·h, state)`.
    fn drift(&mut self, state: &D::State, node: f64) -> D::Derivative {
        let (nanos, epoch) = self.time(node);
        self.object.equations_of_motion(state, nanos, epoch)
    }

    /// Every diffusion column `g_k(t + node·h, state)`.
    fn diffusions(&mut self, state: &D::State, node: f64) -> Vec<D::Derivative> {
        let (nanos, epoch) = self.time(node);
        let columns = self
            .object
            .equations_of_motion_diffusion(state, nanos, epoch);
        assert_eq!(
            columns.len(),
            self.noise_count,
            "equations_of_motion_diffusion returned {} columns for {} noise sources",
            columns.len(),
            self.noise_count
        );
        columns
    }

    /// The diffusion column of one source at a stage built for that source,
    /// discarding the others (upstream `captureDiffusion(slot, stage)`).
    fn diffusion(&mut self, state: &D::State, node: f64, source: usize) -> D::Derivative {
        self.diffusions(state, node)
            .into_iter()
            .nth(source)
            .expect("diffusion column count was checked")
    }

    /// Stage state `base + h Σ aⱼ fⱼ` plus the diffusion increments in order
    /// (upstream `buildStochasticCandidate`).
    fn candidate(
        &self,
        drift: &[(&D::Derivative, f64)],
        diffusion: &[DiffusionIncrement<'_, D::Derivative>],
    ) -> D::State {
        self.candidate_over(drift, self.h, diffusion)
    }

    /// [`candidate`](Self::candidate) with the drift taken over `drift_step`
    /// instead of `h`. Upstream builds its pure-diffusion stages from the
    /// last drift combination over a zero step; for an ordinary sum that is
    /// the base state, and passing the same weights keeps a non-linear
    /// update rule seeing what upstream's does.
    fn candidate_over(
        &self,
        drift: &[(&D::Derivative, f64)],
        drift_step: f64,
        diffusion: &[DiffusionIncrement<'_, D::Derivative>],
    ) -> D::State {
        let drifted = self
            .object
            .state_with_derivatives(self.base, drift, drift_step);
        self.object.state_with_diffusion(&drifted, diffusion)
    }

    /// `state` plus the diffusion increments, without drift.
    fn with_diffusion(
        &self,
        state: &D::State,
        diffusion: &[DiffusionIncrement<'_, D::Derivative>],
    ) -> D::State {
        self.object.state_with_diffusion(state, diffusion)
    }

    /// The accepted state and step output: the drift weights combined over
    /// `h`, then the diffusion increments.
    fn finish(
        self,
        drift: Vec<(D::Derivative, f64)>,
        diffusion: &[DiffusionIncrement<'_, D::Derivative>],
    ) -> (D::State, D::StepOutput) {
        self.object
            .combine_with_diffusion(self.base, drift, self.h, diffusion)
    }
}

/// The first `weights.len()` stages with their weights, as the weight list a
/// candidate takes (upstream `FlatStochasticWorkspace::writeDrift`).
fn weighted<'s, T>(stages: &'s [T], weights: &[f64]) -> Vec<(&'s T, f64)> {
    stages.iter().zip(weights.iter().copied()).collect()
}

/// One source's column of each of the first `weights.len()` diffusion stages,
/// with their weights (upstream `FlatStochasticWorkspace::writeDiffusion`).
/// `stages[j][k]` is source `k` at stage `j`.
fn source_terms<'s, T>(stages: &'s [Vec<T>], source: usize, weights: &[f64]) -> Vec<(&'s T, f64)> {
    stages
        .iter()
        .zip(weights.iter().copied())
        .map(|(stage, weight)| (&stage[source], weight))
        .collect()
}

/// One increment per source `k`, each the weighted stage columns of `k`
/// times `pseudo_steps[k]`.
fn source_increments<'s, T>(
    stages: &'s [Vec<T>],
    weights: &[f64],
    pseudo_steps: &[f64],
) -> Vec<DiffusionIncrement<'s, T>> {
    pseudo_steps
        .iter()
        .enumerate()
        .map(|(source, &pseudo_step)| DiffusionIncrement {
            tangents: source_terms(stages, source, weights),
            pseudo_step,
        })
        .collect()
}

/// One increment per column of a single stage, each with weight `weight` and
/// multiplied by `pseudo_steps[k]`.
fn column_increments<'s, T>(
    columns: &'s [T],
    weight: f64,
    pseudo_steps: &[f64],
) -> Vec<DiffusionIncrement<'s, T>> {
    columns
        .iter()
        .zip(pseudo_steps.iter().copied())
        .map(|(column, pseudo_step)| DiffusionIncrement {
            tangents: vec![(column, weight)],
            pseudo_step,
        })
        .collect()
}

#[cfg(test)]
mod tests;
