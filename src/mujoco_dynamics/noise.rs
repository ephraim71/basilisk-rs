//! Noise processes integrated in a scene's dynamics task: ports of upstream
//! Basilisk `MeanRevertingNoise` and `InhomogeneousGeometricBrownianMotion`
//! (`simulation/mujocoDynamics/_GeneralModuleFiles`).
//!
//! Upstream's two abstract `StatefulSysModel`s integrate one scalar state
//! and hand it to a subclass's `writeOutput`, which scales a message by it
//! ([`StochasticAtmDensity`](super::stochastic_atm_density::StochasticAtmDensity),
//! [`StochasticDragCoeff`](super::stochastic_drag_coeff::StochasticDragCoeff),
//! [`IgbmAtmDensity`](super::igbm_atm_density::IgbmAtmDensity)). The same
//! processes already exist in basilisk-rs as spacecraft state effectors, and
//! these types reuse them for the drift and diffusion:
//!
//! - [`MeanRevertingNoise`] is the Ornstein-Uhlenbeck process
//!   `dx = -x/τ dt + σ_st √(2/τ) dW` of
//!   [`MeanRevertingNoiseStateEffector`];
//! - [`InhomogeneousGeometricBrownianMotion`] is
//!   `dX = (μ - X)/τ dt + σ X dW`, `σ² = (2/τ) σ_st² / (μ² + σ_st²)`, of
//!   [`IgbmNoiseStateEffector`], which integrates the correction
//!   `δ = X - 1`; the state here is reported as `X`.
//!
//! Each has one noise source, so the scene needs a stochastic integrator
//! (a deterministic one integrates the drift only). Upstream's setters raise
//! `BasiliskError` on an invalid parameter; here they panic with the
//! effectors' validation message.

use nalgebra::Vector3;

use super::task::{ModelStateSpec, TaskContext};
use crate::dynamics::igbm_noise_state_effector::{
    IgbmNoiseStateEffector, IgbmNoiseStateEffectorConfig,
};
use crate::dynamics::mean_reverting_noise_state_effector::{
    MeanRevertingNoiseStateEffector, MeanRevertingNoiseStateEffectorConfig,
};
use crate::spacecraft::StateEffector;

/// The Ornstein-Uhlenbeck state of upstream `MeanRevertingNoise`, in
/// stationary form.
#[derive(Clone, Debug)]
pub struct MeanRevertingNoise {
    effector: MeanRevertingNoiseStateEffector,
}

impl Default for MeanRevertingNoise {
    /// Upstream defaults: `σ_st = 0`, `τ = 1 s`, `x(0) = 0`.
    fn default() -> Self {
        Self {
            effector: MeanRevertingNoiseStateEffector::new(
                MeanRevertingNoiseStateEffectorConfig::new("meanRevertingState"),
            ),
        }
    }
}

impl MeanRevertingNoise {
    fn reconfigure(&mut self, change: impl FnOnce(&mut MeanRevertingNoiseStateEffectorConfig)) {
        let mut config = self.effector.config.clone();
        change(&mut config);
        let value = self.effector.state_value();
        self.effector = MeanRevertingNoiseStateEffector::new(config);
        self.effector.load_state(&[value]);
    }

    /// Upstream `getStationaryStd` [-].
    pub fn stationary_std(&self) -> f64 {
        self.effector.config.stationary_std
    }

    /// Upstream `setStationaryStd`.
    ///
    /// # Panics
    ///
    /// If `sigma` is negative or not finite.
    pub fn set_stationary_std(&mut self, sigma: f64) {
        self.reconfigure(|config| config.stationary_std = sigma);
    }

    /// Upstream `getTimeConstant` \[s\].
    pub fn time_constant(&self) -> f64 {
        self.effector.config.time_constant_s
    }

    /// Upstream `setTimeConstant`.
    ///
    /// # Panics
    ///
    /// If `tau` is not positive and finite.
    pub fn set_time_constant(&mut self, tau: f64) {
        self.reconfigure(|config| config.time_constant_s = tau);
    }

    /// Upstream `getTheta` \[1/s\]: the reversion rate `1/τ`.
    pub fn theta(&self) -> f64 {
        1.0 / self.time_constant()
    }

    /// Upstream `getSigma`: the diffusion `σ_st √(2/τ)`.
    pub fn sigma(&self) -> f64 {
        self.stationary_std() * (2.0 / self.time_constant()).sqrt()
    }

    /// Upstream `getStateValue`: the committed state.
    pub fn state_value(&self) -> f64 {
        self.effector.state_value()
    }

    /// Upstream `setStateValue`, as the initial value of the next scene
    /// reset; after it, set the live state with
    /// [`MjScene::set_task_model_state`](super::MjScene::set_task_model_state).
    ///
    /// # Panics
    ///
    /// If `value` is not finite.
    pub fn set_state_value(&mut self, value: f64) {
        self.reconfigure(|config| config.initial_value = value);
        self.effector.load_state(&[value]);
    }

    /// Upstream `registerStates`: the state `meanRevertingState`, one noise
    /// source.
    pub(crate) fn state_spec(&self) -> ModelStateSpec {
        ModelStateSpec::new("meanRevertingState", self.effector.initial_state())
            .with_noise_sources(1)
    }

    pub(crate) fn load_state(&mut self, state: &[f64]) {
        self.effector.load_state(state);
    }

    /// Upstream `UpdateState`'s drift and diffusion of state `state` of
    /// `context`; returns the stage value for `writeOutput`.
    pub(crate) fn evaluate(&self, context: &mut TaskContext<'_>, state: usize) -> f64 {
        let x = context.state(state).to_vec();
        let drift = self
            .effector
            .compute_derivatives(&x, Vector3::zeros(), Vector3::zeros());
        context.set_derivative(state, &drift);
        let diffusion = self.effector.compute_diffusion(&x);
        context.set_diffusion(state, 0, &diffusion[0]);
        x[0]
    }
}

/// The multiplicative-noise state of upstream
/// `InhomogeneousGeometricBrownianMotion`, in stationary form.
#[derive(Clone, Debug)]
pub struct InhomogeneousGeometricBrownianMotion {
    effector: IgbmNoiseStateEffector,
}

impl Default for InhomogeneousGeometricBrownianMotion {
    /// Upstream defaults: `μ = 1`, `σ_st = 0`, `τ = 1 s`, starting at the
    /// mean.
    fn default() -> Self {
        Self {
            effector: IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig::new(
                "inhomogeneousGBMState",
            )),
        }
    }
}

impl InhomogeneousGeometricBrownianMotion {
    fn reconfigure(&mut self, change: impl FnOnce(&mut IgbmNoiseStateEffectorConfig)) {
        let mut config = self.effector.config.clone();
        change(&mut config);
        let value = self.effector.state_value();
        self.effector = IgbmNoiseStateEffector::new(config);
        self.effector.load_state(&[value]);
    }

    /// Upstream `getMean` [-]: the reversion level `μ`.
    pub fn mean(&self) -> f64 {
        self.effector.config.mean
    }

    /// Upstream `setMean`. Upstream's default initial state is the mean, so
    /// until [`set_state_value`](Self::set_state_value) the initial state
    /// follows it.
    ///
    /// # Panics
    ///
    /// If `mean` is not positive and finite.
    pub fn set_mean(&mut self, mean: f64) {
        self.reconfigure(|config| config.mean = mean);
        if self.effector.config.initial_correction.is_none() {
            self.effector.load_state(&[mean - 1.0]);
        }
    }

    /// Upstream `getStationaryStd` [-].
    pub fn stationary_std(&self) -> f64 {
        self.effector.config.stationary_std
    }

    /// Upstream `setStationaryStd`.
    ///
    /// # Panics
    ///
    /// If `sigma` is negative or not finite.
    pub fn set_stationary_std(&mut self, sigma: f64) {
        self.reconfigure(|config| config.stationary_std = sigma);
    }

    /// Upstream `getTimeConstant` \[s\].
    pub fn time_constant(&self) -> f64 {
        self.effector.config.time_constant_s
    }

    /// Upstream `setTimeConstant`.
    ///
    /// # Panics
    ///
    /// If `tau` is not positive and finite.
    pub fn set_time_constant(&mut self, tau: f64) {
        self.reconfigure(|config| config.time_constant_s = tau);
    }

    /// Upstream `getTheta` \[1/s\]: `1/τ`.
    pub fn theta(&self) -> f64 {
        1.0 / self.time_constant()
    }

    /// Upstream `getSigma`: the volatility
    /// `√((2/τ) σ_st² / (μ² + σ_st²))`.
    pub fn sigma(&self) -> f64 {
        let (mean, std) = (self.mean(), self.stationary_std());
        ((2.0 / self.time_constant()) * std * std / (mean * mean + std * std)).sqrt()
    }

    /// Upstream `getStateValue`: the committed state `X = 1 + δ`.
    pub fn state_value(&self) -> f64 {
        1.0 + self.effector.state_value()
    }

    /// Upstream `setStateValue`, as the initial value of the next scene
    /// reset (see [`MeanRevertingNoise::set_state_value`]).
    ///
    /// # Panics
    ///
    /// If `value` is not positive (upstream: "state must be > 0"; the
    /// multiplicative process is only well posed from a positive state).
    pub fn set_state_value(&mut self, value: f64) {
        assert!(
            value > 0.0,
            "InhomogeneousGeometricBrownianMotion::setStateValue: state must be > 0, got {value}"
        );
        self.reconfigure(|config| config.initial_correction = Some(value - 1.0));
        self.effector.load_state(&[value - 1.0]);
    }

    /// Upstream `registerStates`: the state `inhomogeneousGBMState` (here
    /// its correction `δ`), one noise source.
    pub(crate) fn state_spec(&self) -> ModelStateSpec {
        ModelStateSpec::new("inhomogeneousGBMState", self.effector.initial_state())
            .with_noise_sources(1)
    }

    pub(crate) fn load_state(&mut self, state: &[f64]) {
        self.effector.load_state(state);
    }

    /// Upstream `UpdateState`'s drift and diffusion of state `state` of
    /// `context`; returns the stage value `X` for `writeOutput`.
    pub(crate) fn evaluate(&self, context: &mut TaskContext<'_>, state: usize) -> f64 {
        let delta = context.state(state).to_vec();
        let drift = self
            .effector
            .compute_derivatives(&delta, Vector3::zeros(), Vector3::zeros());
        context.set_derivative(state, &drift);
        let diffusion = self.effector.compute_diffusion(&delta);
        context.set_diffusion(state, 0, &diffusion[0]);
        1.0 + delta[0]
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_inhomogeneousGeometricBrownianMotion.py`. The
    //! integrator draws basilisk-rs's seeded Wiener increments rather than
    //! upstream's Mersenne Twister, so the path differs; the statistics and
    //! tolerances are upstream's. Upstream's Python-subclass variant is the
    //! test model here.

    use super::*;
    use crate::Module;
    use crate::integrators::stochastic::EulerMaruyama;
    use crate::mujoco_dynamics::tests::{run, sec2nano};
    use crate::mujoco_dynamics::{MjScene, SceneModel};

    /// Upstream's `PyIGBM`: surfaces the state of every step.
    struct SampledIgbm {
        igbm: InhomogeneousGeometricBrownianMotion,
    }

    impl SceneModel for SampledIgbm {
        fn model_tag(&self) -> &str {
            "igbm"
        }

        fn state_specs(&self) -> Vec<ModelStateSpec> {
            vec![self.igbm.state_spec()]
        }

        fn load_states(&mut self, states: &[&[f64]]) {
            self.igbm.load_state(states[0]);
        }

        fn scene_update(&mut self, context: &mut TaskContext<'_>) {
            self.igbm.evaluate(context, 0);
        }
    }

    /// Upstream `test_inhomogeneousGeometricBrownianMotion`: one long
    /// Euler-Maruyama path has the configured stationary mean and variance
    /// and stays positive.
    #[test]
    fn stationary_statistics_match_and_path_stays_positive() {
        let (dt, tf) = (0.001, 400.0);
        let (mean, tau, sigma_st) = (1.0, 0.2, 1.0 / 3.0);

        let mut igbm = InhomogeneousGeometricBrownianMotion::default();
        igbm.set_mean(mean);
        igbm.set_time_constant(tau);
        igbm.set_stationary_std(sigma_st);
        assert_eq!(igbm.mean(), mean);
        assert_eq!(igbm.time_constant(), tau);
        assert_eq!(igbm.stationary_std(), sigma_st);
        assert_eq!(igbm.theta(), 1.0 / tau);
        let expected_sigma =
            ((2.0 / tau) * sigma_st * sigma_st / (mean * mean + sigma_st * sigma_st)).sqrt();
        assert!((igbm.sigma() - expected_sigma).abs() <= 1e-14 * expected_sigma);
        igbm.set_state_value(mean);
        assert_eq!(igbm.state_value(), mean);

        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        scene.set_integrator(EulerMaruyama::new().with_seed(0));
        let id = scene.add_model_to_dynamics_task(SampledIgbm { igbm }, -1);
        scene.init();
        let mut samples = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(tf), |scene, _| {
            samples.push(scene.task_model(id).igbm.state_value());
        });

        let burn = (5.0 * tau / dt).round() as usize;
        let stationary = &samples[burn..];
        let n = stationary.len() as f64;
        let sample_mean = stationary.iter().sum::<f64>() / n;
        let variance = stationary
            .iter()
            .map(|x| (x - sample_mean).powi(2))
            .sum::<f64>()
            / (n - 1.0);
        assert!(
            samples.iter().all(|x| *x > 0.0),
            "the path went non-positive"
        );
        assert!(
            (sample_mean - mean).abs() <= 0.02 + 0.05 * mean,
            "mean {sample_mean}"
        );
        let target = sigma_st * sigma_st;
        assert!(
            (variance - target).abs() <= 0.2 * target,
            "variance {variance}"
        );
    }

    /// Upstream `test_inhomogeneousGeometricBrownianMotion_rejectsNonPositiveState`.
    #[test]
    fn rejects_non_positive_state() {
        for bad in [0.0, -0.5] {
            let result = std::panic::catch_unwind(|| {
                InhomogeneousGeometricBrownianMotion::default().set_state_value(bad)
            });
            assert!(result.is_err(), "{bad} accepted");
        }
        let mut igbm = InhomogeneousGeometricBrownianMotion::default();
        igbm.set_state_value(0.5);
        assert_eq!(igbm.state_value(), 0.5);
    }

    /// Upstream `test_inhomogeneousGeometricBrownianMotion_rejectsBadParameters`.
    #[test]
    fn rejects_bad_parameters() {
        for bad in [0.0, -1.0] {
            assert!(
                std::panic::catch_unwind(|| {
                    InhomogeneousGeometricBrownianMotion::default().set_mean(bad)
                })
                .is_err()
            );
        }
        for bad in [0.0, -0.1] {
            assert!(
                std::panic::catch_unwind(|| {
                    InhomogeneousGeometricBrownianMotion::default().set_time_constant(bad)
                })
                .is_err()
            );
        }
        assert!(
            std::panic::catch_unwind(|| {
                InhomogeneousGeometricBrownianMotion::default().set_stationary_std(-0.01)
            })
            .is_err()
        );
        let mut igbm = InhomogeneousGeometricBrownianMotion::default();
        igbm.set_mean(2.0);
        igbm.set_time_constant(0.5);
        igbm.set_stationary_std(0.0);
        assert_eq!(
            (igbm.mean(), igbm.time_constant(), igbm.stationary_std()),
            (2.0, 0.5, 0.0)
        );
    }

    /// Upstream `MeanRevertingNoise` setter checks (`setTimeConstant`,
    /// `setStationaryStd`) and its derived parameters.
    #[test]
    fn mean_reverting_noise_parameters() {
        let mut noise = MeanRevertingNoise::default();
        assert_eq!((noise.stationary_std(), noise.time_constant()), (0.0, 1.0));
        noise.set_stationary_std(0.8);
        noise.set_time_constant(0.1);
        assert_eq!(noise.theta(), 10.0);
        assert!((noise.sigma() - 0.8 * 20.0_f64.sqrt()).abs() < 1e-12);
        assert!(
            std::panic::catch_unwind(|| MeanRevertingNoise::default().set_time_constant(0.0))
                .is_err()
        );
        assert!(
            std::panic::catch_unwind(|| MeanRevertingNoise::default().set_stationary_std(-1.0))
                .is_err()
        );
    }
}
