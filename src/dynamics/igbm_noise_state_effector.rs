//! Inhomogeneous geometric Brownian motion noise state: port of upstream
//! Basilisk `simulation/dynamics/igbmNoiseStateEffector`.
//!
//! The effector integrates one scalar correction `δ` with the spacecraft,
//! whose factor `X = 1 + δ` follows the Ito SDE
//!
//! ```text
//! dX = (μ - X)/τ dt + σ X dW,   so   dδ = (μ - 1 - δ)/τ dt + σ (1 + δ) dW.
//! ```
//!
//! It is the multiplicative-noise counterpart of
//! [`MeanRevertingNoiseStateEffector`](super::mean_reverting_noise_state_effector::MeanRevertingNoiseStateEffector),
//! with the same `(1 + δ)` form for consumers that scale a quantity by it.
//! Like the other noise states it is configured in stationary form: the mean
//! `μ` of the factor, the time constant `τ` and the stationary standard
//! deviation `σ_st`, from which the volatility
//! `σ² = (2/τ) σ_st² / (μ² + σ_st²)` follows, so the factor's stationary mean
//! and standard deviation are exactly `μ` and `σ_st`.
//!
//! The exact factor stays positive, but the SDE is integrated as written, so
//! an explicit step can occasionally make `1 + δ ≤ 0` for a large step or
//! increment; a consumer that needs a positive quantity clamps it downstream.
//! The diffusion is multiplicative, so a strong stochastic integrator
//! (Euler-Maruyama or an SRI method) reproduces the sample paths; a
//! deterministic one integrates only the drift. The effector has no mass and
//! no messages.
//!
//! Upstream's setters reject non-finite values, a non-positive mean or time
//! constant, a negative stationary standard deviation and an initial
//! correction at or below `-1` with a `BasiliskError`;
//! [`IgbmNoiseStateEffectorConfig::validate`] reports the same violations and
//! [`IgbmNoiseStateEffector::new`] panics on them. Upstream names the state in
//! its state manager (`nameOfState`); the effector's
//! [`name`](crate::spacecraft::StateEffector::name) is that name.

use std::any::Any;

use nalgebra::Vector3;

use crate::spacecraft::{BackSubMatrices, StateEffector};

/// Parameters of an [`IgbmNoiseStateEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct IgbmNoiseStateEffectorConfig {
    /// Upstream `nameOfState`: the effector and state name.
    pub name: String,
    /// Upstream `mean`: mean level `μ` of the factor `1 + δ` [-], finite and
    /// positive. Default 1 (a mean-preserving correction).
    pub mean: f64,
    /// Upstream `sigmaStationary`: stationary standard deviation `σ_st` of the
    /// factor [-], finite and non-negative. Default 0.
    pub stationary_std: f64,
    /// Upstream `timeConstant`: time constant `τ` \[s\], finite and positive.
    /// Default 1.
    pub time_constant_s: f64,
    /// Upstream `stateInit` when `stateInitSet`: the initial correction `δ`
    /// [-], finite and above `-1` so the initial factor is positive. `None`
    /// (upstream's default) starts the factor at its mean, `δ = μ - 1`, since
    /// a zero factor is degenerate for the multiplicative diffusion.
    pub initial_correction: Option<f64>,
}

impl IgbmNoiseStateEffectorConfig {
    /// Upstream defaults with the given name.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mean: 1.0,
            stationary_std: 0.0,
            time_constant_s: 1.0,
            initial_correction: None,
        }
    }

    /// Upstream's setter checks (`setMean`, `setStationaryStd`,
    /// `setTimeConstant`, `setStateValue`).
    pub fn validate(&self) -> Result<(), String> {
        if !self.mean.is_finite() || self.mean <= 0.0 {
            return Err(format!("mean must be finite and > 0, got {}", self.mean));
        }
        if !self.stationary_std.is_finite() || self.stationary_std < 0.0 {
            return Err(format!(
                "stationary_std must be finite and >= 0, got {}",
                self.stationary_std
            ));
        }
        if !self.time_constant_s.is_finite() || self.time_constant_s <= 0.0 {
            return Err(format!(
                "time_constant_s must be finite and > 0, got {}",
                self.time_constant_s
            ));
        }
        if let Some(correction) = self.initial_correction
            && (!correction.is_finite() || correction <= -1.0)
        {
            return Err(format!(
                "initial_correction must be finite and > -1 (the initial factor 1 + delta \
                 must be positive), got {correction}"
            ));
        }
        Ok(())
    }

    /// The initial correction: the configured one, or `μ - 1`.
    pub fn initial_value(&self) -> f64 {
        self.initial_correction.unwrap_or(self.mean - 1.0)
    }
}

/// Scalar IGBM correction state effector: upstream `IgbmNoiseStateEffector`.
#[derive(Clone, Debug)]
pub struct IgbmNoiseStateEffector {
    pub config: IgbmNoiseStateEffectorConfig,
    value: f64,
}

impl IgbmNoiseStateEffector {
    /// # Panics
    ///
    /// If `config` fails [`IgbmNoiseStateEffectorConfig::validate`].
    pub fn new(config: IgbmNoiseStateEffectorConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid IgbmNoiseStateEffectorConfig: {message}");
        }
        let value = config.initial_value();
        Self { config, value }
    }

    /// Upstream `getStateValue`: the integrated correction `δ`, or the
    /// initial one before the spacecraft has loaded a state.
    pub fn state_value(&self) -> f64 {
        self.value
    }

    /// The SDE volatility `σ = √(2/τ) σ_st / √(μ² + σ_st²)`. Upstream scales
    /// `μ` and `σ_st` by their maximum before the ratio, so finite parameters
    /// cannot overflow it.
    pub fn volatility(&self) -> f64 {
        let scale = self.config.mean.max(self.config.stationary_std);
        let scaled_mean = self.config.mean / scale;
        let scaled_std = self.config.stationary_std / scale;
        (2.0_f64.sqrt() / self.config.time_constant_s.sqrt())
            * (scaled_std / scaled_mean.hypot(scaled_std))
    }
}

impl StateEffector for IgbmNoiseStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        1
    }

    fn initial_state(&self) -> Vec<f64> {
        vec![self.config.initial_value()]
    }

    fn load_state(&mut self, state: &[f64]) {
        self.value = state[0];
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}

    /// The state has no mass and exerts nothing on the hub.
    fn update_contributions(
        &self,
        _effector_state: &[f64],
        _body_omega_radps: Vector3<f64>,
        _gravity_body_mps2: Vector3<f64>,
        _back_sub: &mut BackSubMatrices,
    ) {
    }

    /// Upstream `computeDerivatives`: the drift `(μ - (1 + δ))/τ`.
    ///
    /// # Panics
    ///
    /// If the drift is not finite (upstream raises `BasiliskError`).
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        _body_trans_accel_mps2: Vector3<f64>,
        _body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        let factor = 1.0 + effector_state[0];
        let drift = (self.config.mean - factor) / self.config.time_constant_s;
        assert!(
            drift.is_finite(),
            "IgbmNoiseStateEffector '{}': state and parameters produce non-finite drift",
            self.config.name
        );
        vec![drift]
    }

    fn noise_source_count(&self) -> usize {
        1
    }

    /// Upstream's `setDiffusion` in `computeDerivatives`: one column,
    /// `σ (1 + δ)`.
    fn compute_diffusion(&self, effector_state: &[f64]) -> Vec<Vec<f64>> {
        let diffusion = self.volatility() * (1.0 + effector_state[0]);
        assert!(
            diffusion.is_finite(),
            "IgbmNoiseStateEffector '{}': state and parameters produce non-finite diffusion",
            self.config.name
        );
        vec![vec![diffusion]]
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_igbmNoiseStateEffector.py`. The spacecraft draws
    //! its Wiener increments from basilisk-rs's seeded generator rather than
    //! upstream's Mersenne Twister, so the realization differs; the statistics
    //! and tolerances are upstream's.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::integrators::stochastic::EulerMaruyama;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::{Module, SimulationContext};

    /// Upstream `test_igbmNoiseStateEffector`: the factor `1 + δ`, sampled
    /// every millisecond for 100 s after a five-time-constant transient, has
    /// the stationary mean `μ` and variance `σ_st²`, and stays positive.
    #[test]
    fn stationary_statistics_match_the_process() {
        let dt = 0.001;
        let tf = 100.0;
        let mean = 1.0;
        let sigma_stationary: f64 = 1.0 / 3.0;
        let time_constant = 0.2;
        let step_nanos = 1_000_000;

        let effector = IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig {
            mean,
            stationary_std: sigma_stationary,
            time_constant_s: time_constant,
            ..IgbmNoiseStateEffectorConfig::new("igbmNoiseState")
        });
        let mut spacecraft = Spacecraft::new(SpacecraftConfig {
            mass_kg: 1.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: step_nanos,
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: Some(Box::new(EulerMaruyama::new().with_seed(0))),
        });
        spacecraft.add_state_effector(effector);
        spacecraft.init();

        let noise_state = |spacecraft: &Spacecraft| {
            spacecraft.state_effectors[0]
                .as_any()
                .downcast_ref::<IgbmNoiseStateEffector>()
                .expect("the first effector is the noise state")
                .clone()
        };
        let initial = noise_state(&spacecraft);
        assert_eq!(initial.config.mean, mean);
        assert_eq!(initial.config.time_constant_s, time_constant);
        assert_eq!(initial.config.stationary_std, sigma_stationary);
        // The default initial correction puts the factor at its mean level.
        assert_eq!(initial.state_value(), mean - 1.0);

        let steps = (tf / dt) as usize;
        let epoch = Epoch::from_gregorian_utc_at_midnight(2026, 1, 1);
        let y: Vec<f64> = (1..=steps as u64)
            .map(|step| {
                spacecraft.update(&SimulationContext {
                    current_sim_nanos: step * step_nanos,
                    current_epoch: epoch,
                });
                1.0 + noise_state(&spacecraft).state_value()
            })
            .collect();

        let burn = (5.0 * time_constant / dt).round() as usize;
        let stationary = &y[burn..];
        let n = stationary.len() as f64;
        let mean_estimate = stationary.iter().sum::<f64>() / n;
        let variance_estimate = stationary
            .iter()
            .map(|value| (value - mean_estimate).powi(2))
            .sum::<f64>()
            / (n - 1.0);

        assert!(
            y.iter().all(|&factor| factor > 0.0),
            "IGBM factor went non-positive"
        );
        assert!(
            (mean_estimate - mean).abs() <= 0.02 + 0.05 * mean,
            "mean {mean_estimate}"
        );
        assert!(
            (variance_estimate - sigma_stationary.powi(2)).abs() <= 0.30 * sigma_stationary.powi(2),
            "variance {variance_estimate}, target {}",
            sigma_stationary.powi(2)
        );
    }

    /// Upstream `test_igbmNoiseStateEffector_rejectsNonPositiveInitialFactor`.
    #[test]
    fn rejects_a_non_positive_initial_factor() {
        for bad in [-1.0, -1.5] {
            let config = IgbmNoiseStateEffectorConfig {
                initial_correction: Some(bad),
                ..IgbmNoiseStateEffectorConfig::new("noise")
            };
            assert!(config.validate().is_err(), "{bad}");
        }
        let effector = IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig {
            initial_correction: Some(-0.5),
            ..IgbmNoiseStateEffectorConfig::new("noise")
        });
        assert_eq!(effector.state_value(), -0.5);
    }

    /// Upstream `test_igbmNoiseStateEffector_rejectsBadParameters`, plus the
    /// non-finite values upstream's setters also reject.
    #[test]
    fn rejects_bad_parameters() {
        let valid = IgbmNoiseStateEffectorConfig::new("noise");
        for config in [
            IgbmNoiseStateEffectorConfig {
                mean: 0.0,
                ..valid.clone()
            },
            IgbmNoiseStateEffectorConfig {
                mean: -1.0,
                ..valid.clone()
            },
            IgbmNoiseStateEffectorConfig {
                time_constant_s: 0.0,
                ..valid.clone()
            },
            IgbmNoiseStateEffectorConfig {
                time_constant_s: -0.1,
                ..valid.clone()
            },
            IgbmNoiseStateEffectorConfig {
                stationary_std: -0.01,
                ..valid.clone()
            },
            IgbmNoiseStateEffectorConfig {
                mean: f64::NAN,
                ..valid.clone()
            },
            IgbmNoiseStateEffectorConfig {
                initial_correction: Some(f64::INFINITY),
                ..valid.clone()
            },
        ] {
            assert!(config.validate().is_err(), "{config:?}");
        }
        let accepted = IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig {
            mean: 2.0,
            time_constant_s: 0.5,
            stationary_std: 0.0,
            ..valid
        });
        assert_eq!(accepted.config.mean, 2.0);
        assert_eq!(accepted.config.time_constant_s, 0.5);
        assert_eq!(accepted.config.stationary_std, 0.0);
        assert_eq!(accepted.state_value(), 1.0);
    }

    #[test]
    #[should_panic(expected = "invalid IgbmNoiseStateEffectorConfig")]
    fn new_panics_on_an_invalid_config() {
        IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig {
            mean: 0.0,
            ..IgbmNoiseStateEffectorConfig::new("noise")
        });
    }

    /// The drift, diffusion and volatility follow the documented formulas,
    /// and the volatility gives the factor its stationary moments:
    /// `σ² = (2/τ) σ_st² / (μ² + σ_st²)`.
    #[test]
    fn drift_diffusion_and_volatility() {
        let effector = IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig {
            mean: 1.5,
            stationary_std: 0.4,
            time_constant_s: 0.25,
            ..IgbmNoiseStateEffectorConfig::new("noise")
        });
        let sigma_squared = 2.0 / 0.25 * 0.4_f64.powi(2) / (1.5_f64.powi(2) + 0.4_f64.powi(2));
        assert!((effector.volatility().powi(2) - sigma_squared).abs() < 1.0e-14);
        assert_eq!(effector.state_value(), 0.5);
        let drift = effector.compute_derivatives(&[0.2], Vector3::zeros(), Vector3::zeros());
        assert!((drift[0] - (1.5 - 1.2) / 0.25).abs() < 1.0e-15);
        let diffusion = effector.compute_diffusion(&[0.2]);
        assert_eq!(diffusion.len(), effector.noise_source_count());
        assert!((diffusion[0][0] - effector.volatility() * 1.2).abs() < 1.0e-15);

        let silent = IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig::new("silent"));
        assert_eq!(silent.volatility(), 0.0);
    }

    /// Huge but finite stationary parameters keep the volatility finite
    /// (upstream's scaled ratio).
    #[test]
    fn volatility_does_not_overflow() {
        let effector = IgbmNoiseStateEffector::new(IgbmNoiseStateEffectorConfig {
            mean: 1.0e300,
            stationary_std: 1.0e300,
            time_constant_s: 2.0,
            ..IgbmNoiseStateEffectorConfig::new("noise")
        });
        assert!((effector.volatility() - 0.5_f64.sqrt()).abs() < 1.0e-15);
    }
}
