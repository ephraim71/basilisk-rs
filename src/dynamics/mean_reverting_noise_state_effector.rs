//! Ornstein-Uhlenbeck noise state: port of upstream Basilisk
//! `simulation/dynamics/meanRevertingNoiseStateEffector`.
//!
//! The effector integrates one scalar state `x` with the spacecraft,
//!
//! ```text
//! dx = -x/τ dt + σ_st √(2/τ) dW,
//! ```
//!
//! a mean-reverting process with time constant `τ` whose stationary
//! distribution has zero mean and standard deviation `σ_st`. Consumers that
//! scale a quantity by `(1 + x)` read [`MeanRevertingNoiseStateEffector::state_value`].
//! It has no mass and no messages, and adds nothing to the hub dynamics.
//!
//! The state has one noise source, so the spacecraft needs a stochastic
//! integrator (for example [`EulerMaruyama`](crate::integrators::stochastic::EulerMaruyama));
//! a deterministic one integrates only the drift.
//!
//! Upstream's setters reject a non-finite value, a negative stationary
//! standard deviation and a non-positive time constant with a
//! `BasiliskError` and keep the previous setting; here
//! [`MeanRevertingNoiseStateEffectorConfig::validate`] reports the same
//! violations and [`MeanRevertingNoiseStateEffector::new`] panics on them.
//! Upstream names the state in its state manager (`nameOfState`); the
//! effector's [`name`](crate::spacecraft::StateEffector::name) is that name.

use std::any::Any;

use nalgebra::Vector3;

use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::{BackSubMatrices, StateEffector};

/// Parameters of a [`MeanRevertingNoiseStateEffector`].
#[derive(Clone, Debug, PartialEq)]
pub struct MeanRevertingNoiseStateEffectorConfig {
    /// Upstream `nameOfState`: the effector and state name.
    pub name: String,
    /// Upstream `sigmaStationary`: stationary standard deviation `σ_st` [-],
    /// finite and non-negative. Default 0.
    pub stationary_std: f64,
    /// Upstream `timeConstant`: mean-reversion time constant `τ` \[s\],
    /// finite and positive. Default 1.
    pub time_constant_s: f64,
    /// Upstream `stateInit`: the initial state `x` [-], finite. Default 0.
    pub initial_value: f64,
}

impl MeanRevertingNoiseStateEffectorConfig {
    /// Upstream defaults with the given name.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            stationary_std: 0.0,
            time_constant_s: 1.0,
            initial_value: 0.0,
        }
    }

    /// Upstream's setter checks (`setStationaryStd`, `setTimeConstant`,
    /// `setStateValue`).
    pub fn validate(&self) -> Result<(), String> {
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
        if !self.initial_value.is_finite() {
            return Err(format!(
                "initial_value must be finite, got {}",
                self.initial_value
            ));
        }
        Ok(())
    }
}

/// Scalar Ornstein-Uhlenbeck state effector: upstream
/// `MeanRevertingNoiseStateEffector`.
#[derive(Clone, Debug)]
pub struct MeanRevertingNoiseStateEffector {
    pub config: MeanRevertingNoiseStateEffectorConfig,
    value: f64,
}

impl MeanRevertingNoiseStateEffector {
    /// # Panics
    ///
    /// If `config` fails [`MeanRevertingNoiseStateEffectorConfig::validate`].
    pub fn new(config: MeanRevertingNoiseStateEffectorConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid MeanRevertingNoiseStateEffectorConfig: {message}");
        }
        let value = config.initial_value;
        Self { config, value }
    }

    /// Upstream `getStateValue`: the integrated state, or the initial value
    /// before the spacecraft has loaded one.
    pub fn state_value(&self) -> f64 {
        self.value
    }

    /// The constant diffusion `σ_st √2 / √τ`; taking the square roots before
    /// dividing avoids overflowing `2/τ` for a tiny `τ` (upstream comment).
    pub fn diffusion_amplitude(&self) -> f64 {
        self.config.stationary_std * (2.0_f64.sqrt() / self.config.time_constant_s.sqrt())
    }
}

impl StateEffector for MeanRevertingNoiseStateEffector {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        1
    }

    fn initial_state(&self) -> Vec<f64> {
        vec![self.config.initial_value]
    }

    fn load_state(&mut self, state: &[f64]) {
        self.value = state[0];
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}

    /// The state has no mass and exerts nothing on the hub.
    fn update_contributions(
        &self,
        _effector_state: &[f64],
        _hub_state: &SpacecraftStateMsg,
        _gravity_body_mps2: Vector3<f64>,
        _back_sub: &mut BackSubMatrices,
    ) {
    }

    /// Upstream `computeDerivatives`: the drift `-x/τ`.
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
        let drift = -effector_state[0] / self.config.time_constant_s;
        assert!(
            drift.is_finite(),
            "MeanRevertingNoiseStateEffector '{}': state and parameters produce non-finite drift",
            self.config.name
        );
        vec![drift]
    }

    fn noise_source_count(&self) -> usize {
        1
    }

    /// Upstream's `setDiffusion` in `computeDerivatives`: one constant
    /// column, `σ_st √(2/τ)`.
    fn compute_diffusion(&self, _effector_state: &[f64]) -> Vec<Vec<f64>> {
        let diffusion = self.diffusion_amplitude();
        assert!(
            diffusion.is_finite(),
            "MeanRevertingNoiseStateEffector '{}': parameters produce non-finite diffusion",
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
    //! Port of upstream `test_meanRevertingNoiseStateEffector.py`. The
    //! spacecraft draws its Wiener increments from basilisk-rs's seeded
    //! generator rather than upstream's Mersenne Twister, so the realization
    //! differs; the statistics and tolerances are upstream's.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::*;
    use crate::integrators::stochastic::EulerMaruyama;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};
    use crate::{Module, SimulationContext};

    /// A spacecraft at rest, integrated with Euler-Maruyama seeded with 0
    /// (upstream `setRNGSeed(0)`), carrying `effector`.
    fn spacecraft_with(effector: MeanRevertingNoiseStateEffector, step_nanos: u64) -> Spacecraft {
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
        spacecraft
    }

    fn state_value(spacecraft: &Spacecraft) -> f64 {
        spacecraft.state_effectors[0]
            .as_any()
            .downcast_ref::<MeanRevertingNoiseStateEffector>()
            .expect("the first effector is the noise state")
            .state_value()
    }

    /// Upstream `test_meanRevertingNoiseStateEffector`: `1 + x` sampled every
    /// 2 ms for 100 s has mean 1, variance `σ_st²`, lag-one autocorrelation
    /// `exp(-dt/τ)` and the time constant `τ` it implies.
    #[test]
    fn stationary_statistics_match_the_process() {
        let dt: f64 = 0.002;
        let tf = 100.0;
        let sigma_stationary: f64 = 0.8;
        let time_constant = 0.1;
        let step_nanos = 2_000_000;

        let effector =
            MeanRevertingNoiseStateEffector::new(MeanRevertingNoiseStateEffectorConfig {
                stationary_std: sigma_stationary,
                time_constant_s: time_constant,
                ..MeanRevertingNoiseStateEffectorConfig::new("meanRevertingNoiseState")
            });
        assert_eq!(effector.config.time_constant_s, time_constant);
        assert_eq!(effector.config.stationary_std, sigma_stationary);
        let mut spacecraft = spacecraft_with(effector, step_nanos);
        spacecraft.init();

        let steps = (tf / dt) as usize;
        let epoch = Epoch::from_gregorian_utc_at_midnight(2026, 1, 1);
        let y: Vec<f64> = (1..=steps as u64)
            .map(|step| {
                spacecraft.update(&SimulationContext {
                    current_sim_nanos: step * step_nanos,
                    current_epoch: epoch,
                });
                1.0 + state_value(&spacecraft)
            })
            .collect();

        let n = y.len() as f64;
        let mean = y.iter().sum::<f64>() / n;
        let variance = y.iter().map(|value| (value - mean).powi(2)).sum::<f64>() / (n - 1.0);
        let (lagged, centered): (f64, f64) =
            y.windows(2).fold((0.0, 0.0), |(lagged, centered), pair| {
                let (first, second) = (pair[0] - mean, pair[1] - mean);
                (lagged + first * second, centered + first * first)
            });
        let phi = lagged / centered;
        let phi_target = (-dt / time_constant).exp();
        let time_constant_estimate = -dt / phi.clamp(1.0e-6, 0.999_999).ln();

        assert!((mean - 1.0).abs() <= 0.2, "mean {mean}");
        assert!(
            (variance - sigma_stationary.powi(2)).abs() <= 0.3 * sigma_stationary.powi(2),
            "variance {variance}"
        );
        assert!(
            (phi - phi_target).abs() <= 0.003 + 0.03 * phi_target,
            "autocorrelation {phi}, target {phi_target}"
        );
        assert!(
            (time_constant_estimate - time_constant).abs() <= 0.1 + 0.05 * time_constant,
            "time constant {time_constant_estimate}"
        );
    }

    #[test]
    fn drift_and_diffusion_follow_the_parameters() {
        let effector =
            MeanRevertingNoiseStateEffector::new(MeanRevertingNoiseStateEffectorConfig {
                stationary_std: 0.8,
                time_constant_s: 0.5,
                initial_value: 0.25,
                ..MeanRevertingNoiseStateEffectorConfig::new("noise")
            });
        assert_eq!(effector.state_value(), 0.25);
        assert_eq!(effector.initial_state(), [0.25]);
        let drift = effector.compute_derivatives(&[0.3], Vector3::zeros(), Vector3::zeros());
        assert_eq!(drift, [-0.3 / 0.5]);
        assert_eq!(effector.noise_source_count(), 1);
        let diffusion = effector.compute_diffusion(&[123.0]);
        assert_eq!(diffusion, [[0.8 * (2.0_f64.sqrt() / 0.5_f64.sqrt())]]);
        assert!((diffusion[0][0] - 0.8 * 2.0_f64.sqrt() * 2.0_f64.sqrt()).abs() < 1.0e-15);
    }

    /// Upstream's setters reject what `validate` reports.
    #[test]
    fn validation_rejects_bad_parameters() {
        let valid = MeanRevertingNoiseStateEffectorConfig::new("noise");
        valid.validate().unwrap();
        for config in [
            MeanRevertingNoiseStateEffectorConfig {
                stationary_std: -0.01,
                ..valid.clone()
            },
            MeanRevertingNoiseStateEffectorConfig {
                stationary_std: f64::NAN,
                ..valid.clone()
            },
            MeanRevertingNoiseStateEffectorConfig {
                time_constant_s: 0.0,
                ..valid.clone()
            },
            MeanRevertingNoiseStateEffectorConfig {
                time_constant_s: f64::INFINITY,
                ..valid.clone()
            },
            MeanRevertingNoiseStateEffectorConfig {
                initial_value: f64::NAN,
                ..valid.clone()
            },
        ] {
            assert!(config.validate().is_err(), "{config:?}");
        }
    }

    #[test]
    #[should_panic(expected = "invalid MeanRevertingNoiseStateEffectorConfig")]
    fn new_panics_on_an_invalid_config() {
        MeanRevertingNoiseStateEffector::new(MeanRevertingNoiseStateEffectorConfig {
            time_constant_s: -1.0,
            ..MeanRevertingNoiseStateEffectorConfig::new("noise")
        });
    }
}
