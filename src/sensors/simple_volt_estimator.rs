//! Noisy spacecraft-potential estimate: port of upstream Basilisk
//! `simpleVoltEstimator`.
//!
//! Reads the spacecraft electric potential (upstream `voltInMsg`, a
//! [`VoltMsg`], as published by
//! [`crate::environment::spacecraft_charging_equilibrium`]) and republishes it
//! with a one-state bounded Gauss-Markov error added, the way
//! [`super::navigation`] stands in for a navigation filter. With the default
//! configuration (zero noise) the output is the truth.
//!
//! The error model is the crate's [`GaussMarkov`], upstream's: `p_matrix_sqrt_v`
//! is the 1-sigma error step and the walk is clamped at `walk_bounds_v`, see
//! [`crate::gauss_markov`].
//!
//! As upstream, a reset restarts the random stream from the module's seed and
//! zeroes the error state, so a reset run repeats its realization. The seed
//! comes from the module name, like the other sensors in this crate, and
//! [`SimpleVoltEstimator::set_rng_seed`] replaces it (upstream `RNGSeed`).
//! The stream is not upstream's `std::minstd_rand`, so outputs match upstream
//! statistically, not sample for sample.
//!
//! Upstream's sizes are checked at run time (a wrong-sized `PMatrix` or
//! `walkBounds` raises on reset); here the one-state model makes them scalars.

use nalgebra::{Matrix1, Vector1};
use rand::SeedableRng;
use rand::rngs::StdRng;

use crate::gauss_markov::GaussMarkov;
use crate::messages::{Input, Output, VoltMsg};
use crate::sensors::seed_from_name;
use crate::{Module, SimulationContext};

/// Error parameters of [`SimpleVoltEstimator`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct SimpleVoltEstimatorConfig {
    /// Module name (upstream `ModelTag`); seeds the noise stream.
    pub name: String,
    /// Upstream `PMatrix` \[V\]: square root of the error covariance, the
    /// 1-sigma error step. Zero disables the noise.
    pub p_matrix_sqrt_v: f64,
    /// Upstream `AMatrix`: error propagation coefficient; one (upstream's
    /// identity default) is a random walk, zero independent noise.
    pub a_matrix: f64,
    /// Upstream `walkBounds` \[V\]: bound the error is clamped at; a
    /// non-positive value leaves it unbounded.
    pub walk_bounds_v: f64,
}

impl Default for SimpleVoltEstimatorConfig {
    /// Upstream's constructor: no noise, a random-walk propagation, no bound.
    fn default() -> Self {
        Self {
            name: String::new(),
            p_matrix_sqrt_v: 0.0,
            a_matrix: 1.0,
            walk_bounds_v: 0.0,
        }
    }
}

impl SimpleVoltEstimatorConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("p_matrix_sqrt_v", self.p_matrix_sqrt_v),
            ("a_matrix", self.a_matrix),
            ("walk_bounds_v", self.walk_bounds_v),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite, got {value}"));
            }
        }
        Ok(())
    }

    fn error_model(&self) -> GaussMarkov<1> {
        GaussMarkov::new(
            Matrix1::new(self.a_matrix),
            Matrix1::new(self.p_matrix_sqrt_v),
            Vector1::new(self.walk_bounds_v),
        )
    }
}

/// Spacecraft potential with Gauss-Markov errors: upstream
/// `SimpleVoltEstimator`.
#[derive(Clone, Debug)]
pub struct SimpleVoltEstimator {
    /// Error parameters, read on every update so an override applies at once.
    pub config: Output<SimpleVoltEstimatorConfig>,
    /// Upstream `voltInMsg`: true spacecraft potential.
    pub volt_in_msg: Input<VoltMsg>,
    /// Upstream `voltOutMsg`: potential with errors.
    pub volt_out_msg: Output<VoltMsg>,
    error_model: GaussMarkov<1>,
    seed: u64,
    rng: StdRng,
}

impl SimpleVoltEstimator {
    /// # Panics
    ///
    /// If `config` fails [`SimpleVoltEstimatorConfig::validate`].
    pub fn new(config: SimpleVoltEstimatorConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid SimpleVoltEstimatorConfig: {msg}");
        }
        let seed = seed_from_name(&config.name);
        Self {
            error_model: config.error_model(),
            seed,
            rng: StdRng::seed_from_u64(seed),
            config: Output::new(config),
            volt_in_msg: Input::default(),
            volt_out_msg: Output::default(),
        }
    }

    /// Replace the seed (upstream `RNGSeed`) and restart the stream from it
    /// with a zero error state.
    pub fn set_rng_seed(&mut self, seed: u64) {
        self.seed = seed;
        self.restart_error_model();
    }

    /// Upstream `voltErrors` \[V\]: the error added at the last update.
    pub fn voltage_error_v(&self) -> f64 {
        self.error_model.current_state()[0]
    }

    fn restart_error_model(&mut self) {
        self.error_model = self.config.read().error_model();
        self.rng = StdRng::seed_from_u64(self.seed);
    }

    /// Upstream `computeErrors`: advance the error model with the current
    /// parameters.
    fn compute_error_v(&mut self) -> f64 {
        let config = self.config.read();
        // Assigned rather than rebuilt: the accumulated walk is private state
        // a rebuild would discard.
        self.error_model.prop_matrix = Matrix1::new(config.a_matrix);
        self.error_model.noise_matrix_sqrt = Matrix1::new(config.p_matrix_sqrt_v);
        self.error_model.state_bounds = Vector1::new(config.walk_bounds_v);
        self.error_model.compute_next_state(&mut self.rng)[0]
    }
}

impl Module for SimpleVoltEstimator {
    fn init(&mut self) {
        if !self.volt_in_msg.is_connected() {
            log::error!(
                "SimpleVoltEstimator '{}' volt_in_msg was not linked",
                self.config.read().name
            );
        }
        self.restart_error_model();
        self.volt_out_msg.write(VoltMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        if !self.volt_in_msg.is_connected() {
            log::error!(
                "SimpleVoltEstimator '{}' volt_in_msg was not linked",
                self.config.read().name
            );
        }
        self.restart_error_model();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let truth = self.volt_in_msg.read();
        let error_v = self.compute_error_v();
        self.volt_out_msg.write(VoltMsg {
            voltage_v: truth.voltage_v + error_v,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `simpleVoltEstimator/_UnitTest/test_simpleVoltEstimator.py`
    //! with its parameters and thresholds. Its last case, a 12×12 `PMatrix`
    //! that must raise on reset, cannot be written here: the one-state model
    //! takes scalars, so a wrong size does not compile.
    //!
    //! The statistical test is seeded with upstream's default `RNGSeed`
    //! (`0x1badcad1`) rather than the name hash, so it does not depend on the
    //! standard library's hasher. Its thresholds were written for upstream's
    //! hard-clamped `GaussMarkov`, which this crate's model is.

    use hifitime::Epoch;

    use crate::messages::{Output, VoltMsg};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    use super::{SimpleVoltEstimator, SimpleVoltEstimatorConfig};

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: epoch(),
        }
    }

    /// Upstream `test_unitSimpleVoltEstimator`: a -2000 V potential with a
    /// 50 V noise and a 1000 V walk bound, sampled every 0.1 s for 144 min.
    /// The error exceeds the bound in at most 0.3 % of the samples, and
    /// exceeds 80 % of it at least once.
    #[test]
    fn walk_stays_within_its_bound_and_uses_it() {
        const STEP_NANOS: u64 = 100_000_000;
        let sc_potential_v = -2000.0;
        let volt_bound_v = 1000.0;
        let mut estimator = SimpleVoltEstimator::new(SimpleVoltEstimatorConfig {
            name: "SimpleVoltageEstimation".to_string(),
            p_matrix_sqrt_v: 50.0,
            walk_bounds_v: 1000.0,
            ..Default::default()
        });
        let truth = Output::new(VoltMsg {
            voltage_v: sc_potential_v,
        });
        truth.connect_to(&mut estimator.volt_in_msg);
        // Upstream's default `SysModel::RNGSeed`.
        estimator.set_rng_seed(0x1bad_cad1);

        estimator.init();
        let mut volts = Vec::new();
        let mut nanos = 0;
        while nanos <= 8_640_000_000_000 {
            estimator.update(&context(nanos));
            volts.push(estimator.volt_out_msg.read().voltage_v);
            nanos += STEP_NANOS;
        }
        assert_eq!(volts.len(), 86_401);

        let count_allow = volts.len() as f64 * 0.3 / 100.0;
        let exceed = volts
            .iter()
            .filter(|volt| (*volt - sc_potential_v).abs() > volt_bound_v)
            .count();
        assert!(
            exceed as f64 <= count_allow,
            "too many error counts: {exceed} > {count_allow}"
        );
        let near = volts
            .iter()
            .filter(|volt| (*volt - sc_potential_v).abs() > volt_bound_v * 0.8)
            .count();
        assert!(near >= 1, "too few error counts: {near}");
    }

    /// The default configuration adds no error.
    #[test]
    fn default_configuration_passes_the_truth_through() {
        let mut estimator = SimpleVoltEstimator::new(SimpleVoltEstimatorConfig {
            name: "perfect".to_string(),
            ..Default::default()
        });
        let truth = Output::new(VoltMsg { voltage_v: 12.5 });
        {
            let mut simulation = Simulation::new(epoch(), false);
            simulation.connect(&truth, &mut estimator.volt_in_msg);
            simulation.add_module("volt", &mut estimator, 1_000_000_000, 0);
            simulation.run_for(5_000_000_000);
        }
        assert_eq!(estimator.volt_out_msg.read().voltage_v, 12.5);
        assert_eq!(estimator.voltage_error_v(), 0.0);
    }

    /// A reset restarts the stream: the realization repeats.
    #[test]
    fn reset_repeats_the_realization() {
        let mut estimator = SimpleVoltEstimator::new(SimpleVoltEstimatorConfig {
            name: "repeat".to_string(),
            p_matrix_sqrt_v: 3.0,
            walk_bounds_v: 100.0,
            ..Default::default()
        });
        let truth = Output::new(VoltMsg { voltage_v: 1.0 });
        truth.connect_to(&mut estimator.volt_in_msg);

        let run = |estimator: &mut SimpleVoltEstimator| -> Vec<f64> {
            (0..20)
                .map(|step| {
                    estimator.update(&context(step * 1_000_000));
                    estimator.volt_out_msg.read().voltage_v
                })
                .collect()
        };
        estimator.init();
        let first = run(&mut estimator);
        Module::reset(&mut estimator, &context(0));
        let second = run(&mut estimator);
        assert_eq!(first, second);
        assert!(first.iter().any(|volt| *volt != 1.0));

        estimator.set_rng_seed(17);
        let reseeded = run(&mut estimator);
        assert_ne!(reseeded, first);
    }

    /// With zero propagation the errors are independent with standard
    /// deviation `p_matrix_sqrt_v`.
    #[test]
    fn independent_errors_have_the_noise_entry_as_their_deviation() {
        let mut estimator = SimpleVoltEstimator::new(SimpleVoltEstimatorConfig {
            name: "iid".to_string(),
            p_matrix_sqrt_v: 2.0,
            a_matrix: 0.0,
            walk_bounds_v: 0.0,
        });
        let truth = Output::new(VoltMsg { voltage_v: 0.0 });
        truth.connect_to(&mut estimator.volt_in_msg);
        estimator.init();
        let n = 40_000;
        let sum_sq: f64 = (0..n)
            .map(|step| {
                estimator.update(&context(step));
                estimator.volt_out_msg.read().voltage_v.powi(2)
            })
            .sum();
        let std = (sum_sq / n as f64).sqrt();
        assert!((std - 2.0).abs() < 0.05, "std {std}");
    }

    #[test]
    #[should_panic(expected = "walk_bounds_v")]
    fn rejects_a_non_finite_bound() {
        let _ = SimpleVoltEstimator::new(SimpleVoltEstimatorConfig {
            walk_bounds_v: f64::NAN,
            ..Default::default()
        });
    }
}
