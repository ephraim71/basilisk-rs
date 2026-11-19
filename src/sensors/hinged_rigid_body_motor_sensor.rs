//! Hinged-panel angle sensor: port of upstream Basilisk
//! `hingedRigidBodyMotorSensor`.
//!
//! Reads the true panel state of a hinged rigid body (upstream
//! `hingedRigidBodyMotorSensorInMsg`, a [`HingedRigidBodyMsg`]) and publishes
//! the sensed state on the same message type. The error chain on each of
//! `theta` and `thetaDot` is, in order:
//!
//! 1. Gaussian noise `std * N(0, 1)`. Both channels draw a sample on every
//!    update, even with a zero standard deviation, so disabling one channel
//!    leaves the other's random sequence unchanged (as upstream).
//! 2. A constant bias.
//! 3. Discretization to the nearest multiple of the least significant bit,
//!    when that LSB is positive: the magnitude is floored to a whole number of
//!    LSBs and rounded away from zero when the remainder exceeds half an LSB,
//!    so an exact half rounds toward zero.
//!
//! With the default configuration the sensor is perfect and passes the truth
//! through.
//!
//! The random stream is seeded from the module name (as the other sensors in
//! this crate) and restarts only on [`HingedRigidBodyMotorSensor::set_rng_seed`]
//! (upstream `setRNGSeed`); a reset does not reseed, as upstream's does not.
//! The stream is not upstream's `std::minstd_rand`, so noisy outputs match
//! upstream statistically, not sample for sample.

use rand::SeedableRng;
use rand::rngs::StdRng;
use rand_distr::{Distribution, StandardNormal};

use crate::messages::{HingedRigidBodyMsg, Input, Output};
use crate::sensors::seed_from_name;
use crate::{Module, SimulationContext};

/// Error parameters of [`HingedRigidBodyMotorSensor`].
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct HingedRigidBodyMotorSensorConfig {
    /// Module name (upstream `ModelTag`); seeds the noise stream.
    pub name: String,
    /// Upstream `thetaNoiseStd` \[rad\]: standard deviation of the angle noise.
    pub theta_noise_std_rad: f64,
    /// Upstream `thetaDotNoiseStd` \[rad/s\]: standard deviation of the rate
    /// noise.
    pub theta_dot_noise_std_radps: f64,
    /// Upstream `thetaBias` \[rad\]: bias added to the true angle.
    pub theta_bias_rad: f64,
    /// Upstream `thetaDotBias` \[rad/s\]: bias added to the true rate.
    pub theta_dot_bias_radps: f64,
    /// Upstream `thetaLSB` \[rad\]: angle resolution; a non-positive value
    /// (upstream's default `-1`) disables the discretization.
    pub theta_lsb_rad: f64,
    /// Upstream `thetaDotLSB` \[rad/s\]: rate resolution; a non-positive value
    /// disables the discretization.
    pub theta_dot_lsb_radps: f64,
}

impl Default for HingedRigidBodyMotorSensorConfig {
    /// Upstream's constructor: no noise, no bias, no discretization.
    fn default() -> Self {
        Self {
            name: String::new(),
            theta_noise_std_rad: 0.0,
            theta_dot_noise_std_radps: 0.0,
            theta_bias_rad: 0.0,
            theta_dot_bias_radps: 0.0,
            theta_lsb_rad: -1.0,
            theta_dot_lsb_radps: -1.0,
        }
    }
}

impl HingedRigidBodyMotorSensorConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("theta_noise_std_rad", self.theta_noise_std_rad),
            ("theta_dot_noise_std_radps", self.theta_dot_noise_std_radps),
            ("theta_bias_rad", self.theta_bias_rad),
            ("theta_dot_bias_radps", self.theta_dot_bias_radps),
            ("theta_lsb_rad", self.theta_lsb_rad),
            ("theta_dot_lsb_radps", self.theta_dot_lsb_radps),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite, got {value}"));
            }
        }
        for (label, value) in [
            ("theta_noise_std_rad", self.theta_noise_std_rad),
            ("theta_dot_noise_std_radps", self.theta_dot_noise_std_radps),
        ] {
            if value < 0.0 {
                return Err(format!("{label} must be non-negative, got {value}"));
            }
        }
        Ok(())
    }
}

/// Sensed hinged-panel angle and rate: upstream `HingedRigidBodyMotorSensor`.
#[derive(Clone, Debug)]
pub struct HingedRigidBodyMotorSensor {
    /// Error parameters, read on every update so an override applies at once.
    pub config: Output<HingedRigidBodyMotorSensorConfig>,
    /// Upstream `hingedRigidBodyMotorSensorInMsg`: true panel state.
    pub hinged_rigid_body_motor_sensor_in_msg: Input<HingedRigidBodyMsg>,
    /// Upstream `hingedRigidBodyMotorSensorOutMsg`: sensed panel state.
    pub hinged_rigid_body_motor_sensor_out_msg: Output<HingedRigidBodyMsg>,
    rng: StdRng,
}

impl HingedRigidBodyMotorSensor {
    /// # Panics
    ///
    /// If `config` fails [`HingedRigidBodyMotorSensorConfig::validate`].
    pub fn new(config: HingedRigidBodyMotorSensorConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid HingedRigidBodyMotorSensorConfig: {msg}");
        }
        Self {
            rng: StdRng::seed_from_u64(seed_from_name(&config.name)),
            config: Output::new(config),
            hinged_rigid_body_motor_sensor_in_msg: Input::default(),
            hinged_rigid_body_motor_sensor_out_msg: Output::default(),
        }
    }

    /// Restart the noise stream from `seed` (upstream `setRNGSeed`).
    pub fn set_rng_seed(&mut self, seed: u64) {
        self.rng = StdRng::seed_from_u64(seed);
    }

    fn sense(&mut self, truth: &HingedRigidBodyMsg) -> HingedRigidBodyMsg {
        let config = self.config.read();
        // Scaled standard-normal samples: a zero standard deviation stays
        // valid and still consumes its sample.
        let theta_sample: f64 = StandardNormal.sample(&mut self.rng);
        let theta_dot_sample: f64 = StandardNormal.sample(&mut self.rng);
        let theta =
            truth.theta_rad + config.theta_noise_std_rad * theta_sample + config.theta_bias_rad;
        let theta_dot = truth.theta_dot_radps
            + config.theta_dot_noise_std_radps * theta_dot_sample
            + config.theta_dot_bias_radps;

        HingedRigidBodyMsg {
            theta_rad: discretize(theta, config.theta_lsb_rad),
            theta_dot_radps: discretize(theta_dot, config.theta_dot_lsb_radps),
        }
    }
}

/// Upstream's rounding of `value` to a multiple of `lsb`; a non-positive
/// `lsb` returns `value` unchanged.
fn discretize(value: f64, lsb: f64) -> f64 {
    if lsb <= 0.0 {
        return value;
    }
    let sign = 1.0_f64.copysign(value);
    let mut working = (value.abs() / lsb).floor() * lsb * sign;
    let remainder = value - working;
    if remainder.abs() > lsb / 2.0 {
        working += lsb * sign;
    }
    working
}

impl Module for HingedRigidBodyMotorSensor {
    fn init(&mut self) {
        if !self.hinged_rigid_body_motor_sensor_in_msg.is_connected() {
            log::error!(
                "HingedRigidBodyMotorSensor '{}' hinged_rigid_body_motor_sensor_in_msg was not linked",
                self.config.read().name
            );
        }
        self.hinged_rigid_body_motor_sensor_out_msg
            .write(HingedRigidBodyMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        if !self.hinged_rigid_body_motor_sensor_in_msg.is_connected() {
            log::error!(
                "HingedRigidBodyMotorSensor '{}' hinged_rigid_body_motor_sensor_in_msg was not linked",
                self.config.read().name
            );
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        let truth = self.hinged_rigid_body_motor_sensor_in_msg.read();
        let sensed = self.sense(&truth);
        self.hinged_rigid_body_motor_sensor_out_msg.write(sensed);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `hingedRigidBodyMotorSensor/_UnitTest/test_hingedRigidBodyMotorSensor.py`:
    //! both tests, with every parameter combination, the same run length and
    //! the same tolerances. The random stream is not upstream's, so the noise
    //! test compares against this module's own unit-variance samples, as
    //! upstream compares against its own.

    use hifitime::Epoch;

    use crate::messages::{HingedRigidBodyMsg, Output};
    use crate::simulation::Simulation;
    use crate::{Module, SimulationContext};

    use super::{HingedRigidBodyMotorSensor, HingedRigidBodyMotorSensorConfig, discretize};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `test_noise_scales_standard_samples_with_zero_channels`: with
    /// a zero truth, the output of any standard-deviation pair is the
    /// unit-variance output scaled by that pair, so a disabled channel neither
    /// adds noise nor shifts the other channel's samples.
    #[test]
    fn noise_scales_standard_samples_with_zero_channels() {
        fn sample(theta_std: f64, theta_dot_std: f64) -> Vec<[f64; 2]> {
            let mut module = HingedRigidBodyMotorSensor::new(HingedRigidBodyMotorSensorConfig {
                name: "motor_sensor".to_string(),
                theta_noise_std_rad: theta_std,
                theta_dot_noise_std_radps: theta_dot_std,
                ..Default::default()
            });
            module.set_rng_seed(3);
            let input = Output::new(HingedRigidBodyMsg::default());
            input.connect_to(&mut module.hinged_rigid_body_motor_sensor_in_msg);
            module.init();
            (0..6)
                .map(|nanos| {
                    module.update(&context(nanos));
                    let output = module.hinged_rigid_body_motor_sensor_out_msg.read();
                    [output.theta_rad, output.theta_dot_radps]
                })
                .collect()
        }

        let standard = sample(1.0, 1.0);
        assert!(standard.iter().flatten().any(|value| *value != 0.0));
        for (theta_std, theta_dot_std) in [(0.0, 0.0), (0.0, 0.1), (0.2, 0.0), (0.2, 0.1)] {
            let scaled = sample(theta_std, theta_dot_std);
            for (actual, unit) in scaled.iter().zip(&standard) {
                for (value, (unit_value, std)) in actual
                    .iter()
                    .zip(unit.iter().zip([theta_std, theta_dot_std]))
                {
                    let expected = unit_value * std;
                    // `assert_allclose(rtol=1e-14, atol=0)`.
                    assert!(
                        (value - expected).abs() <= 1.0e-14 * expected.abs(),
                        "std ({theta_std}, {theta_dot_std}): {value} vs {expected}"
                    );
                }
            }
        }
    }

    /// Upstream `test_hingedRigidBodyMotorSensor`: every bias, LSB and truth
    /// combination, with noise off, run for 10 s at 0.5 s; the last output
    /// equals the biased truth rounded to the nearest LSB (Python's `round`)
    /// within 1e-12.
    #[test]
    fn bias_and_discretization_match_the_rounded_truth() {
        const STEP_NANOS: u64 = 500_000_000;
        let accuracy = 1.0e-12;
        for (theta_bias, theta_dot_bias) in [(0.0, 0.0), (-0.1, 0.1), (0.2, -0.01)] {
            for (theta_lsb, theta_dot_lsb) in [(-1.0, -1.0), (0.1, 0.05), (0.01, 0.005)] {
                for (true_theta, true_theta_dot) in [(1.01, -0.23), (-0.229, 0.112)] {
                    let mut module =
                        HingedRigidBodyMotorSensor::new(HingedRigidBodyMotorSensorConfig {
                            name: "hingedRigidBodyMotorSensorTag".to_string(),
                            theta_bias_rad: theta_bias,
                            theta_dot_bias_radps: theta_dot_bias,
                            theta_lsb_rad: theta_lsb,
                            theta_dot_lsb_radps: theta_dot_lsb,
                            ..Default::default()
                        });
                    module.set_rng_seed(2);
                    let truth = Output::new(HingedRigidBodyMsg {
                        theta_rad: true_theta,
                        theta_dot_radps: true_theta_dot,
                    });
                    {
                        let mut simulation = Simulation::new(context(0).current_epoch, false);
                        simulation
                            .connect(&truth, &mut module.hinged_rigid_body_motor_sensor_in_msg);
                        simulation.add_module("motor_sensor", &mut module, STEP_NANOS, 0);
                        simulation.run_for(10_000_000_000);
                    }

                    let round_to = |value: f64, lsb: f64| {
                        if lsb > 0.0 {
                            python_round(value / lsb) * lsb
                        } else {
                            value
                        }
                    };
                    let disc_theta = round_to(true_theta + theta_bias, theta_lsb);
                    let disc_theta_dot = round_to(true_theta_dot + theta_dot_bias, theta_dot_lsb);
                    let sensed = module.hinged_rigid_body_motor_sensor_out_msg.read();
                    let case = format!(
                        "bias ({theta_bias}, {theta_dot_bias}), lsb ({theta_lsb}, \
                         {theta_dot_lsb}), truth ({true_theta}, {true_theta_dot})"
                    );
                    assert!(
                        (sensed.theta_rad - disc_theta).abs() < accuracy,
                        "theta {} vs {disc_theta}: {case}",
                        sensed.theta_rad
                    );
                    assert!(
                        (sensed.theta_dot_radps - disc_theta_dot).abs() < accuracy,
                        "thetaDot {} vs {disc_theta_dot}: {case}",
                        sensed.theta_dot_radps
                    );
                }
            }
        }
    }

    /// Python 3 `round`: to the nearest integer, ties to even.
    fn python_round(value: f64) -> f64 {
        let rounded = value.round();
        if (value - value.trunc()).abs() == 0.5 && rounded % 2.0 != 0.0 {
            rounded - value.signum()
        } else {
            rounded
        }
    }

    /// Upstream's branches: floor to whole LSBs, then one more LSB away from
    /// zero only when the remainder exceeds half an LSB; an exact half rounds
    /// toward zero, and the sign is kept for negative values.
    #[test]
    fn discretization_rounds_half_toward_zero_and_keeps_the_sign() {
        assert_eq!(discretize(0.3, 0.25), 0.25);
        assert_eq!(discretize(0.4, 0.25), 0.5);
        assert_eq!(discretize(-0.4, 0.25), -0.5);
        assert_eq!(discretize(0.375, 0.25), 0.25, "an exact half rounds down");
        assert_eq!(discretize(-0.375, 0.25), -0.25);
        assert_eq!(discretize(0.0, 0.25), 0.0);
        assert_eq!(discretize(0.123, 0.0), 0.123, "a zero LSB disables it");
        assert_eq!(discretize(0.123, -1.0), 0.123);
    }

    /// The default configuration is a perfect sensor.
    #[test]
    fn default_configuration_passes_the_truth_through() {
        let mut module = HingedRigidBodyMotorSensor::new(HingedRigidBodyMotorSensorConfig {
            name: "perfect".to_string(),
            ..Default::default()
        });
        let truth = HingedRigidBodyMsg {
            theta_rad: 0.731,
            theta_dot_radps: -0.0123,
        };
        let input = Output::new(truth.clone());
        input.connect_to(&mut module.hinged_rigid_body_motor_sensor_in_msg);
        module.init();
        module.update(&context(0));
        let sensed = module.hinged_rigid_body_motor_sensor_out_msg.read();
        assert_eq!(sensed.theta_rad, truth.theta_rad);
        assert_eq!(sensed.theta_dot_radps, truth.theta_dot_radps);
    }

    /// A configuration override takes effect on the next update.
    #[test]
    fn a_bias_override_applies_on_the_next_update() {
        use crate::overrides::Request;
        use serde_json::json;

        let mut module = HingedRigidBodyMotorSensor::new(HingedRigidBodyMotorSensorConfig {
            name: "overridden".to_string(),
            ..Default::default()
        });
        let input = Output::new(HingedRigidBodyMsg {
            theta_rad: 1.0,
            theta_dot_radps: 0.0,
        });
        input.connect_to(&mut module.hinged_rigid_body_motor_sensor_in_msg);
        module.init();
        module
            .config
            .set_override(Request::replace(json!({ "/theta_bias_rad": 0.5 })).unwrap())
            .unwrap();
        module.update(&context(0));
        assert_eq!(
            module
                .hinged_rigid_body_motor_sensor_out_msg
                .read()
                .theta_rad,
            1.5
        );
    }

    #[test]
    #[should_panic(expected = "theta_noise_std_rad")]
    fn rejects_a_negative_noise_standard_deviation() {
        let _ = HingedRigidBodyMotorSensor::new(HingedRigidBodyMotorSensorConfig {
            theta_noise_std_rad: -0.1,
            ..Default::default()
        });
    }
}
