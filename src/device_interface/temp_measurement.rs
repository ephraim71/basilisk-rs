//! Temperature sensor with noise, bias and faults: port of upstream Basilisk
//! `simulation/deviceInterface/tempMeasurement`.
//!
//! Reads a true temperature (for example a thermal model's output) and
//! publishes the sensed one,
//!
//! ```text
//! T_sensed = T_true + bias + w
//! ```
//!
//! where `w` is a one-state Gauss-Markov error with standard deviation
//! `sen_noise_std_c` per step, propagation coefficient `a_matrix` (zero, the
//! default, is white noise; one a random walk) and the hard bound
//! `walk_bounds_c` (non-positive for none). With `sen_noise_std_c <= 0` the
//! error is the bias alone and no noise is drawn. A [`TempFaultState`] then
//! replaces the reading: stuck at the configured value, stuck at the last
//! reading, or multiplied by `spike_amount` with probability
//! `spike_probability` each update.
//!
//! The error model is the crate's [`GaussMarkov`], which, like upstream's,
//! takes `sen_noise_std_c` as its 1-sigma step and clamps the walk at
//! `walk_bounds_c`. The noise
//! and spike draws use two streams seeded from the module's seed, which
//! comes from the module name like the other sensors in this crate and is
//! replaced with [`TempMeasurement::set_rng_seed`] (upstream `RNGSeed`); the
//! spike stream's seed is the noise seed XOR upstream's secondary-stream
//! discriminator. The streams are not upstream's `std::minstd_rand`, so the
//! output matches upstream statistically, not sample for sample. A reset
//! restarts both streams and zeroes the error state.

use nalgebra::{Matrix1, Vector1};
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use serde::{Deserialize, Serialize};

use crate::gauss_markov::GaussMarkov;
use crate::messages::{Input, Output, TemperatureMsg};
use crate::sensors::seed_from_name;
use crate::{Module, SimulationContext};

/// Upstream `GaussMarkov::deriveSecondarySeed`'s stream discriminator.
const SECONDARY_STREAM_DISCRIMINATOR: u64 = 0x9E37_79B9_7F4A_7C15;

/// Fault state of the sensor: upstream `TempFaultState_t`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum TempFaultState {
    /// Upstream `TEMP_FAULT_NOMINAL = 0`.
    #[default]
    Nominal = 0,
    /// Upstream `TEMP_FAULT_STUCK_CURRENT = 1`: repeats the last reading.
    StuckCurrent = 1,
    /// Upstream `TEMP_FAULT_STUCK_VALUE = 2`: reads `stuck_value_c`.
    StuckValue = 2,
    /// Upstream `TEMP_FAULT_SPIKING = 3`: multiplies the reading by
    /// `spike_amount` with probability `spike_probability`.
    Spiking = 3,
    /// Upstream `TEMP_FAULT_BIASED = 4`. Upstream gives it no behavior of its
    /// own (the bias applies in every state), so it reads as nominal.
    Biased = 4,
    /// Upstream `TEMP_FAULT_GAUSS_MARKOV = 5`. As [`Self::Biased`], it reads
    /// as nominal: the noise applies in every state.
    GaussMarkov = 5,
}

/// Error and fault parameters of [`TempMeasurement`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TempMeasurementConfig {
    /// Module name (upstream `ModelTag`); seeds the noise streams.
    pub name: String,
    /// Upstream `faultState`.
    pub fault_state: TempFaultState,
    /// Upstream `senBias` \[°C\]: constant error.
    pub sen_bias_c: f64,
    /// Upstream `senNoiseStd` \[°C\]: standard deviation of the noise step;
    /// non-positive disables the noise.
    pub sen_noise_std_c: f64,
    /// Upstream `walkBounds` \[°C\]: hard bound on the noise; non-positive
    /// leaves it unbounded.
    pub walk_bounds_c: f64,
    /// Upstream `stuckValue` \[°C\]: reading of [`TempFaultState::StuckValue`].
    pub stuck_value_c: f64,
    /// Upstream `spikeProbability` \[-\]: chance of a spike per update, in
    /// `[0, 1]`.
    pub spike_probability: f64,
    /// Upstream `spikeAmount` \[-\]: factor applied by a spike.
    pub spike_amount: f64,
    /// Upstream `propagationMatrix` (`setAMatrix`) \[-\]: noise propagation
    /// coefficient.
    pub a_matrix: f64,
}

impl Default for TempMeasurementConfig {
    /// Upstream's constructor: nominal, no bias or noise, no bound, a 10 %
    /// chance of doubling while spiking, white-noise propagation.
    fn default() -> Self {
        Self {
            name: "tempMeasurement".to_string(),
            fault_state: TempFaultState::Nominal,
            sen_bias_c: 0.0,
            sen_noise_std_c: 0.0,
            walk_bounds_c: -1.0,
            stuck_value_c: 0.0,
            spike_probability: 0.1,
            spike_amount: 2.0,
            a_matrix: 0.0,
        }
    }
}

impl TempMeasurementConfig {
    /// Check numeric invariants, among them upstream `Reset`'s spike
    /// probability range. Returns a description of the first violation, or
    /// `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        for (label, value) in [
            ("sen_bias_c", self.sen_bias_c),
            ("sen_noise_std_c", self.sen_noise_std_c),
            ("walk_bounds_c", self.walk_bounds_c),
            ("stuck_value_c", self.stuck_value_c),
            ("spike_amount", self.spike_amount),
            ("a_matrix", self.a_matrix),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite, got {value}"));
            }
        }
        if !(0.0..=1.0).contains(&self.spike_probability) {
            return Err(format!(
                "the probability of temperature spike on fault must be between 0 and 1, got {}",
                self.spike_probability
            ));
        }
        Ok(())
    }

    fn error_model(&self) -> GaussMarkov<1> {
        GaussMarkov::new(
            Matrix1::new(self.a_matrix),
            Matrix1::new(self.sen_noise_std_c),
            Vector1::new(self.walk_bounds_c),
        )
    }
}

/// Temperature measurement with errors and faults: upstream
/// `TempMeasurement`.
#[derive(Clone, Debug)]
pub struct TempMeasurement {
    /// Error and fault parameters, read on every update so an override (a
    /// fault injection) applies at once.
    pub config: Output<TempMeasurementConfig>,
    /// Upstream `tempInMsg`: true temperature.
    pub temp_in_msg: Input<TemperatureMsg>,
    /// Upstream `tempOutMsg`: sensed temperature.
    pub temp_out_msg: Output<TemperatureMsg>,
    error_model: GaussMarkov<1>,
    seed: u64,
    noise_rng: StdRng,
    spike_rng: StdRng,
    /// Upstream `pastValue` \[°C\]: the last reading.
    past_value_c: f64,
}

impl TempMeasurement {
    /// # Panics
    ///
    /// If `config` fails [`TempMeasurementConfig::validate`].
    pub fn new(config: TempMeasurementConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid TempMeasurementConfig: {message}");
        }
        let seed = seed_from_name(&config.name);
        Self {
            error_model: config.error_model(),
            seed,
            noise_rng: StdRng::seed_from_u64(seed),
            spike_rng: StdRng::seed_from_u64(seed ^ SECONDARY_STREAM_DISCRIMINATOR),
            config: Output::new(config),
            temp_in_msg: Input::default(),
            temp_out_msg: Output::default(),
            past_value_c: 0.0,
        }
    }

    /// Replace the seed (upstream `RNGSeed`) and restart both streams from
    /// it with a zero error state.
    pub fn set_rng_seed(&mut self, seed: u64) {
        self.seed = seed;
        self.restart_error_model();
    }

    /// Upstream `getAMatrix`: the noise propagation coefficient.
    pub fn a_matrix(&self) -> f64 {
        self.config.read().a_matrix
    }

    fn restart_error_model(&mut self) {
        self.error_model = self.config.read().error_model();
        self.noise_rng = StdRng::seed_from_u64(self.seed);
        self.spike_rng = StdRng::seed_from_u64(self.seed ^ SECONDARY_STREAM_DISCRIMINATOR);
    }

    /// Upstream `Reset`.
    fn reset_module(&mut self) {
        let config = self.config.read();
        if !self.temp_in_msg.is_connected() {
            log::error!(
                "tempMeasurement '{}' temp_in_msg is not connected",
                config.name
            );
        }
        if !(0.0..=1.0).contains(&config.spike_probability) {
            log::error!(
                "tempMeasurement '{}': the probability of temperature spike on fault must be \
                 between 0 and 1",
                config.name
            );
        }
        self.restart_error_model();
    }

    /// Upstream `applySensorErrors`: the sensed temperature for
    /// `true_temperature_c`.
    fn apply_sensor_errors(&mut self, true_temperature_c: f64) -> f64 {
        let config = self.config.read();
        let sensor_error_c = if config.sen_noise_std_c <= 0.0 {
            config.sen_bias_c
        } else {
            // Assigned rather than rebuilt: the accumulated walk is private
            // state a rebuild would discard.
            self.error_model.prop_matrix = Matrix1::new(config.a_matrix);
            self.error_model.noise_matrix_sqrt = Matrix1::new(config.sen_noise_std_c);
            self.error_model.state_bounds = Vector1::new(config.walk_bounds_c);
            let noise_c = self.error_model.compute_next_state(&mut self.noise_rng)[0];
            config.sen_bias_c + noise_c
        };
        let mut sensed_c = true_temperature_c + sensor_error_c;

        match config.fault_state {
            TempFaultState::StuckValue => sensed_c = config.stuck_value_c,
            TempFaultState::StuckCurrent => sensed_c = self.past_value_c,
            TempFaultState::Spiking => {
                if self.spike_rng.r#gen::<f64>() <= config.spike_probability {
                    sensed_c *= config.spike_amount;
                }
            }
            TempFaultState::Nominal | TempFaultState::Biased | TempFaultState::GaussMarkov => {}
        }
        self.past_value_c = sensed_c;
        sensed_c
    }
}

impl Module for TempMeasurement {
    fn init(&mut self) {
        self.reset_module();
        self.temp_out_msg.write(TemperatureMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_module();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let truth = self.temp_in_msg.read();
        let temperature_c = self.apply_sensor_errors(truth.temperature_c);
        self.temp_out_msg.write(TemperatureMsg { temperature_c });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `tempMeasurement/_UnitTest/test_tempMeasurement.py`
    //! (each fault, with its truth values and tolerances) and
    //! `test_tempMeasurementNoise.py` (its statistical checks).
    //!
    //! `test_tempMeasurement.py` takes the true temperature from upstream
    //! `sensorThermal`, which basilisk-rs does not have yet; the test's
    //! `SensorThermal` reimplements its forward-Euler heat balance for the
    //! pytest's sensor, as the truth source.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{TempFaultState, TempMeasurement, TempMeasurementConfig};
    use crate::messages::{Output, TemperatureMsg};
    use crate::utilities::astro_constants::AU;
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    const STEP_NANOS: u64 = 1_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `sensorThermal`'s `evaluateThermalModel` for the pytest's
    /// sensor: a 1 m^2 plate facing `+z`, drawing 30 W, at Earth's surface
    /// with the Sun along `+x`.
    struct SensorThermal {
        temperature_c: f64,
        previous_time_s: f64,
        temperature_out: Output<TemperatureMsg>,
    }

    impl SensorThermal {
        const N_HAT_B: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);
        const AREA_M2: f64 = 1.0;
        const ABSORPTIVITY: f64 = 0.25;
        const EMISSIVITY: f64 = 0.34;
        const MASS_KG: f64 = 2.0;
        const SPECIFIC_HEAT: f64 = 890.0;
        const POWER_DRAW_W: f64 = 30.0;
        const SOLAR_CONSTANT_WPM2: f64 = 1366.0;
        const BOLTZMANN: f64 = 5.76051e-8;

        fn new() -> Self {
            Self {
                temperature_c: 0.0,
                previous_time_s: 0.0,
                temperature_out: Output::default(),
            }
        }

        fn update(&mut self, t_s: f64) {
            let r_bn_n = Vector3::new(6378.0 * 1000.0, 0.0, 0.0);
            let r_sn_n = Vector3::new(AU * 1000.0, 0.0, 0.0);
            let s_hat_b = mrp_to_dcm(Vector3::zeros()) * (r_sn_n - r_bn_n).normalize();
            let projected_area = (Self::AREA_M2 * s_hat_b.dot(&Self::N_HAT_B)).max(0.0);
            let q_in = Self::SOLAR_CONSTANT_WPM2 * projected_area * Self::ABSORPTIVITY
                + Self::POWER_DRAW_W;
            let q_out = Self::AREA_M2
                * Self::EMISSIVITY
                * Self::BOLTZMANN
                * (self.temperature_c + 273.15).powf(4.0);
            let rate = (q_in - q_out) / (Self::SPECIFIC_HEAT * Self::MASS_KG);
            self.temperature_c += rate * (t_s - self.previous_time_s);
            self.previous_time_s = t_s;
            self.temperature_out.write(TemperatureMsg {
                temperature_c: self.temperature_c,
            });
        }
    }

    /// The pytest's `run`: 1 s steps through 100 s, `fault` (if any) set
    /// after the update at `fault_after_s`. Returns the sensed log.
    fn run_thermal(
        config: TempMeasurementConfig,
        fault: Option<(u64, TempFaultState)>,
    ) -> Vec<f64> {
        let mut thermal = SensorThermal::new();
        let mut sensor = TempMeasurement::new(config);
        sensor.set_rng_seed(464374481);
        thermal.temperature_out.connect_to(&mut sensor.temp_in_msg);
        sensor.init();
        let mut log = Vec::new();
        for second in 0..=100 {
            let t_nanos = second * STEP_NANOS;
            thermal.update(t_nanos as f64 * 1e-9);
            sensor.update(&context(t_nanos));
            log.push(sensor.temp_out_msg.read().temperature_c);
            if let Some((after_s, state)) = fault
                && second == after_s
            {
                let mut config = sensor.config.read();
                config.fault_state = state;
                sensor.config.write(config);
            }
        }
        log
    }

    fn assert_relative(actual: f64, truth: f64, accuracy: f64, what: &str) {
        assert!(
            (actual - truth).abs() <= accuracy * truth.abs(),
            "{what}: {actual} vs {truth}"
        );
    }

    #[test]
    fn nominal_reads_the_truth() {
        let log = run_thermal(TempMeasurementConfig::default(), None);
        assert_relative(log[100], -4.251289338192501, 1e-12, "nominal");
    }

    #[test]
    fn stuck_current_repeats_the_reading_before_the_fault() {
        let log = run_thermal(
            TempMeasurementConfig::default(),
            Some((50, TempFaultState::StuckCurrent)),
        );
        assert_relative(log[100], -2.1722164230619447, 1e-12, "stuck current");
    }

    #[test]
    fn stuck_value_reads_the_stuck_value() {
        let log = run_thermal(
            TempMeasurementConfig {
                stuck_value_c: 10.0,
                ..Default::default()
            },
            Some((50, TempFaultState::StuckValue)),
        );
        assert_relative(log[100], 10.0, 1e-12, "stuck value");
        assert_relative(log[50], -2.1722164230619447, 1e-12, "before the fault");
    }

    #[test]
    fn biased_adds_the_bias() {
        let log = run_thermal(
            TempMeasurementConfig {
                sen_bias_c: 1.0,
                ..Default::default()
            },
            None,
        );
        assert_relative(log[100], -3.251289338192501, 1e-12, "biased");
    }

    /// The pytest's spiking check: the fraction of readings off the nominal
    /// trace is `spike_probability`, within 50 %.
    #[test]
    fn spiking_departs_from_nominal_at_the_spike_probability() {
        let nominals: [f64; 101] = [
            0.,
            -0.04439869,
            -0.08875756,
            -0.13307667,
            -0.17735608,
            -0.22159584,
            -0.265796,
            -0.30995662,
            -0.35407775,
            -0.39815946,
            -0.44220178,
            -0.48620479,
            -0.53016852,
            -0.57409304,
            -0.6179784,
            -0.66182465,
            -0.70563184,
            -0.74940004,
            -0.79312929,
            -0.83681965,
            -0.88047117,
            -0.9240839,
            -0.9676579,
            -1.01119321,
            -1.0546899,
            -1.09814802,
            -1.14156761,
            -1.18494873,
            -1.22829144,
            -1.27159578,
            -1.31486181,
            -1.35808958,
            -1.40127914,
            -1.44443055,
            -1.48754385,
            -1.5306191,
            -1.57365634,
            -1.61665564,
            -1.65961704,
            -1.7025406,
            -1.74542636,
            -1.78827437,
            -1.83108469,
            -1.87385737,
            -1.91659246,
            -1.95929001,
            -2.00195007,
            -2.04457269,
            -2.08715792,
            -2.12970582,
            -2.17221642,
            -2.21468979,
            -2.25712597,
            -2.29952502,
            -2.34188697,
            -2.38421189,
            -2.42649982,
            -2.46875082,
            -2.51096492,
            -2.55314219,
            -2.59528266,
            -2.6373864,
            -2.67945344,
            -2.72148384,
            -2.76347765,
            -2.80543491,
            -2.84735568,
            -2.88924,
            -2.93108792,
            -2.9728995,
            -3.01467477,
            -3.05641378,
            -3.0981166,
            -3.13978325,
            -3.1814138,
            -3.22300829,
            -3.26456677,
            -3.30608928,
            -3.34757587,
            -3.38902659,
            -3.43044149,
            -3.47182062,
            -3.51316402,
            -3.55447174,
            -3.59574383,
            -3.63698033,
            -3.67818129,
            -3.71934677,
            -3.76047679,
            -3.80157142,
            -3.8426307,
            -3.88365467,
            -3.92464338,
            -3.96559689,
            -4.00651522,
            -4.04739844,
            -4.08824658,
            -4.1290597,
            -4.16983783,
            -4.21058103,
            -4.25128934,
        ];
        let log = run_thermal(
            TempMeasurementConfig {
                fault_state: TempFaultState::Spiking,
                ..Default::default()
            },
            None,
        );
        let departures = nominals
            .iter()
            .zip(&log)
            .filter(|(nominal, sensed)| {
                if nominal.abs() > 0.0001 {
                    (*nominal - *sensed).abs() > 1e-6
                } else {
                    sensed.abs() > 1e-6
                }
            })
            .count();
        let fraction = departures as f64 / nominals.len() as f64;
        assert_relative(fraction, 0.1, 5e-1, "spiking fraction");
    }

    /// The noise pytests' `run_sensor`: a constant 20 °C truth, 1 s steps,
    /// `sample_count` samples. Returns the measurement errors.
    fn run_noise(
        sample_count: u64,
        seed: u64,
        a_matrix: Option<f64>,
        walk_bounds_c: Option<f64>,
    ) -> (TempMeasurement, Vec<f64>) {
        let true_temperature_c = 20.0;
        let mut config = TempMeasurementConfig {
            sen_noise_std_c: 1.0,
            ..Default::default()
        };
        if let Some(a_matrix) = a_matrix {
            config.a_matrix = a_matrix;
        }
        if let Some(walk_bounds_c) = walk_bounds_c {
            config.walk_bounds_c = walk_bounds_c;
        }
        let truth_out = Output::new(TemperatureMsg {
            temperature_c: true_temperature_c,
        });
        let mut sensor = TempMeasurement::new(config);
        sensor.set_rng_seed(seed);
        truth_out.connect_to(&mut sensor.temp_in_msg);
        sensor.init();
        let errors = (0..sample_count)
            .map(|second| {
                sensor.update(&context(second * STEP_NANOS));
                sensor.temp_out_msg.read().temperature_c - true_temperature_c
            })
            .collect();
        (sensor, errors)
    }

    fn mean(values: &[f64]) -> f64 {
        values.iter().sum::<f64>() / values.len() as f64
    }

    /// `np.corrcoef(values[:-1], values[1:])[0, 1]`.
    fn lag_one_correlation(values: &[f64]) -> f64 {
        let (head, tail) = (&values[..values.len() - 1], &values[1..]);
        let (head_mean, tail_mean) = (mean(head), mean(tail));
        let covariance: f64 = head
            .iter()
            .zip(tail)
            .map(|(x, y)| (x - head_mean) * (y - tail_mean))
            .sum();
        let head_ss: f64 = head.iter().map(|x| (x - head_mean).powi(2)).sum();
        let tail_ss: f64 = tail.iter().map(|y| (y - tail_mean).powi(2)).sum();
        covariance / (head_ss * tail_ss).sqrt()
    }

    #[test]
    fn default_noise_is_white() {
        let sample_count = 1000;
        let (sensor, errors) = run_noise(sample_count, 1234, None, None);
        let noise_std = 1.0;

        assert_eq!(sensor.a_matrix(), 0.0);
        assert!(sensor.config.read().walk_bounds_c <= 0.0);
        let errors_mean = mean(&errors);
        assert!(errors_mean.abs() < 5.0 * noise_std / (sample_count as f64).sqrt());
        let std = (errors
            .iter()
            .map(|e| (e - errors_mean).powi(2))
            .sum::<f64>()
            / (errors.len() - 1) as f64)
            .sqrt();
        assert!((std / noise_std - 1.0).abs() < 0.1, "std {std}");
        assert!(lag_one_correlation(&errors).abs() < 0.15);
    }

    #[test]
    fn explicit_random_walk_is_bounded() {
        let bound = 5.0;
        let (sensor, errors) = run_noise(400, 1234, Some(1.0), Some(bound));

        assert!(errors.iter().all(|e| e.abs() <= bound));
        assert!(errors.iter().any(|e| (e.abs() - bound).abs() <= 1.0e-12));
        assert!(lag_one_correlation(&errors) > 0.5);
        assert_eq!(sensor.a_matrix(), 1.0);
    }

    #[test]
    fn rng_seed_controls_noise_sequence() {
        let (_, first) = run_noise(8, 1234, None, None);
        let (_, repeated) = run_noise(8, 1234, None, None);
        let (_, different) = run_noise(8, 5678, None, None);

        assert_eq!(first, repeated);
        assert_ne!(first, different);
    }

    /// A reset restarts the streams: the run repeats its realization.
    #[test]
    fn reset_repeats_the_realization() {
        let (mut sensor, first) = run_noise(20, 99, Some(0.5), None);
        Module::reset(&mut sensor, &context(0));
        let again: Vec<f64> = (0..20)
            .map(|second| {
                sensor.update(&context(second * STEP_NANOS));
                sensor.temp_out_msg.read().temperature_c - 20.0
            })
            .collect();
        assert_eq!(first, again);
    }

    #[test]
    #[should_panic(expected = "probability of temperature spike on fault must be between 0 and 1")]
    fn rejects_a_spike_probability_above_one() {
        let _ = TempMeasurement::new(TempMeasurementConfig {
            spike_probability: 1.5,
            ..Default::default()
        });
    }
}
