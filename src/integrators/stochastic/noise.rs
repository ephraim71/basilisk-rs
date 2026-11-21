//! Random variables for the stochastic integrators: port of upstream
//! `stochasticNoiseGenerator.h` and `stochasticWeakRandomVariables.h`.
//!
//! An integrator draws one [`GaussianNoiseSample`] per step from a
//! [`GaussianNoiseGenerator`]. Keeping the source pluggable lets a test replay
//! a known sequence of increments through [`PrescribedGaussianNoiseGenerator`],
//! which is how the integrators are checked against reference
//! implementations. The weak-order methods turn the Gaussian increments into
//! discrete variables with [`three_point`] and [`two_point`], so a prescribed
//! Gaussian sequence drives them exactly as it drives the references.
//!
//! Upstream's allocation-free `generate*Into` overloads collapse into the one
//! [`GaussianNoiseGenerator::generate_with_auxiliary`] call. Upstream throws
//! `std::runtime_error` on a malformed or exhausted replay queue; these panic
//! with the same message, as the integrator interface has no error path.

use std::collections::VecDeque;
use std::fmt;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};
use rand_distr::StandardNormal;

/// One step's Gaussian increments: upstream `GaussianNoiseSample`.
///
/// `dw` holds one Wiener increment per noise source, each `~ N(0, h)`. `dz`
/// holds the second, independent increments the higher-order Roessler
/// methods need; only the prefix an integrator requests is meaningful, and
/// the remaining entries are zero.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct GaussianNoiseSample {
    /// Upstream `dW`: Wiener increment per noise source.
    pub dw: Vec<f64>,
    /// Upstream `dZ`: auxiliary increment per noise source.
    pub dz: Vec<f64>,
}

/// Source of the Gaussian increments a stochastic integrator consumes:
/// upstream `GaussianNoiseGenerator`.
///
/// `Send` because an integrator lives inside a [`Module`](crate::Module).
pub trait GaussianNoiseGenerator: fmt::Debug + Send {
    /// Reseeds the underlying generator (upstream `setSeed`); a no-op for a
    /// replay source.
    fn set_seed(&mut self, seed: u64);

    /// Draws `noise_count` Wiener increments and the first `auxiliary_count`
    /// auxiliary increments for a step of `h` seconds (upstream
    /// `generateWithAuxiliaryInto`). The returned `dz` has `noise_count`
    /// entries; those past `auxiliary_count` carry no information.
    fn generate_with_auxiliary(
        &mut self,
        noise_count: usize,
        auxiliary_count: usize,
        h: f64,
    ) -> GaussianNoiseSample;

    /// Draws both increments for every noise source (upstream `generate`).
    fn generate(&mut self, noise_count: usize, h: f64) -> GaussianNoiseSample {
        self.generate_with_auxiliary(noise_count, noise_count, h)
    }

    /// Draws only the Wiener increments (upstream `generateWiener`).
    fn generate_wiener(&mut self, noise_count: usize, h: f64) -> Vec<f64> {
        self.generate_with_auxiliary(noise_count, 0, h).dw
    }
}

/// Seed of the default generator. Upstream seeds from `std::random_device`;
/// basilisk-rs keeps runs reproducible instead, like its sensor noise.
pub const DEFAULT_NOISE_SEED: u64 = 0;

/// Pseudo-random Gaussian increments: upstream
/// `RandomGaussianNoiseGenerator`.
///
/// Each increment is `sqrt(h) · N(0, 1)`; the `dw` entries are drawn first,
/// then the requested `dz` prefix. Upstream draws from a Mersenne Twister
/// through `std::normal_distribution`, whose sequence the C++ standard leaves
/// to the library; this draws from [`StdRng`], so the sequences differ while
/// the distribution is the same.
#[derive(Clone, Debug)]
pub struct RandomGaussianNoiseGenerator {
    rng: StdRng,
}

impl RandomGaussianNoiseGenerator {
    /// Generator seeded with `seed`.
    pub fn new(seed: u64) -> Self {
        Self {
            rng: StdRng::seed_from_u64(seed),
        }
    }

    fn standard_normal(&mut self) -> f64 {
        self.rng.sample(StandardNormal)
    }
}

impl Default for RandomGaussianNoiseGenerator {
    fn default() -> Self {
        Self::new(DEFAULT_NOISE_SEED)
    }
}

impl GaussianNoiseGenerator for RandomGaussianNoiseGenerator {
    fn set_seed(&mut self, seed: u64) {
        self.rng = StdRng::seed_from_u64(seed);
    }

    fn generate_with_auxiliary(
        &mut self,
        noise_count: usize,
        auxiliary_count: usize,
        h: f64,
    ) -> GaussianNoiseSample {
        assert!(
            auxiliary_count <= noise_count,
            "GaussianNoiseGenerator auxiliary count {auxiliary_count} exceeds the \
             {noise_count} noise sources"
        );
        let sqrt_h = h.sqrt();
        let dw = (0..noise_count)
            .map(|_| sqrt_h * self.standard_normal())
            .collect();
        let mut dz = vec![0.0; noise_count];
        for entry in dz.iter_mut().take(auxiliary_count) {
            *entry = sqrt_h * self.standard_normal();
        }
        GaussianNoiseSample { dw, dz }
    }
}

/// Replays a queued sequence of increments, one sample per step: upstream
/// `PrescribedGaussianNoiseGenerator`.
///
/// The queued values are the increments themselves (already scaled to the
/// step), returned verbatim whatever `h` is requested.
#[derive(Clone, Debug, Default)]
pub struct PrescribedGaussianNoiseGenerator {
    samples: VecDeque<GaussianNoiseSample>,
}

impl PrescribedGaussianNoiseGenerator {
    /// Empty replay queue.
    pub fn new() -> Self {
        Self::default()
    }

    /// Queues one step's increments (upstream `pushStep`). An empty `dz`
    /// replays as zeros, one per entry of `dw`.
    pub fn push_step(&mut self, dw: &[f64], dz: &[f64]) {
        let dz = if dz.is_empty() {
            vec![0.0; dw.len()]
        } else {
            dz.to_vec()
        };
        self.samples.push_back(GaussianNoiseSample {
            dw: dw.to_vec(),
            dz,
        });
    }

    /// Drops every queued sample (upstream `clear`).
    pub fn clear(&mut self) {
        self.samples.clear();
    }

    /// Number of samples still queued (upstream `remaining`).
    pub fn remaining(&self) -> usize {
        self.samples.len()
    }

    /// Takes the next sample, checking it against the requested counts
    /// (upstream `nextAndValidate`).
    fn next_sample(&mut self, noise_count: usize, auxiliary_count: usize) -> GaussianNoiseSample {
        let sample = self.samples.pop_front().unwrap_or_else(|| {
            panic!(
                "PrescribedGaussianNoiseGenerator ran out of prescribed noise samples. \
                 Push one sample per integration step."
            )
        });
        assert_eq!(
            sample.dw.len(),
            noise_count,
            "PrescribedGaussianNoiseGenerator: the queued sample has a different number of \
             noise sources than requested by the integrator."
        );
        assert!(
            sample.dz.len() >= auxiliary_count,
            "PrescribedGaussianNoiseGenerator: the queued sample's dZ is shorter than the \
             auxiliary count requested by the integrator."
        );
        sample
    }
}

impl GaussianNoiseGenerator for PrescribedGaussianNoiseGenerator {
    /// Seeding has no effect on a replay.
    fn set_seed(&mut self, _seed: u64) {}

    fn generate_with_auxiliary(
        &mut self,
        noise_count: usize,
        auxiliary_count: usize,
        _h: f64,
    ) -> GaussianNoiseSample {
        self.next_sample(noise_count, auxiliary_count)
    }
}

/// Upstream `stochasticWeakRV::NORMAL_ONESIX_QUANTILE`: the 1/6 quantile of
/// the standard normal distribution.
pub const NORMAL_ONESIX_QUANTILE: f64 = -0.967_421_566_101_701_4;

/// Three-point random variable taking `-sqrt(3h)`, `0`, `+sqrt(3h)` with
/// probabilities 1/6, 2/3, 1/6, built from a Gaussian increment
/// `dw ~ N(0, h)` by thresholding `dw / sqrt(h)` at the 1/6 quantile:
/// upstream `stochasticWeakRV::threePoint`.
pub fn three_point(dw: f64, h: f64) -> f64 {
    let sqrt_3h = (3.0 * h).sqrt();
    let scaled = dw / h.sqrt();
    if scaled.abs() > -NORMAL_ONESIX_QUANTILE {
        if scaled < NORMAL_ONESIX_QUANTILE {
            -sqrt_3h
        } else {
            sqrt_3h
        }
    } else {
        0.0
    }
}

/// Two-point random variable `±scale`: `+scale` for a strictly positive
/// increment, `-scale` otherwise (upstream `stochasticWeakRV::twoPoint`).
pub fn two_point(dz: f64, scale: f64) -> f64 {
    if dz > 0.0 { scale } else { -scale }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_stochasticNoiseGenerator.py`. The Python-binding
    //! check `test_native_output_buffer_methods_are_not_python_apis` has no
    //! Rust counterpart. Upstream's raises become `#[should_panic]`.

    use super::*;

    #[test]
    fn prescribed_replays_in_fifo_order_with_remaining() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        assert_eq!(generator.remaining(), 0);
        generator.push_step(&[0.1, 0.2], &[0.3, 0.4]);
        generator.push_step(&[1.1, 1.2], &[1.3, 1.4]);
        assert_eq!(generator.remaining(), 2);

        let first = generator.generate(2, 0.01);
        assert_eq!(first.dw, [0.1, 0.2]);
        assert_eq!(first.dz, [0.3, 0.4]);
        assert_eq!(generator.remaining(), 1);

        let second = generator.generate(2, 0.01);
        assert_eq!(second.dw, [1.1, 1.2]);
        assert_eq!(second.dz, [1.3, 1.4]);
        assert_eq!(generator.remaining(), 0);
    }

    #[test]
    fn prescribed_omitted_dz_zero_fills() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        generator.push_step(&[1.0, 2.0], &[]);
        let sample = generator.generate(2, 0.01);
        assert_eq!(sample.dw, [1.0, 2.0]);
        assert_eq!(sample.dz, [0.0, 0.0]);
    }

    #[test]
    fn prescribed_clear() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        generator.push_step(&[1.0], &[]);
        generator.push_step(&[2.0], &[]);
        assert_eq!(generator.remaining(), 2);
        generator.clear();
        assert_eq!(generator.remaining(), 0);
    }

    #[test]
    #[should_panic(expected = "ran out of prescribed noise samples")]
    fn prescribed_queue_exhaustion_panics() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        generator.push_step(&[1.0], &[]);
        generator.generate(1, 0.01);
        generator.generate(1, 0.01);
    }

    #[test]
    #[should_panic(expected = "different number of noise sources")]
    fn prescribed_mismatched_dw_length_panics() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        generator.push_step(&[1.0, 2.0], &[]);
        generator.generate(1, 0.01);
    }

    #[test]
    #[should_panic(expected = "shorter than the auxiliary count")]
    fn prescribed_undersized_dz_panics() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        generator.push_step(&[1.0, 2.0], &[0.5]);
        generator.generate(2, 0.01);
    }

    /// An undersized `dz` is enough when the integrator asks for fewer
    /// auxiliary draws, as the Wiener-only methods do.
    #[test]
    fn prescribed_short_dz_serves_smaller_auxiliary_request() {
        let mut generator = PrescribedGaussianNoiseGenerator::new();
        generator.push_step(&[1.0, 2.0], &[0.5]);
        assert_eq!(generator.generate_with_auxiliary(2, 1, 0.01).dz, [0.5]);
        generator.push_step(&[1.0, 2.0], &[0.5]);
        assert_eq!(generator.generate_wiener(2, 0.01), [1.0, 2.0]);
    }

    #[test]
    fn random_seed_repeatable() {
        let mut first = RandomGaussianNoiseGenerator::default();
        let mut second = RandomGaussianNoiseGenerator::default();
        first.set_seed(12345);
        second.set_seed(12345);
        let sample_first = first.generate(3, 0.04);
        let sample_second = second.generate(3, 0.04);
        assert_eq!(sample_first, sample_second);

        second.set_seed(54321);
        let other = second.generate(3, 0.04);
        assert!(
            sample_first
                .dw
                .iter()
                .zip(&other.dw)
                .any(|(a, b)| (a - b).abs() > 1.0e-12),
            "a different seed should draw a different sequence"
        );
    }

    /// With a fixed seed, a quarter of the step halves every increment.
    #[test]
    fn random_increments_scale_with_sqrt_h() {
        let mut generator = RandomGaussianNoiseGenerator::default();
        generator.set_seed(7);
        let big = generator.generate(4, 1.0).dw;
        generator.set_seed(7);
        let small = generator.generate(4, 0.25).dw;
        for (small, big) in small.iter().zip(&big) {
            assert!((small - 0.5 * big).abs() <= 1.0e-12 * (0.5 * big).abs());
        }
    }

    #[test]
    fn random_wiener_only_skips_auxiliary_draws() {
        let mut first = RandomGaussianNoiseGenerator::default();
        let mut second = RandomGaussianNoiseGenerator::default();
        first.set_seed(4321);
        second.set_seed(4321);

        let first_step = first.generate_wiener(3, 0.25);
        let second_step = first.generate_wiener(3, 0.25);

        assert_eq!(first_step, second.generate_wiener(3, 0.25));
        assert_eq!(second_step, second.generate_wiener(3, 0.25));
    }

    #[test]
    fn random_reseed_restarts_the_sequence() {
        let mut generator = RandomGaussianNoiseGenerator::default();
        let mut reference = RandomGaussianNoiseGenerator::default();
        generator.set_seed(2468);
        reference.set_seed(2468);

        let first = generator.generate_wiener(1, 0.25);
        generator.set_seed(2468);
        let repeated = generator.generate_wiener(1, 0.25);
        let expected = reference.generate_wiener(1, 0.25);

        assert_eq!(first, expected);
        assert_eq!(repeated, expected);
    }

    /// The auxiliary prefix follows the Wiener draws and the rest stays zero.
    #[test]
    fn random_auxiliary_prefix_follows_wiener_draws() {
        let mut generator = RandomGaussianNoiseGenerator::new(99);
        let sample = generator.generate_with_auxiliary(3, 2, 1.0);
        let mut reference = RandomGaussianNoiseGenerator::new(99);
        let draws = reference.generate_wiener(5, 1.0);
        assert_eq!(sample.dw, draws[..3]);
        assert_eq!(sample.dz, [draws[3], draws[4], 0.0]);
    }

    #[test]
    fn three_point_thresholds_at_the_one_sixth_quantile() {
        let h: f64 = 0.25;
        let sqrt_3h = (3.0 * h).sqrt();
        let scale = h.sqrt();
        assert_eq!(three_point(0.0, h), 0.0);
        assert_eq!(three_point(0.9 * scale, h), 0.0);
        assert_eq!(three_point(-0.9 * scale, h), 0.0);
        assert_eq!(three_point(scale, h), sqrt_3h);
        assert_eq!(three_point(-scale, h), -sqrt_3h);
        // Exactly at the quantile the variable stays zero (strict comparison).
        assert_eq!(three_point(NORMAL_ONESIX_QUANTILE * scale, h), 0.0);
    }

    #[test]
    fn two_point_maps_zero_to_the_negative_value() {
        assert_eq!(two_point(1.0e-300, 2.0), 2.0);
        assert_eq!(two_point(0.0, 2.0), -2.0);
        assert_eq!(two_point(-0.5, 2.0), -2.0);
    }
}
