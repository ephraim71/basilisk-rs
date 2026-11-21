//! Brightness temperatures of solar-system radio sources: port of upstream
//! Basilisk `architecture/utilities/brightnessTemperatureSolarSystem.hpp`.
//!
//! Simplified ITU-R P.372-17 (08/2024) models of the quiet Sun, the Earth,
//! the Moon and the galactic background, as functions of frequency.

use crate::utilities::astro_constants::CMB_TEMPERATURE;

/// Upstream `T_0` \[K\]: ITU-R P.372-17 reference temperature.
const REFERENCE_TEMPERATURE_K: f64 = 290.0;

/// A radio source upstream's `getBrightnessTemperatureFromData` knows by
/// name ("Sun", "Earth", "Moon", "Galaxy").
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RadioSource {
    /// The quiet Sun (ITU-R P.372-17 Figure 10).
    Sun,
    /// The Earth: 300 K worst case at 2 GHz and above, neglected below.
    Earth,
    /// The full Moon: 280 K at 2 GHz and above, neglected below.
    Moon,
    /// The galactic background, never below the CMB temperature.
    Galaxy,
}

/// A frequency outside a source model's range (upstream `bskError`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrequencyOutOfRange {
    /// The source asked for.
    pub source: RadioSource,
    /// The frequency asked for \[Hz\].
    pub frequency_hz: f64,
}

impl std::fmt::Display for FrequencyOutOfRange {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "frequency {} Hz is out of range for the {:?} brightness temperature model",
            self.frequency_hz, self.source
        )
    }
}

impl std::error::Error for FrequencyOutOfRange {}

/// Upstream `getBrightnessTemperatureFromData` \[K\]: brightness temperature
/// of `source` at `frequency_hz`.
///
/// # Errors
///
/// Below 50 MHz for the Sun, at or below 10 MHz for the galaxy, or for a NaN
/// frequency (each a `bskError` upstream).
pub fn brightness_temperature_k(
    source: RadioSource,
    frequency_hz: f64,
) -> Result<f64, FrequencyOutOfRange> {
    let out_of_range = FrequencyOutOfRange {
        source,
        frequency_hz,
    };
    let f_ghz = frequency_hz * 1e-9;
    match source {
        RadioSource::Sun => {
            if (5e7..2e8).contains(&frequency_hz) {
                Ok(1e6)
            } else if (2e8..1e9).contains(&frequency_hz) {
                Ok(5e5)
            } else if (1e9..1.4e9).contains(&frequency_hz) {
                Ok(10f64.powf(5.4525) * f_ghz.powf(-1.6044))
            } else if (1.4e9..1e10).contains(&frequency_hz) {
                Ok(10f64.powf(4.8891) * f_ghz.powf(-0.78897))
            } else if (1e10..1e11).contains(&frequency_hz) {
                Ok(10f64.powf(4.1985) * f_ghz.powf(-0.14))
            } else if frequency_hz >= 1e11 {
                Ok(5.0e3)
            } else {
                Err(out_of_range)
            }
        }
        RadioSource::Earth => Ok(if frequency_hz >= 2e9 { 300.0 } else { 0.0 }),
        RadioSource::Moon => Ok(if frequency_hz >= 2e9 { 280.0 } else { 0.0 }),
        RadioSource::Galaxy => {
            if frequency_hz > 1e7 && frequency_hz <= 1e8 {
                // ITU-R P.372-17 eq. 15: noise figure in dB, as a temperature.
                let f_am_db = 52.0 - 23.0 * (frequency_hz * 1e-6).log10();
                let background_k = (10f64.powf(f_am_db / 10.0) - 1.0) * REFERENCE_TEMPERATURE_K;
                Ok(if background_k > CMB_TEMPERATURE {
                    background_k
                } else {
                    CMB_TEMPERATURE
                })
            } else if frequency_hz > 1e8 && frequency_hz <= 1.16e9 {
                Ok(10f64.powf(0.99125) * f_ghz.powf(-2.94474))
            } else if frequency_hz > 1.16e9 {
                // Galactic emission is negligible; only the CMB remains.
                Ok(CMB_TEMPERATURE)
            } else {
                Err(out_of_range)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream has no unit test of its own for this header; these check
    //! each branch against its formula.

    use super::*;

    #[test]
    fn each_branch_matches_its_formula() {
        let t = |source, f: f64| brightness_temperature_k(source, f).unwrap();
        let close = |actual: f64, expected: f64| {
            assert!(
                (actual - expected).abs() <= 1e-12 * expected,
                "{actual} vs {expected}"
            );
        };
        assert_eq!(t(RadioSource::Sun, 1e8), 1e6);
        assert_eq!(t(RadioSource::Sun, 5e8), 5e5);
        close(
            t(RadioSource::Sun, 1.2e9),
            10f64.powf(5.4525) * 1.2f64.powf(-1.6044),
        );
        close(
            t(RadioSource::Sun, 2.2e9),
            10f64.powf(4.8891) * 2.2f64.powf(-0.78897),
        );
        close(
            t(RadioSource::Sun, 20e9),
            10f64.powf(4.1985) * 20f64.powf(-0.14),
        );
        assert_eq!(t(RadioSource::Sun, 2e11), 5e3);
        assert_eq!(t(RadioSource::Earth, 2.2e9), 300.0);
        assert_eq!(t(RadioSource::Earth, 1.0e9), 0.0);
        assert_eq!(t(RadioSource::Moon, 2.2e9), 280.0);
        assert_eq!(t(RadioSource::Moon, 1.0e9), 0.0);
        assert_eq!(t(RadioSource::Galaxy, 2.2e9), CMB_TEMPERATURE);
        close(
            t(RadioSource::Galaxy, 4e8),
            10f64.powf(0.99125) * 0.4f64.powf(-2.94474),
        );
        let f_am_db = 52.0 - 23.0 * 50f64.log10();
        close(
            t(RadioSource::Galaxy, 5e7),
            (10f64.powf(f_am_db / 10.0) - 1.0) * 290.0,
        );
    }

    #[test]
    fn out_of_range_frequencies_are_rejected() {
        assert!(brightness_temperature_k(RadioSource::Sun, 4e7).is_err());
        assert!(brightness_temperature_k(RadioSource::Galaxy, 1e7).is_err());
        assert!(brightness_temperature_k(RadioSource::Sun, f64::NAN).is_err());
    }
}
