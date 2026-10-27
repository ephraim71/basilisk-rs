//! First-order low-pass filter for scalar signals, ported from upstream
//! Basilisk `architecture/utilities/signalCondition.c` (`lowPassFilterSignal`
//! and its `LowPassFilterData` struct). `inertialUKF` filters each gyro axis
//! with it.

/// Port of upstream `LowPassFilterData` together with `lowPassFilterSignal`.
///
/// The filter is the bilinear (Tustin) discretization, without frequency
/// pre-warping, of `y' = omega (u - y)`:
///
/// `y_k = [(2 - h omega) y_(k-1) + h omega (u_k + u_(k-1))] / (2 + h omega)`
///
/// State semantics are upstream's: there is no special first call. The filter
/// starts from whatever [`Self::current_state`] and [`Self::current_meas`]
/// hold, zero for a new (C zero-initialized) filter, so the first output is
/// `h omega / (2 + h omega) * u_1` rather than `u_1`. Set the two state fields
/// to seed it otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LowPassFilter {
    /// Upstream `hStep` \[s\]: filter time step, assumed fixed.
    pub h_step_s: f64,
    /// Upstream `omegCutoff` [rad/s]: cutoff frequency.
    pub omega_cutoff_radps: f64,
    /// Upstream `currentState` [-]: latest filter output `y_k`.
    pub current_state: f64,
    /// Upstream `currentMeas` [-]: latest measurement `u_k` taken in.
    pub current_meas: f64,
}

impl LowPassFilter {
    /// A filter with zero state, like an upstream `LowPassFilterData` with only
    /// `hStep` and `omegCutoff` set.
    pub fn new(h_step_s: f64, omega_cutoff_radps: f64) -> Self {
        Self {
            h_step_s,
            omega_cutoff_radps,
            ..Self::default()
        }
    }

    /// Port of upstream `lowPassFilterSignal`: takes in `new_meas`, advances the
    /// state one step and returns the new [`Self::current_state`].
    pub fn filter(&mut self, new_meas: f64) -> f64 {
        let h_omega = self.h_step_s * self.omega_cutoff_radps;
        self.current_state = 1.0 / (2.0 + h_omega)
            * (self.current_state * (2.0 - h_omega) + h_omega * (new_meas + self.current_meas));
        self.current_meas = new_meas;
        self.current_state
    }
}

#[cfg(test)]
mod tests {
    //! Upstream has no dedicated `signalCondition.c` unit test; `test_inertialKF.py`
    //! only configures it (`hStep = 0.5`, `omegCutoff = 15 / (2 pi)`), which the
    //! tests below use as parameters.

    use std::f64::consts::PI;

    use super::LowPassFilter;

    const H_STEP_S: f64 = 0.5;
    const OMEGA_CUTOFF_RADPS: f64 = 15.0 / (2.0 * PI);

    #[test]
    fn first_call_starts_from_zero_state_without_seeding() {
        let mut filter = LowPassFilter::new(H_STEP_S, OMEGA_CUTOFF_RADPS);
        assert_eq!(filter.current_state, 0.0);
        assert_eq!(filter.current_meas, 0.0);
        assert_eq!(
            filter,
            LowPassFilter {
                h_step_s: H_STEP_S,
                omega_cutoff_radps: OMEGA_CUTOFF_RADPS,
                ..LowPassFilter::default()
            }
        );

        let h_omega = H_STEP_S * OMEGA_CUTOFF_RADPS;
        let output = filter.filter(3.0);
        assert!((output - h_omega / (2.0 + h_omega) * 3.0).abs() <= 1.0e-15);
        assert_eq!(output, filter.current_state);
        assert_eq!(filter.current_meas, 3.0);
    }

    #[test]
    fn step_response_matches_the_discrete_closed_form() {
        let mut filter = LowPassFilter::new(H_STEP_S, OMEGA_CUTOFF_RADPS);
        let h_omega = H_STEP_S * OMEGA_CUTOFF_RADPS;
        let pole = (2.0 - h_omega) / (2.0 + h_omega);
        let first = h_omega / (2.0 + h_omega);
        // For a unit step from rest: y_1 = first, then y_k - 1 = pole (y_(k-1) - 1).
        for k in 1..=40 {
            let expected = 1.0 - (1.0 - first) * pole.powi(k - 1);
            let output = filter.filter(1.0);
            assert!(
                (output - expected).abs() <= 1.0e-14,
                "step {k}: {output} vs {expected}"
            );
        }
        assert!(
            (filter.current_state - 1.0).abs() <= 1.0e-12,
            "unity DC gain"
        );
    }

    #[test]
    fn uses_and_advances_a_user_seeded_state() {
        let mut filter = LowPassFilter {
            current_state: 2.0,
            current_meas: 2.0,
            ..LowPassFilter::new(0.1, 4.0)
        };
        // A seeded steady state stays put under the same input.
        assert!((filter.filter(2.0) - 2.0).abs() <= 1.0e-15);

        let h_omega = 0.1 * 4.0;
        let expected = ((2.0 - h_omega) * 2.0 + h_omega * (-1.0 + 2.0)) / (2.0 + h_omega);
        assert!((filter.filter(-1.0) - expected).abs() <= 1.0e-15);
        assert_eq!(filter.current_meas, -1.0);
    }

    #[test]
    fn zero_cutoff_holds_the_state() {
        let mut filter = LowPassFilter {
            current_state: 0.7,
            ..LowPassFilter::new(0.5, 0.0)
        };
        for input in [1.0, -5.0, 100.0] {
            assert_eq!(filter.filter(input), 0.7);
        }
    }
}
