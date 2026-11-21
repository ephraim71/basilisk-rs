//! Chebyshev-series helpers shared by the ephemeris-fit modules: port of
//! upstream Basilisk `transDetermination/_GeneralModuleFiles/ephemerisUtilities`.
//!
//! Upstream provides one function, `calculateChebyValue`
//! ([`calculate_cheby_value`]). The crate-private helpers below factor out the
//! two steps [`super::cheby_pos_ephem`] and [`super::oe_state_ephem`] repeat
//! verbatim upstream: correlating the vehicle clock with ephemeris time, and
//! scaling that time into a fit interval, railed to `[-1, 1]`.

use std::ops::{Add, Mul};

use crate::messages::TdbVehicleClockCorrelationMsg;
use crate::time::NANO2SEC;

#[cfg(test)]
pub(crate) mod test_support;

/// Evaluates the Chebyshev series `sum_i c_i T_i(x)` at `eval_value`: port of
/// upstream `calculateChebyValue`.
///
/// `coefficients[i]` multiplies the first-kind polynomial `T_i`; the series has
/// `coefficients.len()` terms. The polynomials come from the three-term
/// recurrence `T_i = 2 x T_{i-1} - T_{i-2}` and the terms are summed in index
/// order, as upstream. The coefficient type may be `f64` or a vector such as
/// `Vector3<f64>`; each vector component is then evaluated with exactly the
/// scalar operations.
///
/// An empty series evaluates to `T::default()`, which is zero for `f64` and the
/// nalgebra vectors. Upstream has no empty case: it always reads the constant
/// term, which a zeroed coefficient buffer also makes zero.
pub fn calculate_cheby_value<T>(coefficients: &[T], eval_value: f64) -> T
where
    T: Copy + Default + Add<Output = T> + Mul<f64, Output = T>,
{
    let Some((&constant, higher)) = coefficients.split_first() else {
        return T::default();
    };
    let mut cheby_prev = 1.0;
    let mut cheby_now = eval_value;
    let value_mult = 2.0 * eval_value;

    let mut est_value = constant * cheby_prev;
    let Some((&linear, rest)) = higher.split_first() else {
        return est_value;
    };
    est_value = est_value + linear * cheby_now;
    for &coefficient in rest {
        let cheby_local_prev = cheby_now;
        cheby_now = value_mult * cheby_now - cheby_prev;
        cheby_prev = cheby_local_prev;
        est_value = est_value + coefficient * cheby_now;
    }
    est_value
}

/// Ephemeris time (TDB) \[s\] at vehicle clock time `call_time_nanos`, shifted
/// by the clock correlation, as both ephemeris-fit modules compute it
/// (upstream adds the correlation offset as one term).
pub(crate) fn ephemeris_time_s(
    call_time_nanos: u64,
    correlation: &TdbVehicleClockCorrelationMsg,
) -> f64 {
    call_time_nanos as f64 * NANO2SEC
        + (correlation.ephemeris_time_s - correlation.vehicle_clock_time_s)
}

/// Position of `ephemeris_time_s` within the fit interval centred on
/// `time_mid_s` with half-width `time_radius_s`, scaled to `[-1, 1]`. Times
/// outside the interval rail to the nearest end, so the fitted state stays at
/// its boundary value.
///
/// Upstream rails by dividing by the magnitude rather than taking the sign,
/// which only differs for `±inf` (a zero interval radius): that gives `NaN`, as
/// does a `NaN` input.
pub(crate) fn scaled_fit_time(ephemeris_time_s: f64, time_mid_s: f64, time_radius_s: f64) -> f64 {
    let scaled_time = (ephemeris_time_s - time_mid_s) / time_radius_s;
    if scaled_time.abs() > 1.0 {
        scaled_time / scaled_time.abs()
    } else {
        scaled_time
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::messages::TdbVehicleClockCorrelationMsg;

    use super::{calculate_cheby_value, ephemeris_time_s, scaled_fit_time};

    // Upstream ephemerisUtilities has no unit test; these check the
    // algorithm against closed forms.

    /// `T_n(x) = cos(n acos x)` on `[-1, 1]`.
    fn chebyshev_closed_form(n: usize, x: f64) -> f64 {
        (n as f64 * x.acos()).cos()
    }

    #[test]
    fn a_single_coefficient_series_is_its_basis_polynomial() {
        for degree in 0..40 {
            let mut coefficients = vec![0.0; degree + 1];
            coefficients[degree] = 1.0;
            for x in [-1.0, -0.73, -0.2, 0.0, 0.31, 0.5, 0.99, 1.0] {
                let value = calculate_cheby_value(&coefficients, x);
                assert!(
                    (value - chebyshev_closed_form(degree, x)).abs() < 1e-12,
                    "T_{degree}({x}) = {value}"
                );
            }
        }
    }

    #[test]
    fn sums_the_weighted_basis_polynomials() {
        let coefficients = [0.7, -1.3, 2.1, 0.25, -0.05, 1.5e-3];
        for x in [-0.9, -0.4, 0.0, 0.6, 0.95] {
            let expected: f64 = coefficients
                .iter()
                .enumerate()
                .map(|(n, c)| c * chebyshev_closed_form(n, x))
                .sum();
            assert!((calculate_cheby_value(&coefficients, x) - expected).abs() < 1e-13);
        }
        // One coefficient: upstream returns before the recurrence.
        assert_eq!(calculate_cheby_value(&[4.5], 0.3), 4.5);
        // Two coefficients: c0 + c1 x.
        assert_eq!(calculate_cheby_value(&[4.5, -2.0], 0.25), 4.0);
        // Outside [-1, 1] the recurrence still evaluates the polynomial:
        // T_2(2) = 2 * 2^2 - 1.
        assert_eq!(calculate_cheby_value(&[0.0, 0.0, 1.0], 2.0), 7.0);
    }

    #[test]
    fn an_empty_series_is_zero() {
        assert_eq!(calculate_cheby_value::<f64>(&[], 0.4), 0.0);
        assert_eq!(
            calculate_cheby_value::<Vector3<f64>>(&[], f64::NAN),
            Vector3::zeros()
        );
        // The constant term is weighted by T_0 = 1 even at a NaN time, as
        // upstream's `chebyCoeff[0] * 1.0`.
        assert_eq!(calculate_cheby_value(&[2.0], f64::NAN), 2.0);
    }

    #[test]
    fn vector_coefficients_match_the_scalar_series_bit_for_bit() {
        let coefficients = [
            Vector3::new(1.0, -2.0, 0.5),
            Vector3::new(0.3, 0.7, -1.1),
            Vector3::new(-0.01, 0.2, 0.33),
            Vector3::new(1e-3, -4e-4, 2.5e-2),
        ];
        let x = 0.618;
        let vector = calculate_cheby_value(&coefficients, x);
        for axis in 0..3 {
            let scalar: Vec<f64> = coefficients.iter().map(|c| c[axis]).collect();
            assert_eq!(vector[axis], calculate_cheby_value(&scalar, x));
        }
    }

    #[test]
    fn scales_and_rails_time_to_the_fit_interval() {
        assert_eq!(scaled_fit_time(104.0, 100.0, 10.0), 0.4);
        assert_eq!(scaled_fit_time(90.0, 100.0, 10.0), -1.0);
        assert_eq!(scaled_fit_time(110.0 + 1e-9, 100.0, 10.0), 1.0);
        assert_eq!(scaled_fit_time(-270.0, 100.0, 10.0), -1.0);
        // A zero radius gives inf, which upstream's x / |x| turns into NaN.
        assert!(scaled_fit_time(101.0, 100.0, 0.0).is_nan());
        assert!(scaled_fit_time(100.0, 100.0, 0.0).is_nan());
    }

    #[test]
    fn correlates_vehicle_clock_with_ephemeris_time() {
        let correlation = TdbVehicleClockCorrelationMsg {
            ephemeris_time_s: 5.0e8,
            vehicle_clock_time_s: 20.0,
        };
        assert_eq!(
            ephemeris_time_s(30_500_000_000, &correlation),
            30.5 + 499_999_980.0
        );
    }
}
