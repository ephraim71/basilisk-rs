//! Test-only ports of the Python helpers the upstream `transDetermination`
//! unit tests build their inputs with: numpy's `linspace` and
//! `polynomial.chebyshev.chebfit`, and the Meeus planet ephemerides of
//! `Basilisk.utilities.astroFunctions`.

use nalgebra::{DMatrix, Matrix3, SVector, Vector3};

use crate::utilities::astro_constants::{AU, D2R, MU_SUN};

/// `numpy.linspace(start, stop, num)`: `start + i * step`, with the last
/// sample set to `stop` exactly, as numpy does.
pub(crate) fn linspace(start: f64, stop: f64, num: usize) -> Vec<f64> {
    if num == 1 {
        return vec![start];
    }
    let step = (stop - start) / (num - 1) as f64;
    let mut samples: Vec<f64> = (0..num).map(|i| i as f64 * step + start).collect();
    if let Some(last) = samples.last_mut() {
        *last = stop;
    }
    samples
}

/// `numpy.polynomial.chebyshev.chebfit(x, y, degree)`: least-squares
/// Chebyshev coefficients of each component of `y` sampled at `x`.
///
/// As numpy, the Chebyshev-Vandermonde columns are scaled to unit norm before
/// an SVD least-squares solve that drops singular values below
/// `len(x) * eps` of the largest. Entry `i` of the result multiplies `T_i`.
/// The SVD is taken of the triangular factor of a QR decomposition, which has
/// the same singular values and is much cheaper for tall matrices.
pub(crate) fn chebfit<const K: usize>(
    x: &[f64],
    y: &[SVector<f64, K>],
    degree: usize,
) -> Vec<SVector<f64, K>> {
    assert_eq!(x.len(), y.len(), "chebfit needs one sample per abscissa");
    let terms = degree + 1;
    assert!(
        x.len() >= terms,
        "chebfit needs at least degree + 1 samples"
    );
    // numpy chebvander: v_0 = 1, v_1 = x, v_i = v_{i-1} * 2x - v_{i-2}.
    let mut vandermonde = DMatrix::zeros(x.len(), terms);
    for (row, &xi) in x.iter().enumerate() {
        vandermonde[(row, 0)] = 1.0;
        if terms > 1 {
            vandermonde[(row, 1)] = xi;
        }
        for j in 2..terms {
            vandermonde[(row, j)] =
                vandermonde[(row, j - 1)] * (2.0 * xi) - vandermonde[(row, j - 2)];
        }
    }
    let scale: Vec<f64> = vandermonde
        .column_iter()
        .map(|column| {
            let norm = column.norm();
            if norm == 0.0 { 1.0 } else { norm }
        })
        .collect();
    for (mut column, &norm) in vandermonde.column_iter_mut().zip(&scale) {
        column /= norm;
    }

    let mut rhs = DMatrix::from_fn(y.len(), K, |row, col| y[row][col]);
    let qr = vandermonde.qr();
    qr.q_tr_mul(&mut rhs);
    let svd = qr.r().svd(true, true);
    let cutoff = x.len() as f64 * f64::EPSILON * svd.singular_values.max();
    let solution = svd
        .solve(&rhs.rows(0, terms), cutoff)
        .expect("the SVD was computed with both U and V");
    scale
        .iter()
        .enumerate()
        .map(|(term, &norm)| SVector::from_fn(|col, _| solution[(term, col)] / norm))
        .collect()
}

/// The planets whose Meeus ephemerides the upstream tests use.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Planet {
    Earth,
    Mars,
    Jupiter,
    Saturn,
}

/// `astroFunctions.JulianDate([year, month, day])`, including its final
/// `ceil`.
pub(crate) fn julian_date(year: i64, month: i64, day: i64) -> f64 {
    // Python's int() of these positive quotients is integer division.
    let days = 367 * year - 7 * (year + (month + 9) / 12) / 4 + 275 * month / 9 + day;
    (days as f64 + 1721013.5).ceil()
}

/// `astroFunctions.<Planet>_RV(jde)`: heliocentric ecliptic position \[km\]
/// and velocity \[km/s\] of `planet` from the Meeus mean elements, propagated
/// as a two-body orbit about the sun.
pub(crate) fn planet_rv(planet: Planet, jde: f64) -> (Vector3<f64>, Vector3<f64>) {
    let (a, e, i, raan, arg_periapsis, nu) = meeus_elements(planet, jde);
    elements_to_rv(MU_SUN, a, e, i, raan, arg_periapsis, nu)
}

/// `astroFunctions.ephemeridesMeeus` followed by `computeCOE`:
/// `(a [km], e, i, Omega, w, nu [rad])`.
fn meeus_elements(planet: Planet, jde: f64) -> (f64, f64, f64, f64, f64, f64) {
    let t = (jde - 2451545.0) / 36525.0;
    // Upstream's helper for the outer planets uses T where T^2 belongs.
    let ephem = |c: [f64; 4]| c[0] + c[1] * t + c[2] * t + c[3] * t * t * t;
    let (l, a, e, i, raan, pi) = match planet {
        Planet::Earth => (
            100.466449 + 35999.3728519 * t + (-0.00000568) * t * t,
            1.000001018,
            0.01670862 + (-0.000042037) * t + (-0.0000001236) * t * t + 0.00000000004 * t * t * t,
            0.0130546 * t + (-0.00000931) * t * t + (-0.000000034) * t * t * t,
            174.873174 + (-0.2410908) * t + 0.00004067 * t * t + (-0.000001327) * t * t * t,
            102.937348 + 0.3225557 * t + 0.00015026 * t * t + 0.000000478 * t * t * t,
        ),
        Planet::Mars => (
            355.433275 + 19140.2993313 * t + 0.00000261 * t * t + (-0.000000003) * t * t * t,
            1.523679342,
            0.09340062 + 0.000090483 * t + (-0.0000000806) * t * t + (-0.00000000035) * t * t * t,
            1.849726 + (-0.0081479) * t + (-0.00002255) * t * t + (-0.000000027) * t * t * t,
            49.558093 + (-0.2949846) * t + (-0.00063993) * t * t + (-0.000002143) * t * t * t,
            336.060234 + 0.4438898 * t + (-0.00017321) * t * t + 0.000000300 * t * t * t,
        ),
        Planet::Jupiter => (
            34.351484 + 3034.9056746 * t + (-0.00008501) * t * t + 0.000000004 * t * t * t,
            5.202603191 + 0.0000001913 * t,
            0.04849485 + 0.000163244 * t + -0.0000004719 * t * t + (-0.00000000197) * t * t * t,
            1.303270 + (-0.0019872) * t + 0.00003318 * t * t + 0.000000092 * t * t * t,
            100.464441 + 0.1766828 * t + 0.00090387 * t * t + (-0.000007032) * t * t * t,
            14.331309 + 0.2155525 * t + 0.00072252 * t * t + (-0.000004590) * t * t * t,
        ),
        Planet::Saturn => (
            ephem([50.077471, 1222.1137943, 0.00021004, -0.000000019]),
            ephem([9.554909596, -0.0000021389, 0.0, 0.0]),
            ephem([0.05550862, -0.000346818, -0.0000006456, 0.00000000338]),
            ephem([2.488878, 0.0025515, -0.00004903, 0.000000018]),
            ephem([113.665524, -0.2566649, -0.00018345, 0.000000357]),
            ephem([93.056787, 0.5665496, 0.00052809, 0.000004882]),
        ),
    };

    // computeCOE
    let (l, a, i, raan, pi) = (l * D2R, a * AU, i * D2R, raan * D2R, pi * D2R);
    let arg_periapsis = pi - raan;
    let mean_anomaly = l - pi;
    // Upstream writes the equation-of-centre coefficients (1/4, 5/96, 5/4,
    // ...) with Python floor division, so each evaluates to 0 or 1.
    let equation_of_centre = 2.0 * e * mean_anomaly.sin()
        + e.powi(2) * (2.0 * mean_anomaly).sin()
        + e.powi(3) * (3.0 * mean_anomaly).sin()
        + e.powi(4) * (4.0 * mean_anomaly).sin()
        + e.powi(5) * (5.0 * mean_anomaly).sin();
    (
        a,
        e,
        i,
        raan,
        arg_periapsis,
        mean_anomaly + equation_of_centre,
    )
}

/// `astroFunctions.OE2RV` with `PQW2IJK`.
fn elements_to_rv(
    mu: f64,
    a: f64,
    e: f64,
    i: f64,
    raan: f64,
    arg_periapsis: f64,
    nu: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let p = a * (1.0 - e * e);
    let (s, c) = nu.sin_cos();
    let r_pqw = Vector3::new(p * c / (1.0 + e * c), p * s / (1.0 + e * c), 0.0);
    let v_pqw = Vector3::new(-s * (mu / p).sqrt(), (e + c) * (mu / p).sqrt(), 0.0);

    let (s_o, c_o) = raan.sin_cos();
    let (s_i, c_i) = i.sin_cos();
    let (s_w, c_w) = arg_periapsis.sin_cos();
    let pqw_to_ijk = Matrix3::new(
        c_o * c_w - s_o * s_w * c_i,
        -c_o * s_w - s_o * c_w * c_i,
        s_o * s_i,
        s_o * c_w + c_o * s_w * c_i,
        -s_o * s_w + c_o * c_w * c_i,
        -c_o * s_i,
        s_w * s_i,
        c_w * s_i,
        c_i,
    );
    (pqw_to_ijk * r_pqw, pqw_to_ijk * v_pqw)
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector1;

    use super::{chebfit, julian_date, linspace};

    #[test]
    fn linspace_ends_exactly_on_both_bounds() {
        let samples = linspace(-1.0, 1.0, 5);
        assert_eq!(samples, vec![-1.0, -0.5, 0.0, 0.5, 1.0]);
        assert_eq!(linspace(0.1, 0.7, 7).last(), Some(&0.7));
    }

    #[test]
    fn chebfit_recovers_an_exact_chebyshev_series() {
        let x = linspace(-1.0, 1.0, 41);
        // 0.5 T_0 - 2 T_1 + 0.25 T_3
        let y: Vec<Vector1<f64>> = x
            .iter()
            .map(|&x| Vector1::new(0.5 - 2.0 * x + 0.25 * (4.0 * x * x * x - 3.0 * x)))
            .collect();
        let coefficients = chebfit(&x, &y, 4);
        for (fitted, expected) in coefficients.iter().zip([0.5, -2.0, 0.0, 0.25, 0.0]) {
            assert!(
                (fitted.x - expected).abs() < 1e-13,
                "{fitted} != {expected}"
            );
        }
    }

    #[test]
    fn julian_date_matches_the_upstream_helper() {
        // 2018-10-16 is JD 2458407.5 at 0h; upstream rounds it up.
        assert_eq!(julian_date(2018, 10, 16), 2458408.0);
    }
}
