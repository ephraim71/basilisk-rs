//! Two-body orbital mechanics: anomaly conversions, classical elements to and
//! from inertial state, the first-order J2 mean/osculating element map,
//! equinoctial elements and the Hill (LVLH) frame. Ported from upstream
//! Basilisk `architecture/utilities/orbitalMotion.c`.
//!
//! | upstream | here |
//! |---|---|
//! | `ClassicElements` | [`ClassicElements`] |
//! | `equinoctialElements` | [`EquinoctialElements`] |
//! | `E2f` | [`eccentric_to_true_anomaly`] |
//! | `E2M` | [`eccentric_to_mean_anomaly`] |
//! | `f2E` | [`true_to_eccentric_anomaly`] |
//! | `f2H` | [`true_to_hyperbolic_anomaly`] |
//! | `H2f` | [`hyperbolic_to_true_anomaly`] |
//! | `H2N` | [`hyperbolic_to_mean_hyperbolic_anomaly`] |
//! | `M2E` | [`mean_to_eccentric_anomaly`] |
//! | `N2H` | [`mean_hyperbolic_to_hyperbolic_anomaly`] |
//! | `elem2rv` | [`elements_to_state`] |
//! | `rv2elem` | [`state_to_elements`] |
//! | `clMeanOscMap` (`sgn` = ±1) | [`mean_osculating_map`] ([`MeanOscDirection`]) |
//! | `clElem2eqElem` | [`classic_to_equinoctial`] |
//! | `hillFrame` | [`hill_frame`] |
//! | `hill2rv` | [`hill_to_state`] |
//! | `rv2hill` | [`state_to_hill`] |
//!
//! # Conventions
//!
//! * **Units are whatever the caller uses.** Upstream is unit-agnostic: every
//!   distance (positions, semi-major axis, radii) is in the length unit of
//!   `mu`, and velocities in that unit per second. Basilisk messages are SI,
//!   so flight software normally passes `mu` in m^3/s^2 and gets metres back.
//!   The `MU_*` and `REQ_*` values in [`crate::utilities::astro_constants`]
//!   are **km** based — scale them (`MU_EARTH * 1e9`) when working in metres.
//! * Angles are radians. Frames follow upstream naming: `_n` is inertial,
//!   `_h` is Hill.
//! * Invalid inputs behave as upstream: the anomaly conversions return `NaN`
//!   and log an error (upstream `BSK_PRINT(MSG_ERROR, ...)` becomes
//!   `log::error!`), and the `NaN` propagates through any caller.
//!
//! Upstream's `atmosphericDensity`, `debyeLength`, `atmosphericDrag`,
//! `jPerturb` and `solarRad` are environment models no flight-software
//! module calls, and are not ported here.

use std::f64::consts::TAU;

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use crate::utilities::linear_algebra::{normalize_or_zero, safe_acos, safe_asin};

/// Upstream `eps` in `elem2rv` and `rv2elem`: below this an eccentricity,
/// semi-major axis, inverse semi-major axis or line-of-nodes magnitude is
/// treated as zero (circular, parabolic or equatorial).
const SMALL_ELEMENT: f64 = 1e-12;
/// Upstream `small` in `M2E`/`N2H`: Newton iterations stop once the update is
/// no larger than this (rad).
const ANOMALY_TOLERANCE_RAD: f64 = 1e-13;
/// Upstream `max` in `M2E`/`N2H`: Newton iteration budget before giving up.
const MAX_ANOMALY_ITERATIONS: u32 = 200;
/// Upstream `N2H` clamps its initial guess to this hyperbolic anomaly (rad).
const MAX_INITIAL_HYPERBOLIC_ANOMALY_RAD: f64 = 7.0;

/// Classical (Keplerian) orbit elements. Port of upstream `ClassicElements`.
///
/// Distances are in the length unit of the `mu` they are used with (upstream
/// is unit-agnostic); angles are radians. `Default` is all zeros, matching
/// the zero-initialized C struct most upstream callers start from.
///
/// Special cases, as upstream encodes them:
/// * **Parabolic** (`e = 1`): set `semi_major_axis = 0` and give the orbit
///   through `periapsis_radius`; [`state_to_elements`] reports a parabola
///   with `semi_major_axis = 0` and `apoapsis_radius = 0`.
/// * **Rectilinear** (`e = 1`, `semi_major_axis != 0`): `true_anomaly_rad`
///   holds the eccentric anomaly (elliptic, `a > 0`) or the hyperbolic
///   anomaly (`a < 0`) instead.
/// * **Hyperbolic** (`e > 1`): `semi_major_axis` is negative.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ClassicElements {
    /// Semi-major axis, upstream `a` (length). Negative for hyperbolas,
    /// zero for parabolas.
    pub semi_major_axis: f64,
    /// Eccentricity, upstream `e` (dimensionless).
    pub eccentricity: f64,
    /// Inclination of the orbit plane, upstream `i` (rad).
    pub inclination_rad: f64,
    /// Right ascension of the ascending node, upstream `Omega` (rad).
    pub raan_rad: f64,
    /// Argument of periapsis, upstream `omega` (rad).
    pub arg_periapsis_rad: f64,
    /// True anomaly, upstream `f` (rad); the eccentric or hyperbolic anomaly
    /// for rectilinear orbits.
    pub true_anomaly_rad: f64,
    /// Orbit radius `|r|`, upstream `rmag` (length). Output of
    /// [`state_to_elements`] only.
    pub orbit_radius: f64,
    /// Inverse semi-major axis `1/a = 2/r - v^2/mu`, upstream `alpha`
    /// (1/length). Output of [`state_to_elements`] only.
    pub inverse_semi_major_axis: f64,
    /// Periapsis radius, upstream `rPeriap` (length). Output of
    /// [`state_to_elements`]; input to [`elements_to_state`] for parabolas.
    pub periapsis_radius: f64,
    /// Apoapsis radius, upstream `rApoap` (length); negative for hyperbolas,
    /// zero for parabolas. Output of [`state_to_elements`] only.
    pub apoapsis_radius: f64,
}

/// Equinoctial orbit elements (non-singular for circular and equatorial
/// orbits). Port of upstream `equinoctialElements`; built by
/// [`classic_to_equinoctial`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EquinoctialElements {
    /// Semi-major axis, upstream `a` (length).
    pub semi_major_axis: f64,
    /// `e sin(omega + Omega)`, upstream `P1` (dimensionless).
    pub p1: f64,
    /// `e cos(omega + Omega)`, upstream `P2` (dimensionless).
    pub p2: f64,
    /// `tan(i/2) sin(Omega)`, upstream `Q1` (dimensionless).
    pub q1: f64,
    /// `tan(i/2) cos(Omega)`, upstream `Q2` (dimensionless).
    pub q2: f64,
    /// Mean longitude `Omega + omega + M`, upstream `l` (rad), not wrapped.
    pub mean_longitude_rad: f64,
    /// True longitude `Omega + omega + f`, upstream `L` (rad), not wrapped.
    pub true_longitude_rad: f64,
}

/// Direction of [`mean_osculating_map`]. Upstream passes it as the `sgn`
/// argument of `clMeanOscMap`, which scales the J2 perturbation: `+1` maps
/// mean to osculating elements, `-1` osculating to mean.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MeanOscDirection {
    /// Upstream `sgn = 1`: mean elements in, osculating elements out.
    MeanToOsculating,
    /// Upstream `sgn = -1`: osculating elements in, mean elements out.
    OsculatingToMean,
}

impl MeanOscDirection {
    /// The upstream `sgn` value: `1.0` or `-1.0`.
    pub const fn sign(self) -> f64 {
        match self {
            Self::MeanToOsculating => 1.0,
            Self::OsculatingToMean => -1.0,
        }
    }
}

/// Eccentric anomaly to true anomaly (rad). Port of upstream `E2f`.
///
/// Valid for circular and non-rectilinear elliptic orbits, `0 <= e < 1`;
/// otherwise logs an error and returns `NaN`.
pub fn eccentric_to_true_anomaly(ecc_anomaly_rad: f64, e: f64) -> f64 {
    if !is_elliptic(e) {
        log::error!("E2f() received e = {e}. The value of e should be 0 <= e < 1.");
        return f64::NAN;
    }
    2.0 * ((1.0 + e).sqrt() * (ecc_anomaly_rad / 2.0).sin())
        .atan2((1.0 - e).sqrt() * (ecc_anomaly_rad / 2.0).cos())
}

/// Eccentric anomaly to mean anomaly (rad) by Kepler's equation
/// `M = E - e sin E`. Port of upstream `E2M`.
///
/// Valid for `0 <= e < 1` (2D and 1D elliptic orbits); otherwise logs an
/// error and returns `NaN`.
pub fn eccentric_to_mean_anomaly(ecc_anomaly_rad: f64, e: f64) -> f64 {
    if !is_elliptic(e) {
        log::error!("E2M() received e = {e}. The value of e should be 0 <= e < 1.");
        return f64::NAN;
    }
    ecc_anomaly_rad - e * ecc_anomaly_rad.sin()
}

/// True anomaly to eccentric anomaly (rad). Port of upstream `f2E`.
///
/// Valid for circular and non-rectilinear elliptic orbits, `0 <= e < 1`;
/// otherwise logs an error and returns `NaN`.
pub fn true_to_eccentric_anomaly(true_anomaly_rad: f64, e: f64) -> f64 {
    if !is_elliptic(e) {
        log::error!("f2E() received e = {e}. The value of e should be 0 <= e < 1.");
        return f64::NAN;
    }
    2.0 * ((1.0 - e).sqrt() * (true_anomaly_rad / 2.0).sin())
        .atan2((1.0 + e).sqrt() * (true_anomaly_rad / 2.0).cos())
}

/// True anomaly to hyperbolic anomaly (rad). Port of upstream `f2H`.
///
/// Valid for hyperbolic orbits, `e > 1`; otherwise logs an error and
/// returns `NaN`.
pub fn true_to_hyperbolic_anomaly(true_anomaly_rad: f64, e: f64) -> f64 {
    if !is_hyperbolic(e) {
        log::error!("f2H() received e = {e}. The value of e should be 1 < e.");
        return f64::NAN;
    }
    2.0 * (((e - 1.0) / (e + 1.0)).sqrt() * (true_anomaly_rad / 2.0).tan()).atanh()
}

/// Hyperbolic anomaly to true anomaly (rad). Port of upstream `H2f`.
///
/// Valid for hyperbolic orbits, `e > 1`; otherwise logs an error and
/// returns `NaN`.
pub fn hyperbolic_to_true_anomaly(hyp_anomaly_rad: f64, e: f64) -> f64 {
    if !is_hyperbolic(e) {
        log::error!("H2f() received e = {e}. The value of e should be 1 < e.");
        return f64::NAN;
    }
    2.0 * (((e + 1.0) / (e - 1.0)).sqrt() * (hyp_anomaly_rad / 2.0).tanh()).atan()
}

/// Hyperbolic anomaly to mean hyperbolic anomaly (rad),
/// `N = e sinh H - H`. Port of upstream `H2N`.
///
/// Valid for hyperbolic orbits, `e > 1`; otherwise logs an error and
/// returns `NaN`.
pub fn hyperbolic_to_mean_hyperbolic_anomaly(hyp_anomaly_rad: f64, e: f64) -> f64 {
    if !is_hyperbolic(e) {
        log::error!("H2N() received e = {e}. The value of e should be 1 < e.");
        return f64::NAN;
    }
    e * hyp_anomaly_rad.sinh() - hyp_anomaly_rad
}

/// Mean anomaly to eccentric anomaly (rad), solving Kepler's equation by
/// Newton iteration from `E = M`. Port of upstream `M2E`.
///
/// Iterates until the update is at most `1e-13` rad; after 200 iterations it
/// logs an error and returns the current iterate. Valid for `0 <= e < 1`
/// (2D and 1D elliptic orbits); otherwise logs an error and returns `NaN`.
pub fn mean_to_eccentric_anomaly(mean_anomaly_rad: f64, e: f64) -> f64 {
    if !is_elliptic(e) {
        log::error!("M2E() received e = {e}. The value of e should be 0 <= e < 1.");
        return f64::NAN;
    }
    newton_solve(mean_anomaly_rad, |ecc_anomaly| {
        (ecc_anomaly - e * ecc_anomaly.sin() - mean_anomaly_rad) / (1.0 - e * ecc_anomaly.cos())
    })
    .unwrap_or_else(|last| {
        log::error!("iteration error in M2E({mean_anomaly_rad},{e})");
        last
    })
}

/// Mean hyperbolic anomaly to hyperbolic anomaly (rad), solving
/// `N = e sinh H - H` by Newton iteration. Port of upstream `N2H`.
///
/// The initial guess is `H = N`, clamped to `±7` rad. Iterates until the
/// update is at most `1e-13` rad; after 200 iterations it logs an error and
/// returns the current iterate. Valid for `e > 1`; otherwise logs an error
/// and returns `NaN`.
pub fn mean_hyperbolic_to_hyperbolic_anomaly(mean_hyp_anomaly_rad: f64, e: f64) -> f64 {
    if !is_hyperbolic(e) {
        log::error!("N2H() received e = {e}. The value of e should be e > 1.");
        return f64::NAN;
    }
    let n = mean_hyp_anomaly_rad;
    let initial_guess = if n.abs() > MAX_INITIAL_HYPERBOLIC_ANOMALY_RAD {
        // Upstream `N / fabs(N) * 7.0`, which (unlike copysign) is NaN for N = ±inf.
        n / n.abs() * MAX_INITIAL_HYPERBOLIC_ANOMALY_RAD
    } else {
        n
    };
    newton_solve(initial_guess, |hyp_anomaly| {
        (e * hyp_anomaly.sinh() - hyp_anomaly - n) / (e * hyp_anomaly.cosh() - 1.0)
    })
    .unwrap_or_else(|last| {
        log::error!("iteration error in N2H({n},{e})");
        last
    })
}

/// Classical elements to inertial position and velocity `(r_n, v_n)`. Port
/// of upstream `elem2rv`.
///
/// `mu` is the central body's gravitational parameter; the outputs are in its
/// length unit (and that unit per second). Handles, as upstream:
/// * circular `e = 0`, elliptic `0 < e < 1` and hyperbolic `e > 1`
///   (`a < 0`) orbits through the general branch;
/// * parabolas: `e = 1` with `|a| <= 1e-12`, using
///   `p = 2 * periapsis_radius`;
/// * rectilinear orbits: `|e - 1| < 1e-12` with `|a| > 1e-12`, where
///   `true_anomaly_rad` is the eccentric (`a > 0`) or hyperbolic (`a < 0`)
///   anomaly and the motion is along the periapsis direction, outbound when
///   `sin(anomaly) > 0`.
///
/// Only the six classical elements (plus `periapsis_radius` for parabolas)
/// are read.
pub fn elements_to_state(mu: f64, elements: &ClassicElements) -> (Vector3<f64>, Vector3<f64>) {
    let a = elements.semi_major_axis;
    let e = elements.eccentricity;
    let i = elements.inclination_rad;
    let raan = elements.raan_rad;
    let arg_periapsis = elements.arg_periapsis_rad;
    let f = elements.true_anomaly_rad;

    if (e - 1.0).abs() < SMALL_ELEMENT && a.abs() > SMALL_ELEMENT {
        // 1D rectilinear elliptic/hyperbolic orbit: f is the eccentric/hyperbolic anomaly.
        let anomaly = f;
        let r = if a > 0.0 {
            a * (1.0 - e * anomaly.cos())
        } else {
            a * (1.0 - e * anomaly.cosh())
        };
        let v = (2.0 * mu / r - mu / a).sqrt();
        let ir = Vector3::new(
            raan.cos() * arg_periapsis.cos() - raan.sin() * arg_periapsis.sin() * i.cos(),
            raan.sin() * arg_periapsis.cos() + raan.cos() * arg_periapsis.sin() * i.cos(),
            arg_periapsis.sin() * i.sin(),
        );
        let speed = if anomaly.sin() > 0.0 { v } else { -v };
        return (ir * r, ir * speed);
    }

    // General 2D orbit: semi-latus rectum from a, or from periapsis for parabolas.
    let p = if a.abs() > SMALL_ELEMENT {
        a * (1.0 - e * e)
    } else {
        2.0 * elements.periapsis_radius
    };
    let r = p / (1.0 + e * f.cos());
    let theta = arg_periapsis + f; // true latitude
    let h = (mu * p).sqrt();

    let (sin_raan, cos_raan) = raan.sin_cos();
    let (sin_theta, cos_theta) = theta.sin_cos();
    let (sin_i, cos_i) = i.sin_cos();
    let along_node = sin_theta + e * arg_periapsis.sin();
    let across_node = cos_theta + e * arg_periapsis.cos();

    let r_n = Vector3::new(
        r * (cos_raan * cos_theta - sin_raan * sin_theta * cos_i),
        r * (sin_raan * cos_theta + cos_raan * sin_theta * cos_i),
        r * (sin_theta * sin_i),
    );
    let v_n = Vector3::new(
        -mu / h * (cos_raan * along_node + sin_raan * across_node * cos_i),
        -mu / h * (sin_raan * along_node - cos_raan * across_node * cos_i),
        -mu / h * (-across_node * sin_i),
    );
    (r_n, v_n)
}

/// Inertial position and velocity to classical elements. Port of upstream
/// `rv2elem`; fills every field of [`ClassicElements`].
///
/// `mu` is the central body's gravitational parameter, in the units of `r_n`
/// and `v_n`. Degenerate cases, as upstream (all with tolerance `1e-12`):
/// * equatorial (`|z x h| < 1e-12`): the line of nodes is the inertial x axis,
///   so `raan_rad = 0` and `arg_periapsis_rad` is measured from x;
/// * circular (`e <= 1e-12`): periapsis is placed on the line of nodes, so
///   `arg_periapsis_rad = 0` and `true_anomaly_rad` is the argument of
///   latitude (true longitude when also equatorial);
/// * parabolic (`|1/a| <= 1e-12`): `semi_major_axis = 0` and
///   `apoapsis_radius = 0`.
///
/// `raan_rad`, `arg_periapsis_rad` and `true_anomaly_rad` are in `[0, 2 pi)`.
pub fn state_to_elements(mu: f64, r_n: Vector3<f64>, v_n: Vector3<f64>) -> ClassicElements {
    let n1_hat = Vector3::x();
    let n3_hat = Vector3::z();

    let r = r_n.norm();
    let v = v_n.norm();
    let ir_hat = normalize_or_zero(r_n);

    let h_vec = r_n.cross(&v_n);
    let h = h_vec.norm();
    let ih_hat = normalize_or_zero(h_vec);
    let p = h * h / mu;

    let node_vec = n3_hat.cross(&h_vec);
    let in_hat = if node_vec.norm() < SMALL_ELEMENT {
        n1_hat // near-equatorial orbit
    } else {
        normalize_or_zero(node_vec)
    };

    let e_vec = r_n * (v * v / mu - 1.0 / r) - v_n * (r_n.dot(&v_n) / mu);
    let e = e_vec.norm();
    let ie_hat = if e > SMALL_ELEMENT {
        normalize_or_zero(e_vec)
    } else {
        in_hat // near-circular orbit: periapsis on the line of nodes
    };

    let alpha = 2.0 / r - v * v / mu;
    let (a, r_apoapsis) = if alpha.abs() > SMALL_ELEMENT {
        (1.0 / alpha, p / (1.0 - e)) // elliptic or hyperbolic
    } else {
        (0.0, 0.0) // parabolic
    };

    let raan = n1_hat.cross(&in_hat).z.atan2(in_hat.x);
    let arg_periapsis = ih_hat
        .dot(&in_hat.cross(&ie_hat))
        .atan2(in_hat.dot(&ie_hat));
    let true_anomaly = ih_hat
        .dot(&ie_hat.cross(&ir_hat))
        .atan2(ie_hat.dot(&ir_hat));

    ClassicElements {
        semi_major_axis: a,
        eccentricity: e,
        inclination_rad: safe_acos(h_vec.z / h),
        raan_rad: wrap_negative_angle(raan),
        arg_periapsis_rad: wrap_negative_angle(arg_periapsis),
        true_anomaly_rad: wrap_negative_angle(true_anomaly),
        orbit_radius: r,
        inverse_semi_major_axis: alpha,
        periapsis_radius: p / (1.0 + e),
        apoapsis_radius: r_apoapsis,
    }
}

/// First-order J2 map between mean and osculating classical elements
/// (Brouwer/Kozai theory as given in Schaub & Junkins, *Analytical Mechanics of
/// Space Systems*, 4th ed., Appendix F, eqs. F.1–F.22). Port of upstream
/// `clMeanOscMap`.
///
/// `req` is the central body's equatorial radius, in the same unit as
/// `elements.semi_major_axis`; `j2` is its J2 coefficient. `direction` is
/// upstream's `sgn`: [`MeanOscDirection::MeanToOsculating`] (`+1`) or
/// [`MeanOscDirection::OsculatingToMean`] (`-1`).
///
/// The input must be elliptic (`0 <= e < 1`, else the result is `NaN`, with
/// errors logged). The map is singular at `i = 0` (`1/tan i`) and at the
/// critical inclination `cos^2 i = 1/5`. Only the six classical elements are
/// computed; upstream leaves the other output fields untouched, here they
/// are zero.
pub fn mean_osculating_map(
    req: f64,
    j2: f64,
    elements: &ClassicElements,
    direction: MeanOscDirection,
) -> ClassicElements {
    let a = elements.semi_major_axis;
    let e = elements.eccentricity;
    let i = elements.inclination_rad;
    let raan = elements.raan_rad;
    let w = elements.arg_periapsis_rad;
    let f = elements.true_anomaly_rad;
    let ecc_anomaly = true_to_eccentric_anomaly(f, e);
    let m = eccentric_to_mean_anomaly(ecc_anomaly, e);

    let gamma2 = direction.sign() * j2 / 2.0 * (req / a).powf(2.0); // (F.1), (F.2)
    let eta = (1.0 - e.powf(2.0)).sqrt();
    let gamma2p = gamma2 / eta.powf(4.0); // (F.3)
    let a_r = (1.0 + e * f.cos()) / eta.powf(2.0); // (F.6)

    let ci = i.cos();
    let ci2 = ci.powf(2.0);
    let ci4 = ci.powf(4.0);
    let ci6 = ci.powf(6.0);
    let si2 = 1.0 - ci2; // sin^2 i
    // 1 - 5 cos^2 i vanishes at the critical inclination.
    let critical = 1.0 - 5.0 * ci2;
    let (cf, sf) = (f.cos(), f.sin());
    let (sin_2w, cos_2w) = (2.0 * w).sin_cos();
    let w2_f = 2.0 * w + f;
    let w2_2f = 2.0 * w + 2.0 * f;
    let w2_3f = 2.0 * w + 3.0 * f;
    // Recurring brackets of (F.8)-(F.13).
    let kozai = 1.0 - 11.0 * ci2 - 40.0 * ci4 / critical;
    let node_bracket = 11.0 + 80.0 * ci2 / critical + 200.0 * ci4 / critical.powf(2.0);
    let center = f - m + e * sf; // equation of the center
    let short_period = 6.0 * center - 3.0 * w2_2f.sin() - 3.0 * e * w2_f.sin() - e * w2_3f.sin();

    let ap = a + a
        * gamma2
        * ((3.0 * ci2 - 1.0) * (a_r.powf(3.0) - 1.0 / eta.powf(3.0))
            + 3.0 * si2 * a_r.powf(3.0) * w2_2f.cos()); // (F.7)

    let de1 = gamma2p / 8.0 * e * eta.powf(2.0) * kozai * cos_2w; // (F.8)

    let de = de1
        + eta.powf(2.0) / 2.0
            * (gamma2
                * ((3.0 * ci2 - 1.0) / eta.powf(6.0)
                    * (e * eta
                        + e / (1.0 + eta)
                        + 3.0 * cf
                        + 3.0 * e * cf.powf(2.0)
                        + e.powf(2.0) * cf.powf(3.0))
                    + 3.0 * si2 / eta.powf(6.0)
                        * (e + 3.0 * cf + 3.0 * e * cf.powf(2.0) + e.powf(2.0) * cf.powf(3.0))
                        * w2_2f.cos())
                - gamma2p * si2 * (3.0 * w2_f.cos() + w2_3f.cos())); // (F.9)

    let di = -e * de1 / eta.powf(2.0) / i.tan()
        + gamma2p / 2.0
            * ci
            * si2.sqrt()
            * (3.0 * w2_2f.cos() + 3.0 * e * w2_f.cos() + e * w2_3f.cos()); // (F.10)

    let mean_lon_p = m + w + raan + gamma2p / 8.0 * eta.powf(3.0) * kozai * sin_2w
        - gamma2p / 16.0
            * (2.0 + e.powf(2.0)
                - 11.0 * (2.0 + 3.0 * e.powf(2.0)) * ci2
                - 40.0 * (2.0 + 5.0 * e.powf(2.0)) * ci4 / critical
                - 400.0 * e.powf(2.0) * ci6 / critical.powf(2.0))
            * sin_2w
        + gamma2p / 4.0
            * (-6.0 * critical * center
                + (3.0 - 5.0 * ci2) * (3.0 * w2_2f.sin() + 3.0 * e * w2_f.sin() + e * w2_3f.sin()))
        - gamma2p / 8.0 * e.powf(2.0) * ci * node_bracket * sin_2w
        - gamma2p / 2.0 * ci * short_period; // (F.11): M' + omega' + Omega'

    let are2 = (a_r * eta).powf(2.0);
    // (F.12): e dM
    let e_dm = gamma2p / 8.0 * e * eta.powf(3.0) * kozai * sin_2w
        - gamma2p / 4.0
            * eta.powf(3.0)
            * (2.0 * (3.0 * ci2 - 1.0) * (are2 + a_r + 1.0) * sf
                + 3.0
                    * si2
                    * ((-are2 - a_r + 1.0) * w2_f.sin() + (are2 + a_r + 1.0 / 3.0) * w2_3f.sin()));

    // (F.13)
    let d_raan = -gamma2p / 8.0 * e.powf(2.0) * ci * node_bracket * sin_2w
        - gamma2p / 2.0 * ci * short_period;

    let (sin_m, cos_m) = m.sin_cos();
    let d1 = (e + de) * sin_m + e_dm * cos_m; // (F.14)
    let d2 = (e + de) * cos_m - e_dm * sin_m; // (F.15)
    let mp = d1.atan2(d2); // (F.16)
    let ep = (d1 * d1 + d2 * d2).sqrt(); // (F.17)

    let (sin_half_i, cos_half_i) = (i / 2.0).sin_cos();
    let (sin_raan, cos_raan) = raan.sin_cos();
    let node_sin = sin_half_i + cos_half_i * di / 2.0;
    let d3 = node_sin * sin_raan + sin_half_i * d_raan * cos_raan; // (F.18)
    let d4 = node_sin * cos_raan - sin_half_i * d_raan * sin_raan; // (F.19)
    let raan_p = d3.atan2(d4); // (F.20)
    let ip = 2.0 * safe_asin((d3 * d3 + d4 * d4).sqrt()); // (F.21)
    let wp = mean_lon_p - mp - raan_p; // (F.22)

    let fp = eccentric_to_true_anomaly(mean_to_eccentric_anomaly(mp, ep), ep);

    ClassicElements {
        semi_major_axis: ap,
        eccentricity: ep,
        inclination_rad: ip,
        raan_rad: raan_p,
        arg_periapsis_rad: wp,
        true_anomaly_rad: fp,
        ..ClassicElements::default()
    }
}

/// Classical to equinoctial elements. Port of upstream `clElem2eqElem`.
///
/// The mean longitude goes through the elliptic anomaly conversions, so it
/// is `NaN` (with errors logged) unless `0 <= e < 1`. Longitudes are not
/// wrapped.
pub fn classic_to_equinoctial(elements: &ClassicElements) -> EquinoctialElements {
    let e = elements.eccentricity;
    let raan = elements.raan_rad;
    let longitude_of_periapsis = raan + elements.arg_periapsis_rad;
    let tan_half_i = (elements.inclination_rad / 2.0).tan();
    let ecc_anomaly = true_to_eccentric_anomaly(elements.true_anomaly_rad, e);
    let mean_anomaly = eccentric_to_mean_anomaly(ecc_anomaly, e);
    EquinoctialElements {
        semi_major_axis: elements.semi_major_axis,
        p1: e * longitude_of_periapsis.sin(),
        p2: e * longitude_of_periapsis.cos(),
        q1: tan_half_i * raan.sin(),
        q2: tan_half_i * raan.cos(),
        mean_longitude_rad: longitude_of_periapsis + mean_anomaly,
        true_longitude_rad: longitude_of_periapsis + elements.true_anomaly_rad,
    }
}

/// DCM `[HN]` from inertial to the Hill frame of a chief at `rc_n`, `vc_n`.
/// Port of upstream `hillFrame`.
///
/// Rows are the Hill axes in inertial components: radial `r/|r|`, along-track
/// `h x r`, and orbit normal `h/|h|` with `h = r x v`.
pub fn hill_frame(rc_n: Vector3<f64>, vc_n: Vector3<f64>) -> Matrix3<f64> {
    let ir_n = normalize_or_zero(rc_n);
    let ih_n = normalize_or_zero(rc_n.cross(&vc_n));
    let itheta_n = ih_n.cross(&ir_n);
    Matrix3::from_rows(&[ir_n.transpose(), itheta_n.transpose(), ih_n.transpose()])
}

/// Deputy Hill-frame state to deputy inertial state `(rd_n, vd_n)`. Port of
/// upstream `hill2rv`.
///
/// `rc_n`, `vc_n` are the chief's inertial position and velocity; `rho_h` and
/// `rho_prime_h` the deputy's position relative to the chief and its
/// Hill-frame time derivative, in Hill components. The Hill frame rotates at
/// the chief's true anomaly rate `|r x v| / r^2` about its normal.
pub fn hill_to_state(
    rc_n: Vector3<f64>,
    vc_n: Vector3<f64>,
    rho_h: Vector3<f64>,
    rho_prime_h: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let nh = hill_frame(rc_n, vc_n).transpose();
    let omega_hn_h = hill_frame_rate(rc_n, vc_n);
    let rd_n = rc_n + nh * rho_h;
    let vd_n = nh * (omega_hn_h.cross(&rho_h) + rho_prime_h) + vc_n;
    (rd_n, vd_n)
}

/// Deputy inertial state to deputy Hill-frame state `(rho_h, rho_prime_h)`.
/// Port of upstream `rv2hill`; the inverse of [`hill_to_state`].
///
/// `rho_h` is the deputy position relative to the chief and `rho_prime_h` its
/// time derivative as seen in the rotating Hill frame, both in Hill
/// components.
pub fn state_to_hill(
    rc_n: Vector3<f64>,
    vc_n: Vector3<f64>,
    rd_n: Vector3<f64>,
    vd_n: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let hn = hill_frame(rc_n, vc_n);
    let omega_hn_h = hill_frame_rate(rc_n, vc_n);
    let rho_h = hn * (rd_n - rc_n);
    let rho_dot_h = hn * (vd_n - vc_n);
    let rho_prime_h = rho_dot_h - omega_hn_h.cross(&rho_h);
    (rho_h, rho_prime_h)
}

/// Hill frame angular velocity relative to inertial, in Hill components:
/// `(0, 0, f_dot)` with `f_dot = |r x v| / r^2`.
fn hill_frame_rate(rc_n: Vector3<f64>, vc_n: Vector3<f64>) -> Vector3<f64> {
    let rc = rc_n.norm();
    let f_dot = rc_n.cross(&vc_n).norm() / rc / rc;
    Vector3::new(0.0, 0.0, f_dot)
}

/// Newton iteration `x -= step(x)` from `initial`, as upstream `M2E`/`N2H`
/// run it: stop once a step is no larger than [`ANOMALY_TOLERANCE_RAD`] (a
/// `NaN` step also stops, like upstream's `while (fabs(d) > small)`). After
/// [`MAX_ANOMALY_ITERATIONS`] steps upstream takes one more and gives up
/// whatever its size; that last iterate comes back as `Err`.
fn newton_solve(initial: f64, step: impl Fn(f64) -> f64) -> Result<f64, f64> {
    let mut x = initial;
    for _ in 0..MAX_ANOMALY_ITERATIONS {
        let dx = step(x);
        x -= dx;
        if dx.abs() <= ANOMALY_TOLERANCE_RAD || dx.is_nan() {
            return Ok(x);
        }
    }
    Err(x - step(x))
}

fn is_elliptic(e: f64) -> bool {
    (0.0..1.0).contains(&e)
}

fn is_hyperbolic(e: f64) -> bool {
    e > 1.0
}

/// Map an `atan2` result from `(-pi, pi]` to `[0, 2 pi)`, as upstream
/// `rv2elem` does for each angle.
fn wrap_negative_angle(angle: f64) -> f64 {
    if angle < 0.0 { angle + TAU } else { angle }
}

// Truth values are copied verbatim from upstream, digits included.
#[allow(clippy::excessive_precision)]
#[cfg(test)]
mod tests {
    //! Ports every orbital-element case of upstream
    //! `utilities/tests/test_orbitalMotion.cpp` (its `atmosphericDensity`,
    //! `debyeLength`, `atmosphericDrag`, `jPerturb` and `solarRad` cases test
    //! functions this port skips), and the orbital checks of
    //! `utilitiesSelfCheck/avsLibrarySelfCheck.c` (`testOrbitalAnomalies` at
    //! 1e-10, `testOrbitalHill` at 1e-4, the accuracies its pytest passes).
    //! Round trips cover what upstream does not.

    use super::*;
    use crate::utilities::astro_constants::{D2R, MU_EARTH};
    use crate::utilities::rigid_body_kinematics::wrap_to_pi;
    use std::f64::consts::PI;

    /// Upstream `orbitalElementsAccuracy`.
    const ORBITAL_ELEMENTS_ACCURACY: f64 = 1e-11;

    /// Upstream `EXPECT_NEAR`.
    #[track_caller]
    fn assert_near(first: f64, second: f64, tolerance: f64) {
        assert!(
            (first - second).abs() <= tolerance,
            "{first} vs {second}, tolerance {tolerance}"
        );
    }

    /// Upstream `EXPECT_NEAR_REL` (`isEqualRel`): the error is relative to
    /// `|first|`, and a zero `first` demands exact equality.
    #[track_caller]
    fn assert_near_rel(first: f64, second: f64, tolerance: f64) {
        assert!(
            first.is_finite() && second.is_finite(),
            "{first} vs {second}"
        );
        let equal = if first == 0.0 {
            first == second
        } else {
            (first - second).abs() / first.abs() <= tolerance
        };
        assert!(equal, "{first} vs {second}, relative tolerance {tolerance}");
    }

    /// Upstream `v3IsEqualRel`: every component error relative to `|v1|`.
    fn v3_is_equal_rel(v1: Vector3<f64>, v2: Vector3<f64>, accuracy: f64) -> bool {
        let norm = v1.norm();
        (v1 - v2).iter().all(|d| d.abs() / norm <= accuracy)
    }

    /// Upstream `EXPECT_VECTOR3_NEAR_REL`.
    #[track_caller]
    fn assert_vector3_near_rel(first: Vector3<f64>, second: Vector3<f64>, tolerance: f64) {
        assert!(
            v3_is_equal_rel(first, second, tolerance),
            "{first:?} vs {second:?}, relative tolerance {tolerance}"
        );
    }

    fn elements(a: f64, e: f64, i: f64, raan: f64, arg_periapsis: f64, f: f64) -> ClassicElements {
        ClassicElements {
            semi_major_axis: a,
            eccentricity: e,
            inclination_rad: i,
            raan_rad: raan,
            arg_periapsis_rad: arg_periapsis,
            true_anomaly_rad: f,
            ..ClassicElements::default()
        }
    }

    /// `elements` plus the extra fields the upstream fixtures also set
    /// (`alpha`, `rPeriap`, `rApoap`); `elem2rv` reads none of them here.
    fn elements_with_extras(
        a: f64,
        e: f64,
        i: f64,
        raan: f64,
        arg_periapsis: f64,
        f: f64,
    ) -> ClassicElements {
        ClassicElements {
            inverse_semi_major_axis: 1.0 / a,
            periapsis_radius: a * (1.0 - e),
            apoapsis_radius: a * (1.0 + e),
            ..elements(a, e, i, raan, arg_periapsis, f)
        }
    }

    /// Upstream fixture: elements and the truth state they map to.
    struct Fixture {
        elements: ClassicElements,
        r_n: Vector3<f64>,
        v_n: Vector3<f64>,
    }

    impl Fixture {
        #[track_caller]
        fn check_elem2rv(&self) {
            let (r_n, v_n) = elements_to_state(MU_EARTH, &self.elements);
            assert_vector3_near_rel(r_n, self.r_n, ORBITAL_ELEMENTS_ACCURACY);
            assert_vector3_near_rel(v_n, self.v_n, ORBITAL_ELEMENTS_ACCURACY);
        }

        fn rv2elem(&self) -> ClassicElements {
            state_to_elements(MU_EARTH, self.r_n, self.v_n)
        }
    }

    fn two_dimension_hyperbolic() -> Fixture {
        Fixture {
            elements: elements_with_extras(
                -7500.0,
                1.4,
                40.0 * D2R,
                133.0 * D2R,
                113.0 * D2R,
                23.0 * D2R,
            ),
            r_n: Vector3::new(319.013136281857, -2796.71958333493, 1404.6919948109),
            v_n: Vector3::new(15.3433051336115, -5.87012423567412, -6.05659420479213),
        }
    }

    fn two_dimension_parabolic() -> Fixture {
        Fixture {
            elements: ClassicElements {
                inverse_semi_major_axis: 0.0, // zero orbit energy, i.e. parabolic
                periapsis_radius: 7500.0,
                ..elements(0.0, 1.0, 40.0 * D2R, 133.0 * D2R, 113.0 * D2R, 123.0 * D2R)
            },
            r_n: Vector3::new(27862.6148209797, 795.70270010667, -17554.0435142669),
            v_n: Vector3::new(3.06499561197954, 2.21344887266898, -3.14760065404514),
        }
    }

    fn two_dimension_elliptical() -> Fixture {
        Fixture {
            elements: elements_with_extras(
                7500.0,
                0.5,
                40.0 * D2R,
                133.0 * D2R,
                113.0 * D2R,
                123.0 * D2R,
            ),
            r_n: Vector3::new(6538.3506963942027, 186.7227227879431, -4119.3008399778619),
            v_n: Vector3::new(1.4414106130924005, 5.588901415902356, -4.0828931566657038),
        }
    }

    fn non_circular_equatorial() -> Fixture {
        Fixture {
            elements: elements(
                7500.0,
                0.5,
                0.0 * D2R,
                133.0 * D2R,
                113.0 * D2R,
                123.0 * D2R,
            ),
            r_n: Vector3::new(7634.8714161163643, 1209.2448361913848, -0.0),
            v_n: Vector3::new(2.5282399359829868, 6.6023861555546057, -0.0),
        }
    }

    /// Half the `rv2elem` equatorial tolerance, upstream `eps2`.
    const NEAR_EQUATORIAL_INCLINATION_RAD: f64 = 1e-12 * 0.5;

    fn non_circular_near_equatorial() -> Fixture {
        Fixture {
            elements: elements(
                7500.0,
                0.5,
                NEAR_EQUATORIAL_INCLINATION_RAD,
                133.0 * D2R,
                113.0 * D2R,
                123.0 * D2R,
            ),
            r_n: Vector3::new(7634.87141611636, 1209.24483619139, -3.20424723337984e-09),
            v_n: Vector3::new(2.52823993598298, 6.60238615555461, -3.17592708317508e-12),
        }
    }

    fn circular_inclined() -> Fixture {
        Fixture {
            elements: elements(
                7500.0,
                0.0,
                40.0 * D2R,
                133.0 * D2R,
                113.0 * D2R,
                123.0 * D2R,
            ),
            r_n: Vector3::new(6343.7735859429586, 181.16597468085499, -3996.7130970223939),
            v_n: Vector3::new(-1.8379619466304487, 6.5499717954886121, -2.6203988553352131),
        }
    }

    fn circular_equatorial() -> Fixture {
        Fixture {
            elements: elements(7500.0, 0.0, 0.0, 133.0 * D2R, 113.0 * D2R, 123.0 * D2R),
            r_n: Vector3::new(7407.6625544635335, 1173.2584878017262, 0.0),
            v_n: Vector3::new(-1.1404354122910105, 7.2004258117414572, 0.0),
        }
    }

    fn circular_equatorial_retrograde() -> Fixture {
        Fixture {
            elements: elements(7500.0, 0.0, PI, 133.0 * D2R, 113.0 * D2R, 123.0 * D2R),
            r_n: Vector3::new(-1687.13290757899, -7307.77548588926, 0.0),
            v_n: Vector3::new(-7.10333318346184, 1.63993368302803, 0.0),
        }
    }

    #[test]
    fn matches_upstream_elem2rv_1d_eccentric() {
        let elements = elements_with_extras(
            7500.0,
            1.0,
            40.0 * D2R,
            133.0 * D2R,
            113.0 * D2R,
            23.0 * D2R,
        );
        let (r_n, v_n) = elements_to_state(MU_EARTH, &elements);
        let r2 = Vector3::new(-148.596902253492, -457.100381534593, 352.773096481799);
        let v2 = Vector3::new(-8.93065944520745, -27.4716886950712, 21.2016289595043);
        // Upstream's check, verbatim: as written it only constrains r.
        assert!(
            v3_is_equal_rel(r_n, r2, ORBITAL_ELEMENTS_ACCURACY)
                || !v3_is_equal_rel(v_n, v2, ORBITAL_ELEMENTS_ACCURACY)
        );
        // The velocity matches the upstream truth as well.
        assert_vector3_near_rel(v_n, v2, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_elem2rv_1d_hyperbolic() {
        let elements = elements_with_extras(
            -7500.0,
            1.0,
            40.0 * D2R,
            133.0 * D2R,
            113.0 * D2R,
            23.0 * D2R,
        );
        let (r_n, v_n) = elements_to_state(MU_EARTH, &elements);
        assert_vector3_near_rel(
            r_n,
            Vector3::new(-152.641873349816, -469.543156608544, 362.375968124408),
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_vector3_near_rel(
            v_n,
            Vector3::new(-9.17378720883851, -28.2195763820421, 21.7788208976681),
            ORBITAL_ELEMENTS_ACCURACY,
        );
    }

    #[test]
    fn rectilinear_motion_is_inbound_when_the_anomaly_sine_is_not_positive() {
        let outbound = elements(7500.0, 1.0, 0.7, 2.3, 2.0, 23.0 * D2R);
        let inbound = ClassicElements {
            true_anomaly_rad: -23.0 * D2R,
            ..outbound
        };
        let (r_out, v_out) = elements_to_state(MU_EARTH, &outbound);
        let (r_in, v_in) = elements_to_state(MU_EARTH, &inbound);
        assert_eq!(r_out, r_in);
        assert_eq!(v_out, -v_in);
        assert!(r_out.dot(&v_out) > 0.0);
    }

    #[test]
    fn matches_upstream_two_dimension_hyperbolic() {
        let fixture = two_dimension_hyperbolic();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, -7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.eccentricity, 1.4, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.inclination_rad,
            40.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.raan_rad, 133.0 * D2R, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.arg_periapsis_rad,
            113.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.true_anomaly_rad,
            23.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.orbit_radius,
            3145.881340612725,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.periapsis_radius, 3000.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.apoapsis_radius,
            -18000.0,
            ORBITAL_ELEMENTS_ACCURACY,
        );
    }

    #[test]
    fn matches_upstream_two_dimension_parabolic() {
        let fixture = two_dimension_parabolic();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near(
            elements.inverse_semi_major_axis,
            0.0,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(elements.semi_major_axis, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.eccentricity, 1.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.inclination_rad,
            40.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.raan_rad, 133.0 * D2R, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.arg_periapsis_rad,
            113.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.true_anomaly_rad,
            123.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.orbit_radius,
            32940.89997480352,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.periapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 0.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_two_dimension_elliptical() {
        let fixture = two_dimension_elliptical();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.eccentricity, 0.5, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.inclination_rad,
            40.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.raan_rad, 133.0 * D2R, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.arg_periapsis_rad,
            113.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.true_anomaly_rad,
            123.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.orbit_radius,
            7730.041048693483,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.periapsis_radius, 3750.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 11250.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_non_circular_equatorial() {
        let fixture = non_circular_equatorial();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, 7500.00, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.eccentricity, 0.5, ORBITAL_ELEMENTS_ACCURACY);
        // A zero reference demands exact zeros: the equatorial branch must
        // produce i = 0 and Omega = 0 exactly.
        assert_near_rel(elements.inclination_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.raan_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.arg_periapsis_rad,
            (133.0 + 113.0) * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.true_anomaly_rad,
            123.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.orbit_radius,
            7730.041048693483,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.periapsis_radius, 3750.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 11250.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    /// Upstream has this case twice, as `NonCircularNearEquitorial` and
    /// `NonCircularNearEquitorial180Degree`, with identical data and checks.
    #[test]
    fn matches_upstream_non_circular_near_equatorial() {
        let fixture = non_circular_near_equatorial();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, 7500.00, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.eccentricity, 0.5, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(
            elements.inclination_rad,
            NEAR_EQUATORIAL_INCLINATION_RAD,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            wrap_to_pi(elements.raan_rad + elements.arg_periapsis_rad),
            (133.0 + 113.0 - 360.0) * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.true_anomaly_rad,
            123.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements.orbit_radius,
            7730.041048693483,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.periapsis_radius, 3750.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 11250.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_circular_inclined() {
        let fixture = circular_inclined();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, 7500.00, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.eccentricity, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.inclination_rad,
            40.0 * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.raan_rad, 133.0 * D2R, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            wrap_to_pi(elements.arg_periapsis_rad + elements.true_anomaly_rad),
            (113.0 + 123.0 - 360.0) * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.orbit_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.periapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_circular_equatorial() {
        let fixture = circular_equatorial();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, 7500.00, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.eccentricity, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.inclination_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.raan_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.arg_periapsis_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.true_anomaly_rad,
            (133.0 + 113.0 + 123.0 - 360.0) * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.orbit_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.periapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_circular_equatorial_retrograde() {
        let fixture = circular_equatorial_retrograde();
        fixture.check_elem2rv();

        let elements = fixture.rv2elem();
        assert_near_rel(elements.semi_major_axis, 7500.00, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.eccentricity, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.inclination_rad, PI, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.raan_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near(elements.arg_periapsis_rad, 0.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(
            elements.true_anomaly_rad,
            (-133.0 + 113.0 + 123.0) * D2R,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(elements.orbit_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.periapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
        assert_near_rel(elements.apoapsis_radius, 7500.0, ORBITAL_ELEMENTS_ACCURACY);
    }

    #[test]
    fn matches_upstream_classic_elements_to_mean_elements() {
        let elements = elements(1000.0, 0.2, 0.2, 0.15, 0.5, 0.2);
        let req = 300.0;
        let j2 = 1e-3;
        let elements_p =
            mean_osculating_map(req, j2, &elements, MeanOscDirection::MeanToOsculating);
        assert_near_rel(
            elements_p.semi_major_axis,
            1000.07546442015950560744386166334152,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(
            elements_p.eccentricity,
            0.20017786852908628358882481279579,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements_p.inclination_rad,
            0.20000333960738947425284095515963,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(
            elements_p.raan_rad,
            0.15007256499303692209856819772540,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(
            elements_p.arg_periapsis_rad,
            0.50011857315729335571319325026707,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements_p.true_anomaly_rad,
            0.19982315726261962174348241205735,
            ORBITAL_ELEMENTS_ACCURACY,
        );
    }

    #[test]
    fn matches_upstream_classic_elements_to_equinoctial_elements() {
        let elements_eq = classic_to_equinoctial(&elements(1000.0, 0.2, 0.2, 0.15, 0.5, 0.2));
        assert_near_rel(
            elements_eq.semi_major_axis,
            1000.00000000000000000000000000000000,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(
            elements_eq.p1,
            0.12103728114720790909331071816268,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements_eq.p2,
            0.15921675970981119530023306651856,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(
            elements_eq.q1,
            0.01499382601880069713906618034116,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near(
            elements_eq.q2,
            0.09920802187229026125603326136115,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements_eq.mean_longitude_rad,
            0.78093005232114087732497864635661,
            ORBITAL_ELEMENTS_ACCURACY,
        );
        assert_near_rel(
            elements_eq.true_longitude_rad,
            0.85000000000000008881784197001252,
            ORBITAL_ELEMENTS_ACCURACY,
        );
    }

    #[test]
    fn matches_upstream_self_check_orbital_anomalies() {
        let accuracy = 1e-10;
        let cases: [(&str, f64, f64); 9] = [
            (
                "E2f",
                eccentric_to_true_anomaly(0.3, 0.1),
                0.33111382522243943,
            ),
            (
                "E2f",
                eccentric_to_true_anomaly(0.3 + PI, 0.1),
                3.413322139966247,
            ),
            (
                "E2M",
                eccentric_to_mean_anomaly(0.3 + PI, 0.1),
                3.471144674255927,
            ),
            (
                "f2E",
                true_to_eccentric_anomaly(0.3, 0.1),
                0.2717294863764543,
            ),
            (
                "f2H",
                true_to_hyperbolic_anomaly(0.3, 2.1),
                0.18054632550895094,
            ),
            (
                "H2f",
                hyperbolic_to_true_anomaly(0.3, 2.1),
                0.4898441475256363,
            ),
            (
                "H2N",
                hyperbolic_to_mean_hyperbolic_anomaly(0.3, 2.1),
                0.33949261623899946,
            ),
            (
                "M2E",
                mean_to_eccentric_anomaly(2.0, 0.3),
                2.2360314951724365,
            ),
            (
                "N2H",
                mean_hyperbolic_to_hyperbolic_anomaly(2.0, 2.3),
                1.1098189302932016,
            ),
        ];
        for (name, actual, expected) in cases {
            assert!(
                (actual - expected).abs() <= accuracy,
                "{name}: {actual} vs {expected}"
            );
        }
    }

    #[test]
    fn matches_upstream_self_check_orbital_hill() {
        let accuracy = 1e-4;
        let rc_n = Vector3::new(353.38362479494975, 6494.841478640714, 2507.239669788398);
        let vc_n = Vector3::new(-7.073840333019544, -0.5666429544308719, 2.6565522055197555);
        let rho_h = Vector3::new(-0.286371, 0.012113, 0.875157);
        let rho_prime_h = Vector3::new(0.000689358, 0.000620362, 0.000927434);
        let rd_n = Vector3::new(353.6672082996106, 6494.264242564805, 2507.898786238764);
        let vd_n = Vector3::new(-7.073766857682589, -0.5663665778237081, 2.65770594819381);

        let hn = hill_frame(rc_n, vc_n);
        let hn_true = Matrix3::new(
            0.0506938, 0.931702, 0.35967, //
            -0.93404, -0.0832604, 0.347329, //
            0.353553, -0.353553, 0.866025,
        );
        for row in 0..3 {
            assert!(
                v3_is_equal_rel(
                    hn.row(row).transpose(),
                    hn_true.row(row).transpose(),
                    accuracy
                ),
                "hillFrame row {row}"
            );
        }

        let (rd, vd) = hill_to_state(rc_n, vc_n, rho_h, rho_prime_h);
        assert!(v3_is_equal_rel(rd, rd_n, accuracy), "hill2rv rd_N");
        assert!(v3_is_equal_rel(vd, vd_n, accuracy), "hill2rv vd_N");

        let (rho, rho_prime) = state_to_hill(rc_n, vc_n, rd_n, vd_n);
        assert!(v3_is_equal_rel(rho, rho_h, accuracy), "rv2hill rho_H");
        assert!(
            v3_is_equal_rel(rho_prime, rho_prime_h, accuracy),
            "rv2hill rhoPrime_H"
        );
    }

    /// elements -> state -> elements -> state reproduces the state for every
    /// orbit type `rv2elem` handles, degenerate angle conventions included.
    #[test]
    fn state_round_trips_through_elements_for_every_orbit_type() {
        let cases = [
            ("elliptic", elements(7000.0, 0.1, 0.2, 0.3, -0.4, 0.5)),
            (
                "high eccentricity",
                elements(42000.0, 0.95, 1.1, 4.0, 2.5, 3.0),
            ),
            ("hyperbolic", two_dimension_hyperbolic().elements),
            ("parabolic", two_dimension_parabolic().elements),
            ("circular inclined", circular_inclined().elements),
            ("elliptic equatorial", non_circular_equatorial().elements),
            ("circular equatorial", circular_equatorial().elements),
            (
                "circular retrograde",
                circular_equatorial_retrograde().elements,
            ),
            (
                "elliptic retrograde",
                elements(9000.0, 0.3, PI, 1.0, 2.0, 3.0),
            ),
            ("polar", elements(7000.0, 0.01, 0.5 * PI, 5.0, 1.0, 6.0)),
        ];
        for (name, original) in cases {
            let (r_n, v_n) = elements_to_state(MU_EARTH, &original);
            let recovered = state_to_elements(MU_EARTH, r_n, v_n);
            let (r2_n, v2_n) = elements_to_state(MU_EARTH, &recovered);
            assert!(
                v3_is_equal_rel(r2_n, r_n, 1e-12),
                "{name}: {r2_n:?} vs {r_n:?}"
            );
            assert!(
                v3_is_equal_rel(v2_n, v_n, 1e-12),
                "{name}: {v2_n:?} vs {v_n:?}"
            );
        }
    }

    /// Non-degenerate orbits recover every element, angles in `[0, 2 pi)`.
    #[test]
    fn elements_round_trip_through_state_for_elliptic_and_hyperbolic_orbits() {
        let cases = [
            elements(7000.0, 0.1, 0.2, 0.3, -0.4, 0.5),
            elements(
                26560.0,
                0.7,
                63.4 * D2R,
                350.0 * D2R,
                270.0 * D2R,
                200.0 * D2R,
            ),
            elements(-12000.0, 2.5, 2.8, 1.2, 0.3, -1.0),
            two_dimension_hyperbolic().elements,
        ];
        let angle_error = |a: f64, b: f64| wrap_to_pi(a - b).abs();
        for original in cases {
            let (r_n, v_n) = elements_to_state(MU_EARTH, &original);
            let recovered = state_to_elements(MU_EARTH, r_n, v_n);
            let a = original.semi_major_axis;
            let e = original.eccentricity;
            assert_near_rel(recovered.semi_major_axis, a, 1e-10);
            assert_near(recovered.eccentricity, e, 1e-12);
            assert_near(recovered.inclination_rad, original.inclination_rad, 1e-12);
            for (got, want) in [
                (recovered.raan_rad, original.raan_rad),
                (recovered.arg_periapsis_rad, original.arg_periapsis_rad),
                (recovered.true_anomaly_rad, original.true_anomaly_rad),
            ] {
                assert!((0.0..TAU).contains(&got), "{got} outside [0, 2 pi)");
                assert!(angle_error(got, want) < 1e-11, "{got} vs {want}");
            }
            assert_near_rel(recovered.orbit_radius, r_n.norm(), 1e-15);
            assert_near_rel(recovered.inverse_semi_major_axis, 1.0 / a, 1e-10);
            assert_near_rel(recovered.periapsis_radius, a * (1.0 - e), 1e-10);
            assert_near_rel(recovered.apoapsis_radius, a * (1.0 + e), 1e-10);
        }
    }

    #[test]
    fn elliptic_anomaly_conversions_invert_each_other() {
        for e in [0.0, 0.1, 0.5, 0.9, 0.99] {
            for k in -30..=30 {
                let angle = f64::from(k) * 0.1; // within (-pi, pi)
                let f = eccentric_to_true_anomaly(angle, e);
                assert_near(true_to_eccentric_anomaly(f, e), angle, 1e-12);
                let m = eccentric_to_mean_anomaly(angle, e);
                let ecc = mean_to_eccentric_anomaly(m, e);
                assert_near(ecc, angle, 1e-12);
                assert_near(ecc - e * ecc.sin(), m, 1e-13);
            }
        }
    }

    #[test]
    fn hyperbolic_anomaly_conversions_invert_each_other() {
        for e in [1.01_f64, 1.4, 2.1, 10.0] {
            let max_true_anomaly = (-1.0 / e).acos();
            for k in -9..=9 {
                let f = f64::from(k) / 10.0 * max_true_anomaly;
                let h = true_to_hyperbolic_anomaly(f, e);
                assert_near(hyperbolic_to_true_anomaly(h, e), f, 1e-9);
            }
            // N2H clamps its initial guess to +/-7 rad, so large anomalies
            // converge as well.
            for h in [-12.0, -6.5, -2.0, -0.3, 0.0, 0.3, 2.0, 6.5, 12.0] {
                let n = hyperbolic_to_mean_hyperbolic_anomaly(h, e);
                let recovered = mean_hyperbolic_to_hyperbolic_anomaly(n, e);
                assert_near_rel(recovered, h, 1e-12);
                assert_near_rel(e * recovered.sinh() - recovered, n, 1e-12);
            }
        }
    }

    #[test]
    fn anomaly_conversions_return_nan_outside_their_eccentricity_range() {
        for e in [-0.1, 1.0, 1.5, f64::NAN] {
            assert!(eccentric_to_true_anomaly(0.3, e).is_nan());
            assert!(eccentric_to_mean_anomaly(0.3, e).is_nan());
            assert!(true_to_eccentric_anomaly(0.3, e).is_nan());
            assert!(mean_to_eccentric_anomaly(0.3, e).is_nan());
        }
        for e in [0.0, 0.5, 1.0, f64::NAN] {
            assert!(true_to_hyperbolic_anomaly(0.3, e).is_nan());
            assert!(hyperbolic_to_true_anomaly(0.3, e).is_nan());
            assert!(hyperbolic_to_mean_hyperbolic_anomaly(0.3, e).is_nan());
            assert!(mean_hyperbolic_to_hyperbolic_anomaly(0.3, e).is_nan());
        }
        // A hyperbolic orbit has no mean anomaly, so the mean longitude is NaN.
        let hyperbolic = classic_to_equinoctial(&two_dimension_hyperbolic().elements);
        assert!(hyperbolic.mean_longitude_rad.is_nan());
        assert!(hyperbolic.true_longitude_rad.is_finite());
    }

    #[test]
    fn mean_osculating_map_directions_are_inverse_to_first_order_in_j2() {
        use crate::utilities::astro_constants::{J2_EARTH, REQ_EARTH};
        assert_eq!(MeanOscDirection::MeanToOsculating.sign(), 1.0);
        assert_eq!(MeanOscDirection::OsculatingToMean.sign(), -1.0);

        let mean = elements(7000.0, 0.05, 0.9, 1.0, 2.0, 0.5);
        let osculating = mean_osculating_map(
            REQ_EARTH,
            J2_EARTH,
            &mean,
            MeanOscDirection::MeanToOsculating,
        );
        let recovered = mean_osculating_map(
            REQ_EARTH,
            J2_EARTH,
            &osculating,
            MeanOscDirection::OsculatingToMean,
        );
        // The map shifts the orbit by O(J2) ~ 1e-3; the round trip only keeps
        // an O(J2^2) residual.
        assert!((osculating.semi_major_axis - mean.semi_major_axis).abs() > 1e-1);
        assert_near_rel(recovered.semi_major_axis, mean.semi_major_axis, 1e-6);
        assert_near(recovered.eccentricity, mean.eccentricity, 1e-6);
        assert_near(recovered.inclination_rad, mean.inclination_rad, 1e-6);
        assert_near(recovered.raan_rad, mean.raan_rad, 1e-6);
        assert_near(
            wrap_to_pi(recovered.arg_periapsis_rad + recovered.true_anomaly_rad),
            wrap_to_pi(mean.arg_periapsis_rad + mean.true_anomaly_rad),
            1e-6,
        );
        // Only the six classical elements are produced.
        assert_eq!(osculating.orbit_radius, 0.0);
        assert_eq!(osculating.periapsis_radius, 0.0);

        // With no J2 the map is the identity (up to the anomaly round trip).
        let identity =
            mean_osculating_map(REQ_EARTH, 0.0, &mean, MeanOscDirection::OsculatingToMean);
        assert_near_rel(identity.semi_major_axis, mean.semi_major_axis, 1e-15);
        assert_near(identity.eccentricity, mean.eccentricity, 1e-15);
        assert_near(identity.inclination_rad, mean.inclination_rad, 1e-14);
        assert_near(identity.raan_rad, mean.raan_rad, 1e-14);
        assert_near(identity.arg_periapsis_rad, mean.arg_periapsis_rad, 1e-12);
        assert_near(identity.true_anomaly_rad, mean.true_anomaly_rad, 1e-12);
    }

    #[test]
    fn hill_frame_is_a_proper_rotation_and_hill_states_round_trip() {
        let (rc_n, vc_n) = elements_to_state(MU_EARTH, &two_dimension_elliptical().elements);
        let hn = hill_frame(rc_n, vc_n);
        assert!((hn * hn.transpose() - Matrix3::identity()).norm() < 1e-15);
        assert_near(hn.determinant(), 1.0, 1e-15);
        assert!((hn * rc_n - Vector3::x() * rc_n.norm()).norm() < 1e-9);
        assert_near((hn * vc_n).z, 0.0, 1e-12);

        let rho_h = Vector3::new(1.2, -3.4, 0.5);
        let rho_prime_h = Vector3::new(-0.001, 0.002, 0.0005);
        let (rd_n, vd_n) = hill_to_state(rc_n, vc_n, rho_h, rho_prime_h);
        let (rho, rho_prime) = state_to_hill(rc_n, vc_n, rd_n, vd_n);
        assert!((rho - rho_h).norm() < 1e-9);
        assert!((rho_prime - rho_prime_h).norm() < 1e-12);

        // A deputy co-located with the chief has no relative state.
        let (rho, rho_prime) = state_to_hill(rc_n, vc_n, rc_n, vc_n);
        assert_eq!(rho, Vector3::zeros());
        assert_eq!(rho_prime, Vector3::zeros());
    }
}
