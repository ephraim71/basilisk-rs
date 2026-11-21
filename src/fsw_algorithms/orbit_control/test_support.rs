//! Test-only ports of the Python helpers the upstream `orbitControl` unit
//! tests build their truth with: SciPy's `brentq`, bounded `minimize_scalar`
//! and `hyp2f1`, the `lagrange_time`/`battin_time` flight-time evaluators, the
//! `IzzoLambert.py` reference solver, and `orbitalMotion.elem2rv_parab`.
//!
//! Upstream integrates the two-body equations with SciPy's `solve_ivp`
//! (DOP853, rtol 2e-12) to check a Lambert solution; [`kepler_propagate`]
//! propagates analytically with universal variables instead, which is
//! independent of the Lambert solver and more accurate.

use std::f64::consts::PI;

use hifitime::Epoch;
use nalgebra::Vector3;

use super::lambert_solver::{LambertSolver, LambertSolverConfig};
use crate::messages::{
    LambertPerformanceMsg, LambertProblemMsg, LambertSolutionMsg, LambertSolverMethod,
    LambertTransferDirection, Output,
};
use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
use crate::{Module, SimulationContext};

/// Upstream `MU` of the solver tests [m^3/s^2].
pub(crate) const MU: f64 = 3.986004360e14;
/// Upstream `RADIUS` of the solver tests \[m\].
pub(crate) const RADIUS: f64 = 8.0e6;
/// Both upstream solver methods, Gooding first as upstream parametrizes.
pub(crate) const METHODS: [LambertSolverMethod; 2] =
    [LambertSolverMethod::Gooding, LambertSolverMethod::Izzo];
/// Both transfer directions.
pub(crate) const DIRECTIONS: [LambertTransferDirection; 2] = [
    LambertTransferDirection::Prograde,
    LambertTransferDirection::Retrograde,
];

pub(crate) fn context(current_sim_nanos: u64) -> SimulationContext {
    SimulationContext {
        current_sim_nanos,
        current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
    }
}

/// `-1` for retrograde, `1` for prograde: the sign of the transfer's
/// angular-momentum z-component.
pub(crate) fn direction_sign(direction: LambertTransferDirection) -> f64 {
    match direction {
        LambertTransferDirection::Prograde => 1.0,
        LambertTransferDirection::Retrograde => -1.0,
    }
}

/// The problem upstream's `solve_problem` writes.
pub(crate) fn problem(
    r1: Vector3<f64>,
    r2: Vector3<f64>,
    transfer_time_s: f64,
    revolutions: usize,
    method: LambertSolverMethod,
    direction: LambertTransferDirection,
) -> LambertProblemMsg {
    LambertProblemMsg {
        solver_method: method,
        r1_n_m: r1,
        r2_n_m: r2,
        transfer_time_s,
        mu_m3ps2: MU,
        num_revolutions: revolutions,
        transfer_direction: direction,
    }
}

/// Upstream `solve_problem`: one reset and update of `module` on a freshly
/// written problem.
pub(crate) fn solve_with(
    module: &mut LambertSolver,
    problem: LambertProblemMsg,
) -> (LambertSolutionMsg, LambertPerformanceMsg) {
    let source = Output::new(problem);
    source.connect_to(&mut module.lambert_problem_in_msg);
    module.reset(&context(0));
    module.update(&context(0));
    (
        module.lambert_solution_out_msg.read(),
        module.lambert_performance_out_msg.read(),
    )
}

/// [`solve_with`] on a new module with default settings.
pub(crate) fn solve_problem(
    r1: Vector3<f64>,
    r2: Vector3<f64>,
    transfer_time_s: f64,
    revolutions: usize,
    method: LambertSolverMethod,
    direction: LambertTransferDirection,
) -> (LambertSolutionMsg, LambertPerformanceMsg) {
    let mut module = LambertSolver::new(LambertSolverConfig::default());
    module.init();
    solve_with(
        &mut module,
        problem(r1, r2, transfer_time_s, revolutions, method, direction),
    )
}

/// Upstream `lagrange_time`: elliptic flight time from Lagrange's
/// eccentric-angle formulation.
pub(crate) fn lagrange_time(x: f64, lambda: f64, revolutions: usize) -> f64 {
    let axis = 1.0 / (1.0 - x * x);
    let alpha = 2.0 * x.acos();
    let beta = 2.0 * (lambda / axis.sqrt()).asin();
    axis.powf(1.5) * (alpha - alpha.sin() - beta + beta.sin() + 2.0 * PI * revolutions as f64) / 2.0
}

/// SciPy's `hyp2f1(a, b, c, z)` for `z < 1`: Gauss's series, after Pfaff's
/// transformation `(1 - z)^-b 2F1(c - a, b; c; z / (z - 1))` for
/// `z < -1/2`, summed until the terms stop changing the sum.
pub(crate) fn hyp2f1(a: f64, b: f64, c: f64, z: f64) -> f64 {
    assert!(
        z < 1.0,
        "hyp2f1 is only needed below its branch point, not at {z}"
    );
    if z < -0.5 {
        return (1.0 - z).powf(-b) * hyp2f1(c - a, b, c, z / (z - 1.0));
    }
    let mut sum = 1.0;
    let mut term = 1.0;
    for j in 0..100_000 {
        let j = f64::from(j);
        term *= (a + j) * (b + j) / ((c + j) * (j + 1.0)) * z;
        sum += term;
        if term.abs() <= 1e-18 * sum.abs() {
            return sum;
        }
    }
    panic!("hyp2f1 series did not converge at z = {z}");
}

/// Upstream `battin_time`: zero-revolution flight time from Battin's
/// hypergeometric form, smooth through the parabola.
pub(crate) fn battin_time(x: f64, lambda: f64) -> f64 {
    let y = (1.0 - lambda * lambda * (1.0 - x * x)).sqrt();
    let eta = y - lambda * x;
    (eta.powi(3) * 4.0 / 3.0 * hyp2f1(3.0, 1.0, 2.5, (1.0 - lambda - x * eta) / 2.0)
        + 4.0 * lambda * eta)
        / 2.0
}

/// SciPy's `brentq(f, a, b, xtol)` with its default `rtol = 4 eps` and 100
/// iterations: a port of SciPy's `brentq.c`.
pub(crate) fn brentq(f: impl Fn(f64) -> f64, a: f64, b: f64, xtol: f64) -> f64 {
    let rtol = 4.0 * f64::EPSILON;
    let (mut x_pre, mut x_cur) = (a, b);
    let (mut f_pre, mut f_cur) = (f(x_pre), f(x_cur));
    let (mut x_blk, mut f_blk, mut s_pre, mut s_cur) = (0.0, 0.0, 0.0, 0.0);
    assert!(
        f_pre * f_cur <= 0.0,
        "brentq: f(a) and f(b) must differ in sign"
    );
    if f_pre == 0.0 {
        return x_pre;
    }
    if f_cur == 0.0 {
        return x_cur;
    }
    for _ in 0..100 {
        if f_pre != 0.0 && f_cur != 0.0 && (f_pre.is_sign_negative() != f_cur.is_sign_negative()) {
            x_blk = x_pre;
            f_blk = f_pre;
            s_pre = x_cur - x_pre;
            s_cur = s_pre;
        }
        if f_blk.abs() < f_cur.abs() {
            x_pre = x_cur;
            x_cur = x_blk;
            x_blk = x_pre;
            f_pre = f_cur;
            f_cur = f_blk;
            f_blk = f_pre;
        }
        let delta = (xtol + rtol * x_cur.abs()) / 2.0;
        let s_bis = (x_blk - x_cur) / 2.0;
        if f_cur == 0.0 || s_bis.abs() < delta {
            return x_cur;
        }
        if s_pre.abs() > delta && f_cur.abs() < f_pre.abs() {
            let s_try = if x_pre == x_blk {
                -f_cur * (x_cur - x_pre) / (f_cur - f_pre)
            } else {
                let d_pre = (f_pre - f_cur) / (x_pre - x_cur);
                let d_blk = (f_blk - f_cur) / (x_blk - x_cur);
                -f_cur * (f_blk * d_blk - f_pre * d_pre) / (d_blk * d_pre * (f_blk - f_pre))
            };
            if 2.0 * s_try.abs() < s_pre.abs().min(3.0 * s_bis.abs() - delta) {
                s_pre = s_cur;
                s_cur = s_try;
            } else {
                s_pre = s_bis;
                s_cur = s_bis;
            }
        } else {
            s_pre = s_bis;
            s_cur = s_bis;
        }
        x_pre = x_cur;
        f_pre = f_cur;
        x_cur += if s_cur.abs() > delta {
            s_cur
        } else if s_bis > 0.0 {
            delta
        } else {
            -delta
        };
        f_cur = f(x_cur);
    }
    panic!("brentq did not converge");
}

/// SciPy's `minimize_scalar(f, bounds=(a, b), method="bounded",
/// options={"xatol": xatol})` (Brent's bounded minimization, 500
/// evaluations): `(x_min, f(x_min))`.
pub(crate) fn minimize_bounded(f: impl Fn(f64) -> f64, a: f64, b: f64, xatol: f64) -> (f64, f64) {
    let sqrt_eps = 2.2e-16_f64.sqrt();
    let golden_mean = 0.5 * (3.0 - 5.0_f64.sqrt());
    let (mut a, mut b) = (a, b);
    let mut fulc = a + golden_mean * (b - a);
    let (mut nfc, mut xf) = (fulc, fulc);
    let (mut rat, mut e) = (0.0_f64, 0.0_f64);
    let mut fx = f(xf);
    let mut evaluations = 1;
    let (mut ffulc, mut fnfc) = (fx, fx);
    let mut xm = 0.5 * (a + b);
    let mut tol1 = sqrt_eps * xf.abs() + xatol / 3.0;
    let mut tol2 = 2.0 * tol1;
    let sign = |value: f64| {
        if value > 0.0 {
            1.0
        } else if value < 0.0 {
            -1.0
        } else {
            0.0
        }
    };

    while (xf - xm).abs() > tol2 - 0.5 * (b - a) {
        let mut golden = true;
        if e.abs() > tol1 {
            golden = false;
            let mut r = (xf - nfc) * (fx - ffulc);
            let mut q = (xf - fulc) * (fx - fnfc);
            let mut p = (xf - fulc) * q - (xf - nfc) * r;
            q = 2.0 * (q - r);
            if q > 0.0 {
                p = -p;
            }
            q = q.abs();
            r = e;
            e = rat;
            if p.abs() < (0.5 * q * r).abs() && p > q * (a - xf) && p < q * (b - xf) {
                rat = p / q;
                let x = xf + rat;
                if (x - a) < tol2 || (b - x) < tol2 {
                    let si = sign(xm - xf) + if xm - xf == 0.0 { 1.0 } else { 0.0 };
                    rat = tol1 * si;
                }
            } else {
                golden = true;
            }
        }
        if golden {
            e = if xf >= xm { a - xf } else { b - xf };
            rat = golden_mean * e;
        }
        let si = sign(rat) + if rat == 0.0 { 1.0 } else { 0.0 };
        let x = xf + si * rat.abs().max(tol1);
        let fu = f(x);
        evaluations += 1;
        if fu <= fx {
            if x >= xf {
                a = xf;
            } else {
                b = xf;
            }
            (fulc, ffulc) = (nfc, fnfc);
            (nfc, fnfc) = (xf, fx);
            (xf, fx) = (x, fu);
        } else {
            if x < xf {
                a = x;
            } else {
                b = x;
            }
            if fu <= fnfc || nfc == xf {
                (fulc, ffulc) = (nfc, fnfc);
                (nfc, fnfc) = (x, fu);
            } else if fu <= ffulc || fulc == xf || fulc == nfc {
                (fulc, ffulc) = (x, fu);
            }
        }
        xm = 0.5 * (a + b);
        tol1 = sqrt_eps * xf.abs() + xatol / 3.0;
        tol2 = 2.0 * tol1;
        assert!(evaluations < 500, "bounded minimization did not converge");
    }
    (xf, fx)
}

/// Upstream `problem_geometry`: equal-radius endpoints whose Lambert
/// parameter is `lambda`, the time scale `sqrt(2 mu / s^3)`, and the minimum
/// `(x_min, T_min)` of the Lagrange time curve found independently of the
/// solver.
pub(crate) fn problem_geometry(
    lambda: f64,
    revolutions: usize,
) -> (Vector3<f64>, Vector3<f64>, f64, f64, f64) {
    let angle = PI - 4.0 * lambda.atan();
    let r1 = Vector3::new(RADIUS, 0.0, 0.0);
    let r2 = Vector3::new(angle.cos(), angle.sin(), 0.0) * RADIUS;
    let semiperimeter = (r1.norm() + r2.norm() + (r2 - r1).norm()) / 2.0;
    let time_scale = (2.0 * MU / semiperimeter.powf(3.0)).sqrt();
    let (x_min, t_min) =
        minimize_bounded(|x| lagrange_time(x, lambda, revolutions), -0.9, 0.9, 1e-14);
    (r1, r2, time_scale, x_min, t_min)
}

/// Stumpff functions `(C(z), S(z))` of the universal-variable formulation.
fn stumpff(z: f64) -> (f64, f64) {
    if z.abs() < 0.1 {
        // Series: C = sum (-z)^k / (2k+2)!, S = sum (-z)^k / (2k+3)!.
        let (mut c, mut s) = (0.0, 0.0);
        let (mut c_term, mut s_term) = (0.5, 1.0 / 6.0);
        for k in 0..20 {
            c += c_term;
            s += s_term;
            let k = f64::from(k);
            c_term *= -z / ((2.0 * k + 3.0) * (2.0 * k + 4.0));
            s_term *= -z / ((2.0 * k + 4.0) * (2.0 * k + 5.0));
        }
        (c, s)
    } else if z > 0.0 {
        let root = z.sqrt();
        ((1.0 - root.cos()) / z, (root - root.sin()) / root.powi(3))
    } else {
        let root = (-z).sqrt();
        (
            (root.cosh() - 1.0) / -z,
            (root.sinh() - root) / root.powi(3),
        )
    }
}

/// Two-body propagation of `(r0, v0)` by `dt` with universal variables
/// (Curtis, *Orbital Mechanics for Engineering Students*, algorithms 3.3 and
/// 3.4), valid for every conic. Stands in for upstream's `propagate`.
pub(crate) fn kepler_propagate(
    r0: Vector3<f64>,
    v0: Vector3<f64>,
    dt: f64,
    mu: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let sqrt_mu = mu.sqrt();
    let r0_norm = r0.norm();
    let radial_speed = r0.dot(&v0) / r0_norm;
    let alpha = 2.0 / r0_norm - v0.norm_squared() / mu;

    let mut chi = sqrt_mu * alpha.abs() * dt;
    if alpha <= 0.0 || chi == 0.0 {
        chi = sqrt_mu * dt / r0_norm;
    }
    let mut converged = false;
    for _ in 0..200 {
        let z = alpha * chi * chi;
        let (c, s) = stumpff(z);
        let f = r0_norm * radial_speed / sqrt_mu * chi * chi * c
            + (1.0 - alpha * r0_norm) * chi.powi(3) * s
            + r0_norm * chi
            - sqrt_mu * dt;
        let f_prime = r0_norm * radial_speed / sqrt_mu * chi * (1.0 - alpha * chi * chi * s)
            + (1.0 - alpha * r0_norm) * chi * chi * c
            + r0_norm;
        let step = f / f_prime;
        chi -= step;
        if step.abs() <= 1e-15 * chi.abs().max(1.0) {
            converged = true;
            break;
        }
    }
    assert!(converged, "universal Kepler equation did not converge");

    let z = alpha * chi * chi;
    let (c, s) = stumpff(z);
    let f = 1.0 - chi * chi / r0_norm * c;
    let g = dt - chi.powi(3) * s / sqrt_mu;
    let r = r0 * f + v0 * g;
    let r_norm = r.norm();
    let f_dot = sqrt_mu / (r_norm * r0_norm) * (alpha * chi.powi(3) * s - chi);
    let g_dot = 1.0 - chi * chi / r_norm * c;
    (r, r0 * f_dot + v0 * g_dot)
}

/// Classical elements as upstream's tests set them up for
/// `orbitalMotion.elem2rv_parab`, which reads a parabola (`e = 1`) from a
/// negative `a` as `rPeriap = -a`; here that becomes a zero semi-major axis
/// and the periapsis radius.
pub(crate) fn elements(
    a: f64,
    e: f64,
    inclination_rad: f64,
    raan_rad: f64,
    arg_periapsis_rad: f64,
    true_anomaly_rad: f64,
) -> ClassicElements {
    let parabolic = e == 1.0 && a < 0.0;
    ClassicElements {
        semi_major_axis: if parabolic { 0.0 } else { a },
        eccentricity: e,
        inclination_rad,
        raan_rad,
        arg_periapsis_rad,
        true_anomaly_rad,
        periapsis_radius: if parabolic { -a } else { 0.0 },
        ..Default::default()
    }
}

/// `orbitalMotion.elem2rv_parab` for elements built by [`elements`].
pub(crate) fn conic_state(mu: f64, elements: &ClassicElements) -> (Vector3<f64>, Vector3<f64>) {
    elements_to_state(mu, elements)
}

/// Port of upstream's `Support/IzzoLambert.py` (`IzzoSolve`), the external
/// reference for multi-revolution solutions, with its own starters, its
/// Lagrange/Battin/Lancaster flight time, and plain Householder iterations.
pub(crate) struct IzzoReference {
    max_revolutions: usize,
    max_iterations: usize,
    r1_hat: Vector3<f64>,
    r2_hat: Vector3<f64>,
    t1_hat: Vector3<f64>,
    t2_hat: Vector3<f64>,
    r1: f64,
    r2: f64,
    chord: f64,
    semiperimeter: f64,
    mu: f64,
    /// `lambd`.
    pub(crate) lambda: f64,
    lambda2: f64,
    /// `T`: the non-dimensional flight time.
    pub(crate) time: f64,
    /// `x`: the roots, `[zero-revolution, (left, right) per revolution]`.
    pub(crate) x: Vec<f64>,
    /// `v1`, `v2`: the velocities of each root.
    pub(crate) v1: Vec<Vector3<f64>>,
    pub(crate) v2: Vec<Vector3<f64>>,
}

impl IzzoReference {
    /// `IzzoSolve(r1, r2, tof, mu, max_multiRev)`, which solves on
    /// construction.
    pub(crate) fn new(
        r1: Vector3<f64>,
        r2: Vector3<f64>,
        tof: f64,
        mu: f64,
        max_revolutions: usize,
    ) -> Self {
        assert!(mu > 0.0 && tof > 0.0);
        let chord = (r2 - r1).norm();
        let (r1_norm, r2_norm) = (r1.norm(), r2.norm());
        let semiperimeter = 0.5 * (r1_norm + r2_norm + chord);
        let r1_hat = r1 / r1_norm;
        let r2_hat = r2 / r2_norm;
        let h_hat = r1_hat.cross(&r2_hat) / r1_hat.cross(&r2_hat).norm();
        assert!(h_hat.z != 0.0, "r1 and r2 don't define a plane");
        let sign = h_hat.z.signum();
        let lambda2 = 1.0 - chord / semiperimeter;
        let mut reference = Self {
            max_revolutions,
            max_iterations: 12,
            r1_hat,
            r2_hat,
            t1_hat: -sign * r1_hat.cross(&h_hat),
            t2_hat: -sign * r2_hat.cross(&h_hat),
            r1: r1_norm,
            r2: r2_norm,
            chord,
            semiperimeter,
            mu,
            lambda: sign * lambda2.sqrt(),
            lambda2,
            time: (2.0 * mu / semiperimeter.powi(3)).sqrt() * tof,
            x: Vec::new(),
            v1: Vec::new(),
            v2: Vec::new(),
        };
        reference.solve();
        reference
    }

    fn solve(&mut self) {
        self.x = self.find_x();
        let gamma = (self.mu * self.semiperimeter / 2.0).sqrt();
        let rho = (self.r1 - self.r2) / self.chord;
        let sigma = (1.0 - rho * rho).sqrt();
        let lambda = self.lambda;
        (self.v1, self.v2) = self
            .x
            .iter()
            .map(|&x| {
                let y = (1.0 - self.lambda2 + self.lambda2 * x * x).sqrt();
                let vr1 = gamma * ((lambda * y - x) - rho * (lambda * y + x)) / self.r1;
                let vr2 = -gamma * ((lambda * y - x) + rho * (lambda * y + x)) / self.r2;
                let vt = gamma * sigma * (y + lambda * x);
                (
                    self.r1_hat * vr1 + self.t1_hat * (vt / self.r1),
                    self.r2_hat * vr2 + self.t2_hat * (vt / self.r2),
                )
            })
            .unzip();
    }

    /// The zero-revolution starter `findxy` refines; the upstream test
    /// captures it by mocking `householder`.
    pub(crate) fn zero_revolution_starter(&self) -> f64 {
        let (lambda, time) = (self.lambda, self.time);
        let t00 = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();
        let t1 = 2.0 / 3.0 * (1.0 - lambda.powi(3));
        if time >= t00 {
            -(time - t00) / (time - t00 + 4.0)
        } else if time <= t1 {
            5.0 / 2.0 * t1 * (t1 - time) / (time * (1.0 - lambda.powi(5))) + 1.0
        } else {
            (t00 / time).powf(2.0_f64.ln() / (t00 / t1).ln()) - 1.0
        }
    }

    fn find_x(&self) -> Vec<f64> {
        let (lambda, time) = (self.lambda, self.time);
        let mut m_max = (time / PI).floor();
        let t00 = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();
        let t0 = t00 + m_max * PI;
        if time < t0 && m_max > 0.0 {
            let (mut x_old, mut x_new, mut t_min) = (0.0, 0.0, t0);
            for _ in 0..12 {
                let (dt, d2t, d3t) = self.derivatives(x_old, t_min);
                if dt != 0.0 {
                    x_new = x_old - 2.0 * dt * d2t / (2.0 * d2t * d2t - dt * d3t);
                }
                if (x_old - x_new).abs() < 1e-13 {
                    break;
                }
                t_min = self.time_of_flight(x_new, m_max);
                x_old = x_new;
            }
            if t_min > time {
                m_max -= 1.0;
            }
        }
        let m_max = m_max.min(self.max_revolutions as f64) as usize;

        let mut x = vec![0.0; 2 * m_max + 1];
        x[0] = self.householder(self.zero_revolution_starter(), 0.0, 1e-5);
        for i in 1..=m_max {
            let n = i as f64;
            let left = ((n * PI + PI) / (8.0 * time)).powf(2.0 / 3.0);
            x[2 * i - 1] = self.householder((left - 1.0) / (left + 1.0), n, 1e-8);
            let right = ((8.0 * time) / (n * PI)).powf(2.0 / 3.0);
            x[2 * i] = self.householder((right - 1.0) / (right + 1.0), n, 1e-8);
        }
        x
    }

    fn derivatives(&self, x: f64, time: f64) -> (f64, f64, f64) {
        let lambda3 = self.lambda.powi(3);
        let umx2 = 1.0 - x * x;
        let y = (1.0 - self.lambda2 * umx2).sqrt();
        let dt = 1.0 / umx2 * (3.0 * time * x - 2.0 + 2.0 * lambda3 * x / y);
        let d2t = 1.0 / umx2
            * (3.0 * time + 5.0 * x * dt + 2.0 * (1.0 - self.lambda2) * lambda3 / y.powi(3));
        let d3t = 1.0 / umx2
            * (7.0 * x * d2t + 8.0 * dt
                - 6.0 * (1.0 - self.lambda2) * self.lambda2 * lambda3 * x / y.powi(5));
        (dt, d2t, d3t)
    }

    /// `x2tof2`: Lagrange's formulation.
    fn lagrange_time_of_flight(&self, x: f64, n: f64) -> f64 {
        let a = 1.0 / (1.0 - x * x);
        if a > 0.0 {
            let alpha = 2.0 * x.acos();
            let mut beta = 2.0 * (self.lambda2 / a).sqrt().asin();
            if self.lambda < 0.0 {
                beta = -beta;
            }
            a * a.sqrt() * ((alpha - alpha.sin()) - (beta - beta.sin()) + 2.0 * PI * n) / 2.0
        } else {
            let alpha = 2.0 * x.acosh();
            let mut beta = 2.0 * (-self.lambda2 / a).sqrt().asinh();
            if self.lambda < 0.0 {
                beta = -beta;
            }
            -a * (-a).sqrt() * ((beta - beta.sinh()) - (alpha - alpha.sinh())) / 2.0
        }
    }

    /// `x2tof`, as the reference's own `IzzoSolve.x2tof(x, N)`.
    pub(crate) fn time_of_flight(&self, x: f64, n: f64) -> f64 {
        if x == 1.0 && n == 0.0 {
            return 2.0 / 3.0 * (1.0 - self.lambda.powi(3));
        }
        let (battin, lagrange) = (0.01, 0.2);
        let distance = (x - 1.0).abs();
        if distance < lagrange && distance > battin {
            return self.lagrange_time_of_flight(x, n);
        }
        let e = x * x - 1.0;
        let rho = e.abs();
        let z = (1.0 + self.lambda2 * e).sqrt();
        if distance < battin {
            let eta = z - self.lambda * x;
            let s1 = 0.5 * (1.0 - self.lambda - x * eta);
            let q = 4.0 / 3.0 * reference_hypergeometric(s1, 1e-11);
            (eta.powi(3) * q + 4.0 * self.lambda * eta) / 2.0 + n * PI / rho.powf(1.5)
        } else {
            let y = rho.sqrt();
            let g = x * z - self.lambda * e;
            let d = if e < 0.0 {
                n * PI + g.acos()
            } else {
                (y * (z - self.lambda * x) + g).ln()
            };
            (x - self.lambda * z - d / y) / e
        }
    }

    /// `householder(T, x0, N, eps, iter_max)`, returning the final iterate.
    fn householder(&self, mut x0: f64, n: f64, eps: f64) -> f64 {
        for _ in 0..self.max_iterations {
            let tof = self.time_of_flight(x0, n);
            let delta = tof - self.time;
            let x_new = if n == 0.0 && (x0 - 1.0).abs() < 1e-8 {
                // The analytic derivative limit at the parabola.
                x0 - delta / (2.0 / 5.0 * (self.lambda.powi(5) - 1.0))
            } else {
                let (dt, ddt, dddt) = self.derivatives(x0, tof);
                let dt2 = dt * dt;
                x0 - delta * (dt2 - delta * ddt / 2.0)
                    / (dt * (dt2 - delta * ddt) + dddt * delta * delta / 6.0)
            };
            let error = (x0 - x_new).abs();
            x0 = x_new;
            if error < eps {
                break;
            }
        }
        x0
    }
}

/// The reference's `hypergeometricF`: at most 12 terms.
fn reference_hypergeometric(z: f64, tolerance: f64) -> f64 {
    let (mut sum, mut term) = (1.0, 1.0);
    for j in 0..12 {
        let j = f64::from(j);
        term = term * (3.0 + j) * (1.0 + j) / (2.5 + j) * z / (j + 1.0);
        sum += term;
        if term.abs() < tolerance {
            break;
        }
    }
    sum
}

#[cfg(test)]
mod self_checks {
    //! The truth helpers against known values, so a mistake in them cannot
    //! pass for agreement with the modules.

    use super::*;
    use crate::utilities::orbital_motion::{
        eccentric_to_mean_anomaly, eccentric_to_true_anomaly, mean_to_eccentric_anomaly,
        true_to_eccentric_anomaly,
    };

    #[test]
    fn kepler_propagation_matches_the_mean_anomaly_for_an_ellipse() {
        let start = elements(RADIUS, 0.3, 0.4, 0.5, 0.6, 0.7);
        let (r0, v0) = conic_state(MU, &start);
        let n = (MU / RADIUS.powi(3)).sqrt();
        for dt in [100.0, 5000.0, 3.3e4] {
            let mean = eccentric_to_mean_anomaly(true_to_eccentric_anomaly(0.7, 0.3), 0.3) + n * dt;
            let f = eccentric_to_true_anomaly(mean_to_eccentric_anomaly(mean, 0.3), 0.3);
            let (r_truth, v_truth) = conic_state(
                MU,
                &ClassicElements {
                    true_anomaly_rad: f,
                    ..start
                },
            );
            let (r, v) = kepler_propagate(r0, v0, dt, MU);
            assert!(
                (r - r_truth).norm() < 1e-5,
                "dt {dt}: {}",
                (r - r_truth).norm()
            );
            assert!(
                (v - v_truth).norm() < 1e-9,
                "dt {dt}: {}",
                (v - v_truth).norm()
            );
        }
    }

    #[test]
    fn kepler_propagation_conserves_energy_on_unbound_orbits() {
        for e in [1.0, 1.7] {
            let (r0, v0) = conic_state(MU, &elements(-RADIUS, e, 0.2, 0.1, 0.3, -0.4));
            let energy = |r: Vector3<f64>, v: Vector3<f64>| v.norm_squared() / 2.0 - MU / r.norm();
            let (r, v) = kepler_propagate(r0, v0, 4000.0, MU);
            assert!((energy(r, v) - energy(r0, v0)).abs() < 1e-6 * MU / RADIUS);
            assert!((r.cross(&v) - r0.cross(&v0)).norm() < 1e-9 * r0.cross(&v0).norm());
        }
    }

    #[test]
    fn root_finders_and_series_reach_known_values() {
        let root = brentq(|x| x * x - 2.0, 0.0, 2.0, 1e-14);
        assert!((root - 2.0_f64.sqrt()).abs() < 1e-14);
        let (x, value) = minimize_bounded(|x| (x - 0.3).powi(2) + 1.0, -0.9, 0.9, 1e-14);
        assert!((x - 0.3).abs() < 1e-7 && (value - 1.0).abs() < 1e-14);
        // 2F1(1, 1; 2; z) = -ln(1 - z) / z, also past the Pfaff switch.
        for z in [-3.0, -0.6, 0.2, 0.9] {
            assert!((hyp2f1(1.0, 1.0, 2.0, z) + (1.0 - z).ln() / z).abs() < 1e-13);
        }
        // Battin's and Lagrange's forms of the same flight time.
        for lambda in [-0.5, 0.3] {
            for x in [-0.6, 0.6, 0.9] {
                let battin = battin_time(x, lambda);
                assert!((battin - lagrange_time(x, lambda, 0)).abs() < 1e-12 * battin);
            }
        }
    }
}
