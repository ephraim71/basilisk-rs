//! Ports of upstream `lambertSolver/_UnitTest`: `test_lambertSolver.py`,
//! `test_lambertSolverConvergence.py`, `test_lambertSolverIzzoStarter.py`,
//! `test_lambertSolverMultiRev.py`, `test_lambertSolverValidation.py` and the
//! gtest files `test_lambertInputValidation.cpp` and
//! `test_lambertTransferDirection.cpp`, with their parameters, expected values
//! and tolerances. Python truth comes from `test_support`, including the
//! `IzzoLambert.py` reference solver.
//!
//! What cannot be ported as is:
//! * Upstream's `BasiliskError` from an update becomes an `Err` from the
//!   module's `solve` and an update that publishes nothing; the tests check
//!   both.
//! * Unknown `solverMethod` and `transferDirection` integers and negative
//!   `numRevolutions` (`test_invalid_solver_method`,
//!   `test_invalid_transfer_direction`, `test_negative_revolution_count` and
//!   the gtests `RejectsUnknownSolverMethodsAndRecovers`,
//!   `SolverRejectsInvalidIntegers`) cannot be expressed in a
//!   `LambertProblemMsg`; [`unknown_method_and_direction_codes_cannot_be_expressed`]
//!   checks that the wire format rejects them, and the recovery halves of
//!   those tests are kept.
//! * The gtests' floating-point exception flags (`FE_INVALID`,
//!   `FE_DIVBYZERO`) cannot be read without `unsafe`; their rejections are
//!   checked.
//! * SciPy's `solve_ivp` becomes universal-variable Kepler propagation.

use std::f64::consts::{LN_2, PI, TAU};

use nalgebra::{Matrix3, Vector3};

use super::{InvalidAlignmentThreshold, LambertSolver, LambertSolverConfig};
use crate::Module;
use crate::fsw_algorithms::orbit_control::test_support::{
    DIRECTIONS, IzzoReference, METHODS, MU, RADIUS, battin_time, brentq, conic_state, context,
    direction_sign, elements, kepler_propagate, lagrange_time, problem, problem_geometry,
    solve_problem, solve_with,
};
use crate::messages::{
    LambertPerformanceMsg, LambertProblemMsg, LambertSolutionMsg, LambertSolverMethod,
    LambertTransferDirection, Output,
};
use crate::utilities::astro_constants::{D2R, R2D};
use crate::utilities::lambert::LambertInputError;
use crate::utilities::orbital_motion::{
    ClassicElements, eccentric_to_mean_anomaly, hyperbolic_to_mean_hyperbolic_anomaly,
    true_to_eccentric_anomaly, true_to_hyperbolic_anomaly,
};

use LambertSolverMethod::{Gooding, Izzo};
use LambertTransferDirection::{Prograde, Retrograde};

/// `numpy.testing.assert_allclose` for one value.
#[track_caller]
fn assert_close(actual: f64, expected: f64, rtol: f64, atol: f64, what: &str) {
    assert!(
        (actual - expected).abs() <= atol + rtol * expected.abs(),
        "{what}: {actual:.17e} vs {expected:.17e} (rtol {rtol}, atol {atol})"
    );
}

/// `numpy.testing.assert_allclose` for a vector.
#[track_caller]
fn assert_vector_close(
    actual: Vector3<f64>,
    expected: Vector3<f64>,
    rtol: f64,
    atol: f64,
    what: &str,
) {
    for axis in 0..3 {
        assert_close(
            actual[axis],
            expected[axis],
            rtol,
            atol,
            &format!("{what}[{axis}]"),
        );
    }
}

fn finite(vectors: &[Vector3<f64>]) -> bool {
    vectors
        .iter()
        .all(|vector| vector.iter().all(|value| value.is_finite()))
}

fn new_solver() -> LambertSolver {
    let mut module = LambertSolver::new(LambertSolverConfig::default());
    module.init();
    module
}

fn flip_y(mut r: Vector3<f64>, direction: LambertTransferDirection) -> Vector3<f64> {
    if direction == Retrograde {
        r.y = -r.y;
    }
    r
}

/// Rotation about +z by `angle`.
fn rotation_z(angle: f64) -> Matrix3<f64> {
    let (s, c) = angle.sin_cos();
    Matrix3::new(c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0)
}

/// What the upstream tests assert of a rejected or failed update: both
/// solutions invalid with zero velocities and zero diagnostics.
#[track_caller]
fn assert_cleared(solution: &LambertSolutionMsg, performance: &LambertPerformanceMsg) {
    assert_eq!(*solution, LambertSolutionMsg::default());
    assert_eq!(*performance, LambertPerformanceMsg::default());
}

// ---------------------------------------------------------------------------
// test_lambertSolver.py
// ---------------------------------------------------------------------------

/// One parameter set of `test_lambertSolver`.
#[derive(Clone, Copy, Debug)]
struct SolverCase {
    method: LambertSolverMethod,
    revolutions: usize,
    /// Multi-revolution flight time \[s\]; zero-revolution flight times come
    /// from the reference orbit.
    time_s: f64,
    eccentricity: f64,
    angle_deg: f64,
    alignment_deg: f64,
}

/// Upstream `paramList`, extensions included.
fn upstream_solver_cases() -> Vec<SolverCase> {
    let mut cases = Vec::new();
    for method in METHODS {
        for revolutions in [0, 1, 4] {
            for time_s in [1e2, 1e6] {
                for eccentricity in [0.0, 0.05, 1.0, 1.2] {
                    for angle_deg in [30.0, 90.0, 180.0, 210.0, -60.0, 500.0] {
                        // No long-way or negative angles on unbound conics.
                        if eccentricity >= 1.0 && (angle_deg >= 180.0 || angle_deg < 0.0) {
                            continue;
                        }
                        cases.push(SolverCase {
                            method,
                            revolutions,
                            time_s,
                            eccentricity,
                            angle_deg,
                            alignment_deg: 1.0,
                        });
                    }
                }
            }
        }
    }
    for (angle_deg, alignment_deg) in [(0.5, 1.0), (1.5, 1.0), (1.5, 2.0), (2.5, 2.0)] {
        cases.push(SolverCase {
            method: Gooding,
            revolutions: 0,
            time_s: 1e2,
            eccentricity: 0.0,
            angle_deg,
            alignment_deg,
        });
    }
    for method in METHODS {
        for angle_deg in [210.0, -60.0, 355.0, 500.0] {
            cases.push(SolverCase {
                method,
                revolutions: 0,
                time_s: 1e2,
                eccentricity: 0.7,
                angle_deg,
                alignment_deg: 1.0,
            });
        }
    }
    cases
}

/// Which recorded velocity `corrupted_velocities` perturbs.
#[derive(Clone, Copy, PartialEq)]
enum Corrupt {
    None,
    V1,
    V2,
}

/// Upstream `lambertSolverTestFunction`, returning the variables that fail
/// their checks instead of raising on the first. `corrupt` adds 1 m/s to the
/// x-component of a recorded velocity, as the monkeypatched recorder of
/// `test_zero_revolution_rejects_corrupted_velocities` does.
fn solver_case_failures(case: SolverCase, corrupt: Corrupt) -> Vec<&'static str> {
    let accuracy = 1e-2;
    let mu = 3.986004418e14;
    let radius = 10000.0 * 1000.0;
    let a = if case.eccentricity < 1.0 {
        radius
    } else {
        -radius
    };
    let e = case.eccentricity;
    let oe1 = elements(a, e, 5.0 * D2R, 25.0 * D2R, 30.0 * D2R, 10.0 * D2R);
    let (r1, reference_v1) = conic_state(mu, &oe1);
    let f2 = (oe1.true_anomaly_rad + case.angle_deg * D2R) * R2D;
    let oe2 = ClassicElements {
        true_anomaly_rad: f2.rem_euclid(360.0) * D2R,
        ..oe1
    };
    let (r2, reference_v2) = conic_state(mu, &oe2);
    let (f_1, f_2) = (oe1.true_anomaly_rad, oe2.true_anomaly_rad);

    let transfer_time = if case.revolutions > 0 {
        case.time_s
    } else if e < 1.0 {
        let mean = |f| eccentric_to_mean_anomaly(true_to_eccentric_anomaly(f, e), e);
        let n = (mu / a.powf(3.0)).sqrt();
        // The forward prograde arc, wrapping through periapsis.
        (mean(f_2) - mean(f_1)).rem_euclid(TAU) / n
    } else if e == 1.0 {
        let barker = |f: f64| {
            let d = (f / 2.0).tan();
            d + 1.0 / 3.0 * d.powf(3.0)
        };
        let n = (mu / (2.0 * (-a).powf(3.0))).sqrt();
        (barker(f_2) - barker(f_1)).abs() / n
    } else {
        let mean = |f| hyperbolic_to_mean_hyperbolic_anomaly(true_to_hyperbolic_anomaly(f, e), e);
        let n = (mu / (-a).powf(3.0)).sqrt();
        (mean(f_2) - mean(f_1)).abs() / n
    };

    let mut module = new_solver();
    if case.alignment_deg != 1.0 {
        module
            .set_alignment_threshold_deg(case.alignment_deg)
            .expect("upstream thresholds are valid");
    }
    let (mut solution, performance) = solve_with(
        &mut module,
        LambertProblemMsg {
            mu_m3ps2: mu,
            ..problem(
                r1,
                r2,
                transfer_time,
                case.revolutions,
                case.method,
                Prograde,
            )
        },
    );
    match corrupt {
        Corrupt::None => {}
        Corrupt::V1 => solution.first.v1_n_mps.x += 1.0,
        Corrupt::V2 => solution.first.v2_n_mps.x += 1.0,
    }

    let reference = (case.revolutions > 0 && case.angle_deg != 180.0)
        .then(|| IzzoReference::new(r1, r2, transfer_time, mu, case.revolutions));
    let solutions = reference.as_ref().map_or(0, |reference| reference.x.len());
    let index = (2 * case.revolutions).saturating_sub(1);
    let aligned = (case.angle_deg * D2R).sin().abs() < (case.alignment_deg * D2R).sin().abs();

    let zero = Vector3::zeros();
    let (v1, v2, valid, v1_sol2, v2_sol2, valid_sol2) =
        if aligned || (case.revolutions > 0 && index + 1 > solutions) {
            (zero, zero, 0.0, zero, zero, 0.0)
        } else if case.revolutions == 0 {
            (reference_v1, reference_v2, 1.0, zero, zero, 0.0)
        } else {
            let reference = reference.as_ref().expect("multi-revolution reference");
            (
                reference.v1[index],
                reference.v2[index],
                1.0,
                reference.v1[index + 1],
                reference.v2[index + 1],
                1.0,
            )
        };
    let (x, x_sol2, x_true, x_true_sol2) =
        match reference.as_ref().filter(|_| !aligned && index < solutions) {
            Some(reference) => (
                performance.first.x,
                performance.second.x,
                reference.x[index],
                reference.x[index + 1],
            ),
            None => (0.0, 0.0, 0.0, 0.0),
        };

    let within = |actual: f64, expected: f64| (actual - expected).abs() <= accuracy;
    let vector_within = |actual: Vector3<f64>, expected: Vector3<f64>| {
        (0..3).all(|i| within(actual[i], expected[i]))
    };
    let flag = |valid: bool| if valid { 1.0 } else { 0.0 };
    let checks = [
        ("v1_N", vector_within(solution.first.v1_n_mps, v1)),
        ("v2_N", vector_within(solution.first.v2_n_mps, v2)),
        ("validFlag", within(flag(solution.first.valid), valid)),
        ("v1Sol2_N", vector_within(solution.second.v1_n_mps, v1_sol2)),
        ("v2Sol2_N", vector_within(solution.second.v2_n_mps, v2_sol2)),
        (
            "validFlagSol2",
            within(flag(solution.second.valid), valid_sol2),
        ),
        ("x", within(x, x_true)),
        ("xSol2", within(x_sol2, x_true_sol2)),
    ];
    checks
        .into_iter()
        .filter_map(|(name, passed)| (!passed).then_some(name))
        .collect()
}

/// Upstream `test_lambertSolver`: both algorithms over revolution counts,
/// flight times, conic eccentricities, transfer angles and alignment
/// thresholds, against orbit-derived zero-revolution velocities and the
/// `IzzoLambert.py` multi-revolution reference, at 1e-2 (m/s and `x`).
#[test]
fn matches_upstream_lambert_solver_test() {
    let cases = upstream_solver_cases();
    assert_eq!(cases.len(), 204);
    for case in cases {
        let failures = solver_case_failures(case, Corrupt::None);
        assert!(failures.is_empty(), "{case:?}: {failures:?}");
    }
}

/// Upstream `test_zero_revolution_rejects_corrupted_velocities`: a 1 m/s
/// error in either recorded velocity must fail that velocity's check, for
/// both algorithms and all four conics.
#[test]
fn zero_revolution_check_rejects_corrupted_velocities() {
    for method in METHODS {
        for eccentricity in [0.0, 0.05, 1.0, 1.2] {
            for (corrupt, name) in [(Corrupt::V1, "v1_N"), (Corrupt::V2, "v2_N")] {
                let case = SolverCase {
                    method,
                    revolutions: 0,
                    time_s: 1e2,
                    eccentricity,
                    angle_deg: 90.0,
                    alignment_deg: 1.0,
                };
                assert!(solver_case_failures(case, Corrupt::None).is_empty());
                let failures = solver_case_failures(case, corrupt);
                assert!(failures.contains(&name), "{case:?}: {failures:?}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// test_lambertSolverConvergence.py
// ---------------------------------------------------------------------------

/// Upstream `test_safeguards_preserve_fast_convergence`: starters, fallback
/// recovery and tiny corrections stay within the validator's budget (fewer
/// than six iterations, a final step below 1e-8), on the independently
/// bracketed root.
#[test]
fn safeguards_preserve_fast_convergence() {
    let cases = [
        // Endpoint [m], flight time [s], independently bracketed root [-].
        (
            (10499578.7058981, -1008665.595563449),
            66294.046738751,
            -0.9222028885569338,
        ),
        (
            (-8007153.311464242, 1134336.721226995),
            1517.1851310277925,
            1.003755300013543,
        ),
        (
            (-1373963.0772790438, 21169783.315275904),
            1029.9495735576572,
            3.9070784702480257,
        ),
        (
            (-1160389.9429338065, 9261648.1844296623),
            1347.9302728188266,
            0.9692339680381958,
        ),
        (
            (17175236.41488691, -6712075.4724303577),
            1589.8316798218957,
            1.9840057290698279,
        ),
    ];
    for method in METHODS {
        for direction in DIRECTIONS {
            for ((x2, y2), transfer_time, reference_x) in cases {
                let r1 = Vector3::new(RADIUS, 0.0, 0.0);
                let r2 = Vector3::new(x2, y2, 0.0);
                let chord = (r2 - r1).norm();
                let semiperimeter = (RADIUS + r2.norm() + chord) / 2.0;
                let lambda = (1.0 - chord / semiperimeter).sqrt().copysign(r2.y);
                let target = transfer_time * (2.0 * MU / semiperimeter.powf(3.0)).sqrt();
                let r2 = flip_y(r2, direction);
                let (solution, performance) =
                    solve_problem(r1, r2, transfer_time, 0, method, direction);
                let what = format!("{method:?} {direction:?} {reference_x}");
                assert!(solution.first.valid && !solution.second.valid, "{what}");
                assert!(performance.first.num_iter < 6, "{what}: {performance:?}");
                assert!(performance.first.err_x < 1e-8, "{what}: {performance:?}");
                assert_close(performance.first.x, reference_x, 0.0, 1e-12, &what);
                let reference_time = if reference_x < 1.0 {
                    lagrange_time(performance.first.x, lambda, 0)
                } else {
                    battin_time(performance.first.x, lambda)
                };
                assert_close(reference_time, target, 1e-12, 0.0, &what);
            }
        }
    }
}

/// Upstream `test_parabolic_roundoff_and_nearby_times`: exact and nearby
/// parabolic transfers resolve without a flat flight-time interval; the exact
/// one keeps the analytic root and the parabolic periapsis speed.
#[test]
fn parabolic_roundoff_and_nearby_times() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for angle_deg in [1.01, 1.4455193992490614, 90.0, 175.0] {
                for time_offset in [-5e-10, -5e-13, 0.0, 5e-13, 5e-10] {
                    let what = format!("{method:?} {direction:?} {angle_deg} {time_offset}");
                    let angle = angle_deg * D2R;
                    let sign = direction_sign(direction);
                    let r1 = Vector3::new(RADIUS, 0.0, 0.0);
                    let r2 = Vector3::new(angle.cos(), sign * angle.sin(), 0.0)
                        * (2.0 * RADIUS / (1.0 + angle.cos()));
                    // Barker's equation from periapsis.
                    let tangent = (angle / 2.0).tan();
                    let parabolic_time =
                        (2.0 * RADIUS.powf(3.0) / MU).sqrt() * (tangent + tangent.powf(3.0) / 3.0);
                    let transfer_time = parabolic_time * (1.0 + time_offset);
                    let chord = (r2 - r1).norm();
                    let semiperimeter = (RADIUS + r2.norm() + chord) / 2.0;
                    let lambda = (1.0 - chord / semiperimeter).sqrt();
                    let target = transfer_time * (2.0 * MU / semiperimeter.powf(3.0)).sqrt();
                    let (solution, performance) =
                        solve_problem(r1, r2, transfer_time, 0, method, direction);
                    assert!(solution.first.valid && !solution.second.valid, "{what}");
                    assert_close(
                        battin_time(performance.first.x, lambda),
                        target,
                        1e-12,
                        0.0,
                        &what,
                    );
                    if time_offset == 0.0 {
                        assert_eq!(performance.first.x, 1.0, "{what}");
                        assert_eq!(performance.first.num_iter, 0, "{what}");
                        assert_eq!(performance.first.err_x, 0.0, "{what}");
                        let speed = (2.0 * MU / RADIUS).sqrt();
                        assert_vector_close(
                            solution.first.v1_n_mps,
                            Vector3::new(0.0, sign * speed, 0.0),
                            0.0,
                            1e-8,
                            &what,
                        );
                        if angle_deg < 2.0 {
                            let (final_position, _) =
                                kepler_propagate(r1, solution.first.v1_n_mps, transfer_time, MU);
                            assert_vector_close(final_position, r2, 0.0, 1e-5, &what);
                        }
                    }
                }
            }
        }
    }
}

/// Upstream `test_short_transfer_convergence`: short transfers that once
/// exhausted Izzo's eight iterations reach the target when propagated.
#[test]
fn short_transfer_convergence() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for (lambda, fraction) in [
                (0.9473913043478261, 0.01),
                (0.9615942028985508, 0.04379310344827586),
                (0.99, 0.21275862068965518),
            ] {
                let what = format!("{method:?} {direction:?} {lambda}");
                let (r1, r2, scale, _, _) = problem_geometry(lambda, 1);
                let r2 = flip_y(r2, direction);
                let time_at_zero = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();
                let parabolic_time = 2.0 / 3.0 * (1.0 - lambda.powi(3));
                let transfer_time =
                    (parabolic_time + (time_at_zero - parabolic_time) * fraction) / scale;
                let (solution, performance) =
                    solve_problem(r1, r2, transfer_time, 0, method, direction);
                assert!(solution.first.valid && !solution.second.valid, "{what}");
                assert!(performance.first.err_x.is_finite() && performance.first.err_x <= 1e-8);
                let (position, velocity) =
                    kepler_propagate(r1, solution.first.v1_n_mps, transfer_time, MU);
                assert_vector_close(position, r2, 0.0, 1e-3, &what);
                assert_vector_close(velocity, solution.first.v2_n_mps, 0.0, 1e-5, &what);
            }
        }
    }
}

/// Upstream `test_roots_converge_near_minimum`: both roots just above the
/// minimum flight time, against independent branch brackets.
#[test]
fn roots_converge_near_minimum() {
    for lambda in [-0.99, -0.5, 0.5, 0.99] {
        for revolutions in [1, 3] {
            let (r1, r2, scale, x_min, t_min) = problem_geometry(lambda, revolutions);
            for method in METHODS {
                for direction in DIRECTIONS {
                    for excess_time in [1e-12, 1e-10] {
                        let what = format!(
                            "{method:?} {direction:?} {lambda} {revolutions} {excess_time}"
                        );
                        let target = t_min * (1.0 + excess_time);
                        let (solution, performance) = solve_problem(
                            r1,
                            flip_y(r2, direction),
                            target / scale,
                            revolutions,
                            method,
                            direction,
                        );
                        assert!(solution.first.valid && solution.second.valid, "{what}");
                        let roots = [performance.first.x, performance.second.x];
                        let f = |x| lagrange_time(x, lambda, revolutions) - target;
                        let reference = [
                            brentq(f, -0.99, x_min, 1e-14),
                            brentq(f, x_min, 0.99, 1e-14),
                        ];
                        assert!(
                            -1.0 < roots[0]
                                && roots[0] < x_min
                                && x_min < roots[1]
                                && roots[1] < 1.0,
                            "{what}: {roots:?} around {x_min}"
                        );
                        for (root, reference) in roots.into_iter().zip(reference) {
                            assert_close(root, reference, 0.0, 2e-8, &what);
                            assert_close(
                                lagrange_time(root, lambda, revolutions),
                                target,
                                2e-12,
                                0.0,
                                &what,
                            );
                        }
                        for error in [performance.first.err_x, performance.second.err_x] {
                            assert!(error.is_finite() && error <= 1e-8, "{what}: {error}");
                        }
                    }
                }
            }
        }
    }
}

/// Upstream `test_alignment_rejection_and_recovery`: an aligned update clears
/// both solutions, and the next valid one on the same module matches a fresh
/// module, also when the endpoint and flight time change.
#[test]
fn alignment_rejection_and_recovery() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for revolutions in [0, 1] {
                for angle_deg in [0.0, 0.5, 179.5, 180.0, 180.5, 359.5] {
                    let what = format!("{method:?} {direction:?} {revolutions} {angle_deg}");
                    let r1 = Vector3::new(RADIUS, 0.0, 0.0);
                    let sign = direction_sign(direction);
                    let angle = angle_deg * D2R;
                    let mut aligned_r2 =
                        Vector3::new(angle.cos(), sign * angle.sin(), 0.0) * RADIUS;
                    if angle_deg == 0.0 || angle_deg == 180.0 {
                        aligned_r2.y = 0.0; // an exactly zero cross product
                    }
                    let r2_a = Vector3::new(0.0, sign * RADIUS, 0.0);
                    let r2_b = Vector3::new(-0.5, sign * 0.75_f64.sqrt(), 0.0) * RADIUS;
                    let time_a = if revolutions == 0 { 2000.0 } else { 10000.0 };
                    let time_b = 1.2 * time_a;
                    let expected_a =
                        solve_problem(r1, r2_a, time_a, revolutions, method, direction);
                    let expected_b =
                        solve_problem(r1, r2_b, time_b, revolutions, method, direction);
                    for (solution, _) in [&expected_a, &expected_b] {
                        assert!(solution.first.valid, "{what}");
                        assert_eq!(solution.second.valid, revolutions > 0, "{what}");
                    }

                    let source = Output::new(LambertProblemMsg::default());
                    let mut module = new_solver();
                    source.connect_to(&mut module.lambert_problem_in_msg);
                    module.reset(&context(0));
                    for (r2, transfer_time, expected) in [
                        (aligned_r2, time_a, None),
                        (r2_a, time_a, Some(&expected_a)),
                        (aligned_r2, time_a, None),
                        (r2_b, time_b, Some(&expected_b)),
                        (aligned_r2, time_b, None),
                        (r2_a, time_a, Some(&expected_a)),
                    ] {
                        source.write(problem(
                            r1,
                            r2,
                            transfer_time,
                            revolutions,
                            method,
                            direction,
                        ));
                        module.update(&context(0));
                        let solution = module.lambert_solution_out_msg.read();
                        let performance = module.lambert_performance_out_msg.read();
                        match expected {
                            None => assert_cleared(&solution, &performance),
                            Some((expected_solution, expected_performance)) => {
                                assert_eq!(solution, *expected_solution, "{what}");
                                assert_eq!(performance, *expected_performance, "{what}");
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Upstream `test_alignment_threshold_recovery`: the same input reevaluated
/// as the threshold changes, without a reset.
#[test]
fn alignment_threshold_recovery() {
    for method in METHODS {
        for direction in DIRECTIONS {
            let angle = 1.5 * D2R;
            let sign = direction_sign(direction);
            let source = Output::new(problem(
                Vector3::new(RADIUS, 0.0, 0.0),
                Vector3::new(angle.cos(), sign * angle.sin(), 0.0) * RADIUS,
                2000.0,
                0,
                method,
                direction,
            ));
            let mut module = new_solver();
            source.connect_to(&mut module.lambert_problem_in_msg);
            module.reset(&context(0));
            for (threshold_deg, expected_valid) in
                [(2.0, false), (1.0, true), (2.0, false), (1.0, true)]
            {
                module.set_alignment_threshold_deg(threshold_deg).unwrap();
                module.update(&context(0));
                assert_eq!(
                    module.lambert_solution_out_msg.read().first.valid,
                    expected_valid,
                    "{method:?} {direction:?} {threshold_deg}"
                );
            }
        }
    }
}

/// Upstream `test_numerical_failure_and_recovery`: a flight time whose root
/// double precision cannot resolve clears both solutions, and the next request
/// recovers without reusing them.
#[test]
fn numerical_failure_and_recovery() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for revolutions in [0, 1] {
                let what = format!("{method:?} {direction:?} {revolutions}");
                let r1 = Vector3::new(RADIUS, 0.0, 0.0);
                let r2 = Vector3::new(0.0, direction_sign(direction) * RADIUS, 0.0);
                let normal_time = if revolutions == 0 { 2000.0 } else { 10000.0 };
                let unresolved_time = 1e300;
                let mut module = new_solver();
                let mut previous_velocity = None;
                for transfer_time in [normal_time, unresolved_time, 1.1 * normal_time] {
                    let (solution, performance) = solve_with(
                        &mut module,
                        problem(r1, r2, transfer_time, revolutions, method, direction),
                    );
                    if transfer_time == unresolved_time {
                        assert_cleared(&solution, &performance);
                        continue;
                    }
                    assert!(solution.first.valid, "{what}");
                    assert_eq!(solution.second.valid, revolutions > 0, "{what}");
                    assert!(finite(&[
                        solution.first.v1_n_mps,
                        solution.first.v2_n_mps,
                        solution.second.v1_n_mps,
                        solution.second.v2_n_mps,
                    ]));
                    match previous_velocity {
                        None => previous_velocity = Some(solution.first.v1_n_mps),
                        Some(previous) => {
                            assert!((solution.first.v1_n_mps - previous).norm() > 1e-3, "{what}");
                        }
                    }
                }
            }
        }
    }
}

/// Upstream `test_independent_branch_validity`: a resolved branch is kept when
/// the other exceeds the evaluator's resolution, and a following normal
/// request resolves both.
#[test]
fn independent_branch_validity() {
    let lambda = 0.5;
    let (r1, r2, scale, _, t_min) = problem_geometry(lambda, 1);
    let long_time = lagrange_time(-1.0 + 1e-8, lambda, 1);
    for method in METHODS {
        for direction in DIRECTIONS {
            let mut module = new_solver();
            for target in [long_time, 1.1 * t_min] {
                let what = format!("{method:?} {direction:?} {target}");
                let (solution, performance) = solve_with(
                    &mut module,
                    problem(
                        r1,
                        flip_y(r2, direction),
                        target / scale,
                        1,
                        method,
                        direction,
                    ),
                );
                assert!(solution.first.valid, "{what}");
                assert_close(
                    lagrange_time(performance.first.x, lambda, 1),
                    target,
                    2e-12,
                    0.0,
                    &what,
                );
                if solution.second.valid {
                    assert_close(
                        lagrange_time(performance.second.x, lambda, 1),
                        target,
                        2e-12,
                        0.0,
                        &what,
                    );
                } else {
                    assert_eq!(target, long_time, "{what}");
                    assert_eq!(solution.second, Default::default(), "{what}");
                    assert_eq!(performance.second, Default::default(), "{what}");
                }
            }
        }
    }
}

/// Upstream `test_small_step_alone_is_not_convergence`: near-parabolic
/// requests must meet the independent residual, not just a small final step.
#[test]
fn small_step_alone_is_not_convergence() {
    for lambda in [-0.99, -0.5, 0.5, 0.99] {
        let (r1, r2, scale, _, _) = problem_geometry(lambda, 1);
        for method in METHODS {
            for time_offset in [-5e-10, 5e-10] {
                let what = format!("{method:?} {lambda} {time_offset}");
                let target = 2.0 / 3.0 * (1.0 - lambda.powi(3)) * (1.0 + time_offset);
                let (solution, performance) =
                    solve_problem(r1, r2, target / scale, 0, method, Prograde);
                assert!(solution.first.valid, "{what}");
                assert!(performance.first.err_x <= 1e-8, "{what}");
                assert_close(
                    battin_time(performance.first.x, lambda),
                    target,
                    1e-12,
                    0.0,
                    &what,
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// test_lambertSolverIzzoStarter.py
// ---------------------------------------------------------------------------

/// Upstream `intermediate_problem`: a zero-revolution request a `fraction` of
/// the way along the logarithmic time interval from `x = 0` (0) to the
/// parabola (1); returns the endpoints, the flight time \[s\] and the
/// non-dimensional target.
fn intermediate_problem(lambda: f64, fraction: f64) -> (Vector3<f64>, Vector3<f64>, f64, f64) {
    let angle = PI - 4.0 * lambda.atan();
    let r1 = Vector3::new(RADIUS, 0.0, 0.0);
    let r2 = Vector3::new(angle.cos(), angle.sin(), 0.0) * RADIUS;
    let semiperimeter = (2.0 * RADIUS + (r2 - r1).norm()) / 2.0;
    let time_scale = (2.0 * MU / semiperimeter.powf(3.0)).sqrt();
    let time_at_zero = lambda.acos() + lambda * (1.0 - lambda * lambda).sqrt();
    let parabolic_time = 2.0 / 3.0 * (1.0 - lambda.powi(3));
    let target = time_at_zero * (parabolic_time / time_at_zero).powf(fraction);
    (r1, r2, target / time_scale, target)
}

const STARTER_LAMBDAS: [f64; 6] = [-0.99, -0.5, 0.1, 0.5, 0.9, 0.99];

/// Upstream `test_reference_starter_logarithmic_interpolation`: the
/// reference's intermediate starter interpolates `log(1 + x)` linearly, so it
/// is `2^fraction - 1` whatever the geometry.
#[test]
fn reference_starter_interpolates_logarithmically() {
    for lambda in STARTER_LAMBDAS {
        for fraction in [0.0, 1e-6, 0.1, 0.5, 0.9, 1.0 - 1e-6, 1.0] {
            let (r1, r2, transfer_time, _) = intermediate_problem(lambda, fraction);
            let reference = IzzoReference::new(r1, r2, transfer_time, MU, 0);
            let expected = (fraction * LN_2).exp_m1();
            assert_close(
                reference.zero_revolution_starter(),
                expected,
                0.0,
                2e-14,
                &format!("{lambda} {fraction}"),
            );
        }
    }
}

/// Upstream `test_reference_rounded_parabolic_starter`: the reference
/// refines a starter that rounds to exactly one without a 0/0.
#[test]
fn reference_refines_a_rounded_parabolic_starter() {
    let reference = IzzoReference::new(
        Vector3::new(8000000.0, 0.0, 0.0),
        Vector3::new(2240000.0, -7680000.0, 0.0),
        1216.4446013793424,
        MU,
        0,
    );
    assert_eq!(reference.x.len(), 1);
    assert_close(reference.x[0], 1.0, 0.0, 1e-14, "x");
    assert!(finite(&[reference.v1[0], reference.v2[0]]));
    assert_close(
        battin_time(reference.x[0], reference.lambda),
        reference.time,
        2e-14,
        0.0,
        "T",
    );
}

/// Upstream `test_reference_parabolic_boundary`: the reference stays finite
/// and accurate on both sides of the parabolic time; at it, the velocities are
/// the analytic parabolic ones.
#[test]
fn reference_is_accurate_across_the_parabolic_boundary() {
    for lambda in STARTER_LAMBDAS {
        for time_offset in [-1e-7, -1e-12, -2e-16, 0.0, 2e-16, 1e-12, 1e-7] {
            let what = format!("{lambda} {time_offset}");
            let (r1, r2, parabolic_time, _) = intermediate_problem(lambda, 1.0);
            let transfer_time = parabolic_time * (1.0 + time_offset);
            let reference = IzzoReference::new(r1, r2, transfer_time, MU, 0);
            let analytic_time = reference.time_of_flight(1.0, 0.0);
            let x = reference.x[0];
            assert!(
                x.is_finite() && finite(&[reference.v1[0], reference.v2[0]]),
                "{what}"
            );
            assert_close(
                analytic_time,
                2.0 / 3.0 * (1.0 - reference.lambda.powi(3)),
                2e-15,
                0.0,
                &what,
            );
            assert_close(
                battin_time(x, reference.lambda),
                reference.time,
                2e-12,
                0.0,
                &what,
            );
            if time_offset.abs() >= 1e-12 {
                // Nearby transfers are refined, not flattened to x = 1.
                assert!((x - 1.0) * time_offset < 0.0, "{what}: {x}");
            } else {
                assert_close(x, 1.0, 0.0, 2e-14, &what);
                let semiperimeter = (r1.norm() + r2.norm() + (r2 - r1).norm()) / 2.0;
                let speed_scale = (MU * semiperimeter / 2.0).sqrt() / RADIUS;
                let radial2 = r2 / RADIUS;
                let tangent2 = Vector3::z().cross(&radial2);
                let expected_v1 = Vector3::new(lambda - 1.0, lambda + 1.0, 0.0) * speed_scale;
                let expected_v2 =
                    (radial2 * (1.0 - lambda) + tangent2 * (1.0 + lambda)) * speed_scale;
                assert_vector_close(reference.v1[0], expected_v1, 2e-12, 1e-9, &what);
                assert_vector_close(reference.v2[0], expected_v2, 2e-12, 1e-9, &what);
            }
        }
    }
}

/// Upstream `test_intermediate_izzo_convergence`: Izzo's intermediate
/// interval and its boundaries resolve within the validator's budget, on the
/// root of independent Lagrange and Battin evaluators.
#[test]
fn intermediate_izzo_convergence() {
    for direction in DIRECTIONS {
        for lambda in STARTER_LAMBDAS {
            for fraction in [-1e-6, 1e-6, 0.1, 0.5, 0.9, 1.0 - 1e-6, 1.0 + 1e-6] {
                let what = format!("{direction:?} {lambda} {fraction}");
                let (r1, r2, transfer_time, target) = intermediate_problem(lambda, fraction);
                let (solution, performance) =
                    solve_problem(r1, flip_y(r2, direction), transfer_time, 0, Izzo, direction);
                assert!(solution.first.valid && !solution.second.valid, "{what}");
                assert!(performance.first.num_iter < 6, "{what}: {performance:?}");
                assert!(performance.first.err_x < 1e-8, "{what}: {performance:?}");

                // The elliptic formula cancels near the parabolic root.
                let independent_time = |x: f64| {
                    if x < 0.5 {
                        lagrange_time(x, lambda, 0)
                    } else {
                        battin_time(x, lambda)
                    }
                };
                let reference_x = brentq(|x| independent_time(x) - target, -0.1, 1.1, 1e-14);
                assert_close(performance.first.x, reference_x, 0.0, 2e-12, &what);
                assert_close(
                    independent_time(performance.first.x),
                    target,
                    2e-12,
                    0.0,
                    &what,
                );
                assert!(finite(&[solution.first.v1_n_mps, solution.first.v2_n_mps]));
                let momentum = r1.cross(&solution.first.v1_n_mps);
                assert!(direction_sign(direction) * momentum.z > 0.0, "{what}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// test_lambertSolverMultiRev.py
// ---------------------------------------------------------------------------

/// Upstream `test_gooding_multirevolution_branches`: both roots against
/// independent bracketing, near the minimum and far from it, for short and
/// long transfers (including the issue 1651 geometry).
#[test]
fn gooding_multirevolution_branches() {
    for lambda in [-0.99, -0.9, -0.5, -0.1, 0.1, 0.263584442471, 0.5, 0.9] {
        for revolutions in [1, 2, 3, 4, 7] {
            let (r1, r2, time_scale, x_min, t_min) = problem_geometry(lambda, revolutions);
            for time_ratio in [1.00000001, 1.0001, 1.01, 1.041779966, 1.1, 1.5, 10.0, 100.0] {
                let what = format!("{lambda} {revolutions} {time_ratio}");
                let target = time_ratio * t_min;
                let (solution, performance) =
                    solve_problem(r1, r2, target / time_scale, revolutions, Gooding, Prograde);
                assert!(solution.first.valid && solution.second.valid, "{what}");
                let roots = [performance.first.x, performance.second.x];
                assert!(
                    -1.0 < roots[0] && roots[0] < x_min && x_min < roots[1] && roots[1] < 1.0,
                    "{what}: {roots:?} around {x_min}"
                );
                let f = |x| lagrange_time(x, lambda, revolutions) - target;
                let reference = [
                    brentq(f, -1.0 + 1e-10, x_min, 2e-12),
                    brentq(f, x_min, 1.0 - 1e-10, 2e-12),
                ];
                for (root, reference) in roots.into_iter().zip(reference) {
                    assert_close(root, reference, 0.0, 2e-8, &what);
                    assert_close(
                        lagrange_time(root, lambda, revolutions),
                        target,
                        2e-12,
                        0.0,
                        &what,
                    );
                }
                assert!(finite(&[
                    solution.first.v1_n_mps,
                    solution.first.v2_n_mps,
                    solution.second.v1_n_mps,
                    solution.second.v2_n_mps,
                ]));
                assert!((solution.first.v1_n_mps - solution.second.v1_n_mps).norm() > 1e-3);
            }
        }
    }
}

/// Upstream `test_minimum_time_boundary`: infeasible, coalesced and distinct
/// roots on successive updates of one module.
#[test]
fn minimum_time_boundary() {
    for lambda in [-0.99, -0.9, -0.5, 0.1, 0.5, 0.9] {
        for revolutions in [1, 3, 7] {
            let (r1, r2, time_scale, x_min, t_min) = problem_geometry(lambda, revolutions);
            for method in METHODS {
                for direction in DIRECTIONS {
                    let mut module = new_solver();
                    for ratio in [0.9999, 1.01, 1.0, 0.9999, 1.01] {
                        let what =
                            format!("{method:?} {direction:?} {lambda} {revolutions} {ratio}");
                        let (solution, performance) = solve_with(
                            &mut module,
                            problem(
                                r1,
                                flip_y(r2, direction),
                                ratio * t_min / time_scale,
                                revolutions,
                                method,
                                direction,
                            ),
                        );
                        if ratio < 1.0 {
                            assert!(!solution.first.valid && !solution.second.valid, "{what}");
                            assert_eq!(solution, LambertSolutionMsg::default(), "{what}");
                        } else if ratio == 1.0 {
                            assert!(solution.first.valid && solution.second.valid, "{what}");
                            assert_close(performance.first.x, x_min, 0.0, 2e-8, &what);
                            assert_close(performance.second.x, x_min, 0.0, 2e-8, &what);
                            assert_eq!(solution.first, solution.second, "{what}");
                            assert_eq!(performance.first.num_iter, 0, "{what}");
                            assert_eq!(performance.second.num_iter, 0, "{what}");
                            assert_eq!(performance.first.err_x, 0.0, "{what}");
                            assert_eq!(performance.second.err_x, 0.0, "{what}");
                        } else {
                            assert!(solution.first.valid && solution.second.valid, "{what}");
                            assert!(performance.first.x < x_min && x_min < performance.second.x);
                            for root in [performance.first.x, performance.second.x] {
                                assert_close(
                                    lagrange_time(root, lambda, revolutions),
                                    ratio * t_min,
                                    2e-12,
                                    0.0,
                                    &what,
                                );
                            }
                            assert!(finite(&[
                                solution.first.v1_n_mps,
                                solution.first.v2_n_mps,
                                solution.second.v1_n_mps,
                                solution.second.v2_n_mps,
                            ]));
                        }
                        if solution.first.valid {
                            for velocity in [solution.first.v1_n_mps, solution.second.v1_n_mps] {
                                let momentum = r1.cross(&velocity);
                                assert!(direction_sign(direction) * momentum.z > 0.0, "{what}");
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Upstream `test_nonfinite_position_rejected_before_minimum_search`.
#[test]
fn nonfinite_position_rejected_before_minimum_search() {
    for method in METHODS {
        let mut module = new_solver();
        let source = Output::new(problem(
            Vector3::new(RADIUS, 0.0, 0.0),
            Vector3::new(f64::NAN, RADIUS, 0.0),
            1.0e4,
            1,
            method,
            Prograde,
        ));
        source.connect_to(&mut module.lambert_problem_in_msg);
        assert_eq!(module.solve(), Err(LambertInputError::R2));
    }
}

/// Upstream `test_issue_1651_reported_orbit`: the reported orbit is
/// recovered, and both distinct solutions reach the target.
#[test]
fn issue_1651_reported_orbit() {
    let r1 = Vector3::new(4155609.690, 3501920.014, 3501920.014);
    let r2 = Vector3::new(-8931746.497, 1113628.050, 1113628.050);
    let transfer_time = 9302.364;
    let known_velocity = Vector3::new(-6239.193, 4156.804, 4156.804);
    for method in METHODS {
        let what = format!("{method:?}");
        let (solution, performance) = solve_problem(r1, r2, transfer_time, 1, method, Prograde);
        assert!(solution.first.valid && solution.second.valid, "{what}");
        assert!(performance.first.x < performance.second.x, "{what}");
        assert_vector_close(solution.second.v1_n_mps, known_velocity, 0.0, 2e-3, &what);
        for branch in [solution.first, solution.second] {
            let (position, velocity) = kepler_propagate(r1, branch.v1_n_mps, transfer_time, MU);
            assert_vector_close(position, r2, 0.0, 1e-2, &what);
            assert_vector_close(velocity, branch.v2_n_mps, 0.0, 1e-5, &what);
        }
    }
}

/// Upstream `test_known_elliptic_orbit`: exact Keplerian velocities for zero
/// and multiple revolutions, short and long arcs, prograde and retrograde.
#[test]
fn known_elliptic_orbit() {
    for method in METHODS {
        for revolutions in [0, 1, 2, 3] {
            for final_anomaly_deg in [140.0, 260.0] {
                for inclination_deg in [45.0, 150.0] {
                    let what =
                        format!("{method:?} {revolutions} {final_anomaly_deg} {inclination_deg}");
                    let e = 0.2;
                    let start = elements(
                        RADIUS,
                        e,
                        inclination_deg * D2R,
                        0.0,
                        30.0 * D2R,
                        20.0 * D2R,
                    );
                    let end = ClassicElements {
                        true_anomaly_rad: final_anomaly_deg * D2R,
                        ..start
                    };
                    let (r1, v1) = conic_state(MU, &start);
                    let (r2, v2) = conic_state(MU, &end);
                    let mean = |f| eccentric_to_mean_anomaly(true_to_eccentric_anomaly(f, e), e);
                    let transfer_time = ((mean(end.true_anomaly_rad)
                        - mean(start.true_anomaly_rad))
                    .rem_euclid(TAU)
                        + TAU * revolutions as f64)
                        / (MU / RADIUS.powf(3.0)).sqrt();
                    let direction = if inclination_deg > 90.0 {
                        Retrograde
                    } else {
                        Prograde
                    };
                    let (solution, _) =
                        solve_problem(r1, r2, transfer_time, revolutions, method, direction);
                    assert!(solution.first.valid, "{what}");
                    let branch = if revolutions == 0 {
                        assert!(!solution.second.valid, "{what}");
                        solution.first
                    } else {
                        assert!(solution.second.valid, "{what}");
                        if (solution.first.v1_n_mps - v1).norm()
                            < (solution.second.v1_n_mps - v1).norm()
                        {
                            solution.first
                        } else {
                            solution.second
                        }
                    };
                    assert_vector_close(branch.v1_n_mps, v1, 0.0, 1e-5, &what);
                    assert_vector_close(branch.v2_n_mps, v2, 0.0, 1e-5, &what);
                }
            }
        }
    }
}

/// Upstream `test_issue_1652_reported_retrograde_orbit`: the reported
/// retrograde orbit is recovered, and the default prograde solution kept.
#[test]
fn issue_1652_reported_retrograde_orbit() {
    let r1 = Vector3::new(5660550.077, -2909642.491, 3387572.056);
    let r2 = Vector3::new(-7203517.962, -4382872.685, -734884.231);
    let transfer_time = 2228.114;
    let known_velocity = Vector3::new(-4029.260, -6525.015, 1390.544);
    assert_eq!(LambertProblemMsg::default().transfer_direction, Prograde);
    for method in METHODS {
        let mut module = new_solver();
        let (default, _) = solve_with(
            &mut module,
            problem(
                r1,
                r2,
                transfer_time,
                0,
                method,
                LambertTransferDirection::default(),
            ),
        );
        for direction in [Prograde, Retrograde, Prograde] {
            let what = format!("{method:?} {direction:?}");
            let (solution, _) = solve_with(
                &mut module,
                problem(r1, r2, transfer_time, 0, method, direction),
            );
            assert!(solution.first.valid && !solution.second.valid, "{what}");
            let momentum = r1.cross(&solution.first.v1_n_mps);
            if direction == Retrograde {
                assert_vector_close(solution.first.v1_n_mps, known_velocity, 0.0, 2e-3, &what);
                assert!(momentum.z < 0.0, "{what}");
                let (position, velocity) =
                    kepler_propagate(r1, solution.first.v1_n_mps, transfer_time, MU);
                assert_vector_close(position, r2, 0.0, 1e-2, &what);
                assert_vector_close(velocity, solution.first.v2_n_mps, 0.0, 1e-5, &what);
            } else {
                assert_eq!(solution.first, default.first, "{what}");
                assert!(momentum.z > 0.0, "{what}");
            }
        }
    }
}

/// Upstream `test_directed_unbound_orbit`: parabolic and hyperbolic
/// Keplerian velocities in both directions.
#[test]
fn directed_unbound_orbit() {
    for method in METHODS {
        for e in [1.0, 1.2] {
            for inclination_deg in [30.0, 150.0] {
                let what = format!("{method:?} {e} {inclination_deg}");
                let start = elements(
                    -RADIUS,
                    e,
                    inclination_deg * D2R,
                    35.0 * D2R,
                    30.0 * D2R,
                    10.0 * D2R,
                );
                let (r1, v1) = conic_state(MU, &start);
                let final_anomaly = 130.0 * D2R;
                let transfer_time = if e == 1.0 {
                    let t1 = (start.true_anomaly_rad / 2.0).tan();
                    let t2 = (final_anomaly / 2.0).tan();
                    (t2 + t2.powi(3) / 3.0 - t1 - t1.powi(3) / 3.0)
                        / (MU / (2.0 * RADIUS.powi(3))).sqrt()
                } else {
                    let mean = |f| {
                        hyperbolic_to_mean_hyperbolic_anomaly(true_to_hyperbolic_anomaly(f, e), e)
                    };
                    (mean(final_anomaly) - mean(start.true_anomaly_rad))
                        / (MU / RADIUS.powi(3)).sqrt()
                };
                let (r2, v2) = conic_state(
                    MU,
                    &ClassicElements {
                        true_anomaly_rad: final_anomaly,
                        ..start
                    },
                );
                let direction = if inclination_deg > 90.0 {
                    Retrograde
                } else {
                    Prograde
                };
                let (solution, _) = solve_problem(r1, r2, transfer_time, 0, method, direction);
                assert!(solution.first.valid && !solution.second.valid, "{what}");
                assert_vector_close(solution.first.v1_n_mps, v1, 0.0, 1e-5, &what);
                assert_vector_close(solution.first.v2_n_mps, v2, 0.0, 1e-5, &what);
            }
        }
    }
}

/// Upstream `test_polar_direction_tie`: an orbit normal with an exactly zero
/// z-component follows the documented tie convention.
#[test]
fn polar_direction_tie() {
    for method in METHODS {
        for direction in DIRECTIONS {
            let what = format!("{method:?} {direction:?}");
            let period = TAU * (RADIUS.powi(3) / MU).sqrt();
            let sign = direction_sign(direction);
            let fraction = if direction == Retrograde { 0.75 } else { 0.25 };
            let speed = (MU / RADIUS).sqrt();
            let (solution, _) = solve_problem(
                Vector3::new(RADIUS, 0.0, 0.0),
                Vector3::new(0.0, 0.0, RADIUS),
                period * fraction,
                0,
                method,
                direction,
            );
            assert!(solution.first.valid && !solution.second.valid, "{what}");
            let expected_v1 = Vector3::new(0.0, 0.0, sign * speed);
            let expected_v2 = Vector3::new(-sign * speed, 0.0, 0.0);
            assert_vector_close(solution.first.v1_n_mps, expected_v1, 0.0, 1e-5, &what);
            assert_vector_close(solution.first.v2_n_mps, expected_v2, 0.0, 1e-5, &what);
        }
    }
}

/// Upstream `test_invalid_transfer_direction`, `test_invalid_solver_method`
/// and `test_negative_revolution_count` (and their gtests) feed unknown codes
/// to the solver. Those values have no `LambertProblemMsg` representation;
/// what is left to check is that the wire format refuses them.
#[test]
fn unknown_method_and_direction_codes_cannot_be_expressed() {
    let valid = serde_json::to_value(LambertProblemMsg::default()).unwrap();
    for (field, value) in [
        ("transfer_direction", serde_json::json!(2)),
        ("transfer_direction", serde_json::json!(-1)),
        ("transfer_direction", serde_json::json!("Sideways")),
        ("solver_method", serde_json::json!(2)),
        ("solver_method", serde_json::json!("Battin")),
        ("num_revolutions", serde_json::json!(-1)),
    ] {
        let mut invalid = valid.clone();
        invalid[field] = value.clone();
        assert!(
            serde_json::from_value::<LambertProblemMsg>(invalid).is_err(),
            "{field} = {value} was accepted"
        );
    }
}

/// Upstream `test_rotated_polar_direction_roundoff`: unequal radii whose
/// separately normalized directions leave a small negative z cross product;
/// both branches follow the tie convention and match an axis-aligned reference
/// rotated about z.
#[test]
fn rotated_polar_direction_roundoff() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for revolutions in [0, 1, 3] {
                for azimuth in [0.0, 0.4, 1.7, 3.2] {
                    let what = format!("{method:?} {direction:?} {revolutions} {azimuth}");
                    let r1: Vector3<f64> = Vector3::new(1.0e6, 5.0e6, 6.0e6);
                    let r2: Vector3<f64> = Vector3::new(2.0e6, 10.0e6, -7.0e6);
                    let reference_r1 = Vector3::new(r1.x.hypot(r1.y), 0.0, r1.z);
                    let reference_r2 = Vector3::new(r2.x.hypot(r2.y), 0.0, r2.z);
                    let initial_azimuth = r1.y.atan2(r1.x);
                    let (r1, r2) = (rotation_z(azimuth) * r1, rotation_z(azimuth) * r2);
                    let reference_rotation = rotation_z(initial_azimuth + azimuth);
                    let transfer_time = 5000.0 * (2 * revolutions + 1) as f64;
                    let (reference, _) = solve_problem(
                        reference_r1,
                        reference_r2,
                        transfer_time,
                        revolutions,
                        method,
                        direction,
                    );
                    let (solution, _) =
                        solve_problem(r1, r2, transfer_time, revolutions, method, direction);
                    assert!(solution.first.valid && reference.first.valid, "{what}");
                    assert_eq!(solution.second.valid, revolutions > 0, "{what}");
                    assert_eq!(reference.second.valid, revolutions > 0, "{what}");
                    for (actual, expected) in [
                        (solution.first.v1_n_mps, reference.first.v1_n_mps),
                        (solution.first.v2_n_mps, reference.first.v2_n_mps),
                        (solution.second.v1_n_mps, reference.second.v1_n_mps),
                        (solution.second.v2_n_mps, reference.second.v2_n_mps),
                    ] {
                        assert_vector_close(
                            actual,
                            reference_rotation * expected,
                            0.0,
                            1e-7,
                            &what,
                        );
                    }
                    let mut velocities = vec![solution.first.v1_n_mps];
                    if revolutions > 0 {
                        velocities.push(solution.second.v1_n_mps);
                    }
                    for velocity in velocities {
                        let alignment = r1.cross(&velocity).dot(&r1.cross(&r2));
                        assert!(direction_sign(direction) * alignment > 0.0, "{what}");
                    }
                }
            }
        }
    }
}

/// Upstream `test_resolved_near_polar_direction`: a resolvable
/// angular-momentum sign on either side of the polar tie is kept.
#[test]
fn resolved_near_polar_direction() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for tilt in [-1e-10_f64, 1e-10] {
                let what = format!("{method:?} {direction:?} {tilt}");
                let r1 = Vector3::new(26.0_f64.sqrt() * 1e6, 0.0, 6e6);
                let r2 = Vector3::new(2.0 * 26.0_f64.sqrt() * 1e6, 0.0, -7e6);
                let (s, c) = tilt.sin_cos();
                let rotation_x = Matrix3::new(1.0, 0.0, 0.0, 0.0, c, -s, 0.0, s, c);
                let rotation = rotation_z(0.7) * rotation_x;
                // The unrotated normal is along +y, so its tilted z follows tilt.
                let short_angle = (tilt > 0.0) == (direction == Prograde);
                let reference_direction = if short_angle { Prograde } else { Retrograde };
                let transfer_time = 5000.0;
                let (reference, _) =
                    solve_problem(r1, r2, transfer_time, 0, method, reference_direction);
                let (rotated_r1, rotated_r2) = (rotation * r1, rotation * r2);
                let (solution, _) =
                    solve_problem(rotated_r1, rotated_r2, transfer_time, 0, method, direction);
                assert!(solution.first.valid && reference.first.valid, "{what}");
                assert_vector_close(
                    solution.first.v1_n_mps,
                    rotation * reference.first.v1_n_mps,
                    0.0,
                    1e-7,
                    &what,
                );
                assert_vector_close(
                    solution.first.v2_n_mps,
                    rotation * reference.first.v2_n_mps,
                    0.0,
                    1e-7,
                    &what,
                );
                let momentum = rotated_r1.cross(&solution.first.v1_n_mps);
                assert!(direction_sign(direction) * momentum.z > 0.0, "{what}");
            }
        }
    }
}

// ---------------------------------------------------------------------------
// test_lambertSolverValidation.py
// ---------------------------------------------------------------------------

/// A problem field upstream's `invalid_input_and_recovery` corrupts.
#[derive(Clone, Copy, Debug)]
enum Field {
    Mu(f64),
    TransferTime(f64),
    R1(Vector3<f64>),
    R2(Vector3<f64>),
}

impl Field {
    fn apply(self, problem: &mut LambertProblemMsg) {
        match self {
            Self::Mu(value) => problem.mu_m3ps2 = value,
            Self::TransferTime(value) => problem.transfer_time_s = value,
            Self::R1(value) => problem.r1_n_m = value,
            Self::R2(value) => problem.r2_n_m = value,
        }
    }

    fn error(self) -> LambertInputError {
        match self {
            Self::Mu(_) => LambertInputError::Mu,
            Self::TransferTime(_) => LambertInputError::TransferTime,
            Self::R1(_) => LambertInputError::R1,
            Self::R2(_) => LambertInputError::R2,
        }
    }
}

/// Upstream `invalid_input_and_recovery`: a malformed update is rejected
/// (upstream raises, matching the field name; here `solve` errs and the
/// update publishes nothing), then a corrected request solves as on a fresh
/// module, without a reset.
fn invalid_input_and_recovery(method: LambertSolverMethod, revolutions: usize, field: Field) {
    let what = format!("{method:?} {revolutions} {field:?}");
    let mut payload = problem(
        Vector3::new(RADIUS, 0.0, 0.0),
        Vector3::new(0.0, RADIUS, 0.0),
        if revolutions == 0 { 2000.0 } else { 10000.0 },
        revolutions,
        method,
        Prograde,
    );
    let source = Output::new(payload.clone());
    let mut module = new_solver();
    source.connect_to(&mut module.lambert_problem_in_msg);
    module.reset(&context(0));
    module.update(&context(0));
    let before = module.lambert_solution_out_msg.read();
    assert!(before.first.valid, "{what}");

    let original = payload.clone();
    field.apply(&mut payload);
    source.write(payload.clone());
    assert_eq!(module.solve(), Err(field.error()), "{what}");
    assert!(
        module
            .solve()
            .unwrap_err()
            .to_string()
            .contains(match field.error() {
                LambertInputError::Mu => "mu",
                LambertInputError::TransferTime => "transferTime",
                LambertInputError::R1 => "r1_N",
                LambertInputError::R2 => "r2_N",
            })
    );
    module.update(&context(0));
    assert_eq!(module.lambert_solution_out_msg.read(), before, "{what}");

    payload = LambertProblemMsg {
        transfer_time_s: original.transfer_time_s * 1.1,
        ..original
    };
    source.write(payload.clone());
    module.update(&context(0));
    let mut fresh = new_solver();
    let expected = solve_with(&mut fresh, payload);
    assert_eq!(module.lambert_solution_out_msg.read(), expected.0, "{what}");
    assert_eq!(
        module.lambert_performance_out_msg.read(),
        expected.1,
        "{what}"
    );
}

/// Upstream `test_invalid_scalar_input`.
#[test]
fn invalid_scalar_input() {
    for method in METHODS {
        for revolutions in [0, 1] {
            for value in [0.0, -1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                invalid_input_and_recovery(method, revolutions, Field::Mu(value));
                invalid_input_and_recovery(method, revolutions, Field::TransferTime(value));
            }
        }
    }
}

/// Upstream `test_nonfinite_position_input`.
#[test]
fn nonfinite_position_input() {
    for method in METHODS {
        for revolutions in [0, 1] {
            for component in 0..3 {
                for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                    let mut position = Vector3::new(RADIUS, RADIUS, RADIUS);
                    position[component] = value;
                    invalid_input_and_recovery(method, revolutions, Field::R1(position));
                    invalid_input_and_recovery(method, revolutions, Field::R2(position));
                }
            }
        }
    }
}

/// Upstream `test_unusable_position_norm`: zero, underflowed and overflowed
/// norms.
#[test]
fn unusable_position_norm() {
    for method in METHODS {
        for revolutions in [0, 1] {
            for magnitude in [0.0, 1e-300, 1e300] {
                let position = Vector3::new(magnitude, 0.0, 0.0);
                invalid_input_and_recovery(method, revolutions, Field::R1(position));
                invalid_input_and_recovery(method, revolutions, Field::R2(position));
            }
        }
    }
}

/// Upstream `test_invalid_alignment_threshold`: an invalid threshold is
/// rejected and the previous one kept.
#[test]
fn invalid_alignment_threshold() {
    for method in METHODS {
        for value in [
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            -1.0,
            0.0,
            90.0,
            91.0,
            180.0,
            360.0,
        ] {
            let mut module = new_solver();
            module.set_alignment_threshold_deg(2.0).unwrap();
            let error = module.set_alignment_threshold_deg(value).unwrap_err();
            assert!(
                matches!(error, InvalidAlignmentThreshold(rejected) if rejected.to_bits() == value.to_bits())
            );
            assert!(error.to_string().contains("alignmentThreshold"));
            assert_eq!(module.alignment_threshold_deg(), 2.0);
            let (solution, _) = solve_with(
                &mut module,
                problem(
                    Vector3::new(RADIUS, 0.0, 0.0),
                    Vector3::new(0.0, RADIUS, 0.0),
                    2000.0,
                    0,
                    method,
                    Prograde,
                ),
            );
            assert!(solution.first.valid, "{method:?} {value}");
        }
    }
}

/// Upstream `test_valid_alignment_threshold_boundaries`.
#[test]
fn valid_alignment_threshold_boundaries() {
    for value in [0.0_f64.next_up(), 1.0, 90.0_f64.next_down()] {
        let mut module = new_solver();
        module.set_alignment_threshold_deg(value).unwrap();
        assert_eq!(module.alignment_threshold_deg(), value);
        let (solution, _) = solve_with(
            &mut module,
            problem(
                Vector3::new(RADIUS, 0.0, 0.0),
                Vector3::new(0.0, RADIUS, 0.0),
                2000.0,
                0,
                Gooding,
                Prograde,
            ),
        );
        assert!(solution.first.valid, "{value}");
    }
}

/// Upstream `test_non_axis_aligned_collinearity`: parallel and antiparallel
/// endpoints are rejected despite normalization roundoff, clearing both
/// populated solutions, and a corrected request recovers without a reset.
#[test]
fn non_axis_aligned_collinearity() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for revolutions in [0, 1] {
                for threshold_deg in [0.0_f64.next_up(), 1e-15] {
                    for radial in [[1.0, 2.0, 3.0], [2.0, -5.0, 11.0], [-3.0, 7.0, -13.0]] {
                        for radius_ratio in [-3.0, 3.0] {
                            let what = format!(
                                "{method:?} {direction:?} {revolutions} {threshold_deg} {radial:?} {radius_ratio}"
                            );
                            let r1 = Vector3::from(radial) * 1e6;
                            let r2 = r1 * radius_ratio;
                            assert_eq!(r1.cross(&r2), Vector3::zeros());
                            // Above the one-revolution minimum even for the
                            // largest antiparallel case, so an infeasible time
                            // cannot mask a missing collinearity check.
                            let transfer_time = if revolutions == 0 { 10000.0 } else { 100000.0 };
                            let mut module = new_solver();
                            module.set_alignment_threshold_deg(threshold_deg).unwrap();
                            let source = Output::new(LambertProblemMsg::default());
                            source.connect_to(&mut module.lambert_problem_in_msg);
                            module.reset(&context(0));
                            let axis_r1 = Vector3::new(RADIUS, 0.0, 0.0);
                            let axis_r2 = Vector3::new(0.0, RADIUS, 0.0);
                            let expected = solve_problem(
                                axis_r1,
                                axis_r2,
                                transfer_time,
                                revolutions,
                                method,
                                direction,
                            );
                            assert!(expected.0.first.valid, "{what}");
                            assert_eq!(expected.0.second.valid, revolutions > 0, "{what}");
                            for aligned in [false, true, false] {
                                let (p1, p2) = if aligned {
                                    (r1, r2)
                                } else {
                                    (axis_r1, axis_r2)
                                };
                                source.write(problem(
                                    p1,
                                    p2,
                                    transfer_time,
                                    revolutions,
                                    method,
                                    direction,
                                ));
                                module.update(&context(0));
                                let solution = module.lambert_solution_out_msg.read();
                                let performance = module.lambert_performance_out_msg.read();
                                if aligned {
                                    assert_cleared(&solution, &performance);
                                } else {
                                    assert_eq!(solution, expected.0, "{what}");
                                    assert_eq!(performance, expected.1, "{what}");
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Upstream `test_resolvable_near_collinear_plane`: small cross products
/// above cancellation roundoff are kept.
#[test]
fn resolvable_near_collinear_plane() {
    for method in METHODS {
        for direction in DIRECTIONS {
            for radial in [[1.0, 0.0, 0.0], [1.0, 2.0, 3.0]] {
                for radius_ratio in [-3.0, 3.0] {
                    let r1 = Vector3::from(radial) * 1e6;
                    let transverse = Vector3::new(-radial[1], radial[0], 0.0);
                    let r2 = r1 * radius_ratio + transverse * 1e-5;
                    let mut module = new_solver();
                    module.set_alignment_threshold_deg(1e-15).unwrap();
                    let (solution, _) =
                        solve_with(&mut module, problem(r1, r2, 10000.0, 0, method, direction));
                    assert!(
                        solution.first.valid,
                        "{method:?} {direction:?} {radial:?} {radius_ratio}"
                    );
                    assert!(finite(&[solution.first.v1_n_mps, solution.first.v2_n_mps]));
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// test_lambertInputValidation.cpp and test_lambertTransferDirection.cpp
// ---------------------------------------------------------------------------

/// gtest `validProblem`: an ordinary prograde transfer with the zeroed
/// default (Izzo) method.
fn valid_problem() -> LambertProblemMsg {
    LambertProblemMsg {
        mu_m3ps2: 3.986004360e14,
        transfer_time_s: 2000.0,
        r1_n_m: Vector3::new(8.0e6, 0.0, 0.0),
        r2_n_m: Vector3::new(0.0, 8.0e6, 0.0),
        ..Default::default()
    }
}

/// gtests `DefaultSolverMethodIsIzzo` and `DefaultsToPrograde`.
#[test]
fn zeroed_problem_defaults_to_izzo_and_prograde() {
    let problem = valid_problem();
    assert_eq!(problem.solver_method, Izzo);
    assert_eq!(problem.solver_method as u8, 0);
    assert_eq!(LambertProblemMsg::default().transfer_direction, Prograde);
    assert_eq!(LambertProblemMsg::default().transfer_direction as u8, 0);
    let mut module = new_solver();
    let (solution, _) = solve_with(&mut module, problem);
    assert!(solution.first.valid);
}

/// gtest `RejectsMalformedInputsBeforeGeometry`: zero and non-finite `mu`,
/// flight time and position components are rejected. (Upstream also checks
/// that no floating-point exception was raised on the way.)
#[test]
fn rejects_malformed_inputs_before_geometry() {
    for method in METHODS {
        for value in [0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut fields = vec![Field::Mu(value), Field::TransferTime(value)];
            for component in 0..3 {
                let mut position = Vector3::zeros();
                position[component] = value;
                fields.extend([Field::R1(position), Field::R2(position)]);
            }
            for field in fields {
                let mut payload = LambertProblemMsg {
                    solver_method: method,
                    ..valid_problem()
                };
                field.apply(&mut payload);
                let mut module = new_solver();
                Output::new(payload).connect_to(&mut module.lambert_problem_in_msg);
                assert_eq!(module.solve(), Err(field.error()), "{method:?} {field:?}");
            }
        }
    }
}

/// gtest `TinyThresholdStillRejectsCollinearPositions`.
#[test]
fn tiny_threshold_still_rejects_collinear_positions() {
    for method in METHODS {
        let mut module = new_solver();
        module
            .set_alignment_threshold_deg(0.0_f64.next_up())
            .unwrap();
        let source = Output::new(LambertProblemMsg::default());
        source.connect_to(&mut module.lambert_problem_in_msg);
        module.reset(&context(0));
        for radial in [[1.0, 0.0, 0.0], [1.0, 2.0, 3.0]] {
            for radius_ratio in [-3.0, 3.0] {
                let r1 = Vector3::from(radial) * 1.0e6;
                source.write(LambertProblemMsg {
                    solver_method: method,
                    r1_n_m: r1,
                    r2_n_m: r1 * radius_ratio,
                    ..valid_problem()
                });
                module.update(&context(0));
                let solution = module.lambert_solution_out_msg.read();
                assert!(
                    !solution.first.valid && !solution.second.valid,
                    "{method:?} {radial:?}"
                );
            }
        }
    }
}

/// gtest `RejectsCollinearityBeforeLambdaAndTimeScaling`: antiparallel
/// endpoints that could round `1 - c/s` negative, or underflow `s^3`.
#[test]
fn rejects_collinearity_before_lambda_and_time_scaling() {
    for method in METHODS {
        for threshold_deg in [1.0e-15, 1.0] {
            for position in [
                Vector3::new(-5.10e6, -8.05e6, 3.59e6),
                Vector3::new(-5.10e-110, -8.05e-110, 3.59e-110),
            ] {
                let mut module = new_solver();
                module.set_alignment_threshold_deg(threshold_deg).unwrap();
                let (solution, performance) = solve_with(
                    &mut module,
                    LambertProblemMsg {
                        solver_method: method,
                        transfer_time_s: 10000.0,
                        r1_n_m: position,
                        r2_n_m: position * -7.0,
                        ..valid_problem()
                    },
                );
                assert_cleared(&solution, &performance);
            }
        }
    }
}

/// The recovery half of gtest `SolverRejectsInvalidIntegers`: a retrograde
/// request after the rejected ones solves the retrograde transfer.
#[test]
fn retrograde_request_solves_the_retrograde_transfer() {
    for method in METHODS {
        let mut module = new_solver();
        let (solution, _) = solve_with(
            &mut module,
            LambertProblemMsg {
                solver_method: method,
                transfer_direction: Retrograde,
                ..valid_problem()
            },
        );
        assert!(solution.first.valid, "{method:?}");
        assert!(solution.first.v1_n_mps.y < 0.0, "{method:?}");
    }
}

/// Not upstream: an unconnected input reads the zeroed problem, whose zero
/// `mu` is malformed, so the module publishes nothing past its initial
/// zeroed outputs.
#[test]
fn unconnected_input_publishes_only_the_initial_zeroed_outputs() {
    let mut module = new_solver();
    module.update(&context(0));
    assert_eq!(module.solve(), Err(LambertInputError::Mu));
    assert_cleared(
        &module.lambert_solution_out_msg.read(),
        &module.lambert_performance_out_msg.read(),
    );
}
