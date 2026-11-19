//! Ports of upstream `lambertValidator/_UnitTest/test_lambertValidator.py`
//! and of `test_converged_solution_passes_validator` from
//! `lambertSolver/_UnitTest/test_lambertSolverConvergence.py`, with their
//! parameters and tolerances. The truth of the first propagates every
//! dispersed state with Kepler's equation, as upstream does.

use nalgebra::{Matrix6, Vector3, Vector6};

use super::{LambertValidator, LambertValidatorConfig, LambertValidatorError};
use crate::Module;
use crate::fsw_algorithms::orbit_control::test_support::{
    DIRECTIONS, METHODS, MU, RADIUS, conic_state, context, elements, problem, solve_problem,
};
use crate::messages::{
    DvBurnCommandMsg, LambertPerformanceMsg, LambertProblemMsg, LambertRootPerformance,
    LambertSolutionMsg, LambertSolutionSpecifier, LambertSolverMethod, LambertTransferDirection,
    LambertTransferVelocities, LambertValidatorMsg, NavigationTranslationMsg, Output,
};
use crate::utilities::astro_constants::D2R;
use crate::utilities::orbital_motion::{
    ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly, hill_frame,
    hyperbolic_to_mean_hyperbolic_anomaly, hyperbolic_to_true_anomaly,
    mean_hyperbolic_to_hyperbolic_anomaly, mean_to_eccentric_anomaly, state_to_elements,
    true_to_eccentric_anomaly, true_to_hyperbolic_anomaly,
};

/// The upstream test's ports, wired to their sources.
struct Fixture {
    module: LambertValidator,
    navigation: Output<NavigationTranslationMsg>,
    problem: Output<LambertProblemMsg>,
    solution: Output<LambertSolutionMsg>,
    performance: Output<LambertPerformanceMsg>,
}

fn fixture(
    config: LambertValidatorConfig,
    navigation: NavigationTranslationMsg,
    problem: LambertProblemMsg,
    solution: LambertSolutionMsg,
    performance: LambertPerformanceMsg,
) -> Fixture {
    let mut module = LambertValidator::new(config);
    let fixture_navigation = Output::new(navigation);
    let fixture_problem = Output::new(problem);
    let fixture_solution = Output::new(solution);
    let fixture_performance = Output::new(performance);
    fixture_navigation.connect_to(&mut module.nav_trans_in_msg);
    fixture_problem.connect_to(&mut module.lambert_problem_in_msg);
    fixture_solution.connect_to(&mut module.lambert_solution_in_msg);
    fixture_performance.connect_to(&mut module.lambert_performance_in_msg);
    Fixture {
        module,
        navigation: fixture_navigation,
        problem: fixture_problem,
        solution: fixture_solution,
        performance: fixture_performance,
    }
}

fn navigation(time_tag_s: f64, r: Vector3<f64>, v: Vector3<f64>) -> NavigationTranslationMsg {
    NavigationTranslationMsg {
        time_tag_s,
        position_bn_n_m: r,
        velocity_bn_n_mps: v,
        ..Default::default()
    }
}

fn first_solution(v1: Vector3<f64>, v2: Vector3<f64>) -> LambertSolutionMsg {
    LambertSolutionMsg {
        first: LambertTransferVelocities {
            v1_n_mps: v1,
            v2_n_mps: v2,
            valid: true,
        },
        second: LambertTransferVelocities::default(),
    }
}

/// The true anomaly `dt` after `elements`' (upstream's elliptic, parabolic
/// and hyperbolic Kepler propagation; `e` is the eccentricity it uses).
fn propagated_anomaly(mu: f64, elements: &ClassicElements, e: f64, dt: f64) -> f64 {
    let f = elements.true_anomaly_rad;
    let a = elements.semi_major_axis;
    if e < 1.0 {
        let m = eccentric_to_mean_anomaly(true_to_eccentric_anomaly(f, e), e);
        let n = (mu / a.powf(3.0)).sqrt();
        eccentric_to_true_anomaly(mean_to_eccentric_anomaly(m + n * dt, e), e)
    } else if e == 1.0 {
        let d = (f / 2.0).tan();
        let m = d + 1.0 / 3.0 * d.powf(3.0);
        let n = (mu / (2.0 * (-a).powf(3.0))).sqrt();
        let big_a = 3.0 / 2.0 * (m + n * dt);
        let big_b = (big_a + (big_a * big_a + 1.0).sqrt()).powf(1.0 / 3.0);
        2.0 * (big_b - 1.0 / big_b).atan()
    } else {
        let n0 = hyperbolic_to_mean_hyperbolic_anomaly(true_to_hyperbolic_anomaly(f, e), e);
        let n = (mu / (-a).powf(3.0)).sqrt();
        hyperbolic_to_true_anomaly(mean_hyperbolic_to_hyperbolic_anomaly(n0 + n * dt, e), e)
    }
}

/// The test's `getInitialStates`, a transcription independent of the
/// module's: inertial columns of the 27 dispersed states.
fn upstream_initial_states(
    r1: Vector3<f64>,
    v1: Vector3<f64>,
    dv: Vector3<f64>,
    err_states: &Matrix6<f64>,
    err_dv: f64,
) -> Vec<Vector6<f64>> {
    let dcm_hn = hill_frame(r1, v1);
    let mut dcm_states = Matrix6::zeros();
    dcm_states.fixed_view_mut::<3, 3>(0, 0).copy_from(&dcm_hn);
    dcm_states.fixed_view_mut::<3, 3>(3, 3).copy_from(&dcm_hn);
    let to_n = dcm_states.transpose();
    let mut nominal = Vector6::zeros();
    nominal.fixed_rows_mut::<3>(0).copy_from(&(dcm_hn * r1));
    nominal.fixed_rows_mut::<3>(3).copy_from(&(dcm_hn * v1));
    let dv_h = dcm_hn * dv;
    let dv_hat = dv_h / dv_h.norm();
    let with_dv = |mut x: Vector6<f64>, burn: Vector3<f64>| {
        let mut velocity = x.fixed_rows_mut::<3>(3);
        velocity += burn;
        to_n * x
    };

    let mut states = vec![Vector6::zeros(); 27];
    for ii in 0..6 {
        for jj in 0..2 {
            let multiplier = if jj == 0 { 1.0 } else { -1.0 };
            let x_h = nominal + err_states.column(ii) * multiplier;
            states[jj * 6 + ii] = with_dv(x_h, dv_h - dv_hat * err_dv);
            states[12 + jj * 6 + ii] = with_dv(x_h, dv_h + dv_hat * err_dv);
        }
    }
    states[24] = with_dv(nominal, dv_h);
    states[25] = with_dv(nominal, dv_h - dv_hat * err_dv);
    states[26] = with_dv(nominal, dv_h + dv_hat * err_dv);
    states
}

/// The test's `countViolations`: each dispersed state goes to elements and
/// Kepler's equation to the final time. Its minimum radius is the maneuver
/// radius: upstream copies the maneuver elements, changes their anomaly, and
/// reads `rmag`, which the copy does not recompute.
fn upstream_count_violations(
    states: &[Vector6<f64>],
    mu: f64,
    tm: f64,
    tf: f64,
    r_tn_n: Vector3<f64>,
    max_distance_target: f64,
    min_orbit_radius: f64,
) -> (usize, usize) {
    let (mut distance, mut radius) = (0, 0);
    for x in states {
        let rm = x.fixed_rows::<3>(0).into_owned();
        let vm = x.fixed_rows::<3>(3).into_owned();
        let oem = state_to_elements(mu, rm, vm);
        let oef = ClassicElements {
            true_anomaly_rad: propagated_anomaly(mu, &oem, oem.eccentricity, tf - tm),
            ..oem
        };
        let (rf, _) = conic_state(mu, &oef);
        if (rf - r_tn_n).norm() > max_distance_target {
            distance += 1;
        }
        if oem.orbit_radius < min_orbit_radius {
            radius += 1;
        }
    }
    (distance, radius)
}

/// Upstream `test_lambertValidator`: for two delta-vs, maneuver and final
/// times, and root-finder iteration counts and errors, four updates with the
/// delta-v scaled by 1, 1.5, 1, 1 command it only on the last (the first that
/// repeats the previous one), and only when every check passes. A maneuver
/// after the final time must be reported at reset, where upstream raises.
#[test]
fn matches_upstream_lambert_validator_test() {
    let accuracy = 1e-4;
    let dvs = [
        Vector3::new(1.0, 3.0, 4.0),
        Vector3::new(500.0, -100.0, 200.0),
    ];
    let mut cases = Vec::new();
    for dv in dvs {
        for tm in [1.0e3, 2.0e3] {
            for tf in [2.1e3, 4.0e3] {
                cases.push((dv, tm, tf, 3, 1e-9));
            }
        }
    }
    cases.extend([
        (dvs[0], 1.0e3, 2.0e3, 7, 1.0e-9),
        (dvs[0], 1.0e3, 2.0e3, 3, 1.0e-5),
        (dvs[0], 2.0e3, 1.0e3, 3, 1.0e-9),
        (dvs[0], 2.0e3, 2.0e3, 3, 1.0e-9),
    ]);

    let mu = 3.986004418e14;
    let t0 = 0.0;
    let ecc = 0.1;
    let err_states = Matrix6::from_diagonal(&Vector6::new(5.0, 5.0, 5.0, 0.01, 0.01, 0.001));
    let err_dv = 0.1;
    let max_distance_target = 3000.0;
    let min_orbit_radius = 6378.0 * 1000.0;

    for (dv, tm, tf, num_iter, err_x) in cases {
        let what = format!("dv={dv:?}, tm={tm}, tf={tf}, iterations={num_iter}, errX={err_x}");
        let start = elements(
            10000.0 * 1000.0,
            ecc,
            10.0 * D2R,
            20.0 * D2R,
            30.0 * D2R,
            40.0 * D2R,
        );
        let (r0, v0) = conic_state(mu, &start);
        let oe0 = state_to_elements(mu, r0, v0);
        let oe1 = ClassicElements {
            true_anomaly_rad: propagated_anomaly(mu, &oe0, ecc, tm - t0),
            ..oe0
        };
        let (r1, v1) = conic_state(mu, &oe1);
        let oe2 = state_to_elements(mu, r1, v1 + dv);
        let oe3 = ClassicElements {
            true_anomaly_rad: propagated_anomaly(mu, &oe2, oe2.eccentricity, tf - tm),
            ..oe2
        };
        let (r3, v3) = conic_state(mu, &oe3);

        let config = LambertValidatorConfig {
            final_time_s: tf,
            maneuver_time_s: tm,
            max_distance_target_m: max_distance_target,
            min_orbit_radius_m: min_orbit_radius,
            uncertainty_states: err_states,
            uncertainty_dv_mps: err_dv,
            dv_convergence_tolerance_mps: dv.norm() / 1000.0,
            ..Default::default()
        };
        let mut fixture = fixture(
            config,
            navigation(t0, r0, v0),
            LambertProblemMsg {
                mu_m3ps2: mu,
                ..problem(
                    r1,
                    r3,
                    tf - tm,
                    0,
                    LambertSolverMethod::Izzo,
                    LambertTransferDirection::Prograde,
                )
            },
            first_solution(v1 + dv, v3),
            LambertPerformanceMsg {
                first: LambertRootPerformance {
                    x: 0.0,
                    num_iter,
                    err_x,
                },
                second: LambertRootPerformance::default(),
            },
        );
        let errors = fixture.module.configuration_errors();
        if tm > tf {
            assert_eq!(
                errors,
                [LambertValidatorError::FinalTimeBeforeManeuver],
                "{what}"
            );
            continue;
        }
        assert!(errors.is_empty(), "{what}: {errors:?}");
        fixture.module.init();

        let failed_dv_solution_convergence = [true, true, true, false];
        for (step, scaler) in [1.0, 1.5, 1.0, 1.0].into_iter().enumerate() {
            let what = format!("{what}, step {step}");
            fixture.solution.write(first_solution(v1 + dv * scaler, v3));
            fixture
                .module
                .update(&context(step as u64 * 100_000_000_000));
            let command = fixture.module.dv_burn_cmd_out_msg.read();
            let diagnostics = fixture.module.lambert_validator_out_msg.read();

            let states = upstream_initial_states(r1, v1, dv * scaler, &err_states, err_dv);
            let (distance, radius) = upstream_count_violations(
                &states,
                mu,
                tm,
                tf,
                r3,
                max_distance_target,
                min_orbit_radius,
            );
            let commanded = distance == 0
                && radius == 0
                && !failed_dv_solution_convergence[step]
                && num_iter < 6
                && err_x < 1.0e-8;
            let (dv_true, burn_start_true) = if commanded {
                (dv * scaler, (tm * 1e9) as u64)
            } else {
                (Vector3::zeros(), 0)
            };

            assert!(
                (command.dv_inertial_cmd_mps - dv_true).amax() <= accuracy,
                "{what}: dv_N {command:?}"
            );
            assert_eq!(
                command.burn_start_time_nanos, burn_start_true,
                "{what}: burnStartTime"
            );
            assert_eq!(
                diagnostics.failed_num_iterations_lambert,
                num_iter >= 6,
                "{what}"
            );
            assert_eq!(
                diagnostics.failed_x_tolerance_lambert,
                err_x >= 1.0e-8,
                "{what}"
            );
            assert_eq!(
                diagnostics.failed_dv_solution_convergence, failed_dv_solution_convergence[step],
                "{what}"
            );
            assert_eq!(
                diagnostics.failed_distance_target_constraint,
                distance != 0,
                "{what}"
            );
            assert_eq!(
                diagnostics.failed_orbit_radius_constraint,
                radius != 0,
                "{what}"
            );
            assert!(
                (diagnostics.dv_n_mps - dv * scaler).amax() <= accuracy,
                "{what}: dvTheory"
            );
        }
    }
}

/// Upstream `test_converged_solution_passes_validator`
/// (`test_lambertSolverConvergence.py`): converged solver output passes the
/// validator's default convergence checks on the second identical command,
/// whose delta-v the 1 m target box then lets through.
#[test]
fn converged_solution_passes_validator() {
    let cases = [
        ((-8007153.311464242, 1134336.721226995), 1517.1851310277925),
        (
            (-1373963.0772790438, 21169783.315275904),
            1029.9495735576572,
        ),
        (
            (-1160389.9429338065, 9261648.1844296623),
            1347.9302728188266,
        ),
        ((7823692.80546992, 1670278.6850218216), 168.1732550120743),
    ];
    for method in METHODS {
        for direction in DIRECTIONS {
            for ((x2, y2), transfer_time) in cases {
                let what = format!("{method:?} {direction:?} {transfer_time}");
                let r1 = Vector3::new(RADIUS, 0.0, 0.0);
                let mut r2 = Vector3::new(x2, y2, 0.0);
                if direction == LambertTransferDirection::Retrograde {
                    r2.y = -r2.y;
                }
                let (solution, performance) =
                    solve_problem(r1, r2, transfer_time, 0, method, direction);
                let expected_dv = Vector3::new(1.0, 0.0, 0.0);
                let mut fixture = fixture(
                    LambertValidatorConfig {
                        final_time_s: transfer_time,
                        uncertainty_states: Matrix6::zeros(),
                        uncertainty_dv_mps: 0.0,
                        max_distance_target_m: 1.0,
                        ..Default::default()
                    },
                    navigation(0.0, r1, solution.first.v1_n_mps - expected_dv),
                    problem(r1, r2, transfer_time, 0, method, direction),
                    solution,
                    performance,
                );
                fixture.module.reset(&context(0));
                // Two identical solutions satisfy the default checks on the
                // second command.
                fixture.module.update(&context(0));
                fixture.module.update(&context(0));
                let diagnostics = fixture.module.lambert_validator_out_msg.read();
                assert!(!diagnostics.failed_valid_lambert, "{what}");
                assert!(!diagnostics.failed_num_iterations_lambert, "{what}");
                assert!(!diagnostics.failed_x_tolerance_lambert, "{what}");
                let command = fixture.module.dv_burn_cmd_out_msg.read();
                assert!(
                    (command.dv_inertial_cmd_mps - expected_dv).amax() <= 1e-10,
                    "{what}: {command:?} {diagnostics:?}"
                );
            }
        }
    }
}

/// A validator on a circular orbit whose Lambert solution is the orbit
/// itself plus `dv`, with no dispersion: the base of the tests below.
fn circular_fixture(dv: Vector3<f64>, config: LambertValidatorConfig) -> (Fixture, Vector3<f64>) {
    let r = Vector3::new(RADIUS, 0.0, 0.0);
    let v = Vector3::new(0.0, (MU / RADIUS).sqrt(), 0.0);
    let fixture = fixture(
        LambertValidatorConfig {
            final_time_s: 100.0,
            max_distance_target_m: 1e9,
            ..config
        },
        navigation(0.0, r, v),
        problem(
            r,
            r,
            100.0,
            0,
            LambertSolverMethod::Izzo,
            LambertTransferDirection::Prograde,
        ),
        first_solution(v + dv, Vector3::zeros()),
        LambertPerformanceMsg::default(),
    );
    (fixture, v)
}

/// Not upstream: the second solution is used when specified, and its
/// diagnostics with it.
#[test]
fn second_solution_specifier_reads_the_second_slots() {
    let dv = Vector3::new(0.5, -0.2, 0.1);
    let (mut fixture, v) = circular_fixture(
        Vector3::zeros(),
        LambertValidatorConfig {
            lambert_solution_specifier: LambertSolutionSpecifier::Second,
            ..Default::default()
        },
    );
    fixture.solution.write(LambertSolutionMsg {
        first: LambertTransferVelocities::default(),
        second: LambertTransferVelocities {
            v1_n_mps: v + dv,
            v2_n_mps: Vector3::zeros(),
            valid: true,
        },
    });
    fixture.performance.write(LambertPerformanceMsg {
        first: LambertRootPerformance {
            x: 0.3,
            num_iter: 9,
            err_x: 1.0,
        },
        second: LambertRootPerformance {
            x: 0.0,
            num_iter: 2,
            err_x: 1e-12,
        },
    });
    fixture.module.init();
    fixture.module.update(&context(0));
    fixture.module.update(&context(0));
    let diagnostics = fixture.module.lambert_validator_out_msg.read();
    assert!(!diagnostics.failed_valid_lambert);
    assert!(!diagnostics.failed_num_iterations_lambert);
    assert!(!diagnostics.failed_x_tolerance_lambert);
    let command = fixture.module.dv_burn_cmd_out_msg.read();
    assert!((command.dv_inertial_cmd_mps - dv).amax() < 1e-9);
    assert_eq!(command.dv_rot_vec_unit, Vector3::x());
}

/// Not upstream: ignoring constraint violations commands any valid solution,
/// even on the first update, but never an invalid one.
#[test]
fn ignoring_constraints_commands_any_valid_solution() {
    let dv = Vector3::new(3.0, 0.0, 0.0);
    let (mut fixture, v) = circular_fixture(
        dv,
        LambertValidatorConfig {
            ignore_constraint_violations: true,
            max_distance_target_m: 0.0,
            min_orbit_radius_m: 1e12,
            maneuver_time_s: 20.0,
            ..Default::default()
        },
    );
    fixture.module.init();
    fixture.module.update(&context(0));
    let command = fixture.module.dv_burn_cmd_out_msg.read();
    assert!(command.dv_inertial_cmd_mps.norm() > 2.0);
    assert_eq!(command.burn_start_time_nanos, 20_000_000_000);
    let diagnostics = fixture.module.lambert_validator_out_msg.read();
    assert_eq!(diagnostics.violations_distance_target, 0, "not propagated");

    fixture.solution.write(LambertSolutionMsg {
        first: LambertTransferVelocities {
            valid: false,
            ..first_solution(v + dv, Vector3::zeros()).first
        },
        ..Default::default()
    });
    fixture.module.update(&context(0));
    assert_eq!(
        fixture.module.dv_burn_cmd_out_msg.read(),
        DvBurnCommandMsg::default()
    );
    assert!(
        fixture
            .module
            .lambert_validator_out_msg
            .read()
            .failed_valid_lambert
    );
}

/// Not upstream: the update-time checks upstream raises from `readMessages`
/// publish nothing and leave the history alone; a reset keeps the history,
/// as upstream's.
#[test]
fn update_errors_publish_nothing_and_reset_keeps_the_history() {
    let dv = Vector3::new(0.0, 1.0, 0.0);
    let (mut fixture, _) = circular_fixture(
        dv,
        LambertValidatorConfig {
            maneuver_time_s: 10.0,
            ..Default::default()
        },
    );
    fixture.module.init();
    fixture.module.update(&context(0));
    let diagnostics = fixture.module.lambert_validator_out_msg.read();
    assert!(diagnostics.failed_dv_solution_convergence);

    let navigation_before = fixture.module.nav_trans_in_msg.read();
    fixture.navigation.write(NavigationTranslationMsg {
        time_tag_s: 11.0,
        ..navigation_before.clone()
    });
    assert_eq!(
        fixture.module.validate(),
        Err(LambertValidatorError::ManeuverTimePassed(11.0))
    );
    fixture.navigation.write(navigation_before);
    let problem_before = fixture.module.lambert_problem_in_msg.read();
    fixture.problem.write(LambertProblemMsg {
        mu_m3ps2: 0.0,
        ..problem_before.clone()
    });
    fixture.module.update(&context(0));
    assert_eq!(fixture.module.lambert_validator_out_msg.read(), diagnostics);
    assert_eq!(
        fixture.module.validate(),
        Err(LambertValidatorError::NonPositiveMu(0.0))
    );
    fixture.problem.write(problem_before);

    fixture.module.reset(&context(0));
    fixture.module.update(&context(0));
    // The same delta-v as the last successful update: converged.
    let diagnostics = fixture.module.lambert_validator_out_msg.read();
    assert!(
        !diagnostics.failed_dv_solution_convergence,
        "{diagnostics:?}"
    );
    assert_eq!(diagnostics.dv_solution_difference_mps, 0.0);
    assert_ne!(
        fixture.module.dv_burn_cmd_out_msg.read(),
        DvBurnCommandMsg::default()
    );
}

/// Not upstream: the reset checks.
#[test]
fn reports_unconnected_inputs() {
    let module = LambertValidator::new(LambertValidatorConfig::default());
    assert_eq!(
        module.configuration_errors(),
        [
            LambertValidatorError::InputNotConnected("nav_trans_in_msg"),
            LambertValidatorError::InputNotConnected("lambert_problem_in_msg"),
            LambertValidatorError::InputNotConnected("lambert_solution_in_msg"),
            LambertValidatorError::InputNotConnected("lambert_performance_in_msg"),
        ]
    );
    assert_eq!(LambertValidatorMsg::default().violations_orbit_radius, 0);
}
