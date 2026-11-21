//! Port of upstream `examples/dynamicsComparison/scenarioCompareTorque.py`:
//! a 750 kg rigid body under a constant body torque `[0.2, -0.3, 0.4] N m`
//! from an off-principal-axis initial rate, propagated by the
//! Backsubstitution spacecraft (an `ExtForceTorque` on the hub) and, with
//! the `mujoco` feature, by a free `MjScene` body (a torque actuator at its
//! origin site, high-order attitude integration). See
//! `support/dyncomp_torque.rs`.
//!
//! Each engine runs 600 s with RK4 at 0.1 s twice, on the Keplerian orbit
//! and at rest without gravity: neither gravity nor the translation can
//! torque a compact body about its center of mass, so the attitude change
//! between the two runs is a per-engine error (exactly zero for BSM, a
//! velocity-dependent round-off in MuJoCo's gyroscopic bias). A third run
//! confines the rate and torque to the body z principal axis, whose exact
//! solution `ω_z = ω_z0 + L_z t / I_z`, `θ_z = ω_z0 t + L_z t² / (2 I_z)`
//! both engines are compared with. Attitudes are compared by the principal
//! angle of the relative DCM, `4 atan |σ_rel|`.
//!
//! Outputs, in `examples/output/scenarios/`: `scenarioCompareTorque.csv`
//! (body rates and every attitude- and rate-error history, for
//! `plot_scenario_compare_torque.py`), `scenarioCompareTorque.json`
//! (upstream's metrics) and, with `--timing`,
//! `scenarioCompareTorque_runtime.csv`.
//!
//! `tests` ports `_assert_torque_metrics` of upstream's
//! `test_scenarioDynamicsComparison.py`; without the `mujoco` feature only
//! its BSM bounds run.

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_torque.rs"]
mod dyncomp_torque;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/table_csv.rs"]
mod table_csv;

use basilisk_rs::utilities::rigid_body_kinematics::{dcm_to_mrp, mrp_to_dcm, prv_to_dcm};
use dyncomp::{
    DEFAULT_TIMING_REPEATS, Series, max_of, relative_principal_angles, save_runtime_table, seconds,
    validate_history, validate_matching_histories, vector_differences, vector_headers, write_json,
};
use dyncomp_torque::{
    ANALYTIC_AXIS, ANALYTIC_OMEGA0_B, ANALYTIC_TORQUE_B, AttitudeHistory, INERTIA_DIAG, OMEGA0_B,
    TORQUE_B, TorqueCase, run_bsm,
};
use grav_factory::Body;
use nalgebra::Vector3;
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareTorque";
/// Upstream `dt`, `tf`, `recordDt` \[s\].
const DT_S: f64 = 0.1;
const TF_S: f64 = 600.0;
const RECORD_DT_S: f64 = 1.0;

const USAGE: &str = "\
scenario_compare_torque [--timing]
  --timing   also time both engines and write scenarioCompareTorque_runtime.csv";

#[derive(Clone, Copy, Debug, Default)]
pub struct Config {
    /// Upstream `saveTiming`.
    pub save_timing: bool,
}

/// Upstream's JSON metrics \[rad\], \[rad/s\].
#[derive(Clone, Debug, Default)]
pub struct Metrics {
    pub n_samples: usize,
    pub bsm_analytic_rate_error_max: f64,
    pub bsm_analytic_attitude_error_max: f64,
    pub bsm_motion_attitude_max: f64,
    pub cross_engine_attitude_rest_max: Option<f64>,
    pub cross_engine_attitude_orbit_max: Option<f64>,
    pub body_rate_error_max: Option<f64>,
    pub mujoco_motion_attitude_max: Option<f64>,
    pub mujoco_analytic_rate_error_max: Option<f64>,
    pub mujoco_analytic_attitude_error_max: Option<f64>,
}

pub struct Results {
    pub metrics: Metrics,
    pub csv_path: std::path::PathBuf,
}

/// Upstream `principalAxisAnalyticSolution`: the exact body rate \[rad/s\]
/// and rotation angle \[rad\] of the principal-axis control.
pub fn principal_axis_analytic_solution(times_s: &[f64]) -> (Vec<Vector3<f64>>, Vec<f64>) {
    let alpha = ANALYTIC_TORQUE_B[ANALYTIC_AXIS] / INERTIA_DIAG[ANALYTIC_AXIS];
    let omega0 = ANALYTIC_OMEGA0_B[ANALYTIC_AXIS];
    let omega = times_s
        .iter()
        .map(|t| {
            let mut rate = Vector3::zeros();
            rate[ANALYTIC_AXIS] = omega0 + alpha * t;
            rate
        })
        .collect();
    let theta = times_s
        .iter()
        .map(|t| omega0 * t + 0.5 * alpha * t * t)
        .collect();
    (omega, theta)
}

/// Upstream `principalAxisAttitudeError` \[rad\].
pub fn principal_axis_attitude_error(sigma: &[Vector3<f64>], theta: &[f64]) -> Vec<f64> {
    dyncomp::require_equal_length("principal-axis attitude", &[sigma.len(), theta.len()])
        .unwrap_or_else(|error| panic!("{error}"));
    let mut axis = Vector3::zeros();
    axis[ANALYTIC_AXIS] = 1.0;
    sigma
        .iter()
        .zip(theta)
        .map(|(sigma, angle)| {
            let relative = mrp_to_dcm(*sigma) * prv_to_dcm(axis * *angle).transpose();
            4.0 * dcm_to_mrp(relative).norm().atan()
        })
        .collect()
}

fn check(result: Result<Vec<f64>, String>) -> Vec<f64> {
    result.unwrap_or_else(|error| panic!("{error}"))
}

fn validated(name: &str, history: &AttitudeHistory) -> Vec<f64> {
    let times = seconds(&history.times_nanos);
    check(validate_history(
        name,
        &times,
        Some(TF_S),
        RECORD_DT_S,
        false,
        &[
            Series::vectors("attitude", &history.sigma_bn),
            Series::vectors("rate", &history.omega_radps),
        ],
    ))
}

pub fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let orbiting = TorqueCase::orbiting(mu, TORQUE_B, OMEGA0_B);
    let at_rest = TorqueCase::at_rest(TORQUE_B, OMEGA0_B);
    let analytic = TorqueCase::at_rest(ANALYTIC_TORQUE_B, ANALYTIC_OMEGA0_B);

    if config.save_timing {
        let build_bsm = || run_bsm(&orbiting, DT_S, TF_S, RECORD_DT_S, false).1;
        #[cfg(feature = "mujoco")]
        let (bsm_s, mujoco_s) = {
            let build_mujoco = || {
                dyncomp_torque::run_mujoco(&orbiting, DT_S, TF_S, RECORD_DT_S, false, false).seconds
            };
            let (bsm, mujoco) = dyncomp::paired_propagation_times(
                Box::new(build_bsm),
                Box::new(build_mujoco),
                DEFAULT_TIMING_REPEATS,
            );
            (bsm, Some(mujoco))
        };
        #[cfg(not(feature = "mujoco"))]
        let (bsm_s, mujoco_s) = (
            dyncomp::median_propagation_time(build_bsm, DEFAULT_TIMING_REPEATS),
            None,
        );
        let path = save_runtime_table(
            FILE_NAME,
            &[(
                "Torqued rigid body (600 s, RK4 dt=0.1 s)".to_string(),
                bsm_s,
                mujoco_s,
            )],
            "Case",
        );
        println!("wrote {}", path.display());
    }

    let (bsm_orbit, _) = run_bsm(&orbiting, DT_S, TF_S, RECORD_DT_S, true);
    let (bsm_rest, _) = run_bsm(&at_rest, DT_S, TF_S, RECORD_DT_S, true);
    let time_s = validated("torque BSM orbit", &bsm_orbit);
    let rest_s = validated("torque BSM rest", &bsm_rest);
    check(validate_matching_histories(
        "torque BSM orbit/rest",
        &time_s,
        &rest_s,
        Some(TF_S),
        RECORD_DT_S,
        false,
    ));
    let bsm_motion =
        relative_principal_angles("torque attitude", &bsm_orbit.sigma_bn, &bsm_rest.sigma_bn);

    let (bsm_analytic, _) = run_bsm(&analytic, DT_S, TF_S, RECORD_DT_S, true);
    let analytic_s = validated("torque analytic BSM", &bsm_analytic);
    check(validate_matching_histories(
        "torque baseline/analytic BSM",
        &time_s,
        &analytic_s,
        Some(TF_S),
        RECORD_DT_S,
        false,
    ));
    let (omega_exact, theta_exact) = principal_axis_analytic_solution(&analytic_s);
    let bsm_rate_error = vector_differences(&bsm_analytic.omega_radps, &omega_exact);
    let bsm_att_error = principal_axis_attitude_error(&bsm_analytic.sigma_bn, &theta_exact);

    let mut headers = vec!["time_ns".to_string()];
    headers.extend(vector_headers("bsm_omega_BN_B_rad_per_s"));
    headers.extend([
        "bsm_motion_att_rad".to_string(),
        "bsm_analytic_rate_error_rad_per_s".to_string(),
        "bsm_analytic_att_error_rad".to_string(),
    ]);
    let mut columns: Vec<Vec<f64>> =
        vec![bsm_orbit.times_nanos.iter().map(|t| *t as f64).collect()];
    for axis in 0..3 {
        columns.push(bsm_orbit.omega_radps.iter().map(|w| w[axis]).collect());
    }
    columns.extend([
        bsm_motion.clone(),
        bsm_rate_error.clone(),
        bsm_att_error.clone(),
    ]);

    #[cfg_attr(not(feature = "mujoco"), allow(unused_mut))]
    let mut metrics = Metrics {
        n_samples: time_s.len(),
        bsm_analytic_rate_error_max: max_of(bsm_rate_error),
        bsm_analytic_attitude_error_max: max_of(bsm_att_error),
        bsm_motion_attitude_max: max_of(bsm_motion),
        ..Metrics::default()
    };

    #[cfg(feature = "mujoco")]
    {
        let mujoco_run = |case: &TorqueCase| {
            dyncomp_torque::run_mujoco(case, DT_S, TF_S, RECORD_DT_S, true, false).history
        };
        let mujoco_orbit = mujoco_run(&orbiting);
        let mujoco_rest = mujoco_run(&at_rest);
        let orbit_s = validated("torque MuJoCo orbit", &mujoco_orbit);
        let mujoco_rest_s = validated("torque MuJoCo rest", &mujoco_rest);
        check(validate_matching_histories(
            "torque BSM/MuJoCo orbit",
            &time_s,
            &orbit_s,
            Some(TF_S),
            RECORD_DT_S,
            false,
        ));
        check(validate_matching_histories(
            "torque MuJoCo orbit/rest",
            &orbit_s,
            &mujoco_rest_s,
            Some(TF_S),
            RECORD_DT_S,
            false,
        ));
        let mujoco_motion = relative_principal_angles(
            "torque attitude",
            &mujoco_orbit.sigma_bn,
            &mujoco_rest.sigma_bn,
        );
        let att_error = relative_principal_angles(
            "torque attitude",
            &bsm_orbit.sigma_bn,
            &mujoco_orbit.sigma_bn,
        );
        let att_error_rest =
            relative_principal_angles("torque attitude", &bsm_rest.sigma_bn, &mujoco_rest.sigma_bn);
        let rate_error = vector_differences(&bsm_orbit.omega_radps, &mujoco_orbit.omega_radps);

        let mujoco_analytic = mujoco_run(&analytic);
        let mujoco_analytic_s = validated("torque analytic MuJoCo", &mujoco_analytic);
        check(validate_matching_histories(
            "torque analytic BSM/MuJoCo",
            &analytic_s,
            &mujoco_analytic_s,
            Some(TF_S),
            RECORD_DT_S,
            false,
        ));
        let mujoco_rate_error = vector_differences(&mujoco_analytic.omega_radps, &omega_exact);
        let mujoco_att_error =
            principal_axis_attitude_error(&mujoco_analytic.sigma_bn, &theta_exact);

        metrics.cross_engine_attitude_rest_max = Some(max_of(att_error_rest.iter().copied()));
        metrics.cross_engine_attitude_orbit_max = Some(max_of(att_error.iter().copied()));
        metrics.body_rate_error_max = Some(max_of(rate_error));
        metrics.mujoco_motion_attitude_max = Some(max_of(mujoco_motion.iter().copied()));
        metrics.mujoco_analytic_rate_error_max = Some(max_of(mujoco_rate_error.iter().copied()));
        metrics.mujoco_analytic_attitude_error_max = Some(max_of(mujoco_att_error.iter().copied()));

        headers.extend(vector_headers("mujoco_omega_BN_B_rad_per_s"));
        headers.extend([
            "mujoco_motion_att_rad".to_string(),
            "att_error_orbit_rad".to_string(),
            "att_error_rest_rad".to_string(),
            "mujoco_analytic_rate_error_rad_per_s".to_string(),
            "mujoco_analytic_att_error_rad".to_string(),
        ]);
        for axis in 0..3 {
            columns.push(mujoco_orbit.omega_radps.iter().map(|w| w[axis]).collect());
        }
        columns.extend([
            mujoco_motion,
            att_error,
            att_error_rest,
            mujoco_rate_error,
            mujoco_att_error,
        ]);
    }

    let rows: Vec<Vec<f64>> = (0..time_s.len())
        .map(|index| columns.iter().map(|column| column[index]).collect())
        .collect();
    let csv_path = table_csv::write_table(FILE_NAME, &headers, &rows);
    write_summary(&metrics);
    Results { metrics, csv_path }
}

/// Upstream `writeJsonSummary`.
fn write_summary(metrics: &Metrics) {
    let mut summary = json!({
        "scenario": FILE_NAME,
        "nSamples": metrics.n_samples,
        "analyticAxis": (["x", "y", "z"][ANALYTIC_AXIS]),
        "analyticTorque": ANALYTIC_TORQUE_B[ANALYTIC_AXIS],
        "analyticInitialRate": ANALYTIC_OMEGA0_B[ANALYTIC_AXIS],
        "bsmAnalyticRateErrorMax": metrics.bsm_analytic_rate_error_max,
        "bsmAnalyticAttitudeErrorMax": metrics.bsm_analytic_attitude_error_max,
        "bsmMotionAttitudeMax": metrics.bsm_motion_attitude_max,
    });
    for (key, value) in [
        (
            "crossEngineAttitudeRestMax",
            metrics.cross_engine_attitude_rest_max,
        ),
        (
            "crossEngineAttitudeOrbitMax",
            metrics.cross_engine_attitude_orbit_max,
        ),
        ("bodyRateErrorMax", metrics.body_rate_error_max),
        (
            "mujocoMotionAttitudeMax",
            metrics.mujoco_motion_attitude_max,
        ),
        (
            "mujocoAnalyticRateErrorMax",
            metrics.mujoco_analytic_rate_error_max,
        ),
        (
            "mujocoAnalyticAttitudeErrorMax",
            metrics.mujoco_analytic_attitude_error_max,
        ),
    ] {
        if let Some(value) = value {
            summary[key] = json!(value);
        }
    }
    write_json(FILE_NAME, &summary);
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let results = run(Config {
        save_timing: args.flag("--timing"),
    });
    println!(
        "wrote {}: {:#?}",
        results.csv_path.display(),
        results.metrics
    );
}

#[cfg(test)]
mod tests {
    use super::{Config, run};

    /// `_assert_torque_metrics`.
    #[test]
    fn torque_metrics_match_upstream() {
        let metrics = run(Config::default()).metrics;
        assert!(metrics.bsm_analytic_rate_error_max < 5.0e-13, "{metrics:?}");
        assert!(
            metrics.bsm_analytic_attitude_error_max < 1.0e-8,
            "{metrics:?}"
        );
        assert!(metrics.bsm_motion_attitude_max < 1.0e-14, "{metrics:?}");
        #[cfg(feature = "mujoco")]
        {
            assert!(
                metrics.mujoco_analytic_rate_error_max.unwrap() < 5.0e-13,
                "{metrics:?}"
            );
            assert!(
                metrics.mujoco_analytic_attitude_error_max.unwrap() < 1.0e-7,
                "{metrics:?}"
            );
            assert!(
                metrics.cross_engine_attitude_rest_max.unwrap() < 1.0e-6,
                "{metrics:?}"
            );
            assert!(
                metrics.cross_engine_attitude_orbit_max.unwrap() < 1.0e-4,
                "{metrics:?}"
            );
            // Round-off depends on the platform: bounded, with no minimum
            // amplification over the at-rest error.
            assert!(
                metrics.mujoco_motion_attitude_max.unwrap() < 1.0e-4,
                "{metrics:?}"
            );
            assert!(metrics.body_rate_error_max.unwrap() < 1.0e-6, "{metrics:?}");
        }
    }
}
