//! Port of upstream `examples/dynamicsComparison/scenarioCompareOrbit.py`,
//! the first dynamics-engine comparison: a point-mass spacecraft on a
//! Keplerian orbit about a point-mass Earth, propagated by the
//! Backsubstitution [`Spacecraft`](basilisk_rs::spacecraft::Spacecraft)
//! ("BSM") and, with the `mujoco` feature, by an
//! [`MjScene`](basilisk_rs::mujoco_dynamics::MjScene) body on three slide
//! joints under `NBodyGravity` (see `support/dyncomp_orbit.rs`).
//!
//! Both engines integrate `r'' = -mu r / |r|^3` with RK4 at 10 s over the
//! two-orbit horizon aligned to the 10 s step and 60 s recorder
//! (upstream `_comparisonValidation.alignedHorizon`), and are compared with
//! the analytic Kepler solution and with each other. Without the `mujoco`
//! feature only the BSM side runs, as upstream without a MuJoCo build.
//!
//! Outputs, in `examples/output/scenarios/`:
//!
//! * `scenarioCompareOrbit.csv`: the analytic, BSM and MuJoCo positions and
//!   the position errors at every sample (`plot_scenario_compare_orbit.py`
//!   draws upstream's `_trajectory`, `_accuracy` and `_crossError` figures);
//! * `scenarioCompareOrbit.json`: upstream's metric summary;
//! * with `--timing`, `scenarioCompareOrbit_runtime.csv`: upstream's
//!   `saveTiming` table (median of five interleaved propagations after a
//!   warm-up, setup excluded).
//!
//! `tests` ports the orbit checks of upstream's
//! `test_scenarioDynamicsComparison.py` (`_assert_orbit_metrics`).

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
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

use dyncomp::{
    DEFAULT_TIMING_REPEATS, Series, aligned_horizon, max_of, save_runtime_table, seconds,
    validate_history, vector_differences, vector_headers, write_json,
};
use dyncomp_orbit::{MASS_KG, initial_orbit_state, kepler_truth, run_bsm};
use grav_factory::Body;
use nalgebra::Vector3;
use orbit_tools::orbit_period_s;
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareOrbit";
/// Upstream `dt` \[s\].
const DT_S: f64 = 10.0;
/// Upstream `recordDt` \[s\].
const RECORD_DT_S: f64 = 60.0;

const USAGE: &str = "\
scenario_compare_orbit [--timing]
  --timing   also time both engines and write scenarioCompareOrbit_runtime.csv";

#[derive(Clone, Copy, Debug, Default)]
pub struct Config {
    /// Upstream `saveTiming`.
    pub save_timing: bool,
}

/// Upstream's JSON metrics; the MuJoCo entries are `None` without the
/// `mujoco` feature.
#[derive(Clone, Debug)]
pub struct Metrics {
    pub n_samples: usize,
    pub requested_final_time_s: f64,
    pub final_time_s: f64,
    pub orbit_period_s: f64,
    pub bsm_vs_analytic_max_m: f64,
    pub bsm_energy_drift: f64,
    pub mujoco_vs_analytic_max_m: Option<f64>,
    pub cross_paradigm_pos_max_m: Option<f64>,
    pub mujoco_energy_drift: Option<f64>,
}

pub struct Results {
    pub metrics: Metrics,
    pub csv_path: std::path::PathBuf,
}

/// `max |E - E0| / |E0|` of the specific orbital energy.
fn energy_drift(mu: f64, positions: &[Vector3<f64>], velocities: &[Vector3<f64>]) -> f64 {
    let energy: Vec<f64> = positions
        .iter()
        .zip(velocities)
        .map(|(r, v)| 0.5 * v.norm_squared() - mu / r.norm())
        .collect();
    max_of(energy.iter().map(|e| (e - energy[0]).abs())) / energy[0].abs()
}

pub fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let (_, _, elements) = initial_orbit_state(mu);
    let period = orbit_period_s(mu, elements.semi_major_axis);
    let requested_tf = 2.0 * period;
    let tf = aligned_horizon(requested_tf, &[DT_S, RECORD_DT_S]).expect("aligned horizon");

    if config.save_timing {
        let build_bsm = || run_bsm(MASS_KG, mu, DT_S, tf, RECORD_DT_S, false).1;
        #[cfg(feature = "mujoco")]
        let (bsm_s, mujoco_s) = {
            let build_mujoco = || dyncomp_orbit::run_mujoco(mu, DT_S, tf, RECORD_DT_S, false).1;
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
                "Two-body orbit (about 2 orbits, dt=10 s)".to_string(),
                bsm_s,
                mujoco_s,
            )],
            "Case",
        );
        println!("wrote {}", path.display());
    }

    let (bsm, _) = run_bsm(MASS_KG, mu, DT_S, tf, RECORD_DT_S, true);
    let time_s = seconds(&bsm.times_nanos);
    validate_history(
        "orbit BSM",
        &time_s,
        Some(tf),
        RECORD_DT_S,
        false,
        &[
            Series::vectors("position", &bsm.positions_m),
            Series::vectors("velocity", &bsm.velocities_mps),
        ],
    )
    .unwrap_or_else(|error| panic!("{error}"));
    let truth = kepler_truth(mu, &elements, &time_s);
    let err_bsm = vector_differences(&truth, &bsm.positions_m);

    let mut headers = vec!["time_ns".to_string()];
    headers.extend(vector_headers("truth_r_N_m"));
    headers.extend(vector_headers("bsm_r_N_m"));
    headers.push("bsm_vs_kepler_m".to_string());
    let mut columns: Vec<Vec<f64>> = vec![bsm.times_nanos.iter().map(|t| *t as f64).collect()];
    for axis in 0..3 {
        columns.push(truth.iter().map(|r| r[axis]).collect());
    }
    for axis in 0..3 {
        columns.push(bsm.positions_m.iter().map(|r| r[axis]).collect());
    }
    columns.push(err_bsm.clone());

    #[cfg_attr(not(feature = "mujoco"), allow(unused_mut))]
    let mut metrics = Metrics {
        n_samples: time_s.len(),
        requested_final_time_s: requested_tf,
        final_time_s: tf,
        orbit_period_s: period,
        bsm_vs_analytic_max_m: max_of(err_bsm.iter().copied()),
        bsm_energy_drift: energy_drift(mu, &bsm.positions_m, &bsm.velocities_mps),
        mujoco_vs_analytic_max_m: None,
        cross_paradigm_pos_max_m: None,
        mujoco_energy_drift: None,
    };

    #[cfg(feature = "mujoco")]
    {
        let (mujoco, _) = dyncomp_orbit::run_mujoco(mu, DT_S, tf, RECORD_DT_S, true);
        let mujoco_times = seconds(&mujoco.times_nanos);
        dyncomp::validate_matching_histories(
            "orbit BSM/MuJoCo",
            &time_s,
            &mujoco_times,
            Some(tf),
            RECORD_DT_S,
            false,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        validate_history(
            "orbit MuJoCo",
            &mujoco_times,
            Some(tf),
            RECORD_DT_S,
            false,
            &[
                Series::vectors("position", &mujoco.positions_m),
                Series::vectors("velocity", &mujoco.velocities_mps),
            ],
        )
        .unwrap_or_else(|error| panic!("{error}"));
        let err_mujoco = vector_differences(&truth, &mujoco.positions_m);
        let cross = vector_differences(&bsm.positions_m, &mujoco.positions_m);
        metrics.mujoco_vs_analytic_max_m = Some(max_of(err_mujoco.iter().copied()));
        metrics.cross_paradigm_pos_max_m = Some(max_of(cross.iter().copied()));
        metrics.mujoco_energy_drift = Some(energy_drift(
            mu,
            &mujoco.positions_m,
            &mujoco.velocities_mps,
        ));
        headers.extend(vector_headers("mujoco_r_N_m"));
        headers.push("mujoco_vs_kepler_m".to_string());
        headers.push("cross_error_m".to_string());
        for axis in 0..3 {
            columns.push(mujoco.positions_m.iter().map(|r| r[axis]).collect());
        }
        columns.push(err_mujoco);
        columns.push(cross);
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
        "requestedFinalTime": metrics.requested_final_time_s,
        "finalTime": metrics.final_time_s,
        "orbitPeriod": metrics.orbit_period_s,
        "orbitCount": metrics.final_time_s / metrics.orbit_period_s,
        "bsmVsAnalyticMax": metrics.bsm_vs_analytic_max_m,
        "bsmEnergyDrift": metrics.bsm_energy_drift,
    });
    if let (Some(mujoco), Some(cross), Some(drift)) = (
        metrics.mujoco_vs_analytic_max_m,
        metrics.cross_paradigm_pos_max_m,
        metrics.mujoco_energy_drift,
    ) {
        summary["mujocoVsAnalyticMax"] = json!(mujoco);
        summary["crossParadigmPosMax"] = json!(cross);
        summary["mujocoEnergyDrift"] = json!(drift);
    }
    write_json(FILE_NAME, &summary);
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let results = run(Config {
        save_timing: args.flag("--timing"),
    });
    let metrics = &results.metrics;
    println!(
        "wrote {} ({} samples): BSM vs Kepler {:.4e} m, energy drift {:.2e}",
        results.csv_path.display(),
        metrics.n_samples,
        metrics.bsm_vs_analytic_max_m,
        metrics.bsm_energy_drift
    );
    if let (Some(mujoco), Some(cross)) = (
        metrics.mujoco_vs_analytic_max_m,
        metrics.cross_paradigm_pos_max_m,
    ) {
        println!("  MuJoCo vs Kepler {mujoco:.4e} m, BSM vs MuJoCo {cross:.3e} m");
    }
}

#[cfg(test)]
mod tests {
    use super::{Config, run};

    fn assert_relative(actual: f64, expected: f64, relative: f64) {
        assert!(
            (actual - expected).abs() <= relative * expected.abs(),
            "{actual} not within {relative} of {expected}"
        );
    }

    /// `_assert_orbit_metrics`: each engine matches Kepler to the RK4
    /// truncation level, 0.0304681 m (2 %), conserves the orbital energy
    /// to 6e-11, and the engines agree to 2e-7 m.
    #[test]
    fn orbit_metrics_match_upstream() {
        let metrics = run(Config::default()).metrics;
        assert_relative(metrics.bsm_vs_analytic_max_m, 0.0304681, 0.02);
        assert!(
            metrics.bsm_energy_drift < 6.0e-11,
            "{}",
            metrics.bsm_energy_drift
        );
        #[cfg(feature = "mujoco")]
        {
            assert_relative(metrics.mujoco_vs_analytic_max_m.unwrap(), 0.0304680, 0.02);
            let cross = metrics.cross_paradigm_pos_max_m.unwrap();
            assert!(cross < 2.0e-7, "cross-engine {cross} m");
            let drift = metrics.mujoco_energy_drift.unwrap();
            assert!(drift < 6.0e-11, "MuJoCo energy drift {drift}");
        }
    }
}
