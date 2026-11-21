//! Port of upstream
//! `examples/dynamicsComparison/scenarioCompareFlexPanels.py`: two flexible
//! solar arrays, each discretized into `N` rigid segments joined by
//! spring-damper hinges, on a free-floating hub, built three ways: the BSM
//! spacecraft with `NHingedRigidBodyStateEffector` chains, the BSM
//! spacecraft with `SpinningBodyNDofStateEffector` chains and, with the
//! `mujoco` feature, an `MjScene` of hinge-jointed bodies (see
//! `support/dyncomp_flex_panels.rs`).
//!
//! For each `N` (default `1, 2, 4, 8, 16, 32`, `--segments 1,2,4,8` as
//! upstream's test and documentation profile) the RK4 step resolves the
//! stiffest segment oscillation with 200 steps per period (at most 10 ms);
//! the hub attitudes of the implementations are compared over 8 s, and
//! each implementation's wall-clock for a fixed 600-step budget is timed
//! (median, minimum and standard deviation of five interleaved trials after
//! a 100-step warm-up).
//!
//! Outputs, in `examples/output/scenarios/`: `scenarioCompareFlexPanels.csv`
//! (one row per `N`, for `plot_scenario_compare_flex_panels.py`'s
//! `_validity` and `_runtime` figures) and `scenarioCompareFlexPanels.json`
//! (upstream's rows).
//!
//! `tests` ports `_assert_flex_panel_metrics` on the reduced sweep and the
//! flexible-panel half of
//! `test_articulated_scenarios_have_nontrivial_absolute_motion`.

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_flex_panels.rs"]
mod dyncomp_flex_panels;
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
    IntegratorChoice, Propagation, Series, TimingStats, interleaved_propagation_stats, max_of,
    relative_principal_angles, seconds, validate_history, validate_matching_histories, write_json,
};
use dyncomp_flex_panels::{BsmChain, Chain, run_bsm};
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareFlexPanels";
/// Upstream `STEPS_PER_PERIOD`: RK4 steps per stiffest segment oscillation.
const STEPS_PER_PERIOD: f64 = 200.0;
/// Upstream `TIMING_STEPS`, `TIMING_TRIALS`, `TIMING_WARMUP_STEPS`.
const TIMING_STEPS: f64 = 600.0;
const TIMING_TRIALS: usize = 5;
const TIMING_WARMUP_STEPS: f64 = 100.0;
/// Upstream `accuracyWindow` \[s\].
const ACCURACY_WINDOW_S: f64 = 8.0;
/// Upstream's default `nSegmentsList`.
pub const DEFAULT_SEGMENTS: [usize; 6] = [1, 2, 4, 8, 16, 32];

const USAGE: &str = "\
scenario_compare_flex_panels [--segments 1,2,4,8,16,32]
  --segments   per-chain segment counts to sweep (upstream nSegmentsList)";

#[derive(Clone, Debug)]
pub struct Config {
    /// Upstream `nSegmentsList`.
    pub n_segments: Vec<usize>,
}

/// One row of upstream's sweep.
#[derive(Clone, Debug, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Row {
    pub n_segments: usize,
    pub dof: usize,
    pub time_step: f64,
    #[serde(rename = "ndofVsNHingedMax")]
    pub ndof_vs_n_hinged_max: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub attitude_error_max: Option<f64>,
    #[serde(
        rename = "ndofAttitudeErrorMax",
        skip_serializing_if = "Option::is_none"
    )]
    pub ndof_attitude_error_max: Option<f64>,
    #[serde(skip)]
    pub bsm: TimingStats,
    #[serde(skip)]
    pub ndof: TimingStats,
    #[serde(skip)]
    pub mujoco: Option<TimingStats>,
}

pub struct Results {
    pub rows: Vec<Row>,
    pub csv_path: std::path::PathBuf,
}

/// Upstream `timeStep(nSegments)` \[s\].
pub fn time_step(chain: &Chain) -> f64 {
    let half_length = chain.segment_half_length();
    let effective_inertia =
        chain.segment_inertia()[(1, 1)] + chain.segment_mass() * half_length * half_length;
    let natural_period = 2.0 * std::f64::consts::PI * (effective_inertia / chain.k).sqrt();
    (natural_period / STEPS_PER_PERIOD).min(0.01)
}

fn validated(
    name: &str,
    times_nanos: &[u64],
    sigma: &[nalgebra::Vector3<f64>],
    dt: f64,
) -> Vec<f64> {
    validate_history(
        name,
        &seconds(times_nanos),
        Some(ACCURACY_WINDOW_S),
        dt,
        false,
        &[Series::vectors("attitude", sigma)],
    )
    .unwrap_or_else(|error| panic!("{error}"))
}

/// Upstream `runOne`: the accuracy comparison and the timing of one `N`.
fn run_one(n_segments: usize) -> Row {
    let chain = Chain::flex_panels(n_segments);
    let dt = time_step(&chain);
    let rk4 = IntegratorChoice::RK4;
    let (bsm, _) = run_bsm(BsmChain::NHinged, &chain, rk4, dt, ACCURACY_WINDOW_S, true);
    let time_bsm = validated(
        "flexible-panel nHinged",
        &bsm.times_nanos,
        &bsm.sigma_bn,
        dt,
    );
    let (ndof, _) = run_bsm(
        BsmChain::SpinningBodyNDof,
        &chain,
        rk4,
        dt,
        ACCURACY_WINDOW_S,
        true,
    );
    let time_ndof = validated("flexible-panel NDOF", &ndof.times_nanos, &ndof.sigma_bn, dt);
    validate_matching_histories(
        "flexible-panel nHinged/NDOF",
        &time_bsm,
        &time_ndof,
        Some(ACCURACY_WINDOW_S),
        dt,
        false,
    )
    .unwrap_or_else(|error| panic!("{error}"));
    let attitude = |a: &[nalgebra::Vector3<f64>], b: &[nalgebra::Vector3<f64>]| {
        max_of(relative_principal_angles("flexible-panel attitude", a, b))
    };
    let ndof_vs_n_hinged_max = attitude(&bsm.sigma_bn, &ndof.sigma_bn);

    let timing_stop = TIMING_STEPS * dt;
    let warmup_stop = TIMING_WARMUP_STEPS * dt;
    let bsm_timer = |stop: f64| -> Propagation<'static> {
        Box::new(move || run_bsm(BsmChain::NHinged, &chain, rk4, dt, stop, false).1)
    };
    let ndof_timer = |stop: f64| -> Propagation<'static> {
        Box::new(move || run_bsm(BsmChain::SpinningBodyNDof, &chain, rk4, dt, stop, false).1)
    };
    let mut builders = vec![bsm_timer(timing_stop), ndof_timer(timing_stop)];
    let mut warmups = vec![bsm_timer(warmup_stop), ndof_timer(warmup_stop)];
    #[cfg(not(feature = "mujoco"))]
    let (attitude_error_max, ndof_attitude_error_max) = (None, None);
    #[cfg(feature = "mujoco")]
    let (attitude_error_max, ndof_attitude_error_max) = {
        let (mujoco, _) = dyncomp_flex_panels::run_mujoco(&chain, rk4, dt, ACCURACY_WINDOW_S, true);
        let time_mujoco = validated(
            "flexible-panel MuJoCo",
            &mujoco.times_nanos,
            &mujoco.sigma_bn,
            dt,
        );
        validate_matching_histories(
            "flexible-panel BSM/MuJoCo",
            &time_bsm,
            &time_mujoco,
            Some(ACCURACY_WINDOW_S),
            dt,
            false,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        let mujoco_timer = |stop: f64| -> Propagation<'static> {
            Box::new(move || dyncomp_flex_panels::run_mujoco(&chain, rk4, dt, stop, false).1)
        };
        builders.push(mujoco_timer(timing_stop));
        warmups.push(mujoco_timer(warmup_stop));
        (
            Some(attitude(&bsm.sigma_bn, &mujoco.sigma_bn)),
            Some(attitude(&ndof.sigma_bn, &mujoco.sigma_bn)),
        )
    };

    // Upstream `measureWallClocks`.
    let stats = interleaved_propagation_stats(&mut builders, TIMING_TRIALS, Some(&mut warmups));
    Row {
        n_segments,
        dof: chain.dof(),
        time_step: dt,
        ndof_vs_n_hinged_max,
        attitude_error_max,
        ndof_attitude_error_max,
        bsm: stats[0],
        ndof: stats[1],
        mujoco: stats.get(2).copied(),
    }
}

pub fn run(config: &Config) -> Results {
    let rows: Vec<Row> = config.n_segments.iter().map(|n| run_one(*n)).collect();

    let mut summary_rows = Vec::new();
    for row in &rows {
        let mut entry = serde_json::to_value(row).expect("row");
        for (tag, stats) in [
            ("bsm", Some(row.bsm)),
            ("ndof", Some(row.ndof)),
            ("mujoco", row.mujoco),
        ] {
            if let Some(stats) = stats {
                entry[format!("{tag}Wall")] = json!(stats.median);
                entry[format!("{tag}WallMin")] = json!(stats.min);
                entry[format!("{tag}WallStd")] = json!(stats.std);
            }
        }
        summary_rows.push(entry);
    }
    write_json(
        FILE_NAME,
        &json!({"scenario": FILE_NAME, "rows": summary_rows}),
    );

    let with_mujoco = rows.iter().all(|row| row.mujoco.is_some());
    let mut headers: Vec<String> = [
        "n_segments",
        "dof",
        "time_step_s",
        "ndof_vs_nhinged_max_rad",
        "bsm_wall_s",
        "bsm_wall_min_s",
        "bsm_wall_std_s",
        "ndof_wall_s",
        "ndof_wall_min_s",
        "ndof_wall_std_s",
    ]
    .map(String::from)
    .to_vec();
    if with_mujoco {
        headers.extend(
            [
                "attitude_error_max_rad",
                "ndof_attitude_error_max_rad",
                "mujoco_wall_s",
                "mujoco_wall_min_s",
                "mujoco_wall_std_s",
            ]
            .map(String::from),
        );
    }
    let table: Vec<Vec<f64>> = rows
        .iter()
        .map(|row| {
            let mut values = vec![
                row.n_segments as f64,
                row.dof as f64,
                row.time_step,
                row.ndof_vs_n_hinged_max,
                row.bsm.median,
                row.bsm.min,
                row.bsm.std,
                row.ndof.median,
                row.ndof.min,
                row.ndof.std,
            ];
            if let (true, Some(mujoco)) = (with_mujoco, row.mujoco) {
                values.extend([
                    row.attitude_error_max.unwrap_or(f64::NAN),
                    row.ndof_attitude_error_max.unwrap_or(f64::NAN),
                    mujoco.median,
                    mujoco.min,
                    mujoco.std,
                ]);
            }
            values
        })
        .collect();
    let csv_path = table_csv::write_table(FILE_NAME, &headers, &table);
    Results { rows, csv_path }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let segments: String = args.value("--segments", String::new());
    let n_segments = if segments.is_empty() {
        DEFAULT_SEGMENTS.to_vec()
    } else {
        segments
            .split(',')
            .map(|n| {
                n.trim()
                    .parse()
                    .unwrap_or_else(|_| args.fail(&format!("invalid segment count {n:?}")))
            })
            .collect()
    };
    let results = run(&Config { n_segments });
    for row in &results.rows {
        println!(
            "N={:2} ({} DOF, dt={:.3e} s): NDOF vs nHinged {:.2e} rad, vs MuJoCo {:?} rad; wall \
             BSM {:.4} s, NDOF {:.4} s, MuJoCo {:?} s",
            row.n_segments,
            row.dof,
            row.time_step,
            row.ndof_vs_n_hinged_max,
            row.attitude_error_max,
            row.bsm.median,
            row.ndof.median,
            row.mujoco.map(|stats| stats.median)
        );
    }
    println!("wrote {}", results.csv_path.display());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `_assert_flex_panel_metrics` on upstream's reduced `nSegmentsList =
    /// (1, 2, 4, 8)`: the implementations agree within 1e-12 rad and every
    /// timing is finite and positive.
    #[test]
    fn flex_panel_metrics_match_upstream() {
        let results = run(&Config {
            n_segments: vec![1, 2, 4, 8],
        });
        for row in &results.rows {
            assert!(row.ndof_vs_n_hinged_max < 1.0e-12, "{row:?}");
            for stats in [Some(row.bsm), Some(row.ndof), row.mujoco]
                .into_iter()
                .flatten()
            {
                assert!(stats.median.is_finite() && stats.median > 0.0, "{row:?}");
            }
            #[cfg(feature = "mujoco")]
            {
                assert!(row.attitude_error_max.unwrap() < 1.0e-12, "{row:?}");
                assert!(row.ndof_attitude_error_max.unwrap() < 1.0e-12, "{row:?}");
            }
        }
    }

    /// `test_articulated_scenarios_have_nontrivial_absolute_motion`
    /// (flexible-panel part): with one segment per array the BSM hub turns
    /// 0.215009 rad (1 %) from its initial attitude within 8 s.
    #[test]
    fn single_segment_hub_excursion_matches_upstream() {
        let chain = Chain::flex_panels(1);
        let (history, _) = run_bsm(
            BsmChain::NHinged,
            &chain,
            IntegratorChoice::RK4,
            time_step(&chain),
            8.0,
            true,
        );
        let initial = vec![history.sigma_bn[0]; history.sigma_bn.len()];
        let excursion = max_of(relative_principal_angles(
            "excursion",
            &history.sigma_bn,
            &initial,
        ));
        assert!(
            (excursion - 0.215009).abs() <= 0.01 * 0.215009,
            "excursion {excursion} rad"
        );
    }
}
