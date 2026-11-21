//! Port of upstream
//! `examples/dynamicsComparison/scenarioCompareParetoFlexPanels.py`: the
//! Pareto (work-precision) study of `scenario_compare_pareto_rw_panels`
//! repeated on a stability-limited system, a hub with two lightly damped
//! (`c = 0.05 N m s/rad`) 16-segment `NHingedRigidBodyStateEffector` arrays
//! (38 degrees of freedom; `support/dyncomp_flex_panels.rs`) over 10 s.
//! Fixed-step RK2 and RK4 sweep steps around the explicit stability limit
//! (diverging runs are dropped); adaptive RKF45 and RKF78 hold a 0.1 s task
//! step and sweep the tolerance; each engine is compared with its own RKF78
//! reference at `1e-13` (checked at `1e-12`).
//!
//! As in `scenario_compare_pareto_rw_panels`, the adaptive integrators are
//! basilisk-rs's `Rkf45`/`Rkf78` for both engines, whose step-size control
//! differs from upstream's.
//!
//! `--reduced` runs upstream's test and documentation profile (two RKF45
//! tolerances, `1e-4` and `1e-6` at 0.1 s, over 1 s, against an RKF45
//! reference at 0.2 s and `1e-6`), `--configs` and `--duration` any other.
//! Outputs, in `examples/output/scenarios/`:
//! `scenarioCompareParetoFlexPanels.csv` (for
//! `plot_scenario_compare_pareto_flex_panels.py`) and
//! `scenarioCompareParetoFlexPanels.json`.
//!
//! `tests` ports `_assert_pareto_metrics` on the reduced profile and
//! `test_pareto_flex_requires_common_comparison_horizon`.

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_flex_panels.rs"]
mod dyncomp_flex_panels;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_pareto.rs"]
mod dyncomp_pareto;
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

use dyncomp::{IntegratorChoice, IntegratorName, write_json};
use dyncomp_flex_panels::{BsmChain, Chain, HubHistory};
use dyncomp_pareto::{
    Engine, FinalState, ParetoRow, SweepConfig, add_interleaved_timings, parse_configs,
    sweep_engine,
};
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareParetoFlexPanels";
/// Upstream `SIM_DURATION` \[s\].
pub const SIM_DURATION_S: f64 = 10.0;
/// Upstream `ADAPTIVE_MACRO_DT` \[s\].
pub const ADAPTIVE_MACRO_DT: f64 = 0.1;

use IntegratorName::{Rk2, Rk4, Rkf45, Rkf78};

/// Upstream `REFERENCE`, `REFERENCE_CHECK`.
pub const REFERENCE: SweepConfig = SweepConfig::new(Rkf78, ADAPTIVE_MACRO_DT, Some(1.0e-13));
pub const REFERENCE_CHECK: SweepConfig = SweepConfig::new(Rkf78, ADAPTIVE_MACRO_DT, Some(1.0e-12));

/// Upstream `FULL_SWEEP_CONFIGS`.
pub fn full_sweep_configs() -> Vec<SweepConfig> {
    let mut configs = Vec::new();
    for dt in [1.0e-3, 8.0e-4, 5.0e-4, 2.5e-4] {
        configs.push(SweepConfig::new(Rk2, dt, None));
    }
    for dt in [2.5e-3, 2.0e-3, 1.0e-3, 5.0e-4] {
        configs.push(SweepConfig::new(Rk4, dt, None));
    }
    for tol in [
        1e-4, 3e-5, 1e-5, 3e-6, 1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12,
    ] {
        configs.push(SweepConfig::new(Rkf45, ADAPTIVE_MACRO_DT, Some(tol)));
    }
    for tol in [1e-6, 1e-7, 1e-8, 1e-9, 1e-10, 1e-11, 1e-12] {
        configs.push(SweepConfig::new(Rkf78, ADAPTIVE_MACRO_DT, Some(tol)));
    }
    configs
}

#[derive(Clone, Debug)]
pub struct Config {
    pub sweep: Vec<SweepConfig>,
    /// Upstream `simDuration` \[s\].
    pub sim_duration_s: f64,
    pub reference: SweepConfig,
    pub reference_check: SweepConfig,
}

impl Config {
    pub fn full() -> Self {
        Self {
            sweep: full_sweep_configs(),
            sim_duration_s: SIM_DURATION_S,
            reference: REFERENCE,
            reference_check: REFERENCE_CHECK,
        }
    }

    /// `SCENARIO_RUN_KWARGS["scenarioCompareParetoFlexPanels"]` (the
    /// reference check keeps its default).
    pub fn reduced() -> Self {
        Self {
            sweep: vec![
                SweepConfig::new(Rkf45, 0.1, Some(1.0e-4)),
                SweepConfig::new(Rkf45, 0.1, Some(1.0e-6)),
            ],
            sim_duration_s: 1.0,
            reference: SweepConfig::new(Rkf45, 0.2, Some(1.0e-6)),
            reference_check: REFERENCE_CHECK,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct EngineResults {
    pub rows: Vec<ParetoRow>,
    pub attitude_floor: f64,
    pub position_floor: f64,
}

pub struct Results {
    pub bsm: EngineResults,
    pub mujoco: Option<EngineResults>,
    pub csv_path: std::path::PathBuf,
}

fn last(history: HubHistory) -> Option<(u64, FinalState)> {
    let index = history.times_nanos.len().checked_sub(1)?;
    Some((
        history.times_nanos[index],
        FinalState {
            sigma_bn: history.sigma_bn[index],
            r_bn_n: history.r_bn_n[index],
        },
    ))
}

/// Upstream `buildBSM`, recording every task step.
fn bsm_engine(
    integrator: IntegratorChoice,
    dt: f64,
    horizon: f64,
    record: bool,
) -> (Option<(u64, FinalState)>, f64) {
    let (history, seconds) = dyncomp_flex_panels::run_bsm(
        BsmChain::NHinged,
        &Chain::pareto(),
        integrator,
        dt,
        horizon,
        record,
    );
    (last(history), seconds)
}

#[cfg(feature = "mujoco")]
fn mujoco_engine(
    integrator: IntegratorChoice,
    dt: f64,
    horizon: f64,
    record: bool,
) -> (Option<(u64, FinalState)>, f64) {
    let (history, seconds) =
        dyncomp_flex_panels::run_mujoco(&Chain::pareto(), integrator, dt, horizon, record);
    (last(history), seconds)
}

pub fn run(config: &Config) -> Results {
    let duration = config.sim_duration_s;
    let sweep = |engine: Engine| {
        let (rows, attitude_floor, position_floor) = sweep_engine(
            engine,
            &config.reference,
            &config.reference_check,
            &config.sweep,
            duration,
        );
        EngineResults {
            rows,
            attitude_floor,
            position_floor,
        }
    };
    let mut bsm = sweep(bsm_engine);
    #[cfg(feature = "mujoco")]
    let mut mujoco = Some(sweep(mujoco_engine));
    #[cfg(not(feature = "mujoco"))]
    let mut mujoco: Option<EngineResults> = None;
    {
        #[cfg_attr(not(feature = "mujoco"), allow(unused_mut))]
        let mut engines: Vec<Engine> = vec![bsm_engine];
        let mut rows = vec![&mut bsm.rows];
        #[cfg(feature = "mujoco")]
        engines.push(mujoco_engine);
        if let Some(mujoco) = mujoco.as_mut() {
            rows.push(&mut mujoco.rows);
        }
        add_interleaved_timings(&engines, &mut rows, duration);
    }

    let empty = EngineResults::default();
    let mujoco_results = mujoco.as_ref().unwrap_or(&empty);
    let rows_json = |rows: &[ParetoRow]| rows.iter().map(|r| r.to_json(None)).collect::<Vec<_>>();
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "nSegments": Chain::pareto().n_segments,
            "configuration": {
                "simDuration": duration,
                "reference": config.reference.record(None),
                "referenceCheck": config.reference_check.record(None),
                "sweepConfigs": config.sweep.iter().map(|c| c.record(None)).collect::<Vec<_>>(),
            },
            "referenceSelfConsistency": {
                "bsm": {"attitude": bsm.attitude_floor, "position": bsm.position_floor},
                "mujoco": {
                    "attitude": mujoco_results.attitude_floor,
                    "position": mujoco_results.position_floor,
                },
            },
            "sharedReferenceThreshold": bsm.attitude_floor.max(mujoco_results.attitude_floor),
            "sharedPositionReferenceThreshold":
                bsm.position_floor.max(mujoco_results.position_floor),
            "bsm": rows_json(&bsm.rows),
            "mujoco": rows_json(&mujoco_results.rows),
        }),
    );
    let (headers, table) = dyncomp_pareto::table(&bsm.rows, &mujoco_results.rows);
    let csv_path = table_csv::write_table(FILE_NAME, &headers, &table);
    Results {
        bsm,
        mujoco,
        csv_path,
    }
}

const USAGE: &str = "\
scenario_compare_pareto_flex_panels [--reduced] [--configs LIST] [--duration S]
  --reduced    upstream's test profile (two RKF45 tolerances over 1 s)
  --configs    sweep entries INTEGRATOR:DT[:TOL], comma separated
  --duration   propagation horizon [s] (default 10)";

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let mut config = if args.flag("--reduced") {
        Config::reduced()
    } else {
        Config::full()
    };
    let configs: String = args.value("--configs", String::new());
    if !configs.is_empty() {
        config.sweep = parse_configs(&configs).unwrap_or_else(|error| args.fail(&error));
    }
    config.sim_duration_s = args.value("--duration", config.sim_duration_s);
    let results = run(&config);
    for (engine, results) in [
        ("BSM", Some(&results.bsm)),
        ("MuJoCo", results.mujoco.as_ref()),
    ] {
        let Some(results) = results else { continue };
        println!(
            "{engine}: reference self-consistency {:.2e} rad, {:.2e} m",
            results.attitude_floor, results.position_floor
        );
        for row in &results.rows {
            println!(
                "  {:18} dt={:<7} tol={:<8} error {:.3e} rad, {:.3e} m, wall {:.4} s",
                row.config.integrator.upstream(),
                row.config.dt,
                row.config
                    .tol
                    .map_or("-".to_string(), |tol| format!("{tol:e}")),
                row.error,
                row.position_error,
                row.wall
            );
        }
    }
    println!("wrote {}", results.csv_path.display());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `_assert_pareto_metrics` for one engine on the reduced profile, whose
    /// configurations all carry a tolerance.
    fn assert_pareto_metrics(results: &EngineResults) {
        for row in &results.rows {
            assert!(
                row.error.is_finite() && (0.0..0.1).contains(&row.error),
                "{row:?}"
            );
            assert!(
                row.position_error.is_finite() && (0.0..1.0).contains(&row.position_error),
                "{row:?}"
            );
            assert!(row.wall.is_finite() && row.wall > 0.0, "{row:?}");
        }
        assert!(
            (0.0..1.0e-8).contains(&results.attitude_floor),
            "{results:?}"
        );
        assert!(
            (0.0..1.0e-7).contains(&results.position_floor),
            "{results:?}"
        );
        let errors: Vec<f64> = results.rows.iter().map(|r| r.error).collect();
        let positions: Vec<f64> = results.rows.iter().map(|r| r.position_error).collect();
        let min = |v: &[f64]| v.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(dyncomp::max_of(errors.iter().copied()) > 2.0 * min(&errors));
        assert!(dyncomp::max_of(positions.iter().copied()) > 2.0 * min(&positions));
        let at = |tol: f64| {
            results
                .rows
                .iter()
                .find(|row| row.config.tol == Some(tol))
                .expect("tolerance row")
        };
        assert!(at(1.0e-6).error < at(1.0e-4).error);
        assert!(at(1.0e-6).position_error < at(1.0e-4).position_error);
    }

    #[test]
    fn pareto_metrics_match_upstream() {
        let results = run(&Config::reduced());
        assert_pareto_metrics(&results.bsm);
        #[cfg(feature = "mujoco")]
        assert_pareto_metrics(results.mujoco.as_ref().expect("MuJoCo rows"));
    }

    /// `test_pareto_flex_requires_common_comparison_horizon`.
    #[test]
    fn pareto_flex_requires_common_comparison_horizon() {
        let error =
            dyncomp_pareto::final_state(bsm_engine, &SweepConfig::new(Rk4, 3.0e-4, None), 1.0e-3)
                .expect_err("3e-4 s does not divide 1e-3 s");
        assert!(error.contains("does not divide"), "{error}");
        let stop = dyncomp::sec2nano(SIM_DURATION_S);
        for config in full_sweep_configs() {
            assert_eq!(stop % dyncomp::sec2nano(config.dt), 0, "{config:?}");
        }
    }
}
