//! Port of upstream
//! `examples/dynamicsComparison/scenarioCompareParetoRwPanels.py`: the
//! accuracy-versus-runtime (Pareto) study of the hub-wheels-panels system
//! of `scenario_compare_rw_panels` over 30 s. Fixed-step Euler, RK2 (Heun)
//! and RK4 sweep the step; adaptive RKF45 and RKF78 hold a 0.5 s task step
//! and sweep the tolerance (`relTol = absTol`). Each engine's final hub
//! attitude and position are compared with its own RKF78 reference at
//! `1e-13`, whose disagreement with the `1e-12` run bounds the reference
//! itself; every configuration is timed for both engines, interleaved.
//! Steps that do not divide the horizon are snapped to the nearest divisor
//! (upstream `snapStepToHorizon`).
//!
//! The adaptive integrators are basilisk-rs's `Rkf45`/`Rkf78`, the same
//! for both engines, whose step-size control (an RMS norm over the state
//! components) differs from upstream's `svIntegratorAdaptiveRungeKutta`
//! (a largest per-state relative error); the tolerances therefore map onto
//! somewhat different accuracies than upstream's.
//!
//! `--reduced` runs upstream's test and documentation profile (RK4 at
//! 0.1 s, Euler at 0.02 s, RKF45 at 0.5 s and `1e-7`, an RKF45 reference at
//! 0.05 s and `1e-9`, checked at 0.025 s), `--configs RK4:0.1,RKF45:0.5:1e-7`
//! any other sweep. Outputs, in `examples/output/scenarios/`:
//! `scenarioCompareParetoRwPanels.csv` (one row per configuration and
//! engine, for `plot_scenario_compare_pareto_rw_panels.py`'s `_pareto`,
//! `_frontier`, `_paretoPosition` and `_frontierPosition` figures) and
//! `scenarioCompareParetoRwPanels.json`.
//!
//! `tests` ports `_assert_pareto_metrics` on the reduced profile and
//! `test_pareto_rw_requires_common_comparison_horizon`.

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_pareto.rs"]
mod dyncomp_pareto;
#[path = "support/dyncomp_rw_panels.rs"]
mod dyncomp_rw_panels;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/table_csv.rs"]
mod table_csv;

use dyncomp::{IntegratorChoice, IntegratorName, sec2nano_u64, write_json};
use dyncomp_pareto::{
    Engine, FinalState, ParetoRow, SweepConfig, add_interleaved_timings,
    normalized_sweep_configurations, parse_configs, snapped_step, sweep_engine,
};
use dyncomp_rw_panels::{RW_MASS_KG, RwPanelsCase};
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareParetoRwPanels";
/// Upstream `SIM_DURATION` \[s\].
pub const SIM_DURATION_S: f64 = 30.0;
/// Upstream `ADAPTIVE_MACRO_DT` \[s\].
pub const ADAPTIVE_MACRO_DT: f64 = 0.5;

use IntegratorName::{Euler, Rk2, Rk4, Rkf45, Rkf78};

/// Upstream `REFERENCE`, `REFERENCE_CHECK`.
pub const REFERENCE: SweepConfig = SweepConfig::new(Rkf78, ADAPTIVE_MACRO_DT, Some(1.0e-13));
pub const REFERENCE_CHECK: SweepConfig = SweepConfig::new(Rkf78, ADAPTIVE_MACRO_DT, Some(1.0e-12));

/// Upstream `FULL_SWEEP_CONFIGS`.
pub fn full_sweep_configs() -> Vec<SweepConfig> {
    let mut configs = Vec::new();
    for dt in [0.1, 0.05, 0.02, 0.01, 0.005, 0.002] {
        configs.push(SweepConfig::new(Euler, dt, None));
    }
    for dt in [0.2, 0.1, 0.05, 0.02, 0.01, 0.005] {
        configs.push(SweepConfig::new(Rk2, dt, None));
    }
    for dt in [0.4, 0.2, 0.1, 0.05, 0.02, 0.01] {
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

/// Upstream's test and documentation profile.
#[derive(Clone, Debug)]
pub struct Config {
    pub sweep: Vec<SweepConfig>,
    pub reference: SweepConfig,
    pub reference_check: SweepConfig,
}

impl Config {
    pub fn full() -> Self {
        Self {
            sweep: full_sweep_configs(),
            reference: REFERENCE,
            reference_check: REFERENCE_CHECK,
        }
    }

    /// `SCENARIO_RUN_KWARGS["scenarioCompareParetoRwPanels"]`.
    pub fn reduced() -> Self {
        Self {
            sweep: vec![
                SweepConfig::new(Rk4, 0.1, None),
                SweepConfig::new(Euler, 0.02, None),
                SweepConfig::new(Rkf45, 0.5, Some(1.0e-7)),
            ],
            reference: SweepConfig::new(Rkf45, 0.05, Some(1.0e-9)),
            reference_check: SweepConfig::new(Rkf45, 0.025, Some(1.0e-9)),
        }
    }
}

/// One engine's sweep results.
#[derive(Clone, Debug, Default)]
pub struct EngineResults {
    pub rows: Vec<ParetoRow>,
    /// Upstream `referenceSelfConsistency` \[rad\], \[m\].
    pub attitude_floor: f64,
    pub position_floor: f64,
}

pub struct Results {
    pub bsm: EngineResults,
    pub mujoco: Option<EngineResults>,
    pub csv_path: std::path::PathBuf,
}

fn case(integrator: IntegratorChoice) -> RwPanelsCase {
    RwPanelsCase {
        with_panels: true,
        gravity_mu: None,
        rw_mass_kg: RW_MASS_KG,
        integrator,
    }
}

fn last(history: dyncomp_rw_panels::RwPanelsHistory) -> Option<(u64, FinalState)> {
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
    let period = record.then(|| sec2nano_u64(dt));
    let (history, seconds) = dyncomp_rw_panels::run_bsm(&case(integrator), dt, horizon, period);
    (last(history), seconds)
}

#[cfg(feature = "mujoco")]
fn mujoco_engine(
    integrator: IntegratorChoice,
    dt: f64,
    horizon: f64,
    record: bool,
) -> (Option<(u64, FinalState)>, f64) {
    let period = record.then(|| sec2nano_u64(dt));
    let (history, seconds) = dyncomp_rw_panels::run_mujoco(&case(integrator), dt, horizon, period);
    (last(history), seconds)
}

/// Upstream `normalizedConfiguration`.
fn normalized(config: &SweepConfig) -> (SweepConfig, serde_json::Value) {
    let executed = SweepConfig {
        dt: snapped_step(config.dt, SIM_DURATION_S),
        ..*config
    };
    let record = executed.record((executed.dt != config.dt).then_some(config.dt));
    (executed, record)
}

pub fn run(config: &Config) -> Results {
    let (configurations, records) = normalized_sweep_configurations(&config.sweep, SIM_DURATION_S)
        .unwrap_or_else(|error| panic!("{error}"));
    let (reference, reference_record) = normalized(&config.reference);
    let (reference_check, check_record) = normalized(&config.reference_check);
    let sweep = |engine: Engine| {
        let (rows, attitude_floor, position_floor) = sweep_engine(
            engine,
            &reference,
            &reference_check,
            &configurations,
            SIM_DURATION_S,
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
        add_interleaved_timings(&engines, &mut rows, SIM_DURATION_S);
    }

    let empty = EngineResults::default();
    let mujoco_results = mujoco.as_ref().unwrap_or(&empty);
    let rows_json = |rows: &[ParetoRow]| rows.iter().map(|r| r.to_json(None)).collect::<Vec<_>>();
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "configuration": {
                "simDuration": SIM_DURATION_S,
                "reference": reference_record,
                "referenceCheck": check_record,
                "sweepConfigs": records,
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
scenario_compare_pareto_rw_panels [--reduced] [--configs LIST]
  --reduced   upstream's test profile (three configurations, an RKF45 reference)
  --configs   sweep entries INTEGRATOR:DT[:TOL], comma separated, for example
              RK4:0.1,Euler:0.02,RKF45:0.5:1e-7 (integrators Euler, RK2, RK4,
              RKF45, RKF78)";

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
                "  {:18} dt={:<6} tol={:<8} error {:.3e} rad, {:.3e} m, wall {:.4} s",
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

    /// `_assert_pareto_metrics` for one engine on the reduced profile.
    pub(super) fn assert_pareto_metrics(results: &EngineResults) {
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
        let (max, min) = (dyncomp::max_of, |v: &[f64]| {
            v.iter().copied().fold(f64::INFINITY, f64::min)
        });
        assert!(max(errors.iter().copied()) > 2.0 * min(&errors));
        assert!(max(positions.iter().copied()) > 2.0 * min(&positions));
        let find = |name, dt| {
            results
                .rows
                .iter()
                .find(|row| row.config.integrator == name && row.config.dt == dt)
                .expect("configuration")
        };
        let (rk4, euler) = (find(Rk4, 0.1), find(Euler, 0.02));
        assert!(euler.error > rk4.error);
        assert!(euler.position_error > rk4.position_error);
    }

    #[test]
    fn pareto_metrics_match_upstream() {
        let results = run(&Config::reduced());
        assert_pareto_metrics(&results.bsm);
        #[cfg(feature = "mujoco")]
        assert_pareto_metrics(results.mujoco.as_ref().expect("MuJoCo rows"));
    }

    /// `test_pareto_rw_requires_common_comparison_horizon`.
    #[test]
    fn pareto_rw_requires_common_comparison_horizon() {
        let error = dyncomp_pareto::final_state(
            bsm_engine,
            &SweepConfig::new(Rk4, 0.8, None),
            SIM_DURATION_S,
        )
        .expect_err("0.8 s does not divide 30 s");
        assert!(error.contains("does not divide"), "{error}");
        assert!((snapped_step(0.8, SIM_DURATION_S) - 0.75).abs() < 1e-12);
        let (execution, records) =
            normalized_sweep_configurations(&[SweepConfig::new(Rk4, 0.8, None)], SIM_DURATION_S)
                .unwrap();
        assert_eq!(execution, vec![SweepConfig::new(Rk4, 0.75, None)]);
        assert_eq!(
            records,
            vec![
                json!({"integrator": "svIntegratorRK4", "dt": 0.75, "tol": null, "requestedDt": 0.8})
            ]
        );
        let error = normalized_sweep_configurations(
            &[
                SweepConfig::new(Rk4, 0.8, None),
                SweepConfig::new(Rk4, 0.76, None),
            ],
            SIM_DURATION_S,
        )
        .unwrap_err();
        assert!(error.contains("same executed"), "{error}");
        let stop = dyncomp::sec2nano(SIM_DURATION_S);
        for config in full_sweep_configs() {
            assert_eq!(stop % dyncomp::sec2nano(config.dt), 0, "{config:?}");
        }
    }
}
