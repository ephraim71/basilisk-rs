//! Port of upstream `examples/dynamicsComparison/sweepOrbitDt.py`: the
//! two-body orbit of `scenario_compare_orbit` repeated over a ladder of RK4
//! steps (default 40, 20, 10, 5, 2.5 s) to one common, exactly aligned
//! final epoch, separating the per-engine error against Kepler (RK4
//! truncation, `∝ dt⁴`) from the cross-engine difference (independent of
//! `dt`). Needs the `mujoco` feature.
//!
//! Outputs, in `examples/output/scenarios/`: `sweepOrbitDt.json` (upstream's
//! rows and fitted pairwise truncation orders) and `sweepOrbitDt.csv`
//! (one row per step, for `plot_dyncomp_paper_figures.py`'s `sweepOrbitDt`
//! figure).
//!
//! `tests` ports upstream's `orbit-step` reduced sweep check (orders within
//! 3 % of 4) and `test_orbit_sweep_rejects_incomplete_engine_history`.

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
    Series, aligned_horizon, max_of, recorder_sample_interval, seconds, validate_history,
    validate_matching_histories, vector_differences, write_json,
};
use dyncomp_orbit::{MASS_KG, initial_orbit_state, kepler_truth, run_bsm, run_mujoco};
use grav_factory::Body;
use nalgebra::Vector3;
use orbit_tools::orbit_period_s;
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "sweepOrbitDt";

#[derive(Clone, Debug)]
pub struct Config {
    /// Upstream `dts` \[s\].
    pub dts: Vec<f64>,
    /// Upstream `simDuration` \[s\]; two orbits when `None`.
    pub sim_duration_s: Option<f64>,
    /// Upstream `recordDt` \[s\].
    pub record_dt_s: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            dts: vec![40.0, 20.0, 10.0, 5.0, 2.5],
            sim_duration_s: None,
            record_dt_s: 60.0,
        }
    }
}

/// One rung of the ladder \[s\], \[m\].
#[derive(Clone, Copy, Debug, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Row {
    pub dt: f64,
    pub bsm_vs_analytic_max: f64,
    pub mujoco_vs_analytic_max: f64,
    pub cross_paradigm_pos_max: f64,
}

/// Upstream `comparisonRow`: validates one rung's histories and returns its
/// maximum analytic and cross-engine position errors.
#[allow(clippy::too_many_arguments)]
pub fn comparison_row(
    dt: f64,
    tf: f64,
    record_dt: f64,
    bsm_times: &[f64],
    mujoco_times: &[f64],
    position_bsm: &[Vector3<f64>],
    position_mujoco: &[Vector3<f64>],
    truth: &[Vector3<f64>],
) -> Result<Row, String> {
    let interval = recorder_sample_interval(dt, record_dt)?;
    validate_matching_histories(
        "orbit-step BSM/MuJoCo",
        bsm_times,
        mujoco_times,
        Some(tf),
        interval,
        false,
    )?;
    validate_history(
        "orbit-step BSM",
        bsm_times,
        Some(tf),
        interval,
        false,
        &[
            Series::vectors("position", position_bsm),
            Series::vectors("analyticPosition", truth),
        ],
    )?;
    validate_history(
        "orbit-step MuJoCo",
        mujoco_times,
        Some(tf),
        interval,
        false,
        &[Series::vectors("position", position_mujoco)],
    )?;
    Ok(Row {
        dt,
        bsm_vs_analytic_max: max_of(vector_differences(truth, position_bsm)),
        mujoco_vs_analytic_max: max_of(vector_differences(truth, position_mujoco)),
        cross_paradigm_pos_max: max_of(vector_differences(position_bsm, position_mujoco)),
    })
}

pub struct Results {
    pub rows: Vec<Row>,
    pub pairwise_truncation_orders: Vec<f64>,
}

pub fn run(config: &Config) -> Results {
    assert!(
        !config.dts.is_empty(),
        "At least one integration step is required."
    );
    let mu = Body::Earth.mu_m3ps2();
    let (_, _, elements) = initial_orbit_state(mu);
    let period = orbit_period_s(mu, elements.semi_major_axis);
    let requested_tf = config.sim_duration_s.unwrap_or(2.0 * period);
    let mut intervals = config.dts.clone();
    for dt in &config.dts {
        intervals.push(recorder_sample_interval(*dt, config.record_dt_s).expect("intervals"));
    }
    let tf = aligned_horizon(requested_tf, &intervals).unwrap_or_else(|error| panic!("{error}"));
    let mut rows = Vec::new();
    for dt in &config.dts {
        let (bsm, _) = run_bsm(MASS_KG, mu, *dt, tf, config.record_dt_s, true);
        let (mujoco, _) = run_mujoco(mu, *dt, tf, config.record_dt_s, true);
        let bsm_times = seconds(&bsm.times_nanos);
        let truth = kepler_truth(mu, &elements, &bsm_times);
        let row = comparison_row(
            *dt,
            tf,
            config.record_dt_s,
            &bsm_times,
            &seconds(&mujoco.times_nanos),
            &bsm.positions_m,
            &mujoco.positions_m,
            &truth,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        println!(
            "dt {:5.1} s: BSM vs Kepler {:.3e} m, cross-engine {:.3e} m",
            dt, row.bsm_vs_analytic_max, row.cross_paradigm_pos_max
        );
        rows.push(row);
    }
    let orders: Vec<f64> = rows
        .windows(2)
        .map(|pair| {
            let (hi, lo) = (pair[0], pair[1]);
            (hi.bsm_vs_analytic_max / lo.bsm_vs_analytic_max).ln() / (hi.dt / lo.dt).ln()
        })
        .collect();
    println!("pairwise fitted truncation orders: {orders:.2?}");
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "configuration": {
                "requestedTf": requested_tf,
                "tf": tf,
                "recordDt": config.record_dt_s,
                "orbitPeriod": period,
                "orbitCount": tf / period,
                "orbit": "same elements as scenarioCompareOrbit",
            },
            "rows": rows,
            "pairwiseTruncationOrders": orders,
        }),
    );
    let headers = [
        "dt_s",
        "bsm_vs_analytic_max_m",
        "mujoco_vs_analytic_max_m",
        "cross_paradigm_pos_max_m",
    ]
    .map(String::from)
    .to_vec();
    let table: Vec<Vec<f64>> = rows
        .iter()
        .map(|row| {
            vec![
                row.dt,
                row.bsm_vs_analytic_max,
                row.mujoco_vs_analytic_max,
                row.cross_paradigm_pos_max,
            ]
        })
        .collect();
    let path = table_csv::write_table(FILE_NAME, &headers, &table);
    println!("wrote {}", path.display());
    Results {
        rows,
        pairwise_truncation_orders: orders,
    }
}

const USAGE: &str = "\
sweep_orbit_dt [--dts 40,20,10,5,2.5] [--duration S] [--record-dt S]";

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let mut config = Config::default();
    let dts: String = args.value("--dts", String::new());
    if !dts.is_empty() {
        config.dts = dts
            .split(',')
            .map(|dt| {
                dt.trim()
                    .parse()
                    .unwrap_or_else(|_| args.fail(&format!("invalid step {dt:?}")))
            })
            .collect();
    }
    let duration: f64 = args.value("--duration", 0.0);
    if duration > 0.0 {
        config.sim_duration_s = Some(duration);
    }
    config.record_dt_s = args.value("--record-dt", config.record_dt_s);
    run(&config);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `test_research_sweeps_capture_expected_physical_trends[orbit-step]`.
    #[test]
    fn truncation_order_is_four() {
        let results = run(&Config {
            dts: vec![20.0, 10.0, 5.0],
            sim_duration_s: Some(600.0),
            record_dt_s: 20.0,
        });
        for order in &results.pairwise_truncation_orders {
            assert!((order - 4.0).abs() <= 0.03 * 4.0, "order {order}");
        }
    }

    /// `test_orbit_sweep_rejects_incomplete_engine_history`.
    #[test]
    fn rejects_incomplete_engine_history() {
        let bsm_times = [0.0, 1.0, 2.0];
        let mujoco_times = [0.0, 2.0];
        let zeros = |n| vec![Vector3::zeros(); n];
        let error = comparison_row(
            0.1,
            2.0,
            1.0,
            &bsm_times,
            &mujoco_times,
            &zeros(3),
            &zeros(2),
            &zeros(3),
        )
        .unwrap_err();
        assert!(error.contains("samples"), "{error}");
        let error = comparison_row(
            0.1,
            2.0,
            1.0,
            &bsm_times[..2],
            &bsm_times[..2],
            &zeros(2),
            &zeros(2),
            &zeros(2),
        )
        .unwrap_err();
        assert!(error.contains("ended at"), "{error}");
    }
}
