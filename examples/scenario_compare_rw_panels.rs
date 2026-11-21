//! Port of upstream `examples/dynamicsComparison/scenarioCompareRwPanels.py`:
//! a free-floating 600 kg hub with three orthogonal reaction wheels under
//! constant motor torques and two spring-damper hinged solar panels released
//! from 12°, propagated 120 s with RK4 at 0.02 s by the Backsubstitution
//! spacecraft and, with the `mujoco` feature, by an `MjScene` (see
//! `support/dyncomp_rw_panels.rs` for how each engine models the wheels and
//! panels). Gravity is off, so the comparison isolates the internal
//! multibody coupling; the hub attitudes are compared by the principal angle
//! of the relative DCM.
//!
//! Outputs, in `examples/output/scenarios/`:
//! `scenarioCompareRwPanels.csv` (hub rates, hinge angles and the attitude
//! difference every 0.5 s, for `plot_scenario_compare_rw_panels.py`),
//! `scenarioCompareRwPanels.json` and, with `--timing`,
//! `scenarioCompareRwPanels_runtime.csv`.
//!
//! `tests` ports upstream's `_assert_rw_panel_metrics` (MuJoCo) and the
//! reaction-wheel half of `test_articulated_scenarios_have_nontrivial_absolute_motion`,
//! which bounds the BSM response itself.

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
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

use dyncomp::{
    DEFAULT_TIMING_REPEATS, Series, recorder_period_nanos, save_runtime_table, seconds,
    vector_headers, write_json,
};
use dyncomp_rw_panels::{RwPanelsCase, RwPanelsHistory, run_bsm};
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareRwPanels";
/// Upstream `dt`, `tf`, `recordDt` \[s\].
pub const DT_S: f64 = 0.02;
pub const TF_S: f64 = 120.0;
pub const RECORD_DT_S: f64 = 0.5;

const USAGE: &str = "\
scenario_compare_rw_panels [--timing]
  --timing   also time both engines and write scenarioCompareRwPanels_runtime.csv";

#[derive(Clone, Copy, Debug, Default)]
pub struct Config {
    /// Upstream `saveTiming`.
    pub save_timing: bool,
}

/// Upstream's JSON metrics (MuJoCo only) \[rad\], \[rad/s\].
#[derive(Clone, Copy, Debug, Default)]
pub struct Metrics {
    pub n_samples: usize,
    pub hub_attitude_principal_angle_max: Option<f64>,
    pub hub_body_rate_error_max: Option<f64>,
    pub wheel_speed_error_max: Option<f64>,
    pub panel_angle_error_max: Option<f64>,
}

pub struct Results {
    pub metrics: Metrics,
    pub bsm: RwPanelsHistory,
    pub csv_path: std::path::PathBuf,
}

/// Upstream's `validateHistory` of one engine's histories.
fn validated(name: &str, history: &RwPanelsHistory) -> Vec<f64> {
    let times = seconds(&history.times_nanos);
    let wheels: Vec<Vec<f64>> = history
        .wheel_speeds_radps
        .iter()
        .map(|w| w.to_vec())
        .collect();
    let panels: Vec<Vec<f64>> = history
        .panel_angles_rad
        .iter()
        .map(|p| p.to_vec())
        .collect();
    dyncomp::validate_history(
        name,
        &times,
        Some(TF_S),
        RECORD_DT_S,
        false,
        &[
            Series::vectors("attitude", &history.sigma_bn),
            Series::vectors("rate", &history.omega_radps),
            Series::rows("wheels", &wheels),
            Series::rows("panels", &panels),
        ],
    )
    .unwrap_or_else(|error| panic!("{error}"))
}

pub fn run(config: Config) -> Results {
    let case = RwPanelsCase::FREE;
    let period = recorder_period_nanos(DT_S, RECORD_DT_S);
    if config.save_timing {
        let build_bsm = || run_bsm(&case, DT_S, TF_S, None).1;
        #[cfg(feature = "mujoco")]
        let (bsm_s, mujoco_s) = {
            let build_mujoco = || dyncomp_rw_panels::run_mujoco(&case, DT_S, TF_S, None).1;
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
                "Hub + 3 RWs + 2 flex panels, 11 DOF (120 s, dt=0.02 s)".to_string(),
                bsm_s,
                mujoco_s,
            )],
            "Case",
        );
        println!("wrote {}", path.display());
    }

    let (bsm, _) = run_bsm(&case, DT_S, TF_S, Some(period));
    let time_s = validated("reaction-wheel panel BSM", &bsm);

    let mut headers = vec!["time_ns".to_string()];
    headers.extend(vector_headers("bsm_omega_BN_B_rad_per_s"));
    headers.extend([
        "bsm_panelP_theta_rad".to_string(),
        "bsm_panelM_theta_rad".to_string(),
    ]);
    let mut columns: Vec<Vec<f64>> = vec![bsm.times_nanos.iter().map(|t| *t as f64).collect()];
    for axis in 0..3 {
        columns.push(bsm.omega_radps.iter().map(|w| w[axis]).collect());
    }
    for panel in 0..2 {
        columns.push(bsm.panel_angles_rad.iter().map(|p| p[panel]).collect());
    }
    #[cfg_attr(not(feature = "mujoco"), allow(unused_mut))]
    let mut metrics = Metrics {
        n_samples: time_s.len(),
        ..Metrics::default()
    };

    #[cfg(feature = "mujoco")]
    {
        let (mujoco, _) = dyncomp_rw_panels::run_mujoco(&case, DT_S, TF_S, Some(period));
        let mujoco_s = validated("reaction-wheel panel MuJoCo", &mujoco);
        dyncomp::validate_matching_histories(
            "reaction-wheel panel BSM/MuJoCo",
            &time_s,
            &mujoco_s,
            Some(TF_S),
            RECORD_DT_S,
            false,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        let att_error = dyncomp::relative_principal_angles(
            "reaction-wheel panel attitude",
            &bsm.sigma_bn,
            &mujoco.sigma_bn,
        );
        let norm_of = |a: &[f64], b: &[f64]| {
            a.iter()
                .zip(b)
                .map(|(x, y)| (x - y) * (x - y))
                .sum::<f64>()
                .sqrt()
        };
        metrics.hub_attitude_principal_angle_max = Some(dyncomp::max_of(att_error.iter().copied()));
        metrics.hub_body_rate_error_max = Some(dyncomp::max_of(dyncomp::vector_differences(
            &bsm.omega_radps,
            &mujoco.omega_radps,
        )));
        metrics.wheel_speed_error_max = Some(dyncomp::max_of(
            bsm.wheel_speeds_radps
                .iter()
                .zip(&mujoco.wheel_speeds_radps)
                .map(|(a, b)| norm_of(a, b)),
        ));
        metrics.panel_angle_error_max = Some(dyncomp::max_of(
            bsm.panel_angles_rad
                .iter()
                .zip(&mujoco.panel_angles_rad)
                .map(|(a, b)| norm_of(a, b)),
        ));
        headers.extend(vector_headers("mujoco_omega_BN_B_rad_per_s"));
        headers.extend([
            "mujoco_panelP_theta_rad".to_string(),
            "mujoco_panelM_theta_rad".to_string(),
            "att_error_rad".to_string(),
        ]);
        for axis in 0..3 {
            columns.push(mujoco.omega_radps.iter().map(|w| w[axis]).collect());
        }
        for panel in 0..2 {
            columns.push(mujoco.panel_angles_rad.iter().map(|p| p[panel]).collect());
        }
        columns.push(att_error);
    }

    let rows: Vec<Vec<f64>> = (0..time_s.len())
        .map(|index| columns.iter().map(|column| column[index]).collect())
        .collect();
    let csv_path = table_csv::write_table(FILE_NAME, &headers, &rows);
    write_summary(&metrics);
    Results {
        metrics,
        bsm,
        csv_path,
    }
}

/// Upstream `writeJsonSummary`.
fn write_summary(metrics: &Metrics) {
    let mut summary = json!({"scenario": FILE_NAME, "nSamples": metrics.n_samples});
    for (key, value) in [
        (
            "hubAttitudePrincipalAngleMax",
            metrics.hub_attitude_principal_angle_max,
        ),
        ("hubBodyRateErrorMax", metrics.hub_body_rate_error_max),
        ("wheelSpeedErrorMax", metrics.wheel_speed_error_max),
        ("panelAngleErrorMax", metrics.panel_angle_error_max),
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
        "wrote {}: {:?}",
        results.csv_path.display(),
        results.metrics
    );
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

    /// `test_articulated_scenarios_have_nontrivial_absolute_motion`
    /// (reaction-wheel part): the BSM hub rate peaks at 0.0269258 rad/s,
    /// the wheels change speed by 22.5066 rad/s and the panels swing by
    /// 0.391224 rad (1 % each). With the `mujoco` feature,
    /// `_assert_rw_panel_metrics`: attitude within 2e-11 rad, rates within
    /// 1e-12 rad/s, hinge angles within 1e-9 rad.
    #[test]
    fn rw_panel_metrics_match_upstream() {
        let results = run(Config::default());
        let bsm = &results.bsm;
        let rate = dyncomp_max(bsm.omega_radps.iter().map(|w| w.norm()));
        assert_relative(rate, 0.0269258, 0.01);
        let first_wheels = bsm.wheel_speeds_radps[0];
        let wheel = dyncomp_max(
            bsm.wheel_speeds_radps
                .iter()
                .flat_map(|w| (0..3).map(move |i| (w[i] - first_wheels[i]).abs())),
        );
        assert_relative(wheel, 22.5066, 0.01);
        let first_panels = bsm.panel_angles_rad[0];
        let panel = dyncomp_max(
            bsm.panel_angles_rad
                .iter()
                .flat_map(|p| (0..2).map(move |i| (p[i] - first_panels[i]).abs())),
        );
        assert_relative(panel, 0.391224, 0.01);

        #[cfg(feature = "mujoco")]
        {
            let metrics = results.metrics;
            assert!(
                metrics.hub_attitude_principal_angle_max.unwrap() < 2.0e-11,
                "{metrics:?}"
            );
            assert!(
                metrics.hub_body_rate_error_max.unwrap() < 1.0e-12,
                "{metrics:?}"
            );
            assert!(
                metrics.wheel_speed_error_max.unwrap() < 1.0e-12,
                "{metrics:?}"
            );
            assert!(
                metrics.panel_angle_error_max.unwrap() < 1.0e-9,
                "{metrics:?}"
            );
        }
    }

    fn dyncomp_max(values: impl Iterator<Item = f64>) -> f64 {
        super::dyncomp::max_of(values)
    }
}
