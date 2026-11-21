//! Port of upstream
//! `examples/dynamicsComparison/scenarioCompareOrbitMultibody.py`: the
//! hub-wheels-panels spacecraft of `scenario_compare_rw_panels` on the
//! Keplerian orbit of `scenario_compare_orbit`, for one orbital period with
//! RK4 at 0.1 s, in two regimes:
//!
//! * **compact** (hub and wheels): every massive part at the hub origin, so
//!   gravity acts at one point and both engines reproduce Kepler's orbit to
//!   the RK4 truncation level (about 1e-7 m); the attitude agrees to about
//!   1e-4 rad, MuJoCo's gyroscopic-bias round-off at orbital speed;
//! * **extended** (with the offset panels): the BSM spacecraft applies
//!   gravity once at the system center of mass (no torque), MuJoCo's
//!   `NBodyGravity` at each body's center, whose differential forces torque
//!   the vehicle: the hub attitudes separate by about 0.8 rad, while the
//!   center-of-mass orbits still agree within 2e-6 m.
//!
//! The BSM spacecraft starts with its center of mass on the orbit
//! (upstream `hub.r_CN_NInit`); the MuJoCo hub origin is placed so the
//! scene's total center of mass starts there (`initialMujocoRootState`),
//! and an `MJSystemCoM` reports it. The BSM center of mass is the
//! spacecraft's `com_state_out`.
//!
//! Outputs, in `examples/output/scenarios/`:
//! `scenarioCompareOrbitMultibody.csv` (both regimes' orbit and attitude
//! differences every 60 s, for `plot_scenario_compare_orbit_multibody.py`),
//! `scenarioCompareOrbitMultibody.json` and, with `--timing`,
//! `scenarioCompareOrbitMultibody_runtime.csv`.
//!
//! `tests` ports `_assert_orbit_multibody_metrics` and
//! `test_orbit_multibody_compares_total_center_of_mass`; without the `mujoco`
//! feature only the BSM-versus-Kepler bounds run.

#[path = "support/cli.rs"]
mod cli;
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
#[path = "support/table_csv.rs"]
mod table_csv;

use cli::Cli;
use dyncomp::{
    DEFAULT_TIMING_REPEATS, IntegratorChoice, Series, max_of, recorder_period_nanos,
    save_runtime_table, seconds, validate_history, vector_differences, write_json,
};
use dyncomp_orbit::{initial_orbit_state, kepler_truth};
use dyncomp_rw_panels::{
    PANEL_INERTIA, PANEL_THETA0_RAD, RW_MASS_KG, RwPanelsCase, RwPanelsHistory, hub_inertia_bsm,
    initial_body_mass_distribution, run_bsm,
};
use grav_factory::Body;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::orbit_period_s;
use serde_json::json;

const FILE_NAME: &str = "scenarioCompareOrbitMultibody";
/// Upstream `dt` \[s\]: resolves the ~5 s panel oscillation.
const DT_S: f64 = 0.1;
/// Upstream `recordDt` \[s\].
const RECORD_DT_S: f64 = 60.0;

const USAGE: &str = "\
scenario_compare_orbit_multibody [--timing]
  --timing   also time both engines in both regimes and write
             scenarioCompareOrbitMultibody_runtime.csv";

#[derive(Clone, Copy, Debug, Default)]
pub struct Config {
    /// Upstream `saveTiming`.
    pub save_timing: bool,
}

/// One regime's JSON metrics; the cross-engine ones need MuJoCo.
#[derive(Clone, Copy, Debug, Default)]
pub struct RegimeMetrics {
    pub n_samples: usize,
    pub bsm_vs_kepler_max_m: f64,
    pub cross_paradigm_orbit_max_m: Option<f64>,
    pub cross_paradigm_attitude_max_rad: Option<f64>,
    pub cross_paradigm_wheel_speed_max_radps: Option<f64>,
    pub attitude_accel_scale_from_peak: Option<f64>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Metrics {
    pub gravity_gradient_rate_estimate: f64,
    pub compact: RegimeMetrics,
    pub extended: RegimeMetrics,
}

pub struct Results {
    pub metrics: Metrics,
    pub csv_path: std::path::PathBuf,
}

fn case(with_panels: bool, mu: f64) -> RwPanelsCase {
    RwPanelsCase {
        with_panels,
        gravity_mu: Some(mu),
        rw_mass_kg: RW_MASS_KG,
        integrator: IntegratorChoice::RK4,
    }
}

/// Upstream `gravityGradientRateEstimate`: the angular-acceleration scale
/// of the differential point gravity, `3 (mu/a^3) ΔI_point / (tr I / 3)`
/// \[rad/s^2\].
pub fn gravity_gradient_rate_estimate(mu: f64, semi_major_axis_m: f64) -> f64 {
    let (masses, centers, com) = initial_body_mass_distribution(true);
    let mut point_inertia = Matrix3::zeros();
    for (mass, center) in masses.iter().zip(&centers) {
        let offset = center - com;
        point_inertia +=
            *mass * (offset.dot(&offset) * Matrix3::identity() - offset * offset.transpose());
    }
    let (c, s) = (PANEL_THETA0_RAD.cos(), PANEL_THETA0_RAD.sin());
    let dcm_panel_to_hub = Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c);
    let panel_inertia_hub = dcm_panel_to_hub
        * Matrix3::from_diagonal(&Vector3::from(PANEL_INERTIA))
        * dcm_panel_to_hub.transpose();
    let total_inertia = point_inertia + hub_inertia_bsm() + 2.0 * panel_inertia_hub;
    let mut moments: Vec<f64> = point_inertia
        .symmetric_eigenvalues()
        .iter()
        .copied()
        .collect();
    moments.sort_by(f64::total_cmp);
    let delta_point_inertia = moments[2] - moments[0];
    3.0 * (mu / semi_major_axis_m.powi(3)) * delta_point_inertia / (total_inertia.trace() / 3.0)
}

fn validated(name: &str, history: &RwPanelsHistory, tf: f64) -> Vec<f64> {
    let wheels: Vec<Vec<f64>> = history
        .wheel_speeds_radps
        .iter()
        .map(|w| w.to_vec())
        .collect();
    validate_history(
        name,
        &seconds(&history.times_nanos),
        Some(tf),
        RECORD_DT_S,
        false,
        &[
            Series::vectors("position", &history.r_cn_n),
            Series::vectors("attitude", &history.sigma_bn),
            Series::rows("wheels", &wheels),
        ],
    )
    .unwrap_or_else(|error| panic!("{error}"))
}

/// Named CSV columns.
type Columns = Vec<(String, Vec<f64>)>;

/// One regime's metrics, plotted histories and sample times \[ns\].
fn run_regime(
    label: &str,
    with_panels: bool,
    mu: f64,
    tf: f64,
) -> (RegimeMetrics, Columns, Vec<u64>) {
    let (_, _, elements) = initial_orbit_state(mu);
    let period = recorder_period_nanos(DT_S, RECORD_DT_S);
    let (bsm, _) = run_bsm(&case(with_panels, mu), DT_S, tf, Some(period));
    let time_s = validated(&format!("orbiting multibody {label} BSM"), &bsm, tf);
    let truth = kepler_truth(mu, &elements, &time_s);
    let bsm_vs_kepler = vector_differences(&truth, &bsm.r_cn_n);
    #[cfg_attr(not(feature = "mujoco"), allow(unused_mut))]
    let mut metrics = RegimeMetrics {
        n_samples: time_s.len(),
        bsm_vs_kepler_max_m: max_of(bsm_vs_kepler.iter().copied()),
        ..RegimeMetrics::default()
    };
    #[cfg_attr(not(feature = "mujoco"), allow(unused_mut))]
    let mut columns = vec![(format!("{label}_bsm_vs_kepler_m"), bsm_vs_kepler)];

    #[cfg(feature = "mujoco")]
    {
        let (mujoco, _) =
            dyncomp_rw_panels::run_mujoco(&case(with_panels, mu), DT_S, tf, Some(period));
        let mujoco_s = validated(&format!("orbiting multibody {label} MuJoCo"), &mujoco, tf);
        dyncomp::validate_matching_histories(
            &format!("orbiting multibody {label} BSM/MuJoCo"),
            &time_s,
            &mujoco_s,
            Some(tf),
            RECORD_DT_S,
            false,
        )
        .unwrap_or_else(|error| panic!("{error}"));
        let att_error = dyncomp::relative_principal_angles(
            "orbiting multibody attitude",
            &bsm.sigma_bn,
            &mujoco.sigma_bn,
        );
        let wheel_error: Vec<f64> = bsm
            .wheel_speeds_radps
            .iter()
            .zip(&mujoco.wheel_speeds_radps)
            .map(|(a, b)| (Vector3::from(*a) - Vector3::from(*b)).norm())
            .collect();
        let cross_orbit = vector_differences(&bsm.r_cn_n, &mujoco.r_cn_n);
        let mujoco_vs_kepler = vector_differences(&truth, &mujoco.r_cn_n);
        let t_end = time_s[time_s.len() - 1];
        let peak = max_of(att_error.iter().copied());
        metrics.cross_paradigm_orbit_max_m = Some(max_of(cross_orbit.iter().copied()));
        metrics.cross_paradigm_attitude_max_rad = Some(peak);
        metrics.cross_paradigm_wheel_speed_max_radps = Some(max_of(wheel_error));
        metrics.attitude_accel_scale_from_peak = Some(2.0 * peak / (t_end * t_end));
        columns.extend([
            (format!("{label}_mujoco_vs_kepler_m"), mujoco_vs_kepler),
            (format!("{label}_cross_orbit_m"), cross_orbit),
            (format!("{label}_att_error_rad"), att_error),
        ]);
    }
    (metrics, columns, bsm.times_nanos)
}

pub fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let (_, _, elements) = initial_orbit_state(mu);
    // Upstream `tf`: one full orbital period (~5829 s).
    let tf = orbit_period_s(mu, elements.semi_major_axis);

    if config.save_timing {
        let mut rows = Vec::new();
        for (label, with_panels) in [
            ("Compact (hub + wheels)", false),
            ("Extended (+ offset panels)", true),
        ] {
            let regime = case(with_panels, mu);
            let build_bsm = move || run_bsm(&regime, DT_S, tf, None).1;
            #[cfg(feature = "mujoco")]
            let (bsm_s, mujoco_s) = {
                let build_mujoco = move || dyncomp_rw_panels::run_mujoco(&regime, DT_S, tf, None).1;
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
            rows.push((label.to_string(), bsm_s, mujoco_s));
        }
        let path = save_runtime_table(FILE_NAME, &rows, "Regime");
        println!("wrote {}", path.display());
    }

    let (compact, compact_columns, times) = run_regime("compact", false, mu, tf);
    let (extended, extended_columns, _) = run_regime("extended", true, mu, tf);
    let metrics = Metrics {
        gravity_gradient_rate_estimate: gravity_gradient_rate_estimate(
            mu,
            elements.semi_major_axis,
        ),
        compact,
        extended,
    };

    let mut headers = vec!["time_ns".to_string()];
    let mut columns = vec![times.iter().map(|t| *t as f64).collect::<Vec<f64>>()];
    for (name, values) in compact_columns.into_iter().chain(extended_columns) {
        headers.push(name);
        columns.push(values);
    }
    let rows: Vec<Vec<f64>> = (0..times.len())
        .map(|index| columns.iter().map(|column| column[index]).collect())
        .collect();
    let csv_path = table_csv::write_table(FILE_NAME, &headers, &rows);
    write_summary(&metrics);
    Results { metrics, csv_path }
}

/// Upstream `writeJsonSummary`.
fn write_summary(metrics: &Metrics) {
    let regime = |m: &RegimeMetrics| {
        let mut entry = json!({"nSamples": m.n_samples, "bsmVsKeplerMax": m.bsm_vs_kepler_max_m});
        for (key, value) in [
            ("crossParadigmOrbitMax", m.cross_paradigm_orbit_max_m),
            (
                "crossParadigmAttitudeMax",
                m.cross_paradigm_attitude_max_rad,
            ),
            (
                "crossParadigmWheelSpeedMax",
                m.cross_paradigm_wheel_speed_max_radps,
            ),
            (
                "attitudeAccelScaleFromPeak",
                m.attitude_accel_scale_from_peak,
            ),
        ] {
            if let Some(value) = value {
                entry[key] = json!(value);
            }
        }
        entry
    };
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "gravityGradientRateEstimate": metrics.gravity_gradient_rate_estimate,
            "compact": regime(&metrics.compact),
            "extended": regime(&metrics.extended),
        }),
    );
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let results = run(Config {
        save_timing: args.switch("timing"),
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

    /// `_assert_orbit_multibody_metrics`.
    #[test]
    fn orbit_multibody_metrics_match_upstream() {
        let metrics = run(Config::default()).metrics;
        let (compact, extended) = (metrics.compact, metrics.extended);
        assert!(compact.bsm_vs_kepler_max_m < 2.0e-7, "{compact:?}");
        assert!(extended.bsm_vs_kepler_max_m < 2.0e-6, "{extended:?}");
        #[cfg(feature = "mujoco")]
        {
            assert!(
                compact.cross_paradigm_orbit_max_m.unwrap() < 2.0e-7,
                "{compact:?}"
            );
            let compact_attitude = compact.cross_paradigm_attitude_max_rad.unwrap();
            assert!(compact_attitude < 2.0e-4, "{compact:?}");
            assert!(
                extended.cross_paradigm_orbit_max_m.unwrap() < 2.0e-6,
                "{extended:?}"
            );
            let extended_attitude = extended.cross_paradigm_attitude_max_rad.unwrap();
            assert!((0.7..0.9).contains(&extended_attitude), "{extended:?}");
            assert!(extended_attitude > 1.0e3 * compact_attitude);
        }
    }

    /// `test_orbit_multibody_compares_total_center_of_mass`: with the panels,
    /// both engines start from the same total center of mass, within 1e-6 m
    /// and 1e-8 m/s.
    #[cfg(feature = "mujoco")]
    #[test]
    fn engines_start_from_the_same_total_center_of_mass() {
        use super::{case, dyncomp_rw_panels};
        let mu = super::Body::Earth.mu_m3ps2();
        let dt = 0.01;
        let period = super::dyncomp::sec2nano_u64(dt);
        let (bsm, _) = dyncomp_rw_panels::run_bsm(&case(true, mu), dt, dt, Some(period));
        let (mujoco, _) = dyncomp_rw_panels::run_mujoco(&case(true, mu), dt, dt, Some(period));
        assert!((bsm.r_cn_n[0] - mujoco.r_cn_n[0]).norm() < 1.0e-6);
        assert!((bsm.v_cn_n[0] - mujoco.v_cn_n[0]).norm() < 1.0e-8);
    }
}
