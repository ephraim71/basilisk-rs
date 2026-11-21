//! Port of upstream `examples/dynamicsComparison/sweepRwBookkeepingMass.py`:
//! `scenario_compare_rw_panels` rerun with the MuJoCo wheel bodies'
//! bookkeeping mass reduced (default 1e-6, 1e-8, 1e-10 kg), confirming that
//! this mass, which the ideally massless BSM balanced wheels do not have,
//! sets the cross-engine hub-attitude agreement floor. Needs the `mujoco`
//! feature.
//!
//! Outputs, in `examples/output/scenarios/`: `sweepRwBookkeepingMass.json`
//! and `sweepRwBookkeepingMass.csv` (one row per mass).
//!
//! `tests` ports upstream's `reaction-wheel-mass` reduced sweep check: the
//! floor falls one decade per decade of mass (within 0.02 decades).

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
    Series, max_of, recorder_period_nanos, recorder_sample_interval, relative_principal_angles,
    seconds, validate_history, validate_matching_histories, validate_task_horizon, write_json,
};
use dyncomp_rw_panels::{RwPanelsCase, run_bsm, run_mujoco};
use serde_json::json;

const FILE_NAME: &str = "sweepRwBookkeepingMass";

#[derive(Clone, Debug)]
pub struct Config {
    /// Upstream `rwMasses` \[kg\].
    pub rw_masses: Vec<f64>,
    pub dt_s: f64,
    pub tf_s: f64,
    pub record_dt_s: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            rw_masses: vec![1.0e-6, 1.0e-8, 1.0e-10],
            dt_s: 0.02,
            tf_s: 120.0,
            record_dt_s: 0.5,
        }
    }
}

/// Upstream `attitudeFloor`: the largest cross-engine hub-attitude angle
/// with MuJoCo wheels of `rw_mass` \[rad\].
pub fn attitude_floor(rw_mass: f64, config: &Config) -> f64 {
    validate_task_horizon(
        "reaction-wheel bookkeeping-mass sweep",
        config.tf_s,
        config.dt_s,
    )
    .unwrap_or_else(|error| panic!("{error}"));
    let case = RwPanelsCase {
        rw_mass_kg: rw_mass,
        ..RwPanelsCase::FREE
    };
    let period = Some(recorder_period_nanos(config.dt_s, config.record_dt_s));
    let (bsm, _) = run_bsm(&case, config.dt_s, config.tf_s, period);
    let (mujoco, _) = run_mujoco(&case, config.dt_s, config.tf_s, period);
    let interval = recorder_sample_interval(config.dt_s, config.record_dt_s).expect("interval");
    let (bsm_times, mujoco_times) = (seconds(&bsm.times_nanos), seconds(&mujoco.times_nanos));
    let check = |result: Result<Vec<f64>, String>| {
        result.unwrap_or_else(|error| panic!("{error}"));
    };
    check(validate_matching_histories(
        "reaction-wheel bookkeeping-mass BSM/MuJoCo",
        &bsm_times,
        &mujoco_times,
        Some(config.tf_s),
        interval,
        false,
    ));
    check(validate_history(
        "reaction-wheel bookkeeping-mass BSM",
        &bsm_times,
        Some(config.tf_s),
        interval,
        false,
        &[Series::vectors("attitude", &bsm.sigma_bn)],
    ));
    check(validate_history(
        "reaction-wheel bookkeeping-mass MuJoCo",
        &mujoco_times,
        Some(config.tf_s),
        interval,
        false,
        &[Series::vectors("attitude", &mujoco.sigma_bn)],
    ));
    max_of(relative_principal_angles(
        "reaction-wheel panel attitude",
        &bsm.sigma_bn,
        &mujoco.sigma_bn,
    ))
}

/// `(rwMass, hubAttitudePrincipalAngleMax)` per mass.
pub fn run(config: &Config) -> Vec<(f64, f64)> {
    let rows: Vec<(f64, f64)> = config
        .rw_masses
        .iter()
        .map(|mass| {
            let floor = attitude_floor(*mass, config);
            println!("RW_MASS {mass:.0e} kg -> attitude floor {floor:.3e} rad");
            (*mass, floor)
        })
        .collect();
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "configuration": {
                "dt": config.dt_s,
                "tf": config.tf_s,
                "recordDt": config.record_dt_s,
                "note": "scenarioCompareRwPanels with swept MuJoCo wheel bookkeeping mass",
            },
            "rows": rows
                .iter()
                .map(|(mass, floor)| json!({"rwMass": mass, "hubAttitudePrincipalAngleMax": floor}))
                .collect::<Vec<_>>(),
        }),
    );
    let path = table_csv::write_table(
        FILE_NAME,
        &[
            "rw_mass_kg".to_string(),
            "hub_attitude_principal_angle_max_rad".to_string(),
        ],
        &rows.iter().map(|(m, f)| vec![*m, *f]).collect::<Vec<_>>(),
    );
    println!("wrote {}", path.display());
    rows
}

const USAGE: &str = "\
sweep_rw_bookkeeping_mass [--masses 1e-6,1e-8,1e-10] [--dt S] [--duration S] [--record-dt S]";

fn main() {
    let mut args = Cli::from_env(USAGE);
    let mut config = Config::default();
    let masses: String = args.value("masses", String::new());
    if !masses.is_empty() {
        config.rw_masses = masses
            .split(',')
            .map(|m| {
                m.trim()
                    .parse()
                    .unwrap_or_else(|_| args.fail(&format!("invalid mass {m:?}")))
            })
            .collect();
    }
    config.dt_s = args.value("dt", config.dt_s);
    config.tf_s = args.value("duration", config.tf_s);
    config.record_dt_s = args.value("record-dt", config.record_dt_s);
    run(&config);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `test_research_sweeps_capture_expected_physical_trends[reaction-wheel-mass]`.
    #[test]
    fn floor_scales_with_the_bookkeeping_mass() {
        let rows = run(&Config {
            rw_masses: vec![1.0e-4, 1.0e-6, 1.0e-8],
            dt_s: 0.02,
            tf_s: 2.0,
            record_dt_s: 0.1,
        });
        assert!(rows.iter().all(|(_, floor)| *floor > 0.0), "{rows:?}");
        for pair in rows.windows(2) {
            let ((m0, f0), (m1, f1)) = (pair[0], pair[1]);
            assert!(f1 < f0, "{rows:?}");
            let floor_decades = f1.log10() - f0.log10();
            let mass_decades = m1.log10() - m0.log10();
            assert!((floor_decades - mass_decades).abs() <= 0.02, "{rows:?}");
        }
    }
}
