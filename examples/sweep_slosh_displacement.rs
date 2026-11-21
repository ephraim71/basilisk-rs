//! Port of upstream `examples/dynamicsComparison/sweepSloshDisplacement.py`:
//! the largest cross-engine difference of the three spring-mass-damper
//! slosh displacements of `scenario_compare_variable_mass` over the burn,
//! in deep space and in orbit. Needs the `mujoco` feature.
//!
//! Output: `examples/output/scenarios/sweepSloshDisplacement.json`.
//!
//! `tests` ports upstream's `slosh-displacement` reduced check (0.1 s):
//! finite, and larger in orbit than in deep space.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_variable_mass.rs"]
mod dyncomp_variable_mass;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use cli::Cli;
use dyncomp::{
    Series, seconds, validate_history, validate_matching_histories, validate_task_horizon,
    write_json,
};
use dyncomp_variable_mass::{VariableMassCase, run_bsm, run_mujoco, time_step};
use serde_json::json;

const FILE_NAME: &str = "sweepSloshDisplacement";

/// Upstream `maxSloshDifference` \[m\].
pub fn max_slosh_difference(in_orbit: bool, tf_s: f64) -> f64 {
    let dt = time_step();
    validate_task_horizon("slosh-displacement sweep", tf_s, dt)
        .unwrap_or_else(|error| panic!("{error}"));
    let case = VariableMassCase::new(true, in_orbit, tf_s);
    let (bsm, _) = run_bsm(&case, dt, true);
    let (mujoco, _) = run_mujoco(&case, dt, true, bsm.initial_state());
    let (bsm_times, mujoco_times) = (seconds(&bsm.times_nanos), seconds(&mujoco.times_nanos));
    let rows = |h: &dyncomp_variable_mass::VariableMassHistory| -> Vec<Vec<f64>> {
        h.rho.iter().map(|r| r.to_vec()).collect()
    };
    let check = |result: Result<Vec<f64>, String>| {
        result.unwrap_or_else(|error| panic!("{error}"));
    };
    check(validate_matching_histories(
        "slosh-displacement BSM/MuJoCo",
        &bsm_times,
        &mujoco_times,
        Some(tf_s),
        dt,
        false,
    ));
    check(validate_history(
        "slosh-displacement BSM",
        &bsm_times,
        Some(tf_s),
        dt,
        false,
        &[Series::rows("displacement", &rows(&bsm))],
    ));
    check(validate_history(
        "slosh-displacement MuJoCo",
        &mujoco_times,
        Some(tf_s),
        dt,
        false,
        &[Series::rows("displacement", &rows(&mujoco))],
    ));
    dyncomp::max_of(
        bsm.rho
            .iter()
            .zip(&mujoco.rho)
            .flat_map(|(a, b)| (0..3).map(move |i| (a[i] - b[i]).abs())),
    )
}

/// `(key, maxAbsRhoDiff)` per configuration (`deepSpace`, `inOrbit`).
pub fn run(configurations: &[bool], tf_s: f64) -> Vec<(&'static str, f64)> {
    let out: Vec<(&'static str, f64)> = configurations
        .iter()
        .map(|in_orbit| {
            let key = if *in_orbit { "inOrbit" } else { "deepSpace" };
            let difference = max_slosh_difference(*in_orbit, tf_s);
            println!("{key}: max slosh displacement diff {difference:.3e} m");
            (key, difference)
        })
        .collect();
    let mut by_key = serde_json::Map::new();
    for (key, value) in &out {
        by_key.insert(key.to_string(), json!(value));
    }
    let path = write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "maxAbsRhoDiff_m": by_key,
            "configuration": {
                "dt": time_step(),
                "tf": tf_s,
                "note": "SMD rho1..3 BSM vs MuJoCo, uncompensated",
            },
        }),
    );
    println!("wrote {}", path.display());
    out
}

const USAGE: &str = "sweep_slosh_displacement [--duration S]";

fn main() {
    let mut args = Cli::from_env(USAGE);
    run(&[false, true], args.value("duration", 900.0));
}

#[cfg(test)]
mod tests {
    use super::run;

    /// `test_research_sweeps_capture_expected_physical_trends[slosh-displacement]`.
    #[test]
    fn orbit_separates_the_slosh_more_than_deep_space() {
        let out = run(&[false, true], 0.1);
        assert!(out.iter().all(|(_, value)| value.is_finite()), "{out:?}");
        assert!(out[1].1 > out[0].1, "{out:?}");
    }
}
