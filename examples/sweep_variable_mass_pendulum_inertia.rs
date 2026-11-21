//! Port of upstream
//! `examples/dynamicsComparison/sweepVariableMassPendulumInertia.py`: the
//! deep-space burn of `scenario_compare_variable_mass` with the MuJoCo
//! slosh bob's initial centroidal inertia swept (default 1e-8 to 1e-3
//! kg m²; the BSM spherical pendulum's bob is a point mass, MuJoCo needs a
//! positive inertia) at several RK4 steps (default 0.05, 0.025, 0.0125 s),
//! separating the artificial-inertia contribution from the finite-step
//! disagreement of the two pendulum realizations. Needs the `mujoco`
//! feature.
//!
//! Outputs, in `examples/output/scenarios/`:
//! `sweepVariableMassPendulumInertia.json` and
//! `sweepVariableMassPendulumInertia.csv` (one row per inertia, one column
//! per step, for `plot_sweep_variable_mass_pendulum_inertia.py`).
//!
//! `tests` ports upstream's `variable-mass-inertia` reduced check: the
//! attitude difference grows with the bob inertia.

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
#[path = "support/table_csv.rs"]
mod table_csv;
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use cli::Cli;
use dyncomp::{
    Series, max_of, relative_principal_angles, seconds, validate_history,
    validate_matching_histories, validate_task_horizon, write_json,
};
use dyncomp_variable_mass::{
    SIM_DURATION_S, VariableMassCase, VariableMassHistory, run_bsm, run_mujoco,
};
use serde_json::json;

const FILE_NAME: &str = "sweepVariableMassPendulumInertia";

#[derive(Clone, Debug)]
pub struct Config {
    /// Upstream `bobInertias` \[kg m^2\], `timeSteps` \[s\], `simDuration` \[s\].
    pub bob_inertias: Vec<f64>,
    pub time_steps: Vec<f64>,
    pub sim_duration_s: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            bob_inertias: vec![1.0e-8, 1.0e-5, 1.0e-4, 1.0e-3],
            time_steps: vec![0.05, 0.025, 0.0125],
            sim_duration_s: SIM_DURATION_S,
        }
    }
}

/// Upstream `_bsmReference`.
fn bsm_reference(dt: f64, duration: f64) -> VariableMassHistory {
    let (bsm, _) = run_bsm(&VariableMassCase::new(true, false, duration), dt, true);
    validate_history(
        "variable-mass inertia-sweep BSM",
        &seconds(&bsm.times_nanos),
        Some(duration),
        dt,
        false,
        &[Series::vectors("attitude", &bsm.sigma_bn)],
    )
    .unwrap_or_else(|error| panic!("{error}"));
    bsm
}

/// Upstream `_mujocoAttitudeError` \[rad\].
fn mujoco_attitude_error(
    bsm: &VariableMassHistory,
    dt: f64,
    bob_inertia: f64,
    duration: f64,
) -> f64 {
    let case = VariableMassCase {
        pendulum_bob_inertia: bob_inertia,
        ..VariableMassCase::new(true, false, duration)
    };
    let (mujoco, _) = run_mujoco(&case, dt, true, bsm.initial_state());
    let mujoco_s = seconds(&mujoco.times_nanos);
    let check = |result: Result<Vec<f64>, String>| {
        result.unwrap_or_else(|error| panic!("{error}"));
    };
    check(validate_matching_histories(
        "variable-mass inertia-sweep BSM/MuJoCo",
        &seconds(&bsm.times_nanos),
        &mujoco_s,
        Some(duration),
        dt,
        false,
    ));
    check(validate_history(
        "variable-mass inertia-sweep MuJoCo",
        &mujoco_s,
        Some(duration),
        dt,
        false,
        &[Series::vectors("attitude", &mujoco.sigma_bn)],
    ));
    max_of(relative_principal_angles(
        "variable-mass attitude",
        &bsm.sigma_bn,
        &mujoco.sigma_bn,
    ))
}

/// The largest attitude difference \[rad\], per time step (outer) and bob
/// inertia (inner).
pub fn run(config: &Config) -> Vec<Vec<f64>> {
    assert!(
        !config.bob_inertias.is_empty() && !config.time_steps.is_empty(),
        "At least one bob inertia and time step are required."
    );
    let errors: Vec<Vec<f64>> = config
        .time_steps
        .iter()
        .map(|dt| {
            validate_task_horizon(
                "variable-mass pendulum-inertia sweep",
                config.sim_duration_s,
                *dt,
            )
            .unwrap_or_else(|error| panic!("{error}"));
            let bsm = bsm_reference(*dt, config.sim_duration_s);
            config
                .bob_inertias
                .iter()
                .map(|inertia| {
                    let error = mujoco_attitude_error(&bsm, *dt, *inertia, config.sim_duration_s);
                    println!("dt {dt} s, bob inertia {inertia:.0e} kg m^2: {error:.3e} rad");
                    error
                })
                .collect()
        })
        .collect();
    write_json(
        FILE_NAME,
        &json!({
            "scenario": FILE_NAME,
            "simDuration": config.sim_duration_s,
            "bobInertia": config.bob_inertias,
            "timeStep": config.time_steps,
            "attitudeErrorMax": errors,
        }),
    );
    let mut headers = vec!["bob_inertia_kg_m2".to_string()];
    headers.extend(
        config
            .time_steps
            .iter()
            .map(|dt| format!("att_error_rad_dt_{dt}")),
    );
    let rows: Vec<Vec<f64>> = config
        .bob_inertias
        .iter()
        .enumerate()
        .map(|(i, inertia)| {
            std::iter::once(*inertia)
                .chain(errors.iter().map(|per_step| per_step[i]))
                .collect()
        })
        .collect();
    let path = table_csv::write_table(FILE_NAME, &headers, &rows);
    println!("wrote {}", path.display());
    errors
}

const USAGE: &str = "\
sweep_variable_mass_pendulum_inertia [--inertias 1e-8,1e-5,1e-4,1e-3] [--steps 0.05,0.025,0.0125]
                                     [--duration S]";

fn list(args: &mut Cli, name: &str, default: Vec<f64>) -> Vec<f64> {
    let text: String = args.value(name, String::new());
    if text.is_empty() {
        return default;
    }
    text.split(',')
        .map(|v| {
            v.trim()
                .parse()
                .unwrap_or_else(|_| args.fail(&format!("invalid --{name} {v:?}")))
        })
        .collect()
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let defaults = Config::default();
    run(&Config {
        bob_inertias: list(&mut args, "inertias", defaults.bob_inertias),
        time_steps: list(&mut args, "steps", defaults.time_steps),
        sim_duration_s: args.value("duration", defaults.sim_duration_s),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `test_research_sweeps_capture_expected_physical_trends[variable-mass-inertia]`.
    #[test]
    fn attitude_difference_grows_with_the_bob_inertia() {
        let errors = run(&Config {
            bob_inertias: vec![1.0e-8, 1.0e-6, 1.0e-4],
            time_steps: vec![0.05],
            sim_duration_s: 1.0,
        });
        let errors = &errors[0];
        assert!(errors.iter().all(|e| e.is_finite()), "{errors:?}");
        assert!(
            errors.windows(2).all(|pair| pair[1] > pair[0]),
            "{errors:?}"
        );
    }
}
