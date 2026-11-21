//! Port of upstream
//! `examples/dynamicsComparison/sweepTorqueMechanismChecks.py`: direct
//! controls of the floating-point cancellation behind the velocity-dependent
//! MuJoCo attitude artifact of `scenario_compare_torque`. A quiet free body
//! (no gravity, torque or initial rate) drifts at constant speed:
//!
//! 1. the initial generalized bias force (an `MJJointReactionForces`) is
//!    nonzero along the generic direction `[2, -3, 6]/7`;
//! 2. along a coordinate axis, where the cancellation is exact, it and the
//!    attitude drift vanish;
//! 3. doubling the speed multiplies the attitude drift by four;
//! 4. doubling the mass (inertia fixed) multiplies it by two.
//!
//! Needs the `mujoco` feature. Output:
//! `examples/output/scenarios/sweepTorqueMechanismChecks.json`.
//!
//! `tests` ports upstream's `torque-mechanism` reduced check.

#[path = "support/common.rs"]
mod common;
#[path = "support/dyncomp.rs"]
mod dyncomp;
#[path = "support/dyncomp_orbit.rs"]
mod dyncomp_orbit;
#[path = "support/dyncomp_torque.rs"]
mod dyncomp_torque;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use dyncomp::{
    Series, max_of, relative_principal_angles, seconds, validate_history, validate_task_horizon,
    write_json,
};
use dyncomp_torque::{MASS_KG, TorqueCase, run_mujoco};
use nalgebra::Vector3;
use scenario_args::ScenarioArgs;
use serde_json::json;

const FILE_NAME: &str = "sweepTorqueMechanismChecks";

/// Upstream `GENERIC_DIRECTION`, `AXIS_DIRECTION`.
pub fn generic_direction() -> Vector3<f64> {
    Vector3::new(2.0, -3.0, 6.0) / 7.0
}
pub fn axis_direction() -> Vector3<f64> {
    Vector3::x()
}

#[derive(Clone, Debug)]
pub struct Config {
    /// Upstream `speeds` \[m/s\], `masses` \[kg\].
    pub speeds: Vec<f64>,
    pub masses: Vec<f64>,
    pub time_step_s: f64,
    pub sim_duration_s: f64,
    pub record_step_s: f64,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            speeds: vec![937.5, 1875.0, 3750.0, 7500.0],
            masses: vec![750.0, 1500.0],
            time_step_s: 0.1,
            sim_duration_s: 600.0,
            record_step_s: 1.0,
        }
    }
}

/// Upstream `quietMujocoRun`: the largest attitude drift \[rad\] and, with
/// `capture_bias`, the initial `biasForces`.
pub fn quiet_mujoco_run(
    direction: Vector3<f64>,
    speed: f64,
    mass: f64,
    capture_bias: bool,
    config: &Config,
) -> (f64, Option<Vec<f64>>) {
    validate_task_horizon(
        "torque mechanism control",
        config.sim_duration_s,
        config.time_step_s,
    )
    .unwrap_or_else(|error| panic!("{error}"));
    let case = TorqueCase {
        mass_kg: mass,
        torque_b: None,
        omega0_b: Vector3::zeros(),
        position_m: Vector3::zeros(),
        velocity_mps: speed * direction,
        gravity_mu: None,
    };
    let run = run_mujoco(
        &case,
        config.time_step_s,
        config.sim_duration_s,
        config.record_step_s,
        true,
        capture_bias,
    );
    let sigma = &run.history.sigma_bn;
    validate_history(
        "torque mechanism MuJoCo",
        &seconds(&run.history.times_nanos),
        Some(config.sim_duration_s),
        config.record_step_s,
        false,
        &[Series::vectors("attitude", sigma)],
    )
    .unwrap_or_else(|error| panic!("{error}"));
    let zeros = vec![Vector3::zeros(); sigma.len()];
    let drift = max_of(relative_principal_angles("quiet attitude", sigma, &zeros));
    (drift, run.initial_bias_forces)
}

/// Upstream `_scaledRows`: each result with its ratio to the previous one.
fn scaled_rows(
    key: &str,
    values: &[f64],
    mut evaluate: impl FnMut(f64) -> f64,
) -> Vec<serde_json::Value> {
    let mut previous: Option<f64> = None;
    values
        .iter()
        .map(|value| {
            let result = evaluate(*value);
            let row = json!({
                key: value,
                "mujocoAttitudeMax": result,
                "ratioToPrevious": previous.map(|p| result / p),
            });
            previous = Some(result);
            row
        })
        .collect()
}

pub fn run(config: &Config) -> serde_json::Value {
    assert!(
        !config.speeds.is_empty() && !config.masses.is_empty(),
        "At least one speed and mass are required."
    );
    let fastest = config.speeds[config.speeds.len() - 1];
    let mut generic_bias = None;
    let velocity_rows = scaled_rows("speed", &config.speeds, |speed| {
        let (drift, bias) = quiet_mujoco_run(
            generic_direction(),
            speed,
            MASS_KG,
            speed == fastest,
            config,
        );
        if bias.is_some() {
            generic_bias = bias;
        }
        drift
    });
    let (axis_attitude, axis_bias) =
        quiet_mujoco_run(axis_direction(), fastest, MASS_KG, true, config);
    let fastest_drift = velocity_rows[velocity_rows.len() - 1]["mujocoAttitudeMax"]
        .as_f64()
        .expect("drift");
    let mass_rows = scaled_rows("mass", &config.masses, |mass| {
        if mass == MASS_KG {
            fastest_drift
        } else {
            quiet_mujoco_run(generic_direction(), fastest, mass, false, config).0
        }
    });
    let direction = generic_direction();
    let metrics = json!({
        "scenario": FILE_NAME,
        "configuration": {
            "dt": config.time_step_s,
            "tf": config.sim_duration_s,
            "recordDt": config.record_step_s,
            "genericDirection": [direction.x, direction.y, direction.z],
            "axisDirection": [1.0, 0.0, 0.0],
            "speed": fastest,
            "quietBody": "no gravity, applied torque, or initial body rate",
        },
        "genericInitialBiasForces": generic_bias.expect("generic bias"),
        "axisInitialBiasForces": axis_bias.expect("axis bias"),
        "quietAxisAttitudeMax": axis_attitude,
        "quietVelocityScaling": velocity_rows,
        "quietMassScaling": mass_rows,
    });
    let path = write_json(FILE_NAME, &metrics);
    println!(
        "{}\nwrote {}",
        serde_json::to_string_pretty(&metrics).expect("metrics"),
        path.display()
    );
    metrics
}

const USAGE: &str = "\
sweep_torque_mechanism_checks [--speeds 937.5,1875,3750,7500] [--masses 750,1500]
                              [--dt S] [--duration S] [--record-dt S]";

fn list(args: &ScenarioArgs, name: &str, default: Vec<f64>) -> Vec<f64> {
    let text: String = args.value(name, String::new());
    if text.is_empty() {
        return default;
    }
    text.split(',')
        .map(|v| {
            v.trim()
                .parse()
                .unwrap_or_else(|_| args.fail(&format!("invalid {name} {v:?}")))
        })
        .collect()
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let defaults = Config::default();
    run(&Config {
        speeds: list(&args, "--speeds", defaults.speeds),
        masses: list(&args, "--masses", defaults.masses),
        time_step_s: args.value("--dt", defaults.time_step_s),
        sim_duration_s: args.value("--duration", defaults.sim_duration_s),
        record_step_s: args.value("--record-dt", defaults.record_step_s),
    });
}

#[cfg(test)]
mod tests {
    use super::*;

    fn norm(values: &serde_json::Value) -> f64 {
        values
            .as_array()
            .expect("forces")
            .iter()
            .map(|v| v.as_f64().expect("finite force").powi(2))
            .sum::<f64>()
            .sqrt()
    }

    /// `test_research_sweeps_capture_expected_physical_trends[torque-mechanism]`.
    #[test]
    fn cancellation_mechanism_controls_match_upstream() {
        let metrics = run(&Config {
            speeds: vec![3750.0, 7500.0],
            masses: vec![750.0, 1500.0],
            time_step_s: 0.1,
            sim_duration_s: 0.1,
            record_step_s: 0.1,
        });
        let generic = norm(&metrics["genericInitialBiasForces"]);
        let axis = norm(&metrics["axisInitialBiasForces"]);
        assert!(generic > 0.0 && generic.is_finite(), "{metrics}");
        assert!(axis < 1.0e-8 * generic, "{metrics}");
        let velocity = metrics["quietVelocityScaling"].as_array().unwrap();
        let fastest = &velocity[velocity.len() - 1];
        let quiet_axis = metrics["quietAxisAttitudeMax"].as_f64().unwrap();
        assert!(
            quiet_axis < 1.0e-8 * fastest["mujocoAttitudeMax"].as_f64().unwrap(),
            "{metrics}"
        );
        let ratio = fastest["ratioToPrevious"].as_f64().unwrap();
        assert!(
            (ratio - 4.0).abs() <= 1.0e-6 * 4.0,
            "velocity ratio {ratio}"
        );
        let mass = metrics["quietMassScaling"].as_array().unwrap();
        let ratio = mass[mass.len() - 1]["ratioToPrevious"].as_f64().unwrap();
        assert!((ratio - 2.0).abs() <= 1.0e-6 * 2.0, "mass ratio {ratio}");
    }
}
