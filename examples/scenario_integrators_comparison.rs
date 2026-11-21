//! Port of upstream `examples/scenarioIntegratorsComparison.py`: the final
//! position error, against the two-body (Kepler) solution, of a LEO
//! spacecraft integrated over one orbit (rounded up to a multiple of
//! 2048 s) by seven integrators, each at the time steps 2, 4, ... 2048 s.
//!
//! | upstream key | label | here |
//! |---|---|---|
//! | `euler` | Euler | [`basilisk_rs::integrators::Euler`] |
//! | `rk2` | Heun | [`basilisk_rs::integrators::Heun`] |
//! | `rk4` | RK 4 | [`basilisk_rs::integrators::Rk4`] |
//! | `rkf45 10`, `rkf45 0.1` | RKF4(5), epsilon = 10, 0.1 | [`Rkf45::upstream`] |
//! | `rkf78 10`, `rkf78 0.1` | RKF7(8), epsilon = 10, 0.1 | [`Rkf78::upstream`] |
//!
//! The adaptive cases use upstream's step-size control
//! ([`basilisk_rs::integrators::StepControl::Upstream`]) and set
//! `relTol = 0` and `absTol = epsilon`, as upstream.
//! The results go to `scenarioIntegratorsComparison.csv`: one row per time
//! step, with each integrator's position error (m) and wall-clock run time
//! (s), the two panels of upstream's figure.
//!
//! `test_scenarioIntegratorsComparison.py` only runs the scenario. `tests`
//! checks what the figure shows: each fixed-step method converges at its
//! order, and each adaptive method holds its error far below the fixed-step
//! ones at the coarse steps, tighter for the tighter tolerance.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/table_csv.rs"]
mod table_csv;

use std::time::Instant;

use basilisk_rs::integrators::{AdaptiveController, Euler, Heun, Integrator, Rk4, Rkf45, Rkf78};
use basilisk_rs::schedule;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use grav_factory::Body;
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{kepler_state, orbit_period_s, sec2nano};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_integrators_comparison     run every integrator at every time step";

/// One entry of upstream's `INTEGRATORS`.
#[derive(Clone, Copy, Debug)]
pub struct IntegratorData {
    /// Upstream's dictionary key.
    pub key: &'static str,
    /// Upstream's legend label.
    pub label: &'static str,
    /// Upstream `absTol` (with `relTol = 0`), for the adaptive methods.
    pub abs_tol: Option<f64>,
}

/// Upstream `INTEGRATORS`, in order.
pub const INTEGRATORS: [IntegratorData; 7] = [
    IntegratorData {
        key: "euler",
        label: "Euler",
        abs_tol: None,
    },
    IntegratorData {
        key: "rk2",
        label: "Heun",
        abs_tol: None,
    },
    IntegratorData {
        key: "rk4",
        label: "RK 4",
        abs_tol: None,
    },
    IntegratorData {
        key: "rkf45 10",
        label: "RKF4(5) - eps = 10",
        abs_tol: Some(10.0),
    },
    IntegratorData {
        key: "rkf45 0.1",
        label: "RKF4(5) - eps = 0.1",
        abs_tol: Some(0.1),
    },
    IntegratorData {
        key: "rkf78 10",
        label: "RKF7(8) - eps = 10",
        abs_tol: Some(10.0),
    },
    IntegratorData {
        key: "rkf78 0.1",
        label: "RKF7(8) - eps = 0.1",
        abs_tol: Some(0.1),
    },
];

impl IntegratorData {
    fn integrator(&self) -> Box<dyn Integrator<Spacecraft>> {
        match (self.key.split(' ').next(), self.abs_tol) {
            (Some("euler"), _) => Box::new(Euler),
            (Some("rk2"), _) => Box::new(Heun),
            (Some("rk4"), _) => Box::new(Rk4),
            (Some("rkf45"), Some(abs_tol)) => Box::new(Rkf45::new(upstream_control(abs_tol))),
            (Some("rkf78"), Some(abs_tol)) => Box::new(Rkf78::new(upstream_control(abs_tol))),
            _ => unreachable!("unknown integrator {}", self.key),
        }
    }

    /// The CSV column prefix: the key with `_` for the space.
    pub fn column(&self) -> String {
        self.key.replace(' ', "_")
    }
}

/// Upstream's adaptive step control with `relTol = 0` and `absTol = abs_tol`.
fn upstream_control(abs_tol: f64) -> AdaptiveController {
    AdaptiveController {
        rtol: 0.0,
        atol: abs_tol,
        ..AdaptiveController::upstream()
    }
}

/// Upstream `time_steps_to_test`: `2 ** [1 .. 11]` s.
pub fn time_steps_s() -> Vec<f64> {
    (1..=11).map(|power| 2.0_f64.powi(power)).collect()
}

/// Upstream `get_initial_conditions`: the LEO state and its `rv2elem`
/// elements.
fn initial_conditions() -> (Vector3<f64>, Vector3<f64>, ClassicElements) {
    let degrees = std::f64::consts::PI / 180.0;
    let mu = Body::Earth.mu_m3ps2();
    let (position, velocity) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: 7_000.0 * 1_000.0,
            eccentricity: 0.0001,
            inclination_rad: 33.3 * degrees,
            raan_rad: 48.2 * degrees,
            arg_periapsis_rad: 347.8 * degrees,
            true_anomaly_rad: 85.3 * degrees,
            ..Default::default()
        },
    );
    (
        position,
        velocity,
        state_to_elements(mu, position, velocity),
    )
}

/// Upstream `get_simulation_time`: the orbit period rounded up to a
/// multiple of 2048 s.
pub fn simulation_time_s() -> f64 {
    let (_, _, elements) = initial_conditions();
    let divisor = 2.0_f64.powi(11);
    let period = orbit_period_s(Body::Earth.mu_m3ps2(), elements.semi_major_axis);
    ((period / divisor).floor() + 1.0) * divisor
}

/// Upstream `get_reference`: the Kepler position at the final time.
pub fn reference_position_m() -> Vector3<f64> {
    let (_, _, elements) = initial_conditions();
    kepler_state(Body::Earth.mu_m3ps2(), &elements, simulation_time_s()).0
}

/// Upstream `get_solution`: the position at the final time.
pub fn solution_position_m(integrator: &IntegratorData, time_step_s: f64) -> Vector3<f64> {
    let (position, velocity, _) = initial_conditions();
    let step_nanos = sec2nano(time_step_s);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: step_nanos,
        initial_position_m: position,
        initial_velocity_mps: velocity,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: Some(integrator.integrator()),
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central gravity body");
    let state = spacecraft.state_out.clone();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        schedule! { simulation,
            "spacecraft" => &mut spacecraft, period=step_nanos, priority=20;
        }
        simulation.run_for(sec2nano(simulation_time_s()));
    }
    state.read().position_m
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    /// Upstream `errors[integrator]` (m), one per time step, in
    /// [`INTEGRATORS`] order.
    pub errors_m: Vec<Vec<f64>>,
    /// Upstream `times[integrator]` (s).
    pub run_times_s: Vec<Vec<f64>>,
}

/// Upstream `run`.
pub fn run() -> Results {
    let reference = reference_position_m();
    let steps = time_steps_s();
    let mut errors_m = Vec::new();
    let mut run_times_s = Vec::new();
    for integrator in &INTEGRATORS {
        let mut errors = Vec::new();
        let mut times = Vec::new();
        for &step in &steps {
            let tic = Instant::now();
            let position = solution_position_m(integrator, step);
            times.push(tic.elapsed().as_secs_f64());
            errors.push((reference - position).norm());
        }
        errors_m.push(errors);
        run_times_s.push(times);
    }

    let mut headers = vec!["time_step_s".to_string()];
    for integrator in &INTEGRATORS {
        headers.push(format!("{}_position_error_m", integrator.column()));
    }
    for integrator in &INTEGRATORS {
        headers.push(format!("{}_run_time_s", integrator.column()));
    }
    let rows: Vec<Vec<f64>> = steps
        .iter()
        .enumerate()
        .map(|(index, &step)| {
            std::iter::once(step)
                .chain(errors_m.iter().map(|errors| errors[index]))
                .chain(run_times_s.iter().map(|times| times[index]))
                .collect()
        })
        .collect();
    let csv_path = table_csv::write_table("scenarioIntegratorsComparison", &headers, &rows);
    Results {
        csv_path,
        errors_m,
        run_times_s,
    }
}

fn main() {
    ScenarioArgs::from_env(USAGE);
    let results = run();
    for (integrator, errors) in INTEGRATORS.iter().zip(&results.errors_m) {
        println!(
            "{:>20}: error {:.3e} m at 2 s, {:.3e} m at 2048 s",
            integrator.label,
            errors[0],
            errors[errors.len() - 1]
        );
    }
    println!("wrote {}", results.csv_path.display());
}

#[cfg(test)]
mod tests {
    use super::{INTEGRATORS, run, simulation_time_s, time_steps_s};

    /// `test_scenarioIntegratorsComparison.py` only runs the scenario; this
    /// checks the convergence its figure shows.
    #[test]
    fn integrators_converge_at_their_order() {
        assert_eq!(simulation_time_s(), 6144.0);
        let results = run();
        let steps = time_steps_s();
        let error = |key: &str, step: f64| {
            let integrator = INTEGRATORS.iter().position(|i| i.key == key).unwrap();
            let index = steps.iter().position(|s| *s == step).unwrap();
            results.errors_m[integrator][index]
        };
        // Halving the step divides the error by about 2^order, in the range
        // where truncation error dominates.
        for (key, order, coarse) in [("euler", 1, 8.0), ("rk2", 2, 32.0), ("rk4", 4, 64.0)] {
            let ratio = error(key, coarse) / error(key, coarse / 2.0);
            let expected = 2.0_f64.powi(order);
            assert!(
                (ratio / expected - 1.0).abs() < 0.25,
                "{key}: error ratio {ratio} for order {order}"
            );
        }
        // At the coarse steps, where the tolerance sets the sub-steps, the
        // adaptive methods hold the error far below RK4's, and the tighter
        // tolerance gives the smaller error.
        for method in ["rkf45", "rkf78"] {
            for &step in &steps[8..] {
                let loose = error(&format!("{method} 10"), step);
                let tight = error(&format!("{method} 0.1"), step);
                assert!(tight < loose, "{method} at {step} s: {tight} >= {loose}");
                assert!(
                    loose < 1.0e-2 * error("rk4", step),
                    "{method} at {step} s: {loose} m"
                );
            }
        }
    }
}
