//! Port of upstream `examples/scenarioVariableTimeStepIntegrators.py`: a
//! highly eccentric Earth orbit (a = 160,000 km, e = 0.8) integrated with
//! the variable-step RKF4(5) or RKF7(8) under a coarse 2 h task period,
//! which the adaptive methods split into the sub-steps their tolerances
//! need, and the accuracy study `runAccuracyComparison`.
//!
//! `run` (upstream `run(show_plots, integratorCase, relTol, absTol,
//! simulationTimeStep)`) integrates 0.9 orbits with
//!
//! | `--integrator` | here |
//! |---|---|
//! | `rk4` | basilisk-rs's default [`basilisk_rs::integrators::Rk4`] (no tolerances) |
//! | `rkf45` | [`integrators::UpstreamAdaptive::rkf45`] |
//! | `rkf78` (default) | [`integrators::UpstreamAdaptive::rkf78`] |
//!
//! and `--rel-tol`, `--abs-tol` (defaults `1e-5`, `1e-8`, upstream's
//! `__main__`) and `--step-s` (default 7200). It records the state at every
//! task update (upstream's `samplingTime` rounds down to 1 ns) to
//! `scenarioVariableTimeStepIntegrators<case><tolerances>.csv`, where
//! `<tolerances>` is `DefaultTol` (`1e-4`, `1e-8`), `TightTol` (`1e-8`,
//! `1e-12`), with `10min` appended for a 600 s task, or the values.
//!
//! `--all` runs `test_scenarioVariableTimeStepIntegrators.py`: both tight
//! cases and the five cases of `runAccuracyComparison`, whose position
//! errors against the Kepler solution from the first sample go to
//! `scenarioVariableTimeStepIntegratorsAccuracy.csv` (`case` is the index
//! in [`ACCURACY_CASES`]). Both upstream tests are ported in `tests`.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/integrators.rs"]
mod integrators;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;
#[path = "support/table_csv.rs"]
mod table_csv;

use basilisk_rs::integrators::Integrator;
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use grav_factory::Body;
use hifitime::Epoch;
use integrators::UpstreamAdaptive;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{kepler_state, nano2sec, orbit_period_s, sec2nano};
use recording::{History, sampling_time, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_variable_time_step_integrators [--integrator rk4|rkf45|rkf78]
    [--rel-tol R] [--abs-tol A] [--step-s S]
scenario_variable_time_step_integrators --all     run every upstream test case";

/// Upstream's default task period, `macros.hour2nano(2.)`.
pub const TWO_HOURS_NANOS: u64 = 7_200_000_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum IntegratorCase {
    Rk4,
    Rkf45,
    Rkf78,
}

impl IntegratorCase {
    pub fn label(self) -> &'static str {
        match self {
            Self::Rk4 => "rk4",
            Self::Rkf45 => "rkf45",
            Self::Rkf78 => "rkf78",
        }
    }
}

/// Upstream `run(show_plots, integratorCase, relTol, absTol,
/// simulationTimeStep)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub integrator: IntegratorCase,
    pub rel_tol: f64,
    pub abs_tol: f64,
    pub step_nanos: u64,
}

impl Config {
    /// The CSV suffix.
    pub fn label(&self) -> String {
        let tolerances = match (self.rel_tol, self.abs_tol) {
            (r, a) if r == 1.0e-4 && a == 1.0e-8 => "DefaultTol".to_string(),
            (r, a) if r == 1.0e-8 && a == 1.0e-12 => "TightTol".to_string(),
            (r, a) => format!("Rel{r:e}Abs{a:e}"),
        };
        let step = match self.step_nanos {
            TWO_HOURS_NANOS => String::new(),
            600_000_000_000 => "10min".to_string(),
            nanos => format!("{}s", nanos as f64 * 1.0e-9),
        };
        let tolerances = if self.integrator == IntegratorCase::Rk4 {
            String::new()
        } else {
            tolerances
        };
        format!("{}{tolerances}{step}", self.integrator.label())
    }

    fn integrator(&self) -> Option<Box<dyn Integrator<Spacecraft>>> {
        match self.integrator {
            IntegratorCase::Rk4 => None,
            IntegratorCase::Rkf45 => Some(Box::new(
                UpstreamAdaptive::rkf45().with_tolerances(self.rel_tol, self.abs_tol),
            )),
            IntegratorCase::Rkf78 => Some(Box::new(
                UpstreamAdaptive::rkf78().with_tolerances(self.rel_tol, self.abs_tol),
            )),
        }
    }
}

/// `test_scenarioIntegrators`' runs: tight tolerances, 2 h task.
pub const TEST_CASES: [Config; 2] = [
    Config {
        integrator: IntegratorCase::Rkf45,
        rel_tol: 1.0e-8,
        abs_tol: 1.0e-12,
        step_nanos: TWO_HOURS_NANOS,
    },
    Config {
        integrator: IntegratorCase::Rkf78,
        rel_tol: 1.0e-8,
        abs_tol: 1.0e-12,
        step_nanos: TWO_HOURS_NANOS,
    },
];

/// Upstream `runAccuracyComparison`'s `cases`: label and configuration.
pub const ACCURACY_CASES: [(&str, Config); 5] = [
    (
        "RKF45, default tolerances",
        Config {
            integrator: IntegratorCase::Rkf45,
            rel_tol: 1.0e-4,
            abs_tol: 1.0e-8,
            step_nanos: TWO_HOURS_NANOS,
        },
    ),
    ("RKF45, tightened tolerances", TEST_CASES[0]),
    (
        "RKF78, default tolerances",
        Config {
            integrator: IntegratorCase::Rkf78,
            rel_tol: 1.0e-4,
            abs_tol: 1.0e-8,
            step_nanos: TWO_HOURS_NANOS,
        },
    ),
    ("RKF78, tightened tolerances", TEST_CASES[1]),
    (
        "RKF78, default tolerances, 10 min task",
        Config {
            integrator: IntegratorCase::Rkf78,
            rel_tol: 1.0e-4,
            abs_tol: 1.0e-8,
            step_nanos: 600_000_000_000,
        },
    ),
];

pub struct Results {
    pub csv_path: std::path::PathBuf,
    /// Upstream `dataLog`: `times()`, `r_BN_N`, `v_BN_N`.
    pub states: Vec<(u64, SpacecraftStateMsg)>,
}

/// Upstream's orbit, before `rv2elem`.
fn orbit_elements() -> ClassicElements {
    let degrees = std::f64::consts::PI / 180.0;
    ClassicElements {
        semi_major_axis: 16.0e7,
        eccentricity: 0.8,
        inclination_rad: 33.3 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: -90.0 * degrees,
        ..Default::default()
    }
}

pub fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let (position_m, velocity_mps) = elements_to_state(mu, &orbit_elements());
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let duration_nanos = sec2nano(0.9 * orbit_period_s(mu, elements.semi_major_axis));
    let sample_nanos = sampling_time(duration_nanos, config.step_nanos, 100).max(config.step_nanos);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: config.step_nanos,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: config.integrator(),
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central gravity body");

    let mut csv = scenario_csv(&format!(
        "scenarioVariableTimeStepIntegrators{}",
        config.label()
    ));
    let csv_path = csv.config.output_path.clone();
    let mut history = History::<SpacecraftStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
        );
        schedule! { simulation,
            "spacecraft" => &mut spacecraft, period=config.step_nanos, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "history" => &mut history, period=sample_nanos, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        states: history.into_samples(),
    }
}

/// Upstream `getKeplerPosition` / `getAnalyticalSolution`: the two-body
/// position `dt` after `(r0, v0)`.
pub fn kepler_position(r0: Vector3<f64>, v0: Vector3<f64>, dt_s: f64) -> Vector3<f64> {
    let mu = Body::Earth.mu_m3ps2();
    kepler_state(mu, &state_to_elements(mu, r0, v0), dt_s).0
}

/// Position error (m) of every sample against the Kepler solution from the
/// first one.
pub fn kepler_errors(states: &[(u64, SpacecraftStateMsg)]) -> Vec<(u64, f64)> {
    let (t0, first) = &states[0];
    states
        .iter()
        .map(|(time, state)| {
            let elapsed = time - t0;
            let truth = kepler_position(first.position_m, first.velocity_mps, nano2sec(elapsed));
            (elapsed, (state.position_m - truth).norm())
        })
        .collect()
}

/// Upstream `runAccuracyComparison`: each case's error at the last sample
/// time all cases share, with that time (s), by case.
pub fn run_accuracy_comparison() -> (Vec<f64>, Vec<f64>, std::path::PathBuf) {
    let histories: Vec<Vec<(u64, f64)>> = ACCURACY_CASES
        .iter()
        .map(|(_, config)| kepler_errors(&run(*config).states))
        .collect();
    let common = histories[0]
        .iter()
        .map(|(time, _)| *time)
        .filter(|time| {
            histories
                .iter()
                .all(|history| history.iter().any(|(t, _)| t == time))
        })
        .max()
        .expect("a common sample time");
    let final_errors = histories
        .iter()
        .map(|history| history.iter().find(|(t, _)| *t == common).unwrap().1)
        .collect();
    let rows: Vec<Vec<f64>> = histories
        .iter()
        .enumerate()
        .flat_map(|(case, history)| {
            history
                .iter()
                .map(move |(time, error)| vec![case as f64, nano2sec(*time), *error])
        })
        .collect();
    let path = table_csv::write_table(
        "scenarioVariableTimeStepIntegratorsAccuracy",
        &[
            "case".to_string(),
            "time_s".to_string(),
            "position_error_m".to_string(),
        ],
        &rows,
    );
    (
        final_errors,
        vec![nano2sec(common); ACCURACY_CASES.len()],
        path,
    )
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    if args.flag("--all") {
        for config in TEST_CASES {
            let results = run(config);
            println!("wrote {}", results.csv_path.display());
        }
        let (errors, times, path) = run_accuracy_comparison();
        for ((label, _), (error, time)) in ACCURACY_CASES.iter().zip(errors.iter().zip(&times)) {
            println!("{label:>40}: {error:.3e} m at {:.0} h", time / 3600.0);
        }
        println!("wrote {}", path.display());
        return;
    }
    let integrator = match args.value("--integrator", "rkf78".to_string()).as_str() {
        "rk4" => IntegratorCase::Rk4,
        "rkf45" => IntegratorCase::Rkf45,
        "rkf78" => IntegratorCase::Rkf78,
        other => args.fail(&format!("unknown integrator {other:?}")),
    };
    let config = Config {
        integrator,
        rel_tol: args.value("--rel-tol", 1.0e-5),
        abs_tol: args.value("--abs-tol", 1.0e-8),
        step_nanos: sec2nano(args.value("--step-s", 7_200.0)),
    };
    let results = run(config);
    let errors = kepler_errors(&results.states);
    println!(
        "wrote {} (final Kepler error {:.3e} m)",
        results.csv_path.display(),
        errors.last().unwrap().1
    );
}

#[cfg(test)]
mod tests {
    use super::{
        IntegratorCase, TEST_CASES, kepler_position, nano2sec, run, run_accuracy_comparison,
    };

    /// `test_scenarioIntegrators`: the final position of the tight-tolerance
    /// runs against the Kepler solution from the first sample, per
    /// component, within 2000 m (RKF45) and 100 m (RKF78).
    #[test]
    fn tight_tolerance_runs_match_the_kepler_solution() {
        for config in TEST_CASES {
            let results = run(config);
            let (t0, first) = &results.states[0];
            let (time, last) = results.states.last().unwrap();
            let truth = kepler_position(first.position_m, first.velocity_mps, nano2sec(time - t0));
            let tolerance = match config.integrator {
                IntegratorCase::Rkf45 => 2000.0,
                _ => 100.0,
            };
            for axis in 0..3 {
                let error = (last.position_m[axis] - truth[axis]).abs();
                assert!(
                    error <= tolerance,
                    "{} axis {axis}: error {error} m",
                    config.label()
                );
            }
        }
    }

    /// `test_scenarioVariableTimeStepIntegratorsAccuracy`.
    #[test]
    fn accuracy_comparison_matches_upstream_bounds() {
        let (errors, times, _) = run_accuracy_comparison();
        for time in &times {
            assert!(
                (time - 156.0 * 3600.0).abs() < 1.0e-6,
                "evaluation at {time} s"
            );
        }
        let [default45, tight45, default78, tight78, default78_10min] = errors[..] else {
            unreachable!()
        };
        assert!(tight45 < 2000.0, "RKF45 tight: {tight45} m");
        assert!(tight78 < 100.0, "RKF78 tight: {tight78} m");
        assert!(
            default45 > 10.0 * tight45,
            "RKF45: {default45} vs {tight45}"
        );
        assert!(
            default78 > 10.0 * tight78,
            "RKF78: {default78} vs {tight78}"
        );
        assert!(default78_10min < 1.0, "RKF78 10 min: {default78_10min} m");
    }

    /// The table in upstream's scenario documentation, "Position error
    /// after 156 hours": 3,310 km, 1.6 km, 154 km, 31 m and 0.01 m, each to
    /// its printed precision.
    #[test]
    fn accuracy_comparison_matches_the_documented_table() {
        let (errors, _, _) = run_accuracy_comparison();
        for (error, (documented, half_unit)) in errors.iter().zip([
            (3_310e3, 5e3),
            (1.6e3, 0.05e3),
            (154e3, 0.5e3),
            (31.0, 0.5),
            (0.01, 0.005),
        ]) {
            assert!(
                (error - documented).abs() <= half_unit,
                "{error} m against the documented {documented} m"
            );
        }
    }
}
