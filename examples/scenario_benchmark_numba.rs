//! Benchmark of the attitude scenarios with library modules against modules
//! written in the scenario: port of upstream
//! `examples/scenarioBenchmarkNumba.py`.
//!
//! Upstream times two groups of scenarios, each run `nRuns` times with
//! their output suppressed, and prints the minimum, median and maximum time
//! of `InitializeSimulation` and of `ExecuteSimulation`:
//!
//! * pointing: `scenarioAttitudePointing` (all C++), its `...Py` variant (the
//!   controller a Python module) and its `...Numba` variant (the controller a
//!   Numba-compiled module);
//! * feedback: `scenarioAttitudeFeedback` (all C++) and its `...Numba`
//!   variant (navigation, guidance, tracking error and controller all
//!   Numba modules).
//!
//! The Rust counterparts are the ports of those scenarios, included here as
//! modules: the library modules stand for upstream's C++ ones, and the
//! modules the `...Py` and `...Numba` ports write in the scenario (plain
//! types implementing `Module`, compiled with the scenario) for the Python
//! and Numba ones. Each port's `run` builds, initializes and executes its
//! simulation in one call (the CSV off), so the benchmark times the whole
//! run: upstream's `init` and `exec` columns become one `run` column
//! (initialization in basilisk-rs is a small part of it). As upstream, the
//! minimum is the primary figure, and each group's times are relative to
//! its first entry.
//!
//! Upstream `run(case, nRuns)` maps to `--case pointing|feedback` and
//! `--runs N` (default 10); without `--case` both groups run, as upstream's
//! `__main__`. `--all` runs both groups once, as upstream's
//! `test_scenarioBenchmarkNumba.py` (`nRuns = 1`). Prints the tables; writes
//! no CSV (upstream plots nothing).
//!
//! Because the benchmarked ports are included as modules, their own tests
//! are compiled into this example's tests too and run with them, and the
//! support files they share are loaded once per port.

#![allow(clippy::duplicate_mod)]

#[path = "support/cli.rs"]
mod cli;
#[allow(dead_code)]
#[path = "scenario_attitude_feedback.rs"]
mod scenario_attitude_feedback;
#[allow(dead_code)]
#[path = "scenario_attitude_feedback_numba.rs"]
mod scenario_attitude_feedback_numba;
#[allow(dead_code)]
#[path = "scenario_attitude_pointing.rs"]
mod scenario_attitude_pointing;
#[allow(dead_code)]
#[path = "scenario_attitude_pointing_numba.rs"]
mod scenario_attitude_pointing_numba;
#[allow(dead_code)]
#[path = "scenario_attitude_pointing_py.rs"]
mod scenario_attitude_pointing_py;

use std::time::Instant;

use cli::Cli;

/// Upstream `TimingStats` [s].
#[derive(Clone, Copy, Debug, PartialEq)]
struct TimingStats {
    min: f64,
    median: f64,
    max: f64,
    stdev: f64,
}

/// Upstream `BenchmarkResult`, with one timing.
#[derive(Clone, Debug)]
struct BenchmarkResult {
    label: &'static str,
    run: TimingStats,
}

/// Upstream `Scenario`: a label and a function running the scenario once.
struct Scenario {
    label: &'static str,
    run: fn(),
}

/// Upstream `computeStats`.
fn compute_stats(samples: &[f64]) -> TimingStats {
    assert!(
        !samples.is_empty(),
        "Cannot compute statistics of an empty sample list."
    );
    let mut sorted = samples.to_vec();
    sorted.sort_by(f64::total_cmp);
    let count = sorted.len();
    let median = if count % 2 == 1 {
        sorted[count / 2]
    } else {
        0.5 * (sorted[count / 2 - 1] + sorted[count / 2])
    };
    let mean = sorted.iter().sum::<f64>() / count as f64;
    let stdev = if count > 1 {
        (sorted
            .iter()
            .map(|value| (value - mean).powi(2))
            .sum::<f64>()
            / (count - 1) as f64)
            .sqrt()
    } else {
        0.0
    };
    TimingStats {
        min: sorted[0],
        median,
        max: sorted[count - 1],
        stdev,
    }
}

/// Upstream `measureScenario`.
fn measure_scenario(scenario: &Scenario, n_runs: usize) -> BenchmarkResult {
    let samples: Vec<f64> = (0..n_runs)
        .map(|_| {
            let started = Instant::now();
            (scenario.run)();
            started.elapsed().as_secs_f64()
        })
        .collect();
    BenchmarkResult {
        label: scenario.label,
        run: compute_stats(&samples),
    }
}

/// Upstream `formatSeconds`.
fn format_seconds(seconds: f64) -> String {
    if seconds >= 0.1 {
        format!("{seconds:.3} s")
    } else if seconds >= 1e-3 {
        format!("{:.2} ms", seconds * 1e3)
    } else {
        format!("{:.1} us", seconds * 1e6)
    }
}

/// Upstream `printResultsTable`.
fn print_results_table(results: &[BenchmarkResult], title: &str) {
    const LABEL: usize = 28;
    let baseline = results[0].run.min;
    println!("{title}");
    println!(
        "  {:<LABEL$}  {:>10}  {:>10}  {:>10}  {:>12}",
        "Module", "run min", "run med", "run max", "run min rel"
    );
    println!(
        "  {}  {}  {}  {}  {}",
        "-".repeat(LABEL),
        "-".repeat(10),
        "-".repeat(10),
        "-".repeat(10),
        "-".repeat(12)
    );
    for result in results {
        let ratio = if baseline > 0.0 {
            result.run.min / baseline
        } else {
            f64::INFINITY
        };
        println!(
            "  {:<LABEL$}  {:>10}  {:>10}  {:>10}  {:>11.3}x",
            result.label,
            format_seconds(result.run.min),
            format_seconds(result.run.median),
            format_seconds(result.run.max),
            ratio
        );
    }
    println!();
}

/// Upstream `runGroup`.
fn run_group(title: &str, scenarios: &[Scenario], n_runs: usize) -> Vec<BenchmarkResult> {
    let results: Vec<BenchmarkResult> = scenarios
        .iter()
        .map(|scenario| measure_scenario(scenario, n_runs))
        .collect();
    print_results_table(&results, &format!("{title} ({n_runs} runs)"));
    results
}

/// Upstream's benchmark groups.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Case {
    Pointing,
    Feedback,
}

impl std::str::FromStr for Case {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        match text {
            "pointing" => Ok(Self::Pointing),
            "feedback" => Ok(Self::Feedback),
            _ => Err(format!("unknown case {text}")),
        }
    }
}

/// Upstream `run(case, nRuns)`.
fn run(case: Case, n_runs: usize) -> Vec<BenchmarkResult> {
    match case {
        Case::Pointing => run_group(
            "Pointing  - single control module replaced",
            &[
                Scenario {
                    label: "Library (all library)",
                    run: || {
                        let mut config = scenario_attitude_pointing::Config::new(false);
                        config.write_csv = false;
                        scenario_attitude_pointing::run(config);
                    },
                },
                Scenario {
                    label: "Py port (1 scenario module)",
                    run: || {
                        scenario_attitude_pointing_py::run(false);
                    },
                },
                Scenario {
                    label: "Numba port (1 scen. module)",
                    run: || {
                        scenario_attitude_pointing_numba::run(false);
                    },
                },
            ],
            n_runs,
        ),
        Case::Feedback => run_group(
            "Feedback  - all 4 FSW+sensor modules replaced",
            &[
                Scenario {
                    label: "Library (all library)",
                    run: || {
                        let mut config =
                            scenario_attitude_feedback::Config::new(false, false, false, false);
                        config.write_csv = false;
                        scenario_attitude_feedback::run(config);
                    },
                },
                Scenario {
                    label: "Numba port (4 scen. modules)",
                    run: || {
                        let mut config =
                            scenario_attitude_feedback_numba::Config::new(false, false, false);
                        config.write_csv = false;
                        scenario_attitude_feedback_numba::run(config);
                    },
                },
            ],
            n_runs,
        ),
    }
}

fn main() {
    let mut cli =
        Cli::from_env("scenario_benchmark_numba [--case pointing|feedback] [--runs N] [--all]");
    let all = cli.switch("all");
    let case: String = cli.value("case", String::new());
    let case: Option<Case> = (!case.is_empty()).then(|| {
        case.parse()
            .unwrap_or_else(|error| panic!("--case: {error}"))
    });
    let n_runs = if all { 1 } else { cli.value("runs", 10_usize) };
    cli.finish();
    let cases = match case {
        Some(case) if !all => vec![case],
        _ => vec![Case::Pointing, Case::Feedback],
    };
    for case in cases {
        run(case, n_runs);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `test_scenarioBenchmarkNumba.py`: both groups run once
    //! (`nRuns = 1`); here also their statistics are consistent.

    use super::*;

    #[test]
    fn test_scenario_benchmark_numba() {
        for (case, count) in [(Case::Pointing, 3), (Case::Feedback, 2)] {
            let results = run(case, 1);
            assert_eq!(results.len(), count);
            for result in results {
                let stats = result.run;
                assert!(stats.min > 0.0 && stats.min.is_finite(), "{}", result.label);
                assert_eq!(stats.min, stats.max);
                assert_eq!(stats.stdev, 0.0);
            }
        }
    }

    #[test]
    fn statistics_match_upstream_compute_stats() {
        let stats = compute_stats(&[3.0, 1.0, 2.0, 10.0]);
        assert_eq!(stats.min, 1.0);
        assert_eq!(stats.median, 2.5);
        assert_eq!(stats.max, 10.0);
        // Python's statistics.stdev: the sample standard deviation.
        assert!((stats.stdev - 4.08248290463863).abs() < 1e-12);
        assert_eq!(format_seconds(0.25), "0.250 s");
        assert_eq!(format_seconds(0.0025), "2.50 ms");
        assert_eq!(format_seconds(2.5e-6), "2.5 us");
    }
}
