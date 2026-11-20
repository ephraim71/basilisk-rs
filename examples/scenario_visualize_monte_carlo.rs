//! Plot the retained data of an archived Monte Carlo campaign: port of
//! upstream `examples/MonteCarloExamples/scenarioVisualizeMonteCarlo.py`.
//!
//! Upstream's `MonteCarloPlotter` (`utilities/MonteCarlo/AnalysisBaseClass`)
//! loads `attGuidMsg.sigma_BR` and `attGuidMsg.omega_BR_B` of every run of
//! `scenarioBskSimAttFeedbackMC` and shows each component of each variable
//! with all runs in an interactive Bokeh page (static HTML, or a Bokeh
//! server with `--bokeh-server`), runs entered by number highlighted, and a
//! title with the number of runs and the data size.
//!
//! Here the loading is Rust and the drawing Python, as for every scenario:
//! this example reads the archive of
//! [`scenario_bsk_sim_att_feedback_mc`](scenario_bsk_sim_att_feedback_mc.rs)
//! (`examples/output/montecarlo/scenarioBskSimAttFeedbackMC/`; run that
//! first) and writes, per variable and component,
//! `examples/output/scenarios/MonteCarloPlots_<variable>_<component>.csv`
//! (upstream's figure names; components 1 to 3) with `time_ns` and a
//! `run_<i>` column per successful run. `plot_scenario_visualize_monte_carlo.py`
//! draws them with matplotlib, all runs coloured by run number, the runs of
//! `--runs 1,3` highlighted, and the title upstream shows. The Bokeh server
//! and the interactive selection have no counterpart.
//!
//! `--all` runs the one case of upstream's `test_bskMcTestScript.py`.

#[path = "support/bsk_att_feedback/mod.rs"]
mod bsk_att_feedback;
#[path = "support/cli.rs"]
mod cli;

use std::path::{Path, PathBuf};

use anyhow::Context;
use basilisk_rs::monte_carlo::Archive;
use bsk_att_feedback::table_csv::write_table;
use bsk_att_feedback::{MC_NAME, RetainedMessages, archive_dir};
use cli::Cli;

/// Upstream `plotter.load_data([...])`.
const VARIABLES: [&str; 2] = ["attGuidMsg.sigma_BR", "attGuidMsg.omega_BR_B"];

/// One component of one variable across the runs.
struct Series {
    variable: String,
    /// 1 to 3, as upstream's figure names.
    component: usize,
    times_nanos: Vec<u64>,
    /// `(run, values)` per successful run.
    runs: Vec<(usize, Vec<f64>)>,
}

/// What upstream's plotter loads and shows.
// Some fields are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Visualization {
    /// Upstream `num_runs`.
    num_runs: usize,
    /// Upstream `total_data_size` (here the archived `run<i>.data` files)
    /// \[bytes\].
    total_data_bytes: u64,
    series: Vec<Series>,
    csv_paths: Vec<PathBuf>,
}

/// Upstream `create_document`: loads `VARIABLES` of every successful run
/// archived in `data_dir` and writes one CSV per variable and component.
fn run(data_dir: &Path, write_csv: bool) -> anyhow::Result<Visualization> {
    let archive = Archive::open(data_dir).with_context(|| {
        format!("run scenario_bsk_sim_att_feedback_mc first to archive {MC_NAME}")
    })?;
    let count = archive.campaign::<serde_json::Value>()?.execution_count;
    let failures = archive.failures()?;
    let runs: Vec<usize> = (0..count).filter(|run| !failures.contains(run)).collect();
    let mut total_data_bytes = 0;
    let mut retained = Vec::new();
    for &run in &runs {
        total_data_bytes += std::fs::metadata(data_dir.join(format!("run{run}.data")))?.len();
        retained.push((run, archive.retained_data::<RetainedMessages>(run)?));
    }

    let mut series = Vec::new();
    for variable in VARIABLES {
        for component in 1..=3 {
            let times_nanos = retained
                .first()
                .map(|(_, data)| data.times_nanos.clone())
                .unwrap_or_default();
            let runs = retained
                .iter()
                .map(|(run, data)| {
                    anyhow::ensure!(
                        data.times_nanos == times_nanos,
                        "run {run} was sampled at other times"
                    );
                    let values = data
                        .messages
                        .get(variable)
                        .with_context(|| format!("run {run} did not retain {variable}"))?
                        .iter()
                        .map(|value| f64::from(value[component - 1]))
                        .collect();
                    Ok((*run, values))
                })
                .collect::<anyhow::Result<_>>()?;
            series.push(Series {
                variable: variable.to_string(),
                component,
                times_nanos,
                runs,
            });
        }
    }

    let csv_paths = if write_csv {
        series
            .iter()
            .map(|series| {
                let mut headers = vec!["time_ns".to_string()];
                headers.extend(series.runs.iter().map(|(run, _)| format!("run_{run}")));
                let rows: Vec<Vec<f64>> = series
                    .times_nanos
                    .iter()
                    .enumerate()
                    .map(|(sample, time)| {
                        let mut row = vec![*time as f64];
                        row.extend(series.runs.iter().map(|(_, values)| values[sample]));
                        row
                    })
                    .collect();
                write_table(
                    &format!("MonteCarloPlots_{}_{}", series.variable, series.component),
                    &headers,
                    &rows,
                )
            })
            .collect()
    } else {
        Vec::new()
    };
    Ok(Visualization {
        num_runs: runs.len(),
        total_data_bytes,
        series,
        csv_paths,
    })
}

fn main() -> anyhow::Result<()> {
    let mut cli = Cli::from_env("scenario_visualize_monte_carlo [--all]");
    let _all = cli.switch("all");
    cli.finish();

    let visualization = run(&archive_dir(MC_NAME), true)?;
    // Upstream's title.
    println!(
        "Monte Carlo Visualization: {} runs, {:.2} GB total ({:.1} MB)",
        visualization.num_runs,
        visualization.total_data_bytes as f64 / 1e9,
        visualization.total_data_bytes as f64 / 1e6
    );
    for path in &visualization.csv_paths {
        println!("wrote {}", path.display());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskMcTestScript.py`: the campaign of
    //! `scenarioBskSimAttFeedbackMC`, then this script on its data, both
    //! completing; here also the loaded series are checked.

    use super::*;
    use basilisk_rs::monte_carlo::RunInfo;
    use bsk_att_feedback::{
        AttFeedbackConfig, NUMBER_OF_RUNS, RETAINED_FIELDS, campaign, run_retaining,
    };

    struct TempDir(PathBuf);

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn campaign_then_visualization() {
        let dir = TempDir(std::env::temp_dir().join(format!(
            "scenarioVisualizeMonteCarlo-{}",
            std::process::id()
        )));
        let outcome = campaign()
            .archive_dir(&dir.0)
            .threads(2)
            .show_progress(false)
            .execute(|config: &AttFeedbackConfig, _info: &RunInfo| {
                Ok(run_retaining(config, &RETAINED_FIELDS))
            })
            .unwrap();
        assert!(outcome.failures.is_empty());

        let visualization = run(&dir.0, false).expect("the archive loads");
        assert_eq!(visualization.num_runs, NUMBER_OF_RUNS);
        assert!(visualization.total_data_bytes > 0);
        assert_eq!(visualization.series.len(), 6);
        for series in &visualization.series {
            assert_eq!(series.runs.len(), NUMBER_OF_RUNS);
            let data = &outcome.retained[&0];
            assert_eq!(series.times_nanos, data.times_nanos);
            let expected: Vec<f64> = data
                .field(&series.variable)
                .iter()
                .map(|value| f64::from(value[series.component - 1]))
                .collect();
            assert_eq!(series.runs[0].1, expected);
        }
    }
}
