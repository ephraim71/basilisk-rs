//! Rerun cases of an archived Monte Carlo campaign from their saved initial
//! conditions: port of upstream
//! `examples/MonteCarloExamples/scenarioRerunMonteCarlo.py`.
//!
//! Upstream's script reruns run 1 of `scenarioBskSimAttFeedbackMC` (the
//! BskSim attitude-feedback scenario, here [`bsk_att_feedback`]) from its
//! archived parameters, on 3 processes, without dispersing seeds, retaining
//! `attGuidMsg.sigma_BR`, into `<archive>/rerun`. Here:
//!
//! | upstream | here |
//! |---|---|
//! | `setICDir(icName)`, `setICRunFlag(True)`, `runInitialConditions(runsList)` | [`Archive::initial_conditions`] and [`MonteCarlo::run_configurations`] |
//! | `numProcess = 3` | [`MonteCarlo::threads`] |
//! | `setArchiveDir(icName + "/rerun")` | [`MonteCarlo::archive_dir`] |
//! | `addMessageLog("attGuidMsg", ["sigma_BR"])` | [`RetainedMessages`] of `attGuidMsg.sigma_BR` |
//! | `assert len(failed) == 0` | an error when a run fails |
//!
//! It needs the archive of
//! [`scenario_bsk_sim_att_feedback_mc`](scenario_bsk_sim_att_feedback_mc.rs)
//! (`examples/output/montecarlo/scenarioBskSimAttFeedbackMC/`): run that
//! first. `--runs 1,3` reruns other cases; `--all` reruns upstream's
//! `runsList = [1]`. Upstream's `run(time)` sets a `simBaseTime` attribute
//! that `scenario_AttFeedback` never reads; it has no counterpart. Unlike
//! upstream (`shouldArchiveParameters = False`), the rerun archive also
//! holds each run's parameters.

#[path = "support/bsk_att_feedback/mod.rs"]
mod bsk_att_feedback;
#[path = "support/cli.rs"]
mod cli;

use std::path::Path;

use anyhow::{Context, bail};
use basilisk_rs::monte_carlo::{Archive, CampaignOutcome, MonteCarlo};
use bsk_att_feedback::{AttFeedbackConfig, MC_NAME, RetainedMessages, archive_dir, run_retaining};
use cli::Cli;

/// Upstream `runsList`.
const RUNS: [usize; 1] = [1];
/// Upstream `numProcess`.
const THREADS: usize = 3;
/// Upstream's added retention policy.
const RETAINED_FIELDS: [&str; 1] = ["attGuidMsg.sigma_BR"];

/// Upstream `run()`: reruns `runs` of the campaign archived in `ic_dir`
/// into `ic_dir/rerun`.
fn run(ic_dir: &Path, runs: &[usize]) -> anyhow::Result<CampaignOutcome<RetainedMessages>> {
    let initial_conditions = Archive::open(ic_dir)
        .with_context(|| {
            format!("run scenario_bsk_sim_att_feedback_mc first to archive {MC_NAME}")
        })?
        .initial_conditions::<AttFeedbackConfig>(runs.iter().copied())?;
    let controller = MonteCarlo::new(AttFeedbackConfig::nominal())
        .execution_count(runs.len())
        .threads(THREADS)
        .disperse_seeds(false)
        .archive_dir(ic_dir.join("rerun"));
    let outcome = controller.run_configurations(initial_conditions, |config, _info| {
        Ok(run_retaining(config, &RETAINED_FIELDS))
    })?;
    if !outcome.failures.is_empty() {
        bail!("Should run ICs successfully: {:?}", outcome.failures);
    }
    Ok(outcome)
}

fn main() -> anyhow::Result<()> {
    let mut cli = Cli::from_env("scenario_rerun_monte_carlo [--runs 1[,3,...]] [--all]");
    let _all = cli.switch("all");
    let runs: String = cli.value("runs", format!("{}", RUNS[0]));
    cli.finish();
    let runs: Vec<usize> = runs
        .split(',')
        .map(|run| run.trim().parse().context("--runs takes run numbers"))
        .collect::<anyhow::Result<_>>()?;

    let ic_dir = archive_dir(MC_NAME);
    let outcome = run(&ic_dir, &runs)?;
    println!(
        "reran runs {runs:?} of {} into {}",
        ic_dir.display(),
        ic_dir.join("rerun").display()
    );
    let original = Archive::open(&ic_dir)?;
    for (index, data) in &outcome.retained {
        let archived: RetainedMessages = original.retained_data(*index)?;
        let same = archived.field("attGuidMsg.sigma_BR") == data.field("attGuidMsg.sigma_BR");
        println!("  run {index}: sigma_BR history reproduced: {same}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Upstream has no test of this script (it needs the campaign's data);
    //! this test archives the campaign, reruns upstream's run 1 and checks
    //! that the rerun reproduces the archived history.

    use std::path::PathBuf;

    use super::*;
    use basilisk_rs::monte_carlo::RunInfo;
    use bsk_att_feedback::{RETAINED_FIELDS as CAMPAIGN_FIELDS, campaign};

    struct TempDir(PathBuf);

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn rerun_reproduces_the_archived_run() {
        let dir = TempDir(
            std::env::temp_dir().join(format!("scenarioRerunMonteCarlo-{}", std::process::id())),
        );
        let campaign_outcome = campaign()
            .archive_dir(&dir.0)
            .threads(2)
            .show_progress(false)
            .execute(|config: &AttFeedbackConfig, _info: &RunInfo| {
                Ok(run_retaining(config, &CAMPAIGN_FIELDS))
            })
            .unwrap();
        assert!(campaign_outcome.failures.is_empty());

        let outcome = run(&dir.0, &RUNS).expect("the rerun succeeds");
        assert_eq!(outcome.retained.keys().copied().collect::<Vec<_>>(), RUNS);
        let rerun = &outcome.retained[&1];
        assert_eq!(rerun.messages.len(), 1);
        assert_eq!(
            rerun.field("attGuidMsg.sigma_BR"),
            campaign_outcome.retained[&1].field("attGuidMsg.sigma_BR")
        );
        // The rerun is archived next to the campaign, which it leaves alone.
        let archived: RetainedMessages = Archive::open(dir.0.join("rerun"))
            .unwrap()
            .retained_data(1)
            .unwrap();
        assert_eq!(&archived, rerun);
        assert!(dir.0.join("run0.data").exists());
    }
}
