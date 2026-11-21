//! A Monte Carlo campaign of the BskSim attitude-feedback scenario: port of
//! upstream `examples/MonteCarloExamples/scenarioBskSimAttFeedbackMC.py`.
//!
//! The simulation is upstream BskSim's `scenario_AttFeedback`
//! ([`bsk_att_feedback`]: inertial pointing on four reaction wheels for 10
//! minutes; built from the existing helpers and marked for consolidation
//! with the BskSim framework). A [`basilisk_rs::monte_carlo::MonteCarlo`]
//! campaign runs it 4 times with upstream's dispersions, each bound to a
//! field of the run's [`AttFeedbackConfig`]:
//!
//! | upstream path | dispersion |
//! |---|---|
//! | `TaskList[0].TaskModels[0].hub.sigma_BNInit` | `UniformEulerAngleMRPDispersion` (3-2-3 angles uniform on `[0, 2π]`) |
//! | `TaskList[0].TaskModels[0].hub.omega_BN_BInit` | `NormalVectorCartDispersion`, mean 0, σ = 0.75°/3 /s |
//! | `TaskList[0].TaskModels[0].hub.mHub` | `UniformDispersion` on 750 kg ± 5 % |
//! | `TaskList[0].TaskModels[0].hub.r_BcB_B` | `NormalVectorCartDispersion`, mean `[0, 0, 1]`, σ = `[0.05, 0.05, 0.1] / 3` m |
//!
//! (Upstream also names `hubref.IHubPntBc_B` in its `dispList` but never
//! adds its dispersion; neither does this port.)
//!
//! Each run retains upstream's `RetentionPolicy` message logs,
//! `sNavTransMsg.r_BN_N`, `attGuidMsg.sigma_BR` and `attGuidMsg.omega_BR_B`
//! at the recorders' 0.1 s, downcast to `f32` as `setVarCast('float')`
//! ([`RetainedMessages`]). The campaign is archived in
//! `examples/output/montecarlo/scenarioBskSimAttFeedbackMC/`, which
//! [`scenario_rerun_monte_carlo`](scenario_rerun_monte_carlo.rs) and
//! [`scenario_visualize_monte_carlo`](scenario_visualize_monte_carlo.rs)
//! read. As `setDispMagnitudeFile(True)`, `run<i>mag.txt` holds each
//! dispersion's magnitude (in % of the half range or in σ, upstream's
//! formulas). Upstream's data callback `displayPlots` plots each run's
//! `sigma_BR`; here it writes
//! `examples/output/scenarios/scenarioBskSimAttFeedbackMC_run<i>.csv`, which
//! `plot_scenario_bsk_sim_att_feedback_mc.py` overlays.
//!
//! Upstream `run(show_plots)` has no simulation parameters; `--all` runs
//! the campaign, as `test_bskMcTestScript.py` does, and `--threads n` sets
//! upstream's (commented-out) `setThreadCount`.

#[path = "support/bsk_att_feedback/mod.rs"]
mod bsk_att_feedback;
#[path = "support/cli.rs"]
mod cli;

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::bail;
use basilisk_rs::monte_carlo::{Archive, CampaignOutcome, RunInfo};
use bsk_att_feedback::table_csv::write_table;
use bsk_att_feedback::{
    AttFeedbackConfig, HUB_MASS_BOUNDS_KG, MC_NAME, NUMBER_OF_RUNS, RETAINED_FIELDS,
    RetainedMessages, archive_dir, campaign, com_offset_mean_and_std_m, omega_std_radps,
    run_retaining,
};
use cli::Cli;

/// The campaign's run function.
fn run_case(config: &AttFeedbackConfig, _info: &RunInfo) -> anyhow::Result<RetainedMessages> {
    Ok(run_retaining(config, &RETAINED_FIELDS))
}

/// Python's `str(round(value, 2))`.
fn round2(value: f64) -> String {
    let rounded = (value * 100.0).round() / 100.0;
    if rounded.fract() == 0.0 {
        format!("{rounded:.1}")
    } else {
        format!("{rounded}")
    }
}

/// Upstream's `run<i>mag.txt` of `setDispMagnitudeFile(True)`: each
/// dispersion's `generateMagString()`, sorted by name.
fn dispersion_magnitudes(config: &AttFeedbackConfig) -> String {
    let pi = std::f64::consts::PI;
    let vector = |values: [String; 3]| format!("[{}]", values.join(","));
    let (com_mean, com_std) = com_offset_mean_and_std_m();
    let (low, high) = HUB_MASS_BOUNDS_KG;
    let mid = (low + high) / 2.0;
    let entries = [
        (
            "TaskList[0].TaskModels[0].hub.mHub",
            format!(
                "['{} %']",
                round2((config.hub_mass_kg - mid) / (high - mid) * 100.0)
            ),
        ),
        (
            "TaskList[0].TaskModels[0].hub.omega_BN_BInit",
            vector(std::array::from_fn(|axis| {
                format!(
                    "{} sigma",
                    round2(config.omega_bn_b_init_radps[axis] / omega_std_radps())
                )
            })),
        ),
        (
            "TaskList[0].TaskModels[0].hub.r_BcB_B",
            vector(std::array::from_fn(|axis| {
                format!(
                    "{} sigma",
                    round2((config.hub_com_offset_m[axis] - com_mean[axis]) / com_std[axis])
                )
            })),
        ),
        (
            "TaskList[0].TaskModels[0].hub.sigma_BNInit",
            vector(std::array::from_fn(|axis| {
                format!(
                    "{} %",
                    round2((config.sigma_bn_init[axis] - pi) / pi * 100.0)
                )
            })),
        ),
    ];
    let mut text = String::new();
    for (name, magnitude) in entries {
        let _ = writeln!(text, "'{name}':'{magnitude}', ");
    }
    text
}

/// Upstream's data callback `displayPlots`: a run's retained data as a
/// scenario CSV.
fn write_run_csv(index: usize, data: &RetainedMessages) -> PathBuf {
    let mut headers = vec!["time_ns".to_string()];
    for (field, prefix) in [
        ("attGuidMsg.sigma_BR", "sigma_BR"),
        ("attGuidMsg.omega_BR_B", "omega_BR_B_rad_per_s"),
        ("sNavTransMsg.r_BN_N", "r_BN_N_m"),
    ] {
        if data.messages.contains_key(field) {
            headers.extend((0..3).map(|axis| format!("{prefix}_{axis}")));
        }
    }
    let rows: Vec<Vec<f64>> = (0..data.times_nanos.len())
        .map(|sample| {
            let mut row = vec![data.times_nanos[sample] as f64];
            for field in [
                "attGuidMsg.sigma_BR",
                "attGuidMsg.omega_BR_B",
                "sNavTransMsg.r_BN_N",
            ] {
                if let Some(values) = data.messages.get(field) {
                    row.extend(values[sample].iter().map(|value| f64::from(*value)));
                }
            }
            row
        })
        .collect();
    write_table(&format!("{MC_NAME}_run{index}"), &headers, &rows)
}

/// Upstream `run(show_plots)`: executes the campaign into `archive`, writes
/// the dispersion magnitudes, and (as `executeCallbacks`) each run's CSV.
fn run(
    archive: &Path,
    threads: Option<usize>,
    write_csv: bool,
) -> anyhow::Result<CampaignOutcome<RetainedMessages>> {
    let mut controller = campaign()
        .archive_dir(archive)
        .show_progress(threads.is_none());
    if let Some(threads) = threads {
        controller = controller.threads(threads);
    }
    let outcome = controller.execute(run_case)?;
    for index in 0..NUMBER_OF_RUNS {
        std::fs::write(
            archive.join(format!("run{index}mag.txt")),
            dispersion_magnitudes(&controller.run_configuration(index)),
        )?;
    }
    if write_csv {
        Archive::open(archive)?.for_each_retained(
            outcome.retained.keys().copied(),
            |index, data| {
                write_run_csv(index, &data);
            },
        )?;
    }
    Ok(outcome)
}

fn main() -> anyhow::Result<()> {
    let mut cli = Cli::from_env(
        "scenario_bsk_sim_att_feedback_mc [--threads n] [--all]   (--all: the campaign, as upstream's test)",
    );
    let _all = cli.switch("all");
    let threads: usize = cli.value("threads", 0);
    cli.finish();

    let archive = archive_dir(MC_NAME);
    let outcome = run(&archive, (threads > 0).then_some(threads), true)?;
    if !outcome.failures.is_empty() {
        bail!("failed runs: {:?}", outcome.failures);
    }
    println!(
        "{NUMBER_OF_RUNS} runs archived in {}; wrote examples/output/scenarios/{MC_NAME}_run<i>.csv",
        archive.display()
    );
    for (index, data) in &outcome.retained {
        let sigma = data.field("attGuidMsg.sigma_BR").last().expect("samples");
        let norm = sigma.iter().map(|value| value * value).sum::<f32>().sqrt();
        println!("  run {index}: final |sigma_BR| = {norm:.3e}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    //! Upstream `test_bskMcTestScript.py` runs the campaign and checks that
    //! it completes; these tests also check the archive, the dispersions
    //! and that every dispersed run reaches the inertial attitude.

    use super::*;
    use bsk_att_feedback::DURATION_NANOS;

    /// A temporary archive directory, removed when dropped.
    struct TempDir(PathBuf);

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn campaign_runs_archives_and_converges() {
        let dir = TempDir(std::env::temp_dir().join(format!("{MC_NAME}-{}", std::process::id())));
        let outcome = run(&dir.0, Some(2), false).expect("campaign runs");
        assert!(outcome.failures.is_empty());
        assert_eq!(outcome.retained.len(), NUMBER_OF_RUNS);

        let archive = Archive::open(&dir.0).unwrap();
        let configs: Vec<AttFeedbackConfig> = (0..NUMBER_OF_RUNS)
            .map(|index| archive.parameters(index).unwrap())
            .collect();
        for pair in configs.windows(2) {
            assert_ne!(pair[0].sigma_bn_init, pair[1].sigma_bn_init);
            assert_ne!(pair[0].omega_bn_b_init_radps, pair[1].omega_bn_b_init_radps);
            assert_ne!(pair[0].hub_mass_kg, pair[1].hub_mass_kg);
            assert_ne!(pair[0].hub_com_offset_m, pair[1].hub_com_offset_m);
        }
        for (index, config) in configs.iter().enumerate() {
            assert!((HUB_MASS_BOUNDS_KG.0..=HUB_MASS_BOUNDS_KG.1).contains(&config.hub_mass_kg));
            // The inertia is not dispersed.
            assert_eq!(
                config.hub_inertia_kg_m2,
                bsk_att_feedback::hub_inertia_kg_m2()
            );
            let magnitudes = std::fs::read_to_string(dir.0.join(format!("run{index}mag.txt")))
                .expect("dispersion magnitude file");
            assert_eq!(magnitudes.lines().count(), 4);

            let data: RetainedMessages = archive.retained_data(index).unwrap();
            assert_eq!(data, outcome.retained[&index]);
            let samples = (DURATION_NANOS / bsk_att_feedback::STEP_NANOS + 1) as usize;
            assert_eq!(data.times_nanos.len(), samples);
            for field in RETAINED_FIELDS {
                assert_eq!(data.field(field).len(), samples);
            }
            // Every dispersed run reaches the inertial attitude.
            let last = data.field("attGuidMsg.sigma_BR").last().unwrap();
            let norm = last.iter().map(|value| value * value).sum::<f32>().sqrt();
            assert!(norm < 1e-2, "run {index}: final |sigma_BR| = {norm:e}");
        }
        assert_ne!(
            outcome.retained[&0].field("attGuidMsg.sigma_BR"),
            outcome.retained[&1].field("attGuidMsg.sigma_BR")
        );
    }

    #[test]
    fn magnitudes_follow_upstream_formulas() {
        let mut config = AttFeedbackConfig::nominal();
        config.hub_mass_kg = 750.0 + 0.5 * 37.5;
        config.hub_com_offset_m = nalgebra::Vector3::new(0.05 / 3.0, 0.0, 1.0);
        let text = dispersion_magnitudes(&config);
        assert!(text.contains("'TaskList[0].TaskModels[0].hub.mHub':'['50.0 %']', "));
        assert!(text.contains(
            "'TaskList[0].TaskModels[0].hub.r_BcB_B':'[1.0 sigma,0.0 sigma,0.0 sigma]', "
        ));
    }
}
