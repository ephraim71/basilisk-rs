//! OpNav pointing and orbit determination from the limb, the Monte Carlo
//! variant: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/OpNavMC/scenario_LimbAttOD.py`
//! with `OpNavMC/MonteCarlo.py` importing it.
//!
//! The scenario is `scenario_OpNavAttODLimb` for 100 minutes (3 of
//! `prepOpNav`, then `OpNavAttODLimb`): the OpNav spacecraft on the 18 000 km,
//! `e = 0.6` orbit about Mars at `sigma_BN = [0, -0.3, 0]`, `limbFinding` and
//! `horizonOpNav` (`noiseSF = 20`) feeding `opNavPoint` and `relativeOD`
//! (started 10 km and 100 m/s off). By default it runs as `MonteCarlo.py`
//! runs a scenario module: twice on [`basilisk_rs::monte_carlo::MonteCarlo`],
//! dispersing `relativeOD.noiseSF`, the camera's field of view and the
//! orbit (see [`opnav::monte_carlo`] for the dispersions, the retained
//! messages, and how upstream's execution function resets the dispersed
//! orbit, which `--upstream-reset` reproduces). `--single` runs the
//! scenario's own `__main__`: the nominal case, with its `pull_outputs`
//! data.
//!
//! **Images.** Upstream streams the camera images from the Vizard
//! application. Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the filter's
//! errors) compare with upstream.
//!
//! The campaign is archived in
//! `examples/output/montecarlo/scenario_LimbAttOD_MC_data/`, and each run's
//! percent errors (upstream's `displayPlots`) written to
//! `examples/output/scenarios/scenario_LimbAttOD_run<i>.csv`. `--single`
//! writes `scenario_LimbAttOD.csv` (the retained messages) and
//! `scenario_LimbAttOD_limb.csv` (the limbs), which
//! `plot_scenario_limb_att_od.py` draws as upstream's `pull_outputs` does.
//!
//! Upstream has no test of this script (`test_bskTestOpNav.py` skips the
//! `OpNavMC` scripts); the tests here check that the nominal run converges
//! on the limb and that the campaign runs.

#[path = "support/opnav/mod.rs"]
mod opnav;

use opnav::monte_carlo::{McScenario, main_for};

fn main() -> anyhow::Result<()> {
    main_for(McScenario::LimbAttOd, "scenario_limb_att_od")
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use opnav::monte_carlo::{
        CampaignOptions, McConfig, NUMBER_OF_RUNS, nav_percent_errors, run_campaign, simulate,
    };

    use super::*;

    /// A temporary archive directory, removed when dropped.
    struct TempDir(PathBuf);

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    #[test]
    fn the_nominal_run_converges_on_the_limb() {
        let data = simulate(
            &McConfig::nominal(McScenario::LimbAttOd),
            opnav::monte_carlo::SIM_TIME_MIN,
        );
        // One limb per image of `OpNavAttODLimb`.
        assert!(data.limbs.len() >= 95, "{} limbs", data.limbs.len());
        let errors = nav_percent_errors(&data);
        // Each limb gives a fix, recorded the step after (the last one falls
        // past the end).
        assert!(
            errors.len().abs_diff(data.limbs.len()) <= 1,
            "{} fixes, {} limbs",
            errors.len(),
            data.limbs.len()
        );
        for row in errors
            .iter()
            .filter(|row| row.time_nanos > 3_600_000_000_000)
        {
            assert!(row.position < 2.0, "{row:?}");
            assert!(row.velocity < 5.0, "{row:?}");
        }
    }

    #[test]
    fn campaign_runs() {
        let dir = TempDir(
            std::env::temp_dir().join(format!("scenario_LimbAttOD-{}", std::process::id())),
        );
        let outcome = run_campaign(
            McScenario::LimbAttOd,
            &dir.0,
            CampaignOptions {
                threads: Some(2),
                sim_time_min: 30.0,
                ..CampaignOptions::default()
            },
        )
        .expect("campaign runs");
        assert!(outcome.failures.is_empty(), "No runs should fail");
        assert_eq!(outcome.retained.len(), NUMBER_OF_RUNS);
        for data in outcome.retained.values() {
            assert!(!data.limbs.is_empty());
        }
    }
}
