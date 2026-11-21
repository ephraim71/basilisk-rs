//! A Monte Carlo campaign of OpNav pointing and orbit determination with
//! Hough circles: port of upstream
//! `examples/OpNavScenarios/scenariosOpNav/OpNavMC/MonteCarlo.py` with the
//! scenario it imports, `OpNavMC/scenario_OpNavAttODMC.py`.
//!
//! The scenario is `scenario_OpNavAttOD` for 100 minutes (3 of `prepOpNav`,
//! then `OpNavAttOD`): the OpNav spacecraft on the 18 000 km, `e = 0.6`
//! orbit about Mars, `houghCircles` (`noiseSF = 1`) and `pixelLineConverter`
//! feeding `opNavPoint` and `relativeOD` (`noiseSF = 5`, started 10 km and
//! 100 m/s off). `MonteCarlo.py` runs it twice on
//! [`basilisk_rs::monte_carlo::MonteCarlo`], dispersing `relativeOD.noiseSF`,
//! the camera's field of view and the orbit (see
//! [`opnav::monte_carlo`] for the dispersions, the retained messages and
//! how upstream's execution function resets the dispersed orbit, which
//! `--upstream-reset` reproduces).
//!
//! **Images.** Upstream streams the camera images from the Vizard
//! application. Here a synthetic renderer stands in for Vizard
//! ([`opnav::planet_renderer`]): a ray-traced, Sun-lit, textureless Mars
//! ellipsoid through the camera model, so only outcomes (the filter's
//! errors) compare with upstream.
//!
//! The campaign is archived in
//! `examples/output/montecarlo/scenario_OpNavAttODMC_MC_data/` (upstream's
//! `OpNavMC/MC_data`); upstream's data callback `displayPlots` becomes
//! `examples/output/scenarios/scenario_OpNavAttODMC_run<i>.csv` (the
//! filter's percent errors at the valid images), which
//! `plot_scenario_op_nav_att_od_mc.py` draws. `--single` runs the
//! scenario's own `__main__` instead (the nominal case) and writes its
//! retained messages to `scenario_OpNavAttODMC.csv`.
//!
//! Upstream has no test of the campaign (`test_bskTestOpNav.py` skips the
//! `OpNavMC` scripts); the tests here run it and check that no run fails,
//! that the dispersions differ, and that each run's filter converges.

#[path = "support/opnav/mod.rs"]
mod opnav;

use opnav::monte_carlo::{McScenario, main_for};

fn main() -> anyhow::Result<()> {
    main_for(McScenario::AttOdMc, "scenario_op_nav_att_od_mc")
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use basilisk_rs::monte_carlo::Archive;
    use opnav::monte_carlo::{
        CampaignOptions, McConfig, NUMBER_OF_RUNS, nav_percent_errors, run_campaign,
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
    fn campaign_runs_disperses_and_converges() {
        let dir = TempDir(
            std::env::temp_dir().join(format!("scenario_OpNavAttODMC-{}", std::process::id())),
        );
        let outcome = run_campaign(
            McScenario::AttOdMc,
            &dir.0,
            CampaignOptions {
                threads: Some(2),
                ..CampaignOptions::default()
            },
        )
        .expect("campaign runs");
        assert!(outcome.failures.is_empty(), "No runs should fail");
        assert_eq!(outcome.retained.len(), NUMBER_OF_RUNS);

        let archive = Archive::open(&dir.0).unwrap();
        let configs: Vec<McConfig> = (0..NUMBER_OF_RUNS)
            .map(|index| archive.parameters(index).unwrap())
            .collect();
        assert_ne!(configs[0].r_cn_n_init_m, configs[1].r_cn_n_init_m);
        assert_ne!(configs[0].nav_seed, configs[1].nav_seed);
        for config in &configs {
            assert!((1.0..=10.0).contains(&config.relative_od_noise_sf));
            assert!((config.field_of_view_rad.to_degrees() - 40.0).abs() <= 0.001);
            let radius = config.r_cn_n_init_m.norm();
            assert!(radius > 3_396e3, "starts above Mars: {radius}");
        }
        for (index, data) in &outcome.retained {
            let errors = nav_percent_errors(data);
            assert!(errors.len() >= 90, "run {index}: {} fixes", errors.len());
            // After the first hour every run's filter is within a percent.
            for row in errors
                .iter()
                .filter(|row| row.time_nanos > 3_600_000_000_000)
            {
                assert!(row.position < 1.0, "run {index}: {row:?}");
                assert!(row.velocity < 5.0, "run {index}: {row:?}");
            }
        }
    }
}
