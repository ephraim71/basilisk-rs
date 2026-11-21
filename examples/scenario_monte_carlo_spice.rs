//! SPICE queries inside the runs of a Monte Carlo campaign: port of upstream
//! `examples/scenarioMonteCarloSpice.py`.
//!
//! Upstream shows that Python-side SPICE calls (`pyswice.furnsh_c`,
//! `spkRead`) must be made in each Monte Carlo worker, inside the simulation
//! class, rather than once in the controller. Its campaign runs 12 times on
//! 6 workers; each run builds a spacecraft (`r = [7000, 0, 0] km`,
//! `v = [0, 7.5, 0] km/s`, no gravity, one 10 s task), furnishes upstream's
//! kernels with the Hubble SPK, prints the Hubble state relative to the Sun
//! at 2012 April 29 15:18:14.907 UTC, and runs for 100 s.
//!
//! Here the campaign is [`basilisk_rs::monte_carlo::MonteCarlo`], whose runs
//! are threads of one process. CSPICE is one global state; basilisk-rs's
//! [`SpiceInterface`] serializes every call into it behind one lock and
//! furnishes a kernel once for all its holders, so each run can load the
//! kernels and query them as upstream's runs do: it builds its own
//! interface (`zeroBase = "Sun"`, the Hubble as its spacecraft, the start
//! epoch from upstream's time string) and reads its spacecraft state, the
//! counterpart of `spkRead`. Needs the `spice` feature, as the Hubble SPK
//! is of type 10, which ANISE does not read:
//!
//! ```text
//! cargo run --release --features spice --example scenario_monte_carlo_spice
//! ```
//!
//! Upstream `run()` has no parameters. Upstream archives the campaign in a
//! temporary directory and deletes it; here it is kept in
//! `examples/output/montecarlo/scenarioMonteCarloSpice/` (each run's
//! configuration and retained Hubble state). Upstream loads `naif0011.tls`;
//! this uses the repository's `naif0012.tls`, which differs only in the
//! leap seconds after 2012.

#[path = "support/cli.rs"]
mod cli;

use std::path::PathBuf;

use basilisk_rs::environment::spice_interface::{SpiceInterface, SpiceInterfaceConfig};
use basilisk_rs::monte_carlo::{CampaignOutcome, MonteCarlo, RunInfo};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::{Module, SimulationContext};
use cli::Cli;
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

/// Upstream `macros.sec2nano(10.0)`.
const STEP_NANOS: u64 = 10_000_000_000;
/// Upstream `sim.ConfigureStopTime(macros.sec2nano(100.0))`.
const DURATION_NANOS: u64 = 100_000_000_000;
/// Upstream `startCalendarTime`.
const START_CALENDAR_TIME: &str = "2012 APR 29 15:18:14.907 (UTC)";

/// A run's configuration: upstream `MySimulation` has nothing dispersed.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
struct Config {}

/// What each run retains: the Hubble state from SPICE and the spacecraft's
/// final state.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
struct Retained {
    /// Upstream `stateOut` of `spkRead(...)` [m, m/s].
    hubble_position_m: Vector3<f64>,
    hubble_velocity_mps: Vector3<f64>,
    final_position_m: Vector3<f64>,
}

/// Upstream `MySimulation.accessSpiceKernel`: `spkRead("HUBBLE SPACE
/// TELESCOPE", startCalendarTime, "j2000", "Sun")`, with the kernels
/// furnished by this run.
fn access_spice_kernel() -> (Vector3<f64>, Vector3<f64>) {
    let kernel_paths = [
        "assets/spice/naif0012.tls",
        "assets/spice/pck00010.tpc",
        "assets/spice/de-403-masses.tpc",
        "assets/spice/de430.bsp",
        "assets/spice/hst_edited.bsp",
    ]
    .into_iter()
    .map(PathBuf::from)
    .collect();
    let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
        name: "spkRead".to_string(),
        kernel_paths,
        reference_base: "j2000".to_string(),
        zero_base: "Sun".to_string(),
        utc_cal_init: Some(START_CALENDAR_TIME.to_string()),
        spacecraft_names: vec!["HUBBLE SPACE TELESCOPE".to_string()],
        ..SpiceInterfaceConfig::default()
    });
    spice.init();
    spice.update(&SimulationContext {
        current_sim_nanos: 0,
        // The interface takes its epoch from the time string.
        current_epoch: Epoch::from_gregorian_utc(2012, 4, 29, 15, 18, 14, 907_000_000),
    });
    let state = spice.sc_state_out_msgs[0].read();
    (state.position_m, state.velocity_mps)
}

/// Upstream `MySimulation()` followed by `executeScenario(sim)`.
fn run_case(_config: &Config, info: &RunInfo) -> anyhow::Result<Retained> {
    let (hubble_position_m, hubble_velocity_mps) = access_spice_kernel();
    println!(
        "run {}: Hubble relative to the Sun: r = {:?} m, v = {:?} m/s",
        info.index,
        hubble_position_m.as_slice(),
        hubble_velocity_mps.as_slice()
    );
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::new(7_000_000.0, 0.0, 0.0),
        initial_velocity_mps: Vector3::new(0.0, 7_500.0, 0.0),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    let state = spacecraft.state_out.clone();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2012, 4, 29), false);
        simulation.add_module("spacecraft", &mut spacecraft, STEP_NANOS, 1);
        simulation.run_for(DURATION_NANOS);
    }
    Ok(Retained {
        hubble_position_m,
        hubble_velocity_mps,
        final_position_m: state.read().position_m,
    })
}

/// Upstream `run()`: 12 runs on 6 threads with dispersed seeds.
fn run(archive: Option<PathBuf>) -> CampaignOutcome<Retained> {
    let mut campaign = MonteCarlo::new(Config::default())
        .execution_count(12)
        .threads(6)
        .disperse_seeds(true)
        .verbose(false);
    if let Some(archive) = archive {
        campaign = campaign.archive_dir(archive);
    }
    campaign.execute(run_case).expect("archive written")
}

fn main() {
    let mut cli = Cli::from_env("scenario_monte_carlo_spice [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();
    // The campaign empties its archive before it runs.
    let archive = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/output/montecarlo/scenarioMonteCarloSpice");
    let outcome = run(Some(archive.clone()));
    println!(
        "{} runs, {} failed; archived in {}",
        outcome.retained.len() + outcome.failures.len(),
        outcome.failures.len(),
        archive.display()
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioMonteCarloSpice.py` checks that the campaign
    //! runs without errors. Here also: no run fails, every run read the same
    //! SPICE state (the runs share CSPICE safely), and the force-free
    //! spacecraft moved in a straight line.

    use super::*;

    #[test]
    fn every_run_queries_spice_and_succeeds() {
        let outcome = run(None);
        assert!(outcome.failures.is_empty());
        assert_eq!(outcome.retained.len(), 12);
        let first = &outcome.retained[&0];
        let au_m = 1.495_978_707e11;
        let distance_au = first.hubble_position_m.norm() / au_m;
        assert!((0.98..1.02).contains(&distance_au), "{distance_au}");
        // The Earth's orbital speed about the Sun, about 30 km/s.
        let speed = first.hubble_velocity_mps.norm();
        assert!((25.0e3..36.0e3).contains(&speed), "{speed}");
        for retained in outcome.retained.values() {
            assert_eq!(retained, first);
            let expected = Vector3::new(7_000_000.0, 750_000.0, 0.0);
            assert!((retained.final_position_m - expected).norm() < 1e-6);
        }
    }
}
