//! BskSim basic orbit: port of upstream
//! `examples/BskSim/scenarios/scenario_BasicOrbit.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]: 750 kg hub, four wheels,
//! Sun, Earth and Moon gravity, sensors) on a 7000 km, `e = 0.1` Earth orbit
//! (`i = 33.3 deg`, `Omega = 48.2 deg`, `omega = 347.8 deg`, `f = 85.3 deg`),
//! starting at `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01,
//! 0.03] rad/s`. The flight software is in `standby` (no task runs), so the
//! spacecraft tumbles freely for 10 minutes; the 0.1 s dynamics and FSW
//! tasks are upstream's defaults.
//!
//! Writes `examples/output/scenarios/scenario_BasicOrbit.csv`: the
//! navigation attitude and translation at every dynamics step, upstream's
//! `sNavAttRec` and `sNavTransRec`. `plot_bsk_sim_basic_orbit.py` draws
//! upstream's `orbit` and `orientation` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests here
//! check what it shows: a Keplerian orbit and a torque-free tumble.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    NavigationAttitudeMsg, NavigationTranslationMsg, SpacecraftDiagnosticsMsg,
};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::dynamics::{self, BskDynamics};
use bsk_sim::fsw::{BskFsw, FswMode};
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskSim, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, attitude_columns, elements,
    min2nano, scenario_recorder, translation_columns,
};

const STEM: &str = "scenario_BasicOrbit";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    attitude: MessageLog<NavigationAttitudeMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut base = BskSim::new(0.1, 0.1);
    let mu = bsk_sim::grav_factory::Body::Earth.mu_m3ps2();
    let initial = HubInit::from_elements(
        mu,
        &elements(7_000_000.0, 0.1, 33.3, 48.2, 347.8, 85.3),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    );
    let mut dynamics = BskDynamics::new(&mut base, &initial);
    let mut fsw = BskFsw::new(&mut base, &mut dynamics);
    let (mode, runner) = (fsw.mode_request(), base.runner());

    // `log_outputs`: `recorder()` with no sampling time, every dynamics step.
    let mut attitude = MessageLog::<NavigationAttitudeMsg>::default();
    let mut translation = MessageLog::<NavigationTranslationMsg>::default();
    let mut diagnostics = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let nav = &dynamics.simple_nav;
    nav.attitude_out_msg.connect_to(&mut attitude.input);
    nav.translation_out_msg.connect_to(&mut translation.input);
    dynamics
        .spacecraft
        .diagnostics_out
        .connect_to(&mut diagnostics.input);
    nav.attitude_out_msg
        .connect_to(recorder.add_source::<NavigationAttitudeMsg>(attitude_columns("")));
    nav.translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("")));

    let period = base.dyn_rate_nanos;
    let mut simulation = Simulation::new(dynamics::epoch(), false);
    base.schedule(&mut simulation);
    dynamics.schedule(&mut simulation);
    fsw.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "sNavAttRec" => &mut attitude, period = period, priority = RECORDER_PRIORITY;
        "sNavTransRec" => &mut translation, period = period, priority = RECORDER_PRIORITY;
        "diagnostics" => &mut diagnostics, period = period, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, period, RECORDER_PRIORITY);
    }

    // `runScenario`.
    mode.set(FswMode::Standby);
    runner.execute(&mut simulation, min2nano(10.0));
    drop(simulation);
    drop(recorder);

    Results {
        attitude,
        translation,
        diagnostics,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_basic_orbit");
    cli.finish();
    let results = run(Config { write_csv: true });
    let last = results.translation.last();
    println!(
        "wrote {} (final |r_BN_N| = {:.1} km)",
        results.csv_path.expect("CSV written").display(),
        last.position_bn_n_m.norm() / 1e3,
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    /// Ten minutes at the dynamics rate, the first sample at the start.
    #[test]
    fn the_recorders_sample_every_dynamics_step() {
        let results = quiet();
        assert_eq!(results.translation.values.len(), 6001);
        assert_eq!(results.attitude.times_nanos[1], 100_000_000);
    }

    /// The orbit keeps its elements: only the Sun's and the Moon's tidal
    /// accelerations perturb it.
    #[test]
    fn the_orbit_is_keplerian() {
        let results = quiet();
        let mu = bsk_sim::grav_factory::Body::Earth.mu_m3ps2();
        let first = results.translation.values[1].clone();
        for state in results.translation.values.iter().skip(1) {
            let elements = state_to_elements(mu, state.position_bn_n_m, state.velocity_bn_n_mps);
            assert!(
                (elements.semi_major_axis - 7_000_000.0).abs() < 1.0,
                "{}",
                elements.semi_major_axis
            );
            assert!((elements.eccentricity - 0.1).abs() < 1e-6);
        }
        assert!((results.translation.last().position_bn_n_m - first.position_bn_n_m).norm() > 1e6);
    }

    /// With every FSW task off nothing torques the spacecraft: its rotational
    /// angular momentum and energy (hub and wheels) stay constant while it
    /// tumbles.
    #[test]
    fn the_spacecraft_tumbles_torque_free() {
        let results = quiet();
        let first = &results.diagnostics.values[1];
        let initial_momentum = first.rotational_angular_momentum_inertial_kg_m2ps;
        for sample in results.diagnostics.values.iter().skip(1) {
            let drift =
                (sample.rotational_angular_momentum_inertial_kg_m2ps - initial_momentum).norm();
            assert!(drift < 1e-9 * initial_momentum.norm().max(1.0), "{drift}");
            assert!((sample.rotational_energy_j - first.rotational_energy_j).abs() < 1e-9);
        }
        let sigma_first = results.attitude.values[0].sigma_bn;
        let sigma_last = results.attitude.last().sigma_bn;
        assert!((sigma_last - sigma_first).norm() > 0.1);
    }
}
