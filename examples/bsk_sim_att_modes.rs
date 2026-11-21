//! BskSim attitude mode switching: port of upstream
//! `examples/BskSim/scenarios/scenario_AttModes.py`.
//!
//! The spacecraft of `scenario_AttFeedback` (10 000 km, `e = 0.01`,
//! `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`)
//! for 30 minutes, the mode requested again every 10 minutes from
//! `['hillPoint', 'inertial3D']` in turn: Hill pointing, inertial pointing,
//! Hill pointing. Each request fires its `initiate*` event at the start of
//! the run segment, which disables the tasks, zeroes the gateway and
//! enables the new mode's.
//!
//! Writes `examples/output/scenarios/scenario_AttModes.csv` every FSW step:
//! the guidance and the navigation translation. `plot_bsk_sim_att_modes.py`
//! draws upstream's `attitudeErrorNorm` and `rateError` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that each mode takes over at its switch and converges.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{AttitudeGuidanceMsg, AttitudeReferenceMsg, NavigationTranslationMsg};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, guidance_columns, min2nano,
    scenario_recorder, standard_orbit, translation_columns,
};

const STEM: &str = "scenario_AttModes";
/// Upstream `attitudeModeTime` \[ns\].
const MODE_TIME_NANOS: u64 = 600_000_000_000;
/// Upstream `attitudeMode`.
const MODES: [FswMode; 2] = [FswMode::HillPoint, FswMode::Inertial3D];

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    reference: MessageLog<AttitudeReferenceMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut models = BskModels::new(
        0.1,
        0.1,
        &standard_orbit(0.01, SIGMA_BN_INIT, OMEGA_BN_B_INIT),
    );
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    let sampling = models.fsw.task_period_nanos;
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut reference = MessageLog::<AttitudeReferenceMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let gateway = &models.fsw.gateway;
    gateway.att_guid.connect_to(&mut guidance.input);
    gateway.att_ref.connect_to(&mut reference.input);
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    models
        .dynamics
        .simple_nav
        .translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("")));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "attGuidMsg" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
        "attRefMsg" => &mut reference, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    // `runScenario`.
    let simulation_time = min2nano(30.0);
    let mut current = 0;
    while current < simulation_time {
        mode.set(MODES[(current / MODE_TIME_NANOS) as usize % MODES.len()]);
        current += MODE_TIME_NANOS;
        runner.execute(&mut simulation, current);
    }
    drop(simulation);
    drop(recorder);

    Results {
        guidance,
        reference,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_att_modes");
    cli.finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Each 10-minute segment converges to its mode's reference; the
    /// inertial one is `sigma_R0N = [0.2, 0.4, 0.6]`, at rest.
    #[test]
    fn each_mode_converges_before_the_next_switch() {
        let results = run(Config { write_csv: false });
        for segment in 1..=3u64 {
            let end = segment * MODE_TIME_NANOS;
            let error = results.guidance.at(end).sigma_br.norm();
            assert!(error < 2e-3, "segment {segment}: {error}");
        }
        let inertial = results.reference.at(2 * MODE_TIME_NANOS);
        assert!((inertial.sigma_rn - nalgebra::Vector3::new(0.2, 0.4, 0.6)).norm() < 1e-12);
        assert!(
            results
                .reference
                .at(MODE_TIME_NANOS)
                .omega_rn_n_radps
                .norm()
                > 0.0
        );
    }

    /// The switch at 10 minutes zeroes the gateway right after the last Hill
    /// step (the recorder after the switch sees the zeroed message), and the
    /// inertial reference then sets a new, large error.
    #[test]
    fn a_switch_zeroes_the_gateway_and_sets_a_new_error() {
        let results = run(Config { write_csv: false });
        let step = 100_000_000;
        assert_eq!(
            results.guidance.at(MODE_TIME_NANOS + step).sigma_br,
            nalgebra::Vector3::zeros()
        );
        let after = results
            .guidance
            .at(MODE_TIME_NANOS + 2 * step)
            .sigma_br
            .norm();
        assert!(after > 0.1, "{after}");
    }
}
