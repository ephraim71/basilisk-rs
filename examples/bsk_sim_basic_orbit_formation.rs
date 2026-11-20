//! BskSim formation, both spacecraft pointing inertially: port of upstream
//! `examples/BskSim/scenarios/scenario_BasicOrbitFormation.py`.
//!
//! The formation models ([`bsk_sim::formation_dynamics`],
//! [`bsk_sim::formation_fsw`]): a chief on the 10 000 km, `e = 0.01` orbit
//! (`sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`)
//! and a deputy on a 12 000 km, `e = 0.1` orbit with the same orientation
//! (`sigma_BN = [-0.3, 0, 0.5]`, `omega_BN = [0.003, -0.02, 0.01] rad/s`),
//! about a point-mass Earth. The `inertial3D` mode points both at
//! `sigma_R0N = [0.2, 0.4, 0.6]` with their wheels. 10 minutes on the
//! default 0.1 s tasks.
//!
//! Writes `examples/output/scenarios/scenario_BasicOrbitFormation.csv` every
//! dynamics step: each spacecraft's navigation translation and guidance,
//! columns prefixed `chief_` and `deputy_`. `plot_bsk_sim_basic_orbit_formation.py`
//! draws upstream's `attitude_error_chief`, `rate_error_chief`,
//! `attitude_error_deputy`, `rate_error_deputy` and `orbits` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that both spacecraft reach the reference on their Keplerian orbits.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{AttitudeGuidanceMsg, NavigationTranslationMsg};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::formation_dynamics::FormationDynamics;
use bsk_sim::formation_fsw::{FormationFsw, FormationMode};
use bsk_sim::grav_factory::Body;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskSim, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, RECORDER_TASK2_PRIORITY, SIGMA_BN_INIT,
    dynamics, elements, guidance_columns, min2nano, scenario_recorder, translation_columns,
};

const STEM: &str = "scenario_BasicOrbitFormation";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    chief_translation: MessageLog<NavigationTranslationMsg>,
    deputy_translation: MessageLog<NavigationTranslationMsg>,
    chief_guidance: MessageLog<AttitudeGuidanceMsg>,
    deputy_guidance: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let chief = HubInit::from_elements(
        mu,
        &elements(10_000_000.0, 0.01, 33.3, 48.2, 347.8, 85.3),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    );
    let deputy = HubInit::from_elements(
        mu,
        &elements(12_000_000.0, 0.1, 33.3, 48.2, 347.8, 85.3),
        [-0.3, 0.0, 0.5],
        [0.003, -0.02, 0.01],
    );
    let mut base = BskSim::new(0.1, 0.1);
    let mut dynamics = FormationDynamics::new(&base, &chief, &deputy);
    let mut fsw = FormationFsw::new(&mut base, &mut dynamics);
    let (mode, runner) = (fsw.mode_request(), base.runner());

    // `log_outputs`: every dynamics step, each in its spacecraft's task.
    let sampling = dynamics.task_period_nanos;
    let mut chief_translation = MessageLog::<NavigationTranslationMsg>::default();
    let mut deputy_translation = MessageLog::<NavigationTranslationMsg>::default();
    let mut chief_guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut deputy_guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let chief_nav = &dynamics.chief.simple_nav.translation_out_msg;
    let deputy_nav = &dynamics.deputy.simple_nav.translation_out_msg;
    chief_nav.connect_to(&mut chief_translation.input);
    deputy_nav.connect_to(&mut deputy_translation.input);
    fsw.gateway.att_guid.connect_to(&mut chief_guidance.input);
    fsw.gateway.att_guid2.connect_to(&mut deputy_guidance.input);
    chief_nav
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("chief")));
    deputy_nav
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("deputy")));
    fsw.gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("chief")));
    fsw.gateway
        .att_guid2
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("deputy")));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    base.schedule(&mut simulation);
    dynamics.schedule(&mut simulation);
    fsw.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "sNavTransRec" => &mut chief_translation, period = sampling, priority = RECORDER_PRIORITY;
        "attErrRec" => &mut chief_guidance, period = sampling, priority = RECORDER_PRIORITY;
        "sNavTrans2Rec" => &mut deputy_translation, period = sampling, priority = RECORDER_TASK2_PRIORITY;
        "attErr2Rec" => &mut deputy_guidance, period = sampling, priority = RECORDER_TASK2_PRIORITY;
    }
    if config.write_csv {
        // After both tasks, so every column is sampled where upstream's is.
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_TASK2_PRIORITY);
    }

    mode.set(FormationMode::Inertial3D);
    runner.execute(&mut simulation, min2nano(10.0));
    drop(simulation);
    drop(recorder);

    Results {
        chief_translation,
        deputy_translation,
        chief_guidance,
        deputy_guidance,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_basic_orbit_formation");
    cli.finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final |sigma_BR|: chief {:.3e}, deputy {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.chief_guidance.last().sigma_br.norm(),
        results.deputy_guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    /// Both spacecraft reach `sigma_R0N` with their own wheels.
    #[test]
    fn both_spacecraft_reach_the_inertial_reference() {
        let results = run(Config { write_csv: false });
        for guidance in [&results.chief_guidance, &results.deputy_guidance] {
            assert!(guidance.values[1].sigma_br.norm() > 0.3);
            let last = guidance.last();
            assert!(last.sigma_br.norm() < 2e-3, "{}", last.sigma_br.norm());
            assert!(last.omega_br_b_radps.norm() < 2e-4);
        }
    }

    /// Each keeps its own two-body orbit.
    #[test]
    fn the_orbits_are_keplerian() {
        let results = run(Config { write_csv: false });
        let mu = Body::Earth.mu_m3ps2();
        for (translation, a, e) in [
            (&results.chief_translation, 10_000_000.0, 0.01),
            (&results.deputy_translation, 12_000_000.0, 0.1),
        ] {
            let last = translation.last();
            let elements = state_to_elements(mu, last.position_bn_n_m, last.velocity_bn_n_mps);
            assert!((elements.semi_major_axis - a).abs() < 1e-3 * a / 1e3);
            assert!((elements.eccentricity - e).abs() < 1e-9);
        }
    }
}
