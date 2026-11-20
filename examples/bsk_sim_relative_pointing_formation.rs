//! BskSim formation, the deputy pointing at the chief: port of upstream
//! `examples/BskSim/scenarios/scenario_RelativePointingFormation.py`.
//!
//! The formation models ([`bsk_sim::formation_dynamics`],
//! [`bsk_sim::formation_fsw`]): the chief on an 8000 km, `e = 0.1` orbit
//! (`i = 33.3 deg`, `Omega = 48.2 deg`, `omega = 347.8 deg`, `f = 0`) and the
//! deputy 10 deg of true anomaly behind it, about a point-mass Earth. In the
//! `spacecraftPointing` mode `spacecraftPointing` turns the deputy's body
//! vector `[1, 2, 3]` towards the chief, `trackingError2` compares, and
//! `mrpFeedbackControl` (`K = 10`, `P = 30`, `Ki = 1e-4`) commands the
//! deputy's external torque; the chief's wheels are idle. 10 minutes on the
//! default 0.1 s tasks.
//!
//! Writes `examples/output/scenarios/scenario_RelativePointingFormation.csv`
//! every dynamics step: both navigation translations and attitudes, both
//! guidance messages, the deputy reference and its direct torque command,
//! columns prefixed `chief_` and `deputy_`. As upstream, the deputy attitude
//! is recorded in the chief's task, before the deputy's navigation updates:
//! the previous step's. `plot_bsk_sim_relative_pointing_formation.py` draws
//! upstream's `attitude_error`, `relative_orbit`, `sigma_RN`,
//! `sigma_BN_deputy` and `sigma_BR_deputy` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that the deputy's body vector ends up pointing at the chief.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg,
    NavigationTranslationMsg,
};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::formation_dynamics::FormationDynamics;
use bsk_sim::formation_fsw::{FormationFsw, FormationMode};
use bsk_sim::grav_factory::Body;
use bsk_sim::latch::Latch;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskSim, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, RECORDER_TASK2_PRIORITY, SIGMA_BN_INIT,
    attitude_columns, dynamics, elements, guidance_columns, min2nano, reference_columns,
    scenario_recorder, torque_columns, translation_columns,
};

const STEM: &str = "scenario_RelativePointingFormation";

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
    deputy_attitude: MessageLog<NavigationAttitudeMsg>,
    deputy_guidance: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    // Upstream `oe2 = oe; oe2.f -= 10 deg`.
    let chief = HubInit::from_elements(
        mu,
        &elements(8_000_000.0, 0.1, 33.3, 48.2, 347.8, 0.0),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    );
    let deputy = HubInit::from_elements(
        mu,
        &elements(8_000_000.0, 0.1, 33.3, 48.2, 347.8, -10.0),
        [-0.3, 0.0, 0.5],
        [0.003, -0.02, 0.01],
    );
    let mut base = BskSim::new(0.1, 0.1);
    let mut dynamics = FormationDynamics::new(&base, &chief, &deputy);
    let mut fsw = FormationFsw::new(&mut base, &mut dynamics);
    let (mode, runner) = (fsw.mode_request(), base.runner());

    let sampling = dynamics.task_period_nanos;
    let mut chief_translation = MessageLog::<NavigationTranslationMsg>::default();
    let mut deputy_translation = MessageLog::<NavigationTranslationMsg>::default();
    let mut deputy_attitude = MessageLog::<NavigationAttitudeMsg>::default();
    let mut deputy_guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    // Upstream's `sNavAtt2Rec` sits in the chief's task.
    let mut deputy_attitude_task1 = Latch::<NavigationAttitudeMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let chief_nav = &dynamics.chief.simple_nav;
    let deputy_nav = &dynamics.deputy.simple_nav;
    let gateway = &fsw.gateway;
    chief_nav
        .translation_out_msg
        .connect_to(&mut chief_translation.input);
    deputy_nav
        .translation_out_msg
        .connect_to(&mut deputy_translation.input);
    deputy_nav
        .attitude_out_msg
        .connect_to(&mut deputy_attitude_task1.input);
    deputy_attitude_task1
        .output
        .connect_to(&mut deputy_attitude.input);
    gateway.att_guid2.connect_to(&mut deputy_guidance.input);
    chief_nav
        .translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("chief")));
    deputy_nav
        .translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("deputy")));
    chief_nav
        .attitude_out_msg
        .connect_to(recorder.add_source::<NavigationAttitudeMsg>(attitude_columns("chief")));
    deputy_attitude_task1
        .output
        .connect_to(recorder.add_source::<NavigationAttitudeMsg>(attitude_columns("deputy")));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("chief")));
    gateway
        .att_guid2
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("deputy")));
    fsw.spacecraft_pointing
        .module
        .att_reference_out_msg
        .connect_to(recorder.add_source::<AttitudeReferenceMsg>(reference_columns("deputy")));
    gateway
        .cmd_torque_direct
        .connect_to(recorder.add_source::<BodyTorqueCommandMsg>(torque_columns("deputy")));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    base.schedule(&mut simulation);
    dynamics.schedule(&mut simulation);
    fsw.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "sNavTransRec" => &mut chief_translation, period = sampling, priority = RECORDER_PRIORITY;
        "sNavAtt2Rec" => &mut deputy_attitude_task1, period = sampling, priority = RECORDER_PRIORITY;
        "sNavAtt2Log" => &mut deputy_attitude, period = sampling, priority = RECORDER_PRIORITY;
        "sNavTrans2Rec" => &mut deputy_translation, period = sampling, priority = RECORDER_TASK2_PRIORITY;
        "attErr2Rec" => &mut deputy_guidance, period = sampling, priority = RECORDER_TASK2_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_TASK2_PRIORITY);
    }

    mode.set(FormationMode::SpacecraftPointing);
    runner.execute(&mut simulation, min2nano(10.0));
    drop(simulation);
    drop(recorder);

    Results {
        chief_translation,
        deputy_translation,
        deputy_attitude,
        deputy_guidance,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_relative_pointing_formation");
    cli.finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final deputy |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.deputy_guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
    use nalgebra::Vector3;

    /// The deputy's body vector `[1, 2, 3]` ends up along the line to the
    /// chief.
    #[test]
    fn the_deputy_points_at_the_chief() {
        let results = run(Config { write_csv: false });
        let last = results.deputy_guidance.last();
        assert!(last.sigma_br.norm() < 1e-2, "{}", last.sigma_br.norm());

        let index = results.deputy_attitude.values.len() - 1;
        // The attitude log lags one step behind the translations.
        let chief = &results.chief_translation.values[index - 1];
        let deputy = &results.deputy_translation.values[index - 1];
        let to_chief_n = (chief.position_bn_n_m - deputy.position_bn_n_m).normalize();
        let dcm_bn = mrp_to_dcm(results.deputy_attitude.values[index].sigma_bn);
        let alignment_b = Vector3::new(1.0, 2.0, 3.0).normalize();
        let angle = (dcm_bn.transpose() * alignment_b).dot(&to_chief_n).acos();
        assert!(angle < 2.0_f64.to_radians(), "{} deg", angle.to_degrees());
    }
}
