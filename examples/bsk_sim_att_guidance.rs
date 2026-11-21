//! BskSim Hill-frame pointing: port of upstream
//! `examples/BskSim/scenarios/scenario_AttGuidance.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on the 10 000 km, `e = 0.1`
//! orbit, starting at `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001,
//! -0.01, 0.03] rad/s`, in the `hillPoint` mode: `hillPoint` (Earth from
//! `EarthEphemObject`) and `trackingError` feed `mrpFeedbackRWs` and
//! `rwMotorTorque` on the four wheels. 10 minutes on the default 0.1 s tasks.
//!
//! Writes `examples/output/scenarios/scenario_AttGuidance.csv` every FSW
//! step: the navigation attitude and translation, the reference
//! (`attRefMsg`), the guidance (`attGuidMsg`) and the commanded torque
//! (`cmdTorqueMsg`). `plot_bsk_sim_att_guidance.py` draws upstream's
//! `attitudeErrorNorm`, `rwMotorTorque`, `rateError`, `orientation` and
//! `attitudeGuidance` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that the body axes reach the Hill frame.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg,
    NavigationTranslationMsg,
};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, attitude_columns,
    guidance_columns, min2nano, reference_columns, scenario_recorder, standard_orbit,
    torque_columns, translation_columns,
};

const STEM: &str = "scenario_AttGuidance";

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
    guidance: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut models = BskModels::new(
        0.1,
        0.1,
        &standard_orbit(0.1, SIGMA_BN_INIT, OMEGA_BN_B_INIT),
    );
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    let sampling = models.fsw.task_period_nanos;
    let mut attitude = MessageLog::<NavigationAttitudeMsg>::default();
    let mut translation = MessageLog::<NavigationTranslationMsg>::default();
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let nav = &models.dynamics.simple_nav;
    let gateway = &models.fsw.gateway;
    nav.attitude_out_msg.connect_to(&mut attitude.input);
    nav.translation_out_msg.connect_to(&mut translation.input);
    gateway.att_guid.connect_to(&mut guidance.input);
    nav.attitude_out_msg
        .connect_to(recorder.add_source::<NavigationAttitudeMsg>(attitude_columns("")));
    nav.translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("")));
    gateway
        .att_ref
        .connect_to(recorder.add_source::<AttitudeReferenceMsg>(reference_columns("")));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    gateway
        .cmd_torque
        .connect_to(recorder.add_source::<BodyTorqueCommandMsg>(torque_columns("")));

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "attNavRec" => &mut attitude, period = sampling, priority = RECORDER_PRIORITY;
        "transNavRec" => &mut translation, period = sampling, priority = RECORDER_PRIORITY;
        "attErrRec" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    mode.set(FswMode::HillPoint);
    runner.execute(&mut simulation, min2nano(10.0));
    drop(simulation);
    drop(recorder);

    Results {
        attitude,
        translation,
        guidance,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_att_guidance");
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
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    /// Upstream's `orientation` figure: `i_r . b_1`, `i_theta . b_2` and
    /// `i_h . b_3` reach 1 once the body axes are the Hill frame's.
    #[test]
    fn the_body_axes_reach_the_hill_frame() {
        let results = run(Config { write_csv: false });
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1e-3, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 1e-4);

        let state = results.translation.last();
        let (r, v) = (state.position_bn_n_m, state.velocity_bn_n_mps);
        let i_r = r.normalize();
        let i_h = r.cross(&v).normalize();
        let i_theta = i_h.cross(&i_r);
        let dcm_bn = mrp_to_dcm(results.attitude.last().sigma_bn);
        for (axis, row) in [i_r, i_theta, i_h].iter().zip(0..3) {
            let projection = axis.dot(&dcm_bn.row(row).transpose());
            assert!(projection > 0.999, "{row}: {projection}");
        }
    }
}
