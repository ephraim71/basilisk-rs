//! BskSim velocity pointing on a hyperbola: port of upstream
//! `examples/BskSim/scenarios/scenario_AttGuidHyperbolic.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on an Earth flyby
//! hyperbola (`a = -150 000 km`, `e = 1.5`, `i = 33.3 deg`, `Omega = 48.2
//! deg`, `omega = 347.8 deg`, `f = 30 deg`), starting at `sigma_BN = [0.1,
//! 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`, in the
//! `velocityPoint` mode: `velocityPoint` and `trackingError` feed
//! `mrpFeedbackRWs` and `rwMotorTorque`. 10 minutes on the default 0.1 s
//! tasks.
//!
//! Writes `examples/output/scenarios/scenario_AttGuidHyperbolic.csv` every
//! FSW step: the navigation translation, the guidance and the commanded
//! torque. `plot_bsk_sim_att_guid_hyperbolic.py` draws upstream's
//! `attitudeErrorNorm`, `rwMotorTorque`, `rateError` and `orbit` figures
//! (with `scenarioAttGuideHyperbolic`'s plot functions, as upstream).
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that the body axes reach the velocity frame.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, NavigationAttitudeMsg, NavigationTranslationMsg,
};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::grav_factory::Body;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, HubInit, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, elements,
    guidance_columns, min2nano, scenario_recorder, torque_columns, translation_columns,
};

const STEM: &str = "scenario_AttGuidHyperbolic";

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
    let initial = HubInit::from_elements(
        Body::Earth.mu_m3ps2(),
        &elements(-150_000.0 * 1000.0, 1.5, 33.3, 48.2, 347.8, 30.0),
        SIGMA_BN_INIT,
        OMEGA_BN_B_INIT,
    );
    let mut models = BskModels::new(0.1, 0.1, &initial);
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
    nav.translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("")));
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

    mode.set(FswMode::VelocityPoint);
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
    let cli = Cli::from_env("bsk_sim_att_guid_hyperbolic");
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

    /// `velocityPoint`'s frame: `b_2` along the velocity, `b_3` along the
    /// orbit normal.
    #[test]
    fn the_body_axes_reach_the_velocity_frame() {
        let results = run(Config { write_csv: false });
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1e-3, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 1e-4);

        let state = results.translation.last();
        let (r, v) = (state.position_bn_n_m, state.velocity_bn_n_mps);
        let dcm_bn = mrp_to_dcm(results.attitude.last().sigma_bn);
        let b2 = dcm_bn.row(1).transpose();
        let b3 = dcm_bn.row(2).transpose();
        assert!(b2.dot(&v.normalize()) > 0.999);
        assert!(b3.dot(&r.cross(&v).normalize()) > 0.999);
    }

    /// The spacecraft flies the hyperbola: its energy is positive and its
    /// eccentricity stays 1.5.
    #[test]
    fn the_spacecraft_flies_the_hyperbola() {
        let results = run(Config { write_csv: false });
        let mu = Body::Earth.mu_m3ps2();
        let state = results.translation.last();
        let elements = basilisk_rs::utilities::orbital_motion::state_to_elements(
            mu,
            state.position_bn_n_m,
            state.velocity_bn_n_mps,
        );
        assert!(
            (elements.eccentricity - 1.5).abs() < 1e-4,
            "{}",
            elements.eccentricity
        );
        assert!((elements.semi_major_axis + 150_000.0e3).abs() < 1e3);
    }
}
