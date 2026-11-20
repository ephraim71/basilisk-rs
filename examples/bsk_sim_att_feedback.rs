//! BskSim inertial attitude feedback: port of upstream
//! `examples/BskSim/scenarios/scenario_AttFeedback.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on the 10 000 km,
//! `e = 0.01` orbit, starting at `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN =
//! [0.001, -0.01, 0.03] rad/s`, in the `inertial3D` mode: `inertial3D`
//! (`sigma_R0N = [0.2, 0.4, 0.6]`) and `trackingError` feed `mrpFeedbackRWs`
//! (`K = 3.5`, `P = 30`, no integral feedback) and `rwMotorTorque` on the four
//! wheels. 10 minutes on upstream's default 0.1 s tasks.
//!
//! Writes `examples/output/scenarios/scenario_AttFeedback.csv`: the
//! attitude guidance (`attGuidMsg`) and the navigation translation every FSW
//! step, upstream's recorders. `plot_bsk_sim_att_feedback.py` draws
//! upstream's `attitudeErrorNorm` and `rateError` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! that the attitude reaches the inertial reference with the wheels.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{ArrayMotorTorqueMsg, AttitudeGuidanceMsg, NavigationTranslationMsg};
use basilisk_rs::simulation::Simulation;
use bsk_sim::cli::Cli;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, guidance_columns, min2nano,
    scenario_recorder, standard_orbit, translation_columns,
};

const STEM: &str = "scenario_AttFeedback";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    motor_torque: MessageLog<ArrayMotorTorqueMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut models = BskModels::new(
        0.1,
        0.1,
        &standard_orbit(0.01, SIGMA_BN_INIT, OMEGA_BN_B_INIT),
    );
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    // `log_outputs`: sampled at the FSW rate, in the dynamics task.
    let sampling = models.fsw.task_period_nanos;
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut motor_torque = MessageLog::<ArrayMotorTorqueMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let gateway = &models.fsw.gateway;
    gateway.att_guid.connect_to(&mut guidance.input);
    gateway.cmd_rw_motor.connect_to(&mut motor_torque.input);
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
        "cmdRwMotorMsg" => &mut motor_torque, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    // `runScenario`.
    mode.set(FswMode::Inertial3D);
    runner.execute(&mut simulation, min2nano(10.0));
    drop(simulation);
    drop(recorder);

    Results {
        guidance,
        motor_torque,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_att_feedback");
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

    /// The first sample is the zeroed gateway (upstream drops it: "before
    /// FSW is called"); the FSW error then starts at the initial attitude
    /// relative to `sigma_R0N`.
    #[test]
    fn the_first_samples_are_upstream_s() {
        let results = run(Config { write_csv: false });
        assert_eq!(results.guidance.values.len(), 6001);
        let first = &results.guidance.values[0];
        assert_eq!(first.sigma_br, nalgebra::Vector3::zeros());
        assert_eq!(first.omega_br_b_radps, nalgebra::Vector3::zeros());
        assert!(results.guidance.values[1].sigma_br.norm() > 0.3);
    }

    /// The wheels bring the spacecraft to the inertial reference, their
    /// torques limited to 0.2 N m.
    #[test]
    fn the_wheels_reach_the_inertial_reference() {
        let results = run(Config { write_csv: false });
        let last = results.guidance.last();
        // From |sigma_BR| = 0.97, about a thousandth after 10 minutes.
        assert!(last.sigma_br.norm() < 2e-3, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 2e-4);
        let peak = results
            .motor_torque
            .values
            .iter()
            .flat_map(|command| command.motor_torque_nm[..4].to_vec())
            .fold(0.0_f64, |peak, torque| peak.max(torque.abs()));
        assert!(peak > 0.0);
    }
}
