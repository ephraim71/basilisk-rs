//! BskSim wheel feedback, then direct torque: port of upstream
//! `examples/BskSim/scenarios/scenario_FeedbackRW.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on the 10 000 km,
//! `e = 0.01` orbit, starting at `sigma_BN = [0.1, 0.2, -0.3]` with the large
//! rate `omega_BN = [0.1, -0.01, 0.03] rad/s`. For 10 minutes the
//! `inertial3D` mode points it at `sigma_R0N = [0.2, 0.4, 0.6]` with the four
//! wheels (`mrpFeedbackRWs`, `rwMotorTorque`); the request then changes to
//! `directInertial3D` until 30 minutes, which turns the wheel control off
//! and applies `mrpFeedbackControl`'s torque through the external
//! force/torque effector. Default 0.1 s tasks.
//!
//! Writes `examples/output/scenarios/scenario_FeedbackRW.csv` every FSW
//! step: wheel speeds, wheel torque commands, guidance and each wheel's
//! applied torque (`rwOutMsgs[i].u_current`). `plot_bsk_sim_feedback_rw.py`
//! draws upstream's `attitudeErrorNorm`, `rwMotorTorque`, `rateError` and
//! `rwSpeed` figures. (Upstream also draws its message-connection graph,
//! which has no counterpart here.)
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! the two control phases.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{
    ArrayMotorTorqueMsg, AttitudeGuidanceMsg, ReactionWheelStateMsg, RwSpeedMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use bsk_sim::cli::Cli;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, RECORDER_PRIORITY, SIGMA_BN_INIT, guidance_columns, min2nano, motor_torque_columns,
    scenario_recorder, standard_orbit, wheel_speed_columns,
};

const STEM: &str = "scenario_FeedbackRW";

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
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let initial = standard_orbit(0.01, SIGMA_BN_INIT, [0.1, -0.01, 0.03]);
    let mut models = BskModels::new(0.1, 0.1, &initial);
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    let sampling = models.fsw.task_period_nanos;
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut motor_torque = MessageLog::<ArrayMotorTorqueMsg>::default();
    let mut wheels: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..4).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let gateway = &models.fsw.gateway;
    gateway.att_guid.connect_to(&mut guidance.input);
    gateway.cmd_rw_motor.connect_to(&mut motor_torque.input);
    models
        .dynamics
        .rw_speed_out_msg
        .connect_to(recorder.add_source::<RwSpeedMsg>(wheel_speed_columns()));
    gateway
        .cmd_rw_motor
        .connect_to(recorder.add_source::<ArrayMotorTorqueMsg>(motor_torque_columns()));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    for (index, (state, log)) in models
        .dynamics
        .rw_out_msgs
        .iter()
        .zip(&mut wheels)
        .enumerate()
    {
        state.connect_to(&mut log.input);
        state.connect_to(
            recorder.add_source::<ReactionWheelStateMsg>(CsvSourceConfig::columns([(
                "applied_motor_torque_nm".to_string(),
                format!("rwTorque_Nm_{index}"),
            )])),
        );
    }

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "attErrRec" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
        "rwMotorRec" => &mut motor_torque, period = sampling, priority = RECORDER_PRIORITY;
    }
    for log in &mut wheels {
        simulation.add_module("rwLogs", log, sampling, RECORDER_PRIORITY);
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    // `runScenario`.
    mode.set(FswMode::Inertial3D);
    runner.execute(&mut simulation, min2nano(10.0));
    mode.set(FswMode::DirectInertial3D);
    runner.execute(&mut simulation, min2nano(30.0));
    drop(simulation);
    drop(recorder);

    Results {
        guidance,
        motor_torque,
        wheels,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_feedback_rw");
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

    const SWITCH_NANOS: u64 = 600_000_000_000;

    /// The large initial rate saturates the 0.2 N m wheels in the first
    /// phase; after the switch the wheel commands stay zero (the wheel tasks
    /// are off and the gateway was zeroed) and the external torque brings the
    /// attitude to the reference.
    #[test]
    fn the_two_phases_use_their_actuators() {
        let results = run(Config { write_csv: false });
        let mut saturated = false;
        for wheel in &results.wheels {
            for (time, state) in wheel.times_nanos.iter().zip(&wheel.values) {
                assert!(state.applied_motor_torque_nm.abs() <= 0.2 + 1e-12);
                if *time <= SWITCH_NANOS {
                    saturated |= state.applied_motor_torque_nm.abs() > 0.2 - 1e-9;
                } else if *time > SWITCH_NANOS + 100_000_000 {
                    assert_eq!(state.applied_motor_torque_nm, 0.0, "at {time}");
                }
            }
        }
        assert!(saturated);
        for command in results.motor_torque.since(SWITCH_NANOS + 100_000_000) {
            assert!(command.motor_torque_nm.iter().all(|torque| *torque == 0.0));
        }
        // The wheels leave a large error at the switch (the rate saturated
        // them); 20 minutes of direct control shrink it about 70 times.
        let at_switch = results.guidance.at(SWITCH_NANOS).sigma_br.norm();
        let last = results.guidance.last();
        assert!(at_switch > 0.5, "{at_switch}");
        assert!(last.sigma_br.norm() < 2e-2, "{}", last.sigma_br.norm());
        assert!(last.omega_br_b_radps.norm() < 1e-3);
    }
}
