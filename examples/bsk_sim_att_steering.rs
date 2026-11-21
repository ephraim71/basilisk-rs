//! BskSim MRP steering: port of upstream
//! `examples/BskSim/scenarios/scenario_AttSteering.py`.
//!
//! The BskSim spacecraft ([`bsk_sim::dynamics`]) on the 10 000 km,
//! `e = 0.01` orbit, starting at `sigma_BN = [0.5, 0.6, -0.3]`, `omega_BN =
//! [0.01, -0.01, -0.01] rad/s`, in the `steeringRW` mode: `hillPoint` and
//! `trackingError` feed the MRP steering law (`K1 = 0.05`, `K3 = 0.75`,
//! `omega_max = 1 deg/s`), whose rate command `rate_servo` (`P = 150`,
//! `Ki = 5`) tracks with the four wheels through `rwMotorTorque`. 10 minutes
//! on the default 0.1 s tasks.
//!
//! Writes `examples/output/scenarios/scenario_AttSteering.csv` every FSW
//! step: wheel speeds, wheel torque commands, guidance and the steering rate
//! command. `plot_bsk_sim_att_steering.py` draws upstream's
//! `attitudeErrorNorm`, `rwMotorTorque`, `rateError` and `rwSpeed` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! the steering law's rate bound and the convergence to the Hill frame.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::path::PathBuf;

use basilisk_rs::messages::{ArrayMotorTorqueMsg, AttitudeGuidanceMsg, RateCommandMsg, RwSpeedMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use bsk_sim::cli::Cli;
use bsk_sim::common::vector_columns;
use bsk_sim::dynamics;
use bsk_sim::fsw::FswMode;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, RECORDER_PRIORITY, guidance_columns, min2nano, motor_torque_columns,
    scenario_recorder, standard_orbit, wheel_speed_columns,
};

const STEM: &str = "scenario_AttSteering";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    rate_command: MessageLog<RateCommandMsg>,
    motor_torque: MessageLog<ArrayMotorTorqueMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let initial = standard_orbit(0.01, [0.5, 0.6, -0.3], [0.01, -0.01, -0.01]);
    let mut models = BskModels::new(0.1, 0.1, &initial);
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());

    let sampling = models.fsw.task_period_nanos;
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut rate_command = MessageLog::<RateCommandMsg>::default();
    let mut motor_torque = MessageLog::<ArrayMotorTorqueMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let gateway = &models.fsw.gateway;
    let steering = &models.fsw.mrp_steering.module.rate_cmd_out_msg;
    gateway.att_guid.connect_to(&mut guidance.input);
    steering.connect_to(&mut rate_command.input);
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
    steering.connect_to(
        recorder.add_source::<RateCommandMsg>(CsvSourceConfig::columns(vector_columns(
            "omega_bast_r_b_radps",
            "omega_BastR_B_rad_per_s",
        ))),
    );

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    basilisk_rs::schedule! { simulation,
        "attErrRec" => &mut guidance, period = sampling, priority = RECORDER_PRIORITY;
        "rateCmdRec" => &mut rate_command, period = sampling, priority = RECORDER_PRIORITY;
        "rwMotorRec" => &mut motor_torque, period = sampling, priority = RECORDER_PRIORITY;
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    mode.set(FswMode::SteeringRw);
    runner.execute(&mut simulation, min2nano(10.0));
    drop(simulation);
    drop(recorder);

    Results {
        guidance,
        rate_command,
        motor_torque,
        csv_path,
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_att_steering");
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

    /// The steering law commands at most `omega_max = 1 deg/s` per axis, and
    /// the servo makes the wheels follow it to the Hill frame.
    #[test]
    fn the_steering_law_bounds_the_rate_and_converges() {
        let results = run(Config { write_csv: false });
        let omega_max = 1.0_f64.to_radians();
        for command in &results.rate_command.values {
            assert!(command.omega_bast_r_b_radps.amax() <= omega_max + 1e-12);
        }
        let saturated = results
            .rate_command
            .values
            .iter()
            .any(|command| command.omega_bast_r_b_radps.amax() > 0.9 * omega_max);
        assert!(
            saturated,
            "the large initial error saturates the steering law"
        );
        let last = results.guidance.last();
        assert!(last.sigma_br.norm() < 1e-3, "{}", last.sigma_br.norm());
        for command in &results.motor_torque.values {
            assert!(
                command.motor_torque_nm[..4]
                    .iter()
                    .all(|torque| torque.is_finite())
            );
        }
    }
}
