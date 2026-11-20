//! BskSim reaction-wheel friction faults: port of upstream
//! `examples/BskSim/scenarios/scenario_AddRWFault.py`.
//!
//! The spacecraft of `scenario_AttFeedback` (10 000 km, `e = 0.01`) Hill
//! pointing with its four wheels for 30 minutes, while `BSK_Dynamics`'s
//! fault events act on wheel `RW1`: at 10 minutes the one-time fault adds
//! 0.05 N m of Coulomb friction, and at every dynamics step the repeated
//! fault adds 0.005 N m with probability 1/3000 (upstream
//! `oneTimeRWFaultFlag = 1`, `oneTimeFaultTime = 10 min`,
//! `repeatRWFaultFlag = 1`). Each fault is a rule on the wheel's parameter
//! port (see [`bsk_sim::dynamics::RwFaults`]); the random draws come from a
//! seeded generator, so a rerun repeats them.
//!
//! Writes `examples/output/scenarios/scenario_AddRWFault.csv` every FSW
//! step (wheel speeds, guidance, navigation translation and each wheel's
//! friction torque `u_f`) and `scenario_AddRWFaultLog.csv` (upstream's
//! `RWFaultLog`: time, wheel and friction added). `plot_bsk_sim_add_rw_fault.py`
//! draws upstream's `attitudeErrorNorm`, `rateError`, `RWSpeeds` and
//! `RWFriction` figures.
//!
//! Upstream's `test_bskTestScript.py` only runs the scenario; the tests check
//! the friction the faults add and that the attitude control copes.

#[path = "support/bsk_sim/mod.rs"]
mod bsk_sim;

use std::io::Write;
use std::path::PathBuf;

use basilisk_rs::messages::{
    AttitudeGuidanceMsg, NavigationTranslationMsg, ReactionWheelStateMsg, RwSpeedMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use bsk_sim::cli::Cli;
use bsk_sim::common::scenario_output_path;
use bsk_sim::dynamics::{self, RwFaultRecord};
use bsk_sim::fsw::FswMode;
use bsk_sim::message_log::MessageLog;
use bsk_sim::{
    BskModels, OMEGA_BN_B_INIT, RECORDER_PRIORITY, SIGMA_BN_INIT, guidance_columns, min2nano,
    scenario_recorder, standard_orbit, translation_columns, wheel_speed_columns,
};

const STEM: &str = "scenario_AddRWFault";

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV files (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    wheels: Vec<MessageLog<ReactionWheelStateMsg>>,
    fault_log: Vec<RwFaultRecord>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut models = BskModels::new(
        0.1,
        0.1,
        &standard_orbit(0.01, SIGMA_BN_INIT, OMEGA_BN_B_INIT),
    );
    let (mode, runner) = (models.fsw.mode_request(), models.base.runner());
    let faults = models.dynamics.rw_faults.clone();
    faults.one_time_at(min2nano(10.0));
    faults.repeat();

    let sampling = models.fsw.task_period_nanos;
    let mut guidance = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut wheels: Vec<MessageLog<ReactionWheelStateMsg>> =
        (0..4).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder(STEM, config.write_csv);
    let gateway = &models.fsw.gateway;
    gateway.att_guid.connect_to(&mut guidance.input);
    models
        .dynamics
        .rw_speed_out_msg
        .connect_to(recorder.add_source::<RwSpeedMsg>(wheel_speed_columns()));
    gateway
        .att_guid
        .connect_to(recorder.add_source::<AttitudeGuidanceMsg>(guidance_columns("")));
    models
        .dynamics
        .simple_nav
        .translation_out_msg
        .connect_to(recorder.add_source::<NavigationTranslationMsg>(translation_columns("")));
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
                "friction_torque_nm".to_string(),
                format!("rwFriction_Nm_{index}"),
            )])),
        );
    }

    let mut simulation = Simulation::new(dynamics::epoch(), false);
    models.schedule(&mut simulation);
    simulation.add_module("attGuidMsg", &mut guidance, sampling, RECORDER_PRIORITY);
    for log in &mut wheels {
        simulation.add_module("rwLogs", log, sampling, RECORDER_PRIORITY);
    }
    if config.write_csv {
        simulation.add_module("csv", &mut recorder, sampling, RECORDER_PRIORITY);
    }

    mode.set(FswMode::HillPoint);
    runner.execute(&mut simulation, min2nano(30.0));
    drop(simulation);
    drop(recorder);

    let fault_log = faults.log();
    if config.write_csv {
        write_fault_log(&fault_log);
    }
    Results {
        guidance,
        wheels,
        fault_log,
        csv_path,
    }
}

/// Upstream `RWFaultLog`, for the friction figure's fault markers.
fn write_fault_log(log: &[RwFaultRecord]) {
    let path = scenario_output_path(&format!("{STEM}Log.csv"));
    let mut file = std::fs::File::create(&path).expect("create fault log");
    writeln!(file, "time_min,wheel,friction_Nm").expect("write fault log");
    for record in log {
        writeln!(
            file,
            "{:.18e},{},{:.18e}",
            record.time_min, record.wheel, record.friction_nm
        )
        .expect("write fault log");
    }
}

fn main() {
    let cli = Cli::from_env("bsk_sim_add_rw_fault");
    cli.finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} ({} friction faults on RW1, final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.fault_log.len(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The one-time fault fires at 10 minutes, the random ones at about one
    /// per 300 s; all of them on `RW1`, whose friction torque is then the
    /// added Coulomb friction while it spins. The other wheels stay
    /// frictionless.
    #[test]
    fn the_faults_add_friction_to_rw1() {
        let results = run(Config { write_csv: false });
        let one_time: Vec<_> = results
            .fault_log
            .iter()
            .filter(|record| record.friction_nm == 0.05)
            .collect();
        assert_eq!(one_time.len(), 1);
        assert!((one_time[0].time_min - 10.0).abs() < 1e-9);
        let random = results.fault_log.len() - 1;
        assert!((1..=20).contains(&random), "{random} random faults");
        assert!(results.fault_log.iter().all(|record| record.wheel == 1));

        // A fault checked after the tick at `t` acts from the next tick on.
        let added_before = |nanos: u64| -> f64 {
            results
                .fault_log
                .iter()
                .filter(|record| record.time_min * 60e9 < nanos as f64 - 1.0)
                .map(|record| record.friction_nm)
                .sum()
        };
        let rw1 = &results.wheels[0];
        let mut spinning = 0;
        for (time, state) in rw1.times_nanos.iter().zip(&rw1.values) {
            // Away from zero speed the friction is the Coulomb value. (The
            // friction of a step is taken at its start: near a zero
            // crossing, which 0.2 N m moves the wheel through by up to
            // 0.25 rad/s in a step, it is still the limit-cycle value.)
            if state.omega_radps.abs() > 0.5 {
                let expected = added_before(*time);
                assert!(
                    (state.friction_torque_nm.abs() - expected).abs() < 1e-12,
                    "{} vs {expected} at {time}",
                    state.friction_torque_nm
                );
                spinning += 1;
            }
        }
        assert!(spinning > 10_000);
        for wheel in &results.wheels[1..] {
            assert!(
                wheel
                    .values
                    .iter()
                    .all(|state| state.friction_torque_nm == 0.0)
            );
        }
    }

    /// The wheels absorb the friction: the spacecraft keeps Hill pointing.
    #[test]
    fn the_attitude_control_copes_with_the_friction() {
        let results = run(Config { write_csv: false });
        // Without integral feedback the friction leaves a steady error of a
        // few degrees, which the control holds.
        let worst_after_settling = results
            .guidance
            .since(min2nano(8.0))
            .map(|guidance| guidance.sigma_br.norm())
            .fold(0.0_f64, f64::max);
        assert!(worst_after_settling < 3e-2, "{worst_after_settling}");
    }
}
