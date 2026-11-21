//! Thruster-arm regulation with MuJoCo dynamics: port of upstream
//! `examples/mujoco/scenarioThrArmControl.py`. Needs the `mujoco` feature.
//!
//! A 330 kg hub carries two four-joint arms (a boom and a tip body each,
//! `sat_w_thruster_arms_regulation.xml`) with one 2.5 N thruster at each tip.
//! The flight software of [`thr_arm`] (upstream's `BSKSim` with
//! `BSK_MujocoDynamics` and `BSK_MujocoFSW` and the regulation config)
//! regulates the hub to the origin with zero attitude:
//!
//! - every 10 s (the outer loop) an inertial Cartesian PD force
//!   (`inertialCartFeedback`) and an MRP feedback torque (`mrpFeedback`,
//!   against `inertial3D`) are allocated to arm joint angles and thruster
//!   forces (`jointThrAllocation`), and the forces to rounded on-times
//!   (`thrFiringRound`);
//! - the events then drive the joints to the allocated angles
//!   (`hingedJointArrayMotor`, with `jointMotionCompensator` holding the
//!   hub's attitude), fire the pulse once the joints are there
//!   (`thrOnTimeToForce`, with `thrJointCompensation` holding the arms),
//!   and coast until the next allocation, which cancels any pulse still
//!   firing.
//!
//! The hub starts 1.95 m from the origin, drifting away at 8.7 mm/s, with
//! `σ_BN = (0.15, -0.1, 0.1)` and `ω_BN = (0.01, -0.01, 0.005)` rad/s,
//! the joints at zero. Every 10 ms step of the dynamics task records the
//! system centre of mass and the hub attitude to
//! `examples/output/scenarios/scenarioThrArmControl.csv` (upstream's
//! `posRec` and `attRec`); `plot_scenario_thr_arm_control.py` draws
//! upstream's `_position` and `_attitude` figures.
//!
//! Options (upstream `run(showPlots, runTime, showProgressBar, savePlots)`):
//! `--run-time <s>` (default 500 s). `--all` runs the default.
//!
//! The tests port `src/tests/test_scenarioThrArmControl.py`: a 20.01 s run
//! (two control periods and the next update) checking the pulse timing and
//! quantization, the delivered impulse of each window against the request
//! (truncated by joint-motion delay and cancellation), and that the
//! regulation starts to act. Upstream's check that the scenario has no
//! joint profiler and no motion penalty is the configuration's; its
//! figure-count check is the plot script's.
//!
//! Differences from upstream: SLSQP is replaced by the bounded quasi-Newton
//! method of [`thr_arm::bounded_sqp`], which may pick a different
//! allocation; message headers have the stand-ins documented in
//! [`thr_arm::fsw`]; the `svIntegratorRKF45` step control is basilisk-rs's
//! [`Rkf45`](basilisk_rs::integrators::Rkf45) with upstream's default
//! tolerances.

#[path = "support/thr_arm/mod.rs"]
mod thr_arm;

use basilisk_rs::messages::{SingleActuatorMsg, SpacecraftStateMsg, ThrusterCommandMsg};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use nalgebra::Vector3;
use thr_arm::cli::Cli;
use thr_arm::common::vector_columns;
use thr_arm::config::ThrArmConfig;
use thr_arm::events::InTasks;
use thr_arm::message_log::MessageLog;
use thr_arm::recording::scenario_csv;
use thr_arm::shared::Shared;
use thr_arm::{
    DYNAMICS_RECORDER_PRIORITY, OUTER_LOOP_RECORDER_PRIORITY, RECORDER_TASK_PRIORITY, ThrArmSim,
    epoch, sec2nano,
};

const STEM: &str = "scenarioThrArmControl";
/// Upstream `run`'s default `runTime` \[s\].
const DEFAULT_RUN_TIME_S: f64 = 500.0;

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `runTime` \[s\].
    run_time_s: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// What a run recorded.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `posRec`: the system centre of mass (`r_CN_N`, `v_CN_N` in
    /// `position_m`, `velocity_mps`), every dynamics step.
    position: MessageLog<SpacecraftStateMsg>,
    /// Upstream `attRec`: the hub origin state, every dynamics step.
    attitude: MessageLog<SpacecraftStateMsg>,
    /// The test's `thrFiringRound.thrOnTimeOutMsg` recorder (outer loop),
    /// one per thruster.
    on_times: Vec<MessageLog<ThrusterCommandMsg>>,
    /// The test's `thrFiringTimeGateway` recorder (recorder task), one per
    /// thruster; `command_time_nanos` is upstream's `timesWritten()`.
    firing: Vec<MessageLog<ThrusterCommandMsg>>,
    /// The test's `thrForceGateways` recorders (recorder task).
    forces: Vec<MessageLog<SingleActuatorMsg>>,
    /// Upstream's `fsw.jointProfiler is None` and
    /// `fsw.jointThrAllocation.useThetaPenalty`.
    has_joint_profiler: bool,
    uses_theta_penalty: bool,
}

/// Upstream `configure_initial_conditions`.
fn configure_initial_conditions(scene: &Shared<MjScene>, config: &ThrArmConfig) {
    let mut scene = scene.lock();
    let mut hub = scene.body_mut(&config.hub_name);
    hub.set_position(Vector3::new(1.5, -1.0, 0.75));
    hub.set_velocity(Vector3::new(5e-3, -5e-3, 5e-3));
    hub.set_attitude(Vector3::new(0.15, -0.1, 0.1));
    hub.set_attitude_rate(Vector3::new(0.01, -0.01, 0.005));
    // Zero joint angles and rates put the hub at, and moving with, the
    // system centre of mass.
    for (body, joint) in &config.joint_refs {
        let mut body = scene.body_mut(body);
        body.set_scalar_joint_position(joint, 0.0);
        body.set_scalar_joint_velocity(joint, 0.0);
    }
}

/// Upstream `run(runTime=...)` (`scenarioThrArmControl()`, `runScenario`).
fn run(config: Config) -> Results {
    let mut models = ThrArmSim::new(ThrArmConfig::regulation());
    let thr_config = models.config.clone();
    let scene = models.scene();
    let runner = models.runner();
    let tasks = models.fsw.tasks.clone();
    let dyn_period = models.dynamics.task_period_nanos;
    let (outer_period, inner_period) = (
        models.fsw.outer_period_nanos(),
        models.fsw.inner_period_nanos(),
    );

    // Upstream `log_outputs`: posRec and attRec in the dynamics task.
    let com = models.dynamics.system_com.com_states_out_msg.clone();
    let hub = scene
        .lock()
        .body(&thr_config.hub_name)
        .origin()
        .state_out_msg
        .clone();
    let mut position = MessageLog::<SpacecraftStateMsg>::default();
    let mut attitude = MessageLog::<SpacecraftStateMsg>::default();
    com.connect_to(&mut position.input);
    hub.connect_to(&mut attitude.input);
    // The test's recorders.
    let mut on_times: Vec<_> = models
        .fsw
        .thr_firing_round
        .module
        .thr_on_time_out_msgs
        .iter()
        .map(|output| {
            let mut log = MessageLog::<ThrusterCommandMsg>::default();
            output.connect_to(&mut log.input);
            InTasks::new(log, &[&tasks.outer_loop])
        })
        .collect();
    let mut firing: Vec<_> = models
        .fsw
        .gateways
        .thr_firing_time
        .iter()
        .map(|output| {
            let mut log = MessageLog::<ThrusterCommandMsg>::default();
            output.connect_to(&mut log.input);
            InTasks::new(log, &[&tasks.recorder])
        })
        .collect();
    let mut forces: Vec<_> = models
        .fsw
        .gateways
        .thr_force
        .iter()
        .map(|output| {
            let mut log = MessageLog::<SingleActuatorMsg>::default();
            output.connect_to(&mut log.input);
            InTasks::new(log, &[&tasks.recorder])
        })
        .collect();
    let mut csv = config.write_csv.then(|| {
        let mut csv = scenario_csv(STEM);
        com.connect_to(
            csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(vector_columns(
                "position_m",
                "r_CN_N_m",
            ))),
        );
        hub.connect_to(
            csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(vector_columns(
                "sigma_bn", "sigma_BN",
            ))),
        );
        csv
    });

    {
        let mut simulation = Simulation::new(epoch(), false);
        models.schedule(&mut simulation);
        simulation.add_module(
            "posRec",
            &mut position,
            dyn_period,
            DYNAMICS_RECORDER_PRIORITY,
        );
        simulation.add_module(
            "attRec",
            &mut attitude,
            dyn_period,
            DYNAMICS_RECORDER_PRIORITY,
        );
        if let Some(csv) = csv.as_mut() {
            simulation.add_module("csv", csv, dyn_period, DYNAMICS_RECORDER_PRIORITY);
        }
        for log in &mut on_times {
            simulation.add_module("onTimeRec", log, outer_period, OUTER_LOOP_RECORDER_PRIORITY);
        }
        for log in firing.iter_mut() {
            simulation.add_module("firingRec", log, inner_period, RECORDER_TASK_PRIORITY);
        }
        for log in forces.iter_mut() {
            simulation.add_module("forceRec", log, inner_period, RECORDER_TASK_PRIORITY);
        }
        // Upstream `runScenario`: initialize, set the initial conditions,
        // run.
        simulation.initialize();
        configure_initial_conditions(&scene, &thr_config);
        runner.execute(&mut simulation, sec2nano(config.run_time_s));
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        has_joint_profiler: models.fsw.joint_profiler.is_some(),
        uses_theta_penalty: models.fsw.joint_thr_allocation.module.use_theta_penalty(),
        position,
        attitude,
        on_times: on_times.into_iter().map(|log| log.module).collect(),
        firing: firing.into_iter().map(|log| log.module).collect(),
        forces: forces.into_iter().map(|log| log.module).collect(),
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_thr_arm_control [--run-time <s>] [--all]\n\n\
         Upstream run(runTime=500): thruster-arm regulation with MuJoCo dynamics.",
    );
    cli.switch("all");
    let run_time_s = cli.value("run-time", DEFAULT_RUN_TIME_S);
    cli.finish();
    let results = run(Config {
        run_time_s,
        write_csv: true,
    });
    let last = results.position.last();
    println!(
        "{STEM}: {run_time_s} s, final centre-of-mass distance {:.3} m, |sigma_BN| {:.4}",
        last.position_m.norm(),
        results.attitude.last().sigma_bn.norm()
    );
    let directory =
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/output/scenarios");
    println!("wrote {}", directory.join(format!("{STEM}.csv")).display());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Upstream `test_thr_arm_regulation`: pulse delivery with firing
    /// delays and cancellation, and short-run regulation, over two control
    /// periods and the next outer-loop update.
    #[test]
    fn thr_arm_regulation() {
        let config = ThrArmConfig::regulation();
        let period = config.outer_loop_period_s;
        let dt = config.fsw_time_step_s;
        let results = run(Config {
            run_time_s: 2.0 * period + dt,
            write_csv: false,
        });
        assert!(!results.has_joint_profiler);
        assert!(!results.uses_theta_penalty);
        let n_thr = config.num_thrusters;
        let max_forces = &config.thr_forces_n;

        let outer_times = results.on_times[0].times_s();
        for (time, expected) in outer_times.iter().zip([0.0, period, 2.0 * period]) {
            assert!((time - expected).abs() <= dt, "{time}");
        }
        assert!(outer_times.len() >= 3);
        // on_times[window][thruster]
        let on_times: Vec<Vec<f64>> = (0..outer_times.len())
            .map(|row| {
                (0..n_thr)
                    .map(|thr| results.on_times[thr].values[row].on_time_s)
                    .collect()
            })
            .collect();
        for row in &on_times {
            for on_time in row {
                assert!(on_time.is_finite() && *on_time >= 0.0 && *on_time <= period + dt);
                let pulses = on_time / config.pulse_resolution_s;
                assert!((pulses - pulses.round_ties_even()).abs() <= 1e-8);
            }
        }
        assert!(on_times[..2].iter().flatten().any(|on_time| *on_time > 0.0));

        let force_times = results.forces[0].times_s();
        let forces: Vec<Vec<f64>> = (0..force_times.len())
            .map(|row| {
                (0..n_thr)
                    .map(|thr| results.forces[thr].values[row].input)
                    .collect()
            })
            .collect();
        let close = |a: f64, b: f64| (a - b).abs() <= 1e-8 + 1e-5 * b.abs();
        for row in &forces {
            for (force, max) in row.iter().zip(max_forces) {
                assert!(
                    force.is_finite() && (close(*force, 0.0) || close(*force, *max)),
                    "{force}"
                );
            }
        }
        assert!(forces.iter().flatten().any(|force| *force > 0.0));

        let firing_times: Vec<f64> = results.firing[0]
            .values
            .iter()
            .map(|command| command.command_time_nanos as f64 * 1e-9)
            .collect();
        for window in 0..2 {
            let (start, end) = (outer_times[window], outer_times[window + 1]);
            let mut delivered = vec![0.0; n_thr];
            for (time, row) in force_times.iter().zip(&forces) {
                if *time >= start && *time < end {
                    for (sum, force) in delivered.iter_mut().zip(row) {
                        *sum += force * dt;
                    }
                }
            }
            // The first write in the window of a nonzero on-time: the pulse
            // start, which joint motion delays.
            let first_start = firing_times
                .iter()
                .enumerate()
                .filter(|(row, time)| {
                    **time >= start
                        && **time < end
                        && (0..n_thr).any(|thr| results.firing[thr].values[*row].on_time_s > 0.0)
                })
                .map(|(_, time)| *time)
                .next();
            let available = first_start.map_or(0.0, |first| end - first);
            for thr in 0..n_thr {
                let expected = max_forces[thr] * on_times[window][thr].min(available);
                assert!(
                    (delivered[thr] - expected).abs() <= 2.0 * max_forces[thr] * dt,
                    "pulse delivery in window {window}: expected {expected}, got {}",
                    delivered[thr]
                );
            }
        }

        let radial: Vec<(f64, f64)> = results
            .position
            .values
            .iter()
            .map(|state| {
                let distance = state.position_m.norm();
                (
                    distance,
                    state.position_m.dot(&state.velocity_mps) / distance,
                )
            })
            .collect();
        let attitude: Vec<(f64, f64)> = results
            .attitude
            .values
            .iter()
            .map(|state| (state.sigma_bn.norm(), state.omega_radps.norm()))
            .collect();
        assert!(radial.iter().all(|(d, v)| d.is_finite() && v.is_finite()));
        assert!(attitude.iter().all(|(s, w)| s.is_finite() && w.is_finite()));
        // Initial motion is away from the target; a short run may only slow
        // it.
        let (first, last) = (radial[0], *radial.last().unwrap());
        assert!(first.1 > 0.0, "{first:?}");
        assert!(last.1 < first.1, "{last:?} vs {first:?}");
        let (first, last) = (attitude[0], *attitude.last().unwrap());
        assert!(last.1 < first.1, "{last:?} vs {first:?}");
        assert!(last.0 < 1.2 * first.0, "{last:?} vs {first:?}");
    }
}
