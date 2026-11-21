//! Thruster-arm trajectory tracking with MuJoCo dynamics: port of upstream
//! `examples/mujoco/scenarioThrArmTracking.py`. Needs the `mujoco` feature.
//!
//! The thruster-arm spacecraft of [`thr_arm`] (a 980 kg hub with two
//! four-joint arms, one 0.25 N thruster at each tip,
//! `sat_w_thruster_arms_tracking.xml`) follows upstream's `arcLineHoldRef`
//! path around a target at the origin: half a 20 m circle tilted 45° out of
//! the inertial `x-y` plane over 5 h, a 1 h quintic blend onto the `y` axis,
//! a 110 min deceleration to `(0, -5, 0)` m and a hold, while
//! `locationPointing` keeps the hub's `+x` axis on the target. The outer loop
//! (10 s) allocates the translational and rotational commands to arm angles
//! and thruster pulses, which the events of [`thr_arm::fsw`] sequence into
//! joint motion, firing and coasting, as in `scenario_thr_arm_control`.
//!
//! The hub starts at `(0.5, 19.65, -0.25)` m, moving at `(2.5, -0.4, 1.75)`
//! mm/s, with `σ_BN = (0.16, 0.05, -0.54)` and `ω_BN = (1.0, -0.25, -0.95)`
//! mrad/s, the joints at zero. Every outer-loop update records the system
//! centre of mass, the reference position and the attitude tracking error to
//! `examples/output/scenarios/scenarioThrArmTracking<variant>.csv`
//! (upstream's `posRec`, `desPosRec`, `attErrorRec`), where the variant is
//! empty, `_jointProfiler`, `_motionPenalty` or `_jointProfiler_motionPenalty`;
//! `plot_scenario_thr_arm_tracking.py` draws upstream's `_position` and
//! `_attitude` figures.
//!
//! Options (upstream `run(runTime, useJointProfiler, useMotionPenalty)`):
//! `--run-time <s>` (default 8 h), `--joint-profiler` (a 9.5 s cubic joint
//! reference profile), `--motion-penalty` (allocation joint-motion weight
//! `1e-5`); `--all` runs the four combinations upstream's test runs.
//!
//! The tests port `src/tests/test_scenarioThrArmTracking.py`: both of its
//! tests, `test_thr_arm_tracking_options` in all four option combinations
//! (20.01 s runs) and `test_thr_arm_profiled_pulse_truncation`, which feeds
//! fixed commands to check that a profiler-delayed pulse is cut at the next
//! control update. Upstream's figure-count checks are the plot script's.
//!
//! Differences from upstream: as `scenario_thr_arm_control` (the SLSQP
//! stand-in, header stand-ins, `Rkf45` step control).

#[path = "support/thr_arm/mod.rs"]
mod thr_arm;

use basilisk_rs::device_interface::joint_array_ref_profiler::JointProfile;
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, JointArrayStateMsg, ScalarJointStateMsg, SingleActuatorMsg,
    SpacecraftStateMsg, ThrusterCommandMsg, TranslationReferenceMsg,
};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::{CsvRecorder, CsvSourceConfig};
use nalgebra::Vector3;
use thr_arm::cli::Cli;
use thr_arm::common::vector_columns;
use thr_arm::config::ThrArmConfig;
use thr_arm::events::InTasks;
use thr_arm::message_log::MessageLog;
use thr_arm::recording::scenario_csv;
use thr_arm::shared::Shared;
use thr_arm::{
    JOINT_MOTION_RECORDER_PRIORITY, OUTER_LOOP_RECORDER_PRIORITY, RECORDER_TASK_PRIORITY,
    ThrArmSim, epoch, sec2nano,
};

const STEM: &str = "scenarioThrArmTracking";
/// Upstream `DEFAULT_RUN_TIME` \[s\].
const DEFAULT_RUN_TIME_S: f64 = 8.0 * 3600.0;
/// Upstream `CUBIC_PROFILE_DURATION` \[s\].
const CUBIC_PROFILE_DURATION_S: f64 = 9.5;
/// Upstream `MOTION_PENALTY_WEIGHT`.
const MOTION_PENALTY_WEIGHT: f64 = 1e-5;

/// Upstream `ScenarioThrArmTracking(useJointProfiler, useMotionPenalty)`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Options {
    joint_profiler: bool,
    motion_penalty: bool,
}

impl Options {
    const ALL: [Self; 4] = [
        Self::new(false, false),
        Self::new(true, false),
        Self::new(false, true),
        Self::new(true, true),
    ];

    const fn new(joint_profiler: bool, motion_penalty: bool) -> Self {
        Self {
            joint_profiler,
            motion_penalty,
        }
    }

    /// The CSV file stem of the combination.
    fn stem(self) -> String {
        let mut stem = STEM.to_string();
        if self.joint_profiler {
            stem.push_str("_jointProfiler");
        }
        if self.motion_penalty {
            stem.push_str("_motionPenalty");
        }
        stem
    }

    /// Upstream `ScenarioThrArmTracking.__init__`'s configuration.
    fn config(self) -> ThrArmConfig {
        let mut config = ThrArmConfig::tracking();
        if self.joint_profiler {
            config.use_joint_profiler = true;
            config.joint_profile = Some(JointProfile::Cubic {
                duration_s: CUBIC_PROFILE_DURATION_S,
            });
        }
        if self.motion_penalty {
            config.use_theta_alloc_penalty = true;
            config.theta_weight = Some(MOTION_PENALTY_WEIGHT);
        }
        config
    }
}

/// Upstream `configure_initial_conditions`.
fn configure_initial_conditions(scene: &Shared<MjScene>, config: &ThrArmConfig) {
    let mut scene = scene.lock();
    let mut hub = scene.body_mut(&config.hub_name);
    hub.set_position(Vector3::new(0.5, 19.65, -0.25));
    hub.set_velocity(Vector3::new(2.5e-3, -4.0e-4, 1.75e-3));
    hub.set_attitude(Vector3::new(0.16, 0.05, -0.54));
    hub.set_attitude_rate(Vector3::new(1.0e-3, -0.25e-3, -0.95e-3));
    for (body, joint) in &config.joint_refs {
        let mut body = scene.body_mut(body);
        body.set_scalar_joint_position(joint, 0.0);
        body.set_scalar_joint_velocity(joint, 0.0);
    }
}

/// Upstream `log_outputs` (`posRec`, `desPosRec`, `attErrorRec` in the outer
/// loop) and the test's `_record_thr_arm_control` recorders.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Recorders {
    position: InTasks<MessageLog<SpacecraftStateMsg>>,
    reference: InTasks<MessageLog<TranslationReferenceMsg>>,
    attitude_error: InTasks<MessageLog<AttitudeGuidanceMsg>>,
    /// `thrFiringRound.thrOnTimeOutMsg` (outer loop), one per thruster.
    on_times: Vec<InTasks<MessageLog<ThrusterCommandMsg>>>,
    /// `thrForceGateways` (recorder task).
    forces: Vec<InTasks<MessageLog<SingleActuatorMsg>>>,
    /// Joint angles and rates (recorder task), when recorded.
    angles: Vec<InTasks<MessageLog<ScalarJointStateMsg>>>,
    rates: Vec<InTasks<MessageLog<ScalarJointStateMsg>>>,
    /// `jointProfiler.desJointStatesOutMsg` (joint-motion task).
    profile: Option<InTasks<MessageLog<JointArrayStateMsg>>>,
    csv: Option<CsvRecorder>,
    outer_period: u64,
    inner_period: u64,
}

fn log<T: basilisk_rs::messages::SimulationMessage>(
    output: &basilisk_rs::messages::Output<T>,
    task: &thr_arm::task_switch::TaskSwitch,
) -> InTasks<MessageLog<T>> {
    let mut log = MessageLog::<T>::default();
    output.connect_to(&mut log.input);
    InTasks::new(log, &[task])
}

impl Recorders {
    fn new(models: &ThrArmSim, record_joints: bool, csv_stem: Option<&str>) -> Self {
        let fsw = &models.fsw;
        let tasks = &fsw.tasks;
        let com = models.dynamics.system_com.com_states_out_msg.clone();
        let reference = fsw
            .trans_ref
            .as_ref()
            .expect("tracking has a translation reference")
            .module
            .trans_ref_out_msg
            .clone();
        let guidance = fsw
            .att_ref
            .as_ref()
            .expect("tracking has an attitude reference")
            .module
            .att_guid_out_msg
            .clone();
        let (mut angles, mut rates) = (Vec::new(), Vec::new());
        if record_joints {
            let scene = models.scene();
            let scene = scene.lock();
            for (body, joint) in &models.config.joint_refs {
                let joint = scene.body(body).scalar_joint(joint).clone();
                angles.push(log(&joint.state_out_msg, &tasks.recorder));
                rates.push(log(&joint.state_dot_out_msg, &tasks.recorder));
            }
        }
        let csv =
            csv_stem.map(|stem| {
                let mut csv = scenario_csv(stem);
                com.connect_to(
                    csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(vector_columns(
                        "position_m",
                        "r_CN_N_m",
                    ))),
                );
                reference.connect_to(csv.add_source::<TranslationReferenceMsg>(
                    CsvSourceConfig::columns(vector_columns("position_m", "r_RN_N_m")),
                ));
                guidance.connect_to(csv.add_source::<AttitudeGuidanceMsg>(
                    CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
                ));
                csv
            });
        Self {
            position: log(&com, &tasks.outer_loop),
            reference: log(&reference, &tasks.outer_loop),
            attitude_error: log(&guidance, &tasks.outer_loop),
            on_times: fsw
                .thr_firing_round
                .module
                .thr_on_time_out_msgs
                .iter()
                .map(|output| log(output, &tasks.outer_loop))
                .collect(),
            forces: fsw
                .gateways
                .thr_force
                .iter()
                .map(|output| log(output, &tasks.recorder))
                .collect(),
            angles,
            rates,
            profile: fsw.joint_profiler.as_ref().map(|profiler| {
                log(
                    &profiler.module.des_joint_states_out_msg,
                    &tasks.joint_motion,
                )
            }),
            csv,
            outer_period: fsw.outer_period_nanos(),
            inner_period: fsw.inner_period_nanos(),
        }
    }

    fn schedule<'a>(&'a mut self, simulation: &mut Simulation<'a>) {
        let (outer, inner) = (self.outer_period, self.inner_period);
        let priority = OUTER_LOOP_RECORDER_PRIORITY;
        simulation.add_module("posRec", &mut self.position, outer, priority);
        simulation.add_module("desPosRec", &mut self.reference, outer, priority);
        simulation.add_module("attErrorRec", &mut self.attitude_error, outer, priority);
        if let Some(csv) = self.csv.as_mut() {
            // In the outer loop, which is always enabled.
            simulation.add_module("csv", csv, outer, priority);
        }
        for log in &mut self.on_times {
            simulation.add_module("onTimeRec", log, outer, priority);
        }
        for log in self
            .forces
            .iter_mut()
            .map(|log| log as &mut dyn basilisk_rs::Module)
            .chain(
                self.angles
                    .iter_mut()
                    .map(|log| log as &mut dyn basilisk_rs::Module),
            )
            .chain(
                self.rates
                    .iter_mut()
                    .map(|log| log as &mut dyn basilisk_rs::Module),
            )
        {
            simulation.add_module("recorder", log, inner, RECORDER_TASK_PRIORITY);
        }
        if let Some(profile) = self.profile.as_mut() {
            simulation.add_module("profileRec", profile, inner, JOINT_MOTION_RECORDER_PRIORITY);
        }
    }
}

/// What a run returns.
// Some fields are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    recorders: Recorders,
    has_joint_profiler: bool,
    uses_theta_penalty: bool,
    joint_state_inputs: (usize, bool),
    cost_val: f64,
}

/// Upstream `run(runTime, useJointProfiler=..., useMotionPenalty=...)`.
fn run(options: Options, run_time_s: f64, write_csv: bool) -> Results {
    let mut models = ThrArmSim::new(options.config());
    let config = models.config.clone();
    let scene = models.scene();
    let runner = models.runner();
    let stem = options.stem();
    let mut recorders = Recorders::new(&models, false, write_csv.then_some(stem.as_str()));
    {
        let mut simulation = Simulation::new(epoch(), false);
        models.schedule(&mut simulation);
        recorders.schedule(&mut simulation);
        simulation.initialize();
        configure_initial_conditions(&scene, &config);
        runner.execute(&mut simulation, sec2nano(run_time_s));
    }
    if let Some(csv) = recorders.csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    let allocation = &models.fsw.joint_thr_allocation.module;
    Results {
        has_joint_profiler: models.fsw.joint_profiler.is_some(),
        uses_theta_penalty: allocation.use_theta_penalty(),
        joint_state_inputs: (
            allocation.joint_states_in_msgs.len(),
            allocation
                .joint_states_in_msgs
                .iter()
                .all(|input| input.is_connected()),
        ),
        cost_val: allocation.cost_val,
        recorders,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_thr_arm_tracking [--run-time <s>] [--joint-profiler] [--motion-penalty] [--all]\n\n\
         Upstream run(runTime=8 h, useJointProfiler=False, useMotionPenalty=False): thruster-arm\n\
         trajectory tracking with MuJoCo dynamics. --all runs the four option combinations.",
    );
    let all = cli.switch("all");
    let run_time_s = cli.value("run-time", DEFAULT_RUN_TIME_S);
    let options = Options::new(
        cli.flag("joint-profiler", false),
        cli.flag("motion-penalty", false),
    );
    cli.finish();
    let selected: Vec<Options> = if all {
        Options::ALL.to_vec()
    } else {
        vec![options]
    };
    let directory =
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/output/scenarios");
    for options in selected {
        let results = run(options, run_time_s, true);
        let position = results.recorders.position.module.last().position_m;
        let reference = results.recorders.reference.module.last().position_m;
        println!(
            "{}: {run_time_s} s, final position error {:.3} m, |sigma_BR| {:.4}",
            options.stem(),
            (position - reference).norm(),
            results
                .recorders
                .attitude_error
                .module
                .last()
                .sigma_br
                .norm()
        );
        println!(
            "wrote {}",
            directory.join(format!("{}.csv", options.stem())).display()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use basilisk_rs::messages::{Output, ThrusterArrayForceCommandMsg};
    use thr_arm::joint_thr_allocation::WriteTimeMsg;

    /// `(times [s], values[row][thruster])` of per-thruster logs.
    fn table<T>(
        logs: &[InTasks<MessageLog<T>>],
        value: impl Fn(&T) -> f64,
    ) -> (Vec<f64>, Vec<Vec<f64>>) {
        let times = logs[0].module.times_s();
        let rows = (0..times.len())
            .map(|row| {
                logs.iter()
                    .map(|log| value(&log.module.values[row]))
                    .collect()
            })
            .collect();
        (times, rows)
    }

    fn close(a: f64, b: f64) -> bool {
        (a - b).abs() <= 1e-8 + 1e-5 * b.abs()
    }

    /// Upstream `test_thr_arm_tracking_options`, every combination: short-run
    /// tracking outcomes over two control periods and the next update.
    #[test]
    fn thr_arm_tracking_options() {
        for options in Options::ALL {
            let config = options.config();
            let (period, dt) = (config.outer_loop_period_s, config.fsw_time_step_s);
            let results = run(options, 2.0 * period + dt, false);
            assert_eq!(results.has_joint_profiler, options.joint_profiler);
            assert_eq!(results.uses_theta_penalty, options.motion_penalty);
            if options.motion_penalty {
                assert_eq!(results.joint_state_inputs, (config.num_joints, true));
            }
            let recorders = &results.recorders;
            let (outer_times, on_times) = table(&recorders.on_times, |command| command.on_time_s);
            let (force_times, forces) = table(&recorders.forces, |force| force.input);
            let max_forces = &config.thr_forces_n;

            assert!(outer_times.len() >= 3);
            for (time, expected) in outer_times.iter().zip([0.0, period, 2.0 * period]) {
                assert!((time - expected).abs() <= dt);
            }
            for on_time in on_times.iter().flatten() {
                assert!(on_time.is_finite() && *on_time >= 0.0 && *on_time <= period + dt);
                let pulses = on_time / config.pulse_resolution_s;
                assert!((pulses - pulses.round_ties_even()).abs() <= 1e-8);
            }
            assert!(
                on_times[..2].iter().flatten().any(|on_time| *on_time > 0.0),
                "{options:?}"
            );
            for row in &forces {
                for (force, max) in row.iter().zip(max_forces) {
                    assert!(force.is_finite() && (close(*force, 0.0) || close(*force, *max)));
                }
            }
            assert!(
                forces.iter().flatten().any(|force| *force > 0.0),
                "{options:?}"
            );
            assert!(
                recorders
                    .position
                    .module
                    .values
                    .iter()
                    .all(|state| state.position_m.iter().all(|value| value.is_finite()))
            );
            assert!(
                recorders
                    .attitude_error
                    .module
                    .values
                    .iter()
                    .all(|guidance| guidance.sigma_br.iter().all(|value| value.is_finite()))
            );
            for window in 0..2 {
                let (start, end) = (window as f64 * period, (window + 1) as f64 * period);
                for thr in 0..config.num_thrusters {
                    let delivered: f64 = force_times
                        .iter()
                        .zip(&forces)
                        .filter(|(time, _)| **time >= start && **time < end)
                        .map(|(_, row)| row[thr] * dt)
                        .sum();
                    let requested = max_forces[thr] * on_times[window][thr];
                    assert!(
                        delivered <= requested + 2.0 * max_forces[thr] * dt,
                        "{options:?}"
                    );
                }
            }
            if options.joint_profiler {
                let profile = &recorders
                    .profile
                    .as_ref()
                    .expect("profiler recorder")
                    .module;
                assert!(!profile.values.is_empty());
                assert!(
                    profile
                        .values
                        .iter()
                        .all(|states| states.states.iter().all(|value| value.is_finite()))
                );
            }
            if options.motion_penalty {
                assert!(results.cost_val.is_finite());
            }
        }
    }

    /// Upstream `test_thr_arm_profiled_pulse_truncation`: fixed commands
    /// (a 0.01 rad step of the first joint and full thrust) start a
    /// profiler-delayed pulse, which the next control update cancels.
    #[test]
    fn thr_arm_profiled_pulse_truncation() {
        let options = Options::new(true, false);
        let mut models = ThrArmSim::new(options.config());
        let config = models.config.clone();
        let (period, dt) = (config.outer_loop_period_s, config.fsw_time_step_s);
        let n_joint = config.num_joints;

        // The fixed test commands, connected before initialization.
        let angle_msg = Output::<JointArrayStateMsg>::default();
        let angle_time = Output::<WriteTimeMsg>::default();
        let force_msg = Output::new(ThrusterArrayForceCommandMsg::from_active(
            &config.thr_forces_n,
        ));
        angle_msg.connect_to(&mut models.fsw.des_angle_reader);
        angle_time.connect_to(&mut models.fsw.des_angle_time_reader);
        angle_msg.connect_to(
            &mut models
                .fsw
                .joint_profiler
                .as_mut()
                .expect("profiler")
                .module
                .des_joint_states_in_msg,
        );
        force_msg.connect_to(&mut models.fsw.thr_firing_round.module.thr_force_in_msg);
        let mut angles_cmd = vec![0.0; n_joint];
        angles_cmd[0] = 0.01;
        let angle_payload = JointArrayStateMsg {
            states: angles_cmd.clone(),
            state_dots: vec![0.0; n_joint],
            state_ddots: vec![0.0; n_joint],
        };
        let write_angles = |nanos: u64| {
            angle_msg.write(angle_payload.clone());
            angle_time.write(WriteTimeMsg {
                time_written_nanos: nanos,
            });
        };

        let scene = models.scene();
        let runner = models.runner();
        let mut recorders = Recorders::new(&models, true, None);
        {
            let mut simulation = Simulation::new(epoch(), false);
            models.schedule(&mut simulation);
            recorders.schedule(&mut simulation);
            simulation.initialize();
            configure_initial_conditions(&scene, &config);
            runner.execute(&mut simulation, 0);
            // After the outer loop's on-time calculation, so the event
            // latches the matching pulse.
            write_angles(sec2nano(dt));
            runner.execute(&mut simulation, sec2nano(period - dt));
            force_msg.write(ThrusterArrayForceCommandMsg::from_active(
                &vec![0.0; config.num_thrusters],
            ));
            runner.execute(&mut simulation, sec2nano(period));
            // After the outer loop calculated zero on-time.
            write_angles(sec2nano(period + dt));
            runner.execute(&mut simulation, sec2nano(2.0 * period + dt));
        }

        let (outer_times, on_times) = table(&recorders.on_times, |command| command.on_time_s);
        let (force_times, forces) = table(&recorders.forces, |force| force.input);
        assert!(outer_times.len() >= 2);
        assert!(
            on_times[0]
                .iter()
                .all(|on_time| (on_time - period).abs() <= dt),
            "{:?}",
            on_times[0]
        );
        assert!(
            on_times[1].iter().all(|on_time| on_time.abs() <= dt),
            "{:?}",
            on_times[1]
        );

        let max_forces = &config.thr_forces_n;
        let first_window: Vec<usize> = (0..force_times.len())
            .filter(|row| force_times[*row] < period)
            .collect();
        for thr in 0..config.num_thrusters {
            let delivered: f64 = first_window.iter().map(|row| forces[*row][thr] * dt).sum();
            let requested = max_forces[thr] * on_times[0][thr];
            // The firing phase was reached, and the pulse was cut short.
            assert!(delivered > 0.0);
            assert!(
                delivered < requested - max_forces[thr] * dt,
                "{delivered} vs {requested}"
            );
        }
        for (time, row) in force_times.iter().zip(&forces) {
            if *time >= period + dt {
                assert!(
                    row.iter().all(|force| close(*force, 0.0)),
                    "carryover at {time}"
                );
            }
        }
        let first_firing = *first_window
            .iter()
            .find(|row| forces[**row].iter().any(|force| *force > 0.0))
            .expect("a firing sample");
        assert!(force_times[first_firing] > 0.5 * CUBIC_PROFILE_DURATION_S);
        let largest = |logs: &[InTasks<MessageLog<ScalarJointStateMsg>>], target: &[f64]| {
            logs.iter()
                .zip(target)
                .map(|(log, want)| (log.module.values[first_firing].state - want).abs())
                .fold(0.0, f64::max)
        };
        assert!(
            largest(&recorders.angles, &angles_cmd) < 2.0 * config.joint_position_tolerance_rad
        );
        assert!(
            largest(&recorders.rates, &vec![0.0; n_joint])
                < 2.0 * config.joint_rate_tolerance_radps
        );
        let profile = &recorders
            .profile
            .as_ref()
            .expect("profiler recorder")
            .module;
        assert!(
            profile
                .values
                .iter()
                .all(|states| states.states.iter().all(|value| value.is_finite()))
        );
        assert!(profile.values.last().unwrap().states[0] > profile.values[0].states[0]);
    }
}
