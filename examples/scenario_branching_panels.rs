//! A staged, branching solar-panel deployment: port of upstream
//! `examples/mujoco/scenarioBranchingPanels.py`. Needs the `mujoco` feature.
//!
//! `sat_w_branching_panels.xml` (in `examples/data/mujoco/`) is a
//! free-floating `hub` with a central panel `panel_10`, a panel `panel_20`
//! on its end, and side panels `panel_1p`/`panel_1n` on `panel_10` and
//! `panel_2p`/`panel_2n` on `panel_20`. Every hinge gets a motor
//! (`addJointSingleActuator`) driven by upstream's C++ `JointPIDController`
//! ([`JointPidController`], gains 0.1, 0.002 and 0.0001) tracking a
//! trapezoidal profile to 0° ([`trapezoid_profile`], 0.05 °/s, 0.0001 °/s²)
//! started at its stage's time. Latches are joint constraints: a joint is
//! locked by connecting a stand-alone `ScalarJointStateMsg` to its
//! `constrainedStateInMsg`, and unlocked by disconnecting it. The
//! constraint equalities are softened (`setSolref(5, 1)`,
//! `setSolimp(0.8, 0.95, 0.001, 0.5, 2)`) to keep the problem less stiff.
//! Three 90 min legs, RKF45 with tolerances 1e-5 / 1e-6, 10 s task step:
//!
//! 1. `panel_10` and `panel_20` deploy, the side panels locked stowed (180°);
//! 2. those two are locked at 0°, and `panel_1p` and `panel_2p` deploy;
//! 3. those are locked at 0°, and `panel_1n` and `panel_2n` deploy.
//!
//! Writes `examples/output/scenarios/scenarioBranchingPanels.csv`: each
//! panel's desired and measured angle and its motor torque (upstream's
//! three recorders per panel), every step, with `extraEoMCall` so the
//! dynamics-task outputs belong to each step's final state.
//!
//! One early RKF45 trial of a whole 10 s step drives a controller command
//! past MuJoCo's sanity bound; MuJoCo logs a warning (to `MUJOCO_LOG.TXT` in
//! the working directory) and zeroes that trial's controls, and the step
//! controller rejects the trial.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! staged deployment. Upstream's `unsubscribe()` of a joint's constrained
//! input is replacing it with an unconnected input here; the softened
//! equalities and stowed angles set after `InitializeSimulation` are set on
//! the [`Shared`] scene after [`Simulation::initialize`]. Upstream's Vizard
//! call (`saveFile` commented out) is not ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use std::f64::consts::PI;
use std::time::Instant;

use basilisk_rs::connect;
use basilisk_rs::messages::{Input, Output, ScalarJointStateMsg, SingleActuatorMsg};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::interpolators::ScalarJointStateInterpolator;
use basilisk_rs::mujoco_dynamics::joint_pid_controller::JointPidController;
use basilisk_rs::simulation::Simulation;
use cli::Cli;
use common::seconds;
use message_log::MessageLog;
use mj_scenarios::{epoch, load_scene, rkf45, scalar_column, trapezoid_profile, written_csv};
use recording::scenario_csv;
use shared::Shared;

/// Upstream `dt`.
const STEP_NANOS: u64 = seconds(10);
/// Upstream `operationTime`: the length of each leg (90 min).
const LEG_NANOS: u64 = seconds(90 * 60);
/// The panels, in upstream's `addJointController` order: `(id, initial
/// angle, leg)`.
const PANELS: [(&str, f64, u64); 6] = [
    ("10", PI / 2.0, 0),
    ("20", PI, 0),
    ("1p", PI, 1),
    ("2p", PI, 1),
    ("1n", PI, 2),
    ("2n", PI, 2),
];
const FILE_STEM: &str = "scenarioBranchingPanels";

fn joint_name(panel: &str) -> String {
    format!("panel_{panel}_deploy")
}

/// One panel's recorded histories.
#[cfg_attr(not(test), allow(dead_code))]
struct PanelLog {
    id: &'static str,
    desired: Vec<(u64, ScalarJointStateMsg)>,
    measured: Vec<(u64, ScalarJointStateMsg)>,
    torque: Vec<(u64, SingleActuatorMsg)>,
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    panels: Vec<PanelLog>,
}

/// Upstream `lockJoint`: holds the joint at `angle` through a stand-alone
/// message on its `constrainedStateInMsg`.
fn lock_joint(scene: &mut MjScene, panel: &str, angle: f64) -> Output<ScalarJointStateMsg> {
    let constraint = Output::new(ScalarJointStateMsg { state: angle });
    constraint.connect_to(
        &mut scene
            .scalar_joint_mut(&joint_name(panel))
            .constrained_state_in_msg,
    );
    constraint
}

/// Upstream `unlockJoint`: `constrainedStateInMsg.unsubscribe()`.
fn unlock_joint(scene: &mut MjScene, panel: &str) {
    scene
        .scalar_joint_mut(&joint_name(panel))
        .constrained_state_in_msg = Input::default();
}

fn run(write_csv: bool) -> Results {
    let mut scene = load_scene("sat_w_branching_panels.xml");
    scene.set_integrator(rkf45(1e-5, 1e-6));
    scene.extra_eom_call = true;

    // Upstream `addJointController`.
    let mut taps = Vec::new();
    for (panel, initial, leg) in PANELS {
        let joint = joint_name(panel);
        scene.add_joint_single_actuator(&joint, &joint);
        let (positions, velocities) = trapezoid_profile(
            initial,
            0.0,
            0.05_f64.to_radians(),
            0.0001_f64.to_radians(),
            (leg * LEG_NANOS) as f64,
        );
        let mut position = ScalarJointStateInterpolator::default();
        position
            .set_data_points(&positions, 1)
            .expect("valid profile");
        let mut velocity = ScalarJointStateInterpolator::default();
        velocity
            .set_data_points(&velocities, 1)
            .expect("valid profile");
        let mut controller = JointPidController::default();
        controller.name = format!("{joint}_controller");
        controller.set_proportional_gain(0.1);
        controller.set_derivative_gain(0.002);
        controller.set_integral_gain(0.0001);
        position
            .interpolated_out_msg
            .connect_to(&mut controller.desired_pos_in_msg);
        velocity
            .interpolated_out_msg
            .connect_to(&mut controller.desired_vel_in_msg);
        let (joint_state, joint_rate) = {
            let joint = scene.scalar_joint(&joint);
            (joint.state_out_msg.clone(), joint.state_dot_out_msg.clone())
        };
        joint_state.connect_to(&mut controller.measured_pos_in_msg);
        joint_rate.connect_to(&mut controller.measured_vel_in_msg);
        controller
            .output_out_msg
            .connect_to(&mut scene.single_actuator_mut(&joint).actuator_in_msg);
        taps.push((
            panel,
            position.interpolated_out_msg.clone(),
            joint_state,
            controller.output_out_msg.clone(),
        ));
        scene.add_model_to_dynamics_task(position, 50);
        scene.add_model_to_dynamics_task(velocity, 49);
        scene.add_model_to_dynamics_task(controller, 25);
    }
    // The side panels start latched stowed.
    let mut latches = Vec::new();
    for panel in ["1p", "1n", "2p", "2n"] {
        latches.push(lock_joint(&mut scene, panel, PI));
    }

    let mut desired_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        taps.iter().map(|_| MessageLog::default()).collect();
    let mut measured_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        taps.iter().map(|_| MessageLog::default()).collect();
    let mut torque_logs: Vec<MessageLog<SingleActuatorMsg>> =
        taps.iter().map(|_| MessageLog::default()).collect();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        simulation.add_module("scene", &mut scene_task, STEP_NANOS, 10);
        for (index, ((_, desired, measured, torque), ((desired_log, measured_log), torque_log))) in
            taps.iter()
                .zip(
                    desired_logs
                        .iter_mut()
                        .zip(measured_logs.iter_mut())
                        .zip(torque_logs.iter_mut()),
                )
                .enumerate()
        {
            connect!(&simulation,
                desired => &mut desired_log.input,
                measured => &mut measured_log.input,
                torque => &mut torque_log.input,
            );
            simulation.add_module(format!("desired{index}"), desired_log, STEP_NANOS, 0);
            simulation.add_module(format!("measured{index}"), measured_log, STEP_NANOS, 0);
            simulation.add_module(format!("torque{index}"), torque_log, STEP_NANOS, 0);
        }
        if let Some(csv) = csv.as_mut() {
            for (panel, desired, measured, torque) in &taps {
                connect!(&simulation,
                    desired => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("panel_{panel}_desired_rad")),
                    ),
                    measured => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("panel_{panel}_measured_rad")),
                    ),
                    torque => csv.add_source::<SingleActuatorMsg>(
                        scalar_column("input", &format!("panel_{panel}_torque_Nm")),
                    ),
                );
            }
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            for (panel, initial, _) in PANELS {
                // Softer latches: a 5 s time constant, lower impedance.
                let equality = scene
                    .scalar_joint(&joint_name(panel))
                    .constrained_equality()
                    .to_string();
                scene.set_equality_solref(&equality, 5.0, 1.0);
                scene.set_equality_solimp(&equality, 0.8, 0.95, 0.001, 0.5, 2.0);
                // The stowed configuration.
                scene
                    .body_mut(&format!("panel_{panel}"))
                    .set_scalar_joint_position(&joint_name(panel), initial);
            }
        }

        let started = Instant::now();
        simulation.run_for(LEG_NANOS);
        println!(
            "Run leg 1 in {:.3} seconds",
            started.elapsed().as_secs_f64()
        );

        {
            let mut scene = scene.lock();
            latches.push(lock_joint(&mut scene, "10", 0.0));
            latches.push(lock_joint(&mut scene, "20", 0.0));
            unlock_joint(&mut scene, "1p");
            unlock_joint(&mut scene, "2p");
        }
        let started = Instant::now();
        simulation.run_for(LEG_NANOS);
        println!(
            "Run leg 2 in {:.3} seconds",
            started.elapsed().as_secs_f64()
        );

        {
            let mut scene = scene.lock();
            latches.push(lock_joint(&mut scene, "1p", 0.0));
            latches.push(lock_joint(&mut scene, "2p", 0.0));
            unlock_joint(&mut scene, "1n");
            unlock_joint(&mut scene, "2n");
        }
        let started = Instant::now();
        simulation.run_for(LEG_NANOS);
        println!(
            "Run leg 3 in {:.3} seconds",
            started.elapsed().as_secs_f64()
        );
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    drop(latches);
    Results {
        panels: taps
            .into_iter()
            .zip(desired_logs.into_iter().zip(measured_logs).zip(torque_logs))
            .map(|((id, ..), ((desired, measured), torque))| PanelLog {
                id,
                desired: desired.into_samples(),
                measured: measured.into_samples(),
                torque: torque.into_samples(),
            })
            .collect(),
    }
}

fn main() {
    Cli::from_env("scenario_branching_panels").finish();
    let results = run(true);
    for panel in &results.panels {
        println!(
            "panel {}: final angle {:.4} deg",
            panel.id,
            panel
                .measured
                .last()
                .expect("recorded angles")
                .1
                .state
                .to_degrees()
        );
    }
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! staged deployment it demonstrates.

    use super::*;

    #[test]
    fn panels_deploy_leg_by_leg() {
        let results = run(false);
        let angle_at = |panel: &PanelLog, nanos: u64| {
            panel
                .measured
                .iter()
                .find(|(time, _)| *time == nanos)
                .map(|(_, state)| state.state.to_degrees())
                .unwrap()
        };
        for panel in &results.panels {
            assert_eq!(panel.measured.len(), 3 * 540 + 1);
            let leg = PANELS.iter().find(|(id, ..)| *id == panel.id).unwrap().2;
            let initial = PANELS
                .iter()
                .find(|(id, ..)| *id == panel.id)
                .unwrap()
                .1
                .to_degrees();
            // Latched (or not yet started) before its leg: still stowed.
            if leg > 0 {
                let before = angle_at(panel, leg * LEG_NANOS);
                assert!((before - initial).abs() < 2.0, "{}: {before}", panel.id);
            }
            // Deployed by the end of its leg, and held there (latched)
            // through the later legs.
            let after = angle_at(panel, (leg + 1) * LEG_NANOS);
            assert!(
                after.abs() < 1.0,
                "{}: {after} at the end of leg {}",
                panel.id,
                leg + 1
            );
            let last = panel.measured.last().unwrap().1.state.to_degrees();
            assert!(last.abs() < 1.0, "{}: final {last}", panel.id);
            assert!(
                panel
                    .torque
                    .iter()
                    .all(|(_, torque)| torque.input.is_finite())
            );
        }
    }
}
