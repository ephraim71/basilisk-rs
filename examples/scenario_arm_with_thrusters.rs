//! A spacecraft deploying two thrusters on a branching four-link arm: port
//! of upstream `examples/mujoco/scenarioArmWithThrusters.py`. Needs the
//! `mujoco` feature.
//!
//! `sat_w_deployable_thruster.xml` (in `examples/data/mujoco/`) is a
//! free-floating `hub` with an arm `arm_1` (roll and elevation hinges),
//! `arm_2`, and two end links `arm_3` and `arm_4` carrying the thruster
//! sites `thruster_1` and `thruster_2`. The arm joints are not actuated but
//! constrained: a degree-1 `ScalarJointStateInterpolator` per joint, in the
//! scene's dynamics task, feeds its `constrainedStateInMsg`, so MuJoCo
//! computes the joint forces that make the joints follow the linear
//! deployment profiles. Degree-0 (piecewise constant)
//! `SingleActuatorInterpolator`s switch the 2 N thrusters on and off. The
//! sequence (end times): `arm_1` deploys (5 s), `arm_2` (15 s), `arm_3`
//! (20 s), first thrust (52.5 s), `arm_1` rolls 180° (62.5 s), second
//! thrust (92.5 s), roll back to 90° (97.5 s), `arm_4` deploys (102.5 s),
//! both thrusters fire until 132.5 s. RKF45 with tolerances 1e-3, 0.1 s
//! task step.
//!
//! Writes `examples/output/scenarios/scenarioArmWithThrusters.csv`: the hub
//! origin state (upstream `bodyStateRecorder`) and the five joint angles,
//! every step.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks that
//! the joints follow their profiles and that the thrust moves the hub.
//! Upstream sets the initial joint angles on the scene after
//! `InitializeSimulation`; here the scene is [`Shared`] and set after
//! [`Simulation::initialize`]. Upstream's Vizard call and its thruster-plume
//! writers (`ThrusterVizMessageWriter`, Vizard only, `saveFile` commented
//! out) are not ported.

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

use basilisk_rs::messages::{ScalarJointStateMsg, SpacecraftStateMsg};
use basilisk_rs::mujoco_dynamics::interpolators::{
    ScalarJointStateInterpolator, SingleActuatorInterpolator,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use message_log::MessageLog;
use mj_scenarios::{epoch, load_scene, rkf45, scalar_column, site_state_columns, written_csv};
use recording::scenario_csv;
use shared::Shared;

/// Upstream `dt` \[ns\].
const STEP_NANOS: u64 = 100_000_000;
const FILE_STEM: &str = "scenarioArmWithThrusters";

// Upstream's phase end times [s].
const START: f64 = 0.0;
const ARM_1_DEPLOY: f64 = 5.0;
const ARM_2_DEPLOY: f64 = ARM_1_DEPLOY + 10.0;
const ARM_3_DEPLOY: f64 = ARM_2_DEPLOY + 5.0;
const FIRST_THRUST: f64 = ARM_3_DEPLOY + 32.5;
const FIRST_ROLL: f64 = FIRST_THRUST + 10.0;
const SECOND_THRUST: f64 = FIRST_ROLL + 30.0;
const SECOND_ROLL: f64 = SECOND_THRUST + 5.0;
const ARM_4_DEPLOY: f64 = SECOND_ROLL + 5.0;
const THIRD_THRUST: f64 = ARM_4_DEPLOY + 30.0;
const END: f64 = THIRD_THRUST;

/// Upstream `macros.sec2nano`, as the interpolators' time column.
fn nanos(seconds: f64) -> f64 {
    (seconds * 1e9).round()
}

/// Upstream `ACTUATOR_INTERPOLATION_POINTS`: `(time [ns], thrust [N])`.
fn actuator_points() -> Vec<(&'static str, Vec<(f64, f64)>)> {
    vec![
        (
            "thruster_1",
            vec![
                (nanos(0.0), 0.0),
                (nanos(ARM_3_DEPLOY), 2.0),
                (nanos(FIRST_THRUST), 0.0),
                (nanos(FIRST_ROLL), 2.0),
                (nanos(SECOND_THRUST), 0.0),
                (nanos(SECOND_ROLL), 2.0),
            ],
        ),
        (
            "thruster_2",
            vec![(nanos(0.0), 0.0), (nanos(ARM_4_DEPLOY), 2.0)],
        ),
    ]
}

/// Upstream `JOINT_INTERPOLATION_POINTS` with `BODY_OF_JOINT`: `(joint,
/// body, (time [ns], angle [rad]) points)`, in upstream's dictionary
/// order.
/// A joint, its body and its `(time [ns], angle [rad])` profile.
type JointProfile = (&'static str, &'static str, Vec<(f64, f64)>);

fn joint_points() -> Vec<JointProfile> {
    let deg = f64::to_radians;
    vec![
        (
            "arm_1_elev",
            "arm_1",
            vec![(nanos(START), deg(0.0)), (nanos(ARM_1_DEPLOY), deg(45.0))],
        ),
        (
            "arm_2_elev",
            "arm_2",
            vec![
                (nanos(ARM_1_DEPLOY), deg(180.0)),
                (nanos(ARM_2_DEPLOY), deg(74.93)),
            ],
        ),
        (
            "arm_3_elev",
            "arm_3",
            vec![
                (nanos(ARM_2_DEPLOY), deg(180.0)),
                (nanos(ARM_3_DEPLOY), deg(60.06941955)),
            ],
        ),
        (
            "arm_1_roll",
            "arm_1",
            vec![
                (nanos(FIRST_THRUST), deg(0.0)),
                (nanos(FIRST_ROLL), deg(180.0)),
                (nanos(SECOND_THRUST), deg(180.0)),
                (nanos(SECOND_ROLL), deg(90.0)),
            ],
        ),
        (
            "arm_4_elev",
            "arm_4",
            vec![
                (nanos(SECOND_ROLL), deg(180.0)),
                (nanos(ARM_4_DEPLOY), deg(90.0)),
            ],
        ),
    ]
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    hub: Vec<(u64, SpacecraftStateMsg)>,
    /// Each joint's angle history, in [`joint_points`] order.
    joints: Vec<(&'static str, Vec<(u64, ScalarJointStateMsg)>)>,
}

fn run(write_csv: bool) -> Results {
    let mut scene = load_scene("sat_w_deployable_thruster.xml");
    scene.set_integrator(rkf45(1e-3, 1e-3));

    // The piecewise-constant thrust profiles, evaluated at every integrator
    // stage.
    for (thruster, points) in actuator_points() {
        let mut interpolator = SingleActuatorInterpolator::default();
        interpolator
            .set_data_points(&points, 0)
            .expect("valid thrust profile");
        interpolator
            .interpolated_out_msg
            .connect_to(&mut scene.single_actuator_mut(thruster).actuator_in_msg);
        scene.add_model_to_dynamics_task(interpolator, -1);
    }
    // The linear joint profiles, held by the joints' constraints.
    for (joint, body, points) in joint_points() {
        let mut interpolator = ScalarJointStateInterpolator::default();
        interpolator
            .set_data_points(&points, 1)
            .expect("valid joint profile");
        interpolator.interpolated_out_msg.connect_to(
            &mut scene
                .body_mut(body)
                .scalar_joint_mut(joint)
                .constrained_state_in_msg,
        );
        scene.add_model_to_dynamics_task(interpolator, -1);
    }

    let hub_state = scene.body("hub").origin().state_out_msg.clone();
    let joint_states: Vec<_> = joint_points()
        .into_iter()
        .map(|(joint, _, _)| (joint, scene.scalar_joint(joint).state_out_msg.clone()))
        .collect();
    let mut hub_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut joint_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        joint_states.iter().map(|_| MessageLog::default()).collect();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));

    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation, &hub_state => &mut hub_log.input);
        for ((_, output), log) in joint_states.iter().zip(joint_logs.iter_mut()) {
            connect!(&simulation, output => &mut log.input);
        }
        schedule! { simulation,
            "scene" => &mut scene_task, period=STEP_NANOS, priority=10;
            "bodyStateRecorder" => &mut hub_log, period=STEP_NANOS, priority=0;
        }
        for (log, (joint, _)) in joint_logs.iter_mut().zip(&joint_states) {
            simulation.add_module(format!("{joint}Recorder"), log, STEP_NANOS, 0);
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
            );
            for (joint, output) in &joint_states {
                connect!(&simulation,
                    output => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("{joint}_rad")),
                    ),
                );
            }
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.initialize();
        // The initial joint angles: the first point of each profile.
        {
            let mut scene = scene.lock();
            for (joint, body, points) in joint_points() {
                scene
                    .body_mut(body)
                    .set_scalar_joint_position(joint, points[0].1);
            }
        }
        simulation.run_for(nanos(END) as u64);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        hub: hub_log.into_samples(),
        joints: joint_states
            .iter()
            .map(|(joint, _)| *joint)
            .zip(joint_logs.into_iter().map(MessageLog::into_samples))
            .collect(),
    }
}

fn main() {
    Cli::from_env("scenario_arm_with_thrusters").finish();
    let results = run(true);
    let (_, last) = results.hub.last().expect("recorded hub states");
    println!(
        "final hub velocity {:.4?} m/s, sigma_BN {:.4?}",
        last.velocity_mps, last.sigma_bn
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check that the
    //! constrained joints follow their profiles and that the thrust moves
    //! the hub.

    use super::*;

    #[test]
    fn joints_follow_their_profiles_and_the_thrust_moves_the_hub() {
        let results = run(false);
        assert_eq!(results.hub.len(), 1326);
        let profiles = joint_points();
        for ((joint, log), (_, _, points)) in results.joints.iter().zip(&profiles) {
            let mut interpolator = ScalarJointStateInterpolator::default();
            interpolator.set_data_points(points, 1).unwrap();
            let worst = log
                .iter()
                .map(|(time, state)| (state.state - interpolator.value_at(*time)).abs())
                .fold(0.0, f64::max);
            // Soft constraints (MuJoCo's default 0.02 s time constant): a
            // lag of about a degree at the end of the fastest ramps.
            assert!(worst < 1.5_f64.to_radians(), "{joint}: {worst} rad");
        }
        // Before the first thrust only the arm moves the hub (about the fixed
        // system center of mass); then the thrusters push it and turn it.
        let at = |seconds: f64| {
            results
                .hub
                .iter()
                .find(|(time, _)| *time == nanos(seconds) as u64)
                .map(|(_, state)| state.clone())
                .unwrap()
        };
        let (_, last) = results.hub.last().unwrap();
        assert!(last.velocity_mps.norm() > 5.0 * at(ARM_3_DEPLOY).velocity_mps.norm());
        assert!(last.velocity_mps.norm() > 1e-2, "{}", last.velocity_mps);
        assert!((last.sigma_bn - at(ARM_3_DEPLOY).sigma_bn).norm() > 1e-2);
    }
}
