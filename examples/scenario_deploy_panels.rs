//! Six solar panels deployed by analog PID controllers: port of upstream
//! `examples/mujoco/scenarioDeployPanels.py`. Needs the `mujoco` feature.
//!
//! `sat_w_deployable_panels.xml` (in `examples/data/mujoco/`) is a
//! free-floating `hub` with two three-panel wings (`panel_p1..3`,
//! `panel_n1..3`) on hinges with joint limits and motors. Each joint
//! follows a trapezoidal velocity profile from stowed (90° for the inner
//! panels, 180° for the others) to 0° at 0.05 °/s and 0.0001 °/s²
//! (upstream `generateProfiles`, [`trapezoid_profile`]), fed as position
//! and velocity references by degree-1 interpolators to upstream's Python
//! `PIDController` (a `StatefulSysModel` whose integral error is a scene
//! state, here [`PidController`]), all in the scene's dynamics task. With
//! `extraEoMCall` the scene re-evaluates that task at each step's final
//! state, so the recorded references and torques match the recorded joint
//! angles. RKF45 with tolerances 1e-4, 1 s task step, 80 min.
//!
//! `--initial-spin` gives the hub an initial 0.8 rad/s about y (upstream
//! `run(initialSpin=True)`); upstream's test runs the default only, which
//! `--all` runs too.
//!
//! Writes `examples/output/scenarios/scenarioDeployPanels.csv` (or
//! `scenarioDeployPanelsInitialSpin.csv`): each panel's desired and
//! measured angle and its motor torque (upstream's three recorders per
//! panel), every step.
//!
//! `test_scenarioMujoco.py` only checks that the scenario runs; `tests`
//! checks that every panel deploys, within its joint limits (upstream's low
//! gains leave the panels lagging their references by tens of degrees).
//! Upstream sets the stowed angles after `InitializeSimulation`; here the
//! scene is [`Shared`] and set after [`Simulation::initialize`]. Upstream's
//! Vizard call (`saveFile` commented out) is not ported.

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
use basilisk_rs::messages::{ScalarJointStateMsg, SingleActuatorMsg};
use basilisk_rs::mujoco_dynamics::interpolators::ScalarJointStateInterpolator;
use basilisk_rs::simulation::Simulation;
use cli::Cli;
use common::seconds;
use message_log::MessageLog;
use mj_scenarios::{
    PidController, epoch, load_scene, rkf45, scalar_column, trapezoid_profile, written_csv,
};
use nalgebra::Vector3;
use recording::scenario_csv;
use shared::Shared;

const USAGE: &str = "\
scenario_deploy_panels [--initial-spin]
scenario_deploy_panels --all     run upstream's tested case";

/// Upstream `dt`.
const STEP_NANOS: u64 = seconds(1);
/// Upstream `tf` (80 min).
const DURATION_NANOS: u64 = seconds(80 * 60);
/// Upstream `JOINT_START_END` \[rad\], by panel tier.
const JOINT_START_END: [(f64, f64); 3] = [(PI / 2.0, 0.0), (PI, 0.0), (PI, 0.0)];
/// Upstream `MAX_PROFILE_VEL` (0.05 °/s) \[rad/s\].
fn max_profile_velocity() -> f64 {
    0.05_f64.to_radians()
}
/// Upstream `MAX_PROFILE_ACCEL` (0.0001 °/s²) \[rad/s²\].
fn max_profile_acceleration() -> f64 {
    0.0001_f64.to_radians()
}
/// Upstream `panelIds`.
const PANEL_IDS: [(char, usize); 6] = [('p', 1), ('p', 2), ('p', 3), ('n', 1), ('n', 2), ('n', 3)];

/// Upstream `run(initialSpin, showPlots)`.
#[derive(Clone, Copy, Debug)]
struct Config {
    initial_spin: bool,
}

impl Config {
    fn file_stem(&self) -> &'static str {
        if self.initial_spin {
            "scenarioDeployPanelsInitialSpin"
        } else {
            "scenarioDeployPanels"
        }
    }
}

/// One panel's recorded histories.
#[cfg_attr(not(test), allow(dead_code))]
struct PanelLog {
    name: String,
    desired: Vec<(u64, ScalarJointStateMsg)>,
    measured: Vec<(u64, ScalarJointStateMsg)>,
    torque: Vec<(u64, SingleActuatorMsg)>,
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    panels: Vec<PanelLog>,
}

fn run(config: Config, write_csv: bool) -> Results {
    let mut scene = load_scene("sat_w_deployable_panels.xml");
    scene.set_integrator(rkf45(1e-4, 1e-4));
    // Re-evaluate the dynamics task at the final state of each step, so
    // the recorded references and torques belong to the recorded state.
    scene.extra_eom_call = true;

    let mut taps = Vec::new();
    for (side, tier) in PANEL_IDS {
        let name = format!("panel_{side}{tier}");
        let joint = format!("{name}_deploy");
        let (start, end) = JOINT_START_END[tier - 1];
        let (positions, velocities) = trapezoid_profile(
            start,
            end,
            max_profile_velocity(),
            max_profile_acceleration(),
            0.0,
        );
        let mut position = ScalarJointStateInterpolator::default();
        position
            .set_data_points(&positions, 1)
            .expect("valid profile");
        let mut velocity = ScalarJointStateInterpolator::default();
        velocity
            .set_data_points(&velocities, 1)
            .expect("valid profile");

        let mut controller = PidController::new(format!("{joint}_controller"));
        position
            .interpolated_out_msg
            .connect_to(&mut controller.desired_in_msg);
        velocity
            .interpolated_out_msg
            .connect_to(&mut controller.desired_dot_in_msg);
        let (joint_state, joint_rate) = {
            let joint = scene.scalar_joint(&joint);
            (joint.state_out_msg.clone(), joint.state_dot_out_msg.clone())
        };
        joint_state.connect_to(&mut controller.measured_in_msg);
        joint_rate.connect_to(&mut controller.measured_dot_in_msg);
        controller
            .output_out_msg
            .connect_to(&mut scene.single_actuator_mut(&joint).actuator_in_msg);

        taps.push((
            name,
            position.interpolated_out_msg.clone(),
            joint_state,
            controller.output_out_msg.clone(),
        ));
        // The controller after the interpolators.
        scene.add_model_to_dynamics_task(position, 50);
        scene.add_model_to_dynamics_task(velocity, 49);
        scene.add_model_to_dynamics_task(controller, 25);
    }

    let mut desired_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        taps.iter().map(|_| MessageLog::default()).collect();
    let mut measured_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        taps.iter().map(|_| MessageLog::default()).collect();
    let mut torque_logs: Vec<MessageLog<SingleActuatorMsg>> =
        taps.iter().map(|_| MessageLog::default()).collect();
    let mut csv = write_csv.then(|| scenario_csv(config.file_stem()));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    let started = Instant::now();
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
            for (name, desired, measured, torque) in &taps {
                connect!(&simulation,
                    desired => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("{name}_desired_rad")),
                    ),
                    measured => csv.add_source::<ScalarJointStateMsg>(
                        scalar_column("state", &format!("{name}_measured_rad")),
                    ),
                    torque => csv.add_source::<SingleActuatorMsg>(
                        scalar_column("input", &format!("{name}_torque_Nm")),
                    ),
                );
            }
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            // The stowed angles.
            for (side, tier) in PANEL_IDS {
                let name = format!("panel_{side}{tier}");
                scene.body_mut(&name).set_scalar_joint_position(
                    &format!("{name}_deploy"),
                    JOINT_START_END[tier - 1].0,
                );
            }
            if config.initial_spin {
                scene
                    .body_mut("hub")
                    .set_attitude_rate(Vector3::new(0.0, 0.8, 0.0));
            }
        }
        simulation.run_for(DURATION_NANOS);
    }
    println!("Run sim in {:.3} seconds", started.elapsed().as_secs_f64());
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        panels: taps
            .into_iter()
            .zip(desired_logs.into_iter().zip(measured_logs).zip(torque_logs))
            .map(|((name, ..), ((desired, measured), torque))| PanelLog {
                name,
                desired: desired.into_samples(),
                measured: measured.into_samples(),
                torque: torque.into_samples(),
            })
            .collect(),
    }
}

fn main() {
    let mut cli = Cli::from_env(USAGE);
    let configs = if cli.switch("all") {
        vec![Config {
            initial_spin: false,
        }]
    } else {
        vec![Config {
            initial_spin: cli.flag("initial-spin", false),
        }]
    };
    cli.finish();
    for config in configs {
        let results = run(config, true);
        for panel in &results.panels {
            println!(
                "{}: final angle {:.4} deg",
                panel.name,
                panel
                    .measured
                    .last()
                    .expect("recorded angles")
                    .1
                    .state
                    .to_degrees()
            );
        }
        println!("wrote {}", written_csv(config.file_stem()).display());
    }
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario (default
    //! parameters). These check the deployment it demonstrates.

    use super::*;

    #[test]
    fn every_panel_deploys_within_its_joint_limits() {
        let results = run(
            Config {
                initial_spin: false,
            },
            false,
        );
        for panel in &results.panels {
            assert_eq!(panel.measured.len(), 4801);
            assert_eq!(panel.desired.len(), 4801);
            // The references reach 0° after 38 min (90°) and 68 min (180°).
            let (_, reference) = panel.desired.last().unwrap();
            assert!(reference.state.abs() < 1e-12);
            // Upstream's gains are low next to a wing's inertia (a 1.2 t,
            // 6 m wing on the inner hinge): the panels lag their
            // references by tens of degrees and swing about 0°, bouncing
            // off the joint stop, but end close to deployed.
            let (_, last) = panel.measured.last().unwrap();
            assert!(
                last.state.to_degrees() < 15.0,
                "{}: {}",
                panel.name,
                last.state
            );
            let worst = panel
                .desired
                .iter()
                .zip(&panel.measured)
                .map(|((_, desired), (_, measured))| (desired.state - measured.state).abs())
                .fold(0.0, f64::max);
            assert!(worst < 50.0_f64.to_radians(), "{}: {worst} rad", panel.name);
            // Within the joint limits ([0, 90°] or [0, 180°]), up to the
            // limit constraint's softness.
            let tier_max = if panel.name.ends_with('1') {
                90.0
            } else {
                180.0
            };
            for (_, measured) in &panel.measured {
                let angle = measured.state.to_degrees();
                assert!(
                    angle > -0.5 && angle < tier_max + 0.5,
                    "{}: {angle}",
                    panel.name
                );
            }
            assert!(
                panel
                    .torque
                    .iter()
                    .all(|(_, torque)| torque.input.is_finite())
            );
        }
    }
}
