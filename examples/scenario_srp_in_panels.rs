//! Solar-radiation pressure on deploying panels: port of upstream
//! `examples/mujoco/scenarioSRPInPanels.py`. Needs the `mujoco` feature.
//!
//! The spacecraft of `scenarioDeployPanels` (`sat_w_deployable_panels.xml`
//! in `examples/data/mujoco/`), whose six panels are now constrained to
//! linear profiles from stowed to deployed over 60 s (degree-1
//! interpolators on the joints' `constrainedStateInMsg`). Upstream's Python
//! `SRPPanel` model, here [`SrpPanel`], computes for each panel the
//! flat-plate pressure force `C_SRP cos² θ`, `θ` between the sunlight
//! (fixed along inertial -z) and the normal of the panel's `centroid` site,
//! at every integrator stage, and drives the panel's single actuator along
//! that normal. RKF45 with tolerances 1e-3, `extraEoMCall` so the recorded
//! forces belong to each step's final state.
//!
//! Upstream changes the task period with `task.TaskData.updatePeriod`:
//! 1 s to 59 s, then 0.025 s, then 1 s again from 61 s to 70 s. Its
//! next-start rule (`SysModelTask::updatePeriod`) makes the task run at
//! 0, 1, ..., 59 s, at 60, 60.025, ..., 61 s and at 62, ..., 70 s. Here the
//! s are scheduled at 0.025 s and run only at those times
//! ([`OnSchedule`]); the scene integrates from its last update, so its
//! steps are upstream's.
//!
//! `--initial-spin` gives the hub an initial 0.8 rad/s about y (upstream
//! `run(initialSpin=True)`); upstream's test runs the default only, which
//! `--all` runs too.
//!
//! Writes `examples/output/scenarios/scenarioSRPInPanels.csv` (or
//! `scenarioSRPInPanelsInitialSpin.csv`): each panel's SRP force and the
//! hub origin state, at every task run.
//!
//! `test_scenarioMujoco.py` only checks that the scenario runs; `tests`
//! checks the forces against the panel geometry and the push they give
//! the spacecraft. Upstream's Vizard call (`saveFile` commented out) is
//! not ported.

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
use std::sync::Arc;
use std::time::Instant;

use basilisk_rs::messages::{Input, Output, SingleActuatorMsg, SpacecraftStateMsg};
use basilisk_rs::mujoco_dynamics::interpolators::ScalarJointStateInterpolator;
use basilisk_rs::mujoco_dynamics::{SceneModel, TaskContext};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{Module, SimulationContext, connect};
use cli::Cli;
use message_log::MessageLog;
use mj_scenarios::{epoch, load_scene, rkf45, scalar_column, site_state_columns, written_csv};
use nalgebra::Vector3;
use recording::scenario_csv;
use shared::Shared;

const USAGE: &str = "\
scenario_srp_in_panels [--initial-spin]
scenario_srp_in_panels --all     run upstream's tested case";

/// The finest task period, 0.025 s \[ns\].
const BASE_TICK_NANOS: u64 = 25_000_000;
const SECOND: u64 = 1_000_000_000;
/// Upstream `JOINT_PROFILES` \[rad\] over 60 s, by panel tier.
const JOINT_PROFILES: [(f64, f64); 3] = [(PI / 2.0, 0.0), (PI, 0.0), (PI, 0.0)];
/// Upstream `panelIds`.
const PANEL_IDS: [(char, usize); 6] = [('p', 1), ('p', 2), ('p', 3), ('n', 1), ('n', 2), ('n', 3)];

/// Upstream's Python `SRPPanel(sysModel.SysModel)`: the pressure force on a
/// flat plate, `srpFactor cos² θ`, along the plate normal.
#[derive(Debug)]
pub struct SrpPanel {
    pub name: String,
    /// Upstream `srpFactor` \[N\]: the force with the sunlight along the
    /// normal.
    pub srp_factor: f64,
    /// Upstream `normalVector_B`: the plate normal in the site frame.
    pub normal_vector_s: Vector3<f64>,
    /// Upstream `sunlightDirection_N`.
    pub sunlight_direction_n: Vector3<f64>,
    /// Upstream `frameInMsg`: the plate's site state.
    pub frame_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `forceOutMsg`: the force magnitude \[N\].
    pub force_out_msg: Output<SingleActuatorMsg>,
}

impl SceneModel for SrpPanel {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        let dcm_ns = mrp_to_dcm(self.frame_in_msg.read().sigma_bn).transpose();
        let normal_n = dcm_ns * self.normal_vector_s;
        let cos_angle = (-normal_n).dot(&self.sunlight_direction_n);
        self.force_out_msg.write(SingleActuatorMsg {
            input: self.srp_factor * cos_angle * cos_angle,
        });
    }
}

/// Upstream's task times: `updatePeriod`'s next-start rule applied to the
/// period changes at 59 s and 61 s.
fn task_times() -> Vec<u64> {
    let mut times: Vec<u64> = (0..=59).map(|second| second * SECOND).collect();
    times.extend((0..=40).map(|tick| 60 * SECOND + tick * BASE_TICK_NANOS));
    times.extend((62..=70).map(|second| second * SECOND));
    times
}

/// A module of upstream's re-periodized task: scheduled at the base tick,
/// it updates only at the task's times.
struct OnSchedule<M> {
    module: M,
    times: Arc<Vec<u64>>,
}

impl<M> OnSchedule<M> {
    fn new(module: M, times: &Arc<Vec<u64>>) -> Self {
        Self {
            module,
            times: Arc::clone(times),
        }
    }
}

impl<M: Module> Module for OnSchedule<M> {
    fn init(&mut self) {
        self.module.init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.module.reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        if self.times.binary_search(&context.current_sim_nanos).is_ok() {
            self.module.update(context);
        }
    }
}

/// Upstream `run(initialSpin, showPlots)`.
#[derive(Clone, Copy, Debug)]
struct Config {
    initial_spin: bool,
}

impl Config {
    fn file_stem(&self) -> &'static str {
        if self.initial_spin {
            "scenarioSRPInPanelsInitialSpin"
        } else {
            "scenarioSRPInPanels"
        }
    }
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Each panel's SRP force history, in [`PANEL_IDS`] order.
    forces: Vec<Vec<(u64, SingleActuatorMsg)>>,
    hub: Vec<(u64, SpacecraftStateMsg)>,
}

fn run(config: Config, write_csv: bool) -> Results {
    let mut scene = load_scene("sat_w_deployable_panels.xml");
    scene.set_integrator(rkf45(1e-3, 1e-3));
    scene.extra_eom_call = true;

    // The joints follow linear profiles from stowed to deployed.
    for (side, tier) in PANEL_IDS {
        let (start, end) = JOINT_PROFILES[tier - 1];
        let mut interpolator = ScalarJointStateInterpolator::default();
        interpolator
            .set_data_points(&[(0.0, start), ((60 * SECOND) as f64, end)], 1)
            .expect("valid profile");
        interpolator.interpolated_out_msg.connect_to(
            &mut scene
                .scalar_joint_mut(&format!("panel_{side}{tier}_deploy"))
                .constrained_state_in_msg,
        );
        scene.add_model_to_dynamics_task(interpolator, -1);
    }
    // SRP on the sunlit side of each panel.
    let mut forces = Vec::new();
    for (side, tier) in PANEL_IDS {
        let mut srp = SrpPanel {
            name: format!("SRP_{side}{tier}"),
            srp_factor: 1.0,
            normal_vector_s: Vector3::new(0.0, 0.0, 1.0),
            sunlight_direction_n: Vector3::new(0.0, 0.0, -1.0),
            frame_in_msg: Input::default(),
            force_out_msg: Output::default(),
        };
        scene
            .site(&format!("panel_{side}{tier}_centroid"))
            .state_out_msg
            .connect_to(&mut srp.frame_in_msg);
        srp.force_out_msg.connect_to(
            &mut scene
                .single_actuator_mut(&format!("panel_{side}{tier}_srp"))
                .actuator_in_msg,
        );
        forces.push(srp.force_out_msg.clone());
        scene.add_model_to_dynamics_task(srp, -1);
    }
    let hub_state = scene.body("hub").origin().state_out_msg.clone();

    let times = Arc::new(task_times());
    let mut force_logs: Vec<_> = forces
        .iter()
        .map(|_| OnSchedule::new(MessageLog::<SingleActuatorMsg>::default(), &times))
        .collect();
    let mut hub_log = OnSchedule::new(MessageLog::<SpacecraftStateMsg>::default(), &times);
    let mut csv = write_csv.then(|| OnSchedule::new(scenario_csv(config.file_stem()), &times));
    let scene = Shared::new(scene);
    let mut scene_task = OnSchedule::new(scene.handle(), &times);
    let started = Instant::now();
    {
        let mut simulation = Simulation::new(epoch(), false);
        simulation.add_module("scene", &mut scene_task, BASE_TICK_NANOS, 10);
        for (index, (force, log)) in forces.iter().zip(force_logs.iter_mut()).enumerate() {
            connect!(&simulation, force => &mut log.module.input);
            simulation.add_module(format!("srpForce{index}"), log, BASE_TICK_NANOS, 0);
        }
        connect!(&simulation, &hub_state => &mut hub_log.module.input);
        simulation.add_module("scStateRecorder", &mut hub_log, BASE_TICK_NANOS, 0);
        if let Some(csv) = csv.as_mut() {
            for (force, (side, tier)) in forces.iter().zip(PANEL_IDS) {
                connect!(&simulation,
                    force => csv.module.add_source::<SingleActuatorMsg>(
                        scalar_column("input", &format!("panel_{side}{tier}_srp_N")),
                    ),
                );
            }
            connect!(&simulation,
                &hub_state => csv.module.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
            );
            simulation.add_module("csv", csv, BASE_TICK_NANOS, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            for (side, tier) in PANEL_IDS {
                let body = format!("panel_{side}{tier}");
                scene.body_mut(&body).set_scalar_joint_position(
                    &format!("{body}_deploy"),
                    JOINT_PROFILES[tier - 1].0,
                );
            }
            if config.initial_spin {
                scene
                    .body_mut("hub")
                    .set_attitude_rate(Vector3::new(0.0, 0.8, 0.0));
            }
        }
        // To 59 s at 1 s, to 61 s at 0.025 s, to 70 s at 1 s.
        simulation.run_for(59 * SECOND);
        simulation.run_for(2 * SECOND);
        simulation.run_for(9 * SECOND);
    }
    println!("Run sim in {:.3} seconds", started.elapsed().as_secs_f64());
    if let Some(csv) = csv.as_mut() {
        csv.module.flush().expect("flush scenario CSV");
    }
    Results {
        forces: force_logs
            .into_iter()
            .map(|log| log.module.into_samples())
            .collect(),
        hub: hub_log.module.into_samples(),
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
        let (_, last) = results.hub.last().expect("recorded hub states");
        println!(
            "final hub velocity [{:.4e}, {:.4e}, {:.4e}] m/s",
            last.velocity_mps.x, last.velocity_mps.y, last.velocity_mps.z
        );
        println!("wrote {}", written_csv(config.file_stem()).display());
    }
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario (default
    //! parameters). These check the task timing, the pressure model and the
    //! push it gives the spacecraft.

    use super::*;

    #[test]
    fn deployed_panels_face_the_sun_and_push_the_spacecraft() {
        let results = run(
            Config {
                initial_spin: false,
            },
            false,
        );
        let times = task_times();
        assert_eq!(times.len(), 60 + 41 + 9);
        for log in &results.forces {
            assert_eq!(log.iter().map(|(time, _)| *time).collect::<Vec<_>>(), times);
            // The force never exceeds srpFactor, and deployed (normals
            // along the sunlight) it is srpFactor.
            assert!(
                log.iter()
                    .all(|(_, force)| force.input >= 0.0 && force.input <= 1.0 + 1e-9)
            );
            let (_, last) = log.last().unwrap();
            assert!((last.input - 1.0).abs() < 1e-3, "{}", last.input);
        }
        // Six 1 N forces along the sunlight push the deployed spacecraft
        // along -z.
        let (_, last) = results.hub.last().unwrap();
        let at_60 = &results.hub[times.iter().position(|t| *t == 60 * SECOND).unwrap()].1;
        assert!(last.velocity_mps.z < at_60.velocity_mps.z);
    }
}
