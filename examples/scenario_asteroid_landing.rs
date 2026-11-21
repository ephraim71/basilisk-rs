//! A lander touching down on Itokawa: port of upstream
//! `examples/mujoco/scenarioAsteroidLanding.py`. Needs the `mujoco` feature
//! and upstream's Itokawa shape model (`just fetch-mujoco-scenario-data`).
//!
//! `sat_ast_landing.xml` (in `examples/data/mujoco/`) is a free-floating
//! `hub` with two panels and four landing legs above a fixed `asteroid`
//! body, whose mesh is `ItokawaHayabusa.obj` scaled to metres and loaded
//! into the scene's virtual file system (upstream
//! `MJScene.fromFile(XML_PATH, files=[AST_OBJ_PATH])`). MuJoCo resolves the
//! leg-surface contacts. Gravity is upstream's `ConstantGravity` Python
//! model, here [`ConstantGravity`]: a constant 200 N along inertial -z,
//! rotated into the hub origin frame at every integrator stage and applied
//! through a force actuator added at that site (`addForceActuator`). The
//! hub starts at 1 m/s towards the asteroid with its 275 N thruster
//! braking; the thruster cuts off at 47.5 s and the lander falls onto the
//! surface by 70 s. RKF45 with tolerances 1e-3, 0.02 s task step.
//!
//! Writes `examples/output/scenarios/scenarioAsteroidLanding.csv`: the hub
//! origin state (upstream `bodyStateRecorder`) every step.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! braking and the landing. Upstream's Vizard call, asteroid custom model
//! and thruster-plume writer (Vizard only, `saveFile` commented out) are not
//! ported.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use std::path::PathBuf;

use basilisk_rs::messages::{
    BodyForceCommandMsg, Input, Output, SingleActuatorMsg, SpacecraftStateMsg,
};
use basilisk_rs::mujoco_dynamics::{MjScene, SceneModel, TaskContext};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use mj_scenarios::{epoch, model_path, rkf45, site_state_columns, written_csv};
use nalgebra::Vector3;
use recording::{History, scenario_csv};
use shared::Shared;

/// Upstream `dt` \[ns\].
const STEP_NANOS: u64 = 20_000_000;
/// Upstream `timeThrustTurnOff` \[ns\].
const THRUST_OFF_NANOS: u64 = 47_500_000_000;
/// Upstream `tf` \[ns\].
const DURATION_NANOS: u64 = 70_000_000_000;
/// Upstream `thrust` \[N\].
const THRUST_N: f64 = 275.0;
/// Upstream `ConstantGravity(force_N=...)` \[N\].
const GRAVITY_FORCE_N: Vector3<f64> = Vector3::new(0.0, 0.0, -200.0);
/// Upstream `AST_OBJ_PATH` (`just fetch-mujoco-scenario-data`).
const ASTEROID_OBJ: &str = "assets/data_for_examples/Itokawa/ItokawaHayabusa.obj";
const FILE_STEM: &str = "scenarioAsteroidLanding";

/// Upstream's Python `ConstantGravity(sysModel.SysModel)`: a force fixed
/// in the inertial frame, `force_S = [SN] force_N` in the frame of the site
/// whose state it reads, for a force actuator at that site.
#[derive(Debug)]
pub struct ConstantGravity {
    pub name: String,
    /// The force in the inertial frame \[N\].
    pub force_n: Vector3<f64>,
    /// Upstream `frameInMsg`: the state of the site the force acts at.
    pub frame_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `forceOutMsg` (`ForceAtSiteMsg`; basilisk-rs's site-frame
    /// force message, see the actuators).
    pub force_out_msg: Output<BodyForceCommandMsg>,
}

impl SceneModel for ConstantGravity {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        let dcm_bn = mrp_to_dcm(self.frame_in_msg.read().sigma_bn);
        self.force_out_msg.write(BodyForceCommandMsg {
            force_request_body_n: dcm_bn * self.force_n,
        });
    }
}

fn asteroid_obj() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(ASTEROID_OBJ)
}

fn scene() -> MjScene {
    let obj = asteroid_obj();
    assert!(
        obj.is_file(),
        "{} is missing (just fetch-mujoco-scenario-data)",
        obj.display()
    );
    let xml_path = model_path("sat_ast_landing.xml");
    let xml = std::fs::read_to_string(&xml_path)
        .unwrap_or_else(|error| panic!("cannot read {}: {error}", xml_path.display()));
    MjScene::with_files(&xml, vec![obj]).expect("valid landing scene")
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    hub: Vec<(u64, SpacecraftStateMsg)>,
}

fn run(write_csv: bool) -> Results {
    let mut scene = scene();
    scene.set_integrator(rkf45(1e-3, 1e-3));

    // 200 N along inertial -z at the hub origin, through a new force
    // actuator there.
    let origin = scene.body("hub").origin().name().to_string();
    let mut gravity = ConstantGravity {
        name: "gravity".to_string(),
        force_n: GRAVITY_FORCE_N,
        frame_in_msg: Input::default(),
        force_out_msg: Output::default(),
    };
    gravity.force_out_msg.connect_to(
        &mut scene
            .add_force_actuator("hub_gravity", &origin)
            .force_in_msg,
    );
    scene
        .site(&origin)
        .state_out_msg
        .connect_to(&mut gravity.frame_in_msg);
    scene.add_model_to_dynamics_task(gravity, -1);

    // The braking thrust, cut off near the surface.
    let thrust = Output::new(SingleActuatorMsg { input: THRUST_N });
    thrust.connect_to(&mut scene.single_actuator_mut("thrust").actuator_in_msg);

    let hub_state = scene.body("hub").origin().state_out_msg.clone();
    let mut hub_log = History::<SpacecraftStateMsg>::new();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation, &hub_state => &mut hub_log.input);
        schedule! { simulation,
            "scene" => &mut scene_task, period=STEP_NANOS, priority=10;
            "bodyStateRecorder" => &mut hub_log, period=STEP_NANOS, priority=0;
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
            );
            simulation.add_module("csv", csv, STEP_NANOS, 0);
        }
        simulation.initialize();
        // 1 m/s towards the asteroid.
        scene
            .lock()
            .body_mut("hub")
            .set_velocity(Vector3::new(0.0, 0.0, -1.0));
        simulation.run_for(THRUST_OFF_NANOS);
        // Near the surface: thruster off, gravity lands the hub.
        thrust.write(SingleActuatorMsg { input: 0.0 });
        simulation.run_for(DURATION_NANOS - THRUST_OFF_NANOS);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        hub: hub_log.into_samples(),
    }
}

fn main() {
    Cli::from_env("scenario_asteroid_landing").finish();
    let results = run(true);
    let (_, last) = results.hub.last().expect("recorded hub states");
    println!(
        "final hub position {:.3?} m, velocity {:.3?} m/s",
        last.position_m, last.velocity_mps
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! braked descent and the landing.

    use super::*;

    #[test]
    fn the_lander_brakes_falls_and_comes_to_rest_on_the_surface() {
        let results = run(false);
        assert_eq!(results.hub.len(), 3501);
        let at = |nanos: u64| {
            results
                .hub
                .iter()
                .find(|(time, _)| *time == nanos)
                .map(|(_, state)| state.clone())
                .unwrap()
        };
        // Braking: 75 N net upwards nearly cancels the 1 m/s descent by the
        // cut-off.
        let cut_off = at(THRUST_OFF_NANOS);
        assert!(
            cut_off.velocity_mps.z.abs() < 0.2,
            "{}",
            cut_off.velocity_mps
        );
        assert!(cut_off.position_m.z < -10.0, "{}", cut_off.position_m);
        // Free fall then contact: the hub ends above the asteroid's center,
        // stopped on the surface.
        let (_, last) = results.hub.last().unwrap();
        assert!(last.position_m.z > -150.0, "{}", last.position_m);
        // The legs hold it on the surface, about 24 m below the start:
        // the 1.4 m/s free fall would otherwise take it 30 m down.
        let lowest = results
            .hub
            .iter()
            .map(|(_, state)| state.position_m.z)
            .fold(f64::INFINITY, f64::min);
        assert!(lowest > -26.0 && lowest < -23.6, "{lowest}");
        let fastest = results
            .hub
            .iter()
            .map(|(_, state)| state.velocity_mps.norm())
            .fold(0.0, f64::max);
        assert!(
            last.velocity_mps.norm() < 0.2 * fastest,
            "{} vs {fastest}",
            last.velocity_mps
        );
    }
}
