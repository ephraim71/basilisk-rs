//! The classic hinged-panel burn on MuJoCo dynamics: port of upstream
//! `examples/mujoco/scenarioHingedRigidBodyMuJoCo.py`. Needs the `mujoco`
//! feature.
//!
//! A free-floating 800 kg `hub` with two 100 kg panels on hinges
//! (`hinge1`, `hinge2`), built from upstream's XML string. Each hinge is a
//! torsional spring (1000 N m/rad, no damping) through a joint motor driven
//! by upstream's C++ `JointPIDController` ([`JointPidController`]) towards
//! 0 rad. Earth's point-mass gravity acts on all three bodies through
//! `NBodyGravity`. After coasting for 1% of the near-circular 7000 km
//! orbit, upstream's Python `BurnWindowForceCommand` ([`BurnWindowForceCommand`])
//! requests `(-2050, -1430, -0.00076)` N in the inertial frame for 935 s,
//! which `CmdForceInertialToForceAtSite` rotates into the hub's
//! `thrustSite` for a force actuator there; the panels, started at 5°,
//! respond to the acceleration. Default RK4, 0.1 s task step.
//!
//! Upstream calls `ExecuteSimulation()` without a `ConfigureStopTime`,
//! which stops at 0 s: its run covers neither the coast nor the burn its
//! documentation describes. This port runs both, to the end of the burn
//! window, as upstream's classic `scenarioHingedRigidBody` does.
//!
//! Writes `examples/output/scenarios/scenarioHingedRigidBodyMuJoCo.csv`:
//! the hub center-of-mass state and both hinge angles, at upstream's
//! sampling time (100 points over the coast, 0.5 s).
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! burn's velocity change and the panels' response.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;

use basilisk_rs::dynamics::gravity::PointMassGravityModel;
use basilisk_rs::messages::{
    InertialForceCommandMsg, Output, ScalarJointStateMsg, SpacecraftStateMsg,
};
use basilisk_rs::mujoco_dynamics::cmd_force_inertial_to_force_at_site::CmdForceInertialToForceAtSite;
use basilisk_rs::mujoco_dynamics::joint_pid_controller::JointPidController;
use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
use basilisk_rs::mujoco_dynamics::{MjScene, SceneModel, TaskContext};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use message_log::MessageLog;
use mj_scenarios::{epoch, scalar_column, site_state_columns, written_csv};
use nalgebra::Vector3;
use orbit_tools::{orbit_period_s, sec2nano};
use recording::{sampling_time, scenario_csv};
use shared::Shared;

/// Upstream `timeStep` \[ns\].
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `muEarth` \[m^3/s^2\].
const MU_EARTH: f64 = 0.3986004415e15;
/// Upstream's burn force, inertial \[N\].
const BURN_FORCE_N: Vector3<f64> = Vector3::new(-2050.0, -1430.0, -0.00076);
/// Upstream `T2`: the burn duration \[s\].
const BURN_S: f64 = 935.0;
/// Upstream `simulationTimeFactor`: the coast, as a fraction of an orbit.
const COAST_FRACTION: f64 = 0.01;
const FILE_STEM: &str = "scenarioHingedRigidBodyMuJoCo";

/// Upstream `makeMjXmlString(hubMass=800, busIDiag=(900, 800, 600))`.
fn xml() -> String {
    let (hub_mass, (ixx, iyy, izz)) = (800.0, (900.0, 800.0, 600.0));
    format!(
        r#"
    <mujoco model = "busWith2Panels">
        <compiler angle = "radian" meshdir = ""/>
        <default>
            <default class = "panel_geom">
                <geom type = "box" size = "1 1 0.01"
                contype = "0" conaffinity = "1"/>
            </default>
        </default>
        <worldbody>
            <body name = "hub" pos = "0 0 0">
                <freejoint name = "busFree"/>
                <inertial pos = "0 0 0" mass = "{hub_mass}" diaginertia = "{ixx} {iyy} {izz}"/>
                <geom name = "hubVisual" type = "box" size = "1 1 1" rgba = "1 1 1 1"/>
                <site name = "thrustSite" pos = "0 0 0"/>
                <body name = "panel1" pos = "0.5 0.0 1.0">
                    <joint name = "hinge1" pos = "0 0 0" axis = "0 -1 0" ref = "0"/>
                    <inertial pos = "1.5 0 0" mass = "100.0" diaginertia = "100.0 50.0 50.0"/>
                    <geom name = "panel1_geom" class = "panel_geom" pos = "1.5 0 0"/>
                </body>
                <body name = "panel2" pos = "-0.5 0.0 1.0">
                    <joint name = "hinge2" pos = "0 0 0" axis = "0 1 0" ref = "0"/>
                    <inertial pos = "-1.5 0 0" mass = "100.0" diaginertia = "100.0 50.0 50.0"/>
                    <geom name = "panel2_geom" class = "panel_geom" pos = "-1.5 0 0"/>
                </body>
            </body>
        </worldbody>
    </mujoco>
    "#
    )
}

/// Upstream's Python `BurnWindowForceCommand(sysModel.SysModel)`: a fixed
/// inertial force during the inclusive window `[start, end]`, zero outside.
#[derive(Debug)]
pub struct BurnWindowForceCommand {
    pub name: String,
    pub force_n: Vector3<f64>,
    pub burn_start_nanos: u64,
    pub burn_end_nanos: u64,
    /// Upstream `cmdForceOutMsg`.
    pub cmd_force_out_msg: Output<InertialForceCommandMsg>,
}

impl SceneModel for BurnWindowForceCommand {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let now = context.current_sim_nanos();
        let burning = (self.burn_start_nanos..=self.burn_end_nanos).contains(&now);
        self.cmd_force_out_msg.write(InertialForceCommandMsg {
            force_request_inertial_n: if burning {
                self.force_n
            } else {
                Vector3::zeros()
            },
        });
    }
}

/// The coast length and the burn window \[ns\].
struct Timing {
    coast_nanos: u64,
    burn_end_nanos: u64,
}

fn timing() -> Timing {
    let period = orbit_period_s(MU_EARTH, 7000.0e3);
    let coast_nanos = sec2nano(COAST_FRACTION * period);
    Timing {
        coast_nanos,
        burn_end_nanos: coast_nanos + sec2nano(BURN_S),
    }
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    hub: Vec<(u64, SpacecraftStateMsg)>,
    hinge_1: Vec<(u64, ScalarJointStateMsg)>,
    hinge_2: Vec<(u64, ScalarJointStateMsg)>,
    sampling_nanos: u64,
}

fn run(write_csv: bool) -> Results {
    let mut scene = MjScene::new(&xml()).expect("valid hinged-panel scene");
    scene.name = "mujocoScene".to_string();
    let thrust_actuator = "thrustForce";
    scene.add_force_actuator(thrust_actuator, "thrustSite");

    // Torsional springs at the hinges.
    for (joint, body) in [("hinge1", "panel1"), ("hinge2", "panel2")] {
        let actuator = format!("{joint}Actuator");
        scene.add_joint_single_actuator(&actuator, joint);
        let mut spring = JointPidController::default();
        spring.name = format!("{joint}SpringDamper");
        spring.set_proportional_gain(1000.0);
        spring.set_derivative_gain(0.0);
        let reference = Output::new(ScalarJointStateMsg { state: 0.0 });
        let reference_rate = Output::new(ScalarJointStateMsg { state: 0.0 });
        reference.connect_to(&mut spring.desired_pos_in_msg);
        reference_rate.connect_to(&mut spring.desired_vel_in_msg);
        let joint = scene.body(body).scalar_joint(joint).clone();
        joint
            .state_out_msg
            .connect_to(&mut spring.measured_pos_in_msg);
        joint
            .state_dot_out_msg
            .connect_to(&mut spring.measured_vel_in_msg);
        spring
            .output_out_msg
            .connect_to(&mut scene.single_actuator_mut(&actuator).actuator_in_msg);
        scene.add_model_to_dynamics_task(spring, -1);
    }

    // Earth's point-mass gravity on every body.
    let (position_m, velocity_mps) = elements_to_state(
        MU_EARTH,
        &ClassicElements {
            semi_major_axis: 7000.0e3,
            eccentricity: 0.0001,
            inclination_rad: 0.0,
            raan_rad: 48.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
            ..Default::default()
        },
    );
    let mut gravity = NBodyGravity::new("gravity");
    gravity.add_gravity_source(
        "earth",
        PointMassGravityModel::new(MU_EARTH).expect("positive mu"),
        true,
    );
    for body in ["hub", "panel1", "panel2"] {
        gravity.add_gravity_target_at_body(&mut scene, body, body);
    }
    scene.add_model_to_dynamics_task(gravity, -1);

    // The burn: inertial command, rotated into the thrust site.
    let timing = timing();
    let burn = BurnWindowForceCommand {
        name: "burnCommand".to_string(),
        force_n: BURN_FORCE_N,
        burn_start_nanos: timing.coast_nanos,
        burn_end_nanos: timing.burn_end_nanos,
        cmd_force_out_msg: Output::default(),
    };
    let mut converter = CmdForceInertialToForceAtSite {
        name: "forceConverter".to_string(),
        ..CmdForceInertialToForceAtSite::default()
    };
    burn.cmd_force_out_msg
        .connect_to(&mut converter.cmd_force_inertial_in_msg);
    scene
        .site("thrustSite")
        .state_out_msg
        .connect_to(&mut converter.site_frame_state_in_msg);
    converter
        .force_out_msg
        .connect_to(&mut scene.force_actuator_mut(thrust_actuator).force_in_msg);
    scene.add_model_to_dynamics_task(burn, -1);
    scene.add_model_to_dynamics_task(converter, -1);

    let sampling_nanos = sampling_time(timing.coast_nanos, STEP_NANOS, 100);
    let hub_state = scene.body("hub").center_of_mass().state_out_msg.clone();
    let hinge_1 = scene.scalar_joint("hinge1").state_out_msg.clone();
    let hinge_2 = scene.scalar_joint("hinge2").state_out_msg.clone();
    let mut hub_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut hinge_1_log = MessageLog::<ScalarJointStateMsg>::default();
    let mut hinge_2_log = MessageLog::<ScalarJointStateMsg>::default();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &hub_state => &mut hub_log.input,
            &hinge_1 => &mut hinge_1_log.input,
            &hinge_2 => &mut hinge_2_log.input,
        );
        schedule! { simulation,
            "mujocoScene" => &mut scene_task, period=STEP_NANOS, priority=10;
            "dataLog" => &mut hub_log, period=sampling_nanos, priority=0;
            "pl1Log" => &mut hinge_1_log, period=sampling_nanos, priority=0;
            "pl2Log" => &mut hinge_2_log, period=sampling_nanos, priority=0;
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("hub")),
                &hinge_1 => csv.add_source::<ScalarJointStateMsg>(scalar_column("state", "hinge1_rad")),
                &hinge_2 => csv.add_source::<ScalarJointStateMsg>(scalar_column("state", "hinge2_rad")),
            );
            simulation.add_module("csv", csv, sampling_nanos, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            {
                let mut hub = scene.body_mut("hub");
                hub.set_position(position_m);
                hub.set_velocity(velocity_mps);
            }
            let theta = 5.0_f64.to_radians();
            scene
                .body_mut("panel1")
                .set_scalar_joint_position("hinge1", theta);
            scene
                .body_mut("panel2")
                .set_scalar_joint_position("hinge2", theta);
        }
        // The coast, then the burn.
        simulation.run_for(timing.burn_end_nanos);
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        hub: hub_log.into_samples(),
        hinge_1: hinge_1_log.into_samples(),
        hinge_2: hinge_2_log.into_samples(),
        sampling_nanos,
    }
}

fn main() {
    Cli::from_env("scenario_hinged_rigid_body_mujoco").finish();
    let results = run(true);
    let (_, first) = &results.hub[0];
    let (_, last) = results.hub.last().expect("recorded hub states");
    println!(
        "orbit radius {:.1} -> {:.1} km, speed {:.1} -> {:.1} m/s",
        first.position_m.norm() / 1e3,
        last.position_m.norm() / 1e3,
        first.velocity_mps.norm(),
        last.velocity_mps.norm()
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! burn and the panels' response.

    use super::*;

    #[test]
    fn the_burn_changes_the_velocity_and_the_panels_respond() {
        let results = run(false);
        let timing = timing();
        assert_eq!(results.sampling_nanos, 500_000_000);
        let state_at = |nanos: u64| {
            results
                .hub
                .iter()
                .min_by_key(|(time, _)| time.abs_diff(nanos))
                .map(|(_, state)| state.clone())
                .unwrap()
        };
        // Coasting: two-body motion, the springs swing the panels about 0
        // with their initial 5° amplitude (no damping).
        let coast: Vec<f64> = results
            .hinge_1
            .iter()
            .filter(|(time, _)| *time <= timing.coast_nanos)
            .map(|(_, state)| state.state)
            .collect();
        let amplitude = coast.iter().map(|angle| angle.abs()).fold(0.0, f64::max);
        assert!(
            (amplitude - 5.0_f64.to_radians()).abs() < 0.2_f64.to_radians(),
            "{amplitude}"
        );
        assert!(coast.iter().any(|angle| *angle < 0.0));
        // The burn: the inertially fixed force on the system's 1000 kg, on
        // top of gravity. A point mass integrated through the same window
        // from the last coasting sample lands on the same velocity (the
        // hub's swinging panels move its center of mass by centimetres).
        let (t0, start) = results
            .hub
            .iter()
            .filter(|(time, _)| *time < timing.coast_nanos)
            .last()
            .cloned()
            .unwrap();
        let (t1, end) = results.hub.last().cloned().unwrap();
        let acceleration = |r: Vector3<f64>, thrust: bool| {
            -MU_EARTH * r / r.norm().powi(3)
                + if thrust {
                    BURN_FORCE_N / 1000.0
                } else {
                    Vector3::zeros()
                }
        };
        let mut r = start.position_m;
        let mut v = start.velocity_mps;
        let mut propagate = |from: u64, to: u64, thrust: bool| {
            let steps = ((to - from) as f64 / 1e7).ceil() as usize;
            let h = (to - from) as f64 * 1e-9 / steps as f64;
            for _ in 0..steps {
                let k1 = (v, acceleration(r, thrust));
                let k2 = (v + 0.5 * h * k1.1, acceleration(r + 0.5 * h * k1.0, thrust));
                let k3 = (v + 0.5 * h * k2.1, acceleration(r + 0.5 * h * k2.0, thrust));
                let k4 = (v + h * k3.1, acceleration(r + h * k3.0, thrust));
                r += h / 6.0 * (k1.0 + 2.0 * k2.0 + 2.0 * k3.0 + k4.0);
                v += h / 6.0 * (k1.1 + 2.0 * k2.1 + 2.0 * k3.1 + k4.1);
            }
        };
        propagate(t0, timing.coast_nanos, false);
        propagate(timing.coast_nanos, t1, true);
        let delta_v = (end.velocity_mps - start.velocity_mps).norm();
        assert!(delta_v > 1000.0, "{delta_v}");
        assert!(
            (end.velocity_mps - v).norm() < 1.0,
            "{} vs {v}",
            end.velocity_mps
        );
        assert!(
            (end.position_m - r).norm() < 500.0,
            "{} vs {r}",
            end.position_m
        );
        // The panels stay on their hinges' springs, deflected by the burn.
        let burn_amplitude = results
            .hinge_2
            .iter()
            .map(|(_, state)| state.state.abs())
            .fold(0.0, f64::max);
        assert!(
            burn_amplitude > amplitude && burn_amplitude < 1.0,
            "{burn_amplitude}"
        );
    }
}
