//! A flexible solar panel on MuJoCo dynamics: port of upstream
//! `examples/mujoco/scenarioFlexiblePanelMuJoCo.py`, the MuJoCo translation
//! of `scenarioFlexiblePanel`. Needs the `mujoco` feature.
//!
//! A free-floating 1000 kg `hub` (3 × 3 × 6 m) carries an 18 m, 100 kg panel
//! discretized into five rigid sub-panels (`subPanel1`..`subPanel5`), each
//! on two colocated hinges, a bending joint (`bendJoint<i>`) and a twisting
//! joint (`twistJoint<i>`), built from upstream's XML string. Each hinge is
//! a torsional spring-damper: a joint motor driven by upstream's C++
//! `JointPIDController` ([`JointPidController`]) towards 0 rad (bending
//! 10 N m/rad and 8 N m s/rad, twisting 1 N m/rad and 0.8 N m s/rad). Earth's
//! point-mass gravity acts on the hub and every sub-panel. The FSW points the
//! hub at σ_R0N = (0.3, 0.4, 0.5): `simpleNav` on the hub center of mass,
//! `inertial3D`, `attTrackingError` and `mrpFeedback`, with the hub-plus-panel
//! inertia about the hub point (parallel-axis theorem) and gains for a 50 s
//! decay time at 0.9 damping. Upstream's Python
//! `cmdTorqueBodyToTorqueAtSite` ([`CmdTorqueBodyToTorqueAtSite`]) relays the
//! commanded torque, in the scene's dynamics task, to a torque actuator at
//! `hubSite`. Default RK4 at the 0.5 s dynamics task step, FSW at 1 s,
//! 10 min, on an orbit of 8000 km semi-major axis and eccentricity 0.1.
//!
//! Writes `examples/output/scenarios/scenarioFlexiblePanelMuJoCo.csv`: every
//! bending and twisting angle and rate (dynamics task, 0.5 s), and
//! `scenarioFlexiblePanelMuJoCoAttError.csv`: the attitude and rate errors
//! (FSW task, 1 s).
//!
//! Upstream `run(showPlots)` has no scenario parameters (its
//! `numberOfSegments` is 5), and `test_scenarioMujoco.py` only checks that it
//! runs; `tests` checks that the hub reaches its reference while the panel
//! flexes and settles.

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

use basilisk_rs::dynamics::gravity::PointMassGravityModel;
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::att_tracking_error::{
    AttTrackingError, AttTrackingErrorConfig,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, Output, ScalarJointStateMsg, VehicleConfigMsg,
};
use basilisk_rs::mujoco_dynamics::joint_pid_controller::JointPidController;
use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
use basilisk_rs::mujoco_dynamics::{MjScene, SceneModel, TaskContext};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{seconds, vector_columns};
use message_log::MessageLog;
use mj_scenarios::{epoch, scalar_column, written_csv};
use nalgebra::{Matrix3, Vector3};
use recording::scenario_csv;
use shared::Shared;

/// Upstream `timeStep`: the dynamics task.
const DYN_STEP_NANOS: u64 = 500_000_000;
/// Upstream `fswTimeStep`.
const FSW_STEP_NANOS: u64 = seconds(1);
/// Upstream `simulationTime` (10 min).
const DURATION_NANOS: u64 = seconds(600);
/// Upstream `muEarth` \[m^3/s^2\].
const MU_EARTH: f64 = 0.3986004415e15;
/// Upstream `numberOfSegments`.
const SEGMENTS: usize = 5;
/// Upstream `kBend, cBend` and `kTwist, cTwist`.
const BEND: (f64, f64) = (10.0, 8.0);
const TWIST: (f64, f64) = (1.0, 0.8);
const FILE_STEM: &str = "scenarioFlexiblePanelMuJoCo";

/// Upstream `geometryClass(numberOfSegments)`.
struct Geometry {
    mass_hub: f64,
    length_hub: f64,
    width_hub: f64,
    height_hub: f64,
    length_panel: f64,
    width_panel: f64,
    thickness_panel: f64,
    mass_panel: f64,
    segments: usize,
}

impl Geometry {
    fn new(segments: usize) -> Self {
        Self {
            mass_hub: 1000.0,
            length_hub: 3.0,
            width_hub: 3.0,
            height_hub: 6.0,
            length_panel: 18.0,
            width_panel: 3.0,
            thickness_panel: 0.3,
            mass_panel: 100.0,
            segments,
        }
    }

    fn mass_sub_panel(&self) -> f64 {
        self.mass_panel / self.segments as f64
    }

    fn length_sub_panel(&self) -> f64 {
        self.length_panel / self.segments as f64
    }

    /// The hub's inertia about its center of mass.
    fn hub_inertia(&self) -> Matrix3<f64> {
        let m = self.mass_hub / 12.0;
        Matrix3::from_diagonal(&Vector3::new(
            m * (self.length_hub.powi(2) + self.height_hub.powi(2)),
            m * (self.width_hub.powi(2) + self.height_hub.powi(2)),
            m * (self.length_hub.powi(2) + self.width_hub.powi(2)),
        ))
    }
}

/// Python's `round(x, 6)`, for the sub-panel inertias in the XML.
fn round6(value: f64) -> f64 {
    (value * 1e6).round() / 1e6
}

/// Upstream `panelChainGen(scGeometry, baseIndent=3)`.
fn panel_chain(geometry: &Geometry) -> String {
    let mut open = Vec::new();
    let mut close = Vec::new();
    let (mass, length) = (geometry.mass_sub_panel(), geometry.length_sub_panel());
    let (width, thickness) = (geometry.width_panel, geometry.thickness_panel);
    for index in 0..geometry.segments {
        let n = index + 1;
        let bending_pos = if index == 0 {
            format!(
                "0 {} {}",
                geometry.length_hub / 2.0,
                geometry.height_hub / 2.0 - thickness / 2.0
            )
        } else {
            format!("0 {length} 0")
        };
        let com_offset = format!("0 {} 0", length / 2.0);
        let ixx = round6(mass / 12.0 * (length.powi(2) + thickness.powi(2)));
        let iyy = round6(mass / 12.0 * (width.powi(2) + thickness.powi(2)));
        let izz = round6(mass / 12.0 * (width.powi(2) + length.powi(2)));
        open.push(format!(
            r#"<body name = "subPanel{n}" pos = "{bending_pos}">
   <joint name = "bendJoint{n}" pos = "0 0 0" axis = "1 0 0" ref = "0"/>
   <joint name = "twistJoint{n}" pos = "0 0 0" axis = "0 1 0" ref = "0"/>
   <inertial pos = "{com_offset}" mass = "{mass}" diaginertia = "{ixx} {iyy} {izz}"/>
   <geom name = "subPanel{n}Geom" class = "subpanel_geom" pos = "{com_offset}"/>"#
        ));
        close.push("</body>".to_string());
    }
    close.reverse();
    format!("{}\n{}", open.join("\n"), close.join("\n"))
}

/// Upstream `makeMjXmlString(scGeometry)`.
fn xml(geometry: &Geometry) -> String {
    let inertia = geometry.hub_inertia();
    format!(
        r#"<mujoco model = "busWithFlexiblePanel">
    <compiler angle = "radian" meshdir = ""/>
    <default>
        <default class = "subpanel_geom">
            <geom type = "box" size = "{} {} {}"
            contype = "0" conaffinity = "1"/>
        </default>
    </default>
    <worldbody>
        <body name = "hub" pos = "0 0 0">
            <freejoint name = "busFree"/>
            <inertial pos = "0 0 0" mass = "{}" diaginertia = "{} {} {}"/>
            <geom name = "hubVisual" type = "box" size = "{} {} {}" rgba = "1 1 1 1"/>
            <site name = "hubSite" pos = "0 0 0"/>
{}
        </body>
    </worldbody>
</mujoco>"#,
        geometry.width_panel / 2.0,
        geometry.length_sub_panel() / 2.0,
        geometry.thickness_panel / 2.0,
        geometry.mass_hub,
        inertia[(0, 0)],
        inertia[(1, 1)],
        inertia[(2, 2)],
        geometry.width_hub / 2.0,
        geometry.length_hub / 2.0,
        geometry.height_hub / 2.0,
        panel_chain(geometry),
    )
}

/// Upstream's Python `CmdTorqueBodyToTorqueAtSite(sysModel.SysModel)`
/// (`simulation/deviceInterface/cmdTorqueBodyToTorqueAtSite`): a commanded
/// body-frame torque expressed in a site frame, `torque_S = [SB] torque_B`,
/// for a torque actuator at that site. `dcm_SB` defaults to identity.
#[derive(Debug)]
pub struct CmdTorqueBodyToTorqueAtSite {
    pub name: String,
    pub dcm_sb: Matrix3<f64>,
    /// Upstream `cmdTorqueInMsg`.
    pub cmd_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `torqueOutMsg` (`TorqueAtSiteMsg`; basilisk-rs's site-frame
    /// torque message, see the actuators).
    pub torque_out_msg: Output<BodyTorqueCommandMsg>,
}

impl SceneModel for CmdTorqueBodyToTorqueAtSite {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        self.torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: self.dcm_sb
                * self.cmd_torque_in_msg.read().torque_request_body_nm,
            ..Default::default()
        });
    }
}

/// The skew-symmetric (cross-product) matrix of `v`: upstream `v3Tilde`.
fn tilde(v: Vector3<f64>) -> Matrix3<f64> {
    v.cross_matrix()
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: Vec<(u64, AttitudeGuidanceMsg)>,
    /// `(bend, twist)` angle histories per segment.
    angles: Vec<(AngleHistory, AngleHistory)>,
}

/// One hinge's recorded angles.
type AngleHistory = Vec<(u64, ScalarJointStateMsg)>;

fn run(write_csv: bool) -> Results {
    let geometry = Geometry::new(SEGMENTS);
    let mut scene = MjScene::new(&xml(&geometry)).expect("valid flexible-panel scene");
    scene.name = "mujocoScene".to_string();

    // Spring-dampers on every bending and twisting hinge.
    for segment in 1..=SEGMENTS {
        for (joint, (k, c)) in [
            (format!("bendJoint{segment}"), BEND),
            (format!("twistJoint{segment}"), TWIST),
        ] {
            let actuator = format!("{joint}Actuator");
            scene.add_joint_single_actuator(&actuator, &joint);
            let mut spring = JointPidController::default();
            spring.name = format!("{joint}SpringDamper");
            spring.set_proportional_gain(k);
            spring.set_derivative_gain(c);
            let reference = Output::new(ScalarJointStateMsg { state: 0.0 });
            let reference_rate = Output::new(ScalarJointStateMsg { state: 0.0 });
            reference.connect_to(&mut spring.desired_pos_in_msg);
            reference_rate.connect_to(&mut spring.desired_vel_in_msg);
            let joint = scene.scalar_joint(&joint).clone();
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
    }

    // Earth on the hub and every sub-panel.
    let mut gravity = NBodyGravity::new("gravity");
    gravity.add_gravity_source(
        "earth",
        PointMassGravityModel::new(MU_EARTH).expect("positive mu"),
        true,
    );
    gravity.add_gravity_target_at_body(&mut scene, "hub", "hub");
    for segment in 1..=SEGMENTS {
        let body = format!("subPanel{segment}");
        gravity.add_gravity_target_at_body(&mut scene, &body, &body);
    }
    scene.add_model_to_dynamics_task(gravity, -1);

    // Navigation and attitude control.
    let mut navigation = SimpleNavigation::new("simpleNav");
    scene
        .body("hub")
        .center_of_mass()
        .state_out_msg
        .connect_to(&mut navigation.spacecraft_state_in_msg);
    let mut inertial_guidance = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::new(0.3, 0.4, 0.5),
    });
    let mut tracking_error = AttTrackingError::new(AttTrackingErrorConfig {
        name: "attTrackingError".to_string(),
        sigma_r0r: Vector3::zeros(),
    });
    // The hub-plus-panel inertia about the hub point B.
    let panel_inertia = {
        let m = geometry.mass_panel / 12.0;
        Matrix3::from_diagonal(&Vector3::new(
            m * (geometry.length_panel.powi(2) + geometry.thickness_panel.powi(2)),
            m * (geometry.width_panel.powi(2) + geometry.thickness_panel.powi(2)),
            m * (geometry.width_panel.powi(2) + geometry.length_panel.powi(2)),
        ))
    };
    let r_scb_b = Vector3::new(
        0.0,
        geometry.length_hub / 2.0 + geometry.length_panel / 2.0,
        geometry.height_hub / 2.0 - geometry.thickness_panel / 2.0,
    );
    let inertia = geometry.hub_inertia() + panel_inertia
        - geometry.mass_panel * tilde(r_scb_b) * tilde(r_scb_b);
    let max_inertia = inertia.max();
    let (decay_time, xi) = (50.0, 0.9);
    let p = 2.0 * max_inertia / decay_time;
    let mut controller = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: (p / xi).powi(2) / max_inertia,
        ki: 0.0,
        p,
        integral_limit: 0.0,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia,
        ..Default::default()
    });
    // The commanded torque at the hub site.
    let torque_actuator = "hubTorqueAct";
    scene.add_torque_actuator(torque_actuator, "hubSite");
    let mut bridge = CmdTorqueBodyToTorqueAtSite {
        name: "torqueBridge".to_string(),
        dcm_sb: Matrix3::identity(),
        cmd_torque_in_msg: Input::default(),
        torque_out_msg: Output::default(),
    };
    controller
        .cmd_torque_out_msg
        .connect_to(&mut bridge.cmd_torque_in_msg);
    bridge
        .torque_out_msg
        .connect_to(&mut scene.torque_actuator_mut(torque_actuator).torque_in_msg);
    scene.add_model_to_dynamics_task(bridge, -1);

    let joint_states: Vec<_> = (1..=SEGMENTS)
        .map(|segment| {
            let bend = scene.scalar_joint(&format!("bendJoint{segment}")).clone();
            let twist = scene.scalar_joint(&format!("twistJoint{segment}")).clone();
            (
                bend.state_out_msg,
                bend.state_dot_out_msg,
                twist.state_out_msg,
                twist.state_dot_out_msg,
            )
        })
        .collect();
    let guidance_out = tracking_error.att_guid_out_msg.clone();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut angle_logs: Vec<(
        MessageLog<ScalarJointStateMsg>,
        MessageLog<ScalarJointStateMsg>,
    )> = (0..SEGMENTS)
        .map(|_| (MessageLog::default(), MessageLog::default()))
        .collect();
    let mut csv = write_csv.then(|| scenario_csv(FILE_STEM));
    let mut att_csv = write_csv.then(|| scenario_csv(&format!("{FILE_STEM}AttError")));

    let (position_m, velocity_mps) = elements_to_state(
        MU_EARTH,
        &ClassicElements {
            semi_major_axis: 8e6,
            eccentricity: 0.1,
            ..Default::default()
        },
    );
    let scene = Shared::new(scene);
    let mut scene_task = scene.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        connect!(&simulation,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &inertial_guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &guidance_out => &mut controller.guid_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &guidance_out => &mut guidance_log.input,
        );
        for ((bend, _, twist, _), (bend_log, twist_log)) in
            joint_states.iter().zip(angle_logs.iter_mut())
        {
            connect!(&simulation, bend => &mut bend_log.input, twist => &mut twist_log.input);
        }
        // The dynamics process (scene, then navigation), then the FSW
        // process.
        schedule! { simulation,
            "mujocoScene" => &mut scene_task, period=DYN_STEP_NANOS, priority=100;
            "simpleNav" => &mut navigation, period=DYN_STEP_NANOS, priority=90;
            "inertial3D" => &mut inertial_guidance, period=FSW_STEP_NANOS, priority=50;
            "attTrackingError" => &mut tracking_error, period=FSW_STEP_NANOS, priority=40;
            "mrpFeedback" => &mut controller, period=FSW_STEP_NANOS, priority=30;
            "attErrorLog" => &mut guidance_log, period=FSW_STEP_NANOS, priority=0;
        }
        for (index, (bend_log, twist_log)) in angle_logs.iter_mut().enumerate() {
            simulation.add_module(format!("bendLog{index}"), bend_log, DYN_STEP_NANOS, 80);
            simulation.add_module(format!("twistLog{index}"), twist_log, DYN_STEP_NANOS, 80);
        }
        if let Some(csv) = csv.as_mut() {
            for (index, (bend, bend_rate, twist, twist_rate)) in joint_states.iter().enumerate() {
                let n = index + 1;
                connect!(&simulation,
                    bend => csv.add_source::<ScalarJointStateMsg>(scalar_column("state", &format!("theta_{n}_rad"))),
                    bend_rate => csv.add_source::<ScalarJointStateMsg>(scalar_column("state", &format!("thetaDot_{n}_rad_per_s"))),
                    twist => csv.add_source::<ScalarJointStateMsg>(scalar_column("state", &format!("beta_{n}_rad"))),
                    twist_rate => csv.add_source::<ScalarJointStateMsg>(scalar_column("state", &format!("betaDot_{n}_rad_per_s"))),
                );
            }
            simulation.add_module("csv", csv, DYN_STEP_NANOS, 80);
        }
        if let Some(att_csv) = att_csv.as_mut() {
            connect!(&simulation,
                &guidance_out => att_csv.add_source::<AttitudeGuidanceMsg>(
                    CsvSourceConfig::columns(
                        vector_columns("sigma_br", "sigma_BR")
                            .into_iter()
                            .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
                    ),
                ),
            );
            simulation.add_module("attCsv", att_csv, FSW_STEP_NANOS, 0);
        }
        simulation.initialize();
        {
            let mut scene = scene.lock();
            {
                let mut hub = scene.body_mut("hub");
                hub.set_position(position_m);
                hub.set_velocity(velocity_mps);
            }
            // The panel undeformed (upstream `thetaInit = 0`).
            for segment in 1..=SEGMENTS {
                let mut body = scene.body_mut(&format!("subPanel{segment}"));
                body.set_scalar_joint_position(&format!("bendJoint{segment}"), 0.0);
                body.set_scalar_joint_position(&format!("twistJoint{segment}"), 0.0);
            }
        }
        simulation.run_for(DURATION_NANOS);
    }
    for csv in [csv.as_mut(), att_csv.as_mut()].into_iter().flatten() {
        csv.flush().expect("flush scenario CSV");
    }
    Results {
        guidance: guidance_log.into_samples(),
        angles: angle_logs
            .into_iter()
            .map(|(bend, twist)| (bend.into_samples(), twist.into_samples()))
            .collect(),
    }
}

fn main() {
    Cli::from_env("scenario_flexible_panel_mujoco").finish();
    let results = run(true);
    let (_, last) = results.guidance.last().expect("recorded guidance");
    println!("final |sigma_BR| {:.3e}", last.sigma_br.norm());
    for (index, (bend, twist)) in results.angles.iter().enumerate() {
        let peak = |log: &[(u64, ScalarJointStateMsg)]| {
            log.iter()
                .map(|(_, state)| state.state.abs())
                .fold(0.0, f64::max)
        };
        println!(
            "segment {}: peak bend {:.3e} rad, peak twist {:.3e} rad",
            index + 1,
            peak(bend),
            peak(twist)
        );
    }
    println!("wrote {}", written_csv(FILE_STEM).display());
    println!(
        "wrote {}",
        written_csv(&format!("{FILE_STEM}AttError")).display()
    );
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! slew and the panel's flexing it demonstrates.

    use super::*;

    #[test]
    fn the_hub_reaches_its_reference_while_the_panel_flexes_and_settles() {
        let results = run(false);
        assert_eq!(results.guidance.len(), 601);
        let (_, first) = &results.guidance[0];
        let (_, last) = results.guidance.last().unwrap();
        assert!(first.sigma_br.norm() > 0.5);
        // The flexing panel makes the error ring down rather than decay
        // smoothly; by the end it is below 1% of the initial error.
        assert!(
            last.sigma_br.norm() < 1e-2 * first.sigma_br.norm(),
            "{}",
            last.sigma_br
        );
        for (index, (bend, twist)) in results.angles.iter().enumerate() {
            assert_eq!(bend.len(), 1201);
            assert_eq!(twist.len(), 1201);
            let peak = |log: &[(u64, ScalarJointStateMsg)], since: u64| {
                log.iter()
                    .filter(|(time, _)| *time >= since)
                    .map(|(_, state)| state.state.abs())
                    .fold(0.0, f64::max)
            };
            // The slew bends and twists the panel, and the dampers settle
            // it once the hub is at rest.
            let during = peak(bend, 0).max(peak(twist, 0));
            assert!(
                during > 1e-3 && during < 1.0,
                "segment {}: {during}",
                index + 1
            );
            let settled = peak(bend, seconds(500)).max(peak(twist, seconds(500)));
            assert!(
                settled < 0.3 * during,
                "segment {}: {settled} vs {during}",
                index + 1
            );
        }
    }
}
