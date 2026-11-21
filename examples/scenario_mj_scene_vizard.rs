//! Two `MJScene`s and a regular spacecraft in one Vizard visualization: port
//! of upstream `examples/mujoco/scenarioMJSceneVizard.py`. Needs the `mujoco`
//! and `vizard` features, and the ANISE assets (`just fetch-anise-assets`).
//!
//! On a circular 500 km orbit (i = 28.5°) around Earth, with Earth (central),
//! Moon and Sun point-mass gravity:
//!
//! * the **primary hub**, an `MJScene` of `sat_w_deployable_panels.xml`
//!   (red, six panels), deploying its panels with upstream's Python PID
//!   ([`PidController`], gains 1, 0.2 and 0.0001, clipped to ±10 N m)
//!   along `scenarioDeployPanels`' trapezoidal profiles, with a four-sensor
//!   coarse-sun-sensor pyramid on the hub;
//! * the **companion hub**, an `MJScene` of `sat_w_companion.xml` (blue, two
//!   panels), 15 m ahead along-track, deploying its panels the same way,
//!   with a Vizard star tracker on the hub;
//! * the **trailer**, a regular [`Spacecraft`], 15 m behind, with a Vizard
//!   transceiver.
//!
//! Each scene integrates with RKF45 at tolerances 1e-5 and `extraEoMCall`,
//! its gravity from its own `NBodyGravity` on every body, and the planet
//! ephemerides in its dynamics task at priority 75 (after the forward
//! kinematics, before the gravity). Task step 4 s, three quarters of an
//! orbit.
//!
//! The run writes the Vizard file upstream writes when its `saveFile` line
//! is uncommented, `examples/output/vizard/scenarioMJSceneVizard_UnityViz.bin`:
//! every scene body a spacecraft ([`mj_viz::add_scene`], upstream
//! `vizSupport` for an `MJScene`), the panels as child spacecraft of their
//! hubs with primitive models from their geoms, the CSS, star tracker and
//! transceiver with upstream's instrument settings, and orbit lines. It also
//! writes `examples/output/scenarios/scenarioMJSceneVizard.csv`: the three
//! spacecraft states and the eight panel angles, every step. Upstream makes
//! no plots, so there is no plot script.
//!
//! Stand-ins: as in the other basilisk-rs scenarios (see
//! `support/grav_factory.rs`), the SPICE states come from ANISE (DE440s),
//! one ephemeris module per body and per task (upstream runs one
//! `spiceObject` in both dynamics tasks and the top-level task); upstream's
//! Vizard `GenericSensor` and `Transceiver` are basilisk-rs's equivalents.
//!
//! Upstream `run(showPlots)` has no scenario parameters, and
//! `test_scenarioMujoco.py` only checks that it runs; `tests` checks the
//! formation and the deployments, and the Vizard spacecraft list.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scenarios.rs"]
mod mj_scenarios;
#[path = "support/mj_viz.rs"]
mod mj_viz;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;
#[path = "support/viz.rs"]
mod viz;

use std::f64::consts::PI;

use basilisk_rs::connect;
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::messages::{Output, PlanetStateMsg, ScalarJointStateMsg, SpacecraftStateMsg};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::interpolators::ScalarJointStateInterpolator;
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::vizard::viz_structures::{
    GenericSensor, InstrumentGuiSettings, Transceiver, VizCoarseSunSensor, VizSetting,
};
use cli::Cli;
use common::seconds;
use grav_factory::{Body, PLANETARY_SPK};
use hifitime::Epoch;
use message_log::MessageLog;
use mj_scenarios::{
    GravitySourceSpec, PidController, add_gravity_to_all_bodies, load_scene, rkf45, scalar_column,
    site_state_columns, trapezoid_profile, written_csv,
};
use nalgebra::{Matrix3, UnitQuaternion, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::scenario_csv;
use shared::Shared;

/// Upstream `taskPeriodSec`.
const STEP_NANOS: u64 = seconds(4);
/// Upstream `SC_SEPARATION_M`.
const SC_SEPARATION_M: f64 = 15.0;
/// Upstream `JOINT_START_END` \[rad\], by panel tier.
const JOINT_START_END: [(f64, f64); 3] = [(PI / 2.0, 0.0), (PI, 0.0), (PI, 0.0)];
/// Upstream `spiceDynamicsPriority`.
const SPICE_DYNAMICS_PRIORITY: i32 = 75;
/// Upstream's panels: `(id prefix, (side, tier) list)` per scene.
const PRIMARY_PANELS: [(char, usize); 6] =
    [('p', 1), ('p', 2), ('p', 3), ('n', 1), ('n', 2), ('n', 3)];
const COMPANION_PANELS: [(char, usize); 2] = [('p', 1), ('n', 1)];
const FILE_STEM: &str = "scenarioMJSceneVizard";

/// Upstream's SPICE epoch (UTC).
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2025, 11, 15, 12, 0, 0, 0)
}

/// Upstream `PIDController`: `scenarioDeployPanels`' Python PID with this
/// scenario's gains and its ±10 N m clip (against a transient
/// adaptive-integrator probe tripping MuJoCo's control check).
fn pid(name: String) -> PidController {
    let mut controller = PidController::new(name);
    controller.k_p = 1.0;
    controller.k_d = 0.2;
    controller.k_i = 0.0001;
    controller.output_limit = Some(10.0);
    controller
}

/// Upstream `addDeployment(scene, panelIds, bodyPrefix, ...)`: each joint
/// tracks its profile; returns the joint angle outputs.
fn add_deployment(
    scene: &mut MjScene,
    panels: &[(char, usize)],
    prefix: &str,
) -> Vec<(String, Output<ScalarJointStateMsg>)> {
    let mut angles = Vec::new();
    for &(side, tier) in panels {
        let actuator = format!("{prefix}_{side}{tier}_deploy");
        let (start, end) = JOINT_START_END[tier - 1];
        let (positions, velocities) = trapezoid_profile(
            start,
            end,
            0.05_f64.to_radians(),
            0.0001_f64.to_radians(),
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
        let mut controller = pid(format!("{actuator}_ctrl"));
        position
            .interpolated_out_msg
            .connect_to(&mut controller.desired_in_msg);
        velocity
            .interpolated_out_msg
            .connect_to(&mut controller.desired_dot_in_msg);
        let joint = scene.scalar_joint(&actuator).clone();
        joint
            .state_out_msg
            .connect_to(&mut controller.measured_in_msg);
        joint
            .state_dot_out_msg
            .connect_to(&mut controller.measured_dot_in_msg);
        controller
            .output_out_msg
            .connect_to(&mut scene.single_actuator_mut(&actuator).actuator_in_msg);
        scene.add_model_to_dynamics_task(position, 50);
        scene.add_model_to_dynamics_task(velocity, 49);
        scene.add_model_to_dynamics_task(controller, 25);
        angles.push((actuator, joint.state_out_msg.clone()));
    }
    angles
}

/// A scene with its ephemerides and gravity: RKF45 at 1e-5, `extraEoMCall`,
/// the Earth, Moon and Sun states (relative to Earth, upstream `zeroBase`)
/// in the dynamics task, and `gravFactory.addBodiesTo(scene)`.
fn prepare_scene(file: &str, name: &str) -> MjScene {
    let mut scene = load_scene(file);
    scene.name = name.to_string();
    scene.set_integrator(rkf45(1e-5, 1e-5));
    scene.extra_eom_call = true;
    let mut states = Vec::new();
    for body in [Body::Earth, Body::Moon, Body::Sun] {
        let ephemeris = body.ephemeris(Body::Earth);
        states.push(ephemeris.output_planet_msg.clone());
        scene.add_module_to_dynamics_task(
            &format!("{}_spice", body.name()),
            ephemeris,
            SPICE_DYNAMICS_PRIORITY,
        );
    }
    add_gravity_to_all_bodies(
        &mut scene,
        &[
            GravitySourceSpec {
                name: "earth",
                mu_m3ps2: Body::Earth.mu_m3ps2(),
                is_central_body: true,
                ephemeris: Some(&states[0]),
            },
            GravitySourceSpec {
                name: "moon",
                mu_m3ps2: Body::Moon.mu_m3ps2(),
                is_central_body: false,
                ephemeris: Some(&states[1]),
            },
            GravitySourceSpec {
                name: "sun",
                mu_m3ps2: Body::Sun.mu_m3ps2(),
                is_central_body: false,
                ephemeris: Some(&states[2]),
            },
        ],
    );
    scene
}

/// Upstream's four hub CSS: 80° field of view, scale and maximum output 2,
/// at `r_B = (0, 0, 1)` m.
fn css_units() -> Vec<CoarseSunSensorConfig> {
    [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
    ]
    .into_iter()
    .enumerate()
    .map(|(k, normal)| CoarseSunSensorConfig {
        name: format!("hub_CSS{k}"),
        position_m: Vector3::new(0.0, 0.0, 1.0),
        body_to_sensor_quaternion: UnitQuaternion::rotation_between(&normal, &Vector3::z())
            .expect("non-opposite CSS mounting vectors"),
        fov_half_angle_rad: 80.0_f64.to_radians(),
        scale_factor: 2.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 0.0,
        walk_bounds: 0.0,
        min_output: 0.0,
        max_output: 2.0,
    })
    .collect()
}

#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    primary: Vec<(u64, SpacecraftStateMsg)>,
    companion: Vec<(u64, SpacecraftStateMsg)>,
    trailer: Vec<(u64, SpacecraftStateMsg)>,
    /// `(joint, angle history)` of every panel, primary then companion.
    panels: Vec<(String, Vec<(u64, ScalarJointStateMsg)>)>,
    vizard_spacecraft: Vec<(String, String)>,
}

fn run() -> Results {
    let mut primary = prepare_scene("sat_w_deployable_panels.xml", "primaryScene");
    let mut companion = prepare_scene("sat_w_companion.xml", "companionScene");
    let mut panel_angles = add_deployment(&mut primary, &PRIMARY_PANELS, "panel");
    panel_angles.extend(add_deployment(
        &mut companion,
        &COMPANION_PANELS,
        "companion_panel",
    ));

    // The orbit, and the formation along-track.
    let mu = Body::Earth.mu_m3ps2();
    let elements = ClassicElements {
        semi_major_axis: 6_371_000.0 + 500_000.0,
        eccentricity: 0.0,
        inclination_rad: 28.5_f64.to_radians(),
        raan_rad: 48.2_f64.to_radians(),
        arg_periapsis_rad: 347.8_f64.to_radians(),
        true_anomaly_rad: 85.3_f64.to_radians(),
        ..Default::default()
    };
    let (r_n, v_n) = elements_to_state(mu, &elements);
    let v_hat = v_n.normalize();
    let duration_nanos = sec2nano(0.75 * orbit_period_s(mu, elements.semi_major_axis));

    // The trailer, with the factory's gravity bodies.
    let mut trailer = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: r_n - SC_SEPARATION_M * v_hat,
        initial_velocity_mps: v_n,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    for (body, central) in [(Body::Earth, true), (Body::Moon, false), (Body::Sun, false)] {
        trailer
            .add_grav_body(body.point_mass(central))
            .expect("unique gravity body");
    }
    // The top-level ephemerides (upstream adds `spiceObject` to the task
    // too): the trailer's gravity, the CSS's Sun and Vizard's planets.
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);
    let mut moon_ephemeris = Body::Moon.ephemeris(Body::Earth);
    let mut sun_ephemeris = AniseSunEphemeris::new(AniseSunEphemerisConfig {
        name: "sun_spice".to_string(),
        spk_path: PLANETARY_SPK.into(),
    });
    let moon_state: Output<PlanetStateMsg> = moon_ephemeris.output_planet_msg.clone();
    let sun_state: Output<PlanetStateMsg> = sun_ephemeris.output_planet_msg.clone();
    moon_state.connect_to(trailer.grav_body_input_mut("moon").expect("the Moon"));
    sun_state.connect_to(trailer.grav_body_input_mut("sun").expect("the Sun"));

    // The CSS pyramid on the primary hub.
    let hub_state = primary.body("hub").origin().state_out_msg.clone();
    let mut sensors: Vec<CoarseSunSensor> =
        css_units().into_iter().map(CoarseSunSensor::new).collect();
    for sensor in &mut sensors {
        hub_state.connect_to(&mut sensor.input_state_msg);
        sun_ephemeris
            .output_sun_msg
            .connect_to(&mut sensor.input_sun_msg);
    }

    // Vizard: both scenes and the trailer, with their instruments.
    let mut vizard = viz::saving_interface(viz::vizard_file(&format!("{FILE_STEM}_UnityViz")));
    mj_viz::add_scene(&mut vizard, &primary);
    mj_viz::add_scene(&mut vizard, &companion);
    trailer
        .state_out
        .connect_to(&mut vizard.add_spacecraft("trailer").sc_state_in_msg);
    {
        let hub = mj_viz::spacecraft_mut(&mut vizard, "hub");
        for (sensor, config) in sensors.iter().zip(css_units()) {
            sensor.output_sun_sensor_msg.connect_to(
                hub.add_coarse_sun_sensor(VizCoarseSunSensor::from_sensor_config(&config)),
            );
        }
    }
    mj_viz::spacecraft_mut(&mut vizard, "companion_hub")
        .generic_sensors
        .push(GenericSensor {
            r_sb_b_m: Vector3::new(0.0, 0.0, 1.0),
            field_of_view_rad: vec![8.0_f64.to_radians(), 8.0_f64.to_radians()],
            normal_vector: Vector3::new(0.0, 0.0, 1.0),
            size_m: 1.5,
            color: vec![mj_viz::rgba("yellow")],
            label: "STR".to_string(),
            generic_sensor_cmd: 1,
            ..GenericSensor::default()
        });
    mj_viz::spacecraft_mut(&mut vizard, "trailer")
        .transceivers
        .push(Transceiver {
            r_sb_b_m: Vector3::new(0.0, 0.0, 1.0),
            field_of_view_rad: 30.0_f64.to_radians(),
            normal_vector: Vector3::new(0.0, 0.0, 1.0),
            color: Some(mj_viz::rgba("cyan")),
            label: "TX".to_string(),
            animation_speed: 1,
            ..Transceiver::default()
        });
    vizard.settings.instrument_gui_settings_list.extend([
        InstrumentGuiSettings {
            spacecraft_name: "hub".to_string(),
            view_css_panel: VizSetting::Enabled,
            view_css_coverage: VizSetting::Enabled,
            view_css_boresight: VizSetting::Enabled,
            show_css_labels: VizSetting::Enabled,
            ..InstrumentGuiSettings::default()
        },
        InstrumentGuiSettings {
            spacecraft_name: "companion_hub".to_string(),
            show_generic_sensor_labels: VizSetting::Enabled,
            ..InstrumentGuiSettings::default()
        },
        InstrumentGuiSettings {
            spacecraft_name: "trailer".to_string(),
            show_transceiver_labels: VizSetting::Enabled,
            show_transceiver_frustum: VizSetting::Enabled,
            ..InstrumentGuiSettings::default()
        },
    ]);
    vizard.settings.orbit_lines_on = 2;
    for (body, state) in [
        (Body::Earth, &earth_ephemeris.output_planet_msg),
        (Body::Moon, &moon_state),
        (Body::Sun, &sun_state),
    ] {
        state.connect_to(vizard.add_celestial_body(viz::grav_body_info(body)));
    }
    let vizard_spacecraft = vizard
        .spacecraft
        .iter()
        .map(|spacecraft| {
            (
                spacecraft.spacecraft_name.clone(),
                spacecraft.parent_spacecraft_name.clone(),
            )
        })
        .collect();

    let companion_state = companion
        .body("companion_hub")
        .origin()
        .state_out_msg
        .clone();
    let trailer_state = trailer.state_out.clone();
    let mut primary_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut companion_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut trailer_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut panel_logs: Vec<MessageLog<ScalarJointStateMsg>> =
        panel_angles.iter().map(|_| MessageLog::default()).collect();
    let mut csv = scenario_csv(FILE_STEM);
    let primary = Shared::new(primary);
    let companion = Shared::new(companion);
    let mut primary_task = primary.handle();
    let mut companion_task = companion.handle();
    {
        let mut simulation = Simulation::new(epoch(), true);
        connect!(&simulation,
            &hub_state => &mut primary_log.input,
            &companion_state => &mut companion_log.input,
            &trailer_state => &mut trailer_log.input,
            &hub_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("primary_hub")),
            &companion_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("companion_hub")),
            &trailer_state => csv.add_source::<SpacecraftStateMsg>(site_state_columns("trailer")),
        );
        for ((joint, angle), log) in panel_angles.iter().zip(panel_logs.iter_mut()) {
            connect!(&simulation,
                angle => &mut log.input,
                angle => csv.add_source::<ScalarJointStateMsg>(
                    scalar_column("state", &format!("{joint}_rad")),
                ),
            );
        }
        // Upstream's task, in insertion order.
        simulation.add_module("primaryScene", &mut primary_task, STEP_NANOS, 100);
        simulation.add_module("companionScene", &mut companion_task, STEP_NANOS, 99);
        simulation.add_module("earthSpice", &mut earth_ephemeris, STEP_NANOS, 98);
        simulation.add_module("moonSpice", &mut moon_ephemeris, STEP_NANOS, 98);
        simulation.add_module("sunSpice", &mut sun_ephemeris, STEP_NANOS, 98);
        simulation.add_module("trailer", &mut trailer, STEP_NANOS, 97);
        for sensor in &mut sensors {
            simulation.add_module(sensor.config.name.clone(), sensor, STEP_NANOS, 96);
        }
        simulation.add_module("primaryLog", &mut primary_log, STEP_NANOS, 0);
        simulation.add_module("companionLog", &mut companion_log, STEP_NANOS, 0);
        simulation.add_module("trailerLog", &mut trailer_log, STEP_NANOS, 0);
        for (index, log) in panel_logs.iter_mut().enumerate() {
            simulation.add_module(format!("panelLog{index}"), log, STEP_NANOS, 0);
        }
        simulation.add_module("csv", &mut csv, STEP_NANOS, 0);
        simulation.add_module("vizInterface", &mut vizard, STEP_NANOS, -10);
        simulation.initialize();
        {
            // The panels stowed, the hubs on the orbit.
            let mut primary = primary.lock();
            for (side, tier) in PRIMARY_PANELS {
                let body = format!("panel_{side}{tier}");
                primary.body_mut(&body).set_scalar_joint_position(
                    &format!("{body}_deploy"),
                    JOINT_START_END[tier - 1].0,
                );
            }
            let mut hub = primary.body_mut("hub");
            hub.set_position(r_n);
            hub.set_velocity(v_n);
        }
        {
            let mut companion = companion.lock();
            for (side, tier) in COMPANION_PANELS {
                let body = format!("companion_panel_{side}{tier}");
                companion.body_mut(&body).set_scalar_joint_position(
                    &format!("{body}_deploy"),
                    JOINT_START_END[tier - 1].0,
                );
            }
            let mut hub = companion.body_mut("companion_hub");
            hub.set_position(r_n + SC_SEPARATION_M * v_hat);
            hub.set_velocity(v_n);
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        primary: primary_log.into_samples(),
        companion: companion_log.into_samples(),
        trailer: trailer_log.into_samples(),
        panels: panel_angles
            .into_iter()
            .map(|(joint, _)| joint)
            .zip(panel_logs.into_iter().map(MessageLog::into_samples))
            .collect(),
        vizard_spacecraft,
    }
}

fn main() {
    Cli::from_env("scenario_mj_scene_vizard").finish();
    let results = run();
    let last = results.primary.len() - 1;
    println!(
        "separations at the end: companion {:.2} m, trailer {:.2} m",
        (results.companion[last].1.position_m - results.primary[last].1.position_m).norm(),
        (results.trailer[last].1.position_m - results.primary[last].1.position_m).norm()
    );
    println!(
        "wrote {}",
        std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join(format!("examples/output/vizard/{FILE_STEM}_UnityViz.bin"))
            .display()
    );
    println!("wrote {}", written_csv(FILE_STEM).display());
}

#[cfg(test)]
mod tests {
    //! `test_scenarioMujoco.py` only runs the scenario. These check the
    //! scene it builds: the Vizard spacecraft, the formation and the
    //! deployments.

    use super::*;

    #[test]
    fn the_formation_flies_and_the_panels_deploy() {
        let results = run();
        // Upstream's emitted spacecraft: each scene's world-attached hub,
        // then its other bodies with their MuJoCo parents, then the trailer.
        let names: Vec<&str> = results
            .vizard_spacecraft
            .iter()
            .map(|(name, _)| name.as_str())
            .collect();
        assert_eq!(names[0], "hub");
        assert!(names.contains(&"panel_p3") && names.contains(&"wheel_1"));
        assert!(names.contains(&"companion_hub") && names.ends_with(&["trailer"]));
        let parent = |name: &str| {
            results
                .vizard_spacecraft
                .iter()
                .find(|(spacecraft, _)| spacecraft == name)
                .map(|(_, parent)| parent.clone())
                .unwrap()
        };
        assert_eq!(parent("hub"), "");
        assert_eq!(parent("panel_p2"), "panel_p1");
        assert_eq!(parent("companion_panel_n1"), "companion_hub");
        assert_eq!(results.vizard_spacecraft.len(), 8 + 3 + 1);

        // The three spacecraft keep roughly their 15 m along-track spacing
        // (the 500 km orbit uses `6371 km + 500 km` with astroConstants' mu,
        // so the formation drifts only slowly).
        let last = results.primary.len() - 1;
        let primary = results.primary[last].1.position_m;
        let ahead = (results.companion[last].1.position_m - primary).norm();
        let behind = (results.trailer[last].1.position_m - primary).norm();
        assert!(ahead > 10.0 && ahead < 25.0, "{ahead}");
        assert!(behind > 10.0 && behind < 25.0, "{behind}");
        assert!(primary.norm() > 6_371_000.0 + 400_000.0);
        // The profiles end within the run: every panel ends deployed.
        for (joint, log) in &results.panels {
            let (_, last) = log.last().unwrap();
            assert!(
                last.state.to_degrees().abs() < 5.0,
                "{joint}: {}",
                last.state
            );
        }
    }
}
