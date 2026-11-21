//! Two independently moving camera platforms on a spacecraft: port of
//! upstream `examples/scenarioMovingCamera.py`.
//!
//! A 100 kg host (`I = diag(20, 30, 40) kg m²`, "cameraHost") on a circular
//! 500 km Earth orbit (point-mass Earth) holds the inertial attitude
//! `sigma_BN = [0.1, -0.2, 0.05]`, prescribed through its `attRefInMsg`,
//! with both cameras' `+z` axes towards the Earth at the start. Each camera
//! sits on a massless `prescribedMotionStateEffector` platform P mounted at
//! `r_MB_B = [±1, 0, 0] m` (mount attitudes `[0, 0, 0]` and `[0, 0, 0.1]`):
//! a `prescribedRotation1DOF` profiler (0.1 rad/s²) turns platform 1 about
//! its mount `y` axis and platform 2 about `x`, and a
//! `prescribedLinearTranslation` profiler (0.05 m/s²) moves platform 1
//! along its mount `x` axis. The commands enter as messages: 20° and -30°
//! and 0.3 m first; at 15 s, -10° and 40° and -0.2 m. One 0.1 s task,
//! 30 s, every state recorded.
//!
//! The profilers run after the spacecraft here (upstream adds them before
//! it): basilisk-rs's prescribed-motion effector takes its inputs as the
//! state at the start of each integration step, so it reads the profile of
//! the previous tick, as its docs ask. Once a profile has settled, as at the
//! checked times, the poses are the same.
//!
//! Upstream `run(show_plots, enable_viz)` maps to `--viz`, which needs the
//! `vizard` feature: the host and the two platforms (their poses from the
//! platforms' `prescribedMotionConfigLogOutMsg`, parented to the host, drawn
//! as grey boxes) and a 0.2 rad, 256 × 256 pixel camera on each platform
//! (parented to the platform) are saved to
//! `examples/output/vizard/scenarioMovingCamera.bin`, with the camera
//! frustums shown. The script's default is `--viz` (here: when built with
//! the feature); `--all` runs the core case and, with the feature, the
//! visualization case, as upstream's `test_scenarioMovingCamera.py`. Writes
//! `examples/output/scenarios/scenarioMovingCamera.csv`.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[cfg(feature = "vizard")]
#[path = "support/viz.rs"]
mod viz;

use basilisk_rs::device_interface::prescribed_linear_translation::{
    PrescribedLinearTranslation, PrescribedLinearTranslationConfig,
};
use basilisk_rs::device_interface::prescribed_rotation_1dof::{
    PrescribedRotation1Dof, PrescribedRotation1DofConfig,
};
use basilisk_rs::dynamics::prescribed_motion_state_effector::{
    PrescribedMotionConfig, PrescribedMotionStateEffector,
};
use basilisk_rs::messages::{
    AttitudeReferenceMsg, HingedRigidBodyMsg, LinearTranslationRigidBodyMsg, Output,
    PrescribedRotationMsg, PrescribedTranslationMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{scenario_output_path, vector_columns};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};

/// Upstream `step_s = 0.1`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `switch_time_s = 15.0`.
const SWITCH_NANOS: u64 = 15_000_000_000;
/// Upstream `final_time_s = 30.0`.
const FINAL_NANOS: u64 = 30_000_000_000;
/// Upstream `hub_sigma`.
const HUB_SIGMA: [f64; 3] = [0.1, -0.2, 0.05];
/// Upstream `offsets` [m]: mount positions in B.
const OFFSETS_M: [[f64; 3]; 2] = [[1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]];
/// Upstream `axes`: rotation axes in M.
const AXES: [[f64; 3]; 2] = [[0.0, 1.0, 0.0], [1.0, 0.0, 0.0]];
/// Upstream `mount_sigmas`.
const MOUNT_SIGMAS: [[f64; 3]; 2] = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.1]];
/// Upstream `commands_rad` [deg], by stage and camera.
const COMMANDS_DEG: [[f64; 2]; 2] = [[20.0, -30.0], [-10.0, 40.0]];
/// Upstream `translations_m` [m], by stage.
const TRANSLATIONS_M: [f64; 2] = [0.3, -0.2];
/// Upstream `altitude_m`.
const ALTITUDE_M: f64 = 500.0e3;

/// The recorded states (upstream's returned `results` dictionary).
// Some fields are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    hub: MessageLog<SpacecraftStateMsg>,
    platforms: Vec<MessageLog<SpacecraftStateMsg>>,
    rotations: Vec<MessageLog<PrescribedRotationMsg>>,
    translation: MessageLog<PrescribedTranslationMsg>,
    /// Upstream `camera_parents`: the cameras' parent names; empty without
    /// the visualization.
    camera_parents: Vec<String>,
    /// From the last frame of the Vizard file: upstream `viz_parents`,
    /// `viz_camera_parents` and `viz_platform_sigma`.
    viz_parents: Vec<String>,
    viz_camera_parents: Vec<String>,
    viz_platform_sigma: Vec<Vector3<f64>>,
}

fn run(enable_viz: bool, write_csv: bool) -> Results {
    let earth = Body::Earth;
    let orbit_radius_m = earth.radius_equator_m() + ALTITUDE_M;
    let dcm_nb = mrp_to_dcm(Vector3::from(HUB_SIGMA)).transpose();
    // The camera +z axes towards the Earth; the velocity along hub +x.
    let mut hub = Spacecraft::new(SpacecraftConfig {
        mass_kg: 100.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(20.0, 30.0, 40.0)),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: -orbit_radius_m * dcm_nb.column(2),
        initial_velocity_mps: (earth.mu_m3ps2() / orbit_radius_m).sqrt() * dcm_nb.column(0),
        initial_sigma_bn: Vector3::from(HUB_SIGMA),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    hub.add_grav_body(earth.point_mass(true))
        .expect("one central body");
    let attitude_msg = Output::new(AttitudeReferenceMsg {
        sigma_rn: Vector3::from(HUB_SIGMA),
        ..AttitudeReferenceMsg::default()
    });

    let mut rotations: Vec<PrescribedRotation1Dof> = (0..2)
        .map(|index| {
            PrescribedRotation1Dof::new(PrescribedRotation1DofConfig {
                name: format!("cameraRotation{}", index + 1),
                rot_hat_m: Vector3::from(AXES[index]),
                theta_ddot_max_radps2: 0.1,
                ..PrescribedRotation1DofConfig::default()
            })
        })
        .collect();
    let rotation_msgs: Vec<Output<HingedRigidBodyMsg>> = (0..2)
        .map(|index| {
            Output::new(HingedRigidBodyMsg {
                theta_rad: COMMANDS_DEG[0][index].to_radians(),
                ..HingedRigidBodyMsg::default()
            })
        })
        .collect();
    let mut translation = PrescribedLinearTranslation::new(PrescribedLinearTranslationConfig {
        name: "cameraTranslation1".to_string(),
        trans_hat_m: Vector3::x(),
        trans_accel_max_mps2: 0.05,
        ..PrescribedLinearTranslationConfig::default()
    });
    let translation_msg = Output::new(LinearTranslationRigidBodyMsg {
        rho_m: TRANSLATIONS_M[0],
        ..LinearTranslationRigidBodyMsg::default()
    });

    let mut hub_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut platform_logs: Vec<MessageLog<SpacecraftStateMsg>> =
        (0..2).map(|_| MessageLog::default()).collect();
    let mut rotation_logs: Vec<MessageLog<PrescribedRotationMsg>> =
        (0..2).map(|_| MessageLog::default()).collect();
    let mut translation_log = MessageLog::<PrescribedTranslationMsg>::default();
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: "scenarioMovingCamera".to_string(),
        output_path: if write_csv {
            scenario_output_path("scenarioMovingCamera.csv")
        } else {
            Default::default()
        },
    })
    .with_format(CsvFormat::Scenario);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
    let mut platform_poses = Vec::new();
    for index in 0..2 {
        let mut platform = PrescribedMotionStateEffector::new(PrescribedMotionConfig {
            mass_kg: 0.0,
            inertia_about_com_p_kg_m2: Matrix3::zeros(),
            center_of_mass_p_m: Vector3::zeros(),
            mount_position_body_m: Vector3::from(OFFSETS_M[index]),
            sigma_mb: Vector3::from(MOUNT_SIGMAS[index]),
            ..PrescribedMotionConfig::new(format!("cameraPlatform{}", index + 1))
        });
        connect!(&simulation,
            &rotation_msgs[index] => &mut rotations[index].spinning_body_in_msg,
            &rotations[index].prescribed_rotation_out_msg => &mut platform.prescribed_rotation_in,
            &rotations[index].prescribed_rotation_out_msg => &mut rotation_logs[index].input,
            &platform.prescribed_motion_state_out => &mut platform_logs[index].input,
            &platform.prescribed_motion_state_out => recorder.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_m", &format!("platform{}_r_PN_N_m", index + 1))
                        .into_iter()
                        .chain(vector_columns("sigma_bn", &format!("platform{}_sigma_PN", index + 1))),
                ),
            ),
        );
        if index == 0 {
            connect!(&simulation,
                &translation_msg => &mut translation.linear_translation_rigid_body_in_msg,
                &translation.prescribed_translation_out_msg => &mut platform.prescribed_translation_in,
            );
        }
        platform_poses.push(platform.prescribed_motion_state_out.clone());
        hub.add_state_effector(platform);
    }
    connect!(&simulation,
        &attitude_msg => &mut hub.att_ref_in,
        &hub.state_out => &mut hub_log.input,
        &translation.prescribed_translation_out_msg => &mut translation_log.input,
        &hub.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
                    .chain(vector_columns("sigma_bn", "sigma_BN")),
            ),
        ),
        &translation.prescribed_translation_out_msg => recorder.add_source::<PrescribedTranslationMsg>(
            CsvSourceConfig::columns(vector_columns("r_pm_m_m", "platform1_r_PM_M_m")),
        ),
    );

    #[cfg(not(feature = "vizard"))]
    assert!(
        !enable_viz,
        "--viz needs the vizard feature (--features vizard)"
    );
    #[cfg(feature = "vizard")]
    let vizard_path = enable_viz.then(|| {
        if write_csv {
            viz::vizard_file("scenarioMovingCamera")
        } else {
            // A file of its own for each test run, removed once read.
            static RUN: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
            let run = RUN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            viz::vizard_file(&format!(
                "scenarioMovingCamera_test{}_{run}",
                std::process::id()
            ))
        }
    });
    #[cfg(feature = "vizard")]
    let camera_msgs: Vec<Output<basilisk_rs::messages::CameraConfigMsg>> = (0..2)
        .map(|index| {
            // Upstream `vizSupport.createCameraConfigMsg(viz, cameraID, parentName, ...)`.
            Output::new(basilisk_rs::messages::CameraConfigMsg {
                camera_id: (index + 1) as i64,
                is_on: true,
                parent_name: format!("cameraPlatform{}", index + 1),
                field_of_view_rad: 0.2,
                resolution_px: [256, 256],
                render_rate_nanos: STEP_NANOS,
                camera_pos_b_m: Vector3::zeros(),
                sigma_cb: Vector3::zeros(),
                ..Default::default()
            })
        })
        .collect();
    #[cfg(feature = "vizard")]
    let mut vizard = vizard_path.clone().map(|path| {
        use basilisk_rs::vizard::viz_structures::{CustomModel, VizSetting};
        let mut vizard = viz::saving_interface(path);
        vizard.settings.view_camera_frustum_hud = VizSetting::Enabled;
        vizard.add_celestial_body(viz::grav_body_info(earth));
        let host = vizard.add_spacecraft("cameraHost");
        simulation.connect(&hub.state_out, &mut host.sc_state_in_msg);
        for (index, pose) in platform_poses.iter().enumerate() {
            let platform = vizard.add_spacecraft(format!("cameraPlatform{}", index + 1));
            platform.parent_spacecraft_name = "cameraHost".to_string();
            simulation.connect(pose, &mut platform.sc_state_in_msg);
        }
        vizard.settings.custom_model_list.push(CustomModel {
            model_path: "CUBE".to_string(),
            sim_bodies_to_modify: vec![
                "cameraPlatform1".to_string(),
                "cameraPlatform2".to_string(),
            ],
            scale: Vector3::new(0.3, 0.3, 0.2),
            offset_m: Vector3::new(0.0, 0.0, -0.1),
            color: Some([128, 128, 128, 255]),
            ..CustomModel::default()
        });
        for camera_msg in &camera_msgs {
            let camera = vizard.add_camera();
            simulation.connect(camera_msg, &mut camera.camera_config_in_msg);
        }
        vizard
    });
    #[cfg(not(feature = "vizard"))]
    let _ = &platform_poses;

    let [rotation_1, rotation_2] = &mut rotations[..] else {
        unreachable!("two platforms")
    };
    // Profilers after the spacecraft; see the module docs.
    schedule! { simulation,
        "cameraHost" => &mut hub, period=STEP_NANOS, priority=20;
        "cameraRotation1" => rotation_1, period=STEP_NANOS, priority=15;
        "cameraRotation2" => rotation_2, period=STEP_NANOS, priority=15;
        "cameraTranslation1" => &mut translation, period=STEP_NANOS, priority=15;
        "hub_log" => &mut hub_log, period=STEP_NANOS, priority=-10;
        "translation_log" => &mut translation_log, period=STEP_NANOS, priority=-10;
    }
    for log in platform_logs.iter_mut() {
        simulation.add_module("platform_log", log, STEP_NANOS, -10);
    }
    for log in rotation_logs.iter_mut() {
        simulation.add_module("rotation_log", log, STEP_NANOS, -10);
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, STEP_NANOS, -10);
    }
    #[cfg(feature = "vizard")]
    if let Some(vizard) = vizard.as_mut() {
        simulation.add_module("vizInterface", vizard, STEP_NANOS, -20);
    }

    simulation.run_for(SWITCH_NANOS);
    // Retarget by writing the command messages.
    for (index, reference) in rotation_msgs.iter().enumerate() {
        reference.write(HingedRigidBodyMsg {
            theta_rad: COMMANDS_DEG[1][index].to_radians(),
            ..HingedRigidBodyMsg::default()
        });
    }
    translation_msg.write(LinearTranslationRigidBodyMsg {
        rho_m: TRANSLATIONS_M[1],
        ..LinearTranslationRigidBodyMsg::default()
    });
    simulation.run_for(FINAL_NANOS - SWITCH_NANOS);
    drop(simulation);
    drop(recorder);

    #[cfg_attr(not(feature = "vizard"), allow(unused_mut))]
    let mut results = Results {
        hub: hub_log,
        platforms: platform_logs,
        rotations: rotation_logs,
        translation: translation_log,
        camera_parents: Vec::new(),
        viz_parents: Vec::new(),
        viz_camera_parents: Vec::new(),
        viz_platform_sigma: Vec::new(),
    };
    #[cfg(feature = "vizard")]
    if let Some(path) = vizard_path {
        // The interface flushes its file when it drops.
        drop(vizard);
        let frames =
            basilisk_rs::vizard::viz_interface::read_viz_file(&path).expect("readable Vizard file");
        let last = frames.last().expect("Vizard frames");
        results.camera_parents = camera_msgs
            .iter()
            .map(|msg| msg.read().parent_name)
            .collect();
        results.viz_parents = last
            .spacecraft
            .iter()
            .map(|spacecraft| spacecraft.parent_spacecraft_name.clone())
            .collect();
        results.viz_camera_parents = last
            .cameras
            .iter()
            .map(|camera| camera.parent_name.clone())
            .collect();
        results.viz_platform_sigma = last.spacecraft[1..]
            .iter()
            .map(|spacecraft| Vector3::from_column_slice(&spacecraft.rotation))
            .collect();
        if write_csv {
            println!("wrote {}", path.display());
        } else {
            std::fs::remove_file(&path).expect("remove the test's Vizard file");
        }
    }
    results
}

fn main() {
    let mut cli = Cli::from_env("scenario_moving_camera [--[no-]viz] [--all]");
    let cases = if cli.switch("all") {
        let mut cases = vec![false];
        if cfg!(feature = "vizard") {
            cases.push(true);
        }
        cases
    } else {
        // Upstream's `__main__` runs `run(show_plots=True)`, visualization on.
        vec![cli.flag("viz", cfg!(feature = "vizard"))]
    };
    cli.finish();
    for enable_viz in cases {
        let results = run(enable_viz, true);
        let last = results.platforms[0].last();
        println!(
            "wrote examples/output/scenarios/scenarioMovingCamera.csv (platform 1 at {:.3?} m)",
            last.position_m.as_slice()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `test_scenarioMovingCamera.py`: the settled platform
    //! poses against the commanded angles and displacements (DCM tolerance
    //! 2e-8, position tolerance 10 nm, absolute), the host's circular orbit
    //! at fixed attitude against the Kepler solution (1e-6 m), the moving
    //! boresights, and, with the `vizard` feature, the Vizard parent wiring
    //! and platform poses (from the saved file's last frame, 1e-14).

    use basilisk_rs::utilities::rigid_body_kinematics::prv_to_dcm;

    use super::*;

    fn cases() -> Vec<bool> {
        if cfg!(feature = "vizard") {
            vec![false, true]
        } else {
            vec![false]
        }
    }

    #[test]
    fn test_camera_platform_commands() {
        for enable_viz in cases() {
            let data = run(enable_viz, false);
            let switch = data
                .hub
                .times_nanos
                .iter()
                .position(|time| *time >= SWITCH_NANOS)
                .expect("switch sample");
            for (stage, sample) in [switch, data.hub.values.len() - 1].into_iter().enumerate() {
                let hub = &data.hub.values[sample];
                let dcm_bn = mrp_to_dcm(hub.sigma_bn);
                for camera in 0..2 {
                    let axis = Vector3::from(AXES[camera]);
                    let dcm_pm = prv_to_dcm(axis * COMMANDS_DEG[stage][camera].to_radians());
                    let dcm_mb = mrp_to_dcm(Vector3::from(MOUNT_SIGMAS[camera]));
                    let expected = dcm_pm * dcm_mb * dcm_bn;
                    let platform = &data.platforms[camera].values[sample];
                    let actual = mrp_to_dcm(platform.sigma_bn);
                    let error = (actual - expected).abs().max();
                    assert!(error <= 2e-8, "stage {stage} camera {camera}: {error}");

                    let mut displacement_m = Vector3::zeros();
                    if camera == 0 {
                        displacement_m.x = TRANSLATIONS_M[stage];
                    }
                    let position = hub.position_m
                        + dcm_bn.transpose()
                            * (Vector3::from(OFFSETS_M[camera])
                                + dcm_mb.transpose() * displacement_m);
                    let error = (platform.position_m - position).abs().max();
                    assert!(error <= 1e-8, "stage {stage} camera {camera}: {error} m");

                    // The profilers' settled references (upstream's
                    // `rotation_reference` and `translation_reference`).
                    let sigma_pm = data.rotations[camera].values[sample].sigma_pm;
                    assert!((mrp_to_dcm(sigma_pm) - dcm_pm).abs().max() <= 2e-8);
                    if camera == 0 {
                        let r_pm_m = data.translation.values[sample].r_pm_m_m;
                        assert!((r_pm_m - displacement_m).abs().max() <= 1e-8);
                    }
                }
            }
        }
    }

    #[test]
    fn test_camera_host_follows_orbit() {
        for enable_viz in cases() {
            let data = run(enable_viz, false);
            for state in &data.hub.values {
                assert!((state.sigma_bn - Vector3::from(HUB_SIGMA)).abs().max() <= 1e-14);
            }
            let earth = Body::Earth;
            let initial = &data.hub.values[0];
            let orbit_radius_m = initial.position_m.norm();
            assert!((orbit_radius_m - (earth.radius_equator_m() + ALTITUDE_M)).abs() <= 1e-6);
            let mean_motion = (earth.mu_m3ps2() / orbit_radius_m.powi(3)).sqrt();
            for (time, state) in data.hub.times_nanos.iter().zip(&data.hub.values) {
                let angle = mean_motion * *time as f64 * 1e-9;
                let expected = angle.cos() * initial.position_m
                    + angle.sin() * initial.velocity_mps / mean_motion;
                let error = (state.position_m - expected).abs().max();
                assert!(error <= 1e-6, "{time}: {error} m");
            }
            for platform in &data.platforms {
                // Both boresights move while the host attitude is fixed.
                let boresight =
                    |state: &SpacecraftStateMsg| mrp_to_dcm(state.sigma_bn).row(2).transpose();
                let start = boresight(&platform.values[0]);
                let end = boresight(platform.last());
                assert!((end - start).norm() > 0.1);
            }
        }
    }

    #[test]
    fn test_camera_visualization_parent_wiring() {
        for enable_viz in cases() {
            let data = run(enable_viz, false);
            if !enable_viz {
                assert!(data.camera_parents.is_empty());
                assert!(data.viz_parents.is_empty());
                continue;
            }
            let expected = ["cameraPlatform1", "cameraPlatform2"];
            assert_eq!(data.camera_parents, expected);
            assert_eq!(data.viz_camera_parents, expected);
            assert_eq!(data.viz_parents, ["", "cameraHost", "cameraHost"]);
            for camera in 0..2 {
                let error = (data.viz_platform_sigma[camera]
                    - data.platforms[camera].last().sigma_bn)
                    .abs()
                    .max();
                assert!(error <= 1e-14);
            }
        }
    }
}
