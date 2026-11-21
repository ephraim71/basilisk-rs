//! Inertial pointing set up for a Vizard instrument camera: port of upstream
//! `examples/scenarioVizPoint.py`.
//!
//! The spacecraft of `scenarioAttitudeFeedback` (750 kg, from
//! `sigma_BN = [0.1, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`)
//! is pointed by `inertial3D`, `attTrackingError` and `mrpFeedback`
//! (`K = 3.5`, `P = 30`) through an `extForceTorque` effector, on one 0.1 s
//! task, with a camera configuration message for Vizard (camera 1, at
//! `[5, 0, 0] m`, `sigma_CB = [-1/3, 1/3, -1/3]`):
//!
//! * `dscovr`: NASA's DSCOVR at the Sun-Earth L1 distance, 1 405 708 km from
//!   the Earth (central) towards the Sun, at rest, on 2018 October 23
//!   04:35:25 UTC, with the Sun's gravity. The reference attitude points
//!   `b1` from the Sun to the Earth, `b3` towards ecliptic north, using
//!   upstream's hard-coded Earth and Sun vectors. A 0.62° camera of
//!   2048 × 2048 pixels; 10 minutes.
//! * `marsOrbit`: an orbit about the Mars barycenter (`a = 16 000 km`,
//!   `e = 0.1`, `i = 10°`, `Ω = 25°`, `ω = 10°`, `f = 160°`), reference
//!   `sigma_R0N = [0, 0, 0.1]`, a 50° camera of 512 × 512 pixels; 6.25
//!   minutes.
//!
//! The Sun's ephemeris relative to the Earth (upstream's `spiceInterface`
//! with `zeroBase = "earth"`) comes from ANISE with DE440s.
//!
//! Upstream `run(show_plots, missionType, saveVizardFile)` maps to
//! `--mission dscovr|marsOrbit` and `--save-vizard-file`, which saves the
//! Vizard messages (the spacecraft, the bodies and the camera, with the
//! frustum HUD on) to `examples/output/vizard/scenarioVizPoint_<mission>.bin`
//! and needs the `vizard` feature. The script's own default is `marsOrbit`
//! with the Vizard file (here: when built with the feature); `--all` runs
//! both missions without the file, as upstream's `test_scenarioVizPoint.py`.
//! Writes `examples/output/scenarios/scenarioVizPoint_<mission>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
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

use std::path::PathBuf;
use std::str::FromStr;

use attitude::{
    DYNAMICS_PERIOD_NANOS, HubState, att_tracking_error, external_disturbance, hub_inertia_kg_m2,
    mrp_feedback, sampling_period_nanos, scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, CameraConfigMsg, NavigationTranslationMsg, Output,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::dcm_to_mrp;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::{elem2rv, vector_columns};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};

/// Upstream `missionType`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mission {
    Dscovr,
    MarsOrbit,
}

impl Mission {
    const ALL: [Self; 2] = [Self::Dscovr, Self::MarsOrbit];

    fn name(self) -> &'static str {
        match self {
            Self::Dscovr => "dscovr",
            Self::MarsOrbit => "marsOrbit",
        }
    }

    /// Upstream `simulationTime`.
    fn duration_nanos(self) -> u64 {
        match self {
            Self::Dscovr => 600_000_000_000,
            Self::MarsOrbit => 375_000_000_000,
        }
    }

    /// Upstream's `cameraConfig`.
    fn camera(self) -> CameraConfigMsg {
        let (field_of_view_deg, resolution) = match self {
            Self::Dscovr => (0.62, 2048),
            Self::MarsOrbit => (50.0, 512),
        };
        CameraConfigMsg {
            camera_id: 1,
            render_rate_nanos: 0,
            sigma_cb: Vector3::new(-0.333333, 0.333333, -0.333333),
            camera_pos_b_m: Vector3::new(5000.0 * 1e-3, 0.0, 0.0),
            field_of_view_rad: f64::to_radians(field_of_view_deg),
            resolution_px: [resolution, resolution],
            ..CameraConfigMsg::default()
        }
    }
}

impl FromStr for Mission {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|mission| mission.name() == text)
            .ok_or_else(|| format!("wrong mission type {text}; options are dscovr, marsOrbit"))
    }
}

/// Upstream's DSCOVR geometry: the initial position (1 405 708 km from the
/// Earth along the Earth-to-Sun line of the hard-coded vectors) and the
/// reference attitude `sigma_R0N` (`b1` from the Sun to the Earth, `b3`
/// towards the ecliptic north).
fn dscovr_geometry() -> (Vector3<f64>, Vector3<f64>) {
    let earth = Vector3::new(129559501208.24178, 68180766143.44236, 29544768114.76163);
    let normal: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);
    let sun = Vector3::new(-32509693.54023, 1002377617.77831, 423017670.86700);
    let dscovr_earth_distance_m = 1405708000.0;
    let r_sc = dscovr_earth_distance_m * (sun - earth).normalize();
    let b1 = -(sun - earth).normalize();
    let b3: Vector3<f64> = (normal - normal.dot(&b1) * b1).normalize();
    assert!(b1.dot(&b3).abs() < 1e-10, "Wrong dcm");
    let b2 = b3.cross(&b1).normalize();
    let nb = Matrix3::from_columns(&[b1, b2, b3]);
    (r_sc, dcm_to_mrp(nb.transpose()))
}

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    mission: Mission,
    save_vizard_file: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    guidance: MessageLog<AttitudeGuidanceMsg>,
    torque: MessageLog<BodyTorqueCommandMsg>,
    translation: MessageLog<NavigationTranslationMsg>,
    csv_path: Option<PathBuf>,
    vizard_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mission = config.mission;
    let duration_nanos = mission.duration_nanos();
    let (central, start_epoch) = match mission {
        Mission::Dscovr => (
            Body::Earth,
            Epoch::from_gregorian_utc(2018, 10, 23, 4, 35, 25, 0),
        ),
        Mission::MarsOrbit => (
            Body::MarsBarycenter,
            Epoch::from_gregorian_utc_at_midnight(2019, 1, 1),
        ),
    };
    let ((position_m, velocity_mps), sigma_r0n) = match mission {
        Mission::Dscovr => {
            let (position, sigma) = dscovr_geometry();
            ((position, Vector3::zeros()), sigma)
        }
        Mission::MarsOrbit => (
            elem2rv(
                central.mu_m3ps2(),
                16_000_000.0,
                0.1,
                10.0_f64.to_radians(),
                25.0_f64.to_radians(),
                10.0_f64.to_radians(),
                160.0_f64.to_radians(),
            ),
            Vector3::new(0.0, 0.0, 0.1),
        ),
    };
    let inertia = hub_inertia_kg_m2();
    let initial = HubState {
        position_m,
        velocity_mps,
        ..HubState::in_deep_space()
    };
    let mut spacecraft = spacecraft(inertia, &initial);
    spacecraft
        .add_grav_body(central.point_mass(true))
        .expect("one central body");
    // DSCOVR: the Sun's gravity, on its ephemeris relative to the Earth.
    let mut sun_ephemeris = (mission == Mission::Dscovr).then(|| {
        spacecraft
            .add_grav_body(Body::Sun.point_mass(false))
            .expect("unique gravity bodies");
        Body::Sun.ephemeris(Body::Earth)
    });
    let camera_msg = Output::new(mission.camera());

    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n,
    });
    let mut tracking_error = att_tracking_error(Vector3::zeros());
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let sample_period = sampling_period_nanos(duration_nanos, DYNAMICS_PERIOD_NANOS, 100);
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut torque_log = MessageLog::<BodyTorqueCommandMsg>::default();
    let mut translation_log = MessageLog::<NavigationTranslationMsg>::default();
    let stem = format!("scenarioVizPoint_{}", mission.name());
    let (mut recorder, csv_path) = scenario_recorder(&stem, config.write_csv);

    let mut simulation = Simulation::new(start_epoch, false);
    connect!(&simulation, &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg);
    spacecraft.add_dynamic_effector(external_torque);
    if let Some(ephemeris) = &sun_ephemeris {
        simulation.connect(
            &ephemeris.output_planet_msg,
            spacecraft.grav_body_input_mut("sun").expect("Sun gravity"),
        );
    }
    connect!(&simulation,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &controller.cmd_torque_out_msg => &mut torque_log.input,
        &navigation.translation_out_msg => &mut translation_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
        &controller.cmd_torque_out_msg => recorder.add_source::<BodyTorqueCommandMsg>(
            CsvSourceConfig::columns(vector_columns("torque_request_body_nm", "torqueRequestBody_Nm")),
        ),
        &navigation.translation_out_msg => recorder.add_source::<NavigationTranslationMsg>(
            CsvSourceConfig::columns(vector_columns("position_bn_n_m", "r_BN_N_m")),
        ),
    );

    #[cfg(feature = "vizard")]
    let vizard_path = config.save_vizard_file.then(|| viz::vizard_file(&stem));
    #[cfg(not(feature = "vizard"))]
    let vizard_path: Option<PathBuf> = None;
    #[cfg(not(feature = "vizard"))]
    assert!(
        !config.save_vizard_file,
        "--save-vizard-file needs the vizard feature (--features vizard)"
    );
    #[cfg(feature = "vizard")]
    let mut vizard = vizard_path.clone().map(|path| {
        use basilisk_rs::vizard::viz_structures::VizSetting;
        let mut vizard = viz::saving_interface(path);
        vizard.settings.view_camera_frustum_hud = VizSetting::Enabled;
        let spacecraft_data = vizard.add_spacecraft("spacecraftBody");
        simulation.connect(&spacecraft.state_out, &mut spacecraft_data.sc_state_in_msg);
        let camera = vizard.add_camera();
        simulation.connect(&camera_msg, &mut camera.camera_config_in_msg);
        // The central body sits at the origin.
        vizard.add_celestial_body(viz::grav_body_info(central));
        if let Some(ephemeris) = &sun_ephemeris {
            let sun_input = vizard.add_celestial_body(viz::grav_body_info(Body::Sun));
            simulation.connect(&ephemeris.output_planet_msg, sun_input);
        }
        vizard
    });
    #[cfg(not(feature = "vizard"))]
    let _ = &camera_msg;

    if let Some(ephemeris) = sun_ephemeris.as_mut() {
        simulation.add_module("spiceObject", ephemeris, DYNAMICS_PERIOD_NANOS, 110);
    }
    // Upstream's task order.
    schedule! { simulation,
        "spacecraftBody" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "SimpleNavigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "inertial3D" => &mut inertial_reference, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "attErrorInertial3D" => &mut tracking_error, period=DYNAMICS_PERIOD_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=DYNAMICS_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=sample_period, priority=0;
        "torque_log" => &mut torque_log, period=sample_period, priority=0;
        "translation_log" => &mut translation_log, period=sample_period, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    #[cfg(feature = "vizard")]
    if let Some(vizard) = vizard.as_mut() {
        simulation.add_module("vizInterface", vizard, DYNAMICS_PERIOD_NANOS, -10);
    }
    simulation.run_for(duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        torque: torque_log,
        translation: translation_log,
        csv_path,
        vizard_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_viz_point [--mission dscovr|marsOrbit] [--[no-]save-vizard-file] [--all]",
    );
    let configs = if cli.switch("all") {
        Mission::ALL
            .into_iter()
            .map(|mission| Config {
                mission,
                save_vizard_file: false,
                write_csv: true,
            })
            .collect()
    } else {
        // Upstream's `__main__` runs `run(True, 'marsOrbit', True)`.
        vec![Config {
            mission: cli.value("mission", Mission::MarsOrbit),
            save_vizard_file: cli.flag("save-vizard-file", cfg!(feature = "vizard")),
            write_csv: true,
        }]
    };
    cli.finish();
    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e})",
            results.csv_path.expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
        );
        if let Some(path) = results.vizard_path {
            println!("wrote {}", path.display());
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioVizPoint.py` only checks that both missions
    //! run (without the Vizard file). These tests check the pointing the
    //! scenario sets up.

    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    fn quiet(mission: Mission) -> Results {
        run(Config {
            mission,
            save_vizard_file: false,
            write_csv: false,
        })
    }

    /// DSCOVR's reference frame has `b1` along the Sun-to-Earth direction of
    /// upstream's vectors and `b3` in the plane of that line and `n3`.
    #[test]
    fn the_dscovr_reference_points_b1_from_the_sun_to_the_earth() {
        let (position, sigma_r0n) = dscovr_geometry();
        assert!((position.norm() - 1405708000.0).abs() < 1e-3);
        let dcm = mrp_to_dcm(sigma_r0n);
        let b1 = dcm.row(0).transpose();
        let b3 = dcm.row(2).transpose();
        assert!((b1 + position.normalize()).norm() < 1e-12);
        assert!(b3.z > 0.0 && b3.dot(&Vector3::z().cross(&b1)).abs() < 1e-12);
    }

    /// The attitude error decays from its initial value towards the
    /// reference, while the orbit is unaffected by the attitude.
    #[test]
    fn both_missions_point_at_their_reference() {
        for mission in Mission::ALL {
            let results = quiet(mission);
            // `samplingTime`: 100 samples, 102 for the 6.25 minutes.
            assert!((101..=102).contains(&results.guidance.values.len()));
            assert!(results.torque.values[0].torque_request_body_nm.norm() > 0.5);
            let initial = results.guidance.values[0].sigma_br.norm();
            let last = results.guidance.last().sigma_br.norm();
            assert!(initial > 0.1, "{mission:?}: {initial}");
            assert!(last < 5e-3, "{mission:?}: {last}");
            assert!(results.guidance.last().omega_br_b_radps.norm() < 1e-3);
            if mission == Mission::Dscovr {
                // At rest 1.4 million km out, the spacecraft starts to fall
                // towards the Sun and the Earth: 10 minutes move it by
                // under a kilometre.
                let moved = (results.translation.last().position_bn_n_m
                    - results.translation.values[0].position_bn_n_m)
                    .norm();
                assert!(moved > 0.0 && moved < 1e3, "{moved}");
            }
        }
    }
}
