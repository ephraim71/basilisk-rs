//! Vizard quad maps on the Earth and on a spacecraft: port of upstream
//! `examples/scenarioQuadMaps.py`. Needs the `vizard` feature:
//!
//! ```text
//! cargo run --release --features vizard --example scenario_quad_maps
//! ```
//!
//! A spacecraft at upstream's hub defaults (1 kg, identity inertia) flies an
//! eccentric MEO orbit (`a = 11 260 km`, `e = 0.25`, `i = 28 deg`) about the
//! point-mass Earth from `2000 November 26, 09:30:00.0 TDB`, for 6 hours on
//! a 10 s task. [`VizInterface`] draws quad maps (Vizard regions) over it:
//!
//! * an "Arctic Region" (66.5 to 90 deg N, 8 deg cells) and "Colorado"
//!   (37 to 41 deg N, 102.05 to 109.05 deg W, 5 deg cells) meshed on the
//!   Earth's reference ellipsoid (`radiusRatio = 0.9966`), and a "Solar
//!   Cell" rectangle on the spacecraft;
//! * every 20 minutes, the footprint on the ellipsoid of a 10 deg standard
//!   camera looking from the spacecraft at the Earth's center, meshed 3 x 3,
//!   when all four corners hit the Earth;
//! * halfway through, the Arctic region is hidden, Colorado recoloured and
//!   relabelled, and the solar cell's label removed.
//!
//! Upstream `run(show_plots)` has no parameters and no figures; `--all` is
//! accepted and runs the same single case. Upstream enables Vizard without a
//! save file; `--save <file.bin>` saves the Vizard messages (upstream
//! `saveFile`) to open in Vizard.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `spiceInterface` with `zeroBase = 'Earth'` is an ANISE
//!   ephemeris of the Earth relative to itself with its `IAU_EARTH`
//!   orientation, run first and at initialization ([`UpdatedAtInit`]); the
//!   footprints are planet-fixed;
//! * upstream's `vizSupport.qms` geometry (`computeRectMesh`,
//!   `computeCamFOVBox`, `subdivideFOVBox`) is [`quad_map_support`];
//! * upstream adds quad maps to the Vizard interface between
//!   `ExecuteSimulation` calls; here the interface is a [`Shared`] module,
//!   changed between `run_for` calls. Each batch goes out with the next
//!   frame, as upstream's.
//!
//! Writes `examples/output/scenarios/scenarioQuadMaps.csv` with upstream's
//! `dataRec` spacecraft state `r_BN_N_m_<i>`, `v_BN_N_m_per_s_<i>` (100
//! samples).

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/quad_map_support.rs"]
mod quad_map_support;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/shared.rs"]
mod shared;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use basilisk_rs::messages::{Output, PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::astro_constants::D2R;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use basilisk_rs::vizard::viz_interface::{VizInterface, VizInterfaceConfig};
use basilisk_rs::vizard::viz_structures::{
    GravBodyInfo, QuadMap, Rgba, StdCameraMode, StdCameraSettings,
};
use basilisk_rs::{Module, schedule};
use cli::Cli;
use grav_factory::Body;
use hifitime::{Epoch, TimeScale};
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use quad_map_support::{
    Ellipsoid, StdCamera, compute_cam_fov_box, compute_rect_mesh, subdivide_fov_box,
};
use recording::{sampling_time, scenario_csv, state_columns};
use shared::Shared;
use staged::UpdatedAtInit;

/// Upstream `simulationTimeStep = macros.sec2nano(10.)`.
const TASK_PERIOD_NANOS: u64 = 10_000_000_000;
/// Upstream `simulationTime = macros.hour2nano(6)`.
const DURATION_NANOS: u64 = 6 * 3600 * 1_000_000_000;
/// Upstream `imgTimeStep = macros.min2nano(20)`: between camera footprints.
const IMAGE_PERIOD_NANOS: u64 = 20 * 60 * 1_000_000_000;
/// Upstream `numDataPoints`.
const DATA_POINTS: u64 = 100;
/// Upstream `planet.radiusRatio`.
const EARTH_RADIUS_RATIO: f64 = 0.9966;
/// Upstream `fieldOfViewSubdivs`.
const FIELD_OF_VIEW_SUBDIVISIONS: usize = 3;
/// Upstream `camQM_ID`: the first footprint's quad-map ID.
const FIRST_FOOTPRINT_ID: i32 = 10;
/// Upstream's spacecraft `ModelTag`.
const SPACECRAFT_NAME: &str = "bsk-Sat";
/// Upstream's "Solar Cell" corners, spacecraft body frame \[m\].
const SOLAR_CELL_M: [[f64; 3]; 4] = [
    [-2.551, 0.341, 1.01],
    [-2.804, 0.341, 1.01],
    [-2.804, 0.111, 1.01],
    [-2.551, 0.111, 1.01],
];

/// Upstream `run(show_plots)`, with where to save the Vizard file.
#[derive(Clone, Debug)]
struct Config {
    /// Upstream `saveFile`: none, as upstream.
    save_file: Option<PathBuf>,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded history and the footprints drawn.
// Some fields are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `dataRec`.
    state: MessageLog<SpacecraftStateMsg>,
    /// Each camera footprint: its quad-map ID, the spacecraft position
    /// relative to the Earth and the four corners, planet-fixed \[m\].
    footprints: Vec<(i32, Vector3<f64>, Vec<Vector3<f64>>)>,
    csv_path: Option<PathBuf>,
}

/// Upstream `timeInitString = "2000 November 26, 09:30:00.0 TDB"`.
fn epoch() -> Epoch {
    Epoch::from_gregorian(2000, 11, 26, 9, 30, 0, 0, TimeScale::TDB)
}

/// The Earth's reference ellipsoid (upstream `planet.radEquator`,
/// `planet.radiusRatio`).
fn earth_ellipsoid() -> Ellipsoid {
    Ellipsoid {
        radius_equator_m: Body::Earth.radius_equator_m(),
        radius_ratio: EARTH_RADIUS_RATIO,
    }
}

/// Upstream `vizSupport.addQuadMap(viz, ID, parentBodyName, vertices, color,
/// isHidden, label)`.
fn quad_map(
    id: i32,
    parent_body_name: &str,
    vertices_m: Vec<Vector3<f64>>,
    color: Rgba,
    is_hidden: bool,
    label: &str,
) -> QuadMap {
    QuadMap {
        id,
        parent_body_name: parent_body_name.to_string(),
        vertices_m,
        color: Some(color),
        is_hidden,
        label: label.to_string(),
    }
}

/// Upstream's three regions; `changed` gives the versions it sends halfway.
fn regions(changed: bool) -> Vec<QuadMap> {
    let earth = earth_ellipsoid();
    let arctic = compute_rect_mesh(&earth, [66.5, 90.0], [-180.0, 180.0], 8.0);
    let colorado = compute_rect_mesh(&earth, [37.0, 41.0], [-102.0467, -109.0467], 5.0);
    let solar_cell = SOLAR_CELL_M.iter().map(|v| Vector3::from(*v)).collect();
    if changed {
        vec![
            quad_map(1, "earth", arctic, [0, 0, 255, 100], true, ""),
            quad_map(
                2,
                "earth",
                colorado,
                [255, 255, 0, 100],
                false,
                "CO (new color!)",
            ),
            quad_map(
                3,
                SPACECRAFT_NAME,
                solar_cell,
                [255, 128, 0, 100],
                false,
                "NOLABEL",
            ),
        ]
    } else {
        vec![
            quad_map(1, "earth", arctic, [0, 0, 255, 100], false, "Arctic Region"),
            quad_map(2, "earth", colorado, [0, 255, 0, 100], false, "Colorado"),
            quad_map(
                3,
                SPACECRAFT_NAME,
                solar_cell,
                [255, 128, 0, 100],
                false,
                "Solar Cell",
            ),
        ]
    }
}

/// Upstream `vizSupport.createStandardCamera(viz, setMode=0,
/// bodyTarget='earth', fieldOfView=np.deg2rad(10))`.
fn standard_camera() -> StdCameraSettings {
    StdCameraSettings {
        spacecraft_name: SPACECRAFT_NAME.to_string(),
        set_mode: StdCameraMode::BodyTarget,
        body_target: "earth".to_string(),
        field_of_view_rad: Some(10.0 * D2R),
        ..StdCameraSettings::default()
    }
}

/// Upstream `qms.computeCamFOVBox(planet, spiceObject, scObject, cam)` from
/// the current spacecraft state and Earth orientation, with the spacecraft
/// position relative to the Earth, planet-fixed.
fn camera_footprint(
    state: &SpacecraftStateMsg,
    planet: &PlanetStateMsg,
) -> (Vector3<f64>, Vec<Vector3<f64>>) {
    let camera = standard_camera();
    let dcm_pn = planet
        .orientation
        .as_ref()
        .map_or_else(Matrix3::identity, |orientation| {
            orientation.inertial_to_fixed
        });
    let r_bp_n_m = state.position_m - planet.position_inertial_m;
    let corners = compute_cam_fov_box(
        &earth_ellipsoid(),
        &dcm_pn,
        &r_bp_n_m,
        &mrp_to_dcm(state.sigma_bn),
        &StdCamera {
            position_b_m: camera.position_b_m,
            field_of_view_rad: camera.field_of_view_rad.expect("set above"),
        },
    );
    (dcm_pn * r_bp_n_m, corners)
}

fn run(config: &Config) -> Results {
    let earth = Body::Earth;
    let mu = earth.mu_m3ps2();
    let elements = ClassicElements {
        semi_major_axis: 11260.0 * 1000.0,
        eccentricity: 0.25,
        inclination_rad: 28.0 * D2R,
        raan_rad: 10.5 * D2R,
        arg_periapsis_rad: 20.5 * D2R,
        true_anomaly_rad: 10.0 * D2R,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("one central gravity body");
    let mut ephemeris = UpdatedAtInit::new(earth.ephemeris(Body::Earth), epoch());
    let planet: Output<PlanetStateMsg> = ephemeris.module.output_planet_msg.clone();
    let state = spacecraft.state_out.clone();

    let viz = Shared::new(VizInterface::new(VizInterfaceConfig {
        name: "vizInterface".to_string(),
        save_file: config.save_file.clone(),
        ..VizInterfaceConfig::default()
    }));
    {
        let mut viz = viz.lock();
        let spacecraft_data = viz.add_spacecraft(SPACECRAFT_NAME);
        state.connect_to(&mut spacecraft_data.sc_state_in_msg);
        planet.connect_to(viz.add_celestial_body(GravBodyInfo {
            body_name: earth.name().to_string(),
            mu_m3ps2: mu,
            rad_equator_m: earth.radius_equator_m(),
            radius_ratio: EARTH_RADIUS_RATIO,
            ..GravBodyInfo::default()
        }));
        viz.settings.std_camera_list.push(standard_camera());
        viz.settings.main_camera_target = "earth".to_string();
        viz.quad_maps.clear();
        viz.quad_maps.extend(regions(false));
    }

    let sample_nanos = sampling_time(DURATION_NANOS, TASK_PERIOD_NANOS, DATA_POINTS);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    state.connect_to(&mut state_log.input);
    let mut csv = config.write_csv.then(|| {
        let mut csv = scenario_csv("scenarioQuadMaps");
        state.connect_to(csv.add_source::<SpacecraftStateMsg>(state_columns()));
        csv
    });
    let csv_path = csv.as_ref().map(|csv| csv.config.output_path.clone());

    let mut footprints = Vec::new();
    let mut viz_module = viz.handle();
    {
        let mut simulation = Simulation::new(epoch(), false);
        // Upstream's task order: the spacecraft, the ephemeris (run first
        // here, see the module docs), the recorder and Vizard.
        schedule! { simulation,
            "spice" => &mut ephemeris, period=TASK_PERIOD_NANOS, priority=100;
            "bsk-Sat" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=90;
            "dataRec" => &mut state_log, period=sample_nanos, priority=0;
            "vizInterface" => &mut viz_module, period=TASK_PERIOD_NANOS, priority=-10;
        }
        if let Some(csv) = csv.as_mut() {
            simulation.add_module("csv", csv as &mut dyn Module, sample_nanos, 0);
        }
        simulation.initialize();

        let mut stop_nanos = 0;
        let mut footprint_id = FIRST_FOOTPRINT_ID;
        while stop_nanos < DURATION_NANOS {
            let (r_bp_p_m, corners) = camera_footprint(&state.read(), &planet.read());
            let mut viz = viz.lock();
            if corners.len() == 4 {
                let mesh =
                    subdivide_fov_box(&earth_ellipsoid(), &corners, FIELD_OF_VIEW_SUBDIVISIONS);
                viz.quad_maps.push(quad_map(
                    footprint_id,
                    "earth",
                    mesh,
                    [255, 0, 0, 60],
                    false,
                    "",
                ));
                footprints.push((footprint_id, r_bp_p_m, corners));
                footprint_id += 1;
            }
            if stop_nanos == DURATION_NANOS / 2 {
                viz.quad_maps.extend(regions(true));
            }
            drop(viz);
            stop_nanos += IMAGE_PERIOD_NANOS;
            orbit_tools::run_until(&mut simulation, stop_nanos);
        }
    }
    drop(csv);
    drop(viz_module);
    // Close the Vizard file.
    drop(viz);

    Results {
        state: state_log,
        footprints,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_quad_maps [--save <file.bin>] [--all]");
    cli.switch("all");
    let save: String = cli.value("save", String::new());
    let config = Config {
        save_file: (!save.is_empty()).then(|| PathBuf::from(save)),
        write_csv: true,
    };
    cli.finish();

    let results = run(&config);
    println!(
        "wrote {} ({} camera footprints)",
        results.csv_path.as_ref().expect("CSV written").display(),
        results.footprints.len(),
    );
    if let Some(path) = &config.save_file {
        println!("wrote {}", path.display());
    }
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it runs. These tests check the quad-map geometry and that
    //! the Vizard file carries the quad maps upstream sends, when it sends
    //! them.

    use basilisk_rs::vizard::viz_interface::read_viz_file;

    use super::*;

    fn quiet(save_file: Option<PathBuf>) -> Results {
        run(&Config {
            save_file,
            write_csv: false,
        })
    }

    #[test]
    fn the_region_meshes_lie_on_the_ellipsoid() {
        let earth = earth_ellipsoid();
        let maps = regions(false);
        // Arctic: 3 latitude bands of 45 longitude cells; Colorado: 1 x 2.
        assert_eq!(maps[0].vertices_m.len(), 4 * 3 * 45);
        assert_eq!(maps[1].vertices_m.len(), 4 * 2);
        for map in &maps[..2] {
            assert!(map.vertices_m.iter().all(|v| earth.contains(v, 1e-12)));
        }
        // Round trip of the two conversions.
        let point = earth.lla_to_fixed(0.7, -1.9, 1234.0);
        let (lat, lon, alt) = earth.fixed_to_lla(&point);
        assert!((lat - 0.7).abs() < 1e-12 && (lon + 1.9).abs() < 1e-12);
        assert!((alt - 1234.0).abs() < 1e-6);
    }

    #[test]
    fn the_footprints_are_the_camera_view_of_the_earth() {
        let results = quiet(None);
        let earth = earth_ellipsoid();
        // One footprint every 20 minutes: from this orbit (perigee about
        // 2100 km up) the Earth always fills the 10 deg view.
        assert_eq!(results.footprints.len(), 18);
        let ids: Vec<i32> = results.footprints.iter().map(|(id, _, _)| *id).collect();
        assert_eq!(
            ids,
            (FIRST_FOOTPRINT_ID..FIRST_FOOTPRINT_ID + 18).collect::<Vec<_>>()
        );
        // Seen from the spacecraft, each corner is half the field of view's
        // diagonal from the Earth's center, and it is the near side of the
        // ellipsoid.
        let half_diagonal = ((10.0 * D2R / 2.0).tan() * 2.0_f64.sqrt()).atan();
        // The spacecraft flies its two-body orbit.
        let mu = Body::Earth.mu_m3ps2();
        let elements = ClassicElements {
            semi_major_axis: 11260.0 * 1000.0,
            eccentricity: 0.25,
            inclination_rad: 28.0 * D2R,
            raan_rad: 10.5 * D2R,
            arg_periapsis_rad: 20.5 * D2R,
            true_anomaly_rad: 10.0 * D2R,
            ..Default::default()
        };
        for (time, state) in results.state.times_nanos.iter().zip(&results.state.values) {
            let (expected, _) = orbit_tools::kepler_state(mu, &elements, *time as f64 * 1e-9);
            assert!((state.position_m - expected).norm() < 1.0, "at {time} ns");
        }
        for (_, r_bp_p_m, corners) in &results.footprints {
            assert_eq!(corners.len(), 4);
            for corner in corners {
                assert!(earth.contains(corner, 1e-12));
                let ray = corner - r_bp_p_m;
                assert!((ray.angle(&-r_bp_p_m) - half_diagonal).abs() < 1e-9);
                assert!(ray.norm() < r_bp_p_m.norm());
            }
        }
    }

    #[test]
    fn the_vizard_file_carries_the_quad_maps() {
        let path =
            std::env::temp_dir().join(format!("basilisk-rs-quad-maps-{}.bin", std::process::id()));
        quiet(Some(path.clone()));
        let frames = read_viz_file(&path).expect("readable Vizard file");
        let _ = std::fs::remove_file(&path);
        // Like upstream's, the interface sends no frame at t = 0: frame `k`
        // is at `(k + 1) 10 s`.
        assert_eq!(frames.len() as u64, DURATION_NANOS / TASK_PERIOD_NANOS);
        let ids = |frame: &basilisk_rs::vizard::proto::VizMessage| -> Vec<i32> {
            frame.quad_maps.iter().map(|map| map.id).collect()
        };
        // The regions and the first footprint go out with the first frame,
        // each later footprint with the frame after its 20 minute stop, and
        // the changed regions with the one after the 3 h stop.
        assert_eq!(ids(&frames[0]), [1, 2, 3, FIRST_FOOTPRINT_ID]);
        let per_image = (IMAGE_PERIOD_NANOS / TASK_PERIOD_NANOS) as usize;
        for (k, frame) in frames.iter().enumerate().skip(1) {
            let image = k / per_image;
            let expected: Vec<i32> = if k % per_image == 0 {
                let id = FIRST_FOOTPRINT_ID + image as i32;
                if image == 9 {
                    vec![id, 1, 2, 3]
                } else {
                    vec![id]
                }
            } else {
                Vec::new()
            };
            assert_eq!(ids(frame), expected, "frame {k}");
        }
        let changed = &frames[9 * per_image].quad_maps;
        assert!(changed[1].is_hidden);
        assert_eq!(changed[2].label, "CO (new color!)");
        assert_eq!(changed[3].label, "NOLABEL");
        assert_eq!(frames[0].quad_maps[3].vertices.len(), 3 * 4 * 9);
    }
}
