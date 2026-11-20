//! Landmark pixels of asteroid 433 Eros from a pinhole camera: port of
//! upstream `examples/scenarioSmallBodyLandmarks.py`.
//!
//! A 750 kg rigid spacecraft (`I = diag(900, 800, 600) kg m^2`) orbits Eros
//! on a 34 km, `e = 0.001`, 45 deg orbit given in Eros' rotating frame, from
//! `sigma_BN = [0.7, 0.2, -0.3]`, `omega_BN = [0.001, -0.01, 0.03] rad/s`.
//! Eros (a `mu = 4.4631e5 m^3/s^2` point mass, the central body) follows a
//! two-body heliocentric `planetEphemeris` orbit and spins about its pole
//! (RA 11.369 deg, dec 17.227 deg, a 5.27 h period). `hillPoint`, with Eros
//! from an `ephemerisConverter`, and `attTrackingError` with
//! `sigma_R0R = [0, 1, 0]` (`b1` to the negative radial direction) feed
//! `mrpFeedback` (`K = 3.5`, `P = 30`, no integral feedback), whose torque an
//! `extForceTorque` effector applies. A `pinholeCamera` (25 mm focal length,
//! 2048 x 1536 pixels 17.3/2048 mm wide, `c3 = b1`) watches 100 landmarks:
//! the centers and normals of 100 faces of upstream's Eros polyhedron
//! (`eros007790.tab`, `just fetch-small-body-data`) drawn after
//! `np.random.seed(0)`, exactly as upstream draws them (see
//! [`landmarks`](crate::landmarks)). One 0.1 s task, 100 minutes, the
//! histories recorded every 60 s. Gravity stays Keplerian, as upstream's: the
//! polyhedron only places the landmarks.
//!
//! Upstream `run(show_plots, useBatch)` maps to `--batch`: also run the
//! camera model detached from the simulation, upstream
//! `camera.processBatch(r_BP_P, mrp_BP, e_SP_P)`, here
//! [`PinholeCamera::process_batch`], on the recorded states expressed in the
//! Eros frame, and compare it with what the camera published in the
//! simulation (upstream's `batch_diff`). The upstream script's own default
//! is off; `--all` runs the parametrization of upstream's
//! `test_scenarioSmallBodyLandmarks.py` (batch on).
//!
//! Upstream adds `extForceTorque` to the task, where it reads the torque
//! command once per step before `mrpFeedback` writes the next one; here the
//! effector reads the command when the spacecraft integrates, as in the
//! other attitude scenario ports.
//!
//! Writes `examples/output/scenarios/scenarioSmallBodyLandmarks.csv`: the
//! spacecraft state (`r_BN_N_m_*`, `v_BN_N_m_per_s_*`, `sigma_BN_*`), Eros'
//! ephemeris (`r_PN_N_m_*`, `v_PN_N_m_per_s_*`, `sigma_PN_*`) and, for each
//! landmark `j`, `isVisible_j` and its pixel `pL_j_0`, `pL_j_1`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/landmarks.rs"]
mod landmarks;
#[path = "support/message_log.rs"]
mod message_log;

use std::f64::consts::{FRAC_PI_2, PI};
use std::path::PathBuf;

use attitude::{
    HubState, att_tracking_error, external_disturbance, hill_point, hub_inertia_kg_m2,
    mrp_feedback, scenario_recorder, spacecraft, vehicle_config,
};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::environment::planet_ephemeris::{
    PlanetEphemeris, PlanetEphemerisBody, PlanetEphemerisConfig, PlanetRotation,
};
use basilisk_rs::messages::{EphemerisMsg, LandmarkMsg, Output, SpacecraftStateMsg};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::sensors::pinhole_camera::{PinholeCamera, PinholeCameraConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::{dcm_to_mrp, euler323_to_dcm, mrp_to_dcm};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::custom_point_mass;
use hifitime::Epoch;
use landmarks::{EROS_SHAPE, PolyhedronShape, landmark_distribution};
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};

/// Upstream `simulationTimeStep = macros.sec2nano(0.1)`.
const TASK_PERIOD_NANOS: u64 = 100_000_000;
/// Upstream `simulationTime = macros.min2nano(100.0)`.
const DURATION_NANOS: u64 = 6_000_000_000_000;
/// Upstream `samplingTime = macros.sec2nano(60)`.
const SAMPLE_PERIOD_NANOS: u64 = 60_000_000_000;
/// Upstream `n_lmk`.
const LANDMARK_COUNT: usize = 100;
/// Eros' gravitational parameter \[m^3/s^2\].
const EROS_MU_M3PS2: f64 = 4.4631e5;
/// Upstream `test_scenarioSmallBodyLandmarks.py` `accuracy`.
const BATCH_ACCURACY: f64 = 1e-6;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `useBatch`.
    use_batch: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_batch: bool) -> Self {
        Self {
            use_batch,
            write_csv: true,
        }
    }
}

/// The parametrization of upstream `test_scenarioSmallBodyLandmarks.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config::new(true)];

/// The recorded histories, every 60 s.
struct Results {
    /// Upstream `spacecraftLog`.
    spacecraft: MessageLog<SpacecraftStateMsg>,
    /// Upstream `asteroidLog`.
    asteroid: MessageLog<EphemerisMsg>,
    /// Upstream `landmarkLog`, one per landmark.
    landmarks: Vec<MessageLog<LandmarkMsg>>,
    /// Upstream `batch_diff`, with `--batch`.
    batch_diff: Option<[f64; 3]>,
    csv_path: Option<PathBuf>,
}

/// One sample of upstream's post-processing, in the Eros frame `P`.
struct PlanetFrameSample {
    /// Upstream `r_BP_P` \[m\].
    r_bp_p: Vector3<f64>,
    /// Upstream `v_BP_P` \[m/s\]: the inertial relative velocity, rotated.
    v_bp_p: Vector3<f64>,
    /// Upstream `mrp_BP`.
    sigma_bp: Vector3<f64>,
    /// Upstream `-r_PN_P / |r_PN_P|`: the Eros-to-Sun direction.
    e_sp_p: Vector3<f64>,
    /// Upstream `dcm_PC`: the camera axes (columns) in `P`.
    dcm_pc: Matrix3<f64>,
}

impl PlanetFrameSample {
    /// Upstream `plot_orientation`'s data: `i_r · c3`, `i_h · c1` and
    /// `i_theta · c2` (its `dcm_PH` columns are `i_r`, `i_h`, `i_theta`).
    fn orientation(&self) -> Vector3<f64> {
        let i_r = self.r_bp_p.normalize();
        let i_h = self.r_bp_p.cross(&self.v_bp_p).normalize();
        let i_theta = i_h.cross(&i_r);
        Vector3::new(
            i_r.dot(&self.dcm_pc.column(2)),
            i_h.dot(&self.dcm_pc.column(0)),
            i_theta.dot(&self.dcm_pc.column(1)),
        )
    }
}

/// Upstream `camera.dcm_CB`: `c1 = -b3`, `c2 = b2`, `c3 = b1`.
fn dcm_cb() -> Matrix3<f64> {
    Matrix3::new(0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0)
}

impl Results {
    /// Upstream's `r_BP_P`, `v_BP_P`, `mrp_BP`, Sun direction and `dcm_PC`.
    fn planet_frame(&self) -> Vec<PlanetFrameSample> {
        self.spacecraft
            .values
            .iter()
            .zip(&self.asteroid.values)
            .map(|(spacecraft, asteroid)| {
                let dcm_pn = mrp_to_dcm(asteroid.sigma_bn);
                let dcm_bp = mrp_to_dcm(spacecraft.sigma_bn) * dcm_pn.transpose();
                let r_pn_p = dcm_pn * asteroid.position_bdy_zero_n_m;
                PlanetFrameSample {
                    r_bp_p: dcm_pn * (spacecraft.position_m - asteroid.position_bdy_zero_n_m),
                    v_bp_p: dcm_pn * (spacecraft.velocity_mps - asteroid.velocity_bdy_zero_n_mps),
                    sigma_bp: dcm_to_mrp(dcm_bp),
                    e_sp_p: -r_pn_p / r_pn_p.norm(),
                    dcm_pc: (dcm_cb() * dcm_bp).transpose(),
                }
            })
            .collect()
    }

    /// Upstream `isvisibleLmk`, sample by sample.
    fn visibility(&self) -> Vec<Vec<f64>> {
        (0..self.spacecraft.values.len())
            .map(|sample| {
                self.landmarks
                    .iter()
                    .map(|log| f64::from(u8::from(log.values[sample].is_visible)))
                    .collect()
            })
            .collect()
    }

    /// Upstream `nvisibleLmk`.
    fn visible_counts(&self) -> Vec<usize> {
        (0..self.spacecraft.values.len())
            .map(|sample| {
                self.landmarks
                    .iter()
                    .filter(|log| log.values[sample].is_visible)
                    .count()
            })
            .collect()
    }
}

/// `‖a - b‖ / ‖b‖` over every element, upstream's `np.linalg.norm` ratios.
fn relative_difference(batch: &[f64], simulation: &[f64]) -> f64 {
    let difference: f64 = batch
        .iter()
        .zip(simulation)
        .map(|(a, b)| (a - b).powi(2))
        .sum();
    let reference: f64 = simulation.iter().map(|b| b * b).sum();
    (difference / reference).sqrt()
}

/// Upstream's `useBatch` branch: `processBatch` on the recorded states, and
/// its `batch_diff` against the simulation's landmark messages (visible
/// counts, visibility flags, pixels).
fn batch_difference(camera: &PinholeCamera, results: &Results) -> [f64; 3] {
    let samples = results.planet_frame();
    let r_bp_p: Vec<_> = samples.iter().map(|sample| sample.r_bp_p).collect();
    let sigma_bp: Vec<_> = samples.iter().map(|sample| sample.sigma_bp).collect();
    let e_sp_p: Vec<_> = samples.iter().map(|sample| sample.e_sp_p).collect();
    let batch = camera.process_batch(&r_bp_p, &sigma_bp, &e_sp_p);

    let flags = |row: &[LandmarkMsg]| -> Vec<f64> {
        row.iter()
            .map(|landmark| f64::from(u8::from(landmark.is_visible)))
            .collect()
    };
    let batch_visibility: Vec<f64> = batch.iter().flat_map(|row| flags(row)).collect();
    let visibility: Vec<f64> = results.visibility().into_iter().flatten().collect();
    let batch_counts: Vec<f64> = batch
        .iter()
        .map(|row| flags(row).iter().sum::<f64>())
        .collect();
    let counts: Vec<f64> = results
        .visible_counts()
        .into_iter()
        .map(|count| count as f64)
        .collect();
    // Upstream's `pixelBatchLmk[:, :, 0:2]` and `pixelLmk`.
    let batch_pixels: Vec<f64> = batch
        .iter()
        .flatten()
        .flat_map(|landmark| landmark.pixel.map(f64::from))
        .collect();
    let pixels: Vec<f64> = (0..results.spacecraft.values.len())
        .flat_map(|sample| {
            results
                .landmarks
                .iter()
                .flat_map(move |log| log.values[sample].pixel.map(f64::from))
        })
        .collect();
    [
        relative_difference(&batch_counts, &counts),
        relative_difference(&batch_visibility, &visibility),
        relative_difference(&batch_pixels, &pixels),
    ]
}

fn run(config: Config) -> Results {
    let degrees = PI / 180.0;
    // Upstream's Eros elements and spin state, January 21st, 2022.
    let right_ascension = 11.369 * degrees;
    let declination = 17.227 * degrees;
    let lst0 = 0.0;
    let mut eros_ephemeris = PlanetEphemeris::new(PlanetEphemerisConfig {
        name: "erosEphemeris".to_string(),
        bodies: vec![PlanetEphemerisBody {
            name: "eros".to_string(),
            elements: ClassicElements {
                semi_major_axis: 1.4583 * 149_597_870.7 * 1e3,
                eccentricity: 0.2227,
                inclination_rad: 10.829 * PI / 180.0,
                raan_rad: 304.3 * PI / 180.0,
                arg_periapsis_rad: 178.9 * PI / 180.0,
                true_anomaly_rad: 246.9 * PI / 180.0,
                ..Default::default()
            },
            rotation: Some(PlanetRotation {
                right_ascension_rad: right_ascension,
                declination_rad: declination,
                lst0_rad: lst0,
                rotation_rate_radps: 360.0 * degrees / (5.27 * 3600.0),
            }),
        }],
        zero_base: None,
    });
    let dcm_pn = euler323_to_dcm(Vector3::new(right_ascension, FRAC_PI_2 - declination, lst0));

    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "ephemConverter".to_string(),
        body_names: vec!["eros".to_string()],
    });

    // The spacecraft orbit, given in the Eros frame.
    let elements = ClassicElements {
        semi_major_axis: 34.0 * 1e3,
        eccentricity: 0.001,
        inclination_rad: 45.0 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    };
    let (r_p, v_p) = elements_to_state(EROS_MU_M3PS2, &elements);
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = spacecraft(
        inertia,
        &HubState {
            position_m: dcm_pn.transpose() * r_p,
            velocity_mps: dcm_pn.transpose() * v_p,
            sigma_bn: Vector3::new(0.7, 0.2, -0.3),
            omega_bn_b_radps: Vector3::new(0.001, -0.01, 0.03),
        },
    );
    spacecraft
        .add_grav_body(custom_point_mass("eros", EROS_MU_M3PS2, true))
        .expect("one central gravity body");

    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut guidance = hill_point();
    let mut tracking_error = att_tracking_error(Vector3::new(0.0, 1.0, 0.0));
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let shape = PolyhedronShape::from_pds3_tab(EROS_SHAPE);
    let (landmark_positions, landmark_normals) = landmark_distribution(&shape, LANDMARK_COUNT);
    let mut camera = PinholeCamera::new(PinholeCameraConfig {
        name: "pinholeCamera".to_string(),
        focal_length_m: 25.0 * 1e-3,
        nx_pixel: 2048.0,
        ny_pixel: 1536.0,
        pixel_width_m: (17.3 * 1e-3) / 2048.0,
        dcm_cb: dcm_cb(),
        ..Default::default()
    });
    for (position, normal) in landmark_positions.iter().zip(&landmark_normals) {
        camera.add_landmark(*position, *normal);
    }

    let mut spacecraft_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut asteroid_log = MessageLog::<EphemerisMsg>::default();
    let mut landmark_logs: Vec<MessageLog<LandmarkMsg>> =
        (0..LANDMARK_COUNT).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioSmallBodyLandmarks", config.write_csv);

    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2022, 1, 21), false);
        connect!(&simulation,
            &eros_ephemeris.planet_out_msgs[0] => spacecraft
                .grav_body_input_mut("eros")
                .expect("Eros is a gravity body"),
            &eros_ephemeris.planet_out_msgs[0] => &mut ephemeris_converter.planet_in_msgs[0],
            &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
        );
        spacecraft.add_dynamic_effector(external_torque);
        connect!(&simulation,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &navigation.translation_out_msg => &mut guidance.trans_nav_in_msg,
            &ephemeris_converter.ephem_out_msgs[0] => &mut guidance.cel_body_in_msg,
            &guidance.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
            &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
            &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
            &vehicle_config_msg => &mut controller.veh_config_in_msg,
            &ephemeris_converter.ephem_out_msgs[0] => &mut camera.ephemeris_in_msg,
            &spacecraft.state_out => &mut camera.sc_state_in_msg,
            &spacecraft.state_out => &mut spacecraft_log.input,
            &ephemeris_converter.ephem_out_msgs[0] => &mut asteroid_log.input,
            &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_m", "r_BN_N_m")
                        .into_iter()
                        .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
                        .chain(vector_columns("sigma_bn", "sigma_BN")),
                ),
            ),
            &ephemeris_converter.ephem_out_msgs[0] => recorder.add_source::<EphemerisMsg>(
                CsvSourceConfig::columns(
                    vector_columns("position_bdy_zero_n_m", "r_PN_N_m")
                        .into_iter()
                        .chain(vector_columns("velocity_bdy_zero_n_mps", "v_PN_N_m_per_s"))
                        .chain(vector_columns("sigma_bn", "sigma_PN")),
                ),
            ),
        );
        for (index, (output, log)) in camera
            .landmark_out_msgs
            .iter()
            .zip(&mut landmark_logs)
            .enumerate()
        {
            simulation.connect(output, &mut log.input);
            simulation.connect(
                output,
                recorder.add_source::<LandmarkMsg>(CsvSourceConfig::columns([
                    ("is_visible", format!("isVisible_{index}")),
                    ("pixel.0", format!("pL_{index}_0")),
                    ("pixel.1", format!("pL_{index}_1")),
                ])),
            );
        }
        // Upstream's `ModelPriority` values; the recorders follow.
        schedule! { simulation,
            "erosEphemeris" => &mut eros_ephemeris, period=TASK_PERIOD_NANOS, priority=100;
            "ephemConverter" => &mut ephemeris_converter, period=TASK_PERIOD_NANOS, priority=99;
            "bsk-Sat" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=98;
            "SimpleNavigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=96;
            "hillPoint" => &mut guidance, period=TASK_PERIOD_NANOS, priority=95;
            "attErrorInertial3D" => &mut tracking_error, period=TASK_PERIOD_NANOS, priority=94;
            "mrpFeedback" => &mut controller, period=TASK_PERIOD_NANOS, priority=93;
            "pinholeCamera" => &mut camera, period=TASK_PERIOD_NANOS, priority=92;
            "asteroidLog" => &mut asteroid_log, period=SAMPLE_PERIOD_NANOS, priority=0;
            "spacecraftLog" => &mut spacecraft_log, period=SAMPLE_PERIOD_NANOS, priority=0;
        }
        for (index, log) in landmark_logs.iter_mut().enumerate() {
            simulation.add_module(format!("landmarkLog{index}"), log, SAMPLE_PERIOD_NANOS, 0);
        }
        if config.write_csv {
            simulation.add_module("recorder", &mut recorder, SAMPLE_PERIOD_NANOS, 0);
        }
        simulation.run_for(DURATION_NANOS);
    }
    drop(recorder);

    let mut results = Results {
        spacecraft: spacecraft_log,
        asteroid: asteroid_log,
        landmarks: landmark_logs,
        batch_diff: None,
        csv_path,
    };
    if config.use_batch {
        results.batch_diff = Some(batch_difference(&camera, &results));
    }
    results
}

fn main() {
    let mut cli = Cli::from_env("scenario_small_body_landmarks [--[no-]batch] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, False)`.
        vec![Config::new(cli.flag("batch", false))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let counts = results.visible_counts();
        let orientation = results
            .planet_frame()
            .last()
            .expect("samples")
            .orientation();
        println!(
            "wrote {} ({} visible landmarks at t0, {} at tf; i_r.c3, i_h.c1, i_theta.c2 = \
             [{:.6}, {:.6}, {:.6}])",
            results.csv_path.as_ref().expect("CSV written").display(),
            counts[0],
            counts[counts.len() - 1],
            orientation.x,
            orientation.y,
            orientation.z,
        );
        if let Some(batch_diff) = results.batch_diff {
            println!(
                "processBatch vs simulation (batch_diff): {batch_diff:?}, within {BATCH_ACCURACY}: {}",
                batch_diff.iter().all(|value| *value <= BATCH_ACCURACY)
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// Upstream `test_scenarioSmallBodyLandmarks.py` (`useBatch = True`):
    /// `processBatch` on the recorded states reproduces the simulation's
    /// visible counts, visibility flags and pixels, every relative
    /// difference within `1e-6`. The documented geometry is checked on the
    /// same run: no landmark in view at `t0`, where the attitude has not
    /// converged; landmarks in view at `tf`, with the camera's `c3` on the
    /// negative radial direction, `c1` on the orbit normal and `c2` along
    /// track (`sigma_R0R` turns `b1` to `-i_r`, and `c3 = b1`).
    #[test]
    fn batch_processing_matches_the_simulation() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert_eq!(results.spacecraft.values.len(), 101);
        let batch_diff = results.batch_diff.expect("batch run");
        assert!(
            batch_diff.iter().all(|value| *value <= BATCH_ACCURACY),
            "batch_diff = {batch_diff:?}"
        );

        let counts = results.visible_counts();
        assert_eq!(counts[0], 0);
        assert!(counts[counts.len() - 1] > 0, "{counts:?}");
        let orientation = results
            .planet_frame()
            .last()
            .expect("samples")
            .orientation();
        assert!(
            (orientation - Vector3::new(-1.0, 1.0, 1.0)).norm() < 1.0e-3,
            "{orientation}"
        );
    }
}
