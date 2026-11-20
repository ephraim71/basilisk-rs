//! Mapping the Earth's surface with a nadir-pointed instrument: port of
//! upstream `examples/scenarioGroundMapping.py`.
//!
//! A 750 kg spacecraft (inertia `diag(900, 800, 600)` kg m^2) on a 2000 km,
//! 63.3 deg Earth orbit (`e = 0.01`) from `2020 MAY 21 18:28:03 (UTC)`
//! points its body +z axis at the Earth's center with `locationPointing`
//! (the Earth's ephemeris through `ephemerisConverter`), `mrpFeedback`
//! (`K = 5.5`, `P = 30`, no integral feedback) and an `extForceTorque`
//! effector. `groundMapping` checks 500 points spread over the rotating
//! Earth's surface (a Fibonacci sphere of the equatorial radius) against a
//! +z instrument cone (22.5 deg half angle) and a 45 deg minimum elevation;
//! a `mappingInstrument` produces 1 bit/s for each point in view, and a
//! `partitionedStorageUnit` (8e9 bit) keeps one partition per point. One 5 s
//! task, one hour.
//!
//! Upstream `run(show_plots, useCentral)` maps to `--central` /
//! `--no-central` (the Earth as the central gravity body or not); the
//! default and `--all` run upstream's test case, `useCentral = True`.
//! `--step <s>` (not an upstream parameter) changes the 5 s task period;
//! see below.
//!
//! # The 5 s attitude loop diverges
//!
//! With upstream's 5 s task the initial 136 deg slew to nadir does not
//! settle: as the body rate grows past about 0.1 rad/s (0.5 rad per step),
//! `locationPointing`'s finite-difference rate error and `mrpFeedback`'s
//! gyroscopic feed-forward terms drive it up instead of down, past 1 rad/s
//! about 70 s in, and the state later stops being finite. An independent
//! re-implementation of upstream's `locationPointing`, `mrpFeedback` and
//! rigid-body equations at the same step reproduces the attitude error of
//! this port to ten digits through the divergence, so this is upstream's
//! behaviour rather than an artifact of the port; the same loop with a
//! 0.5 s step converges to nadir within about 15 minutes. Upstream's test
//! only checks that the script raises no `OSError`, which non-finite states
//! do not. Here the run stops at the first step whose body rate exceeds
//! [`DIVERGED_RATE_RADPS`] and reports it; the CSV ends there.
//! `--step 0.5` runs the converging loop, to see the mapping.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `spiceInterface` (the Earth relative to the solar-system
//!   barycenter, with its `IAU_EARTH` orientation) is an ANISE ephemeris of
//!   the Earth with that orientation. With `--central` it is the Earth
//!   relative to itself, so the Earth is the inertial origin (upstream adds
//!   the Earth's barycentric position to the spacecraft's, which every
//!   relative quantity here cancels). With `--no-central` it is the Earth
//!   relative to the solar-system barycenter, and the spacecraft starts at
//!   the Earth's barycentric state from the same ephemeris (upstream
//!   `planetStates.planetPositionVelocity`) plus its orbit;
//! * upstream's task order is kept (navigation and the flight software
//!   first, then the storage unit, the instrument and `groundMapping`, each
//!   reading the previous step's outputs, then the spacecraft), except that
//!   the ephemeris runs just before the spacecraft rather than just after:
//!   the gravity effector takes the planet message as written at the current
//!   update;
//! * upstream's post-processing of its 500 access recorders and the storage
//!   recorder (`map_access`, `stored_points`, `sc_pos_PCPF`) is the
//!   example-local module [`MappingProgress`], which evaluates it every step.
//!
//! Writes `examples/output/scenarios/scenarioGroundMapping<True|False>.csv`
//! (upstream's `str(useCentral)`, with `Step<ms>ms` appended for a
//! `--step` other than 5 s) every step with `sigma_BR_<i>`,
//! `storedPoints`, `accessiblePoints` and the planet-fixed spacecraft
//! position `r_BP_P_m_<i>`, and
//! `scenarioGroundMapping<True|False>Points.csv` with each mapping point's
//! planet-fixed position `r_LP_P_m_<i>` and whether it was ever in view.

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
#[path = "support/staged.rs"]
mod staged;

use std::fmt::Write as _;
use std::path::PathBuf;

use anise::constants::frames::{EARTH_J2000, SSB_J2000};
use attitude::{
    HUB_MASS_KG, external_disturbance, hub_inertia_kg_m2, mrp_feedback, scenario_recorder,
    vehicle_config,
};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::environment::ground_mapping::{GroundMapping, GroundMappingConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::messages::{
    AccessMsg, AttitudeGuidanceMsg, DataStorageStatusMsg, Input, Output, PlanetStateMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::onboard_data_handling::mapping_instrument::MappingInstrument;
use basilisk_rs::onboard_data_handling::partitioned_storage_unit::PartitionedStorageUnit;
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvSourceConfig, TelemetryField, TelemetryMessage};
use basilisk_rs::utilities::astro_constants::{D2R, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::{scenario_output_path, vector_columns};
use grav_factory::{Body, spk_state};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};
use staged::UpdatedAtInit;

/// Upstream `simulationTimeStep = macros.sec2nano(5.0)`.
const TASK_PERIOD_NANOS: u64 = 5_000_000_000;
/// \[rad/s\] body rate above which the attitude loop has diverged and the
/// run stops (see the module docs).
const DIVERGED_RATE_RADPS: f64 = 1.0;
/// Upstream `macros.hour2nano(1)`.
const DURATION_NANOS: u64 = 3600 * 1_000_000_000;
/// Upstream `N`: the number of mapping points.
const POINT_COUNT: usize = 500;
/// Upstream `mapInstrument.nodeBaudRate` \[bit/s\].
const MAPPING_BAUD_BPS: f64 = 1.0;
/// Upstream `groundMap.minimumElevation` \[rad\].
const MINIMUM_ELEVATION_RAD: f64 = 45.0 * D2R;
/// Upstream `groundMap.halfFieldOfView` \[rad\].
const HALF_FIELD_OF_VIEW_RAD: f64 = 22.5 * D2R;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `useCentral`: the Earth is the central gravity body.
    use_central: bool,
    /// The task period \[ns\]; upstream's is [`TASK_PERIOD_NANOS`].
    task_period_nanos: u64,
    /// Write the CSV files (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_central: bool) -> Self {
        Self {
            use_central,
            task_period_nanos: TASK_PERIOD_NANOS,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        let suffix = if self.use_central { "True" } else { "False" };
        let step = if self.task_period_nanos == TASK_PERIOD_NANOS {
            String::new()
        } else {
            format!("Step{}ms", self.task_period_nanos / 1_000_000)
        };
        format!("scenarioGroundMapping{suffix}{step}")
    }
}

/// The parametrization of upstream `test_scenarioGroundMapping.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config::new(true)];

/// Upstream `timeInitString = '2020 MAY 21 18:28:03 (UTC)'`.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2020, 5, 21, 18, 28, 3, 0)
}

/// Upstream `generate_mapping_points(num_points, radius)`: a Fibonacci
/// sphere, `y` from `+radius` to `-radius`.
fn mapping_points(count: usize, radius_m: f64) -> Vec<Vector3<f64>> {
    let golden_angle = std::f64::consts::PI * (3.0 - 5.0_f64.sqrt());
    (0..count)
        .map(|index| {
            let y = 1.0 - (index as f64 / (count - 1) as f64) * 2.0;
            let radius = (1.0 - y * y).sqrt();
            let theta = golden_angle * index as f64;
            radius_m * Vector3::new(theta.cos() * radius, y, theta.sin() * radius)
        })
        .collect()
}

/// The quantities upstream derives from its recorders after the run, every
/// step.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct MappingProgressMsg {
    /// Upstream `stored_points`: partitions holding data.
    stored_points: f64,
    /// Points in view this step.
    accessible_points: f64,
    /// Upstream `sc_pos_PCPF` \[m\]: the spacecraft relative to the planet,
    /// planet-fixed components.
    r_bp_p_m: Vector3<f64>,
}

impl TelemetryMessage for MappingProgressMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        let field = |path: &str, value| TelemetryField {
            path: path.to_string(),
            value,
        };
        vec![
            field("stored_points", self.stored_points),
            field("accessible_points", self.accessible_points),
            field("r_bp_p_m.x", self.r_bp_p_m.x),
            field("r_bp_p_m.y", self.r_bp_p_m.y),
            field("r_bp_p_m.z", self.r_bp_p_m.z),
        ]
    }
}

/// Upstream's post-processing of `mapLog`, `storageLog`, `scLog` and
/// `planetLog`, evaluated each step: schedule it with the recorders.
#[derive(Default)]
struct MappingProgress {
    state_in: Input<SpacecraftStateMsg>,
    planet_in: Input<PlanetStateMsg>,
    storage_in: Input<DataStorageStatusMsg>,
    access_in: Vec<Input<AccessMsg>>,
    progress_out: Output<MappingProgressMsg>,
    /// The points in view at each update.
    in_view: Vec<Vec<usize>>,
}

impl Module for MappingProgress {
    fn init(&mut self) {
        self.in_view.clear();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let in_view: Vec<usize> = self
            .access_in
            .iter()
            .enumerate()
            .filter(|(_, input)| input.read().has_access)
            .map(|(index, _)| index)
            .collect();
        let state = self.state_in.read();
        let planet = self.planet_in.read();
        let dcm_pn = planet
            .orientation
            .map_or_else(Matrix3::identity, |orientation| {
                orientation.inertial_to_fixed
            });
        let storage = self.storage_in.read();
        self.progress_out.write(MappingProgressMsg {
            stored_points: storage
                .stored_data_bits
                .iter()
                .filter(|bits| **bits != 0.0)
                .count() as f64,
            accessible_points: in_view.len() as f64,
            r_bp_p_m: dcm_pn * (state.position_m - planet.position_inertial_m),
        });
        self.in_view.push(in_view);
    }
}

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `attErrLog`.
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// Upstream `storageLog`.
    storage: MessageLog<DataStorageStatusMsg>,
    /// Upstream `stored_points` and `sc_pos_PCPF`.
    progress: MessageLog<MappingProgressMsg>,
    /// The points in view at each step (upstream's `mapLog` `hasAccess`).
    in_view: Vec<Vec<usize>>,
    /// Upstream `mapping_points` \[m\], planet-fixed.
    points: Vec<Vector3<f64>>,
    /// When the run stopped on a diverged attitude \[s\], if it did.
    diverged_at_s: Option<f64>,
    csv_paths: Vec<PathBuf>,
}

fn run(config: Config) -> Results {
    let step = config.task_period_nanos;
    let earth = Body::Earth;
    let mu = earth.mu_m3ps2();
    let elements = ClassicElements {
        semi_major_axis: (6378.0 + 2000.0) * 1000.0,
        eccentricity: 0.01,
        inclination_rad: 63.3 * D2R,
        raan_rad: 88.2 * D2R,
        arg_periapsis_rad: 347.8 * D2R,
        true_anomaly_rad: 135.3 * D2R,
        ..Default::default()
    };
    let (mut initial_position_m, mut initial_velocity_mps) = elements_to_state(mu, &elements);
    let ephemeris = if config.use_central {
        earth.ephemeris(Body::Earth)
    } else {
        let (earth_position_m, earth_velocity_mps) =
            spk_state(EARTH_J2000, SSB_J2000, epoch(), &[]);
        initial_position_m += earth_position_m;
        initial_velocity_mps += earth_velocity_mps;
        earth.ephemeris_from_ssb()
    };
    let mut ephemeris = UpdatedAtInit::new(ephemeris, epoch());
    let planet = ephemeris.module.output_planet_msg.clone();

    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: step,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(config.use_central))
        .expect("one gravity body");
    planet.connect_to(
        spacecraft
            .grav_body_input_mut("earth")
            .expect("the Earth is a gravity body"),
    );
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let points = mapping_points(POINT_COUNT, REQ_EARTH * 1e3);
    let mut ground_map = GroundMapping::new(GroundMappingConfig {
        name: "groundMapping".to_string(),
        minimum_elevation_rad: MINIMUM_ELEVATION_RAD,
        maximum_range_m: Some(1e9),
        camera_pos_b_m: Vector3::zeros(),
        half_field_of_view_rad: HALF_FIELD_OF_VIEW_RAD,
        n_hat_b: Vector3::z(),
    });
    let mut instrument = MappingInstrument::new("mapInstrument");
    instrument.node_baud_rate_bps = MAPPING_BAUD_BPS;
    let mut storage = PartitionedStorageUnit::new("storageUnit");
    storage.storage_capacity_bits = 8_000_000_000;
    let mut progress = MappingProgress::default();
    for (index, point) in points.iter().enumerate() {
        ground_map.add_point_to_model(*point);
        instrument.add_mapping_point(index.to_string());
        let access = &ground_map.access_out_msgs[index];
        access.connect_to(&mut instrument.access_in_msgs[index]);
        progress.access_in.push(Input::default());
        access.connect_to(&mut progress.access_in[index]);
        instrument.data_node_out_msgs[index].connect_to(storage.add_data_node_to_model());
        storage.add_partition(index.to_string());
    }

    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "ephemConverter".to_string(),
        body_names: vec!["earth".to_string()],
    });
    let mut pointing = LocationPointing::new(LocationPointingConfig {
        name: "locPoint".to_string(),
        p_hat_b: Vector3::z(),
        ..LocationPointingConfig::default()
    });
    let mut controller = mrp_feedback(5.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));

    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut storage_log = MessageLog::<DataStorageStatusMsg>::default();
    let mut progress_log = MessageLog::<MappingProgressMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(epoch(), false);
    let state = spacecraft.state_out.clone();
    connect!(&simulation,
        &state => &mut navigation.spacecraft_state_in_msg,
        &state => &mut ground_map.sc_state_in_msg,
        &planet => &mut ground_map.planet_in_msg,
        &planet => &mut ephemeris_converter.planet_in_msgs[0],
        &navigation.attitude_out_msg => &mut pointing.sc_att_in_msg,
        &navigation.translation_out_msg => &mut pointing.sc_trans_in_msg,
        &ephemeris_converter.ephem_out_msgs[0] => &mut pointing.cel_body_in_msg,
        &pointing.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
        &state => &mut progress.state_in,
        &planet => &mut progress.planet_in,
        &storage.storage_unit_data_out_msg => &mut progress.storage_in,
        &pointing.att_guid_out_msg => &mut guidance_log.input,
        &storage.storage_unit_data_out_msg => &mut storage_log.input,
        &progress.progress_out => &mut progress_log.input,
        &pointing.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
        &progress.progress_out => recorder.add_source::<MappingProgressMsg>(
            CsvSourceConfig::columns(
                [
                    ("stored_points".to_string(), "storedPoints".to_string()),
                    ("accessible_points".to_string(), "accessiblePoints".to_string()),
                ]
                .into_iter()
                .chain(vector_columns("r_bp_p_m", "r_BP_P_m")),
            ),
        ),
    );
    spacecraft.add_dynamic_effector(external_torque);

    // Upstream's priorities: navigation (99), pointing (97), control (96;
    // the torque effector at 95 is part of the spacecraft), the storage unit
    // (3), the instrument (2), `groundMapping` (1), then at the default
    // priority the spacecraft, the ephemeris (moved ahead of it, see the
    // module docs), the ephemeris converter and the recorders.
    schedule! { simulation,
        "navigation" => &mut navigation, period=step, priority=99;
        "locPoint" => &mut pointing, period=step, priority=97;
        "mrpFeedback" => &mut controller, period=step, priority=96;
        "storageUnit" => &mut storage, period=step, priority=3;
        "mapInstrument" => &mut instrument, period=step, priority=2;
        "groundMapping" => &mut ground_map, period=step, priority=1;
        "spice" => &mut ephemeris, period=step, priority=-1;
        "spacecraft" => &mut spacecraft, period=step, priority=-2;
        "ephemConverter" => &mut ephemeris_converter, period=step, priority=-3;
        "mappingProgress" => &mut progress, period=step, priority=-10;
        "attErrLog" => &mut guidance_log, period=step, priority=-10;
        "storageLog" => &mut storage_log, period=step, priority=-10;
        "progressLog" => &mut progress_log, period=step, priority=-10;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, step, -20);
    }
    let completed = simulation.run_for_chunked_while(DURATION_NANOS, step, || {
        state.read().omega_radps.norm() < DIVERGED_RATE_RADPS
    });
    let diverged_at_s = (!completed).then(|| simulation.current_sim_nanos() as f64 * 1e-9);
    drop(simulation);
    drop(recorder);

    let in_view = std::mem::take(&mut progress.in_view);
    let mut csv_paths: Vec<PathBuf> = csv_path.into_iter().collect();
    if config.write_csv {
        csv_paths.push(write_points_csv(&config.file_stem(), &points, &in_view));
    }
    Results {
        guidance: guidance_log,
        storage: storage_log,
        progress: progress_log,
        in_view,
        points,
        diverged_at_s,
        csv_paths,
    }
}

/// Upstream `map_access`: which points were ever in view.
fn ever_in_view(in_view: &[Vec<usize>]) -> Vec<bool> {
    let mut mapped = vec![false; POINT_COUNT];
    for index in in_view.iter().flatten() {
        mapped[*index] = true;
    }
    mapped
}

/// Writes `<stem>Points.csv`: each point's planet-fixed position and whether
/// it was ever in view.
fn write_points_csv(stem: &str, points: &[Vector3<f64>], in_view: &[Vec<usize>]) -> PathBuf {
    let mapped = ever_in_view(in_view);
    let mut text = String::from("point,r_LP_P_m_0,r_LP_P_m_1,r_LP_P_m_2,mapped\n");
    for (index, (point, mapped)) in points.iter().zip(mapped).enumerate() {
        writeln!(
            text,
            "{index},{:.18e},{:.18e},{:.18e},{}",
            point.x,
            point.y,
            point.z,
            u8::from(mapped)
        )
        .expect("writing to a string");
    }
    let path = scenario_output_path(&format!("{stem}Points.csv"));
    std::fs::write(&path, text).expect("write the mapping points CSV");
    path
}

fn main() {
    let mut cli = Cli::from_env("scenario_ground_mapping [--[no-]central] [--step <s>] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, True)`.
        let step_s: f64 = cli.value("step", TASK_PERIOD_NANOS as f64 * 1e-9);
        vec![Config {
            task_period_nanos: (step_s * 1e9).round() as u64,
            ..Config::new(cli.flag("central", true))
        }]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let mapped = ever_in_view(&results.in_view)
            .into_iter()
            .filter(|mapped| *mapped)
            .count();
        for path in &results.csv_paths {
            println!("wrote {}", path.display());
        }
        println!(
            "{mapped} of {POINT_COUNT} points mapped, {} stored",
            results.progress.last().stored_points
        );
        if let Some(time) = results.diverged_at_s {
            println!(
                "the attitude loop diverged: stopped at {time} s, when the body rate passed \
                 {DIVERGED_RATE_RADPS} rad/s (see the scenario docs; --step 0.5 converges)"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioGroundMapping.py` only checks that
    //! `run(False, True)` runs. With upstream's 5 s task the attitude loop
    //! diverges (see the module docs); these tests check that, and, with
    //! the converging 0.5 s task, what the scenario demonstrates: the
    //! instrument turns to nadir and stays there, a point is mapped only
    //! from within view, and each point's partition holds 1 bit/s for every
    //! step the point was in view.

    use super::*;

    /// A task period at which the attitude loop converges (see the module
    /// docs).
    const CONVERGING_TASK_PERIOD_NANOS: u64 = 500_000_000;

    fn quiet(use_central: bool, task_period_nanos: u64) -> Results {
        run(Config {
            use_central,
            task_period_nanos,
            write_csv: false,
        })
    }

    /// The documented divergence of upstream's 5 s loop.
    #[test]
    fn the_five_second_loop_diverges() {
        let results = quiet(true, TASK_PERIOD_NANOS);
        let stopped = results.diverged_at_s.expect("the 5 s loop diverges");
        assert!((40.0..200.0).contains(&stopped), "stopped at {stopped} s");
        // It first closes in on nadir, then turns away.
        let closest = results
            .guidance
            .values
            .iter()
            .map(|guidance| guidance.sigma_br.norm())
            .fold(f64::INFINITY, f64::min);
        assert!(closest < 0.3);
        assert!(results.guidance.last().sigma_br.norm() > 0.5);
    }

    #[test]
    fn the_instrument_turns_to_nadir() {
        let results = quiet(true, CONVERGING_TASK_PERIOD_NANOS);
        assert_eq!(results.diverged_at_s, None);
        assert_eq!(
            results.guidance.values.len() as u64,
            DURATION_NANOS / CONVERGING_TASK_PERIOD_NANOS + 1
        );
        // From 15 minutes on, the boresight is on nadir to better than
        // 0.005 (MRP, about 1.1 deg).
        let worst = results
            .guidance
            .since(15 * 60 * 1_000_000_000)
            .map(|guidance| guidance.sigma_br.norm())
            .fold(0.0, f64::max);
        assert!(worst < 0.005, "|sigma_BR| up to {worst}");
    }

    #[test]
    fn mapped_points_lie_under_the_spacecraft() {
        let results = quiet(true, CONVERGING_TASK_PERIOD_NANOS);
        // The 45 deg elevation mask limits the view to points within the
        // central angle at which the spacecraft is 45 deg up. `groundMapping`
        // runs before the spacecraft and the ephemeris: each step's access is
        // that of the previous step's geometry.
        let r_ground = REQ_EARTH * 1e3;
        for (in_view, progress) in results.in_view[1..].iter().zip(&results.progress.values) {
            let r_sc = progress.r_bp_p_m.norm();
            let cos_e = MINIMUM_ELEVATION_RAD.cos();
            let limit = (r_ground * cos_e / r_sc).acos() - MINIMUM_ELEVATION_RAD;
            for index in in_view {
                let angle = results.points[*index].angle(&progress.r_bp_p_m);
                assert!(angle < limit + 1e-9, "point {index}: {angle} rad");
            }
        }
        let mapped = ever_in_view(&results.in_view)
            .into_iter()
            .filter(|mapped| *mapped)
            .count();
        assert!((5..100).contains(&mapped), "{mapped} points mapped");
    }

    #[test]
    fn partitions_hold_one_bit_per_second_in_view() {
        let results = quiet(true, CONVERGING_TASK_PERIOD_NANOS);
        let steps = results.in_view.len();
        // The storage unit runs before the instrument, which runs before
        // `groundMapping`: a step in view reaches storage two steps later.
        // The unit stores the rate times its step in whole bits, the step
        // being the difference of the two times in seconds: half a bit,
        // rounded to 1, or to 0 where that difference falls just short.
        let step = CONVERGING_TASK_PERIOD_NANOS;
        let seconds = |tick: usize| (tick as u64 * step) as f64 * 1.0e-9;
        let mut expected = vec![0.0; POINT_COUNT];
        for (tick, in_view) in results.in_view[..steps - 2].iter().enumerate() {
            let bits = (MAPPING_BAUD_BPS * (seconds(tick + 2) - seconds(tick + 1))).round();
            for index in in_view {
                expected[*index] += bits;
            }
        }
        assert_eq!(results.storage.last().stored_data_bits, expected);
        // Stored points only grow, and end at the points stored so far.
        let stored: Vec<f64> = results
            .progress
            .values
            .iter()
            .map(|progress| progress.stored_points)
            .collect();
        assert!(stored.windows(2).all(|pair| pair[0] <= pair[1]));
        let nonzero = expected.iter().filter(|bits| **bits != 0.0).count() as f64;
        assert_eq!(results.progress.last().stored_points, nonzero);
        assert!(nonzero > 0.0);
    }

    #[test]
    fn the_non_central_earth_maps_much_the_same_region() {
        // Without a central body the spacecraft is integrated about the
        // solar-system barycenter under the Earth's gravity alone, so the
        // Earth's solar acceleration (about 6 mm/s^2) separates it from the
        // central case by tens of km in the hour.
        let central = quiet(true, CONVERGING_TASK_PERIOD_NANOS);
        let free = quiet(false, CONVERGING_TASK_PERIOD_NANOS);
        assert_eq!(free.diverged_at_s, None);
        let gap = (central.progress.last().r_bp_p_m - free.progress.last().r_bp_p_m).norm();
        assert!((1e3..1e5).contains(&gap), "{gap} m apart");
        let central_mapped = ever_in_view(&central.in_view);
        let free_mapped = ever_in_view(&free.in_view);
        let both = central_mapped
            .iter()
            .zip(&free_mapped)
            .filter(|(a, b)| **a && **b)
            .count();
        let count = central_mapped.iter().filter(|m| **m).count();
        assert!(both * 2 > count, "{both} of {count} shared");
    }
}
