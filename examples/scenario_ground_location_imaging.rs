//! Imaging ground targets and downlinking the images: port of upstream
//! `examples/scenarioGroundLocationImaging.py`.
//!
//! A 750 kg spacecraft (inertia `diag(900, 800, 600)` kg m^2) on a 600 km,
//! 63.3 deg Earth orbit (`e = 0.01`) about a point-mass, non-rotating Earth
//! (no planet ephemeris, as upstream) points its body +z axis at a
//! `groundLocation` imaging target with `locationPointing`, `mrpFeedback`
//! (`K = 5.5`, `P = 30`, no integral feedback) and an `extForceTorque`
//! effector. A `simpleInstrumentController` commands one image (an 8e6 bit
//! `simpleInstrument` burst) once the target is accessible (10 deg minimum
//! elevation) and `|sigma_BR| < 0.1`. The images go into a
//! `partitionedStorageUnit` (2e7 bit, partitions `boulder` and `santiago`),
//! from which a `spaceToGroundTransmitter` (-1e5 baud, -8e6 bit packets)
//! downlinks while a Singapore station (5 deg minimum elevation) has access.
//! One 1 s task: 20 minutes targeting Boulder, Colorado, then 60 minutes
//! targeting Santiago, Chile.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case.
//!
//! Mapping to basilisk-rs: between its two `ExecuteSimulation` calls upstream
//! moves the target (`imagingTarget.specifyLocation`), renames the
//! instrument's data (`instrument.nodeDataName = "santiago"`) and re-arms the
//! controller (`simpleInsControl.imaged = 0`). Here those are [`Staged`]
//! changes after 20 minutes; the target's move resets the ground location,
//! whose reset only re-resolves the site, as `specifyLocation` does.
//!
//! # The slew to Santiago diverges
//!
//! `locationPointing` constrains only the boresight, so the spin about it
//! that the first phase leaves (about 0.058 rad/s, grown from the initial
//! 0.03 rad/s) is free. When the target jumps to Santiago, `|sigma_BR|`
//! jumps from 0.002 to 0.57 in one 1 s step and `locationPointing`'s
//! finite-difference rate error jumps with it (to about 2 rad/s);
//! `mrpFeedback`'s rate and gyroscopic feed-forward terms then command about
//! 160 N m, and with the boresight spin the loop spins the spacecraft up: its
//! rate passes 1 rad/s 11 s after the switch and grows without bound. An
//! independent re-implementation of upstream's `locationPointing`,
//! `mrpFeedback` and rigid-body equations, run from `t = 0`, reproduces the
//! state at the switch and diverges at the same step, so this is upstream's
//! behaviour rather than an artifact of the port (with no boresight spin at
//! the switch the same slew converges). Upstream's documentation describes
//! the switch to Santiago and its image; its test only checks that the
//! script raises no `OSError`, which non-finite states do not. Here the run stops at the
//! first step whose body rate exceeds [`DIVERGED_RATE_RADPS`] and reports
//! it; the CSV ends there.
//!
//! Writes `examples/output/scenarios/scenarioGroundLocationImaging.csv` every
//! step with upstream's recorders: `sigma_BR_<i>`, the target's `hasAccess`,
//! the controller's `deviceCmd`, the storage unit's `storageLevel_bits`,
//! `currentNetBaud_bps` and `storedData_bits_<i>`, and the transmitter's
//! `transmitterBaud_bps`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/data_system.rs"]
mod data_system;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::{
    HUB_MASS_KG, external_disturbance, hub_inertia_kg_m2, mrp_feedback, scenario_recorder,
    vehicle_config,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::ground_location::{
    GroundLocation, GroundLocationConfig, SiteLocation,
};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::fsw_algorithms::sensor_interfaces::simple_instrument_controller::{
    SimpleInstrumentController, SimpleInstrumentControllerConfig,
};
use basilisk_rs::messages::{
    AccessMsg, AttitudeGuidanceMsg, DataNodeUsageMsg, DataStorageStatusMsg, DeviceCommandMsg,
    Output,
};
use basilisk_rs::onboard_data_handling::partitioned_storage_unit::PartitionedStorageUnit;
use basilisk_rs::onboard_data_handling::simple_instrument::SimpleInstrument;
use basilisk_rs::onboard_data_handling::space_to_ground_transmitter::SpaceToGroundTransmitter;
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::geodetic_conversion::Lla;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use data_system::storage_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use staged::Staged;

/// Upstream `simulationTimeStep = macros.sec2nano(1.0)`.
const TASK_PERIOD_NANOS: u64 = 1_000_000_000;
/// Upstream `simulationTime = macros.min2nano(20.0)`: the Boulder phase.
const FIRST_PHASE_NANOS: u64 = 20 * 60 * 1_000_000_000;
/// Upstream's second stop, `simulationTime + 3 * simulationTime`.
const STOP_NANOS: u64 = 4 * FIRST_PHASE_NANOS;
/// Upstream `instrument.nodeBaudRate` \[bit/s\]: one image per 1 s step.
const IMAGE_BAUD_BPS: f64 = 8e6;
/// Upstream `transmitter.nodeBaudRate` \[bit/s\].
const TRANSMITTER_BAUD_BPS: f64 = -1e5;
/// Upstream `transmitter.packetSize` \[bit\].
const PACKET_SIZE_BITS: f64 = -8e6;
/// Upstream `dataMonitor.storageCapacity` \[bit\].
const STORAGE_CAPACITY_BITS: i64 = 20_000_000;
/// Upstream `simpleInsControl.attErrTolerance` (MRP norm).
const ATT_ERR_TOLERANCE: f64 = 0.1;
/// Upstream's partitions, also the instrument's data names.
const PARTITIONS: [&str; 2] = ["boulder", "santiago"];
/// \[rad/s\] body rate above which the attitude loop has diverged and the
/// run stops (see the module docs).
const DIVERGED_RATE_RADPS: f64 = 1.0;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `attErrLog`.
    guidance: MessageLog<AttitudeGuidanceMsg>,
    /// Upstream `locationLog`: the imaging target's access.
    target_access: MessageLog<AccessMsg>,
    /// The Singapore station's access (upstream records none).
    station_access: MessageLog<AccessMsg>,
    /// Upstream `deviceLog`.
    device: MessageLog<DeviceCommandMsg>,
    /// Upstream `dataMonLog`.
    storage: MessageLog<DataStorageStatusMsg>,
    /// Upstream `transmitterLog`.
    transmitter: MessageLog<DataNodeUsageMsg>,
    /// When the run stopped on a diverged attitude \[s\], if it did.
    diverged_at_s: Option<u64>,
    csv_path: Option<PathBuf>,
}

/// Upstream `specifyLocation(np.radians(lat), np.radians(lon), alt)`.
fn site(latitude_deg: f64, longitude_deg: f64, altitude_m: f64) -> SiteLocation {
    SiteLocation::Geodetic(Lla {
        latitude_rad: latitude_deg * D2R,
        longitude_rad: longitude_deg * D2R,
        altitude_m,
    })
}

fn run(config: Config) -> Results {
    let mu = MU_EARTH * 1e9;
    let elements = ClassicElements {
        semi_major_axis: (6378.0 + 600.0) * 1000.0,
        eccentricity: 0.01,
        inclination_rad: 63.3 * D2R,
        raan_rad: 88.2 * D2R,
        arg_periapsis_rad: 347.8 * D2R,
        true_anomaly_rad: 135.3 * D2R,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let inertia = hub_inertia_kg_m2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: inertia,
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut navigation = SimpleNavigation::new("SimpleNavigation");

    let mut imaging_target = Staged::new(GroundLocation::new(GroundLocationConfig {
        name: "ImagingTarget".to_string(),
        planet_radius_m: REQ_EARTH * 1e3,
        minimum_elevation_rad: 10.0 * D2R,
        maximum_range_m: Some(1e9),
        location: site(40.009971, -105.243895, 1624.0),
    }))
    .after(FIRST_PHASE_NANOS, |target, context| {
        target.config.location = site(-33.4489, -70.6693, 570.0);
        basilisk_rs::Module::reset(target, context);
    });
    let target_index = imaging_target.module.add_spacecraft_to_model();
    let mut singapore = GroundLocation::new(GroundLocationConfig {
        name: "SingaporeStation".to_string(),
        planet_radius_m: REQ_EARTH * 1e3,
        minimum_elevation_rad: 5.0 * D2R,
        maximum_range_m: Some(1e9),
        location: site(1.3521, 103.8198, 15.0),
    });
    let singapore_index = singapore.add_spacecraft_to_model();

    let mut transmitter = SpaceToGroundTransmitter::new("transmitter");
    transmitter.node_baud_rate_bps = TRANSMITTER_BAUD_BPS;
    transmitter.model.packet_size_bits = PACKET_SIZE_BITS;
    let mut instrument = SimpleInstrument::new("instrument1");
    instrument.node_baud_rate_bps = IMAGE_BAUD_BPS;
    instrument.node_data_name = PARTITIONS[0].to_string();
    let mut instrument = Staged::new(instrument).after(FIRST_PHASE_NANOS, |instrument, _| {
        instrument.node_data_name = PARTITIONS[1].to_string();
    });
    let mut storage = PartitionedStorageUnit::new("dataMonitor");
    storage.storage_capacity_bits = STORAGE_CAPACITY_BITS;

    let mut pointing = LocationPointing::new(LocationPointingConfig {
        name: "locPoint".to_string(),
        p_hat_b: Vector3::z(),
        ..LocationPointingConfig::default()
    });
    let mut controller = mrp_feedback(5.5, -1.0, 30.0, Vector3::zeros());
    let vehicle_config_msg = Output::new(vehicle_config(inertia));
    let mut instrument_controller = Staged::new(SimpleInstrumentController::new(
        SimpleInstrumentControllerConfig {
            name: "instrumentController".to_string(),
            att_err_tolerance: ATT_ERR_TOLERANCE,
            ..SimpleInstrumentControllerConfig::default()
        },
    ))
    .after(FIRST_PHASE_NANOS, |controller, _| controller.imaged = false);

    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let mut target_access_log = MessageLog::<AccessMsg>::default();
    let mut station_access_log = MessageLog::<AccessMsg>::default();
    let mut device_log = MessageLog::<DeviceCommandMsg>::default();
    let mut storage_log = MessageLog::<DataStorageStatusMsg>::default();
    let mut transmitter_log = MessageLog::<DataNodeUsageMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioGroundLocationImaging", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    let state = spacecraft.state_out.clone();
    let target = &mut imaging_target.module;
    let target_access = target.access_out_msgs[target_index].clone();
    let station_access = singapore.access_out_msgs[singapore_index].clone();
    connect!(&simulation,
        &state => &mut navigation.spacecraft_state_in_msg,
        &state => &mut target.sc_state_in_msgs[target_index],
        &state => &mut singapore.sc_state_in_msgs[singapore_index],
        &station_access => transmitter.add_access_msg_to_transmitter(),
        &storage.storage_unit_data_out_msg => transmitter.add_storage_unit_to_transmitter(),
        &instrument.module.node_data_out_msg => storage.add_data_node_to_model(),
        &transmitter.node_data_out_msg => storage.add_data_node_to_model(),
        &navigation.attitude_out_msg => &mut pointing.sc_att_in_msg,
        &navigation.translation_out_msg => &mut pointing.sc_trans_in_msg,
        &target.current_ground_state_out_msg => &mut pointing.location_in_msg,
        &pointing.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &controller.cmd_torque_out_msg => &mut external_torque.cmd_torque_in_msg,
        &pointing.att_guid_out_msg => &mut instrument_controller.module.att_guid_in_msg,
        &target_access => &mut instrument_controller.module.location_access_in_msg,
        &instrument_controller.module.device_cmd_out_msg => &mut instrument.module.node_status_in_msg,
        &pointing.att_guid_out_msg => &mut guidance_log.input,
        &target_access => &mut target_access_log.input,
        &station_access => &mut station_access_log.input,
        &instrument_controller.module.device_cmd_out_msg => &mut device_log.input,
        &storage.storage_unit_data_out_msg => &mut storage_log.input,
        &transmitter.node_data_out_msg => &mut transmitter_log.input,
        &pointing.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(vector_columns("sigma_br", "sigma_BR")),
        ),
        &target_access => recorder.add_source::<AccessMsg>(
            CsvSourceConfig::columns([("has_access", "hasAccess")]),
        ),
        &instrument_controller.module.device_cmd_out_msg => recorder
            .add_source::<DeviceCommandMsg>(CsvSourceConfig::columns([("device_cmd", "deviceCmd")])),
        &storage.storage_unit_data_out_msg => recorder
            .add_source::<DataStorageStatusMsg>(storage_columns(PARTITIONS.len())),
        &transmitter.node_data_out_msg => recorder.add_source::<DataNodeUsageMsg>(
            CsvSourceConfig::columns([("baud_rate_bps", "transmitterBaud_bps")]),
        ),
    );
    for name in PARTITIONS {
        storage.add_partition(name);
    }
    spacecraft.add_dynamic_effector(external_torque);

    // Upstream's priorities: the spacecraft, navigation and both ground
    // locations (100), the transmitter and pointing (99), the controller
    // (98), the instrument controller (97), the instrument (90; the torque
    // effector there is part of the spacecraft), the storage unit (89), then
    // the recorders.
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=100;
        "navigation" => &mut navigation, period=TASK_PERIOD_NANOS, priority=100;
        "imagingTarget" => &mut imaging_target, period=TASK_PERIOD_NANOS, priority=100;
        "singaporeStation" => &mut singapore, period=TASK_PERIOD_NANOS, priority=100;
        "transmitter" => &mut transmitter, period=TASK_PERIOD_NANOS, priority=99;
        "locPoint" => &mut pointing, period=TASK_PERIOD_NANOS, priority=99;
        "mrpFeedback" => &mut controller, period=TASK_PERIOD_NANOS, priority=98;
        "instrumentController" => &mut instrument_controller, period=TASK_PERIOD_NANOS, priority=97;
        "instrument1" => &mut instrument, period=TASK_PERIOD_NANOS, priority=90;
        "dataMonitor" => &mut storage, period=TASK_PERIOD_NANOS, priority=89;
        "attErrLog" => &mut guidance_log, period=TASK_PERIOD_NANOS, priority=0;
        "locationLog" => &mut target_access_log, period=TASK_PERIOD_NANOS, priority=0;
        "stationLog" => &mut station_access_log, period=TASK_PERIOD_NANOS, priority=0;
        "deviceLog" => &mut device_log, period=TASK_PERIOD_NANOS, priority=0;
        "dataMonLog" => &mut storage_log, period=TASK_PERIOD_NANOS, priority=0;
        "transmitterLog" => &mut transmitter_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(FIRST_PHASE_NANOS);
    let completed =
        simulation.run_for_chunked_while(STOP_NANOS - FIRST_PHASE_NANOS, TASK_PERIOD_NANOS, || {
            state.read().omega_radps.norm() < DIVERGED_RATE_RADPS
        });
    let diverged_at_s = (!completed).then(|| simulation.current_sim_nanos() / 1_000_000_000);
    drop(simulation);
    drop(recorder);

    Results {
        guidance: guidance_log,
        target_access: target_access_log,
        station_access: station_access_log,
        device: device_log,
        storage: storage_log,
        transmitter: transmitter_log,
        diverged_at_s,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_ground_location_imaging [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let images: Vec<u64> = results
            .device
            .times_nanos
            .iter()
            .zip(&results.device.values)
            .filter(|(_, command)| command.device_cmd == 1)
            .map(|(time, _)| time / 1_000_000_000)
            .collect();
        println!(
            "wrote {} (images taken at {images:?} s; stored {:?} bit at the end)",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.storage.last().stored_data_bits,
        );
        if let Some(time) = results.diverged_at_s {
            println!(
                "the slew to Santiago diverged: stopped at {time} s, when the body rate passed \
                 {DIVERGED_RATE_RADPS} rad/s (see the scenario docs)"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it runs. These tests check what it demonstrates up to the
    //! documented divergence: the controller images Boulder once, when it is
    //! in view and the pointing has settled, the image lands in its
    //! partition, and the transmitter would downlink only over Singapore.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    /// `(time, partition)` of every image taken.
    fn images(results: &Results) -> Vec<(u64, usize)> {
        results
            .device
            .times_nanos
            .iter()
            .zip(&results.device.values)
            .filter(|(_, command)| command.device_cmd == 1)
            .map(|(time, _)| (*time, usize::from(*time > FIRST_PHASE_NANOS)))
            .collect()
    }

    #[test]
    fn boulder_is_imaged_once_when_in_view_and_pointed() {
        let results = quiet();
        let images = images(&results);
        // Santiago is never imaged: the run stops in the slew to it.
        assert_eq!(images.len(), 1, "images {images:?}");
        let (time, partition) = images[0];
        assert_eq!(partition, 0);
        let access = results.target_access.at(time);
        let guidance = results.guidance.at(time);
        assert!(access.has_access);
        assert!(guidance.sigma_br.norm() < ATT_ERR_TOLERANCE);
        // The first step meeting both constraints.
        for earlier in results.guidance.times_nanos.iter().filter(|t| **t < time) {
            let access = results.target_access.at(*earlier);
            let guidance = results.guidance.at(*earlier);
            assert!(!access.has_access || guidance.sigma_br.norm() >= ATT_ERR_TOLERANCE);
        }
    }

    #[test]
    fn the_image_fills_its_partition() {
        let results = quiet();
        let images = images(&results);
        for (index, storage) in results.storage.values.iter().enumerate() {
            let time = results.storage.times_nanos[index];
            assert_eq!(storage.stored_data_name, PARTITIONS);
            let sending = results.transmitter.values[index].baud_rate_bps;
            assert!(sending == 0.0 || sending == TRANSMITTER_BAUD_BPS);
            if sending != 0.0 {
                assert!(results.station_access.values[index].has_access);
            }
            if index == 0 {
                continue;
            }
            let previous = &results.storage.values[index - 1];
            let image = images.iter().find(|(image_time, _)| *image_time == time);
            for (partition, name) in PARTITIONS.iter().enumerate() {
                let mut expected = previous.stored_data_bits[partition];
                if image.is_some_and(|(_, imaged)| *imaged == partition) {
                    expected += IMAGE_BAUD_BPS;
                }
                if sending != 0.0 && results.transmitter.values[index].data_name == *name {
                    expected = (expected + sending).max(0.0);
                }
                assert_eq!(
                    storage.stored_data_bits[partition], expected,
                    "at {time} ns"
                );
            }
        }
        // Singapore is not in view before the run stops: the image stays.
        assert!(
            results
                .station_access
                .values
                .iter()
                .all(|access| !access.has_access)
        );
        assert_eq!(
            results.storage.last().stored_data_bits,
            [IMAGE_BAUD_BPS, 0.0]
        );
    }

    #[test]
    fn the_pointing_settles_on_boulder() {
        let results = quiet();
        // Within ten minutes the boresight is on the target to better than
        // 0.01 (MRP, about 2.3 deg), and stays there.
        let worst = results
            .guidance
            .times_nanos
            .iter()
            .zip(&results.guidance.values)
            .filter(|(time, _)| (10 * 60 * TASK_PERIOD_NANOS..=FIRST_PHASE_NANOS).contains(*time))
            .map(|(_, guidance)| guidance.sigma_br.norm())
            .fold(0.0, f64::max);
        assert!(worst < 0.01, "|sigma_BR| up to {worst}");
    }

    /// The documented divergence of the slew to Santiago.
    #[test]
    fn the_slew_to_santiago_diverges() {
        let results = quiet();
        let stopped = results.diverged_at_s.expect("the slew diverges");
        let switch_s = FIRST_PHASE_NANOS / TASK_PERIOD_NANOS;
        assert!(
            (switch_s + 5..switch_s + 30).contains(&stopped),
            "stopped at {stopped} s"
        );
        let last = results.guidance.times_nanos.last().copied();
        assert_eq!(last, Some(stopped * TASK_PERIOD_NANOS));
    }
}
