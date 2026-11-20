//! Data downlink gated by access to a ground station: port of upstream
//! `examples/scenarioGroundDownlink.py`.
//!
//! A small satellite on an ISS-like orbit (`a = R_E + 418 km`,
//! `e = 0.00061`, `i = 51.6418 deg`) about the J2 Earth (`GGM03S-J2-only`
//! to degree 2) from `2020 MAY 21 18:28:03 (UTC)`, the hub at upstream's
//! defaults (1 kg, identity inertia), not rotating. A `groundLocation` in
//! Boulder, Colorado (40.009971 deg N, 105.243895 deg W, 1624 m; 10 deg
//! minimum elevation) tracks it on the rotating Earth. Two
//! `simpleInstrument`s (2400 baud each) collect data continuously into the
//! data system of [`data_system`]; a `spaceToGroundTransmitter`
//! (-9600 baud, -1e6 bit packets) downlinks it while Boulder has access.
//! One 10 s task, 24 hours.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case. Upstream stops at 12 h only to recolour a Vizard
//! location, so the run here is one 24 h stretch.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `spiceInterface` (the Earth relative to the solar-system
//!   barycenter, with its `IAU_EARTH` orientation) is an ANISE ephemeris of
//!   the Earth relative to itself with that orientation
//!   ([`Body::ephemeris`]), so the Earth is the inertial origin; it rotates
//!   the ground station and orients the J2 field. Upstream's plots subtract
//!   the Earth's position from the spacecraft's; here `r_BN_N` already is
//!   relative to the Earth. The ephemeris runs before the spacecraft
//!   (upstream gives the spacecraft priority 1, ahead of it): the gravity
//!   effector takes the planet message as written at the current update;
//! * upstream's `transmitter.numBuffers` is never read upstream and is not
//!   ported.
//!
//! Writes `examples/output/scenarios/scenarioGroundDownlink.csv` every step
//! with the partitioned unit's `storageLevel_bits`, `currentNetBaud_bps` and
//! `storedData_bits_<i>`, the spacecraft `r_BN_N_m_<i>`, the station's
//! `r_LP_N_m_<i>`, and its access: `hasAccess`, `elevation_rad`,
//! `azimuth_rad` and `slantRange_m`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/data_system.rs"]
mod data_system;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::environment::ground_location::{
    GroundLocation, GroundLocationConfig, SiteLocation,
};
use basilisk_rs::messages::{
    AccessMsg, DataNodeUsageMsg, DataStorageStatusMsg, GroundStateMsg, SpacecraftStateMsg,
};
use basilisk_rs::onboard_data_handling::space_to_ground_transmitter::SpaceToGroundTransmitter;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, REQ_EARTH};
use basilisk_rs::utilities::geodetic_conversion::Lla;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use data_system::{DataSystem, PARTITIONS, storage_columns};
use grav_factory::{Body, GGM03S_J2_ONLY};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use staged::UpdatedAtInit;

/// Upstream `testProcessRate = macros.sec2nano(10.0)`.
const TASK_PERIOD_NANOS: u64 = 10_000_000_000;
/// Upstream `macros.hour2nano(24)`.
const DURATION_NANOS: u64 = 24 * 3600 * 1_000_000_000;
/// Upstream `instrument.nodeBaudRate` of each instrument \[bit/s\].
const INSTRUMENT_BAUD_BPS: f64 = 2400.0;
/// Upstream `transmitter.nodeBaudRate` \[bit/s\].
const TRANSMITTER_BAUD_BPS: f64 = -9600.0;
/// Upstream `transmitter.packetSize` \[bit\].
const PACKET_SIZE_BITS: f64 = -1e6;
/// Upstream `groundStation.minimumElevation` \[rad\].
const MINIMUM_ELEVATION_RAD: f64 = 10.0 * D2R;

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
    /// Upstream `dataUnitLog`.
    storage: MessageLog<DataStorageStatusMsg>,
    /// Upstream `dataUnitLog2`.
    simple_storage: MessageLog<DataStorageStatusMsg>,
    /// Upstream `instLog`.
    instrument: MessageLog<DataNodeUsageMsg>,
    /// The transmitter's data node (upstream records none).
    transmitter: MessageLog<DataNodeUsageMsg>,
    /// Upstream `gsAccessLog`.
    access: MessageLog<AccessMsg>,
    /// Upstream `dataLog`.
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream `timeInitString = "2020 MAY 21 18:28:03 (UTC)"`.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2020, 5, 21, 18, 28, 3, 0)
}

fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let mu = earth.mu_m3ps2();
    let elements = ClassicElements {
        semi_major_axis: REQ_EARTH * 1e3 + 418e3,
        eccentricity: 0.00061,
        inclination_rad: 51.6418 * D2R,
        raan_rad: 119.2314 * D2R,
        arg_periapsis_rad: 337.8329 * D2R,
        true_anomaly_rad: 22.2753 * D2R,
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
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.spherical_harmonics(GGM03S_J2_ONLY, 2, true))
        .expect("one central gravity body");
    let mut ephemeris = UpdatedAtInit::new(earth.ephemeris(Body::Earth), epoch());

    let mut ground_station = GroundLocation::new(GroundLocationConfig {
        name: "BoulderGroundStation".to_string(),
        planet_radius_m: REQ_EARTH * 1e3,
        minimum_elevation_rad: MINIMUM_ELEVATION_RAD,
        maximum_range_m: Some(1e9),
        location: SiteLocation::Geodetic(Lla {
            latitude_rad: 40.009971 * D2R,
            longitude_rad: -105.243895 * D2R,
            altitude_m: 1624.0,
        }),
    });
    let station_index = ground_station.add_spacecraft_to_model();
    let station_access = ground_station.access_out_msgs[station_index].clone();

    let mut transmitter = SpaceToGroundTransmitter::new("transmitter");
    transmitter.node_baud_rate_bps = TRANSMITTER_BAUD_BPS;
    transmitter.model.packet_size_bits = PACKET_SIZE_BITS;
    let DataSystem {
        mut instruments,
        mut partitioned,
        mut simple,
    } = DataSystem::new(INSTRUMENT_BAUD_BPS, &transmitter.node_data_out_msg);

    let mut storage_log = MessageLog::<DataStorageStatusMsg>::default();
    let mut simple_storage_log = MessageLog::<DataStorageStatusMsg>::default();
    let mut instrument_log = MessageLog::<DataNodeUsageMsg>::default();
    let mut transmitter_log = MessageLog::<DataNodeUsageMsg>::default();
    let mut access_log = MessageLog::<AccessMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioGroundDownlink", config.write_csv);

    let mut simulation = Simulation::new(epoch(), false);
    let planet = ephemeris.module.output_planet_msg.clone();
    connect!(&simulation,
        &planet => spacecraft.grav_body_input_mut("earth").expect("the Earth is a gravity body"),
        &planet => &mut ground_station.planet_in_msg,
        &spacecraft.state_out => &mut ground_station.sc_state_in_msgs[station_index],
        &station_access => transmitter.add_access_msg_to_transmitter(),
        &partitioned.storage_unit_data_out_msg => transmitter.add_storage_unit_to_transmitter(),
        &partitioned.storage_unit_data_out_msg => &mut storage_log.input,
        &simple.storage_unit_data_out_msg => &mut simple_storage_log.input,
        &instruments[0].node_data_out_msg => &mut instrument_log.input,
        &transmitter.node_data_out_msg => &mut transmitter_log.input,
        &station_access => &mut access_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &partitioned.storage_unit_data_out_msg => recorder
            .add_source::<DataStorageStatusMsg>(storage_columns(PARTITIONS.len())),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_m", "r_BN_N_m")),
        ),
        &ground_station.current_ground_state_out_msg => recorder.add_source::<GroundStateMsg>(
            CsvSourceConfig::columns(vector_columns("position_lp_n_m", "r_LP_N_m")),
        ),
        &station_access => recorder.add_source::<AccessMsg>(CsvSourceConfig::columns([
            ("has_access", "hasAccess"),
            ("elevation_rad", "elevation_rad"),
            ("azimuth_rad", "azimuth_rad"),
            ("slant_range_m", "slantRange_m"),
        ])),
    );

    // Upstream's task order: the spacecraft (priority 1), then at the
    // default priority the ephemeris, the station, the instruments, the
    // transmitter, the storage units and the recorders. The ephemeris is
    // moved ahead of the spacecraft (see the module docs).
    let [instrument1, instrument2] = &mut instruments;
    schedule! { simulation,
        "spice" => &mut ephemeris, period=TASK_PERIOD_NANOS, priority=100;
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=90;
        "groundStation" => &mut ground_station, period=TASK_PERIOD_NANOS, priority=80;
        "instrument1" => instrument1, period=TASK_PERIOD_NANOS, priority=70;
        "instrument2" => instrument2, period=TASK_PERIOD_NANOS, priority=70;
        "transmitter" => &mut transmitter, period=TASK_PERIOD_NANOS, priority=60;
        "dataMonitor" => &mut partitioned, period=TASK_PERIOD_NANOS, priority=50;
        "dataMonitor2" => &mut simple, period=TASK_PERIOD_NANOS, priority=40;
        "dataUnitLog" => &mut storage_log, period=TASK_PERIOD_NANOS, priority=0;
        "dataUnitLog2" => &mut simple_storage_log, period=TASK_PERIOD_NANOS, priority=0;
        "instLog" => &mut instrument_log, period=TASK_PERIOD_NANOS, priority=0;
        "transmitterLog" => &mut transmitter_log, period=TASK_PERIOD_NANOS, priority=0;
        "gsAccessLog" => &mut access_log, period=TASK_PERIOD_NANOS, priority=0;
        "dataLog" => &mut state_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        storage: storage_log,
        simple_storage: simple_storage_log,
        instrument: instrument_log,
        transmitter: transmitter_log,
        access: access_log,
        state: state_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_ground_downlink [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let access_steps = results
            .access
            .values
            .iter()
            .filter(|access| access.has_access)
            .count();
        println!(
            "wrote {} ({} s of access to Boulder, {:.0} bit stored after 24 h)",
            results.csv_path.as_ref().expect("CSV written").display(),
            access_steps as u64 * TASK_PERIOD_NANOS / 1_000_000_000,
            results.storage.last().storage_level_bits,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it runs. These tests check what it demonstrates: Boulder
    //! sees the satellite above its 10 deg mask on a few passes a day, and
    //! data goes down only during those passes, a whole packet at a time.

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn access_follows_the_elevation_mask() {
        let results = quiet();
        let access = &results.access.values;
        assert_eq!(access.len() as u64, DURATION_NANOS / TASK_PERIOD_NANOS + 1);
        for sample in access {
            assert_eq!(
                sample.has_access,
                sample.elevation_rad > MINIMUM_ELEVATION_RAD
            );
            assert!(sample.slant_range_m > 400e3);
        }
        // Each pass is a run of access steps; an ISS-like orbit passes
        // over Boulder several times a day, each pass under ~10 minutes.
        let mut passes = Vec::new();
        let mut length = 0;
        for sample in access {
            if sample.has_access {
                length += 1;
            } else if length > 0 {
                passes.push(length);
                length = 0;
            }
        }
        assert!((2..=8).contains(&passes.len()), "passes {passes:?}");
        assert!(passes.iter().all(|steps| *steps <= 60), "passes {passes:?}");
        // The overhead geometry: at its highest a pass is well above the mask.
        let highest = access
            .iter()
            .map(|sample| sample.elevation_rad)
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(highest > 30.0 * D2R);
    }

    #[test]
    fn data_goes_down_only_with_access() {
        let results = quiet();
        let generated = 2.0 * INSTRUMENT_BAUD_BPS;
        let step_s = TASK_PERIOD_NANOS as f64 * 1e-9;
        let mut downlink_steps = 0;
        for (index, storage) in results.storage.values.iter().enumerate() {
            let sending = results.transmitter.values[index].baud_rate_bps;
            assert!(sending == 0.0 || sending == TRANSMITTER_BAUD_BPS);
            if sending != 0.0 {
                // The station runs before the transmitter in the same step.
                assert!(results.access.values[index].has_access);
                downlink_steps += 1;
            }
            assert_eq!(storage.current_net_baud_bps, generated + sending);
            assert_eq!(
                results.instrument.values[index].baud_rate_bps,
                INSTRUMENT_BAUD_BPS
            );
            if index > 0 {
                // No partition runs empty: the level integrates the net rate.
                let previous = &results.storage.values[index - 1];
                assert_eq!(
                    storage.storage_level_bits - previous.storage_level_bits,
                    storage.current_net_baud_bps * step_s
                );
            }
        }
        // Several 1e6 bit packets, of ceil(1e6 / 96 000) = 11 steps each,
        // went down; the last may be cut off by the end of a pass.
        assert!(downlink_steps >= 3 * 11, "{downlink_steps} downlink steps");
    }

    #[test]
    fn the_simple_unit_saturates() {
        let results = quiet();
        // 4800 bit/s fill the 1e5 bit unit within the first minute.
        let last = results.simple_storage.last();
        assert!(last.storage_level_bits <= data_system::SIMPLE_CAPACITY_BITS as f64);
        assert!(
            results
                .simple_storage
                .at(6 * TASK_PERIOD_NANOS)
                .storage_level_bits
                > 0.9 * data_system::SIMPLE_CAPACITY_BITS as f64
        );
        // The spacecraft stays on its ~418 km orbit.
        for state in &results.state.values {
            let altitude = state.position_m.norm() - REQ_EARTH * 1e3;
            assert!((380e3..460e3).contains(&altitude), "altitude {altitude}");
        }
    }
}
