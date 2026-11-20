//! Onboard data generation, storage and downlink: port of upstream
//! `examples/scenarioDataDemo.py`.
//!
//! The tutorial spacecraft of [`resource_demo`] (400 km equatorial Earth
//! orbit, one 1 s task, one orbit) carries two `simpleInstrument`s
//! (1200 baud each, named `Instrument 1` and `Instrument 2`) and a
//! `simpleTransmitter` (-16 000 baud, -1e6 bit packets). A
//! `partitionedStorageUnit` (8e9 bit, one partition per instrument) takes all
//! three data nodes, and the transmitter downlinks from it, a packet at a
//! time from its fullest partition once that holds a whole packet. A
//! `simpleStorageUnit` (1e5 bit, one partition) takes the same three nodes.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case. Upstream's `transmitter.numBuffers` is never read
//! upstream and is not ported (see `simple_transmitter`).
//!
//! Writes `examples/output/scenarios/scenarioDataDemo.csv` with upstream's
//! `dataMonLog` of the partitioned unit: `storageLevel_bits`,
//! `currentNetBaud_bps` and `storedData_bits_<i>` per partition, every step.

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
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/resource_demo.rs"]
mod resource_demo;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::messages::DataStorageStatusMsg;
use basilisk_rs::onboard_data_handling::simple_transmitter::SimpleTransmitter;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use data_system::{DataSystem, PARTITIONS, storage_columns};
use message_log::MessageLog;
use resource_demo::{ResourceDemo, TASK_PERIOD_NANOS, epoch};

/// Upstream `instrument.nodeBaudRate` of each instrument \[bit/s\].
const INSTRUMENT_BAUD_BPS: f64 = 1200.0;
/// Upstream `transmitter.nodeBaudRate` \[bit/s\].
const TRANSMITTER_BAUD_BPS: f64 = -16_000.0;
/// Upstream `transmitter.packetSize` \[bit\].
const PACKET_SIZE_BITS: f64 = -1e6;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioDataDemo.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `dataMonLog`: the partitioned storage unit.
    partitioned: MessageLog<DataStorageStatusMsg>,
    /// The simple storage unit (upstream records none).
    simple: MessageLog<DataStorageStatusMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let ResourceDemo {
        mut spacecraft,
        mut sun,
        period_s,
    } = ResourceDemo::new();
    let duration_nanos = orbit_tools::sec2nano(period_s);

    let mut transmitter = SimpleTransmitter::new("transmitter");
    transmitter.node_baud_rate_bps = TRANSMITTER_BAUD_BPS;
    transmitter.model.packet_size_bits = PACKET_SIZE_BITS;
    let DataSystem {
        mut instruments,
        mut partitioned,
        mut simple,
    } = DataSystem::new(INSTRUMENT_BAUD_BPS, &transmitter.node_data_out_msg);

    let mut partitioned_log = MessageLog::<DataStorageStatusMsg>::default();
    let mut simple_log = MessageLog::<DataStorageStatusMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioDataDemo", config.write_csv);

    let mut simulation = Simulation::new(epoch(), false);
    connect!(&simulation,
        &partitioned.storage_unit_data_out_msg => transmitter.add_storage_unit_to_transmitter(),
        &partitioned.storage_unit_data_out_msg => &mut partitioned_log.input,
        &simple.storage_unit_data_out_msg => &mut simple_log.input,
        &partitioned.storage_unit_data_out_msg => recorder
            .add_source::<DataStorageStatusMsg>(storage_columns(PARTITIONS.len())),
    );

    // Upstream's task order (all at the default priority): the ephemeris,
    // the spacecraft, the instruments, the transmitter, the two storage
    // units, then the recorder.
    let [instrument1, instrument2] = &mut instruments;
    schedule! { simulation,
        "spice" => &mut sun, period=TASK_PERIOD_NANOS, priority=100;
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=90;
        "instrument1" => instrument1, period=TASK_PERIOD_NANOS, priority=80;
        "instrument2" => instrument2, period=TASK_PERIOD_NANOS, priority=80;
        "transmitter" => &mut transmitter, period=TASK_PERIOD_NANOS, priority=70;
        "dataMonitor" => &mut partitioned, period=TASK_PERIOD_NANOS, priority=60;
        "dataMonitor2" => &mut simple, period=TASK_PERIOD_NANOS, priority=50;
        "dataMonLog" => &mut partitioned_log, period=TASK_PERIOD_NANOS, priority=0;
        "dataMonLog2" => &mut simple_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        partitioned: partitioned_log,
        simple: simple_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_data_demo [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        let last = results.partitioned.last();
        println!(
            "wrote {} (stored {:.0} bit after one orbit: {:?})",
            results.csv_path.as_ref().expect("CSV written").display(),
            last.storage_level_bits,
            last.stored_data_bits,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioDataDemo.py` only checks that the scenario
    //! runs. These tests check the data accounting the tutorial shows: the
    //! instruments fill both partitions at 1200 bit/s each, and the
    //! transmitter drains one whole 1e6 bit packet at a time.

    use super::*;
    use data_system::{PARTITIONED_CAPACITY_BITS, SIMPLE_CAPACITY_BITS};

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    const GENERATED_BPS: f64 = 2.0 * INSTRUMENT_BAUD_BPS;

    #[test]
    fn the_partitions_hold_the_stored_data() {
        let results = quiet();
        let log = &results.partitioned;
        // One sample per 1 s step over one orbit, t = 0 included.
        let period_s = ResourceDemo::new().period_s;
        assert_eq!(log.values.len(), period_s as usize + 1);
        for status in &log.values {
            assert_eq!(status.stored_data_name, PARTITIONS);
            let sum: f64 = status.stored_data_bits.iter().sum();
            assert_eq!(status.storage_level_bits, sum);
            assert!(status.stored_data_bits.iter().all(|bits| *bits >= 0.0));
            assert!(status.storage_level_bits <= PARTITIONED_CAPACITY_BITS as f64);
        }
    }

    #[test]
    fn storage_integrates_generation_minus_downlink() {
        let results = quiet();
        let log = &results.partitioned;
        let downlink_net = GENERATED_BPS + TRANSMITTER_BAUD_BPS;
        let mut downlink_steps = 0;
        for (previous, current) in log.values.iter().zip(&log.values[1..]) {
            // Generating only, or generating while a packet goes down.
            let net = current.current_net_baud_bps;
            assert!(net == GENERATED_BPS || net == downlink_net, "net {net}");
            downlink_steps += usize::from(net == downlink_net);
            assert_eq!(
                current.storage_level_bits - previous.storage_level_bits,
                net
            );
        }
        // Each packet takes ceil(1e6 / 16 000) = 63 steps, and at least one
        // went down during the orbit.
        assert!(downlink_steps > 0);
        assert_eq!(downlink_steps % 63, 0, "{downlink_steps} downlink steps");
        let steps = (log.values.len() - 1) as f64;
        assert_eq!(
            log.last().storage_level_bits,
            GENERATED_BPS * steps + TRANSMITTER_BAUD_BPS * downlink_steps as f64
        );
    }

    #[test]
    fn downlink_waits_for_a_whole_packet() {
        let results = quiet();
        let log = &results.partitioned;
        let first_downlink = log
            .values
            .iter()
            .position(|status| status.current_net_baud_bps < 0.0)
            .expect("a packet goes down during the orbit");
        // The transmitter reads the previous step's partitions: the fullest
        // (the first of equals, `Instrument 1`) holds a packet one step
        // before the first downlink.
        let before = &log.values[first_downlink - 1];
        assert!(before.stored_data_bits[0] >= -PACKET_SIZE_BITS);
        let earlier = &log.values[first_downlink - 2];
        assert!(earlier.stored_data_bits[0] < -PACKET_SIZE_BITS);
    }

    #[test]
    fn the_simple_unit_stays_within_its_capacity() {
        let results = quiet();
        let simple = &results.simple;
        assert!(
            simple
                .values
                .iter()
                .all(|status| status.storage_level_bits <= SIMPLE_CAPACITY_BITS as f64)
        );
        // The three nodes fill its single partition in under a minute; then
        // it refuses each instrument's step whose data would overflow it.
        let full = simple.at(60 * TASK_PERIOD_NANOS).storage_level_bits;
        assert!(
            full > SIMPLE_CAPACITY_BITS as f64 - INSTRUMENT_BAUD_BPS,
            "{full}"
        );
        assert!(
            simple
                .values
                .iter()
                .all(|status| status.stored_data_bits.len() == 1)
        );
    }
}
