//! A battery charged by a body-fixed solar panel and drained by a constant
//! load: port of upstream `examples/scenarioPowerDemo.py`.
//!
//! The tutorial spacecraft of [`resource_demo`] (400 km equatorial Earth
//! orbit, tumbling slowly, one 1 s task, one orbit) carries a
//! `simpleSolarPanel` (normal `+x` body, 0.06 m^2, 20 % efficient) shadowed
//! by an `eclipse` module for the Earth, a `simplePowerSink` drawing 3 W,
//! and a `simpleBattery` of 10 W h (36 kJ) that starts full and sums both
//! nodes.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case.
//!
//! Mapping to basilisk-rs: upstream's `eclipse` reads the Earth's and Sun's
//! SPICE states; here, with the Earth at the inertial origin, it reads the
//! Sun's [`SunEphemerisMsg`] and the Earth's equatorial radius.
//!
//! Writes `examples/output/scenarios/scenarioPowerDemo.csv` with upstream's
//! three recorders every step: `panelPower_W` (`spLog.netPower`),
//! `sinkPower_W` (`psLog.netPower`), `storageLevel_Ws` and
//! `currentNetPower_W` (`pmLog`), plus the panel's `illuminationFactor`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
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
use basilisk_rs::environment::eclipse::{Eclipse, EclipseConfig};
use basilisk_rs::messages::{EclipseMsg, PowerNodeUsageMsg, PowerStorageStatusMsg};
use basilisk_rs::power::simple_battery::{SimpleBattery, SimpleBatteryConfig};
use basilisk_rs::power::simple_power_sink::{SimplePowerSink, SimplePowerSinkModel};
use basilisk_rs::power::simple_solar_panel::{SimpleSolarPanel, SimpleSolarPanelModel};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::REQ_EARTH;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use message_log::MessageLog;
use nalgebra::Vector3;
use resource_demo::{ResourceDemo, TASK_PERIOD_NANOS, epoch};

/// Upstream `powerMonitor.storageCapacity` and `storedCharge_Init` \[J\]:
/// 10 W h.
const BATTERY_CAPACITY_J: f64 = 10.0 * 3600.0;
/// Upstream `powerSink.nodePowerOut` \[W\].
const SINK_POWER_W: f64 = -3.0;
/// Upstream `setPanelParameters([1, 0, 0], 0.2 * 0.3, 0.20)`: normal (body),
/// area \[m^2\] and efficiency.
const PANEL_AREA_M2: f64 = 0.2 * 0.3;
const PANEL_EFFICIENCY: f64 = 0.20;

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioPowerDemo.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// The recorded histories, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `spLog`.
    panel: MessageLog<PowerNodeUsageMsg>,
    /// Upstream `psLog`.
    sink: MessageLog<PowerNodeUsageMsg>,
    /// Upstream `pmLog`.
    battery: MessageLog<PowerStorageStatusMsg>,
    /// The panel's illumination factor.
    eclipse: MessageLog<EclipseMsg>,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let ResourceDemo {
        mut spacecraft,
        mut sun,
        period_s,
    } = ResourceDemo::new();
    let duration_nanos = orbit_tools::sec2nano(period_s);

    let mut eclipse = Eclipse::new(EclipseConfig {
        name: "eclipse".to_string(),
        occulting_body_radius_m: REQ_EARTH * 1e3,
    });
    let mut panel = SimpleSolarPanel::new(
        "solarPanel",
        SimpleSolarPanelModel::new(Vector3::x(), PANEL_AREA_M2, PANEL_EFFICIENCY),
    );
    let mut sink = SimplePowerSink::new(
        "powerSink2",
        SimplePowerSinkModel {
            node_power_out_w: SINK_POWER_W,
        },
    );
    let mut battery = SimpleBattery::new(SimpleBatteryConfig {
        name: "powerMonitor".to_string(),
        storage_capacity_j: BATTERY_CAPACITY_J,
        stored_charge_init_j: BATTERY_CAPACITY_J,
    });

    let mut panel_log = MessageLog::<PowerNodeUsageMsg>::default();
    let mut sink_log = MessageLog::<PowerNodeUsageMsg>::default();
    let mut battery_log = MessageLog::<PowerStorageStatusMsg>::default();
    let mut eclipse_log = MessageLog::<EclipseMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioPowerDemo", config.write_csv);

    let mut simulation = Simulation::new(epoch(), false);
    let state = spacecraft.state_out.clone();
    connect!(&simulation,
        &state => &mut eclipse.input_state_msg,
        &sun.module.output_sun_msg => &mut eclipse.input_sun_msg,
        &state => &mut panel.model.state_in_msg,
        &eclipse.output_eclipse_msg => &mut panel.model.sun_eclipse_in_msg,
        &sun.module.output_sun_msg => &mut panel.model.sun_in_msg,
        &panel.node_power_out_msg => battery.add_power_node_to_model(),
        &sink.node_power_out_msg => battery.add_power_node_to_model(),
        &panel.node_power_out_msg => &mut panel_log.input,
        &sink.node_power_out_msg => &mut sink_log.input,
        &battery.bat_power_out_msg => &mut battery_log.input,
        &eclipse.output_eclipse_msg => &mut eclipse_log.input,
        &panel.node_power_out_msg => recorder.add_source::<PowerNodeUsageMsg>(
            CsvSourceConfig::columns([("net_power_w", "panelPower_W")]),
        ),
        &sink.node_power_out_msg => recorder.add_source::<PowerNodeUsageMsg>(
            CsvSourceConfig::columns([("net_power_w", "sinkPower_W")]),
        ),
        &battery.bat_power_out_msg => recorder.add_source::<PowerStorageStatusMsg>(
            CsvSourceConfig::columns([
                ("storage_level_j", "storageLevel_Ws"),
                ("current_net_power_w", "currentNetPower_W"),
            ]),
        ),
        &eclipse.output_eclipse_msg => recorder.add_source::<EclipseMsg>(
            CsvSourceConfig::columns([("illumination_factor", "illuminationFactor")]),
        ),
    );

    // Upstream's task order (all at the default priority): the ephemeris,
    // the spacecraft, the eclipse, the panel, the sink, the battery, then
    // the recorders.
    schedule! { simulation,
        "spice" => &mut sun, period=TASK_PERIOD_NANOS, priority=100;
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=90;
        "eclipse" => &mut eclipse, period=TASK_PERIOD_NANOS, priority=80;
        "solarPanel" => &mut panel, period=TASK_PERIOD_NANOS, priority=70;
        "powerSink2" => &mut sink, period=TASK_PERIOD_NANOS, priority=60;
        "powerMonitor" => &mut battery, period=TASK_PERIOD_NANOS, priority=50;
        "spLog" => &mut panel_log, period=TASK_PERIOD_NANOS, priority=0;
        "psLog" => &mut sink_log, period=TASK_PERIOD_NANOS, priority=0;
        "pmLog" => &mut battery_log, period=TASK_PERIOD_NANOS, priority=0;
        "eclipseLog" => &mut eclipse_log, period=TASK_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, 0);
    }
    simulation.run_for(duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        panel: panel_log,
        sink: sink_log,
        battery: battery_log,
        eclipse: eclipse_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_power_demo [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        println!(
            "wrote {} (battery {:.1} W h after one orbit)",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.battery.last().storage_level_j / 3600.0,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioPowerDemo.py` only checks that the scenario
    //! runs. These tests check the power balance the tutorial shows: the
    //! battery integrates panel supply and sink draw, and the panel goes
    //! dark in the Earth's shadow.

    use basilisk_rs::utilities::astro_constants::SOLAR_FLUX_EARTH;

    use super::*;

    fn quiet() -> Results {
        run(Config { write_csv: false })
    }

    #[test]
    fn the_battery_integrates_the_net_power() {
        let results = quiet();
        let battery = &results.battery.values;
        assert_eq!(battery[0].storage_level_j, BATTERY_CAPACITY_J);
        for (index, status) in battery.iter().enumerate().skip(1) {
            let panel = results.panel.values[index].net_power_w;
            let sink = results.sink.values[index].net_power_w;
            assert_eq!(sink, SINK_POWER_W);
            assert_eq!(status.current_net_power_w, panel + sink);
            // One 1 s step of the net power, kept within the capacity.
            let expected = (battery[index - 1].storage_level_j + status.current_net_power_w)
                .clamp(0.0, BATTERY_CAPACITY_J);
            assert!((status.storage_level_j - expected).abs() < 1e-9);
        }
    }

    #[test]
    fn the_panel_supplies_only_in_sunlight() {
        let results = quiet();
        // At 1 AU the panel can give at most flux x area x efficiency.
        let ceiling = 1.02 * SOLAR_FLUX_EARTH * PANEL_AREA_M2 * PANEL_EFFICIENCY;
        let mut shadowed = 0;
        for (panel, eclipse) in results.panel.values.iter().zip(&results.eclipse.values) {
            assert!((0.0..ceiling).contains(&panel.net_power_w));
            if eclipse.illumination_factor == 0.0 {
                shadowed += 1;
                assert_eq!(panel.net_power_w, 0.0);
            }
        }
        // A 400 km orbit spends about a third of its period in the umbra.
        let fraction = shadowed as f64 / results.panel.values.len() as f64;
        assert!((0.3..0.45).contains(&fraction), "umbra fraction {fraction}");
        // The panel normal sweeps a 54.7 deg cone about the torque-free spin
        // axis `[1, -1, 1]`, which points about 140 deg from the Sun: the
        // panel catches the Sun only obliquely, at most about 1.4 W, before
        // the eclipse and again after it.
        let peak = results
            .panel
            .values
            .iter()
            .map(|panel| panel.net_power_w)
            .fold(0.0, f64::max);
        assert!((1.0..2.0).contains(&peak), "peak panel power {peak} W");
    }

    #[test]
    fn the_load_drains_the_battery_in_the_shadow() {
        let results = quiet();
        let battery = &results.battery.values;
        let lowest = battery
            .iter()
            .map(|status| status.storage_level_j)
            .fold(f64::INFINITY, f64::min);
        // The 3 W load alone, over the umbra.
        let shadowed = results
            .eclipse
            .values
            .iter()
            .filter(|eclipse| eclipse.illumination_factor == 0.0)
            .count() as f64;
        assert!(lowest <= BATTERY_CAPACITY_J + SINK_POWER_W * shadowed + 1e-6);
        assert!(lowest > 0.0);
    }
}
