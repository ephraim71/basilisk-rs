//! Changing the verbosity of a simulation's log messages: port of upstream
//! `examples/scenarioBskLog.py`.
//!
//! Upstream runs `cModuleTemplate` (its 0.5 s task, 1 s) and changes the
//! verbosity of `bskLog`, Basilisk's logging, either globally or for that one
//! module. basilisk-rs modules log through the [`log`] crate, so the Rust
//! counterpart is a [`log::Log`] implementation, [`ScenarioLogger`], with a
//! process-wide default level and per-module levels keyed by the log target
//! (the module name):
//!
//! | upstream (`bskLogging`) | here (`log` crate) |
//! |---|---|
//! | `_bskLog(logger, BSK_INFORMATION, ...)` in a module | `log::info!(target: <module name>, ...)` |
//! | `DEBUG`, `INFORMATION`, `WARNING`, `ERROR`, `SILENT` (0-4) | [`BskLogLevel`]: `Debug`, `Info`, `Warn`, `Error`, `Off` |
//! | `setDefaultLogLevel(level)`, `getDefaultLogLevel()` | [`ScenarioLogger::set_default_level`], [`ScenarioLogger::default_level`] |
//! | `BSKLogger()`, `setLevel`, `module.bskLogger = logger` | [`ScenarioLogger::set_module_level`] for the module's target |
//! | `printDefaultLogLevel()`, `printLogLevel()` | printed by [`run`] |
//!
//! The module, [`ScenarioModuleTemplate`], is upstream's `cModuleTemplate`
//! written in the scenario: each update adds its counter to the first
//! component of the input vector, and its reset and update log
//! `INFORMATION` messages. Upstream's `CModuleTemplateMsg` is the scenario's
//! [`ModuleTemplateMsg`].
//!
//! Upstream `run(case)` maps to `--case 0|1|2` (0: default verbosity, 1:
//! verbosity changed globally to `WARNING`, 2: the module's verbosity set to
//! `ERROR`); the script's own default is 2 and `--all` runs the cases 1 and
//! 2 of upstream's `test_scenarioBskLog.py`. Writes the module output to
//! `examples/output/scenarios/scenarioBskLog<case>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::collections::HashMap;
use std::sync::{OnceLock, RwLock};

use attitude::scenario_recorder;
use basilisk_rs::messages::{Input, Output};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::{CsvSourceConfig, TelemetryField, TelemetryMessage};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::{seconds, vector_columns};
use hifitime::Epoch;
use log::{Level, LevelFilter, Log, Metadata, Record};
use message_log::MessageLog;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// Upstream `testProcessRate = macros.sec2nano(0.5)`.
const TASK_PERIOD_NANOS: u64 = 500_000_000;
/// Upstream `ConfigureStopTime(macros.sec2nano(1.0))`.
const DURATION_NANOS: u64 = seconds(1);
const MODULE_NAME: &str = "cModuleTemplate";

/// Upstream's `bskLogging` levels, by their integer codes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum BskLogLevel {
    Debug = 0,
    Information = 1,
    Warning = 2,
    Error = 3,
    Silent = 4,
}

impl BskLogLevel {
    fn filter(self) -> LevelFilter {
        match self {
            Self::Debug => LevelFilter::Debug,
            Self::Information => LevelFilter::Info,
            Self::Warning => LevelFilter::Warn,
            Self::Error => LevelFilter::Error,
            Self::Silent => LevelFilter::Off,
        }
    }

    fn from_filter(filter: LevelFilter) -> Self {
        match filter {
            LevelFilter::Trace | LevelFilter::Debug => Self::Debug,
            LevelFilter::Info => Self::Information,
            LevelFilter::Warn => Self::Warning,
            LevelFilter::Error => Self::Error,
            LevelFilter::Off => Self::Silent,
        }
    }

    /// Upstream's level names (`printLogLevel`).
    fn name(self) -> &'static str {
        match self {
            Self::Debug => "DEBUG",
            Self::Information => "INFORMATION",
            Self::Warning => "WARNING",
            Self::Error => "ERROR",
            Self::Silent => "SILENT",
        }
    }
}

/// The scenario's logger: a default verbosity and per-module verbosities.
struct ScenarioLogger {
    default: RwLock<LevelFilter>,
    modules: RwLock<HashMap<String, LevelFilter>>,
    /// Messages written, by target (read by the tests).
    written: RwLock<HashMap<String, usize>>,
}

impl ScenarioLogger {
    /// The process's logger, installed on first use. Upstream's compiled-in
    /// default verbosity is `DEBUG`.
    fn get() -> &'static Self {
        static LOGGER: OnceLock<ScenarioLogger> = OnceLock::new();
        let mut installed = false;
        let logger = LOGGER.get_or_init(|| {
            installed = true;
            ScenarioLogger {
                default: RwLock::new(LevelFilter::Debug),
                modules: RwLock::new(HashMap::new()),
                written: RwLock::new(HashMap::new()),
            }
        });
        if installed {
            log::set_logger(logger).expect("no other logger is installed");
            log::set_max_level(LevelFilter::Trace);
        }
        logger
    }

    /// Upstream `setDefaultLogLevel`.
    fn set_default_level(&self, level: BskLogLevel) {
        *self.default.write().expect("unpoisoned") = level.filter();
    }

    /// Upstream `getDefaultLogLevel`.
    fn default_level(&self) -> BskLogLevel {
        BskLogLevel::from_filter(*self.default.read().expect("unpoisoned"))
    }

    /// Upstream `module.bskLogger = logger` with `logger.setLevel(level)`.
    fn set_module_level(&self, module: &str, level: BskLogLevel) {
        self.modules
            .write()
            .expect("unpoisoned")
            .insert(module.to_string(), level.filter());
    }

    /// Upstream `logger.getLogLevel()`: the module's level, else the default.
    fn module_level(&self, module: &str) -> BskLogLevel {
        let modules = self.modules.read().expect("unpoisoned");
        modules.get(module).map_or_else(
            || self.default_level(),
            |filter| BskLogLevel::from_filter(*filter),
        )
    }

    #[cfg_attr(not(test), allow(dead_code))]
    fn written(&self, target: &str) -> usize {
        self.written
            .read()
            .expect("unpoisoned")
            .get(target)
            .copied()
            .unwrap_or(0)
    }
}

impl Log for ScenarioLogger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= self.module_level(metadata.target()).filter()
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        let level = match record.level() {
            Level::Error => "BSK_ERROR",
            Level::Warn => "BSK_WARNING",
            Level::Info => "BSK_INFORMATION",
            Level::Debug | Level::Trace => "BSK_DEBUG",
        };
        println!("{level}: {}", record.args());
        *self
            .written
            .write()
            .expect("unpoisoned")
            .entry(record.target().to_string())
            .or_default() += 1;
    }

    fn flush(&self) {}
}

/// Upstream `CModuleTemplateMsgPayload`.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct ModuleTemplateMsg {
    /// Upstream `dataVector`.
    data_vector: Vector3<f64>,
}

impl TelemetryMessage for ModuleTemplateMsg {
    fn flatten(&self) -> Vec<TelemetryField> {
        ["x", "y", "z"]
            .iter()
            .zip(self.data_vector.iter())
            .map(|(axis, value)| TelemetryField {
                path: format!("data_vector.{axis}"),
                value: *value,
            })
            .collect()
    }
}

/// Upstream `cModuleTemplate`, written in the scenario.
struct ScenarioModuleTemplate {
    name: String,
    /// Upstream `dataInMsg` (optional).
    data_in_msg: Input<ModuleTemplateMsg>,
    /// Upstream `dataOutMsg`.
    data_out_msg: Output<ModuleTemplateMsg>,
    /// Upstream `updateCounter`.
    update_counter: f64,
}

impl Module for ScenarioModuleTemplate {
    /// Upstream `Reset`: zero the counter and the output.
    fn init(&mut self) {
        self.update_counter = 0.0;
        log::info!(
            target: &self.name,
            "Variable updateCounter set to {:.6} in reset.",
            self.update_counter
        );
        self.data_out_msg.write(ModuleTemplateMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let input = if self.data_in_msg.is_connected() {
            self.data_in_msg.read().data_vector
        } else {
            Vector3::zeros()
        };
        self.update_counter += 1.0;
        let mut output = ModuleTemplateMsg { data_vector: input };
        output.data_vector[0] += self.update_counter;
        self.data_out_msg.write(output);
        log::info!(
            target: &self.name,
            "{} ran Update at {:.6}s",
            self.name,
            context.current_sim_nanos as f64 * 1e-9
        );
    }
}

/// Upstream `run(case)`'s returned level, and the module output.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    level: BskLogLevel,
    output: MessageLog<ModuleTemplateMsg>,
}

fn run(case: u8, write_csv: bool) -> Results {
    let logger = ScenarioLogger::get();
    if case == 1 {
        // Upstream sets the global verbosity before creating the simulation.
        logger.set_default_level(BskLogLevel::Warning);
    }

    // Upstream's input message, `dataVector = [1.0, 1.0, 0.7]`.
    let data_msg = Output::new(ModuleTemplateMsg {
        data_vector: Vector3::new(1.0, 1.0, 0.7),
    });
    let mut module = ScenarioModuleTemplate {
        name: MODULE_NAME.to_string(),
        data_in_msg: Input::default(),
        data_out_msg: Output::default(),
        update_counter: 1.0,
    };
    let mut output_log = MessageLog::<ModuleTemplateMsg>::default();
    let (mut recorder, _) = scenario_recorder(&format!("scenarioBskLog{case}"), write_csv);

    let level = match case {
        0 => {
            println!("BSK default log level: {}", logger.default_level().name());
            logger.default_level()
        }
        1 => {
            println!("BSK default log level: {}", logger.default_level().name());
            println!("The verbosity was globally changed.");
            logger.default_level()
        }
        _ => {
            logger.set_module_level(MODULE_NAME, BskLogLevel::Error);
            println!("The verbosity is only changed for this module.");
            println!("BSK log level: {}", logger.module_level(MODULE_NAME).name());
            logger.module_level(MODULE_NAME)
        }
    };

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &data_msg => &mut module.data_in_msg,
        &module.data_out_msg => &mut output_log.input,
        &module.data_out_msg => recorder.add_source::<ModuleTemplateMsg>(CsvSourceConfig::columns(
            vector_columns("data_vector", "dataVector"),
        )),
    );
    schedule! { simulation,
        "cModuleTemplate" => &mut module, period=TASK_PERIOD_NANOS, priority=0;
        "output_log" => &mut output_log, period=TASK_PERIOD_NANOS, priority=-1;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, TASK_PERIOD_NANOS, -1);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);

    Results {
        level,
        output: output_log,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_bsk_log [--case 0|1|2] [--all]");
    let cases = if cli.switch("all") {
        vec![1, 2]
    } else {
        // Upstream's `__main__` runs `run(2)`.
        vec![cli.value("case", 2_u8)]
    };
    cli.finish();
    for case in cases {
        let results = run(case, true);
        println!(
            "case {case}: log level {} ({}); wrote examples/output/scenarios/scenarioBskLog{case}.csv",
            results.level as u8,
            results.level.name()
        );
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use super::*;

    /// The logger is process-wide; the tests take turns changing it.
    static LOGGER_LOCK: Mutex<()> = Mutex::new(());

    /// Upstream `test_scenarioBskLog.py`: case 1 returns `WARNING` (2) and
    /// case 2 `ERROR` (3). Upstream leaves case 0 out because the default
    /// depends on what ran before; here the test restores it first.
    #[test]
    fn the_returned_verbosity_matches_upstream_truth() {
        let _guard = LOGGER_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let logger = ScenarioLogger::get();
        logger.set_default_level(BskLogLevel::Debug);
        assert_eq!(run(0, false).level as u8, 0);
        assert_eq!(run(1, false).level as u8, 2);
        logger.set_default_level(BskLogLevel::Debug);
        assert_eq!(run(2, false).level as u8, 3);
    }

    /// The module's `INFORMATION` messages pass at the default `DEBUG`
    /// verbosity and are filtered by the global `WARNING` and the module's
    /// `ERROR`; its output is the template's sum.
    #[test]
    fn the_verbosity_filters_the_module_messages() {
        let _guard = LOGGER_LOCK
            .lock()
            .unwrap_or_else(|poison| poison.into_inner());
        let logger = ScenarioLogger::get();
        logger.modules.write().unwrap().clear();
        let mut written = Vec::new();
        for (case, default) in [
            (0, BskLogLevel::Debug),
            (1, BskLogLevel::Debug),
            (2, BskLogLevel::Debug),
        ] {
            logger.set_default_level(default);
            let before = logger.written(MODULE_NAME);
            let results = run(case, false);
            written.push(logger.written(MODULE_NAME) - before);
            // Reset at t = 0, then 3 updates (0, 0.5, 1 s): counter 1, 2, 3.
            assert_eq!(results.output.values.len(), 3);
            for (step, value) in results.output.values.iter().enumerate() {
                let expected = Vector3::new(1.0 + (step + 1) as f64, 1.0, 0.7);
                assert_eq!(value.data_vector, expected);
            }
        }
        // One reset message and three update messages at DEBUG.
        assert_eq!(written, vec![4, 0, 0]);
        logger.set_default_level(BskLogLevel::Debug);
        logger.modules.write().unwrap().clear();
    }
}
