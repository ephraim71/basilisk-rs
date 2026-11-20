//! Recording helpers for the orbit scenarios.
//!
//! * [`History`]: an in-memory message recorder, upstream
//!   `msg.recorder(samplingTime)`, which `run()` returns for the upstream-truth
//!   tests; the CSV files are still written by
//!   [`basilisk_rs::telemetry::CsvRecorder`].
//! * [`sampling_time`]: upstream `simHelpers.samplingTime`.
//! * [`scenario_csv`] and [`state_columns`]: a scenario-format CSV recorder and
//!   the spacecraft-state columns every orbit scenario writes.

#![allow(dead_code)]

use basilisk_rs::messages::{Input, SimulationMessage};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::{Module, SimulationContext};

use super::common::{scenario_output_path, vector_columns};

/// Records every update's input message with its simulation time. Schedule it
/// at the sampling period, after the module that writes the message, as
/// upstream adds a recorder to the task after the modules it records.
#[derive(Debug, Default)]
pub struct History<T> {
    pub input: Input<T>,
    samples: Vec<(u64, T)>,
}

impl<T: SimulationMessage> History<T> {
    pub fn new() -> Self {
        Self {
            input: Input::default(),
            samples: Vec::new(),
        }
    }

    /// `(time_ns, message)` in recording order: upstream `times()` and the
    /// recorded payloads.
    pub fn samples(&self) -> &[(u64, T)] {
        &self.samples
    }

    pub fn times_nanos(&self) -> Vec<u64> {
        self.samples.iter().map(|(time, _)| *time).collect()
    }

    pub fn messages(&self) -> impl Iterator<Item = &T> {
        self.samples.iter().map(|(_, message)| message)
    }

    pub fn last(&self) -> Option<&T> {
        self.samples.last().map(|(_, message)| message)
    }

    pub fn into_samples(self) -> Vec<(u64, T)> {
        self.samples
    }
}

impl<T: SimulationMessage> Module for History<T> {
    fn init(&mut self) {}

    fn update(&mut self, context: &SimulationContext) {
        self.samples
            .push((context.current_sim_nanos, self.input.read()));
    }
}

/// Upstream `simHelpers.samplingTime`: the multiple of `base_step_nanos`
/// closest below `duration / (points - 1)`, at least 1 ns.
pub fn sampling_time(duration_nanos: u64, base_step_nanos: u64, points: u64) -> u64 {
    let steps = (duration_nanos as f64 / base_step_nanos as f64 / (points - 1) as f64).floor();
    ((steps * base_step_nanos as f64) as u64).max(1)
}

/// A scenario-format CSV recorder writing
/// `examples/output/scenarios/<file_stem>.csv`.
pub fn scenario_csv(file_stem: &str) -> CsvRecorder {
    CsvRecorder::new(CsvRecorderConfig {
        topic: file_stem.to_string(),
        output_path: scenario_output_path(&format!("{file_stem}.csv")),
    })
    .with_format(CsvFormat::Scenario)
}

/// `r_BN_N_m_i` and `v_BN_N_m_per_s_i` from a spacecraft state message, the
/// columns of the existing basic-orbit CSV.
pub fn state_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", "r_BN_N_m")
            .into_iter()
            .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
    )
}

/// `<prefix>_r_N_m_i` and `<prefix>_v_N_m_per_s_i` from a planet state message.
pub fn planet_columns(prefix: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_inertial_m", &format!("{prefix}_r_N_m"))
            .into_iter()
            .chain(vector_columns(
                "velocity_inertial_mps",
                &format!("{prefix}_v_N_m_per_s"),
            )),
    )
}

#[cfg(test)]
mod tests {
    use super::sampling_time;

    #[test]
    fn sampling_time_matches_upstream_formula() {
        // scenarioBasicOrbit LEO: 0.75 P = 4371.4 s, 10 s steps, 100 points.
        assert_eq!(
            sampling_time(4_371_400_000_000, 10_000_000_000, 100),
            40_000_000_000
        );
        assert_eq!(sampling_time(10, 1_000_000_000, 100), 1);
    }
}
