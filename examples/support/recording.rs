//! Recording helpers for the orbit scenarios.
//!
//! * [`sampling_time`]: upstream `simHelpers.samplingTime`.
//! * [`scenario_csv`], [`state_columns`] and [`planet_columns`]: a
//!   scenario-format CSV recorder and the spacecraft- and planet-state
//!   columns the orbit scenarios write.
//!
//! The in-memory recorder that `run()` returns for the upstream-truth tests
//! is `message_log::MessageLog`.

#![allow(dead_code)]

use basilisk_rs::telemetry::{CsvRecorder, CsvSourceConfig};

use super::common::{scenario_recorder, vector_columns};

#[path = "message_log.rs"]
mod message_log;

/// The former name of [`MessageLog`](message_log::MessageLog), kept only for
/// `scenario_integrators` and `scenario_variable_time_step_integrators`;
/// remove it once they use `message_log::MessageLog` directly.
#[allow(unused_imports)]
pub use message_log::MessageLog as History;

/// Upstream `simHelpers.samplingTime`: the multiple of `base_step_nanos`
/// closest below `duration / (points - 1)`, at least 1 ns.
pub fn sampling_time(duration_nanos: u64, base_step_nanos: u64, points: u64) -> u64 {
    let steps = (duration_nanos as f64 / base_step_nanos as f64 / (points - 1) as f64).floor();
    ((steps * base_step_nanos as f64) as u64).max(1)
}

/// A scenario-format CSV recorder writing
/// `examples/output/scenarios/<file_stem>.csv`.
pub fn scenario_csv(file_stem: &str) -> CsvRecorder {
    scenario_recorder(file_stem, true).0
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
