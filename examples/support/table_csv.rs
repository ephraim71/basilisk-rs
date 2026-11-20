//! Summary tables for the scenarios whose result is a sweep over runs (a
//! convergence study, a parameter sensitivity) rather than one recorded
//! history: the values upstream collects in Python lists and plots, written
//! as `examples/output/scenarios/<file_stem>.csv` in the scenario CSV number
//! format (`{:.18e}`).

#![allow(dead_code)]

use std::fmt::Write as _;
use std::path::PathBuf;

use super::common::scenario_output_path;

/// Writes `rows` under `headers` and returns the path.
pub fn write_table(file_stem: &str, headers: &[String], rows: &[Vec<f64>]) -> PathBuf {
    let path = scenario_output_path(&format!("{file_stem}.csv"));
    let mut text = headers.join(",");
    text.push('\n');
    for row in rows {
        assert_eq!(row.len(), headers.len(), "table row length");
        let fields: Vec<String> = row.iter().map(|value| format!("{value:.18e}")).collect();
        let _ = writeln!(text, "{}", fields.join(","));
    }
    std::fs::write(&path, text)
        .unwrap_or_else(|error| panic!("cannot write {}: {error}", path.display()));
    path
}
