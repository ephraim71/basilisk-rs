//! Port of upstream `examples/scenarioDragSensitivity.py`: the
//! differential-drag rendezvous of `scenario_drag_rendezvous` (upstream
//! `scenarioDragRendezvous.drag_simulator`, here
//! `support/drag_rendezvous.rs`), swept over the deputy's initial
//! true-anomaly lag and the atmosphere's density, to show how the static
//! LQR gain copes with a mis-modeled density.
//!
//! `run(nu_offset_count, density_count)` (upstream `run(show_plots,
//! nuOffsetNum, densityNum, rerunSims=True)`) runs
//! `nu = arange(0.001, 1, 0.999 / nu_offset_count)` deg against
//! `logspace(-1, 0.4, density_count)` density multipliers, every
//! combination, with no altitude offset. Upstream pickles the results and
//! sweeps them in `multiprocessing`; here the runs are sequential and
//! `scenarioDragSensitivity.csv` holds upstream's `results_to_ranges_and_plot`
//! table: per run the offset, the multiplier, and the Hill-frame range and
//! speed at the second sample and at the end. Every fifth run's Hill
//! trajectory goes to `scenarioDragSensitivityTrajectories.csv` (`run`,
//! `time_s`, `r_DC_H_m_0`, `r_DC_H_m_1`), the trajectories upstream plots.
//!
//! The default is upstream's `__main__`, 15 x 20 runs; `--all` runs
//! `test_scenarioDragSensitivity.py`'s 5 x 10. The upstream test only runs
//! the sweep; `tests` checks what the sweep shows: the static gain closes
//! the gap where the density is near its design value and degrades away
//! from it.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/drag_rendezvous.rs"]
mod drag_rendezvous;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/table_csv.rs"]
mod table_csv;

use cli::Cli;
use drag_rendezvous::{RendezvousConfig, drag_simulator};

const USAGE: &str = "\
scenario_drag_sensitivity [--nu-offsets 15] [--densities 20]
scenario_drag_sensitivity --all     run the upstream test sweep (5 x 10)";

/// Upstream `nu_offsets = np.arange(0.001, 1, (1 - 0.001) / count)` (deg).
pub fn nu_offsets_deg(count: usize) -> Vec<f64> {
    let step = (1.0 - 0.001) / count as f64;
    let length = ((1.0 - 0.001) / step).ceil() as usize;
    (0..length).map(|i| 0.001 + i as f64 * step).collect()
}

/// Upstream `density_multipliers = np.logspace(-1, 0.4, num=count)`.
pub fn density_multipliers(count: usize) -> Vec<f64> {
    let step = (0.4 - -1.0) / (count - 1) as f64;
    (0..count)
        .map(|i| {
            let exponent = if i == count - 1 {
                0.4
            } else {
                -1.0 + i as f64 * step
            };
            10.0_f64.powf(exponent)
        })
        .collect()
}

/// One row of upstream's `results_to_ranges_and_plot`.
#[derive(Clone, Copy, Debug)]
pub struct SweepResult {
    pub nu_offset_deg: f64,
    pub density_multiplier: f64,
    /// `init_relpos`, `init_relvel`: at the second sample.
    pub initial_range_m: f64,
    pub initial_speed_mps: f64,
    /// `pos_errs`, `vel_errs`: at the end.
    pub final_range_m: f64,
    pub final_speed_mps: f64,
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub trajectories_path: std::path::PathBuf,
    /// In upstream's `meshgrid(...).ravel()` order: offsets outer,
    /// multipliers inner.
    pub sweep: Vec<SweepResult>,
}

pub fn run(nu_offset_count: usize, density_count: usize) -> Results {
    let mut sweep = Vec::new();
    let mut trajectories = Vec::new();
    for nu in nu_offsets_deg(nu_offset_count) {
        for density in density_multipliers(density_count) {
            let results = drag_simulator(
                RendezvousConfig {
                    alt_offset_m: 0.0,
                    true_anomaly_offset_deg: nu,
                    density_multiplier: density,
                    use_j2: false,
                },
                None,
            );
            let index = sweep.len();
            let first = &results.hill[1];
            let last = results.hill.last().expect("recorded Hill states");
            sweep.push(SweepResult {
                nu_offset_deg: nu,
                density_multiplier: density,
                initial_range_m: first.position_dc_h_m.norm(),
                initial_speed_mps: first.velocity_dc_h_mps.norm(),
                final_range_m: last.position_dc_h_m.norm(),
                final_speed_mps: last.velocity_dc_h_mps.norm(),
            });
            if index % 5 == 0 {
                for (time, hill) in results.times_nanos.iter().zip(&results.hill) {
                    trajectories.push(vec![
                        index as f64,
                        *time as f64 * 1e-9,
                        hill.position_dc_h_m.x,
                        hill.position_dc_h_m.y,
                    ]);
                }
            }
        }
    }
    let headers: Vec<String> = [
        "nu_offset_deg",
        "density_multiplier",
        "initial_range_m",
        "initial_speed_mps",
        "final_range_m",
        "final_speed_mps",
    ]
    .map(String::from)
    .to_vec();
    let rows: Vec<Vec<f64>> = sweep
        .iter()
        .map(|r| {
            vec![
                r.nu_offset_deg,
                r.density_multiplier,
                r.initial_range_m,
                r.initial_speed_mps,
                r.final_range_m,
                r.final_speed_mps,
            ]
        })
        .collect();
    let csv_path = table_csv::write_table("scenarioDragSensitivity", &headers, &rows);
    let trajectories_path = table_csv::write_table(
        "scenarioDragSensitivityTrajectories",
        &["run", "time_s", "r_DC_H_m_0", "r_DC_H_m_1"].map(String::from),
        &trajectories,
    );
    Results {
        csv_path,
        trajectories_path,
        sweep,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let (nu_count, density_count) = if args.switch("all") {
        (5, 10)
    } else {
        (args.value("nu-offsets", 15), args.value("densities", 20))
    };
    let results = run(nu_count, density_count);
    let worst = results
        .sweep
        .iter()
        .map(|r| r.final_range_m)
        .fold(0.0_f64, f64::max);
    println!(
        "wrote {} and {} ({} runs, largest final range {:.2} m)",
        results.csv_path.display(),
        results.trajectories_path.display(),
        results.sweep.len(),
        worst
    );
}

#[cfg(test)]
mod tests {
    use super::{density_multipliers, nu_offsets_deg, run};

    #[test]
    fn sweep_grids_match_numpy() {
        let nu = nu_offsets_deg(5);
        assert_eq!(nu.len(), 5);
        assert!((nu[4] - 0.8002).abs() < 1e-12);
        assert_eq!(nu_offsets_deg(15).len(), 15);
        let density = density_multipliers(10);
        assert_eq!(density.len(), 10);
        assert!((density[0] - 0.1).abs() < 1e-15);
        assert!((density[9] - 10.0_f64.powf(0.4)).abs() < 1e-15);
    }

    /// `test_scenarioDragSensitivity.py`: `run(False, 5, 10)`, no checks.
    /// Below twice the nominal density every run reduces its range; with
    /// at least 0.85 of the nominal density, the small offsets close to the
    /// steady residual of about 23 m within the 40 orbits. (At 2.5 times
    /// the density both craft decay out of the 300 km orbit, which the
    /// static gain was not designed for.)
    #[test]
    fn sweep_shows_the_density_sensitivity() {
        let results = run(5, 10);
        assert_eq!(results.sweep.len(), 50);
        for result in &results.sweep {
            if result.density_multiplier < 2.0 {
                assert!(result.final_range_m < result.initial_range_m, "{result:?}");
            }
            if result.nu_offset_deg < 0.3 && (0.85..2.0).contains(&result.density_multiplier) {
                assert!(result.final_range_m < 30.0, "{result:?}");
            }
        }
    }
}
