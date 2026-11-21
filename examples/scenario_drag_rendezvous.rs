//! Port of upstream `examples/scenarioDragRendezvous.py`: two small
//! spacecraft on a 300 km orbit; the deputy, trailing the chief, rotates its
//! drag plate under an LQR law on the Hill-frame relative state until the
//! differential drag brings it to the chief, over 40 orbits at 60 s steps.
//! The simulation is `support/drag_rendezvous.rs` (upstream
//! `drag_simulator`), shared with `scenario_drag_sensitivity`; it prescribes
//! both attitudes through the spacecraft's attitude-reference input
//! (upstream `attRefInMsg`).
//!
//! Options: `--alt-offset-m` (default 0), `--true-anomaly-offset-deg` (0.1),
//! `--density-multiplier` (1), `--j2` (J2 gravity from
//! `GGM03S-J2-only.txt`). The run writes `scenarioDragRendezvous.csv`
//! (`scenarioDragRendezvousJ2.csv` with J2, the values in the name otherwise
//! when they are not the defaults): both states and attitudes, the Hill
//! relative state, densities, drag forces and the deputy's reference.
//! `--all` runs the two configurations of `test_scenarioDragRendezvous.py`:
//! upstream's `__main__` case and the same with J2.
//!
//! Upstream's tests only run the scenario (the J2 one also checks for data
//! and for no planet-orientation warning; the J2-only field is zonal and the
//! body keeps a fixed orientation here, so there is none to give). `tests`
//! checks the rendezvous: the deputy's along-track offset of about 11.7 km
//! closes to a small residual, the relative MRP saturates at 0.2 while far
//! away, and the densities stay those of the 300 km orbit.

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

use cli::Cli;
use drag_rendezvous::{RendezvousConfig, RendezvousResults, drag_simulator};

const USAGE: &str = "\
scenario_drag_rendezvous [--alt-offset-m 0] [--true-anomaly-offset-deg 0.1]
    [--density-multiplier 1] [--j2]
scenario_drag_rendezvous --all     run every upstream test configuration";

/// Upstream `__main__` and `test_scenarioDragRendezvous`: `(0.0, 0.1, 1)`,
/// `lqr`, no J2.
pub const DEFAULT: RendezvousConfig = RendezvousConfig {
    alt_offset_m: 0.0,
    true_anomaly_offset_deg: 0.1,
    density_multiplier: 1.0,
    use_j2: false,
};

/// The configurations of `test_scenarioDragRendezvous.py`.
pub const TEST_CASES: [RendezvousConfig; 2] = [
    DEFAULT,
    RendezvousConfig {
        use_j2: true,
        ..DEFAULT
    },
];

/// The CSV stem for `config`.
pub fn file_stem(config: &RendezvousConfig) -> String {
    let mut stem = "scenarioDragRendezvous".to_string();
    if config.alt_offset_m != DEFAULT.alt_offset_m
        || config.true_anomaly_offset_deg != DEFAULT.true_anomaly_offset_deg
        || config.density_multiplier != DEFAULT.density_multiplier
    {
        stem += &format!(
            "_alt{}_nu{}_dens{}",
            config.alt_offset_m, config.true_anomaly_offset_deg, config.density_multiplier
        );
    }
    if config.use_j2 {
        stem += "J2";
    }
    stem
}

/// Upstream `run(show_plots, altOffset, trueAnomOffset, densMultiplier,
/// ctrlType, useJ2)`.
pub fn run(config: RendezvousConfig) -> RendezvousResults {
    drag_simulator(config, Some(&file_stem(&config)))
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let configs = if args.switch("all") {
        TEST_CASES.to_vec()
    } else {
        vec![RendezvousConfig {
            alt_offset_m: args.value("alt-offset-m", DEFAULT.alt_offset_m),
            true_anomaly_offset_deg: args
                .value("true-anomaly-offset-deg", DEFAULT.true_anomaly_offset_deg),
            density_multiplier: args.value("density-multiplier", DEFAULT.density_multiplier),
            use_j2: args.switch("j2"),
        }]
    };
    for config in configs {
        let results = run(config);
        let first = results.hill[1].position_dc_h_m.norm();
        let last = results.hill.last().unwrap().position_dc_h_m.norm();
        println!(
            "wrote {} (Hill range {:.1} m -> {:.3} m, final speed {:.2e} m/s)",
            results.csv_path.as_ref().unwrap().display(),
            first,
            last,
            results.hill.last().unwrap().velocity_dc_h_mps.norm()
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{TEST_CASES, drag_simulator};
    use basilisk_rs::utilities::rigid_body_kinematics::sub_mrp;

    /// `test_scenarioDragRendezvous` (no checks) and
    /// `test_scenario_drag_rendezvous_j2_no_wind` (data recorded): the
    /// differential drag must close the deputy's 11.7 km along-track offset.
    #[test]
    fn differential_drag_closes_the_gap() {
        for config in TEST_CASES {
            let results = drag_simulator(config, None);
            assert!(!results.times_nanos.is_empty());
            let initial = results.hill[1].position_dc_h_m.norm();
            assert!(
                (initial - 11_655.0).abs() < 50.0,
                "initial range {initial} m"
            );
            let last = results.hill.last().unwrap();
            assert!(
                last.position_dc_h_m.norm() < 0.01 * initial,
                "J2 {}: final range {} m",
                config.use_j2,
                last.position_dc_h_m.norm()
            );
            // Far from the chief the commanded relative attitude saturates at
            // 0.2 (about the deputy's Hill frame, a hair off the chief's).
            let relative = sub_mrp(results.deputy[2].sigma_bn, results.chief[2].sigma_bn);
            assert!(
                (relative.amax() - 0.2).abs() < 0.01,
                "early relative MRP {relative:?}"
            );
            for density in results.chief_density.iter().chain(&results.deputy_density) {
                assert!(
                    *density > 1.0e-11 && *density < 1.0e-10,
                    "density {density}"
                );
            }
        }
    }
}
