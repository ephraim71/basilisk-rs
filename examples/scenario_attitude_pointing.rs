//! Detumbling and inertial pointing in deep space: port of upstream
//! `examples/scenarioAttitudePointing.py`.
//!
//! A 750 kg rigid spacecraft with no orbit (no gravity body, at rest at the
//! origin) starts at `sigma_BN = [0.1, 0.2, -0.3]` and is brought to the
//! inertial frame by `inertial3D`, `attTrackingError` and `mrpFeedback`
//! (`K = 3.5`, `P = 30`, no integral feedback) through an `extForceTorque`
//! effector. One 0.1 s task, 10 minutes, histories sampled every 12.2 s
//! (`samplingTime(T, 0.1 s, 50)`). The simulation is `support/pointing.rs`,
//! shared with the Python- and Numba-module variants of this scenario.
//!
//! Upstream `run(show_plots, useLargeTumble)` maps to `--large-tumble`: the
//! initial rate is `[0.8, -0.6, 0.5] rad/s` instead of
//! `[0.001, -0.01, 0.03] rad/s`. `--all` runs both, as upstream's
//! `test_scenarioAttitudePointing.py` does.
//!
//! Writes `examples/output/scenarios/scenarioAttitudePointing<lt>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/pointing.rs"]
mod pointing;

use attitude::{INITIAL_OMEGA_BN_RADPS, bit, mrp_feedback};
use cli::Cli;
use nalgebra::Vector3;
use pointing::{ControllerPorts, PointingResults, run_pointing};

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_large_tumble: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(use_large_tumble: bool) -> Self {
        Self {
            use_large_tumble,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!("scenarioAttitudePointing{}", bit(self.use_large_tumble))
    }
}

/// The parametrizations of upstream `test_scenarioAttitudePointing.py`.
const TEST_PARAMETRIZATIONS: [Config; 2] = [Config::new(false), Config::new(true)];

/// Upstream `omega_BN_BInit` \[rad/s\] of the large tumble.
const LARGE_TUMBLE_RADPS: [f64; 3] = [0.8, -0.6, 0.5];

fn run(config: Config) -> PointingResults {
    let mut controller = mrp_feedback(3.5, -1.0, 30.0, Vector3::zeros());
    let ports = ControllerPorts {
        guid_in_msg: controller.guid_in_msg.clone(),
        cmd_torque_out_msg: controller.cmd_torque_out_msg.clone(),
        veh_config_in_msg: Some(controller.veh_config_in_msg.clone()),
    };
    let initial_rate = Vector3::from(if config.use_large_tumble {
        LARGE_TUMBLE_RADPS
    } else {
        INITIAL_OMEGA_BN_RADPS
    });
    run_pointing(
        &mut controller,
        ports,
        initial_rate,
        &config.file_stem(),
        config.write_csv,
    )
}

fn main() {
    let mut cli = Cli::from_env("scenario_attitude_pointing [--[no-]large-tumble] [--all]");
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(cli.flag("large-tumble", false))]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} (final |sigma_BR| = {:.3e})",
            results.csv_path.expect("CSV written").display(),
            results.guidance.last().sigma_br.norm(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudePointing.py` only checks that both
    //! tumble cases run. These tests check that both are detumbled and
    //! brought to the inertial frame.

    use super::*;

    fn quiet(config: Config) -> PointingResults {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    #[test]
    fn both_tumbles_are_stabilized_to_the_inertial_frame() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            // samplingTime(10 min, 0.1 s, 50) = 12.2 s: 50 samples.
            assert_eq!(results.guidance.values.len(), 50);
            let first = &results.guidance.values[0];
            assert_eq!(first.sigma_br, Vector3::new(0.1, 0.2, -0.3));
            let initial_rate = if config.use_large_tumble {
                LARGE_TUMBLE_RADPS
            } else {
                INITIAL_OMEGA_BN_RADPS
            };
            assert_eq!(first.omega_br_b_radps, Vector3::from(initial_rate));

            let last = results.guidance.last();
            assert!(
                last.sigma_br.norm() < 1.0e-3,
                "{config:?}: {}",
                last.sigma_br
            );
            assert!(last.omega_br_b_radps.norm() < 1.0e-4, "{config:?}");
            assert!(results.torque.last().torque_request_body_nm.norm() < 1.0e-2);
        }
    }

    /// "The spacecraft quickly regains a stable orientation without
    /// tumbling past 180 degrees": the small tumble's attitude error stays
    /// below `|sigma| = 0.5`, a 106 degree principal angle.
    #[test]
    fn the_small_tumble_never_rotates_past_180_degrees() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        assert!(
            results
                .guidance
                .values
                .iter()
                .all(|guidance| guidance.sigma_br.norm() < 0.5)
        );
    }
}
