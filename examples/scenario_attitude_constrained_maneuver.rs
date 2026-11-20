//! A constraint-compliant slew planned by `constrainedAttitudeManeuver`:
//! port of upstream `examples/scenarioAttitudeConstrainedManeuver.py`.
//!
//! The Bevo 2 slew of [`constrained_attitude`], set up as in
//! `scenarioAttitudeConstraintViolation`, with `inertial3D` replaced by
//! `constrainedAttitudeManeuver` (`N = 8`, `avgOmega = 0.04 rad/s`,
//! interpolating B-spline, minimum-effort A* search): the star tracker
//! (body `x`) is its keep-out instrument and the sun sensors its keep-in
//! instruments, with the Sun as the bright body. It plans the maneuver at
//! initialization and publishes the reference trajectory. One 0.01 s task,
//! 3.5 minutes, 500 samples. Where `inertial3D` violates the constraints,
//! the planned maneuver does not.
//!
//! Upstream `run(show_plots, use2SunSensors, starTrackerFov, sunSensorFov,
//! attitudeSetCase)` maps to `--2-sun-sensors`, `--star-tracker-fov`,
//! `--sun-sensor-fov` and `--attitude-set-case`; the script's own default is
//! `--2-sun-sensors --star-tracker-fov 30 --sun-sensor-fov 70
//! --attitude-set-case 3`, and `--all` runs the three cases of upstream's
//! `test_scenarioAttitudeConstrainedManeuver.py`. Writes
//! `examples/output/scenarios/scenarioAttitudeConstrainedManeuver<1|0><st><ss><case>.csv`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/constrained_attitude.rs"]
mod constrained_attitude;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/rw_factory.rs"]
mod rw_factory;
#[path = "support/staged.rs"]
mod staged;

use cli::Cli;
use constrained_attitude::{Case, Guidance, Results, Setup};

/// The scenario's settings: `constrainedAttitudeManeuver`, upstream's
/// 0.01 s step, 3.5 minutes and 500 samples.
fn setup() -> Setup {
    Setup {
        stem: "scenarioAttitudeConstrainedManeuver",
        guidance: Guidance::ConstrainedManeuver,
        step_nanos: 10_000_000,
        duration_nanos: 210_000_000_000,
        data_points: 500,
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeConstrainedManeuver.py`.
const TEST_PARAMETRIZATIONS: [Case; 3] = [
    Case::new(false, 20, 70, 0),
    Case::new(true, 20, 70, 1),
    Case::new(true, 20, 70, 2),
];

fn run(case: Case, write_csv: bool) -> Results {
    constrained_attitude::run(&setup(), case, write_csv)
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_attitude_constrained_maneuver [--[no-]2-sun-sensors] [--star-tracker-fov DEG] \
         [--sun-sensor-fov DEG] [--attitude-set-case 0..3] [--all]",
    );
    let cases = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, True, 30, 70, 3)`.
        vec![Case::new(
            cli.flag("2-sun-sensors", true),
            cli.value("star-tracker-fov", 30),
            cli.value("sun-sensor-fov", 70),
            cli.value("attitude-set-case", 3),
        )]
    };
    cli.finish();
    for case in cases {
        let results = run(case, true);
        let st_min = results
            .star_tracker_angles()
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let ss_max = results
            .sun_sensor_angles()
            .iter()
            .copied()
            .fold(0.0, f64::max);
        println!(
            "wrote {} (star tracker Sun angle >= {:.1} deg, sun sensor <= {:.1} deg)",
            results.csv_path.expect("CSV written").display(),
            st_min.to_degrees(),
            ss_max.to_degrees()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeConstrainedManeuver.py` only checks
    //! that each case runs. These tests check what the scenario documents:
    //! the planned slew violates neither constraint, and ends at the goal.

    use basilisk_rs::utilities::rigid_body_kinematics::{mrp_to_dcm, sub_mrp};
    use nalgebra::Vector3;

    use super::*;
    use constrained_attitude::SIGMA_BN_TARGET;

    #[test]
    fn the_planned_slews_respect_both_constraints() {
        for case in TEST_PARAMETRIZATIONS {
            let results = run(case, false);
            let st_min = results
                .star_tracker_angles()
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min);
            let ss_max = results
                .sun_sensor_angles()
                .iter()
                .copied()
                .fold(0.0, f64::max);
            assert!(
                st_min > case.star_tracker_fov_rad(),
                "{case:?}: {}",
                st_min.to_degrees()
            );
            assert!(
                ss_max < case.sun_sensor_fov_rad(),
                "{case:?}: {}",
                ss_max.to_degrees()
            );

            // The reference ends at the goal, at rest, and the spacecraft
            // follows it there.
            let goal = Vector3::from(SIGMA_BN_TARGET[case.attitude_set_case]);
            let reference = results.reference.last();
            assert!(
                (mrp_to_dcm(reference.sigma_rn) - mrp_to_dcm(goal)).norm() < 1e-9,
                "{case:?}"
            );
            assert!(reference.omega_rn_n_radps.norm() < 1e-12);
            let attitude = results.state.last().sigma_bn;
            assert!(
                sub_mrp(attitude, goal).norm() < 1e-2,
                "{case:?}: {attitude}"
            );
        }
    }
}
