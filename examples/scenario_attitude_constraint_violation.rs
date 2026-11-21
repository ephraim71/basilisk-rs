//! Keep-in and keep-out constraint violations of a constraint-naive slew:
//! port of upstream `examples/scenarioAttitudeConstraintViolation.py`.
//!
//! The Bevo 2 slew of [`constrained_attitude`] with `inertial3D` pointing
//! straight at the target attitude. One 0.5 s task, 1.5 minutes, 200
//! samples. The four initial and target attitudes are chosen to show the
//! violations: case 0 the sun sensor's keep-in cone only (with one sensor;
//! with both, one of them always sees the Sun), case 1 both sun sensors'
//! keep-in, case 2 the star tracker's keep-out and both keep-ins, and case
//! 3, with a 30° star tracker, the keep-out only.
//!
//! Upstream `run(show_plots, use2SunSensors, starTrackerFov, sunSensorFov,
//! attitudeSetCase)` maps to `--2-sun-sensors`, `--star-tracker-fov`,
//! `--sun-sensor-fov` and `--attitude-set-case`; the script's own default is
//! `--2-sun-sensors --star-tracker-fov 20 --sun-sensor-fov 70
//! --attitude-set-case 2`, and `--all` runs the five cases of upstream's
//! `test_scenarioAttitudeConstraintViolation.py`. Writes
//! `examples/output/scenarios/scenarioAttitudeConstraintViolation<1|0><st><ss><case>.csv`.

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

/// The scenario's settings: `inertial3D`, upstream's 0.5 s step,
/// 1.5 minutes and 200 samples.
fn setup() -> Setup {
    Setup {
        stem: "scenarioAttitudeConstraintViolation",
        guidance: Guidance::Inertial3D,
        step_nanos: 500_000_000,
        duration_nanos: 90_000_000_000,
        data_points: 200,
    }
}

/// The parametrizations of upstream `test_scenarioAttitudeConstraintViolation.py`.
const TEST_PARAMETRIZATIONS: [Case; 5] = [
    Case::new(false, 20, 70, 0),
    Case::new(true, 20, 70, 0),
    Case::new(true, 20, 70, 1),
    Case::new(true, 20, 70, 2),
    Case::new(true, 30, 70, 3),
];

fn run(case: Case, write_csv: bool) -> Results {
    constrained_attitude::run(&setup(), case, write_csv)
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_attitude_constraint_violation [--[no-]2-sun-sensors] [--star-tracker-fov DEG] \
         [--sun-sensor-fov DEG] [--attitude-set-case 0..3] [--all]",
    );
    let cases = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        // Upstream's `__main__` runs `run(True, True, 20, 70, 2)`.
        vec![Case::new(
            cli.flag("2-sun-sensors", true),
            cli.value("star-tracker-fov", 20),
            cli.value("sun-sensor-fov", 70),
            cli.value("attitude-set-case", 2),
        )]
    };
    cli.finish();
    for case in cases {
        let results = run(case, true);
        let degrees = |angles: Vec<f64>| -> (f64, f64) {
            let min = angles.iter().copied().fold(f64::INFINITY, f64::min);
            let max = angles.iter().copied().fold(0.0, f64::max);
            (min.to_degrees(), max.to_degrees())
        };
        let (st_min, _) = degrees(results.star_tracker_angles());
        let (_, ss_max) = degrees(results.sun_sensor_angles());
        println!(
            "wrote {} (star tracker Sun angle >= {st_min:.1} deg, sun sensor <= {ss_max:.1} deg)",
            results.csv_path.expect("CSV written").display()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAttitudeConstraintViolation.py` only checks
    //! that each case runs. These tests check the violations the scenario
    //! documents for each case.

    use super::*;

    /// Whether the star tracker sees the Sun (keep-out violated) and whether
    /// no sun sensor sees it (keep-in violated) at some sample.
    fn violations(case: Case) -> (bool, bool) {
        let results = run(case, false);
        let keep_out = results
            .star_tracker_angles()
            .iter()
            .any(|angle| *angle < case.star_tracker_fov_rad());
        let keep_in = results
            .sun_sensor_angles()
            .iter()
            .any(|angle| *angle > case.sun_sensor_fov_rad());
        // The slew reaches the target attitude.
        assert!(results.guidance.last().sigma_br.norm() < 0.05, "{case:?}");
        (keep_out, keep_in)
    }

    #[test]
    fn each_case_violates_the_documented_constraints() {
        let expected = [
            // One sun sensor: its keep-in cone only.
            (false, true),
            // Both sun sensors: one always sees the Sun.
            (false, false),
            // Both keep-in cones.
            (false, true),
            // The keep-out cone and both keep-in cones.
            (true, true),
            // A 30 deg star tracker: the keep-out cone only.
            (true, false),
        ];
        for (case, expected) in TEST_PARAMETRIZATIONS.into_iter().zip(expected) {
            assert_eq!(violations(case), expected, "{case:?}");
        }
    }
}
