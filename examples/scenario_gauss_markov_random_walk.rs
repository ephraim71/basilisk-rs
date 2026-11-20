//! Bounded Gauss-Markov random walk against white noise, on two rate gyros:
//! port of upstream `examples/scenarioGaussMarkovRandomWalk.py`.
//!
//! A 750 kg spacecraft (inertia `diag(900, 800, 600) kg m^2`) on an
//! `a = REQ + 7500 km`, `e = 0.01` Earth orbit, integrated with RKF45 at
//! upstream's default tolerances (`relTol = 1e-4`, `absTol = 1e-8`), tumbles
//! torque-free from `sigma_BN = [0.1, 0.2, -0.3]`,
//! `omega_BN = [0.001, -0.01, 0.03] rad/s`. Two IMUs measure its rate for
//! 10 minutes on a 0.1 s task, recorded every step:
//!
//! * `imuSensor1`: a random walk (`AMatrixGyro = I`) with process noise
//!   `PMatrixGyro = processNoiseLevel I` and error bounds `±walkBounds`;
//! * `imuSensor2`: white noise (`AMatrixGyro = 0`) of the same level, no
//!   bounds.
//!
//! Upstream `run(show_plots, processNoiseLevel=0.5, walkBounds=3.0)` maps to
//! `--process-noise-level` and `--walk-bounds`; `--all` runs the single
//! parametrization of upstream's `test_scenarioGaussMarkovRandomWalk.py`,
//! which is upstream's default.
//!
//! # Noise model
//!
//! basilisk-rs's [`GaussMarkov`](basilisk_rs::gauss_markov) is upstream's:
//! the noise matrix `processNoiseLevel I` gives each step a one-sigma of
//! `processNoiseLevel`, and the walk is clamped at `walkBounds`.
//!
//! Writes `examples/output/scenarios/scenarioGaussMarkovRandomWalk.csv` with
//! both gyros' rates (`AngVelPlatform1_rad_per_s_*`,
//! `AngVelPlatform2_rad_per_s_*`) and the true body rate
//! (`omega_BN_B_rad_per_s_*`).

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{
    DURATION_NANOS, DYNAMICS_PERIOD_NANOS, HUB_MASS_KG, hub_inertia_kg_m2, scenario_recorder,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::integrators::{AdaptiveController, Rkf45};
use basilisk_rs::messages::{ImuMsg, SpacecraftStateMsg};
use basilisk_rs::sensors::imu_sensor::{ImuSensor, ImuSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, REQ_EARTH};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    process_noise_level: f64,
    walk_bounds: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(process_noise_level: f64, walk_bounds: f64) -> Self {
        Self {
            process_noise_level,
            walk_bounds,
            write_csv: true,
        }
    }
}

/// The parametrization of upstream `test_scenarioGaussMarkovRandomWalk.py`.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config::new(0.5, 3.0)];

/// The recorded histories (upstream `dataLog1`, `dataLog2`), every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    imu1: MessageLog<ImuMsg>,
    imu2: MessageLog<ImuMsg>,
    state: MessageLog<SpacecraftStateMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream `imuSensor.ImuSensor()` with `PMatrixGyro = level I`,
/// `AMatrixGyro` and error bounds `bounds` (zero for none).
fn gyro(name: &str, level: f64, a_matrix: Matrix3<f64>, bounds: f64) -> ImuSensor {
    ImuSensor::new(ImuSensorConfig {
        name: name.to_string(),
        p_matrix_sqrt_radps: Matrix3::from_diagonal_element(level),
        a_matrix,
        walk_bounds_radps: Vector3::repeat(bounds),
        ..ImuSensorConfig::default()
    })
}

fn run(config: Config) -> Results {
    let mu = MU_EARTH * 1e9;
    let elements = ClassicElements {
        semi_major_axis: REQ_EARTH * 1e3 + 7500e3,
        eccentricity: 0.01,
        inclination_rad: 33.3 * D2R,
        raan_rad: 48.2 * D2R,
        arg_periapsis_rad: 347.8 * D2R,
        true_anomaly_rad: 85.3 * D2R,
        ..Default::default()
    };
    let (initial_position_m, initial_velocity_mps) = elements_to_state(mu, &elements);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: hub_inertia_kg_m2(),
        integration_step_nanos: DYNAMICS_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        initial_omega_radps: Vector3::new(0.001, -0.01, 0.03),
        // Upstream `svIntegratorRKF45` with its default tolerances.
        integrator: Some(Box::new(Rkf45::new(AdaptiveController {
            atol: 1e-8,
            rtol: 1e-4,
            ..AdaptiveController::default()
        }))),
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");

    let mut imu1 = gyro(
        "imuSensor1",
        config.process_noise_level,
        Matrix3::identity(),
        config.walk_bounds,
    );
    let mut imu2 = gyro(
        "imuSensor2",
        config.process_noise_level,
        Matrix3::zeros(),
        0.0,
    );

    let mut imu1_log = MessageLog::<ImuMsg>::default();
    let mut imu2_log = MessageLog::<ImuMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioGaussMarkovRandomWalk", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    connect!(&simulation,
        &spacecraft.state_out => &mut imu1.input_state_msg,
        &spacecraft.state_out => &mut imu2.input_state_msg,
        &imu1.output_imu_msg => &mut imu1_log.input,
        &imu2.output_imu_msg => &mut imu2_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &imu1.output_imu_msg => recorder.add_source::<ImuMsg>(CsvSourceConfig::columns(
            vector_columns("angular_rate_sensor_radps", "AngVelPlatform1_rad_per_s"),
        )),
        &imu2.output_imu_msg => recorder.add_source::<ImuMsg>(CsvSourceConfig::columns(
            vector_columns("angular_rate_sensor_radps", "AngVelPlatform2_rad_per_s"),
        )),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
        ),
    );
    schedule! { simulation,
        "spacecraft" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "imu1" => &mut imu1, period=DYNAMICS_PERIOD_NANOS, priority=90;
        "imu2" => &mut imu2, period=DYNAMICS_PERIOD_NANOS, priority=80;
        "imu1_log" => &mut imu1_log, period=DYNAMICS_PERIOD_NANOS, priority=0;
        "imu2_log" => &mut imu2_log, period=DYNAMICS_PERIOD_NANOS, priority=0;
        "state_log" => &mut state_log, period=DYNAMICS_PERIOD_NANOS, priority=0;
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, DYNAMICS_PERIOD_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        imu1: imu1_log,
        imu2: imu2_log,
        state: state_log,
        csv_path,
    }
}

/// Population standard deviation (upstream `np.std`).
fn standard_deviation(values: &[f64]) -> f64 {
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    (values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / values.len() as f64)
        .sqrt()
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_gauss_markov_random_walk [--process-noise-level <rad/s>] \
         [--walk-bounds <rad/s>] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.value("process-noise-level", 0.5),
            cli.value("walk-bounds", 3.0),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        // Upstream prints the standard deviations of `AngVelPlatform[0]`.
        let rate_x = |log: &MessageLog<ImuMsg>| -> Vec<f64> {
            log.values
                .iter()
                .map(|reading| reading.angular_rate_sensor_radps.x)
                .collect()
        };
        println!("Standard deviations:");
        println!(
            "IMU1: {:.3} rad/s",
            standard_deviation(&rate_x(&results.imu1))
        );
        println!(
            "IMU2: {:.3} rad/s",
            standard_deviation(&rate_x(&results.imu2))
        );
        println!("wrote {}", results.csv_path.expect("CSV written").display());
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioGaussMarkovRandomWalk.py` only checks that its
    //! parametrization runs. These tests check what the scenario
    //! demonstrates, on the gyro errors (reading minus true rate) about each
    //! axis: the white-noise gyro has upstream's one-sigma step and no
    //! memory, the random-walk gyro is strongly correlated in time and held
    //! at its bounds, which clamp it.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// The gyro errors about body axis `axis`: reading minus the true rate.
    fn errors(
        log: &MessageLog<ImuMsg>,
        state: &MessageLog<SpacecraftStateMsg>,
        axis: usize,
    ) -> Vec<f64> {
        log.values
            .iter()
            .zip(&state.values)
            .map(|(reading, truth)| {
                reading.angular_rate_sensor_radps[axis] - truth.omega_radps[axis]
            })
            .collect()
    }

    fn lag_one_autocorrelation(values: &[f64]) -> f64 {
        let mean = values.iter().sum::<f64>() / values.len() as f64;
        let variance: f64 = values.iter().map(|value| (value - mean).powi(2)).sum();
        let covariance: f64 = values
            .windows(2)
            .map(|pair| (pair[0] - mean) * (pair[1] - mean))
            .sum();
        covariance / variance
    }

    #[test]
    fn the_upstream_parametrization_records_every_step() {
        let results = quiet(TEST_PARAMETRIZATIONS[0]);
        // 10 min at 0.1 s: 6001 samples, as upstream's unsampled recorders.
        assert_eq!(results.imu1.values.len(), 6001);
        assert_eq!(results.imu2.values.len(), 6001);
    }

    #[test]
    fn the_white_noise_gyro_has_the_process_noise_sigma_and_no_memory() {
        let config = TEST_PARAMETRIZATIONS[0];
        let results = quiet(config);
        for axis in 0..3 {
            let errors = errors(&results.imu2, &results.state, axis);
            let sigma = standard_deviation(&errors);
            assert!(
                (sigma - config.process_noise_level).abs() < 0.05 * config.process_noise_level,
                "axis {axis}: {sigma}"
            );
            assert!(lag_one_autocorrelation(&errors).abs() < 0.05);
        }
    }

    #[test]
    fn the_random_walk_gyro_is_correlated_and_held_near_its_bounds() {
        let config = TEST_PARAMETRIZATIONS[0];
        let results = quiet(config);
        for axis in 0..3 {
            let errors = errors(&results.imu1, &results.state, axis);
            assert!(lag_one_autocorrelation(&errors) > 0.9, "axis {axis}");
            let largest = errors
                .iter()
                .fold(0.0_f64, |largest, error| largest.max(error.abs()));
            assert!(largest <= config.walk_bounds, "axis {axis}: {largest}");
            // An unbounded walk of 6000 such steps would spread to
            // sqrt(6000) * 0.5 = 39 rad/s.
            assert!(
                standard_deviation(&errors) < config.walk_bounds,
                "axis {axis}"
            );
        }
    }
}
