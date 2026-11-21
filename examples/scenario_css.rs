//! Coarse Sun sensors on a spinning spacecraft: port of upstream
//! `examples/scenarioCSS.py`.
//!
//! A 750 kg rigid spacecraft at the origin spins at 1 deg/s about its z axis
//! with the Sun 1 AU along inertial y. Three coarse Sun sensors (`CSS1`
//! along +x and `CSS2` along +y with an 80 deg field of view, `CSS3` along -x
//! with 45 deg; scale factor 2, outputs limited to `[0.5, 2]`) measure it on
//! one 1 s task for 300 s.
//!
//! Upstream `run(show_plots, useCSSConstellation, usePlatform, useEclipse,
//! useKelly)` maps to the flags `--css-constellation`, `--platform`,
//! `--eclipse` (a fixed 0.5 illumination factor) and `--kelly` (Kelly factor
//! 0.2, upstream's default `kPower = 2`). With no flags the example runs the
//! upstream script's own default, `0010`; `--all` runs the five
//! parametrizations of upstream's `test_scenarioCSS.py`.
//!
//! * `usePlatform` gives the sensors their normals through upstream's
//!   platform frame instead: `setBodyToPlatformDCM(90 deg, 0, 0)` and the
//!   azimuth/elevation `(theta, phi)` of each sensor, `(-90, 0)`, `(0, 0)` and
//!   `(90, 0)` deg, which are the same normals up to rounding.
//! * `useCSSConstellation` evaluates the three sensors as one entity,
//!   upstream's `CSSConstellation`, here [`CssConstellation`], a module
//!   written in the example. basilisk-rs sensors keep one output message per
//!   device (no `CSSArraySensorMsg`), so the constellation publishes the same
//!   three messages, updated in the constellation's order, and records the
//!   same columns.
//!
//! The Sun sits at `149 597 870 700 m`, the value of the established
//! reference output of this example; upstream's `om.AU * 1000` is
//! `149 597 870 693 m`, a relative change of `1e-13` in the signals.
//!
//! Writes `examples/output/scenarios/scenarioCSS<c><p><e><k>.csv` (upstream's
//! figure suffix, `str(int(flag))` of the four flags) with `css1Signal`,
//! `css2Signal` and `css3Signal`.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;

use std::path::PathBuf;

use attitude::{bit, scenario_recorder};
use basilisk_rs::messages::{EclipseMsg, Output, SunEphemerisMsg, SunSensorMsg};
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::euler321_to_dcm;
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::seconds;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;
const TASK_PERIOD_NANOS: u64 = seconds(1);
const DURATION_NANOS: u64 = seconds(300);

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    use_css_constellation: bool,
    use_platform: bool,
    use_eclipse: bool,
    use_kelly: bool,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(
        use_css_constellation: bool,
        use_platform: bool,
        use_eclipse: bool,
        use_kelly: bool,
    ) -> Self {
        Self {
            use_css_constellation,
            use_platform,
            use_eclipse,
            use_kelly,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioCSS{}{}{}{}",
            bit(self.use_css_constellation),
            bit(self.use_platform),
            bit(self.use_eclipse),
            bit(self.use_kelly),
        )
    }
}

/// The parametrizations of upstream `test_scenarioCSS.py`.
const TEST_PARAMETRIZATIONS: [Config; 5] = [
    Config::new(false, false, false, false),
    Config::new(false, true, false, false),
    Config::new(false, false, true, false),
    Config::new(false, false, false, true),
    Config::new(true, false, false, false),
];

/// Upstream `CSSConstellation`: the sensors of a cluster evaluated as one
/// module, in order. Each sensor keeps its own output message.
struct CssConstellation {
    sensors: Vec<CoarseSunSensor>,
}

impl Module for CssConstellation {
    fn init(&mut self) {
        for sensor in &mut self.sensors {
            sensor.init();
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        for sensor in &mut self.sensors {
            sensor.update(context);
        }
    }
}

/// Upstream `setBodyToPlatformDCM(90 deg, 0, 0)` and
/// `setUnitDirectionVectorWithPerturbation(0, 0)`: the sensor normal at
/// azimuth `theta` and elevation `phi` in the platform frame, in body
/// components.
fn platform_normal(theta_rad: f64, phi_rad: f64) -> Vector3<f64> {
    let dcm_pb: Matrix3<f64> = euler321_to_dcm(Vector3::new(90.0_f64.to_radians(), 0.0, 0.0));
    let normal_p = Vector3::new(
        phi_rad.cos() * theta_rad.cos(),
        phi_rad.cos() * theta_rad.sin(),
        phi_rad.sin(),
    );
    dcm_pb.transpose() * normal_p
}

fn sensor(
    name: &str,
    position_m: Vector3<f64>,
    normal_body: Vector3<f64>,
    fov_deg: f64,
    use_kelly: bool,
) -> CoarseSunSensor {
    let body_to_sensor_quaternion =
        UnitQuaternion::rotation_between(&normal_body.normalize(), &Vector3::z())
            .expect("non-opposite CSS mounting vectors");
    CoarseSunSensor::new(CoarseSunSensorConfig {
        name: name.to_string(),
        position_m,
        body_to_sensor_quaternion,
        fov_half_angle_rad: fov_deg.to_radians(),
        scale_factor: 2.0,
        kelly_factor: if use_kelly { 0.2 } else { 0.0 },
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 0.0,
        walk_bounds: 0.0,
        min_output: 0.5,
        max_output: 2.0,
    })
}

/// The recorded signals of `CSS1`, `CSS2` and `CSS3`, every step.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    signals: [MessageLog<SunSensorMsg>; 3],
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 750.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.0, 1.0_f64.to_radians()),
        integrator: None,
    });
    let normals = if config.use_platform {
        [
            platform_normal((-90.0_f64).to_radians(), 0.0),
            platform_normal(0.0, 0.0),
            platform_normal(90.0_f64.to_radians(), 0.0),
        ]
    } else {
        [Vector3::x(), Vector3::y(), -Vector3::x()]
    };
    let mut sensors = vec![
        sensor(
            "CSS1_sensor",
            Vector3::new(2.00131, 2.36638, 1.0),
            normals[0],
            80.0,
            config.use_kelly,
        ),
        sensor(
            "CSS2_sensor",
            Vector3::new(-3.05, 0.55, 1.0),
            normals[1],
            80.0,
            config.use_kelly,
        ),
        sensor(
            "CSS3_sensor",
            Vector3::new(-3.05, 0.55, 1.0),
            normals[2],
            45.0,
            config.use_kelly,
        ),
    ];

    let sun = Output::new(SunEphemerisMsg {
        sun_position_inertial_m: Vector3::new(0.0, ASTRONOMICAL_UNIT_M, 0.0),
        sun_velocity_inertial_mps: Vector3::zeros(),
    });
    let eclipse = Output::new(EclipseMsg {
        illumination_factor: 0.5,
    });
    let mut signal_logs: [MessageLog<SunSensorMsg>; 3] =
        std::array::from_fn(|_| MessageLog::default());
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut constellation = CssConstellation {
        sensors: Vec::new(),
    };

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (index, (sensor, log)) in sensors.iter_mut().zip(&mut signal_logs).enumerate() {
        connect!(&simulation,
            &spacecraft.state_out => &mut sensor.input_state_msg,
            &sun => &mut sensor.input_sun_msg,
            &sensor.output_sun_sensor_msg => &mut log.input,
            &sensor.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
                CsvSourceConfig::columns([(
                    "sensed_value".to_string(),
                    format!("css{}Signal", index + 1),
                )]),
            ),
        );
        if config.use_eclipse {
            connect!(&simulation, &eclipse => &mut sensor.input_eclipse_msg);
        }
    }
    simulation.add_module("spacecraft", &mut spacecraft, TASK_PERIOD_NANOS, 30);
    if config.use_css_constellation {
        constellation.sensors = sensors;
        simulation.add_module("css_array", &mut constellation, TASK_PERIOD_NANOS, 20);
    } else {
        for (index, sensor) in sensors.iter_mut().enumerate() {
            simulation.add_module(format!("css{}", index + 1), sensor, TASK_PERIOD_NANOS, 20);
        }
    }
    for log in &mut signal_logs {
        simulation.add_module("signal_log", log, TASK_PERIOD_NANOS, 0);
    }
    if config.write_csv {
        schedule! { simulation,
            "recorder" => &mut recorder, period=TASK_PERIOD_NANOS, priority=0;
        }
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        signals: signal_logs,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_css [--[no-]css-constellation] [--[no-]platform] [--[no-]eclipse] \
         [--[no-]kelly] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("css-constellation", false),
            cli.flag("platform", false),
            cli.flag("eclipse", true),
            cli.flag("kelly", false),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!("wrote {}", results.csv_path.expect("CSV written").display());
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioCSS.py` only checks that each parametrization
    //! runs. These tests run the same parametrizations and check the signals
    //! against the cosine law the scenario demonstrates: at time `t` the Sun
    //! is at azimuth `90 deg - t deg/s` in the body x-y plane, so a sensor
    //! with normal azimuth `alpha` reads `2 f cos(90 deg - t - alpha)` inside
    //! its field of view, limited to `[0.5, 2]`, where `f` is the
    //! illumination factor and the Kelly fit `1 - exp(-cos^2 / 0.2)` when
    //! enabled.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// The expected `[CSS1, CSS2, CSS3]` signals at `t` seconds, `None` for a
    /// sensor within `1e-9` of its field-of-view edge.
    fn expected(config: &Config, t: f64) -> [Option<f64>; 3] {
        let normals_deg: [f64; 3] = [0.0, 90.0, 180.0];
        let fov_deg = [80.0_f64, 80.0, 45.0];
        let illumination = if config.use_eclipse { 0.5 } else { 1.0 };
        std::array::from_fn(|index| {
            let cosine = (90.0 - t - normals_deg[index]).to_radians().cos();
            let edge = fov_deg[index].to_radians().cos();
            if (cosine - edge).abs() < 1.0e-9 {
                return None;
            }
            let mut signal = if cosine >= edge { cosine } else { 0.0 };
            if config.use_kelly && signal > 0.0 {
                signal *= 1.0 - (-signal * signal / 0.2).exp();
            }
            let distance = ASTRONOMICAL_UNIT_M / 149_597_870_693.0;
            Some((2.0 * signal * illumination * distance * distance).clamp(0.5, 2.0))
        })
    }

    #[test]
    fn every_upstream_parametrization_follows_the_cosine_law() {
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            for log in &results.signals {
                assert_eq!(log.values.len(), 301, "{config:?}");
            }
            for step in 0..=300 {
                let truth = expected(&config, f64::from(step));
                for (log, truth) in results.signals.iter().zip(truth) {
                    let Some(truth) = truth else { continue };
                    let signal = log.values[step as usize].sensed_value;
                    assert!(
                        (signal - truth).abs() < 1.0e-9,
                        "{config:?} t = {step}: {signal} vs {truth}"
                    );
                }
            }
        }
    }

    /// The platform frame and the constellation change how the sensors are
    /// set up and run, not what they measure.
    #[test]
    fn the_platform_and_constellation_variants_match_the_plain_sensors() {
        let plain = quiet(TEST_PARAMETRIZATIONS[0]);
        for config in [TEST_PARAMETRIZATIONS[1], TEST_PARAMETRIZATIONS[4]] {
            let variant = quiet(config);
            for (left, right) in plain.signals.iter().zip(&variant.signals) {
                for (left, right) in left.values.iter().zip(&right.values) {
                    assert!((left.sensed_value - right.sensed_value).abs() < 1.0e-12);
                }
            }
        }
    }

    #[test]
    fn the_platform_normals_are_the_body_axes() {
        let normals = [
            platform_normal((-90.0_f64).to_radians(), 0.0),
            platform_normal(0.0, 0.0),
            platform_normal(90.0_f64.to_radians(), 0.0),
        ];
        for (normal, axis) in normals
            .iter()
            .zip([Vector3::x(), Vector3::y(), -Vector3::x()])
        {
            assert!((normal - axis).norm() < 1.0e-15, "{normal}");
        }
    }
}
