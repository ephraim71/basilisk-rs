//! Planetary albedo seen by coarse Sun sensors: port of upstream
//! `examples/scenarioAlbedo.py`.
//!
//! The Sun sits 1 AU along inertial -x and a 750 kg spacecraft spins about
//! its z axis. One to three coarse Sun sensors (normals `+x`, `-x`, `-y`; 80
//! deg field of view, outputs limited to `[0, 1]`) read the Sun and the
//! planetary albedo that an `albedo` module computes at each of them.
//!
//! * Single planet: a 500 km, near-circular equatorial Earth orbit starting
//!   on the sunlit side (`f = 180 deg`), spinning at 0.5 deg/s, for half an
//!   orbit. Earth's albedo is either the CERES 5x5 deg map
//!   (`Earth_ALB_2018_CERES_All_5x5.csv`, `just fetch-albedo`) or a uniform
//!   0.5 on a 200 x 200 grid. With `useEclipse`, an `eclipse`
//!   module shadows the sensors and the albedo module shadows each patch.
//! * `multiplePlanet`: no gravity; the spacecraft starts 500 km above Earth
//!   (on inertial +y) at rest spinning at 1 deg/s, flies towards the Moon at
//!   375 300 m/s from 500 s to 1500 s, and stops 485 km above it until
//!   2000 s. The Moon has its average albedo (upstream `getAlbedoAverage`,
//!   0.123) on upstream's default 180 x 360 grid.
//!
//! Upstream `run(show_plots, albedoData, multipleInstrument,
//! multiplePlanet, useEclipse, simTimeStep)` maps to the flags
//! `--albedo-data`, `--multiple-instrument`, `--multiple-planet`,
//! `--eclipse` and `--sim-time-step <s>` (10 s when not given, as upstream's
//! `None`); `--all` runs the four parametrizations of upstream's
//! `test_scenarioAlbedo.py`, all with its 25 s step.
//!
//! Mapping to basilisk-rs:
//!
//! * each sensor reads its albedo through the coarse Sun sensor's
//!   `input_albedo_msg` (upstream `albedoInMsg`);
//! * upstream sets the hub velocity between `ExecuteSimulation` calls
//!   (`velRef.setState`); here the spacecraft is scheduled as a
//!   [`Staged`] that, at its first update after each of those stop times,
//!   restarts from its current position, attitude and rate with the new
//!   velocity (exact here: the multiple-planet case has no gravity and no
//!   effectors).
//!
//! Writes `examples/output/scenarios/scenarioAlbedo<ad><mi><mp><e>.csv`
//! (upstream's figure suffix) with `r_BN_N_m_*`, `css<i>Signal` and
//! `albedo<i>` (upstream's `albedoAtInstrument`) per sensor.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::{HUB_MASS_KG, bit, hub_inertia_kg_m2, scenario_recorder};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::albedo::{
    Albedo, AlbedoConfig, AlbedoInstrument, AlbedoPlanet, load_albedo_map,
};
use basilisk_rs::environment::eclipse::{Eclipse, EclipseConfig};
use basilisk_rs::messages::{
    AlbedoMsg, EclipseMsg, Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg,
    SunEphemerisMsg, SunSensorMsg,
};
use basilisk_rs::sensors::coarse_sun_sensor::{CoarseSunSensor, CoarseSunSensorConfig};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::{AU, D2R, MU_EARTH, REQ_EARTH, REQ_MOON};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{Module, connect};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{UnitQuaternion, Vector3};
use staged::Staged;

/// Upstream `simTimeStep` of `test_scenarioAlbedo.py` \[s\].
const TEST_TIME_STEP_S: f64 = 25.0;
/// Upstream's `simTimeStep = None` default \[s\].
const DEFAULT_TIME_STEP_S: f64 = 10.0;
/// Upstream's CSS field of view \[deg\].
const CSS_FOV_DEG: f64 = 80.0;
/// Upstream's lunar position (inertial +y) \[m\].
const MOON_DISTANCE_M: f64 = 384_400.0 * 1000.0;
/// Upstream's transfer velocity of the multiple-planet case \[m/s\].
const TRANSFER_SPEED_MPS: f64 = 375_300.0;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    albedo_data: bool,
    multiple_instrument: bool,
    multiple_planet: bool,
    use_eclipse: bool,
    sim_time_step_s: f64,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

impl Config {
    const fn new(
        albedo_data: bool,
        multiple_instrument: bool,
        multiple_planet: bool,
        use_eclipse: bool,
        sim_time_step_s: f64,
    ) -> Self {
        Self {
            albedo_data,
            multiple_instrument,
            multiple_planet,
            use_eclipse,
            sim_time_step_s,
            write_csv: true,
        }
    }

    fn file_stem(&self) -> String {
        format!(
            "scenarioAlbedo{}{}{}{}",
            bit(self.albedo_data),
            bit(self.multiple_instrument),
            bit(self.multiple_planet),
            bit(self.use_eclipse),
        )
    }

    fn step_nanos(&self) -> u64 {
        (self.sim_time_step_s * 1e9 + 0.5) as u64
    }
}

/// The parametrizations of upstream `test_scenarioAlbedo.py`.
const TEST_PARAMETRIZATIONS: [Config; 4] = [
    Config::new(true, false, false, false, TEST_TIME_STEP_S),
    Config::new(false, true, false, false, TEST_TIME_STEP_S),
    Config::new(false, true, false, true, TEST_TIME_STEP_S),
    Config::new(false, true, true, false, TEST_TIME_STEP_S),
];

/// The recorded histories, every step (upstream `css<i>Log`, `alb<i>Log`,
/// `dataLog`).
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    css: Vec<MessageLog<SunSensorMsg>>,
    albedo: Vec<MessageLog<AlbedoMsg>>,
    state: MessageLog<SpacecraftStateMsg>,
    /// The sensors' illumination factor, with `useEclipse`.
    eclipse: Option<MessageLog<EclipseMsg>>,
    csv_path: Option<PathBuf>,
}

/// Upstream `setupCSS`: the default sensor with an 80 deg field of view and
/// `maxOutput = 1`.
fn css(name: &str, normal: Vector3<f64>) -> CoarseSunSensor {
    CoarseSunSensor::new(CoarseSunSensorConfig {
        name: name.to_string(),
        position_m: Vector3::zeros(),
        body_to_sensor_quaternion: UnitQuaternion::rotation_between(&normal, &Vector3::z())
            .expect("non-opposite CSS mounting vectors"),
        fov_half_angle_rad: CSS_FOV_DEG * D2R,
        scale_factor: 1.0,
        kelly_factor: 0.0,
        k_power: 2.0,
        bias: 0.0,
        noise_std: 0.0,
        noise_prop: 0.0,
        walk_bounds: -1.0,
        min_output: 0.0,
        max_output: 1.0,
    })
}

/// The spacecraft restarted from `old`'s current position, attitude and
/// rate with the hub velocity `velocity_mps`: upstream's
/// `velRef.setState(...)` for a spacecraft with no gravity body and no
/// effectors. It keeps `old`'s output ports.
fn with_velocity(old: &Spacecraft, velocity_mps: Vector3<f64>) -> Spacecraft {
    let state = old.state_out.read();
    let mut restarted = Spacecraft::new(SpacecraftConfig {
        mass_kg: old.config.mass_kg,
        hub_center_of_mass_body_m: old.config.hub_center_of_mass_body_m,
        inertia_kg_m2: old.config.inertia_kg_m2,
        integration_step_nanos: old.config.integration_step_nanos,
        initial_position_m: state.position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: state.sigma_bn,
        initial_omega_radps: state.omega_radps,
        integrator: None,
    });
    restarted.state_out = old.state_out.clone();
    restarted.mass_props_out = old.mass_props_out.clone();
    restarted.diagnostics_out = old.diagnostics_out.clone();
    restarted.init();
    restarted
}

fn run(config: Config) -> Results {
    let step = config.step_nanos();
    let mu = MU_EARTH * 1e9;
    let r_leo = REQ_EARTH * 1e3 + 500.0 * 1000.0;
    let (position_m, velocity_mps, omega_z, duration) = if config.multiple_planet {
        (
            Vector3::new(0.0, r_leo, 0.0),
            Vector3::zeros(),
            1.0 * D2R,
            2_000_000_000_000,
        )
    } else {
        let elements = ClassicElements {
            semi_major_axis: r_leo,
            eccentricity: 0.0001,
            true_anomaly_rad: 180.0 * D2R,
            ..Default::default()
        };
        let (r, v) = elements_to_state(mu, &elements);
        let period_s = 2.0 * std::f64::consts::PI / (mu / r_leo.powi(3)).sqrt();
        (r, v, 0.5 * D2R, (0.5 * period_s * 1e9 + 0.5) as u64)
    };
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: hub_inertia_kg_m2(),
        integration_step_nanos: step,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.0, 0.0, omega_z),
        integrator: None,
    });
    if !config.multiple_planet {
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass("earth", mu, true, Vector3::zeros(), Vector3::zeros())
                    .expect("valid point-mass Earth"),
            )
            .expect("unique central gravity body");
    }
    let mut spacecraft = Staged::new(spacecraft);
    if config.multiple_planet {
        // Upstream's `velRef.setState` at 500 s and 1500 s.
        spacecraft = spacecraft
            .after(500_000_000_000, |spacecraft, _| {
                *spacecraft = with_velocity(spacecraft, Vector3::new(0.0, TRANSFER_SPEED_MPS, 0.0));
            })
            .after(1_500_000_000_000, |spacecraft, _| {
                *spacecraft = with_velocity(spacecraft, Vector3::zeros());
            });
    }

    let sun = Output::new(SunEphemerisMsg {
        sun_position_inertial_m: Vector3::new(-AU * 1000.0, 0.0, 0.0),
        sun_velocity_inertial_mps: Vector3::zeros(),
    });
    let earth = Output::new(PlanetStateMsg {
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });
    let moon = Output::new(PlanetStateMsg {
        position_inertial_m: Vector3::new(0.0, MOON_DISTANCE_M, 0.0),
        orientation: Some(PlanetOrientation::identity()),
        ..PlanetStateMsg::default()
    });

    let mut eclipse = Eclipse::new(EclipseConfig {
        name: "eclipse".to_string(),
        occulting_body_radius_m: REQ_EARTH * 1e3,
    });
    let normals = [Vector3::x(), -Vector3::x(), -Vector3::y()];
    let mut sensors = vec![css("CSS1", normals[0])];
    if config.multiple_instrument {
        sensors.push(css("CSS2", normals[1]));
        sensors.push(css("CSS3", normals[2]));
    }

    // Upstream `addPlanetandAlbedoDataModel` (the CERES map) or
    // `addPlanetandAlbedoAverageModel(planet, 0.5, 200, 200)` for Earth, and
    // `addPlanetandAlbedoAverageModel(moon)`; one instrument per sensor.
    let mut albedo_config = AlbedoConfig {
        albedo_avg: 0.5,
        num_lat: 200,
        num_lon: 200,
        eclipse_case: config.use_eclipse,
        ..AlbedoConfig::earth(
            "AlbedoModule",
            sensors
                .iter()
                .zip(normals)
                .map(|(sensor, normal)| AlbedoInstrument {
                    fov_half_angle_rad: sensor.config.fov_half_angle_rad,
                    normal_body: normal,
                    position_body_m: Vector3::zeros(),
                })
                .collect(),
        )
    };
    if config.albedo_data {
        albedo_config.albedo_map = Some(
            load_albedo_map("assets/albedo/Earth_ALB_2018_CERES_All_5x5.csv")
                .expect("readable CERES albedo map (just fetch-albedo)"),
        );
    }
    if config.multiple_planet {
        albedo_config.other_planets.push(AlbedoPlanet {
            albedo_avg: 0.123,
            albedo_map: None,
            num_lat: 180,
            num_lon: 360,
            equatorial_radius_m: REQ_MOON * 1e3,
            polar_radius_m: -1.0,
        });
    }
    let mut albedo = Albedo::new(albedo_config);

    let mut css_logs: Vec<MessageLog<SunSensorMsg>> =
        sensors.iter().map(|_| MessageLog::default()).collect();
    let mut albedo_logs: Vec<MessageLog<AlbedoMsg>> =
        sensors.iter().map(|_| MessageLog::default()).collect();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut eclipse_log = MessageLog::<EclipseMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(&config.file_stem(), config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    let state_out = spacecraft.module.state_out.clone();
    connect!(&simulation, &earth => &mut albedo.input_planet_msg);
    if config.multiple_planet {
        connect!(&simulation, &moon => &mut albedo.input_other_planet_msgs[0]);
    }
    for (sensor, (albedo_out, (css_log, albedo_log))) in sensors.iter_mut().zip(
        albedo
            .output_albedo_msgs
            .iter()
            .zip(css_logs.iter_mut().zip(&mut albedo_logs)),
    ) {
        connect!(&simulation,
            &state_out => &mut sensor.input_state_msg,
            &sun => &mut sensor.input_sun_msg,
            albedo_out => &mut sensor.input_albedo_msg,
            &sensor.output_sun_sensor_msg => &mut css_log.input,
            albedo_out => &mut albedo_log.input,
        );
        if config.use_eclipse {
            connect!(&simulation, &eclipse.output_eclipse_msg => &mut sensor.input_eclipse_msg);
        }
    }
    connect!(&simulation,
        &state_out => &mut albedo.input_state_msg,
        &sun => &mut albedo.input_sun_msg,
        &state_out => &mut eclipse.input_state_msg,
        &sun => &mut eclipse.input_sun_msg,
        &state_out => &mut state_log.input,
        &eclipse.output_eclipse_msg => &mut eclipse_log.input,
        &state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m"),
        )),
    );
    for (index, sensor) in sensors.iter().enumerate() {
        connect!(&simulation,
            &sensor.output_sun_sensor_msg => recorder.add_source::<SunSensorMsg>(
                CsvSourceConfig::columns([(
                    "sensed_value".to_string(),
                    format!("css{}Signal", index + 1),
                )]),
            ),
            &albedo.output_albedo_msgs[index] => recorder.add_source::<AlbedoMsg>(
                CsvSourceConfig::columns([(
                    "albedo_at_instrument".to_string(),
                    format!("albedo{index}"),
                )]),
            ),
        );
    }
    // Upstream's task order: the spacecraft, the eclipse, the albedo, then
    // the sensors.
    simulation.add_module("spacecraft", &mut spacecraft, step, 100);
    if config.use_eclipse {
        simulation.add_module("eclipse", &mut eclipse, step, 90);
        simulation.add_module("eclipse_log", &mut eclipse_log, step, 0);
    }
    simulation.add_module("albedo", &mut albedo, step, 80);
    for sensor in &mut sensors {
        simulation.add_module("css", sensor as &mut dyn Module, step, 70);
    }
    for log in &mut css_logs {
        simulation.add_module("css_log", log, step, 0);
    }
    for log in &mut albedo_logs {
        simulation.add_module("albedo_log", log, step, 0);
    }
    simulation.add_module("state_log", &mut state_log, step, 0);
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, step, 0);
    }
    simulation.run_for(duration);
    drop(simulation);
    drop(recorder);

    Results {
        css: css_logs,
        albedo: albedo_logs,
        state: state_log,
        eclipse: config.use_eclipse.then_some(eclipse_log),
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_albedo [--[no-]albedo-data] [--[no-]multiple-instrument] \
         [--[no-]multiple-planet] [--[no-]eclipse] [--sim-time-step <s>] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        vec![Config::new(
            cli.flag("albedo-data", false),
            cli.flag("multiple-instrument", true),
            cli.flag("multiple-planet", false),
            cli.flag("eclipse", true),
            cli.value("sim-time-step", DEFAULT_TIME_STEP_S),
        )]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        let peak = results
            .albedo
            .iter()
            .flat_map(|log| log.values.iter())
            .fold(0.0_f64, |peak, sample| {
                peak.max(sample.albedo_at_instrument)
            });
        println!(
            "wrote {} (peak albedo at an instrument {peak:.4})",
            results.csv_path.expect("CSV written").display(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioAlbedo.py` only checks that each
    //! parametrization runs. These tests run the same parametrizations and
    //! check what they demonstrate: each sensor reads the Sun's cosine (in
    //! its field of view, shadowed by the eclipse when modelled) plus the
    //! albedo at it, limited to `[0, 1]`; the albedo is a fraction of the
    //! solar flux that shadowing only reduces; and on the way to the Moon the
    //! albedo fades with distance from Earth and returns near the Moon.

    use super::*;

    fn quiet(config: Config) -> Results {
        run(Config {
            write_csv: false,
            ..config
        })
    }

    /// The sensor's Sun cosine with an 80 deg field of view, at `state`.
    fn sun_cosine(state: &SpacecraftStateMsg, normal: Vector3<f64>) -> f64 {
        let to_sun_n = Vector3::new(-AU * 1000.0, 0.0, 0.0) - state.position_m;
        let distance_factor = (AU * 1000.0 / to_sun_n.norm()).powi(2);
        let cosine = normal.dot(
            &state
                .inertial_to_body()
                .transform_vector(&to_sun_n.normalize()),
        );
        if cosine >= (CSS_FOV_DEG * D2R).cos() {
            cosine * distance_factor
        } else {
            0.0
        }
    }

    #[test]
    fn every_sensor_reads_the_sun_plus_its_albedo() {
        let normals = [Vector3::x(), -Vector3::x(), -Vector3::y()];
        for config in TEST_PARAMETRIZATIONS {
            let results = quiet(config);
            for (index, (css, albedo)) in results.css.iter().zip(&results.albedo).enumerate() {
                for (sample, ((reading, albedo), state)) in css
                    .values
                    .iter()
                    .zip(&albedo.values)
                    .zip(&results.state.values)
                    .enumerate()
                {
                    let illumination = results
                        .eclipse
                        .as_ref()
                        .map_or(1.0, |log| log.values[sample].illumination_factor);
                    let expected = (sun_cosine(state, normals[index]) * illumination
                        + albedo.albedo_at_instrument)
                        .clamp(0.0, 1.0);
                    assert!(
                        (reading.sensed_value - expected).abs() < 1.0e-9,
                        "{config:?} CSS{}: {} vs {expected}",
                        index + 1,
                        reading.sensed_value
                    );
                }
            }
        }
    }

    #[test]
    fn the_albedo_is_a_fraction_of_the_solar_flux_and_shadowing_reduces_it() {
        let lit = quiet(TEST_PARAMETRIZATIONS[1]);
        let shadowed = quiet(TEST_PARAMETRIZATIONS[2]);
        let mut seen = false;
        for (lit, shadowed) in lit.albedo.iter().zip(&shadowed.albedo) {
            for (lit, shadowed) in lit.values.iter().zip(&shadowed.values) {
                assert!((0.0..1.0).contains(&lit.albedo_at_instrument));
                assert!(shadowed.albedo_at_instrument <= lit.albedo_at_instrument + 1.0e-15);
                seen |= lit.albedo_at_instrument > 0.01;
            }
        }
        assert!(seen, "no sensor ever sees Earth's albedo");
        // The CERES map gives a comparable albedo.
        let data = quiet(TEST_PARAMETRIZATIONS[0]);
        let peak = data.albedo[0].values.iter().fold(0.0_f64, |peak, sample| {
            peak.max(sample.albedo_at_instrument)
        });
        assert!((0.01..1.0).contains(&peak), "{peak}");
    }

    #[test]
    fn the_albedo_fades_on_the_way_to_the_moon_and_returns_near_it() {
        let results = quiet(TEST_PARAMETRIZATIONS[3]);
        let total = |index: usize| -> f64 {
            results
                .albedo
                .iter()
                .map(|log| log.values[index].albedo_at_instrument_max)
                .sum()
        };
        let times = results.state.times_s();
        let at = |time_s: f64| {
            times
                .iter()
                .position(|t| (*t - time_s).abs() < 1.0)
                .unwrap()
        };
        // Parked at 500 km, moving, then parked 485 km above the Moon.
        assert!(results.state.values[at(400.0)].velocity_mps.norm() < 1.0e-9);
        assert!(
            (results.state.values[at(1000.0)].velocity_mps.y - TRANSFER_SPEED_MPS).abs() < 1.0e-6
        );
        assert!(results.state.last().velocity_mps.norm() < 1.0e-9);
        let near_moon =
            (results.state.last().position_m - Vector3::new(0.0, MOON_DISTANCE_M, 0.0)).norm();
        assert!(
            (near_moon - 1737.4e3 - 485.0e3).abs() < 30.0e3,
            "{near_moon}"
        );
        assert!(total(at(400.0)) > 10.0 * total(at(1000.0)));
        assert!(total(at(1900.0)) > 10.0 * total(at(1000.0)));
    }
}
