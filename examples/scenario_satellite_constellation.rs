//! A Walker-delta constellation of nadir-pointed satellites: port of upstream
//! `examples/scenarioSatelliteConstellation.py`.
//!
//! `T` satellites (750 kg, inertia `diag(900, 800, 600)` kg m^2) in `P`
//! equally spaced orbit planes with phasing `F` (the Walker `i: T/P/F`
//! pattern, `e = 0.01`) about the point-mass Earth, from
//! `2029 June 12 5:30:30.0` (UTC). Each has `simpleNav` and a
//! `locationPointing` that turns its body +x axis to the Earth's center (the
//! Earth's ephemeris through `ephemerisConverter`); the spacecraft takes the
//! guidance's reference attitude as its own (upstream `attRefInMsg`), so the
//! attitude is prescribed rather than controlled. One 30 s task for 1.5
//! orbit periods, each state sampled 500 times.
//!
//! Upstream `run(show_plots, a, i, T, P, F)` maps to `--a <m>`,
//! `--inclination <deg>`, `--total <T>`, `--planes <P>` and `--phasing <F>`;
//! the defaults and `--all` run upstream's test case, the GPS-like
//! `56 deg: 24/3/1` at `a = 29 994 km`.
//!
//! Mapping to basilisk-rs:
//!
//! * upstream's `spiceInterface` with `zeroBase = 'Earth'` (only the Earth)
//!   is an ANISE ephemeris of the Earth relative to itself, run first and
//!   at initialization ([`UpdatedAtInit`]);
//! * upstream's `scList[i].attRefInMsg` is the spacecraft's
//!   `att_ref_in_msg`, which replaces the integrated hub attitude and rate
//!   with the reference after each step, as upstream.
//!
//! Writes `examples/output/scenarios/scenarioSatelliteConstellation.csv`
//! with `sat<n>_r_BN_N_m_<i>` per satellite, at upstream's sampling times.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::{HUB_MASS_KG, hub_inertia_kg_m2, scenario_recorder};
use basilisk_rs::environment::ephemeris_converter::{EphemerisConverter, EphemerisConverterConfig};
use basilisk_rs::fsw_algorithms::att_guidance::location_pointing::{
    LocationPointing, LocationPointingConfig,
};
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::D2R;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{Module, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use orbit_tools::{orbit_period_s, sec2nano};
use recording::sampling_time;
use staged::UpdatedAtInit;

/// Upstream `simulationTimeStep = macros.sec2nano(30.)`.
const TASK_PERIOD_NANOS: u64 = 30_000_000_000;
/// Upstream `numDataPoints`.
const DATA_POINTS: u64 = 500;
/// Upstream `oe.e` of every satellite.
const ECCENTRICITY: f64 = 0.01;

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `a` \[m\]: semi-major axis of all satellites.
    semi_major_axis_m: f64,
    /// Upstream `i` \[rad\]: orbit inclination.
    inclination_rad: f64,
    /// Upstream `T`: total number of satellites.
    total: usize,
    /// Upstream `P`: number of equally spaced orbit planes.
    planes: usize,
    /// Upstream `F`: phasing between satellites in adjacent planes.
    phasing: usize,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The parametrization of upstream `test_scenarioSatelliteConstellation.py`
/// (and its `__main__`).
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config {
    semi_major_axis_m: 29_994_000.0,
    inclination_rad: 56.0 * D2R,
    total: 24,
    planes: 3,
    phasing: 1,
    write_csv: true,
}];

impl Config {
    /// Upstream's checks: positive integers, `T` a multiple of `P`.
    fn validate(&self) -> Result<(), String> {
        if self.total < 1 || self.planes < 1 || self.phasing < 1 {
            return Err(
                "number of satellites T, number of planes P, and relative spacing F must be \
                 positive integer numbers"
                    .to_string(),
            );
        }
        if !self.total.is_multiple_of(self.planes) {
            return Err("number of satellites T must be an integer multiple of P".to_string());
        }
        Ok(())
    }

    /// Satellite `index`'s elements: upstream's Walker pattern unit
    /// `PU = 360 / T`, `S = T / P` satellites per plane, planes `S PU` apart
    /// in RAAN, satellites `P PU` apart in a plane, and the argument of
    /// periapsis `F` degrees per plane.
    fn elements(&self, index: usize) -> ClassicElements {
        let pattern_unit_deg = 360.0 / self.total as f64;
        let per_plane = self.total / self.planes;
        let plane = (index / per_plane) as f64;
        ClassicElements {
            semi_major_axis: self.semi_major_axis_m,
            eccentricity: ECCENTRICITY,
            inclination_rad: self.inclination_rad,
            raan_rad: per_plane as f64 * pattern_unit_deg * plane * D2R,
            arg_periapsis_rad: self.phasing as f64 * plane * D2R,
            true_anomaly_rad: self.planes as f64
                * pattern_unit_deg
                * (index % per_plane) as f64
                * D2R,
            ..Default::default()
        }
    }
}

/// Upstream `time="2029 June 12 5:30:30.0"`.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2029, 6, 12, 5, 30, 30, 0)
}

/// One satellite: upstream `scList[i]`, `navList[i]` and `guideList[i]`.
struct Satellite {
    spacecraft: Spacecraft,
    navigation: SimpleNavigation,
    pointing: LocationPointing,
}

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `dataRec`, one per satellite.
    states: Vec<MessageLog<SpacecraftStateMsg>>,
    duration_nanos: u64,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    if let Err(message) = config.validate() {
        panic!("{message}");
    }
    let mu = Body::Earth.mu_m3ps2();
    let mut ephemeris = UpdatedAtInit::new(Body::Earth.ephemeris(Body::Earth), epoch());
    let mut ephemeris_converter = EphemerisConverter::new(EphemerisConverterConfig {
        name: "ephemData".to_string(),
        body_names: vec!["earth".to_string()],
    });
    ephemeris
        .module
        .output_planet_msg
        .connect_to(&mut ephemeris_converter.planet_in_msgs[0]);
    let duration_nanos = sec2nano(1.5 * orbit_period_s(mu, config.semi_major_axis_m));
    let sample_nanos = sampling_time(duration_nanos, TASK_PERIOD_NANOS, DATA_POINTS);

    let mut satellites: Vec<Satellite> = (0..config.total)
        .map(|index| {
            let (position_m, velocity_mps) = elements_to_state(mu, &config.elements(index));
            let mut spacecraft = Spacecraft::new(SpacecraftConfig {
                mass_kg: HUB_MASS_KG,
                hub_center_of_mass_body_m: Vector3::zeros(),
                inertia_kg_m2: hub_inertia_kg_m2(),
                integration_step_nanos: TASK_PERIOD_NANOS,
                initial_position_m: position_m,
                initial_velocity_mps: velocity_mps,
                initial_sigma_bn: Vector3::zeros(),
                initial_omega_radps: Vector3::zeros(),
                integrator: None,
            });
            spacecraft
                .add_grav_body(Body::Earth.point_mass(true))
                .expect("one central gravity body");
            let mut navigation = SimpleNavigation::new(format!("SimpleNav{index}"));
            let mut pointing = LocationPointing::new(LocationPointingConfig {
                name: format!("nadirPoint{index}"),
                p_hat_b: Vector3::x(),
                ..LocationPointingConfig::default()
            });
            spacecraft
                .state_out
                .connect_to(&mut navigation.spacecraft_state_in_msg);
            navigation
                .translation_out_msg
                .connect_to(&mut pointing.sc_trans_in_msg);
            navigation
                .attitude_out_msg
                .connect_to(&mut pointing.sc_att_in_msg);
            ephemeris_converter.ephem_out_msgs[0].connect_to(&mut pointing.cel_body_in_msg);
            pointing
                .att_ref_out_msg
                .connect_to(&mut spacecraft.att_ref_in);
            Satellite {
                spacecraft,
                navigation,
                pointing,
            }
        })
        .collect();

    let mut logs: Vec<MessageLog<SpacecraftStateMsg>> =
        satellites.iter().map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioSatelliteConstellation", config.write_csv);
    for (index, (satellite, log)) in satellites.iter().zip(&mut logs).enumerate() {
        let state = &satellite.spacecraft.state_out;
        state.connect_to(&mut log.input);
        state.connect_to(
            recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(vector_columns(
                "position_m",
                &format!("sat{index}_r_BN_N_m"),
            ))),
        );
    }

    let mut simulation = Simulation::new(epoch(), false);
    // Upstream's task order: the ephemeris and its converter, then each
    // satellite's spacecraft, navigation, guidance and recorder in turn
    // (the satellites are independent, so they are grouped by module here).
    schedule! { simulation,
        "spice" => &mut ephemeris, period=TASK_PERIOD_NANOS, priority=100;
        "ephemData" => &mut ephemeris_converter, period=TASK_PERIOD_NANOS, priority=90;
    }
    for satellite in &mut satellites {
        let Satellite {
            spacecraft,
            navigation,
            pointing,
        } = satellite;
        simulation.add_module(
            "spacecraft",
            spacecraft as &mut dyn Module,
            TASK_PERIOD_NANOS,
            50,
        );
        simulation.add_module(
            "simpleNav",
            navigation as &mut dyn Module,
            TASK_PERIOD_NANOS,
            40,
        );
        simulation.add_module(
            "nadirPoint",
            pointing as &mut dyn Module,
            TASK_PERIOD_NANOS,
            30,
        );
    }
    for log in &mut logs {
        simulation.add_module("dataRec", log, sample_nanos, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, sample_nanos, 0);
    }
    simulation.run_for(duration_nanos);
    drop(simulation);
    drop(recorder);

    Results {
        states: logs,
        duration_nanos,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env(
        "scenario_satellite_constellation [--a <m>] [--inclination <deg>] [--total <T>] \
         [--planes <P>] [--phasing <F>] [--all]",
    );
    let configs = if cli.switch("all") {
        TEST_PARAMETRIZATIONS.to_vec()
    } else {
        let default = TEST_PARAMETRIZATIONS[0];
        vec![Config {
            semi_major_axis_m: cli.value("a", default.semi_major_axis_m),
            inclination_rad: cli.value("inclination", 56.0) * D2R,
            total: cli.value("total", default.total),
            planes: cli.value("planes", default.planes),
            phasing: cli.value("phasing", default.phasing),
            write_csv: true,
        }]
    };
    cli.finish();

    for config in configs {
        let results = run(config);
        println!(
            "wrote {} ({} satellites over {:.2} h)",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.states.len(),
            results.duration_nanos as f64 / 3.6e12,
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioSatelliteConstellation.py` only checks that
    //! the scenario runs. These tests check the Walker pattern it sets up,
    //! that it keeps its shape (two-body motion), and that every satellite
    //! points its +x axis at the Earth.

    use basilisk_rs::utilities::orbital_motion::state_to_elements;
    use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;

    use super::*;

    fn quiet() -> Results {
        run(Config {
            write_csv: false,
            ..TEST_PARAMETRIZATIONS[0]
        })
    }

    /// `|a - b|` as angles, in `[0, pi]`.
    fn angle_between(a: f64, b: f64) -> f64 {
        let difference = a - b;
        difference.sin().atan2(difference.cos()).abs()
    }

    #[test]
    fn the_constellation_is_a_walker_delta_pattern() {
        let config = TEST_PARAMETRIZATIONS[0];
        let results = quiet();
        let mu = Body::Earth.mu_m3ps2();
        let first: Vec<ClassicElements> = results
            .states
            .iter()
            .map(|log| {
                let state = &log.values[0];
                state_to_elements(mu, state.position_m, state.velocity_mps)
            })
            .collect();
        for (index, elements) in first.iter().enumerate() {
            assert!((elements.semi_major_axis - config.semi_major_axis_m).abs() < 1e-3);
            assert!((elements.inclination_rad - 56.0 * D2R).abs() < 1e-12);
            // Three planes 120 deg apart in RAAN, eight satellites 45 deg
            // apart in each, the planes' perigees 1 deg apart.
            let plane = index / 8;
            assert!(angle_between(elements.raan_rad, plane as f64 * 120.0 * D2R) < 1e-9);
            let argument_of_latitude = elements.arg_periapsis_rad + elements.true_anomaly_rad;
            let expected = plane as f64 * D2R + (index % 8) as f64 * 45.0 * D2R;
            assert!(
                angle_between(argument_of_latitude, expected) < 1e-9,
                "satellite {index}"
            );
        }
    }

    #[test]
    fn the_pattern_keeps_its_shape() {
        let results = quiet();
        let mu = Body::Earth.mu_m3ps2();
        // Two-body motion: every orbit keeps its elements over 1.5 periods.
        for log in &results.states {
            let first = &log.values[0];
            let last = log.last();
            let start = state_to_elements(mu, first.position_m, first.velocity_mps);
            let end = state_to_elements(mu, last.position_m, last.velocity_mps);
            assert!((end.semi_major_axis - start.semi_major_axis).abs() < 1.0);
            assert!((end.eccentricity - start.eccentricity).abs() < 1e-9);
            assert!(angle_between(end.raan_rad, start.raan_rad) < 1e-9);
        }
    }

    #[test]
    fn every_satellite_points_its_x_axis_at_the_earth() {
        let results = quiet();
        // The prescribed attitude is the reference computed one 30 s step
        // earlier: off nadir by about the orbit's motion in a step, n dt
        // (0.2 deg). At t = 0 the reference is not yet written.
        for log in &results.states {
            for state in &log.values[1..] {
                let x_n = mrp_to_dcm(state.sigma_bn).transpose() * Vector3::x();
                let off_nadir = x_n.angle(&(-state.position_m));
                assert!(off_nadir < 0.5 * D2R, "{} deg off nadir", off_nadir / D2R);
            }
        }
    }
}
