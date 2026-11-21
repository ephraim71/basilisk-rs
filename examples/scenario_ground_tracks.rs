//! Five spacecraft on different Earth orbits, for Vizard's orbit and ground
//! track lines: port of upstream `examples/scenarioGroundTracks.py`.
//!
//! Five spacecraft at upstream's hub defaults (1 kg, identity inertia) fly
//! a LEO (7000 km, `i = 33.3 deg`), a slightly inclined GEO, a GTO, a
//! hyperbolic flyby (`a = -40 000 km`, `e = 1.2`, from `f = -120 deg`) and a
//! Molniya orbit about the point-mass Earth, perturbed by the Sun and the
//! Moon, from `2012 MAY 1 00:28:30.0` (UTC). One 10 s task for two Molniya
//! periods (about 23.5 hours), each spacecraft's state sampled 300 times.
//!
//! Upstream `run(show_plots)` has no parameters; `--all` is accepted and runs
//! the same single case. The orbit and ground track lines themselves are
//! Vizard settings (`osculatingOrbitLineRange`,
//! `showOsculatingGroundTrackLines`, ...) and are not ported; the scenario's
//! figure is the orbit radii.
//!
//! Mapping to basilisk-rs: upstream's `spiceInterface` with
//! `zeroBase = 'Earth'` is ANISE ephemerides of the Sun and the Moon relative
//! to the Earth ([`Body::ephemeris`]), run before the spacecraft, which take
//! the bodies' gravity from them, and at initialization ([`UpdatedAtInit`]).
//!
//! Writes `examples/output/scenarios/scenarioGroundTracks.csv` with
//! `<orbit>_r_BN_N_m_<i>` for `LEO`, `GEO`, `GTO`, `Hyperbolic` and
//! `Molniya`.

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

use attitude::scenario_recorder;
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::astro_constants::D2R;
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{Module, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::sampling_time;
use staged::UpdatedAtInit;

/// Upstream `simulationTimeStep = macros.sec2nano(10.)`.
const TASK_PERIOD_NANOS: u64 = 10_000_000_000;
/// Upstream `numDataPoints`.
const DATA_POINTS: u64 = 300;
/// Upstream `rLEO` and `rGEO` \[m\].
const R_LEO_M: f64 = 7000.0 * 1000.0;
const R_GEO_M: f64 = 42164.0 * 1000.0;

/// Upstream's spacecraft, in `scList` order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Orbit {
    Leo,
    Geo,
    Gto,
    Hyperbolic,
    Molniya,
}

impl Orbit {
    const ALL: [Self; 5] = [
        Self::Leo,
        Self::Geo,
        Self::Gto,
        Self::Hyperbolic,
        Self::Molniya,
    ];

    /// The CSV column prefix (upstream `ModelTag` without `bsk-`).
    fn label(self) -> &'static str {
        match self {
            Self::Leo => "LEO",
            Self::Geo => "GEO",
            Self::Gto => "GTO",
            Self::Hyperbolic => "Hyperbolic",
            Self::Molniya => "Molniya",
        }
    }

    /// Upstream's elements for this spacecraft.
    fn elements(self) -> ClassicElements {
        let (semi_major_axis, eccentricity, inclination, raan, arg_periapsis, anomaly) = match self
        {
            Self::Geo => (R_GEO_M, 0.00001, 10.0, 0.0, 0.0, 85.3),
            Self::Gto => {
                let a = (R_LEO_M + R_GEO_M) / 2.0;
                (a, 1.0 - R_LEO_M / a, 0.0, 148.2, 47.8, 85.3)
            }
            Self::Molniya => (26000.0 * 1000.0, 0.74, 63.4, 48.2, 275.0, 85.3),
            Self::Hyperbolic => (-40000.0 * 1000.0, 1.2, 33.0, 32.0, 275.0, -120.0),
            Self::Leo => (R_LEO_M, 0.0001, 33.3, 48.2, 347.8, 85.3),
        };
        ClassicElements {
            semi_major_axis,
            eccentricity,
            inclination_rad: inclination * D2R,
            raan_rad: raan * D2R,
            arg_periapsis_rad: arg_periapsis * D2R,
            true_anomaly_rad: anomaly * D2R,
            ..Default::default()
        }
    }
}

/// Upstream `run(show_plots)`: no parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The single run of upstream's `test_scenarioTest.py` entry.
const TEST_PARAMETRIZATIONS: [Config; 1] = [Config { write_csv: true }];

/// Upstream `timeInitString = "2012 MAY 1 00:28:30.0"`.
fn epoch() -> Epoch {
    Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0)
}

/// The recorded histories.
// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Upstream `dataList`, one per spacecraft in [`Orbit::ALL`] order.
    states: Vec<MessageLog<SpacecraftStateMsg>>,
    /// Upstream `oe` after `rv2elem`, per spacecraft.
    initial_elements: Vec<ClassicElements>,
    duration_nanos: u64,
    csv_path: Option<PathBuf>,
}

fn run(config: Config) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let mut sun = UpdatedAtInit::new(Body::Sun.ephemeris(Body::Earth), epoch());
    let mut moon = UpdatedAtInit::new(Body::Moon.ephemeris(Body::Earth), epoch());

    let mut initial_elements = Vec::new();
    let mut spacecraft: Vec<Spacecraft> = Orbit::ALL
        .iter()
        .map(|orbit| {
            let (position_m, velocity_mps) = elements_to_state(mu, &orbit.elements());
            // Upstream stores the consistent elements of the state.
            initial_elements.push(state_to_elements(mu, position_m, velocity_mps));
            let mut spacecraft = Spacecraft::new(SpacecraftConfig {
                mass_kg: 1.0,
                hub_center_of_mass_body_m: Vector3::zeros(),
                inertia_kg_m2: Matrix3::identity(),
                integration_step_nanos: TASK_PERIOD_NANOS,
                initial_position_m: position_m,
                initial_velocity_mps: velocity_mps,
                initial_sigma_bn: Vector3::zeros(),
                initial_omega_radps: Vector3::zeros(),
                integrator: None,
            });
            for (body, central) in [(Body::Earth, true), (Body::Sun, false), (Body::Moon, false)] {
                spacecraft
                    .add_grav_body(body.point_mass(central))
                    .expect("unique gravity body");
            }
            for (body, ephemeris) in [(Body::Sun, &sun), (Body::Moon, &moon)] {
                ephemeris.module.output_planet_msg.connect_to(
                    spacecraft
                        .grav_body_input_mut(body.name())
                        .expect("a gravity body"),
                );
            }
            spacecraft
        })
        .collect();
    // Upstream's `P` is that of the last spacecraft set up, the Molniya.
    let molniya = initial_elements.last().expect("five spacecraft");
    let duration_nanos = sec2nano(2.0 * orbit_period_s(mu, molniya.semi_major_axis));
    let sample_nanos = sampling_time(duration_nanos, TASK_PERIOD_NANOS, DATA_POINTS);

    let mut logs: Vec<MessageLog<SpacecraftStateMsg>> =
        Orbit::ALL.iter().map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder("scenarioGroundTracks", config.write_csv);
    for ((orbit, spacecraft), log) in Orbit::ALL.iter().zip(&spacecraft).zip(&mut logs) {
        spacecraft.state_out.connect_to(&mut log.input);
        spacecraft
            .state_out
            .connect_to(
                recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
                    vector_columns("position_m", &format!("{}_r_BN_N_m", orbit.label())),
                )),
            );
    }

    let mut simulation = Simulation::new(epoch(), false);
    // Upstream adds the spacecraft before the ephemeris; it runs first here
    // so the gravity bodies see the current update's states.
    schedule! { simulation,
        "sun" => &mut sun, period=TASK_PERIOD_NANOS, priority=100;
        "moon" => &mut moon, period=TASK_PERIOD_NANOS, priority=100;
    }
    for spacecraft in &mut spacecraft {
        simulation.add_module(
            "spacecraft",
            spacecraft as &mut dyn Module,
            TASK_PERIOD_NANOS,
            50,
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
        initial_elements,
        duration_nanos,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_ground_tracks [--all]");
    cli.switch("all");
    cli.finish();

    for config in TEST_PARAMETRIZATIONS {
        let results = run(config);
        println!(
            "wrote {} ({:.2} h, {} samples per spacecraft)",
            results.csv_path.as_ref().expect("CSV written").display(),
            results.duration_nanos as f64 / 3.6e12,
            results.states[0].values.len(),
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream runs this scenario in `test_scenarioTest.py`, which only
    //! checks that it runs. These tests check that each spacecraft keeps to
    //! its conic: the Sun and the Moon only perturb it.

    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    use super::*;

    #[test]
    fn each_spacecraft_keeps_to_its_conic() {
        let results = run(Config { write_csv: false });
        let mu = Body::Earth.mu_m3ps2();
        // Two Molniya periods, sampled by upstream's `samplingTime`.
        assert!((83_000e9..84_000e9).contains(&(results.duration_nanos as f64)));
        let sample_nanos = sampling_time(results.duration_nanos, TASK_PERIOD_NANOS, DATA_POINTS);
        for ((orbit, log), initial) in Orbit::ALL
            .iter()
            .zip(&results.states)
            .zip(&results.initial_elements)
        {
            assert_eq!(
                log.values.len() as u64,
                results.duration_nanos / sample_nanos + 1,
                "{orbit:?}"
            );
            let a = initial.semi_major_axis;
            let e = initial.eccentricity;
            for state in &log.values {
                let elements = state_to_elements(mu, state.position_m, state.velocity_mps);
                // Luni-solar perturbations change the osculating size and
                // shape of the orbit by well under a percent in a day.
                let relative_a = (elements.semi_major_axis - a).abs() / a.abs();
                assert!(relative_a < 2e-3, "{orbit:?}: a off by {relative_a}");
                assert!((elements.eccentricity - e).abs() < 2e-3, "{orbit:?}");
                let radius = state.position_m.norm();
                let periapsis = a * (1.0 - e);
                assert!(radius > 0.99 * periapsis, "{orbit:?} below periapsis");
                if e < 1.0 {
                    assert!(radius < 1.01 * a * (1.0 + e), "{orbit:?} above apoapsis");
                }
            }
        }
        // The hyperbolic flyby passes periapsis and leaves.
        let flyby = &results.states[3].values;
        let radii: Vec<f64> = flyby.iter().map(|state| state.position_m.norm()).collect();
        let closest = radii
            .iter()
            .enumerate()
            .min_by(|a, b| a.1.total_cmp(b.1))
            .map(|(index, _)| index)
            .expect("samples");
        assert!(closest > 0 && closest < radii.len() / 2);
        assert!(radii.windows(2).skip(closest).all(|pair| pair[1] > pair[0]));
        assert!(*radii.last().expect("samples") > 10.0 * radii[closest]);
    }
}
