//! A spacecraft's heliocentric trajectory from a custom SPK file: port of
//! upstream `examples/scenarioHelioTransSpice.py`.
//!
//! Upstream's `spacecraft_21T01.bsp` holds the trajectory of spacecraft
//! `-60000` from 2028 to 2034, with the Venus, Earth and Mars flybys of
//! `scenarioFlybySpice`, out to 2.65 AU from the Sun. Loaded next to
//! the planetary ephemeris, it prescribes the translation of a 750 kg
//! spacecraft through its `transRefInMsg` (relative to the solar-system
//! barycenter, upstream's default `zeroBase`), with no orbit elements set.
//! The Earth, Sun, Venus and Mars barycenter are point-mass gravity bodies
//! on their ephemerides; none is central. The attitude starts at
//! `sigma_BN = [0.1, 0.2, -0.3]` at rest. One 5 h task from
//! 2028 February 24 05:30:30 UTC; upstream runs to half a year, changes a
//! Vizard orbit-line colour, and runs on to 4.5 years.
//!
//! Upstream's `spiceInterface` reads the SPK with CSPICE. The SPK is of type
//! 13, which ANISE reads, so here the trajectory and the planets come from
//! ANISE with DE440s ([`ephemeris::SpkSpacecraft`],
//! [`grav_factory::Body::ephemeris_from_ssb`]) and the default build runs the
//! scenario. Upstream's purpose is the Vizard view, whose settings are not
//! ported; the trajectory is written to CSV instead.
//!
//! Upstream `run()` has no parameters. Reads
//! `assets/spice/spacecraft_21T01.bsp` (`just fetch-misc-scenario-data`);
//! writes `examples/output/scenarios/scenarioHelioTransSpice.csv` (the
//! spacecraft and planet positions, about 1000 samples).

#[path = "support/att_scenario.rs"]
mod att_scenario;
#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/ephemeris.rs"]
mod ephemeris;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/recording.rs"]
mod recording;

use std::path::PathBuf;

use anise::constants::frames::SSB_J2000;
use att_scenario::rigid_spacecraft;
use attitude::{HubState, hub_inertia_kg_m2, sampling_period_nanos, scenario_recorder};
use basilisk_rs::messages::{PlanetStateMsg, SpacecraftStateMsg, TranslationReferenceMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use ephemeris::{SPACECRAFT_21T01_ID, SPACECRAFT_21T01_SPK, SpkSpacecraft};
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use recording::planet_columns;

/// Upstream `simulationTimeStep = macros.sec2nano(5 * 60 * 60.)`.
const STEP_NANOS: u64 = 5 * 3_600_000_000_000;
const DAY_NANOS: u64 = 86_400_000_000_000;
/// Upstream's first `ConfigureStopTime(0.5 * 365 * day)`.
const FIRST_STOP_NANOS: u64 = 365 * DAY_NANOS / 2;
/// Upstream's second `ConfigureStopTime(4.5 * 365 * day)`.
const DURATION_NANOS: u64 = 9 * 365 * DAY_NANOS / 2;
/// Upstream `gravFactory.createBodies("earth", "sun", "venus", "mars barycenter")`.
const BODIES: [Body; 4] = [Body::Earth, Body::Sun, Body::Venus, Body::MarsBarycenter];

/// Upstream `timeInitString = "2028 February 24 5:30:30.0"` (UTC).
fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2028, 2, 24, 5, 30, 30, 0)
}

// Some histories are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    state: MessageLog<SpacecraftStateMsg>,
    reference: MessageLog<TranslationReferenceMsg>,
    sun: MessageLog<PlanetStateMsg>,
    csv_path: Option<PathBuf>,
}

fn run(write_csv: bool) -> Results {
    let initial = HubState {
        position_m: Vector3::zeros(),
        velocity_mps: Vector3::zeros(),
        sigma_bn: Vector3::new(0.1, 0.2, -0.3),
        omega_bn_b_radps: Vector3::zeros(),
    };
    let mut spacecraft = rigid_spacecraft(hub_inertia_kg_m2(), &initial, STEP_NANOS);
    let mut ephemerides = Vec::new();
    for body in BODIES {
        spacecraft
            .add_grav_body(body.point_mass(false))
            .expect("unique gravity bodies");
        ephemerides.push(body.ephemeris_from_ssb());
    }
    let mut trajectory = SpkSpacecraft::new(
        "-60000",
        SPACECRAFT_21T01_ID,
        SPACECRAFT_21T01_SPK,
        SSB_J2000,
    );

    let sample_period = sampling_period_nanos(DURATION_NANOS, STEP_NANOS, 1000);
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut reference_log = MessageLog::<TranslationReferenceMsg>::default();
    let mut sun_log = MessageLog::<PlanetStateMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder("scenarioHelioTransSpice", write_csv);

    let mut simulation = Simulation::new(start_epoch(), false);
    for (body, ephemeris) in BODIES.iter().zip(&ephemerides) {
        simulation.connect(
            &ephemeris.output_planet_msg,
            spacecraft
                .grav_body_input_mut(body.name())
                .expect("gravity body"),
        );
        connect!(&simulation,
            &ephemeris.output_planet_msg => recorder.add_source::<PlanetStateMsg>(
                planet_columns(&body.name().replace(' ', "_")),
            ),
        );
    }
    connect!(&simulation,
        &trajectory.trans_ref_out_msg => &mut spacecraft.trans_ref_in,
        &spacecraft.state_out => &mut state_log.input,
        &trajectory.trans_ref_out_msg => &mut reference_log.input,
        &ephemerides[1].output_planet_msg => &mut sun_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s"))
                    .chain(vector_columns("sigma_bn", "sigma_BN")),
            ),
        ),
    );
    for (body, ephemeris) in BODIES.iter().zip(&mut ephemerides) {
        simulation.add_module(
            format!("{}_ephemeris", body.name()),
            ephemeris,
            STEP_NANOS,
            120,
        );
    }
    // Upstream's task order: spiceInterface, then the spacecraft.
    schedule! { simulation,
        "spiceInterface" => &mut trajectory, period=STEP_NANOS, priority=110;
        "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "state_log" => &mut state_log, period=sample_period, priority=0;
        "reference_log" => &mut reference_log, period=sample_period, priority=0;
        "sun_log" => &mut sun_log, period=sample_period, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, sample_period, 0);
    }
    simulation.run_for(FIRST_STOP_NANOS);
    simulation.run_for(DURATION_NANOS - FIRST_STOP_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        state: state_log,
        reference: reference_log,
        sun: sun_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_helio_trans_spice [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();
    let results = run(true);
    let au_m = 1.495_978_707e11;
    let heliocentric = |state: &SpacecraftStateMsg, sun: &PlanetStateMsg| {
        (state.position_m - sun.position_inertial_m).norm() / au_m
    };
    let distances: Vec<f64> = results
        .state
        .values
        .iter()
        .zip(&results.sun.values)
        .map(|(state, sun)| heliocentric(state, sun))
        .collect();
    println!(
        "wrote {} (heliocentric distance {:.3} to {:.3} AU)",
        results.csv_path.expect("CSV written").display(),
        distances.iter().copied().fold(f64::INFINITY, f64::min),
        distances.iter().copied().fold(0.0, f64::max),
    );
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioHelioTransSpice.py` only checks that the
    //! scenario runs. These tests check what it demonstrates: the spacecraft
    //! flies the SPK trajectory, an interplanetary one, for the whole run.

    use super::*;
    use ephemeris::{almanac, j2000_frame, spk_read};

    #[test]
    fn the_spacecraft_flies_the_spk_trajectory() {
        let results = run(false);
        // 4.5 years of 5 h steps, sampled every 7 steps.
        assert_eq!(results.state.values.len(), 7884 / 7 + 1);
        let almanac = almanac(&[SPACECRAFT_21T01_SPK]);
        let au_m = 1.495_978_707e11;
        let mut distances = Vec::new();
        for (index, ((time, state), reference)) in results
            .state
            .times_nanos
            .iter()
            .zip(&results.state.values)
            .zip(&results.reference.values)
            .enumerate()
        {
            assert_eq!(state.position_m, reference.position_m);
            assert_eq!(state.velocity_mps, reference.velocity_mps);
            // Torque free and at rest: the attitude is unchanged.
            assert!((state.sigma_bn - Vector3::new(0.1, 0.2, -0.3)).norm() < 1e-12);
            if index % 100 == 0 {
                let epoch =
                    start_epoch() + hifitime::Duration::from_total_nanoseconds(*time as i128);
                let (position, velocity) =
                    spk_read(&almanac, j2000_frame(SPACECRAFT_21T01_ID), SSB_J2000, epoch);
                assert!((state.position_m - position).norm() < 1e-3);
                assert!((state.velocity_mps - velocity).norm() < 1e-9);
            }
            distances.push(
                (state.position_m - results.sun.values[index].position_inertial_m).norm() / au_m,
            );
        }
        // In to the Venus orbit and out beyond Mars's: 0.72 to 2.65 AU.
        let nearest = distances.iter().copied().fold(f64::INFINITY, f64::min);
        let farthest = distances.iter().copied().fold(0.0, f64::max);
        assert!((0.7..0.75).contains(&nearest), "{nearest}");
        assert!((2.6..2.7).contains(&farthest), "{farthest}");
    }
}
