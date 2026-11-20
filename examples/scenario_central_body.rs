//! Port of upstream `examples/scenarioCentralBody.py`: the same LEO orbit
//! propagated relative to the Earth as central body, or in the
//! solar-system-barycentric frame with no central body.
//!
//! Earth and Sun gravity follow the DE440s ephemeris (upstream
//! `createSpiceInterface` with DE430, here the ANISE ephemeris, see
//! `support/grav_factory.rs`) from 2012 May 1 00:28:30 UTC. With
//! `--use-central` the hub state is Earth-relative and Earth is the central
//! body; without it the hub starts at the Earth-relative state plus Earth's
//! barycentric state (upstream `planetStates.planetPositionVelocity`).
//!
//! Writes `scenarioCentralBody<0|1>.csv` (the spacecraft and Earth states);
//! `--all` runs both cases of `test_scenarioCentralBody.py`, whose check is
//! ported in `tests`: the final Earth-relative radius and speed are within 1 %
//! of the initial ones.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use anise::constants::frames::SSB_J2000;
use basilisk_rs::messages::{PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::{connect, schedule};
use grav_factory::{Body, spk_state};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, sec2nano};
use recording::{History, planet_columns, sampling_time, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_central_body [--use-central]
scenario_central_body --all     run both upstream test cases";

const STEP_NANOS: u64 = 10_000_000_000;

/// Upstream `UTCInit = "2012 MAY 1 00:28:30.0"`.
fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0)
}

pub struct Results {
    /// Upstream `out_r`, `out_v`: final Earth-relative radius (m) and speed (m/s).
    pub final_radius_m: f64,
    pub final_speed_mps: f64,
    /// Upstream `truth_r`, `truth_v`: the initial ones.
    pub initial_radius_m: f64,
    pub initial_speed_mps: f64,
    /// Earth-relative spacecraft positions (m) at the samples.
    pub relative_positions_m: Vec<Vector3<f64>>,
    pub elements: ClassicElements,
}

/// Upstream `run(show_plots, useCentral)`.
pub fn run(use_central: bool) -> Results {
    let mu = Body::Earth.mu_m3ps2();
    let (position_m, velocity_mps) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: 7_000.0 * 1_000.0,
            eccentricity: 0.000001,
            ..Default::default()
        },
    );
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let (initial_position_m, initial_velocity_mps) = if use_central {
        (position_m, velocity_mps)
    } else {
        let (earth_position_m, earth_velocity_mps) =
            spk_state(Body::Earth.data().frame, SSB_J2000, start_epoch(), &[]);
        (
            position_m + earth_position_m,
            velocity_mps + earth_velocity_mps,
        )
    };
    let period_s = orbit_period_s(mu, elements.semi_major_axis);
    let duration_nanos = sec2nano(0.75 * period_s);
    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, 100);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(use_central))
        .expect("unique gravity body");
    spacecraft
        .add_grav_body(Body::Sun.point_mass(false))
        .expect("unique gravity body");
    // Upstream's default SPICE `zeroBase` is the solar-system barycenter.
    let mut earth_ephemeris = Body::Earth.ephemeris_from_ssb();
    let mut sun_ephemeris = Body::Sun.ephemeris_from_ssb();

    let mut csv = scenario_csv(&format!("scenarioCentralBody{}", u8::from(use_central)));
    let mut spacecraft_history = History::<SpacecraftStateMsg>::new();
    let mut earth_history = History::<PlanetStateMsg>::new();
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &sun_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("sun").expect("sun"),
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &earth_ephemeris.output_planet_msg => csv.add_source::<PlanetStateMsg>(planet_columns("earth")),
            &spacecraft.state_out => &mut spacecraft_history.input,
            &earth_ephemeris.output_planet_msg => &mut earth_history.input,
        );
        schedule! { simulation,
            "earth_ephemeris" => &mut earth_ephemeris, period=STEP_NANOS, priority=30;
            "sun_ephemeris" => &mut sun_ephemeris, period=STEP_NANOS, priority=30;
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "spacecraft_history" => &mut spacecraft_history, period=sample_nanos, priority=0;
            "earth_history" => &mut earth_history, period=sample_nanos, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");

    // Upstream subtracts the recorded Earth state from the spacecraft's.
    let relative: Vec<(Vector3<f64>, Vector3<f64>)> = spacecraft_history
        .messages()
        .zip(earth_history.messages())
        .map(|(spacecraft, earth)| {
            (
                spacecraft.position_m - earth.position_inertial_m,
                spacecraft.velocity_mps - earth.velocity_inertial_mps,
            )
        })
        .collect();
    let (final_position_m, final_velocity_mps) = *relative.last().expect("recorded states");
    Results {
        final_radius_m: final_position_m.norm(),
        final_speed_mps: final_velocity_mps.norm(),
        initial_radius_m: position_m.norm(),
        initial_speed_mps: velocity_mps.norm(),
        relative_positions_m: relative.iter().map(|(position, _)| *position).collect(),
        elements,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let cases = if args.flag("--all") {
        vec![false, true]
    } else {
        vec![args.flag("--use-central")]
    };
    for use_central in cases {
        let results = run(use_central);
        println!(
            "useCentral={use_central}: final radius {:.3} km (initial {:.3} km), speed {:.6} km/s (initial {:.6} km/s)",
            results.final_radius_m / 1e3,
            results.initial_radius_m / 1e3,
            results.final_speed_mps / 1e3,
            results.initial_speed_mps / 1e3,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::run;

    /// Upstream `unitTestSupport.isDoubleEqualRelative(a, b, accuracy)`.
    fn is_double_equal_relative(result: f64, truth: f64, accuracy: f64) -> bool {
        ((truth - result) / truth).abs() <= accuracy
    }

    /// `test_scenarioCentralBody.py`, `useCentral` False and True: final
    /// Earth-relative radius and speed equal the initial ones to 1 %.
    #[test]
    fn final_radius_and_speed_match_the_initial_orbit() {
        for use_central in [false, true] {
            let results = run(use_central);
            assert!(
                is_double_equal_relative(results.final_radius_m, results.initial_radius_m, 0.01),
                "useCentral={use_central}: radius {} vs {}",
                results.final_radius_m,
                results.initial_radius_m
            );
            assert!(
                is_double_equal_relative(results.final_speed_mps, results.initial_speed_mps, 0.01),
                "useCentral={use_central}: speed {} vs {}",
                results.final_speed_mps,
                results.initial_speed_mps
            );
        }
    }
}
