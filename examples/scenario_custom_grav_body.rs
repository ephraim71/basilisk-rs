//! Port of upstream `examples/scenarioCustomGravBody.py`: a spacecraft in a
//! 500 m orbit about the asteroid Itokawa, a custom gravity body whose
//! heliocentric motion (and the Earth's) comes from a two-body
//! `planetEphemeris`.
//!
//! The Sun is a point mass at the origin (no ephemeris message, as upstream);
//! Itokawa (`mu = 2.34268 m^3/s^2`, the central body) and the Earth are
//! `createCustomGravObject` point masses following
//! [`PlanetEphemeris`](basilisk_rs::environment::planet_ephemeris).
//! Upstream's Vizard custom-model (`ItokawaHayabusa.obj`) is display only and
//! not ported.
//!
//! Writes `scenarioCustomGravBody.csv`: the spacecraft state and Itokawa's.
//! Upstream's test only runs the scenario; `tests` checks the physical end
//! state instead: the solar tide on a 500 m orbit is negligible, so the
//! Itokawa-relative motion stays the two-body orbit it started on.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::environment::planet_ephemeris::{
    PlanetEphemeris, PlanetEphemerisBody, PlanetEphemerisConfig, PlanetRotation,
};
use basilisk_rs::messages::{PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::astro_constants::AU;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use grav_factory::{Body, custom_point_mass};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{min2nano, nano2sec};
use recording::{History, planet_columns, sampling_time, scenario_csv, state_columns};

const STEP_NANOS: u64 = 10_000_000_000;
/// Itokawa's gravitational parameter (m^3/s^2).
pub const ITOKAWA_MU_M3PS2: f64 = 2.34268;

pub struct Results {
    /// Sample times (ns) and Itokawa-relative positions (m): upstream
    /// `posData = scRec.r_BN_N - astRec.PositionVector`.
    pub times_nanos: Vec<u64>,
    pub relative_positions_m: Vec<Vector3<f64>>,
    /// The spacecraft's initial Itokawa-relative orbit.
    pub elements: ClassicElements,
}

pub fn run() -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let duration_nanos = min2nano(1120.0);
    let mut ephemeris = PlanetEphemeris::new(PlanetEphemerisConfig {
        name: "planetEphemeris".to_string(),
        bodies: vec![
            PlanetEphemerisBody {
                name: "Itokawa".to_string(),
                elements: ClassicElements {
                    semi_major_axis: 1.3241 * AU * 1000.0,
                    eccentricity: 0.2801,
                    inclination_rad: 1.6214 * degrees,
                    raan_rad: 69.081 * degrees,
                    arg_periapsis_rad: 162.82 * degrees,
                    true_anomaly_rad: 90.0 * degrees,
                    ..Default::default()
                },
                rotation: Some(PlanetRotation {
                    rotation_rate_radps: 360.0 * degrees / (12.132 * 3600.0),
                    ..Default::default()
                }),
            },
            PlanetEphemerisBody {
                name: "earth".to_string(),
                elements: ClassicElements {
                    semi_major_axis: AU * 1000.0,
                    eccentricity: 0.0167086,
                    inclination_rad: 7.155 * degrees,
                    raan_rad: 174.9 * degrees,
                    arg_periapsis_rad: 288.1 * degrees,
                    true_anomaly_rad: 270.0 * degrees,
                    ..Default::default()
                },
                rotation: Some(PlanetRotation {
                    rotation_rate_radps: 360.0 * degrees / (24.0 * 3600.0),
                    ..Default::default()
                }),
            },
        ],
        zero_base: None,
    });

    let elements = ClassicElements {
        semi_major_axis: 500.0,
        eccentricity: 0.0001,
        inclination_rad: 33.3 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(ITOKAWA_MU_M3PS2, &elements);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    for body in [
        Body::Sun.point_mass(false),
        custom_point_mass("Itokawa", ITOKAWA_MU_M3PS2, true),
        custom_point_mass("earth", 0.3986004415E+15, false),
    ] {
        spacecraft
            .add_grav_body(body)
            .expect("unique gravity bodies");
    }

    let sample_nanos = sampling_time(duration_nanos, STEP_NANOS, 100);
    let mut csv = scenario_csv("scenarioCustomGravBody");
    let mut spacecraft_history = History::<SpacecraftStateMsg>::new();
    let mut asteroid_history = History::<PlanetStateMsg>::new();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &ephemeris.planet_out_msgs[0] => spacecraft.grav_body_input_mut("Itokawa").expect("Itokawa"),
            &ephemeris.planet_out_msgs[1] => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &ephemeris.planet_out_msgs[0] => csv.add_source::<PlanetStateMsg>(planet_columns("itokawa")),
            &spacecraft.state_out => &mut spacecraft_history.input,
            &ephemeris.planet_out_msgs[0] => &mut asteroid_history.input,
        );
        schedule! { simulation,
            "planetEphemeris" => &mut ephemeris, period=STEP_NANOS, priority=30;
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "spacecraft_history" => &mut spacecraft_history, period=sample_nanos, priority=0;
            "asteroid_history" => &mut asteroid_history, period=sample_nanos, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");

    let relative_positions_m = spacecraft_history
        .messages()
        .zip(asteroid_history.messages())
        .map(|(spacecraft, asteroid)| spacecraft.position_m - asteroid.position_inertial_m)
        .collect();
    Results {
        times_nanos: spacecraft_history.times_nanos(),
        relative_positions_m,
        elements,
    }
}

fn main() {
    let results = run();
    let last = results.relative_positions_m.last().expect("samples");
    println!(
        "wrote scenarioCustomGravBody.csv: Itokawa-relative radius {:.3} m after {:.0} min",
        last.norm(),
        nano2sec(*results.times_nanos.last().expect("samples")) / 60.0
    );
}

#[cfg(test)]
mod tests {
    use super::{ITOKAWA_MU_M3PS2, run};
    use crate::orbit_tools::{kepler_state, nano2sec};

    /// No upstream truth (the test only runs the scenario): the
    /// Itokawa-relative trajectory stays on its two-body orbit, within 1 m of
    /// the Kepler solution at every sample.
    #[test]
    fn spacecraft_stays_on_its_itokawa_orbit() {
        let results = run();
        assert_eq!(results.times_nanos.len(), 101);
        for (time, position) in results
            .times_nanos
            .iter()
            .zip(&results.relative_positions_m)
        {
            let (kepler_position, _) =
                kepler_state(ITOKAWA_MU_M3PS2, &results.elements, nano2sec(*time));
            assert!(
                (position - kepler_position).norm() < 1.0,
                "t = {} s: {} m from the Kepler orbit",
                nano2sec(*time),
                (position - kepler_position).norm()
            );
        }
    }
}
