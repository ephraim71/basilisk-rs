//! Port of upstream `examples/scenarioHaloOrbit.py`: a spacecraft on a
//! near-halo orbit about the Earth-Moon L2 point.
//!
//! As in `scenario_lagrange_point_orbit`, Earth (central) and Moon are point
//! masses with the Moon's Earth-relative state and the Earth's orientation from
//! the ANISE ephemeris (upstream `createSpiceInterface`, `zeroBase = "earth"`,
//! from 2022 August 31 15:00 UTC). Upstream's non-dimensional
//! circular-restricted three-body initial conditions
//! (`x0 = 1.182212 a + a mu`, `z0 = 0.049 a`, `dy0 = -0.167 a / T`) are
//! rotated from the Earth-Moon frame of the initial Moon state into the
//! inertial frame. 17.5 days with 300 s steps, sampled 1000 times.
//!
//! Writes `scenarioHaloOrbit.csv` (spacecraft and Moon states). Upstream has
//! no test for this scenario; `tests` checks that the spacecraft stays on its
//! orbit about L2, beyond the Moon in the rotating frame.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::messages::{PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::state_to_elements;
use basilisk_rs::{connect, schedule};
use grav_factory::{Body, spk_state};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{day2nano, sec2nano};
use recording::{History, planet_columns, sampling_time, scenario_csv, state_columns};

/// Upstream `timestep` (s).
const STEP_SECONDS: f64 = 300.0;

/// Upstream `timeInitString = "2022 August 31 15:00:00.0"`.
fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2022, 8, 31, 15, 0, 0, 0)
}

pub struct Results {
    pub spacecraft: Vec<(u64, SpacecraftStateMsg)>,
    pub moon: Vec<(u64, PlanetStateMsg)>,
}

/// The Earth-Moon rotating frame of a Moon state, rows `[r, v, r x v]`
/// normalized (upstream `DCMInit` / `DCM`).
pub fn rotating_dcm(
    moon_position_m: Vector3<f64>,
    moon_velocity_mps: Vector3<f64>,
) -> Matrix3<f64> {
    let h = moon_position_m.cross(&moon_velocity_mps);
    Matrix3::from_rows(&[
        moon_position_m.normalize().transpose(),
        moon_velocity_mps.normalize().transpose(),
        h.normalize().transpose(),
    ])
}

pub fn run() -> Results {
    let (earth, moon) = (Body::Earth.data(), Body::Moon.data());
    let (moon_position_m, moon_velocity_mps) =
        spk_state(moon.frame, earth.frame, start_epoch(), &[]);
    let moon_a =
        state_to_elements(earth.mu_m3ps2, moon_position_m, moon_velocity_mps).semi_major_axis;
    let dcm_init = rotating_dcm(moon_position_m, moon_velocity_mps);
    // Non-dimensional time unit (s) and mass ratio.
    let time_unit_s = (moon_a.powi(3) / (earth.mu_m3ps2 + moon.mu_m3ps2)).sqrt();
    let mass_ratio = moon.mu_m3ps2 / (earth.mu_m3ps2 + moon.mu_m3ps2);
    let x0 = 1.182212 * moon_a + moon_a * mass_ratio;
    let z0 = 0.049 * moon_a;
    let dy0 = -0.167 * moon_a / time_unit_s;
    let position_m = dcm_init.transpose() * Vector3::new(x0, 0.0, z0);
    let velocity_mps =
        dcm_init.transpose() * Vector3::new(0.0, moon_velocity_mps.norm() + dy0, 0.0);

    let step_nanos = sec2nano(STEP_SECONDS);
    let duration_nanos = day2nano(17.5);
    let sample_nanos = sampling_time(duration_nanos, step_nanos, 1000);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: step_nanos,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Moon.point_mass(false))
        .expect("unique gravity body");
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("unique gravity body");
    let mut moon_ephemeris = Body::Moon.ephemeris(Body::Earth);
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);

    let mut csv = scenario_csv("scenarioHaloOrbit");
    let mut spacecraft_history = History::<SpacecraftStateMsg>::new();
    let mut moon_history = History::<PlanetStateMsg>::new();
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        connect!(&simulation,
            &moon_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("moon").expect("moon"),
            &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &moon_ephemeris.output_planet_msg => csv.add_source::<PlanetStateMsg>(planet_columns("moon")),
            &spacecraft.state_out => &mut spacecraft_history.input,
            &moon_ephemeris.output_planet_msg => &mut moon_history.input,
        );
        schedule! { simulation,
            "moon_ephemeris" => &mut moon_ephemeris, period=step_nanos, priority=1;
            "earth_ephemeris" => &mut earth_ephemeris, period=step_nanos, priority=1;
            "HaloSat" => &mut spacecraft, period=step_nanos, priority=0;
            "csv" => &mut csv, period=sample_nanos, priority=-1;
            "spacecraft_history" => &mut spacecraft_history, period=sample_nanos, priority=-1;
            "moon_history" => &mut moon_history, period=sample_nanos, priority=-1;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        spacecraft: spacecraft_history.into_samples(),
        moon: moon_history.into_samples(),
    }
}

/// Rotating-frame spacecraft positions (m) at the samples.
pub fn rotating_positions(results: &Results) -> Vec<Vector3<f64>> {
    results
        .spacecraft
        .iter()
        .zip(&results.moon)
        .map(|((_, spacecraft), (_, moon))| {
            rotating_dcm(moon.position_inertial_m, moon.velocity_inertial_mps)
                * spacecraft.position_m
        })
        .collect()
}

fn main() {
    let results = run();
    let positions = rotating_positions(&results);
    let last = positions.last().expect("samples") / 1e3;
    println!(
        "wrote scenarioHaloOrbit.csv: final rotating-frame position [{:.0}, {:.0}, {:.0}] km",
        last.x, last.y, last.z
    );
}

#[cfg(test)]
mod tests {
    use super::{rotating_positions, run};

    /// No upstream test. The near-halo orbit loops once about L2 in the
    /// 17.5 days: always on the far side of the Moon, within 130 000 km of
    /// the L2 point (1.16 Moon distances out), swinging to either side of the
    /// Earth-Moon axis and out of the Moon's orbit plane.
    #[test]
    fn spacecraft_loops_about_l2() {
        let results = run();
        let positions = rotating_positions(&results);
        assert!(positions.len() >= 1000);
        for ((_, moon), position) in results.moon.iter().zip(&positions) {
            let moon_distance = moon.position_inertial_m.norm();
            assert!(position.x > moon_distance, "behind the Moon: {position:?}");
            let from_l2 = position - nalgebra::Vector3::new(1.16 * moon_distance, 0.0, 0.0);
            assert!(from_l2.norm() < 1.3e8, "left L2: {position:?}");
        }
        let (min_y, max_y) = positions.iter().fold((f64::MAX, f64::MIN), |(lo, hi), p| {
            (lo.min(p.y), hi.max(p.y))
        });
        assert!(
            min_y < -5.0e7 && max_y > 5.0e7,
            "no loop: y in [{min_y}, {max_y}] m"
        );
        let max_z = positions.iter().map(|p| p.z.abs()).fold(0.0, f64::max);
        assert!(max_z > 1.0e7, "no out-of-plane motion: {max_z} m");
    }
}
