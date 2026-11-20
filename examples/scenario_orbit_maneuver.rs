//! Port of upstream `examples/scenarioOrbitManeuver.py`: impulsive orbit
//! maneuvers applied by changing the hub velocity between simulation runs.
//!
//! * `--maneuver 0` (Hohmann): from a 7000 km LEO to GEO, a tangential burn
//!   after a quarter orbit, a half transfer orbit, the circularizing burn, and
//!   an eighth of the GEO orbit.
//! * `--maneuver 1` (inclination): an 8 degree plane change after a quarter
//!   orbit, a quarter orbit, a 4 degree plane change, a quarter orbit.
//!
//! The velocity is changed through the spacecraft's
//! [`HubStateHandle`](basilisk_rs::spacecraft::HubStateHandle), the stand-in
//! for upstream's `dynManager.getStateObject(...).setState(...)`. Earth is a
//! point mass whose (zero) state and orientation come from the ANISE
//! ephemeris with the Earth as zero base (upstream `createSpiceInterface`
//! with `zeroBase = 'Earth'`, from 2024 September 21 21:00 TDB).
//!
//! Writes `scenarioOrbitManeuver<case>.csv`; `--all` runs both cases.
//! Upstream's test only runs them; `tests` checks the orbits they reach: GEO
//! for the Hohmann transfer, and the 8.93845 degree inclination upstream's
//! plot marks for the two plane changes.

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

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use grav_factory::Body;
use hifitime::{Epoch, TimeScale};
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{orbit_period_s, run_until, sec2nano};
use recording::{History, sampling_time, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_orbit_maneuver [--maneuver 0|1]   0: Hohmann transfer, 1: inclination change
scenario_orbit_maneuver --all              run both upstream test cases";

const STEP_NANOS: u64 = 10_000_000_000;

/// Upstream `maneuverCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Maneuver {
    Hohmann = 0,
    Inclination = 1,
}

pub struct Results {
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream `rGEO` (m).
    pub geo_radius_m: f64,
    /// The hub state at the end (upstream `posRef.getState()`, `velRef`).
    pub final_position_m: Vector3<f64>,
    pub final_velocity_mps: Vector3<f64>,
}

/// Upstream's plane change: rotate the velocity's horizontal component by
/// `delta_i` about the radial direction.
fn plane_change(
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    delta_i: f64,
) -> Vector3<f64> {
    let r_hat = position_m.normalize();
    let h_hat = position_m.cross(&velocity_mps).normalize();
    let v_hat = h_hat.cross(&r_hat);
    let v0 = v_hat.dot(&velocity_mps);
    velocity_mps - (1.0 - delta_i.cos()) * v0 * v_hat + delta_i.sin() * v0 * h_hat
}

/// Upstream `run(show_plots, maneuverCase)`.
pub fn run(maneuver: Maneuver) -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mu = Body::Earth.mu_m3ps2();
    let r_geo = (mu / (2.0 * std::f64::consts::PI / (24.0 * 3600.0)).powi(2)).cbrt();
    let elements = ClassicElements {
        semi_major_axis: 7_000.0 * 1_000.0,
        eccentricity: 0.0001,
        inclination_rad: 0.0,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(mu, &elements);
    let period_s = orbit_period_s(mu, elements.semi_major_axis);
    let first_stop_nanos = sec2nano(0.25 * period_s);
    let sample_nanos = sampling_time(first_stop_nanos, STEP_NANOS, 100);

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
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central body");
    let hub = spacecraft.hub_state_handle();
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);

    let mut csv = scenario_csv(&format!("scenarioOrbitManeuver{}", maneuver as u8));
    let mut history = History::<SpacecraftStateMsg>::new();
    {
        let start = Epoch::from_gregorian(2024, 9, 21, 21, 0, 0, 0, TimeScale::TDB);
        let mut simulation = Simulation::new(start, false);
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
        );
        schedule! { simulation,
            "spiceObject" => &mut earth_ephemeris, period=STEP_NANOS, priority=30;
            "spacecraftBody" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "dataRec" => &mut history, period=sample_nanos, priority=0;
        }
        run_until(&mut simulation, first_stop_nanos);

        // First maneuver.
        let (r, v) = (hub.position_m(), hub.velocity_mps());
        let second_leg_nanos = match maneuver {
            Maneuver::Inclination => {
                hub.set_velocity_mps(plane_change(r, v, 8.0 * degrees));
                sec2nano(period_s * 0.25)
            }
            Maneuver::Hohmann => {
                let (v0, r0) = (v.norm(), r.norm());
                let transfer_a = (r0 + r_geo) * 0.5;
                let v0p = (mu / transfer_a * r_geo / r0).sqrt();
                let n1 = (mu / transfer_a.powi(3)).sqrt();
                hub.set_velocity_mps(v + v.normalize() * (v0p - v0));
                sec2nano(std::f64::consts::PI / n1)
            }
        };
        run_until(&mut simulation, first_stop_nanos + second_leg_nanos);

        // Second maneuver.
        let (r, v) = (hub.position_m(), hub.velocity_mps());
        let third_leg_nanos = match maneuver {
            Maneuver::Inclination => {
                hub.set_velocity_mps(plane_change(r, v, 4.0 * degrees));
                sec2nano(period_s * 0.25)
            }
            Maneuver::Hohmann => {
                let v1 = v.norm();
                let v1p = (mu / r_geo).sqrt();
                let n1 = (mu / r_geo.powi(3)).sqrt();
                hub.set_velocity_mps(v + v.normalize() * (v1p - v1));
                sec2nano(0.25 * std::f64::consts::PI / n1)
            }
        };
        run_until(
            &mut simulation,
            first_stop_nanos + second_leg_nanos + third_leg_nanos,
        );
    }
    csv.flush().expect("flush scenario CSV");

    Results {
        states: history.into_samples(),
        geo_radius_m: r_geo,
        final_position_m: hub.position_m(),
        final_velocity_mps: hub.velocity_mps(),
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let cases = if args.flag("--all") {
        vec![Maneuver::Hohmann, Maneuver::Inclination]
    } else {
        match args.value("--maneuver", 0u8) {
            0 => vec![Maneuver::Hohmann],
            1 => vec![Maneuver::Inclination],
            other => args.fail(&format!("unknown maneuver case {other}")),
        }
    };
    let mu = Body::Earth.mu_m3ps2();
    for maneuver in cases {
        let results = run(maneuver);
        let elements = basilisk_rs::utilities::orbital_motion::state_to_elements(
            mu,
            results.final_position_m,
            results.final_velocity_mps,
        );
        println!(
            "scenarioOrbitManeuver{}: final radius {:.1} km, inclination {:.5} deg",
            maneuver as u8,
            results.final_position_m.norm() / 1e3,
            elements.inclination_rad.to_degrees()
        );
    }
}

#[cfg(test)]
mod tests {
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    use super::{Maneuver, run};
    use crate::grav_factory::Body;

    /// The Hohmann transfer ends on a circular GEO orbit.
    #[test]
    fn hohmann_transfer_reaches_geo() {
        let results = run(Maneuver::Hohmann);
        let elements = state_to_elements(
            Body::Earth.mu_m3ps2(),
            results.final_position_m,
            results.final_velocity_mps,
        );
        // The burns happen on the 10 s task grid, up to one step from perigee
        // and apogee, which leaves a small eccentricity (as upstream).
        let sma_error = (elements.semi_major_axis - results.geo_radius_m).abs();
        assert!(
            sma_error < 1.0e-4 * results.geo_radius_m,
            "a is {sma_error} m off GEO"
        );
        assert!(
            elements.eccentricity < 2.0e-3,
            "e = {}",
            elements.eccentricity
        );
        assert!(results.states.len() > 100);
    }

    /// The 8 and 4 degree plane changes a quarter orbit apart leave the
    /// 8.93845 degree inclination of upstream's reference line,
    /// `acos(cos 8 cos 4)`; the burns fall on the 10 s grid a few seconds
    /// short of the quarter orbit, which shifts it by hundredths of a degree.
    #[test]
    fn plane_changes_reach_the_combined_inclination() {
        let results = run(Maneuver::Inclination);
        let elements = state_to_elements(
            Body::Earth.mu_m3ps2(),
            results.final_position_m,
            results.final_velocity_mps,
        );
        let inclination_deg = elements.inclination_rad.to_degrees();
        assert!(
            (inclination_deg - 8.93845).abs() < 5.0e-2,
            "inclination {inclination_deg} deg"
        );
    }
}
