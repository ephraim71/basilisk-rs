//! Port of upstream `examples/scenarioJupiterArrival.py`: a spacecraft
//! arriving at Jupiter on the hyperbola of an Earth-Jupiter Hohmann
//! transfer, which an impulsive burn at periapsis turns into a circular
//! 800,000 km parking orbit.
//!
//! The run integrates the hyperbola (point-mass Jupiter, 10 s step) to the
//! time of periapsis, adds the parking-orbit `Delta_V` along the velocity of
//! the last recorded sample (upstream's `dataRec.v_BN_N[-1]`), sets it as
//! the hub velocity (upstream `velRef.setState`, here
//! [`basilisk_rs::spacecraft::Spacecraft::hub_state_handle`]), and continues
//! for 300,000 s. The states, sampled 100 times over the arrival, go to
//! `scenarioJupiterArrival.csv`.
//!
//! Upstream has no `test_scenarioJupiterArrival.py`; `tests` checks the
//! orbit the burn reaches: circular to within a few percent at the parking
//! radius.

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/latch.rs"]
mod latch;
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
use hifitime::Epoch;
use latch::Latch;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{run_until, sec2nano};
use recording::{History, sampling_time, scenario_csv, state_columns};
use scenario_args::ScenarioArgs;

const USAGE: &str = "scenario_jupiter_arrival     run the Jupiter arrival and capture burn";

/// Upstream `simulationTimeStep`.
const STEP_NANOS: u64 = 10_000_000_000;
/// Upstream `sunmu` (m^3/s^2).
const SUN_MU: f64 = 132_600_000_000_000_000_000.0;
/// Jupiter-Sun distance (m).
const R_JUPITER_SUN: f64 = 778_298_361.0 * 1000.0;
/// Earth-Sun distance (m).
const R_EARTH_SUN: f64 = 149_598_023.0 * 1000.0;
/// Desired circular parking orbit radius (m).
pub const R_PARK: f64 = 800_000.0 * 1000.0;

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream `Delta_V_Parking_Orbit` (m/s).
    pub delta_v_mps: f64,
    /// Upstream `t_tp`: time from the start to periapsis (s).
    pub time_to_periapsis_s: f64,
    /// The state at the end of the run.
    pub final_state: SpacecraftStateMsg,
    pub mu_m3ps2: f64,
}

pub fn run() -> Results {
    let jupiter = Body::JupiterBarycenter;
    let mu = jupiter.mu_m3ps2();
    let degrees = std::f64::consts::PI / 180.0;
    // (1) Jupiter's circular speed about the Sun; (2) the parking speed;
    // (3) the transfer ellipse's apoapsis speed about the Sun; (4) that
    // speed relative to Jupiter, the hyperbolic excess; (5) the hyperbola's
    // periapsis speed.
    let v_jupiter_sun = (SUN_MU / R_JUPITER_SUN).sqrt();
    let v_park = (mu / R_PARK).sqrt();
    let v_apoapsis_sun =
        (SUN_MU * R_EARTH_SUN / ((R_JUPITER_SUN + R_EARTH_SUN) * R_JUPITER_SUN)).sqrt();
    let v_infinity = v_apoapsis_sun - v_jupiter_sun;
    let a_hyperbola = -(mu / (v_infinity * v_infinity));
    let v_periapsis = (v_infinity * v_infinity + 2.0 * mu / R_PARK).sqrt();
    let delta_v_mps = v_park - v_periapsis;
    let e_hyperbola = 1.0 + (R_PARK * v_infinity * v_infinity) / mu;

    let elements = ClassicElements {
        semi_major_axis: a_hyperbola,
        eccentricity: e_hyperbola,
        inclination_rad: 0.0,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 280.0 * degrees,
        ..Default::default()
    };
    // Time from the start to periapsis (hyperbolic Kepler equation, with
    // upstream's Gudermannian form of the hyperbolic anomaly).
    let f = elements.true_anomaly_rad;
    let e = elements.eccentricity;
    let zeta = 2.0 * ((f / 2.0).tan() * ((e - 1.0) / (e + 1.0)).sqrt()).atan();
    let time_to_periapsis_s = ((-a_hyperbola.powi(3) / mu).sqrt()
        * (e * zeta.tan() - (zeta / 2.0 + std::f64::consts::FRAC_PI_4).tan().ln()))
    .abs();
    let (position_m, velocity_mps) = elements_to_state(mu, &elements);

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
        .add_grav_body(jupiter.point_mass(true))
        .expect("one central gravity body");
    let hub = spacecraft.hub_state_handle();
    let state_out = spacecraft.state_out.clone();

    let arrival_nanos = sec2nano(time_to_periapsis_s);
    let sample_nanos = sampling_time(arrival_nanos, STEP_NANOS, 100).max(STEP_NANOS);
    let mut csv = scenario_csv("scenarioJupiterArrival");
    let csv_path = csv.config.output_path.clone();
    let mut history = History::<SpacecraftStateMsg>::new();
    // Upstream reads `dataRec.v_BN_N[-1]`, the last sample, which trails
    // the current state unless the run ended on a sample.
    let mut last_sample = Latch::<SpacecraftStateMsg>::default();
    let last_sampled = last_sample.output.clone();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
            &spacecraft.state_out => &mut last_sample.input,
        );
        schedule! { simulation,
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "history" => &mut history, period=sample_nanos, priority=0;
            "last_sample" => &mut last_sample, period=sample_nanos, priority=0;
        }
        simulation.run_for(0);
        run_until(&mut simulation, arrival_nanos);

        // The capture burn, along the last recorded velocity.
        let last_velocity = last_sampled.read().velocity_mps;
        let burned = last_velocity + delta_v_mps * last_velocity.normalize();
        hub.set_velocity_mps(burned);
        run_until(&mut simulation, arrival_nanos + sec2nano(300_000.0));
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        final_state: state_out.read(),
        states: history.into_samples(),
        delta_v_mps,
        time_to_periapsis_s,
        mu_m3ps2: mu,
    }
}

fn main() {
    ScenarioArgs::from_env(USAGE);
    let results = run();
    println!(
        "wrote {} (periapsis after {:.0} s, burn {:.1} m/s, final radius {:.0} km)",
        results.csv_path.display(),
        results.time_to_periapsis_s,
        results.delta_v_mps,
        results.final_state.position_m.norm() / 1000.0
    );
}

#[cfg(test)]
mod tests {
    use super::{R_PARK, run};
    use basilisk_rs::utilities::orbital_motion::state_to_elements;

    /// No upstream test: the burn must capture the spacecraft into a
    /// near-circular orbit at the parking radius, which it then keeps.
    #[test]
    fn capture_burn_reaches_the_parking_orbit() {
        let results = run();
        assert!(results.delta_v_mps < 0.0, "a braking burn");
        let elements = state_to_elements(
            results.mu_m3ps2,
            results.final_state.position_m,
            results.final_state.velocity_mps,
        );
        assert!(
            elements.eccentricity < 0.05,
            "e = {}",
            elements.eccentricity
        );
        assert!(
            (elements.semi_major_axis / R_PARK - 1.0).abs() < 0.05,
            "a = {} km",
            elements.semi_major_axis / 1000.0
        );
        // Before the burn the hyperbola reaches periapsis at the parking
        // radius.
        let closest = results
            .states
            .iter()
            .map(|(_, state)| state.position_m.norm())
            .fold(f64::INFINITY, f64::min);
        assert!(
            (closest / R_PARK - 1.0).abs() < 0.05,
            "closest {} km",
            closest / 1000.0
        );
    }
}
