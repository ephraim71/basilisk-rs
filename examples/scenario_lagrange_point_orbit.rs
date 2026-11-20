//! Port of upstream `examples/scenarioLagrangePointOrbit.py`: a spacecraft
//! placed near one of the five Earth-Moon Lagrange points.
//!
//! Earth (central) and Moon are point masses; the Moon's state relative to the
//! Earth, and the Earth's orientation, come from the ANISE ephemeris (upstream
//! `createSpiceInterface` with `zeroBase = "Earth"`, from 2022 August 31
//! 15:00 UTC). The spacecraft starts on the Moon's osculating orbit with the
//! semi-major axis and true anomaly offsets upstream uses for each point.
//!
//! `--point 1..5`, `--orbits` (default 1) and `--step` (seconds, default 300)
//! are upstream's `lagrangePoint`, `nOrbits` and `timestep`; `--all` runs the
//! five cases of `test_scenarioLagrangePointOrbit.py`. Writes
//! `scenarioLagrangePointOrbitL<point>.csv` (spacecraft and Moon states).
//!
//! Upstream's test only runs the cases. `tests` checks where each spacecraft
//! is after the one month in the rotating Earth-Moon frame instead: L4 and L5
//! are stable, so those spacecraft stay near their points; the collinear
//! points are unstable and their spacecraft drift, but stay in the
//! Earth-Moon system.

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
scenario_lagrange_point_orbit [--point 1..5] [--orbits 1] [--step 300]
scenario_lagrange_point_orbit --all     run the five upstream test cases";

/// Upstream `timeInitString = "2022 August 31 15:00:00.0"`.
pub fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2022, 8, 31, 15, 0, 0, 0)
}

/// Upstream `run(lagrangePoint, nOrbits, timestep, showPlots)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub lagrange_point: u8,
    pub orbits: f64,
    pub step_seconds: f64,
}

pub struct Results {
    pub spacecraft: Vec<(u64, SpacecraftStateMsg)>,
    pub moon: Vec<(u64, PlanetStateMsg)>,
    /// The spacecraft's initial elements (upstream `oe`).
    pub elements: ClassicElements,
}

/// Rotating Earth-Moon frame coordinates of `position`: x along the Moon,
/// y along its velocity, z along its angular momentum (upstream's `DCM`).
pub fn rotating_frame(moon: &PlanetStateMsg, position: Vector3<f64>) -> Vector3<f64> {
    let r = moon.position_inertial_m;
    let v = moon.velocity_inertial_mps;
    let h = r.cross(&v);
    Vector3::new(
        r.normalize().dot(&position),
        v.normalize().dot(&position),
        h.normalize().dot(&position),
    )
}

pub fn run(config: Config) -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let (earth, moon) = (Body::Earth.data(), Body::Moon.data());
    let (moon_position_m, moon_velocity_mps) =
        spk_state(moon.frame, earth.frame, start_epoch(), &[]);
    let mut elements = state_to_elements(earth.mu_m3ps2, moon_position_m, moon_velocity_mps);
    let moon_a = elements.semi_major_axis;
    let mass_ratio = moon.mu_m3ps2 / earth.mu_m3ps2;
    match config.lagrange_point {
        1 => {
            elements.semi_major_axis *= 1.0 - (mass_ratio / 3.0).cbrt();
            elements.true_anomaly_rad += 4.0 * degrees;
        }
        2 => {
            elements.semi_major_axis *= 1.0 + (mass_ratio / 3.0).cbrt();
            elements.true_anomaly_rad -= 4.0 * degrees;
        }
        3 => {
            elements.semi_major_axis *= 1.0 - 7.0 * mass_ratio / 12.0;
            elements.true_anomaly_rad += std::f64::consts::PI;
        }
        4 => elements.true_anomaly_rad += std::f64::consts::PI / 3.0,
        _ => elements.true_anomaly_rad -= std::f64::consts::PI / 3.0,
    }
    elements.true_anomaly_rad -= 2.0 * degrees;
    let (position_m, velocity_mps) = elements_to_state(earth.mu_m3ps2, &elements);

    let step_nanos = sec2nano(config.step_seconds);
    let duration_nanos = sec2nano(config.orbits * orbit_period_s(earth.mu_m3ps2, moon_a));
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

    let mut csv = scenario_csv(&format!(
        "scenarioLagrangePointOrbitL{}",
        config.lagrange_point
    ));
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
            "lagrangeSat" => &mut spacecraft, period=step_nanos, priority=0;
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
        elements,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let points = if args.flag("--all") {
        vec![1, 2, 3, 4, 5]
    } else {
        vec![args.value("--point", 1u8)]
    };
    for lagrange_point in points {
        if !(1..=5).contains(&lagrange_point) {
            args.fail("--point must be 1 to 5");
        }
        let results = run(Config {
            lagrange_point,
            orbits: args.value("--orbits", 1.0),
            step_seconds: args.value("--step", 300.0),
        });
        let (_, last) = results.spacecraft.last().expect("samples");
        let (_, moon) = results.moon.last().expect("samples");
        let rotating = rotating_frame(moon, last.position_m) / 1e3;
        println!(
            "wrote scenarioLagrangePointOrbitL{lagrange_point}.csv: final rotating-frame position [{:.0}, {:.0}, {:.0}] km",
            rotating.x, rotating.y, rotating.z
        );
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use super::{Config, rotating_frame, run};

    /// Rotating-frame positions (m) at the start and the end of the
    /// one-orbit, 300 s step run of `test_scenarioLagrangePointOrbit.py`.
    fn rotating_start_and_end(lagrange_point: u8) -> (Vector3<f64>, Vector3<f64>) {
        let results = run(Config {
            lagrange_point,
            orbits: 1.0,
            step_seconds: 300.0,
        });
        let (_, first) = results.spacecraft.first().expect("samples");
        let (_, first_moon) = results.moon.first().expect("samples");
        let (_, last) = results.spacecraft.last().expect("samples");
        let (_, last_moon) = results.moon.last().expect("samples");
        (
            rotating_frame(first_moon, first.position_m),
            rotating_frame(last_moon, last.position_m),
        )
    }

    /// Upstream's test only runs L1 to L5. Its documentation: from L4 and L5,
    /// stable points 60 degrees ahead of and behind the Moon at the Moon's
    /// distance, the spacecraft remains near the point.
    #[test]
    fn spacecraft_remain_near_the_triangular_points() {
        for (point, lead_deg) in [(4, 60.0), (5, -60.0)] {
            let (start, end) = rotating_start_and_end(point);
            let angle_deg = end.y.atan2(end.x).to_degrees();
            assert!(
                (angle_deg - lead_deg).abs() < 15.0,
                "L{point}: {angle_deg} deg from the Moon"
            );
            assert!(
                (end.norm() / start.norm() - 1.0).abs() < 0.1,
                "L{point}: radius {} km",
                end.norm() / 1e3
            );
        }
    }

    /// From L1, L2 and L3, unstable points, the spacecraft move away within
    /// the month: more than 100 000 km in the rotating frame.
    #[test]
    fn spacecraft_leave_the_collinear_points() {
        for point in [1, 2, 3] {
            let (start, end) = rotating_start_and_end(point);
            assert!(
                (end - start).norm() > 1.0e8,
                "L{point}: moved only {} km",
                (end - start).norm() / 1e3
            );
        }
    }
}
