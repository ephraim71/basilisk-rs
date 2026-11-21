//! Port of upstream `examples/scenarioAerocapture.py`: a 2530 kg spacecraft
//! arriving on a hyperbola dips into the atmosphere of Earth or Mars, a
//! tabulated GRAM atmosphere ([`TabularAtmosphere`]), and loses enough energy
//! to cannonball drag to be captured.
//!
//! | `--planet` | atmosphere table | entry state | duration | CSV |
//! |---|---|---|---|---|
//! | `Earth` | `EarthGRAMNominal.txt` | 6503 km, 11.2 km/s, -5.15 deg | 300 s | `scenarioAerocaptureEarth.csv` |
//! | `Mars` (default) | `MarsGRAMNominal.txt` | 3522.2 km, 6 km/s, -10 deg | 400 s | `scenarioAerocaptureMars.csv` |
//!
//! `--all` runs both, the parametrizations of `test_scenarioAerocapture.py`
//! (`useWind = False`; the SPICE wind case is not tested upstream and not
//! ported). The tables come from `just fetch-environment-data`.
//!
//! Upstream adds the atmosphere to the spacecraft's task, before the
//! spacecraft, and to a task of its own after it, followed by the drag
//! effector's task. Both evaluations see the same spacecraft state message,
//! so the density the drag uses over a step and the one recorded are those
//! of the state at the step's start. Here the atmosphere runs once per tick,
//! before the spacecraft, and the drag effector reads it during the
//! integration, which gives the same.
//!
//! `test_scenarioAerocapture.py` only runs the scenario; `tests` checks the
//! capture its figures show: the hyperbolic arrival ends on a bound orbit
//! without reaching the surface.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::dynamics::drag_dynamic_effector::{
    DragDynamicEffector, DragDynamicEffectorConfig,
};
use basilisk_rs::environment::atmosphere::tabular_atmosphere::{
    AtmTableFormat, TabularAtmosphere, TabularAtmosphereConfig, read_atm_table,
};
use basilisk_rs::messages::{AtmosphereMsg, SpacecraftStateMsg};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use recording::{scenario_csv, state_columns};

const USAGE: &str = "\
scenario_aerocapture [--planet Earth|Mars]
scenario_aerocapture --all     run every upstream test parametrization";

/// Upstream `simulationTimeStep`.
const STEP_NANOS: u64 = 10_000_000_000;

/// Upstream `planetCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanetCase {
    Earth,
    Mars,
}

impl PlanetCase {
    pub fn label(self) -> &'static str {
        match self {
            Self::Earth => "Earth",
            Self::Mars => "Mars",
        }
    }

    /// Upstream `gravFactory.createBody(planetCase)`.
    pub fn body(self) -> Body {
        match self {
            Self::Earth => Body::Earth,
            Self::Mars => Body::Mars,
        }
    }

    /// Upstream `r_eq`, the radius the atmosphere's altitude is measured
    /// from (m).
    pub fn atmosphere_radius_m(self) -> f64 {
        match self {
            Self::Earth => 6_378_136.6,
            Self::Mars => 3_397.2 * 1000.0,
        }
    }
}

/// `test_scenarioAerocapture.py`'s `planetCase`s.
pub const TEST_CASES: [PlanetCase; 2] = [PlanetCase::Earth, PlanetCase::Mars];

/// Upstream `sph2rv`: radius, longitude, latitude, speed, flight-path angle
/// and heading to inertial position and velocity, with the inertial and
/// planet-fixed frames aligned.
pub fn sph2rv(
    r: f64,
    lon: f64,
    lat: f64,
    u: f64,
    gam: f64,
    hda: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let ie = Matrix3::new(
        lat.cos() * lon.cos(),
        -lon.sin(),
        -lat.sin() * lon.cos(),
        lat.cos() * lon.sin(),
        lon.cos(),
        -lat.sin() * lon.sin(),
        lat.sin(),
        0.0,
        lat.cos(),
    );
    let es = Matrix3::new(
        gam.cos(),
        0.0,
        gam.sin(),
        -gam.sin() * hda.sin(),
        hda.cos(),
        gam.cos() * hda.sin(),
        -gam.sin() * hda.cos(),
        -hda.sin(),
        gam.cos() * hda.cos(),
    );
    (r * ie * Vector3::x(), u * ie * es * Vector3::z())
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    /// Upstream `dataLog` and `dataNewAtmoLog`, every update.
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub atmosphere: Vec<(u64, AtmosphereMsg)>,
    pub mu_m3ps2: f64,
}

pub fn run(planet: PlanetCase) -> Results {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"));
    let (table_file, format) = match planet {
        PlanetCase::Earth => ("EarthGRAMNominal.txt", AtmTableFormat::EarthGram),
        PlanetCase::Mars => ("MarsGRAMNominal.txt", AtmTableFormat::MarsGram),
    };
    let table = read_atm_table(root.join("assets/atmosphere").join(table_file), format)
        .unwrap_or_else(|error| panic!("{error} (just fetch-environment-data)"));
    let mut atmosphere = TabularAtmosphere::new(TabularAtmosphereConfig::from_table(
        "tabularAtmosphere",
        planet.atmosphere_radius_m(),
        table,
    ));

    let degrees = std::f64::consts::PI / 180.0;
    let (r, u, gam) = match planet {
        PlanetCase::Earth => (6503.0 * 1000.0, 11.2 * 1000.0, -5.15 * degrees),
        PlanetCase::Mars => ((3397.2 + 125.0) * 1000.0, 6.0 * 1000.0, -10.0 * degrees),
    };
    let (position_m, velocity_mps) = sph2rv(r, 0.0, 0.0, u, gam, std::f64::consts::FRAC_PI_2);
    let duration_nanos = match planet {
        PlanetCase::Earth => 300_000_000_000,
        PlanetCase::Mars => 400_000_000_000,
    };

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 2530.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    let body = planet.body();
    spacecraft
        .add_grav_body(body.point_mass(true))
        .expect("one central gravity body");
    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "DragEff".to_string(),
        projected_area_m2: 10.0,
        drag_coeff: 2.2,
        com_offset_m: Vector3::new(1.0, 0.0, 0.0),
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    let mut navigation = SimpleNavigation::new("simpleNav");

    let mut csv = scenario_csv(&format!("scenarioAerocapture{}", planet.label()));
    let csv_path = csv.config.output_path.clone();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut atmosphere_log = MessageLog::<AtmosphereMsg>::default();
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
        connect!(&simulation,
            &atmosphere.output_atmosphere_msg => &mut drag.input_atmosphere_msg,
            &spacecraft.state_out => &mut atmosphere.input_state_msg,
            &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &atmosphere.output_atmosphere_msg => csv.add_source::<AtmosphereMsg>(
                CsvSourceConfig::columns([(
                    "neutral_density_kgpm3",
                    "neutralDensity_kg_per_m3",
                )]),
            ),
            &spacecraft.state_out => &mut state_log.input,
            &atmosphere.output_atmosphere_msg => &mut atmosphere_log.input,
        );
        spacecraft.add_dynamic_effector(drag);
        schedule! { simulation,
            "atmosphere" => &mut atmosphere, period=STEP_NANOS, priority=30;
            "navigation" => &mut navigation, period=STEP_NANOS, priority=25;
            "spacecraft" => &mut spacecraft, period=STEP_NANOS, priority=20;
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "state_log" => &mut state_log, period=STEP_NANOS, priority=0;
            "atmosphere_log" => &mut atmosphere_log, period=STEP_NANOS, priority=0;
        }
        simulation.run_for(duration_nanos);
    }
    csv.flush().expect("flush scenario CSV");
    Results {
        csv_path,
        states: state_log.into_samples(),
        atmosphere: atmosphere_log.into_samples(),
        mu_m3ps2: body.mu_m3ps2(),
    }
}

/// Specific orbital energy (m^2/s^2), upstream's `-mu / (2 a)`.
pub fn energy(mu: f64, state: &SpacecraftStateMsg) -> f64 {
    0.5 * state.velocity_mps.norm_squared() - mu / state.position_m.norm()
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let planets = if args.switch("all") {
        TEST_CASES.to_vec()
    } else {
        match args.value("planet", "Mars".to_string()).as_str() {
            "Earth" => vec![PlanetCase::Earth],
            "Mars" => vec![PlanetCase::Mars],
            other => args.fail(&format!("unknown planet case {other:?}")),
        }
    };
    for planet in planets {
        let results = run(planet);
        let first = &results.states[0].1;
        let last = &results.states.last().unwrap().1;
        println!(
            "wrote {} (energy {:.3} -> {:.3} km^2/s^2)",
            results.csv_path.display(),
            energy(results.mu_m3ps2, first) * 1.0e-6,
            energy(results.mu_m3ps2, last) * 1.0e-6
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{TEST_CASES, energy, run};

    /// `test_scenarioAerocapture.py` runs both planets without checks; the
    /// arrival must be hyperbolic, the final orbit bound, with drag acting
    /// in between and the spacecraft above the surface throughout.
    #[test]
    fn hyperbolic_arrivals_are_captured() {
        for planet in TEST_CASES {
            let results = run(planet);
            let mu = results.mu_m3ps2;
            let first = &results.states[0].1;
            let last = &results.states.last().unwrap().1;
            assert!(
                energy(mu, first) > 0.0,
                "{planet:?}: arrival not hyperbolic"
            );
            assert!(energy(mu, last) < 0.0, "{planet:?}: not captured");
            assert!(
                results
                    .atmosphere
                    .iter()
                    .any(|(_, atmosphere)| atmosphere.neutral_density_kgpm3 > 0.0)
            );
            let lowest = results
                .states
                .iter()
                .map(|(_, state)| state.position_m.norm() - planet.atmosphere_radius_m())
                .fold(f64::INFINITY, f64::min);
            assert!(lowest > 0.0, "{planet:?}: lowest altitude {lowest} m");
        }
    }
}
