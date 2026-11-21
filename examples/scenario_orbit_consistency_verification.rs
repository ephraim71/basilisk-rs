//! Port of upstream `examples/scenarioOrbitConsistencyVerification.py`: a
//! sun-synchronous LEO (TerraSAR-X-like) propagated twice under the GGM03S
//! field, once with the Earth's rotation supplied and once without.
//!
//! With rotation, the Earth's `IAU_EARTH` orientation comes from the ANISE
//! ephemeris (upstream `createSpiceInterface` with `zeroBase = "Earth"`), so
//! the field is evaluated in the rotating Earth-fixed frame. Without it the
//! Earth keeps the identity orientation upstream falls back to, and the
//! tesseral and sectoral terms (order >= 1) no longer average out: the
//! eccentricity and inclination drift (upstream issue #1352).
//!
//! `--days`, `--degree` and `--step` set upstream's `simDays`, `gravDeg` and
//! `stepSeconds` (script defaults 30, 20, 10; upstream's test uses 12, 8, 30).
//! Writes `scenarioOrbitConsistencyVerificationRotating.csv` and
//! `...NonRotating.csv`, sampled hourly.
//!
//! `tests` ports `test_scenarioOrbitConsistencyVerification.py`: the
//! non-rotating eccentricity excursion exceeds the rotating one by more than
//! a factor of three.

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

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::state_to_elements;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use grav_factory::{Body, GGM03S};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{nano2sec, sec2nano};
use recording::{scenario_csv, state_columns};

const USAGE: &str = "\
scenario_orbit_consistency_verification [--days 30] [--degree 20] [--step 10]";

/// Upstream `R_N` (m) and `V_N` (m/s).
const INITIAL_POSITION_M: [f64; 3] = [-138514.048884, -888553.853077, 6818674.278964];
const INITIAL_VELOCITY_MPS: [f64; 3] = [-7306.6601, 2126.208457, 128.787296];
/// Upstream `sampleMinutes`.
const SAMPLE_MINUTES: f64 = 60.0;

/// Upstream `EPOCH = "2020 DEC 04 00:58:00"`.
fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2020, 12, 4, 0, 58, 0, 0)
}

/// Upstream `run(show_plots, simDays, gravDeg, stepSeconds)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub sim_days: f64,
    pub grav_degree: usize,
    pub step_seconds: f64,
}

/// One propagation: upstream `propagate`'s `(days, a, e, inc[deg])`.
#[derive(Clone, Debug, Default)]
pub struct ElementHistory {
    pub days: Vec<f64>,
    pub semi_major_axis_m: Vec<f64>,
    pub eccentricity: Vec<f64>,
    pub inclination_deg: Vec<f64>,
}

pub struct Results {
    pub rotating: ElementHistory,
    pub non_rotating: ElementHistory,
    /// Upstream `driftRatio`.
    pub drift_ratio: f64,
}

/// Upstream `propagate(useEarthRotation, simDays, gravDeg, sampleMinutes, stepSeconds)`.
pub fn propagate(use_earth_rotation: bool, config: Config) -> ElementHistory {
    let step_nanos = sec2nano(config.step_seconds);
    let sample_nanos = sec2nano(SAMPLE_MINUTES * 60.0);
    let mu = Body::Earth.mu_m3ps2();
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: step_nanos,
        initial_position_m: Vector3::from(INITIAL_POSITION_M),
        initial_velocity_mps: Vector3::from(INITIAL_VELOCITY_MPS),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Earth.spherical_harmonics(GGM03S, config.grav_degree, true))
        .expect("one central body");
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);

    let suffix = if use_earth_rotation {
        "Rotating"
    } else {
        "NonRotating"
    };
    let mut csv = scenario_csv(&format!("scenarioOrbitConsistencyVerification{suffix}"));
    let mut history = MessageLog::<SpacecraftStateMsg>::default();
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        if use_earth_rotation {
            connect!(&simulation,
                &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            );
            schedule! { simulation,
                "spiceObject" => &mut earth_ephemeris, period=step_nanos, priority=100;
            }
        }
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
        );
        // As upstream, the recorder (priority 1) runs before the spacecraft
        // (default priority -1): each sample is the state of the step before.
        schedule! { simulation,
            "csv" => &mut csv, period=sample_nanos, priority=1;
            "rec" => &mut history, period=sample_nanos, priority=1;
            "tsx" => &mut spacecraft, period=step_nanos, priority=-1;
        }
        simulation.run_for(sec2nano(config.sim_days * 86_400.0));
    }
    csv.flush().expect("flush scenario CSV");

    let mut elements = ElementHistory::default();
    for (time, state) in history.times_nanos.iter().zip(&history.values) {
        let oe = state_to_elements(mu, state.position_m, state.velocity_mps);
        elements.days.push(nano2sec(*time) / 86_400.0);
        elements.semi_major_axis_m.push(oe.semi_major_axis);
        elements.eccentricity.push(oe.eccentricity);
        elements
            .inclination_deg
            .push(oe.inclination_rad.to_degrees());
    }
    elements
}

/// Upstream `secularDrift`: the change of the one-day mean over the run.
fn secular_drift(days: &[f64], series: &[f64]) -> f64 {
    const WINDOW_DAYS: f64 = 1.0;
    let mean = |selected: Vec<f64>| selected.iter().sum::<f64>() / selected.len() as f64;
    let first_day = days[0];
    let last_day = *days.last().expect("samples");
    let first = mean(
        days.iter()
            .zip(series)
            .filter(|(day, _)| **day <= first_day + WINDOW_DAYS)
            .map(|(_, value)| *value)
            .collect(),
    );
    let last = mean(
        days.iter()
            .zip(series)
            .filter(|(day, _)| **day >= last_day - WINDOW_DAYS)
            .map(|(_, value)| *value)
            .collect(),
    );
    (last - first).abs()
}

pub fn run(config: Config) -> Results {
    let rotating = propagate(true, config);
    let non_rotating = propagate(false, config);
    let e_drift_rotating = secular_drift(&rotating.days, &rotating.eccentricity);
    let e_drift_non_rotating = secular_drift(&non_rotating.days, &non_rotating.eccentricity);
    let i_drift_rotating = secular_drift(&rotating.days, &rotating.inclination_deg);
    let i_drift_non_rotating = secular_drift(&non_rotating.days, &non_rotating.inclination_deg);
    let drift_ratio = if e_drift_rotating > 0.0 {
        e_drift_non_rotating / e_drift_rotating
    } else {
        f64::INFINITY
    };
    println!(
        "eccentricity excursion: rotating={e_drift_rotating:.3e}, non-rotating={e_drift_non_rotating:.3e}"
    );
    println!(
        "inclination excursion [deg]: rotating={i_drift_rotating:.4}, non-rotating={i_drift_non_rotating:.4}"
    );
    Results {
        rotating,
        non_rotating,
        drift_ratio,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let results = run(Config {
        sim_days: args.value("days", 30.0),
        grav_degree: args.value("degree", 20),
        step_seconds: args.value("step", 10.0),
    });
    println!("drift ratio {:.2}", results.drift_ratio);
}

#[cfg(test)]
mod tests {
    use super::{Config, run};

    /// `test_scenarioOrbitConsistencyVerification.py`: with `simDays = 12`,
    /// `gravDeg = 8`, `stepSeconds = 30`, `driftRatio > 3`.
    #[test]
    fn non_rotating_eccentricity_drift_dominates() {
        let results = run(Config {
            sim_days: 12.0,
            grav_degree: 8,
            step_seconds: 30.0,
        });
        assert!(
            results.drift_ratio > 3.0,
            "non-rotating eccentricity drift not dominant (ratio={})",
            results.drift_ratio
        );
    }
}
