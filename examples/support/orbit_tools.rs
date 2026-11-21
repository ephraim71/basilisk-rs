//! Small orbit-scenario helpers: upstream `macros` time conversions and the
//! two-body (Kepler) solution the scenarios compare their runs against.

#![allow(dead_code)]

use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly, elements_to_state,
    mean_to_eccentric_anomaly, true_to_eccentric_anomaly,
};
use nalgebra::Vector3;

/// Upstream `macros.sec2nano`: `int(t * 1e9 + 0.5)`.
pub fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1.0e9 + 0.5) as u64
}

/// Upstream `macros.min2nano`.
pub fn min2nano(minutes: f64) -> u64 {
    (minutes * 1.0e9 * 60.0 + 0.5) as u64
}

/// Upstream `macros.day2nano`.
pub fn day2nano(days: f64) -> u64 {
    (days * 1.0e9 * 86_400.0 + 0.5) as u64
}

/// Upstream `NANO2SEC`.
pub fn nano2sec(nanos: u64) -> f64 {
    nanos as f64 * 1.0e-9
}

/// Orbit period `2 pi sqrt(a^3 / mu)` (s).
pub fn orbit_period_s(mu_m3ps2: f64, semi_major_axis_m: f64) -> f64 {
    2.0 * std::f64::consts::PI * (semi_major_axis_m.powi(3) / mu_m3ps2).sqrt()
}

/// Two-body state `elapsed_s` after `elements` (elliptic orbits): upstream's
/// `f2E`, `E2M`, `M2E`, `E2f` and `elem2rv` sequence in `scenarioBasicOrbit`'s
/// trajectory-difference plot and `test_scenarioOrbitFromOmm`.
pub fn kepler_state(
    mu_m3ps2: f64,
    elements: &ClassicElements,
    elapsed_s: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let e = elements.eccentricity;
    let initial_mean_anomaly =
        eccentric_to_mean_anomaly(true_to_eccentric_anomaly(elements.true_anomaly_rad, e), e);
    let mean_motion = (mu_m3ps2 / elements.semi_major_axis.powi(3)).sqrt();
    let mean_anomaly = initial_mean_anomaly + mean_motion * elapsed_s;
    let propagated = ClassicElements {
        true_anomaly_rad: eccentric_to_true_anomaly(mean_to_eccentric_anomaly(mean_anomaly, e), e),
        ..*elements
    };
    elements_to_state(mu_m3ps2, &propagated)
}

/// Runs `simulation` up to the absolute time `stop_nanos`: upstream
/// `ConfigureStopTime(stop)` + `ExecuteSimulation()`. `run_for` counts from
/// the last executed tick, which trails an off-grid stop time.
pub fn run_until(simulation: &mut basilisk_rs::simulation::Simulation<'_>, stop_nanos: u64) {
    let now = simulation.current_sim_nanos();
    simulation.run_for(stop_nanos.saturating_sub(now));
}
