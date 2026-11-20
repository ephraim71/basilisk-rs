//! Setup shared by the ports of upstream's formation-flying example scenarios
//! (`scenarioFormationBasic`, `scenarioFormationMeanOEFeedback`,
//! `scenarioFormationReconfig`, `scenarioRendezVous`): rigid hubs on Earth
//! orbits, the chief/deputy state columns of their CSV files, and upstream's
//! `unitTestSupport` array comparisons for the ported truth checks.
//!
//! Include it next to `common.rs`:
//!
//! ```ignore
//! #[path = "support/common.rs"]
//! mod common;
//! #[path = "support/formation.rs"]
//! mod formation;
//! ```

#![allow(dead_code)]

use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{
    eccentric_to_true_anomaly, mean_to_eccentric_anomaly,
};
use nalgebra::{Matrix3, Vector3};

use super::common::vector_columns;

/// Upstream `I = [900., 0., 0., 0., 800., 0., 0., 0., 600.]` \[kg m^2\], the
/// hub inertia of every formation scenario's (first) spacecraft.
pub fn hub_inertia_kg_m2() -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0))
}

/// The initial hub state of one spacecraft: upstream `hub.r_CN_NInit`,
/// `v_CN_NInit`, `sigma_BNInit` and `omega_BN_BInit`.
#[derive(Clone, Copy, Debug)]
pub struct InitialState {
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    pub sigma_bn: Vector3<f64>,
    pub omega_bn_b_radps: Vector3<f64>,
}

impl InitialState {
    /// At `(position_m, velocity_mps)`, aligned with `N` and not rotating
    /// (upstream's defaults when a scenario sets no attitude).
    pub fn at_rest((position_m, velocity_mps): (Vector3<f64>, Vector3<f64>)) -> Self {
        Self {
            position_m,
            velocity_mps,
            sigma_bn: Vector3::zeros(),
            omega_bn_b_radps: Vector3::zeros(),
        }
    }

    pub fn with_attitude(mut self, sigma_bn: [f64; 3], omega_bn_b_radps: [f64; 3]) -> Self {
        self.sigma_bn = Vector3::from(sigma_bn);
        self.omega_bn_b_radps = Vector3::from(omega_bn_b_radps);
        self
    }
}

/// Upstream `spacecraft.Spacecraft()` with `mHub`, `r_BcB_B = 0`,
/// `IHubPntBc_B` and the initial state, integrated with RK4 every
/// `step_nanos`.
pub fn rigid_spacecraft(
    mass_kg: f64,
    inertia_kg_m2: Matrix3<f64>,
    step_nanos: u64,
    initial: &InitialState,
) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2,
        integration_step_nanos: step_nanos,
        initial_position_m: initial.position_m,
        initial_velocity_mps: initial.velocity_mps,
        initial_sigma_bn: initial.sigma_bn,
        initial_omega_radps: initial.omega_bn_b_radps,
        integrator: None,
    })
}

/// Upstream `orbitalMotion.E2f(orbitalMotion.M2E(M, e), e)`.
pub fn mean_to_true_anomaly(mean_anomaly_rad: f64, eccentricity: f64) -> f64 {
    eccentric_to_true_anomaly(
        mean_to_eccentric_anomaly(mean_anomaly_rad, eccentricity),
        eccentricity,
    )
}

/// `<prefix>_r_BN_N_m_i` and `<prefix>_v_BN_N_m_per_s_i` from a spacecraft
/// state message: one spacecraft of a formation.
pub fn spacecraft_state_columns(prefix: &str) -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_m", &format!("{prefix}_r_BN_N_m"))
            .into_iter()
            .chain(vector_columns(
                "velocity_mps",
                &format!("{prefix}_v_BN_N_m_per_s"),
            )),
    )
}

/// Upstream `data[::skip]`.
pub fn every_nth<T: Copy>(values: &[T], skip: usize) -> Vec<T> {
    values.iter().step_by(skip).copied().collect()
}

/// Upstream `unitTestSupport.compareArrayRelative`: every component of each
/// `data` row within `accuracy` of the `truth` row, relative to the truth
/// (an exact match where the truth is zero). Returns the failure messages.
pub fn compare_array_relative(
    truth: &[[f64; 3]],
    data: &[Vector3<f64>],
    accuracy: f64,
    label: &str,
) -> Vec<String> {
    compare_rows(truth, data, label, |value, expected| {
        if expected == 0.0 {
            value == 0.0
        } else {
            ((value - expected) / expected).abs() <= accuracy
        }
    })
}

/// Upstream `unitTestSupport.compareArray`: every component of each `data`
/// row within `accuracy` of the `truth` row. Returns the failure messages.
pub fn compare_array(
    truth: &[[f64; 3]],
    data: &[Vector3<f64>],
    accuracy: f64,
    label: &str,
) -> Vec<String> {
    compare_rows(truth, data, label, |value, expected| {
        (value - expected).abs() <= accuracy
    })
}

fn compare_rows(
    truth: &[[f64; 3]],
    data: &[Vector3<f64>],
    label: &str,
    equal: impl Fn(f64, f64) -> bool,
) -> Vec<String> {
    if truth.len() != data.len() {
        return vec![format!(
            "{label}: {} rows, truth has {}",
            data.len(),
            truth.len()
        )];
    }
    truth
        .iter()
        .zip(data)
        .enumerate()
        .filter(|(_, (expected, value))| {
            value.iter().any(|component| component.is_nan())
                || (0..3).any(|axis| !equal(value[axis], expected[axis]))
        })
        .map(|(row, (expected, value))| {
            format!(
                "{label} row {row}: [{:.10e}, {:.10e}, {:.10e}], truth {expected:?}",
                value.x, value.y, value.z
            )
        })
        .collect()
}
