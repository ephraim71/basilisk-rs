//! Pieces shared by the ports of upstream's attitude-guidance example
//! scenarios built on the 750 kg hub of [`super::attitude`]
//! (`scenarioAttGuideHyperbolic`, `scenarioAttLocPoint`,
//! `scenarioInertialSpiral`): the hub on a task of any period, and the CSV
//! columns of the messages those scripts record.
//!
//! Include it next to `common.rs` and `attitude.rs`:
//!
//! ```ignore
//! #[path = "support/att_scenario.rs"]
//! mod att_scenario;
//! ```

#![allow(dead_code)]

use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use nalgebra::{Matrix3, Vector3};

use super::attitude::{HUB_MASS_KG, HubState};
use super::common::vector_columns;

/// Upstream `spacecraft.Spacecraft()` with `mHub = 750`, `r_BcB_B = 0` and
/// hub inertia `inertia_kg_m2`, integrated with RK4 on a task of
/// `step_nanos` (upstream `simulationTimeStep`).
pub fn rigid_spacecraft(
    inertia_kg_m2: Matrix3<f64>,
    initial: &HubState,
    step_nanos: u64,
) -> Spacecraft {
    Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
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

/// `sigma_BR_i` and `omega_BR_B_rad_per_s_i` of an `AttitudeGuidanceMsg`
/// (upstream `attGuidOutMsg.recorder()`'s `sigma_BR`, `omega_BR_B`).
pub fn guidance_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("sigma_br", "sigma_BR")
            .into_iter()
            .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
    )
}

/// `torqueRequestBody_Nm_i` of a `BodyTorqueCommandMsg`.
pub fn torque_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(vector_columns(
        "torque_request_body_nm",
        "torqueRequestBody_Nm",
    ))
}

/// `sigma_BN_i` and `omega_BN_B_rad_per_s_i` of a `NavigationAttitudeMsg`.
pub fn attitude_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("sigma_bn", "sigma_BN")
            .into_iter()
            .chain(vector_columns("omega_bn_b_radps", "omega_BN_B_rad_per_s")),
    )
}

/// `r_BN_N_m_i` and `v_BN_N_m_per_s_i` of a `NavigationTranslationMsg`.
pub fn translation_columns() -> CsvSourceConfig {
    CsvSourceConfig::columns(
        vector_columns("position_bn_n_m", "r_BN_N_m")
            .into_iter()
            .chain(vector_columns("velocity_bn_n_mps", "v_BN_N_m_per_s")),
    )
}
