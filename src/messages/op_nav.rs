//! Optical-navigation position measurement (upstream `OpNavMsgPayload`).

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Spacecraft position relative to a planet from optical navigation, in three
/// frames: upstream `OpNavMsg`.
///
/// Upstream stores each covariance as a flat row-major `double[9]`; convert
/// with `Matrix3::from_row_slice`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct OpNavMsg {
    /// Upstream `timeTag` \[s\]: time of validity of the measurement.
    pub time_tag_s: f64,
    /// Upstream `valid` (`int`, 1 valid / 0 invalid).
    pub valid: bool,
    /// Upstream `covar_N` [m^2]: position covariance, inertial components.
    pub covar_n_m2: Matrix3<f64>,
    /// Upstream `covar_B` [m^2]: position covariance, body components.
    pub covar_b_m2: Matrix3<f64>,
    /// Upstream `covar_C` [m^2]: position covariance, camera components.
    pub covar_c_m2: Matrix3<f64>,
    /// Upstream `r_BN_N` \[m\]: spacecraft position relative to the planet,
    /// inertial components.
    pub position_bn_n_m: Vector3<f64>,
    /// Upstream `r_BN_B` \[m\]: the same position, body components.
    pub position_bn_b_m: Vector3<f64>,
    /// Upstream `r_BN_C` \[m\]: the same position, camera components.
    pub position_bn_c_m: Vector3<f64>,
    /// Upstream `planetID` [-]: planet being navigated (Earth = 1, Mars = 2,
    /// Jupiter = 3).
    pub planet_id: i32,
    /// Upstream `faultDetected` (`int` used as a boolean).
    pub fault_detected: bool,
}

impl_telemetry!(OpNavMsg {
    time_tag_s,
    valid,
    covar_n_m2,
    covar_b_m2,
    covar_c_m2,
    position_bn_n_m,
    position_bn_b_m,
    position_bn_c_m,
    planet_id,
    fault_detected,
});
