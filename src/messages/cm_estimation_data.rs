//! Center-of-mass estimator output (upstream `CMEstDataMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Estimate and residuals of `thrustCMEstimation`: upstream `CMEstDataMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CmEstimationDataMsg {
    /// Upstream `attError` [-]: attitude convergence error
    /// `sqrt(|σ_BR|² + |ω_BR|²)`.
    pub att_error: f64,
    /// Upstream `state` \[m\]: estimated center-of-mass location relative to
    /// point B, body components.
    pub state_m: Vector3<f64>,
    /// Upstream `stateError` \[m\]: estimate minus the true location.
    pub state_error_m: Vector3<f64>,
    /// Upstream `covariance` \[m\]: despite the name (and upstream's `[m^2]`),
    /// the 1-σ standard deviation of each estimated component.
    pub covariance_m: Vector3<f64>,
    /// Upstream `preFitRes` \[N m\]: pre-fit torque residuals.
    pub pre_fit_res_nm: Vector3<f64>,
    /// Upstream `postFitRes` \[N m\]: post-fit torque residuals.
    pub post_fit_res_nm: Vector3<f64>,
}

impl_telemetry!(CmEstimationDataMsg {
    att_error,
    state_m,
    state_error_m,
    covariance_m,
    pre_fit_res_nm,
    post_fit_res_nm,
});
