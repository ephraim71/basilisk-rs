//! Small-body navigation filter state (upstream `SmallBodyNavMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::SMALL_BODY_NAV_N_STATES;
use super::flatten::impl_telemetry;

/// State and covariance of `smallBodyNavEKF`: upstream `SmallBodyNavMsg`.
///
/// The state is the spacecraft position \[m\] and velocity \[m/s\] relative to
/// the small body in its Hill frame, then the small body's MRP attitude
/// relative to inertial and its angular velocity \[rad/s\] in body-fixed
/// components.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SmallBodyNavMsg {
    /// Upstream `state`: current state estimate.
    pub state: SVector<f64, SMALL_BODY_NAV_N_STATES>,
    /// Upstream `covar`: current state covariance. Upstream stores it as a
    /// row-major `double[12][12]`.
    pub covar: SMatrix<f64, SMALL_BODY_NAV_N_STATES, SMALL_BODY_NAV_N_STATES>,
}

impl_telemetry!(SmallBodyNavMsg { state, covar });
