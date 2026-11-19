//! Small-body navigation UKF state (upstream `SmallBodyNavUKFMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::SMALL_BODY_NAV_UKF_N_STATES;
use super::flatten::impl_telemetry;

/// State and covariance of `smallBodyNavUKF`: upstream `SmallBodyNavUKFMsg`.
///
/// The state is the spacecraft position \[m\], velocity \[m/s\] and
/// non-Keplerian acceleration \[m/s²\] relative to the small body, in its
/// body-fixed frame.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SmallBodyNavUkfMsg {
    /// Upstream `state`: current state estimate.
    pub state: SVector<f64, SMALL_BODY_NAV_UKF_N_STATES>,
    /// Upstream `covar`: current state covariance. Upstream stores it as a
    /// row-major `double[9][9]`.
    pub covar: SMatrix<f64, SMALL_BODY_NAV_UKF_N_STATES, SMALL_BODY_NAV_UKF_N_STATES>,
}

impl_telemetry!(SmallBodyNavUkfMsg { state, covar });
