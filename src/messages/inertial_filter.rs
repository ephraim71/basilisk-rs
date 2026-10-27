//! Inertial attitude filter state (upstream `InertialFilterMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::AKF_N_STATES;
use super::flatten::impl_telemetry;

/// Filter-state output of `inertialUKF`: upstream `InertialFilterMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InertialFilterMsg {
    /// Upstream `timeTag` \[s\]: time of validity of the output.
    pub time_tag_s: f64,
    /// Upstream `covar` [-]: filter covariance. Upstream stores it as a flat
    /// row-major `double[AKF_N_STATES * AKF_N_STATES]`; convert with
    /// `SMatrix::from_row_slice`.
    pub covar: SMatrix<f64, AKF_N_STATES, AKF_N_STATES>,
    /// Upstream `state` [-]: estimated filter state.
    pub state: SVector<f64, AKF_N_STATES>,
    /// Upstream `numObs` [-]: number of valid observations this frame.
    pub num_obs: usize,
}

impl_telemetry!(InertialFilterMsg {
    time_tag_s,
    covar,
    state,
    num_obs,
});
