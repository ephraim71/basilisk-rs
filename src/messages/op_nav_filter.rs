//! Relative orbit-determination filter state (upstream
//! `OpNavFilterMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{ODUKF_N_MEAS, ODUKF_N_STATES};

/// Filter-state output of `relativeODuKF`: upstream `OpNavFilterMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct OpNavFilterMsg {
    /// Upstream `timeTag` \[s\]: time of validity of the output.
    pub time_tag_s: f64,
    /// Upstream `covar` [-]: filter covariance (`relativeODuKF` scales its
    /// internal km-based covariance by 1e6 on output). Upstream stores it as a
    /// flat row-major
    /// `double[ODUKF_N_STATES * ODUKF_N_STATES]`; convert with
    /// `SMatrix::from_row_slice`.
    pub covar: SMatrix<f64, ODUKF_N_STATES, ODUKF_N_STATES>,
    /// Upstream `state` [-]: estimated filter state.
    pub state: SVector<f64, ODUKF_N_STATES>,
    /// Upstream `stateError` [-]: deviation of the state from the reference
    /// state.
    pub state_error: SVector<f64, ODUKF_N_STATES>,
    /// Upstream `postFitRes` [-]: post-fit residuals.
    pub post_fit_res: SVector<f64, ODUKF_N_MEAS>,
    /// Upstream `numObs` [-]: number of valid observations this frame.
    pub num_obs: usize,
}

impl_telemetry!(OpNavFilterMsg {
    time_tag_s,
    covar,
    state,
    state_error,
    post_fit_res,
    num_obs,
});
