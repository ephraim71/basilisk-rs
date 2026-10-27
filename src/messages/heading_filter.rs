//! Heading estimator filter state (upstream `HeadingFilterMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{HEAD_N_STATES_SWITCH, OPNAV_MEAS};

/// Filter-state output of `headingSuKF`: upstream `HeadingFilterMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct HeadingFilterMsg {
    /// Upstream `timeTag` \[s\]: time of validity of the output.
    pub time_tag_s: f64,
    /// Upstream `covar` [-]: filter covariance. Upstream stores it as a flat
    /// row-major `double[HEAD_N_STATES_SWITCH * HEAD_N_STATES_SWITCH]`;
    /// convert with `SMatrix::from_row_slice`.
    pub covar: SMatrix<f64, HEAD_N_STATES_SWITCH, HEAD_N_STATES_SWITCH>,
    /// Upstream `state` [-]: estimated filter state.
    pub state: SVector<f64, HEAD_N_STATES_SWITCH>,
    /// Upstream `stateError` [-]: deviation of the state from the reference
    /// state.
    pub state_error: SVector<f64, HEAD_N_STATES_SWITCH>,
    /// Upstream `postFitRes` (`double[3]`) [-]: post-fit residuals.
    pub post_fit_res: SVector<f64, OPNAV_MEAS>,
}

impl_telemetry!(HeadingFilterMsg {
    time_tag_s,
    covar,
    state,
    state_error,
    post_fit_res,
});
