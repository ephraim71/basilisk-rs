//! Pixel-and-line bias filter state (upstream `PixelLineFilterMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{PIXLINE_N_MEAS, PIXLINE_N_STATES};

/// Filter-state output of `pixelLineBiasUKF`: upstream `PixelLineFilterMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct PixelLineFilterMsg {
    /// Upstream `timeTag` \[s\]: time of validity of the output.
    pub time_tag_s: f64,
    /// Upstream `covar` [-]: filter covariance. Upstream stores it as a flat
    /// row-major `double[PIXLINE_N_STATES * PIXLINE_N_STATES]`; convert with
    /// `SMatrix::from_row_slice`.
    pub covar: SMatrix<f64, PIXLINE_N_STATES, PIXLINE_N_STATES>,
    /// Upstream `state` [-]: estimated filter state.
    pub state: SVector<f64, PIXLINE_N_STATES>,
    /// Upstream `stateError` [-]: deviation of the state from the reference
    /// state.
    pub state_error: SVector<f64, PIXLINE_N_STATES>,
    /// Upstream `postFitRes` [-]: post-fit residuals.
    pub post_fit_res: SVector<f64, PIXLINE_N_MEAS>,
    /// Upstream `numObs` [-]: number of valid observations this frame.
    pub num_obs: usize,
}

impl_telemetry!(PixelLineFilterMsg {
    time_tag_s,
    covar,
    state,
    state_error,
    post_fit_res,
    num_obs,
});
