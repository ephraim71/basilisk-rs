//! Sunline estimator filter state (upstream `SunlineFilterMsgPayload`).

use nalgebra::{SMatrix, SVector};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_N_CSS_MEAS, SKF_N_STATES};

/// Filter-state output of the sunline estimators (`sunlineUKF`, `sunlineEKF`,
/// `sunlineSuKF`, `sunlineSEKF`, `okeefeEKF`): upstream `SunlineFilterMsg`.
///
/// The payload is sized for [`SKF_N_STATES`] states. Filters with fewer states
/// fill the leading entries of `state`/`state_error`, and pack their smaller
/// covariance into the front of upstream's flat row-major `covar` array — use
/// [`Self::set_packed_covariance`] / [`Self::packed_covariance`] for those.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SunlineFilterMsg {
    /// Upstream `timeTag` \[s\]: time of validity of the output.
    pub time_tag_s: f64,
    /// Upstream `covar` [-]: filter covariance. Upstream stores it as a flat
    /// row-major `double[SKF_N_STATES * SKF_N_STATES]`; convert with
    /// `SMatrix::from_row_slice`.
    pub covar: SMatrix<f64, SKF_N_STATES, SKF_N_STATES>,
    /// Upstream `state` [-]: estimated filter state.
    pub state: SVector<f64, SKF_N_STATES>,
    /// Upstream `stateError` [-]: deviation of the state from the reference
    /// state.
    pub state_error: SVector<f64, SKF_N_STATES>,
    /// Upstream `postFitRes` [-]: post-fit residual per CSS measurement.
    pub post_fit_res: SVector<f64, MAX_N_CSS_MEAS>,
    /// Upstream `numObs` [-]: number of valid observations this frame.
    pub num_obs: usize,
}

impl SunlineFilterMsg {
    /// Stores an `N`×`N` covariance the way upstream's reduced-state filters
    /// do (`memmove(covar, filterCovar, N * N * sizeof(double))`): its `N²`
    /// row-major entries fill the first `N²` slots of the flat row-major
    /// 6×6 `covar`, and the remaining slots are left untouched.
    ///
    /// `N = SKF_N_STATES` is a plain assignment.
    ///
    /// # Panics
    ///
    /// If `N` exceeds [`SKF_N_STATES`].
    pub fn set_packed_covariance<const N: usize>(&mut self, covariance: &SMatrix<f64, N, N>) {
        assert!(
            N <= SKF_N_STATES,
            "a {N}x{N} covariance does not fit the {SKF_N_STATES}x{SKF_N_STATES} payload"
        );
        for (flat, value) in covariance.transpose().iter().enumerate() {
            self.covar[(flat / SKF_N_STATES, flat % SKF_N_STATES)] = *value;
        }
    }

    /// Reads back a covariance stored by [`Self::set_packed_covariance`]: the
    /// first `N²` slots of the flat row-major `covar`, as an `N`×`N` matrix.
    ///
    /// # Panics
    ///
    /// If `N` exceeds [`SKF_N_STATES`].
    pub fn packed_covariance<const N: usize>(&self) -> SMatrix<f64, N, N> {
        assert!(
            N <= SKF_N_STATES,
            "a {N}x{N} covariance does not fit the {SKF_N_STATES}x{SKF_N_STATES} payload"
        );
        SMatrix::from_fn(|row, column| {
            let flat = row * N + column;
            self.covar[(flat / SKF_N_STATES, flat % SKF_N_STATES)]
        })
    }
}

impl_telemetry!(SunlineFilterMsg {
    time_tag_s,
    covar,
    state,
    state_error,
    post_fit_res,
    num_obs,
});
