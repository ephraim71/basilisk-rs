//! Circles found in a navigation image (upstream `OpNavCirclesMsgPayload`).

use nalgebra::{Matrix3, Vector2};
use serde::{Deserialize, Serialize};

use super::MAX_CIRCLE_NUM;
use super::flatten::impl_telemetry;

/// Planet circles extracted from one camera image: upstream
/// `OpNavCirclesMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct OpNavCirclesMsg {
    /// Upstream `timeTag` \[ns\]: vehicle time tag of the image.
    pub time_tag_nanos: u64,
    /// Upstream `valid` (`int`, 1 valid / 0 not).
    pub valid: bool,
    /// Upstream `cameraID` [-]: ID of the camera that took the image.
    pub camera_id: i64,
    /// Upstream `planetIds` [-]: ID of each identified body (stored as
    /// `double` upstream).
    pub planet_ids: [f64; MAX_CIRCLE_NUM],
    /// Upstream `circlesCenters` \[pixels\]: centre `(x, y)` of each circle.
    /// Upstream interleaves them in a flat `double[2 * MAX_CIRCLE_NUM]`
    /// (`x0, y0, x1, y1, …`); entry `i` here is upstream `[2i]`, `[2i + 1]`.
    pub circles_centers_px: [Vector2<f64>; MAX_CIRCLE_NUM],
    /// Upstream `circlesRadii` \[pixels\]: radius of each circle.
    pub circles_radii_px: [f64; MAX_CIRCLE_NUM],
    /// Upstream `uncertainty` [-]: covariance of `(x, y, rho)` of the main
    /// circle. Upstream stores it as a flat row-major `double[9]`; convert with
    /// `Matrix3::from_row_slice`.
    pub uncertainty: Matrix3<f64>,
}

impl_telemetry!(OpNavCirclesMsg {
    time_tag_nanos,
    valid,
    camera_id,
    planet_ids,
    circles_centers_px,
    circles_radii_px,
    uncertainty,
});
