//! Planet limb points found in a navigation image (upstream
//! `OpNavLimbMsgPayload`).

use nalgebra::Vector2;
use serde::{Deserialize, Serialize};

use super::MAX_LIMB_PNTS;
use super::flatten::impl_telemetry;

/// Planet limb points extracted from one camera image: upstream
/// `OpNavLimbMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OpNavLimbMsg {
    /// Upstream `timeTag` \[s\]: vehicle time tag of the image.
    pub time_tag_s: f64,
    /// Upstream `valid` (`int`, 1 valid / 0 not).
    pub valid: bool,
    /// Upstream `numLimbPoints` (`int32_t`) [-]: number of limb points found,
    /// the leading entries of `limb_points_px`.
    pub num_limb_points: usize,
    /// Upstream `cameraID` [-]: ID of the camera that took the image.
    pub camera_id: i64,
    /// Upstream `planetIds` [-]: ID of the identified body — a single `double`
    /// despite the plural name.
    pub planet_id: f64,
    /// Upstream `limbPoints` \[pixels\]: `(x, y)` of each limb point. Upstream
    /// interleaves them in a flat `double[2 * MAX_LIMB_PNTS]`
    /// (`x0, y0, x1, y1, …`); entry `i` here is upstream `[2i]`, `[2i + 1]`.
    #[serde(with = "super::big_array")]
    pub limb_points_px: [Vector2<f64>; MAX_LIMB_PNTS],
}

impl Default for OpNavLimbMsg {
    fn default() -> Self {
        Self {
            time_tag_s: 0.0,
            valid: false,
            num_limb_points: 0,
            camera_id: 0,
            planet_id: 0.0,
            limb_points_px: [Vector2::zeros(); MAX_LIMB_PNTS],
        }
    }
}

impl OpNavLimbMsg {
    /// The first `num_limb_points` entries (at most [`MAX_LIMB_PNTS`]).
    pub fn active_limb_points(&self) -> &[Vector2<f64>] {
        &self.limb_points_px[..self.num_limb_points.min(MAX_LIMB_PNTS)]
    }
}

impl_telemetry!(OpNavLimbMsg {
    time_tag_s,
    valid,
    num_limb_points,
    camera_id,
    planet_id,
    limb_points_px,
});
