//! A landmark seen by a pinhole camera (upstream `LandmarkMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Visibility and image pixel of one landmark: upstream `LandmarkMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct LandmarkMsg {
    /// Upstream `isVisible` (`int`, 1 visible / 0 not): the landmark is in
    /// the camera's field of view, faces it, and is lit.
    pub is_visible: bool,
    /// Upstream `pL` \[pixels\]: the landmark's `(x, y)` pixel, counted from
    /// the image center; zero when it is not visible.
    pub pixel: [i32; 2],
}

impl_telemetry!(LandmarkMsg { is_visible, pixel });
