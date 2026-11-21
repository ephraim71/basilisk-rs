//! Cannonball drag geometry (upstream `DragGeometryMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Drag properties of a body seen as a sphere: upstream `DragGeometryMsg`,
/// read by [`CannonballDrag`](crate::mujoco_dynamics::cannonball_drag::CannonballDrag).
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct DragGeometryMsg {
    /// Upstream `projectedArea` \[m^2\]: area projected along the velocity.
    pub projected_area_m2: f64,
    /// Upstream `dragCoeff` [-]: drag coefficient.
    pub drag_coeff: f64,
    /// Upstream `r_CP_S` \[m\]: center of pressure relative to the origin of
    /// the reference frame `S`, in `S` components.
    pub r_cp_s_m: Vector3<f64>,
}

impl_telemetry!(DragGeometryMsg {
    projected_area_m2,
    drag_coeff,
    r_cp_s_m,
});
