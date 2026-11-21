//! Deputy state relative to the chief in the Hill frame (upstream
//! `HillRelStateMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Relative state of a deputy `D` with respect to a chief `C`, in Hill-frame
/// `H` components: upstream `HillRelStateMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct HillRelativeStateMsg {
    /// Upstream `r_DC_H` \[m\]: relative position, Hill components.
    pub position_dc_h_m: Vector3<f64>,
    /// Upstream `v_DC_H` [m/s]: relative velocity, Hill components.
    pub velocity_dc_h_mps: Vector3<f64>,
}

impl_telemetry!(HillRelativeStateMsg {
    position_dc_h_m,
    velocity_dc_h_mps,
});
