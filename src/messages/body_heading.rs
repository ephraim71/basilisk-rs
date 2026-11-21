//! Heading to a target in the body frame (upstream `BodyHeadingMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Unit direction from the spacecraft to an arbitrary target `X`, body
/// components: upstream `BodyHeadingMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BodyHeadingMsg {
    /// Upstream `rHat_XB_B` [-]: unit heading to `X`, body components.
    pub r_hat_xb_b: Vector3<f64>,
}

impl_telemetry!(BodyHeadingMsg { r_hat_xb_b });
