//! Heading in the inertial frame (upstream `InertialHeadingMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Unit direction to an arbitrary target `X`, inertial components: upstream
/// `InertialHeadingMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InertialHeadingMsg {
    /// Upstream `rHat_XN_N` [-]: unit heading to `X`, inertial components.
    pub r_hat_xn_n: Vector3<f64>,
}

impl_telemetry!(InertialHeadingMsg { r_hat_xn_n });
