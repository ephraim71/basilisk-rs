//! Control force in inertial components (upstream
//! `CmdForceInertialMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Requested control force, inertial components: upstream
/// `CmdForceInertialMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct InertialForceCommandMsg {
    /// Upstream `forceRequestInertial` \[N\]: requested control force, inertial
    /// components.
    pub force_request_inertial_n: Vector3<f64>,
}

impl_telemetry!(InertialForceCommandMsg {
    force_request_inertial_n
});
