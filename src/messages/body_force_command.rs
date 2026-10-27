//! Control force in body components (upstream `CmdForceBodyMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Requested control force, body components: upstream `CmdForceBodyMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct BodyForceCommandMsg {
    /// Upstream `forceRequestBody` \[N\]: requested control force, body
    /// components.
    pub force_request_body_n: Vector3<f64>,
}

impl_telemetry!(BodyForceCommandMsg {
    force_request_body_n
});
