//! Generic attitude coordinate set and rate (upstream `AttStateMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// A three-component attitude set and its rate, whatever coordinates the
/// producer uses: upstream `AttStateMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AttitudeStateMsg {
    /// Upstream `state`: attitude coordinates — \[rad\] for Euler angles,
    /// dimensionless for MRPs and the like.
    pub state: Vector3<f64>,
    /// Upstream `rate`: attitude rate — an angular velocity [rad/s] or the
    /// coordinate rates of `state`, depending on the producer.
    pub rate: Vector3<f64>,
}

impl_telemetry!(AttitudeStateMsg { state, rate });
