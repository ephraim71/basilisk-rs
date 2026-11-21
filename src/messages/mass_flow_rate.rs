//! A scalar mass flow rate (upstream `MassFlowRateMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// A mass flow rate: upstream `MassFlowRateMsg`. A fuel tank reads it for
/// its leak rate (upstream `fuelLeakRateInMsg`) and for the propellant each
/// thruster it feeds consumes, which the thruster effectors publish.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MassFlowRateMsg {
    /// Upstream `massFlowRate` \[kg/s\].
    pub mass_flow_rate_kgps: f64,
}

impl_telemetry!(MassFlowRateMsg {
    mass_flow_rate_kgps
});
