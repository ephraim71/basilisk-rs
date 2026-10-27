//! Thruster force commands (upstream `THRArrayCmdForceMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;
use super::{MAX_EFF_COUNT, filled_prefix};

/// Commanded force of every thruster: upstream `THRArrayCmdForceMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ThrusterArrayForceCommandMsg {
    /// Upstream `thrForce` \[N\]: commanded force per thruster, in thruster
    /// order.
    #[serde(with = "super::big_array")]
    pub thrust_force_n: [f64; MAX_EFF_COUNT],
}

impl Default for ThrusterArrayForceCommandMsg {
    fn default() -> Self {
        Self {
            thrust_force_n: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl ThrusterArrayForceCommandMsg {
    /// Builds the message from the forces of the thrusters in use; the
    /// remaining entries stay zero.
    ///
    /// # Panics
    ///
    /// If more than [`MAX_EFF_COUNT`] values are given.
    pub fn from_active(thrust_force_n: &[f64]) -> Self {
        Self {
            thrust_force_n: filled_prefix(thrust_force_n, "thruster force commands"),
        }
    }
}

impl_telemetry!(ThrusterArrayForceCommandMsg { thrust_force_n });
