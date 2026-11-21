//! Motor force of every translational joint (upstream
//! `ArrayMotorForceMsgPayload`).

use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;
use super::flatten::impl_telemetry;

/// Motor force commanded along each translational joint axis: upstream
/// `ArrayMotorForceMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayMotorForceMsg {
    /// Upstream `motorForce` \[N\], one entry per joint.
    #[serde(with = "super::big_array")]
    pub motor_force_n: [f64; MAX_EFF_COUNT],
}

impl Default for ArrayMotorForceMsg {
    fn default() -> Self {
        Self {
            motor_force_n: [0.0; MAX_EFF_COUNT],
        }
    }
}

impl ArrayMotorForceMsg {
    /// A message whose first entries are `values`, the rest zero.
    ///
    /// # Panics
    ///
    /// If `values` has more than [`MAX_EFF_COUNT`] entries.
    pub fn from_active(values: &[f64]) -> Self {
        assert!(
            values.len() <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} motor forces are supported"
        );
        let mut message = Self::default();
        message.motor_force_n[..values.len()].copy_from_slice(values);
        message
    }
}

impl_telemetry!(ArrayMotorForceMsg { motor_force_n });
