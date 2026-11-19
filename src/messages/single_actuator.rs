//! Input to one actuator (upstream `SingleActuatorMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Real-valued command of a single actuator: upstream `SingleActuatorMsg`.
///
/// What the value means is up to the actuator it drives: a joint motor
/// torque \[N m\], a hub torque about one axis \[N m\] or a thruster force
/// \[N\].
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SingleActuatorMsg {
    /// Upstream `input` [-]: the actuator command.
    pub input: f64,
}

impl_telemetry!(SingleActuatorMsg { input });
