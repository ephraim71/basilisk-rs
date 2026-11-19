//! State of a stepper motor (upstream `StepperMotorMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Angle, rate, acceleration and step count of a stepper motor: upstream
/// `StepperMotorMsg`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct StepperMotorMsg {
    /// Upstream `theta` \[rad\]: motor angle.
    pub theta_rad: f64,
    /// Upstream `thetaDot` \[rad/s\]: motor angle rate.
    pub theta_dot_radps: f64,
    /// Upstream `thetaDDot` \[rad/s^2\]: motor angular acceleration.
    pub theta_ddot_radps2: f64,
    /// Upstream `stepsCommanded` \[steps\]: steps of the current command.
    pub steps_commanded: i32,
    /// Upstream `stepCount` \[steps\]: steps taken of the current command.
    pub step_count: i32,
}

impl_telemetry!(StepperMotorMsg {
    theta_rad,
    theta_dot_radps,
    theta_ddot_radps2,
    steps_commanded,
    step_count,
});
