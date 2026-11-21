//! Step command of a stepper motor (upstream `MotorStepCommandMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Number of steps commanded to a stepper motor: upstream
/// `MotorStepCommandMsg`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MotorStepCommandMsg {
    /// Upstream `stepsCommanded` \[steps\]: signed number of steps to take
    /// from the motor's angle when the command arrives.
    pub steps_commanded: i32,
    /// Sim time \[ns\] of the update that issued the command: the stand-in
    /// for upstream's message-header `timeWritten`, which the stepper motor
    /// compares with the last command's to tell a new command from a repeat.
    /// Producers stamp every new command with their update time, as for
    /// [`super::ThrusterCommandMsg::command_time_nanos`].
    pub command_time_nanos: u64,
}

impl_telemetry!(MotorStepCommandMsg {
    steps_commanded,
    command_time_nanos,
});
