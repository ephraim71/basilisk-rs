//! [`Latch`]: holds a copy of a message, taken when the latch runs.
//!
//! Two upstream behaviours need it:
//!
//! * a module that reads its input in `UpdateState` and uses it during the
//!   next integration (an `extForceTorque` command, a drag effector's
//!   density) sees the message as it was when it ran, while basilisk-rs's
//!   dynamic effectors read their inputs during the integration. Scheduling
//!   a latch where upstream's module runs, and connecting the effector to
//!   the latch, restores upstream's timing;
//! * a scenario reading a recorder's last sample between runs
//!   (`dataRec.v_BN_N[-1]`): a latch scheduled with the recorder keeps it
//!   where the scenario can read it ([`Output`] clones share the message).

#![allow(dead_code)]

use basilisk_rs::messages::{Input, Output, SimulationMessage};
use basilisk_rs::{Module, SimulationContext};

/// Copies `input` to `output` at every update; `output` starts at the
/// default (zero) message, as an unwritten upstream message reads.
#[derive(Debug)]
pub struct Latch<T> {
    pub input: Input<T>,
    pub output: Output<T>,
}

impl<T: SimulationMessage> Default for Latch<T> {
    fn default() -> Self {
        Self {
            input: Input::default(),
            output: Output::default(),
        }
    }
}

impl<T: SimulationMessage> Module for Latch<T> {
    fn init(&mut self) {
        self.output.write(T::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.output.write(self.input.read());
    }
}
