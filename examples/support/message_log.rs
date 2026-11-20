//! In-memory message history: the Rust counterpart of upstream's
//! `msg.recorder(samplingTime)`, for the values a scenario returns to its
//! caller (and its tests) rather than writes to CSV.

#![allow(dead_code)]

use basilisk_rs::messages::{Input, SimulationMessage};
use basilisk_rs::{Module, SimulationContext};

/// Samples one message every time it is scheduled.
#[derive(Debug)]
pub struct MessageLog<T> {
    /// The message to sample.
    pub input: Input<T>,
    /// Simulation time \[ns\] of each sample.
    pub times_nanos: Vec<u64>,
    /// The samples, in time order.
    pub values: Vec<T>,
}

impl<T> Default for MessageLog<T> {
    fn default() -> Self {
        Self {
            input: Input::default(),
            times_nanos: Vec::new(),
            values: Vec::new(),
        }
    }
}

impl<T> MessageLog<T> {
    /// Sample times \[s\].
    pub fn times_s(&self) -> Vec<f64> {
        self.times_nanos
            .iter()
            .map(|nanos| *nanos as f64 * 1.0e-9)
            .collect()
    }

    /// The last sample.
    pub fn last(&self) -> &T {
        self.values.last().expect("message log has no samples")
    }

    /// The sample taken at `nanos`.
    pub fn at(&self, nanos: u64) -> &T {
        let index = self
            .times_nanos
            .iter()
            .position(|time| *time == nanos)
            .unwrap_or_else(|| panic!("message log has no sample at {nanos} ns"));
        &self.values[index]
    }

    /// The samples taken at or after `nanos`.
    pub fn since(&self, nanos: u64) -> impl Iterator<Item = &T> {
        self.times_nanos
            .iter()
            .zip(&self.values)
            .filter(move |(time, _)| **time >= nanos)
            .map(|(_, value)| value)
    }
}

impl<T: SimulationMessage> Module for MessageLog<T> {
    fn init(&mut self) {
        self.times_nanos.clear();
        self.values.clear();
    }

    fn update(&mut self, context: &SimulationContext) {
        self.times_nanos.push(context.current_sim_nanos);
        self.values.push(self.input.read());
    }
}
