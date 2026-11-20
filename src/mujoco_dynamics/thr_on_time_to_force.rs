//! Thruster on-time commands to thruster forces: port of upstream Basilisk
//! `simulation/mujocoDynamics/thrOnTimeToForce`.
//!
//! Each thruster fires at its fixed force while its latched on-time has not
//! run out: a new command sets the remaining time, and every later update
//! counts it down by the time since the previous update. The force goes to
//! a single actuator along the thruster's axis in a scene. A negative
//! on-time is treated as zero, with a warning.
//!
//! Upstream reads one `THRArrayOnTimeCmdMsg` and latches it when its header
//! `timeWritten` changes. Following basilisk-rs's per-device thruster ports,
//! here each thruster has its own [`ThrusterCommandMsg`] input, made by
//! [`ThrOnTimeToForce::new`] with its force output; a command is new when its
//! `command_time_nanos` (the stand-in for `timeWritten`, stamped by the FSW
//! producers) or its on-time differs from the last one latched, the rule of
//! [`ThrusterDynamicEffector`](crate::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector).
//! A thruster that latches a new command does not count down at that update,
//! as upstream skips the countdown after a new array command.

use crate::messages::{Input, Output, SingleActuatorMsg, ThrusterCommandMsg};
use crate::{Module, SimulationContext};

/// Remaining times below this count as zero (upstream's guard against
/// round-off).
const TIME_EPSILON_S: f64 = 1e-12;

/// User-settable parameters of [`ThrOnTimeToForce`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ThrOnTimeToForceConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `thrMag` \[N\]: each thruster's force while firing.
    pub thr_mag_n: Vec<f64>,
}

/// Upstream `ThrOnTimeToForce`.
#[derive(Debug)]
pub struct ThrOnTimeToForce {
    pub config: ThrOnTimeToForceConfig,
    /// Upstream `onTimeInMsg`, one input per thruster.
    pub on_time_in_msgs: Vec<Input<ThrusterCommandMsg>>,
    /// Upstream `thrusterForceOutMsgs` \[N\], one per thruster.
    pub thruster_force_out_msgs: Vec<Output<SingleActuatorMsg>>,
    firing_time_remaining_s: Vec<f64>,
    latched: Vec<Option<ThrusterCommandMsg>>,
    previous_update_time_s: f64,
}

impl ThrOnTimeToForce {
    /// The module with one command input and one force output per thruster
    /// of `config` (upstream `setThrMag` and one `addThruster` per thruster).
    pub fn new(config: ThrOnTimeToForceConfig) -> Self {
        let count = config.thr_mag_n.len();
        Self {
            config,
            on_time_in_msgs: (0..count).map(|_| Input::default()).collect(),
            thruster_force_out_msgs: (0..count).map(|_| Output::default()).collect(),
            firing_time_remaining_s: vec![0.0; count],
            latched: vec![None; count],
            previous_update_time_s: 0.0,
        }
    }

    /// Upstream `getThrMag` \[N\].
    pub fn thr_mag(&self) -> &[f64] {
        &self.config.thr_mag_n
    }

    /// Upstream `Reset`.
    ///
    /// # Panics
    ///
    /// If a thruster's command input is unconnected (upstream:
    /// "ThrOnTimeToForce.onTimeInMsg was not linked").
    fn reset_at(&mut self, current_sim_nanos: u64) {
        for (index, input) in self.on_time_in_msgs.iter().enumerate() {
            assert!(
                input.is_connected(),
                "ThrOnTimeToForce '{}': the on-time input of thruster {index} was not linked.",
                self.config.name
            );
        }
        self.firing_time_remaining_s = vec![0.0; self.config.thr_mag_n.len()];
        self.latched = vec![None; self.config.thr_mag_n.len()];
        self.previous_update_time_s = current_sim_nanos as f64 * 1e-9;
    }
}

impl Module for ThrOnTimeToForce {
    fn init(&mut self) {
        self.reset_at(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_at(context.current_sim_nanos);
    }

    /// Upstream `UpdateState`.
    fn update(&mut self, context: &SimulationContext) {
        let now_s = context.current_sim_nanos as f64 * 1e-9;
        let dt_s = (now_s - self.previous_update_time_s).max(0.0);
        self.previous_update_time_s = now_s;
        for index in 0..self.config.thr_mag_n.len() {
            let command = self.on_time_in_msgs[index].read();
            let is_new = self.latched[index].as_ref().is_none_or(|last| {
                command.command_time_nanos != last.command_time_nanos
                    || (command.on_time_s - last.on_time_s).abs() > 1.0e-15
            });
            let remaining = &mut self.firing_time_remaining_s[index];
            if is_new {
                if command.on_time_s < 0.0 {
                    log::warn!(
                        "ThrOnTimeToForce received negative on-time command ({} s) for thruster \
                         {index}. Setting to 0.",
                        command.on_time_s
                    );
                    *remaining = 0.0;
                } else {
                    *remaining = command.on_time_s;
                }
                self.latched[index] = Some(command);
            } else {
                *remaining = (*remaining - dt_s).max(0.0);
                if *remaining < TIME_EPSILON_S {
                    *remaining = 0.0;
                }
            }
            let input = if *remaining > TIME_EPSILON_S {
                self.config.thr_mag_n[index]
            } else {
                0.0
            };
            self.thruster_force_out_msgs[index].write(SingleActuatorMsg { input });
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_thrOnTimeToForce.py`, with one command input
    //! per thruster.

    use super::*;
    use crate::mujoco_dynamics::tests::context;

    fn command(on_time_s: f64, command_time_nanos: u64) -> ThrusterCommandMsg {
        ThrusterCommandMsg {
            on_time_s,
            command_time_nanos,
        }
    }

    /// Upstream `test_thrOnTimeToForceCountdown`: 0.2 s and 0.3 s pulses at
    /// a 0.1 s step fire for two and three updates.
    #[test]
    fn countdown_realizes_the_pulses() {
        let mut module = ThrOnTimeToForce::new(ThrOnTimeToForceConfig {
            name: "thrOnTimeToForceTag".to_string(),
            thr_mag_n: vec![2.0, 3.0],
        });
        assert_eq!(module.thr_mag(), &[2.0, 3.0]);
        let commands = [Output::new(command(0.2, 0)), Output::new(command(0.3, 0))];
        for (output, input) in commands.iter().zip(module.on_time_in_msgs.iter_mut()) {
            output.connect_to(input);
        }
        module.init();
        let mut outputs = [Vec::new(), Vec::new()];
        for step in 0..5 {
            module.update(&context(step * 100_000_000));
            for (series, output) in outputs.iter_mut().zip(&module.thruster_force_out_msgs) {
                series.push(output.read().input);
            }
        }
        assert_eq!(outputs[0], vec![2.0, 2.0, 0.0, 0.0, 0.0]);
        assert_eq!(outputs[1], vec![3.0, 3.0, 3.0, 0.0, 0.0]);
    }

    /// Upstream `test_thrOnTimeToForceGapThenFire`: a command written while
    /// the module is not running is latched in full at its first update.
    #[test]
    fn gap_then_fire_latches_the_full_on_time() {
        let mut module = ThrOnTimeToForce::new(ThrOnTimeToForceConfig {
            name: "thrOnTimeToForceTag".to_string(),
            thr_mag_n: vec![1.5],
        });
        let on_time = Output::new(command(0.0, 0));
        on_time.connect_to(&mut module.on_time_in_msgs[0]);
        module.init();
        // The task is disabled until 1.5 s; the command is rewritten at 1 s.
        on_time.write(command(0.2, 1_000_000_000));
        let mut output = Vec::new();
        for step in 15..19 {
            module.update(&context(step * 100_000_000));
            output.push(module.thruster_force_out_msgs[0].read().input);
        }
        assert_eq!(output, vec![1.5, 1.5, 0.0, 0.0]);
    }

    /// A negative on-time fires nothing.
    #[test]
    fn negative_on_time_is_zero() {
        let mut module = ThrOnTimeToForce::new(ThrOnTimeToForceConfig {
            name: "thr".to_string(),
            thr_mag_n: vec![1.0],
        });
        let on_time = Output::new(command(-1.0, 0));
        on_time.connect_to(&mut module.on_time_in_msgs[0]);
        module.init();
        module.update(&context(0));
        assert_eq!(module.thruster_force_out_msgs[0].read().input, 0.0);
    }

    #[test]
    #[should_panic(expected = "was not linked")]
    fn reset_needs_every_input() {
        ThrOnTimeToForce::new(ThrOnTimeToForceConfig {
            name: "thr".to_string(),
            thr_mag_n: vec![1.0],
        })
        .init();
    }
}
