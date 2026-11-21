//! Thruster force commands to rounded on-time commands: port of upstream's
//! pure-Python `fswAlgorithms/effectorInterfaces/thrFiringRound/thrFiringRound.py`.
//!
//! Each update maps every thruster's force command to the fraction of the
//! control period it must fire at its maximum force,
//!
//! ```text
//! t_on = clip(T max(F, 0) / F_max, 0, T),
//! ```
//!
//! rounded to the nearest multiple of the on-time resolution, set to zero
//! below the minimum fire time, and clipped to the period again.
//!
//! Differences from upstream: upstream publishes one `THRArrayOnTimeCmdMsg`;
//! following basilisk-rs's per-device thruster ports, each thruster has its
//! own [`ThrusterCommandMsg`] output, registered with
//! [`ThrFiringRound::add_thruster_command_output`] in thruster order, with the
//! write time in `command_time_nanos` (the stand-in for the header
//! `timeWritten`; 0 at reset, as upstream's `write(payload)` there). The
//! thruster count is the number of outputs (upstream `setNumThrusters`).
//! Upstream raises `BasiliskError` for an unconnected input at `Reset`;
//! here `init` panics.

#![allow(dead_code)]

use basilisk_rs::messages::{Input, Output, ThrusterArrayForceCommandMsg, ThrusterCommandMsg};
use basilisk_rs::{Module, SimulationContext};

/// User-settable parameters of [`ThrFiringRound`].
#[derive(Clone, Debug, PartialEq)]
pub struct ThrFiringRoundConfig {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `controlPeriodSec` \[s\]; default 1 s.
    pub control_period_s: f64,
    /// Upstream `onTimeResolutionSec` \[s\]; default 0.01 s.
    pub on_time_resolution_s: f64,
    /// Upstream `thrMinFireTimeSec` \[s\]; default 0.
    pub thr_min_fire_time_s: f64,
    /// Upstream `thrForceMax` \[N\]: one value for every thruster, or one
    /// per thruster; default 1 N.
    pub thr_force_max_n: Vec<f64>,
}

impl Default for ThrFiringRoundConfig {
    fn default() -> Self {
        Self {
            name: "thrFiringRound".to_string(),
            control_period_s: 1.0,
            on_time_resolution_s: 0.01,
            thr_min_fire_time_s: 0.0,
            thr_force_max_n: vec![1.0],
        }
    }
}

impl ThrFiringRoundConfig {
    /// Upstream `resolveThrForceMax`.
    pub fn resolve_thr_force_max(&self, count: usize) -> Result<Vec<f64>, String> {
        match self.thr_force_max_n.as_slice() {
            [value] => Ok(vec![*value; count]),
            values if values.len() == count => Ok(values.to_vec()),
            values => Err(format!(
                "thrForceMax vector length {} does not match nThr {count}.",
                values.len()
            )),
        }
    }

    /// The on-time of one thruster for the force command `force_n`.
    pub fn on_time(&self, force_n: f64, force_max_n: f64) -> f64 {
        let period = self.control_period_s;
        let on_time = (period * force_n.max(0.0) / force_max_n).clamp(0.0, period);
        let mut on_time =
            (on_time / self.on_time_resolution_s).round_ties_even() * self.on_time_resolution_s;
        if on_time < self.thr_min_fire_time_s {
            on_time = 0.0;
        }
        on_time.clamp(0.0, period)
    }
}

/// Upstream `ThrFiringRound`.
#[derive(Debug)]
pub struct ThrFiringRound {
    pub config: ThrFiringRoundConfig,
    /// Upstream `thrForceInMsg` \[N\].
    pub thr_force_in_msg: Input<ThrusterArrayForceCommandMsg>,
    /// Upstream `thrOnTimeOutMsg`, one output per thruster.
    pub thr_on_time_out_msgs: Vec<Output<ThrusterCommandMsg>>,
}

impl ThrFiringRound {
    pub fn new(config: ThrFiringRoundConfig) -> Self {
        Self {
            config,
            thr_force_in_msg: Input::default(),
            thr_on_time_out_msgs: Vec::new(),
        }
    }

    /// Registers the on-time output of the next thruster.
    pub fn add_thruster_command_output(&mut self) -> &Output<ThrusterCommandMsg> {
        self.thr_on_time_out_msgs.push(Output::default());
        self.thr_on_time_out_msgs.last().expect("just added")
    }

    /// The on-times of the force commands `forces_n` (upstream
    /// `UpdateState`'s arithmetic).
    pub fn on_times(&self, forces_n: &[f64]) -> Vec<f64> {
        let count = self.thr_on_time_out_msgs.len();
        let maximum = self
            .config
            .resolve_thr_force_max(count)
            .unwrap_or_else(|error| panic!("{}: {error}", self.config.name));
        forces_n
            .iter()
            .zip(maximum)
            .take(count)
            .map(|(force, force_max)| self.config.on_time(*force, force_max))
            .collect()
    }
}

impl Module for ThrFiringRound {
    fn init(&mut self) {
        assert!(
            self.thr_force_in_msg.is_connected(),
            "ThrFiringRound.thrForceInMsg was not linked."
        );
        for output in &self.thr_on_time_out_msgs {
            output.write(ThrusterCommandMsg::default());
        }
    }

    fn update(&mut self, context: &SimulationContext) {
        let forces = self.thr_force_in_msg.read().thrust_force_n;
        for (output, on_time_s) in self.thr_on_time_out_msgs.iter().zip(self.on_times(&forces)) {
            output.write(ThrusterCommandMsg {
                on_time_s,
                command_time_nanos: context.current_sim_nanos,
            });
        }
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `thrFiringRound/_UnitTest/test_thrFiringRound.py`.

    use super::*;
    use hifitime::Epoch;

    fn context(nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos: nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn run(config: ThrFiringRoundConfig, forces: &[f64], thrusters: usize) -> Vec<f64> {
        let mut module = ThrFiringRound::new(config);
        for _ in 0..thrusters {
            module.add_thruster_command_output();
        }
        let command = Output::new(ThrusterArrayForceCommandMsg::from_active(forces));
        command.connect_to(&mut module.thr_force_in_msg);
        module.init();
        module.update(&context(1));
        module
            .thr_on_time_out_msgs
            .iter()
            .map(|output| {
                let message = output.read();
                assert_eq!(message.command_time_nanos, 1);
                message.on_time_s
            })
            .collect()
    }

    /// Upstream `test_thr_firing_round_output_payload`: forces 0.24, 2.0 and
    /// -1.0 N of thrusters limited to 1 and 2 N fire 0.2 and 1.0 s of a 1 s
    /// period at 0.1 s resolution.
    #[test]
    fn output_payload() {
        let config = ThrFiringRoundConfig {
            control_period_s: 1.0,
            on_time_resolution_s: 0.1,
            thr_force_max_n: vec![1.0, 2.0],
            ..ThrFiringRoundConfig::default()
        };
        let on_times = run(config, &[0.24, 2.0, -1.0], 2);
        assert!((on_times[0] - 0.2).abs() < 1e-12 && (on_times[1] - 1.0).abs() < 1e-12);
    }

    /// Upstream `test_thr_firing_round_configuration_helpers`.
    #[test]
    fn configuration_helpers() {
        let mut config = ThrFiringRoundConfig {
            thr_force_max_n: vec![3.0],
            ..ThrFiringRoundConfig::default()
        };
        assert_eq!(config.resolve_thr_force_max(3).unwrap(), vec![3.0; 3]);
        config.thr_force_max_n = vec![1.0, 2.0];
        assert_eq!(config.resolve_thr_force_max(2).unwrap(), vec![1.0, 2.0]);
        assert!(config.resolve_thr_force_max(3).is_err());
    }

    /// Upstream `test_thr_firing_round_minimum_fire_time`.
    #[test]
    fn minimum_fire_time() {
        let config = ThrFiringRoundConfig {
            control_period_s: 1.0,
            on_time_resolution_s: 0.1,
            thr_min_fire_time_s: 0.15,
            thr_force_max_n: vec![1.0],
            ..ThrFiringRoundConfig::default()
        };
        let on_times = run(config, &[0.14, 0.16], 2);
        assert!(on_times[0] == 0.0 && (on_times[1] - 0.2).abs() < 1e-12);
    }

    /// Upstream `test_reset_rejects_missing_input_message`.
    #[test]
    #[should_panic(expected = "ThrFiringRound.thrForceInMsg")]
    fn missing_input_is_rejected() {
        ThrFiringRound::new(ThrFiringRoundConfig::default()).init();
    }
}
