//! Constant-power device: port of upstream Basilisk `simplePowerSink`.
//!
//! A [`PowerNode`] whose net power is a fixed value,
//! [`SimplePowerSinkModel::node_power_out_w`] (upstream `nodePowerOut`):
//! negative for a load, positive for a source. Like every node it publishes
//! zero while switched off; see [`super::power_node`].

use super::power_node::{PowerModel, PowerNode};

/// The constant power of a [`SimplePowerSink`]: upstream `SimplePowerSink`'s
/// `evaluatePowerModel`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SimplePowerSinkModel {
    /// Upstream `nodePowerOut` \[W\]: power supplied (+) or drawn (-); zero
    /// by default.
    pub node_power_out_w: f64,
}

impl PowerModel for SimplePowerSinkModel {
    fn evaluate_power_model(&mut self) -> f64 {
        self.node_power_out_w
    }
}

/// Upstream `SimplePowerSink`: a [`PowerNode`] running
/// [`SimplePowerSinkModel`].
pub type SimplePowerSink = PowerNode<SimplePowerSinkModel>;

#[cfg(test)]
mod tests {
    //! Ports upstream `simplePowerSink/_UnitTest/test_unitSimplePowerSink.py`
    //! (`defaultPowerSink` and `statusPowerSink`) with its 1e-16 accuracy.

    use hifitime::Epoch;

    use crate::messages::{DeviceState, DeviceStatusMsg, Output, PowerNodeUsageMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;

    use super::{SimplePowerSink, SimplePowerSinkModel};

    const STEP_NANOS: u64 = 500_000_000;

    /// Runs `sink` for one second at 0.5 s and returns the logged net power.
    fn logged_power(sink: &mut SimplePowerSink) -> Vec<f64> {
        let mut recorder = MessageRecorder::<PowerNodeUsageMsg>::default();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&sink.node_power_out_msg, &mut recorder.input_msg);
            sim.add_module("powerSink", sink, STEP_NANOS, 1);
            sim.add_module("recorder", &mut recorder, STEP_NANOS, 0);
            sim.run_for(1_000_000_000);
        }
        recorder
            .samples
            .iter()
            .map(|sample| sample.net_power_w)
            .collect()
    }

    fn assert_power(actual: &[f64], truth: f64) {
        assert_eq!(actual.len(), 3);
        for power in actual {
            assert!((power - truth).abs() <= 1.0e-16, "{power} != {truth}");
        }
    }

    /// Upstream `defaultPowerSink`: a 10 W node with no status message.
    #[test]
    fn default_power_sink_publishes_its_power() {
        let mut sink = SimplePowerSink::new(
            "powerSink",
            SimplePowerSinkModel {
                node_power_out_w: 10.0,
            },
        );
        assert_power(&logged_power(&mut sink), 10.0);
    }

    /// Upstream `statusPowerSink`: the same node switched off by its status
    /// message.
    #[test]
    fn status_message_switches_the_sink_off() {
        let mut sink = SimplePowerSink::new(
            "powerSink",
            SimplePowerSinkModel {
                node_power_out_w: 10.0,
            },
        );
        let status = Output::new(DeviceStatusMsg {
            device_status: DeviceState::Off,
        });
        status.connect_to(&mut sink.node_status_in_msg);
        assert_power(&logged_power(&mut sink), 0.0);
    }

    #[test]
    fn default_model_draws_nothing() {
        let mut sink = SimplePowerSink::new("idle", SimplePowerSinkModel::default());
        assert_power(&logged_power(&mut sink), 0.0);
    }
}
