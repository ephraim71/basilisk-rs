//! Antenna power draw: port of upstream Basilisk `antennaPower`.
//!
//! A [`PowerNode`] for one antenna. It reads the antenna's [`AntennaLogMsg`]
//! (upstream `simpleAntenna`'s `antennaOutMsg`) and draws a base power plus
//! the power of whatever the antenna is doing:
//!
//! | [`AntennaState`] | net power |
//! |---|---|
//! | `Off` | `-p_base` |
//! | `Rx` | `-P_Rx - p_base` |
//! | `Tx` | `-P_Tx - p_base` |
//! | `RxTx` | `-P_Rx - P_Tx - p_base` |
//!
//! # Differences from upstream
//!
//! * No message headers; the stand-in is "connected ⇒ written at the current
//!   update". As upstream with `antennaSetStateInMsg` unlinked, an unconnected
//!   antenna reads as the zeroed message, which is [`AntennaState::Off`], so
//!   the node draws its base power.
//! * [`AntennaState`] has no value outside the four states, so upstream's
//!   "invalid antenna state" error cannot occur.
//!
//! # Panics
//!
//! At initialization or reset, if `base_power_need_w` is negative (upstream
//! `bskError`).

use super::power_node::{PowerModel, PowerNode};
use crate::messages::{AntennaLogMsg, AntennaState, Input};

/// Base load and antenna input of an [`AntennaPower`]: upstream
/// `AntennaPower`'s own members.
#[derive(Clone, Debug, Default)]
pub struct AntennaPowerModel {
    /// Upstream `antennaSetStateInMsg`: the antenna's state and its receive
    /// and transmit powers.
    pub antenna_set_state_in_msg: Input<AntennaLogMsg>,
    /// Upstream `basePowerNeed` \[W\]: power the antenna draws just to be on;
    /// non-negative, zero by default.
    pub base_power_need_w: f64,
    /// Upstream `antennaStatusMsgBuffer`: the antenna read this update.
    antenna_status: AntennaLogMsg,
}

impl PowerModel for AntennaPowerModel {
    /// Upstream `customReset`.
    fn reset(&mut self, node_name: &str) {
        assert!(
            self.base_power_need_w >= 0.0,
            "antenna power '{node_name}': base_power_need_w cannot be negative, got {}",
            self.base_power_need_w
        );
    }

    /// Upstream `customReadMessages`: the antenna, zeroed (off) while
    /// unconnected. Always succeeds.
    fn read_messages(&mut self) -> bool {
        self.antenna_status = self.antenna_set_state_in_msg.read();
        true
    }

    /// Upstream `evaluatePowerModel`.
    fn evaluate_power_model(&mut self) -> f64 {
        let antenna = &self.antenna_status;
        match antenna.antenna_state {
            AntennaState::Off => -self.base_power_need_w,
            AntennaState::Rx => -antenna.p_rx_w - self.base_power_need_w,
            AntennaState::Tx => -antenna.p_tx_w - self.base_power_need_w,
            AntennaState::RxTx => -antenna.p_rx_w - antenna.p_tx_w - self.base_power_need_w,
        }
    }
}

/// Upstream `AntennaPower`: a [`PowerNode`] running [`AntennaPowerModel`].
pub type AntennaPower = PowerNode<AntennaPowerModel>;

#[cfg(test)]
mod tests {
    //! Ports upstream `antennaPower/_UnitTest/test_unitAntennaPower.py` over
    //! all four states and both base powers, with its 1e-12 accuracy.

    use hifitime::Epoch;

    use crate::messages::{
        AntennaLogMsg, AntennaState, DeviceState, DeviceStatusMsg, Output, PowerNodeUsageMsg,
    };
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{AntennaPower, AntennaPowerModel};

    const STEP_NANOS: u64 = 500_000_000;

    /// Runs `node` for one second at 0.5 s and returns the logged powers.
    fn logged_power(node: &mut AntennaPower) -> Vec<f64> {
        let mut recorder = MessageRecorder::<PowerNodeUsageMsg>::default();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&node.node_power_out_msg, &mut recorder.input_msg);
            sim.add_module("antennaPower", node, STEP_NANOS, 1);
            sim.add_module("recorder", &mut recorder, STEP_NANOS, 0);
            sim.run_for(1_000_000_000);
        }
        recorder
            .samples
            .iter()
            .map(|sample| sample.net_power_w)
            .collect()
    }

    /// Upstream `test_antennaPower`: a 2 W receiver and 10 W transmitter in
    /// each state, with no base load and with 5 W.
    #[test]
    fn power_follows_the_antenna_state() {
        let p_rx = 2.0;
        let p_tx = 10.0;
        for antenna_state in [
            AntennaState::Off,
            AntennaState::Rx,
            AntennaState::Tx,
            AntennaState::RxTx,
        ] {
            for base_power_need in [0.0, 5.0] {
                let antenna = Output::new(AntennaLogMsg {
                    antenna_state,
                    p_rx_w: p_rx,
                    p_tx_w: p_tx,
                    ..Default::default()
                });
                let mut model = AntennaPowerModel {
                    base_power_need_w: base_power_need,
                    ..Default::default()
                };
                antenna.connect_to(&mut model.antenna_set_state_in_msg);
                let mut node = AntennaPower::new("antennaPower", model);

                let true_power = match antenna_state {
                    AntennaState::Off => -base_power_need,
                    AntennaState::Rx => -(p_rx + base_power_need),
                    AntennaState::Tx => -(p_tx + base_power_need),
                    AntennaState::RxTx => -(p_rx + p_tx + base_power_need),
                };
                let power = logged_power(&mut node);
                assert_eq!(power.len(), 3);
                for (index, value) in power.iter().enumerate() {
                    assert!(
                        (value - true_power).abs() < 1.0e-12,
                        "{antenna_state:?}, base {base_power_need} W: \
                         netPower[{index}] {value} != {true_power}"
                    );
                }
            }
        }
    }

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Unconnected, the antenna reads as off: the node draws its base power.
    #[test]
    fn an_unconnected_antenna_draws_its_base_power() {
        let mut node = AntennaPower::new(
            "antennaPower",
            AntennaPowerModel {
                base_power_need_w: 3.0,
                ..Default::default()
            },
        );
        node.init();
        node.update(&context());
        assert_eq!(node.node_power_out_msg.read().net_power_w, -3.0);
    }

    /// The base-class switch overrides the antenna state.
    #[test]
    fn a_switched_off_node_draws_nothing() {
        let antenna = Output::new(AntennaLogMsg {
            antenna_state: AntennaState::RxTx,
            p_rx_w: 2.0,
            p_tx_w: 10.0,
            ..Default::default()
        });
        let status = Output::new(DeviceStatusMsg {
            device_status: DeviceState::Off,
        });
        let mut model = AntennaPowerModel {
            base_power_need_w: 5.0,
            ..Default::default()
        };
        antenna.connect_to(&mut model.antenna_set_state_in_msg);
        let mut node = AntennaPower::new("antennaPower", model);
        status.connect_to(&mut node.node_status_in_msg);
        assert_eq!(logged_power(&mut node), [0.0; 3]);
    }

    #[test]
    #[should_panic(expected = "base_power_need_w")]
    fn a_negative_base_power_is_rejected() {
        let mut node = AntennaPower::new(
            "antennaPower",
            AntennaPowerModel {
                base_power_need_w: -1.0,
                ..Default::default()
            },
        );
        node.init();
    }
}
