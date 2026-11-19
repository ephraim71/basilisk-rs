//! Power nodes: port of upstream Basilisk
//! `simulation/power/_GeneralModuleFiles/powerNodeBase`, the shared plumbing
//! of every device that supplies or draws electrical power.
//!
//! [`PowerNode`] is the module. Each update it reads its optional on/off
//! status, asks its [`PowerModel`] for the device's net power while the device
//! is on, and publishes that as a [`PowerNodeUsageMsg`] (positive supplied,
//! negative drawn). A power store such as
//! [`SimpleBattery`](super::simple_battery::SimpleBattery) or
//! [`SimplePowerMonitor`](super::simple_power_monitor::SimplePowerMonitor)
//! sums the nodes wired to its `power_node_in_msgs`.
//!
//! A device is a type implementing [`PowerModel`], used as `PowerNode<Model>`.
//! The ported ones are
//! [`SimplePowerSink`](super::simple_power_sink::SimplePowerSink),
//! [`SimpleSolarPanel`](super::simple_solar_panel::SimpleSolarPanel),
//! [`ReactionWheelPower`](super::reaction_wheel_power::ReactionWheelPower) and
//! [`AntennaPower`](super::antenna_power::AntennaPower); each one's own inputs
//! and parameters are on its [`PowerNode::model`].
//!
//! # Switching
//!
//! [`PowerNode::power_status`] (upstream `powerStatus`) starts
//! [`DeviceState::On`]. While `node_status_in_msg` is connected, every update
//! copies its `device_status` there first. Off, the node publishes zero power
//! without evaluating its model.
//!
//! # Differences from upstream
//!
//! * No message headers. Upstream publishes zero power while a linked input is
//!   unwritten. The stand-in is "connected ⇒ written at the current update":
//!   the status input is read whenever it is connected, and each model's
//!   [`PowerModel::read_messages`] documents what an unconnected input of its
//!   own means.
//! * Upstream's `powerStatus` is an integer whose non-zero values all mean on;
//!   here it is the status message's [`DeviceState`].
//! * Upstream's `nodePowerOut` is declared on the base class but read only by
//!   `simplePowerSink`, so it lives on
//!   [`SimplePowerSinkModel`](super::simple_power_sink::SimplePowerSinkModel).

use crate::messages::{DeviceState, DeviceStatusMsg, Input, Output, PowerNodeUsageMsg};
use crate::{Module, SimulationContext};

/// What a device adds to [`PowerNode`]: upstream `PowerNodeBase`'s virtual
/// methods.
pub trait PowerModel: Send {
    /// Upstream `customReset`: checks the device configuration, at
    /// initialization and at every reset. `node_name` names the module in
    /// messages.
    fn reset(&mut self, _node_name: &str) {}

    /// Upstream `customReadMessages`: reads the device's own inputs. Returns
    /// `false` when one the device needs cannot be read; the node then
    /// publishes zero power.
    fn read_messages(&mut self) -> bool {
        true
    }

    /// Upstream `evaluatePowerModel` \[W\]: the device's net power while it is
    /// on, supplied (+) or drawn (-).
    fn evaluate_power_model(&mut self) -> f64;
}

/// A device on the power network: upstream `PowerNodeBase`, generic over the
/// device model.
#[derive(Clone, Debug)]
pub struct PowerNode<M> {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// The device: its parameters and its own inputs.
    pub model: M,
    /// Upstream `nodeStatusInMsg`: optional on/off command.
    pub node_status_in_msg: Input<DeviceStatusMsg>,
    /// Upstream `nodePowerOutMsg`: the device's net power.
    pub node_power_out_msg: Output<PowerNodeUsageMsg>,
    /// Upstream `powerStatus`: whether the device is on. Overwritten by
    /// `node_status_in_msg` while that is connected.
    pub power_status: DeviceState,
}

impl<M: PowerModel> PowerNode<M> {
    /// A node named `name` running `model`, switched on.
    pub fn new(name: impl Into<String>, model: M) -> Self {
        Self {
            name: name.into(),
            model,
            node_status_in_msg: Input::default(),
            node_power_out_msg: Output::default(),
            power_status: DeviceState::On,
        }
    }

    /// Upstream `readMessages`: the status, then the device's own inputs.
    fn read_messages(&mut self) -> bool {
        if self.node_status_in_msg.is_connected() {
            self.power_status = self.node_status_in_msg.read().device_status;
        }
        self.model.read_messages()
    }

    /// Upstream `computePowerStatus`: the device's power while on, else zero.
    fn compute_power_status(&mut self) -> PowerNodeUsageMsg {
        if self.power_status.is_on() {
            PowerNodeUsageMsg {
                net_power_w: self.model.evaluate_power_model(),
            }
        } else {
            PowerNodeUsageMsg::default()
        }
    }
}

impl<M: PowerModel> Module for PowerNode<M> {
    fn init(&mut self) {
        self.model.reset(&self.name);
        self.node_power_out_msg.write(PowerNodeUsageMsg::default());
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.model.reset(&self.name);
    }

    fn update(&mut self, _context: &SimulationContext) {
        let usage = if self.read_messages() {
            self.compute_power_status()
        } else {
            PowerNodeUsageMsg::default()
        };
        self.node_power_out_msg.write(usage);
    }
}

#[cfg(test)]
mod tests {
    //! The switching and read-failure logic every node shares. The upstream
    //! unit tests of the nodes are ported in each node's own module.

    use hifitime::Epoch;

    use crate::messages::{DeviceState, DeviceStatusMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{PowerModel, PowerNode};

    /// A device drawing `power_w`, whose own read can be made to fail, and
    /// which counts its evaluations.
    #[derive(Clone, Debug)]
    struct TestDevice {
        power_w: f64,
        readable: bool,
        evaluations: usize,
        resets: usize,
    }

    impl PowerModel for TestDevice {
        fn reset(&mut self, _node_name: &str) {
            self.resets += 1;
        }

        fn read_messages(&mut self) -> bool {
            self.readable
        }

        fn evaluate_power_model(&mut self) -> f64 {
            self.evaluations += 1;
            self.power_w
        }
    }

    fn node(readable: bool) -> PowerNode<TestDevice> {
        PowerNode::new(
            "device",
            TestDevice {
                power_w: -4.5,
                readable,
                evaluations: 0,
                resets: 0,
            },
        )
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    #[test]
    fn an_unswitched_node_is_on_and_publishes_its_model() {
        let mut node = node(true);
        node.init();
        assert_eq!(node.model.resets, 1);
        assert_eq!(node.node_power_out_msg.read().net_power_w, 0.0);

        node.update(&context(0));
        assert_eq!(node.node_power_out_msg.read().net_power_w, -4.5);
        assert_eq!(node.model.evaluations, 1);

        Module::reset(&mut node, &context(0));
        assert_eq!(node.model.resets, 2);
    }

    #[test]
    fn the_status_message_switches_the_node_and_sticks() {
        let mut node = node(true);
        let status = Output::new(DeviceStatusMsg {
            device_status: DeviceState::Off,
        });
        status.connect_to(&mut node.node_status_in_msg);
        node.init();

        node.update(&context(0));
        assert_eq!(node.node_power_out_msg.read().net_power_w, 0.0);
        assert_eq!(node.power_status, DeviceState::Off);
        // Off, the model is not evaluated at all.
        assert_eq!(node.model.evaluations, 0);

        status.write(DeviceStatusMsg {
            device_status: DeviceState::On,
        });
        node.update(&context(1));
        assert_eq!(node.node_power_out_msg.read().net_power_w, -4.5);
        assert_eq!(node.power_status, DeviceState::On);
    }

    #[test]
    fn without_a_status_message_the_power_status_field_switches_the_node() {
        let mut node = node(true);
        node.power_status = DeviceState::Off;
        node.init();
        node.update(&context(0));
        assert_eq!(node.node_power_out_msg.read().net_power_w, 0.0);
        assert_eq!(node.model.evaluations, 0);
    }

    #[test]
    fn a_failed_device_read_publishes_zero_power() {
        let mut node = node(false);
        node.init();
        node.update(&context(0));
        assert_eq!(node.node_power_out_msg.read().net_power_w, 0.0);
        assert_eq!(node.model.evaluations, 0);
    }
}
