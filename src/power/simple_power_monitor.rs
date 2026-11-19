//! Power bookkeeping without limits: port of upstream Basilisk
//! `simplePowerMonitor`.
//!
//! Sums the net power of the nodes wired to `power_node_in_msgs` (as
//! [`SimpleBattery`](super::simple_battery::SimpleBattery) does) and integrates
//! it with an Euler step into a stored energy that has no capacity and no
//! floor: it can grow without bound and go negative. The published
//! [`PowerStorageStatusMsg`] carries that energy, the net power, and a
//! capacity of -1 (upstream's marker for "no capacity").
//!
//! The storage plumbing is upstream `PowerStorageBase`'s, as in
//! [`SimpleBattery`](super::simple_battery::SimpleBattery): with no node added
//! the status is zeroed, an added but unconnected node reads as 0 W, the
//! first update integrates from time zero, and a reset restarts from
//! [`SimplePowerMonitor::stored_charge_init_j`] at time zero.
//!
//! # Panics
//!
//! At initialization or reset, if `stored_charge_init_j` is negative
//! (upstream `bskError`).

use crate::messages::{Input, Output, PowerNodeUsageMsg, PowerStorageStatusMsg};
use crate::{Module, SimulationContext};

/// Upstream's `storageCapacity` of a monitor: no capacity.
const NO_CAPACITY_J: f64 = -1.0;

/// Net-power integrator: upstream `SimplePowerMonitor`.
#[derive(Clone, Debug)]
pub struct SimplePowerMonitor {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `storedCharge_Init` \[J\]: energy at initialization and
    /// reset; non-negative, zero by default.
    pub stored_charge_init_j: f64,
    /// Upstream `nodePowerUseInMsgs`: the nodes supplying (+) or drawing (-)
    /// power.
    pub power_node_in_msgs: Vec<Input<PowerNodeUsageMsg>>,
    /// Upstream `batPowerOutMsg`: integrated energy and net power.
    pub bat_power_out_msg: Output<PowerStorageStatusMsg>,
    /// Upstream `storedCharge` \[J\].
    stored_charge_j: f64,
    /// Upstream `previousTime` \[s\].
    previous_time_s: f64,
}

impl SimplePowerMonitor {
    /// A monitor named `name` starting from zero energy, with no nodes.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            stored_charge_init_j: 0.0,
            power_node_in_msgs: Vec::new(),
            bat_power_out_msg: Output::default(),
            stored_charge_j: 0.0,
            previous_time_s: 0.0,
        }
    }

    /// Upstream `addPowerNodeToModel`: adds a node input and returns it for
    /// wiring.
    pub fn add_power_node_to_model(&mut self) -> &mut Input<PowerNodeUsageMsg> {
        self.power_node_in_msgs.push(Input::default());
        self.power_node_in_msgs
            .last_mut()
            .expect("just pushed a power node input")
    }

    /// Upstream `sumAllInputs` \[W\]: the net power of all nodes.
    pub fn current_net_power_w(&self) -> f64 {
        self.power_node_in_msgs
            .iter()
            .map(|input| input.read().net_power_w)
            .sum()
    }

    /// Upstream `storedCharge` \[J\]: the integrated energy.
    pub fn stored_charge_j(&self) -> f64 {
        self.stored_charge_j
    }

    /// Upstream `PowerStorageBase::Reset`.
    fn restart(&mut self) {
        assert!(
            self.stored_charge_init_j >= 0.0,
            "simple power monitor '{}': stored_charge_init_j must be non-negative, got {}",
            self.name,
            self.stored_charge_init_j
        );
        self.previous_time_s = 0.0;
        self.stored_charge_j = self.stored_charge_init_j;
    }
}

impl Module for SimplePowerMonitor {
    fn init(&mut self) {
        self.restart();
        self.bat_power_out_msg.write(PowerStorageStatusMsg {
            storage_level_j: self.stored_charge_j,
            storage_capacity_j: NO_CAPACITY_J,
            current_net_power_w: 0.0,
        });
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.restart();
    }

    /// Upstream `PowerStorageBase::UpdateState` with `evaluateBatteryModel`.
    fn update(&mut self, context: &SimulationContext) {
        if self.power_node_in_msgs.is_empty() {
            self.bat_power_out_msg
                .write(PowerStorageStatusMsg::default());
            return;
        }

        let current_time_s = context.current_sim_nanos as f64 * 1.0e-9;
        let current_timestep_s = current_time_s - self.previous_time_s;
        let current_power_sum_w = self.current_net_power_w();
        self.stored_charge_j += current_power_sum_w * current_timestep_s;
        self.previous_time_s = current_time_s;
        self.bat_power_out_msg.write(PowerStorageStatusMsg {
            storage_level_j: self.stored_charge_j,
            storage_capacity_j: NO_CAPACITY_J,
            current_net_power_w: current_power_sum_w,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `simplePowerMonitor/_UnitTest/test_unitSimplePowerMonitor.py` with its
    //! 1e-8 relative accuracy.

    use hifitime::Epoch;

    use crate::messages::{Output, PowerNodeUsageMsg, PowerStorageStatusMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::SimplePowerMonitor;

    fn epoch() -> Epoch {
        Epoch::from_gregorian_utc_at_midnight(2025, 1, 1)
    }

    /// Upstream `storage_limits`: two 5 W nodes over one second at 0.1 s give
    /// a 10 W net power and 10 J.
    #[test]
    fn integrates_the_net_power_of_its_nodes() {
        let power_1 = Output::new(PowerNodeUsageMsg { net_power_w: 5.0 });
        let power_2 = Output::new(PowerNodeUsageMsg { net_power_w: 5.0 });
        let mut monitor = SimplePowerMonitor::new("monitor");
        monitor.stored_charge_init_j = 0.0;
        let mut recorder = MessageRecorder::<PowerStorageStatusMsg>::default();
        {
            let mut sim = Simulation::new(epoch(), false);
            sim.connect(&power_1, monitor.add_power_node_to_model());
            sim.connect(&power_2, monitor.add_power_node_to_model());
            sim.connect(&monitor.bat_power_out_msg, &mut recorder.input_msg);
            sim.add_module("monitor", &mut monitor, 100_000_000, 1);
            sim.add_module("recorder", &mut recorder, 100_000_000, 0);
            sim.run_for(1_000_000_000);
        }

        assert_eq!(recorder.samples.len(), 11);
        for sample in &recorder.samples {
            assert!(
                sample.current_net_power_w >= 10.0,
                "net power {}",
                sample.current_net_power_w
            );
        }
        let stored = recorder.samples.last().expect("logged").storage_level_j;
        assert!(
            ((stored - 10.0) / 10.0).abs() < 1.0e-8,
            "stored {stored} J, expected 10 J"
        );
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: epoch(),
        }
    }

    /// Unlike a battery, the monitor has no capacity and no floor.
    #[test]
    fn energy_is_unbounded_and_capacity_is_minus_one() {
        let drain = Output::new(PowerNodeUsageMsg { net_power_w: -3.0 });
        let mut monitor = SimplePowerMonitor::new("monitor");
        monitor.stored_charge_init_j = 1.0;
        drain.connect_to(monitor.add_power_node_to_model());

        monitor.init();
        for step in 0..=10 {
            monitor.update(&context(step * 1_000_000_000));
        }
        let status = monitor.bat_power_out_msg.read();
        assert_eq!(status.storage_capacity_j, -1.0);
        assert_eq!(status.current_net_power_w, -3.0);
        assert!((status.storage_level_j - (1.0 - 30.0)).abs() < 1.0e-12);
        assert_eq!(monitor.stored_charge_j(), status.storage_level_j);

        // A reset restarts from the initial energy at time zero.
        Module::reset(&mut monitor, &context(0));
        monitor.update(&context(1_000_000_000));
        assert!((monitor.stored_charge_j() - (1.0 - 3.0)).abs() < 1.0e-12);
    }

    #[test]
    fn no_power_nodes_writes_a_zero_status() {
        let mut monitor = SimplePowerMonitor::new("monitor");
        monitor.stored_charge_init_j = 4.0;
        monitor.init();
        assert_eq!(monitor.bat_power_out_msg.read().storage_level_j, 4.0);
        monitor.update(&context(1_000_000_000));
        let status = monitor.bat_power_out_msg.read();
        assert_eq!(status.storage_level_j, 0.0);
        assert_eq!(status.storage_capacity_j, 0.0);
        assert_eq!(status.current_net_power_w, 0.0);
    }

    #[test]
    #[should_panic(expected = "stored_charge_init_j")]
    fn a_negative_initial_charge_is_rejected() {
        let mut monitor = SimplePowerMonitor::new("monitor");
        monitor.stored_charge_init_j = -1.0;
        monitor.init();
    }
}
