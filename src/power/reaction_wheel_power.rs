//! Reaction wheel power draw: port of upstream Basilisk `ReactionWheelPower`.
//!
//! A [`PowerNode`] for one reaction wheel. It reads that wheel's
//! [`ReactionWheelStateMsg`] (one of
//! [`ReactionWheelStateEffector`](crate::dynamics::reaction_wheel_state_effector::ReactionWheelStateEffector)'s
//! per-wheel `state_out` ports) and draws a base power plus the electrical
//! power that the motor torque `u` needs to change the wheel speed `Ω`:
//!
//! ```text
//! p_mech = Ω · u
//! p_RW   = p_base + |p_mech| / η_e2m     when p_mech > 0 or η_m2e < 0
//! p_RW   = p_base + η_m2e · p_mech       otherwise (braking recovers power)
//! P      = -p_RW
//! ```
//!
//! Spinning the wheel up always takes power. Braking it takes power too while
//! the mechanical-to-electrical efficiency `η_m2e` is negative (the default),
//! takes none when it is zero, and returns power to the bus when it is
//! positive, so `p_RW` can go negative.
//!
//! # Differences from upstream
//!
//! * One wheel's [`ReactionWheelStateMsg`] stands in for upstream's
//!   `RWConfigLogMsgPayload`; `Omega` and `u_current` are its `omega_radps`
//!   and `applied_motor_torque_nm`.
//! * No message headers; the stand-in is "connected ⇒ written at the current
//!   update". Upstream publishes zero power, with a warning, while
//!   `rwStateInMsg` is linked but unwritten; here that is the unconnected
//!   case (upstream's unlinked path reads an uninitialized wheel state, and
//!   its documentation requires the subscription). The warning is logged at
//!   initialization and reset rather than at every update.
//!
//! # Panics
//!
//! At initialization or reset, if `elec_to_mech_efficiency` is not strictly
//! positive (upstream `bskError`).

use super::power_node::{PowerModel, PowerNode};
use crate::messages::{Input, ReactionWheelStateMsg};

/// Efficiencies, base load and wheel input of a [`ReactionWheelPower`]:
/// upstream `ReactionWheelPower`'s own members.
#[derive(Clone, Debug)]
pub struct ReactionWheelPowerModel {
    /// Upstream `rwStateInMsg`: the wheel's speed and motor torque
    /// (required; see the module docs).
    pub rw_state_in_msg: Input<ReactionWheelStateMsg>,
    /// Upstream `elecToMechEfficiency` [-]: fraction of the electrical power
    /// that becomes mechanical power; strictly positive, one by default.
    pub elec_to_mech_efficiency: f64,
    /// Upstream `mechToElecEfficiency` [-]: fraction of the braking power
    /// recovered; negative (the default, -1) models braking as drawing power
    /// like spinning up.
    pub mech_to_elec_efficiency: f64,
    /// Upstream `basePowerNeed` \[W\]: power the device draws just to be on,
    /// typically positive; zero by default.
    pub base_power_need_w: f64,
    /// Upstream `rwStatus`: the wheel state read this update.
    rw_status: ReactionWheelStateMsg,
}

impl Default for ReactionWheelPowerModel {
    /// Upstream's constructor: no base load, an ideal motor, no recovery.
    fn default() -> Self {
        Self {
            rw_state_in_msg: Input::default(),
            elec_to_mech_efficiency: 1.0,
            mech_to_elec_efficiency: -1.0,
            base_power_need_w: 0.0,
            rw_status: ReactionWheelStateMsg::default(),
        }
    }
}

impl PowerModel for ReactionWheelPowerModel {
    /// Upstream `customReset`.
    fn reset(&mut self, node_name: &str) {
        assert!(
            self.elec_to_mech_efficiency > 0.0,
            "reaction wheel power '{node_name}': elec_to_mech_efficiency is {}, \
             must be strictly positive",
            self.elec_to_mech_efficiency
        );
        if !self.rw_state_in_msg.is_connected() {
            log::warn!(
                "reaction wheel power '{node_name}': rw_state_in_msg is not connected, \
                 so the node publishes zero power"
            );
        }
    }

    /// Upstream `customReadMessages`: fails while the wheel is unconnected.
    fn read_messages(&mut self) -> bool {
        if !self.rw_state_in_msg.is_connected() {
            return false;
        }
        self.rw_status = self.rw_state_in_msg.read();
        true
    }

    /// Upstream `evaluatePowerModel`.
    fn evaluate_power_model(&mut self) -> f64 {
        let mut rw_power_need_w = self.base_power_need_w;
        let wheel_power_w = self.rw_status.omega_radps * self.rw_status.applied_motor_torque_nm;
        if wheel_power_w > 0.0 || self.mech_to_elec_efficiency < 0.0 {
            // Spinning up always takes power; so does braking without
            // recovery.
            rw_power_need_w += wheel_power_w.abs() / self.elec_to_mech_efficiency;
        } else {
            // Braking, recovering part of the mechanical energy.
            rw_power_need_w += self.mech_to_elec_efficiency * wheel_power_w;
        }
        // A positive need is a draw on the bus.
        -rw_power_need_w
    }
}

/// Upstream `ReactionWheelPower`: a [`PowerNode`] running
/// [`ReactionWheelPowerModel`].
pub type ReactionWheelPower = PowerNode<ReactionWheelPowerModel>;

#[cfg(test)]
mod tests {
    //! Ports upstream `ReactionWheelPower/_UnitTest/test_unitReactionWheelPower.py`
    //! over all 72 parameter combinations with its 1e-12 accuracy, its truth
    //! computed as the pytest does. Upstream's "RW message linked but never
    //! written" case (`setRwMsg = False`) is the unconnected wheel here; see
    //! the module docs.

    use std::f64::consts::PI;

    use hifitime::Epoch;

    use crate::messages::{
        DeviceState, DeviceStatusMsg, Output, PowerNodeUsageMsg, ReactionWheelStateMsg,
    };
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{ReactionWheelPower, ReactionWheelPowerModel};

    const STEP_NANOS: u64 = 500_000_000;
    /// Upstream `macros.RPM` \[rad/s per RPM\].
    const RPM: f64 = 2.0 * PI / 60.0;

    /// Upstream `powerRW`: builds the node, runs it for one second at 0.5 s,
    /// and checks the three logged powers against the pytest's truth.
    fn power_rw(
        set_rw_msg: bool,
        set_device_status_msg: u8,
        set_eta_e2m: bool,
        omega_value_rpm: f64,
        set_eta_m2c: u8,
        accuracy: f64,
    ) {
        let mut model = ReactionWheelPowerModel {
            base_power_need_w: 10.0,
            ..Default::default()
        };
        let eta_e2m = if set_eta_e2m {
            model.elec_to_mech_efficiency = 0.9;
            model.elec_to_mech_efficiency
        } else {
            1.0
        };
        let eta_m2e = if set_eta_m2c > 0 {
            model.mech_to_elec_efficiency = (f64::from(set_eta_m2c) - 1.0) / 2.0;
            model.mech_to_elec_efficiency
        } else {
            -1.0
        };

        let omega_radps = omega_value_rpm * RPM;
        let rw_status = ReactionWheelStateMsg {
            omega_radps,
            applied_motor_torque_nm: 0.010,
            ..Default::default()
        };
        let rw_msg = Output::new(rw_status.clone());
        if set_rw_msg {
            rw_msg.connect_to(&mut model.rw_state_in_msg);
        }
        let mut node = ReactionWheelPower::new("bskSat", model);
        let status_msg = Output::new(DeviceStatusMsg {
            device_status: if set_device_status_msg == 2 {
                DeviceState::On
            } else {
                DeviceState::Off
            },
        });
        if set_device_status_msg > 0 {
            status_msg.connect_to(&mut node.node_status_in_msg);
        }

        let mut recorder = MessageRecorder::<PowerNodeUsageMsg>::default();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&node.node_power_out_msg, &mut recorder.input_msg);
            sim.add_module("bskSat", &mut node, STEP_NANOS, 1);
            sim.add_module("recorder", &mut recorder, STEP_NANOS, 0);
            sim.run_for(1_000_000_000);
        }

        let true_power = if set_rw_msg && set_device_status_msg != 1 {
            let wheel_power = omega_radps * rw_status.applied_motor_torque_nm;
            let mut true_power = 10.0;
            if wheel_power > 0.0 || eta_m2e < 0.0 {
                true_power += wheel_power.abs() / eta_e2m;
            } else {
                true_power += eta_m2e * wheel_power;
            }
            -true_power
        } else {
            0.0
        };

        assert_eq!(recorder.samples.len(), 3);
        for sample in &recorder.samples {
            assert!(
                (sample.net_power_w - true_power).abs() <= accuracy,
                "setRwMsg {set_rw_msg}, setDeviceStatusMsg {set_device_status_msg}, \
                 setEta_e2m {set_eta_e2m}, Omega {omega_value_rpm} RPM, \
                 setEta_m2c {set_eta_m2c}: {} W != {true_power} W",
                sample.net_power_w
            );
        }
    }

    /// Upstream `test_module`: every combination of wheel message, status
    /// message, efficiencies and spin direction.
    #[test]
    fn power_follows_wheel_state_status_and_efficiencies() {
        for set_rw_msg in [true, false] {
            for set_eta_e2m in [true, false] {
                for set_eta_m2c in [0, 1, 2] {
                    for omega_value_rpm in [100.0, -100.0] {
                        for set_device_status_msg in [0, 1, 2] {
                            power_rw(
                                set_rw_msg,
                                set_device_status_msg,
                                set_eta_e2m,
                                omega_value_rpm,
                                set_eta_m2c,
                                1.0e-12,
                            );
                        }
                    }
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

    /// With full recovery, braking returns more power than the base load
    /// draws: the node becomes a source.
    #[test]
    fn braking_with_recovery_can_supply_power() {
        let wheel = Output::new(ReactionWheelStateMsg {
            omega_radps: 200.0,
            applied_motor_torque_nm: -0.1,
            ..Default::default()
        });
        let mut model = ReactionWheelPowerModel {
            base_power_need_w: 5.0,
            mech_to_elec_efficiency: 1.0,
            ..Default::default()
        };
        wheel.connect_to(&mut model.rw_state_in_msg);
        let mut node = ReactionWheelPower::new("rw", model);
        node.init();
        node.update(&context());
        assert_eq!(node.node_power_out_msg.read().net_power_w, 15.0);
    }

    #[test]
    #[should_panic(expected = "elec_to_mech_efficiency")]
    fn a_non_positive_motor_efficiency_is_rejected() {
        let mut node = ReactionWheelPower::new(
            "rw",
            ReactionWheelPowerModel {
                elec_to_mech_efficiency: 0.0,
                ..Default::default()
            },
        );
        node.init();
    }
}
