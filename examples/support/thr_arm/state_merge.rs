//! Attitude from one state, translation from another: port of upstream's
//! pure-Python `simulation/navigation/stateMerge/stateMerge.py`.
//!
//! Each update copies `σ_BN` and `ω_BN_B` of the attitude source (a hub's
//! origin site) and the centre-of-mass position and velocity of the
//! translation source into one spacecraft state, for navigation modules that
//! read both from one message. Upstream reads the translation source's
//! `r_CN_N` / `v_CN_N`; basilisk-rs's
//! [`MjSystemCom`](basilisk_rs::mujoco_dynamics::mj_system_com::MjSystemCom)
//! publishes the system centre of mass in `position_m` / `velocity_mps`,
//! which are read here. Upstream raises `BasiliskError` for an unconnected
//! input at `Reset`; here `init` panics.

#![allow(dead_code)]

use basilisk_rs::messages::{Input, Output, SpacecraftStateMsg};
use basilisk_rs::{Module, SimulationContext};

/// Upstream `StateMerge`.
#[derive(Debug, Default)]
pub struct StateMerge {
    /// Upstream `attStateInMsg`.
    pub att_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `transStateInMsg`.
    pub trans_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `stateOutMsg`.
    pub state_out_msg: Output<SpacecraftStateMsg>,
}

impl Module for StateMerge {
    fn init(&mut self) {
        for (connected, name) in [
            (self.att_state_in_msg.is_connected(), "attStateInMsg"),
            (self.trans_state_in_msg.is_connected(), "transStateInMsg"),
        ] {
            assert!(connected, "StateMerge.{name} was not linked.");
        }
        self.state_out_msg.write(SpacecraftStateMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let attitude = self.att_state_in_msg.read();
        let translation = self.trans_state_in_msg.read();
        self.state_out_msg.write(SpacecraftStateMsg {
            position_m: translation.position_m,
            velocity_mps: translation.velocity_mps,
            sigma_bn: attitude.sigma_bn,
            omega_radps: attitude.omega_radps,
            ..SpacecraftStateMsg::default()
        });
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `stateMerge/_UnitTest/test_stateMerge.py`.

    use super::*;
    use hifitime::Epoch;
    use nalgebra::Vector3;

    #[test]
    fn output_payload() {
        let attitude = Output::new(SpacecraftStateMsg {
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            omega_radps: Vector3::new(0.01, -0.02, 0.03),
            ..SpacecraftStateMsg::default()
        });
        let translation = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_mps: Vector3::new(-0.1, 0.2, -0.3),
            ..SpacecraftStateMsg::default()
        });
        let mut module = StateMerge::default();
        attitude.connect_to(&mut module.att_state_in_msg);
        translation.connect_to(&mut module.trans_state_in_msg);
        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 1,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        let merged = module.state_out_msg.read();
        assert_eq!(merged.sigma_bn, attitude.read().sigma_bn);
        assert_eq!(merged.omega_radps, attitude.read().omega_radps);
        assert_eq!(merged.position_m, translation.read().position_m);
        assert_eq!(merged.velocity_mps, translation.read().velocity_mps);
    }

    #[test]
    fn missing_inputs_are_rejected() {
        for missing in ["attStateInMsg", "transStateInMsg"] {
            let source = Output::new(SpacecraftStateMsg::default());
            let mut module = StateMerge::default();
            if missing != "attStateInMsg" {
                source.connect_to(&mut module.att_state_in_msg);
            }
            if missing != "transStateInMsg" {
                source.connect_to(&mut module.trans_state_in_msg);
            }
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| module.init()));
            let message = result.unwrap_err();
            let text = message
                .downcast_ref::<String>()
                .cloned()
                .unwrap_or_default();
            assert!(text.contains(&format!("StateMerge.{missing}")), "{text}");
        }
    }
}
