//! Joint rates to a reaction-wheel speed message: port of upstream Basilisk
//! `simulation/deviceInterface/scalarJointStatesToRWSpeed`.
//!
//! A multibody simulation models each reaction wheel as a spinning joint and
//! publishes its rate as a [`ScalarJointStateMsg`]; flight software reads the
//! wheel speeds as one [`RwSpeedMsg`]. This module copies the rate on joint
//! input `i` into `wheel_speeds_radps[i]`, in order, every update; the wheel
//! angles and the unused entries stay zero. The inputs are made by
//! [`ScalarJointStatesToRwSpeed::new`] from the configured joint count.

use crate::messages::{Input, MAX_EFF_COUNT, Output, RwSpeedMsg, ScalarJointStateMsg};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ScalarJointStatesToRwSpeed`].
#[derive(Clone, Debug)]
pub struct ScalarJointStatesToRwSpeedConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `numJoints` (`setNumJoints`): the number of joint-rate
    /// inputs, at most [`MAX_EFF_COUNT`]. Fixed at construction.
    pub num_joints: usize,
}

/// Collects joint rates into a [`RwSpeedMsg`]: upstream
/// `ScalarJointStatesToRWSpeed`.
#[derive(Clone, Debug)]
pub struct ScalarJointStatesToRwSpeed {
    /// Module configuration.
    pub config: ScalarJointStatesToRwSpeedConfig,
    /// Upstream `jointStateInMsgs`: the rate \[rad/s\] of each wheel joint.
    pub joint_state_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `rwSpeedOutMsg`: the wheel speeds, in joint order.
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
}

impl ScalarJointStatesToRwSpeed {
    /// Builds the module with `config.num_joints` unconnected inputs.
    ///
    /// # Panics
    ///
    /// If `config.num_joints` exceeds [`MAX_EFF_COUNT`], the capacity of
    /// [`RwSpeedMsg`].
    pub fn new(config: ScalarJointStatesToRwSpeedConfig) -> Self {
        assert!(
            config.num_joints <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} joints are supported, got {}",
            config.num_joints
        );
        // Not `vec![_; n]`: cloning a port aliases it.
        let joint_state_in_msgs = (0..config.num_joints).map(|_| Input::default()).collect();
        Self {
            config,
            joint_state_in_msgs,
            rw_speed_out_msg: Output::default(),
        }
    }
}

impl Module for ScalarJointStatesToRwSpeed {
    fn init(&mut self) {
        for (index, input) in self.joint_state_in_msgs.iter().enumerate() {
            if !input.is_connected() {
                log::error!(
                    "scalarJointStatesToRWSpeed '{}' joint_state_in_msgs[{index}] is not connected",
                    self.config.name
                );
            }
        }
        self.rw_speed_out_msg.write(RwSpeedMsg::default());
    }

    /// Upstream `Reset`: zeroes the published speeds.
    fn reset(&mut self, _context: &SimulationContext) {
        self.rw_speed_out_msg.write(RwSpeedMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let mut speeds = RwSpeedMsg::default();
        for (speed, input) in speeds
            .wheel_speeds_radps
            .iter_mut()
            .zip(&self.joint_state_in_msgs)
        {
            *speed = input.read().state;
        }
        self.rw_speed_out_msg.write(speeds);
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `scalarJointStatesToRWSpeed/_UnitTest/test_scalarJointStatesToRWSpeed.py`
    //! over its parameters.

    use hifitime::Epoch;

    use super::{ScalarJointStatesToRwSpeed, ScalarJointStatesToRwSpeedConfig};
    use crate::messages::{MAX_EFF_COUNT, Output, ScalarJointStateMsg};
    use crate::{Module, SimulationContext};

    fn adapter(num_joints: usize) -> ScalarJointStatesToRwSpeed {
        ScalarJointStatesToRwSpeed::new(ScalarJointStatesToRwSpeedConfig {
            name: "scalarJointStatesToRWSpeed".to_string(),
            num_joints,
        })
    }

    /// Upstream `test_scalarJointStatesToRWSpeed` for 1, 3 and 8 joints: rate
    /// `0.2 (i + 1)` on joint `i`, two task ticks.
    #[test]
    fn each_wheel_speed_is_its_joint_rate() {
        for num_joints in [1, 3, 8] {
            let rates: Vec<f64> = (0..num_joints)
                .map(|index| (index + 1) as f64 * 0.2)
                .collect();
            let rate_outs: Vec<_> = rates
                .iter()
                .map(|&state| Output::new(ScalarJointStateMsg { state }))
                .collect();
            let mut module = adapter(num_joints);
            for (rate_out, input) in rate_outs.iter().zip(&mut module.joint_state_in_msgs) {
                rate_out.connect_to(input);
            }

            module.init();
            for tick in 0..2 {
                module.update(&SimulationContext {
                    current_sim_nanos: tick * 100_000_000,
                    current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
                });
            }

            let speeds = module.rw_speed_out_msg.read();
            for (speed, truth) in speeds.wheel_speeds_radps.iter().zip(&rates) {
                assert!((speed - truth).abs() <= 1e-14);
            }
            assert!(
                speeds.wheel_speeds_radps[num_joints..]
                    .iter()
                    .all(|v| *v == 0.0)
            );
            assert!(speeds.wheel_angles_rad.iter().all(|v| *v == 0.0));
        }
    }

    #[test]
    #[should_panic(expected = "at most 36 joints are supported")]
    fn rejects_more_joints_than_the_message_holds() {
        let _ = adapter(MAX_EFF_COUNT + 1);
    }
}
