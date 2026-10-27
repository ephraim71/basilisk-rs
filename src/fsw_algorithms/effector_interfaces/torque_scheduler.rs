//! Schedules two motor torques onto one array message and decides, per motor,
//! whether it is actuated or locked.
//!
//! Port of upstream Basilisk `torqueScheduler`. Entry 0 of each input torque
//! message is always forwarded (input 1 to output entry 0, input 2 to output
//! entry 1); the [`ArrayEffectorLockMsg`] tells the effectors which of the two
//! torques to apply, according to the configured [`LockFlag`] and the time
//! elapsed since the last reset.

use crate::messages::{ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, Input, Output};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// Scheduling logic (upstream `lockFlag`); discriminants are the upstream codes.
///
/// "Before" and "after" refer to the time since reset compared with
/// [`TorqueSchedulerConfig::t_switch_s`]: `t <= t_switch` is before.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum LockFlag {
    /// Both torques are applied at all times (upstream `0`, the default).
    #[default]
    ApplyBoth = 0,
    /// Torque 1 is applied before the switch time, torque 2 after (upstream `1`).
    FirstThenSecond = 1,
    /// Torque 2 is applied before the switch time, torque 1 after (upstream `2`).
    SecondThenFirst = 2,
    /// Neither torque is applied (upstream `3`).
    LockBoth = 3,
}

/// User-settable parameters of [`TorqueScheduler`]; upstream defaults both to zero.
#[derive(Clone, Debug)]
pub struct TorqueSchedulerConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Scheduling logic (upstream `lockFlag`).
    pub lock_flag: LockFlag,
    /// Time after reset at which the schedule switches (upstream `tSwitch`) \[s\].
    pub t_switch_s: f64,
}

impl Default for TorqueSchedulerConfig {
    fn default() -> Self {
        Self {
            name: "torqueScheduler".to_string(),
            lock_flag: LockFlag::ApplyBoth,
            t_switch_s: 0.0,
        }
    }
}

/// Combines two motor torques and schedules which of them is applied.
#[derive(Clone, Debug)]
pub struct TorqueScheduler {
    /// Module configuration.
    pub config: TorqueSchedulerConfig,
    /// First input torque; entry 0 is used (upstream `motorTorque1InMsg`).
    pub motor_torque1_in_msg: Input<ArrayMotorTorqueMsg>,
    /// Second input torque; entry 0 is used (upstream `motorTorque2InMsg`).
    pub motor_torque2_in_msg: Input<ArrayMotorTorqueMsg>,
    /// Both torques, in entries 0 and 1 (upstream `motorTorqueOutMsg`).
    pub motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
    /// Lock flags of the two motors, in entries 0 and 1 (upstream `effectorLockOutMsg`).
    pub effector_lock_out_msg: Output<ArrayEffectorLockMsg>,
    reset_time_nanos: u64,
}

impl TorqueScheduler {
    /// Creates the module with unconnected ports.
    pub fn new(config: TorqueSchedulerConfig) -> Self {
        Self {
            config,
            motor_torque1_in_msg: Input::default(),
            motor_torque2_in_msg: Input::default(),
            motor_torque_out_msg: Output::default(),
            effector_lock_out_msg: Output::default(),
            reset_time_nanos: 0,
        }
    }

    fn reset_at(&mut self, call_time_nanos: u64) {
        if !self.motor_torque1_in_msg.is_connected() {
            log::error!("torqueScheduler.motor_torque1_in_msg is not connected");
        }
        if !self.motor_torque2_in_msg.is_connected() {
            log::error!("torqueScheduler.motor_torque2_in_msg is not connected");
        }
        self.reset_time_nanos = call_time_nanos;

        // Upstream zeros both outputs so a disabled scheduler leaves the
        // effectors unlocked with no torque.
        self.motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
        self.effector_lock_out_msg
            .write(ArrayEffectorLockMsg::default());
    }

    /// Lock flags of motors 1 and 2 at `t_s` seconds after reset.
    fn lock_flags(&self, t_s: f64) -> [EffectorLock; 2] {
        use EffectorLock::{Locked, Unlocked};
        let after_switch = t_s > self.config.t_switch_s;
        match self.config.lock_flag {
            LockFlag::ApplyBoth => [Unlocked, Unlocked],
            LockFlag::FirstThenSecond if after_switch => [Locked, Unlocked],
            LockFlag::FirstThenSecond => [Unlocked, Locked],
            LockFlag::SecondThenFirst if after_switch => [Unlocked, Locked],
            LockFlag::SecondThenFirst => [Locked, Unlocked],
            LockFlag::LockBoth => [Locked, Locked],
        }
    }
}

impl Module for TorqueScheduler {
    fn init(&mut self) {
        self.reset_at(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_at(context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let torque1 = self.motor_torque1_in_msg.read();
        let torque2 = self.motor_torque2_in_msg.read();
        let t_s = diff_nanos_to_seconds(context.current_sim_nanos, self.reset_time_nanos);

        let torques = ArrayMotorTorqueMsg::from_active(&[
            torque1.motor_torque_nm[0],
            torque2.motor_torque_nm[0],
        ]);
        let mut locks = ArrayEffectorLockMsg::default();
        locks.effector_lock[..2].copy_from_slice(&self.lock_flags(t_s));

        self.motor_torque_out_msg.write(torques);
        self.effector_lock_out_msg.write(locks);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;

    use crate::messages::{
        ArrayEffectorLockMsg, ArrayMotorTorqueMsg, EffectorLock, MAX_EFF_COUNT, Output,
    };
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{LockFlag, TorqueScheduler, TorqueSchedulerConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn flag_value(lock: EffectorLock) -> f64 {
        f64::from(lock as u8)
    }

    /// Upstream truth of `torqueSchedulerTestFunction`: expected lock flags of
    /// motors 1 and 2 at time `t_s`.
    fn upstream_truth(lock_flag: u8, t_switch_s: f64, t_s: f64) -> [f64; 2] {
        match lock_flag {
            0 => [0.0, 0.0],
            1 if t_s > t_switch_s => [1.0, 0.0],
            1 => [0.0, 1.0],
            2 if t_s > t_switch_s => [0.0, 1.0],
            2 => [1.0, 0.0],
            _ => [1.0, 1.0],
        }
    }

    /// Port of `test_torqueScheduler`: 1 s task rate, 10 s run, every sample checked.
    #[test]
    fn matches_upstream_torque_scheduler_unit_test() {
        let accuracy = 1e-12;
        for (lock_flag, code) in [
            (LockFlag::ApplyBoth, 0),
            (LockFlag::FirstThenSecond, 1),
            (LockFlag::SecondThenFirst, 2),
            (LockFlag::LockBoth, 3),
        ] {
            for t_switch_s in [3.0, 6.0] {
                let torque1 = Output::new(ArrayMotorTorqueMsg::from_active(&[1.0]));
                let torque2 = Output::new(ArrayMotorTorqueMsg::from_active(&[3.0]));
                let mut scheduler = TorqueScheduler::new(TorqueSchedulerConfig {
                    lock_flag,
                    t_switch_s,
                    ..Default::default()
                });
                let mut torque_log = MessageRecorder::default();
                let mut lock_log = MessageRecorder::default();
                let mut simulation =
                    Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
                simulation.connect(&torque1, &mut scheduler.motor_torque1_in_msg);
                simulation.connect(&torque2, &mut scheduler.motor_torque2_in_msg);
                simulation.connect(&scheduler.motor_torque_out_msg, &mut torque_log.input_msg);
                simulation.connect(&scheduler.effector_lock_out_msg, &mut lock_log.input_msg);
                simulation.add_module("torqueScheduler", &mut scheduler, 1_000_000_000, 0);
                simulation.add_module("torqueLog", &mut torque_log, 1_000_000_000, 0);
                simulation.add_module("lockLog", &mut lock_log, 1_000_000_000, 0);
                simulation.run_for(10_000_000_000);
                drop(simulation);

                assert_eq!(torque_log.samples.len(), 11);
                for (index, (torques, locks)) in
                    torque_log.samples.iter().zip(&lock_log.samples).enumerate()
                {
                    let t_s = index as f64;
                    assert!((torques.motor_torque_nm[0] - 1.0).abs() <= accuracy);
                    assert!((torques.motor_torque_nm[1] - 3.0).abs() <= accuracy);
                    let expected = upstream_truth(code, t_switch_s, t_s);
                    for (motor, expected_flag) in expected.iter().enumerate() {
                        assert!(
                            (flag_value(locks.effector_lock[motor]) - expected_flag).abs()
                                <= accuracy,
                            "lockFlag={code}, tSwitch={t_switch_s}, t={t_s}, motor={motor}"
                        );
                    }
                }
            }
        }
    }

    /// Port of `test_reset`: Reset() zeros both outputs after a run produced
    /// non-zero torques.
    #[test]
    fn matches_upstream_reset_test() {
        let torque1 = Output::new(ArrayMotorTorqueMsg {
            motor_torque_nm: [1.0; MAX_EFF_COUNT],
        });
        let torque2 = Output::new(ArrayMotorTorqueMsg {
            motor_torque_nm: [3.0; MAX_EFF_COUNT],
        });
        let mut scheduler = TorqueScheduler::new(TorqueSchedulerConfig {
            lock_flag: LockFlag::ApplyBoth,
            t_switch_s: 3.0,
            ..Default::default()
        });
        torque1.connect_to(&mut scheduler.motor_torque1_in_msg);
        torque2.connect_to(&mut scheduler.motor_torque2_in_msg);
        scheduler.init();
        scheduler.update(&context(0));
        assert_eq!(
            scheduler.motor_torque_out_msg.read().motor_torque_nm[1],
            3.0
        );

        Module::reset(&mut scheduler, &context(0));
        assert!(
            scheduler
                .motor_torque_out_msg
                .read()
                .motor_torque_nm
                .iter()
                .all(|torque| torque.abs() <= 1e-12)
        );
        assert_eq!(
            scheduler.effector_lock_out_msg.read(),
            ArrayEffectorLockMsg::default()
        );
    }

    #[test]
    fn measures_the_switch_time_from_the_last_reset() {
        let torque1 = Output::new(ArrayMotorTorqueMsg::from_active(&[1.0]));
        let torque2 = Output::new(ArrayMotorTorqueMsg::from_active(&[3.0]));
        let mut scheduler = TorqueScheduler::new(TorqueSchedulerConfig {
            lock_flag: LockFlag::FirstThenSecond,
            t_switch_s: 2.0,
            ..Default::default()
        });
        torque1.connect_to(&mut scheduler.motor_torque1_in_msg);
        torque2.connect_to(&mut scheduler.motor_torque2_in_msg);
        scheduler.init();

        Module::reset(&mut scheduler, &context(10_000_000_000));
        scheduler.update(&context(12_000_000_000));
        let locks = scheduler.effector_lock_out_msg.read().effector_lock;
        assert_eq!(locks[..2], [EffectorLock::Unlocked, EffectorLock::Locked]);

        scheduler.update(&context(12_500_000_000));
        let locks = scheduler.effector_lock_out_msg.read().effector_lock;
        assert_eq!(locks[..2], [EffectorLock::Locked, EffectorLock::Unlocked]);
        assert!(
            locks[2..]
                .iter()
                .all(|lock| *lock == EffectorLock::Unlocked)
        );
    }
}
