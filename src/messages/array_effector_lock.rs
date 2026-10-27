//! Effector lock flags (upstream `ArrayEffectorLockMsgPayload`).

use serde::{Deserialize, Serialize};

use super::MAX_EFF_COUNT;
use super::flatten::impl_telemetry;

/// Whether one effector degree of freedom is held fixed: upstream
/// `effectorLockFlag` entry.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum EffectorLock {
    /// Free to move; upstream `0`, the zeroed default.
    #[default]
    Unlocked = 0,
    /// Locked; upstream `1`.
    Locked = 1,
}

/// Lock flag of every effector: upstream `ArrayEffectorLockMsg`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArrayEffectorLockMsg {
    /// Upstream `effectorLockFlag` (`int` 0 / 1 per effector).
    #[serde(with = "super::big_array")]
    pub effector_lock: [EffectorLock; MAX_EFF_COUNT],
}

impl Default for ArrayEffectorLockMsg {
    fn default() -> Self {
        Self {
            effector_lock: [EffectorLock::Unlocked; MAX_EFF_COUNT],
        }
    }
}

impl_telemetry!(enum EffectorLock as u8);
impl_telemetry!(ArrayEffectorLockMsg { effector_lock });
