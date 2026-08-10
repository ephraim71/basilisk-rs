//! What a rule is: how it computes its value, and what identifies it.
//!
//! These three types are the vocabulary shared by both halves of the feature.
//! The port machinery in [`crate::messages`] applies rules; the registry in the
//! parent module addresses them by name. Neither is the natural owner, so they
//! sit in a file of their own that depends on nothing but serde — which is what
//! lets `messages` use them without depending on the registry.
//!
//! Not public as a path: every type here is re-exported from
//! [`crate::overrides`], and one way in is better than two.

use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// How a rule produces its value from the one beneath it.
///
/// [`Self::Patch`] and [`Self::PointerReplace`] are **relative**: they modify
/// what they are laid over. The rest are absolute — they define the whole
/// message and mask everything below them while installed. `OverrideCell::fold`
/// turns on that split, so a new mode has to declare which side it is on.
///
/// Renamed `camelCase` rather than `lowercase` so `PointerReplace` reaches the
/// wire as `pointerReplace`. Every other variant is a single word and
/// unaffected.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Mode {
    Replace,
    Patch,
    Freeze,
    Default,
    /// The `replace` operation of RFC 6902 — **that one and no other** — which
    /// unlike [`Self::Patch`] can address a single element of an array.
    ///
    /// A merge replaces an array wholesale, so `Patch` cannot change one
    /// component of a vector without also pinning its siblings to whatever the
    /// sender happened to write. On a live value — a body rate, a magnetic
    /// field, an ECEF position — that turns a single-axis fault into a
    /// whole-vector freeze.
    ///
    /// Named for what it accepts rather than for RFC 6902 as a whole: every
    /// other operation of that RFC is refused. See `apply_pointer_replace` for
    /// which, and why.
    PointerReplace,
}

impl Mode {
    /// Whether this rule modifies the value beneath it rather than replacing the
    /// whole message.
    ///
    /// Public because a client has to know it to describe the mode honestly, and
    /// deriving it a second time downstream is how the two answers drift.
    pub fn is_relative(self) -> bool {
        matches!(self, Self::Patch | Self::PointerReplace)
    }
}

/// Identifies one installed rule.
///
/// A timed fault must clear the rule *it* installed. Without an id, a timer that
/// fires after a second rule was applied to the same target clears the second
/// one.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct RuleId(u64);

impl RuleId {
    /// The identity of a rule that is installed nowhere.
    ///
    /// A previewed candidate is folded alongside the installed stack but never
    /// stored, so it needs an identity that cannot collide with one that is.
    /// [`next_rule_id`] counts up from 1 and never issues this.
    pub(crate) const UNINSTALLED: Self = Self(0);
}

pub(crate) fn next_rule_id() -> RuleId {
    static NEXT: AtomicU64 = AtomicU64::new(1);
    RuleId(NEXT.fetch_add(1, Ordering::Relaxed))
}

/// One rule in force on a port: what to do, with what, and which fault owns it.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Rule {
    pub mode: Mode,
    pub value: Value,
    pub id: RuleId,
}

impl Rule {
    /// Builds the rule `mode` and `value` ask for.
    ///
    /// A `freeze` stores the value it captured rather than the (empty) payload
    /// that requested it, so the rule stands on its own once installed.
    /// `freeze_source` carries that capture; every other mode ignores it.
    pub(crate) fn new(mode: Mode, value: Value, freeze_source: Option<Value>, id: RuleId) -> Self {
        Self {
            mode,
            value: match mode {
                Mode::Freeze => freeze_source.unwrap_or(value),
                _ => value,
            },
            id,
        }
    }
}
