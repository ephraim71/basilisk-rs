//! What a rule is, and what it computes.
//!
//! [`Mode`], [`Rule`] and [`RuleId`] are the vocabulary both halves of the
//! feature speak: the port machinery in [`crate::messages`] applies rules, and
//! the registry in the parent module addresses them by name. Neither is the
//! natural owner, so they sit here.
//!
//! What a rule *does* to a value lives here too, in both its numbers:
//! [`apply_override`] for one rule, [`fold_rules`] for a whole stack. They sit
//! beside the modes they dispatch on, so adding a mode is one file — a variant,
//! a line in [`Mode::is_relative`], and an arm in each. Split across modules, as
//! they were, the halves could disagree about which modes exist and only a test
//! would say so.
//!
//! What is *not* here is custody. [`crate::messages`] owns each port's stack and
//! decides when a fold is allowed to become the value readers see; this file only
//! answers what a given stack computes, and answers it the same way for a
//! candidate stack as for an installed one.
//!
//! Not public as a path: every type is re-exported from [`crate::overrides`],
//! and one way in is better than two.

use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Serialize};
use serde_json::Value;

use crate::messages::SimulationMessage;

/// How a rule produces its value from the one beneath it.
///
/// [`Self::Patch`] and [`Self::ReplaceAt`] are **relative**: they modify
/// what they are laid over. The rest are absolute — they define the whole
/// message and mask everything below them while installed. `fold_rules` turns
/// on that split, so a new mode has to declare which side it is on.
///
/// Renamed `camelCase` rather than `lowercase` so `ReplaceAt` reaches the
/// wire as `replaceAt`. Every other variant is a single word and
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
    /// Named for the one thing it does — replace the value *at* a path — rather
    /// than for RFC 6902 as a whole: every other operation of that RFC is
    /// refused. See `apply_replace_at` below for which, and why.
    ///
    /// Do not read the shared prefix with [`Self::Replace`] as kinship. That
    /// one is absolute and defines the whole message; this one is relative and
    /// touches only the paths it names. They sit on opposite sides of the split
    /// `fold` cares about.
    ReplaceAt,
}

impl Mode {
    /// Whether this rule modifies the value beneath it rather than replacing the
    /// whole message.
    ///
    /// Public because a client has to know it to describe the mode honestly, and
    /// deriving it a second time downstream is how the two answers drift.
    pub fn is_relative(self) -> bool {
        matches!(self, Self::Patch | Self::ReplaceAt)
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

/// Overlays `patch` onto `base`, descending only where both sides are objects.
///
/// This is RFC 7396's merge **except for `null`**, and the exception is
/// deliberate. RFC 7396 defines `null` as a request to remove a member; here it
/// is assigned like any other value. Removal is not something this can offer —
/// a message has a fixed schema, so dropping a member yields a value that will
/// not deserialise — while assignment is exactly what an operator setting an
/// `Option` field to `None` is asking for.
///
/// Arrays follow RFC 7396: they are replaced whole rather than merged
/// element-wise, since a merge document has no way to name one element. That is
/// what [`Mode::ReplaceAt`] exists for.
fn merge_json(base: &mut Value, patch: Value) {
    match (base, patch) {
        (Value::Object(base_map), Value::Object(patch_map)) => {
            for (key, patch_value) in patch_map {
                if let Some(base_value) = base_map.get_mut(&key) {
                    merge_json(base_value, patch_value);
                } else {
                    base_map.insert(key, patch_value);
                }
            }
        }
        (base_value, patch_value) => *base_value = patch_value,
    }
}

/// Applies the `replace` operation of an RFC 6902 document, and no other.
///
/// **This is a subset, not an implementation of RFC 6902.** A conforming
/// processor accepts six operations; a document using any of the other five is
/// refused here rather than approximated, and a caller must not assume a
/// general patch document will apply.
///
/// The subset is what the target model can express, not what is convenient:
///
/// - **`add`, `remove`** — on a *member*, a message has a fixed schema, so
///   adding one produces a field no type declares and removing one produces a
///   value that will not deserialise. On an *array element*, both change the
///   length, and every array reachable here is a fixed-size vector. In the one
///   case where `add` is well defined — an existing member — it is `replace`
///   under another name.
/// - **`move`, `copy`** — a `remove` and an `add` composed, so they inherit
///   the above.
/// - **`test`** — well defined, and pointless here: it guards a document
///   against a concurrent writer, and no such document exists. Each rule is a
///   layer folded fresh over the upstream value on every write, so there is
///   nothing shared to lose a race with.
///
/// Paths are RFC 6901 pointers, which [`Value::pointer_mut`] resolves. A
/// pointer that does not resolve is an error, so an index past the end of a
/// fixed-size vector is reported as such rather than silently doing nothing.
fn apply_replace_at(base: &mut Value, document: &Value) -> Result<(), serde_json::Error> {
    let operations = document
        .as_array()
        .ok_or_else(|| json_error(format!("a replaceAt value must be an array: {document}")))?;

    for operation in operations {
        let name = operation
            .get("op")
            .and_then(Value::as_str)
            .ok_or_else(|| json_error(format!("a replaceAt operation needs 'op': {operation}")))?;
        if name != "replace" {
            return Err(json_error(format!(
                "this mode implements the 'replace' operation of RFC 6902 and no other, \
                 so '{name}' is refused rather than approximated: a message has a fixed \
                 schema and fixed-size arrays, which leaves the other five either \
                 unrepresentable or indistinguishable from 'replace'"
            )));
        }

        let path = operation
            .get("path")
            .and_then(Value::as_str)
            .ok_or_else(|| {
                json_error(format!("a replaceAt operation needs 'path': {operation}"))
            })?;
        let value = operation
            .get("value")
            .ok_or_else(|| json_error(format!("a replaceAt replace needs 'value': {operation}")))?;

        match base.pointer_mut(path) {
            Some(slot) => *slot = value.clone(),
            None => {
                return Err(json_error(format!(
                    "replaceAt path '{path}' does not resolve on this value"
                )));
            }
        }
    }
    Ok(())
}

/// A `serde_json::Error` carrying a message of our own.
///
/// The override path returns `serde_json::Error` throughout, and the failures
/// above are not deserialisation failures. Widening the error type would ripple
/// through `fold`, `install` and `preview` on both ports; this borrows the one
/// constructor serde offers for the purpose.
fn json_error(message: impl std::fmt::Display) -> serde_json::Error {
    <serde_json::Error as serde::de::Error>::custom(message)
}

/// Produces the value `rule` yields when laid over `upstream`.
pub(crate) fn apply_override<T: SimulationMessage>(
    rule: &Rule,
    upstream: T,
) -> Result<T, serde_json::Error> {
    match rule.mode {
        Mode::Replace | Mode::Freeze => serde_json::from_value(rule.value.clone()),
        Mode::Patch => {
            let mut base = serde_json::to_value(upstream)?;
            merge_json(&mut base, rule.value.clone());
            serde_json::from_value(base)
        }
        Mode::ReplaceAt => {
            let mut base = serde_json::to_value(upstream)?;
            apply_replace_at(&mut base, &rule.value)?;
            serde_json::from_value(base)
        }
        Mode::Default => Ok(T::default()),
    }
}

/// Produces the value a whole stack yields when laid over `upstream`, innermost
/// first.
///
/// The plural of [`apply_override`], and not merely a loop over it: it starts at
/// the outermost absolute rule. `Replace`, `Freeze` and `Default` define the
/// whole message, so nothing below one can be observed while it is installed —
/// but the layers stay in the stack, and reappear when it is removed. Discarding
/// them instead would let a timed `replace` destroy a fault that was already
/// running.
///
/// Takes the rules as an argument rather than reading them from a port: a
/// candidate stack has to be folded *before* it is stored, which is what lets a
/// rejected rule leave the installed ones untouched.
pub(crate) fn fold_rules<T: SimulationMessage>(
    rules: &[Rule],
    upstream: T,
) -> Result<T, serde_json::Error> {
    let visible = rules
        .iter()
        .rposition(|rule| !rule.mode.is_relative())
        .unwrap_or(0);
    rules[visible..]
        .iter()
        .try_fold(upstream, |value, rule| apply_override(rule, value))
}
