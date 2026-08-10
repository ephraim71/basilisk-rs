//! What a single rule computes.
//!
//! `OverrideCell::fold` stores and composes rules; this decides what
//! one of them does to a value. `apply_override` is the seam between the two —
//! `fold` calls it once per visible layer.
//!
//! A child module of `messages` because that is whose mechanics it implements,
//! not for access: everything it touches is public now that the rule vocabulary
//! lives in [`crate::overrides::mode`]. `OverrideCell::fold` is the only caller,
//! and keeping the two together is what makes the seam between "compose the
//! stack" and "apply one layer" legible.

use serde_json::Value;

use crate::overrides::mode::{Mode, Rule};

use super::SimulationMessage;

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
/// what [`Mode::PointerReplace`] exists for.
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
fn apply_pointer_replace(base: &mut Value, document: &Value) -> Result<(), serde_json::Error> {
    let operations = document.as_array().ok_or_else(|| {
        json_error(format!(
            "a pointerReplace value must be an array: {document}"
        ))
    })?;

    for operation in operations {
        let name = operation.get("op").and_then(Value::as_str).ok_or_else(|| {
            json_error(format!(
                "a pointerReplace operation needs 'op': {operation}"
            ))
        })?;
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
                json_error(format!(
                    "a pointerReplace operation needs 'path': {operation}"
                ))
            })?;
        let value = operation.get("value").ok_or_else(|| {
            json_error(format!(
                "a pointerReplace replace needs 'value': {operation}"
            ))
        })?;

        match base.pointer_mut(path) {
            Some(slot) => *slot = value.clone(),
            None => {
                return Err(json_error(format!(
                    "pointerReplace path '{path}' does not resolve on this value"
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
pub(super) fn apply_override<T: SimulationMessage>(
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
        Mode::PointerReplace => {
            let mut base = serde_json::to_value(upstream)?;
            apply_pointer_replace(&mut base, &rule.value)?;
            serde_json::from_value(base)
        }
        Mode::Default => Ok(T::default()),
    }
}
