//! What a payload may name, and the "did you mean" when it names something that
//! does not exist.
//!
//! Two functions cross out of this module: [`reject_unknown_paths`] and
//! [`children_of_null_fields`]. Everything else here serves them.
//!
//! Neither asks what mode is in force. A [`Request`] reports the paths it
//! addresses, so these read that rather than re-deriving it from a raw payload.

use std::collections::BTreeSet;

use anyhow::{Result, bail};

use super::rule::Request;
use super::schema::{FieldSpec, TargetSpec};

/// Refuses any path the type does not have.
///
/// Serde ignores unknown struct fields, so without this a misspelled field
/// merges in, deserialises cleanly, and changes nothing while reporting
/// success.
pub(super) fn reject_unknown_paths(spec: &TargetSpec, request: &Request) -> Result<()> {
    let known: BTreeSet<&str> = spec
        .fields
        .iter()
        .map(|field| field.path.as_str())
        .collect();

    for path in request.named_paths() {
        // The root is the one pointer with no name in it to misspell, so it is
        // always a path the type has. A payload assigning an object there had
        // its fields walked and named one by one, and reaches here only when it
        // assigned something with no fields — a scalar, which a newtype message
        // is. Refusing it told the operator such a message had no such field,
        // when what they had was the only field it has.
        if path.is_empty() {
            continue;
        }
        if known.contains(path.as_str()) || indexes_past_a_known_array(spec, &path) {
            continue;
        }
        // A pointer names one place, and a place may be a whole object. The
        // schema lists only leaves, so a container is recognised by having
        // leaves beneath it rather than by being listed itself. The value is
        // left to the apply, which deserialises it as that field's type.
        if known.iter().any(|leaf| descends_from(leaf, &path)) {
            continue;
        }
        bail!(
            "unknown field '{path}' on a {}{}",
            short_type_name(spec.type_name),
            nearest_path(&known, &path),
        );
    }
    Ok(())
}

/// The payload's paths that reach under a field the value holds as `null`.
///
/// A `None` advertises no children, so a payload naming one names something no
/// runtime value can confirm *or deny* — see `Target::validate`. Treated as
/// plausible so the apply's own error surfaces instead. Narrowly: a name with no
/// such parent is still a name error and still gets its suggestion.
pub(super) fn children_of_null_fields(spec: &TargetSpec, request: &Request) -> Vec<FieldSpec> {
    let unpopulated: Vec<&str> = spec
        .fields
        .iter()
        .filter(|field| field.kind == "null")
        .map(|field| field.path.as_str())
        .collect();
    if unpopulated.is_empty() {
        return Vec::new();
    }

    request
        .named_paths()
        .into_iter()
        .filter(|path| unpopulated.iter().any(|parent| descends_from(path, parent)))
        .map(|path| FieldSpec {
            path,
            kind: "null",
            default: serde_json::Value::Null,
        })
        .collect()
}

/// Whether `path` names something strictly inside `parent`.
///
/// A prefix test alone would match `/orientation_dot` against `/orientation`, so
/// the separator has to be there.
fn descends_from(path: &str, parent: &str) -> bool {
    path.len() > parent.len() && path.starts_with(parent) && path.as_bytes()[parent.len()] == b'/'
}

/// Whether `path` reaches past the end of an array the reference value holds.
///
/// Elements that exist are enumerated, so an index within range is checked like
/// any other path — that is what catches `/wheels/0/torqeu`. Past the end there
/// is nothing to check against, and an array the payload grows is legitimately
/// addressable there, so the pointer resolving reports what misses instead. An
/// array that is empty here is entirely unchecked for the same reason.
///
/// Every ancestor is tried, not just the immediate parent: an element need not
/// end the path. `RwArrayConfigMsg::spin_axes_body` is one 3-vector per wheel,
/// so wheel 0's y-axis is `/spin_axes_body/0/1`, two segments below the array.
fn indexes_past_a_known_array(spec: &TargetSpec, path: &str) -> bool {
    let mut boundary = path.len();
    while let Some(slash) = path[..boundary].rfind('/') {
        let ancestor = &path[..slash];
        let beneath = path[slash + 1..].split('/').next().unwrap_or_default();
        if let Ok(index) = beneath.parse::<usize>()
            && spec.fields.iter().any(|field| {
                field.path == ancestor
                    && field
                        .default
                        .as_array()
                        .is_some_and(|items| index >= items.len())
            })
        {
            return true;
        }
        boundary = slash;
    }
    false
}

/// The last segment of a `std::any::type_name`, which is the name a person
/// would recognise. Unqualified names are returned as they are.
fn short_type_name(type_name: &str) -> &str {
    type_name
        .rsplit_once("::")
        .map_or(type_name, |(_, name)| name)
}

/// A "did you mean" for a single-character slip, which is what a hand-typed
/// field name usually is.
fn nearest_path(known: &BTreeSet<&str>, path: &str) -> String {
    known
        .iter()
        .find(|candidate| differs_by_one_edit(candidate, path))
        .map_or_else(String::new, |candidate| {
            format!(" (did you mean '{candidate}'?)")
        })
}

/// Whether one edit turns `left` into `right`: a character changed, inserted,
/// deleted, or swapped with its neighbour.
///
/// Measured rather than approximated, and that matters more than it looks: array
/// elements put every index in the known set, so a loose measure has hundreds of
/// near-identical paths to choose wrongly among. Counting characters the two
/// strings do not share suggested `/spin_axes_body/10/0` for
/// `/spin_axes_body/0/why`, which shares all of them.
fn differs_by_one_edit(left: &str, right: &str) -> bool {
    let (left, right): (Vec<char>, Vec<char>) = (left.chars().collect(), right.chars().collect());
    let (short, long) = if left.len() <= right.len() {
        (&left, &right)
    } else {
        (&right, &left)
    };
    if long.len() - short.len() > 1 {
        return false;
    }

    // What matches from each end. An insertion or deletion is covered when the
    // two ends meet; a substitution when they leave one character between them.
    let head = short.iter().zip(long).take_while(|(a, b)| a == b).count();
    let tail = short
        .iter()
        .rev()
        .zip(long.iter().rev())
        .take_while(|(a, b)| a == b)
        .count();
    let same_length = short.len() == long.len();
    if head + tail + usize::from(same_length) >= short.len() {
        return true;
    }

    // A swap leaves two characters between the ends, each the other's opposite.
    same_length
        && head + tail + 2 == short.len()
        && short[head] == long[head + 1]
        && short[head + 1] == long[head]
}
