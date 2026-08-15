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
        if known.contains(path.as_str()) || indexes_a_known_array(spec, &path) {
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

/// Whether `path` addresses an element of a field the type holds as an array.
///
/// The schema names arrays rather than their elements, and deliberately: an
/// element path is derivable from the array's own default, so listing both
/// would have a client render one control per component *and* another for the
/// whole vector. Checking an index against its parent instead also keeps a
/// growable array addressable past the length its default happens to have.
///
/// Whether the index is *in range* is left to the pointer resolving, which
/// reports the miss with the path in it.
fn indexes_a_known_array(spec: &TargetSpec, path: &str) -> bool {
    let Some((parent, index)) = path.rsplit_once('/') else {
        return false;
    };
    index.parse::<usize>().is_ok()
        && spec
            .fields
            .iter()
            .any(|field| field.path == parent && field.kind == "array")
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
        .find(|candidate| {
            candidate.len().abs_diff(path.len()) <= 1
                && candidate
                    .chars()
                    .filter(|character| !path.contains(*character))
                    .count()
                    <= 1
        })
        .map_or_else(String::new, |candidate| {
            format!(" (did you mean '{candidate}'?)")
        })
}
