//! What a payload may name, whether it names enough of them, and the "did you
//! mean" when it names something that does not exist.
//!
//! Two functions cross out of this module: [`reject_unknown_paths`] and
//! [`require_complete_replacement`]. Everything else here serves them.

use std::collections::BTreeSet;

use anyhow::{Context, Result, bail};
use serde_json::Value;

use super::mode::Mode;
use super::schema::{FieldSpec, TargetSpec, walk_leaves};

/// The leaf paths an override payload actually addresses.
pub(super) fn payload_paths(value: &Value) -> Vec<String> {
    let mut paths = Vec::new();
    walk_leaves("", value, &mut |path, _| paths.push(path.to_string()));
    paths
}

/// Refuses any path the type does not have.
///
/// Serde ignores unknown struct fields, so without this a misspelled field
/// merges in, deserialises cleanly, and changes nothing while reporting
/// success.
///
/// Where the paths come from depends on the mode. A merge document is walked; a
/// [`Mode::PointerReplace`] document carries its paths inside the
/// operations, in pointer rather than dotted form.
pub(super) fn reject_unknown_paths(spec: &TargetSpec, mode: Mode, value: &Value) -> Result<()> {
    let known: BTreeSet<&str> = spec
        .fields
        .iter()
        .map(|field| field.path.as_str())
        .collect();

    let paths = match mode {
        Mode::PointerReplace => patch_operation_paths(value)?,
        _ => payload_paths(value),
    };

    for path in paths {
        if known.contains(path.as_str()) || indexes_a_known_array(spec, &path) {
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

/// Refuses a [`Mode::Replace`] that does not name every field.
///
/// `replace` advertises "a complete message", and without this it is not one.
/// Message and config types carry `#[serde(default)]` so that a new field does
/// not invalidate stored values, which also means a partial document
/// deserialises happily — omitted fields come back as `T::default()` rather than
/// as an error. So a payload naming one field is accepted, reported as
/// installed, and silently resets every field it did not mention:
///
/// ```text
/// baseline               { bias: 0.1, scale: 4.0, sample_hz: 200.0 }
/// replace {"scale":1.0}  { bias: 0.0, scale: 1.0, sample_hz:   0.0 }   <- was accepted
/// ```
///
/// Zeroing a sample rate while reporting success is a worse failure than any
/// fault an operator meant to inject, so the shortfall is refused here instead.
/// Every other mode is unaffected: `patch` and `pointerReplace` are partial by
/// definition, and `freeze` and `default` supply no payload to be short.
pub(super) fn require_complete_replacement(
    spec: &TargetSpec,
    mode: Mode,
    value: &Value,
) -> Result<()> {
    if mode != Mode::Replace {
        return Ok(());
    }

    let named: BTreeSet<String> = payload_paths(value).into_iter().collect();
    let missing: Vec<&str> = spec
        .fields
        .iter()
        .map(|field| field.path.as_str())
        .filter(|path| !named.contains(*path))
        .collect();

    if missing.is_empty() {
        return Ok(());
    }
    bail!(
        "a replace must name every field of a {}, and this one omits {} of {}: {}. \
         Omitted fields would be silently reset to the type default rather than \
         left alone — use 'patch' to change only the fields you name.",
        short_type_name(spec.type_name),
        missing.len(),
        spec.fields.len(),
        missing.join(", "),
    )
}

/// The payload's paths that reach under a field the value holds as `null`.
///
/// An `Option` that is `None` serialises to a null leaf, so it advertises no
/// children — and a payload naming one of them names something no runtime value
/// can confirm *or deny*. Calling that a misspelling denies a field that does
/// exist: `PlanetStateMsg::default()` has `orientation: None`, so a replacement
/// filling it in was told `unknown field 'orientation.inertial_to_fixed'` when
/// the field was real and only its value was wrong.
///
/// Settling it properly needs a static schema of the type; this crate derives its
/// schema from values and cannot see into a `None`. So these paths are treated as
/// plausible, which lets the apply's own error be the one that surfaces. The
/// narrowness matters: a name with no such parent is still a name error and still
/// gets its suggestion.
pub(super) fn children_of_null_fields(
    spec: &TargetSpec,
    mode: Mode,
    value: &Value,
) -> Vec<FieldSpec> {
    let unpopulated: Vec<&str> = spec
        .fields
        .iter()
        .filter(|field| field.kind == "null")
        .map(|field| field.path.as_str())
        .collect();
    if unpopulated.is_empty() {
        return Vec::new();
    }

    let paths = match mode {
        // A malformed document is reported by `reject_unknown_paths`, which runs
        // next and raises the same error rather than an empty path list.
        Mode::PointerReplace => patch_operation_paths(value).unwrap_or_default(),
        _ => payload_paths(value),
    };

    paths
        .into_iter()
        .filter(|path| unpopulated.iter().any(|parent| descends_from(path, parent)))
        .map(|path| FieldSpec {
            path,
            kind: "null",
            default: Value::Null,
        })
        .collect()
}

/// Whether `path` names something strictly inside `parent`.
///
/// A prefix test alone would match `orientation_dot` against `orientation`, so
/// the separator has to be there.
fn descends_from(path: &str, parent: &str) -> bool {
    path.len() > parent.len() && path.starts_with(parent) && path.as_bytes()[parent.len()] == b'.'
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
    let Some((parent, index)) = path.rsplit_once('.') else {
        return false;
    };
    index.parse::<usize>().is_ok()
        && spec
            .fields
            .iter()
            .any(|field| field.path == parent && field.kind == "array")
}

/// The paths an RFC 6902 document addresses, in the dotted form the schema uses.
fn patch_operation_paths(document: &Value) -> Result<Vec<String>> {
    document
        .as_array()
        .with_context(|| {
            format!("a pointerReplace value must be an array of operations: {document}")
        })?
        .iter()
        .map(|operation| {
            operation
                .get("path")
                .and_then(Value::as_str)
                .map(pointer_to_dotted)
                .with_context(|| format!("a pointerReplace operation needs a 'path': {operation}"))
        })
        .collect()
}

/// An RFC 6901 pointer written as the dotted path the schema uses.
///
/// The two escapes belong to the pointer syntax rather than to anything a Rust
/// field name can contain, so they are undone rather than carried through.
fn pointer_to_dotted(pointer: &str) -> String {
    pointer
        .trim_start_matches('/')
        .split('/')
        .map(|token| token.replace("~1", "/").replace("~0", "~"))
        .collect::<Vec<_>>()
        .join(".")
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
