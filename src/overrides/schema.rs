//! What a target reports about itself, and how that is derived.
//!
//! The schema is not written by hand anywhere. It is read off `T::default()` at
//! runtime: serialise the type's own default, walk the result, and every leaf it
//! reaches is a field an operator may address. A field added to a message type
//! therefore reaches clients without anything here being touched, and a field
//! removed stops being advertised for the same reason.

use serde::{Serialize, Serializer};
use serde_json::Value;

/// What layer of the simulation a target sits at.
///
/// The two named kinds are the only ones this crate can recognise, because they
/// are the two ports it defines: [`Output`](crate::messages::Output) and
/// [`Input`](crate::messages::Input). The distinction between them is a fact
/// about the mechanism — an output override is seen by every consumer, an input
/// override by one — so it means the same thing in every simulation built on
/// this crate.
///
/// What an application *puts* in a port is a different question, and not this
/// crate's to answer. A port carrying a device's tuning, a link's fault
/// settings, a test harness's inputs: all three are an `Output<T>` here, and
/// which of them a given target is describes that application's hardware, not
/// this crate's model of it. Those names are declared downstream through
/// [`TargetKind::Custom`] rather than added here.
///
/// Deliberately has no `Default`: every registration states its kind, and a
/// default would let a missed one pass as an output.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TargetKind {
    /// A message a module publishes. Every consumer sees the override.
    Output,
    /// One consumer's view of a message. Other consumers are unaffected.
    Input,
    /// A kind this crate has no opinion about, named by the application.
    ///
    /// The string reaches a client verbatim, so it is part of *that*
    /// application's contract with its own tooling, not of this crate's.
    Custom(&'static str),
}

impl TargetKind {
    /// The name a client sees. `Custom` passes its own through unchanged.
    pub fn as_str(self) -> &'static str {
        match self {
            Self::Output => "output",
            Self::Input => "input",
            Self::Custom(name) => name,
        }
    }
}

/// Serialised as the bare name rather than as a tagged enum, so a `Custom` kind
/// is indistinguishable on the wire from a built-in one. A client reads `kind`
/// as a string and should not have to know which of the two it is looking at.
impl Serialize for TargetKind {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(self.as_str())
    }
}

/// One overridable field.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct FieldSpec {
    /// Dotted path from the root of the target, e.g. `bias.x` or `sample_hz`.
    pub path: String,
    /// `number`, `integer`, `boolean`, `string`, `array`, `object` or `null`.
    pub kind: &'static str,
    /// This field's value in `T::default()`.
    pub default: Value,
}

/// A target's static schema: everything knowable without a running simulation.
#[derive(Clone, Debug, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct TargetSpec {
    pub kind: TargetKind,
    pub type_name: &'static str,
    /// `T::default()`, which is what the `default` override mode applies.
    ///
    /// Not the configured baseline that `clear` restores — that is only
    /// knowable from a running simulation.
    pub type_default: Value,
    pub fields: Vec<FieldSpec>,
}

/// The name a client sees for the JSON type of a default value.
fn value_kind(value: &Value) -> &'static str {
    match value {
        Value::Null => "null",
        Value::Bool(_) => "boolean",
        Value::Number(number) if number.is_f64() => "number",
        Value::Number(_) => "integer",
        Value::String(_) => "string",
        Value::Array(_) => "array",
        Value::Object(_) => "object",
    }
}

/// Visits every addressable leaf of `value` with its dotted path.
///
/// Objects recurse; arrays do not. A JSON merge replaces an array wholesale, so
/// an element path would advertise a per-index patch the merge cannot honour.
/// An empty object is a leaf for the same reason: there is nothing under it to
/// name. The root itself is not a field, so it is never visited.
///
/// One walk serves both the schema and the payload check, so a target cannot
/// advertise a path shape that the check then reads differently. That is why
/// this is shared with [`super::paths`] rather than duplicated there.
pub(super) fn walk_leaves<F: FnMut(&str, &Value)>(prefix: &str, value: &Value, visit: &mut F) {
    if let Value::Object(map) = value
        && !map.is_empty()
    {
        for (key, child) in map {
            let path = if prefix.is_empty() {
                key.clone()
            } else {
                format!("{prefix}.{key}")
            };
            walk_leaves(&path, child, visit);
        }
        return;
    }
    if !prefix.is_empty() {
        visit(prefix, value);
    }
}

/// One entry per overridable leaf of a type default.
pub(super) fn leaf_fields(type_default: &Value) -> Vec<FieldSpec> {
    let mut fields = Vec::new();
    walk_leaves("", type_default, &mut |path, value| {
        fields.push(FieldSpec {
            path: path.to_string(),
            kind: value_kind(value),
            default: value.clone(),
        });
    });
    fields
}
