//! What a rule is, and what it computes.
//!
//! Three modes, and they differ only in where the value comes from: a
//! [`Mode::Replace`] takes it from the payload, a [`Mode::Default`] from the
//! type's own default, a [`Mode::Freeze`] from the live value at the moment it
//! is installed. All three are **relative** — they touch the paths they name and
//! leave the rest to the layers beneath — so a stack is always the composition
//! of all of its rules and nothing masks anything.
//!
//! The payload is typed rather than raw JSON, and a [`Rule`] can only be built
//! through [`Rule::build`], which resolves and checks it. So an installed rule
//! is a rule that was validated: there is no second field to disagree with the
//! first, and no way to construct one that skipped the checks.
//!
//! Custody is not here — [`crate::messages`] owns each port's stack and decides
//! when a fold becomes visible; this file answers what a given stack computes,
//! identically for a candidate stack and an installed one.

use std::sync::atomic::{AtomicU64, Ordering};

use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::{Map, Value};

use crate::messages::SimulationMessage;

/// Which of the three a rule is. A label for reporting; the payload that goes
/// with it lives in [`Request`], where it cannot be paired with the wrong one.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Mode {
    Replace,
    Freeze,
    Default,
}

// ---------------------------------------------------------------------------
// Pointers
// ---------------------------------------------------------------------------

/// An RFC 6901 JSON pointer, checked when it is built.
///
/// One path syntax serves the whole feature: the schema publishes pointers, a
/// `replace` document addresses them, and a freeze names them. Nothing converts
/// between two spellings, so nothing can disagree about what a path means.
///
/// The empty pointer addresses the whole document, which is what makes
/// "the whole message" expressible as a path rather than as a special case.
#[derive(Clone, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct Pointer(String);

impl Pointer {
    /// The whole document. RFC 6901 spells this as the empty string.
    pub const ROOT: Self = Self(String::new());

    /// Checks the one rule pointer syntax has: empty, or starting with `/`.
    ///
    /// A dotted path is the likely mistake — it is what the schema published
    /// before — so it is answered with its own translation rather than with a
    /// restatement of the rule.
    pub fn parse(raw: &str) -> Result<Self, String> {
        if !raw.is_empty() && !raw.starts_with('/') {
            return Err(format!(
                "'{raw}' is not a JSON pointer: write '/{}' instead",
                raw.replace('.', "/")
            ));
        }
        // `~` introduces an escape, and RFC 6901 defines exactly two. A stray
        // one is a typo rather than a literal tilde — the literal is `~0` — and
        // resolving it would silently address a member nobody named.
        let mut characters = raw.chars();
        while let Some(character) = characters.next() {
            if character == '~' && !matches!(characters.next(), Some('0' | '1')) {
                return Err(format!(
                    "'{raw}' is not a JSON pointer: '~' escapes only '~0' (a literal '~') \
                     and '~1' (a literal '/')"
                ));
            }
        }
        Ok(Self(raw.to_string()))
    }

    pub fn as_str(&self) -> &str {
        &self.0
    }
}

impl Serialize for Pointer {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_str(&self.0)
    }
}

impl<'de> Deserialize<'de> for Pointer {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let raw = String::deserialize(deserializer)?;
        Self::parse(&raw).map_err(serde::de::Error::custom)
    }
}

// ---------------------------------------------------------------------------
// Payloads
// ---------------------------------------------------------------------------

/// One `replace` operation: a location, and what to put there.
///
/// Carries no operation name, because there is only one it could hold. On the
/// wire it still reads and writes `"op": "replace"`, so a client that sends a
/// conforming RFC 6902 document is understood unchanged.
#[derive(Clone, Debug, PartialEq)]
pub struct ReplaceOp {
    pub path: Pointer,
    pub value: Value,
}

impl ReplaceOp {
    fn from_value(operation: &Value) -> Result<Self, String> {
        let object = operation
            .as_object()
            .ok_or_else(|| format!("an operation must be an object: {operation}"))?;

        match object.get("op").and_then(Value::as_str) {
            Some("replace") => {}
            Some(other) => {
                return Err(format!(
                    "this implements the 'replace' operation of RFC 6902 and no other, so \
                     '{other}' is refused rather than approximated: a message has a fixed \
                     schema and fixed-size arrays, which leaves the other five either \
                     unrepresentable or indistinguishable from 'replace'"
                ));
            }
            None => return Err(format!("an operation needs 'op': {operation}")),
        }

        let path = object
            .get("path")
            .and_then(Value::as_str)
            .ok_or_else(|| format!("an operation needs 'path': {operation}"))?;
        let value = object
            .get("value")
            .ok_or_else(|| format!("a replace needs 'value': {operation}"))?;

        Ok(Self {
            path: Pointer::parse(path)?,
            value: value.clone(),
        })
    }
}

impl Serialize for ReplaceOp {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        let mut object = Map::new();
        object.insert("op".to_string(), Value::from("replace"));
        object.insert("path".to_string(), Value::from(self.path.as_str()));
        object.insert("value".to_string(), self.value.clone());
        Value::Object(object).serialize(serializer)
    }
}

impl<'de> Deserialize<'de> for ReplaceOp {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = Value::deserialize(deserializer)?;
        Self::from_value(&value).map_err(serde::de::Error::custom)
    }
}

/// What a [`Mode::Replace`] carries: either shape of patch document.
///
/// The two are told apart by their JSON shape, which is unambiguous — a merge
/// document is an object, an RFC 6902 document is an array — so no extra tag is
/// needed on the wire.
#[derive(Clone, Debug, PartialEq)]
pub enum Document {
    /// A merge document. Nested JSON mirroring the message, so it names no
    /// paths and cannot address one element of an array.
    Merge(Map<String, Value>),
    /// RFC 6902 operations, which can.
    Ops(Vec<ReplaceOp>),
}

impl Document {
    fn from_value(value: Value) -> Result<Self, String> {
        match value {
            Value::Object(map) => Ok(Self::Merge(map)),
            Value::Array(operations) => operations
                .iter()
                .map(ReplaceOp::from_value)
                .collect::<Result<Vec<_>, _>>()
                .map(Self::Ops),
            other => Err(format!(
                "a replace takes an object (a merge document) or an array (RFC 6902 \
                 operations), and this is neither: {other}"
            )),
        }
    }
}

impl Serialize for Document {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match self {
            Self::Merge(map) => map.serialize(serializer),
            Self::Ops(operations) => operations.serialize(serializer),
        }
    }
}

impl<'de> Deserialize<'de> for Document {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = Value::deserialize(deserializer)?;
        Self::from_value(value).map_err(serde::de::Error::custom)
    }
}

/// Which fields a [`Mode::Freeze`] or [`Mode::Default`] acts on.
///
/// An empty payload means the whole message.
///
/// Holds its pointers privately, and [`Selection::fields`] folds an empty list
/// into [`Selection::whole`], so "the whole message" has exactly one
/// representation and the same JSON cannot mean two things depending on which
/// side built it.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Selection {
    /// `None` is the whole message. Never `Some` of an empty list.
    fields: Option<Vec<Pointer>>,
}

impl Selection {
    /// The whole message.
    pub fn whole() -> Self {
        Self { fields: None }
    }

    /// Just the fields these pointers name, or the whole message if there are
    /// none — an empty selection is not a rule that does nothing.
    pub fn fields(pointers: Vec<Pointer>) -> Self {
        if pointers.is_empty() {
            return Self::whole();
        }
        Self {
            fields: Some(pointers),
        }
    }

    fn from_value(value: Value) -> Result<Self, String> {
        match value {
            Value::Null => Ok(Self::whole()),
            Value::Object(map) if map.is_empty() => Ok(Self::whole()),
            Value::Array(paths) => paths
                .iter()
                .map(|path| {
                    path.as_str()
                        .ok_or_else(|| format!("a field must be named by a pointer: {path}"))
                        .and_then(Pointer::parse)
                })
                .collect::<Result<Vec<_>, _>>()
                .map(Self::fields),
            other => Err(format!(
                "a freeze or default takes an array of JSON pointers naming the fields to \
                 act on, or an empty value for the whole message, and this is neither: {other}"
            )),
        }
    }

    fn pointers(&self) -> &[Pointer] {
        self.fields.as_deref().unwrap_or(&[])
    }

    /// Whether this names the whole message.
    pub fn is_whole(&self) -> bool {
        self.fields.is_none()
    }
}

impl Serialize for Selection {
    fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
        match &self.fields {
            Some(paths) => paths.serialize(serializer),
            None => Value::Null.serialize(serializer),
        }
    }
}

impl<'de> Deserialize<'de> for Selection {
    fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        let value = Value::deserialize(deserializer)?;
        Self::from_value(value).map_err(serde::de::Error::custom)
    }
}

// ---------------------------------------------------------------------------
// Requests
// ---------------------------------------------------------------------------

/// What a client asks for, parsed once at the boundary.
///
/// Adjacently tagged, so the wire form is `{"mode": ..., "value": ...}` —
/// exactly what it was when the mode and its payload were separate fields.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "mode", content = "value", rename_all = "camelCase")]
pub enum Request {
    Replace(Document),
    Freeze(Selection),
    Default(Selection),
}

impl Request {
    /// A replace carrying either patch format, chosen by the payload's shape.
    pub fn replace(value: Value) -> anyhow::Result<Self> {
        Document::from_value(value)
            .map(Self::Replace)
            .map_err(anyhow::Error::msg)
    }

    /// A freeze of the fields `value` names, or of the whole message when it is
    /// empty.
    pub fn freeze(value: Value) -> anyhow::Result<Self> {
        Selection::from_value(value)
            .map(Self::Freeze)
            .map_err(anyhow::Error::msg)
    }

    /// A reset of the fields `value` names, or of the whole message when it is
    /// empty.
    pub fn default(value: Value) -> anyhow::Result<Self> {
        Selection::from_value(value)
            .map(Self::Default)
            .map_err(anyhow::Error::msg)
    }

    pub fn mode(&self) -> Mode {
        match self {
            Self::Replace(_) => Mode::Replace,
            Self::Freeze(_) => Mode::Freeze,
            Self::Default(_) => Mode::Default,
        }
    }

    /// The paths this request addresses, as pointers.
    ///
    /// What the name checks read, so that they ask the request what it names
    /// rather than re-deriving it from a raw payload. A whole-message selection
    /// names nothing: there is no path to be wrong.
    pub(crate) fn named_paths(&self) -> Vec<String> {
        match self {
            Self::Replace(Document::Merge(map)) => {
                let mut paths = Vec::new();
                super::schema::walk_leaves("", &Value::Object(map.clone()), &mut |path, _| {
                    paths.push(path.to_string());
                });
                paths
            }
            Self::Replace(Document::Ops(operations)) => operations
                .iter()
                .map(|operation| operation.path.as_str().to_string())
                .collect(),
            Self::Freeze(selection) | Self::Default(selection) => selection
                .pointers()
                .iter()
                .map(|path| path.as_str().to_string())
                .collect(),
        }
    }

    /// Whether this request addresses locations rather than nesting into the
    /// message, which decides whether a named path may be a container.
    pub(crate) fn addresses_locations(&self) -> bool {
        !matches!(self, Self::Replace(Document::Merge(_)))
    }
}

// ---------------------------------------------------------------------------
// Rules
// ---------------------------------------------------------------------------

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

/// One rule in force on a port: what it does, and which fault owns it.
///
/// The fields are private and `Rule::build` is the only constructor, so
/// holding a `Rule` is evidence that its payload was parsed, its pointers
/// resolved and its values captured. Nothing downstream re-checks any of that.
///
/// A freeze and a default are already resolved to concrete values by the time
/// they get here, so all three modes are one document to apply; `mode` survives
/// only to tell a client which was asked for.
#[derive(Clone, Debug, PartialEq, Serialize)]
pub struct Rule {
    mode: Mode,
    #[serde(rename = "value")]
    document: Document,
    id: RuleId,
}

impl Rule {
    /// Resolves `request` against the values it draws from, and checks it.
    ///
    /// `live` and `type_default` are required rather than optional, so a freeze
    /// cannot be built without the value it freezes.
    ///
    /// A pointer is resolved against the value that mode reads from — the live
    /// value for a freeze, the type default for a default — and one that misses
    /// is refused here, rather than installed as a layer that does nothing on
    /// every write.
    pub(crate) fn build(
        request: Request,
        live: &Value,
        type_default: &Value,
        id: RuleId,
    ) -> Result<Self, serde_json::Error> {
        let mode = request.mode();
        let document = match request {
            Request::Replace(document) => document,
            Request::Freeze(selection) => capture(&selection, live, "freeze")?,
            Request::Default(selection) => capture(&selection, type_default, "default")?,
        };
        Ok(Self { mode, document, id })
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn id(&self) -> RuleId {
        self.id
    }

    /// The document this rule applies, with a freeze or default already
    /// resolved to the values it captured.
    pub fn document(&self) -> &Document {
        &self.document
    }
}

/// Reads the values `selection` names out of `source`, as replace operations.
///
/// The whole message is the root pointer rather than a special case, so a
/// whole-message freeze and a per-field one produce the same kind of document.
fn capture(
    selection: &Selection,
    source: &Value,
    what: &str,
) -> Result<Document, serde_json::Error> {
    if selection.is_whole() {
        return Ok(Document::Ops(vec![ReplaceOp {
            path: Pointer::ROOT,
            value: source.clone(),
        }]));
    }
    selection
        .pointers()
        .iter()
        .map(|path| {
            source
                .pointer(path.as_str())
                .map(|found| ReplaceOp {
                    path: path.clone(),
                    value: found.clone(),
                })
                .ok_or_else(|| {
                    json_error(format!(
                        "a {what} names '{}', which does not resolve on the value it reads from",
                        path.as_str()
                    ))
                })
        })
        .collect::<Result<Vec<_>, _>>()
        .map(Document::Ops)
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
/// element-wise, since a merge document has no way to name one element. An
/// RFC 6902 document is what addresses a single element.
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

/// A `serde_json::Error` carrying a message of our own.
///
/// The override path returns `serde_json::Error` throughout, and the failures
/// here are not deserialisation failures. Widening the error type would ripple
/// through `fold`, `install` and `preview` on both ports; this borrows the one
/// constructor serde offers for the purpose.
fn json_error(message: impl std::fmt::Display) -> serde_json::Error {
    <serde_json::Error as serde::de::Error>::custom(message)
}

/// Produces the value `rule` yields when laid over `upstream`.
///
/// Takes no view on the mode: by this point a freeze and a default are documents
/// like any other, so there is one way to apply all three.
pub(crate) fn apply_override<T: SimulationMessage>(
    rule: &Rule,
    upstream: T,
) -> Result<T, serde_json::Error> {
    let mut base = serde_json::to_value(upstream)?;
    match rule.document() {
        Document::Merge(map) => merge_json(&mut base, Value::Object(map.clone())),
        Document::Ops(operations) => {
            for operation in operations {
                match base.pointer_mut(operation.path.as_str()) {
                    Some(slot) => *slot = operation.value.clone(),
                    None => {
                        return Err(json_error(format!(
                            "'{}' does not resolve on this value",
                            operation.path.as_str()
                        )));
                    }
                }
            }
        }
    }
    serde_json::from_value(base)
}

/// Produces the value an installed stack yields when laid over `upstream`,
/// innermost first, skipping any rule that cannot apply to this value.
///
/// Every rule is relative, so this is the plain composition of all of them.
///
/// A rule is checked against the message when it is installed, but the *shape*
/// of that message can change afterwards — an `Option` a module was populating
/// goes `None`, and a pointer addressing something beneath it stops resolving.
/// That is a rule with nothing to do this tick rather than a bad rule, and
/// failing the fold over it would discard every fault installed alongside it.
/// For a mechanism whose purpose is injecting faults, quietly injecting none is
/// the worst answer available.
///
/// Returns the rules that were skipped so the caller can say so, rather than
/// dropping them silently.
pub(crate) fn fold_installed<T: SimulationMessage>(
    rules: &[Rule],
    upstream: T,
) -> (T, Vec<(RuleId, serde_json::Error)>) {
    let mut skipped = Vec::new();
    let value = rules.iter().fold(upstream, |value, rule| {
        match apply_override(rule, value.clone()) {
            Ok(applied) => applied,
            Err(error) => {
                skipped.push((rule.id(), error));
                value
            }
        }
    });
    (value, skipped)
}

/// Produces the value `installed` plus one candidate yields, refusing the
/// candidate if it cannot apply.
///
/// Used where a rule is being *admitted* — an install or a preview. Only the
/// candidate is judged: a candidate that cannot apply is the caller's mistake
/// and refusing it is what leaves the installed stack alone, while the rules
/// already in force are read exactly as they are on any other write. Judging
/// both strictly let one stale layer refuse every later change to the stack.
pub(crate) fn fold_candidate<T: SimulationMessage>(
    installed: &[Rule],
    candidate: &Rule,
    upstream: T,
) -> Result<T, serde_json::Error> {
    let (value, _) = fold_installed(installed, upstream);
    apply_override(candidate, value)
}
