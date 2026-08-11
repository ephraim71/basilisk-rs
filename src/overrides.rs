//! The registry of things a fault can be injected into.
//!
//! [`crate::messages`] gives every port the *mechanics* of an override: the stack
//! of rules behind a port, and what a `patch` or a `replace` does to a value. The
//! vocabulary it speaks while doing so — [`Mode`], [`Rule`], [`RuleId`] — is
//! declared here, in `mode`, because both halves need it and neither is its
//! natural owner. What it does not give you is any way to find out
//! what exists. A caller holding an [`Output`](crate::messages::Output) can
//! override it; a caller holding a whole simulation has no way to ask what is
//! overridable, what fields a target has, or whether a hand-typed field name is
//! real.
//!
//! This module is that half. Everything injectable — a message a module
//! publishes, one consumer's view of a message, a device's configuration, or
//! whatever an application layers on top of a port — is a [`Target`]:
//! it describes its own schema, judges a proposed override against that schema,
//! installs one, and reports the value before and after. The registry stores
//! them all behind that one trait, so a single control path serves every kind.
//!
//! There is one way in: [`Registry::register`] takes any [`Port`] — an `Output`
//! or an `Input` — and the [`TargetKind`] to file it under. The kind is always
//! named rather than inferred, so a registration states what a client will see
//! instead of leaving it to be deduced from which method was called.
//!
//! # Layout
//!
//! | file | holds |
//! |---|---|
//! | this one | the two public traits, the one implementation behind them, and the registry |
//! | `mode` | [`Mode`], [`Rule`], [`RuleId`] — the vocabulary both halves share |
//! | `schema` | [`TargetKind`], [`FieldSpec`], [`TargetSpec`], and the walk that derives them from `T::default()` |
//! | `paths` | whether a payload may name what it names, and the "did you mean" when it may not |
//!
//! `Entry<B>` and the registry share this file because the registry constructs
//! `Entry` directly, reading fields that are private to it. Splitting them
//! would mean widening those fields for no gain.
//!
//! # The two gates
//!
//! A proposed override passes two independent checks before it is visible:
//!
//! 1. **Here**, in [`Target::install`]: every path the payload names must
//!    exist on the type. Serde would silently ignore an unknown field, so
//!    without this a typo reports success and changes nothing.
//! 2. **In [`crate::messages`]**, inside the port: the folded result must
//!    deserialise back into the message type, and that trial runs on a *copy* of
//!    the rule stack so a rejected rule cannot disturb the rules already
//!    installed.
//!
//! The first is about names, the second about values. Neither subsumes the
//! other, but the first is built *on* the second: it asks the port to apply the
//! payload without installing it, and compares the names the payload used
//! against the message that came back. Only the round trip knows which fields
//! exist on the value at hand, because an `Option` that is `None` in
//! `T::default()` has children the default cannot show.
//!
//! # What is deliberately not here
//!
//! No JSON envelope. The registry hands back [`TargetSpec`] and
//! `Arc<dyn Target>`; deciding what a manifest or a status dump looks
//! like on the wire is the application's business, because that shape is a
//! contract with *its* clients and would otherwise be frozen by this crate.
//! [`Registry::targets`] is the door for building one.

pub(crate) mod mode;

mod paths;
mod schema;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use anyhow::{Result, bail};
use serde_json::Value;

use crate::messages::Port;

use paths::{children_of_null_fields, reject_unknown_paths, require_complete_replacement};
use schema::leaf_fields;

pub use mode::{Mode, Rule, RuleId};
pub use schema::{FieldSpec, TargetKind, TargetSpec};

// ---------------------------------------------------------------------------
// Abstraction: the two public traits
// ---------------------------------------------------------------------------

/// A module that can list its own overridable ports.
///
/// Implemented once per module *type*, listing its ports. Exposing an instance
/// then costs a single line where the simulation is assembled instead of one
/// line per port, and a port added to the module reaches clients without that
/// assembly being touched.
///
/// ```
/// use basilisk_rs::messages::Output;
/// use basilisk_rs::overrides::{Mode, Overridable, Registry, TargetKind};
/// use serde_json::json;
///
/// #[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
/// struct GyroMsg {
///     rate_dps: f64,
/// }
///
/// struct Gyro {
///     output_msg: Output<GyroMsg>,
/// }
///
/// // Written once for the type. A port added here reaches every client without
/// // the assembly below being touched.
/// impl Overridable for Gyro {
///     fn register_targets(
///         &self,
///         registry: &Registry,
///         prefix: &str,
///         group: &'static str,
///     ) -> anyhow::Result<()> {
///         registry.register(
///             group,
///             format!("{prefix}.output_msg"),
///             &self.output_msg,
///             TargetKind::Output,
///         )
///     }
/// }
///
/// let gyro = Gyro {
///     output_msg: Output::new(GyroMsg { rate_dps: 1.0 }),
/// };
///
/// let registry = Registry::new();
/// registry.register_module("GYRO", "gyro_0", &gyro)?;
///
/// // The name is what an operator addresses: `<prefix>.<port>`.
/// registry.install("gyro_0.output_msg", Mode::Patch, json!({ "rate_dps": 9.0 }))?;
///
/// // The module reads its own port and sees the fault, without knowing one was
/// // installed. What it published is still available separately.
/// assert_eq!(gyro.output_msg.read().rate_dps, 9.0);
/// assert_eq!(gyro.output_msg.read_upstream().rate_dps, 1.0);
/// # Ok::<(), anyhow::Error>(())
/// ```
pub trait Overridable {
    /// Registers this module's targets under `prefix`, e.g. `gyro_0`, tagged
    /// with the hardware family `group`, e.g. `GYRO`.
    fn register_targets(
        &self,
        registry: &Registry,
        prefix: &str,
        group: &'static str,
    ) -> Result<()>;
}

/// Something a fault can be injected into.
pub trait Target: Send + Sync {
    /// The schema and type default. Static; needs no running simulation.
    fn spec(&self) -> Result<TargetSpec>;

    /// Whether this override would be accepted, without installing it.
    fn validate(&self, mode: Mode, value: &Value) -> Result<()>;

    /// Validates, then installs. Returns the id of the new rule.
    fn install(&self, mode: Mode, value: Value) -> Result<RuleId>;

    fn clear(&self);

    /// Removes the one layer `id` names, leaving every other rule in force.
    /// Returns whether that layer was there to remove.
    fn clear_rule(&self, id: RuleId) -> bool;

    /// The rules in force, innermost first. Empty when nothing is overridden.
    fn rules(&self) -> Vec<Rule>;

    /// The value before *this* target's override.
    ///
    /// For an output that is what the simulation produced. For an input it is
    /// the producer's effective value, which is itself overridden if the
    /// producer has a rule — it is not ground truth.
    fn upstream(&self) -> Result<Value>;

    /// The value after this target's override.
    fn effective(&self) -> Result<Value>;
}

// ---------------------------------------------------------------------------
// Implementation: one `Entry<B>` serving every kind
// ---------------------------------------------------------------------------

/// The handful of operations a target must supply, in JSON terms.
///
/// Schema generation and payload checking are derived from these once, in
/// [`Entry`], rather than reimplemented per kind. Private because the only
/// implementation is the blanket one below: a caller supplies a
/// [`Port`] and this crate adapts it, so there is nothing here for an
/// application to implement.
trait TargetBackend: Send + Sync {
    fn type_name(&self) -> &'static str;
    fn type_default(&self) -> Result<Value>;
    fn upstream(&self) -> Result<Value>;
    fn effective(&self) -> Result<Value>;
    /// The value this target would take, without installing anything.
    fn preview(&self, mode: Mode, value: Value) -> Result<Value>;
    fn install(&self, mode: Mode, value: Value) -> Result<RuleId>;
    fn clear(&self);
    fn clear_rule(&self, id: RuleId) -> bool;
    fn rules(&self) -> Vec<Rule>;
}

/// One implementation serving both directions.
///
/// [`Port`] is the whole reason this is one block rather than two: the adapters
/// it replaced were identical apart from the type they wrapped, and a second
/// copy of a body is a second place for a fix to be forgotten.
impl<P: Port> TargetBackend for P {
    fn type_name(&self) -> &'static str {
        std::any::type_name::<P::Message>()
    }

    fn type_default(&self) -> Result<Value> {
        Ok(serde_json::to_value(P::Message::default())?)
    }

    fn upstream(&self) -> Result<Value> {
        Ok(serde_json::to_value(Port::read_upstream(self))?)
    }

    fn effective(&self) -> Result<Value> {
        Ok(serde_json::to_value(Port::read(self))?)
    }

    fn preview(&self, mode: Mode, value: Value) -> Result<Value> {
        Ok(serde_json::to_value(Port::preview_override(
            self, mode, value,
        )?)?)
    }

    fn install(&self, mode: Mode, value: Value) -> Result<RuleId> {
        Ok(Port::set_override(self, mode, value)?)
    }

    fn clear(&self) {
        Port::clear_override(self);
    }

    fn clear_rule(&self, id: RuleId) -> bool {
        Port::clear_override_by_id(self, id)
    }

    fn rules(&self) -> Vec<Rule> {
        Port::installed_overrides(self)
    }
}

struct Entry<B> {
    backend: B,
    kind: TargetKind,
    group: &'static str,
}

impl<B: TargetBackend> Entry<B> {
    /// The policy both entry points share, derived once from one spec so the
    /// two checks cannot disagree about what the target's fields are.
    /// Checks the payload's names against the message it would *produce*, not
    /// against `T::default()`.
    ///
    /// The default cannot answer the question on its own. An `Option` field is
    /// `None` there, so it serialises to one null leaf and advertises no
    /// children — while a payload that fills it in names those children, and
    /// they are real fields of the type. Deriving the check from the default
    /// refused a complete, correctly typed replacement of
    /// `PlanetStateMsg { orientation: Some(..), .. }` as naming a field that
    /// "does not exist". The same held for a `patch` of any option a module had
    /// already populated. Enums and maps fail the same way for the same reason.
    ///
    /// So the candidate is asked instead: the payload applied and round-tripped
    /// through `T`. Serde has already dropped whatever it did not recognise by
    /// then, which makes the comparison exact rather than approximate — a name
    /// the payload uses and the candidate lacks is precisely a name serde
    /// ignored, which is the whole failure this check exists to catch.
    fn check_payload(&self, mode: Mode, value: &Value) -> Result<()> {
        let mut spec = self.spec()?;

        match self.backend.preview(mode, value.clone()) {
            // The payload applies. The message it produced is then the authority
            // on which fields exist, and any name the payload used that is absent
            // from it is exactly a name serde ignored.
            Ok(candidate) => {
                spec.fields = leaf_fields(&candidate);
                reject_unknown_paths(&spec, mode, value)?;
                require_complete_replacement(&spec, mode, value)
            }
            // The payload does not apply, and there are two very different
            // reasons for that. Its values may be wrong — `inertial_to_fixed:
            // "not a matrix"` — in which case the apply's own error is the
            // truthful answer and reporting "unknown field" would deny a field
            // that exists. Or it may name something that is not a field at all,
            // which for a `pointerReplace` fails in the apply too, as a pointer
            // that would not resolve.
            //
            // Names are judged against every inhabitation reachable *without*
            // applying: the type default, plus the value the port holds right
            // now, which may have options populated that the default leaves
            // `None`. A name unknown to both is a name error, and gets its
            // suggestion. Anything else means the names were plausible and the
            // values were not, so the apply's error stands.
            Err(apply_error) => {
                if let Ok(effective) = self.backend.effective() {
                    spec.fields.extend(leaf_fields(&effective));
                }
                // Neither value can see inside an option that is `None` in both,
                // so a path under one is not evidence of a misspelling.
                let unprovable = children_of_null_fields(&spec, mode, value);
                spec.fields.extend(unprovable);

                reject_unknown_paths(&spec, mode, value)?;
                Err(apply_error)
            }
        }
    }
}

/// One implementation serves every kind: `Entry<B>` is a [`Target`] for any
/// backend, so a new kind needs a backend and no new target logic.
impl<B: TargetBackend> Target for Entry<B> {
    fn spec(&self) -> Result<TargetSpec> {
        let type_default = self.backend.type_default()?;
        Ok(TargetSpec {
            kind: self.kind,
            group: self.group,
            type_name: self.backend.type_name(),
            fields: leaf_fields(&type_default),
            type_default,
        })
    }

    /// Three checks, all about shape rather than plausibility: the payload may
    /// only name fields the target has, a `replace` must name all of them, and
    /// the result has to be a well-formed value of the message type. Any value
    /// that survives those is installed — a fault is allowed to be absurd, and a
    /// simulation that cannot survive one is the result being sought.
    ///
    /// Unknown paths are reported before a shortfall, so a misspelling in an
    /// otherwise complete `replace` reads as the typo it is rather than as a
    /// missing field plus an unknown one.
    ///
    /// One apply, not two: [`Self::check_payload`] has to apply the payload to
    /// learn which fields the result has, so it already answers the third check
    /// on the way to the first.
    fn validate(&self, mode: Mode, value: &Value) -> Result<()> {
        self.check_payload(mode, value)
    }

    fn install(&self, mode: Mode, value: Value) -> Result<RuleId> {
        self.check_payload(mode, &value)?;
        self.backend.install(mode, value)
    }

    fn clear(&self) {
        self.backend.clear();
    }

    fn clear_rule(&self, id: RuleId) -> bool {
        self.backend.clear_rule(id)
    }

    fn rules(&self) -> Vec<Rule> {
        self.backend.rules()
    }

    fn upstream(&self) -> Result<Value> {
        self.backend.upstream()
    }

    fn effective(&self) -> Result<Value> {
        self.backend.effective()
    }
}

// ---------------------------------------------------------------------------
// Registry: the entry point callers hold
// ---------------------------------------------------------------------------

/// Every injectable target in one simulation, addressed by name.
///
/// Names are `module.port`, e.g. `gyro_0.config`, assigned where the module
/// is registered. Cloning shares the same set of targets, so a control socket
/// and a schema dump see one registry.
#[derive(Clone, Default)]
pub struct Registry {
    targets: Arc<Mutex<BTreeMap<String, Arc<dyn Target>>>>,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers every target a module exposes, under `name`, tagged `group`.
    pub fn register_module(
        &self,
        group: &'static str,
        name: &str,
        module: &dyn Overridable,
    ) -> Result<()> {
        module.register_targets(self, name, group)
    }

    /// Registers one port — an [`Output`](crate::messages::Output) or an
    /// [`Input`](crate::messages::Input) — under `kind`.
    ///
    /// The kind is always named rather than inferred from the port type, so
    /// every registration says on its own line what a client will see. It is a
    /// label for the operator and never a restriction: two targets of different
    /// kinds accept exactly the same commands. [`TargetKind::Custom`] carries a
    /// word this crate has no opinion about, which is how an application
    /// describes hardware in its own vocabulary.
    ///
    /// Every leaf of the message type is registered, without bounds.
    ///
    /// A port that cannot yet report honestly is refused — see
    /// [`Port::registration_refusal`], which is how an input registered before
    /// it was wired is caught here rather than left to describe faults against a
    /// value no producer published.
    pub fn register<P: Port + 'static>(
        &self,
        group: &'static str,
        name: impl Into<String>,
        port: &P,
        kind: TargetKind,
    ) -> Result<()> {
        let name = name.into();
        if let Some(refusal) = port.registration_refusal() {
            bail!("override target '{name}' cannot be registered: {refusal}");
        }
        self.insert(
            name,
            Arc::new(Entry {
                backend: port.clone(),
                kind,
                group,
            }),
        )
    }

    fn insert(&self, name: String, target: Arc<dyn Target>) -> Result<()> {
        let mut targets = self.lock();
        if targets.contains_key(&name) {
            bail!("override target '{name}' is already registered");
        }
        targets.insert(name, target);
        Ok(())
    }

    pub fn install(&self, target: &str, mode: Mode, value: Value) -> Result<RuleId> {
        self.target(target)?.install(mode, value)
    }

    /// Whether an override would be accepted, without installing it.
    pub fn validate(&self, target: &str, mode: Mode, value: &Value) -> Result<()> {
        self.target(target)?.validate(mode, value)
    }

    pub fn clear(&self, target: &str) -> Result<()> {
        self.target(target)?.clear();
        Ok(())
    }

    /// Removes the one override `id` names from `target`.
    ///
    /// A timed fault clears itself this way, so an override applied after it
    /// cannot be cleared by its expiry.
    ///
    /// `Ok(false)` means the id named no layer that was still installed. An
    /// unknown target is an error rather than the same `false`, so a mistyped
    /// name cannot be read as a fault that had already gone.
    pub fn clear_rule(&self, target: &str, id: RuleId) -> Result<bool> {
        Ok(self.target(target)?.clear_rule(id))
    }

    pub fn clear_all(&self) {
        for target in self.lock().values() {
            target.clear();
        }
    }

    pub fn upstream(&self, target: &str) -> Result<Value> {
        self.target(target)?.upstream()
    }

    pub fn effective(&self, target: &str) -> Result<Value> {
        self.target(target)?.effective()
    }

    pub fn spec(&self, target: &str) -> Result<TargetSpec> {
        self.target(target)?.spec()
    }

    pub fn target_names(&self) -> Vec<String> {
        self.lock().keys().cloned().collect()
    }

    /// Whether a target of this name is registered.
    pub fn contains(&self, target: &str) -> bool {
        self.lock().contains_key(target)
    }

    /// Every registered target with its name, in name order.
    ///
    /// This is how an application builds its own schema dump or status view: the
    /// registry deliberately publishes no JSON envelope of its own, because that
    /// shape is a contract with the application's clients rather than with this
    /// crate's.
    ///
    /// A snapshot. Targets registered afterwards are not in it, and the registry
    /// lock is not held while the caller works through the result.
    pub fn targets(&self) -> Vec<(String, Arc<dyn Target>)> {
        self.lock()
            .iter()
            .map(|(name, target)| (name.clone(), Arc::clone(target)))
            .collect()
    }

    /// An empty registry is reported as such rather than as a missing name.
    ///
    /// A simulation that registered nothing refuses every command, and
    /// "unknown target 'x'" reads as a typo — sending an operator to check a
    /// spelling that was never wrong. The two cases need different answers
    /// because they have different fixes: correct the name, or wire the
    /// registry up in the first place.
    fn target(&self, name: &str) -> Result<Arc<dyn Target>> {
        let targets = self.lock();
        if targets.is_empty() {
            bail!(
                "this simulation registered no override targets, so '{name}' \
                 cannot be resolved and no target can be"
            );
        }
        match targets.get(name) {
            Some(target) => Ok(Arc::clone(target)),
            None => bail!("unknown override target '{name}'"),
        }
    }

    fn lock(&self) -> MutexGuard<'_, BTreeMap<String, Arc<dyn Target>>> {
        self.targets.lock().expect("override registry lock")
    }
}
