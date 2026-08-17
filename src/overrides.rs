//! The registry of things a fault can be injected into.
//!
//! [`crate::messages`] can override a port you already hold. It cannot tell you
//! what exists — what is overridable, what fields it has, or whether a
//! hand-typed field name is real. This module is that half: anything injectable
//! becomes a [`Target`], and [`Registry::register`] is the only way to make one.
//!
//! Every path a payload names is checked against the type here, because serde
//! silently ignores a field it does not recognise — without the check a typo
//! reports success and changes nothing.
//!
//! What is injectable and what a payload may say is here; *when* a rule goes in
//! and when it comes out is `schedule`, which drives this registry from a clock
//! rather than adding anything to it.
//!
//! | file | holds |
//! |---|---|
//! | this one | [`Target`] and the registry |
//! | `rule` | [`Request`], [`Rule`], [`RuleId`], and what they compute |
//! | `schema` | [`TargetKind`], [`FieldSpec`], [`TargetSpec`], derived from `T::default()` |
//! | `paths` | whether a payload may name what it names, and the "did you mean" when it may not |
//! | `schedule` | [`Fault`], [`Case`], [`FaultSchedule`] — when a rule is installed and when it lifts |

pub(crate) mod rule;

mod paths;
mod schedule;
mod schema;

#[cfg(test)]
mod tests;

use std::collections::BTreeMap;
use std::sync::{Arc, Mutex, MutexGuard};

use anyhow::{Result, bail};
use serde_json::Value;

use crate::messages::Port;

use paths::{children_of_null_fields, reject_unknown_paths};
use schema::{addressable_fields, leaf_fields};

pub use rule::{Assignment, Document, Mode, Pointer, Request, Rule, RuleId, Selection};
pub use schedule::{Case, Fault, FaultEvent, FaultEventKind, FaultSchedule, FaultSender};
pub use schema::{FieldSpec, TargetKind, TargetSpec};

/// A module that can list its own overridable ports.
///
/// Implemented once per module *type*, so exposing an instance costs one line
/// rather than one per port.
///
/// ```
/// use basilisk_rs::messages::Output;
/// use basilisk_rs::overrides::{Overridable, Registry, Request, TargetKind};
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
///     fn register_targets(&self, registry: &Registry, prefix: &str) -> anyhow::Result<()> {
///         registry.register(
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
/// registry.register_module("gyro_0", &gyro)?;
///
/// // The name is what an operator addresses: `<prefix>.<port>`.
/// registry.install("gyro_0.output_msg", Request::replace(json!({ "/rate_dps": 9.0 }))?)?;
///
/// // The module reads its own port and sees the fault, without knowing one was
/// // installed. What it published is still available separately.
/// assert_eq!(gyro.output_msg.read().rate_dps, 9.0);
/// assert_eq!(gyro.output_msg.read_upstream().rate_dps, 1.0);
/// # Ok::<(), anyhow::Error>(())
/// ```
pub trait Overridable {
    /// Registers this module's targets under `prefix`, e.g. `gyro_0`.
    fn register_targets(&self, registry: &Registry, prefix: &str) -> Result<()>;
}

/// The port operations a target is built from, in JSON terms.
trait TargetBackend: Send + Sync {
    fn type_name(&self) -> &'static str;
    fn type_default(&self) -> Result<Value>;
    fn upstream(&self) -> Result<Value>;
    fn effective(&self) -> Result<Value>;
    /// The value this target would take, without installing anything.
    fn preview(&self, request: Request) -> Result<Value>;
    fn install(&self, request: Request) -> Result<RuleId>;
    fn clear(&self);
    fn clear_rule(&self, id: RuleId) -> bool;
    fn rules(&self) -> Vec<Rule>;
}

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

    fn preview(&self, request: Request) -> Result<Value> {
        Ok(serde_json::to_value(Port::preview_override(
            self, request,
        )?)?)
    }

    fn install(&self, request: Request) -> Result<RuleId> {
        Ok(Port::set_override(self, request)?)
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

/// Something a fault can be injected into.
///
/// One registered [`Port`], plus the `kind` label the port itself has no way to
/// know. The port is held type-erased, so targets carrying unrelated message
/// types share one map.
///
/// [`Registry::register`] is the only constructor, so no target can exist that
/// skips [`Self::install`]'s checks.
pub struct Target {
    backend: Box<dyn TargetBackend>,
    kind: TargetKind,
}

impl Target {
    fn new<P: Port + 'static>(port: &P, kind: TargetKind) -> Self {
        Self {
            backend: Box::new(port.clone()),
            kind,
        }
    }

    /// The schema and type default. Static; needs no running simulation.
    pub fn spec(&self) -> Result<TargetSpec> {
        let type_default = self.backend.type_default()?;
        Ok(TargetSpec {
            kind: self.kind,
            type_name: self.backend.type_name(),
            fields: leaf_fields(&type_default),
            type_default,
        })
    }

    /// Shape only: the payload may name just fields the target has, and the
    /// result must be a well-formed value of the message type. Values are never
    /// judged — an absurd fault is the point.
    ///
    /// The names are checked against the message the payload would *produce*,
    /// not against `T::default()`. The default cannot answer the question: an
    /// `Option` field is `None` there, so it advertises no children — while a
    /// payload that fills it in names those children, and they are real fields
    /// of the type. Checking against the default refused a complete, correctly
    /// typed replacement of `PlanetStateMsg { orientation: Some(..), .. }` as
    /// naming a field that "does not exist". Enums and maps fail the same way.
    ///
    /// The candidate is asked instead — the payload applied and round-tripped
    /// through `T`. Serde has already dropped what it did not recognise by then,
    /// so a name the payload uses and the candidate lacks is precisely a name
    /// serde ignored, which is the failure this exists to catch.
    pub fn validate(&self, request: &Request) -> Result<()> {
        let mut spec = self.spec()?;

        match self.backend.preview(request.clone()) {
            // The message produced is the authority on which fields exist.
            Ok(candidate) => {
                spec.fields = addressable_fields(&candidate);
                reject_unknown_paths(&spec, request)
            }
            // Two very different reasons the payload might not apply. Its
            // values may be wrong — `inertial_to_fixed: "not a matrix"` — where
            // the apply's own error is the truthful answer and "unknown field"
            // would deny a field that exists. Or it names nothing real, which
            // fails in the apply as an unresolvable pointer.
            //
            // So names are judged against everything reachable *without*
            // applying: the type default, plus what the port holds now, which
            // may have options the default leaves `None`. Unknown to both is a
            // name error. Otherwise the apply's error stands.
            Err(apply_error) => {
                spec.fields = addressable_fields(&spec.type_default);
                if let Ok(effective) = self.backend.effective() {
                    spec.fields.extend(addressable_fields(&effective));
                }
                // Neither value can see inside an option that is `None` in both,
                // so a path under one is not evidence of a misspelling.
                let unprovable = children_of_null_fields(&spec, request);
                spec.fields.extend(unprovable);

                reject_unknown_paths(&spec, request)?;
                Err(apply_error)
            }
        }
    }

    /// Validates, then installs. Returns the id of the new rule.
    pub fn install(&self, request: Request) -> Result<RuleId> {
        self.validate(&request)?;
        self.backend.install(request)
    }

    pub fn clear(&self) {
        self.backend.clear();
    }

    /// Removes the one layer `id` names, leaving every other rule in force.
    /// Returns whether that layer was there to remove.
    pub fn clear_rule(&self, id: RuleId) -> bool {
        self.backend.clear_rule(id)
    }

    /// The rules in force, innermost first. Empty when nothing is overridden.
    pub fn rules(&self) -> Vec<Rule> {
        self.backend.rules()
    }

    /// The value before *this* target's override.
    ///
    /// For an output that is what the simulation produced. For an input it is
    /// the producer's effective value, which is itself overridden if the
    /// producer has a rule — it is not ground truth.
    pub fn upstream(&self) -> Result<Value> {
        self.backend.upstream()
    }

    /// The value after this target's override.
    pub fn effective(&self) -> Result<Value> {
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
    targets: Arc<Mutex<BTreeMap<String, Arc<Target>>>>,
}

impl Registry {
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers every target a module exposes, under `name`.
    pub fn register_module(&self, name: &str, module: &dyn Overridable) -> Result<()> {
        module.register_targets(self, name)
    }

    /// Registers one port — an [`Output`](crate::messages::Output) or an
    /// [`Input`](crate::messages::Input) — under `kind`.
    ///
    /// `kind` is a label for the operator, never a restriction: targets of
    /// different kinds accept exactly the same commands. [`TargetKind::Custom`]
    /// carries a word this crate has no opinion about.
    ///
    /// Every leaf of the message type is registered, without bounds.
    ///
    /// Refuses a port that cannot yet report honestly — see
    /// [`Port::registration_refusal`], which catches an input registered before
    /// it was wired, rather than letting it describe faults against a value no
    /// producer published.
    pub fn register<P: Port + 'static>(
        &self,
        name: impl Into<String>,
        port: &P,
        kind: TargetKind,
    ) -> Result<()> {
        let name = name.into();
        if let Some(refusal) = port.registration_refusal() {
            bail!("override target '{name}' cannot be registered: {refusal}");
        }
        self.insert(name, Arc::new(Target::new(port, kind)))
    }

    fn insert(&self, name: String, target: Arc<Target>) -> Result<()> {
        let mut targets = self.lock();
        if targets.contains_key(&name) {
            bail!("override target '{name}' is already registered");
        }
        targets.insert(name, target);
        Ok(())
    }

    pub fn install(&self, target: &str, request: Request) -> Result<RuleId> {
        self.target(target)?.install(request)
    }

    /// Whether an override would be accepted, without installing it.
    pub fn validate(&self, target: &str, request: &Request) -> Result<()> {
        self.target(target)?.validate(request)
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
    /// A snapshot: targets registered afterwards are not in it, and the registry
    /// lock is not held while the caller works through the result.
    pub fn targets(&self) -> Vec<(String, Arc<Target>)> {
        self.lock()
            .iter()
            .map(|(name, target)| (name.clone(), Arc::clone(target)))
            .collect()
    }

    /// An empty registry is reported as such rather than as a missing name: the
    /// two have different fixes — correct the name, or wire the registry up at
    /// all — and "unknown target 'x'" would send an operator to check a spelling
    /// that was never wrong.
    fn target(&self, name: &str) -> Result<Arc<Target>> {
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

    fn lock(&self) -> MutexGuard<'_, BTreeMap<String, Arc<Target>>> {
        self.targets.lock().expect("override registry lock")
    }
}
