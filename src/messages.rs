//! Messages, the ports that carry them, and the override rules that can be laid
//! over one in flight.
//!
//! This module is four things at once, which is worth knowing before reading it:
//!
//! | region | holds |
//! |---|---|
//! | Re-exports | every concrete message type, one per file in `messages/` |
//! | Message contract | [`SimulationMessage`] — what a type must satisfy to travel |
//! | Override rules | the layer stack behind a port; what each mode *does* is `messages/rules.rs` |
//! | Ports | [`Output`] and [`Input`] — the handles modules actually hold |
//!
//! # Overrides: what lives here, and what does not
//!
//! This module owns the **mechanics**: the stack of rules behind a port, how they
//! fold together, and what a `patch` or a `pointerReplace` does to a JSON value.
//! The vocabulary those mechanics speak — [`Mode`], [`Rule`], [`RuleId`] — is
//! declared in [`crate::overrides`] and used here, not defined here. This module
//! has no idea what a *target* is, cannot enumerate what is injectable, and never
//! validates a field name.
//!
//! That half — the registry, the schema derived from `T::default()` and the
//! unknown-path check — is [`crate::overrides`]. It decides **what may be
//! overridden and whether a request is well formed**; this module decides
//! **what an installed rule does to a value**.
//!
//! The line between them is privacy, not taste. `OverrideCell` is a private
//! field of [`Output`] with private methods that [`Output::write`] calls, so it
//! cannot move without widening that surface. [`crate::overrides`] touches only
//! the public API of these ports, which is why it *can* sit apart.
//!
//! Neither module renders JSON. What a manifest or a status dump looks like on
//! the wire is a contract with an application's clients, so that shape belongs
//! to the application — see [`crate::overrides::Registry::targets`].

use serde::{Serialize, de::DeserializeOwned};
use serde_json::Value;
use std::sync::{Arc, RwLock};

use crate::overrides::mode::{Mode, Rule, RuleId, next_rule_id};

mod array_motor_torque;
mod array_motor_voltage;
mod atmosphere;
mod attitude_guidance;
mod attitude_reference;
mod body_torque_command;
mod eclipse;
mod gps;
mod hinged_rigid_body;
mod imu;
mod magnetic_dipole_command;
mod magnetic_field;
mod motor_torque;
mod mtb_array_command;
mod mtb_array_config;
mod navigation_attitude;
mod planet_state;
mod power_node_usage;
mod power_storage_fault;
mod power_storage_status;
mod reaction_wheel_state;
mod rw_array_config;
mod rw_availability;
mod rw_speed;
mod solar_flux;
mod spacecraft_diagnostics;
mod spacecraft_mass_props;
mod spacecraft_state;
mod star_tracker;
mod sun_ephemeris;
mod sun_line;
mod sun_sensor;
mod tam_sensor;
mod tam_sensor_body;
mod thruster_command;
mod translation_reference;
mod vehicle_config;

/// Not a message type: what a single override rule computes.
mod rules;

use rules::apply_override;

pub use array_motor_torque::ArrayMotorTorqueMsg;
pub use array_motor_voltage::ArrayMotorVoltageMsg;
pub use atmosphere::AtmosphereMsg;
pub use attitude_guidance::AttitudeGuidanceMsg;
pub use attitude_reference::AttitudeReferenceMsg;
pub use body_torque_command::BodyTorqueCommandMsg;
pub use eclipse::EclipseMsg;
pub use gps::GpsMsg;
pub use hinged_rigid_body::HingedRigidBodyMsg;
pub use imu::ImuMsg;
pub use magnetic_dipole_command::MagneticDipoleCommandMsg;
pub use magnetic_field::MagneticFieldMsg;
pub use motor_torque::MotorTorqueMsg;
pub use mtb_array_command::MtbArrayCommandMsg;
pub use mtb_array_config::MtbArrayConfigMsg;
pub use navigation_attitude::NavigationAttitudeMsg;
pub use planet_state::{PlanetOrientation, PlanetStateMsg};
pub use power_node_usage::PowerNodeUsageMsg;
pub use power_storage_fault::PowerStorageFaultMsg;
pub use power_storage_status::PowerStorageStatusMsg;
pub use reaction_wheel_state::ReactionWheelStateMsg;
pub use rw_array_config::RwArrayConfigMsg;
pub use rw_availability::{RwAvailability, RwAvailabilityMsg};
pub use rw_speed::RwSpeedMsg;
pub use solar_flux::SolarFluxMsg;
pub use spacecraft_diagnostics::SpacecraftDiagnosticsMsg;
pub use spacecraft_mass_props::SpacecraftMassPropsMsg;
pub use spacecraft_state::SpacecraftStateMsg;
pub use star_tracker::StarTrackerMsg;
pub use sun_ephemeris::SunEphemerisMsg;
pub use sun_line::SunLineMsg;
pub use sun_sensor::SunSensorMsg;
pub use tam_sensor::TamSensorMsg;
pub use tam_sensor_body::TamSensorBodyMsg;
pub use thruster_command::ThrusterCommandMsg;
pub use translation_reference::TranslationReferenceMsg;
pub use vehicle_config::VehicleConfigMsg;

pub const MAX_EFF_COUNT: usize = 36;

/// Serde support for the `[T; MAX_EFF_COUNT]` effector arrays.
///
/// `serde` only provides array impls up to length 32, and `MAX_EFF_COUNT` is
/// larger, so the arrays are carried over the wire as sequences instead.
pub(crate) mod big_array {
    use serde::{Deserialize, Deserializer, Serialize, Serializer, de::Error as _};

    pub fn serialize<S, T, const N: usize>(value: &[T; N], serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
        T: Serialize,
    {
        value.as_slice().serialize(serializer)
    }

    pub fn deserialize<'de, D, T, const N: usize>(deserializer: D) -> Result<[T; N], D::Error>
    where
        D: Deserializer<'de>,
        T: Deserialize<'de>,
    {
        let values = Vec::<T>::deserialize(deserializer)?;
        let len = values.len();
        <[T; N]>::try_from(values).map_err(|_| D::Error::invalid_length(len, &"MAX_EFF_COUNT"))
    }
}

// ---------------------------------------------------------------------------
// Message contract
// ---------------------------------------------------------------------------

/// What a type must satisfy to travel between modules: cloneable for the read
/// path, defaultable for an unconnected input, and serde-round-trippable so an
/// override can be expressed as JSON.
pub trait SimulationMessage:
    Clone + Default + Serialize + DeserializeOwned + Send + Sync + 'static
{
}

impl<T> SimulationMessage for T where
    T: Clone + Default + Serialize + DeserializeOwned + Send + Sync + 'static
{
}

/// The override rules behind an `Output` or an `Input`, innermost first.
///
/// A stack rather than a single rule because a fault has an owner. Merging
/// successive patches into one rule loses that: the merged rule carries one
/// id, so removing the fault that owns it either takes the others with it or
/// cannot reach it at all. Layers keep each fault removable on its own.
///
/// Both directions of a connection need identical semantics, so this lives here
/// once rather than being reimplemented on each side.
#[derive(Clone, Debug, Default)]
struct OverrideCell {
    rules: Arc<RwLock<Vec<Rule>>>,
}

impl OverrideCell {
    fn snapshot(&self) -> Vec<Rule> {
        self.rules
            .read()
            .expect("failed to lock override rules for read")
            .clone()
    }

    /// Whether anything is installed, without copying the stack to find out.
    fn is_empty(&self) -> bool {
        self.rules
            .read()
            .expect("failed to lock override rules for read")
            .is_empty()
    }

    /// Lays `rules` over `upstream` in order, innermost first.
    ///
    /// Starts at the outermost absolute rule. `Replace`, `Freeze` and `Default`
    /// define the whole message, so nothing below one can be observed while it
    /// is installed — but the layers stay in the stack, and reappear when it is
    /// removed. Discarding them instead would let a timed `replace` destroy a
    /// fault that was already running.
    fn fold<T: SimulationMessage>(rules: &[Rule], upstream: T) -> Result<T, serde_json::Error> {
        let visible = rules
            .iter()
            .rposition(|rule| !rule.mode.is_relative())
            .unwrap_or(0);
        rules[visible..]
            .iter()
            .try_fold(upstream, |value, rule| apply_override(rule, value))
    }

    /// The value `mode` and `value` would produce, without installing anything.
    ///
    /// Takes the same already-sampled `freeze_source` as [`Self::install`], so
    /// that previewing a rule and installing it read from identical inputs.
    fn preview<T: SimulationMessage>(
        &self,
        mode: Mode,
        value: Value,
        freeze_source: Option<Value>,
        upstream: T,
    ) -> Result<T, serde_json::Error> {
        let mut rules = self.snapshot();
        rules.push(Rule::new(mode, value, freeze_source, RuleId::UNINSTALLED));
        Self::fold(&rules, upstream)
    }

    /// Installs a rule, returning its id and the value the whole stack now
    /// produces.
    ///
    /// The lock spans building, applying and storing so that a concurrent
    /// caller cannot install a layer between the value being computed and the
    /// rule being stored.
    ///
    /// `upstream` and `freeze_source` are sampled by the caller *before* this is
    /// entered: reading them here would take the value locks while holding the
    /// rules lock, which is the opposite of the order `write` uses.
    fn install<T>(
        &self,
        mode: Mode,
        value: Value,
        freeze_source: Option<Value>,
        upstream: T,
    ) -> Result<(RuleId, T), serde_json::Error>
    where
        T: SimulationMessage,
    {
        let mut rules = self
            .rules
            .write()
            .expect("failed to lock override rules for write");

        let candidate = Rule::new(mode, value, freeze_source, next_rule_id());
        let id = candidate.id;
        // Validate against a copy and commit only on success. Undoing a failed
        // install in place cannot be done by removing the candidate alone: an
        // absolute rule displaces everything beneath it, and those layers would
        // stay lost.
        let mut candidates = rules.clone();
        candidates.push(candidate);
        let effective = Self::fold(&candidates, upstream)?;

        *rules = candidates;
        Ok((id, effective))
    }

    fn clear(&self) {
        self.rules
            .write()
            .expect("failed to lock override rules for write")
            .clear();
    }

    /// Removes the layer `id` identifies, leaving the rest in place.
    fn clear_rule(&self, id: RuleId) -> bool {
        let mut rules = self
            .rules
            .write()
            .expect("failed to lock override rules for write");
        let before = rules.len();
        rules.retain(|rule| rule.id != id);
        rules.len() != before
    }

    fn apply_installed<T: SimulationMessage>(&self, upstream: T) -> Result<T, serde_json::Error> {
        Self::fold(&self.snapshot(), upstream)
    }
}

// ---------------------------------------------------------------------------
// Ports
// ---------------------------------------------------------------------------

/// A message a module publishes.
///
/// Cloning gives another handle to the same message, not a copy of it.
///
/// # Consistency under concurrent access
///
/// The raw value, the rule stack and the effective value are three pieces of
/// state behind separate locks. Each is individually consistent, and each read
/// method returns a coherent value, but a `write` racing a `set_override` or a
/// `clear_override` is **eventually** consistent rather than atomic across all
/// three: the two orderings differ in which effective value is stored, and the
/// next `write` settles it either way.
///
/// The scheduler steps modules one at a time, so this is only reachable from a
/// control path running on another thread. What it costs there is one tick of a
/// stale effective value; the rules themselves are never lost or duplicated.
#[derive(Clone, Debug)]
pub struct Output<T> {
    slot: Arc<RwLock<T>>,
    raw_slot: Arc<RwLock<T>>,
    overrides: OverrideCell,
}

impl<T> Output<T> {
    pub(crate) fn slot(&self) -> Arc<RwLock<T>> {
        Arc::clone(&self.slot)
    }

    /// Points `input` at this output.
    ///
    /// [`crate::simulation::Simulation::connect`] delegates here; this form is
    /// for wiring done outside a simulation, such as tests.
    pub fn connect_to(&self, input: &mut Input<T>) {
        input.connect(self.slot());
    }

    /// The rules in force, innermost first. Empty when nothing is overridden.
    pub fn installed_overrides(&self) -> Vec<Rule> {
        self.overrides.snapshot()
    }

    pub fn is_overridden(&self) -> bool {
        !self.overrides.is_empty()
    }
}

impl<T: Clone> Output<T> {
    pub fn new(initial: T) -> Self {
        Self {
            slot: Arc::new(RwLock::new(initial.clone())),
            raw_slot: Arc::new(RwLock::new(initial)),
            overrides: OverrideCell::default(),
        }
    }

    /// The value every consumer sees: raw, with any override applied.
    pub fn read(&self) -> T {
        self.slot
            .read()
            .expect("failed to lock output message for read")
            .clone()
    }

    /// The value the simulation actually produced, ignoring any override.
    pub fn read_upstream(&self) -> T {
        self.raw_slot
            .read()
            .expect("failed to lock raw output message for read")
            .clone()
    }
}

impl<T> Output<T>
where
    T: SimulationMessage,
{
    pub fn write(&self, value: T) {
        *self
            .raw_slot
            .write()
            .expect("failed to lock raw output message for write") = value.clone();

        let effective = self.overrides.apply_installed(value).unwrap_or_else(|err| {
            eprintln!("[override] failed to apply override: {err}");
            self.read_upstream()
        });

        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = effective;
    }

    /// The value a `freeze` would capture, sampled before any rule lock is
    /// taken. `None` for every other mode, which ignores it.
    ///
    /// See [`OverrideCell::install`] for why this cannot be read inside it.
    fn freeze_source(&self, mode: Mode) -> Result<Option<Value>, serde_json::Error> {
        match mode {
            Mode::Freeze => serde_json::to_value(self.read()).map(Some),
            _ => Ok(None),
        }
    }

    /// The value this output would take if the override were installed.
    ///
    /// Installs nothing. Callers that enforce their own rules validate this
    /// result rather than the incoming fragment, so a `patch` is judged on what
    /// it accumulates to rather than on the keys it happens to mention.
    pub fn preview_override(&self, mode: Mode, value: Value) -> Result<T, serde_json::Error> {
        self.overrides
            .preview(mode, value, self.freeze_source(mode)?, self.read_upstream())
    }

    /// Installs an override, returning an id that identifies it.
    ///
    /// The candidate rule is applied to the current raw value first, so a rule
    /// that cannot produce a well-formed `T` is rejected and the previously
    /// installed rule is left untouched.
    pub fn set_override(&self, mode: Mode, value: Value) -> Result<RuleId, serde_json::Error> {
        let freeze_source = self.freeze_source(mode)?;
        let (id, effective) =
            self.overrides
                .install(mode, value, freeze_source, self.read_upstream())?;

        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = effective;
        Ok(id)
    }

    pub fn clear_override(&self) {
        self.overrides.clear();
        self.reapply();
    }

    /// Removes the one layer `id` identifies, leaving every other rule on
    /// this output in force.
    ///
    /// Returns whether that layer was there to remove.
    pub fn clear_override_by_id(&self, id: RuleId) -> bool {
        if self.overrides.clear_rule(id) {
            self.reapply();
            true
        } else {
            false
        }
    }

    /// Recomputes what consumers see from the raw value and the rules still
    /// installed.
    ///
    /// Removing one layer of a stack leaves the others in force, so the visible
    /// value cannot simply be reset to raw — that would drop every fault except
    /// the one being withdrawn.
    fn reapply(&self) {
        let raw = self.read_upstream();
        let effective = self
            .overrides
            .apply_installed(raw.clone())
            .unwrap_or_else(|err| {
                eprintln!("[override] failed to reapply overrides: {err}");
                raw
            });
        *self
            .slot
            .write()
            .expect("failed to lock output message for write") = effective;
    }
}

impl<T: Clone + Default> Default for Output<T> {
    fn default() -> Self {
        Self::new(T::default())
    }
}

#[derive(Clone, Debug)]
pub struct Input<T> {
    slot: Option<Arc<RwLock<T>>>,
    overrides: OverrideCell,
}

impl<T> Default for Input<T> {
    fn default() -> Self {
        Self {
            slot: None,
            overrides: OverrideCell::default(),
        }
    }
}

impl<T> Input<T> {
    pub fn is_connected(&self) -> bool {
        self.slot.is_some()
    }

    pub(crate) fn connect(&mut self, slot: Arc<RwLock<T>>) {
        self.slot = Some(slot);
    }

    /// The rules in force, innermost first. Empty when nothing is overridden.
    pub fn installed_overrides(&self) -> Vec<Rule> {
        self.overrides.snapshot()
    }

    pub fn is_overridden(&self) -> bool {
        !self.overrides.is_empty()
    }

    pub fn clear_override(&self) {
        self.overrides.clear();
    }

    /// Removes the one layer `id` identifies, leaving every other rule on
    /// this input in force.
    ///
    /// Returns whether that layer was there to remove.
    pub fn clear_override_by_id(&self, id: RuleId) -> bool {
        self.overrides.clear_rule(id)
    }
}

impl<T: Clone + Default> Input<T> {
    /// The producer's value, before this input's own override.
    ///
    /// This is not necessarily what the simulation produced: if the producer's
    /// output is itself overridden, this is the producer's *effective* value.
    pub fn read_upstream(&self) -> T {
        self.slot
            .as_ref()
            .map(|slot| {
                slot.read()
                    .expect("failed to lock input message for read")
                    .clone()
            })
            .unwrap_or_default()
    }
}

impl<T: SimulationMessage> Input<T> {
    /// The value this consumer sees: upstream, with this input's override applied.
    pub fn read(&self) -> T {
        let upstream = self.read_upstream();
        // Uncontended `RwLock` read of an empty `Vec` when nothing is injected.
        self.overrides
            .apply_installed(upstream.clone())
            .unwrap_or_else(|err| {
                eprintln!("[override] failed to apply input override: {err}");
                upstream
            })
    }

    /// The value a `freeze` would capture, sampled before any rule lock is
    /// taken. `None` for every other mode, which ignores it.
    fn freeze_source(&self, mode: Mode) -> Result<Option<Value>, serde_json::Error> {
        match mode {
            Mode::Freeze => serde_json::to_value(self.read()).map(Some),
            _ => Ok(None),
        }
    }

    /// The value this consumer would read if the override were installed.
    ///
    /// Installs nothing. See [`Output::preview_override`].
    pub fn preview_override(&self, mode: Mode, value: Value) -> Result<T, serde_json::Error> {
        self.overrides
            .preview(mode, value, self.freeze_source(mode)?, self.read_upstream())
    }

    /// Installs an override on this consumer's view alone.
    ///
    /// Unlike overriding the producer's output, this is invisible to every
    /// other consumer of the same message.
    pub fn set_override(&self, mode: Mode, value: Value) -> Result<RuleId, serde_json::Error> {
        let freeze_source = self.freeze_source(mode)?;
        let (id, _) = self
            .overrides
            .install(mode, value, freeze_source, self.read_upstream())?;
        Ok(id)
    }
}

#[cfg(test)]
mod tests;
