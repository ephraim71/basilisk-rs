# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

Slated for 0.4.0. **Everything under "Changed" and "Removed" is breaking**, and
those two sections are deliberately short: they list only what a 0.3.0 user can
actually have written against. Nothing has a deprecated alias.

### Added

- `overrides` module: a registry of everything a fault can be injected into,
  addressed by name. `Registry` holds them; `Target` is the one type each one
  is, whatever it wraps; `Overridable` lets a module list its own ports in one
  call. `Registry::register` is the single way in: it takes any `messages::Port`
  and the `TargetKind` to file it under, so one path serves outputs, inputs,
  configurations and whatever an application names for itself. The kind is
  always given rather than inferred from which method was called.
  `Target` is a struct with no public constructor rather than a trait, so
  registering a port is the only way to produce one and no target can exist
  that skips the name and shape checks `Target::install` applies.
- `messages::Port`, the override surface `Output<T>` and `Input<T>` share. The
  two have identical operations, and without a trait saying so anything wanting
  to accept either had to be written twice.
- Schemas derived at runtime from `T::default()`, so a field added to a message
  type is advertised without anything being hand-maintained: `TargetSpec`,
  `FieldSpec`, `TargetKind`.
- `TargetKind::Custom(&'static str)`, so an application can name a kind this
  crate has no opinion about and have it reach a client verbatim.
- Three modes, differing only in where the value comes from: `Replace` takes it
  from the payload, `Default` from the type's own default, `Freeze` from the
  live value at the moment it is installed.
- `Mode::Replace` takes an object mapping RFC 6901 pointers to the values to put
  there — `{"/omega_radps/1": 0.5}` — so one element of a vector is addressable
  without pinning its siblings to whatever the sender happened to write. Nested
  JSON mirroring the message is not a payload shape; there is only this one.
- `Freeze` and `Default` take an array of the same pointers, so one axis of a
  body rate can be held or reset without touching the others. An empty payload
  means the whole message.
- Every rule is relative: a rule touches the paths it names and leaves the rest
  to the layers beneath, so a stack is always the composition of all of it.
- RFC 6901 pointers are the single path syntax across the feature. `FieldSpec`
  publishes them, a `replace` assigns to them, and a `freeze` names them, so a
  path read off a target's schema is pasted into any of the three modes
  unchanged and nothing translates between two spellings. The whole message is
  the root pointer, `""`, rather than a special case.
- A payload is parsed once into a type that cannot hold an invalid combination:
  `Request`, `Document`, `Selection`, `Assignment` and `Pointer`. `Rule` has
  private fields and one constructor, which requires the values a freeze
  captures rather than taking them as an option that can be omitted.
- A rule that cannot apply to a particular value is skipped for that value
  rather than cancelling the rest of the stack — an `Option` going `None` under
  a pointer is a rule with nothing to do this tick, not a reason to publish the
  raw upstream value with every fault discarded.
- Overriding an `Input` at all. 0.3.0's `Input<T>` held only a slot, so a fault
  could be applied to a message but never to one consumer's view of it:
  `Input::set_override`, `clear_override`, `clear_override_by_id`,
  `installed_overrides`, `preview_override`, `is_overridden`.
- Rules now stack per port and each is removable on its own, where 0.3.0 held a
  single optional rule: `Rule`, `RuleId`, `Output::clear_override_by_id`,
  `Output::installed_overrides`, `Output::preview_override`,
  `Output::is_overridden`.
- `Output::read_upstream` and `Input::read_upstream`: the value before that port's
  own rules, so a caller can see what the simulation produced alongside what a
  consumer now sees.
- An override payload is refused if it names a field the type does not have, with
  a nearest-match suggestion. Serde ignores unknown fields, so without this a
  misspelled name reported success and changed nothing. The names are checked
  against the message the payload *produces*, not against `T::default()`, so a
  field inside a populated `Option` is addressable even though the default has
  that option as `None` and cannot advertise its children.
- A payload naming a field the type does not have is refused before it is
  installed, and a pointer that does not resolve is refused with it, so no rule
  is installed that would quietly do nothing.

### Changed

Six items, and these are the only ones that can affect code written against
0.3.0. The override surface was reworked substantially before release, but the
rest of that rework renamed items that 0.3.0 either kept private or never had,
so it is not listed here — a changelog that reports internal churn as breaking
makes it harder to find the changes that are.

- `Output::set_override` takes one `overrides::Request` where 0.3.0 took a
  `(MessageOverrideMode, Value)` pair, so every 0.3.0 call site changes:

  ```rust
  // 0.3.0
  output.set_override(MessageOverrideMode::Patch, json!({ "rate_dps": 9.0 }))?;
  // now
  output.set_override(Request::replace(json!({ "/rate_dps": 9.0 }))?)?;
  ```

  A `Request` is parsed once, at the boundary, and cannot pair a mode with a
  payload that mode has no meaning for — which is what two loose arguments
  allowed and what the freeze payload being ignored entirely was a case of. The
  wire form is unchanged: `Request` is adjacently tagged, so it still reads and
  writes `{"mode": ..., "value": ...}`.
- `MessageOverrideMode::Patch` is absorbed into `Request::replace`, and both
  0.3.0 payload shapes have to be rewritten: keys are RFC 6901 pointers now, not
  nested JSON mirroring the message. `{"gain": {"value": 1.0}}` becomes
  `{"/gain/value": 1.0}`, and a whole-message `Replace` becomes `{"": {...}}`.
  The behaviour change worth reading twice is that `Replace` used to be refused
  unless it named every field and no longer is, so a `Replace` that omits a
  field now leaves that field to the layers beneath it rather than being
  rejected. Nothing enforces a complete message any more.
- `messages::MessageOverrideMode` is now `overrides::Mode`, renamed *and* moved
  to a new module — and demoted: it is a label reported on an installed `Rule`
  rather than something a caller passes.
- `Output::set_override` returns `Result<RuleId, serde_json::Error>` rather than
  `Result<(), _>`. The id is what makes a single rule removable later, which a
  timed fault needs: without it, a timer firing after a second override was
  applied to the same port clears the wrong one.
- `Input::<T>::read` now requires `T: SimulationMessage` rather than
  `T: Clone + Default`. `SimulationMessage` adds `Serialize + DeserializeOwned +
  Send + Sync + 'static`, so a message type that is cloneable and defaultable but
  not serde-able can no longer be read through an `Input`. Reading an input now
  folds any rule installed on that consumer's view, and folding is defined in
  terms of JSON.
- Cloning an `Input<T>` now aliases its connection instead of copying it, so a
  clone follows a later `connect`. `vec![Input::default(); n]` therefore yields
  one input under `n` names, where 0.3.0 let each entry be wired to a different
  producer; build a collection with `(0..n).map(|_| Input::default()).collect()`
  instead. The rule stack was already shared this way, so such a collection also
  already shared one set of overrides — the two halves now agree, and `Output`
  has always aliased on clone.

### Removed

- `messages::MessageOverrideMode`, as the move above describes. No deprecated
  re-export: `messages` cannot name it without depending on the module that now
  owns it.
- Its `Patch` variant has no counterpart in `overrides::Mode`, which has three.
  See the absorption into `Request::replace` above.

## [0.3.0] - 2026-08-06

Released without a changelog entry; recorded here after the fact.

### Changed

- Actuator effectors take per-device command ports rather than one array-valued
  port, so a device can be commanded, and overridden, on its own. Affects
  `MtbDynamicEffector`, `ReactionWheelStateEffector`, `MotorVoltageInterface`
  and `MtbMomentumManagement`.

### Added

- `MagneticDipoleCommandMsg` and `MotorTorqueMsg`.

## [0.2.0] - 2026-08-06

### Added

- Runtime message overrides with patch, replace, freeze, and default modes.
- Real-time simulation pacing through `ClockSync`.
- Interruptible chunked simulation stepping.
- Serde support for simulation messages, including fixed-size effector arrays.

### Changed

- `Output` now requires messages to implement `SimulationMessage` so values can
  be serialized for runtime overrides.

### Fixed

- Reject invalid message overrides without replacing the last valid output or
  override rule.
- Initialize `ClockSync` without recording a false first-update overrun.

## [0.1.0] - 2026-07-28

- Initial crates.io release.

[Unreleased]: https://github.com/ephraim71/basilisk-rs/compare/v0.3.0...HEAD
[0.3.0]: https://github.com/ephraim71/basilisk-rs/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/ephraim71/basilisk-rs/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/ephraim71/basilisk-rs/releases/tag/v0.1.0
