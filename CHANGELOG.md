# Changelog

All notable changes to this project are documented in this file.

## [Unreleased]

Slated for 0.4.0. **Everything under "Changed" and "Removed" is breaking**, and
those two sections are deliberately short: they list only what a 0.3.0 user can
actually have written against. Nothing has a deprecated alias.

### Added

- `overrides` module: a registry of everything a fault can be injected into,
  addressed by name. `Registry` holds them; `Target` is the trait each one
  implements; `Overridable` lets a module list its own ports in one call.
- Schemas derived at runtime from `T::default()`, so a field added to a message
  type is advertised without anything being hand-maintained: `TargetSpec`,
  `FieldSpec`, `TargetKind`.
- `TargetKind::Custom(&'static str)`, so an application can name a kind this
  crate has no opinion about and have it reach a client verbatim.
- `Mode::PointerReplace`, a mode 0.3.0 did not have: the `replace` operation of
  RFC 6902 and no other, which unlike `Mode::Patch` can address a single element
  of an array. A merge replaces an array wholesale, so `Patch` cannot change one
  component of a vector without pinning its siblings to whatever the sender
  happened to write.
- `Mode::is_relative`, so a client can describe a mode without re-deriving
  whether it composes with the rule beneath it or masks it.
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
- A `replace` must name every field of its type. A partial one is refused rather
  than silently resetting the fields it omits to the type default.

### Changed

Three items, and these are the only ones that can affect code written against
0.3.0. The override surface was reworked substantially before release, but the
rest of that rework renamed items that 0.3.0 either kept private or never had,
so it is not listed here — a changelog that reports internal churn as breaking
makes it harder to find the changes that are.

- `messages::MessageOverrideMode` is now `overrides::Mode`: renamed *and* moved
  to a new module. It appeared in every call to `Output::set_override`, so this
  is the one a 0.3.0 user is most likely to hit.
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

### Removed

- `messages::MessageOverrideMode`, as the move above describes. No deprecated
  re-export: `messages` cannot name it without depending on the module that now
  owns it.

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
