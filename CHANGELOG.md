# Changelog

All notable changes to this project are documented in this file.

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

[0.2.0]: https://github.com/ephraim71/basilisk-rs/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/ephraim71/basilisk-rs/releases/tag/v0.1.0
