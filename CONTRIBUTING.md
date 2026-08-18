# Contributing

A Rust astrodynamics framework, currently being built out module by module. The
initial modules are translated from [Basilisk](https://github.com/AVSLab/basilisk)
and validated against it, which is how correctness gets established while the
foundations go in.

Early days, so nothing here is set in stone. Open an issue before starting anything
big and we'll figure it out.

## What background helps

Rust, no `unsafe` or async. Beyond that, depends what you touch: orbital mechanics
for `dynamics/` and `environment/`, ADCS and MRPs for `fsw_algorithms/`, numerical
integration for `integrators/`. Python only in the plotting scripts.

For a module that isn't in yet, most of the work is reading the upstream C++ and
knowing the domain well enough to tell a real difference from a floating-point one.

## Getting going

```bash
just fetch-assets      # ephemeris + gravity data, some tests need it
cargo test
```

You don't need Basilisk installed to work on this.

`src/` is organized by module family. Two things that differ from upstream and tend
to surprise people: modules are wired explicitly through `Output<T>`/`Input<T>`
slots in scenario code rather than a name-keyed message bus, and the scheduler runs
on integer nanoseconds with each module ticking only on its own period. Both are
deliberate, and the type-checked wiring is one of the things Rust buys us here.

Good places to start, all with a clear reference implementation to work from: `examples/`

## Tests and correctness

Unit tests go in `mod tests` in the same file. The usual pattern is to reimplement
the reference algorithm in the test and assert the module agrees across a sweep of
inputs. `upstream_truth` in `src/fsw_algorithms/mrp_feedback.rs` shows the idea. The
duplication is deliberate.
