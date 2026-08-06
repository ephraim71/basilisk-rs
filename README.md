# basilisk-rs

A Rust translation of the [Basilisk astrodynamics framework](https://github.com/AVSLab/basilisk).

The project is under active development. Modules and scenarios are validated against Basilisk as they are ported.

## Library usage

Add the crate to a Rust project:

```bash
cargo add basilisk-rs@0.2.0
```

## Setup

Install Rust:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

Install `matplotlib` to render scenario plots:

```bash
python3 -m pip install matplotlib
```

## Fetching assets

Scenarios load ephemeris, planetary orientation, and gravity data that are not
checked into the repository. Download them into `assets/` before running any
scenario:

```bash
just fetch-assets
```

This is a convenience wrapper around two recipes:

- `just fetch-anise-assets` — SPICE/ANISE kernels into `assets/anise/`
  (`de440s.bsp`, `earth_latest_high_prec.bpc`, `pck11.pca`).
- `just fetch-gravity` — the GGM03S spherical-harmonic gravity model into
  `assets/gravity/GGM03S.txt`.

The recipes only run `curl`, so if you don't have [`just`](https://github.com/casey/just)
installed you can fetch each file by hand instead:

```bash
# SPICE/ANISE kernels
mkdir -p assets/anise
curl --fail --location --progress-bar --output assets/anise/de440s.bsp \
  https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp
curl --fail --location --progress-bar --output assets/anise/earth_latest_high_prec.bpc \
  https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc
curl --fail --location --progress-bar --output assets/anise/pck11.pca \
  https://public-data.nyxspace.com/anise/v0.4/pck11.pca

# Gravity model
mkdir -p assets/gravity
curl --fail --location --progress-bar --output assets/gravity/GGM03S.txt \
  https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData/LocalGravData/GGM03S.txt
```

If the `pck11.pca` download fails on a TLS certificate error, retry that one
command with `--insecure` (the `just` recipe falls back to this automatically).

## Basic orbit scenario

Run the simulation:

```bash
just run-basic-orbit
# equivalently:
cargo run --release --example scenario_basic_orbit
```

Any example can also be run by name with the generic `run` recipe, which
forwards extra arguments to the binary after `--`. The name is the file stem of
a `.rs` file in `examples/` (e.g. `scenario_basic_orbit` for
`examples/scenario_basic_orbit.rs`); Cargo builds each such file with a `main()`
as its own example target.

```bash
just run scenario_basic_orbit
# equivalently:
cargo run --release --example scenario_basic_orbit
```

The scenario writes `examples/output/scenarios/scenarioBasicOrbitLEO0Earth.csv` by default.

Open the same three figures as Basilisk's point-mass LEO example:

```bash
python3 examples/plot_scenario_basic_orbit.py
```

The plotting script reads only the generated CSV and opens the figures on screen.

## Regression comparison

The comparison workflow is separate from simulation and plotting. It runs all five
ported scenarios and checks their generated CSV files against a supplied reference
archive:

```bash
BASILISK_REFERENCE_CSV_ZIP=/path/to/csvs.zip just verify-scenarios
```

## Contributing

Contributions, reviews, and issues are welcome.
