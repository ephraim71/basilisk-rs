# basilisk-rs

A Rust translation of the [Basilisk astrodynamics framework](https://github.com/AVSLab/basilisk).

The project is under active development. Modules and scenarios are validated against Basilisk as they are ported.

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

## Basic orbit scenario

Run the simulation:

```bash
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
