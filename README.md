# basilisk-rs

A Rust translation of the [Basilisk astrodynamics framework](https://github.com/AVSLab/basilisk).

The project is under active development. Modules and scenarios are validated against Basilisk as they are ported.

## Library usage

Add the crate to a Rust project:

```bash
cargo add basilisk-rs@0.3.0
```

## Setup

Install Rust:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

Install `matplotlib` and `numpy` to render scenario plots:

```bash
python3 -m pip install matplotlib numpy
```

The recipes below are in the `justfile`; install [`just`](https://github.com/casey/just)
with `cargo install just`, or copy the commands out of it.

## Fetching assets

Scenarios load ephemeris, planetary orientation, and gravity data that are not
checked into the repository. Download them into `assets/` before running any
scenario:

```bash
just fetch-assets
```

This is a convenience wrapper around four recipes:

- `just fetch-anise-assets` — SPICE/ANISE kernels into `assets/anise/`
  (`de440s.bsp`, `earth_latest_high_prec.bpc`, `pck11.pca`).
- `just fetch-gravity` — the GGM03S spherical-harmonic gravity model into
  `assets/gravity/GGM03S.txt`.
- `just fetch-albedo` — Basilisk's albedo lookup tables (Earth CERES 2018
  all-sky and clear-sky, Mars TES; 1x1, 5x5 and 10x10 deg) into `assets/albedo/`.
- `just fetch-environment-data` — upstream's Denton GEO plasma flux tables and
  tabulated atmospheres into `assets/denton/` and `assets/atmosphere/`.

Some example scenarios read more of upstream's `supportData` and
`dataForExamples`. Each run recipe's comment in the `justfile` names the
recipes it needs:

- `just fetch-spice-kernels` — the CSPICE text kernels, the Hubble SPK and
  DE430 (114 MB) into `assets/spice/`, for the `spice` feature.
- `just fetch-orbit-scenario-data` — the J2-only Earth and Mars fields, the
  GGM2B Mars field and the New Horizons SPK.
- `just fetch-small-body-data` — the Vesta field and the Eros shape model.
- `just fetch-misc-scenario-data` — the custom spacecraft SPK and the state
  files of `scenarioDataToViz`.
- `just fetch-opencv-assets` — the image-processing test images and the
  `centerRadiusCNN` network (`CAD.onnx`, 26 MB), for the `opencv` feature.
- `just fetch-mujoco-scenario-data` — the Itokawa mesh of
  `scenario_asteroid_landing`.

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

# Albedo lookup tables (repeat for each file listed above)
mkdir -p assets/albedo
curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_All_5x5.csv \
  https://raw.githubusercontent.com/AVSLab/basilisk/ea95ec4efe6d3e842a26546de653b7505dd054c3/supportData/AlbedoData/Earth_ALB_2018_CERES_All_5x5.csv
```

If the `pck11.pca` download fails on a TLS certificate error, retry that one
command with `--insecure` (the `just` recipe falls back to this automatically).

## Optional features

The modules that link native libraries are behind cargo features, so a
default build needs only Rust:

| Feature | Enables | Needs |
|---|---|---|
| `spice` | `environment::spice_interface`, upstream's `spiceInterface` on NAIF CSPICE | libclang; the build downloads the CSPICE package from naif.jpl.nasa.gov unless `CSPICE_DIR` points at an install; kernels from `just fetch-spice-kernels` |
| `mujoco` | `mujoco_dynamics`, upstream's `MJScene` and the models that run in it | nothing system-wide: the first build downloads MuJoCo 3.17.0 (upstream's version), checks its SHA-256 and unpacks it into `assets/mujoco/` (`MUJOCO_DOWNLOAD_DIR` in `.cargo/config.toml`) |
| `vizard` | `vizard::VizInterface` and `vizard::DataFileToViz` | `protoc` for the protobuf messages and libzmq for live streaming |
| `opencv` | the image-processing modules (`hough_circles`, `limb_finding`, `center_radius_cnn`) and `sensors::camera` | the system OpenCV 4 or 5 (found through pkg-config) and libclang; test images from `just fetch-opencv-assets` |

```bash
just fetch-anise-assets fetch-spice-kernels
cargo test --features spice
just fetch-gravity   # GGM03S, for the spherical-harmonics gravity test
cargo test --features mujoco mujoco_dynamics
```

The test and example binaries find the downloaded MuJoCo library through
their runtime search path. To use a MuJoCo 3.17.0 you already have instead, point `MUJOCO_DYNAMIC_LINK_DIR` at
its `lib/` directory (an absolute path) and put that directory on
`LD_LIBRARY_PATH` when running. Examples that need a feature declare it as
`required-features` in `Cargo.toml`, so a default `cargo build --examples`
skips them.

## Example scenarios

`examples/` holds the ports of upstream's example scenarios. Each one writes
its recorded histories as CSV to `examples/output/scenarios/`, under the
upstream scenario's name, and most have a `plot_<name>.py` script that draws
upstream's figures from that CSV. Run one by its file stem, forwarding
arguments after `--`:

```bash
just run scenario_basic_orbit
# equivalently:
cargo run --release --example scenario_basic_orbit
cargo run --release --example scenario_basic_orbit -- --help   # its parametrizations
python3 examples/plot_scenario_basic_orbit.py
```

The scenario writes `examples/output/scenarios/scenarioBasicOrbitLEO0Earth.csv`
by default, and the plot opens the same three figures as Basilisk's point-mass
LEO example.

Scenarios run by group, each in every parametrization upstream's
`test_scenario*.py` exercises, and their figures render from the CSV files
alone:

```bash
just run-orbit-scenarios
just plot-orbit-scenarios
just run-all-scenarios   # every group; needs all four features and every fetch recipe
```

| Group | Scenarios |
|---|---|
| `orbit` | orbits and gravity fields |
| `traj` | trajectories, integrators and drag |
| `formation` | formation flying, rendezvous, small bodies and attitude guidance |
| `ground` | ground access, onboard data, power and multiple spacecraft |
| `multibody-a` | hinged, flexible, slosh, spinning-body, prescribed-motion and branching bodies |
| `multibody-b` | constrained dynamics, prescribed motion and robotic arms |
| `misc` | SPICE, Vizard, Monte Carlo, logging, constrained attitude and benchmarks |
| `bsksim` | upstream's BskSim framework |
| `multisat` | upstream's MultiSatBskSim framework |
| `montecarlo` | upstream's `MonteCarloExamples` |
| `opnav` | upstream's `OpNavScenarios` (`opencv`) |
| `dyncomp` | upstream's `dynamicsComparison` series (`mujoco` for the MuJoCo side) |
| `mujoco` | upstream's `examples/mujoco` (`mujoco`) |

`run-<group>-scenarios` runs a group and `plot-<group>-scenarios` renders it;
the attitude, sensor and momentum-management scenarios are in
`run-all-scenarios` itself. A recipe's comment lists the features and assets
its group needs.

Each example also carries upstream's `test_scenario*.py` checks, with the
same truth values and tolerances, as tests:

```bash
cargo test --release --examples                       # every default-feature example
cargo test --release --examples --features mujoco     # plus the MuJoCo ones
cargo test --release --example scenario_basic_orbit   # one example
```

## Regression comparison

The comparison workflow is separate from simulation and plotting. It runs all five
ported scenarios and checks their generated CSV files against a supplied reference
archive:

```bash
BASILISK_REFERENCE_CSV_ZIP=/path/to/csvs.zip just verify-scenarios
```

## Contributing

Contributions, reviews, and issues are welcome. See
[CONTRIBUTING.md](CONTRIBUTING.md) for what background helps, how the crate is laid
out, and where to start.
