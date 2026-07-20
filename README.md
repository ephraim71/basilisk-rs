# basilisk-rs

THIS LIBRARY IS CURRENTLY IN DEVELOPMENT AND IS NOT MEANT TO BE USED IN PRODUCTION. 
    
A Rust port of the [Basilisk](https://github.com/AVSLab/basilisk) astrodynamics framework.          
The goal is to port both simulation and fsw modules to Rust.              
Python bindings will be supported in a future version.          

## Reasons for porting : 
- Memory safety throughout the full simulation framework, eliminitaing a whole class of bugs. 
- Rust comes with great error messages. 
- Small build size (<10MB), making it straightforward to run in the target directly. 
- Faster run times
- Faster build times
- Easily supports multiple recorder backends


## Install Rust

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source "$HOME/.cargo/env"
```

Install `just`:

```bash
cargo install just
```

If you want to use the plotting script, install Python and `matplotlib`:

```bash
python3 -m pip install matplotlib
```

## Clone

```bash
git clone git@github.com:ephraim71/basilisk-rs.git
cd basilisk-rs
```

## First-Time Setup

Run this once after cloning to download the required assets:

```bash
just fetch-assets
```

This step requires network access.

## Build

```bash
cargo build --release --example full_satellite_stack
```

## Run Example

Run without recorders:

```bash
SHOW_PROGRESS=1 cargo run --release --example full_satellite_stack
```

Run with CSV recorders enabled:

```bash
SHOW_PROGRESS=1 ENABLE_RECORDING=1 cargo run --release --example full_satellite_stack
```

CSV recorders are currently not optimized and are slow. More recorder options will be added later.

Recorded CSV output is written to `examples/output/full_satellite_stack/`.

If CSV recording is enabled, you can plot the output with:

```bash
python3 examples/plot_full_satellite_stack.py
```

Or point it at a specific output directory:

```bash
python3 examples/plot_full_satellite_stack.py examples/output/full_satellite_stack
```

## Profile Simulation

```bash
SHOW_PROGRESS=1 PROFILE_SIM=1 cargo run --release --example full_satellite_stack
```

## Scheduler Model

Modules with higher numeric priority execute first. Equal-priority modules retain insertion order. There is no Basilisk-style concept of processes or tasks in this library.
