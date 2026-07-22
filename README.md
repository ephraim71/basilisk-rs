# basilisk-rs : A Rust port of Basilisk astrodynamics framework


This library is in active development.        
All existing modules and examples are validated against [Basilisk](https://github.com/avslab/basilisk).           
    
## Reasons for porting : 
- Memory safety throughout the full simulation and flight software framework, eliminitaing a whole class of bugs. 
- Small build size (<10MB), making it straightforward to run in the target directly. 
- Faster run times
- Faster build times
- Easily supports multiple recorder backends
- Reuse fsw modules in onboard Rust based software architectures.  
- Rust comes with great error messages, enabling fast iteration by coding agents.   


## How to setup

Install Rust

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

Clone

```bash
cd basilisk-rs
```

First-Time Setup

Run this once after cloning to download the required assets:

```bash
just fetch-assets
```

This step requires network access.

Build

```bash
cargo build --release --example full_satellite_stack
```

Run Example

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

Profile Simulation

```bash
SHOW_PROGRESS=1 PROFILE_SIM=1 cargo run --release --example full_satellite_stack
```

## Future work

- Modules ported   
- Examples ported   
- Basilisk style tasks and processes do not exist - all modules are in the same process/task.   
- Python bindings   
- More backend recorders   
- Vizard support

## Contributing
This repo is in active development, contributions, reviews and issues are most welcome!  

