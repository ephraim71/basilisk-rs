set shell := ["bash", "-euo", "pipefail", "-c"]

de440s_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp"
earth_bpc_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
pck11_url := "https://public-data.nyxspace.com/anise/v0.4/pck11.pca"
ggm03s_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData/LocalGravData/GGM03S.txt"
reference_csv_zip := env_var_or_default("BASILISK_REFERENCE_CSV_ZIP", "/home/ephraim/hamming/csvs.zip")

default:
    @just --list

fetch-assets: fetch-anise-assets fetch-gravity

fetch-anise-assets:
    mkdir -p assets/anise
    curl --fail --location --progress-bar --output assets/anise/de440s.bsp {{de440s_url}}
    curl --fail --location --progress-bar --output assets/anise/earth_latest_high_prec.bpc {{earth_bpc_url}}
    curl --fail --location --progress-bar --output assets/anise/pck11.pca {{pck11_url}} || curl --fail --location --progress-bar --insecure --output assets/anise/pck11.pca {{pck11_url}}

fetch-gravity:
    mkdir -p assets/gravity
    curl --fail --location --progress-bar --output assets/gravity/GGM03S.txt {{ggm03s_url}}

# Run any example by name, forwarding extra args to the binary
# (e.g. `just run scenario_basic_orbit` or `just run scenario_css -- --help`).
run example *args:
    cargo run --release --example {{example}} {{args}}


# Run the five deterministic scenarios and write their CSV files.
run-scenarios:
    cargo run --release --example scenario_basic_orbit
    cargo run --release --example scenario_drag_deorbit
    cargo run --release --example scenario_attitude_feedback_rw
    cargo run --release --example scenario_css
    cargo run --release --example scenario_mtb_momentum_management

# Compare existing Rust outputs without rerunning the simulations.
compare-scenarios reference_zip=reference_csv_zip generated_dir="examples/output/scenarios":
    python3 examples/compare_scenario_csvs.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"

# Run all five scenarios, then enforce the reference parity gates.
verify-scenarios reference_zip=reference_csv_zip generated_dir="examples/output/scenarios": run-scenarios
    python3 examples/compare_scenario_csvs.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"

# Render all five scenarios from generated CSV files only.
plot-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_basic_orbit.py --generated-dir "{{generated_dir}}" --save --no-show
    python3 examples/plot_scenario_drag_deorbit.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_attitude_feedback_rw.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_css.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_mtb_momentum_management.py --generated-dir "{{generated_dir}}"
