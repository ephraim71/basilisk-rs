set shell := ["bash", "-euo", "pipefail", "-c"]

de440s_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp"
earth_bpc_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
pck11_url := "https://public-data.nyxspace.com/anise/v0.4/pck11.pca"
ggm03s_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData/LocalGravData/GGM03S.txt"
reference_csv_zip := env_var_or_default("BASILISK_REFERENCE_CSV_ZIP", "/home/ephraim/hamming/csvs.zip")

default:
    @just --list

fetch-assets: fetch-anise-assets fetch-gravity

run-full-stack-profile-samply: init-samply run-samply

init-samply:
    cargo install samply
    cargo install cargo-samply

fetch-anise-assets:
    mkdir -p assets/anise
    curl --fail --location --progress-bar --output assets/anise/de440s.bsp {{de440s_url}}
    curl --fail --location --progress-bar --output assets/anise/earth_latest_high_prec.bpc {{earth_bpc_url}}
    curl --fail --location --progress-bar --output assets/anise/pck11.pca {{pck11_url}} || curl --fail --location --progress-bar --insecure --output assets/anise/pck11.pca {{pck11_url}}

fetch-gravity:
    mkdir -p assets/gravity
    curl --fail --location --progress-bar --output assets/gravity/GGM03S.txt {{ggm03s_url}}

run-full-stack-profile:
    @SIM_SECS=300; START=$(date +%s.%N); SHOW_PROGRESS=1 PROFILE_SIM=1 cargo run --release --example full_satellite_stack; END=$(date +%s.%N); python3 -c "import sys; sim=float(sys.argv[1]); start=float(sys.argv[2]); end=float(sys.argv[3]); wall=end-start; print(); print('wall_clock_s={:.3f}'.format(wall)); print('sim_speedup={:.3f}x'.format(sim / wall))" "$SIM_SECS" "$START" "$END"

run-samply:
    @SIM_SECS=300; START=$(date +%s.%N); SHOW_PROGRESS=1 PROFILE_SIM=1 cargo samply --example full_satellite_stack; END=$(date +%s.%N); python3 -c "import sys; sim=float(sys.argv[1]); start=float(sys.argv[2]); end=float(sys.argv[3]); wall=end-start; print(); print('wall_clock_s={:.3f}'.format(wall)); print('sim_speedup={:.3f}x'.format(sim / wall))" "$SIM_SECS" "$START" "$END"

run-sun-pointing-profile:
    @SIM_SECS=600; START=$(date +%s.%N); SHOW_PROGRESS=1 PROFILE_SIM=1 cargo run --release --example sun_pointing; END=$(date +%s.%N); python3 -c "import sys; sim=float(sys.argv[1]); start=float(sys.argv[2]); end=float(sys.argv[3]); wall=end-start; print(); print('wall_clock_s={:.3f}'.format(wall)); print('sim_speedup={:.3f}x'.format(sim / wall))" "$SIM_SECS" "$START" "$END"

# Run the five deterministic Basilisk C++ parity scenarios.
run-reference-scenarios:
    cargo run --release --example scenario_basic_orbit
    cargo run --release --example scenario_drag_deorbit
    cargo run --release --example scenario_attitude_feedback_rw
    cargo run --release --example scenario_css
    cargo run --release --example scenario_mtb_momentum_management

# Compare existing Rust outputs without rerunning the simulations.
compare-scenarios reference_zip=reference_csv_zip generated_dir="examples/output/scenarios":
    python3 examples/compare_scenario_csvs.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"

# Run all five scenarios, then enforce the C++ parity gates.
verify-scenarios reference_zip=reference_csv_zip generated_dir="examples/output/scenarios": run-reference-scenarios
    python3 examples/compare_scenario_csvs.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"

# Render all five Rust-versus-C++ comparison figures.
plot-scenarios reference_zip=reference_csv_zip generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_basic_orbit.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_drag_deorbit.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_attitude_feedback_rw.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_css.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_mtb_momentum_management.py --reference-zip "{{reference_zip}}" --generated-dir "{{generated_dir}}"
