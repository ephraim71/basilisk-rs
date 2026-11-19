set shell := ["bash", "-euo", "pipefail", "-c"]

de440s_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp"
earth_bpc_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
pck11_url := "https://public-data.nyxspace.com/anise/v0.4/pck11.pca"
ggm03s_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData/LocalGravData/GGM03S.txt"
support_data_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData"
naif_generic_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels"
hst_spk_url := "https://naif.jpl.nasa.gov/pub/naif/HST/kernels/spk/hst_edited.bsp"
reference_csv_zip := env_var_or_default("BASILISK_REFERENCE_CSV_ZIP", "/home/ephraim/hamming/csvs.zip")

default:
    @just --list

fetch-assets: fetch-anise-assets fetch-gravity fetch-environment-data

fetch-anise-assets:
    mkdir -p assets/anise
    curl --fail --location --progress-bar --output assets/anise/de440s.bsp {{de440s_url}}
    curl --fail --location --progress-bar --output assets/anise/earth_latest_high_prec.bpc {{earth_bpc_url}}
    curl --fail --location --progress-bar --output assets/anise/pck11.pca {{pck11_url}} || curl --fail --location --progress-bar --insecure --output assets/anise/pck11.pca {{pck11_url}}

# Upstream `supportData/EphemerisData` kernels for the `spice` feature's
# `spiceInterface`: the text kernels its default set reads next to
# `assets/anise/de440s.bsp`, the Hubble SPK, and DE430 (114 MB), against which
# upstream's JPL Horizons truth was made.
fetch-spice-kernels:
    mkdir -p assets/spice
    curl --fail --location --progress-bar --output assets/spice/naif0012.tls {{naif_generic_url}}/lsk/naif0012.tls
    curl --fail --location --progress-bar --output assets/spice/pck00010.tpc {{naif_generic_url}}/pck/pck00010.tpc
    curl --fail --location --progress-bar --output assets/spice/de-403-masses.tpc {{naif_generic_url}}/pck/de-403-masses.tpc
    curl --fail --location --progress-bar --output assets/spice/hst_edited.bsp {{hst_spk_url}}
    curl --fail --location --progress-bar --output assets/spice/de430.bsp {{naif_generic_url}}/spk/planets/de430.bsp

fetch-gravity:
    mkdir -p assets/gravity
    curl --fail --location --progress-bar --output assets/gravity/GGM03S.txt {{ggm03s_url}}

# Albedo maps, Denton GEO plasma flux tables and tabulated atmospheres from
# upstream supportData, for the albedo, Earth radiation, Denton flux and
# tabular atmosphere modules.
fetch-environment-data:
    mkdir -p assets/albedo assets/denton assets/atmosphere
    curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_All_10x10.csv {{support_data_url}}/AlbedoData/Earth_ALB_2018_CERES_All_10x10.csv
    curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_All_5x5.csv {{support_data_url}}/AlbedoData/Earth_ALB_2018_CERES_All_5x5.csv
    curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_All_1x1.csv {{support_data_url}}/AlbedoData/Earth_ALB_2018_CERES_All_1x1.csv
    curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_Clear_10x10.csv {{support_data_url}}/AlbedoData/Earth_ALB_2018_CERES_Clear_10x10.csv
    curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_Clear_5x5.csv {{support_data_url}}/AlbedoData/Earth_ALB_2018_CERES_Clear_5x5.csv
    curl --fail --location --progress-bar --output assets/albedo/Earth_ALB_2018_CERES_Clear_1x1.csv {{support_data_url}}/AlbedoData/Earth_ALB_2018_CERES_Clear_1x1.csv
    curl --fail --location --progress-bar --output assets/albedo/Mars_ALB_TES_10x10.csv {{support_data_url}}/AlbedoData/Mars_ALB_TES_10x10.csv
    curl --fail --location --progress-bar --output assets/albedo/Mars_ALB_TES_5x5.csv {{support_data_url}}/AlbedoData/Mars_ALB_TES_5x5.csv
    curl --fail --location --progress-bar --output assets/albedo/Mars_ALB_TES_1x1.csv {{support_data_url}}/AlbedoData/Mars_ALB_TES_1x1.csv
    curl --fail --location --progress-bar --output assets/denton/model_e_array_all.txt {{support_data_url}}/DentonGEO/model_e_array_all.txt
    curl --fail --location --progress-bar --output assets/denton/model_e_array_low.txt {{support_data_url}}/DentonGEO/model_e_array_low.txt
    curl --fail --location --progress-bar --output assets/denton/model_e_array_mid.txt {{support_data_url}}/DentonGEO/model_e_array_mid.txt
    curl --fail --location --progress-bar --output assets/denton/model_e_array_high.txt {{support_data_url}}/DentonGEO/model_e_array_high.txt
    curl --fail --location --progress-bar --output assets/denton/model_i_array_all.txt {{support_data_url}}/DentonGEO/model_i_array_all.txt
    curl --fail --location --progress-bar --output assets/denton/model_i_array_low.txt {{support_data_url}}/DentonGEO/model_i_array_low.txt
    curl --fail --location --progress-bar --output assets/denton/model_i_array_mid.txt {{support_data_url}}/DentonGEO/model_i_array_mid.txt
    curl --fail --location --progress-bar --output assets/denton/model_i_array_high.txt {{support_data_url}}/DentonGEO/model_i_array_high.txt
    curl --fail --location --progress-bar --output assets/atmosphere/EarthGRAMNominal.txt {{support_data_url}}/AtmosphereData/EarthGRAMNominal.txt
    curl --fail --location --progress-bar --output assets/atmosphere/MarsGRAMNominal.txt {{support_data_url}}/AtmosphereData/MarsGRAMNominal.txt
    curl --fail --location --progress-bar --output assets/atmosphere/NRLMSISE00Nominal.txt {{support_data_url}}/AtmosphereData/NRLMSISE00Nominal.txt
    curl --fail --location --progress-bar --output assets/atmosphere/USStandardAtmosphere1976.csv {{support_data_url}}/AtmosphereData/USStandardAtmosphere1976.csv
    curl --fail --location --progress-bar --output assets/atmosphere/VenusGRAMNominal.csv {{support_data_url}}/AtmosphereData/VenusGRAMNominal.csv
    curl --fail --location --progress-bar --output assets/atmosphere/UranusGRAMNominal.csv {{support_data_url}}/AtmosphereData/UranusGRAMNominal.csv
    curl --fail --location --progress-bar --output assets/atmosphere/TitanGRAMNominal.csv {{support_data_url}}/AtmosphereData/TitanGRAMNominal.csv
    curl --fail --location --progress-bar --output assets/atmosphere/JupiterGRAMNominal.csv {{support_data_url}}/AtmosphereData/JupiterGRAMNominal.csv

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
