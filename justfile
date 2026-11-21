set shell := ["bash", "-euo", "pipefail", "-c"]

de440s_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp"
earth_bpc_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
pck11_url := "https://public-data.nyxspace.com/anise/v0.4/pck11.pca"
ggm03s_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData/LocalGravData/GGM03S.txt"
support_data_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData"
naif_generic_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels"
hst_spk_url := "https://naif.jpl.nasa.gov/pub/naif/HST/kernels/spk/hst_edited.bsp"
upstream_src_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/src"
upstream_examples_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/examples"
nh_spk_url := "https://naif.jpl.nasa.gov/pub/naif/pds/data/nh-j_p_ss-spice-6-v1.0/nhsp_1000/data/spk/nh_pred_od077.bsp"
albedo_url := "https://raw.githubusercontent.com/AVSLab/basilisk/ea95ec4efe6d3e842a26546de653b7505dd054c3/supportData/AlbedoData"
reference_csv_zip := env_var_or_default("BASILISK_REFERENCE_CSV_ZIP", "/home/ephraim/hamming/csvs.zip")

default:
    @just --list

fetch-assets: fetch-anise-assets fetch-gravity fetch-albedo fetch-environment-data

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

# Gravity fields and the New Horizons SPK that the orbit example scenarios read
# besides `fetch-gravity` and `fetch-spice-kernels`: the J2-only Earth and Mars
# fields, the full GGM2B Mars field and upstream's `nh_pred_od077.bsp`.
fetch-orbit-scenario-data:
    mkdir -p assets/gravity assets/spice
    curl --fail --location --progress-bar --output assets/gravity/GGM03S-J2-only.txt {{support_data_url}}/LocalGravData/GGM03S-J2-only.txt
    curl --fail --location --progress-bar --output assets/gravity/GGM2B-J2-only.txt {{support_data_url}}/LocalGravData/GGM2B-J2-only.txt
    curl --fail --location --progress-bar --output assets/gravity/GGM2BData.txt {{support_data_url}}/LocalGravData/GGM2BData.txt
    curl --fail --location --progress-bar --output assets/spice/nh_pred_od077.bsp {{nh_spk_url}}

# Small-body data the formation and small-body example scenarios read:
# upstream's Vesta spherical-harmonic field (`VESTA20H.txt`) and the Eros
# polyhedral shape (`eros007790.tab`).
fetch-small-body-data:
    mkdir -p assets/gravity
    curl --fail --location --progress-bar --output assets/gravity/VESTA20H.txt {{support_data_url}}/LocalGravData/VESTA20H.txt
    curl --fail --location --progress-bar --output assets/gravity/eros007790.tab {{support_data_url}}/LocalGravData/eros007790.tab

# Data the SPICE, Vizard and Monte Carlo example scenarios read besides
# `fetch-assets` and `fetch-spice-kernels`: upstream's custom spacecraft SPK
# (`spacecraft_21T01.bsp`) and the two servicer/target state files of
# `scenarioDataToViz`.
fetch-misc-scenario-data:
    mkdir -p assets/spice assets/data_for_examples
    curl --fail --location --progress-bar --output assets/spice/spacecraft_21T01.bsp {{upstream_examples_url}}/dataForExamples/Spice/spacecraft_21T01.bsp
    curl --fail --location --progress-bar --output assets/data_for_examples/scHoldTraj_rotating_MRP.csv {{upstream_examples_url}}/dataForExamples/scHoldTraj_rotating_MRP.csv
    curl --fail --location --progress-bar --output assets/data_for_examples/scHoldTraj_rotating_EP.csv {{upstream_examples_url}}/dataForExamples/scHoldTraj_rotating_EP.csv

# Denton GEO plasma flux tables and tabulated atmospheres from upstream
# supportData, for the Denton flux and tabular atmosphere modules (the albedo
# maps are `fetch-albedo`'s).
fetch-environment-data:
    mkdir -p assets/denton assets/atmosphere
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

# Test images from upstream's `_UnitTest` directories for the `opencv`
# feature's `houghCircles`, `limbFinding`, `centerRadiusCNN` and `camera`, and
# the trained `centerRadiusCNN` network (`CAD.onnx`, 26 MB).
fetch-opencv-assets:
    mkdir -p assets/opencv/hough_circles assets/opencv/limb_finding assets/opencv/center_radius_cnn assets/opencv/camera
    curl --fail --location --progress-bar --output assets/opencv/hough_circles/mars.png {{upstream_src_url}}/fswAlgorithms/imageProcessing/houghCircles/_UnitTest/mars.png
    curl --fail --location --progress-bar --output assets/opencv/hough_circles/moons.png {{upstream_src_url}}/fswAlgorithms/imageProcessing/houghCircles/_UnitTest/moons.png
    curl --fail --location --progress-bar --output assets/opencv/limb_finding/MarsBright.jpg {{upstream_src_url}}/fswAlgorithms/imageProcessing/limbFinding/_UnitTest/MarsBright.jpg
    curl --fail --location --progress-bar --output assets/opencv/limb_finding/MarsDark.jpg {{upstream_src_url}}/fswAlgorithms/imageProcessing/limbFinding/_UnitTest/MarsDark.jpg
    curl --fail --location --progress-bar --output assets/opencv/limb_finding/moons.jpg {{upstream_src_url}}/fswAlgorithms/imageProcessing/limbFinding/_UnitTest/moons.jpg
    curl --fail --location --progress-bar --output assets/opencv/center_radius_cnn/mars.jpg {{upstream_src_url}}/fswAlgorithms/imageProcessing/centerRadiusCNN/_UnitTest/mars.jpg
    curl --fail --location --progress-bar --output assets/opencv/center_radius_cnn/mars2.jpg {{upstream_src_url}}/fswAlgorithms/imageProcessing/centerRadiusCNN/_UnitTest/mars2.jpg
    curl --fail --location --progress-bar --output assets/opencv/center_radius_cnn/mars3.jpg {{upstream_src_url}}/fswAlgorithms/imageProcessing/centerRadiusCNN/_UnitTest/mars3.jpg
    curl --fail --location --progress-bar --output assets/opencv/center_radius_cnn/CAD.onnx {{upstream_src_url}}/fswAlgorithms/imageProcessing/centerRadiusCNN/CAD.onnx
    curl --fail --location --progress-bar --output assets/opencv/camera/mars.jpg {{upstream_src_url}}/simulation/sensors/camera/_UnitTest/mars.jpg
    curl --fail --location --progress-bar --output assets/opencv/camera/tv_test.png {{upstream_src_url}}/simulation/sensors/camera/_UnitTest/tv_test.png

fetch-albedo:
    mkdir -p assets/albedo
    for f in Earth_ALB_2018_CERES_All Earth_ALB_2018_CERES_Clear Mars_ALB_TES; do for r in 1x1 5x5 10x10; do curl --fail --location --progress-bar --output assets/albedo/${f}_${r}.csv {{albedo_url}}/${f}_${r}.csv; done; done

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

# Run every ported upstream example scenario and write its CSV files, each in
# all the parametrizations upstream's `test_scenario*.py` runs.
run-all-scenarios: run-scenarios run-orbit-scenarios run-traj-scenarios run-formation-scenarios run-ground-scenarios run-multibody-b-scenarios run-misc-scenarios run-bsksim-scenarios run-multisat-scenarios run-montecarlo-scenarios run-multibody-a-scenarios run-opnav-scenarios run-dyncomp-scenarios
    cargo run --release --example scenario_attitude_feedback -- --all
    cargo run --release --example scenario_attitude_feedback_2t -- --all
    cargo run --release --example scenario_attitude_feedback_2t_th -- --all
    cargo run --release --example scenario_attitude_feedback_2t_state_eff_th -- --all
    cargo run --release --example scenario_attitude_feedback_no_earth -- --all
    cargo run --release --example scenario_attitude_feedback_numba -- --all
    cargo run --release --example scenario_attitude_feedback_rw -- --all
    cargo run --release --example scenario_attitude_feedback_rw_power -- --all
    cargo run --release --example scenario_attitude_gg
    cargo run --release --example scenario_attitude_guidance -- --all
    cargo run --release --example scenario_attitude_pointing -- --all
    cargo run --release --example scenario_attitude_pointing_numba
    cargo run --release --example scenario_attitude_pointing_py
    cargo run --release --example scenario_attitude_steering -- --all
    cargo run --release --example scenario_css -- --all
    cargo run --release --example scenario_css_filters -- --all
    cargo run --release --example scenario_tam -- --all
    cargo run --release --example scenario_tam_comparison -- --all
    cargo run --release --example scenario_magnetic_field_centered_dipole -- --all
    cargo run --release --example scenario_magnetic_field_wmm -- --all
    cargo run --release --example scenario_albedo -- --all
    cargo run --release --example scenario_gauss_markov_random_walk -- --all
    cargo run --release --example scenario_temp_measurement_attitude
    cargo run --release --example scenario_sensor_thermal
    cargo run --release --example scenario_momentum_dumping
    cargo run --release --example scenario_mtb_momentum_management
    cargo run --release --example scenario_mtb_momentum_management_simple
    cargo run --release --example scenario_sep_momentum_management -- --all

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

# Run the orbit and gravity example scenarios in every parametrization of
# upstream's `test_scenario*.py` and write their CSV files. Reads the assets of
# `fetch-assets`, `fetch-spice-kernels` and `fetch-orbit-scenario-data`;
# `scenario_orbit_multi_body` needs the `spice` feature and
# `scenario_basic_orbit_stream` the `vizard` feature.
run-orbit-scenarios:
    cargo run --release --example scenario_basic_orbit -- --all
    cargo run --release --example scenario_central_body -- --all
    cargo run --release --example scenario_custom_grav_body
    cargo run --release --features spice --example scenario_orbit_multi_body -- --all
    cargo run --release --example scenario_patched_conics
    cargo run --release --example scenario_hohmann -- --r-second 30000000
    cargo run --release --example scenario_orbit_maneuver -- --all
    cargo run --release --example scenario_orbit_maneuver_th
    cargo run --release --example scenario_orbit_from_omm
    cargo run --release --example scenario_orbit_consistency_verification -- --days 12 --degree 8 --step 30
    cargo run --release --example scenario_halo_orbit
    cargo run --release --example scenario_lagrange_point_orbit -- --all
    cargo run --release --example scenario_lambert_solver
    cargo run --release --features vizard --example scenario_basic_orbit_stream -- --all

# Render the orbit scenarios' figures from their CSV files.
plot-orbit-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_basic_orbit.py --generated-dir "{{generated_dir}}" --save --no-show
    python3 examples/plot_scenario_central_body.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_custom_grav_body.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_orbit_multi_body.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_patched_conics.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_hohmann.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_orbit_maneuver.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_orbit_maneuver_th.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_orbit_from_omm.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_orbit_consistency_verification.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_halo_orbit.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_lagrange_point_orbit.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_lambert_solver.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_basic_orbit_stream.py --generated-dir "{{generated_dir}}"

# Run the trajectory, integrator and drag example scenarios in every
# parametrization of upstream's `test_scenario*.py` and write their CSV files.
# Reads the assets of `fetch-assets` and `fetch-orbit-scenario-data`.
run-traj-scenarios:
    cargo run --release --example scenario_jupiter_arrival
    cargo run --release --example scenario_integrators -- --all
    cargo run --release --example scenario_integrators_comparison
    cargo run --release --example scenario_variable_time_step_integrators -- --all
    cargo run --release --example scenario_aerobrake -- --all
    cargo run --release --example scenario_aerocapture -- --all
    cargo run --release --example scenario_drag_deorbit -- --all
    cargo run --release --example scenario_drag_sensitivity -- --all
    cargo run --release --example scenario_drag_rendezvous -- --all
    cargo run --release --example scenario_stochastic_drag_spacecraft -- --all
    cargo run --release --example scenario_debris_reorbit_et
    cargo run --release --example scenario_impact

# Render the trajectory, integrator and drag scenarios' figures from their CSV
# files.
plot-traj-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_jupiter_arrival.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_integrators.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_integrators_comparison.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_variable_time_step_integrators.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_aerobrake.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_aerocapture.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_drag_deorbit.py --generated-dir "{{generated_dir}}" --variant all
    python3 examples/plot_scenario_drag_sensitivity.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_drag_rendezvous.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_stochastic_drag_spacecraft.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_debris_reorbit_et.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_impact.py --generated-dir "{{generated_dir}}"
# Run the formation-flying, rendezvous, small-body and attitude-guidance
# example scenarios in every parametrization of upstream's `test_scenario*.py`
# and write their CSV files. Reads the assets of `fetch-assets`,
# `fetch-small-body-data` and `fetch-spice-kernels`;
# `scenario_vesta_orientation` needs the `mujoco` and `spice` features.
run-formation-scenarios:
    cargo run --release --example scenario_formation_basic
    cargo run --release --example scenario_formation_mean_oe_feedback -- --all
    cargo run --release --example scenario_formation_reconfig -- --all
    cargo run --release --example scenario_rendez_vous
    cargo run --release --example scenario_asteroid_arrival
    cargo run --release --example scenario_small_body_landmarks -- --all
    cargo run --release --example scenario_small_body_nav
    cargo run --release --example scenario_small_body_nav_ukf
    cargo run --release --example scenario_small_body_feedback_control
    cargo run --release --features mujoco,spice --example scenario_vesta_orientation
    cargo run --release --example scenario_att_guide_hyperbolic -- --all
    cargo run --release --example scenario_att_loc_point
    cargo run --release --example scenario_inertial_spiral
# Render the formation, small-body and attitude-guidance scenarios' figures
# from their CSV files.
plot-formation-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_formation_basic.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_formation_mean_oe_feedback.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_formation_reconfig.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_rendez_vous.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_asteroid_arrival.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_small_body_landmarks.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_small_body_nav.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_small_body_nav_ukf.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_small_body_feedback_control.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_vesta_orientation.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_att_guide_hyperbolic.py --variant all --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_att_loc_point.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_inertial_spiral.py --generated-dir "{{generated_dir}}"
# Run the ground-access, onboard data, power and multi-spacecraft example
# scenarios in every parametrization of upstream's `test_scenario*.py` and
# write their CSV files. Reads the assets of `fetch-assets` and
# `fetch-orbit-scenario-data`; `scenario_quad_maps` needs the `vizard`
# feature (`-- --save <file.bin>` also writes its Vizard file).
run-ground-scenarios:
    cargo run --release --example scenario_data_demo
    cargo run --release --example scenario_power_demo
    cargo run --release --example scenario_ground_downlink
    cargo run --release --example scenario_ground_location_imaging
    cargo run --release --example scenario_ground_mapping -- --all
    cargo run --release --example scenario_ground_tracks
    cargo run --release --example scenario_strip_imaging
    cargo run --release --example scenario_spacecraft_location
    cargo run --release --example scenario_satellite_constellation -- --all
    cargo run --release --features vizard --example scenario_quad_maps
    cargo run --release --example scenario_two_charged_sc
# Render the ground scenarios' figures from their CSV files
# (`scenario_quad_maps` has none, as upstream).
plot-ground-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_data_demo.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_power_demo.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_ground_downlink.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_ground_location_imaging.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_ground_mapping.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_ground_tracks.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_strip_imaging.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_spacecraft_location.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_satellite_constellation.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_two_charged_sc.py --generated-dir "{{generated_dir}}"

# Run the hinged, flexible, slosh, spinning-body, prescribed-motion and
# branching example scenarios in every parametrization of upstream's
# `test_scenario*.py` and write their CSV files. The deploying-panel,
# flexible-panel and extending-boom scenarios read the assets of
# `fetch-anise-assets`.
run-multibody-a-scenarios:
    cargo run --release --example scenario_hinged_rigid_body
    cargo run --release --example scenario_deploying_panel
    cargo run --release --example scenario_deploying_solar_arrays
    cargo run --release --example scenario_extending_boom
    cargo run --release --example scenario_flexible_panel -- --segments 5
    cargo run --release --example scenario_rotating_panel
    cargo run --release --example scenario_fuel_slosh -- --all
    cargo run --release --example scenario_spinning_bodies_two_dof -- --all
    cargo run --release --example scenario_sweeping_spacecraft -- --all
    cargo run --release --example scenario_thruster_arm
    cargo run --release --example scenario_branching_verification

# Render the multibody scenarios' figures from their CSV files.
plot-multibody-a-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_hinged_rigid_body.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_deploying_panel.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_deploying_solar_arrays.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_extending_boom.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_flexible_panel.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_rotating_panel.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_fuel_slosh.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_spinning_bodies_two_dof.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_sweeping_spacecraft.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_thruster_arm.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_branching_verification.py --generated-dir "{{generated_dir}}"

# Run the constrained-dynamics, prescribed-motion and robotic-arm example
# write their CSV files. `scenario_robotic_arm` reads the ANISE assets of
# `fetch-anise-assets`. `scenario_robotic_grappling` runs both upstream's
# default (journal-paper) configuration and its test configuration.
run-multibody-b-scenarios:
    cargo run --release --example scenario_constrained_dynamics -- --all
    cargo run --release --example scenario_constrained_dynamics_component_analysis -- --all
    cargo run --release --example scenario_constrained_dynamics_frequency_analysis
    cargo run --release --example scenario_constrained_dynamics_maneuver_analysis -- --all
    cargo run --release --example scenario_prescribed_motion_with_rotation_branching
    cargo run --release --example scenario_prescribed_motion_with_translation_branching
    cargo run --release --example scenario_prescribed_screw_motion
    cargo run --release --example scenario_rot_trans_array_deployment
    cargo run --release --example scenario_robotic_arm
    cargo run --release --example scenario_robotic_grappling
    cargo run --release --example scenario_robotic_grappling -- --test

# Render the constrained-dynamics, prescribed-motion and robotic-arm
# scenarios' figures from their CSV files.
plot-multibody-b-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_constrained_dynamics.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_constrained_dynamics_component_analysis.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_constrained_dynamics_frequency_analysis.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_constrained_dynamics_maneuver_analysis.py --generated-dir "{{generated_dir}}" --maneuver attitude
    python3 examples/plot_scenario_constrained_dynamics_maneuver_analysis.py --generated-dir "{{generated_dir}}" --maneuver orbit
    python3 examples/plot_scenario_prescribed_motion_with_rotation_branching.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_prescribed_motion_with_translation_branching.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_prescribed_screw_motion.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_rot_trans_array_deployment.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_robotic_arm.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_robotic_grappling.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_robotic_grappling.py --generated-dir "{{generated_dir}}" --variant Test
# Run the SPICE, Vizard, Monte Carlo, logging, constrained-attitude and
# benchmark example scenarios in every parametrization of upstream's
# `test_scenario*.py` and write their CSV files. Reads the assets of
# `fetch-assets`, `fetch-spice-kernels` and `fetch-misc-scenario-data`;
# `scenario_spice_spacecraft` and `scenario_monte_carlo_spice` need the
# `spice` feature, `scenario_spice_reconstruction` the `mujoco` and `spice`
# features and `scenario_data_to_viz` the `vizard` feature, which also adds
# the Vizard cases of `scenario_moving_camera` and `scenario_viz_point`.
run-misc-scenarios:
    cargo run --release --example scenario_bsk_log -- --all
    cargo run --release --example scenario_attitude_prescribed -- --all
    cargo run --release --example scenario_attitude_constraint_violation -- --all
    cargo run --release --example scenario_attitude_constrained_maneuver -- --all
    cargo run --release --example scenario_helio_trans_spice
    cargo run --release --example scenario_flyby_spice -- --all
    cargo run --release --features spice --example scenario_spice_spacecraft
    cargo run --release --features spice --example scenario_monte_carlo_spice
    cargo run --release --features mujoco,spice --example scenario_spice_reconstruction
    cargo run --release --example scenario_monte_carlo_att_rw -- --all
    cargo run --release --features vizard --example scenario_viz_point -- --all
    cargo run --release --features vizard --example scenario_viz_point
    cargo run --release --features vizard --example scenario_moving_camera -- --all
    cargo run --release --features vizard --example scenario_data_to_viz -- --all
    cargo run --release --example scenario_benchmark_numba -- --all
# Render the misc scenarios' figures from their CSV files.
plot-misc-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_attitude_prescribed.py --variant all --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_attitude_constraint_violation.py --variant all --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_attitude_constrained_maneuver.py --variant all --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_spice_spacecraft.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_spice_reconstruction.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_monte_carlo_att_rw.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_monte_carlo_att_rw.py --ic --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_viz_point.py --variant all --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_moving_camera.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_data_to_viz.py --variant all --generated-dir "{{generated_dir}}"

# Run the scenarios of upstream's BskSim framework (`examples/BskSim`) and
# write their CSV files. Reads the ANISE assets of `fetch-anise-assets`.
run-bsksim-scenarios:
    cargo run --release --example bsk_sim_basic_orbit
    cargo run --release --example bsk_sim_att_feedback
    cargo run --release --example bsk_sim_att_guidance
    cargo run --release --example bsk_sim_att_guid_hyperbolic
    cargo run --release --example bsk_sim_att_steering
    cargo run --release --example bsk_sim_att_eclipse
    cargo run --release --example bsk_sim_att_modes
    cargo run --release --example bsk_sim_feedback_rw
    cargo run --release --example bsk_sim_add_rw_fault
    cargo run --release --example bsk_sim_fault_list
    cargo run --release --example bsk_sim_lambert_guidance
    cargo run --release --example bsk_sim_basic_orbit_formation
    cargo run --release --example bsk_sim_relative_pointing_formation

# Render the BskSim scenarios' figures from their CSV files.
plot-bsksim-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_bsk_sim_basic_orbit.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_att_feedback.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_att_guidance.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_att_guid_hyperbolic.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_att_steering.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_att_eclipse.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_att_modes.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_feedback_rw.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_add_rw_fault.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_fault_list.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_lambert_guidance.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_basic_orbit_formation.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_bsk_sim_relative_pointing_formation.py --generated-dir "{{generated_dir}}"
# Run the MultiSatBskSim example scenarios (`multi_sat_*`, built on
# `examples/support/multi_sat`) in every parametrization of upstream's
# `test_scenario*.py` and write their CSV files. Reads the assets of
# `fetch-anise-assets`; the TLE files are in `examples/data/tle`.
run-multisat-scenarios:
    cargo run --release --example multi_sat_basic_orbit -- --all
    cargo run --release --example multi_sat_basic_orbit_mt -- --all
    cargo run --release --example multi_sat_att_guid -- --all
    cargo run --release --example multi_sat_station_keeping -- --all
    cargo run --release --example multi_sat_constellation_from_tle -- --all
# Render the MultiSatBskSim scenarios' figures from their CSV files.
plot-multisat-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_multi_sat_basic_orbit.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_multi_sat_basic_orbit_mt.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_multi_sat_att_guid.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_multi_sat_station_keeping.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_multi_sat_constellation_from_tle.py --generated-dir "{{generated_dir}}"
# Run the MonteCarloExamples scenarios in order: the campaign of the BskSim
# attitude-feedback scenario (archived in
# `examples/output/montecarlo/scenarioBskSimAttFeedbackMC`), the rerun of its
# run 1, and the visualization of its retained data. Reads the assets of
# `fetch-anise-assets`.
run-montecarlo-scenarios:
    cargo run --release --example scenario_bsk_sim_att_feedback_mc -- --all
    cargo run --release --example scenario_rerun_monte_carlo -- --all
    cargo run --release --example scenario_visualize_monte_carlo -- --all
# Render the Monte Carlo scenarios' figures from their CSV files.
plot-montecarlo-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_bsk_sim_att_feedback_mc.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_visualize_monte_carlo.py --generated-dir "{{generated_dir}}"
# Run the OpNavScenarios examples (built on `examples/support/opnav`, their
# camera images from the synthetic renderer that stands in for Vizard) at
# upstream's full lengths, the Monte Carlo campaigns of `OpNavMC` and the
# CNN training-image campaign of `CNN_ImageGen`, and write their CSV files
# (the campaigns' archives and images go to `examples/output/montecarlo`).
# Needs the `opencv` feature (system OpenCV, libclang) and the assets of
# `fetch-anise-assets`, `fetch-orbit-scenario-data` (`GGM2BData.txt`) and
# `fetch-opencv-assets` (the CNN's `CAD.onnx`).
run-opnav-scenarios:
    cargo run --release --features opencv --example scenario_op_nav_point -- --all
    cargo run --release --features opencv --example scenario_op_nav_point_limb -- --all
    cargo run --release --features opencv --example scenario_op_nav_heading -- --all
    cargo run --release --features opencv --example scenario_op_nav_od -- --all
    cargo run --release --features opencv --example scenario_op_nav_od_limb -- --all
    cargo run --release --features opencv --example scenario_op_nav_att_od -- --all
    cargo run --release --features opencv --example scenario_op_nav_att_od_limb -- --all
    cargo run --release --features opencv --example scenario_cnn_att_od -- --all
    cargo run --release --features opencv --example scenario_fault_det_op_nav -- --all
    cargo run --release --features opencv --example scenario_op_nav_att_od_mc -- --all
    cargo run --release --features opencv --example scenario_limb_att_od -- --all
    cargo run --release --features opencv --example scenario_limb_att_od -- --single
    cargo run --release --features opencv --example scenario_cnn_images -- --all
# Render the OpNav scenarios' figures from their CSV files.
plot-opnav-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_op_nav_point.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_point_limb.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_heading.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_od.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_od_limb.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_att_od.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_att_od_limb.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_cnn_att_od.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_fault_det_op_nav.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_att_od_mc.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_op_nav_att_od_mc.py --scenario scenario_LimbAttOD --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_limb_att_od.py --generated-dir "{{generated_dir}}"

# Run the dynamics-engine comparison series (upstream
# `examples/dynamicsComparison`: `runAllComparisons.py` and the six sweep
# drivers) and write their CSV and JSON files to `examples/output/scenarios`.
# The comparisons need the `mujoco` feature (without it each scenario runs
# its Backsubstitution side only). The five accuracy scenarios also write
# their runtime tables (`--timing`, as `runAllComparisons.py`), the
# variable-mass scenario also flies the deep-space burn and its near-rigid
# control, and the flexible-panel and Pareto studies run upstream's full
# default sweeps, the slowest part.
run-dyncomp-scenarios:
    cargo run --release --features mujoco --example scenario_compare_orbit -- --timing
    cargo run --release --features mujoco --example scenario_compare_torque -- --timing
    cargo run --release --features mujoco --example scenario_compare_rw_panels -- --timing
    cargo run --release --features mujoco --example scenario_compare_orbit_multibody -- --timing
    cargo run --release --features mujoco --example scenario_compare_variable_mass -- --timing
    cargo run --release --features mujoco --example scenario_compare_variable_mass -- --deep-space
    cargo run --release --features mujoco --example scenario_compare_variable_mass -- --deep-space --near-rigid --duration 120
    cargo run --release --features mujoco --example scenario_compare_flex_panels
    cargo run --release --features mujoco --example scenario_compare_pareto_rw_panels
    cargo run --release --features mujoco --example scenario_compare_pareto_flex_panels
    cargo run --release --features mujoco --example sweep_orbit_dt
    cargo run --release --features mujoco --example sweep_torque_artifact
    cargo run --release --features mujoco --example sweep_torque_mechanism_checks
    cargo run --release --features mujoco --example sweep_rw_bookkeeping_mass
    cargo run --release --features mujoco --example sweep_slosh_displacement
    cargo run --release --features mujoco --example sweep_variable_mass_pendulum_inertia

# Render the dynamics-comparison figures from their CSV files, the runtime
# tables (`dyncompRuntime.md`, `.png`) and the paper's print PDFs (`paper/`).
plot-dyncomp-scenarios generated_dir="examples/output/scenarios":
    python3 examples/plot_scenario_compare_orbit.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_torque.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_rw_panels.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_flex_panels.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_orbit_multibody.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_variable_mass.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_pareto_rw_panels.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_scenario_compare_pareto_flex_panels.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_sweep_variable_mass_pendulum_inertia.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_dyncomp_runtime.py --generated-dir "{{generated_dir}}"
    python3 examples/plot_dyncomp_paper_figures.py --generated-dir "{{generated_dir}}"

# Upstream's `runAllComparisons.py --timing-only` (`make -C docs
# comparison-runtime-tables`): time both engines in the five accuracy
# scenarios, then collect the tables. Use a quiet machine; the times
# describe this host and build only.
dyncomp-runtime-tables generated_dir="examples/output/scenarios":
    cargo run --release --features mujoco --example scenario_compare_orbit -- --timing
    cargo run --release --features mujoco --example scenario_compare_torque -- --timing
    cargo run --release --features mujoco --example scenario_compare_rw_panels -- --timing
    cargo run --release --features mujoco --example scenario_compare_orbit_multibody -- --timing
    cargo run --release --features mujoco --example scenario_compare_variable_mass -- --timing
    python3 examples/plot_dyncomp_runtime.py --generated-dir "{{generated_dir}}"

# Upstream's `paperFigures.py --update-provenance`: regenerate every input of
# the paper's figures and render the print PDFs and their hash manifest into
# `<generated_dir>/paper` (`plot_dyncomp_paper_figures.py --verify` checks
# them later).
dyncomp-paper-figures generated_dir="examples/output/scenarios":
    cargo run --release --features mujoco --example scenario_compare_pareto_rw_panels
    cargo run --release --features mujoco --example scenario_compare_pareto_flex_panels
    cargo run --release --features mujoco --example scenario_compare_flex_panels
    cargo run --release --features mujoco --example sweep_orbit_dt
    cargo run --release --features mujoco --example sweep_torque_artifact
    python3 examples/plot_dyncomp_paper_figures.py --generated-dir "{{generated_dir}}"
