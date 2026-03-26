set shell := ["bash", "-euo", "pipefail", "-c"]

de440s_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp"
earth_bpc_url := "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc"
pck11_url := "https://public-data.nyxspace.com/anise/v0.4/pck11.pca"
ggm03s_url := "https://raw.githubusercontent.com/AVSLab/basilisk/develop/supportData/LocalGravData/GGM03S.txt"

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
