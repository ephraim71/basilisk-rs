//! Ambient plasma differential fluxes (upstream `PlasmaFluxMsgPayload`).

use serde::{Deserialize, Serialize};

use super::MAX_PLASMA_FLUX_SIZE;
use super::flatten::impl_telemetry;

/// Mean electron and ion differential fluxes on an energy grid: upstream
/// `PlasmaFluxMsg`. Entries past the bins in use are zero.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlasmaFluxMsg {
    /// Upstream `meanElectronFlux` \[m^-2 s^-1 sr^-1 eV^-1\]: electron
    /// differential flux at each energy. (Upstream labels it sr^-2.)
    #[serde(with = "super::big_array")]
    pub mean_electron_flux: [f64; MAX_PLASMA_FLUX_SIZE],
    /// Upstream `meanIonFlux` \[m^-2 s^-1 sr^-1 eV^-1\]: ion differential
    /// flux at each energy.
    #[serde(with = "super::big_array")]
    pub mean_ion_flux: [f64; MAX_PLASMA_FLUX_SIZE],
    /// Upstream `energies` \[eV\]: the energy grid.
    #[serde(with = "super::big_array")]
    pub energies_ev: [f64; MAX_PLASMA_FLUX_SIZE],
}

impl Default for PlasmaFluxMsg {
    fn default() -> Self {
        Self {
            mean_electron_flux: [0.0; MAX_PLASMA_FLUX_SIZE],
            mean_ion_flux: [0.0; MAX_PLASMA_FLUX_SIZE],
            energies_ev: [0.0; MAX_PLASMA_FLUX_SIZE],
        }
    }
}

impl_telemetry!(PlasmaFluxMsg {
    mean_electron_flux,
    mean_ion_flux,
    energies_ev,
});
