//! The current terms of a charging equilibrium (upstream
//! `ScChargingCurrentsMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Every current acting on one spacecraft at its equilibrium potential:
/// upstream `ScChargingCurrentsMsg`, published by the charging equilibrium
/// module.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ScChargingCurrentsMsg {
    /// Upstream `electronCurrent` \[A\]: ambient plasma electron current.
    pub electron_current_a: f64,
    /// Upstream `ionCurrent` \[A\]: ambient plasma ion current.
    pub ion_current_a: f64,
    /// Upstream `seeElectronCurrent` \[A\]: secondary electron emission from
    /// ambient electrons.
    pub see_electron_current_a: f64,
    /// Upstream `seeIonCurrent` \[A\]: secondary electron emission from
    /// ambient ions.
    pub see_ion_current_a: f64,
    /// Upstream `backscatteringCurrent` \[A\]: ambient electron
    /// backscattering.
    pub backscattering_current_a: f64,
    /// Upstream `photoelectricCurrent` \[A\]: photoelectric current.
    pub photoelectric_current_a: f64,
    /// Upstream `nonBeamCurrent` \[A\]: sum of the six terms above.
    pub non_beam_current_a: f64,
    /// Upstream `beamCurrent` \[A\]: primary beam current (emitted by the
    /// servicer, received by the target).
    pub beam_current_a: f64,
    /// Upstream `beamSeeCurrent` \[A\]: beam-induced secondary electron
    /// emission (target only).
    pub beam_see_current_a: f64,
    /// Upstream `beamBackscatteringCurrent` \[A\]: beam-induced
    /// backscattering (target only).
    pub beam_backscattering_current_a: f64,
    /// Upstream `totalCurrent` \[A\]: the total the equilibrium solve zeroes.
    pub total_current_a: f64,
}

impl_telemetry!(ScChargingCurrentsMsg {
    electron_current_a,
    ion_current_a,
    see_electron_current_a,
    see_ion_current_a,
    backscattering_current_a,
    photoelectric_current_a,
    non_beam_current_a,
    beam_current_a,
    beam_see_current_a,
    beam_backscattering_current_a,
    total_current_a,
});
