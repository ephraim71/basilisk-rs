//! Planetary albedo seen by one instrument (upstream `AlbedoMsgPayload`).

use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Reflected sunlight reaching one instrument: upstream `AlbedoMsg`, one per
/// instrument of the albedo module.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AlbedoMsg {
    /// Upstream `albedoAtInstrumentMax` \[-\]: albedo flux ratio from every
    /// visible patch, ignoring the instrument's normal and field of view.
    pub albedo_at_instrument_max: f64,
    /// Upstream `AfluxAtInstrumentMax` \[W/m^2\]: albedo flux from every
    /// visible patch.
    pub albedo_flux_at_instrument_max_w_per_m2: f64,
    /// Upstream `albedoAtInstrument` \[-\]: albedo flux ratio through the
    /// instrument's field of view, cosine weighted by its normal.
    pub albedo_at_instrument: f64,
    /// Upstream `AfluxAtInstrument` \[W/m^2\]: albedo flux through the
    /// instrument's field of view.
    pub albedo_flux_at_instrument_w_per_m2: f64,
}

impl_telemetry!(AlbedoMsg {
    albedo_at_instrument_max,
    albedo_flux_at_instrument_max_w_per_m2,
    albedo_at_instrument,
    albedo_flux_at_instrument_w_per_m2,
});
