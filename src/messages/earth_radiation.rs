//! Earth albedo and infrared flux at a spacecraft (upstream
//! `EarthRadiationMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Total albedo and infrared flux at the spacecraft with their flux-weighted
/// directions: upstream `EarthRadiationMsg`, published by the Earth radiation
/// model.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct EarthRadiationMsg {
    /// Upstream `albedoFlux` \[W/m^2\]: total reflected solar flux.
    pub albedo_flux_w_per_m2: f64,
    /// Upstream `irFlux` \[W/m^2\]: total infrared flux.
    pub ir_flux_w_per_m2: f64,
    /// Upstream `albedoDir_N` \[-\]: flux-weighted net albedo force direction,
    /// inertial components; zero when there is no albedo flux.
    pub albedo_direction_inertial: Vector3<f64>,
    /// Upstream `irDir_N` \[-\]: flux-weighted net infrared force direction,
    /// inertial components; zero when there is no infrared flux.
    pub ir_direction_inertial: Vector3<f64>,
    /// Upstream `timeTag` \[ns\]: simulation time the fluxes are valid at.
    pub time_tag_nanos: u64,
}

impl_telemetry!(EarthRadiationMsg {
    albedo_flux_w_per_m2,
    ir_flux_w_per_m2,
    albedo_direction_inertial,
    ir_direction_inertial,
    time_tag_nanos,
});
