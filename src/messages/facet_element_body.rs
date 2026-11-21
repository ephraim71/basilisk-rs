//! One spacecraft facet in the hub body frame (upstream
//! `FacetElementBodyMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Geometry and optical properties of one facet, hub body frame components:
/// upstream `FacetElementBodyMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FacetElementBodyMsg {
    /// Upstream `area` \[m^2\]: facet area.
    pub area_m2: f64,
    /// Upstream `r_CopB_B` \[m\]: facet center of pressure relative to the
    /// body frame origin `B`, body components.
    pub r_cop_b_body_m: Vector3<f64>,
    /// Upstream `nHat_B` \[-\]: facet unit normal, body components.
    pub normal_body: Vector3<f64>,
    /// Upstream `rotHat_B` \[-\]: facet rotation axis, body components (zero
    /// for a fixed facet).
    pub rotation_axis_body: Vector3<f64>,
    /// Upstream `c_diffuse` \[-\]: diffuse reflection coefficient.
    pub diffuse_coeff: f64,
    /// Upstream `c_specular` \[-\]: specular reflection coefficient.
    pub specular_coeff: f64,
}

impl_telemetry!(FacetElementBodyMsg {
    area_m2,
    r_cop_b_body_m,
    normal_body,
    rotation_axis_body,
    diffuse_coeff,
    specular_coeff,
});
