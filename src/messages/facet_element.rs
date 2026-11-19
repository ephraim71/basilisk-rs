//! One spacecraft facet in its own facet frame (upstream
//! `FacetElementMsgPayload`).

use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Geometry and optical properties of one facet, facet frame `F`
/// components: upstream `FacetElementMsg`. `facetedSpacecraftModel` maps it
/// to the hub frame as a [`super::FacetElementBodyMsg`].
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct FacetElementMsg {
    /// Upstream `area` \[m^2\]: facet area.
    pub area_m2: f64,
    /// Upstream `r_CopF_F` \[m\]: facet center of pressure relative to the
    /// facet frame origin `F`, facet components.
    pub r_cop_f_facet_m: Vector3<f64>,
    /// Upstream `nHat_F` \[-\]: facet normal, facet components.
    pub normal_facet: Vector3<f64>,
    /// Upstream `rotHat_F` \[-\]: facet articulation axis, facet components
    /// (zero for a fixed facet).
    pub rotation_axis_facet: Vector3<f64>,
    /// Upstream `dcm_F0B` \[-\]: initial facet frame `F0` relative to the hub
    /// frame `B`.
    pub dcm_facet0_body: Matrix3<f64>,
    /// Upstream `r_FB_B` \[m\]: facet frame origin `F` relative to the hub
    /// frame origin `B`, body components.
    pub r_fb_body_m: Vector3<f64>,
    /// Upstream `c_diffuse` \[-\]: diffuse reflection coefficient.
    pub diffuse_coeff: f64,
    /// Upstream `c_specular` \[-\]: specular reflection coefficient.
    pub specular_coeff: f64,
}

impl_telemetry!(FacetElementMsg {
    area_m2,
    r_cop_f_facet_m,
    normal_facet,
    rotation_axis_facet,
    dcm_facet0_body,
    r_fb_body_m,
    diffuse_coeff,
    specular_coeff,
});
