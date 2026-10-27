//! Ground-location access to a spacecraft (upstream `AccessMsgPayload`).

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::flatten::impl_telemetry;

/// Visibility, range and viewing geometry of a spacecraft `B` from a ground
/// location `L`, in the location's south-east-zenith (SEZ) frame: upstream
/// `AccessMsg`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct AccessMsg {
    /// Upstream `hasAccess` (`uint64_t`, 1 / 0): the location has access to
    /// the spacecraft.
    pub has_access: bool,
    /// Upstream `slantRange` \[m\]: range from the location to the spacecraft.
    pub slant_range_m: f64,
    /// Upstream `elevation` \[rad\]: elevation angle of the spacecraft.
    pub elevation_rad: f64,
    /// Upstream `azimuth` \[rad\]: azimuth angle of the spacecraft.
    pub azimuth_rad: f64,
    /// Upstream `range_dot` [m/s]: range rate in the rotating SEZ frame.
    pub range_dot_mps: f64,
    /// Upstream `el_dot` [rad/s]: elevation rate in the rotating SEZ frame.
    pub elevation_dot_radps: f64,
    /// Upstream `az_dot` [rad/s]: azimuth rate in the rotating SEZ frame.
    pub azimuth_dot_radps: f64,
    /// Upstream `r_BL_L` \[m\]: spacecraft position relative to the location,
    /// SEZ components.
    pub position_bl_l_m: Vector3<f64>,
    /// Upstream `v_BL_L` [m/s]: SEZ-relative time derivative of
    /// `position_bl_l_m`, SEZ components.
    pub velocity_bl_l_mps: Vector3<f64>,
    /// Upstream `hasIllumination` (`uint64_t`, 1 / 0): the illumination
    /// constraints are met.
    pub has_illumination: bool,
    /// Upstream `sunIncidenceAngle` \[rad\]: angle between the surface normal
    /// and the location-to-Sun vector.
    pub sun_incidence_angle_rad: f64,
    /// Upstream `scViewAngle` \[rad\]: angle between the surface normal and the
    /// location-to-spacecraft vector.
    pub sc_view_angle_rad: f64,
    /// Upstream `hasGlare` (`uint64_t`, 1 / 0): the specular glare factor
    /// meets the configured threshold.
    pub has_glare: bool,
    /// Upstream `glareFactor` [-]: alignment of the spacecraft with the ideal
    /// specular reflection, in [0, 1].
    pub glare_factor: f64,
}

impl_telemetry!(AccessMsg {
    has_access,
    slant_range_m,
    elevation_rad,
    azimuth_rad,
    range_dot_mps,
    elevation_dot_radps,
    azimuth_dot_radps,
    position_bl_l_m,
    velocity_bl_l_mps,
    has_illumination,
    sun_incidence_angle_rad,
    sc_view_angle_rad,
    has_glare,
    glare_factor,
});
