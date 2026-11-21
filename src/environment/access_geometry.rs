//! Geometry the surface-location access models share (`groundLocation`,
//! `groundMapping`, `stripLocation`): the planet frame read from a planet
//! state, the overhead tolerance, and the south-east-zenith (SEZ) azimuth and
//! angle rates. Upstream repeats this code in each module; it lives here once.

use nalgebra::{Matrix3, Vector3};

use crate::messages::PlanetStateMsg;

/// Planet translation and orientation relative to the inertial frame, from a
/// [`PlanetStateMsg`].
///
/// A planet without an orientation, or an unconnected planet input (which
/// reads the default message), is the identity-oriented, non-rotating planet
/// at the origin that upstream assumes when `planetInMsg` is not linked.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PlanetFrame {
    /// Upstream `dcm_PN`: inertial to planet-fixed DCM.
    pub dcm_pn: Matrix3<f64>,
    /// Upstream `r_PN_N` \[m\]: planet position, inertial components.
    pub r_pn_n: Vector3<f64>,
    /// Upstream `v_PN_N` [m/s]: planet velocity, inertial components.
    pub v_pn_n: Vector3<f64>,
    /// Upstream `w_PN_P` [rad/s]: angular velocity of the planet-fixed frame
    /// relative to inertial, planet-fixed components, from
    /// `[ω̃] = -[PN]' [PN]ᵀ`.
    pub omega_pn_p: Vector3<f64>,
}

impl PlanetFrame {
    pub fn from_planet_state(planet: &PlanetStateMsg) -> Self {
        let (dcm_pn, dcm_pn_dot) = planet.orientation.as_ref().map_or(
            (Matrix3::identity(), Matrix3::zeros()),
            |orientation| {
                (
                    orientation.inertial_to_fixed,
                    orientation.inertial_to_fixed_dot,
                )
            },
        );
        let omega_tilde_pn_p = -dcm_pn_dot * dcm_pn.transpose();
        Self {
            dcm_pn,
            r_pn_n: planet.position_inertial_m,
            v_pn_n: planet.velocity_inertial_mps,
            omega_pn_p: Vector3::new(
                omega_tilde_pn_p[(2, 1)],
                omega_tilde_pn_p[(0, 2)],
                omega_tilde_pn_p[(1, 0)],
            ),
        }
    }

    /// Upstream `w_PN` [rad/s] of `groundLocation`/`groundMapping`: the
    /// planet angular velocity in inertial components.
    pub fn omega_pn_n(&self) -> Vector3<f64> {
        self.dcm_pn.transpose() * self.omega_pn_p
    }
}

/// Upstream `overheadTol` \[m\]: below this horizontal SEZ distance the
/// spacecraft is directly overhead and the azimuth undefined. It separates
/// the quantization of a distant inertial origin from the round-off of the two
/// position differences and two frame rotations behind the SEZ position.
pub(crate) fn overhead_tolerance_m(
    r_pn_n: &Vector3<f64>,
    r_bp_n: &Vector3<f64>,
    r_lp_n: &Vector3<f64>,
) -> f64 {
    f64::EPSILON * (r_pn_n.norm() + 8.0 * (r_bp_n.norm() + r_lp_n.norm()))
}

/// Azimuth of a spacecraft and the rates of its azimuth and elevation, in the
/// location's SEZ frame.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct AzimuthAndRates {
    /// Upstream `azimuth` \[rad\].
    pub azimuth_rad: f64,
    /// Upstream `az_dot` [rad/s].
    pub azimuth_dot_radps: f64,
    /// Upstream `el_dot` [rad/s].
    pub elevation_dot_radps: f64,
}

/// Azimuth (measured from north, toward east) and the azimuth and elevation
/// rates of the SEZ position `r_bl_l` moving at `v_bl_l`. All three are zero
/// when the spacecraft is overhead, within `overhead_tolerance_m` of the
/// zenith axis.
pub(crate) fn azimuth_and_rates(
    r_bl_l: &Vector3<f64>,
    v_bl_l: &Vector3<f64>,
    overhead_tolerance_m: f64,
) -> AzimuthAndRates {
    let xy_norm = (r_bl_l.x.powi(2) + r_bl_l.y.powi(2)).sqrt();
    if xy_norm < overhead_tolerance_m {
        return AzimuthAndRates::default();
    }
    let cos_az = -r_bl_l.x / xy_norm;
    let sin_az = r_bl_l.y / xy_norm;
    AzimuthAndRates {
        azimuth_rad: sin_az.atan2(cos_az),
        azimuth_dot_radps: (-r_bl_l.x * v_bl_l.y + r_bl_l.y * v_bl_l.x) / xy_norm.powi(2),
        elevation_dot_radps: (v_bl_l.z / xy_norm
            - r_bl_l.z * (r_bl_l.x * v_bl_l.x + r_bl_l.y * v_bl_l.y) / xy_norm.powf(3.0))
            / (1.0 + (r_bl_l.z / xy_norm).powi(2)),
    }
}
