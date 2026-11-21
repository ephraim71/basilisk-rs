//! Spacecraft-to-spacecraft access: port of upstream Basilisk
//! `spacecraftLocation`.
//!
//! A location `L` fixed on a primary spacecraft `B` (offset `r_lb_b_m` from
//! its body origin) checks its line of sight to each tracked spacecraft `S`
//! past an ellipsoidal planet `P`, and publishes one [`AccessMsg`] per
//! tracked spacecraft. The ellipsoid is mapped onto a sphere of the
//! equatorial radius by scaling the planet-fixed 3-axis by `rEquator /
//! rPolar`; the line of sight is blocked when the closest point of the
//! `L`-to-`S` segment lies inside that sphere. An unblocked spacecraft has
//! access, which further optional conditions can remove:
//!
//! * a slant range beyond `maximum_range_m`;
//! * with a [`Boresight`], a direction outside its half-cone angle (the
//!   message's `elevation` is then the angle from the cone edge, `π/2 - φ`);
//! * with a boresight, which is also the surface normal: a view angle beyond
//!   `theta_view_rad` from the normal;
//! * with the Sun input connected: a Sun incidence angle beyond
//!   `theta_solar_rad`, an eclipse illumination factor below
//!   `min_illumination_factor` (when the eclipse input is connected), or
//!   specular glare at or above `glare_threshold` when
//!   `use_glare_constraint` is set.
//!
//! With the Sun input connected, the message also reports whether the
//! illumination conditions are met, the Sun incidence angle, and the glare
//! factor: the alignment of the viewing direction with the mirror reflection
//! of the Sun direction about the normal, zero unless both face the surface.
//! The view angle is reported whenever a boresight is set.
//!
//! The planet input is optional: unconnected, or without an orientation, the
//! planet is identity-oriented at the inertial origin. Upstream's Sun input is
//! a SPICE planet state; here it is the [`SunEphemerisMsg`] the other Sun
//! consumers (`eclipse`, `coarseSunSensor`) read. Upstream reads every
//! input's `isWritten` flag but never uses the result, so no write-time
//! stand-in is needed.

use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::messages::{
    AccessMsg, EclipseMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::utilities::linear_algebra::safe_acos;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `surfaceNormalTolerance` [-]: a shorter `aHat_B` is no surface
/// normal.
const SURFACE_NORMAL_TOLERANCE: f64 = 1e-3;
/// Upstream `vectorNormTolerance` \[m\]: shorter location-to-Sun and
/// location-to-spacecraft vectors are an error.
const VECTOR_NORM_TOLERANCE_M: f64 = 1e-12;
/// Upstream `glareTolerance` [-]: round-off allowance of the glare threshold
/// test, for the normalizations, the reflection and the dot products.
const GLARE_TOLERANCE: f64 = 16.0 * f64::EPSILON;

/// Eigen's `normalized()`, as upstream: division by the norm, and a zero
/// vector returned unchanged.
fn eigen_normalized(v: Vector3<f64>) -> Vector3<f64> {
    let squared_norm = v.norm_squared();
    if squared_norm > 0.0 {
        v / squared_norm.sqrt()
    } else {
        v
    }
}

/// Sensor or communication boresight of the primary spacecraft, also the
/// surface normal of the location for illumination, viewing and glare.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Boresight {
    /// Upstream `aHat_B` [-]: boresight direction, body components;
    /// normalized at reset.
    pub a_hat_b: Vector3<f64>,
    /// Upstream `theta` \[rad\]: half-cone angle the tracked spacecraft must
    /// lie within. Zero, as upstream, applies no cone.
    pub theta_rad: f64,
}

/// User-settable parameters of [`SpacecraftLocation`]. Build it with
/// [`Self::new`], which takes the one parameter upstream requires.
#[derive(Clone, Debug)]
pub struct SpacecraftLocationConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `rEquator` \[m\]: equatorial planet radius.
    pub r_equator_m: f64,
    /// Upstream `rPolar` \[m\]: polar planet radius; `None` (upstream's `-1`)
    /// is a sphere of `r_equator_m`.
    pub r_polar_m: Option<f64>,
    /// Upstream `maximumRange` \[m\]: largest slant range with access. `None`
    /// is upstream's `-1`, no limit; as upstream, zero also disables it.
    pub maximum_range_m: Option<f64>,
    /// Upstream `r_LB_B` \[m\]: location relative to the primary's body
    /// origin, body components.
    pub r_lb_b_m: Vector3<f64>,
    /// Upstream `aHat_B` and `theta` (optional).
    pub boresight: Option<Boresight>,
    /// Upstream `theta_solar` \[rad\] (optional, needs a boresight): largest
    /// Sun incidence angle at which the location is illuminated.
    pub theta_solar_rad: Option<f64>,
    /// Upstream `theta_view` \[rad\] (optional, needs a boresight): largest
    /// angle from the surface normal to the tracked spacecraft.
    pub theta_view_rad: Option<f64>,
    /// Upstream `min_illumination_factor` [-] (optional): smallest eclipse
    /// illumination factor at which the location is illuminated. As
    /// upstream, a value of zero or below never applies. Upstream's
    /// deprecated alias `min_shadow_factor` is not carried over.
    pub min_illumination_factor: Option<f64>,
    /// Upstream `glareThreshold` [-]: smallest glare factor classified as
    /// glare, in `[0, 1]`; 0.95 by default.
    pub glare_threshold: f64,
    /// Upstream `useGlareConstraint`: whether glare removes access.
    pub use_glare_constraint: bool,
}

impl SpacecraftLocationConfig {
    /// Upstream's constructor defaults, for a planet of equatorial radius
    /// `r_equator_m` \[m\].
    pub fn new(r_equator_m: f64) -> Self {
        Self {
            name: "spacecraftLocation".to_string(),
            r_equator_m,
            r_polar_m: None,
            maximum_range_m: None,
            r_lb_b_m: Vector3::zeros(),
            boresight: None,
            theta_solar_rad: None,
            theta_view_rad: None,
            min_illumination_factor: None,
            glare_threshold: 0.95,
            use_glare_constraint: false,
        }
    }

    /// Checks the configuration, as upstream's `Reset` does.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.r_equator_m > 0.0 && self.r_equator_m.is_finite()) {
            return Err(format!(
                "r_equator_m must be set to the planet equatorial radius, got {}",
                self.r_equator_m
            ));
        }
        if let Some(r_polar_m) = self.r_polar_m
            && !(r_polar_m > 0.0 && r_polar_m.is_finite())
        {
            return Err(format!(
                "r_polar_m must be positive and finite, got {r_polar_m}"
            ));
        }
        if let Some(boresight) = self.boresight {
            if !(boresight.a_hat_b.norm() > SURFACE_NORMAL_TOLERANCE
                && boresight.a_hat_b.iter().all(|c| c.is_finite()))
            {
                return Err(format!(
                    "boresight.a_hat_b must be a finite nonzero direction, got {:?}",
                    boresight.a_hat_b
                ));
            }
            if boresight.theta_rad.is_nan() || boresight.theta_rad < 0.0 {
                return Err(format!(
                    "boresight.theta_rad must be set (non-negative) with a_hat_b, got {}",
                    boresight.theta_rad
                ));
            }
        }
        for (name, angle) in [
            ("theta_solar_rad", self.theta_solar_rad),
            ("theta_view_rad", self.theta_view_rad),
        ] {
            let Some(angle) = angle else { continue };
            if self.boresight.is_none() {
                return Err(format!("must set boresight.a_hat_b if you specify {name}"));
            }
            if angle.is_nan() || angle < 0.0 {
                return Err(format!("{name} must be non-negative, got {angle}"));
            }
        }
        if !(self.glare_threshold.is_finite() && (0.0..=1.0).contains(&self.glare_threshold)) {
            return Err(format!(
                "glare_threshold must be finite and in the range [0, 1], got {}",
                self.glare_threshold
            ));
        }
        Ok(())
    }
}

/// Line of sight and sensing conditions from a primary spacecraft to others:
/// upstream `SpacecraftLocation`.
#[derive(Clone, Debug)]
pub struct SpacecraftLocation {
    /// User-settable parameters.
    pub config: SpacecraftLocationConfig,
    /// Upstream `primaryScStateInMsg`: the spacecraft carrying the location.
    pub primary_sc_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `planetInMsg` (optional): planet state and orientation.
    pub planet_in_msg: Input<PlanetStateMsg>,
    /// Upstream `sunInMsg` (optional): Sun position.
    pub sun_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `eclipseInMsg` (optional): illumination factor at the
    /// location.
    pub eclipse_in_msg: Input<EclipseMsg>,
    /// Upstream `scStateInMsgs`: one per tracked spacecraft, added by
    /// [`Self::add_spacecraft_to_model`].
    pub sc_state_in_msgs: Vec<Input<SpacecraftStateMsg>>,
    /// Upstream `accessOutMsgs`: access to each tracked spacecraft, in the
    /// order of `sc_state_in_msgs`.
    pub access_out_msgs: Vec<Output<AccessMsg>>,
    /// Upstream `zScale`: `rEquator / rPolar`.
    z_scale: f64,
}

impl SpacecraftLocation {
    /// Builds the module with unconnected ports and no tracked spacecraft.
    ///
    /// # Panics
    ///
    /// If `config` fails [`SpacecraftLocationConfig::validate`].
    pub fn new(config: SpacecraftLocationConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid SpacecraftLocationConfig: {message}");
        }
        let z_scale = config.r_equator_m / config.r_polar_m.unwrap_or(config.r_equator_m);
        Self {
            config,
            primary_sc_state_in_msg: Input::default(),
            planet_in_msg: Input::default(),
            sun_in_msg: Input::default(),
            eclipse_in_msg: Input::default(),
            sc_state_in_msgs: Vec::new(),
            access_out_msgs: Vec::new(),
            z_scale,
        }
    }

    /// Upstream `addSpacecraftToModel`: adds a tracked spacecraft state input
    /// and its access output, and returns their index in `sc_state_in_msgs`
    /// and `access_out_msgs`.
    pub fn add_spacecraft_to_model(&mut self) -> usize {
        self.sc_state_in_msgs.push(Input::default());
        self.access_out_msgs.push(Output::default());
        self.access_out_msgs.len() - 1
    }

    /// Upstream `Reset`: checks the ports and configuration, and fixes the
    /// ellipsoid scaling.
    fn reset_state(&mut self) {
        if self.sc_state_in_msgs.is_empty() {
            log::error!(
                "{}: must have at least one spacecraft added through `add_spacecraft_to_model`",
                self.config.name
            );
        }
        if !self.primary_sc_state_in_msg.is_connected() {
            log::error!(
                "{}: `primary_sc_state_in_msg` must be connected",
                self.config.name
            );
        }
        if let Err(message) = self.config.validate() {
            log::error!("{}: {message}", self.config.name);
        }
        self.z_scale =
            self.config.r_equator_m / self.config.r_polar_m.unwrap_or(self.config.r_equator_m);
    }

    /// Upstream `computeAccess`: one access message per tracked spacecraft.
    fn compute_access(&self) -> Vec<AccessMsg> {
        let config = &self.config;
        let planet = self.planet_in_msg.read();
        let dcm_pn = planet
            .orientation
            .as_ref()
            .map_or_else(nalgebra::Matrix3::identity, |o| o.inertial_to_fixed);
        let r_pn_n = planet.position_inertial_m;
        let sun_connected = self.sun_in_msg.is_connected();
        let r_hn_n = self.sun_in_msg.read().sun_position_inertial_m;
        let eclipse = self
            .eclipse_in_msg
            .is_connected()
            .then(|| self.eclipse_in_msg.read());

        let primary = self.primary_sc_state_in_msg.read();
        let dcm_nb = mrp_to_dcm(primary.sigma_bn).transpose();
        let r_ln_n = primary.position_m + dcm_nb * config.r_lb_b_m;
        let mut r_lp_p = dcm_pn * (r_ln_n - r_pn_n);

        let a_hat_b = config
            .boresight
            .map(|boresight| eigen_normalized(boresight.a_hat_b));
        let a_hat_n = a_hat_b.map(|a_hat_b| eigen_normalized(dcm_nb * a_hat_b));
        let mut sun_hat_l_n = Vector3::zeros();
        let mut sun_incidence_angle_rad = 0.0;
        if let (true, Some(a_hat_n)) = (sun_connected, a_hat_n) {
            let r_hl_n = r_hn_n - r_ln_n;
            if r_hl_n.norm() < VECTOR_NORM_TOLERANCE_M {
                log::error!(
                    "{}: location-to-Sun vector must have nonzero length",
                    config.name
                );
            }
            sun_hat_l_n = eigen_normalized(r_hl_n);
            sun_incidence_angle_rad = safe_acos(a_hat_n.dot(&sun_hat_l_n));
        }

        // Affine scaling that maps the ellipsoid onto a sphere.
        r_lp_p.z *= self.z_scale;

        self.sc_state_in_msgs
            .iter()
            .map(|input| {
                let r_sn_n = input.read().position_m;
                let mut r_sp_p = dcm_pn * (r_sn_n - r_pn_n);
                r_sp_p.z *= self.z_scale;
                let mut r_sl_p = r_sp_p - r_lp_p;

                // Closest point of the segment to the planet centre, as
                // `std::max(std::min(param, 1.0), 0.0)`: a NaN stays NaN.
                let param = -r_lp_p.dot(&r_sl_p) / r_sl_p.dot(&r_sl_p);
                let param = if 1.0 < param { 1.0 } else { param };
                let param = if param < 0.0 { 0.0 } else { param };
                let r_close = r_lp_p + param * r_sl_p;

                let mut access = AccessMsg::default();
                if r_close.norm() > config.r_equator_m {
                    r_sl_p.z /= self.z_scale;
                    let range = r_sl_p.norm();
                    access.slant_range_m = range;
                    access.has_access = true;
                    if config
                        .maximum_range_m
                        .is_some_and(|maximum| maximum > 0.0 && range > maximum)
                    {
                        access.has_access = false;
                    }
                    if let (Some(boresight), Some(a_hat_b)) = (config.boresight, a_hat_b)
                        && boresight.theta_rad > 0.0
                    {
                        let a_hat_p = dcm_pn * dcm_nb * a_hat_b;
                        let phi = safe_acos(r_sl_p.dot(&a_hat_p) / range);
                        access.elevation_rad = FRAC_PI_2 - phi;
                        if phi > boresight.theta_rad {
                            access.has_access = false;
                        }
                    }
                }

                let mut sc_view_hat_l_n = Vector3::zeros();
                if let Some(a_hat_n) = a_hat_n {
                    // Viewing geometry does not need the Sun.
                    let r_sl_n = r_sn_n - r_ln_n;
                    if r_sl_n.norm() < VECTOR_NORM_TOLERANCE_M {
                        log::error!(
                            "{}: location-to-spacecraft vector must have nonzero length",
                            config.name
                        );
                    }
                    sc_view_hat_l_n = eigen_normalized(r_sl_n);
                    access.sc_view_angle_rad = safe_acos(a_hat_n.dot(&sc_view_hat_l_n));
                    if config
                        .theta_view_rad
                        .is_some_and(|limit| access.sc_view_angle_rad > limit)
                    {
                        access.has_access = false;
                    }
                }

                if sun_connected {
                    access.has_illumination = true;
                    if let Some(a_hat_n) = a_hat_n {
                        access.sun_incidence_angle_rad = sun_incidence_angle_rad;
                        let sun_normal_projection = a_hat_n.dot(&sun_hat_l_n);
                        let view_normal_projection = a_hat_n.dot(&sc_view_hat_l_n);
                        let front_facing =
                            sun_normal_projection > 0.0 && view_normal_projection > 0.0;
                        if front_facing {
                            let reflected_sun_hat_n =
                                2.0 * sun_normal_projection * a_hat_n - sun_hat_l_n;
                            access.glare_factor =
                                reflected_sun_hat_n.dot(&sc_view_hat_l_n).clamp(0.0, 1.0);
                        }
                        if front_facing
                            && access.glare_factor + GLARE_TOLERANCE >= config.glare_threshold
                        {
                            access.has_glare = true;
                            if config.use_glare_constraint {
                                access.has_access = false;
                            }
                        }
                    }
                    if config
                        .theta_solar_rad
                        .is_some_and(|limit| sun_incidence_angle_rad > limit)
                    {
                        access.has_access = false;
                        access.has_illumination = false;
                    }
                    if let (Some(eclipse), Some(minimum)) =
                        (&eclipse, config.min_illumination_factor)
                        && minimum > 0.0
                        && eclipse.illumination_factor < minimum
                    {
                        access.has_access = false;
                        access.has_illumination = false;
                    }
                }
                access
            })
            .collect()
    }
}

impl Module for SpacecraftLocation {
    fn init(&mut self) {
        self.reset_state();
        for output in &self.access_out_msgs {
            output.write(AccessMsg::default());
        }
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        for (output, access) in self.access_out_msgs.iter().zip(self.compute_access()) {
            output.write(access);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports every case of upstream
    //! `spacecraftLocation/_UnitTest/test_unitSpacecraftLocation.py` with its
    //! tolerances. Upstream's check that `Reset` replaces an unset `rPolar`
    //! is covered by its geometric consequence (the default-polar-radius
    //! cases); the deprecated `min_shadow_factor` alias is not ported.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Matrix3;

    use super::*;
    use crate::messages::PlanetOrientation;
    use crate::utilities::astro_constants::{AU, REQ_EARTH};
    use crate::utilities::rigid_body_kinematics::euler3;

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    #[track_caller]
    fn assert_near(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{actual} vs {expected}, tolerance {tolerance}"
        );
    }

    /// Inputs of one upstream case, left unconnected when `None`.
    #[derive(Default)]
    struct Inputs {
        planet: Option<PlanetStateMsg>,
        sun_position_m: Option<Vector3<f64>>,
        eclipse_factor: Option<f64>,
    }

    /// Initializes `config` with a primary and the tracked spacecraft, and
    /// runs one update at time zero.
    fn single_step(
        config: SpacecraftLocationConfig,
        primary: SpacecraftStateMsg,
        others: &[Vector3<f64>],
        inputs: Inputs,
    ) -> Vec<AccessMsg> {
        let primary_output = Output::new(primary);
        let other_outputs: Vec<_> = others
            .iter()
            .map(|position_m| {
                Output::new(SpacecraftStateMsg {
                    position_m: *position_m,
                    ..SpacecraftStateMsg::default()
                })
            })
            .collect();
        let planet_output = inputs.planet.map(Output::new);
        let sun_output = inputs.sun_position_m.map(|position| {
            Output::new(SunEphemerisMsg {
                sun_position_inertial_m: position,
                ..SunEphemerisMsg::default()
            })
        });
        let eclipse_output = inputs.eclipse_factor.map(|illumination_factor| {
            Output::new(EclipseMsg {
                illumination_factor,
            })
        });

        let mut module = SpacecraftLocation::new(config);
        primary_output.connect_to(&mut module.primary_sc_state_in_msg);
        for output in &other_outputs {
            let index = module.add_spacecraft_to_model();
            output.connect_to(&mut module.sc_state_in_msgs[index]);
        }
        if let Some(output) = &planet_output {
            output.connect_to(&mut module.planet_in_msg);
        }
        if let Some(output) = &sun_output {
            output.connect_to(&mut module.sun_in_msg);
        }
        if let Some(output) = &eclipse_output {
            output.connect_to(&mut module.eclipse_in_msg);
        }
        module.init();
        module.update(&context());
        module.access_out_msgs.iter().map(Output::read).collect()
    }

    fn at(position_m: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            position_m,
            ..SpacecraftStateMsg::default()
        }
    }

    /// Upstream `test_spacecraftLocation`: default polar radius, default
    /// planet, oblateness and the optional boresight cone.
    #[test]
    fn occultation_range_and_boresight_cone() {
        let req_m = REQ_EARTH * 1000.0;
        for default_polar_radius in [false, true] {
            for default_planet in [false, true] {
                for latitude_deg in [55.0_f64, 65.0, 115.0] {
                    for max_range in [-1.0, 3000.0 * 1000.0] {
                        for cone in [0.0, 1.0, -1.0] {
                            let mut config = SpacecraftLocationConfig::new(req_m);
                            if !default_polar_radius {
                                config.r_polar_m = Some(req_m * 0.5);
                            }
                            config.maximum_range_m = (max_range > 0.0).then_some(max_range);
                            if cone != 0.0 {
                                config.boresight = Some(Boresight {
                                    a_hat_b: Vector3::new(0.0, 0.0, cone),
                                    theta_rad: 80.0_f64.to_radians(),
                                });
                            }
                            let mut planet_position = Vector3::zeros();
                            let mut inputs = Inputs::default();
                            if !default_planet {
                                planet_position = Vector3::new(AU * 1000.0, 0.0, 0.0);
                                inputs.planet = Some(PlanetStateMsg {
                                    position_inertial_m: planet_position,
                                    orientation: Some(PlanetOrientation {
                                        inertial_to_fixed: euler3((-90.0_f64).to_radians()),
                                        inertial_to_fixed_dot: Matrix3::zeros(),
                                    }),
                                    ..PlanetStateMsg::default()
                                });
                            }
                            let r = req_m + 1000.0 * 1000.0;
                            let angle = latitude_deg.to_radians();
                            let access = single_step(
                                config,
                                SpacecraftStateMsg {
                                    position_m: planet_position + Vector3::new(r, 0.0, 0.0),
                                    sigma_bn: Vector3::new(1.0, 0.0, 0.0),
                                    ..SpacecraftStateMsg::default()
                                },
                                &[planet_position
                                    + Vector3::new(r * angle.cos(), 0.0, r * angle.sin())],
                                inputs,
                            )[0]
                            .clone();

                            let case = format!(
                                "polar default {default_polar_radius}, planet default \
                                 {default_planet}, latitude {latitude_deg}, range {max_range}, \
                                 cone {cone}"
                            );
                            if latitude_deg == 55.0
                                || (latitude_deg == 65.0 && !default_polar_radius)
                            {
                                let mut true_access = true;
                                if max_range > 0.0 && access.slant_range_m > max_range {
                                    true_access = false;
                                }
                                if cone == 1.0 {
                                    true_access = false;
                                }
                                assert_eq!(access.has_access, true_access, "{case}");
                                assert!(access.slant_range_m > 1e-6, "{case}");
                            }
                            if (latitude_deg == 65.0 && default_polar_radius)
                                || latitude_deg == 115.0
                            {
                                assert!(!access.has_access, "{case}");
                                assert!(access.slant_range_m.abs() <= 1e-6, "{case}");
                            }
                        }
                    }
                }
            }
        }
    }

    /// Upstream `test_spacecraftLocationColinearAccess` (issues #644/#946):
    /// radially aligned spacecraft see each other.
    #[test]
    fn radially_aligned_spacecraft_have_access() {
        let req_m = REQ_EARTH * 1000.0;
        let r_primary = req_m + 1000.0e3;
        let r_other = req_m + 2000.0e3;
        let access = single_step(
            SpacecraftLocationConfig::new(req_m),
            at(Vector3::new(r_primary, 0.0, 0.0)),
            &[Vector3::new(r_other, 0.0, 0.0)],
            Inputs::default(),
        )[0]
        .clone();
        assert!(access.has_access);
        assert_near(
            access.slant_range_m,
            r_other - r_primary,
            1e-6 * (r_other - r_primary),
        );
    }

    /// Upstream `direction_from_normal`: a unit direction at `polar_angle`
    /// from the 1-axis normal and `azimuth` about it.
    fn direction_from_normal(polar_angle: f64, azimuth: f64) -> Vector3<f64> {
        Vector3::new(
            polar_angle.cos(),
            polar_angle.sin() * azimuth.cos(),
            polar_angle.sin() * azimuth.sin(),
        )
    }

    fn glare_config(glare_threshold: f64, use_glare_constraint: bool) -> SpacecraftLocationConfig {
        SpacecraftLocationConfig {
            r_polar_m: Some(1.0),
            r_lb_b_m: Vector3::new(2.0, -3.0, 1.0),
            boresight: Some(Boresight {
                a_hat_b: Vector3::new(1.0, 0.0, 0.0),
                theta_rad: PI,
            }),
            glare_threshold,
            use_glare_constraint,
            ..SpacecraftLocationConfig::new(1.0)
        }
    }

    /// Upstream `test_spacecraftLocationGlare`.
    #[test]
    fn specular_glare_geometry_and_constraint() {
        // (case, sun polar, sun azimuth, view polar, view azimuth, glare
        // factor, threshold, glare, constraint, access)
        let cases = [
            (
                "normal_incidence",
                0.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.95,
                true,
                true,
                false,
            ),
            (
                "exact_oblique",
                PI / 6.0,
                0.0,
                PI / 6.0,
                PI,
                1.0,
                0.95,
                true,
                false,
                true,
            ),
            (
                "near_miss",
                PI / 6.0,
                0.0,
                PI / 18.0,
                PI,
                (PI / 9.0).cos(),
                0.95,
                false,
                true,
                true,
            ),
            (
                "out_of_plane",
                PI / 6.0,
                0.0,
                PI / 6.0,
                PI / 2.0,
                0.75,
                0.95,
                false,
                true,
                true,
            ),
            (
                "back_facing_sun",
                2.0 * PI / 3.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.95,
                false,
                true,
                true,
            ),
            (
                "back_facing_sun_zero_threshold",
                2.0 * PI / 3.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                false,
                true,
                true,
            ),
            (
                "back_facing_view_zero_threshold",
                0.0,
                0.0,
                2.0 * PI / 3.0,
                0.0,
                0.0,
                0.0,
                false,
                true,
                true,
            ),
            (
                "threshold_boundary",
                0.0,
                0.0,
                0.95_f64.acos(),
                0.0,
                0.95,
                0.95,
                true,
                false,
                true,
            ),
        ];
        for (
            case,
            sun_polar,
            sun_azimuth,
            view_polar,
            view_azimuth,
            expected_glare_factor,
            threshold,
            expected_glare,
            use_glare_constraint,
            expected_access,
        ) in cases
        {
            let primary_position_n = Vector3::new(1.0e9, -2.0e9, 3.0e9);
            let location_position_n = primary_position_n + Vector3::new(2.0, -3.0, 1.0);
            let access = single_step(
                glare_config(threshold, use_glare_constraint),
                at(primary_position_n),
                &[location_position_n + 1.0e3 * direction_from_normal(view_polar, view_azimuth)],
                Inputs {
                    sun_position_m: Some(
                        location_position_n + 1.0e6 * direction_from_normal(sun_polar, sun_azimuth),
                    ),
                    ..Inputs::default()
                },
            )[0]
            .clone();
            assert_near(access.sun_incidence_angle_rad, sun_polar, 1e-10);
            assert_near(access.sc_view_angle_rad, view_polar, 1e-10);
            assert_near(access.glare_factor, expected_glare_factor, 1e-10);
            assert_eq!(access.has_glare, expected_glare, "{case}");
            assert_eq!(access.has_access, expected_access, "{case}");
        }
    }

    /// Upstream `test_spacecraft_location_glare_roundoff`: round-off does not
    /// move views at or just outside the glare boundaries across them.
    #[test]
    fn glare_threshold_survives_roundoff() {
        // (view offset, threshold, expected glare)
        let views = [
            (0.0, 1.0, true),
            (1.0e-6, 1.0, false),
            (0.95_f64.acos(), 0.95, true),
            (0.95_f64.acos() + 1.0e-6, 0.95, false),
        ];
        for incidence_deg in [0.0_f64, 2.0, 25.0, 43.0, 65.0, 89.0] {
            for sigma_bn in [Vector3::zeros(), Vector3::new(0.1, -0.2, 0.3)] {
                for use_glare_constraint in [false, true] {
                    for (view_offset, threshold, expected_glare) in views {
                        let incidence = incidence_deg.to_radians();
                        let mut config = glare_config(threshold, use_glare_constraint);
                        config.r_polar_m = None;
                        let primary_position_n = Vector3::new(100.0, 200.0, -300.0);
                        let dcm_nb = mrp_to_dcm(sigma_bn).transpose();
                        let location_position_n =
                            primary_position_n + dcm_nb * Vector3::new(2.0, -3.0, 1.0);
                        let sun_direction_b = Vector3::new(incidence.cos(), incidence.sin(), 0.0);
                        let view_direction_b = Vector3::new(
                            (incidence - view_offset).cos(),
                            -(incidence - view_offset).sin(),
                            0.0,
                        );
                        let access = single_step(
                            config,
                            SpacecraftStateMsg {
                                position_m: primary_position_n,
                                sigma_bn,
                                ..SpacecraftStateMsg::default()
                            },
                            &[location_position_n + 1.0e3 * (dcm_nb * view_direction_b)],
                            Inputs {
                                sun_position_m: Some(
                                    location_position_n + 1.0e6 * (dcm_nb * sun_direction_b),
                                ),
                                ..Inputs::default()
                            },
                        )[0]
                        .clone();
                        let case = format!(
                            "incidence {incidence_deg}, sigma {sigma_bn:?}, constraint \
                             {use_glare_constraint}, offset {view_offset}"
                        );
                        assert!(
                            (access.glare_factor - view_offset.cos()).abs() <= 2.0e-14,
                            "{case}: {}",
                            access.glare_factor
                        );
                        assert_eq!(access.has_glare, expected_glare, "{case}");
                        assert_eq!(
                            access.has_access,
                            !(use_glare_constraint && expected_glare),
                            "{case}"
                        );
                    }
                }
            }
        }
    }

    /// Upstream `test_spacecraft_location_glare_multiple_observers`: each
    /// output follows its own observer, and glare does not persist.
    #[test]
    fn glare_is_independent_per_observer() {
        for use_glare_constraint in [false, true] {
            for sigma_bn in [Vector3::zeros(), Vector3::new(0.1, -0.2, 0.3)] {
                let mut config = glare_config(0.95, use_glare_constraint);
                config.r_polar_m = None;
                let primary_output = Output::new(SpacecraftStateMsg {
                    position_m: Vector3::new(100.0, 200.0, -300.0),
                    sigma_bn,
                    ..SpacecraftStateMsg::default()
                });
                let dcm_nb = mrp_to_dcm(sigma_bn).transpose();
                let location_position_n =
                    Vector3::new(100.0, 200.0, -300.0) + dcm_nb * Vector3::new(2.0, -3.0, 1.0);
                let incidence = 30.0_f64.to_radians();
                let view_offset = 20.0_f64.to_radians();
                let sun_output = Output::new(SunEphemerisMsg {
                    sun_position_inertial_m: location_position_n
                        + 1.0e6 * (dcm_nb * Vector3::new(incidence.cos(), incidence.sin(), 0.0)),
                    ..SunEphemerisMsg::default()
                });
                let view_directions_b = [
                    Vector3::new(incidence.cos(), -incidence.sin(), 0.0),
                    Vector3::new(
                        (incidence + view_offset).cos(),
                        -(incidence + view_offset).sin(),
                        0.0,
                    ),
                ];
                let observer_position = |direction: usize| SpacecraftStateMsg {
                    position_m: location_position_n
                        + 1.0e3 * (dcm_nb * view_directions_b[direction]),
                    ..SpacecraftStateMsg::default()
                };
                let observers = [
                    Output::new(observer_position(0)),
                    Output::new(observer_position(1)),
                ];

                let mut module = SpacecraftLocation::new(config);
                primary_output.connect_to(&mut module.primary_sc_state_in_msg);
                sun_output.connect_to(&mut module.sun_in_msg);
                for observer in &observers {
                    let index = module.add_spacecraft_to_model();
                    observer.connect_to(&mut module.sc_state_in_msgs[index]);
                }
                module.init();
                for (step, directions) in [[0, 1], [1, 0], [1, 1]].into_iter().enumerate() {
                    for (observer, direction) in observers.iter().zip(directions) {
                        observer.write(observer_position(direction));
                    }
                    module.update(&SimulationContext {
                        current_sim_nanos: step as u64 * 1_000_000_000,
                        ..context()
                    });
                    for (output, direction) in module.access_out_msgs.iter().zip(directions) {
                        let access = output.read();
                        let expected_glare = direction == 0;
                        let expected_factor = if expected_glare {
                            1.0
                        } else {
                            view_offset.cos()
                        };
                        assert!((access.glare_factor - expected_factor).abs() <= 2.0e-14);
                        assert_eq!(access.has_glare, expected_glare);
                        assert_eq!(access.has_access, !(use_glare_constraint && expected_glare));
                    }
                }
            }
        }
    }

    /// Upstream `test_spacecraftLocationViewAngleRequirement`.
    #[test]
    fn view_angle_requirement() {
        // (view angle, limit (None = upstream's -1), connect Sun, access)
        let cases = [
            (PI / 6.0, Some(PI / 4.0), false, true),
            (PI / 3.0, Some(PI / 4.0), false, false),
            (0.0, Some(0.0), true, true),
            (PI / 3.0, Some(PI / 4.0), true, false),
            (PI / 3.0, None, false, true),
        ];
        for (view_angle, limit, connect_sun, expected_access) in cases {
            let location_position_n = Vector3::new(1.0e9, 0.0, 0.0);
            let config = SpacecraftLocationConfig {
                r_polar_m: Some(1.0),
                boresight: Some(Boresight {
                    a_hat_b: Vector3::new(1.0, 0.0, 0.0),
                    theta_rad: PI,
                }),
                theta_view_rad: limit,
                ..SpacecraftLocationConfig::new(1.0)
            };
            let access = single_step(
                config,
                at(location_position_n),
                &[location_position_n
                    + 1.0e3 * Vector3::new(view_angle.cos(), view_angle.sin(), 0.0)],
                Inputs {
                    sun_position_m: connect_sun
                        .then(|| location_position_n + Vector3::new(1.0e6, 0.0, 0.0)),
                    ..Inputs::default()
                },
            )[0]
            .clone();
            assert_near(access.sc_view_angle_rad, view_angle, 1e-10);
            assert_eq!(
                access.has_access, expected_access,
                "view angle {view_angle}"
            );
            if !connect_sun {
                assert!(!access.has_illumination);
                assert_eq!(access.glare_factor, 0.0);
                assert!(!access.has_glare);
            }
        }
    }

    /// Upstream `test_spacecraftLocationViewAngleRequiresNormal`.
    #[test]
    #[should_panic(expected = "theta_view_rad")]
    fn view_angle_requires_a_surface_normal() {
        SpacecraftLocation::new(SpacecraftLocationConfig {
            theta_view_rad: Some(PI / 4.0),
            ..SpacecraftLocationConfig::new(1.0)
        });
    }

    /// Not an upstream case: upstream's `Reset` also requires `aHat_B` for
    /// `theta_solar`.
    #[test]
    fn solar_angle_requires_a_surface_normal() {
        let config = SpacecraftLocationConfig {
            theta_solar_rad: Some(PI / 4.0),
            ..SpacecraftLocationConfig::new(1.0)
        };
        assert!(config.validate().unwrap_err().contains("theta_solar_rad"));
    }

    /// Upstream `test_spacecraftLocationIlluminationRequirements`.
    #[test]
    fn illumination_angle_and_eclipse_requirements() {
        // (case, sun polar angle, solar limit, eclipse factor, minimum
        // illumination, connect Sun, connect eclipse, illumination, access)
        let cases = [
            (
                "inside_solar_angle",
                PI / 6.0,
                Some(PI / 4.0),
                1.0,
                Some(0.5),
                true,
                false,
                true,
                true,
            ),
            (
                "outside_solar_angle",
                PI / 3.0,
                Some(PI / 4.0),
                1.0,
                Some(0.5),
                true,
                false,
                false,
                false,
            ),
            (
                "eclipse_above_threshold",
                PI / 6.0,
                Some(PI / 4.0),
                0.75,
                Some(0.5),
                true,
                true,
                true,
                true,
            ),
            (
                "eclipse_below_threshold",
                PI / 6.0,
                Some(PI / 4.0),
                0.25,
                Some(0.5),
                true,
                true,
                false,
                false,
            ),
            (
                "eclipse_optional",
                PI / 6.0,
                Some(PI / 4.0),
                0.0,
                Some(0.5),
                true,
                false,
                true,
                true,
            ),
            (
                "sun_optional",
                0.0,
                None,
                1.0,
                None,
                false,
                false,
                false,
                true,
            ),
        ];
        for (
            case,
            sun_polar,
            solar_limit,
            eclipse_factor,
            minimum_illumination,
            connect_sun,
            connect_eclipse,
            expected_illumination,
            expected_access,
        ) in cases
        {
            let location_position_n = Vector3::new(1.0e9, 0.0, 0.0);
            let config = SpacecraftLocationConfig {
                r_polar_m: Some(1.0),
                boresight: Some(Boresight {
                    a_hat_b: Vector3::new(1.0, 0.0, 0.0),
                    theta_rad: PI / 2.0,
                }),
                theta_solar_rad: solar_limit,
                min_illumination_factor: minimum_illumination,
                ..SpacecraftLocationConfig::new(1.0)
            };
            let access = single_step(
                config,
                at(location_position_n),
                &[location_position_n + Vector3::new(1.0e3, 0.0, 0.0)],
                Inputs {
                    sun_position_m: connect_sun.then(|| {
                        location_position_n
                            + 1.0e6 * Vector3::new(sun_polar.cos(), sun_polar.sin(), 0.0)
                    }),
                    eclipse_factor: connect_eclipse.then_some(eclipse_factor),
                    ..Inputs::default()
                },
            )[0]
            .clone();
            assert_eq!(access.has_illumination, expected_illumination, "{case}");
            assert_eq!(access.has_access, expected_access, "{case}");
            if connect_sun {
                assert_near(access.sun_incidence_angle_rad, sun_polar, 1e-10);
            } else {
                assert!(!access.has_glare);
                assert_eq!(access.glare_factor, 0.0);
                assert_eq!(access.sc_view_angle_rad, 0.0);
            }
        }
    }

    /// Upstream `test_spacecraftLocationGlareThresholdValidation`.
    #[test]
    fn glare_threshold_must_be_finite_in_the_unit_interval() {
        for threshold in [-0.01, 1.01, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let config = SpacecraftLocationConfig {
                glare_threshold: threshold,
                ..SpacecraftLocationConfig::new(1.0)
            };
            assert!(
                config.validate().unwrap_err().contains("glare_threshold"),
                "threshold {threshold}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "glare_threshold")]
    fn new_rejects_an_invalid_glare_threshold() {
        SpacecraftLocation::new(SpacecraftLocationConfig {
            glare_threshold: f64::NAN,
            ..SpacecraftLocationConfig::new(1.0)
        });
    }

    #[test]
    #[should_panic(expected = "r_equator_m")]
    fn new_rejects_an_unset_equatorial_radius() {
        SpacecraftLocation::new(SpacecraftLocationConfig::new(-1.0));
    }
}
