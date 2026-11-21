//! Formation reference state from a swarm of spacecraft: port of upstream
//! Basilisk `formationBarycenter`.
//!
//! Every update reads the navigation state and the mass of each spacecraft
//! and publishes one mass-weighted reference state, built in one of three
//! ways:
//!
//! * **Cartesian** (the default): the centre of mass of the positions and of
//!   the velocities, `x̄ = Σ m_i x_i / Σ m_i`.
//! * **Classical elements** (`use_orbital_elements` with
//!   [`OrbitalElementType::Classical`]): each state is converted to classical
//!   elements; `a`, `e` and `i` are averaged linearly and `Ω`, `ω` and `f` as
//!   circular means, `atan2(Σ m_i sin α_i, Σ m_i cos α_i)`, so that angles
//!   either side of zero average to zero rather than π. The averaged elements
//!   are converted back to a state. This mode is unreliable near circular or
//!   equatorial orbits, where some of the separately averaged angles are
//!   undefined.
//! * **Equinoctial elements** ([`OrbitalElementType::EquinoctialPrograde`] or
//!   [`OrbitalElementType::EquinoctialRetrograde`]): one common equinoctial
//!   representation, with retrograde factor `I = ±1`,
//!
//!   ```text
//!   P1 = e sin(ω + IΩ)    Q1 = tan(i/2)^I sin Ω    l = M + ω + IΩ
//!   P2 = e cos(ω + IΩ)    Q2 = tan(i/2)^I cos Ω
//!   ```
//!
//!   is computed directly from each state. `a`, `P1`, `P2`, `Q1` and `Q2` are
//!   averaged linearly and the mean longitude `l` as a circular mean; the
//!   state is rebuilt in the averaged equinoctial plane without recovering
//!   any classical angle, so circular and equatorial formations are fine.
//!   The prograde form is singular at `i = π` and the retrograde one at
//!   `i = 0`; the module does not switch between them. Only finite,
//!   non-radial elliptic states with nonnegative masses are accepted; anything
//!   else stops the update with a [`BarycenterError`].
//!
//! An orbital-element average is in general not the physical centre of mass.
//!
//! The module has one navigation input and one vehicle-configuration input
//! per spacecraft, made by [`FormationBarycenter::new`] from
//! [`FormationBarycenterConfig::num_spacecraft`] (upstream adds them in pairs
//! with `addSpacecraftToModel`). Upstream's C-wrapped copy of the output
//! (`transOutMsgC`) carries the same payload and is not ported.

use std::fmt;

use nalgebra::Vector3;

use crate::messages::{Input, NavigationTranslationMsg, Output, VehicleConfigMsg};
use crate::utilities::orbital_motion::{
    ClassicElements, elements_to_state, mean_to_eccentric_anomaly, state_to_elements,
};
use crate::{Module, SimulationContext};

/// Upstream's `8 * DBL_EPSILON` rounding allowance of the equinoctial guards.
const ROUNDOFF: f64 = 8.0 * f64::EPSILON;

/// The orbital-element average used when
/// [`FormationBarycenterConfig::use_orbital_elements`] is set (upstream
/// `OrbitalElementType`; the discriminants are upstream's codes).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OrbitalElementType {
    /// Classical-element average (upstream `CLASSICAL`, 0).
    #[default]
    Classical = 0,
    /// Mean-longitude equinoctial average with `I = +1`, singular at the
    /// retrograde equatorial pole (upstream `EQUINOCTIAL_PROGRADE`, 1).
    EquinoctialPrograde = 1,
    /// Mean-longitude equinoctial average with `I = -1`, singular at the
    /// prograde equatorial pole (upstream `EQUINOCTIAL_RETROGRADE`, 2).
    EquinoctialRetrograde = 2,
}

impl OrbitalElementType {
    /// The common retrograde factor `I` of an equinoctial type, `None` for
    /// the classical average.
    fn retrograde_factor(self) -> Option<f64> {
        match self {
            Self::Classical => None,
            Self::EquinoctialPrograde => Some(1.0),
            Self::EquinoctialRetrograde => Some(-1.0),
        }
    }
}

/// Why an equinoctial average stops the update. Upstream raises a
/// `BasiliskError` with the message shown by `Display`, before the output is
/// written.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BarycenterError {
    /// `mu` is not finite and positive.
    InvalidGravitationalParameter,
    /// A spacecraft mass is negative or not finite (`spacecraft` is its
    /// index).
    InvalidMass { spacecraft: usize },
    /// The masses do not add up to a finite positive total.
    InvalidTotalMass,
    /// A position or velocity component is not finite.
    NonFiniteState { spacecraft: usize },
    /// The orbit radius is zero or not finite.
    InvalidRadius { spacecraft: usize },
    /// `sqrt(mu / r)` is not a finite positive speed.
    UnrepresentableState { spacecraft: usize },
    /// The state is radial, or its binding energy is not resolvably
    /// negative (parabolic or hyperbolic).
    UnresolvableState { spacecraft: usize },
    /// The orbit normal lies at the pole the selected representation
    /// excludes.
    ExcludedPole { spacecraft: usize },
    /// The member's semi-major axis or eccentricity is not that of a finite
    /// ellipse.
    UnrepresentableElements { spacecraft: usize },
    /// The weighted mean-longitude directions cancel.
    UndefinedMeanLongitude,
    /// The averaged elements are not a finite ellipse.
    UnrepresentableAverage,
    /// The rebuilt reference state is not finite.
    NonFiniteReference,
}

impl fmt::Display for BarycenterError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let message = match self {
            Self::InvalidGravitationalParameter => {
                "equinoctial averaging requires a finite positive gravitational parameter"
            }
            Self::InvalidMass { .. } => {
                "equinoctial averaging requires finite nonnegative spacecraft mass"
            }
            Self::InvalidTotalMass => "equinoctial averaging requires finite positive total mass",
            Self::NonFiniteState { .. } => {
                "equinoctial averaging requires finite position and velocity"
            }
            Self::InvalidRadius { .. } => {
                "equinoctial averaging requires a nonzero finite orbital radius"
            }
            Self::UnrepresentableState { .. } => {
                "equinoctial averaging requires a representable elliptic state"
            }
            Self::UnresolvableState { .. } => {
                "equinoctial averaging supports only resolvable nonradial elliptic states (e < 1)"
            }
            Self::ExcludedPole { .. } => {
                "selected equinoctial representation is singular at the opposite equatorial pole"
            }
            Self::UnrepresentableElements { .. } => {
                "equinoctial averaging requires representable elliptic elements (e < 1)"
            }
            Self::UndefinedMeanLongitude => {
                "mean longitude is undefined because the weighted directions cancel"
            }
            Self::UnrepresentableAverage => {
                "averaged equinoctial elements are not a representable ellipse"
            }
            Self::NonFiniteReference => {
                "equinoctial reference is not representable as finite position and velocity"
            }
        };
        write!(f, "formationBarycenter {message}")?;
        match self {
            Self::InvalidMass { spacecraft }
            | Self::NonFiniteState { spacecraft }
            | Self::InvalidRadius { spacecraft }
            | Self::UnrepresentableState { spacecraft }
            | Self::UnresolvableState { spacecraft }
            | Self::ExcludedPole { spacecraft }
            | Self::UnrepresentableElements { spacecraft } => {
                write!(f, " (spacecraft {spacecraft})")
            }
            _ => Ok(()),
        }
    }
}

/// User-settable parameters of [`FormationBarycenter`].
#[derive(Clone, Debug)]
pub struct FormationBarycenterConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Number of spacecraft in the formation; [`FormationBarycenter::new`]
    /// makes one pair of inputs per spacecraft from it (upstream calls
    /// `addSpacecraftToModel` once per spacecraft).
    pub num_spacecraft: usize,
    /// Upstream `useOrbitalElements`: average orbital elements instead of
    /// Cartesian states (default `false`).
    pub use_orbital_elements: bool,
    /// Upstream `orbitalElementType`: which orbital-element average, used
    /// only with `use_orbital_elements` (default classical).
    pub orbital_element_type: OrbitalElementType,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central body,
    /// needed by the orbital-element averages (default 0).
    pub mu_m3ps2: f64,
}

impl Default for FormationBarycenterConfig {
    fn default() -> Self {
        Self {
            name: "formationBarycenter".to_string(),
            num_spacecraft: 0,
            use_orbital_elements: false,
            orbital_element_type: OrbitalElementType::Classical,
            mu_m3ps2: 0.0,
        }
    }
}

/// Mass-weighted formation reference state: upstream `FormationBarycenter`.
#[derive(Clone, Debug)]
pub struct FormationBarycenter {
    /// Module configuration.
    pub config: FormationBarycenterConfig,
    /// Upstream `scNavInMsgs`: navigation state of each spacecraft, one input
    /// per spacecraft.
    pub sc_nav_in_msgs: Vec<Input<NavigationTranslationMsg>>,
    /// Upstream `scPayloadInMsgs`: mass of each spacecraft (`mass_kg`), one
    /// input per spacecraft, in the order of `sc_nav_in_msgs`.
    pub sc_payload_in_msgs: Vec<Input<VehicleConfigMsg>>,
    /// Upstream `transOutMsg`: formation reference position and velocity.
    pub trans_out_msg: Output<NavigationTranslationMsg>,
}

impl FormationBarycenter {
    /// Builds the module with one unconnected navigation input and one
    /// unconnected vehicle-configuration input per configured spacecraft.
    pub fn new(config: FormationBarycenterConfig) -> Self {
        // Not `vec![_; n]`: cloning a port aliases it.
        let sc_nav_in_msgs = (0..config.num_spacecraft)
            .map(|_| Input::default())
            .collect();
        let sc_payload_in_msgs = (0..config.num_spacecraft)
            .map(|_| Input::default())
            .collect();
        Self {
            config,
            sc_nav_in_msgs,
            sc_payload_in_msgs,
            trans_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset`: reports a formation without spacecraft, missing
    /// inputs and an unusable gravitational parameter.
    ///
    /// # Panics
    ///
    /// If the number of inputs no longer matches
    /// `config.num_spacecraft`: the inputs are made from it in `new`.
    fn check_configuration(&self) {
        let num_spacecraft = self.config.num_spacecraft;
        assert!(
            self.sc_nav_in_msgs.len() == num_spacecraft
                && self.sc_payload_in_msgs.len() == num_spacecraft,
            "formationBarycenter needs one sc_nav_in_msgs and one sc_payload_in_msgs input per \
             config.num_spacecraft spacecraft"
        );
        if num_spacecraft == 0 {
            log::error!("formationBarycenter must have at least one spacecraft");
        }
        for (index, (nav, payload)) in self
            .sc_nav_in_msgs
            .iter()
            .zip(&self.sc_payload_in_msgs)
            .enumerate()
        {
            if !nav.is_connected() {
                log::error!("formationBarycenter.sc_nav_in_msgs[{index}] is not connected");
            }
            if !payload.is_connected() {
                log::error!("formationBarycenter.sc_payload_in_msgs[{index}] is not connected");
            }
        }
        if self.config.use_orbital_elements {
            if self.config.mu_m3ps2 == 0.0 {
                log::error!(
                    "formationBarycenter requires a gravitational parameter when averaging \
                     orbital elements"
                );
            }
            if self.config.orbital_element_type != OrbitalElementType::Classical
                && let Err(error) = self.check_equinoctial_mu()
            {
                log::error!("{error}");
            }
        }
    }

    /// Upstream `validateEquinoctialConfiguration`. (Its check of the element
    /// type has no counterpart: [`OrbitalElementType`] only holds supported
    /// choices.)
    fn check_equinoctial_mu(&self) -> Result<(), BarycenterError> {
        let mu = self.config.mu_m3ps2;
        if mu.is_finite() && mu > 0.0 {
            Ok(())
        } else {
            Err(BarycenterError::InvalidGravitationalParameter)
        }
    }

    /// Upstream `computeBaricenter`: the reference position \[m\] and velocity
    /// \[m/s\] of the formation from the current inputs.
    fn compute_barycenter(&self) -> Result<(Vector3<f64>, Vector3<f64>), BarycenterError> {
        let members: Vec<(NavigationTranslationMsg, f64)> = self
            .sc_nav_in_msgs
            .iter()
            .zip(&self.sc_payload_in_msgs)
            .map(|(nav, payload)| (nav.read(), payload.read().mass_kg))
            .collect();

        if !self.config.use_orbital_elements {
            return Ok(cartesian_barycenter(&members));
        }
        match self.config.orbital_element_type.retrograde_factor() {
            None => Ok(classical_barycenter(&members, self.config.mu_m3ps2)),
            Some(factor) => {
                self.check_equinoctial_mu()?;
                equinoctial_barycenter(&members, self.config.mu_m3ps2, factor)
            }
        }
    }
}

/// Mass-weighted mean of the positions and of the velocities.
fn cartesian_barycenter(
    members: &[(NavigationTranslationMsg, f64)],
) -> (Vector3<f64>, Vector3<f64>) {
    let mut position_m = Vector3::zeros();
    let mut velocity_mps = Vector3::zeros();
    let mut total_mass_kg = 0.0;
    for (nav, mass_kg) in members {
        position_m += *mass_kg * nav.position_bn_n_m;
        velocity_mps += *mass_kg * nav.velocity_bn_n_mps;
        total_mass_kg += mass_kg;
    }
    (position_m / total_mass_kg, velocity_mps / total_mass_kg)
}

/// Mass-weighted classical elements: linear means of `a`, `e` and `i`,
/// circular means of `Ω`, `ω` and `f`.
fn classical_barycenter(
    members: &[(NavigationTranslationMsg, f64)],
    mu: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let mut mean = ClassicElements::default();
    let (mut raan_sin, mut raan_cos) = (0.0, 0.0);
    let (mut arg_periapsis_sin, mut arg_periapsis_cos) = (0.0, 0.0);
    let (mut true_anomaly_sin, mut true_anomaly_cos) = (0.0, 0.0);
    let mut total_mass_kg = 0.0;
    for (nav, mass_kg) in members {
        let elements = state_to_elements(mu, nav.position_bn_n_m, nav.velocity_bn_n_mps);
        mean.semi_major_axis += mass_kg * elements.semi_major_axis;
        mean.eccentricity += mass_kg * elements.eccentricity;
        mean.inclination_rad += mass_kg * elements.inclination_rad;
        raan_sin += mass_kg * elements.raan_rad.sin();
        raan_cos += mass_kg * elements.raan_rad.cos();
        arg_periapsis_sin += mass_kg * elements.arg_periapsis_rad.sin();
        arg_periapsis_cos += mass_kg * elements.arg_periapsis_rad.cos();
        true_anomaly_sin += mass_kg * elements.true_anomaly_rad.sin();
        true_anomaly_cos += mass_kg * elements.true_anomaly_rad.cos();
        total_mass_kg += mass_kg;
    }
    mean.semi_major_axis /= total_mass_kg;
    mean.eccentricity /= total_mass_kg;
    mean.inclination_rad /= total_mass_kg;
    mean.raan_rad = raan_sin.atan2(raan_cos);
    mean.arg_periapsis_rad = arg_periapsis_sin.atan2(arg_periapsis_cos);
    mean.true_anomaly_rad = true_anomaly_sin.atan2(true_anomaly_cos);
    elements_to_state(mu, &mean)
}

/// Upstream `equinoctialPlane`: the orthonormal axes `(f̂, ĝ)` of the
/// equinoctial plane with tilt coordinates `(Q1, Q2)` and retrograde factor
/// `I`, inertial components.
fn equinoctial_plane(tilt_sin: f64, tilt_cos: f64, factor: f64) -> (Vector3<f64>, Vector3<f64>) {
    let denominator = 1.0 + tilt_sin * tilt_sin + tilt_cos * tilt_cos;
    let first_axis_n = Vector3::new(
        (1.0 - tilt_sin * tilt_sin + tilt_cos * tilt_cos) / denominator,
        2.0 * tilt_sin * tilt_cos / denominator,
        -2.0 * factor * tilt_sin / denominator,
    );
    let second_axis_n = Vector3::new(
        2.0 * factor * tilt_sin * tilt_cos / denominator,
        factor * (1.0 + tilt_sin * tilt_sin - tilt_cos * tilt_cos) / denominator,
        2.0 * tilt_cos / denominator,
    );
    (first_axis_n, second_axis_n)
}

/// Upstream `computeEquinoctialBarycenter`: the mass-weighted equinoctial
/// reference with retrograde factor `factor` (±1).
fn equinoctial_barycenter(
    members: &[(NavigationTranslationMsg, f64)],
    mu: f64,
    factor: f64,
) -> Result<(Vector3<f64>, Vector3<f64>), BarycenterError> {
    let mut total_mass_kg = 0.0;
    for (spacecraft, (_, mass_kg)) in members.iter().enumerate() {
        if !mass_kg.is_finite() || *mass_kg < 0.0 {
            return Err(BarycenterError::InvalidMass { spacecraft });
        }
        total_mass_kg += mass_kg;
    }
    if !total_mass_kg.is_finite() || total_mass_kg <= 0.0 {
        return Err(BarycenterError::InvalidTotalMass);
    }

    let mut semi_major_axis_m = 0.0;
    let (mut shape_sin, mut shape_cos) = (0.0, 0.0);
    let (mut tilt_sin, mut tilt_cos) = (0.0, 0.0);
    let (mut longitude_sin, mut longitude_cos) = (0.0, 0.0);
    for (spacecraft, (nav, mass_kg)) in members.iter().enumerate() {
        let weight = mass_kg / total_mass_kg;
        if weight == 0.0 {
            continue;
        }
        let member = equinoctial_member(nav, mu, factor, spacecraft)?;
        semi_major_axis_m += weight * member.semi_major_axis_m;
        shape_sin += weight * member.shape_sin;
        shape_cos += weight * member.shape_cos;
        tilt_sin += weight * member.tilt_sin;
        tilt_cos += weight * member.tilt_cos;
        longitude_sin += weight * member.mean_longitude_rad.sin();
        longitude_cos += weight * member.mean_longitude_rad.cos();
    }
    if longitude_sin.hypot(longitude_cos) <= 64.0 * f64::EPSILON {
        return Err(BarycenterError::UndefinedMeanLongitude);
    }
    let eccentricity = shape_sin.hypot(shape_cos);
    if !semi_major_axis_m.is_finite()
        || semi_major_axis_m <= 0.0
        || !eccentricity.is_finite()
        || eccentricity >= 1.0
    {
        return Err(BarycenterError::UnrepresentableAverage);
    }

    let mean_longitude_rad = longitude_sin.atan2(longitude_cos);
    let periapsis_longitude_rad = if eccentricity == 0.0 {
        0.0
    } else {
        shape_sin.atan2(shape_cos)
    };
    let eccentric_longitude_rad =
        mean_to_eccentric_anomaly(mean_longitude_rad - periapsis_longitude_rad, eccentricity)
            + periapsis_longitude_rad;
    let cosine = eccentric_longitude_rad.cos();
    let sine = eccentric_longitude_rad.sin();
    let beta = 1.0 / (1.0 + ((1.0 - eccentricity) * (1.0 + eccentricity)).sqrt());
    let cross_term = beta * shape_sin * shape_cos;
    let first_scale = 1.0 - beta * shape_sin * shape_sin;
    let second_scale = 1.0 - beta * shape_cos * shape_cos;
    let longitude_rate_radps = ((mu / semi_major_axis_m).sqrt() / semi_major_axis_m)
        / (1.0 - shape_cos * cosine - shape_sin * sine);
    let (first_axis_n, second_axis_n) = equinoctial_plane(tilt_sin, tilt_cos, factor);

    let position_n_m = semi_major_axis_m
        * ((first_scale * cosine + cross_term * sine - shape_cos) * first_axis_n
            + (second_scale * sine + cross_term * cosine - shape_sin) * second_axis_n);
    let velocity_n_mps = semi_major_axis_m
        * longitude_rate_radps
        * ((-first_scale * sine + cross_term * cosine) * first_axis_n
            + (second_scale * cosine - cross_term * sine) * second_axis_n);
    if position_n_m
        .iter()
        .chain(&velocity_n_mps)
        .all(|x| x.is_finite())
    {
        Ok((position_n_m, velocity_n_mps))
    } else {
        Err(BarycenterError::NonFiniteReference)
    }
}

/// One spacecraft's equinoctial elements in the common representation.
struct EquinoctialMember {
    semi_major_axis_m: f64,
    /// `P1 = e sin(ω + IΩ)`.
    shape_sin: f64,
    /// `P2 = e cos(ω + IΩ)`.
    shape_cos: f64,
    /// `Q1 = tan(i/2)^I sin Ω`.
    tilt_sin: f64,
    /// `Q2 = tan(i/2)^I cos Ω`.
    tilt_cos: f64,
    /// `l = M + ω + IΩ`.
    mean_longitude_rad: f64,
}

/// The equinoctial elements of `nav`, computed directly from the state in
/// units scaled by the radius and the local circular speed.
fn equinoctial_member(
    nav: &NavigationTranslationMsg,
    mu: f64,
    factor: f64,
    spacecraft: usize,
) -> Result<EquinoctialMember, BarycenterError> {
    let position_n_m = nav.position_bn_n_m;
    let velocity_n_mps = nav.velocity_bn_n_mps;
    if !position_n_m
        .iter()
        .chain(&velocity_n_mps)
        .all(|x| x.is_finite())
    {
        return Err(BarycenterError::NonFiniteState { spacecraft });
    }
    let radius_m = position_n_m.norm();
    if !radius_m.is_finite() || radius_m <= 0.0 {
        return Err(BarycenterError::InvalidRadius { spacecraft });
    }
    let speed_scale_mps = (mu / radius_m).sqrt();
    if !speed_scale_mps.is_finite() || speed_scale_mps <= 0.0 {
        return Err(BarycenterError::UnrepresentableState { spacecraft });
    }

    let radial_n = position_n_m / radius_m;
    let scaled_velocity_n = velocity_n_mps / speed_scale_mps;
    let speed_squared = scaled_velocity_n.dot(&scaled_velocity_n);
    let binding = 2.0 - speed_squared;
    let momentum_n = radial_n.cross(&scaled_velocity_n);
    let momentum = momentum_n.norm();
    if !binding.is_finite()
        || binding <= ROUNDOFF * speed_squared.max(2.0)
        || !momentum.is_finite()
        || momentum <= ROUNDOFF * speed_squared.sqrt()
    {
        return Err(BarycenterError::UnresolvableState { spacecraft });
    }
    let semi_major_axis_m = radius_m / binding;
    let normal_n = momentum_n / momentum;
    let transverse = normal_n.x.hypot(normal_n.y);
    let pole = factor * normal_n.z;
    if pole < 0.0 && transverse <= ROUNDOFF {
        return Err(BarycenterError::ExcludedPole { spacecraft });
    }
    // Rationalized near the excluded pole to avoid subtracting nearly equal
    // numbers.
    let tilt_scale = if pole >= 0.0 {
        1.0 / (1.0 + pole)
    } else {
        (1.0 - pole) / (transverse * transverse)
    };
    let tilt_sin = normal_n.x * tilt_scale;
    let tilt_cos = -normal_n.y * tilt_scale;
    let (first_axis_n, second_axis_n) = equinoctial_plane(tilt_sin, tilt_cos, factor);
    let eccentricity_n = scaled_velocity_n.cross(&momentum_n) - radial_n;
    let shape_sin = eccentricity_n.dot(&second_axis_n);
    let shape_cos = eccentricity_n.dot(&first_axis_n);
    let eccentricity = shape_sin.hypot(shape_cos);
    if !semi_major_axis_m.is_finite() || !eccentricity.is_finite() || eccentricity >= 1.0 {
        return Err(BarycenterError::UnrepresentableElements { spacecraft });
    }

    let true_longitude_rad = position_n_m
        .dot(&second_axis_n)
        .atan2(position_n_m.dot(&first_axis_n));
    let elliptic_factor = ((1.0 - eccentricity) * (1.0 + eccentricity)).sqrt();
    let (sin_true, cos_true) = true_longitude_rad.sin_cos();
    let eccentric_longitude_rad = true_longitude_rad
        + 2.0
            * (shape_sin * cos_true - shape_cos * sin_true)
                .atan2(1.0 + elliptic_factor + shape_cos * cos_true + shape_sin * sin_true);
    let mean_longitude_rad = eccentric_longitude_rad - shape_cos * eccentric_longitude_rad.sin()
        + shape_sin * eccentric_longitude_rad.cos();
    Ok(EquinoctialMember {
        semi_major_axis_m,
        shape_sin,
        shape_cos,
        tilt_sin,
        tilt_cos,
        mean_longitude_rad,
    })
}

impl Module for FormationBarycenter {
    fn init(&mut self) {
        self.trans_out_msg
            .write(NavigationTranslationMsg::default());
        self.check_configuration();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_configuration();
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.compute_barycenter() {
            // The rest of the payload stays zero, as upstream's zeroed buffer.
            Ok((position_bn_n_m, velocity_bn_n_mps)) => {
                self.trans_out_msg.write(NavigationTranslationMsg {
                    position_bn_n_m,
                    velocity_bn_n_mps,
                    ..Default::default()
                })
            }
            // Upstream throws before writing: the output keeps its previous
            // value.
            Err(error) => log::error!("{error}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3, Vector6};

    use super::{
        BarycenterError, FormationBarycenter, FormationBarycenterConfig, OrbitalElementType,
    };
    use crate::fsw_algorithms::formation_flying::mean_oe_feedback::test_orbits::python_elem2rv;
    use crate::messages::{NavigationTranslationMsg, Output, VehicleConfigMsg};
    use crate::utilities::astro_constants::{MU_EARTH, REQ_EARTH};
    use crate::utilities::orbital_motion::{
        ClassicElements, eccentric_to_mean_anomaly, hill_to_state, state_to_elements,
        true_to_eccentric_anomaly,
    };
    use crate::{Module, SimulationContext};

    use OrbitalElementType::{Classical, EquinoctialPrograde, EquinoctialRetrograde};

    /// `test_formationBarycenter_equinoctial.py`'s `MU` \[m^3/s^2\].
    const MU: f64 = 3.986004418e14;
    const EQUINOCTIAL: [OrbitalElementType; 2] = [EquinoctialPrograde, EquinoctialRetrograde];

    /// Inertial position \[m\] and velocity \[m/s\].
    type State = (Vector3<f64>, Vector3<f64>);
    /// Semi-major axis \[m\], eccentricity, inclination, ascending node,
    /// argument of periapsis and mean anomaly \[rad\].
    type Orbit = (f64, f64, f64, f64, f64, f64);
    /// Member orbits, masses \[kg\], reference state and case name of
    /// `test_classical_compatibility`.
    type ClassicalCase = ([Orbit; 3], [f64; 3], [f64; 6], &'static str);

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn factor(element_type: OrbitalElementType) -> f64 {
        element_type
            .retrograde_factor()
            .expect("an equinoctial element type")
    }

    fn concat(state: State) -> Vector6<f64> {
        Vector6::new(
            state.0.x, state.0.y, state.0.z, state.1.x, state.1.y, state.1.z,
        )
    }

    /// The pytest's `classical_state`: an elliptic state from mean elements,
    /// solving Kepler's equation by bisection, independently of the module.
    fn classical_state(orbit: Orbit) -> State {
        let (axis, eccentricity, inclination, node, periapsis, mean_anomaly) = orbit;
        let mut lower = mean_anomaly - eccentricity;
        let mut upper = mean_anomaly + eccentricity;
        let mut eccentric_anomaly = 0.0;
        for _ in 0..100 {
            eccentric_anomaly = (lower + upper) / 2.0;
            if eccentric_anomaly == lower || eccentric_anomaly == upper {
                break;
            }
            if eccentric_anomaly - eccentricity * eccentric_anomaly.sin() > mean_anomaly {
                upper = eccentric_anomaly;
            } else {
                lower = eccentric_anomaly;
            }
        }
        let cosine = eccentric_anomaly.cos();
        let sine = eccentric_anomaly.sin();
        let beta = (1.0 - eccentricity * eccentricity).sqrt();
        let radius = axis * (1.0 - eccentricity * cosine);
        let position = axis * Vector3::new(cosine - eccentricity, beta * sine, 0.0);
        let velocity = (MU * axis).sqrt() / radius * Vector3::new(-sine, beta * cosine, 0.0);
        let (co, so) = (node.cos(), node.sin());
        let (cw, sw) = (periapsis.cos(), periapsis.sin());
        let (ci, si) = (inclination.cos(), inclination.sin());
        let rotation = Matrix3::new(
            co * cw - so * sw * ci,
            -co * sw - so * cw * ci,
            so * si,
            so * cw + co * sw * ci,
            -so * sw + co * cw * ci,
            -co * si,
            sw * si,
            cw * si,
            ci,
        );
        (rotation * position, rotation * velocity)
    }

    /// The pytest's `weighted_reference`: averages known elements and
    /// rebuilds the state through [`classical_state`].
    fn weighted_reference(orbits: &[Orbit], masses: &[f64], factor: f64) -> Vector6<f64> {
        let total: f64 = masses.iter().sum();
        let mut averaged = [0.0; 5];
        let (mut sine_sum, mut cosine_sum) = (0.0, 0.0);
        for (&mass, &(axis, eccentricity, inclination, node, periapsis, mean_anomaly)) in
            masses.iter().zip(orbits)
        {
            let weight = mass / total;
            let longitude_periapsis = periapsis + factor * node;
            let tilt = if factor == 1.0 {
                (inclination / 2.0).tan()
            } else {
                1.0 / (inclination / 2.0).tan()
            };
            let terms = [
                axis,
                eccentricity * longitude_periapsis.sin(),
                eccentricity * longitude_periapsis.cos(),
                tilt * node.sin(),
                tilt * node.cos(),
            ];
            for (sum, term) in averaged.iter_mut().zip(terms) {
                *sum += weight * term;
            }
            let longitude = mean_anomaly + longitude_periapsis;
            sine_sum += weight * longitude.sin();
            cosine_sum += weight * longitude.cos();
        }
        let [axis, shape_sine, shape_cosine, tilt_sine, tilt_cosine] = averaged;
        let eccentricity = shape_sine.hypot(shape_cosine);
        let tilt = tilt_sine.hypot(tilt_cosine);
        let inclination = if factor == 1.0 {
            2.0 * tilt.atan()
        } else {
            2.0 * 1.0_f64.atan2(tilt)
        };
        let node = tilt_sine.atan2(tilt_cosine);
        let longitude_periapsis = shape_sine.atan2(shape_cosine);
        let periapsis = longitude_periapsis - factor * node;
        let mean_anomaly = sine_sum.atan2(cosine_sum) - longitude_periapsis;
        concat(classical_state((
            axis,
            eccentricity,
            inclination,
            node,
            periapsis,
            mean_anomaly,
        )))
    }

    /// The module with its inputs connected to writable outputs (the
    /// pytest's `make_simulation`), initialized.
    struct Fixture {
        module: FormationBarycenter,
        navigation: Vec<Output<NavigationTranslationMsg>>,
        vehicles: Vec<Output<VehicleConfigMsg>>,
    }

    fn navigation(state: State) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            position_bn_n_m: state.0,
            velocity_bn_n_mps: state.1,
            ..Default::default()
        }
    }

    fn vehicle(mass_kg: f64) -> VehicleConfigMsg {
        VehicleConfigMsg {
            mass_kg,
            ..Default::default()
        }
    }

    fn fixture(
        states: &[State],
        masses: &[f64],
        element_type: Option<OrbitalElementType>,
        use_orbital_elements: bool,
        mu: f64,
    ) -> Fixture {
        assert_eq!(states.len(), masses.len());
        let mut config = FormationBarycenterConfig {
            num_spacecraft: states.len(),
            use_orbital_elements,
            mu_m3ps2: mu,
            ..Default::default()
        };
        if let Some(element_type) = element_type {
            config.orbital_element_type = element_type;
        }
        let mut module = FormationBarycenter::new(config);
        let navigation: Vec<_> = states.iter().map(|s| Output::new(navigation(*s))).collect();
        let vehicles: Vec<_> = masses.iter().map(|m| Output::new(vehicle(*m))).collect();
        for (output, input) in navigation.iter().zip(&mut module.sc_nav_in_msgs) {
            output.connect_to(input);
        }
        for (output, input) in vehicles.iter().zip(&mut module.sc_payload_in_msgs) {
            output.connect_to(input);
        }
        module.init();
        Fixture {
            module,
            navigation,
            vehicles,
        }
    }

    /// One update; on success the published state, which must equal what
    /// the computation returned (upstream's `recorded_state`, which also
    /// requires a finite result).
    fn step(module: &mut FormationBarycenter, nanos: u64) -> Result<Vector6<f64>, BarycenterError> {
        let result = module.compute_barycenter();
        module.update(&context(nanos));
        let state = result?;
        let output = module.trans_out_msg.read();
        assert_eq!((output.position_bn_n_m, output.velocity_bn_n_mps), state);
        let state = concat(state);
        assert!(state.iter().all(|x| x.is_finite()));
        Ok(state)
    }

    /// The pytest's `run_module` without `after_reset`.
    fn run_module(
        states: &[State],
        masses: &[f64],
        element_type: Option<OrbitalElementType>,
        mu: f64,
    ) -> Result<Vector6<f64>, BarycenterError> {
        step(
            &mut fixture(states, masses, element_type, true, mu).module,
            0,
        )
    }

    fn run_equinoctial(
        states: &[State],
        masses: &[f64],
        element_type: OrbitalElementType,
    ) -> Vector6<f64> {
        run_module(states, masses, Some(element_type), MU).expect("a representable reference")
    }

    /// `np.testing.assert_allclose(actual, expected, rtol, atol)` on each
    /// half of the state.
    fn assert_close(
        actual: &Vector6<f64>,
        expected: &Vector6<f64>,
        rtol: f64,
        atol: [f64; 2],
        what: &str,
    ) {
        for (index, (a, e)) in actual.iter().zip(expected.iter()).enumerate() {
            let tolerance = atol[index / 3] + rtol * e.abs();
            assert!(
                (a - e).abs() <= tolerance,
                "{what}: component {index}: {a:.17e} vs {e:.17e}"
            );
        }
    }

    /// The pytest's `assert_state`: 2 µm in position and 2e-8 m/s in
    /// velocity, plus `rtol`.
    fn assert_state(actual: &Vector6<f64>, expected: &Vector6<f64>, rtol: f64, what: &str) {
        assert_close(actual, expected, rtol, [2e-6, 2e-8], what);
    }

    const DEFAULT_RTOL: f64 = 2e-11;

    fn expect_error(result: Result<Vector6<f64>, BarycenterError>, pattern: &str) {
        match result {
            Ok(state) => panic!("expected an error matching {pattern:?}, got {state:?}"),
            Err(error) => assert!(
                error.to_string().contains(pattern),
                "{error} does not match {pattern:?}"
            ),
        }
    }

    /// Upstream `test_formationBarycenter.py`: three spacecraft of 100, 150
    /// and 250 kg, a Cartesian update and then an orbital-element update once
    /// `useOrbitalElements` and `mu` are set, against the hard-coded truth
    /// with the absolute accuracy 1e-1. (The C-wrapped output upstream also
    /// checks carries the same payload and is not ported.)
    #[test]
    fn matches_upstream_formation_barycenter_unit_test() {
        let mu = MU_EARTH * 1e9;
        let oe1 = ClassicElements {
            semi_major_axis: 1.1 * REQ_EARTH * 1e3,
            eccentricity: 0.01,
            inclination_rad: 45.0_f64.to_radians(),
            raan_rad: 48.2_f64.to_radians(),
            arg_periapsis_rad: 347.8_f64.to_radians(),
            true_anomaly_rad: 85.3_f64.to_radians(),
            ..Default::default()
        };
        let oe2 = ClassicElements {
            eccentricity: 1.05 * oe1.eccentricity,
            inclination_rad: 1.05 * oe1.inclination_rad,
            true_anomaly_rad: 1.05 * oe1.true_anomaly_rad,
            ..oe1
        };
        let oe3 = ClassicElements {
            eccentricity: 0.95 * oe1.eccentricity,
            inclination_rad: 0.90 * oe1.inclination_rad,
            true_anomaly_rad: 1.10 * oe1.true_anomaly_rad,
            ..oe1
        };
        let states: Vec<State> = [oe1, oe2, oe3]
            .iter()
            .map(|oe| python_elem2rv(mu, oe))
            .collect();
        let mut fixture = fixture(&states, &[100.0, 150.0, 250.0], None, false, 0.0);
        let cartesian = step(&mut fixture.module, 0).expect("a Cartesian barycenter");
        fixture.module.config.use_orbital_elements = true;
        fixture.module.config.mu_m3ps2 = mu;
        let orbital = step(&mut fixture.module, 1_000_000_000).expect("an element barycenter");

        let accuracy = 1e-1;
        let true_barycenter = [
            -2795611.0423523,
            4349073.25701624,
            4711567.73659507,
            -5738.71806601,
            -4744.64063227,
            1079.61501999,
        ];
        for (index, (value, truth)) in cartesian.iter().zip(true_barycenter).enumerate() {
            assert!(
                (value - truth).abs() <= accuracy,
                "Cartesian component {index}: {value} vs {truth}"
            );
        }
        let elements = state_to_elements(
            mu,
            orbital.fixed_rows::<3>(0).into(),
            orbital.fixed_rows::<3>(3).into(),
        );
        let actual = [
            elements.semi_major_axis,
            elements.eccentricity,
            elements.inclination_rad,
            elements.raan_rad,
            elements.arg_periapsis_rad,
            elements.true_anomaly_rad,
        ];
        let true_elements = [
            7015950.259999997,
            0.0099,
            0.7579092276785376,
            0.8412486994612671,
            6.07025513843626,
            1.5855546253383273,
        ];
        for (index, (value, truth)) in actual.iter().zip(true_elements).enumerate() {
            assert!(
                (value - truth).abs() <= accuracy,
                "element {index}: {value} vs {truth}"
            );
        }
    }

    /// Upstream `test_default_and_named_choices`: Cartesian averaging and the
    /// classical selector by default, and upstream's selector codes.
    #[test]
    fn defaults_to_cartesian_and_keeps_upstream_selector_codes() {
        let config = FormationBarycenterConfig::default();
        assert!(!config.use_orbital_elements);
        assert_eq!(config.orbital_element_type, Classical);
        assert_eq!(OrbitalElementType::default(), Classical);
        assert_eq!(Classical as i32, 0);
        assert_eq!(EquinoctialPrograde as i32, 1);
        assert_eq!(EquinoctialRetrograde as i32, 2);
    }

    /// Upstream `test_cartesian_ignores_element_selector`: with
    /// `useOrbitalElements` off, every selector gives the same Cartesian
    /// mean, within 8 epsilon of the weighted average. (Upstream's invalid
    /// selector `-1` cannot be expressed.)
    #[test]
    fn cartesian_average_ignores_the_element_selector() {
        let states: Vec<State> = [0.7, 0.9]
            .iter()
            .map(|&phase| classical_state((7e6, 0.1, 0.9, 0.3, 0.4, phase)))
            .collect();
        let masses = [100.0, 300.0];
        let total: f64 = masses.iter().sum();
        let expected = states
            .iter()
            .zip(masses)
            .map(|(state, mass)| concat(*state) * mass)
            .sum::<Vector6<f64>>()
            / total;
        let baseline = step(&mut fixture(&states, &masses, None, false, 0.0).module, 0).unwrap();
        for element_type in [Classical, EquinoctialPrograde, EquinoctialRetrograde] {
            let actual = step(
                &mut fixture(&states, &masses, Some(element_type), false, 0.0).module,
                0,
            )
            .unwrap();
            assert_eq!(actual, baseline, "{element_type:?}");
            assert_close(
                &actual,
                &expected,
                8.0 * f64::EPSILON,
                [0.0, 0.0],
                "Cartesian",
            );
        }
    }

    /// Upstream `test_classical_compatibility`: the classical average must
    /// reproduce the states captured from the module before the equinoctial
    /// choices were added (2 µm, 2e-8 m/s), with the selector omitted, set
    /// to classical explicitly, or enabled after a Cartesian update.
    #[test]
    fn classical_average_matches_upstream_reference_states() {
        let cases: [ClassicalCase; 4] = [
            (
                [
                    (7e6, 0.1, 0.5, 0.2, 0.4, 0.6),
                    (8e6, 0.2, 0.7, 0.3, 0.5, 0.8),
                    (9e6, 0.3, 1.0, 0.4, 0.6, 1.0),
                ],
                [100.0, 200.0, 400.0],
                [
                    -3805024.012545443,
                    3691119.6983278044,
                    5336085.789998154,
                    -7017.942535029498,
                    -3023.7993314431715,
                    -547.6782626053625,
                ],
                "weighted-inclined",
            ),
            (
                [
                    (7e6, 0.1, 0.9, 6.20, 6.15, 6.24),
                    (7.2e6, 0.15, 0.8, 0.04, 0.08, 0.03),
                    (6.8e6, 0.05, 1.0, 0.12, 0.20, 0.10),
                ],
                [100.0, 200.0, 300.0],
                [
                    6194409.791060193,
                    1002729.1491828125,
                    824797.744204107,
                    -1614.2214430779236,
                    4893.206053782084,
                    6495.775280548105,
                ],
                "angle-wrap",
            ),
            (
                [
                    (2e7, 0.6, 2.5, 2.8, 0.5, 0.3),
                    (2.1e7, 0.7, 2.6, 2.9, 0.6, 0.6),
                    (1.9e7, 0.8, 2.7, 3.0, 0.7, 1.2),
                ],
                [400.0, 100.0, 200.0],
                [
                    10068721.357163044,
                    5134873.813868816,
                    4896469.296928595,
                    6254.6602691876315,
                    -2322.4582653040334,
                    -364.5000674161748,
                ],
                "eccentric-retrograde",
            ),
            (
                [
                    (7e6, 0.01, 0.78, 0.84, 6.07, 1.48),
                    (7e6, 0.0105, 0.82, 0.84, 6.07, 1.56),
                    (7e6, 0.0095, 0.70, 0.84, 6.07, 1.63),
                ],
                [100.0, 150.0, 250.0],
                [
                    -2881895.08515755,
                    4314497.779819102,
                    4700781.108437926,
                    -5735.687341594877,
                    -4798.5621701086875,
                    999.1089719904237,
                ],
                "low-eccentricity",
            ),
        ];
        for (orbits, masses, expected, id) in cases {
            let states: Vec<State> = orbits.iter().map(|o| classical_state(*o)).collect();
            let expected = Vector6::from(expected);
            for configuration in ["default", "explicit-classical", "legacy-runtime"] {
                let actual = if configuration == "legacy-runtime" {
                    let mut fixture = fixture(&states, &masses, None, false, 0.0);
                    step(&mut fixture.module, 0).unwrap();
                    fixture.module.config.use_orbital_elements = true;
                    fixture.module.config.mu_m3ps2 = MU;
                    step(&mut fixture.module, 1_000_000_000).unwrap()
                } else {
                    let element_type = (configuration == "explicit-classical").then_some(Classical);
                    run_module(&states, &masses, element_type, MU).unwrap()
                };
                assert_state(&actual, &expected, 0.0, &format!("{id} {configuration}"));
            }
        }
    }

    /// Upstream `test_single_spacecraft_identity`: a single spacecraft is its
    /// own reference, for every eccentricity, tilt (including each
    /// representation's own pole) and phase.
    #[test]
    fn a_single_spacecraft_is_its_own_reference() {
        for element_type in EQUINOCTIAL {
            let inclinations = if element_type == EquinoctialPrograde {
                [0.0, 1e-12, 0.9, 2.8]
            } else {
                [0.2, 0.9, PI - 1e-12, PI]
            };
            for eccentricity in [0.0, 1e-8, 0.2, 0.7, 0.95] {
                for inclination in inclinations {
                    for mean_anomaly in [0.0, 0.7, 3.4, 6.1] {
                        let state = classical_state((
                            2e7,
                            eccentricity,
                            inclination,
                            0.3,
                            1.2,
                            mean_anomaly,
                        ));
                        assert_state(
                            &run_equinoctial(&[state], &[100.0], element_type),
                            &concat(state),
                            DEFAULT_RTOL,
                            &format!(
                                "{element_type:?} e {eccentricity} i {inclination} M {mean_anomaly}"
                            ),
                        );
                    }
                }
            }
        }
    }

    /// Upstream `test_reported_compact_formation`: 100 m formations about a
    /// circular chief (upstream issue 1656) match the independent reference
    /// exactly (2 µm) and stay within 1 cm of the Cartesian centre.
    #[test]
    fn compact_formations_match_the_reference_and_the_cartesian_centre() {
        let radius = 7e6;
        let spread = 100.0;
        let chief = ClassicElements {
            semi_major_axis: radius,
            eccentricity: 0.0,
            inclination_rad: 0.9,
            raan_rad: 0.3,
            arg_periapsis_rad: 0.0,
            true_anomaly_rad: 1.0,
            ..Default::default()
        };
        let (rc, vc) = python_elem2rv(MU, &chief);
        let rate = (MU / radius.powi(3)).sqrt();
        let phase_sets: [&[f64]; 6] = [
            &[0.0, 120.0, 240.0],
            &[0.0, 120.1, 240.0],
            &[0.0, 121.0, 240.0],
            &[0.0, 125.0, 240.0],
            &[0.0, 140.0, 240.0],
            &[0.0, 90.0, 180.0, 270.0],
        ];
        for phases in phase_sets {
            for element_type in EQUINOCTIAL {
                let states: Vec<State> = phases
                    .iter()
                    .map(|degrees| {
                        let phase = degrees.to_radians();
                        let offset =
                            spread * Vector3::new(0.5 * phase.cos(), -phase.sin(), phase.cos());
                        let offset_rate = spread
                            * rate
                            * Vector3::new(-0.5 * phase.sin(), -phase.cos(), -phase.sin());
                        hill_to_state(rc, vc, offset, offset_rate)
                    })
                    .collect();
                let cartesian =
                    states.iter().map(|s| s.0).sum::<Vector3<f64>>() / states.len() as f64;
                let masses = vec![100.0; states.len()];
                let orbits: Vec<Orbit> = states
                    .iter()
                    .map(|(position, velocity)| {
                        let elements = state_to_elements(MU, *position, *velocity);
                        assert!(
                            elements.eccentricity > 1e-7
                                && 0.1 < elements.inclination_rad
                                && elements.inclination_rad < PI - 0.1
                        );
                        let mean_anomaly = eccentric_to_mean_anomaly(
                            true_to_eccentric_anomaly(
                                elements.true_anomaly_rad,
                                elements.eccentricity,
                            ),
                            elements.eccentricity,
                        );
                        (
                            elements.semi_major_axis,
                            elements.eccentricity,
                            elements.inclination_rad,
                            elements.raan_rad,
                            elements.arg_periapsis_rad,
                            mean_anomaly,
                        )
                    })
                    .collect();
                let expected = weighted_reference(&orbits, &masses, factor(element_type));
                let actual = run_equinoctial(&states, &masses, element_type);
                let what = format!("{phases:?} {element_type:?}");
                assert_state(&actual, &expected, 0.0, &what);
                let locality_budget = 0.01;
                assert!((expected.fixed_rows::<3>(0) - cartesian).norm() < locality_budget);
                assert!(
                    (actual.fixed_rows::<3>(0) - cartesian).norm() < locality_budget,
                    "{what}"
                );
            }
        }
    }

    /// Upstream `test_equal_axis_preserves_orbital_clock`: two equal masses
    /// on one ellipse, symmetric in mean anomaly, average to the midpoint.
    #[test]
    fn equal_axes_keep_the_orbital_clock() {
        let axis: f64 = 2e7;
        for element_type in EQUINOCTIAL {
            for elapsed in [0.0, 60.0, 200.0] {
                let inclination = if element_type == EquinoctialPrograde {
                    0.9
                } else {
                    PI
                };
                let mean_midpoint = 1.1 + (MU / axis.powi(3)).sqrt() * elapsed;
                let states: Vec<State> = [-0.2, 0.2]
                    .iter()
                    .map(|delta| {
                        classical_state((axis, 0.6, inclination, 0.3, 1.2, mean_midpoint + delta))
                    })
                    .collect();
                let expected = classical_state((axis, 0.6, inclination, 0.3, 1.2, mean_midpoint));
                assert_state(
                    &run_equinoctial(&states, &[100.0, 100.0], element_type),
                    &concat(expected),
                    DEFAULT_RTOL,
                    &format!("{element_type:?} {elapsed}"),
                );
            }
        }
    }

    /// Upstream `test_equatorial_formation`: unequal members at or within
    /// 1e-10 rad of the representation's equator, circular, near-circular and
    /// eccentric.
    #[test]
    fn equatorial_formations_match_the_reference() {
        for element_type in EQUINOCTIAL {
            let factor = factor(element_type);
            for equator_offset in [0.0, 1e-10] {
                for eccentricity in [0.0, 1e-8, 0.2] {
                    let inclination = if factor == 1.0 {
                        equator_offset
                    } else {
                        PI - equator_offset
                    };
                    let orbits = [
                        (
                            7e6,
                            0.5 * eccentricity,
                            inclination,
                            0.1,
                            0.4 - factor * 0.1,
                            0.2,
                        ),
                        (
                            7.1e6,
                            eccentricity,
                            inclination,
                            2.2,
                            0.5 - factor * 2.2,
                            0.3,
                        ),
                        (
                            6.9e6,
                            1.5 * eccentricity,
                            inclination,
                            4.3,
                            0.3 - factor * 4.3,
                            0.4,
                        ),
                    ];
                    let masses = [100.0, 200.0, 400.0];
                    let states: Vec<State> = orbits.iter().map(|o| classical_state(*o)).collect();
                    assert_state(
                        &run_equinoctial(&states, &masses, element_type),
                        &weighted_reference(&orbits, &masses, factor),
                        0.0,
                        &format!("{element_type:?} offset {equator_offset} e {eccentricity}"),
                    );
                }
            }
        }
    }

    /// Upstream `test_equinoctial_component_cancellation`: masses 1:3 with
    /// opposite eccentricity and/or tilt components of magnitude 3:1 average
    /// to a circular and/or equatorial orbit known analytically.
    #[test]
    fn cancelling_components_give_a_circular_or_equatorial_mean() {
        for element_type in EQUINOCTIAL {
            let factor = factor(element_type);
            for cancelled in ["eccentricity", "inclination", "both"] {
                let cancel_eccentricity = cancelled != "inclination";
                let cancel_inclination = cancelled != "eccentricity";
                let axes = [7e6, 9e6];
                let masses = [100.0, 300.0];
                let eccentricities = if cancel_eccentricity {
                    [0.3, 0.1]
                } else {
                    [0.2, 0.2]
                };
                let periapsis_longitudes = if cancel_eccentricity {
                    [0.7, 0.7 + PI]
                } else {
                    [0.7, 0.7]
                };
                let tilts = if cancel_inclination {
                    [0.3, 0.1]
                } else {
                    [0.1, 0.1]
                };
                let nodes = if cancel_inclination {
                    [0.4, 0.4 + PI]
                } else {
                    [0.4, 0.4]
                };
                let mean_longitude = 0.8;
                let inclination_of = |tilt: f64| {
                    if factor == 1.0 {
                        2.0 * tilt.atan()
                    } else {
                        2.0 * 1.0_f64.atan2(tilt)
                    }
                };
                let states: Vec<State> = (0..2)
                    .map(|k| {
                        classical_state((
                            axes[k],
                            eccentricities[k],
                            inclination_of(tilts[k]),
                            nodes[k],
                            periapsis_longitudes[k] - factor * nodes[k],
                            mean_longitude - periapsis_longitudes[k],
                        ))
                    })
                    .collect();

                let mean_eccentricity = if cancel_eccentricity { 0.0 } else { 0.2 };
                let mean_tilt = if cancel_inclination { 0.0 } else { 0.1 };
                let mean_node = if cancel_inclination { 0.0 } else { 0.4 };
                let mean_periapsis_longitude = if cancel_eccentricity { 0.0 } else { 0.7 };
                let expected = classical_state((
                    8.5e6,
                    mean_eccentricity,
                    inclination_of(mean_tilt),
                    mean_node,
                    mean_periapsis_longitude - factor * mean_node,
                    mean_longitude - mean_periapsis_longitude,
                ));
                assert_state(
                    &run_equinoctial(&states, &masses, element_type),
                    &concat(expected),
                    0.0,
                    &format!("{element_type:?} {cancelled}"),
                );
            }
        }
    }

    /// Upstream `test_weighting_wrap_and_reordering`: unequal members across
    /// the angle wrap match the reference, and neither reordering the
    /// members nor scaling every mass changes the result.
    #[test]
    fn weighting_is_order_and_scale_invariant() {
        for element_type in EQUINOCTIAL {
            let factor = factor(element_type);
            let mut orbits = vec![
                (2e7, 0.2, 0.6, 0.1, 0.2, 6.0),
                (2.1e7, 0.4, 0.8, 0.2, 0.1, 0.0),
                (1.9e7, 0.1, 1.0, 0.3, 0.0, 0.2),
            ];
            if factor == -1.0 {
                for orbit in &mut orbits {
                    orbit.2 = PI - orbit.2;
                }
            }
            let masses = [100.0, 200.0, 400.0];
            let states: Vec<State> = orbits.iter().map(|o| classical_state(*o)).collect();
            let actual = run_equinoctial(&states, &masses, element_type);
            let what = format!("{element_type:?}");
            assert_state(
                &actual,
                &weighted_reference(&orbits, &masses, factor),
                DEFAULT_RTOL,
                &what,
            );
            let reversed_states: Vec<State> = states.iter().rev().copied().collect();
            let reversed_masses: Vec<f64> = masses.iter().rev().copied().collect();
            assert_state(
                &run_equinoctial(&reversed_states, &reversed_masses, element_type),
                &actual,
                DEFAULT_RTOL,
                &what,
            );
            let scaled_masses = masses.map(|m| m * 7.0);
            assert_state(
                &run_equinoctial(&states, &scaled_masses, element_type),
                &actual,
                DEFAULT_RTOL,
                &what,
            );
        }
    }

    /// Upstream `test_multiple_updates_refresh_navigation_and_mass`: six
    /// updates, the members advancing at their common mean motion and the
    /// masses changing halfway, each matching a fresh reference.
    #[test]
    fn every_update_reads_fresh_navigation_and_masses() {
        let axis: f64 = 2e7;
        let rate = (MU / axis.powi(3)).sqrt();
        for element_type in EQUINOCTIAL {
            let factor = factor(element_type);
            let mut orbits = vec![
                (axis, 0.2, 0.6, 0.1, 0.2, 0.8),
                (axis, 0.4, 0.8, 0.2, 0.1, 1.0),
                (axis, 0.6, 1.0, 0.3, 0.0, 1.2),
            ];
            if factor == -1.0 {
                for orbit in &mut orbits {
                    orbit.2 = PI - orbit.2;
                }
            }
            let mut masses = [100.0, 200.0, 400.0];
            let states: Vec<State> = orbits.iter().map(|o| classical_state(*o)).collect();
            let mut fixture = fixture(&states, &masses, Some(element_type), true, MU);
            assert_eq!(fixture.module.config.orbital_element_type, element_type);
            for step_index in 0..6_u64 {
                let elapsed = step_index as f64;
                let current: Vec<Orbit> = orbits
                    .iter()
                    .map(|&(a, e, i, node, periapsis, phase)| {
                        (a, e, i, node, periapsis, phase + rate * elapsed)
                    })
                    .collect();
                if step_index == 3 {
                    masses = [400.0, 100.0, 200.0];
                }
                for (index, (orbit, mass)) in current.iter().zip(masses).enumerate() {
                    fixture.navigation[index].write(navigation(classical_state(*orbit)));
                    fixture.vehicles[index].write(vehicle(mass));
                }
                let actual = step(&mut fixture.module, step_index * 1_000_000_000).unwrap();
                assert_state(
                    &actual,
                    &weighted_reference(&current, &masses, factor),
                    DEFAULT_RTOL,
                    &format!("{element_type:?} step {step_index}"),
                );
            }
        }
    }

    /// Upstream `test_binding_energy_cutoff`: a unit-scale state 8 epsilon
    /// from parabolic is rejected, one 64 epsilon away accepted.
    #[test]
    fn rejects_binding_energy_below_the_cutoff() {
        let epsilon = f64::EPSILON;
        for element_type in EQUINOCTIAL {
            for (binding_eps, accepted) in [(8.0, false), (64.0, true)] {
                let speed = (2.0 - binding_eps * epsilon).sqrt();
                let state = (
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, factor(element_type) * speed, 0.0),
                );
                let binding = 2.0 - speed * speed;
                let cutoff = 8.0 * epsilon * (speed * speed).max(2.0);
                assert_eq!(binding > cutoff, accepted);
                let result = run_module(&[state], &[1.0], Some(element_type), 1.0);
                if accepted {
                    assert!(result.is_ok(), "{element_type:?}: {result:?}");
                } else {
                    expect_error(result, "resolvable nonradial");
                }
            }
        }
    }

    /// Upstream `test_near_radial_cutoff`: almost radial unit states are
    /// rejected by the radial guard at or below its cutoff and by the
    /// elliptic-elements guard above it.
    #[test]
    fn rejects_near_radial_states() {
        let epsilon = f64::EPSILON;
        for element_type in EQUINOCTIAL {
            for (transverse_eps, first_guard) in [(4.0, true), (8.0, true), (16.0, false)] {
                let transverse_speed = transverse_eps * epsilon;
                let state = (
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(1.0, factor(element_type) * transverse_speed, 0.0),
                );
                assert_eq!(
                    transverse_speed <= 8.0 * epsilon * (1.0 + transverse_speed.powi(2)).sqrt(),
                    first_guard
                );
                let pattern = if first_guard {
                    "resolvable nonradial"
                } else {
                    "representable elliptic elements"
                };
                expect_error(
                    run_module(&[state], &[1.0], Some(element_type), 1.0),
                    pattern,
                );
            }
        }
    }

    /// Upstream `test_excluded_pole_cutoff`: an orbit normal within 8
    /// epsilon of the excluded pole is rejected; one 16 epsilon away is
    /// recovered to 64 epsilon.
    #[test]
    fn rejects_the_excluded_pole_and_recovers_just_outside_it() {
        let epsilon = f64::EPSILON;
        for element_type in EQUINOCTIAL {
            for (tilt_eps, accepted) in [(4.0, false), (8.0, false), (16.0, true)] {
                let transverse_speed = tilt_eps * epsilon;
                let state = (
                    Vector3::new(1.0, 0.0, 0.0),
                    Vector3::new(0.0, -factor(element_type), transverse_speed),
                );
                assert_eq!(transverse_speed > 8.0 * epsilon, accepted);
                let result = run_module(&[state], &[1.0], Some(element_type), 1.0);
                if accepted {
                    let atol = 64.0 * epsilon;
                    assert_close(&result.unwrap(), &concat(state), 0.0, [atol, atol], "pole");
                } else {
                    expect_error(result, "pole");
                }
            }
        }
    }

    /// Upstream `test_longitude_cancellation_cutoff`: equal near-opposite
    /// phases are rejected when the longitude resultant is at most 64
    /// epsilon and averaged to +y otherwise.
    #[test]
    fn rejects_cancelling_mean_longitudes() {
        let epsilon = f64::EPSILON;
        for element_type in EQUINOCTIAL {
            let factor = factor(element_type);
            for (offset_eps, accepted) in [(64.0, false), (256.0, true)] {
                let offset = offset_eps * epsilon;
                let states = [
                    (Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, factor, 0.0)),
                    (
                        Vector3::new(-1.0, offset, 0.0),
                        Vector3::new(-factor * offset, -factor, 0.0),
                    ),
                ];
                assert_eq!(offset / 2.0 > 64.0 * epsilon, accepted);
                let result = run_module(&states, &[1.0, 1.0], Some(element_type), 1.0);
                if accepted {
                    let expected = Vector6::new(0.0, 1.0, 0.0, -factor, 0.0, 0.0);
                    let atol = 64.0 * epsilon;
                    assert_close(&result.unwrap(), &expected, 0.0, [atol, atol], "longitude");
                } else {
                    expect_error(result, "longitude");
                }
            }
        }
    }

    // Upstream `test_invalid_selector` (selector codes -1 and 3) has no
    // counterpart: `OrbitalElementType` cannot hold an unsupported choice.

    /// Upstream `test_invalid_gravity`: the equinoctial averages need a
    /// finite positive `mu`.
    #[test]
    fn rejects_an_invalid_gravitational_parameter() {
        let state = classical_state((7e6, 0.1, 0.9, 0.3, 0.4, 0.7));
        for mu in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            expect_error(
                run_module(&[state], &[100.0], Some(EquinoctialPrograde), mu),
                "gravitational parameter",
            );
        }
    }

    /// Upstream `test_invalid_mass`: zero total, negative and non-finite
    /// masses are rejected.
    #[test]
    fn rejects_invalid_masses() {
        let state = classical_state((7e6, 0.1, 0.9, 0.3, 0.4, 0.7));
        for mass in [0.0, -1.0, f64::INFINITY, f64::NAN] {
            expect_error(
                run_module(&[state], &[mass], Some(EquinoctialPrograde), MU),
                "mass",
            );
        }
    }

    /// Upstream `test_nonelliptic_input_is_rejected`: exact unit-scale
    /// parabolic and hyperbolic states.
    #[test]
    fn rejects_parabolic_and_hyperbolic_states() {
        for velocity in [Vector3::new(0.0, 1.0, 1.0), Vector3::new(0.0, 2.0, 0.0)] {
            let state = (Vector3::new(1.0, 0.0, 0.0), velocity);
            expect_error(
                run_module(&[state], &[1.0], Some(EquinoctialPrograde), 1.0),
                "elliptic",
            );
        }
    }

    /// Upstream `test_opposite_chart_pole_is_rejected`: each representation
    /// rejects the pole it excludes.
    #[test]
    fn rejects_the_opposite_chart_pole() {
        for (element_type, inclination) in [(EquinoctialPrograde, PI), (EquinoctialRetrograde, 0.0)]
        {
            let state = classical_state((7e6, 0.1, inclination, 0.3, 0.4, 0.7));
            expect_error(
                run_module(&[state], &[100.0], Some(element_type), MU),
                "pole",
            );
        }
    }

    /// Upstream `test_undefined_longitude_is_rejected`: equal masses on
    /// opposite phases of one circular orbit.
    #[test]
    fn rejects_opposite_phases() {
        let speed = (MU / 7e6).sqrt();
        let state = (Vector3::new(7e6, 0.0, 0.0), Vector3::new(0.0, speed, 0.0));
        let opposite = (-state.0, -state.1);
        expect_error(
            run_module(
                &[state, opposite],
                &[100.0, 100.0],
                Some(EquinoctialPrograde),
                MU,
            ),
            "longitude",
        );
    }

    /// Upstream `test_invalid_navigation`: zero radius, zero velocity and
    /// non-finite states stop the update.
    #[test]
    fn rejects_invalid_navigation() {
        let bad_states = [
            (Vector3::zeros(), Vector3::new(0.0, 7500.0, 0.0)),
            (Vector3::new(7e6, 0.0, 0.0), Vector3::zeros()),
            (
                Vector3::new(f64::NAN, 0.0, 0.0),
                Vector3::new(0.0, 7500.0, 0.0),
            ),
            (
                Vector3::new(7e6, 0.0, 0.0),
                Vector3::new(0.0, f64::INFINITY, 0.0),
            ),
        ];
        for state in bad_states {
            assert!(run_module(&[state], &[100.0], Some(EquinoctialPrograde), MU).is_err());
        }
    }

    /// Upstream `test_zero_mass_member_does_not_contribute`.
    #[test]
    fn a_zero_mass_member_does_not_contribute() {
        let first = classical_state((7e6, 0.1, 0.9, 0.3, 0.4, 0.7));
        let second = classical_state((8e6, 0.3, 1.1, 0.5, 0.6, 1.7));
        assert_state(
            &run_equinoctial(&[first, second], &[100.0, 0.0], EquinoctialPrograde),
            &concat(first),
            DEFAULT_RTOL,
            "zero mass",
        );
    }

    /// Upstream `test_runtime_selection`: every change of the orbital
    /// selector between updates takes effect at the next update, without a
    /// reset.
    #[test]
    fn the_selector_can_change_between_updates() {
        let (axis, eccentricity, inclination) = (7e6, 0.2, 0.9);
        let (node, periapsis, mean_anomaly) = (0.3, 0.5, 0.8);
        let orbits: Vec<Orbit> = [-0.2, 0.2]
            .iter()
            .map(|offset| {
                (
                    axis,
                    eccentricity,
                    inclination,
                    node + offset,
                    periapsis + offset,
                    mean_anomaly,
                )
            })
            .collect();
        let masses = [100.0, 100.0];
        let states: Vec<State> = orbits.iter().map(|o| classical_state(*o)).collect();
        let expected = |element_type: OrbitalElementType| match element_type {
            Classical => concat(classical_state((
                axis,
                eccentricity,
                inclination,
                node,
                periapsis,
                mean_anomaly,
            ))),
            EquinoctialPrograde => weighted_reference(&orbits, &masses, 1.0),
            EquinoctialRetrograde => weighted_reference(&orbits, &masses, -1.0),
        };
        let transitions = [
            (Classical, EquinoctialPrograde),
            (Classical, EquinoctialRetrograde),
            (EquinoctialPrograde, Classical),
            (EquinoctialPrograde, EquinoctialRetrograde),
            (EquinoctialRetrograde, Classical),
            (EquinoctialRetrograde, EquinoctialPrograde),
        ];
        for (initial_type, selected_type) in transitions {
            let difference = expected(selected_type) - expected(initial_type);
            assert!(difference.fixed_rows::<3>(0).norm() > 1.0);
            assert!(difference.fixed_rows::<3>(3).norm() > 1e-3);

            let mut fixture = fixture(&states, &masses, Some(initial_type), true, MU);
            for (step_index, element_type) in [selected_type, initial_type].into_iter().enumerate()
            {
                fixture.module.config.orbital_element_type = element_type;
                let actual = step(&mut fixture.module, step_index as u64 * 1_000_000_000).unwrap();
                assert_state(
                    &actual,
                    &expected(element_type),
                    0.0,
                    &format!("{initial_type:?} -> {selected_type:?}, at {element_type:?}"),
                );
            }
        }
    }

    /// Upstream `test_runtime_validation` (its `mu = NaN` case; the selector
    /// case cannot be expressed): a `mu` broken after the reset is caught
    /// by the next update, which then publishes nothing.
    #[test]
    fn an_invalid_mu_set_after_reset_stops_the_update() {
        let state = classical_state((7e6, 0.1, 0.9, 0.3, 0.4, 0.7));
        let mut fixture = fixture(&[state], &[100.0], Some(EquinoctialPrograde), true, MU);
        let first = step(&mut fixture.module, 0).unwrap();
        fixture.module.config.mu_m3ps2 = f64::NAN;
        assert_eq!(
            step(&mut fixture.module, 1_000_000_000),
            Err(BarycenterError::InvalidGravitationalParameter)
        );
        // The output keeps the last value written, as upstream's unwritten
        // message after the exception.
        let output = fixture.module.trans_out_msg.read();
        assert_eq!(
            concat((output.position_bn_n_m, output.velocity_bn_n_mps)),
            first
        );
    }

    #[test]
    fn makes_one_input_pair_per_configured_spacecraft() {
        let module = FormationBarycenter::new(FormationBarycenterConfig {
            num_spacecraft: 4,
            ..Default::default()
        });
        assert_eq!(module.sc_nav_in_msgs.len(), 4);
        assert_eq!(module.sc_payload_in_msgs.len(), 4);
        assert!(
            module
                .sc_nav_in_msgs
                .iter()
                .all(|input| !input.is_connected())
        );
    }
}
