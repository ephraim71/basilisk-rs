//! Setup shared by the ports of upstream's magnetic-field and magnetometer
//! scenarios (`scenarioMagneticFieldCenteredDipole`,
//! `scenarioMagneticFieldWMM`, `scenarioTAM`, `scenarioTAMcomparison`): the
//! point-mass planet, the one-orbit polar orbit, the field models as
//! `simSetPlanetEnvironment.centeredDipoleMagField` and the scripts configure
//! them, and the scripts' three-axis magnetometer.

#![allow(dead_code)]

use std::str::FromStr;

use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::magnetic_field_centered_dipole::{
    MagneticFieldCenteredDipole, MagneticFieldCenteredDipoleConfig,
};
use basilisk_rs::environment::wmm_field::{WmmField, WmmFieldConfig};
use basilisk_rs::sensors::magnetometer::{Magnetometer, MagnetometerConfig};
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::astro_constants::{D2R, MU_EARTH, MU_JUPITER, REQ_EARTH, REQ_JUPITER};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use hifitime::Epoch;
use nalgebra::{Matrix3, UnitQuaternion, Vector3};

/// Upstream `simulationTimeStep = macros.sec2nano(10.)` of the dipole and
/// magnetometer scenarios.
pub const TASK_PERIOD_NANOS: u64 = 10_000_000_000;

/// Upstream `epochMsg = timeStringToGregorianUTCMsg('2019 June 27,
/// 10:23:0.0 (UTC)')`, the WMM epoch: here the simulation start epoch, which
/// [`WmmField`] evaluates the model at.
pub fn wmm_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2019, 6, 27, 10, 23, 0, 0)
}

/// Upstream `planetCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Planet {
    Earth,
    Jupiter,
}

impl Planet {
    /// Upstream `planet.mu` of `gravFactory.createEarth()` /
    /// `createJupiter()` \[m^3/s^2\].
    pub fn mu_m3ps2(self) -> f64 {
        match self {
            Self::Earth => MU_EARTH * 1e9,
            Self::Jupiter => MU_JUPITER * 1e9,
        }
    }

    /// Upstream `planet.radEquator` \[m\].
    pub fn radius_equator_m(self) -> f64 {
        match self {
            Self::Earth => REQ_EARTH * 1e3,
            Self::Jupiter => REQ_JUPITER * 1e3,
        }
    }

    /// Upstream's `planetCase` string.
    pub fn name(self) -> &'static str {
        match self {
            Self::Earth => "Earth",
            Self::Jupiter => "Jupiter",
        }
    }

    /// The planet as the central point-mass gravity body at the origin.
    pub fn add_to(self, spacecraft: &mut Spacecraft) {
        let name = match self {
            Self::Earth => "earth",
            Self::Jupiter => "jupiter barycenter",
        };
        spacecraft
            .add_grav_body(
                GravBodyData::point_mass(
                    name,
                    self.mu_m3ps2(),
                    true,
                    Vector3::zeros(),
                    Vector3::zeros(),
                )
                .expect("valid point-mass planet"),
            )
            .expect("unique central gravity body");
    }

    /// Upstream `simSetPlanetEnvironment.centeredDipoleMagField(magModule,
    /// name)`: the 2020 IGRF dipole for Earth, and for Jupiter the NASA fact
    /// sheet's 4.30 G dipole tilted 9.4 deg at 200.1 deg longitude
    /// (`convertToIgrfDipoleCoefficients`).
    pub fn centered_dipole(self, name: &str) -> MagneticFieldCenteredDipoleConfig {
        let (g10_t, g11_t, h11_t, planet_radius_m) = match self {
            Self::Earth => (
                -30926.00 / 1e9,
                -2318.00 / 1e9,
                5817.00 / 1e9,
                6371.2 * 1000.0,
            ),
            Self::Jupiter => {
                let (g10, g11, h11) =
                    igrf_dipole_coefficients(4.30 / 10000.0, 9.4 * D2R, 200.1 * D2R);
                (g10, g11, h11, 71398.0 * 1000.0)
            }
        };
        MagneticFieldCenteredDipoleConfig {
            name: name.to_string(),
            g10_t,
            g11_t,
            h11_t,
            planet_radius_m,
            env_min_reach_m: None,
            env_max_reach_m: None,
        }
    }
}

impl FromStr for Planet {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value.to_ascii_lowercase().as_str() {
            "earth" => Ok(Self::Earth),
            "jupiter" => Ok(Self::Jupiter),
            _ => Err(format!("unknown planet {value}")),
        }
    }
}

/// Upstream `simSetPlanetEnvironment.convertToIgrfDipoleCoefficients`:
/// `(g10, g11, h11)` \[T\] of a dipole of strength `nominal_field_t` tilted
/// `tilt_rad` from the spin axis towards `longitude_of_tilt_rad`.
pub fn igrf_dipole_coefficients(
    nominal_field_t: f64,
    tilt_rad: f64,
    longitude_of_tilt_rad: f64,
) -> (f64, f64, f64) {
    let theta_m = std::f64::consts::PI - tilt_rad;
    let alpha_m = std::f64::consts::PI - longitude_of_tilt_rad;
    (
        nominal_field_t * theta_m.cos(),
        nominal_field_t * theta_m.sin() * alpha_m.cos(),
        nominal_field_t * theta_m.sin() * alpha_m.sin(),
    )
}

/// The second, half-strength Earth dipole of the elliptical-orbit cases
/// (upstream `magModule2`), reaching out to `1.3 REQ`. The first dipole's
/// `envMinReach` is set to the same radius.
pub fn inner_earth_dipole(name: &str) -> MagneticFieldCenteredDipoleConfig {
    MagneticFieldCenteredDipoleConfig {
        name: name.to_string(),
        g10_t: -30926.00 / 1e9 * 0.5,
        g11_t: -2318.00 / 1e9 * 0.5,
        h11_t: 5817.00 / 1e9 * 0.5,
        planet_radius_m: 6371.2 * 1000.0,
        env_min_reach_m: None,
        env_max_reach_m: Some(inner_dipole_reach_m()),
    }
}

/// Upstream `magModule2.envMaxReach = req*1.3` \[m\].
pub fn inner_dipole_reach_m() -> f64 {
    Planet::Earth.radius_equator_m() * 1.3
}

/// A centered-dipole module.
pub fn centered_dipole(config: MagneticFieldCenteredDipoleConfig) -> MagneticFieldCenteredDipole {
    MagneticFieldCenteredDipole::new(config)
}

/// Upstream `magneticFieldWMM` named `WMM`, with the elliptical-orbit
/// cases' reach band `10 000 km < r < 20 000 km`.
pub fn wmm(orbit: OrbitCase) -> WmmField {
    let (env_min_reach_m, env_max_reach_m) = wmm_reach_m(orbit);
    WmmField::new(WmmFieldConfig {
        name: "WMM".to_string(),
        env_min_reach_m,
        env_max_reach_m,
    })
}

/// The WMM reach band of `orbit` \[m\].
pub fn wmm_reach_m(orbit: OrbitCase) -> (Option<f64>, Option<f64>) {
    match orbit {
        OrbitCase::Circular => (None, None),
        OrbitCase::Elliptical => (Some(10000.0 * 1000.0), Some(20000.0 * 1000.0)),
    }
}

/// Upstream `orbitCase`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrbitCase {
    Circular,
    Elliptical,
}

impl OrbitCase {
    /// Upstream's `orbitCase` string.
    pub fn name(self) -> &'static str {
        match self {
            Self::Circular => "circular",
            Self::Elliptical => "elliptical",
        }
    }
}

impl FromStr for OrbitCase {
    type Err = String;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "circular" => Ok(Self::Circular),
            "elliptical" => Ok(Self::Elliptical),
            _ => Err(format!("unknown orbit case {value}")),
        }
    }
}

/// The scenarios' orbit: periapsis at `1.1 REQ`, apoapsis at `3.5 REQ` when
/// elliptical, `i = 85 deg`, `Ω = 48.2 deg`, `ω = 347.8 deg`, `f = 85.3 deg`.
pub struct ScenarioOrbit {
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    /// Upstream `P = 2 pi / n` from the `rv2elem` semimajor axis \[s\].
    pub period_s: f64,
}

impl ScenarioOrbit {
    pub fn new(planet: Planet, case: OrbitCase) -> Self {
        let mu = planet.mu_m3ps2();
        let req = planet.radius_equator_m();
        let r_periapsis = req * 1.1;
        let (semi_major_axis, eccentricity) = match case {
            OrbitCase::Circular => (r_periapsis, 0.0),
            OrbitCase::Elliptical => {
                let r_apoapsis = req * 3.5;
                let a = (r_periapsis + r_apoapsis) / 2.0;
                (a, 1.0 - r_periapsis / a)
            }
        };
        let elements = ClassicElements {
            semi_major_axis,
            eccentricity,
            inclination_rad: 85.0 * D2R,
            raan_rad: 48.2 * D2R,
            arg_periapsis_rad: 347.8 * D2R,
            true_anomaly_rad: 85.3 * D2R,
            ..Default::default()
        };
        let (position_m, velocity_mps) = elements_to_state(mu, &elements);
        let a = state_to_elements(mu, position_m, velocity_mps).semi_major_axis;
        let mean_motion = (mu / a / a / a).sqrt();
        Self {
            position_m,
            velocity_mps,
            period_s: 2.0 * std::f64::consts::PI / mean_motion,
        }
    }

    /// Upstream `simulationTime = macros.sec2nano(1. * P)`.
    pub fn duration_nanos(&self) -> u64 {
        sec2nano(self.period_s)
    }

    /// The scripts' `spacecraft.Spacecraft()` with the hub defaults (unit
    /// mass, identity inertia) on this orbit, integrated on `step_nanos`.
    pub fn spacecraft(&self, step_nanos: u64) -> Spacecraft {
        Spacecraft::new(SpacecraftConfig {
            mass_kg: 1.0,
            hub_center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::identity(),
            integration_step_nanos: step_nanos,
            initial_position_m: self.position_m,
            initial_velocity_mps: self.velocity_mps,
            initial_sigma_bn: Vector3::zeros(),
            initial_omega_radps: Vector3::zeros(),
            integrator: None,
        })
    }
}

/// Upstream `macros.sec2nano`: `int(time * 1e9 + 0.5)`.
pub fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1e9 + 0.5) as u64
}

/// Upstream `TAM.senNoiseStd = [100e-9] * 3` \[T\], one sigma per step.
pub const TAM_NOISE_STD_T: f64 = 100e-9;
/// Upstream `TAM.senBias = [0, 0, -1e-6]` \[T\] of the `useBias` cases.
pub const TAM_BIAS_T: [f64; 3] = [0.0, 0.0, -1e-6];

/// Upstream `magnetometer.Magnetometer()` with `scaleFactor = 1`, the
/// scripts' 100 nT white noise, the optional `[0, 0, -1 uT]` bias and the
/// optional symmetric saturation `±bound_t`. The noise matrix is in
/// basilisk-rs's 3-sigma units (see [`basilisk_rs::gauss_markov`]), so it is
/// `3 senNoiseStd` for upstream's one-sigma `senNoiseStd`.
pub fn tam(name: &str, use_bias: bool, bound_t: Option<f64>) -> Magnetometer {
    let (min_output_t, max_output_t) = match bound_t {
        Some(bound) => (-bound, bound),
        None => (-1e200, 1e200),
    };
    Magnetometer::new(MagnetometerConfig {
        name: name.to_string(),
        body_to_sensor_quaternion: UnitQuaternion::identity(),
        bias_t: if use_bias {
            Vector3::from(TAM_BIAS_T)
        } else {
            Vector3::zeros()
        },
        p_matrix_sqrt_t: Matrix3::from_diagonal_element(3.0 * TAM_NOISE_STD_T),
        a_matrix: Matrix3::zeros(),
        walk_bounds_t: Vector3::zeros(),
        scale_factor: 1.0,
        min_output_t,
        max_output_t,
        mtb_residual_field: None,
    })
}
