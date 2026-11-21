//! Statistical dispersions of a Monte Carlo campaign's parameters: port of
//! upstream Basilisk `utilities/MonteCarlo/Dispersions.py`.
//!
//! Each dispersion draws a new value of one parameter type from the run's
//! random-number generator, given the value in the nominal configuration
//! ([`Dispersion::sample`]). Upstream names the parameter by an attribute
//! path string (`"TaskList[0].TaskModels[0].hub.mHub"`); here a campaign binds
//! a dispersion to an explicit field of its configuration type with
//! [`MonteCarlo::disperse`](super::MonteCarlo::disperse).
//!
//! | upstream | here | value |
//! |---|---|---|
//! | `UniformDispersion` | [`UniformDispersion`] | `f64` |
//! | `NormalDispersion` | [`NormalDispersion`] | `f64` |
//! | `UniformVectorDispersion`, `UniformVectorCartDispersion` | [`UniformVectorCartDispersion`] | `Vector3` |
//! | `NormalVectorDispersion`, `NormalVectorCartDispersion` | [`NormalVectorCartDispersion`] | `Vector3` |
//! | `UniformVectorAngleDispersion` | [`UniformVectorAngleDispersion`] | unit `Vector3` |
//! | `NormalVectorAngleDispersion` | [`NormalVectorAngleDispersion`] | unit `Vector3` |
//! | `UniformEulerAngleMRPDispersion` | [`UniformEulerAngleMrpDispersion`] | MRP `Vector3` |
//! | `NormalThrusterUnitDirectionVectorDispersion` | [`NormalThrusterUnitDirectionVectorDispersion`] | `Vector3` |
//! | `MRPDispersionPerAxis` | [`MrpDispersionPerAxis`] | `Vector3` |
//! | `InertiaTensorDispersion` | [`InertiaTensorDispersion`] | `Matrix3` |
//! | `OrbitalElementDispersion` | [`OrbitalElementDispersion`] | `(r_N, v_N)` |
//!
//! As upstream, the uniform and normal scalar and Cartesian-vector
//! dispersions draw absolute values (the nominal value is ignored), while the
//! angle and inertia dispersions perturb the nominal value. Upstream draws
//! from Python's `random` and NumPy's global generators; here every draw
//! comes from the one generator of the run, so a run's values are fixed by
//! its seed but are not upstream's numbers. Upstream's dispersion
//! "magnitude" strings (`setDispMagnitudeFile`) are not ported.

use nalgebra::{Matrix3, Vector3};
use rand::Rng;
use rand_distr::StandardNormal;

use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, euler321_to_dcm, euler323_to_dcm};

/// The random-number generator a run's dispersions draw from.
pub type RunRng = rand::rngs::StdRng;

/// A distribution of one parameter of a campaign's configuration.
pub trait Dispersion: Send + Sync {
    /// The dispersed parameter's type.
    type Value;

    /// A dispersed value; `nominal` is the parameter's value in the nominal
    /// configuration (upstream `getNestedAttr(sim, varName)`).
    fn sample(&self, nominal: &Self::Value, rng: &mut RunRng) -> Self::Value;
}

/// Python's `random.uniform(a, b)`: `a + (b - a) U`, `U` in `[0, 1)`.
fn uniform(rng: &mut RunRng, bounds: (f64, f64)) -> f64 {
    bounds.0 + (bounds.1 - bounds.0) * rng.r#gen::<f64>()
}

/// Python's `random.gauss(mean, std)`.
fn gauss(rng: &mut RunRng, mean: f64, std_deviation: f64) -> f64 {
    let standard: f64 = rng.sample(StandardNormal);
    mean + std_deviation * standard
}

/// Upstream `checkBounds`: `value` clamped into `bounds`.
fn clamp(value: f64, bounds: (f64, f64)) -> f64 {
    value.max(bounds.0).min(bounds.1)
}

/// Upstream `UniformDispersion`: uniform on `bounds` (default `[-1, 1]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniformDispersion {
    pub bounds: (f64, f64),
}

impl UniformDispersion {
    pub fn new(lower: f64, upper: f64) -> Self {
        Self {
            bounds: (lower, upper),
        }
    }
}

impl Default for UniformDispersion {
    fn default() -> Self {
        Self::new(-1.0, 1.0)
    }
}

impl Dispersion for UniformDispersion {
    type Value = f64;

    fn sample(&self, _nominal: &f64, rng: &mut RunRng) -> f64 {
        uniform(rng, self.bounds)
    }
}

/// Upstream `NormalDispersion`: normal with `mean` and `std_deviation`,
/// clamped into `bounds` when given.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalDispersion {
    pub mean: f64,
    pub std_deviation: f64,
    pub bounds: Option<(f64, f64)>,
}

impl NormalDispersion {
    pub fn new(mean: f64, std_deviation: f64) -> Self {
        Self {
            mean,
            std_deviation,
            bounds: None,
        }
    }

    pub fn with_bounds(mut self, lower: f64, upper: f64) -> Self {
        self.bounds = Some((lower, upper));
        self
    }
}

impl Default for NormalDispersion {
    /// Upstream's defaults, `mean = 0`, `stdDeviation = 0.5`.
    fn default() -> Self {
        Self::new(0.0, 0.5)
    }
}

impl Dispersion for NormalDispersion {
    type Value = f64;

    fn sample(&self, _nominal: &f64, rng: &mut RunRng) -> f64 {
        let value = gauss(rng, self.mean, self.std_deviation);
        self.bounds.map_or(value, |bounds| clamp(value, bounds))
    }
}

/// Upstream `UniformVectorCartDispersion` (and `UniformVectorDispersion`):
/// each component uniform on `bounds` (default `[-1, 1]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniformVectorCartDispersion {
    pub bounds: (f64, f64),
}

impl UniformVectorCartDispersion {
    pub fn new(lower: f64, upper: f64) -> Self {
        Self {
            bounds: (lower, upper),
        }
    }
}

impl Default for UniformVectorCartDispersion {
    fn default() -> Self {
        Self::new(-1.0, 1.0)
    }
}

impl Dispersion for UniformVectorCartDispersion {
    type Value = Vector3<f64>;

    fn sample(&self, _nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        Vector3::from_fn(|_, _| uniform(rng, self.bounds))
    }
}

/// Upstream `NormalVectorCartDispersion` (and `NormalVectorDispersion`):
/// component `i` normal with `mean[i]` and `std_deviation[i]`, clamped into
/// `bounds` when given.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalVectorCartDispersion {
    pub mean: Vector3<f64>,
    pub std_deviation: Vector3<f64>,
    pub bounds: Option<(f64, f64)>,
}

impl NormalVectorCartDispersion {
    pub fn new(mean: Vector3<f64>, std_deviation: Vector3<f64>) -> Self {
        Self {
            mean,
            std_deviation,
            bounds: None,
        }
    }

    /// The same mean and standard deviation on every component (upstream's
    /// scalar `mean` and `stdDeviation`).
    pub fn isotropic(mean: f64, std_deviation: f64) -> Self {
        Self::new(Vector3::repeat(mean), Vector3::repeat(std_deviation))
    }

    pub fn with_bounds(mut self, lower: f64, upper: f64) -> Self {
        self.bounds = Some((lower, upper));
        self
    }
}

impl Dispersion for NormalVectorCartDispersion {
    type Value = Vector3<f64>;

    fn sample(&self, _nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        Vector3::from_fn(|index, _| {
            let value = gauss(rng, self.mean[index], self.std_deviation[index]);
            self.bounds.map_or(value, |bounds| clamp(value, bounds))
        })
    }
}

/// Upstream `cart2Spherical` of a unit vector: `(phi, theta)`, the azimuth
/// `atan2(y, x)` and the polar angle `acos(z)`.
fn unit_to_spherical(unit: &Vector3<f64>) -> (f64, f64) {
    (unit.y.atan2(unit.x), unit.z.clamp(-1.0, 1.0).acos())
}

/// Upstream `spherical2Cart([1, phi, theta])`.
fn spherical_to_unit(phi: f64, theta: f64) -> Vector3<f64> {
    Vector3::new(
        theta.sin() * phi.cos(),
        theta.sin() * phi.sin(),
        theta.cos(),
    )
}

/// Upstream `UniformVectorAngleDispersion`: a unit vector whose azimuth and
/// polar angle are uniform on the nominal direction's angles plus
/// `phi_bounds_off_nominal` and `theta_bounds_off_nominal` (defaults
/// `[-π/2, π/2]` and `[-π, π]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniformVectorAngleDispersion {
    pub phi_bounds_off_nominal: (f64, f64),
    pub theta_bounds_off_nominal: (f64, f64),
}

impl Default for UniformVectorAngleDispersion {
    fn default() -> Self {
        use std::f64::consts::{FRAC_PI_2, PI};
        Self {
            phi_bounds_off_nominal: (-FRAC_PI_2, FRAC_PI_2),
            theta_bounds_off_nominal: (-PI, PI),
        }
    }
}

impl Dispersion for UniformVectorAngleDispersion {
    type Value = Vector3<f64>;

    fn sample(&self, nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        let (phi, theta) = unit_to_spherical(&nominal.normalize());
        let phi_bounds = (
            phi + self.phi_bounds_off_nominal.0,
            phi + self.phi_bounds_off_nominal.1,
        );
        let theta_bounds = (
            theta + self.theta_bounds_off_nominal.0,
            theta + self.theta_bounds_off_nominal.1,
        );
        let phi = clamp(uniform(rng, phi_bounds), phi_bounds);
        let theta = clamp(uniform(rng, theta_bounds), theta_bounds);
        spherical_to_unit(phi, theta).normalize()
    }
}

/// Upstream `NormalVectorAngleDispersion`: a unit vector whose azimuth and
/// polar angle are normal about the nominal direction's, with `phi_std` and
/// `theta_std` (default π/3), clamped to the nominal angles plus the bounds
/// off nominal (defaults `[-π/2, π/2]` and `[-π, π]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalVectorAngleDispersion {
    pub theta_std: f64,
    pub phi_std: f64,
    pub theta_bounds_off_nominal: (f64, f64),
    pub phi_bounds_off_nominal: (f64, f64),
}

impl Default for NormalVectorAngleDispersion {
    fn default() -> Self {
        use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};
        Self {
            theta_std: FRAC_PI_3,
            phi_std: FRAC_PI_3,
            theta_bounds_off_nominal: (-PI, PI),
            phi_bounds_off_nominal: (-FRAC_PI_2, FRAC_PI_2),
        }
    }
}

impl Dispersion for NormalVectorAngleDispersion {
    type Value = Vector3<f64>;

    fn sample(&self, nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        let (phi, theta) = unit_to_spherical(&nominal.normalize());
        let phi_sample = gauss(rng, phi, self.phi_std);
        let theta_sample = gauss(rng, theta, self.theta_std);
        let phi_sample = clamp(
            phi_sample,
            (
                phi + self.phi_bounds_off_nominal.0,
                phi + self.phi_bounds_off_nominal.1,
            ),
        );
        let theta_sample = clamp(
            theta_sample,
            (
                theta + self.theta_bounds_off_nominal.0,
                theta + self.theta_bounds_off_nominal.1,
            ),
        );
        spherical_to_unit(phi_sample, theta_sample).normalize()
    }
}

/// Upstream `UniformEulerAngleMRPDispersion`: the MRP of a 3-2-3 Euler
/// angle set whose three angles are uniform on `bounds` (default `[0, 2π]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UniformEulerAngleMrpDispersion {
    pub bounds: (f64, f64),
}

impl Default for UniformEulerAngleMrpDispersion {
    fn default() -> Self {
        Self {
            bounds: (0.0, std::f64::consts::TAU),
        }
    }
}

impl Dispersion for UniformEulerAngleMrpDispersion {
    type Value = Vector3<f64>;

    fn sample(&self, _nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        let angles = Vector3::from_fn(|_, _| uniform(rng, self.bounds));
        dcm_to_mrp(euler323_to_dcm(angles))
    }
}

/// Upstream `MRPDispersionPerAxis`: component `i` uniform on `bounds[i]`
/// (default `[0, 2π]` each), taken as the MRP itself, as upstream does.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MrpDispersionPerAxis {
    pub bounds: [(f64, f64); 3],
}

impl Default for MrpDispersionPerAxis {
    fn default() -> Self {
        Self {
            bounds: [(0.0, std::f64::consts::TAU); 3],
        }
    }
}

impl Dispersion for MrpDispersionPerAxis {
    type Value = Vector3<f64>;

    fn sample(&self, _nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        Vector3::from_fn(|index, _| uniform(rng, self.bounds[index]))
    }
}

/// Upstream `eigAxisAndAngleToDCM`: the DCM of a rotation by `angle` about
/// `axis` (normalized).
fn eigen_axis_angle_to_dcm(axis: &Vector3<f64>, angle: f64) -> Matrix3<f64> {
    let axis = axis.normalize();
    let sigma = 1.0 - angle.cos();
    let (sin, cos) = angle.sin_cos();
    Matrix3::new(
        axis[0] * axis[0] * sigma + cos,
        axis[0] * axis[1] * sigma + axis[2] * sin,
        axis[0] * axis[2] * sigma - axis[1] * sin,
        axis[1] * axis[0] * sigma - axis[2] * sin,
        axis[1] * axis[1] * sigma + cos,
        axis[1] * axis[2] * sigma + axis[0] * sin,
        axis[2] * axis[0] * sigma + axis[1] * sin,
        axis[2] * axis[1] * sigma - axis[0] * sin,
        axis[2] * axis[2] * sigma + cos,
    )
}

/// Upstream `NormalThrusterUnitDirectionVectorDispersion`: the nominal
/// direction rotated by a normal angle (`phi_std`, default 0.1745 rad) about
/// an axis perpendicular to it and to a random vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NormalThrusterUnitDirectionVectorDispersion {
    pub phi_std: f64,
}

impl Default for NormalThrusterUnitDirectionVectorDispersion {
    fn default() -> Self {
        Self { phi_std: 0.1745 }
    }
}

impl Dispersion for NormalThrusterUnitDirectionVectorDispersion {
    type Value = Vector3<f64>;

    fn sample(&self, nominal: &Vector3<f64>, rng: &mut RunRng) -> Vector3<f64> {
        let angle = gauss(rng, 0.0, self.phi_std);
        // Upstream `perturbVectorByAngle`.
        let mut random = Vector3::from_fn(|_, _| rng.r#gen::<f64>());
        if random.dot(nominal) > 0.95 {
            random[0] *= -1.0;
        }
        let axis = nominal.cross(&random);
        eigen_axis_angle_to_dcm(&axis, angle) * nominal
    }
}

/// Upstream `InertiaTensorDispersion`: the nominal inertia plus a diagonal
/// of normal values (`std_diag`, default 1 kg m², clamped into
/// `bounds_diag`, default `[-1, 1]`), turned by a similarity transform with
/// the 3-2-1 Euler angles of three normal angles (`std_angle`, default 0):
/// `[C] ([I] + diag) [C]ᵀ`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct InertiaTensorDispersion {
    pub std_diag: f64,
    pub bounds_diag: (f64, f64),
    pub std_angle: f64,
}

impl Default for InertiaTensorDispersion {
    fn default() -> Self {
        Self {
            std_diag: 1.0,
            bounds_diag: (-1.0, 1.0),
            std_angle: 0.0,
        }
    }
}

impl InertiaTensorDispersion {
    /// Upstream `InertiaTensorDispersion(varName, stdAngle=...)`, the other
    /// parameters at their defaults.
    pub fn with_std_angle(std_angle: f64) -> Self {
        Self {
            std_angle,
            ..Self::default()
        }
    }
}

impl Dispersion for InertiaTensorDispersion {
    type Value = Matrix3<f64>;

    fn sample(&self, nominal: &Matrix3<f64>, rng: &mut RunRng) -> Matrix3<f64> {
        let diagonal =
            Vector3::from_fn(|_, _| clamp(gauss(rng, 0.0, self.std_diag), self.bounds_diag));
        let angles = Vector3::from_fn(|_, _| gauss(rng, 0.0, self.std_angle));
        let rotation = euler321_to_dcm(angles);
        rotation * (nominal + Matrix3::from_diagonal(&diagonal)) * rotation.transpose()
    }
}

/// How one classical element of an [`OrbitalElementDispersion`] is drawn
/// (upstream's `['normal', mean, std]` or `['uniform', low, high]`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ElementDistribution {
    Normal { mean: f64, std_deviation: f64 },
    Uniform { lower: f64, upper: f64 },
}

impl ElementDistribution {
    fn draw(&self, rng: &mut RunRng) -> f64 {
        match *self {
            Self::Normal {
                mean,
                std_deviation,
            } => gauss(rng, mean, std_deviation),
            Self::Uniform { lower, upper } => uniform(rng, (lower, upper)),
        }
    }
}

/// Upstream `OrbitalElementDispersion`: classical elements drawn from their
/// distributions (an element without one is zero), a negative eccentricity
/// set to zero, and turned into the inertial position and velocity
/// (upstream `elem2rv_parab`) about a body of gravitational parameter
/// `mu`. Upstream sets its two variables from the one draw; here the value
/// is the pair `(r_N, v_N)`, so bind it to a field of that type, or use
/// [`Self::sample_state`] with
/// [`MonteCarlo::disperse_with`](super::MonteCarlo::disperse_with).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OrbitalElementDispersion {
    /// Gravitational parameter, in the units of the elements.
    pub mu: f64,
    pub semi_major_axis: Option<ElementDistribution>,
    pub eccentricity: Option<ElementDistribution>,
    pub inclination_rad: Option<ElementDistribution>,
    pub raan_rad: Option<ElementDistribution>,
    pub arg_periapsis_rad: Option<ElementDistribution>,
    pub true_anomaly_rad: Option<ElementDistribution>,
}

impl OrbitalElementDispersion {
    /// A dispersed position and velocity.
    pub fn sample_state(&self, rng: &mut RunRng) -> (Vector3<f64>, Vector3<f64>) {
        let mut draw = |distribution: &Option<ElementDistribution>| {
            distribution.map_or(0.0, |distribution| distribution.draw(rng))
        };
        // Upstream draws the elements in its dictionary's order.
        let semi_major_axis = draw(&self.semi_major_axis);
        let eccentricity = draw(&self.eccentricity).max(0.0);
        let inclination_rad = draw(&self.inclination_rad);
        let raan_rad = draw(&self.raan_rad);
        let arg_periapsis_rad = draw(&self.arg_periapsis_rad);
        let true_anomaly_rad = draw(&self.true_anomaly_rad);
        let elements = ClassicElements {
            semi_major_axis,
            eccentricity,
            inclination_rad,
            raan_rad,
            arg_periapsis_rad,
            true_anomaly_rad,
            ..ClassicElements::default()
        };
        elements_to_state(self.mu, &elements)
    }
}

impl Dispersion for OrbitalElementDispersion {
    type Value = (Vector3<f64>, Vector3<f64>);

    fn sample(
        &self,
        _nominal: &(Vector3<f64>, Vector3<f64>),
        rng: &mut RunRng,
    ) -> (Vector3<f64>, Vector3<f64>) {
        self.sample_state(rng)
    }
}

#[cfg(test)]
mod tests {
    use rand::SeedableRng;

    use super::*;

    fn rng() -> RunRng {
        RunRng::seed_from_u64(7)
    }

    /// Mean and standard deviation of `samples`.
    fn statistics(samples: &[f64]) -> (f64, f64) {
        let count = samples.len() as f64;
        let mean = samples.iter().sum::<f64>() / count;
        let variance = samples.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (count - 1.0);
        (mean, variance.sqrt())
    }

    /// Upstream `test_normal_vector_dispersion_uses_configured_statistics`:
    /// a zero standard deviation gives the mean on every component.
    #[test]
    fn normal_vector_dispersion_uses_configured_statistics() {
        let dispersion = NormalVectorCartDispersion::isotropic(2.5, 0.0);
        let value = dispersion.sample(&Vector3::zeros(), &mut rng());
        assert_eq!(value, Vector3::repeat(2.5));
    }

    /// The draws have the configured per-axis mean and standard deviation.
    #[test]
    fn normal_vector_dispersion_has_the_configured_moments() {
        let mean = Vector3::new(0.02, 0.04, -0.06);
        let std_deviation = Vector3::new(0.002, 0.004, 0.006);
        let dispersion = NormalVectorCartDispersion::new(mean, std_deviation);
        let mut rng = rng();
        let samples: Vec<Vector3<f64>> = (0..20_000)
            .map(|_| dispersion.sample(&Vector3::zeros(), &mut rng))
            .collect();
        for axis in 0..3 {
            let values: Vec<f64> = samples.iter().map(|sample| sample[axis]).collect();
            let (sample_mean, sample_std) = statistics(&values);
            assert!((sample_mean - mean[axis]).abs() < 4.0 * std_deviation[axis] / 141.0);
            assert!((sample_std / std_deviation[axis] - 1.0).abs() < 0.03);
        }
        let bounded = dispersion.with_bounds(0.0, 0.03);
        for _ in 0..100 {
            let value = bounded.sample(&Vector3::zeros(), &mut rng);
            assert!(
                value
                    .iter()
                    .all(|component| (0.0..=0.03).contains(component))
            );
        }
    }

    /// Upstream `test_uniform_dispersion_randomizes_nested_sim_parameter`'s
    /// bounds: draws stay within them and differ.
    #[test]
    fn uniform_dispersion_stays_within_bounds() {
        let dispersion = UniformDispersion::new(24.0, 26.0);
        let mut rng = rng();
        let samples: Vec<f64> = (0..1000)
            .map(|_| dispersion.sample(&1.0, &mut rng))
            .collect();
        assert!(samples.iter().all(|mass| (24.0..=26.0).contains(mass)));
        let (mean, std) = statistics(&samples);
        assert!((mean - 25.0).abs() < 0.1);
        assert!((std - 2.0 / 12.0_f64.sqrt()).abs() < 0.03);
        assert_ne!(samples[0], samples[1]);
    }

    /// Upstream `test_vector_angle_dispersion_reads_nested_method_path` and
    /// `test_uniform_vector_angle_dispersion_uses_bounds_off_nominal`: a unit
    /// vector whose angles stay within the bounds about the nominal's.
    #[test]
    fn uniform_vector_angle_dispersion_uses_bounds_off_nominal() {
        let nominal = Vector3::new(1.0, 1.0, 1.0);
        let dispersion = UniformVectorAngleDispersion {
            phi_bounds_off_nominal: (-0.02, 0.03),
            theta_bounds_off_nominal: (-0.04, 0.05),
        };
        let (phi, theta) = unit_to_spherical(&nominal.normalize());
        let mut rng = rng();
        for _ in 0..500 {
            let value = dispersion.sample(&nominal, &mut rng);
            assert!((value.norm() - 1.0).abs() < 1e-12);
            let (sample_phi, sample_theta) = unit_to_spherical(&value);
            assert!((phi - 0.02 - 1e-12..=phi + 0.03 + 1e-12).contains(&sample_phi));
            assert!((theta - 0.04 - 1e-12..=theta + 0.05 + 1e-12).contains(&sample_theta));
        }
    }

    /// Upstream `test_normal_vector_angle_dispersion_uses_scalar_angle_samples`:
    /// a unit vector, its angles normal about the nominal's.
    #[test]
    fn normal_vector_angle_dispersion_is_centred_on_the_nominal_angles() {
        let nominal = Vector3::new(1.0, 1.0, 1.0);
        let dispersion = NormalVectorAngleDispersion {
            phi_std: 0.02,
            theta_std: 0.03,
            phi_bounds_off_nominal: (-0.1, 0.1),
            theta_bounds_off_nominal: (-0.1, 0.1),
        };
        let (phi, theta) = unit_to_spherical(&nominal.normalize());
        let mut rng = rng();
        let mut phis = Vec::new();
        let mut thetas = Vec::new();
        for _ in 0..5000 {
            let value = dispersion.sample(&nominal, &mut rng);
            assert!((value.norm() - 1.0).abs() < 1e-12);
            let (sample_phi, sample_theta) = unit_to_spherical(&value);
            phis.push(sample_phi);
            thetas.push(sample_theta);
        }
        let (phi_mean, phi_std) = statistics(&phis);
        let (theta_mean, theta_std) = statistics(&thetas);
        assert!((phi_mean - phi).abs() < 2e-3);
        assert!((theta_mean - theta).abs() < 3e-3);
        assert!((phi_std / 0.02 - 1.0).abs() < 0.05);
        assert!((theta_std / 0.03 - 1.0).abs() < 0.05);
    }

    /// The similarity transform keeps the inertia symmetric, and with no
    /// diagonal dispersion keeps its principal moments.
    #[test]
    fn inertia_tensor_dispersion_rotates_the_nominal_inertia() {
        let nominal = Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0));
        let dispersion = InertiaTensorDispersion {
            std_diag: 0.0,
            std_angle: 0.25_f64.to_radians(),
            ..InertiaTensorDispersion::default()
        };
        let value = dispersion.sample(&nominal, &mut rng());
        assert!((value - value.transpose()).norm() < 1e-9);
        assert!((value.trace() - nominal.trace()).abs() < 1e-9);
        assert!((value.determinant() / nominal.determinant() - 1.0).abs() < 1e-12);
        assert!(value[(0, 1)] != 0.0 && value[(0, 1)].abs() < 5.0);

        let diagonal = InertiaTensorDispersion::default().sample(&nominal, &mut rng());
        for axis in 0..3 {
            assert!((diagonal[(axis, axis)] - nominal[(axis, axis)]).abs() <= 1.0);
        }
        assert_eq!(diagonal[(0, 1)], 0.0);
    }

    /// The thruster direction keeps its length and turns by the drawn angle.
    #[test]
    fn thruster_direction_dispersion_turns_the_nominal_direction() {
        let nominal = Vector3::new(0.0, 0.0, 1.0);
        let dispersion = NormalThrusterUnitDirectionVectorDispersion { phi_std: 0.01 };
        let mut rng = rng();
        let angles: Vec<f64> = (0..2000)
            .map(|_| {
                let value = dispersion.sample(&nominal, &mut rng);
                assert!((value.norm() - 1.0).abs() < 1e-12);
                value.dot(&nominal).clamp(-1.0, 1.0).acos()
            })
            .collect();
        // |N(0, σ)| has mean σ sqrt(2/π).
        let (mean, _) = statistics(&angles);
        assert!((mean / (0.01 * (2.0 / std::f64::consts::PI).sqrt()) - 1.0).abs() < 0.05);
    }

    #[test]
    fn euler_angle_mrp_dispersion_is_a_valid_attitude() {
        let mut rng = rng();
        for _ in 0..100 {
            let sigma =
                UniformEulerAngleMrpDispersion::default().sample(&Vector3::zeros(), &mut rng);
            assert!(sigma.norm() <= 1.0 + 1e-12);
        }
        let per_axis = MrpDispersionPerAxis {
            bounds: [(0.0, 0.1), (0.2, 0.3), (-0.1, 0.0)],
        }
        .sample(&Vector3::zeros(), &mut rng);
        assert!((0.0..=0.1).contains(&per_axis.x));
        assert!((0.2..=0.3).contains(&per_axis.y));
        assert!((-0.1..=0.0).contains(&per_axis.z));
    }

    /// A circular equatorial orbit of radius `a` from fixed elements.
    #[test]
    fn orbital_element_dispersion_builds_the_state_from_the_elements() {
        let mu = 3.986_004_415e14;
        let fixed = |value: f64| {
            Some(ElementDistribution::Uniform {
                lower: value,
                upper: value,
            })
        };
        let dispersion = OrbitalElementDispersion {
            mu,
            semi_major_axis: fixed(7.0e6),
            eccentricity: Some(ElementDistribution::Normal {
                mean: -1.0,
                std_deviation: 0.0,
            }),
            inclination_rad: None,
            raan_rad: None,
            arg_periapsis_rad: None,
            true_anomaly_rad: fixed(0.0),
        };
        let (position, velocity) = dispersion.sample_state(&mut rng());
        assert!((position - Vector3::new(7.0e6, 0.0, 0.0)).norm() < 1e-6);
        assert!((velocity - Vector3::new(0.0, (mu / 7.0e6).sqrt(), 0.0)).norm() < 1e-9);
    }
}
