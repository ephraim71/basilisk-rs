//! Opt-in reconstruction of planet states from a coarse grid of SPICE knots
//! (upstream `SpiceInterface::SpiceReconstruction`).
//!
//! By default a planet's position and orientation are queried from SPICE at
//! the exact simulation time on every update. A reconstructed channel is
//! instead evaluated from cached SPICE "knots" `knot_step_nanos` apart on an
//! absolute grid anchored at the initial epoch: a Newton-form Hermite
//! polynomial when the knot velocity is matched as well, a Lagrange one
//! otherwise, over `n_knots` knots that bracket the time (interpolation) or
//! trail it (extrapolation). Orientation is interpolated as the principal
//! rotation vector of each knot's attitude relative to the first knot of
//! the stencil. The emitted velocity and orientation rate are derivatives of
//! the same reconstruction, so a consumer that Euler-steps them stays
//! self-consistent.

use std::collections::BTreeMap;

use nalgebra::{Matrix3, Vector3};

use crate::time::NANO2SEC;
use crate::utilities::rigid_body_kinematics::{d_omega_to_prv, dcm_to_prv, prv_to_dcm};

/// Upstream `defaultKnotStep` \[ns\]: knot spacing of the default settings.
pub const DEFAULT_KNOT_STEP_NANOS: u64 = 60_000_000_000;

/// Upstream `CACHE_SLACK`: knots kept beyond the stencil.
const CACHE_SLACK: usize = 4;

/// Knot-unit step of the orientation-rate central difference.
const ORIENTATION_RATE_STEP: f64 = 1.0e-5;

/// PRV norm below which a knot's PRV rate is its angular velocity
/// (`[B(0)] = I`).
const SMALL_PRV_RAD: f64 = 1.0e-7;

/// The knobs of a reconstructed channel: the arguments of upstream's
/// `setPlanetPositionReconstruction`/`setPlanetOrientationReconstruction`.
/// [`Default`] is upstream's defaults, cubic Hermite interpolation on a 60 s
/// grid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ReconstructionSettings {
    /// Upstream `knotStep` \[ns\]: knot spacing; must be positive.
    pub knot_step_nanos: u64,
    /// Upstream `nKnots`: stencil size, the polynomial order; zero counts as
    /// one.
    pub n_knots: usize,
    /// Upstream `useVelocity`: also match the SPICE velocity (or angular
    /// velocity) at each knot, a Hermite fit, instead of the values only, a
    /// Lagrange fit.
    pub use_velocity: bool,
    /// Upstream `interpolate`: bracket the time with past and future knots
    /// instead of extrapolating from past knots only.
    pub interpolate: bool,
}

impl Default for ReconstructionSettings {
    fn default() -> Self {
        Self {
            knot_step_nanos: DEFAULT_KNOT_STEP_NANOS,
            n_knots: 2,
            use_velocity: true,
            interpolate: true,
        }
    }
}

/// How one channel of a planet is produced (upstream `ChannelMode`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) enum ChannelMode {
    /// Query SPICE at the exact time on every update.
    #[default]
    Exact,
    /// Reconstruct from the cached knot grid.
    Reconstruct,
    /// Emit zero position and velocity, or the identity attitude and zero
    /// rate, without querying SPICE.
    Disabled,
}

/// The two channels of a planet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Channel {
    Position,
    Orientation,
}

/// One channel's configuration (upstream `ChannelCfg`).
#[derive(Clone, Copy, Debug, Default)]
struct ChannelConfig {
    mode: ChannelMode,
    settings: ReconstructionSettings,
}

/// A position knot \[m, m/s\]; `None` if its SPICE query failed, which is
/// queried again on later calls.
type PositionKnot = Option<(Vector3<f64>, Vector3<f64>)>;

/// An orientation knot, the inertial-to-fixed DCM and its rate; `None` if
/// its SPICE query failed.
type OrientationKnot = Option<(Matrix3<f64>, Matrix3<f64>)>;

/// Per-planet state (upstream `PlanetInterp`): the two channels and their
/// rolling knot caches, keyed by knot index.
#[derive(Clone, Debug, Default)]
struct PlanetInterp {
    position: ChannelConfig,
    orientation: ChannelConfig,
    position_knots: BTreeMap<i64, PositionKnot>,
    orientation_knots: BTreeMap<i64, OrientationKnot>,
}

/// The SPICE queries a knot fill makes; the implementation counts them.
pub(crate) trait KnotSource {
    /// Position \[m\] and velocity \[m/s\] of `planet` at ephemeris time `et`.
    fn position(&mut self, planet: &str, et: f64) -> Result<(Vector3<f64>, Vector3<f64>), String>;

    /// Inertial-to-`frame` DCM of `planet` and its rate at ephemeris time
    /// `et`.
    fn orientation(
        &mut self,
        planet: &str,
        frame: &str,
        et: f64,
    ) -> Result<(Matrix3<f64>, Matrix3<f64>), String>;
}

/// The reconstruction state of every configured planet.
#[derive(Clone, Debug, Default)]
pub(crate) struct Reconstruction {
    /// Upstream `planets`, keyed by uppercase name.
    planets: BTreeMap<String, PlanetInterp>,
    /// Upstream `et0` \[s\]: ephemeris time at simulation time zero.
    et0: f64,
}

impl Reconstruction {
    /// An empty reconstruction with its knot grid anchored at `et0` (upstream
    /// `reconState`, which anchors it at `J2000ETInit` once that is set).
    pub(crate) fn new(et0: f64) -> Self {
        Self {
            planets: BTreeMap::new(),
            et0,
        }
    }

    /// Upstream `entry`: the planet's state, created if absent.
    fn entry(&mut self, planet: &str) -> &mut PlanetInterp {
        self.planets.entry(normalize_body_name(planet)).or_default()
    }

    /// Upstream `configure`: puts one channel of `planet` in reconstruct mode
    /// and drops its cached knots, which are indices on the old grid. The
    /// caller has checked that the knot step is positive.
    pub(crate) fn configure(
        &mut self,
        planet: &str,
        channel: Channel,
        settings: ReconstructionSettings,
    ) {
        let interp = self.entry(planet);
        let config = ChannelConfig {
            mode: ChannelMode::Reconstruct,
            settings: ReconstructionSettings {
                n_knots: settings.n_knots.max(1),
                ..settings
            },
        };
        match channel {
            Channel::Position => {
                interp.position = config;
                interp.position_knots.clear();
            }
            Channel::Orientation => {
                interp.orientation = config;
                interp.orientation_knots.clear();
            }
        }
    }

    /// Sets the mode of one channel of `planet`, keeping its settings.
    pub(crate) fn set_mode(&mut self, planet: &str, channel: Channel, mode: ChannelMode) {
        let interp = self.entry(planet);
        match channel {
            Channel::Position => interp.position.mode = mode,
            Channel::Orientation => interp.orientation.mode = mode,
        }
    }

    /// Upstream `reset`: anchors the grid at `et0`, clears every cache, drops
    /// planets that are not among `planets` (name, has an orientation) and
    /// returns orientation reconstruction of a frameless planet to the exact
    /// path, each with a warning.
    pub(crate) fn reset(&mut self, et0: f64, planets: &[(&str, bool)]) {
        self.et0 = et0;
        self.planets.retain(|name, interp| {
            interp.position_knots.clear();
            interp.orientation_knots.clear();
            let Some(&(_, orientable)) = planets
                .iter()
                .find(|(planet, _)| normalize_body_name(planet) == *name)
            else {
                log::warn!(
                    "spiceInterface: planet '{name}' was configured for reconstruction but was \
                     not added via addPlanetNames; ignoring it."
                );
                return false;
            };
            if interp.orientation.mode == ChannelMode::Reconstruct && !orientable {
                log::warn!(
                    "spiceInterface: planet '{name}' orientation reconstruction was requested, \
                     but the body has no orientation frame (computeOrient is off); leaving \
                     orientation on the exact (identity) path."
                );
                interp.orientation.mode = ChannelMode::Exact;
            }
            true
        });
    }

    /// Upstream `position`: position \[m\] and velocity \[m/s\] of `planet` at
    /// simulation time `t_nanos`, or `None` when the exact SPICE path must
    /// run: the channel is exact, the planet is not configured, or a stencil
    /// knot could not be queried (a look-ahead knot past kernel coverage, for
    /// one).
    pub(crate) fn position(
        &mut self,
        planet: &str,
        t_nanos: u64,
        source: &mut dyn KnotSource,
    ) -> Option<(Vector3<f64>, Vector3<f64>)> {
        let et0 = self.et0;
        let interp = self.planets.get_mut(&normalize_body_name(planet))?;
        match interp.position.mode {
            ChannelMode::Exact => return None,
            ChannelMode::Disabled => return Some((Vector3::zeros(), Vector3::zeros())),
            ChannelMode::Reconstruct => {}
        }

        let settings = interp.position.settings;
        let kf = knot_coordinate(t_nanos, settings.knot_step_nanos);
        let knot_step_s = settings.knot_step_nanos as f64 * NANO2SEC;
        let stencil = build_stencil(kf, settings.n_knots, settings.interpolate);
        // Every knot is ensured, even after one fails, as upstream.
        let knots: Vec<_> = stencil
            .iter()
            .map(|&index| {
                ensure_position_knot(interp, planet, index, et0, source).map(|knot| (index, knot))
            })
            .collect();
        let knots: Vec<_> = knots.into_iter().collect::<Option<_>>()?;

        // Confluent Newton nodes: a knot repeats when its velocity is matched,
        // the velocity scaled to per-knot units.
        let repetitions = if settings.use_velocity { 2 } else { 1 };
        let mut nodes = Vec::with_capacity(knots.len() * repetitions);
        let mut values = Vec::with_capacity(nodes.capacity());
        let mut derivatives = Vec::with_capacity(nodes.capacity());
        for (index, (position, velocity)) in knots {
            for _ in 0..repetitions {
                nodes.push(index as f64);
                values.push(position);
                derivatives.push(velocity * knot_step_s);
            }
        }
        let coefficients = newton_coefficients(&values, &derivatives, &nodes);
        let (position, d_position_dk) = newton_evaluate(&coefficients, &nodes, kf);
        // The velocity is the time derivative of the same polynomial.
        Some((position, d_position_dk / knot_step_s))
    }

    /// Upstream `orientation`: inertial-to-fixed DCM of `planet` and its rate
    /// at simulation time `t_nanos`, or `None` when the exact SPICE path must
    /// run (see [`Self::position`]).
    pub(crate) fn orientation(
        &mut self,
        planet: &str,
        frame: &str,
        t_nanos: u64,
        source: &mut dyn KnotSource,
    ) -> Option<(Matrix3<f64>, Matrix3<f64>)> {
        let et0 = self.et0;
        let interp = self.planets.get_mut(&normalize_body_name(planet))?;
        match interp.orientation.mode {
            ChannelMode::Exact => return None,
            ChannelMode::Disabled => return Some((Matrix3::identity(), Matrix3::zeros())),
            ChannelMode::Reconstruct => {}
        }

        let settings = interp.orientation.settings;
        let kf = knot_coordinate(t_nanos, settings.knot_step_nanos);
        let knot_step_s = settings.knot_step_nanos as f64 * NANO2SEC;
        let stencil = build_stencil(kf, settings.n_knots, settings.interpolate);
        let knots: Vec<_> = stencil
            .iter()
            .map(|&index| {
                ensure_orientation_knot(interp, planet, frame, index, et0, source)
                    .map(|knot| (index, knot))
            })
            .collect();
        let knots: Vec<_> = knots.into_iter().collect::<Option<_>>()?;

        // Each knot's attitude relative to the first stencil knot, a fixed
        // base for the whole interval, as a principal rotation vector. A base
        // that changed within the interval (the nearest knot, say) would make
        // the reconstruction jump there for a tumbling body.
        let base_dcm = knots[0].1.0;
        let repetitions = if settings.use_velocity { 2 } else { 1 };
        let mut nodes = Vec::with_capacity(knots.len() * repetitions);
        let mut thetas = Vec::with_capacity(nodes.capacity());
        let mut theta_rates = Vec::with_capacity(nodes.capacity());
        for (index, (dcm, dcm_dot)) in knots {
            let theta = dcm_to_prv(dcm * base_dcm.transpose());
            let theta_rate = if settings.use_velocity {
                // Ċ = -[ω×] C, so ω = unhat(-Ċ Cᵀ); the relative attitude has
                // the same ω, the base being constant.
                let w = -dcm_dot * dcm.transpose();
                let omega = Vector3::new(w[(2, 1)], w[(0, 2)], w[(1, 0)]);
                if theta.norm() < SMALL_PRV_RAD {
                    omega * knot_step_s
                } else {
                    d_omega_to_prv(theta, omega) * knot_step_s
                }
            } else {
                Vector3::zeros()
            };
            for _ in 0..repetitions {
                nodes.push(index as f64);
                thetas.push(theta);
                theta_rates.push(theta_rate);
            }
        }

        let coefficients = newton_coefficients(&thetas, &theta_rates, &nodes);
        let dcm_at = |k: f64| prv_to_dcm(newton_evaluate(&coefficients, &nodes, k).0) * base_dcm;
        let dcm = dcm_at(kf);
        // The rate is a central difference of the same interpolant, in knot
        // units so it does not depend on the knot spacing.
        let h = ORIENTATION_RATE_STEP;
        let dcm_dot = (dcm_at(kf + h) - dcm_at(kf - h)) / (2.0 * h * knot_step_s);
        Some((dcm, dcm_dot))
    }
}

/// Upstream `ensurePosKnot`: knot `index` of the planet's position, queried
/// unless a good one is cached; `None` if the query failed.
fn ensure_position_knot(
    interp: &mut PlanetInterp,
    planet: &str,
    index: i64,
    et0: f64,
    source: &mut dyn KnotSource,
) -> PositionKnot {
    if let Some(&Some(knot)) = interp.position_knots.get(&index) {
        return Some(knot);
    }
    let et = et0 + index as f64 * (interp.position.settings.knot_step_nanos as f64 * NANO2SEC);
    let knot = source
        .position(planet, et)
        .map_err(|message| {
            log::warn!(
                "spiceInterface: position knot query failed for '{planet}' at ET {et}; using \
                 zero state for that knot (will retry on later calls). {message}"
            );
        })
        .ok();
    interp.position_knots.insert(index, knot);
    evict_knots(
        &mut interp.position_knots,
        interp.position.settings.n_knots,
        index,
    );
    knot
}

/// Upstream `ensureOrientKnot`: knot `index` of the planet's orientation,
/// queried unless a good one is cached; `None` if the query failed.
fn ensure_orientation_knot(
    interp: &mut PlanetInterp,
    planet: &str,
    frame: &str,
    index: i64,
    et0: f64,
    source: &mut dyn KnotSource,
) -> OrientationKnot {
    if let Some(&Some(knot)) = interp.orientation_knots.get(&index) {
        return Some(knot);
    }
    let et = et0 + index as f64 * (interp.orientation.settings.knot_step_nanos as f64 * NANO2SEC);
    let knot = source
        .orientation(planet, frame, et)
        .map_err(|message| {
            log::warn!(
                "spiceInterface: orientation knot query failed for '{planet}' (frame {frame}) at \
                 ET {et}; using identity for that knot (will retry on later calls). {message}"
            );
        })
        .ok();
    interp.orientation_knots.insert(index, knot);
    evict_knots(
        &mut interp.orientation_knots,
        interp.orientation.settings.n_knots,
        index,
    );
    knot
}

/// Upstream `evictKnots`: drops the knots farthest from `index` until the
/// stencil and [`CACHE_SLACK`] remain.
fn evict_knots<K>(knots: &mut BTreeMap<i64, K>, n_knots: usize, index: i64) {
    let capacity = n_knots.max(1) + CACHE_SLACK;
    while knots.len() > capacity {
        let (Some(&low), Some(&high)) = (knots.keys().next(), knots.keys().next_back()) else {
            return;
        };
        if (high - index).abs() >= (low - index).abs() {
            knots.remove(&high);
        } else {
            knots.remove(&low);
        }
    }
}

/// Upstream `knotCoord`: the knot coordinate of `t_nanos` on a grid
/// `knot_step_nanos` apart, from integer nanoseconds so a time on a knot is
/// an exact integer.
pub(crate) fn knot_coordinate(t_nanos: u64, knot_step_nanos: u64) -> f64 {
    let k = t_nanos / knot_step_nanos;
    let remainder = t_nanos - k * knot_step_nanos;
    k as f64 + remainder as f64 / knot_step_nanos as f64
}

/// Upstream `buildStencil`: `n_knots` knot indices centred on the interval
/// `[k, k+1]` holding `kf` (interpolation) or ending at `k` (extrapolation).
/// A single knot is `k` in both modes.
pub(crate) fn build_stencil(kf: f64, n_knots: usize, interpolate: bool) -> Vec<i64> {
    let k = kf.floor() as i64;
    let m = n_knots.max(1) as i64;
    let start = if m == 1 {
        k
    } else if interpolate {
        k - (m - 1) / 2
    } else {
        k - (m - 1)
    };
    (start..start + m).collect()
}

/// Upstream `newtonCoeffs`: Newton divided-difference coefficients over
/// the (possibly confluent) `nodes`, taking `derivatives` where two nodes
/// coincide.
pub(crate) fn newton_coefficients(
    values: &[Vector3<f64>],
    derivatives: &[Vector3<f64>],
    nodes: &[f64],
) -> Vec<Vector3<f64>> {
    let n = nodes.len();
    let mut previous = values.to_vec();
    let mut coefficients = vec![previous[0]];
    for column in 1..n {
        let current: Vec<_> = (0..n - column)
            .map(|i| {
                if nodes[i + column] == nodes[i] {
                    derivatives[i]
                } else {
                    (previous[i + 1] - previous[i]) / (nodes[i + column] - nodes[i])
                }
            })
            .collect();
        coefficients.push(current[0]);
        previous = current;
    }
    coefficients
}

/// Upstream `newtonEval`: the Newton-form polynomial at `x` by Horner's rule,
/// and its derivative by the product-rule recurrence.
pub(crate) fn newton_evaluate(
    coefficients: &[Vector3<f64>],
    nodes: &[f64],
    x: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let n = nodes.len();
    let mut result = coefficients[n - 1];
    let mut derivative = Vector3::zeros();
    for i in (0..n - 1).rev() {
        derivative = derivative * (x - nodes[i]) + result;
        result = result * (x - nodes[i]) + coefficients[i];
    }
    (result, derivative)
}

/// Upstream `normalizeBodyName`: the uppercase name, the case-insensitive
/// key.
pub(crate) fn normalize_body_name(name: &str) -> String {
    name.to_ascii_uppercase()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A source with a cubic position and a constant-rate spin about z, that
    /// fails at the listed ephemeris times.
    #[derive(Default)]
    struct Synthetic {
        failing_et: Vec<f64>,
        position_queries: usize,
        orientation_queries: usize,
    }

    const RATE_RADPS: f64 = 7.292_115e-5;

    fn cubic(t: f64) -> (Vector3<f64>, Vector3<f64>) {
        let position = Vector3::new(
            1.0e3 + 2.0 * t - 3.0e-3 * t * t + 1.0e-6 * t * t * t,
            -t,
            5.0,
        );
        let velocity = Vector3::new(2.0 - 6.0e-3 * t + 3.0e-6 * t * t, -1.0, 0.0);
        (position, velocity)
    }

    fn spin(t: f64) -> (Matrix3<f64>, Matrix3<f64>) {
        let (s, c) = (RATE_RADPS * t).sin_cos();
        let dcm = Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0);
        let dcm_dot = RATE_RADPS * Matrix3::new(-s, c, 0.0, -c, -s, 0.0, 0.0, 0.0, 0.0);
        (dcm, dcm_dot)
    }

    impl Synthetic {
        fn fails_at(&self, et: f64) -> bool {
            self.failing_et
                .iter()
                .any(|failing| (failing - et).abs() < 1.0e-6)
        }
    }

    impl KnotSource for Synthetic {
        fn position(&mut self, _: &str, et: f64) -> Result<(Vector3<f64>, Vector3<f64>), String> {
            self.position_queries += 1;
            if self.fails_at(et) {
                return Err("no coverage".to_string());
            }
            Ok(cubic(et))
        }

        fn orientation(
            &mut self,
            _: &str,
            _: &str,
            et: f64,
        ) -> Result<(Matrix3<f64>, Matrix3<f64>), String> {
            self.orientation_queries += 1;
            if self.fails_at(et) {
                return Err("no coverage".to_string());
            }
            Ok(spin(et))
        }
    }

    fn settings(
        knot_step_s: u64,
        n_knots: usize,
        use_velocity: bool,
        interpolate: bool,
    ) -> ReconstructionSettings {
        ReconstructionSettings {
            knot_step_nanos: knot_step_s * 1_000_000_000,
            n_knots,
            use_velocity,
            interpolate,
        }
    }

    #[test]
    fn a_time_on_a_knot_has_an_integer_coordinate() {
        let step = 60_000_000_000;
        assert_eq!(knot_coordinate(3 * step, step), 3.0);
        assert_eq!(knot_coordinate(3 * step + step / 4, step), 3.25);
        assert_eq!(knot_coordinate(0, step), 0.0);
    }

    #[test]
    fn stencils_are_centred_or_trailing() {
        for interpolate in [true, false] {
            assert_eq!(build_stencil(5.5, 1, interpolate), vec![5]);
            assert_eq!(build_stencil(5.5, 0, interpolate), vec![5]);
        }
        assert_eq!(build_stencil(5.5, 2, true), vec![5, 6]);
        assert_eq!(build_stencil(5.5, 3, true), vec![4, 5, 6]);
        assert_eq!(build_stencil(5.5, 4, true), vec![4, 5, 6, 7]);
        assert_eq!(build_stencil(5.5, 2, false), vec![4, 5]);
        assert_eq!(build_stencil(5.5, 3, false), vec![3, 4, 5]);
        assert_eq!(build_stencil(-0.5, 2, true), vec![-1, 0]);
    }

    #[test]
    fn two_hermite_knots_reproduce_a_cubic_and_its_derivative() {
        let nodes = [0.0, 0.0, 1.0, 1.0];
        let f = |x: f64| Vector3::new(1.0 + 2.0 * x - x * x + 0.5 * x * x * x, 3.0, -x);
        let df = |x: f64| Vector3::new(2.0 - 2.0 * x + 1.5 * x * x, 0.0, -1.0);
        let values = [f(0.0), f(0.0), f(1.0), f(1.0)];
        let derivatives = [df(0.0), df(0.0), df(1.0), df(1.0)];
        let coefficients = newton_coefficients(&values, &derivatives, &nodes);
        for x in [0.0, 0.3, 0.5, 1.0, 1.7] {
            let (value, derivative) = newton_evaluate(&coefficients, &nodes, x);
            assert!((value - f(x)).norm() < 1.0e-12, "x = {x}");
            assert!((derivative - df(x)).norm() < 1.0e-12, "x = {x}");
        }
    }

    #[test]
    fn eviction_keeps_the_knots_nearest_the_new_one() {
        let mut knots: BTreeMap<i64, ()> = (0..10).map(|index| (index, ())).collect();
        evict_knots(&mut knots, 2, 9);
        assert_eq!(
            knots.keys().copied().collect::<Vec<_>>(),
            vec![4, 5, 6, 7, 8, 9]
        );
        evict_knots(&mut knots, 2, 4);
        assert_eq!(knots.len(), 6);
    }

    #[test]
    fn a_cubic_position_is_reconstructed_exactly_with_few_queries() {
        let mut reconstruction = Reconstruction::new(100.0);
        reconstruction.configure("Earth", Channel::Position, settings(60, 2, true, true));
        let mut source = Synthetic::default();
        for t_s in [0, 10, 30, 59, 60, 61, 119, 120, 150] {
            let (position, velocity) = reconstruction
                .position("EARTH", t_s * 1_000_000_000, &mut source)
                .expect("reconstructed");
            let (truth, truth_rate) = cubic(100.0 + t_s as f64);
            assert!((position - truth).norm() < 1.0e-9, "t = {t_s} s");
            assert!((velocity - truth_rate).norm() < 1.0e-12, "t = {t_s} s");
        }
        // Knots 0..=3 only.
        assert_eq!(source.position_queries, 4);
    }

    #[test]
    fn a_constant_spin_is_reconstructed_from_one_knot() {
        let mut reconstruction = Reconstruction::new(0.0);
        reconstruction.configure(
            "earth",
            Channel::Orientation,
            settings(100_000, 1, true, true),
        );
        let mut source = Synthetic::default();
        for t_s in [0, 600, 3600, 50_000] {
            let (dcm, dcm_dot) = reconstruction
                .orientation("earth", "IAU_EARTH", t_s * 1_000_000_000, &mut source)
                .expect("reconstructed");
            let (truth, truth_rate) = spin(t_s as f64);
            assert!((dcm - truth).abs().max() < 1.0e-12, "t = {t_s} s");
            assert!((dcm_dot - truth_rate).abs().max() < 1.0e-12, "t = {t_s} s");
        }
        assert_eq!(source.orientation_queries, 1);
    }

    #[test]
    fn a_failed_knot_falls_back_to_the_exact_path_and_is_retried() {
        let mut reconstruction = Reconstruction::new(0.0);
        reconstruction.configure("earth", Channel::Position, settings(60, 2, true, true));
        let mut source = Synthetic {
            failing_et: vec![60.0],
            ..Synthetic::default()
        };
        assert!(
            reconstruction
                .position("earth", 30_000_000_000, &mut source)
                .is_none()
        );
        source.failing_et.clear();
        assert!(
            reconstruction
                .position("earth", 30_000_000_000, &mut source)
                .is_some()
        );
        // Knot 0 kept, knot 1 asked twice.
        assert_eq!(source.position_queries, 3);
    }

    #[test]
    fn exact_disabled_and_unconfigured_channels() {
        let mut reconstruction = Reconstruction::new(0.0);
        let mut source = Synthetic::default();
        assert!(reconstruction.position("earth", 0, &mut source).is_none());
        reconstruction.set_mode("earth", Channel::Position, ChannelMode::Disabled);
        reconstruction.set_mode("earth", Channel::Orientation, ChannelMode::Disabled);
        assert_eq!(
            reconstruction.position("earth", 0, &mut source),
            Some((Vector3::zeros(), Vector3::zeros()))
        );
        assert_eq!(
            reconstruction.orientation("earth", "IAU_EARTH", 0, &mut source),
            Some((Matrix3::identity(), Matrix3::zeros()))
        );
        reconstruction.set_mode("earth", Channel::Position, ChannelMode::Exact);
        assert!(reconstruction.position("earth", 0, &mut source).is_none());
        assert_eq!(source.position_queries + source.orientation_queries, 0);
    }

    #[test]
    fn reset_drops_unknown_planets_and_frameless_orientation() {
        let mut reconstruction = Reconstruction::new(0.0);
        reconstruction.configure("jupiter", Channel::Position, settings(60, 2, true, true));
        reconstruction.configure(
            "mars barycenter",
            Channel::Orientation,
            settings(60, 2, true, true),
        );
        reconstruction.reset(5.0, &[("Mars Barycenter", false)]);
        assert_eq!(reconstruction.et0, 5.0);
        assert!(!reconstruction.planets.contains_key("JUPITER"));
        assert_eq!(
            reconstruction.planets["MARS BARYCENTER"].orientation.mode,
            ChannelMode::Exact
        );
    }
}
