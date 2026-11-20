//! Interpolated message sources: port of upstream Basilisk `MJInterpolators`
//! (`SingleActuatorInterpolator`, `ScalarJointStateInterpolator`) and the
//! `Interpolator` template of `simulation/dynamics/_GeneralModuleFiles`.
//!
//! An interpolator publishes, at every update, a spline through a table of
//! `(time [ns], value)` points: degree 0 is piecewise constant (each value
//! holds until the next point), degree 1 piecewise linear, degree 3 a cubic
//! with continuous first and second derivatives. Before the first and after
//! the last point the spline holds its end values. Upstream fits the spline
//! with Eigen's `SplineFitting::Interpolate` on times scaled to `[0, 1]`
//! with averaged knots; [`Spline`] is that algorithm. Degree 0, which Eigen
//! does not support, is upstream's degree-1 spline through the points
//! doubled one nanosecond before each next point.
//!
//! Interpolators are [`Module`]s and [`SceneModel`]s: add one to the
//! simulation, or to a scene's dynamics task (upstream's usual choice, so
//! the commanded value follows every integrator stage). Upstream `Reset`
//! publishes the value at the reset time.

use super::task::{SceneModel, SceneView, TaskContext};
use crate::messages::{Output, ScalarJointStateMsg, SingleActuatorMsg};
use crate::{Module, SimulationContext};

/// Errors of [`Spline::fit`] and of the interpolators' `set_data_points`
/// (upstream raises `std::invalid_argument`).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum InterpolatorError {
    /// Fewer points than the degree plus one.
    TooFewPoints,
    /// All points have the same time.
    DegenerateTimes,
}

impl std::fmt::Display for InterpolatorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::TooFewPoints => write!(
                f,
                "Provided 'degrees' is higher or equal than number of data points (illegal for \
                 interpolation)."
            ),
            Self::DegenerateTimes => write!(
                f,
                "First column of points (xp) contains only one element or all elements are the \
                 same."
            ),
        }
    }
}

impl std::error::Error for InterpolatorError {}

/// An interpolating B-spline of a scalar over a scaled parameter (Eigen's
/// `Spline<double, 1>` as fitted by `SplineFitting::Interpolate`).
#[derive(Clone, Debug, PartialEq)]
pub struct Spline {
    degree: usize,
    knots: Vec<f64>,
    control_points: Vec<f64>,
    x_min: f64,
    x_max: f64,
}

impl Spline {
    /// Upstream `setDataPoints(points, degrees)`: the spline of `degree`
    /// through `(x, y)` points (`x` the time \[ns\]).
    ///
    /// # Errors
    ///
    /// If there are not more points than `degree`, or every point has the
    /// same time.
    pub fn fit(points: &[(f64, f64)], degree: usize) -> Result<Self, InterpolatorError> {
        if points.len() <= degree {
            return Err(InterpolatorError::TooFewPoints);
        }
        if degree == 0 {
            // Each value holds until one nanosecond before the next point.
            let mut doubled = Vec::with_capacity(2 * points.len() - 1);
            for pair in points.windows(2) {
                doubled.push(pair[0]);
                doubled.push((pair[1].0 - 1.0, pair[0].1));
            }
            doubled.push(*points.last().expect("at least one point"));
            return Self::fit_scaled(&doubled, 1);
        }
        Self::fit_scaled(points, degree)
    }

    fn fit_scaled(points: &[(f64, f64)], degree: usize) -> Result<Self, InterpolatorError> {
        let x_max = points.iter().map(|point| point.0).fold(f64::MIN, f64::max);
        let x_min = points.iter().map(|point| point.0).fold(f64::MAX, f64::min);
        if x_max == x_min {
            return Err(InterpolatorError::DegenerateTimes);
        }
        let parameters: Vec<f64> = points
            .iter()
            .map(|point| (point.0 - x_min) / (x_max - x_min))
            .collect();
        let n = points.len();

        // Eigen `KnotAveraging`.
        let mut knots = vec![0.0; n + degree + 1];
        for j in 1..n - degree {
            knots[j + degree] = parameters[j..j + degree].iter().sum::<f64>() / degree as f64;
        }
        for knot in &mut knots[n..] {
            *knot = 1.0;
        }

        // Eigen `SplineFitting::Interpolate`: the collocation system.
        let mut system = nalgebra::DMatrix::zeros(n, n);
        system[(0, 0)] = 1.0;
        system[(n - 1, n - 1)] = 1.0;
        for (i, &u) in parameters.iter().enumerate().take(n - 1).skip(1) {
            let span = span(u, degree, &knots);
            let basis = basis_functions(u, degree, &knots, span);
            for (k, value) in basis.iter().enumerate() {
                system[(i, span - degree + k)] = *value;
            }
        }
        let values = nalgebra::DVector::from_iterator(n, points.iter().map(|point| point.1));
        let control_points = system
            .full_piv_lu()
            .solve(&values)
            .expect("the collocation matrix of distinct parameters is invertible");

        Ok(Self {
            degree,
            knots,
            control_points: control_points.as_slice().to_vec(),
            x_min,
            x_max,
        })
    }

    /// The spline at time `x` \[ns\], held at the end values outside the
    /// data (upstream `spline(scale(x))`).
    pub fn evaluate(&self, x: f64) -> f64 {
        let u = ((x - self.x_min) / (self.x_max - self.x_min)).clamp(0.0, 1.0);
        let span = span(u, self.degree, &self.knots);
        basis_functions(u, self.degree, &self.knots, span)
            .iter()
            .enumerate()
            .map(|(k, basis)| basis * self.control_points[span - self.degree + k])
            .sum()
    }
}

/// Eigen `Spline::Span`: the knot span of `u`.
fn span(u: f64, degree: usize, knots: &[f64]) -> usize {
    if u <= knots[0] {
        return degree;
    }
    let search = &knots[degree - 1..knots.len() - degree - 1];
    let upper = search.partition_point(|knot| *knot <= u);
    degree - 1 + upper - 1
}

/// Eigen `Spline::BasisFunctions` (The NURBS Book, algorithm A2.2): the
/// `degree + 1` nonzero basis functions at `u` in knot span `span`.
fn basis_functions(u: f64, degree: usize, knots: &[f64], span: usize) -> Vec<f64> {
    let mut basis = vec![0.0; degree + 1];
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];
    basis[0] = 1.0;
    for j in 1..=degree {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;
        for r in 0..j {
            let temp = basis[r] / (right[r + 1] + left[j - r]);
            basis[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }
        basis[j] = saved;
    }
    basis
}

macro_rules! interpolator {
    ($(#[$doc:meta])* $name:ident, $msg:ident, $field:ident) => {
        $(#[$doc])*
        #[derive(Debug)]
        pub struct $name {
            /// Upstream `ModelTag`.
            pub name: String,
            /// Upstream `interpolatedOutMsg`.
            pub interpolated_out_msg: Output<$msg>,
            spline: Option<Spline>,
        }

        impl Default for $name {
            fn default() -> Self {
                Self {
                    name: stringify!($name).to_string(),
                    interpolated_out_msg: Output::default(),
                    spline: None,
                }
            }
        }

        impl $name {
            /// Upstream `setDataPoints(points, degrees)`: `(time [ns],
            /// value)` points and the spline degree (upstream's default is
            /// 1).
            ///
            /// # Errors
            ///
            /// See [`Spline::fit`].
            pub fn set_data_points(
                &mut self,
                points: &[(f64, f64)],
                degree: usize,
            ) -> Result<(), InterpolatorError> {
                self.spline = Some(Spline::fit(points, degree)?);
                Ok(())
            }

            /// The interpolated value at `nanos`.
            ///
            /// # Panics
            ///
            /// Before [`set_data_points`](Self::set_data_points) (upstream:
            /// "Tried to call Reset or UpdateState on Interpolator before
            /// setDataPoints was called").
            pub fn value_at(&self, nanos: u64) -> f64 {
                self.spline
                    .as_ref()
                    .expect(
                        "Tried to call Reset or UpdateState on Interpolator before setDataPoints \
                         was called.",
                    )
                    .evaluate(nanos as f64)
            }

            /// Upstream `UpdateState`.
            fn publish(&self, nanos: u64) {
                self.interpolated_out_msg.write($msg {
                    $field: self.value_at(nanos),
                });
            }
        }

        impl Module for $name {
            fn init(&mut self) {
                self.publish(0);
            }

            fn reset(&mut self, context: &SimulationContext) {
                self.publish(context.current_sim_nanos);
            }

            fn update(&mut self, context: &SimulationContext) {
                self.publish(context.current_sim_nanos);
            }
        }

        impl SceneModel for $name {
            fn model_tag(&self) -> &str {
                &self.name
            }

            fn scene_reset(&mut self, _scene: &SceneView<'_>, context: &SimulationContext) {
                self.publish(context.current_sim_nanos);
            }

            fn scene_update(&mut self, context: &mut TaskContext<'_>) {
                self.publish(context.current_sim_nanos());
            }
        }
    };
}

interpolator!(
    /// Upstream `SingleActuatorInterpolator`: publishes the interpolated
    /// value as an actuator command.
    SingleActuatorInterpolator,
    SingleActuatorMsg,
    input
);

interpolator!(
    /// Upstream `ScalarJointStateInterpolator`: publishes the interpolated
    /// value as a joint state (for example a joint's constrained state or a
    /// controller's desired state).
    ScalarJointStateInterpolator,
    ScalarJointStateMsg,
    state
);

#[cfg(test)]
mod tests {
    //! Upstream has no unit test of its own for the interpolators (they run
    //! in `test_JointPIDController.py`); these check the spline cases the
    //! template documents.

    use super::*;

    #[test]
    fn degree_zero_holds_each_value_until_the_next_point() {
        let spline = Spline::fit(&[(0.0, 5.0), (1e9, 20.0), (3e9, 50.0)], 0).expect("fit");
        assert_eq!(spline.evaluate(0.5e9), 5.0);
        assert_eq!(spline.evaluate(1.5e9), 20.0);
        assert_eq!(spline.evaluate(1e9), 20.0);
        assert_eq!(spline.evaluate(3e9), 50.0);
        assert!((spline.evaluate(1e9 - 1.0) - 5.0).abs() < 1e-9);
    }

    #[test]
    fn degree_one_is_linear_and_holds_the_end_values() {
        let spline = Spline::fit(&[(0.0, 0.0), (2.0, 4.0), (4.0, 0.0)], 1).expect("fit");
        assert!((spline.evaluate(1.0) - 2.0).abs() < 1e-12);
        assert!((spline.evaluate(3.0) - 2.0).abs() < 1e-12);
        assert_eq!(spline.evaluate(-5.0), 0.0);
        assert!(spline.evaluate(10.0).abs() < 1e-12);
    }

    #[test]
    fn cubic_spline_passes_through_the_points_and_reproduces_a_cubic() {
        let cubic = |x: f64| 1.0 - 2.0 * x + 0.5 * x * x * x;
        let points: Vec<(f64, f64)> = [0.0, 1.0, 1.5, 3.0, 4.0, 6.0]
            .iter()
            .map(|&x| (x, cubic(x)))
            .collect();
        let spline = Spline::fit(&points, 3).expect("fit");
        for &(x, y) in &points {
            assert!((spline.evaluate(x) - y).abs() < 1e-10, "at {x}");
        }
        // A cubic is in the space of cubic splines, so it is reproduced.
        for x in [0.3, 2.2, 5.1] {
            assert!((spline.evaluate(x) - cubic(x)).abs() < 1e-9, "at {x}");
        }
    }

    #[test]
    fn rejects_too_few_points_and_degenerate_times() {
        assert_eq!(
            Spline::fit(&[(0.0, 1.0), (1.0, 2.0)], 2),
            Err(InterpolatorError::TooFewPoints)
        );
        assert_eq!(
            Spline::fit(&[(1.0, 1.0), (1.0, 2.0)], 1),
            Err(InterpolatorError::DegenerateTimes)
        );
    }

    #[test]
    fn interpolators_publish_the_value_at_the_update_time() {
        let mut interpolator = ScalarJointStateInterpolator::default();
        interpolator
            .set_data_points(&[(0.0, 0.0), (10e9, 1.0)], 1)
            .expect("points");
        Module::init(&mut interpolator);
        assert_eq!(interpolator.interpolated_out_msg.read().state, 0.0);
        let context = SimulationContext {
            current_sim_nanos: 2_500_000_000,
            current_epoch: crate::mujoco_dynamics::tests::epoch(),
        };
        Module::update(&mut interpolator, &context);
        assert!((interpolator.interpolated_out_msg.read().state - 0.25).abs() < 1e-12);

        let mut actuator = SingleActuatorInterpolator::default();
        actuator
            .set_data_points(&[(0.0, 2.0), (1e9, 4.0)], 0)
            .expect("points");
        Module::update(&mut actuator, &context);
        assert_eq!(actuator.interpolated_out_msg.read().input, 4.0);
    }

    #[test]
    #[should_panic(expected = "before setDataPoints")]
    fn update_before_data_points_panics() {
        Module::init(&mut SingleActuatorInterpolator::default());
    }
}
