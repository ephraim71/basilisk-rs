//! B-spline interpolation and least-squares approximation of 3-D waypoints.
//! Port of upstream `architecture/utilities/BSpline`.
//!
//! Both fits work in a normalized parameter `u = t / T_tot ∈ [0, 1]`, where
//! `t` are the waypoint time tags and `T_tot` the last one, and sample the
//! fitted curve and its first two time derivatives at `num` evenly spaced
//! times. Optional first and second derivatives at either end add
//! constraints (upstream `XDot_0`, `XDot_N`, `XDDot_0`, `XDDot_N`).
//!
//! - [`interpolate`] passes exactly through every waypoint: `N + K + 1`
//!   control points for `N + 1` waypoints and `K` end-derivative constraints,
//!   knots placed by averaging the waypoint parameters.
//! - [`approximate`] passes exactly through the end points only and fits the
//!   interior waypoints in the (optionally weighted) least-squares sense with
//!   `Q + 1` control points.

use nalgebra::{DMatrix, DVector, Vector3};

/// How the waypoint time tags are chosen (upstream `T_flag`/`AvgXDot_flag`,
/// which are mutually exclusive).
#[derive(Clone, Debug, Default, PartialEq)]
pub enum WaypointTiming {
    /// Neither flag: each waypoint is tagged with the cumulative chord length
    /// (Cartesian distance) along the waypoints.
    #[default]
    ChordLength,
    /// Upstream `setT`: the time tag of every waypoint.
    TimeTags(Vec<f64>),
    /// Upstream `setAvgXDot`: chord-length tags scaled for this average
    /// velocity norm.
    AverageSpeed(f64),
}

/// The waypoints to fit and the optional constraints: upstream
/// `InputDataSet`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BSplineInput {
    /// Upstream `X1`, `X2`, `X3`: the waypoint coordinates.
    pub waypoints: Vec<Vector3<f64>>,
    /// Upstream `T`/`AvgXDot`: the time tag of each waypoint.
    pub timing: WaypointTiming,
    /// Upstream `W` (optional): least-squares weight of each waypoint, used by
    /// [`approximate`] for the interior ones; unit weights when absent.
    pub weights: Option<Vec<f64>>,
    /// Upstream `XDot_0` (optional): first derivative at the first waypoint.
    pub x_dot_0: Option<Vector3<f64>>,
    /// Upstream `XDot_N` (optional): first derivative at the last waypoint.
    pub x_dot_n: Option<Vector3<f64>>,
    /// Upstream `XDDot_0` (optional): second derivative at the first
    /// waypoint.
    pub x_ddot_0: Option<Vector3<f64>>,
    /// Upstream `XDDot_N` (optional): second derivative at the last waypoint.
    pub x_ddot_n: Option<Vector3<f64>>,
}

impl BSplineInput {
    /// Upstream `InputDataSet(X1, X2, X3)`: the waypoints, chord-length time
    /// tags and no other constraint.
    pub fn new(waypoints: Vec<Vector3<f64>>) -> Self {
        Self {
            waypoints,
            ..Default::default()
        }
    }

    /// Upstream `K`: the number of end-derivative constraints.
    fn end_constraint_count(&self) -> usize {
        [
            self.x_dot_0.is_some(),
            self.x_dot_n.is_some(),
            self.x_ddot_0.is_some(),
            self.x_ddot_n.is_some(),
        ]
        .into_iter()
        .filter(|&set| set)
        .count()
    }

    /// The waypoint time tags `T` upstream's fits compute: given, or the
    /// cumulative chord length, scaled for an average speed when one is set.
    fn time_tags(&self) -> Vec<f64> {
        let waypoints = &self.waypoints;
        if let WaypointTiming::TimeTags(tags) = &self.timing {
            return tags.clone();
        }
        let mut tags = vec![0.0; waypoints.len()];
        let mut length = 0.0;
        for n in 1..waypoints.len() {
            tags[n] = tags[n - 1] + (waypoints[n] - waypoints[n - 1]).norm();
            length += tags[n] - tags[n - 1];
        }
        if let WaypointTiming::AverageSpeed(speed) = self.timing {
            let last = tags[tags.len() - 1];
            for tag in &mut tags {
                *tag = *tag / last * length / speed;
            }
        }
        tags
    }
}

/// A sampled B-spline curve and the spline itself: upstream `OutputDataSet`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct BSplineOutput {
    /// Upstream `T`: the sample times.
    pub t: Vec<f64>,
    /// Upstream `X1`, `X2`, `X3`: the curve at each sample time.
    pub x: Vec<Vector3<f64>>,
    /// Upstream `XD1`, `XD2`, `XD3`: its first time derivative.
    pub x_dot: Vec<Vector3<f64>>,
    /// Upstream `XDD1`, `XDD2`, `XDD3`: its second time derivative.
    pub x_ddot: Vec<Vector3<f64>>,
    /// Upstream `P`: polynomial degree.
    pub degree: usize,
    /// Upstream `U`: knot vector, in the normalized parameter.
    pub knots: Vec<f64>,
    /// Upstream `C1`, `C2`, `C3`: control points.
    pub control_points: Vec<Vector3<f64>>,
}

/// The curve and its first two time derivatives at one time.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BSplineState {
    /// The curve.
    pub x: Vector3<f64>,
    /// Its first time derivative.
    pub x_dot: Vector3<f64>,
    /// Its second time derivative.
    pub x_ddot: Vector3<f64>,
}

impl BSplineOutput {
    /// Upstream `getData`/`getStates`: the curve and its derivatives at time
    /// `t`. After the last sample time the curve holds its last sampled value
    /// with zero derivatives.
    ///
    /// # Panics
    ///
    /// If the output holds no samples.
    pub fn state_at(&self, t: f64) -> BSplineState {
        let last = self.t.len() - 1;
        let t_tot = self.t[last];
        if t <= t_tot {
            let basis = basis_functions(
                t / t_tot,
                &self.knots,
                self.control_points.len(),
                self.degree,
            );
            self.evaluate(&basis, t_tot)
        } else {
            BSplineState {
                x: self.x[last],
                x_dot: Vector3::zeros(),
                x_ddot: Vector3::zeros(),
            }
        }
    }

    /// The curve and its time derivatives from the basis values at one
    /// parameter and the total time `t_tot`.
    fn evaluate(&self, basis: &BasisFunctions, t_tot: f64) -> BSplineState {
        let combine = |weights: &DVector<f64>| {
            self.control_points
                .iter()
                .zip(weights.iter())
                .fold(Vector3::zeros(), |sum, (point, weight)| {
                    sum + point * *weight
                })
        };
        BSplineState {
            x: combine(&basis.n),
            x_dot: combine(&basis.n1) / t_tot,
            x_ddot: combine(&basis.n2) / t_tot.powi(2),
        }
    }

    /// Fills `t`, `x`, `x_dot` and `x_ddot` with `num` samples evenly spaced
    /// in the normalized parameter, accumulated as upstream does.
    fn sample(&mut self, num: usize, t_tot: f64) {
        let dt = 1.0 / (num as f64 - 1.0);
        let mut u = 0.0;
        self.t.clear();
        self.x.clear();
        self.x_dot.clear();
        self.x_ddot.clear();
        for _ in 0..num {
            let basis = basis_functions(u, &self.knots, self.control_points.len(), self.degree);
            let state = self.evaluate(&basis, t_tot);
            self.t.push(u * t_tot);
            self.x.push(state.x);
            self.x_dot.push(state.x_dot);
            self.x_ddot.push(state.x_ddot);
            u += dt;
        }
    }
}

/// B-spline basis functions of one degree at one parameter value, and their
/// first and second derivatives with respect to the parameter (upstream
/// `basisFunction`'s `NN`, `NN1`, `NN2`).
#[derive(Clone, Debug, PartialEq)]
pub struct BasisFunctions {
    /// The basis functions `N_i,P(u)`.
    pub n: DVector<f64>,
    /// Their first derivatives.
    pub n1: DVector<f64>,
    /// Their second derivatives.
    pub n2: DVector<f64>,
}

/// Upstream `basisFunction`: the `count` basis functions of degree `degree`
/// on `knots` at parameter `u`, by the Cox–de Boor recursion.
///
/// As upstream, the zero-degree function of the last span is also one when
/// `u` is within `1e-5` of 1, so the curve is defined at its end. The
/// recursion reads one function past the last; upstream reads past its
/// buffer there (harmless for clamped knots, which zero that term), here it
/// is zero.
pub fn basis_functions(u: f64, knots: &[f64], count: usize, degree: usize) -> BasisFunctions {
    let mut n = DMatrix::<f64>::zeros(count + 1, degree + 1);
    let mut n1 = DMatrix::<f64>::zeros(count + 1, degree + 1);
    let mut n2 = DMatrix::<f64>::zeros(count + 1, degree + 1);
    for i in 0..count {
        if u >= knots[i] && u < knots[i + 1] {
            n[(i, 0)] = 1.0;
        }
    }
    if (u - 1.0).abs() < 1e-5 {
        n[(count - 1, 0)] = 1.0;
    }
    for p in 1..=degree {
        let order = p as f64;
        for i in 0..count {
            let left_span = knots[i + p] - knots[i];
            if left_span != 0.0 {
                n[(i, p)] += (u - knots[i]) / left_span * n[(i, p - 1)];
                n1[(i, p)] += order / left_span * n[(i, p - 1)];
                n2[(i, p)] += order / left_span * n1[(i, p - 1)];
            }
            let right_span = knots[i + p + 1] - knots[i + 1];
            if right_span != 0.0 {
                n[(i, p)] += (knots[i + p + 1] - u) / right_span * n[(i + 1, p - 1)];
                n1[(i, p)] -= order / right_span * n[(i + 1, p - 1)];
                n2[(i, p)] -= order / right_span * n1[(i + 1, p - 1)];
            }
        }
    }
    let column = |m: &DMatrix<f64>| {
        DVector::from_iterator(count, m.column(degree).rows(0, count).iter().copied())
    };
    BasisFunctions {
        n: column(&n),
        n1: column(&n1),
        n2: column(&n2),
    }
}

/// The normalized waypoint parameters `uk = T / T_tot` and `T_tot`.
fn normalized_parameters(tags: &[f64]) -> (Vec<f64>, f64) {
    let t_tot = tags[tags.len() - 1];
    (tags.iter().map(|tag| tag / t_tot).collect(), t_tot)
}

/// Solves `A c = q` for each coordinate as upstream does, through `A⁻¹`.
fn solve_coordinates(a: &DMatrix<f64>, q: &[Vector3<f64>]) -> Vec<Vector3<f64>> {
    let b = a.clone().try_inverse().unwrap_or_else(|| {
        log::error!("BSpline: the linear system is singular");
        DMatrix::from_element(a.nrows(), a.ncols(), f64::NAN)
    });
    (0..b.nrows())
        .map(|row| {
            q.iter()
                .enumerate()
                .fold(Vector3::zeros(), |sum, (col, value)| {
                    sum + value * b[(row, col)]
                })
        })
        .collect()
}

/// Upstream `interpolate`: the degree-`degree` B-spline through every
/// waypoint, sampled at `num` times.
///
/// Logs an error, as upstream, when the degree exceeds `N + K` (the system is
/// then singular).
///
/// # Panics
///
/// If there are fewer than two waypoints, fewer than two samples, or a second
/// derivative is constrained with `degree < 2`.
pub fn interpolate(input: &BSplineInput, num: usize, degree: usize) -> BSplineOutput {
    assert!(
        input.waypoints.len() >= 2,
        "BSpline interpolation needs at least two waypoints"
    );
    assert!(num >= 2, "BSpline interpolation needs at least two samples");
    let p = degree;
    let order = p as f64;
    let n_last = input.waypoints.len() - 1;
    let (uk, t_tot) = normalized_parameters(&input.time_tags());
    let k = input.end_constraint_count();
    if p > n_last + k {
        log::error!(
            "BSpline.interpolate: the desired polynomial order P is too high; the mass matrix A will be singular"
        );
    }

    // Knot vector of size M + 1: P + 1 zeros, averaged waypoint parameters,
    // P + 1 ones.
    let m = n_last + p + k + 1;
    let mut knots = vec![0.0; m + 1];
    for j in 0..(n_last + k).saturating_sub(p) {
        let mut u = 0.0;
        for i in j..j + p {
            u += uk[i.min(n_last)] / order;
        }
        knots[p + j + 1] = u;
    }
    for knot in &mut knots[m - p..] {
        *knot = 1.0;
    }

    // One row per end-derivative constraint and per waypoint.
    let size = n_last + k + 1;
    let mut a = DMatrix::<f64>::zeros(size, size);
    let mut q = vec![Vector3::zeros(); size];
    let mut row = 0;
    if let Some(x_dot_0) = input.x_dot_0 {
        a[(row, 0)] = -1.0;
        a[(row, 1)] = 1.0;
        q[row] = x_dot_0 * (knots[p + 1] / order) * t_tot;
        row += 1;
    }
    if let Some(x_ddot_0) = input.x_ddot_0 {
        a[(row, 0)] = knots[p + 2];
        a[(row, 1)] = -(knots[p + 1] + knots[p + 2]);
        a[(row, 2)] = knots[p + 1];
        let scale = knots[p + 1].powi(2) * knots[p + 2] / (order * (order - 1.0));
        q[row] = x_ddot_0 * scale * t_tot.powi(2);
        row += 1;
    }
    for (waypoint, &u) in input.waypoints.iter().zip(&uk) {
        let basis = basis_functions(u, &knots, size, p);
        a.row_mut(row).copy_from(&basis.n.transpose());
        q[row] = *waypoint;
        row += 1;
    }
    if let Some(x_ddot_n) = input.x_ddot_n {
        a[(row, n_last + k - 2)] = 1.0 - knots[m - p - 1];
        a[(row, n_last + k - 1)] = -(2.0 - knots[m - p - 1] - knots[m - p - 2]);
        a[(row, n_last + k)] = 1.0 - knots[m - p - 2];
        let scale =
            (1.0 - knots[m - p - 1]).powi(2) * (1.0 - knots[m - p - 2]) / (order * (order - 1.0));
        q[row] = x_ddot_n * scale * t_tot.powi(2);
        row += 1;
    }
    if let Some(x_dot_n) = input.x_dot_n {
        a[(row, n_last + k - 1)] = -1.0;
        a[(row, n_last + k)] = 1.0;
        q[row] = x_dot_n * ((1.0 - knots[m - p - 1]) / order) * t_tot;
    }

    let mut output = BSplineOutput {
        degree: p,
        knots,
        control_points: solve_coordinates(&a, &q),
        ..Default::default()
    };
    output.sample(num, t_tot);
    output
}

/// Upstream `approximate`: the degree-`degree` B-spline with `q + 1` control
/// points through the first and last waypoints (and their derivative
/// constraints) that fits the interior waypoints in the weighted
/// least-squares sense, sampled at `num` times.
///
/// Logs an error, as upstream, when the degree exceeds `q`.
///
/// # Panics
///
/// If there are fewer than three waypoints, fewer than two samples, or the
/// knot placement would reach before the first waypoint (`N + 1 < q - P + 1`).
pub fn approximate(input: &BSplineInput, num: usize, q: usize, degree: usize) -> BSplineOutput {
    assert!(
        input.waypoints.len() >= 3,
        "BSpline approximation needs at least three waypoints"
    );
    assert!(num >= 2, "BSpline approximation needs at least two samples");
    let p = degree;
    let n_last = input.waypoints.len() - 1;
    let (uk, t_tot) = normalized_parameters(&input.time_tags());
    if p > q {
        log::error!(
            "BSpline.approximate: the desired polynomial order P can't be higher than the number of control points Q"
        );
    }

    // Knot vector of size Q + P + 2: P + 1 zeros, interior knots interpolated
    // between the waypoint parameters, P + 1 ones.
    let mut knots = vec![0.0; q + p + 2];
    let d = (n_last + 1) as f64 / (q as f64 - p as f64 + 1.0).abs();
    for j in 1..(q + 1).saturating_sub(p) {
        let position = j as f64 * d;
        let knot_index = position as usize;
        let alpha = position - knot_index as f64;
        knots[p + j] = (1.0 - alpha) * uk[knot_index - 1] + alpha * uk[knot_index];
    }
    for knot in &mut knots[q + 1..] {
        *knot = 1.0;
    }

    let k = input.end_constraint_count();
    let count = q + 1;

    // The end points and their derivative constraints fix K + 2 control
    // points.
    let mut md = DMatrix::<f64>::zeros(k + 2, k + 2);
    let mut t_rhs = vec![Vector3::zeros(); k + 2];
    let start = basis_functions(uk[0], &knots, count, p);
    let mut row = 0;
    md[(0, 0)] = start.n[0];
    t_rhs[0] = input.waypoints[0];
    if let Some(x_dot_0) = input.x_dot_0 {
        row += 1;
        md[(row, 0)] = start.n1[0];
        md[(row, 1)] = start.n1[1];
        t_rhs[row] = x_dot_0 * t_tot;
    }
    if let Some(x_ddot_0) = input.x_ddot_0 {
        row += 1;
        md[(row, 0)] = start.n2[0];
        md[(row, 1)] = start.n2[1];
        md[(row, 2)] = start.n2[2];
        t_rhs[row] = x_ddot_0 * t_tot.powi(2);
    }
    let end = basis_functions(uk[n_last], &knots, count, p);
    if let Some(x_ddot_n) = input.x_ddot_n {
        row += 1;
        md[(row, k - 1)] = end.n2[q - 2];
        md[(row, k)] = end.n2[q - 1];
        md[(row, k + 1)] = end.n2[q];
        t_rhs[row] = x_ddot_n * t_tot.powi(2);
    }
    if let Some(x_dot_n) = input.x_dot_n {
        row += 1;
        md[(row, k)] = end.n1[q - 1];
        md[(row, k + 1)] = end.n1[q];
        t_rhs[row] = x_dot_n * t_tot;
    }
    row += 1;
    md[(row, k + 1)] = end.n[q];
    t_rhs[row] = input.waypoints[n_last];
    let fixed = solve_coordinates(&md, &t_rhs);

    // Interior waypoints minus the fixed control points' contribution.
    let leading = 1 + usize::from(input.x_dot_0.is_some()) + usize::from(input.x_ddot_0.is_some());
    let free = q - k - 1;
    let mut residuals = vec![Vector3::zeros(); n_last - 1];
    let mut nd = DMatrix::<f64>::zeros(n_last - 1, free);
    for n in 1..n_last {
        let basis = basis_functions(uk[n], &knots, count, p);
        let nn = &basis.n;
        let mut residual = input.waypoints[n] - fixed[0] * nn[0] - fixed[k + 1] * nn[q];
        if input.x_dot_0.is_some() {
            residual -= fixed[1] * nn[1];
        }
        if input.x_ddot_0.is_some() {
            residual -= fixed[2] * nn[2];
        }
        if input.x_ddot_n.is_some() {
            residual -= fixed[k - 1] * nn[q - 2];
        }
        if input.x_dot_n.is_some() {
            residual -= fixed[k] * nn[q - 1];
        }
        residuals[n - 1] = residual;
        for b in 0..free {
            nd[(n - 1, b)] = nn[leading + b];
        }
    }

    // Weighted least squares for the free control points.
    let weights = DMatrix::from_fn(n_last - 1, n_last - 1, |row, col| {
        if row != col {
            0.0
        } else {
            input
                .weights
                .as_ref()
                .map_or(1.0, |weights| weights[row + 1])
        }
    });
    let nt_w = nd.transpose() * weights;
    let projected: Vec<Vector3<f64>> = (0..free)
        .map(|row| {
            residuals
                .iter()
                .enumerate()
                .fold(Vector3::zeros(), |sum, (col, value)| {
                    sum + value * nt_w[(row, col)]
                })
        })
        .collect();
    let least_squares = solve_coordinates(&(&nt_w * &nd), &projected);

    // Control points: the fixed leading ones, the fitted ones, the fixed
    // trailing ones.
    let mut control_points = Vec::with_capacity(count);
    control_points.extend_from_slice(&fixed[..leading]);
    control_points.extend_from_slice(&least_squares);
    control_points.extend_from_slice(&fixed[leading..]);

    let mut output = BSplineOutput {
        degree: p,
        knots,
        control_points,
        ..Default::default()
    };
    output.sample(num, t_tot);
    output
}

#[cfg(test)]
mod tests {
    //! Upstream tests BSpline through Python (`test_BSpline.py`, not part of
    //! this source tree's ports); these check the defining properties
    //! instead: the interpolant passes through the waypoints at their time
    //! tags and honours the end-derivative constraints, the basis is a
    //! partition of unity, and the approximation keeps its end points.

    use nalgebra::Vector3;

    use super::{BSplineInput, WaypointTiming, approximate, basis_functions, interpolate};

    fn waypoints() -> Vec<Vector3<f64>> {
        [
            [0.0, 5.0, 3.0],
            [1.0, 4.0, 2.0],
            [2.0, 3.0, 1.0],
            [3.0, 2.0, 2.0],
            [4.0, 1.0, 3.0],
            [5.0, 0.0, 4.0],
            [6.0, 1.0, 5.0],
        ]
        .map(Vector3::from)
        .to_vec()
    }

    fn input(x_dot: bool, x_ddot: bool) -> BSplineInput {
        let mut input = BSplineInput::new(waypoints());
        input.timing = WaypointTiming::TimeTags(vec![0.0, 2.0, 3.0, 5.0, 7.0, 8.0, 10.0]);
        if x_dot {
            input.x_dot_0 = Some(Vector3::new(1.0, 0.0, -0.5));
            input.x_dot_n = Some(Vector3::new(0.0, 0.5, 0.0));
        }
        if x_ddot {
            input.x_ddot_0 = Some(Vector3::new(0.0, 0.0, 0.0));
            input.x_ddot_n = Some(Vector3::new(0.2, 0.0, 0.0));
        }
        input
    }

    #[test]
    fn basis_is_a_partition_of_unity() {
        let output = interpolate(&input(true, true), 11, 5);
        for u in [0.0, 0.13, 0.5, 0.77, 1.0] {
            let basis = basis_functions(u, &output.knots, output.control_points.len(), 5);
            assert!((basis.n.sum() - 1.0).abs() < 1e-12, "u = {u}");
            assert!(basis.n1.sum().abs() < 1e-9, "u = {u}");
            assert!(basis.n2.sum().abs() < 1e-7, "u = {u}");
        }
    }

    #[test]
    fn interpolation_passes_through_waypoints_and_meets_end_derivatives() {
        for degree in [4, 5, 6] {
            for (x_dot, x_ddot) in [(false, false), (true, false), (false, true), (true, true)] {
                let input = input(x_dot, x_ddot);
                let output = interpolate(&input, 101, degree);
                let WaypointTiming::TimeTags(tags) = &input.timing else {
                    unreachable!()
                };
                for (waypoint, &tag) in input.waypoints.iter().zip(tags) {
                    let state = output.state_at(tag);
                    assert!(
                        (state.x - waypoint).norm() < 1e-9,
                        "P = {degree}, t = {tag}"
                    );
                    // Samples fall on every integer second.
                    let sample = output
                        .t
                        .iter()
                        .position(|t| (t - tag).abs() < 1e-9)
                        .unwrap();
                    assert!((output.x[sample] - waypoint).norm() < 1e-9);
                }
                let first = output.state_at(0.0);
                let last = output.state_at(10.0);
                if let Some(x_dot_0) = input.x_dot_0 {
                    assert!((first.x_dot - x_dot_0).norm() < 1e-9);
                    assert!((last.x_dot - input.x_dot_n.unwrap()).norm() < 1e-9);
                }
                if let Some(x_ddot_0) = input.x_ddot_0 {
                    assert!((first.x_ddot - x_ddot_0).norm() < 1e-9);
                    assert!((last.x_ddot - input.x_ddot_n.unwrap()).norm() < 1e-9);
                }
                // After the last time tag the curve holds its end point.
                let after = output.state_at(10.5);
                assert_eq!(after.x, output.x[100]);
                assert_eq!(after.x_dot, Vector3::zeros());
            }
        }
    }

    #[test]
    fn chord_length_and_average_speed_timing() {
        let mut input = BSplineInput::new(waypoints());
        let output = interpolate(&input, 50, 4);
        let length: f64 = waypoints().windows(2).map(|w| (w[1] - w[0]).norm()).sum();
        assert!((output.t[49] - length).abs() < 1e-12);

        input.timing = WaypointTiming::AverageSpeed(0.5);
        let output = interpolate(&input, 50, 4);
        assert!((output.t[49] - length / 0.5).abs() < 1e-12);
    }

    #[test]
    fn approximation_keeps_end_points_and_derivatives() {
        for (x_dot, x_ddot) in [(false, false), (true, false), (true, true)] {
            let input = input(x_dot, x_ddot);
            let output = approximate(&input, 101, 6, 4);
            let first = output.state_at(0.0);
            let last = output.state_at(10.0);
            assert!((first.x - input.waypoints[0]).norm() < 1e-9);
            assert!((last.x - input.waypoints[6]).norm() < 1e-9);
            if let Some(x_dot_0) = input.x_dot_0 {
                assert!((first.x_dot - x_dot_0).norm() < 1e-9);
                assert!((last.x_dot - input.x_dot_n.unwrap()).norm() < 1e-9);
            }
            if let Some(x_ddot_0) = input.x_ddot_0 {
                assert!((first.x_ddot - x_ddot_0).norm() < 1e-9);
                assert!((last.x_ddot - input.x_ddot_n.unwrap()).norm() < 1e-9);
            }
        }
    }

    #[test]
    fn approximation_with_as_many_control_points_as_data_interpolates() {
        // Q + 1 = N + 1 control points and no derivative constraint leave an
        // exactly determined fit of the interior waypoints, whatever the
        // weights.
        let mut input = input(false, false);
        input.weights = Some(vec![1.0, 2.0, 0.5, 1.0, 3.0, 1.0, 1.0]);
        let output = approximate(&input, 101, 6, 3);
        let WaypointTiming::TimeTags(tags) = &input.timing else {
            unreachable!()
        };
        for (waypoint, &tag) in input.waypoints.iter().zip(tags) {
            assert!(
                (output.state_at(tag).x - waypoint).norm() < 1e-9,
                "t = {tag}"
            );
        }
    }
}
