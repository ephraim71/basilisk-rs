//! Singular value decomposition and SVD least-squares solve, ported from
//! upstream Basilisk `architecture/utilities/svd.c` (Numerical Recipes in C
//! `svdcmp`, 0-based and allocation-free in upstream's revision).
//!
//! The decomposition is a literal port, not nalgebra's SVD: Householder
//! bidiagonalization followed by implicitly shifted QR, with upstream's
//! operation order, its `SIGN` macro (which treats `0` as negative, unlike
//! Numerical Recipes' `>= 0`) and its final sort by decreasing singular value.
//! It handles any shape, including more columns than rows. Upstream FSW uses it
//! through [`solve_svd`] in `mtbMomentumManagement`.

use nalgebra::{DMatrix, DVector};

use super::linear_algebra::sequential_product;

/// Upstream's iteration limit for the QR diagonalization of each singular value.
const MAX_QR_ITERATIONS: usize = 30;

/// Output of [`svdcmp`]: `mx = u * diag(w) * v^T`.
#[derive(Clone, Debug, PartialEq)]
pub struct SvdDecomposition {
    /// Left singular vectors, `dim1 x dim2` (upstream overwrites `mx` with
    /// `U`). When `dim1 < dim2` only the first `dim1` columns are orthonormal;
    /// the others pair with (numerically) zero singular values.
    pub u: DMatrix<f64>,
    /// Singular values (upstream `w`, length `dim2`), non-negative and sorted in
    /// decreasing order.
    pub w: DVector<f64>,
    /// Right singular vectors `V` (not `V^T`), `dim2 x dim2` (upstream `v`).
    pub v: DMatrix<f64>,
}

/// Port of upstream `svdcmp`: singular value decomposition of the
/// `dim1 x dim2` matrix `mx` (any shape).
///
/// Upstream returns `-1` only when a dimension exceeds `INT_MAX`, which cannot
/// be allocated here, so this is infallible. As upstream, a singular value that
/// does not converge within 30 QR iterations prints a warning
/// (`log::warn!`) and the last iterate is kept.
pub fn svdcmp(mx: &DMatrix<f64>) -> SvdDecomposition {
    let dim2 = mx.ncols();
    let mut u = mx.clone();
    let mut w = DVector::zeros(dim2);
    let mut v = DMatrix::zeros(dim2, dim2);
    let mut rv1 = DVector::zeros(dim2);

    let anorm = bidiagonalize(&mut u, &mut w, &mut rv1);
    accumulate_right_transformations(&u, &rv1, &mut v);
    accumulate_left_transformations(&mut u, &w);
    diagonalize(&mut u, &mut w, &mut v, &mut rv1, anorm);
    sort_by_decreasing_singular_value(&mut u, &mut w, &mut v);

    SvdDecomposition { u, w, v }
}

/// Port of upstream `solveSVD`: `x = V W^+ U^T b`, the SVD (minimum-norm least
/// squares) solution of `mx x = b` for a `dim1 x dim2` matrix.
///
/// The pseudo-inverse keeps singular values `w[j] >= min_sv`: the cutoff is
/// **absolute** (not relative to the largest singular value) and **inclusive**,
/// as upstream. `min_sv <= 0` keeps zero singular values and yields inf/NaN.
/// `mtbMomentumManagement` calls this with `min_sv = 1e-11`.
///
/// # Panics
///
/// If `b.len()` differs from `mx.nrows()`.
pub fn solve_svd(mx: &DMatrix<f64>, b: &DVector<f64>, min_sv: f64) -> DVector<f64> {
    let (dim1, dim2) = mx.shape();
    assert_eq!(
        b.len(),
        dim1,
        "solve_svd: b must have one entry per row of mx"
    );

    let SvdDecomposition { u, w, v } = svdcmp(mx);
    let w_inv_diag = DMatrix::from_fn(dim2, dim2, |row, col| {
        if row == col && w[col] >= min_sv {
            1.0 / w[col]
        } else {
            0.0
        }
    });

    // Upstream forms the full dim2 x dim1 pseudo-inverse with mMultM, then mMultV.
    let pseudo_inverse = sequential_product(&sequential_product(&v, &w_inv_diag), &u.transpose());
    DVector::from_fn(dim2, |row, _| {
        (0..dim1).fold(0.0, |acc, col| acc + pseudo_inverse[(row, col)] * b[col])
    })
}

/// Upstream `SIGN(a, b)`: `|a|` if `b > 0`, else `-|a|` (zero counts as negative).
fn sign(a: f64, b: f64) -> f64 {
    if b > 0.0 { a.abs() } else { -a.abs() }
}

/// Upstream `pythag`: `sqrt(a^2 + b^2)` without destructive overflow or underflow.
fn pythag(a: f64, b: f64) -> f64 {
    let abs_a = a.abs();
    let abs_b = b.abs();
    if abs_a > abs_b {
        let ratio = abs_b / abs_a;
        abs_a * (1.0 + ratio * ratio).sqrt()
    } else if abs_b == 0.0 {
        0.0
    } else {
        let ratio = abs_a / abs_b;
        abs_b * (1.0 + ratio * ratio).sqrt()
    }
}

/// Householder reduction to bidiagonal form: the diagonal goes to `w`, the
/// superdiagonal to `rv1[1..]`, and the Householder vectors stay in `a`.
/// Returns upstream's `anorm`, the largest `|w[i]| + |rv1[i]|`.
fn bidiagonalize(a: &mut DMatrix<f64>, w: &mut DVector<f64>, rv1: &mut DVector<f64>) -> f64 {
    let (dim1, dim2) = a.shape();
    let mut g = 0.0;
    let mut scale = 0.0;
    let mut anorm: f64 = 0.0;
    for i in 0..dim2 {
        let l = i + 1;
        rv1[i] = scale * g;

        // Left transformation, zeroing column i below the diagonal.
        g = 0.0;
        scale = 0.0;
        if i < dim1 {
            scale = (i..dim1).fold(0.0, |acc, k| acc + a[(k, i)].abs());
            if scale != 0.0 {
                let mut s = 0.0;
                for k in i..dim1 {
                    a[(k, i)] /= scale;
                    s += a[(k, i)] * a[(k, i)];
                }
                let f = a[(i, i)];
                g = -sign(s.sqrt(), f);
                let h = f * g - s;
                a[(i, i)] = f - g;
                for j in l..dim2 {
                    let s = (i..dim1).fold(0.0, |acc, k| acc + a[(k, i)] * a[(k, j)]);
                    let f = s / h;
                    for k in i..dim1 {
                        a[(k, j)] += f * a[(k, i)];
                    }
                }
                for k in i..dim1 {
                    a[(k, i)] *= scale;
                }
            }
        }
        w[i] = scale * g;

        // Right transformation, zeroing row i right of the superdiagonal.
        g = 0.0;
        scale = 0.0;
        if i < dim1 && l != dim2 {
            scale = (l..dim2).fold(0.0, |acc, k| acc + a[(i, k)].abs());
            if scale != 0.0 {
                let mut s = 0.0;
                for k in l..dim2 {
                    a[(i, k)] /= scale;
                    s += a[(i, k)] * a[(i, k)];
                }
                let f = a[(i, l)];
                g = -sign(s.sqrt(), f);
                let h = f * g - s;
                a[(i, l)] = f - g;
                for k in l..dim2 {
                    rv1[k] = a[(i, k)] / h;
                }
                for j in l..dim1 {
                    let s = (l..dim2).fold(0.0, |acc, k| acc + a[(j, k)] * a[(i, k)]);
                    for k in l..dim2 {
                        a[(j, k)] += s * rv1[k];
                    }
                }
                for k in l..dim2 {
                    a[(i, k)] *= scale;
                }
            }
        }
        // Upstream `DMAX(anorm, ...)`, kept as a plain comparison.
        let candidate = w[i].abs() + rv1[i].abs();
        anorm = if anorm > candidate { anorm } else { candidate };
    }
    anorm
}

/// Accumulates the right-hand Householder transformations into `v`.
fn accumulate_right_transformations(a: &DMatrix<f64>, rv1: &DVector<f64>, v: &mut DMatrix<f64>) {
    let dim2 = a.ncols();
    for i in (0..dim2).rev() {
        let l = i + 1;
        if l < dim2 {
            let g = rv1[l];
            if g != 0.0 {
                // Double division avoids a possible underflow (Numerical Recipes).
                for j in l..dim2 {
                    v[(j, i)] = (a[(i, j)] / a[(i, l)]) / g;
                }
                for j in l..dim2 {
                    let s = (l..dim2).fold(0.0, |acc, k| acc + a[(i, k)] * v[(k, j)]);
                    for k in l..dim2 {
                        v[(k, j)] += s * v[(k, i)];
                    }
                }
            }
            for j in l..dim2 {
                v[(i, j)] = 0.0;
                v[(j, i)] = 0.0;
            }
        }
        v[(i, i)] = 1.0;
    }
}

/// Accumulates the left-hand Householder transformations, turning `a` into `U`.
fn accumulate_left_transformations(a: &mut DMatrix<f64>, w: &DVector<f64>) {
    let (dim1, dim2) = a.shape();
    for i in (0..dim1.min(dim2)).rev() {
        let l = i + 1;
        for j in l..dim2 {
            a[(i, j)] = 0.0;
        }
        if w[i] != 0.0 {
            let g = 1.0 / w[i];
            for j in l..dim2 {
                let s = (l..dim1).fold(0.0, |acc, k| acc + a[(k, i)] * a[(k, j)]);
                let f = (s / a[(i, i)]) * g;
                for k in i..dim1 {
                    a[(k, j)] += f * a[(k, i)];
                }
            }
            for j in i..dim1 {
                a[(j, i)] *= g;
            }
        } else {
            for j in i..dim1 {
                a[(j, i)] = 0.0;
            }
        }
        a[(i, i)] += 1.0;
    }
}

/// Diagonalizes the bidiagonal form by implicitly shifted QR sweeps, applying
/// the rotations to `u` and `v` and making every singular value non-negative.
fn diagonalize(
    u: &mut DMatrix<f64>,
    w: &mut DVector<f64>,
    v: &mut DMatrix<f64>,
    rv1: &mut DVector<f64>,
    anorm: f64,
) {
    let dim2 = w.len();
    for k in (0..dim2).rev() {
        for iteration in 0..MAX_QR_ITERATIONS {
            // Test for splitting. rv1[0] is always zero, so upstream always stops
            // at l = 0 there; that is made explicit to avoid reading w[-1].
            let mut l = k;
            let cancel = loop {
                if l == 0 || rv1[l].abs() + anorm == anorm {
                    break false;
                }
                if w[l - 1].abs() + anorm == anorm {
                    break true;
                }
                l -= 1;
            };
            if cancel {
                // Cancellation of rv1[l] (w[l - 1] is negligible).
                let nm = l - 1;
                let mut c = 0.0;
                let mut s = 1.0;
                for i in l..=k {
                    let f = s * rv1[i];
                    rv1[i] *= c;
                    if f.abs() + anorm == anorm {
                        break;
                    }
                    let g = w[i];
                    let h = pythag(f, g);
                    w[i] = h;
                    let h = 1.0 / h;
                    c = g * h;
                    s = -f * h;
                    rotate_columns(u, nm, i, c, s);
                }
            }

            let z = w[k];
            if l == k {
                // Converged: make the singular value non-negative.
                if z < 0.0 {
                    w[k] = -z;
                    v.column_mut(k).neg_mut();
                }
                break;
            }
            if iteration == MAX_QR_ITERATIONS - 1 {
                log::warn!("no convergence in 30 svdcmp iterations");
            }

            // Shift from the bottom 2x2 minor.
            let mut x = w[l];
            let nm = k - 1;
            let mut y = w[nm];
            let mut g = rv1[nm];
            let mut h = rv1[k];
            let mut f = ((y - z) * (y + z) + (g - h) * (g + h)) / (2.0 * h * y);
            g = pythag(f, 1.0);
            f = ((x - z) * (x + z) + h * ((y / (f + sign(g, f))) - h)) / x;

            // Next QR transformation. When a rotation norm vanishes upstream keeps
            // the previous c and s, hence the carried state.
            let mut c = 1.0;
            let mut s = 1.0;
            for j in l..=nm {
                let i = j + 1;
                g = rv1[i];
                y = w[i];
                h = s * g;
                g *= c;
                let z = pythag(f, h);
                rv1[j] = z;
                c = f / z;
                s = h / z;
                f = x * c + g * s;
                g = g * c - x * s;
                h = y * s;
                y *= c;
                rotate_columns(v, j, i, c, s);
                let z = pythag(f, h);
                w[j] = z;
                if z != 0.0 {
                    let z = 1.0 / z;
                    c = f * z;
                    s = h * z;
                }
                f = c * g + s * y;
                x = c * y - s * g;
                rotate_columns(u, j, i, c, s);
            }
            rv1[l] = 0.0;
            rv1[k] = f;
            w[k] = x;
        }
    }
}

/// Upstream's final selection sort: singular values in decreasing order, with
/// the matching columns of `u` and `v` swapped along.
fn sort_by_decreasing_singular_value(
    u: &mut DMatrix<f64>,
    w: &mut DVector<f64>,
    v: &mut DMatrix<f64>,
) {
    let dim2 = w.len();
    for i in 0..dim2.saturating_sub(1) {
        let largest = ((i + 1)..dim2).fold(i, |best, j| if w[j] > w[best] { j } else { best });
        if largest != i {
            u.swap_columns(i, largest);
            v.swap_columns(i, largest);
            w.swap_rows(i, largest);
        }
    }
}

/// Upstream's plane rotation of columns `a` and `b`:
/// `(y, z) -> (y c + z s, z c - y s)` on every row.
fn rotate_columns(m: &mut DMatrix<f64>, a: usize, b: usize, c: f64, s: f64) {
    for row in 0..m.nrows() {
        let y = m[(row, a)];
        let z = m[(row, b)];
        m[(row, a)] = y * c + z * s;
        m[(row, b)] = z * c - y * s;
    }
}

#[cfg(test)]
mod tests {
    //! Upstream has no dedicated `svd.c` unit test; it is exercised only through
    //! `mtbMomentumManagement`. These tests check the decomposition's defining
    //! properties, compare against nalgebra's SVD least squares, and pin the
    //! upstream cutoff semantics.

    use nalgebra::{DMatrix, DVector};

    use super::*;

    fn assert_valid_decomposition(mx: &DMatrix<f64>, tolerance: f64) -> SvdDecomposition {
        let svd = svdcmp(mx);
        let (dim1, dim2) = mx.shape();
        assert_eq!(svd.u.shape(), (dim1, dim2));
        assert_eq!(svd.w.len(), dim2);
        assert_eq!(svd.v.shape(), (dim2, dim2));

        let reconstructed = &svd.u * DMatrix::from_diagonal(&svd.w) * svd.v.transpose();
        assert!(
            (&reconstructed - mx).norm() <= tolerance * mx.norm().max(1.0),
            "U W V^T must reproduce the matrix"
        );
        assert!((svd.v.transpose() * &svd.v - DMatrix::identity(dim2, dim2)).norm() <= tolerance);
        let rank_columns = dim1.min(dim2);
        let u_used = svd.u.columns(0, rank_columns);
        assert!(
            (u_used.transpose() * u_used - DMatrix::identity(rank_columns, rank_columns)).norm()
                <= tolerance
        );
        for pair in svd.w.as_slice().windows(2) {
            assert!(pair[0] >= pair[1], "singular values must decrease");
        }
        assert!(svd.w.iter().all(|&value| value >= 0.0));

        let mut truth = mx
            .clone()
            .svd(false, false)
            .singular_values
            .as_slice()
            .to_vec();
        truth.sort_by(|a, b| b.total_cmp(a));
        truth.resize(dim2, 0.0);
        for (actual, expected) in svd.w.iter().zip(truth) {
            assert!((actual - expected).abs() <= tolerance * mx.norm().max(1.0));
        }
        svd
    }

    fn sample_matrix(rows: usize, cols: usize, seed: f64) -> DMatrix<f64> {
        DMatrix::from_fn(rows, cols, |i, j| {
            ((i * 5 + j * 3) as f64 * seed + 0.3).sin() * 2.0
        })
    }

    /// Checks the four Moore-Penrose conditions for `p` as pseudo-inverse of `m`.
    fn assert_moore_penrose(m: &DMatrix<f64>, p: &DMatrix<f64>, tolerance: f64) {
        assert!((m * p * m - m).norm() <= tolerance, "M P M = M");
        assert!(
            (p * m * p - p).norm() <= tolerance * p.norm().max(1.0),
            "P M P = P"
        );
        let mp = m * p;
        assert!((&mp - mp.transpose()).norm() <= tolerance, "(M P)^T = M P");
        let pm = p * m;
        assert!((&pm - pm.transpose()).norm() <= tolerance, "(P M)^T = P M");
    }

    fn pseudo_inverse_by_columns(m: &DMatrix<f64>, min_sv: f64) -> DMatrix<f64> {
        let (rows, cols) = m.shape();
        let mut p = DMatrix::zeros(cols, rows);
        for j in 0..rows {
            let mut unit = DVector::zeros(rows);
            unit[j] = 1.0;
            p.set_column(j, &solve_svd(m, &unit, min_sv));
        }
        p
    }

    #[test]
    fn decomposes_tall_square_and_wide_matrices() {
        for (rows, cols) in [
            (5, 3),
            (3, 3),
            (6, 6),
            (3, 4),
            (2, 5),
            (1, 1),
            (4, 1),
            (1, 3),
        ] {
            assert_valid_decomposition(&sample_matrix(rows, cols, 0.7), 1.0e-13);
        }
    }

    #[test]
    fn decomposes_diagonal_matrices_exactly_and_sorts_them() {
        let diagonal = DMatrix::from_diagonal(&DVector::from_vec(vec![-3.0, 0.5, 7.0]));
        let svd = assert_valid_decomposition(&diagonal, 1.0e-15);
        assert_eq!(svd.w.as_slice(), &[7.0, 3.0, 0.5]);
        let identity = svdcmp(&DMatrix::identity(3, 3));
        assert_eq!(identity.w.as_slice(), &[1.0, 1.0, 1.0]);
    }

    #[test]
    fn solve_matches_nalgebra_svd_least_squares_for_full_rank_systems() {
        let cases = [
            (
                sample_matrix(3, 3, 0.9),
                DVector::from_vec(vec![1.0, -2.0, 0.5]),
            ),
            (
                sample_matrix(5, 3, 0.4),
                DVector::from_vec(vec![1.0, 2.0, 3.0, -1.0, 0.25]),
            ),
            (
                sample_matrix(3, 4, 1.3),
                DVector::from_vec(vec![0.1, -0.7, 2.0]),
            ),
        ];
        for (mx, b) in cases {
            let x = solve_svd(&mx, &b, 1.0e-11);
            let truth = mx
                .clone()
                .svd(true, true)
                .solve(&b, 1.0e-11)
                .expect("U and V computed");
            assert!((&x - &truth).norm() <= 1.0e-12 * truth.norm().max(1.0));
        }
    }

    #[test]
    fn solve_drops_singular_values_of_a_rank_deficient_matrix() {
        // Third column is the sum of the first two: rank 2.
        let mx = DMatrix::from_row_slice(
            4,
            3,
            &[1.0, 2.0, 3.0, -1.0, 0.5, -0.5, 2.0, 1.0, 3.0, 0.0, 4.0, 4.0],
        );
        let svd = assert_valid_decomposition(&mx, 1.0e-13);
        assert!(
            svd.w[2] < 1.0e-11,
            "third singular value must be numerically zero"
        );

        let b = DVector::from_vec(vec![1.0, 2.0, 3.0, 4.0]);
        let x = solve_svd(&mx, &b, 1.0e-11);
        let truth = mx
            .clone()
            .svd(true, true)
            .solve(&b, 1.0e-11)
            .expect("U and V computed");
        assert!((&x - truth).norm() <= 1.0e-12);
        // Minimum-norm solution: orthogonal to the null space (1, 1, -1).
        assert!((x[0] + x[1] - x[2]).abs() <= 1.0e-12);

        let p = pseudo_inverse_by_columns(&mx, 1.0e-11);
        assert_moore_penrose(&mx, &p, 1.0e-12);
    }

    #[test]
    fn singular_value_cutoff_is_absolute_and_inclusive() {
        let cutoff = 1.0e-11;
        let mx = DMatrix::from_diagonal(&DVector::from_vec(vec![cutoff, 0.5 * cutoff, 2.0]));
        let b = DVector::from_vec(vec![1.0e-11, 1.0, 4.0]);
        let x = solve_svd(&mx, &b, cutoff);
        assert!((x[0] - 1.0).abs() <= 1.0e-15, "w == min_sv is kept");
        assert_eq!(x[1], 0.0, "w < min_sv is dropped, however large b is");
        assert!((x[2] - 2.0).abs() <= 1.0e-15);

        // A zero matrix has no usable singular value: the solution is zero.
        let zero = solve_svd(
            &DMatrix::zeros(3, 2),
            &DVector::from_element(3, 1.0),
            cutoff,
        );
        assert_eq!(zero, DVector::zeros(2));
    }

    #[test]
    #[allow(clippy::excessive_precision)] // verbatim captured values
    fn pseudo_inverse_is_accurate_for_the_mtb_rank2_field_geometry() {
        // `tilde(B) * Gt` captured from the mtbMomentumManagement parity scenario
        // (rank 2, 3x4), where nalgebra's own SVD loses accuracy (~1e-7).
        let b_gt = DMatrix::from_row_slice(
            3,
            4,
            &[
                0.0,
                1.06879128500276908e-05,
                -2.72834298179075497e-05,
                7.55749564030370400e-06,
                -1.06879128500276908e-05,
                0.0,
                -3.15802641687453088e-05,
                -7.55749564030370400e-06,
                2.72834298179075497e-05,
                3.15802641687453088e-05,
                0.0,
                4.16229171138074661e-05,
            ],
        );
        let svd = svdcmp(&b_gt);
        let reconstructed = &svd.u * DMatrix::from_diagonal(&svd.w) * svd.v.transpose();
        assert!((reconstructed - &b_gt).norm() <= 1.0e-14 * b_gt.norm());
        assert!(svd.w[2] < 1.0e-11 && svd.w[3] < 1.0e-11, "rank 2");

        let p = pseudo_inverse_by_columns(&b_gt, 1.0e-11);
        let residual = (&b_gt * &p * &b_gt - &b_gt).norm();
        assert!(
            residual <= 1.0e-15,
            "Moore-Penrose residual too large: {residual:e}"
        );
        assert_moore_penrose(&b_gt, &p, 1.0e-9);
    }
}
