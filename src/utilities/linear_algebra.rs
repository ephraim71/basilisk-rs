//! Linear-algebra helpers that nalgebra lacks, ported from upstream Basilisk
//! `architecture/utilities/linearAlgebra.c`, and a map from the upstream calls
//! FSW modules use to their nalgebra equivalents.
//!
//! Upstream matrices are row-major flat `double` arrays
//! (`MXINDEX(dim2, row, col) = row * dim2 + col`): build nalgebra matrices from
//! them with `Matrix3::from_row_slice` / `DMatrix::from_row_slice`
//! (`from_column_slice` and `from_vec` are column-major).
//!
//! # Upstream → nalgebra
//!
//! | upstream | Rust |
//! |---|---|
//! | `v3Add`, `v3Subtract`, `vAdd`, `mAdd`, `m33Subtract` ... | `a + b`, `a - b` |
//! | `v3Scale(s, v, r)`, `vScale`, `m33Scale`, `mScale` | `s * v` |
//! | `v3Dot(a, b)`, `vDot` | `a.dot(&b)` |
//! | `v3Cross(a, b, r)` | `a.cross(&b)` |
//! | `v3Norm(v)`, `vNorm` | `v.norm()` |
//! | `v3Normalize(v, r)`, `vNormalize` | [`normalize_or_zero`] (see gotchas) |
//! | `v3OuterProduct(a, b, r)` | `a * b.transpose()` |
//! | `v3Tilde(v, r)` | `v.cross_matrix()` (or `rigid_body_kinematics::tilde`) |
//! | `vElementwiseMult(a, n, b, r)` | `a.component_mul(&b)` |
//! | `vMaxAbs(v, n)`, `vMax(v, n)` | `v.amax()`, `v.max()` |
//! | `v3SetZero`, `vSetZero`, `mSetZero`, `m33SetZero` | `Vector3::zeros()`, `DVector::zeros(n)`, `DMatrix::zeros(r, c)`, `Matrix3::zeros()` |
//! | `v3Copy`, `vCopy`, `mCopy`, `m33Copy` | assignment (`Vector3`/`Matrix3` are `Copy`) or `.clone()` |
//! | `v3Set(x, y, z, r)`, `m33Set(...)` | `Vector3::new(x, y, z)`, `Matrix3::new(...)` (row-major arguments) |
//! | `m33MultV3(m, v, r)`, `mMultV` | `m * v` |
//! | `m33tMultV3(m, v, r)` | `m.tr_mul(&v)` (`= m.transpose() * v`) |
//! | `m33MultM33(a, b, r)`, `mMultM` | `a * b` |
//! | `m33MultM33t(a, b, r)`, `mMultMt` | `a * b.transpose()` |
//! | `m33tMultM33(a, b, r)`, `mtMultM` | `a.tr_mul(&b)` |
//! | `m33Transpose(m, r)`, `mTranspose` | `m.transpose()` |
//! | `m33Determinant(m)`, `mDeterminant` | `m.determinant()` (different expansion order: last-bit differences) |
//! | `m33Inverse(m, r)`, `mInverse` | [`m33_inverse`], [`m_inverse`] (upstream numerics and singular handling); `m.try_inverse()` otherwise |
//! | `m33SetIdentity`, `mSetIdentity(m, r, c)` | `Matrix3::identity()`, `DMatrix::identity(r, c)` |
//! | `mDiag(v, n, r)` | `DMatrix::from_diagonal(&v)`, `Matrix3::from_diagonal(&v)` |
//! | `mSetSubMatrix(sub, r, c, m, R, C, row0, col0)` | `m.view_mut((row0, col0), (r, c)).copy_from(&sub)` (or `m.fixed_view_mut::<3, 3>(row0, col0)`) |
//! | `mGetSubMatrix(m, R, C, row0, col0, r, c, sub)` | `m.view((row0, col0), (r, c))` (or `m.fixed_view::<3, 3>(row0, col0)`) |
//! | `v3IsEqual`, `vIsEqual`, `m33IsEqual`, `mIsEqual` | [`is_equal_within`] |
//! | `v3IsZero`, `vIsZero`, `m33IsZero`, `mIsZero` | [`is_zero_within`] |
//! | `safeAcos`, `safeAsin` | [`safe_acos`], [`safe_asin`] |
//! | `m33EigenValues` | [`m33_eigen_values`] |
//! | `mMinimumNormInverse` | [`m_minimum_norm_inverse`] |
//! | `v3Perpendicular` | [`v3_perpendicular`] |
//!
//! # Gotchas
//!
//! * `v3Normalize`/`vNormalize` return the zero vector when the norm is
//!   `<= DB0_EPS` (1e-30) and otherwise scale by `1 / norm`. nalgebra's
//!   `normalize()` returns NaN for a zero vector and divides by the norm
//!   (results differ by up to 1 ulp); [`normalize_or_zero`] reproduces
//!   upstream exactly.
//! * `m33Inverse`/`mInverse` of a matrix with `|det| <= DB0_EPS` logs an error,
//!   writes the zero matrix and returns status 1. nalgebra's `try_inverse`
//!   returns `None` only for an exactly zero determinant (3×3) or a failed LU
//!   factorization, with different rounding. [`m33_inverse`]/[`m_inverse`]
//!   return `None` in upstream's singular case; `.unwrap_or_else(Matrix3::zeros)`
//!   reproduces upstream's output.
//! * Upstream `mInverse` of a 1×1 matrix returns zero (see [`m_inverse`]).
//! * `mMultM` with mismatched dimensions logs and leaves the output untouched;
//!   nalgebra panics.
//! * nalgebra multiplies large dynamic matrices (all dimensions > 5) with the
//!   `matrixmultiply` kernel, whose summation order (and FMA use) differs from
//!   upstream's sequential loops; expect last-bit differences there, or use
//!   [`sequential_product`], which keeps upstream's order.
//! * [`m33_eigen_values`] uses Rust's `cbrt`, which is correctly rounded where
//!   glibc's is not, so its Cardano branch (non-symmetric matrices, repeated
//!   roots) can differ from an upstream build in the last bits (≈1e-14
//!   relative). Everything else in this module and in `rigid_body_kinematics`
//!   matched a glibc build of upstream bit for bit in randomized comparisons.
//!
//! Not ported because no FSW module uses them: `cubicRoots` and `v3Sort`
//! (private here), `m33SingularValues`, `m33ConditionNumber`, `v3IsEqualRel`,
//! `safeSqrt`, `mLeastSquaresInverse`.

use nalgebra::allocator::Allocator;
use nalgebra::{
    DMatrix, DefaultAllocator, Dim, Matrix, Matrix3, OMatrix, OVector, RawStorage, Vector3,
};

/// Upstream `DB0_EPS`: magnitude at or below which upstream treats a norm or a
/// determinant as zero.
pub const DB0_EPS: f64 = 1e-30;

/// `acos` with its argument clamped to `[-1, 1]` (upstream `safeAcos`), so
/// round-off just outside the domain yields 0 or π instead of NaN. NaN stays NaN.
pub fn safe_acos(x: f64) -> f64 {
    x.clamp(-1.0, 1.0).acos()
}

/// `asin` with its argument clamped to `[-1, 1]` (upstream `safeAsin`), so
/// round-off just outside the domain yields ±π/2 instead of NaN. NaN stays NaN.
pub fn safe_asin(x: f64) -> f64 {
    x.clamp(-1.0, 1.0).asin()
}

/// Unit vector along `v`, or the zero vector when `|v| <= DB0_EPS` or the norm
/// is NaN (upstream `v3Normalize` / `vNormalize`, including their
/// multiply-by-`1/norm` rounding).
pub fn normalize_or_zero<D: Dim>(mut v: OVector<f64, D>) -> OVector<f64, D>
where
    DefaultAllocator: Allocator<D>,
{
    // Sequential sum of squares, as upstream `vDot`; nalgebra's `norm()` splits
    // the sum into eight accumulators for vectors of eight or more elements.
    let norm = v.iter().fold(0.0, |sum, x| sum + x * x).sqrt();
    if norm > DB0_EPS {
        v *= 1.0 / norm;
    } else {
        v.fill(0.0);
    }
    v
}

/// `true` when `a` and `b` have the same shape and no element pair differs by
/// more than `accuracy` (upstream `v3IsEqual`, `vIsEqual`, `m33IsEqual`,
/// `mIsEqual`). As upstream, a NaN difference does not count as a mismatch.
pub fn is_equal_within<R, C, S1, S2>(
    a: &Matrix<f64, R, C, S1>,
    b: &Matrix<f64, R, C, S2>,
    accuracy: f64,
) -> bool
where
    R: Dim,
    C: Dim,
    S1: RawStorage<f64, R, C>,
    S2: RawStorage<f64, R, C>,
{
    a.shape() == b.shape()
        && !a
            .iter()
            .zip(b.iter())
            .any(|(x, y)| (x - y).abs() > accuracy)
}

/// `true` when no element of `m` exceeds `accuracy` in magnitude (upstream
/// `v3IsZero`, `vIsZero`, `m33IsZero`, `mIsZero`). As upstream, NaN elements do
/// not count as nonzero.
pub fn is_zero_within<R, C, S>(m: &Matrix<f64, R, C, S>, accuracy: f64) -> bool
where
    R: Dim,
    C: Dim,
    S: RawStorage<f64, R, C>,
{
    !m.iter().any(|x| x.abs() > accuracy)
}

/// A unit vector perpendicular to `v` (upstream `v3Perpendicular`).
///
/// Upstream solves `v · p = 0` for the component of `p` along the first
/// nonzero (`> DB0_EPS` in magnitude) element of `v`, sets the other two to 1,
/// and normalizes. A zero `v` produces a NaN candidate, which normalizes to
/// the zero vector.
pub fn v3_perpendicular(v: Vector3<f64>) -> Vector3<f64> {
    let candidate = if v[0].abs() > DB0_EPS {
        Vector3::new(-(v[1] + v[2]) / v[0], 1.0, 1.0)
    } else if v[1].abs() > DB0_EPS {
        Vector3::new(1.0, -(v[0] + v[2]) / v[1], 1.0)
    } else {
        Vector3::new(1.0, 1.0, -(v[0] + v[1]) / v[2])
    };
    normalize_or_zero(candidate)
}

/// Inverse of a 3×3 matrix by the adjugate formula (upstream `m33Inverse`).
///
/// Returns `None`, after logging upstream's error, when `|det| <= DB0_EPS` or
/// the determinant is NaN;
/// upstream then writes the zero matrix and returns status 1, so
/// `m33_inverse(&m).unwrap_or_else(Matrix3::zeros)` reproduces its output.
pub fn m33_inverse(m: &Matrix3<f64>) -> Option<Matrix3<f64>> {
    let det = m33_determinant(m);
    // Upstream inverts only when `fabs(det) > DB0_EPS`, so a NaN determinant
    // takes the singular branch too.
    if det.is_nan() || det.abs() <= DB0_EPS {
        log::error!("Error: singular 3x3 matrix inverse");
        return None;
    }
    let det_inv = 1.0 / det;
    Some(Matrix3::new(
        (m[(1, 1)] * m[(2, 2)] - m[(1, 2)] * m[(2, 1)]) * det_inv,
        -(m[(0, 1)] * m[(2, 2)] - m[(0, 2)] * m[(2, 1)]) * det_inv,
        (m[(0, 1)] * m[(1, 2)] - m[(0, 2)] * m[(1, 1)]) * det_inv,
        -(m[(1, 0)] * m[(2, 2)] - m[(1, 2)] * m[(2, 0)]) * det_inv,
        (m[(0, 0)] * m[(2, 2)] - m[(0, 2)] * m[(2, 0)]) * det_inv,
        -(m[(0, 0)] * m[(1, 2)] - m[(0, 2)] * m[(1, 0)]) * det_inv,
        (m[(1, 0)] * m[(2, 1)] - m[(1, 1)] * m[(2, 0)]) * det_inv,
        -(m[(0, 0)] * m[(2, 1)] - m[(0, 1)] * m[(2, 0)]) * det_inv,
        (m[(0, 0)] * m[(1, 1)] - m[(0, 1)] * m[(1, 0)]) * det_inv,
    ))
}

/// Inverse of a square matrix as adjugate over determinant, both by cofactor
/// (Laplace) expansion (upstream `mInverse`).
///
/// Returns `None`, after logging upstream's error, when `|det| <= DB0_EPS` or
/// the determinant is NaN;
/// upstream then writes the zero matrix and returns status 1.
///
/// Upstream bug, preserved: the cofactor of a 1×1 matrix is the determinant of
/// an empty matrix, which upstream `mDeterminant` defines as 0, so a
/// nonsingular 1×1 matrix inverts to `[0]`.
///
/// # Panics
/// If `m` is not square.
pub fn m_inverse<R, C, S>(m: &Matrix<f64, R, C, S>) -> Option<DMatrix<f64>>
where
    R: Dim,
    C: Dim,
    S: RawStorage<f64, R, C>,
{
    assert!(m.is_square(), "m_inverse requires a square matrix");
    let m = to_dmatrix(m);
    let det = laplace_determinant(&m);
    // Upstream inverts only when `fabs(det) > DB0_EPS`, so a NaN determinant
    // takes the singular branch too.
    if det.is_nan() || det.abs() <= DB0_EPS {
        log::error!("Error: cannot invert singular matrix");
        return None;
    }
    let dim = m.nrows();
    let cofactors = DMatrix::from_fn(dim, dim, |i, j| {
        let sign = if (i + j) % 2 == 0 { 1.0 } else { -1.0 };
        sign * laplace_determinant(&minor(&m, i, j))
    });
    Some(cofactors.transpose() * (1.0 / det))
}

/// Minimum-norm right inverse `Mᵀ (M Mᵀ)⁻¹` of a `dim1 × dim2` matrix with
/// `dim1 <= dim2` (upstream `mMinimumNormInverse`), returned as `dim2 × dim1`.
///
/// The products use upstream's sequential loops and the inverse is
/// [`m_inverse`]; like upstream, a singular `M Mᵀ` (error logged) yields the
/// zero matrix.
pub fn m_minimum_norm_inverse<R, C, S>(m: &Matrix<f64, R, C, S>) -> DMatrix<f64>
where
    R: Dim,
    C: Dim,
    S: RawStorage<f64, R, C>,
{
    let m = to_dmatrix(m);
    let m_transpose = m.transpose();
    let gram = sequential_product(&m, &m_transpose);
    let gram_inverse =
        m_inverse(&gram).unwrap_or_else(|| DMatrix::zeros(gram.nrows(), gram.ncols()));
    sequential_product(&m_transpose, &gram_inverse)
}

/// Eigenvalues of a 3×3 matrix from the closed-form roots of its
/// characteristic polynomial, sorted in descending order (upstream
/// `m33EigenValues`, `cubicRoots` and `v3Sort`).
///
/// With three distinct real roots (always the case for a symmetric matrix
/// with distinct eigenvalues) upstream uses the trigonometric form. Otherwise
/// it uses Cardano's form, which for a non-symmetric matrix with a complex
/// conjugate pair `α ± iβ` returns the real root and `α` twice — the imaginary
/// parts are discarded. Repeated roots also take this branch.
pub fn m33_eigen_values(m: &Matrix3<f64>) -> Vector3<f64> {
    let coefficients = [
        -m33_determinant(m),
        m[(0, 0)] * m[(1, 1)] - m[(0, 1)] * m[(1, 0)] + m[(0, 0)] * m[(2, 2)]
            - m[(0, 2)] * m[(2, 0)]
            + m[(1, 1)] * m[(2, 2)]
            - m[(1, 2)] * m[(2, 1)],
        -m[(0, 0)] - m[(1, 1)] - m[(2, 2)],
    ];
    sort_descending(cubic_roots(coefficients))
}

/// Upstream `m33Determinant`: the six-term rule of Sarrus, in upstream's order
/// (so thresholds on it see upstream's rounding).
pub fn m33_determinant(m: &Matrix3<f64>) -> f64 {
    m[(0, 0)] * m[(1, 1)] * m[(2, 2)]
        + m[(0, 1)] * m[(1, 2)] * m[(2, 0)]
        + m[(0, 2)] * m[(1, 0)] * m[(2, 1)]
        - m[(0, 0)] * m[(1, 2)] * m[(2, 1)]
        - m[(0, 1)] * m[(1, 0)] * m[(2, 2)]
        - m[(0, 2)] * m[(1, 1)] * m[(2, 0)]
}

/// Upstream `cubicRoots`: roots of `x³ + a[2] x² + a[1] x + a[0] = 0`
/// (<http://mathworld.wolfram.com/CubicFormula.html>).
fn cubic_roots(a: [f64; 3]) -> [f64; 3] {
    let a2sq = a[2] * a[2];
    let a2d3 = a[2] / 3.0;
    let q = (3.0 * a[1] - a2sq) / 9.0;
    let r = (a[2] * (9.0 * a[1] - 2.0 * a2sq) - 27.0 * a[0]) / 54.0;
    let r_over_sqrt_neg_q3 = r / (-q * q * q).sqrt();

    if q < 0.0 && r_over_sqrt_neg_q3.abs() < 1.0 {
        // Three real roots: A cos(t/3 + 2πk/3) - a[2]/3 with A = 2 √(-Q).
        let amplitude = 2.0 * (-q).sqrt();
        let td3 = safe_acos(r_over_sqrt_neg_q3) / 3.0;
        let costd3 = td3.cos();
        let sintd3 = td3.sin();
        let sqrt3d2 = 3.0_f64.sqrt() * 0.5;
        let temp1 = -0.5 * costd3;
        let temp2 = sqrt3d2 * sintd3;
        [
            amplitude * costd3 - a2d3,
            amplitude * (temp1 - temp2) - a2d3,
            amplitude * (temp1 + temp2) - a2d3,
        ]
    } else {
        let d = q * q * q + r * r;
        let sqrt_d = d.sqrt();
        let s = (r + sqrt_d).cbrt();
        let t = (r - sqrt_d).cbrt();
        let real_part = -a2d3 - 0.5 * (s + t);
        [-a2d3 + (s + t), real_part, real_part]
    }
}

/// Upstream `v3Sort`: descending order by strict comparisons (NaNs stay put).
fn sort_descending(values: [f64; 3]) -> Vector3<f64> {
    let mut r = values;
    if r[0] < r[1] {
        r.swap(0, 1);
    }
    if r[1] < r[2] {
        if r[0] < r[2] {
            r = [r[2], r[0], r[1]];
        } else {
            r.swap(1, 2);
        }
    }
    Vector3::from(r)
}

/// Upstream `mDeterminant`: recursive cofactor expansion along the first row;
/// an empty matrix has determinant 0.
fn laplace_determinant(m: &DMatrix<f64>) -> f64 {
    match m.nrows() {
        0 => 0.0,
        1 => m[(0, 0)],
        2 => m[(0, 0)] * m[(1, 1)] - m[(1, 0)] * m[(0, 1)],
        dim => (0..dim).fold(0.0, |det, k| {
            let sign = if k % 2 == 0 { 1.0 } else { -1.0 };
            det + sign * m[(0, k)] * laplace_determinant(&minor(m, 0, k))
        }),
    }
}

/// `m` without row `row` and column `col`.
fn minor(m: &DMatrix<f64>, row: usize, col: usize) -> DMatrix<f64> {
    m.clone().remove_row(row).remove_column(col)
}

/// Upstream `mMultM` (and `m33MultM33`, `mMultV`, ...): `a b` with every
/// element summed in index order from `0.0`.
///
/// Use it where bit-level agreement with upstream matters: nalgebra hands
/// dynamic products whose dimensions all exceed five to `matrixmultiply`,
/// which blocks (and may fuse) the sums.
///
/// # Panics
///
/// If the inner dimensions of `a` and `b` differ.
pub fn sequential_product<R1, C1, S1, R2, C2, S2>(
    a: &Matrix<f64, R1, C1, S1>,
    b: &Matrix<f64, R2, C2, S2>,
) -> OMatrix<f64, R1, C2>
where
    R1: Dim,
    C1: Dim,
    R2: Dim,
    C2: Dim,
    S1: RawStorage<f64, R1, C1>,
    S2: RawStorage<f64, R2, C2>,
    DefaultAllocator: Allocator<R1, C2>,
{
    assert_eq!(a.ncols(), b.nrows(), "matrix product dimensions must agree");
    OMatrix::from_fn_generic(a.shape_generic().0, b.shape_generic().1, |i, j| {
        (0..a.ncols()).fold(0.0, |sum, k| sum + a[(i, k)] * b[(k, j)])
    })
}

fn to_dmatrix<R, C, S>(m: &Matrix<f64, R, C, S>) -> DMatrix<f64>
where
    R: Dim,
    C: Dim,
    S: RawStorage<f64, R, C>,
{
    DMatrix::from_fn(m.nrows(), m.ncols(), |i, j| m[(i, j)])
}

#[cfg(test)]
mod tests {
    //! Upstream has no unit test for `linearAlgebra.c`; the truth values here are
    //! derived by hand from the algorithms.

    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

    use nalgebra::{DMatrix, DVector, Matrix2x3, Matrix3, Matrix3x4, Vector2, Vector3};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    use super::*;

    fn assert_matrix_near<R, C, S1, S2>(
        actual: &Matrix<f64, R, C, S1>,
        expected: &Matrix<f64, R, C, S2>,
        tolerance: f64,
    ) where
        R: Dim,
        C: Dim,
        S1: RawStorage<f64, R, C>,
        S2: RawStorage<f64, R, C>,
    {
        assert_eq!(actual.shape(), expected.shape());
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= tolerance,
                "{actual} != {expected} within {tolerance:e}"
            );
        }
    }

    fn random_matrix(rng: &mut StdRng, rows: usize, columns: usize) -> DMatrix<f64> {
        DMatrix::from_fn(rows, columns, |_, _| rng.gen_range(-2.0..2.0))
    }

    #[test]
    fn safe_trigonometric_inverses_clamp_their_argument() {
        assert_eq!(safe_acos(1.0 + 1e-12), 0.0);
        assert_eq!(safe_acos(-1.0 - 1e-12), PI);
        assert_eq!(safe_acos(0.5), 0.5_f64.acos());
        assert!(safe_acos(f64::NAN).is_nan());
        assert_eq!(safe_asin(1.5), FRAC_PI_2);
        assert_eq!(safe_asin(-1.5), -FRAC_PI_2);
        assert_eq!(safe_asin(0.5), 0.5_f64.asin());
        assert!(safe_asin(f64::NAN).is_nan());
    }

    #[test]
    fn normalize_or_zero_zeroes_vectors_at_or_below_db0_eps() {
        assert_eq!(normalize_or_zero(Vector3::zeros()), Vector3::zeros());
        assert_eq!(
            normalize_or_zero(Vector3::new(1e-31, 0.0, 0.0)),
            Vector3::zeros()
        );
        assert_eq!(
            normalize_or_zero(Vector3::new(f64::NAN, 1.0, 0.0)),
            Vector3::zeros()
        );
        assert_matrix_near(
            &normalize_or_zero(Vector3::new(1e-29, 0.0, 0.0)),
            &Vector3::new(1.0, 0.0, 0.0),
            1e-15,
        );
        assert_matrix_near(
            &normalize_or_zero(Vector3::new(3.0, 4.0, 0.0)),
            &Vector3::new(0.6, 0.8, 0.0),
            1e-15,
        );
        let ones = normalize_or_zero(DVector::from_element(10, 1.0));
        assert_matrix_near(&ones, &DVector::from_element(10, 0.1_f64.sqrt()), 1e-15);
        assert_eq!(normalize_or_zero(DVector::zeros(4)), DVector::zeros(4));
    }

    #[test]
    fn tolerance_comparisons_follow_upstream_semantics() {
        let a = Vector3::new(1.0, 2.0, 3.0);
        assert!(is_equal_within(&a, &Vector3::new(1.0, 2.0, 3.0001), 1e-3));
        assert!(!is_equal_within(&a, &Vector3::new(1.0, 2.0, 3.0001), 1e-5));
        // A difference equal to the accuracy still counts as equal (strict `>`).
        assert!(is_equal_within(
            &Vector2::new(0.0, 0.0),
            &Vector2::new(0.0, 0.5),
            0.5
        ));
        // Upstream quirk: NaN differences never exceed the accuracy.
        assert!(is_equal_within(
            &Vector2::new(f64::NAN, 0.0),
            &Vector2::new(1.0, 0.0),
            0.0
        ));
        let m = DMatrix::from_row_slice(2, 3, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        assert!(is_equal_within(&m, &m.clone(), 0.0));
        assert!(!is_equal_within(&m, &m.transpose(), 1e9));
        assert!(is_equal_within(
            &m.view((0, 1), (2, 2)),
            &DMatrix::from_row_slice(2, 2, &[2.0, 3.0, 5.0, 6.0]),
            0.0
        ));

        assert!(is_zero_within(&Vector2::new(1e-8, -1e-8), 1e-7));
        assert!(!is_zero_within(&Vector2::new(1e-8, -1e-6), 1e-7));
        assert!(is_zero_within(&Matrix3::<f64>::zeros(), 0.0));
        assert!(is_zero_within(&Vector2::new(f64::NAN, 0.0), 0.0));
    }

    #[test]
    fn v3_perpendicular_solves_for_the_first_nonzero_component() {
        let root27 = 27.0_f64.sqrt();
        assert_matrix_near(
            &v3_perpendicular(Vector3::new(1.0, 2.0, 3.0)),
            &Vector3::new(-5.0 / root27, 1.0 / root27, 1.0 / root27),
            1e-15,
        );
        let root = 4.25_f64.sqrt();
        assert_matrix_near(
            &v3_perpendicular(Vector3::new(0.0, 2.0, 3.0)),
            &Vector3::new(1.0 / root, -1.5 / root, 1.0 / root),
            1e-15,
        );
        assert_matrix_near(
            &v3_perpendicular(Vector3::new(0.0, 0.0, 2.0)),
            &Vector3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0),
            1e-15,
        );
        // Components at or below DB0_EPS are skipped.
        assert_matrix_near(
            &v3_perpendicular(Vector3::new(1e-31, 0.0, 5.0)),
            &Vector3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0),
            1e-15,
        );
        assert_eq!(v3_perpendicular(Vector3::zeros()), Vector3::zeros());

        let mut rng = StdRng::seed_from_u64(1);
        for _ in 0..200 {
            let v = Vector3::new(
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
            );
            let p = v3_perpendicular(v);
            assert!(p.dot(&v).abs() < 1e-14);
            assert!((p.norm() - 1.0).abs() < 1e-15);
        }
    }

    #[test]
    fn m33_inverse_matches_the_adjugate_and_rejects_upstream_singular_matrices() {
        assert_eq!(
            m33_inverse(&Matrix3::from_diagonal(&Vector3::new(2.0, 4.0, 8.0))),
            Some(Matrix3::from_diagonal(&Vector3::new(0.5, 0.25, 0.125)))
        );
        let a = Matrix3::new(4.0, 7.0, 2.0, 3.0, 6.0, 1.0, 2.0, 5.0, 3.0);
        let expected = Matrix3::new(13.0, -11.0, -5.0, -7.0, 8.0, 2.0, 3.0, -6.0, 3.0) / 9.0;
        let inverse = m33_inverse(&a).expect("det = 9");
        assert_matrix_near(&inverse, &expected, 1e-15);
        assert_matrix_near(&(a * inverse), &Matrix3::identity(), 1e-14);

        let singular = Matrix3::new(1.0, 2.0, 3.0, 2.0, 4.0, 6.0, 1.0, 1.0, 1.0);
        assert_eq!(m33_inverse(&singular), None);
        // |det| = 1e-31 <= DB0_EPS is singular upstream, though invertible in floating point.
        let tiny = Matrix3::from_diagonal(&Vector3::new(1e-11, 1e-10, 1e-10));
        assert_eq!(m33_inverse(&tiny), None);
        assert!(tiny.try_inverse().is_some());
    }

    #[test]
    fn m_inverse_uses_cofactors_and_keeps_upstream_singular_handling() {
        let two = DMatrix::from_row_slice(2, 2, &[4.0, 7.0, 2.0, 6.0]);
        assert_matrix_near(
            &m_inverse(&two).expect("det = 10"),
            &DMatrix::from_row_slice(2, 2, &[0.6, -0.7, -0.2, 0.4]),
            1e-15,
        );
        let three = Matrix3::new(4.0, 7.0, 2.0, 3.0, 6.0, 1.0, 2.0, 5.0, 3.0);
        let expected = m33_inverse(&three).expect("det = 9");
        assert_matrix_near(
            &m_inverse(&three).expect("det = 9"),
            &DMatrix::from_row_slice(3, 3, expected.transpose().as_slice()),
            1e-15,
        );
        let mut rng = StdRng::seed_from_u64(2);
        for dim in 2..=6 {
            let m = random_matrix(&mut rng, dim, dim);
            let inverse = m_inverse(&m).expect("random matrices are regular");
            assert_matrix_near(&(&m * inverse), &DMatrix::identity(dim, dim), 1e-12);
        }

        assert_eq!(
            m_inverse(&DMatrix::from_row_slice(2, 2, &[1.0, 2.0, 2.0, 4.0])),
            None
        );
        // Upstream bug, preserved: a 1x1 matrix inverts to zero.
        assert_eq!(
            m_inverse(&DMatrix::from_element(1, 1, 2.0)),
            Some(DMatrix::zeros(1, 1))
        );
    }

    #[test]
    #[should_panic(expected = "square")]
    fn m_inverse_rejects_non_square_matrices() {
        let _ = m_inverse(&DMatrix::<f64>::zeros(2, 3));
    }

    #[test]
    fn sequential_product_sums_in_index_order() {
        let a = DMatrix::from_row_slice(2, 3, &[1.0, 1.0e16, -1.0e16, 2.0, 3.0, 4.0]);
        let b = DVector::from_vec(vec![1.0, 1.0, 1.0]);
        // ((0 + 1) + 1e16) - 1e16 = 0 in double precision.
        assert_eq!(
            sequential_product(&a, &b),
            DVector::from_vec(vec![0.0, 9.0])
        );

        let a = Matrix2x3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0);
        let b = Matrix3x4::from_fn(|i, j| (i * 4 + j) as f64);
        assert_eq!(sequential_product(&a, &b), a * b);
    }

    #[test]
    fn m_minimum_norm_inverse_is_a_right_inverse() {
        let m = Matrix2x3::new(1.0, 0.0, 0.0, 0.0, 2.0, 0.0);
        assert_eq!(
            m_minimum_norm_inverse(&m),
            DMatrix::from_row_slice(3, 2, &[1.0, 0.0, 0.0, 0.5, 0.0, 0.0])
        );

        let mut rng = StdRng::seed_from_u64(4);
        for (rows, columns) in [(2, 3), (3, 4), (3, 7), (6, 9)] {
            let m = random_matrix(&mut rng, rows, columns);
            let inverse = m_minimum_norm_inverse(&m);
            assert_eq!(inverse.shape(), (columns, rows));
            assert_matrix_near(&(&m * &inverse), &DMatrix::identity(rows, rows), 1e-12);
            let pseudo_inverse = m.clone().pseudo_inverse(1e-15).expect("full row rank");
            assert_matrix_near(&inverse, &pseudo_inverse, 1e-12);
        }

        let static_input = Matrix3x4::new(
            1.0, 0.0, 0.0, 1.0, //
            0.0, 1.0, 0.0, 1.0, //
            0.0, 0.0, 1.0, 1.0,
        );
        let inverse = m_minimum_norm_inverse(&static_input);
        assert_matrix_near(
            &(DMatrix::from_row_slice(3, 4, static_input.transpose().as_slice()) * inverse),
            &DMatrix::identity(3, 3),
            1e-15,
        );

        // A rank-deficient M Mᵀ is singular: upstream multiplies by its zero "inverse".
        let dependent = DMatrix::from_row_slice(2, 3, &[1.0, 2.0, 3.0, 2.0, 4.0, 6.0]);
        assert_eq!(m_minimum_norm_inverse(&dependent), DMatrix::zeros(3, 2));
        // Upstream bug, preserved: a single row hits the 1x1 `mInverse` and yields zero.
        let row = DMatrix::from_row_slice(1, 2, &[1.0, 1.0]);
        assert_eq!(m_minimum_norm_inverse(&row), DMatrix::zeros(2, 1));
    }

    #[test]
    fn m33_eigen_values_are_sorted_in_descending_order() {
        let cases = [
            (
                Matrix3::from_diagonal(&Vector3::new(1.0, 3.0, 2.0)),
                [3.0, 2.0, 1.0],
            ),
            (
                Matrix3::from_diagonal(&Vector3::new(-1.0, 4.0, 0.5)),
                [4.0, 0.5, -1.0],
            ),
            (
                Matrix3::new(2.0, 1.0, 0.0, 1.0, 2.0, 0.0, 0.0, 0.0, 5.0),
                [5.0, 3.0, 1.0],
            ),
            // Repeated roots take the Cardano branch.
            (
                Matrix3::from_diagonal(&Vector3::new(2.0, 2.0, 5.0)),
                [5.0, 2.0, 2.0],
            ),
        ];
        for (m, expected) in cases {
            assert_matrix_near(&m33_eigen_values(&m), &Vector3::from(expected), 1e-14);
        }
        assert_eq!(
            m33_eigen_values(&Matrix3::identity()),
            Vector3::new(1.0, 1.0, 1.0)
        );
    }

    #[test]
    fn m33_eigen_values_return_only_real_parts_of_a_complex_pair() {
        // Eigenvalues 2 and ±i: upstream returns the real root and 0 twice.
        let rotation = Matrix3::new(0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 2.0);
        assert_matrix_near(
            &m33_eigen_values(&rotation),
            &Vector3::new(2.0, 0.0, 0.0),
            1e-14,
        );
        // Eigenvalues 1 ± 2i and -3.
        let spiral = Matrix3::new(1.0, -2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, -3.0);
        assert_matrix_near(
            &m33_eigen_values(&spiral),
            &Vector3::new(1.0, 1.0, -3.0),
            1e-14,
        );
    }
}
