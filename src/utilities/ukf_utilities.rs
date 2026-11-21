//! Square-root unscented Kalman filter numerics, ported from upstream Basilisk
//! `architecture/utilities/ukfUtilities.c`.
//!
//! These routines are the numerical core of the upstream UKFs (`sunlineUKF`,
//! `sunlineSuKF`, `inertialUKF`, `headingSuKF`, `relativeODuKF`,
//! `pixelLineBiasUKF`, ...). They are deliberately *not* nalgebra's own
//! decompositions: each one reproduces upstream's algorithm, operation order and
//! sign conventions so that filter outputs match the upstream numpy-truth tests.
//! Every sum is accumulated left to right in upstream's loop order (nalgebra's
//! `dot` reorders summations).
//!
//! Conventions:
//!
//! * Matrices are [`DMatrix<f64>`] indexed `(row, column)` exactly like
//!   upstream's row-major `m[row * nCol + col]` arrays.
//! * Square-root covariances are **lower**-triangular factors `S` with
//!   `P = S S^T` (upstream `sBar`, `sy`). [`ukf_qrd_just_r`] returns the
//!   **upper**-triangular `R = S^T`; callers transpose it, as upstream does.
//! * Upstream returns `-1` on failure while still leaving a (partially) written
//!   destination buffer, which several filters keep using. Fallible routines
//!   therefore return a [`UkfResult`], whose error ([`UkfFailure`]) carries that
//!   partial output. [`UkfResultExt`] reproduces the two upstream call patterns:
//!   ignoring the status, or accumulating it into `badUpdate`.
//! * Every failure also emits the upstream warning through `log::warn!`.
//! * Upstream works in fixed `UKF_MAX_DIM = 20` scratch buffers (and overflows
//!   them for larger inputs); this port has no size limit.

use std::error::Error;
use std::fmt;

use nalgebra::{DMatrix, DVector};

/// Upstream `TINY` in `ukfLUD`: rows whose largest magnitude is below this are
/// singular, and an exactly-zero pivot is replaced by it.
pub const LU_TINY: f64 = 1.0e-14;

/// Why an upstream `ukfUtilities` routine returned `-1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum UkfError {
    /// The input matrix is not square (`nRow != nCol`).
    NonSquare {
        /// Number of rows of the offending matrix.
        rows: usize,
        /// Number of columns of the offending matrix.
        cols: usize,
    },
    /// An exactly-zero diagonal element at `index` would have been divided by
    /// (upstream "Can't invert zero matrix." / "Dividing by zero.").
    ZeroDiagonal {
        /// Row/column of the zero diagonal element.
        index: usize,
    },
    /// `ukfLUD`: row `row` has no element of magnitude `>= `[`LU_TINY`]
    /// (upstream "Singular matrix encountered in ukfLUD LU decomposition.").
    SingularRow {
        /// The (unpermuted) row that is numerically zero.
        row: usize,
    },
    /// `ukfCholDecomp` / `ukfCholDownDate`: the radicand of diagonal element
    /// `index` is negative, i.e. the matrix is not positive semi-definite
    /// (upstream "Invalid SQRT ..., skipping value.").
    NegativeSqrt {
        /// Row/column of the diagonal element whose square root failed.
        index: usize,
    },
}

impl fmt::Display for UkfError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NonSquare { rows, cols } => {
                write!(f, "matrix is not square ({rows}x{cols})")
            }
            Self::ZeroDiagonal { index } => {
                write!(
                    f,
                    "zero diagonal element at index {index} (division by zero)"
                )
            }
            Self::SingularRow { row } => write!(
                f,
                "singular matrix: row {row} has no element of magnitude >= {LU_TINY:e}"
            ),
            Self::NegativeSqrt { index } => {
                write!(f, "negative square root argument at diagonal index {index}")
            }
        }
    }
}

impl Error for UkfError {}

/// A failed routine: the [`UkfError`] plus the contents upstream leaves in its
/// destination buffer when it returns `-1` (zeros for a size error, otherwise
/// whatever was computed before the failure).
#[derive(Clone, Debug, PartialEq)]
pub struct UkfFailure<T> {
    /// Why the routine failed.
    pub error: UkfError,
    /// Upstream's destination contents at the point of failure.
    pub partial: T,
}

impl<T> UkfFailure<T> {
    /// Consumes the failure and returns upstream's partial destination contents.
    pub fn into_partial(self) -> T {
        self.partial
    }
}

impl<T> fmt::Display for UkfFailure<T> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.error.fmt(f)
    }
}

impl<T: fmt::Debug> Error for UkfFailure<T> {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        Some(&self.error)
    }
}

/// Result of a fallible `ukfUtilities` routine: `Ok` where upstream returns `0`,
/// `Err` (carrying the partial output) where it returns `-1`.
pub type UkfResult<T> = Result<T, UkfFailure<T>>;

/// The two ways upstream filters consume a routine's status.
pub trait UkfResultExt<T> {
    /// Upstream callers that ignore the return code (`sunlineUKF`,
    /// `headingSuKF`): the destination contents, complete or partial.
    fn into_output(self) -> T;

    /// Upstream `badUpdate += ukfX(...)`: the destination contents, setting
    /// `*bad_update = true` on failure (it is left unchanged on success).
    fn into_output_flagging(self, bad_update: &mut bool) -> T;
}

impl<T> UkfResultExt<T> for UkfResult<T> {
    fn into_output(self) -> T {
        self.unwrap_or_else(UkfFailure::into_partial)
    }

    fn into_output_flagging(self, bad_update: &mut bool) -> T {
        self.unwrap_or_else(|failure| {
            *bad_update = true;
            failure.partial
        })
    }
}

/// Output of [`ukf_lud`] (upstream `ukfLUD`'s `destMat` and `indx`).
#[derive(Clone, Debug, PartialEq)]
pub struct LuDecomposition {
    /// Crout-packed factors of the row-permuted input: the strictly-lower part
    /// is `L` (whose unit diagonal is implied), the upper part including the
    /// diagonal is `U`.
    pub lu: DMatrix<f64>,
    /// Upstream `indx`: at elimination step `j`, row `j` was interchanged with
    /// row `row_swaps[j]` (`>= j`). This is a sequence of swaps, not a
    /// permutation vector.
    pub row_swaps: Vec<usize>,
}

impl LuDecomposition {
    /// Sign (`+1.0` or `-1.0`) of the row permutation, i.e. the parity of the
    /// interchanges in [`Self::row_swaps`]. Upstream `ukfLUD` does not compute
    /// Numerical Recipes' `d`; this is provided for determinants.
    pub fn permutation_sign(&self) -> f64 {
        let swaps = self
            .row_swaps
            .iter()
            .enumerate()
            .filter(|&(step, &row)| step != row)
            .count();
        if swaps % 2 == 0 { 1.0 } else { -1.0 }
    }
}

/// Port of upstream `ukfQRDJustR`: the `R` factor of a QR decomposition of the
/// (generally non-square) `a`, computed with modified Gram-Schmidt.
///
/// Returns the `ncols x ncols` upper-triangular `R` with `R^T R = A^T A` and a
/// **non-negative diagonal** (column norms), i.e. numpy's `r` with rows flipped
/// to a positive diagonal. `Q` is never formed. The filters call this on the
/// stacked `(2 L + n) x n` matrix whose rows are the weighted sigma-point
/// deviations followed by the noise square root, then transpose `R` into the
/// lower-triangular `sBar`/`sy`.
///
/// `a` must have full column rank: an exactly dependent column gives a zero
/// diagonal and, as upstream, NaN in every entry computed after it (there is no
/// error path).
pub fn ukf_qrd_just_r(a: &DMatrix<f64>) -> DMatrix<f64> {
    let n = a.ncols();
    let mut source = a.clone();
    let mut r = DMatrix::zeros(n, n);
    for i in 0..n {
        let norm = sum_of_products(source.column(i).iter(), source.column(i).iter()).sqrt();
        r[(i, i)] = norm;
        let q_i = source.column(i).map(|element| element / norm);
        for j in (i + 1)..n {
            let projection = sum_of_products(source.column(j).iter(), q_i.iter());
            r[(i, j)] = projection;
            for (element, q) in source.column_mut(j).iter_mut().zip(q_i.iter()) {
                *element -= projection * q;
            }
        }
    }
    r
}

/// Port of upstream `ukfLInv`: inverse of a lower-triangular matrix by
/// back-substitution. Only the diagonal and strictly-lower part of `l` are
/// read; the result is lower-triangular.
///
/// # Errors
///
/// * [`UkfError::NonSquare`] if `l` is not square (partial output: zeros of
///   `l`'s shape).
/// * [`UkfError::ZeroDiagonal`] at the highest-index exactly-zero diagonal
///   element (partial output: the columns after it already inverted, as upstream
///   processes columns from last to first).
pub fn ukf_l_inv(l: &DMatrix<f64>) -> UkfResult<DMatrix<f64>> {
    const ROUTINE: &str = "ukfLInv";
    let (rows, cols) = l.shape();
    let mut inverse = DMatrix::zeros(rows, cols);
    if rows != cols {
        return fail(ROUTINE, UkfError::NonSquare { rows, cols }, inverse);
    }
    for i in (0..rows).rev() {
        if l[(i, i)] == 0.0 {
            return fail(ROUTINE, UkfError::ZeroDiagonal { index: i }, inverse);
        }
        inverse[(i, i)] = 1.0 / l[(i, i)];
        for j in ((i + 1)..rows).rev() {
            let sum = ((i + 1)..=j).fold(0.0, |acc, k| acc - l[(k, i)] * inverse[(j, k)]);
            inverse[(j, i)] = sum * inverse[(i, i)];
        }
    }
    Ok(inverse)
}

/// Port of upstream `ukfUInv`: inverse of an upper-triangular matrix by
/// back-substitution. Only the diagonal and strictly-upper part of `u` are
/// read; the result is upper-triangular.
///
/// # Errors
///
/// * [`UkfError::NonSquare`] if `u` is not square (partial output: zeros of
///   `u`'s shape).
/// * [`UkfError::ZeroDiagonal`] at the highest-index exactly-zero diagonal
///   element (partial output: the rows after it already inverted).
pub fn ukf_u_inv(u: &DMatrix<f64>) -> UkfResult<DMatrix<f64>> {
    const ROUTINE: &str = "ukfUInv";
    let (rows, cols) = u.shape();
    let mut inverse = DMatrix::zeros(rows, cols);
    if rows != cols {
        return fail(ROUTINE, UkfError::NonSquare { rows, cols }, inverse);
    }
    for i in (0..rows).rev() {
        if u[(i, i)] == 0.0 {
            return fail(ROUTINE, UkfError::ZeroDiagonal { index: i }, inverse);
        }
        inverse[(i, i)] = 1.0 / u[(i, i)];
        for j in ((i + 1)..rows).rev() {
            let sum = ((i + 1)..=j).fold(0.0, |acc, k| acc - u[(i, k)] * inverse[(k, j)]);
            inverse[(i, j)] = sum * inverse[(i, i)];
        }
    }
    Ok(inverse)
}

/// Port of upstream `ukfLUD`: Crout LU decomposition with implicit-scaling
/// partial pivoting (Numerical Recipes `ludcmp`), so that `L U` equals `a` with
/// the interchanges of [`LuDecomposition::row_swaps`] applied in order.
///
/// Upstream quirks kept: pivot ties go to the *last* candidate row (`>=`), and
/// an exactly-zero pivot is replaced by [`LU_TINY`] instead of failing.
///
/// # Errors
///
/// * [`UkfError::NonSquare`] if `a` is not square (partial output: zero `lu`
///   of `a`'s shape, identity `row_swaps`).
/// * [`UkfError::SingularRow`] for the first row whose largest magnitude is
///   below [`LU_TINY`] (partial output: `lu` is an unfactored copy of `a`,
///   `row_swaps` the identity).
pub fn ukf_lud(a: &DMatrix<f64>) -> UkfResult<LuDecomposition> {
    const ROUTINE: &str = "ukfLUD";
    let (rows, cols) = a.shape();
    let row_swaps: Vec<usize> = (0..rows).collect();
    if rows != cols {
        let partial = LuDecomposition {
            lu: DMatrix::zeros(rows, cols),
            row_swaps,
        };
        return fail(ROUTINE, UkfError::NonSquare { rows, cols }, partial);
    }
    let n = rows;
    let mut decomposition = LuDecomposition {
        lu: a.clone(),
        row_swaps,
    };

    // Implicit scaling: one over the largest magnitude of each row.
    let mut row_scale = Vec::with_capacity(n);
    for row in 0..n {
        let big = decomposition
            .lu
            .row(row)
            .iter()
            .fold(0.0, |big: f64, element| big.max(element.abs()));
        if big < LU_TINY {
            return fail(ROUTINE, UkfError::SingularRow { row }, decomposition);
        }
        row_scale.push(1.0 / big);
    }

    let lu = &mut decomposition.lu;
    for j in 0..n {
        for i in 0..j {
            lu[(i, j)] = (0..i).fold(lu[(i, j)], |sum, k| sum - lu[(i, k)] * lu[(k, j)]);
        }
        let mut big = 0.0;
        let mut pivot_row = j;
        for i in j..n {
            let sum = (0..j).fold(lu[(i, j)], |sum, k| sum - lu[(i, k)] * lu[(k, j)]);
            lu[(i, j)] = sum;
            let figure_of_merit = row_scale[i] * sum.abs();
            if figure_of_merit >= big {
                big = figure_of_merit;
                pivot_row = i;
            }
        }
        if j != pivot_row {
            lu.swap_rows(pivot_row, j);
            row_scale[pivot_row] = row_scale[j];
        }
        decomposition.row_swaps[j] = pivot_row;
        if lu[(j, j)] == 0.0 {
            lu[(j, j)] = LU_TINY;
        }
        if j != n - 1 {
            let reciprocal_pivot = 1.0 / lu[(j, j)];
            for i in (j + 1)..n {
                lu[(i, j)] *= reciprocal_pivot;
            }
        }
    }
    Ok(decomposition)
}

/// Port of upstream `ukfLUBckSlv`: solves `A x = b` by forward and back
/// substitution with a [`ukf_lud`] decomposition of `A` (Numerical Recipes
/// `lubksb`).
///
/// **Upstream bug kept:** the forward pass starts at the first permuted entry
/// of `b` that is non-zero *after truncation to an integer* (`(int)sum` where
/// Numerical Recipes tests `sum`). Leading fractional entries (`|b_i| < 1`) are
/// therefore not propagated to the later rows and the solution is wrong when
/// they are non-zero. Unit and integer-valued right-hand sides (the only ones
/// upstream uses, through [`ukf_mat_inv`]) are unaffected.
///
/// # Errors
///
/// * [`UkfError::NonSquare`] if `decomposition.lu` is not square (partial
///   output: zeros of length `lu.nrows()`).
/// * [`UkfError::ZeroDiagonal`] at the highest-index exactly-zero diagonal of
///   `U` (partial output: `x` with the back substitution done below that row).
///
/// # Panics
///
/// If `b.len()` differs from the size of a square `decomposition.lu`, or a
/// `row_swaps` entry is out of range.
pub fn ukf_lu_bck_slv(
    decomposition: &LuDecomposition,
    b: &DVector<f64>,
) -> UkfResult<DVector<f64>> {
    const ROUTINE: &str = "ukfLUBckSlv";
    let lu = &decomposition.lu;
    let (rows, cols) = lu.shape();
    if rows != cols {
        return fail(
            ROUTINE,
            UkfError::NonSquare { rows, cols },
            DVector::zeros(rows),
        );
    }
    assert_eq!(
        b.len(),
        rows,
        "ukf_lu_bck_slv: right-hand side length must match the LU matrix size"
    );
    let n = rows;
    let mut x = b.clone();

    let mut first_nonzero: Option<usize> = None;
    for i in 0..n {
        let pivot_row = decomposition.row_swaps[i];
        let mut sum = x[pivot_row];
        x[pivot_row] = x[i];
        if let Some(first) = first_nonzero {
            sum = (first..i).fold(sum, |acc, j| acc - lu[(i, j)] * x[j]);
        } else if sum.trunc() != 0.0 {
            // Upstream tests `(int)sum`, not `sum`: see the doc comment.
            first_nonzero = Some(i);
        }
        x[i] = sum;
    }

    for i in (0..n).rev() {
        let sum = ((i + 1)..n).fold(x[i], |acc, j| acc - lu[(i, j)] * x[j]);
        if lu[(i, i)] == 0.0 {
            return fail(ROUTINE, UkfError::ZeroDiagonal { index: i }, x);
        }
        x[i] = sum / lu[(i, i)];
    }
    Ok(x)
}

/// Port of upstream `ukfMatInv`: general matrix inverse, solving for each
/// column of the identity with [`ukf_lud`] and [`ukf_lu_bck_slv`].
///
/// Upstream quirk kept: the status of [`ukf_lud`] is ignored, so a matrix with
/// a [`UkfError::SingularRow`] is back-solved with its *unfactored* copy and
/// only fails if that copy has an exactly-zero diagonal element.
///
/// # Errors
///
/// * [`UkfError::NonSquare`] if `a` is not square (partial output: zeros of
///   `a`'s shape).
/// * [`UkfError::ZeroDiagonal`] if the back-solves divide by a zero diagonal
///   (partial output: every column as far as its back-solve got). Upstream
///   reports the last column's status, which is the same for every column.
pub fn ukf_mat_inv(a: &DMatrix<f64>) -> UkfResult<DMatrix<f64>> {
    let (rows, cols) = a.shape();
    if rows != cols {
        return fail(
            "ukfMatInv",
            UkfError::NonSquare { rows, cols },
            DMatrix::zeros(rows, cols),
        );
    }
    let decomposition = ukf_lud(a).into_output();
    let mut inverse = DMatrix::zeros(rows, cols);
    let mut status = None;
    for j in 0..rows {
        let mut unit = DVector::zeros(rows);
        unit[j] = 1.0;
        let column = match ukf_lu_bck_slv(&decomposition, &unit) {
            Ok(column) => {
                status = None;
                column
            }
            Err(failure) => {
                status = Some(failure.error);
                failure.partial
            }
        };
        inverse.set_column(j, &column);
    }
    match status {
        None => Ok(inverse),
        Some(error) => Err(UkfFailure {
            error,
            partial: inverse,
        }),
    }
}

/// Port of upstream `ukfCholDecomp`: lower-triangular Cholesky factor `L` with
/// `L L^T = a` (Cholesky-Banachiewicz, row by row). Only the diagonal and
/// strictly-lower part of `a` are read. A zero radicand is accepted and gives a
/// zero diagonal element (a singular PSD matrix can therefore succeed when only
/// its last pivot vanishes).
///
/// # Errors
///
/// * [`UkfError::NonSquare`] if `a` is not square (partial output: zeros of
///   `a`'s shape).
/// * [`UkfError::NegativeSqrt`] when a diagonal radicand is negative, i.e. `a`
///   is not positive semi-definite (partial output: the rows above it complete,
///   plus the off-diagonal entries of its own row).
/// * [`UkfError::ZeroDiagonal`] when an off-diagonal element would divide by a
///   zero diagonal element of `L` (e.g. the zero matrix).
pub fn ukf_chol_decomp(a: &DMatrix<f64>) -> UkfResult<DMatrix<f64>> {
    const ROUTINE: &str = "ukfCholDecomp";
    let (rows, cols) = a.shape();
    let mut l = DMatrix::zeros(rows, cols);
    if rows != cols {
        return fail(ROUTINE, UkfError::NonSquare { rows, cols }, l);
    }
    for i in 0..rows {
        for j in 0..=i {
            let sigma = (0..j).fold(a[(i, j)], |acc, k| acc - l[(i, k)] * l[(j, k)]);
            if i == j {
                if sigma < 0.0 {
                    return fail(ROUTINE, UkfError::NegativeSqrt { index: i }, l);
                }
                l[(i, i)] = sigma.sqrt();
            } else {
                if l[(j, j)] == 0.0 {
                    return fail(ROUTINE, UkfError::ZeroDiagonal { index: j }, l);
                }
                l[(i, j)] = sigma / l[(j, j)];
            }
        }
    }
    Ok(l)
}

/// Port of upstream `ukfCholDownDate`: rank-one update of a lower-triangular
/// Cholesky factor. Given `r` with `P = r r^T` (only its diagonal and
/// strictly-lower part are read), returns the lower-triangular `r'` with
/// `r' r'^T = P + beta x x^T`.
///
/// `beta < 0` is a down-date: the filters pass the zeroth covariance weight
/// `wC[0]` (often negative) and `-1.0` for the measurement-update columns.
/// `r` must have a non-zero diagonal; a zero element yields inf/NaN, without an
/// error, as upstream.
///
/// # Errors
///
/// [`UkfError::NegativeSqrt`] when the down-date would make `P` indefinite
/// (partial output: the columns before that diagonal element, complete).
///
/// # Panics
///
/// If `r` is not square or `x.len()` differs from its size (upstream takes a
/// single `nStates` for both).
pub fn ukf_chol_down_date(
    r: &DMatrix<f64>,
    x: &DVector<f64>,
    beta: f64,
) -> UkfResult<DMatrix<f64>> {
    let n = r.nrows();
    assert_eq!(
        r.ncols(),
        n,
        "ukf_chol_down_date: the factor must be square"
    );
    assert_eq!(
        x.len(),
        n,
        "ukf_chol_down_date: vector length must match the factor size"
    );

    let mut w = x.clone();
    let mut r_out = DMatrix::zeros(n, n);
    let mut b_param = 1.0;
    for i in 0..n {
        let r_ii = r[(i, i)];
        let r_el2 = r_ii * r_ii;
        let radicand = r_el2 + beta / b_param * w[i] * w[i];
        if radicand < 0.0 {
            return fail(
                "ukfCholDownDate",
                UkfError::NegativeSqrt { index: i },
                r_out,
            );
        }
        r_out[(i, i)] = radicand.sqrt();
        let gamma = r_el2 * b_param + beta * w[i] * w[i];
        for j in (i + 1)..n {
            w[j] -= w[i] / r_ii * r[(j, i)];
            r_out[(j, i)] = r_out[(i, i)] / r_ii * r[(j, i)];
            r_out[(j, i)] += r_out[(i, i)] * beta * w[j] * w[i] / gamma;
        }
        b_param += beta * w[i] * w[i] / r_el2;
    }
    Ok(r_out)
}

/// Logs upstream's warning for `routine` and returns the failure.
fn fail<T>(routine: &str, error: UkfError, partial: T) -> UkfResult<T> {
    log::warn!("{routine}: {error}");
    Err(UkfFailure { error, partial })
}

/// Left-to-right sum of products, in upstream's accumulation order.
fn sum_of_products<'a>(
    a: impl IntoIterator<Item = &'a f64>,
    b: impl IntoIterator<Item = &'a f64>,
) -> f64 {
    a.into_iter().zip(b).fold(0.0, |acc, (x, y)| acc + x * y)
}

#[cfg(test)]
mod tests {
    //! Ports `src/tests/test_utilitiesUKF.py` (`utilities_nominal` and
    //! `utilities_fault`). Upstream parametrizes it over the `inertialUKF` and
    //! `sunlineSuKF` SWIG wrappers, which export the same C functions, so it runs
    //! once here. The same checks are duplicated verbatim in upstream
    //! `test_SunLineUKF.py` and `test_headingSuKF.py`. numpy truth (`qr`,
    //! `cholesky`) is recomputed with nalgebra's independent decompositions.

    use nalgebra::{DMatrix, DVector};

    use super::*;

    fn upstream_numpy_r(a: &DMatrix<f64>) -> DMatrix<f64> {
        let mut r = a.clone().qr().r();
        for i in 0..r.nrows() {
            if r[(i, i)] < 0.0 {
                r.row_mut(i).neg_mut();
            }
        }
        r
    }

    fn test_matrix(n: usize, seed: f64) -> DMatrix<f64> {
        DMatrix::from_fn(n, n, |i, j| ((i * 7 + j * 3) as f64 * seed + seed).sin())
    }

    fn spd_matrix(n: usize, seed: f64) -> DMatrix<f64> {
        let m = test_matrix(n, seed);
        &m * m.transpose() + DMatrix::identity(n, n) * 0.5
    }

    fn expect_err<T: std::fmt::Debug>(result: UkfResult<T>) -> UkfFailure<T> {
        result.expect_err("upstream returns -1 here")
    }

    #[test]
    fn matches_upstream_utilities_ukf_qr_decompositions() {
        let cases: [(usize, usize, Vec<f64>, f64); 3] = [
            (
                6,
                4,
                vec![
                    0.488894, 0.888396, 0.325191, 0.319207, 1.03469, -1.14707, -0.754928, 0.312859,
                    0.726885, -1.06887, 1.3703, -0.86488, -0.303441, -0.809499, -1.71152,
                    -0.0300513, 0.293871, -2.94428, -0.102242, -0.164879, -0.787283, 1.43838,
                    -0.241447, 0.627707,
                ],
                1.0e-15,
            ),
            (
                5,
                4,
                vec![
                    1.09327,
                    1.10927,
                    -0.863653,
                    1.32288,
                    -1.21412,
                    -1.1135,
                    -0.00684933,
                    -2.43508,
                    -0.769666,
                    0.371379,
                    -0.225584,
                    -1.76492,
                    -1.08906,
                    0.0325575,
                    0.552527,
                    -1.6256,
                    1.54421,
                    0.0859311,
                    -1.49159,
                    1.59683,
                ],
                1.0e-14,
            ),
            (
                6,
                2,
                vec![
                    0.2236, 0.0, 0.0, 0.2236, -0.2236, 0.0, 0.0, -0.2236, 0.0170, 0.0, 0.0, 0.0170,
                ],
                1.0e-15,
            ),
        ];
        for (rows, cols, values, accuracy) in cases {
            let a = DMatrix::from_row_slice(rows, cols, &values);
            let r = ukf_qrd_just_r(&a);
            let error = (&r - upstream_numpy_r(&a)).norm();
            assert!(
                error <= accuracy,
                "QR decomposition accuracy failure for {rows}x{cols}: {error:e}"
            );
        }
    }

    #[test]
    fn matches_upstream_utilities_ukf_lu_decomposition() {
        let source = DMatrix::from_row_slice(3, 3, &[8.0, 1.0, 6.0, 3.0, 5.0, 7.0, 4.0, 9.0, 2.0]);
        let decomposition = ukf_lud(&source).expect("magic square is regular");
        let l = DMatrix::from_fn(3, 3, |i, j| match i.cmp(&j) {
            std::cmp::Ordering::Greater => decomposition.lu[(i, j)],
            std::cmp::Ordering::Equal => 1.0,
            std::cmp::Ordering::Less => 0.0,
        });
        let u = decomposition.lu.upper_triangle();
        // Upstream's reconstruction loop, verbatim.
        let mut out = &l * &u;
        let mut out_swap = DMatrix::zeros(3, 3);
        for i in 0..3 {
            let current_row = decomposition.row_swaps[i];
            out_swap.set_row(i, &out.row(current_row));
            let row_i = out.row(i).into_owned();
            out.set_row(current_row, &row_i);
        }
        assert!(
            (out_swap - source).norm() <= 1.0e-14,
            "LU decomposition accuracy failure"
        );
    }

    #[test]
    fn matches_upstream_utilities_ukf_lu_back_solve() {
        let equations =
            DMatrix::from_row_slice(3, 3, &[2.0, 1.0, 3.0, 2.0, 6.0, 8.0, 6.0, 8.0, 18.0]);
        let b = DVector::from_vec(vec![1.0, 3.0, 5.0]);
        let decomposition = ukf_lud(&equations).expect("regular system");
        let x = ukf_lu_bck_slv(&decomposition, &b).expect("regular system");
        let expected = [3.0 / 10.0, 4.0 / 10.0, 0.0];
        let error: f64 = x.iter().zip(expected).map(|(x, e)| (x - e).abs()).sum();
        assert!(
            error <= 1.0e-14,
            "LU back-solve accuracy failure: {error:e}"
        );
    }

    #[test]
    fn matches_upstream_utilities_ukf_matrix_inverse() {
        let source = DMatrix::from_row_slice(3, 3, &[8.0, 1.0, 6.0, 3.0, 5.0, 7.0, 4.0, 9.0, 2.0]);
        let inverse = ukf_mat_inv(&source).expect("magic square is regular");
        let error = (inverse * source - DMatrix::identity(3, 3)).norm();
        assert!(
            error <= 1.0e-14,
            "LU matrix inverse accuracy failure: {error:e}"
        );
    }

    #[test]
    fn matches_upstream_utilities_ukf_cholesky() {
        let source =
            DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]);
        let chol = ukf_chol_decomp(&source).expect("positive definite");
        let numpy = source.clone().cholesky().expect("positive definite").l();
        assert!(
            (chol - numpy).norm() <= 1.0e-14,
            "Cholesky decomposition accuracy failure"
        );
    }

    /// Upstream `InvSourceMat` (row-major lower-triangular), verbatim.
    #[allow(clippy::excessive_precision)]
    const UPSTREAM_LOWER: [f64; 16] = [
        2.1950926119414667,
        0.0,
        0.0,
        0.0,
        1.0974804773131115,
        1.9010439702743847,
        0.0,
        0.0,
        0.0,
        1.2672359635912551,
        1.7923572711881284,
        0.0,
        1.0974804773131113,
        -0.63357997864171967,
        1.7920348101787789,
        0.033997451205364251,
    ];

    #[test]
    fn matches_upstream_utilities_ukf_triangular_inverses() {
        let lower = DMatrix::from_row_slice(4, 4, &UPSTREAM_LOWER);
        let l_inv = ukf_l_inv(&lower).expect("non-zero diagonal");
        let error = (l_inv * &lower - DMatrix::identity(4, 4)).norm();
        assert!(
            error <= 1.0e-12,
            "L matrix inverse accuracy failure: {error:e}"
        );

        let upper = lower.transpose();
        let u_inv = ukf_u_inv(&upper).expect("non-zero diagonal");
        let error = (u_inv * &upper - DMatrix::identity(4, 4)).norm();
        assert!(
            error <= 1.0e-12,
            "U matrix inverse accuracy failure: {error:e}"
        );
    }

    #[test]
    fn matches_upstream_utilities_ukf_fault_cases() {
        let lu_source = DMatrix::from_row_slice(
            3,
            4,
            &[8.0, 1.0, 6.0, 3.0, 5.0, 7.0, 4.0, 9.0, 2.0, 1.0, 9.0, 3.0],
        );
        let lud = expect_err(ukf_lud(&lu_source));
        assert_eq!(lud.error, UkfError::NonSquare { rows: 3, cols: 4 });
        let b = DVector::zeros(3);
        let back_solve = expect_err(ukf_lu_bck_slv(&lud.partial, &b));
        assert_eq!(back_solve.error, UkfError::NonSquare { rows: 3, cols: 4 });

        let zeros = DMatrix::zeros(3, 3);
        let lud = expect_err(ukf_lud(&zeros));
        assert_eq!(lud.error, UkfError::SingularRow { row: 0 });
        let back_solve = expect_err(ukf_lu_bck_slv(&lud.partial, &b));
        assert_eq!(back_solve.error, UkfError::ZeroDiagonal { index: 2 });

        let inv_source = DMatrix::from_row_slice(
            3,
            4,
            &[8.0, 1.0, 6.0, 3.0, 5.0, 7.0, 4.0, 9.0, 2.0, 4.0, 4.0, 5.0],
        );
        assert_eq!(
            expect_err(ukf_mat_inv(&inv_source)).error,
            UkfError::NonSquare { rows: 3, cols: 4 }
        );

        let chol_long = DMatrix::from_row_slice(
            3,
            4,
            &[
                1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0, 10.0, 5.0, 10.0,
            ],
        );
        assert_eq!(
            expect_err(ukf_chol_decomp(&chol_long)).error,
            UkfError::NonSquare { rows: 3, cols: 4 }
        );
        assert_eq!(
            expect_err(ukf_chol_decomp(&zeros)).error,
            UkfError::ZeroDiagonal { index: 0 }
        );

        // Upstream computes this down-date's status but then re-checks the
        // previous Cholesky status (a test bug); the intended check is here.
        let r = DMatrix::from_row_slice(3, 3, &[1.0, 0.0, 0.0, 0.0, 10.0, 5.0, 0.0, 5.0, 10.0]);
        let x = DVector::from_vec(vec![100.0, 0.0, 0.0]);
        assert_eq!(
            expect_err(ukf_chol_down_date(&r, &x, -100.0)).error,
            UkfError::NegativeSqrt { index: 0 }
        );

        // Upstream `InvSourceMatRect` is UPSTREAM_LOWER with a column of ones
        // appended. Upstream passes it with nRow = nCol = int(sqrt(20)) = 4, i.e.
        // its leading 16 elements, which have a zero diagonal element.
        let rect_values: Vec<f64> = UPSTREAM_LOWER
            .chunks(4)
            .flat_map(|row| row.iter().copied().chain([1.0]))
            .collect();
        let truncated = DMatrix::from_row_slice(4, 4, &rect_values[..16]);
        assert_eq!(
            expect_err(ukf_l_inv(&truncated)).error,
            UkfError::ZeroDiagonal { index: 2 },
            "L matrix inverse didn't catch size error"
        );
        let rectangular = DMatrix::from_row_slice(4, 5, &rect_values);
        assert_eq!(
            expect_err(ukf_l_inv(&rectangular)).error,
            UkfError::NonSquare { rows: 4, cols: 5 }
        );
        assert_eq!(
            expect_err(ukf_u_inv(&rectangular)).error,
            UkfError::NonSquare { rows: 4, cols: 5 }
        );

        let zeros4 = DMatrix::zeros(4, 4);
        assert_eq!(
            expect_err(ukf_l_inv(&zeros4)).error,
            UkfError::ZeroDiagonal { index: 3 }
        );
        assert_eq!(
            expect_err(ukf_u_inv(&zeros4)).error,
            UkfError::ZeroDiagonal { index: 3 }
        );
    }

    #[test]
    fn qr_r_reproduces_the_gram_matrix_of_a_stacked_sigma_point_matrix() {
        // Shape of the sunlineUKF time update: (2L + n) x n with L = n = 6.
        let a = DMatrix::from_fn(18, 6, |i, j| ((i * 5 + j * 11) as f64 * 0.37).cos() * 0.3);
        let r = ukf_qrd_just_r(&a);
        assert_eq!(r.shape(), (6, 6));
        let gram = a.transpose() * &a;
        assert!((r.transpose() * &r - &gram).norm() <= 1.0e-13 * gram.norm());
        for i in 0..6 {
            assert!(r[(i, i)] > 0.0, "R diagonal must be positive");
            for j in 0..i {
                assert_eq!(r[(i, j)], 0.0, "R must be upper triangular");
            }
        }
        assert!((&r - upstream_numpy_r(&a)).norm() <= 1.0e-13);
    }

    #[test]
    fn qr_r_of_identity_and_scalar_edge_cases() {
        assert_eq!(
            ukf_qrd_just_r(&DMatrix::identity(4, 4)),
            DMatrix::identity(4, 4)
        );
        let column = DMatrix::from_column_slice(3, 1, &[-3.0, 0.0, 4.0]);
        assert_eq!(ukf_qrd_just_r(&column), DMatrix::from_element(1, 1, 5.0));
        let scalar = DMatrix::from_element(1, 1, -2.5);
        assert_eq!(ukf_qrd_just_r(&scalar), DMatrix::from_element(1, 1, 2.5));
    }

    #[test]
    fn qr_r_of_dependent_columns_is_nan_like_upstream() {
        let a = DMatrix::from_row_slice(3, 2, &[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]);
        let r = ukf_qrd_just_r(&a);
        assert_eq!(r[(0, 0)], 1.0);
        assert_eq!(r[(1, 1)], 0.0);
        // Column 1 = 2 * column 0: its 0/0 normalisation poisons later columns.
        let b = DMatrix::from_row_slice(3, 3, &[1.0, 2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]);
        let r = ukf_qrd_just_r(&b);
        assert_eq!(
            (r[(0, 0)], r[(0, 1)], r[(0, 2)], r[(1, 1)]),
            (1.0, 2.0, 0.0, 0.0)
        );
        assert!(r[(1, 2)].is_nan() && r[(2, 2)].is_nan());
    }

    #[test]
    fn triangular_inverses_invert_and_ignore_the_other_triangle() {
        let l = ukf_chol_decomp(&spd_matrix(6, 0.3)).expect("SPD");
        let mut l_dirty = l.clone();
        let mut u_dirty = l.transpose();
        for i in 0..6 {
            for j in (i + 1)..6 {
                l_dirty[(i, j)] = 99.0;
                u_dirty[(j, i)] = -99.0;
            }
        }
        let l_inv = ukf_l_inv(&l_dirty).expect("non-zero diagonal");
        assert!((&l * &l_inv - DMatrix::identity(6, 6)).norm() <= 1.0e-12);
        assert_eq!(l_inv, l_inv.lower_triangle());
        let u_inv = ukf_u_inv(&u_dirty).expect("non-zero diagonal");
        assert!((l.transpose() * &u_inv - DMatrix::identity(6, 6)).norm() <= 1.0e-12);
        assert_eq!(u_inv, u_inv.upper_triangle());
        // The inverse of L^T is the transpose of the inverse of L.
        assert!((u_inv - l_inv.transpose()).norm() <= 1.0e-12);

        assert_eq!(
            ukf_l_inv(&DMatrix::identity(3, 3)),
            Ok(DMatrix::identity(3, 3))
        );
        assert_eq!(
            ukf_u_inv(&DMatrix::from_element(1, 1, 4.0)),
            Ok(DMatrix::from_element(1, 1, 0.25))
        );
    }

    #[test]
    fn triangular_inverse_failure_keeps_the_already_inverted_part() {
        let l = DMatrix::from_row_slice(3, 3, &[0.0, 0.0, 0.0, 1.0, 2.0, 0.0, 3.0, 4.0, 5.0]);
        let failure = expect_err(ukf_l_inv(&l));
        assert_eq!(failure.error, UkfError::ZeroDiagonal { index: 0 });
        let expected =
            DMatrix::from_row_slice(3, 3, &[0.0, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, -0.4, 0.2]);
        assert!((failure.partial - expected).norm() <= 1.0e-15);
    }

    #[test]
    fn lu_solve_matches_nalgebra_on_a_pivoting_system() {
        let a = DMatrix::from_row_slice(
            4,
            4,
            &[
                1.0e-3, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.5, 2.0, -1.0, 0.5, 3.0, -4.0, 2.0, 1.0,
                0.25,
            ],
        );
        let b = DVector::from_vec(vec![3.0, -2.0, 7.0, 11.0]);
        let decomposition = ukf_lud(&a).expect("regular");
        assert_ne!(
            decomposition.row_swaps,
            vec![0, 1, 2, 3],
            "system must pivot"
        );
        let x = ukf_lu_bck_slv(&decomposition, &b).expect("regular");
        let truth = a.clone().lu().solve(&b).expect("regular");
        assert!((&x - truth).norm() <= 1.0e-13);
        let determinant = decomposition.permutation_sign() * decomposition.lu.diagonal().product();
        assert!((determinant - a.determinant()).abs() <= 1.0e-12 * a.determinant().abs());
    }

    #[test]
    fn lu_edge_cases() {
        let identity = ukf_lud(&DMatrix::identity(3, 3)).expect("regular");
        assert_eq!(identity.lu, DMatrix::identity(3, 3));
        assert_eq!(identity.row_swaps, vec![0, 1, 2]);
        assert_eq!(identity.permutation_sign(), 1.0);

        let scalar = ukf_lud(&DMatrix::from_element(1, 1, -4.0)).expect("regular");
        let x = ukf_lu_bck_slv(&scalar, &DVector::from_element(1, 2.0)).expect("regular");
        assert_eq!(x[0], -0.5);

        // An exactly-zero pivot becomes TINY instead of failing.
        let singular = DMatrix::from_row_slice(2, 2, &[1.0, 1.0, 1.0, 1.0]);
        let decomposition = ukf_lud(&singular).expect("upstream does not fail here");
        assert_eq!(decomposition.lu[(1, 1)], LU_TINY);

        // Pivot ties go to the last candidate row.
        let tie = DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 1.0, 1.0]);
        let decomposition = ukf_lud(&tie).expect("regular");
        assert_eq!(decomposition.row_swaps, vec![1, 1]);
        assert_eq!(decomposition.permutation_sign(), -1.0);
    }

    #[test]
    fn lu_back_solve_keeps_upstream_int_truncation_quirk() {
        // L = [[1, 0], [1, 1]], U = I, no swaps: A = [[1, 0], [1, 1]].
        let decomposition = LuDecomposition {
            lu: DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 1.0, 1.0]),
            row_swaps: vec![0, 1],
        };
        // b = [2, 3]: the true solution [2, 1].
        let x = ukf_lu_bck_slv(&decomposition, &DVector::from_vec(vec![2.0, 3.0])).expect("ok");
        assert_eq!(x.as_slice(), &[2.0, 1.0]);
        // b = [0.5, 3]: (int)0.5 == 0 hides the first entry, so upstream
        // returns [0.5, 3] instead of the true solution [0.5, 2.5].
        let x = ukf_lu_bck_slv(&decomposition, &DVector::from_vec(vec![0.5, 3.0])).expect("ok");
        assert_eq!(x.as_slice(), &[0.5, 3.0]);
    }

    #[test]
    fn matrix_inverse_matches_nalgebra() {
        let a = test_matrix(6, 0.7) + DMatrix::identity(6, 6) * 2.0;
        let inverse = ukf_mat_inv(&a).expect("regular");
        let truth = a.clone().try_inverse().expect("regular");
        assert!((&inverse - truth).norm() <= 1.0e-12);
        assert!((inverse * a - DMatrix::identity(6, 6)).norm() <= 1.0e-12);
        assert_eq!(
            ukf_mat_inv(&DMatrix::identity(2, 2)),
            Ok(DMatrix::identity(2, 2))
        );
        assert_eq!(
            ukf_mat_inv(&DMatrix::from_element(1, 1, 8.0)),
            Ok(DMatrix::from_element(1, 1, 0.125))
        );
    }

    #[test]
    fn matrix_inverse_ignores_the_lu_singular_row_status_like_upstream() {
        // Row 0 is below TINY, so ukfLUD fails and leaves an unfactored copy,
        // which happens to be a valid (diagonal) LU packing: upstream succeeds.
        let a = DMatrix::from_row_slice(2, 2, &[1.0e-15, 0.0, 0.0, 2.0]);
        assert_eq!(
            expect_err(ukf_lud(&a)).error,
            UkfError::SingularRow { row: 0 }
        );
        let inverse = ukf_mat_inv(&a).expect("upstream ignores the ukfLUD status");
        assert!((inverse[(0, 0)] - 1.0e15).abs() <= 1.0);
        assert_eq!(inverse[(1, 1)], 0.5);

        let zero_row = DMatrix::from_row_slice(2, 2, &[1.0, 2.0, 0.0, 0.0]);
        assert_eq!(
            expect_err(ukf_mat_inv(&zero_row)).error,
            UkfError::ZeroDiagonal { index: 1 }
        );
    }

    #[test]
    fn cholesky_reproduces_the_matrix_and_matches_nalgebra() {
        let a = spd_matrix(6, 0.45);
        let l = ukf_chol_decomp(&a).expect("SPD");
        assert!((&l * l.transpose() - &a).norm() <= 1.0e-13 * a.norm());
        assert_eq!(l, l.lower_triangle());
        let truth = a.clone().cholesky().expect("SPD").l();
        assert!((l - truth).norm() <= 1.0e-13);

        assert_eq!(
            ukf_chol_decomp(&DMatrix::identity(4, 4)),
            Ok(DMatrix::identity(4, 4))
        );
        assert_eq!(
            ukf_chol_decomp(&DMatrix::from_element(1, 1, 9.0)),
            Ok(DMatrix::from_element(1, 1, 3.0))
        );
    }

    #[test]
    fn cholesky_reads_only_the_lower_triangle() {
        let a = spd_matrix(4, 0.2);
        let mut dirty = a.clone();
        dirty[(0, 3)] = 1.0e6;
        dirty[(1, 2)] = -1.0e6;
        assert_eq!(ukf_chol_decomp(&dirty), ukf_chol_decomp(&a));
    }

    #[test]
    fn cholesky_of_non_positive_definite_input_fails_with_partial_factor() {
        // Singular PSD: L[1][1] = sqrt(1 - 1) = 0 is accepted, then row 2 would
        // divide by it.
        let singular =
            DMatrix::from_row_slice(3, 3, &[4.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0]);
        let failure = expect_err(ukf_chol_decomp(&singular));
        assert_eq!(failure.error, UkfError::ZeroDiagonal { index: 1 });
        assert_eq!(failure.partial[(0, 0)], 2.0);
        assert_eq!(failure.partial[(1, 0)], 1.0);

        let negative = DMatrix::from_row_slice(2, 2, &[1.0, 2.0, 2.0, 1.0]);
        let failure = expect_err(ukf_chol_decomp(&negative));
        assert_eq!(failure.error, UkfError::NegativeSqrt { index: 1 });
        assert_eq!(
            failure.partial,
            DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 2.0, 0.0])
        );
        assert_eq!(
            expect_err(ukf_chol_decomp(&DMatrix::from_element(1, 1, -1.0))).error,
            UkfError::NegativeSqrt { index: 0 }
        );

        // A singular PSD matrix whose only vanishing pivot is the last one succeeds.
        let psd = DMatrix::from_row_slice(2, 2, &[1.0, 1.0, 1.0, 1.0]);
        assert_eq!(
            ukf_chol_decomp(&psd),
            Ok(DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 1.0, 0.0]))
        );
    }

    #[test]
    fn rank_one_down_date_equals_refactoring_the_updated_matrix() {
        let p = spd_matrix(6, 0.6);
        let l = ukf_chol_decomp(&p).expect("SPD");
        let x = DVector::from_fn(6, |i, _| 0.3 * (i as f64 + 1.0).sin());
        for beta in [-1.0, -0.5, 0.25, 2.0, 1.0e-3] {
            let updated = ukf_chol_down_date(&l, &x, beta).expect("stays positive definite");
            let target = &p + beta * &x * x.transpose();
            assert!((&updated * updated.transpose() - &target).norm() <= 1.0e-12 * target.norm());
            let refactored = ukf_chol_decomp(&target).expect("stays positive definite");
            assert!(
                (&updated - refactored).norm() <= 1.0e-12,
                "down-date with beta = {beta} differs from re-factoring"
            );
        }
    }

    #[test]
    fn down_date_edge_cases() {
        let identity = DMatrix::identity(3, 3);
        let zero = DVector::zeros(3);
        assert_eq!(
            ukf_chol_down_date(&identity, &zero, -1.0),
            Ok(identity.clone())
        );

        let scalar = DMatrix::from_element(1, 1, 5.0);
        let x = DVector::from_element(1, 3.0);
        assert_eq!(
            ukf_chol_down_date(&scalar, &x, -1.0),
            Ok(DMatrix::from_element(1, 1, 4.0))
        );
        assert_eq!(
            expect_err(ukf_chol_down_date(&scalar, &x, -3.0)).error,
            UkfError::NegativeSqrt { index: 0 }
        );

        // Failing at the second diagonal keeps the completed first column.
        let x = DVector::from_vec(vec![0.0, 2.0]);
        let failure = expect_err(ukf_chol_down_date(&DMatrix::identity(2, 2), &x, -1.0));
        assert_eq!(failure.error, UkfError::NegativeSqrt { index: 1 });
        assert_eq!(
            failure.partial,
            DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 0.0, 0.0])
        );
    }

    #[test]
    fn result_extension_reproduces_upstream_status_handling() {
        let mut bad_update = false;
        let ok = ukf_chol_decomp(&DMatrix::identity(2, 2)).into_output_flagging(&mut bad_update);
        assert_eq!(ok, DMatrix::identity(2, 2));
        assert!(!bad_update);

        let negative = DMatrix::from_row_slice(2, 2, &[1.0, 2.0, 2.0, 1.0]);
        let partial = ukf_chol_decomp(&negative).into_output_flagging(&mut bad_update);
        assert!(bad_update);
        assert_eq!(
            partial,
            DMatrix::from_row_slice(2, 2, &[1.0, 0.0, 2.0, 0.0])
        );
        assert_eq!(ukf_chol_decomp(&negative).into_output(), partial);

        let ok_again =
            ukf_chol_decomp(&DMatrix::identity(1, 1)).into_output_flagging(&mut bad_update);
        assert_eq!(ok_again, DMatrix::identity(1, 1));
        assert!(bad_update, "a later success does not clear the flag");

        let failure = expect_err(ukf_l_inv(&DMatrix::zeros(2, 3)));
        assert_eq!(failure.to_string(), "matrix is not square (2x3)");
        assert_eq!(failure.into_partial(), DMatrix::zeros(2, 3));
    }
}
