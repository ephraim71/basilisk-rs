//! Matrix validity checks ported from upstream Basilisk
//! `architecture/utilities/avsEigenSupport.cpp`, used by simulation modules to
//! validate their configuration.
//!
//! The conversion helpers of `avsEigenSupport` (`eigenC2MRP`, `eigenM2`,
//! `eigenTilde`, ...) are not ported: their equivalents live in
//! [`rigid_body_kinematics`](super::rigid_body_kinematics)
//! (`dcm_to_mrp`, `euler2`, `tilde`).

use nalgebra::Matrix3;

/// Default tolerance of the upstream checks.
pub const DEFAULT_TOLERANCE: f64 = 1e-9;

/// Whether `dcm` is a proper rotation matrix: `‖DᵀD − I‖ <= tolerance` (Frobenius
/// norm) and `|det D − 1| <= tolerance` (upstream `eigenIsRotationMatrix`). A
/// matrix with a non-finite entry fails both tests.
pub fn is_rotation_matrix(dcm: &Matrix3<f64>, tolerance: f64) -> bool {
    let is_orthonormal = (dcm.transpose() * dcm - Matrix3::identity()).norm() <= tolerance;
    let is_right_handed = (dcm.determinant() - 1.0).abs() <= tolerance;
    is_orthonormal && is_right_handed
}

/// Whether `matrix` is finite, symmetric (`‖M − Mᵀ‖ <= tolerance`) and positive
/// semidefinite (smallest eigenvalue `>= -tolerance`) (upstream
/// `eigenIsPositiveSemidefiniteMatrix`).
pub fn is_positive_semidefinite(matrix: &Matrix3<f64>, tolerance: f64) -> bool {
    if !matrix.iter().all(|value| value.is_finite()) {
        return false;
    }
    if (matrix - matrix.transpose()).norm() > tolerance {
        return false;
    }
    matrix.symmetric_eigen().eigenvalues.min() >= -tolerance
}

/// Whether `inertia` is a physically realizable inertia tensor (upstream
/// `eigenIsValidInertiaMatrix`): finite, symmetric (`‖I − Iᵀ‖ <= tolerance`),
/// with at most one zero principal moment, and with principal moments
/// `λ0 <= λ1 <= λ2` satisfying the triangle inequality `λ0 + λ1 >= λ2 (1 − tolerance)`,
/// which also makes it positive semidefinite. The zero-moment check is
/// relative: `λ1 > tolerance · λ2`.
pub fn is_valid_inertia_matrix(inertia: &Matrix3<f64>, tolerance: f64) -> bool {
    if !inertia.iter().all(|value| value.is_finite()) {
        return false;
    }
    if (inertia - inertia.transpose()).norm() > tolerance {
        return false;
    }
    let mut moments: Vec<f64> = inertia
        .symmetric_eigen()
        .eigenvalues
        .iter()
        .copied()
        .collect();
    moments.sort_by(f64::total_cmp);
    if moments[1] <= tolerance * moments[2] {
        return false;
    }
    moments[0] + moments[1] >= moments[2] * (1.0 - tolerance)
}

#[cfg(test)]
mod tests {
    use super::{
        DEFAULT_TOLERANCE, is_positive_semidefinite, is_rotation_matrix, is_valid_inertia_matrix,
    };
    use crate::utilities::rigid_body_kinematics::euler321_to_dcm;
    use nalgebra::{Matrix3, Vector3};

    #[test]
    fn rotation_matrix_check_accepts_rotations_and_rejects_reflections() {
        let rotation = euler321_to_dcm(Vector3::new(0.3, -1.1, 2.4));
        assert!(is_rotation_matrix(&rotation, DEFAULT_TOLERANCE));
        assert!(is_rotation_matrix(&Matrix3::identity(), DEFAULT_TOLERANCE));

        let reflection = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0));
        assert!(!is_rotation_matrix(&reflection, DEFAULT_TOLERANCE));
        assert!(!is_rotation_matrix(&(2.0 * rotation), DEFAULT_TOLERANCE));
        let mut non_finite = rotation;
        non_finite[(1, 2)] = f64::NAN;
        assert!(!is_rotation_matrix(&non_finite, DEFAULT_TOLERANCE));
    }

    #[test]
    fn semidefinite_check_matches_upstream_damping_cases() {
        // Accepted and rejected damping matrices of upstream test_sphericalPendulum.py.
        let accepted = [
            Matrix3::zeros(),
            Matrix3::from_diagonal(&Vector3::new(10.0, 10.0, 10.0)),
            Matrix3::from_diagonal(&Vector3::new(50.0, 0.0, 0.0)),
        ];
        for matrix in accepted {
            assert!(is_positive_semidefinite(&matrix, DEFAULT_TOLERANCE));
        }
        let rejected = [
            Matrix3::new(1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, 1.0)),
            Matrix3::new(1.0, f64::NAN, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        ];
        for matrix in rejected {
            assert!(!is_positive_semidefinite(&matrix, DEFAULT_TOLERANCE));
        }
    }

    #[test]
    fn inertia_check_matches_upstream_validation_cases() {
        // Valid and invalid tensors of the upstream state effector reset checks
        // (`test_spinningBodyOneDOFResetChecks.py`,
        // `test_linearTranslationOneDOFStateEffector.py`).
        let accepted = [
            Matrix3::from_diagonal(&Vector3::new(50.0, 30.0, 40.0)),
            Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, 60.0)),
            // A thin rod: one zero principal moment.
            Matrix3::from_diagonal(&Vector3::new(0.0, 10.0, 10.0)),
        ];
        for matrix in accepted {
            assert!(
                is_valid_inertia_matrix(&matrix, DEFAULT_TOLERANCE),
                "{matrix}"
            );
        }
        let rejected = [
            Matrix3::new(50.0, 5.0, 0.0, 0.0, 30.0, 0.0, 0.0, 0.0, 40.0),
            Matrix3::from_diagonal(&Vector3::new(-50.0, 30.0, 40.0)),
            Matrix3::from_diagonal(&Vector3::new(50.0, 80.0, -10.0)),
            Matrix3::from_diagonal(&Vector3::new(10.0, 10.0, 90.0)),
            Matrix3::zeros(),
            Matrix3::from_diagonal(&Vector3::new(0.0, 0.0, 1.0)),
            Matrix3::new(1.0, f64::NAN, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        ];
        for matrix in rejected {
            assert!(
                !is_valid_inertia_matrix(&matrix, DEFAULT_TOLERANCE),
                "{matrix}"
            );
        }
    }
}
