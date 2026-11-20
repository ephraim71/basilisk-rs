//! Matrix validity checks ported from upstream Basilisk
//! `architecture/utilities/avsEigenSupport.cpp`, used by simulation modules to
//! validate their configuration.
//!
//! `eigenIsRotationMatrix` and `eigenIsValidInertiaMatrix` are
//! `crate::kinematics::is_rotation_matrix` and `is_valid_inertia_matrix`.
//! The conversion helpers of `avsEigenSupport` (`eigenC2MRP`, `eigenM2`,
//! `eigenTilde`, ...) are not ported: their equivalents live in
//! [`rigid_body_kinematics`](super::rigid_body_kinematics)
//! (`dcm_to_mrp`, `euler2`, `tilde`).

use nalgebra::Matrix3;

/// Default tolerance of the upstream checks.
pub const DEFAULT_TOLERANCE: f64 = 1e-9;

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

#[cfg(test)]
mod tests {
    use super::{DEFAULT_TOLERANCE, is_positive_semidefinite};
    use nalgebra::{Matrix3, Vector3};

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
}
