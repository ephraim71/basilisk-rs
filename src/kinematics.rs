use nalgebra::{Matrix3, SymmetricEigen, Vector3};

/// Orthonormal and right-handed within `1e-9`, as Basilisk's `eigenIsRotationMatrix`.
pub(crate) fn is_rotation_matrix(dcm: &Matrix3<f64>) -> bool {
    dcm.iter().all(|v| v.is_finite())
        && (dcm.transpose() * dcm - Matrix3::identity()).norm() <= 1.0e-9
        && (dcm.determinant() - 1.0).abs() <= 1.0e-9
}

/// Finite, symmetric, at most one zero principal moment, and the principal
/// moments satisfy the triangle inequality, as Basilisk's `eigenIsValidInertiaMatrix`.
pub(crate) fn is_valid_inertia_matrix(inertia: &Matrix3<f64>) -> bool {
    const TOLERANCE: f64 = 1.0e-9;
    if !inertia.iter().all(|v| v.is_finite()) || (inertia - inertia.transpose()).norm() > TOLERANCE
    {
        return false;
    }
    let mut moments = SymmetricEigen::new(*inertia).eigenvalues;
    moments.as_mut_slice().sort_by(f64::total_cmp);
    moments[1] > TOLERANCE * moments[2] && moments[0] + moments[1] >= moments[2] * (1.0 - TOLERANCE)
}

/// Compose two successive modified Rodrigues parameter rotations, selecting
/// the shadow set to avoid the parameter singularity.
pub(crate) fn add_mrp(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    let mut first = first;
    let mut denominator =
        1.0 + first.norm_squared() * second.norm_squared() - 2.0 * first.dot(&second);

    if denominator.abs() < 0.1 {
        first = -first / first.norm_squared();
        denominator = 1.0 + first.norm_squared() * second.norm_squared() - 2.0 * first.dot(&second);
    }

    let result = ((1.0 - second.norm_squared()) * first
        + 2.0 * first.cross(&second)
        + (1.0 - first.norm_squared()) * second)
        / denominator;
    map_mrp_to_inner_set(result)
}

/// Compute the relative MRP rotation from `second` to `first`, selecting the
/// shadow set to avoid the parameter singularity.
pub(crate) fn subtract_mrp(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    let mut first = first;
    let mut denominator =
        1.0 + first.norm_squared() * second.norm_squared() + 2.0 * first.dot(&second);

    if denominator.abs() < 0.1 {
        first = -first / first.norm_squared();
        denominator = 1.0 + first.norm_squared() * second.norm_squared() + 2.0 * first.dot(&second);
    }

    let result = (2.0 * first.cross(&second) + (1.0 - second.norm_squared()) * first
        - (1.0 - first.norm_squared()) * second)
        / denominator;
    map_mrp_to_inner_set(result)
}

/// Convert MRPs to the passive direction cosine matrix.
pub(crate) fn mrp_to_dcm(mrp: Vector3<f64>) -> Matrix3<f64> {
    let q1 = mrp.x;
    let q2 = mrp.y;
    let q3 = mrp.z;
    let magnitude_squared = mrp.norm_squared();
    let scalar = 1.0 - magnitude_squared;
    let denominator = (1.0 + magnitude_squared).powi(2);

    Matrix3::new(
        4.0 * (2.0 * q1 * q1 - magnitude_squared) + scalar * scalar,
        8.0 * q1 * q2 + 4.0 * q3 * scalar,
        8.0 * q1 * q3 - 4.0 * q2 * scalar,
        8.0 * q2 * q1 - 4.0 * q3 * scalar,
        4.0 * (2.0 * q2 * q2 - magnitude_squared) + scalar * scalar,
        8.0 * q2 * q3 + 4.0 * q1 * scalar,
        8.0 * q3 * q1 + 4.0 * q2 * scalar,
        8.0 * q3 * q2 - 4.0 * q1 * scalar,
        4.0 * (2.0 * q3 * q3 - magnitude_squared) + scalar * scalar,
    ) / denominator
}

fn map_mrp_to_inner_set(mrp: Vector3<f64>) -> Vector3<f64> {
    let magnitude_squared = mrp.norm_squared();
    if magnitude_squared > 1.0 {
        -mrp / magnitude_squared
    } else {
        mrp
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use super::{add_mrp, is_rotation_matrix, is_valid_inertia_matrix, mrp_to_dcm, subtract_mrp};
    use crate::utilities::rigid_body_kinematics::euler321_to_dcm;

    #[test]
    fn matches_att_tracking_error_reference_values() {
        let correction = Vector3::new(0.01, 0.05, -0.55);
        let reference = Vector3::new(0.35, -0.25, 0.15);
        let body = Vector3::new(0.25, -0.45, 0.75);
        let corrected_reference = add_mrp(reference, -correction);
        let error = subtract_mrp(body, corrected_reference);

        assert!(
            (corrected_reference
                - Vector3::new(
                    -0.028_069_713_080_616_992,
                    -0.683_112_989_114_892_6,
                    0.568_107_819_050_883_7,
                ))
            .norm()
                < 1.0e-14
        );
        assert!(
            (error
                - Vector3::new(
                    0.183_684_148_175_340_82,
                    -0.097_444_776_941_816_55,
                    -0.098_960_695_605_181_55,
                ))
            .norm()
                < 1.0e-14
        );

        let dcm = mrp_to_dcm(body);
        assert!((dcm * dcm.transpose() - nalgebra::Matrix3::identity()).norm() < 1.0e-14);
    }

    #[test]
    fn rotation_matrix_check_accepts_rotations_and_rejects_reflections() {
        let rotation = euler321_to_dcm(Vector3::new(0.3, -1.1, 2.4));
        assert!(is_rotation_matrix(&rotation));
        assert!(is_rotation_matrix(&Matrix3::identity()));

        let reflection = Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, -1.0));
        assert!(!is_rotation_matrix(&reflection));
        assert!(!is_rotation_matrix(&(2.0 * rotation)));
        let mut non_finite = rotation;
        non_finite[(1, 2)] = f64::NAN;
        assert!(!is_rotation_matrix(&non_finite));
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
            assert!(is_valid_inertia_matrix(&matrix), "{matrix}");
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
            assert!(!is_valid_inertia_matrix(&matrix), "{matrix}");
        }
    }
}
