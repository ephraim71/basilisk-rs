use nalgebra::{Matrix3, Vector3};

/// Compose two successive modified Rodrigues parameter rotations using the
/// same shadow-set selection as Basilisk's `addMRP`.
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

/// Compute the relative MRP rotation from `second` to `first`, matching
/// Basilisk's `subMRP` operation and shadow-set behavior.
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

/// Convert MRPs to the passive direction cosine matrix returned by
/// Basilisk's `MRP2C`.
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
    use nalgebra::Vector3;

    use super::{add_mrp, mrp_to_dcm, subtract_mrp};

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
}
