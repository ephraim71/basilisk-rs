use nalgebra::{Matrix3, Vector3};

pub(crate) fn body_to_inertial_dcm_from_sigma_bn(sigma_bn: Vector3<f64>) -> Matrix3<f64> {
    let sigma_squared = sigma_bn.norm_squared();
    let sigma_tilde = sigma_bn.cross_matrix();
    let correction = (8.0 * sigma_tilde * sigma_tilde - 4.0 * (1.0 - sigma_squared) * sigma_tilde)
        / (1.0 + sigma_squared).powi(2);
    Matrix3::identity() + correction
}

pub(crate) fn shadow_mrp(sigma_bn: Vector3<f64>) -> Vector3<f64> {
    let sigma_norm_squared = sigma_bn.norm_squared();
    if sigma_norm_squared > 1.0 {
        -sigma_bn / sigma_norm_squared
    } else {
        sigma_bn
    }
}

pub(crate) fn mrp_kinematics(sigma_bn: Vector3<f64>, omega_radps: Vector3<f64>) -> Vector3<f64> {
    0.25 * mrp_b_matrix(sigma_bn) * omega_radps
}

fn mrp_b_matrix(sigma_bn: Vector3<f64>) -> Matrix3<f64> {
    let sigma_squared = sigma_bn.norm_squared();
    (1.0 - sigma_squared) * Matrix3::identity()
        + 2.0 * sigma_bn.cross_matrix()
        + 2.0 * (sigma_bn * sigma_bn.transpose())
}
