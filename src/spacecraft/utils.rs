use nalgebra::{Matrix3, Vector3};

use crate::{messages::SpacecraftStateMsg, spacecraft::IntegratedState};

pub fn combine_effector_state_derivatives(
    weighted_derivatives: Vec<(Vec<Vec<f64>>, f64)>,
    effector_states: &Vec<Vec<f64>>,
) -> Vec<Vec<f64>> {
    weighted_derivatives.iter().for_each(|(derivatives, _)| {
        assert_eq!(
            derivatives.len(),
            effector_states.len(),
            "state-effector derivative count does not match state count"
        );
    });

    effector_states
        .iter()
        .enumerate()
        .map(|(effector_index, effector_state)| {
            (0..effector_state.len())
                .map(|state_index| {
                    weighted_derivatives
                        .iter()
                        .map(|(derivatives, weight)| {
                            derivatives[effector_index][state_index] * weight
                        })
                        .sum()
                })
                .collect()
        })
        .collect()
}

pub fn gravity_gradient_torque_body(
    body_origin_position_m: Vector3<f64>,
    body_to_inertial_dcm: Matrix3<f64>,
    mass_kg: f64,
    center_of_mass_body_m: Vector3<f64>,
    inertia_about_point_b_body_kg_m2: Matrix3<f64>,
    earth_mu_m3ps2: f64,
) -> Vector3<f64> {
    let com_position_inertial_m =
        body_origin_position_m + body_to_inertial_dcm * center_of_mass_body_m;
    let radius_m = com_position_inertial_m.norm();
    if radius_m < 1.0 {
        return Vector3::zeros();
    }
    let inertia_about_com_body_kg_m2 = inertia_about_point_b_body_kg_m2
        - mass_kg * tilde(center_of_mass_body_m) * tilde(center_of_mass_body_m).transpose();
    let r_hat_body = body_to_inertial_dcm.transpose() * (com_position_inertial_m / radius_m);
    3.0 * earth_mu_m3ps2 / radius_m.powi(3)
        * r_hat_body.cross(&(inertia_about_com_body_kg_m2 * r_hat_body))
}

pub fn spacecraft_state_msg_from_integrated_state(
    state: &IntegratedState,
    inertial_position_m: Vector3<f64>,
    inertial_velocity_mps: Vector3<f64>,
) -> SpacecraftStateMsg {
    SpacecraftStateMsg {
        position_m: inertial_position_m,
        velocity_mps: inertial_velocity_mps,
        sigma_bn: state.sigma_bn,
        omega_radps: state.omega_radps,
    }
}

pub fn tilde(vector: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}
