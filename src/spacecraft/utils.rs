use nalgebra::{Matrix3, Vector3};

use crate::{messages::SpacecraftStateMsg, spacecraft::IntegratedState};

pub fn combine_effector_state_derivatives(
    weighted_derivatives: Vec<(Vec<Vec<f64>>, f64)>,
    effector_states: &[Vec<f64>],
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

/// The first-order gravity-gradient torque of one planet, shared with
/// [`GravityGradientEffector`](crate::dynamics::gravity_gradient_effector::GravityGradientEffector)
/// so that the spacecraft's single-body switch and the effector apply one
/// implementation.
pub use crate::dynamics::gravity_gradient_effector::gravity_gradient_torque_body;

/// The kinematic part of the state message for `state`; the accumulated ΔV
/// is left zero, as the spacecraft fills it only in the messages it
/// publishes (dynamic effectors see stage states without it).
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
        ..SpacecraftStateMsg::default()
    }
}

pub fn tilde(vector: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}
