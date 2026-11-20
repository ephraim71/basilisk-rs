//! Upstream's initial translational state of a spacecraft with appendages:
//! `hub.r_CN_NInit` and `hub.v_CN_NInit` are the position and velocity of the
//! spacecraft's center of mass C, and `Spacecraft::initializeDynamics`
//! converts them to the hub states `r_BN_N` and `v_BN_N` with the mass
//! properties at the initial effector states:
//!
//! ```text
//! r_BN_N = r_CN_NInit - [NB] c_B
//! v_BN_N = v_CN_NInit - [NB] (c'_B + ω_BN_B × c_B)
//! ```
//!
//! basilisk-rs's `SpacecraftConfig::initial_position_m` and
//! `initial_velocity_mps` are `r_BN_N` and `v_BN_N` themselves, so the
//! multibody scenarios, whose centers of mass are off `B`, convert with
//! [`set_center_of_mass_initial_state`] once the state effectors are added.

#![allow(dead_code)]

use basilisk_rs::spacecraft::Spacecraft;
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use nalgebra::Vector3;

/// The spacecraft's center of mass `c_B` and its body-frame derivative
/// `c'_B` at the configured initial state: the hub and every state
/// effector's mass properties (upstream `updateSCMassProps(0.0)`).
pub fn initial_center_of_mass_body(spacecraft: &mut Spacecraft) -> (Vector3<f64>, Vector3<f64>) {
    let hub_mass_kg = spacecraft.config.mass_kg;
    let mut mass_kg = hub_mass_kg;
    let mut first_moment = hub_mass_kg * spacecraft.config.hub_center_of_mass_body_m;
    let mut first_moment_prime = Vector3::zeros();
    for effector in &mut spacecraft.state_effectors {
        let state = effector.initial_state();
        effector.load_state(&state);
        let props = effector.mass_properties(&state);
        mass_kg += props.mass_kg;
        first_moment += props.mass_kg * props.center_of_mass_body_m;
        first_moment_prime += props.mass_kg * props.center_of_mass_prime_body_mps;
    }
    (first_moment / mass_kg, first_moment_prime / mass_kg)
}

/// Sets the hub's initial position and velocity from upstream's
/// `r_CN_NInit` and `v_CN_NInit`, at the configured initial attitude and
/// rate and effector states. Call it after the state effectors are added.
pub fn set_center_of_mass_initial_state(
    spacecraft: &mut Spacecraft,
    r_cn_n_m: Vector3<f64>,
    v_cn_n_mps: Vector3<f64>,
) {
    let (c_b, c_prime_b) = initial_center_of_mass_body(spacecraft);
    let dcm_nb = mrp_to_dcm(spacecraft.config.initial_sigma_bn).transpose();
    let c_dot_b = c_prime_b + spacecraft.config.initial_omega_radps.cross(&c_b);
    spacecraft.config.initial_position_m = r_cn_n_m - dcm_nb * c_b;
    spacecraft.config.initial_velocity_mps = v_cn_n_mps - dcm_nb * c_dot_b;
}
