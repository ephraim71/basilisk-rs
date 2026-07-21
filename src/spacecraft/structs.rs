use nalgebra::{Matrix3, Vector3};

use crate::integrators::Integrator;
use crate::spacecraft::Spacecraft;

#[derive(Clone, Debug, Default)]
pub struct BackSubMatrices {
    pub matrix_a: Matrix3<f64>,
    pub matrix_b: Matrix3<f64>,
    pub matrix_c: Matrix3<f64>,
    pub matrix_d: Matrix3<f64>,
    pub vec_trans: Vector3<f64>,
    pub vec_rot: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct IntegratedState {
    pub position_wrt_central_body_m: Vector3<f64>,
    pub velocity_wrt_central_body_mps: Vector3<f64>,
    pub sigma_bn: Vector3<f64>,
    pub omega_radps: Vector3<f64>,
    pub effector_states: Vec<Vec<f64>>,
}

#[derive(Clone, Debug)]
pub struct StateDerivative {
    pub position_wrt_central_body_dot_mps: Vector3<f64>,
    pub velocity_wrt_central_body_dot_mps2: Vector3<f64>,
    pub gravity_accel_inertial_mps2: Vector3<f64>,
    pub sigma_dot: Vector3<f64>,
    pub omega_dot_radps2: Vector3<f64>,
    pub effector_state_dots: Vec<Vec<f64>>,
}

#[derive(Clone, Debug, Default)]
pub struct EffectorOutput {
    pub force_inertial_n: Vector3<f64>,
    pub torque_body_nm: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct SpacecraftConfig {
    pub mass_kg: f64,
    /// Hub center of mass relative to the body frame origin.
    pub hub_center_of_mass_body_m: Vector3<f64>,
    /// Hub inertia about the hub center of mass, expressed in body-frame components.
    pub inertia_kg_m2: Matrix3<f64>,
    pub integration_step_nanos: u64,
    pub initial_position_m: Vector3<f64>,
    pub initial_velocity_mps: Vector3<f64>,
    pub initial_sigma_bn: Vector3<f64>,
    pub initial_omega_radps: Vector3<f64>,
    /// The integration scheme used to advance the state each time-step.
    /// Defaults to the classic Runge-Kutta method ([`Rk4`](crate::integrators::Rk4))
    /// if `None`. Set it to any built-in integrator struct, or your own type
    /// implementing [`Integrator`], boxed:
    /// `Some(Box::new(Dopri45::default()))`.
    pub integrator: Option<Box<dyn Integrator<Spacecraft>>>,
}

#[derive(Clone, Debug)]
pub struct StateEffectorMassProps {
    pub mass_kg: f64,
    pub mass_dot_kgps: f64,
    pub center_of_mass_body_m: Vector3<f64>,
    pub center_of_mass_prime_body_mps: Vector3<f64>,
    pub inertia_about_point_b_body_kg_m2: Matrix3<f64>,
    pub inertia_about_point_b_body_prime_kg_m2ps: Matrix3<f64>,
}

impl Default for StateEffectorMassProps {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m: Vector3::zeros(),
            center_of_mass_prime_body_mps: Vector3::zeros(),
            inertia_about_point_b_body_kg_m2: Matrix3::zeros(),
            inertia_about_point_b_body_prime_kg_m2ps: Matrix3::zeros(),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct SpacecraftMassProps {
    pub(super) mass_kg: f64,
    pub(super) mass_dot_kgps: f64,
    pub(super) center_of_mass_body_m: Vector3<f64>,
    pub(super) center_of_mass_prime_body_mps: Vector3<f64>,
    pub(super) inertia_about_point_b_body_kg_m2: Matrix3<f64>,
    pub(super) inertia_about_point_b_body_prime_kg_m2ps: Matrix3<f64>,
}
