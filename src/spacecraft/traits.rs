use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::structs::BackSubMatrices;
use crate::spacecraft::{EffectorOutput, StateEffectorMassProps};
use nalgebra::Vector3;
use std::any::Any;

pub trait DynamicEffector: Send {
    fn name(&self) -> &str;
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}
    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput;
    fn as_any(&self) -> &dyn Any;
}

pub trait StateEffector: Send {
    fn name(&self) -> &str;
    fn state_len(&self) -> usize;
    fn initial_state(&self) -> Vec<f64>;
    fn load_state(&mut self, state: &[f64]);
    fn pre_integration(&mut self, current_sim_nanos: u64, dt_seconds: f64);
    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    );
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64>;
    fn mass_properties(&self, _effector_state: &[f64]) -> StateEffectorMassProps {
        StateEffectorMassProps::default()
    }
    fn rotational_angular_momentum_body(
        &self,
        _effector_state: &[f64],
        _body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        Vector3::zeros()
    }
    fn rotational_energy_j(&self, _effector_state: &[f64], _body_omega_radps: Vector3<f64>) -> f64 {
        0.0
    }
    fn write_outputs(&mut self, _current_sim_nanos: u64, _hub_state: &SpacecraftStateMsg) {}
    fn as_any(&self) -> &dyn Any;
}
