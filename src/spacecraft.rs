use crate::gravity::GravityEffector;
use crate::messages::{Input, Output, PlanetStateMsg, SpacecraftMassPropsMsg, SpacecraftStateMsg};
use hifitime::{Duration, Epoch};
use nalgebra::{Matrix3, Vector3};
use std::any::Any;

use self::mrp::{body_to_inertial_dcm_from_sigma_bn, mrp_kinematics, shadow_mrp};
use crate::{Module, SimulationContext};

mod mrp;

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
}

#[derive(Clone, Debug, Default)]
pub struct EffectorOutput {
    pub force_inertial_n: Vector3<f64>,
    pub torque_body_nm: Vector3<f64>,
}

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
pub struct StateEffectorMassProps {
    pub mass_kg: f64,
    pub center_of_mass_body_m: Vector3<f64>,
    pub inertia_about_point_b_body_kg_m2: Matrix3<f64>,
}

impl Default for StateEffectorMassProps {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_about_point_b_body_kg_m2: Matrix3::zeros(),
        }
    }
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
    fn write_outputs(&mut self, _current_sim_nanos: u64, _hub_state: &SpacecraftStateMsg) {}
    fn as_any(&self) -> &dyn Any;
}

pub trait DynamicEffector: Send {
    fn name(&self) -> &str;
    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}
    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput;
    fn as_any(&self) -> &dyn Any;
}

pub struct Spacecraft {
    pub config: SpacecraftConfig,
    pub state_out: Output<SpacecraftStateMsg>,
    pub mass_props_out: Output<SpacecraftMassPropsMsg>,
    pub gravity: GravityEffector,
    pub state_effectors: Vec<Box<dyn StateEffector>>,
    pub dynamic_effectors: Vec<Box<dyn DynamicEffector>>,
    integration_step_seconds: f64,
    integration_step_duration: Duration,
    integrated_state: Option<IntegratedState>,
}

impl Spacecraft {
    pub fn new(config: SpacecraftConfig) -> Self {
        let integration_step_seconds = config.integration_step_nanos as f64 * 1.0e-9;
        let integration_step_duration =
            Duration::from_total_nanoseconds(config.integration_step_nanos as i128);
        Self {
            state_out: Output::default(),
            mass_props_out: Output::default(),
            config,
            gravity: GravityEffector::new(),
            state_effectors: Vec::new(),
            dynamic_effectors: Vec::new(),
            integration_step_seconds,
            integration_step_duration,
            integrated_state: None,
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.gravity.set_timing_enabled(enabled);
    }

    pub fn add_state_effector<E>(&mut self, effector: E)
    where
        E: StateEffector + 'static,
    {
        self.state_effectors.push(Box::new(effector));
    }

    pub fn add_dynamic_effector<E>(&mut self, effector: E)
    where
        E: DynamicEffector + 'static,
    {
        self.dynamic_effectors.push(Box::new(effector));
    }

    pub fn add_grav_body(&mut self, grav_body: crate::gravity::GravBodyData) {
        self.gravity.add_grav_body(grav_body);
    }

    pub fn grav_body_input_mut(&mut self, planet_name: &str) -> Option<&mut Input<PlanetStateMsg>> {
        self.gravity.planet_body_input_mut(planet_name)
    }
}

impl Module for Spacecraft {
    fn init(&mut self) {
        self.integrated_state = Some(IntegratedState {
            position_wrt_central_body_m: self.config.initial_position_m,
            velocity_wrt_central_body_mps: self.config.initial_velocity_mps,
            sigma_bn: self.config.initial_sigma_bn,
            omega_radps: self.config.initial_omega_radps,
            effector_states: self.initial_effector_states(),
        });
        self.sync_effectors_from_integrated_state();

        let integrated_state = self
            .integrated_state
            .as_ref()
            .expect("spacecraft integrated state must exist");
        let (inertial_position_m, inertial_velocity_mps) =
            self.gravity.inertial_position_and_velocity(
                integrated_state.position_wrt_central_body_m,
                integrated_state.velocity_wrt_central_body_mps,
                0,
            );

        let state_msg = SpacecraftStateMsg {
            position_m: inertial_position_m,
            velocity_mps: inertial_velocity_mps,
            sigma_bn: self.config.initial_sigma_bn,
            omega_radps: self.config.initial_omega_radps,
        };
        self.state_out.write(state_msg.clone());
        self.write_state_effector_outputs(0, &state_msg);
        self.mass_props_out.write(self.current_mass_props_msg());
    }

    fn update(&mut self, context: &SimulationContext) {
        if context.current_sim_nanos == 0 {
            return;
        }

        let dt_seconds = self.integration_step_seconds;
        let step_start_nanos = context
            .current_sim_nanos
            .saturating_sub(self.config.integration_step_nanos);
        let step_start_epoch = context.current_epoch - self.integration_step_duration;

        if dt_seconds == 0.0 {
            return;
        }

        self.gravity.update_cache(
            context.current_sim_nanos,
            context.current_epoch,
            self.config.integration_step_nanos,
        );

        self.sync_effectors_from_integrated_state();

        for effector in &mut self.state_effectors {
            effector.pre_integration(step_start_nanos, dt_seconds);
        }

        for effector in &mut self.dynamic_effectors {
            effector.pre_integration(step_start_nanos, dt_seconds);
        }

        let integrated_state = self
            .integrated_state
            .clone()
            .expect("spacecraft integrated state must be initialized");
        let new_state = self.propagate_rk4(
            &integrated_state,
            step_start_nanos,
            step_start_epoch,
            dt_seconds,
        );

        self.integrated_state = Some(new_state);

        self.sync_effectors_from_integrated_state();
        let integrated_state = self
            .integrated_state
            .as_ref()
            .expect("spacecraft integrated state must remain initialized");
        let (inertial_position_m, inertial_velocity_mps) =
            self.gravity.inertial_position_and_velocity(
                integrated_state.position_wrt_central_body_m,
                integrated_state.velocity_wrt_central_body_mps,
                context.current_sim_nanos,
            );

        let state_msg = SpacecraftStateMsg {
            position_m: inertial_position_m,
            velocity_mps: inertial_velocity_mps,
            sigma_bn: integrated_state.sigma_bn,
            omega_radps: integrated_state.omega_radps,
        };
        self.state_out.write(state_msg.clone());
        self.write_state_effector_outputs(context.current_sim_nanos, &state_msg);
        self.mass_props_out.write(self.current_mass_props_msg());
    }
}

#[derive(Clone, Debug)]
struct IntegratedState {
    position_wrt_central_body_m: Vector3<f64>,
    velocity_wrt_central_body_mps: Vector3<f64>,
    sigma_bn: Vector3<f64>,
    omega_radps: Vector3<f64>,
    effector_states: Vec<Vec<f64>>,
}

#[derive(Clone, Debug)]
struct StateDerivative {
    position_wrt_central_body_dot_mps: Vector3<f64>,
    velocity_wrt_central_body_dot_mps2: Vector3<f64>,
    sigma_dot: Vector3<f64>,
    omega_dot_radps2: Vector3<f64>,
    effector_state_dots: Vec<Vec<f64>>,
}

impl Spacecraft {
    fn initial_effector_states(&self) -> Vec<Vec<f64>> {
        self.state_effectors
            .iter()
            .map(|effector| {
                let state = effector.initial_state();
                assert_eq!(
                    state.len(),
                    effector.state_len(),
                    "state effector '{}' returned an invalid initial state length",
                    effector.name()
                );
                state
            })
            .collect()
    }

    fn sync_effectors_from_integrated_state(&mut self) {
        let Some(integrated_state) = &self.integrated_state else {
            return;
        };

        assert_eq!(
            self.state_effectors.len(),
            integrated_state.effector_states.len(),
            "state effector count does not match integrated state count"
        );

        for (effector, effector_state) in self
            .state_effectors
            .iter_mut()
            .zip(integrated_state.effector_states.iter())
        {
            assert_eq!(
                effector.state_len(),
                effector_state.len(),
                "state length mismatch for state effector '{}'",
                effector.name()
            );
            effector.load_state(effector_state);
        }
    }

    fn propagate_rk4(
        &mut self,
        state: &IntegratedState,
        current_sim_nanos: u64,
        current_epoch: Epoch,
        dt_seconds: f64,
    ) -> IntegratedState {
        let k1 = self.equations_of_motion(state, current_sim_nanos, current_epoch);
        let k2 = self.equations_of_motion(
            &self.state_with_derivative(state, &k1, dt_seconds * 0.5),
            current_sim_nanos + (dt_seconds * 0.5 * 1.0e9) as u64,
            current_epoch + Duration::from_seconds(dt_seconds * 0.5),
        );
        let k3 = self.equations_of_motion(
            &self.state_with_derivative(state, &k2, dt_seconds * 0.5),
            current_sim_nanos + (dt_seconds * 0.5 * 1.0e9) as u64,
            current_epoch + Duration::from_seconds(dt_seconds * 0.5),
        );
        let k4 = self.equations_of_motion(
            &self.state_with_derivative(state, &k3, dt_seconds),
            current_sim_nanos + (dt_seconds * 1.0e9) as u64,
            current_epoch + Duration::from_seconds(dt_seconds),
        );

        let combined_position_dot = k1.position_wrt_central_body_dot_mps
            + 2.0 * k2.position_wrt_central_body_dot_mps
            + 2.0 * k3.position_wrt_central_body_dot_mps
            + k4.position_wrt_central_body_dot_mps;
        let combined_velocity_dot = k1.velocity_wrt_central_body_dot_mps2
            + 2.0 * k2.velocity_wrt_central_body_dot_mps2
            + 2.0 * k3.velocity_wrt_central_body_dot_mps2
            + k4.velocity_wrt_central_body_dot_mps2;
        let combined_omega_dot = k1.omega_dot_radps2
            + 2.0 * k2.omega_dot_radps2
            + 2.0 * k3.omega_dot_radps2
            + k4.omega_dot_radps2;
        let combined_effector_state_dots = combine_effector_state_derivatives(
            [
                (&k1.effector_state_dots, 1.0),
                (&k2.effector_state_dots, 2.0),
                (&k3.effector_state_dots, 2.0),
                (&k4.effector_state_dots, 1.0),
            ],
            &state.effector_states,
        );
        IntegratedState {
            position_wrt_central_body_m: state.position_wrt_central_body_m
                + combined_position_dot * (dt_seconds / 6.0),
            velocity_wrt_central_body_mps: state.velocity_wrt_central_body_mps
                + combined_velocity_dot * (dt_seconds / 6.0),
            sigma_bn: shadow_mrp(
                state.sigma_bn
                    + (k1.sigma_dot + 2.0 * k2.sigma_dot + 2.0 * k3.sigma_dot + k4.sigma_dot)
                        * (dt_seconds / 6.0),
            ),
            omega_radps: state.omega_radps + combined_omega_dot * (dt_seconds / 6.0),
            effector_states: state
                .effector_states
                .iter()
                .zip(combined_effector_state_dots.iter())
                .map(|(effector_state, effector_state_dot)| {
                    effector_state
                        .iter()
                        .zip(effector_state_dot.iter())
                        .map(|(value, value_dot)| value + value_dot * (dt_seconds / 6.0))
                        .collect()
                })
                .collect(),
        }
    }

    fn state_with_derivative(
        &self,
        state: &IntegratedState,
        derivative: &StateDerivative,
        dt_seconds: f64,
    ) -> IntegratedState {
        IntegratedState {
            position_wrt_central_body_m: state.position_wrt_central_body_m
                + derivative.position_wrt_central_body_dot_mps * dt_seconds,
            velocity_wrt_central_body_mps: state.velocity_wrt_central_body_mps
                + derivative.velocity_wrt_central_body_dot_mps2 * dt_seconds,
            sigma_bn: shadow_mrp(state.sigma_bn + derivative.sigma_dot * dt_seconds),
            omega_radps: state.omega_radps + derivative.omega_dot_radps2 * dt_seconds,
            effector_states: state
                .effector_states
                .iter()
                .zip(derivative.effector_state_dots.iter())
                .map(|(effector_state, effector_state_dot)| {
                    effector_state
                        .iter()
                        .zip(effector_state_dot.iter())
                        .map(|(value, value_dot)| value + value_dot * dt_seconds)
                        .collect()
                })
                .collect(),
        }
    }

    fn equations_of_motion(
        &mut self,
        state: &IntegratedState,
        current_sim_nanos: u64,
        current_epoch: Epoch,
    ) -> StateDerivative {
        let mass_props = self.mass_props_msg_for_effector_states(&state.effector_states);
        let inertia = mass_props.inertia_about_point_b_body_kg_m2;
        let body_to_inertial_dcm = body_to_inertial_dcm_from_sigma_bn(state.sigma_bn);
        let inertial_to_body_dcm = body_to_inertial_dcm.transpose();
        let (inertial_position_m, inertial_velocity_mps) =
            self.gravity.inertial_position_and_velocity(
                state.position_wrt_central_body_m,
                state.velocity_wrt_central_body_mps,
                current_sim_nanos,
            );
        let state_msg = spacecraft_state_msg_from_integrated_state(
            state,
            inertial_position_m,
            inertial_velocity_mps,
        );

        let center_of_mass_tilde = tilde(mass_props.center_of_mass_body_m);
        let mut back_sub = BackSubMatrices {
            matrix_a: mass_props.mass_kg * Matrix3::identity(),
            matrix_b: -mass_props.mass_kg * center_of_mass_tilde,
            matrix_c: mass_props.mass_kg * center_of_mass_tilde,
            matrix_d: inertia,
            vec_trans: -mass_props.mass_kg
                * state
                    .omega_radps
                    .cross(&state.omega_radps.cross(&mass_props.center_of_mass_body_m)),
            vec_rot: Vector3::zeros(),
        };
        let gravity_accel = self.gravity.compute_gravity_field(
            state.position_wrt_central_body_m,
            current_epoch,
            current_sim_nanos,
        );
        let gravity_body_mps2 = inertial_to_body_dcm * gravity_accel;

        for effector in &self.dynamic_effectors {
            let output = effector.compute_output(&state_msg);
            back_sub.vec_trans += inertial_to_body_dcm * output.force_inertial_n;
            back_sub.vec_rot += output.torque_body_nm;
        }

        assert_eq!(
            self.state_effectors.len(),
            state.effector_states.len(),
            "state effector count does not match integrated state count"
        );
        for (effector, effector_state) in self
            .state_effectors
            .iter()
            .zip(state.effector_states.iter())
        {
            effector.update_contributions(
                effector_state,
                state.omega_radps,
                gravity_body_mps2,
                &mut back_sub,
            );
        }
        let angular_momentum = inertia * state.omega_radps;
        back_sub.vec_rot += -state.omega_radps.cross(&angular_momentum);
        let gravity_force_body = inertial_to_body_dcm * (mass_props.mass_kg * gravity_accel);
        back_sub.vec_trans += gravity_force_body;

        let matrix_a_inverse = back_sub
            .matrix_a
            .try_inverse()
            .expect("spacecraft translational mass matrix must be invertible");
        let rotation_rhs =
            back_sub.vec_rot - back_sub.matrix_c * matrix_a_inverse * back_sub.vec_trans;
        let rotation_lhs =
            back_sub.matrix_d - back_sub.matrix_c * matrix_a_inverse * back_sub.matrix_b;
        let omega_dot = rotation_lhs
            .try_inverse()
            .expect("spacecraft rotational back-sub matrix must be invertible")
            * rotation_rhs;
        let body_frame_translational_accel =
            matrix_a_inverse * (back_sub.vec_trans - back_sub.matrix_b * omega_dot);
        let translational_accel = body_to_inertial_dcm * body_frame_translational_accel;

        let effector_state_dots = self
            .state_effectors
            .iter()
            .zip(state.effector_states.iter())
            .map(|(effector, effector_state)| {
                effector.compute_derivatives(
                    effector_state,
                    body_frame_translational_accel,
                    omega_dot,
                )
            })
            .collect();

        StateDerivative {
            position_wrt_central_body_dot_mps: state.velocity_wrt_central_body_mps,
            velocity_wrt_central_body_dot_mps2: translational_accel,
            sigma_dot: mrp_kinematics(state.sigma_bn, state.omega_radps),
            omega_dot_radps2: omega_dot,
            effector_state_dots,
        }
    }

    fn current_mass_props_msg(&self) -> SpacecraftMassPropsMsg {
        if let Some(integrated_state) = &self.integrated_state {
            return self.mass_props_msg_for_effector_states(&integrated_state.effector_states);
        }

        self.mass_props_msg_for_effector_states(&[])
    }

    fn mass_props_msg_for_effector_states(
        &self,
        effector_states: &[Vec<f64>],
    ) -> SpacecraftMassPropsMsg {
        let hub_center_of_mass_tilde = tilde(self.config.hub_center_of_mass_body_m);
        let hub_inertia_about_point_b = self.config.inertia_kg_m2
            + self.config.mass_kg * hub_center_of_mass_tilde * hub_center_of_mass_tilde.transpose();

        assert_eq!(
            self.state_effectors.len(),
            effector_states.len(),
            "state effector count does not match integrated state count"
        );

        let (effector_mass_kg, effectors_first_moment, effector_inertia_about_point_b) = self
            .state_effectors
            .iter()
            .zip(effector_states.iter())
            .fold(
                (0.0, Vector3::zeros(), Matrix3::zeros()),
                |(mass_sum, first_moment_sum, inertia_sum), (effector, effector_state)| {
                    assert_eq!(
                        effector.state_len(),
                        effector_state.len(),
                        "state length mismatch for state effector '{}'",
                        effector.name()
                    );
                    let props = effector.mass_properties(effector_state);
                    (
                        mass_sum + props.mass_kg,
                        first_moment_sum + props.mass_kg * props.center_of_mass_body_m,
                        inertia_sum + props.inertia_about_point_b_body_kg_m2,
                    )
                },
            );
        let total_mass_kg = self.config.mass_kg + effector_mass_kg;
        let total_first_moment =
            self.config.mass_kg * self.config.hub_center_of_mass_body_m + effectors_first_moment;

        let center_of_mass_body_m = if total_mass_kg > 0.0 {
            total_first_moment / total_mass_kg
        } else {
            Vector3::zeros()
        };

        SpacecraftMassPropsMsg {
            mass_kg: total_mass_kg,
            center_of_mass_body_m,
            inertia_about_point_b_body_kg_m2: hub_inertia_about_point_b
                + effector_inertia_about_point_b,
        }
    }

    fn write_state_effector_outputs(
        &mut self,
        current_sim_nanos: u64,
        hub_state: &SpacecraftStateMsg,
    ) {
        for effector in &mut self.state_effectors {
            effector.write_outputs(current_sim_nanos, hub_state);
        }
    }
}

fn combine_effector_state_derivatives(
    weighted_derivatives: [(&[Vec<f64>], f64); 4],
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

fn spacecraft_state_msg_from_integrated_state(
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

fn tilde(vector: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -vector.z, vector.y, vector.z, 0.0, -vector.x, -vector.y, vector.x, 0.0,
    )
}

#[cfg(test)]
mod tests;
