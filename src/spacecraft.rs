use crate::dynamics::gravity::{GravBodyData, GravityEffector, GravityError};
use crate::integrators::Rk4;
use crate::integrators::traits::DynamicObject;
use crate::messages::{
    Input, Output, PlanetStateMsg, SpacecraftDiagnosticsMsg, SpacecraftMassPropsMsg,
    SpacecraftStateMsg,
};
use crate::{Module, SimulationContext};
use hifitime::{Duration, Epoch};
use mrp::{body_to_inertial_dcm_from_sigma_bn, mrp_kinematics, shadow_mrp};
use nalgebra::{Matrix3, Vector3};
use utils::{
    combine_effector_state_derivatives, gravity_gradient_torque_body,
    spacecraft_state_msg_from_integrated_state, tilde,
};

pub mod mrp;
pub mod structs;
pub mod traits;
pub mod utils;

pub use structs::*;
pub use traits::*;

pub struct Spacecraft {
    pub config: SpacecraftConfig,
    pub state_out: Output<SpacecraftStateMsg>,
    pub mass_props_out: Output<SpacecraftMassPropsMsg>,
    pub diagnostics_out: Output<SpacecraftDiagnosticsMsg>,
    pub gravity: GravityEffector,
    pub state_effectors: Vec<Box<dyn StateEffector>>,
    pub dynamic_effectors: Vec<Box<dyn DynamicEffector>>,
    gravity_gradient_mu_m3ps2: Option<f64>,
    integration_step_seconds: f64,
    integration_step_duration: Duration,
    integrated_state: Option<IntegratedState>,
    last_gravity_accel_inertial_mps2: Vector3<f64>,
    last_non_conservative_accel_body_mps2: Vector3<f64>,
}

impl Spacecraft {
    pub fn new(config: SpacecraftConfig) -> Self {
        let integration_step_seconds = config.integration_step_nanos as f64 * 1.0e-9;
        let integration_step_duration =
            Duration::from_total_nanoseconds(config.integration_step_nanos as i128);
        Self {
            state_out: Output::default(),
            mass_props_out: Output::default(),
            diagnostics_out: Output::default(),
            config,
            gravity: GravityEffector::new(),
            state_effectors: Vec::new(),
            dynamic_effectors: Vec::new(),
            gravity_gradient_mu_m3ps2: None,
            integration_step_seconds,
            integration_step_duration,
            integrated_state: None,
            last_gravity_accel_inertial_mps2: Vector3::zeros(),
            last_non_conservative_accel_body_mps2: Vector3::zeros(),
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

    pub fn add_grav_body(&mut self, grav_body: GravBodyData) -> Result<(), GravityError> {
        self.gravity.add_grav_body(grav_body)
    }

    pub fn enable_gravity_gradient(&mut self, earth_mu_m3ps2: f64) {
        self.gravity_gradient_mu_m3ps2 = Some(earth_mu_m3ps2);
    }

    pub fn grav_body_input_mut(
        &mut self,
        planet_name: &str,
    ) -> Result<&mut Input<PlanetStateMsg>, GravityError> {
        self.gravity.planet_body_input_mut(planet_name)
    }

    fn refresh_outputs_without_integration(&mut self, context: &SimulationContext) {
        self.sync_effectors_from_integrated_state();
        let Some(integrated_state) = self.integrated_state.clone() else {
            return;
        };
        let (inertial_position_m, inertial_velocity_mps) =
            self.gravity.inertial_position_and_velocity(
                integrated_state.position_wrt_central_body_m,
                integrated_state.velocity_wrt_central_body_mps,
                context.current_sim_nanos,
            );
        let mass_props = self.mass_props_for_effector_states(&integrated_state.effector_states);
        let body_to_inertial_dcm = body_to_inertial_dcm_from_sigma_bn(integrated_state.sigma_bn);
        let com_position_wrt_central_body_m = integrated_state.position_wrt_central_body_m
            + body_to_inertial_dcm * mass_props.center_of_mass_body_m;
        self.last_gravity_accel_inertial_mps2 = self
            .gravity
            .compute_gravity_field(com_position_wrt_central_body_m, context.current_sim_nanos)
            .unwrap_or_else(|error| panic!("failed to evaluate gravity acceleration: {error}"));
        let state_msg = SpacecraftStateMsg {
            position_m: inertial_position_m,
            velocity_mps: inertial_velocity_mps,
            sigma_bn: integrated_state.sigma_bn,
            omega_radps: integrated_state.omega_radps,
        };
        let diagnostics_msg = self.current_diagnostics_msg(
            &integrated_state,
            &state_msg,
            Vector3::zeros(),
            context.current_sim_nanos,
        );
        self.state_out.write(state_msg.clone());
        self.write_state_effector_outputs(context.current_sim_nanos, &state_msg);
        self.mass_props_out.write(self.current_mass_props_msg());
        self.diagnostics_out.write(diagnostics_msg);
    }

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

    // Called after RK4 completes to publish the final committed state to other modules.
    fn sync_effectors_from_integrated_state(&mut self) {
        let Some(integrated_state) = &self.integrated_state else {
            return;
        };

        let effector_states = integrated_state.effector_states.clone();
        self.sync_state_effectors_from_states(&effector_states);
    }

    // Writes effector_states into each state effector so their output messages reflect
    // the given state. Called at every RK4 substep with the trial state, so dynamic
    // effectors (e.g. SRP) read the correct panel angle/MOI when computing forces.
    fn sync_state_effectors_from_states(&mut self, effector_states: &[Vec<f64>]) {
        assert_eq!(
            self.state_effectors.len(),
            effector_states.len(),
            "state effector count does not match integrated state count"
        );

        for (effector, effector_state) in
            self.state_effectors.iter_mut().zip(effector_states.iter())
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

    fn current_mass_props_msg(&self) -> SpacecraftMassPropsMsg {
        let props = if let Some(integrated_state) = &self.integrated_state {
            self.mass_props_for_effector_states(&integrated_state.effector_states)
        } else {
            self.mass_props_for_effector_states(&[])
        };

        SpacecraftMassPropsMsg {
            mass_kg: props.mass_kg,
            center_of_mass_body_m: props.center_of_mass_body_m,
            inertia_about_point_b_body_kg_m2: props.inertia_about_point_b_body_kg_m2,
        }
    }

    fn current_diagnostics_msg(
        &mut self,
        state: &IntegratedState,
        state_msg: &SpacecraftStateMsg,
        omega_dot_radps2: Vector3<f64>,
        current_sim_nanos: u64,
    ) -> SpacecraftDiagnosticsMsg {
        let mass_props = self.mass_props_for_effector_states(&state.effector_states);
        let body_to_inertial_dcm = body_to_inertial_dcm_from_sigma_bn(state.sigma_bn);
        let inertial_to_body_dcm = body_to_inertial_dcm.transpose();
        let hub_center_of_mass_tilde = tilde(self.config.hub_center_of_mass_body_m);
        let hub_inertia_about_point_b = self.config.inertia_kg_m2
            + self.config.mass_kg * hub_center_of_mass_tilde * hub_center_of_mass_tilde.transpose();

        let mut rotational_angular_momentum_body = hub_inertia_about_point_b * state.omega_radps;
        let mut rotational_energy_j = 0.5
            * state
                .omega_radps
                .dot(&(hub_inertia_about_point_b * state.omega_radps));

        for (effector, effector_state) in self
            .state_effectors
            .iter()
            .zip(state.effector_states.iter())
        {
            rotational_angular_momentum_body +=
                effector.rotational_angular_momentum_body(effector_state, state.omega_radps);
            rotational_energy_j += effector.rotational_energy_j(effector_state, state.omega_radps);
        }

        let center_of_mass_dot_body_mps = mass_props.center_of_mass_prime_body_mps
            + state.omega_radps.cross(&mass_props.center_of_mass_body_m);
        rotational_angular_momentum_body -= mass_props.mass_kg
            * mass_props
                .center_of_mass_body_m
                .cross(&center_of_mass_dot_body_mps);
        rotational_energy_j -=
            0.5 * mass_props.mass_kg * center_of_mass_dot_body_mps.norm_squared();

        let mut drag_force_body_n = Vector3::zeros();
        let mut drag_torque_body_nm = Vector3::zeros();
        let mut srp_force_body_n = Vector3::zeros();
        let mut srp_force_inertial_n = Vector3::zeros();
        let mut srp_torque_body_nm = Vector3::zeros();
        for effector in &self.dynamic_effectors {
            let output = effector.compute_output(state_msg);
            let force_body_n = inertial_to_body_dcm * output.force_inertial_n;
            let name = effector.name().to_ascii_lowercase();
            if name.contains("drag") {
                drag_force_body_n += force_body_n;
                drag_torque_body_nm += output.torque_body_nm;
            } else if name.contains("solar") || name.contains("srp") {
                srp_force_body_n += force_body_n;
                srp_force_inertial_n += output.force_inertial_n;
                srp_torque_body_nm += output.torque_body_nm;
            }
        }

        let com_position_wrt_central_body_m = state.position_wrt_central_body_m
            + body_to_inertial_dcm * mass_props.center_of_mass_body_m;
        let com_velocity_wrt_central_body_mps = state.velocity_wrt_central_body_mps
            + body_to_inertial_dcm * center_of_mass_dot_body_mps;
        let orbital_kinetic_energy_j =
            0.5 * mass_props.mass_kg * com_velocity_wrt_central_body_mps.norm_squared();
        let orbital_potential_energy_j = if self.gravity.has_complete_cache() {
            mass_props.mass_kg
                * self
                    .gravity
                    .specific_potential_jpkg(com_position_wrt_central_body_m, current_sim_nanos)
                    .unwrap_or_else(|error| panic!("failed to evaluate gravity potential: {error}"))
        } else {
            0.0
        };

        SpacecraftDiagnosticsMsg {
            omega_dot_radps2,
            gravity_accel_inertial_mps2: self.last_gravity_accel_inertial_mps2,
            non_conservative_accel_body_mps2: self.last_non_conservative_accel_body_mps2,
            drag_force_body_n,
            drag_torque_body_nm,
            srp_force_body_n,
            srp_force_inertial_n,
            srp_torque_body_nm,
            orbital_kinetic_energy_j,
            orbital_potential_energy_j,
            orbital_energy_j: orbital_kinetic_energy_j + orbital_potential_energy_j,
            rotational_energy_j,
            orbital_angular_momentum_inertial_kg_m2ps: mass_props.mass_kg
                * com_position_wrt_central_body_m.cross(&com_velocity_wrt_central_body_mps),
            rotational_angular_momentum_inertial_kg_m2ps: body_to_inertial_dcm
                * rotational_angular_momentum_body,
        }
    }

    fn mass_props_for_effector_states(&self, effector_states: &[Vec<f64>]) -> SpacecraftMassProps {
        let hub_center_of_mass_tilde = tilde(self.config.hub_center_of_mass_body_m);
        let hub_inertia_about_point_b = self.config.inertia_kg_m2
            + self.config.mass_kg * hub_center_of_mass_tilde * hub_center_of_mass_tilde.transpose();

        assert_eq!(
            self.state_effectors.len(),
            effector_states.len(),
            "state effector count does not match integrated state count"
        );

        let (
            effector_mass_kg,
            effector_mass_dot_kgps,
            effectors_first_moment,
            effectors_first_moment_prime,
            effector_inertia_about_point_b,
            effector_inertia_about_point_b_prime,
        ) = self
            .state_effectors
            .iter()
            .zip(effector_states.iter())
            .fold(
                (
                    0.0,
                    0.0,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Matrix3::zeros(),
                    Matrix3::zeros(),
                ),
                |(
                    mass_sum,
                    mass_dot_sum,
                    first_moment_sum,
                    first_moment_prime_sum,
                    inertia_sum,
                    inertia_prime_sum,
                ),
                 (effector, effector_state)| {
                    assert_eq!(
                        effector.state_len(),
                        effector_state.len(),
                        "state length mismatch for state effector '{}'",
                        effector.name()
                    );
                    let props = effector.mass_properties(effector_state);
                    (
                        mass_sum + props.mass_kg,
                        mass_dot_sum + props.mass_dot_kgps,
                        first_moment_sum + props.mass_kg * props.center_of_mass_body_m,
                        first_moment_prime_sum
                            + props.mass_kg * props.center_of_mass_prime_body_mps,
                        inertia_sum + props.inertia_about_point_b_body_kg_m2,
                        inertia_prime_sum + props.inertia_about_point_b_body_prime_kg_m2ps,
                    )
                },
            );
        let total_mass_kg = self.config.mass_kg + effector_mass_kg;
        let total_mass_dot_kgps = effector_mass_dot_kgps;
        let total_first_moment =
            self.config.mass_kg * self.config.hub_center_of_mass_body_m + effectors_first_moment;

        let center_of_mass_body_m = if total_mass_kg > 0.0 {
            total_first_moment / total_mass_kg
        } else {
            Vector3::zeros()
        };
        let center_of_mass_prime_body_mps = if total_mass_kg > 0.0 {
            effectors_first_moment_prime / total_mass_kg
                - total_mass_dot_kgps * center_of_mass_body_m / total_mass_kg / total_mass_kg
        } else {
            Vector3::zeros()
        };

        SpacecraftMassProps {
            mass_kg: total_mass_kg,
            mass_dot_kgps: total_mass_dot_kgps,
            center_of_mass_body_m,
            center_of_mass_prime_body_mps,
            inertia_about_point_b_body_kg_m2: hub_inertia_about_point_b
                + effector_inertia_about_point_b,
            inertia_about_point_b_body_prime_kg_m2ps: effector_inertia_about_point_b_prime,
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
            .expect("spacecraft integrated state must exist")
            .clone();
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
        let diagnostics_msg =
            self.current_diagnostics_msg(&integrated_state, &state_msg, Vector3::zeros(), 0);
        self.state_out.write(state_msg.clone());
        self.write_state_effector_outputs(0, &state_msg);
        self.mass_props_out.write(self.current_mass_props_msg());
        self.diagnostics_out.write(diagnostics_msg);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.gravity
            .update_cache(context.current_sim_nanos)
            .unwrap_or_else(|error| panic!("failed to refresh gravity inputs: {error}"));

        if context.current_sim_nanos == 0 {
            self.refresh_outputs_without_integration(context);
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
        let old_inertial_velocity_mps = self
            .gravity
            .inertial_position_and_velocity(
                integrated_state.position_wrt_central_body_m,
                integrated_state.velocity_wrt_central_body_mps,
                step_start_nanos,
            )
            .1;

        let integrator = self
            .config
            .integrator
            .clone()
            .unwrap_or(Box::new(Rk4::default()));
        let (new_state, gravity_delta_velocity_inertial_mps) = integrator.propagate(
            self,
            &integrated_state,
            step_start_nanos,
            step_start_epoch,
            dt_seconds,
        );

        let omega_dot_radps2 = (new_state.omega_radps - integrated_state.omega_radps) / dt_seconds;

        self.integrated_state = Some(new_state);

        self.sync_effectors_from_integrated_state();
        let integrated_state = self
            .integrated_state
            .as_ref()
            .expect("spacecraft integrated state must remain initialized")
            .clone();
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
        let body_to_inertial_dcm = body_to_inertial_dcm_from_sigma_bn(integrated_state.sigma_bn);
        self.last_non_conservative_accel_body_mps2 = body_to_inertial_dcm.transpose()
            * ((inertial_velocity_mps
                - old_inertial_velocity_mps
                - gravity_delta_velocity_inertial_mps)
                / dt_seconds);
        let diagnostics_msg = self.current_diagnostics_msg(
            &integrated_state,
            &state_msg,
            omega_dot_radps2,
            context.current_sim_nanos,
        );
        self.state_out.write(state_msg.clone());
        self.write_state_effector_outputs(context.current_sim_nanos, &state_msg);
        self.mass_props_out.write(self.current_mass_props_msg());
        self.diagnostics_out.write(diagnostics_msg);
    }
}

impl DynamicObject for Spacecraft {
    type State = IntegratedState;
    type Derivative = StateDerivative;
    type StepOutput = Vector3<f64>;

    fn state_with_derivative(
        &self,
        state: &IntegratedState,
        derivative: &StateDerivative,
        dt: f64,
    ) -> IntegratedState {
        IntegratedState {
            position_wrt_central_body_m: state.position_wrt_central_body_m
                + derivative.position_wrt_central_body_dot_mps * dt,
            velocity_wrt_central_body_mps: state.velocity_wrt_central_body_mps
                + derivative.velocity_wrt_central_body_dot_mps2 * dt,
            sigma_bn: state.sigma_bn + derivative.sigma_dot * dt,
            omega_radps: state.omega_radps + derivative.omega_dot_radps2 * dt,
            effector_states: state
                .effector_states
                .iter()
                .zip(derivative.effector_state_dots.iter())
                .map(|(effector_state, effector_state_dot)| {
                    effector_state
                        .iter()
                        .zip(effector_state_dot.iter())
                        .map(|(value, value_dot)| value + value_dot * dt)
                        .collect()
                })
                .collect(),
        }
    }

    fn equations_of_motion(
        &mut self,
        state: &IntegratedState,
        current_sim_nanos: u64,
        _current_epoch: Epoch,
    ) -> StateDerivative {
        // Sync outputs (e.g. panel angle) so dynamic effectors read the correct trial state.
        self.sync_state_effectors_from_states(&state.effector_states);
        let mass_props = self.mass_props_for_effector_states(&state.effector_states);
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
        let omega_cross_omega_cross_center = state
            .omega_radps
            .cross(&state.omega_radps.cross(&mass_props.center_of_mass_body_m));
        let omega_cross_center_prime = state
            .omega_radps
            .cross(&mass_props.center_of_mass_prime_body_mps);
        let mut back_sub = BackSubMatrices {
            matrix_a: mass_props.mass_kg * Matrix3::identity(),
            matrix_b: -mass_props.mass_kg * center_of_mass_tilde,
            matrix_c: mass_props.mass_kg * center_of_mass_tilde,
            matrix_d: inertia,
            vec_trans: -2.0 * mass_props.mass_kg * omega_cross_center_prime
                - mass_props.mass_kg * omega_cross_omega_cross_center
                - 2.0
                    * mass_props.mass_dot_kgps
                    * (mass_props.center_of_mass_prime_body_mps
                        + state.omega_radps.cross(&mass_props.center_of_mass_body_m)),
            vec_rot: -mass_props.inertia_about_point_b_body_prime_kg_m2ps * state.omega_radps,
        };
        let com_position_wrt_central_body_m = state.position_wrt_central_body_m
            + body_to_inertial_dcm * mass_props.center_of_mass_body_m;
        let gravity_accel = self
            .gravity
            .compute_gravity_field(com_position_wrt_central_body_m, current_sim_nanos)
            .unwrap_or_else(|error| panic!("failed to evaluate gravity acceleration: {error}"));
        self.last_gravity_accel_inertial_mps2 = gravity_accel;
        let gravity_body_mps2 = inertial_to_body_dcm * gravity_accel;

        for effector in &self.dynamic_effectors {
            let output = effector.compute_output(&state_msg);
            back_sub.vec_trans += inertial_to_body_dcm * output.force_inertial_n;
            back_sub.vec_rot += output.torque_body_nm;
        }
        if let Some(mu_m3ps2) = self.gravity_gradient_mu_m3ps2 {
            back_sub.vec_rot += gravity_gradient_torque_body(
                state.position_wrt_central_body_m,
                body_to_inertial_dcm,
                mass_props.mass_kg,
                mass_props.center_of_mass_body_m,
                mass_props.inertia_about_point_b_body_kg_m2,
                mu_m3ps2,
            );
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
        back_sub.vec_rot += center_of_mass_tilde * gravity_force_body;
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
            gravity_accel_inertial_mps2: gravity_accel,
            sigma_dot: mrp_kinematics(state.sigma_bn, state.omega_radps),
            omega_dot_radps2: omega_dot,
            effector_state_dots,
        }
    }

    /// Weighted RMS error norm across every integrated scalar (translation,
    /// attitude, body rate, and each state-effector coordinate). Each component
    /// is scaled by `atol + rtol * max(|high|, |low|)`, so the mixed units of
    /// position, velocity, MRP and rate are compared on a relative footing.
    fn error_norm(
        &self,
        high: &IntegratedState,
        low: &IntegratedState,
        atol: f64,
        rtol: f64,
    ) -> Option<f64> {
        let mut sum_of_squares = 0.0;
        let mut count = 0usize;
        let mut accumulate = |high_component: f64, low_component: f64| {
            let scale = atol + rtol * high_component.abs().max(low_component.abs());
            let scaled_error = (high_component - low_component) / scale;
            sum_of_squares += scaled_error * scaled_error;
            count += 1;
        };

        for axis in 0..3 {
            accumulate(
                high.position_wrt_central_body_m[axis],
                low.position_wrt_central_body_m[axis],
            );
            accumulate(
                high.velocity_wrt_central_body_mps[axis],
                low.velocity_wrt_central_body_mps[axis],
            );
            accumulate(high.sigma_bn[axis], low.sigma_bn[axis]);
            accumulate(high.omega_radps[axis], low.omega_radps[axis]);
        }
        for (high_effector, low_effector) in
            high.effector_states.iter().zip(low.effector_states.iter())
        {
            for (high_component, low_component) in high_effector.iter().zip(low_effector.iter()) {
                accumulate(*high_component, *low_component);
            }
        }

        if count == 0 {
            return Some(0.0);
        }
        Some((sum_of_squares / count as f64).sqrt())
    }

    /// The step output is the gravity-induced velocity change over the step, an
    /// accumulated quantity, so adaptive sub-steps sum.
    fn merge_step_outputs(&self, accumulated: Vector3<f64>, next: Vector3<f64>) -> Vector3<f64> {
        accumulated + next
    }

    /// Performs Butcher Normalization
    fn combine(
        &self,
        state: &Self::State,
        weighted: Vec<(Self::Derivative, f64)>,
        step: f64,
    ) -> (Self::State, Self::StepOutput) {
        let combined_position_dot =
            weighted
                .iter()
                .fold(Vector3::<f64>::zeros(), |acc, (derivative, weight)| {
                    acc + (derivative.position_wrt_central_body_dot_mps * *weight)
                });
        let combined_velocity_dot =
            weighted
                .iter()
                .fold(Vector3::<f64>::zeros(), |acc, (derivative, weight)| {
                    acc + (derivative.velocity_wrt_central_body_dot_mps2 * *weight)
                });
        let combined_omega_dot = weighted
            .iter()
            .fold(Vector3::<f64>::zeros(), |acc, (derivative, weight)| {
                acc + (derivative.omega_dot_radps2 * *weight)
            });
        let combined_gravity_accel = weighted
            .iter()
            .fold(Vector3::<f64>::zeros(), |acc, (derivative, weight)| {
                acc + (derivative.gravity_accel_inertial_mps2 * *weight)
            });
        let combined_sigma_dot = weighted
            .iter()
            .fold(Vector3::<f64>::zeros(), |acc, (derivative, weight)| {
                acc + (derivative.sigma_dot * *weight)
            });
        let combined_effector_state_dots_input = weighted
            .into_iter()
            .map(|(derivative, weight)| (derivative.effector_state_dots, weight))
            .collect::<Vec<_>>();

        let combined_effector_state_dots = combine_effector_state_derivatives(
            combined_effector_state_dots_input,
            &state.effector_states,
        );

        let combined_state = IntegratedState {
            position_wrt_central_body_m: state.position_wrt_central_body_m
                + combined_position_dot * step,
            velocity_wrt_central_body_mps: state.velocity_wrt_central_body_mps
                + combined_velocity_dot * step,
            sigma_bn: shadow_mrp(state.sigma_bn + combined_sigma_dot * step),
            omega_radps: state.omega_radps + combined_omega_dot * step,
            effector_states: state
                .effector_states
                .iter()
                .zip(combined_effector_state_dots.iter())
                .map(|(effector_state, effector_state_dot)| {
                    effector_state
                        .iter()
                        .zip(effector_state_dot.iter())
                        .map(|(value, value_dot)| value + value_dot * step)
                        .collect()
                })
                .collect(),
        };
        let step_output = combined_gravity_accel * step;

        (combined_state, step_output)
    }
}

#[cfg(test)]
mod tests;
