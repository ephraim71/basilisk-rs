use crate::drag::Drag;
use crate::gravity::GravityEffector;
use crate::mtb::Mtb;
use crate::reaction_wheel::ReactionWheel;
use crate::srp::SolarRadiationPressure;
use crate::thruster::Thruster;
use hifitime::{Duration, Epoch};
use nalgebra::{
    Matrix3, Quaternion as RawQuaternion, UnitQuaternion, Vector3,
};

use crate::messages::{Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::{Mat3, Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct SpacecraftConfig {
    pub mass_kg: f64,
    pub inertia_kg_m2: Mat3,
    pub initial_position_m: Vector3<f64>,
    pub initial_velocity_mps: Vector3<f64>,
    pub initial_attitude_b_to_i: UnitQuaternion<f64>,
    pub initial_omega_radps: Vector3<f64>,
}

#[derive(Clone, Debug, Default)]
pub struct EffectorOutput {
    pub force_inertial_n: Vector3<f64>,
    pub torque_body_nm: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub enum SpacecraftEffector {
    Drag(Drag),
    Mtb(Mtb),
    ReactionWheel(ReactionWheel),
    SolarRadiationPressure(SolarRadiationPressure),
    Thruster(Thruster),
}

impl SpacecraftEffector {
    pub fn name(&self) -> &str {
        match self {
            Self::Drag(drag) => &drag.config.name,
            Self::Mtb(mtb) => &mtb.config.name,
            Self::ReactionWheel(reaction_wheel) => &reaction_wheel.config.name,
            Self::SolarRadiationPressure(srp) => &srp.config.name,
            Self::Thruster(thruster) => &thruster.config.name,
        }
    }

    pub fn prepare_for_step(&mut self, current_sim_nanos: u64, dt_seconds: f64) {
        match self {
            Self::Drag(_) => {}
            Self::Mtb(_) => {}
            Self::ReactionWheel(reaction_wheel) => reaction_wheel.prepare_for_step(dt_seconds),
            Self::SolarRadiationPressure(_) => {}
            Self::Thruster(thruster) => thruster.prepare_for_step(current_sim_nanos),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        match self {
            Self::Drag(drag) => drag.compute_output(state),
            Self::Mtb(mtb) => mtb.compute_output(state),
            Self::ReactionWheel(reaction_wheel) => reaction_wheel.compute_output(state),
            Self::SolarRadiationPressure(srp) => srp.compute_output(state),
            Self::Thruster(thruster) => thruster.compute_output(state),
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct Spacecraft {
    pub config: SpacecraftConfig,
    pub state_out: Output<SpacecraftStateMsg>,
    pub grav_field: GravityEffector,
    pub effectors: Vec<SpacecraftEffector>,
    start_epoch: Option<Epoch>,
    previous_update_nanos: Option<u64>,
    integrated_state: Option<IntegratedState>,
}

impl Spacecraft {
    pub fn new(config: SpacecraftConfig) -> Self {
        Self {
            state_out: Output::default(),
            config,
            grav_field: GravityEffector::new(),
            effectors: Vec::new(),
            start_epoch: None,
            previous_update_nanos: None,
            integrated_state: None,
        }
    }

    pub fn add_effector(&mut self, effector: SpacecraftEffector) {
        self.effectors.push(effector);
    }

    pub fn add_grav_body(&mut self, grav_body: crate::gravity::GravBodyData) {
        self.grav_field.add_grav_body(grav_body);
    }

    pub fn grav_body_input_mut(&mut self, planet_name: &str) -> Option<&mut Input<PlanetStateMsg>> {
        self.grav_field.planet_body_input_mut(planet_name)
    }

    pub fn seed_initial_state(&mut self) {
        self.integrated_state = Some(IntegratedState {
            relative_position_m: self.config.initial_position_m,
            relative_velocity_mps: self.config.initial_velocity_mps,
            attitude_b_to_i: self.config.initial_attitude_b_to_i,
            omega_radps: self.config.initial_omega_radps,
        });

        let (position_m, velocity_mps) = self.grav_field.inertial_position_and_velocity(
            self.integrated_state
                .as_ref()
                .expect("spacecraft integrated state must exist")
                .relative_position_m,
            self.integrated_state
                .as_ref()
                .expect("spacecraft integrated state must exist")
                .relative_velocity_mps,
            0,
        );

        self.state_out.write(SpacecraftStateMsg {
            position_m,
            velocity_mps,
            attitude_b_to_i: self.config.initial_attitude_b_to_i,
            omega_radps: self.config.initial_omega_radps,
        });
    }
}

impl Module for Spacecraft {
    fn init(&mut self) {
        self.seed_initial_state();
        self.start_epoch = None;
        self.previous_update_nanos = None;
    }

    fn update(&mut self, context: &SimulationContext) {
        self.start_epoch.get_or_insert(context.start_epoch);
        let Some(previous_update_nanos) = self.previous_update_nanos else {
            self.previous_update_nanos = Some(context.current_sim_nanos);
            return;
        };

        let dt_seconds = (context.current_sim_nanos - previous_update_nanos) as f64 * 1.0e-9;
        self.previous_update_nanos = Some(context.current_sim_nanos);

        if dt_seconds == 0.0 {
            return;
        }

        for effector in &mut self.effectors {
            effector.prepare_for_step(previous_update_nanos, dt_seconds);
        }

        let integrated_state = self
            .integrated_state
            .clone()
            .expect("spacecraft integrated state must be initialized");
        let new_state = self.propagate_rk4(&integrated_state, previous_update_nanos, dt_seconds);
        let (inertial_position_m, inertial_velocity_mps) =
            self.grav_field.inertial_position_and_velocity(
                new_state.relative_position_m,
                new_state.relative_velocity_mps,
                context.current_sim_nanos,
            );
        self.integrated_state = Some(new_state.clone());
        self.state_out.write(SpacecraftStateMsg {
            position_m: inertial_position_m,
            velocity_mps: inertial_velocity_mps,
            attitude_b_to_i: new_state.attitude_b_to_i,
            omega_radps: new_state.omega_radps,
        });
    }
}

#[derive(Clone, Debug)]
struct IntegratedState {
    relative_position_m: Vector3<f64>,
    relative_velocity_mps: Vector3<f64>,
    attitude_b_to_i: UnitQuaternion<f64>,
    omega_radps: Vector3<f64>,
}

#[derive(Clone, Debug)]
struct StateDerivative {
    relative_position_dot_mps: Vector3<f64>,
    relative_velocity_dot_mps2: Vector3<f64>,
    attitude_dot: RawQuaternion<f64>,
    omega_dot_radps2: Vector3<f64>,
}

impl Spacecraft {
    fn propagate_rk4(
        &self,
        state: &IntegratedState,
        current_sim_nanos: u64,
        dt_seconds: f64,
    ) -> IntegratedState {
        let k1 = self.equations_of_motion(state, current_sim_nanos);
        let k2 = self.equations_of_motion(
            &self.state_with_derivative(state, &k1, dt_seconds * 0.5),
            current_sim_nanos + (dt_seconds * 0.5 * 1.0e9) as u64,
        );
        let k3 = self.equations_of_motion(
            &self.state_with_derivative(state, &k2, dt_seconds * 0.5),
            current_sim_nanos + (dt_seconds * 0.5 * 1.0e9) as u64,
        );
        let k4 = self.equations_of_motion(
            &self.state_with_derivative(state, &k3, dt_seconds),
            current_sim_nanos + (dt_seconds * 1.0e9) as u64,
        );

        let combined_position_dot = k1.relative_position_dot_mps
            + 2.0 * k2.relative_position_dot_mps
            + 2.0 * k3.relative_position_dot_mps
            + k4.relative_position_dot_mps;
        let combined_velocity_dot = k1.relative_velocity_dot_mps2
            + 2.0 * k2.relative_velocity_dot_mps2
            + 2.0 * k3.relative_velocity_dot_mps2
            + k4.relative_velocity_dot_mps2;
        let combined_omega_dot = k1.omega_dot_radps2
            + 2.0 * k2.omega_dot_radps2
            + 2.0 * k3.omega_dot_radps2
            + k4.omega_dot_radps2;
        let combined_attitude_dot = RawQuaternion::new(
            k1.attitude_dot.w
                + 2.0 * k2.attitude_dot.w
                + 2.0 * k3.attitude_dot.w
                + k4.attitude_dot.w,
            k1.attitude_dot.i
                + 2.0 * k2.attitude_dot.i
                + 2.0 * k3.attitude_dot.i
                + k4.attitude_dot.i,
            k1.attitude_dot.j
                + 2.0 * k2.attitude_dot.j
                + 2.0 * k3.attitude_dot.j
                + k4.attitude_dot.j,
            k1.attitude_dot.k
                + 2.0 * k2.attitude_dot.k
                + 2.0 * k3.attitude_dot.k
                + k4.attitude_dot.k,
        );

        let updated_quaternion =
            state.attitude_b_to_i.quaternion() + combined_attitude_dot * (dt_seconds / 6.0);

        IntegratedState {
            relative_position_m: state.relative_position_m
                + combined_position_dot * (dt_seconds / 6.0),
            relative_velocity_mps: state.relative_velocity_mps
                + combined_velocity_dot * (dt_seconds / 6.0),
            attitude_b_to_i: UnitQuaternion::new_normalize(updated_quaternion),
            omega_radps: state.omega_radps + combined_omega_dot * (dt_seconds / 6.0),
        }
    }

    fn state_with_derivative(
        &self,
        state: &IntegratedState,
        derivative: &StateDerivative,
        dt_seconds: f64,
    ) -> IntegratedState {
        let attitude = UnitQuaternion::new_normalize(
            state.attitude_b_to_i.quaternion() + derivative.attitude_dot * dt_seconds,
        );

        IntegratedState {
            relative_position_m: state.relative_position_m
                + derivative.relative_position_dot_mps * dt_seconds,
            relative_velocity_mps: state.relative_velocity_mps
                + derivative.relative_velocity_dot_mps2 * dt_seconds,
            attitude_b_to_i: attitude,
            omega_radps: state.omega_radps + derivative.omega_dot_radps2 * dt_seconds,
        }
    }

    fn equations_of_motion(
        &self,
        state: &IntegratedState,
        current_sim_nanos: u64,
    ) -> StateDerivative {
        let inertia = Matrix3::from_row_slice(&[
            self.config.inertia_kg_m2[0][0],
            self.config.inertia_kg_m2[0][1],
            self.config.inertia_kg_m2[0][2],
            self.config.inertia_kg_m2[1][0],
            self.config.inertia_kg_m2[1][1],
            self.config.inertia_kg_m2[1][2],
            self.config.inertia_kg_m2[2][0],
            self.config.inertia_kg_m2[2][1],
            self.config.inertia_kg_m2[2][2],
        ]);
        let inertia_inverse = inertia
            .try_inverse()
            .expect("spacecraft inertia matrix must be invertible");
        let (inertial_position_m, inertial_velocity_mps) =
            self.grav_field.inertial_position_and_velocity(
                state.relative_position_m,
                state.relative_velocity_mps,
                current_sim_nanos,
            );
        let state_msg = spacecraft_state_msg_from_integrated_state(
            state,
            inertial_position_m,
            inertial_velocity_mps,
        );
        let effector_outputs: Vec<_> = self
            .effectors
            .iter()
            .map(|effector| effector.compute_output(&state_msg))
            .collect();
        let total_force_inertial = effector_outputs
            .iter()
            .fold(Vector3::zeros(), |sum, output| {
                sum + output.force_inertial_n
            });
        let total_torque_body = effector_outputs
            .iter()
            .fold(Vector3::zeros(), |sum, output| sum + output.torque_body_nm);
        let angular_momentum = inertia * state.omega_radps;
        let omega_dot =
            inertia_inverse * (total_torque_body - state.omega_radps.cross(&angular_momentum));
        let gravity_epoch = self
            .start_epoch
            .expect("spacecraft start epoch must be initialized")
            + Duration::from_total_nanoseconds(current_sim_nanos as i128);
        let gravity_accel = self.grav_field.compute_gravity_field(
            state.relative_position_m,
            gravity_epoch,
            current_sim_nanos,
        );
        let translational_accel = gravity_accel + total_force_inertial / self.config.mass_kg;
        let omega_quaternion = RawQuaternion::new(
            0.0,
            state.omega_radps.x,
            state.omega_radps.y,
            state.omega_radps.z,
        );
        let attitude_dot = state.attitude_b_to_i.quaternion() * omega_quaternion * 0.5;

        StateDerivative {
            relative_position_dot_mps: state.relative_velocity_mps,
            relative_velocity_dot_mps2: translational_accel,
            attitude_dot,
            omega_dot_radps2: omega_dot,
        }
    }
}

fn spacecraft_state_msg_from_integrated_state(
    state: &IntegratedState,
    inertial_position_m: Vector3<f64>,
    inertial_velocity_mps: Vector3<f64>,
) -> SpacecraftStateMsg {
    SpacecraftStateMsg {
        position_m: inertial_position_m,
        velocity_mps: inertial_velocity_mps,
        attitude_b_to_i: state.attitude_b_to_i,
        omega_radps: state.omega_radps,
    }
}

impl Default for SpacecraftConfig {
    fn default() -> Self {
        Self {
            mass_kg: 0.0,
            inertia_kg_m2: [[0.0; 3]; 3],
            initial_position_m: Vector3::zeros(),
            initial_velocity_mps: Vector3::zeros(),
            initial_attitude_b_to_i: UnitQuaternion::identity(),
            initial_omega_radps: Vector3::zeros(),
        }
    }
}
