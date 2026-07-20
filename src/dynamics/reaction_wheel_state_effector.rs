use nalgebra::{Matrix3, Vector3};
use std::any::Any;

use crate::messages::{
    ArrayMotorTorqueMsg, Input, MAX_EFF_COUNT, Output, ReactionWheelStateMsg, RwSpeedMsg,
};
use crate::spacecraft::{BackSubMatrices, EffectorOutput, StateEffector};

#[derive(Clone, Debug, Default)]
pub struct ReactionWheelBackSubContribution {
    pub matrix_d_correction_kg_m2: Matrix3<f64>,
    pub force_body_n: Vector3<f64>,
    pub torque_body_nm: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReactionWheelModel {
    BalancedWheels,
    JitterSimple,
    JitterFullyCoupled,
}

#[derive(Clone, Debug)]
pub struct ReactionWheelStateEffectorConfig {
    pub name: String,
    pub position_m: Vector3<f64>,
    pub spin_axis_body: Vector3<f64>,
    pub torque_axis_body: Vector3<f64>,
    pub gimbal_axis_body: Vector3<f64>,
    pub max_torque_nm: f64,
    pub min_torque_nm: f64,
    pub max_momentum_nms: f64,
    pub max_speed_radps: f64,
    pub max_power_w: f64,
    pub js_kg_m2: f64,
    pub jt_kg_m2: f64,
    pub jg_kg_m2: f64,
    pub mass_kg: f64,
    pub static_imbalance_kg_m: f64,
    pub dynamic_imbalance_kg_m2: f64,
    pub com_offset_m: f64,
    pub j13_kg_m2: f64,
    pub coulomb_friction_nm: f64,
    pub static_friction_nm: f64,
    pub beta_static: f64,
    pub viscous_friction_nms_per_rad: f64,
    pub omega_limit_cycle_radps: f64,
    pub jitter_phase_delay_sec: f64,
    pub model: ReactionWheelModel,
    pub initial_omega_radps: f64,
}

impl ReactionWheelStateEffectorConfig {
    pub fn balanced(
        name: impl Into<String>,
        position_m: Vector3<f64>,
        spin_axis_body: Vector3<f64>,
        max_torque_nm: f64,
        max_momentum_nms: f64,
    ) -> Self {
        Self {
            name: name.into(),
            position_m,
            spin_axis_body,
            torque_axis_body: orthogonal_unit_vector(spin_axis_body),
            gimbal_axis_body: orthogonal_unit_vector_2(spin_axis_body),
            max_torque_nm,
            min_torque_nm: 0.0,
            max_momentum_nms,
            max_speed_radps: if max_momentum_nms > 0.0 {
                max_momentum_nms
            } else {
                -1.0
            },
            max_power_w: -1.0,
            js_kg_m2: 1.0,
            jt_kg_m2: 1.0,
            jg_kg_m2: 1.0,
            mass_kg: 1.0,
            static_imbalance_kg_m: 0.0,
            dynamic_imbalance_kg_m2: 0.0,
            com_offset_m: 0.0,
            j13_kg_m2: 0.0,
            coulomb_friction_nm: 0.0,
            static_friction_nm: 0.0,
            beta_static: -1.0,
            viscous_friction_nms_per_rad: 0.0,
            omega_limit_cycle_radps: 1.0e-4,
            jitter_phase_delay_sec: 0.0,
            model: ReactionWheelModel::BalancedWheels,
            initial_omega_radps: 0.0,
        }
    }

    pub fn jitter_simple(
        name: impl Into<String>,
        position_m: Vector3<f64>,
        spin_axis_body: Vector3<f64>,
        max_torque_nm: f64,
        max_momentum_nms: f64,
        mass_kg: f64,
        static_imbalance_kg_m: f64,
        dynamic_imbalance_kg_m2: f64,
    ) -> Self {
        let mut config = Self::balanced(
            name,
            position_m,
            spin_axis_body,
            max_torque_nm,
            max_momentum_nms,
        );
        config.model = ReactionWheelModel::JitterSimple;
        config.mass_kg = mass_kg;
        config.static_imbalance_kg_m = static_imbalance_kg_m;
        config.dynamic_imbalance_kg_m2 = dynamic_imbalance_kg_m2;
        config
    }
}

#[derive(Clone, Debug)]
pub struct ReactionWheel {
    pub config: ReactionWheelStateEffectorConfig,
    pub state_out: Output<ReactionWheelStateMsg>,
    pub omega_radps: f64,
    pub theta_rad: f64,
    pub u_current_nm: f64,
    pub friction_torque_nm: f64,
    omega_before_radps: f64,
    friction_stribeck: bool,
    w2_hat_b: Vector3<f64>,
    w3_hat_b: Vector3<f64>,
}

impl ReactionWheel {
    pub fn new(config: ReactionWheelStateEffectorConfig) -> Self {
        let spin_axis = normalize_or_zero(config.spin_axis_body);
        let torque_axis = normalize_or_fallback(
            config.torque_axis_body,
            normalize_or_zero(orthogonal_unit_vector(config.spin_axis_body)),
        );
        let gimbal_axis =
            normalize_or_fallback(config.gimbal_axis_body, spin_axis.cross(&torque_axis));
        Self {
            omega_radps: config.initial_omega_radps,
            theta_rad: 0.0,
            u_current_nm: 0.0,
            friction_torque_nm: 0.0,
            omega_before_radps: config.initial_omega_radps,
            friction_stribeck: false,
            w2_hat_b: torque_axis,
            w3_hat_b: gimbal_axis,
            config,
            state_out: Output::default(),
        }
    }

    pub fn pre_integration(&mut self, dt_seconds: f64, requested_torque_nm: f64) {
        self.configure_rw_request(requested_torque_nm);
        self.update_friction_torque();
        self.omega_before_radps = self.omega_radps;
        if matches!(self.config.model, ReactionWheelModel::JitterFullyCoupled) && dt_seconds > 0.0 {
            self.theta_rad += self.omega_radps * dt_seconds;
            self.update_jitter_axes();
        }
    }

    pub fn compute_output(&self, _state: &crate::messages::SpacecraftStateMsg) -> EffectorOutput {
        let spin_axis = normalize_or_zero(self.config.spin_axis_body);
        let motor_and_friction_torque = -(self.u_current_nm + self.friction_torque_nm) * spin_axis;

        if matches!(
            self.config.model,
            ReactionWheelModel::JitterSimple | ReactionWheelModel::JitterFullyCoupled
        ) {
            let static_force_body = self.config.static_imbalance_kg_m
                * self.omega_radps
                * self.omega_radps
                * self.w2_hat_b;
            let imbalance_torque_body = self.config.position_m.cross(&static_force_body)
                + self.config.dynamic_imbalance_kg_m2
                    * self.omega_radps
                    * self.omega_radps
                    * self.w2_hat_b;

            EffectorOutput {
                force_inertial_n: static_force_body,
                torque_body_nm: motor_and_friction_torque + imbalance_torque_body,
            }
        } else {
            EffectorOutput {
                force_inertial_n: Vector3::zeros(),
                torque_body_nm: motor_and_friction_torque,
            }
        }
    }

    pub fn set_omega_radps(&mut self, omega_radps: f64) {
        self.omega_radps = omega_radps;
        if matches!(
            self.config.model,
            ReactionWheelModel::JitterSimple | ReactionWheelModel::JitterFullyCoupled
        ) {
            self.update_jitter_axes();
        }
    }

    pub fn back_sub_contribution(
        &self,
        wheel_omega_radps: f64,
        wheel_theta_rad: f64,
        body_omega_radps: Vector3<f64>,
    ) -> Option<ReactionWheelBackSubContribution> {
        if !matches!(
            self.config.model,
            ReactionWheelModel::BalancedWheels | ReactionWheelModel::JitterSimple
        ) {
            return None;
        }

        let spin_axis = normalize_or_zero(self.config.spin_axis_body);
        let matrix_d_correction_kg_m2 = -self.config.js_kg_m2 * (spin_axis * spin_axis.transpose());
        let mut torque_body_nm = -spin_axis * (self.u_current_nm + self.friction_torque_nm)
            - self.config.js_kg_m2 * wheel_omega_radps * body_omega_radps.cross(&spin_axis);
        let mut force_body_n = Vector3::zeros();

        if matches!(self.config.model, ReactionWheelModel::JitterSimple) {
            let (w2_hat_b, _) = self.jitter_axes_for_theta(wheel_theta_rad);
            force_body_n = self.config.static_imbalance_kg_m * wheel_omega_radps.powi(2) * w2_hat_b;
            torque_body_nm += self.config.position_m.cross(&force_body_n)
                + self.config.dynamic_imbalance_kg_m2 * wheel_omega_radps.powi(2) * w2_hat_b;
        }

        Some(ReactionWheelBackSubContribution {
            matrix_d_correction_kg_m2,
            torque_body_nm,
            force_body_n,
        })
    }

    pub fn omega_dot_radps2(&self, body_omega_dot_radps2: Vector3<f64>) -> Option<f64> {
        if !matches!(
            self.config.model,
            ReactionWheelModel::BalancedWheels | ReactionWheelModel::JitterSimple
        ) {
            return None;
        }

        let spin_axis = normalize_or_zero(self.config.spin_axis_body);
        let js = self.config.js_kg_m2;
        if js <= 0.0 {
            return Some(0.0);
        }

        Some(
            (self.u_current_nm + self.friction_torque_nm) / js
                - spin_axis.dot(&body_omega_dot_radps2),
        )
    }

    fn configure_rw_request(&mut self, requested_torque_nm: f64) {
        let mut requested_torque = requested_torque_nm;

        if self.config.max_torque_nm >= 0.0 {
            requested_torque =
                requested_torque.clamp(-self.config.max_torque_nm, self.config.max_torque_nm);
        }

        if self.config.min_torque_nm > 0.0 && requested_torque.abs() < self.config.min_torque_nm {
            requested_torque = 0.0;
        }

        let max_speed = if self.config.max_speed_radps >= 0.0 {
            self.config.max_speed_radps
        } else if self.config.max_momentum_nms > 0.0 && self.config.js_kg_m2 > 0.0 {
            self.config.max_momentum_nms / self.config.js_kg_m2
        } else {
            -1.0
        };
        if max_speed >= 0.0
            && self.omega_radps.abs() >= max_speed
            && requested_torque.signum() == self.omega_radps.signum()
        {
            requested_torque = 0.0;
        }

        if self.config.max_power_w >= 0.0 && self.omega_radps.abs() > 0.0 {
            let power_limited_torque = self.config.max_power_w / self.omega_radps.abs();
            requested_torque = requested_torque.clamp(-power_limited_torque, power_limited_torque);
        }

        self.u_current_nm = requested_torque;
    }

    fn update_friction_torque(&mut self) {
        if self.omega_radps.abs() < 0.10 * self.config.omega_limit_cycle_radps
            && self.config.beta_static > 0.0
        {
            self.friction_stribeck = true;
        }

        let sign_of_omega = self.omega_radps.signum();
        let omega_dot = self.omega_radps - self.omega_before_radps;
        let sign_of_omega_dot = omega_dot.signum();
        if self.friction_stribeck
            && (sign_of_omega - sign_of_omega_dot).abs() < 2.0
            && self.config.beta_static > 0.0
        {
            self.friction_stribeck = true;
        } else {
            self.friction_stribeck = false;
        }

        let mut friction_force = if self.friction_stribeck && self.config.beta_static > 0.0 {
            let omega_ratio = self.omega_radps / self.config.beta_static;
            (2.0 * std::f64::consts::E).sqrt()
                * (self.config.static_friction_nm - self.config.coulomb_friction_nm)
                * (-(omega_ratio * omega_ratio) / 2.0).exp()
                * self.omega_radps
                / (self.config.beta_static * 2.0_f64.sqrt())
                + self.config.coulomb_friction_nm
                    * (self.omega_radps * 10.0 / self.config.beta_static).tanh()
                + self.config.viscous_friction_nms_per_rad * self.omega_radps
        } else {
            sign_of_omega * self.config.coulomb_friction_nm
                + self.config.viscous_friction_nms_per_rad * self.omega_radps
        };

        let friction_force_at_limit_cycle = if self.friction_stribeck
            && self.config.beta_static > 0.0
        {
            let omega_ratio = self.config.omega_limit_cycle_radps / self.config.beta_static;
            (2.0 * std::f64::consts::E).sqrt()
                * (self.config.static_friction_nm - self.config.coulomb_friction_nm)
                * (-(omega_ratio * omega_ratio) / 2.0).exp()
                * self.config.omega_limit_cycle_radps
                / (self.config.beta_static * 2.0_f64.sqrt())
                + self.config.coulomb_friction_nm
                    * (self.config.omega_limit_cycle_radps * 10.0 / self.config.beta_static).tanh()
                + self.config.viscous_friction_nms_per_rad * self.config.omega_limit_cycle_radps
        } else {
            self.config.coulomb_friction_nm
                + self.config.viscous_friction_nms_per_rad * self.config.omega_limit_cycle_radps
        };

        if self.omega_radps.abs() < self.config.omega_limit_cycle_radps {
            friction_force = friction_force_at_limit_cycle / self.config.omega_limit_cycle_radps
                * self.omega_radps;
        }

        self.friction_torque_nm = -friction_force;
    }

    fn update_jitter_axes(&mut self) {
        let (w2_hat_b, w3_hat_b) = self.jitter_axes_for_theta(self.theta_rad);
        self.w2_hat_b = w2_hat_b;
        self.w3_hat_b = w3_hat_b;
    }

    fn jitter_axes_for_theta(&self, theta_rad: f64) -> (Vector3<f64>, Vector3<f64>) {
        let spin_axis = normalize_or_zero(self.config.spin_axis_body);
        let phase_rad = theta_rad - self.omega_radps * self.config.jitter_phase_delay_sec;
        let rotation = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(spin_axis),
            phase_rad,
        );
        let w2_hat_b = rotation.transform_vector(&normalize_or_fallback(
            self.config.torque_axis_body,
            normalize_or_zero(orthogonal_unit_vector(self.config.spin_axis_body)),
        ));
        let w3_hat_b = rotation.transform_vector(&normalize_or_fallback(
            self.config.gimbal_axis_body,
            normalize_or_zero(orthogonal_unit_vector_2(self.config.spin_axis_body)),
        ));
        (w2_hat_b, w3_hat_b)
    }
}

/// Aggregate reaction-wheel state effector. Wheel order defines the mapping
/// from `rw_motor_cmd_in_msg.motor_torque_nm[i]` to `wheels[i]`, matching
/// Basilisk's `ReactionWheelStateEffector::addReactionWheel` convention.
#[derive(Clone, Debug)]
pub struct ReactionWheelStateEffector {
    pub name: String,
    pub rw_motor_cmd_in_msg: Input<ArrayMotorTorqueMsg>,
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
    wheels: Vec<ReactionWheel>,
}

impl ReactionWheelStateEffector {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            rw_motor_cmd_in_msg: Input::default(),
            rw_speed_out_msg: Output::default(),
            wheels: Vec::new(),
        }
    }

    pub fn add_reaction_wheel(&mut self, config: ReactionWheelStateEffectorConfig) {
        assert!(
            self.wheels.len() < MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} reaction wheels are supported"
        );
        self.wheels.push(ReactionWheel::new(config));
    }

    /// Reaction wheels in the same order used by the aggregate messages.
    pub fn wheels(&self) -> &[ReactionWheel] {
        &self.wheels
    }

    fn jitter_wheel_count(&self) -> usize {
        self.wheels
            .iter()
            .filter(|wheel| is_jitter_model(wheel.config.model))
            .count()
    }

    fn theta_state_index(&self, wheel_index: usize) -> Option<usize> {
        is_jitter_model(self.wheels[wheel_index].config.model).then(|| {
            self.wheels.len()
                + self.wheels[..wheel_index]
                    .iter()
                    .filter(|wheel| is_jitter_model(wheel.config.model))
                    .count()
        })
    }

    fn state_for_wheel(&self, state: &[f64], wheel_index: usize) -> (f64, f64) {
        let omega_radps = state[wheel_index];
        let theta_rad = self
            .theta_state_index(wheel_index)
            .map_or(0.0, |index| state[index]);
        (omega_radps, theta_rad)
    }

    fn assert_state_length(&self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "reaction wheel state length mismatch"
        );
    }
}

impl StateEffector for ReactionWheelStateEffector {
    fn name(&self) -> &str {
        &self.name
    }

    fn state_len(&self) -> usize {
        self.wheels.len() + self.jitter_wheel_count()
    }

    fn initial_state(&self) -> Vec<f64> {
        let mut state = Vec::with_capacity(self.state_len());
        for wheel in &self.wheels {
            assert!(
                matches!(
                    wheel.config.model,
                    ReactionWheelModel::BalancedWheels | ReactionWheelModel::JitterSimple
                ),
                "spacecraft dynamics currently only support balanced or JitterSimple reaction wheels with back substitution"
            );
            state.push(wheel.omega_radps);
        }
        for wheel in &self.wheels {
            if is_jitter_model(wheel.config.model) {
                state.push(wheel.theta_rad);
            }
        }
        state
    }

    fn load_state(&mut self, state: &[f64]) {
        self.assert_state_length(state);
        let wheel_count = self.wheels.len();
        let mut theta_index = wheel_count;
        for (wheel_index, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.omega_radps = state[wheel_index];
            if is_jitter_model(wheel.config.model) {
                wheel.theta_rad = state[theta_index];
                theta_index += 1;
                wheel.update_jitter_axes();
            } else {
                wheel.theta_rad = 0.0;
            }
        }
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, dt_seconds: f64) {
        let command = self.rw_motor_cmd_in_msg.read();
        for (index, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.pre_integration(dt_seconds, command.motor_torque_nm[index]);
        }
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        _gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.assert_state_length(effector_state);
        for (wheel_index, wheel) in self.wheels.iter().enumerate() {
            let (omega_radps, theta_rad) = self.state_for_wheel(effector_state, wheel_index);
            let contribution = wheel
                .back_sub_contribution(omega_radps, theta_rad, body_omega_radps)
                .expect("unsupported reaction wheel model for back substitution");
            back_sub.matrix_d += contribution.matrix_d_correction_kg_m2;
            back_sub.vec_trans += contribution.force_body_n;
            back_sub.vec_rot += contribution.torque_body_nm;
        }
    }

    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        _body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.assert_state_length(effector_state);
        let mut derivatives = vec![0.0; self.state_len()];
        for (wheel_index, wheel) in self.wheels.iter().enumerate() {
            let omega_dot_radps2 = wheel
                .omega_dot_radps2(body_omega_dot_radps2)
                .expect("unsupported reaction wheel model for back substitution");
            derivatives[wheel_index] = omega_dot_radps2;
            if let Some(theta_index) = self.theta_state_index(wheel_index) {
                derivatives[theta_index] = effector_state[wheel_index];
            }
        }
        derivatives
    }

    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        _body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.assert_state_length(effector_state);
        self.wheels
            .iter()
            .enumerate()
            .fold(Vector3::zeros(), |total, (wheel_index, wheel)| {
                let spin_axis = normalize_or_zero(wheel.config.spin_axis_body);
                total + spin_axis * (wheel.config.js_kg_m2 * effector_state[wheel_index])
            })
    }

    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.assert_state_length(effector_state);
        self.wheels
            .iter()
            .enumerate()
            .map(|(wheel_index, wheel)| {
                let spin_axis = normalize_or_zero(wheel.config.spin_axis_body);
                let wheel_omega = effector_state[wheel_index];
                0.5 * wheel.config.js_kg_m2 * wheel_omega * wheel_omega
                    + wheel.config.js_kg_m2 * wheel_omega * spin_axis.dot(&body_omega_radps)
            })
            .sum()
    }

    fn write_outputs(
        &mut self,
        _current_sim_nanos: u64,
        _hub_state: &crate::messages::SpacecraftStateMsg,
    ) {
        let mut speed_message = RwSpeedMsg::default();
        for (index, wheel) in self.wheels.iter_mut().enumerate() {
            wheel.state_out.write(ReactionWheelStateMsg {
                omega_radps: wheel.omega_radps,
                theta_rad: wheel.theta_rad,
            });
            speed_message.wheel_speeds_radps[index] = wheel.omega_radps;
            speed_message.wheel_angles_rad[index] = wheel.theta_rad;
        }
        self.rw_speed_out_msg.write(speed_message);
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn is_jitter_model(model: ReactionWheelModel) -> bool {
    matches!(
        model,
        ReactionWheelModel::JitterSimple | ReactionWheelModel::JitterFullyCoupled
    )
}

fn normalize_or_zero(vector: Vector3<f64>) -> Vector3<f64> {
    if vector.norm_squared() > 0.0 {
        vector.normalize()
    } else {
        Vector3::zeros()
    }
}

fn normalize_or_fallback(vector: Vector3<f64>, fallback: Vector3<f64>) -> Vector3<f64> {
    let candidate = vector;
    if candidate.norm_squared() > 0.0 {
        candidate.normalize()
    } else if fallback.norm_squared() > 0.0 {
        fallback.normalize()
    } else {
        Vector3::zeros()
    }
}

fn orthogonal_unit_vector(spin_axis_body: Vector3<f64>) -> Vector3<f64> {
    let spin_axis = normalize_or_zero(spin_axis_body);
    let candidate = if spin_axis.x.abs() < 0.9 {
        Vector3::new(1.0, 0.0, 0.0)
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    };
    let orthogonal = spin_axis.cross(&candidate);
    if orthogonal.norm_squared() > 0.0 {
        orthogonal.normalize()
    } else {
        Vector3::new(0.0, 1.0, 0.0)
    }
}

fn orthogonal_unit_vector_2(spin_axis_body: Vector3<f64>) -> Vector3<f64> {
    let spin_axis = normalize_or_zero(spin_axis_body);
    let torque_axis = normalize_or_zero(orthogonal_unit_vector(spin_axis_body));
    let gimbal = spin_axis.cross(&torque_axis);
    if gimbal.norm_squared() > 0.0 {
        gimbal.normalize()
    } else {
        Vector3::new(0.0, 0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{ArrayMotorTorqueMsg, Output, SpacecraftStateMsg};
    use crate::spacecraft::{BackSubMatrices, StateEffector};

    use super::{ReactionWheelStateEffector, ReactionWheelStateEffectorConfig};

    fn rw_with_command(
        max_torque_nm: f64,
        min_torque_nm: f64,
        max_momentum_nms: f64,
        max_power_w: f64,
        initial_omega_radps: f64,
        command_nm: f64,
    ) -> ReactionWheelStateEffector {
        let mut config = ReactionWheelStateEffectorConfig::balanced(
            "rw",
            Vector3::zeros(),
            Vector3::new(1.0, 0.0, 0.0),
            max_torque_nm,
            max_momentum_nms,
        );
        config.min_torque_nm = min_torque_nm;
        config.max_power_w = max_power_w;
        config.initial_omega_radps = initial_omega_radps;
        let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
        reaction_wheels.add_reaction_wheel(config);
        let command = Output::new(ArrayMotorTorqueMsg::from_active(&[command_nm]));
        reaction_wheels.rw_motor_cmd_in_msg.connect(command.slot());
        reaction_wheels
    }

    /// Commands [-1.2, 1.5, 2.5] Nm with limits [1, 2, 2] Nm → clamped to [-1.0, 1.5, 2.0].
    #[test]
    fn torque_saturation() {
        let cases = [(-1.2, 1.0, -1.0), (1.5, 2.0, 1.5), (2.5, 2.0, 2.0)];
        for (cmd, limit, expected) in cases {
            let mut rw = rw_with_command(limit, 0.0, 0.0, -1.0, 0.0, cmd);
            StateEffector::pre_integration(&mut rw, 0, 1.0);
            assert!(
                (rw.wheels()[0].u_current_nm - expected).abs() < 1e-10,
                "cmd={cmd} limit={limit}: expected u={expected}, got {}",
                rw.wheels()[0].u_current_nm
            );
        }
    }

    /// cmd=-0.09 with min=0.1 → zeroed; cmd=0.0001 with min=0.0 → passed through.
    #[test]
    fn minimum_torque_threshold() {
        let mut rw0 = rw_with_command(10.0, 0.1, 0.0, -1.0, 0.0, -0.09);
        StateEffector::pre_integration(&mut rw0, 0, 1.0);
        assert!(
            rw0.wheels()[0].u_current_nm.abs() < 1e-10,
            "expected zero (below min), got {}",
            rw0.wheels()[0].u_current_nm
        );

        let mut rw1 = rw_with_command(10.0, 0.0, 0.0, -1.0, 0.0, 0.0001);
        StateEffector::pre_integration(&mut rw1, 0, 1.0);
        assert!(
            (rw1.wheels()[0].u_current_nm - 0.0001).abs() < 1e-10,
            "expected 0.0001, got {}",
            rw1.wheels()[0].u_current_nm
        );
    }

    #[test]
    fn jitter_simple_load_state_updates_imbalance_axes_with_theta() {
        let mut config = ReactionWheelStateEffectorConfig::jitter_simple(
            "rw",
            Vector3::zeros(),
            Vector3::z(),
            0.05,
            1.0,
            1.0,
            1.0e-6,
            1.0e-8,
        );
        config.torque_axis_body = Vector3::x();
        config.gimbal_axis_body = Vector3::y();
        let mut rw = ReactionWheelStateEffector::new("reaction_wheels");
        rw.add_reaction_wheel(config);

        rw.load_state(&[10.0, std::f64::consts::FRAC_PI_2]);

        assert!((rw.wheels()[0].w2_hat_b - Vector3::y()).norm() < 1.0e-12);
    }

    #[test]
    fn jitter_simple_phase_delay_lags_axes_by_omega_dt() {
        let mut config = ReactionWheelStateEffectorConfig::jitter_simple(
            "rw",
            Vector3::zeros(),
            Vector3::z(),
            0.05,
            1.0,
            1.0,
            1.0e-6,
            1.0e-8,
        );
        config.torque_axis_body = Vector3::x();
        config.gimbal_axis_body = Vector3::y();
        config.jitter_phase_delay_sec = std::f64::consts::FRAC_PI_2 / 10.0;
        let mut rw = ReactionWheelStateEffector::new("reaction_wheels");
        rw.add_reaction_wheel(config);

        rw.load_state(&[10.0, std::f64::consts::FRAC_PI_2]);

        assert!((rw.wheels()[0].w2_hat_b - Vector3::x()).norm() < 1.0e-12);
    }

    /// omega=[49, 51, -52] rad/s, limit=50, commands all 1.5 Nm → [1.5, 0.0, 1.5].
    /// Wheel at 51 rad/s is zeroed (same sign as torque); wheel at -52 is not (opposite sign).
    #[test]
    fn speed_saturation() {
        let limit = 50.0; // max_momentum_nms=50 → max_speed_radps=50 (js=1 kg·m²)
        let cases = [(49.0, 1.5), (51.0, 0.0), (-52.0, 1.5)];
        for (omega, expected) in cases {
            let mut rw = rw_with_command(10.0, 0.0, limit, -1.0, omega, 1.5);
            StateEffector::pre_integration(&mut rw, 0, 1.0);
            assert!(
                (rw.wheels()[0].u_current_nm - expected).abs() < 1e-10,
                "omega={omega}: expected u={expected}, got {}",
                rw.wheels()[0].u_current_nm
            );
        }
    }

    /// P_max=1 W, omega=50 rad/s → torque limit = P/|omega| = 0.02 Nm.
    /// Commands [0.01, -0.04, 0.04] → [0.01, -0.02, 0.02].
    #[test]
    fn power_saturation() {
        let cases = [(0.01, 0.01), (-0.04, -0.02), (0.04, 0.02)];
        for (cmd, expected) in cases {
            let mut rw = rw_with_command(10.0, 0.0, 0.0, 1.0, 50.0, cmd);
            StateEffector::pre_integration(&mut rw, 0, 1.0);
            assert!(
                (rw.wheels()[0].u_current_nm - expected).abs() < 1e-10,
                "cmd={cmd}: expected u={expected}, got {}",
                rw.wheels()[0].u_current_nm
            );
        }
    }

    #[test]
    fn aggregate_command_and_speed_messages_follow_wheel_order() {
        let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
        reaction_wheels.add_reaction_wheel(ReactionWheelStateEffectorConfig::balanced(
            "rw_x",
            Vector3::zeros(),
            Vector3::x(),
            1.0,
            10.0,
        ));
        reaction_wheels.add_reaction_wheel(ReactionWheelStateEffectorConfig::jitter_simple(
            "rw_y",
            Vector3::zeros(),
            Vector3::y(),
            1.0,
            10.0,
            1.0,
            0.0,
            0.0,
        ));
        let command = Output::new(ArrayMotorTorqueMsg::from_active(&[0.25, -0.5]));
        reaction_wheels.rw_motor_cmd_in_msg.connect(command.slot());

        StateEffector::pre_integration(&mut reaction_wheels, 0, 0.1);
        assert_eq!(reaction_wheels.wheels()[0].u_current_nm, 0.25);
        assert_eq!(reaction_wheels.wheels()[1].u_current_nm, -0.5);

        reaction_wheels.load_state(&[12.0, -8.0, 0.2]);
        reaction_wheels.write_outputs(0, &SpacecraftStateMsg::default());
        let speed = reaction_wheels.rw_speed_out_msg.read();
        assert_eq!(speed.wheel_speeds_radps[..2], [12.0, -8.0]);
        assert_eq!(speed.wheel_angles_rad[..2], [0.0, 0.2]);
        assert!(
            speed.wheel_speeds_radps[2..]
                .iter()
                .all(|value| *value == 0.0)
        );
    }

    #[test]
    fn mixed_wheels_share_one_state_and_sum_dynamics_contributions() {
        let mut balanced = ReactionWheelStateEffectorConfig::balanced(
            "rw_x",
            Vector3::zeros(),
            Vector3::x(),
            1.0,
            10.0,
        );
        balanced.js_kg_m2 = 2.0;
        balanced.initial_omega_radps = 4.0;

        let mut jitter = ReactionWheelStateEffectorConfig::jitter_simple(
            "rw_y",
            Vector3::zeros(),
            Vector3::y(),
            1.0,
            10.0,
            1.0,
            0.0,
            0.0,
        );
        jitter.js_kg_m2 = 3.0;
        jitter.initial_omega_radps = 5.0;

        let mut reaction_wheels = ReactionWheelStateEffector::new("reaction_wheels");
        reaction_wheels.add_reaction_wheel(balanced);
        reaction_wheels.add_reaction_wheel(jitter);

        // Basilisk stores every wheel speed first, followed only by the
        // jitter-wheel angles: [omega_x, omega_y, theta_y].
        assert_eq!(reaction_wheels.state_len(), 3);
        assert_eq!(reaction_wheels.initial_state(), vec![4.0, 5.0, 0.0]);
        let state = [4.0, 5.0, 0.2];
        reaction_wheels.load_state(&state);

        let mut back_sub = BackSubMatrices::default();
        reaction_wheels.update_contributions(
            &state,
            Vector3::zeros(),
            Vector3::zeros(),
            &mut back_sub,
        );
        let expected_matrix_d = Matrix3::from_diagonal(&Vector3::new(-2.0, -3.0, 0.0));
        assert!((back_sub.matrix_d - expected_matrix_d).norm() < 1.0e-12);
        assert_eq!(back_sub.vec_trans, Vector3::zeros());
        assert_eq!(back_sub.vec_rot, Vector3::zeros());

        assert_eq!(
            reaction_wheels.compute_derivatives(
                &state,
                Vector3::zeros(),
                Vector3::new(1.0, 2.0, 3.0),
            ),
            vec![-1.0, -2.0, 5.0]
        );
        assert_eq!(
            reaction_wheels.rotational_angular_momentum_body(&state, Vector3::zeros()),
            Vector3::new(8.0, 15.0, 0.0)
        );
        assert_eq!(
            reaction_wheels.rotational_energy_j(&state, Vector3::zeros()),
            53.5
        );
    }
}
