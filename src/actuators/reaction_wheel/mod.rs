use nalgebra::{Matrix3, Vector3};
use std::any::Any;

use crate::messages::{Input, ReactionWheelCommandMsg};
use crate::spacecraft::{BackSubMatrices, EffectorOutput, StateEffector};

#[derive(Clone, Debug, Default)]
pub struct ReactionWheelBackSubContribution {
    pub matrix_d_correction_kg_m2: Matrix3<f64>,
    pub torque_body_nm: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReactionWheelModel {
    BalancedWheels,
    JitterSimple,
    JitterFullyCoupled,
}

#[derive(Clone, Debug)]
pub struct ReactionWheelConfig {
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
    pub model: ReactionWheelModel,
    pub initial_omega_radps: f64,
}

impl ReactionWheelConfig {
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
    pub config: ReactionWheelConfig,
    pub command_in: Input<ReactionWheelCommandMsg>,
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
    pub fn new(config: ReactionWheelConfig) -> Self {
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
            command_in: Input::default(),
        }
    }

    pub fn pre_integration(&mut self, dt_seconds: f64) {
        self.configure_rw_request();
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
        body_omega_radps: Vector3<f64>,
    ) -> Option<ReactionWheelBackSubContribution> {
        if !matches!(self.config.model, ReactionWheelModel::BalancedWheels) {
            return None;
        }

        let spin_axis = normalize_or_zero(self.config.spin_axis_body);
        let matrix_d_correction_kg_m2 = -self.config.js_kg_m2 * (spin_axis * spin_axis.transpose());
        let torque_body_nm = -spin_axis * (self.u_current_nm + self.friction_torque_nm)
            - self.config.js_kg_m2 * wheel_omega_radps * body_omega_radps.cross(&spin_axis);

        Some(ReactionWheelBackSubContribution {
            matrix_d_correction_kg_m2,
            torque_body_nm,
        })
    }

    pub fn omega_dot_radps2(&self, body_omega_dot_radps2: Vector3<f64>) -> Option<f64> {
        if !matches!(self.config.model, ReactionWheelModel::BalancedWheels) {
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

    fn configure_rw_request(&mut self) {
        let mut requested_torque = self.command_in.read().motor_torque_nm;

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
        let spin_axis = normalize_or_zero(self.config.spin_axis_body);
        let rotation = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Unit::new_normalize(spin_axis),
            self.theta_rad,
        );
        self.w2_hat_b = rotation.transform_vector(&normalize_or_fallback(
            self.config.torque_axis_body,
            normalize_or_zero(orthogonal_unit_vector(self.config.spin_axis_body)),
        ));
        self.w3_hat_b = rotation.transform_vector(&normalize_or_fallback(
            self.config.gimbal_axis_body,
            normalize_or_zero(orthogonal_unit_vector_2(self.config.spin_axis_body)),
        ));
    }
}

impl StateEffector for ReactionWheel {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        2
    }

    fn initial_state(&self) -> Vec<f64> {
        assert!(
            matches!(self.config.model, ReactionWheelModel::BalancedWheels),
            "spacecraft dynamics currently only support balanced reaction wheels with back substitution"
        );
        vec![self.omega_radps, self.theta_rad]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "reaction wheel state length mismatch"
        );
        self.set_omega_radps(state[0]);
        self.theta_rad = state[1];
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, dt_seconds: f64) {
        ReactionWheel::pre_integration(self, dt_seconds);
    }

    fn update_contributions(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "reaction wheel state length mismatch"
        );
        let contribution = self
            .back_sub_contribution(effector_state[0], body_omega_radps)
            .expect(
                "spacecraft dynamics currently only support balanced reaction wheels with back substitution",
            );
        back_sub.matrix_d += contribution.matrix_d_correction_kg_m2;
        back_sub.vec_rot += contribution.torque_body_nm;
    }

    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "reaction wheel state length mismatch"
        );
        let omega_dot_radps2 = self
            .omega_dot_radps2(body_omega_dot_radps2)
            .expect(
                "spacecraft dynamics currently only support balanced reaction wheels with back substitution",
            );
        vec![omega_dot_radps2, effector_state[0]]
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
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
    use nalgebra::Vector3;

    use crate::messages::{Output, ReactionWheelCommandMsg};

    use super::{ReactionWheel, ReactionWheelConfig};

    fn rw_with_command(
        max_torque_nm: f64,
        min_torque_nm: f64,
        max_momentum_nms: f64,
        max_power_w: f64,
        initial_omega_radps: f64,
        command_nm: f64,
    ) -> ReactionWheel {
        let mut config = ReactionWheelConfig::balanced(
            "rw",
            Vector3::zeros(),
            Vector3::new(1.0, 0.0, 0.0),
            max_torque_nm,
            max_momentum_nms,
        );
        config.min_torque_nm = min_torque_nm;
        config.max_power_w = max_power_w;
        config.initial_omega_radps = initial_omega_radps;
        let mut rw = ReactionWheel::new(config);
        let cmd = Output::new(ReactionWheelCommandMsg {
            motor_torque_nm: command_nm,
        });
        rw.command_in.connect(cmd.slot());
        rw
    }

    /// Commands [-1.2, 1.5, 2.5] Nm with limits [1, 2, 2] Nm → clamped to [-1.0, 1.5, 2.0].
    #[test]
    fn torque_saturation() {
        let cases = [(-1.2, 1.0, -1.0), (1.5, 2.0, 1.5), (2.5, 2.0, 2.0)];
        for (cmd, limit, expected) in cases {
            let mut rw = rw_with_command(limit, 0.0, 0.0, -1.0, 0.0, cmd);
            rw.pre_integration(1.0);
            assert!(
                (rw.u_current_nm - expected).abs() < 1e-10,
                "cmd={cmd} limit={limit}: expected u={expected}, got {}",
                rw.u_current_nm
            );
        }
    }

    /// cmd=-0.09 with min=0.1 → zeroed; cmd=0.0001 with min=0.0 → passed through.
    #[test]
    fn minimum_torque_threshold() {
        let mut rw0 = rw_with_command(10.0, 0.1, 0.0, -1.0, 0.0, -0.09);
        rw0.pre_integration(1.0);
        assert!(
            rw0.u_current_nm.abs() < 1e-10,
            "expected zero (below min), got {}",
            rw0.u_current_nm
        );

        let mut rw1 = rw_with_command(10.0, 0.0, 0.0, -1.0, 0.0, 0.0001);
        rw1.pre_integration(1.0);
        assert!(
            (rw1.u_current_nm - 0.0001).abs() < 1e-10,
            "expected 0.0001, got {}",
            rw1.u_current_nm
        );
    }

    /// omega=[49, 51, -52] rad/s, limit=50, commands all 1.5 Nm → [1.5, 0.0, 1.5].
    /// Wheel at 51 rad/s is zeroed (same sign as torque); wheel at -52 is not (opposite sign).
    #[test]
    fn speed_saturation() {
        let limit = 50.0; // max_momentum_nms=50 → max_speed_radps=50 (js=1 kg·m²)
        let cases = [(49.0, 1.5), (51.0, 0.0), (-52.0, 1.5)];
        for (omega, expected) in cases {
            let mut rw = rw_with_command(10.0, 0.0, limit, -1.0, omega, 1.5);
            rw.pre_integration(1.0);
            assert!(
                (rw.u_current_nm - expected).abs() < 1e-10,
                "omega={omega}: expected u={expected}, got {}",
                rw.u_current_nm
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
            rw.pre_integration(1.0);
            assert!(
                (rw.u_current_nm - expected).abs() < 1e-10,
                "cmd={cmd}: expected u={expected}, got {}",
                rw.u_current_nm
            );
        }
    }
}
