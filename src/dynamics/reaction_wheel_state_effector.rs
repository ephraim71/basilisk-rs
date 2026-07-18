use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::cell::RefCell;

use crate::messages::{
    ArrayMotorTorqueMsg, Input, MAX_EFF_COUNT, Output, ReactionWheelStateMsg, RwSpeedMsg,
};
use crate::spacecraft::{BackSubMatrices, EffectorOutput, StateEffector, StateEffectorMassProps};

/// Wheel-acceleration clamp applied in `compute_derivatives` as a numerical
/// safety net against unlimited torque on a small-inertia hub.
const MAX_WHEEL_ACCELERATION_RADPS2: f64 = 1.0e6;

#[derive(Clone, Debug, Default)]
pub struct ReactionWheelBackSubContribution {
    pub matrix_d_correction_kg_m2: Matrix3<f64>,
    pub force_body_n: Vector3<f64>,
    pub torque_body_nm: Vector3<f64>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ReactionWheelModel {
    /// Ideal balanced wheel: only the spin-axis reaction torque couples to the hub.
    BalancedWheels,
    /// Balanced back-substitution plus static/dynamic imbalance modelled as an
    /// external force and torque (does not add wheel mass/inertia to the hub).
    JitterSimple,
    /// Wheel mass, imbalance center-of-mass offset, and inertia are fully coupled
    /// into the hub equations of motion through the imbalance parameters.
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
    /// Transverse and gimbal-axis wheel inertias. Only read by `JitterFullyCoupled`.
    pub jt_kg_m2: f64,
    pub jg_kg_m2: f64,
    /// Wheel mass. Only read by `JitterFullyCoupled`.
    pub mass_kg: f64,
    pub static_imbalance_kg_m: f64,
    pub dynamic_imbalance_kg_m2: f64,
    /// Wheel center-of-mass offset along the imbalance axis. Only read by
    /// `JitterFullyCoupled`.
    pub com_offset_m: f64,
    /// Off-diagonal (spin/gimbal) wheel inertia term. Only read by
    /// `JitterFullyCoupled`.
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

    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable. Fields that use a negative
    /// value to disable a limit (max torque/momentum/speed/power, `beta_static`)
    /// are only checked for finiteness.
    pub fn validate(&self) -> Result<(), String> {
        for (label, vector) in [
            ("position_m", self.position_m),
            ("spin_axis_body", self.spin_axis_body),
            ("torque_axis_body", self.torque_axis_body),
            ("gimbal_axis_body", self.gimbal_axis_body),
        ] {
            if !vector.iter().all(|v| v.is_finite()) {
                return Err(format!("{label} must be finite, got {vector:?}"));
            }
        }
        if self.spin_axis_body.norm_squared() == 0.0 {
            return Err("spin_axis_body must be non-zero".to_string());
        }
        for (label, value) in [
            ("max_torque_nm", self.max_torque_nm),
            ("max_momentum_nms", self.max_momentum_nms),
            ("max_speed_radps", self.max_speed_radps),
            ("max_power_w", self.max_power_w),
            ("beta_static", self.beta_static),
            ("initial_omega_radps", self.initial_omega_radps),
            ("jitter_phase_delay_sec", self.jitter_phase_delay_sec),
            ("com_offset_m", self.com_offset_m),
        ] {
            if !value.is_finite() {
                return Err(format!("{label} must be finite, got {value}"));
            }
        }
        for (label, value) in [
            ("min_torque_nm", self.min_torque_nm),
            ("jt_kg_m2", self.jt_kg_m2),
            ("jg_kg_m2", self.jg_kg_m2),
            ("mass_kg", self.mass_kg),
            ("static_imbalance_kg_m", self.static_imbalance_kg_m),
            ("dynamic_imbalance_kg_m2", self.dynamic_imbalance_kg_m2),
            ("j13_kg_m2", self.j13_kg_m2),
            ("coulomb_friction_nm", self.coulomb_friction_nm),
            ("static_friction_nm", self.static_friction_nm),
            ("viscous_friction_nms_per_rad", self.viscous_friction_nms_per_rad),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(format!("{label} must be finite and >= 0, got {value}"));
            }
        }
        if !self.js_kg_m2.is_finite() || self.js_kg_m2 <= 0.0 {
            return Err(format!(
                "js_kg_m2 (spin inertia) must be finite and > 0, got {}",
                self.js_kg_m2
            ));
        }
        // The fully-coupled model needs a positive mass to build the wheel
        // inertia-about-hub and center-of-mass coupling terms.
        if matches!(self.model, ReactionWheelModel::JitterFullyCoupled) && self.mass_kg <= 0.0 {
            return Err(format!(
                "JitterFullyCoupled requires mass_kg > 0, got {}",
                self.mass_kg
            ));
        }
        // omega_limit_cycle_radps divides the friction limit-cycle term.
        if !self.omega_limit_cycle_radps.is_finite() || self.omega_limit_cycle_radps <= 0.0 {
            return Err(format!(
                "omega_limit_cycle_radps must be finite and > 0, got {}",
                self.omega_limit_cycle_radps
            ));
        }
        Ok(())
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

    /// Fully-coupled jitter wheel: the wheel mass, imbalance center-of-mass
    /// offset (`com_offset_m`), and inertia tensor (`jt`/`jg`/`j13`) are coupled
    /// into the hub equations of motion.
    #[allow(clippy::too_many_arguments)]
    pub fn jitter_fully_coupled(
        name: impl Into<String>,
        position_m: Vector3<f64>,
        spin_axis_body: Vector3<f64>,
        max_torque_nm: f64,
        max_momentum_nms: f64,
        mass_kg: f64,
        jt_kg_m2: f64,
        jg_kg_m2: f64,
        com_offset_m: f64,
        j13_kg_m2: f64,
    ) -> Self {
        let mut config = Self::balanced(
            name,
            position_m,
            spin_axis_body,
            max_torque_nm,
            max_momentum_nms,
        );
        config.model = ReactionWheelModel::JitterFullyCoupled;
        config.mass_kg = mass_kg;
        config.jt_kg_m2 = jt_kg_m2;
        config.jg_kg_m2 = jg_kg_m2;
        config.com_offset_m = com_offset_m;
        config.j13_kg_m2 = j13_kg_m2;
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
    /// Fully-coupled wheel-acceleration back-substitution vectors, cached in
    /// `update_contributions` (which sees the body rate) and consumed in
    /// `compute_derivatives`. Unused by the balanced and JitterSimple models.
    fully_coupled_terms: RefCell<FullyCoupledOmegaTerms>,
}

impl ReactionWheel {
    pub fn new(config: ReactionWheelStateEffectorConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid ReactionWheelStateEffectorConfig: {msg}");
        }
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
            fully_coupled_terms: RefCell::new(FullyCoupledOmegaTerms::default()),
            config,
            state_out: Output::default(),
        }
    }

    pub fn pre_integration(&mut self, dt_seconds: f64, requested_torque_nm: f64) {
        self.configure_rw_request(requested_torque_nm);
        self.update_friction_torque();
        self.omega_before_radps = self.omega_radps;
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

        if self.config.max_torque_nm > 0.0 {
            requested_torque =
                requested_torque.clamp(-self.config.max_torque_nm, self.config.max_torque_nm);
        }

        if self.config.min_torque_nm > 0.0 && requested_torque.abs() < self.config.min_torque_nm {
            requested_torque = 0.0;
        }

        let max_speed = if self.config.max_speed_radps > 0.0 {
            self.config.max_speed_radps
        } else if self.config.max_momentum_nms > 0.0 && self.config.js_kg_m2 > 0.0 {
            self.config.max_momentum_nms / self.config.js_kg_m2
        } else {
            -1.0
        };
        if max_speed > 0.0
            && self.omega_radps.abs() >= max_speed
            && self.omega_radps * requested_torque >= 0.0
        {
            requested_torque = 0.0;
        }

        if self.config.max_power_w > 0.0 && self.omega_radps.abs() > 0.0 {
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

        let sign_of_omega = sign(self.omega_radps);
        let omega_dot = self.omega_radps - self.omega_before_radps;
        let sign_of_omega_dot = sign(omega_dot);
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

    /// Imbalance axes `(w2_hat_b, w3_hat_b)` at the given wheel angle, phase-delayed
    /// by `jitter_phase_delay_sec * omega`. `w2_hat_b` is the direction the static
    /// and dynamic imbalance force and torque act along; `w3_hat_b` is the third
    /// (gimbal) wheel-frame axis used by the fully-coupled model.
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

    /// Wheel-frame kinematics for the fully-coupled model at a given wheel angle
    /// and speed: the wheel inertia tensor, center-of-mass position, and their
    /// body-frame time derivatives. `com_offset_m` is the imbalance center-of-mass
    /// offset along `w2_hat` and `j13_kg_m2` is the off-diagonal wheel inertia.
    fn fully_coupled_kinematics(
        &self,
        theta_rad: f64,
        omega_radps: f64,
    ) -> FullyCoupledKinematics {
        let gs_hat = normalize_or_zero(self.config.spin_axis_body);
        let (w2_hat, w3_hat) = self.jitter_axes_for_theta(theta_rad);

        let mass = self.config.mass_kg;
        let d = self.config.com_offset_m;
        let j13 = self.config.j13_kg_m2;

        // dcm_BW has the wheel frame axes as its columns.
        let dcm_body_wheel = Matrix3::from_columns(&[gs_hat, w2_hat, w3_hat]);
        let inertia_wheel = Matrix3::new(
            self.config.js_kg_m2, 0.0, j13,
            0.0, self.config.jt_kg_m2, 0.0,
            j13, 0.0, self.config.jg_kg_m2,
        );
        let inertia_body = dcm_body_wheel * inertia_wheel * dcm_body_wheel.transpose();
        let inertia_prime_wheel = Matrix3::new(
            0.0, -j13, 0.0,
            -j13, 0.0, 0.0,
            0.0, 0.0, 0.0,
        ) * omega_radps;
        let inertia_prime_body =
            dcm_body_wheel * inertia_prime_wheel * dcm_body_wheel.transpose();

        let com_position_body = self.config.position_m + d * w2_hat;
        let com_prime_body = d * omega_radps * w3_hat;

        FullyCoupledKinematics {
            gs_hat,
            w2_hat,
            w3_hat,
            d,
            mass,
            inertia_body,
            inertia_prime_body,
            com_position_body,
            com_prime_body,
        }
    }

    /// Fully-coupled back-substitution contribution plus the wheel-acceleration
    /// vectors (`a_omega`, `b_omega`, `c_omega`) that give
    /// `Omega_dot = a_omega . r_ddot + b_omega . omega_dot + c_omega`.
    fn fully_coupled_back_sub(
        &self,
        omega_radps: f64,
        theta_rad: f64,
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> (FullyCoupledBackSub, FullyCoupledOmegaTerms) {
        let k = self.fully_coupled_kinematics(theta_rad, omega_radps);
        let mass = k.mass;
        let d = k.d;
        let mass_d = mass * d;
        let j13 = self.config.j13_kg_m2;
        let denom = self.config.js_kg_m2 + mass * d * d;

        let omega_s = k.gs_hat.dot(&body_omega_radps);
        let omega_w2 = k.w2_hat.dot(&body_omega_radps);
        let omega_w3 = k.w3_hat.dot(&body_omega_radps);

        let gravity_torque_about_wheel = d * k.w2_hat.cross(&(mass * gravity_body_mps2));

        let a_omega = -mass_d / denom * k.w3_hat;
        let b_omega = -(denom * k.gs_hat
            + j13 * k.w3_hat
            + mass_d * self.config.position_m.cross(&k.w3_hat))
            / denom;
        let c_omega = (omega_w2 * omega_w3 * (-mass * d * d)
            - j13 * omega_w2 * omega_s
            - mass_d
                * k.w3_hat.dot(
                    &body_omega_radps
                        .cross(&body_omega_radps.cross(&self.config.position_m)),
                )
            + (self.u_current_nm + self.friction_torque_nm)
            + k.gs_hat.dot(&gravity_torque_about_wheel))
            / denom;

        let inertia_gs_plus =
            k.inertia_body * k.gs_hat + mass_d * k.com_position_body.cross(&k.w3_hat);
        let omega_squared = omega_radps * omega_radps;

        let terms = FullyCoupledBackSub {
            matrix_a: mass_d * k.w3_hat * a_omega.transpose(),
            matrix_b: mass_d * k.w3_hat * b_omega.transpose(),
            matrix_c: inertia_gs_plus * a_omega.transpose(),
            matrix_d: inertia_gs_plus * b_omega.transpose(),
            vec_trans: mass_d * (omega_squared * k.w2_hat - c_omega * k.w3_hat),
            vec_rot: mass_d * omega_squared * k.com_position_body.cross(&k.w2_hat)
                - k.inertia_prime_body * omega_radps * k.gs_hat
                - body_omega_radps.cross(
                    &(k.inertia_body * omega_radps * k.gs_hat
                        + mass * k.com_position_body.cross(&k.com_prime_body)),
                )
                - inertia_gs_plus * c_omega,
        };

        (
            terms,
            FullyCoupledOmegaTerms {
                a_omega,
                b_omega,
                c_omega,
            },
        )
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
                    ReactionWheelModel::BalancedWheels
                        | ReactionWheelModel::JitterSimple
                        | ReactionWheelModel::JitterFullyCoupled
                ),
                "unsupported reaction wheel model"
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
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        self.assert_state_length(effector_state);
        for (wheel_index, wheel) in self.wheels.iter().enumerate() {
            let (omega_radps, theta_rad) = self.state_for_wheel(effector_state, wheel_index);
            if matches!(wheel.config.model, ReactionWheelModel::JitterFullyCoupled) {
                let (terms, omega_terms) = wheel.fully_coupled_back_sub(
                    omega_radps,
                    theta_rad,
                    body_omega_radps,
                    gravity_body_mps2,
                );
                back_sub.matrix_a += terms.matrix_a;
                back_sub.matrix_b += terms.matrix_b;
                back_sub.matrix_c += terms.matrix_c;
                back_sub.matrix_d += terms.matrix_d;
                back_sub.vec_trans += terms.vec_trans;
                back_sub.vec_rot += terms.vec_rot;
                wheel.fully_coupled_terms.replace(omega_terms);
            } else {
                let contribution = wheel
                    .back_sub_contribution(omega_radps, theta_rad, body_omega_radps)
                    .expect("unsupported reaction wheel model for back substitution");
                back_sub.matrix_d += contribution.matrix_d_correction_kg_m2;
                back_sub.vec_trans += contribution.force_body_n;
                back_sub.vec_rot += contribution.torque_body_nm;
            }
        }
    }

    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.assert_state_length(effector_state);
        let mut derivatives = vec![0.0; self.state_len()];
        for (wheel_index, wheel) in self.wheels.iter().enumerate() {
            let omega_dot_radps2 = if matches!(wheel.config.model, ReactionWheelModel::JitterFullyCoupled)
            {
                // a_omega/b_omega/c_omega were cached in update_contributions, which
                // has the body rate that compute_derivatives is not given.
                let terms = wheel.fully_coupled_terms.borrow();
                terms.a_omega.dot(&body_trans_accel_mps2)
                    + terms.b_omega.dot(&body_omega_dot_radps2)
                    + terms.c_omega
            } else {
                wheel
                    .omega_dot_radps2(body_omega_dot_radps2)
                    .expect("unsupported reaction wheel model for back substitution")
            };
            // Numerical-stability guard: unlimited torque on a small-inertia hub can
            // otherwise produce an unbounded wheel acceleration.
            let omega_dot_radps2 =
                omega_dot_radps2.clamp(-MAX_WHEEL_ACCELERATION_RADPS2, MAX_WHEEL_ACCELERATION_RADPS2);
            derivatives[wheel_index] = omega_dot_radps2;
            if let Some(theta_index) = self.theta_state_index(wheel_index) {
                derivatives[theta_index] = effector_state[wheel_index];
            }
        }
        derivatives
    }

    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        self.assert_state_length(effector_state);
        // Only the fully-coupled model adds wheel mass/inertia to the hub; the
        // balanced and JitterSimple models are handled entirely in back-substitution.
        let (mass_kg, first_moment, first_moment_prime, inertia_about_b, inertia_about_b_prime) =
            self.wheels.iter().enumerate().fold(
                (
                    0.0,
                    Vector3::zeros(),
                    Vector3::zeros(),
                    Matrix3::zeros(),
                    Matrix3::zeros(),
                ),
                |(mass_sum, first_moment_sum, first_moment_prime_sum, inertia_sum, inertia_prime_sum),
                 (wheel_index, wheel)| {
                    if !matches!(wheel.config.model, ReactionWheelModel::JitterFullyCoupled) {
                        return (
                            mass_sum,
                            first_moment_sum,
                            first_moment_prime_sum,
                            inertia_sum,
                            inertia_prime_sum,
                        );
                    }
                    let (omega_radps, theta_rad) = self.state_for_wheel(effector_state, wheel_index);
                    let k = wheel.fully_coupled_kinematics(theta_rad, omega_radps);
                    let r_tilde = tilde(k.com_position_body);
                    let r_prime_tilde = tilde(k.com_prime_body);
                    let inertia_about_point_b = k.inertia_body + k.mass * r_tilde * r_tilde.transpose();
                    let inertia_about_point_b_prime = k.inertia_prime_body
                        + k.mass
                            * (r_prime_tilde * r_tilde.transpose() + r_tilde * r_prime_tilde.transpose());
                    (
                        mass_sum + k.mass,
                        first_moment_sum + k.mass * k.com_position_body,
                        first_moment_prime_sum + k.mass * k.com_prime_body,
                        inertia_sum + inertia_about_point_b,
                        inertia_prime_sum + inertia_about_point_b_prime,
                    )
                },
            );

        let center_of_mass_body_m = if mass_kg > 0.0 {
            first_moment / mass_kg
        } else {
            Vector3::zeros()
        };
        let center_of_mass_prime_body_mps = if mass_kg > 0.0 {
            first_moment_prime / mass_kg
        } else {
            Vector3::zeros()
        };

        StateEffectorMassProps {
            mass_kg,
            mass_dot_kgps: 0.0,
            center_of_mass_body_m,
            center_of_mass_prime_body_mps,
            inertia_about_point_b_body_kg_m2: inertia_about_b,
            inertia_about_point_b_body_prime_kg_m2ps: inertia_about_b_prime,
        }
    }

    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        self.assert_state_length(effector_state);
        self.wheels
            .iter()
            .enumerate()
            .fold(Vector3::zeros(), |total, (wheel_index, wheel)| {
                let (omega_radps, theta_rad) = self.state_for_wheel(effector_state, wheel_index);
                if matches!(wheel.config.model, ReactionWheelModel::JitterFullyCoupled) {
                    let k = wheel.fully_coupled_kinematics(theta_rad, omega_radps);
                    let omega_wheel_body = body_omega_radps + omega_radps * k.gs_hat;
                    let com_velocity_body =
                        k.com_prime_body + body_omega_radps.cross(&k.com_position_body);
                    total
                        + k.inertia_body * omega_wheel_body
                        + k.mass * k.com_position_body.cross(&com_velocity_body)
                } else {
                    let spin_axis = normalize_or_zero(wheel.config.spin_axis_body);
                    total + spin_axis * (wheel.config.js_kg_m2 * omega_radps)
                }
            })
    }

    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        self.assert_state_length(effector_state);
        self.wheels
            .iter()
            .enumerate()
            .map(|(wheel_index, wheel)| {
                let (omega_radps, theta_rad) = self.state_for_wheel(effector_state, wheel_index);
                if matches!(wheel.config.model, ReactionWheelModel::JitterFullyCoupled) {
                    let k = wheel.fully_coupled_kinematics(theta_rad, omega_radps);
                    let omega_wheel_body = body_omega_radps + omega_radps * k.gs_hat;
                    let com_velocity_body =
                        k.com_prime_body + body_omega_radps.cross(&k.com_position_body);
                    0.5 * omega_wheel_body.dot(&(k.inertia_body * omega_wheel_body))
                        + 0.5 * k.mass * com_velocity_body.dot(&com_velocity_body)
                } else {
                    let spin_axis = normalize_or_zero(wheel.config.spin_axis_body);
                    0.5 * wheel.config.js_kg_m2 * omega_radps * omega_radps
                        + wheel.config.js_kg_m2 * omega_radps * spin_axis.dot(&body_omega_radps)
                }
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

/// Wheel-frame quantities for the fully-coupled model at a given angle and speed.
struct FullyCoupledKinematics {
    gs_hat: Vector3<f64>,
    w2_hat: Vector3<f64>,
    w3_hat: Vector3<f64>,
    /// Imbalance center-of-mass offset along `w2_hat`.
    d: f64,
    mass: f64,
    /// Wheel inertia about the wheel center of mass, body frame.
    inertia_body: Matrix3<f64>,
    /// Body-frame time derivative of `inertia_body`.
    inertia_prime_body: Matrix3<f64>,
    /// Wheel center-of-mass position relative to the body origin.
    com_position_body: Vector3<f64>,
    /// Body-frame time derivative of `com_position_body`.
    com_prime_body: Vector3<f64>,
}

/// Fully-coupled back-substitution matrix and vector contributions.
struct FullyCoupledBackSub {
    matrix_a: Matrix3<f64>,
    matrix_b: Matrix3<f64>,
    matrix_c: Matrix3<f64>,
    matrix_d: Matrix3<f64>,
    vec_trans: Vector3<f64>,
    vec_rot: Vector3<f64>,
}

/// Cached wheel-acceleration coupling vectors for the fully-coupled model.
#[derive(Clone, Copy, Debug, Default)]
struct FullyCoupledOmegaTerms {
    a_omega: Vector3<f64>,
    b_omega: Vector3<f64>,
    c_omega: f64,
}

/// Skew-symmetric cross-product matrix: `tilde(v) * x == v.cross(&x)`.
fn tilde(v: Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(0.0, -v.z, v.y, v.z, 0.0, -v.x, -v.y, v.x, 0.0)
}

/// Sign of `x`, `0.0` at `x == 0.0` (unlike `f64::signum`, which returns `1.0`
/// there) — needed so a zero-speed direction reversal is detected correctly.
fn sign(x: f64) -> f64 {
    (x > 0.0) as i8 as f64 - (x < 0.0) as i8 as f64
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

    /// With Stribeck disabled (`beta_static <= 0`) and the wheel above the limit
    /// cycle, friction is plain Coulomb + viscous: `-(sign(omega)*fCoulomb +
    /// cViscous*omega)`.
    #[test]
    fn coulomb_and_viscous_friction_without_stribeck() {
        let cases = [(10.0, -0.06), (-10.0, 0.06)];
        for (omega, expected) in cases {
            let mut rw = rw_with_command(10.0, 0.0, 0.0, -1.0, omega, 0.0);
            rw.wheels[0].config.coulomb_friction_nm = 0.05;
            rw.wheels[0].config.viscous_friction_nms_per_rad = 0.001;
            StateEffector::pre_integration(&mut rw, 0, 1.0);
            assert!(
                (rw.wheels()[0].friction_torque_nm - expected).abs() < 1e-10,
                "omega={omega}: expected friction={expected}, got {}",
                rw.wheels()[0].friction_torque_nm
            );
        }
    }

    /// Below `0.1 * omega_limit_cycle_radps`, Stribeck engages, and within the
    /// limit cycle the friction is linearly blended toward zero. Recomputes the
    /// closed form independently to pin down the transcription.
    #[test]
    fn stribeck_friction_blends_linearly_inside_limit_cycle() {
        let beta_static = 0.5;
        let static_friction_nm = 0.02;
        let coulomb_friction_nm = 0.01;
        let viscous = 0.0;
        let omega_limit_cycle = 1.0;
        let omega = 0.05; // < 0.1 * omega_limit_cycle: Stribeck engages

        let mut rw = rw_with_command(10.0, 0.0, 0.0, -1.0, omega, 0.0);
        rw.wheels[0].config.beta_static = beta_static;
        rw.wheels[0].config.static_friction_nm = static_friction_nm;
        rw.wheels[0].config.coulomb_friction_nm = coulomb_friction_nm;
        rw.wheels[0].config.viscous_friction_nms_per_rad = viscous;
        rw.wheels[0].config.omega_limit_cycle_radps = omega_limit_cycle;
        StateEffector::pre_integration(&mut rw, 0, 1.0);

        let ratio = omega_limit_cycle / beta_static;
        let friction_at_limit_cycle = (2.0 * std::f64::consts::E).sqrt()
            * (static_friction_nm - coulomb_friction_nm)
            * (-(ratio * ratio) / 2.0).exp()
            * omega_limit_cycle
            / (beta_static * 2.0_f64.sqrt())
            + coulomb_friction_nm * (omega_limit_cycle * 10.0 / beta_static).tanh()
            + viscous * omega_limit_cycle;
        let expected_friction_force = friction_at_limit_cycle / omega_limit_cycle * omega;

        assert!(
            rw.wheels()[0].friction_stribeck,
            "expected Stribeck to engage below 0.1 * omega_limit_cycle"
        );
        assert!(
            (rw.wheels()[0].friction_torque_nm - (-expected_friction_force)).abs() < 1e-12,
            "expected friction={}, got {}",
            -expected_friction_force,
            rw.wheels()[0].friction_torque_nm
        );
    }
}
