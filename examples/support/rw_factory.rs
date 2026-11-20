//! Reaction wheels as upstream `simIncludeRW.rwFactory` creates them.
//!
//! `rwFactory.create('Honeywell_HR16', gsHat_B, maxMomentum=..., Omega=...,
//! rWB_B=..., RWModel=...)` with the factory defaults: wheel friction off
//! (`useRWfriction=False` zeroes `fCoulomb`), no minimum torque
//! (`useMinTorque=False`), the 0.2 N m maximum torque on, no power limit, the
//! Stribeck model off, and the spin inertia from the momentum capacity,
//! `Js = maxMomentum / Omega_max`, `Jt = Jg = Js / 2`.

#![allow(dead_code)]

use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelModel, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::messages::RwArrayConfigMsg;
use nalgebra::Vector3;

use super::common::rpm_to_radps;

/// Upstream `rwFactory`: the wheels in creation order.
#[derive(Clone, Debug, Default)]
pub struct RwFactory {
    pub wheels: Vec<ReactionWheelStateEffectorConfig>,
}

impl RwFactory {
    /// Upstream `create('Honeywell_HR16', gsHat_B, maxMomentum=...,
    /// Omega=..., rWB_B=..., RWModel=...)`.
    ///
    /// # Panics
    ///
    /// If `max_momentum_nms` is not one of the HR16 sizes, 50, 75 or
    /// 100 N m s (upstream exits).
    pub fn create_honeywell_hr16(
        &mut self,
        spin_axis_body: Vector3<f64>,
        max_momentum_nms: f64,
        initial_speed_rpm: f64,
        position_m: Vector3<f64>,
        model: ReactionWheelModel,
    ) -> &mut ReactionWheelStateEffectorConfig {
        // Upstream `Honeywell_HR16`: mass [kg], static [kg m] and dynamic
        // [kg m^2] imbalance by momentum capacity.
        let (mass_kg, static_imbalance, dynamic_imbalance) = if max_momentum_nms == 100.0 {
            (12.0, 4.8e-6, 15.4e-7)
        } else if max_momentum_nms == 75.0 {
            (10.4, 3.8e-6, 11.5e-7)
        } else if max_momentum_nms == 50.0 {
            (9.0, 2.8e-6, 7.7e-7)
        } else {
            panic!("Honeywell_HR16 has no {max_momentum_nms} N m s size")
        };
        let max_speed_radps = rpm_to_radps(6_000.0);
        let mut config = ReactionWheelStateEffectorConfig::balanced(
            format!("RW{}", self.wheels.len() + 1),
            position_m,
            spin_axis_body.normalize(),
            0.2,
            max_momentum_nms,
        );
        config.max_speed_radps = max_speed_radps;
        config.js_kg_m2 = max_momentum_nms / max_speed_radps;
        config.jt_kg_m2 = 0.5 * config.js_kg_m2;
        config.jg_kg_m2 = config.jt_kg_m2;
        config.mass_kg = mass_kg;
        config.static_imbalance_kg_m = static_imbalance;
        config.dynamic_imbalance_kg_m2 = dynamic_imbalance;
        config.model = model;
        config.initial_omega_radps = rpm_to_radps(initial_speed_rpm);
        self.wheels.push(config);
        self.wheels.last_mut().expect("just created")
    }

    /// Upstream `create('BCT_RWP015', gsHat_B, Omega_max=..., RWModel=...,
    /// useRWfriction=False)`: 4 mN m maximum torque, the fixed 15 mN m s
    /// momentum capacity, `Js = maxMomentum / Omega_max`, `Jt = Jg = Js / 2`,
    /// 0.130 kg, no friction and no minimum torque, at rest at the origin.
    pub fn create_bct_rwp015(
        &mut self,
        spin_axis_body: Vector3<f64>,
        max_speed_rpm: f64,
        model: ReactionWheelModel,
    ) -> &mut ReactionWheelStateEffectorConfig {
        let max_speed_radps = rpm_to_radps(max_speed_rpm);
        let mut config = ReactionWheelStateEffectorConfig::balanced(
            format!("RW{}", self.wheels.len() + 1),
            Vector3::zeros(),
            spin_axis_body,
            0.004,
            0.015,
        );
        config.js_kg_m2 = 0.015 / max_speed_radps;
        config.jt_kg_m2 = 0.5 * config.js_kg_m2;
        config.jg_kg_m2 = config.jt_kg_m2;
        config.max_speed_radps = max_speed_radps;
        config.mass_kg = 0.130;
        config.static_imbalance_kg_m = 1e-7;
        config.dynamic_imbalance_kg_m2 = 1e-8;
        config.model = model;
        self.wheels.push(config);
        self.wheels.last_mut().expect("just created")
    }

    /// Upstream `getConfigMessage()` (also what `fswSetupRW.create(gsHat_B,
    /// Js, 0.2)` builds): spin axes, spin inertias and maximum torques.
    pub fn config_message(&self) -> RwArrayConfigMsg {
        let mut config = RwArrayConfigMsg {
            num_rw: self.wheels.len(),
            ..Default::default()
        };
        for (index, wheel) in self.wheels.iter().enumerate() {
            config.spin_axes_body[index] = wheel.spin_axis_body;
            config.spin_axis_inertias_kg_m2[index] = wheel.js_kg_m2;
            config.max_motor_torques_nm[index] = wheel.max_torque_nm;
        }
        config
    }
}
