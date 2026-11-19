//! Ideal PD hinge controller: port of upstream Basilisk
//! `simulation/dynamics/hingedRigidBodyMotor` (`HingedRigidBodyMotor`).
//!
//! The module commands the hinge torque
//! `u = K (θ_ref − θ_sensed) + P (θ̇_ref − θ̇_sensed)`, optionally clamped to
//! `±u_max`, into entry 0 of an [`ArrayMotorTorqueMsg`], the motor-torque input
//! of [`HingedRigidBodyStateEffector`]. Despite the name it does not model
//! motor electromechanics; the gains are active feedback, separate from the
//! hinge's passive spring and damper.
//!
//! [`HingedRigidBodyStateEffector`]: super::hinged_rigid_body_state_effector::HingedRigidBodyStateEffector

use crate::messages::{ArrayMotorTorqueMsg, HingedRigidBodyMsg, Input, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`HingedRigidBodyMotor`].
#[derive(Clone, Debug)]
pub struct HingedRigidBodyMotorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `K` \[N m/rad\]: finite, non-negative gain on the hinge-angle
    /// tracking error (default 0).
    pub k_nm_per_rad: f64,
    /// Upstream `P` \[N m s/rad\]: finite, non-negative gain on the hinge-rate
    /// tracking error (default 0).
    pub p_nm_s_per_rad: f64,
    /// Upstream `uMax` \[N m\]: finite torque magnitude limit; a negative value
    /// disables saturation (default −1).
    pub u_max_nm: f64,
}

impl HingedRigidBodyMotorConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            k_nm_per_rad: 0.0,
            p_nm_s_per_rad: 0.0,
            u_max_nm: -1.0,
        }
    }

    /// The gain and limit checks of upstream `Reset`.
    pub fn validate(&self) -> Result<(), String> {
        let gains_valid = [self.k_nm_per_rad, self.p_nm_s_per_rad]
            .iter()
            .all(|gain| gain.is_finite() && *gain >= 0.0);
        if !gains_valid {
            return Err("K and P must be set to finite, non-negative values".to_string());
        }
        if !self.u_max_nm.is_finite() {
            return Err("uMax must be set to a finite value".to_string());
        }
        Ok(())
    }
}

/// PD hinge torque controller (upstream `HingedRigidBodyMotor`).
#[derive(Clone, Debug)]
pub struct HingedRigidBodyMotor {
    pub config: HingedRigidBodyMotorConfig,
    /// Upstream `hingedBodyStateSensedInMsg`: sensed hinge angle and rate.
    pub hinged_body_state_sensed_in_msg: Input<HingedRigidBodyMsg>,
    /// Upstream `hingedBodyStateReferenceInMsg`: reference hinge angle and
    /// rate.
    pub hinged_body_state_reference_in_msg: Input<HingedRigidBodyMsg>,
    /// Upstream `motorTorqueOutMsg`: the commanded torque in entry 0.
    pub motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
}

impl HingedRigidBodyMotor {
    pub fn new(config: HingedRigidBodyMotorConfig) -> Self {
        Self {
            config,
            hinged_body_state_sensed_in_msg: Input::default(),
            hinged_body_state_reference_in_msg: Input::default(),
            motor_torque_out_msg: Output::default(),
        }
    }

    /// The commanded torque \[N m\] for a sensed and a reference hinge state.
    pub fn torque_nm(&self, sensed: &HingedRigidBodyMsg, reference: &HingedRigidBodyMsg) -> f64 {
        let torque_nm = self.config.k_nm_per_rad * (reference.theta_rad - sensed.theta_rad)
            + self.config.p_nm_s_per_rad * (reference.theta_dot_radps - sensed.theta_dot_radps);
        let u_max_nm = self.config.u_max_nm;
        if u_max_nm >= 0.0 {
            torque_nm.clamp(-u_max_nm, u_max_nm)
        } else {
            torque_nm
        }
    }
}

impl Module for HingedRigidBodyMotor {
    /// Upstream `Reset`: reports unconnected inputs and checks the gains.
    ///
    /// # Panics
    ///
    /// If [`HingedRigidBodyMotorConfig::validate`] fails.
    fn init(&mut self) {
        if !self.hinged_body_state_sensed_in_msg.is_connected() {
            log::error!(
                "HingedRigidBodyMotor '{}': hinged_body_state_sensed_in_msg is not connected",
                self.config.name
            );
        }
        if !self.hinged_body_state_reference_in_msg.is_connected() {
            log::error!(
                "HingedRigidBodyMotor '{}': hinged_body_state_reference_in_msg is not connected",
                self.config.name
            );
        }
        if let Err(message) = self.config.validate() {
            panic!("HingedRigidBodyMotor '{}': {message}", self.config.name);
        }
    }

    /// Upstream `UpdateState`.
    fn update(&mut self, _context: &SimulationContext) {
        let torque_nm = self.torque_nm(
            &self.hinged_body_state_sensed_in_msg.read(),
            &self.hinged_body_state_reference_in_msg.read(),
        );
        self.motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::from_active(&[torque_nm]));
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_hingedRigidBodyMotor.py`.

    use super::{HingedRigidBodyMotor, HingedRigidBodyMotorConfig};
    use crate::Module;
    use crate::messages::{HingedRigidBodyMsg, Output};
    use crate::simulation::Simulation;
    use hifitime::Epoch;

    fn reset_ready_module(config: HingedRigidBodyMotorConfig) -> HingedRigidBodyMotor {
        let mut module = HingedRigidBodyMotor::new(config);
        let sensed = Output::new(HingedRigidBodyMsg::default());
        let reference = Output::new(HingedRigidBodyMsg::default());
        module
            .hinged_body_state_sensed_in_msg
            .connect(sensed.slot());
        module
            .hinged_body_state_reference_in_msg
            .connect(reference.slot());
        module
    }

    /// Upstream `run_motor_case`: 1 s at a 0.5 s rate; the last torque output.
    fn run_motor_case(
        k: f64,
        p: f64,
        sensed: (f64, f64),
        reference: (f64, f64),
        u_max: Option<f64>,
    ) -> f64 {
        let mut config = HingedRigidBodyMotorConfig::new("hingedRigidBodyMotorTag");
        config.k_nm_per_rad = k;
        config.p_nm_s_per_rad = p;
        if let Some(u_max) = u_max {
            config.u_max_nm = u_max;
        }
        let mut module = HingedRigidBodyMotor::new(config);
        let sensed_out = Output::new(HingedRigidBodyMsg {
            theta_rad: sensed.0,
            theta_dot_radps: sensed.1,
        });
        let reference_out = Output::new(HingedRigidBodyMsg {
            theta_rad: reference.0,
            theta_dot_radps: reference.1,
        });
        let torque_out = module.motor_torque_out_msg.clone();
        {
            let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            sim.connect(&sensed_out, &mut module.hinged_body_state_sensed_in_msg);
            sim.connect(
                &reference_out,
                &mut module.hinged_body_state_reference_in_msg,
            );
            sim.add_module("motor", &mut module, 500_000_000, 0);
            sim.run_for(1_000_000_000);
        }
        torque_out.read().motor_torque_nm[0]
    }

    /// Upstream `test_hingedRigidBodyMotor` (all nine parametrizations).
    #[test]
    fn hinged_rigid_body_motor() {
        let cases = [
            (
                "unlimited-positive",
                5.0,
                1.0,
                (1.0, 0.1),
                (1.2, 0.2),
                None,
                1.1,
            ),
            (
                "unlimited-negative",
                5.0,
                1.0,
                (1.0, 0.1),
                (0.8, -0.1),
                None,
                -1.2,
            ),
            (
                "limited-in-range",
                5.0,
                1.0,
                (1.0, 0.1),
                (1.2, 0.2),
                Some(2.0),
                1.1,
            ),
            (
                "limited-positive",
                5.0,
                1.0,
                (1.0, 0.1),
                (1.2, 0.2),
                Some(0.5),
                0.5,
            ),
            (
                "limited-negative",
                5.0,
                1.0,
                (1.0, 0.1),
                (0.8, -0.1),
                Some(0.5),
                -0.5,
            ),
            (
                "zero-limit",
                5.0,
                1.0,
                (1.0, 0.1),
                (1.2, 0.2),
                Some(0.0),
                0.0,
            ),
            ("rate-only", 0.0, 1.0, (1.0, 0.1), (1.2, 0.2), None, 0.1),
            (
                "proportional-only",
                5.0,
                0.0,
                (1.0, 0.1),
                (1.2, 0.2),
                None,
                1.0,
            ),
            ("zero-gains", 0.0, 0.0, (1.0, 0.1), (1.2, 0.2), None, 0.0),
        ];
        for (id, k, p, sensed, reference, u_max, expected_torque) in cases {
            let actual_torque = run_motor_case(k, p, sensed, reference, u_max);
            // pytest.approx(expected, rel=1e-12), which also allows 1e-12 absolute.
            let tolerance = (1e-12 * f64::abs(expected_torque)).max(1e-12);
            assert!(
                (actual_torque - expected_torque).abs() <= tolerance,
                "{id}: torque {actual_torque} != {expected_torque}"
            );
        }
    }

    /// Upstream `test_hingedRigidBodyMotor_rejects_invalid_gains`.
    #[test]
    fn hinged_rigid_body_motor_rejects_invalid_gains() {
        let cases = [
            ("negative-K", -1.0, 1.0),
            ("negative-P", 1.0, -1.0),
            ("nan-K", f64::NAN, 1.0),
            ("nan-P", 1.0, f64::NAN),
            ("infinite-K", f64::INFINITY, 1.0),
            ("infinite-P", 1.0, f64::INFINITY),
        ];
        for (id, k, p) in cases {
            let mut config = HingedRigidBodyMotorConfig::new(id);
            config.k_nm_per_rad = k;
            config.p_nm_s_per_rad = p;
            let error = config.validate().expect_err(id);
            assert!(
                error.contains("K and P must be set to finite, non-negative values"),
                "{id}: {error}"
            );
        }
    }

    /// Upstream `test_hingedRigidBodyMotor_rejects_nonfinite_torque_limit`.
    #[test]
    fn hinged_rigid_body_motor_rejects_nonfinite_torque_limit() {
        for u_max in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut config = HingedRigidBodyMotorConfig::new("motor");
            config.u_max_nm = u_max;
            let error = config.validate().expect_err("non-finite uMax");
            assert!(
                error.contains("uMax must be set to a finite value"),
                "{u_max}: {error}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "K and P must be set to finite, non-negative values")]
    fn reset_rejects_invalid_gains() {
        let mut config = HingedRigidBodyMotorConfig::new("motor");
        config.k_nm_per_rad = -1.0;
        reset_ready_module(config).init();
    }

    #[test]
    #[should_panic(expected = "uMax must be set to a finite value")]
    fn reset_rejects_nonfinite_torque_limit() {
        let mut config = HingedRigidBodyMotorConfig::new("motor");
        config.u_max_nm = f64::NAN;
        reset_ready_module(config).init();
    }
}
