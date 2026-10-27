//! PID motor torque for a one-degree-of-freedom hinged or spinning body, such
//! as a solar-array drive.
//!
//! Port of upstream Basilisk `hingedRigidBodyPIDMotor`. The commanded torque is
//!
//! ```text
//! T = K (θ_R − θ) + P (θ̇_R − θ̇) + I ∫ (θ_R − θ) dt
//! ```
//!
//! with the integral accumulated by the trapezoid rule between updates. The
//! torque is published as the first entry of an [`ArrayMotorTorqueMsg`].

use crate::messages::{ArrayMotorTorqueMsg, HingedRigidBodyMsg, Input, Output};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`HingedRigidBodyPidMotor`]; upstream defaults
/// every gain to zero.
#[derive(Clone, Debug)]
pub struct HingedRigidBodyPidMotorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Proportional gain (upstream `K`) \[N m/rad\].
    pub k: f64,
    /// Derivative gain (upstream `P`) \[N m s/rad\].
    pub p: f64,
    /// Integral gain (upstream `I`) \[N m/(rad s)\].
    pub i: f64,
}

impl Default for HingedRigidBodyPidMotorConfig {
    fn default() -> Self {
        Self {
            name: "hingedRigidBodyPIDMotor".to_string(),
            k: 0.0,
            p: 0.0,
            i: 0.0,
        }
    }
}

/// PID controller driving a hinged body towards its reference angle.
#[derive(Clone, Debug)]
pub struct HingedRigidBodyPidMotor {
    /// Module configuration.
    pub config: HingedRigidBodyPidMotorConfig,
    /// Current hinge angle and rate (upstream `hingedRigidBodyInMsg`).
    pub hinged_rigid_body_in_msg: Input<HingedRigidBodyMsg>,
    /// Reference hinge angle and rate (upstream `hingedRigidBodyRefInMsg`).
    pub hinged_rigid_body_ref_in_msg: Input<HingedRigidBodyMsg>,
    /// Motor torque; only entry 0 is used (upstream `motorTorqueOutMsg`).
    pub motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
    prior_time_nanos: u64,
    prior_theta_error_rad: f64,
    integral_error_rad_s: f64,
}

impl HingedRigidBodyPidMotor {
    /// Creates the module with unconnected ports and a zero integral.
    pub fn new(config: HingedRigidBodyPidMotorConfig) -> Self {
        Self {
            config,
            hinged_rigid_body_in_msg: Input::default(),
            hinged_rigid_body_ref_in_msg: Input::default(),
            motor_torque_out_msg: Output::default(),
            prior_time_nanos: 0,
            prior_theta_error_rad: 0.0,
            integral_error_rad_s: 0.0,
        }
    }

    /// Accumulated trapezoid integral of the angle error \[rad s\].
    pub fn integral_error_rad_s(&self) -> f64 {
        self.integral_error_rad_s
    }

    fn reset_state(&mut self) {
        if !self.hinged_rigid_body_in_msg.is_connected() {
            log::error!("hingedRigidBodyPIDMotor.hinged_rigid_body_in_msg is not connected");
        }
        if !self.hinged_rigid_body_ref_in_msg.is_connected() {
            log::error!("hingedRigidBodyPIDMotor.hinged_rigid_body_ref_in_msg is not connected");
        }
        self.prior_time_nanos = 0;
        self.prior_theta_error_rad = 0.0;
        self.integral_error_rad_s = 0.0;
    }

    fn compute_torque_nm(&mut self, call_time_nanos: u64) -> f64 {
        let state = self.hinged_rigid_body_in_msg.read();
        let reference = self.hinged_rigid_body_ref_in_msg.read();
        let theta_error = reference.theta_rad - state.theta_rad;
        let theta_error_dot = reference.theta_dot_radps - state.theta_dot_radps;

        // Upstream skips the integral only when the call time is zero, not on
        // the first call after a reset: the first update after a reset at
        // t > 0 integrates from t = 0 with a zero prior error.
        if call_time_nanos != 0 {
            let dt = diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos);
            self.integral_error_rad_s += (theta_error + self.prior_theta_error_rad) * dt / 2.0;
        }
        self.prior_theta_error_rad = theta_error;
        self.prior_time_nanos = call_time_nanos;

        self.config.k * theta_error
            + self.config.p * theta_error_dot
            + self.config.i * self.integral_error_rad_s
    }
}

impl Module for HingedRigidBodyPidMotor {
    fn init(&mut self) {
        self.motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let torque_nm = self.compute_torque_nm(context.current_sim_nanos);
        self.motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::from_active(&[torque_nm]));
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;

    use crate::messages::{HingedRigidBodyMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{HingedRigidBodyPidMotor, HingedRigidBodyPidMotorConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn hinged(theta_rad: f64, theta_dot_radps: f64) -> HingedRigidBodyMsg {
        HingedRigidBodyMsg {
            theta_rad,
            theta_dot_radps,
        }
    }

    fn connected_motor(
        config: HingedRigidBodyPidMotorConfig,
        state: &Output<HingedRigidBodyMsg>,
        reference: &Output<HingedRigidBodyMsg>,
    ) -> HingedRigidBodyPidMotor {
        let mut motor = HingedRigidBodyPidMotor::new(config);
        state.connect_to(&mut motor.hinged_rigid_body_in_msg);
        reference.connect_to(&mut motor.hinged_rigid_body_ref_in_msg);
        motor.init();
        motor
    }

    /// Port of `test_hingedRigidBodyPIDMotor.py`: the sim runs at 1 s and stops
    /// at 0.5 s, so only the t = 0 output (no integral contribution) is checked.
    #[test]
    fn matches_upstream_hinged_rigid_body_pid_motor_unit_test() {
        let accuracy = 1e-8;
        for theta_ref in [PI / 2.0, PI / 6.0] {
            for theta_dot_ref in [0.1, 0.2] {
                for theta in [7.0 * PI / 6.0, 3.0 * PI / 2.0] {
                    for theta_dot in [-0.1, -0.5] {
                        for k in [0.0, 2.0] {
                            for p in [0.0, 5.0] {
                                for i in [0.0, 1.0] {
                                    let state = Output::new(hinged(theta, theta_dot));
                                    let reference = Output::new(hinged(theta_ref, theta_dot_ref));
                                    let mut motor = connected_motor(
                                        HingedRigidBodyPidMotorConfig {
                                            k,
                                            p,
                                            i,
                                            ..Default::default()
                                        },
                                        &state,
                                        &reference,
                                    );
                                    motor.update(&context(0));

                                    let expected =
                                        k * (theta_ref - theta) + p * (theta_dot_ref - theta_dot);
                                    let actual = motor.motor_torque_out_msg.read().motor_torque_nm;
                                    assert!(
                                        (actual[0] - expected).abs() <= accuracy,
                                        "thetaR={theta_ref}, thetaDotR={theta_dot_ref}, theta={theta}, \
                                         thetaDot={theta_dot}, K={k}, P={p}, I={i}: expected {expected}, got {}",
                                        actual[0]
                                    );
                                    assert!(actual[1..].iter().all(|torque| *torque == 0.0));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn integrates_the_angle_error_with_the_trapezoid_rule() {
        let state = Output::new(hinged(0.0, 0.0));
        let reference = Output::new(hinged(0.4, 0.0));
        let mut motor = connected_motor(
            HingedRigidBodyPidMotorConfig {
                k: 0.0,
                p: 0.0,
                i: 2.0,
                ..Default::default()
            },
            &state,
            &reference,
        );

        // t = 0: no integration. t = 1 s: (0.4 + 0.4) / 2 * 1 s = 0.4 rad s.
        motor.update(&context(0));
        assert_eq!(motor.motor_torque_out_msg.read().motor_torque_nm[0], 0.0);
        motor.update(&context(1_000_000_000));
        assert!((motor.integral_error_rad_s() - 0.4).abs() <= 1e-15);

        // Error drops to 0.2 rad; trapezoid over 0.5 s adds (0.2 + 0.4) / 2 * 0.5.
        state.write(hinged(0.2, 0.0));
        motor.update(&context(1_500_000_000));
        assert!((motor.integral_error_rad_s() - 0.55).abs() <= 1e-15);
        assert!((motor.motor_torque_out_msg.read().motor_torque_nm[0] - 1.1).abs() <= 1e-15);
    }

    #[test]
    fn reset_clears_the_integral_state() {
        let state = Output::new(hinged(0.0, 0.0));
        let reference = Output::new(hinged(1.0, 0.0));
        let mut motor = connected_motor(
            HingedRigidBodyPidMotorConfig {
                i: 1.0,
                ..Default::default()
            },
            &state,
            &reference,
        );
        motor.update(&context(0));
        motor.update(&context(1_000_000_000));
        assert!((motor.integral_error_rad_s() - 1.0).abs() <= 1e-15);

        Module::reset(&mut motor, &context(2_000_000_000));
        assert_eq!(motor.integral_error_rad_s(), 0.0);

        // Upstream quirk: the first update after a reset at t > 0 integrates
        // from t = 0 with a zero prior error, (1 + 0) / 2 * 3 s.
        motor.update(&context(3_000_000_000));
        assert!((motor.integral_error_rad_s() - 1.5).abs() <= 1e-15);
    }
}
