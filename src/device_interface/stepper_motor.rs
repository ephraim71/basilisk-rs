//! Stepper motor: port of upstream Basilisk
//! `simulation/deviceInterface/stepperMotor`.
//!
//! The motor turns through a commanded signed number of steps of
//! `step_angle_rad`, each one taking `step_time_s`. Every step is a bang-bang
//! profile from rest to rest: the maximum acceleration
//! `θ̈_max = step_angle / (step_time / 2)^2` for the first half, its opposite
//! for the second. The motor publishes its angle, rate, acceleration, the
//! steps commanded and the steps taken.
//!
//! A new command arriving when the motor is idle starts from the current
//! angle. One arriving mid-actuation interrupts it: the step in progress is
//! completed, then the motor takes the new command's steps from the angle it
//! reached. A command of zero steps stops the motor after the step in
//! progress and clears the step count.
//!
//! Upstream tells a new command from a repeat by the message-header
//! `timeWritten`. Stand-in: the producer's time stamp,
//! [`MotorStepCommandMsg::command_time_nanos`]; a command is new when its
//! stamp is later than the last one taken (the first command read after a
//! reset is always new). As upstream, a connected input is read every update
//! (connected ⇒ written); an unconnected one is never read.
//!
//! As upstream, the configured initial angle is applied by `init`; a later
//! reset starts from the angle at which the last command began.

use crate::messages::{Input, MotorStepCommandMsg, Output, StepperMotorMsg};
use crate::time::NANO2SEC;
use crate::{Module, SimulationContext};

/// Time tolerance \[s\] of upstream's step-segment checks.
const STEP_TIME_TOLERANCE_S: f64 = 1e-5;

/// User-settable parameters of [`StepperMotor`].
#[derive(Clone, Debug, PartialEq)]
pub struct StepperMotorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `thetaInit` \[rad\]: motor angle before the first command.
    pub theta_init_rad: f64,
    /// Upstream `stepAngle` \[rad\]: angle of one step, positive.
    pub step_angle_rad: f64,
    /// Upstream `stepTime` \[s\]: duration of one step, positive.
    pub step_time_s: f64,
}

impl StepperMotorConfig {
    /// Upstream's setter assertions: a positive step angle and step time.
    pub fn validate(&self) -> Result<(), String> {
        if !(self.step_angle_rad > 0.0 && self.step_angle_rad.is_finite()) {
            return Err(format!(
                "step_angle_rad must be positive, got {}",
                self.step_angle_rad
            ));
        }
        if !(self.step_time_s > 0.0 && self.step_time_s.is_finite()) {
            return Err(format!(
                "step_time_s must be positive, got {}",
                self.step_time_s
            ));
        }
        if !self.theta_init_rad.is_finite() {
            return Err(format!(
                "theta_init_rad must be finite, got {}",
                self.theta_init_rad
            ));
        }
        Ok(())
    }

    /// Upstream `thetaDDotMax` \[rad/s^2\].
    pub fn theta_ddot_max_radps2(&self) -> f64 {
        self.step_angle_rad / (0.25 * self.step_time_s * self.step_time_s)
    }
}

/// Stepper motor actuation profile: upstream `StepperMotor`.
#[derive(Clone, Debug)]
pub struct StepperMotor {
    /// Module configuration.
    pub config: StepperMotorConfig,
    /// Upstream `motorStepCommandInMsg`: the number of steps to take.
    pub motor_step_command_in_msg: Input<MotorStepCommandMsg>,
    /// Upstream `stepperMotorOutMsg`: the motor state.
    pub stepper_motor_out_msg: Output<StepperMotorMsg>,
    steps_commanded: i32,
    step_count: i32,
    /// Upstream `thetaInit`: angle at which the current command began.
    theta_init_rad: f64,
    intermediate_theta_init_rad: f64,
    intermediate_theta_ref_rad: f64,
    theta_rad: f64,
    theta_dot_radps: f64,
    theta_ddot_radps2: f64,
    theta_ddot_max_radps2: f64,
    /// Upstream `tInit` \[s\]: start of the current step.
    t_init_s: f64,
    /// Upstream `previousWrittenTime`: stamp of the last command taken.
    previous_command_time_nanos: Option<u64>,
    /// Upstream `ts` \[s\]: midpoint of the current step.
    t_switch_s: f64,
    /// Upstream `tf` \[s\]: end of the current step.
    t_final_s: f64,
    actuation_complete: bool,
    step_complete: bool,
    new_msg: bool,
    interrupt_msg: bool,
    /// Parabola constants of the two halves of a step.
    a: f64,
    b: f64,
}

impl StepperMotor {
    /// Builds the motor with unconnected ports.
    ///
    /// # Panics
    ///
    /// If `config` fails [`StepperMotorConfig::validate`].
    pub fn new(config: StepperMotorConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid StepperMotorConfig: {message}");
        }
        Self {
            theta_init_rad: config.theta_init_rad,
            config,
            motor_step_command_in_msg: Input::default(),
            stepper_motor_out_msg: Output::default(),
            steps_commanded: 0,
            step_count: 0,
            intermediate_theta_init_rad: 0.0,
            intermediate_theta_ref_rad: 0.0,
            theta_rad: 0.0,
            theta_dot_radps: 0.0,
            theta_ddot_radps2: 0.0,
            theta_ddot_max_radps2: 0.0,
            t_init_s: 0.0,
            previous_command_time_nanos: None,
            t_switch_s: 0.0,
            t_final_s: 0.0,
            actuation_complete: true,
            step_complete: true,
            new_msg: false,
            interrupt_msg: false,
            a: 0.0,
            b: 0.0,
        }
    }

    /// Upstream `thetaDDotMax` \[rad/s^2\], set at reset.
    pub fn theta_ddot_max_radps2(&self) -> f64 {
        self.theta_ddot_max_radps2
    }

    /// Upstream `Reset`.
    fn reset_motor_states(&mut self) {
        if !self.motor_step_command_in_msg.is_connected() {
            log::error!(
                "stepperMotor '{}' motor_step_command_in_msg is not connected",
                self.config.name
            );
        }
        self.theta_rad = self.theta_init_rad;
        self.theta_dot_radps = 0.0;
        self.theta_ddot_radps2 = 0.0;
        self.t_init_s = 0.0;
        self.step_count = 0;
        self.previous_command_time_nanos = None;
        self.actuation_complete = true;
        self.step_complete = true;
        self.new_msg = false;
        self.interrupt_msg = false;
        self.theta_ddot_max_radps2 = self.config.theta_ddot_max_radps2();
    }

    /// Upstream `UpdateState`'s command handling: takes a command with a
    /// later stamp than the last.
    fn read_command(&mut self) {
        if !self.motor_step_command_in_msg.is_connected() {
            return;
        }
        let command = self.motor_step_command_in_msg.read();
        let is_new = self
            .previous_command_time_nanos
            .is_none_or(|previous| previous < command.command_time_nanos);
        if !is_new {
            return;
        }
        self.steps_commanded = command.steps_commanded;
        self.previous_command_time_nanos = Some(command.command_time_nanos);
        self.new_msg = true;
        self.interrupt_msg = !self.actuation_complete;
        if self.steps_commanded == 0 {
            self.actuation_complete = true;
            self.step_count = 0;
        } else {
            self.actuation_complete = false;
        }
    }

    /// Upstream `UpdateState`'s state propagation at `t_s`.
    fn propagate(&mut self, t_s: f64) {
        if self.actuation_complete {
            self.t_init_s = t_s;
            self.theta_ddot_radps2 = 0.0;
            return;
        }
        // Restart the step count on a new idle-time command, or once the
        // step an interrupting command arrived in has finished.
        if (self.new_msg && !self.interrupt_msg) || (self.interrupt_msg && self.step_complete) {
            self.reset_motor();
        }
        self.actuate_motor(t_s);
    }

    /// Upstream `actuateMotor`.
    fn actuate_motor(&mut self, t_s: f64) {
        if self.step_complete {
            self.update_step_parameters();
        }
        if self.is_in_step_first_half(t_s) {
            self.compute_step_first_half(t_s);
        } else if self.is_in_step_second_half(t_s) {
            self.compute_step_second_half(t_s);
        } else {
            self.compute_step_complete(t_s);
        }
    }

    /// Upstream `resetMotor`.
    fn reset_motor(&mut self) {
        self.step_count = 0;
        self.theta_init_rad = self.theta_rad;
        self.new_msg = false;
        self.interrupt_msg = false;
    }

    /// Upstream `updateStepParameters`.
    fn update_step_parameters(&mut self) {
        let step_angle = self.config.step_angle_rad;
        let step_time = self.config.step_time_s;
        self.step_complete = false;
        self.t_final_s = self.t_init_s + step_time;
        self.t_switch_s = self.t_init_s + step_time / 2.0;
        self.intermediate_theta_init_rad =
            self.theta_init_rad + f64::from(self.step_count) * step_angle;

        let (next_step, signed_step_angle) = if self.steps_commanded > 0 {
            (self.step_count + 1, step_angle)
        } else {
            (self.step_count - 1, -step_angle)
        };
        self.intermediate_theta_ref_rad = self.theta_init_rad + f64::from(next_step) * step_angle;
        self.a = 0.5 * signed_step_angle
            / ((self.t_switch_s - self.t_init_s) * (self.t_switch_s - self.t_init_s));
        self.b = -0.5 * signed_step_angle
            / ((self.t_switch_s - self.t_final_s) * (self.t_switch_s - self.t_final_s));
    }

    fn is_in_step_first_half(&self, t_s: f64) -> bool {
        t_s < self.t_switch_s && (self.t_switch_s - t_s).abs() > STEP_TIME_TOLERANCE_S
    }

    fn is_in_step_second_half(&self, t_s: f64) -> bool {
        (t_s >= self.t_switch_s || (self.t_switch_s - t_s).abs() < STEP_TIME_TOLERANCE_S)
            && (self.t_final_s - t_s).abs() > STEP_TIME_TOLERANCE_S
    }

    /// Whether the step in progress turns the motor forward.
    fn stepping_forward(&self) -> bool {
        self.intermediate_theta_ref_rad > self.intermediate_theta_init_rad
    }

    fn compute_step_first_half(&mut self, t_s: f64) {
        self.theta_ddot_radps2 = if self.stepping_forward() {
            self.theta_ddot_max_radps2
        } else {
            -self.theta_ddot_max_radps2
        };
        let elapsed_s = t_s - self.t_init_s;
        self.theta_dot_radps = self.theta_ddot_radps2 * elapsed_s;
        self.theta_rad = self.a * elapsed_s * elapsed_s + self.intermediate_theta_init_rad;
    }

    fn compute_step_second_half(&mut self, t_s: f64) {
        self.theta_ddot_radps2 = if self.stepping_forward() {
            -self.theta_ddot_max_radps2
        } else {
            self.theta_ddot_max_radps2
        };
        self.theta_dot_radps = self.theta_ddot_radps2 * (t_s - self.t_init_s)
            - self.theta_ddot_radps2 * (self.t_final_s - self.t_init_s);
        self.theta_rad = self.b * (t_s - self.t_final_s) * (t_s - self.t_final_s)
            + self.intermediate_theta_ref_rad;
    }

    fn compute_step_complete(&mut self, t_s: f64) {
        self.step_complete = true;
        self.theta_dot_radps = 0.0;
        self.theta_rad = self.intermediate_theta_ref_rad;
        self.t_init_s = t_s;
        if self.stepping_forward() {
            self.step_count += 1;
        } else {
            self.step_count -= 1;
        }
        if self.step_count == self.steps_commanded && !self.interrupt_msg {
            self.actuation_complete = true;
        }
    }
}

impl Module for StepperMotor {
    fn init(&mut self) {
        self.theta_init_rad = self.config.theta_init_rad;
        self.reset_motor_states();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_motor_states();
    }

    fn update(&mut self, context: &SimulationContext) {
        self.read_command();
        self.propagate(context.current_sim_nanos as f64 * NANO2SEC);
        self.stepper_motor_out_msg.write(StepperMotorMsg {
            theta_rad: self.theta_rad,
            theta_dot_radps: self.theta_dot_radps,
            theta_ddot_radps2: self.theta_ddot_radps2,
            steps_commanded: self.steps_commanded,
            step_count: self.step_count,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `stepperMotor/_UnitTest/test_stepperMotor.py`, its
    //! nominal and interruption tests over their full parameter grids with
    //! their tolerances. The pytest writes the second command with the time
    //! the first simulation chunk ended; that time is the command's stamp.

    use hifitime::Epoch;

    use super::{StepperMotor, StepperMotorConfig};
    use crate::messages::{MotorStepCommandMsg, Output, StepperMotorMsg};
    use crate::{Module, SimulationContext};

    const D2R: f64 = std::f64::consts::PI / 180.0;
    const R2D: f64 = 180.0 / std::f64::consts::PI;
    /// Upstream `test_time_step_sec` \[s\] in nanoseconds.
    const STEP_NANOS: u64 = 10_000_000;
    const ACCURACY: f64 = 1e-12;

    /// Upstream `macros.sec2nano`.
    fn sec2nano(time_s: f64) -> u64 {
        (time_s * 1e9 + 0.5) as u64
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Two simulation chunks: the first command from t = 0 through
    /// `first_stop`, the second, stamped `first_stop`, from the next tick
    /// through `second_stop`. Returns the log of every tick.
    fn run_two_commands(
        config: StepperMotorConfig,
        steps_commanded_1: i32,
        steps_commanded_2: i32,
        first_stop_nanos: u64,
        second_stop_nanos: u64,
    ) -> Vec<StepperMotorMsg> {
        let command_out = Output::new(MotorStepCommandMsg {
            steps_commanded: steps_commanded_1,
            command_time_nanos: 0,
        });
        let mut motor = StepperMotor::new(config);
        command_out.connect_to(&mut motor.motor_step_command_in_msg);
        motor.init();

        let mut log = Vec::new();
        let mut t_nanos = 0;
        while t_nanos <= first_stop_nanos {
            motor.update(&context(t_nanos));
            log.push(motor.stepper_motor_out_msg.read());
            t_nanos += STEP_NANOS;
        }
        command_out.write(MotorStepCommandMsg {
            steps_commanded: steps_commanded_2,
            command_time_nanos: first_stop_nanos,
        });
        while t_nanos <= second_stop_nanos {
            motor.update(&context(t_nanos));
            log.push(motor.stepper_motor_out_msg.read());
            t_nanos += STEP_NANOS;
        }
        log
    }

    fn assert_close(actual: f64, expected: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= ACCURACY,
            "{what}: {actual} vs {expected}"
        );
    }

    /// Upstream `test_stepper_motor_nominal` over its grid: each command runs
    /// to completion, with 5 s of rest after it.
    #[test]
    fn nominal_commands_reach_their_reference_angles() {
        for theta_init in [0.0, 10.0 * D2R, -5.0 * D2R] {
            for steps_commanded_1 in [0_i32, 10, -15] {
                for steps_commanded_2 in [0_i32, 10, -15] {
                    for step_angle in [0.1 * D2R, 0.5 * D2R, 1.0 * D2R] {
                        for step_time in [0.3, 0.5, 1.0] {
                            let what = format!(
                                "theta_init {theta_init}, steps {steps_commanded_1} then \
                                 {steps_commanded_2}, step angle {step_angle}, step time \
                                 {step_time}"
                            );
                            let sim_time_1 = step_time * f64::from(steps_commanded_1.abs());
                            let sim_time_2 = step_time * f64::from(steps_commanded_2.abs());
                            let sim_time_extra = 5.0;
                            let log = run_two_commands(
                                StepperMotorConfig {
                                    name: "StepperMotor".to_string(),
                                    theta_init_rad: theta_init,
                                    step_angle_rad: step_angle,
                                    step_time_s: step_time,
                                },
                                steps_commanded_1,
                                steps_commanded_2,
                                sec2nano(sim_time_1 + sim_time_extra),
                                sec2nano(sim_time_1 + sim_time_extra + sim_time_2 + sim_time_extra),
                            );

                            let final_1 = (sim_time_1 / 0.01).round() as usize;
                            let theta_ref_1 =
                                theta_init + f64::from(steps_commanded_1) * step_angle;
                            assert_close(R2D * log[final_1].theta_rad, R2D * theta_ref_1, &what);
                            assert_close(R2D * log[final_1].theta_dot_radps, 0.0, &what);
                            assert_close(R2D * log[final_1 + 1].theta_ddot_radps2, 0.0, &what);
                            assert_eq!(log[final_1].step_count, steps_commanded_1, "{what}");

                            let final_2 = ((sim_time_1 + sim_time_extra + sim_time_2) / 0.01)
                                .round() as usize;
                            let theta_ref_2 =
                                theta_ref_1 + f64::from(steps_commanded_2) * step_angle;
                            assert_close(R2D * log[final_2].theta_rad, R2D * theta_ref_2, &what);
                            assert_close(R2D * log[final_2].theta_dot_radps, 0.0, &what);
                            assert_close(R2D * log[final_2 + 1].theta_ddot_radps2, 0.0, &what);
                            assert_eq!(log[final_2 + 1].step_count, steps_commanded_2, "{what}");
                        }
                    }
                }
            }
        }
    }

    /// Upstream `test_stepper_motor_interrupt` over its grid: the second
    /// command arrives a fraction of a step after half the first command's
    /// steps; the step in progress completes before the second command runs.
    #[test]
    fn an_interrupting_command_runs_after_the_step_in_progress() {
        let step_time = 1.0;
        let step_angle = 1.0 * D2R;
        for steps_commanded_1 in [10_i32, -16, -20] {
            for steps_commanded_2 in [-10_i32, -16, 20] {
                for interrupt_fraction in [0.0, 0.25, 0.5, 0.75, 1.0] {
                    let what = format!(
                        "steps {steps_commanded_1} then {steps_commanded_2}, interrupt \
                         fraction {interrupt_fraction}"
                    );
                    let sim_time_1_no_interrupt = step_time * f64::from(steps_commanded_1.abs());
                    let sim_time_1_interrupt =
                        0.5 * sim_time_1_no_interrupt + interrupt_fraction * step_time;
                    let sim_time_2 = step_time * f64::from(steps_commanded_2.abs());
                    let sim_time_extra = 5.0;
                    let log = run_two_commands(
                        StepperMotorConfig {
                            name: "StepperMotor".to_string(),
                            theta_init_rad: 0.0,
                            step_angle_rad: step_angle,
                            step_time_s: step_time,
                        },
                        steps_commanded_1,
                        steps_commanded_2,
                        sec2nano(sim_time_1_interrupt),
                        sec2nano(sim_time_1_interrupt + sim_time_2 + sim_time_extra),
                    );

                    let mut steps_taken_1 = (sim_time_1_interrupt / step_time).ceil() as i32;
                    if steps_commanded_1 < 0 {
                        steps_taken_1 = -steps_taken_1;
                    }
                    let sim_time_1 = (step_time * f64::from(steps_taken_1)).abs();

                    let final_1 = (sim_time_1 / 0.01).round() as usize;
                    let theta_ref_1 = step_angle * f64::from(steps_taken_1);
                    assert_close(R2D * log[final_1].theta_rad, R2D * theta_ref_1, &what);
                    assert_close(R2D * log[final_1].theta_dot_radps, 0.0, &what);
                    assert_eq!(log[final_1].step_count, steps_taken_1, "{what}");

                    let final_2 = ((sim_time_1 + sim_time_2) / 0.01).round() as usize;
                    let theta_ref_2 = theta_ref_1 + step_angle * f64::from(steps_commanded_2);
                    assert_close(R2D * log[final_2].theta_rad, R2D * theta_ref_2, &what);
                    assert_close(R2D * log[final_2].theta_dot_radps, 0.0, &what);
                    assert_close(R2D * log[final_2 + 1].theta_ddot_radps2, 0.0, &what);
                    assert_eq!(log[final_2].step_count, steps_commanded_2, "{what}");
                }
            }
        }
    }

    /// One step in closed form, and a repeated command (same stamp) taking
    /// no effect.
    #[test]
    fn a_step_is_a_bang_bang_profile_and_a_repeat_is_ignored() {
        let config = StepperMotorConfig {
            name: "StepperMotor".to_string(),
            theta_init_rad: 0.0,
            step_angle_rad: 0.02,
            step_time_s: 1.0,
        };
        let theta_ddot_max = config.theta_ddot_max_radps2();
        assert!((theta_ddot_max - 0.08).abs() < 1e-15);
        let command_out = Output::new(MotorStepCommandMsg {
            steps_commanded: 1,
            command_time_nanos: 0,
        });
        let mut motor = StepperMotor::new(config);
        command_out.connect_to(&mut motor.motor_step_command_in_msg);
        motor.init();

        motor.update(&context(0));
        motor.update(&context(250_000_000));
        let quarter = motor.stepper_motor_out_msg.read();
        assert!((quarter.theta_rad - 0.5 * theta_ddot_max * 0.25 * 0.25).abs() < 1e-15);
        assert!((quarter.theta_dot_radps - theta_ddot_max * 0.25).abs() < 1e-15);
        assert_eq!(quarter.theta_ddot_radps2, theta_ddot_max);
        motor.update(&context(750_000_000));
        let three_quarters = motor.stepper_motor_out_msg.read();
        assert!((three_quarters.theta_rad - (0.02 - 0.5 * theta_ddot_max * 0.0625)).abs() < 1e-15);
        assert_eq!(three_quarters.theta_ddot_radps2, -theta_ddot_max);
        motor.update(&context(1_000_000_000));
        let done = motor.stepper_motor_out_msg.read();
        assert_eq!(
            (done.theta_rad, done.theta_dot_radps, done.step_count),
            (0.02, 0.0, 1)
        );

        // Same stamp: still the old command, so the motor stays put.
        command_out.write(MotorStepCommandMsg {
            steps_commanded: 1,
            command_time_nanos: 0,
        });
        motor.update(&context(2_000_000_000));
        motor.update(&context(2_500_000_000));
        assert_eq!(motor.stepper_motor_out_msg.read().theta_rad, 0.02);
    }

    #[test]
    #[should_panic(expected = "step_time_s must be positive")]
    fn rejects_a_non_positive_step_time() {
        let _ = StepperMotor::new(StepperMotorConfig {
            name: "StepperMotor".to_string(),
            theta_init_rad: 0.0,
            step_angle_rad: 0.01,
            step_time_s: 0.0,
        });
    }
}
