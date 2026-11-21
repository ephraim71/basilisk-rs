//! PID control of a scalar joint: port of upstream Basilisk
//! `simulation/mujocoDynamics/JointPIDController` (and its
//! `PIDController` template in `_GeneralModuleFiles`).
//!
//! The output command is
//!
//! ```text
//! u = Kp (θ_desired - θ) + Kd (θ̇_desired - θ̇) + Ki e_I
//! ```
//!
//! from the joint state and rate (a scene joint's `state_out_msg` and
//! `state_dot_out_msg`) and the desired state and rate, typically into the
//! joint's motor (an actuator from
//! [`MjScene::add_joint_single_actuator`](super::MjScene::add_joint_single_actuator)).
//! The model runs in a scene's dynamics task. Inputs are only read for the
//! terms with a nonzero gain.
//!
//! The integral error `e_I` is a state of the model, registered as
//! `integralError` and zero at every reset. Upstream never sets its
//! derivative, so it keeps the value it is given
//! ([`MjScene::set_task_model_state`](super::MjScene::set_task_model_state)
//! for upstream's `setIntegralError`); this port keeps that.

use super::task::{ModelStateSpec, SceneModel, TaskContext};
use crate::messages::{Input, Output, ScalarJointStateMsg, SingleActuatorMsg};

/// Upstream `JointPIDController`.
#[derive(Debug)]
pub struct JointPidController {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `measuredPosInMsg`: the joint angle \[rad\] or position \[m\].
    pub measured_pos_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `desiredPosInMsg`.
    pub desired_pos_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `measuredVelInMsg`: the joint rate.
    pub measured_vel_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `desiredVelInMsg`.
    pub desired_vel_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `outputOutMsg`: the actuator command.
    pub output_out_msg: Output<SingleActuatorMsg>,
    kp: f64,
    kd: f64,
    ki: f64,
    integral_error: f64,
}

impl Default for JointPidController {
    fn default() -> Self {
        Self {
            name: "jointPIDController".to_string(),
            measured_pos_in_msg: Input::default(),
            desired_pos_in_msg: Input::default(),
            measured_vel_in_msg: Input::default(),
            desired_vel_in_msg: Input::default(),
            output_out_msg: Output::default(),
            kp: 0.0,
            kd: 0.0,
            ki: 0.0,
            integral_error: 0.0,
        }
    }
}

impl JointPidController {
    /// Upstream `getProportionalGain`.
    pub fn proportional_gain(&self) -> f64 {
        self.kp
    }

    /// Upstream `setProportionalGain`.
    pub fn set_proportional_gain(&mut self, value: f64) {
        self.kp = value;
    }

    /// Upstream `getDerivativeGain`.
    pub fn derivative_gain(&self) -> f64 {
        self.kd
    }

    /// Upstream `setDerivativeGain`.
    pub fn set_derivative_gain(&mut self, value: f64) {
        self.kd = value;
    }

    /// Upstream `getIntegralGain`.
    pub fn integral_gain(&self) -> f64 {
        self.ki
    }

    /// Upstream `setIntegralGain`.
    pub fn set_integral_gain(&mut self, value: f64) {
        self.ki = value;
    }

    /// Upstream `getIntegralError`: the committed integral-error state.
    pub fn integral_error(&self) -> f64 {
        self.integral_error
    }

    fn require(&self, input: &Input<ScalarJointStateMsg>, input_name: &str, gain: &str) {
        assert!(
            input.is_connected(),
            "PIDController [{}]: {input_name} must be linked when {gain} is non-zero.",
            self.name
        );
    }
}

impl SceneModel for JointPidController {
    fn model_tag(&self) -> &str {
        &self.name
    }

    /// Upstream `registerStates`: `integralError`, set to zero.
    fn state_specs(&self) -> Vec<ModelStateSpec> {
        vec![ModelStateSpec::new("integralError", vec![0.0])]
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.integral_error = states[0][0];
    }

    /// Upstream `UpdateState`.
    ///
    /// # Panics
    ///
    /// If an input a nonzero gain needs is not connected (upstream raises
    /// `BasiliskError`).
    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let mut position_error = 0.0;
        if self.kp != 0.0 {
            self.require(&self.desired_pos_in_msg, "desiredPosInMsg", "Kp");
            self.require(&self.measured_pos_in_msg, "measuredPosInMsg", "Kp");
            position_error =
                self.desired_pos_in_msg.read().state - self.measured_pos_in_msg.read().state;
        }
        let mut velocity_error = 0.0;
        if self.kd != 0.0 {
            self.require(&self.desired_vel_in_msg, "desiredVelInMsg", "Kd");
            self.require(&self.measured_vel_in_msg, "measuredVelInMsg", "Kd");
            velocity_error =
                self.desired_vel_in_msg.read().state - self.measured_vel_in_msg.read().state;
        }
        let integral_error = context.state(0)[0];
        self.output_out_msg.write(SingleActuatorMsg {
            input: self.kp * position_error + self.kd * velocity_error + self.ki * integral_error,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_JointPIDController.py`.

    use std::f64::consts::FRAC_PI_2;

    use super::*;
    use crate::Module;
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::interpolators::ScalarJointStateInterpolator;
    use crate::mujoco_dynamics::tests::{run, sec2nano};

    /// Upstream `test_jointPIDController`: a hinged arm driven by the PID
    /// tracks a ramp to π/2 over 50 s, its torque is the PID law at every
    /// step, and it ends at the desired angle and rate.
    #[test]
    fn controller_tracks_a_ramp_with_the_pid_law() {
        let (dt, tf) = (0.1, 50.0);
        let desired_position = FRAC_PI_2;
        let desired_velocity = desired_position / tf;

        let mut scene = MjScene::new(
            r#"
            <mujoco>
                <option gravity="0 0 0"/>
                <worldbody>
                    <body name="arm">
                        <geom name="arm" type="capsule" size="0.1" fromto="0 0 0 0 0 1"/>
                        <joint name="arm" axis="1 0 0"/>
                    </body>
                </worldbody>
            </mujoco>
            "#,
        )
        .expect("scene");
        scene.extra_eom_call = true;
        let joint = scene.body("arm").scalar_joint("arm").clone();

        let mut desired = ScalarJointStateInterpolator::default();
        desired
            .set_data_points(&[(0.0, 0.0), (sec2nano(tf) as f64, desired_position)], 1)
            .expect("points");
        let desired_out = desired.interpolated_out_msg.clone();
        scene.add_model_to_dynamics_task(desired, -1);

        let mut controller = JointPidController::default();
        controller.set_proportional_gain(1.0);
        controller.set_derivative_gain(2.0);
        controller.set_integral_gain(0.02);
        let desired_velocity_msg = Output::new(ScalarJointStateMsg {
            state: desired_velocity,
        });
        joint
            .state_out_msg
            .connect_to(&mut controller.measured_pos_in_msg);
        joint
            .state_dot_out_msg
            .connect_to(&mut controller.measured_vel_in_msg);
        desired_out.connect_to(&mut controller.desired_pos_in_msg);
        desired_velocity_msg.connect_to(&mut controller.desired_vel_in_msg);
        let torque_out = controller.output_out_msg.clone();
        torque_out.connect_to(
            &mut scene
                .add_joint_single_actuator("arm", "arm")
                .actuator_in_msg,
        );
        let id = scene.add_model_to_dynamics_task(controller, -1);

        scene.init();
        let mut samples = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(tf), |scene, _| {
            samples.push((
                desired_out.read().state,
                joint.state_out_msg.read().state,
                joint.state_dot_out_msg.read().state,
                torque_out.read().input,
                scene.task_model(id).integral_error(),
            ));
        });

        let controller = scene.task_model(id);
        for &(desired, position, velocity, torque, integral) in &samples {
            let expected = controller.proportional_gain() * (desired - position)
                + controller.derivative_gain() * (desired_velocity - velocity)
                + controller.integral_gain() * integral;
            assert!((torque - expected).abs() <= 1e-6, "{torque} vs {expected}");
        }
        let &(_, position, velocity, _, _) = samples.last().expect("samples");
        assert!((position - desired_position).abs() <= 1e-2 * desired_position);
        assert!((velocity - desired_velocity).abs() <= 1e-2 * desired_velocity);
    }

    #[test]
    #[should_panic(expected = "desiredPosInMsg must be linked when Kp is non-zero")]
    fn proportional_gain_needs_the_position_inputs() {
        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        let mut controller = JointPidController::default();
        controller.set_proportional_gain(1.0);
        scene.add_model_to_dynamics_task(controller, -1);
        scene.init();
        scene.update(&crate::mujoco_dynamics::tests::context(0));
    }
}
