//! Damping of a two-hinge pendulum: port of upstream Basilisk
//! `simulation/forceTorque/twoHingeDamper`.
//!
//! A rod of length `L` hangs from two hinges, `φ` then `θ`, and its bob is
//! damped isotropically in Cartesian space with coefficient `c_cart`. With
//! `c = c_cart L²`, the generalized damping torques are
//!
//! ```text
//! Q_φ = -c cos²θ φ̇,    Q_θ = -c θ̇.
//! ```
//!
//! The model reads the joint states of a scene and drives one joint motor
//! per hinge; [`TwoHingeDamper::apply_to`] wires it (upstream's Python
//! `applyTo`). It is a [`Module`] and, to damp at every integrator stage, a
//! [`SceneModel`].

use super::scene::MjScene;
use super::task::{SceneModel, SceneView, TaskContext};
use crate::messages::{Input, Output, ScalarJointStateMsg, SingleActuatorMsg};
use crate::{Module, SimulationContext};

/// Upstream `TwoHingeDamper`.
#[derive(Debug)]
pub struct TwoHingeDamper {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `dampingCoeff` \[N m s\]: the Cartesian damping coefficient
    /// times the rod length squared.
    pub damping_coeff: f64,
    /// Upstream `thetaInMsg` \[rad\]: the `θ` joint angle.
    pub theta_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `phiDotInMsg` \[rad/s\].
    pub phi_dot_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `thetaDotInMsg` \[rad/s\].
    pub theta_dot_in_msg: Input<ScalarJointStateMsg>,
    /// Upstream `phiTorqueOutMsg` \[N m\].
    pub phi_torque_out_msg: Output<SingleActuatorMsg>,
    /// Upstream `thetaTorqueOutMsg` \[N m\].
    pub theta_torque_out_msg: Output<SingleActuatorMsg>,
}

impl Default for TwoHingeDamper {
    fn default() -> Self {
        Self {
            name: "twoHingeDamper".to_string(),
            damping_coeff: 0.0,
            theta_in_msg: Input::default(),
            phi_dot_in_msg: Input::default(),
            theta_dot_in_msg: Input::default(),
            phi_torque_out_msg: Output::default(),
            theta_torque_out_msg: Output::default(),
        }
    }
}

impl TwoHingeDamper {
    /// Upstream's Python `applyTo(phiJoint, thetaJoint)`: reads the two
    /// hinges of `scene` and drives a new motor on each (named
    /// `<name>_<joint>`).
    ///
    /// # Errors
    ///
    /// If a joint is not a hinge, or both are the same joint; nothing is
    /// added then.
    ///
    /// # Panics
    ///
    /// If the scene has no such scalar joint.
    pub fn apply_to(
        &mut self,
        scene: &mut MjScene,
        phi_joint: &str,
        theta_joint: &str,
    ) -> Result<(), String> {
        for (label, joint) in [("phiJoint", phi_joint), ("thetaJoint", theta_joint)] {
            if !scene.scalar_joint(joint).is_hinge() {
                return Err(format!(
                    "TwoHingeDamper.applyTo: {label} must be a hinge joint."
                ));
            }
        }
        if phi_joint == theta_joint {
            return Err(
                "TwoHingeDamper.applyTo: phiJoint and thetaJoint must be distinct hinge joints."
                    .to_string(),
            );
        }
        let phi = scene.scalar_joint(phi_joint).clone();
        let theta = scene.scalar_joint(theta_joint).clone();
        theta.state_out_msg.connect_to(&mut self.theta_in_msg);
        phi.state_dot_out_msg.connect_to(&mut self.phi_dot_in_msg);
        theta
            .state_dot_out_msg
            .connect_to(&mut self.theta_dot_in_msg);
        let tag = if self.name.is_empty() {
            "twoHingeDamper"
        } else {
            &self.name
        };
        self.phi_torque_out_msg.connect_to(
            &mut scene
                .add_joint_single_actuator(&format!("{tag}_{phi_joint}"), phi_joint)
                .actuator_in_msg,
        );
        self.theta_torque_out_msg.connect_to(
            &mut scene
                .add_joint_single_actuator(&format!("{tag}_{theta_joint}"), theta_joint)
                .actuator_in_msg,
        );
        Ok(())
    }

    /// Upstream `Reset`.
    ///
    /// # Panics
    ///
    /// If an input is unconnected or the damping coefficient is negative or
    /// not finite (upstream raises `BasiliskError`).
    fn check(&self) {
        for (input, name) in [
            (&self.theta_in_msg, "thetaInMsg"),
            (&self.phi_dot_in_msg, "phiDotInMsg"),
            (&self.theta_dot_in_msg, "thetaDotInMsg"),
        ] {
            assert!(input.is_connected(), "TwoHingeDamper.{name} is not linked.");
        }
        assert!(
            self.damping_coeff.is_finite() && self.damping_coeff >= 0.0,
            "TwoHingeDamper.dampingCoeff must be finite and nonnegative."
        );
    }

    /// Upstream `UpdateState`.
    fn damp(&self) {
        let cos_theta = self.theta_in_msg.read().state.cos();
        self.phi_torque_out_msg.write(SingleActuatorMsg {
            input: -self.damping_coeff * cos_theta * cos_theta * self.phi_dot_in_msg.read().state,
        });
        self.theta_torque_out_msg.write(SingleActuatorMsg {
            input: -self.damping_coeff * self.theta_dot_in_msg.read().state,
        });
    }
}

impl Module for TwoHingeDamper {
    fn init(&mut self) {
        self.check();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.damp();
    }
}

impl SceneModel for TwoHingeDamper {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn scene_reset(&mut self, _scene: &SceneView<'_>, _context: &SimulationContext) {
        self.check();
    }

    fn scene_update(&mut self, _context: &mut TaskContext<'_>) {
        self.damp();
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_twoHingeDamper.py`.

    use std::f64::consts::PI;

    use super::*;
    use crate::mujoco_dynamics::tests::context;

    fn joint_state(value: f64) -> Output<ScalarJointStateMsg> {
        Output::new(ScalarJointStateMsg { state: value })
    }

    /// Upstream `test_two_hinge_damper_cartesian_law`.
    #[test]
    fn generalized_torques_follow_the_cartesian_law() {
        for (theta, phi_dot, theta_dot, coeff) in [
            (0.6, 0.4, -0.3, 2.5),
            (0.0, -0.2, 0.5, 1.0),
            (0.5 * PI, 0.4, 0.3, 2.0),
            (0.3, 0.0, 0.0, 0.0),
        ] {
            let mut damper = TwoHingeDamper {
                damping_coeff: coeff,
                ..TwoHingeDamper::default()
            };
            let (theta_msg, phi_dot_msg, theta_dot_msg) = (
                joint_state(theta),
                joint_state(phi_dot),
                joint_state(theta_dot),
            );
            theta_msg.connect_to(&mut damper.theta_in_msg);
            phi_dot_msg.connect_to(&mut damper.phi_dot_in_msg);
            theta_dot_msg.connect_to(&mut damper.theta_dot_in_msg);
            Module::init(&mut damper);
            Module::update(&mut damper, &context(100_000_000));
            let phi = damper.phi_torque_out_msg.read().input;
            let expected_phi = -coeff * theta.cos().powi(2) * phi_dot;
            assert!((phi - expected_phi).abs() <= 1e-6 * expected_phi.abs().max(1e-6));
            let expected_theta = -coeff * theta_dot;
            assert!((damper.theta_torque_out_msg.read().input - expected_theta).abs() <= 1e-12);
        }
    }

    /// Upstream `test_two_hinge_damper_reset_validation`.
    #[test]
    fn reset_rejects_missing_inputs_and_invalid_coefficients() {
        for (missing, coeff, expected) in [
            (Some("thetaInMsg"), 1.0, "thetaInMsg is not linked"),
            (Some("phiDotInMsg"), 1.0, "phiDotInMsg is not linked"),
            (Some("thetaDotInMsg"), 1.0, "thetaDotInMsg is not linked"),
            (None, -1.0, "dampingCoeff must be finite and nonnegative"),
            (
                None,
                f64::NAN,
                "dampingCoeff must be finite and nonnegative",
            ),
        ] {
            let mut damper = TwoHingeDamper {
                damping_coeff: coeff,
                ..TwoHingeDamper::default()
            };
            let message = joint_state(0.0);
            if missing != Some("thetaInMsg") {
                message.connect_to(&mut damper.theta_in_msg);
            }
            if missing != Some("phiDotInMsg") {
                message.connect_to(&mut damper.phi_dot_in_msg);
            }
            if missing != Some("thetaDotInMsg") {
                message.connect_to(&mut damper.theta_dot_in_msg);
            }
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                Module::init(&mut damper);
            }));
            let error = result.expect_err("reset must fail");
            let text = error
                .downcast_ref::<String>()
                .cloned()
                .or_else(|| error.downcast_ref::<&str>().map(|text| text.to_string()))
                .unwrap_or_default();
            assert!(text.contains(expected), "{text}");
        }
    }

    fn pendulum_scene(phi: &str, theta: &str) -> MjScene {
        MjScene::new(&format!(
            r#"
            <mujoco>
              <worldbody>
                <body name="pendulum">
                  <joint name="phi" type="{phi}" axis="1 0 0"/>
                  <joint name="theta" type="{theta}" axis="0 1 0"/>
                  <geom type="sphere" size="0.1" mass="1"/>
                  <body name="ballBody">
                    <joint name="ball" type="ball"/>
                    <geom type="sphere" size="0.1" mass="1"/>
                  </body>
                </body>
              </worldbody>
            </mujoco>
            "#
        ))
        .expect("scene")
    }

    /// Upstream `test_two_hinge_damper_apply_to_enforces_joint_contract`
    /// (the ball-joint and two-scene cases cannot be expressed: the joints
    /// are named in one scene and must be scalar joints).
    #[test]
    fn apply_to_enforces_the_joint_contract() {
        let mut scene = pendulum_scene("hinge", "hinge");
        let error = TwoHingeDamper::default()
            .apply_to(&mut scene, "phi", "phi")
            .expect_err("same joint");
        assert!(error.contains("distinct hinge joints"));
        let actuators_before = scene.actuator_names().len();
        TwoHingeDamper::default()
            .apply_to(&mut scene, "phi", "theta")
            .expect("valid hinges");
        assert_eq!(scene.actuator_names().len(), actuators_before + 2);

        for (phi, theta, expected) in [
            ("slide", "hinge", "phiJoint"),
            ("hinge", "slide", "thetaJoint"),
        ] {
            let mut scene = pendulum_scene(phi, theta);
            let error = TwoHingeDamper::default()
                .apply_to(&mut scene, "phi", "theta")
                .expect_err("a slide");
            assert!(
                error.contains(&format!("{expected} must be a hinge")),
                "{error}"
            );
        }
    }

    /// The damper wired to a scene slows the pendulum down.
    #[test]
    fn damps_a_scene_pendulum() {
        let mut scene = pendulum_scene("hinge", "hinge");
        let mut damper = TwoHingeDamper {
            damping_coeff: 0.5,
            ..TwoHingeDamper::default()
        };
        damper.apply_to(&mut scene, "phi", "theta").expect("hinges");
        scene.add_model_to_dynamics_task(damper, -1);
        scene.init();
        scene
            .body_mut("pendulum")
            .set_scalar_joint_velocity("phi", 1.0);
        scene
            .body_mut("pendulum")
            .set_scalar_joint_velocity("theta", 1.0);
        crate::mujoco_dynamics::tests::run(&mut scene, 10_000_000, 2_000_000_000, |_, _| {});
        let pendulum = scene.body("pendulum");
        assert!(pendulum.scalar_joint_velocity("phi").abs() < 1.0);
        assert!(pendulum.scalar_joint_velocity("theta").abs() < 1.0);
    }
}
