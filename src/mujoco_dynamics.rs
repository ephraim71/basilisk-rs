//! MuJoCo-based multibody dynamics: port of upstream Basilisk
//! `simulation/mujocoDynamics` (and the `forceTorque` models that plug into
//! it), behind the `mujoco` cargo feature.
//!
//! [`MjScene`] is a dynamic object solved by MuJoCo, integrated by the same
//! basilisk-rs [`Integrator`](crate::integrators::Integrator)s as a
//! spacecraft. Models that compute forces, controls or noise states from
//! the scene's state run in its dynamics task ([`SceneModel`]); the analysis
//! modules ([`mj_system_com`], [`mj_system_mass_matrix`],
//! [`mj_joint_reaction_forces`]) read the scene from the simulation's own
//! schedule through an [`MjSceneHandle`].
//!
//! The MuJoCo library comes through the [`mujoco_rs`] crate, re-exported
//! here for models that read the MuJoCo data directly.

mod actuator;
mod body;
pub mod cannonball_drag;
pub mod cmd_force_inertial_to_force_at_site;
pub mod igbm_atm_density;
pub mod interpolators;
pub mod joint_pid_controller;
pub mod linear_time_invariant_system;
pub mod mj_joint_reaction_forces;
pub mod mj_system_com;
pub mod mj_system_mass_matrix;
pub mod n_body_gravity;
pub mod noise;
pub mod orbital_element_control;
mod scene;
mod spec;
mod state;
pub mod stochastic_atm_density;
pub mod stochastic_drag_coeff;
mod task;
pub mod thr_on_time_to_force;
pub mod two_hinge_damper;

pub use mujoco_rs;

pub use actuator::{MjForceActuator, MjForceTorqueActuator, MjSingleActuator, MjTorqueActuator};
pub use body::{MjBallJoint, MjBody, MjFreeJoint, MjJointKind, MjScalarJoint, MjSite};
pub use scene::{BodyMut, BodyRef, MjEquality, MjGeomInfo, MjScene, MjSceneHandle, TaskModelId};
pub use spec::MjSceneError;
pub use state::{MjSceneDerivative, MjSceneState};
pub use task::{
    AsAny, MjSceneData, ModelStateSpec, ModuleTaskModel, SceneModel, SceneView, TaskContext,
};

#[cfg(test)]
mod tests;
