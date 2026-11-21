//! Joint state of a multibody scene (upstream C++ `MJSceneStateMsgPayload`).

use serde::{Deserialize, Serialize};

/// Generalized positions, velocities and actuator activations of a MuJoCo
/// scene: upstream `MJSceneStateMsg`.
///
/// The vectors follow MuJoCo's `qpos`, `qvel` and `act` layouts. Upstream's
/// payload holds `Eigen::VectorXd`s, so the length is the scene's and the
/// message has no telemetry columns.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct MjSceneStateMsg {
    /// Upstream `qpos` [-]: joint positions, \[m\] or \[rad\], with a
    /// scalar-first quaternion for each free and ball joint.
    pub qpos: Vec<f64>,
    /// Upstream `qvel` [-]: joint velocities, \[m/s\] or \[rad/s\].
    pub qvel: Vec<f64>,
    /// Upstream `act` [-]: actuator activations.
    pub act: Vec<f64>,
}
