//! Pieces for the scenarios that propagate a MuJoCo scene (`mujoco`
//! feature): the scene with an initial body state applied after its
//! initialization, and NumPy's `interp` for comparing runs at different
//! steps.
//!
//! ```ignore
//! #[path = "support/mj_scene.rs"]
//! mod mj_scene;
//! ```

#![allow(dead_code)]

use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::{Module, SimulationContext};
use nalgebra::Vector3;

/// A scene whose body `body` is put at `position_m` and `velocity_mps`
/// right after `init`: upstream's `InitializeSimulation()` followed by
/// `body.setPosition(rN)` and `body.setVelocity(vN)`.
pub struct InitializedScene {
    pub scene: MjScene,
    pub body: String,
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
}

impl Module for InitializedScene {
    fn init(&mut self) {
        self.scene.init();
        let mut body = self.scene.body_mut(&self.body);
        body.set_position(self.position_m);
        body.set_velocity(self.velocity_mps);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.scene.reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.scene.update(context);
    }
}

/// `values` (sampled at the ascending `times`) linearly interpolated at
/// `time`: NumPy `interp`, holding the end values outside the samples.
pub fn interpolate(times: &[f64], values: &[Vector3<f64>], time: f64) -> Vector3<f64> {
    let upper = times.partition_point(|sample| *sample < time);
    if upper == 0 {
        return values[0];
    }
    if upper == times.len() {
        return values[times.len() - 1];
    }
    let (t0, t1) = (times[upper - 1], times[upper]);
    let fraction = (time - t0) / (t1 - t0);
    values[upper - 1] + (values[upper] - values[upper - 1]) * fraction
}
