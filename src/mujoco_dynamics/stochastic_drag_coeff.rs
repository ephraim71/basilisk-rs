//! Stochastic drag coefficient: port of upstream Basilisk
//! `simulation/mujocoDynamics/stochasticDragCoeff`.
//!
//! A [`MeanRevertingNoise`] state `x` scales the nominal drag coefficient,
//! `C_D = C_D,nominal (1 + x)`; the rest of the drag geometry passes
//! through. The model runs in a scene's dynamics task.

use super::noise::MeanRevertingNoise;
use super::task::{ModelStateSpec, SceneModel, TaskContext};
use crate::messages::{DragGeometryMsg, Input, Output};

/// Upstream `StochasticDragCoeff`.
#[derive(Debug)]
pub struct StochasticDragCoeff {
    /// Upstream `ModelTag`.
    pub name: String,
    /// The coefficient correction process (upstream's `MeanRevertingNoise`
    /// base).
    pub noise: MeanRevertingNoise,
    /// Upstream `dragGeomInMsg`: the nominal drag geometry.
    pub drag_geom_in_msg: Input<DragGeometryMsg>,
    /// Upstream `dragGeomOutMsg`: the geometry with the corrected
    /// coefficient.
    pub drag_geom_out_msg: Output<DragGeometryMsg>,
}

impl Default for StochasticDragCoeff {
    fn default() -> Self {
        Self {
            name: "stochasticDragCoeff".to_string(),
            noise: MeanRevertingNoise::default(),
            drag_geom_in_msg: Input::default(),
            drag_geom_out_msg: Output::default(),
        }
    }
}

impl SceneModel for StochasticDragCoeff {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        vec![self.noise.state_spec()]
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.noise.load_state(states[0]);
    }

    /// Upstream `UpdateState` and `writeOutput`.
    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let x = self.noise.evaluate(context, 0);
        let mut out = self.drag_geom_in_msg.read();
        out.drag_coeff *= 1.0 + x;
        self.drag_geom_out_msg.write(out);
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_stochasticDragCoeff.py`, with basilisk-rs's
    //! seeded Wiener increments; the statistics and tolerances are
    //! upstream's.

    use super::*;
    use crate::Module;
    use crate::integrators::stochastic::EulerMaruyama;
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::tests::{ornstein_uhlenbeck_statistics, run, sec2nano};

    /// Upstream `test_stochasticDragCoeff`.
    #[test]
    fn drag_coefficient_follows_ornstein_uhlenbeck_statistics() {
        let (dt, tf) = (0.002, 100.0);
        let (nominal_cd, sigma_st, tau) = (2.2, 0.4, 0.1);

        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        scene.set_integrator(EulerMaruyama::new().with_seed(0));
        let nominal = Output::new(DragGeometryMsg {
            drag_coeff: nominal_cd,
            ..DragGeometryMsg::default()
        });
        let mut model = StochasticDragCoeff::default();
        model.noise.set_stationary_std(sigma_st);
        model.noise.set_time_constant(tau);
        nominal.connect_to(&mut model.drag_geom_in_msg);
        let out = model.drag_geom_out_msg.clone();
        let id = scene.add_model_to_dynamics_task(model, -1);
        scene.init();
        assert_eq!(scene.task_model(id).noise.time_constant(), tau);
        assert_eq!(scene.task_model(id).noise.stationary_std(), sigma_st);

        let mut cd = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(tf), |_, _| {
            cd.push(out.read().drag_coeff);
        });

        let stats = ornstein_uhlenbeck_statistics(&cd, dt);
        assert!((stats.mean - nominal_cd).abs() <= 0.2 * nominal_cd);
        let variance_target = (nominal_cd * sigma_st).powi(2);
        assert!((stats.variance - variance_target).abs() <= 0.3 * variance_target);
        let phi_target = (-dt / tau).exp();
        assert!((stats.phi - phi_target).abs() <= 0.003 + 0.03 * phi_target);
        assert!((stats.time_constant - tau).abs() <= 0.1 + 0.05 * tau);
    }
}
