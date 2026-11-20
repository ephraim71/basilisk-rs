//! Stochastic atmospheric density: port of upstream Basilisk
//! `simulation/mujocoDynamics/stochasticAtmDensity`.
//!
//! A [`MeanRevertingNoise`] state `x` scales the nominal neutral density,
//! `ρ = ρ_nominal (1 + x)`; the other atmosphere fields pass through. The
//! model runs in a scene's dynamics task, which integrates `x` (with a
//! stochastic integrator for the noise to act).

use super::noise::MeanRevertingNoise;
use super::task::{ModelStateSpec, SceneModel, TaskContext};
use crate::messages::{AtmosphereMsg, Input, Output};

/// Upstream `StochasticAtmDensity`.
#[derive(Debug)]
pub struct StochasticAtmDensity {
    /// Upstream `ModelTag`.
    pub name: String,
    /// The density correction process (upstream's `MeanRevertingNoise`
    /// base: `setStationaryStd`, `setTimeConstant`, `getStateValue`...).
    pub noise: MeanRevertingNoise,
    /// Upstream `atmoDensInMsg`: the nominal atmosphere.
    pub atmo_dens_in_msg: Input<AtmosphereMsg>,
    /// Upstream `atmoDensOutMsg`: the atmosphere with the corrected density.
    pub atmo_dens_out_msg: Output<AtmosphereMsg>,
}

impl Default for StochasticAtmDensity {
    fn default() -> Self {
        Self {
            name: "stochasticAtmDensity".to_string(),
            noise: MeanRevertingNoise::default(),
            atmo_dens_in_msg: Input::default(),
            atmo_dens_out_msg: Output::default(),
        }
    }
}

impl SceneModel for StochasticAtmDensity {
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
        let mut out = self.atmo_dens_in_msg.read();
        out.neutral_density_kgpm3 *= 1.0 + x;
        self.atmo_dens_out_msg.write(out);
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_stochasticAtmDensity.py`. basilisk-rs's seeded
    //! Wiener increments replace upstream's Mersenne Twister, so the path
    //! differs; the statistics and tolerances are upstream's. Upstream's
    //! Python-subclass variant is the same model.

    use super::*;
    use crate::Module;
    use crate::integrators::stochastic::EulerMaruyama;
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::tests::{ornstein_uhlenbeck_statistics, run, sec2nano};

    /// Upstream `test_stochasticAtmDensity`: the density follows the OU
    /// statistics (mean, variance, lag-one correlation and time constant).
    #[test]
    fn density_follows_ornstein_uhlenbeck_statistics() {
        let (dt, tf) = (0.002, 100.0);
        let (sigma_st, tau) = (0.8, 0.1);

        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        scene.set_integrator(EulerMaruyama::new().with_seed(0));
        let nominal = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: 1.0,
            ..AtmosphereMsg::default()
        });
        let mut model = StochasticAtmDensity::default();
        model.noise.set_stationary_std(sigma_st);
        model.noise.set_time_constant(tau);
        nominal.connect_to(&mut model.atmo_dens_in_msg);
        let out = model.atmo_dens_out_msg.clone();
        let id = scene.add_model_to_dynamics_task(model, -1);
        scene.init();
        assert_eq!(scene.task_model(id).noise.time_constant(), tau);
        assert_eq!(scene.task_model(id).noise.stationary_std(), sigma_st);

        let mut density = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(tf), |_, _| {
            density.push(out.read().neutral_density_kgpm3);
        });

        let stats = ornstein_uhlenbeck_statistics(&density, dt);
        assert!((stats.mean - 1.0).abs() <= 0.2, "mean {}", stats.mean);
        let variance_target = sigma_st * sigma_st;
        assert!((stats.variance - variance_target).abs() <= 0.3 * variance_target);
        let phi_target = (-dt / tau).exp();
        assert!((stats.phi - phi_target).abs() <= 0.003 + 0.03 * phi_target);
        assert!((stats.time_constant - tau).abs() <= 0.1 + 0.05 * tau);
    }
}
