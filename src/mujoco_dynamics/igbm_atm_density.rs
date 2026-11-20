//! Atmospheric density with a multiplicative noise factor: port of upstream
//! Basilisk `simulation/mujocoDynamics/igbmAtmDensity`.
//!
//! An [`InhomogeneousGeometricBrownianMotion`] factor `X` scales the nominal
//! neutral density, `ρ = max(0, ρ_nominal X)`: the exact process stays
//! positive, but an explicit step can make `X ≤ 0` for a large increment, and
//! the clamp keeps the density physical. The other atmosphere fields pass
//! through. The model runs in a scene's dynamics task.

use super::noise::InhomogeneousGeometricBrownianMotion;
use super::task::{ModelStateSpec, SceneModel, TaskContext};
use crate::messages::{AtmosphereMsg, Input, Output};

/// Upstream `IgbmAtmDensity`.
#[derive(Debug)]
pub struct IgbmAtmDensity {
    /// Upstream `ModelTag`.
    pub name: String,
    /// The density factor process (upstream's
    /// `InhomogeneousGeometricBrownianMotion` base: `setMean`,
    /// `setStationaryStd`, `setTimeConstant`...).
    pub igbm: InhomogeneousGeometricBrownianMotion,
    /// Upstream `atmoDensInMsg`: the nominal atmosphere.
    pub atmo_dens_in_msg: Input<AtmosphereMsg>,
    /// Upstream `atmoDensOutMsg`: the atmosphere with the scaled density.
    pub atmo_dens_out_msg: Output<AtmosphereMsg>,
}

impl Default for IgbmAtmDensity {
    fn default() -> Self {
        Self {
            name: "igbmAtmDensity".to_string(),
            igbm: InhomogeneousGeometricBrownianMotion::default(),
            atmo_dens_in_msg: Input::default(),
            atmo_dens_out_msg: Output::default(),
        }
    }
}

impl SceneModel for IgbmAtmDensity {
    fn model_tag(&self) -> &str {
        &self.name
    }

    fn state_specs(&self) -> Vec<ModelStateSpec> {
        vec![self.igbm.state_spec()]
    }

    fn load_states(&mut self, states: &[&[f64]]) {
        self.igbm.load_state(states[0]);
    }

    /// Upstream `UpdateState` and `writeOutput`.
    fn scene_update(&mut self, context: &mut TaskContext<'_>) {
        let x = self.igbm.evaluate(context, 0);
        let mut out = self.atmo_dens_in_msg.read();
        out.neutral_density_kgpm3 = (out.neutral_density_kgpm3 * x).max(0.0);
        self.atmo_dens_out_msg.write(out);
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `test_igbmAtmDensity.py`, with basilisk-rs's seeded
    //! Wiener increments (the prescribed-increment test uses upstream's
    //! sequence); the statistics and tolerances are upstream's.

    use std::sync::{Arc, Mutex};

    use super::*;
    use crate::Module;
    use crate::integrators::stochastic::{EulerMaruyama, PrescribedGaussianNoiseGenerator};
    use crate::mujoco_dynamics::MjScene;
    use crate::mujoco_dynamics::tests::{run, sec2nano};

    fn scene_with(
        model: IgbmAtmDensity,
        integrator: EulerMaruyama,
    ) -> (MjScene, Output<AtmosphereMsg>) {
        let mut scene = MjScene::new("<mujoco/>").expect("scene");
        scene.set_integrator(integrator);
        let out = model.atmo_dens_out_msg.clone();
        scene.add_model_to_dynamics_task(model, -1);
        (scene, out)
    }

    /// Upstream `test_igbmAtmDensity`: the density has the stationary mean
    /// and variance of the factor and stays non-negative.
    #[test]
    fn density_follows_igbm_statistics() {
        let (dt, tf) = (0.001, 400.0);
        let (mean, tau, sigma_st) = (1.0, 0.2, 1.0 / 3.0);
        let nominal = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: 1.0,
            ..AtmosphereMsg::default()
        });
        let mut model = IgbmAtmDensity::default();
        model.igbm.set_mean(mean);
        model.igbm.set_time_constant(tau);
        model.igbm.set_stationary_std(sigma_st);
        model.igbm.set_state_value(mean);
        nominal.connect_to(&mut model.atmo_dens_in_msg);
        let (mut scene, out) = scene_with(model, EulerMaruyama::new().with_seed(0));
        scene.init();

        let mut density = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(tf), |_, _| {
            density.push(out.read().neutral_density_kgpm3);
        });
        let burn = (5.0 * tau / dt).round() as usize;
        let stationary = &density[burn..];
        let n = stationary.len() as f64;
        let sample_mean = stationary.iter().sum::<f64>() / n;
        let variance = stationary
            .iter()
            .map(|x| (x - sample_mean).powi(2))
            .sum::<f64>()
            / (n - 1.0);
        assert!(stationary.iter().all(|rho| *rho >= 0.0));
        assert!(
            (sample_mean - mean).abs() <= 0.02 + 0.05 * mean,
            "mean {sample_mean}"
        );
        let target = sigma_st * sigma_st;
        assert!(
            (variance - target).abs() <= 0.2 * target,
            "variance {variance}"
        );
    }

    /// Upstream `test_igbmAtmDensity_passesThroughOtherFields`.
    #[test]
    fn passes_through_other_fields() {
        let dt = 0.01;
        let local_temp = 933.7;
        let nominal = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: 2.5,
            local_temp_k: local_temp,
        });
        let mut model = IgbmAtmDensity::default();
        model.igbm.set_mean(1.0);
        model.igbm.set_time_constant(1.0);
        model.igbm.set_stationary_std(0.2);
        nominal.connect_to(&mut model.atmo_dens_in_msg);
        let (mut scene, out) = scene_with(model, EulerMaruyama::new().with_seed(0));
        scene.init();
        let mut samples = Vec::new();
        run(&mut scene, sec2nano(dt), sec2nano(10.0 * dt), |_, _| {
            samples.push(out.read())
        });
        for sample in &samples[1..] {
            assert_eq!(sample.local_temp_k, local_temp);
            assert!(sample.neutral_density_kgpm3 > 0.0);
        }
    }

    /// Upstream `test_igbmAtmDensity_clampsNegativeDensity`: prescribed
    /// −4σ increments at a 1 s step drive the factor negative, and the
    /// density clamps at zero.
    #[test]
    fn clamps_negative_density() {
        let steps = 5;
        let mut prescribed = PrescribedGaussianNoiseGenerator::new();
        for _ in 0..steps {
            prescribed.push_step(&[-4.0], &[]);
        }
        let nominal = Output::new(AtmosphereMsg {
            neutral_density_kgpm3: 1.0,
            ..AtmosphereMsg::default()
        });
        let mut model = IgbmAtmDensity::default();
        model.igbm.set_mean(1.0);
        model.igbm.set_time_constant(1.0);
        model.igbm.set_stationary_std(1.0);
        nominal.connect_to(&mut model.atmo_dens_in_msg);
        let integrator =
            EulerMaruyama::new().with_noise_generator(Arc::new(Mutex::new(prescribed)));
        let (mut scene, out) = scene_with(model, integrator);
        scene.init();
        let mut density = Vec::new();
        run(&mut scene, sec2nano(1.0), sec2nano(steps as f64), |_, _| {
            density.push(out.read().neutral_density_kgpm3);
        });
        assert!(density.iter().all(|rho| *rho >= 0.0), "{density:?}");
        assert!(density.contains(&0.0), "{density:?}");
    }

    /// Upstream `test_igbmAtmDensity_rejectsBadParameters`.
    #[test]
    fn rejects_bad_parameters() {
        for bad in [0.0, -1.0] {
            assert!(
                std::panic::catch_unwind(|| IgbmAtmDensity::default().igbm.set_mean(bad)).is_err()
            );
        }
        for bad in [0.0, -0.1] {
            assert!(
                std::panic::catch_unwind(|| IgbmAtmDensity::default().igbm.set_time_constant(bad))
                    .is_err()
            );
        }
        assert!(
            std::panic::catch_unwind(|| IgbmAtmDensity::default().igbm.set_stationary_std(-0.01))
                .is_err()
        );
    }
}
