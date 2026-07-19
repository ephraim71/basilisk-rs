use crate::messages::{AtmosphereMsg, Input, Output, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct ExponentialAtmosphereConfig {
    pub name: String,
    pub planet_radius_m: f64,
    pub reference_altitude_m: f64,
    pub reference_density_kgpm3: f64,
    pub scale_height_m: f64,
}

pub struct ExponentialAtmosphere {
    pub config: ExponentialAtmosphereConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub output_atmosphere_msg: Output<AtmosphereMsg>,
}

impl Module for ExponentialAtmosphere {
    fn init(&mut self) {
        self.output_atmosphere_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.input_state_msg.read();
        let radius_m = state.position_m.norm();
        let altitude_m = (radius_m - self.config.planet_radius_m).max(0.0);
        let density = if self.config.scale_height_m > 0.0 {
            self.config.reference_density_kgpm3
                * (-(altitude_m - self.config.reference_altitude_m) / self.config.scale_height_m)
                    .exp()
        } else {
            0.0
        };

        self.output_atmosphere_msg.write(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
    }
}

impl ExponentialAtmosphere {
    pub fn new(config: ExponentialAtmosphereConfig) -> Self {
        Self {
            config,
            input_state_msg: Input::default(),
            output_atmosphere_msg: Output::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ExponentialAtmosphere, ExponentialAtmosphereConfig};
    use crate::messages::{Output, SpacecraftStateMsg};
    use crate::{Module, SimulationContext};
    use hifitime::Epoch;
    use nalgebra::Vector3;

    fn dummy_context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    #[test]
    fn density_matches_exponential_atmosphere_closed_form() {
        let planet_radius_m = 6_378_136.6;
        let mut atmosphere = ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
            name: "exp_atmo".to_string(),
            planet_radius_m,
            reference_altitude_m: 0.0,
            reference_density_kgpm3: 1.217,
            scale_height_m: 8500.0,
        });
        let state_out = Output::new(SpacecraftStateMsg::default());
        atmosphere.input_state_msg.connect(state_out.slot());
        atmosphere.init();

        let cases = [
            (6_571_000.0_f64, 1.703062e-10_f64),
            (6_600_000.0, 5.617201e-12),
        ];
        for (radius_m, truth_density) in cases {
            state_out.write(SpacecraftStateMsg {
                position_m: Vector3::new(radius_m, 0.0, 0.0),
                ..Default::default()
            });
            atmosphere.update(&dummy_context());
            let density = atmosphere
                .output_atmosphere_msg
                .read()
                .neutral_density_kgpm3;

            let alt = radius_m - planet_radius_m;
            let analytic = 1.217 * (-alt / 8500.0).exp();
            let rel_err = (density - analytic).abs() / analytic;
            assert!(rel_err < 1e-12, "rust vs analytic mismatch: {rel_err:.2e}");

            let rel_err_truth = (density - truth_density).abs() / truth_density;
            assert!(
                rel_err_truth < 1e-5,
                "density at r={radius_m:.0} m does not match expected truth: \
                 got {density:.6e}, expected {truth_density:.6e} (rel_err={rel_err_truth:.2e})"
            );
        }
    }
}
