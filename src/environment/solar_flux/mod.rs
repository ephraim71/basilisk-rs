use crate::messages::{
    EclipseMsg, Input, Output, SolarFluxMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::{Module, SimulationContext};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_W_PER_M2: f64 = 1372.5398;

#[derive(Clone, Debug)]
pub struct SolarFluxConfig {
    pub name: String,
}

#[derive(Clone, Debug)]
pub struct SolarFlux {
    pub config: SolarFluxConfig,
    pub sun_position_in_msg: Input<SunEphemerisMsg>,
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    pub eclipse_in_msg: Input<EclipseMsg>,
    pub solar_flux_out_msg: Output<SolarFluxMsg>,
}

impl SolarFlux {
    pub fn new(config: SolarFluxConfig) -> Self {
        Self {
            config,
            sun_position_in_msg: Input::default(),
            spacecraft_state_in_msg: Input::default(),
            eclipse_in_msg: Input::default(),
            solar_flux_out_msg: Output::default(),
        }
    }

    pub fn compute_flux_w_per_m2(&self) -> f64 {
        let sun = self.sun_position_in_msg.read();
        let spacecraft = self.spacecraft_state_in_msg.read();
        let sun_to_spacecraft_m = spacecraft.position_m - sun.sun_position_inertial_m;
        let distance_m = sun_to_spacecraft_m.norm();

        if distance_m == 0.0 {
            return 0.0;
        }

        let eclipse_factor = if self.eclipse_in_msg.is_connected() {
            self.eclipse_in_msg.read().illumination_factor
        } else {
            1.0
        };

        SOLAR_FLUX_AT_EARTH_W_PER_M2 * ASTRONOMICAL_UNIT_M.powi(2) / distance_m.powi(2)
            * eclipse_factor
    }
}

impl Module for SolarFlux {
    fn init(&mut self) {
        self.solar_flux_out_msg.write(SolarFluxMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.solar_flux_out_msg.write(SolarFluxMsg {
            flux_w_per_m2: self.compute_flux_w_per_m2(),
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{EclipseMsg, Output, SpacecraftStateMsg, SunEphemerisMsg};
    use crate::{Module, SimulationContext};

    use super::{ASTRONOMICAL_UNIT_M, SOLAR_FLUX_AT_EARTH_W_PER_M2, SolarFlux, SolarFluxConfig};

    fn dummy_context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn make_solar_flux(
        spacecraft_position_m: Vector3<f64>,
        illumination_factor: Option<f64>,
    ) -> (
        SolarFlux,
        Output<SunEphemerisMsg>,
        Output<SpacecraftStateMsg>,
        Option<Output<EclipseMsg>>,
    ) {
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::zeros(),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let spacecraft_out = Output::new(SpacecraftStateMsg {
            position_m: spacecraft_position_m,
            ..Default::default()
        });
        let eclipse_out = illumination_factor.map(|factor| {
            Output::new(EclipseMsg {
                illumination_factor: factor,
            })
        });
        let mut solar_flux = SolarFlux::new(SolarFluxConfig {
            name: "solarFlux".to_string(),
        });
        solar_flux.sun_position_in_msg.connect(sun_out.slot());
        solar_flux
            .spacecraft_state_in_msg
            .connect(spacecraft_out.slot());
        if let Some(eclipse_out) = &eclipse_out {
            solar_flux.eclipse_in_msg.connect(eclipse_out.slot());
        }

        (solar_flux, sun_out, spacecraft_out, eclipse_out)
    }

    #[test]
    fn flux_at_one_au_matches_basilisk_constant() {
        let (mut solar_flux, _sun, _spacecraft, _eclipse) =
            make_solar_flux(Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M), None);

        solar_flux.init();
        solar_flux.update(&dummy_context());

        assert!(
            (solar_flux.solar_flux_out_msg.read().flux_w_per_m2 - SOLAR_FLUX_AT_EARTH_W_PER_M2)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn flux_scales_by_distance_squared_like_basilisk_unit_test() {
        let position_factor = 2.0_f64.sqrt();
        let illumination_factor = 0.5;
        let (mut solar_flux, _sun, spacecraft_out, _eclipse) = make_solar_flux(
            Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M),
            Some(illumination_factor),
        );

        solar_flux.init();
        solar_flux.update(&dummy_context());
        let flux_at_earth = solar_flux.solar_flux_out_msg.read().flux_w_per_m2;

        spacecraft_out.write(SpacecraftStateMsg {
            position_m: Vector3::new(0.0, 0.0, position_factor * ASTRONOMICAL_UNIT_M),
            ..Default::default()
        });
        solar_flux.update(&dummy_context());
        let flux_farther = solar_flux.solar_flux_out_msg.read().flux_w_per_m2;

        let expected = flux_at_earth / position_factor.powi(2);
        assert!(
            ((flux_farther - expected) / expected).abs() < 1.0e-8,
            "expected {expected:.12e} W/m^2, got {flux_farther:.12e} W/m^2"
        );
    }

    #[test]
    fn optional_eclipse_defaults_to_full_illumination() {
        let (mut with_eclipse, _sun_a, _sc_a, _eclipse) =
            make_solar_flux(Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M), Some(1.0));
        let (mut without_eclipse, _sun_b, _sc_b, _none) =
            make_solar_flux(Vector3::new(0.0, 0.0, ASTRONOMICAL_UNIT_M), None);

        with_eclipse.update(&dummy_context());
        without_eclipse.update(&dummy_context());

        assert!(
            (with_eclipse.solar_flux_out_msg.read().flux_w_per_m2
                - without_eclipse.solar_flux_out_msg.read().flux_w_per_m2)
                .abs()
                < 1.0e-12
        );
    }

    #[test]
    fn zero_sun_spacecraft_distance_yields_zero_flux() {
        let (mut solar_flux, _sun, _spacecraft, _eclipse) = make_solar_flux(Vector3::zeros(), None);

        solar_flux.update(&dummy_context());

        assert_eq!(solar_flux.solar_flux_out_msg.read().flux_w_per_m2, 0.0);
    }
}
