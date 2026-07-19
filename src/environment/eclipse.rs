use nalgebra::Vector3;

use crate::messages::{EclipseMsg, Input, Output, SpacecraftStateMsg, SunEphemerisMsg};
use crate::{Module, SimulationContext};

const SUN_RADIUS_M: f64 = 695_700_000.0;

#[derive(Clone, Debug)]
pub struct EclipseConfig {
    pub name: String,
    pub occulting_body_radius_m: f64,
}

pub struct Eclipse {
    pub config: EclipseConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub output_eclipse_msg: Output<EclipseMsg>,
}

impl Module for Eclipse {
    fn init(&mut self) {
        self.output_eclipse_msg.write(EclipseMsg {
            illumination_factor: 1.0,
        });
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.input_state_msg.read();
        let sun = self.input_sun_msg.read();
        let spacecraft_position = state.position_m;
        let sun_to_spacecraft = spacecraft_position - sun.sun_position_inertial_m;
        let planet_to_spacecraft = spacecraft_position;
        let sun_to_planet = -sun.sun_position_inertial_m;

        let illumination_factor = if sun_to_spacecraft.norm() < sun_to_planet.norm() {
            1.0
        } else {
            self.compute_illumination_factor(planet_to_spacecraft, sun_to_spacecraft)
        };

        self.output_eclipse_msg.write(EclipseMsg {
            illumination_factor,
        });
    }
}

impl Eclipse {
    pub fn new(config: EclipseConfig) -> Self {
        Self {
            config,
            input_state_msg: Input::default(),
            input_sun_msg: Input::default(),
            output_eclipse_msg: Output::default(),
        }
    }

    fn compute_illumination_factor(
        &self,
        planet_to_spacecraft: Vector3<f64>,
        sun_to_spacecraft: Vector3<f64>,
    ) -> f64 {
        let apparent_sun_radius_rad = safe_asin(SUN_RADIUS_M / sun_to_spacecraft.norm());
        let apparent_planet_radius_rad =
            safe_asin(self.config.occulting_body_radius_m / planet_to_spacecraft.norm());
        // Basilisk cpp: acos((-s_BP_N · r_HB_N) / ...) where r_HB_N = sun - sc = -sun_to_spacecraft
        // Expanding: -s_BP_N · (-sun_to_spacecraft) = planet_to_spacecraft · sun_to_spacecraft
        let separation_rad = safe_acos(
            planet_to_spacecraft.dot(&sun_to_spacecraft)
                / (planet_to_spacecraft.norm() * sun_to_spacecraft.norm()),
        );

        if separation_rad < apparent_planet_radius_rad - apparent_sun_radius_rad {
            0.0
        } else if separation_rad < apparent_sun_radius_rad - apparent_planet_radius_rad {
            let sun_area = std::f64::consts::PI * apparent_sun_radius_rad * apparent_sun_radius_rad;
            let planet_area =
                std::f64::consts::PI * apparent_planet_radius_rad * apparent_planet_radius_rad;
            (1.0 - (sun_area - planet_area) / sun_area).clamp(0.0, 1.0)
        } else if separation_rad < apparent_sun_radius_rad + apparent_planet_radius_rad {
            let x = (separation_rad * separation_rad
                + apparent_sun_radius_rad * apparent_sun_radius_rad
                - apparent_planet_radius_rad * apparent_planet_radius_rad)
                / (2.0 * separation_rad);
            let y = (apparent_sun_radius_rad * apparent_sun_radius_rad - x * x)
                .max(0.0)
                .sqrt();
            let overlap_area = apparent_sun_radius_rad
                * apparent_sun_radius_rad
                * safe_acos(x / apparent_sun_radius_rad)
                + apparent_planet_radius_rad
                    * apparent_planet_radius_rad
                    * safe_acos((separation_rad - x) / apparent_planet_radius_rad)
                - separation_rad * y;
            let sun_area = std::f64::consts::PI * apparent_sun_radius_rad * apparent_sun_radius_rad;
            (1.0 - overlap_area / sun_area).clamp(0.0, 1.0)
        } else {
            1.0
        }
    }
}

fn safe_asin(value: f64) -> f64 {
    value.clamp(-1.0, 1.0).asin()
}

fn safe_acos(value: f64) -> f64 {
    value.clamp(-1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{Output, SpacecraftStateMsg, SunEphemerisMsg};
    use crate::{Module, SimulationContext};

    use super::{Eclipse, EclipseConfig};

    const AU: f64 = 149_597_870_693.0; // m
    const R_EARTH: f64 = 6_371_000.0; // m

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    fn run_eclipse(sc_position: Vector3<f64>, sun_position: Vector3<f64>, body_radius: f64) -> f64 {
        let mut eclipse = Eclipse::new(EclipseConfig {
            name: "eclipse".to_string(),
            occulting_body_radius_m: body_radius,
        });

        let state_out = Output::new(SpacecraftStateMsg {
            position_m: sc_position,
            velocity_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
            omega_radps: Vector3::zeros(),
        });
        let sun_out = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sun_position,
            sun_velocity_inertial_mps: Vector3::zeros(),
        });

        eclipse.input_state_msg.connect(state_out.slot());
        eclipse.input_sun_msg.connect(sun_out.slot());
        eclipse.init();
        eclipse.update(&dummy_context());
        eclipse.output_eclipse_msg.read().illumination_factor
    }

    /// Spacecraft at [-7000 km, 0, 0], sun at [+1AU, 0, 0], planet (Earth radius) at origin.
    /// Expected: illumination_factor = 0.0.
    #[test]
    fn spacecraft_in_umbra_yields_zero_illumination() {
        let sc_pos = Vector3::new(-7_000_000.0, 0.0, 0.0);
        let sun_pos = Vector3::new(AU, 0.0, 0.0);
        let factor = run_eclipse(sc_pos, sun_pos, R_EARTH);
        assert_eq!(factor, 0.0, "expected full shadow, got {factor}");
    }

    /// Guard condition: sun_to_sc.norm() < sun_to_planet.norm() → return 1.0 immediately.
    #[test]
    fn spacecraft_closer_to_sun_than_planet_yields_full_illumination() {
        // Spacecraft at [1AU - 1e8, 0, 0] (between Earth and Sun, 1e8 m from Sun)
        let sc_pos = Vector3::new(AU - 1e8, 0.0, 0.0);
        let sun_pos = Vector3::new(AU, 0.0, 0.0);
        let factor = run_eclipse(sc_pos, sun_pos, R_EARTH);
        assert_eq!(factor, 1.0, "expected full illumination, got {factor}");
    }

    /// Angular separation >> sum of apparent radii → no eclipse.
    #[test]
    fn spacecraft_perpendicular_to_sun_yields_full_illumination() {
        // Sun along +y, spacecraft along +x — large angular separation, no shadow possible
        let sc_pos = Vector3::new(7_000_000.0, 0.0, 0.0);
        let sun_pos = Vector3::new(0.0, AU, 0.0);
        let factor = run_eclipse(sc_pos, sun_pos, R_EARTH);
        assert_eq!(factor, 1.0, "expected full illumination, got {factor}");
    }
}
