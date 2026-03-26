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
        let separation_rad = safe_acos(
            (-planet_to_spacecraft.dot(&sun_to_spacecraft))
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
