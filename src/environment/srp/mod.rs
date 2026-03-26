use nalgebra::Vector3;

use crate::messages::{EclipseMsg, Input, SpacecraftStateMsg, SunEphemerisMsg};
use crate::spacecraft::EffectorOutput;

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_WPM2: f64 = 1372.5398;
const SPEED_OF_LIGHT_MPS: f64 = 299_792_458.0;

#[derive(Clone, Debug)]
pub struct SolarRadiationPressureConfig {
    pub name: String,
    pub area_m2: f64,
    pub coefficient_reflection: f64,
}

#[derive(Clone, Debug)]
pub struct SolarRadiationPressure {
    pub config: SolarRadiationPressureConfig,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_eclipse_msg: Input<EclipseMsg>,
}

impl SolarRadiationPressure {
    pub fn new(config: SolarRadiationPressureConfig) -> Self {
        Self {
            config,
            input_sun_msg: Input::default(),
            input_eclipse_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let sun = self.input_sun_msg.read();
        let eclipse = self.input_eclipse_msg.read();
        let spacecraft_position = state.position_m;
        let sun_to_spacecraft = spacecraft_position - sun.sun_position_inertial_m;
        let sun_distance_m = sun_to_spacecraft.norm();

        if sun_distance_m == 0.0 {
            return EffectorOutput::default();
        }

        let scale_factor = -self.config.coefficient_reflection
            * self.config.area_m2
            * SOLAR_FLUX_AT_EARTH_WPM2
            * ASTRONOMICAL_UNIT_M.powi(2)
            / (SPEED_OF_LIGHT_MPS * sun_distance_m.powi(3));

        EffectorOutput {
            force_inertial_n: scale_factor * sun_to_spacecraft * eclipse.illumination_factor,
            torque_body_nm: Vector3::zeros(),
        }
    }
}
