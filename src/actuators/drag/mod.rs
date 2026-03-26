use nalgebra::Vector3;

use crate::messages::{AtmosphereMsg, Input, SpacecraftStateMsg};
use crate::spacecraft::EffectorOutput;

#[derive(Clone, Debug)]
pub struct DragConfig {
    pub name: String,
    pub projected_area_m2: f64,
    pub drag_coeff: f64,
    pub com_offset_m: Vector3<f64>,
    pub planet_rotation_rate_radps: Vector3<f64>,
}

#[derive(Clone, Debug)]
pub struct Drag {
    pub config: DragConfig,
    pub input_atmosphere_msg: Input<AtmosphereMsg>,
}

impl Drag {
    pub fn new(config: DragConfig) -> Self {
        Self {
            config,
            input_atmosphere_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let atmosphere = self.input_atmosphere_msg.read();
        if atmosphere.neutral_density_kgpm3 <= 0.0 {
            return EffectorOutput::default();
        }

        let position_inertial = state.position_m;
        let velocity_inertial = state.velocity_mps;
        let relative_velocity_inertial = velocity_inertial
            - self
                .config
                .planet_rotation_rate_radps
                .cross(&position_inertial);
        let relative_speed = relative_velocity_inertial.norm();
        if relative_speed == 0.0 {
            return EffectorOutput::default();
        }

        let body_to_inertial = state.attitude_b_to_i;
        let inertial_to_body = body_to_inertial.inverse();
        let relative_velocity_body = inertial_to_body.transform_vector(&relative_velocity_inertial);
        let force_body = -0.5
            * self.config.drag_coeff
            * self.config.projected_area_m2
            * atmosphere.neutral_density_kgpm3
            * relative_speed
            * relative_speed
            * relative_velocity_body.normalize();

        EffectorOutput {
            force_inertial_n: body_to_inertial.transform_vector(&force_body),
            torque_body_nm: self.config.com_offset_m.cross(&force_body),
        }
    }
}
