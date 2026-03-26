use nalgebra::Vector3;

use crate::messages::{Input, MagneticFieldMsg, MtbCommandMsg, SpacecraftStateMsg};
use crate::spacecraft::EffectorOutput;

#[derive(Clone, Debug)]
pub struct MtbConfig {
    pub name: String,
    pub dipole_axis_body: Vector3<f64>,
    pub max_dipole_am2: f64,
}

#[derive(Clone, Debug)]
pub struct Mtb {
    pub config: MtbConfig,
    pub command_in: Input<MtbCommandMsg>,
    pub input_magnetic_field_msg: Input<MagneticFieldMsg>,
}

impl Mtb {
    pub fn new(config: MtbConfig) -> Self {
        Self {
            config,
            command_in: Input::default(),
            input_magnetic_field_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let attitude_body_to_inertial = state.attitude_b_to_i;
        let magnetic_field_body_t = attitude_body_to_inertial
            .inverse()
            .transform_vector(&self.input_magnetic_field_msg.read().magnetic_field_inertial_t);
        let dipole_axis_body = normalize_or_zero(self.config.dipole_axis_body);
        let commanded_dipole = self
            .command_in
            .read()
            .dipole_cmd_am2
            .clamp(-self.config.max_dipole_am2, self.config.max_dipole_am2);
        let dipole_body_am2 = dipole_axis_body * commanded_dipole;

        EffectorOutput {
            force_inertial_n: Vector3::zeros(),
            torque_body_nm: dipole_body_am2.cross(&magnetic_field_body_t),
        }
    }
}

fn normalize_or_zero(vector: Vector3<f64>) -> Vector3<f64> {
    if vector.norm_squared() > 0.0 {
        vector.normalize()
    } else {
        Vector3::zeros()
    }
}
