use nalgebra::Vector3;
use std::any::Any;

use crate::messages::{Input, MagneticFieldMsg, MtbCommandMsg, SpacecraftStateMsg};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

#[derive(Clone, Debug)]
pub struct MtbConfig {
    pub name: String,
    pub dipole_axis_body: Vector3<f64>,
    pub max_dipole_am2: f64,
}

impl MtbConfig {
    /// Check numeric invariants. Returns a description of the first violation,
    /// or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        if !self.dipole_axis_body.iter().all(|v| v.is_finite()) {
            return Err(format!(
                "dipole_axis_body must be finite, got {:?}",
                self.dipole_axis_body
            ));
        }
        if !self.max_dipole_am2.is_finite() || self.max_dipole_am2 < 0.0 {
            return Err(format!(
                "max_dipole_am2 must be finite and >= 0, got {}",
                self.max_dipole_am2
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct Mtb {
    pub config: MtbConfig,
    pub command_in: Input<MtbCommandMsg>,
    pub input_magnetic_field_msg: Input<MagneticFieldMsg>,
}

impl Mtb {
    pub fn new(config: MtbConfig) -> Self {
        if let Err(msg) = config.validate() {
            panic!("invalid MtbConfig: {msg}");
        }
        Self {
            config,
            command_in: Input::default(),
            input_magnetic_field_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let attitude_body_to_inertial = state.body_to_inertial();
        let magnetic_field_body_t = attitude_body_to_inertial.inverse().transform_vector(
            &self
                .input_magnetic_field_msg
                .read()
                .magnetic_field_inertial_t,
        );
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

impl DynamicEffector for Mtb {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        Mtb::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

fn normalize_or_zero(vector: Vector3<f64>) -> Vector3<f64> {
    if vector.norm_squared() > 0.0 {
        vector.normalize()
    } else {
        Vector3::zeros()
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::messages::{MagneticFieldMsg, MtbCommandMsg, Output, SpacecraftStateMsg};

    use super::{Mtb, MtbConfig};

    #[allow(clippy::type_complexity)]
    fn make_mtb(
        name: &str,
        axis: Vector3<f64>,
        dipole_cmd_am2: f64,
        max_dipole_am2: f64,
        field_inertial_t: Vector3<f64>,
    ) -> (Mtb, Output<MtbCommandMsg>, Output<MagneticFieldMsg>) {
        let cmd_out = Output::new(MtbCommandMsg { dipole_cmd_am2 });
        let field_out = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t: field_inertial_t,
        });
        let mut mtb = Mtb::new(MtbConfig {
            name: name.to_string(),
            dipole_axis_body: axis,
            max_dipole_am2,
        });
        mtb.command_in.connect(cmd_out.slot());
        mtb.input_magnetic_field_msg.connect(field_out.slot());
        (mtb, cmd_out, field_out)
    }

    fn state(sigma_bn: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            sigma_bn,
            ..Default::default()
        }
    }

    #[test]
    fn single_bar_torque_is_dipole_cross_field() {
        let field = Vector3::new(0.0, 1.0e-5, 0.0);
        let (mtb, _c, _f) = make_mtb("mtb", Vector3::new(1.0, 0.0, 0.0), 0.5, 10.0, field);
        // Identity attitude: field_body == field. m = [0.5,0,0].
        // torque = m x B = [0,0, 0.5 * 1e-5].
        let out = mtb.compute_output(&state(Vector3::zeros()));
        let expected = Vector3::new(0.5, 0.0, 0.0).cross(&field);
        assert_eq!(out.force_inertial_n, Vector3::zeros());
        assert!((out.torque_body_nm - expected).norm() < 1e-18, "got {:?}", out.torque_body_nm);
    }

    #[test]
    fn command_is_clamped_to_max_dipole() {
        let field = Vector3::new(0.0, 1.0e-5, 0.0);
        let (mtb, _c, _f) = make_mtb("mtb", Vector3::new(1.0, 0.0, 0.0), 20.0, 10.0, field);
        let out = mtb.compute_output(&state(Vector3::zeros()));
        // command 20 clamps to 10 -> m = [10,0,0].
        let expected = Vector3::new(10.0, 0.0, 0.0).cross(&field);
        assert!((out.torque_body_nm - expected).norm() < 1e-15, "got {:?}", out.torque_body_nm);
    }

    #[test]
    fn three_bars_match_aggregate_truth() {
        // Mirrors the reference net-torque check: tau = [0.2,0.2,0.2] x B_body,
        // built from three orthogonal single-axis bars.
        let field = Vector3::new(1.0e-5, 2.0e-5, 1.5e-5);
        let sigma_bn = Vector3::new(0.3, 0.2, 0.1);
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

        let mut total = Vector3::zeros();
        for (i, axis) in axes.iter().enumerate() {
            let (mtb, _c, _f) = make_mtb(&format!("mtb_{i}"), *axis, 0.2, 10.0, field);
            total += mtb.compute_output(&state(sigma_bn)).torque_body_nm;
        }

        let field_body = state(sigma_bn).inertial_to_body().transform_vector(&field);
        let net_dipole = Vector3::new(0.2, 0.2, 0.2);
        let expected = net_dipole.cross(&field_body);
        assert!((total - expected).norm() < 1e-15, "got {total:?}, want {expected:?}");
    }

    #[test]
    #[should_panic(expected = "max_dipole_am2")]
    fn rejects_negative_max_dipole() {
        let _ = Mtb::new(MtbConfig {
            name: "mtb".to_string(),
            dipole_axis_body: Vector3::x(),
            max_dipole_am2: -1.0,
        });
    }
}
