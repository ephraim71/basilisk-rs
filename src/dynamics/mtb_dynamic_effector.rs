use std::any::Any;

use nalgebra::Vector3;

use crate::messages::{
    Input, MAX_EFF_COUNT, MagneticDipoleCommandMsg, MagneticFieldMsg, SpacecraftStateMsg,
};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

/// Static configuration for one magnetic torque bar.
#[derive(Clone, Debug)]
pub struct MtbConfig {
    pub name: String,
    pub dipole_axis_body: Vector3<f64>,
    pub max_dipole_am2: f64,
}

/// One magnetic torque bar and its independently connectable command port.
#[derive(Clone, Debug)]
pub struct Mtb {
    pub config: MtbConfig,
    pub dipole_cmd_in_msg: Input<MagneticDipoleCommandMsg>,
}

impl Mtb {
    pub fn new(config: MtbConfig) -> Self {
        assert!(
            config.max_dipole_am2 >= 0.0,
            "maximum MTB dipole must be non-negative"
        );
        Self {
            config,
            dipole_cmd_in_msg: Input::default(),
        }
    }
}

/// Aggregate magnetic-torque-bar effector with one scalar command port per bar.
#[derive(Clone, Debug)]
pub struct MtbEffector {
    pub name: String,
    pub mag_in_msg: Input<MagneticFieldMsg>,
    mtbs: Vec<Mtb>,
}

impl MtbEffector {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mag_in_msg: Input::default(),
            mtbs: Vec::new(),
        }
    }

    /// Registers one torque bar and returns its independently connectable port.
    pub fn add_mtb(&mut self, config: MtbConfig) -> &mut Mtb {
        assert!(
            self.mtbs.len() < MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} MTBs are supported"
        );
        self.mtbs.push(Mtb::new(config));
        self.mtbs
            .last_mut()
            .expect("registered MTB should be available")
    }

    /// Torque bars in registration order.
    pub fn mtbs(&self) -> &[Mtb] {
        &self.mtbs
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let magnetic_field_body_t = state
            .inertial_to_body()
            .transform_vector(&self.mag_in_msg.read().magnetic_field_inertial_t);

        let mut net_dipole_body_am2 = Vector3::zeros();
        for mtb in &self.mtbs {
            let dipole_moment_am2 = mtb
                .dipole_cmd_in_msg
                .read()
                .dipole_moment_am2
                .clamp(-mtb.config.max_dipole_am2, mtb.config.max_dipole_am2);
            // Configured axes are used verbatim and are not normalized inside
            // the dynamics module.
            net_dipole_body_am2 += mtb.config.dipole_axis_body * dipole_moment_am2;
        }

        EffectorOutput {
            force_inertial_n: Vector3::zeros(),
            torque_body_nm: net_dipole_body_am2.cross(&magnetic_field_body_t),
        }
    }
}

impl DynamicEffector for MtbEffector {
    fn name(&self) -> &str {
        &self.name
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        MtbEffector::compute_output(self, state)
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::Vector3;

    use crate::messages::{MagneticDipoleCommandMsg, MagneticFieldMsg, Output, SpacecraftStateMsg};

    use super::{MtbConfig, MtbEffector};

    fn make_mtb(
        axes_body: &[Vector3<f64>],
        dipole_commands_am2: &[f64],
        max_dipoles_am2: &[f64],
        magnetic_field_inertial_t: Vector3<f64>,
    ) -> MtbEffector {
        assert_eq!(axes_body.len(), dipole_commands_am2.len());
        assert_eq!(axes_body.len(), max_dipoles_am2.len());

        let magnetic_field = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t,
        });
        let mut mtb_effector = MtbEffector::new("mtb");
        mtb_effector.mag_in_msg.connect(magnetic_field.slot());

        for (index, ((axis_body, command_am2), max_dipole_am2)) in axes_body
            .iter()
            .zip(dipole_commands_am2)
            .zip(max_dipoles_am2)
            .enumerate()
        {
            let command = Output::new(MagneticDipoleCommandMsg {
                dipole_moment_am2: *command_am2,
            });
            let mtb = mtb_effector.add_mtb(MtbConfig {
                name: format!("mtb{index}"),
                dipole_axis_body: *axis_body,
                max_dipole_am2: *max_dipole_am2,
            });
            mtb.dipole_cmd_in_msg.connect(command.slot());
        }
        mtb_effector
    }

    fn state(sigma_bn: Vector3<f64>) -> SpacecraftStateMsg {
        SpacecraftStateMsg {
            sigma_bn,
            ..Default::default()
        }
    }

    #[test]
    fn single_bar_torque_is_dipole_cross_field() {
        let magnetic_field = Vector3::new(0.0, 1.0e-5, 0.0);
        let mtb = make_mtb(&[Vector3::x()], &[0.5], &[10.0], magnetic_field);

        let output = mtb.compute_output(&state(Vector3::zeros()));
        let expected = Vector3::new(0.5, 0.0, 0.0).cross(&magnetic_field);
        assert_eq!(output.force_inertial_n, Vector3::zeros());
        assert!((output.torque_body_nm - expected).norm() < 1.0e-18);
    }

    #[test]
    fn three_bars_apply_independent_commands_and_saturation() {
        let magnetic_field = Vector3::new(1.0e-5, 2.0e-5, 1.5e-5);
        let sigma_bn = Vector3::new(0.3, 0.2, 0.1);
        let axes = [Vector3::x(), Vector3::y(), Vector3::z()];

        for (maximum, expected_command) in [(10.0, 0.2), (0.1, 0.1)] {
            let mtb = make_mtb(&axes, &[0.2, -0.2, 0.2], &[maximum; 3], magnetic_field);
            let output = mtb.compute_output(&state(sigma_bn));
            let magnetic_field_body = state(sigma_bn)
                .inertial_to_body()
                .transform_vector(&magnetic_field);
            let expected = Vector3::new(expected_command, -expected_command, expected_command)
                .cross(&magnetic_field_body);
            assert!(
                (output.torque_body_nm - expected).norm() < 1.0e-15,
                "got {:?}, want {expected:?}",
                output.torque_body_nm
            );
        }
    }

    #[test]
    fn configured_axes_are_used_verbatim() {
        let mtb = make_mtb(&[2.0 * Vector3::x()], &[0.5], &[10.0], Vector3::y());
        let output = mtb.compute_output(&state(Vector3::zeros()));
        assert_eq!(output.torque_body_nm, Vector3::z());
    }

    #[test]
    fn supports_the_full_fixed_capacity() {
        let axes = [Vector3::x(); crate::messages::MAX_EFF_COUNT];
        let commands = [0.25; crate::messages::MAX_EFF_COUNT];
        let maxima = [1.0; crate::messages::MAX_EFF_COUNT];
        let mtb = make_mtb(&axes, &commands, &maxima, Vector3::y());

        let output = mtb.compute_output(&state(Vector3::zeros()));
        assert_eq!(
            output.torque_body_nm,
            Vector3::z() * (crate::messages::MAX_EFF_COUNT as f64 * 0.25)
        );
    }

    #[test]
    #[should_panic(expected = "at most 36 MTBs are supported")]
    fn rejects_more_than_fixed_capacity() {
        let mut mtb = MtbEffector::new("mtb");
        for index in 0..=crate::messages::MAX_EFF_COUNT {
            mtb.add_mtb(MtbConfig {
                name: format!("mtb{index}"),
                dipole_axis_body: Vector3::x(),
                max_dipole_am2: 1.0,
            });
        }
    }
}
