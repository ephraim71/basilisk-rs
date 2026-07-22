use std::any::Any;

use nalgebra::Vector3;

use crate::messages::{
    Input, MAX_EFF_COUNT, MagneticFieldMsg, MtbArrayCommandMsg, MtbArrayConfigMsg,
    SpacecraftStateMsg,
};
use crate::spacecraft::{DynamicEffector, EffectorOutput};

/// Aggregate magnetic-torque-bar effector.
///
/// One instance represents the complete MTB array. Active command and
/// configuration slots share the same index.
#[derive(Clone, Debug)]
pub struct MtbEffector {
    pub name: String,
    pub mtb_cmd_in_msg: Input<MtbArrayCommandMsg>,
    pub mag_in_msg: Input<MagneticFieldMsg>,
    pub mtb_params_in_msg: Input<MtbArrayConfigMsg>,
}

impl MtbEffector {
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            mtb_cmd_in_msg: Input::default(),
            mag_in_msg: Input::default(),
            mtb_params_in_msg: Input::default(),
        }
    }

    pub fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        let config = self.mtb_params_in_msg.read();
        let command = self.mtb_cmd_in_msg.read();
        assert!(
            config.num_mtb <= MAX_EFF_COUNT,
            "MTB count {} exceeds fixed payload capacity {MAX_EFF_COUNT}",
            config.num_mtb
        );
        let magnetic_field_body_t = state
            .inertial_to_body()
            .transform_vector(&self.mag_in_msg.read().magnetic_field_inertial_t);

        let mut net_dipole_body_am2 = Vector3::zeros();
        for index in 0..config.num_mtb {
            let axis_body = config.dipole_axes_body[index];
            let max_dipole_am2 = config.max_dipoles_am2[index];
            let mut dipole_command_am2 = command.dipole_cmds_am2[index];
            if dipole_command_am2 > max_dipole_am2 {
                dipole_command_am2 = max_dipole_am2;
            } else if dipole_command_am2 < -max_dipole_am2 {
                dipole_command_am2 = -max_dipole_am2;
            }
            // Each Gt matrix column is used verbatim; configured MTB axes are
            // not normalized inside the dynamics module.
            net_dipole_body_am2 += axis_body * dipole_command_am2;
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

    use crate::messages::{
        MAX_EFF_COUNT, MagneticFieldMsg, MtbArrayCommandMsg, MtbArrayConfigMsg, Output,
        SpacecraftStateMsg,
    };

    use super::MtbEffector;

    fn make_mtb(
        axes_body: &[Vector3<f64>],
        dipole_commands_am2: &[f64],
        max_dipoles_am2: &[f64],
        magnetic_field_inertial_t: Vector3<f64>,
    ) -> MtbEffector {
        let command = Output::new(MtbArrayCommandMsg::from_active(dipole_commands_am2));
        let config = Output::new(MtbArrayConfigMsg::from_active(axes_body, max_dipoles_am2));
        let magnetic_field = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t,
        });
        let mut mtb = MtbEffector::new("mtb");
        mtb.mtb_cmd_in_msg.connect(command.slot());
        mtb.mtb_params_in_msg.connect(config.slot());
        mtb.mag_in_msg.connect(magnetic_field.slot());
        mtb
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
    fn three_bars_match_upstream_aggregate_truth_with_and_without_saturation() {
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
    fn active_axes_are_used_verbatim_and_inactive_slots_are_ignored() {
        let command = Output::new(MtbArrayCommandMsg::from_active(&[0.5, 100.0]));
        let mut config = MtbArrayConfigMsg::from_active(&[2.0 * Vector3::x()], &[10.0]);
        config.dipole_axes_body[1] = Vector3::y();
        config.max_dipoles_am2[1] = 100.0;
        let config = Output::new(config);
        let magnetic_field = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t: Vector3::y(),
        });
        let mut mtb = MtbEffector::new("mtb");
        mtb.mtb_cmd_in_msg.connect(command.slot());
        mtb.mtb_params_in_msg.connect(config.slot());
        mtb.mag_in_msg.connect(magnetic_field.slot());

        let output = mtb.compute_output(&state(Vector3::zeros()));
        assert_eq!(output.torque_body_nm, Vector3::z());
    }

    #[test]
    fn supports_the_full_fixed_capacity() {
        let axes = [Vector3::x(); MAX_EFF_COUNT];
        let commands = [0.25; MAX_EFF_COUNT];
        let maxima = [1.0; MAX_EFF_COUNT];
        let mtb = make_mtb(&axes, &commands, &maxima, Vector3::y());

        let output = mtb.compute_output(&state(Vector3::zeros()));
        assert_eq!(
            output.torque_body_nm,
            Vector3::z() * (MAX_EFF_COUNT as f64 * 0.25)
        );
    }

    #[test]
    #[should_panic(expected = "exceeds fixed payload capacity")]
    fn rejects_count_above_fixed_capacity() {
        let command = Output::new(MtbArrayCommandMsg {
            dipole_cmds_am2: [1.0; MAX_EFF_COUNT],
        });
        let config = Output::new(MtbArrayConfigMsg {
            num_mtb: MAX_EFF_COUNT + 1,
            dipole_axes_body: [Vector3::x(); MAX_EFF_COUNT],
            max_dipoles_am2: [1.0; MAX_EFF_COUNT],
        });
        let magnetic_field = Output::new(MagneticFieldMsg {
            magnetic_field_inertial_t: Vector3::y(),
        });
        let mut mtb = MtbEffector::new("mtb");
        mtb.mtb_cmd_in_msg.connect(command.slot());
        mtb.mtb_params_in_msg.connect(config.slot());
        mtb.mag_in_msg.connect(magnetic_field.slot());

        let _ = mtb.compute_output(&state(Vector3::zeros()));
    }
}
