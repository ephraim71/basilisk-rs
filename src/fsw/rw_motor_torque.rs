use nalgebra::{DMatrix, DVector, Vector3};

use crate::messages::{BodyTorqueCommandMsg, Input, Output, ReactionWheelCommandMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct RwMotorTorqueConfig {
    pub name: String,
    pub control_axes_body: Vec<Vector3<f64>>,
    pub wheel_spin_axes_body: Vec<Vector3<f64>>,
}

#[derive(Clone, Debug)]
pub struct RwMotorTorque {
    pub config: RwMotorTorqueConfig,
    pub veh_control_in_msg: Input<BodyTorqueCommandMsg>,
    pub rw_motor_torque_out_msgs: Vec<Output<ReactionWheelCommandMsg>>,
}

impl RwMotorTorque {
    pub fn new(config: RwMotorTorqueConfig) -> Self {
        Self {
            rw_motor_torque_out_msgs: (0..config.wheel_spin_axes_body.len())
                .map(|_| Output::default())
                .collect(),
            config,
            veh_control_in_msg: Input::default(),
        }
    }

    fn compute_wheel_torques(&self) -> Vec<f64> {
        let mut torques = vec![0.0; self.config.wheel_spin_axes_body.len()];
        let requested_torque = self.veh_control_in_msg.read().torque_request_body_nm;

        let control_axes: Vec<_> = self
            .config
            .control_axes_body
            .iter()
            .copied()
            .filter(|axis| axis.norm_squared() > 0.0)
            .map(|axis| axis.normalize())
            .collect();
        if control_axes.is_empty() {
            return torques;
        }

        let wheel_axes: Vec<_> = self
            .config
            .wheel_spin_axes_body
            .iter()
            .copied()
            .map(normalize_or_zero)
            .collect();
        if wheel_axes.len() < control_axes.len() {
            return torques;
        }

        let lr_b = -requested_torque;
        let mut lr_c = DVector::zeros(control_axes.len());
        for (row, axis) in control_axes.iter().enumerate() {
            lr_c[row] = axis.dot(&lr_b);
        }

        let mut cgs = DMatrix::zeros(control_axes.len(), wheel_axes.len());
        for (i, axis) in control_axes.iter().enumerate() {
            for (j, wheel_axis) in wheel_axes.iter().enumerate() {
                cgs[(i, j)] = axis.dot(wheel_axis);
            }
        }

        let m = &cgs * cgs.transpose();
        let Some(m_inv) = m.try_inverse() else {
            return torques;
        };
        let us = cgs.transpose() * m_inv * lr_c;
        for (torque, command) in torques.iter_mut().zip(us.iter()) {
            *torque = *command;
        }
        torques
    }
}

impl Module for RwMotorTorque {
    fn init(&mut self) {
        for output in &self.rw_motor_torque_out_msgs {
            output.write(ReactionWheelCommandMsg::default());
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        for (output, torque) in self
            .rw_motor_torque_out_msgs
            .iter()
            .zip(self.compute_wheel_torques().into_iter())
        {
            output.write(ReactionWheelCommandMsg {
                motor_torque_nm: torque,
            });
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

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{BodyTorqueCommandMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{RwMotorTorque, RwMotorTorqueConfig};

    fn dummy_context() -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        }
    }

    #[test]
    fn identity_two_axis_mapping_matches_expected_sign() {
        let mut module = RwMotorTorque::new(RwMotorTorqueConfig {
            name: "rwMotorTorque".to_string(),
            control_axes_body: vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
            wheel_spin_axes_body: vec![Vector3::new(1.0, 0.0, 0.0), Vector3::new(0.0, 1.0, 0.0)],
        });
        let torque_out = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(0.1, -0.2, 0.0),
        });
        module.veh_control_in_msg.connect(torque_out.slot());

        module.init();
        module.update(&dummy_context());

        let wheel_0 = module.rw_motor_torque_out_msgs[0].read().motor_torque_nm;
        let wheel_1 = module.rw_motor_torque_out_msgs[1].read().motor_torque_nm;
        assert!((wheel_0 + 0.1).abs() < 1.0e-12);
        assert!((wheel_1 - 0.2).abs() < 1.0e-12);
    }
}
