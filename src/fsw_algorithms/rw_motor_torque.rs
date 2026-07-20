use nalgebra::{DMatrix, DVector, Vector3};

use crate::messages::{ArrayMotorTorqueMsg, BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output};
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
    pub rw_motor_torque_out_msg: Output<ArrayMotorTorqueMsg>,
}

impl RwMotorTorque {
    pub fn new(config: RwMotorTorqueConfig) -> Self {
        assert!(
            config.wheel_spin_axes_body.len() <= MAX_EFF_COUNT,
            "at most {MAX_EFF_COUNT} reaction wheels are supported"
        );
        Self {
            rw_motor_torque_out_msg: Output::default(),
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
        self.rw_motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.rw_motor_torque_out_msg
            .write(ArrayMotorTorqueMsg::from_active(
                &self.compute_wheel_torques(),
            ));
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

        let command = module.rw_motor_torque_out_msg.read();
        let wheel_0 = command.motor_torque_nm[0];
        let wheel_1 = command.motor_torque_nm[1];
        assert!((wheel_0 + 0.1).abs() < 1.0e-12);
        assert!((wheel_1 - 0.2).abs() < 1.0e-12);
        assert!(
            command.motor_torque_nm[2..]
                .iter()
                .all(|value| *value == 0.0)
        );
    }
}
