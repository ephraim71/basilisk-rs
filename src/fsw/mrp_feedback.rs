use nalgebra::{Matrix3, Vector3};

use crate::messages::{AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, Output};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct MrpFeedbackConfig {
    pub name: String,
    pub inertia_kg_m2: Matrix3<f64>,
    pub k: f64,
    pub ki: f64,
    pub p: f64,
    pub integral_limit: f64,
    pub known_torque_body_nm: Vector3<f64>,
    pub control_law_type: u8,
}

#[derive(Clone, Debug)]
pub struct MrpFeedback {
    pub config: MrpFeedbackConfig,
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    pub cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    int_sigma: Vector3<f64>,
    prior_time_nanos: Option<u64>,
}

impl MrpFeedback {
    pub fn new(config: MrpFeedbackConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
            int_sigma: Vector3::zeros(),
            prior_time_nanos: None,
        }
    }

    fn compute_control_torque(&mut self, call_time_nanos: u64) -> BodyTorqueCommandMsg {
        let guid = self.guid_in_msg.read();
        let dt = self
            .prior_time_nanos
            .map(|prior| (call_time_nanos - prior) as f64 * 1.0e-9)
            .unwrap_or(0.0);
        self.prior_time_nanos = Some(call_time_nanos);

        let omega_bn_b = guid.omega_br_b_radps + guid.omega_rn_b_radps;
        let mut z = Vector3::zeros();
        if self.config.ki > 0.0 {
            self.int_sigma += self.config.k * dt * guid.sigma_br;
            for component in self.int_sigma.iter_mut() {
                let magnitude = component.abs();
                if magnitude > self.config.integral_limit {
                    *component *= self.config.integral_limit / magnitude;
                }
            }
            z = self.int_sigma + self.config.inertia_kg_m2 * guid.omega_br_b_radps;
        }

        let mut lr = self.config.k * guid.sigma_br + self.config.p * guid.omega_br_b_radps;
        lr += self.config.p * self.config.ki * z;

        let v8 = if self.config.control_law_type == 0 {
            guid.omega_rn_b_radps + self.config.ki * z
        } else {
            omega_bn_b
        };
        let v6 = self.config.inertia_kg_m2 * omega_bn_b;
        lr -= v8.cross(&v6);
        lr += self.config.inertia_kg_m2
            * (-guid.domega_rn_b_radps2 + omega_bn_b.cross(&guid.omega_rn_b_radps));
        lr += self.config.known_torque_body_nm;
        lr = -lr;

        BodyTorqueCommandMsg {
            torque_request_body_nm: lr,
        }
    }
}

impl Module for MrpFeedback {
    fn init(&mut self) {
        self.int_sigma = Vector3::zeros();
        self.prior_time_nanos = None;
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let torque_cmd = self.compute_control_torque(context.current_sim_nanos);
        self.cmd_torque_out_msg.write(torque_cmd);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{AttitudeGuidanceMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{MrpFeedback, MrpFeedbackConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        let epoch = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
        SimulationContext {
            start_epoch: epoch,
            current_sim_nanos,
            current_epoch: epoch,
        }
    }

    #[test]
    fn zero_guidance_error_yields_zero_torque() {
        let mut module = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFeedback".to_string(),
            inertia_kg_m2: Matrix3::identity(),
            k: 1.0,
            ki: -1.0,
            p: 2.0,
            integral_limit: 0.1,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
        });
        let guidance_out = Output::new(AttitudeGuidanceMsg::default());
        module.guid_in_msg.connect(guidance_out.slot());

        module.init();
        module.update(&context(0));

        assert!(
            module
                .cmd_torque_out_msg
                .read()
                .torque_request_body_nm
                .norm()
                < 1.0e-12
        );
    }
}
