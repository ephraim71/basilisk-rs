use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, RwArrayConfigMsg,
    RwAvailabilityMsg, RwSpeedMsg, VehicleConfigMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct MrpFeedbackConfig {
    pub name: String,
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
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    pub rw_avail_in_msg: Input<RwAvailabilityMsg>,
    pub cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    pub int_feedback_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// Integral state retained between controller updates.
    pub int_sigma: Vector3<f64>,
    /// Upstream's diagnostic integral vector.
    pub z: Vector3<f64>,
    inertia_kg_m2: Matrix3<f64>,
    rw_config: RwArrayConfigMsg,
    prior_time_nanos: u64,
}

impl MrpFeedback {
    pub fn new(config: MrpFeedbackConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            rw_params_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_avail_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
            int_feedback_torque_out_msg: Output::default(),
            int_sigma: Vector3::zeros(),
            z: Vector3::zeros(),
            inertia_kg_m2: Matrix3::zeros(),
            rw_config: RwArrayConfigMsg::default(),
            prior_time_nanos: 0,
        }
    }

    /// Reset cached configuration and retained controller state.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        if self.rw_params_in_msg.is_connected() && !self.rw_speeds_in_msg.is_connected() {
            log::error!(
                "mrpFeedback.rw_speeds_in_msg must be connected when rw_params_in_msg is connected"
            );
        }
        if !self.guid_in_msg.is_connected() {
            log::error!("mrpFeedback.guid_in_msg is not connected");
        }
        if !self.veh_config_in_msg.is_connected() {
            log::error!("mrpFeedback.veh_config_in_msg is not connected");
        }

        self.inertia_kg_m2 = self.veh_config_in_msg.read().inertia_about_point_b_kg_m2;
        self.rw_config = if self.rw_params_in_msg.is_connected() {
            self.rw_params_in_msg.read()
        } else {
            RwArrayConfigMsg::default()
        };
        if self.rw_config.num_rw > MAX_EFF_COUNT {
            log::error!("mrpFeedback.num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating");
            self.rw_config.num_rw = MAX_EFF_COUNT;
        }
        self.int_sigma = Vector3::zeros();
        self.prior_time_nanos = 0;
    }

    fn compute_control_torque(
        &mut self,
        call_time_nanos: u64,
    ) -> (BodyTorqueCommandMsg, BodyTorqueCommandMsg) {
        let guidance = self.guid_in_msg.read();
        let wheel_speeds = self.rw_speeds_in_msg.read();
        let wheel_availability = self.rw_avail_in_msg.read();

        // Preserve the C module's zero-time sentinel exactly. In particular,
        // an update at t=0 leaves the next update's dt at zero as well.
        let dt_seconds = if self.prior_time_nanos == 0 {
            0.0
        } else {
            diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos)
        };
        self.prior_time_nanos = call_time_nanos;

        let omega_bn_b = guidance.omega_br_b_radps + guidance.omega_rn_b_radps;

        self.z = Vector3::zeros();
        if self.config.ki > 0.0 {
            self.int_sigma += self.config.k * dt_seconds * guidance.sigma_br;
            for component in self.int_sigma.iter_mut() {
                let magnitude = component.abs();
                if magnitude > self.config.integral_limit {
                    *component *= self.config.integral_limit / magnitude;
                }
            }
            self.z = self.int_sigma + self.inertia_kg_m2 * guidance.omega_br_b_radps;
        }

        let mut requested = self.config.k * guidance.sigma_br;
        requested += self.config.p * guidance.omega_br_b_radps;
        let integral_state_feedback = self.config.ki * self.z;
        let integral_feedback = self.config.p * integral_state_feedback;
        requested += integral_feedback;

        let mut angular_momentum_body = self.inertia_kg_m2 * omega_bn_b;
        let num_rw = self.rw_config.num_rw.min(MAX_EFF_COUNT);
        for index in 0..num_rw {
            if !wheel_availability.is_available(index) {
                continue;
            }
            let spin_axis = self.rw_config.spin_axes_body[index];
            let spin_inertia = self.rw_config.spin_axis_inertias_kg_m2[index];
            let wheel_speed = wheel_speeds.wheel_speeds_radps[index];
            let wheel_momentum =
                spin_inertia * (omega_bn_b.dot(&spin_axis) + wheel_speed) * spin_axis;
            angular_momentum_body += wheel_momentum;
        }

        let feedback_rate = if self.config.control_law_type == 0 {
            guidance.omega_rn_b_radps + integral_state_feedback
        } else {
            omega_bn_b
        };
        requested -= feedback_rate.cross(&angular_momentum_body);
        requested += self.inertia_kg_m2
            * (-guidance.domega_rn_b_radps2 + omega_bn_b.cross(&guidance.omega_rn_b_radps));
        requested += self.config.known_torque_body_nm;
        requested = -requested;

        (
            BodyTorqueCommandMsg {
                torque_request_body_nm: requested,
            },
            BodyTorqueCommandMsg {
                torque_request_body_nm: -integral_feedback,
            },
        )
    }
}

impl Module for MrpFeedback {
    fn init(&mut self) {
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.int_feedback_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        MrpFeedback::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let (torque, integral_feedback) = self.compute_control_torque(context.current_sim_nanos);
        self.cmd_torque_out_msg.write(torque);
        self.int_feedback_torque_out_msg.write(integral_feedback);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{
        AttitudeGuidanceMsg, Output, RwArrayConfigMsg, RwAvailability, RwAvailabilityMsg,
        RwSpeedMsg, VehicleConfigMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{MrpFeedback, MrpFeedbackConfig};

    #[derive(Clone, Copy)]
    enum AvailabilityMode {
        NoMessage,
        On,
        Off,
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn upstream_truth(
        config: &MrpFeedbackConfig,
        guidance: &AttitudeGuidanceMsg,
        inertia: Matrix3<f64>,
        rw_config: &RwArrayConfigMsg,
        rw_speeds: &RwSpeedMsg,
        availability: &RwAvailabilityMsg,
        dt_seconds: f64,
        sigma_integral: &mut Vector3<f64>,
    ) -> Vector3<f64> {
        if dt_seconds == 0.0 {
            *sigma_integral = Vector3::zeros();
        }
        let omega_bn_b = guidance.omega_br_b_radps + guidance.omega_rn_b_radps;
        let z = if config.ki > 0.0 {
            *sigma_integral += config.k * dt_seconds * guidance.sigma_br;
            for value in sigma_integral.iter_mut() {
                if value.abs() > config.integral_limit {
                    *value *= config.integral_limit / value.abs();
                }
            }
            *sigma_integral + inertia * guidance.omega_br_b_radps
        } else {
            Vector3::zeros()
        };

        let mut wheel_momentum = Vector3::zeros();
        for index in 0..rw_config.num_rw {
            if availability.is_available(index) {
                let axis = rw_config.spin_axes_body[index];
                wheel_momentum += rw_config.spin_axis_inertias_kg_m2[index]
                    * (omega_bn_b.dot(&axis) + rw_speeds.wheel_speeds_radps[index])
                    * axis;
            }
        }

        let mut lr = config.k * guidance.sigma_br
            + config.p * guidance.omega_br_b_radps
            + config.p * config.ki * z;
        let feedback_rate = if config.control_law_type == 0 {
            guidance.omega_rn_b_radps + config.ki * z
        } else {
            omega_bn_b
        };
        lr -= feedback_rate.cross(&(inertia * omega_bn_b + wheel_momentum));
        lr +=
            inertia * (-guidance.domega_rn_b_radps2 + omega_bn_b.cross(&guidance.omega_rn_b_radps));
        -(lr + config.known_torque_body_nm)
    }

    #[test]
    fn matches_all_upstream_mrp_feedback_parameter_combinations() {
        let guidance = AttitudeGuidanceMsg {
            sigma_br: Vector3::new(0.3, -0.5, 0.7),
            omega_br_b_radps: Vector3::new(0.010, -0.020, 0.015),
            omega_rn_b_radps: Vector3::new(-0.02, -0.01, 0.005),
            domega_rn_b_radps2: Vector3::new(0.0002, 0.0003, 0.0001),
        };
        let inertia = Matrix3::new(1000.0, 0.0, 0.0, 0.0, 800.0, 0.0, 0.0, 0.0, 800.0);
        let mut rw_speeds = RwSpeedMsg::default();
        rw_speeds.wheel_speeds_radps[..4].copy_from_slice(&[10.0, 25.0, 50.0, 100.0]);
        let axes = vec![
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::new(0.577_350_269_190, 0.577_350_269_190, 0.577_350_269_190),
        ];

        for int_gain in [0.01, -1.0] {
            for rw_num in [4, 0] {
                for integral_limit in [0.0, 20.0] {
                    for control_law_type in [0, 1] {
                        for availability_mode in [
                            AvailabilityMode::NoMessage,
                            AvailabilityMode::On,
                            AvailabilityMode::Off,
                        ] {
                            let config = MrpFeedbackConfig {
                                name: "mrpFeedback".to_string(),
                                k: 0.15,
                                ki: int_gain,
                                p: 150.0,
                                integral_limit,
                                known_torque_body_nm: Vector3::new(1.0, 1.0, 1.0),
                                control_law_type,
                            };
                            let mut rw_config = RwArrayConfigMsg {
                                num_rw: rw_num,
                                ..Default::default()
                            };
                            rw_config.spin_axes_body[..4].copy_from_slice(&axes);
                            rw_config.spin_axis_inertias_kg_m2[..4].copy_from_slice(&[0.1; 4]);
                            let mut availability = RwAvailabilityMsg::default();
                            if matches!(availability_mode, AvailabilityMode::Off) {
                                availability.wheel_availability[..4]
                                    .fill(RwAvailability::Unavailable);
                            }

                            let guidance_out = Output::new(guidance.clone());
                            let vehicle_out = Output::new(VehicleConfigMsg {
                                inertia_about_point_b_kg_m2: inertia,
                                ..Default::default()
                            });
                            let rw_config_out = Output::new(rw_config.clone());
                            let rw_speed_out = Output::new(rw_speeds.clone());
                            let availability_out = Output::new(availability.clone());
                            let mut module = MrpFeedback::new(config.clone());
                            module.guid_in_msg.connect(guidance_out.slot());
                            module.veh_config_in_msg.connect(vehicle_out.slot());
                            if rw_num > 0 {
                                module.rw_params_in_msg.connect(rw_config_out.slot());
                                module.rw_speeds_in_msg.connect(rw_speed_out.slot());
                            }
                            if !matches!(availability_mode, AvailabilityMode::NoMessage) {
                                module.rw_avail_in_msg.connect(availability_out.slot());
                            }
                            module.init();

                            let mut sigma_integral = Vector3::zeros();
                            let mut expected = Vec::new();
                            for (time, dt) in [(0, 0.0), (500_000_000, 0.0), (1_000_000_000, 0.5)] {
                                module.update(&context(time));
                                expected.push(upstream_truth(
                                    &config,
                                    &guidance,
                                    inertia,
                                    &rw_config,
                                    &rw_speeds,
                                    &availability,
                                    dt,
                                    &mut sigma_integral,
                                ));
                                assert_components_close(
                                    module.cmd_torque_out_msg.read().torque_request_body_nm,
                                    *expected.last().unwrap(),
                                    1.0e-8,
                                );
                            }

                            module.reset(1);
                            sigma_integral = Vector3::zeros();
                            for (time, dt) in [(1_500_000_000, 0.0), (2_000_000_000, 0.5)] {
                                module.update(&context(time));
                                let truth = upstream_truth(
                                    &config,
                                    &guidance,
                                    inertia,
                                    &rw_config,
                                    &rw_speeds,
                                    &availability,
                                    dt,
                                    &mut sigma_integral,
                                );
                                assert_components_close(
                                    module.cmd_torque_out_msg.read().torque_request_body_nm,
                                    truth,
                                    1.0e-8,
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn publishes_integral_feedback_torque_separately() {
        let guidance = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(0.1, -0.2, 0.3),
            ..Default::default()
        });
        let vehicle = Output::new(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: Matrix3::identity(),
            ..Default::default()
        });
        let mut module = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFeedback".to_string(),
            k: 1.0,
            ki: 0.5,
            p: 2.0,
            integral_limit: 10.0,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
        });
        module.guid_in_msg.connect(guidance.slot());
        module.veh_config_in_msg.connect(vehicle.slot());
        module.init();
        module.update(&context(1_000_000_000));

        let expected = -module.config.p * module.config.ki * module.z;
        assert!(
            (module
                .int_feedback_torque_out_msg
                .read()
                .torque_request_body_nm
                - expected)
                .norm()
                <= 1.0e-12
        );
    }

    #[test]
    fn saturates_each_integral_component_and_preserves_z_across_reset() {
        let guidance = Output::new(AttitudeGuidanceMsg {
            sigma_br: Vector3::new(1.0, -1.0, 2.0),
            ..Default::default()
        });
        let vehicle = Output::new(VehicleConfigMsg::default());
        let mut module = MrpFeedback::new(MrpFeedbackConfig {
            name: "mrpFeedback".to_string(),
            k: 1.0,
            ki: 1.0,
            p: 1.0,
            integral_limit: 0.05,
            known_torque_body_nm: Vector3::zeros(),
            control_law_type: 0,
        });
        module.guid_in_msg.connect(guidance.slot());
        module.veh_config_in_msg.connect(vehicle.slot());
        module.init();
        module.update(&context(1_000_000_000));
        module.update(&context(2_000_000_000));

        assert_components_close(
            module.int_sigma,
            Vector3::new(0.05, -0.05, 0.05),
            f64::EPSILON,
        );
        let z_before_reset = module.z;
        module.reset(2_000_000_000);

        assert_eq!(module.int_sigma, Vector3::zeros());
        assert_eq!(module.z, z_before_reset);
    }

    fn assert_components_close(actual: Vector3<f64>, expected: Vector3<f64>, tolerance: f64) {
        for index in 0..3 {
            assert!((actual[index] - expected[index]).abs() <= tolerance);
        }
    }
}
