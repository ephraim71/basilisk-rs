use crate::messages::{
    ArrayMotorTorqueMsg, ArrayMotorVoltageMsg, Input, MAX_EFF_COUNT, Output, RwArrayConfigMsg,
    RwAvailabilityMsg, RwSpeedMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct RwMotorVoltageConfig {
    pub name: String,
    /// Minimum nonzero motor voltage, corresponding to the actuator dead band.
    pub v_min_v: f64,
    /// Maximum magnitude of the commanded motor voltage.
    pub v_max_v: f64,
    /// Proportional gain used by the optional torque-tracking loop.
    pub k: f64,
}

#[derive(Clone, Debug)]
pub struct RwMotorVoltage {
    pub config: RwMotorVoltageConfig,
    pub voltage_out_msg: Output<ArrayMotorVoltageMsg>,
    pub torque_in_msg: Input<ArrayMotorTorqueMsg>,
    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    pub rw_speed_in_msg: Input<RwSpeedMsg>,
    pub rw_avail_in_msg: Input<RwAvailabilityMsg>,
    rw_config_params: RwArrayConfigMsg,
    rw_speed_old_radps: [f64; MAX_EFF_COUNT],
    prior_time_nanos: u64,
    reset_flag: bool,
}

impl RwMotorVoltage {
    pub fn new(config: RwMotorVoltageConfig) -> Self {
        Self {
            config,
            voltage_out_msg: Output::default(),
            torque_in_msg: Input::default(),
            rw_params_in_msg: Input::default(),
            rw_speed_in_msg: Input::default(),
            rw_avail_in_msg: Input::default(),
            rw_config_params: RwArrayConfigMsg::default(),
            rw_speed_old_radps: [0.0; MAX_EFF_COUNT],
            prior_time_nanos: 0,
            reset_flag: true,
        }
    }

    /// Reset the cached configuration and closed-loop speed history.
    ///
    /// This is the Rust equivalent of upstream `Reset_rwMotorVoltage`. The
    /// simulation framework invokes it through both [`Module::init`] and its
    /// reset lifecycle, and callers may also invoke it directly.
    pub fn reset(&mut self) {
        if !self.rw_params_in_msg.is_connected() {
            log::error!("rwMotorVoltage.rw_params_in_msg is not connected");
        }

        // The upstream module intentionally caches this nominally static
        // message only on reset rather than reading it every control update.
        self.rw_config_params = self.rw_params_in_msg.read();
        if self.rw_config_params.num_rw > MAX_EFF_COUNT {
            log::error!(
                "rwMotorVoltage.num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
            self.rw_config_params.num_rw = MAX_EFF_COUNT;
        }
        self.rw_speed_old_radps.fill(0.0);
        self.reset_flag = true;
        self.prior_time_nanos = 0;
        self.voltage_out_msg.write(ArrayMotorVoltageMsg::default());
    }

    fn update_closed_loop_torque(
        &mut self,
        call_time_nanos: u64,
        torque_command: &mut ArrayMotorTorqueMsg,
        wheel_speeds: &RwSpeedMsg,
        availability: &RwAvailabilityMsg,
    ) {
        // `prior_time_nanos == 0` is deliberately retained as the upstream
        // initialization sentinel. In a simulation starting at t=0, this
        // postpones history processing by an additional update.
        if self.prior_time_nanos != 0 {
            let dt_seconds = diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos);

            for wheel_index in 0..self.rw_config_params.num_rw {
                if availability.is_available(wheel_index) && !self.reset_flag {
                    let omega_dot_radps2 = (wheel_speeds.wheel_speeds_radps[wheel_index]
                        - self.rw_speed_old_radps[wheel_index])
                        / dt_seconds;
                    torque_command.motor_torque_nm[wheel_index] -= self.config.k
                        * (self.rw_config_params.spin_axis_inertias_kg_m2[wheel_index]
                            * omega_dot_radps2
                            - torque_command.motor_torque_nm[wheel_index]);
                }

                self.rw_speed_old_radps[wheel_index] = wheel_speeds.wheel_speeds_radps[wheel_index];
            }

            self.reset_flag = false;
        }

        self.prior_time_nanos = call_time_nanos;
    }

    fn map_torque_to_voltage(
        &self,
        torque_command: &ArrayMotorTorqueMsg,
        availability: &RwAvailabilityMsg,
    ) -> ArrayMotorVoltageMsg {
        let mut output = ArrayMotorVoltageMsg::default();

        for wheel_index in 0..self.rw_config_params.num_rw {
            let mut voltage_v = 0.0;

            if availability.is_available(wheel_index) {
                voltage_v = (self.config.v_max_v - self.config.v_min_v)
                    / self.rw_config_params.max_motor_torques_nm[wheel_index]
                    * torque_command.motor_torque_nm[wheel_index];

                if voltage_v > 0.0 {
                    voltage_v += self.config.v_min_v;
                }
                if voltage_v < 0.0 {
                    voltage_v -= self.config.v_min_v;
                }
            }

            // Keep these as two sequential comparisons rather than `clamp`:
            // this exactly matches upstream behavior for all configured values.
            if voltage_v > self.config.v_max_v {
                voltage_v = self.config.v_max_v;
            }
            if voltage_v < -self.config.v_max_v {
                voltage_v = -self.config.v_max_v;
            }

            output.voltage_v[wheel_index] = voltage_v;
        }

        output
    }
}

impl Module for RwMotorVoltage {
    fn init(&mut self) {
        self.voltage_out_msg.write(ArrayMotorVoltageMsg::default());
        self.reset();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        RwMotorVoltage::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        if !self.torque_in_msg.is_connected() {
            log::error!("rwMotorVoltage.torque_in_msg is not connected");
        }

        let mut torque_command = self.torque_in_msg.read();
        let speed_input_connected = self.rw_speed_in_msg.is_connected();
        let wheel_speeds = if speed_input_connected {
            self.rw_speed_in_msg.read()
        } else {
            RwSpeedMsg::default()
        };
        let availability = if self.rw_avail_in_msg.is_connected() {
            self.rw_avail_in_msg.read()
        } else {
            RwAvailabilityMsg::default()
        };

        if speed_input_connected {
            self.update_closed_loop_torque(
                context.current_sim_nanos,
                &mut torque_command,
                &wheel_speeds,
                &availability,
            );
        }

        let voltage_output = self.map_torque_to_voltage(&torque_command, &availability);
        self.voltage_out_msg.write(voltage_output);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        ArrayMotorTorqueMsg, Output, RwArrayConfigMsg, RwAvailability, RwAvailabilityMsg,
        RwSpeedMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{RwMotorVoltage, RwMotorVoltageConfig};

    const FUNCTIONAL_TOLERANCE: f64 = 1.0e-10;
    const HALF_SECOND_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn rw_configuration() -> RwArrayConfigMsg {
        let mut configuration = RwArrayConfigMsg {
            num_rw: 4,
            ..Default::default()
        };
        configuration.spin_axes_body[0] = Vector3::x();
        configuration.spin_axes_body[1] = Vector3::y();
        configuration.spin_axes_body[2] = Vector3::z();
        configuration.spin_axes_body[3] = Vector3::new(1.0, 1.0, 1.0).normalize();
        configuration.spin_axis_inertias_kg_m2[..4].fill(0.1);
        configuration.max_motor_torques_nm[..4].fill(0.2);
        configuration
    }

    fn motor_torque(values: [f64; 4]) -> ArrayMotorTorqueMsg {
        let mut message = ArrayMotorTorqueMsg::default();
        message.motor_torque_nm[..4].copy_from_slice(&values);
        message
    }

    fn wheel_speeds(values: [f64; 4]) -> RwSpeedMsg {
        let mut message = RwSpeedMsg::default();
        message.wheel_speeds_radps[..4].copy_from_slice(&values);
        message
    }

    fn make_module(
        torques: [f64; 4],
        use_availability: bool,
        use_torque_loop: bool,
    ) -> (RwMotorVoltage, Option<Output<RwSpeedMsg>>) {
        let rw_config_output = Output::new(rw_configuration());
        let torque_output = Output::new(motor_torque(torques));
        let mut module = RwMotorVoltage::new(RwMotorVoltageConfig {
            name: "rwMotorVoltage".to_string(),
            v_min_v: 1.0,
            v_max_v: 11.0,
            k: if use_torque_loop { 1.5 } else { 0.0 },
        });
        module.rw_params_in_msg.connect(rw_config_output.slot());
        module.torque_in_msg.connect(torque_output.slot());

        if use_availability {
            let mut availability = RwAvailabilityMsg::default();
            availability.wheel_availability[2] = RwAvailability::Unavailable;
            let availability_output = Output::new(availability);
            module.rw_avail_in_msg.connect(availability_output.slot());
        }

        let speed_output = use_torque_loop.then(|| {
            let output = Output::new(wheel_speeds([1.0, 2.0, 1.5, -3.0]));
            module.rw_speed_in_msg.connect(output.slot());
            output
        });

        (module, speed_output)
    }

    fn run_upstream_case(
        torques: [f64; 4],
        use_availability: bool,
        use_torque_loop: bool,
    ) -> Vec<[f64; 4]> {
        let (mut module, speed_output) = make_module(torques, use_availability, use_torque_loop);
        module.init();

        let mut samples = Vec::with_capacity(7);
        for call_time in [0, HALF_SECOND_NANOS, 2 * HALF_SECOND_NANOS] {
            module.update(&context(call_time));
            samples.push(first_four_voltages(&module));
        }

        if let Some(speed_output) = &speed_output {
            speed_output.write(wheel_speeds([1.1, 2.1, 1.1, -4.1]));
        }
        module.update(&context(3 * HALF_SECOND_NANOS));
        samples.push(first_four_voltages(&module));

        module.reset();
        for call_time in [
            4 * HALF_SECOND_NANOS,
            5 * HALF_SECOND_NANOS,
            6 * HALF_SECOND_NANOS,
        ] {
            module.update(&context(call_time));
            samples.push(first_four_voltages(&module));
        }

        samples
    }

    fn first_four_voltages(module: &RwMotorVoltage) -> [f64; 4] {
        let output = module.voltage_out_msg.read();
        output.voltage_v[..4]
            .try_into()
            .expect("four configured reaction wheels")
    }

    fn assert_samples_match(actual: &[[f64; 4]], expected: &[[f64; 4]]) {
        assert_eq!(actual.len(), expected.len());
        for (sample_index, (actual_sample, expected_sample)) in
            actual.iter().zip(expected).enumerate()
        {
            for component_index in 0..4 {
                assert!(
                    (actual_sample[component_index] - expected_sample[component_index]).abs()
                        <= FUNCTIONAL_TOLERANCE,
                    "sample {sample_index}, wheel {component_index}: expected {}, got {}",
                    expected_sample[component_index],
                    actual_sample[component_index]
                );
            }
        }
    }

    #[test]
    fn matches_upstream_open_loop_case() {
        let actual = run_upstream_case([0.05, 0.0, -0.15, -0.2], false, false);
        let expected = [[3.5, 0.0, -8.5, -11.0]; 7];
        assert_samples_match(&actual, &expected);
    }

    #[test]
    fn matches_upstream_voltage_saturation_case() {
        let actual = run_upstream_case([0.5, 0.0, -0.15, -0.5], false, false);
        let expected = [[11.0, 0.0, -8.5, -11.0]; 7];
        assert_samples_match(&actual, &expected);
    }

    #[test]
    fn matches_upstream_availability_case() {
        let actual = run_upstream_case([0.05, 0.0, -0.15, -0.2], true, false);
        let expected = [[3.5, 0.0, 0.0, -11.0]; 7];
        assert_samples_match(&actual, &expected);
    }

    #[test]
    fn matches_upstream_closed_loop_case() {
        let actual = run_upstream_case([0.05, 0.0, -0.15, -0.2], false, true);
        let expected = [
            [3.5, 0.0, -8.5, -11.0],
            [3.5, 0.0, -8.5, -11.0],
            [3.5, 0.0, -8.5, -11.0],
            [5.75, -2.5, -11.0, -9.5],
            [3.5, 0.0, -8.5, -11.0],
            [3.5, 0.0, -8.5, -11.0],
            [7.25, 0.0, -11.0, -11.0],
        ];
        assert_samples_match(&actual, &expected);
    }

    #[test]
    fn matches_upstream_reset_zero_output_test() {
        let (mut module, _) = make_module([0.5, 0.0, -0.15, -0.5], false, false);
        module.init();
        module.update(&context(0));
        assert!(module.voltage_out_msg.read().voltage_v[0].abs() > 1.0e-3);

        module.reset();

        assert!(
            module
                .voltage_out_msg
                .read()
                .voltage_v
                .iter()
                .all(|voltage| voltage.abs() <= 1.0e-3)
        );
    }
}
