//! Nonlinear body-rate tracking servo: port of upstream Basilisk
//! `rateServoFullNonlinear`.
//!
//! A kinematic steering law ([`super::mrp_steering`], [`super::prv_steering`])
//! commands a desired body frame `B*` through its rate relative to the
//! reference frame, `ω_B*/R`, and that rate's body-frame derivative
//! `ω'_B*/R`. With the rate tracking error `δω = ω_B/N - ω_B*/N` and its
//! integral `z`, the servo requests the control torque
//!
//! ```text
//! L_r = -P δω - K_i z + ω_B*/N × ([I] ω_B/N + Σ h_s,i ĝ_s,i)
//!       + [I](ω'_B*/R + ω̇_R/N - ω_B/N × ω_R/N) - L
//! ```
//!
//! where `h_s,i = J_s,i (ĝ_s,i · ω_B/N + Ω_i)` is the spin-axis momentum of
//! each available reaction wheel and `L` a known external torque. Each
//! component of `z` is limited to `integral_limit` to avoid wind-up, and a
//! non-positive `ki` turns the integral feedback off. The wheel terms enter
//! only when the optional RW configuration message is connected. The inertia
//! and the RW configuration are read at reset only.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, RateCommandMsg,
    RwArrayConfigMsg, RwAvailabilityMsg, RwSpeedMsg, VehicleConfigMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`RateServoFullNonlinear`]. `Default` is
/// upstream's zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct RateServoFullNonlinearConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `P` \[N m s\]: rate error feedback gain.
    pub p: f64,
    /// Upstream `Ki` \[N m\]: integral feedback gain on the rate error; a
    /// non-positive value turns the integral feedback off.
    pub ki: f64,
    /// Upstream `knownTorquePntB_B` \[N m\]: known external torque, body
    /// components.
    pub known_torque_body_nm: Vector3<f64>,
    /// Upstream `integralLimit` (labelled \[N m\] upstream): limit on each
    /// component of the rate error integral `z` \[rad\].
    pub integral_limit: f64,
}

impl Default for RateServoFullNonlinearConfig {
    fn default() -> Self {
        Self {
            name: "rateServoFullNonlinear".to_string(),
            p: 0.0,
            ki: 0.0,
            known_torque_body_nm: Vector3::zeros(),
            integral_limit: 0.0,
        }
    }
}

/// Nonlinear rate servo: upstream `rateServoFullNonlinear`.
#[derive(Clone, Debug)]
pub struct RateServoFullNonlinear {
    /// User-settable parameters.
    pub config: RateServoFullNonlinearConfig,
    /// Upstream `guidInMsg`: attitude guidance (body rate and reference
    /// motion).
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `vehConfigInMsg`: vehicle configuration (inertia), read at
    /// reset.
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `rwSpeedsInMsg` (optional): wheel speeds; required when
    /// `rw_params_in_msg` is connected.
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Upstream `rwAvailInMsg` (optional): wheel availability; every wheel is
    /// available when it is not connected.
    pub rw_avail_in_msg: Input<RwAvailabilityMsg>,
    /// Upstream `rwParamsInMsg` (optional): RW configuration, read at reset.
    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    /// Upstream `rateSteeringInMsg`: desired rate from the steering law.
    pub rate_steering_in_msg: Input<RateCommandMsg>,
    /// Upstream `cmdTorqueOutMsg`: commanded control torque.
    pub cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `priorTime` \[ns\]; zero means no previous update.
    prior_time_nanos: u64,
    /// Upstream `z` \[rad\]: integral of the rate tracking error.
    z: Vector3<f64>,
    /// Upstream `ISCPntB_B` \[kg m^2\]: inertia read at reset.
    inertia_kg_m2: Matrix3<f64>,
    /// Upstream `rwConfigParams`: RW configuration read at reset.
    rw_config_params: RwArrayConfigMsg,
}

impl RateServoFullNonlinear {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: RateServoFullNonlinearConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            veh_config_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_avail_in_msg: Input::default(),
            rw_params_in_msg: Input::default(),
            rate_steering_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
            prior_time_nanos: 0,
            z: Vector3::zeros(),
            inertia_kg_m2: Matrix3::zeros(),
            rw_config_params: RwArrayConfigMsg::default(),
        }
    }

    /// Upstream `rwConfigParams`: the RW configuration read at the last reset.
    /// Its `num_rw` is zero when `rw_params_in_msg` is not connected.
    pub fn rw_config_params(&self) -> &RwArrayConfigMsg {
        &self.rw_config_params
    }

    /// Upstream `Reset_rateServoFullNonlinear`.
    fn reset_servo(&mut self) {
        if self.rw_params_in_msg.is_connected() && !self.rw_speeds_in_msg.is_connected() {
            log::error!(
                "rateServoFullNonlinear.rw_speeds_in_msg is not connected while rw_params_in_msg is connected"
            );
        }
        if !self.guid_in_msg.is_connected() {
            log::error!("rateServoFullNonlinear.guid_in_msg is not connected");
        }
        if !self.veh_config_in_msg.is_connected() {
            log::error!("rateServoFullNonlinear.veh_config_in_msg is not connected");
        }
        if !self.rate_steering_in_msg.is_connected() {
            log::error!("rateServoFullNonlinear.rate_steering_in_msg is not connected");
        }

        self.inertia_kg_m2 = self.veh_config_in_msg.read().inertia_about_point_b_kg_m2;
        // As upstream, an unconnected RW message only zeroes the wheel count.
        self.rw_config_params.num_rw = 0;
        if self.rw_params_in_msg.is_connected() {
            self.rw_config_params = self.rw_params_in_msg.read();
        }
        if self.rw_config_params.num_rw > MAX_EFF_COUNT {
            log::error!(
                "rateServoFullNonlinear: num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); only the first {MAX_EFF_COUNT} wheels are used"
            );
        }

        self.z = Vector3::zeros();
        self.prior_time_nanos = 0;
    }

    /// Control period \[s\] since the previous update.
    fn control_period_s(&mut self, call_time_nanos: u64) -> f64 {
        // Upstream's zero-time sentinel, kept exactly: an update at t = 0
        // leaves the next update's period at zero as well.
        let dt_s = if self.prior_time_nanos == 0 {
            0.0
        } else {
            diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos)
        };
        self.prior_time_nanos = call_time_nanos;
        dt_s
    }

    /// Integrates the rate tracking error into `z`, limiting each component.
    fn integrate_rate_error(&mut self, omega_bbast_b: Vector3<f64>, dt_s: f64) {
        if self.config.ki > 0.0 {
            self.z = omega_bbast_b * dt_s + self.z;
            let limit = self.config.integral_limit;
            for component in self.z.iter_mut() {
                let magnitude = component.abs();
                if magnitude > limit {
                    *component *= limit / magnitude;
                }
            }
        } else {
            self.z = Vector3::zeros();
        }
    }

    /// Body angular momentum `[I] ω_B/N + Σ h_s,i ĝ_s,i` \[N m s\] of the hub
    /// and the available wheels.
    fn angular_momentum_nms(
        &self,
        omega_bn_b: Vector3<f64>,
        wheel_speeds: &RwSpeedMsg,
        availability: &RwAvailabilityMsg,
    ) -> Vector3<f64> {
        let rw = &self.rw_config_params;
        (0..rw.num_rw.min(MAX_EFF_COUNT))
            .filter(|&index| availability.is_available(index))
            .fold(self.inertia_kg_m2 * omega_bn_b, |momentum, index| {
                let spin_axis = rw.spin_axes_body[index];
                let wheel_momentum = rw.spin_axis_inertias_kg_m2[index]
                    * (omega_bn_b.dot(&spin_axis) + wheel_speeds.wheel_speeds_radps[index]);
                spin_axis * wheel_momentum + momentum
            })
    }

    /// Upstream `Update_rateServoFullNonlinear`: the control torque \[N m\].
    fn control_torque(&mut self, call_time_nanos: u64) -> Vector3<f64> {
        let dt_s = self.control_period_s(call_time_nanos);
        let guidance = self.guid_in_msg.read();
        let rate_command = self.rate_steering_in_msg.read();
        let (wheel_speeds, availability) = if self.rw_config_params.num_rw > 0 {
            let availability = if self.rw_avail_in_msg.is_connected() {
                self.rw_avail_in_msg.read()
            } else {
                RwAvailabilityMsg::default()
            };
            (self.rw_speeds_in_msg.read(), availability)
        } else {
            (RwSpeedMsg::default(), RwAvailabilityMsg::default())
        };

        let omega_rn_b = guidance.omega_rn_b_radps;
        let omega_bn_b = guidance.omega_br_b_radps + omega_rn_b;
        let omega_bastn_b = rate_command.omega_bast_r_b_radps + omega_rn_b;
        let omega_bbast_b = omega_bn_b - omega_bastn_b;
        self.integrate_rate_error(omega_bbast_b, dt_s);

        let config = &self.config;
        let feedback = self.z * config.ki + omega_bbast_b * config.p;
        let gyroscopic = omega_bastn_b.cross(&self.angular_momentum_nms(
            omega_bn_b,
            &wheel_speeds,
            &availability,
        ));
        let feedforward = self.inertia_kg_m2
            * ((guidance.domega_rn_b_radps2 - omega_bn_b.cross(&omega_rn_b))
                + rate_command.omega_prime_bast_r_b_radps2);
        -(config.known_torque_body_nm + (feedback - gyroscopic - feedforward))
    }
}

impl Module for RateServoFullNonlinear {
    fn init(&mut self) {
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.reset_servo();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_servo();
    }

    fn update(&mut self, context: &SimulationContext) {
        let torque_request_body_nm = self.control_torque(context.current_sim_nanos);
        self.cmd_torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm,
        });
    }
}

/// Inputs and schedule shared by the upstream unit tests that run this servo
/// (`test_rateServoFullNonlinear.py`, `test_MRP_steeringInt.py`,
/// `test_PRV_Steering.py`) and the other attitude control tests that use the
/// same guidance message.
#[cfg(test)]
pub(crate) mod test_support {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{RateServoFullNonlinear, RateServoFullNonlinearConfig};
    use crate::messages::{
        AttitudeGuidanceMsg, Output, RateCommandMsg, RwArrayConfigMsg, RwAvailability,
        RwAvailabilityMsg, RwSpeedMsg, VehicleConfigMsg,
    };
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    pub(crate) const TASK_NANOS: u64 = 500_000_000;

    /// The spin axes of upstream's four-wheel test configuration.
    pub(crate) const UPSTREAM_SPIN_AXES: [[f64; 3]; 4] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.5773502691896258, 0.5773502691896258, 0.5773502691896258],
    ];

    pub(crate) fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The guidance message of the upstream attitude control tests.
    pub(crate) fn guidance() -> AttitudeGuidanceMsg {
        AttitudeGuidanceMsg {
            sigma_br: Vector3::new(0.3, -0.5, 0.7),
            omega_br_b_radps: Vector3::new(0.010, -0.020, 0.015),
            omega_rn_b_radps: Vector3::new(-0.02, -0.01, 0.005),
            domega_rn_b_radps2: Vector3::new(0.0002, 0.0003, 0.0001),
        }
    }

    /// The vehicle configuration of the upstream attitude control tests.
    pub(crate) fn vehicle_config() -> VehicleConfigMsg {
        VehicleConfigMsg {
            inertia_about_point_b_kg_m2: Matrix3::from_row_slice(&[
                1000., 0., 0., 0., 800., 0., 0., 0., 800.,
            ]),
            ..Default::default()
        }
    }

    /// Upstream's wheel speeds `[10, 25, 50, 100]` rad/s.
    pub(crate) fn wheel_speeds() -> RwSpeedMsg {
        let mut speeds = RwSpeedMsg::default();
        speeds.wheel_speeds_radps[..4].copy_from_slice(&[10.0, 25.0, 50.0, 100.0]);
        speeds
    }

    /// Upstream's RW configuration: four wheels with `J_s = 0.1` kg m^2 on
    /// `spin_axes`, of which the first `num_rw` are declared.
    pub(crate) fn rw_config(spin_axes: [[f64; 3]; 4], num_rw: usize) -> RwArrayConfigMsg {
        let mut config = RwArrayConfigMsg {
            num_rw,
            ..Default::default()
        };
        for (axis, upstream) in config.spin_axes_body.iter_mut().zip(spin_axes) {
            *axis = Vector3::from(upstream);
        }
        config.spin_axis_inertias_kg_m2[..4].fill(0.1);
        config
    }

    /// The servo configuration of `test_MRP_steeringInt.py` and
    /// `test_PRV_Steering.py`, set after construction as upstream does.
    pub(crate) fn steering_test_servo() -> RateServoFullNonlinear {
        let mut servo = RateServoFullNonlinear::new(RateServoFullNonlinearConfig::default());
        servo.config.ki = 0.01;
        servo.config.p = 150.0;
        servo.config.integral_limit = 2.0 / servo.config.ki * 0.1;
        servo.config.known_torque_body_nm = Vector3::zeros();
        servo
    }

    /// The servo inputs other than the rate command, as the upstream steering
    /// tests publish them.
    pub(crate) struct ServoInputs {
        pub(crate) guidance: Output<AttitudeGuidanceMsg>,
        pub(crate) vehicle_config: Output<VehicleConfigMsg>,
        pub(crate) wheel_speeds: Output<RwSpeedMsg>,
        pub(crate) rw_params: Output<RwArrayConfigMsg>,
        pub(crate) rw_availability: Output<RwAvailabilityMsg>,
    }

    impl ServoInputs {
        /// All wheels available, as the upstream steering tests set.
        pub(crate) fn new(guidance: AttitudeGuidanceMsg, rw_params: RwArrayConfigMsg) -> Self {
            Self {
                guidance: Output::new(guidance),
                vehicle_config: Output::new(vehicle_config()),
                wheel_speeds: Output::new(wheel_speeds()),
                rw_params: Output::new(rw_params),
                rw_availability: Output::new(RwAvailabilityMsg::default()),
            }
        }

        pub(crate) fn connect(&self, servo: &mut RateServoFullNonlinear) {
            self.guidance.connect_to(&mut servo.guid_in_msg);
            self.vehicle_config.connect_to(&mut servo.veh_config_in_msg);
            self.wheel_speeds.connect_to(&mut servo.rw_speeds_in_msg);
            self.rw_params.connect_to(&mut servo.rw_params_in_msg);
            self.rw_availability.connect_to(&mut servo.rw_avail_in_msg);
        }
    }

    /// Upstream's schedule: a 0.5 s task run to 1 s, `servo.Reset(1)`, then
    /// run to 2 s. Each tick updates `steering` (when given) before the servo,
    /// as upstream adds them to the task in that order, and records the
    /// servo's torque.
    pub(crate) fn run_upstream_schedule(
        mut steering: Option<&mut dyn Module>,
        servo: &mut RateServoFullNonlinear,
    ) -> Vec<Vector3<f64>> {
        if let Some(steering) = steering.as_deref_mut() {
            steering.init();
        }
        servo.init();
        let mut samples = Vec::new();
        for tick in 0..=4 {
            if tick == 3 {
                servo.reset(&context(1));
            }
            let context = context(tick * TASK_NANOS);
            if let Some(steering) = steering.as_deref_mut() {
                steering.update(&context);
            }
            servo.update(&context);
            samples.push(servo.cmd_torque_out_msg.read().torque_request_body_nm);
        }
        samples
    }

    /// Upstream `findTrueTorques` of `test_rateServoFullNonlinear.py` and
    /// `test_MRP_steeringInt.py`: one torque per sample of
    /// [`run_upstream_schedule`], whose control periods are
    /// `[0, 0, 0.5, 0, 0.5]` s. As upstream, it limits only positive `z`
    /// components, which is all the upstream inputs reach.
    pub(crate) fn upstream_true_torques(
        config: &RateServoFullNonlinearConfig,
        guidance: &AttitudeGuidanceMsg,
        rw: &RwArrayConfigMsg,
        availability: &RwAvailabilityMsg,
        rate_command: &RateCommandMsg,
    ) -> Vec<Vector3<f64>> {
        let inertia = vehicle_config().inertia_about_point_b_kg_m2;
        let wheel_speeds = wheel_speeds();
        let omega_bn_b = guidance.omega_br_b_radps + guidance.omega_rn_b_radps;
        let omega_bastn_b = rate_command.omega_bast_r_b_radps + guidance.omega_rn_b_radps;
        let omega_bbast_b = omega_bn_b - omega_bastn_b;

        let mut z = Vector3::zeros();
        [0.0, 0.0, 0.5, 0.0, 0.5]
            .into_iter()
            .map(|dt| {
                if dt == 0.0 {
                    z = Vector3::zeros();
                }
                if config.ki > 0.0 && config.integral_limit.abs() > 0.0 {
                    z += dt * omega_bbast_b;
                    for component in z.iter_mut() {
                        if *component > config.integral_limit {
                            *component = component.signum() * config.integral_limit;
                        }
                    }
                } else {
                    z = Vector3::zeros();
                }

                let mut wheel_momentum = Vector3::zeros();
                for index in 0..rw.num_rw {
                    if availability.wheel_availability[index] == RwAvailability::Available {
                        let axis = rw.spin_axes_body[index];
                        wheel_momentum += axis
                            * (rw.spin_axis_inertias_kg_m2[index]
                                * (omega_bn_b.dot(&axis) + wheel_speeds.wheel_speeds_radps[index]));
                    }
                }

                let lr = config.ki * z + config.p * omega_bbast_b
                    - omega_bastn_b.cross(&(inertia * omega_bn_b + wheel_momentum))
                    - inertia
                        * (rate_command.omega_prime_bast_r_b_radps2 + guidance.domega_rn_b_radps2
                            - omega_bn_b.cross(&guidance.omega_rn_b_radps));
                -(lr + config.known_torque_body_nm)
            })
            .collect()
    }

    /// Upstream `isArrayEqual`: absolute tolerance per component, NaN fails.
    pub(crate) fn assert_array_equal(
        actual: Vector3<f64>,
        truth: Vector3<f64>,
        accuracy: f64,
        what: &str,
    ) {
        for (index, (value, expected)) in actual.iter().zip(truth.iter()).enumerate() {
            assert!(
                (value - expected).abs() <= accuracy,
                "{what}[{index}] = {value:.17e}, truth {expected:.17e}"
            );
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use super::test_support::{
        TASK_NANOS, UPSTREAM_SPIN_AXES, assert_array_equal, context, guidance,
        run_upstream_schedule, rw_config, upstream_true_torques, vehicle_config, wheel_speeds,
    };
    use super::{RateServoFullNonlinear, RateServoFullNonlinearConfig};
    use crate::Module;
    use crate::messages::{
        AttitudeGuidanceMsg, Output, RateCommandMsg, RwArrayConfigMsg, RwAvailability,
        RwAvailabilityMsg, VehicleConfigMsg,
    };

    /// `useRwAvailability` of `test_rateServoFullNonlinear.py`.
    #[derive(Clone, Copy, Debug)]
    enum RwAvailabilityCase {
        /// No availability message: every wheel counts as available.
        NoMessage,
        On,
        Off,
    }

    /// `test_rate_servo_full_nonlinear`, all 96 parametrizations, with
    /// upstream's Python truth and `accuracy = 1e-8`. Upstream connects the RW
    /// configuration and speed messages in every case, including
    /// `rwNum = 0`.
    #[test]
    fn matches_all_upstream_rate_servo_full_nonlinear_parameter_combinations() {
        let omegap_cases = [
            Vector3::new(1.87766650e-04, -3.91233583e-05, 3.56369489e-05),
            Vector3::zeros(),
        ];
        let omega_cases = [
            Vector3::new(-2.23886891e-02, 2.47942516e-02, -2.55601849e-02),
            Vector3::zeros(),
        ];
        let mut cases = 0;
        for rw_num in [4, 0] {
            for int_gain in [0.01, -1.0] {
                for omegap_bast_r_b in omegap_cases {
                    for omega_bast_r_b in omega_cases {
                        for integral_limit in [0.0, 20.0] {
                            for availability_case in [
                                RwAvailabilityCase::NoMessage,
                                RwAvailabilityCase::On,
                                RwAvailabilityCase::Off,
                            ] {
                                let mut module = RateServoFullNonlinear::new(
                                    RateServoFullNonlinearConfig::default(),
                                );
                                module.config.ki = int_gain;
                                module.config.p = 150.0;
                                module.config.integral_limit = integral_limit;
                                module.config.known_torque_body_nm = Vector3::new(1.0, 1.0, 1.0);

                                let guidance_out = Output::new(guidance());
                                let vehicle_out = Output::new(vehicle_config());
                                let speeds_out = Output::new(wheel_speeds());
                                let rw = rw_config(UPSTREAM_SPIN_AXES, rw_num);
                                let rw_out = Output::new(rw.clone());
                                let mut availability = RwAvailabilityMsg::default();
                                if matches!(availability_case, RwAvailabilityCase::Off) {
                                    availability.wheel_availability[..4]
                                        .fill(RwAvailability::Unavailable);
                                }
                                let availability_out = Output::new(availability.clone());
                                let rate_command = RateCommandMsg {
                                    omega_bast_r_b_radps: omega_bast_r_b,
                                    omega_prime_bast_r_b_radps2: omegap_bast_r_b,
                                };
                                let rate_out = Output::new(rate_command.clone());

                                guidance_out.connect_to(&mut module.guid_in_msg);
                                vehicle_out.connect_to(&mut module.veh_config_in_msg);
                                rw_out.connect_to(&mut module.rw_params_in_msg);
                                speeds_out.connect_to(&mut module.rw_speeds_in_msg);
                                rate_out.connect_to(&mut module.rate_steering_in_msg);
                                if !matches!(availability_case, RwAvailabilityCase::NoMessage) {
                                    availability_out.connect_to(&mut module.rw_avail_in_msg);
                                }

                                let samples = run_upstream_schedule(None, &mut module);
                                let truth = upstream_true_torques(
                                    &module.config,
                                    &guidance(),
                                    &rw,
                                    &availability,
                                    &rate_command,
                                );
                                assert_eq!(samples.len(), truth.len());
                                for (index, (sample, expected)) in
                                    samples.iter().zip(&truth).enumerate()
                                {
                                    assert_array_equal(
                                        *sample,
                                        *expected,
                                        1e-8,
                                        &format!(
                                            "rwNum {rw_num}, Ki {int_gain}, omegap {omegap_bast_r_b:?}, \
                                             omega {omega_bast_r_b:?}, limit {integral_limit}, \
                                             {availability_case:?}, sample {index}"
                                        ),
                                    );
                                }
                                cases += 1;
                            }
                        }
                    }
                }
            }
        }
        assert_eq!(cases, 96);
    }

    /// Not upstream: each component of `z` is limited in magnitude, for both
    /// signs. Without inertia or wheels the torque is `-(P δω + Ki z)`.
    #[test]
    fn limits_each_integral_component_in_magnitude() {
        let guidance_out = Output::new(AttitudeGuidanceMsg {
            omega_br_b_radps: Vector3::new(1.0, -2.0, 0.5),
            ..Default::default()
        });
        let rate_out = Output::new(RateCommandMsg::default());
        let mut module = RateServoFullNonlinear::new(RateServoFullNonlinearConfig {
            p: 2.0,
            ki: 0.5,
            integral_limit: 0.6,
            ..Default::default()
        });
        guidance_out.connect_to(&mut module.guid_in_msg);
        rate_out.connect_to(&mut module.rate_steering_in_msg);
        module.init();
        // The t = 0 sentinel makes the first two periods zero.
        for seconds in 0..=2 {
            module.update(&context(seconds * 1_000_000_000));
        }
        assert_eq!(module.z, Vector3::new(0.6, -0.6, 0.5));
        let expected = -(Vector3::new(1.0, -2.0, 0.5) * 2.0 + Vector3::new(0.6, -0.6, 0.5) * 0.5);
        assert_eq!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            expected
        );

        module.reset(&context(2_000_000_000));
        assert_eq!(module.z, Vector3::zeros());
    }

    /// Not upstream: a non-positive integral gain clears an accumulated
    /// integral on the next update.
    #[test]
    fn a_non_positive_integral_gain_clears_the_integral_state() {
        let guidance_out = Output::new(AttitudeGuidanceMsg {
            omega_br_b_radps: Vector3::new(0.1, 0.2, 0.3),
            ..Default::default()
        });
        let mut module = RateServoFullNonlinear::new(RateServoFullNonlinearConfig {
            ki: 1.0,
            integral_limit: 10.0,
            ..Default::default()
        });
        guidance_out.connect_to(&mut module.guid_in_msg);
        module.init();
        module.update(&context(TASK_NANOS));
        module.update(&context(2 * TASK_NANOS));
        assert_eq!(module.z, Vector3::new(0.1, 0.2, 0.3) * 0.5);

        module.config.ki = 0.0;
        module.update(&context(3 * TASK_NANOS));
        assert_eq!(module.z, Vector3::zeros());
    }

    /// Not upstream: as upstream, the inertia and the RW configuration are
    /// read at reset; later changes of those messages apply after the next
    /// reset.
    #[test]
    fn reads_the_inertia_and_rw_configuration_only_at_reset() {
        let guidance_out = Output::new(guidance());
        let vehicle_out = Output::new(vehicle_config());
        let speeds_out = Output::new(wheel_speeds());
        let rw_out = Output::new(rw_config(UPSTREAM_SPIN_AXES, 4));
        let rate_out = Output::new(RateCommandMsg::default());
        let mut module = RateServoFullNonlinear::new(RateServoFullNonlinearConfig {
            p: 150.0,
            ..Default::default()
        });
        guidance_out.connect_to(&mut module.guid_in_msg);
        vehicle_out.connect_to(&mut module.veh_config_in_msg);
        speeds_out.connect_to(&mut module.rw_speeds_in_msg);
        rw_out.connect_to(&mut module.rw_params_in_msg);
        rate_out.connect_to(&mut module.rate_steering_in_msg);
        module.init();
        module.update(&context(0));
        let before = module.cmd_torque_out_msg.read().torque_request_body_nm;

        vehicle_out.write(VehicleConfigMsg {
            inertia_about_point_b_kg_m2: Matrix3::identity() * 5.0,
            ..Default::default()
        });
        rw_out.write(rw_config(UPSTREAM_SPIN_AXES, 0));
        module.update(&context(TASK_NANOS));
        assert_eq!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            before
        );
        assert_eq!(module.rw_config_params().num_rw, 4);

        module.reset(&context(TASK_NANOS));
        assert_eq!(module.rw_config_params().num_rw, 0);
        module.update(&context(2 * TASK_NANOS));
        assert_ne!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            before
        );
    }

    /// Not upstream: without the RW configuration message the wheel speeds
    /// and availability are ignored, and the torque is that of a rigid hub.
    #[test]
    fn ignores_the_wheels_without_an_rw_configuration() {
        let config = RateServoFullNonlinearConfig {
            p: 150.0,
            ..Default::default()
        };
        let torque_with = |connect_wheels: bool| {
            let guidance_out = Output::new(guidance());
            let vehicle_out = Output::new(vehicle_config());
            let speeds_out = Output::new(wheel_speeds());
            let rate_out = Output::new(RateCommandMsg::default());
            let mut module = RateServoFullNonlinear::new(config.clone());
            guidance_out.connect_to(&mut module.guid_in_msg);
            vehicle_out.connect_to(&mut module.veh_config_in_msg);
            rate_out.connect_to(&mut module.rate_steering_in_msg);
            if connect_wheels {
                speeds_out.connect_to(&mut module.rw_speeds_in_msg);
            }
            module.init();
            module.update(&context(0));
            module.cmd_torque_out_msg.read().torque_request_body_nm
        };
        let rigid = upstream_true_torques(
            &config,
            &guidance(),
            &RwArrayConfigMsg::default(),
            &RwAvailabilityMsg::default(),
            &RateCommandMsg::default(),
        )[0];
        assert_array_equal(torque_with(true), rigid, 1e-12, "with wheel speeds");
        assert_array_equal(torque_with(false), rigid, 1e-12, "without wheel speeds");
    }

    #[test]
    fn init_publishes_a_zero_torque() {
        let mut module = RateServoFullNonlinear::new(RateServoFullNonlinearConfig::default());
        module
            .cmd_torque_out_msg
            .write(crate::messages::BodyTorqueCommandMsg {
                torque_request_body_nm: Vector3::new(1.0, 2.0, 3.0),
            });
        module.init();
        assert_eq!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            Vector3::zeros()
        );
    }
}
