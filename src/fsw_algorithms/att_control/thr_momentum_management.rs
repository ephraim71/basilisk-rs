//! Reaction-wheel momentum to dump with thrusters: port of upstream Basilisk
//! `thrMomentumManagement`.
//!
//! On the first update after each reset the module computes the net
//! reaction-wheel momentum in body components,
//!
//! `h_s = Σ J_s,i Ω_i ĝ_s,i`,
//!
//! and publishes the momentum change that brings its magnitude down to the
//! configured minimum `h_s,min`:
//!
//! `Δh = -(|h_s| - h_s,min) / |h_s| h_s`, or zero when `|h_s| < h_s,min`.
//!
//! Later updates publish nothing, and reset does not clear the output, so it
//! keeps that request until the first update after the next reset; a thruster
//! momentum-dumping module turns it into thruster firings. The request
//! travels in a body torque command message, as upstream. Upstream's
//! `thrMomentumDumping` notices the one-time write through the message's
//! `timeWritten` header, which the Rust message bus does not carry.

use nalgebra::Vector3;

use crate::messages::{
    BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, RwArrayConfigMsg, RwSpeedMsg,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ThrMomentumManagement`].
#[derive(Clone, Debug)]
pub struct ThrMomentumManagementConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `hs_min` \[N m s\]: wheel momentum magnitude to keep after
    /// dumping; no dump is requested below it. Default 0.
    pub hs_min_nms: f64,
}

impl Default for ThrMomentumManagementConfig {
    fn default() -> Self {
        Self {
            name: "thrMomentumManagement".to_string(),
            hs_min_nms: 0.0,
        }
    }
}

/// One-shot momentum dumping request: upstream `thrMomentumManagement`.
#[derive(Clone, Debug)]
pub struct ThrMomentumManagement {
    /// User-settable parameters.
    pub config: ThrMomentumManagementConfig,
    /// Upstream `rwSpeedsInMsg`: wheel speeds \[rad/s\].
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Upstream `rwConfigDataInMsg`: wheel spin axes and inertias, read at
    /// reset.
    pub rw_config_data_in_msg: Input<RwArrayConfigMsg>,
    /// Upstream `deltaHOutMsg`: requested momentum change \[N m s\], body
    /// components, in the `torque_request_body_nm` field.
    pub delta_h_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `rwConfigParams`: the wheel layout cached at reset.
    rw_config: RwArrayConfigMsg,
    /// Upstream `initRequest`: the momentum check is still due since the last
    /// reset.
    init_request: bool,
}

impl ThrMomentumManagement {
    /// Builds the module with `config` and unconnected ports.
    pub fn new(config: ThrMomentumManagementConfig) -> Self {
        Self {
            config,
            rw_speeds_in_msg: Input::default(),
            rw_config_data_in_msg: Input::default(),
            delta_h_out_msg: Output::default(),
            rw_config: RwArrayConfigMsg::default(),
            init_request: false,
        }
    }

    /// Upstream `Reset_thrMomentumManagement`: reports missing inputs, caches
    /// the wheel layout and re-arms the momentum check.
    fn reset_state(&mut self) {
        if !self.rw_config_data_in_msg.is_connected() {
            log::error!("thrMomentumManagement.rw_config_data_in_msg is not connected");
        }
        if !self.rw_speeds_in_msg.is_connected() {
            log::error!("thrMomentumManagement.rw_speeds_in_msg is not connected");
        }
        self.rw_config = self.rw_config_data_in_msg.read();
        if self.rw_config.num_rw > MAX_EFF_COUNT {
            log::error!(
                "thrMomentumManagement: num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
            self.rw_config.num_rw = MAX_EFF_COUNT;
        }
        self.init_request = true;
    }

    /// Momentum change \[N m s\] that leaves `hs_min_nms` of wheel momentum.
    fn momentum_change(&self, wheel_speeds_radps: &[f64]) -> Vector3<f64> {
        let config = &self.rw_config;
        let wheel_momentum = config.spin_axes_body[..config.num_rw]
            .iter()
            .zip(&config.spin_axis_inertias_kg_m2)
            .zip(wheel_speeds_radps)
            .fold(Vector3::zeros(), |sum, ((axis, inertia), speed)| {
                sum + axis * (inertia * speed)
            });
        let magnitude = wheel_momentum.norm();
        let hs_min = self.config.hs_min_nms;
        if magnitude < hs_min {
            Vector3::zeros()
        } else {
            // Upstream quirk kept: zero momentum with hs_min <= 0 divides by
            // zero and requests NaN.
            wheel_momentum * (-(magnitude - hs_min) / magnitude)
        }
    }
}

impl Module for ThrMomentumManagement {
    fn init(&mut self) {
        self.delta_h_out_msg.write(BodyTorqueCommandMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        if !self.init_request {
            return;
        }
        let wheel_speeds_radps = self.rw_speeds_in_msg.read().wheel_speeds_radps;
        let delta_h_nms = self.momentum_change(&wheel_speeds_radps);
        self.init_request = false;
        self.delta_h_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: delta_h_nms,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports `test_thrMomentumManagement.py` except its
    //! `simHelpers.writeTeXSnippet` calls, which only write LaTeX report
    //! snippets and check nothing.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{BodyTorqueCommandMsg, Output, RwArrayConfigMsg, RwSpeedMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    use super::{ThrMomentumManagement, ThrMomentumManagementConfig};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_RATE_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, truth: Vector3<f64>, accuracy: f64) {
        assert!(
            (actual - truth).norm() <= accuracy,
            "{actual:?} differs from {truth:?}"
        );
    }

    fn speeds_msg(leading: &[f64]) -> RwSpeedMsg {
        let mut message = RwSpeedMsg::default();
        message.wheel_speeds_radps[..leading.len()].copy_from_slice(leading);
        message
    }

    /// Upstream `fswSetupRW.create(gsHat_B, Js)` for each wheel followed by
    /// `writeConfigMessage()`: each axis is divided by its numpy norm and
    /// `uMax` defaults to NaN.
    fn fsw_setup_rw(wheels: &[([f64; 3], f64)]) -> RwArrayConfigMsg {
        let mut config = RwArrayConfigMsg {
            num_rw: wheels.len(),
            ..Default::default()
        };
        for (index, (axis, inertia)) in wheels.iter().enumerate() {
            let axis = Vector3::from(*axis);
            config.spin_axes_body[index] = axis / axis.norm();
            config.spin_axis_inertias_kg_m2[index] = *inertia;
            config.max_motor_torques_nm[index] = f64::NAN;
        }
        config
    }

    /// The wheels of `test_thrMomentumManagement.py`.
    fn upstream_wheels() -> RwArrayConfigMsg {
        let js = 0.1;
        fsw_setup_rw(&[
            ([1.0, 0.0, 0.0], js),
            ([0.0, 1.0, 0.0], js),
            ([0.0, 0.0, 1.0], js),
            (
                [
                    0.577_350_269_189_625_8,
                    0.577_350_269_189_625_8,
                    0.577_350_269_189_625_8,
                ],
                js,
            ),
        ])
    }

    /// `test_thrMomentumManagement.py::test_thrMomentumManagement`, both
    /// `hsMinCheck` parametrizations, with upstream's hard-coded truth and
    /// `accuracy = 1e-12` per component. Upstream runs a 0.5 s task to
    /// t = 0.5 s and checks both recorded samples: the request is computed
    /// at t = 0 and the message still holds it at t = 0.5 s.
    #[test]
    fn matches_upstream_thr_momentum_management_unit_test() {
        let accuracy = 1e-12;
        let cases = [
            (
                0,
                100.0 / 6000.0 * 100.0,
                [
                    -5.914_369_484_146_579,
                    -2.858_300_248_464_629,
                    -9.407_020_039_211_664,
                ],
            ),
            (1, 1000.0 / 6000.0 * 100.0, [0.0, 0.0, 0.0]),
        ];
        for (hs_min_check, hs_min_nms, truth) in cases {
            let speeds = Output::new(speeds_msg(&[10.0, -25.0, 50.0, 100.0]));
            let wheels = Output::new(upstream_wheels());
            let mut module = ThrMomentumManagement::new(ThrMomentumManagementConfig {
                hs_min_nms,
                ..Default::default()
            });
            let mut recorder = MessageRecorder::<BodyTorqueCommandMsg>::default();

            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&speeds, &mut module.rw_speeds_in_msg);
            simulation.connect(&wheels, &mut module.rw_config_data_in_msg);
            simulation.connect(&module.delta_h_out_msg, &mut recorder.input_msg);
            simulation.add_module("thrMomentumManagement", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(TASK_RATE_NANOS);
            drop(simulation);

            assert_eq!(recorder.samples.len(), 2, "hsMinCheck = {hs_min_check}");
            for (sample, message) in recorder.samples.iter().enumerate() {
                for (axis, (value, expected)) in
                    message.torque_request_body_nm.iter().zip(truth).enumerate()
                {
                    assert!(
                        (value - expected).abs() <= accuracy,
                        "hsMinCheck = {hs_min_check}, sample {sample}, axis {axis}: \
                         {value:.17e} != {expected:.17e}"
                    );
                }
            }
        }
    }

    // Not upstream: the check runs once per reset. Later wheel speeds are
    // ignored until a reset re-arms it, and the reset re-reads the wheels.
    // Hand-derived: one wheel along x with J_s = 1 kg m^2 at 3 rad/s and
    // hs_min = 1 N m s requests Δh = -(3 - 1)/3 * 3 x̂ = -2 x̂.
    #[test]
    fn computes_the_request_once_per_reset() {
        let mut config = RwArrayConfigMsg {
            num_rw: 1,
            ..Default::default()
        };
        config.spin_axes_body[0] = Vector3::x();
        config.spin_axis_inertias_kg_m2[0] = 1.0;
        let speeds = Output::new(speeds_msg(&[3.0]));
        let wheels = Output::new(config.clone());
        let mut module = ThrMomentumManagement::new(ThrMomentumManagementConfig {
            hs_min_nms: 1.0,
            ..Default::default()
        });
        speeds.connect_to(&mut module.rw_speeds_in_msg);
        wheels.connect_to(&mut module.rw_config_data_in_msg);
        module.init();

        module.update(&context(0));
        assert_vector_close(
            module.delta_h_out_msg.read().torque_request_body_nm,
            Vector3::new(-2.0, 0.0, 0.0),
            1e-15,
        );

        speeds.write(speeds_msg(&[5.0]));
        config.spin_axes_body[0] = Vector3::y();
        wheels.write(config);
        module.update(&context(TASK_RATE_NANOS));
        assert_vector_close(
            module.delta_h_out_msg.read().torque_request_body_nm,
            Vector3::new(-2.0, 0.0, 0.0),
            1e-15,
        );

        // Reset re-arms the check but does not touch the output.
        Module::reset(&mut module, &context(2 * TASK_RATE_NANOS));
        assert_vector_close(
            module.delta_h_out_msg.read().torque_request_body_nm,
            Vector3::new(-2.0, 0.0, 0.0),
            1e-15,
        );
        module.update(&context(2 * TASK_RATE_NANOS));
        assert_vector_close(
            module.delta_h_out_msg.read().torque_request_body_nm,
            Vector3::new(0.0, -4.0, 0.0),
            1e-15,
        );
    }

    // Not upstream: hs_min is read at the update that runs the check, so a
    // value set after init applies; momentum exactly at hs_min requests a
    // zero change.
    #[test]
    fn minimum_momentum_set_after_init_applies() {
        let mut config = RwArrayConfigMsg {
            num_rw: 1,
            ..Default::default()
        };
        config.spin_axes_body[0] = Vector3::z();
        config.spin_axis_inertias_kg_m2[0] = 0.5;
        let speeds = Output::new(speeds_msg(&[4.0]));
        let wheels = Output::new(config);
        let mut module = ThrMomentumManagement::new(ThrMomentumManagementConfig::default());
        speeds.connect_to(&mut module.rw_speeds_in_msg);
        wheels.connect_to(&mut module.rw_config_data_in_msg);
        module.init();
        module.config.hs_min_nms = 2.0;
        module.update(&context(0));

        assert_eq!(
            module.delta_h_out_msg.read().torque_request_body_nm,
            Vector3::zeros()
        );
    }

    // Not upstream: upstream divides by |h_s| without a guard, so zero wheel
    // momentum with the default hs_min = 0 requests NaN.
    #[test]
    fn zero_momentum_with_zero_minimum_requests_nan_as_upstream() {
        let mut module = ThrMomentumManagement::new(ThrMomentumManagementConfig::default());
        module.init();
        module.update(&context(0));

        assert!(
            module
                .delta_h_out_msg
                .read()
                .torque_request_body_nm
                .iter()
                .all(|value| value.is_nan())
        );
    }
}
