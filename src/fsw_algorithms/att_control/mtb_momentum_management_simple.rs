//! Proportional reaction-wheel momentum dumping with torque rods: port of
//! upstream Basilisk `mtbMomentumManagementSimple`.
//!
//! The net reaction-wheel momentum in body components,
//!
//! `h_wheels = Σ ĝ_s,i J_s,i Ω_i`,
//!
//! is driven to zero with the proportional law
//!
//! `τ_desired = -K_p h_wheels`,
//!
//! published as the body torque the torque rods should produce (to be turned
//! into a dipole by `torque2Dipole` and `dipoleMapping`). The wheel layout
//! comes from the RW array configuration message, which is read at reset; the
//! gain is read at every update.

use nalgebra::Vector3;

use crate::messages::{
    BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, RwArrayConfigMsg, RwSpeedMsg,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`MtbMomentumManagementSimple`].
#[derive(Clone, Debug)]
pub struct MtbMomentumManagementSimpleConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `Kp` \[1/s\]: momentum feedback gain. Default 0. A negative
    /// gain is logged as an error at reset (where upstream's `_bskError`
    /// raises) and then applied.
    pub kp_per_s: f64,
}

impl Default for MtbMomentumManagementSimpleConfig {
    fn default() -> Self {
        Self {
            name: "mtbMomentumManagementSimple".to_string(),
            kp_per_s: 0.0,
        }
    }
}

/// Torque-rod torque request that dumps the wheel momentum: upstream
/// `mtbMomentumManagementSimple`.
#[derive(Clone, Debug)]
pub struct MtbMomentumManagementSimple {
    /// User-settable parameters.
    pub config: MtbMomentumManagementSimpleConfig,
    /// Upstream `rwParamsInMsg`: wheel spin axes and inertias, read at reset.
    pub rw_params_in_msg: Input<RwArrayConfigMsg>,
    /// Upstream `rwSpeedsInMsg`: wheel speeds \[rad/s\].
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Upstream `tauMtbRequestOutMsg`: torque \[N m\] requested from the torque
    /// rods, body components.
    pub tau_mtb_request_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `rwConfigParams`: the wheel layout cached at reset. Its spin
    /// axes are the columns of upstream's `Gs` (the transposed `GsMatrix_B`).
    rw_config: RwArrayConfigMsg,
}

impl MtbMomentumManagementSimple {
    /// Builds the module with `config` and unconnected ports.
    pub fn new(config: MtbMomentumManagementSimpleConfig) -> Self {
        Self {
            config,
            rw_params_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            tau_mtb_request_out_msg: Output::default(),
            rw_config: RwArrayConfigMsg::default(),
        }
    }

    /// Upstream `Reset_mtbMomentumManagementSimple`: reports missing inputs,
    /// caches the wheel layout and checks the gain.
    fn reset_state(&mut self) {
        if !self.rw_params_in_msg.is_connected() {
            log::error!("mtbMomentumManagementSimple.rw_params_in_msg is not connected");
        }
        if !self.rw_speeds_in_msg.is_connected() {
            log::error!("mtbMomentumManagementSimple.rw_speeds_in_msg is not connected");
        }
        self.rw_config = self.rw_params_in_msg.read();
        if self.rw_config.num_rw > MAX_EFF_COUNT {
            log::error!(
                "mtbMomentumManagementSimple: num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
            self.rw_config.num_rw = MAX_EFF_COUNT;
        }
        if self.config.kp_per_s < 0.0 {
            log::error!("mtbMomentumManagementSimple: kp_per_s must not be negative");
        }
    }

    /// Net wheel momentum `[Gs] (J_s Ω)` \[N m s\] in body components.
    fn wheel_momentum_body(&self, wheel_speeds_radps: &[f64]) -> Vector3<f64> {
        let config = &self.rw_config;
        // Accumulated wheel by wheel from zero, in upstream mMultV's order.
        config.spin_axes_body[..config.num_rw]
            .iter()
            .zip(wheel_speeds_radps)
            .zip(&config.spin_axis_inertias_kg_m2)
            .fold(Vector3::zeros(), |sum, ((axis, speed), inertia)| {
                sum + axis * (speed * inertia)
            })
    }
}

impl Module for MtbMomentumManagementSimple {
    fn init(&mut self) {
        self.tau_mtb_request_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let wheel_speeds_radps = self.rw_speeds_in_msg.read().wheel_speeds_radps;
        let momentum_nms = self.wheel_momentum_body(&wheel_speeds_radps);
        self.tau_mtb_request_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: -self.config.kp_per_s * momentum_nms,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3x4, Vector3, Vector4};

    use crate::messages::{MAX_EFF_COUNT, Output, RwArrayConfigMsg, RwSpeedMsg};
    use crate::{Module, SimulationContext};

    use super::{MtbMomentumManagementSimple, MtbMomentumManagementSimpleConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `unitTestSupport.isVectorEqual`: the norm of the difference is
    /// within `accuracy` and the result holds no NaN.
    fn assert_vector_equal(actual: Vector3<f64>, truth: Vector3<f64>, accuracy: f64, what: &str) {
        assert!(
            !actual.iter().any(|value| value.is_nan()),
            "{what}: NaN in {actual}"
        );
        let error = (actual - truth).norm();
        assert!(
            error <= accuracy,
            "{what}: {actual:?} differs from {truth:?} by {error:e}"
        );
    }

    fn speeds_msg(leading: &[f64]) -> RwSpeedMsg {
        let mut message = RwSpeedMsg::default();
        message.wheel_speeds_radps[..leading.len()].copy_from_slice(leading);
        message
    }

    /// The four-wheel pyramid of `test_mtbMomentumManagementSimple.py`:
    /// `GsMatrix_B` lists one spin axis per wheel, `JsList = [0.002] * 4`.
    fn upstream_wheels() -> RwArrayConfigMsg {
        let beta = 45.0 * std::f64::consts::PI / 180.0;
        let (sine, cosine) = beta.sin_cos();
        let mut config = RwArrayConfigMsg {
            num_rw: 4,
            ..Default::default()
        };
        config.spin_axes_body[..4].copy_from_slice(&[
            Vector3::new(0.0, cosine, sine),
            Vector3::new(0.0, sine, -cosine),
            Vector3::new(cosine, -sine, 0.0),
            Vector3::new(-cosine, -sine, 0.0),
        ]);
        config.spin_axis_inertias_kg_m2[..4].fill(0.002);
        config
    }

    /// The truth `test_mtbMomentumManagementSimple.py` computes with numpy:
    /// `-Kp * Gs @ (wheelSpeeds * JsList[0])`, with its hand-written `Gs`.
    fn upstream_truth(kp: f64, wheel_speeds: Vector4<f64>) -> Vector3<f64> {
        let beta = 45.0 * std::f64::consts::PI / 180.0;
        let (sine, cosine) = beta.sin_cos();
        let gs = Matrix3x4::new(
            0.0, 0.0, cosine, -cosine, //
            cosine, sine, -sine, -sine, //
            sine, -cosine, 0.0, 0.0,
        );
        -kp * (gs * (wheel_speeds * 0.002))
    }

    /// `test_mtbMomentumManagementSimple.py`, all three sub-tests and the
    /// closing `Reset`, `accuracy = 1e-8`. Each sub-test re-initializes the
    /// simulation and runs the single t = 0 tick; sub-test 3 sets `Kp = 0`
    /// on the module before re-initializing.
    #[test]
    fn matches_upstream_mtb_momentum_management_simple_unit_test() {
        let accuracy = 1e-8;
        let wheel_speeds = Vector4::new(100.0, 200.0, 300.0, 400.0);
        let rw_params = Output::new(upstream_wheels());
        let rw_speeds = Output::new(speeds_msg(wheel_speeds.as_slice()));
        let mut module = MtbMomentumManagementSimple::new(MtbMomentumManagementSimpleConfig {
            kp_per_s: 0.003,
            ..Default::default()
        });
        rw_params.connect_to(&mut module.rw_params_in_msg);
        rw_speeds.connect_to(&mut module.rw_speeds_in_msg);

        // TEST 1: a non-zero torque for non-zero wheel speeds.
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            upstream_truth(0.003, wheel_speeds),
            accuracy,
            "tauMtbRequestOutMsg",
        );

        // TEST 2: zero torque for zero wheel speeds.
        rw_speeds.write(speeds_msg(&[0.0; 4]));
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::zeros(),
            accuracy,
            "tauMtbRequestOutMsg",
        );

        // TEST 3: zero torque for a zero gain.
        rw_speeds.write(speeds_msg(wheel_speeds.as_slice()));
        module.config.kp_per_s = 0.0;
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::zeros(),
            accuracy,
            "tauMtbRequestOutMsg",
        );

        // Upstream closes with module.Reset(0) and checks nothing after it;
        // the Reset leaves the last (zero) request in place.
        Module::reset(&mut module, &context(0));
        assert_vector_equal(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::zeros(),
            accuracy,
            "tauMtbRequestOutMsg",
        );
    }

    // Not upstream: hand-derived momentum. Wheels along x and z with
    // J_s = 0.1 and 0.2 kg m^2 at 10 and -5 rad/s give h = (1, 0, -1) N m s,
    // so Kp = 0.01 1/s requests τ = (-0.01, 0, 0.01) N m.
    #[test]
    fn requests_minus_kp_times_the_wheel_momentum() {
        let mut config = RwArrayConfigMsg {
            num_rw: 2,
            ..Default::default()
        };
        config.spin_axes_body[..2].copy_from_slice(&[Vector3::x(), Vector3::z()]);
        config.spin_axis_inertias_kg_m2[..2].copy_from_slice(&[0.1, 0.2]);
        let rw_params = Output::new(config);
        let rw_speeds = Output::new(speeds_msg(&[10.0, -5.0, 1e3]));
        let mut module = MtbMomentumManagementSimple::new(MtbMomentumManagementSimpleConfig {
            kp_per_s: 0.01,
            ..Default::default()
        });
        rw_params.connect_to(&mut module.rw_params_in_msg);
        rw_speeds.connect_to(&mut module.rw_speeds_in_msg);
        module.init();
        module.update(&context(0));

        assert_vector_equal(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::new(-0.01, 0.0, 0.01),
            1e-17,
            "tauMtbRequestOutMsg",
        );
    }

    // Not upstream: upstream reads Kp at every update but the wheel layout
    // only at reset. A changed gain applies at the next update; a changed
    // layout waits for the next reset, which itself leaves the output alone.
    #[test]
    fn gain_is_read_at_update_and_wheel_layout_at_reset() {
        let mut config = RwArrayConfigMsg {
            num_rw: 1,
            ..Default::default()
        };
        config.spin_axes_body[0] = Vector3::y();
        config.spin_axis_inertias_kg_m2[0] = 1.0;
        let rw_params = Output::new(config.clone());
        let rw_speeds = Output::new(speeds_msg(&[2.0]));
        let mut module = MtbMomentumManagementSimple::new(Default::default());
        rw_params.connect_to(&mut module.rw_params_in_msg);
        rw_speeds.connect_to(&mut module.rw_speeds_in_msg);
        module.init();

        module.config.kp_per_s = 0.5;
        config.spin_axes_body[0] = Vector3::x();
        rw_params.write(config);
        module.update(&context(0));
        assert_eq!(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::new(0.0, -1.0, 0.0)
        );

        Module::reset(&mut module, &context(10_000_000));
        assert_eq!(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::new(0.0, -1.0, 0.0)
        );
        module.update(&context(10_000_000));
        assert_eq!(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::new(-1.0, 0.0, 0.0)
        );
    }

    // Not upstream: a negative gain is logged as an error at reset (upstream's
    // _bskError raises there instead) and then applied.
    #[test]
    fn negative_gain_is_logged_and_still_applied() {
        let mut config = RwArrayConfigMsg {
            num_rw: 1,
            ..Default::default()
        };
        config.spin_axes_body[0] = Vector3::z();
        config.spin_axis_inertias_kg_m2[0] = 1.0;
        let rw_params = Output::new(config);
        let rw_speeds = Output::new(speeds_msg(&[4.0]));
        let mut module = MtbMomentumManagementSimple::new(MtbMomentumManagementSimpleConfig {
            kp_per_s: -0.25,
            ..Default::default()
        });
        rw_params.connect_to(&mut module.rw_params_in_msg);
        rw_speeds.connect_to(&mut module.rw_speeds_in_msg);
        module.init();
        module.update(&context(0));

        assert_eq!(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::new(0.0, 0.0, 1.0)
        );
    }

    // Not upstream: a wheel count beyond the message capacity is truncated to
    // MAX_EFF_COUNT (upstream would read past its arrays).
    #[test]
    fn wheel_count_is_truncated_to_the_message_capacity() {
        let mut config = RwArrayConfigMsg {
            num_rw: MAX_EFF_COUNT + 2,
            ..Default::default()
        };
        config.spin_axes_body.fill(Vector3::x());
        config.spin_axis_inertias_kg_m2.fill(1.0);
        let rw_params = Output::new(config);
        let rw_speeds = Output::new(speeds_msg(&[1.0; MAX_EFF_COUNT]));
        let mut module = MtbMomentumManagementSimple::new(MtbMomentumManagementSimpleConfig {
            kp_per_s: 1.0,
            ..Default::default()
        });
        rw_params.connect_to(&mut module.rw_params_in_msg);
        rw_speeds.connect_to(&mut module.rw_speeds_in_msg);
        module.init();
        module.update(&context(0));

        assert_eq!(
            module.tau_mtb_request_out_msg.read().torque_request_body_nm,
            Vector3::new(-(MAX_EFF_COUNT as f64), 0.0, 0.0)
        );
    }
}
