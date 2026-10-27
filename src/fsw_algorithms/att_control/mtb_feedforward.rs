//! Torque-rod feedforward on the commanded body torque: port of upstream
//! Basilisk `mtbFeedforward`.
//!
//! The torque rods' dipole commands `μ` produce the body torque
//!
//! `τ_rods = ([Gt] μ) × b`
//!
//! in the measured magnetic field `b`, where the columns of `[Gt]` are the rod
//! axes in body components. The module subtracts that torque from the
//! commanded body torque, so the reaction wheels are driven to absorb the
//! momentum the rods dump:
//!
//! `τ_out = τ_in - τ_rods`.
//!
//! The rod layout comes from the MTB array configuration message, which is
//! read at reset.

use nalgebra::Vector3;

use crate::messages::{
    BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, MtbArrayCommandMsg, MtbArrayConfigMsg, Output,
    TamSensorBodyMsg,
};
use crate::{Module, SimulationContext};

/// Configuration of [`MtbFeedforward`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct MtbFeedforwardConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
}

impl Default for MtbFeedforwardConfig {
    fn default() -> Self {
        Self {
            name: "mtbFeedforward".to_string(),
        }
    }
}

/// Torque-rod torque feedforward: upstream `mtbFeedforward`.
#[derive(Clone, Debug)]
pub struct MtbFeedforward {
    /// Module configuration.
    pub config: MtbFeedforwardConfig,
    /// Upstream `vehControlInMsg`: commanded body torque \[N m\].
    pub veh_control_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `dipoleRequestMtbInMsg`: individual rod dipole commands
    /// \[A m^2\].
    pub dipole_request_mtb_in_msg: Input<MtbArrayCommandMsg>,
    /// Upstream `tamSensorBodyInMsg`: measured magnetic field \[T\], body
    /// components.
    pub tam_sensor_body_in_msg: Input<TamSensorBodyMsg>,
    /// Upstream `mtbArrayConfigParamsInMsg`: rod count and alignment `[Gt]`,
    /// read at reset.
    pub mtb_array_config_params_in_msg: Input<MtbArrayConfigMsg>,
    /// Upstream `vehControlOutMsg`: commanded body torque less the rod torque
    /// \[N m\].
    pub veh_control_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `mtbArrayConfigParams`: the rod layout cached at reset.
    mtb_array_config: MtbArrayConfigMsg,
}

impl MtbFeedforward {
    /// Builds the module with unconnected ports.
    pub fn new(config: MtbFeedforwardConfig) -> Self {
        Self {
            config,
            veh_control_in_msg: Input::default(),
            dipole_request_mtb_in_msg: Input::default(),
            tam_sensor_body_in_msg: Input::default(),
            mtb_array_config_params_in_msg: Input::default(),
            veh_control_out_msg: Output::default(),
            mtb_array_config: MtbArrayConfigMsg::default(),
        }
    }

    /// Upstream `Reset_mtbFeedforward`: reports missing inputs and caches the
    /// rod layout.
    fn reset_state(&mut self) {
        if !self.dipole_request_mtb_in_msg.is_connected() {
            log::error!("mtbFeedforward.dipole_request_mtb_in_msg is not connected");
        }
        if !self.veh_control_in_msg.is_connected() {
            log::error!("mtbFeedforward.veh_control_in_msg is not connected");
        }
        if !self.tam_sensor_body_in_msg.is_connected() {
            log::error!("mtbFeedforward.tam_sensor_body_in_msg is not connected");
        }
        if !self.mtb_array_config_params_in_msg.is_connected() {
            log::error!("mtbFeedforward.mtb_array_config_params_in_msg is not connected");
        }
        self.mtb_array_config = self.mtb_array_config_params_in_msg.read();
        if self.mtb_array_config.num_mtb > MAX_EFF_COUNT {
            log::error!(
                "mtbFeedforward: num_mtb exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
            self.mtb_array_config.num_mtb = MAX_EFF_COUNT;
        }
    }

    /// Net rod dipole `[Gt] μ` \[A m^2\] in body components.
    fn net_rod_dipole(&self, dipole_cmds_am2: &[f64]) -> Vector3<f64> {
        let config = &self.mtb_array_config;
        // Accumulated rod by rod from zero, in upstream mMultV's order.
        config.dipole_axes_body[..config.num_mtb]
            .iter()
            .zip(dipole_cmds_am2)
            .fold(Vector3::zeros(), |sum, (axis, command)| {
                sum + axis * *command
            })
    }
}

impl Module for MtbFeedforward {
    fn init(&mut self) {
        self.veh_control_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let dipole_cmds_am2 = self.dipole_request_mtb_in_msg.read().dipole_cmds_am2;
        let field_t = self.tam_sensor_body_in_msg.read().magnetic_field_body_t;
        let commanded_nm = self.veh_control_in_msg.read().torque_request_body_nm;

        let rod_torque_nm = self.net_rod_dipole(&dipole_cmds_am2).cross(&field_t);
        self.veh_control_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm: commanded_nm - rod_torque_nm,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3x2, Vector2, Vector3};

    use crate::messages::{
        BodyTorqueCommandMsg, MAX_EFF_COUNT, MtbArrayCommandMsg, MtbArrayConfigMsg, Output,
        TamSensorBodyMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{MtbFeedforward, MtbFeedforwardConfig};

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

    /// The truth `test_mtbFeedforward.py` computes with numpy:
    /// `-np.cross(Gt @ m, b)`.
    fn upstream_truth(
        gt: &[Vector3<f64>],
        dipoles_am2: &[f64],
        field_t: Vector3<f64>,
    ) -> Vector3<f64> {
        let net_dipole: Vector3<f64> = gt
            .iter()
            .zip(dipoles_am2)
            .map(|(axis, dipole)| axis * *dipole)
            .sum();
        -net_dipole.cross(&field_t)
    }

    struct Inputs {
        torque: Output<BodyTorqueCommandMsg>,
        dipoles: Output<MtbArrayCommandMsg>,
        field: Output<TamSensorBodyMsg>,
        mtb_config: Output<MtbArrayConfigMsg>,
    }

    impl Inputs {
        fn connect(&self, module: &mut MtbFeedforward) {
            self.torque.connect_to(&mut module.veh_control_in_msg);
            self.dipoles
                .connect_to(&mut module.dipole_request_mtb_in_msg);
            self.field.connect_to(&mut module.tam_sensor_body_in_msg);
            self.mtb_config
                .connect_to(&mut module.mtb_array_config_params_in_msg);
        }
    }

    fn field_msg(magnetic_field_body_t: Vector3<f64>) -> TamSensorBodyMsg {
        TamSensorBodyMsg {
            magnetic_field_body_t,
        }
    }

    /// `test_mtbFeedforward.py` sets `maxMtbDipoles = [1E3, 1E3, 1E3]` once
    /// and rewrites `numMTB` and `GtMatrix_B`; its unset entries are zero.
    fn mtb_config(axes: &[Vector3<f64>]) -> MtbArrayConfigMsg {
        let mut config = MtbArrayConfigMsg {
            num_mtb: axes.len(),
            ..Default::default()
        };
        config.dipole_axes_body[..axes.len()].copy_from_slice(axes);
        config.max_dipoles_am2[..3].fill(1e3);
        config
    }

    /// `test_mtbFeedforward.py::test_mtbFeedforward_module`, all four
    /// sub-tests, `accuracy = 1e-12`. Each sub-test re-initializes the
    /// simulation, which re-reads the MTB configuration, and runs the single
    /// t = 0 tick.
    #[test]
    fn matches_upstream_mtb_feedforward_unit_test() {
        let accuracy = 1e-12;
        let field_t = Vector3::new(1e-5, -3e-5, 5e-5);
        let identity_gt = [Vector3::x(), Vector3::y(), Vector3::z()];
        let inputs = Inputs {
            torque: Output::new(BodyTorqueCommandMsg::default()),
            dipoles: Output::new(MtbArrayCommandMsg::from_active(&[1.0, 2.0, 3.0])),
            field: Output::new(field_msg(field_t)),
            mtb_config: Output::new(mtb_config(&identity_gt)),
        };
        let mut module = MtbFeedforward::new(MtbFeedforwardConfig::default());
        inputs.connect(&mut module);

        // TEST 1: non-zero rod torque.
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.veh_control_out_msg.read().torque_request_body_nm,
            upstream_truth(&identity_gt, &[1.0, 2.0, 3.0], field_t),
            accuracy,
            "torqueRequestBody",
        );

        // TEST 2: zero torque for a zero field.
        inputs.field.write(field_msg(Vector3::zeros()));
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::zeros(),
            accuracy,
            "torqueRequestBody",
        );

        // TEST 3: zero torque for zero dipoles.
        inputs.field.write(field_msg(field_t));
        inputs
            .dipoles
            .write(MtbArrayCommandMsg::from_active(&[0.0, 0.0, 0.0]));
        module.init();
        module.update(&context(0));
        assert_vector_equal(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::zeros(),
            accuracy,
            "torqueRequestBody",
        );

        // TEST 4: two rods 45 deg apart in the x-y plane. GtMatrix_B is
        // 3 x numMTB row-major upstream; the message holds its columns.
        let beta = 45.0 * std::f64::consts::PI / 180.0;
        let gt = Matrix3x2::new(beta.cos(), -beta.sin(), beta.sin(), beta.cos(), 0.0, 0.0);
        let dipoles = Vector2::new(7.0, -3.0);
        inputs
            .dipoles
            .write(MtbArrayCommandMsg::from_active(dipoles.as_slice()));
        inputs.mtb_config.write(mtb_config(&[
            gt.column(0).clone_owned(),
            gt.column(1).clone_owned(),
        ]));
        module.init();
        module.update(&context(0));
        let expected = -(gt * dipoles).cross(&field_t);
        assert_vector_equal(
            module.veh_control_out_msg.read().torque_request_body_nm,
            expected,
            accuracy,
            "torqueRequestBody",
        );
    }

    // Not upstream (it always commands zero torque): the rod torque is
    // subtracted from the commanded torque. Hand-derived: one rod along x
    // with 2 A m^2 in b = 3e-5 T along y gives τ_rods = 2 x̂ × 3e-5 ŷ =
    // 6e-5 ẑ N m.
    #[test]
    fn subtracts_the_rod_torque_from_the_commanded_torque() {
        let inputs = Inputs {
            torque: Output::new(BodyTorqueCommandMsg {
                torque_request_body_nm: Vector3::new(1e-4, -2e-4, 3e-4),
            }),
            dipoles: Output::new(MtbArrayCommandMsg::from_active(&[2.0])),
            field: Output::new(field_msg(Vector3::new(0.0, 3e-5, 0.0))),
            mtb_config: Output::new(mtb_config(&[Vector3::x()])),
        };
        let mut module = MtbFeedforward::new(MtbFeedforwardConfig::default());
        inputs.connect(&mut module);
        module.init();
        module.update(&context(0));

        assert_vector_equal(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::new(1e-4, -2e-4, 3e-4 - 6e-5),
            1e-18,
            "torqueRequestBody",
        );
    }

    // Not upstream: the rod layout is cached at reset, as upstream reads it
    // in Reset only, and dipole commands past num_mtb are ignored.
    #[test]
    fn rod_layout_is_read_at_reset() {
        let inputs = Inputs {
            torque: Output::new(BodyTorqueCommandMsg::default()),
            dipoles: Output::new(MtbArrayCommandMsg::from_active(&[1.0, 1.0])),
            field: Output::new(field_msg(Vector3::z())),
            mtb_config: Output::new(mtb_config(&[Vector3::x()])),
        };
        let mut module = MtbFeedforward::new(MtbFeedforwardConfig::default());
        inputs.connect(&mut module);
        module.init();

        inputs
            .mtb_config
            .write(mtb_config(&[Vector3::x(), Vector3::y()]));
        module.update(&context(0));
        // -(x̂ × ẑ) = ŷ
        assert_eq!(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::y()
        );

        // Reset re-reads the layout but leaves the output alone.
        Module::reset(&mut module, &context(10_000_000));
        assert_eq!(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::y()
        );
        module.update(&context(10_000_000));
        // -((x̂ + ŷ) × ẑ) = ŷ - x̂
        assert_eq!(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::new(-1.0, 1.0, 0.0)
        );
    }

    // Not upstream: a rod count beyond the message capacity is truncated to
    // MAX_EFF_COUNT (upstream would read past its arrays).
    #[test]
    fn rod_count_is_truncated_to_the_message_capacity() {
        let mut config = MtbArrayConfigMsg {
            num_mtb: MAX_EFF_COUNT + 1,
            ..Default::default()
        };
        config.dipole_axes_body.fill(Vector3::x());
        let inputs = Inputs {
            torque: Output::new(BodyTorqueCommandMsg::default()),
            dipoles: Output::new(MtbArrayCommandMsg::from_active(&[1.0; MAX_EFF_COUNT])),
            field: Output::new(field_msg(Vector3::z())),
            mtb_config: Output::new(config),
        };
        let mut module = MtbFeedforward::new(MtbFeedforwardConfig::default());
        inputs.connect(&mut module);
        module.init();
        module.update(&context(0));

        assert_eq!(
            module.veh_control_out_msg.read().torque_request_body_nm,
            Vector3::new(0.0, MAX_EFF_COUNT as f64, 0.0)
        );
    }
}
