//! Torque-rod dipole allocation: port of upstream Basilisk `dipoleMapping`.
//!
//! Maps the requested body-frame dipole `μ_B` to individual torque-rod dipole
//! commands with a user-supplied steering matrix, normally the pseudo-inverse
//! of the rod alignment matrix `[Gt]`,
//!
//! `μ_cmd = [Gt]^† μ_B`,
//!
//! and saturates each command at the rod's `±max` dipole. The rod count and
//! dipole limits come from the MTB array configuration message, which is read
//! at reset. Reset also zeroes the output so that a disabled module leaves the
//! rods idle.

use nalgebra::Vector3;

use crate::messages::{
    DipoleRequestBodyMsg, Input, MAX_EFF_COUNT, MtbArrayCommandMsg, MtbArrayConfigMsg, Output,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`DipoleMapping`].
#[derive(Clone, Debug)]
pub struct DipoleMappingConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `steeringMatrix`: one entry per torque rod, entry `j` being
    /// row `j` of the `numMTB`-by-3 matrix that maps the body dipole request
    /// to rod `j`'s dipole (upstream stores it row-major, so its row `j` is
    /// elements `3j..3j+3`). Rods without an entry get a zero row, as upstream's
    /// zero-filled buffer does.
    pub steering_matrix: Vec<Vector3<f64>>,
}

impl Default for DipoleMappingConfig {
    fn default() -> Self {
        Self {
            name: "dipoleMapping".to_string(),
            steering_matrix: Vec::new(),
        }
    }
}

/// Body dipole to torque-rod dipole mapping: upstream `dipoleMapping`.
#[derive(Clone, Debug)]
pub struct DipoleMapping {
    /// User-settable parameters.
    pub config: DipoleMappingConfig,
    /// Upstream `mtbArrayConfigParamsInMsg`: rod count and dipole limits, read
    /// at reset.
    pub mtb_array_config_params_in_msg: Input<MtbArrayConfigMsg>,
    /// Upstream `dipoleRequestBodyInMsg`: requested dipole \[A m^2\], body
    /// components.
    pub dipole_request_body_in_msg: Input<DipoleRequestBodyMsg>,
    /// Upstream `dipoleRequestMtbOutMsg`: saturated rod dipole commands
    /// \[A m^2\].
    pub dipole_request_mtb_out_msg: Output<MtbArrayCommandMsg>,
    /// Upstream `mtbArrayConfigParams`: the rod configuration cached at reset.
    mtb_array_config: MtbArrayConfigMsg,
}

impl DipoleMapping {
    /// Builds the module with `config` and unconnected ports.
    pub fn new(config: DipoleMappingConfig) -> Self {
        Self {
            config,
            mtb_array_config_params_in_msg: Input::default(),
            dipole_request_body_in_msg: Input::default(),
            dipole_request_mtb_out_msg: Output::default(),
            mtb_array_config: MtbArrayConfigMsg::default(),
        }
    }

    /// Upstream `Reset_dipoleMapping`: reports missing inputs, caches the rod
    /// configuration and zeroes the rod commands.
    fn reset_state(&mut self) {
        if !self.dipole_request_body_in_msg.is_connected() {
            log::error!("dipoleMapping.dipole_request_body_in_msg is not connected");
        }
        if !self.mtb_array_config_params_in_msg.is_connected() {
            log::error!("dipoleMapping.mtb_array_config_params_in_msg is not connected");
        }
        self.mtb_array_config = self.mtb_array_config_params_in_msg.read();
        if self.mtb_array_config.num_mtb > MAX_EFF_COUNT {
            log::error!(
                "dipoleMapping: num_mtb exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
            self.mtb_array_config.num_mtb = MAX_EFF_COUNT;
        }
        self.dipole_request_mtb_out_msg
            .write(MtbArrayCommandMsg::default());
    }

    /// Saturated rod commands for the body dipole request; rods past
    /// `num_mtb` stay zero.
    fn rod_dipole_commands(&self, dipole_body_am2: Vector3<f64>) -> MtbArrayCommandMsg {
        let config = &self.mtb_array_config;
        let mut output = MtbArrayCommandMsg::default();
        for (rod, (command, &max_dipole)) in output
            .dipole_cmds_am2
            .iter_mut()
            .zip(&config.max_dipoles_am2)
            .take(config.num_mtb)
            .enumerate()
        {
            let steering_row = self
                .config
                .steering_matrix
                .get(rod)
                .copied()
                .unwrap_or_else(Vector3::zeros);
            *command = saturate(row_dot(steering_row, dipole_body_am2), max_dipole);
        }
        output
    }
}

/// One row of upstream's `mMultV`: summed from `+0.0` in index order, which
/// gives `+0.0` where `Vector3::dot` would give `-0.0` (all products `-0.0`).
fn row_dot(steering_row: Vector3<f64>, dipole_body_am2: Vector3<f64>) -> f64 {
    steering_row
        .iter()
        .zip(dipole_body_am2.iter())
        .fold(0.0, |sum, (steering, dipole)| sum + steering * dipole)
}

/// Upstream's two sequential limit checks, upper first. Unlike `f64::clamp`
/// this does not panic for a negative limit (it returns `-max`) and passes NaN
/// through.
fn saturate(command: f64, max_dipole: f64) -> f64 {
    let mut saturated = command;
    if saturated > max_dipole {
        saturated = max_dipole;
    }
    if saturated < -max_dipole {
        saturated = -max_dipole;
    }
    saturated
}

impl Module for DipoleMapping {
    fn init(&mut self) {
        self.dipole_request_mtb_out_msg
            .write(MtbArrayCommandMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let dipole_body_am2 = self.dipole_request_body_in_msg.read().dipole_b_am2;
        self.dipole_request_mtb_out_msg
            .write(self.rod_dipole_commands(dipole_body_am2));
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix2x3, Matrix3x2, Vector3};

    use crate::messages::{DipoleRequestBodyMsg, MAX_EFF_COUNT, MtbArrayConfigMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{DipoleMapping, DipoleMappingConfig, saturate};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `unitTestSupport.isVectorEqual` on the full `MAX_EFF_CNT`
    /// array: the norm of the difference is within `accuracy` and the result
    /// holds no NaN.
    fn assert_commands_equal(actual: &[f64], truth: &[f64], accuracy: f64, what: &str) {
        assert_eq!(actual.len(), truth.len(), "{what}: length");
        assert!(!actual.iter().any(|value| value.is_nan()), "{what}: NaN");
        let error = actual
            .iter()
            .zip(truth)
            .map(|(value, expected)| (value - expected).powi(2))
            .sum::<f64>()
            .sqrt();
        assert!(error <= accuracy, "{what}: off by {error:e}: {actual:?}");
    }

    /// `[0.] * MAX_EFF_CNT` with the leading entries replaced.
    fn padded(leading: &[f64]) -> [f64; MAX_EFF_COUNT] {
        let mut commands = [0.0; MAX_EFF_COUNT];
        commands[..leading.len()].copy_from_slice(leading);
        commands
    }

    fn dipole_msg(dipole_b_am2: Vector3<f64>) -> DipoleRequestBodyMsg {
        DipoleRequestBodyMsg { dipole_b_am2 }
    }

    /// `test_dipoleMapping.py::test_dipoleMapping_module`, all four
    /// sub-tests, `accuracy = 1e-12` (1e-3 for the reset check, as upstream).
    /// Each sub-test re-initializes the simulation, which re-reads the MTB
    /// configuration, and runs the single t = 0 tick; sub-test 3's second
    /// `ExecuteSimulation` to the same stop time runs no further tick.
    #[test]
    fn matches_upstream_dipole_mapping_unit_test() {
        let accuracy = 1e-12;
        let mut mtb_config = MtbArrayConfigMsg {
            num_mtb: 3,
            ..Default::default()
        };
        mtb_config.max_dipoles_am2[..3].fill(1e3);
        mtb_config.dipole_axes_body[..3].copy_from_slice(&[
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
        ]);
        let mtb_config_out = Output::new(mtb_config.clone());
        let dipole_out = Output::new(dipole_msg(Vector3::new(1.0, 2.0, 3.0)));

        let mut module = DipoleMapping::new(DipoleMappingConfig::default());
        module.config.steering_matrix = vec![Vector3::x(), Vector3::y(), Vector3::z()];
        mtb_config_out.connect_to(&mut module.mtb_array_config_params_in_msg);
        dipole_out.connect_to(&mut module.dipole_request_body_in_msg);

        // TEST 1: identity steering matrix.
        module.init();
        module.update(&context(0));
        assert_commands_equal(
            &module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            &padded(&[1.0, 2.0, 3.0]),
            accuracy,
            "dipoles",
        );

        // TEST 2: two rods 45 deg apart in the x-y plane, steered with
        // np.linalg.pinv(Gt). GtMatrix_B is 3 x numMTB row-major upstream;
        // the message holds its columns.
        let beta = 45.0 * std::f64::consts::PI / 180.0;
        let gt = Matrix3x2::new(beta.cos(), -beta.sin(), beta.sin(), beta.cos(), 0.0, 0.0);
        let gt_inverse: Matrix2x3<f64> = gt
            .pseudo_inverse(1e-15)
            .expect("the SVD pseudo-inverse of a fixed-size matrix always exists");
        mtb_config.num_mtb = 2;
        mtb_config.dipole_axes_body = [Vector3::zeros(); MAX_EFF_COUNT];
        mtb_config.dipole_axes_body[..2]
            .copy_from_slice(&[gt.column(0).into(), gt.column(1).into()]);
        mtb_config_out.write(mtb_config);
        module.config.steering_matrix = gt_inverse.row_iter().map(|row| row.transpose()).collect();

        module.init();
        module.update(&context(0));
        let expected = gt_inverse * Vector3::new(1.0, 2.0, 3.0);
        assert_commands_equal(
            &module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            &padded(expected.as_slice()),
            accuracy,
            "dipoles",
        );

        // TEST 3: zero dipole request.
        dipole_out.write(dipole_msg(Vector3::zeros()));
        module.init();
        module.update(&context(0));
        assert_commands_equal(
            &module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            &padded(&[]),
            accuracy,
            "dipoles",
        );

        // TEST 4: Reset() zeroes the output message.
        dipole_out.write(dipole_msg(Vector3::new(1.0, 2.0, 3.0)));
        module.init();
        module.update(&context(0));
        assert!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2[..2]
                .iter()
                .all(|command| command.abs() > 0.1)
        );
        Module::reset(&mut module, &context(0));
        assert_commands_equal(
            &module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            &padded(&[]),
            1e-3,
            "Reset() zeroed dipole",
        );
    }

    // Not upstream (its limits of 1e3 A m^2 are never reached): each command
    // is clamped to its own rod's limit, in either direction.
    #[test]
    fn saturates_each_rod_at_its_own_limit() {
        let mut mtb_config = MtbArrayConfigMsg {
            num_mtb: 3,
            ..Default::default()
        };
        mtb_config.max_dipoles_am2[..3].copy_from_slice(&[2.0, 3.0, 1.0]);
        let mtb_config_out = Output::new(mtb_config);
        let dipole_out = Output::new(dipole_msg(Vector3::new(5.0, -7.0, 0.5)));
        let mut module = DipoleMapping::new(DipoleMappingConfig {
            steering_matrix: vec![Vector3::x(), Vector3::y(), Vector3::z()],
            ..Default::default()
        });
        mtb_config_out.connect_to(&mut module.mtb_array_config_params_in_msg);
        dipole_out.connect_to(&mut module.dipole_request_body_in_msg);
        module.init();
        module.update(&context(0));

        assert_eq!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            padded(&[2.0, -3.0, 0.5])
        );
    }

    // Not upstream: the steering matrix is read at every update, so it can be
    // set or changed after construction; rods with no steering row get a zero
    // command, and rods past num_mtb stay zero whatever the steering matrix
    // holds.
    #[test]
    fn missing_steering_rows_and_inactive_rods_give_zero_commands() {
        let mut mtb_config = MtbArrayConfigMsg {
            num_mtb: 3,
            ..Default::default()
        };
        mtb_config.max_dipoles_am2.fill(10.0);
        let mtb_config_out = Output::new(mtb_config);
        let dipole_out = Output::new(dipole_msg(Vector3::new(1.0, 2.0, 3.0)));
        let mut module = DipoleMapping::new(DipoleMappingConfig::default());
        mtb_config_out.connect_to(&mut module.mtb_array_config_params_in_msg);
        dipole_out.connect_to(&mut module.dipole_request_body_in_msg);
        module.init();

        module.config.steering_matrix = vec![Vector3::new(1.0, 1.0, 0.0)];
        module.update(&context(0));
        assert_eq!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            padded(&[3.0])
        );

        module.config.steering_matrix = vec![Vector3::z(); 5];
        module.update(&context(10_000_000));
        assert_eq!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            padded(&[3.0, 3.0, 3.0])
        );
    }

    // Not upstream: the MTB configuration is cached at reset, as upstream
    // reads it in Reset only; a new configuration takes effect at the next
    // reset.
    #[test]
    fn mtb_configuration_is_read_at_reset() {
        let mut mtb_config = MtbArrayConfigMsg {
            num_mtb: 1,
            ..Default::default()
        };
        mtb_config.max_dipoles_am2[..2].fill(10.0);
        let mtb_config_out = Output::new(mtb_config.clone());
        let dipole_out = Output::new(dipole_msg(Vector3::new(1.0, 2.0, 3.0)));
        let mut module = DipoleMapping::new(DipoleMappingConfig {
            steering_matrix: vec![Vector3::x(), Vector3::y()],
            ..Default::default()
        });
        mtb_config_out.connect_to(&mut module.mtb_array_config_params_in_msg);
        dipole_out.connect_to(&mut module.dipole_request_body_in_msg);
        module.init();

        mtb_config.num_mtb = 2;
        mtb_config_out.write(mtb_config);
        module.update(&context(0));
        assert_eq!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            padded(&[1.0])
        );

        Module::reset(&mut module, &context(10_000_000));
        module.update(&context(10_000_000));
        assert_eq!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            padded(&[1.0, 2.0])
        );
    }

    // Not upstream: a rod count beyond the message capacity is truncated to
    // MAX_EFF_COUNT (upstream would read past its arrays).
    #[test]
    fn rod_count_is_truncated_to_the_message_capacity() {
        let mut mtb_config = MtbArrayConfigMsg {
            num_mtb: MAX_EFF_COUNT + 4,
            ..Default::default()
        };
        mtb_config.max_dipoles_am2.fill(10.0);
        let mtb_config_out = Output::new(mtb_config);
        let dipole_out = Output::new(dipole_msg(Vector3::new(1.0, 0.0, 0.0)));
        let mut module = DipoleMapping::new(DipoleMappingConfig {
            steering_matrix: vec![Vector3::x(); MAX_EFF_COUNT + 4],
            ..Default::default()
        });
        mtb_config_out.connect_to(&mut module.mtb_array_config_params_in_msg);
        dipole_out.connect_to(&mut module.dipole_request_body_in_msg);
        module.init();
        module.update(&context(0));

        assert_eq!(
            module.dipole_request_mtb_out_msg.read().dipole_cmds_am2,
            [1.0; MAX_EFF_COUNT]
        );
    }

    // Not upstream: the limit checks run upper first, as upstream, so a
    // negative limit yields -max rather than a panic, and NaN passes through.
    #[test]
    fn saturation_keeps_upstream_check_order() {
        assert_eq!(saturate(5.0, 2.0), 2.0);
        assert_eq!(saturate(-5.0, 2.0), -2.0);
        assert_eq!(saturate(1.5, 2.0), 1.5);
        assert_eq!(saturate(0.0, -1.0), 1.0);
        assert!(saturate(f64::NAN, 2.0).is_nan());
    }

    // Not upstream: mMultV sums each row from +0.0, so a zero request through
    // a negative steering row (every product -0.0) commands +0.0, bit for bit
    // as the compiled upstream C does.
    #[test]
    fn zero_request_commands_positive_zero_like_upstream() {
        let mut mtb_config = MtbArrayConfigMsg {
            num_mtb: 1,
            ..Default::default()
        };
        mtb_config.max_dipoles_am2[0] = 1.0;
        let mtb_config_out = Output::new(mtb_config);
        let dipole_out = Output::new(dipole_msg(Vector3::zeros()));
        let mut module = DipoleMapping::new(DipoleMappingConfig {
            steering_matrix: vec![Vector3::new(-1.0, -2.0, -3.0)],
            ..Default::default()
        });
        mtb_config_out.connect_to(&mut module.mtb_array_config_params_in_msg);
        dipole_out.connect_to(&mut module.dipole_request_body_in_msg);
        module.init();
        module.update(&context(0));

        let command = module.dipole_request_mtb_out_msg.read().dipole_cmds_am2[0];
        assert_eq!(command.to_bits(), 0.0_f64.to_bits());
    }
}
