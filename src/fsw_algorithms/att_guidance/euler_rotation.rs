//! Constant Euler-rate rotation of an attitude reference: port of upstream
//! Basilisk `eulerRotation`.
//!
//! The module superimposes on the input reference frame `R0` a rotation
//! `R/R0` given by a 3-2-1 Euler angle set `θ` that moves at constant Euler
//! angle rates `θ'`. Every update first integrates `θ += Δt θ'` over the time
//! since the previous update and then publishes
//!
//! - `[RN] = [RR0(θ)] [R0N]`,
//! - `ω_R/N = ω_R0/N + [RN]^T [B⁻¹(θ)] θ'`,
//! - `ω'_R/N = ω'_R0/N + ω_R0/N × ω_R/R0 + [RN]^T [dB⁻¹/dt] θ'`.
//!
//! The initial set and rates come from the configuration. When the optional
//! desired-attitude input (typically fed by `rasterManager`) is connected, a
//! command whose content differs from the last accepted one replaces both.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{AttitudeReferenceMsg, AttitudeStateMsg, Input, Output};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::is_equal_within;
use crate::utilities::rigid_body_kinematics::{
    b_inv_euler321, dcm_to_mrp, euler321_to_dcm, mrp_to_dcm,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`EulerRotation`]. `Default` is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct EulerRotationConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `angleSet` \[rad\]: initial 3-2-1 Euler angle set of `R`
    /// relative to the input reference `R0`.
    pub angle_set_rad: Vector3<f64>,
    /// Upstream `angleRates` \[rad/s\]: constant 3-2-1 Euler angle rates.
    pub angle_rates_radps: Vector3<f64>,
}

impl Default for EulerRotationConfig {
    fn default() -> Self {
        Self {
            name: "eulerRotation".to_string(),
            angle_set_rad: Vector3::zeros(),
            angle_rates_radps: Vector3::zeros(),
        }
    }
}

/// Reference frame rotating at constant 3-2-1 Euler angle rates relative to
/// an input reference: upstream `eulerRotation`.
#[derive(Clone, Debug)]
pub struct EulerRotation {
    /// User-settable parameters. The angle set and rates are copied into
    /// [`Self::angle_set_rad`] and [`Self::angle_rates_radps`] by
    /// [`Module::init`].
    pub config: EulerRotationConfig,
    /// Upstream `attRefInMsg`: input reference frame `R0`.
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Upstream `desiredAttInMsg` (optional): commanded 3-2-1 set and rates.
    pub desired_att_in_msg: Input<AttitudeStateMsg>,
    /// Upstream `attRefOutMsg`: rotating reference frame `R`.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `angleSet` \[rad\]: current 3-2-1 set `R/R0`, integrated on
    /// every update. Upstream lets the user overwrite it at any time; so does
    /// this public field. A reset leaves it unchanged, as upstream.
    pub angle_set_rad: Vector3<f64>,
    /// Upstream `angleRates` \[rad/s\]: current 3-2-1 Euler angle rates.
    pub angle_rates_radps: Vector3<f64>,
    command_latch: CommandLatch,
    prior_time_nanos: u64,
}

impl EulerRotation {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: EulerRotationConfig) -> Self {
        Self {
            angle_set_rad: config.angle_set_rad,
            angle_rates_radps: config.angle_rates_radps,
            config,
            att_ref_in_msg: Input::default(),
            desired_att_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            command_latch: CommandLatch::default(),
            prior_time_nanos: 0,
        }
    }

    /// Upstream `Reset_eulerRotation`: forget the prior call time and the
    /// last accepted command. The current angle set and rates are kept.
    fn reset_state(&mut self) {
        if !self.att_ref_in_msg.is_connected() {
            log::error!("eulerRotation.att_ref_in_msg is not connected");
        }
        self.prior_time_nanos = 0;
        self.command_latch = CommandLatch::default();
    }

    /// Upstream `computeEulerRotationReference`: advance the angle set by
    /// `dt_s` and build the rotating reference on top of `input`.
    fn rotated_reference(
        &mut self,
        input: &AttitudeReferenceMsg,
        dt_s: f64,
    ) -> AttitudeReferenceMsg {
        let dcm_r0n = mrp_to_dcm(input.sigma_rn);
        self.angle_set_rad += dt_s * self.angle_rates_radps;
        let dcm_rn = euler321_to_dcm(self.angle_set_rad) * dcm_r0n;

        let omega_rr0_r = b_inv_euler321(self.angle_set_rad) * self.angle_rates_radps;
        let omega_rr0_n = dcm_rn.transpose() * omega_rr0_r;

        let domega_rr0_r = b_inv_euler321_derivative(self.angle_set_rad, self.angle_rates_radps)
            * self.angle_rates_radps;
        let domega_rr0_n =
            input.omega_rn_n_radps.cross(&omega_rr0_n) + dcm_rn.transpose() * domega_rr0_r;

        AttitudeReferenceMsg {
            sigma_rn: dcm_to_mrp(dcm_rn),
            omega_rn_n_radps: input.omega_rn_n_radps + omega_rr0_n,
            domega_rn_n_radps2: domega_rr0_n + input.domega_rn_n_radps2,
        }
    }
}

/// Upstream `computeEuler321_Binv_derivative`: time derivative of
/// [`b_inv_euler321`] for the 3-2-1 set `angles` moving at `rates`.
///
/// Upstream bug, kept: the `(3, 1)` entry uses `-θ'₂ cos θ₃ cos θ₂` where the
/// derivative of `cos θ₂ cos θ₃` is `-θ'₂ cos θ₃ sin θ₂`. It only matters when
/// the second angle moves.
fn b_inv_euler321_derivative(angles: Vector3<f64>, rates: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = angles[1].sin_cos();
    let (s3, c3) = angles[2].sin_cos();
    Matrix3::new(
        -rates[1] * c2,
        0.0,
        0.0,
        rates[2] * c3 * c2 - rates[1] * s3 * s2,
        -rates[2] * s3,
        0.0,
        -rates[2] * s3 * c2 - rates[1] * c3 * c2,
        -rates[2] * c3,
        0.0,
    )
}

/// Upstream `computeTimeStep`, shared with `mrpRotation`: the integration
/// step \[s\] since `prior_time_nanos`.
///
/// Upstream quirk, kept: a prior time of zero means "no prior call" and gives
/// a zero step, so after a first call at t = 0 the second call integrates
/// nothing either.
pub(crate) fn integration_step_s(prior_time_nanos: u64, call_time_nanos: u64) -> f64 {
    if prior_time_nanos == 0 {
        0.0
    } else {
        diff_nanos_to_seconds(call_time_nanos, prior_time_nanos)
    }
}

/// Change detection of upstream `checkRasterCommands`, shared with
/// `mrpRotation`: remembers the last accepted desired-attitude command.
#[derive(Clone, Debug, Default)]
pub(crate) struct CommandLatch {
    prior: AttitudeStateMsg,
}

impl CommandLatch {
    /// Upstream's `v3IsEqual` tolerance for "the same command".
    const TOLERANCE: f64 = 1e-12;

    /// `Some(command)` when `command` differs from the last accepted one by
    /// more than 1e-12 in some component, and it then becomes the accepted
    /// command; `None` otherwise.
    ///
    /// Upstream quirk, kept: a fresh or reset latch holds the zero command, so
    /// an all-zero first command is ignored.
    pub(crate) fn accept(&mut self, command: AttitudeStateMsg) -> Option<AttitudeStateMsg> {
        let unchanged = is_equal_within(&command.state, &self.prior.state, Self::TOLERANCE)
            && is_equal_within(&command.rate, &self.prior.rate, Self::TOLERANCE);
        if unchanged {
            return None;
        }
        self.prior = command.clone();
        Some(command)
    }
}

impl Module for EulerRotation {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.angle_set_rad = self.config.angle_set_rad;
        self.angle_rates_radps = self.config.angle_rates_radps;
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let input = self.att_ref_in_msg.read();
        if self.desired_att_in_msg.is_connected()
            && let Some(command) = self.command_latch.accept(self.desired_att_in_msg.read())
        {
            self.angle_set_rad = command.state;
            self.angle_rates_radps = command.rate;
        }

        let dt_s = integration_step_s(self.prior_time_nanos, context.current_sim_nanos);
        let reference = self.rotated_reference(&input, dt_s);
        self.att_ref_out_msg.write(reference);
        self.prior_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports `eulerRotation/_UnitTest/test_eulerRotation.py`, both
    //! parametrizations (`run` and `run2`), with its hard-coded truth.
    //! `Support/results_eulerRotation.py` is a standalone script that printed
    //! that truth; the test does not import it.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeReferenceMsg, AttitudeStateMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, euler321_to_dcm, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{CommandLatch, EulerRotation, EulerRotationConfig, b_inv_euler321_derivative};

    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual} != {expected}");
        }
    }

    fn upstream_input_reference() -> AttitudeReferenceMsg {
        AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.1, 0.2, 0.3),
            omega_rn_n_radps: Vector3::new(0.1, 0.0, 0.0),
            domega_rn_n_radps2: Vector3::zeros(),
        }
    }

    /// Upstream `test_all_test_eulerRotation[run]` and `[run2]`: `run2` adds
    /// a zero desired-attitude command, which equals the reset latch and is
    /// therefore ignored, so both expect the same output at t = 0, 0.5, 1 and
    /// 1.5 s (`accuracy = 1e-12`).
    #[test]
    fn matches_upstream_euler_rotation_unit_test() {
        let true_sigma = [
            [-0.193031238249, 0.608048400483, 0.386062476497],
            [-0.193031238249, 0.608048400483, 0.386062476497],
            [-0.193144351314, 0.607931107381, 0.386360300559],
            [-0.193257454832, 0.607813704445, 0.386658117585],
        ];
        let true_omega = [0.101246280045, 0.000182644489, 0.001208139578];
        let true_domega = [0.000000000000e+00, -1.208139577635e-04, 1.826444892823e-05];
        let accuracy = 1e-12;

        for with_desired_attitude in [false, true] {
            let reference = Output::new(upstream_input_reference());
            let desired = Output::new(AttitudeStateMsg::default());
            let mut module = EulerRotation::new(EulerRotationConfig::default());
            module.config.angle_set_rad = Vector3::new(0.0, 90.0, 0.0) * D2R;
            module.config.angle_rates_radps = Vector3::new(0.1, 0.0, 0.0) * D2R;
            let mut recorder = MessageRecorder::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&reference, &mut module.att_ref_in_msg);
            if with_desired_attitude {
                simulation.connect(&desired, &mut module.desired_att_in_msg);
            }
            simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
            simulation.add_module("eulerRotation", &mut module, TASK_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_NANOS, 0);
            simulation.run_for(1_500_000_000);
            drop(simulation);

            assert_eq!(recorder.samples.len(), true_sigma.len());
            for (sample, sigma) in recorder.samples.iter().zip(&true_sigma) {
                assert_vector_close(sample.sigma_rn, Vector3::from(*sigma), accuracy);
                assert_vector_close(sample.omega_rn_n_radps, Vector3::from(true_omega), accuracy);
                assert_vector_close(
                    sample.domega_rn_n_radps2,
                    Vector3::from(true_domega),
                    accuracy,
                );
            }
        }
    }

    /// Not upstream: a nonzero desired command replaces the configured set and
    /// rates on the first update; the same command again is ignored, and a
    /// reset re-arms the latch so the command is applied once more.
    #[test]
    fn a_new_desired_command_restarts_the_rotation() {
        let input = upstream_input_reference();
        let command = AttitudeStateMsg {
            state: Vector3::new(0.3, -0.2, 0.1),
            rate: Vector3::new(0.01, 0.02, -0.03),
        };
        let reference = Output::new(input.clone());
        let desired = Output::new(command.clone());
        let mut module = EulerRotation::new(EulerRotationConfig {
            angle_set_rad: Vector3::new(1.0, 0.5, -0.5),
            angle_rates_radps: Vector3::new(0.1, 0.1, 0.1),
            ..Default::default()
        });
        reference.connect_to(&mut module.att_ref_in_msg);
        desired.connect_to(&mut module.desired_att_in_msg);
        module.init();

        let expected_sigma =
            |angles: Vector3<f64>| dcm_to_mrp(euler321_to_dcm(angles) * mrp_to_dcm(input.sigma_rn));
        module.update(&context(TASK_NANOS));
        assert_eq!(module.angle_set_rad, command.state);
        assert_eq!(module.angle_rates_radps, command.rate);
        assert_vector_close(
            module.att_ref_out_msg.read().sigma_rn,
            expected_sigma(command.state),
            1e-15,
        );

        module.update(&context(2 * TASK_NANOS));
        let integrated = command.state + 0.5 * command.rate;
        assert_vector_close(module.angle_set_rad, integrated, 1e-15);
        assert_vector_close(
            module.att_ref_out_msg.read().sigma_rn,
            expected_sigma(integrated),
            1e-15,
        );

        Module::reset(&mut module, &context(2 * TASK_NANOS));
        module.update(&context(3 * TASK_NANOS));
        assert_eq!(module.angle_set_rad, command.state);
    }

    /// Not upstream: a command within upstream's 1e-12 `v3IsEqual` tolerance
    /// of the accepted one is the same command; one just outside it is new.
    #[test]
    fn a_command_within_the_tolerance_is_not_new() {
        let command = AttitudeStateMsg {
            state: Vector3::new(0.3, -0.2, 0.1),
            rate: Vector3::new(0.01, 0.02, -0.03),
        };
        let nudged = |delta: f64| AttitudeStateMsg {
            state: command.state + Vector3::new(0.0, delta, 0.0),
            rate: command.rate,
        };
        let mut latch = CommandLatch::default();
        assert_eq!(latch.accept(command.clone()), Some(command.clone()));
        assert_eq!(latch.accept(nudged(0.5e-12)), None);
        assert_eq!(latch.accept(nudged(2e-12)), Some(nudged(2e-12)));
    }

    /// Not upstream: with the second angle moving, the angular acceleration
    /// keeps upstream's `(3, 1)` entry of the `B⁻¹` derivative, which uses
    /// `cos θ₂` where the exact derivative has `sin θ₂`.
    #[test]
    fn keeps_upstream_b_inverse_derivative_entry() {
        let angles = Vector3::new(0.2, 0.4, 0.6);
        let rates = Vector3::new(0.0, 0.5, 0.0);
        let derivative = b_inv_euler321_derivative(angles, rates);
        assert_eq!(derivative[(2, 0)], -0.5 * 0.6_f64.cos() * 0.4_f64.cos());
        assert_eq!(derivative[(1, 0)], -0.5 * 0.6_f64.sin() * 0.4_f64.sin());
        assert_eq!(derivative[(0, 0)], -0.5 * 0.4_f64.cos());
    }

    /// Not upstream: the configured set is read at `init`, so a configuration
    /// changed after construction takes effect; a reset keeps the integrated
    /// set, as upstream.
    #[test]
    fn reads_the_configuration_at_init_and_keeps_the_set_across_reset() {
        let reference = Output::new(AttitudeReferenceMsg::default());
        let mut module = EulerRotation::new(EulerRotationConfig::default());
        module.config.angle_set_rad = Vector3::new(0.1, 0.2, 0.3);
        module.config.angle_rates_radps = Vector3::new(0.02, 0.0, 0.0);
        reference.connect_to(&mut module.att_ref_in_msg);
        module.init();
        for tick in 1..=3 {
            module.update(&context(tick * TASK_NANOS));
        }
        let integrated = Vector3::new(0.1 + 2.0 * 0.5 * 0.02, 0.2, 0.3);
        assert_vector_close(module.angle_set_rad, integrated, 1e-15);

        Module::reset(&mut module, &context(3 * TASK_NANOS));
        module.update(&context(4 * TASK_NANOS));
        assert_vector_close(module.angle_set_rad, integrated, 1e-15);
    }
}
