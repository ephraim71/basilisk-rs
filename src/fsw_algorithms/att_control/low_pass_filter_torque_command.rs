//! First-order low-pass filter on a commanded body torque: port of upstream
//! Basilisk `lowPassFilterTorqueCommand`.
//!
//! The filter is the Tustin (bilinear) discretization, at the fixed step `h`,
//! of the continuous first-order low-pass filter with cut-off frequency `ω_c`.
//! The cut-off is prewarped, `hw = 2 tan(ω_c h / 2)`, and each update computes
//!
//! `L_F,k = (hw (L_k + L_{k-1}) + (2 - hw) L_F,k-1) / (2 + hw)`
//!
//! from the current command `L_k`, the previous command and the previous
//! filtered command. The first update after a reset starts the filter: the
//! previous command is taken equal to the current one, the previous output is
//! zero, and the coefficients are computed from the configuration at that
//! moment and kept until the next reset.

use nalgebra::Vector3;

use crate::messages::{BodyTorqueCommandMsg, Input, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`LowPassFilterTorqueCommand`]. `Default` is
/// upstream's zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct LowPassFilterTorqueCommandConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `h` \[s\]: filter time step, assumed fixed and equal to the
    /// module's update period.
    pub h_step_s: f64,
    /// Upstream `wc` \[rad/s\]: continuous-time cut-off frequency.
    pub omega_cutoff_radps: f64,
}

impl Default for LowPassFilterTorqueCommandConfig {
    fn default() -> Self {
        Self {
            name: "lowPassFilterTorqueCommand".to_string(),
            h_step_s: 0.0,
            omega_cutoff_radps: 0.0,
        }
    }
}

/// Coefficients of the discrete filter: upstream `a` (applied to the outputs)
/// and `b` (applied to the inputs), newest first.
#[derive(Clone, Copy, Debug, PartialEq)]
struct FilterCoefficients {
    a: [f64; 2],
    b: [f64; 2],
}

impl FilterCoefficients {
    /// Upstream's start-up computation of `hw`, `a` and `b`.
    fn first_order(h_step_s: f64, omega_cutoff_radps: f64) -> Self {
        // Upstream `hw`: h times the prewarped discrete-time cut-off frequency.
        let hw = (omega_cutoff_radps * h_step_s / 2.0).tan() * 2.0;
        Self {
            a: [2.0 + hw, 2.0 - hw],
            b: [hw, hw],
        }
    }
}

/// Low-pass filter of a body torque command: upstream
/// `lowPassFilterTorqueCommand`.
#[derive(Clone, Debug)]
pub struct LowPassFilterTorqueCommand {
    /// User-settable parameters, read when the filter starts.
    pub config: LowPassFilterTorqueCommandConfig,
    /// Upstream `cmdTorqueInMsg`: unfiltered torque command.
    pub cmd_torque_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `cmdTorqueOutMsg`: filtered torque command.
    pub cmd_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `Lr[1]` \[N m\]: previous unfiltered command.
    previous_command_nm: Vector3<f64>,
    /// Upstream `LrF[1]` \[N m\]: previous filtered command.
    previous_filtered_nm: Vector3<f64>,
    /// Coefficients of the running filter. `None` is upstream's `reset` flag:
    /// the next update starts the filter.
    coefficients: Option<FilterCoefficients>,
}

impl LowPassFilterTorqueCommand {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: LowPassFilterTorqueCommandConfig) -> Self {
        Self {
            config,
            cmd_torque_in_msg: Input::default(),
            cmd_torque_out_msg: Output::default(),
            previous_command_nm: Vector3::zeros(),
            previous_filtered_nm: Vector3::zeros(),
            coefficients: None,
        }
    }

    /// Upstream `Reset_lowPassFilterTorqueCommand`: clears the history and
    /// makes the next update restart the filter.
    fn reset_filter(&mut self) {
        if !self.cmd_torque_in_msg.is_connected() {
            log::error!("lowPassFilterTorqueCommand.cmd_torque_in_msg is not connected");
        }
        self.coefficients = None;
        self.previous_command_nm = Vector3::zeros();
        self.previous_filtered_nm = Vector3::zeros();
    }

    /// Upstream's start-up branch: seeds the history with `command_nm` and a
    /// zero output, and latches the coefficients of the current configuration.
    fn start_filter(&mut self, command_nm: Vector3<f64>) -> FilterCoefficients {
        self.previous_command_nm = command_nm;
        self.previous_filtered_nm = Vector3::zeros();
        let coefficients =
            FilterCoefficients::first_order(self.config.h_step_s, self.config.omega_cutoff_radps);
        self.coefficients = Some(coefficients);
        coefficients
    }

    /// One filter step on `command_nm`; returns the filtered command \[N m\].
    fn filter(&mut self, command_nm: Vector3<f64>) -> Vector3<f64> {
        let FilterCoefficients { a, b } = match self.coefficients {
            Some(coefficients) => coefficients,
            None => self.start_filter(command_nm),
        };
        // Accumulated from zero in upstream's order, then scaled by 1/a[0]
        // (a multiplication, as upstream, not a division).
        let weighted_history = [
            (b[0], command_nm),
            (b[1], self.previous_command_nm),
            (a[1], self.previous_filtered_nm),
        ];
        let weighted_sum = weighted_history
            .into_iter()
            .fold(Vector3::zeros(), |sum, (weight, torque)| {
                torque * weight + sum
            });
        let filtered_nm = weighted_sum * (1.0 / a[0]);
        self.previous_command_nm = command_nm;
        self.previous_filtered_nm = filtered_nm;
        filtered_nm
    }
}

impl Module for LowPassFilterTorqueCommand {
    fn init(&mut self) {
        self.cmd_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.reset_filter();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_filter();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let command_nm = self.cmd_torque_in_msg.read().torque_request_body_nm;
        let torque_request_body_nm = self.filter(command_nm);
        self.cmd_torque_out_msg.write(BodyTorqueCommandMsg {
            torque_request_body_nm,
        });
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{LowPassFilterTorqueCommand, LowPassFilterTorqueCommandConfig};
    use crate::messages::{BodyTorqueCommandMsg, Output};
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn torque(torque_request_body_nm: Vector3<f64>) -> BodyTorqueCommandMsg {
        BodyTorqueCommandMsg {
            torque_request_body_nm,
        }
    }

    /// The configuration `test_lowPassFilterTorqueCommand.py` sets, applied
    /// after construction as upstream does.
    fn upstream_module(input: &Output<BodyTorqueCommandMsg>) -> LowPassFilterTorqueCommand {
        let mut module =
            LowPassFilterTorqueCommand::new(LowPassFilterTorqueCommandConfig::default());
        module.config.omega_cutoff_radps = 0.1 * PI * 2.0;
        module.config.h_step_s = 0.5;
        input.connect_to(&mut module.cmd_torque_in_msg);
        module
    }

    /// Upstream `isArrayEqual`: absolute tolerance per component, NaN fails.
    fn assert_array_equal(actual: Vector3<f64>, truth: [f64; 3], accuracy: f64, what: &str) {
        for (index, (value, expected)) in actual.iter().zip(truth).enumerate() {
            assert!(
                (value - expected).abs() <= accuracy,
                "{what}[{index}] = {value:.17e}, truth {expected:.17e}"
            );
        }
    }

    /// `test_lowPassFilterControlTorque` with upstream's hard-coded truth and
    /// accuracy: a 0.5 s task run to 1 s, `module.Reset(1)`, then run to 2 s.
    /// Upstream also sets `module.reset = 1` before initializing, which the
    /// initial reset already does.
    #[test]
    fn matches_upstream_low_pass_filter_control_torque_unit_test() {
        let input = Output::new(torque(Vector3::new(1.0, -0.5, 0.7)));
        let mut module = upstream_module(&input);
        module.init();

        let mut samples = Vec::new();
        for tick in 0..=2 {
            module.update(&context(tick * TASK_NANOS));
            samples.push(module.cmd_torque_out_msg.read().torque_request_body_nm);
        }
        module.reset(&context(1));
        for tick in 3..=4 {
            module.update(&context(tick * TASK_NANOS));
            samples.push(module.cmd_torque_out_msg.read().torque_request_body_nm);
        }

        let truth = [
            [0.2734574719946391, -0.1367287359973196, 0.1914202303962474],
            [0.4721359549995794, -0.2360679774997897, 0.3304951684997055],
            [0.6164843223022588, -0.3082421611511294, 0.4315390256115811],
            [0.2734574719946391, -0.1367287359973196, 0.1914202303962474],
            [0.4721359549995794, -0.2360679774997897, 0.3304951684997055],
        ];
        assert_eq!(samples.len(), truth.len());
        for (tick, (sample, expected)) in samples.iter().zip(truth).enumerate() {
            assert_array_equal(*sample, expected, 1e-12, &format!("LrF at tick {tick}"));
        }
    }

    /// Not upstream: a varying command against the Tustin recursion written
    /// out independently, with the start-up seeding the previous input with the
    /// first command and the previous output with zero.
    #[test]
    fn follows_the_tustin_recursion_for_a_varying_command() {
        let input = Output::new(BodyTorqueCommandMsg::default());
        let mut module = upstream_module(&input);
        module.init();

        let hw = 2.0 * (0.1 * PI * 2.0 * 0.5 / 2.0_f64).tan();
        let commands = [
            Vector3::new(1.0, -0.5, 0.7),
            Vector3::new(-2.0, 0.25, 0.0),
            Vector3::new(0.3, 3.0, -1.5),
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(5.0, -4.0, 2.0),
        ];
        let mut previous_command = commands[0];
        let mut previous_output = Vector3::zeros();
        for (tick, command) in commands.iter().enumerate() {
            input.write(torque(*command));
            module.update(&context(tick as u64 * TASK_NANOS));
            let expected =
                (hw * (command + previous_command) + (2.0 - hw) * previous_output) / (2.0 + hw);
            let actual = module.cmd_torque_out_msg.read().torque_request_body_nm;
            assert!((actual - expected).abs().max() <= 1e-14, "tick {tick}");
            previous_command = *command;
            previous_output = expected;
        }
    }

    /// Not upstream: the filter has unit DC gain, so a constant command is
    /// reached once the start-up transient has decayed.
    #[test]
    fn converges_to_a_constant_command() {
        let command = Vector3::new(1.0, -0.5, 0.7);
        let input = Output::new(torque(command));
        let mut module = upstream_module(&input);
        module.init();
        for tick in 0..200 {
            module.update(&context(tick * TASK_NANOS));
        }
        let output = module.cmd_torque_out_msg.read().torque_request_body_nm;
        assert!((output - command).abs().max() <= 1e-12, "{output}");
    }

    /// Not upstream: as upstream, the coefficients are computed when the
    /// filter starts and a later configuration change only applies after the
    /// next reset.
    #[test]
    fn latches_the_coefficients_until_the_next_reset() {
        let input = Output::new(torque(Vector3::new(1.0, -0.5, 0.7)));
        let mut module = upstream_module(&input);
        module.init();
        module.update(&context(0));
        let first = module.cmd_torque_out_msg.read().torque_request_body_nm;

        module.config.omega_cutoff_radps *= 4.0;
        module.update(&context(TASK_NANOS));
        assert_array_equal(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            [0.4721359549995794, -0.2360679774997897, 0.3304951684997055],
            1e-12,
            "second sample with the old cut-off",
        );

        module.reset(&context(TASK_NANOS));
        module.update(&context(2 * TASK_NANOS));
        let restarted = module.cmd_torque_out_msg.read().torque_request_body_nm;
        let hw = 2.0 * (0.4 * PI * 2.0 * 0.5 / 2.0_f64).tan();
        let expected = Vector3::new(1.0, -0.5, 0.7) * (2.0 * hw / (2.0 + hw));
        assert!((restarted - expected).abs().max() <= 1e-14);
        assert!((restarted - first).norm() > 0.1);
    }

    #[test]
    fn init_publishes_a_zero_torque() {
        let mut module =
            LowPassFilterTorqueCommand::new(LowPassFilterTorqueCommandConfig::default());
        module
            .cmd_torque_out_msg
            .write(torque(Vector3::new(1.0, 2.0, 3.0)));
        module.init();
        assert_eq!(
            module.cmd_torque_out_msg.read().torque_request_body_nm,
            Vector3::zeros()
        );
    }
}
