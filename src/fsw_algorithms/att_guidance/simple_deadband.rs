//! Two-level attitude-error deadband: port of upstream Basilisk
//! `simpleDeadband`.
//!
//! The module passes the attitude guidance message through, except that it
//! zeroes the attitude and rate tracking errors `σ_B/R` and `ω_B/R` (turning
//! the downstream control off) while the errors are small. The scalar errors
//! are the principal rotation angle `4 atan |σ_B/R|` and `|ω_B/R|`. With
//! inner and outer thresholds on both, the logic has hysteresis:
//!
//! - both errors below their inner thresholds: control off;
//! - both below their outer thresholds, but not both below the inner ones:
//!   control stays as it was;
//! - otherwise: control on.
//!
//! After a reset the control counts as off. `ω_R/N` and its derivative always
//! pass through.

use nalgebra::Vector3;

use crate::messages::{AttitudeGuidanceMsg, Input, Output};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`SimpleDeadband`]. `Default` is upstream's
/// zero-initialized configuration, which never turns the control off.
#[derive(Clone, Debug)]
pub struct SimpleDeadbandConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `innerAttThresh` \[rad\]: attitude error below which control
    /// turns off (with the rate error below its inner threshold too).
    pub inner_att_thresh_rad: f64,
    /// Upstream `outerAttThresh` \[rad\]: attitude error above which control
    /// turns on.
    pub outer_att_thresh_rad: f64,
    /// Upstream `innerRateThresh` \[rad/s\]: rate error below which control
    /// turns off (with the attitude error below its inner threshold too).
    pub inner_rate_thresh_radps: f64,
    /// Upstream `outerRateThresh` \[rad/s\]: rate error above which control
    /// turns on.
    pub outer_rate_thresh_radps: f64,
}

impl Default for SimpleDeadbandConfig {
    fn default() -> Self {
        Self {
            name: "simpleDeadband".to_string(),
            inner_att_thresh_rad: 0.0,
            outer_att_thresh_rad: 0.0,
            inner_rate_thresh_radps: 0.0,
            outer_rate_thresh_radps: 0.0,
        }
    }
}

/// Deadband on the attitude guidance errors: upstream `simpleDeadband`.
#[derive(Clone, Debug)]
pub struct SimpleDeadband {
    /// User-settable parameters, read on every update.
    pub config: SimpleDeadbandConfig,
    /// Upstream `guidInMsg`: attitude guidance to filter.
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `attGuidOutMsg`: guidance with the errors zeroed while the
    /// control is off.
    pub att_guid_out_msg: Output<AttitudeGuidanceMsg>,
    /// Upstream `attError` \[rad\]: principal-angle attitude error of the
    /// last update.
    pub att_error_rad: f64,
    /// Upstream `rateError` \[rad/s\]: rate error magnitude of the last
    /// update.
    pub rate_error_radps: f64,
    /// Upstream `wasControlOff`.
    was_control_off: bool,
}

impl SimpleDeadband {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: SimpleDeadbandConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            att_guid_out_msg: Output::default(),
            att_error_rad: 0.0,
            rate_error_radps: 0.0,
            was_control_off: true,
        }
    }

    /// Upstream `Reset_simpleDeadband`: the control counts as off.
    fn reset_state(&mut self) {
        if !self.guid_in_msg.is_connected() {
            log::error!("simpleDeadband.guid_in_msg is not connected");
        }
        self.was_control_off = true;
    }

    /// Upstream `applyDBLogic_simpleDeadband`: whether the control is off
    /// for the current errors, given whether it was off before.
    fn control_off(&self) -> bool {
        let config = &self.config;
        let below_outer = self.att_error_rad < config.outer_att_thresh_rad
            && self.rate_error_radps < config.outer_rate_thresh_radps;
        let below_inner = self.att_error_rad < config.inner_att_thresh_rad
            && self.rate_error_radps < config.inner_rate_thresh_radps;
        below_outer && (below_inner || self.was_control_off)
    }
}

impl Module for SimpleDeadband {
    fn init(&mut self) {
        self.att_guid_out_msg.write(AttitudeGuidanceMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let mut guidance = self.guid_in_msg.read();
        self.att_error_rad = 4.0 * guidance.sigma_br.norm().atan();
        self.rate_error_radps = guidance.omega_br_b_radps.norm();

        self.was_control_off = self.control_off();
        if self.was_control_off {
            guidance.sigma_br = Vector3::zeros();
            guidance.omega_br_b_radps = Vector3::zeros();
        }
        self.att_guid_out_msg.write(guidance);
    }
}

#[cfg(test)]
mod tests {
    //! Upstream has no unit test for `simpleDeadband`; these tests follow the
    //! `.rst` documentation and `simpleDeadband.c`, with hand-derived truth.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeGuidanceMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{SimpleDeadband, SimpleDeadbandConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Guidance with a principal attitude error of `att_error_rad` about an
    /// oblique axis and a rate error of magnitude `rate_error_radps`.
    fn guidance(att_error_rad: f64, rate_error_radps: f64) -> AttitudeGuidanceMsg {
        let axis = Vector3::new(1.0, -2.0, 2.0) / 3.0;
        AttitudeGuidanceMsg {
            sigma_br: (att_error_rad / 4.0).tan() * axis,
            omega_br_b_radps: rate_error_radps * Vector3::new(0.0, 0.6, -0.8),
            omega_rn_b_radps: Vector3::new(0.01, 0.02, 0.03),
            domega_rn_b_radps2: Vector3::new(-1e-4, 2e-4, -3e-4),
        }
    }

    fn deadband() -> SimpleDeadband {
        SimpleDeadband::new(SimpleDeadbandConfig {
            inner_att_thresh_rad: 0.1,
            outer_att_thresh_rad: 0.2,
            inner_rate_thresh_radps: 0.01,
            outer_rate_thresh_radps: 0.02,
            ..Default::default()
        })
    }

    /// Runs the module from `init` through `(attitude error, rate error)`
    /// inputs and returns whether the control was off after each update,
    /// checking that the output errors are zeroed exactly then.
    fn control_off_sequence(module: &mut SimpleDeadband, inputs: &[(f64, f64)]) -> Vec<bool> {
        let input = Output::new(AttitudeGuidanceMsg::default());
        input.connect_to(&mut module.guid_in_msg);
        module.init();
        inputs
            .iter()
            .enumerate()
            .map(|(tick, &(att, rate))| {
                let message = guidance(att, rate);
                input.write(message.clone());
                module.update(&context(tick as u64 * 500_000_000));
                let off = module.was_control_off;
                let output = module.att_guid_out_msg.read();
                let (sigma_br, omega_br_b) = if off {
                    (Vector3::zeros(), Vector3::zeros())
                } else {
                    (message.sigma_br, message.omega_br_b_radps)
                };
                assert_eq!(output.sigma_br, sigma_br);
                assert_eq!(output.omega_br_b_radps, omega_br_b);
                assert_eq!(output.omega_rn_b_radps, message.omega_rn_b_radps);
                assert_eq!(output.domega_rn_b_radps2, message.domega_rn_b_radps2);
                off
            })
            .collect()
    }

    /// The hysteresis loop: between the thresholds the control keeps its
    /// previous state, starting from "off" after a reset.
    #[test]
    fn applies_the_two_level_deadband_with_hysteresis() {
        let mut module = deadband();
        let inputs = [
            (0.15, 0.015), // between the thresholds, off after reset: stays off
            (0.25, 0.005), // attitude above outer: on
            (0.15, 0.005), // between: stays on
            (0.05, 0.015), // rate between: stays on
            (0.05, 0.005), // both below inner: off
            (0.15, 0.015), // between: stays off
            (0.05, 0.025), // rate above outer: on
        ];
        let expected = [true, false, false, false, true, true, false];
        assert_eq!(control_off_sequence(&mut module, &inputs), expected);
    }

    /// The scalar errors are the principal rotation angle and the rate norm.
    #[test]
    fn evaluates_the_principal_angle_and_rate_errors() {
        let mut module = deadband();
        control_off_sequence(&mut module, &[(0.3, 0.04)]);
        assert!((module.att_error_rad - 0.3).abs() <= 1e-15);
        assert!((module.rate_error_radps - 0.04).abs() <= 1e-15);
    }

    /// The thresholds are strict: an error equal to a threshold is not below
    /// it.
    #[test]
    fn thresholds_are_strict() {
        let mut module = SimpleDeadband::new(SimpleDeadbandConfig {
            inner_att_thresh_rad: 0.0,
            outer_att_thresh_rad: 1.0,
            inner_rate_thresh_radps: 0.0,
            outer_rate_thresh_radps: 1.0,
            ..Default::default()
        });
        // Zero errors are not below zero inner thresholds, but they are below
        // the outer ones, so the control stays off after the reset.
        assert_eq!(control_off_sequence(&mut module, &[(0.0, 0.0)]), [true]);
    }

    /// Upstream's zeroed thresholds never turn the control off; a reset
    /// returns the logic to "off" in between.
    #[test]
    fn zero_thresholds_always_pass_through_and_reset_rearms() {
        let mut module = SimpleDeadband::new(SimpleDeadbandConfig::default());
        assert_eq!(
            control_off_sequence(&mut module, &[(0.0, 0.0), (0.5, 0.1)]),
            [false, false]
        );

        let mut module = deadband();
        control_off_sequence(&mut module, &[(0.25, 0.005)]);
        assert!(!module.was_control_off);
        Module::reset(&mut module, &context(1));
        assert!(module.was_control_off);
    }
}
