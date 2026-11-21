//! Limits on one actuator command: port of upstream Basilisk
//! `simulation/deviceInterface/saturationSingleActuator`.
//!
//! Each update reads one [`SingleActuatorMsg`] command and republishes it
//! clamped to `[min_input, max_input]`, or unchanged while saturation is
//! disabled. The clamp is upstream's `std::max(min, std::min(u, max))`
//! written out, so a NaN command saturates to `min_input` as it does
//! upstream, and inverted limits give `min_input`.

use crate::messages::{Input, Output, SingleActuatorMsg};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`SaturationSingleActuator`].
#[derive(Clone, Debug, PartialEq)]
pub struct SaturationSingleActuatorConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `saturationEnabled` (`setSaturationEnabled`): clamp the
    /// command when set, pass it through otherwise.
    pub saturation_enabled: bool,
    /// Upstream `minInput` (`setMinInput`): lowest command passed on.
    pub min_input: f64,
    /// Upstream `maxInput` (`setMaxInput`): highest command passed on.
    pub max_input: f64,
}

impl Default for SaturationSingleActuatorConfig {
    /// Upstream's defaults: saturation enabled, with infinite limits.
    fn default() -> Self {
        Self {
            name: "saturationSingleActuator".to_string(),
            saturation_enabled: true,
            min_input: f64::NEG_INFINITY,
            max_input: f64::INFINITY,
        }
    }
}

impl SaturationSingleActuatorConfig {
    /// Upstream `applySaturation`: `input` limited to the configured range,
    /// or unchanged with saturation disabled.
    pub fn apply(&self, input: f64) -> f64 {
        if !self.saturation_enabled {
            return input;
        }
        // `std::min(u, max)` then `std::max(min, ·)`, comparisons as upstream.
        let upper_limited = if self.max_input < input {
            self.max_input
        } else {
            input
        };
        if self.min_input < upper_limited {
            upper_limited
        } else {
            self.min_input
        }
    }
}

/// Saturates one actuator command: upstream `SaturationSingleActuator`.
#[derive(Clone, Debug)]
pub struct SaturationSingleActuator {
    /// Module configuration, read every update.
    pub config: SaturationSingleActuatorConfig,
    /// Upstream `actuatorInMsg`: the raw command.
    pub actuator_in_msg: Input<SingleActuatorMsg>,
    /// Upstream `actuatorOutMsg`: the limited command.
    pub actuator_out_msg: Output<SingleActuatorMsg>,
}

impl SaturationSingleActuator {
    /// Builds the module with unconnected ports.
    pub fn new(config: SaturationSingleActuatorConfig) -> Self {
        Self {
            config,
            actuator_in_msg: Input::default(),
            actuator_out_msg: Output::default(),
        }
    }
}

impl Module for SaturationSingleActuator {
    /// Upstream has no `Reset`; the output keeps its last value.
    fn init(&mut self) {
        if !self.actuator_in_msg.is_connected() {
            log::error!(
                "saturationSingleActuator '{}' actuator_in_msg is not connected",
                self.config.name
            );
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        let command = self.actuator_in_msg.read();
        self.actuator_out_msg.write(SingleActuatorMsg {
            input: self.config.apply(command.input),
        });
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream
    //! `saturationSingleActuator/_UnitTest/test_saturationSingleActuator.py`
    //! with both settings of `saturationEnabled`.

    use hifitime::Epoch;

    use super::{SaturationSingleActuator, SaturationSingleActuatorConfig};
    use crate::messages::{Output, SingleActuatorMsg};
    use crate::{Module, SimulationContext};

    fn limiter(saturation_enabled: bool) -> SaturationSingleActuator {
        SaturationSingleActuator::new(SaturationSingleActuatorConfig {
            saturation_enabled,
            min_input: -0.2,
            max_input: 0.15,
            ..Default::default()
        })
    }

    #[test]
    fn limits_only_while_saturation_is_enabled() {
        let context = SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        for saturation_enabled in [false, true] {
            for value in [-0.5, -0.2, -0.1, 0.0, 0.1, 0.15, 0.3] {
                let command_out = Output::new(SingleActuatorMsg { input: value });
                let mut module = limiter(saturation_enabled);
                command_out.connect_to(&mut module.actuator_in_msg);
                module.init();
                module.update(&context);

                let truth = if saturation_enabled {
                    value.clamp(-0.2, 0.15)
                } else {
                    value
                };
                let out = module.actuator_out_msg.read().input;
                assert!(
                    (out - truth).abs() <= 1e-14,
                    "enabled = {saturation_enabled}, {value}: {out} vs {truth}"
                );
            }
        }
    }

    #[test]
    fn default_limits_pass_every_finite_command() {
        let config = SaturationSingleActuatorConfig::default();
        assert!(config.saturation_enabled);
        for value in [-1e300, -1.0, 0.0, 2.5, 1e300] {
            assert_eq!(config.apply(value), value);
        }
    }

    /// Upstream's `std::max`/`std::min` order: NaN and inverted limits give
    /// the lower limit.
    #[test]
    fn nan_and_inverted_limits_follow_upstreams_comparisons() {
        let config = limiter(true).config;
        assert_eq!(config.apply(f64::NAN), -0.2);
        assert!(limiter(false).config.apply(f64::NAN).is_nan());

        let inverted = SaturationSingleActuatorConfig {
            min_input: 1.0,
            max_input: -1.0,
            ..Default::default()
        };
        for value in [-2.0, 0.0, 2.0] {
            assert_eq!(inverted.apply(value), 1.0);
        }
    }
}
