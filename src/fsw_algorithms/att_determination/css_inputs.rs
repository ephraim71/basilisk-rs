//! Coarse-sun-sensor inputs shared by the five sunline filters (`sunlineUKF`,
//! `sunlineSuKF`, `sunlineEKF`, `sunlineSEKF` and `okeefeEKF`): one
//! [`SunSensorMsg`] input per configured sensor, and the stand-in for the
//! upstream message header those inputs lack.

use nalgebra::Vector3;

use crate::SimulationContext;
use crate::messages::{Input, MAX_N_CSS_MEAS, SunSensorMsg};

/// One coarse sun sensor as the sunline filters model it: an entry of
/// upstream's `CSSConfigMsg` (`CSSUnitConfigMsgPayload`), which the filters
/// take as configuration rather than as a message.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct CssUnit {
    /// Upstream `nHat_B` [-]: sensor unit normal, body components.
    pub n_hat_b: Vector3<f64>,
    /// Upstream `CBias` [-]: calibration gain of this sensor relative to the
    /// others — 1.0 nominally, 1.1 for a sensor reading 10 % strong. (Upstream
    /// labels it \[W\], but it multiplies a dimensionless cosine.)
    pub c_bias: f64,
}

/// One unconnected CSS input per configured sensor. Not `vec![_; n]`:
/// cloning a port aliases it, so every sensor would share one connection and
/// one override stack.
pub(crate) fn css_inputs(css_units: &[CssUnit]) -> Vec<Input<SunSensorMsg>> {
    css_units.iter().map(|_| Input::default()).collect()
}

/// Upstream Reset's check that `cssDataInMsg` is linked, per sensor input.
///
/// # Panics
///
/// If there is not one input per configured sensor: the filters make their
/// inputs from the configuration in `new`, which fixes the sensor count.
pub(crate) fn check_css_inputs(
    inputs: &[Input<SunSensorMsg>],
    css_units: &[CssUnit],
    module: &str,
) {
    assert_eq!(
        inputs.len(),
        css_units.len(),
        "{module} needs one css_data_in_msgs input per config.css_units sensor",
    );
    for (index, input) in inputs.iter().enumerate() {
        if !input.is_connected() {
            log::error!("{module}.css_data_in_msgs[{index}] is not connected");
        }
    }
}

/// The configured sensors a filter uses (upstream `numCSSTotal`, `cssNHat_B`,
/// `CBias`). Upstream logs an error for more than `MAX_N_CSS_MEAS` sensors
/// and then overruns its arrays; the port keeps the first `MAX_N_CSS_MEAS`.
pub(crate) fn configured_css_units(css_units: &[CssUnit], module: &str) -> Vec<CssUnit> {
    if css_units.len() > MAX_N_CSS_MEAS {
        log::error!(
            "{module}.config.css_units must not hold more than MAX_N_CSS_MEAS ({MAX_N_CSS_MEAS}) sensors"
        );
    }
    css_units[..css_units.len().min(MAX_N_CSS_MEAS)].to_vec()
}

/// Upstream `cssSensorInBuffer.CosValue`: the `sensed_value` of every sensor
/// input, in sensor order. An unconnected input reads zero, as upstream's
/// unwritten message does.
pub(crate) fn css_readings(inputs: &[Input<SunSensorMsg>]) -> Vec<f64> {
    inputs
        .iter()
        .map(|input| input.read().sensed_value)
        .collect()
}

/// Upstream's CSS message header (`isWritten`, `timeWritten`) as far as the
/// ports can tell: [`SunSensorMsg`] carries no header and no time tag, so the
/// readings count as written at the current update when every sensor input
/// is connected (and there is at least one) — what upstream sees when the
/// CSS producers run before the filter on every tick.
pub(crate) fn css_written_nanos(
    inputs: &[Input<SunSensorMsg>],
    context: &SimulationContext,
) -> Option<u64> {
    let all_connected = !inputs.is_empty() && inputs.iter().all(Input::is_connected);
    all_connected.then_some(context.current_sim_nanos)
}

#[cfg(test)]
pub(crate) mod test_support {
    //! Replays the message timeline of the upstream sunline-filter pytests.

    /// Upstream's 0.5 s task rate (`macros.sec2nano(0.5)`).
    pub(crate) const TASK_RATE_NANOS: u64 = 500_000_000;

    /// The pytests' CSS message between `ExecuteSimulation` calls:
    /// `cssDataInMsg.write(data, CurrentNanos)` stamps it with the time of the
    /// last executed tick, and the next call runs every tick up to its stop
    /// time. (The first call after initialization runs both t = 0 and the
    /// first tick.) The readings are one per sensor, in sensor order.
    pub(crate) struct UpstreamTimeline {
        current_nanos: u64,
        next_tick_nanos: u64,
        readings: Vec<f64>,
        written_nanos: Option<u64>,
    }

    impl UpstreamTimeline {
        /// The timeline of a filter with `num_sensors` sensors. Until the
        /// first write the message reads zeros, as upstream's unwritten one.
        pub(crate) fn new(num_sensors: usize) -> Self {
            Self {
                current_nanos: 0,
                next_tick_nanos: 0,
                readings: vec![0.0; num_sensors],
                written_nanos: None,
            }
        }

        /// `cssDataInMsg.write(readings, unitTestSim.TotalSim.CurrentNanos)`.
        pub(crate) fn write(&mut self, readings: Vec<f64>) {
            self.readings = readings;
            self.written_nanos = Some(self.current_nanos);
        }

        /// `ConfigureStopTime(stop_nanos)` and `ExecuteSimulation()`: calls
        /// `step(call_nanos, readings, written_nanos)` for every tick up to the
        /// stop time, with the message contents and header upstream reads.
        pub(crate) fn execute_until(
            &mut self,
            stop_nanos: u64,
            mut step: impl FnMut(u64, &[f64], Option<u64>),
        ) {
            while self.next_tick_nanos <= stop_nanos {
                self.current_nanos = self.next_tick_nanos;
                step(self.current_nanos, &self.readings, self.written_nanos);
                self.next_tick_nanos += TASK_RATE_NANOS;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{CssUnit, configured_css_units, css_inputs, css_readings, css_written_nanos};
    use crate::SimulationContext;
    use crate::messages::{MAX_N_CSS_MEAS, Output, SunSensorMsg};

    #[test]
    fn css_readings_count_as_written_only_with_every_input_connected() {
        let context = SimulationContext {
            current_sim_nanos: 1_500_000_000,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        };
        let mut inputs = css_inputs(&[CssUnit::default(); 3]);
        assert_eq!(css_written_nanos(&inputs, &context), None);

        let outputs: Vec<Output<SunSensorMsg>> = [0.25, -0.5, 0.75]
            .into_iter()
            .map(|sensed_value| {
                Output::new(SunSensorMsg {
                    sensed_value,
                    true_value: 1.0,
                    valid: true,
                })
            })
            .collect();
        for (input, output) in inputs.iter_mut().zip(&outputs).take(2) {
            output.connect_to(input);
        }
        // One unconnected input keeps the readings unwritten; it reads zero.
        assert_eq!(css_written_nanos(&inputs, &context), None);
        assert_eq!(css_readings(&inputs), [0.25, -0.5, 0.0]);

        outputs[2].connect_to(&mut inputs[2]);
        assert_eq!(css_written_nanos(&inputs, &context), Some(1_500_000_000));
        assert_eq!(css_readings(&inputs), [0.25, -0.5, 0.75]);
        // Without sensors there is nothing to have been written.
        assert_eq!(css_written_nanos(&[], &context), None);
    }

    #[test]
    fn at_most_max_n_css_meas_configured_sensors_are_used() {
        let units: Vec<CssUnit> = (0..MAX_N_CSS_MEAS + 2)
            .map(|index| CssUnit {
                n_hat_b: Vector3::x(),
                c_bias: index as f64,
            })
            .collect();
        assert_eq!(
            configured_css_units(&units, "sunlineUKF"),
            units[..MAX_N_CSS_MEAS]
        );
        assert_eq!(configured_css_units(&units[..3], "sunlineUKF"), units[..3]);
    }
}
