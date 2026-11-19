//! Reaction-wheel speed encoder: port of upstream Basilisk
//! `simulation/deviceInterface/encoder`.
//!
//! An optical encoder counts whole clicks, `clicks_per_rotation` of them per
//! wheel revolution. Over each update interval `Δt` the wheel turns
//! `ω Δt` and the encoder registers
//!
//! ```text
//! n = trunc(ω Δt c + r),    c = clicks_per_rotation / 2π
//! ```
//!
//! clicks, carrying the fractional remainder `r` to the next interval; the
//! published speed is `n / (c Δt)`. On the first update after a reset
//! (`Δt = 0`) the true message is passed through unchanged.
//!
//! Each wheel's [`EncoderSignal`] models a failure: an encoder that is off
//! reads zero and loses its remainder, one that is stuck repeats its last
//! output. Reset puts every wheel back to nominal.
//!
//! As upstream, only `wheel_speeds_radps` is encoded: the wheel angles
//! published are those passed through on the first update after a reset.

use serde::{Deserialize, Serialize};

use crate::messages::{Input, MAX_EFF_COUNT, Output, RwSpeedMsg};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// Signal state of one wheel's encoder (upstream `SIGNAL_NOMINAL`,
/// `SIGNAL_OFF`, `SIGNAL_STUCK`).
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq, Serialize, Deserialize)]
#[repr(u8)]
pub enum EncoderSignal {
    /// Upstream `SIGNAL_NOMINAL = 0`: the encoder counts clicks.
    #[default]
    Nominal = 0,
    /// Upstream `SIGNAL_OFF = 1`: the encoder reads zero.
    Off = 1,
    /// Upstream `SIGNAL_STUCK = 2`: the encoder repeats its last reading.
    Stuck = 2,
}

/// User-settable parameters of [`Encoder`].
#[derive(Clone, Debug)]
pub struct EncoderConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `clicksPerRotation`: encoder resolution, positive.
    pub clicks_per_rotation: u32,
    /// Upstream `numRW`: number of wheels encoded, from 1 to
    /// [`MAX_EFF_COUNT`].
    pub num_rw: usize,
}

impl EncoderConfig {
    /// Upstream `Reset`'s checks: a positive resolution and wheel count.
    pub fn validate(&self) -> Result<(), String> {
        if self.clicks_per_rotation == 0 {
            return Err("number of clicks must be a positive integer".to_string());
        }
        if self.num_rw == 0 || self.num_rw > MAX_EFF_COUNT {
            return Err(format!(
                "number of reaction wheels must be between 1 and {MAX_EFF_COUNT}, got {}",
                self.num_rw
            ));
        }
        Ok(())
    }
}

/// Discretizes reaction-wheel speeds: upstream `Encoder`.
#[derive(Clone, Debug)]
pub struct Encoder {
    /// Module configuration.
    pub config: EncoderConfig,
    /// Upstream `rwSpeedInMsg`: true wheel speeds.
    pub rw_speed_in_msg: Input<RwSpeedMsg>,
    /// Upstream `rwSpeedOutMsg`: encoded wheel speeds.
    pub rw_speed_out_msg: Output<RwSpeedMsg>,
    /// Upstream `rwSignalState`: signal state of each wheel's encoder, all
    /// nominal after a reset.
    pub rw_signal_state: [EncoderSignal; MAX_EFF_COUNT],
    /// Upstream `rwSpeedConverted`: the speeds last published.
    rw_speed_converted: RwSpeedMsg,
    /// Upstream `remainingClicks`: fractional clicks carried per wheel.
    remaining_clicks: [f64; MAX_EFF_COUNT],
    /// Upstream `prevTime` \[ns\].
    prev_time_nanos: u64,
}

impl Encoder {
    /// Builds the encoder with unconnected ports.
    ///
    /// # Panics
    ///
    /// If `config` fails [`EncoderConfig::validate`].
    pub fn new(config: EncoderConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid EncoderConfig: {message}");
        }
        Self {
            config,
            rw_speed_in_msg: Input::default(),
            rw_speed_out_msg: Output::default(),
            rw_signal_state: [EncoderSignal::Nominal; MAX_EFF_COUNT],
            rw_speed_converted: RwSpeedMsg::default(),
            remaining_clicks: [0.0; MAX_EFF_COUNT],
            prev_time_nanos: 0,
        }
    }

    /// Upstream `Reset`: zeroes the output buffer and the remainders, and
    /// returns every encoder to nominal.
    fn reset_at(&mut self, call_time_nanos: u64) {
        if !self.rw_speed_in_msg.is_connected() {
            log::error!(
                "encoder '{}' rw_speed_in_msg is not connected",
                self.config.name
            );
        }
        self.prev_time_nanos = call_time_nanos;
        self.rw_speed_converted = RwSpeedMsg::default();
        self.rw_signal_state = [EncoderSignal::Nominal; MAX_EFF_COUNT];
        self.remaining_clicks = [0.0; MAX_EFF_COUNT];
    }

    /// Upstream `encode`.
    fn encode(&mut self, true_speeds: RwSpeedMsg, call_time_nanos: u64) {
        let clicks_per_radian =
            f64::from(self.config.clicks_per_rotation) / (2.0 * std::f64::consts::PI);
        let time_step_s = diff_nanos_to_seconds(call_time_nanos, self.prev_time_nanos);
        if time_step_s == 0.0 {
            self.rw_speed_converted = true_speeds;
            return;
        }
        for wheel in 0..self.config.num_rw {
            match self.rw_signal_state[wheel] {
                EncoderSignal::Nominal => {
                    let clicks =
                        true_speeds.wheel_speeds_radps[wheel] * time_step_s * clicks_per_radian
                            + self.remaining_clicks[wheel];
                    let whole_clicks = clicks.trunc();
                    self.remaining_clicks[wheel] = clicks - whole_clicks;
                    self.rw_speed_converted.wheel_speeds_radps[wheel] =
                        whole_clicks / (clicks_per_radian * time_step_s);
                }
                EncoderSignal::Off => {
                    self.rw_speed_converted.wheel_speeds_radps[wheel] = 0.0;
                    self.remaining_clicks[wheel] = 0.0;
                }
                EncoderSignal::Stuck => {}
            }
        }
    }
}

impl Module for Encoder {
    fn init(&mut self) {
        self.reset_at(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.reset_at(context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let true_speeds = self.rw_speed_in_msg.read();
        self.encode(true_speeds, context.current_sim_nanos);
        self.rw_speed_out_msg.write(self.rw_speed_converted.clone());
        self.prev_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `encoder/_UnitTest/test_encoder.py` with its truth
    //! table and accuracy.

    use hifitime::Epoch;

    use super::{Encoder, EncoderConfig, EncoderSignal};
    use crate::messages::{MAX_EFF_COUNT, Output, RwSpeedMsg};
    use crate::{Module, SimulationContext};

    const STEP_NANOS: u64 = 1_000_000_000;

    fn speeds(values: &[f64]) -> RwSpeedMsg {
        let mut message = RwSpeedMsg::default();
        message.wheel_speeds_radps[..values.len()].copy_from_slice(values);
        message
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `encoderTest`: three wheels, two clicks per rotation, 1 s
    /// steps; nominal, then off, then nominal at new speeds, then stuck.
    #[test]
    fn discretizes_and_fails_as_upstream() {
        let num_rw = 3;
        let speed_out = Output::new(speeds(&[100.0, 200.0, 300.0]));
        let mut encoder = Encoder::new(EncoderConfig {
            name: "rwSpeedsEncoder".to_string(),
            clicks_per_rotation: 2,
            num_rw,
        });
        speed_out.connect_to(&mut encoder.rw_speed_in_msg);
        encoder.init();

        let mut log = Vec::new();
        let mut step = 0;
        let mut run_step = |encoder: &mut Encoder, log: &mut Vec<RwSpeedMsg>| {
            encoder.update(&context(step * STEP_NANOS));
            log.push(encoder.rw_speed_out_msg.read());
            step += 1;
        };
        for _ in 0..3 {
            run_step(&mut encoder, &mut log);
        }
        encoder.rw_signal_state[..num_rw].fill(EncoderSignal::Off);
        run_step(&mut encoder, &mut log);
        speed_out.write(speeds(&[500.0, 400.0, 300.0]));
        encoder.rw_signal_state[..num_rw].fill(EncoderSignal::Nominal);
        run_step(&mut encoder, &mut log);
        speed_out.write(speeds(&[100.0, 200.0, 300.0]));
        encoder.rw_signal_state[..num_rw].fill(EncoderSignal::Stuck);
        run_step(&mut encoder, &mut log);

        let truth = [
            [100.0, 200.0, 300.0],
            [97.38937226, 197.92033718, 298.45130209],
            [100.53096491, 201.06192983, 298.45130209],
            [0.0, 0.0, 0.0],
            [499.51323192, 398.98226701, 298.45130209],
            [499.51323192, 398.98226701, 298.45130209],
        ];
        let accuracy = 1e-8;
        for (row, (sample, expected)) in log.iter().zip(&truth).enumerate() {
            for (wheel, (speed, truth)) in
                sample.wheel_speeds_radps.iter().zip(expected).enumerate()
            {
                assert!(
                    (speed - truth).abs() <= accuracy,
                    "row {row}, wheel {wheel}: {speed} vs {truth}"
                );
            }
        }
    }

    #[test]
    fn reset_returns_every_encoder_to_nominal_and_passes_the_truth_through() {
        let mut truth = speeds(&[10.0, -20.0]);
        truth.wheel_angles_rad[0] = 0.5;
        let speed_out = Output::new(truth.clone());
        let mut encoder = Encoder::new(EncoderConfig {
            name: "encoder".to_string(),
            clicks_per_rotation: 64,
            num_rw: 2,
        });
        speed_out.connect_to(&mut encoder.rw_speed_in_msg);
        encoder.init();
        encoder.update(&context(0));
        encoder.rw_signal_state[0] = EncoderSignal::Off;
        encoder.update(&context(STEP_NANOS));
        assert_eq!(encoder.rw_speed_out_msg.read().wheel_speeds_radps[0], 0.0);

        Module::reset(&mut encoder, &context(2 * STEP_NANOS));
        assert_eq!(
            encoder.rw_signal_state,
            [EncoderSignal::Nominal; MAX_EFF_COUNT]
        );
        encoder.update(&context(2 * STEP_NANOS));
        assert_eq!(encoder.rw_speed_out_msg.read(), truth);
    }

    #[test]
    #[should_panic(expected = "number of clicks must be a positive integer")]
    fn rejects_a_zero_resolution() {
        let _ = Encoder::new(EncoderConfig {
            name: "encoder".to_string(),
            clicks_per_rotation: 0,
            num_rw: 3,
        });
    }

    #[test]
    #[should_panic(expected = "number of reaction wheels must be between 1 and 36")]
    fn rejects_an_unset_wheel_count() {
        let _ = Encoder::new(EncoderConfig {
            name: "encoder".to_string(),
            clicks_per_rotation: 2,
            num_rw: 0,
        });
    }
}
