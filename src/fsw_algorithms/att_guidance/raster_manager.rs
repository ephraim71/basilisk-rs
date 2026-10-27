//! Timed sequence of attitude commands: port of upstream Basilisk
//! `rasterManager`.
//!
//! The module steps through a list of raster segments, each a commanded
//! attitude set, its rate and a duration, and publishes the active segment as
//! an attitude state message. It is meant to feed the desired-attitude input
//! of a dynamic reference module such as `eulerRotation` (3-2-1 Euler angles
//! and rates) or `mrpRotation` (an MRP set and a body rate).
//!
//! A segment is active from the update that starts it until an update finds
//! its duration elapsed. That update advances to the next segment but still
//! publishes the previous command, and the next segment starts timing on the
//! following update, as upstream. After the last segment the sequence starts
//! over with the first.

use nalgebra::Vector3;

use crate::messages::{AttitudeStateMsg, Output};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// Upstream `MAX_RASTER_SET`: the most raster segments the module uses.
pub const MAX_RASTER_SET: usize = 36;

/// One commanded segment of a raster sequence.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RasterSegment {
    /// Upstream `scanningAngles[3i..3i+3]`: commanded attitude set, in the
    /// coordinates of the consumer (e.g. 3-2-1 Euler angles \[rad\]).
    pub scanning_angles: Vector3<f64>,
    /// Upstream `scanningRates[3i..3i+3]`: commanded attitude rate (e.g. 3-2-1
    /// Euler angle rates \[rad/s\]).
    pub scanning_rates: Vector3<f64>,
    /// Upstream `rasterTimes[i]` \[s\]: how long the segment is commanded.
    pub raster_time_s: f64,
}

/// User-settable parameters of [`RasterManager`]. `Default` has no segments,
/// like upstream's zero-initialized configuration, and must be filled in.
#[derive(Clone, Debug)]
pub struct RasterManagerConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `scanningAngles`, `scanningRates` and `rasterTimes`, one
    /// entry per segment; the length is upstream `numRasters`. At most
    /// [`MAX_RASTER_SET`] segments are used.
    pub rasters: Vec<RasterSegment>,
}

impl Default for RasterManagerConfig {
    fn default() -> Self {
        Self {
            name: "rasterManager".to_string(),
            rasters: Vec::new(),
        }
    }
}

/// Sequencer of timed attitude commands: upstream `rasterManager`.
#[derive(Clone, Debug)]
pub struct RasterManager {
    /// User-settable parameters, read on every update.
    pub config: RasterManagerConfig,
    /// Upstream `attStateOutMsg`: commanded attitude set and rate.
    pub att_state_out_msg: Output<AttitudeStateMsg>,
    /// Upstream `scanSelector`: index of the current segment. It is reduced
    /// modulo the segment count at the start of each update.
    scan_selector: usize,
    /// Upstream `mnvrStartTime` \[ns\] while upstream `mnvrActive` is set.
    maneuver_start_nanos: Option<u64>,
    /// Upstream `attOutSet`: the last published command, kept across resets.
    att_out_set: AttitudeStateMsg,
}

impl RasterManager {
    /// Creates the module with `config` and an unconnected output.
    pub fn new(config: RasterManagerConfig) -> Self {
        Self {
            config,
            att_state_out_msg: Output::default(),
            scan_selector: 0,
            maneuver_start_nanos: None,
            att_out_set: AttitudeStateMsg::default(),
        }
    }

    /// Upstream `Reset_rasterManager`: restart the sequence at the first
    /// segment. The last command stays published until an update replaces it.
    fn reset_sequence(&mut self) {
        // Not upstream, which reads past its fixed arrays for more than
        // `MAX_RASTER_SET` segments; the port uses the first ones and says so.
        if self.config.rasters.len() > MAX_RASTER_SET {
            log::warn!(
                "rasterManager has {} raster segments; only the first {MAX_RASTER_SET} are used",
                self.config.rasters.len()
            );
        }
        self.maneuver_start_nanos = None;
        self.scan_selector = 0;
    }

    /// The segments in use: upstream reads at most `MAX_RASTER_SET` entries.
    fn active_rasters(&self) -> &[RasterSegment] {
        let count = self.config.rasters.len().min(MAX_RASTER_SET);
        &self.config.rasters[..count]
    }

    /// Upstream `Update_rasterManager` without the message write: the
    /// command to publish at `call_time_nanos`, or `None` when no segment is
    /// configured.
    fn advance(&mut self, call_time_nanos: u64) -> Option<AttitudeStateMsg> {
        let num_rasters = self.active_rasters().len();
        if num_rasters == 0 {
            // Upstream evaluates `scanSelector % numRasters` here and divides
            // by zero; the port reports it and publishes nothing.
            log::error!("rasterManager has no raster segments configured");
            return None;
        }
        self.scan_selector %= num_rasters;
        let segment = self.active_rasters()[self.scan_selector];
        let start_nanos = *self.maneuver_start_nanos.get_or_insert(call_time_nanos);

        if diff_nanos_to_seconds(call_time_nanos, start_nanos) < segment.raster_time_s {
            self.att_out_set = AttitudeStateMsg {
                state: segment.scanning_angles,
                rate: segment.scanning_rates,
            };
        } else {
            self.maneuver_start_nanos = None;
            self.scan_selector += 1;
            let AttitudeStateMsg { state, rate } = &self.att_out_set;
            log::info!(
                "Raster: {}. AngleSet = [{:.6}, {:.6}, {:.6}], RateSet = [{:.6}, {:.6}, {:.6}] ",
                self.scan_selector,
                state[0],
                state[1],
                state[2],
                rate[0],
                rate[1],
                rate[2]
            );
        }
        Some(self.att_out_set.clone())
    }
}

impl Module for RasterManager {
    fn init(&mut self) {
        self.att_state_out_msg.write(AttitudeStateMsg::default());
        self.reset_sequence();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_sequence();
    }

    fn update(&mut self, context: &SimulationContext) {
        if let Some(command) = self.advance(context.current_sim_nanos) {
            self.att_state_out_msg.write(command);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Upstream has no unit test for `rasterManager`; these tests follow the
    //! algorithm of `rasterManager.c`, with hand-derived truth.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeReferenceMsg, AttitudeStateMsg, Output};
    use crate::{Module, SimulationContext};

    use super::{MAX_RASTER_SET, RasterManager, RasterManagerConfig, RasterSegment};
    use crate::fsw_algorithms::att_guidance::euler_rotation::{EulerRotation, EulerRotationConfig};

    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn segment(index: f64, raster_time_s: f64) -> RasterSegment {
        RasterSegment {
            scanning_angles: Vector3::new(index, 0.0, -index),
            scanning_rates: Vector3::new(0.0, 0.1 * index, 0.0),
            raster_time_s,
        }
    }

    fn command(segment: &RasterSegment) -> AttitudeStateMsg {
        AttitudeStateMsg {
            state: segment.scanning_angles,
            rate: segment.scanning_rates,
        }
    }

    /// Two segments of 1 s and 0.5 s at a 0.5 s task rate. Each segment ends
    /// with one update that still publishes it, and the next segment starts
    /// on the update after that, so the sequence repeats every 5 updates.
    #[test]
    fn cycles_through_the_segments_with_one_update_of_overlap() {
        let segments = [segment(1.0, 1.0), segment(2.0, 0.5)];
        let mut module = RasterManager::new(RasterManagerConfig::default());
        module.config.rasters = segments.to_vec();
        module.init();

        // Index of the segment published by each update.
        let expected = [0, 0, 0, 1, 1, 0, 0, 0, 1, 1];
        for (tick, &published) in expected.iter().enumerate() {
            module.update(&context(tick as u64 * TASK_NANOS));
            assert_eq!(
                module.att_state_out_msg.read(),
                command(&segments[published]),
                "tick {tick}"
            );
        }
    }

    /// Reset restarts the sequence at the first segment, timing from the
    /// next update.
    #[test]
    fn reset_restarts_the_sequence() {
        let segments = [segment(1.0, 0.5), segment(2.0, 10.0)];
        let mut module = RasterManager::new(RasterManagerConfig {
            rasters: segments.to_vec(),
            ..Default::default()
        });
        module.init();
        for tick in 0..=3 {
            module.update(&context(tick * TASK_NANOS));
        }
        assert_eq!(module.att_state_out_msg.read(), command(&segments[1]));

        Module::reset(&mut module, &context(3 * TASK_NANOS));
        // The last command stays published until the next update.
        assert_eq!(module.att_state_out_msg.read(), command(&segments[1]));
        module.update(&context(4 * TASK_NANOS));
        assert_eq!(module.att_state_out_msg.read(), command(&segments[0]));
        assert_eq!(module.scan_selector, 0);
        module.update(&context(5 * TASK_NANOS));
        assert_eq!(module.scan_selector, 1);
    }

    /// A zero-duration first segment is never published: its first update
    /// already finds it elapsed and publishes the zero initial command.
    #[test]
    fn a_zero_duration_segment_is_skipped_with_the_previous_command() {
        let segments = [segment(1.0, 0.0), segment(2.0, 5.0)];
        let mut module = RasterManager::new(RasterManagerConfig {
            rasters: segments.to_vec(),
            ..Default::default()
        });
        module.init();
        module.update(&context(0));
        assert_eq!(module.att_state_out_msg.read(), AttitudeStateMsg::default());
        module.update(&context(TASK_NANOS));
        assert_eq!(module.att_state_out_msg.read(), command(&segments[1]));
    }

    /// Without segments upstream divides by zero; the port logs an error and
    /// publishes nothing.
    #[test]
    fn publishes_nothing_without_segments() {
        let mut module = RasterManager::new(RasterManagerConfig::default());
        module.init();
        module.update(&context(0));
        assert_eq!(module.att_state_out_msg.read(), AttitudeStateMsg::default());
    }

    /// Segments beyond `MAX_RASTER_SET` are not used: the selector wraps
    /// after the 36th segment.
    #[test]
    fn uses_at_most_max_raster_set_segments() {
        let segments: Vec<_> = (0..MAX_RASTER_SET + 2)
            .map(|index| segment(index as f64, 0.0))
            .collect();
        let mut module = RasterManager::new(RasterManagerConfig {
            rasters: segments,
            ..Default::default()
        });
        module.init();
        for tick in 0..MAX_RASTER_SET as u64 {
            module.update(&context(tick * TASK_NANOS));
        }
        assert_eq!(module.scan_selector, MAX_RASTER_SET);
        module.update(&context(MAX_RASTER_SET as u64 * TASK_NANOS));
        assert_eq!(module.scan_selector, 1);
    }

    /// The raster commands drive `eulerRotation`: each new segment replaces
    /// its Euler angle set and rates, and a segment repeated on later updates
    /// is integrated instead.
    #[test]
    fn drives_the_euler_rotation_desired_attitude() {
        let segments = [segment(0.2, 1.0), segment(0.4, 1.0)];
        let mut raster = RasterManager::new(RasterManagerConfig {
            rasters: segments.to_vec(),
            ..Default::default()
        });
        let reference = Output::new(AttitudeReferenceMsg::default());
        let mut rotation = EulerRotation::new(EulerRotationConfig::default());
        reference.connect_to(&mut rotation.att_ref_in_msg);
        raster
            .att_state_out_msg
            .connect_to(&mut rotation.desired_att_in_msg);
        raster.init();
        rotation.init();
        let mut step = |tick: u64| {
            raster.update(&context(tick * TASK_NANOS));
            rotation.update(&context(tick * TASK_NANOS));
            rotation.angle_set_rad
        };

        // The first segment is published at t = 0, 0.5 and 1 s. The step at
        // t = 0.5 s is zero (eulerRotation's zero-time sentinel), so only the
        // 0.5 s step at t = 1 s integrates it.
        let [first, second] = segments;
        step(0);
        step(1);
        assert_eq!(step(2), first.scanning_angles + 0.5 * first.scanning_rates);
        // The second segment starts at t = 1.5 s.
        assert_eq!(
            step(3),
            second.scanning_angles + 0.5 * second.scanning_rates
        );
    }
}
