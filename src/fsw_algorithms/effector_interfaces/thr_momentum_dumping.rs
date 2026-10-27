//! Thruster firing sequence for reaction-wheel momentum dumping: port of
//! upstream Basilisk `thrMomentumDumping`
//! (`fswAlgorithms/effectorInterfaces/thrMomentumDumping`).
//!
//! The module reads the net impulse each thruster must deliver
//! (`thrusterImpulseInMsg`, carried in a force message) and turns it into
//! on-time commands. A thruster fires for at most one control period at a
//! time; after each firing all thrusters rest for `max_counter_value` control
//! periods so the wheels can settle the attitude again, until every requested
//! impulse is delivered. A new dumping cycle starts when the momentum-change
//! request `deltaHInMsg` is rewritten.
//!
//! # Request write time
//!
//! Upstream never reads the `deltaHInMsg` payload: it starts a new cycle when
//! the message's write time (`timeWritten`) differs from the one it last acted
//! on, and Reset takes the current write time (0 while never written) as
//! already acted on. Rust messages carry no write metadata, so here the request
//! counts as rewritten when its torque changes, at the call time the change is
//! first seen (`WriteTime`). This keeps upstream's comparison and its quirk
//! that a request with write time 0 never starts a cycle: here, one already
//! seen at the t = 0 update (a producer scheduled ahead of this module). Unlike
//! upstream, rewriting the same torque does not start a new cycle, and a
//! request written after this module's t = 0 update counts as written at the
//! next update (upstream would stamp it 0 and ignore it). Two different
//! requests first seen at the same call time (possible only when updates
//! repeat a time, e.g. after the clock restarts) count as one write, as two
//! upstream writes with the same write time do.

use nalgebra::Vector3;

use crate::messages::{
    BodyTorqueCommandMsg, Input, MAX_EFF_COUNT, Output, ThrusterArrayConfigMsg,
    ThrusterArrayForceCommandMsg, ThrusterArrayOnTimeCommandMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ThrMomentumDumping`]. Defaults are upstream's
/// zero initialization.
#[derive(Clone, Debug)]
pub struct ThrMomentumDumpingConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `maxCounterValue` [-]: control periods to rest after each
    /// firing. Must be positive; Reset warns otherwise.
    pub max_counter_value: i32,
    /// Upstream `thrMinFireTime` \[s\]: on-times shorter than this are not
    /// commanded.
    pub thr_min_fire_time_s: f64,
}

impl Default for ThrMomentumDumpingConfig {
    fn default() -> Self {
        Self {
            name: "thrMomentumDumping".to_string(),
            max_counter_value: 0,
            thr_min_fire_time_s: 0.0,
        }
    }
}

/// Momentum-dumping thruster sequencer (upstream `thrMomentumDumpingConfig`).
#[derive(Clone, Debug)]
pub struct ThrMomentumDumping {
    /// Module configuration, read at every reset and update.
    pub config: ThrMomentumDumpingConfig,
    /// Upstream `thrusterImpulseInMsg`: net impulse \[N s\] requested per
    /// thruster.
    pub thruster_impulse_in_msg: Input<ThrusterArrayForceCommandMsg>,
    /// Upstream `thrusterConfInMsg`: thruster cluster, read at reset.
    pub thruster_conf_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `deltaHInMsg`: momentum-change request whose (re)writing starts
    /// a dumping cycle; its torque is not used otherwise.
    pub delta_h_in_msg: Input<BodyTorqueCommandMsg>,
    /// Upstream `thrusterOnTimeOutMsg`.
    pub thruster_on_time_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
    thrusters: Vec<DumpingThruster>,
    /// Upstream `thrDumpingCounter`: control periods left to rest.
    thr_dumping_counter: i32,
    /// Upstream `lastDeltaHInMsgTime`.
    last_delta_h_in_msg_nanos: u64,
    delta_h_write_time: WriteTime,
    /// Upstream `priorTime`; zero means "first call after reset".
    prior_time_nanos: u64,
}

/// Per-thruster state cached at reset.
#[derive(Clone, Copy, Debug)]
struct DumpingThruster {
    /// Upstream `thrMaxForce` \[N\].
    max_thrust_n: f64,
    /// Upstream `thrOnTimeRemaining` \[s\].
    on_time_remaining_s: f64,
}

/// Stand-in for the write time (`timeWritten`) of upstream's `deltaHInMsg`:
/// the request counts as written when its torque differs from the previous
/// observation, at the call time the change is first seen. Starts at upstream's
/// never-written state (zero payload, write time 0). A torque holding NaN never
/// compares equal and so counts as rewritten on every observation.
#[derive(Clone, Debug, Default)]
struct WriteTime {
    torque_nm: Vector3<f64>,
    written_nanos: u64,
}

impl WriteTime {
    /// Records the request as read at `call_time_nanos`; returns its write time.
    fn observe(&mut self, request: &BodyTorqueCommandMsg, call_time_nanos: u64) -> u64 {
        if request.torque_request_body_nm != self.torque_nm {
            self.torque_nm = request.torque_request_body_nm;
            self.written_nanos = call_time_nanos;
        }
        self.written_nanos
    }
}

impl ThrMomentumDumping {
    /// Builds the module with unconnected inputs; the thruster cluster is
    /// read at reset.
    pub fn new(config: ThrMomentumDumpingConfig) -> Self {
        Self {
            config,
            thruster_impulse_in_msg: Input::default(),
            thruster_conf_in_msg: Input::default(),
            delta_h_in_msg: Input::default(),
            thruster_on_time_out_msg: Output::default(),
            thrusters: Vec::new(),
            thr_dumping_counter: 0,
            last_delta_h_in_msg_nanos: 0,
            delta_h_write_time: WriteTime::default(),
            prior_time_nanos: 0,
        }
    }

    /// Upstream `Reset_thrMomentumDumping`: cache the thruster limits and
    /// restart the dumping logic.
    pub fn reset(&mut self, call_time_nanos: u64) {
        self.prior_time_nanos = 0;

        if !self.thruster_conf_in_msg.is_connected() {
            log::error!("thrMomentumDumping.thruster_conf_in_msg is not connected");
        }
        if !self.delta_h_in_msg.is_connected() {
            log::error!("thrMomentumDumping.delta_h_in_msg is not connected");
        }
        if !self.thruster_impulse_in_msg.is_connected() {
            log::error!("thrMomentumDumping.thruster_impulse_in_msg is not connected");
        }

        let thruster_config = self.thruster_conf_in_msg.read();
        if thruster_config.num_thrusters > MAX_EFF_COUNT {
            log::error!(
                "thrMomentumDumping.num_thrusters exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
        }
        self.thrusters = thruster_config
            .active_thrusters()
            .iter()
            .map(|thruster| DumpingThruster {
                max_thrust_n: thruster.max_thrust_n,
                on_time_remaining_s: 0.0,
            })
            .collect();
        self.thr_dumping_counter = 0;

        // The request standing at reset counts as already acted on.
        self.last_delta_h_in_msg_nanos = self
            .delta_h_write_time
            .observe(&self.delta_h_in_msg.read(), call_time_nanos);

        if self.config.max_counter_value < 1 {
            log::warn!("thrMomentumDumping.max_counter_value must be set to a positive value");
        }
    }

    fn compute_on_times(&mut self, call_time_nanos: u64) -> ThrusterArrayOnTimeCommandMsg {
        let delta_h_written_nanos = self
            .delta_h_write_time
            .observe(&self.delta_h_in_msg.read(), call_time_nanos);
        let mut output = ThrusterArrayOnTimeCommandMsg::default();

        // Preserve upstream's zero-time sentinel: the first call after a reset
        // only records the time, and so does every call at t = 0.
        if self.prior_time_nanos != 0 {
            let mut dt_s = diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos);
            if dt_s < 0.0 {
                dt_s = 0.0;
            }
            let impulses = self.thruster_impulse_in_msg.read();
            let on_times = &mut output.on_time_request_s[..self.thrusters.len()];

            if self.last_delta_h_in_msg_nanos == delta_h_written_nanos {
                self.continue_cycle(on_times, dt_s);
            } else {
                self.last_delta_h_in_msg_nanos = delta_h_written_nanos;
                self.start_cycle(on_times, &impulses.thrust_force_n, dt_s);
            }
            self.limit_on_times(on_times, dt_s);
        }

        self.prior_time_nanos = call_time_nanos;
        output
    }

    /// New request: fire for the whole required time, capped to one period
    /// later by [`Self::limit_on_times`].
    fn start_cycle(&mut self, on_times_s: &mut [f64], impulses_ns: &[f64], dt_s: f64) {
        for ((on_time_s, thruster), impulse_ns) in on_times_s
            .iter_mut()
            .zip(&mut self.thrusters)
            .zip(impulses_ns)
        {
            thruster.on_time_remaining_s = impulse_ns / thruster.max_thrust_n;
            *on_time_s = thruster.on_time_remaining_s;
            thruster.on_time_remaining_s -= dt_s;
        }
        self.thr_dumping_counter = self.config.max_counter_value;
    }

    /// Same request: fire the remaining time once the rest periods are over.
    fn continue_cycle(&mut self, on_times_s: &mut [f64], dt_s: f64) {
        if self.thr_dumping_counter <= 0 {
            for (on_time_s, thruster) in on_times_s.iter_mut().zip(&mut self.thrusters) {
                *on_time_s = thruster.on_time_remaining_s;
                if thruster.on_time_remaining_s > 0.0 {
                    thruster.on_time_remaining_s -= dt_s;
                }
            }
            self.thr_dumping_counter = self.config.max_counter_value;
        } else {
            self.thr_dumping_counter -= 1;
        }
    }

    /// Drop on-times below the minimum firing time, clear negative remainders
    /// and cap on-times at the control period, in upstream's order.
    fn limit_on_times(&mut self, on_times_s: &mut [f64], dt_s: f64) {
        for (on_time_s, thruster) in on_times_s.iter_mut().zip(&mut self.thrusters) {
            if *on_time_s < self.config.thr_min_fire_time_s {
                *on_time_s = 0.0;
            }
            if thruster.on_time_remaining_s < 0.0 {
                thruster.on_time_remaining_s = 0.0;
            }
            if *on_time_s > dt_s {
                *on_time_s = dt_s;
            }
        }
    }
}

impl Module for ThrMomentumDumping {
    fn init(&mut self) {
        self.thruster_on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrMomentumDumping::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let output = self.compute_on_times(context.current_sim_nanos);
        self.thruster_on_time_out_msg.write(output);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_thrMomentumDumping.py` (all three parametrizations).
    //!
    //! Upstream writes the zero torque to `deltaHInMsg` at 0.5 s (and again at
    //! 3.5 s in the reset case); only the write time matters to the module.
    //! The Rust stand-in sees a write when the torque changes (see the module
    //! docs), so these tests write distinct non-zero torques instead. The
    //! truth tables are upstream's, verbatim.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        BodyTorqueCommandMsg, MAX_EFF_COUNT, Output, ThrusterArrayConfigMsg,
        ThrusterArrayForceCommandMsg, ThrusterConfigMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{ThrMomentumDumping, ThrMomentumDumpingConfig};

    const NUM_THRUSTERS: usize = 8;
    const ACCURACY: f64 = 1e-12;

    const RCS_LOCATIONS_M: [[f64; 3]; NUM_THRUSTERS] = [
        [-0.86360, -0.82550, 1.79070],
        [-0.82550, -0.86360, 1.79070],
        [0.82550, 0.86360, 1.79070],
        [0.86360, 0.82550, 1.79070],
        [-0.86360, -0.82550, -1.79070],
        [-0.82550, -0.86360, -1.79070],
        [0.82550, 0.86360, -1.79070],
        [0.86360, 0.82550, -1.79070],
    ];
    const RCS_DIRECTIONS: [[f64; 3]; NUM_THRUSTERS] = [
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [-1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
    ];
    const IMPULSES_NS: [f64; NUM_THRUSTERS] = [1.2, 0.2, 0.0, 1.6, 1.2, 0.2, 1.6, 0.0];

    const TRUTH_RESET: [[f64; NUM_THRUSTERS]; 12] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.1, 0.0, 0.5, 0.5, 0.1, 0.5, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.1, 0.0, 0.5, 0.5, 0.1, 0.5, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0],
    ];
    const TRUTH_LARGE_MIN_FIRE_TIME: [[f64; NUM_THRUSTERS]; 7] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.0, 0.0, 0.5, 0.5, 0.0, 0.5, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ];
    const TRUTH_DEFAULT: [[f64; NUM_THRUSTERS]; 7] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.1, 0.0, 0.5, 0.5, 0.1, 0.5, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.1, 0.0, 0.0, 0.3, 0.1, 0.0, 0.3, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ];

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn seconds(value_s: f64) -> u64 {
        (value_s * 1e9).round() as u64
    }

    fn thruster_cluster() -> ThrusterArrayConfigMsg {
        let thrusters: Vec<_> = RCS_LOCATIONS_M
            .iter()
            .zip(RCS_DIRECTIONS)
            .map(|(location, direction)| ThrusterConfigMsg {
                position_thrust_b_m: Vector3::from(*location),
                t_hat_thrust_b: Vector3::from(direction),
                max_thrust_n: 2.0,
                swirl_torque_nm: 0.0,
            })
            .collect();
        ThrusterArrayConfigMsg::from_active(&thrusters)
    }

    fn request(torque_x_nm: f64) -> BodyTorqueCommandMsg {
        BodyTorqueCommandMsg {
            torque_request_body_nm: Vector3::new(torque_x_nm, 0.0, 0.0),
        }
    }

    /// A module wired to the upstream thruster cluster and impulse request,
    /// initialized, with the output driving its `deltaHInMsg`.
    fn wired_module(
        config: ThrMomentumDumpingConfig,
        initial_request: BodyTorqueCommandMsg,
    ) -> (ThrMomentumDumping, Output<BodyTorqueCommandMsg>) {
        let delta_h_out = Output::new(initial_request);
        let mut module = ThrMomentumDumping::new(config);
        Output::new(thruster_cluster()).connect_to(&mut module.thruster_conf_in_msg);
        Output::new(ThrusterArrayForceCommandMsg::from_active(&IMPULSES_NS))
            .connect_to(&mut module.thruster_impulse_in_msg);
        delta_h_out.connect_to(&mut module.delta_h_in_msg);
        module.init();
        (module, delta_h_out)
    }

    fn fires_nothing(module: &ThrMomentumDumping) -> bool {
        module
            .thruster_on_time_out_msg
            .read()
            .on_time_request_s
            .iter()
            .all(|on_time| *on_time == 0.0)
    }

    /// The upstream test sequence; returns the on-times recorded every 0.5 s.
    fn run_upstream_sequence(reset_check: bool, large_min_fire_time: bool) -> Vec<[f64; 8]> {
        let (mut module, delta_h_out) = wired_module(
            ThrMomentumDumpingConfig {
                max_counter_value: 2,
                thr_min_fire_time_s: if large_min_fire_time { 0.200 } else { 0.020 },
                ..Default::default()
            },
            BodyTorqueCommandMsg::default(),
        );

        let mut samples = Vec::new();
        let mut run = |module: &mut ThrMomentumDumping, times_s: &[f64]| {
            for &time_s in times_s {
                module.update(&context(seconds(time_s)));
                let output = module.thruster_on_time_out_msg.read();
                let mut row = [0.0; NUM_THRUSTERS];
                row.copy_from_slice(&output.on_time_request_s[..NUM_THRUSTERS]);
                samples.push(row);
            }
        };

        run(&mut module, &[0.0, 0.5]);
        delta_h_out.write(request(1.0));
        run(&mut module, &[1.0, 1.5, 2.0, 2.5, 3.0]);
        if reset_check {
            module.reset(seconds(3.0));
            run(&mut module, &[3.5]);
            delta_h_out.write(request(2.0));
            run(&mut module, &[4.0, 4.5, 5.0, 5.5]);
        }
        samples
    }

    fn assert_rows_match(actual: &[[f64; 8]], truth: &[[f64; 8]], case: &str) {
        assert_eq!(actual.len(), truth.len(), "{case}: number of samples");
        for (row, (actual_row, truth_row)) in actual.iter().zip(truth).enumerate() {
            for (thruster, (actual, truth)) in actual_row.iter().zip(truth_row).enumerate() {
                assert!(
                    (actual - truth).abs() <= ACCURACY,
                    "{case}: sample {row}, thruster {thruster}: expected {truth}, got {actual}"
                );
            }
        }
    }

    #[test]
    fn matches_upstream_thr_momentum_dumping_unit_test() {
        for (reset_check, large_min_fire_time) in [(false, false), (true, false), (false, true)] {
            let truth: &[[f64; 8]] = if reset_check {
                &TRUTH_RESET
            } else if large_min_fire_time {
                &TRUTH_LARGE_MIN_FIRE_TIME
            } else {
                &TRUTH_DEFAULT
            };
            let samples = run_upstream_sequence(reset_check, large_min_fire_time);
            assert_rows_match(
                &samples,
                truth,
                &format!("resetCheck={reset_check}, largeMinFireTime={large_min_fire_time}"),
            );
        }
    }

    #[test]
    fn fires_every_period_without_rest_until_the_impulse_is_delivered() {
        // 2.4 N s at 2 N needs 1.2 s of firing: 0.5 s, 0.5 s, then the 0.2 s
        // remainder, with no rest periods for maxCounterValue = 0.
        let (mut module, delta_h_out) = wired_module(
            ThrMomentumDumpingConfig {
                max_counter_value: 0,
                thr_min_fire_time_s: 0.02,
                ..Default::default()
            },
            BodyTorqueCommandMsg::default(),
        );
        module.update(&context(seconds(0.5)));
        let impulse = Output::new(ThrusterArrayForceCommandMsg::from_active(&[2.4]));
        impulse.connect_to(&mut module.thruster_impulse_in_msg);
        delta_h_out.write(request(1.0));

        for (time_s, expected_s) in [(1.0, 0.5), (1.5, 0.5), (2.0, 0.2), (2.5, 0.0), (3.0, 0.0)] {
            module.update(&context(seconds(time_s)));
            let on_times = module.thruster_on_time_out_msg.read().on_time_request_s;
            assert!(
                (on_times[0] - expected_s).abs() <= ACCURACY,
                "{time_s} s: expected {expected_s}, got {}",
                on_times[0]
            );
            assert!(on_times[1..].iter().all(|on_time| *on_time == 0.0));
        }
    }

    #[test]
    fn unchanged_request_after_reset_does_not_restart_the_cycle() {
        let (mut module, _delta_h_out) = wired_module(
            ThrMomentumDumpingConfig {
                max_counter_value: 2,
                thr_min_fire_time_s: 0.02,
                ..Default::default()
            },
            request(1.0),
        );
        for time_s in [0.5, 1.0, 1.5, 2.0, 2.5] {
            module.update(&context(seconds(time_s)));
            assert!(
                fires_nothing(&module),
                "the request standing at reset must not fire at {time_s} s"
            );
        }
    }

    #[test]
    fn request_seen_at_time_zero_is_ignored_like_upstream() {
        // Upstream compares write times, and a request written at t = 0 has
        // the never-written write time 0.
        let (mut module, delta_h_out) = wired_module(
            ThrMomentumDumpingConfig {
                max_counter_value: 1,
                ..Default::default()
            },
            BodyTorqueCommandMsg::default(),
        );
        delta_h_out.write(request(1.0));
        for time_s in [0.0, 0.5, 1.0, 1.5, 2.0] {
            module.update(&context(seconds(time_s)));
            assert!(fires_nothing(&module), "fired at {time_s} s");
        }

        // A later write does start a cycle.
        delta_h_out.write(request(2.0));
        module.update(&context(seconds(2.5)));
        let on_times = module.thruster_on_time_out_msg.read().on_time_request_s;
        assert_eq!(on_times[..4], [0.5, 0.1, 0.0, 0.5]);
    }

    #[test]
    fn going_back_in_time_commands_nothing() {
        let (mut module, delta_h_out) = wired_module(
            ThrMomentumDumpingConfig {
                max_counter_value: 2,
                ..Default::default()
            },
            BodyTorqueCommandMsg::default(),
        );
        module.update(&context(seconds(1.0)));
        delta_h_out.write(request(1.0));
        // dt is negative and clamped to zero, so every on-time is capped at 0.
        module.update(&context(seconds(0.5)));
        assert_eq!(
            module.thruster_on_time_out_msg.read().on_time_request_s,
            [0.0; MAX_EFF_COUNT]
        );
    }
}
