//! Thruster firing with remainder tracking: port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/thrFiringRemainder`.
//!
//! The module turns commanded thruster forces into thruster on-times. A
//! request shorter than the minimum fire time is not fired; it is kept as a
//! remainder (in units of minimum pulses) and added to the next request, so
//! that small requests accumulate until they reach a full minimum pulse. A
//! request at or beyond the control period is over-saturated to 1.1 control
//! periods. Off-pulsing (thrusters nominally on, such as DV thrusters) is
//! handled by adding each thruster's maximum thrust to the negative force
//! request.
//!
//! The on/off-pulsing mode [`BaseThrustState`], the control-period clock and
//! the force-to-on-time conversion are shared with
//! [`super::thr_firing_schmitt`].

use crate::messages::{
    Input, Output, ThrusterArrayConfigMsg, ThrusterArrayForceCommandMsg,
    ThrusterArrayOnTimeCommandMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::{Module, SimulationContext};

/// Nominal thruster state (upstream `baseThrustState`: 0 on-pulsing, 1
/// off-pulsing).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum BaseThrustState {
    /// Thrusters are nominally off and pulsed on (RCS thrusters, upstream 0).
    #[default]
    OnPulsing,
    /// Thrusters are nominally on and pulsed off (DV thrusters, upstream 1);
    /// force requests are negative offsets from the maximum thrust.
    OffPulsing,
}

impl BaseThrustState {
    /// On-time requested for every thruster on the first update after a reset,
    /// when no control period is known yet: upstream `baseThrustState * 2.0`
    /// seconds, i.e. all thrusters off or all on for longer than any period.
    fn first_call_on_time_s(self) -> f64 {
        match self {
            Self::OnPulsing => 0.0,
            Self::OffPulsing => 2.0,
        }
    }

    /// Thruster on-times for the first update after a reset.
    pub(crate) fn first_call_on_times(self, num_thrusters: usize) -> ThrusterArrayOnTimeCommandMsg {
        ThrusterArrayOnTimeCommandMsg::from_active(&vec![
            self.first_call_on_time_s();
            num_thrusters
        ])
    }
}

/// Control-period bookkeeping shared by the thruster firing modules.
///
/// Upstream stores the previous call time and treats a stored value of zero as
/// "no previous call". The sentinel is kept as is rather than turned into an
/// `Option`: an update at t = 0 stores 0, so the following update is again
/// treated as a first call.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct ControlPeriodClock {
    prev_call_time_nanos: u64,
}

impl ControlPeriodClock {
    pub(crate) fn reset(&mut self) {
        self.prev_call_time_nanos = 0;
    }

    /// Records the call at `call_time_nanos` and returns the time since the
    /// previous call \[s\], or `None` on a first call.
    pub(crate) fn tick(&mut self, call_time_nanos: u64) -> Option<f64> {
        let prev_call_time_nanos =
            std::mem::replace(&mut self.prev_call_time_nanos, call_time_nanos);
        (prev_call_time_nanos != 0)
            .then(|| diff_nanos_to_seconds(call_time_nanos, prev_call_time_nanos))
    }
}

/// On-time \[s\] that implements `force_n` with a thruster of `max_thrust_n`
/// over `control_period_s`, before any minimum-pulse logic.
///
/// Off-pulsing requests are offsets from the maximum thrust, and negative
/// forces are clipped to zero (an explicit comparison, as upstream, so that a
/// NaN request stays NaN).
pub(crate) fn requested_on_time_s(
    force_n: f64,
    max_thrust_n: f64,
    base_thrust_state: BaseThrustState,
    control_period_s: f64,
) -> f64 {
    let mut force_n = match base_thrust_state {
        BaseThrustState::OnPulsing => force_n,
        BaseThrustState::OffPulsing => force_n + max_thrust_n,
    };
    if force_n < 0.0 {
        force_n = 0.0;
    }
    force_n / max_thrust_n * control_period_s
}

/// User-settable parameters of [`ThrFiringRemainder`].
#[derive(Clone, Debug)]
pub struct ThrFiringRemainderConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `thrMinFireTime` \[s\]: minimum thruster on-time (default 0).
    pub thr_min_fire_time_s: f64,
    /// Upstream `baseThrustState`: on- or off-pulsing (default on-pulsing).
    pub base_thrust_state: BaseThrustState,
}

impl Default for ThrFiringRemainderConfig {
    fn default() -> Self {
        Self {
            name: "thrFiringRemainder".to_string(),
            thr_min_fire_time_s: 0.0,
            base_thrust_state: BaseThrustState::OnPulsing,
        }
    }
}

/// Maps thruster force commands to on-times with remainder tracking (upstream
/// `thrFiringRemainder`).
#[derive(Clone, Debug)]
pub struct ThrFiringRemainder {
    /// Module configuration.
    pub config: ThrFiringRemainderConfig,
    /// Upstream `thrForceInMsg`: commanded thruster forces.
    pub thr_force_in_msg: Input<ThrusterArrayForceCommandMsg>,
    /// Upstream `thrConfInMsg`: thruster cluster configuration, read at reset.
    pub thr_conf_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `onTimeOutMsg`: commanded thruster on-times.
    pub on_time_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
    /// Upstream `maxThrust` \[N\], one entry per installed thruster.
    max_thrust_n: Vec<f64>,
    /// Upstream `pulseRemainder` \[-\]: unimplemented request per thruster, in
    /// minimum pulses.
    pulse_remainder: Vec<f64>,
    clock: ControlPeriodClock,
}

impl ThrFiringRemainder {
    /// Builds the module with unconnected ports and no thrusters; the cluster
    /// is read at reset.
    pub fn new(config: ThrFiringRemainderConfig) -> Self {
        Self {
            config,
            thr_force_in_msg: Input::default(),
            thr_conf_in_msg: Input::default(),
            on_time_out_msg: Output::default(),
            max_thrust_n: Vec::new(),
            pulse_remainder: Vec::new(),
            clock: ControlPeriodClock::default(),
        }
    }

    /// Upstream `Reset_thrFiringRemainder`: reads the thruster configuration,
    /// clears the remainders and the control-period history, and publishes
    /// zero on-times so that a disabled module leaves the thrusters off.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        self.clock.reset();

        if !self.thr_conf_in_msg.is_connected() {
            log::error!("thrFiringRemainder.thr_conf_in_msg is not connected");
        }
        if !self.thr_force_in_msg.is_connected() {
            log::error!("thrFiringRemainder.thr_force_in_msg is not connected");
        }

        self.max_thrust_n = self
            .thr_conf_in_msg
            .read()
            .active_thrusters()
            .iter()
            .map(|thruster| thruster.max_thrust_n)
            .collect();
        self.pulse_remainder = vec![0.0; self.max_thrust_n.len()];

        self.on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
    }

    fn compute_on_times(&mut self, call_time_nanos: u64) -> ThrusterArrayOnTimeCommandMsg {
        let base_thrust_state = self.config.base_thrust_state;
        let Some(control_period_s) = self.clock.tick(call_time_nanos) else {
            return base_thrust_state.first_call_on_times(self.max_thrust_n.len());
        };

        let min_fire_time_s = self.config.thr_min_fire_time_s;
        let forces = self.thr_force_in_msg.read();
        let on_times: Vec<f64> = forces
            .thrust_force_n
            .iter()
            .zip(&self.max_thrust_n)
            .zip(&mut self.pulse_remainder)
            .map(|((&force_n, &max_thrust_n), remainder)| {
                let on_time_s =
                    requested_on_time_s(force_n, max_thrust_n, base_thrust_state, control_period_s)
                        + *remainder * min_fire_time_s;
                *remainder = 0.0;
                if on_time_s < min_fire_time_s {
                    *remainder = on_time_s / min_fire_time_s;
                    0.0
                } else if on_time_s >= control_period_s {
                    // Over-saturate so the thruster stays on for the whole period.
                    1.1 * control_period_s
                } else {
                    on_time_s
                }
            })
            .collect();
        ThrusterArrayOnTimeCommandMsg::from_active(&on_times)
    }
}

impl Module for ThrFiringRemainder {
    fn init(&mut self) {
        self.on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrFiringRemainder::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let on_times = self.compute_on_times(context.current_sim_nanos);
        self.on_time_out_msg.write(on_times);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        MAX_EFF_COUNT, Output, ThrusterArrayConfigMsg, ThrusterArrayForceCommandMsg,
        ThrusterConfigMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{BaseThrustState, ThrFiringRemainder, ThrFiringRemainderConfig};

    const TASK_PERIOD_NANOS: u64 = 500_000_000;

    /// The pytest's `fswSetupThrusters` cluster: eight RCS thrusters of 0.5 N.
    fn upstream_rcs_cluster() -> ThrusterArrayConfigMsg {
        let locations = [
            [-0.86360, -0.82550, 1.79070],
            [-0.82550, -0.86360, 1.79070],
            [0.82550, 0.86360, 1.79070],
            [0.86360, 0.82550, 1.79070],
            [-0.86360, -0.82550, -1.79070],
            [-0.82550, -0.86360, -1.79070],
            [0.82550, 0.86360, -1.79070],
            [0.86360, 0.82550, -1.79070],
        ];
        let directions = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
        ];
        let thrusters: Vec<_> = locations
            .iter()
            .zip(&directions)
            .map(|(location, direction)| ThrusterConfigMsg {
                position_thrust_b_m: Vector3::from(*location),
                t_hat_thrust_b: Vector3::from(*direction),
                max_thrust_n: 0.5,
                swirl_torque_nm: 0.0,
            })
            .collect();
        ThrusterArrayConfigMsg::from_active(&thrusters)
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Ticks of the pytest's 0.5 s task from `start` to `stop`, inclusive.
    fn ticks(start_nanos: u64, stop_nanos: u64) -> impl Iterator<Item = u64> {
        (start_nanos..=stop_nanos).step_by(TASK_PERIOD_NANOS as usize)
    }

    /// Updates at `nanos` and returns the on-times of the eight thrusters, as
    /// the pytest's recorder logs them.
    fn record(module: &mut ThrFiringRemainder, nanos: u64) -> [f64; 8] {
        module.update(&context(nanos));
        let output = module.on_time_out_msg.read().on_time_request_s;
        std::array::from_fn(|index| output[index])
    }

    fn assert_rows_close(actual: &[[f64; 8]], expected: &[[f64; 8]], accuracy: f64) {
        assert_eq!(actual.len(), expected.len());
        for (row, (actual_row, expected_row)) in actual.iter().zip(expected).enumerate() {
            for (column, (a, e)) in actual_row.iter().zip(expected_row).enumerate() {
                assert!(
                    (a - e).abs() <= accuracy,
                    "row {row}, thruster {column}: expected {e}, got {a}"
                );
            }
        }
    }

    const ON_PULSING_TRUTH: [[f64; 8]; 7] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.55, 0.2, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
    ];

    const OFF_PULSING_TRUTH: [[f64; 8]; 7] = [
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.24],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
    ];

    const ON_PULSING_RESET_TRUTH: [[f64; 8]; 12] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.55, 0.2, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.49],
        [0.55, 0.2, 0.2, 0.3, 0.38, 0.0, 0.2, 0.49],
    ];

    const OFF_PULSING_RESET_TRUTH: [[f64; 8]; 12] = [
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.24],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0],
        [0.0, 0.55, 0.4, 0.3, 0.2, 0.32, 0.22, 0.24],
    ];

    /// Upstream `test_thrFiringRemainder`, all four `(resetCheck, dvOn)`
    /// parametrizations: a 0.5 s task run to 3 s (and, with `resetCheck`, a
    /// Reset at 3 s and a run to 5.5 s), then a re-initialization with a
    /// nonzero request and a final Reset that must zero the output. Upstream's
    /// `compareArray` only checks the first three thrusters of each sample;
    /// all eight are checked here.
    #[test]
    fn matches_upstream_thr_firing_remainder_unit_test() {
        for (reset_check, dv_on) in [(false, false), (true, false), (false, true), (true, true)] {
            let forces = if dv_on {
                [-0.5, 0.0, -0.1, -0.2, -0.3, -0.34, -0.39, -0.44]
            } else {
                [0.5, 0.05, 0.1, 0.15, 0.19, 0.0, 0.2, 0.49]
            };
            let cluster = Output::new(upstream_rcs_cluster());
            let force_cmd = Output::new(ThrusterArrayForceCommandMsg::from_active(&forces));
            // Configured after construction, as the pytest does.
            let mut module = ThrFiringRemainder::new(ThrFiringRemainderConfig::default());
            module.config.thr_min_fire_time_s = 0.2;
            module.config.base_thrust_state = if dv_on {
                BaseThrustState::OffPulsing
            } else {
                BaseThrustState::OnPulsing
            };
            cluster.connect_to(&mut module.thr_conf_in_msg);
            force_cmd.connect_to(&mut module.thr_force_in_msg);
            module.init();

            let mut stop_nanos = 3_000_000_000;
            let mut samples: Vec<[f64; 8]> = ticks(0, stop_nanos)
                .map(|nanos| record(&mut module, nanos))
                .collect();
            if reset_check {
                module.reset(3_000_000_000);
                stop_nanos = 5_500_000_000;
                samples.extend(
                    ticks(3_500_000_000, stop_nanos).map(|nanos| record(&mut module, nanos)),
                );
            }

            let expected: &[[f64; 8]] = match (reset_check, dv_on) {
                (false, false) => &ON_PULSING_TRUTH,
                (false, true) => &OFF_PULSING_TRUTH,
                (true, false) => &ON_PULSING_RESET_TRUTH,
                (true, true) => &OFF_PULSING_RESET_TRUTH,
            };
            assert_rows_close(&samples, expected, 1e-12);

            force_cmd.write(ThrusterArrayForceCommandMsg::from_active(&[
                0.5, 0.05, 0.1, 0.15, 0.19, 0.0, 0.2, 0.49,
            ]));
            module.init();
            ticks(0, stop_nanos).for_each(|nanos| module.update(&context(nanos)));
            module.reset(0);
            // `compareVector`: the norm of the difference.
            let output = module.on_time_out_msg.read().on_time_request_s;
            assert_eq!(output.len(), MAX_EFF_COUNT);
            assert!(output.iter().map(|t| t * t).sum::<f64>().sqrt() <= 1e-3);
        }
    }

    #[test]
    fn accumulates_small_requests_until_a_minimum_pulse_is_reached() {
        let cluster = Output::new(ThrusterArrayConfigMsg::from_active(&[ThrusterConfigMsg {
            max_thrust_n: 1.0,
            ..Default::default()
        }]));
        // 0.12 N of a 1 N thruster over a 1 s period is 0.12 s, below the
        // 0.25 s minimum pulse.
        let force_cmd = Output::new(ThrusterArrayForceCommandMsg::from_active(&[0.12]));
        let mut module = ThrFiringRemainder::new(ThrFiringRemainderConfig {
            thr_min_fire_time_s: 0.25,
            ..Default::default()
        });
        cluster.connect_to(&mut module.thr_conf_in_msg);
        force_cmd.connect_to(&mut module.thr_force_in_msg);
        module.init();

        let on_times = [1, 2, 3, 4, 5].map(|seconds| {
            module.update(&context(seconds * 1_000_000_000));
            module.on_time_out_msg.read().on_time_request_s[0]
        });
        // First call; 0.12 s and 0.24 s are kept as remainders; 0.36 s fires;
        // the next 0.12 s is kept again.
        assert_eq!(on_times[..3], [0.0, 0.0, 0.0]);
        assert!((on_times[3] - 0.36).abs() <= 1e-15);
        assert_eq!(on_times[4], 0.0);
        assert!((module.pulse_remainder[0] - 0.48).abs() <= 1e-15);
    }

    #[test]
    fn reset_publishes_zero_on_times_and_restarts_the_first_call() {
        let cluster = Output::new(upstream_rcs_cluster());
        let force_cmd = Output::new(ThrusterArrayForceCommandMsg::from_active(&[-0.2; 8]));
        let mut module = ThrFiringRemainder::new(ThrFiringRemainderConfig {
            thr_min_fire_time_s: 0.2,
            base_thrust_state: BaseThrustState::OffPulsing,
            ..Default::default()
        });
        cluster.connect_to(&mut module.thr_conf_in_msg);
        force_cmd.connect_to(&mut module.thr_force_in_msg);
        module.init();
        module.update(&context(1_000_000_000));
        module.update(&context(1_500_000_000));
        // (0.5 - 0.2) N of 0.5 N over 0.5 s.
        assert!((module.on_time_out_msg.read().on_time_request_s[0] - 0.3).abs() <= 1e-15);

        module.reset(1_500_000_000);
        assert_eq!(
            module.on_time_out_msg.read().on_time_request_s,
            [0.0; MAX_EFF_COUNT]
        );
        module.update(&context(2_000_000_000));
        let output = module.on_time_out_msg.read().on_time_request_s;
        assert_eq!(output[..8], [2.0; 8]);
        assert_eq!(output[8..], [0.0; MAX_EFF_COUNT - 8]);
    }
}
