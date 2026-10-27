//! Thruster firing with a Schmitt trigger: port of upstream Basilisk
//! `fswAlgorithms/effectorInterfaces/thrFiringSchmitt`.
//!
//! The module turns commanded thruster forces into thruster on-times. A
//! request shorter than the minimum fire time is expressed as a duty level
//! (a fraction of the minimum pulse): at or above `level_on` the thruster fires
//! one minimum pulse, at or below `level_off` it stays off, and in between it
//! keeps its previous on/off state. Longer requests are fired as requested,
//! and requests at or beyond the control period are over-saturated to 1.1
//! control periods. Off-pulsing (thrusters nominally on, such as DV thrusters)
//! is handled by adding each thruster's maximum thrust to the negative force
//! request.

use super::thr_firing_remainder::{BaseThrustState, ControlPeriodClock, requested_on_time_s};
use crate::messages::{
    Input, Output, ThrusterArrayConfigMsg, ThrusterArrayForceCommandMsg,
    ThrusterArrayOnTimeCommandMsg,
};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`ThrFiringSchmitt`].
#[derive(Clone, Debug)]
pub struct ThrFiringSchmittConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `level_on` \[-\]: duty level (fraction of a minimum pulse) at or
    /// above which a short request fires a minimum pulse (default 0).
    pub level_on: f64,
    /// Upstream `level_off` \[-\]: duty level at or below which a short request
    /// leaves the thruster off (default 0).
    pub level_off: f64,
    /// Upstream `thrMinFireTime` \[s\]: minimum thruster on-time (default 0).
    pub thr_min_fire_time_s: f64,
    /// Upstream `baseThrustState`: on- or off-pulsing (default on-pulsing).
    pub base_thrust_state: BaseThrustState,
}

impl Default for ThrFiringSchmittConfig {
    fn default() -> Self {
        Self {
            name: "thrFiringSchmitt".to_string(),
            level_on: 0.0,
            level_off: 0.0,
            thr_min_fire_time_s: 0.0,
            base_thrust_state: BaseThrustState::OnPulsing,
        }
    }
}

/// Maps thruster force commands to on-times with a Schmitt trigger (upstream
/// `thrFiringSchmitt`).
#[derive(Clone, Debug)]
pub struct ThrFiringSchmitt {
    /// Module configuration.
    pub config: ThrFiringSchmittConfig,
    /// Upstream `thrForceInMsg`: commanded thruster forces.
    pub thr_force_in_msg: Input<ThrusterArrayForceCommandMsg>,
    /// Upstream `thrConfInMsg`: thruster cluster configuration, read at reset.
    pub thr_conf_in_msg: Input<ThrusterArrayConfigMsg>,
    /// Upstream `onTimeOutMsg`: commanded thruster on-times.
    pub on_time_out_msg: Output<ThrusterArrayOnTimeCommandMsg>,
    /// Upstream `maxThrust` \[N\], one entry per installed thruster.
    max_thrust_n: Vec<f64>,
    /// Upstream `lastThrustState`: whether each thruster was left on.
    last_thrust_on: Vec<bool>,
    clock: ControlPeriodClock,
}

impl ThrFiringSchmitt {
    /// Builds the module with unconnected ports and no thrusters; the cluster
    /// is read at reset.
    pub fn new(config: ThrFiringSchmittConfig) -> Self {
        Self {
            config,
            thr_force_in_msg: Input::default(),
            thr_conf_in_msg: Input::default(),
            on_time_out_msg: Output::default(),
            max_thrust_n: Vec::new(),
            last_thrust_on: Vec::new(),
            clock: ControlPeriodClock::default(),
        }
    }

    /// Upstream `Reset_thrFiringSchmitt`: reads the thruster configuration,
    /// turns every trigger off, clears the control-period history, and
    /// publishes zero on-times so that a disabled module leaves the thrusters
    /// off.
    pub fn reset(&mut self, _call_time_nanos: u64) {
        self.clock.reset();

        if !self.thr_conf_in_msg.is_connected() {
            log::error!("thrFiringSchmitt.thr_conf_in_msg is not connected");
        }
        if !self.thr_force_in_msg.is_connected() {
            log::error!("thrFiringSchmitt.thr_force_in_msg is not connected");
        }

        self.max_thrust_n = self
            .thr_conf_in_msg
            .read()
            .active_thrusters()
            .iter()
            .map(|thruster| thruster.max_thrust_n)
            .collect();
        self.last_thrust_on = vec![false; self.max_thrust_n.len()];

        self.on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
    }

    fn compute_on_times(&mut self, call_time_nanos: u64) -> ThrusterArrayOnTimeCommandMsg {
        let base_thrust_state = self.config.base_thrust_state;
        let Some(control_period_s) = self.clock.tick(call_time_nanos) else {
            return base_thrust_state.first_call_on_times(self.max_thrust_n.len());
        };

        let forces = self.thr_force_in_msg.read();
        let on_times: Vec<f64> = forces
            .thrust_force_n
            .iter()
            .zip(&self.max_thrust_n)
            .zip(&mut self.last_thrust_on)
            .map(|((&force_n, &max_thrust_n), last_on)| {
                let on_time_s =
                    requested_on_time_s(force_n, max_thrust_n, base_thrust_state, control_period_s);
                self.config
                    .schmitt_trigger(on_time_s, control_period_s, last_on)
            })
            .collect();
        ThrusterArrayOnTimeCommandMsg::from_active(&on_times)
    }
}

impl ThrFiringSchmittConfig {
    /// Applies the trigger to one thruster's requested on-time \[s\] and
    /// updates its on/off memory.
    fn schmitt_trigger(&self, on_time_s: f64, control_period_s: f64, last_on: &mut bool) -> f64 {
        let min_fire_time_s = self.thr_min_fire_time_s;
        if on_time_s < min_fire_time_s {
            let level = on_time_s / min_fire_time_s;
            if level >= self.level_on {
                *last_on = true;
            } else if level <= self.level_off {
                *last_on = false;
            }
            if *last_on { min_fire_time_s } else { 0.0 }
        } else if on_time_s >= control_period_s {
            *last_on = true;
            // Over-saturate so the thruster stays on for the whole period.
            1.1 * control_period_s
        } else {
            *last_on = true;
            on_time_s
        }
    }
}

impl Module for ThrFiringSchmitt {
    fn init(&mut self) {
        self.on_time_out_msg
            .write(ThrusterArrayOnTimeCommandMsg::default());
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrFiringSchmitt::reset(self, context.current_sim_nanos);
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

    use super::{BaseThrustState, ThrFiringSchmitt, ThrFiringSchmittConfig};

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
    fn record(module: &mut ThrFiringSchmitt, nanos: u64) -> [f64; 8] {
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
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
    ];

    const OFF_PULSING_TRUTH: [[f64; 8]; 7] = [
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
    ];

    const ON_PULSING_RESET_TRUTH: [[f64; 8]; 12] = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.49],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
        [0.55, 0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.0],
    ];

    const OFF_PULSING_RESET_TRUTH: [[f64; 8]; 12] = [
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.2, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
        [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
        [0.55, 0.4, 0.3, 0.2, 0.0, 0.0, 0.0, 0.0],
    ];

    /// Upstream `test_thrFiringSchmitt`, all four `(resetCheck, dvOn)`
    /// parametrizations: a 0.5 s task fed four successive force requests up to
    /// 3 s (and, with `resetCheck`, a Reset at 3 s and a run to 5.5 s), then a
    /// re-initialization with a nonzero request and a final Reset that must
    /// zero the output. Upstream's `compareArray` only checks the first three
    /// thrusters of each sample; all eight are checked here.
    #[test]
    fn matches_upstream_thr_firing_schmitt_unit_test() {
        for (reset_check, dv_on) in [(false, false), (true, false), (false, true), (true, true)] {
            // (request, stop time of the run that uses it)
            let schedule: [([f64; 8], u64); 4] = if dv_on {
                [
                    (
                        [0.0, -0.1, -0.2, -0.3, -0.349, -0.351, -0.451, -0.5],
                        1_000_000_000,
                    ),
                    (
                        [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5],
                        2_000_000_000,
                    ),
                    (
                        [0.0, -0.1, -0.2, -0.3, -0.5, -0.351, -0.451, -0.5],
                        2_500_000_000,
                    ),
                    (
                        [0.0, -0.1, -0.2, -0.3, -0.351, -0.351, -0.451, -0.5],
                        3_000_000_000,
                    ),
                ]
            } else {
                [
                    (
                        [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.49],
                        1_000_000_000,
                    ),
                    (
                        [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11],
                        2_000_000_000,
                    ),
                    (
                        [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.01],
                        2_500_000_000,
                    ),
                    (
                        [0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.11],
                        3_000_000_000,
                    ),
                ]
            };
            let cluster = Output::new(upstream_rcs_cluster());
            let force_cmd = Output::new(ThrusterArrayForceCommandMsg::default());
            // Configured after construction, as the pytest does.
            let mut module = ThrFiringSchmitt::new(ThrFiringSchmittConfig::default());
            module.config.thr_min_fire_time_s = 0.2;
            module.config.base_thrust_state = if dv_on {
                BaseThrustState::OffPulsing
            } else {
                BaseThrustState::OnPulsing
            };
            module.config.level_on = 0.75;
            module.config.level_off = 0.25;
            cluster.connect_to(&mut module.thr_conf_in_msg);
            force_cmd.connect_to(&mut module.thr_force_in_msg);
            module.init();

            let mut samples = Vec::new();
            let mut next_tick_nanos = 0;
            for (request, stop_nanos) in schedule {
                force_cmd.write(ThrusterArrayForceCommandMsg::from_active(&request));
                samples.extend(
                    ticks(next_tick_nanos, stop_nanos).map(|nanos| record(&mut module, nanos)),
                );
                next_tick_nanos = stop_nanos + TASK_PERIOD_NANOS;
            }
            let mut stop_nanos = 3_000_000_000;
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
                0.5, 0.05, 0.09, 0.11, 0.16, 0.18, 0.2, 0.49,
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
    fn keeps_the_previous_state_between_the_two_levels() {
        let cluster = Output::new(ThrusterArrayConfigMsg::from_active(&[ThrusterConfigMsg {
            max_thrust_n: 1.0,
            ..Default::default()
        }]));
        let force_cmd = Output::new(ThrusterArrayForceCommandMsg::default());
        let mut module = ThrFiringSchmitt::new(ThrFiringSchmittConfig {
            level_on: 0.75,
            level_off: 0.25,
            thr_min_fire_time_s: 0.5,
            ..Default::default()
        });
        cluster.connect_to(&mut module.thr_conf_in_msg);
        force_cmd.connect_to(&mut module.thr_force_in_msg);
        module.init();
        module.update(&context(1_000_000_000));

        // With a 1 N thruster and a 1 s period the duty level is force / 0.5.
        let on_times: Vec<f64> = [0.25, 0.4, 0.1, 0.25, 0.3]
            .iter()
            .zip(2..)
            .map(|(&force_n, seconds)| {
                force_cmd.write(ThrusterArrayForceCommandMsg::from_active(&[force_n]));
                module.update(&context(seconds * 1_000_000_000));
                module.on_time_out_msg.read().on_time_request_s[0]
            })
            .collect();
        // Level 0.5 starting off stays off; 0.8 turns on; 0.2 turns off; 0.5
        // stays off; 0.6 stays off.
        assert_eq!(on_times, [0.0, 0.5, 0.0, 0.0, 0.0]);

        // A request of at least one minimum pulse turns the trigger on, so a
        // following in-between level keeps firing.
        force_cmd.write(ThrusterArrayForceCommandMsg::from_active(&[0.6]));
        module.update(&context(7_000_000_000));
        assert_eq!(module.on_time_out_msg.read().on_time_request_s[0], 0.6);
        force_cmd.write(ThrusterArrayForceCommandMsg::from_active(&[0.25]));
        module.update(&context(8_000_000_000));
        assert_eq!(module.on_time_out_msg.read().on_time_request_s[0], 0.5);
        assert!(module.last_thrust_on[0]);

        module.reset(8_000_000_000);
        assert!(!module.last_thrust_on[0]);
    }
}
