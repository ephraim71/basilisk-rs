//! Pulsed external torque: port of upstream Basilisk
//! `simulation/dynamics/ExtPulsedTorque`.
//!
//! Applies a torque about the body frame origin `B` in a repeating sequence of
//! fixed-length intervals of `pulse_interval_s` each: `count_on_pulse`
//! intervals of `+T`, `count_on_pulse` intervals of `−T`, then `count_off`
//! intervals of no torque. The phase is a pure function of the evaluation
//! time, so repeated, retried or out-of-order evaluations (adaptive
//! integrators, resets) neither advance nor rewind it. Time within a quarter
//! interval, and within a few rounding errors of the time itself, of a
//! transition is taken to be on it, so a decimal interval's rounding cannot
//! delay a transition by one evaluation. `count_on_pulse = 0` disables the
//! torque.
//!
//! Upstream evaluates the pulse at each integrator stage's time. That time is
//! passed to [`DynamicEffector::compute_output_at`];
//! [`DynamicEffector::compute_output`], which carries no time, evaluates at
//! the start of the current integration step, as recorded in
//! `pre_integration`.

use std::any::Any;

use nalgebra::Vector3;

use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::{DynamicEffector, EffectorOutput};
use crate::time::NANO2SEC;

/// User-settable parameters of [`ExtPulsedTorque`]. Upstream's counts are
/// `int`s that must not be negative; here they are unsigned.
#[derive(Clone, Debug)]
pub struct ExtPulsedTorqueConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `pulsedTorqueExternalPntB_B` \[N m\]: the pulse torque `T`
    /// about `B`, body components.
    pub pulsed_torque_body_nm: Vector3<f64>,
    /// Upstream `countOnPulse`: intervals of each of the positive and the
    /// negative pulse; zero disables the torque.
    pub count_on_pulse: u32,
    /// Upstream `countOff`: intervals of no torque after each pulse pair.
    pub count_off: u32,
    /// Upstream `pulseInterval` \[s\]: length of one interval, independent
    /// of the integration step (default 1 s).
    pub pulse_interval_s: f64,
}

impl Default for ExtPulsedTorqueConfig {
    fn default() -> Self {
        Self {
            name: String::new(),
            pulsed_torque_body_nm: Vector3::zeros(),
            count_on_pulse: 0,
            count_off: 0,
            pulse_interval_s: 1.0,
        }
    }
}

impl ExtPulsedTorqueConfig {
    /// Upstream `validateConfiguration`: a finite torque, a finite positive
    /// interval and a finite cycle length.
    pub fn validate(&self) -> Result<(), String> {
        if !self
            .pulsed_torque_body_nm
            .iter()
            .all(|value| value.is_finite())
        {
            return Err(format!(
                "pulsed_torque_body_nm must contain only finite values, got {:?}",
                self.pulsed_torque_body_nm
            ));
        }
        if !self.pulse_interval_s.is_finite() || self.pulse_interval_s <= 0.0 {
            return Err(format!(
                "pulse_interval_s must be finite and greater than zero, got {}",
                self.pulse_interval_s
            ));
        }
        if !(self.count_period() * self.pulse_interval_s).is_finite() {
            return Err(
                "pulse counts and pulse_interval_s must produce a finite cycle duration"
                    .to_string(),
            );
        }
        Ok(())
    }

    /// Intervals in one cycle, `2 count_on_pulse + count_off`, in floating
    /// point so that large counts cannot overflow.
    fn count_period(&self) -> f64 {
        2.0 * f64::from(self.count_on_pulse) + f64::from(self.count_off)
    }
}

/// Pulsed torque about the body frame origin: upstream `ExtPulsedTorque`.
#[derive(Clone, Debug)]
pub struct ExtPulsedTorque {
    /// User-settable parameters, checked again at every evaluation.
    pub config: ExtPulsedTorqueConfig,
    /// Start \[ns\] of the current integration step.
    step_start_nanos: u64,
}

impl ExtPulsedTorque {
    /// # Panics
    ///
    /// If `config` fails [`ExtPulsedTorqueConfig::validate`].
    pub fn new(config: ExtPulsedTorqueConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid ExtPulsedTorqueConfig: {message}");
        }
        Self {
            config,
            step_start_nanos: 0,
        }
    }

    /// Upstream `computeForceTorque`: the torque \[N m\] about `B` at time
    /// `time_s` \[s\].
    ///
    /// # Panics
    ///
    /// If the configuration has become invalid, or `time_s` is not finite and
    /// non-negative.
    pub fn torque_at(&self, time_s: f64) -> Vector3<f64> {
        let config = &self.config;
        if let Err(message) = config.validate() {
            panic!("invalid ExtPulsedTorqueConfig: {message}");
        }
        assert!(
            time_s.is_finite() && time_s >= 0.0,
            "ExtPulsedTorque: integTime must be finite and non-negative, in seconds, got {time_s}"
        );
        if config.count_on_pulse == 0 {
            return Vector3::zeros();
        }

        let interval_s = config.pulse_interval_s;
        let on_duration_s = f64::from(config.count_on_pulse) * interval_s;
        let cycle_duration_s = config.count_period() * interval_s;
        let raw_phase_s = time_s % cycle_duration_s;
        // Each tolerance scales with its own transition and the evaluation
        // time, so a long off period cannot widen the window around the first
        // pulse edges; the quarter-interval bound keeps windows apart.
        let is_near_transition = |transition_s: f64| {
            let tolerance_s =
                (0.25 * interval_s).min(4.0 * f64::EPSILON * time_s.max(transition_s));
            (raw_phase_s - transition_s).abs() <= tolerance_s
        };
        let phase_s = if is_near_transition(cycle_duration_s) {
            0.0
        } else if is_near_transition(on_duration_s) {
            on_duration_s
        } else if is_near_transition(2.0 * on_duration_s) {
            2.0 * on_duration_s
        } else {
            raw_phase_s
        };

        if phase_s < on_duration_s {
            config.pulsed_torque_body_nm
        } else if phase_s < 2.0 * on_duration_s {
            -config.pulsed_torque_body_nm
        } else {
            Vector3::zeros()
        }
    }
}

impl DynamicEffector for ExtPulsedTorque {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn pre_integration(&mut self, current_sim_nanos: u64, _dt_seconds: f64) {
        self.step_start_nanos = current_sim_nanos;
    }

    fn compute_output(&self, state: &SpacecraftStateMsg) -> EffectorOutput {
        self.compute_output_at(state, self.step_start_nanos)
    }

    fn compute_output_at(
        &self,
        _state: &SpacecraftStateMsg,
        current_sim_nanos: u64,
    ) -> EffectorOutput {
        EffectorOutput {
            force_inertial_n: Vector3::zeros(),
            torque_body_nm: self.torque_at(current_sim_nanos as f64 * NANO2SEC),
        }
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `ExtPulsedTorque/_UnitTest/test_extPulsedTorque.py` with
    //! its exact comparisons. Upstream's checks of negative counts are type
    //! guarantees here (the counts are unsigned); its `Reset`, attachment and
    //! evaluation validation paths are construction and evaluation. The
    //! integrated case runs the spacecraft's own integrator (RK4, and the
    //! Heun (upstream RK2) and RKF45 schemes upstream also tries).

    use hifitime::Epoch;
    use nalgebra::Matrix3;

    use super::*;
    use crate::integrators::{Heun, Integrator, Rkf45};
    use crate::simulation::Simulation;
    use crate::spacecraft::{Spacecraft, SpacecraftConfig};

    /// Upstream `configured_pulse`: transitions at exactly representable
    /// times.
    fn configured_pulse(count_off: u32) -> ExtPulsedTorque {
        ExtPulsedTorque::new(ExtPulsedTorqueConfig {
            name: "pulse".to_string(),
            pulsed_torque_body_nm: Vector3::new(-1.0, 2.0, -3.0),
            count_on_pulse: 2,
            count_off,
            pulse_interval_s: 0.125,
        })
    }

    #[track_caller]
    fn assert_sign(effector: &ExtPulsedTorque, time_s: f64, sign: f64) {
        assert_eq!(
            effector.torque_at(time_s),
            sign * effector.config.pulsed_torque_body_nm,
            "at t = {time_s:e} s"
        );
    }

    /// The largest double below `value` (upstream `np.nextafter(value, 0)`).
    fn next_below(value: f64) -> f64 {
        f64::from_bits(value.to_bits() - 1)
    }

    /// Upstream `test_pulse_sequence`.
    #[test]
    fn counts_give_positive_negative_and_off_intervals() {
        for count_off in [0, 3] {
            let effector = configured_pulse(count_off);
            let mut signs = vec![1.0, 1.0, -1.0, -1.0];
            signs.extend(std::iter::repeat_n(0.0, count_off as usize));
            let expected: Vec<f64> = signs.iter().chain(&signs).copied().collect();
            for (index, sign) in expected.iter().enumerate() {
                for fraction in [0.0, 0.25, 0.75] {
                    let time_s = (index as f64 + fraction) * effector.config.pulse_interval_s;
                    assert_sign(&effector, time_s, *sign);
                }
            }
        }
    }

    /// Upstream `test_pulse_transition_boundaries`.
    #[test]
    fn each_transition_selects_the_new_segment() {
        let effector = configured_pulse(3);
        for (time_s, sign) in [
            (0.0, 1.0),
            (0.25 - 1e-12, 1.0),
            (next_below(0.25), -1.0),
            (0.25, -1.0),
            (0.5 - 1e-12, -1.0),
            (next_below(0.5), 0.0),
            (0.5, 0.0),
            (0.875 - 1e-12, 0.0),
            (next_below(0.875), 1.0),
            (0.875, 1.0),
        ] {
            assert_sign(&effector, time_s, sign);
        }
    }

    /// Upstream `test_decimal_pulse_intervals_select_the_new_segment_at_boundaries`.
    #[test]
    fn decimal_intervals_switch_on_their_boundaries() {
        for interval_s in [0.1, 0.03, 0.07] {
            let mut effector = configured_pulse(1);
            effector.config.count_on_pulse = 1;
            effector.config.pulse_interval_s = interval_s;
            for index in (0..30).chain([1_000_000, 1_000_001, 1_000_002]) {
                let sign = [1.0, -1.0, 0.0][index % 3];
                assert_sign(&effector, index as f64 * interval_s, sign);
            }
        }
    }

    /// Upstream `test_long_off_period_does_not_advance_initial_pulse_transitions`.
    #[test]
    fn a_long_off_period_does_not_move_the_first_edges() {
        for count_off in [1, 1_000_000, 2_000_000_000] {
            for interval_s in [0.001, 1.0, 1_000_000.0] {
                let mut effector = configured_pulse(count_off);
                effector.config.count_on_pulse = 1;
                effector.config.pulse_interval_s = interval_s;
                for (boundary, preceding, following) in [(1.0, 1.0, -1.0), (2.0, -1.0, 0.0)] {
                    let transition_s = boundary * interval_s;
                    for (time_s, sign) in [
                        (transition_s - 1e-6 * interval_s, preceding),
                        (transition_s - 1e-12 * interval_s, preceding),
                        (next_below(transition_s), following),
                        (transition_s, following),
                        (transition_s + 1e-12 * interval_s, following),
                    ] {
                        assert_sign(&effector, time_s, sign);
                    }
                }
            }
        }
    }

    /// Upstream `test_pulse_repeated_and_reversed_evaluations_preserve_phase`,
    /// through the effector interface, whose step bookkeeping must not move
    /// the phase either.
    #[test]
    fn repeated_and_reversed_evaluations_keep_the_phase() {
        let mut effector = configured_pulse(3);
        let state = SpacecraftStateMsg::default();
        for (time_s, sign) in [
            (0.75, 0.0),
            (0.25, -1.0),
            (0.125, 1.0),
            (0.0, 1.0),
            (0.875, 1.0),
            (0.25, -1.0),
        ] {
            let nanos = (time_s * 1e9) as u64;
            for _ in 0..3 {
                effector.pre_integration(nanos, 0.01);
                let torque = sign * effector.config.pulsed_torque_body_nm;
                assert_eq!(
                    effector.compute_output_at(&state, nanos).torque_body_nm,
                    torque
                );
                assert_eq!(effector.compute_output(&state).torque_body_nm, torque);
            }
        }
    }

    /// Upstream `test_pulse_defaults_and_disabling_clear_loads`.
    #[test]
    fn defaults_are_inert_and_disabling_clears_the_torque() {
        let mut effector = ExtPulsedTorque::new(ExtPulsedTorqueConfig::default());
        let state = SpacecraftStateMsg::default();
        assert_eq!(
            effector.compute_output_at(&state, 0).torque_body_nm,
            Vector3::zeros()
        );
        effector.config.pulsed_torque_body_nm = Vector3::new(1.0, 0.0, 0.0);
        effector.config.count_on_pulse = 1;
        assert_eq!(effector.torque_at(0.0), Vector3::new(1.0, 0.0, 0.0));
        effector.config.count_on_pulse = 0;
        let output = effector.compute_output_at(&state, 0);
        assert_eq!(output.torque_body_nm, Vector3::zeros());
        assert_eq!(output.force_inertial_n, Vector3::zeros());
    }

    fn invalid_configs() -> Vec<ExtPulsedTorqueConfig> {
        let base = configured_pulse(3).config;
        let mut configs = Vec::new();
        for interval_s in [
            0.0,
            -1.0,
            f64::NAN,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::MAX,
        ] {
            configs.push(ExtPulsedTorqueConfig {
                pulse_interval_s: interval_s,
                ..base.clone()
            });
        }
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            configs.push(ExtPulsedTorqueConfig {
                pulsed_torque_body_nm: Vector3::new(value, 0.0, 0.0),
                ..base.clone()
            });
        }
        configs
    }

    /// Upstream `test_invalid_pulse_configuration`, its `reset` and
    /// `attachment` paths.
    #[test]
    fn rejects_invalid_configurations_at_construction() {
        for config in invalid_configs() {
            assert!(config.validate().is_err(), "{config:?}");
            let result = std::panic::catch_unwind(|| ExtPulsedTorque::new(config.clone()));
            assert!(result.is_err(), "{config:?}");
        }
    }

    /// Upstream `test_invalid_pulse_configuration`, its `evaluation` path.
    #[test]
    fn rejects_invalid_configurations_at_evaluation() {
        for config in invalid_configs() {
            let mut effector = configured_pulse(3);
            effector.config = config.clone();
            let result = std::panic::catch_unwind(|| effector.torque_at(0.0));
            assert!(result.is_err(), "{config:?}");
        }
    }

    /// Upstream `test_invalid_pulse_evaluation_time`.
    #[test]
    fn rejects_invalid_evaluation_times() {
        for time_s in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -1.0] {
            let effector = configured_pulse(3);
            let result = std::panic::catch_unwind(|| effector.torque_at(time_s));
            let message = result.expect_err("invalid time must be rejected");
            let message = message.downcast_ref::<String>().unwrap();
            assert!(message.contains("integTime"), "{message}");
        }
    }

    /// Upstream `test_pulse_large_counts_do_not_overflow_integer_arithmetic`.
    #[test]
    fn large_counts_do_not_overflow() {
        let mut effector = configured_pulse(3);
        effector.config.count_on_pulse = i32::MAX as u32;
        effector.config.count_off = i32::MAX as u32;
        effector.config.pulse_interval_s = 1.0;
        assert_sign(&effector, f64::from(effector.config.count_on_pulse), -1.0);
    }

    /// Upstream `test_integrator_stages_and_task_order_do_not_shorten_pulses`:
    /// a unit torque on a unit inertia for 0.1875 s, before the first
    /// transition at 0.25 s, gives a rate of 0.1875 rad/s whatever the
    /// integrator and step.
    #[test]
    fn integrator_stages_do_not_shorten_pulses() {
        type MakeIntegrator = fn() -> Option<Box<dyn Integrator<Spacecraft>>>;
        let integrators: [MakeIntegrator; 3] = [
            || Some(Box::new(Heun)),
            || None,
            || Some(Box::new(Rkf45::default())),
        ];
        for integrator in integrators {
            for step_s in [0.015625, 0.03125] {
                let step_nanos = (step_s * 1e9) as u64;
                let mut effector = configured_pulse(3);
                effector.config.pulsed_torque_body_nm = Vector3::new(1.0, 0.0, 0.0);
                let mut spacecraft = Spacecraft::new(SpacecraftConfig {
                    mass_kg: 1.0,
                    hub_center_of_mass_body_m: Vector3::zeros(),
                    inertia_kg_m2: Matrix3::identity(),
                    integration_step_nanos: step_nanos,
                    initial_position_m: Vector3::zeros(),
                    initial_velocity_mps: Vector3::zeros(),
                    initial_sigma_bn: Vector3::zeros(),
                    initial_omega_radps: Vector3::zeros(),
                    integrator: integrator(),
                });
                spacecraft.add_dynamic_effector(effector);
                let state_out = spacecraft.state_out.clone();
                {
                    let mut sim =
                        Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
                    sim.add_module("spacecraft", &mut spacecraft, step_nanos, 0);
                    sim.run_for(187_500_000);
                }
                let omega = state_out.read().omega_radps;
                let expected = Vector3::new(0.1875, 0.0, 0.0);
                for (actual, truth) in omega.iter().zip(expected.iter()) {
                    assert!(
                        (actual - truth).abs() <= 1e-14 + 1e-13 * truth.abs(),
                        "step {step_s}: {omega:?}"
                    );
                }
            }
        }
    }
}
