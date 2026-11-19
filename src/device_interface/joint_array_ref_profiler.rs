//! Smooth reference profiles for an array of joints: port of upstream
//! Basilisk `simulation/deviceInterface/jointArrayRefProfiler`.
//!
//! A desired joint state that jumps (a new set of target angles) is turned
//! into a smooth reference for a joint controller such as
//! [`HingedJointArrayMotor`](crate::fsw_algorithms::effector_interfaces::hinged_joint_array_motor::HingedJointArrayMotor).
//! Whenever the desired states change, a profile starts from each joint's
//! current angle `θ_0` and rate `θ̇_0` toward its target `θ_f`, of one of
//! four [`JointProfile`] shapes over the profile time `τ`:
//!
//! * low-pass: `θ_k = θ_{k-1} + β (θ_f − θ_{k-1})`, `β = 1 − e^(−ω_c Δt)`,
//!   rates and accelerations by finite differences over `Δt`; it starts at
//!   the current state;
//! * linear: `θ_0 + (θ_f − θ_0) τ / T`;
//! * cubic: matches `θ_0`, `θ̇_0` and `θ_f` with zero final rate;
//! * quintic: as cubic, with zero initial and final accelerations.
//!
//! The polynomial profiles hold `θ_f` at rest once `τ ≥ T`. Angles may be
//! wrapped into `[start, start + 2π)` (`joint_angle_wrap_start_rad`), both
//! on the way in and on the published reference; with `use_shortest_path`
//! the target is moved by whole turns to the nearest equivalent of the
//! desired angle. The desired rates and accelerations are compared, never
//! tracked.
//!
//! Upstream compares the whole desired message with the last one to detect a
//! new command, so rewriting an identical command does not restart the
//! profile; this port does the same. Upstream raises when the desired
//! message is not written. Stand-in: connected ⇒ written; an unconnected or
//! misshapen desired message is reported and publishes nothing.
//! The shortest-path displacement is [`wrap_to_pi`], which differs from
//! upstream's `std::remainder(Δ, 2π)` only for a displacement of exactly an
//! odd multiple of `π`.

use crate::messages::{Input, JointArrayStateMsg, Output, ScalarJointStateMsg};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::rigid_body_kinematics::wrap_to_pi;
use crate::{Module, SimulationContext};

const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Shape of the reference profile: upstream `profileType` with the
/// parameters it uses.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum JointProfile {
    /// Upstream `"lowPass"`: a first-order filter toward the target.
    LowPass {
        /// Upstream `wc` \[rad/s\]: cutoff frequency, positive.
        wc_radps: f64,
        /// Upstream `filterDt` \[s\]: filter time step, positive; the module
        /// should run at this period.
        filter_dt_s: f64,
    },
    /// Upstream `"linear"`.
    Linear {
        /// Upstream `profileDuration` \[s\], positive.
        duration_s: f64,
    },
    /// Upstream `"cubic"`.
    Cubic {
        /// Upstream `profileDuration` \[s\], positive.
        duration_s: f64,
    },
    /// Upstream `"quintic"`.
    Quintic {
        /// Upstream `profileDuration` \[s\], positive.
        duration_s: f64,
    },
}

/// User-settable parameters of [`JointArrayRefProfiler`].
#[derive(Clone, Debug, PartialEq)]
pub struct JointArrayRefProfilerConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Number of joints (upstream calls `addHingedJoint` once per joint);
    /// [`JointArrayRefProfiler::new`] makes the joint inputs from it.
    pub num_hinged_joints: usize,
    /// Upstream `profileType` and its parameters.
    pub profile: JointProfile,
    /// Upstream `jointAngleWrapStart` \[rad\] while
    /// `jointAngleWrappingEnabled`: angles are wrapped into
    /// `[start, start + 2π)`; `None` leaves them unwrapped (upstream's
    /// default, and `disableJointAngleWrapping`).
    pub joint_angle_wrap_start_rad: Option<f64>,
    /// Upstream `useShortestPath`: profile each joint along the shorter way
    /// round to its target.
    pub use_shortest_path: bool,
}

impl JointArrayRefProfilerConfig {
    /// Upstream `Reset`'s checks. Returns a description of the first
    /// violation, or `Ok(())` when the configuration is usable.
    pub fn validate(&self) -> Result<(), String> {
        if self.num_hinged_joints == 0 {
            return Err("jointArrayRefProfiler has no hinged joints".to_string());
        }
        let positive = |label: &str, value: f64| {
            if value > 0.0 && value.is_finite() {
                Ok(())
            } else {
                Err(format!("{label} must be positive, got {value}"))
            }
        };
        match self.profile {
            JointProfile::LowPass {
                wc_radps,
                filter_dt_s,
            } => {
                positive("wc_radps", wc_radps)?;
                positive("filter_dt_s", filter_dt_s)?;
            }
            JointProfile::Linear { duration_s }
            | JointProfile::Cubic { duration_s }
            | JointProfile::Quintic { duration_s } => positive("duration_s", duration_s)?,
        }
        if let Some(start) = self.joint_angle_wrap_start_rad
            && !start.is_finite()
        {
            return Err(format!(
                "joint_angle_wrap_start_rad must be finite when joint-angle wrapping is enabled, \
                 got {start}"
            ));
        }
        Ok(())
    }

    /// Upstream `wrapJointAngle`, or `angle_rad` itself without wrapping.
    fn wrap(&self, angle_rad: f64) -> f64 {
        match self.joint_angle_wrap_start_rad {
            Some(start) => {
                let mut wrapped = (angle_rad - start) % TWO_PI;
                if wrapped < 0.0 {
                    wrapped += TWO_PI;
                }
                start + wrapped
            }
            None => angle_rad,
        }
    }
}

/// Joint reference profiler: upstream `JointArrayRefProfiler`.
#[derive(Clone, Debug)]
pub struct JointArrayRefProfiler {
    /// Module configuration, read every update.
    pub config: JointArrayRefProfilerConfig,
    /// Upstream `jointStatesInMsgs`: angle of each joint.
    pub joint_states_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `jointStateDotsInMsgs`: rate of each joint.
    pub joint_state_dots_in_msgs: Vec<Input<ScalarJointStateMsg>>,
    /// Upstream `desJointStatesInMsg`: the desired joint states.
    pub des_joint_states_in_msg: Input<JointArrayStateMsg>,
    /// Upstream `desJointStatesOutMsg`: the profiled reference.
    pub des_joint_states_out_msg: Output<JointArrayStateMsg>,
    /// Upstream `profileStartTime` while `profileStartTimeSet` \[ns\].
    profile_start_time_nanos: Option<u64>,
    start_angles_rad: Vec<f64>,
    start_rates_radps: Vec<f64>,
    target_angles_rad: Vec<f64>,
    ref_angles_rad: Vec<f64>,
    ref_rates_radps: Vec<f64>,
    ref_accels_radps2: Vec<f64>,
    /// Upstream `prevDesJointStates`: the command the profile started from.
    prev_des_joint_states: JointArrayStateMsg,
}

impl JointArrayRefProfiler {
    /// Builds the module with one unconnected angle and rate input per
    /// configured joint.
    ///
    /// # Panics
    ///
    /// If `config` fails [`JointArrayRefProfilerConfig::validate`].
    pub fn new(config: JointArrayRefProfilerConfig) -> Self {
        if let Err(message) = config.validate() {
            panic!("invalid JointArrayRefProfilerConfig: {message}");
        }
        let n = config.num_hinged_joints;
        // Not `vec![_; n]`: cloning a port aliases it.
        let joint_inputs = || (0..n).map(|_| Input::default()).collect();
        Self {
            joint_states_in_msgs: joint_inputs(),
            joint_state_dots_in_msgs: joint_inputs(),
            config,
            des_joint_states_in_msg: Input::default(),
            des_joint_states_out_msg: Output::default(),
            profile_start_time_nanos: None,
            start_angles_rad: vec![0.0; n],
            start_rates_radps: vec![0.0; n],
            target_angles_rad: vec![0.0; n],
            ref_angles_rad: vec![0.0; n],
            ref_rates_radps: vec![0.0; n],
            ref_accels_radps2: vec![0.0; n],
            prev_des_joint_states: JointArrayStateMsg::default(),
        }
    }

    /// Upstream `Reset`: reports missing inputs and forgets the profile.
    fn reset_module(&mut self) {
        let name = &self.config.name;
        if let Err(message) = self.config.validate() {
            log::error!("jointArrayRefProfiler '{name}': {message}");
        }
        for (label, inputs) in [
            ("joint_states_in_msgs", &self.joint_states_in_msgs),
            ("joint_state_dots_in_msgs", &self.joint_state_dots_in_msgs),
        ] {
            for (index, input) in inputs.iter().enumerate() {
                if !input.is_connected() {
                    log::error!("jointArrayRefProfiler '{name}' {label}[{index}] is not connected");
                }
            }
        }
        if !self.des_joint_states_in_msg.is_connected() {
            log::error!("jointArrayRefProfiler '{name}' des_joint_states_in_msg is not connected");
        }
        let n = self.config.num_hinged_joints;
        self.profile_start_time_nanos = None;
        self.ref_angles_rad = vec![0.0; n];
        self.ref_rates_radps = vec![0.0; n];
        self.ref_accels_radps2 = vec![0.0; n];
        self.target_angles_rad = vec![0.0; n];
    }

    /// Upstream `UpdateState`: the reference at `call_time_nanos`, or `None`
    /// when the desired states cannot be used.
    fn compute_reference(&mut self, call_time_nanos: u64) -> Option<JointArrayStateMsg> {
        let n = self.config.num_hinged_joints;
        let name = &self.config.name;
        if !self.des_joint_states_in_msg.is_connected() {
            log::error!(
                "jointArrayRefProfiler '{name}' called before the desired joint states were written"
            );
            return None;
        }
        let desired = self.des_joint_states_in_msg.read();
        for (label, values) in [
            ("states", &desired.states),
            ("state_dots", &desired.state_dots),
            ("state_ddots", &desired.state_ddots),
        ] {
            if values.len() != n {
                log::error!(
                    "jointArrayRefProfiler '{name}' des_joint_states_in_msg.{label} holds {} \
                     values for {n} joints",
                    values.len()
                );
                return None;
            }
        }

        let new_command =
            self.profile_start_time_nanos.is_none() || desired != self.prev_des_joint_states;
        if new_command {
            self.start_profile(call_time_nanos, &desired);
            self.prev_des_joint_states = desired;
        }

        match self.config.profile {
            JointProfile::LowPass {
                wc_radps,
                filter_dt_s,
            } => {
                if !new_command {
                    self.low_pass(wc_radps, filter_dt_s);
                }
            }
            JointProfile::Linear { duration_s } => {
                self.polynomial(call_time_nanos, duration_s, linear_profile);
            }
            JointProfile::Cubic { duration_s } => {
                self.polynomial(call_time_nanos, duration_s, cubic_profile);
            }
            JointProfile::Quintic { duration_s } => {
                self.polynomial(call_time_nanos, duration_s, quintic_profile);
            }
        }

        Some(JointArrayStateMsg {
            states: self
                .ref_angles_rad
                .iter()
                .map(|&angle| self.config.wrap(angle))
                .collect(),
            state_dots: self.ref_rates_radps.clone(),
            state_ddots: self.ref_accels_radps2.clone(),
        })
    }

    /// A new command: the profile starts from the current joint states.
    fn start_profile(&mut self, call_time_nanos: u64, desired: &JointArrayStateMsg) {
        self.profile_start_time_nanos = Some(call_time_nanos);
        for joint in 0..self.config.num_hinged_joints {
            let start = self
                .config
                .wrap(self.joint_states_in_msgs[joint].read().state);
            let desired_angle = self.config.wrap(desired.states[joint]);
            self.start_angles_rad[joint] = start;
            self.start_rates_radps[joint] = self.joint_state_dots_in_msgs[joint].read().state;
            self.target_angles_rad[joint] = if self.config.use_shortest_path {
                start + wrap_to_pi(desired_angle - start)
            } else {
                desired_angle
            };
        }
        if matches!(self.config.profile, JointProfile::LowPass { .. }) {
            self.ref_angles_rad.clone_from(&self.start_angles_rad);
            self.ref_rates_radps.clone_from(&self.start_rates_radps);
            self.ref_accels_radps2.fill(0.0);
        }
    }

    /// Upstream `computeLowPassFilter`: one filter step.
    fn low_pass(&mut self, wc_radps: f64, filter_dt_s: f64) {
        let beta = 1.0 - (-wc_radps * filter_dt_s).exp();
        for joint in 0..self.config.num_hinged_joints {
            let previous_angle = self.ref_angles_rad[joint];
            let previous_rate = self.ref_rates_radps[joint];
            let angle = previous_angle + beta * (self.target_angles_rad[joint] - previous_angle);
            let rate = (angle - previous_angle) / filter_dt_s;
            self.ref_angles_rad[joint] = angle;
            self.ref_rates_radps[joint] = rate;
            self.ref_accels_radps2[joint] = (rate - previous_rate) / filter_dt_s;
        }
    }

    /// Upstream `computeLinearProfile` / `computeCubicProfile` /
    /// `computeQuinticProfile`: `profile(θ_0, θ_f, θ̇_0, T, τ)` per joint,
    /// and the target at rest once `τ ≥ T`.
    fn polynomial(&mut self, call_time_nanos: u64, duration_s: f64, profile: PolynomialProfile) {
        let start_nanos = self.profile_start_time_nanos.unwrap_or(call_time_nanos);
        let tau = diff_nanos_to_seconds(call_time_nanos, start_nanos);
        for joint in 0..self.config.num_hinged_joints {
            let (angle, rate, accel) = if tau >= duration_s {
                (self.target_angles_rad[joint], 0.0, 0.0)
            } else {
                profile(
                    self.start_angles_rad[joint],
                    self.target_angles_rad[joint],
                    self.start_rates_radps[joint],
                    duration_s,
                    tau,
                )
            };
            self.ref_angles_rad[joint] = angle;
            self.ref_rates_radps[joint] = rate;
            self.ref_accels_radps2[joint] = accel;
        }
    }
}

/// One joint's polynomial profile: `(θ_0, θ_f, θ̇_0, T, τ)` to the angle,
/// rate and acceleration at `τ`.
type PolynomialProfile = fn(f64, f64, f64, f64, f64) -> (f64, f64, f64);

/// Upstream `computeLinearProfile` for one joint: angle, rate, acceleration.
fn linear_profile(theta0: f64, thetaf: f64, _theta_dot0: f64, t: f64, tau: f64) -> (f64, f64, f64) {
    (
        theta0 + (thetaf - theta0) * tau / t,
        (thetaf - theta0) / t,
        0.0,
    )
}

/// Upstream `computeCubicProfile` for one joint.
fn cubic_profile(theta0: f64, thetaf: f64, theta_dot0: f64, t: f64, tau: f64) -> (f64, f64, f64) {
    let a0 = theta0;
    let a1 = theta_dot0;
    let a2 = (3.0 * (thetaf - theta0)) / (t * t) - (2.0 * theta_dot0) / t;
    let a3 = -(2.0 * (thetaf - theta0)) / (t * t * t) + theta_dot0 / (t * t);
    (
        a0 + a1 * tau + a2 * tau * tau + a3 * tau * tau * tau,
        a1 + 2.0 * a2 * tau + 3.0 * a3 * tau * tau,
        2.0 * a2 + 6.0 * a3 * tau,
    )
}

/// Upstream `computeQuinticProfile` for one joint.
fn quintic_profile(theta0: f64, thetaf: f64, theta_dot0: f64, t: f64, tau: f64) -> (f64, f64, f64) {
    let delta = thetaf - theta0;
    let a0 = theta0;
    let a1 = theta_dot0;
    let a2 = 0.0;
    let a3 = (20.0 * delta - 12.0 * theta_dot0 * t) / (2.0 * t * t * t);
    let a4 = (-30.0 * delta + 16.0 * theta_dot0 * t) / (2.0 * t * t * t * t);
    let a5 = (12.0 * delta - 6.0 * theta_dot0 * t) / (2.0 * t * t * t * t * t);
    let tau2 = tau * tau;
    let tau3 = tau2 * tau;
    let tau4 = tau3 * tau;
    let tau5 = tau4 * tau;
    (
        a0 + a1 * tau + a2 * tau2 + a3 * tau3 + a4 * tau4 + a5 * tau5,
        a1 + 2.0 * a2 * tau + 3.0 * a3 * tau2 + 4.0 * a4 * tau3 + 5.0 * a5 * tau4,
        2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau2 + 20.0 * a5 * tau3,
    )
}

impl Module for JointArrayRefProfiler {
    fn init(&mut self) {
        self.reset_module();
    }

    fn update(&mut self, context: &SimulationContext) {
        if let Some(reference) = self.compute_reference(context.current_sim_nanos) {
            self.des_joint_states_out_msg.write(reference);
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports upstream `jointArrayRefProfiler/_UnitTest/test_jointArrayRefProfiler.py`,
    //! every test with its truth and `np.testing.assert_allclose` tolerances
    //! (`rtol = 1e-7` unless the pytest gives an `atol`).

    use std::f64::consts::PI;

    use hifitime::Epoch;

    use super::{JointArrayRefProfiler, JointArrayRefProfilerConfig, JointProfile};
    use crate::messages::{JointArrayStateMsg, Output, ScalarJointStateMsg};
    use crate::{Module, SimulationContext};

    const JOINT_ANGLES: [f64; 3] = [0.1, 0.2, -0.3];
    const JOINT_RATES: [f64; 3] = [0.05, -0.07, 0.04];
    const DES_JOINT_ANGLES: [f64; 3] = [1.0, -1.0, 1.5];
    const ZEROS: [f64; 3] = [0.0; 3];
    const PROFILE_DURATION: f64 = 0.4;

    /// Upstream `macros.sec2nano`.
    fn sec2nano(time_s: f64) -> u64 {
        (time_s * 1e9 + 0.5) as u64
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// `np.testing.assert_allclose(actual, desired, rtol, atol)`.
    fn assert_allclose(actual: &[f64], desired: &[f64], rtol: f64, atol: f64, what: &str) {
        assert_eq!(actual.len(), desired.len(), "{what}: length");
        for (index, (a, d)) in actual.iter().zip(desired).enumerate() {
            assert!(
                (a - d).abs() <= atol + rtol * d.abs(),
                "{what}[{index}]: {a} vs {d} (rtol {rtol}, atol {atol})"
            );
        }
    }

    fn assert_default_close(actual: &[f64], desired: &[f64], what: &str) {
        assert_allclose(actual, desired, 1e-7, 0.0, what);
    }

    /// The module with its joints and desired states wired to outputs the
    /// test rewrites, as the pytest resubscribes to new messages.
    struct Fixture {
        module: JointArrayRefProfiler,
        joint_states: Vec<Output<ScalarJointStateMsg>>,
        joint_rates: Vec<Output<ScalarJointStateMsg>>,
        desired: Output<JointArrayStateMsg>,
    }

    impl Fixture {
        /// Upstream `setup_joint_array_ref_profiler`.
        fn new(profile: JointProfile) -> Self {
            let mut module = JointArrayRefProfiler::new(JointArrayRefProfilerConfig {
                name: "jointArrayRefProfiler".to_string(),
                num_hinged_joints: 3,
                profile,
                joint_angle_wrap_start_rad: None,
                use_shortest_path: false,
            });
            let joint_states: Vec<_> = JOINT_ANGLES
                .iter()
                .map(|&state| Output::new(ScalarJointStateMsg { state }))
                .collect();
            let joint_rates: Vec<_> = JOINT_RATES
                .iter()
                .map(|&state| Output::new(ScalarJointStateMsg { state }))
                .collect();
            for (output, input) in joint_states.iter().zip(&mut module.joint_states_in_msgs) {
                output.connect_to(input);
            }
            for (output, input) in joint_rates.iter().zip(&mut module.joint_state_dots_in_msgs) {
                output.connect_to(input);
            }
            let desired = Output::new(desired(&DES_JOINT_ANGLES, &ZEROS));
            desired.connect_to(&mut module.des_joint_states_in_msg);
            Self {
                module,
                joint_states,
                joint_rates,
                desired,
            }
        }

        fn set_joints(&self, angles: &[f64], rates: Option<&[f64]>) {
            for (output, &state) in self.joint_states.iter().zip(angles) {
                output.write(ScalarJointStateMsg { state });
            }
            if let Some(rates) = rates {
                for (output, &state) in self.joint_rates.iter().zip(rates) {
                    output.write(ScalarJointStateMsg { state });
                }
            }
        }

        fn update(&mut self, time_s: f64) -> JointArrayStateMsg {
            self.module.update(&context(sec2nano(time_s)));
            self.module.des_joint_states_out_msg.read()
        }
    }

    fn desired(angles: &[f64], rates: &[f64]) -> JointArrayStateMsg {
        JointArrayStateMsg {
            states: angles.to_vec(),
            state_dots: rates.to_vec(),
            state_ddots: ZEROS.to_vec(),
        }
    }

    /// Upstream cubic coefficients for one joint at `tau`.
    fn cubic(theta0: f64, thetaf: f64, theta_dot0: f64, tau: f64) -> (f64, f64, f64) {
        let t = PROFILE_DURATION;
        let a0 = theta0;
        let a1 = theta_dot0;
        let a2 = (3.0 * (thetaf - theta0)) / t.powi(2) - (2.0 * theta_dot0) / t;
        let a3 = -(2.0 * (thetaf - theta0)) / t.powi(3) + theta_dot0 / t.powi(2);
        (
            a0 + a1 * tau + a2 * tau.powi(2) + a3 * tau.powi(3),
            a1 + 2.0 * a2 * tau + 3.0 * a3 * tau.powi(2),
            2.0 * a2 + 6.0 * a3 * tau,
        )
    }

    /// Upstream quintic coefficients for one joint at `tau`.
    fn quintic(theta0: f64, thetaf: f64, theta_dot0: f64, tau: f64) -> (f64, f64, f64) {
        let t = PROFILE_DURATION;
        let delta = thetaf - theta0;
        let a0 = theta0;
        let a1 = theta_dot0;
        let a2 = 0.0;
        let a3 = (20.0 * delta - 12.0 * theta_dot0 * t) / (2.0 * t.powi(3));
        let a4 = (-30.0 * delta + 16.0 * theta_dot0 * t) / (2.0 * t.powi(4));
        let a5 = (12.0 * delta - 6.0 * theta_dot0 * t) / (2.0 * t.powi(5));
        (
            a0 + a1 * tau
                + a2 * tau.powi(2)
                + a3 * tau.powi(3)
                + a4 * tau.powi(4)
                + a5 * tau.powi(5),
            a1 + 2.0 * a2 * tau
                + 3.0 * a3 * tau.powi(2)
                + 4.0 * a4 * tau.powi(3)
                + 5.0 * a5 * tau.powi(4),
            2.0 * a2 + 6.0 * a3 * tau + 12.0 * a4 * tau.powi(2) + 20.0 * a5 * tau.powi(3),
        )
    }

    /// `np.remainder(x - start, 2π) + start`.
    fn np_wrap(x: f64, start: f64) -> f64 {
        (x - start).rem_euclid(2.0 * PI) + start
    }

    fn per_joint(f: impl Fn(usize) -> (f64, f64, f64)) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut angles = Vec::new();
        let mut rates = Vec::new();
        let mut accels = Vec::new();
        for joint in 0..3 {
            let (angle, rate, accel) = f(joint);
            angles.push(angle);
            rates.push(rate);
            accels.push(accel);
        }
        (angles, rates, accels)
    }

    /// Upstream `test_joint_array_ref_profiler_filter`.
    #[test]
    fn low_pass_filter_steps_toward_the_target() {
        let filter_dt = 0.1;
        let wc = 2.0 * PI * 0.5;
        let mut fixture = Fixture::new(JointProfile::LowPass {
            wc_radps: wc,
            filter_dt_s: filter_dt,
        });
        fixture.module.init();

        let out = fixture.update(0.0);
        assert_default_close(&out.states, &JOINT_ANGLES, "states");
        assert_default_close(&out.state_dots, &JOINT_RATES, "state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "state ddots");

        let beta = 1.0 - (-wc * filter_dt).exp();
        let mut angles = JOINT_ANGLES.to_vec();
        let mut rates = JOINT_RATES.to_vec();
        for step in 1..=2 {
            let out = fixture.update(step as f64 * filter_dt);
            let mut accels = vec![0.0; 3];
            for joint in 0..3 {
                let previous_angle = angles[joint];
                let previous_rate = rates[joint];
                angles[joint] = previous_angle + beta * (DES_JOINT_ANGLES[joint] - previous_angle);
                rates[joint] = (angles[joint] - previous_angle) / filter_dt;
                accels[joint] = (rates[joint] - previous_rate) / filter_dt;
            }
            assert_default_close(&out.states, &angles, "states");
            assert_default_close(&out.state_dots, &rates, "state dots");
            assert_default_close(&out.state_ddots, &accels, "state ddots");
        }
    }

    /// Upstream `test_joint_array_ref_profiler_filter_wrap_shortest_path`.
    #[test]
    fn low_pass_filter_converges_across_the_wrap_boundary() {
        let filter_dt = 0.1;
        let wc = 1.0;
        let wrap_start = -PI;
        let mut fixture = Fixture::new(JointProfile::LowPass {
            wc_radps: wc,
            filter_dt_s: filter_dt,
        });
        fixture.module.config.joint_angle_wrap_start_rad = Some(wrap_start);
        fixture.module.config.use_shortest_path = true;
        let joint_angles = [3.0, 0.0, 0.0];
        let des_joint_angles = [3.2, 0.0, 0.0];
        fixture.set_joints(&joint_angles, Some(&ZEROS));
        fixture.desired.write(desired(&des_joint_angles, &ZEROS));
        fixture.module.init();
        fixture.update(0.0);

        let wrapped_des = np_wrap(des_joint_angles[0], wrap_start);
        let target =
            joint_angles[0] + (wrapped_des - joint_angles[0] + PI).rem_euclid(2.0 * PI) - PI;
        let beta = 1.0 - (-wc * filter_dt).exp();
        let mut unwrapped = joint_angles[0];
        let mut rate = 0.0;
        for step in 1..=60 {
            let previous_angle = unwrapped;
            let previous_rate = rate;
            unwrapped += beta * (target - previous_angle);
            rate = (unwrapped - previous_angle) / filter_dt;
            let accel = (rate - previous_rate) / filter_dt;
            let expected_angle = np_wrap(unwrapped, wrap_start);

            let out = fixture.update(step as f64 * filter_dt);
            assert_default_close(&out.states[..1], &[expected_angle], "state");
            assert_default_close(&out.state_dots[..1], &[rate], "state dot");
            assert_default_close(&out.state_ddots[..1], &[accel], "state ddot");
        }
    }

    /// Upstream `test_joint_array_ref_profiler_linear`.
    #[test]
    fn linear_profile_ramps_to_the_target() {
        let mut fixture = Fixture::new(JointProfile::Linear {
            duration_s: PROFILE_DURATION,
        });
        fixture.module.init();
        let slope: Vec<f64> = (0..3)
            .map(|joint| (DES_JOINT_ANGLES[joint] - JOINT_ANGLES[joint]) / PROFILE_DURATION)
            .collect();

        let out = fixture.update(0.0);
        assert_default_close(&out.states, &JOINT_ANGLES, "start states");
        assert_default_close(&out.state_dots, &slope, "start state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "start state ddots");

        let mid_time = PROFILE_DURATION / 2.0;
        let out = fixture.update(mid_time);
        let mid: Vec<f64> = (0..3)
            .map(|joint| {
                JOINT_ANGLES[joint]
                    + (DES_JOINT_ANGLES[joint] - JOINT_ANGLES[joint]) * mid_time / PROFILE_DURATION
            })
            .collect();
        assert_default_close(&out.states, &mid, "mid states");
        assert_default_close(&out.state_dots, &slope, "mid state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "mid state ddots");

        let out = fixture.update(PROFILE_DURATION + 0.1);
        assert_default_close(&out.states, &DES_JOINT_ANGLES, "final states");
        assert_default_close(&out.state_dots, &ZEROS, "final state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "final state ddots");
    }

    /// Upstream `test_joint_array_ref_profiler_cubic`: multi-turn angles stay
    /// unwrapped with wrapping disabled (the default).
    #[test]
    fn cubic_profile_keeps_multi_turn_angles_unwrapped() {
        let mut fixture = Fixture::new(JointProfile::Cubic {
            duration_s: PROFILE_DURATION,
        });
        assert_eq!(fixture.module.config.joint_angle_wrap_start_rad, None);
        let turns_start = [8.0, -6.0, 10.0];
        let turns_des = [-8.0, 10.0, -6.0];
        let start: Vec<f64> = (0..3)
            .map(|j| JOINT_ANGLES[j] + turns_start[j] * PI)
            .collect();
        let des: Vec<f64> = (0..3)
            .map(|j| DES_JOINT_ANGLES[j] + turns_des[j] * PI)
            .collect();
        fixture.set_joints(&start, None);
        fixture.desired.write(desired(&des, &ZEROS));
        fixture.module.init();

        let out = fixture.update(0.0);
        let expected_accels: Vec<f64> = (0..3)
            .map(|j| {
                2.0 * ((3.0 * (des[j] - start[j])) / PROFILE_DURATION.powi(2)
                    - (2.0 * JOINT_RATES[j]) / PROFILE_DURATION)
            })
            .collect();
        assert_default_close(&out.states, &start, "start states");
        assert_default_close(&out.state_dots, &JOINT_RATES, "start state dots");
        assert_default_close(&out.state_ddots, &expected_accels, "start state ddots");

        let tau = PROFILE_DURATION / 2.0;
        let out = fixture.update(tau);
        let (angles, rates, accels) = per_joint(|j| cubic(start[j], des[j], JOINT_RATES[j], tau));
        assert_default_close(&out.states, &angles, "mid states");
        assert_default_close(&out.state_dots, &rates, "mid state dots");
        assert_default_close(&out.state_ddots, &accels, "mid state ddots");

        let out = fixture.update(PROFILE_DURATION + 0.1);
        assert_default_close(&out.states, &des, "final states");
        assert_default_close(&out.state_dots, &ZEROS, "final state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "final state ddots");
    }

    /// Upstream `test_joint_array_ref_profiler_cubic_joint_angle_wrap_range`:
    /// a wrap interval starting at zero, then wrapping disabled for a new
    /// command.
    #[test]
    fn cubic_profile_wraps_into_the_configured_interval() {
        let wrap_start = 0.0;
        let mut fixture = Fixture::new(JointProfile::Cubic {
            duration_s: PROFILE_DURATION,
        });
        fixture.module.config.joint_angle_wrap_start_rad = Some(wrap_start);
        let turns_start = [8.0, -6.0, 10.0];
        let turns_des = [-8.0, 10.0, -6.0];
        let start: Vec<f64> = (0..3)
            .map(|j| JOINT_ANGLES[j] + turns_start[j] * PI)
            .collect();
        let des: Vec<f64> = (0..3)
            .map(|j| DES_JOINT_ANGLES[j] + turns_des[j] * PI)
            .collect();
        fixture.set_joints(&start, None);
        fixture.desired.write(desired(&des, &ZEROS));
        let wrapped_start: Vec<f64> = start.iter().map(|&a| np_wrap(a, wrap_start)).collect();
        let wrapped_des: Vec<f64> = des.iter().map(|&a| np_wrap(a, wrap_start)).collect();
        fixture.module.init();

        let out = fixture.update(0.0);
        let expected_accels: Vec<f64> = (0..3)
            .map(|j| {
                2.0 * ((3.0 * (wrapped_des[j] - wrapped_start[j])) / PROFILE_DURATION.powi(2)
                    - (2.0 * JOINT_RATES[j]) / PROFILE_DURATION)
            })
            .collect();
        assert_default_close(&out.states, &wrapped_start, "start states");
        assert_default_close(&out.state_dots, &JOINT_RATES, "start state dots");
        assert_default_close(&out.state_ddots, &expected_accels, "start state ddots");

        let tau = PROFILE_DURATION / 2.0;
        let out = fixture.update(tau);
        let (angles, rates, accels) = per_joint(|j| {
            let (angle, rate, accel) = cubic(wrapped_start[j], wrapped_des[j], JOINT_RATES[j], tau);
            (np_wrap(angle, wrap_start), rate, accel)
        });
        assert_default_close(&out.states, &angles, "mid states");
        assert_default_close(&out.state_dots, &rates, "mid state dots");
        assert_default_close(&out.state_ddots, &accels, "mid state ddots");

        let out = fixture.update(PROFILE_DURATION + 0.1);
        assert_default_close(&out.states, &wrapped_des, "final states");
        assert_default_close(&out.state_dots, &ZEROS, "final state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "final state ddots");

        fixture.module.config.joint_angle_wrap_start_rad = None;
        let new_des: Vec<f64> = des.iter().map(|a| a + 0.1).collect();
        fixture.desired.write(desired(&new_des, &ZEROS));
        let out = fixture.update(PROFILE_DURATION + 0.2);
        assert_default_close(&out.states, &start, "unwrapped restart");
    }

    /// Upstream `test_joint_array_ref_profiler_cubic_shortest_path`.
    #[test]
    fn cubic_profile_takes_the_shortest_path_when_asked() {
        let degrees = PI / 180.0;
        let joint_angles = [-170.0 * degrees, 170.0 * degrees, -170.0 * degrees];
        let des_joint_angles = [170.0 * degrees, -170.0 * degrees, 170.0 * degrees];
        let configure = |use_shortest_path: bool| {
            let mut fixture = Fixture::new(JointProfile::Cubic {
                duration_s: PROFILE_DURATION,
            });
            fixture.module.config.use_shortest_path = use_shortest_path;
            fixture.set_joints(&joint_angles, Some(&ZEROS));
            fixture.desired.write(desired(&des_joint_angles, &ZEROS));
            fixture.module.init();
            fixture
        };
        let mut long_path = configure(false);
        let mut short_path = configure(true);
        long_path.update(0.0);
        short_path.update(0.0);

        let tau = PROFILE_DURATION / 2.0;
        let long_out = long_path.update(tau);
        let short_out = short_path.update(tau);
        let long_delta: Vec<f64> = (0..3)
            .map(|j| des_joint_angles[j] - joint_angles[j])
            .collect();
        let short_delta: Vec<f64> = long_delta
            .iter()
            .map(|d| (d + PI).rem_euclid(2.0 * PI) - PI)
            .collect();
        let fraction = tau / PROFILE_DURATION;
        let position_scale = 3.0 * fraction.powi(2) - 2.0 * fraction.powi(3);
        let rate_scale = (6.0 * fraction - 6.0 * fraction.powi(2)) / PROFILE_DURATION;
        let accel_scale = (6.0 - 12.0 * fraction) / PROFILE_DURATION.powi(2);
        let scaled = |scale: f64, delta: &[f64], offset: bool| -> Vec<f64> {
            (0..3)
                .map(|j| if offset { joint_angles[j] } else { 0.0 } + scale * delta[j])
                .collect()
        };
        assert_allclose(
            &long_out.states,
            &scaled(position_scale, &long_delta, true),
            1e-7,
            1e-12,
            "long path states",
        );
        assert_default_close(
            &short_out.states,
            &scaled(position_scale, &short_delta, true),
            "short path states",
        );
        assert_default_close(
            &long_out.state_dots,
            &scaled(rate_scale, &long_delta, false),
            "long path state dots",
        );
        assert_default_close(
            &short_out.state_dots,
            &scaled(rate_scale, &short_delta, false),
            "short path state dots",
        );
        assert_allclose(
            &long_out.state_ddots,
            &scaled(accel_scale, &long_delta, false),
            1e-7,
            1e-12,
            "long path state ddots",
        );
        assert_allclose(
            &short_out.state_ddots,
            &scaled(accel_scale, &short_delta, false),
            1e-7,
            1e-12,
            "short path state ddots",
        );

        let long_out = long_path.update(PROFILE_DURATION + 0.1);
        let short_out = short_path.update(PROFILE_DURATION + 0.1);
        assert_default_close(&long_out.states, &des_joint_angles, "long path final");
        assert_default_close(
            &short_out.states,
            &scaled(1.0, &short_delta, true),
            "short path final",
        );
        for out in [&long_out, &short_out] {
            assert_allclose(&out.state_dots, &ZEROS, 1e-7, 1e-12, "final state dots");
            assert_allclose(&out.state_ddots, &ZEROS, 1e-7, 1e-12, "final state ddots");
        }
    }

    /// Upstream `test_joint_array_ref_profiler_quintic`.
    #[test]
    fn quintic_profile_matches_its_coefficients() {
        let mut fixture = Fixture::new(JointProfile::Quintic {
            duration_s: PROFILE_DURATION,
        });
        fixture.module.init();

        let out = fixture.update(0.0);
        assert_default_close(&out.states, &JOINT_ANGLES, "start states");
        assert_default_close(&out.state_dots, &JOINT_RATES, "start state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "start state ddots");

        let tau = PROFILE_DURATION / 2.0;
        let out = fixture.update(tau);
        let (angles, rates, accels) =
            per_joint(|j| quintic(JOINT_ANGLES[j], DES_JOINT_ANGLES[j], JOINT_RATES[j], tau));
        assert_default_close(&out.states, &angles, "mid states");
        assert_default_close(&out.state_dots, &rates, "mid state dots");
        assert_default_close(&out.state_ddots, &accels, "mid state ddots");

        let out = fixture.update(PROFILE_DURATION + 0.1);
        assert_default_close(&out.states, &DES_JOINT_ANGLES, "final states");
        assert_default_close(&out.state_dots, &ZEROS, "final state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "final state ddots");
    }

    /// Upstream `test_joint_array_ref_profiler_filter_reinitializes_on_new_command`.
    #[test]
    fn low_pass_filter_restarts_from_the_joints_on_a_new_command() {
        let filter_dt = 0.1;
        let mut fixture = Fixture::new(JointProfile::LowPass {
            wc_radps: 2.0 * PI * 0.5,
            filter_dt_s: filter_dt,
        });
        fixture.module.init();
        fixture.update(0.0);
        fixture.update(filter_dt);

        let new_joint_angles = [0.3, -0.1, 0.8];
        let new_joint_rates = [-0.02, 0.06, -0.01];
        fixture.set_joints(&new_joint_angles, Some(&new_joint_rates));
        fixture.desired.write(desired(&[-0.4, 0.6, 0.9], &ZEROS));

        let out = fixture.update(2.0 * filter_dt);
        assert_default_close(&out.states, &new_joint_angles, "states");
        assert_default_close(&out.state_dots, &new_joint_rates, "state dots");
        assert_default_close(&out.state_ddots, &ZEROS, "state ddots");
    }

    /// Upstream `test_joint_array_ref_profiler_quintic_repeated_identical_writes`.
    #[test]
    fn rewriting_an_identical_command_does_not_restart_the_profile() {
        let mut fixture = Fixture::new(JointProfile::Quintic {
            duration_s: PROFILE_DURATION,
        });
        fixture.module.init();
        fixture.update(0.0);
        fixture.update(0.1);

        fixture.desired.write(desired(&DES_JOINT_ANGLES, &ZEROS));
        let second_tau = 0.2;
        let out = fixture.update(second_tau);
        let (angles, rates, accels) = per_joint(|j| {
            quintic(
                JOINT_ANGLES[j],
                DES_JOINT_ANGLES[j],
                JOINT_RATES[j],
                second_tau,
            )
        });
        assert_default_close(&out.states, &angles, "states");
        assert_default_close(&out.state_dots, &rates, "state dots");
        assert_default_close(&out.state_ddots, &accels, "state ddots");
    }

    /// A desired message with the wrong number of joints publishes nothing.
    #[test]
    fn a_misshapen_command_publishes_nothing() {
        let mut fixture = Fixture::new(JointProfile::Linear {
            duration_s: PROFILE_DURATION,
        });
        fixture.desired.write(desired(&[1.0, 2.0], &[0.0, 0.0]));
        fixture.module.init();
        let out = fixture.update(0.0);
        assert_eq!(out, JointArrayStateMsg::default());
    }

    #[test]
    #[should_panic(expected = "duration_s must be positive")]
    fn rejects_a_non_positive_profile_duration() {
        let _ = JointArrayRefProfiler::new(JointArrayRefProfilerConfig {
            name: "jointArrayRefProfiler".to_string(),
            num_hinged_joints: 1,
            profile: JointProfile::Cubic { duration_s: 0.0 },
            joint_angle_wrap_start_rad: None,
            use_shortest_path: false,
        });
    }
}
