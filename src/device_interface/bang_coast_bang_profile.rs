//! The scalar motion profile shared by
//! [`PrescribedRotation1Dof`](super::prescribed_rotation_1dof::PrescribedRotation1Dof)
//! and
//! [`PrescribedLinearTranslation`](super::prescribed_linear_translation::PrescribedLinearTranslation).
//!
//! Upstream `prescribedRotation1DOF` and `prescribedLinearTranslation` carry
//! the same profiler twice, once over an angle and once over a displacement;
//! it is written once here over a generic coordinate `x`. A profile moves `x`
//! from rest at `x_0` to rest at the reference `x_r` with acceleration
//! magnitude `a_max`, in one of four shapes chosen by two durations:
//!
//! | `coast_option_bang_duration_s` | `smoothing_duration_s` | shape |
//! |---|---|---|
//! | 0 | 0 | bang-bang: `+a_max` for half the time, `−a_max` for the rest |
//! | `t_b` > 0 | 0 | bang-coast-bang: `±a_max` for `t_b`, coasting between |
//! | 0 | `t_s` > 0 | smoothed bang-bang: cubic acceleration ramps of `t_s` around the bangs |
//! | `t_b` > 0 | `t_s` > 0 | smoothed bang-coast-bang |
//!
//! The ramps make the smoothed shapes start and end at zero acceleration.
//! Formulas, segment boundaries and their open/closed ends are upstream's.

/// Shape parameters of a profile, read when a profile starts and on every
/// evaluation (upstream reads its members each time, so a change between
/// updates takes effect at once).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct ProfileShape {
    /// Upstream `coastOptionBangDuration` \[s\]: bang duration of the coast
    /// option; zero selects the bang-bang shapes.
    pub coast_option_bang_duration_s: f64,
    /// Upstream `smoothingDuration` \[s\]: acceleration ramp duration; zero
    /// selects the unsmoothed shapes.
    pub smoothing_duration_s: f64,
    /// Upstream `thetaDDotMax` / `transAccelMax`: bang acceleration magnitude.
    pub accel_max: f64,
}

impl ProfileShape {
    fn coasts(&self) -> bool {
        self.coast_option_bang_duration_s > 0.0
    }

    fn smooths(&self) -> bool {
        self.smoothing_duration_s > 0.0
    }
}

/// State of the profiler: the current coordinate, the boundary states and
/// times of the profile in progress, and whether it has completed.
#[derive(Clone, Debug, Default)]
pub(crate) struct BangCoastBangProfile {
    /// Upstream `thetaInit` / `transPosInit`: start of the current profile.
    pub(crate) init: f64,
    /// Upstream `thetaRef` / `transPosRef`: end of the current profile.
    reference: f64,
    /// Upstream `theta` / `transPos`: current coordinate.
    pub(crate) pos: f64,
    /// Upstream `thetaDot` / `transVel`: current rate.
    pub(crate) vel: f64,
    /// Upstream `thetaDDot` / `transAccel`: current acceleration.
    pub(crate) accel: f64,
    pos_tb1: f64,
    vel_tb1: f64,
    pos_tb2: f64,
    vel_tb2: f64,
    pos_ts1: f64,
    vel_ts1: f64,
    pos_ts2: f64,
    vel_ts2: f64,
    pos_ts3: f64,
    vel_ts3: f64,
    pos_tc: f64,
    vel_tc: f64,
    /// Upstream `tInit` \[s\]: start time of the current profile.
    t_init: f64,
    t_b1: f64,
    t_b2: f64,
    t_s1: f64,
    t_s2: f64,
    t_s3: f64,
    t_c: f64,
    /// Upstream `t_f` \[s\]: end time of the current profile.
    t_f: f64,
    /// Upstream `convergence`: the profile in progress has completed, so the
    /// next update starts a new one.
    converged: bool,
    /// Parabola constants of the unsmoothed bang segments.
    a: f64,
    b: f64,
}

impl BangCoastBangProfile {
    /// Upstream `Reset`: at rest at `initial`, ready to start a profile on
    /// the next update.
    pub(crate) fn reset(&mut self, initial: f64) {
        self.init = initial;
        self.t_init = 0.0;
        self.pos = initial;
        self.vel = 0.0;
        self.converged = true;
    }

    /// Upstream `UpdateState` without the messages: starts a profile from the
    /// current coordinate to `reference` if the last one has completed, then
    /// evaluates the profile at `t_s`.
    pub(crate) fn update(&mut self, shape: &ProfileShape, t_s: f64, reference: f64) {
        if self.converged {
            self.t_init = t_s;
            self.init = self.pos;
            self.reference = reference;
            if self.init != self.reference {
                self.compute_parameters(shape);
            } else {
                self.t_f = self.t_init;
            }
            self.converged = false;
        }
        self.compute_current_state(shape, t_s);
    }

    /// `(x_r − x_0) / |x_r − x_0|`, upstream's direction of motion.
    fn sign(&self) -> f64 {
        let delta = self.reference - self.init;
        delta / delta.abs()
    }

    /// Whether the profile in progress moves at all.
    fn moves(&self) -> bool {
        self.t_f - self.t_init != 0.0
    }

    /// Upstream `computeRotationParameters` / `computeTranslationParameters`.
    fn compute_parameters(&mut self, shape: &ProfileShape) {
        match (shape.coasts(), shape.smooths()) {
            (true, true) => self.smoothed_bang_coast_bang_parameters(shape),
            (true, false) => self.bang_coast_bang_parameters(shape),
            (false, true) => self.smoothed_bang_bang_parameters(shape),
            (false, false) => self.bang_bang_parameters(shape),
        }
    }

    fn bang_bang_parameters(&mut self, shape: &ProfileShape) {
        let total_time =
            (((0.5 * (self.reference - self.init).abs()) * 8.0) / shape.accel_max).sqrt();
        self.t_f = self.t_init + total_time;
        self.t_b1 = self.t_init + (total_time / 2.0);
        self.a = 0.5 * (self.reference - self.init)
            / ((self.t_b1 - self.t_init) * (self.t_b1 - self.t_init));
        self.b =
            -0.5 * (self.reference - self.init) / ((self.t_b1 - self.t_f) * (self.t_b1 - self.t_f));
    }

    fn bang_coast_bang_parameters(&mut self, shape: &ProfileShape) {
        let sign = self.sign();
        let t_bang = shape.coast_option_bang_duration_s;
        self.t_b1 = self.t_init + t_bang;
        self.pos_tb1 = sign * 0.5 * shape.accel_max * t_bang * t_bang + self.init;
        self.vel_tb1 = sign * shape.accel_max * t_bang;

        let delta_coast = self.reference - self.init - 2.0 * (self.pos_tb1 - self.init);
        let coast_duration = (delta_coast / self.vel_tb1).abs();
        self.t_c = self.t_b1 + coast_duration;
        // Upstream shadows the member `theta_tc` with a local here.
        let pos_tc = self.pos_tb1 + delta_coast;
        self.t_f = self.t_c + t_bang;

        self.a =
            (self.pos_tb1 - self.init) / ((self.t_b1 - self.t_init) * (self.t_b1 - self.t_init));
        self.b = -(self.reference - pos_tc) / ((self.t_c - self.t_f) * (self.t_c - self.t_f));
    }

    fn smoothed_bang_bang_parameters(&mut self, shape: &ProfileShape) {
        let sign = self.sign();
        let accel_max = shape.accel_max;
        let t_s = shape.smoothing_duration_s;

        self.t_s1 = self.t_init + t_s;
        self.vel_ts1 = sign * 0.5 * accel_max * t_s;
        self.pos_ts1 = sign * (3.0 / 20.0) * accel_max * t_s * t_s + self.init;

        // Bang duration: the positive root of the quadratic that puts the
        // midpoint of the profile halfway through the second ramp.
        let a_term = sign * 0.5 * accel_max;
        let b_term = (sign * accel_max * t_s + self.vel_ts1) / a_term;
        let c_term =
            (sign * (2.0 / 5.0) * accel_max * t_s * t_s + self.vel_ts1 * t_s + self.pos_ts1
                - 0.5 * (self.reference + self.init))
                / a_term;
        let bang_duration = (-b_term + (b_term * b_term - 4.0 * c_term).sqrt()) / 2.0;

        self.t_b1 = self.t_s1 + bang_duration;
        self.vel_tb1 = sign * accel_max * bang_duration + self.vel_ts1;
        self.pos_tb1 = sign * 0.5 * accel_max * bang_duration * bang_duration
            + self.vel_ts1 * bang_duration
            + self.pos_ts1;

        self.t_s2 = self.t_b1 + 2.0 * t_s;
        self.vel_ts2 = self.vel_tb1;
        self.pos_ts2 =
            sign * (4.0 / 5.0) * accel_max * t_s * t_s + self.vel_tb1 * 2.0 * t_s + self.pos_tb1;

        self.t_b2 = self.t_s2 + bang_duration;
        self.vel_tb2 = -sign * accel_max * bang_duration + self.vel_ts2;
        self.pos_tb2 = -sign * 0.5 * accel_max * bang_duration * bang_duration
            + self.vel_ts2 * bang_duration
            + self.pos_ts2;

        self.t_f = self.t_b2 + t_s;
    }

    fn smoothed_bang_coast_bang_parameters(&mut self, shape: &ProfileShape) {
        let sign = self.sign();
        let accel_max = shape.accel_max;
        let t_s = shape.smoothing_duration_s;
        let t_bang = shape.coast_option_bang_duration_s;

        self.t_s1 = self.t_init + t_s;
        self.vel_ts1 = sign * 0.5 * accel_max * t_s;
        self.pos_ts1 = sign * (3.0 / 20.0) * accel_max * t_s * t_s + self.init;

        self.t_b1 = self.t_s1 + t_bang;
        self.vel_tb1 = sign * accel_max * t_bang + self.vel_ts1;
        self.pos_tb1 =
            sign * 0.5 * accel_max * t_bang * t_bang + self.vel_ts1 * t_bang + self.pos_ts1;

        self.t_s2 = self.t_b1 + t_s;
        self.vel_ts2 = sign * 0.5 * accel_max * t_s + self.vel_tb1;
        self.pos_ts2 =
            sign * (7.0 / 20.0) * accel_max * t_s * t_s + self.vel_tb1 * t_s + self.pos_tb1;

        let delta_coast = (self.reference - self.init) - 2.0 * (self.pos_ts2 - self.init);
        self.t_c = (delta_coast / self.vel_ts2) + self.t_s2;
        self.vel_tc = self.vel_ts2;
        self.pos_tc = self.vel_ts2 * (self.t_c - self.t_s2) + self.pos_ts2;

        self.t_s3 = self.t_c + t_s;
        self.vel_ts3 = -sign * 0.5 * accel_max * t_s + self.vel_tc;
        self.pos_ts3 =
            -sign * (3.0 / 20.0) * accel_max * t_s * t_s + self.vel_tc * t_s + self.pos_tc;

        self.t_b2 = self.t_s3 + t_bang;
        self.vel_tb2 = -sign * accel_max * t_bang + self.vel_ts3;
        self.pos_tb2 =
            -sign * 0.5 * accel_max * t_bang * t_bang + self.vel_ts3 * t_bang + self.pos_ts3;

        self.t_f = self.t_b2 + t_s;
    }

    /// Upstream `computeCurrentState`: picks the segment `t` falls in.
    fn compute_current_state(&mut self, shape: &ProfileShape, t: f64) {
        match (shape.coasts(), shape.smooths()) {
            (true, true) => {
                if self.in_first_smoothed_segment(t) {
                    self.first_smoothed_segment(shape, t);
                } else if self.in_first_bang_segment(shape, t) {
                    self.first_bang_segment(shape, t);
                } else if self.in_second_smoothed_segment(t) {
                    self.second_smoothed_segment(shape, t);
                } else if self.in_coast_segment(shape, t) {
                    self.coast_segment(shape, t);
                } else if self.in_third_smoothed_segment(shape, t) {
                    self.third_smoothed_segment(shape, t);
                } else if self.in_second_bang_segment(shape, t) {
                    self.second_bang_segment(shape, t);
                } else if self.in_fourth_smoothed_segment(t) {
                    self.fourth_smoothed_segment(shape, t);
                } else {
                    self.complete();
                }
            }
            (true, false) => {
                if self.in_first_bang_segment(shape, t) {
                    self.first_bang_segment(shape, t);
                } else if self.in_coast_segment(shape, t) {
                    self.coast_segment(shape, t);
                } else if self.in_second_bang_segment(shape, t) {
                    self.second_bang_segment(shape, t);
                } else {
                    self.complete();
                }
            }
            (false, true) => {
                if self.in_first_smoothed_segment(t) {
                    self.first_smoothed_segment(shape, t);
                } else if self.in_first_bang_segment(shape, t) {
                    self.first_bang_segment(shape, t);
                } else if self.in_second_smoothed_segment(t) {
                    self.second_smoothed_segment(shape, t);
                } else if self.in_second_bang_segment(shape, t) {
                    self.second_bang_segment(shape, t);
                } else if self.in_third_smoothed_segment(shape, t) {
                    self.third_smoothed_segment(shape, t);
                } else {
                    self.complete();
                }
            }
            (false, false) => {
                if self.in_first_bang_segment(shape, t) {
                    self.first_bang_segment(shape, t);
                } else if self.in_second_bang_segment(shape, t) {
                    self.second_bang_segment(shape, t);
                } else {
                    self.complete();
                }
            }
        }
    }

    fn in_first_bang_segment(&self, shape: &ProfileShape, t: f64) -> bool {
        if shape.smooths() {
            t > self.t_s1 && t <= self.t_b1 && self.moves()
        } else {
            t <= self.t_b1 && self.moves()
        }
    }

    fn in_second_bang_segment(&self, shape: &ProfileShape, t: f64) -> bool {
        match (shape.coasts(), shape.smooths()) {
            (true, true) => t > self.t_s3 && t <= self.t_b2 && self.moves(),
            (true, false) => t > self.t_c && t <= self.t_f && self.moves(),
            (false, true) => t > self.t_s2 && t <= self.t_b2 && self.moves(),
            (false, false) => t > self.t_b1 && t <= self.t_f && self.moves(),
        }
    }

    fn in_first_smoothed_segment(&self, t: f64) -> bool {
        t <= self.t_s1 && self.moves()
    }

    fn in_second_smoothed_segment(&self, t: f64) -> bool {
        t > self.t_b1 && t <= self.t_s2 && self.moves()
    }

    fn in_third_smoothed_segment(&self, shape: &ProfileShape, t: f64) -> bool {
        if shape.coasts() {
            t > self.t_c && t <= self.t_s3 && self.moves()
        } else {
            t > self.t_b2 && t <= self.t_f && self.moves()
        }
    }

    fn in_fourth_smoothed_segment(&self, t: f64) -> bool {
        t > self.t_b2 && t <= self.t_f && self.moves()
    }

    fn in_coast_segment(&self, shape: &ProfileShape, t: f64) -> bool {
        if shape.smooths() {
            t > self.t_s2 && t <= self.t_c && self.moves()
        } else {
            t > self.t_b1 && t <= self.t_c && self.moves()
        }
    }

    fn first_bang_segment(&mut self, shape: &ProfileShape, t: f64) {
        self.accel = self.sign() * shape.accel_max;
        if shape.smooths() {
            let dt = t - self.t_s1;
            self.vel = self.accel * dt + self.vel_ts1;
            self.pos = 0.5 * self.accel * dt * dt + self.vel_ts1 * dt + self.pos_ts1;
        } else {
            self.vel = self.accel * (t - self.t_init);
            self.pos = self.a * (t - self.t_init) * (t - self.t_init) + self.init;
        }
    }

    fn second_bang_segment(&mut self, shape: &ProfileShape, t: f64) {
        self.accel = -self.sign() * shape.accel_max;
        if shape.smooths() {
            let (t_start, vel_start, pos_start) = if shape.coasts() {
                (self.t_s3, self.vel_ts3, self.pos_ts3)
            } else {
                (self.t_s2, self.vel_ts2, self.pos_ts2)
            };
            let dt = t - t_start;
            self.vel = self.accel * dt + vel_start;
            self.pos = 0.5 * self.accel * dt * dt + vel_start * dt + pos_start;
        } else {
            self.vel = self.accel * (t - self.t_f);
            self.pos = self.b * (t - self.t_f) * (t - self.t_f) + self.reference;
        }
    }

    fn first_smoothed_segment(&mut self, shape: &ProfileShape, t: f64) {
        let sign = self.sign();
        let t_s = shape.smoothing_duration_s;
        let dt = t - self.t_init;

        let term1 = (3.0 * dt * dt) / (t_s * t_s);
        let term2 = (2.0 * dt * dt * dt) / (t_s * t_s * t_s);
        let term3 = (dt * dt * dt) / (t_s * t_s);
        let term4 = (dt * dt * dt * dt) / (2.0 * t_s * t_s * t_s);
        let term5 = (dt * dt * dt * dt) / (4.0 * t_s * t_s);
        let term6 = (dt * dt * dt * dt * dt) / (10.0 * t_s * t_s * t_s);

        self.accel = sign * shape.accel_max * (term1 - term2);
        self.vel = sign * shape.accel_max * (term3 - term4);
        self.pos = sign * shape.accel_max * (term5 - term6) + self.init;
    }

    fn second_smoothed_segment(&mut self, shape: &ProfileShape, t: f64) {
        let sign = self.sign();
        let t_s = shape.smoothing_duration_s;
        let dt = t - self.t_b1;

        let (term1, term2, term3, term4, term5, term6, term7) = if shape.coasts() {
            (
                (3.0 * dt * dt) / (t_s * t_s),
                (2.0 * dt * dt * dt) / (t_s * t_s * t_s),
                (dt * dt * dt) / (t_s * t_s),
                (dt * dt * dt * dt) / (2.0 * t_s * t_s * t_s),
                0.5 * dt * dt,
                (dt * dt * dt * dt) / (4.0 * t_s * t_s),
                (dt * dt * dt * dt * dt) / (10.0 * t_s * t_s * t_s),
            )
        } else {
            (
                (3.0 * dt * dt) / (2.0 * t_s * t_s),
                (dt * dt * dt) / (2.0 * t_s * t_s * t_s),
                (dt * dt * dt) / (2.0 * t_s * t_s),
                (dt * dt * dt * dt) / (8.0 * t_s * t_s * t_s),
                0.5 * dt * dt,
                (dt * dt * dt * dt) / (8.0 * t_s * t_s),
                (dt * dt * dt * dt * dt) / (40.0 * t_s * t_s * t_s),
            )
        };

        self.accel = sign * shape.accel_max * (1.0 - term1 + term2);
        self.vel = sign * shape.accel_max * (dt - term3 + term4) + self.vel_tb1;
        self.pos =
            sign * shape.accel_max * (term5 - term6 + term7) + self.vel_tb1 * dt + self.pos_tb1;
    }

    fn third_smoothed_segment(&mut self, shape: &ProfileShape, t: f64) {
        let sign = self.sign();
        let t_s = shape.smoothing_duration_s;

        if shape.coasts() {
            let dt = t - self.t_c;
            let term1 = (3.0 * dt * dt) / (t_s * t_s);
            let term2 = (2.0 * dt * dt * dt) / (t_s * t_s * t_s);
            let term3 = (dt * dt * dt) / (t_s * t_s);
            let term4 = (dt * dt * dt * dt) / (2.0 * t_s * t_s * t_s);
            let term5 = (dt * dt * dt * dt) / (4.0 * t_s * t_s);
            let term6 = (dt * dt * dt * dt * dt) / (10.0 * t_s * t_s * t_s);

            self.accel = -sign * shape.accel_max * (term1 - term2);
            self.vel = -sign * shape.accel_max * (term3 - term4) + self.vel_tc;
            self.pos = -sign * shape.accel_max * (term5 - term6) + self.vel_tc * dt + self.pos_tc;
        } else {
            let dt = t - self.t_b2;
            let term1 = (3.0 * dt * dt) / (t_s * t_s);
            let term2 = (2.0 * dt * dt * dt) / (t_s * t_s * t_s);
            let term3 = (dt * dt * dt) / (t_s * t_s);
            let term4 = (dt * dt * dt * dt) / (2.0 * t_s * t_s * t_s);
            let term5 = -0.5 * dt * dt;
            let term6 = (dt * dt * dt * dt) / (4.0 * t_s * t_s);
            let term7 = (dt * dt * dt * dt * dt) / (10.0 * t_s * t_s * t_s);

            self.accel = sign * shape.accel_max * (-1.0 + term1 - term2);
            self.vel = sign * shape.accel_max * (-dt + term3 - term4) + self.vel_tb2;
            self.pos =
                sign * shape.accel_max * (term5 + term6 - term7) + self.vel_tb2 * dt + self.pos_tb2;
        }
    }

    fn fourth_smoothed_segment(&mut self, shape: &ProfileShape, t: f64) {
        let t_s = shape.smoothing_duration_s;
        let dt = self.t_f - t;

        let term1 = (3.0 * dt * dt) / (t_s * t_s);
        let term2 = (2.0 * dt * dt * dt) / (t_s * t_s * t_s);
        let term3 = (dt * dt * dt) / (t_s * t_s);
        let term4 = (dt * dt * dt * dt) / (2.0 * t_s * t_s * t_s);
        let term5 = (dt * dt * dt * dt) / (4.0 * t_s * t_s);
        let term6 = (dt * dt * dt * dt * dt) / (10.0 * t_s * t_s * t_s);

        let sign = self.sign();
        self.accel = -sign * shape.accel_max * (term1 - term2);
        self.vel = sign * shape.accel_max * (term3 - term4);
        self.pos = -sign * shape.accel_max * (term5 - term6) + self.reference;
    }

    fn coast_segment(&mut self, shape: &ProfileShape, t: f64) {
        self.accel = 0.0;
        if shape.smooths() {
            self.vel = self.vel_ts2;
            self.pos = self.vel_ts2 * (t - self.t_s2) + self.pos_ts2;
        } else {
            self.vel = self.vel_tb1;
            self.pos = self.vel_tb1 * (t - self.t_b1) + self.pos_tb1;
        }
    }

    /// Upstream `computeRotationComplete` / `computeTranslationComplete`.
    fn complete(&mut self) {
        self.accel = 0.0;
        self.vel = 0.0;
        self.pos = self.reference;
        self.converged = true;
    }
}

#[cfg(test)]
pub(crate) mod test_support {
    //! The parameter grid and checks shared by the upstream
    //! `test_prescribedRotation1DOF.py` and
    //! `test_prescribedLinearTranslation.py`, which differ only in names and
    //! values: two successive profiles, 300 s each at 0.1 s.

    /// Upstream `testTimeStepSec` \[s\] in nanoseconds.
    pub(crate) const STEP_NANOS: u64 = 100_000_000;
    /// Upstream `simTime` \[s\]: duration of each profile segment.
    pub(crate) const SEGMENT_S: u64 = 300;

    /// One logged sample: coordinate, rate and acceleration.
    #[derive(Clone, Copy, Debug)]
    pub(crate) struct Sample {
        pub(crate) pos: f64,
        pub(crate) vel: f64,
        pub(crate) accel: f64,
    }

    /// The pytest's timeline: `step(t_nanos, reference)` at every tick from
    /// 0 to 600 s, with the first reference through 300 s and the second after.
    pub(crate) fn two_segment_run(
        reference_1: f64,
        reference_2: f64,
        mut step: impl FnMut(u64, f64) -> Sample,
    ) -> Vec<Sample> {
        let ticks_per_segment = SEGMENT_S * 1_000_000_000 / STEP_NANOS;
        (0..=2 * ticks_per_segment)
            .map(|tick| {
                let reference = if tick <= ticks_per_segment {
                    reference_1
                } else {
                    reference_2
                };
                step(tick * STEP_NANOS, reference)
            })
            .collect()
    }

    /// The pytest's checks, in the units it applies them in (`scale` converts
    /// the coordinate into them): the coordinate equals each reference at the
    /// end of its segment within `accuracy`, and for the smoothed shapes the
    /// forward differences of the coordinate and rate match the logged rate
    /// and acceleration within 1e-2.
    pub(crate) fn check_two_segment_run(
        samples: &[Sample],
        references: [f64; 2],
        smoothed: bool,
        scale: f64,
        accuracy: f64,
        what: &str,
    ) {
        let first_final_index = (SEGMENT_S * 1_000_000_000 / STEP_NANOS) as usize + 1;
        let finals = [
            samples[first_final_index].pos,
            samples[samples.len() - 1].pos,
        ];
        for (actual, reference) in finals.iter().zip(references) {
            assert!(
                (scale * actual - scale * reference).abs() <= accuracy,
                "{what}: final {actual} vs reference {reference}"
            );
        }

        if smoothed {
            let dt_s = STEP_NANOS as f64 * 1e-9;
            for (index, pair) in samples.windows(2).enumerate() {
                let accel_numerical = (scale * pair[1].vel - scale * pair[0].vel) / dt_s;
                let vel_numerical = (scale * pair[1].pos - scale * pair[0].pos) / dt_s;
                assert!(
                    (scale * pair[0].accel - accel_numerical).abs() <= 1e-2,
                    "{what}: acceleration at sample {index}: {} vs {accel_numerical}",
                    scale * pair[0].accel
                );
                assert!(
                    (scale * pair[0].vel - vel_numerical).abs() <= 1e-2,
                    "{what}: rate at sample {index}: {} vs {vel_numerical}",
                    scale * pair[0].vel
                );
            }
        }
    }
}
