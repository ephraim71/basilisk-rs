//! Fixed arc, line and hold translation reference: port of upstream's
//! pure-Python `fswAlgorithms/transGuidance/arcLineHoldRef/arcLineHoldRef.py`.
//!
//! The reference follows a circular arc with equal inertial `x` and `z`
//! components (45° out of the inertial `x-y` plane) at the rate
//! `ω = π / (t_circle + t_transition)`, blends into a line with a quintic
//! that matches position, velocity and acceleration at both ends,
//! decelerates along the inertial `y` axis to the final position, and holds
//! there:
//!
//! ```text
//! circle  r = R (sin ωt / √2, cos ωt, sin ωt / √2),   t < t_circle
//! blend   quintic to r_blend, v = ω R ŷ, a = 0,        t < t_circle + t_transition
//! line    y quintic to r_final at rest,                t < ... + t_linear
//! hold    r = r_final
//! ```
//!
//! Upstream raises `BasiliskError` at `Reset` for non-finite parameters,
//! non-positive durations and endpoints off the `y` axis; here
//! [`ArcLineHoldRefConfig::validate`] reports them and `init` panics.

#![allow(dead_code)]

use std::f64::consts::{FRAC_1_SQRT_2, PI};

use basilisk_rs::messages::{Output, TranslationReferenceMsg};
use basilisk_rs::{Module, SimulationContext};
use nalgebra::Vector3;

/// Position, velocity and acceleration of the reference.
pub type States = (Vector3<f64>, Vector3<f64>, Vector3<f64>);

/// User-settable parameters of [`ArcLineHoldRef`]: upstream's setters.
#[derive(Clone, Debug, PartialEq)]
pub struct ArcLineHoldRefConfig {
    /// Upstream `ModelTag`.
    pub name: String,
    /// Upstream `radius` \[m\]; default 20 m.
    pub radius_m: f64,
    /// Upstream `finalPosition` \[m\]; default (0, -5, 0) m.
    pub final_position_m: Vector3<f64>,
    /// Upstream `blendEndPosition` \[m\]; default (0, -16, 0) m.
    pub blend_end_position_m: Vector3<f64>,
    /// Upstream `tCircle`, `tTransition`, `tLinear` \[s\]; defaults 5 h, 1 h
    /// and 110 min.
    pub t_circle_s: f64,
    pub t_transition_s: f64,
    pub t_linear_s: f64,
}

impl Default for ArcLineHoldRefConfig {
    fn default() -> Self {
        Self {
            name: "arcLineHoldRef".to_string(),
            radius_m: 20.0,
            final_position_m: Vector3::new(0.0, -5.0, 0.0),
            blend_end_position_m: Vector3::new(0.0, -16.0, 0.0),
            t_circle_s: 5.0 * 3600.0,
            t_transition_s: 3600.0,
            t_linear_s: 110.0 * 60.0,
        }
    }
}

impl ArcLineHoldRefConfig {
    /// Upstream `Reset`'s checks, in upstream's order.
    pub fn validate(&self) -> Result<(), String> {
        for (name, value) in [
            ("radius", self.radius_m),
            ("tCircle", self.t_circle_s),
            ("tTransition", self.t_transition_s),
            ("tLinear", self.t_linear_s),
        ] {
            if !value.is_finite() {
                return Err(format!("ArcLineHoldRef.{name} must be finite."));
            }
        }
        for (name, endpoint) in [
            ("blendEndPosition", self.blend_end_position_m),
            ("finalPosition", self.final_position_m),
        ] {
            if endpoint.iter().any(|value| !value.is_finite()) {
                return Err(format!(
                    "ArcLineHoldRef.{name} must be a finite three-component position."
                ));
            }
            if endpoint.x != 0.0 || endpoint.z != 0.0 {
                return Err(format!(
                    "ArcLineHoldRef.{name} must have zero inertial x and z components because \
                     the deceleration and final hold lie on the y axis."
                ));
            }
        }
        if self.t_transition_s <= 0.0 || self.t_linear_s <= 0.0 {
            return Err(
                "ArcLineHoldRef requires positive transition and deceleration durations.".into(),
            );
        }
        if self.t_circle_s + self.t_transition_s <= 0.0 {
            return Err(
                "ArcLineHoldRef requires a positive circular-plus-transition duration.".into(),
            );
        }
        Ok(())
    }
}

/// Upstream `ArcLineHoldRef`.
#[derive(Debug)]
pub struct ArcLineHoldRef {
    pub config: ArcLineHoldRefConfig,
    /// Upstream `transRefOutMsg`.
    pub trans_ref_out_msg: Output<TranslationReferenceMsg>,
    omega_radps: f64,
    blend_start: States,
    blend_end: States,
}

impl ArcLineHoldRef {
    pub fn new(config: ArcLineHoldRefConfig) -> Self {
        let mut module = Self {
            config,
            trans_ref_out_msg: Output::default(),
            omega_radps: 0.0,
            blend_start: (Vector3::zeros(), Vector3::zeros(), Vector3::zeros()),
            blend_end: (Vector3::zeros(), Vector3::zeros(), Vector3::zeros()),
        };
        module.prepare();
        module
    }

    /// Upstream `Reset`'s boundary states.
    fn prepare(&mut self) {
        let config = &self.config;
        self.omega_radps = PI / (config.t_circle_s + config.t_transition_s);
        self.blend_start = self.circle_states(config.t_circle_s);
        self.blend_end = (
            config.blend_end_position_m,
            Vector3::y() * (self.omega_radps * config.radius_m),
            Vector3::zeros(),
        );
    }

    /// Upstream `circleStates`.
    pub fn circle_states(&self, time_s: f64) -> States {
        let (omega, radius) = (self.omega_radps, self.config.radius_m);
        let (sin, cos) = (omega * time_s).sin_cos();
        let position = Vector3::new(
            radius * FRAC_1_SQRT_2 * sin,
            radius * cos,
            radius * FRAC_1_SQRT_2 * sin,
        );
        let velocity = Vector3::new(
            radius * FRAC_1_SQRT_2 * cos,
            -radius * sin,
            radius * FRAC_1_SQRT_2 * cos,
        ) * omega;
        (position, velocity, position * -(omega * omega))
    }

    /// Upstream `blendStates`: the quintic transition, `elapsed_s` after it
    /// starts.
    pub fn blend_states(&self, elapsed_s: f64) -> States {
        let duration = self.config.t_transition_s;
        let (r0, v0, a0) = self.blend_start;
        let (r1, v1, a1) = self.blend_end;
        let mut states = (Vector3::zeros(), Vector3::zeros(), Vector3::zeros());
        let t = elapsed_s;
        for index in 0..3 {
            let (p0, p1, v0, v1, a0, a1) = (
                r0[index], r1[index], v0[index], v1[index], a0[index], a1[index],
            );
            let d = duration;
            let c0 = p0;
            let c1 = v0;
            let c2 = a0 / 2.0;
            let c3 = (-3.0 * a0 * d * d + a1 * d * d - 12.0 * v0 * d - 8.0 * v1 * d - 20.0 * p0
                + 20.0 * p1)
                / (2.0 * d.powi(3));
            let c4 = (1.5 * a0 * d * d - a1 * d * d + 8.0 * d * v0 + 7.0 * d * v1 + 15.0 * p0
                - 15.0 * p1)
                / d.powi(4);
            let c5 = (-a0 * d * d + a1 * d * d - 6.0 * d * v0 - 6.0 * d * v1 - 12.0 * p0
                + 12.0 * p1)
                / (2.0 * d.powi(5));
            states.0[index] =
                c0 + c1 * t + c2 * t.powi(2) + c3 * t.powi(3) + c4 * t.powi(4) + c5 * t.powi(5);
            states.1[index] = c1
                + 2.0 * c2 * t
                + 3.0 * c3 * t.powi(2)
                + 4.0 * c4 * t.powi(3)
                + 5.0 * c5 * t.powi(4);
            states.2[index] =
                2.0 * c2 + 6.0 * c3 * t + 12.0 * c4 * t.powi(2) + 20.0 * c5 * t.powi(3);
        }
        states
    }

    /// Upstream `decelStates`: the `y`-axis deceleration, `elapsed_s` after
    /// it starts.
    pub fn decel_states(&self, elapsed_s: f64) -> States {
        let duration = self.config.t_linear_s;
        let start_y = self.blend_end.0.y;
        let speed = self.blend_end.1.y;
        let change = self.config.final_position_m.y - start_y;
        let c3 = (10.0 * change - 6.0 * speed * duration) / duration.powi(3);
        let c4 = (-15.0 * change + 8.0 * speed * duration) / duration.powi(4);
        let c5 = (6.0 * change - 3.0 * speed * duration) / duration.powi(5);
        let t = elapsed_s;
        (
            Vector3::y() * (start_y + speed * t + c3 * t.powi(3) + c4 * t.powi(4) + c5 * t.powi(5)),
            Vector3::y()
                * (speed + 3.0 * c3 * t.powi(2) + 4.0 * c4 * t.powi(3) + 5.0 * c5 * t.powi(4)),
            Vector3::y() * (6.0 * c3 * t + 12.0 * c4 * t.powi(2) + 20.0 * c5 * t.powi(3)),
        )
    }

    /// The reference at `time_s` (upstream `UpdateState`'s phase choice).
    pub fn states_at(&self, time_s: f64) -> States {
        let config = &self.config;
        let blend_start = config.t_circle_s;
        let decel_start = blend_start + config.t_transition_s;
        if time_s < blend_start {
            self.circle_states(time_s)
        } else if time_s < decel_start {
            self.blend_states(time_s - blend_start)
        } else if time_s < decel_start + config.t_linear_s {
            self.decel_states(time_s - decel_start)
        } else {
            (config.final_position_m, Vector3::zeros(), Vector3::zeros())
        }
    }
}

impl Module for ArcLineHoldRef {
    fn init(&mut self) {
        if let Err(error) = self.config.validate() {
            panic!("{error}");
        }
        self.prepare();
    }

    fn update(&mut self, context: &SimulationContext) {
        let (position_m, velocity_mps, acceleration_mps2) =
            self.states_at(context.current_sim_nanos as f64 * 1e-9);
        self.trans_ref_out_msg.write(TranslationReferenceMsg {
            position_m,
            velocity_mps,
            acceleration_mps2,
        });
    }
}

#[cfg(test)]
mod tests {
    //! Port of upstream `arcLineHoldRef/_UnitTest/test_arcLineHoldRef.py`.

    use super::*;
    use hifitime::Epoch;

    /// Upstream `_configured_module`: a short maneuver.
    fn configured() -> ArcLineHoldRef {
        let mut module = ArcLineHoldRef::new(ArcLineHoldRefConfig {
            radius_m: 6.0,
            t_circle_s: 20.0,
            t_transition_s: 5.0,
            t_linear_s: 10.0,
            ..ArcLineHoldRefConfig::default()
        });
        module.init();
        module
    }

    fn assert_states_close(actual: States, expected: States) {
        for (a, e) in [
            (actual.0, expected.0),
            (actual.1, expected.1),
            (actual.2, expected.2),
        ] {
            assert!((a - e).abs().max() <= 1e-9, "{a} vs {e}");
        }
    }

    #[test]
    fn phase_boundary_continuity() {
        let module = configured();
        let config = &module.config;
        let hold = (config.final_position_m, Vector3::zeros(), Vector3::zeros());
        assert_states_close(
            module.circle_states(config.t_circle_s),
            module.blend_states(0.0),
        );
        assert_states_close(
            module.blend_states(config.t_transition_s),
            module.decel_states(0.0),
        );
        assert_states_close(module.decel_states(config.t_linear_s), hold);
    }

    #[test]
    fn output_across_phases() {
        let mut module = configured();
        let config = module.config.clone();
        let arc_end = config.t_circle_s;
        let blend_end = arc_end + config.t_transition_s;
        let stop = blend_end + config.t_linear_s;
        let hold = (config.final_position_m, Vector3::zeros(), Vector3::zeros());
        let cases = [
            (0.0, module.circle_states(0.0)),
            (arc_end / 2.0, module.circle_states(arc_end / 2.0)),
            (arc_end, module.blend_states(0.0)),
            (
                arc_end + config.t_transition_s / 2.0,
                module.blend_states(config.t_transition_s / 2.0),
            ),
            (blend_end, module.decel_states(0.0)),
            (
                blend_end + config.t_linear_s / 2.0,
                module.decel_states(config.t_linear_s / 2.0),
            ),
            (stop, hold),
            (stop + 1.0, hold),
        ];
        for (time_s, expected) in cases {
            module.update(&SimulationContext {
                current_sim_nanos: (time_s * 1e9 + 0.5) as u64,
                current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
            });
            let message = module.trans_ref_out_msg.read();
            assert_states_close(
                (
                    message.position_m,
                    message.velocity_mps,
                    message.acceleration_mps2,
                ),
                expected,
            );
        }
    }

    #[test]
    fn phase_derivatives() {
        let module = configured();
        let config = module.config.clone();
        let phases: [(&dyn Fn(f64) -> States, f64); 3] = [
            (&|t| module.circle_states(t), config.t_circle_s / 2.0),
            (&|t| module.blend_states(t), config.t_transition_s / 2.0),
            (&|t| module.decel_states(t), config.t_linear_s / 2.0),
        ];
        let step = 1e-4;
        for (states, time) in phases {
            let before = states(time - step);
            let (_, velocity, acceleration) = states(time);
            let after = states(time + step);
            let velocity_difference = (after.0 - before.0) / (2.0 * step);
            let acceleration_difference = (after.1 - before.1) / (2.0 * step);
            assert!((velocity - velocity_difference).abs().max() <= 1e-6);
            assert!((acceleration - acceleration_difference).abs().max() <= 1e-6);
        }
    }

    #[test]
    fn invalid_endpoints() {
        let cases = [
            (false, Vector3::new(1.0, -16.0, 0.0), "blendEndPosition"),
            (false, Vector3::new(0.0, -16.0, -2.0), "blendEndPosition"),
            (true, Vector3::new(1.0, -5.0, 0.0), "finalPosition"),
            (true, Vector3::new(0.0, -5.0, 2.0), "finalPosition"),
            (false, Vector3::new(0.0, f64::NAN, 0.0), "blendEndPosition"),
        ];
        // Upstream's two-component final position cannot be expressed by a
        // `Vector3`.
        for (final_position, endpoint, name) in cases {
            let mut config = ArcLineHoldRefConfig::default();
            if final_position {
                config.final_position_m = endpoint;
            } else {
                config.blend_end_position_m = endpoint;
            }
            let error = config.validate().unwrap_err();
            assert!(error.contains(name), "{error}");
        }
    }

    #[test]
    fn invalid_durations() {
        for (field, value) in [(0, 0.0), (1, 0.0), (2, -3601.0)] {
            let mut config = ArcLineHoldRefConfig::default();
            match field {
                0 => config.t_transition_s = value,
                1 => config.t_linear_s = value,
                _ => config.t_circle_s = value,
            }
            assert!(config.validate().unwrap_err().contains("duration"));
        }
    }

    #[test]
    fn nonfinite_trajectory_parameters() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for name in ["radius", "tCircle", "tTransition", "tLinear"] {
                let mut config = ArcLineHoldRefConfig::default();
                match name {
                    "radius" => config.radius_m = value,
                    "tCircle" => config.t_circle_s = value,
                    "tTransition" => config.t_transition_s = value,
                    _ => config.t_linear_s = value,
                }
                let error = config.validate().unwrap_err();
                assert!(error.contains(&format!("{name} must be finite")), "{error}");
            }
        }
    }
}
