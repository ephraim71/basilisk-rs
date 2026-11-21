//! Attitude reference from a Hill-frame relative state through a gain matrix:
//! port of upstream Basilisk `hillToAttRef`.
//!
//! The module serves attitude-driven formation control laws whose gains are
//! designed elsewhere, such as differential-drag rendezvous (Linear Coupled
//! Attitude-Orbit Control Through Aerodynamic Drag,
//! <https://arc.aiaa.org/doi/10.2514/1.G004521>). Every update it
//!
//! 1. stacks the deputy's Hill-frame relative position and velocity into
//!    `x = (r_DC_H, v_DC_H)`,
//! 2. maps it to a relative MRP set `σ_rel = [G] x` with the 3×6 gain matrix,
//! 3. saturates each component of `σ_rel` to `[rel_mrp_min, rel_mrp_max]`,
//! 4. adds `σ_rel` to a base attitude, taken from either an attitude
//!    reference or a navigation attitude message, and publishes the result
//!    with zero reference rate and acceleration.

use nalgebra::{Matrix3x6, Vector3, Vector6};

use crate::messages::{
    AttitudeReferenceMsg, HillRelativeStateMsg, Input, NavigationAttitudeMsg, Output,
};
use crate::utilities::rigid_body_kinematics::add_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`HillToAttRef`]. `Default` is upstream's
/// zero-initialized configuration.
///
/// Upstream quirk, kept: the saturation limits default to zero, so until
/// `rel_mrp_min` and `rel_mrp_max` are set the relative attitude is clamped
/// to zero and the module republishes the base attitude unchanged.
#[derive(Clone, Debug)]
pub struct HillToAttRefConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `gainMatrix` (`double[3][6]`): maps the Hill-frame state
    /// `(r_DC_H [m], v_DC_H [m/s])` to a relative MRP set. Build it from
    /// upstream's row-major array with [`Matrix3x6::from_row_slice`].
    pub gain_matrix: Matrix3x6<f64>,
    /// Upstream `relMRPMin`: lower bound of each relative MRP component.
    pub rel_mrp_min: f64,
    /// Upstream `relMRPMax`: upper bound of each relative MRP component.
    pub rel_mrp_max: f64,
}

impl Default for HillToAttRefConfig {
    fn default() -> Self {
        Self {
            name: "hillToAttRef".to_string(),
            gain_matrix: Matrix3x6::zeros(),
            rel_mrp_min: 0.0,
            rel_mrp_max: 0.0,
        }
    }
}

/// Hill-state feedback attitude reference generator: upstream `hillToAttRef`.
#[derive(Clone, Debug)]
pub struct HillToAttRef {
    /// User-settable parameters.
    pub config: HillToAttRefConfig,
    /// Upstream `hillStateInMsg`: deputy state relative to the chief.
    pub hill_state_in_msg: Input<HillRelativeStateMsg>,
    /// Upstream `attRefInMsg` (optional): base attitude `sigma_RN`. Connect
    /// exactly one of this and `att_nav_in_msg`; this one wins if both are.
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Upstream `attNavInMsg` (optional): base attitude `sigma_BN`, typically
    /// the chief's navigation attitude.
    pub att_nav_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `attRefOutMsg`: commanded attitude reference.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
}

impl HillToAttRef {
    /// Builds the module with `config` and unconnected ports.
    pub fn new(config: HillToAttRefConfig) -> Self {
        Self {
            config,
            hill_state_in_msg: Input::default(),
            att_ref_in_msg: Input::default(),
            att_nav_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_hillToAttRef`: the module keeps no state, so a reset
    /// only checks that the Hill state and exactly one base attitude are
    /// connected.
    fn check_connections(&self) {
        if !self.hill_state_in_msg.is_connected() {
            log::error!("hillToAttRef.hill_state_in_msg is not connected");
        }
        match (
            self.att_ref_in_msg.is_connected(),
            self.att_nav_in_msg.is_connected(),
        ) {
            (true, true) => log::error!(
                "hillToAttRef can't have both att_ref_in_msg and att_nav_in_msg connected"
            ),
            (false, false) => log::error!(
                "hillToAttRef must have one of att_ref_in_msg and att_nav_in_msg connected"
            ),
            _ => {}
        }
    }

    /// Base attitude the relative attitude is added to. Upstream leaves its
    /// buffer uninitialized when neither input is connected (an error at
    /// reset); zero stands in for that indeterminate value.
    fn base_attitude(&self) -> Vector3<f64> {
        if self.att_ref_in_msg.is_connected() {
            self.att_ref_in_msg.read().sigma_rn
        } else if self.att_nav_in_msg.is_connected() {
            self.att_nav_in_msg.read().sigma_bn
        } else {
            Vector3::zeros()
        }
    }

    /// Relative MRP set `[G] x` of the Hill-frame state, before saturation.
    fn relative_attitude(&self, hill_state: &HillRelativeStateMsg) -> Vector3<f64> {
        let state = Vector6::from_iterator(
            hill_state
                .position_dc_h_m
                .iter()
                .chain(&hill_state.velocity_dc_h_mps)
                .copied(),
        );
        self.config.gain_matrix * state
    }

    /// Upstream `RelativeToInertialMRP`: saturates `relative_att`
    /// component-wise and adds it to `sigma_xn`; the reference rate and
    /// acceleration are zero.
    fn relative_to_inertial_mrp(
        &self,
        relative_att: Vector3<f64>,
        sigma_xn: Vector3<f64>,
    ) -> AttitudeReferenceMsg {
        // fmax then fmin, as upstream: a NaN component becomes the lower
        // bound, and with min > max every component becomes max.
        // `f64::clamp` would keep the NaN and panic on min > max.
        let saturated = relative_att.map(|sigma| {
            sigma
                .max(self.config.rel_mrp_min)
                .min(self.config.rel_mrp_max)
        });
        AttitudeReferenceMsg {
            sigma_rn: add_mrp(sigma_xn, saturated),
            omega_rn_n_radps: Vector3::zeros(),
            domega_rn_n_radps2: Vector3::zeros(),
        }
    }
}

impl Module for HillToAttRef {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let hill_state = self.hill_state_in_msg.read();
        let base = self.base_attitude();
        let relative = self.relative_attitude(&hill_state);
        self.att_ref_out_msg
            .write(self.relative_to_inertial_mrp(relative, base));
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3x6, Vector3};

    use super::{HillToAttRef, HillToAttRefConfig};
    use crate::messages::{
        AttitudeReferenceMsg, HillRelativeStateMsg, NavigationAttitudeMsg, Output,
    };
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::add_mrp;
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(1.0)`.
    const TASK_RATE_NANOS: u64 = 1_000_000_000;

    #[derive(Clone, Copy, Debug)]
    enum BaseMessage {
        AttRef,
        NavAtt,
    }

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The gain of `test_hillToAttRef.py`: `lqr_gain_set.T`, with
    /// `σ_rel = (y, 0, 0.25 ẏ)`.
    fn upstream_gain() -> Matrix3x6<f64> {
        #[rustfmt::skip]
        let gain = Matrix3x6::from_row_slice(&[
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.25, 0.0,
        ]);
        gain
    }

    /// The relative state of `test_hillToAttRef.py`.
    fn upstream_hill_state() -> HillRelativeStateMsg {
        HillRelativeStateMsg {
            position_dc_h_m: Vector3::new(1000.0, 0.0, 0.0),
            velocity_dc_h_mps: Vector3::new(0.0, 5.0, 0.0),
        }
    }

    /// `pytest.approx` with its defaults: relative 1e-6, absolute 1e-12.
    fn assert_approx(actual: Vector3<f64>, expected: [f64; 3], what: &str) {
        for (index, (value, truth)) in actual.iter().zip(expected).enumerate() {
            let tolerance = (1e-6 * truth.abs()).max(1e-12);
            assert!(
                (value - truth).abs() <= tolerance,
                "{what}[{index}] = {value:.17e}, truth {truth:.17e}"
            );
        }
    }

    /// The module with its Hill state connected and, if given, one base
    /// attitude input. An input keeps its producer's value alive, so the
    /// producers can be dropped.
    fn connected_module(
        config: HillToAttRefConfig,
        hill_state: HillRelativeStateMsg,
        base: Option<(BaseMessage, Vector3<f64>)>,
    ) -> HillToAttRef {
        let mut module = HillToAttRef::new(config);
        Output::new(hill_state).connect_to(&mut module.hill_state_in_msg);
        match base {
            Some((BaseMessage::AttRef, sigma_rn)) => Output::new(AttitudeReferenceMsg {
                sigma_rn,
                ..Default::default()
            })
            .connect_to(&mut module.att_ref_in_msg),
            Some((BaseMessage::NavAtt, sigma_bn)) => Output::new(NavigationAttitudeMsg {
                sigma_bn,
                ..Default::default()
            })
            .connect_to(&mut module.att_nav_in_msg),
            None => {}
        }
        module
    }

    /// Upstream `test_hillToAttRef.py`, all four `msg_type` x `use_limits`
    /// parametrizations, with upstream's hard-coded truth and `pytest.approx`.
    /// Upstream runs a 1 s task to t = 1 s and checks the last sample.
    ///
    /// Without limits the zero default bounds clamp the relative attitude to
    /// zero, so the output is the base attitude, as upstream expects.
    #[test]
    fn matches_upstream_hill_to_att_ref_unit_test() {
        let cases = [
            (
                BaseMessage::AttRef,
                true,
                [0.2165725, 0.32956685, 0.51789077],
            ),
            (
                BaseMessage::NavAtt,
                true,
                [-0.37398374, -0.25203252, -0.57723577],
            ),
            (BaseMessage::AttRef, false, [0.2, 0.2, 0.2]),
            (BaseMessage::NavAtt, false, [0.5, 0.5, 0.5]),
        ];
        for (base_message, use_limits, truth) in cases {
            let base_sigma = match base_message {
                BaseMessage::AttRef => Vector3::new(0.2, 0.2, 0.2),
                BaseMessage::NavAtt => Vector3::new(0.5, 0.5, 0.5),
            };
            let config = HillToAttRefConfig {
                gain_matrix: upstream_gain(),
                ..Default::default()
            };
            let mut module = connected_module(
                config,
                upstream_hill_state(),
                Some((base_message, base_sigma)),
            );
            // Upstream sets the limits after connecting, before initializing.
            if use_limits {
                module.config.rel_mrp_min = -0.2;
                module.config.rel_mrp_max = 0.2;
            }

            let mut recorder = MessageRecorder::<AttitudeReferenceMsg>::default();
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
            simulation.add_module("hillToAttRef", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(TASK_RATE_NANOS);

            assert_eq!(recorder.samples.len(), 2);
            let last = recorder.samples.last().expect("two samples were recorded");
            let what = format!("{base_message:?} use_limits={use_limits} sigma_RN");
            assert_approx(last.sigma_rn, truth, &what);
            assert_eq!(last.omega_rn_n_radps, Vector3::zeros());
            assert_eq!(last.domega_rn_n_radps2, Vector3::zeros());
        }
    }

    /// Not upstream: with bounds that never bind the output is exactly
    /// `add_mrp(base, [G] x)`.
    #[test]
    fn adds_the_unsaturated_gain_product_to_the_base_attitude() {
        #[rustfmt::skip]
        let gain = Matrix3x6::from_row_slice(&[
            1e-4, 0.0, 2e-5, 0.0, 1e-2, 0.0,
            0.0, -3e-4, 0.0, 5e-3, 0.0, 0.0,
            2e-5, 0.0, 1e-4, 0.0, 0.0, -2e-2,
        ]);
        let hill_state = HillRelativeStateMsg {
            position_dc_h_m: Vector3::new(120.0, -300.0, 40.0),
            velocity_dc_h_mps: Vector3::new(0.1, 2.0, -1.5),
        };
        let base = Vector3::new(0.1, -0.2, 0.3);
        let mut module = connected_module(
            HillToAttRefConfig {
                gain_matrix: gain,
                rel_mrp_min: -1.0,
                rel_mrp_max: 1.0,
                ..Default::default()
            },
            hill_state.clone(),
            Some((BaseMessage::NavAtt, base)),
        );
        module.init();
        module.update(&context(0));

        let x = [120.0, -300.0, 40.0, 0.1, 2.0, -1.5];
        let relative = Vector3::from_fn(|row, _| (0..6).map(|col| gain[(row, col)] * x[col]).sum());
        let expected = add_mrp(base, relative);
        let output = module.att_ref_out_msg.read();
        assert!(
            (output.sigma_rn - expected).amax() < 1e-15,
            "{}",
            output.sigma_rn
        );
    }

    /// Not upstream: upstream's `fmax` then `fmin` saturation. With
    /// `min > max` every component becomes `max`; a NaN component takes the
    /// bound instead of propagating.
    #[test]
    fn saturation_applies_the_lower_bound_then_the_upper_bound() {
        let saturated = |min: f64, max: f64, relative: Vector3<f64>| {
            let module = HillToAttRef::new(HillToAttRefConfig {
                rel_mrp_min: min,
                rel_mrp_max: max,
                ..Default::default()
            });
            // A zero base attitude makes the output the saturated relative set.
            module
                .relative_to_inertial_mrp(relative, Vector3::zeros())
                .sigma_rn
        };
        assert_eq!(
            saturated(-0.2, 0.2, Vector3::new(-1.0, 0.1, 3.0)),
            add_mrp(Vector3::zeros(), Vector3::new(-0.2, 0.1, 0.2))
        );
        assert_eq!(
            saturated(0.3, 0.1, Vector3::new(-1.0, 0.2, 3.0)),
            add_mrp(Vector3::zeros(), Vector3::new(0.1, 0.1, 0.1))
        );
        assert_eq!(
            saturated(-0.2, 0.2, Vector3::new(f64::NAN, 0.0, 0.0)),
            add_mrp(Vector3::zeros(), Vector3::new(-0.2, 0.0, 0.0))
        );
    }

    /// Not upstream: the attitude reference input takes precedence when both
    /// base inputs are connected (an error at reset), as upstream's update
    /// checks it first; with neither connected the base is zero.
    #[test]
    fn base_attitude_prefers_the_attitude_reference_input() {
        let config = HillToAttRefConfig {
            gain_matrix: upstream_gain(),
            rel_mrp_min: -0.2,
            rel_mrp_max: 0.2,
            ..Default::default()
        };
        let mut module = connected_module(
            config.clone(),
            upstream_hill_state(),
            Some((BaseMessage::AttRef, Vector3::new(0.2, 0.2, 0.2))),
        );
        let att_nav = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.5, 0.5, 0.5),
            ..Default::default()
        });
        att_nav.connect_to(&mut module.att_nav_in_msg);
        module.init();
        module.update(&context(0));
        assert_approx(
            module.att_ref_out_msg.read().sigma_rn,
            [0.2165725, 0.32956685, 0.51789077],
            "both connected",
        );

        let mut module = connected_module(config, upstream_hill_state(), None);
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.att_ref_out_msg.read().sigma_rn,
            add_mrp(Vector3::zeros(), Vector3::new(0.0, 0.2, 0.2))
        );
    }

    #[test]
    fn init_publishes_a_zero_attitude_reference() {
        let mut module = HillToAttRef::new(HillToAttRefConfig::default());
        module.att_ref_out_msg.write(AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.1, 0.2, 0.3),
            omega_rn_n_radps: Vector3::new(1.0, 2.0, 3.0),
            domega_rn_n_radps2: Vector3::new(4.0, 5.0, 6.0),
        });
        module.init();
        let output = module.att_ref_out_msg.read();
        assert_eq!(output.sigma_rn, Vector3::zeros());
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
        assert_eq!(output.domega_rn_n_radps2, Vector3::zeros());
    }
}
