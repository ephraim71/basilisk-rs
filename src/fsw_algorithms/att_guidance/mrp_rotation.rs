//! Constant-rate MRP rotation of an attitude reference: port of upstream
//! Basilisk `mrpRotation`.
//!
//! The module superimposes on the input reference frame `R0` a rotation
//! `R/R0` with an initial MRP set `σ_R/R0` and an angular velocity
//! `ω_R/R0` that is constant in `R`-frame components. Every update first
//! integrates `σ_R/R0 += Δt ¼ [B(σ_R/R0)] ω_R/R0`, switches the result to the
//! short-rotation set, and then publishes
//!
//! - `[RN] = [RR0(σ_R/R0)] [R0N]`,
//! - `ω_R/N = ω_R0/N + [RN]^T ω_R/R0`,
//! - `ω'_R/N = ω'_R0/N + ω_R0/N × ω_R/R0`.
//!
//! The initial set and rate come from the configuration. When the optional
//! desired-attitude input is connected, a command whose content differs from
//! the last accepted one replaces both.

use nalgebra::Vector3;

use super::euler_rotation::{CommandLatch, integration_step_s};
use crate::messages::{AttitudeReferenceMsg, AttitudeStateMsg, Input, Output};
use crate::utilities::rigid_body_kinematics::{b_mat_mrp, dcm_to_mrp, mrp_switch, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`MrpRotation`]. `Default` is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct MrpRotationConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mrpSet`: initial MRP set `σ_R/R0` of `R` relative to the
    /// input reference `R0`.
    pub sigma_rr0: Vector3<f64>,
    /// Upstream `omega_RR0_R` \[rad/s\]: angular velocity of `R` relative to
    /// `R0`, constant in `R`-frame components.
    pub omega_rr0_r_radps: Vector3<f64>,
}

impl Default for MrpRotationConfig {
    fn default() -> Self {
        Self {
            name: "mrpRotation".to_string(),
            sigma_rr0: Vector3::zeros(),
            omega_rr0_r_radps: Vector3::zeros(),
        }
    }
}

/// Reference frame rotating at a constant body rate relative to an input
/// reference: upstream `mrpRotation`.
#[derive(Clone, Debug)]
pub struct MrpRotation {
    /// User-settable parameters. The set and rate are copied into
    /// [`Self::sigma_rr0`] and [`Self::omega_rr0_r_radps`] by
    /// [`Module::init`].
    pub config: MrpRotationConfig,
    /// Upstream `attRefInMsg`: input reference frame `R0`.
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Upstream `desiredAttInMsg` (optional): commanded MRP set `σ_R/R0` and
    /// rate `ω_R/R0` \[rad/s\].
    pub desired_att_in_msg: Input<AttitudeStateMsg>,
    /// Upstream `attRefOutMsg`: rotating reference frame `R`.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `mrpSet`: current MRP set `σ_R/R0`, integrated on every
    /// update. Upstream lets the user overwrite it at any time; so does this
    /// public field. A reset leaves it unchanged, as upstream.
    pub sigma_rr0: Vector3<f64>,
    /// Upstream `omega_RR0_R` \[rad/s\]: current rate of `R` relative to `R0`.
    pub omega_rr0_r_radps: Vector3<f64>,
    command_latch: CommandLatch,
    prior_time_nanos: u64,
}

impl MrpRotation {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: MrpRotationConfig) -> Self {
        Self {
            sigma_rr0: config.sigma_rr0,
            omega_rr0_r_radps: config.omega_rr0_r_radps,
            config,
            att_ref_in_msg: Input::default(),
            desired_att_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            command_latch: CommandLatch::default(),
            prior_time_nanos: 0,
        }
    }

    /// Upstream `Reset_mrpRotation`: forget the prior call time and the last
    /// accepted command. The current set and rate are kept.
    fn reset_state(&mut self) {
        if !self.att_ref_in_msg.is_connected() {
            log::error!("mrpRotation.att_ref_in_msg is not connected");
        }
        self.prior_time_nanos = 0;
        self.command_latch = CommandLatch::default();
    }

    /// Upstream `computeMRPRotationReference`: advance the MRP set by `dt_s`
    /// and build the rotating reference on top of `input`.
    fn rotated_reference(
        &mut self,
        input: &AttitudeReferenceMsg,
        dt_s: f64,
    ) -> AttitudeReferenceMsg {
        let dcm_r0n = mrp_to_dcm(input.sigma_rn);
        let sigma_dot_rr0 = 0.25 * (b_mat_mrp(self.sigma_rr0) * self.omega_rr0_r_radps);
        self.sigma_rr0 = mrp_switch(self.sigma_rr0 + dt_s * sigma_dot_rr0, 1.0);
        let dcm_rn = mrp_to_dcm(self.sigma_rr0) * dcm_r0n;

        let omega_rr0_n = dcm_rn.transpose() * self.omega_rr0_r_radps;
        AttitudeReferenceMsg {
            sigma_rn: dcm_to_mrp(dcm_rn),
            omega_rn_n_radps: input.omega_rn_n_radps + omega_rr0_n,
            domega_rn_n_radps2: input.omega_rn_n_radps.cross(&omega_rr0_n)
                + input.domega_rn_n_radps2,
        }
    }
}

impl Module for MrpRotation {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.sigma_rr0 = self.config.sigma_rr0;
        self.omega_rr0_r_radps = self.config.omega_rr0_r_radps;
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let input = self.att_ref_in_msg.read();
        if self.desired_att_in_msg.is_connected()
            && let Some(command) = self.command_latch.accept(self.desired_att_in_msg.read())
        {
            self.sigma_rr0 = command.state;
            self.omega_rr0_r_radps = command.rate;
        }

        let dt_s = integration_step_s(self.prior_time_nanos, context.current_sim_nanos);
        let reference = self.rotated_reference(&input, dt_s);
        self.att_ref_out_msg.write(reference);
        self.prior_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports `mrpRotation/_UnitTest/test_mrpRotation.py` over all four
    //! `(cmdStateFlag, testReset)` combinations, with the truth of
    //! `Support/truth_mrpRotation.py` re-implemented in [`upstream_truth`].
    //! The pytest's `writeTeXSnippet` documentation output is not ported.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeReferenceMsg, AttitudeStateMsg, Output};
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{add_mrp, b_mat_mrp, dcm_to_mrp, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{MrpRotation, MrpRotationConfig};

    const TASK_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!((a - e).abs() <= accuracy, "{actual} != {expected}");
        }
    }

    /// `truth_mrpRotation.results`: the expected `(sigma_RN, omega_RN_N,
    /// domega_RN_N)` samples, two zero-step samples followed by two 0.5 s
    /// steps, and after a reset one zero step and one 0.5 s step.
    fn upstream_truth(
        mut sigma_rr0: Vector3<f64>,
        omega_rr0_r: Vector3<f64>,
        input: &AttitudeReferenceMsg,
        dt: f64,
        cmd_state_flag: bool,
        test_reset: bool,
    ) -> Vec<[Vector3<f64>; 3]> {
        let dcm_r0n = mrp_to_dcm(input.sigma_rn);
        let omega_r0n_n = input.omega_rn_n_radps;
        let domega_r0n_n = input.domega_rn_n_radps2;
        let rates = |sigma_rr0: Vector3<f64>| {
            let dcm_rn = mrp_to_dcm(sigma_rr0) * dcm_r0n;
            let omega_rr0_n = dcm_rn.transpose() * omega_rr0_r;
            (
                omega_rr0_n + omega_r0n_n,
                omega_r0n_n.cross(&omega_rr0_n) + domega_r0n_n,
            )
        };
        let initial_sample = |sigma_rr0: Vector3<f64>| {
            let (omega, domega) = rates(sigma_rr0);
            [add_mrp(input.sigma_rn, sigma_rr0), omega, domega]
        };
        let step = |sigma_rr0: &mut Vector3<f64>| {
            *sigma_rr0 += dt * 0.25 * (b_mat_mrp(*sigma_rr0) * omega_rr0_r);
            let sigma_rn = dcm_to_mrp(mrp_to_dcm(*sigma_rr0) * dcm_r0n);
            let (omega, domega) = rates(*sigma_rr0);
            [sigma_rn, omega, domega]
        };

        let s0 = sigma_rr0;
        let mut samples = vec![initial_sample(sigma_rr0), initial_sample(sigma_rr0)];
        samples.push(step(&mut sigma_rr0));
        samples.push(step(&mut sigma_rr0));
        if test_reset {
            if cmd_state_flag {
                sigma_rr0 = s0;
            }
            samples.push(initial_sample(sigma_rr0));
            samples.push(step(&mut sigma_rr0));
        }
        samples
    }

    /// Upstream `test_mrpRotation[testReset, cmdStateFlag]`: updates every
    /// 0.5 s up to 1.5 s, then for `testReset` a `Reset(1)` and two more
    /// updates up to 2.5 s (`accuracy = 1e-12`).
    #[test]
    fn matches_upstream_mrp_rotation_unit_test() {
        let accuracy = 1e-12;
        let input = AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.1, 0.2, 0.3),
            omega_rn_n_radps: Vector3::new(0.1, 0.0, 0.0),
            domega_rn_n_radps2: Vector3::zeros(),
        };

        for test_reset in [false, true] {
            for cmd_state_flag in [false, true] {
                let mut sigma_rr0 = Vector3::new(0.3, 0.5, 0.0);
                let mut omega_rr0_r = Vector3::new(0.1, 0.0, 0.0) * D2R;
                let mut module = MrpRotation::new(MrpRotationConfig::default());
                module.config.sigma_rr0 = sigma_rr0;
                module.config.omega_rr0_r_radps = omega_rr0_r;

                let desired = Output::new(AttitudeStateMsg::default());
                if cmd_state_flag {
                    sigma_rr0 = Vector3::new(0.1, 0.0, -0.2);
                    omega_rr0_r = Vector3::new(0.1, 1.0, 0.5) * D2R;
                    desired.write(AttitudeStateMsg {
                        state: sigma_rr0,
                        rate: omega_rr0_r,
                    });
                    desired.connect_to(&mut module.desired_att_in_msg);
                }
                let reference = Output::new(input.clone());
                reference.connect_to(&mut module.att_ref_in_msg);

                module.init();
                let mut samples = Vec::new();
                for tick in 0..=3 {
                    module.update(&context(tick * TASK_NANOS));
                    samples.push(module.att_ref_out_msg.read());
                }
                if test_reset {
                    Module::reset(&mut module, &context(1));
                    for tick in 4..=5 {
                        module.update(&context(tick * TASK_NANOS));
                        samples.push(module.att_ref_out_msg.read());
                    }
                }

                let truth = upstream_truth(
                    sigma_rr0,
                    omega_rr0_r,
                    &input,
                    0.5,
                    cmd_state_flag,
                    test_reset,
                );
                assert_eq!(samples.len(), truth.len());
                for (sample, [sigma, omega, domega]) in samples.iter().zip(&truth) {
                    assert_vector_close(sample.sigma_rn, *sigma, accuracy);
                    assert_vector_close(sample.omega_rn_n_radps, *omega, accuracy);
                    assert_vector_close(sample.domega_rn_n_radps2, *domega, accuracy);
                }
            }
        }
    }

    /// Not upstream: the integrated set is switched to the short rotation
    /// once it leaves the unit sphere.
    #[test]
    fn switches_the_integrated_set_to_the_short_rotation() {
        let reference = Output::new(AttitudeReferenceMsg::default());
        let mut module = MrpRotation::new(MrpRotationConfig {
            sigma_rr0: Vector3::new(0.999, 0.0, 0.0),
            omega_rr0_r_radps: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        });
        reference.connect_to(&mut module.att_ref_in_msg);
        module.init();
        module.update(&context(TASK_NANOS));
        module.update(&context(2 * TASK_NANOS));

        // σ' = ¼ (1 + σ²) ω along x, then the shadow set -σ/|σ|².
        let unswitched = 0.999 + 0.5 * 0.25 * (1.0 + 0.999 * 0.999);
        assert!(unswitched > 1.0);
        assert_vector_close(
            module.sigma_rr0,
            Vector3::new(-1.0 / unswitched, 0.0, 0.0),
            1e-15,
        );
        assert!(module.att_ref_out_msg.read().sigma_rn.norm() <= 1.0);
    }

    /// Not upstream: a configuration changed after construction is read at
    /// `init`.
    #[test]
    fn reads_the_configuration_at_init() {
        let reference = Output::new(AttitudeReferenceMsg::default());
        let mut module = MrpRotation::new(MrpRotationConfig::default());
        module.config.sigma_rr0 = Vector3::new(0.1, -0.2, 0.3);
        reference.connect_to(&mut module.att_ref_in_msg);
        module.init();
        module.update(&context(0));
        assert_vector_close(
            module.att_ref_out_msg.read().sigma_rn,
            Vector3::new(0.1, -0.2, 0.3),
            1e-15,
        );
    }
}
