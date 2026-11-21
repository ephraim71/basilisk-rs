//! Reference frame spinning at a constant rate: port of upstream Basilisk
//! `inertial3DSpin`.
//!
//! The output frame `R` spins with the constant angular velocity `ω_R/R0`
//! relative to the input reference frame `R0`, or relative to the inertial
//! frame when no input is connected (a zero input reference is then assumed,
//! as upstream). The attitude `σ_R/N` starts from the input attitude on the
//! first update after a reset and is then integrated on its own:
//!
//! - `ω_R/N = ω_R0/N + [RN]^T ω_R/R0`,
//! - `ω'_R/N = ω'_R0/N + ω_R0/N × ω_R/R0`,
//! - `σ_R/N += Δt ¼ [B(σ_R/N)] [RN] ω_R/N`, switched to the short rotation.
//!
//! As upstream, the rates are evaluated with the attitude before this
//! update's integration step, the published attitude is the one after it, and
//! the input attitude is only read on that first update.

use nalgebra::Vector3;

use crate::messages::{AttitudeReferenceMsg, Input, Output};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::rigid_body_kinematics::{b_mat_mrp, mrp_switch, mrp_to_dcm};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`Inertial3DSpin`]. `Default` is upstream's
/// zero-initialized configuration (no spin).
#[derive(Clone, Debug)]
pub struct Inertial3DSpinConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `omega_RR0_R0` \[rad/s\]: constant spin of `R` relative to
    /// `R0`. A spin about a fixed axis has the same components in `R` and
    /// `R0`; upstream maps it to inertial components with `[RN]^T`.
    pub omega_rr0_r0_radps: Vector3<f64>,
}

impl Default for Inertial3DSpinConfig {
    fn default() -> Self {
        Self {
            name: "inertial3DSpin".to_string(),
            omega_rr0_r0_radps: Vector3::zeros(),
        }
    }
}

/// Reference frame spinning at a constant rate relative to an input (or the
/// inertial) frame: upstream `inertial3DSpin`.
#[derive(Clone, Debug)]
pub struct Inertial3DSpin {
    /// User-settable parameters, read on every update.
    pub config: Inertial3DSpinConfig,
    /// Upstream `attRefInMsg` (optional): input reference frame `R0`.
    pub att_ref_in_msg: Input<AttitudeReferenceMsg>,
    /// Upstream `attRefOutMsg`: spinning reference frame `R`.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `sigma_RN`: integrated attitude of `R` relative to `N`.
    sigma_rn: Vector3<f64>,
    /// Upstream `priorTime` \[ns\]; zero means "no prior update".
    prior_time_nanos: u64,
}

impl Inertial3DSpin {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: Inertial3DSpinConfig) -> Self {
        Self {
            config,
            att_ref_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            sigma_rn: Vector3::zeros(),
            prior_time_nanos: 0,
        }
    }

    /// Upstream `computeReference_inertial3DSpin`: the reference rates at the
    /// current attitude, then one integration step of `dt_s`.
    fn spinning_reference(
        &mut self,
        input: &AttitudeReferenceMsg,
        dt_s: f64,
    ) -> AttitudeReferenceMsg {
        let dcm_rn = mrp_to_dcm(self.sigma_rn);
        let omega_rr0_n = dcm_rn.transpose() * self.config.omega_rr0_r0_radps;
        let omega_rn_n = input.omega_rn_n_radps + omega_rr0_n;
        let domega_rn_n = input.omega_rn_n_radps.cross(&omega_rr0_n) + input.domega_rn_n_radps2;

        let omega_rn_r = dcm_rn * omega_rn_n;
        // [B] is scaled before the product, as upstream, for the same rounding.
        let sigma_rn_step = (b_mat_mrp(self.sigma_rn) * (0.25 * dt_s)) * omega_rn_r;
        self.sigma_rn = mrp_switch(self.sigma_rn + sigma_rn_step, 1.0);

        AttitudeReferenceMsg {
            sigma_rn: self.sigma_rn,
            omega_rn_n_radps: omega_rn_n,
            domega_rn_n_radps2: domega_rn_n,
        }
    }
}

impl Module for Inertial3DSpin {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.prior_time_nanos = 0;
    }

    /// Upstream `Reset_inertial3DSpin`: the next update restarts from the
    /// input attitude.
    fn reset(&mut self, _context: &SimulationContext) {
        self.prior_time_nanos = 0;
    }

    fn update(&mut self, context: &SimulationContext) {
        // An unconnected input reads the zero reference, as upstream.
        let input = self.att_ref_in_msg.read();

        // Upstream quirk, kept: a prior time of zero means "no prior update",
        // so after a first update at t = 0 the second update also restarts
        // from the input attitude with a zero step.
        let dt_s = if self.prior_time_nanos == 0 {
            self.sigma_rn = input.sigma_rn;
            0.0
        } else {
            diff_nanos_to_seconds(context.current_sim_nanos, self.prior_time_nanos)
        };

        let reference = self.spinning_reference(&input, dt_s);
        self.att_ref_out_msg.write(reference);
        self.prior_time_nanos = context.current_sim_nanos;
    }
}

#[cfg(test)]
mod tests {
    //! Ports `inertial3DSpin/_UnitTest/test_inertial3DSpin.py` with its
    //! hard-coded truth. Its two parametrizations, `subModuleTestFunction`
    //! and `subModuleTestFunction2`, are identical apart from line breaks, so
    //! one Rust test covers both. Their trailing `ConfigureStopTime(0.6 s)`
    //! lies before the current time, runs nothing and checks nothing.

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{AttitudeReferenceMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::{Inertial3DSpin, Inertial3DSpinConfig};

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

    /// Upstream `test_stateArchitectureAllTests[subModuleTestFunction]` and
    /// `[subModuleTestFunction2]`: updates at t = 0, 0.5, 1 and 1.5 s
    /// (`accuracy = 1e-12`).
    #[test]
    fn matches_upstream_inertial_3d_spin_unit_test() {
        let true_sigma = [
            [0.1, 0.2, 0.3],
            [0.1, 0.2, 0.3],
            [0.103643374814, 0.199258235068, 0.299694567381],
            [0.10728593457, 0.198511279747, 0.299381655572],
        ];
        let true_omega = [
            [0.02142849611, 0.01021197571, -0.011041933756],
            [0.02142849611, 0.01021197571, -0.011041933756],
            [0.02142849611, 0.01021197571, -0.011041933756],
            [0.021428270863, 0.010212299678, -0.011042071256],
        ];
        let accuracy = 1e-12;

        let reference = Output::new(AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.1, 0.2, 0.3),
            omega_rn_n_radps: Vector3::zeros(),
            domega_rn_n_radps2: Vector3::zeros(),
        });
        let mut module = Inertial3DSpin::new(Inertial3DSpinConfig::default());
        module.config.omega_rr0_r0_radps = Vector3::new(1.0, -1.0, 0.5) * D2R;
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&reference, &mut module.att_ref_in_msg);
        simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
        simulation.add_module("inertial3DSpin", &mut module, TASK_NANOS, 0);
        simulation.add_module("recorder", &mut recorder, TASK_NANOS, 0);
        simulation.run_for(1_500_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), true_sigma.len());
        for ((sample, sigma), omega) in recorder.samples.iter().zip(&true_sigma).zip(&true_omega) {
            assert_vector_close(sample.sigma_rn, Vector3::from(*sigma), accuracy);
            assert_vector_close(sample.omega_rn_n_radps, Vector3::from(*omega), accuracy);
            assert_vector_close(sample.domega_rn_n_radps2, Vector3::zeros(), accuracy);
        }
    }

    /// Not upstream: without an input the spin is relative to the inertial
    /// frame from the identity attitude; a spinning input frame adds its rate
    /// and the transport term `ω_R0/N × ω_R/R0`, and a reset restarts from
    /// the input attitude.
    #[test]
    fn spins_relative_to_the_input_frame_and_restarts_after_reset() {
        let spin = Vector3::new(0.0, 0.0, 0.2);
        let mut module = Inertial3DSpin::new(Inertial3DSpinConfig {
            omega_rr0_r0_radps: spin,
            ..Default::default()
        });
        module.init();
        module.update(&context(TASK_NANOS));
        let output = module.att_ref_out_msg.read();
        assert_eq!(output.sigma_rn, Vector3::zeros());
        assert_eq!(output.omega_rn_n_radps, spin);

        module.update(&context(2 * TASK_NANOS));
        // About z from the identity: σ' = ¼ ω.
        let sigma = Vector3::new(0.0, 0.0, 0.5 * 0.25 * 0.2);
        assert_vector_close(module.att_ref_out_msg.read().sigma_rn, sigma, 1e-16);

        let input = AttitudeReferenceMsg {
            sigma_rn: Vector3::new(0.2, -0.1, 0.05),
            omega_rn_n_radps: Vector3::new(0.01, 0.02, 0.03),
            domega_rn_n_radps2: Vector3::new(1e-3, 0.0, -1e-3),
        };
        let reference = Output::new(input.clone());
        reference.connect_to(&mut module.att_ref_in_msg);
        Module::reset(&mut module, &context(2 * TASK_NANOS));
        module.update(&context(3 * TASK_NANOS));

        let output = module.att_ref_out_msg.read();
        let omega_rr0_n = mrp_to_dcm(input.sigma_rn).transpose() * spin;
        assert_eq!(output.sigma_rn, input.sigma_rn);
        assert_vector_close(
            output.omega_rn_n_radps,
            input.omega_rn_n_radps + omega_rr0_n,
            1e-16,
        );
        assert_vector_close(
            output.domega_rn_n_radps2,
            input.omega_rn_n_radps.cross(&omega_rr0_n) + input.domega_rn_n_radps2,
            1e-16,
        );
    }
}
