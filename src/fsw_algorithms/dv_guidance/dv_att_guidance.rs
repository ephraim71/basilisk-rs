//! Delta-v burn attitude reference: port of upstream Basilisk `dvGuidance`
//! (`dvAttGuidance`).
//!
//! From a [`DvBurnCommandMsg`] the module builds the base burn frame `Bub`,
//! whose first axis is the commanded Δv direction `Δv̂`, whose second is
//! along `ê_rot × Δv̂` (`ê_rot` the commanded rotation axis) and whose third
//! completes the triad. The reference frame `But` turns away from it about
//! the third axis at the constant commanded rate `ω_rot`, starting at the burn
//! start time (and backwards before it):
//!
//! `[ButN] = M3(ω_rot (t - t_start)) [BubN]`.
//!
//! The output carries `σ_RN` of `But`, the constant angular velocity
//! `ω_rot b̂_3` in inertial components and a zero angular acceleration.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{AttitudeReferenceMsg, DvBurnCommandMsg, Input, Output};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{dcm_to_mrp, prv_to_dcm};
use crate::{Module, SimulationContext};

/// Configuration of [`DvGuidance`]; upstream has no parameters.
#[derive(Clone, Debug)]
pub struct DvGuidanceConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
}

impl Default for DvGuidanceConfig {
    fn default() -> Self {
        Self {
            name: "dvGuidance".to_string(),
        }
    }
}

/// Rotating delta-v burn reference frame: upstream `dvGuidance`.
#[derive(Clone, Debug)]
pub struct DvGuidance {
    /// Module configuration.
    pub config: DvGuidanceConfig,
    /// Upstream `burnDataInMsg`: burn direction, rotation and start time.
    pub burn_data_in_msg: Input<DvBurnCommandMsg>,
    /// Upstream `attRefOutMsg`: attitude reference of the burn frame.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
}

impl DvGuidance {
    /// Creates the module with unconnected ports.
    pub fn new(config: DvGuidanceConfig) -> Self {
        Self {
            config,
            burn_data_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_dvGuidance`: the module retains no state.
    fn check_connections(&self) {
        if !self.burn_data_in_msg.is_connected() {
            log::error!("dvGuidance.burn_data_in_msg is not connected");
        }
    }
}

/// `[BubN]`: rows `Δv̂`, the unit vector along `ê_rot × Δv̂`, and their
/// normalized cross product, each normalized as upstream.
fn base_burn_frame(burn: &DvBurnCommandMsg) -> Matrix3<f64> {
    let dv_hat_n = normalize_or_zero(burn.dv_inertial_cmd_mps);
    let bu2_n = normalize_or_zero(burn.dv_rot_vec_unit.cross(&dv_hat_n));
    let bu3_n = normalize_or_zero(dv_hat_n.cross(&bu2_n));
    Matrix3::from_rows(&[dv_hat_n.transpose(), bu2_n.transpose(), bu3_n.transpose()])
}

/// The burn frame reference at `call_nanos`; negative burn times are allowed.
fn burn_reference(burn: &DvBurnCommandMsg, call_nanos: u64) -> AttitudeReferenceMsg {
    let burn_time_s = diff_nanos_to_seconds(call_nanos, burn.burn_start_time_nanos);
    let rotation_prv = Vector3::z() * (burn_time_s * burn.dv_rot_vec_mag_radps);
    let dcm_butn = prv_to_dcm(rotation_prv) * base_burn_frame(burn);
    AttitudeReferenceMsg {
        sigma_rn: dcm_to_mrp(dcm_butn),
        // The third row of [ButN] is the rotation axis in inertial components.
        omega_rn_n_radps: dcm_butn.row(2).transpose() * burn.dv_rot_vec_mag_radps,
        domega_rn_n_radps2: Vector3::zeros(),
    }
}

impl Module for DvGuidance {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, context: &SimulationContext) {
        let burn = self.burn_data_in_msg.read();
        self.att_ref_out_msg
            .write(burn_reference(&burn, context.current_sim_nanos));
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{DvGuidance, DvGuidanceConfig};
    use crate::messages::{DvBurnCommandMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_RATE_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `isArrayEqual`: absolute tolerance per component.
    fn assert_vector_close(actual: Vector3<f64>, truth: [f64; 3], accuracy: f64, what: &str) {
        for (index, (value, expected)) in actual.iter().zip(truth).enumerate() {
            assert!(
                (value - expected).abs() <= accuracy,
                "{what}[{index}] = {value:.12e}, truth {expected:.12e}"
            );
        }
    }

    /// `test_dvGuidance.py::test_dv_guidance`: a burn starting at 0.5 s, run
    /// on a 0.5 s task to 1 s, so burn times of -0.5, 0 and 0.5 s. Upstream's
    /// hard-coded truth ("from a previous run") at `accuracy = 1e-9`.
    #[test]
    fn matches_upstream_dv_guidance_unit_test() {
        let true_sigma = [
            [5.69822629e-01, 1.99143700e-01, 2.72649472e-01],
            [6.12361487e-01, 1.31298090e-01, 3.16981631e-01],
            [6.50967464e-01, 5.62624705e-02, 3.61117890e-01],
        ];
        let true_omega = [4.08248290e-01, -2.04124145e-01, -2.04124145e-01];
        let accuracy = 1e-9;

        let burn_output = Output::new(DvBurnCommandMsg {
            dv_inertial_cmd_mps: Vector3::new(5.0, 5.0, 5.0),
            dv_rot_vec_unit: Vector3::new(1.0, 0.0, 0.0),
            dv_rot_vec_mag_radps: 0.5,
            burn_start_time_nanos: 500_000_000,
        });
        let mut module = DvGuidance::new(DvGuidanceConfig::default());
        let mut recorder = MessageRecorder::default();
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        simulation.connect(&burn_output, &mut module.burn_data_in_msg);
        simulation.connect(&module.att_ref_out_msg, &mut recorder.input_msg);
        simulation.add_module("dvGuidance", &mut module, TASK_RATE_NANOS, 0);
        simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
        simulation.run_for(1_000_000_000);
        drop(simulation);

        assert_eq!(recorder.samples.len(), true_sigma.len());
        for (sample, sigma) in recorder.samples.iter().zip(true_sigma) {
            assert_vector_close(sample.sigma_rn, sigma, accuracy, "sigma_RN");
            assert_vector_close(sample.omega_rn_n_radps, true_omega, accuracy, "omega_RN_N");
            assert_vector_close(sample.domega_rn_n_radps2, [0.0; 3], accuracy, "domega_RN_N");
        }
    }

    /// Beyond `test_dvGuidance.py`: the upstream test inputs plus two general
    /// burns (one after and one long before the start time, with a negative
    /// rate), against the upstream C module compiled with gcc
    /// (`-ffp-contract=off`); the port reproduces it bit for bit.
    #[test]
    fn matches_upstream_c_bit_for_bit() {
        #[rustfmt::skip]
        let cases = [
            ([5.0, 5.0, 5.0], [1.0, 0.0, 0.0], 0.5, 500_000_000, 0,
             [0.569822629321663, 0.19914370005207516, 0.27264947244992216],
             [0.4082482904638631, -0.20412414523193154, -0.20412414523193154]),
            ([5.0, 5.0, 5.0], [1.0, 0.0, 0.0], 0.5, 500_000_000, 500_000_000,
             [0.6123614872212251, 0.13129809048406266, 0.31698163076031394],
             [0.4082482904638631, -0.20412414523193154, -0.20412414523193154]),
            ([5.0, 5.0, 5.0], [1.0, 0.0, 0.0], 0.5, 500_000_000, 1_000_000_000,
             [0.6509674643723068, 0.05626247049433212, 0.361117890220258],
             [0.4082482904638631, -0.20412414523193154, -0.20412414523193154]),
            ([-1.5, 0.3, 2.2], [0.2, -0.7, 0.4], 0.013, 7_000_000_000, 13_250_000_000,
             [0.28633270318945797, -0.3551948809358323, 0.6263707002550412],
             [0.004400991640821861, -0.011355044730739275, 0.004549091309297534]),
            ([0.4, -3.0, 0.9], [0.0, 0.6, 0.8], -0.2, 9_000_000_000, 2_500_000_000,
             [-0.07520189676718113, 0.006104011225652114, -0.037440729325792864],
             [-0.009222231342945189, -0.058535551718416026, -0.19101973624229998]),
        ];
        for (dv, axis, rate, start_nanos, time_nanos, sigma, omega) in cases {
            let burn_output = Output::new(DvBurnCommandMsg {
                dv_inertial_cmd_mps: Vector3::from(dv),
                dv_rot_vec_unit: Vector3::from(axis),
                dv_rot_vec_mag_radps: rate,
                burn_start_time_nanos: start_nanos,
            });
            let mut module = DvGuidance::new(DvGuidanceConfig::default());
            burn_output.connect_to(&mut module.burn_data_in_msg);
            module.init();
            module.update(&context(time_nanos));
            let output = module.att_ref_out_msg.read();
            assert_eq!(output.sigma_rn, Vector3::from(sigma), "t = {time_nanos} ns");
            assert_eq!(
                output.omega_rn_n_radps,
                Vector3::from(omega),
                "t = {time_nanos} ns"
            );
        }
    }

    // Not upstream: Δv along x and rotation axis z give a base burn frame
    // equal to the inertial frame (b2 = z × x = y, b3 = x × y = z), so the
    // reference is a pure rotation by ω_rot (t - t_start) about z:
    // σ_RN = tan(angle / 4) z and ω_RN = ω_rot z.
    #[test]
    fn rotates_about_the_third_burn_axis_at_the_commanded_rate() {
        let rate_radps = 0.1;
        let burn_output = Output::new(DvBurnCommandMsg {
            dv_inertial_cmd_mps: Vector3::new(3.0, 0.0, 0.0),
            dv_rot_vec_unit: Vector3::z(),
            dv_rot_vec_mag_radps: rate_radps,
            burn_start_time_nanos: 10_000_000_000,
        });
        let mut module = DvGuidance::new(DvGuidanceConfig::default());
        burn_output.connect_to(&mut module.burn_data_in_msg);
        module.init();

        for (time_nanos, burn_time_s) in [(0, -10.0), (10_000_000_000, 0.0), (25_000_000_000, 15.0)]
        {
            module.update(&context(time_nanos));
            let output = module.att_ref_out_msg.read();
            let angle = rate_radps * burn_time_s;
            assert_vector_close(
                output.sigma_rn,
                [0.0, 0.0, (angle / 4.0).tan()],
                1e-14,
                "sigma_RN",
            );
            assert_vector_close(
                output.omega_rn_n_radps,
                [0.0, 0.0, rate_radps],
                1e-15,
                "omega",
            );
            assert_eq!(output.domega_rn_n_radps2, Vector3::zeros());
        }
    }

    // Not upstream: a rotation axis parallel to Δv leaves the second axis
    // undefined; upstream's v3Normalize turns it, and so the third axis,
    // into zero rather than NaN, which zeroes the reference rate.
    #[test]
    fn degenerate_rotation_axis_gives_zero_axes_not_nan() {
        let burn_output = Output::new(DvBurnCommandMsg {
            dv_inertial_cmd_mps: Vector3::new(0.0, 2.0, 0.0),
            dv_rot_vec_unit: Vector3::y(),
            dv_rot_vec_mag_radps: 0.5,
            burn_start_time_nanos: 0,
        });
        let mut module = DvGuidance::new(DvGuidanceConfig::default());
        burn_output.connect_to(&mut module.burn_data_in_msg);
        module.init();
        module.update(&context(1_000_000_000));
        let output = module.att_ref_out_msg.read();
        assert!(output.sigma_rn.iter().all(|value| value.is_finite()));
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
    }

    // Not upstream: with an unconnected input every vector reads zero, the
    // burn frame collapses to zero rows and the rate is zero; nothing is NaN.
    #[test]
    fn unconnected_input_gives_a_finite_reference() {
        let mut module = DvGuidance::new(DvGuidanceConfig::default());
        module.init();
        module.update(&context(0));
        let output = module.att_ref_out_msg.read();
        assert!(output.sigma_rn.iter().all(|value| value.is_finite()));
        assert_eq!(output.omega_rn_n_radps, Vector3::zeros());
        assert_eq!(output.domega_rn_n_radps2, Vector3::zeros());
    }
}
