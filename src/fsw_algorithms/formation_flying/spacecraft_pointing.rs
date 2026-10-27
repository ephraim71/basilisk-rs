//! Attitude reference that points a body-fixed vector of a deputy spacecraft
//! at a chief spacecraft: port of upstream Basilisk `spacecraftPointing`.
//!
//! Every update builds the frame `R` whose first axis is the unit vector from
//! the deputy to the chief, `σ_RN` its MRP set. The reference rate and
//! acceleration come from backward differences between updates:
//! `ω_RN = 4 [B̄]^T σ̇` (Schaub & Junkins Eq. 3.163, with `[B]^{-1} =
//! [B]^T / (1 + σ²)²`) uses the average of `[B(σ)]` and of `1 / (1 + σ²)²` at
//! the previous and the current `σ_RN`, switching the previous set to its
//! shadow when that gives the shorter step; `ω̇_RN` differences successive
//! rates. The first two rates and the first three accelerations after a reset
//! are not yet meaningful and are published as zero.
//!
//! The published attitude is `σ_R1N = σ_RN ⊕ σ_BA`, where `A` is a frame built
//! at reset around the configured body vector (an antenna, say): when `B`
//! tracks `R1`, `A` tracks `R` and the body vector points at the chief.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{AttitudeReferenceMsg, Input, NavigationTranslationMsg, Output};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{
    add_mrp, b_mat_mrp, dcm_to_mrp, mrp_switch, mrp_to_dcm,
};
use crate::{Module, SimulationContext};

/// Number of updates after a reset for which upstream publishes a zero rate.
const RATE_WARM_UP_UPDATES: u8 = 2;
/// Number of updates after a reset for which upstream publishes a zero
/// acceleration.
const ACCELERATION_WARM_UP_UPDATES: u8 = 3;

/// User-settable parameters of [`SpacecraftPointing`]. `Default` is
/// upstream's zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct SpacecraftPointingConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `alignmentVector_B`: body-frame vector (e.g. an antenna
    /// boresight) to point at the chief; need not be a unit vector. Read at
    /// reset.
    pub alignment_vector_b: Vector3<f64>,
}

impl Default for SpacecraftPointingConfig {
    fn default() -> Self {
        Self {
            name: "spacecraftPointing".to_string(),
            alignment_vector_b: Vector3::zeros(),
        }
    }
}

/// Deputy-to-chief pointing attitude reference: upstream `spacecraftPointing`.
#[derive(Clone, Debug)]
pub struct SpacecraftPointing {
    /// User-settable parameters.
    pub config: SpacecraftPointingConfig,
    /// Upstream `chiefPositionInMsg`: chief inertial position.
    pub chief_position_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `deputyPositionInMsg`: deputy inertial position.
    pub deputy_position_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `attReferenceOutMsg`: attitude reference of the deputy.
    pub att_reference_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `sigma_BA`, set at reset from `alignment_vector_b`.
    sigma_ba: Vector3<f64>,
    /// Upstream `old_sigma_RN`: `σ_RN` of the previous update.
    old_sigma_rn: Vector3<f64>,
    /// Upstream `old_omega_RN_N` \[rad/s\]: unmasked `ω_RN` of the previous
    /// update.
    old_omega_rn_n_radps: Vector3<f64>,
    /// Upstream `i`: updates since the last reset, counted up to
    /// [`ACCELERATION_WARM_UP_UPDATES`].
    warm_up_updates: u8,
    /// Upstream `priorTime` \[ns\]: time of the previous update.
    prior_time_nanos: u64,
}

impl SpacecraftPointing {
    /// Builds the module with `config` and unconnected ports.
    pub fn new(config: SpacecraftPointingConfig) -> Self {
        Self {
            config,
            chief_position_in_msg: Input::default(),
            deputy_position_in_msg: Input::default(),
            att_reference_out_msg: Output::default(),
            sigma_ba: Vector3::zeros(),
            old_sigma_rn: Vector3::zeros(),
            old_omega_rn_n_radps: Vector3::zeros(),
            warm_up_updates: 0,
            prior_time_nanos: 0,
        }
    }

    /// Upstream `sigma_BA`: MRP set of the body frame relative to the frame
    /// `A` built around `alignment_vector_b` at the last reset.
    pub fn sigma_ba(&self) -> Vector3<f64> {
        self.sigma_ba
    }

    /// Upstream `Reset_spacecraftPointing`.
    ///
    /// Upstream quirk, kept: `priorTime` is not reset, so the first update
    /// after construction differences against t = 0 and the first update
    /// after a later reset against the last update before it. The rates of
    /// those updates are masked by the warm-up counter.
    fn reset_state(&mut self) {
        if !self.chief_position_in_msg.is_connected() {
            log::error!("spacecraftPointing.chief_position_in_msg is not connected");
        }
        if !self.deputy_position_in_msg.is_connected() {
            log::error!("spacecraftPointing.deputy_position_in_msg is not connected");
        }

        let alignment = self.config.alignment_vector_b;
        // Upstream crosses with the unnormalized alignment vector here.
        let dcm_ab = pointing_frame(normalize_or_zero(alignment), alignment);
        self.sigma_ba = -dcm_to_mrp(dcm_ab);
        self.old_sigma_rn = Vector3::zeros();
        self.old_omega_rn_n_radps = Vector3::zeros();
        self.warm_up_updates = 0;
    }

    /// Upstream `Update_spacecraftPointing` after the message reads.
    fn attitude_reference(
        &mut self,
        chief: &NavigationTranslationMsg,
        deputy: &NavigationTranslationMsg,
        current_sim_nanos: u64,
    ) -> AttitudeReferenceMsg {
        let rho_hat_n = normalize_or_zero(chief.position_bn_n_m - deputy.position_bn_n_m);
        let sigma_rn = dcm_to_mrp(pointing_frame(rho_hat_n, rho_hat_n));

        // Backward difference against the previous set or its shadow,
        // whichever is closer. With a 0 threshold MRPswitch returns the shadow
        // of any nonzero set; on a tie upstream takes the shadow.
        let shadow_old_sigma_rn = mrp_switch(self.old_sigma_rn, 0.0);
        let mut delta_sigma_rn = sigma_rn - self.old_sigma_rn;
        let delta_sigma_rn_shadow = sigma_rn - shadow_old_sigma_rn;
        if delta_sigma_rn.norm() >= delta_sigma_rn_shadow.norm() {
            delta_sigma_rn = delta_sigma_rn_shadow;
            self.old_sigma_rn = shadow_old_sigma_rn;
        }

        let dt_s = diff_nanos_to_seconds(current_sim_nanos, self.prior_time_nanos);
        self.prior_time_nanos = current_sim_nanos;
        // Upstream quirk, kept: a zero step (the first update at t = 0) gives
        // NaN rates, which the warm-up masking hides from the output.
        let sigma_dot_rn = delta_sigma_rn * (1.0 / dt_s);

        let mut omega_rn_n = mean_rate(self.old_sigma_rn, sigma_rn, sigma_dot_rn);
        let mut domega_rn_n = (omega_rn_n - self.old_omega_rn_n_radps) * (1.0 / dt_s);
        self.old_omega_rn_n_radps = omega_rn_n;
        self.old_sigma_rn = sigma_rn;

        if self.warm_up_updates < RATE_WARM_UP_UPDATES {
            omega_rn_n = Vector3::zeros();
        }
        if self.warm_up_updates < ACCELERATION_WARM_UP_UPDATES {
            domega_rn_n = Vector3::zeros();
            self.warm_up_updates += 1;
        }

        AttitudeReferenceMsg {
            sigma_rn: add_mrp(sigma_rn, self.sigma_ba),
            omega_rn_n_radps: omega_rn_n,
            domega_rn_n_radps2: domega_rn_n,
        }
    }
}

/// DCM whose rows are a right-handed frame with first axis `x_hat`: the
/// second axis is `ẑ × v`, or `v × ŷ` when that is shorter than 1e-6
/// (`v` along `ẑ`), normalized; the third is `x_hat × y_hat`, normalized.
/// Upstream passes the unnormalized alignment vector as `v` at reset and
/// `x_hat` itself at update.
fn pointing_frame(x_hat: Vector3<f64>, v: Vector3<f64>) -> Matrix3<f64> {
    let mut y_axis = Vector3::z().cross(&v);
    if y_axis.norm() < 1e-6 {
        y_axis = v.cross(&Vector3::y());
    }
    let y_hat = normalize_or_zero(y_axis);
    let z_hat = normalize_or_zero(x_hat.cross(&y_hat));
    Matrix3::from_rows(&[x_hat.transpose(), y_hat.transpose(), z_hat.transpose()])
}

/// `ω_RN` \[rad/s\] in N components over a step from `old_sigma_rn` to
/// `sigma_rn` at the mean MRP rate `sigma_dot_rn`: `4 s̄ [B̄]^T σ̇` in R
/// components, with `[B̄]` and `s̄` the averages of `[B(σ)]` and of
/// `1 / (1 + σ²)²` over both ends, rotated to N with `[NR] = [C(-σ_RN)]`.
fn mean_rate(
    old_sigma_rn: Vector3<f64>,
    sigma_rn: Vector3<f64>,
    sigma_dot_rn: Vector3<f64>,
) -> Vector3<f64> {
    let old_sigma_squared = old_sigma_rn.norm_squared();
    let sigma_squared = sigma_rn.norm_squared();
    let mean_b = (b_mat_mrp(old_sigma_rn) + b_mat_mrp(sigma_rn)) / 2.0;
    let mean_scale = 0.5
        * (1.0 / ((1.0 + sigma_squared) * (1.0 + sigma_squared))
            + 1.0 / ((1.0 + old_sigma_squared) * (1.0 + old_sigma_squared)));
    let omega_rn_r = (mean_b.transpose() * mean_scale) * sigma_dot_rn * 4.0;
    mrp_to_dcm(-sigma_rn) * omega_rn_r
}

impl Module for SpacecraftPointing {
    fn init(&mut self) {
        self.att_reference_out_msg
            .write(AttitudeReferenceMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let chief = self.chief_position_in_msg.read();
        let deputy = self.deputy_position_in_msg.read();
        let reference = self.attitude_reference(&chief, &deputy, context.current_sim_nanos);
        self.att_reference_out_msg.write(reference);
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{SpacecraftPointing, SpacecraftPointingConfig, pointing_frame};
    use crate::messages::{AttitudeReferenceMsg, NavigationTranslationMsg, Output};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::rigid_body_kinematics::{add_mrp, dcm_to_mrp, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.1)`.
    const TASK_RATE_NANOS: u64 = 100_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    fn position(position_bn_n_m: Vector3<f64>) -> NavigationTranslationMsg {
        NavigationTranslationMsg {
            position_bn_n_m,
            ..Default::default()
        }
    }

    /// Upstream `isArrayEqual`: absolute tolerance per component, NaN fails.
    fn assert_array_equal(actual: Vector3<f64>, truth: [f64; 3], accuracy: f64, what: &str) {
        for (index, (value, expected)) in actual.iter().zip(truth).enumerate() {
            assert!(
                (value - expected).abs() <= accuracy,
                "{what}[{index}] = {value:.17e}, truth {expected:.17e}"
            );
        }
    }

    /// Runs `test_spacecraftPointing.py`'s simulation: a 0.1 s task, first
    /// run to t = 0.1 s on the first chief position, then one 0.1 s run per
    /// further position, rewriting the input messages before each run. The
    /// chief moves on the unit circle in steps of 1 mrad, the deputy stays at
    /// the origin. Returns the six recorded samples (t = 0 ... 0.5 s).
    fn run_upstream_scenario(
        alignment_vector_b: Vector3<f64>,
    ) -> (SpacecraftPointing, Vec<AttitudeReferenceMsg>) {
        let chief_positions: Vec<_> = (0..5)
            .map(|k| {
                let angle = 0.001 * f64::from(k);
                Vector3::new(angle.cos(), angle.sin(), 0.0)
            })
            .collect();

        let chief_out = Output::new(position(chief_positions[0]));
        let deputy_out = Output::new(position(Vector3::zeros()));
        let mut module = SpacecraftPointing::new(SpacecraftPointingConfig::default());
        chief_out.connect_to(&mut module.chief_position_in_msg);
        deputy_out.connect_to(&mut module.deputy_position_in_msg);
        module.config.alignment_vector_b = alignment_vector_b;

        let mut recorder = MessageRecorder::<AttitudeReferenceMsg>::default();
        {
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&module.att_reference_out_msg, &mut recorder.input_msg);
            simulation.add_module("spacecraftPointing", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("recorder", &mut recorder, TASK_RATE_NANOS, 0);
            simulation.run_for(TASK_RATE_NANOS);
            for chief_position in &chief_positions[1..] {
                chief_out.write(position(*chief_position));
                deputy_out.write(position(Vector3::zeros()));
                simulation.run_for(TASK_RATE_NANOS);
            }
        }
        (module, recorder.samples)
    }

    /// Upstream `test_spacecraftPointing.py`, case 1 (alignment vector along
    /// body x): `sigma_RN`, `omega_RN_N` and `domega_RN_N` of all six samples
    /// against upstream's hard-coded truth and accuracies.
    #[test]
    fn matches_upstream_spacecraft_pointing_unit_test_regular_alignment() {
        let (_, samples) = run_upstream_scenario(Vector3::new(1.0, 0.0, 0.0));
        assert_eq!(samples.len(), 6);

        let true_sigma = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0002500000052],
            [0.0, 0.0, 0.0005000000417],
            [0.0, 0.0, 0.0007500001406],
            [0.0, 0.0, 0.001000000333],
        ];
        let true_omega = [
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
            [0.0, 0.0, 0.01],
            [0.0, 0.0, 0.01],
            [0.0, 0.0, 0.01],
            [0.0, 0.0, 0.01],
        ];
        for (index, sample) in samples.iter().enumerate() {
            let what = |name: &str| format!("{name} sample {index}");
            assert_array_equal(sample.sigma_rn, true_sigma[index], 1e-12, &what("sigma_RN"));
            assert_array_equal(
                sample.omega_rn_n_radps,
                true_omega[index],
                1e-9,
                &what("omega_RN_N"),
            );
            assert_array_equal(
                sample.domega_rn_n_radps2,
                [0.0; 3],
                1e-12,
                &what("domega_RN_N"),
            );
        }
    }

    /// Upstream `test_spacecraftPointing.py`, case 2 (alignment vector along
    /// body z, the fallback branch of the frame construction): `sigma_BA`
    /// against upstream's truth with `isVectorEqual` (norm of the error) at
    /// accuracy 1e-12.
    #[test]
    fn matches_upstream_spacecraft_pointing_unit_test_alignment_along_z() {
        let (module, samples) = run_upstream_scenario(Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(samples.len(), 6);
        let truth = Vector3::new(-1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0);
        assert!(
            (module.sigma_ba() - truth).norm() <= 1e-12,
            "sigma_BA = {}",
            module.sigma_ba()
        );
    }

    /// Not upstream: `σ_BA` is the inverse of the frame built around the
    /// alignment vector, so `[C(σ_BA)]^T` maps body x to the normalized
    /// alignment vector, and the published attitude carries it.
    #[test]
    fn alignment_frame_maps_its_first_axis_to_the_alignment_vector() {
        let alignment = Vector3::new(0.3, -2.0, 0.7);
        let mut module = SpacecraftPointing::new(SpacecraftPointingConfig {
            alignment_vector_b: alignment,
            ..Default::default()
        });
        let chief_out = Output::new(position(Vector3::new(3.0, 4.0, 12.0)));
        let deputy_out = Output::new(position(Vector3::new(1.0, 1.0, 1.0)));
        chief_out.connect_to(&mut module.chief_position_in_msg);
        deputy_out.connect_to(&mut module.deputy_position_in_msg);
        module.init();

        // [AB] has the unit alignment vector as its first row: [BA] x = a_hat.
        let dcm_ba = mrp_to_dcm(module.sigma_ba());
        assert!((dcm_ba * Vector3::x() - alignment.normalize()).amax() < 1e-14);

        module.update(&context(TASK_RATE_NANOS));
        let reference = module.att_reference_out_msg.read();
        // When B tracks R1 the alignment vector points from deputy to chief.
        let dcm_r1n = mrp_to_dcm(reference.sigma_rn);
        let rho_hat_n = Vector3::new(2.0, 3.0, 11.0).normalize();
        let alignment_n = dcm_r1n.transpose() * alignment.normalize();
        assert!((alignment_n - rho_hat_n).amax() < 1e-14, "{alignment_n}");
        let expected = add_mrp(
            dcm_to_mrp(pointing_frame(rho_hat_n, rho_hat_n)),
            module.sigma_ba(),
        );
        assert!((reference.sigma_rn - expected).amax() < 1e-15);
    }

    /// Not upstream: a faster rotation of the line of sight about inertial z
    /// over more updates than upstream's test; once the warm-up is over the
    /// rate is the commanded rate and the acceleration zero.
    #[test]
    fn rates_follow_a_constant_rotation_of_the_line_of_sight() {
        let rate_radps = 0.02;
        let mut module = SpacecraftPointing::new(SpacecraftPointingConfig {
            alignment_vector_b: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        });
        let chief_out = Output::new(position(Vector3::new(0.0, 0.0, 1.0)));
        let deputy_out = Output::new(position(Vector3::zeros()));
        chief_out.connect_to(&mut module.chief_position_in_msg);
        deputy_out.connect_to(&mut module.deputy_position_in_msg);
        module.init();

        let mut outputs = Vec::new();
        for k in 0..8_u32 {
            let angle = rate_radps * 0.1 * f64::from(k);
            chief_out.write(position(Vector3::new(angle.cos(), angle.sin(), 0.0) * 5.0));
            module.update(&context(u64::from(k) * TASK_RATE_NANOS));
            outputs.push(module.att_reference_out_msg.read());
        }
        assert_eq!(outputs[1].omega_rn_n_radps, Vector3::zeros());
        assert_eq!(outputs[2].domega_rn_n_radps2, Vector3::zeros());
        for output in &outputs[3..] {
            assert!((output.omega_rn_n_radps - Vector3::new(0.0, 0.0, rate_radps)).amax() < 1e-9);
            assert!(output.domega_rn_n_radps2.amax() < 1e-9);
        }
    }

    /// Not upstream: the MRP set of the line of sight switches to its shadow
    /// past a half turn; the backward difference then uses the shadow of the
    /// previous set and the rate stays continuous.
    #[test]
    fn rate_is_continuous_across_an_mrp_switch() {
        let rate_radps = 0.5;
        let mut module = SpacecraftPointing::new(SpacecraftPointingConfig {
            alignment_vector_b: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        });
        let chief_out = Output::new(position(Vector3::new(1.0, 0.0, 0.0)));
        let deputy_out = Output::new(position(Vector3::zeros()));
        chief_out.connect_to(&mut module.chief_position_in_msg);
        deputy_out.connect_to(&mut module.deputy_position_in_msg);
        module.init();

        let mut switched = false;
        let mut previous_sigma: Option<Vector3<f64>> = None;
        // From 170 deg to about 181 deg about z, through the half turn where
        // the set switches.
        for k in 0..40_u32 {
            let angle = 170.0_f64.to_radians() + rate_radps * 0.01 * f64::from(k);
            chief_out.write(position(Vector3::new(angle.cos(), angle.sin(), 0.0)));
            module.update(&context(u64::from(k + 1) * 10_000_000));
            let output = module.att_reference_out_msg.read();
            if let Some(previous) = previous_sigma {
                switched |= (output.sigma_rn - previous).norm() > 1.0;
            }
            previous_sigma = Some(output.sigma_rn);
            // The averaged backward difference is accurate to a few 1e-6
            // here; a difference against the unswitched set would be off by
            // about 2 / 0.01 s.
            if k >= 3 {
                assert!(
                    (output.omega_rn_n_radps - Vector3::new(0.0, 0.0, rate_radps)).amax() < 1e-5,
                    "k = {k}: {:?}",
                    output.omega_rn_n_radps
                );
            }
        }
        assert!(switched, "the MRP set never switched");
    }

    /// Not upstream: a reset restarts the warm-up masking and the backward
    /// differences from a zero attitude, as upstream's reset does.
    #[test]
    fn reset_restarts_the_warm_up() {
        let mut module = SpacecraftPointing::new(SpacecraftPointingConfig {
            alignment_vector_b: Vector3::new(1.0, 0.0, 0.0),
            ..Default::default()
        });
        let chief_out = Output::new(position(Vector3::new(1.0, 0.0, 0.0)));
        let deputy_out = Output::new(position(Vector3::zeros()));
        chief_out.connect_to(&mut module.chief_position_in_msg);
        deputy_out.connect_to(&mut module.deputy_position_in_msg);
        module.init();
        for k in 0..6_u32 {
            let angle = 0.001 * f64::from(k);
            chief_out.write(position(Vector3::new(angle.cos(), angle.sin(), 0.0)));
            module.update(&context(u64::from(k) * TASK_RATE_NANOS));
        }
        assert!(module.att_reference_out_msg.read().omega_rn_n_radps.z > 0.0);

        module.reset(&context(6 * TASK_RATE_NANOS));
        assert_eq!(module.old_sigma_rn, Vector3::zeros());
        assert_eq!(module.old_omega_rn_n_radps, Vector3::zeros());
        assert_eq!(module.warm_up_updates, 0);
        // priorTime survives the reset, as upstream.
        assert_eq!(module.prior_time_nanos, 5 * TASK_RATE_NANOS);
        for k in 6..8_u64 {
            module.update(&context(k * TASK_RATE_NANOS));
            assert_eq!(
                module.att_reference_out_msg.read().omega_rn_n_radps,
                Vector3::zeros()
            );
        }
    }

    /// Not upstream: config set after construction takes effect at init.
    #[test]
    fn alignment_vector_is_read_at_reset() {
        let mut module = SpacecraftPointing::new(SpacecraftPointingConfig::default());
        module.config.alignment_vector_b = Vector3::new(0.0, 0.0, 2.0);
        module.init();
        assert!(
            (module.sigma_ba() - Vector3::new(-1.0 / 3.0, 1.0 / 3.0, -1.0 / 3.0)).norm() < 1e-15
        );
        module.config.alignment_vector_b = Vector3::new(1.0, 0.0, 0.0);
        module.reset(&context(0));
        assert_eq!(module.sigma_ba(), Vector3::zeros());
    }
}
