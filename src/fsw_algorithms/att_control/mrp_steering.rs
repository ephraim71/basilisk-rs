//! MRP kinematic attitude steering law: port of upstream Basilisk
//! `mrpSteering`.
//!
//! The module turns the MRP attitude error `σ_B/R` into the body rate of a
//! desired body frame `B*` relative to the reference frame, axis by axis
//! (Schaub and Hogan, "Speed-constrained three-axes attitude control using
//! kinematic steering", Acta Astronautica, 2018):
//!
//! `ω_B*/R,i = -f(σ_i)`, `f(σ) = (2 ω_max / π) atan((K1 σ + K3 σ³) π / (2 ω_max))`.
//!
//! `f` behaves like `K1 σ` for small errors and saturates smoothly at
//! `±ω_max`; `K3` sets how quickly the limit is approached. The module also
//! publishes the body-frame derivative of that rate,
//! `ω'_B*/R,i = -∂f/∂σ_i σ̇_i` with `σ̇ = ¼ [B(σ)] ω_B*/R`, as the feedforward
//! of a rate servo such as [`super::rate_servo_full_nonlinear`]; it is zero
//! when `ignore_outer_loop_feedforward` is set.

use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::messages::{AttitudeGuidanceMsg, Input, Output, RateCommandMsg};
use crate::utilities::rigid_body_kinematics::b_mat_mrp;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`MrpSteering`]. `Default` is upstream's
/// zero-initialized configuration; `omega_max_radps` must be set to a
/// positive value.
#[derive(Clone, Debug)]
pub struct MrpSteeringConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `K1` \[rad/s\]: proportional gain on the MRP error.
    pub k1: f64,
    /// Upstream `K3` \[rad/s\]: cubic gain on the MRP error in the saturation
    /// function.
    pub k3: f64,
    /// Upstream `omega_max` \[rad/s\]: largest commanded rate per axis.
    pub omega_max_radps: f64,
    /// Upstream `ignoreOuterLoopFeedforward`: publish a zero rate derivative.
    pub ignore_outer_loop_feedforward: bool,
}

impl Default for MrpSteeringConfig {
    fn default() -> Self {
        Self {
            name: "mrpSteering".to_string(),
            k1: 0.0,
            k3: 0.0,
            omega_max_radps: 0.0,
            ignore_outer_loop_feedforward: false,
        }
    }
}

/// MRP steering law: upstream `mrpSteering`.
#[derive(Clone, Debug)]
pub struct MrpSteering {
    /// User-settable parameters.
    pub config: MrpSteeringConfig,
    /// Upstream `guidInMsg`: attitude guidance; only `sigma_br` is used.
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `rateCmdOutMsg`: commanded rate and its derivative.
    pub rate_cmd_out_msg: Output<RateCommandMsg>,
}

impl MrpSteering {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: MrpSteeringConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            rate_cmd_out_msg: Output::default(),
        }
    }

    fn check_connections(&self) {
        if !self.guid_in_msg.is_connected() {
            log::error!("mrpSteering.guid_in_msg is not connected");
        }
    }
}

/// Upstream `MRPSteeringLaw`: the commanded rate `ω_B*/R` \[rad/s\] and its
/// body-frame derivative \[rad/s^2\] for the MRP attitude error `sigma_br`.
pub fn mrp_steering_law(config: &MrpSteeringConfig, sigma_br: Vector3<f64>) -> RateCommandMsg {
    let MrpSteeringConfig {
        k1,
        k3,
        omega_max_radps: omega_max,
        ..
    } = *config;
    // The arctangent argument (π/2 / ω_max)(K1 σ + K3 σ³), in upstream's
    // operation order.
    let saturation_argument = sigma_br.map(|s| FRAC_PI_2 / omega_max * (k1 * s + k3 * s * s * s));
    let omega_bast_r_b_radps = saturation_argument.map(|x| -(x.atan() / FRAC_PI_2 * omega_max));

    let omega_prime_bast_r_b_radps2 = if config.ignore_outer_loop_feedforward {
        Vector3::zeros()
    } else {
        // σ̇ = ¼ [B(σ)] ω_B*/R as upstream's BmatMRP, m33MultV3, v3Scale.
        // m33MultV3 sums each row from +0.0; adding +0.0 to nalgebra's product
        // reproduces that bit for bit (it turns an all -0.0 row sum, which a
        // zero commanded rate gives, into upstream's +0.0).
        let sigma_dot = (b_mat_mrp(sigma_br) * omega_bast_r_b_radps + Vector3::zeros()) * 0.25;
        // ∂f/∂σ_i; upstream's pow(x, 2), which compilers fold to x * x.
        let slope = sigma_br.zip_map(&saturation_argument, |s, x| {
            (3.0 * k3 * s * s + k1) / (x * x + 1.0)
        });
        -slope.component_mul(&sigma_dot)
    };

    RateCommandMsg {
        omega_bast_r_b_radps,
        omega_prime_bast_r_b_radps2,
    }
}

impl Module for MrpSteering {
    fn init(&mut self) {
        self.rate_cmd_out_msg.write(RateCommandMsg::default());
        self.check_connections();
    }

    /// Upstream `Reset_mrpSteering` only checks the input connection.
    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let guidance = self.guid_in_msg.read();
        self.rate_cmd_out_msg
            .write(mrp_steering_law(&self.config, guidance.sigma_br));
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{MrpSteering, MrpSteeringConfig, mrp_steering_law};
    use crate::Module;
    use crate::fsw_algorithms::att_control::rate_servo_full_nonlinear::test_support::{
        ServoInputs, TASK_NANOS, UPSTREAM_SPIN_AXES, assert_array_equal, context, guidance,
        run_upstream_schedule, rw_config, steering_test_servo, upstream_true_torques,
    };
    use crate::messages::{AttitudeGuidanceMsg, Output, RateCommandMsg};
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::b_mat_mrp;

    fn config(k1: f64, k3: f64, omega_max_radps: f64) -> MrpSteeringConfig {
        MrpSteeringConfig {
            k1,
            k3,
            omega_max_radps,
            ..Default::default()
        }
    }

    /// Upstream `findTrueValues` (both test files): the steering rate and,
    /// always (upstream's `if 1:`), its feedforward derivative.
    fn upstream_true_values(
        sigma: Vector3<f64>,
        k1: f64,
        k3: f64,
        omega_max: f64,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let omega_ast = sigma.map(|s| {
            -(2.0 * omega_max / PI) * ((k1 * s + k3 * s * s * s) * PI / (2.0 * omega_max)).atan()
        });
        let sigma_p = 0.25 * b_mat_mrp(sigma) * omega_ast;
        let omega_ast_p = Vector3::from_fn(|i, _| {
            let s = sigma[i];
            let rate = (k1 + 3.0 * k3 * s.powi(2))
                / (1.0 + (k1 * s + k3 * s.powi(3)).powi(2) * (PI / (2.0 * omega_max)).powi(2))
                * sigma_p[i];
            -rate
        });
        (omega_ast, omega_ast_p)
    }

    /// `test_MRP_steeringUnit.py`, all eight parametrizations, with upstream's
    /// Python truth and `accuracy = 1e-12`: a 0.5 s task run to 1 s, all three
    /// samples checked.
    #[test]
    fn matches_upstream_mrp_steering_unit_test() {
        let mut cases = 0;
        for k1 in [0.15, 0.0] {
            for k3 in [1.0, 0.0] {
                for omega_max in [1.5 * D2R, 0.001 * D2R] {
                    let guidance_out = Output::new(guidance());
                    let mut module = MrpSteering::new(MrpSteeringConfig::default());
                    module.config.k1 = k1;
                    module.config.k3 = k3;
                    module.config.omega_max_radps = omega_max;
                    guidance_out.connect_to(&mut module.guid_in_msg);

                    let mut recorder = MessageRecorder::<RateCommandMsg>::default();
                    let mut simulation =
                        Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
                    simulation.connect(&module.rate_cmd_out_msg, &mut recorder.input_msg);
                    simulation.add_module("mrpSteering", &mut module, TASK_NANOS, 0);
                    simulation.add_module("recorder", &mut recorder, TASK_NANOS, 0);
                    simulation.run_for(2 * TASK_NANOS);
                    drop(simulation);

                    let (omega_ast, omega_ast_p) =
                        upstream_true_values(guidance().sigma_br, k1, k3, omega_max);
                    let what = format!("K1 {k1}, K3 {k3}, omegaMax {omega_max}");
                    assert_eq!(recorder.samples.len(), 3, "{what}");
                    for (index, sample) in recorder.samples.iter().enumerate() {
                        assert_array_equal(
                            sample.omega_bast_r_b_radps,
                            omega_ast,
                            1e-12,
                            &format!("{what} omega_BastR_B sample {index}"),
                        );
                        assert_array_equal(
                            sample.omega_prime_bast_r_b_radps2,
                            omega_ast_p,
                            1e-12,
                            &format!("{what} omegap_BastR_B sample {index}"),
                        );
                    }
                    cases += 1;
                }
            }
        }
        assert_eq!(cases, 8);
    }

    /// `test_MRP_steeringInt.py`, all eight parametrizations: the steering law
    /// feeding [`rateServoFullNonlinear`](super::super::rate_servo_full_nonlinear),
    /// with upstream's Python truth and `accuracy = 1e-12`. The 0.5 s task runs
    /// to 1 s, the servo is reset, and the task runs on to 2 s. As upstream, the
    /// truth reads the servo's stored RW configuration.
    #[test]
    fn matches_upstream_mrp_steering_integrated_test() {
        let mut cases = 0;
        for k1 in [0.15, 0.0] {
            for k3 in [1.0, 0.0] {
                for omega_max in [1.5 * D2R, 0.001] {
                    let inputs = ServoInputs::new(guidance(), rw_config(UPSTREAM_SPIN_AXES, 4));
                    let mut module = MrpSteering::new(MrpSteeringConfig::default());
                    module.config.k1 = k1;
                    module.config.k3 = k3;
                    module.config.omega_max_radps = omega_max;
                    let mut servo = steering_test_servo();
                    inputs.guidance.connect_to(&mut module.guid_in_msg);
                    inputs.connect(&mut servo);
                    module
                        .rate_cmd_out_msg
                        .connect_to(&mut servo.rate_steering_in_msg);

                    let samples = run_upstream_schedule(Some(&mut module), &mut servo);

                    let (omega_bast_r_b_radps, omega_prime_bast_r_b_radps2) =
                        upstream_true_values(guidance().sigma_br, k1, k3, omega_max);
                    let truth = upstream_true_torques(
                        &servo.config,
                        &guidance(),
                        servo.rw_config_params(),
                        &inputs.rw_availability.read(),
                        &RateCommandMsg {
                            omega_bast_r_b_radps,
                            omega_prime_bast_r_b_radps2,
                        },
                    );
                    assert_eq!(samples.len(), truth.len());
                    for (index, (sample, expected)) in samples.iter().zip(&truth).enumerate() {
                        assert_array_equal(
                            *sample,
                            *expected,
                            1e-12,
                            &format!(
                                "K1 {k1}, K3 {k3}, omegaMax {omega_max} torqueRequestBody sample {index}"
                            ),
                        );
                    }
                    cases += 1;
                }
            }
        }
        assert_eq!(cases, 8);
    }

    /// Not upstream (its truth always includes the feedforward): the flag
    /// zeroes the rate derivative and leaves the rate unchanged.
    #[test]
    fn ignoring_the_outer_loop_feedforward_zeroes_the_rate_derivative() {
        let sigma = guidance().sigma_br;
        let with = mrp_steering_law(&config(0.15, 1.0, 1.5 * D2R), sigma);
        let without = mrp_steering_law(
            &MrpSteeringConfig {
                ignore_outer_loop_feedforward: true,
                ..config(0.15, 1.0, 1.5 * D2R)
            },
            sigma,
        );
        assert_eq!(without.omega_bast_r_b_radps, with.omega_bast_r_b_radps);
        assert_eq!(without.omega_prime_bast_r_b_radps2, Vector3::zeros());
        assert!(with.omega_prime_bast_r_b_radps2.norm() > 0.0);
    }

    /// Not upstream: the law is linear, `-K1 σ`, for small errors, and each
    /// axis saturates below `ω_max` for large ones.
    #[test]
    fn is_linear_for_small_errors_and_saturates_at_omega_max() {
        let omega_max = 1.5 * D2R;
        let small = Vector3::new(1e-7, -2e-7, 3e-7);
        let rate = mrp_steering_law(&config(0.15, 1.0, omega_max), small).omega_bast_r_b_radps;
        assert!((rate + 0.15 * small).abs().max() <= 1e-15, "{rate}");

        let large = Vector3::new(0.9, -0.8, 0.7);
        let rate = mrp_steering_law(&config(100.0, 1.0, omega_max), large).omega_bast_r_b_radps;
        for (value, sigma) in rate.iter().zip(large.iter()) {
            assert_eq!(value.signum(), -sigma.signum());
            assert!(value.abs() < omega_max && value.abs() > 0.999 * omega_max);
        }
    }

    /// Not upstream: the feedforward is the time derivative of the rate
    /// command along the kinematics `σ̇ = ¼[B(σ)] ω_B*/R`, checked against a
    /// central difference.
    #[test]
    fn feedforward_is_the_derivative_of_the_rate_command() {
        let law = config(0.15, 1.0, 1.5 * D2R);
        let sigma = guidance().sigma_br;
        let command = mrp_steering_law(&law, sigma);
        let sigma_dot = 0.25 * b_mat_mrp(sigma) * command.omega_bast_r_b_radps;
        let step = 1e-4;
        let ahead = mrp_steering_law(&law, sigma + sigma_dot * step).omega_bast_r_b_radps;
        let behind = mrp_steering_law(&law, sigma - sigma_dot * step).omega_bast_r_b_radps;
        let derivative = (ahead - behind) / (2.0 * step);
        assert!(
            (derivative - command.omega_prime_bast_r_b_radps2)
                .abs()
                .max()
                <= 1e-12,
            "{derivative} vs {}",
            command.omega_prime_bast_r_b_radps2
        );
    }

    /// Not upstream: with no attitude error the commanded rate is -0.0 and,
    /// because upstream's m33MultV3 sums from +0.0, its derivative is
    /// `-K1 · (+0.0) = -0.0`, as the compiled upstream C gives.
    #[test]
    fn a_zero_error_gives_upstreams_signed_zero_rates() {
        let command = mrp_steering_law(&config(0.15, 1.0, 1.5 * D2R), Vector3::zeros());
        for (rate, derivative) in command
            .omega_bast_r_b_radps
            .iter()
            .zip(command.omega_prime_bast_r_b_radps2.iter())
        {
            assert_eq!(rate.to_bits(), (-0.0_f64).to_bits());
            assert_eq!(derivative.to_bits(), (-0.0_f64).to_bits());
        }
    }

    #[test]
    fn init_publishes_a_zero_rate_command() {
        let mut module = MrpSteering::new(MrpSteeringConfig::default());
        module.rate_cmd_out_msg.write(RateCommandMsg {
            omega_bast_r_b_radps: Vector3::new(1.0, 2.0, 3.0),
            omega_prime_bast_r_b_radps2: Vector3::new(4.0, 5.0, 6.0),
        });
        module.init();
        assert_eq!(module.rate_cmd_out_msg.read(), RateCommandMsg::default());
    }

    /// Not upstream: the module reads only `sigma_br` from the guidance.
    #[test]
    fn uses_only_the_attitude_error_of_the_guidance() {
        let sigma_br = Vector3::new(0.1, 0.2, -0.3);
        let guidance_out = Output::new(AttitudeGuidanceMsg {
            sigma_br,
            ..guidance()
        });
        let mut module = MrpSteering::new(config(0.15, 1.0, 1.5 * D2R));
        guidance_out.connect_to(&mut module.guid_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.rate_cmd_out_msg.read(),
            mrp_steering_law(&module.config, sigma_br)
        );
    }
}
