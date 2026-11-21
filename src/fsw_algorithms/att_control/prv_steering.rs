//! Principal-rotation-vector kinematic attitude steering law: port of upstream
//! Basilisk `prvSteering`.
//!
//! Like [`super::mrp_steering`], the module turns the MRP attitude error
//! `σ_B/R` into the body rate of a desired body frame `B*` relative to the
//! reference frame, but it saturates the principal rotation angle
//! `φ = 4 atan |σ_B/R|` about the principal axis `ê = σ_B/R / |σ_B/R|` as a
//! whole instead of each MRP component:
//!
//! `ω_B*/R = -f(φ) ê`, `f(φ) = (2 ω_max / π) atan((K1 φ + K3 φ³) π / (2 ω_max))`.
//!
//! The body-frame derivative of that rate, `ω'_B*/R = f(φ) f'(φ) ê` (with
//! `φ̇ = -f(φ)` along a fixed axis), is the feedforward for a rate servo such
//! as [`super::rate_servo_full_nonlinear`]. Below an MRP norm of `1e-11` the
//! axis is taken as `b̂_1`.

use std::f64::consts::FRAC_PI_2;

use nalgebra::Vector3;

use crate::messages::{AttitudeGuidanceMsg, Input, Output, RateCommandMsg};
use crate::{Module, SimulationContext};

/// MRP norm at or below which upstream takes `b̂_1` as the principal axis.
const PRINCIPAL_AXIS_MIN_SIGMA: f64 = 0.000_000_000_01;

/// User-settable parameters of [`PrvSteering`]. `Default` is upstream's
/// zero-initialized configuration; `omega_max_radps` must be set to a
/// positive value.
#[derive(Clone, Debug)]
pub struct PrvSteeringConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `K1` \[rad/s\]: proportional gain on the principal rotation
    /// angle error.
    pub k1: f64,
    /// Upstream `K3` \[rad/s\]: cubic gain on the principal rotation angle
    /// error in the saturation function.
    pub k3: f64,
    /// Upstream `omega_max` \[rad/s\]: largest commanded rate.
    pub omega_max_radps: f64,
}

impl Default for PrvSteeringConfig {
    fn default() -> Self {
        Self {
            name: "prvSteering".to_string(),
            k1: 0.0,
            k3: 0.0,
            omega_max_radps: 0.0,
        }
    }
}

/// PRV steering law: upstream `prvSteering`.
#[derive(Clone, Debug)]
pub struct PrvSteering {
    /// User-settable parameters.
    pub config: PrvSteeringConfig,
    /// Upstream `guidInMsg`: attitude guidance; only `sigma_br` is used.
    pub guid_in_msg: Input<AttitudeGuidanceMsg>,
    /// Upstream `rateCmdOutMsg`: commanded rate and its derivative.
    pub rate_cmd_out_msg: Output<RateCommandMsg>,
}

impl PrvSteering {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: PrvSteeringConfig) -> Self {
        Self {
            config,
            guid_in_msg: Input::default(),
            rate_cmd_out_msg: Output::default(),
        }
    }

    fn check_connections(&self) {
        if !self.guid_in_msg.is_connected() {
            log::error!("prvSteering.guid_in_msg is not connected");
        }
    }
}

/// Upstream `PRVSteeringLaw`: the commanded rate `ω_B*/R` \[rad/s\] and its
/// body-frame derivative \[rad/s^2\] for the MRP attitude error `sigma_br`.
pub fn prv_steering_law(config: &PrvSteeringConfig, sigma_br: Vector3<f64>) -> RateCommandMsg {
    let PrvSteeringConfig {
        k1,
        k3,
        omega_max_radps: omega_max,
        ..
    } = *config;
    let sigma_norm = sigma_br.norm();
    let e_hat = if sigma_norm > PRINCIPAL_AXIS_MIN_SIGMA {
        // A multiplication by 1/|σ|, as upstream.
        sigma_br * (1.0 / sigma_norm)
    } else {
        Vector3::x()
    };
    let phi = 4.0 * sigma_norm.atan();

    // The arctangent argument (π/2 / ω_max)(K1 φ + K3 φ³), in upstream's
    // operation order.
    let saturation_argument = FRAC_PI_2 / omega_max * (k1 * phi + k3 * phi * phi * phi);
    let rate = saturation_argument.atan() / FRAC_PI_2 * omega_max;
    // f(φ) f'(φ); upstream's pow(x, 2), which compilers fold to x * x.
    let rate_derivative =
        rate * ((3.0 * k3 * phi * phi + k1) / (saturation_argument * saturation_argument + 1.0));

    RateCommandMsg {
        omega_bast_r_b_radps: e_hat * -rate,
        omega_prime_bast_r_b_radps2: e_hat * rate_derivative,
    }
}

impl Module for PrvSteering {
    fn init(&mut self) {
        self.rate_cmd_out_msg.write(RateCommandMsg::default());
        self.check_connections();
    }

    /// Upstream `Reset_prvSteering` only checks the input connection.
    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let guidance = self.guid_in_msg.read();
        self.rate_cmd_out_msg
            .write(prv_steering_law(&self.config, guidance.sigma_br));
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use nalgebra::Vector3;

    use super::{PrvSteering, PrvSteeringConfig, prv_steering_law};
    use crate::Module;
    use crate::fsw_algorithms::att_control::rate_servo_full_nonlinear::test_support::{
        ServoInputs, assert_array_equal, context, guidance, run_upstream_schedule, rw_config,
        steering_test_servo,
    };
    use crate::messages::{AttitudeGuidanceMsg, Output, RateCommandMsg};
    use crate::utilities::astro_constants::D2R;

    /// The steering configuration of `test_PRV_Steering.py`.
    fn upstream_config() -> PrvSteeringConfig {
        PrvSteeringConfig {
            k1: 0.15,
            k3: 1.0,
            omega_max_radps: 1.5 * D2R,
            ..Default::default()
        }
    }

    /// `test_prvSteering`, both `simCase` parametrizations, with upstream's
    /// hard-coded truth and `accuracy = 1e-12`. The steering law feeds
    /// [`rateServoFullNonlinear`](super::super::rate_servo_full_nonlinear) with
    /// all-zero RW spin axes; the 0.5 s task runs to 1 s, the servo is reset,
    /// and the task runs on to 2 s.
    #[test]
    #[allow(clippy::excessive_precision)] // upstream truth, copied verbatim
    fn matches_upstream_prv_steering_unit_test() {
        let cases = [
            (
                Vector3::new(0.3, -0.5, 0.7),
                [
                    [-2.9352922876097969, 6.2831737715827778, -4.0554726129822907],
                    [-2.9352922876097969, 6.2831737715827778, -4.0554726129822907],
                    [-2.9353853745179044, 6.2833455830962901, -4.0556481491012084],
                    [-2.9352922876097969, 6.2831737715827778, -4.0554726129822907],
                    [-2.9353853745179044, 6.2833455830962901, -4.0556481491012084],
                ],
            ),
            (
                Vector3::zeros(),
                [
                    [-1.39, 3.79, -1.39],
                    [-1.39, 3.79, -1.39],
                    [-1.39005, 3.7901, -1.390075],
                    [-1.39, 3.79, -1.39],
                    [-1.39005, 3.7901, -1.390075],
                ],
            ),
        ];
        for (sim_case, (sigma_br, truth)) in cases.into_iter().enumerate() {
            let inputs = ServoInputs::new(
                AttitudeGuidanceMsg {
                    sigma_br,
                    ..guidance()
                },
                rw_config([[0.0; 3]; 4], 4),
            );
            let mut module = PrvSteering::new(PrvSteeringConfig::default());
            module.config = upstream_config();
            let mut servo = steering_test_servo();
            inputs.guidance.connect_to(&mut module.guid_in_msg);
            inputs.connect(&mut servo);
            module
                .rate_cmd_out_msg
                .connect_to(&mut servo.rate_steering_in_msg);

            let samples = run_upstream_schedule(Some(&mut module), &mut servo);
            assert_eq!(samples.len(), truth.len());
            for (index, (sample, expected)) in samples.iter().zip(truth).enumerate() {
                assert_array_equal(
                    *sample,
                    Vector3::from(expected),
                    1e-12,
                    &format!("simCase {sim_case} torqueRequestBody sample {index}"),
                );
            }
        }
    }

    /// Not upstream: a 90° error about a unit axis, against the law written
    /// out by hand.
    #[test]
    fn commands_a_rate_along_the_principal_axis() {
        let config = upstream_config();
        let axis = Vector3::new(2.0, -1.0, 2.0) / 3.0;
        let phi = PI / 2.0;
        let sigma = axis * (phi / 4.0).tan();
        let command = prv_steering_law(&config, sigma);

        let omega_max = config.omega_max_radps;
        let polynomial = config.k1 * phi + config.k3 * phi.powi(3);
        let argument = polynomial * PI / (2.0 * omega_max);
        let f = 2.0 * omega_max / PI * argument.atan();
        let f_prime = (config.k1 + 3.0 * config.k3 * phi * phi) / (1.0 + argument * argument);
        assert!((command.omega_bast_r_b_radps + f * axis).abs().max() <= 1e-15);
        assert!(
            (command.omega_prime_bast_r_b_radps2 - f * f_prime * axis)
                .abs()
                .max()
                <= 1e-15
        );
        assert!(command.omega_bast_r_b_radps.norm() < omega_max);
    }

    /// Not upstream: at or below `|σ| = 1e-11` the axis falls back to `b̂_1`;
    /// above it, the axis is that of `σ`.
    #[test]
    fn falls_back_to_the_first_body_axis_for_a_vanishing_error() {
        let config = upstream_config();
        let zero = prv_steering_law(&config, Vector3::zeros());
        assert_eq!(zero.omega_bast_r_b_radps, Vector3::zeros());
        assert_eq!(zero.omega_prime_bast_r_b_radps2, Vector3::zeros());

        let tiny = prv_steering_law(&config, Vector3::new(0.0, 1e-12, 0.0));
        let rate = tiny.omega_bast_r_b_radps;
        assert!(rate.x < 0.0 && rate.y == 0.0 && rate.z == 0.0, "{rate}");

        let small = prv_steering_law(&config, Vector3::new(0.0, 1e-10, 0.0));
        let rate = small.omega_bast_r_b_radps;
        assert!(rate.x == 0.0 && rate.y < 0.0 && rate.z == 0.0, "{rate}");
    }

    #[test]
    fn init_publishes_a_zero_rate_command() {
        let mut module = PrvSteering::new(PrvSteeringConfig::default());
        module.rate_cmd_out_msg.write(RateCommandMsg {
            omega_bast_r_b_radps: Vector3::new(1.0, 2.0, 3.0),
            omega_prime_bast_r_b_radps2: Vector3::new(4.0, 5.0, 6.0),
        });
        module.init();
        assert_eq!(module.rate_cmd_out_msg.read(), RateCommandMsg::default());
    }

    /// Not upstream: the module publishes the law for the guidance's
    /// `sigma_br`.
    #[test]
    fn publishes_the_law_for_the_guidance_attitude_error() {
        let guidance_out = Output::new(guidance());
        let mut module = PrvSteering::new(upstream_config());
        guidance_out.connect_to(&mut module.guid_in_msg);
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.rate_cmd_out_msg.read(),
            prv_steering_law(&module.config, guidance().sigma_br)
        );
    }
}
