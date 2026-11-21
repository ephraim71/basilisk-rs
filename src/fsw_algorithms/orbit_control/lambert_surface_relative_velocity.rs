//! Inertial velocity for a velocity relative to a rotating body's surface:
//! port of upstream Basilisk
//! `fswAlgorithms/orbitControl/lambertSurfaceRelativeVelocity`.
//!
//! Every update takes the Lambert target position `r2_N` as the place on the
//! body, builds the local East-North-Up surface frame `S` there, and publishes
//! the inertial velocity that moves at the configured velocity relative to
//! the surface: the body's rotation `omega x r` plus the relative velocity
//! rotated out of `S`. East is along `omega x r`, or along `z x r` when the
//! body does not rotate (`|omega| <= 1e-6` rad/s). The maneuver time comes
//! from the configuration.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{DesiredVelocityMsg, EphemerisMsg, Input, LambertProblemMsg, Output};
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream threshold [rad/s] below which the body counts as not rotating.
const MIN_ROTATION_RATE_RADPS: f64 = 1e-6;

/// User-settable parameters of [`LambertSurfaceRelativeVelocity`] (upstream's
/// setters).
#[derive(Clone, Debug)]
pub struct LambertSurfaceRelativeVelocityConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `vRelativeDesired_S` [m/s]: desired velocity relative to the
    /// surface, East-North-Up components. Upstream leaves it uninitialized;
    /// here it defaults to zero.
    pub v_relative_desired_s_mps: Vector3<f64>,
    /// Upstream `time` \[s\]: the maneuver time published with the velocity.
    pub time_s: f64,
}

impl Default for LambertSurfaceRelativeVelocityConfig {
    fn default() -> Self {
        Self {
            name: "lambertSurfaceRelativeVelocity".to_string(),
            v_relative_desired_s_mps: Vector3::zeros(),
            time_s: 0.0,
        }
    }
}

/// Turns a surface-relative velocity at the Lambert target into an inertial
/// one: upstream `LambertSurfaceRelativeVelocity`.
#[derive(Clone, Debug)]
pub struct LambertSurfaceRelativeVelocity {
    /// Module configuration.
    pub config: LambertSurfaceRelativeVelocityConfig,
    /// Upstream `lambertProblemInMsg`: only its target position `r2_N` is
    /// used.
    pub lambert_problem_in_msg: Input<LambertProblemMsg>,
    /// Upstream `ephemerisInMsg`: attitude and rotation rate of the body.
    pub ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `desiredVelocityOutMsg`: the inertial velocity to reach.
    pub desired_velocity_out_msg: Output<DesiredVelocityMsg>,
}

impl LambertSurfaceRelativeVelocity {
    /// Builds the module with unconnected ports.
    pub fn new(config: LambertSurfaceRelativeVelocityConfig) -> Self {
        Self {
            config,
            lambert_problem_in_msg: Input::default(),
            ephemeris_in_msg: Input::default(),
            desired_velocity_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset`: the module keeps no state, so a reset only reports
    /// missing inputs.
    fn check_connections(&self) {
        if !self.lambert_problem_in_msg.is_connected() {
            log::error!("lambertSurfaceRelativeVelocity.lambert_problem_in_msg is not connected");
        }
        if !self.ephemeris_in_msg.is_connected() {
            log::error!("lambertSurfaceRelativeVelocity.ephemeris_in_msg is not connected");
        }
    }

    fn desired_velocity(&self) -> DesiredVelocityMsg {
        let r_bn_n = self.lambert_problem_in_msg.read().r2_n_m;
        let ephemeris = self.ephemeris_in_msg.read();
        let dcm_pn = mrp_to_dcm(ephemeris.sigma_bn);
        let omega_pn_n = dcm_pn.transpose() * ephemeris.omega_bn_b_radps;

        // East, North, Up.
        let east_axis = if omega_pn_n.norm() > MIN_ROTATION_RATE_RADPS {
            omega_pn_n
        } else {
            Vector3::z()
        };
        let s1_hat_n = normalize_or_zero(east_axis.cross(&r_bn_n));
        let s3_hat_n = normalize_or_zero(r_bn_n);
        let s2_hat_n = normalize_or_zero(s3_hat_n.cross(&s1_hat_n));
        let dcm_sn = Matrix3::from_rows(&[
            s1_hat_n.transpose(),
            s2_hat_n.transpose(),
            s3_hat_n.transpose(),
        ]);

        DesiredVelocityMsg {
            v_desired_n_mps: omega_pn_n.cross(&r_bn_n)
                + dcm_sn.transpose() * self.config.v_relative_desired_s_mps,
            maneuver_time_s: self.config.time_s,
        }
    }
}

impl Module for LambertSurfaceRelativeVelocity {
    fn init(&mut self) {
        self.desired_velocity_out_msg
            .write(DesiredVelocityMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.desired_velocity_out_msg.write(self.desired_velocity());
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use super::{LambertSurfaceRelativeVelocity, LambertSurfaceRelativeVelocityConfig};
    use crate::Module;
    use crate::fsw_algorithms::orbit_control::test_support::{conic_state, context, elements};
    use crate::messages::{EphemerisMsg, LambertProblemMsg, Output};
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, prv_to_mrp};

    /// Upstream `test_lambertSurfaceRelativeVelocity`: relative velocities,
    /// maneuver times, true anomalies, planet rotation angles and rates, with
    /// the truth built from the surface-frame definition, at 1e-4.
    #[test]
    fn matches_upstream_lambert_surface_relative_velocity_test() {
        let accuracy = 1e-4;
        let mu = 3.986004418e14;
        for v_relative in [Vector3::zeros(), Vector3::new(100.0, -300.0, 200.0)] {
            for tm in [1.0e3, 2.0e3] {
                for f_deg in [0.0, 70.0] {
                    for rotation_deg in [0.0, 230.0] {
                        for omega_pn_n in [Vector3::zeros(), Vector3::new(1.0, -3.0, 2.0)] {
                            let what = format!(
                                "v_rel={v_relative:?}, tm={tm}, f={f_deg}, rot={rotation_deg}, \
                                 omega={omega_pn_n:?}"
                            );
                            let omega_hat = if omega_pn_n.norm() > 1e-6 {
                                omega_pn_n / omega_pn_n.norm()
                            } else {
                                Vector3::z()
                            };
                            let sigma_pn = prv_to_mrp(omega_hat * (rotation_deg * D2R));
                            let omega_pn_p = mrp_to_dcm(sigma_pn) * omega_pn_n;
                            let oe = elements(
                                10000.0 * 1000.0,
                                0.1,
                                10.0 * D2R,
                                20.0 * D2R,
                                30.0 * D2R,
                                f_deg * D2R,
                            );
                            let (r0, _) = conic_state(mu, &oe);

                            let mut module = LambertSurfaceRelativeVelocity::new(
                                LambertSurfaceRelativeVelocityConfig {
                                    v_relative_desired_s_mps: v_relative,
                                    time_s: tm,
                                    ..Default::default()
                                },
                            );
                            let ephemeris = Output::new(EphemerisMsg {
                                sigma_bn: sigma_pn,
                                omega_bn_b_radps: omega_pn_p,
                                ..Default::default()
                            });
                            let problem = Output::new(LambertProblemMsg {
                                r2_n_m: r0,
                                ..Default::default()
                            });
                            ephemeris.connect_to(&mut module.ephemeris_in_msg);
                            problem.connect_to(&mut module.lambert_problem_in_msg);
                            module.init();
                            module.update(&context(0));
                            let output = module.desired_velocity_out_msg.read();

                            let east_axis = if omega_pn_n.norm() > 1e-6 {
                                omega_pn_n
                            } else {
                                Vector3::z()
                            };
                            let s1 = east_axis.cross(&r0) / east_axis.cross(&r0).norm();
                            let s3 = r0 / r0.norm();
                            let s2 = s3.cross(&s1) / s3.cross(&s1).norm();
                            let dcm_sn = Matrix3::from_rows(&[
                                s1.transpose(),
                                s2.transpose(),
                                s3.transpose(),
                            ]);
                            let v_true = omega_pn_n.cross(&r0) + dcm_sn.transpose() * v_relative;

                            assert!(
                                (output.v_desired_n_mps - v_true).amax() <= accuracy,
                                "{what}: {output:?} vs {v_true}"
                            );
                            assert!((output.maneuver_time_s - tm).abs() <= accuracy, "{what}");
                        }
                    }
                }
            }
        }
    }
}
