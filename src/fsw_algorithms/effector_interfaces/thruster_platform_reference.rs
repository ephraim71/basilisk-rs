//! Tip-and-tilt reference for a gimballed thruster platform: port of upstream
//! Basilisk `thrusterPlatformReference`
//! (`fswAlgorithms/effectorInterfaces/thrusterPlatformReference`).
//!
//! A thruster sits on a dual-gimballed platform (frame F) attached to the hub
//! through a mount frame M. The module computes the platform orientation
//! `[FM]` that points the thrust through the system center of mass, subject
//! to the platform constraint that `[FM]` has a zero (2,1) element, in three
//! successive rotations (Calaon et al., "Momentum Management of a Spacecraft
//! equipped with a Dual-Gimballed Electric Thruster"). When the optional
//! wheel messages are both connected at reset, the aim point is offset from
//! the center of mass by `d = -t × (K h_w + Ki ∫h_w dt) / t²` so the thrust
//! dumps wheel momentum. The tip and tilt angles are extracted from `[FM]`,
//! optionally bounded, and published with the resulting thrust heading,
//! thruster torque and body-frame thruster configuration.

use std::f64::consts::PI;

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    BodyHeadingMsg, BodyTorqueCommandMsg, HingedRigidBodyMsg, Input, MAX_EFF_COUNT, Output,
    RwArrayConfigMsg, RwSpeedMsg, ThrusterConfigMsg, VehicleConfigMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::normalize_or_zero;
use crate::utilities::rigid_body_kinematics::{euler123_to_dcm, mrp_to_dcm, prv_to_dcm};
use crate::{Module, SimulationContext};

/// Upstream `epsilon`: the module's tolerance for zero.
const EPSILON: f64 = 1e-12;

/// User-settable parameters of [`ThrusterPlatformReference`]. Defaults are
/// upstream's zero initialization.
#[derive(Clone, Debug)]
pub struct ThrusterPlatformReferenceConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `sigma_MB` [-]: MRP attitude of the mount frame M relative to
    /// the body frame B.
    pub sigma_mb: Vector3<f64>,
    /// Upstream `r_BM_M` \[m\]: position of the B origin relative to the M
    /// origin, M components.
    pub r_bm_m_m: Vector3<f64>,
    /// Upstream `r_FM_F` \[m\]: position of the F origin relative to the M
    /// origin, F components.
    pub r_fm_f_m: Vector3<f64>,
    /// Upstream `K` \[1/s\]: proportional gain of the momentum-dumping offset.
    pub k: f64,
    /// Upstream `Ki` [-]: integral gain of the momentum-dumping offset; used
    /// only when positive.
    pub ki: f64,
    /// Upstream `theta1Max` \[rad\]: bound on the tip angle; values up to
    /// 1e-12 (including negative ones) disable it.
    pub theta1_max_rad: f64,
    /// Upstream `theta2Max` \[rad\]: bound on the tilt angle; values up to
    /// 1e-12 (including negative ones) disable it.
    pub theta2_max_rad: f64,
}

impl Default for ThrusterPlatformReferenceConfig {
    fn default() -> Self {
        Self {
            name: "thrusterPlatformReference".to_string(),
            sigma_mb: Vector3::zeros(),
            r_bm_m_m: Vector3::zeros(),
            r_fm_f_m: Vector3::zeros(),
            k: 0.0,
            ki: 0.0,
            theta1_max_rad: 0.0,
            theta2_max_rad: 0.0,
        }
    }
}

/// Thruster-platform reference generator (upstream
/// `ThrusterPlatformReferenceConfig`).
#[derive(Clone, Debug)]
pub struct ThrusterPlatformReference {
    /// Module configuration, read at every update.
    pub config: ThrusterPlatformReferenceConfig,
    /// Upstream `vehConfigInMsg`: center of mass.
    pub veh_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `thrusterConfigFInMsg`: thruster in platform (F) coordinates;
    /// its `position_thrust_b_m` is `r_TF_F`.
    pub thruster_config_f_in_msg: Input<ThrusterConfigMsg>,
    /// Upstream `rwSpeedsInMsg` (optional).
    pub rw_speeds_in_msg: Input<RwSpeedMsg>,
    /// Upstream `rwConfigDataInMsg` (optional, read at reset).
    pub rw_config_data_in_msg: Input<RwArrayConfigMsg>,
    /// Upstream `hingedRigidBodyRef1OutMsg`: tip-angle reference.
    pub hinged_rigid_body_ref1_out_msg: Output<HingedRigidBodyMsg>,
    /// Upstream `hingedRigidBodyRef2OutMsg`: tilt-angle reference.
    pub hinged_rigid_body_ref2_out_msg: Output<HingedRigidBodyMsg>,
    /// Upstream `bodyHeadingOutMsg`: thrust direction, body frame.
    pub body_heading_out_msg: Output<BodyHeadingMsg>,
    /// Upstream `thrusterTorqueOutMsg`: opposite of the thruster torque about
    /// the center of mass, for the wheels to compensate.
    pub thruster_torque_out_msg: Output<BodyTorqueCommandMsg>,
    /// Upstream `thrusterConfigBOutMsg`: thruster in body (B) coordinates.
    pub thruster_config_b_out_msg: Output<ThrusterConfigMsg>,
    /// Upstream `rwConfigParams`, cached at reset; `Some` exactly when both
    /// wheel messages were connected (upstream `momentumDumping == Yes`).
    rw_config_params: Option<RwArrayConfigMsg>,
    /// Upstream `hsInt_M` \[N m s^2\]: time integral of the wheel momentum.
    hs_int_m: Vector3<f64>,
    /// Upstream `priorHs_M` \[N m s\]: wheel momentum at the previous update.
    prior_hs_m: Vector3<f64>,
    /// Upstream `priorTime` \[ns\].
    prior_time_nanos: u64,
}

/// Everything one update publishes.
struct PlatformReference {
    theta1_rad: f64,
    theta2_rad: f64,
    body_heading: BodyHeadingMsg,
    thruster_torque: BodyTorqueCommandMsg,
    thruster_config_b: ThrusterConfigMsg,
}

impl ThrusterPlatformReference {
    /// Builds the module with unconnected inputs and no momentum dumping
    /// until reset finds both wheel messages connected.
    pub fn new(config: ThrusterPlatformReferenceConfig) -> Self {
        Self {
            config,
            veh_config_in_msg: Input::default(),
            thruster_config_f_in_msg: Input::default(),
            rw_speeds_in_msg: Input::default(),
            rw_config_data_in_msg: Input::default(),
            hinged_rigid_body_ref1_out_msg: Output::default(),
            hinged_rigid_body_ref2_out_msg: Output::default(),
            body_heading_out_msg: Output::default(),
            thruster_torque_out_msg: Output::default(),
            thruster_config_b_out_msg: Output::default(),
            rw_config_params: None,
            hs_int_m: Vector3::zeros(),
            prior_hs_m: Vector3::zeros(),
            prior_time_nanos: 0,
        }
    }

    /// Upstream `Reset_thrusterPlatformReference`: decide on momentum dumping,
    /// clear the momentum integral and zero every output.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.veh_config_in_msg.is_connected() {
            log::error!("thrusterPlatformReference.veh_config_in_msg is not connected");
        }
        if !self.thruster_config_f_in_msg.is_connected() {
            log::error!("thrusterPlatformReference.thruster_config_f_in_msg is not connected");
        }
        self.rw_config_params = (self.rw_config_data_in_msg.is_connected()
            && self.rw_speeds_in_msg.is_connected())
        .then(|| self.rw_config_data_in_msg.read());
        if self
            .rw_config_params
            .as_ref()
            .is_some_and(|rw_config| rw_config.num_rw > MAX_EFF_COUNT)
        {
            log::error!(
                "thrusterPlatformReference.num_rw exceeds MAX_EFF_COUNT ({MAX_EFF_COUNT}); truncating"
            );
        }

        self.hs_int_m = Vector3::zeros();
        self.prior_hs_m = Vector3::zeros();
        self.prior_time_nanos = call_time_nanos;

        self.write_zero_outputs();
    }

    fn write_zero_outputs(&self) {
        self.hinged_rigid_body_ref1_out_msg
            .write(HingedRigidBodyMsg::default());
        self.hinged_rigid_body_ref2_out_msg
            .write(HingedRigidBodyMsg::default());
        self.body_heading_out_msg.write(BodyHeadingMsg::default());
        self.thruster_torque_out_msg
            .write(BodyTorqueCommandMsg::default());
        self.thruster_config_b_out_msg
            .write(ThrusterConfigMsg::default());
    }

    fn compute_reference(&mut self, call_time_nanos: u64) -> PlatformReference {
        let vehicle_config = self.veh_config_in_msg.read();
        let thruster_f = self.thruster_config_f_in_msg.read();

        let dcm_mb = mrp_to_dcm(self.config.sigma_mb);
        let r_cb_b = vehicle_config.center_of_mass_body_m;
        let r_cm_m = dcm_mb * r_cb_b + self.config.r_bm_m_m;
        let r_tm_f = self.config.r_fm_f_m + thruster_f.position_thrust_b_m;
        let t_f = thruster_f.max_thrust_n * thruster_f.t_hat_thrust_b;

        let mut dcm_fm = platform_rotation(r_cm_m, r_tm_f, t_f);
        if let Some(rw_config) = &self.rw_config_params {
            // Aim off the center of mass so the thrust torque dumps momentum.
            let hs_m = dcm_mb * wheel_momentum_b(rw_config, &self.rw_speeds_in_msg.read());
            let offset_m = self.momentum_dumping_offset(call_time_nanos, hs_m, &dcm_fm, t_f);
            dcm_fm = platform_rotation(r_cm_m + offset_m, r_tm_f, t_f);
        }

        let theta1_rad = bound_angle(
            dcm_fm[(1, 2)].atan2(dcm_fm[(1, 1)]),
            self.config.theta1_max_rad,
        );
        let theta2_rad = bound_angle(
            dcm_fm[(2, 0)].atan2(dcm_fm[(0, 0)]),
            self.config.theta2_max_rad,
        );

        // The constraint-compliant platform attitude the angles describe.
        let dcm_fm = euler123_to_dcm(Vector3::new(theta1_rad, theta2_rad, 0.0));
        let dcm_fb = dcm_fm * dcm_mb;

        let thrust_direction_b = normalize_or_zero(dcm_fb.tr_mul(&t_f));
        let r_cm_f = dcm_fm * r_cm_m;
        let r_tc_f = r_tm_f - r_cm_f;
        // Opposite of the thruster torque, for the wheels to compensate.
        let torque_f = t_f.cross(&r_tc_f);

        PlatformReference {
            theta1_rad,
            theta2_rad,
            body_heading: BodyHeadingMsg {
                r_hat_xb_b: thrust_direction_b,
            },
            thruster_torque: BodyTorqueCommandMsg {
                torque_request_body_nm: dcm_fb.tr_mul(&torque_f),
            },
            thruster_config_b: ThrusterConfigMsg {
                position_thrust_b_m: r_cb_b + dcm_fb.tr_mul(&r_tc_f),
                t_hat_thrust_b: thrust_direction_b,
                max_thrust_n: t_f.norm(),
                swirl_torque_nm: 0.0,
            },
        }
    }

    /// Integrate the wheel momentum `hs_m` \[N m s\] (M components) with the
    /// trapezoidal rule and return the aim-point offset `d_M` \[m\] from the
    /// center of mass, M components.
    fn momentum_dumping_offset(
        &mut self,
        call_time_nanos: u64,
        hs_m: Vector3<f64>,
        dcm_fm: &Matrix3<f64>,
        t_f: Vector3<f64>,
    ) -> Vector3<f64> {
        let dt_s = diff_nanos_to_seconds(call_time_nanos, self.prior_time_nanos);
        self.hs_int_m += (0.5 * dt_s) * (self.prior_hs_m + hs_m);
        self.prior_hs_m = hs_m;
        self.prior_time_nanos = call_time_nanos;

        let t_m = dcm_fm.tr_mul(&t_f);
        let mut momentum_feedback = self.config.k * hs_m;
        if self.config.ki > 0.0 {
            momentum_feedback += self.config.ki * self.hs_int_m;
        }
        (-1.0 / t_m.dot(&t_m)) * t_m.cross(&momentum_feedback)
    }
}

/// Net wheel angular momentum \[N m s\], body frame.
fn wheel_momentum_b(rw_config: &RwArrayConfigMsg, wheel_speeds: &RwSpeedMsg) -> Vector3<f64> {
    let num_rw = rw_config.num_rw.min(MAX_EFF_COUNT);
    (0..num_rw).fold(Vector3::zeros(), |sum, i| {
        sum + (rw_config.spin_axis_inertias_kg_m2[i] * wheel_speeds.wheel_speeds_radps[i])
            * rw_config.spin_axes_body[i]
    })
}

/// Clamp a reference angle to `±max_rad` when the bound is enabled
/// (`max_rad > EPSILON`).
fn bound_angle(theta_rad: f64, max_rad: f64) -> f64 {
    if max_rad > EPSILON && theta_rad > max_rad {
        max_rad
    } else if max_rad > EPSILON && theta_rad < -max_rad {
        -max_rad
    } else {
        theta_rad
    }
}

/// Upstream's `fmin(fmax(x, -1), 1)`. Unlike `f64::clamp`, a NaN argument
/// gives -1, as C's `fmax` ignores NaN.
#[allow(
    clippy::manual_clamp,
    reason = "clamp keeps NaN; upstream's fmax/fmin map it to -1"
)]
fn clamp_unit(x: f64) -> f64 {
    x.max(-1.0).min(1.0)
}

/// Upstream `tprComputeFinalRotation`: the platform attitude `[FM]` that
/// points the thrust `t_f` \[N\], applied at `r_tm_f` \[m\], through the point
/// `r_cm_m` \[m\] and satisfies the platform constraint as closely as possible.
fn platform_rotation(
    r_cm_m: Vector3<f64>,
    r_tm_f: Vector3<f64>,
    t_f: Vector3<f64>,
) -> Matrix3<f64> {
    let r_hat_cm_m = normalize_or_zero(r_cm_m);
    let t_hat_f = normalize_or_zero(t_f);
    // The first rotation assumes F starts aligned with M.
    let r_hat_cm_f = r_hat_cm_m;

    // Make the thrust parallel to the center-of-mass direction.
    let dcm_f1m = first_rotation(t_hat_f, r_hat_cm_f);
    let r_cm_f = dcm_f1m * r_cm_m;
    let r_ct_f = r_cm_f - r_tm_f;

    // Remove the offset between the thrust line and the center of mass.
    let dcm_f2f1 = second_rotation(r_cm_f, r_tm_f, r_ct_f, t_hat_f);
    let dcm_f2m = dcm_f2f1 * dcm_f1m;

    // Rotate about the center-of-mass direction to meet the constraint.
    let e_theta = normalize_or_zero(dcm_f2m * r_cm_m);
    third_rotation(e_theta, &dcm_f2m) * dcm_f2m
}

/// Upstream `tprComputeFirstRotation`: the rotation `[F1M]` taking `t_hat_f`
/// onto `r_hat_cm_f`.
fn first_rotation(t_hat_f: Vector3<f64>, r_hat_cm_f: Vector3<f64>) -> Matrix3<f64> {
    let mut phi = clamp_unit(t_hat_f.dot(&r_hat_cm_f)).acos();
    let mut e_phi = t_hat_f.cross(&r_hat_cm_f);
    if (phi - PI).abs() < EPSILON {
        // Antiparallel: any axis perpendicular to the thrust works.
        phi = PI;
        let [t1, t2, t3] = [t_hat_f[0], t_hat_f[1], t_hat_f[2]];
        e_phi = if t1.abs() > EPSILON {
            Vector3::new(-(t2 + t3) / t1, 1.0, 1.0)
        } else if t2.abs() > EPSILON {
            Vector3::new(1.0, -(t1 + t3) / t2, 1.0)
        } else {
            Vector3::new(1.0, 1.0, -(t1 + t2) / t3)
        };
    } else if phi.abs() < EPSILON {
        phi = 0.0;
    }
    prv_to_dcm(phi * normalize_or_zero(e_phi))
}

/// Upstream `tprComputeSecondRotation`: the rotation `[F2F1]` about
/// `t_hat_f × r_ct_f` that brings the thrust line through the center of mass.
fn second_rotation(
    r_cm_f: Vector3<f64>,
    r_tm_f: Vector3<f64>,
    r_ct_f: Vector3<f64>,
    t_hat_f: Vector3<f64>,
) -> Matrix3<f64> {
    let a_vec = r_tm_f;
    let a = a_vec.norm();
    let b = r_cm_f.norm();
    let c1 = r_ct_f.norm();

    let psi = if a.abs() < EPSILON {
        // No offset between M and the thrust point: nothing to correct.
        0.0
    } else {
        let beta = (-clamp_unit(a_vec.dot(&t_hat_f) / a)).acos();
        let nu = (-clamp_unit(a_vec.dot(&r_ct_f) / (a * c1))).acos();
        let c2 = a * beta.cos() + (b * b - a * a * beta.sin() * beta.sin()).sqrt();
        let cos_gamma1 = (a * a + b * b - c1 * c1) / (2.0 * a * b);
        let cos_gamma2 = (a * a + b * b - c2 * c2) / (2.0 * a * b);
        clamp_unit((c1 * nu.sin() * cos_gamma2 - c2 * beta.sin() * cos_gamma1) / b).asin()
    };

    let e_psi = normalize_or_zero(t_hat_f.cross(&r_ct_f));
    prv_to_dcm(psi * e_psi)
}

/// Upstream `tprComputeThirdRotation`: the rotation `[F3F2]` about `e_theta`
/// that zeroes element (2,1) of `[F3F2][F2M]`, or minimizes it when no exact
/// solution exists. Solves `A t² + B t + C = 0` for `t = tan(theta/2)`.
fn third_rotation(e_theta: Vector3<f64>, dcm_f2m: &Matrix3<f64>) -> Matrix3<f64> {
    let [e1, e2, e3] = [e_theta[0], e_theta[1], e_theta[2]];
    let a = 2.0
        * (dcm_f2m[(1, 0)] * e2 * e2 + dcm_f2m[(0, 0)] * e1 * e2 + dcm_f2m[(2, 0)] * e2 * e3)
        - dcm_f2m[(1, 0)];
    let b = 2.0 * (dcm_f2m[(2, 0)] * e1 - dcm_f2m[(0, 0)] * e3);
    let c = dcm_f2m[(1, 0)];
    let delta = b * b - 4.0 * a * c;
    let residual = |t: f64| (a * t * t + b * t + c) / (1.0 + t * t);

    let theta = if a.abs() < EPSILON {
        if b.abs() < EPSILON {
            // The zeroth-order equation has no root; theta = pi minimizes it.
            PI
        } else {
            2.0 * (-c / b).atan()
        }
    } else if delta < 0.0 {
        // No real root: take the stationary point with the smaller residual.
        let t = if b.abs() < EPSILON {
            0.0
        } else {
            let q = (a - c) / b;
            let t1 = q + (q * q + 1.0).sqrt();
            let t2 = q - (q * q + 1.0).sqrt();
            if residual(t2).abs() < residual(t1).abs() {
                t2
            } else {
                t1
            }
        };
        // The residual at theta = pi is A.
        if a.abs() < residual(t).abs() {
            PI
        } else {
            2.0 * t.atan()
        }
    } else {
        let t1 = (-b + delta.sqrt()) / (2.0 * a);
        let t2 = (-b - delta.sqrt()) / (2.0 * a);
        let t = if t2.abs() < t1.abs() { t2 } else { t1 };
        2.0 * t.atan()
    };

    prv_to_dcm(theta * e_theta)
}

impl Module for ThrusterPlatformReference {
    fn init(&mut self) {
        self.write_zero_outputs();
        self.reset(0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        ThrusterPlatformReference::reset(self, context.current_sim_nanos);
    }

    fn update(&mut self, context: &SimulationContext) {
        let reference = self.compute_reference(context.current_sim_nanos);
        self.hinged_rigid_body_ref1_out_msg
            .write(HingedRigidBodyMsg {
                theta_rad: reference.theta1_rad,
                theta_dot_radps: 0.0,
            });
        self.hinged_rigid_body_ref2_out_msg
            .write(HingedRigidBodyMsg {
                theta_rad: reference.theta2_rad,
                theta_dot_radps: 0.0,
            });
        self.body_heading_out_msg.write(reference.body_heading);
        self.thruster_torque_out_msg
            .write(reference.thruster_torque);
        self.thruster_config_b_out_msg
            .write(reference.thruster_config_b);
    }
}

#[cfg(test)]
mod tests {
    //! Port of `test_thrusterPlatformReference.py` (all 240 parametrizations),
    //! plus tests of Reset, the optional momentum-dumping inputs and the
    //! integral term (which upstream does not test).
    //!
    //! Upstream seeds Python's `random` but draws the center-of-mass direction
    //! from the unseeded `np.random.rand(3)`, so its draws differ on every
    //! run. Here each `seed` selects a deterministic uniform draw in
    //! `[0, 1)^3` instead.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{
        Output, RwArrayConfigMsg, RwSpeedMsg, ThrusterConfigMsg, VehicleConfigMsg,
    };
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{euler123_to_dcm, euler123_to_mrp, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{ThrusterPlatformReference, ThrusterPlatformReferenceConfig};

    const R_BM_M: Vector3<f64> = Vector3::new(0.0, 0.1, 1.4);
    const R_FM_F: Vector3<f64> = Vector3::new(0.0, 0.0, -0.1);
    const R_TF_F: Vector3<f64> = Vector3::new(-0.01, 0.03, 0.02);
    const T_F: Vector3<f64> = Vector3::new(1.0, 1.0, 10.0);

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// A uniform draw in `[0, 1)^3` (splitmix64), standing in for upstream's
    /// `np.random.rand(3)`.
    fn uniform_draw(seed: u64) -> Vector3<f64> {
        let mut state = seed;
        let mut next = || {
            state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
            let mut z = state;
            z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
            ((z ^ (z >> 31)) >> 11) as f64 / 2.0_f64.powi(53)
        };
        Vector3::new(next(), next(), next())
    }

    fn sigma_mb() -> Vector3<f64> {
        euler123_to_mrp(Vector3::new(5.0 * D2R, 10.0 * D2R, 0.0))
    }

    fn thruster_f() -> ThrusterConfigMsg {
        let max_thrust_n = T_F.norm();
        ThrusterConfigMsg {
            position_thrust_b_m: R_TF_F,
            t_hat_thrust_b: T_F / max_thrust_n,
            max_thrust_n,
            swirl_torque_nm: 0.0,
        }
    }

    fn rw_config() -> RwArrayConfigMsg {
        let mut config = RwArrayConfigMsg {
            num_rw: 3,
            ..Default::default()
        };
        config.spin_axes_body[..3].copy_from_slice(&[Vector3::x(), Vector3::y(), Vector3::z()]);
        config.spin_axis_inertias_kg_m2[..3].copy_from_slice(&[0.01; 3]);
        config.max_motor_torques_nm[..3].copy_from_slice(&[0.001; 3]);
        config
    }

    fn rw_speeds() -> RwSpeedMsg {
        let mut speeds = RwSpeedMsg::default();
        speeds.wheel_speeds_radps[..3].copy_from_slice(&[100.0; 3]);
        speeds
    }

    /// The upstream test's module and inputs, with the wheel messages
    /// connected as selected.
    fn upstream_module(
        config: ThrusterPlatformReferenceConfig,
        r_cb_b: Vector3<f64>,
        connect_rw_config: bool,
        connect_rw_speeds: bool,
    ) -> ThrusterPlatformReference {
        let mut module = ThrusterPlatformReference::new(config);
        Output::new(VehicleConfigMsg {
            center_of_mass_body_m: r_cb_b,
            ..Default::default()
        })
        .connect_to(&mut module.veh_config_in_msg);
        Output::new(thruster_f()).connect_to(&mut module.thruster_config_f_in_msg);
        if connect_rw_config {
            Output::new(rw_config()).connect_to(&mut module.rw_config_data_in_msg);
        }
        if connect_rw_speeds {
            Output::new(rw_speeds()).connect_to(&mut module.rw_speeds_in_msg);
        }
        module.init();
        module
    }

    fn upstream_config(k: f64, theta_max_rad: f64) -> ThrusterPlatformReferenceConfig {
        ThrusterPlatformReferenceConfig {
            sigma_mb: sigma_mb(),
            r_bm_m_m: R_BM_M,
            r_fm_f_m: R_FM_F,
            k,
            ki: 0.0,
            theta1_max_rad: theta_max_rad,
            theta2_max_rad: theta_max_rad,
            ..Default::default()
        }
    }

    fn assert_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64, what: &str) {
        assert!(
            (actual - expected).amax() <= accuracy,
            "{what}: expected {expected:?}, got {actual:?}"
        );
    }

    #[test]
    fn matches_upstream_platform_rotation_unit_test() {
        let accuracy = 1e-10;
        for seed in 1..=10 {
            for delta_cm in [0.1, 0.2, 0.3] {
                for k in [0.0, 1.0, 5.0, 10.0] {
                    for theta_max in [-1.0, PI / 36.0] {
                        let case = format!(
                            "seed={seed}, delta_CM={delta_cm}, K={k}, thetaMax={theta_max}"
                        );
                        let draw = uniform_draw(seed);
                        let r_cb_b = draw / draw.norm() * delta_cm;
                        let mut module =
                            upstream_module(upstream_config(k, theta_max), r_cb_b, true, true);
                        // Upstream runs to 0.5 s at a 1 s rate: one update at t = 0.
                        module.update(&context(0));

                        let theta1 = module.hinged_rigid_body_ref1_out_msg.read().theta_rad;
                        let theta2 = module.hinged_rigid_body_ref2_out_msg.read().theta_rad;

                        let dcm_fm = euler123_to_dcm(Vector3::new(theta1, theta2, 0.0));
                        let dcm_mb = mrp_to_dcm(sigma_mb());
                        let r_cm_m = dcm_mb * r_cb_b + R_BM_M;
                        let r_cm_f = dcm_fm * r_cm_m;
                        let r_ct_f = r_cm_f - R_FM_F - R_TF_F;

                        if k == 0.0 && theta_max < 0.0 {
                            let offset = (r_ct_f.cross(&T_F) / r_ct_f.norm() / T_F.norm()).norm();
                            assert!(offset.abs() <= accuracy, "{case}: offset {offset}");
                        }

                        let dcm_fb: Matrix3<f64> = dcm_fm * dcm_mb;
                        let t_b = dcm_fb.transpose() * T_F;
                        let t_b_hat = t_b / t_b.norm();
                        assert_close(
                            module.body_heading_out_msg.read().r_hat_xb_b,
                            t_b_hat,
                            accuracy,
                            &format!("{case}: rHat_XB_B"),
                        );

                        let l_b = dcm_fb.transpose() * r_ct_f.cross(&T_F);
                        assert_close(
                            module.thruster_torque_out_msg.read().torque_request_body_nm,
                            l_b,
                            accuracy,
                            &format!("{case}: torqueRequestBody"),
                        );

                        let thruster_b = module.thruster_config_b_out_msg.read();
                        let r_tb_b = r_cb_b - dcm_fb.transpose() * r_ct_f;
                        assert_close(
                            thruster_b.position_thrust_b_m,
                            r_tb_b,
                            accuracy,
                            &format!("{case}: rThrust_B"),
                        );
                        assert_close(
                            thruster_b.t_hat_thrust_b,
                            t_b_hat,
                            accuracy,
                            &format!("{case}: tHatThrust_B"),
                        );
                        assert!(
                            (thruster_b.max_thrust_n - t_b.norm()).abs() <= accuracy,
                            "{case}: maxThrust"
                        );

                        if theta_max > 0.0 {
                            assert!(theta1 < theta_max + accuracy, "{case}: theta1 {theta1}");
                            assert!(theta2 < theta_max + accuracy, "{case}: theta2 {theta2}");
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn reset_zeros_every_output() {
        let mut module = upstream_module(
            upstream_config(1.0, -1.0),
            Vector3::new(0.05, 0.05, 0.05),
            true,
            true,
        );
        module.update(&context(0));
        assert_ne!(
            module.thruster_config_b_out_msg.read(),
            ThrusterConfigMsg::default()
        );

        module.reset(1_000_000_000);
        for theta in [
            module.hinged_rigid_body_ref1_out_msg.read(),
            module.hinged_rigid_body_ref2_out_msg.read(),
        ] {
            assert_eq!(theta.theta_rad, 0.0);
            assert_eq!(theta.theta_dot_radps, 0.0);
        }
        assert_eq!(
            module.body_heading_out_msg.read().r_hat_xb_b,
            Vector3::zeros()
        );
        assert_eq!(
            module.thruster_torque_out_msg.read().torque_request_body_nm,
            Vector3::zeros()
        );
        assert_eq!(
            module.thruster_config_b_out_msg.read(),
            ThrusterConfigMsg::default()
        );
    }

    #[test]
    fn dumps_momentum_only_when_both_wheel_messages_are_connected() {
        let r_cb_b = Vector3::new(0.05, 0.1, 0.02);
        let theta1 = |module: &mut ThrusterPlatformReference| {
            module.update(&context(0));
            module.hinged_rigid_body_ref1_out_msg.read().theta_rad
        };
        let aligned = theta1(&mut upstream_module(
            upstream_config(0.0, -1.0),
            r_cb_b,
            true,
            true,
        ));
        for (config, speeds) in [(true, false), (false, true), (false, false)] {
            let mut module = upstream_module(upstream_config(5.0, -1.0), r_cb_b, config, speeds);
            assert_eq!(theta1(&mut module), aligned);
        }
        let mut dumping = upstream_module(upstream_config(5.0, -1.0), r_cb_b, true, true);
        assert!((theta1(&mut dumping) - aligned).abs() > 1e-6);
    }

    #[test]
    fn integral_term_uses_the_trapezoidal_momentum_integral() {
        let r_cb_b = Vector3::new(0.05, 0.1, 0.02);
        let mut integral = upstream_module(
            ThrusterPlatformReferenceConfig {
                ki: 0.5,
                ..upstream_config(1.0, -1.0)
            },
            r_cb_b,
            true,
            true,
        );
        // Constant wheel momentum h: the integral is 0, h and 2h at 0, 1, 2 s.
        let hs_m = mrp_to_dcm(sigma_mb()) * Vector3::new(1.0, 1.0, 1.0);
        for (time_s, expected) in [(0, 0.0), (1, 1.0), (2, 2.0)] {
            integral.update(&context(time_s * 1_000_000_000));
            assert_close(integral.hs_int_m, expected * hs_m, 1e-15, "hsInt_M");
        }

        // At 2 s, K h + Ki (2 h) = 2 h: the same reference as K = 2 without
        // the integral term.
        let mut proportional = upstream_module(upstream_config(2.0, -1.0), r_cb_b, true, true);
        proportional.update(&context(2_000_000_000));
        assert_eq!(
            integral.hinged_rigid_body_ref1_out_msg.read().theta_rad,
            proportional.hinged_rigid_body_ref1_out_msg.read().theta_rad
        );
        assert_eq!(
            integral.hinged_rigid_body_ref2_out_msg.read().theta_rad,
            proportional.hinged_rigid_body_ref2_out_msg.read().theta_rad
        );
    }

    #[test]
    fn bounds_both_reference_angles_symmetrically() {
        let r_cb_b = Vector3::new(-0.3, 0.3, 0.0);
        let mut free = upstream_module(upstream_config(0.0, -1.0), r_cb_b, false, false);
        free.update(&context(0));
        let theta1 = free.hinged_rigid_body_ref1_out_msg.read().theta_rad;
        let theta2 = free.hinged_rigid_body_ref2_out_msg.read().theta_rad;

        let bound = 0.5 * theta1.abs().min(theta2.abs());
        assert!(bound > 1e-3, "both free angles must be well away from zero");
        let mut bounded = upstream_module(upstream_config(0.0, bound), r_cb_b, false, false);
        bounded.update(&context(0));
        assert_eq!(
            bounded.hinged_rigid_body_ref1_out_msg.read().theta_rad,
            bound.copysign(theta1)
        );
        assert_eq!(
            bounded.hinged_rigid_body_ref2_out_msg.read().theta_rad,
            bound.copysign(theta2)
        );
    }
}
