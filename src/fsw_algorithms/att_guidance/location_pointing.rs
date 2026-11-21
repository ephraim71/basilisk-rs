//! Target-pointing attitude guidance and reference: port of upstream Basilisk
//! `locationPointing`.
//!
//! The module rotates the body-fixed axis `p_hat_b` onto the heading from the
//! spacecraft to a target, which is a ground location, a celestial body, a
//! second spacecraft or the current point of an imaging strip, whichever input
//! is connected (in that priority order: ground location, strip, celestial
//! body, target spacecraft). The rotation is the eigen-axis rotation
//! `σ_B/R = -tan(φ/4) ê` with `ê ∝ p̂_B × r̂_L/S`, so only two attitude degrees
//! of freedom are constrained.
//!
//! In strip-imaging mode an extra rotation about `p_hat_b` then turns the
//! body-fixed axis `c_hat_b` perpendicular to the strip's scanning direction,
//! unless the target speed is below `strip_inertial_speed_threshold_mps` or
//! the scanning direction is nearly along `p_hat_b`.
//!
//! The tracking-error rate `ω_B/R` is the finite difference of `σ_B/R` between
//! updates (zero at the first update after a reset). With
//! `use_boresight_rate_damping` the body rate about the target heading is
//! added to it, outside strip imaging. The module publishes both the guidance
//! message (for a controller) and the reference message (for
//! `attTrackingError`).

use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::messages::{
    AttitudeGuidanceMsg, AttitudeReferenceMsg, EphemerisMsg, GroundStateMsg, Input,
    NavigationAttitudeMsg, NavigationTranslationMsg, Output, StripStateMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::{normalize_or_zero, safe_acos};
use crate::utilities::rigid_body_kinematics::{add_mrp, b_inv_mrp, mrp_to_dcm, sub_mrp};
use crate::{Module, SimulationContext};

/// Upstream's default `alignmentThreshold`, applied at reset when it is not
/// positive.
const DEFAULT_ALIGNMENT_THRESHOLD: f64 = 0.1;
/// Upstream's default `stripInertialSpeedThreshold` \[m/s\], applied at reset
/// when it is not positive.
const DEFAULT_STRIP_INERTIAL_SPEED_THRESHOLD_MPS: f64 = 1e-12;

/// User-settable parameters of [`LocationPointing`]. `Default` is upstream's
/// zero-initialized configuration.
///
/// As upstream, reset writes back into this configuration: it normalizes
/// `p_hat_b` (and `c_hat_b` when strip imaging is connected) and replaces
/// non-positive thresholds with their defaults. Updates read the fields
/// directly.
#[derive(Clone, Debug)]
pub struct LocationPointingConfig {
    /// Instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `pHat_B`: body-fixed axis to aim at the target. Must be set;
    /// reset normalizes it.
    pub p_hat_b: Vector3<f64>,
    /// Upstream `cHat_B`: body-fixed scan-line axis turned perpendicular to
    /// the strip direction, orthogonal to `p_hat_b`. Used, and normalized at
    /// reset, only in strip imaging.
    pub c_hat_b: Vector3<f64>,
    /// Upstream `smallAngle` \[rad\]: rotations smaller than this (pointing or
    /// strip) are set to zero, and pointing errors within this of 180 deg use
    /// a fixed eigen axis.
    pub small_angle_rad: f64,
    /// Upstream `useBoresightRateDamping`: add the body rate about the target
    /// heading to the rate error (ignored in strip imaging).
    pub use_boresight_rate_damping: bool,
    /// Upstream `alignmentThreshold` \[-\]: minimum `|p̂_B × v̂|` of the strip
    /// scanning direction for the extra strip rotation. Reset replaces a value
    /// `<= 0` with 0.1.
    pub alignment_threshold: f64,
    /// Upstream `stripInertialSpeedThreshold` \[m/s\]: minimum strip target
    /// speed relative to the planet for the extra strip rotation. Reset
    /// replaces a value `<= 0` with 1e-12.
    pub strip_inertial_speed_threshold_mps: f64,
}

impl Default for LocationPointingConfig {
    fn default() -> Self {
        Self {
            name: "locationPointing".to_string(),
            p_hat_b: Vector3::zeros(),
            c_hat_b: Vector3::zeros(),
            small_angle_rad: 0.0,
            use_boresight_rate_damping: false,
            alignment_threshold: 0.0,
            strip_inertial_speed_threshold_mps: 0.0,
        }
    }
}

/// Target-pointing guidance: upstream `locationPointing`.
#[derive(Clone, Debug)]
pub struct LocationPointing {
    /// User-settable parameters.
    pub config: LocationPointingConfig,
    /// Upstream `scAttInMsg`: spacecraft inertial attitude and rate.
    pub sc_att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `scTransInMsg`: spacecraft inertial position.
    pub sc_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `locationInMsg`: ground location target.
    pub location_in_msg: Input<GroundStateMsg>,
    /// Upstream `locationstripInMsg`: current strip target, selects strip
    /// imaging.
    pub locationstrip_in_msg: Input<StripStateMsg>,
    /// Upstream `celBodyInMsg`: celestial body target.
    pub cel_body_in_msg: Input<EphemerisMsg>,
    /// Upstream `scTargetInMsg`: target spacecraft position.
    pub sc_target_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `attGuidOutMsg`: attitude tracking errors.
    pub att_guid_out_msg: Output<AttitudeGuidanceMsg>,
    /// Upstream `attRefOutMsg`: attitude reference.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    /// Upstream `eHat180_B`: eigen axis used when the target is behind
    /// `p_hat_b`.
    e_hat_180_b: Vector3<f64>,
    /// Upstream `sigma_BR_old` and `time_old`; `None` until the first update
    /// after a reset (upstream's `init` counter).
    previous: Option<PreviousSample>,
}

/// Tracking error of the previous update, for the finite-difference rate.
#[derive(Clone, Copy, Debug)]
struct PreviousSample {
    sigma_br: Vector3<f64>,
    time_nanos: u64,
}

/// Inertial target of the current update.
#[derive(Clone, Copy, Debug)]
struct Target {
    /// Upstream `r_TN_N` \[m\].
    position_n_m: Vector3<f64>,
    /// Upstream `v_TP_N` \[m/s\] in strip imaging, `None` otherwise.
    strip_velocity_n_mps: Option<Vector3<f64>>,
}

/// `acos` of a cosine, as upstream: a cosine beyond ±1 is first rescaled to
/// ±1, then passed to `safeAcos`.
fn rescaled_acos(mut cosine: f64) -> f64 {
    if cosine.abs() > 1.0 {
        cosine /= cosine.abs();
    }
    safe_acos(cosine)
}

impl LocationPointing {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: LocationPointingConfig) -> Self {
        Self {
            config,
            sc_att_in_msg: Input::default(),
            sc_trans_in_msg: Input::default(),
            location_in_msg: Input::default(),
            locationstrip_in_msg: Input::default(),
            cel_body_in_msg: Input::default(),
            sc_target_in_msg: Input::default(),
            att_guid_out_msg: Output::default(),
            att_ref_out_msg: Output::default(),
            e_hat_180_b: Vector3::zeros(),
            previous: None,
        }
    }

    /// Upstream `Reset_locationPointing`.
    fn reset_state(&mut self) {
        if !self.sc_att_in_msg.is_connected() {
            log::error!("locationPointing.sc_att_in_msg is not connected");
        }
        if !self.sc_trans_in_msg.is_connected() {
            log::error!("locationPointing.sc_trans_in_msg is not connected");
        }
        let num_targets = [
            self.location_in_msg.is_connected(),
            self.locationstrip_in_msg.is_connected(),
            self.cel_body_in_msg.is_connected(),
            self.sc_target_in_msg.is_connected(),
        ]
        .into_iter()
        .filter(|&connected| connected)
        .count();
        match num_targets {
            0 => log::error!("locationPointing: no target messages are connected"),
            1 => {}
            _ => log::error!(
                "locationPointing: multiple target messages are connected. Defaulting to either \
                 ground location, planet location or spacecraft location, in that order."
            ),
        }

        self.previous = None;
        if self.config.alignment_threshold <= 0.0 {
            self.config.alignment_threshold = DEFAULT_ALIGNMENT_THRESHOLD;
        }
        if self.config.strip_inertial_speed_threshold_mps <= 0.0 {
            self.config.strip_inertial_speed_threshold_mps =
                DEFAULT_STRIP_INERTIAL_SPEED_THRESHOLD_MPS;
        }

        let p_hat_b = self.config.p_hat_b;
        if p_hat_b.norm() < 0.1 {
            // As upstream, the eigen axis then keeps its previous value.
            log::error!(
                "locationPointing: vector p_hat_b is not setup as a unit vector [{:.6}, {:.6}, {:.6}]",
                p_hat_b.x,
                p_hat_b.y,
                p_hat_b.z
            );
        } else {
            let p_hat_b = normalize_or_zero(p_hat_b);
            self.config.p_hat_b = p_hat_b;
            let mut e_hat_180_b = p_hat_b.cross(&Vector3::x());
            if e_hat_180_b.norm() < 0.1 {
                e_hat_180_b = p_hat_b.cross(&Vector3::y());
            }
            self.e_hat_180_b = normalize_or_zero(e_hat_180_b);
        }

        if self.locationstrip_in_msg.is_connected() {
            self.check_scan_line_axis();
        }
    }

    /// Upstream's strip-imaging check of `cHat_B`, which also normalizes it.
    fn check_scan_line_axis(&mut self) {
        let c_hat_b = self.config.c_hat_b;
        if c_hat_b.norm() < 0.1 {
            log::error!(
                "locationPointing: vector c_hat_b is not setup as a unit vector [{:.6}, {:.6}, {:.6}]",
                c_hat_b.x,
                c_hat_b.y,
                c_hat_b.z
            );
            return;
        }
        let c_hat_b = normalize_or_zero(c_hat_b);
        self.config.c_hat_b = c_hat_b;
        let dot = self.config.p_hat_b.dot(&c_hat_b).abs();
        if dot > 1e-6 {
            log::error!(
                "locationPointing: c_hat_b is not perpendicular to p_hat_b (dot product = \
                 {dot:e}). These vectors must be orthogonal for strip imaging."
            );
        }
    }

    /// The target of the first connected target input, in upstream's
    /// priority order; an unconnected target spacecraft reads as zero.
    fn target(&self) -> Target {
        if self.location_in_msg.is_connected() {
            Target {
                position_n_m: self.location_in_msg.read().position_ln_n_m,
                strip_velocity_n_mps: None,
            }
        } else if self.locationstrip_in_msg.is_connected() {
            let strip = self.locationstrip_in_msg.read();
            Target {
                position_n_m: strip.position_ln_n_m,
                strip_velocity_n_mps: Some(strip.velocity_lp_n_mps),
            }
        } else if self.cel_body_in_msg.is_connected() {
            Target {
                position_n_m: self.cel_body_in_msg.read().position_bdy_zero_n_m,
                strip_velocity_n_mps: None,
            }
        } else {
            Target {
                position_n_m: self.sc_target_in_msg.read().position_bn_n_m,
                strip_velocity_n_mps: None,
            }
        }
    }

    /// Tracking error `σ_B/R` that rotates `p_hat_b` onto the target heading
    /// `r_hat_ls_b` (body components).
    fn pointing_error(&self, r_hat_ls_b: Vector3<f64>) -> Vector3<f64> {
        let phi = rescaled_acos(self.config.p_hat_b.dot(&r_hat_ls_b));
        if phi < self.config.small_angle_rad {
            return Vector3::zeros();
        }
        let e_hat_b = if PI - phi < self.config.small_angle_rad {
            self.e_hat_180_b
        } else {
            self.config.p_hat_b.cross(&r_hat_ls_b)
        };
        -(phi / 4.0).tan() * normalize_or_zero(e_hat_b)
    }

    /// Strip-imaging tracking error `σ_B/R2` and reference `σ_R2/N`, which add
    /// to the pointing reference `sigma_rn` the rotation about `p_hat_b` that
    /// turns `c_hat_b` perpendicular to the strip velocity `v_tp_n`. `None`
    /// when the strip direction is undefined and upstream falls back to pure
    /// location pointing.
    fn strip_reference(
        &self,
        sigma_bn: Vector3<f64>,
        sigma_rn: Vector3<f64>,
        v_tp_n: Vector3<f64>,
    ) -> Option<(Vector3<f64>, Vector3<f64>)> {
        if v_tp_n.norm() <= self.config.strip_inertial_speed_threshold_mps {
            return None;
        }
        // Scanning direction in the pointing reference frame.
        let v_tp_b = normalize_or_zero(mrp_to_dcm(sigma_rn) * normalize_or_zero(v_tp_n));
        let v_perp = self.config.p_hat_b.cross(&v_tp_b);
        // A scanning direction (nearly) along p_hat_b leaves v_perp undefined
        // or unstable.
        if v_perp.norm() < self.config.alignment_threshold {
            return None;
        }

        // Both signs are perpendicular to the strip: take the one closer to
        // c_hat_b, for the smaller rotation.
        let mut v_perp = normalize_or_zero(v_perp);
        let c_hat_b = self.config.c_hat_b;
        if c_hat_b.dot(&v_perp) < 0.0 {
            v_perp = -v_perp;
        }
        let angle = rescaled_acos(c_hat_b.dot(&v_perp));
        let sigma_r2r = if angle < self.config.small_angle_rad {
            Vector3::zeros()
        } else {
            (angle / 4.0).tan() * normalize_or_zero(c_hat_b.cross(&v_perp))
        };

        let sigma_r2n = add_mrp(sigma_rn, sigma_r2r);
        Some((sub_mrp(sigma_bn, sigma_r2n), sigma_r2n))
    }

    /// `ω_B/R` from the finite difference of `σ_B/R` since the previous
    /// update, `ω = 4 [B(σ)]⁻¹ σ̇`; zero at the first update after a reset.
    fn finite_difference_rate(&self, sigma_br: Vector3<f64>, time_nanos: u64) -> Vector3<f64> {
        self.previous.map_or_else(Vector3::zeros, |previous| {
            let dt_s = diff_nanos_to_seconds(time_nanos, previous.time_nanos);
            let sigma_dot_br = (1.0 / dt_s) * (sigma_br - previous.sigma_br);
            b_inv_mrp(sigma_br) * (4.0 * sigma_dot_br)
        })
    }
}

impl Module for LocationPointing {
    fn init(&mut self) {
        self.att_guid_out_msg.write(AttitudeGuidanceMsg::default());
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        self.reset_state();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_state();
    }

    fn update(&mut self, context: &SimulationContext) {
        let attitude = self.sc_att_in_msg.read();
        let translation = self.sc_trans_in_msg.read();
        let target = self.target();

        let r_ls_n = target.position_n_m - translation.position_bn_n_m;
        let dcm_bn = mrp_to_dcm(attitude.sigma_bn);
        let r_hat_ls_b = normalize_or_zero(dcm_bn * r_ls_n);

        let sigma_br = self.pointing_error(r_hat_ls_b);
        let sigma_rn = add_mrp(attitude.sigma_bn, -sigma_br);
        let (sigma_br, sigma_rn) = target
            .strip_velocity_n_mps
            .and_then(|v_tp_n| self.strip_reference(attitude.sigma_bn, sigma_rn, v_tp_n))
            .unwrap_or((sigma_br, sigma_rn));

        let mut omega_br_b = self.finite_difference_rate(sigma_br, context.current_sim_nanos);
        if self.config.use_boresight_rate_damping && target.strip_velocity_n_mps.is_none() {
            omega_br_b += attitude.omega_bn_b_radps.dot(&r_hat_ls_b) * r_hat_ls_b;
        }
        let omega_rn_b = attitude.omega_bn_b_radps - omega_br_b;

        self.previous = Some(PreviousSample {
            sigma_br,
            time_nanos: context.current_sim_nanos,
        });

        self.att_guid_out_msg.write(AttitudeGuidanceMsg {
            sigma_br,
            omega_br_b_radps: omega_br_b,
            omega_rn_b_radps: omega_rn_b,
            domega_rn_b_radps2: Vector3::zeros(),
        });
        self.att_ref_out_msg.write(AttitudeReferenceMsg {
            sigma_rn,
            omega_rn_n_radps: dcm_bn.tr_mul(&omega_rn_b),
            domega_rn_n_radps2: Vector3::zeros(),
        });
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{LocationPointing, LocationPointingConfig};
    use crate::messages::{
        AttitudeGuidanceMsg, AttitudeReferenceMsg, EphemerisMsg, GroundStateMsg,
        NavigationAttitudeMsg, NavigationTranslationMsg, Output, StripStateMsg,
    };
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::rigid_body_kinematics::{add_mrp, b_inv_mrp, mrp_to_dcm, sub_mrp};
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.1)`.
    const TIME_STEP_NANOS: u64 = 100_000_000;
    /// Upstream test time step \[s\].
    const TIME_STEP_S: f64 = 0.1;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `unitTestSupport.isArrayEqual`: absolute, per component.
    fn vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) -> bool {
        actual
            .iter()
            .zip(expected.iter())
            .all(|(a, e)| (a - e).abs() <= accuracy)
    }

    fn assert_vector_close(actual: Vector3<f64>, expected: Vector3<f64>, accuracy: f64) {
        assert!(
            vector_close(actual, expected, accuracy),
            "{actual:?} != {expected:?} within {accuracy}"
        );
    }

    /// Target input of upstream's `locationType` parametrization.
    #[derive(Clone, Copy, Debug)]
    enum LocationType {
        Ground,
        CelestialBody,
        TargetSpacecraft,
        Strip,
    }

    /// Output of upstream test helper `truthValues`, one entry per update.
    #[derive(Default)]
    struct Truth {
        sigma_br: Vec<Vector3<f64>>,
        omega_br_b: Vec<Vector3<f64>>,
        sigma_rn: Vec<Vector3<f64>>,
        omega_rn_b: Vec<Vector3<f64>>,
        omega_rn_n: Vec<Vector3<f64>>,
    }

    /// Upstream test helper `truthValues`. `v_lp_n` is `Some` in strip
    /// imaging (upstream `isStrip`).
    #[allow(clippy::too_many_arguments)]
    fn upstream_truth(
        p_hat_b: Vector3<f64>,
        c_hat_b: Vector3<f64>,
        r_ln_n: Vector3<f64>,
        r_sn_n: Vector3<f64>,
        sigma_bn_list: &[Vector3<f64>],
        omega_bn_list: &[Vector3<f64>],
        small_angle: f64,
        dt: f64,
        use_3d_rate: bool,
        v_lp_n: Option<Vector3<f64>>,
    ) -> Truth {
        let mut e_hat_180_b = p_hat_b.cross(&Vector3::new(1.0, 0.0, 0.0));
        if e_hat_180_b.norm() < 0.1 {
            e_hat_180_b = p_hat_b.cross(&Vector3::new(0.0, 1.0, 0.0));
        }
        e_hat_180_b /= e_hat_180_b.norm();

        let r_ls_n = r_ln_n - r_sn_n;
        let mut omega_br_b = Vector3::zeros();
        let mut truth = Truth::default();
        for counter in 0..=2 {
            let sigma_bn = sigma_bn_list[counter];
            let dcm_bn = mrp_to_dcm(sigma_bn);
            let r_ls_b = dcm_bn * r_ls_n;
            let mut dum1 = p_hat_b.dot(&r_ls_b) / r_ls_b.norm();
            if dum1.abs() > 1.0 {
                dum1 /= dum1.abs();
            }
            let phi = dum1.acos();
            let mut sigma_br = if phi < small_angle {
                Vector3::zeros()
            } else {
                let mut e_hat_b = if PI - phi < small_angle {
                    e_hat_180_b
                } else {
                    p_hat_b.cross(&r_ls_b)
                };
                e_hat_b /= e_hat_b.norm();
                -(phi / 4.0).tan() * e_hat_b
            };

            let sigma_rn = add_mrp(sigma_bn, -sigma_br);

            if let Some(v_lp_n) = v_lp_n {
                let alignment_threshold = 0.1;
                let dcm_rn = mrp_to_dcm(sigma_rn);
                let v_hat = v_lp_n / v_lp_n.norm();
                let mut v_tp_b = dcm_rn * v_hat;
                v_tp_b /= v_tp_b.norm();
                let mut v_perp = p_hat_b.cross(&v_tp_b);
                if v_perp.norm() >= alignment_threshold {
                    v_perp /= v_perp.norm();
                    if c_hat_b.dot(&v_perp) < 0.0 {
                        v_perp = -v_perp;
                    }
                    let mut dot_prod_2 = c_hat_b.dot(&v_perp);
                    if dot_prod_2.abs() > 1.0 {
                        dot_prod_2 /= dot_prod_2.abs();
                    }
                    let angle = dot_prod_2.acos();
                    let sigma_r2r = if angle < small_angle {
                        Vector3::zeros()
                    } else {
                        let mut e_hat_r2r = c_hat_b.cross(&v_perp);
                        e_hat_r2r /= e_hat_r2r.norm();
                        (angle / 4.0).tan() * e_hat_r2r
                    };
                    let sigma_r2n = add_mrp(sigma_rn, sigma_r2r);
                    sigma_br = sub_mrp(sigma_bn, sigma_r2n);
                }
            }

            if counter >= 1 {
                let dsigma = (sigma_br - truth.sigma_br[counter - 1]) / dt;
                omega_br_b = b_inv_mrp(sigma_br) * dsigma * 4.0;
            }
            if use_3d_rate && v_lp_n.is_none() {
                let r_hat_ls_b = r_ls_b / r_ls_b.norm();
                omega_br_b += omega_bn_list[counter].dot(&r_hat_ls_b) * r_hat_ls_b;
            }

            truth.sigma_br.push(sigma_br);
            truth.omega_br_b.push(omega_br_b);
            truth
                .sigma_rn
                .push(add_mrp(sigma_bn_list[counter], -sigma_br));
            truth.omega_rn_b.push(omega_bn_list[counter] - omega_br_b);
            truth
                .omega_rn_n
                .push(dcm_bn.transpose() * (omega_bn_list[counter] - omega_br_b));
        }
        truth
    }

    /// Upstream test helper `checkReferenceGeometry`: `p_hat_b` of every
    /// reference attitude points at the target and, in strip imaging,
    /// `c_hat_b` is perpendicular to the strip direction wherever upstream's
    /// alignment check lets the strip rotation happen. Returns the failures.
    fn reference_geometry_failures(
        sigma_rn_list: &[Vector3<f64>],
        r_ln_n: Vector3<f64>,
        r_sn_n: Vector3<f64>,
        p_hat_b: Vector3<f64>,
        c_hat_b: Vector3<f64>,
        v_lp_n: Option<Vector3<f64>>,
        accuracy: f64,
    ) -> Vec<String> {
        let mut failures = Vec::new();
        let r_ls_n = r_ln_n - r_sn_n;
        let r_hat_ls_n = r_ls_n / r_ls_n.norm();
        for (index, sigma_rn) in sigma_rn_list.iter().enumerate() {
            let dcm_rn = mrp_to_dcm(*sigma_rn);
            let p_hat_n = dcm_rn.transpose() * p_hat_b;
            let misalignment = p_hat_n.cross(&r_hat_ls_n).norm();
            if misalignment > accuracy {
                failures.push(format!("pointing check at {index}: {misalignment:e}"));
            }
            if let Some(v_lp_n) = v_lp_n.filter(|v| v.norm() > 1e-12) {
                let v_hat_n = v_lp_n / v_lp_n.norm();
                let alignment_threshold = 0.1;
                let mut v_tp_b_check = dcm_rn * v_hat_n;
                v_tp_b_check /= v_tp_b_check.norm();
                let v_perp_check = p_hat_b.cross(&v_tp_b_check);
                if v_perp_check.norm() >= alignment_threshold {
                    let c_hat_n = dcm_rn.transpose() * c_hat_b;
                    let dot_product = c_hat_n.dot(&v_hat_n).abs();
                    if dot_product > accuracy {
                        failures.push(format!("scan-line check at {index}: {dot_product:e}"));
                    }
                }
            }
        }
        failures
    }

    /// Upstream `test_locationPointing.py::test_locationPointing`, every
    /// parametrization: five target headings, the three point targets and
    /// three strip velocities, 2D and 3D rate control, accuracy 1e-12. The
    /// attitude input advances by `omega_BN_B * dt` before each of the three
    /// updates at 0, 0.1 and 0.2 s. The reference geometry is checked to the
    /// module's `smallAngle`, as upstream.
    #[test]
    fn matches_upstream_location_pointing_unit_test() {
        let accuracy = 1e-12;
        let r_ls_n_cases = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [1.0, 1.0, 1.0],
        ];
        let location_cases = [
            (LocationType::Ground, [0.0, 0.0, 0.0]),
            (LocationType::CelestialBody, [0.0, 0.0, 0.0]),
            (LocationType::TargetSpacecraft, [0.0, 0.0, 0.0]),
            (LocationType::Strip, [0.0, 1.0, 0.0]),
            (LocationType::Strip, [0.0, 0.0, 1.0]),
            (LocationType::Strip, [1.0, 0.0, 0.0]),
        ];

        for r_ls_n_in in r_ls_n_cases {
            for (location_type, v_lp_n_in) in location_cases {
                for use_3d_rate in [true, false] {
                    let failures = run_upstream_case(
                        r_ls_n_in,
                        location_type,
                        v_lp_n_in,
                        use_3d_rate,
                        accuracy,
                    );
                    assert!(
                        failures.is_empty(),
                        "r_LS_N {r_ls_n_in:?}, {location_type:?}, v_LP_N {v_lp_n_in:?}, \
                         use3DRate {use_3d_rate}: {failures:?}"
                    );
                }
            }
        }
    }

    /// Upstream `locationPointingTestFunction`; returns the failed checks.
    fn run_upstream_case(
        r_ls_n_in: [f64; 3],
        location_type: LocationType,
        v_lp_n_in: [f64; 3],
        use_3d_rate: bool,
        accuracy: f64,
    ) -> Vec<String> {
        let p_hat_b = Vector3::new(1.0, 0.0, 0.0);
        let c_hat_b = Vector3::new(0.0, 1.0, 0.0);
        let r_sn_n = Vector3::new(10.0, 11.0, 12.0);
        let r_ls_n = Vector3::from(r_ls_n_in);
        let v_lp_n = Vector3::from(v_lp_n_in);
        let omega_bn_b = Vector3::new(0.001, 0.002, 0.003);
        let sigma_bn_0 = Vector3::zeros();
        let r_ln_n = r_ls_n + r_sn_n;
        let r_pn_n = Vector3::new(5.0, 6.0, 7.0);
        let r_lp_n_pos = r_ln_n - r_pn_n;

        let eps = 0.1 * D2R;
        let mut module = LocationPointing::new(LocationPointingConfig {
            name: "locationPointingTag".to_string(),
            p_hat_b,
            c_hat_b,
            small_angle_rad: eps,
            use_boresight_rate_damping: use_3d_rate,
            ..Default::default()
        });

        let sc_trans_output = Output::new(NavigationTranslationMsg {
            position_bn_n_m: r_sn_n,
            ..Default::default()
        });
        let sc_att_output = Output::new(NavigationAttitudeMsg {
            omega_bn_b_radps: omega_bn_b,
            sigma_bn: sigma_bn_0,
            ..Default::default()
        });
        let ground_output = Output::new(GroundStateMsg {
            position_ln_n_m: r_ln_n,
            ..Default::default()
        });
        let body_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: r_ln_n,
            ..Default::default()
        });
        let target_output = Output::new(NavigationTranslationMsg {
            position_bn_n_m: r_ln_n,
            ..Default::default()
        });
        let strip_output = Output::new(StripStateMsg {
            position_ln_n_m: r_ln_n,
            position_lp_n_m: r_lp_n_pos,
            velocity_lp_n_mps: v_lp_n,
            ..Default::default()
        });
        match location_type {
            LocationType::Ground => ground_output.connect_to(&mut module.location_in_msg),
            LocationType::CelestialBody => body_output.connect_to(&mut module.cel_body_in_msg),
            LocationType::TargetSpacecraft => {
                target_output.connect_to(&mut module.sc_target_in_msg)
            }
            LocationType::Strip => strip_output.connect_to(&mut module.locationstrip_in_msg),
        }
        sc_trans_output.connect_to(&mut module.sc_trans_in_msg);
        sc_att_output.connect_to(&mut module.sc_att_in_msg);

        module.init();
        let mut sigma_bn_record = Vec::new();
        let mut omega_bn_record = Vec::new();
        let mut guidance: Vec<AttitudeGuidanceMsg> = Vec::new();
        let mut reference: Vec<AttitudeReferenceMsg> = Vec::new();
        for counter in 0..3_u32 {
            let attitude = NavigationAttitudeMsg {
                omega_bn_b_radps: omega_bn_b,
                sigma_bn: sigma_bn_0 + omega_bn_b * TIME_STEP_S * f64::from(counter),
                ..Default::default()
            };
            sc_att_output.write(attitude.clone());
            module.update(&context(u64::from(counter) * TIME_STEP_NANOS));
            sigma_bn_record.push(attitude.sigma_bn);
            omega_bn_record.push(attitude.omega_bn_b_radps);
            guidance.push(module.att_guid_out_msg.read());
            reference.push(module.att_ref_out_msg.read());
        }

        let is_strip = matches!(location_type, LocationType::Strip);
        let truth = upstream_truth(
            p_hat_b,
            c_hat_b,
            r_ln_n,
            r_sn_n,
            &sigma_bn_record,
            &omega_bn_record,
            eps,
            TIME_STEP_S,
            use_3d_rate,
            is_strip.then_some(v_lp_n),
        );

        let mut failures = Vec::new();
        let mut check = |name: &str, actual: Vector3<f64>, expected: Vector3<f64>, i: usize| {
            if !vector_close(actual, expected, accuracy) {
                failures.push(format!("{name} at {i}: {actual:?} != {expected:?}"));
            }
        };
        for i in 0..3 {
            check("sigma_BR", guidance[i].sigma_br, truth.sigma_br[i], i);
            check(
                "omega_BR_B",
                guidance[i].omega_br_b_radps,
                truth.omega_br_b[i],
                i,
            );
            check("sigma_RN", reference[i].sigma_rn, truth.sigma_rn[i], i);
            check(
                "omega_RN_B",
                guidance[i].omega_rn_b_radps,
                truth.omega_rn_b[i],
                i,
            );
            check(
                "omega_RN_N",
                reference[i].omega_rn_n_radps,
                truth.omega_rn_n[i],
                i,
            );
        }

        let sigma_rn_list: Vec<_> = reference.iter().map(|r| r.sigma_rn).collect();
        failures.extend(reference_geometry_failures(
            &sigma_rn_list,
            r_ln_n,
            r_sn_n,
            p_hat_b,
            c_hat_b,
            is_strip.then_some(v_lp_n),
            eps,
        ));
        failures
    }

    fn pointing_module(p_hat_b: Vector3<f64>) -> LocationPointing {
        LocationPointing::new(LocationPointingConfig {
            p_hat_b,
            c_hat_b: Vector3::new(0.0, 1.0, 0.0),
            small_angle_rad: 0.1 * D2R,
            ..Default::default()
        })
    }

    // Not upstream: a target directly behind p_hat_b uses the 180 deg eigen
    // axis. For p_hat_b = x, x × x vanishes, so the axis is x × y = z and
    // σ_B/R = -tan(π/4) z.
    #[test]
    fn target_behind_the_pointing_axis_uses_the_180_degree_eigen_axis() {
        let sc_trans_output = Output::new(NavigationTranslationMsg::default());
        let sc_att_output = Output::new(NavigationAttitudeMsg::default());
        let ground_output = Output::new(GroundStateMsg {
            position_ln_n_m: Vector3::new(-3.0, 0.0, 0.0),
            ..Default::default()
        });
        let mut module = pointing_module(Vector3::new(1.0, 0.0, 0.0));
        sc_trans_output.connect_to(&mut module.sc_trans_in_msg);
        sc_att_output.connect_to(&mut module.sc_att_in_msg);
        ground_output.connect_to(&mut module.location_in_msg);
        module.init();
        module.update(&context(0));

        assert_vector_close(
            module.att_guid_out_msg.read().sigma_br,
            Vector3::new(0.0, 0.0, -1.0),
            1e-12,
        );
        let p_hat_n = mrp_to_dcm(module.att_ref_out_msg.read().sigma_rn).transpose() * Vector3::x();
        assert_vector_close(p_hat_n, Vector3::new(-1.0, 0.0, 0.0), 1e-12);
    }

    // Not upstream: as upstream, reset normalizes the axes in place and
    // replaces non-positive thresholds with their defaults; c_hat_b is only
    // touched when strip imaging is connected.
    #[test]
    fn reset_normalizes_the_axes_and_applies_default_thresholds() {
        let mut module = pointing_module(Vector3::new(0.0, 2.0, 0.0));
        module.config.c_hat_b = Vector3::new(0.0, 0.0, 2.0);
        module.config.alignment_threshold = -1.0;
        module.init();
        assert_eq!(module.config.p_hat_b, Vector3::new(0.0, 1.0, 0.0));
        assert_eq!(module.config.c_hat_b, Vector3::new(0.0, 0.0, 2.0));
        assert_eq!(module.config.alignment_threshold, 0.1);
        assert_eq!(module.config.strip_inertial_speed_threshold_mps, 1e-12);
        // p_hat_b = y: y × x = -z.
        assert_eq!(module.e_hat_180_b, Vector3::new(0.0, 0.0, -1.0));

        let strip_output = Output::new(StripStateMsg::default());
        strip_output.connect_to(&mut module.locationstrip_in_msg);
        module.config.alignment_threshold = 0.3;
        Module::reset(&mut module, &context(0));
        assert_eq!(module.config.c_hat_b, Vector3::new(0.0, 0.0, 1.0));
        assert_eq!(module.config.alignment_threshold, 0.3);
    }

    // Not upstream: as upstream, a reset with p_hat_b shorter than 0.1 logs an
    // error and leaves both p_hat_b and the 180 deg eigen axis as they were.
    #[test]
    fn invalid_p_hat_keeps_the_previous_180_degree_eigen_axis() {
        let mut module = pointing_module(Vector3::new(0.0, 0.0, 3.0));
        module.init();
        // p_hat_b = z: z × x = y.
        assert_eq!(module.e_hat_180_b, Vector3::new(0.0, 1.0, 0.0));

        module.config.p_hat_b = Vector3::new(0.05, 0.0, 0.0);
        Module::reset(&mut module, &context(0));
        assert_eq!(module.config.p_hat_b, Vector3::new(0.05, 0.0, 0.0));
        assert_eq!(module.e_hat_180_b, Vector3::new(0.0, 1.0, 0.0));
    }

    // Not upstream: a strip target without speed has no scanning direction, so
    // the module falls back to exactly the location-pointing outputs (without
    // boresight rate damping, which strip imaging ignores).
    #[test]
    fn strip_target_without_speed_falls_back_to_location_pointing() {
        let r_ln_n = Vector3::new(3.0, 4.0, -2.0);
        let run = |strip: bool| {
            let sc_trans_output = Output::new(NavigationTranslationMsg::default());
            let sc_att_output = Output::new(NavigationAttitudeMsg {
                sigma_bn: Vector3::new(0.1, -0.2, 0.05),
                omega_bn_b_radps: Vector3::new(0.01, 0.02, -0.01),
                ..Default::default()
            });
            let ground_output = Output::new(GroundStateMsg {
                position_ln_n_m: r_ln_n,
                ..Default::default()
            });
            let strip_output = Output::new(StripStateMsg {
                position_ln_n_m: r_ln_n,
                ..Default::default()
            });
            let mut module = pointing_module(Vector3::new(0.0, 0.0, 1.0));
            sc_trans_output.connect_to(&mut module.sc_trans_in_msg);
            sc_att_output.connect_to(&mut module.sc_att_in_msg);
            if strip {
                strip_output.connect_to(&mut module.locationstrip_in_msg);
            } else {
                ground_output.connect_to(&mut module.location_in_msg);
            }
            module.init();
            module.update(&context(0));
            module.update(&context(TIME_STEP_NANOS));
            (
                module.att_guid_out_msg.read(),
                module.att_ref_out_msg.read(),
            )
        };

        let (strip_guidance, strip_reference) = run(true);
        let (guidance, reference) = run(false);
        assert_eq!(strip_guidance.sigma_br, guidance.sigma_br);
        assert_eq!(strip_guidance.omega_br_b_radps, guidance.omega_br_b_radps);
        assert_eq!(strip_guidance.omega_rn_b_radps, guidance.omega_rn_b_radps);
        assert_eq!(strip_reference.sigma_rn, reference.sigma_rn);
        assert_eq!(strip_reference.omega_rn_n_radps, reference.omega_rn_n_radps);
    }

    // Not upstream: with several target inputs connected, upstream uses the
    // first of ground location, strip, celestial body and target spacecraft.
    // With none connected, the unconnected target spacecraft reads as zero, so
    // p_hat_b is aimed at the inertial origin.
    #[test]
    fn uses_the_first_connected_target_in_upstream_priority_order() {
        let r_bn_n = Vector3::new(1.0, -2.0, 0.5);
        let sc_trans_output = Output::new(NavigationTranslationMsg {
            position_bn_n_m: r_bn_n,
            ..Default::default()
        });
        let sc_att_output = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.1, 0.2, -0.3),
            ..Default::default()
        });
        let ground_output = Output::new(GroundStateMsg {
            position_ln_n_m: Vector3::new(5.0, 1.0, 2.0),
            ..Default::default()
        });
        let strip_output = Output::new(StripStateMsg {
            position_ln_n_m: Vector3::new(-3.0, 4.0, 1.0),
            velocity_lp_n_mps: Vector3::new(0.0, 0.0, 2.0),
            ..Default::default()
        });
        let body_output = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(2.0, 7.0, -1.0),
            ..Default::default()
        });
        let target_output = Output::new(NavigationTranslationMsg {
            position_bn_n_m: Vector3::new(-6.0, -1.0, 3.0),
            ..Default::default()
        });
        // Reference attitude with the given [ground, strip, body, target]
        // inputs connected.
        let reference = |connected: [bool; 4]| {
            let mut module = pointing_module(Vector3::new(1.0, 0.0, 0.0));
            sc_trans_output.connect_to(&mut module.sc_trans_in_msg);
            sc_att_output.connect_to(&mut module.sc_att_in_msg);
            if connected[0] {
                ground_output.connect_to(&mut module.location_in_msg);
            }
            if connected[1] {
                strip_output.connect_to(&mut module.locationstrip_in_msg);
            }
            if connected[2] {
                body_output.connect_to(&mut module.cel_body_in_msg);
            }
            if connected[3] {
                target_output.connect_to(&mut module.sc_target_in_msg);
            }
            module.init();
            module.update(&context(0));
            module.att_ref_out_msg.read().sigma_rn
        };

        let ground = reference([true, false, false, false]);
        let strip = reference([false, true, false, false]);
        let body = reference([false, false, true, false]);
        let target = reference([false, false, false, true]);
        assert_eq!(reference([true, true, true, true]), ground);
        assert_eq!(reference([false, true, true, true]), strip);
        assert_eq!(reference([false, false, true, true]), body);
        for (index, a) in [ground, strip, body, target].iter().enumerate() {
            for b in &[ground, strip, body, target][index + 1..] {
                assert_ne!(a, b, "the targets give distinct references");
            }
        }

        let p_hat_n = mrp_to_dcm(reference([false; 4])).transpose() * Vector3::x();
        assert_vector_close(p_hat_n, -r_bn_n / r_bn_n.norm(), 1e-12);
    }

    // Not upstream: reset restarts the finite-difference rate, so the first
    // update after it reports a zero tracking-error rate.
    #[test]
    fn reset_restarts_the_finite_difference_rate() {
        let sc_trans_output = Output::new(NavigationTranslationMsg::default());
        let sc_att_output = Output::new(NavigationAttitudeMsg::default());
        let ground_output = Output::new(GroundStateMsg {
            position_ln_n_m: Vector3::new(1.0, 2.0, 3.0),
            ..Default::default()
        });
        let mut module = pointing_module(Vector3::new(1.0, 0.0, 0.0));
        sc_trans_output.connect_to(&mut module.sc_trans_in_msg);
        sc_att_output.connect_to(&mut module.sc_att_in_msg);
        ground_output.connect_to(&mut module.location_in_msg);
        module.init();

        let attitude = |sigma_bn: Vector3<f64>| NavigationAttitudeMsg {
            sigma_bn,
            ..Default::default()
        };
        sc_att_output.write(attitude(Vector3::zeros()));
        module.update(&context(0));
        assert_eq!(
            module.att_guid_out_msg.read().omega_br_b_radps,
            Vector3::zeros()
        );
        sc_att_output.write(attitude(Vector3::new(0.01, 0.0, 0.0)));
        module.update(&context(TIME_STEP_NANOS));
        assert_ne!(
            module.att_guid_out_msg.read().omega_br_b_radps,
            Vector3::zeros()
        );

        Module::reset(&mut module, &context(2 * TIME_STEP_NANOS));
        sc_att_output.write(attitude(Vector3::new(0.02, 0.0, 0.0)));
        module.update(&context(2 * TIME_STEP_NANOS));
        assert_eq!(
            module.att_guid_out_msg.read().omega_br_b_radps,
            Vector3::zeros()
        );
    }
}
