//! Reference attitude that points a body heading along an inertial heading
//! while keeping the solar-array drive axis as close to perpendicular to the
//! Sun as possible: port of upstream Basilisk `oneAxisSolarArrayPoint`.
//!
//! The reference frame is built from three successive rotations of the body
//! frame (R. Calaon, C. Allard and H. Schaub, "Attitude Reference Generation
//! for Spacecraft with Rotating Solar Arrays and Pointing Constraints"):
//!
//! 1. the shortest rotation that aligns the body heading with the inertial
//!    heading;
//! 2. a rotation about the body heading that makes the drive axis `a1` as
//!    perpendicular to the Sun as possible, using `a2` (a body axis to keep
//!    close to the Sun) to choose between the two exact solutions;
//! 3. with [`AlignmentPriority::SolarArrayAlignment`] only, a rotation that
//!    breaks the heading alignment to make `a1` exactly perpendicular to the
//!    Sun.
//!
//! When a second body heading `h2` is configured, the frame built on it
//! replaces the first if it brings `a2` closer to the Sun. The reference rate
//! and acceleration come from first- and second-order finite differences of
//! the reference attitude over the last three updates.

use std::f64::consts::{FRAC_PI_2, PI};

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AttitudeReferenceMsg, BodyHeadingMsg, EphemerisMsg, InertialHeadingMsg, Input,
    NavigationAttitudeMsg, NavigationTranslationMsg, Output,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::linear_algebra::{normalize_or_zero, v3_perpendicular};
use crate::utilities::rigid_body_kinematics::{
    d_mrp_to_omega, dcm_to_mrp, dd_mrp_to_domega, mrp_shadow, mrp_to_dcm, prv_to_dcm,
};
use crate::{Module, SimulationContext};

/// Upstream `epsilon`: the module's tolerance for zero.
const EPSILON: f64 = 1e-12;

/// Which pointing constraint is met exactly: upstream `AlignmentPriority`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum AlignmentPriority {
    /// Upstream `prioritizeAxisAlignment` (0): the body heading lies exactly on
    /// the inertial heading, and the solar-array incidence is the best it can
    /// be.
    #[default]
    AxisAlignment = 0,
    /// Upstream `prioritizeSolarArrayAlignment` (1): the drive axis is exactly
    /// perpendicular to the Sun, and the body heading is as close to the
    /// inertial heading as it can be.
    SolarArrayAlignment = 1,
}

/// Where the body heading comes from: upstream `BodyAxisInput`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum BodyAxisSource {
    /// Upstream `inputBodyHeadingParameter`: the `h1_hat_b` parameter.
    #[default]
    Parameter,
    /// Upstream `inputBodyHeadingMsg`: `body_heading_in_msg`.
    Message,
}

/// Where the inertial heading comes from: upstream `InertialAxisInput`.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
enum InertialAxisSource {
    /// Upstream `inputInertialHeadingParameter`: the `h_hat_n` parameter.
    #[default]
    Parameter,
    /// Upstream `inputInertialHeadingMsg`: `inertial_heading_in_msg`.
    Message,
    /// Upstream `inputEphemerisMsg`: from the spacecraft to the body of
    /// `ephemeris_in_msg`.
    Ephemeris,
}

/// User parameters of [`OneAxisSolarArrayPoint`]; the defaults are upstream's
/// zero-initialized ones.
#[derive(Clone, Debug)]
pub struct OneAxisSolarArrayPointConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `a1Hat_B` [-]: solar-array drive axis, body components. Must
    /// be set; it is normalized before use.
    pub a1_hat_b: Vector3<f64>,
    /// Upstream `alignmentPriority`: which constraint is met exactly.
    pub alignment_priority: AlignmentPriority,
    /// Upstream `h1Hat_B` [-]: body heading to point along the inertial
    /// heading, used when `body_heading_in_msg` is not connected.
    pub h1_hat_b: Vector3<f64>,
    /// Upstream `h2Hat_B` [-]: optional second body heading; zero disables
    /// it. Upstream uses it without normalizing it.
    pub h2_hat_b: Vector3<f64>,
    /// Upstream `hHat_N` [-]: inertial heading, used when neither
    /// `inertial_heading_in_msg` nor `ephemeris_in_msg` is connected.
    pub h_hat_n: Vector3<f64>,
    /// Upstream `a2Hat_B` [-]: optional body axis to keep as close to the Sun
    /// as possible; zero disables it.
    pub a2_hat_b: Vector3<f64>,
}

impl Default for OneAxisSolarArrayPointConfig {
    fn default() -> Self {
        Self {
            name: "oneAxisSolarArrayPoint".to_string(),
            a1_hat_b: Vector3::zeros(),
            alignment_priority: AlignmentPriority::default(),
            h1_hat_b: Vector3::zeros(),
            h2_hat_b: Vector3::zeros(),
            h_hat_n: Vector3::zeros(),
            a2_hat_b: Vector3::zeros(),
        }
    }
}

/// Reference attitudes of the previous updates, for the finite differences
/// (upstream `updateCallCount`, `T1NanoSeconds`, `T2NanoSeconds`,
/// `sigma_RN_1`, `sigma_RN_2`).
#[derive(Clone, Copy, Debug, Default)]
enum ReferenceHistory {
    /// No update since the last reset: the rates are zero.
    #[default]
    Empty,
    /// One previous update: first-order differences.
    One {
        time_nanos: u64,
        sigma_rn: Vector3<f64>,
    },
    /// Two or more previous updates: second-order differences.
    Two {
        last_time_nanos: u64,
        last_sigma_rn: Vector3<f64>,
        before_last_time_nanos: u64,
        before_last_sigma_rn: Vector3<f64>,
    },
}

/// Solar-array-aware one-axis pointing guidance: upstream
/// `oneAxisSolarArrayPoint`.
#[derive(Clone, Debug)]
pub struct OneAxisSolarArrayPoint {
    /// Module configuration.
    pub config: OneAxisSolarArrayPointConfig,
    /// Upstream `attNavInMsg`: attitude and Sun direction (required).
    pub att_nav_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `bodyHeadingInMsg`: optional body heading; overrides
    /// `h1_hat_b`.
    pub body_heading_in_msg: Input<BodyHeadingMsg>,
    /// Upstream `inertialHeadingInMsg`: optional inertial heading; overrides
    /// `h_hat_n` and `ephemeris_in_msg`.
    pub inertial_heading_in_msg: Input<InertialHeadingMsg>,
    /// Upstream `transNavInMsg`: spacecraft position, needed with
    /// `ephemeris_in_msg`.
    pub trans_nav_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `ephemerisInMsg`: optional body to point the heading at.
    pub ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `attRefOutMsg`: reference attitude, rate and acceleration.
    pub att_ref_out_msg: Output<AttitudeReferenceMsg>,
    body_axis_source: BodyAxisSource,
    inertial_axis_source: InertialAxisSource,
    history: ReferenceHistory,
}

impl OneAxisSolarArrayPoint {
    /// Builds the module with unconnected inputs.
    pub fn new(config: OneAxisSolarArrayPointConfig) -> Self {
        Self {
            config,
            att_nav_in_msg: Input::default(),
            body_heading_in_msg: Input::default(),
            inertial_heading_in_msg: Input::default(),
            trans_nav_in_msg: Input::default(),
            ephemeris_in_msg: Input::default(),
            att_ref_out_msg: Output::default(),
            body_axis_source: BodyAxisSource::default(),
            inertial_axis_source: InertialAxisSource::default(),
            history: ReferenceHistory::default(),
        }
    }

    /// Upstream `Reset_oneAxisSolarArrayPoint`: choose the heading sources and
    /// restart the finite differences. A source that cannot be resolved logs
    /// an error and keeps its previous value, as upstream.
    pub fn reset(&mut self) {
        if !self.att_nav_in_msg.is_connected() {
            log::error!("oneAxisSolarArrayPoint.att_nav_in_msg is not connected");
        }

        if self.body_heading_in_msg.is_connected() {
            self.body_axis_source = BodyAxisSource::Message;
        } else if self.config.h1_hat_b.norm() > EPSILON {
            self.body_axis_source = BodyAxisSource::Parameter;
        } else {
            log::error!(
                "oneAxisSolarArrayPoint.body_heading_in_msg is not connected and no body heading h1_hat_b was specified"
            );
        }

        if self.inertial_heading_in_msg.is_connected() {
            self.inertial_axis_source = InertialAxisSource::Message;
            if self.ephemeris_in_msg.is_connected() {
                log::warn!(
                    "both oneAxisSolarArrayPoint.inertial_heading_in_msg and oneAxisSolarArrayPoint.ephemeris_in_msg are connected; the inertial heading is taken from inertial_heading_in_msg"
                );
            }
        } else if self.ephemeris_in_msg.is_connected() {
            if self.trans_nav_in_msg.is_connected() {
                self.inertial_axis_source = InertialAxisSource::Ephemeris;
            } else {
                log::error!(
                    "oneAxisSolarArrayPoint.ephemeris_in_msg is connected but oneAxisSolarArrayPoint.trans_nav_in_msg is not"
                );
            }
        } else if self.config.h_hat_n.norm() > EPSILON {
            self.inertial_axis_source = InertialAxisSource::Parameter;
        } else {
            log::error!(
                "neither oneAxisSolarArrayPoint.inertial_heading_in_msg nor oneAxisSolarArrayPoint.ephemeris_in_msg is connected and no inertial heading h_hat_n was specified"
            );
        }

        self.history = ReferenceHistory::Empty;
    }

    fn requested_heading_n(&self) -> Vector3<f64> {
        match self.inertial_axis_source {
            InertialAxisSource::Parameter => normalize_or_zero(self.config.h_hat_n),
            InertialAxisSource::Message => {
                normalize_or_zero(self.inertial_heading_in_msg.read().r_hat_xn_n)
            }
            InertialAxisSource::Ephemeris => normalize_or_zero(
                self.ephemeris_in_msg.read().position_bdy_zero_n_m
                    - self.trans_nav_in_msg.read().position_bn_n_m,
            ),
        }
    }

    fn body_heading_b(&self) -> Vector3<f64> {
        match self.body_axis_source {
            BodyAxisSource::Parameter => normalize_or_zero(self.config.h1_hat_b),
            BodyAxisSource::Message => {
                normalize_or_zero(self.body_heading_in_msg.read().r_hat_xb_b)
            }
        }
    }

    /// Reference attitude rate and acceleration `(σ̇_RN, σ̈_RN)` by finite
    /// differences over the stored history, which this call then advances.
    fn reference_rates(
        &mut self,
        call_time_nanos: u64,
        sigma_rn: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let (rates, history) = match self.history {
            ReferenceHistory::Empty => (
                (Vector3::zeros(), Vector3::zeros()),
                ReferenceHistory::One {
                    time_nanos: call_time_nanos,
                    sigma_rn,
                },
            ),
            ReferenceHistory::One {
                time_nanos,
                sigma_rn: last_sigma_rn,
            } => {
                let sigma_1 = closest_mrp_set(last_sigma_rn, sigma_rn);
                // Upstream's signed differences are negative: t_1 - t.
                let t1 = diff_nanos_to_seconds(time_nanos, call_time_nanos);
                (
                    ((sigma_1 - sigma_rn) / t1, Vector3::zeros()),
                    ReferenceHistory::Two {
                        last_time_nanos: call_time_nanos,
                        last_sigma_rn: sigma_rn,
                        before_last_time_nanos: time_nanos,
                        before_last_sigma_rn: last_sigma_rn,
                    },
                )
            }
            ReferenceHistory::Two {
                last_time_nanos,
                last_sigma_rn,
                before_last_time_nanos,
                before_last_sigma_rn,
            } => {
                let sigma_1 = closest_mrp_set(last_sigma_rn, sigma_rn);
                // Upstream compares the older set with the possibly switched
                // newer one, not with the current attitude.
                let sigma_2 = closest_mrp_set(before_last_sigma_rn, sigma_1);
                let t1 = diff_nanos_to_seconds(last_time_nanos, call_time_nanos);
                let t2 = diff_nanos_to_seconds(before_last_time_nanos, call_time_nanos);
                let sigma_dot = ((sigma_1 * t2 * t2 - sigma_2 * t1 * t1) / (t2 - t1)
                    - sigma_rn * (t2 + t1))
                    / t1
                    / t2;
                let sigma_ddot =
                    2.0 * ((sigma_1 * t2 - sigma_2 * t1) / (t1 - t2) + sigma_rn) / t1 / t2;
                (
                    (sigma_dot, sigma_ddot),
                    ReferenceHistory::Two {
                        last_time_nanos: call_time_nanos,
                        last_sigma_rn: sigma_rn,
                        before_last_time_nanos: last_time_nanos,
                        before_last_sigma_rn: last_sigma_rn,
                    },
                )
            }
        };
        self.history = history;
        rates
    }

    /// Upstream `Update_oneAxisSolarArrayPoint`.
    fn compute_reference(&mut self, call_time_nanos: u64) -> AttitudeReferenceMsg {
        let navigation = self.att_nav_in_msg.read();
        let h_req_hat_n = self.requested_heading_n();
        let h_ref_hat_b = self.body_heading_b();

        let dcm_bn = mrp_to_dcm(navigation.sigma_bn);
        let a2_hat_b = if self.config.a2_hat_b.norm() > EPSILON {
            normalize_or_zero(self.config.a2_hat_b)
        } else {
            Vector3::zeros()
        };
        let geometry = PointingGeometry {
            alignment_priority: self.config.alignment_priority,
            dcm_bn,
            r_hat_sb_b: navigation.vehicle_sun_point_body,
            h_req_hat_b: dcm_bn * h_req_hat_n,
            a1_hat_b: normalize_or_zero(self.config.a1_hat_b),
            a2_hat_b,
        };

        let mut dcm_rn = geometry.reference_frame(h_ref_hat_b);
        let mut sigma_rn = dcm_to_mrp(dcm_rn);
        if self.config.h2_hat_b.norm() > EPSILON {
            let r_hat_sb_r1 = geometry.sun_direction_in(&dcm_rn);
            // Upstream overwrites its `RN` with the second frame whichever
            // frame wins, so the rates below are always mapped to the
            // inertial frame through the second frame's DCM. Kept as upstream.
            dcm_rn = geometry.reference_frame(self.config.h2_hat_b);
            let r_hat_sb_r2 = geometry.sun_direction_in(&dcm_rn);
            let dot_1 = r_hat_sb_r1.dot(&geometry.a2_hat_b);
            let dot_2 = r_hat_sb_r2.dot(&geometry.a2_hat_b);
            if dot_2 > dot_1 && (dot_2 - dot_1).abs() > EPSILON {
                sigma_rn = dcm_to_mrp(dcm_rn);
            }
        }

        let (sigma_dot_rn, sigma_ddot_rn) = self.reference_rates(call_time_nanos, sigma_rn);
        let omega_rn_r = d_mrp_to_omega(sigma_rn, sigma_dot_rn);
        let domega_rn_r = dd_mrp_to_domega(sigma_rn, sigma_dot_rn, sigma_ddot_rn);

        AttitudeReferenceMsg {
            sigma_rn,
            omega_rn_n_radps: dcm_rn.tr_mul(&omega_rn_r),
            domega_rn_n_radps2: dcm_rn.tr_mul(&domega_rn_r),
        }
    }
}

impl Module for OneAxisSolarArrayPoint {
    fn init(&mut self) {
        self.att_ref_out_msg.write(AttitudeReferenceMsg::default());
        OneAxisSolarArrayPoint::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        OneAxisSolarArrayPoint::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        let reference = self.compute_reference(context.current_sim_nanos);
        self.att_ref_out_msg.write(reference);
    }
}

/// `previous`, or its shadow set when it is more than 1 away from `current`
/// (upstream's switch before differencing).
fn closest_mrp_set(previous: Vector3<f64>, current: Vector3<f64>) -> Vector3<f64> {
    if (current - previous).norm() > 1.0 {
        mrp_shadow(previous)
    } else {
        previous
    }
}

/// The inputs of one reference-frame construction (upstream
/// `oasapComputeFinalRotation` without its body heading).
struct PointingGeometry {
    alignment_priority: AlignmentPriority,
    dcm_bn: Matrix3<f64>,
    /// Sun direction, body components (not normalized, as upstream).
    r_hat_sb_b: Vector3<f64>,
    /// Requested inertial heading, body components.
    h_req_hat_b: Vector3<f64>,
    a1_hat_b: Vector3<f64>,
    a2_hat_b: Vector3<f64>,
}

impl PointingGeometry {
    /// `[RN]` of the frame that points `h_ref_hat_b` (upstream
    /// `oasapComputeFinalRotation`).
    fn reference_frame(&self, h_ref_hat_b: Vector3<f64>) -> Matrix3<f64> {
        let dcm_r1b = first_rotation(h_ref_hat_b, self.h_req_hat_b);
        let r_hat_sb_r1 = dcm_r1b * self.r_hat_sb_b;
        let dcm_r2r1 = second_rotation(h_ref_hat_b, r_hat_sb_r1, self.a1_hat_b, self.a2_hat_b);
        let r_hat_sb_r2 = dcm_r2r1 * r_hat_sb_r1;
        let dcm_r3r2 = third_rotation(
            self.alignment_priority,
            h_ref_hat_b,
            r_hat_sb_r2,
            self.a1_hat_b,
        );
        let dcm_r1n = dcm_r1b * self.dcm_bn;
        let dcm_r2n = dcm_r2r1 * dcm_r1n;
        dcm_r3r2 * dcm_r2n
    }

    /// The Sun direction in the frame `dcm_rn`, through `[RB] = [RN][BN]^T`.
    fn sun_direction_in(&self, dcm_rn: &Matrix3<f64>) -> Vector3<f64> {
        let dcm_rb = dcm_rn * self.dcm_bn.transpose();
        dcm_rb * self.r_hat_sb_b
    }
}

/// Upstream `fmin(fmax(x, -1), 1)`. Unlike `f64::clamp`, it maps NaN to -1.
#[allow(clippy::manual_clamp)]
fn fmin_fmax_unit(x: f64) -> f64 {
    x.max(-1.0).min(1.0)
}

/// `[R1B]`: the principal rotation that takes `h_ref_hat_b` onto
/// `h_req_hat_b` (upstream `oasapComputeFirstRotation`).
fn first_rotation(h_ref_hat_b: Vector3<f64>, h_req_hat_b: Vector3<f64>) -> Matrix3<f64> {
    let mut phi = fmin_fmax_unit(h_ref_hat_b.dot(&h_req_hat_b)).acos();
    let mut e_phi = h_ref_hat_b.cross(&h_req_hat_b);
    if (phi - PI).abs() < EPSILON {
        // Any axis perpendicular to the heading performs the half turn.
        phi = PI;
        e_phi = v3_perpendicular(h_ref_hat_b);
    } else if phi.abs() < EPSILON {
        phi = 0.0;
    }
    prv_to_dcm(phi * normalize_or_zero(e_phi))
}

/// `[R2R1]`: the rotation about the body heading that brings the drive axis
/// closest to perpendicular to the Sun (upstream `oasapComputeSecondRotation`).
///
/// Writing `t = tan(ψ/2)`, `a1 · r̂_S` after the rotation is
/// `(A t² + B t + C) / (1 + t²)`; its zeros give the perfect-incidence
/// rotations and, when there are none, its extremum the best one. The same
/// form with `E`, `F`, `G` measures `a2 · r̂_S` and picks between two zeros.
fn second_rotation(
    h_ref_hat_b: Vector3<f64>,
    r_hat_sb_r1: Vector3<f64>,
    a1_hat_b: Vector3<f64>,
    a2_hat_b: Vector3<f64>,
) -> Matrix3<f64> {
    let e_psi = h_ref_hat_b;
    let b3 = r_hat_sb_r1.cross(&e_psi);
    let a = 2.0 * r_hat_sb_r1.dot(&e_psi) * e_psi.dot(&a1_hat_b) - a1_hat_b.dot(&r_hat_sb_r1);
    let b = 2.0 * a1_hat_b.dot(&b3);
    let c = a1_hat_b.dot(&r_hat_sb_r1);
    let delta = b * b - 4.0 * a * c;

    let e = 2.0 * r_hat_sb_r1.dot(&e_psi) * e_psi.dot(&a2_hat_b) - a2_hat_b.dot(&r_hat_sb_r1);
    let f = 2.0 * a2_hat_b.dot(&b3);
    let g = a2_hat_b.dot(&r_hat_sb_r1);

    let incidence = |t: f64| (a * t * t + b * t + c) / (1.0 + t * t);
    let a2_alignment = |t: f64| (e * t * t + f * t + g) / (1.0 + t * t);

    let psi = if a.abs() < EPSILON {
        if b.abs() < EPSILON {
            // The equation has no solution; ψ = π minimizes the incidence.
            PI
        } else {
            2.0 * (-c / b).atan()
        }
    } else if delta < 0.0 {
        // No perfect incidence: take the extremum of the incidence function.
        let t = if b.abs() < EPSILON {
            0.0
        } else {
            let q = (a - c) / b;
            let t1 = q + (q * q + 1.0).sqrt();
            let t2 = q - (q * q + 1.0).sqrt();
            if incidence(t2).abs() < incidence(t1).abs() {
                t2
            } else {
                t1
            }
        };
        // t → ∞ (ψ = π) gives the value A, which may be smaller still.
        if a.abs() < incidence(t).abs() {
            PI
        } else {
            2.0 * t.atan()
        }
    } else {
        let t1 = (-b + delta.sqrt()) / (2.0 * a);
        let t2 = (-b - delta.sqrt()) / (2.0 * a);
        let mut t = t1;
        if (h_ref_hat_b.dot(&a2_hat_b) - 1.0).abs() > EPSILON
            && a2_alignment(t2) - a2_alignment(t1) > EPSILON
        {
            t = t2;
        }
        2.0 * t.atan()
    };

    prv_to_dcm(psi * e_psi)
}

/// `[R3R2]`: with [`AlignmentPriority::SolarArrayAlignment`], the rotation
/// that makes the drive axis exactly perpendicular to the Sun; the identity
/// otherwise (upstream `oasapComputeThirdRotation`).
fn third_rotation(
    alignment_priority: AlignmentPriority,
    h_ref_hat_b: Vector3<f64>,
    r_hat_sb_r2: Vector3<f64>,
    a1_hat_b: Vector3<f64>,
) -> Matrix3<f64> {
    let prv_theta = match alignment_priority {
        AlignmentPriority::AxisAlignment => Vector3::zeros(),
        AlignmentPriority::SolarArrayAlignment => {
            let s_theta = r_hat_sb_r2.dot(&a1_hat_b);
            let theta = fmin_fmax_unit(s_theta.abs()).asin();
            if theta.abs() < EPSILON {
                // Already perpendicular: no third rotation.
                Vector3::zeros()
            } else {
                let e_theta = if (theta.abs() - FRAC_PI_2).abs() > EPSILON {
                    // Project the drive axis onto the plane perpendicular to
                    // the Sun and rotate it there.
                    let a_p_hat_b = (a1_hat_b - s_theta * r_hat_sb_r2) / (1.0 - s_theta * s_theta);
                    a1_hat_b.cross(&a_p_hat_b)
                } else {
                    // Drive axis on the Sun line: rotate about the axis that
                    // moves the body heading least.
                    let mut a_p_hat_b = r_hat_sb_r2.cross(&h_ref_hat_b);
                    if a_p_hat_b.norm() < EPSILON {
                        a_p_hat_b = v3_perpendicular(r_hat_sb_r2);
                    }
                    a1_hat_b.cross(&a_p_hat_b)
                };
                theta * normalize_or_zero(e_theta)
            }
        }
    };
    prv_to_dcm(prv_theta)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use crate::messages::{
        BodyHeadingMsg, EphemerisMsg, InertialHeadingMsg, NavigationAttitudeMsg,
        NavigationTranslationMsg, Output,
    };
    use crate::utilities::rigid_body_kinematics::{mrp_to_dcm, prv_to_dcm};
    use crate::{Module, SimulationContext};

    use super::{AlignmentPriority, OneAxisSolarArrayPoint, OneAxisSolarArrayPointConfig};

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// `v / |v|`, as numpy divides by `np.linalg.norm`.
    fn unit(v: Vector3<f64>) -> Vector3<f64> {
        v / v.norm()
    }

    /// Upstream `computeGamma`: the best achievable angle between the drive
    /// axis and the plane perpendicular to the Sun.
    fn upstream_gamma(alpha: f64, delta: f64) -> f64 {
        if (0.0..=PI / 2.0).contains(&alpha) {
            if delta < PI / 2.0 - alpha {
                PI / 2.0 - alpha - delta
            } else if delta > alpha + PI / 2.0 {
                -PI / 2.0 - alpha + delta
            } else {
                0.0
            }
        } else if delta < alpha - PI / 2.0 {
            -PI / 2.0 + alpha - delta
        } else if delta > 3.0 / 2.0 * PI - alpha {
            alpha + delta - 3.0 / 2.0 * PI
        } else {
            0.0
        }
    }

    #[derive(Clone, Copy, Debug)]
    enum BodyInput {
        Parameter,
        Message,
    }

    #[derive(Clone, Copy, Debug)]
    enum InertialInput {
        Parameter,
        Message,
        Ephemeris,
    }

    /// Port of `test_oneAxisSolarArrayPointTestFunction`: every combination of
    /// `alpha`, `delta`, body input, inertial input and priority, checking the
    /// incidence angle of the first (t = 0) output against `computeGamma` to
    /// 1e-12.
    #[test]
    fn matches_upstream_one_axis_solar_array_point_unit_test() {
        let accuracy = 1e-12;
        // np.linspace(0, pi, 5, endpoint=False)
        let angles: Vec<f64> = (0..5).map(|i| f64::from(i) * (PI / 5.0)).collect();

        let r_hat_sb_n = unit(Vector3::new(1.0, 2.0, 3.0));
        let a1_hat_b = unit(Vector3::new(9.0, 8.0, 7.0));
        let a = unit(r_hat_sb_n.cross(&Vector3::new(4.0, 5.0, 6.0)));
        let d = unit(a1_hat_b.cross(&Vector3::new(6.0, 5.0, 4.0)));

        for &alpha in &angles {
            for &delta in &angles {
                for body_input in [BodyInput::Parameter, BodyInput::Message] {
                    for inertial_input in [
                        InertialInput::Parameter,
                        InertialInput::Message,
                        InertialInput::Ephemeris,
                    ] {
                        for priority in [
                            AlignmentPriority::AxisAlignment,
                            AlignmentPriority::SolarArrayAlignment,
                        ] {
                            let h_hat_n = prv_to_dcm(a * alpha) * r_hat_sb_n;
                            let h_hat_b = prv_to_dcm(d * delta) * a1_hat_b;

                            let mut config = OneAxisSolarArrayPointConfig {
                                a1_hat_b,
                                alignment_priority: priority,
                                ..Default::default()
                            };
                            // sigma_BN = 0, so the Sun direction is the same in B and N.
                            let navigation = Output::new(NavigationAttitudeMsg {
                                sigma_bn: Vector3::zeros(),
                                vehicle_sun_point_body: r_hat_sb_n,
                                ..Default::default()
                            });
                            let body_heading = Output::new(BodyHeadingMsg {
                                r_hat_xb_b: h_hat_b,
                            });
                            let inertial_heading = Output::new(InertialHeadingMsg {
                                r_hat_xn_n: h_hat_n,
                            });
                            let translation = Output::new(NavigationTranslationMsg::default());
                            let ephemeris = Output::new(EphemerisMsg {
                                position_bdy_zero_n_m: h_hat_n,
                                ..Default::default()
                            });
                            if matches!(body_input, BodyInput::Parameter) {
                                config.h1_hat_b = h_hat_b;
                            }
                            if matches!(inertial_input, InertialInput::Parameter) {
                                config.h_hat_n = h_hat_n;
                            }
                            let mut module = OneAxisSolarArrayPoint::new(config);
                            navigation.connect_to(&mut module.att_nav_in_msg);
                            if matches!(body_input, BodyInput::Message) {
                                body_heading.connect_to(&mut module.body_heading_in_msg);
                            }
                            match inertial_input {
                                InertialInput::Parameter => {}
                                InertialInput::Message => {
                                    inertial_heading.connect_to(&mut module.inertial_heading_in_msg)
                                }
                                InertialInput::Ephemeris => {
                                    translation.connect_to(&mut module.trans_nav_in_msg);
                                    ephemeris.connect_to(&mut module.ephemeris_in_msg);
                                }
                            }
                            module.init();
                            module.update(&context(0));

                            let dcm_rn = mrp_to_dcm(module.att_ref_out_msg.read().sigma_rn);
                            let a1_hat_n = dcm_rn.transpose() * a1_hat_b;
                            let gamma_sim = r_hat_sb_n.dot(&a1_hat_n).clamp(-1.0, 1.0).abs().asin();
                            let gamma_true = match priority {
                                AlignmentPriority::AxisAlignment => upstream_gamma(alpha, delta),
                                AlignmentPriority::SolarArrayAlignment => 0.0,
                            };
                            assert!(
                                (gamma_sim - gamma_true).abs() <= accuracy,
                                "alpha {alpha}, delta {delta}, {body_input:?}, {inertial_input:?}, {priority:?}: gamma {gamma_sim} != {gamma_true}"
                            );
                        }
                    }
                }
            }
        }
    }

    const SPIN_RATE_RADPS: f64 = 0.02;

    /// Body x pointed along an inertial heading that turns about inertial z
    /// at [`SPIN_RATE_RADPS`], with the Sun on z and the drive axis on body y
    /// (already perpendicular to the Sun): the reference is the pure rotation
    /// `[RN] = R_z(ωt)`, so `ω_RN = ω ẑ` and `ω̇_RN = 0`.
    fn spinning_heading_module() -> (OneAxisSolarArrayPoint, Output<InertialHeadingMsg>) {
        let navigation = Output::new(NavigationAttitudeMsg {
            vehicle_sun_point_body: Vector3::z(),
            ..Default::default()
        });
        let heading = Output::new(InertialHeadingMsg::default());
        let mut module = OneAxisSolarArrayPoint::new(OneAxisSolarArrayPointConfig {
            a1_hat_b: Vector3::y(),
            h1_hat_b: Vector3::x(),
            ..Default::default()
        });
        navigation.connect_to(&mut module.att_nav_in_msg);
        heading.connect_to(&mut module.inertial_heading_in_msg);
        module.init();
        (module, heading)
    }

    fn spin_to(
        module: &mut OneAxisSolarArrayPoint,
        heading: &Output<InertialHeadingMsg>,
        t_s: f64,
    ) {
        let angle = SPIN_RATE_RADPS * t_s;
        heading.write(InertialHeadingMsg {
            r_hat_xn_n: Vector3::new(angle.cos(), angle.sin(), 0.0),
        });
        module.update(&context((t_s * 1e9) as u64));
    }

    // Not upstream (its test checks only the first attitude): the finite
    // differences give zero rates first, then the first- and second-order
    // estimates of the true constant spin. The truncation errors for
    // ωΔt = 0.02 are about 1e-6 here; a wrong sign or formula would be off by
    // about ω = 0.02.
    #[test]
    fn finite_difference_rates_follow_a_spinning_reference() {
        let (mut module, heading) = spinning_heading_module();
        let spin = Vector3::new(0.0, 0.0, SPIN_RATE_RADPS);
        for (t_s, omega_tolerance, domega_tolerance) in [
            (0.0, 0.0, 0.0),
            (1.0, 1e-5, 1e-5),
            (2.0, 1e-5, 1e-5),
            (3.0, 1e-5, 1e-5),
        ] {
            spin_to(&mut module, &heading, t_s);
            let reference = module.att_ref_out_msg.read();
            let sigma_true = Vector3::new(0.0, 0.0, (SPIN_RATE_RADPS * t_s / 4.0).tan());
            assert!(
                (reference.sigma_rn - sigma_true).norm() < 1e-15,
                "t = {t_s}"
            );
            let omega_true = if t_s == 0.0 { Vector3::zeros() } else { spin };
            assert!(
                (reference.omega_rn_n_radps - omega_true).norm() <= omega_tolerance,
                "t = {t_s}: {}",
                reference.omega_rn_n_radps
            );
            assert!(
                reference.domega_rn_n_radps2.norm() <= domega_tolerance,
                "t = {t_s}: {}",
                reference.domega_rn_n_radps2
            );
        }
    }

    #[test]
    fn reset_restarts_the_finite_differences() {
        let (mut module, heading) = spinning_heading_module();
        for t_s in [0.0, 1.0, 2.0] {
            spin_to(&mut module, &heading, t_s);
        }
        assert!(module.att_ref_out_msg.read().omega_rn_n_radps.norm() > 0.0);

        Module::reset(&mut module, &context(2_500_000_000));
        spin_to(&mut module, &heading, 3.0);
        let reference = module.att_ref_out_msg.read();
        assert_eq!(reference.omega_rn_n_radps, Vector3::zeros());
        assert_eq!(reference.domega_rn_n_radps2, Vector3::zeros());
        spin_to(&mut module, &heading, 4.0);
        let omega = module.att_ref_out_msg.read().omega_rn_n_radps;
        assert!((omega - Vector3::new(0.0, 0.0, SPIN_RATE_RADPS)).norm() < 1e-5);
    }

    // Not upstream: Reset's choice of inertial heading source. The heading
    // message wins over the ephemeris, and an ephemeris without a translation
    // message is ignored (upstream logs an error and keeps the previous
    // source, here the default parameter).
    #[test]
    fn resolves_the_inertial_heading_source_like_upstream() {
        let navigation = Output::new(NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.1, 0.2, -0.1),
            vehicle_sun_point_body: unit(Vector3::new(0.2, -0.4, 0.9)),
            ..Default::default()
        });
        let heading_n = unit(Vector3::new(0.3, 0.8, -0.2));
        let inertial_heading = Output::new(InertialHeadingMsg {
            r_hat_xn_n: heading_n,
        });
        let ephemeris = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(-5.0, 1.0, 2.0),
            ..Default::default()
        });
        let translation = Output::new(NavigationTranslationMsg::default());
        let reference_sigma =
            |h_hat_n: Vector3<f64>, connect: &dyn Fn(&mut OneAxisSolarArrayPoint)| {
                let mut module = OneAxisSolarArrayPoint::new(OneAxisSolarArrayPointConfig {
                    a1_hat_b: Vector3::y(),
                    h1_hat_b: Vector3::x(),
                    h_hat_n,
                    ..Default::default()
                });
                navigation.connect_to(&mut module.att_nav_in_msg);
                connect(&mut module);
                module.init();
                module.update(&context(0));
                module.att_ref_out_msg.read().sigma_rn
            };

        let from_parameter = reference_sigma(heading_n, &|_| {});
        let ephemeris_without_translation = reference_sigma(heading_n, &|module| {
            ephemeris.connect_to(&mut module.ephemeris_in_msg);
        });
        assert_eq!(ephemeris_without_translation, from_parameter);

        let message_and_ephemeris = reference_sigma(Vector3::zeros(), &|module| {
            inertial_heading.connect_to(&mut module.inertial_heading_in_msg);
            ephemeris.connect_to(&mut module.ephemeris_in_msg);
            translation.connect_to(&mut module.trans_nav_in_msg);
        });
        assert_eq!(message_and_ephemeris, from_parameter);

        // With the translation message the ephemeris heading is used.
        let from_ephemeris = reference_sigma(heading_n, &|module| {
            ephemeris.connect_to(&mut module.ephemeris_in_msg);
            translation.connect_to(&mut module.trans_nav_in_msg);
        });
        assert!((from_ephemeris - from_parameter).norm() > 1e-3);
    }

    // Not upstream: with a second body heading the module publishes whichever
    // of the two single-heading frames brings a2 closer to the Sun.
    #[test]
    fn picks_the_body_heading_that_brings_a2_closer_to_the_sun() {
        let sigma_bn = Vector3::new(0.1, -0.2, 0.3);
        let sun_b = unit(Vector3::new(0.3, 0.5, -0.8));
        let h1_hat_b = unit(Vector3::new(1.0, 0.2, 0.1));
        let h2_hat_b = unit(Vector3::new(-0.1, 0.3, 1.0));
        let reference_sigma = |h1: Vector3<f64>, h2: Vector3<f64>, a2: Vector3<f64>| {
            let navigation = Output::new(NavigationAttitudeMsg {
                sigma_bn,
                vehicle_sun_point_body: sun_b,
                ..Default::default()
            });
            let mut module = OneAxisSolarArrayPoint::new(OneAxisSolarArrayPointConfig {
                a1_hat_b: unit(Vector3::new(0.2, 1.0, -0.3)),
                h1_hat_b: h1,
                h2_hat_b: h2,
                h_hat_n: unit(Vector3::new(0.4, -0.7, 0.2)),
                a2_hat_b: a2,
                ..Default::default()
            });
            navigation.connect_to(&mut module.att_nav_in_msg);
            module.init();
            module.update(&context(0));
            module.att_ref_out_msg.read().sigma_rn
        };
        // a2 · Sun in the reference frame of `sigma_rn`.
        let a2_sun = |sigma_rn: Vector3<f64>, a2: Vector3<f64>| {
            let sun_r = mrp_to_dcm(sigma_rn) * mrp_to_dcm(sigma_bn).transpose() * sun_b;
            sun_r.dot(&a2)
        };

        let mut winners = Vec::new();
        for a2 in [
            Vector3::x(),
            -Vector3::x(),
            Vector3::y(),
            -Vector3::y(),
            Vector3::z(),
            -Vector3::z(),
        ] {
            let first = reference_sigma(h1_hat_b, Vector3::zeros(), a2);
            let second = reference_sigma(h2_hat_b, Vector3::zeros(), a2);
            let both = reference_sigma(h1_hat_b, h2_hat_b, a2);
            let (dot_1, dot_2) = (a2_sun(first, a2), a2_sun(second, a2));
            let second_wins = dot_2 > dot_1 && (dot_2 - dot_1).abs() > 1e-12;
            let expected = if second_wins { second } else { first };
            assert!(
                (both - expected).norm() < 1e-14,
                "a2 {a2}: {both} != {expected}"
            );
            winners.push(second_wins);
        }
        // Both outcomes occur.
        assert!(
            winners.contains(&true) && winners.contains(&false),
            "{winners:?}"
        );
    }
}
