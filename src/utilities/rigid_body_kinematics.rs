//! Attitude kinematics: conversions among attitude parameterizations, their
//! composition, and their differential kinematics. Port of upstream Basilisk
//! `architecture/utilities/rigidBodyKinematics.c`.
//!
//! Every function ports the upstream algorithm literally (branch logic,
//! operation order, singularity tolerances), so results agree with upstream to
//! machine precision: `dcm_to_ep` is Sheppard's method, `dcm_to_mrp` goes
//! through `dcm_to_ep`, the Euler-angle extraction uses upstream's
//! singularity-robust sequence algorithm, and so on.
//!
//! # Conventions
//!
//! | quantity | Rust type | convention |
//! |---|---|---|
//! | direction cosine matrix (DCM) | `Matrix3<f64>` | `[BN]` maps N-frame components to B-frame components: `v_B = [BN] v_N` |
//! | Euler parameters (EP, unit quaternion) | `Vector4<f64>` | `[β0, β1, β2, β3]`, **scalar first**; deliberately not `nalgebra::Quaternion`, whose storage is vector-first |
//! | modified Rodrigues parameters (MRP) | `Vector3<f64>` | `σ = ê tan(Φ/4)` (dimensionless) |
//! | principal rotation vector (PRV) | `Vector3<f64>` | `ê Φ` \[rad\] |
//! | Euler angles (3-2-1, 1-2-3, 3-1-3) | `Vector3<f64>` | \[rad\], `[θ1, θ2, θ3]` in the order of the set name (3-2-1: yaw, pitch, roll) |
//!
//! * `add_*(first, second)` composes two successive rotations: if `first`
//!   describes B relative to N and `second` describes F relative to B, the
//!   result describes F relative to N (`[FN] = [FB][BN]`).
//! * `sub_*(a, b)` is the attitude of `a` relative to `b`:
//!   `sub_mrp(sigma_bn, sigma_rn) = sigma_br` (`[BR] = [BN][RN]ᵀ`).
//! * Matrices are indexed `m[(row, col)]`, matching upstream `C[row][col]`.
//!
//! # Upstream name map
//!
//! | upstream | Rust |
//! |---|---|
//! | `C2EP`, `C2MRP`, `C2PRV` | [`dcm_to_ep`], [`dcm_to_mrp`], [`dcm_to_prv`] |
//! | `C2Euler321`, `C2Euler123`, `C2Euler313` | [`dcm_to_euler321`], [`dcm_to_euler123`], [`dcm_to_euler313`] |
//! | `EP2C`, `EP2MRP`, `EP2PRV`, `EP2Euler321/123/313` | [`ep_to_dcm`], [`ep_to_mrp`], [`ep_to_prv`], [`ep_to_euler321`] ... |
//! | `MRP2C`, `MRP2EP`, `MRP2PRV`, `MRP2Euler321/123/313` | [`mrp_to_dcm`], [`mrp_to_ep`], [`mrp_to_prv`], [`mrp_to_euler321`] ... |
//! | `PRV2C`, `PRV2EP`, `PRV2MRP`, `PRV2Euler321/123/313` | [`prv_to_dcm`], [`prv_to_ep`], [`prv_to_mrp`], [`prv_to_euler321`] ... |
//! | `Euler3212C/EP/MRP/PRV` (likewise `1232`, `3132`) | [`euler321_to_dcm`], [`euler321_to_ep`], [`euler321_to_mrp`], [`euler321_to_prv`] ... |
//! | `PRV2elem`, `elem2PRV` | [`prv_to_elem`], [`elem_to_prv`] |
//! | `addEP`/`subEP`, `addMRP`/`subMRP`, `addPRV`/`subPRV` | [`add_ep`]/[`sub_ep`], [`add_mrp`]/[`sub_mrp`], [`add_prv`]/[`sub_prv`] |
//! | `addEuler321`/`subEuler321` (likewise `123`, `313`) | [`add_euler321`]/[`sub_euler321`] ... |
//! | `BmatMRP`, `BinvMRP`, `BdotmatMRP` | [`b_mat_mrp`], [`b_inv_mrp`], [`b_dot_mat_mrp`] |
//! | `BmatEP`, `BinvEP`, `BmatPRV`, `BinvPRV` | [`b_mat_ep`], [`b_inv_ep`], [`b_mat_prv`], [`b_inv_prv`] |
//! | `BmatEuler321`/`BinvEuler321` (likewise `123`, `313`) | [`b_mat_euler321`]/[`b_inv_euler321`] ... |
//! | `dMRP`, `dMRP2Omega`, `ddMRP`, `ddMRP2dOmega` | [`d_omega_to_mrp`], [`d_mrp_to_omega`], [`domega_to_dd_mrp`], [`dd_mrp_to_domega`] |
//! | `dEP`, `dPRV`, `dEuler321` (likewise `123`, `313`) | [`d_omega_to_ep`], [`d_omega_to_prv`], [`d_omega_to_euler321`] ... |
//! | `MRPswitch`, `MRPshadow`, `wrapToPi` | [`mrp_switch`], [`mrp_shadow`], [`wrap_to_pi`] |
//! | `Mi`, `Euler1`, `Euler2`, `Euler3`, `tilde` | [`mi`], [`euler1`], [`euler2`], [`euler3`], [`tilde`] |
//!
//! Gibbs vectors and the other nine Euler sequences (1-2-1, 1-3-2, ...) are not
//! ported; the private sequence helpers below cover them, so adding one is a
//! three-line wrapper.

use std::f64::consts::PI;

use nalgebra::{Matrix3, Matrix3x4, Matrix4x3, Vector3, Vector4};

/// Upstream's tolerance on the sine or cosine of the middle Euler angle below
/// which an attitude is treated as singular.
const EULER_SINGULARITY_TOLERANCE: f64 = 1e-14;

/// Upstream's switch to the series expansions in `BmatPRV` / `BinvPRV` \[rad\].
const PRV_SERIES_ANGLE_RAD: f64 = 1e-2;

// ---------------------------------------------------------------------------
// Elementary rotations and the cross-product matrix
// ---------------------------------------------------------------------------

/// Body axis of an elementary rotation, upstream `Mi`'s one-based axis index.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Axis {
    /// First body axis (upstream index 1).
    X,
    /// Second body axis (upstream index 2).
    Y,
    /// Third body axis (upstream index 3).
    Z,
}

impl Axis {
    /// Map upstream's one-based axis index (1, 2, 3) to an axis. Upstream `Mi`
    /// logs `"Mi() error: incorrect axis %d selected."` for any other index and
    /// leaves its output untouched; callers porting that path get `None`.
    pub fn from_index(index: i32) -> Option<Self> {
        match index {
            1 => Some(Self::X),
            2 => Some(Self::Y),
            3 => Some(Self::Z),
            _ => None,
        }
    }
}

/// Passive DCM of a rotation by `angle_rad` about one body axis (upstream `Mi`).
pub fn mi(angle_rad: f64, axis: Axis) -> Matrix3<f64> {
    let c = angle_rad.cos();
    let s = angle_rad.sin();
    match axis {
        Axis::X => Matrix3::new(1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c),
        Axis::Y => Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c),
        Axis::Z => Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0),
    }
}

/// Passive DCM of a rotation by `angle_rad` about the first body axis
/// (upstream `Euler1`; identical to `mi(angle_rad, Axis::X)`).
pub fn euler1(angle_rad: f64) -> Matrix3<f64> {
    mi(angle_rad, Axis::X)
}

/// Passive DCM of a rotation by `angle_rad` about the second body axis
/// (upstream `Euler2`; identical to `mi(angle_rad, Axis::Y)`).
pub fn euler2(angle_rad: f64) -> Matrix3<f64> {
    mi(angle_rad, Axis::Y)
}

/// Passive DCM of a rotation by `angle_rad` about the third body axis
/// (upstream `Euler3`; identical to `mi(angle_rad, Axis::Z)`).
pub fn euler3(angle_rad: f64) -> Matrix3<f64> {
    mi(angle_rad, Axis::Z)
}

/// Skew-symmetric cross-product matrix `[ṽ]`, so that `tilde(a) * b = a × b`
/// (upstream `tilde`; same as `v.cross_matrix()`, kept for porting clarity).
pub fn tilde(v: Vector3<f64>) -> Matrix3<f64> {
    v.cross_matrix()
}

/// Wrap an angle with any number of revolutions to `[-π, π]` (upstream
/// `wrapToPi`). Non-finite inputs return NaN; exact half-turn remainders and
/// zero keep the sign of the input.
pub fn wrap_to_pi(angle_rad: f64) -> f64 {
    if !angle_rad.is_finite() {
        return f64::NAN;
    }
    let two_pi = 2.0 * PI;
    let wrapped = angle_rad % two_pi;
    if wrapped > PI {
        wrapped - two_pi
    } else if wrapped < -PI {
        wrapped + two_pi
    } else {
        wrapped
    }
}

// ---------------------------------------------------------------------------
// Euler parameters
// ---------------------------------------------------------------------------

/// DCM of Euler parameters `[β0, β1, β2, β3]` (upstream `EP2C`).
pub fn ep_to_dcm(ep: Vector4<f64>) -> Matrix3<f64> {
    let [q0, q1, q2, q3] = [ep[0], ep[1], ep[2], ep[3]];
    Matrix3::new(
        q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
        2.0 * (q1 * q2 + q0 * q3),
        2.0 * (q1 * q3 - q0 * q2),
        2.0 * (q1 * q2 - q0 * q3),
        q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
        2.0 * (q2 * q3 + q0 * q1),
        2.0 * (q1 * q3 + q0 * q2),
        2.0 * (q2 * q3 - q0 * q1),
        q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
    )
}

/// Euler parameters of a DCM with `β0 >= 0` (upstream `C2EP`, Sheppard's
/// method: the largest of the four squared parameters is extracted first).
pub fn dcm_to_ep(c: Matrix3<f64>) -> Vector4<f64> {
    let trace = c[(0, 0)] + c[(1, 1)] + c[(2, 2)];
    let squared = [
        (1.0 + trace) / 4.0,
        (1.0 + 2.0 * c[(0, 0)] - trace) / 4.0,
        (1.0 + 2.0 * c[(1, 1)] - trace) / 4.0,
        (1.0 + 2.0 * c[(2, 2)] - trace) / 4.0,
    ];
    // Strict comparison: ties keep the lowest index, as upstream does.
    let largest = (1..4).fold(
        0,
        |best, j| if squared[j] > squared[best] { j } else { best },
    );

    match largest {
        0 => {
            let b0 = squared[0].sqrt();
            Vector4::new(
                b0,
                (c[(1, 2)] - c[(2, 1)]) / 4.0 / b0,
                (c[(2, 0)] - c[(0, 2)]) / 4.0 / b0,
                (c[(0, 1)] - c[(1, 0)]) / 4.0 / b0,
            )
        }
        1 => {
            let mut b1 = squared[1].sqrt();
            let mut b0 = (c[(1, 2)] - c[(2, 1)]) / 4.0 / b1;
            if b0 < 0.0 {
                b1 = -b1;
                b0 = -b0;
            }
            Vector4::new(
                b0,
                b1,
                (c[(0, 1)] + c[(1, 0)]) / 4.0 / b1,
                (c[(2, 0)] + c[(0, 2)]) / 4.0 / b1,
            )
        }
        2 => {
            let mut b2 = squared[2].sqrt();
            let mut b0 = (c[(2, 0)] - c[(0, 2)]) / 4.0 / b2;
            if b0 < 0.0 {
                b2 = -b2;
                b0 = -b0;
            }
            Vector4::new(
                b0,
                (c[(0, 1)] + c[(1, 0)]) / 4.0 / b2,
                b2,
                (c[(1, 2)] + c[(2, 1)]) / 4.0 / b2,
            )
        }
        _ => {
            let mut b3 = squared[3].sqrt();
            let mut b0 = (c[(0, 1)] - c[(1, 0)]) / 4.0 / b3;
            if b0 < 0.0 {
                b3 = -b3;
                b0 = -b0;
            }
            Vector4::new(
                b0,
                (c[(2, 0)] + c[(0, 2)]) / 4.0 / b3,
                (c[(1, 2)] + c[(2, 1)]) / 4.0 / b3,
                b3,
            )
        }
    }
}

/// MRP of Euler parameters (upstream `EP2MRP`). A negative scalar part maps
/// through `-ep`, so the result always has `|σ| <= 1`.
pub fn ep_to_mrp(ep: Vector4<f64>) -> Vector3<f64> {
    if ep[0] >= 0.0 {
        Vector3::new(
            ep[1] / (1.0 + ep[0]),
            ep[2] / (1.0 + ep[0]),
            ep[3] / (1.0 + ep[0]),
        )
    } else {
        Vector3::new(
            -ep[1] / (1.0 - ep[0]),
            -ep[2] / (1.0 - ep[0]),
            -ep[3] / (1.0 - ep[0]),
        )
    }
}

/// PRV \[rad\] of Euler parameters (upstream `EP2PRV`). Both identity
/// quaternions map to zero; a negative scalar part keeps the branch with a
/// rotation angle greater than π.
pub fn ep_to_prv(ep: Vector4<f64>) -> Vector3<f64> {
    let sp = ep[1].hypot(ep[2].hypot(ep[3]));
    if sp == 0.0 {
        return Vector3::zeros();
    }
    let p = 2.0 * sp.atan2(ep[0]);
    Vector3::new(ep[1] / sp * p, ep[2] / sp * p, ep[3] / sp * p)
}

/// 3-2-1 Euler angles \[rad\] of Euler parameters (upstream `EP2Euler321`,
/// via the DCM).
pub fn ep_to_euler321(ep: Vector4<f64>) -> Vector3<f64> {
    dcm_to_euler321(ep_to_dcm(ep))
}

/// 1-2-3 Euler angles \[rad\] of Euler parameters (upstream `EP2Euler123`,
/// via the DCM).
pub fn ep_to_euler123(ep: Vector4<f64>) -> Vector3<f64> {
    dcm_to_euler123(ep_to_dcm(ep))
}

/// 3-1-3 Euler angles \[rad\] of Euler parameters (upstream `EP2Euler313`).
/// Away from singularities the outer angles come straight from the quaternion
/// and are not wrapped, so they may lie outside `[-π, π]`.
pub fn ep_to_euler313(ep: Vector4<f64>) -> Vector3<f64> {
    ep_to_euler_repeated(ep, EulerSequence::S313)
}

/// Euler parameters of performing rotation `first` and then `second`
/// (upstream `addEP`): `[C(result)] = [C(second)][C(first)]`.
pub fn add_ep(first: Vector4<f64>, second: Vector4<f64>) -> Vector4<f64> {
    let (b1, b2) = (first, second);
    Vector4::new(
        b2[0] * b1[0] - b2[1] * b1[1] - b2[2] * b1[2] - b2[3] * b1[3],
        b2[1] * b1[0] + b2[0] * b1[1] + b2[3] * b1[2] - b2[2] * b1[3],
        b2[2] * b1[0] - b2[3] * b1[1] + b2[0] * b1[2] + b2[1] * b1[3],
        b2[3] * b1[0] + b2[2] * b1[1] - b2[1] * b1[2] + b2[0] * b1[3],
    )
}

/// Euler parameters of `attitude` relative to `reference` (upstream `subEP`):
/// `[C(result)] = [C(attitude)][C(reference)]ᵀ`.
pub fn sub_ep(attitude: Vector4<f64>, reference: Vector4<f64>) -> Vector4<f64> {
    let (b1, b2) = (attitude, reference);
    Vector4::new(
        b2[0] * b1[0] + b2[1] * b1[1] + b2[2] * b1[2] + b2[3] * b1[3],
        -b2[1] * b1[0] + b2[0] * b1[1] + b2[3] * b1[2] - b2[2] * b1[3],
        -b2[2] * b1[0] - b2[3] * b1[1] + b2[0] * b1[2] + b2[1] * b1[3],
        -b2[3] * b1[0] + b2[2] * b1[1] - b2[1] * b1[2] + b2[0] * b1[3],
    )
}

/// 4×3 matrix `[B(β)]` of the Euler-parameter kinematics
/// `β̇ = ½ [B(β)] ω` (upstream `BmatEP`).
pub fn b_mat_ep(ep: Vector4<f64>) -> Matrix4x3<f64> {
    let q = ep;
    Matrix4x3::new(
        -q[1], -q[2], -q[3], //
        q[0], -q[3], q[2], //
        q[3], q[0], -q[1], //
        -q[2], q[1], q[0],
    )
}

/// 3×4 matrix `[B(β)]⁻¹ = [B(β)]ᵀ` of the inverse Euler-parameter kinematics
/// `ω = 2 [B(β)]⁻¹ β̇` (upstream `BinvEP`).
pub fn b_inv_ep(ep: Vector4<f64>) -> Matrix3x4<f64> {
    let q = ep;
    Matrix3x4::new(
        -q[1], q[0], q[3], -q[2], //
        -q[2], -q[3], q[0], q[1], //
        -q[3], q[2], -q[1], q[0],
    )
}

/// Euler-parameter rate `β̇` \[1/s\] for body rate `omega` \[rad/s\]
/// (upstream `dEP`).
pub fn d_omega_to_ep(ep: Vector4<f64>, omega: Vector3<f64>) -> Vector4<f64> {
    (b_mat_ep(ep) * omega) * 0.5
}

// ---------------------------------------------------------------------------
// Modified Rodrigues parameters
// ---------------------------------------------------------------------------

/// DCM of an MRP set (upstream `MRP2C`).
pub fn mrp_to_dcm(sigma: Vector3<f64>) -> Matrix3<f64> {
    let [q1, q2, q3] = [sigma[0], sigma[1], sigma[2]];
    let d1 = sigma.norm_squared();
    let s = 1.0 - d1;
    let d = (1.0 + d1) * (1.0 + d1);
    Matrix3::new(
        4.0 * (2.0 * q1 * q1 - d1) + s * s,
        8.0 * q1 * q2 + 4.0 * q3 * s,
        8.0 * q1 * q3 - 4.0 * q2 * s,
        8.0 * q2 * q1 - 4.0 * q3 * s,
        4.0 * (2.0 * q2 * q2 - d1) + s * s,
        8.0 * q2 * q3 + 4.0 * q1 * s,
        8.0 * q3 * q1 + 4.0 * q2 * s,
        8.0 * q3 * q2 - 4.0 * q1 * s,
        4.0 * (2.0 * q3 * q3 - d1) + s * s,
    ) * (1.0 / d)
}

/// MRP set with `|σ| <= 1` of a DCM (upstream `C2MRP`, via [`dcm_to_ep`]).
pub fn dcm_to_mrp(c: Matrix3<f64>) -> Vector3<f64> {
    let b = dcm_to_ep(c);
    Vector3::new(
        b[1] / (1.0 + b[0]),
        b[2] / (1.0 + b[0]),
        b[3] / (1.0 + b[0]),
    )
}

/// Euler parameters of an MRP set (upstream `MRP2EP`). A set with `|σ| > 1`
/// yields a negative scalar part.
pub fn mrp_to_ep(sigma: Vector3<f64>) -> Vector4<f64> {
    let ps = 1.0 + sigma.norm_squared();
    Vector4::new(
        (1.0 - sigma.norm_squared()) / ps,
        2.0 * sigma[0] / ps,
        2.0 * sigma[1] / ps,
        2.0 * sigma[2] / ps,
    )
}

/// PRV \[rad\] of an MRP set (upstream `MRP2PRV`); only an exactly zero set
/// maps to zero.
pub fn mrp_to_prv(sigma: Vector3<f64>) -> Vector3<f64> {
    let tp = sigma[0].hypot(sigma[1].hypot(sigma[2]));
    if tp == 0.0 {
        return Vector3::zeros();
    }
    let p = 4.0 * tp.atan();
    Vector3::new(sigma[0] / tp * p, sigma[1] / tp * p, sigma[2] / tp * p)
}

/// 3-2-1 Euler angles \[rad\] of an MRP set (upstream `MRP2Euler321`).
pub fn mrp_to_euler321(sigma: Vector3<f64>) -> Vector3<f64> {
    ep_to_euler321(mrp_to_ep(sigma))
}

/// 1-2-3 Euler angles \[rad\] of an MRP set (upstream `MRP2Euler123`).
pub fn mrp_to_euler123(sigma: Vector3<f64>) -> Vector3<f64> {
    ep_to_euler123(mrp_to_ep(sigma))
}

/// 3-1-3 Euler angles \[rad\] of an MRP set (upstream `MRP2Euler313`).
pub fn mrp_to_euler313(sigma: Vector3<f64>) -> Vector3<f64> {
    ep_to_euler313(mrp_to_ep(sigma))
}

/// MRP set (`|σ| <= 1`) of performing rotation `first` and then `second`
/// (upstream `addMRP`): `[C(result)] = [C(second)][C(first)]`. Exactly opposite
/// inputs return zero.
pub fn add_mrp(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    compose_mrp(first, second, 1.0)
}

/// MRP set (`|σ| <= 1`) of `attitude` relative to `reference` (upstream
/// `subMRP`): `sub_mrp(sigma_bn, sigma_rn) = sigma_br`. Exactly equal inputs
/// return zero.
pub fn sub_mrp(attitude: Vector3<f64>, reference: Vector3<f64>) -> Vector3<f64> {
    compose_mrp(attitude, reference, -1.0)
}

/// Upstream `composeMRP`: compose through quaternions and return the bounded
/// set; `sign = -1` composes with the inverse of `second`.
fn compose_mrp(first: Vector3<f64>, second: Vector3<f64>, sign: f64) -> Vector3<f64> {
    if first == -sign * second {
        return Vector3::zeros();
    }
    let first_ep = mrp_to_ep(first);
    let second_ep = mrp_to_ep(second);
    let first_vector = ep_vector_part(first_ep);
    let second_vector = ep_vector_part(second_ep);
    let cross = first_vector.cross(&second_vector);
    let scalar = first_ep[0] * second_ep[0] - sign * first_vector.dot(&second_vector);
    // Upstream cancels the linear terms before adding the smaller cross product.
    let vector = (second_ep[0] * first_vector + sign * first_ep[0] * second_vector) + cross;
    ep_to_mrp(Vector4::new(scalar, vector[0], vector[1], vector[2]))
}

/// Switch to the shadow MRP set when `|σ| > threshold` (upstream `MRPswitch`;
/// the threshold is compared by magnitude, `|σ|² > threshold²`).
pub fn mrp_switch(sigma: Vector3<f64>, threshold: f64) -> Vector3<f64> {
    let q2 = sigma.norm_squared();
    if q2 > threshold * threshold {
        sigma * (-1.0 / q2)
    } else {
        sigma
    }
}

/// Shadow set `-σ/|σ|²` of a nonzero MRP set (upstream `MRPshadow`).
pub fn mrp_shadow(sigma: Vector3<f64>) -> Vector3<f64> {
    sigma * (-1.0 / sigma.norm_squared())
}

/// Matrix `[B(σ)]` of the MRP kinematics `σ̇ = ¼ [B(σ)] ω` (upstream `BmatMRP`).
pub fn b_mat_mrp(sigma: Vector3<f64>) -> Matrix3<f64> {
    let q = sigma;
    let s2 = q.norm_squared();
    Matrix3::new(
        1.0 - s2 + 2.0 * q[0] * q[0],
        2.0 * (q[0] * q[1] - q[2]),
        2.0 * (q[0] * q[2] + q[1]),
        2.0 * (q[1] * q[0] + q[2]),
        1.0 - s2 + 2.0 * q[1] * q[1],
        2.0 * (q[1] * q[2] - q[0]),
        2.0 * (q[2] * q[0] - q[1]),
        2.0 * (q[2] * q[1] + q[0]),
        1.0 - s2 + 2.0 * q[2] * q[2],
    )
}

/// Inverse `[B(σ)]⁻¹` of the MRP kinematics matrix, `ω = 4 [B(σ)]⁻¹ σ̇`
/// (upstream `BinvMRP`).
pub fn b_inv_mrp(sigma: Vector3<f64>) -> Matrix3<f64> {
    let q = sigma;
    let s2 = q.norm_squared();
    Matrix3::new(
        1.0 - s2 + 2.0 * q[0] * q[0],
        2.0 * (q[0] * q[1] + q[2]),
        2.0 * (q[0] * q[2] - q[1]),
        2.0 * (q[1] * q[0] - q[2]),
        1.0 - s2 + 2.0 * q[1] * q[1],
        2.0 * (q[1] * q[2] + q[0]),
        2.0 * (q[2] * q[0] + q[1]),
        2.0 * (q[2] * q[1] - q[0]),
        1.0 - s2 + 2.0 * q[2] * q[2],
    ) * (1.0 / (1.0 + s2) / (1.0 + s2))
}

/// Time derivative `[Ḃ(σ, σ̇)]` \[1/s\] of the MRP kinematics matrix
/// (upstream `BdotmatMRP`).
pub fn b_dot_mat_mrp(sigma: Vector3<f64>, sigma_dot: Vector3<f64>) -> Matrix3<f64> {
    let (q, dq) = (sigma, sigma_dot);
    let s = -2.0 * q.dot(&dq);
    Matrix3::new(
        s + 4.0 * (q[0] * dq[0]),
        2.0 * (-dq[2] + q[0] * dq[1] + dq[0] * q[1]),
        2.0 * (dq[1] + q[0] * dq[2] + dq[0] * q[2]),
        2.0 * (dq[2] + q[0] * dq[1] + dq[0] * q[1]),
        s + 4.0 * (q[1] * dq[1]),
        2.0 * (-dq[0] + q[1] * dq[2] + dq[1] * q[2]),
        2.0 * (-dq[1] + q[0] * dq[2] + dq[0] * q[2]),
        2.0 * (dq[0] + q[1] * dq[2] + dq[1] * q[2]),
        s + 4.0 * (q[2] * dq[2]),
    )
}

/// MRP rate `σ̇` \[1/s\] for body rate `omega` \[rad/s\] (upstream `dMRP`).
pub fn d_omega_to_mrp(sigma: Vector3<f64>, omega: Vector3<f64>) -> Vector3<f64> {
    (b_mat_mrp(sigma) * omega) * 0.25
}

/// Body rate `ω` \[rad/s\] for MRP rate `sigma_dot` \[1/s\] (upstream
/// `dMRP2Omega`).
pub fn d_mrp_to_omega(sigma: Vector3<f64>, sigma_dot: Vector3<f64>) -> Vector3<f64> {
    (b_inv_mrp(sigma) * sigma_dot) * 4.0
}

/// MRP acceleration `σ̈` \[1/s²\] for body rate `omega` \[rad/s\] and body
/// angular acceleration `omega_dot` \[rad/s²\] (upstream `ddMRP`).
pub fn domega_to_dd_mrp(
    sigma: Vector3<f64>,
    sigma_dot: Vector3<f64>,
    omega: Vector3<f64>,
    omega_dot: Vector3<f64>,
) -> Vector3<f64> {
    let s1 = b_mat_mrp(sigma) * omega_dot;
    let s2 = b_dot_mat_mrp(sigma, sigma_dot) * omega;
    (s1 + s2) * 0.25
}

/// Body angular acceleration `ω̇` \[rad/s²\] for MRP rate `sigma_dot` \[1/s\]
/// and MRP acceleration `sigma_ddot` \[1/s²\] (upstream `ddMRP2dOmega`).
pub fn dd_mrp_to_domega(
    sigma: Vector3<f64>,
    sigma_dot: Vector3<f64>,
    sigma_ddot: Vector3<f64>,
) -> Vector3<f64> {
    let b_inv = b_inv_mrp(sigma);
    let b_dot = b_dot_mat_mrp(sigma, sigma_dot);
    let s1 = b_inv * sigma_dot;
    let s2 = b_dot * s1;
    (b_inv * (sigma_ddot - s2)) * 4.0
}

// ---------------------------------------------------------------------------
// Principal rotation vectors
// ---------------------------------------------------------------------------

/// Principal rotation angle \[rad\] and unit axis of a PRV (upstream
/// `PRV2elem`, which packs them as `[Φ, ê]`). Only an exactly zero vector
/// returns a zero angle and a zero axis.
pub fn prv_to_elem(prv: Vector3<f64>) -> (f64, Vector3<f64>) {
    let angle = prv[0].hypot(prv[1].hypot(prv[2]));
    if angle == 0.0 {
        (0.0, Vector3::zeros())
    } else {
        (angle, prv / angle)
    }
}

/// PRV \[rad\] of a principal rotation angle \[rad\] and axis (upstream
/// `elem2PRV`).
pub fn elem_to_prv(angle_rad: f64, axis: Vector3<f64>) -> Vector3<f64> {
    axis * angle_rad
}

/// DCM of a PRV \[rad\] (upstream `PRV2C`); an exactly zero vector gives the
/// identity.
pub fn prv_to_dcm(prv: Vector3<f64>) -> Matrix3<f64> {
    let q0 = prv[0].hypot(prv[1].hypot(prv[2]));
    if q0 == 0.0 {
        return Matrix3::identity();
    }
    let [q1, q2, q3] = [prv[0] / q0, prv[1] / q0, prv[2] / q0];
    let cp = q0.cos();
    let sp = q0.sin();
    let d1 = 2.0 * (q0 / 2.0).sin() * (q0 / 2.0).sin();
    Matrix3::new(
        q1 * q1 * d1 + cp,
        q1 * q2 * d1 + q3 * sp,
        q1 * q3 * d1 - q2 * sp,
        q2 * q1 * d1 - q3 * sp,
        q2 * q2 * d1 + cp,
        q2 * q3 * d1 + q1 * sp,
        q3 * q1 * d1 + q2 * sp,
        q3 * q2 * d1 - q1 * sp,
        q3 * q3 * d1 + cp,
    )
}

/// PRV \[rad\] with angle in `[0, π]` of a DCM (upstream `C2PRV`, via
/// [`dcm_to_ep`]).
pub fn dcm_to_prv(c: Matrix3<f64>) -> Vector3<f64> {
    ep_to_prv(dcm_to_ep(c))
}

/// Euler parameters of a PRV \[rad\] (upstream `PRV2EP`).
pub fn prv_to_ep(prv: Vector3<f64>) -> Vector4<f64> {
    let (angle, axis) = prv_to_elem(prv);
    let sp = (angle / 2.0).sin();
    Vector4::new(
        (angle / 2.0).cos(),
        axis[0] * sp,
        axis[1] * sp,
        axis[2] * sp,
    )
}

/// MRP set of a PRV \[rad\] (upstream `PRV2MRP`); angles beyond π give
/// `|σ| > 1`.
pub fn prv_to_mrp(prv: Vector3<f64>) -> Vector3<f64> {
    let (angle, axis) = prv_to_elem(prv);
    axis * (angle / 4.0).tan()
}

/// 3-2-1 Euler angles \[rad\] of a PRV \[rad\] (upstream `PRV2Euler321`).
pub fn prv_to_euler321(prv: Vector3<f64>) -> Vector3<f64> {
    ep_to_euler321(prv_to_ep(prv))
}

/// 1-2-3 Euler angles \[rad\] of a PRV \[rad\] (upstream `PRV2Euler123`).
pub fn prv_to_euler123(prv: Vector3<f64>) -> Vector3<f64> {
    ep_to_euler123(prv_to_ep(prv))
}

/// 3-1-3 Euler angles \[rad\] of a PRV \[rad\] (upstream `PRV2Euler313`).
pub fn prv_to_euler313(prv: Vector3<f64>) -> Vector3<f64> {
    ep_to_euler313(prv_to_ep(prv))
}

/// PRV \[rad\] of performing rotation `first` and then `second` (upstream
/// `addPRV`). Exactly opposite inputs return zero; otherwise the composed
/// quaternion's angle branch is kept (the angle may exceed π).
pub fn add_prv(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    if first == -second {
        return Vector3::zeros();
    }
    ep_to_prv(add_ep(prv_to_ep(first), prv_to_ep(second)))
}

/// PRV \[rad\] of `attitude` relative to `reference` (upstream `subPRV`).
/// Exactly equal inputs return zero; otherwise the relative quaternion's angle
/// branch is kept.
pub fn sub_prv(attitude: Vector3<f64>, reference: Vector3<f64>) -> Vector3<f64> {
    if attitude == reference {
        return Vector3::zeros();
    }
    ep_to_prv(sub_ep(prv_to_ep(attitude), prv_to_ep(reference)))
}

/// Matrix `[B(γ)]` of the PRV kinematics `γ̇ = [B(γ)] ω` (upstream `BmatPRV`);
/// angles below 0.01 rad use upstream's series expansion.
pub fn b_mat_prv(prv: Vector3<f64>) -> Matrix3<f64> {
    let q = prv;
    let p = q.norm();
    let c = if p < PRV_SERIES_ANGLE_RAD {
        let p2 = p * p;
        1.0 / 12.0 + p2 / 720.0 + p2 * p2 / 30240.0 + p2 * p2 * p2 / 1209600.0
    } else {
        (1.0 - p / 2.0 / (p / 2.0).tan()) / (p * p)
    };
    Matrix3::new(
        1.0 - c * (q[1] * q[1] + q[2] * q[2]),
        -q[2] / 2.0 + c * (q[0] * q[1]),
        q[1] / 2.0 + c * (q[0] * q[2]),
        q[2] / 2.0 + c * (q[0] * q[1]),
        1.0 - c * (q[0] * q[0] + q[2] * q[2]),
        -q[0] / 2.0 + c * (q[1] * q[2]),
        -q[1] / 2.0 + c * (q[0] * q[2]),
        q[0] / 2.0 + c * (q[1] * q[2]),
        1.0 - c * (q[0] * q[0] + q[1] * q[1]),
    )
}

/// Inverse `[B(γ)]⁻¹` of the PRV kinematics matrix, `ω = [B(γ)]⁻¹ γ̇`
/// (upstream `BinvPRV`); angles below 0.01 rad use upstream's series
/// expansions.
pub fn b_inv_prv(prv: Vector3<f64>) -> Matrix3<f64> {
    let q = prv;
    let p = q.norm();
    let (c1, c2) = if p < PRV_SERIES_ANGLE_RAD {
        let p2 = p * p;
        (
            0.5 - p2 / 24.0 + p2 * p2 / 720.0 - p2 * p2 * p2 / 40320.0,
            1.0 / 6.0 - p2 / 120.0 + p2 * p2 / 5040.0 - p2 * p2 * p2 / 362880.0,
        )
    } else {
        let half_sinc = (p / 2.0).sin() / p;
        (2.0 * half_sinc * half_sinc, (p - p.sin()) / (p * p * p))
    };
    Matrix3::new(
        1.0 - c2 * (q[1] * q[1] + q[2] * q[2]),
        c1 * q[2] + c2 * q[0] * q[1],
        -c1 * q[1] + c2 * q[0] * q[2],
        -c1 * q[2] + c2 * q[0] * q[1],
        1.0 - c2 * (q[0] * q[0] + q[2] * q[2]),
        c1 * q[0] + c2 * q[1] * q[2],
        c1 * q[1] + c2 * q[2] * q[0],
        -c1 * q[0] + c2 * q[2] * q[1],
        1.0 - c2 * (q[0] * q[0] + q[1] * q[1]),
    )
}

/// PRV rate `γ̇` \[rad/s\] for body rate `omega` \[rad/s\] (upstream `dPRV`).
pub fn d_omega_to_prv(prv: Vector3<f64>, omega: Vector3<f64>) -> Vector3<f64> {
    b_mat_prv(prv) * omega
}

// ---------------------------------------------------------------------------
// Euler angles: 3-2-1
// ---------------------------------------------------------------------------

/// DCM of 3-2-1 Euler angles \[rad\] (upstream `Euler3212C`).
pub fn euler321_to_dcm(angles: Vector3<f64>) -> Matrix3<f64> {
    let (st1, ct1) = (angles[0].sin(), angles[0].cos());
    let (st2, ct2) = (angles[1].sin(), angles[1].cos());
    let (st3, ct3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(
        ct2 * ct1,
        ct2 * st1,
        -st2,
        st3 * st2 * ct1 - ct3 * st1,
        st3 * st2 * st1 + ct3 * ct1,
        st3 * ct2,
        ct3 * st2 * ct1 + st3 * st1,
        ct3 * st2 * st1 - st3 * ct1,
        ct3 * ct2,
    )
}

/// 3-2-1 Euler angles \[rad\] of a DCM (upstream `C2Euler321`), with the middle
/// angle in `[-π/2, π/2]`. At a singularity the third angle is zero and the
/// first carries the observable combined rotation.
pub fn dcm_to_euler321(c: Matrix3<f64>) -> Vector3<f64> {
    dcm_to_euler_sequence(&c, EulerSequence::S321)
}

/// Euler parameters of 3-2-1 Euler angles \[rad\] (upstream `Euler3212EP`).
pub fn euler321_to_ep(angles: Vector3<f64>) -> Vector4<f64> {
    let (c1, s1) = ((angles[0] / 2.0).cos(), (angles[0] / 2.0).sin());
    let (c2, s2) = ((angles[1] / 2.0).cos(), (angles[1] / 2.0).sin());
    let (c3, s3) = ((angles[2] / 2.0).cos(), (angles[2] / 2.0).sin());
    Vector4::new(
        c1 * c2 * c3 + s1 * s2 * s3,
        c1 * c2 * s3 - s1 * s2 * c3,
        c1 * s2 * c3 + s1 * c2 * s3,
        s1 * c2 * c3 - c1 * s2 * s3,
    )
}

/// MRP set (`|σ| <= 1`) of 3-2-1 Euler angles \[rad\] (upstream `Euler3212MRP`).
pub fn euler321_to_mrp(angles: Vector3<f64>) -> Vector3<f64> {
    ep_to_mrp(euler321_to_ep(angles))
}

/// PRV \[rad\] of 3-2-1 Euler angles \[rad\] (upstream `Euler3212PRV`).
pub fn euler321_to_prv(angles: Vector3<f64>) -> Vector3<f64> {
    ep_to_prv(euler321_to_ep(angles))
}

/// 3-2-1 Euler angles \[rad\] of performing rotation `first` and then `second`
/// (upstream `addEuler321`).
pub fn add_euler321(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    dcm_to_euler321(euler321_to_dcm(second) * euler321_to_dcm(first))
}

/// 3-2-1 Euler angles \[rad\] of `attitude` relative to `reference` (upstream
/// `subEuler321`).
pub fn sub_euler321(attitude: Vector3<f64>, reference: Vector3<f64>) -> Vector3<f64> {
    dcm_to_euler321(euler321_to_dcm(attitude) * euler321_to_dcm(reference).transpose())
}

/// Matrix `[B(θ)]` of the 3-2-1 kinematics `θ̇ = [B(θ)] ω` (upstream
/// `BmatEuler321`); singular at `cos θ2 = 0`.
pub fn b_mat_euler321(angles: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = (angles[1].sin(), angles[1].cos());
    let (s3, c3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(0.0, s3, c3, 0.0, c2 * c3, -c2 * s3, c2, s2 * s3, s2 * c3) * (1.0 / c2)
}

/// Inverse `[B(θ)]⁻¹` of the 3-2-1 kinematics matrix, `ω = [B(θ)]⁻¹ θ̇`
/// (upstream `BinvEuler321`).
pub fn b_inv_euler321(angles: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = (angles[1].sin(), angles[1].cos());
    let (s3, c3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(-s2, 0.0, 1.0, c2 * s3, c3, 0.0, c2 * c3, -s3, 0.0)
}

/// 3-2-1 Euler angle rates \[rad/s\] for body rate `omega` \[rad/s\] (upstream
/// `dEuler321`).
pub fn d_omega_to_euler321(angles: Vector3<f64>, omega: Vector3<f64>) -> Vector3<f64> {
    b_mat_euler321(angles) * omega
}

// ---------------------------------------------------------------------------
// Euler angles: 1-2-3
// ---------------------------------------------------------------------------

/// DCM of 1-2-3 Euler angles \[rad\] (upstream `Euler1232C`).
pub fn euler123_to_dcm(angles: Vector3<f64>) -> Matrix3<f64> {
    let (st1, ct1) = (angles[0].sin(), angles[0].cos());
    let (st2, ct2) = (angles[1].sin(), angles[1].cos());
    let (st3, ct3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(
        ct2 * ct3,
        ct3 * st1 * st2 + ct1 * st3,
        st1 * st3 - ct1 * ct3 * st2,
        -ct2 * st3,
        ct1 * ct3 - st1 * st2 * st3,
        ct3 * st1 + ct1 * st2 * st3,
        st2,
        -ct2 * st1,
        ct1 * ct2,
    )
}

/// 1-2-3 Euler angles \[rad\] of a DCM (upstream `C2Euler123`), with the middle
/// angle in `[-π/2, π/2]`. At a singularity the third angle is zero and the
/// first carries the observable combined rotation.
pub fn dcm_to_euler123(c: Matrix3<f64>) -> Vector3<f64> {
    dcm_to_euler_sequence(&c, EulerSequence::S123)
}

/// Euler parameters of 1-2-3 Euler angles \[rad\] (upstream `Euler1232EP`).
pub fn euler123_to_ep(angles: Vector3<f64>) -> Vector4<f64> {
    let (c1, s1) = ((angles[0] / 2.0).cos(), (angles[0] / 2.0).sin());
    let (c2, s2) = ((angles[1] / 2.0).cos(), (angles[1] / 2.0).sin());
    let (c3, s3) = ((angles[2] / 2.0).cos(), (angles[2] / 2.0).sin());
    Vector4::new(
        c1 * c2 * c3 - s1 * s2 * s3,
        s1 * c2 * c3 + c1 * s2 * s3,
        c1 * s2 * c3 - s1 * c2 * s3,
        c1 * c2 * s3 + s1 * s2 * c3,
    )
}

/// MRP set (`|σ| <= 1`) of 1-2-3 Euler angles \[rad\] (upstream `Euler1232MRP`).
pub fn euler123_to_mrp(angles: Vector3<f64>) -> Vector3<f64> {
    ep_to_mrp(euler123_to_ep(angles))
}

/// PRV \[rad\] of 1-2-3 Euler angles \[rad\] (upstream `Euler1232PRV`).
pub fn euler123_to_prv(angles: Vector3<f64>) -> Vector3<f64> {
    ep_to_prv(euler123_to_ep(angles))
}

/// 1-2-3 Euler angles \[rad\] of performing rotation `first` and then `second`
/// (upstream `addEuler123`).
pub fn add_euler123(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    dcm_to_euler123(euler123_to_dcm(second) * euler123_to_dcm(first))
}

/// 1-2-3 Euler angles \[rad\] of `attitude` relative to `reference` (upstream
/// `subEuler123`).
pub fn sub_euler123(attitude: Vector3<f64>, reference: Vector3<f64>) -> Vector3<f64> {
    dcm_to_euler123(euler123_to_dcm(attitude) * euler123_to_dcm(reference).transpose())
}

/// Matrix `[B(θ)]` of the 1-2-3 kinematics `θ̇ = [B(θ)] ω` (upstream
/// `BmatEuler123`); singular at `cos θ2 = 0`.
pub fn b_mat_euler123(angles: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = (angles[1].sin(), angles[1].cos());
    let (s3, c3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(c3, -s3, 0.0, c2 * s3, c2 * c3, 0.0, -s2 * c3, s2 * s3, c2) * (1.0 / c2)
}

/// Inverse `[B(θ)]⁻¹` of the 1-2-3 kinematics matrix, `ω = [B(θ)]⁻¹ θ̇`
/// (upstream `BinvEuler123`).
pub fn b_inv_euler123(angles: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = (angles[1].sin(), angles[1].cos());
    let (s3, c3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(c2 * c3, s3, 0.0, -c2 * s3, c3, 0.0, s2, 0.0, 1.0)
}

/// 1-2-3 Euler angle rates \[rad/s\] for body rate `omega` \[rad/s\] (upstream
/// `dEuler123`).
pub fn d_omega_to_euler123(angles: Vector3<f64>, omega: Vector3<f64>) -> Vector3<f64> {
    b_mat_euler123(angles) * omega
}

// ---------------------------------------------------------------------------
// Euler angles: 3-1-3
// ---------------------------------------------------------------------------

/// DCM of 3-1-3 Euler angles \[rad\] (upstream `Euler3132C`).
pub fn euler313_to_dcm(angles: Vector3<f64>) -> Matrix3<f64> {
    let (st1, ct1) = (angles[0].sin(), angles[0].cos());
    let (st2, ct2) = (angles[1].sin(), angles[1].cos());
    let (st3, ct3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(
        ct3 * ct1 - st3 * ct2 * st1,
        ct3 * st1 + st3 * ct2 * ct1,
        st3 * st2,
        -st3 * ct1 - ct3 * ct2 * st1,
        -st3 * st1 + ct3 * ct2 * ct1,
        ct3 * st2,
        st2 * st1,
        -st2 * ct1,
        ct2,
    )
}

/// 3-1-3 Euler angles \[rad\] of a DCM (upstream `C2Euler313`), with the middle
/// angle in `[0, π]`. At a singularity the third angle is zero and the first
/// carries the observable combined rotation.
pub fn dcm_to_euler313(c: Matrix3<f64>) -> Vector3<f64> {
    dcm_to_euler_sequence(&c, EulerSequence::S313)
}

/// Euler parameters of 3-1-3 Euler angles \[rad\] (upstream `Euler3132EP`).
pub fn euler313_to_ep(angles: Vector3<f64>) -> Vector4<f64> {
    let e1 = angles[0] / 2.0;
    let e2 = angles[1] / 2.0;
    let e3 = angles[2] / 2.0;
    Vector4::new(
        e2.cos() * (e1 + e3).cos(),
        e2.sin() * (e1 - e3).cos(),
        e2.sin() * (e1 - e3).sin(),
        e2.cos() * (e1 + e3).sin(),
    )
}

/// MRP set (`|σ| <= 1`) of 3-1-3 Euler angles \[rad\] (upstream `Euler3132MRP`).
pub fn euler313_to_mrp(angles: Vector3<f64>) -> Vector3<f64> {
    ep_to_mrp(euler313_to_ep(angles))
}

/// PRV \[rad\] of 3-1-3 Euler angles \[rad\] (upstream `Euler3132PRV`).
pub fn euler313_to_prv(angles: Vector3<f64>) -> Vector3<f64> {
    ep_to_prv(euler313_to_ep(angles))
}

/// 3-1-3 Euler angles \[rad\] of performing rotation `first` and then `second`
/// (upstream `addEuler313`).
pub fn add_euler313(first: Vector3<f64>, second: Vector3<f64>) -> Vector3<f64> {
    dcm_to_euler313(euler313_to_dcm(second) * euler313_to_dcm(first))
}

/// 3-1-3 Euler angles \[rad\] of `attitude` relative to `reference` (upstream
/// `subEuler313`).
pub fn sub_euler313(attitude: Vector3<f64>, reference: Vector3<f64>) -> Vector3<f64> {
    dcm_to_euler313(euler313_to_dcm(attitude) * euler313_to_dcm(reference).transpose())
}

/// Matrix `[B(θ)]` of the 3-1-3 kinematics `θ̇ = [B(θ)] ω` (upstream
/// `BmatEuler313`); singular at `sin θ2 = 0`.
pub fn b_mat_euler313(angles: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = (angles[1].sin(), angles[1].cos());
    let (s3, c3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(s3, c3, 0.0, c3 * s2, -s3 * s2, 0.0, -s3 * c2, -c3 * c2, s2) * (1.0 / s2)
}

/// Inverse `[B(θ)]⁻¹` of the 3-1-3 kinematics matrix, `ω = [B(θ)]⁻¹ θ̇`
/// (upstream `BinvEuler313`).
pub fn b_inv_euler313(angles: Vector3<f64>) -> Matrix3<f64> {
    let (s2, c2) = (angles[1].sin(), angles[1].cos());
    let (s3, c3) = (angles[2].sin(), angles[2].cos());
    Matrix3::new(s2 * s3, c3, 0.0, s2 * c3, -s3, 0.0, c2, 0.0, 1.0)
}

/// 3-1-3 Euler angle rates \[rad/s\] for body rate `omega` \[rad/s\] (upstream
/// `dEuler313`).
pub fn d_omega_to_euler313(angles: Vector3<f64>, omega: Vector3<f64>) -> Vector3<f64> {
    b_mat_euler313(angles) * omega
}

// ---------------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------------

fn ep_vector_part(ep: Vector4<f64>) -> Vector3<f64> {
    Vector3::new(ep[1], ep[2], ep[3])
}

/// Zero-based axes of an Euler sequence, the `firstAxis`, `secondAxis` and
/// `repeated` arguments of upstream `C2EulerSequence` / `EP2EulerRepeated`.
#[derive(Clone, Copy, Debug)]
struct EulerSequence {
    first: usize,
    second: usize,
    repeated: bool,
}

impl EulerSequence {
    const S123: Self = Self {
        first: 0,
        second: 1,
        repeated: false,
    };
    const S313: Self = Self {
        first: 2,
        second: 0,
        repeated: true,
    };
    const S321: Self = Self {
        first: 2,
        second: 1,
        repeated: false,
    };

    fn remaining(self) -> usize {
        3 - self.first - self.second
    }

    /// +1 for a cyclic (1→2, 2→3, 3→1) first-to-second step, -1 otherwise.
    fn parity(self) -> f64 {
        if self.second == (self.first + 1) % 3 {
            1.0
        } else {
            -1.0
        }
    }
}

/// Upstream `C2EulerSequence`: remove the third rotation before extracting the
/// first angle from the second-axis row, which the middle rotation leaves
/// untouched. This avoids extracting two ill-conditioned outer angles
/// independently near a singularity, where the third angle is set to zero.
fn dcm_to_euler_sequence(c: &Matrix3<f64>, sequence: EulerSequence) -> Vector3<f64> {
    let (i, j, k) = (sequence.first, sequence.second, sequence.remaining());
    let parity = sequence.parity();
    let (middle, last, sine, cosine);
    if sequence.repeated {
        let magnitude = c[(i, j)].hypot(c[(i, k)]);
        middle = magnitude.atan2(c[(i, i)]);
        last = if magnitude > EULER_SINGULARITY_TOLERANCE {
            c[(j, i)].atan2(parity * c[(k, i)])
        } else {
            0.0
        };
        let (sine_last, cosine_last) = (last.sin(), last.cos());
        sine = parity * cosine_last * c[(j, k)] - sine_last * c[(k, k)];
        cosine = cosine_last * c[(j, j)] - parity * sine_last * c[(k, j)];
    } else {
        let magnitude = c[(j, i)].hypot(c[(i, i)]);
        middle = (parity * c[(k, i)]).atan2(magnitude);
        last = if magnitude > EULER_SINGULARITY_TOLERANCE {
            (-parity * c[(j, i)]).atan2(c[(i, i)])
        } else {
            0.0
        };
        let (sine_last, cosine_last) = (last.sin(), last.cos());
        sine = parity * cosine_last * c[(j, k)] + sine_last * c[(i, k)];
        cosine = cosine_last * c[(j, j)] + parity * sine_last * c[(i, j)];
    }
    Vector3::new(sine.atan2(cosine), middle, last)
}

/// Upstream `EP2EulerRepeated`: repeated-axis Euler angles straight from the
/// quaternion, falling back to the DCM extraction at a singularity.
fn ep_to_euler_repeated(ep: Vector4<f64>, sequence: EulerSequence) -> Vector3<f64> {
    let (i, j, k) = (sequence.first, sequence.second, sequence.remaining());
    let parity = sequence.parity();
    let sine_half = ep[j + 1].hypot(ep[k + 1]);
    let cosine_half = ep[0].hypot(ep[i + 1]);
    if 2.0 * sine_half.min(cosine_half) <= EULER_SINGULARITY_TOLERANCE {
        return dcm_to_euler_sequence(&ep_to_dcm(ep), sequence);
    }
    let difference = ep[k + 1].atan2(ep[j + 1]);
    let total = ep[i + 1].atan2(ep[0]);
    Vector3::new(
        total + parity * difference,
        2.0 * sine_half.atan2(cosine_half),
        total - parity * difference,
    )
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `architecture/utilities/tests/test_rigidBodyKinematics.cpp`
    //! (for the ported Euler sequences 1-2-3, 3-1-3 and 3-2-1) and of the
    //! hard-coded truth values in `src/tests/test_rbkRoutine.py` (compared like
    //! `np.allclose(atol=1e-14)`, i.e. with numpy's default `rtol=1e-5`), plus
    //! round-trip and consistency properties.
    //!
    //! Not ported: the Gibbs-vector checks (`PRV2Gibbs`, `Gibbs2PRV`) of
    //! `PrvKinematics.SmallConversions` and the `AttitudeConversions.InPlaceQuaternion`
    //! test, which checks C pointer aliasing that a by-value Rust API cannot have.

    use std::f64::consts::PI;

    use nalgebra::{Dim, Matrix, Matrix3, RawStorage, Vector3, Vector4};
    use rand::rngs::StdRng;
    use rand::{Rng, SeedableRng};

    use super::*;

    const D2R: f64 = PI / 180.0;
    const DCM_TOLERANCE: f64 = 5e-14;

    /// A composition or rate function of two 3-vectors.
    type BinaryFn = fn(Vector3<f64>, Vector3<f64>) -> Vector3<f64>;
    /// A kinematic-matrix function of one 3-vector.
    type MatrixFn = fn(Vector3<f64>) -> Matrix3<f64>;

    // -- comparison helpers ---------------------------------------------------

    /// `np.allclose(actual, expected, atol=1e-14)` as used by `test_rbkRoutine.py`.
    fn assert_allclose<R, C, S1, S2>(
        name: &str,
        actual: &Matrix<f64, R, C, S1>,
        expected: &Matrix<f64, R, C, S2>,
    ) where
        R: Dim,
        C: Dim,
        S1: RawStorage<f64, R, C>,
        S2: RawStorage<f64, R, C>,
    {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert!(
                (a - e).abs() <= 1e-14 + 1e-5 * e.abs(),
                "{name}: {actual} != {expected}"
            );
        }
    }

    fn assert_near(actual: f64, expected: f64, tolerance: f64, context: &str) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "{context}: {actual:e} vs {expected:e} (tolerance {tolerance:e})"
        );
    }

    /// googletest `EXPECT_DOUBLE_EQ`: at most four ulps apart.
    fn assert_double_eq(actual: f64, expected: f64, context: &str) {
        fn biased(x: f64) -> u64 {
            const SIGN: u64 = 1 << 63;
            let bits = x.to_bits();
            if bits & SIGN != 0 {
                (!bits).wrapping_add(1)
            } else {
                bits | SIGN
            }
        }
        assert!(
            !actual.is_nan()
                && !expected.is_nan()
                && biased(actual).abs_diff(biased(expected)) <= 4,
            "{context}: {actual:e} is not within 4 ulps of {expected:e}"
        );
    }

    // -- test_rbkRoutine.py truth values ---------------------------------------

    fn ep_pair() -> (Vector4<f64>, Vector4<f64>) {
        (
            Vector4::new(
                0.45226701686665,
                0.75377836144441,
                0.15075567228888,
                0.45226701686665,
            ),
            Vector4::new(
                -0.18663083698528,
                0.46657709246321,
                0.83983876643378,
                -0.20529392068381,
            ),
        )
    }

    fn euler_pair() -> (Vector3<f64>, Vector3<f64>) {
        (
            Vector3::new(10.0, 20.0, 30.0) * D2R,
            Vector3::new(-30.0, 200.0, 81.0) * D2R,
        )
    }

    fn reference_dcm() -> Matrix3<f64> {
        Matrix3::new(
            -0.506611258027956,
            -0.05213449187759728,
            0.860596902153381,
            -0.7789950887797505,
            -0.4000755572346052,
            -0.4828107291273137,
            0.3694748772194938,
            -0.9149981110691346,
            0.1620702682281828,
        )
    }

    fn reference_ep() -> Vector4<f64> {
        Vector4::new(
            0.2526773896521122,
            0.4276078901804977,
            -0.4859180570232927,
            0.7191587243944733,
        )
    }

    #[test]
    fn matches_upstream_composition_truth_values() {
        let (b1, b2) = ep_pair();
        assert_allclose(
            "addEP",
            &add_ep(b1, b2),
            &Vector4::new(
                -0.46986547690254,
                -0.34044145332460,
                0.71745926113861,
                0.38545850500388,
            ),
        );
        assert_allclose(
            "subEP",
            &sub_ep(b1, b2),
            &Vector4::new(
                0.3010515331052196,
                -0.762476312817895,
                -0.0422034859493331,
                0.5711538431809339,
            ),
        );

        let (e1, e2) = euler_pair();
        let cases: [(&str, BinaryFn, [f64; 3]); 6] = [
            (
                "addEuler313",
                add_euler313,
                [-2.96705972839036, 2.44346095279206, -1.72787595947439],
            ),
            (
                "addEuler123",
                add_euler123,
                [2.65556257351773, -0.34257634487528, -2.38843896474589],
            ),
            (
                "addEuler321",
                add_euler321,
                [2.93168877067466, -0.89056295435594, -2.11231276758895],
            ),
            (
                "subEuler123",
                sub_euler123,
                [3.116108453572625, -0.6539785291371149, -0.9652248604105184],
            ),
            (
                "subEuler321",
                sub_euler321,
                [2.932019083757663, 0.6246626379494424, -1.519867235625338],
            ),
            (
                "subEuler313",
                sub_euler313,
                [-0.172468571243551, 2.907100217278789, 2.423943306316236],
            ),
        ];
        for (name, compose, expected) in cases {
            assert_allclose(name, &compose(e1, e2), &Vector3::from(expected));
        }

        let q1 = Vector3::new(1.5, 0.5, 0.5);
        let q2 = Vector3::new(-0.5, 0.25, 0.15);
        assert_allclose(
            "addMRP",
            &add_mrp(q1, q2),
            &Vector3::new(0.58667769962764, -0.34919321472900, 0.43690525444766),
        );
        assert_allclose(
            "subMRP",
            &sub_mrp(q1, q2),
            &Vector3::new(-0.00537634, 0.04301075, -0.44086022),
        );
        assert_allclose(
            "addPRV",
            &add_prv(q1, q2),
            &Vector3::new(1.00227389370983, 0.41720669426711, 0.86837149207759),
        );
        assert_allclose(
            "subPRV",
            &sub_prv(q1, q2),
            &Vector3::new(1.899971363060601, 0.06138537390284331, 0.7174863730592785),
        );
    }

    #[test]
    fn matches_upstream_euler_kinematic_matrix_truth_values() {
        let v = Vector3::new(30.0, -40.0, 15.0) * D2R;
        let cases: [(&str, MatrixFn, [f64; 9]); 6] = [
            (
                "BmatEuler123",
                b_mat_euler123,
                [
                    1.26092661459205,
                    -0.33786426809485,
                    0.0,
                    0.25881904510252,
                    0.96592582628907,
                    0.0,
                    0.81050800458377,
                    -0.21717496528718,
                    1.0,
                ],
            ),
            (
                "BmatEuler313",
                b_mat_euler313,
                [
                    -0.40265095531125,
                    -1.50271382293774,
                    0.0,
                    0.96592582628907,
                    -0.25881904510252,
                    0.0,
                    0.30844852683273,
                    1.15114557365953,
                    1.0,
                ],
            ),
            (
                "BmatEuler321",
                b_mat_euler321,
                [
                    0.0,
                    0.33786426809485,
                    1.26092661459205,
                    0.0,
                    0.96592582628907,
                    -0.25881904510252,
                    1.0,
                    -0.21717496528718,
                    -0.81050800458377,
                ],
            ),
            (
                "BinvEuler123",
                b_inv_euler123,
                [
                    0.73994211169385,
                    0.25881904510252,
                    0.0,
                    -0.19826689127415,
                    0.96592582628907,
                    0.0,
                    -0.64278760968654,
                    0.0,
                    1.0,
                ],
            ),
            (
                "BinvEuler313",
                b_inv_euler313,
                [
                    -0.16636567534280,
                    0.96592582628907,
                    0.0,
                    -0.62088515301485,
                    -0.25881904510252,
                    0.0,
                    0.76604444311898,
                    0.0,
                    1.0,
                ],
            ),
            (
                "BinvEuler321",
                b_inv_euler321,
                [
                    0.64278760968654,
                    0.0,
                    1.0,
                    0.19826689127415,
                    0.96592582628907,
                    0.0,
                    0.73994211169385,
                    -0.25881904510252,
                    0.0,
                ],
            ),
        ];
        for (name, matrix, expected) in cases {
            assert_allclose(name, &matrix(v), &Matrix3::from_row_slice(&expected));
        }
    }

    #[test]
    fn matches_upstream_mrp_and_prv_kinematic_matrix_truth_values() {
        let v = Vector3::new(0.25, 0.5, -0.5);
        let cases: [(&str, MatrixFn, [f64; 9]); 4] = [
            (
                "BinvMRP",
                b_inv_mrp,
                [
                    0.2304, -0.3072, -0.512, 0.512, 0.384, 0.0, 0.3072, -0.4096, 0.3840,
                ],
            ),
            (
                "BinvPRV",
                b_inv_prv,
                [
                    0.91897927113877,
                    -0.21824360100796,
                    -0.25875396543858,
                    0.25875396543858,
                    0.94936204446173,
                    0.07873902718102,
                    0.21824360100796,
                    -0.15975975604225,
                    0.94936204446173,
                ],
            ),
            (
                "BmatMRP",
                b_mat_mrp,
                [0.5625, 1.25, 0.75, -0.75, 0.9375, -1.0, -1.25, 0.0, 0.9375],
            ),
            (
                "BmatPRV",
                b_mat_prv,
                [
                    0.95793740211924,
                    0.26051564947019,
                    0.23948435052981,
                    -0.23948435052981,
                    0.97371087632453,
                    -0.14603129894038,
                    -0.26051564947019,
                    0.10396870105962,
                    0.97371087632453,
                ],
            ),
        ];
        for (name, matrix, expected) in cases {
            assert_allclose(name, &matrix(v), &Matrix3::from_row_slice(&expected));
        }
    }

    #[test]
    fn matches_upstream_dcm_and_ep_conversion_truth_values() {
        let c = reference_dcm();
        let q = reference_ep();
        assert_allclose("C2EP", &dcm_to_ep(c), &q);
        assert_allclose("EP2C", &ep_to_dcm(q), &c);
        let euler123 = Vector3::new(1.395488250243478, 0.3784438476398376, 2.147410157986089);
        let euler321 = Vector3::new(-3.039045355374235, -1.036440549977791, -1.246934586231547);
        assert_allclose("C2Euler123", &dcm_to_euler123(c), &euler123);
        assert_allclose(
            "C2Euler313",
            &dcm_to_euler313(c),
            &Vector3::new(0.3837766626244829, 1.408008028147626, 2.082059614484753),
        );
        assert_allclose("C2Euler321", &dcm_to_euler321(c), &euler321);
        assert_allclose(
            "C2MRP",
            &dcm_to_mrp(c),
            &Vector3::new(0.3413551595269481, -0.3879035903715318, 0.5740973137498672),
        );
        assert_allclose(
            "C2PRV",
            &dcm_to_prv(c),
            &Vector3::new(1.162634795241009, -1.321175903682964, 1.955340337450788),
        );
        assert_allclose("EP2Euler123", &ep_to_euler123(q), &euler123);
        assert_allclose(
            "EP2Euler313",
            &ep_to_euler313(q),
            &Vector3::new(0.3837766626244828, 1.408008028147627, 2.082059614484753),
        );
        assert_allclose("EP2Euler321", &ep_to_euler321(q), &euler321);
        assert_allclose(
            "EP2MRP",
            &ep_to_mrp(q),
            &Vector3::new(0.3413551595269481, -0.3879035903715319, 0.5740973137498672),
        );
        assert_allclose(
            "EP2PRV",
            &ep_to_prv(q),
            &Vector3::new(1.162634795241009, -1.321175903682965, 1.955340337450788),
        );
    }

    #[test]
    fn matches_upstream_euler_conversion_truth_values() {
        let e = Vector3::new(0.5746957711326909, -0.7662610281769212, 0.2873478855663454);
        assert_allclose(
            "euler1232C",
            &euler123_to_dcm(e),
            &Matrix3::new(
                0.6909668228739537,
                -0.1236057418710468,
                0.7122404581768593,
                -0.2041991989591971,
                0.9117724894309838,
                0.3563335721781613,
                -0.6934461311680212,
                -0.391653607277317,
                0.6047643467291773,
            ),
        );
        assert_allclose(
            "euler1232EP",
            &euler123_to_ep(e),
            &Vector4::new(
                0.8954752451958283,
                0.2088240806958052,
                -0.3924414987701519,
                0.02250019124496444,
            ),
        );
        assert_allclose(
            "euler1232MRP",
            &euler123_to_mrp(e),
            &Vector3::new(0.1101697746911123, -0.2070412155288303, 0.01187047485953311),
        );
        assert_allclose(
            "euler1232PRV",
            &euler123_to_prv(e),
            &Vector3::new(0.4328366663508259, -0.8134266388215754, 0.04663690000825693),
        );
        assert_allclose(
            "euler3132C",
            &euler313_to_dcm(e),
            &Matrix3::new(
                0.6939446195986547,
                0.692688266609151,
                -0.1965294640304305,
                -0.6134776155495705,
                0.4259125598286639,
                -0.6650140649638986,
                -0.3769430728235922,
                0.5820493593177511,
                0.7205084754311385,
            ),
        );
        assert_allclose(
            "euler3132EP",
            &euler313_to_ep(e),
            &Vector4::new(
                0.8426692196316502,
                -0.3699741829975614,
                -0.05352444488005169,
                0.3875084824890354,
            ),
        );
        assert_allclose(
            "euler3132MRP",
            &euler313_to_mrp(e),
            &Vector3::new(
                -0.2007816590497557,
                -0.02904723447366817,
                0.2102973655610845,
            ),
        );
        assert_allclose(
            "euler3132PRV",
            &euler313_to_prv(e),
            &Vector3::new(-0.7813731087574279, -0.1130418386266624, 0.8184049632304388),
        );
        assert_allclose(
            "euler3212C",
            &euler321_to_dcm(e),
            &Matrix3::new(
                0.6047643467291773,
                0.391653607277317,
                0.6934461311680212,
                -0.6862506154337003,
                0.6981137299618809,
                0.2041991989591971,
                -0.404128912281835,
                -0.5993702294453531,
                0.6909668228739537,
            ),
        );
        assert_allclose(
            "euler3212EP",
            &euler321_to_ep(e),
            &Vector4::new(
                0.8651365354042408,
                0.2322088466732818,
                -0.3171681574333834,
                0.3114838463640192,
            ),
        );
        assert_allclose(
            "euler3212MRP",
            &euler321_to_mrp(e),
            &Vector3::new(0.1244996504360223, -0.1700509058789317, 0.1670032410235906),
        );
        assert_allclose(
            "euler3212PRV",
            &euler321_to_prv(e),
            &Vector3::new(0.4864908592507521, -0.6644854907437873, 0.6525765328552258),
        );
    }

    #[test]
    fn matches_upstream_mrp_conversion_truth_values() {
        let v = Vector3::new(0.2, -0.25, 0.3);
        assert_allclose(
            "MRP2C",
            &mrp_to_dcm(v),
            &Matrix3::new(
                0.1420873822677549,
                0.4001248192538094,
                0.9053790945330048,
                -0.9626904702257736,
                0.2686646537364468,
                0.03234752493088797,
                -0.2303003133666478,
                -0.876196001388834,
                0.4233702077537369,
            ),
        );
        assert_allclose(
            "MRP2EP",
            &mrp_to_ep(v),
            &Vector4::new(
                0.6771488469601677,
                0.3354297693920336,
                -0.419287211740042,
                0.5031446540880503,
            ),
        );
        assert_allclose(
            "MRP2Euler123",
            &mrp_to_euler123(v),
            &Vector3::new(1.120685944613971, -0.2323862804943196, 1.424260216144192),
        );
        assert_allclose(
            "MRP2Euler313",
            &mrp_to_euler313(v),
            &Vector3::new(-0.257027406977469, 1.133634172515794, 1.535083362165219),
        );
        assert_allclose(
            "MRP2Euler321",
            &mrp_to_euler321(v),
            &Vector3::new(1.22957853325386, -1.13227169191098, 0.0762566635156139),
        );
        assert_allclose(
            "MRP2PRV",
            &mrp_to_prv(v),
            &Vector3::new(0.7538859486650076, -0.9423574358312593, 1.130828922997511),
        );
        assert_allclose("MRPswitch", &mrp_switch(v, 1.0), &v);
        assert_allclose(
            "MRPswitch",
            &mrp_switch(v, 0.4),
            &Vector3::new(-1.038961038961039, 1.298701298701299, -1.558441558441558),
        );
    }

    #[test]
    fn matches_upstream_prv_conversion_truth_values() {
        let v = Vector3::new(0.2, -0.25, 0.3);
        assert_allclose(
            "PRV2C",
            &prv_to_dcm(v),
            &Matrix3::new(
                0.9249653552860658,
                0.2658656942983466,
                0.2715778417245783,
                -0.3150687400124018,
                0.9360360405717283,
                0.1567425271513747,
                -0.212534186867712,
                -0.2305470957224576,
                0.9495668781430935,
            ),
        );
        assert_allclose(
            "PRV2EP",
            &prv_to_ep(v),
            &Vector4::new(
                0.9760338459808767,
                0.09919984446969178,
                -0.1239998055871147,
                0.1487997667045377,
            ),
        );
        assert_allclose(
            "PRV2Euler123",
            &prv_to_euler123(v),
            &Vector3::new(0.2381830975647435, -0.2141676691157164, 0.3283009769818029),
        );
        assert_allclose(
            "PRV2Euler313",
            &prv_to_euler313(v),
            &Vector3::new(-0.7447668031423726, 0.3189446151924337, 1.047343966000315),
        );
        assert_allclose(
            "PRV2Euler321",
            &prv_to_euler321(v),
            &Vector3::new(0.2798880637473677, -0.2750321114914171, 0.1635922230133545),
        );
        assert_allclose(
            "PRV2MRP",
            &prv_to_mrp(v),
            &Vector3::new(
                0.05020149056224809,
                -0.06275186320281011,
                0.07530223584337212,
            ),
        );
        let elements = Vector4::new(
            0.9110886174894189,
            0.5746957711326909,
            -0.7662610281769212,
            0.2873478855663454,
        );
        assert_allclose(
            "elem2PRV",
            &elem_to_prv(elements[0], ep_vector_part(elements)),
            &Vector3::new(0.5235987755982988, -0.6981317007977318, 0.2617993877991494),
        );
    }

    #[test]
    fn matches_upstream_rate_truth_values() {
        let w = Vector3::new(0.2, 0.1, -0.5);
        assert_allclose(
            "dEP",
            &d_omega_to_ep(reference_ep(), w),
            &Vector4::new(
                0.1613247949317332,
                0.1107893170013107,
                0.1914517144671774,
                0.006802852798326098,
            ),
        );
        let q = Vector3::new(30.0, -40.0, 15.0) * D2R;
        let cases: [(&str, BinaryFn, [f64; 3]); 5] = [
            (
                "dEuler123",
                d_omega_to_euler123,
                [0.2183988961089258, 0.148356391649411, -0.3596158956119647],
            ),
            (
                "dEuler313",
                d_omega_to_euler313,
                [-0.2308015733560238, 0.1673032607475616, -0.3231957372675008],
            ),
            (
                "dEuler321",
                d_omega_to_euler321,
                [-0.596676880486542, 0.2260021051801672, 0.5835365057631652],
            ),
            (
                "dMRP",
                d_omega_to_mrp,
                [0.144807895231133, 0.1948354871330581, 0.062187948908334],
            ),
            (
                "dPRV",
                d_omega_to_prv,
                [0.34316538031149, 0.255728121815202, -0.3710557691157747],
            ),
        ];
        for (name, rate, expected) in cases {
            assert_allclose(name, &rate(q, w), &Vector3::from(expected));
        }
    }

    #[test]
    fn matches_upstream_elementary_rotation_truth_values() {
        let x = 1.3;
        let (c, s) = (0.2674988286245874, 0.963558185417193);
        assert_allclose(
            "euler1",
            &euler1(x),
            &Matrix3::new(1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c),
        );
        assert_allclose(
            "euler2",
            &euler2(x),
            &Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c),
        );
        assert_allclose(
            "euler3",
            &euler3(x),
            &Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0),
        );

        let theta = 30.0 * D2R;
        let (c, s) = (0.8660254037844387, 0.4999999999999999);
        assert_allclose(
            "Mi(3)",
            &mi(theta, Axis::Z),
            &Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0),
        );
        assert_allclose(
            "Mi(2)",
            &mi(theta, Axis::Y),
            &Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c),
        );
        assert_allclose(
            "Mi(1)",
            &mi(theta, Axis::X),
            &Matrix3::new(1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c),
        );
    }

    // -- test_rigidBodyKinematics.cpp: EulerConversions -------------------------

    struct SequenceCase {
        name: &'static str,
        axes: [usize; 3],
        from_dcm: fn(Matrix3<f64>) -> Vector3<f64>,
        from_ep: fn(Vector4<f64>) -> Vector3<f64>,
        add: fn(Vector3<f64>, Vector3<f64>) -> Vector3<f64>,
        subtract: fn(Vector3<f64>, Vector3<f64>) -> Vector3<f64>,
    }

    impl SequenceCase {
        fn repeated(&self) -> bool {
            self.axes[0] == self.axes[2]
        }

        /// Range of the middle angle: `[0, π]` for repeated-axis sets.
        fn middle_limits(&self) -> [f64; 2] {
            if self.repeated() {
                [0.0, PI]
            } else {
                [-PI / 2.0, PI / 2.0]
            }
        }
    }

    const SEQUENCES: [SequenceCase; 3] = [
        SequenceCase {
            name: "123",
            axes: [0, 1, 2],
            from_dcm: dcm_to_euler123,
            from_ep: ep_to_euler123,
            add: add_euler123,
            subtract: sub_euler123,
        },
        SequenceCase {
            name: "313",
            axes: [2, 0, 2],
            from_dcm: dcm_to_euler313,
            from_ep: ep_to_euler313,
            add: add_euler313,
            subtract: sub_euler313,
        },
        SequenceCase {
            name: "321",
            axes: [2, 1, 0],
            from_dcm: dcm_to_euler321,
            from_ep: ep_to_euler321,
            add: add_euler321,
            subtract: sub_euler321,
        },
    ];

    /// Upstream `elementaryProduct`: passive DCM built independently from
    /// elementary rotations, optionally snapping tiny sines and cosines to zero.
    fn elementary_product(axes: [usize; 3], angles: [f64; 3], exact: bool) -> Matrix3<f64> {
        axes.iter()
            .zip(angles)
            .fold(Matrix3::identity(), |matrix, (&axis, angle)| {
                let row = (axis + 1) % 3;
                let column = (axis + 2) % 3;
                let (mut cosine, mut sine) = (angle.cos(), angle.sin());
                if exact {
                    cosine = if cosine.abs() < 1e-15 { 0.0 } else { cosine };
                    sine = if sine.abs() < 1e-15 { 0.0 } else { sine };
                }
                let mut rotation = Matrix3::identity();
                rotation[(row, row)] = cosine;
                rotation[(column, column)] = cosine;
                rotation[(row, column)] = sine;
                rotation[(column, row)] = -sine;
                rotation * matrix
            })
    }

    /// Upstream `checkRotation`.
    fn check_rotation(
        sequence: &SequenceCase,
        matrix: &Matrix3<f64>,
        angles: Vector3<f64>,
        singular: bool,
    ) {
        let context = format!("sequence {} angles {angles:?}", sequence.name);
        assert!(angles.iter().all(|a| a.is_finite()), "{context}");
        if singular {
            assert_double_eq(angles[2], 0.0, &context);
        }
        let [low, high] = sequence.middle_limits();
        assert!(angles[1] >= low && angles[1] <= high, "{context}");
        let recovered = elementary_product(sequence.axes, angles.into(), false);
        for (r, m) in recovered.iter().zip(matrix.iter()) {
            assert_near(*r, *m, DCM_TOLERANCE, &context);
        }
    }

    /// Upstream `checkConversions`: the DCM and both signs of its quaternion.
    fn check_conversions(sequence: &SequenceCase, matrix: &Matrix3<f64>, singular: bool) {
        check_rotation(sequence, matrix, (sequence.from_dcm)(*matrix), singular);
        let quaternion = dcm_to_ep(*matrix);
        for q in [quaternion, -quaternion] {
            check_rotation(sequence, matrix, (sequence.from_ep)(q), singular);
        }
    }

    #[test]
    fn euler_conversions_preserve_singular_attitudes() {
        for sequence in &SEQUENCES {
            for middle in sequence.middle_limits() {
                let mut matrix = elementary_product(sequence.axes, [0.7, middle, -1.1], true);
                for zero in [0.0, -0.0] {
                    for value in matrix.iter_mut() {
                        if *value == 0.0 {
                            *value = zero;
                        }
                    }
                    check_conversions(sequence, &matrix, true);
                }
            }
        }
    }

    #[test]
    fn euler_conversions_preserve_first_axis_rotations() {
        for sequence in &SEQUENCES {
            for angle in [0.0, 0.5] {
                let matrix = elementary_product(sequence.axes, [angle, 0.0, 0.0], false);
                check_conversions(sequence, &matrix, sequence.repeated());
            }
        }
    }

    #[test]
    fn euler_conversions_preserve_rotations_near_singularities() {
        for sequence in &SEQUENCES {
            for middle in sequence.middle_limits() {
                for offset in [1e-4, 1e-8, 1e-12, 1e-15] {
                    for direction in [-1.0, 1.0] {
                        let angles = [0.7, middle + direction * offset, -1.1];
                        let matrix = elementary_product(sequence.axes, angles, false);
                        check_conversions(sequence, &matrix, false);
                    }
                }
            }
        }
    }

    #[test]
    fn euler_conversions_preserve_ordinary_rotations() {
        for sequence in &SEQUENCES {
            for angles in [[0.7, 0.8, -1.1], [-2.8, 0.8, 2.6], [2.9, 0.8, 2.7]] {
                let matrix = elementary_product(sequence.axes, angles, false);
                check_conversions(sequence, &matrix, false);
            }
        }
    }

    #[test]
    fn euler_composition_matches_dcm_products() {
        let cases = [
            ([0.2, 0.5, 0.7], [0.0, 0.0, 0.0]),
            ([0.0, 0.0, 0.0], [0.2, 0.5, 0.7]),
            ([0.2, 0.5, 0.7], [0.2, 0.5, 0.7]),
            ([0.7, 0.0, -1.1], [-0.3, 0.8, 0.9]),
            ([0.7, PI, -1.1], [-0.3, 0.8, 0.9]),
            ([0.7, 1e-12, -1.1], [-0.3, PI - 1e-12, 0.9]),
            ([0.7, 0.0, 0.0], [-0.7, 0.0, 0.0]),
            ([-2.8, 0.8, 2.6], [2.9, -0.6, 2.7]),
        ];
        for sequence in &SEQUENCES {
            for (first_angles, second_angles) in cases {
                let first = elementary_product(sequence.axes, first_angles, false);
                let second = elementary_product(sequence.axes, second_angles, false);
                let (a, b) = (Vector3::from(first_angles), Vector3::from(second_angles));
                check_rotation(sequence, &(second * first), (sequence.add)(a, b), false);
                check_rotation(
                    sequence,
                    &(first * second.transpose()),
                    (sequence.subtract)(a, b),
                    false,
                );
            }
        }
    }

    // -- test_rigidBodyKinematics.cpp: PrvKinematics ----------------------------

    /// One series term `term * [-ṽ] / divisor`, summed in upstream's order.
    fn series_step(
        term: &Matrix3<f64>,
        negative_skew: &Matrix3<f64>,
        divisor: f64,
    ) -> Matrix3<f64> {
        Matrix3::from_fn(|row, column| {
            (0..3).fold(0.0, |sum, inner| {
                sum + term[(row, inner)] * negative_skew[(inner, column)] / divisor
            })
        })
    }

    #[test]
    fn prv_rate_matrices_match_the_integrated_exponential_series() {
        let angles = [
            0.0, 1e-16, 1e-12, 1e-9, 1e-5, 0.009999, 0.01, 0.010001, 0.2, 2.0, 5.0,
        ];
        for angle in angles {
            let context = format!("angle {angle:e}");
            let vector = Vector3::new(angle * 2.0 / 7.0, -angle * 3.0 / 7.0, angle * 6.0 / 7.0);
            let negative_skew = -vector.cross_matrix();
            // Σ (-[ṽ])ⁿ / (n + 1)!, the exact inverse PRV rate matrix.
            let mut expected = Matrix3::identity();
            let mut term = Matrix3::identity();
            for order in 1..=48 {
                term = series_step(&term, &negative_skew, f64::from(order + 1));
                expected += term;
            }
            let matrix = b_mat_prv(vector);
            let inverse = b_inv_prv(vector);
            let omega = Vector3::new(0.1, -0.2, 0.3);
            let derivative = d_omega_to_prv(vector, omega);
            for row in 0..3 {
                let mut recovered_rate = 0.0;
                for column in 0..3 {
                    let identity = if row == column { 1.0 } else { 0.0 };
                    let e = expected[(row, column)];
                    assert!(matrix[(row, column)].is_finite(), "{context}");
                    assert_near(inverse[(row, column)], e, 3e-14 * e.abs() + 1e-22, &context);
                    let product = (0..3).fold(0.0, |sum, inner| {
                        sum + expected[(row, inner)] * matrix[(inner, column)]
                    });
                    assert_near(product, identity, 3e-14, &context);
                    recovered_rate += e * derivative[column];
                    if angle == 0.0 {
                        assert_double_eq(matrix[(row, column)], identity, &context);
                        assert_double_eq(inverse[(row, column)], identity, &context);
                    }
                }
                assert_near(recovered_rate, omega[row], 3e-14, &context);
                if angle == 0.0 {
                    assert_double_eq(derivative[row], omega[row], &context);
                }
            }
        }
    }

    #[test]
    fn prv_conversions_preserve_small_rotations_and_both_quaternion_branches() {
        let axes = [
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(2.0 / 7.0, -3.0 / 7.0, 6.0 / 7.0),
        ];
        for angle in [0.0_f64, 1e-16, 1e-13, 1e-9, 1e-8, 1e-7, 1e-3, 0.4] {
            for axis in axes {
                let context = format!("angle {angle:e} axis {axis:?}");
                let half_sine = (angle / 2.0).sin();
                let mut quaternion = Vector4::new(
                    (angle / 2.0).cos(),
                    axis[0] * half_sine,
                    axis[1] * half_sine,
                    axis[2] * half_sine,
                );
                let vector = axis * angle;
                let (element_angle, element_axis) = prv_to_elem(vector);
                let converted = prv_to_ep(vector);
                assert_near(element_angle, angle, angle * 5e-15, &context);
                for index in 0..4 {
                    let q = quaternion[index];
                    assert_near(converted[index], q, q.abs() * 5e-15, &context);
                }
                for index in 0..3 {
                    let expected_axis = if angle == 0.0 { 0.0 } else { axis[index] };
                    assert_near(element_axis[index], expected_axis, 5e-15, &context);
                }
                let mrp = prv_to_mrp(vector);
                for index in 0..3 {
                    let expected = axis[index] * (angle / 4.0).tan();
                    assert_near(mrp[index], expected, angle * 5e-15, &context);
                }
                let matrix = prv_to_dcm(vector);
                let expected = ep_to_dcm(quaternion);
                for (m, e) in matrix.iter().zip(expected.iter()) {
                    assert_near(*m, *e, e.abs() * 5e-15, &context);
                }
                for result in [dcm_to_prv(expected), mrp_to_prv(mrp), ep_to_prv(quaternion)] {
                    for index in 0..3 {
                        assert_near(result[index], vector[index], angle * 5e-15, &context);
                    }
                }
                quaternion = -quaternion;
                let result = ep_to_prv(quaternion);
                for index in 0..3 {
                    let expected_angle = if angle == 0.0 {
                        0.0
                    } else {
                        (angle - 2.0 * PI) * axis[index]
                    };
                    assert_near(result[index], expected_angle, 1e-14, &context);
                }
            }
        }
    }

    /// Upstream `rotationVectorMatrix`: the passive DCM `exp(-[ṽ])` by series.
    fn rotation_vector_matrix(vector: Vector3<f64>) -> Matrix3<f64> {
        let negative_skew = -vector.cross_matrix();
        let mut matrix = Matrix3::identity();
        let mut term = Matrix3::identity();
        for order in 1..=64 {
            term = series_step(&term, &negative_skew, f64::from(order));
            matrix += term;
        }
        matrix
    }

    #[test]
    fn prv_composition_handles_cancelling_and_tiny_noncommuting_rotations() {
        let cases = [
            ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            ([1.0, -0.5, 0.3], [0.0, 0.0, 0.0]),
            ([0.0, 0.0, 0.0], [1.0, -0.5, 0.3]),
            ([1.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ([1.0, -0.5, 0.3], [-1.0, 0.5, -0.3]),
            ([1.0, -0.5, 0.3], [1.0, -0.5, 0.3]),
            ([1.0, 0.0, 0.0], [-1.0 + 1e-12, 0.0, 0.0]),
            ([1.0, 0.0, 0.0], [1.0 + 1e-12, 0.0, 0.0]),
            ([2.0, 0.0, 0.0], [2.0, 0.0, 0.0]),
            ([1.5, 0.5, 0.5], [-0.5, 0.25, 0.15]),
            ([1e-9, 2e-9, 0.0], [0.0, 0.7, -0.3]),
            ([1e-13, 2e-13, 0.0], [0.0, -3e-13, 4e-13]),
        ];
        for (first_vector, second_vector) in cases {
            let (a, b) = (Vector3::from(first_vector), Vector3::from(second_vector));
            let first = rotation_vector_matrix(a);
            let second = rotation_vector_matrix(b);
            for subtract in [false, true] {
                let context = format!("{a:?} {b:?} subtract {subtract}");
                let result = if subtract {
                    sub_prv(a, b)
                } else {
                    add_prv(a, b)
                };
                let recovered = rotation_vector_matrix(result);
                assert!(result.iter().all(|r| r.is_finite()), "{context}");
                let cancels = if subtract { a == b } else { a == -b };
                let expected = if subtract {
                    first * second.transpose()
                } else {
                    second * first
                };
                for row in 0..3 {
                    if cancels {
                        assert_double_eq(result[row], 0.0, &context);
                    }
                    for column in 0..3 {
                        assert_near(
                            recovered[(row, column)],
                            expected[(row, column)],
                            2e-14,
                            &context,
                        );
                    }
                }
            }
        }
        let result = add_prv(Vector3::new(2.0, 0.0, 0.0), Vector3::new(2.0, 0.0, 0.0));
        assert_near(result[0], 4.0, 1e-15, "addPRV keeps the 4 rad branch");
        assert_double_eq(result[1], 0.0, "addPRV");
        assert_double_eq(result[2], 0.0, "addPRV");
    }

    // -- test_rigidBodyKinematics.cpp: MrpKinematics ----------------------------

    #[test]
    fn mrp_shadow_and_composition_branches_include_a_full_turn() {
        let first = Vector3::new(1.0, 0.0, 0.0);
        let first_matrix = mrp_to_dcm(first);
        for transverse in [0.0, 1e-4] {
            let mut second = Vector3::new(1.0, transverse, 0.0);
            for subtract in [false, true] {
                if subtract {
                    second *= -1.0;
                }
                let second_matrix = mrp_to_dcm(second);
                let (expected, result) = if subtract {
                    (
                        first_matrix * second_matrix.transpose(),
                        sub_mrp(first, second),
                    )
                } else {
                    (second_matrix * first_matrix, add_mrp(first, second))
                };
                let context = format!("transverse {transverse} subtract {subtract}");
                for (a, e) in mrp_to_dcm(result).iter().zip(expected.iter()) {
                    assert_near(*a, *e, 5e-15, &context);
                }
                assert!(result.norm() <= 1.0, "{context}");
            }
        }
        let sigma = Vector3::new(2.0, 0.0, 0.0);
        let quaternion = mrp_to_ep(sigma);
        assert_near(quaternion[0], -0.6, 1e-15, "MRP2EP");
        assert_near(quaternion[1], 0.8, 1e-15, "MRP2EP");
        let shadow = mrp_switch(sigma, 1.0);
        assert_double_eq(shadow[0], -0.5, "MRPswitch");
        assert_double_eq(mrp_shadow(sigma)[0], shadow[0], "MRPshadow");
    }

    // -- test_rigidBodyKinematics.cpp: AngleWrapping ----------------------------

    #[test]
    fn wrap_to_pi_removes_multiple_revolutions() {
        let turns = [
            -1_000_000, -100, -10, -3, -2, -1, 0, 1, 2, 3, 10, 100, 1_000_000,
        ];
        for turn in turns {
            for offset in [-2.9, -0.2, 0.0, 0.2, 2.9] {
                let angle = f64::from(turn) * (2.0 * PI) + offset;
                let result = wrap_to_pi(angle);
                let scale = if angle.abs() > 1.0 { angle.abs() } else { 1.0 };
                let tolerance = 4.0 * f64::EPSILON * scale;
                let context = format!("angle {angle}");
                assert!((-PI..=PI).contains(&result), "{context}");
                assert_near(result, offset, tolerance, &context);
                assert_double_eq(wrap_to_pi(result), result, &context);
            }
        }
    }

    #[test]
    fn wrap_to_pi_preserves_the_closed_interval_signed_zeros_and_half_turns() {
        for angle in [
            0.0,
            -0.0,
            1e-12,
            -1e-12,
            0.2,
            -0.2,
            PI,
            -PI,
            PI.next_down(),
            (-PI).next_up(),
        ] {
            let result = wrap_to_pi(angle);
            assert_double_eq(result, angle, "boundary");
            assert_eq!(result.is_sign_negative(), angle.is_sign_negative());
        }
        for multiple in [1.0, 3.0, 5.0] {
            for sign in [-1.0, 1.0] {
                let boundary: f64 = sign * (multiple * PI);
                assert_double_eq(wrap_to_pi(boundary), sign * PI, "half turn");
                let (inside, outside) = if sign > 0.0 {
                    (boundary.next_down(), boundary.next_up())
                } else {
                    (boundary.next_up(), boundary.next_down())
                };
                let inside = wrap_to_pi(inside);
                let outside = wrap_to_pi(outside);
                assert!(sign * inside > 0.0 && sign * inside < PI, "{boundary}");
                assert!(sign * outside > -PI && sign * outside < 0.0, "{boundary}");
            }
        }
        for multiple in [2.0, 4.0, 8.0] {
            for sign in [-1.0, 1.0] {
                let result = wrap_to_pi(sign * multiple * PI);
                assert_double_eq(result, 0.0, "full turn");
                assert_eq!(result.is_sign_negative(), sign < 0.0);
            }
        }
    }

    #[test]
    fn wrap_to_pi_returns_nan_for_nonfinite_angles() {
        for angle in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(wrap_to_pi(angle).is_nan());
        }
    }

    // -- properties --------------------------------------------------------------

    const SAMPLES: usize = 500;
    const ROUND_TRIP_TOLERANCE: f64 = 1e-12;

    fn unit_vector(rng: &mut StdRng) -> Vector3<f64> {
        loop {
            let v = Vector3::new(
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
                rng.gen_range(-1.0..1.0),
            );
            let norm = v.norm();
            if norm > 1e-3 && norm <= 1.0 {
                return v / norm;
            }
        }
    }

    fn assert_vectors_near<R, C, S1, S2>(
        actual: &Matrix<f64, R, C, S1>,
        expected: &Matrix<f64, R, C, S2>,
        context: &str,
    ) where
        R: Dim,
        C: Dim,
        S1: RawStorage<f64, R, C>,
        S2: RawStorage<f64, R, C>,
    {
        for (a, e) in actual.iter().zip(expected.iter()) {
            assert_near(*a, *e, ROUND_TRIP_TOLERANCE, context);
        }
    }

    #[test]
    fn every_parameterization_round_trips_through_the_dcm() {
        let mut rng = StdRng::seed_from_u64(0x5eed);
        for _ in 0..SAMPLES {
            let axis = unit_vector(&mut rng);

            let sigma = axis * rng.gen_range(0.0..0.99);
            assert_vectors_near(&dcm_to_mrp(mrp_to_dcm(sigma)), &sigma, "MRP");

            let angle = rng.gen_range(0.0..PI - 0.01);
            let ep = Vector4::new(
                (angle / 2.0).cos(),
                axis[0] * (angle / 2.0).sin(),
                axis[1] * (angle / 2.0).sin(),
                axis[2] * (angle / 2.0).sin(),
            );
            assert_vectors_near(&dcm_to_ep(ep_to_dcm(ep)), &ep, "EP");

            let prv = axis * angle;
            assert_vectors_near(&dcm_to_prv(prv_to_dcm(prv)), &prv, "PRV");

            let outer = |rng: &mut StdRng| rng.gen_range(-PI + 1e-6..PI - 1e-6);
            let tilted = Vector3::new(
                outer(&mut rng),
                rng.gen_range(-PI / 2.0 + 0.01..PI / 2.0 - 0.01),
                outer(&mut rng),
            );
            assert_vectors_near(&dcm_to_euler321(euler321_to_dcm(tilted)), &tilted, "321");
            assert_vectors_near(&dcm_to_euler123(euler123_to_dcm(tilted)), &tilted, "123");

            let precessing = Vector3::new(
                outer(&mut rng),
                rng.gen_range(0.01..PI - 0.01),
                outer(&mut rng),
            );
            assert_vectors_near(
                &dcm_to_euler313(euler313_to_dcm(precessing)),
                &precessing,
                "313",
            );
        }
    }

    #[test]
    fn every_parameterization_round_trips_through_euler_parameters() {
        let mut rng = StdRng::seed_from_u64(0xe9);
        for _ in 0..SAMPLES {
            let axis = unit_vector(&mut rng);
            let sigma = axis * rng.gen_range(0.0..0.99);
            assert_vectors_near(&ep_to_mrp(mrp_to_ep(sigma)), &sigma, "MRP");
            let prv = axis * rng.gen_range(0.0..PI - 0.01);
            assert_vectors_near(&ep_to_prv(prv_to_ep(prv)), &prv, "PRV");

            let outer = |rng: &mut StdRng| rng.gen_range(-PI + 1e-6..PI - 1e-6);
            let tilted = Vector3::new(
                outer(&mut rng),
                rng.gen_range(-PI / 2.0 + 0.01..PI / 2.0 - 0.01),
                outer(&mut rng),
            );
            assert_vectors_near(&ep_to_euler321(euler321_to_ep(tilted)), &tilted, "321");
            assert_vectors_near(&ep_to_euler123(euler123_to_ep(tilted)), &tilted, "123");
            let precessing = Vector3::new(
                outer(&mut rng),
                rng.gen_range(0.01..PI - 0.01),
                outer(&mut rng),
            );
            assert_vectors_near(
                &ep_to_euler313(euler313_to_ep(precessing)),
                &precessing,
                "313",
            );
        }
    }

    #[test]
    fn conversions_agree_on_the_same_attitude() {
        let mut rng = StdRng::seed_from_u64(7);
        for _ in 0..SAMPLES {
            let sigma = unit_vector(&mut rng) * rng.gen_range(0.0..1.0);
            let dcm = mrp_to_dcm(sigma);
            let context = format!("sigma {sigma:?}");
            for other in [
                ep_to_dcm(mrp_to_ep(sigma)),
                prv_to_dcm(mrp_to_prv(sigma)),
                euler321_to_dcm(mrp_to_euler321(sigma)),
                euler123_to_dcm(mrp_to_euler123(sigma)),
                euler313_to_dcm(mrp_to_euler313(sigma)),
                mrp_to_dcm(prv_to_mrp(mrp_to_prv(sigma))),
                mrp_to_dcm(euler321_to_mrp(mrp_to_euler321(sigma))),
                mrp_to_dcm(euler123_to_mrp(mrp_to_euler123(sigma))),
                mrp_to_dcm(euler313_to_mrp(mrp_to_euler313(sigma))),
                prv_to_dcm(ep_to_prv(prv_to_ep(mrp_to_prv(sigma)))),
                euler321_to_dcm(prv_to_euler321(mrp_to_prv(sigma))),
                euler321_to_dcm(ep_to_euler321(euler321_to_ep(mrp_to_euler321(sigma)))),
                prv_to_dcm(euler321_to_prv(mrp_to_euler321(sigma))),
                prv_to_dcm(euler123_to_prv(mrp_to_euler123(sigma))),
                prv_to_dcm(euler313_to_prv(mrp_to_euler313(sigma))),
                euler123_to_dcm(prv_to_euler123(mrp_to_prv(sigma))),
                euler313_to_dcm(prv_to_euler313(mrp_to_prv(sigma))),
            ] {
                assert_vectors_near(&other, &dcm, &context);
            }
            assert!((dcm * dcm.transpose() - Matrix3::identity()).norm() < 1e-14);
            assert!((dcm.determinant() - 1.0).abs() < 1e-14);
        }
    }

    #[test]
    fn composition_matches_dcm_products_for_every_set() {
        let mut rng = StdRng::seed_from_u64(11);
        for _ in 0..SAMPLES {
            let first = unit_vector(&mut rng) * rng.gen_range(0.0..1.0);
            let second = unit_vector(&mut rng) * rng.gen_range(0.0..1.0);
            let (c1, c2) = (mrp_to_dcm(first), mrp_to_dcm(second));
            let context = format!("{first:?} {second:?}");
            let added = c2 * c1;
            let relative = c1 * c2.transpose();
            assert_vectors_near(&mrp_to_dcm(add_mrp(first, second)), &added, &context);
            assert_vectors_near(&mrp_to_dcm(sub_mrp(first, second)), &relative, &context);
            assert!(add_mrp(first, second).norm() <= 1.0 && sub_mrp(first, second).norm() <= 1.0);
            let (q1, q2) = (mrp_to_ep(first), mrp_to_ep(second));
            assert_vectors_near(&ep_to_dcm(add_ep(q1, q2)), &added, &context);
            assert_vectors_near(&ep_to_dcm(sub_ep(q1, q2)), &relative, &context);
        }
    }

    #[test]
    fn kinematic_matrices_invert_each_other() {
        let mut rng = StdRng::seed_from_u64(3);
        let identity = Matrix3::identity();
        for _ in 0..SAMPLES {
            let axis = unit_vector(&mut rng);
            let sigma = axis * rng.gen_range(0.0..1.5);
            assert_vectors_near(&(b_mat_mrp(sigma) * b_inv_mrp(sigma)), &identity, "MRP");
            let prv = axis * rng.gen_range(0.0..2.0 * PI - 0.1);
            assert_vectors_near(&(b_mat_prv(prv) * b_inv_prv(prv)), &identity, "PRV");
            let ep = mrp_to_ep(sigma);
            assert_vectors_near(&(b_inv_ep(ep) * b_mat_ep(ep)), &identity, "EP");

            let tilted = Vector3::new(
                rng.gen_range(-PI..PI),
                rng.gen_range(-1.4..1.4),
                rng.gen_range(-PI..PI),
            );
            let precessing = Vector3::new(
                rng.gen_range(-PI..PI),
                rng.gen_range(0.2..PI - 0.2),
                rng.gen_range(-PI..PI),
            );
            for (product, name) in [
                (b_mat_euler321(tilted) * b_inv_euler321(tilted), "321"),
                (b_mat_euler123(tilted) * b_inv_euler123(tilted), "123"),
                (
                    b_mat_euler313(precessing) * b_inv_euler313(precessing),
                    "313",
                ),
            ] {
                assert_vectors_near(&product, &identity, name);
            }
        }
    }

    #[test]
    fn mrp_differential_kinematics_round_trip() {
        let mut rng = StdRng::seed_from_u64(5);
        for _ in 0..SAMPLES {
            let sigma = unit_vector(&mut rng) * rng.gen_range(0.0..1.0);
            let omega = unit_vector(&mut rng) * rng.gen_range(0.0..0.5);
            let omega_dot = unit_vector(&mut rng) * rng.gen_range(0.0..0.1);
            let sigma_dot = d_omega_to_mrp(sigma, omega);
            assert_vectors_near(&d_mrp_to_omega(sigma, sigma_dot), &omega, "omega");
            let sigma_ddot = domega_to_dd_mrp(sigma, sigma_dot, omega, omega_dot);
            assert_vectors_near(
                &dd_mrp_to_domega(sigma, sigma_dot, sigma_ddot),
                &omega_dot,
                "omega_dot",
            );
            // [B(σ)] is quadratic in σ, so a central difference is exact up to rounding.
            let h = 1e-3;
            let difference =
                (b_mat_mrp(sigma + sigma_dot * h) - b_mat_mrp(sigma - sigma_dot * h)) / (2.0 * h);
            for (a, e) in b_dot_mat_mrp(sigma, sigma_dot)
                .iter()
                .zip(difference.iter())
            {
                assert_near(*a, *e, 1e-10, "BdotmatMRP");
            }
        }
    }

    #[test]
    fn mrp_switch_and_shadow_select_the_equivalent_set() {
        let sigma = Vector3::new(0.6, -0.7, 0.5);
        assert_eq!(mrp_switch(sigma, 1.0), mrp_shadow(sigma));
        assert_eq!(mrp_switch(sigma, 2.0), sigma);
        // The threshold is compared by magnitude, as upstream squares it.
        assert_eq!(mrp_switch(sigma, -2.0), sigma);
        assert_vectors_near(&mrp_to_dcm(mrp_shadow(sigma)), &mrp_to_dcm(sigma), "shadow");
        assert_eq!(mrp_shadow(mrp_shadow(sigma)), sigma);
    }

    #[test]
    fn exactly_cancelling_inputs_compose_to_zero() {
        let sigma = Vector3::new(0.3, -0.2, 0.1);
        assert_eq!(add_mrp(sigma, -sigma), Vector3::zeros());
        assert_eq!(sub_mrp(sigma, sigma), Vector3::zeros());
        let prv = Vector3::new(1.0, 2.0, -0.5);
        assert_eq!(add_prv(prv, -prv), Vector3::zeros());
        assert_eq!(sub_prv(prv, prv), Vector3::zeros());
        assert_eq!(prv_to_dcm(Vector3::zeros()), Matrix3::identity());
        assert_eq!(prv_to_elem(Vector3::zeros()), (0.0, Vector3::zeros()));
    }

    #[test]
    fn tilde_and_elementary_rotations_follow_upstream_conventions() {
        let a = Vector3::new(1.0, -2.0, 0.5);
        let b = Vector3::new(-0.3, 0.4, 2.0);
        assert_eq!(tilde(a) * b, a.cross(&b));
        assert_eq!(
            tilde(a),
            Matrix3::new(0.0, -0.5, -2.0, 0.5, 0.0, -1.0, 2.0, 1.0, 0.0)
        );
        assert_eq!(Axis::from_index(1), Some(Axis::X));
        assert_eq!(Axis::from_index(2), Some(Axis::Y));
        assert_eq!(Axis::from_index(3), Some(Axis::Z));
        assert_eq!(Axis::from_index(0), None);
        assert_eq!(Axis::from_index(4), None);
        let e = Vector3::new(0.4, -0.3, 1.2);
        assert_eq!(
            euler321_to_dcm(e),
            euler1(e[2]) * euler2(e[1]) * euler3(e[0])
        );
        assert_vectors_near(
            &euler313_to_dcm(e),
            &(euler3(e[2]) * euler1(e[1]) * euler3(e[0])),
            "313",
        );
        assert_vectors_near(
            &euler123_to_dcm(e),
            &(euler3(e[2]) * euler2(e[1]) * euler1(e[0])),
            "123",
        );
    }
}
