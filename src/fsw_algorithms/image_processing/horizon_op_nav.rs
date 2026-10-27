//! Spacecraft position from the limb of a planet in a camera image: port of
//! upstream Basilisk `horizonOpNav` (J. Christian's non-iterative horizon
//! navigation).
//!
//! Each limb point is mapped through the pinhole camera model to a line of
//! sight, rotated to the inertial frame and scaled by the planet's inverse
//! radii, so that the planet becomes the unit sphere. There every unit line of
//! sight `h_i` to the limb satisfies `h_i · n = 1`; a least-squares solve
//! (Gram-Schmidt QR decomposition and back substitution) gives `n`, and the
//! position follows as `r_C = -[B]⁻¹ n / √(n·n - 1)`. The covariance comes from
//! a pixel noise of `noise_sf · width / N` propagated through the same solve.
//! The position and covariance are published in the camera, inertial and body
//! frames, in metres.

use std::fmt;

use nalgebra::{Matrix3, MatrixXx3, SMatrix, SVector, Vector3};

use crate::messages::{
    CameraConfigMsg, Input, NavigationAttitudeMsg, OpNavLimbMsg, OpNavMsg, Output,
};
use crate::utilities::astro_constants::{REQ_EARTH, REQ_JUPITER, REQ_MARS, RP_EARTH, RP_MARS};
use crate::utilities::linear_algebra::{m33_inverse, normalize_or_zero};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream caps the limb points at `LINEAR_ALGEBRA_MAX_ARRAY_SIZE / 3`
/// (`64 * 64 / 3`), the size of its linear-algebra scratch buffers.
const MAX_SOLVED_LIMB_POINTS: usize = 64 * 64 / 3;

/// Upstream `alpha`: the camera skew, zero until the camera message provides
/// one (an upstream "to do").
const CAMERA_SKEW: f64 = 0.0;

/// User parameters of [`HorizonOpNav`]; the defaults are upstream's
/// zero-initialized ones.
#[derive(Clone, Debug)]
pub struct HorizonOpNavConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `planetTarget` [-]: planet whose limb is observed (Earth 1,
    /// Mars 2, Jupiter 3). A nonzero planet ID in a valid limb message
    /// replaces it, as upstream.
    pub planet_target: i32,
    /// Upstream `noiseSF` [-]: scale factor of the pixel noise.
    pub noise_sf: f64,
}

impl Default for HorizonOpNavConfig {
    fn default() -> Self {
        Self {
            name: "horizonOpNav".to_string(),
            planet_target: 0,
            noise_sf: 0.0,
        }
    }
}

/// Why an update publishes nothing. Upstream raises `BasiliskError` in these
/// cases, which aborts the update before the output is written.
#[derive(Clone, Copy, Debug, PartialEq)]
enum HorizonOpNavError {
    /// The limb message names a planet other than 1, 2 or 3.
    UnsupportedLimbPlanetId(f64),
    /// The planet target is not Earth (1), Mars (2) or Jupiter (3).
    UnsupportedPlanetTarget(i32),
    /// A valid limb message without limb points.
    NoLimbPoints,
}

impl fmt::Display for HorizonOpNavError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::UnsupportedLimbPlanetId(id) => write!(f, "unsupported limb planet ID {id}"),
            Self::UnsupportedPlanetTarget(target) => write!(
                f,
                "planetTarget must be Earth (1), Mars (2), or Jupiter (3), not {target}"
            ),
            Self::NoLimbPoints => write!(f, "numLimbPoints must be positive"),
        }
    }
}

/// Horizon-based optical navigation: upstream `horizonOpNav`.
#[derive(Clone, Debug)]
pub struct HorizonOpNav {
    /// Module configuration.
    pub config: HorizonOpNavConfig,
    /// Upstream `cameraConfigInMsg`: camera intrinsics and mounting.
    pub camera_config_in_msg: Input<CameraConfigMsg>,
    /// Upstream `attInMsg`: spacecraft attitude.
    pub att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `limbInMsg`: limb points found in the image.
    pub limb_in_msg: Input<OpNavLimbMsg>,
    /// Upstream `opNavOutMsg`: planet-relative position measurement.
    pub op_nav_out_msg: Output<OpNavMsg>,
}

impl HorizonOpNav {
    /// Builds the module with unconnected inputs.
    pub fn new(config: HorizonOpNavConfig) -> Self {
        Self {
            config,
            camera_config_in_msg: Input::default(),
            att_in_msg: Input::default(),
            limb_in_msg: Input::default(),
            op_nav_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_horizonOpNav`: check that every input is connected.
    pub fn reset(&mut self) {
        if !self.camera_config_in_msg.is_connected() {
            log::error!("horizonOpNav.camera_config_in_msg is not connected");
        }
        if !self.limb_in_msg.is_connected() {
            log::error!("horizonOpNav.limb_in_msg is not connected");
        }
        if !self.att_in_msg.is_connected() {
            log::error!("horizonOpNav.att_in_msg is not connected");
        }
    }

    /// Upstream `Update_horizonOpNav`: the measurement to publish.
    fn compute_measurement(&mut self) -> Result<OpNavMsg, HorizonOpNavError> {
        let camera = self.camera_config_in_msg.read();
        let limb = self.limb_in_msg.read();
        let navigation = self.att_in_msg.read();

        if !limb.valid {
            return Ok(OpNavMsg::default());
        }
        // A zero ID means "unspecified" and keeps the configured target.
        if limb.planet_id != 0.0 {
            if !(limb.planet_id == 1.0 || limb.planet_id == 2.0 || limb.planet_id == 3.0) {
                return Err(HorizonOpNavError::UnsupportedLimbPlanetId(limb.planet_id));
            }
            self.config.planet_target = limb.planet_id as i32;
        }
        let planet_target = self.config.planet_target;
        let (equatorial_radius_m, polar_radius_m) = planet_radii_m(planet_target)
            .ok_or(HorizonOpNavError::UnsupportedPlanetTarget(planet_target))?;
        let num_points = limb.num_limb_points.min(MAX_SOLVED_LIMB_POINTS);
        if num_points == 0 {
            return Err(HorizonOpNavError::NoLimbPoints);
        }

        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        let dcm_bn = mrp_to_dcm(navigation.sigma_bn);
        let dcm_nc = (dcm_cb * dcm_bn).transpose();
        // `[B] = [Q][NC]`: camera components to the inertial frame scaled by
        // the inverse radii (upstream's square-root shape matrix `Q`), where
        // the planet is the unit sphere.
        let shape = Matrix3::from_diagonal(&Vector3::new(
            1.0 / equatorial_radius_m,
            1.0 / equatorial_radius_m,
            1.0 / polar_radius_m,
        ));
        let dcm_b = shape * dcm_nc;

        let camera_model = PinholeCamera::new(&camera);
        let sigma_pix =
            self.config.noise_sf * f64::from(camera.resolution_px[0]) / num_points as f64;
        // Upstream scales both pixel axes by `d_x`.
        let pixel_variance = (sigma_pix * sigma_pix) / (camera_model.d_x * camera_model.d_x);
        let noise = dcm_b
            * Matrix3::from_diagonal(&Vector3::new(pixel_variance, pixel_variance, 0.0))
            * dcm_b.transpose();

        // Upstream `s_bar`: the scaled lines of sight, and `H`: their unit
        // vectors, one row per limb point.
        let lines_of_sight: Vec<Vector3<f64>> = limb.limb_points_px[..num_points]
            .iter()
            .map(|pixel| {
                dcm_b * (camera_model.pixel_to_camera * Vector3::new(pixel.x, pixel.y, 1.0))
            })
            .collect();
        let unit_rows: Vec<_> = lines_of_sight
            .iter()
            .map(|s| normalize_or_zero(*s).transpose())
            .collect();
        let h = MatrixXx3::from_rows(&unit_rows);

        // Least squares for H n = 1: R n = Qᵀ 1.
        let (q, r) = qr_decomposition(&h);
        let rhs = Vector3::from_fn(|i, _| q.column(i).iter().fold(0.0, |sum, x| sum + x));
        let n = back_substitution(&r, &rhs);

        let weights: Vec<f64> = lines_of_sight
            .iter()
            .map(|s| limb_point_weight(s, &n, &noise))
            .collect();
        let p_n = m33_inverse(&weighted_normal_matrix(&h, &weights)).unwrap_or_else(Matrix3::zeros);

        let n_norm2 = n.dot(&n);
        let scale_factor = -1.0 / (n_norm2 - 1.0).sqrt();
        let projector = Matrix3::identity() - (n * n.transpose()) * (1.0 / (n_norm2 - 1.0));
        let dcm_b_inv = m33_inverse(&dcm_b).unwrap_or_else(Matrix3::zeros);
        let position_c = (dcm_b_inv * n) * scale_factor;
        // Upstream's `F`, the Jacobian of the position with respect to `n`.
        let jacobian = (dcm_b_inv * projector.transpose()) * scale_factor;
        let covar_c = jacobian * p_n * jacobian.transpose();

        let position_n = dcm_nc * position_c;
        let covar_n = dcm_nc * covar_c * dcm_nc.transpose();
        Ok(OpNavMsg {
            time_tag_s: limb.time_tag_s,
            valid: true,
            covar_n_m2: covar_n,
            covar_b_m2: dcm_bn * covar_n * dcm_bn.transpose(),
            covar_c_m2: covar_c,
            position_bn_n_m: position_n,
            position_bn_b_m: dcm_bn * position_n,
            position_bn_c_m: position_c,
            planet_id: planet_target,
            fault_detected: false,
        })
    }
}

impl Module for HorizonOpNav {
    fn init(&mut self) {
        self.op_nav_out_msg.write(OpNavMsg::default());
        HorizonOpNav::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        HorizonOpNav::reset(self);
    }

    fn update(&mut self, _context: &SimulationContext) {
        match self.compute_measurement() {
            Ok(measurement) => self.op_nav_out_msg.write(measurement),
            // Upstream throws here, before writing: the output keeps its
            // previous value.
            Err(error) => log::error!("horizonOpNav: {error}"),
        }
    }
}

/// Equatorial and polar radii \[m\] of an upstream planet ID; Jupiter is
/// treated as a sphere.
fn planet_radii_m(planet_id: i32) -> Option<(f64, f64)> {
    match planet_id {
        1 => Some((REQ_EARTH * 1e3, RP_EARTH * 1e3)),
        2 => Some((REQ_MARS * 1e3, RP_MARS * 1e3)),
        3 => Some((REQ_JUPITER * 1e3, REQ_JUPITER * 1e3)),
        _ => None,
    }
}

/// The pinhole model of the camera message.
struct PinholeCamera {
    /// Upstream `d_x` \[pixel\]: pixels per unit focal-plane coordinate along x.
    d_x: f64,
    /// Upstream `tranf`: pixel `[u, v, 1]` to the camera-frame line of sight
    /// `[x, y, 1]` (eq. 8 of the journal paper).
    pixel_to_camera: Matrix3<f64>,
}

impl PinholeCamera {
    fn new(camera: &CameraConfigMsg) -> Self {
        let [width, height] = camera.resolution_px;
        let (width_px, height_px) = (f64::from(width), f64::from(height));
        // Sensor size over focal length is 2 tan(FOV / 2) per axis.
        let p_x = 2.0 * (camera.field_of_view_rad * width_px / height_px / 2.0).tan();
        let p_y = 2.0 * (camera.field_of_view_rad / 2.0).tan();
        let d_x = width_px / p_x;
        let d_y = height_px / p_y;
        // The principal point uses integer division, as upstream.
        let u_p = f64::from(width / 2);
        let v_p = f64::from(height / 2);
        #[rustfmt::skip]
        let pixel_to_camera = Matrix3::new(
            1.0 / d_x, -CAMERA_SKEW / (d_x * d_y), (CAMERA_SKEW * v_p - d_y * u_p) / (d_x * d_y),
            0.0, 1.0 / d_y, -v_p / d_y,
            0.0, 0.0, 1.0,
        );
        Self {
            d_x,
            pixel_to_camera,
        }
    }
}

/// Upstream `R_yInv[i][i]`: the inverse variance of `h_i · n` for the scaled
/// line of sight `s` (eqs. 31 and 50 of the journal paper).
fn limb_point_weight(s: &Vector3<f64>, n: &Vector3<f64>, noise: &Matrix3<f64>) -> f64 {
    let s_hat = normalize_or_zero(*s);
    let projector = Matrix3::identity() - s_hat * s_hat.transpose();
    let jacobian = projector.tr_mul(n) * (1.0 / s.norm());
    1.0 / noise.tr_mul(&jacobian).dot(&jacobian)
}

/// `Hᵀ W H` for the diagonal weights `W`. Upstream multiplies out the full
/// `N x N` matrix; only its diagonal contributes, and the `0.0 +` keeps
/// upstream's `+0` accumulator for a zero product.
fn weighted_normal_matrix(h: &MatrixXx3<f64>, weights: &[f64]) -> Matrix3<f64> {
    Matrix3::from_fn(|a, b| {
        h.row_iter().zip(weights).fold(0.0, |sum, (row, weight)| {
            sum + row[a] * (0.0 + weight * row[b])
        })
    })
}

/// Upstream `QRDecomp`: modified Gram-Schmidt QR decomposition of the
/// `N x 3` matrix `h = Q R`, returning `Q` (`N x 3`, orthonormal columns) and
/// the upper-triangular `R`. Sums run in row order, as upstream.
pub fn qr_decomposition(h: &MatrixXx3<f64>) -> (MatrixXx3<f64>, Matrix3<f64>) {
    let mut q = h.clone();
    let mut r = Matrix3::zeros();
    for i in 0..3 {
        for j in 0..i {
            let q_j = q.column(j).clone_owned();
            let mut q_i = q.column_mut(i);
            let r_ji = sequential_dot(q_i.iter(), q_j.iter());
            q_i.iter_mut()
                .zip(q_j.iter())
                .for_each(|(x, y)| *x -= r_ji * y);
            r[(j, i)] = r_ji;
        }
        let mut q_i = q.column_mut(i);
        let r_ii = sequential_dot(q_i.iter(), q_i.iter()).sqrt();
        let inverse_norm = 1.0 / r_ii;
        q_i.iter_mut().for_each(|x| *x *= inverse_norm);
        r[(i, i)] = r_ii;
    }
    (q, r)
}

/// Upstream `BackSub`: solves `R n = v` for the upper-triangular `R`.
pub fn back_substitution<const N: usize>(
    r: &SMatrix<f64, N, N>,
    v: &SVector<f64, N>,
) -> SVector<f64, N> {
    let mut n = SVector::<f64, N>::zeros();
    for i in (0..N).rev() {
        let sum = (i + 1..N).fold(0.0, |sum, j| sum + r[(i, j)] * n[j]);
        n[i] = (v[i] - sum) / r[(i, i)];
    }
    n
}

/// Upstream `vDot`: the dot product summed in order from zero.
fn sequential_dot<'a>(a: impl Iterator<Item = &'a f64>, b: impl Iterator<Item = &'a f64>) -> f64 {
    a.zip(b).fold(0.0, |sum, (x, y)| sum + x * y)
}

#[cfg(test)]
mod tests {
    //! Ports of `test_horizonOpNav.py`. Its `np.linalg.qr` reference is
    //! re-implemented as LAPACK's Householder QR (`dgeqrf`/`dorgqr`), whose
    //! sign convention the pytest relies on.

    use hifitime::Epoch;
    use nalgebra::{DMatrix, DVector, Matrix3, MatrixXx3, Vector2, Vector3};

    use crate::messages::{CameraConfigMsg, NavigationAttitudeMsg, OpNavLimbMsg, OpNavMsg, Output};
    use crate::utilities::astro_constants::REQ_JUPITER;
    use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
    use crate::{Module, SimulationContext};

    use super::{
        HorizonOpNav, HorizonOpNavConfig, HorizonOpNavError, back_substitution, qr_decomposition,
    };

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The pytest's `back_substitution`, including its sum starting at
    /// `j = i` (harmless, `x[i]` is still zero there).
    fn python_back_substitution(a: &DMatrix<f64>, b: &DVector<f64>) -> DVector<f64> {
        let n = b.len();
        let mut x = DVector::zeros(n);
        x[n - 1] = b[n - 1] / a[(n - 1, n - 1)];
        for i in (0..n - 1).rev() {
            let sum: f64 = (i..n).map(|j| a[(i, j)] * x[j]).sum();
            x[i] = (b[i] - sum) / a[(i, i)];
        }
        x
    }

    /// The pytest's classical Gram-Schmidt QR.
    fn python_gram_schmidt(h: &DMatrix<f64>) -> (DMatrix<f64>, DMatrix<f64>) {
        let rows = h.nrows();
        let mut q = DMatrix::zeros(rows, 3);
        let mut r = DMatrix::zeros(3, 3);
        for i in 0..3 {
            q.set_column(i, &h.column(i));
            for j in 0..i {
                r[(j, i)] = q.column(j).dot(&h.column(i));
                let update = q.column(j) * r[(j, i)];
                let column = q.column(i) - update;
                q.set_column(i, &column);
            }
            r[(i, i)] = q.column(i).norm();
            let column = q.column(i) / r[(i, i)];
            q.set_column(i, &column);
        }
        (q, r)
    }

    /// `np.linalg.qr` (reduced): LAPACK's Householder QR, where each
    /// reflector maps its pivot column to `beta = -sign(alpha) ‖x‖`.
    fn lapack_householder_qr(h: &DMatrix<f64>) -> (DMatrix<f64>, DMatrix<f64>) {
        let (rows, cols) = h.shape();
        let mut a = h.clone();
        let mut reflectors = Vec::with_capacity(cols);
        for k in 0..cols {
            let alpha = a[(k, k)];
            let x_norm = a.view((k + 1, k), (rows - k - 1, 1)).norm();
            let mut v = DVector::zeros(rows);
            v[k] = 1.0;
            if x_norm == 0.0 {
                reflectors.push((v, 0.0));
                continue;
            }
            let beta = -alpha.signum() * alpha.hypot(x_norm);
            let tau = (beta - alpha) / beta;
            for i in k + 1..rows {
                v[i] = a[(i, k)] / (alpha - beta);
            }
            let projection = v.transpose() * &a;
            a -= tau * &v * projection;
            reflectors.push((v, tau));
        }
        let mut q = DMatrix::identity(rows, cols);
        for (v, tau) in reflectors.iter().rev() {
            let projection = v.transpose() * &q;
            q -= *tau * v * projection;
        }
        (q, a.rows(0, cols).upper_triangle())
    }

    /// Port of `horizonOpNav_methods`: `QRDecomp` against the pytest's
    /// Gram-Schmidt and against Householder QR (whose `Q` and `R` are the
    /// negatives for this input), and `BackSub` against the pytest's
    /// back substitution, all to 1e-10.
    #[test]
    fn matches_upstream_qr_decomposition_and_back_substitution() {
        #[rustfmt::skip]
        let h_input = DMatrix::from_row_slice(6, 3, &[
            1.0, 2.0, 3.0,
            1.0, 20.0, 3.0,
            3.0, 0.0, 1.0,
            2.0, 1.0, 0.0,
            20.0, -1.0, -5.0,
            0.0, 10.0, -5.0,
        ]);
        let h = MatrixXx3::from_iterator(6, h_input.iter().copied());
        let (q_test, r_test) = qr_decomposition(&h);
        let q_test = DMatrix::from_iterator(6, 3, q_test.iter().copied());
        let r_test = DMatrix::from_iterator(3, 3, r_test.iter().copied());

        let (q_py, r_py) = python_gram_schmidt(&h_input);
        assert!((&q_py - &q_test).norm() <= 1e-10, "Q: {q_test} != {q_py}");
        assert!((&r_py - &r_test).norm() <= 1e-10, "R: {r_test} != {r_py}");

        let (q_np, r_np) = lapack_householder_qr(&h_input);
        assert!((&q_np + &q_test).norm() <= 1e-10, "Q: {q_test} != -{q_np}");
        assert!((&r_np + &r_test).norm() <= 1e-10, "R: {r_test} != -{r_np}");

        let v = DVector::from_element(3, 1.0);
        let expected = python_back_substitution(&r_np, &v);
        let r_np_fixed = Matrix3::from_iterator(r_np.iter().copied());
        let back_sub = back_substitution(&r_np_fixed, &Vector3::repeat(1.0));
        assert!(
            (DVector::from_iterator(3, back_sub.iter().copied()) - &expected).norm() <= 1e-10,
            "BackSub: {back_sub} != {expected}"
        );
    }

    /// The limb points of the upstream pytest, `(x, y)` in pixels.
    #[rustfmt::skip]
    const UPSTREAM_LIMB_POINTS: [[f64; 2]; 384] = [
        [226.0, 113.0], [227.0, 113.0], [223.0, 114.0], [224.0, 114.0], [225.0, 114.0],
        [219.0, 115.0], [220.0, 115.0], [221.0, 115.0], [222.0, 115.0], [215.0, 116.0],
        [216.0, 116.0], [217.0, 116.0], [218.0, 116.0], [212.0, 117.0], [213.0, 117.0],
        [214.0, 117.0], [209.0, 118.0], [210.0, 118.0], [211.0, 118.0], [205.0, 119.0],
        [206.0, 119.0], [207.0, 119.0], [208.0, 119.0], [204.0, 120.0], [205.0, 120.0],
        [201.0, 121.0], [202.0, 121.0], [203.0, 121.0], [199.0, 122.0], [200.0, 122.0],
        [197.0, 123.0], [198.0, 123.0], [195.0, 124.0], [196.0, 124.0], [193.0, 125.0],
        [194.0, 125.0], [191.0, 126.0], [192.0, 126.0], [189.0, 127.0], [190.0, 127.0],
        [187.0, 128.0], [188.0, 128.0], [185.0, 129.0], [186.0, 129.0], [183.0, 130.0],
        [184.0, 130.0], [181.0, 131.0], [182.0, 131.0], [180.0, 132.0], [181.0, 132.0],
        [178.0, 133.0], [179.0, 133.0], [177.0, 134.0], [178.0, 134.0], [175.0, 135.0],
        [176.0, 135.0], [174.0, 136.0], [175.0, 136.0], [172.0, 137.0], [173.0, 137.0],
        [171.0, 138.0], [172.0, 138.0], [170.0, 139.0], [171.0, 139.0], [168.0, 140.0],
        [169.0, 140.0], [167.0, 141.0], [168.0, 141.0], [166.0, 142.0], [167.0, 142.0],
        [164.0, 143.0], [165.0, 143.0], [163.0, 144.0], [164.0, 144.0], [162.0, 145.0],
        [163.0, 145.0], [161.0, 146.0], [162.0, 146.0], [160.0, 147.0], [161.0, 147.0],
        [159.0, 148.0], [160.0, 148.0], [158.0, 149.0], [159.0, 149.0], [156.0, 150.0],
        [157.0, 150.0], [155.0, 151.0], [156.0, 151.0], [154.0, 152.0], [155.0, 152.0],
        [153.0, 153.0], [154.0, 153.0], [153.0, 154.0], [152.0, 155.0], [151.0, 156.0],
        [152.0, 156.0], [150.0, 157.0], [151.0, 157.0], [149.0, 158.0], [150.0, 158.0],
        [148.0, 159.0], [149.0, 159.0], [147.0, 160.0], [148.0, 160.0], [146.0, 161.0],
        [147.0, 161.0], [145.0, 162.0], [146.0, 162.0], [145.0, 163.0], [144.0, 164.0],
        [143.0, 165.0], [144.0, 165.0], [142.0, 166.0], [143.0, 166.0], [142.0, 167.0],
        [141.0, 168.0], [140.0, 169.0], [141.0, 169.0], [139.0, 170.0], [140.0, 170.0],
        [139.0, 171.0], [138.0, 172.0], [137.0, 173.0], [138.0, 173.0], [137.0, 174.0],
        [136.0, 175.0], [135.0, 176.0], [136.0, 176.0], [135.0, 177.0], [134.0, 178.0],
        [133.0, 179.0], [134.0, 179.0], [133.0, 180.0], [132.0, 181.0], [132.0, 182.0],
        [131.0, 183.0], [131.0, 184.0], [130.0, 185.0], [129.0, 186.0], [130.0, 186.0],
        [129.0, 187.0], [128.0, 188.0], [128.0, 189.0], [127.0, 190.0], [127.0, 191.0],
        [126.0, 192.0], [126.0, 193.0], [125.0, 194.0], [125.0, 195.0], [125.0, 196.0],
        [124.0, 197.0], [124.0, 198.0], [123.0, 199.0], [123.0, 200.0], [122.0, 201.0],
        [122.0, 202.0], [122.0, 203.0], [121.0, 204.0], [120.0, 205.0], [121.0, 205.0],
        [120.0, 206.0], [120.0, 207.0], [120.0, 208.0], [119.0, 209.0], [119.0, 210.0],
        [119.0, 211.0], [118.0, 212.0], [118.0, 213.0], [118.0, 214.0], [117.0, 215.0],
        [117.0, 216.0], [117.0, 217.0], [117.0, 218.0], [116.0, 219.0], [116.0, 220.0],
        [116.0, 221.0], [116.0, 222.0], [115.0, 223.0], [115.0, 224.0], [115.0, 225.0],
        [115.0, 226.0], [114.0, 227.0], [114.0, 228.0], [114.0, 229.0], [114.0, 230.0],
        [114.0, 231.0], [114.0, 232.0], [113.0, 233.0], [113.0, 234.0], [113.0, 235.0],
        [113.0, 236.0], [113.0, 237.0], [113.0, 238.0], [113.0, 239.0], [112.0, 240.0],
        [112.0, 241.0], [112.0, 242.0], [112.0, 243.0], [112.0, 244.0], [112.0, 245.0],
        [112.0, 246.0], [112.0, 247.0], [112.0, 248.0], [112.0, 249.0], [112.0, 250.0],
        [112.0, 251.0], [112.0, 252.0], [112.0, 253.0], [112.0, 254.0], [111.0, 255.0],
        [111.0, 256.0], [112.0, 257.0], [112.0, 258.0], [112.0, 259.0], [112.0, 260.0],
        [112.0, 261.0], [112.0, 262.0], [112.0, 263.0], [112.0, 264.0], [112.0, 265.0],
        [112.0, 266.0], [112.0, 267.0], [112.0, 268.0], [112.0, 269.0], [112.0, 270.0],
        [112.0, 271.0], [113.0, 272.0], [113.0, 273.0], [113.0, 274.0], [113.0, 275.0],
        [113.0, 276.0], [113.0, 277.0], [113.0, 278.0], [114.0, 279.0], [114.0, 280.0],
        [114.0, 281.0], [114.0, 282.0], [114.0, 283.0], [114.0, 284.0], [115.0, 285.0],
        [115.0, 286.0], [115.0, 287.0], [115.0, 288.0], [116.0, 289.0], [116.0, 290.0],
        [116.0, 291.0], [116.0, 292.0], [117.0, 293.0], [117.0, 294.0], [117.0, 295.0],
        [117.0, 296.0], [118.0, 297.0], [118.0, 298.0], [118.0, 299.0], [119.0, 300.0],
        [119.0, 301.0], [119.0, 302.0], [120.0, 303.0], [120.0, 304.0], [120.0, 305.0],
        [121.0, 306.0], [121.0, 307.0], [122.0, 308.0], [122.0, 309.0], [122.0, 310.0],
        [123.0, 311.0], [123.0, 312.0], [124.0, 313.0], [124.0, 314.0], [125.0, 315.0],
        [125.0, 316.0], [125.0, 317.0], [126.0, 318.0], [126.0, 319.0], [127.0, 320.0],
        [127.0, 321.0], [128.0, 322.0], [128.0, 323.0], [129.0, 324.0], [129.0, 325.0],
        [130.0, 325.0], [130.0, 326.0], [131.0, 327.0], [131.0, 328.0], [132.0, 329.0],
        [132.0, 330.0], [133.0, 331.0], [133.0, 332.0], [134.0, 332.0], [134.0, 333.0],
        [135.0, 334.0], [135.0, 335.0], [136.0, 335.0], [136.0, 336.0], [137.0, 337.0],
        [137.0, 338.0], [138.0, 338.0], [138.0, 339.0], [139.0, 340.0], [139.0, 341.0],
        [140.0, 341.0], [140.0, 342.0], [141.0, 342.0], [141.0, 343.0], [142.0, 344.0],
        [142.0, 345.0], [143.0, 345.0], [143.0, 346.0], [144.0, 346.0], [144.0, 347.0],
        [145.0, 348.0], [145.0, 349.0], [146.0, 349.0], [146.0, 350.0], [147.0, 350.0],
        [147.0, 351.0], [148.0, 351.0], [148.0, 352.0], [149.0, 352.0], [149.0, 353.0],
        [150.0, 353.0], [150.0, 354.0], [151.0, 354.0], [151.0, 355.0], [152.0, 356.0],
        [152.0, 357.0], [153.0, 357.0], [153.0, 358.0], [154.0, 358.0], [154.0, 359.0],
        [155.0, 359.0], [155.0, 360.0], [156.0, 360.0], [156.0, 361.0], [157.0, 361.0],
        [158.0, 362.0], [159.0, 362.0], [159.0, 363.0], [160.0, 363.0], [160.0, 364.0],
        [161.0, 364.0], [161.0, 365.0], [162.0, 365.0], [162.0, 366.0], [163.0, 366.0],
        [163.0, 367.0], [164.0, 367.0], [164.0, 368.0], [165.0, 368.0], [166.0, 369.0],
        [167.0, 369.0], [167.0, 370.0], [168.0, 370.0], [168.0, 371.0], [169.0, 371.0],
        [169.0, 372.0], [170.0, 372.0], [171.0, 373.0], [172.0, 373.0], [172.0, 374.0],
        [173.0, 374.0], [174.0, 375.0], [175.0, 375.0], [175.0, 376.0], [176.0, 376.0],
        [177.0, 377.0], [178.0, 377.0], [178.0, 378.0], [179.0, 378.0], [180.0, 379.0],
        [181.0, 379.0], [181.0, 380.0], [182.0, 380.0], [183.0, 381.0], [184.0, 381.0],
        [185.0, 382.0], [186.0, 382.0], [187.0, 383.0], [188.0, 383.0], [188.0, 384.0],
        [189.0, 384.0], [190.0, 385.0], [191.0, 385.0], [192.0, 386.0],
    ];

    fn upstream_camera() -> CameraConfigMsg {
        CameraConfigMsg {
            field_of_view_rad: 2.0 * (10.0e-3 / 2.0 / 1.0_f64).atan(),
            resolution_px: [512, 512],
            sigma_cb: Vector3::new(1.0, 0.2, 0.3),
            ..Default::default()
        }
    }

    fn upstream_attitude() -> NavigationAttitudeMsg {
        NavigationAttitudeMsg {
            sigma_bn: Vector3::new(0.6, 1.0, 0.1),
            ..Default::default()
        }
    }

    fn limb_message(points: &[[f64; 2]], planet_id: f64) -> OpNavLimbMsg {
        let mut limb = OpNavLimbMsg {
            time_tag_s: 12345.0,
            valid: true,
            num_limb_points: points.len(),
            planet_id,
            ..Default::default()
        };
        for (slot, point) in limb.limb_points_px.iter_mut().zip(points) {
            *slot = Vector2::from(*point);
        }
        limb
    }

    fn connected_module(
        config: HorizonOpNavConfig,
        camera: &Output<CameraConfigMsg>,
        attitude: &Output<NavigationAttitudeMsg>,
        limb: &Output<OpNavLimbMsg>,
    ) -> HorizonOpNav {
        let mut module = HorizonOpNav::new(config);
        camera.connect_to(&mut module.camera_config_in_msg);
        attitude.connect_to(&mut module.att_in_msg);
        limb.connect_to(&mut module.limb_in_msg);
        module.init();
        module
    }

    /// Internal checks of the pytest's truth computation.
    struct TruthChecks {
        /// `‖Q R - H‖` of the pytest's Gram-Schmidt.
        qr_residual: f64,
        /// `‖R⁻¹ RHS - n‖`.
        back_sub_error: f64,
    }

    /// The pytest's truth for `r_BN_C` and `covar_C`, with Mars hard-coded.
    fn upstream_truth(
        camera: &CameraConfigMsg,
        attitude: &NavigationAttitudeMsg,
        points: &[[f64; 2]],
        noise_sf: f64,
    ) -> (Vector3<f64>, Matrix3<f64>, TruthChecks) {
        let q = Matrix3::from_diagonal(&Vector3::new(
            1.0 / (3396.19 * 1e3),
            1.0 / (3396.19 * 1e3),
            1.0 / (3376.2 * 1e3),
        ));
        let num_points = points.len();
        let cn = mrp_to_dcm(camera.sigma_cb) * mrp_to_dcm(attitude.sigma_bn);
        let b = q * cn.transpose();

        let alpha = 0.0;
        let [width, height] = camera.resolution_px.map(f64::from);
        // Python 3 `/` is float division.
        let (up, vp) = (width / 2.0, height / 2.0);
        let p_x = 2.0 * (camera.field_of_view_rad * width / height / 2.0).tan();
        let p_y = 2.0 * (camera.field_of_view_rad / 2.0).tan();
        let d_x = width / p_x;
        let d_y = height / p_y;
        let transf = Matrix3::new(
            1.0 / d_x,
            -alpha / (d_x * d_y),
            (alpha * vp - d_y * up) / (d_x * d_y),
            0.0,
            1.0 / d_y,
            -vp / d_y,
            0.0,
            0.0,
            1.0,
        );

        let s_bar: Vec<Vector3<f64>> = points
            .iter()
            .map(|p| b * (transf * Vector3::new(p[0], p[1], 1.0)))
            .collect();
        let s_bar_prime: Vec<Vector3<f64>> = s_bar.iter().map(|s| s / s.norm()).collect();
        let h = DMatrix::from_fn(num_points, 3, |i, j| s_bar_prime[i][j]);

        let (q_py, r_py) = python_gram_schmidt(&h);
        let qr_residual = (&q_py * &r_py - &h).norm();
        let rhs = q_py.transpose() * DVector::from_element(num_points, 1.0);
        let n_dyn = python_back_substitution(&r_py, &rhs);
        let n_test = r_py.clone().try_inverse().expect("R is invertible") * &rhs;
        let back_sub_error = (&n_test - &n_dyn).norm();
        let n = Vector3::new(n_dyn[0], n_dyn[1], n_dyn[2]);

        let sigma = noise_sf * width / num_points as f64;
        let r_s = b
            * (sigma.powi(2) / d_x.powi(2) * Matrix3::from_diagonal(&Vector3::new(1.0, 1.0, 0.0)))
            * b.transpose();
        let mut r_y_inv = DMatrix::zeros(num_points, num_points);
        for i in 0..num_points {
            let projector = Matrix3::identity() - s_bar_prime[i] * s_bar_prime[i].transpose();
            let j = (n.transpose() * projector).transpose() / s_bar[i].norm();
            let temp = r_s * j;
            r_y_inv[(i, i)] = 1.0 / temp.dot(&j);
        }
        let information = h.transpose() * &r_y_inv * &h;
        let p_n = information
            .try_inverse()
            .expect("information is invertible");
        let p_n = Matrix3::from_iterator(p_n.iter().copied());
        let n_n = n.dot(&n);
        let b_inv = b.try_inverse().expect("B is invertible");
        let f = -(n_n - 1.0).powf(-0.5)
            * b_inv
            * (Matrix3::identity() - n * n.transpose() / (n_n - 1.0));
        let covar_c = f * p_n * f.transpose();
        let r_bn_c = -(n_n - 1.0).powf(-0.5) * (b_inv * n);
        (
            r_bn_c,
            covar_c,
            TruthChecks {
                qr_residual,
                back_sub_error,
            },
        )
    }

    /// `np.testing.assert_allclose(actual, desired, rtol)` with `atol = 0`.
    fn assert_allclose<'a>(
        actual: impl IntoIterator<Item = &'a f64>,
        desired: impl IntoIterator<Item = &'a f64>,
        rtol: f64,
        label: &str,
    ) {
        for (a, d) in actual.into_iter().zip(desired) {
            assert!((a - d).abs() <= rtol * d.abs(), "{label}: {a} != {d}");
        }
    }

    /// Port of `test_horizonOpNav` (its `horizonOpNav_update` part) for every
    /// `(configuredPlanet, messagePlanet)` parametrization: both updates (0
    /// and 0.5 s) against the pytest's truth, position to rtol 1e-3 and
    /// covariance to rtol 1e-5, with Mars as the resulting target.
    #[test]
    fn matches_upstream_horizon_op_nav_update() {
        let camera = upstream_camera();
        let attitude = upstream_attitude();
        let noise_sf = 2.0;
        let (r_bn_c, covar_c, checks) =
            upstream_truth(&camera, &attitude, &UPSTREAM_LIMB_POINTS, noise_sf);
        assert!(
            checks.qr_residual <= 1e-8,
            "truth QR {}",
            checks.qr_residual
        );
        assert!(
            checks.back_sub_error <= 1e-8,
            "truth back sub {}",
            checks.back_sub_error
        );

        for (configured_planet, message_planet) in [(2, 0.0), (0, 2.0), (1, 2.0)] {
            let camera_out = Output::new(camera.clone());
            let attitude_out = Output::new(attitude.clone());
            let limb_out = Output::new(limb_message(&UPSTREAM_LIMB_POINTS, message_planet));
            let mut module = connected_module(
                HorizonOpNavConfig {
                    planet_target: configured_planet,
                    noise_sf,
                    ..Default::default()
                },
                &camera_out,
                &attitude_out,
                &limb_out,
            );
            for time_nanos in [0, 500_000_000] {
                module.update(&context(time_nanos));
                let output = module.op_nav_out_msg.read();
                let label = format!("({configured_planet}, {message_planet}) at {time_nanos} ns");
                assert_eq!(output.planet_id, 2, "{label}");
                assert!(
                    output.position_bn_c_m.iter().all(|x| x.is_finite()),
                    "{label}"
                );
                assert!(output.covar_c_m2.iter().all(|x| x.is_finite()), "{label}");
                assert_allclose(&output.position_bn_c_m, &r_bn_c, 1e-3, &label);
                assert_allclose(&output.covar_c_m2, &covar_c, 1e-5, &label);
            }
            assert_eq!(module.config.planet_target, 2);
        }
    }

    /// Port of `test_horizonOpNav_rejects_unknown_planet`: every
    /// parametrization fails the update with a planet error (upstream raises
    /// `BasiliskError` matching `horizonOpNav:.*planet`) and publishes
    /// nothing.
    #[test]
    fn rejects_unsupported_planets() {
        for (configured_planet, message_planet) in
            [(0, 0.0), (4, 0.0), (2, -1.0), (2, 4.0), (2, f64::NAN)]
        {
            let camera = Output::new(CameraConfigMsg::default());
            let attitude = Output::new(NavigationAttitudeMsg::default());
            let limb = Output::new(OpNavLimbMsg {
                valid: true,
                planet_id: message_planet,
                ..Default::default()
            });
            let mut module = connected_module(
                HorizonOpNavConfig {
                    planet_target: configured_planet,
                    ..Default::default()
                },
                &camera,
                &attitude,
                &limb,
            );
            let error = module
                .compute_measurement()
                .expect_err("an unsupported planet must fail");
            let label = format!("({configured_planet}, {message_planet})");
            if message_planet == 0.0 {
                assert_eq!(
                    error,
                    HorizonOpNavError::UnsupportedPlanetTarget(configured_planet),
                    "{label}"
                );
            } else {
                assert!(
                    matches!(error, HorizonOpNavError::UnsupportedLimbPlanetId(_)),
                    "{label}: {error:?}"
                );
                // The target is left alone, as upstream throws first.
                assert_eq!(module.config.planet_target, configured_planet, "{label}");
            }
            assert!(error.to_string().contains("planet"), "{label}: {error}");

            module.update(&context(0));
            assert_eq!(module.op_nav_out_msg.read(), OpNavMsg::default(), "{label}");
        }
    }

    #[test]
    fn publishes_an_invalid_measurement_for_an_invalid_limb() {
        let camera = Output::new(upstream_camera());
        let attitude = Output::new(upstream_attitude());
        let mut limb_msg = limb_message(&UPSTREAM_LIMB_POINTS, 2.0);
        limb_msg.valid = false;
        let limb = Output::new(limb_msg);
        let mut module = connected_module(
            HorizonOpNavConfig {
                noise_sf: 2.0,
                ..Default::default()
            },
            &camera,
            &attitude,
            &limb,
        );
        // A previous valid measurement is replaced by the zeroed one.
        module.op_nav_out_msg.write(OpNavMsg {
            valid: true,
            ..Default::default()
        });
        module.update(&context(0));
        assert_eq!(module.op_nav_out_msg.read(), OpNavMsg::default());
        // The invalid image returns before the planet ID is read.
        assert_eq!(module.config.planet_target, 0);
    }

    #[test]
    fn rejects_a_limb_without_points() {
        let camera = Output::new(upstream_camera());
        let attitude = Output::new(upstream_attitude());
        let limb = Output::new(limb_message(&[], 2.0));
        let mut module = connected_module(HorizonOpNavConfig::default(), &camera, &attitude, &limb);
        assert_eq!(
            module.compute_measurement(),
            Err(HorizonOpNavError::NoLimbPoints)
        );
    }

    /// Noise-free limb points of a sphere (Jupiter) seen from a known
    /// position: the module recovers the position exactly in every frame.
    #[test]
    fn recovers_the_position_from_an_exact_spherical_limb() {
        let camera = CameraConfigMsg {
            field_of_view_rad: 20.0_f64.to_radians(),
            resolution_px: [1024, 768],
            sigma_cb: Vector3::new(0.1, -0.2, 0.05),
            ..Default::default()
        };
        let attitude = NavigationAttitudeMsg {
            sigma_bn: Vector3::new(-0.3, 0.1, 0.2),
            ..Default::default()
        };
        // Spacecraft relative to Jupiter, camera components: the planet is
        // in front of the camera, slightly off the boresight.
        let position_c = Vector3::new(0.04_f64, -0.03, -1.0) * 1.2e9;
        let radius_m = REQ_JUPITER * 1e3;

        // Unit lines of sight tangent to the sphere, around the planet centre.
        let center = (-position_c).normalize();
        let half_angle = (radius_m / position_c.norm()).asin();
        let across = center.cross(&Vector3::x()).normalize();
        let along = center.cross(&across);
        let [width, height] = camera.resolution_px.map(f64::from);
        let d_x = width / (2.0 * (camera.field_of_view_rad * width / height / 2.0).tan());
        let d_y = height / (2.0 * (camera.field_of_view_rad / 2.0).tan());
        let points: Vec<[f64; 2]> = (0..90)
            .map(|k| {
                let azimuth = f64::from(k) * 4.0_f64.to_radians();
                let direction = half_angle.cos() * center
                    + half_angle.sin() * (azimuth.cos() * across + azimuth.sin() * along);
                [
                    d_x * direction.x / direction.z + width / 2.0,
                    d_y * direction.y / direction.z + height / 2.0,
                ]
            })
            .collect();

        let camera_out = Output::new(camera.clone());
        let attitude_out = Output::new(attitude.clone());
        let limb = Output::new(limb_message(&points, 3.0));
        let mut module = connected_module(
            HorizonOpNavConfig {
                noise_sf: 1.0,
                ..Default::default()
            },
            &camera_out,
            &attitude_out,
            &limb,
        );
        module.update(&context(0));
        let output = module.op_nav_out_msg.read();

        let dcm_cb = mrp_to_dcm(camera.sigma_cb);
        let dcm_bn = mrp_to_dcm(attitude.sigma_bn);
        let position_n = (dcm_cb * dcm_bn).transpose() * position_c;
        let tolerance = 1e-9 * position_c.norm();
        assert!(output.valid);
        assert_eq!(output.planet_id, 3);
        assert_eq!(output.time_tag_s, 12345.0);
        assert!((output.position_bn_c_m - position_c).norm() < tolerance);
        assert!((output.position_bn_n_m - position_n).norm() < tolerance);
        assert!((output.position_bn_b_m - dcm_bn * position_n).norm() < tolerance);
        // The covariance is symmetric and consistent across frames.
        let covar_c = output.covar_c_m2;
        assert!((covar_c - covar_c.transpose()).norm() <= 1e-9 * covar_c.norm());
        let dcm_nc = (dcm_cb * dcm_bn).transpose();
        assert!(
            (output.covar_n_m2 - dcm_nc * covar_c * dcm_nc.transpose()).norm()
                <= 1e-9 * covar_c.norm()
        );
    }
}
