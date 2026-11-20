//! Vizard quad-map geometry: upstream `utilities/quadMapSupport.py`
//! (`vizSupport.qms`) and the two ellipsoid conversions of `vizSupport` it
//! uses. Pure geometry: the quad maps themselves are
//! `basilisk_rs::vizard::viz_structures::QuadMap`s, behind the `vizard`
//! feature.
//!
//! A reference ellipsoid is upstream's parent gravity body's `radEquator`
//! and `radiusRatio` (polar over equatorial radius), [`Ellipsoid`] here.
//! Vertices come as four planet-fixed corners per quad, as Vizard takes
//! them.

#![allow(dead_code)]

use nalgebra::{Matrix3, Vector3};

/// A planet's reference ellipsoid.
#[derive(Clone, Copy, Debug)]
pub struct Ellipsoid {
    /// Upstream `radEquator` \[m\].
    pub radius_equator_m: f64,
    /// Upstream `radiusRatio`: polar over equatorial radius.
    pub radius_ratio: f64,
}

/// A camera for [`compute_cam_fov_box`]: upstream's
/// `vizInterface.StdCameraSettings` branch, a camera at `position_b_m` on
/// the spacecraft looking at the planet's center.
#[derive(Clone, Copy, Debug)]
pub struct StdCamera {
    /// Upstream `position_B` \[m\].
    pub position_b_m: Vector3<f64>,
    /// Upstream `fieldOfView` \[rad\]: edge to edge.
    pub field_of_view_rad: f64,
}

impl Ellipsoid {
    /// Upstream `vizSupport.lla2fixedframe(lla_GP, radEquator, radRatio)`:
    /// planet-fixed position \[m\] of latitude, longitude \[rad\] and altitude
    /// \[m\] above the ellipsoid.
    ///
    /// # Panics
    ///
    /// If the latitude is outside `[-pi/2, pi/2]` (upstream `ValueError`).
    pub fn lla_to_fixed(
        &self,
        latitude_rad: f64,
        longitude_rad: f64,
        altitude_m: f64,
    ) -> Vector3<f64> {
        assert!(
            (-std::f64::consts::FRAC_PI_2..=std::f64::consts::FRAC_PI_2).contains(&latitude_rad),
            "Latitude must be between -pi/2 and pi/2 radians, not {latitude_rad}"
        );
        let e2 = 1.0 - self.radius_ratio * self.radius_ratio;
        let n = self.radius_equator_m / (1.0 - e2 * latitude_rad.sin().powi(2)).sqrt();
        Vector3::new(
            (n + altitude_m) * latitude_rad.cos() * longitude_rad.cos(),
            (n + altitude_m) * latitude_rad.cos() * longitude_rad.sin(),
            (n * (1.0 - e2) + altitude_m) * latitude_rad.sin(),
        )
    }

    /// Upstream `vizSupport.fixedframe2lla(r_GP_P, radEquator, radRatio)`:
    /// latitude, longitude \[rad\] and altitude \[m\] of a planet-fixed
    /// position, by Bowring's formula.
    pub fn fixed_to_lla(&self, r_gp_p_m: &Vector3<f64>) -> (f64, f64, f64) {
        let a = self.radius_equator_m;
        let b = a * self.radius_ratio;
        let e2 = 1.0 - (b / a).powi(2);
        let ep2 = (a * a - b * b) / (b * b);
        let r = r_gp_p_m.x.hypot(r_gp_p_m.y);
        let longitude = r_gp_p_m.y.atan2(r_gp_p_m.x);
        let theta = (r_gp_p_m.z * a).atan2(r * b);
        let latitude =
            (r_gp_p_m.z + ep2 * b * theta.sin().powi(3)).atan2(r - e2 * a * theta.cos().powi(3));
        let n = a / (1.0 - e2 * latitude.sin().powi(2)).sqrt();
        (latitude, longitude, r / latitude.cos() - n)
    }

    /// Whether `point` lies on the ellipsoid surface to `tolerance` (in the
    /// normalized quadric `x²/a² + y²/a² + z²/b²`).
    pub fn contains(&self, point: &Vector3<f64>, tolerance: f64) -> bool {
        let a = self.radius_equator_m;
        let b = a * self.radius_ratio;
        ((point.x / a).powi(2) + (point.y / a).powi(2) + (point.z / b).powi(2) - 1.0).abs()
            < tolerance
    }
}

/// `numpy.linspace(start, end, count)`.
fn linspace(start: f64, end: f64, count: usize) -> Vec<f64> {
    if count == 1 {
        return vec![start];
    }
    (0..count)
        .map(|index| start + (end - start) * index as f64 / (count - 1) as f64)
        .collect()
}

/// Upstream `computeRectMesh(parentBody, latBounds, lonBounds,
/// maxAngWidth)`: a latitude-longitude grid of quads on the ellipsoid
/// surface, no quad wider than `max_angular_width_deg` \[deg\]; bounds in
/// degrees.
pub fn compute_rect_mesh(
    ellipsoid: &Ellipsoid,
    latitude_bounds_deg: [f64; 2],
    longitude_bounds_deg: [f64; 2],
    max_angular_width_deg: f64,
) -> Vec<Vector3<f64>> {
    let grid = |[start, end]: [f64; 2]| {
        let count = ((end - start).abs() / max_angular_width_deg).ceil() as usize + 1;
        linspace(start, end, count)
            .into_iter()
            .map(f64::to_radians)
            .collect::<Vec<_>>()
    };
    let latitudes = grid(latitude_bounds_deg);
    let longitudes = grid(longitude_bounds_deg);
    let mut vertices = Vec::new();
    for lat in latitudes.windows(2) {
        for lon in longitudes.windows(2) {
            vertices.push(ellipsoid.lla_to_fixed(lat[0], lon[0], 0.0));
            vertices.push(ellipsoid.lla_to_fixed(lat[1], lon[0], 0.0));
            vertices.push(ellipsoid.lla_to_fixed(lat[1], lon[1], 0.0));
            vertices.push(ellipsoid.lla_to_fixed(lat[0], lon[1], 0.0));
        }
    }
    vertices
}

/// Upstream `computeCamFOVBox(parentBody, spiceObject, scObject, cam)` for a
/// standard camera: the planet-fixed points where the four corner rays of
/// the camera's square field of view, looking at the planet's center with
/// "up" towards the planet's +z, first meet the ellipsoid. Rays that miss
/// are left out, so a full box has four points.
///
/// `dcm_pn` is the planet's inertial-to-fixed DCM, `r_bp_n_m` the spacecraft
/// relative to the planet (inertial) and `dcm_bn` its attitude. (Upstream
/// uses the spacecraft's inertial position with the planet at the origin,
/// its `zeroBase = 'Earth'` case.)
pub fn compute_cam_fov_box(
    ellipsoid: &Ellipsoid,
    dcm_pn: &Matrix3<f64>,
    r_bp_n_m: &Vector3<f64>,
    dcm_bn: &Matrix3<f64>,
    camera: &StdCamera,
) -> Vec<Vector3<f64>> {
    let r_bp_p = dcm_pn * r_bp_n_m;
    let r_cb_p = dcm_pn * dcm_bn.transpose() * camera.position_b_m;
    let r_cp_p = r_bp_p + r_cb_p;
    let cp_1 = -r_cp_p / r_cp_p.norm();
    let side = cp_1.cross(&Vector3::z());
    let cp_2 = -side / side.norm();
    let cp_3 = cp_1.cross(&cp_2);
    let dcm_pc = Matrix3::from_columns(&[cp_1, cp_2, cp_3]);

    let half = (camera.field_of_view_rad / 2.0).tan();
    let a = ellipsoid.radius_equator_m;
    let b = ellipsoid.radius_ratio * a;
    [(half, half), (-half, half), (-half, -half), (half, -half)]
        .into_iter()
        .filter_map(|(y, z)| {
            let d = dcm_pc * Vector3::new(1.0, y, z);
            let quadratic_a = (d.x / a).powi(2) + (d.y / a).powi(2) + (d.z / b).powi(2);
            let quadratic_b = 2.0
                * (r_cp_p.x * d.x / (a * a) + r_cp_p.y * d.y / (a * a) + r_cp_p.z * d.z / (b * b));
            let quadratic_c =
                (r_cp_p.x / a).powi(2) + (r_cp_p.y / a).powi(2) + (r_cp_p.z / b).powi(2) - 1.0;
            let discriminant = quadratic_b * quadratic_b - 4.0 * quadratic_a * quadratic_c;
            if discriminant < 0.0 {
                return None;
            }
            let root = discriminant.sqrt();
            [
                (-quadratic_b + root) / (2.0 * quadratic_a),
                (-quadratic_b - root) / (2.0 * quadratic_a),
            ]
            .into_iter()
            .filter(|t| *t > 0.0)
            .reduce(f64::min)
            .map(|t| r_cp_p + t * d)
        })
        .collect()
}

/// Upstream `subdivideFOVBox(parentBody, FOVBox, fieldOfViewSubdivs)`: an
/// `n x n` mesh of quads interpolated bilinearly in latitude and longitude
/// between the four box corners (top-left, top-right, bottom-right,
/// bottom-left), drawn `1e-4` equatorial radii above the surface.
///
/// # Panics
///
/// If `fov_box` does not have four corners.
pub fn subdivide_fov_box(
    ellipsoid: &Ellipsoid,
    fov_box: &[Vector3<f64>],
    subdivisions: usize,
) -> Vec<Vector3<f64>> {
    assert_eq!(fov_box.len(), 4, "a full FOV box has four corners");
    let draw_height_m = 0.0001 * ellipsoid.radius_equator_m;
    let lla: Vec<(f64, f64, f64)> = fov_box
        .iter()
        .map(|corner| ellipsoid.fixed_to_lla(corner))
        .collect();
    let (top_left, top_right, bottom_right, bottom_left) = (lla[0], lla[1], lla[2], lla[3]);
    let n = subdivisions as f64;
    let mut vertices = Vec::new();
    for i in 0..subdivisions {
        let (i0, i1) = (i as f64 / n, (i + 1) as f64 / n);
        let edge = |top: f64, top_end: f64, bottom: f64, bottom_end: f64, fraction: f64| {
            linspace(
                bottom + (bottom_end - bottom) * fraction,
                top + (top_end - top) * fraction,
                subdivisions + 1,
            )
        };
        let left_lon = edge(top_left.1, top_right.1, bottom_left.1, bottom_right.1, i0);
        let right_lon = edge(top_left.1, top_right.1, bottom_left.1, bottom_right.1, i1);
        let left_lat = edge(top_left.0, top_right.0, bottom_left.0, bottom_right.0, i0);
        let right_lat = edge(top_left.0, top_right.0, bottom_left.0, bottom_right.0, i1);
        for j in 0..subdivisions {
            for (lat, lon) in [
                (left_lat[j], left_lon[j]),
                (right_lat[j], right_lon[j]),
                (right_lat[j + 1], right_lon[j + 1]),
                (left_lat[j + 1], left_lon[j + 1]),
            ] {
                vertices.push(ellipsoid.lla_to_fixed(lat, lon, draw_height_m));
            }
        }
    }
    vertices
}
