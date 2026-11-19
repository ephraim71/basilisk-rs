//! Planet radiation patch grid: port of upstream Basilisk
//! `simulation/environment/_GeneralModuleFiles/planetRadiationBase`, the
//! surface model shared by [`super::albedo`] and
//! [`super::earth_radiation_model`].
//!
//! A planet is split into latitude/longitude patches. Each patch reflects
//! sunlight (albedo) and emits infrared as a Lambertian surface, and
//! [`PlanetGrid::compute_patches`] returns, per patch visible from a point,
//! the fluxes that patch delivers there. An oblate planet is mapped onto its
//! authalic (equal-area) sphere, with authalic latitudes for the patch areas.
//!
//! Upstream's base class also carries the module plumbing (message reading
//! and the planet loop); here each module does its own, and this file keeps
//! the shared numerics.

use std::f64::consts::{FRAC_1_PI, PI};
use std::fmt;
use std::path::PathBuf;

use nalgebra::{Matrix3, Vector3};

use crate::messages::PlanetStateMsg;
use crate::utilities::astro_constants::{AU2M, REQ_SUN, SOLAR_FLUX_EARTH};
use crate::utilities::linear_algebra::{safe_acos, safe_asin};

/// Flux and geometry of one patch seen from a point (upstream `PatchResult`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PatchResult {
    /// Upstream `albedoFlux` \[W/m^2\]: reflected sunlight from this patch;
    /// zero when the patch is not sunlit.
    pub albedo_flux_w_per_m2: f64,
    /// Upstream `irFlux` \[W/m^2\]: infrared from this patch.
    pub ir_flux_w_per_m2: f64,
    /// Upstream `rHat_IdA_N` \[-\]: unit vector from the patch to the point,
    /// inertial components.
    pub r_hat_ida_n: Vector3<f64>,
    /// Upstream `solidAngle` \[sr\]: solid angle of the patch seen from the
    /// point.
    pub solid_angle_sr: f64,
    /// Upstream `r_dAP_N` \[m\]: patch centroid relative to the planet center,
    /// inertial components.
    pub r_dap_n_m: Vector3<f64>,
}

/// A CSV albedo map: upstream's `albedoDataPath` and `albedoDataFile`.
///
/// The file holds one row per latitude band (south to north) and one column
/// per longitude, comma separated; its shape sets the grid size.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AlbedoDataFile {
    /// Directory holding the map (upstream `albedoDataPath`).
    pub data_path: PathBuf,
    /// File name inside `data_path` (upstream `albedoDataFile`). It must not
    /// lead outside `data_path`.
    pub file_name: String,
}

/// Configuration of a [`PlanetGrid`]: upstream `PlanetGrid`'s public members
/// once resolved.
#[derive(Clone, Debug, PartialEq)]
pub struct PlanetGridConfig {
    /// Upstream `nLat`: latitude bands; ignored with an albedo map.
    pub num_lat: usize,
    /// Upstream `nLon`: longitude zones per band; ignored with an albedo map.
    pub num_lon: usize,
    /// Upstream `irFluxMean` \[W/m^2\]: mean outgoing infrared exitance.
    pub ir_flux_mean_w_per_m2: f64,
    /// Upstream `albedoAvg` \[-\]: Bond albedo of every patch, when there is
    /// no albedo map.
    pub albedo_avg: f64,
    /// Upstream `albedoDataPath`/`albedoDataFile`: a per-patch albedo map,
    /// which replaces `albedo_avg` and the grid size.
    pub albedo_data: Option<AlbedoDataFile>,
    /// Upstream `REQ_m` \[m\]: equatorial radius.
    pub req_m: f64,
    /// Upstream `RP_m` \[m\]: polar radius, or `None` for a sphere (upstream
    /// -1).
    pub rp_m: Option<f64>,
}

impl Default for PlanetGridConfig {
    /// Upstream `PlanetGrid`'s defaults, with Earth's equatorial radius.
    fn default() -> Self {
        Self {
            num_lat: 180,
            num_lon: 360,
            ir_flux_mean_w_per_m2: 237.0,
            albedo_avg: 0.30,
            albedo_data: None,
            req_m: crate::utilities::astro_constants::REQ_EARTH * 1000.0,
            rp_m: None,
        }
    }
}

/// Why a planet grid could not be built (each a `bskError` upstream).
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlanetRadiationError {
    /// The grid size or the radii are unusable.
    InvalidGrid(String),
    /// The albedo map could not be loaded.
    AlbedoData { path: PathBuf, message: String },
}

impl fmt::Display for PlanetRadiationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidGrid(message) => write!(f, "planet grid: {message}"),
            Self::AlbedoData { path, message } => {
                write!(f, "albedo data {}: {message}", path.display())
            }
        }
    }
}

impl std::error::Error for PlanetRadiationError {}

/// An initialized patch grid for one planet (upstream `PlanetGrid` after
/// `initialize`).
///
/// Upstream's `computePatches` throws when called before `initialize`; here
/// a grid only exists initialized.
#[derive(Clone, Debug)]
pub struct PlanetGrid {
    num_lat: usize,
    num_lon: usize,
    ir_flux_mean_w_per_m2: f64,
    /// \[-\] patch centroid directions, planet-fixed components, row-major by
    /// latitude band.
    patch_dirs_p: Vec<Vector3<f64>>,
    /// \[sr\] patch areas on the unit sphere.
    norm_areas: Vec<f64>,
    /// \[-\] albedo of each patch.
    patch_albedo: Vec<f64>,
}

impl PlanetGrid {
    /// Upstream `PlanetGrid::initialize`: builds the patches, loading the
    /// albedo map if there is one.
    pub fn new(config: &PlanetGridConfig) -> Result<Self, PlanetRadiationError> {
        if config.num_lat == 0 {
            return Err(PlanetRadiationError::InvalidGrid(
                "nLat must be positive, got 0".to_string(),
            ));
        }
        if config.num_lon == 0 {
            return Err(PlanetRadiationError::InvalidGrid(
                "nLon must be positive, got 0".to_string(),
            ));
        }
        let authalic = authalic_latitude_coefficients(config.req_m, config.rp_m)?;

        let albedo_grid = config
            .albedo_data
            .as_ref()
            .map(load_albedo_grid)
            .transpose()?;
        let (num_lat, num_lon) = albedo_grid
            .as_ref()
            .map_or((config.num_lat, config.num_lon), |grid| {
                (grid.len(), grid[0].len())
            });

        // Patch centers span -90..90 deg in latitude and -180..180 deg in
        // longitude, in upstream's integer arithmetic.
        let lat_diff = (180.0 / num_lat as f64) * PI / 180.0;
        let lon_diff = (360.0 / num_lon as f64) * PI / 180.0;
        let half_lat = (num_lat / 2) as i64;
        let half_lon = (num_lon / 2) as i64;

        let patch_count = num_lat * num_lon;
        let mut patch_dirs_p = Vec::with_capacity(patch_count);
        let mut norm_areas = Vec::with_capacity(patch_count);
        let mut patch_albedo = Vec::with_capacity(patch_count);
        for ilat in 0..num_lat {
            let lat = ((ilat as i64 - half_lat) as f64 + 0.5) * lat_diff;
            let mut lat1 = lat + 0.5 * lat_diff;
            let mut lat2 = lat - 0.5 * lat_diff;
            if let Some(t) = authalic {
                lat1 -= t[0] * (2.0 * lat1).sin() - t[1] * (4.0 * lat1).sin()
                    + t[2] * (6.0 * lat1).sin();
                lat2 -= t[0] * (2.0 * lat2).sin() - t[1] * (4.0 * lat2).sin()
                    + t[2] * (6.0 * lat2).sin();
            }
            // Solid angle of the patch on the unit sphere.
            let norm_area = lon_diff * (lat1.sin() - lat2.sin()).abs();
            let (sin_lat, cos_lat) = lat.sin_cos();
            for ilon in 0..num_lon {
                let lon = ((ilon as i64 - half_lon) as f64 + 0.5) * lon_diff;
                patch_dirs_p.push(Vector3::new(
                    cos_lat * lon.cos(),
                    cos_lat * lon.sin(),
                    sin_lat,
                ));
                norm_areas.push(norm_area);
                patch_albedo.push(
                    albedo_grid
                        .as_ref()
                        .map_or(config.albedo_avg, |grid| grid[ilat][ilon]),
                );
            }
        }

        Ok(Self {
            num_lat,
            num_lon,
            ir_flux_mean_w_per_m2: config.ir_flux_mean_w_per_m2,
            patch_dirs_p,
            norm_areas,
            patch_albedo,
        })
    }

    /// Latitude bands in use (from the albedo map when there is one).
    pub fn num_lat(&self) -> usize {
        self.num_lat
    }

    /// Longitude zones per band in use.
    pub fn num_lon(&self) -> usize {
        self.num_lon
    }

    /// Upstream `PlanetGrid::computePatches`: the flux of every patch visible
    /// from `r_sat_n_m`.
    ///
    /// * `r_sat_n_m`, `r_sun_n_m`, `r_planet_n_m` \[m\]: positions of the
    ///   point, the Sun and the planet center, inertial components.
    /// * `inertial_to_fixed`: the planet orientation `[PN]` (upstream
    ///   `J20002Pfix`).
    /// * `r_planet_m` \[m\]: the (authalic) planet radius.
    /// * `s_sun_w_per_m2` \[W/m^2\]: solar flux at the planet.
    ///
    /// Eclipse shadowing is not applied here.
    pub fn compute_patches(
        &self,
        r_sat_n_m: Vector3<f64>,
        r_sun_n_m: Vector3<f64>,
        r_planet_n_m: Vector3<f64>,
        inertial_to_fixed: &Matrix3<f64>,
        r_planet_m: f64,
        s_sun_w_per_m2: f64,
    ) -> Vec<PatchResult> {
        let dcm_np = inertial_to_fixed.transpose();
        let rs = r_sat_n_m - r_planet_n_m;
        let r_sun = r_sun_n_m - r_planet_n_m;
        let r2 = r_planet_m.powi(2);

        let mut results = Vec::with_capacity(self.patch_dirs_p.len() / 2);
        for (index, dir_p) in self.patch_dirs_p.iter().enumerate() {
            let r_dap_n = r_planet_m * (dcm_np * dir_p);
            let r_hat_dap_n = r_dap_n / r_dap_n.norm();

            let r_ida_n = rs - r_dap_n;
            let d = r_ida_n.norm();
            let r_hat_ida_n = r_ida_n / d;

            // Cosine between the patch normal and the direction to the point;
            // a back-facing patch is not visible.
            let f2 = r_hat_dap_n.dot(&r_hat_ida_n);
            if f2 <= 0.0 {
                continue;
            }

            let d_area = self.norm_areas[index] * r2;
            let d_squared = d.powi(2);
            let ir_flux = self.ir_flux_mean_w_per_m2 * FRAC_1_PI * f2 * d_area / d_squared;

            let r_sda_n = r_sun - r_dap_n;
            let f1 = r_hat_dap_n.dot(&(r_sda_n / r_sda_n.norm()));
            let albedo_flux = if f1 > 0.0 {
                (s_sun_w_per_m2 * FRAC_1_PI) * self.patch_albedo[index] * f1 * f2 * d_area
                    / d_squared
            } else {
                0.0
            };

            results.push(PatchResult {
                albedo_flux_w_per_m2: albedo_flux,
                ir_flux_w_per_m2: ir_flux,
                r_hat_ida_n,
                solid_angle_sr: f2 * d_area / d_squared,
                r_dap_n_m: r_dap_n,
            });
        }
        results
    }
}

/// The authalic latitude correction coefficients of an oblate spheroid, or
/// `None` for a sphere; upstream `initialize`'s radius checks.
fn authalic_latitude_coefficients(
    req_m: f64,
    rp_m: Option<f64>,
) -> Result<Option<[f64; 3]>, PlanetRadiationError> {
    if !(req_m.is_finite() && req_m > 0.0) {
        return Err(PlanetRadiationError::InvalidGrid(format!(
            "REQ_m must be positive, got {req_m}"
        )));
    }
    let Some(rp_m) = rp_m else {
        return Ok(None);
    };
    if !(rp_m.is_finite() && rp_m > 0.0) {
        return Err(PlanetRadiationError::InvalidGrid(format!(
            "RP_m must be positive when set, got RP_m={rp_m}, REQ_m={req_m}"
        )));
    }
    if rp_m == req_m {
        return Ok(None);
    }
    if rp_m > req_m {
        return Err(PlanetRadiationError::InvalidGrid(format!(
            "RP_m must be less than REQ_m for an oblate spheroid, got RP_m={rp_m}, REQ_m={req_m}"
        )));
    }
    let e = (1.0 - rp_m.powi(2) / req_m.powi(2)).sqrt();
    let e2 = e.powi(2);
    let e4 = e2.powi(2);
    let e6 = e4 * e2;
    Ok(Some([
        e2 / 3.0 + 31.0 * e4 / 180.0 + 59.0 * e6 / 560.0,
        17.0 * e4 / 360.0 + 61.0 * e6 / 1260.0,
        383.0 * e6 / 45360.0,
    ]))
}

/// The authalic (equal-area) radius \[m\] of a spheroid with equatorial radius
/// `req_m` and polar radius `rp_m` (`None` for a sphere): upstream
/// `PlanetRadiationBase::Reset`.
///
/// # Panics
///
/// If `rp_m` exceeds `req_m`; [`PlanetGrid::new`] rejects that first.
pub fn authalic_radius(req_m: f64, rp_m: Option<f64>) -> f64 {
    match rp_m {
        Some(rp_m) if rp_m > 0.0 && rp_m != req_m => {
            assert!(
                rp_m < req_m,
                "RP_m must be less than REQ_m, got RP_m={rp_m}, REQ_m={req_m}"
            );
            let ec = (1.0 - rp_m.powi(2) / req_m.powi(2)).sqrt();
            (req_m.powi(2)
                * 0.5
                * (1.0 + (1.0 - ec.powi(2)) / (2.0 * ec) * ((1.0 + ec) / (1.0 - ec)).ln()))
            .sqrt()
        }
        _ => req_m,
    }
}

/// Upstream `PlanetGrid::loadAlbedoGridFromCsv`: the albedo map, one row per
/// latitude band.
pub fn load_albedo_grid(file: &AlbedoDataFile) -> Result<Vec<Vec<f64>>, PlanetRadiationError> {
    let error = |path: PathBuf, message: String| PlanetRadiationError::AlbedoData { path, message };

    let base = file.data_path.canonicalize().map_err(|source| {
        error(
            file.data_path.clone(),
            format!("invalid albedoDataPath ({source})"),
        )
    })?;
    let joined = base.join(&file.file_name);
    let path = joined.canonicalize().map_err(|source| {
        error(
            joined.clone(),
            format!("cannot open albedo file ({source})"),
        )
    })?;
    // The file must stay inside the data directory.
    if !path.starts_with(&base) {
        return Err(error(path, "path not acceptable".to_string()));
    }
    let text = std::fs::read_to_string(&path)
        .map_err(|source| error(path.clone(), format!("cannot open albedo file ({source})")))?;

    let mut grid: Vec<Vec<f64>> = Vec::new();
    for (line_number, line) in text.lines().enumerate() {
        if line.is_empty() {
            continue;
        }
        let row = line
            .split(',')
            .map(|field| {
                let field = field.trim();
                if field.is_empty() {
                    return Err(format!("empty field at line {line_number}"));
                }
                field
                    .parse::<f64>()
                    .map_err(|_| format!("invalid double '{field}' at line {line_number}"))
            })
            .collect::<Result<Vec<_>, _>>()
            .map_err(|message| error(path.clone(), message))?;
        if let Some(first) = grid.first()
            && first.len() != row.len()
        {
            return Err(error(path, "inconsistent number of columns".to_string()));
        }
        grid.push(row);
    }
    if grid.is_empty() {
        return Err(error(path, "empty albedo file".to_string()));
    }
    Ok(grid)
}

/// Upstream `PlanetRadiationBase::computeIlluminationAtdA`: the fraction of
/// the solar disk visible from a surface patch, 1 in full sunlight, 0 in the
/// umbra and in between in the penumbra (the overlap of the apparent Sun and
/// planet disks).
///
/// * `r_planet_m` \[m\]: planet radius.
/// * `r_dap_n_m` \[m\]: patch position relative to the planet center.
/// * `r_sp_n_m` \[m\]: Sun position relative to the planet center.
pub fn compute_illumination_at_da(
    r_planet_m: f64,
    r_dap_n_m: Vector3<f64>,
    r_sp_n_m: Vector3<f64>,
) -> f64 {
    let req_sun_m = REQ_SUN * 1000.0;

    let r_sda_n = r_sp_n_m - r_dap_n_m;
    let s = r_dap_n_m.norm();
    let f_1 = safe_asin((req_sun_m + r_planet_m) / r_sp_n_m.norm());
    let f_2 = safe_asin((req_sun_m - r_planet_m) / r_sp_n_m.norm());
    let s_0 = (-r_dap_n_m.dot(&r_sp_n_m)) / r_sp_n_m.norm();
    let c_1 = s_0 + r_planet_m / f_1.sin();
    let c_2 = s_0 - r_planet_m / f_2.sin();
    let l = (s.powi(2) - s_0.powi(2)).max(0.0).sqrt();
    let l_1 = c_1 * f_1.tan();
    let l_2 = c_2 * f_2.tan();

    let a = safe_asin(req_sun_m / r_sda_n.norm());
    let b = safe_asin(r_planet_m / r_dap_n_m.norm());
    let c = safe_acos((-r_dap_n_m.dot(&r_sda_n)) / (r_dap_n_m.norm() * r_sda_n.norm()));

    let mut illumination_factor = 1.0;
    if l.abs() < l_2.abs() || l.abs() < l_1.abs() {
        if c < b - a {
            illumination_factor = 0.0;
        } else if c < a - b {
            // Annular: the planet disk is inside the solar disk.
            let area_sun = PI * a.powi(2);
            let area_body = PI * b.powi(2);
            illumination_factor = 1.0 - area_body / area_sun;
        } else if c < a + b {
            let x = (c.powi(2) + a.powi(2) - b.powi(2)) / (2.0 * c);
            let y = (a.powi(2) - x.powi(2)).max(0.0).sqrt();
            let area = a.powi(2) * safe_acos(x / a) + b.powi(2) * safe_acos((c - x) / b) - c * y;
            illumination_factor = 1.0 - area / (PI * a.powi(2));
        }
    }
    illumination_factor.clamp(0.0, 1.0)
}

/// Upstream `UpdateState`'s solar flux \[W/m^2\] at `distance_m` from the
/// Sun.
pub(crate) fn solar_flux_at(distance_m: f64) -> f64 {
    SOLAR_FLUX_EARTH * AU2M.powi(2) / distance_m.powi(2)
}

/// The planet orientation `[PN]` (upstream `J20002Pfix`), the identity when
/// the message has none or a singular one (upstream replaces a zero
/// determinant by the identity).
pub(crate) fn planet_inertial_to_fixed(planet: &PlanetStateMsg) -> Matrix3<f64> {
    match &planet.orientation {
        Some(orientation) if orientation.inertial_to_fixed.determinant() != 0.0 => {
            orientation.inertial_to_fixed
        }
        _ => Matrix3::identity(),
    }
}

/// Ports of the upstream `PlanetGrid` tests in `test_albedo.py`.
#[cfg(test)]
mod tests {
    use nalgebra::{Matrix3, Vector3};

    use super::{
        AlbedoDataFile, PlanetGrid, PlanetGridConfig, PlanetRadiationError, authalic_radius,
        compute_illumination_at_da,
    };

    /// Upstream `test_ir_flux_conservation`: far from a uniformly emitting
    /// sphere, the infrared is `M R^2 / D^2`.
    #[test]
    fn ir_flux_is_conserved_far_from_the_planet() {
        let config = PlanetGridConfig {
            num_lat: 180,
            num_lon: 360,
            req_m: 6_371_000.0,
            rp_m: Some(6_371_000.0),
            ir_flux_mean_w_per_m2: 237.0,
            albedo_avg: 0.0,
            albedo_data: None,
        };
        let grid = PlanetGrid::new(&config).expect("valid grid");

        let d_center = 1e9;
        let patches = grid.compute_patches(
            Vector3::new(d_center, 0.0, 0.0),
            Vector3::new(1e11, 0.0, 0.0),
            Vector3::zeros(),
            &Matrix3::identity(),
            config.req_m,
            1361.0,
        );
        let total_flux: f64 = patches.iter().map(|patch| patch.ir_flux_w_per_m2).sum();
        let expected = config.ir_flux_mean_w_per_m2 * config.req_m.powi(2) / d_center.powi(2);

        assert!(total_flux > 0.0, "no IR flux");
        assert!(!patches.is_empty(), "no visible patch");
        let rel_error = (total_flux - expected).abs() / expected;
        assert!(rel_error < 1e-4, "relative error {rel_error:.4e}");
    }

    /// Upstream `test_planet_grid_initialize_invalid_albedo_file_raises`: an
    /// albedo map with no directory cannot be loaded.
    ///
    /// Upstream's `test_planet_grid_compute_patches_requires_initialize` has
    /// no counterpart: [`PlanetGrid::new`] is the only way to get a grid.
    #[test]
    fn an_unreadable_albedo_file_is_an_error() {
        let config = PlanetGridConfig {
            albedo_data: Some(AlbedoDataFile {
                data_path: "".into(),
                file_name: "nope.csv".to_string(),
            }),
            ..Default::default()
        };
        assert!(matches!(
            PlanetGrid::new(&config),
            Err(PlanetRadiationError::AlbedoData { .. })
        ));

        let config = PlanetGridConfig {
            albedo_data: Some(AlbedoDataFile {
                data_path: std::env::temp_dir(),
                file_name: "does_not_exist.file".to_string(),
            }),
            ..Default::default()
        };
        let error = PlanetGrid::new(&config).unwrap_err();
        assert!(
            error.to_string().contains("cannot open albedo file"),
            "{error}"
        );
    }

    #[test]
    fn albedo_files_outside_the_data_directory_are_rejected() {
        let directory = std::env::temp_dir().join(format!("bsk_rs_albedo_{}", std::process::id()));
        let inner = directory.join("inner");
        std::fs::create_dir_all(&inner).unwrap();
        std::fs::write(directory.join("map.csv"), "0.1,0.2\n0.3,0.4\n").unwrap();
        std::fs::write(inner.join("map.csv"), "0.1,0.2\n0.3,0.4\n").unwrap();

        let outside = PlanetGrid::new(&PlanetGridConfig {
            albedo_data: Some(AlbedoDataFile {
                data_path: inner.clone(),
                file_name: "../map.csv".to_string(),
            }),
            ..Default::default()
        });
        let inside = PlanetGrid::new(&PlanetGridConfig {
            albedo_data: Some(AlbedoDataFile {
                data_path: inner,
                file_name: "map.csv".to_string(),
            }),
            ..Default::default()
        });
        std::fs::remove_dir_all(&directory).unwrap();

        assert!(
            outside
                .unwrap_err()
                .to_string()
                .contains("path not acceptable")
        );
        let inside = inside.expect("map inside the directory");
        assert_eq!((inside.num_lat(), inside.num_lon()), (2, 2));
    }

    #[test]
    fn invalid_grids_are_rejected() {
        for config in [
            PlanetGridConfig {
                num_lat: 0,
                ..Default::default()
            },
            PlanetGridConfig {
                num_lon: 0,
                ..Default::default()
            },
            PlanetGridConfig {
                rp_m: Some(7.0e6),
                ..Default::default()
            },
            PlanetGridConfig {
                rp_m: Some(-2.0),
                ..Default::default()
            },
            PlanetGridConfig {
                req_m: -1.0,
                ..Default::default()
            },
        ] {
            assert!(
                matches!(
                    PlanetGrid::new(&config),
                    Err(PlanetRadiationError::InvalidGrid(_))
                ),
                "{config:?} should be rejected"
            );
        }
    }

    #[test]
    fn authalic_radius_lies_between_the_polar_and_equatorial_radii() {
        let (req, rp) = (6_378_136.6, 6_356_751.9);
        let radius = authalic_radius(req, Some(rp));
        assert!(rp < radius && radius < req, "{radius}");
        // A sphere, given or implied, keeps its radius.
        assert_eq!(authalic_radius(req, Some(req)), req);
        assert_eq!(authalic_radius(req, None), req);
    }

    #[test]
    fn illumination_is_full_on_the_day_side_and_zero_behind_the_planet() {
        let r_planet = 6_371_000.0;
        let r_sp = Vector3::new(1.496e11, 0.0, 0.0);
        let day = compute_illumination_at_da(r_planet, Vector3::new(r_planet, 0.0, 0.0), r_sp);
        let night = compute_illumination_at_da(r_planet, Vector3::new(-r_planet, 0.0, 0.0), r_sp);
        assert_eq!(day, 1.0);
        assert_eq!(night, 0.0);
    }
}
