//! Planetary albedo at instruments: port of upstream Basilisk
//! `simulation/environment/albedo` and of the patch grid of its
//! `planetRadiationBase` (`PlanetGrid`, shared with
//! [`EarthRadiationModel`](super::earth_radiation_model::EarthRadiationModel)).
//!
//! Upstream's `altitudeRateLimit`, `illuminationFactorAtdA` and settable
//! instrument defaults (`fov_default`, `nHat_B_default`) are not ported.

use std::f64::consts::{FRAC_1_PI, FRAC_PI_2, PI};
use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AlbedoMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::utilities::astro_constants::REQ_SUN;
use crate::utilities::linear_algebra::{safe_acos, safe_asin};
use crate::{Module, SimulationContext};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_WPM2: f64 = 1361.0;

#[derive(Clone, Debug)]
pub struct AlbedoInstrument {
    pub fov_half_angle_rad: f64,
    pub normal_body: Vector3<f64>,
    pub position_body_m: Vector3<f64>,
}

/// A planet's surface: its reflectivity and shape.
#[derive(Clone, Debug)]
pub struct AlbedoPlanet {
    pub albedo_avg: f64,
    /// Per-patch albedo, rows south to north, columns from -180 deg longitude.
    /// Replaces `albedo_avg`, `num_lat` and `num_lon` when set.
    pub albedo_map: Option<Vec<Vec<f64>>>,
    pub num_lat: usize,
    pub num_lon: usize,
    pub equatorial_radius_m: f64,
    /// Zero or negative means a spherical planet.
    pub polar_radius_m: f64,
}

#[derive(Clone, Debug)]
pub struct AlbedoConfig {
    pub name: String,
    pub albedo_avg: f64,
    /// Per-patch albedo, rows south to north, columns from -180 deg longitude.
    /// Replaces `albedo_avg`, `num_lat` and `num_lon` when set.
    pub albedo_map: Option<Vec<Vec<f64>>>,
    pub num_lat: usize,
    pub num_lon: usize,
    pub equatorial_radius_m: f64,
    /// Zero or negative means a spherical planet.
    pub polar_radius_m: f64,
    pub instruments: Vec<AlbedoInstrument>,
    /// Upstream `eclipseCase`: dims each patch by the fraction of the solar
    /// disk its planet leaves visible from it (the penumbra model of
    /// `illumination_at_patch`). Off, every sunlit patch is fully lit.
    pub eclipse_case: bool,
    /// Further planets, each read from the matching entry of
    /// [`Albedo::input_other_planet_msgs`]. Their albedo adds to the planet
    /// above at every instrument, as upstream sums every planet added with
    /// `addPlanetandAlbedo*Model`.
    pub other_planets: Vec<AlbedoPlanet>,
}

impl AlbedoConfig {
    pub fn earth(name: &str, instruments: Vec<AlbedoInstrument>) -> Self {
        Self {
            name: name.to_string(),
            albedo_avg: 0.29,
            albedo_map: None,
            num_lat: 180,
            num_lon: 360,
            equatorial_radius_m: 6_378_136.6,
            polar_radius_m: 6_356_751.9,
            instruments,
            eclipse_case: false,
            other_planets: Vec::new(),
        }
    }
}

/// Port of Basilisk's `albedo`, average or lookup-table model. The planet input is
/// optional: unconnected means the planet sits at the origin with no rotation.
/// The inputs of [`AlbedoConfig::other_planets`] must be connected.
pub struct Albedo {
    pub config: AlbedoConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_planet_msg: Input<PlanetStateMsg>,
    /// One per [`AlbedoConfig::other_planets`] entry, in the same order.
    pub input_other_planet_msgs: Vec<Input<PlanetStateMsg>>,
    pub output_albedo_msgs: Vec<Output<AlbedoMsg>>,
    /// The planet's grid, then one per other planet.
    grids: Vec<PlanetGrid>,
}

impl Module for Albedo {
    fn init(&mut self) {
        assert!(
            self.input_state_msg.is_connected() && self.input_sun_msg.is_connected(),
            "albedo {}: state and sun inputs must be connected",
            self.config.name
        );
        assert_eq!(
            self.config.instruments.len(),
            self.output_albedo_msgs.len(),
            "albedo {}: instruments changed after construction",
            self.config.name
        );
        assert_eq!(
            self.config.other_planets.len(),
            self.input_other_planet_msgs.len(),
            "albedo {}: other planets changed after construction",
            self.config.name
        );
        assert!(
            self.input_other_planet_msgs
                .iter()
                .all(|input| input.is_connected()),
            "albedo {}: every other planet input must be connected",
            self.config.name
        );
        for (i, instrument) in self.config.instruments.iter_mut().enumerate() {
            if instrument.fov_half_angle_rad.is_nan() || instrument.fov_half_angle_rad < 0.0 {
                log::warn!("albedo instrument {i}: invalid FOV, using 90 deg");
                instrument.fov_half_angle_rad = FRAC_PI_2;
            }
            if !instrument.normal_body.iter().all(|x| x.is_finite())
                || instrument.normal_body.norm() == 0.0
            {
                log::warn!("albedo instrument {i}: zero normal, using +x");
                instrument.normal_body = Vector3::x();
            }
        }
        self.build_grids();
        for output in &self.output_albedo_msgs {
            output.write(AlbedoMsg::default());
        }
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.input_state_msg.read();
        let sun = self.input_sun_msg.read();
        let planet = if self.input_planet_msg.is_connected() {
            self.input_planet_msg.read()
        } else {
            PlanetStateMsg::default()
        };
        let planets =
            std::iter::once(planet).chain(self.input_other_planet_msgs.iter().map(Input::read));
        let body_to_inertial = state.body_to_inertial();

        // Per instrument: ratio and flux from every visible lit patch, then
        // inside the field of view.
        let mut sums = vec![[0.0; 4]; self.config.instruments.len()];
        for (grid, planet) in self.grids.iter().zip(planets) {
            let fixed_to_inertial = planet_fixed_to_inertial(&planet);
            let sun_from_planet = sun.sun_position_inertial_m - planet.position_inertial_m;
            assert!(
                sun_from_planet.norm() > 0.0,
                "albedo {}: sun and planet positions coincide",
                self.config.name
            );
            let patches: Vec<Vector3<f64>> = grid
                .patch_dirs_fixed
                .iter()
                .map(|dir| grid.radius_m * (fixed_to_inertial * dir))
                .collect();

            for (instrument, sum) in self.config.instruments.iter().zip(&mut sums) {
                let instrument_from_planet = state.position_m
                    + body_to_inertial.transform_vector(&instrument.position_body_m)
                    - planet.position_inertial_m;
                let normal = body_to_inertial.transform_vector(&instrument.normal_body.normalize());
                let cos_fov = instrument.fov_half_angle_rad.cos();

                let (mut ratio_max, mut ratio) = (0.0, 0.0);
                for (k, patch) in patches.iter().enumerate() {
                    let patch_normal = patch.normalize();
                    let to_instrument = instrument_from_planet - patch;
                    let distance_m = to_instrument.norm();
                    let to_instrument_hat = to_instrument / distance_m;
                    let f2 = patch_normal.dot(&to_instrument_hat);
                    let f1 = patch_normal.dot(&(sun_from_planet - patch).normalize());
                    if f2 <= 0.0 || f1 <= 0.0 {
                        continue;
                    }
                    let mut patch_ratio =
                        FRAC_1_PI * f1 * f2 * grid.patch_albedo_area_sr(k) * grid.radius_m.powi(2)
                            / distance_m.powi(2);
                    if self.config.eclipse_case {
                        patch_ratio *=
                            illumination_at_patch(grid.radius_m, *patch, sun_from_planet);
                    }
                    ratio_max += patch_ratio;
                    let f3 = -normal.dot(&to_instrument_hat);
                    if f3 >= cos_fov {
                        ratio += patch_ratio * f3;
                    }
                }

                let solar_flux = SOLAR_FLUX_AT_EARTH_WPM2 * ASTRONOMICAL_UNIT_M.powi(2)
                    / (sun_from_planet - instrument_from_planet).norm_squared();
                sum[0] += ratio_max;
                sum[1] += solar_flux * ratio_max;
                sum[2] += ratio;
                sum[3] += solar_flux * ratio;
            }
        }

        for (sum, output) in sums.iter().zip(&self.output_albedo_msgs) {
            output.write(AlbedoMsg {
                albedo_at_instrument_max: sum[0],
                albedo_flux_at_instrument_max_wpm2: sum[1],
                albedo_at_instrument: sum[2],
                albedo_flux_at_instrument_wpm2: sum[3],
            });
        }
    }
}

impl Albedo {
    pub fn new(config: AlbedoConfig) -> Self {
        let output_albedo_msgs = config
            .instruments
            .iter()
            .map(|_| Output::default())
            .collect();
        let input_other_planet_msgs = config
            .other_planets
            .iter()
            .map(|_| Input::default())
            .collect();
        Self {
            config,
            input_state_msg: Input::default(),
            input_sun_msg: Input::default(),
            input_planet_msg: Input::default(),
            input_other_planet_msgs,
            output_albedo_msgs,
            grids: Vec::new(),
        }
    }

    fn build_grids(&mut self) {
        let c = &self.config;
        let first = PlanetGrid::new(
            c.albedo_avg,
            c.albedo_map.as_deref(),
            c.num_lat,
            c.num_lon,
            c.equatorial_radius_m,
            c.polar_radius_m,
        );
        let others = c.other_planets.iter().map(|p| {
            PlanetGrid::new(
                p.albedo_avg,
                p.albedo_map.as_deref(),
                p.num_lat,
                p.num_lon,
                p.equatorial_radius_m,
                p.polar_radius_m,
            )
        });
        self.grids = std::iter::once(first).chain(others).collect();
    }
}

/// A planet as latitude/longitude patches on its authalic sphere (upstream
/// `PlanetGrid`), shared with
/// [`EarthRadiationModel`](super::earth_radiation_model::EarthRadiationModel).
#[derive(Clone, Debug)]
pub(crate) struct PlanetGrid {
    /// \[m\] authalic radius.
    pub(crate) radius_m: f64,
    /// Patch centres, unit vectors in planet-fixed axes, south to north and
    /// from -180 deg longitude.
    pub(crate) patch_dirs_fixed: Vec<Vector3<f64>>,
    /// \[sr\] patch areas on the unit sphere.
    pub(crate) patch_area_sr: Vec<f64>,
    /// \[-\] patch albedos.
    pub(crate) patch_albedo: Vec<f64>,
}

impl PlanetGrid {
    /// The grid of a planet with `albedo_avg` on `num_lat` x `num_lon`
    /// patches, or with the patches and albedos of `albedo_map`; a zero or
    /// negative `polar_radius_m` is a sphere.
    ///
    /// # Panics
    ///
    /// On an empty, ragged or non-finite grid, a non-positive equatorial
    /// radius, or a polar radius above the equatorial one.
    pub(crate) fn new(
        albedo_avg: f64,
        albedo_map: Option<&[Vec<f64>]>,
        num_lat: usize,
        num_lon: usize,
        equatorial_radius_m: f64,
        polar_radius_m: f64,
    ) -> Self {
        let (num_lat, num_lon) = match albedo_map {
            Some(map) => (map.len(), map.first().map_or(0, Vec::len)),
            None => (num_lat, num_lon),
        };
        assert!(num_lat > 0 && num_lon > 0, "albedo grid must be non-empty");
        if let Some(map) = albedo_map {
            assert!(
                map.iter()
                    .all(|row| row.len() == num_lon && row.iter().all(|a| a.is_finite())),
                "albedo map must be rectangular and finite"
            );
        }
        assert!(
            equatorial_radius_m > 0.0,
            "equatorial radius must be positive"
        );
        let oblate = polar_radius_m > 0.0 && polar_radius_m != equatorial_radius_m;
        assert!(
            !oblate || polar_radius_m < equatorial_radius_m,
            "polar radius must be less than equatorial radius"
        );

        // Authalic sphere: equal-area latitudes and radius.
        let (mut t, mut radius_m) = ([0.0; 3], equatorial_radius_m);
        if oblate {
            let e2 = 1.0 - (polar_radius_m / equatorial_radius_m).powi(2);
            let e = e2.sqrt();
            let (e4, e6) = (e2 * e2, e2 * e2 * e2);
            t = [
                e2 / 3.0 + 31.0 * e4 / 180.0 + 59.0 * e6 / 560.0,
                17.0 * e4 / 360.0 + 61.0 * e6 / 1260.0,
                383.0 * e6 / 45360.0,
            ];
            radius_m = (equatorial_radius_m.powi(2)
                * 0.5
                * (1.0 + (1.0 - e2) / (2.0 * e) * ((1.0 + e) / (1.0 - e)).ln()))
            .sqrt();
        }
        let authalic = |lat: f64| {
            lat - (t[0] * (2.0 * lat).sin() - t[1] * (4.0 * lat).sin() + t[2] * (6.0 * lat).sin())
        };

        let lat_step = PI / num_lat as f64;
        let lon_step = 2.0 * PI / num_lon as f64;
        // Basilisk uses integer halves, which shifts odd grids by half a cell.
        let (half_lat, half_lon) = (num_lat as f64 / 2.0, num_lon as f64 / 2.0);
        let mut grid = Self {
            radius_m,
            patch_dirs_fixed: Vec::with_capacity(num_lat * num_lon),
            patch_area_sr: Vec::with_capacity(num_lat * num_lon),
            patch_albedo: Vec::with_capacity(num_lat * num_lon),
        };
        for i in 0..num_lat {
            let lat = (i as f64 - half_lat + 0.5) * lat_step;
            let area_sr = lon_step
                * (authalic(lat + 0.5 * lat_step).sin() - authalic(lat - 0.5 * lat_step).sin())
                    .abs();
            for j in 0..num_lon {
                let lon = (j as f64 - half_lon + 0.5) * lon_step;
                grid.patch_dirs_fixed.push(Vector3::new(
                    lat.cos() * lon.cos(),
                    lat.cos() * lon.sin(),
                    lat.sin(),
                ));
                grid.patch_area_sr.push(area_sr);
                grid.patch_albedo
                    .push(albedo_map.map_or(albedo_avg, |map| map[i][j]));
            }
        }
        grid
    }

    /// Albedo times area \[sr\] of patch `k`.
    pub(crate) fn patch_albedo_area_sr(&self, k: usize) -> f64 {
        self.patch_albedo[k] * self.patch_area_sr[k]
    }
}

/// The planet's `[NP]`: the transpose of its orientation, the identity when
/// it has none or a singular one.
pub(crate) fn planet_fixed_to_inertial(planet: &PlanetStateMsg) -> Matrix3<f64> {
    planet
        .orientation
        .as_ref()
        .map(|o| o.inertial_to_fixed)
        .filter(|m| m.determinant() != 0.0)
        .unwrap_or_else(Matrix3::identity)
        .transpose()
}

/// Upstream `PlanetRadiationBase::computeIlluminationAtdA`: the fraction of
/// the solar disk visible from a surface patch, 1 in full sunlight, 0 in the
/// umbra and in between in the penumbra (the overlap of the apparent Sun and
/// planet disks).
///
/// * `radius_m` \[m\]: planet radius.
/// * `patch_from_planet_m` \[m\]: patch position relative to the planet center.
/// * `sun_from_planet_m` \[m\]: Sun position relative to the planet center.
pub(crate) fn illumination_at_patch(
    radius_m: f64,
    patch_from_planet_m: Vector3<f64>,
    sun_from_planet_m: Vector3<f64>,
) -> f64 {
    let req_sun_m = REQ_SUN * 1000.0;
    let r_dap = patch_from_planet_m;
    let r_sp = sun_from_planet_m;

    let r_sda = r_sp - r_dap;
    let s = r_dap.norm();
    let f_1 = safe_asin((req_sun_m + radius_m) / r_sp.norm());
    let f_2 = safe_asin((req_sun_m - radius_m) / r_sp.norm());
    let s_0 = (-r_dap.dot(&r_sp)) / r_sp.norm();
    let c_1 = s_0 + radius_m / f_1.sin();
    let c_2 = s_0 - radius_m / f_2.sin();
    let l = (s.powi(2) - s_0.powi(2)).max(0.0).sqrt();
    let l_1 = c_1 * f_1.tan();
    let l_2 = c_2 * f_2.tan();

    let a = safe_asin(req_sun_m / r_sda.norm());
    let b = safe_asin(radius_m / r_dap.norm());
    let c = safe_acos((-r_dap.dot(&r_sda)) / (r_dap.norm() * r_sda.norm()));

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

/// Reads a Basilisk albedo CSV (`supportData/AlbedoData`), e.g. from `just fetch-albedo`.
pub fn load_albedo_map(path: impl AsRef<Path>) -> Result<Vec<Vec<f64>>, String> {
    let path = resolve_repo_relative_path(path.as_ref());
    let text = std::fs::read_to_string(&path).map_err(|e| format!("{}: {e}", path.display()))?;
    let map: Vec<Vec<f64>> = text
        .lines()
        .filter(|line| !line.trim().is_empty())
        .map(|line| {
            line.trim()
                .trim_end_matches(',')
                .split(',')
                .map(|v| {
                    v.trim()
                        .parse::<f64>()
                        .ok()
                        .filter(|x| x.is_finite())
                        .ok_or_else(|| format!("{}: invalid albedo {v:?}", path.display()))
                })
                .collect()
        })
        .collect::<Result<_, _>>()?;
    if map.is_empty() || map.iter().any(|row| row.len() != map[0].len()) {
        return Err(format!("{}: empty or ragged albedo table", path.display()));
    }
    Ok(map)
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use crate::messages::{
        AlbedoMsg, Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{
        Albedo, AlbedoConfig, AlbedoInstrument, AlbedoPlanet, PlanetGrid, illumination_at_patch,
        load_albedo_map,
    };

    struct Case {
        config: AlbedoConfig,
        planet_m: Vector3<f64>,
        inertial_to_fixed: Option<Matrix3<f64>>,
        spacecraft_from_planet_m: Vector3<f64>,
    }

    fn instruments() -> Vec<AlbedoInstrument> {
        vec![
            AlbedoInstrument {
                fov_half_angle_rad: 80.0_f64.to_radians(),
                normal_body: Vector3::new(-1.0, 0.0, 0.0),
                position_body_m: Vector3::new(0.5, 0.2, -0.1),
            },
            AlbedoInstrument {
                fov_half_angle_rad: 30.0_f64.to_radians(),
                normal_body: Vector3::new(-0.6, -0.5, -0.3),
                position_body_m: Vector3::zeros(),
            },
        ]
    }

    fn earth(inertial_to_fixed: Option<Matrix3<f64>>, table: bool) -> Case {
        let mut config = AlbedoConfig::earth("albedo", instruments());
        if table {
            config.albedo_map = Some(map("Earth_ALB_2018_CERES_All_5x5.csv"));
        }
        Case {
            config,
            planet_m: Vector3::zeros(),
            inertial_to_fixed,
            spacecraft_from_planet_m: Vector3::new(5.0e6, 4.0e6, 2.0e6),
        }
    }

    fn map(file: &str) -> Vec<Vec<f64>> {
        load_albedo_map(format!("assets/albedo/{file}")).expect("run `just fetch-albedo`")
    }

    /// Basilisk's `J20002Pfix` for a z rotation by `a` after an x rotation by `b`.
    fn rotation(a: f64, b: f64) -> Matrix3<f64> {
        let (sa, ca) = a.sin_cos();
        let (sb, cb) = b.sin_cos();
        Matrix3::new(ca, sa, 0.0, -sa, ca, 0.0, 0.0, 0.0, 1.0)
            * Matrix3::new(1.0, 0.0, 0.0, 0.0, cb, sb, 0.0, -sb, cb)
    }

    fn run(case: Case) -> Vec<[f64; 4]> {
        let mut albedo = Albedo::new(case.config);
        let state = Output::new(SpacecraftStateMsg {
            position_m: case.planet_m + case.spacecraft_from_planet_m,
            sigma_bn: Vector3::new(0.1, -0.2, 0.3),
            ..Default::default()
        });
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(1.4e11, 3.0e10, 1.2e10),
            sun_velocity_inertial_mps: Vector3::zeros(),
        });
        let planet = Output::new(PlanetStateMsg {
            position_inertial_m: case.planet_m,
            orientation: case.inertial_to_fixed.map(|m| PlanetOrientation {
                inertial_to_fixed: m,
                inertial_to_fixed_dot: Matrix3::zeros(),
            }),
            ..Default::default()
        });
        albedo.input_state_msg.connect(state.slot());
        albedo.input_sun_msg.connect(sun.slot());
        albedo.input_planet_msg.connect(planet.slot());
        albedo.init();
        albedo.update(&context());
        albedo
            .output_albedo_msgs
            .iter()
            .map(|o| {
                let m = o.read();
                [
                    m.albedo_at_instrument_max,
                    m.albedo_flux_at_instrument_max_wpm2,
                    m.albedo_at_instrument,
                    m.albedo_flux_at_instrument_wpm2,
                ]
            })
            .collect()
    }

    fn context() -> SimulationContext {
        SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    // References come from a Basilisk build that predates upstream's switch to
    // 1361 W/m^2, so its fluxes use 1372.5398.
    fn assert_close(actual: Vec<[f64; 4]>, expected: [[f64; 4]; 2]) {
        assert_close_to(actual, expected, 1.0e-13);
    }

    fn assert_close_to(actual: Vec<[f64; 4]>, expected: [[f64; 4]; 2], tolerance: f64) {
        for (a, e) in actual.iter().zip(&expected) {
            for k in 0..4 {
                let e = if k % 2 == 1 {
                    e[k] * 1361.0 / 1372.5398
                } else {
                    e[k]
                };
                assert!(
                    ((a[k] - e) / e).abs() < tolerance,
                    "got {}, Basilisk {e}",
                    a[k]
                );
            }
        }
    }

    #[test]
    fn matches_basilisk_without_planet_rotation() {
        assert_close(
            run(earth(None, false)),
            [
                [
                    0.3461328262838361,
                    515.0628426311036,
                    0.18028158435788924,
                    268.26795456051144,
                ],
                [
                    0.34613291144419045,
                    515.0629693534131,
                    0.01629851307194263,
                    24.252997219635205,
                ],
            ],
        );
    }

    #[test]
    fn matches_basilisk_with_planet_rotation() {
        assert_close(
            run(earth(Some(rotation(0.7, 0.0)), false)),
            [
                [
                    0.346131853285857,
                    515.0613947617686,
                    0.17984862134079602,
                    267.6236840799205,
                ],
                [
                    0.3461319384729301,
                    515.061521523837,
                    0.016560134494855185,
                    24.64230289521969,
                ],
            ],
        );
    }

    #[test]
    fn matches_basilisk_with_albedo_table() {
        assert_close(
            run(earth(None, true)),
            [
                [
                    0.29200436448511863,
                    434.51700217841363,
                    0.16857246456200076,
                    250.84420255314487,
                ],
                [
                    0.2920045775825069,
                    434.5173192774687,
                    0.010856416124623423,
                    16.15488657912962,
                ],
            ],
        );
        assert_close(
            run(earth(Some(rotation(0.7, 0.0)), true)),
            [
                [
                    0.4186413682562449,
                    622.9591555705781,
                    0.24036077548183857,
                    357.6687759027309,
                ],
                [
                    0.41864175989621,
                    622.9597383496647,
                    0.012015562494176936,
                    17.879753967574203,
                ],
            ],
        );
    }

    /// Far from the origin one ulp of position moves the output by ~1e-12.
    const OFFSET_TOLERANCE: f64 = 1.0e-11;

    #[test]
    fn matches_basilisk_for_offset_rotated_oblate_mars_table() {
        let mut config = AlbedoConfig::earth("albedo", instruments());
        config.albedo_map = Some(map("Mars_ALB_TES_10x10.csv"));
        config.equatorial_radius_m = 3_396_190.0;
        config.polar_radius_m = 3_376_200.0;
        assert_close_to(
            run(Case {
                config,
                planet_m: Vector3::new(-2.0e10, 1.0e10, 5.0e9),
                inertial_to_fixed: Some(rotation(1.1, 0.4)),
                spacecraft_from_planet_m: Vector3::new(3.0e6, 2.0e6, 1.5e6),
            }),
            [
                [
                    0.16190964252220383,
                    190.93041575121475,
                    0.09927786429051681,
                    117.07248319865566,
                ],
                [
                    0.16190970802321736,
                    190.9304929926143,
                    0.002376136961800661,
                    2.802037055551462,
                ],
            ],
            OFFSET_TOLERANCE,
        );
    }

    #[test]
    fn matches_basilisk_for_offset_rotated_spherical_moon_grid() {
        let mut config = AlbedoConfig::earth("albedo", instruments());
        config.albedo_avg = 0.123;
        (config.num_lat, config.num_lon) = (36, 72);
        config.equatorial_radius_m = 1_737_400.0;
        config.polar_radius_m = -1.0;
        assert_close_to(
            run(Case {
                config,
                planet_m: Vector3::new(3.0e8, -2.0e8, 1.0e8),
                inertial_to_fixed: Some(rotation(-0.5, 0.2)),
                spacecraft_from_planet_m: Vector3::new(1.5e6, 0.8e6, 0.5e6),
            }),
            [
                [
                    0.10194873427673944,
                    152.24355860510343,
                    0.036839319630295374,
                    55.01342568788178,
                ],
                [
                    0.10194853829918336,
                    152.24326594490634,
                    0.0015381843746845219,
                    2.2970237409403933,
                ],
            ],
            OFFSET_TOLERANCE,
        );
    }

    #[test]
    fn odd_grid_covers_the_sphere_once() {
        for polar_radius_m in [-1.0, 6_356_751.9] {
            let mut config = AlbedoConfig::earth("albedo", instruments());
            (config.num_lat, config.num_lon) = (11, 7);
            config.polar_radius_m = polar_radius_m;
            let mut albedo = Albedo::new(config);
            albedo.build_grids();
            let grid = &albedo.grids[0];
            let area: f64 = grid.patch_area_sr.iter().sum();
            let centroid: Vector3<f64> = grid.patch_dirs_fixed.iter().sum();
            assert!((area - 4.0 * PI).abs() < 1.0e-12, "area {area}");
            assert!(centroid.norm() < 1.0e-12, "centroid {centroid}");
        }
    }

    #[test]
    #[should_panic(expected = "rectangular")]
    fn ragged_map_is_rejected() {
        let mut config = AlbedoConfig::earth("albedo", instruments());
        config.albedo_map = Some(vec![vec![0.3, 0.3], vec![0.3]]);
        Albedo::new(config).build_grids();
    }

    #[test]
    #[should_panic(expected = "must be connected")]
    fn unconnected_inputs_are_rejected() {
        Albedo::new(AlbedoConfig::earth("albedo", instruments())).init();
    }

    /// Upstream `test_albedo_instConfig_defaults_fallback`, with upstream's
    /// default `fov_default` (90 deg) and `nHat_B_default` (`+x`), which are
    /// fixed here.
    #[test]
    fn invalid_instrument_falls_back_to_basilisk_defaults() {
        let mut case = earth(None, false);
        case.config.instruments = vec![AlbedoInstrument {
            fov_half_angle_rad: -1.0,
            normal_body: Vector3::zeros(),
            position_body_m: Vector3::zeros(),
        }];
        let mut fixed = earth(None, false);
        fixed.config.instruments = vec![AlbedoInstrument {
            fov_half_angle_rad: PI / 2.0,
            normal_body: Vector3::x(),
            position_body_m: Vector3::zeros(),
        }];
        assert_eq!(run(case), run(fixed));
    }

    /// Also upstream `test_albedo_invalid_file` and
    /// `test_planet_grid_initialize_invalid_albedo_file_raises`: a missing
    /// table is an error.
    #[test]
    fn loader_accepts_trailing_commas_and_rejects_non_finite_values() {
        let path = std::env::temp_dir().join(format!("albedo_{}.csv", std::process::id()));
        std::fs::write(&path, "0.1,0.2,\n0.3,0.4,\n").unwrap();
        assert_eq!(
            load_albedo_map(&path),
            Ok(vec![vec![0.1, 0.2], vec![0.3, 0.4]])
        );
        std::fs::write(&path, "0.1,nan\n0.3,0.4\n").unwrap();
        assert!(load_albedo_map(&path).is_err());
        std::fs::remove_file(path).unwrap();
        assert!(load_albedo_map(std::env::temp_dir().join("does_not_exist.file")).is_err());
    }

    // Ports of upstream `test_albedo.py`. Not ported: the tests of
    // `altitudeRateLimit` and `illuminationFactorAtdA`, which this module
    // does not have; `test_planet_grid_compute_patches_requires_initialize`,
    // as a [`PlanetGrid`] only exists built; and
    // `test_eclipseCase_cutoff_notice.py`, a reminder to retire upstream's
    // deprecated public `eclipseCase` member. `test_ir_flux_conservation` is
    // in `earth_radiation_model`, which has the infrared.

    /// Upstream's planets: `(albedo_avg, radii)` of `getAlbedoAverage` and
    /// `lookupPlanetRadius`.
    fn upstream_planet(mars: bool) -> AlbedoConfig {
        let mut config = AlbedoConfig::earth("Albedo_0", Vec::new());
        if mars {
            config.albedo_avg = 0.16;
            config.equatorial_radius_m = 3_396_190.0;
            config.polar_radius_m = 3_376_200.0;
        }
        config
    }

    const ALPHA_DEG: f64 = 71.0;

    /// Upstream's instrument: an 80 deg field of view looking back at the
    /// planet centre from 71 deg off the Sun line.
    fn upstream_instrument() -> AlbedoInstrument {
        let alpha = ALPHA_DEG.to_radians();
        AlbedoInstrument {
            fov_half_angle_rad: 80.0_f64.to_radians(),
            normal_body: Vector3::new(-alpha.cos(), -alpha.sin(), 0.0),
            position_body_m: Vector3::zeros(),
        }
    }

    /// One update of `config` with the Sun at `sun_m`, the planet at
    /// `planet_m` and the other planets at `others_m`, the spacecraft at
    /// `spacecraft_m` with zero attitude. Returns the first instrument's
    /// output.
    fn run_scene(
        config: AlbedoConfig,
        sun_m: Vector3<f64>,
        planet_m: Vector3<f64>,
        others_m: &[Vector3<f64>],
        spacecraft_m: Vector3<f64>,
    ) -> AlbedoMsg {
        let at = |position_inertial_m| PlanetStateMsg {
            position_inertial_m,
            orientation: Some(PlanetOrientation::identity()),
            ..Default::default()
        };
        let mut albedo = Albedo::new(config);
        let state = Output::new(SpacecraftStateMsg {
            position_m: spacecraft_m,
            ..Default::default()
        });
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: sun_m,
            ..Default::default()
        });
        let planet = Output::new(at(planet_m));
        let others: Vec<_> = others_m.iter().map(|&p| Output::new(at(p))).collect();
        albedo.input_state_msg.connect(state.slot());
        albedo.input_sun_msg.connect(sun.slot());
        albedo.input_planet_msg.connect(planet.slot());
        for (input, output) in albedo.input_other_planet_msgs.iter_mut().zip(&others) {
            input.connect(output.slot());
        }
        albedo.init();
        albedo.update(&context());
        albedo.output_albedo_msgs[0].read()
    }

    /// Upstream's geometry: the Sun along `-x` (1 AU for Earth, 1.5 AU for
    /// Mars), the spacecraft 6000 km above the equator 71 deg from `+x`, the
    /// scene shifted by `offset_m`.
    fn upstream_scene(config: AlbedoConfig, mars: bool, offset_m: Vector3<f64>) -> AlbedoMsg {
        let au_m = crate::utilities::astro_constants::AU * 1000.0;
        let sun_m = Vector3::new(if mars { -1.5 * au_m } else { -au_m }, 0.0, 0.0);
        let r_sc = config.equatorial_radius_m + 6000.0e3;
        let alpha = ALPHA_DEG.to_radians();
        run_scene(
            config,
            offset_m + sun_m,
            offset_m,
            &[],
            offset_m + r_sc * Vector3::new(alpha.cos(), alpha.sin(), 0.0),
        )
    }

    /// Upstream `ALBEDO_AVG_EXPLICIT`: albedo 0.5 on 200 x 400 patches.
    fn explicit(mut config: AlbedoConfig) -> AlbedoConfig {
        config.albedo_avg = 0.5;
        (config.num_lat, config.num_lon) = (200, 400);
        config
    }

    /// Upstream `test_unitAlbedo`: Earth and Mars, with the planet's
    /// average albedo on the default 180 x 360 grid (`ALBEDO_AVG_IMPLICIT`),
    /// 0.5 on 200 x 400 (`ALBEDO_AVG_EXPLICIT`) or the 10 x 10 deg table
    /// (`ALBEDO_DATA`), with and without eclipses, to 1e-12.
    #[test]
    fn albedo_at_instrument_matches_upstream_truth() {
        let cases = [
            (false, "data", true, 0.0022055492477917),
            (false, "data", false, 0.0022055492477917),
            (false, "explicit", true, 0.0041742091531996),
            (false, "explicit", false, 0.004174209177079),
            (false, "implicit", true, 0.002421222716229847),
            (false, "implicit", false, 0.002421222716229847),
            (true, "data", true, 0.0014001432717662),
            (true, "data", false, 0.0014001432717662),
            (true, "explicit", true, 0.0035681407388827),
            (true, "explicit", false, 0.0035681407390035),
            (true, "implicit", true, 0.0011418311186365906),
            (true, "implicit", false, 0.0011418311186365906),
        ];
        for (mars, model, eclipse_case, truth) in cases {
            let mut config = upstream_planet(mars);
            match model {
                "data" => {
                    config.albedo_map = Some(map(if mars {
                        "Mars_ALB_TES_10x10.csv"
                    } else {
                        "Earth_ALB_2018_CERES_All_10x10.csv"
                    }));
                }
                "explicit" => config = explicit(config),
                _ => {}
            }
            config.eclipse_case = eclipse_case;
            config.instruments = vec![upstream_instrument()];
            let albedo = upstream_scene(config, mars, Vector3::zeros()).albedo_at_instrument;
            assert!(
                (albedo - truth).abs() < 1e-12,
                "mars={mars} {model} eclipse={eclipse_case}: got {albedo:.16}, expected {truth}"
            );
        }
    }

    /// Upstream `_albedo_run_earth`: the explicit Earth, no eclipses, one
    /// instrument.
    fn earth_with(instrument: AlbedoInstrument) -> AlbedoMsg {
        let mut config = explicit(upstream_planet(false));
        config.instruments = vec![instrument];
        upstream_scene(config, false, Vector3::zeros())
    }

    fn assert_relative(actual: f64, expected: f64, rel: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= rel * expected.abs(),
            "{what}: {actual:.15e} vs {expected:.15e}"
        );
    }

    /// Upstream `test_albedo_eclipse_uses_planet_relative_sun_vector`.
    #[test]
    fn eclipse_output_is_translation_invariant() {
        let shifted = |offset_m| {
            let mut config = explicit(upstream_planet(false));
            config.eclipse_case = true;
            config.instruments = vec![upstream_instrument()];
            upstream_scene(config, false, offset_m).albedo_at_instrument
        };
        let at_origin = shifted(Vector3::zeros());
        let offset = shifted(Vector3::new(1e12, 0.0, 0.0));
        assert!(
            at_origin > 0.0,
            "reference albedo at origin must be positive"
        );
        let rel_err = (at_origin - offset).abs() / at_origin;
        assert!(
            rel_err < 1e-10,
            "origin={at_origin:.15e}, offset={offset:.15e}, rel_err={rel_err:.2e}"
        );
    }

    /// Upstream `test_albedo_nHat_B_normalization`.
    #[test]
    fn instrument_normal_is_normalized() {
        let unit = earth_with(upstream_instrument());
        let scaled = earth_with(AlbedoInstrument {
            normal_body: 5.0 * upstream_instrument().normal_body,
            ..upstream_instrument()
        });
        assert!(unit.albedo_at_instrument > 0.0);
        assert_relative(
            scaled.albedo_at_instrument,
            unit.albedo_at_instrument,
            1e-10,
            "scaled normal",
        );
    }

    /// Upstream `test_albedo_AfluxAtInstrument_positive_and_consistent`.
    #[test]
    fn albedo_flux_is_positive_and_bounded_by_the_unfiltered_flux() {
        let out = earth_with(upstream_instrument());
        assert!(out.albedo_flux_at_instrument_wpm2 > 0.0);
        assert!(out.albedo_flux_at_instrument_max_wpm2 > 0.0);
        assert!(
            out.albedo_flux_at_instrument_max_wpm2 >= out.albedo_flux_at_instrument_wpm2 - 1e-12
        );
    }

    /// Upstream `_run_albedo_multi_planet_scene`: 120 x 240 "earths" at
    /// `planets_m`, one wide instrument along `+x`.
    fn multi_planet_scene(planets_m: &[Vector3<f64>]) -> AlbedoMsg {
        let mut config = upstream_planet(false);
        config.albedo_avg = 0.5;
        (config.num_lat, config.num_lon) = (120, 240);
        let earth = AlbedoPlanet {
            albedo_avg: config.albedo_avg,
            albedo_map: None,
            num_lat: config.num_lat,
            num_lon: config.num_lon,
            equatorial_radius_m: config.equatorial_radius_m,
            polar_radius_m: config.polar_radius_m,
        };
        config.other_planets = vec![earth; planets_m.len() - 1];
        config.instruments = vec![AlbedoInstrument {
            fov_half_angle_rad: 120.0_f64.to_radians(),
            normal_body: Vector3::x(),
            position_body_m: Vector3::zeros(),
        }];
        run_scene(
            config,
            Vector3::new(-1.0e8, 0.0, 0.0),
            planets_m[0],
            &planets_m[1..],
            Vector3::new(0.0, 8.0e7, 0.0),
        )
    }

    /// Upstream `test_albedo_two_planet_superposition_ratio_max`.
    #[test]
    fn two_planets_superpose() {
        let p_a = Vector3::new(-2.0e7, 0.0, 0.0);
        let p_b = Vector3::new(2.0e7, 0.0, 0.0);
        let a = multi_planet_scene(&[p_a]).albedo_at_instrument_max;
        let b = multi_planet_scene(&[p_b]).albedo_at_instrument_max;
        let ab = multi_planet_scene(&[p_a, p_b]).albedo_at_instrument_max;
        assert!(a > 0.0 && b > 0.0 && ab > 0.0);
        assert_relative(ab, a + b, 5e-3, "superposition");
    }

    /// Upstream `test_albedo_two_planet_order_invariance_max_fields`.
    #[test]
    fn planet_order_does_not_matter() {
        let p_a = Vector3::new(-2.0e7, 0.0, 0.0);
        let p_b = Vector3::new(2.0e7, 0.0, 0.0);
        let ab = multi_planet_scene(&[p_a, p_b]);
        let ba = multi_planet_scene(&[p_b, p_a]);
        assert_relative(
            ab.albedo_at_instrument_max,
            ba.albedo_at_instrument_max,
            1e-10,
            "ratio",
        );
        assert_relative(
            ab.albedo_flux_at_instrument_max_wpm2,
            ba.albedo_flux_at_instrument_max_wpm2,
            1e-10,
            "flux",
        );
    }

    #[test]
    #[should_panic(expected = "every other planet input must be connected")]
    fn unconnected_other_planet_is_rejected() {
        let mut config = AlbedoConfig::earth("albedo", instruments());
        config.other_planets.push(AlbedoPlanet {
            albedo_avg: 0.123,
            albedo_map: None,
            num_lat: 18,
            num_lon: 36,
            equatorial_radius_m: 1_737_400.0,
            polar_radius_m: -1.0,
        });
        let mut albedo = Albedo::new(config);
        let state = Output::new(SpacecraftStateMsg::default());
        let sun = Output::new(SunEphemerisMsg::default());
        albedo.input_state_msg.connect(state.slot());
        albedo.input_sun_msg.connect(sun.slot());
        albedo.init();
    }

    #[test]
    fn authalic_radius_lies_between_the_polar_and_equatorial_radii() {
        let (req, rp) = (6_378_136.6, 6_356_751.9);
        let radius = PlanetGrid::new(0.3, None, 2, 4, req, rp).radius_m;
        assert!(rp < radius && radius < req, "{radius}");
        // A sphere, given or implied, keeps its radius.
        assert_eq!(PlanetGrid::new(0.3, None, 2, 4, req, req).radius_m, req);
        assert_eq!(PlanetGrid::new(0.3, None, 2, 4, req, -1.0).radius_m, req);
    }

    #[test]
    #[should_panic(expected = "polar radius must be less than equatorial radius")]
    fn prolate_planet_is_rejected() {
        PlanetGrid::new(0.3, None, 2, 4, 6.0e6, 7.0e6);
    }

    #[test]
    fn illumination_is_full_on_the_day_side_and_zero_behind_the_planet() {
        let r_planet = 6_371_000.0;
        let r_sp = Vector3::new(1.496e11, 0.0, 0.0);
        let day = illumination_at_patch(r_planet, Vector3::new(r_planet, 0.0, 0.0), r_sp);
        let night = illumination_at_patch(r_planet, Vector3::new(-r_planet, 0.0, 0.0), r_sp);
        assert_eq!(day, 1.0);
        assert_eq!(night, 0.0);
    }
}
