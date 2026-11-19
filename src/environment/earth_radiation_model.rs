//! Earth radiation at a spacecraft: port of upstream Basilisk
//! `simulation/environment/earthRadiationModel`.
//!
//! The Earth is a [`PlanetGrid`] of Lambertian patches. The module sums the
//! reflected sunlight (albedo) and the infrared every visible patch sends to
//! the spacecraft, and the flux-weighted directions of both, which are the
//! net force directions of an extended source (neither parallel rays nor
//! anti-nadir). The albedo is uniform ([`EarthRadiationModelConfig::albedo_avg`])
//! or a CSV albedo map (upstream `supportData/AlbedoData`, fetched by `just
//! fetch-environment-data`). Optionally each patch is shadowed by the
//! penumbra model; the infrared is emitted regardless.
//!
//! # Differences from upstream
//!
//! * Upstream builds the grid at `Reset`, where an unreadable albedo map is a
//!   `bskError`; here [`EarthRadiationModel::new`] builds it and returns the
//!   error.
//! * The Earth is `planet_state_in_msg`, upstream's `planetInMsg` (the
//!   radiation base's single-planet fallback, the only planet upstream's
//!   model uses).

use std::fmt;

use nalgebra::Vector3;

use super::planet_radiation::{
    AlbedoDataFile, PlanetGrid, PlanetGridConfig, PlanetRadiationError, authalic_radius,
    compute_illumination_at_da, planet_inertial_to_fixed, solar_flux_at,
};
use crate::messages::{
    EarthRadiationMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::utilities::astro_constants::{REQ_EARTH, RP_EARTH};
use crate::{Module, SimulationContext};

/// User-settable parameters of [`EarthRadiationModel`]: upstream's scalar
/// configuration of the radiation base.
#[derive(Clone, Debug)]
pub struct EarthRadiationModelConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `defaultNumLat`: latitude bands; ignored with an albedo map.
    pub num_lat: usize,
    /// Upstream `defaultNumLon`: longitude zones; ignored with an albedo map.
    pub num_lon: usize,
    /// Upstream `irFluxMean` \[W/m^2\]: mean outgoing infrared exitance.
    pub ir_flux_mean_w_per_m2: f64,
    /// Upstream `albedoAvg` \[-\]: Bond albedo when there is no albedo map.
    pub albedo_avg: f64,
    /// Upstream `albedoDataPath`/`albedoDataFile`: an albedo map.
    pub albedo_data: Option<AlbedoDataFile>,
    /// Upstream `REQ_m` \[m\]: equatorial radius; `None` (upstream -1) is
    /// Earth's.
    pub req_m: Option<f64>,
    /// Upstream `RP_m` \[m\]: polar radius; `None` (upstream -1) is Earth's.
    pub rp_m: Option<f64>,
    /// Upstream `eclipseCase`: shadow each patch's albedo with the penumbra
    /// model.
    pub eclipse_case: bool,
}

impl Default for EarthRadiationModelConfig {
    fn default() -> Self {
        Self {
            name: "earthRadiation".to_string(),
            num_lat: 180,
            num_lon: 360,
            ir_flux_mean_w_per_m2: 237.0,
            albedo_avg: 0.30,
            albedo_data: None,
            req_m: None,
            rp_m: None,
            eclipse_case: false,
        }
    }
}

/// An input upstream's `Reset` requires, reported with `bskError` there and
/// logged at [`Module::init`] here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EarthRadiationInputNotConnected(pub &'static str);

impl fmt::Display for EarthRadiationInputNotConnected {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} is not connected", self.0)
    }
}

/// Earth albedo and infrared at a spacecraft (upstream
/// `EarthRadiationModel`).
#[derive(Clone, Debug)]
pub struct EarthRadiationModel {
    /// Module configuration.
    pub config: EarthRadiationModelConfig,
    /// Upstream `spacecraftStateInMsg`: spacecraft position.
    pub spacecraft_state_in_msg: Input<SpacecraftStateMsg>,
    /// Upstream `planetInMsg`: Earth position and orientation.
    pub planet_state_in_msg: Input<PlanetStateMsg>,
    /// Upstream `sunPositionInMsg`: Sun position.
    pub sun_position_in_msg: Input<SunEphemerisMsg>,
    /// Upstream `earthRadiationOutMsg`: total fluxes and directions.
    pub earth_radiation_out_msg: Output<EarthRadiationMsg>,
    grid: PlanetGrid,
    /// \[m\] authalic Earth radius.
    r_planet_m: f64,
}

impl EarthRadiationModel {
    /// Builds the Earth grid (upstream `Reset`'s single-planet fallback).
    pub fn new(config: EarthRadiationModelConfig) -> Result<Self, PlanetRadiationError> {
        let req_m = config
            .req_m
            .filter(|&r| r > 0.0)
            .unwrap_or(REQ_EARTH * 1000.0);
        let rp_m = config
            .rp_m
            .filter(|&r| r > 0.0)
            .unwrap_or(RP_EARTH * 1000.0);
        let grid = PlanetGrid::new(&PlanetGridConfig {
            num_lat: config.num_lat,
            num_lon: config.num_lon,
            ir_flux_mean_w_per_m2: config.ir_flux_mean_w_per_m2,
            albedo_avg: config.albedo_avg,
            albedo_data: config.albedo_data.clone(),
            req_m,
            rp_m: Some(rp_m),
        })?;
        Ok(Self {
            config,
            spacecraft_state_in_msg: Input::default(),
            planet_state_in_msg: Input::default(),
            sun_position_in_msg: Input::default(),
            earth_radiation_out_msg: Output::default(),
            grid,
            r_planet_m: authalic_radius(req_m, Some(rp_m)),
        })
    }

    /// Upstream `Reset`'s input checks.
    pub fn configuration_errors(&self) -> Vec<EarthRadiationInputNotConnected> {
        [
            (
                self.spacecraft_state_in_msg.is_connected(),
                "spacecraft_state_in_msg",
            ),
            (
                self.sun_position_in_msg.is_connected(),
                "sun_position_in_msg",
            ),
            (
                self.planet_state_in_msg.is_connected(),
                "planet_state_in_msg",
            ),
        ]
        .into_iter()
        .filter(|(connected, _)| !connected)
        .map(|(_, port)| EarthRadiationInputNotConnected(port))
        .collect()
    }

    /// Upstream `evaluatePlanet` and `onUpdateEnd`.
    fn radiation(&self, context: &SimulationContext) -> EarthRadiationMsg {
        let r_sat_n_m = self.spacecraft_state_in_msg.read().position_m;
        let r_sun_n_m = self.sun_position_in_msg.read().sun_position_inertial_m;
        let planet = self.planet_state_in_msg.read();
        let r_sp_n_m = r_sun_n_m - planet.position_inertial_m;

        let mut albedo_flux = 0.0;
        let mut ir_flux = 0.0;
        let mut albedo_vec_n = Vector3::zeros();
        let mut ir_vec_n = Vector3::zeros();
        let sun_distance_m = r_sp_n_m.norm();
        if sun_distance_m <= 0.0 {
            log::error!("{}: Sun-Earth distance <= 0 m", self.config.name);
        } else {
            let patches = self.grid.compute_patches(
                r_sat_n_m,
                r_sun_n_m,
                planet.position_inertial_m,
                &planet_inertial_to_fixed(&planet),
                self.r_planet_m,
                solar_flux_at(sun_distance_m),
            );
            for patch in patches {
                let illumination = if self.config.eclipse_case {
                    compute_illumination_at_da(self.r_planet_m, patch.r_dap_n_m, r_sp_n_m)
                } else {
                    1.0
                };
                albedo_flux += patch.albedo_flux_w_per_m2 * illumination;
                albedo_vec_n += patch.albedo_flux_w_per_m2 * illumination * patch.r_hat_ida_n;
                // Infrared is emitted whatever the illumination.
                ir_flux += patch.ir_flux_w_per_m2;
                ir_vec_n += patch.ir_flux_w_per_m2 * patch.r_hat_ida_n;
            }
        }

        EarthRadiationMsg {
            albedo_flux_w_per_m2: albedo_flux,
            ir_flux_w_per_m2: ir_flux,
            albedo_direction_inertial: unit_or_zero(albedo_flux, albedo_vec_n),
            ir_direction_inertial: unit_or_zero(ir_flux, ir_vec_n),
            time_tag_nanos: context.current_sim_nanos,
        }
    }
}

/// The direction of a flux-weighted sum, zero when there is no flux.
fn unit_or_zero(flux: f64, weighted_n: Vector3<f64>) -> Vector3<f64> {
    let magnitude = weighted_n.norm();
    if flux > 0.0 && magnitude > 0.0 {
        weighted_n / magnitude
    } else {
        Vector3::zeros()
    }
}

impl Module for EarthRadiationModel {
    fn init(&mut self) {
        for error in self.configuration_errors() {
            log::error!("{}: {error}", self.config.name);
        }
        self.earth_radiation_out_msg
            .write(EarthRadiationMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let radiation = self.radiation(context);
        self.earth_radiation_out_msg.write(radiation);
    }
}

/// Ports of upstream `test_earthRadiationModel.py`.
#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{EarthRadiationInputNotConnected, EarthRadiationModel, EarthRadiationModelConfig};
    use crate::environment::planet_radiation::{AlbedoDataFile, PlanetRadiationError};
    use crate::environment::test_assets::asset_or_skip;
    use crate::messages::{
        EarthRadiationMsg, Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg,
        SunEphemerisMsg,
    };
    use crate::utilities::astro_constants::{AU, REQ_EARTH, RP_EARTH};
    use crate::{Module, SimulationContext};

    const REQ_EARTH_M: f64 = REQ_EARTH * 1000.0;
    const RP_EARTH_M: f64 = RP_EARTH * 1000.0;
    const ALT_M: f64 = 600.0e3;
    const S0_WM2: f64 = 1361.0;
    const AU2M: f64 = AU * 1000.0;
    const OLR_DEFAULT: f64 = 237.0;
    const ALBEDO_DEFAULT: f64 = 0.30;
    const NLAT_DEFAULT: usize = 180;
    const NLON_DEFAULT: usize = 360;
    const CERES_1X1: &str = "albedo/Earth_ALB_2018_CERES_All_1x1.csv";

    struct Run {
        albedo_data: bool,
        eclipse_case: bool,
        sun_position_m: Vector3<f64>,
        spacecraft_position_m: Vector3<f64>,
    }

    impl Default for Run {
        /// Upstream's geometry: Earth at the origin, Sun along `+x`, the
        /// spacecraft 600 km up along `+y`.
        fn default() -> Self {
            Self {
                albedo_data: false,
                eclipse_case: false,
                sun_position_m: Vector3::new(AU2M, 0.0, 0.0),
                spacecraft_position_m: Vector3::new(0.0, REQ_EARTH_M + ALT_M, 0.0),
            }
        }
    }

    /// Upstream `_run_sim`: one update of the default 180 x 360 model.
    /// `None` when the albedo map asked for is not fetched.
    fn run_sim(run: Run) -> Option<EarthRadiationMsg> {
        let albedo_data = if run.albedo_data {
            let path = asset_or_skip(CERES_1X1)?;
            Some(AlbedoDataFile {
                data_path: path.parent().unwrap().to_path_buf(),
                file_name: path.file_name().unwrap().to_str().unwrap().to_string(),
            })
        } else {
            None
        };
        let mut erm = EarthRadiationModel::new(EarthRadiationModelConfig {
            albedo_data,
            eclipse_case: run.eclipse_case,
            ..Default::default()
        })
        .expect("valid model");

        let earth = Output::new(PlanetStateMsg {
            orientation: Some(PlanetOrientation::identity()),
            ..Default::default()
        });
        let sun = Output::new(SunEphemerisMsg {
            sun_position_inertial_m: run.sun_position_m,
            ..Default::default()
        });
        let spacecraft = Output::new(SpacecraftStateMsg {
            position_m: run.spacecraft_position_m,
            ..Default::default()
        });
        erm.planet_state_in_msg.connect(earth.slot());
        erm.sun_position_in_msg.connect(sun.slot());
        erm.spacecraft_state_in_msg.connect(spacecraft.slot());

        erm.init();
        erm.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
        Some(erm.earth_radiation_out_msg.read())
    }

    #[test]
    fn average_albedo_physics_checks() {
        let out = run_sim(Run::default()).unwrap();
        assert!(out.albedo_flux_w_per_m2 > 0.0);
        assert!(out.ir_flux_w_per_m2 > 0.0);
        assert!(out.albedo_flux_w_per_m2 < 1000.0);
        assert!(out.ir_flux_w_per_m2 < 1000.0);
        assert!((out.albedo_direction_inertial.norm() - 1.0).abs() < 1e-10);
        assert!((out.ir_direction_inertial.norm() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn albedo_map_gives_nonzero_flux() {
        let Some(out) = run_sim(Run {
            albedo_data: true,
            ..Default::default()
        }) else {
            return;
        };
        assert!(out.albedo_flux_w_per_m2 > 0.0);
        assert!(out.ir_flux_w_per_m2 > 0.0);
    }

    /// The infrared comes from below a spacecraft on `+y`, so its
    /// patch-to-spacecraft direction points up.
    #[test]
    fn ir_direction_points_away_from_the_earth() {
        let out = run_sim(Run::default()).unwrap();
        assert!(
            out.ir_direction_inertial.y > 0.0,
            "{:?}",
            out.ir_direction_inertial
        );
    }

    #[test]
    fn unconnected_inputs_are_configuration_errors() {
        let erm = EarthRadiationModel::new(EarthRadiationModelConfig::default()).unwrap();
        assert_eq!(
            erm.configuration_errors(),
            vec![
                EarthRadiationInputNotConnected("spacecraft_state_in_msg"),
                EarthRadiationInputNotConnected("sun_position_in_msg"),
                EarthRadiationInputNotConnected("planet_state_in_msg"),
            ]
        );
    }

    #[test]
    fn a_missing_albedo_file_is_an_error() {
        let result = EarthRadiationModel::new(EarthRadiationModelConfig {
            albedo_data: Some(AlbedoDataFile {
                data_path: std::env::temp_dir(),
                file_name: "does_not_exist.csv".to_string(),
            }),
            ..Default::default()
        });
        assert!(matches!(
            result,
            Err(PlanetRadiationError::AlbedoData { .. })
        ));
    }

    /// The test's authalic radius (`_authalic_radius`).
    fn authalic_radius(req_m: f64, rp_m: f64) -> f64 {
        let e = (1.0 - (rp_m / req_m).powi(2)).sqrt();
        (req_m.powi(2) * 0.5 * (1.0 + (1.0 - e * e) / (2.0 * e) * ((1.0 + e) / (1.0 - e)).ln()))
            .sqrt()
    }

    /// The test's independent Lambertian patch model (`_python_lambertian`),
    /// returning `(albedo flux, infrared flux)`.
    #[allow(clippy::too_many_arguments)]
    fn python_lambertian(
        r_sat_n: Vector3<f64>,
        r_sun_n: Vector3<f64>,
        r_planet_n: Vector3<f64>,
        j2000_to_pfix: Matrix3<f64>,
        r_planet: f64,
        olr: f64,
        s_sun: f64,
        n_lat: usize,
        n_lon: usize,
        albedo_grid: &[f64],
        req_m: f64,
        rp_m: f64,
    ) -> (f64, f64) {
        let has_aut = req_m > 0.0 && rp_m > 0.0 && req_m != rp_m;
        let mut t_aut = [0.0; 3];
        if has_aut {
            let e = (1.0 - (rp_m / req_m).powi(2)).sqrt();
            let (e2, e4, e6) = (e.powi(2), e.powi(4), e.powi(6));
            t_aut[0] = e2 / 3.0 + 31.0 * e4 / 180.0 + 59.0 * e6 / 560.0;
            t_aut[1] = 17.0 * e4 / 360.0 + 61.0 * e6 / 1260.0;
            t_aut[2] = 383.0 * e6 / 45360.0;
        }
        let aut = |x: f64| {
            if has_aut {
                x - (t_aut[0] * (2.0 * x).sin() - t_aut[1] * (4.0 * x).sin()
                    + t_aut[2] * (6.0 * x).sin())
            } else {
                x
            }
        };

        let lat_diff = (180.0 / n_lat as f64) * PI / 180.0;
        let lon_diff = (360.0 / n_lon as f64) * PI / 180.0;
        let (half_lat, half_lon) = ((n_lat / 2) as f64, (n_lon / 2) as f64);
        let dcm_np = j2000_to_pfix.transpose();
        let rs_sat = r_sat_n - r_planet_n;
        let rs_sun = r_sun_n - r_planet_n;
        let r2 = r_planet.powi(2);
        let inv_pi = 1.0 / PI;

        let (mut alb_sum, mut ir_sum) = (0.0, 0.0);
        for ilat in 0..n_lat {
            let lat = (ilat as f64 - half_lat + 0.5) * lat_diff;
            let lat1 = aut(lat + 0.5 * lat_diff);
            let lat2 = aut(lat - 0.5 * lat_diff);
            let d_a = lon_diff * (lat1.sin() - lat2.sin()).abs() * r2;
            for ilon in 0..n_lon {
                let lon = (ilon as f64 - half_lon + 0.5) * lon_diff;
                let dir_p = Vector3::new(lat.cos() * lon.cos(), lat.cos() * lon.sin(), lat.sin());
                let r_dap_n = r_planet * (dcm_np * dir_p);
                let n_hat = r_dap_n / r_dap_n.norm();
                let r_ida = rs_sat - r_dap_n;
                let d = r_ida.norm();
                if d < 1.0 {
                    continue;
                }
                let cos_sat = n_hat.dot(&(r_ida / d));
                if cos_sat <= 0.0 {
                    continue;
                }
                ir_sum += (olr * inv_pi) * cos_sat * d_a / (d * d);
                let r_sda = rs_sun - r_dap_n;
                let r_sda_n = r_sda.norm();
                if r_sda_n > 1.0 {
                    let cos_sun = n_hat.dot(&(r_sda / r_sda_n));
                    if cos_sun > 0.0 {
                        alb_sum += (s_sun * inv_pi)
                            * albedo_grid[ilat * n_lon + ilon]
                            * cos_sun
                            * cos_sat
                            * d_a
                            / (d * d);
                    }
                }
            }
        }
        (alb_sum, ir_sum)
    }

    /// Upstream `test_earthRadiation_analytical_ir`: over a Lambertian
    /// sphere, `F = 2 M (1 - sqrt(1 - (R/D)^2))`.
    #[test]
    fn ir_flux_matches_the_lambertian_sphere() {
        let out = run_sim(Run::default()).unwrap();
        let r_aut = authalic_radius(REQ_EARTH_M, RP_EARTH_M);
        let d = REQ_EARTH_M + ALT_M;
        let ir_ref = 2.0 * OLR_DEFAULT * (1.0 - (1.0 - (r_aut / d).powi(2)).sqrt());
        let rel_err = (out.ir_flux_w_per_m2 - ir_ref).abs() / ir_ref;
        assert!(
            rel_err < 0.01,
            "irFlux={:.4} vs analytical {ir_ref:.4} (rel err {:.2} %)",
            out.ir_flux_w_per_m2,
            rel_err * 100.0
        );
    }

    /// Upstream `test_earthRadiation_numpy_reference`, average albedo and
    /// CERES map.
    #[test]
    fn fluxes_match_the_independent_reference() {
        for use_albedo_data in [false, true] {
            let albedo_grid = if use_albedo_data {
                let Some(path) = asset_or_skip(CERES_1X1) else {
                    continue;
                };
                let text = std::fs::read_to_string(path).unwrap();
                let rows: Vec<Vec<f64>> = text
                    .lines()
                    .filter(|line| !line.trim().is_empty())
                    .map(|line| line.split(',').map(|v| v.trim().parse().unwrap()).collect())
                    .collect();
                assert_eq!((rows.len(), rows[0].len()), (NLAT_DEFAULT, NLON_DEFAULT));
                rows.concat()
            } else {
                vec![ALBEDO_DEFAULT; NLAT_DEFAULT * NLON_DEFAULT]
            };
            let out = run_sim(Run {
                albedo_data: use_albedo_data,
                ..Default::default()
            })
            .unwrap();

            let r_sun_n = Vector3::new(AU2M, 0.0, 0.0);
            let s_sun = S0_WM2 * (AU2M / r_sun_n.norm()).powi(2);
            let (alb_ref, ir_ref) = python_lambertian(
                Vector3::new(0.0, REQ_EARTH_M + ALT_M, 0.0),
                r_sun_n,
                Vector3::zeros(),
                Matrix3::identity(),
                authalic_radius(REQ_EARTH_M, RP_EARTH_M),
                OLR_DEFAULT,
                s_sun,
                NLAT_DEFAULT,
                NLON_DEFAULT,
                &albedo_grid,
                REQ_EARTH_M,
                RP_EARTH_M,
            );

            assert!(out.albedo_flux_w_per_m2 > 0.0 && alb_ref > 0.0);
            assert!(out.ir_flux_w_per_m2 > 0.0 && ir_ref > 0.0);
            let alb_rel = (out.albedo_flux_w_per_m2 - alb_ref).abs() / alb_ref;
            let ir_rel = (out.ir_flux_w_per_m2 - ir_ref).abs() / ir_ref;
            assert!(
                alb_rel < 1e-7,
                "albedo map {use_albedo_data}: albedo rel err {alb_rel:.2e}"
            );
            assert!(
                ir_rel < 1e-7,
                "albedo map {use_albedo_data}: IR rel err {ir_rel:.2e}"
            );
        }
    }

    /// Upstream `test_earthRadiation_eclipseCase_applied`.
    #[test]
    fn eclipse_shadowing_only_reduces_albedo() {
        let no_eclipse = run_sim(Run::default()).unwrap();
        let eclipse = run_sim(Run {
            eclipse_case: true,
            ..Default::default()
        })
        .unwrap();

        assert!(eclipse.albedo_flux_w_per_m2 >= 0.0);
        assert!(no_eclipse.albedo_flux_w_per_m2 > 0.0);
        assert!(eclipse.albedo_flux_w_per_m2 <= no_eclipse.albedo_flux_w_per_m2 + 1e-12);
        assert!(
            (eclipse.ir_flux_w_per_m2 - no_eclipse.ir_flux_w_per_m2).abs()
                < 1e-12 * no_eclipse.ir_flux_w_per_m2.max(1.0)
        );
        let rel_diff = (eclipse.albedo_flux_w_per_m2 - no_eclipse.albedo_flux_w_per_m2).abs()
            / no_eclipse.albedo_flux_w_per_m2;
        assert!(rel_diff < 0.01, "rel_diff={rel_diff:.2e}");
    }

    /// Upstream `test_earthRadiation_zero_albedo_direction_is_zero`: on the
    /// night side no patch is both sunlit and visible.
    #[test]
    fn night_side_has_no_albedo_and_a_zero_direction() {
        let out = run_sim(Run {
            spacecraft_position_m: Vector3::new(-REQ_EARTH_M - ALT_M, 0.0, 0.0),
            ..Default::default()
        })
        .unwrap();
        assert!(out.albedo_flux_w_per_m2.abs() <= 1e-12);
        assert!(out.albedo_direction_inertial.amax() <= 1e-12);
        assert!(out.ir_flux_w_per_m2 > 0.0);
    }

    /// Upstream `test_earthRadiation_albedoDir_geometry`.
    #[test]
    fn albedo_direction_points_towards_the_spacecraft() {
        let out = run_sim(Run::default()).unwrap();
        assert!(out.albedo_flux_w_per_m2 > 0.0);
        assert!((out.albedo_direction_inertial.norm() - 1.0).abs() < 1e-10);
        assert!(out.albedo_direction_inertial.y > 0.0);
    }

    /// Upstream `test_earthRadiation_solar_flux_distance_scaling`.
    #[test]
    fn albedo_scales_with_the_inverse_square_sun_distance() {
        let at_1au = run_sim(Run::default()).unwrap();
        let at_2au = run_sim(Run {
            sun_position_m: Vector3::new(2.0 * AU2M, 0.0, 0.0),
            ..Default::default()
        })
        .unwrap();

        assert!(at_1au.albedo_flux_w_per_m2 > 0.0 && at_2au.albedo_flux_w_per_m2 > 0.0);
        let ratio = at_2au.albedo_flux_w_per_m2 / at_1au.albedo_flux_w_per_m2;
        assert!((ratio - 0.25).abs() <= 1e-3 * 0.25, "ratio {ratio:.6e}");
        assert!(
            (at_1au.ir_flux_w_per_m2 - at_2au.ir_flux_w_per_m2).abs()
                <= 1e-10 * at_2au.ir_flux_w_per_m2
        );
    }
}
