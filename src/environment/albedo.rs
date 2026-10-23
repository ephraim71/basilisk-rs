use std::f64::consts::{FRAC_1_PI, FRAC_PI_2, PI};
use std::path::{Path, PathBuf};

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    AlbedoMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
};
use crate::{Module, SimulationContext};

const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_693.0;
const SOLAR_FLUX_AT_EARTH_WPM2: f64 = 1361.0;

#[derive(Clone, Debug)]
pub struct AlbedoInstrument {
    pub fov_half_angle_rad: f64,
    pub normal_body: Vector3<f64>,
    pub position_body_m: Vector3<f64>,
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
        }
    }
}

/// Port of Basilisk's `albedo`, average or lookup-table model. The planet input is
/// optional: unconnected means the planet sits at the origin with no rotation.
pub struct Albedo {
    pub config: AlbedoConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_sun_msg: Input<SunEphemerisMsg>,
    pub input_planet_msg: Input<PlanetStateMsg>,
    pub output_albedo_msgs: Vec<Output<AlbedoMsg>>,
    radius_m: f64,
    patch_dirs_fixed: Vec<Vector3<f64>>,
    patch_albedo_area_sr: Vec<f64>,
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
        self.build_grid();
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

        let fixed_to_inertial = planet
            .orientation
            .as_ref()
            .map(|o| o.inertial_to_fixed)
            .filter(|m| m.determinant() != 0.0)
            .unwrap_or_else(Matrix3::identity)
            .transpose();
        let sun_from_planet = sun.sun_position_inertial_m - planet.position_inertial_m;
        assert!(
            sun_from_planet.norm() > 0.0,
            "albedo {}: sun and planet positions coincide",
            self.config.name
        );
        let patches: Vec<Vector3<f64>> = self
            .patch_dirs_fixed
            .iter()
            .map(|dir| self.radius_m * (fixed_to_inertial * dir))
            .collect();
        let body_to_inertial = state.body_to_inertial();

        for (instrument, output) in self.config.instruments.iter().zip(&self.output_albedo_msgs) {
            let instrument_from_planet = state.position_m
                + body_to_inertial.transform_vector(&instrument.position_body_m)
                - planet.position_inertial_m;
            let normal = body_to_inertial.transform_vector(&instrument.normal_body.normalize());
            let cos_fov = instrument.fov_half_angle_rad.cos();

            let (mut ratio_max, mut ratio) = (0.0, 0.0);
            for (patch, albedo_area_sr) in patches.iter().zip(&self.patch_albedo_area_sr) {
                let patch_normal = patch.normalize();
                let to_instrument = instrument_from_planet - patch;
                let distance_m = to_instrument.norm();
                let to_instrument_hat = to_instrument / distance_m;
                let f2 = patch_normal.dot(&to_instrument_hat);
                let f1 = patch_normal.dot(&(sun_from_planet - patch).normalize());
                if f2 <= 0.0 || f1 <= 0.0 {
                    continue;
                }
                let patch_ratio = FRAC_1_PI * f1 * f2 * albedo_area_sr * self.radius_m.powi(2)
                    / distance_m.powi(2);
                ratio_max += patch_ratio;
                let f3 = -normal.dot(&to_instrument_hat);
                if f3 >= cos_fov {
                    ratio += patch_ratio * f3;
                }
            }

            let solar_flux = SOLAR_FLUX_AT_EARTH_WPM2 * ASTRONOMICAL_UNIT_M.powi(2)
                / (sun_from_planet - instrument_from_planet).norm_squared();
            output.write(AlbedoMsg {
                albedo_at_instrument_max: ratio_max,
                albedo_flux_at_instrument_max_wpm2: solar_flux * ratio_max,
                albedo_at_instrument: ratio,
                albedo_flux_at_instrument_wpm2: solar_flux * ratio,
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
        Self {
            config,
            input_state_msg: Input::default(),
            input_sun_msg: Input::default(),
            input_planet_msg: Input::default(),
            output_albedo_msgs,
            radius_m: 0.0,
            patch_dirs_fixed: Vec::new(),
            patch_albedo_area_sr: Vec::new(),
        }
    }

    fn build_grid(&mut self) {
        let c = &self.config;
        let (num_lat, num_lon) = match &c.albedo_map {
            Some(map) => (map.len(), map.first().map_or(0, Vec::len)),
            None => (c.num_lat, c.num_lon),
        };
        assert!(num_lat > 0 && num_lon > 0, "albedo grid must be non-empty");
        if let Some(map) = &c.albedo_map {
            assert!(
                map.iter()
                    .all(|row| row.len() == num_lon && row.iter().all(|a| a.is_finite())),
                "albedo map must be rectangular and finite"
            );
        }
        assert!(
            c.equatorial_radius_m > 0.0,
            "equatorial radius must be positive"
        );
        let oblate = c.polar_radius_m > 0.0 && c.polar_radius_m != c.equatorial_radius_m;
        assert!(
            !oblate || c.polar_radius_m < c.equatorial_radius_m,
            "polar radius must be less than equatorial radius"
        );

        // Authalic sphere: equal-area latitudes and radius.
        let (mut t, mut radius_m) = ([0.0; 3], c.equatorial_radius_m);
        if oblate {
            let e2 = 1.0 - (c.polar_radius_m / c.equatorial_radius_m).powi(2);
            let e = e2.sqrt();
            let (e4, e6) = (e2 * e2, e2 * e2 * e2);
            t = [
                e2 / 3.0 + 31.0 * e4 / 180.0 + 59.0 * e6 / 560.0,
                17.0 * e4 / 360.0 + 61.0 * e6 / 1260.0,
                383.0 * e6 / 45360.0,
            ];
            radius_m = (c.equatorial_radius_m.powi(2)
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
        self.radius_m = radius_m;
        self.patch_dirs_fixed.clear();
        self.patch_albedo_area_sr.clear();
        for i in 0..num_lat {
            let lat = (i as f64 - half_lat + 0.5) * lat_step;
            let area_sr = lon_step
                * (authalic(lat + 0.5 * lat_step).sin() - authalic(lat - 0.5 * lat_step).sin())
                    .abs();
            for j in 0..num_lon {
                let lon = (j as f64 - half_lon + 0.5) * lon_step;
                self.patch_dirs_fixed.push(Vector3::new(
                    lat.cos() * lon.cos(),
                    lat.cos() * lon.sin(),
                    lat.sin(),
                ));
                let albedo = c.albedo_map.as_ref().map_or(c.albedo_avg, |map| map[i][j]);
                self.patch_albedo_area_sr.push(albedo * area_sr);
            }
        }
    }
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
        Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg, SunEphemerisMsg,
    };
    use crate::{Module, SimulationContext};

    use super::{Albedo, AlbedoConfig, AlbedoInstrument, load_albedo_map};

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
        albedo.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        });
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
            albedo.build_grid();
            let area: f64 = albedo.patch_albedo_area_sr.iter().sum::<f64>() / 0.29;
            let centroid: Vector3<f64> = albedo.patch_dirs_fixed.iter().sum();
            assert!((area - 4.0 * PI).abs() < 1.0e-12, "area {area}");
            assert!(centroid.norm() < 1.0e-12, "centroid {centroid}");
        }
    }

    #[test]
    #[should_panic(expected = "rectangular")]
    fn ragged_map_is_rejected() {
        let mut config = AlbedoConfig::earth("albedo", instruments());
        config.albedo_map = Some(vec![vec![0.3, 0.3], vec![0.3]]);
        Albedo::new(config).build_grid();
    }

    #[test]
    #[should_panic(expected = "must be connected")]
    fn unconnected_inputs_are_rejected() {
        Albedo::new(AlbedoConfig::earth("albedo", instruments())).init();
    }

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
    }
}
