use std::fmt;
use std::path::{Path, PathBuf};

use anise::almanac::Almanac;
use anise::frames::Frame;
use hifitime::Epoch;
use nalgebra::Vector3;

use crate::messages::{AtmosphereMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::nrlmsise::nrlmsise_with_inputs;
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct ExponentialAtmosphereConfig {
    pub name: String,
    pub planet_radius_m: f64,
    pub reference_altitude_m: f64,
    pub reference_density_kgpm3: f64,
    pub scale_height_m: f64,
}

pub struct ExponentialAtmosphere {
    pub config: ExponentialAtmosphereConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub output_atmosphere_msg: Output<AtmosphereMsg>,
}

impl Module for ExponentialAtmosphere {
    fn init(&mut self) {
        self.output_atmosphere_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        let state = self.input_state_msg.read();
        let radius_m = state.position_m.norm();
        let altitude_m = (radius_m - self.config.planet_radius_m).max(0.0);
        let density = if self.config.scale_height_m > 0.0 {
            self.config.reference_density_kgpm3
                * (-(altitude_m - self.config.reference_altitude_m) / self.config.scale_height_m)
                    .exp()
        } else {
            0.0
        };

        self.output_atmosphere_msg.write(AtmosphereMsg {
            neutral_density_kgpm3: density,
            local_temp_k: 0.0,
        });
    }
}

impl ExponentialAtmosphere {
    pub fn new(config: ExponentialAtmosphereConfig) -> Self {
        Self {
            config,
            input_state_msg: Input::default(),
            output_atmosphere_msg: Output::default(),
        }
    }
}

#[derive(Clone, Debug)]
pub struct MsisAtmosphereConfig {
    pub name: String,
    pub planet_radius_m: f64,
    pub first_kernel_path: PathBuf,
    pub additional_kernel_paths: Vec<PathBuf>,
    pub inertial_frame: Frame,
    pub fixed_frame: Frame,
    pub ap_daily: f64,
    pub ap_3hr: f64,
    pub f107_daily: f64,
    pub f107_average: f64,
}

#[derive(Clone, Debug)]
pub struct MsisAtmosphere {
    pub config: MsisAtmosphereConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_planet_msg: Input<PlanetStateMsg>,
    pub output_atmosphere_msg: Output<AtmosphereMsg>,
    orientation_model: AniseOrientationModel,
}

impl Module for MsisAtmosphere {
    fn init(&mut self) {
        self.output_atmosphere_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.input_state_msg.read();
        let atmosphere = self.evaluate_msis(&state, context.current_epoch);
        self.output_atmosphere_msg.write(atmosphere);
    }
}

impl MsisAtmosphere {
    pub fn new(config: MsisAtmosphereConfig) -> Self {
        let orientation_model = AniseOrientationModel::new(
            &config.first_kernel_path,
            &config.additional_kernel_paths,
            config.inertial_frame,
            config.fixed_frame,
        );

        Self {
            config,
            input_state_msg: Input::default(),
            input_planet_msg: Input::default(),
            output_atmosphere_msg: Output::default(),
            orientation_model,
        }
    }

    fn evaluate_msis(&self, state: &SpacecraftStateMsg, current_epoch: Epoch) -> AtmosphereMsg {
        let spacecraft_position_inertial_m = state.position_m;
        let planet_position_inertial_m = if self.input_planet_msg.is_connected() {
            self.input_planet_msg.read().position_inertial_m
        } else {
            Vector3::zeros()
        };
        let relative_position_inertial_m =
            spacecraft_position_inertial_m - planet_position_inertial_m;
        let rotation = self
            .orientation_model
            .almanac
            .rotate(
                self.orientation_model.inertial_frame,
                self.orientation_model.fixed_frame,
                current_epoch,
            )
            .expect("failed to compute ANISE Earth-fixed rotation for MSIS");
        let relative_position_fixed_m =
            apply_anise_rotation(&rotation.rot_mat, relative_position_inertial_m);
        let (latitude_rad, longitude_rad, altitude_m) = ecef_to_geodetic(relative_position_fixed_m);

        if altitude_m < 0.0 {
            return AtmosphereMsg::default();
        }

        let (density_kgpm3, local_temp_k) = nrlmsise_with_inputs(
            altitude_m / 1_000.0,
            latitude_rad.to_degrees(),
            longitude_rad.to_degrees(),
            current_epoch.day_of_year().floor() as i32,
            seconds_of_day(current_epoch),
            self.config.f107_daily,
            self.config.f107_average,
            self.config.ap_daily,
            Some([
                self.config.ap_daily,
                self.config.ap_3hr,
                self.config.ap_3hr,
                self.config.ap_3hr,
                self.config.ap_3hr,
                self.config.ap_3hr,
                self.config.ap_3hr,
            ]),
        );

        AtmosphereMsg {
            neutral_density_kgpm3: density_kgpm3,
            local_temp_k,
        }
    }
}

#[derive(Clone)]
struct AniseOrientationModel {
    almanac: Almanac,
    inertial_frame: Frame,
    fixed_frame: Frame,
}

impl AniseOrientationModel {
    fn new(
        first_kernel_path: impl AsRef<Path>,
        additional_kernel_paths: &[PathBuf],
        inertial_frame: Frame,
        fixed_frame: Frame,
    ) -> Self {
        let first_kernel_path = resolve_repo_relative_path(first_kernel_path.as_ref());
        let mut almanac = Almanac::new(&first_kernel_path.to_string_lossy())
            .expect("failed to load ANISE kernel");
        for kernel_path in additional_kernel_paths {
            let resolved_path = resolve_repo_relative_path(kernel_path);
            almanac = almanac
                .load(&resolved_path.to_string_lossy())
                .expect("failed to load additional ANISE kernel");
        }

        Self {
            almanac,
            inertial_frame,
            fixed_frame,
        }
    }
}

impl fmt::Debug for AniseOrientationModel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("AniseOrientationModel")
            .field("inertial_frame", &self.inertial_frame)
            .field("fixed_frame", &self.fixed_frame)
            .finish()
    }
}

fn seconds_of_day(current_epoch: Epoch) -> f64 {
    let (_, _, _, hour, minute, second, nanos) = current_epoch.to_gregorian_utc();
    hour as f64 * 3600.0 + minute as f64 * 60.0 + second as f64 + nanos as f64 * 1.0e-9
}

fn ecef_to_geodetic(position_fixed_m: Vector3<f64>) -> (f64, f64, f64) {
    const WGS84_A_M: f64 = 6_378_137.0;
    const WGS84_F: f64 = 1.0 / 298.257_223_563;
    const WGS84_B_M: f64 = WGS84_A_M * (1.0 - WGS84_F);
    const WGS84_E2: f64 = WGS84_F * (2.0 - WGS84_F);
    const WGS84_EP2: f64 =
        (WGS84_A_M * WGS84_A_M - WGS84_B_M * WGS84_B_M) / (WGS84_B_M * WGS84_B_M);

    let x = position_fixed_m.x;
    let y = position_fixed_m.y;
    let z = position_fixed_m.z;
    let longitude_rad = y.atan2(x);
    let p = (x * x + y * y).sqrt();

    if p < 1.0e-9 {
        let latitude_rad = if z >= 0.0 {
            std::f64::consts::FRAC_PI_2
        } else {
            -std::f64::consts::FRAC_PI_2
        };
        return (latitude_rad, 0.0, z.abs() - WGS84_B_M);
    }

    let theta = (z * WGS84_A_M).atan2(p * WGS84_B_M);
    let sin_theta = theta.sin();
    let cos_theta = theta.cos();
    let latitude_rad = (z + WGS84_EP2 * WGS84_B_M * sin_theta.powi(3))
        .atan2(p - WGS84_E2 * WGS84_A_M * cos_theta.powi(3));
    let sin_lat = latitude_rad.sin();
    let radius_curvature = WGS84_A_M / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    let altitude_m = p / latitude_rad.cos() - radius_curvature;

    (latitude_rad, longitude_rad, altitude_m)
}

fn apply_anise_rotation(rot_mat: &anise::math::Matrix3, vector: Vector3<f64>) -> Vector3<f64> {
    Vector3::new(
        rot_mat[(0, 0)] * vector.x + rot_mat[(0, 1)] * vector.y + rot_mat[(0, 2)] * vector.z,
        rot_mat[(1, 0)] * vector.x + rot_mat[(1, 1)] * vector.y + rot_mat[(1, 2)] * vector.z,
        rot_mat[(2, 0)] * vector.x + rot_mat[(2, 1)] * vector.y + rot_mat[(2, 2)] * vector.z,
    )
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}
