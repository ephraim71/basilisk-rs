use std::fmt;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anise::almanac::Almanac;
use anise::frames::Frame;
use hifitime::Epoch;
use igrf::Error as IgrfError;
use nalgebra::{Matrix3, Vector3};
use time::{Date, Month};

use crate::messages::{Input, MagneticFieldMsg, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct IgrfFieldConfig {
    pub name: String,
    pub first_kernel_path: PathBuf,
    pub additional_kernel_paths: Vec<PathBuf>,
    pub inertial_frame: Frame,
    pub fixed_frame: Frame,
}

#[derive(Clone, Debug)]
pub struct IgrfField {
    pub config: IgrfFieldConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_planet_msg: Input<PlanetStateMsg>,
    pub output_magnetic_field_msg: Output<MagneticFieldMsg>,
    orientation_model: AniseOrientationModel,
    timing_enabled: bool,
    timing_stats: IgrfFieldTimingStats,
}

#[derive(Clone, Debug, Default)]
pub struct IgrfFieldTimingStats {
    pub update_calls: u64,
    pub total_update_nanos: u128,
    pub read_input_nanos: u128,
    pub anise_rotation_nanos: u128,
    pub geodetic_nanos: u128,
    pub igrf_eval_nanos: u128,
    pub frame_transform_nanos: u128,
    pub output_write_nanos: u128,
}

impl Module for IgrfField {
    fn init(&mut self) {
        self.output_magnetic_field_msg
            .write(MagneticFieldMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let total_started = self.timing_enabled.then(Instant::now);

        let phase_started = self.timing_enabled.then(Instant::now);
        let state = self.read_input_message();
        if let Some(started) = phase_started {
            self.timing_stats.read_input_nanos += started.elapsed().as_nanos();
        }

        let magnetic_field_inertial_t = self.compute_magnetic_field(&state, context.current_epoch);

        let phase_started = self.timing_enabled.then(Instant::now);
        self.write_output_message(magnetic_field_inertial_t);
        if let Some(started) = phase_started {
            self.timing_stats.output_write_nanos += started.elapsed().as_nanos();
        }

        if let Some(started) = total_started {
            self.timing_stats.update_calls += 1;
            self.timing_stats.total_update_nanos += started.elapsed().as_nanos();
        }
    }
}

impl IgrfField {
    pub fn new(config: IgrfFieldConfig) -> Self {
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
            output_magnetic_field_msg: Output::default(),
            orientation_model,
            timing_enabled: false,
            timing_stats: IgrfFieldTimingStats::default(),
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    pub fn timing_stats(&self) -> &IgrfFieldTimingStats {
        &self.timing_stats
    }

    fn read_input_message(&self) -> SpacecraftStateMsg {
        self.input_state_msg.read()
    }

    fn compute_magnetic_field(
        &mut self,
        state: &SpacecraftStateMsg,
        current_epoch: Epoch,
    ) -> Vector3<f64> {
        let (position_inertial_m, inertial_to_fixed_from_msg) =
            if self.input_planet_msg.is_connected() {
                let planet_state = self.input_planet_msg.read();
                let relative_position_inertial_m =
                    state.position_m - planet_state.position_inertial_m;
                if planet_state.has_orientation {
                    (
                        relative_position_inertial_m,
                        Some(planet_state.inertial_to_fixed),
                    )
                } else {
                    (relative_position_inertial_m, None)
                }
            } else {
                (state.position_m, None)
            };

        let mut fallback_inertial_to_fixed = None;
        let position_fixed_m = if let Some(inertial_to_fixed) = inertial_to_fixed_from_msg {
            inertial_to_fixed * position_inertial_m
        } else {
            let phase_started = self.timing_enabled.then(Instant::now);
            let rotation = self
                .orientation_model
                .almanac
                .rotate(
                    self.orientation_model.inertial_frame,
                    self.orientation_model.fixed_frame,
                    current_epoch,
                )
                .expect("failed to compute ANISE Earth-fixed rotation for IGRF");
            if let Some(started) = phase_started {
                self.timing_stats.anise_rotation_nanos += started.elapsed().as_nanos();
            }
            fallback_inertial_to_fixed = Some(matrix3_from_anise_rotation(&rotation.rot_mat));
            apply_anise_rotation(&rotation.rot_mat, position_inertial_m)
        };

        let phase_started = self.timing_enabled.then(Instant::now);
        let (latitude_rad, longitude_rad, altitude_m) = ecef_to_geodetic(position_fixed_m);
        if let Some(started) = phase_started {
            self.timing_stats.geodetic_nanos += started.elapsed().as_nanos();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        let field_ned_t =
            local_ned_field_from_igrf(latitude_rad, longitude_rad, altitude_m, current_epoch);
        if let Some(started) = phase_started {
            self.timing_stats.igrf_eval_nanos += started.elapsed().as_nanos();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        let field_fixed_t = ned_to_ecef(latitude_rad, longitude_rad, field_ned_t);
        let field_inertial_t = if let Some(inertial_to_fixed) =
            inertial_to_fixed_from_msg.or(fallback_inertial_to_fixed)
        {
            inertial_to_fixed.transpose() * field_fixed_t
        } else {
            field_fixed_t
        };
        if let Some(started) = phase_started {
            self.timing_stats.frame_transform_nanos += started.elapsed().as_nanos();
        }

        field_inertial_t
    }

    fn write_output_message(&mut self, magnetic_field_inertial_t: Vector3<f64>) {
        self.output_magnetic_field_msg.write(MagneticFieldMsg {
            magnetic_field_inertial_t,
        });
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

fn local_ned_field_from_igrf(
    latitude_rad: f64,
    longitude_rad: f64,
    altitude_m: f64,
    current_epoch: Epoch,
) -> Vector3<f64> {
    let date = igrf_date_from_epoch(current_epoch);
    let field = match igrf::declination(
        latitude_rad.to_degrees(),
        longitude_rad.to_degrees(),
        altitude_m.max(0.0).round() as u32,
        date,
    ) {
        Ok(field) => field,
        Err(IgrfError::DateOutOfRange(field)) => field,
        Err(error) => panic!("failed to evaluate IGRF field: {error}"),
    };

    Vector3::new(field.x, field.y, field.z) * 1.0e-9
}

fn igrf_date_from_epoch(current_epoch: Epoch) -> Date {
    let (year, month, day, _, _, _, _) = current_epoch.to_gregorian_utc();
    Date::from_calendar_date(
        year,
        Month::try_from(month).expect("invalid Gregorian month from hifitime"),
        day,
    )
    .expect("invalid Gregorian date from hifitime")
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

fn ned_to_ecef(latitude_rad: f64, longitude_rad: f64, field_ned_t: Vector3<f64>) -> Vector3<f64> {
    let sin_lat = latitude_rad.sin();
    let cos_lat = latitude_rad.cos();
    let sin_lon = longitude_rad.sin();
    let cos_lon = longitude_rad.cos();

    let north = Vector3::new(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    let east = Vector3::new(-sin_lon, cos_lon, 0.0);
    let down = Vector3::new(-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat);

    north * field_ned_t.x + east * field_ned_t.y + down * field_ned_t.z
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

fn matrix3_from_anise_rotation(rotation: &anise::math::Matrix3) -> Matrix3<f64> {
    Matrix3::new(
        rotation[(0, 0)],
        rotation[(0, 1)],
        rotation[(0, 2)],
        rotation[(1, 0)],
        rotation[(1, 1)],
        rotation[(1, 2)],
        rotation[(2, 0)],
        rotation[(2, 1)],
        rotation[(2, 2)],
    )
}
