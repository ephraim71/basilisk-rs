use std::fmt;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anise::almanac::Almanac;
use anise::frames::Frame;
use hifitime::Epoch;
use nalgebra::Vector3;

use crate::messages::{AtmosphereMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

use super::nrlmsise::nrlmsise_with_inputs;

#[derive(Clone, Debug)]
pub struct MsisAtmosphereConfig {
    pub name: String,
    pub first_kernel_path: PathBuf,
    pub additional_kernel_paths: Vec<PathBuf>,
    pub inertial_frame: Frame,
    pub fixed_frame: Frame,
    /// Geomagnetic activity inputs. Basilisk's default NRLMSISE switch-9 mode
    /// consumes the daily scalar in slot 0; the history slots are retained for
    /// message/configuration parity but are not used in that mode.
    pub ap_array: [f64; 7],
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
    timing_enabled: bool,
    timing_stats: MsisAtmosphereTimingStats,
}

#[derive(Clone, Debug, Default)]
pub struct MsisAtmosphereTimingStats {
    pub update_calls: u64,
    pub total_update_nanos: u128,
    pub read_state_nanos: u128,
    pub read_planet_nanos: u128,
    pub anise_rotation_nanos: u128,
    pub geodetic_nanos: u128,
    pub msis_eval_nanos: u128,
    pub output_write_nanos: u128,
}

impl Module for MsisAtmosphere {
    fn init(&mut self) {
        self.output_atmosphere_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let total_started = self.timing_enabled.then(Instant::now);

        let phase_started = self.timing_enabled.then(Instant::now);
        let state = self.input_state_msg.read();
        if let Some(started) = phase_started {
            self.timing_stats.read_state_nanos += started.elapsed().as_nanos();
        }

        let atmosphere = self.evaluate_msis(&state, context.current_epoch);

        let phase_started = self.timing_enabled.then(Instant::now);
        self.output_atmosphere_msg.write(atmosphere);
        if let Some(started) = phase_started {
            self.timing_stats.output_write_nanos += started.elapsed().as_nanos();
        }

        if let Some(started) = total_started {
            self.timing_stats.update_calls += 1;
            self.timing_stats.total_update_nanos += started.elapsed().as_nanos();
        }
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
            timing_enabled: false,
            timing_stats: MsisAtmosphereTimingStats::default(),
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    pub fn timing_stats(&self) -> &MsisAtmosphereTimingStats {
        &self.timing_stats
    }

    fn evaluate_msis(&mut self, state: &SpacecraftStateMsg, current_epoch: Epoch) -> AtmosphereMsg {
        let spacecraft_position_inertial_m = state.position_m;

        let phase_started = self.timing_enabled.then(Instant::now);
        let planet_position_inertial_m = if self.input_planet_msg.is_connected() {
            self.input_planet_msg.read().position_inertial_m
        } else {
            Vector3::zeros()
        };
        if let Some(started) = phase_started {
            self.timing_stats.read_planet_nanos += started.elapsed().as_nanos();
        }

        let relative_position_inertial_m =
            spacecraft_position_inertial_m - planet_position_inertial_m;

        let relative_position_fixed_m = if self.input_planet_msg.is_connected() {
            let planet_state = self.input_planet_msg.read();
            if let Some(orientation) = planet_state.orientation {
                orientation.inertial_to_fixed * relative_position_inertial_m
            } else {
                self.rotate_inertial_to_fixed(relative_position_inertial_m, current_epoch)
            }
        } else {
            self.rotate_inertial_to_fixed(relative_position_inertial_m, current_epoch)
        };

        let phase_started = self.timing_enabled.then(Instant::now);
        let (latitude_rad, longitude_rad, altitude_m) =
            ecef_to_spherical_lla(relative_position_fixed_m);
        if let Some(started) = phase_started {
            self.timing_stats.geodetic_nanos += started.elapsed().as_nanos();
        }

        if altitude_m < 0.0 {
            return AtmosphereMsg::default();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        let (density_kgpm3, local_temp_k) = nrlmsise_with_inputs(
            altitude_m / 1_000.0,
            latitude_rad.to_degrees(),
            longitude_rad.to_degrees(),
            current_epoch.day_of_year().floor() as i32,
            seconds_of_day(current_epoch),
            self.config.f107_daily,
            self.config.f107_average,
            self.config.ap_array[0],
            // Basilisk's MsisAtmosphere keeps NRLMSISE switch 9 at +1, so the
            // daily scalar Ap value is used even though the history structure
            // is populated from its space-weather messages.
            None,
        );
        if let Some(started) = phase_started {
            self.timing_stats.msis_eval_nanos += started.elapsed().as_nanos();
        }

        AtmosphereMsg {
            neutral_density_kgpm3: density_kgpm3,
            local_temp_k,
        }
    }

    /// Rotate an inertial position into the planet-fixed frame using the
    /// orientation model, when the planet message carries no orientation.
    fn rotate_inertial_to_fixed(
        &mut self,
        relative_position_inertial_m: Vector3<f64>,
        current_epoch: Epoch,
    ) -> Vector3<f64> {
        let phase_started = self.timing_enabled.then(Instant::now);
        let rotation = self
            .orientation_model
            .almanac
            .rotate(
                self.orientation_model.inertial_frame,
                self.orientation_model.fixed_frame,
                current_epoch,
            )
            .expect("failed to compute ANISE Earth-fixed rotation for MSIS");
        if let Some(started) = phase_started {
            self.timing_stats.anise_rotation_nanos += started.elapsed().as_nanos();
        }
        apply_anise_rotation(&rotation.rot_mat, relative_position_inertial_m)
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

fn ecef_to_spherical_lla(position_fixed_m: Vector3<f64>) -> (f64, f64, f64) {
    // Upstream calls PCI2LLA with only the equatorial radius. That overload
    // deliberately treats Earth as a sphere rather than using WGS-84.
    const EARTH_EQUATORIAL_RADIUS_M: f64 = 6_378_136.6;
    let x = position_fixed_m.x;
    let y = position_fixed_m.y;
    let z = position_fixed_m.z;
    let longitude_rad = y.atan2(x);
    let cylindrical_radius_m = x.hypot(y);
    let latitude_rad = z.atan2(cylindrical_radius_m);
    let altitude_m = position_fixed_m.norm() - EARTH_EQUATORIAL_RADIUS_M;

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
