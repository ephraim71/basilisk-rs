//! NRLMSISE-00 neutral atmosphere: port of upstream Basilisk
//! `MsisAtmosphere`.
//!
//! # Space weather
//!
//! Upstream reads its solar and geomagnetic indices from the 23
//! `swDataInMsgs`, in the channel order `spaceWeatherData` publishes them
//! ([`SpaceWeatherData::sw_data_out_msgs`]): 0 the daily Ap, 1 to 20 the
//! 3-hour Ap history, 21 the 81-day average F10.7 and 22 the previous day's
//! F10.7. Here they are [`MsisAtmosphere::sw_data_in_msgs`]. With every one
//! connected, the module takes the daily Ap from channel 0 and the F10.7
//! values from channels 21 and 22, as upstream's `updateSwIndices` does.
//! NRLMSISE-00 switch 9 stays at +1 (upstream's default, the daily Ap), so
//! the history channels must be connected but do not change the result.
//!
//! With none connected, the indices come from the configuration
//! ([`MsisAtmosphereConfig::ap_array`], `f107_daily`, `f107_average`), a
//! basilisk-rs extension for runs without a space-weather source. Upstream
//! requires all 23 messages; a partly connected set logs an error at init and
//! writes a zero atmosphere, as upstream does when a space-weather message is
//! missing. Stand-in for upstream's `isWritten`: connected ⇒ written.
//!
//! [`SpaceWeatherData::sw_data_out_msgs`]: crate::environment::space_weather_data::SpaceWeatherData::sw_data_out_msgs

use std::fmt;
use std::path::{Path, PathBuf};
use std::time::Instant;

use anise::almanac::Almanac;
use anise::frames::Frame;
use hifitime::Epoch;
use nalgebra::Vector3;

use crate::environment::space_weather_data::NUM_SW_MESSAGES;
use crate::messages::{
    AtmosphereMsg, Input, Output, PlanetStateMsg, SpacecraftStateMsg, SwDataMsg,
};
use crate::{Module, SimulationContext};

use super::nrlmsise::nrlmsise_with_inputs;

#[derive(Clone, Debug)]
pub struct MsisAtmosphereConfig {
    pub name: String,
    pub first_kernel_path: PathBuf,
    pub additional_kernel_paths: Vec<PathBuf>,
    pub inertial_frame: Frame,
    pub fixed_frame: Frame,
    /// Geomagnetic activity inputs, used while
    /// [`MsisAtmosphere::sw_data_in_msgs`] are unconnected. The default
    /// NRLMSISE switch-9 mode consumes the daily scalar in slot 0; the history
    /// slots are retained for configuration parity but are not used in that
    /// mode.
    pub ap_array: [f64; 7],
    /// Previous day's F10.7 \[sfu\], used while the space-weather inputs are
    /// unconnected (upstream channel 22, `f107_24_-24`).
    pub f107_daily: f64,
    /// 81-day average F10.7 \[sfu\], used while the space-weather inputs are
    /// unconnected (upstream channel 21, `f107_1944_0`).
    pub f107_average: f64,
}

/// The NRLMSISE-00 indices of one evaluation (upstream `ap`, `f107`,
/// `f107A`).
#[derive(Clone, Copy, Debug, PartialEq)]
struct SpaceWeatherIndices {
    ap: f64,
    f107: f64,
    f107_average: f64,
}

#[derive(Clone, Debug)]
pub struct MsisAtmosphere {
    pub config: MsisAtmosphereConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_planet_msg: Input<PlanetStateMsg>,
    /// Upstream `swDataInMsgs`: the 23 space-weather channels, in the order
    /// of [`SpaceWeatherData::sw_data_out_msgs`]; see the module docs.
    ///
    /// [`SpaceWeatherData::sw_data_out_msgs`]: crate::environment::space_weather_data::SpaceWeatherData::sw_data_out_msgs
    pub sw_data_in_msgs: [Input<SwDataMsg>; NUM_SW_MESSAGES],
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
        // Upstream `customReset`: every space-weather message is required.
        // Here none connected selects the configured indices instead.
        if self.sw_data_in_msgs.iter().any(Input::is_connected) {
            for (index, input) in self.sw_data_in_msgs.iter().enumerate() {
                if !input.is_connected() {
                    log::error!(
                        "MSIS atmosphere '{}': required space-weather input No. {index} is not \
                         connected",
                        self.config.name
                    );
                }
            }
        }
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
            sw_data_in_msgs: std::array::from_fn(|_| Input::default()),
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

    /// Upstream `customReadMessages` and `updateSwIndices`: the indices from
    /// the space-weather inputs when all are connected, the configured ones
    /// when none is, and `None` (upstream's failed read) otherwise.
    fn space_weather_indices(&self) -> Option<SpaceWeatherIndices> {
        let connected = self
            .sw_data_in_msgs
            .iter()
            .filter(|input| input.is_connected())
            .count();
        if connected == 0 {
            return Some(SpaceWeatherIndices {
                ap: self.config.ap_array[0],
                f107: self.config.f107_daily,
                f107_average: self.config.f107_average,
            });
        }
        if connected < NUM_SW_MESSAGES {
            return None;
        }
        let channel = |index: usize| self.sw_data_in_msgs[index].read().data_value;
        Some(SpaceWeatherIndices {
            ap: channel(0),
            f107: channel(22),
            f107_average: channel(21),
        })
    }

    fn evaluate_msis(&mut self, state: &SpacecraftStateMsg, current_epoch: Epoch) -> AtmosphereMsg {
        // Upstream writes the zero payload when a required message is missing.
        let Some(indices) = self.space_weather_indices() else {
            return AtmosphereMsg::default();
        };
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
            indices.f107,
            indices.f107_average,
            indices.ap,
            // NRLMSISE switch 9 is kept at +1 (upstream's default), so the
            // daily scalar Ap is used and the 3-hour history is not.
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

#[cfg(test)]
mod tests {
    //! The space-weather wiring. The checks follow upstream's
    //! `test_msisApHistory.py` with the history off (upstream's default):
    //! the same spacecraft on the equator, a planet-fixed frame equal to the
    //! inertial one, F10.7 150 sfu and Ap 15 on every channel unless changed.

    use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{MsisAtmosphere, MsisAtmosphereConfig};
    use crate::environment::space_weather_data::NUM_SW_MESSAGES;
    use crate::environment::test_assets::asset_or_skip;
    use crate::messages::{
        AtmosphereMsg, Output, PlanetOrientation, PlanetStateMsg, SpacecraftStateMsg, SwDataMsg,
    };
    use crate::{Module, SimulationContext};

    /// Upstream `F107` \[sfu\] and `AP_QUIET` \[-\].
    const F107: f64 = 150.0;
    const AP_QUIET: f64 = 15.0;

    /// The configured indices, used while the space-weather inputs are
    /// unconnected.
    struct Configured {
        ap: f64,
        f107_daily: f64,
        f107_average: f64,
    }

    /// Upstream `_density`: the atmosphere at `R_SC = [6778137, 0, 0] m` at
    /// 2026-01-01 12:00 UTC, from `channels` (one value per space-weather
    /// input; `None` leaves that input unconnected) or the `configured`
    /// indices. `None` when the ANISE kernels are not fetched.
    fn atmosphere(channels: &[Option<f64>], configured: Configured) -> Option<AtmosphereMsg> {
        let pck = asset_or_skip("anise/pck11.pca")?;
        let mut module = MsisAtmosphere::new(MsisAtmosphereConfig {
            name: "MsisAtmo".to_string(),
            first_kernel_path: pck,
            additional_kernel_paths: Vec::new(),
            inertial_frame: EARTH_J2000,
            fixed_frame: IAU_EARTH_FRAME,
            ap_array: [configured.ap; 7],
            f107_daily: configured.f107_daily,
            f107_average: configured.f107_average,
        });
        let state = Output::new(SpacecraftStateMsg {
            position_m: Vector3::new(6_778_137.0, 0.0, 0.0),
            ..Default::default()
        });
        let planet = Output::new(PlanetStateMsg {
            orientation: Some(PlanetOrientation::identity()),
            ..Default::default()
        });
        state.connect_to(&mut module.input_state_msg);
        planet.connect_to(&mut module.input_planet_msg);
        let outputs: Vec<Output<SwDataMsg>> = channels
            .iter()
            .zip(&mut module.sw_data_in_msgs)
            .filter_map(|(value, input)| {
                value.map(|data_value| {
                    let output = Output::new(SwDataMsg { data_value });
                    output.connect_to(input);
                    output
                })
            })
            .collect();
        module.init();
        module.update(&SimulationContext {
            current_sim_nanos: 1_000_000_000,
            current_epoch: Epoch::from_gregorian_utc_hms(2026, 1, 1, 12, 0, 1),
        });
        drop(outputs);
        Some(module.output_atmosphere_msg.read())
    }

    /// Every channel connected: Ap `ap_values` (21 values, upstream's order)
    /// and F10.7 on channels 21 and 22.
    fn connected(ap_values: &[f64; 21], f107_average: f64, f107: f64) -> Vec<Option<f64>> {
        ap_values
            .iter()
            .copied()
            .chain([f107_average, f107])
            .map(Some)
            .collect()
    }

    /// Configured indices no test's messages agree with, so a module that
    /// read them instead of its inputs would fail.
    fn unrelated_configuration() -> Configured {
        Configured {
            ap: 300.0,
            f107_daily: 70.0,
            f107_average: 250.0,
        }
    }

    /// Upstream `_storm`: a daily Ap and a current 3-hour Ap over quiet
    /// history.
    fn storm(daily: f64, current: f64) -> [f64; 21] {
        let mut values = [AP_QUIET; 21];
        values[0] = daily;
        values[1] = current;
        values
    }

    #[test]
    fn connected_inputs_replace_the_configured_indices() {
        let Some(from_messages) = atmosphere(
            &connected(&[AP_QUIET; 21], 140.0, F107),
            unrelated_configuration(),
        ) else {
            return;
        };
        let configured = atmosphere(
            &[],
            Configured {
                ap: AP_QUIET,
                f107_daily: F107,
                f107_average: 140.0,
            },
        )
        .expect("kernels present");
        assert!(from_messages.neutral_density_kgpm3 > 0.0);
        assert_eq!(
            from_messages.neutral_density_kgpm3,
            configured.neutral_density_kgpm3
        );
        assert_eq!(from_messages.local_temp_k, configured.local_temp_k);
    }

    /// Upstream `test_ap_history_ignores_daily_ap` without the history: the
    /// daily Ap (channel 0) changes the density.
    #[test]
    fn the_daily_ap_channel_changes_the_density() {
        let Some(quiet) = atmosphere(
            &connected(&storm(AP_QUIET, 300.0), F107, F107),
            unrelated_configuration(),
        ) else {
            return;
        };
        let stormy = atmosphere(
            &connected(&storm(105.0, 300.0), F107, F107),
            unrelated_configuration(),
        )
        .expect("kernels present");
        let relative = (stormy.neutral_density_kgpm3 / quiet.neutral_density_kgpm3 - 1.0).abs();
        assert!(relative > 1.0e-6, "{relative}");
    }

    /// Upstream `test_ap_history_follows_current_three_hour_ap` without the
    /// history: the current 3-hour Ap (channel 1) leaves the density as it is.
    #[test]
    fn the_three_hour_history_is_unused_with_the_daily_ap() {
        let Some(quiet) = atmosphere(
            &connected(&[AP_QUIET; 21], F107, F107),
            unrelated_configuration(),
        ) else {
            return;
        };
        let storm_start = atmosphere(
            &connected(&storm(AP_QUIET, 300.0), F107, F107),
            unrelated_configuration(),
        )
        .expect("kernels present");
        assert_eq!(
            storm_start.neutral_density_kgpm3,
            quiet.neutral_density_kgpm3
        );
    }

    /// Channel 22 is the daily F10.7 and channel 21 the 81-day average
    /// (upstream `f107 = swDataList[22]`, `f107A = swDataList[21]`).
    #[test]
    fn the_f107_channels_map_as_upstream() {
        let Some(from_messages) = atmosphere(
            &connected(&[AP_QUIET; 21], 200.0, 90.0),
            unrelated_configuration(),
        ) else {
            return;
        };
        let configured = |f107_daily, f107_average| {
            atmosphere(
                &[],
                Configured {
                    ap: AP_QUIET,
                    f107_daily,
                    f107_average,
                },
            )
            .expect("kernels present")
            .neutral_density_kgpm3
        };
        assert_eq!(from_messages.neutral_density_kgpm3, configured(90.0, 200.0));
        assert_ne!(from_messages.neutral_density_kgpm3, configured(200.0, 90.0));
    }

    /// Upstream writes the zero atmosphere when a required space-weather
    /// message is missing.
    #[test]
    fn a_partly_connected_set_writes_a_zero_atmosphere() {
        let mut channels = connected(&[AP_QUIET; 21], F107, F107);
        channels[NUM_SW_MESSAGES - 1] = None;
        let Some(output) = atmosphere(&channels, unrelated_configuration()) else {
            return;
        };
        assert_eq!(output.neutral_density_kgpm3, 0.0);
        assert_eq!(output.local_temp_k, 0.0);
    }
}
