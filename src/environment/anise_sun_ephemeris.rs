use std::path::{Path, PathBuf};
use std::time::Instant;

use anise::almanac::Almanac;
use anise::constants::frames::{EARTH_J2000, SUN_J2000};
use nalgebra::Vector3;

use crate::messages::{Output, PlanetStateMsg, SunEphemerisMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct AniseSunEphemerisConfig {
    pub name: String,
    pub spk_path: PathBuf,
}

pub struct AniseSunEphemeris {
    pub config: AniseSunEphemerisConfig,
    pub output_planet_msg: Output<PlanetStateMsg>,
    pub output_sun_msg: Output<SunEphemerisMsg>,
    almanac: Almanac,
    timing_enabled: bool,
    timing_stats: AniseSunEphemerisTimingStats,
}

#[derive(Clone, Debug, Default)]
pub struct AniseSunEphemerisTimingStats {
    pub update_calls: u64,
    pub total_update_nanos: u128,
    pub translate_nanos: u128,
    pub write_sun_msg_nanos: u128,
    pub write_planet_msg_nanos: u128,
}

impl Module for AniseSunEphemeris {
    fn init(&mut self) {
        self.output_planet_msg.write(PlanetStateMsg::default());
        self.output_sun_msg.write(SunEphemerisMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let total_started = self.timing_enabled.then(Instant::now);
        let phase_started = self.timing_enabled.then(Instant::now);
        let sun_state = self
            .almanac
            .translate_geometric(SUN_J2000, EARTH_J2000, context.current_epoch)
            .expect("failed to compute Sun ephemeris with ANISE");
        if let Some(started) = phase_started {
            self.timing_stats.translate_nanos += started.elapsed().as_nanos();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        self.output_sun_msg.write(SunEphemerisMsg {
            sun_position_inertial_m: Vector3::new(
                sun_state.radius_km.x * 1.0e3,
                sun_state.radius_km.y * 1.0e3,
                sun_state.radius_km.z * 1.0e3,
            ),
            sun_velocity_inertial_mps: Vector3::new(
                sun_state.velocity_km_s.x * 1.0e3,
                sun_state.velocity_km_s.y * 1.0e3,
                sun_state.velocity_km_s.z * 1.0e3,
            ),
        });
        if let Some(started) = phase_started {
            self.timing_stats.write_sun_msg_nanos += started.elapsed().as_nanos();
        }

        let phase_started = self.timing_enabled.then(Instant::now);
        self.output_planet_msg.write(PlanetStateMsg {
            position_inertial_m: Vector3::new(
                sun_state.radius_km.x * 1.0e3,
                sun_state.radius_km.y * 1.0e3,
                sun_state.radius_km.z * 1.0e3,
            ),
            velocity_inertial_mps: Vector3::new(
                sun_state.velocity_km_s.x * 1.0e3,
                sun_state.velocity_km_s.y * 1.0e3,
                sun_state.velocity_km_s.z * 1.0e3,
            ),
            has_orientation: false,
            inertial_to_fixed: nalgebra::Matrix3::zeros(),
            inertial_to_fixed_dot: nalgebra::Matrix3::zeros(),
        });
        if let Some(started) = phase_started {
            self.timing_stats.write_planet_msg_nanos += started.elapsed().as_nanos();
        }

        if let Some(started) = total_started {
            self.timing_stats.update_calls += 1;
            self.timing_stats.total_update_nanos += started.elapsed().as_nanos();
        }
    }
}

impl AniseSunEphemeris {
    pub fn new(config: AniseSunEphemerisConfig) -> Self {
        let resolved_spk_path = resolve_repo_relative_path(&config.spk_path);
        let almanac = Almanac::new(&resolved_spk_path.to_string_lossy())
            .expect("failed to load ANISE SPK asset");

        Self {
            config,
            output_planet_msg: Output::default(),
            output_sun_msg: Output::default(),
            almanac,
            timing_enabled: false,
            timing_stats: AniseSunEphemerisTimingStats::default(),
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    pub fn timing_stats(&self) -> &AniseSunEphemerisTimingStats {
        &self.timing_stats
    }
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}
