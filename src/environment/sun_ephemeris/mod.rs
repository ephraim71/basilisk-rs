use std::path::{Path, PathBuf};

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
}

impl Module for AniseSunEphemeris {
    fn init(&mut self) {
        self.output_planet_msg.write(PlanetStateMsg::default());
        self.output_sun_msg.write(SunEphemerisMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let sun_state = self
            .almanac
            .translate_geometric(SUN_J2000, EARTH_J2000, context.current_epoch)
            .expect("failed to compute Sun ephemeris with ANISE");

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
        });
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
        }
    }
}

fn resolve_repo_relative_path(path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        Path::new(env!("CARGO_MANIFEST_DIR")).join(path)
    }
}
