use std::path::{Path, PathBuf};

use anise::almanac::Almanac;
use anise::frames::Frame;
use nalgebra::Vector3;

use crate::messages::{Output, PlanetStateMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct AnisePlanetEphemerisConfig {
    pub name: String,
    pub spk_path: PathBuf,
    pub source_frame: Frame,
    pub observer_frame: Frame,
}

pub struct AnisePlanetEphemeris {
    pub config: AnisePlanetEphemerisConfig,
    pub output_planet_msg: Output<PlanetStateMsg>,
    almanac: Almanac,
}

impl Module for AnisePlanetEphemeris {
    fn init(&mut self) {
        self.output_planet_msg.write(PlanetStateMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let planet_state = self
            .almanac
            .translate_geometric(
                self.config.source_frame,
                self.config.observer_frame,
                context.current_epoch,
            )
            .expect("failed to compute ANISE planet ephemeris");

        self.output_planet_msg.write(PlanetStateMsg {
            position_inertial_m: Vector3::new(
                planet_state.radius_km.x * 1.0e3,
                planet_state.radius_km.y * 1.0e3,
                planet_state.radius_km.z * 1.0e3,
            ),
            velocity_inertial_mps: Vector3::new(
                planet_state.velocity_km_s.x * 1.0e3,
                planet_state.velocity_km_s.y * 1.0e3,
                planet_state.velocity_km_s.z * 1.0e3,
            ),
        });
    }
}

impl AnisePlanetEphemeris {
    pub fn new(config: AnisePlanetEphemerisConfig) -> Self {
        let resolved_spk_path = resolve_repo_relative_path(&config.spk_path);
        let almanac = Almanac::new(&resolved_spk_path.to_string_lossy())
            .expect("failed to load ANISE SPK asset");

        Self {
            config,
            output_planet_msg: Output::default(),
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
