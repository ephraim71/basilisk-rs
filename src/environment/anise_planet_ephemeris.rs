use std::path::{Path, PathBuf};
use std::time::Instant;

use anise::almanac::Almanac;
use anise::frames::Frame;
use nalgebra::{Matrix3, Vector3};

use crate::messages::{Output, PlanetStateMsg};
use crate::{Module, SimulationContext};

#[derive(Clone, Debug)]
pub struct AnisePlanetEphemerisConfig {
    pub name: String,
    pub spk_path: PathBuf,
    pub additional_kernel_paths: Vec<PathBuf>,
    pub source_frame: Frame,
    pub observer_frame: Frame,
    pub fixed_frame: Option<Frame>,
}

pub struct AnisePlanetEphemeris {
    pub config: AnisePlanetEphemerisConfig,
    pub output_planet_msg: Output<PlanetStateMsg>,
    almanac: Almanac,
    timing_enabled: bool,
    timing_stats: AnisePlanetEphemerisTimingStats,
}

#[derive(Clone, Debug, Default)]
pub struct AnisePlanetEphemerisTimingStats {
    pub update_calls: u64,
    pub total_update_nanos: u128,
    pub translate_nanos: u128,
    pub write_output_nanos: u128,
}

impl Module for AnisePlanetEphemeris {
    fn init(&mut self) {
        self.output_planet_msg.write(PlanetStateMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let total_started = self.timing_enabled.then(Instant::now);
        let phase_started = self.timing_enabled.then(Instant::now);
        let planet_state = self
            .almanac
            .translate_geometric(
                self.config.source_frame,
                self.config.observer_frame,
                context.current_epoch,
            )
            .expect("failed to compute ANISE planet ephemeris");
        if let Some(started) = phase_started {
            self.timing_stats.translate_nanos += started.elapsed().as_nanos();
        }

        let (has_orientation, inertial_to_fixed, inertial_to_fixed_dot) =
            if let Some(fixed_frame) = self.config.fixed_frame {
                let rotation = self
                    .almanac
                    .rotate(self.config.source_frame, fixed_frame, context.current_epoch)
                    .expect("failed to compute ANISE planet orientation");
                (
                    true,
                    matrix3_from_anise_rotation(&rotation.rot_mat),
                    rotation
                        .rot_mat_dt
                        .as_ref()
                        .map(matrix3_from_anise_rotation)
                        .unwrap_or_else(Matrix3::zeros),
                )
            } else {
                (false, Matrix3::zeros(), Matrix3::zeros())
            };

        let phase_started = self.timing_enabled.then(Instant::now);
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
            has_orientation,
            inertial_to_fixed,
            inertial_to_fixed_dot,
        });
        if let Some(started) = phase_started {
            self.timing_stats.write_output_nanos += started.elapsed().as_nanos();
        }

        if let Some(started) = total_started {
            self.timing_stats.update_calls += 1;
            self.timing_stats.total_update_nanos += started.elapsed().as_nanos();
        }
    }
}

impl AnisePlanetEphemeris {
    pub fn new(config: AnisePlanetEphemerisConfig) -> Self {
        let resolved_spk_path = resolve_repo_relative_path(&config.spk_path);
        let mut almanac = Almanac::new(&resolved_spk_path.to_string_lossy())
            .expect("failed to load ANISE SPK asset");
        for kernel_path in &config.additional_kernel_paths {
            let resolved_path = resolve_repo_relative_path(kernel_path);
            almanac = almanac
                .load(&resolved_path.to_string_lossy())
                .expect("failed to load additional ANISE kernel");
        }

        Self {
            config,
            output_planet_msg: Output::default(),
            almanac,
            timing_enabled: false,
            timing_stats: AnisePlanetEphemerisTimingStats::default(),
        }
    }

    pub fn set_timing_enabled(&mut self, enabled: bool) {
        self.timing_enabled = enabled;
    }

    pub fn timing_stats(&self) -> &AnisePlanetEphemerisTimingStats {
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
