//! Port of upstream `examples/scenarioDragDeorbit.py`: a 1 kg spacecraft
//! with 10 m² of cannonball drag, started on a 250 km orbit, propagated
//! until it falls below 100 km, in an MSIS or exponential atmosphere
//! without wind.
//!
//! | `--model` | upstream `model` | CSV |
//! |---|---|---|
//! | `msis` (default) | `"msis"` | `scenarioDragDeorbitmsis0.csv` |
//! | `exponential` | `"exponential"` | `scenarioDragDeorbitexponential0.csv` |
//!
//! `--all` runs both, the parametrizations of `test_scenarioDragDeorbit.py`
//! (`initialAlt = 250`, `deorbitAlt = 100`); `--initial-alt-km` and
//! `--deorbit-alt-km` change those. The trailing `0` is upstream's
//! `useWind = False`; the wind case (a `zeroWindModel` on a SPICE Earth) is
//! not tested upstream and not ported here.
//!
//! The simulation advances on a 15-second task cadence and stops on the first
//! task tick below the deorbit altitude (upstream's terminal `Deorbited`
//! event), at most after 100 orbits. Upstream runs the drag effector in its
//! own task after the spacecraft's, so the density it reads in `UpdateState`
//! is used in the next integration: [`AtmosphereLatch`] reproduces that
//! one-task delay. The state, drag force and density are sampled every 45 s
//! (upstream `samplingTime` for 10,000 points over 100 orbits).
//!
//! `test_scenarioDragDeorbit.py` only runs the scenario; `tests` checks
//! that both runs deorbit, with the density growing as the orbit decays.

#[path = "support/common.rs"]
mod common;
#[path = "support/scenario_args.rs"]
mod scenario_args;

use std::path::PathBuf;

use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
use basilisk_rs::dynamics::drag_dynamic_effector::{
    DragDynamicEffector, DragDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::environment::atmosphere::msis_atmosphere::{MsisAtmosphere, MsisAtmosphereConfig};
use basilisk_rs::messages::{
    AtmosphereMsg, Input, Output, PlanetOrientation, PlanetStateMsg, SpacecraftDiagnosticsMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvFormat, CsvRecorder, CsvRecorderConfig, CsvSourceConfig};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use common::{
    EARTH_EQUATORIAL_RADIUS_M, MU_EARTH_M3PS2, elem2rv, scenario_output_path, seconds,
    vector_columns,
};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};
use scenario_args::ScenarioArgs;

const USAGE: &str = "\
scenario_drag_deorbit [--model msis|exponential] [--initial-alt-km 250] [--deorbit-alt-km 100]
scenario_drag_deorbit --all     run every upstream test parametrization";

const TASK_PERIOD_NANOS: u64 = seconds(15);
const SAMPLE_PERIOD_NANOS: u64 = seconds(45);
const PROJECTED_AREA_M2: f64 = 10.0;
const DRAG_COEFFICIENT: f64 = 2.2;

/// Upstream `model`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AtmosphereModel {
    Msis,
    Exponential,
}

impl AtmosphereModel {
    pub fn label(self) -> &'static str {
        match self {
            Self::Msis => "msis",
            Self::Exponential => "exponential",
        }
    }
}

/// Upstream `run(show_plots, initialAlt, deorbitAlt, model, useWind=False)`.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub initial_altitude_km: f64,
    pub deorbit_altitude_km: f64,
    pub model: AtmosphereModel,
}

impl Config {
    /// The CSV stem, upstream's figure prefix `fileName + model` with
    /// `useWind = False`.
    pub fn file_stem(&self) -> String {
        format!("scenarioDragDeorbit{}0", self.model.label())
    }
}

/// The parametrizations of `test_scenarioDragDeorbit.py`.
pub const TEST_CASES: [Config; 2] = [
    Config {
        initial_altitude_km: 250.0,
        deorbit_altitude_km: 100.0,
        model: AtmosphereModel::Exponential,
    },
    Config {
        initial_altitude_km: 250.0,
        deorbit_altitude_km: 100.0,
        model: AtmosphereModel::Msis,
    },
];

/// Models the separate upstream drag task: it reads the atmosphere after the
/// spacecraft task and the cached density is consumed on the next integration
/// tick. This one-task delay is observable in the recorded trajectory.
#[derive(Clone, Debug, Default)]
struct AtmosphereLatch {
    atmosphere_in_msg: Input<AtmosphereMsg>,
    atmosphere_out_msg: Output<AtmosphereMsg>,
}

impl Module for AtmosphereLatch {
    fn init(&mut self) {
        self.atmosphere_out_msg.write(AtmosphereMsg::default());
    }

    fn update(&mut self, _context: &SimulationContext) {
        self.atmosphere_out_msg.write(self.atmosphere_in_msg.read());
    }
}

/// The atmosphere of `model`, reading `state`, and its output.
fn atmosphere(
    model: AtmosphereModel,
    simulation: &Simulation<'_>,
    state: &Output<SpacecraftStateMsg>,
) -> (Box<dyn Module>, Output<AtmosphereMsg>) {
    match model {
        AtmosphereModel::Msis => {
            let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
            let mut atmosphere = MsisAtmosphere::new(MsisAtmosphereConfig {
                name: "MsisAtmo".to_string(),
                first_kernel_path: repo_root.join("assets/anise/pck11.pca"),
                additional_kernel_paths: vec![
                    repo_root.join("assets/anise/earth_latest_high_prec.bpc"),
                ],
                inertial_frame: EARTH_J2000,
                fixed_frame: IAU_EARTH_FRAME,
                ap_array: [8.0; 7],
                f107_daily: 110.0,
                f107_average: 110.0,
            });
            // The source module defaults to an identity planet DCM when no
            // ephemeris message is supplied. Supplying that state explicitly
            // avoids ANISE's rotating-Earth fallback while retaining the
            // shared MSIS module API.
            let planet_state = Output::new(PlanetStateMsg {
                position_inertial_m: Vector3::zeros(),
                velocity_inertial_mps: Vector3::zeros(),
                orientation: Some(PlanetOrientation::identity()),
            });
            connect!(simulation,
                state => &mut atmosphere.input_state_msg,
                &planet_state => &mut atmosphere.input_planet_msg,
            );
            let output = atmosphere.output_atmosphere_msg.clone();
            (Box::new(atmosphere), output)
        }
        AtmosphereModel::Exponential => {
            // Upstream `simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")`.
            let mut atmosphere = ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
                name: "ExpAtmo".to_string(),
                planet_radius_m: 6_378_136.6,
                reference_altitude_m: 0.0,
                reference_density_kgpm3: 1.217,
                scale_height_m: 8_500.0,
            });
            connect!(simulation, state => &mut atmosphere.input_state_msg);
            let output = atmosphere.output_atmosphere_msg.clone();
            (Box::new(atmosphere), output)
        }
    }
}

pub struct Results {
    pub csv_path: PathBuf,
    /// The time of the terminal tick (ns).
    pub terminal_nanos: u64,
    /// The altitude at the terminal tick (m).
    pub final_altitude_m: f64,
    /// The density at the first and the terminal tick (kg/m^3).
    pub initial_density_kgpm3: f64,
    pub final_density_kgpm3: f64,
}

pub fn run(config: Config) -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let semimajor_axis_m = EARTH_EQUATORIAL_RADIUS_M + config.initial_altitude_km * 1_000.0;
    let (initial_position_m, initial_velocity_mps) = elem2rv(
        MU_EARTH_M3PS2,
        semimajor_axis_m,
        0.0001,
        33.3 * degrees,
        48.2 * degrees,
        347.8 * degrees,
        85.3 * degrees,
    );

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        // The upstream example intentionally leaves the hub at its 1 kg default.
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: TASK_PERIOD_NANOS,
        initial_position_m,
        initial_velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid point-mass Earth"),
        )
        .expect("unique central gravity body");

    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "DragEff".to_string(),
        projected_area_m2: PROJECTED_AREA_M2,
        drag_coeff: DRAG_COEFFICIENT,
        com_offset_m: Vector3::zeros(),
        // `useWind=False`: upstream drag uses inertial spacecraft velocity.
        planet_rotation_rate_radps: Vector3::zeros(),
    });
    let mut atmosphere_latch = AtmosphereLatch::default();
    let start_epoch = Epoch::from_gregorian_utc_at_midnight(2019, 1, 1);
    let mut simulation = Simulation::new(start_epoch, false);
    connect!(&simulation,
        &atmosphere_latch.atmosphere_out_msg => &mut drag.input_atmosphere_msg,
    );
    spacecraft.add_dynamic_effector(drag);

    let state_handle = spacecraft.state_out.clone();
    let (mut atmosphere, atmosphere_out) =
        atmosphere(config.model, &simulation, &spacecraft.state_out);
    let file_stem = config.file_stem();
    let output_path = scenario_output_path(&format!("{file_stem}.csv"));
    let mut recorder = CsvRecorder::new(CsvRecorderConfig {
        topic: file_stem,
        output_path: output_path.clone(),
    })
    .with_format(CsvFormat::Scenario);

    connect!(&simulation,
        &atmosphere_out => &mut atmosphere_latch.atmosphere_in_msg,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(
            CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("velocity_mps", "v_BN_N_m_per_s")),
            ),
        ),
        &spacecraft.diagnostics_out => recorder.add_source::<SpacecraftDiagnosticsMsg>(
            CsvSourceConfig::columns(vector_columns(
                "drag_force_body_n",
                "forceExternal_B_N",
            )),
        ),
        &atmosphere_out => recorder.add_source::<AtmosphereMsg>(
            CsvSourceConfig::columns([(
                "neutral_density_kgpm3",
                "neutralDensity_kg_per_m3",
            )]),
        ),
    );
    schedule! { simulation,
        "atmosphere" => atmosphere.as_mut(), period=TASK_PERIOD_NANOS, priority=30;
        "spacecraft" => &mut spacecraft, period=TASK_PERIOD_NANOS, priority=20;
        "atmosphere_latch" => &mut atmosphere_latch, period=TASK_PERIOD_NANOS, priority=5;
        "recorder" => &mut recorder, period=SAMPLE_PERIOD_NANOS, priority=0;
    }

    let orbit_period_s =
        2.0 * std::f64::consts::PI * (semimajor_axis_m.powi(3) / MU_EARTH_M3PS2).sqrt();
    let maximum_duration_nanos = (100.0 * orbit_period_s * 1.0e9) as u64;
    let terminal_altitude_m = config.deorbit_altitude_km * 1_000.0;
    let terminal_radius_m = EARTH_EQUATORIAL_RADIUS_M + terminal_altitude_m;

    simulation.run_for(0);
    let initial_density_kgpm3 = atmosphere_out.read().neutral_density_kgpm3;
    while simulation.current_sim_nanos() < maximum_duration_nanos
        && state_handle.read().position_m.norm() >= terminal_radius_m
    {
        simulation.run_for(TASK_PERIOD_NANOS);
    }
    let terminal_nanos = simulation.current_sim_nanos();
    drop(simulation);
    recorder.flush().expect("flush scenario CSV");

    Results {
        csv_path: output_path,
        terminal_nanos,
        final_altitude_m: state_handle.read().position_m.norm() - EARTH_EQUATORIAL_RADIUS_M,
        initial_density_kgpm3,
        final_density_kgpm3: atmosphere_out.read().neutral_density_kgpm3,
    }
}

fn main() {
    let args = ScenarioArgs::from_env(USAGE);
    let configs = if args.flag("--all") {
        TEST_CASES.to_vec()
    } else {
        let model = match args.value("--model", "msis".to_string()).as_str() {
            "msis" => AtmosphereModel::Msis,
            "exponential" => AtmosphereModel::Exponential,
            other => args.fail(&format!("unknown atmosphere model {other:?}")),
        };
        vec![Config {
            initial_altitude_km: args.value("--initial-alt-km", 250.0),
            deorbit_altitude_km: args.value("--deorbit-alt-km", 100.0),
            model,
        }]
    };
    for config in configs {
        let results = run(config);
        assert!(
            results.final_altitude_m < config.deorbit_altitude_km * 1_000.0,
            "spacecraft did not deorbit within the 100-orbit cap"
        );
        println!(
            "wrote {} (terminal tick {:.0} s, altitude {:.3} km)",
            results.csv_path.display(),
            results.terminal_nanos as f64 * 1.0e-9,
            results.final_altitude_m / 1_000.0,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{TEST_CASES, run};

    /// `test_scenarioDragDeorbit.py` runs both models without checks; each
    /// run must end below the deorbit altitude, before the 100-orbit cap,
    /// with the density far above its 250 km value.
    #[test]
    fn both_atmospheres_deorbit_the_spacecraft() {
        for config in TEST_CASES {
            let results = run(config);
            assert!(
                results.final_altitude_m < config.deorbit_altitude_km * 1_000.0,
                "{}: final altitude {} m",
                config.file_stem(),
                results.final_altitude_m
            );
            assert!(results.final_altitude_m > 0.9 * config.deorbit_altitude_km * 1_000.0);
            assert!(results.initial_density_kgpm3 > 0.0);
            assert!(
                results.final_density_kgpm3 > 1.0e3 * results.initial_density_kgpm3,
                "{}: density {} -> {}",
                config.file_stem(),
                results.initial_density_kgpm3,
                results.final_density_kgpm3
            );
        }
    }
}
