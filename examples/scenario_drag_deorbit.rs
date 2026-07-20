//! Scenario: Drag Deorbit
//!
//! Port of Basilisk's `scenarioDragDeorbit.py`.
//!
//! A spacecraft orbits Earth subject to atmospheric drag and slowly deorbits.
//! The simulation runs until the geocentric radius falls below a terminal
//! deorbit altitude (Basilisk's terminal `createNewEvent`), or until a hard cap
//! of 100 orbital periods is reached.
//!
//! ## Basilisk -> basilisk-rs mapping
//!
//! | Basilisk (Python)                                            | basilisk-rs (Rust)                                                        |
//! |--------------------------------------------------------------|---------------------------------------------------------------------------|
//! | `SimBaseClass()` + process/task                              | [`Simulation`] + [`Simulation::add_module`]                               |
//! | `spacecraft.Spacecraft()`                                    | [`Spacecraft::new`]                                                        |
//! | `ExponentialAtmosphere()` + `simSetPlanetEnvironment`        | [`ExponentialAtmosphere::new`]                                            |
//! | `MsisAtmosphere()`                                           | [`MsisAtmosphere::new`]                                                   |
//! | `DragDynamicEffector()`, `coreParams.projectedArea/dragCoeff`| [`DragDynamicEffector::new`] with [`DragDynamicEffectorConfig`]            |
//! | `gravFactory.createEarth()` + `addBodiesTo`                  | [`Spacecraft::add_grav_body`] with a point-mass Earth                     |
//! | `atmo.addSpacecraftToModel(scObject.scStateOutMsg)`          | `sim.connect(&spacecraft.state_out, atmo.state_input())`                  |
//! | `scObject.addDynamicEffector(dragEffector)`                  | `spacecraft.add_dynamic_effector(drag)`                                   |
//! | `dragEffector.atmoDensInMsg.subscribeTo(atmo.envOutMsgs[0])` | `sim.connect(&atmo_density, &mut drag.input_atmosphere_msg)`              |
//! | `zeroWindModel` (`useWind=True`)                             | [`DragDynamicEffectorConfig::planet_rotation_rate_radps`] (co-rotating)   |
//! | `orbitalMotion.elem2rv`                                      | local [`elem2rv`] helper                                                  |
//! | terminal `createNewEvent(... < radEquator + deorbitAlt)`     | chunked `run_for` loop with an explicit radius check                      |
//! | `scStateOutMsg.recorder(...)`, `dragEffector.logger(...)`    | optional [`CsvRecorder`] on the state / atmosphere / diagnostics messages |
//!
//! ## Run
//!
//! ```text
//! cargo run --release --example scenario_drag_deorbit                     # exponential atmosphere
//! cargo run --release --example scenario_drag_deorbit -- msis             # NRLMSISE-00 (needs `just fetch-assets`)
//! cargo run --release --example scenario_drag_deorbit -- exponential wind # co-rotating atmosphere
//! cargo run --release --example scenario_drag_deorbit -- init=300 deorbit=120
//! ```
//!
//! Environment variables (mirroring `full_satellite_stack`):
//!
//! ```text
//! ENABLE_RECORDING=1  write CSV telemetry to examples/output/scenario_drag_deorbit/
//! SHOW_PROGRESS=1     render a live completion bar (elapsed / % / altitude)
//! PROFILE_SIM=1       collect and print per-module timings
//! ```

use std::path::{Path, PathBuf};

use anise::constants::frames::{EARTH_J2000, IAU_EARTH_FRAME};
use basilisk_rs::Module;
use basilisk_rs::dynamics::drag_dynamic_effector::{
    DragDynamicEffector, DragDynamicEffectorConfig,
};
use basilisk_rs::dynamics::gravity::GravBodyData;
use basilisk_rs::environment::atmosphere::exponential_atmosphere::{
    ExponentialAtmosphere, ExponentialAtmosphereConfig,
};
use basilisk_rs::environment::atmosphere::msis_atmosphere::{MsisAtmosphere, MsisAtmosphereConfig};
use basilisk_rs::messages::{AtmosphereMsg, Input, Output, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::{CsvRecorder, CsvRecorderConfig};
use hifitime::Epoch;
use indicatif::{ProgressBar, ProgressStyle};
use nalgebra::{Matrix3, Vector3};

// --- Earth constants (match Basilisk `gravFactory.createEarth()` and
// --- `simSetPlanetEnvironment.exponentialAtmosphere(atmo, "earth")`) ---------
const MU_EARTH_M3PS2: f64 = 3.986_004_415e14; // createEarth mu
const R_EQUATOR_M: f64 = 6_378_136.6; // REQ_EARTH (also the exp-atmosphere planet radius)
const EARTH_ROTATION_RATE_RADPS: f64 = 7.292_115_9e-5; // sidereal rate about +z

// Exponential atmosphere "earth" preset used by simSetPlanetEnvironment.
const EXP_BASE_DENSITY_KGPM3: f64 = 1.217;
const EXP_SCALE_HEIGHT_M: f64 = 8500.0;

// Basilisk's spacecraft hub leaves `mHub` at its default of 1.0 kg in this
// scenario (the script never sets a mass). Combined with the 10 m^2 projected
// area, this is what lets the vehicle decay within the 100-orbit cap. Change it
// to model a real ballistic coefficient.
const SPACECRAFT_MASS_KG: f64 = 1.0;

// Drag effector parameters (Basilisk coreParams).
const PROJECTED_AREA_M2: f64 = 10.0;
const DRAG_COEFF: f64 = 2.2;

// A small fixed integration step keeps the explicit RK4 stable as the vehicle
// descends into denser atmosphere, where drag stiffens sharply. (At 15 s the
// per-step drag Δv near 100 km already approached the orbital velocity for this
// 1 kg / 10 m² vehicle, which is what previously diverged.)
const STEP_NANOS: u64 = 1_000_000_000; // 1 s integration step
// Record telemetry every SAMPLE_NANOS of sim time so the CSVs stay a sensible
// size independent of the (smaller) integration step. Must be a multiple of STEP_NANOS.
const SAMPLE_NANOS: u64 = 1_000_000_000; // 1 s
const MAX_ORBITS: f64 = 100.0; // simulationTime = 100 * P

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Model {
    Exponential,
    Msis,
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let model = if args.iter().any(|a| a.eq_ignore_ascii_case("msis")) {
        Model::Msis
    } else {
        Model::Exponential
    };
    let use_wind = args.iter().any(|a| a.eq_ignore_ascii_case("wind"));
    let initial_alt_km = parse_kv(&args, "init").unwrap_or(250.0);
    let deorbit_alt_km = parse_kv(&args, "deorbit").unwrap_or(100.0);

    // Runtime switches, mirroring `full_satellite_stack`.
    let enable_recording = std::env::var_os("ENABLE_RECORDING").is_some();
    let show_progress = std::env::var_os("SHOW_PROGRESS").is_some();
    let profile_sim = std::env::var_os("PROFILE_SIM").is_some();

    // Scheduler priorities (higher runs first each tick): the environment and
    // dynamics update before the recorders sample them.
    const PRIORITY_ENVIRONMENT: i32 = 30;
    const PRIORITY_RECORDERS: i32 = 0;

    // With ENABLE_RECORDING set, this example records its telemetry to CSV (one
    // file per message) via the CsvRecorder modules wired into the sim below.
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let output_dir = repo_root.join("examples/output/scenario_drag_deorbit");
    if enable_recording && output_dir.exists() {
        std::fs::remove_dir_all(&output_dir).expect("failed to clear previous output");
    }

    // --- Set up a near-circular orbit from classical elements (matches Python) --
    let deg = std::f64::consts::PI / 180.0;
    let a = R_EQUATOR_M + initial_alt_km * 1000.0;
    let e = 0.0001;
    let inc = 33.3 * deg;
    let raan = 48.2 * deg;
    let aop = 347.8 * deg;
    let ta0 = 85.3 * deg;
    let (r0, v0) = elem2rv(MU_EARTH_M3PS2, a, e, inc, raan, aop, ta0);

    let n = (MU_EARTH_M3PS2 / a.powi(3)).sqrt();
    let period_s = 2.0 * std::f64::consts::PI / n;
    let max_sim_nanos = (MAX_ORBITS * period_s * 1.0e9) as u64;
    let deorbit_radius_m = R_EQUATOR_M + deorbit_alt_km * 1000.0;

    println!("=== scenario_drag_deorbit ===");
    println!(
        "Model:   {}   wind: {}",
        match model {
            Model::Exponential => "exponential",
            Model::Msis => "msis (NRLMSISE-00)",
        },
        if use_wind { "on (co-rotating)" } else { "off" }
    );
    println!(
        "Orbit:   a = {:.1} km  e = {:.4}  i = {:.1}°   ({:.0} km circular)",
        a / 1e3,
        e,
        inc / deg,
        initial_alt_km
    );
    println!("Period:  {period_s:.1} s   deorbit at {deorbit_alt_km:.0} km altitude");
    println!("Drag:    A = {PROJECTED_AREA_M2} m²  Cd = {DRAG_COEFF}  m = {SPACECRAFT_MASS_KG} kg");
    println!();

    // --- Spacecraft (Earth point-mass gravity, cannonball drag) ----------------
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: SPACECRAFT_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: r0,
        initial_velocity_mps: v0,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft.set_timing_enabled(profile_sim);
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("failed to configure Earth gravity"),
        )
        .expect("failed to add Earth gravity body");

    // --- Atmosphere model (exponential or MSIS) --------------------------------
    let mut atmosphere: Box<dyn AtmosphereModule> = match model {
        Model::Exponential => Box::new(ExponentialAtmosphere::new(ExponentialAtmosphereConfig {
            name: "ExpAtmo".to_string(),
            planet_radius_m: R_EQUATOR_M,
            reference_altitude_m: 0.0,
            reference_density_kgpm3: EXP_BASE_DENSITY_KGPM3,
            scale_height_m: EXP_SCALE_HEIGHT_M,
        })),
        Model::Msis => Box::new(MsisAtmosphere::new(MsisAtmosphereConfig {
            name: "MsisAtmo".to_string(),
            // MSIS needs Earth-orientation kernels (fetched by `just fetch-assets`).
            first_kernel_path: repo_root.join("assets/anise/pck11.pca"),
            additional_kernel_paths: vec![
                repo_root.join("assets/anise/earth_latest_high_prec.bpc"),
            ],
            inertial_frame: EARTH_J2000,
            fixed_frame: IAU_EARTH_FRAME,
            // Space-weather inputs (Python sw_msg: ap = 8, f107 = 110). The
            // 7-element ap_array carries the daily Ap plus the 3-hourly history;
            // a constant value matches the Python scenario's single `ap` setting.
            ap_array: [8.0; 7],
            f107_daily: 110.0,
            f107_average: 110.0,
        })),
    };

    // --- Drag effector ---------------------------------------------------------
    // `useWind=True` in Basilisk links a ZeroWindModel so drag acts on the
    // atmosphere-relative velocity. Here the same physics is expressed directly
    // by the planet rotation rate: v_rel = v - omega x r. `useWind=False` -> 0.
    let planet_rotation_rate_radps = if use_wind {
        Vector3::new(0.0, 0.0, EARTH_ROTATION_RATE_RADPS)
    } else {
        Vector3::zeros()
    };
    let mut drag = DragDynamicEffector::new(DragDynamicEffectorConfig {
        name: "DragEff".to_string(),
        projected_area_m2: PROJECTED_AREA_M2,
        drag_coeff: DRAG_COEFF,
        com_offset_m: Vector3::zeros(),
        planet_rotation_rate_radps,
    });

    // --- Wiring ----------------------------------------------------------------
    // Independent Arc-backed handles so we can read messages while the modules
    // are mutably borrowed by the simulation (used for the terminal condition
    // and status/CSV reporting).
    let state_handle = spacecraft.state_out.clone();
    let diagnostics_handle = spacecraft.diagnostics_out.clone();
    let density_handle: Output<AtmosphereMsg> = atmosphere.density_output().clone();

    // CSV recorders — one per message, matching the `full_satellite_stack` pattern:
    //   spacecraft_state.csv  position/velocity/attitude  (orbit + altitude)
    //   atmosphere.csv        neutral density             (density vs. altitude)
    //   drag.csv              diagnostics incl. drag force (|F_drag| vs. time)
    let mut state_recorder = csv_recorder::<SpacecraftStateMsg>("spacecraft_state", &output_dir);
    let mut atmo_recorder = csv_recorder::<AtmosphereMsg>("atmosphere", &output_dir);
    let mut drag_recorder =
        csv_recorder::<basilisk_rs::messages::SpacecraftDiagnosticsMsg>("drag", &output_dir);

    let result = {
        // The sim's built-in progress bar is created and finished inside each
        // `run_for`, so it can't span the chunked step loop below — with
        // `show_progress` it would restart the bar every step. This scenario
        // reports progress through its own status table instead (gated on
        // `show_progress`), so the sim bar stays off.
        let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2020, 5, 21), false);
        sim.set_timing_enabled(profile_sim);

        // atmo <- spacecraft state ; drag <- atmo density
        sim.connect(&spacecraft.state_out, atmosphere.state_input());
        sim.connect(&density_handle, &mut drag.input_atmosphere_msg);
        spacecraft.add_dynamic_effector(drag);

        // Recorder inputs subscribe to the messages they log (only when recording).
        if enable_recording {
            sim.connect(&spacecraft.state_out, &mut state_recorder.input_msg);
            sim.connect(&density_handle, &mut atmo_recorder.input_msg);
            sim.connect(&spacecraft.diagnostics_out, &mut drag_recorder.input_msg);
        }

        // Environment first (atmosphere before spacecraft so drag reads a fresh
        // density each step); recorders run last so they sample the updated state.
        sim.add_module(
            "atmosphere",
            atmosphere.as_module(),
            STEP_NANOS,
            PRIORITY_ENVIRONMENT,
        );
        sim.add_module(
            "spacecraft",
            &mut spacecraft,
            STEP_NANOS,
            PRIORITY_ENVIRONMENT,
        );

        // Recorders sample every SAMPLE_NANOS, like the recorders in
        // `full_satellite_stack` which run at the sim step rate.
        if enable_recording {
            sim.add_module(
                "state_recorder",
                &mut state_recorder,
                SAMPLE_NANOS,
                PRIORITY_RECORDERS,
            );
            sim.add_module(
                "atmo_recorder",
                &mut atmo_recorder,
                SAMPLE_NANOS,
                PRIORITY_RECORDERS,
            );
            sim.add_module(
                "drag_recorder",
                &mut drag_recorder,
                SAMPLE_NANOS,
                PRIORITY_RECORDERS,
            );
        }

        let telemetry = DeorbitTelemetry {
            state: &state_handle,
            density: &density_handle,
            diagnostics: &diagnostics_handle,
        };
        let result = run_to_deorbit(
            &mut sim,
            telemetry,
            max_sim_nanos,
            deorbit_radius_m,
            show_progress,
        );

        if profile_sim {
            println!();
            println!("module_timings_ms =");
            for timing in sim.module_timings() {
                println!(
                    "  {:>16}  priority={:>2}  calls={:>7}  total_ms={:>10.3}",
                    timing.name,
                    timing.priority,
                    timing.num_updates,
                    timing.total_update_nanos as f64 * 1.0e-6,
                );
            }
        }

        result
    };

    // --- Summary ---------------------------------------------------------------
    let final_radius_m = state_handle.read().position_m.norm();
    let final_alt_km = (final_radius_m - R_EQUATOR_M) / 1000.0;
    println!();
    if result.deorbited {
        println!(
            "DEORBITED after {:.3} h ({:.2} orbits) at altitude {:.1} km",
            result.final_time_s / 3600.0,
            result.final_time_s / period_s,
            final_alt_km
        );
    } else {
        println!(
            "Did NOT reach deorbit altitude within {MAX_ORBITS:.0} orbits \
             ({:.2} h); final altitude {:.1} km",
            result.final_time_s / 3600.0,
            final_alt_km
        );
    }
    println!();
    println!("recording_enabled = {enable_recording}");
    println!("show_progress = {show_progress}");
    println!("profile_sim = {profile_sim}");
    if enable_recording {
        println!("wrote CSV telemetry to {}", output_dir.display());
    }
}

/// A common interface over the two atmosphere modules so the wiring above stays
/// model-agnostic. Both expose the same state input and density output.
trait AtmosphereModule: Module {
    fn state_input(&mut self) -> &mut Input<SpacecraftStateMsg>;
    fn density_output(&self) -> &Output<AtmosphereMsg>;
    fn as_module(&mut self) -> &mut dyn Module;
}

impl AtmosphereModule for ExponentialAtmosphere {
    fn state_input(&mut self) -> &mut Input<SpacecraftStateMsg> {
        &mut self.input_state_msg
    }
    fn density_output(&self) -> &Output<AtmosphereMsg> {
        &self.output_atmosphere_msg
    }
    fn as_module(&mut self) -> &mut dyn Module {
        self
    }
}

impl AtmosphereModule for MsisAtmosphere {
    fn state_input(&mut self) -> &mut Input<SpacecraftStateMsg> {
        &mut self.input_state_msg
    }
    fn density_output(&self) -> &Output<AtmosphereMsg> {
        &self.output_atmosphere_msg
    }
    fn as_module(&mut self) -> &mut dyn Module {
        self
    }
}

struct DeorbitResult {
    deorbited: bool,
    final_time_s: f64,
}

/// Message handles the deorbit loop reads while the modules are borrowed by the
/// sim (used for the terminal condition and the live progress bar).
#[derive(Clone, Copy)]
struct DeorbitTelemetry<'a> {
    state: &'a Output<SpacecraftStateMsg>,
    density: &'a Output<AtmosphereMsg>,
    diagnostics: &'a Output<basilisk_rs::messages::SpacecraftDiagnosticsMsg>,
}

/// Advance the simulation one step at a time until the geocentric radius drops
/// below `deorbit_radius_m` (Basilisk's terminal event) or `max_sim_nanos` is
/// reached. When `show_progress` (SHOW_PROGRESS), renders a live completion bar
/// (elapsed / percent of the orbit cap / current altitude, density, drag);
/// otherwise it runs quietly.
fn run_to_deorbit(
    sim: &mut Simulation,
    telemetry: DeorbitTelemetry,
    max_sim_nanos: u64,
    deorbit_radius_m: f64,
    show_progress: bool,
) -> DeorbitResult {
    sim.initialize();

    // A single progress bar spanning the whole run. The sim's built-in bar is
    // created and finished inside one `run_for`, so it can't span the chunked
    // step loop this scenario needs for the terminal deorbit check — we drive
    // our own here, reusing the same style as `Simulation::run_for`.
    let progress = show_progress.then(|| {
        let bar = ProgressBar::new(max_sim_nanos);
        bar.set_style(
            ProgressStyle::with_template(
                "[{elapsed_precise}] {bar:40.cyan/blue} {percent:>3}% {msg}",
            )
            .expect("valid progress template")
            .progress_chars("##-"),
        );
        bar
    });

    let mut deorbited = false;
    while sim.current_sim_nanos() < max_sim_nanos {
        let remaining = max_sim_nanos - sim.current_sim_nanos();
        sim.run_for(STEP_NANOS.min(remaining));

        let radius_m = telemetry.state.read().position_m.norm();
        if let Some(bar) = &progress {
            bar.set_position(sim.current_sim_nanos());
            bar.set_message(format!(
                "alt {:.1} km  ρ {:.2e}  |F| {:.2e} N",
                (radius_m - R_EQUATOR_M) / 1000.0,
                telemetry.density.read().neutral_density_kgpm3,
                telemetry.diagnostics.read().drag_force_body_n.norm(),
            ));
        }

        if radius_m < deorbit_radius_m {
            deorbited = true;
            break;
        }
    }

    if let Some(bar) = progress {
        let alt_km = (telemetry.state.read().position_m.norm() - R_EQUATOR_M) / 1000.0;
        bar.finish_with_message(if deorbited {
            format!("deorbited at {alt_km:.1} km")
        } else {
            format!("orbit cap reached at {alt_km:.1} km")
        });
    }

    DeorbitResult {
        deorbited,
        final_time_s: sim.current_sim_nanos() as f64 * 1.0e-9,
    }
}

// --- Helpers -----------------------------------------------------------------

/// Parse a `key=value` numeric argument (e.g. `init=300`).
fn parse_kv(args: &[String], key: &str) -> Option<f64> {
    let prefix = format!("{key}=");
    args.iter()
        .find_map(|a| a.strip_prefix(&prefix))
        .and_then(|v| v.parse().ok())
}

fn csv_recorder<T>(topic: &str, output_dir: &Path) -> CsvRecorder<T> {
    CsvRecorder::new(CsvRecorderConfig {
        topic: topic.to_string(),
        output_path: output_dir.join(format!("{topic}.csv")),
    })
}

/// Classical orbital elements -> inertial position and velocity.
/// Matches Basilisk `orbitalMotion.elem2rv`.
fn elem2rv(
    mu: f64,
    a: f64,
    e: f64,
    inc: f64,
    raan: f64,
    aop: f64,
    ta: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let p = a * (1.0 - e * e);
    let r_mag = p / (1.0 + e * ta.cos());

    let (si, ci) = inc.sin_cos();
    let (sr, cr) = raan.sin_cos();
    let (sw, cw) = aop.sin_cos();
    let p_hat = Vector3::new(cr * cw - sr * sw * ci, sr * cw + cr * sw * ci, sw * si);
    let q_hat = Vector3::new(-cr * sw - sr * cw * ci, -sr * sw + cr * cw * ci, cw * si);

    let (sf, cf) = ta.sin_cos();
    let r = r_mag * (cf * p_hat + sf * q_hat);
    let v = (mu / p).sqrt() * (-sf * p_hat + (e + cf) * q_hat);
    (r, v)
}
