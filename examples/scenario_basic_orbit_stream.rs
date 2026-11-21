//! Port of upstream `examples/scenarioBasicOrbitStream.py`: the basic orbit
//! scenario stepped one task at a time while streaming to Vizard, with
//! keyboard and dialog interaction. Needs the `vizard` feature:
//!
//! ```text
//! cargo run --release --features vizard --example scenario_basic_orbit_stream -- --live-stream
//! ```
//!
//! A 50 kg spacecraft with one MOOG Monarc-22-6 thruster (22 N along body
//! -y, [`ThrusterDynamicEffector`]) on a LEO (`a = 17 500 km`, `e = 0.1`),
//! GTO or GEO orbit about a point-mass Earth or Mars barycenter; with
//! `--spherical-harmonics` the Earth's J2-only field or Mars's GGM2B field to
//! degree 80, with the planet's `IAU_*` orientation from the ANISE ephemeris
//! (upstream `createSpiceInterface`). One orbit (three with spherical
//! harmonics), sampled 100 (400) times; `--step` is upstream's `timeStep`
//! (default 10 s).
//!
//! [`VizInterface`] streams live (`--live-stream`, two-way on
//! `tcp://0.0.0.0:5556`, with a 50x [`ClockSync`]) or broadcasts
//! (`--broadcast-stream`, `tcp://0.0.0.0:5570`); `--save <file>` writes the
//! Vizard file instead (upstream `saveFile`). As upstream, the loop steps the
//! simulation one task at a time, shows a HUD dialog with altitude and speed,
//! an information panel at step 100, and handles the keys Vizard reports:
//! `b` offers a burn (the dialog reply starts 100 s thruster on-times every
//! step), `x` stops it, `p` pauses (re-sending the frame), `z` ends the run and
//! `q` ends it and closes Vizard.
//!
//! The loop changes the Vizard interface (its dialogs and live settings)
//! between steps, as upstream's script does. The simulation holds its modules
//! for its whole life, so the interface and the clock are shared with the
//! loop through a [`Shared`] module (an `Arc<Mutex<_>>` the simulation steps).
//!
//! Writes `scenarioBasicOrbitStream<orbit><sh><planet>.csv`; `--all` runs the
//! five parametrizations of `test_scenarioBasicOrbitStream.py` (no streaming).
//! Upstream's test only runs them; `tests` also checks the point-mass orbits
//! against the Kepler solution and the Vizard file's frames.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use std::path::PathBuf;
use std::sync::{Arc, Mutex, MutexGuard};
use std::time::{Duration, Instant};

use basilisk_rs::clock_sync::ClockSync;
use basilisk_rs::dynamics::thruster_dynamic_effector::{
    ThrusterDynamicEffector, ThrusterDynamicEffectorConfig,
};
use basilisk_rs::messages::{Output, SpacecraftStateMsg, ThrusterCommandMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{
    ClassicElements, elements_to_state, state_to_elements,
};
use basilisk_rs::vizard::viz_interface::{VizInterface, VizInterfaceConfig};
use basilisk_rs::vizard::viz_structures::{
    GravBodyInfo, ThrClusterMap, VizEventDialog, VizSetting,
};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use grav_factory::{Body, GGM2B, GGM03S_J2_ONLY};
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{nano2sec, orbit_period_s, run_until, sec2nano};
use recording::{sampling_time, scenario_csv, state_columns};

const USAGE: &str = "\
scenario_basic_orbit_stream [--orbit LEO|GTO|GEO] [--planet Earth|Mars] [--spherical-harmonics]
                            [--step 10] [--live-stream] [--broadcast-stream] [--save <file.bin>]
scenario_basic_orbit_stream --all     run the five upstream test cases (no streaming)";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OrbitCase {
    Leo,
    Gto,
    Geo,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PlanetCase {
    Earth,
    Mars,
}

/// Upstream `run(show_plots, liveStream, broadcastStream, timeStep,
/// orbitCase, useSphericalHarmonics, planetCase)`.
#[derive(Clone, Debug)]
pub struct Config {
    pub live_stream: bool,
    pub broadcast_stream: bool,
    pub step_seconds: f64,
    pub orbit: OrbitCase,
    pub spherical_harmonics: bool,
    pub planet: PlanetCase,
    /// Upstream `saveFile`.
    pub save_file: Option<PathBuf>,
}

impl Config {
    pub fn label(&self) -> String {
        let orbit = match self.orbit {
            OrbitCase::Leo => "LEO",
            OrbitCase::Gto => "GTO",
            OrbitCase::Geo => "GEO",
        };
        let planet = match self.planet {
            PlanetCase::Earth => "Earth",
            PlanetCase::Mars => "Mars",
        };
        format!("{orbit}{}{planet}", u8::from(self.spherical_harmonics))
    }
}

/// The parametrizations of `test_scenarioBasicOrbitStream.py`.
pub fn test_cases() -> Vec<Config> {
    [
        (OrbitCase::Leo, false, PlanetCase::Earth),
        (OrbitCase::Gto, false, PlanetCase::Earth),
        (OrbitCase::Geo, false, PlanetCase::Earth),
        (OrbitCase::Leo, true, PlanetCase::Earth),
        (OrbitCase::Leo, false, PlanetCase::Mars),
    ]
    .into_iter()
    .map(|(orbit, spherical_harmonics, planet)| Config {
        live_stream: false,
        broadcast_stream: false,
        step_seconds: 10.0,
        orbit,
        spherical_harmonics,
        planet,
        save_file: None,
    })
    .collect()
}

/// A module shared between the simulation, which steps it, and the scenario
/// loop, which changes it between steps.
pub struct Shared<M>(Arc<Mutex<M>>);

impl<M> Shared<M> {
    pub fn new(module: M) -> Self {
        Self(Arc::new(Mutex::new(module)))
    }

    pub fn lock(&self) -> MutexGuard<'_, M> {
        self.0
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
    }

    pub fn handle(&self) -> Self {
        Self(Arc::clone(&self.0))
    }
}

impl<M: Module> Module for Shared<M> {
    fn init(&mut self) {
        self.lock().init();
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.lock().reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.lock().update(context);
    }
}

fn dialog(id: &str, text: &str, format: &str) -> VizEventDialog {
    VizEventDialog {
        event_handler_id: id.to_string(),
        display_string: text.to_string(),
        duration_of_display_s: 0.0,
        dialog_format: format.to_string(),
        ..VizEventDialog::default()
    }
}

pub struct Results {
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub mu_m3ps2: f64,
    pub elements: ClassicElements,
    pub duration_nanos: u64,
}

pub fn run(config: &Config) -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let step_nanos = sec2nano(config.step_seconds);
    let body = match config.planet {
        PlanetCase::Mars => Body::MarsBarycenter,
        PlanetCase::Earth => Body::Earth,
    };
    let gravity = match (config.spherical_harmonics, config.planet) {
        (false, _) => body.point_mass(true),
        (true, PlanetCase::Earth) => body.spherical_harmonics(GGM03S_J2_ONLY, 2, true),
        (true, PlanetCase::Mars) => body.spherical_harmonics(GGM2B, 80, true),
    };
    let mu = body.mu_m3ps2();
    let (r_leo, r_geo) = (7000.0 * 1000.0, 42000.0 * 1000.0);
    let (semi_major_axis, eccentricity, inclination_rad) = match config.orbit {
        OrbitCase::Geo => (r_geo, 0.00001, 0.0),
        OrbitCase::Gto => {
            let a = (r_leo + r_geo) / 2.0;
            (a, 1.0 - r_leo / a, 0.0)
        }
        OrbitCase::Leo => (2.5 * r_leo, 0.10, 33.3 * degrees),
    };
    let (position_m, velocity_mps) = elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis,
            eccentricity,
            inclination_rad,
            raan_rad: 48.2 * degrees,
            arg_periapsis_rad: 347.8 * degrees,
            true_anomaly_rad: 85.3 * degrees,
            ..Default::default()
        },
    );
    let elements = state_to_elements(mu, position_m, velocity_mps);
    let period_s = orbit_period_s(mu, elements.semi_major_axis);
    let (duration_nanos, points) = if config.spherical_harmonics {
        (sec2nano(3.0 * period_s), 400)
    } else {
        (sec2nano(period_s), 100)
    };
    let sample_nanos = sampling_time(duration_nanos, step_nanos, points);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 50.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(60.0, 30.0, 40.0)),
        integration_step_nanos: step_nanos,
        initial_position_m: position_m,
        initial_velocity_mps: velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft.add_grav_body(gravity).expect("one central body");
    let state_out = spacecraft.state_out.clone();
    // Upstream `createSpiceInterface(time="2012 MAY 1 00:28:30.0")` with the
    // planet as zero base, for its orientation.
    let mut planet_ephemeris = config.spherical_harmonics.then(|| body.ephemeris(body));

    // Upstream `thFactory.create("MOOG_Monarc_22_6", [0, 0, 0], [0, -1.5, 0])`,
    // the direction normalized.
    let mut thruster_config = ThrusterDynamicEffectorConfig::generic_pulsed(
        "ACSThruster",
        Vector3::zeros(),
        -Vector3::y(),
        22.0,
        0.020,
        229.5,
    );
    thruster_config.area_nozzle_m2 = 0.0045;
    let mut thruster = ThrusterDynamicEffector::new(thruster_config);
    let thruster_command = Output::new(ThrusterCommandMsg::default());
    let thruster_out = thruster.thruster_out_msg.clone();

    let viz = Shared::new(VizInterface::new(VizInterfaceConfig {
        save_file: config.save_file.clone(),
        live_stream: config.live_stream,
        broadcast_stream: config.broadcast_stream,
        ..VizInterfaceConfig::default()
    }));
    let user_input = {
        let mut viz = viz.lock();
        viz.settings.keyboard_live_input = "bxpqz".to_string();
        viz.add_celestial_body(GravBodyInfo {
            body_name: body.name().to_string(),
            mu_m3ps2: mu,
            rad_equator_m: body.radius_equator_m(),
            radius_ratio: 1.0,
            ..GravBodyInfo::default()
        });
        viz.user_input_msg.clone()
    };
    let clock = Shared::new(ClockSync::new(50.0, 10_000_000, false));

    let mut csv = scenario_csv(&format!("scenarioBasicOrbitStream{}", config.label()));
    let mut history = MessageLog::<SpacecraftStateMsg>::default();
    let start = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
    {
        let mut simulation = Simulation::new(start, false);
        connect!(&simulation,
            &thruster_command => &mut thruster.command_in,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            &spacecraft.state_out => &mut history.input,
        );
        {
            let mut viz = viz.lock();
            let spacecraft_data = viz.add_spacecraft("bskSat");
            simulation.connect(&spacecraft.state_out, &mut spacecraft_data.sc_state_in_msg);
            spacecraft_data.thrusters.push(Default::default());
            let viz_thruster = spacecraft_data.thrusters.last_mut().expect("pushed");
            viz_thruster.thr_info = ThrClusterMap {
                thr_tag: "ACSThrusterDynamics".to_string(),
                color: Some([255, 255, 255, 255]),
                ..ThrClusterMap::default()
            };
            simulation.connect(&thruster_out, &mut viz_thruster.thr_in_msg);
        }
        spacecraft.add_dynamic_effector(thruster);
        if let Some(ephemeris) = planet_ephemeris.as_mut() {
            simulation.connect(
                &ephemeris.output_planet_msg,
                spacecraft.grav_body_input_mut(body.name()).expect("planet"),
            );
            simulation.add_module("spiceObject", ephemeris, step_nanos, 100);
        }
        let mut viz_module = viz.handle();
        let mut clock_module = clock.handle();
        schedule! { simulation,
            "bskSat" => &mut spacecraft, period=step_nanos, priority=10;
            "csv" => &mut csv, period=sample_nanos, priority=0;
            "dataLog" => &mut history, period=sample_nanos, priority=0;
        }
        if config.live_stream {
            simulation.add_module("clockSync", &mut clock_module, step_nanos, -5);
        }
        simulation.add_module("vizInterface", &mut viz_module, step_nanos, -10);

        let mut info_panel = dialog(
            "INFO PANEL",
            "This is an information panel. Vizard is reporting 'b', 'p', 'x' and 'z' keystrokes back \
             to BSK, which you can hook up to specific sim states. In this case, press 'b' to show a \
             burn panel. If you initiate a burn, press 'x' to stop the burn. Press 'p' to pause the \
             simulation, 'z' to stop the simulation, 'q' to stop the simulation and cleanly quit Vizard.",
            "CAUTION",
        );
        info_panel.hide_on_selection = 0;
        let mut burn_panel = dialog(
            "OPTION PANEL",
            "This panel accepts a user response. Initiate burn?",
            "WARNING",
        );
        burn_panel.hide_on_selection = 1;
        burn_panel.user_options = vec!["Yes".to_string(), "No".to_string()];
        burn_panel.use_confirmation_panel = VizSetting::Enabled;

        let mut stop_nanos = 0;
        let mut continue_burn = false;
        let (mut paused, mut end) = (false, false);
        let mut prior_key_press = Instant::now() - Duration::from_secs(2);
        while stop_nanos < duration_nanos && !end {
            let state = state_out.read();
            let altitude_m = state.position_m.norm() - body.radius_equator_m();
            viz.lock().viz_event_dialogs.push(dialog(
                "HUD",
                &format!(
                    "HUD\nAltitude: {:.2} km\nInertial Velocity: {:.2} km/s",
                    altitude_m / 1000.0,
                    state.velocity_mps.norm() / 1000.0
                ),
                "none",
            ));
            if paused {
                // Upstream `viz.UpdateState(t)` and `clockSync.Reset(0)`.
                viz.lock().update(&simulation.context());
                clock.lock().reset(&simulation.context());
            } else {
                stop_nanos += step_nanos;
                run_until(&mut simulation, stop_nanos);
            }
            if !config.live_stream {
                continue;
            }
            let inputs = user_input.read();
            if prior_key_press.elapsed() > Duration::from_secs(1) {
                let keys = inputs.keyboard_input.as_str();
                if keys.contains('b') {
                    prior_key_press = Instant::now();
                    if !continue_burn {
                        viz.lock().viz_event_dialogs.push(burn_panel.clone());
                    }
                }
                if keys.contains('q') {
                    let mut viz = viz.lock();
                    viz.live_settings.terminate_vizard = true;
                    viz.update(&simulation.context());
                    end = true;
                }
                if keys.contains('x') {
                    prior_key_press = Instant::now();
                    if continue_burn {
                        continue_burn = false;
                        thruster_command.write(ThrusterCommandMsg {
                            on_time_s: 0.0,
                            command_time_nanos: stop_nanos,
                        });
                    }
                }
                if keys.contains('z') {
                    end = true;
                }
                if keys.contains('p') {
                    prior_key_press = Instant::now();
                    paused = !paused;
                }
            }
            for reply in &inputs.viz_event_replies {
                if reply.event_handler_id == "OPTION PANEL" {
                    continue_burn = reply.reply == "Yes";
                }
            }
            if stop_nanos == 100 * step_nanos {
                viz.lock().viz_event_dialogs.push(info_panel.clone());
            }
            if continue_burn {
                thruster_command.write(ThrusterCommandMsg {
                    on_time_s: 100.0,
                    command_time_nanos: stop_nanos,
                });
            }
        }
    }
    csv.flush().expect("flush scenario CSV");
    drop(viz);

    Results {
        states: history.into_samples(),
        mu_m3ps2: mu,
        elements,
        duration_nanos,
    }
}

fn main() {
    let mut args = Cli::from_env(USAGE);
    let configs = if args.switch("all") {
        test_cases()
    } else {
        let orbit = match args.value("orbit", "LEO".to_string()).as_str() {
            "LEO" => OrbitCase::Leo,
            "GTO" => OrbitCase::Gto,
            "GEO" => OrbitCase::Geo,
            other => args.fail(&format!("unknown orbit case {other:?}")),
        };
        let planet = match args.value("planet", "Earth".to_string()).as_str() {
            "Earth" => PlanetCase::Earth,
            "Mars" => PlanetCase::Mars,
            other => args.fail(&format!("unknown planet case {other:?}")),
        };
        let save: String = args.value("save", String::new());
        vec![Config {
            live_stream: args.switch("live-stream"),
            broadcast_stream: args.switch("broadcast-stream"),
            step_seconds: args.value("step", 10.0),
            orbit,
            spherical_harmonics: args.switch("spherical-harmonics"),
            planet,
            save_file: (!save.is_empty()).then(|| PathBuf::from(save)),
        }]
    };
    for config in configs {
        let results = run(&config);
        let (time, _) = results.states.last().expect("samples");
        println!(
            "wrote scenarioBasicOrbitStream{}.csv ({} samples to {:.0} s)",
            config.label(),
            results.states.len(),
            nano2sec(*time)
        );
    }
}

#[cfg(test)]
mod tests {
    use basilisk_rs::vizard::viz_interface::read_viz_file;

    use super::{run, test_cases};
    use crate::orbit_tools::{kepler_state, nano2sec};

    /// `test_scenarioBasicOrbitStream.py` runs five cases with streaming off
    /// and checks nothing. Here the cases save their Vizard file, which holds
    /// one frame per step after the first, and the point-mass orbits follow
    /// the Kepler solution (the thruster is never commanded).
    #[test]
    fn upstream_cases_run_and_save_vizard_frames() {
        for mut config in test_cases() {
            let path = std::env::temp_dir().join(format!(
                "basilisk-rs-orbit-stream-{}-{}.bin",
                config.label(),
                std::process::id()
            ));
            config.save_file = Some(path.clone());
            let results = run(&config);
            let frames = read_viz_file(&path).expect("readable Vizard file");
            std::fs::remove_file(&path).ok();
            let steps = results.duration_nanos.div_ceil(10_000_000_000);
            assert_eq!(frames.len() as u64, steps, "{}", config.label());

            if !config.spherical_harmonics {
                let (time, last) = results.states.last().expect("samples");
                let (kepler, _) =
                    kepler_state(results.mu_m3ps2, &results.elements, nano2sec(*time));
                let error = (last.position_m - kepler).norm();
                assert!(
                    error < 10.0,
                    "{}: {error} m from the Kepler orbit",
                    config.label()
                );
            }
        }
    }
}
