//! SPICE orientation reconstruction for a constant-rate spinner: port of
//! upstream `examples/scenarioVestaOrientation.py`. Needs the `mujoco` and
//! `spice` features.
//!
//! A 1 kg free body in an `MJScene` orbits (4) Vesta at 50 km altitude on a
//! circular orbit inclined 40°, deep in Vesta's degree-8 spherical-harmonic
//! field (`VESTA20H.txt`, `just fetch-small-body-data`). `NBodyGravity`
//! rotates that field into Vesta's body-fixed frame with the orientation a
//! `SpiceInterface` publishes (`IAU_VESTA` from `pck00010.tpc`, position
//! disabled, epoch 2025 November 15 12:00 UTC). Both run in the scene's
//! dynamics task, the ephemeris first, so the attitude follows every RK4
//! stage. Three runs over three orbits:
//!
//! * the reference: a 1 s RK4 step, the exact SPICE orientation at every
//!   stage;
//! * a 60 s step with the exact orientation at every stage;
//! * a 60 s step whose orientation is reconstructed from a single SPICE knot
//!   by first-order (velocity) extrapolation of the principal rotation
//!   vector (`setPlanetOrientationReconstruction(TARGET, 10 tf, 1, True,
//!   False)`).
//!
//! Vesta's IAU rotation is a fixed axis at a constant rate, so its principal
//! rotation vector is linear in time and the one-knot extrapolation is
//! exact: the two coarse runs have the same error against the reference
//! (their integrator's), one with a single SPICE query.
//!
//! Writes, to `examples/output/scenarios/`, each run's origin-site position
//! (upstream `body.getOrigin().stateOutMsg.recorder()`, every task step):
//! `scenarioVestaOrientationReference.csv`, `...Exact.csv`,
//! `...Extrapolated.csv`, and the orientation query counts of the coarse
//! runs (the figure legend) in `scenarioVestaOrientationQueries.csv`. The
//! plot script interpolates the reference at the coarse times, as upstream.
//!
//! Upstream `run(show_plots)` has no scenario parameters; its test only
//! checks that a figure was produced. `tests` checks the claims the
//! scenario makes.
//!
//! Differences from upstream: `InitializeSimulation()` followed by
//! `body.setPosition/setVelocity` is [`InitializedScene`], which sets the
//! initial state right after the scene's `init`; upstream's
//! `NBodyGravity` has no `ModelTag` set, here it is `gravity`.

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/recording.rs"]
mod recording;

use std::path::PathBuf;
use std::time::Instant;

use basilisk_rs::dynamics::gravity::SphericalHarmonicsGravityModel;
use basilisk_rs::environment::spice_interface::{
    ReconstructionSettings, SpiceInterface, SpiceInterfaceConfig,
};
use basilisk_rs::integrators::Rk4;
use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use common::scenario_output_path;
use hifitime::Epoch;
use message_log::MessageLog;
use nalgebra::Vector3;
use orbit_tools::sec2nano;
use recording::{scenario_csv, state_columns};

/// Upstream `EPOCH` (UTC), the `SpiceInterface`'s `UTCCalInit`.
const EPOCH: &str = "2025 NOVEMBER 15 12:00:00.000";
/// Upstream `VESTA_SH_FILE`.
const VESTA_SH_FILE: &str = "assets/gravity/VESTA20H.txt";
/// Upstream `SH_DEGREE`.
const SH_DEGREE: usize = 8;
/// Upstream `TARGET`: the SPICE body; `IAU_VESTA` is its body-fixed frame.
const TARGET: &str = "vesta";
/// Upstream `ORBIT_ALT` \[m\].
const ORBIT_ALT_M: f64 = 50.0e3;
/// Upstream `ORBIT_INC` \[deg\].
const ORBIT_INC_DEG: f64 = 40.0;
/// Upstream `COARSE_DT` \[s\]: RK4 step of the two comparison runs.
const COARSE_DT_S: f64 = 60.0;
/// Upstream `REFERENCE_DT` \[s\]: RK4 step of the reference.
const REFERENCE_DT_S: f64 = 1.0;
/// Upstream `N_ORBITS`: propagation horizon in orbital periods.
const N_ORBITS: f64 = 3.0;
/// Upstream's leap-second and planetary-constants kernels
/// (`just fetch-spice-kernels`); the PCK carries Vesta's IAU rotation.
const KERNELS: [&str; 2] = ["assets/spice/naif0012.tls", "assets/spice/pck00010.tpc"];
/// CSV file stem, upstream's figure-name prefix.
const FILE_STEM: &str = "scenarioVestaOrientation";

/// Upstream `SC_XML`: one free body with unit mass and inertia.
const SC_XML: &str = r#"
<mujoco>
    <worldbody>
        <body name="sc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
    </worldbody>
</mujoco>
"#;

/// How a run gets Vesta's orientation.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Attitude {
    /// The exact SPICE orientation at every integrator stage.
    Exact,
    /// Single-knot velocity extrapolation on a knot grid of this step
    /// \[ns\] (upstream `knotStep`).
    Extrapolated { knot_step_nanos: u64 },
    /// No orientation: Vesta's field fixed in the inertial frame (not
    /// upstream; the tests' counterfactual).
    #[cfg_attr(not(test), allow(dead_code))]
    Disabled,
}

/// One propagation: upstream `runOrbit`'s return values.
struct OrbitRun {
    /// Orientation SPICE queries over the run (upstream
    /// `getPlanetSpiceQueryCount(TARGET)[1]`).
    queries: i64,
    /// Recording times \[s\].
    times_s: Vec<f64>,
    /// Origin-site positions \[m\].
    positions_m: Vec<Vector3<f64>>,
    /// Wall-clock run time \[s\].
    wall_clock_s: f64,
}

/// The scene with its initial body state applied right after `init`:
/// upstream's `InitializeSimulation()` followed by `body.setPosition(rN)`
/// and `body.setVelocity(vN)`.
struct InitializedScene {
    scene: MjScene,
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
}

impl Module for InitializedScene {
    fn init(&mut self) {
        self.scene.init();
        let mut body = self.scene.body_mut("sc");
        body.set_position(self.position_m);
        body.set_velocity(self.velocity_mps);
    }

    fn reset(&mut self, context: &SimulationContext) {
        self.scene.reset(context);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.scene.update(context);
    }
}

/// Upstream `runOrbit`: propagates the orbit from `position_m`,
/// `velocity_mps` for `final_time_s` at an RK4 step of `step_s`, writing
/// `examples/output/scenarios/<csv_stem>.csv` when given.
fn run_orbit(
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    final_time_s: f64,
    step_s: f64,
    attitude: Attitude,
    csv_stem: Option<&str>,
) -> OrbitRun {
    let started = Instant::now();
    let step_nanos = sec2nano(step_s);
    let mut scene = MjScene::new(SC_XML).expect("valid scene XML");
    scene.name = "vestaScene".to_string();
    scene.set_integrator(Rk4);

    // Attitude producer: Vesta's orientation only, position disabled. In the
    // dynamics task ahead of gravity, so the attitude is current at every
    // integrator stage.
    let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
        name: "vestaOrient".to_string(),
        kernel_paths: KERNELS.iter().map(PathBuf::from).collect(),
        utc_cal_init: Some(EPOCH.to_string()),
        planet_names: vec![TARGET.to_string()],
        ..Default::default()
    });
    spice.set_planet_position_disabled(TARGET);
    match attitude {
        Attitude::Exact => {}
        Attitude::Extrapolated { knot_step_nanos } => spice
            .set_planet_orientation_reconstruction(
                TARGET,
                ReconstructionSettings {
                    knot_step_nanos,
                    n_knots: 1,
                    use_velocity: true,
                    interpolate: false,
                },
            )
            .expect("positive knot step"),
        Attitude::Disabled => spice.set_planet_orientation_disabled(TARGET),
    }
    let vesta_state = spice.planet_state_out_msgs[0].clone();
    let spice_id = scene.add_module_to_dynamics_task("vestaOrient", spice, -1);

    // Vesta's degree-8 field, rotated into the body frame by the supplied
    // orientation: what makes the acceleration attitude dependent.
    let mut gravity = NBodyGravity::new("gravity");
    let field = SphericalHarmonicsGravityModel::from_file(VESTA_SH_FILE, SH_DEGREE)
        .unwrap_or_else(|error| panic!("{error} (just fetch-small-body-data)"));
    vesta_state.connect_to(&mut gravity.add_gravity_source(TARGET, field, true).state_in_msg);
    gravity.add_gravity_target_at_body(&mut scene, "sc", "sc");
    scene.add_model_to_dynamics_task(gravity, -1);

    let origin_state = scene.body("sc").origin().state_out_msg.clone();
    let mut scene = InitializedScene {
        scene,
        position_m,
        velocity_mps,
    };
    let mut history = MessageLog::<SpacecraftStateMsg>::default();
    let mut csv = csv_stem.map(scenario_csv);
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 11, 15), false);
        connect!(&simulation, &origin_state => &mut history.input);
        schedule! { simulation,
            "vestaScene" => &mut scene, period=step_nanos, priority=10;
            "scRec" => &mut history, period=step_nanos, priority=0;
        }
        if let Some(csv) = csv.as_mut() {
            connect!(&simulation,
                &origin_state => csv.add_source::<SpacecraftStateMsg>(state_columns()),
            );
            simulation.add_module("csv", csv, step_nanos, 0);
        }
        simulation.run_for(sec2nano(final_time_s));
    }
    if let Some(csv) = csv.as_mut() {
        csv.flush().expect("flush scenario CSV");
    }

    let queries = scene
        .scene
        .task_model(spice_id)
        .module
        .planet_spice_query_count(TARGET)
        .1;
    OrbitRun {
        queries,
        times_s: history.times_s(),
        positions_m: history
            .values
            .iter()
            .map(|state| state.position_m)
            .collect(),
        wall_clock_s: started.elapsed().as_secs_f64(),
    }
}

/// `values` (sampled at `times`) linearly interpolated at `time`: numpy
/// `interp`, holding the end values outside the samples.
fn interpolate(times: &[f64], values: &[Vector3<f64>], time: f64) -> Vector3<f64> {
    let upper = times.partition_point(|sample| *sample < time);
    if upper == 0 {
        return values[0];
    }
    if upper == times.len() {
        return values[times.len() - 1];
    }
    let (t0, t1) = (times[upper - 1], times[upper]);
    let fraction = (time - t0) / (t1 - t0);
    values[upper - 1] + (values[upper] - values[upper - 1]) * fraction
}

/// Position error \[m\] of `run` against `reference`, at `run`'s times.
fn errors_against(reference: &OrbitRun, run: &OrbitRun) -> Vec<f64> {
    run.times_s
        .iter()
        .zip(&run.positions_m)
        .map(|(time, position)| {
            (position - interpolate(&reference.times_s, &reference.positions_m, *time)).norm()
        })
        .collect()
}

/// The study's inputs: upstream `run`'s set-up.
// `mu_m3ps2` and `radius_m` are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Setup {
    /// Vesta's `mu` \[m^3/s^2\] and reference radius \[m\], from the file
    /// header.
    mu_m3ps2: f64,
    radius_m: f64,
    position_m: Vector3<f64>,
    velocity_mps: Vector3<f64>,
    /// Upstream `tf` \[s\].
    final_time_s: f64,
}

fn setup() -> Setup {
    let field = SphericalHarmonicsGravityModel::from_file(VESTA_SH_FILE, SH_DEGREE)
        .unwrap_or_else(|error| panic!("{error} (just fetch-small-body-data)"));
    let (mu_m3ps2, radius_m) = (field.mu_m3ps2(), field.reference_radius_m());
    let elements = ClassicElements {
        semi_major_axis: radius_m + ORBIT_ALT_M,
        eccentricity: 0.0,
        inclination_rad: ORBIT_INC_DEG.to_radians(),
        ..Default::default()
    };
    let (position_m, velocity_mps) = elements_to_state(mu_m3ps2, &elements);
    let period_s =
        2.0 * std::f64::consts::PI * (elements.semi_major_axis.powi(3) / mu_m3ps2).sqrt();
    Setup {
        mu_m3ps2,
        radius_m,
        position_m,
        velocity_mps,
        final_time_s: N_ORBITS * period_s,
    }
}

/// The study's results.
// Some fields are only read by the tests.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    setup: Setup,
    reference: OrbitRun,
    exact: OrbitRun,
    extrapolated: OrbitRun,
    /// Upstream `errExact`, `errOne` \[m\].
    exact_errors_m: Vec<f64>,
    extrapolated_errors_m: Vec<f64>,
}

/// Upstream `run`, writing the CSV files when `write_csv`.
fn run(write_csv: bool) -> Results {
    let setup = setup();
    let stem = |suffix: &str| format!("{FILE_STEM}{suffix}");
    let csv = |suffix: &'static str| write_csv.then(|| stem(suffix));
    let orbit = |step_s, attitude, csv_stem: Option<String>| {
        run_orbit(
            setup.position_m,
            setup.velocity_mps,
            setup.final_time_s,
            step_s,
            attitude,
            csv_stem.as_deref(),
        )
    };

    // Integrator-converged truth: the exact attitude at a fine step.
    let reference = orbit(REFERENCE_DT_S, Attitude::Exact, csv("Reference"));
    // Two coarse runs that differ only in how the attitude is supplied.
    let exact = orbit(COARSE_DT_S, Attitude::Exact, csv("Exact"));
    let extrapolated = orbit(
        COARSE_DT_S,
        Attitude::Extrapolated {
            knot_step_nanos: sec2nano(10.0 * setup.final_time_s),
        },
        csv("Extrapolated"),
    );
    let exact_errors_m = errors_against(&reference, &exact);
    let extrapolated_errors_m = errors_against(&reference, &extrapolated);

    if write_csv {
        let path = scenario_output_path(&format!("{FILE_STEM}Queries.csv"));
        std::fs::write(
            &path,
            format!(
                "run,orientation_queries\nExact,{}\nExtrapolated,{}\n",
                exact.queries, extrapolated.queries
            ),
        )
        .expect("write query counts");
    }
    Results {
        setup,
        reference,
        exact,
        extrapolated,
        exact_errors_m,
        extrapolated_errors_m,
    }
}

fn peak(values: &[f64]) -> f64 {
    values.iter().copied().fold(0.0, f64::max)
}

fn main() {
    Cli::from_env("scenario_vesta_orientation").finish();
    let results = run(true);
    println!(
        "[study] Vesta {N_ORBITS}-orbit run at {} km altitude",
        ORBIT_ALT_M / 1e3
    );
    println!("[study] reference: exact SPICE at {REFERENCE_DT_S}s step (integrator-converged)");
    println!(
        "[study] coarse ({COARSE_DT_S}s) exact:  {} queries, peak err {:.2} m, {:.0} ms",
        results.exact.queries,
        peak(&results.exact_errors_m),
        results.exact.wall_clock_s * 1e3
    );
    println!(
        "[study] coarse ({COARSE_DT_S}s) extrap: {} query, peak err {:.2} m, {:.0} ms",
        results.extrapolated.queries,
        peak(&results.extrapolated_errors_m),
        results.extrapolated.wall_clock_s * 1e3
    );
    println!(
        "[study] runtime: {:+.0}% from reconstruction (within timing noise on this short run; SH \
         gravity dominates the step, not SPICE)",
        100.0 * (1.0 - results.extrapolated.wall_clock_s / results.exact.wall_clock_s)
    );
    let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("examples/output/scenarios");
    for suffix in ["Reference", "Exact", "Extrapolated", "Queries"] {
        println!(
            "wrote {}",
            directory.join(format!("{FILE_STEM}{suffix}.csv")).display()
        );
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_scenarioVestaOrientation.py` only checks that the
    //! scenario produces a figure. These tests check what it demonstrates.

    use super::*;

    /// The coarse runs record every 60 s step of the three orbits; the
    /// single-knot run spends one orientation query (a knot fill), the
    /// exact one a query per RK4 stage.
    #[test]
    fn extrapolation_needs_one_query_and_matches_the_exact_attitude() {
        let results = run(false);
        let steps = (results.setup.final_time_s / COARSE_DT_S).floor() as usize;
        assert_eq!(results.exact.times_s.len(), steps + 1);
        let fine_steps = (results.setup.final_time_s / REFERENCE_DT_S).floor() as usize;
        assert_eq!(results.reference.times_s.len(), fine_steps + 1);
        assert_eq!(results.extrapolated.times_s.len(), steps + 1);
        assert!(
            results.extrapolated.queries == 1,
            "{}",
            results.extrapolated.queries
        );
        assert!(
            results.exact.queries >= 4 * steps as i64,
            "{} queries for {steps} steps",
            results.exact.queries
        );

        // The two coarse runs lie on top of each other: the extrapolated
        // attitude matches SPICE to roundoff, which leaves the trajectories
        // micrometres apart after three orbits.
        for (exact, extrapolated) in results
            .exact_errors_m
            .iter()
            .zip(&results.extrapolated_errors_m)
        {
            assert!(
                (exact - extrapolated).abs() < 1.0e-4,
                "{exact} vs {extrapolated}"
            );
        }
        // Their error is the integrator's: small next to the 315 km orbit,
        // but not zero.
        let peak_error = peak(&results.exact_errors_m);
        assert!(peak_error > 1.0e-3 && peak_error < 1.0e3, "{peak_error} m");
    }

    /// The low orbit stays bound around Vesta, above its surface, and the
    /// fine run is integrator-converged against the coarse one far below the
    /// orbit's own size.
    #[test]
    fn the_reference_orbit_stays_around_vesta() {
        let setup = setup();
        let reference = run_orbit(
            setup.position_m,
            setup.velocity_mps,
            setup.final_time_s,
            REFERENCE_DT_S,
            Attitude::Exact,
            None,
        );
        for position in &reference.positions_m {
            let radius = position.norm();
            assert!(
                radius > setup.radius_m && radius < 2.0 * setup.radius_m,
                "{radius} m"
            );
        }
        // Vesta's field rotates with Vesta: freezing its orientation moves
        // the coarse orbit by far more than the integrator error.
        let frozen = run_orbit(
            setup.position_m,
            setup.velocity_mps,
            setup.final_time_s,
            COARSE_DT_S,
            Attitude::Disabled,
            None,
        );
        assert_eq!(frozen.queries, 0);
        let frozen_error = peak(&errors_against(&reference, &frozen));
        assert!(frozen_error > 1.0e3, "{frozen_error} m");
        assert!(setup.mu_m3ps2 > 1.7e10 && setup.mu_m3ps2 < 1.8e10);
    }
}
