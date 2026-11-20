//! The accuracy and cost of reconstructing an ephemeris from a coarse knot
//! grid, on a cislunar flyby: port of upstream
//! `examples/scenarioSpiceReconstruction.py`.
//!
//! A transfer from a 600 km perigee is aimed at the Moon's position at
//! arrival and pulled short (`APOGEE_FRACTION = 0.94`), so the spacecraft
//! turns around about 18 000 km from the Moon and the flyby bends its path:
//! the Moon's position supplied to the gravity model drives the final state.
//! From 2025 November 15 12:00 UTC the orbit is propagated for 15 600 RK4
//! steps of 30 s (130 h) with the Earth (central) and the Moon as point
//! masses:
//!
//! * the **reference**: a one-body MuJoCo scene (`MJScene`) with
//!   `NBodyGravity`, `spiceInterface` in its dynamics task, so the Moon is
//!   queried exactly at every RK4 stage;
//! * the **integrator floor**: the same at a 2 s step, the converged truth
//!   the 30 s reference is differenced against, its error with a perfect
//!   ephemeris;
//! * the **reconstruction sweep**: the reference with the Moon's position
//!   reconstructed (cubic Hermite, upstream's defaults) from knot grids of
//!   64 to 6144 task steps, error against the reference versus Moon queries;
//! * the **classic spacecraft**: `spacecraft` with its gravity effector,
//!   `spiceInterface` queried once per task step and the Moon extrapolated
//!   linearly across the RK4 stages.
//!
//! Then one more reconstruction run on the coarsest grid whose final error
//! is under the integrator floor, and the classic path with that grid, both
//! timed against their exact counterparts.
//!
//! Needs the `mujoco` and `spice` features (the reconstruction is a feature
//! of the CSPICE interface):
//!
//! ```text
//! cargo run --release --features mujoco,spice --example scenario_spice_reconstruction
//! ```
//!
//! The kernels are upstream's (`de430.bsp`, `naif0012.tls`,
//! `de-403-masses.tpc`, `pck00010.tpc`, `just fetch-spice-kernels`). The
//! initial state comes from the same CSPICE interface queried at the
//! arrival times (upstream's direct `spkezr_c` calls), rounded to whole
//! nanoseconds.
//!
//! One difference from upstream: upstream's classic run adds the
//! `spiceInterface` after the spacecraft, whose gravity effector
//! Euler-steps the Moon from the time its message was written. basilisk-rs's
//! gravity effector takes a connected planet message as written at the
//! current update, so the interface runs before the spacecraft here, as
//! every scenario does.
//!
//! Upstream `run(show_plots)` returns the classic path's final error
//! against the reference, which `test_scenarioSpiceReconstruction.py`
//! requires under 10 m. Writes to `examples/output/scenarios/`:
//! `scenarioSpiceReconstruction.csv` (the reference trajectory and the
//! Moon's), `scenarioSpiceReconstructionSweep.csv` (one row per knot grid,
//! then the classic and exact references, with the floor) and
//! `scenarioSpiceReconstructionErrors.csv` (the errors over time).

#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/mj_scene.rs"]
mod mj_scene;
#[path = "support/table_csv.rs"]
mod table_csv;

use std::path::PathBuf;
use std::time::Instant;

use basilisk_rs::dynamics::gravity::PointMassGravityModel;
use basilisk_rs::environment::spice_interface::{
    ReconstructionSettings, SpiceInterface, SpiceInterfaceConfig,
};
use basilisk_rs::integrators::Rk4;
use basilisk_rs::messages::{PlanetStateMsg, SpacecraftStateMsg};
use basilisk_rs::mujoco_dynamics::MjScene;
use basilisk_rs::mujoco_dynamics::n_body_gravity::NBodyGravity;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::{Module, SimulationContext, connect, schedule};
use cli::Cli;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use mj_scene::{InitializedScene, interpolate};
use nalgebra::{Matrix3, Vector3};

/// Upstream `EPOCH` (UTC).
const EPOCH: &str = "2025 NOVEMBER 15 12:00:00.000";
/// Upstream `RK4_DT` [s].
const RK4_DT_S: f64 = 30.0;
/// Upstream `N_STEPS`.
const N_STEPS: u64 = 15_600;
/// Upstream `REFERENCE_DT` [s].
const REFERENCE_DT_S: f64 = 2.0;
/// Upstream `APOGEE_FRACTION`.
const APOGEE_FRACTION: f64 = 0.94;
/// Upstream `KNOT_MULTIPLES`: knot spacings in task steps.
const KNOT_MULTIPLES: [u64; 9] = [64, 256, 512, 1024, 1536, 2048, 3072, 4096, 6144];
/// Upstream's kernels.
const KERNELS: [&str; 4] = [
    "assets/spice/de430.bsp",
    "assets/spice/naif0012.tls",
    "assets/spice/de-403-masses.tpc",
    "assets/spice/pck00010.tpc",
];
/// Upstream `SC_XML`: a free-flying point mass.
const SC_XML: &str = r#"
<mujoco>
    <worldbody>
        <body name="sc"><freejoint/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
    </worldbody>
</mujoco>
"#;

fn sec2nano(seconds: f64) -> u64 {
    (seconds * 1e9).round() as u64
}

/// Upstream `TF` [ns].
fn final_time_nanos() -> u64 {
    sec2nano(RK4_DT_S) * N_STEPS
}

/// The simulation's start epoch, [`EPOCH`].
fn start_epoch() -> Epoch {
    Epoch::from_gregorian_utc(2025, 11, 15, 12, 0, 0, 0)
}

/// Upstream `gravFactory.createSpiceInterface(time=EPOCH)` for the Earth and
/// the Moon, `zeroBase = "Earth"`, the Moon's position reconstructed on a
/// grid of `knot_step_nanos` when given.
fn spice_interface(knot_step_nanos: Option<u64>) -> SpiceInterface {
    let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
        name: "spiceObject".to_string(),
        kernel_paths: KERNELS.iter().map(PathBuf::from).collect(),
        zero_base: "Earth".to_string(),
        utc_cal_init: Some(EPOCH.to_string()),
        planet_names: vec!["earth".to_string(), "moon".to_string()],
        ..SpiceInterfaceConfig::default()
    });
    if let Some(knot_step_nanos) = knot_step_nanos {
        spice
            .set_planet_position_reconstruction(
                "moon",
                ReconstructionSettings {
                    knot_step_nanos,
                    ..ReconstructionSettings::default()
                },
            )
            .expect("positive knot step");
    }
    spice
}

/// One propagation: upstream's `(rFinal, nQ, [times, positions, Moon])`.
struct Run {
    final_position_m: Vector3<f64>,
    /// Moon position queries (upstream `getPlanetSpiceQueryCount("moon")[0]`).
    moon_queries: i64,
    times_s: Vec<f64>,
    positions_m: Vec<Vector3<f64>>,
    moon_positions_m: Vec<Vector3<f64>>,
    wall_clock_s: f64,
}

/// Upstream `runMJScene`: the orbit in a MuJoCo scene at RK4 step `dt_s`,
/// the Moon exact at every stage or reconstructed on `knot_step_nanos`.
fn run_mj_scene(
    initial: (Vector3<f64>, Vector3<f64>),
    dt_s: f64,
    knot_step_nanos: Option<u64>,
    record: bool,
) -> Run {
    let started = Instant::now();
    let step_nanos = sec2nano(dt_s);
    let mut scene = MjScene::new(SC_XML).expect("valid scene XML");
    scene.name = "cislunarScene".to_string();
    scene.set_integrator(Rk4);

    // The SPICE producer first in the dynamics task, so the planet states
    // are current at every stage; then the gravity model, Earth central.
    let spice = spice_interface(knot_step_nanos);
    let earth_state = spice.planet_state_out_msgs[0].clone();
    let moon_state = spice.planet_state_out_msgs[1].clone();
    let spice_id = scene.add_module_to_dynamics_task("spiceObject", spice, -1);
    let mut gravity = NBodyGravity::new("gravity");
    let earth = PointMassGravityModel::new(Body::Earth.mu_m3ps2()).expect("positive mu");
    let moon = PointMassGravityModel::new(Body::Moon.mu_m3ps2()).expect("positive mu");
    earth_state.connect_to(
        &mut gravity
            .add_gravity_source("earth", earth, true)
            .state_in_msg,
    );
    moon_state.connect_to(&mut gravity.add_gravity_source("moon", moon, false).state_in_msg);
    gravity.add_gravity_targets_for_all_bodies(&mut scene);
    scene.add_model_to_dynamics_task(gravity, -1);

    let origin_state = scene.body("sc").origin().state_out_msg.clone();
    let mut scene = InitializedScene {
        scene,
        body: "sc".to_string(),
        position_m: initial.0,
        velocity_mps: initial.1,
    };
    let mut sc_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut moon_log = MessageLog::<PlanetStateMsg>::default();
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        simulation.add_module("cislunarScene", &mut scene, step_nanos, 10);
        if record {
            connect!(&simulation,
                &origin_state => &mut sc_log.input,
                &moon_state => &mut moon_log.input,
            );
            schedule! { simulation,
                "scRec" => &mut sc_log, period=step_nanos, priority=0;
                "moonRec" => &mut moon_log, period=step_nanos, priority=0;
            }
        }
        simulation.run_for(final_time_nanos());
    }
    let moon_queries = scene
        .scene
        .task_model(spice_id)
        .module
        .planet_spice_query_count("moon")
        .0;
    Run {
        final_position_m: origin_state.read().position_m,
        moon_queries,
        times_s: sc_log.times_s(),
        positions_m: sc_log.values.iter().map(|state| state.position_m).collect(),
        moon_positions_m: moon_log
            .values
            .iter()
            .map(|planet| planet.position_inertial_m)
            .collect(),
        wall_clock_s: started.elapsed().as_secs_f64(),
    }
}

/// Upstream `runClassicSpacecraft`: the orbit with `spacecraft` and its
/// gravity effector, `spiceInterface` once per step.
fn run_classic_spacecraft(
    initial: (Vector3<f64>, Vector3<f64>),
    dt_s: f64,
    knot_step_nanos: Option<u64>,
    record: bool,
) -> Run {
    let started = Instant::now();
    let step_nanos = sec2nano(dt_s);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: 1.0,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::identity(),
        integration_step_nanos: step_nanos,
        initial_position_m: initial.0,
        initial_velocity_mps: initial.1,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("one central body");
    spacecraft
        .add_grav_body(Body::Moon.point_mass(false))
        .expect("unique gravity bodies");
    let mut spice = spice_interface(knot_step_nanos);
    let mut sc_log = MessageLog::<SpacecraftStateMsg>::default();
    let state = spacecraft.state_out.clone();
    {
        let mut simulation = Simulation::new(start_epoch(), false);
        for (index, name) in ["earth", "moon"].into_iter().enumerate() {
            simulation.connect(
                &spice.planet_state_out_msgs[index],
                spacecraft.grav_body_input_mut(name).expect("gravity body"),
            );
        }
        schedule! { simulation,
            "spiceObject" => &mut spice, period=step_nanos, priority=20;
            "cislunarSat" => &mut spacecraft, period=step_nanos, priority=10;
        }
        if record {
            connect!(&simulation, &state => &mut sc_log.input);
            simulation.add_module("scRec", &mut sc_log, step_nanos, 0);
        }
        simulation.run_for(final_time_nanos());
    }
    Run {
        final_position_m: state.read().position_m,
        moon_queries: spice.planet_spice_query_count("moon").0,
        times_s: sc_log.times_s(),
        positions_m: sc_log.values.iter().map(|state| state.position_m).collect(),
        moon_positions_m: Vec::new(),
        wall_clock_s: started.elapsed().as_secs_f64(),
    }
}

/// Upstream `moonFlybyInitialState`: the perigee state of a transfer whose
/// apogee is `APOGEE_FRACTION` of the Moon's distance at arrival, in the
/// Moon's orbit plane, the time of flight (the perigee-to-apogee half
/// period) iterated to self-consistency.
fn moon_flyby_initial_state(mu: f64, radius_equator_m: f64) -> (Vector3<f64>, Vector3<f64>) {
    let mut spice = SpiceInterface::new(SpiceInterfaceConfig {
        name: "moonState".to_string(),
        kernel_paths: KERNELS.iter().map(PathBuf::from).collect(),
        zero_base: "EARTH".to_string(),
        utc_cal_init: Some(EPOCH.to_string()),
        planet_names: vec!["moon".to_string()],
        ..SpiceInterfaceConfig::default()
    });
    spice.init();
    let mut moon_state = |tof_s: f64| {
        spice.update(&SimulationContext {
            current_sim_nanos: sec2nano(tof_s),
            current_epoch: start_epoch(),
        });
        let state = spice.planet_state_out_msgs[0].read();
        (state.position_inertial_m, state.velocity_inertial_mps)
    };
    let rp = radius_equator_m + 600e3;
    let mut tof = 120.0 * 3600.0;
    let mut a = 0.0;
    for _ in 0..8 {
        let ra = APOGEE_FRACTION * moon_state(tof).0.norm();
        a = (rp + ra) / 2.0;
        tof = std::f64::consts::PI * (a.powi(3) / mu).sqrt();
    }
    let (arrival_position, arrival_velocity) = moon_state(tof);
    let apo_hat = arrival_position.normalize();
    let h_hat = arrival_position.cross(&arrival_velocity).normalize();
    let position = -rp * apo_hat;
    let velocity = (mu * (2.0 / rp - 1.0 / a)).sqrt() * h_hat.cross(&-apo_hat);
    (position, velocity)
}

/// The study's initial state, upstream's `rN, vN`.
fn initial_state() -> (Vector3<f64>, Vector3<f64>) {
    moon_flyby_initial_state(Body::Earth.mu_m3ps2(), Body::Earth.radius_equator_m())
}

/// One point of the reconstruction sweep.
struct SweepPoint {
    multiple: u64,
    queries: i64,
    error_m: f64,
}

/// Upstream `run(show_plots)`: the study, with `errClassic`.
struct Study {
    err_classic_m: f64,
    integrator_floor_m: f64,
    exact_queries: i64,
    classic_queries: i64,
    sweep: Vec<SweepPoint>,
}

fn run_study(write_csv: bool) -> Study {
    let initial = initial_state();
    let truth = run_mj_scene(initial, RK4_DT_S, None, true);
    let classic = run_classic_spacecraft(initial, RK4_DT_S, None, true);
    let err_classic_m = (classic.final_position_m - truth.final_position_m).norm();

    // The integrator error: the study step against the fine step.
    let fine = run_mj_scene(initial, REFERENCE_DT_S, None, true);
    let floor_t: Vec<f64> = truth
        .times_s
        .iter()
        .zip(&truth.positions_m)
        .map(|(time, position)| {
            (position - interpolate(&fine.times_s, &fine.positions_m, *time)).norm()
        })
        .collect();
    let integrator_floor_m = *floor_t.last().expect("samples");

    let sweep: Vec<SweepPoint> = KNOT_MULTIPLES
        .iter()
        .map(|multiple| {
            let run = run_mj_scene(
                initial,
                RK4_DT_S,
                Some(multiple * sec2nano(RK4_DT_S)),
                false,
            );
            SweepPoint {
                multiple: *multiple,
                queries: run.moon_queries,
                error_m: (run.final_position_m - truth.final_position_m).norm(),
            }
        })
        .collect();
    println!(
        "[study] cislunar leg, RK4 dt={RK4_DT_S}s, horizon={:.1}h",
        final_time_nanos() as f64 * 1e-9 / 3600.0
    );
    println!(
        "[study] integrator error floor (exact SPICE, {RK4_DT_S}s vs {REFERENCE_DT_S}s): {integrator_floor_m:8.1} m"
    );
    println!(
        "[study] exact SPICE (per sub-step):  {:6} Moon queries  (truth)",
        truth.moon_queries
    );
    println!(
        "[study] classic sc + gravityEffector:{:6} Moon queries  err={err_classic_m:8.1} m",
        classic.moon_queries
    );
    for point in &sweep {
        println!(
            "[study] reconstruct knot={:4}xdt:  {:6} Moon queries  err={:8.1} m",
            point.multiple, point.queries, point.error_m
        );
    }

    // The cheapest grid whose final error is under the floor (else the
    // finest), and the classic path on the same grid.
    let good = sweep
        .iter()
        .filter(|point| point.error_m < integrator_floor_m)
        .max_by_key(|point| point.multiple)
        .unwrap_or(&sweep[0]);
    let good_knot = good.multiple * sec2nano(RK4_DT_S);
    let reconstructed = run_mj_scene(initial, RK4_DT_S, Some(good_knot), true);
    let classic_reconstructed = run_classic_spacecraft(initial, RK4_DT_S, Some(good_knot), false);
    let saving = |exact: f64, reconstruction: f64| 100.0 * (1.0 - reconstruction / exact);
    println!(
        "[study] runtime classic path:  exact {:.2}s -> reconstruction {:.2}s  ({:.0}% less, SPICE-only change)",
        classic.wall_clock_s,
        classic_reconstructed.wall_clock_s,
        saving(classic.wall_clock_s, classic_reconstructed.wall_clock_s)
    );
    println!(
        "[study] runtime MJScene path:  exact {:.2}s -> reconstruction {:.2}s  ({:.0}% less, 4 SPICE calls/step -> few)",
        truth.wall_clock_s,
        reconstructed.wall_clock_s,
        saving(truth.wall_clock_s, reconstructed.wall_clock_s)
    );

    if write_csv {
        let vector_headers = |prefix: &str| -> Vec<String> {
            (0..3).map(|index| format!("{prefix}_{index}")).collect()
        };
        let mut headers = vec!["time_ns".to_string()];
        headers.extend(vector_headers("r_BN_N_m"));
        headers.extend(vector_headers("moon_r_N_m"));
        let rows: Vec<Vec<f64>> = (0..truth.times_s.len())
            .map(|sample| {
                let mut row = vec![(truth.times_s[sample] * 1e9).round()];
                row.extend(truth.positions_m[sample].iter());
                row.extend(truth.moon_positions_m[sample].iter());
                row
            })
            .collect();
        table_csv::write_table("scenarioSpiceReconstruction", &headers, &rows);

        // `kind`: 0 reconstruction, 1 classic, 2 exact reference.
        let headers: Vec<String> = [
            "kind",
            "knot_multiple",
            "moon_queries",
            "error_m",
            "integrator_floor_m",
        ]
        .iter()
        .map(|name| name.to_string())
        .collect();
        let mut rows: Vec<Vec<f64>> = sweep
            .iter()
            .map(|point| {
                vec![
                    0.0,
                    point.multiple as f64,
                    point.queries as f64,
                    point.error_m,
                    integrator_floor_m,
                ]
            })
            .collect();
        rows.push(vec![
            1.0,
            1.0,
            classic.moon_queries as f64,
            err_classic_m,
            integrator_floor_m,
        ]);
        rows.push(vec![
            2.0,
            0.0,
            truth.moon_queries as f64,
            0.0,
            integrator_floor_m,
        ]);
        table_csv::write_table("scenarioSpiceReconstructionSweep", &headers, &rows);

        let headers: Vec<String> = [
            "time_ns",
            "integrator_error_m",
            "reconstruction_error_m",
            "classic_error_m",
            "reconstruction_moon_queries",
            "classic_moon_queries",
        ]
        .iter()
        .map(|name| name.to_string())
        .collect();
        let rows: Vec<Vec<f64>> = (0..truth.times_s.len())
            .map(|sample| {
                vec![
                    (truth.times_s[sample] * 1e9).round(),
                    floor_t[sample],
                    (reconstructed.positions_m[sample] - truth.positions_m[sample]).norm(),
                    (classic.positions_m[sample] - truth.positions_m[sample]).norm(),
                    reconstructed.moon_queries as f64,
                    classic.moon_queries as f64,
                ]
            })
            .collect();
        table_csv::write_table("scenarioSpiceReconstructionErrors", &headers, &rows);
    }

    Study {
        err_classic_m,
        integrator_floor_m,
        exact_queries: truth.moon_queries,
        classic_queries: classic.moon_queries,
        sweep,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_spice_reconstruction [--all]");
    // Upstream's test runs the one configuration there is.
    cli.switch("all");
    cli.finish();
    let study = run_study(true);
    println!(
        "errClassic = {:.3} m (floor {:.1} m, {} exact and {} classic Moon queries, {} sweep grids); \
         wrote examples/output/scenarios/scenarioSpiceReconstruction*.csv",
        study.err_classic_m,
        study.integrator_floor_m,
        study.exact_queries,
        study.classic_queries,
        study.sweep.len()
    );
}

#[cfg(test)]
mod tests {
    //! Ports upstream `test_scenarioSpiceReconstruction.py`: the classic
    //! `gravityEffector` path's final position is within 10 m of the
    //! exact-SPICE MuJoCo reference. Also checks the flyby geometry and that
    //! a fine reconstruction grid costs few queries for a small error.

    use super::*;

    #[test]
    fn test_scenario_spice_reconstruction() {
        let initial = initial_state();
        let truth = run_mj_scene(initial, RK4_DT_S, None, true);
        let classic = run_classic_spacecraft(initial, RK4_DT_S, None, false);
        let err_classic = (classic.final_position_m - truth.final_position_m).norm();
        assert!(err_classic < 10.0, "errClassic = {err_classic} m");

        // One exact query per RK4 stage for the reference (and one for the
        // evaluation at t = 0), one per step for the classic path.
        assert_eq!(truth.moon_queries, 4 * N_STEPS as i64 + 1);
        assert_eq!(classic.moon_queries, N_STEPS as i64 + 1);

        // A deep flyby: closest approach well inside the Moon's sphere of
        // influence (about 66 000 km), outside the Moon.
        let closest_km = truth
            .positions_m
            .iter()
            .zip(&truth.moon_positions_m)
            .map(|(sc, moon)| (sc - moon).norm() * 1e-3)
            .fold(f64::INFINITY, f64::min);
        assert!((5_000.0..40_000.0).contains(&closest_km), "{closest_km} km");

        let reconstruction = run_mj_scene(initial, RK4_DT_S, Some(64 * sec2nano(RK4_DT_S)), false);
        assert!(reconstruction.moon_queries < truth.moon_queries / 50);
        let error = (reconstruction.final_position_m - truth.final_position_m).norm();
        assert!(error < 1.0, "{error} m");
    }
}
