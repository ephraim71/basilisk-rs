//! A telescoping boom extending from a tumbling spacecraft: port of
//! upstream `examples/scenarioExtendingBoom.py`.
//!
//! A 400 kg, 3 × 3 × 6 m hub tumbling at `[0.1, 0.1, 0.1] rad/s` on an
//! 8000 km, `e = 0.1` Earth orbit (with the Sun's gravity) carries a
//! [`LinearTranslationNDofStateEffector`] of four 100 kg, 3 m cylinders
//! nested along the hub's `y` axis, each on a `k = 100 N/m`,
//! `c = 400 N s/m` joint. The second, third and fourth segments each track
//! a [`PrescribedLinearTranslation`] reference that extends them 3 m from
//! their parent (bang-bang at 0.0005 m/s^2, 10 s smoothing); the first
//! stays retracted on its spring. RK4, 0.1 s steps, 3 minutes.
//!
//! Upstream's `createSpiceInterface` (zero base Earth, epoch
//! 2016 JUNE 3 01:34:30) is [`AniseSunEphemeris`], the Sun relative to Earth
//! from DE440s (`just fetch-anise-assets`); the Earth stays at the origin.
//! Upstream's effector reads its reference inputs in its own `UpdateState`,
//! after the spacecraft integrated and before the profilers of the same
//! task run, for the next step; basilisk-rs's reads them as the step
//! starts, so a [`Latch`] per reference, scheduled where upstream's
//! effector runs, restores upstream's timing.
//!
//! The hub attitude and rate and every segment's displacement and rate are
//! recorded every step to
//! `examples/output/scenarios/scenarioExtendingBoom.csv`.
//!
//! Upstream only runs this scenario (`test_scenarioTest.py`); `tests`
//! checks what the figures show: the driven segments extend to their 3 m
//! references, and the tumble slows as the boom's inertia grows, conserving
//! the angular momentum.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::device_interface::prescribed_linear_translation::{
    PrescribedLinearTranslation, PrescribedLinearTranslationConfig,
};
use basilisk_rs::dynamics::linear_translation_n_dof_state_effector::{
    LinearTranslationNDofStateEffector, TranslatingBody,
};
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::messages::{
    LinearTranslationRigidBodyMsg, Output, SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::{Body, PLANETARY_SPK};
use hifitime::Epoch;
use latch::Latch;
use message_log::MessageLog;
use multibody::{box_inertia, set_center_of_mass_initial_state};
use nalgebra::{Matrix3, Vector3};

/// Upstream `simulationTimeStep = macros.sec2nano(.1)`.
const STEP_NANOS: u64 = 100_000_000;
/// Upstream `simulationTime = macros.min2nano(3.0)`.
const DURATION_NANOS: u64 = 180_000_000_000;
/// Upstream `geometryClass`.
const MASS_HUB: f64 = 400.0;
const LENGTH_HUB: f64 = 3.0;
const WIDTH_HUB: f64 = 3.0;
const HEIGHT_HUB: f64 = 6.0;
const HEIGHT_ARM: f64 = 3.0;
const DIAMETER_ARM: f64 = 0.6;
/// Upstream `translatingBody.setMass(100)`.
const SEGMENT_MASS_KG: f64 = 100.0;

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories, every step.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: MessageLog<SpacecraftStateMsg>,
    segments: Vec<MessageLog<LinearTranslationRigidBodyMsg>>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream's `translatingBody<i>`: a 100 kg, 3 m cylinder sliding along
/// its parent's `y` axis, its center of mass mid-length, the first one at
/// the hub's `+y` face.
fn segment(first: bool) -> TranslatingBody {
    let radius = DIAMETER_ARM / 2.0;
    let transverse = SEGMENT_MASS_KG / 12.0 * (3.0 * radius * radius + HEIGHT_ARM * HEIGHT_ARM);
    TranslatingBody {
        mass_kg: SEGMENT_MASS_KG,
        spring_constant_n_per_m: 100.0,
        damping_n_s_per_m: 400.0,
        rho_init_m: 0.0,
        rho_dot_init_mps: 0.0,
        axis_parent: Vector3::y(),
        center_of_mass_translating_m: Vector3::new(0.0, HEIGHT_ARM / 2.0, 0.0),
        origin0_parent_m: if first {
            Vector3::new(0.0, LENGTH_HUB / 2.0, 0.0)
        } else {
            Vector3::zeros()
        },
        // Upstream's `IPntFc_F`, its axial term `m r^2 / 12` as given.
        inertia_about_com_translating_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            transverse,
            SEGMENT_MASS_KG / 12.0 * radius * radius,
            transverse,
        )),
        parent_to_translating_dcm: Matrix3::identity(),
    }
}

fn run(config: Config) -> Results {
    let earth = Body::Earth;
    let (r_cn_n, v_cn_n) = elements_to_state(
        earth.mu_m3ps2(),
        &ClassicElements {
            semi_major_axis: 8.0e6,
            eccentricity: 0.1,
            ..Default::default()
        },
    );
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: MASS_HUB,
        hub_center_of_mass_body_m: Vector3::zeros(),
        // Upstream's hub box: `lengthHub` with x, `widthHub` with y.
        inertia_kg_m2: box_inertia(MASS_HUB, WIDTH_HUB, LENGTH_HUB, HEIGHT_HUB),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.1, 0.1, 0.1),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("unique central gravity body");
    spacecraft
        .add_grav_body(Body::Sun.point_mass(false))
        .expect("unique gravity body");
    let mut sun_ephemeris = AniseSunEphemeris::new(AniseSunEphemerisConfig {
        name: "spiceObject".to_string(),
        spk_path: PLANETARY_SPK.into(),
    });

    let mut boom = LinearTranslationNDofStateEffector::new("translatingBodyEffector");
    for index in 0..4 {
        boom.add_translating_body(segment(index == 0));
    }
    // Upstream `LinearTranslationRigidBodyMsgPayload(rho=heightArm)`.
    let reference = Output::new(LinearTranslationRigidBodyMsg {
        rho_m: HEIGHT_ARM,
        rho_dot_mps: 0.0,
    });
    let mut profilers: Vec<PrescribedLinearTranslation> = (0..3)
        .map(|_| {
            PrescribedLinearTranslation::new(PrescribedLinearTranslationConfig {
                name: "profiler".to_string(),
                trans_accel_max_mps2: 0.0005,
                trans_pos_init_m: 0.0,
                smoothing_duration_s: 10.0,
                ..PrescribedLinearTranslationConfig::default()
            })
        })
        .collect();
    let mut reference_latches: Vec<Latch<LinearTranslationRigidBodyMsg>> =
        (0..3).map(|_| Latch::default()).collect();

    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut segment_logs: Vec<MessageLog<LinearTranslationRigidBodyMsg>> =
        (0..4).map(|_| MessageLog::default()).collect();
    let (mut recorder, csv_path) = scenario_recorder("scenarioExtendingBoom", config.write_csv);

    // Upstream `timeInitString = "2016 JUNE 3 01:34:30.0"`.
    let epoch = Epoch::from_gregorian_utc(2016, 6, 3, 1, 34, 30, 0);
    let mut simulation = Simulation::new(epoch, false);
    for ((profiler, latch), body_index) in profilers
        .iter_mut()
        .zip(reference_latches.iter_mut())
        .zip(1..)
    {
        connect!(&simulation,
            &reference => &mut profiler.linear_translation_rigid_body_in_msg,
            &profiler.linear_translation_rigid_body_out_msg => &mut latch.input,
            &latch.output => &mut boom.translating_body_ref_in[body_index],
        );
    }
    for (index, (output, log)) in boom
        .translating_body_out
        .iter()
        .zip(segment_logs.iter_mut())
        .enumerate()
    {
        connect!(&simulation,
            output => &mut log.input,
            output => recorder.add_source::<LinearTranslationRigidBodyMsg>(CsvSourceConfig::columns([
                ("rho_m".to_string(), format!("rho{}_m", index + 1)),
                ("rho_dot_mps".to_string(), format!("rho{}Dot_m_per_s", index + 1)),
            ])),
        );
    }
    connect!(&simulation,
        &sun_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("sun").expect("sun"),
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("sigma_bn", "sigma_BN")
                .into_iter()
                .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
        )),
    );
    spacecraft.add_state_effector(boom);
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    schedule! { simulation,
        "spiceObject" => &mut sun_ephemeris, period=STEP_NANOS, priority=110;
        "scObject" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "state_log" => &mut state_log, period=STEP_NANOS, priority=0;
        "diagnostics_log" => &mut diagnostics_log, period=STEP_NANOS, priority=0;
    }
    // Upstream task order: scObject, the effector (its reference reads),
    // spiceObject, profiler2, profiler3, profiler4.
    for latch in &mut reference_latches {
        simulation.add_module("translatingBodyEffector_inputs", latch, STEP_NANOS, 95);
    }
    for profiler in &mut profilers {
        simulation.add_module("profiler", profiler, STEP_NANOS, 90);
    }
    for log in &mut segment_logs {
        simulation.add_module("rhoLog", log, STEP_NANOS, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, STEP_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        states: state_log,
        segments: segment_logs,
        diagnostics: diagnostics_log,
        csv_path,
    }
}

fn main() {
    Cli::from_env("scenario_extending_boom").finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final displacements {:?} m, |omega| {:.4} rad/s)",
        results.csv_path.expect("CSV written").display(),
        results
            .segments
            .iter()
            .map(|log| (log.last().rho_m * 1000.0).round() / 1000.0)
            .collect::<Vec<_>>(),
        results.states.last().omega_radps.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The three driven segments extend to about their 3 m references
    /// (the centrifugal load of the tumble stretching their springs a little
    /// further), while the undriven first one only sags outward on its
    /// spring.
    #[test]
    fn the_driven_segments_extend() {
        let results = run(Config { write_csv: false });
        for log in &results.segments {
            assert_eq!(log.values[0].rho_m, 0.0);
        }
        let first = results.segments[0].last();
        assert!(first.rho_m > 0.0 && first.rho_m < 0.05, "{first:?}");
        for log in &results.segments[1..] {
            let last = log.last();
            assert!(
                last.rho_m > HEIGHT_ARM && last.rho_m < HEIGHT_ARM + 0.05,
                "{last:?}"
            );
            assert!(last.rho_dot_mps.abs() < 0.05, "{last:?}");
        }
    }

    /// No external torque about the center of mass: the angular momentum
    /// is conserved while the extension multiplies the inertia and slows
    /// the tumble.
    #[test]
    fn the_tumble_slows_as_the_boom_extends() {
        let results = run(Config { write_csv: false });
        let diagnostics = &results.diagnostics.values;
        let momentum = diagnostics[0].rotational_angular_momentum_inertial_kg_m2ps;
        for sample in diagnostics {
            let drift = sample.rotational_angular_momentum_inertial_kg_m2ps - momentum;
            assert!(drift.norm() < 1.0e-6 * momentum.norm(), "drift {drift}");
        }
        let initial_rate = results.states.values[0].omega_radps.norm();
        let final_rate = results.states.last().omega_radps.norm();
        assert!(
            final_rate < 0.5 * initial_rate,
            "{initial_rate} -> {final_rate} rad/s"
        );
    }
}
