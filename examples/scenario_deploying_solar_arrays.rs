//! Two circular solar arrays unfolding from prescribed-motion elements: port
//! of upstream `examples/scenarioDeployingSolarArrays.py`.
//!
//! An 800 kg, 4 × 2 × 2 m hub at rest in field-free space carries two
//! arrays mounted at `±2 m` along its `x` axis. Each array is ten 5 kg
//! triangular wedges (radius 4 m, `width = 2 r cos 72°`), each a
//! [`PrescribedMotionStateEffector`] whose rotation about the mount frame's
//! `y` axis a [`PrescribedRotation1Dof`] profiles, bang-coast-bang with 2 s
//! bangs. The deployment runs in four phases, each followed by 10 s of
//! rest:
//!
//! 1. array 1's stowed stack swings out 108° in 5 minutes;
//! 2. its elements fan out about the array's center, element `i` by
//!    `36 i` degrees, in 30 minutes;
//! 3. array 2 swings out -108° in 5 minutes;
//! 4. its elements fan out by `36 i` degrees in 30 minutes.
//!
//! Before the fan-out of an array upstream re-anchors its elements at the
//! array center: their prescribed translation becomes `r_PM_M = [±r, 0, 0]`
//! and their center of mass `r_PcP_P = [0, 0, -2/3 r sin 72°]`, the same
//! points as before. Upstream's `setR_PM_M` and `setSigma_PM` on the
//! effectors and `setThetaInit` on the profilers are superseded by the
//! effectors' and profilers' next message read, upstream as here; the
//! center of mass is changed through a [`Hosted`] effector's
//! [`EffectorEdits`], the profilers' accelerations in a [`Staged`] wrapper,
//! and the reference and translation messages are rewritten between runs.
//! Everything runs every 2 s, the profilers after the spacecraft.
//!
//! Upstream's effectors read their prescribed states in their own
//! `UpdateState`, after the spacecraft integrated and before the profilers
//! run, and integrate the next step from them; basilisk-rs's read them as
//! the step starts, so a [`Latch`] per input, scheduled where upstream's
//! effector runs, restores upstream's one-step lag. It also reproduces
//! upstream's step after each re-anchoring, in which the new center of mass
//! goes with the old translation until the effectors read the new one.
//!
//! The hub state and every element's angle and rate are recorded every
//! 2 s to `examples/output/scenarios/scenarioDeployingSolarArrays.csv`.
//!
//! Upstream only runs this scenario (`test_scenarioTest.py`); `tests`
//! checks what its figures show: each element follows its profile to its
//! reference, and the hub turns against the moving elements, conserving
//! the spacecraft's angular momentum while they coast.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/latch.rs"]
mod latch;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/orbit_tools.rs"]
mod orbit_tools;
#[path = "support/staged.rs"]
mod staged;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::device_interface::prescribed_rotation_1dof::{
    PrescribedRotation1Dof, PrescribedRotation1DofConfig,
};
use basilisk_rs::dynamics::prescribed_motion_state_effector::{
    PrescribedMotionConfig, PrescribedMotionStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, Output, PrescribedRotationMsg, PrescribedTranslationMsg,
    SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use hifitime::Epoch;
use latch::Latch;
use message_log::MessageLog;
use multibody::{EffectorEdits, Hosted, box_inertia};
use nalgebra::{Matrix3, Vector3};
use orbit_tools::{run_until, sec2nano};
use staged::Staged;

/// Upstream `dynTimeStep = fswTimeStep = dataRecStep = 2.0` \[s\].
const STEP_NANOS: u64 = 2_000_000_000;
/// Upstream `num_elements`, `mass_element`, `radius_array`.
const NUM_ELEMENTS: usize = 10;
const MASS_ELEMENT: f64 = 5.0;
const RADIUS_ARRAY: f64 = 4.0;
/// Upstream `ramp_duration`, `init_deploy_duration`, `main_deploy_duration`
/// \[s\].
const RAMP_DURATION: f64 = 2.0;
const INIT_DEPLOY_DURATION: f64 = 5.0 * 60.0;
const MAIN_DEPLOY_DURATION: f64 = 30.0 * 60.0;
/// Upstream `array1ThetaInit2`: the swing-out angle of array 1 (array 2's
/// is its negative).
const SWING_OUT_DEG: f64 = 108.0;

/// Upstream `simTime1` ... `simTime4`: the four phases' run times \[s\].
const PHASE_DURATIONS_S: [f64; 4] = [
    INIT_DEPLOY_DURATION + 10.0,
    MAIN_DEPLOY_DURATION + 10.0,
    INIT_DEPLOY_DURATION + 10.0,
    MAIN_DEPLOY_DURATION + 10.0,
];

#[derive(Clone, Copy, Debug)]
struct Config {
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories, every 2 s.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    states: MessageLog<SpacecraftStateMsg>,
    diagnostics: MessageLog<SpacecraftDiagnosticsMsg>,
    /// `elements[array][i]`: upstream `array<a>Element<i>PrescribedDataLog`.
    elements: [Vec<MessageLog<HingedRigidBodyMsg>>; 2],
    /// End of each phase \[ns\].
    phase_ends_nanos: [u64; 4],
    csv_path: Option<PathBuf>,
}

/// Upstream's bang-coast-bang acceleration for a rotation of `delta_rad` in
/// a deployment of `duration_s`: `|Δθ| / (t_coast t_ramp + t_ramp^2)` with
/// `t_coast = duration - 2 t_ramp`.
fn theta_ddot_max(delta_rad: f64, duration_s: f64) -> f64 {
    let coast = duration_s - 2.0 * RAMP_DURATION;
    delta_rad.abs() / (coast * RAMP_DURATION + RAMP_DURATION * RAMP_DURATION)
}

/// `+1` for array 1 (at `+x`), `-1` for array 2.
fn side(array: usize) -> f64 {
    if array == 0 { 1.0 } else { -1.0 }
}

/// The fan-out reference of element `i` of `array` \[rad\]:
/// `36 i` degrees past the swing-out angle.
fn fan_out_reference(array: usize, element: usize) -> f64 {
    (36.0 * element as f64 + side(array) * SWING_OUT_DEG).to_radians()
}

/// Upstream's element of `array`, stowed.
fn element(array: usize, element: usize) -> PrescribedMotionStateEffector {
    let length = RADIUS_ARRAY;
    let width = 2.0 * RADIUS_ARRAY * 72.0_f64.to_radians().cos();
    let thickness = 0.01;
    let s = side(array);
    PrescribedMotionStateEffector::new(PrescribedMotionConfig {
        mass_kg: MASS_ELEMENT,
        // Upstream's `IElement_PntPc_P`: a plate of `width` along x,
        // `thickness` along y and `length` along z.
        inertia_about_com_p_kg_m2: box_inertia(MASS_ELEMENT, width, thickness, length),
        mount_position_body_m: Vector3::new(s * 2.0, 0.0, 0.0),
        // The wedge's centroid, for the stowed stack hinged at the mount.
        center_of_mass_p_m: Vector3::new(
            -s * RADIUS_ARRAY * 72.0_f64.to_radians().cos(),
            0.0,
            RADIUS_ARRAY / 3.0 * 72.0_f64.to_radians().sin(),
        ),
        ..PrescribedMotionConfig::new(format!("array{}Element{}", array + 1, element + 1))
    })
}

/// Upstream's element centroid after re-anchoring at the array center.
fn anchored_center_of_mass() -> Vector3<f64> {
    Vector3::new(
        0.0,
        0.0,
        -2.0 / 3.0 * RADIUS_ARRAY * 72.0_f64.to_radians().sin(),
    )
}

fn run(config: Config) -> Results {
    let (length_hub, width_hub, depth_hub) = (4.0, 2.0, 2.0);
    let mass_hub = 800.0;
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: mass_hub,
        hub_center_of_mass_body_m: Vector3::zeros(),
        // Upstream's `IHub_11 = m/12 (l^2 + d^2)`, `IHub_22 = m/12 (l^2 + w^2)`,
        // `IHub_33 = m/12 (w^2 + d^2)`.
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            mass_hub / 12.0 * (length_hub * length_hub + depth_hub * depth_hub),
            mass_hub / 12.0 * (length_hub * length_hub + width_hub * width_hub),
            mass_hub / 12.0 * (width_hub * width_hub + depth_hub * depth_hub),
        )),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });

    // Upstream's stand-alone messages: one translation per array, one
    // reference angle per element.
    let translations: [Output<PrescribedTranslationMsg>; 2] = Default::default();
    let references: [Vec<Output<HingedRigidBodyMsg>>; 2] = [0, 1].map(|array| {
        (0..NUM_ELEMENTS)
            .map(|_| {
                Output::new(HingedRigidBodyMsg {
                    // Array 1 swings out first; array 2 holds at zero.
                    theta_rad: if array == 0 {
                        SWING_OUT_DEG.to_radians()
                    } else {
                        0.0
                    },
                    theta_dot_radps: 0.0,
                })
            })
            .collect()
    });

    let mut effectors = Vec::new();
    let mut edits: [Vec<EffectorEdits<PrescribedMotionStateEffector>>; 2] = Default::default();
    let mut profilers: Vec<Staged<PrescribedRotation1Dof>> = Vec::new();
    let mut rotation_latches: Vec<Latch<PrescribedRotationMsg>> = Vec::new();
    let mut translation_latches: Vec<Latch<PrescribedTranslationMsg>> = Vec::new();
    let mut element_logs: [Vec<MessageLog<HingedRigidBodyMsg>>; 2] = Default::default();
    let mut phase_ends_nanos = [0; 4];
    let mut stop_nanos = 0;
    for (end, duration) in phase_ends_nanos.iter_mut().zip(PHASE_DURATIONS_S) {
        stop_nanos += sec2nano(duration);
        *end = stop_nanos;
    }

    for array in 0..2 {
        for index in 0..NUM_ELEMENTS {
            let effector = Hosted::new(element(array, index));
            edits[array].push(effector.edits());
            effectors.push(effector);
            let swing_out = side(array) * SWING_OUT_DEG.to_radians();
            let fan_out = fan_out_reference(array, index);
            let mut profiler =
                Staged::new(PrescribedRotation1Dof::new(PrescribedRotation1DofConfig {
                    name: format!(
                        "prescribedRotation1DOFArray{}Element{}",
                        array + 1,
                        index + 1
                    ),
                    coast_option_bang_duration_s: RAMP_DURATION,
                    rot_hat_m: Vector3::y(),
                    theta_ddot_max_radps2: if array == 0 {
                        theta_ddot_max(swing_out, INIT_DEPLOY_DURATION)
                    } else {
                        0.0
                    },
                    theta_init_rad: 0.0,
                    ..PrescribedRotation1DofConfig::default()
                }));
            // The accelerations of the later phases (upstream
            // `setThetaDDotMax` and `setThetaInit` between runs).
            if array == 0 {
                let accel = theta_ddot_max(fan_out - swing_out, MAIN_DEPLOY_DURATION);
                profiler = profiler.after(
                    phase_ends_nanos[0],
                    move |module: &mut PrescribedRotation1Dof, _| {
                        module.config.theta_init_rad = swing_out;
                        module.config.theta_ddot_max_radps2 = accel;
                    },
                );
            } else {
                let swing_accel = theta_ddot_max(swing_out, INIT_DEPLOY_DURATION);
                let fan_accel = theta_ddot_max(fan_out - swing_out, MAIN_DEPLOY_DURATION);
                profiler = profiler
                    .after(
                        phase_ends_nanos[1],
                        move |module: &mut PrescribedRotation1Dof, _| {
                            module.config.theta_ddot_max_radps2 = swing_accel;
                        },
                    )
                    .after(
                        phase_ends_nanos[2],
                        move |module: &mut PrescribedRotation1Dof, _| {
                            module.config.theta_init_rad = swing_out;
                            module.config.theta_ddot_max_radps2 = fan_accel;
                        },
                    );
            }
            profilers.push(profiler);
            rotation_latches.push(Latch::default());
            translation_latches.push(Latch::default());
            element_logs[array].push(MessageLog::default());
        }
    }

    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let (mut recorder, csv_path) =
        scenario_recorder("scenarioDeployingSolarArrays", config.write_csv);

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    let element_ports = effectors
        .iter_mut()
        .zip(profilers.iter_mut())
        .zip(
            rotation_latches
                .iter_mut()
                .zip(translation_latches.iter_mut()),
        )
        .zip(element_logs.iter_mut().flatten())
        .enumerate();
    for (flat, (((effector, profiler), (rotation_latch, translation_latch)), log)) in element_ports
    {
        let (array, index) = (flat / NUM_ELEMENTS, flat % NUM_ELEMENTS);
        connect!(&simulation,
            &references[array][index] => &mut profiler.module.spinning_body_in_msg,
            &profiler.module.prescribed_rotation_out_msg => &mut rotation_latch.input,
            &rotation_latch.output => &mut effector.inner.prescribed_rotation_in,
            &translations[array] => &mut translation_latch.input,
            &translation_latch.output => &mut effector.inner.prescribed_translation_in,
            &profiler.module.spinning_body_out_msg => &mut log.input,
            &profiler.module.spinning_body_out_msg => recorder.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([
                    ("theta_rad".to_string(), format!("array{}Element{}_theta_rad", array + 1, index + 1)),
                    (
                        "theta_dot_radps".to_string(),
                        format!("array{}Element{}_thetaDot_rad_per_s", array + 1, index + 1),
                    ),
                ]),
            ),
        );
    }
    connect!(&simulation,
        &spacecraft.state_out => &mut state_log.input,
        &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("position_m", "r_BN_N_m")
                .into_iter()
                .chain(vector_columns("sigma_bn", "sigma_BN"))
                .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
        )),
    );
    for effector in effectors {
        spacecraft.add_state_effector(effector);
    }

    schedule! { simulation,
        "spacecraftBody" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "scStateData" => &mut state_log, period=STEP_NANOS, priority=0;
        "diagnostics_log" => &mut diagnostics_log, period=STEP_NANOS, priority=0;
    }
    // Upstream's dynamics task holds the effectors (their message reads),
    // the flight-software task after it the profilers.
    for latch in &mut rotation_latches {
        simulation.add_module("element_rotation_read", latch, STEP_NANOS, 95);
    }
    for latch in &mut translation_latches {
        simulation.add_module("element_translation_read", latch, STEP_NANOS, 95);
    }
    for profiler in &mut profilers {
        simulation.add_module("prescribedRotation1DOF", profiler, STEP_NANOS, 90);
    }
    for log in element_logs.iter_mut().flatten() {
        simulation.add_module("PrescribedDataLog", log, STEP_NANOS, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, STEP_NANOS, 0);
    }

    // Phase 1: array 1 swings out.
    run_until(&mut simulation, phase_ends_nanos[0]);
    // Phase 2: array 1 re-anchored at its center, its elements fan out.
    re_anchor(&translations[0], &edits[0], 0);
    for (index, reference) in references[0].iter().enumerate() {
        reference.write(HingedRigidBodyMsg {
            theta_rad: fan_out_reference(0, index),
            theta_dot_radps: 0.0,
        });
    }
    run_until(&mut simulation, phase_ends_nanos[1]);
    // Phase 3: array 2 swings out.
    for reference in &references[1] {
        reference.write(HingedRigidBodyMsg {
            theta_rad: -SWING_OUT_DEG.to_radians(),
            theta_dot_radps: 0.0,
        });
    }
    run_until(&mut simulation, phase_ends_nanos[2]);
    // Phase 4: array 2 re-anchored, its elements fan out.
    re_anchor(&translations[1], &edits[1], 1);
    for (index, reference) in references[1].iter().enumerate() {
        reference.write(HingedRigidBodyMsg {
            theta_rad: fan_out_reference(1, index),
            theta_dot_radps: 0.0,
        });
    }
    run_until(&mut simulation, phase_ends_nanos[3]);
    drop(simulation);
    drop(recorder);

    Results {
        states: state_log,
        diagnostics: diagnostics_log,
        elements: element_logs,
        phase_ends_nanos,
        csv_path,
    }
}

/// Upstream's re-anchoring of `array`'s elements at the array center:
/// a new translation message `r_PM_M = [±r, 0, 0]` and `setR_PcP_P`.
fn re_anchor(
    translation: &Output<PrescribedTranslationMsg>,
    edits: &[EffectorEdits<PrescribedMotionStateEffector>],
    array: usize,
) {
    translation.write(PrescribedTranslationMsg {
        r_pm_m_m: Vector3::new(side(array) * RADIUS_ARRAY, 0.0, 0.0),
        r_prime_pm_m_mps: Vector3::zeros(),
        r_prime_prime_pm_m_mps2: Vector3::zeros(),
    });
    for edit in edits {
        edit.edit(|effector| effector.config.center_of_mass_p_m = anchored_center_of_mass());
    }
}

fn main() {
    Cli::from_env("scenario_deploying_solar_arrays").finish();
    let results = run(Config { write_csv: true });
    println!(
        "wrote {} (final hub rate {:.2e} deg/s)",
        results.csv_path.expect("CSV written").display(),
        results.states.last().omega_radps.norm().to_degrees(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every element holds its reference at the end of each phase: array 1
    /// at 108° then fanned out by `36 i` degrees, array 2 stowed until the
    /// third phase, then at -108° and fanned out.
    #[test]
    fn the_elements_reach_their_references() {
        let results = run(Config { write_csv: false });
        let ends = results.phase_ends_nanos;
        let swing = SWING_OUT_DEG.to_radians();
        for index in 0..NUM_ELEMENTS {
            let [one, two] = [&results.elements[0][index], &results.elements[1][index]];
            let expected = [
                (one, ends[0], swing),
                (one, ends[1], fan_out_reference(0, index)),
                (one, ends[3], fan_out_reference(0, index)),
                (two, ends[0], 0.0),
                (two, ends[1], 0.0),
                (two, ends[2], -swing),
                (two, ends[3], fan_out_reference(1, index)),
            ];
            for (log, time, theta) in expected {
                let sample = log.at(time);
                assert!(
                    (sample.theta_rad - theta).abs() < 1.0e-9,
                    "element {index} at {time}: {sample:?}"
                );
                assert!(sample.theta_dot_radps.abs() < 1.0e-12);
            }
        }
        // Array 1's fan-out takes the full 30 minutes for the last element.
        let last = &results.elements[0][NUM_ELEMENTS - 1];
        let midway = ends[0] + sec2nano(MAIN_DEPLOY_DURATION / 2.0);
        assert!(
            (last.at(midway).theta_rad
                - 0.5 * (SWING_OUT_DEG + 36.0 * 9.0 + SWING_OUT_DEG).to_radians())
            .abs()
                < 1.0e-2
        );
    }

    /// The prescribed motion is internal, so while every element coasts at
    /// a constant rate the spacecraft's angular momentum is conserved, and
    /// the hub turns against the moving elements. The momentum only changes
    /// across the 2 s bangs, where the prescribed rate restarts from each
    /// profiler message (upstream's effector does the same), so the hub
    /// keeps a small rate after each phase.
    #[test]
    fn the_hub_reacts_and_momentum_holds_while_coasting() {
        let results = run(Config { write_csv: false });
        let ends = results.phase_ends_nanos;
        let starts = [0, ends[0], ends[1], ends[2]];
        let mut peak_rate = 0.0_f64;
        for (start, end) in starts.into_iter().zip(ends) {
            // Each coast: from shortly after the first bang to shortly
            // before the last one (the profiles start one step into the
            // phase and end 10 s before it does).
            let coast = |time: &u64| *time > start + sec2nano(10.0) && *time + sec2nano(20.0) < end;
            let momenta: Vec<Vector3<f64>> = results
                .diagnostics
                .times_nanos
                .iter()
                .zip(&results.diagnostics.values)
                .filter(|(time, _)| coast(time))
                .map(|(_, diagnostics)| diagnostics.rotational_angular_momentum_inertial_kg_m2ps)
                .collect();
            for momentum in &momenta {
                let drift = (momentum - momenta[0]).norm();
                assert!(
                    drift < 1.0e-9 * momenta[0].norm().max(1.0),
                    "drift {drift} in the phase ending at {end}"
                );
            }
            let phase_peak = results
                .states
                .times_nanos
                .iter()
                .zip(&results.states.values)
                .filter(|(time, _)| coast(time))
                .fold(0.0_f64, |max, (_, state)| max.max(state.omega_radps.norm()));
            assert!(
                phase_peak > 1.0e-5,
                "peak hub rate {phase_peak} in the phase ending at {end}"
            );
            peak_rate = peak_rate.max(phase_peak);
        }
        // The elements turn about the mounts' y axes, so does the hub.
        for state in &results.states.values {
            assert!(state.omega_radps.x.abs() < 1.0e-12 && state.omega_radps.z.abs() < 1.0e-12);
        }
        assert!(results.states.last().sigma_bn.y.abs() > 1.0e-2);
        assert!(peak_rate < 1.0e-2);
    }
}
