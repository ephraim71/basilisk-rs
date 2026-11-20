//! Port of upstream `examples/scenarioRotTransArrayDeployment.py`: two solar
//! arrays, each of four 50 kg elements, unfold from an 800 kg hub. Every
//! element is a
//! [`PrescribedMotionStateEffector`](basilisk_rs::dynamics::prescribed_motion_state_effector::PrescribedMotionStateEffector)
//! mounted on the hub's ±x̂ face, driven by its own rotation and translation
//! profilers
//! ([`PrescribedRotation1Dof`](basilisk_rs::device_interface::prescribed_rotation_1dof::PrescribedRotation1Dof),
//! [`PrescribedLinearTranslation`](basilisk_rs::device_interface::prescribed_linear_translation::PrescribedLinearTranslation),
//! bang-bang). First the stowed stacks swing out about ŷ, array 1 by −90° in
//! 30 s and array 2 by 90° in 37.5 s; then, in three extension steps, the
//! outer elements slide out by one element length plus a 0.1 m gap each time,
//! array 2 taking 25 % longer for each motion, which makes the hub respond.
//! The dynamics and the profilers run at 0.05 s; the run lasts 83.15 s.
//!
//! `scenarioRotTransArrayDeployment.csv` holds, every 2 s (upstream's
//! `data_rec_step`), the hub state (`r_BN_N_m_i`, `sigma_BN_i`,
//! `omega_BN_B_rad_per_s_i`) and each element's profiled angle
//! (`theta_array<a>Element<k>_rad`) and displacement
//! (`rho_array<a>Element<k>_m`). `plot_scenario_rot_trans_array_deployment.py`
//! draws upstream's five figures. Upstream's Vizard setup is not ported.
//!
//! Upstream re-subscribes the translation profilers to new reference
//! messages between runs; here each profiler's reference is an output the
//! scenario rewrites then.
//!
//! Upstream checks this scenario only by running it (`test_scenarioTest.py`).
//! `tests` checks what the figures show: every element ends at its array's
//! angle and its extension, the hub turns and drifts in response to the
//! unequal deployment rates, and the total angular momentum is conserved
//! through the extensions.

#[path = "support/common.rs"]
mod common;
#[path = "support/hub_init.rs"]
mod hub_init;
#[path = "support/prescribed.rs"]
mod prescribed;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::device_interface::prescribed_linear_translation::PrescribedLinearTranslation;
use basilisk_rs::device_interface::prescribed_rotation_1dof::PrescribedRotation1Dof;
use basilisk_rs::dynamics::prescribed_motion_state_effector::{
    PrescribedMotionConfig, PrescribedMotionStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, LinearTranslationRigidBodyMsg, Output, SpacecraftDiagnosticsMsg,
    SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::prv_to_mrp;
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use hifitime::Epoch;
use hub_init::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use prescribed::{box_inertia_kg_m2, hub_at_rest, rotation_profiler, translation_profiler};
use recording::{History, scenario_csv};

const STEM: &str = "scenarioRotTransArrayDeployment";
/// Upstream `dyn_time_step = fsw_time_step = 0.05` s.
const STEP_NANOS: u64 = 50_000_000;
/// Upstream `data_rec_step = 2.0` s.
const DATA_STEP_NANOS: u64 = 2_000_000_000;
/// Upstream `num_elements`.
pub const ELEMENTS: usize = 4;
/// Upstream `length_element` \[m\].
const ELEMENT_LENGTH_M: f64 = 1.5;
/// Upstream `gap` \[m\]: between deployed elements.
const GAP_M: f64 = 0.1;
/// Upstream `array_1_theta_ref`, `array_2_theta_ref` \[rad\].
pub const THETA_REFS_RAD: [f64; 2] = [-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2];
/// Upstream `array1_rot_duration`, `array2_rot_duration` \[s\].
const ROTATION_DURATIONS_S: [f64; 2] = [30.0, 37.5];
/// Upstream `array1_translation_duration`, `array2_translation_duration` \[s\].
const TRANSLATION_DURATIONS_S: [f64; 2] = [10.0, 12.5];

/// Each array's elements, profilers and references.
struct Array {
    elements: Vec<PrescribedMotionStateEffector>,
    rotation_profilers: Vec<PrescribedRotation1Dof>,
    translation_profilers: Vec<PrescribedLinearTranslation>,
    translation_refs: Vec<Output<LinearTranslationRigidBodyMsg>>,
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub diagnostics: Vec<SpacecraftDiagnosticsMsg>,
    /// Upstream `array<a>_prescribed_data_log[k].theta`, by array.
    pub angles: [Vec<Vec<HingedRigidBodyMsg>>; 2],
    /// Upstream `array<a>_trans_data_log_list[k].rho`, by array.
    pub displacements: [Vec<Vec<LinearTranslationRigidBodyMsg>>; 2],
    pub stop_nanos: u64,
}

/// Upstream's deployed displacement of `element` \[m\], signed by array:
/// the outer elements move out once per extension step they take part in.
pub fn deployed_rho_m(array: usize, element: usize) -> f64 {
    let sign = if array == 0 { 1.0 } else { -1.0 };
    sign * (ELEMENTS - 1 - element) as f64 * (ELEMENT_LENGTH_M + GAP_M)
}

fn array(index: usize) -> Array {
    let width_hub = 1.5;
    let (width_element, thickness_element) = (0.75_f64, 0.01_f64);
    let mass_element = 50.0;
    let element_inertia = Matrix3::from_diagonal(&Vector3::new(
        mass_element * (width_element.powi(2) + ELEMENT_LENGTH_M.powi(2)) / 12.0,
        mass_element * (ELEMENT_LENGTH_M.powi(2) + thickness_element.powi(2)) / 12.0,
        mass_element * (thickness_element.powi(2) + width_element.powi(2)) / 12.0,
    ));
    let sign = if index == 0 { 1.0 } else { -1.0 };
    let rot_hat_m = Vector3::y();
    let theta_init_rad = 0.0;
    let rot_accel_max =
        4.0 * (THETA_REFS_RAD[index] - theta_init_rad).abs() / ROTATION_DURATIONS_S[index].powi(2);
    let target_rho_m = ELEMENT_LENGTH_M + GAP_M;
    let trans_accel_max = 4.0 * target_rho_m / TRANSLATION_DURATIONS_S[index].powi(2);
    let rotation_ref = Output::new(HingedRigidBodyMsg {
        theta_rad: THETA_REFS_RAD[index],
        theta_dot_radps: 0.0,
    });
    let mut array = Array {
        elements: Vec::new(),
        rotation_profilers: Vec::new(),
        translation_profilers: Vec::new(),
        translation_refs: Vec::new(),
    };
    for element in 0..ELEMENTS {
        let label = format!("array{}Element{}", index + 1, element + 1);
        let mut body = PrescribedMotionStateEffector::new(PrescribedMotionConfig {
            mass_kg: mass_element,
            inertia_about_com_p_kg_m2: element_inertia,
            mount_position_body_m: Vector3::new(sign * width_hub / 2.0, 0.0, 0.0),
            center_of_mass_p_m: Vector3::new(0.0, 0.0, -ELEMENT_LENGTH_M / 2.0),
            sigma_pm: prv_to_mrp(theta_init_rad * rot_hat_m),
            ..PrescribedMotionConfig::new(label.clone())
        });
        let mut rotation = rotation_profiler(
            &format!(
                "prescribedRotation1DOFArray{}Element{}",
                index + 1,
                element + 1
            ),
            rot_hat_m,
            rot_accel_max,
            theta_init_rad,
            0.0,
            0.0,
        );
        let mut translation = translation_profiler(
            &format!("{label}Translation"),
            Vector3::x(),
            trans_accel_max,
            0.0,
            0.0,
            0.0,
        );
        // No movement for the first run.
        let translation_ref = Output::new(LinearTranslationRigidBodyMsg::default());
        rotation_ref.connect_to(&mut rotation.spinning_body_in_msg);
        rotation
            .prescribed_rotation_out_msg
            .connect_to(&mut body.prescribed_rotation_in);
        translation_ref.connect_to(&mut translation.linear_translation_rigid_body_in_msg);
        translation
            .prescribed_translation_out_msg
            .connect_to(&mut body.prescribed_translation_in);
        array.elements.push(body);
        array.rotation_profilers.push(rotation);
        array.translation_profilers.push(translation);
        array.translation_refs.push(translation_ref);
    }
    array
}

pub fn run() -> Results {
    let mass_hub = 800.0;
    let mut spacecraft = hub_at_rest(
        mass_hub,
        box_inertia_kg_m2(mass_hub, 4.0, 1.5, 1.5),
        STEP_NANOS,
    );
    let mut arrays = [array(0), array(1)];
    // Upstream adds the elements alternating between the arrays.
    let elements_1 = std::mem::take(&mut arrays[0].elements);
    let elements_2 = std::mem::take(&mut arrays[1].elements);
    for (element_1, element_2) in elements_1.into_iter().zip(elements_2) {
        spacecraft.add_state_effector(element_1);
        spacecraft.add_state_effector(element_2);
    }
    set_center_of_mass_initial_state(&mut spacecraft, Vector3::zeros(), Vector3::zeros());

    let mut csv = scenario_csv(STEM);
    let csv_path = csv.config.output_path.clone();
    let mut state_log = History::<SpacecraftStateMsg>::new();
    let mut diagnostics_log = History::<SpacecraftDiagnosticsMsg>::new();
    let mut angle_logs: Vec<History<HingedRigidBodyMsg>> = Vec::new();
    let mut rho_logs: Vec<History<LinearTranslationRigidBodyMsg>> = Vec::new();
    let stop_nanos;
    {
        let mut simulation =
            Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
        connect!(&simulation,
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
                vector_columns("position_m", "r_BN_N_m")
                    .into_iter()
                    .chain(vector_columns("sigma_bn", "sigma_BN"))
                    .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
            )),
            &spacecraft.state_out => &mut state_log.input,
            &spacecraft.diagnostics_out => &mut diagnostics_log.input,
        );
        for (index, array) in arrays.iter().enumerate() {
            for element in 0..ELEMENTS {
                let label = format!("array{}Element{}", index + 1, element + 1);
                let mut angle_log = History::new();
                let mut rho_log = History::new();
                connect!(&simulation,
                    &array.rotation_profilers[element].spinning_body_out_msg => csv
                        .add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([(
                            "theta_rad".to_string(),
                            format!("theta_{label}_rad"),
                        )])),
                    &array.translation_profilers[element].linear_translation_rigid_body_out_msg => csv
                        .add_source::<LinearTranslationRigidBodyMsg>(CsvSourceConfig::columns([(
                            "rho_m".to_string(),
                            format!("rho_{label}_m"),
                        )])),
                    &array.rotation_profilers[element].spinning_body_out_msg => &mut angle_log.input,
                    &array.translation_profilers[element].linear_translation_rigid_body_out_msg => &mut rho_log.input,
                );
                angle_logs.push(angle_log);
                rho_logs.push(rho_log);
            }
        }
        schedule! { simulation,
            "spacecraftBody" => &mut spacecraft, period=STEP_NANOS, priority=100;
        }
        let [array_1, array_2] = &mut arrays;
        // Upstream's FSW task order: the elements of both arrays in turn,
        // the rotation profilers, then the translation profilers.
        for (rotation_1, rotation_2) in array_1
            .rotation_profilers
            .iter_mut()
            .zip(array_2.rotation_profilers.iter_mut())
        {
            for profiler in [rotation_1, rotation_2] {
                let name = profiler.config.name.clone();
                simulation.add_module(name, profiler, STEP_NANOS, 50);
            }
        }
        for (translation_1, translation_2) in array_1
            .translation_profilers
            .iter_mut()
            .zip(array_2.translation_profilers.iter_mut())
        {
            for profiler in [translation_1, translation_2] {
                let name = profiler.config.name.clone();
                simulation.add_module(name, profiler, STEP_NANOS, 49);
            }
        }
        schedule! { simulation,
            "csv" => &mut csv, period=DATA_STEP_NANOS, priority=0;
            "scStateLog" => &mut state_log, period=DATA_STEP_NANOS, priority=0;
            "diagnosticsLog" => &mut diagnostics_log, period=STEP_NANOS, priority=0;
        }
        for log in &mut angle_logs {
            simulation.add_module("angleLog", log, DATA_STEP_NANOS, 0);
        }
        for log in &mut rho_logs {
            simulation.add_module("rhoLog", log, DATA_STEP_NANOS, 0);
        }

        // The arrays swing out.
        let mut current_s = ROTATION_DURATIONS_S[0].max(ROTATION_DURATIONS_S[1]) + 5.0;
        simulation.run_for(seconds_to_nanos(current_s));
        // Then extend, the outer elements first.
        let translation_s = TRANSLATION_DURATIONS_S[0].max(TRANSLATION_DURATIONS_S[1]);
        for count in 1..ELEMENTS {
            let target_rho_m = count as f64 * (ELEMENT_LENGTH_M + GAP_M);
            for element in 0..ELEMENTS - count {
                array_1.translation_refs[element].write(LinearTranslationRigidBodyMsg {
                    rho_m: target_rho_m,
                    rho_dot_mps: 0.0,
                });
                array_2.translation_refs[element].write(LinearTranslationRigidBodyMsg {
                    rho_m: -target_rho_m,
                    rho_dot_mps: 0.0,
                });
            }
            // One task step for the profilers to read the new commands.
            let mut next_s = current_s + translation_s + STEP_NANOS as f64 * 1.0e-9;
            if count == ELEMENTS - 1 {
                next_s += 3.0;
            }
            simulation.run_for(seconds_to_nanos(next_s) - seconds_to_nanos(current_s));
            current_s = next_s;
        }
        stop_nanos = simulation.current_sim_nanos();
    }
    csv.flush().expect("flush scenario CSV");

    let mut angles: [Vec<Vec<HingedRigidBodyMsg>>; 2] = [Vec::new(), Vec::new()];
    let mut displacements: [Vec<Vec<LinearTranslationRigidBodyMsg>>; 2] = [Vec::new(), Vec::new()];
    for (index, (angle_log, rho_log)) in angle_logs.iter().zip(&rho_logs).enumerate() {
        angles[index / ELEMENTS].push(angle_log.messages().cloned().collect());
        displacements[index / ELEMENTS].push(rho_log.messages().cloned().collect());
    }
    Results {
        csv_path,
        states: state_log.into_samples(),
        diagnostics: diagnostics_log.messages().cloned().collect(),
        angles,
        displacements,
        stop_nanos,
    }
}

/// Upstream `macros.sec2nano`: rounded to the nanosecond.
fn seconds_to_nanos(seconds: f64) -> u64 {
    (seconds * 1.0e9).round() as u64
}

fn main() {
    let results = run();
    println!(
        "wrote {} (stopped at {} s; final array 1 displacements {:?} m)",
        results.csv_path.display(),
        results.stop_nanos as f64 * 1.0e-9,
        results.displacements[0]
            .iter()
            .map(|log| log.last().expect("samples").rho_m)
            .collect::<Vec<_>>(),
    );
}

#[cfg(test)]
mod tests {
    use super::{
        ELEMENTS, THETA_REFS_RAD, deployed_rho_m, prescribed::final_relative_momentum, run,
    };

    /// Upstream runs the scenario in `test_scenarioTest.py` without checking
    /// values. The figures' story, checked here: the run stops at 83.15 s;
    /// every element ends at its array's angle (−90° and 90°) and at its
    /// extension (4.8, 3.2, 1.6 and 0 m outward); the hub turns and drifts
    /// as the arrays deploy at unequal rates; and the total angular
    /// momentum, zero at the start, is conserved through the extensions and
    /// left at 2 % of the hub's largest momentum by the swing-out: array 2's
    /// bang-bang switch falls a rounding error past a step boundary, so its
    /// one-step latching (see `prescribed::final_relative_momentum`) jumps
    /// the momentum by about the array's inertia times `θ̈ Δt` there and at
    /// the end of the swing.
    #[test]
    fn arrays_swing_out_and_extend() {
        let results = run();
        assert_eq!(results.stop_nanos, 83_150_000_000);
        for array in 0..2 {
            for element in 0..ELEMENTS {
                let angle = results.angles[array][element].last().expect("samples");
                assert!((angle.theta_rad - THETA_REFS_RAD[array]).abs() < 1.0e-9);
                let rho = results.displacements[array][element]
                    .last()
                    .expect("samples");
                assert!(
                    (rho.rho_m - deployed_rho_m(array, element)).abs() < 1.0e-9,
                    "array {array} element {element}: rho {} m",
                    rho.rho_m
                );
                assert_eq!(rho.rho_dot_mps, 0.0);
            }
        }
        let hub_rate = results
            .states
            .iter()
            .map(|(_, state)| state.omega_radps.norm())
            .fold(0.0, f64::max);
        assert!(hub_rate > 1.0e-4, "hub rate {hub_rate:e} rad/s");
        let (_, last) = results.states.last().expect("samples");
        assert!(
            last.position_m.norm() > 1.0e-3,
            "hub position {:?}",
            last.position_m
        );
        let hub_inertia = 800.0 * (16.0 + 2.25) / 12.0;
        let scale = hub_inertia * hub_rate;
        let momentum = final_relative_momentum(&results.diagnostics, scale);
        assert!(momentum < 2.0e-2, "final angular momentum {momentum:e}");
        // Once the arrays have swung out (by 40 s), the extensions conserve
        // it.
        let swung_out = results.diagnostics[800].rotational_angular_momentum_inertial_kg_m2ps;
        for sample in &results.diagnostics[800..] {
            let change = (sample.rotational_angular_momentum_inertial_kg_m2ps - swung_out).norm();
            assert!(
                change < 1.0e-6 * scale,
                "angular momentum change {change:e}"
            );
        }
    }
}
