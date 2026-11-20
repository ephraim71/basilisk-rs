//! Port of upstream `examples/scenarioPrescribedMotionWithTranslationBranching.py`:
//! translating bodies riding on a prescribed-motion platform (upstream
//! "branching"), a lander concept. Under a 1000 kg cubic hub, a 200 kg
//! platform
//! ([`PrescribedMotionStateEffector`](basilisk_rs::dynamics::prescribed_motion_state_effector::PrescribedMotionStateEffector))
//! carries three telescoping landing struts
//! ([`LinearTranslationOneDofStateEffector`](basilisk_rs::dynamics::linear_translation_one_dof_state_effector::LinearTranslationOneDofStateEffector),
//! 50 kg, a soft spring–damper), canted 60° from the platform base and 120°
//! apart. Starting at rest, the platform is profiled to rotate 10° about ŷ
//! and translate 0.5 m along −ẑ with smoothed bang-coast-bang profiles; at
//! 90 s the struts are commanded to 0.2, 0.1 and 0.15 m. The run lasts
//! 150 s at a 0.01 s step.
//!
//! `scenarioPrescribedMotionWithTranslationBranching.csv` holds at every
//! step (upstream records on the dynamics task) the hub state
//! (`r_BN_N_m_i`, `sigma_BN_i`, `omega_BN_B_rad_per_s_i`), the platform's
//! prescribed states (`r_PM_M_m_i`, `rPrime_PM_M_m_per_s_i`,
//! `rPrimePrime_PM_M_m_per_s2_i`, `omega_PM_P_rad_per_s_i`,
//! `omegaPrime_PM_P_rad_per_s2_i`), the profiled angle `theta_P_rad`, and
//! each strut's `rho_T<k>_m` and `rhoDot_T<k>_m_per_s`.
//! `plot_scenario_prescribed_motion_with_translation_branching.py` draws
//! upstream's eight figures. Upstream's Vizard setup is not ported.
//!
//! Upstream subscribes the struts' reference inputs after the first 90 s;
//! here they are connected from the start to references at the struts'
//! initial `rho_ref = 0` and rewritten then. Upstream's struts read the new
//! references in their `UpdateState` of the first step after 90 s and use
//! them from the next step on, which here is the first step's end.
//!
//! Upstream checks this scenario only by running it (`test_scenarioTest.py`).
//! `tests` checks what the figures show: the platform completes both
//! motions, the struts deploy to their references, the hub reacts, and the
//! total angular momentum is zero again once the platform has stopped, and
//! constant while the struts deploy.

#[path = "support/common.rs"]
mod common;
#[path = "support/hub_init.rs"]
mod hub_init;
#[path = "support/prescribed.rs"]
mod prescribed;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::dynamics::linear_translation_one_dof_state_effector::{
    LinearTranslationOneDofConfig, LinearTranslationOneDofStateEffector,
};
use basilisk_rs::dynamics::prescribed_motion_state_effector::{
    PrescribedMotionConfig, PrescribedMotionStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, LinearTranslationRigidBodyMsg, Output, PrescribedRotationMsg,
    PrescribedTranslationMsg, SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
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

const STEM: &str = "scenarioPrescribedMotionWithTranslationBranching";
/// Upstream `dyn_time_step_sec = 0.01`.
const DYN_STEP_NANOS: u64 = 10_000_000;
/// Upstream `sim_time_1 = 90.0` s: the platform alone.
const PLATFORM_PHASE_NANOS: u64 = 90_000_000_000;
/// Upstream `sim_time_2 = 60.0` s: the struts deploy.
const STRUT_PHASE_NANOS: u64 = 60_000_000_000;
/// Upstream `prescribed_theta_ref` \[rad\].
const THETA_REF_RAD: f64 = 10.0 * std::f64::consts::PI / 180.0;
/// Upstream `prescribed_pos_ref` \[m\].
const RHO_REF_M: f64 = 0.5;
/// Upstream `strut_<k>_rho_ref` \[m\].
pub const STRUT_RHO_REFS_M: [f64; 3] = [0.2, 0.1, 0.15];

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub diagnostics: Vec<SpacecraftDiagnosticsMsg>,
    pub platform_translation: Vec<PrescribedTranslationMsg>,
    pub platform_rotation: Vec<PrescribedRotationMsg>,
    pub platform_angle: Vec<HingedRigidBodyMsg>,
    pub struts: [Vec<LinearTranslationRigidBodyMsg>; 3],
}

/// Upstream's `[[c, s, 0], [-s, c, 0], [0, 0, 1]]`: a rotation by `angle`
/// about the third axis.
fn about_third_axis(angle_rad: f64) -> Matrix3<f64> {
    let (sin, cos) = angle_rad.sin_cos();
    Matrix3::new(cos, sin, 0.0, -sin, cos, 0.0, 0.0, 0.0, 1.0)
}

/// Upstream's `[[c, 0, -s], [0, 1, 0], [s, 0, c]]`: a rotation by `angle`
/// about the second axis.
fn about_second_axis(angle_rad: f64) -> Matrix3<f64> {
    let (sin, cos) = angle_rad.sin_cos();
    Matrix3::new(cos, 0.0, -sin, 0.0, 1.0, 0.0, sin, 0.0, cos)
}

pub fn run() -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mass_hub = 1_000.0;
    let (length_hub, width_hub, depth_hub) = (1.0, 1.0, 1.0);
    let mut spacecraft = hub_at_rest(
        mass_hub,
        box_inertia_kg_m2(mass_hub, length_hub, width_hub, depth_hub),
        DYN_STEP_NANOS,
    );

    // The platform: 200 kg, 1 m x 1 m x 0.2 m, under the hub.
    let mass_platform = 200.0;
    let depth_platform = 0.2;
    let rot_axis_m = Vector3::y();
    let trans_axis_m = Vector3::new(0.0, 0.0, -1.0);
    let mut platform = PrescribedMotionStateEffector::new(PrescribedMotionConfig {
        mass_kg: mass_platform,
        inertia_about_com_p_kg_m2: box_inertia_kg_m2(mass_platform, 1.0, 1.0, depth_platform),
        mount_position_body_m: Vector3::new(0.0, 0.0, -0.5 * depth_hub),
        center_of_mass_p_m: Vector3::new(0.0, 0.0, 0.5 * depth_platform),
        r_pm_m_m: Vector3::new(0.0, 0.0, -depth_platform),
        sigma_pm: prv_to_mrp(0.0 * rot_axis_m),
        ..PrescribedMotionConfig::new("prescribedPlatform")
    });

    let mut rotation = rotation_profiler(
        "prescribedRotation1DOF",
        rot_axis_m,
        0.5 * degrees,
        0.0,
        0.75,
        0.75,
    );
    let mut translation = translation_profiler(
        "prescribedLinearTranslation",
        trans_axis_m,
        0.0025,
        depth_platform,
        1.0,
        1.0,
    );
    let rotation_ref = Output::new(HingedRigidBodyMsg {
        theta_rad: THETA_REF_RAD,
        theta_dot_radps: 0.0,
    });
    let translation_ref = Output::new(LinearTranslationRigidBodyMsg {
        rho_m: RHO_REF_M,
        rho_dot_mps: 0.0,
    });
    rotation_ref.connect_to(&mut rotation.spinning_body_in_msg);
    rotation
        .prescribed_rotation_out_msg
        .connect_to(&mut platform.prescribed_rotation_in);
    translation_ref.connect_to(&mut translation.linear_translation_rigid_body_in_msg);
    translation
        .prescribed_translation_out_msg
        .connect_to(&mut platform.prescribed_translation_in);

    // The telescoping struts: 50 kg, 0.2 m x 0.2 m x 0.5 m, canted 60°.
    let mass_strut = 50.0;
    let (length_strut, width_strut, depth_strut) = (0.2, 0.2, 0.5);
    let cant_rad = 60.0 * degrees;
    let spacing_m = 0.25;
    let l = spacing_m - 0.5 * width_strut * cant_rad.cos();
    let z = -(0.5 * width_strut * cant_rad.sin());
    let strut_origins_m = [
        Vector3::new(l * (30.0 * degrees).cos(), l * (30.0 * degrees).sin(), z),
        Vector3::new(0.0, -l, z),
        Vector3::new(-l * (30.0 * degrees).cos(), l * (30.0 * degrees).sin(), z),
    ];
    let strut_azimuths_rad = [30.0 * degrees, -90.0 * degrees, 150.0 * degrees];
    let strut_refs = [(); 3].map(|_| Output::new(LinearTranslationRigidBodyMsg::default()));
    let mut strut_out = Vec::new();
    for (index, (origin_m, azimuth_rad)) in strut_origins_m
        .into_iter()
        .zip(strut_azimuths_rad)
        .enumerate()
    {
        let dcm_fp = about_second_axis(cant_rad) * about_third_axis(azimuth_rad);
        let mut strut = LinearTranslationOneDofStateEffector::new(LinearTranslationOneDofConfig {
            mass_kg: mass_strut,
            spring_constant_n_per_m: 8.0,
            damping_n_s_per_m: 8.0,
            rho_init_m: 0.0,
            rho_dot_init_mps: 0.0,
            axis_body: dcm_fp.transpose() * Vector3::x(),
            center_of_mass_translating_m: Vector3::new(0.5 * depth_strut, 0.0, 0.0),
            origin0_body_m: origin_m,
            inertia_about_com_translating_kg_m2: box_inertia_kg_m2(
                mass_strut,
                length_strut,
                width_strut,
                depth_strut,
            ),
            body_to_translating_dcm: dcm_fp,
            ..LinearTranslationOneDofConfig::new(format!("translatingBody{}", index + 1))
        });
        strut_refs[index].connect_to(&mut strut.translating_body_ref_in);
        strut_out.push(strut.translating_body_out.clone());
        platform.add_state_effector(strut);
    }

    let platform_translation_out = platform.prescribed_translation_out.clone();
    let platform_rotation_out = platform.prescribed_rotation_out.clone();
    spacecraft.add_state_effector(platform);
    set_center_of_mass_initial_state(&mut spacecraft, Vector3::zeros(), Vector3::zeros());

    let mut csv = scenario_csv(STEM);
    let csv_path = csv.config.output_path.clone();
    let mut state_log = History::<SpacecraftStateMsg>::new();
    let mut diagnostics_log = History::<SpacecraftDiagnosticsMsg>::new();
    let mut translation_log = History::<PrescribedTranslationMsg>::new();
    let mut rotation_log = History::<PrescribedRotationMsg>::new();
    let mut angle_log = History::<HingedRigidBodyMsg>::new();
    let mut strut_logs = [(); 3].map(|_| History::<LinearTranslationRigidBodyMsg>::new());
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
            &platform_translation_out => csv.add_source::<PrescribedTranslationMsg>(
                CsvSourceConfig::columns(
                    vector_columns("r_pm_m_m", "r_PM_M_m")
                        .into_iter()
                        .chain(vector_columns("r_prime_pm_m_mps", "rPrime_PM_M_m_per_s"))
                        .chain(vector_columns(
                            "r_prime_prime_pm_m_mps2",
                            "rPrimePrime_PM_M_m_per_s2",
                        )),
                ),
            ),
            &platform_rotation_out => csv.add_source::<PrescribedRotationMsg>(
                CsvSourceConfig::columns(
                    vector_columns("omega_pm_p_radps", "omega_PM_P_rad_per_s")
                        .into_iter()
                        .chain(vector_columns(
                            "omega_prime_pm_p_radps2",
                            "omegaPrime_PM_P_rad_per_s2",
                        )),
                ),
            ),
            &rotation.spinning_body_out_msg => csv.add_source::<HingedRigidBodyMsg>(
                CsvSourceConfig::columns([("theta_rad", "theta_P_rad")]),
            ),
            &spacecraft.state_out => &mut state_log.input,
            &spacecraft.diagnostics_out => &mut diagnostics_log.input,
            &platform_translation_out => &mut translation_log.input,
            &platform_rotation_out => &mut rotation_log.input,
            &rotation.spinning_body_out_msg => &mut angle_log.input,
        );
        for (index, (output, log)) in strut_out.iter().zip(strut_logs.iter_mut()).enumerate() {
            let k = index + 1;
            connect!(&simulation,
                output => csv.add_source::<LinearTranslationRigidBodyMsg>(CsvSourceConfig::columns([
                    ("rho_m".to_string(), format!("rho_T{k}_m")),
                    ("rho_dot_mps".to_string(), format!("rhoDot_T{k}_m_per_s")),
                ])),
                output => &mut log.input,
            );
        }
        let [strut_log_1, strut_log_2, strut_log_3] = &mut strut_logs;
        schedule! { simulation,
            "scObject" => &mut spacecraft, period=DYN_STEP_NANOS, priority=100;
            "prescribedRotation1DOF" => &mut rotation, period=DYN_STEP_NANOS, priority=50;
            "prescribedLinearTranslation" => &mut translation, period=DYN_STEP_NANOS, priority=50;
            "csv" => &mut csv, period=DYN_STEP_NANOS, priority=0;
            "scStateLog" => &mut state_log, period=DYN_STEP_NANOS, priority=0;
            "diagnosticsLog" => &mut diagnostics_log, period=DYN_STEP_NANOS, priority=0;
            "prescribedTranslationLog" => &mut translation_log, period=DYN_STEP_NANOS, priority=0;
            "prescribedRotationLog" => &mut rotation_log, period=DYN_STEP_NANOS, priority=0;
            "profilerLog" => &mut angle_log, period=DYN_STEP_NANOS, priority=0;
            "strut1Log" => strut_log_1, period=DYN_STEP_NANOS, priority=0;
            "strut2Log" => strut_log_2, period=DYN_STEP_NANOS, priority=0;
            "strut3Log" => strut_log_3, period=DYN_STEP_NANOS, priority=0;
        }

        // Chunk 1: the platform alone.
        simulation.run_for(PLATFORM_PHASE_NANOS);
        // Chunk 2: the struts deploy, from the step after the first.
        simulation.run_for(DYN_STEP_NANOS);
        for (reference, rho_m) in strut_refs.iter().zip(STRUT_RHO_REFS_M) {
            reference.write(LinearTranslationRigidBodyMsg {
                rho_m,
                rho_dot_mps: 0.0,
            });
        }
        simulation.run_for(STRUT_PHASE_NANOS - DYN_STEP_NANOS);
    }
    csv.flush().expect("flush scenario CSV");

    let [strut_log_1, strut_log_2, strut_log_3] = strut_logs;
    Results {
        csv_path,
        states: state_log.into_samples(),
        diagnostics: diagnostics_log.messages().cloned().collect(),
        platform_translation: translation_log.messages().cloned().collect(),
        platform_rotation: rotation_log.messages().cloned().collect(),
        platform_angle: angle_log.messages().cloned().collect(),
        struts: [strut_log_1, strut_log_2, strut_log_3]
            .map(|log| log.messages().cloned().collect::<Vec<_>>()),
    }
}

fn main() {
    let results = run();
    println!(
        "wrote {} (final platform angle {:.4} deg, final strut displacements {:?} m)",
        results.csv_path.display(),
        results
            .platform_angle
            .last()
            .expect("samples")
            .theta_rad
            .to_degrees(),
        results
            .struts
            .iter()
            .map(|strut| strut.last().expect("samples").rho_m)
            .collect::<Vec<_>>(),
    );
}

#[cfg(test)]
mod tests {
    use super::{
        RHO_REF_M, STRUT_RHO_REFS_M, THETA_REF_RAD, prescribed::final_relative_momentum, run,
    };
    use nalgebra::Vector3;

    /// Upstream runs the scenario in `test_scenarioTest.py` without checking
    /// values. The figures' story, checked here: the platform ends at rest at
    /// 10° and 0.5 m; the struts deploy when they are commanded at 90 s
    /// and end within 2 % of their references; the hub moves and turns
    /// in reaction; and the total angular momentum, zero at the start, is
    /// zero again at the end (to 1e-4 of the hub's largest momentum, what
    /// the prescribed profiles' one-step latching leaves) and constant while
    /// the struts deploy.
    #[test]
    fn platform_moves_and_the_struts_deploy() {
        let results = run();
        assert_eq!(results.states.len(), 15_001);
        let angle = results.platform_angle.last().expect("samples");
        assert!((angle.theta_rad - THETA_REF_RAD).abs() < 1.0e-9);
        assert_eq!(angle.theta_dot_radps, 0.0);
        let translation = results.platform_translation.last().expect("samples");
        assert!((translation.r_pm_m_m - Vector3::new(0.0, 0.0, -RHO_REF_M)).norm() < 1.0e-9);
        assert!(translation.r_prime_pm_m_mps.norm() < 1.0e-12);
        assert!(
            results
                .platform_rotation
                .last()
                .expect("samples")
                .omega_pm_p_radps
                .norm()
                < 1.0e-12
        );
        for (strut, reference) in results.struts.iter().zip(STRUT_RHO_REFS_M) {
            // Sample 9000 is t = 90 s, before the command: the struts only
            // sway with the platform's accelerations.
            let before = strut[9000].rho_m;
            assert!(
                (before - reference).abs() > 0.5 * reference,
                "strut at {before} m before deploying"
            );
            let last = strut.last().expect("samples").rho_m;
            assert!(
                (last - reference).abs() < 0.02 * reference,
                "strut at {last} m"
            );
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
        let hub_inertia = 1_000.0 * 2.0 / 12.0;
        let scale = hub_inertia * hub_rate;
        let momentum = final_relative_momentum(&results.diagnostics, scale);
        assert!(momentum < 1.0e-4, "final angular momentum {momentum:e}");
        // With the platform at rest (it stops before 70 s), the struts'
        // motion conserves it exactly.
        let settled = results.diagnostics[7000].rotational_angular_momentum_inertial_kg_m2ps;
        for sample in &results.diagnostics[7000..] {
            let change = (sample.rotational_angular_momentum_inertial_kg_m2ps - settled).norm();
            assert!(
                change < 1.0e-9 * scale,
                "angular momentum change {change:e}"
            );
        }
    }
}
