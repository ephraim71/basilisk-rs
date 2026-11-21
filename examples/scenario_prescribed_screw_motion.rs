//! Port of upstream `examples/scenarioPrescribedScrewMotion.py`: helical
//! screw motion of two prescribed-motion bodies
//! ([`PrescribedMotionStateEffector`](basilisk_rs::dynamics::prescribed_motion_state_effector::PrescribedMotionStateEffector)),
//! each driven by a
//! [`PrescribedRotation1Dof`](basilisk_rs::device_interface::prescribed_rotation_1dof::PrescribedRotation1Dof)
//! profiler with a screw constant, which ties a translation along the spin
//! axis to the angle: `ρ = ρ0 + c θ`. On a 1000 kg cylindrical hub at rest,
//! the first 20 kg body turns about ŷ against the hub's side (`c = 0.1`
//! m/rad), the second about ẑ away from the hub (`c = 0.12` m/rad); both
//! complete two revolutions (720°) with smoothed bang-coast-bang profiles
//! (1 deg/s², 0.75 s bangs and smoothing), starting 0.4 m from their mounts.
//! The run lasts 600 s at a 0.01 s step.
//!
//! `scenarioPrescribedScrewMotion.csv` holds, every 0.1 s (upstream's
//! data-recording rate; upstream's recorders sample every step, which would
//! make the file ten times larger), the hub state (`r_BN_N_m_i`,
//! `sigma_BN_i`, `omega_BN_B_rad_per_s_i`), each body's prescribed states
//! (`r_P<k>M_M_m_i`, `rPrime_P<k>M_M_m_per_s_i`, `rPrimePrime_P<k>M_M_m_per_s2_i`,
//! `omega_P<k>M_P<k>_rad_per_s_i`, `omegaPrime_P<k>M_P<k>_rad_per_s2_i`) and
//! the profiled angles `theta_P<k>_rad`.
//! `plot_scenario_prescribed_screw_motion.py` draws upstream's eight figures.
//! Upstream's Vizard setup is not ported.
//!
//! Upstream checks this scenario only by running it (`test_scenarioTest.py`).
//! `tests` checks what the figures show: each body's displacement follows
//! the screw relation `ρ = ρ0 + c θ` throughout (upstream's "Prescribed
//! Displacement Errors"), both complete their two revolutions and stop, the
//! hub reacts, and the total angular momentum is zero again at the end.

#[path = "support/common.rs"]
mod common;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/prescribed.rs"]
mod prescribed;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::dynamics::prescribed_motion_state_effector::{
    PrescribedMotionConfig, PrescribedMotionStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, Output, PrescribedRotationMsg, PrescribedTranslationMsg,
    SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use prescribed::{box_inertia_kg_m2, hub_at_rest, rotation_profiler};
use recording::scenario_csv;

const STEM: &str = "scenarioPrescribedScrewMotion";
/// Upstream `dyn_time_step_sec = 0.01`.
const DYN_STEP_NANOS: u64 = 10_000_000;
/// Upstream `data_rec_time_step_sec = 0.1`.
const DATA_STEP_NANOS: u64 = 100_000_000;
/// Upstream `sim_time_1 = 600.0` s.
const DURATION_NANOS: u64 = 600_000_000_000;
/// Upstream `prescribed_theta_ref` \[rad\]: two revolutions.
pub const THETA_REF_RAD: f64 = 720.0 * std::f64::consts::PI / 180.0;
/// Upstream `init_displacement = 0.5 * width_prescribed` \[m\].
pub const INIT_DISPLACEMENT_M: f64 = 0.4;
/// Upstream `screw_constant_1` and `screw_constant_2` \[m/rad\].
pub const SCREW_CONSTANTS: [f64; 2] = [0.1, 0.12];

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub diagnostics: Vec<SpacecraftDiagnosticsMsg>,
    /// Upstream `prescribed_body_<k>.prescribedTranslationOutMsg`.
    pub translations: [Vec<PrescribedTranslationMsg>; 2],
    /// Upstream `prescribed_body_<k>.prescribedRotationOutMsg`.
    pub rotations: [Vec<PrescribedRotationMsg>; 2],
    /// Upstream `one_dof_rotation_profiler_<k>.spinningBodyOutMsg`.
    pub angles: [Vec<HingedRigidBodyMsg>; 2],
    /// The spin axes, `M` components.
    pub axes: [Vector3<f64>; 2],
}

pub fn run() -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mass_hub = 1_000.0;
    let (length_hub, width_hub, depth_hub) = (1.0, 1.0, 2.0);
    let mut spacecraft = hub_at_rest(
        mass_hub,
        box_inertia_kg_m2(mass_hub, length_hub, width_hub, depth_hub),
        DYN_STEP_NANOS,
    );

    // The bodies: 20 kg, 0.2 m x 0.8 m x 0.2 m, the second turned to its
    // spin axis.
    let mass_body = 20.0;
    let (length_body, width_body, depth_body) = (0.2_f64, 0.8_f64, 0.2_f64);
    let i_length = mass_body * (width_body.powi(2) + depth_body.powi(2)) / 12.0;
    let i_width = mass_body * (length_body.powi(2) + depth_body.powi(2)) / 12.0;
    let i_depth = mass_body * (length_body.powi(2) + width_body.powi(2)) / 12.0;
    let axes = [Vector3::y(), Vector3::z()];
    let body = |name: &str, inertia: Matrix3<f64>, mount_m: Vector3<f64>, axis: Vector3<f64>| {
        PrescribedMotionStateEffector::new(PrescribedMotionConfig {
            mass_kg: mass_body,
            inertia_about_com_p_kg_m2: inertia,
            mount_position_body_m: mount_m,
            r_pm_m_m: INIT_DISPLACEMENT_M * axis,
            ..PrescribedMotionConfig::new(name)
        })
    };
    let mut bodies = [
        body(
            "prescribedBody1",
            Matrix3::from_diagonal(&Vector3::new(i_length, i_width, i_depth)),
            Vector3::new(0.0, 0.5 * width_hub, 0.0),
            axes[0],
        ),
        body(
            "prescribedBody2",
            Matrix3::from_diagonal(&Vector3::new(i_length, i_depth, i_width)),
            Vector3::new(0.65 * length_hub, 0.0, -0.5 * depth_hub),
            axes[1],
        ),
    ];

    let reference = Output::new(HingedRigidBodyMsg {
        theta_rad: THETA_REF_RAD,
        theta_dot_radps: 0.0,
    });
    let mut profilers = [0, 1].map(|index| {
        let mut profiler = rotation_profiler(
            &format!("prescribedRotation1DOF{}", index + 1),
            axes[index],
            1.0 * degrees,
            0.0,
            0.75,
            0.75,
        );
        profiler.config.init_displacement_m = INIT_DISPLACEMENT_M;
        profiler.config.screw_constant_m_per_rad = SCREW_CONSTANTS[index];
        profiler
    });
    for (profiler, body) in profilers.iter_mut().zip(&mut bodies) {
        reference.connect_to(&mut profiler.spinning_body_in_msg);
        profiler
            .prescribed_rotation_out_msg
            .connect_to(&mut body.prescribed_rotation_in);
        profiler
            .prescribed_translation_out_msg
            .connect_to(&mut body.prescribed_translation_in);
    }
    let translation_out = [0, 1].map(|index| bodies[index].prescribed_translation_out.clone());
    let rotation_out = [0, 1].map(|index| bodies[index].prescribed_rotation_out.clone());
    for body in bodies {
        spacecraft.add_state_effector(body);
    }
    set_center_of_mass_initial_state(&mut spacecraft, Vector3::zeros(), Vector3::zeros());

    let mut csv = scenario_csv(STEM);
    let csv_path = csv.config.output_path.clone();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut translation_logs = [(); 2].map(|_| MessageLog::<PrescribedTranslationMsg>::default());
    let mut rotation_logs = [(); 2].map(|_| MessageLog::<PrescribedRotationMsg>::default());
    let mut angle_logs = [(); 2].map(|_| MessageLog::<HingedRigidBodyMsg>::default());
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
        for index in 0..2 {
            let k = index + 1;
            connect!(&simulation,
                &translation_out[index] => csv.add_source::<PrescribedTranslationMsg>(
                    CsvSourceConfig::columns(
                        vector_columns("r_pm_m_m", &format!("r_P{k}M_M_m"))
                            .into_iter()
                            .chain(vector_columns(
                                "r_prime_pm_m_mps",
                                &format!("rPrime_P{k}M_M_m_per_s"),
                            ))
                            .chain(vector_columns(
                                "r_prime_prime_pm_m_mps2",
                                &format!("rPrimePrime_P{k}M_M_m_per_s2"),
                            )),
                    ),
                ),
                &rotation_out[index] => csv.add_source::<PrescribedRotationMsg>(
                    CsvSourceConfig::columns(
                        vector_columns("omega_pm_p_radps", &format!("omega_P{k}M_P{k}_rad_per_s"))
                            .into_iter()
                            .chain(vector_columns(
                                "omega_prime_pm_p_radps2",
                                &format!("omegaPrime_P{k}M_P{k}_rad_per_s2"),
                            )),
                    ),
                ),
                &profilers[index].spinning_body_out_msg => csv.add_source::<HingedRigidBodyMsg>(
                    CsvSourceConfig::columns([("theta_rad".to_string(), format!("theta_P{k}_rad"))]),
                ),
                &translation_out[index] => &mut translation_logs[index].input,
                &rotation_out[index] => &mut rotation_logs[index].input,
                &profilers[index].spinning_body_out_msg => &mut angle_logs[index].input,
            );
        }
        let [profiler_1, profiler_2] = &mut profilers;
        let [translation_log_1, translation_log_2] = &mut translation_logs;
        let [rotation_log_1, rotation_log_2] = &mut rotation_logs;
        let [angle_log_1, angle_log_2] = &mut angle_logs;
        schedule! { simulation,
            "scObject" => &mut spacecraft, period=DYN_STEP_NANOS, priority=100;
            "prescribedRotation1DOF1" => profiler_1, period=DYN_STEP_NANOS, priority=50;
            "prescribedRotation1DOF2" => profiler_2, period=DYN_STEP_NANOS, priority=50;
            "csv" => &mut csv, period=DATA_STEP_NANOS, priority=0;
            "scStateLog" => &mut state_log, period=DYN_STEP_NANOS, priority=0;
            "diagnosticsLog" => &mut diagnostics_log, period=DYN_STEP_NANOS, priority=0;
            "translation1Log" => translation_log_1, period=DYN_STEP_NANOS, priority=0;
            "translation2Log" => translation_log_2, period=DYN_STEP_NANOS, priority=0;
            "rotation1Log" => rotation_log_1, period=DYN_STEP_NANOS, priority=0;
            "rotation2Log" => rotation_log_2, period=DYN_STEP_NANOS, priority=0;
            "profiler1Log" => angle_log_1, period=DYN_STEP_NANOS, priority=0;
            "profiler2Log" => angle_log_2, period=DYN_STEP_NANOS, priority=0;
        }
        simulation.run_for(DURATION_NANOS);
    }
    csv.flush().expect("flush scenario CSV");

    let [translation_log_1, translation_log_2] = translation_logs;
    let [rotation_log_1, rotation_log_2] = rotation_logs;
    let [angle_log_1, angle_log_2] = angle_logs;
    Results {
        csv_path,
        states: state_log.into_samples(),
        diagnostics: diagnostics_log.values.iter().cloned().collect(),
        translations: [translation_log_1, translation_log_2]
            .map(|log| log.values.iter().cloned().collect::<Vec<_>>()),
        rotations: [rotation_log_1, rotation_log_2]
            .map(|log| log.values.iter().cloned().collect::<Vec<_>>()),
        angles: [angle_log_1, angle_log_2]
            .map(|log| log.values.iter().cloned().collect::<Vec<_>>()),
        axes,
    }
}

impl Results {
    /// Upstream's `prescribed_rho_<k>_error`: the largest
    /// `|ρ0 + c θ - r_PM_M · axis|` \[m\] after the first sample.
    pub fn max_screw_error_m(&self, body: usize) -> f64 {
        self.angles[body]
            .iter()
            .zip(&self.translations[body])
            .skip(1)
            .map(|(angle, translation)| {
                let truth = SCREW_CONSTANTS[body] * angle.theta_rad + INIT_DISPLACEMENT_M;
                (truth - translation.r_pm_m_m.dot(&self.axes[body])).abs()
            })
            .fold(0.0, f64::max)
    }
}

fn main() {
    let results = run();
    println!(
        "wrote {} (final angles {:.3} and {:.3} deg, largest screw errors {:.3e} and {:.3e} m)",
        results.csv_path.display(),
        results.angles[0]
            .last()
            .expect("samples")
            .theta_rad
            .to_degrees(),
        results.angles[1]
            .last()
            .expect("samples")
            .theta_rad
            .to_degrees(),
        results.max_screw_error_m(0),
        results.max_screw_error_m(1),
    );
}

#[cfg(test)]
mod tests {
    use super::{
        INIT_DISPLACEMENT_M, SCREW_CONSTANTS, THETA_REF_RAD, prescribed::final_relative_momentum,
        run,
    };

    /// Upstream runs the scenario in `test_scenarioTest.py` without checking
    /// values. The figures' story, checked here: each body's displacement
    /// follows `ρ0 + c θ` to within a micrometre throughout; both bodies end
    /// at rest after two revolutions, at `ρ0 + 4π c`; the hub reacts to
    /// their motion; and the total angular momentum, zero at the start, is
    /// zero again at the end.
    #[test]
    fn bodies_screw_out_along_their_axes() {
        let results = run();
        assert_eq!(results.states.len(), 60_001);
        for body in 0..2 {
            let error_m = results.max_screw_error_m(body);
            assert!(error_m < 1.0e-6, "body {body}: screw error {error_m:e} m");
            let angle = results.angles[body].last().expect("samples");
            assert!(
                (angle.theta_rad - THETA_REF_RAD).abs() < 1.0e-9,
                "{}",
                angle.theta_rad
            );
            assert_eq!(angle.theta_dot_radps, 0.0);
            let translation = results.translations[body].last().expect("samples");
            let rho_m = translation.r_pm_m_m.dot(&results.axes[body]);
            let expected_m = INIT_DISPLACEMENT_M + SCREW_CONSTANTS[body] * THETA_REF_RAD;
            assert!(
                (rho_m - expected_m).abs() < 1.0e-9,
                "body {body}: rho {rho_m} m"
            );
            assert!(translation.r_prime_pm_m_mps.norm() < 1.0e-12);
        }
        let hub_rate = results
            .states
            .iter()
            .map(|(_, state)| state.omega_radps.norm())
            .fold(0.0, f64::max);
        assert!(hub_rate > 1.0e-5, "hub rate {hub_rate:e} rad/s");
        let hub_inertia = 1_000.0 * (1.0 + 4.0) / 12.0;
        let momentum = final_relative_momentum(&results.diagnostics, hub_inertia * hub_rate);
        assert!(momentum < 1.0e-4, "final angular momentum {momentum:e}");
    }
}
