//! Port of upstream `examples/scenarioPrescribedMotionWithRotationBranching.py`:
//! state effectors riding on prescribed-motion bodies (upstream
//! "branching"). An ISS-scale spacecraft, a 15 000 kg hub, carries two 50 m
//! trusses, each a
//! [`PrescribedMotionStateEffector`](basilisk_rs::dynamics::prescribed_motion_state_effector::PrescribedMotionStateEffector)
//! mounted 4 m from the hub center, the second turned 180° about ŷ. Four
//! single-axis solar panels
//! ([`SpinningBodyOneDofStateEffector`](basilisk_rs::dynamics::spinning_body_one_dof_state_effector::SpinningBodyOneDofStateEffector),
//! 1000 kg, 30 m, a stiff spring–damper on their bending axis) ride on each
//! truss. Starting at rest, the trusses are profiled to rotate 45° and −45°
//! about their long axes with smoothed bang-coast-bang profiles
//! ([`PrescribedRotation1Dof`](basilisk_rs::device_interface::prescribed_rotation_1dof::PrescribedRotation1Dof),
//! 0.5 deg/s², 1 s bangs and smoothing); their translation profilers hold
//! them in place. The run lasts 90 s at a 0.01 s step.
//!
//! `scenarioPrescribedMotionWithRotationBranching.csv` holds, every 0.1 s
//! (upstream's data-recording task), the hub state (`r_BN_N_m_i`,
//! `sigma_BN_i`, `omega_BN_B_rad_per_s_i`), the profiled truss angles
//! (`theta_P1_rad`, `theta_P2_rad`), each truss's `omega_P<k>M_P<k>_rad_per_s_i`
//! and `omegaPrime_P<k>M_P<k>_rad_per_s2_i`, and the angles and rates of
//! panels 1 and 3 (`theta_S1_rad`, `thetaDot_S1_rad_per_s`, ...).
//! `plot_scenario_prescribed_motion_with_rotation_branching.py` draws
//! upstream's eight figures. Upstream's Vizard setup is not ported.
//!
//! Upstream checks this scenario only by running it (`test_scenarioTest.py`).
//! `tests` checks what the figures show: both trusses complete their
//! rotations and come to rest, the panels flex in response and the hub
//! counter-rotates, and the total angular momentum is zero again once the
//! trusses have stopped.

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
use basilisk_rs::dynamics::spinning_body_one_dof_state_effector::{
    SpinningBodyOneDofConfig, SpinningBodyOneDofStateEffector,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, LinearTranslationRigidBodyMsg, Output, PrescribedRotationMsg,
    SpacecraftDiagnosticsMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::rigid_body_kinematics::{dcm_to_mrp, prv_to_mrp};
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use prescribed::{box_inertia_kg_m2, hub_at_rest, rotation_profiler, translation_profiler};
use recording::scenario_csv;

const STEM: &str = "scenarioPrescribedMotionWithRotationBranching";
/// Upstream `dyn_time_step_sec = 0.01`.
const DYN_STEP_NANOS: u64 = 10_000_000;
/// Upstream `data_rec_time_step_sec = 0.1`.
const DATA_STEP_NANOS: u64 = 100_000_000;
/// Upstream `sim_time = 90.0` s.
const DURATION_NANOS: u64 = 90_000_000_000;
/// Upstream `prescribed_theta_ref` \[rad\]: truss 1 turns `+45°`, truss 2
/// `−45°`.
const THETA_REF_RAD: f64 = 45.0 * std::f64::consts::PI / 180.0;

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    pub diagnostics: Vec<SpacecraftDiagnosticsMsg>,
    /// Upstream `one_dof_rotation_profiler_<k>.spinningBodyOutMsg`.
    pub truss_angles: [Vec<HingedRigidBodyMsg>; 2],
    /// Upstream `prescribed_truss_<k>.prescribedRotationOutMsg`.
    pub truss_rotations: [Vec<PrescribedRotationMsg>; 2],
    /// Upstream `spinning_panel_list[0]` and `[2]` `spinningBodyOutMsg`.
    pub panels: [Vec<HingedRigidBodyMsg>; 2],
}

pub fn run() -> Results {
    let degrees = std::f64::consts::PI / 180.0;
    let mass_hub = 15_000.0;
    let mut spacecraft = hub_at_rest(
        mass_hub,
        box_inertia_kg_m2(mass_hub, 8.0, 8.0, 20.0),
        DYN_STEP_NANOS,
    );

    // The trusses: 6000 kg, 50 m x 4 m x 4 m.
    let rot_axis_m = Vector3::x();
    let sigma_p0m = prv_to_mrp(0.0 * rot_axis_m);
    let mass_truss = 6_000.0;
    let truss = |name: &str, mount_m: Vector3<f64>, sigma_mb: Vector3<f64>| {
        PrescribedMotionStateEffector::new(PrescribedMotionConfig {
            mass_kg: mass_truss,
            inertia_about_com_p_kg_m2: box_inertia_kg_m2(mass_truss, 50.0, 4.0, 4.0),
            center_of_mass_p_m: Vector3::new(25.0, 0.0, 0.0),
            mount_position_body_m: mount_m,
            sigma_mb,
            sigma_pm: sigma_p0m,
            ..PrescribedMotionConfig::new(name)
        })
    };
    let flipped = Matrix3::from_diagonal(&Vector3::new(-1.0, 1.0, -1.0));
    let mut trusses = [
        truss(
            "prescribedTruss1",
            Vector3::new(4.0, 0.0, 0.0),
            Vector3::zeros(),
        ),
        truss(
            "prescribedTruss2",
            Vector3::new(-4.0, 0.0, 0.0),
            dcm_to_mrp(flipped),
        ),
    ];

    // The profilers: 45° and -45° about the truss axes, no translation.
    let mut rotation_profilers = [1, 2].map(|index| {
        rotation_profiler(
            &format!("prescribedRotation1DOF_{index}"),
            rot_axis_m,
            0.5 * degrees,
            0.0,
            1.0,
            1.0,
        )
    });
    let mut translation_profilers = [1, 2].map(|index| {
        translation_profiler(
            &format!("prescribedLinearTranslation_{index}"),
            Vector3::x(),
            0.005,
            0.0,
            1.0,
            1.0,
        )
    });
    let rotation_refs = [THETA_REF_RAD, -THETA_REF_RAD].map(|theta_rad| {
        Output::new(HingedRigidBodyMsg {
            theta_rad,
            theta_dot_radps: 0.0,
        })
    });
    let translation_ref = Output::new(LinearTranslationRigidBodyMsg {
        rho_m: 0.0,
        rho_dot_mps: 0.0,
    });
    for (((truss, rotation), translation), reference) in trusses
        .iter_mut()
        .zip(&mut rotation_profilers)
        .zip(&mut translation_profilers)
        .zip(&rotation_refs)
    {
        reference.connect_to(&mut rotation.spinning_body_in_msg);
        rotation
            .prescribed_rotation_out_msg
            .connect_to(&mut truss.prescribed_rotation_in);
        translation_ref.connect_to(&mut translation.linear_translation_rigid_body_in_msg);
        translation
            .prescribed_translation_out_msg
            .connect_to(&mut truss.prescribed_translation_in);
    }

    // The solar panels: 1000 kg, 10 m x 0.3 m x 30 m, on the truss's P
    // frame. Upstream's `I_panel_33` uses the truss width.
    let mass_panel = 1_000.0;
    let (length_panel, width_panel, depth_panel) = (10.0, 0.3, 30.0);
    let width_truss = 4.0;
    let panel_inertia = Matrix3::from_diagonal(&Vector3::new(
        mass_panel * (length_panel * length_panel + depth_panel * depth_panel) / 12.0,
        mass_panel * (length_panel * length_panel + width_panel * width_panel) / 12.0,
        mass_panel * (width_truss * width_truss + depth_panel * depth_panel) / 12.0,
    ));
    let panel_positions_m = [
        [50.0, 0.0, 2.0],
        [35.0, 0.0, 2.0],
        [50.0, 0.0, -2.0],
        [35.0, 0.0, -2.0],
        [35.0, 0.0, -2.0],
        [50.0, 0.0, -2.0],
        [35.0, 0.0, 2.0],
        [50.0, 0.0, 2.0],
    ];
    let mut logged_panels = Vec::new();
    for (index, position_m) in panel_positions_m.into_iter().enumerate() {
        let panel_number = index + 1;
        let dcm_s0p = if [1, 2, 7, 8].contains(&panel_number) {
            Matrix3::identity()
        } else {
            flipped
        };
        let panel = SpinningBodyOneDofStateEffector::new(SpinningBodyOneDofConfig {
            mass_kg: mass_panel,
            spring_constant_nm_per_rad: 700_000.0,
            damping_nm_s_per_rad: 50_000.0,
            spinner_origin_body_m: Vector3::from(position_m),
            center_of_mass_spinner_m: Vector3::new(0.0, 0.0, 15.0),
            spin_axis_spinner: Vector3::x(),
            inertia_about_com_spinner_kg_m2: panel_inertia,
            body_to_spinner0_dcm: dcm_s0p,
            ..SpinningBodyOneDofConfig::new(format!("spinningBody{panel_number}"))
        });
        if panel_number == 1 || panel_number == 3 {
            logged_panels.push(panel.spinning_body_out.clone());
        }
        trusses[usize::from(index >= 4)].add_state_effector(panel);
    }

    let truss_rotation_out = [0, 1].map(|index| trusses[index].prescribed_rotation_out.clone());
    let [truss_1, truss_2] = trusses;
    spacecraft.add_state_effector(truss_1);
    spacecraft.add_state_effector(truss_2);
    set_center_of_mass_initial_state(&mut spacecraft, Vector3::zeros(), Vector3::zeros());

    let mut csv = scenario_csv(STEM);
    let csv_path = csv.config.output_path.clone();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut diagnostics_log = MessageLog::<SpacecraftDiagnosticsMsg>::default();
    let mut angle_logs = [(); 2].map(|_| MessageLog::<HingedRigidBodyMsg>::default());
    let mut rotation_logs = [(); 2].map(|_| MessageLog::<PrescribedRotationMsg>::default());
    let mut panel_logs = [(); 2].map(|_| MessageLog::<HingedRigidBodyMsg>::default());
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
                &rotation_profilers[index].spinning_body_out_msg => csv
                    .add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([(
                        "theta_rad".to_string(),
                        format!("theta_P{k}_rad"),
                    )])),
                &rotation_profilers[index].spinning_body_out_msg => &mut angle_logs[index].input,
                &truss_rotation_out[index] => csv.add_source::<PrescribedRotationMsg>(
                    CsvSourceConfig::columns(
                        vector_columns("omega_pm_p_radps", &format!("omega_P{k}M_P{k}_rad_per_s"))
                            .into_iter()
                            .chain(vector_columns(
                                "omega_prime_pm_p_radps2",
                                &format!("omegaPrime_P{k}M_P{k}_rad_per_s2"),
                            )),
                    ),
                ),
                &truss_rotation_out[index] => &mut rotation_logs[index].input,
            );
        }
        for (output, (log, panel_number)) in
            logged_panels.iter().zip(panel_logs.iter_mut().zip([1, 3]))
        {
            connect!(&simulation,
                output => csv.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
                    ("theta_rad".to_string(), format!("theta_S{panel_number}_rad")),
                    ("theta_dot_radps".to_string(), format!("thetaDot_S{panel_number}_rad_per_s")),
                ])),
                output => &mut log.input,
            );
        }
        let [rotation_1, rotation_2] = &mut rotation_profilers;
        let [translation_1, translation_2] = &mut translation_profilers;
        let [angle_1, angle_2] = &mut angle_logs;
        let [truss_log_1, truss_log_2] = &mut rotation_logs;
        let [panel_log_1, panel_log_3] = &mut panel_logs;
        schedule! { simulation,
            "scObject" => &mut spacecraft, period=DYN_STEP_NANOS, priority=100;
            "prescribedRotation1DOF_1" => rotation_1, period=DYN_STEP_NANOS, priority=50;
            "prescribedRotation1DOF_2" => rotation_2, period=DYN_STEP_NANOS, priority=50;
            "prescribedLinearTranslation_1" => translation_1, period=DYN_STEP_NANOS, priority=50;
            "prescribedLinearTranslation_2" => translation_2, period=DYN_STEP_NANOS, priority=50;
            "csv" => &mut csv, period=DATA_STEP_NANOS, priority=0;
            "scStateLog" => &mut state_log, period=DATA_STEP_NANOS, priority=0;
            "diagnosticsLog" => &mut diagnostics_log, period=DATA_STEP_NANOS, priority=0;
            "profiler1Log" => angle_1, period=DATA_STEP_NANOS, priority=0;
            "profiler2Log" => angle_2, period=DATA_STEP_NANOS, priority=0;
            "truss1Log" => truss_log_1, period=DATA_STEP_NANOS, priority=0;
            "truss2Log" => truss_log_2, period=DATA_STEP_NANOS, priority=0;
            "panel1Log" => panel_log_1, period=DATA_STEP_NANOS, priority=0;
            "panel3Log" => panel_log_3, period=DATA_STEP_NANOS, priority=0;
        }
        simulation.run_for(DURATION_NANOS);
    }
    csv.flush().expect("flush scenario CSV");

    let messages =
        |log: MessageLog<HingedRigidBodyMsg>| log.values.iter().cloned().collect::<Vec<_>>();
    let [angle_1, angle_2] = angle_logs;
    let [truss_log_1, truss_log_2] = rotation_logs;
    let [panel_log_1, panel_log_3] = panel_logs;
    Results {
        csv_path,
        states: state_log.into_samples(),
        diagnostics: diagnostics_log.values.iter().cloned().collect(),
        truss_angles: [messages(angle_1), messages(angle_2)],
        truss_rotations: [truss_log_1, truss_log_2]
            .map(|log| log.values.iter().cloned().collect::<Vec<_>>()),
        panels: [messages(panel_log_1), messages(panel_log_3)],
    }
}

fn main() {
    let results = run();
    let (_, last) = results.states.last().expect("samples");
    println!(
        "wrote {} (final truss angles {:.3} and {:.3} deg, final hub rate {:?} rad/s)",
        results.csv_path.display(),
        results.truss_angles[0]
            .last()
            .expect("samples")
            .theta_rad
            .to_degrees(),
        results.truss_angles[1]
            .last()
            .expect("samples")
            .theta_rad
            .to_degrees(),
        last.omega_radps.as_slice(),
    );
}

#[cfg(test)]
mod tests {
    use super::{THETA_REF_RAD, prescribed::final_relative_momentum, run};

    /// Upstream runs the scenario in `test_scenarioTest.py` without checking
    /// values. The figures' story, checked here: both trusses end at rest at
    /// `±45°`; the panels deflect while the trusses turn; the hub
    /// counter-rotates about the trusses' axis; and the total angular
    /// momentum, zero at the start, is zero again once the trusses have
    /// stopped (to 1e-9 of the hub's largest momentum).
    #[test]
    fn trusses_turn_and_the_panels_flex() {
        let results = run();
        assert_eq!(results.states.len(), 901);
        for (angles, reference) in results
            .truss_angles
            .iter()
            .zip([THETA_REF_RAD, -THETA_REF_RAD])
        {
            let last = angles.last().expect("samples");
            assert!(
                (last.theta_rad - reference).abs() < 1.0e-9,
                "{}",
                last.theta_rad
            );
            assert_eq!(last.theta_dot_radps, 0.0);
        }
        for rotation in &results.truss_rotations {
            assert!(rotation.last().expect("samples").omega_pm_p_radps.norm() < 1.0e-12);
        }
        for panel in &results.panels {
            let deflection = panel
                .iter()
                .map(|sample| sample.theta_rad.abs())
                .fold(0.0, f64::max);
            assert!(deflection > 1.0e-6, "panel deflection {deflection:e} rad");
        }
        let hub_rate_x = results
            .states
            .iter()
            .map(|(_, state)| state.omega_radps.x.abs())
            .fold(0.0, f64::max);
        assert!(hub_rate_x > 1.0e-4, "hub rate {hub_rate_x:e} rad/s");
        let hub_inertia_x = 15_000.0 * (8.0 * 8.0 + 20.0 * 20.0) / 12.0;
        let momentum = final_relative_momentum(&results.diagnostics, hub_inertia_x * hub_rate_x);
        assert!(momentum < 1.0e-9, "final angular momentum {momentum:e}");
    }
}
