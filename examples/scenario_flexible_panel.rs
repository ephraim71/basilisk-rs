//! An attitude slew of a spacecraft with a flexible solar array: port of
//! upstream `examples/scenarioFlexiblePanel.py`.
//!
//! A 1000 kg, 3 × 3 × 6 m hub carries an 18 m, 100 kg panel split into
//! `numberOfSegments` sub-panels, modelled as one
//! [`SpinningBodyNDofStateEffector`]: per sub-panel a massless bending joint
//! about `x` (`k = 10 N m/rad`, `c = 8 N m s/rad`) carrying the sub-panel on
//! a torsional joint about `y` (`k = 1`, `c = 0.8`). From rest on an
//! 8000 km, `e = 0.1` Earth orbit (with the Sun's gravity), `mrpFeedback`
//! slews the hub to the inertial attitude `sigma_R0N = [0.3, 0.4, 0.5]`
//! through an `extForceTorque`, its gains `P = 2 max(I) / 50 s` and
//! `K = (P / 0.9)^2 / max(I)` from the rigid spacecraft's inertia about B.
//! The dynamics (spacecraft, panel, torque, `simpleNav`) run every 0.1 s
//! and the flight software (`inertial3D`, `attTrackingError`, `mrpFeedback`)
//! every 0.5 s, after the dynamics; 10 minutes.
//!
//! Upstream `run(show_plots, numberOfSegments)` maps to `--segments`
//! (default 5, which upstream's `test_scenarioFlexiblePanel.py` runs).
//!
//! Upstream's `createSpiceInterface` (zero base Earth, epoch
//! 2012 MAY 1 00:28:30) is [`AniseSunEphemeris`], the Sun relative to Earth
//! from DE440s (`just fetch-anise-assets`); the Earth stays at the origin.
//! Upstream's `extForceTorque` reads its torque command in its
//! `UpdateState`, after the spacecraft integrated, for the next step;
//! basilisk-rs's reads it during the integration, so a [`Latch`] scheduled
//! after the spacecraft holds the command it sees.
//!
//! Every joint angle and rate and the attitude tracking error are recorded
//! every dynamics step to
//! `examples/output/scenarios/scenarioFlexiblePanel<segments>.csv` (the
//! tracking error changes every 0.5 s, upstream's flight-software
//! recorder).
//!
//! Upstream's test only checks that the scenario runs. `tests` checks what
//! the figures show: the slew excites the array, the hub reaches the
//! reference, and the damped oscillation of the array decays.

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

use attitude::{att_tracking_error, external_disturbance, scenario_recorder};
use basilisk_rs::dynamics::spinning_body_n_dof_state_effector::{
    SpinningBody, SpinningBodyNDofStateEffector,
};
use basilisk_rs::environment::anise_sun_ephemeris::{AniseSunEphemeris, AniseSunEphemerisConfig};
use basilisk_rs::fsw_algorithms::att_control::mrp_feedback::{MrpFeedback, MrpFeedbackConfig};
use basilisk_rs::fsw_algorithms::att_guidance::inertial_3d::{Inertial3D, Inertial3DConfig};
use basilisk_rs::messages::{
    AttitudeGuidanceMsg, BodyTorqueCommandMsg, HingedRigidBodyMsg, Output, VehicleConfigMsg,
};
use basilisk_rs::sensors::navigation::SimpleNavigation;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::tilde;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::{Body, PLANETARY_SPK};
use hifitime::Epoch;
use latch::Latch;
use message_log::MessageLog;
use multibody::{box_inertia, set_center_of_mass_initial_state};
use nalgebra::{Matrix3, Vector3};

/// Upstream `simTimeStep = macros.sec2nano(0.1)`: the dynamics task.
const DYNAMICS_PERIOD_NANOS: u64 = 100_000_000;
/// Upstream `fswTimeStep = macros.sec2nano(0.5)`.
const FSW_PERIOD_NANOS: u64 = 500_000_000;
/// Upstream `simulationTime = macros.min2nano(10.0)`.
const DURATION_NANOS: u64 = 600_000_000_000;
/// Upstream `inertial3DObj.sigma_R0N`.
const SIGMA_R0N: [f64; 3] = [0.3, 0.4, 0.5];

/// Upstream `geometryClass`.
#[derive(Clone, Copy, Debug)]
struct Geometry {
    number_of_segments: usize,
}

impl Geometry {
    const MASS_HUB: f64 = 1000.0;
    const LENGTH_HUB: f64 = 3.0;
    const WIDTH_HUB: f64 = 3.0;
    const HEIGHT_HUB: f64 = 6.0;
    const LENGTH_PANEL: f64 = 18.0;
    const WIDTH_PANEL: f64 = 3.0;
    const THICKNESS_PANEL: f64 = 0.3;
    const MASS_PANEL: f64 = 100.0;

    fn mass_sub_panel(&self) -> f64 {
        Self::MASS_PANEL / self.number_of_segments as f64
    }

    fn length_sub_panel(&self) -> f64 {
        Self::LENGTH_PANEL / self.number_of_segments as f64
    }

    /// Upstream `IHubPntBc_B`: the hub as a box, `lengthHub` along x and y
    /// and `heightHub` along z (upstream's formula pairs `lengthHub` with
    /// the x axis and `widthHub` with y).
    fn hub_inertia(&self) -> Matrix3<f64> {
        box_inertia(
            Self::MASS_HUB,
            Self::WIDTH_HUB,
            Self::LENGTH_HUB,
            Self::HEIGHT_HUB,
        )
    }
}

/// Upstream `run(...)` parameters.
#[derive(Clone, Copy, Debug)]
struct Config {
    /// Upstream `numberOfSegments`.
    number_of_segments: usize,
    /// Write the CSV file (off in the tests).
    write_csv: bool,
}

/// The recorded histories: the joints every dynamics step, the tracking
/// error every flight-software step.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    /// Bending (even) and torsional (odd) joints, outward from the hub.
    joints: Vec<MessageLog<HingedRigidBodyMsg>>,
    guidance: MessageLog<AttitudeGuidanceMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream `setUpFlexiblePanel`.
fn flexible_panel(geometry: &Geometry) -> SpinningBodyNDofStateEffector {
    let length = geometry.length_sub_panel();
    let mass = geometry.mass_sub_panel();
    let mut panel = SpinningBodyNDofStateEffector::new("panel");
    for index in 0..geometry.number_of_segments {
        panel.add_spinning_body(SpinningBody {
            mass_kg: 0.0,
            inertia_about_com_spinner_kg_m2: Matrix3::zeros(),
            parent_to_spinner0_dcm: Matrix3::identity(),
            center_of_mass_spinner_m: Vector3::new(0.0, length / 2.0, 0.0),
            origin_parent_m: if index == 0 {
                Vector3::new(
                    0.0,
                    Geometry::LENGTH_HUB / 2.0,
                    Geometry::HEIGHT_HUB / 2.0 - Geometry::THICKNESS_PANEL / 2.0,
                )
            } else {
                Vector3::new(0.0, length, 0.0)
            },
            spin_axis_spinner: Vector3::x(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            spring_constant_nm_per_rad: 10.0,
            damping_nm_s_per_rad: 8.0,
        });
        panel.add_spinning_body(SpinningBody {
            mass_kg: mass,
            // Upstream's sub-panel: `lengthSubPanel` along y, `widthSubPanel`
            // along x, `thicknessSubPanel` along z.
            inertia_about_com_spinner_kg_m2: box_inertia(
                mass,
                Geometry::WIDTH_PANEL,
                length,
                Geometry::THICKNESS_PANEL,
            ),
            parent_to_spinner0_dcm: Matrix3::identity(),
            center_of_mass_spinner_m: Vector3::new(0.0, length / 2.0, 0.0),
            origin_parent_m: Vector3::zeros(),
            spin_axis_spinner: Vector3::y(),
            theta_init_rad: 0.0,
            theta_dot_init_radps: 0.0,
            spring_constant_nm_per_rad: 1.0,
            damping_nm_s_per_rad: 0.8,
        });
    }
    panel
}

/// Upstream `setUpControl`'s `IHubPntB_B`: the hub and the undeformed panel
/// (as one 18 m plate) about B.
fn rigid_inertia_about_b(geometry: &Geometry) -> Matrix3<f64> {
    let panel = box_inertia(
        Geometry::MASS_PANEL,
        Geometry::WIDTH_PANEL,
        Geometry::LENGTH_PANEL,
        Geometry::THICKNESS_PANEL,
    );
    let r_scb_b = Vector3::new(
        0.0,
        Geometry::LENGTH_HUB / 2.0 + Geometry::LENGTH_PANEL / 2.0,
        Geometry::HEIGHT_HUB / 2.0 - Geometry::THICKNESS_PANEL / 2.0,
    );
    geometry.hub_inertia() + panel - Geometry::MASS_PANEL * tilde(r_scb_b) * tilde(r_scb_b)
}

fn run(config: Config) -> Results {
    let geometry = Geometry {
        number_of_segments: config.number_of_segments,
    };
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
        mass_kg: Geometry::MASS_HUB,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: geometry.hub_inertia(),
        integration_step_nanos: DYNAMICS_PERIOD_NANOS,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
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

    let panel = flexible_panel(&geometry);
    let mut external_torque = external_disturbance(Vector3::zeros());
    let mut torque_latch = Latch::<BodyTorqueCommandMsg>::default();
    let mut navigation = SimpleNavigation::new("SimpleNavigation");
    let mut inertial_reference = Inertial3D::new(Inertial3DConfig {
        name: "inertial3D".to_string(),
        sigma_r0n: Vector3::from(SIGMA_R0N),
    });
    let mut tracking_error = att_tracking_error(Vector3::zeros());

    // Upstream `setUpControl`: gains from the rigid spacecraft's largest
    // inertia, `decayTime = 50`, `xi = 0.9`; no integral feedback.
    let inertia_about_b = rigid_inertia_about_b(&geometry);
    let max_inertia = inertia_about_b.max();
    let p = 2.0 * max_inertia / 50.0;
    let mut controller = MrpFeedback::new(MrpFeedbackConfig {
        name: "mrpFeedback".to_string(),
        k: (p / 0.9).powi(2) / max_inertia,
        ki: 0.0,
        p,
        integral_limit: 0.0,
        known_torque_body_nm: Vector3::zeros(),
        control_law_type: 0,
    });
    let vehicle_config_msg = Output::new(VehicleConfigMsg {
        inertia_about_point_b_kg_m2: inertia_about_b,
        ..Default::default()
    });

    let mut joint_logs: Vec<MessageLog<HingedRigidBodyMsg>> = (0..2 * geometry.number_of_segments)
        .map(|_| MessageLog::default())
        .collect();
    let mut guidance_log = MessageLog::<AttitudeGuidanceMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(
        &format!("scenarioFlexiblePanel{}", geometry.number_of_segments),
        config.write_csv,
    );

    let epoch = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
    let mut simulation = Simulation::new(epoch, false);
    for (index, (output, log)) in panel
        .spinning_body_out
        .iter()
        .zip(joint_logs.iter_mut())
        .enumerate()
    {
        let (name, segment) = if index % 2 == 0 {
            ("theta", index / 2 + 1)
        } else {
            ("beta", index / 2 + 1)
        };
        connect!(&simulation,
            output => &mut log.input,
            output => recorder.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
                ("theta_rad".to_string(), format!("{name}{segment}_rad")),
                ("theta_dot_radps".to_string(), format!("{name}{segment}Dot_rad_per_s")),
            ])),
        );
    }
    connect!(&simulation,
        &sun_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("sun").expect("sun"),
        &controller.cmd_torque_out_msg => &mut torque_latch.input,
        &torque_latch.output => &mut external_torque.cmd_torque_in_msg,
        &spacecraft.state_out => &mut navigation.spacecraft_state_in_msg,
        &navigation.attitude_out_msg => &mut tracking_error.att_nav_in_msg,
        &inertial_reference.att_ref_out_msg => &mut tracking_error.att_ref_in_msg,
        &tracking_error.att_guid_out_msg => &mut controller.guid_in_msg,
        &vehicle_config_msg => &mut controller.veh_config_in_msg,
        &tracking_error.att_guid_out_msg => &mut guidance_log.input,
        &tracking_error.att_guid_out_msg => recorder.add_source::<AttitudeGuidanceMsg>(
            CsvSourceConfig::columns(
                vector_columns("sigma_br", "sigma_BR")
                    .into_iter()
                    .chain(vector_columns("omega_br_b_radps", "omega_BR_B_rad_per_s")),
            ),
        ),
    );
    spacecraft.add_state_effector(panel);
    spacecraft.add_dynamic_effector(external_torque);
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    schedule! { simulation,
        // Dynamics process.
        "spiceObject" => &mut sun_ephemeris, period=DYNAMICS_PERIOD_NANOS, priority=110;
        "scObject" => &mut spacecraft, period=DYNAMICS_PERIOD_NANOS, priority=100;
        "externalDisturbance" => &mut torque_latch, period=DYNAMICS_PERIOD_NANOS, priority=95;
        "SimpleNavigation" => &mut navigation, period=DYNAMICS_PERIOD_NANOS, priority=90;
        // Flight-software process, after the dynamics at shared ticks.
        "inertial3D" => &mut inertial_reference, period=FSW_PERIOD_NANOS, priority=80;
        "attErrorInertial3D" => &mut tracking_error, period=FSW_PERIOD_NANOS, priority=70;
        "mrpFeedback" => &mut controller, period=FSW_PERIOD_NANOS, priority=60;
        "guidance_log" => &mut guidance_log, period=FSW_PERIOD_NANOS, priority=0;
    }
    for log in &mut joint_logs {
        simulation.add_module("joint_log", log, DYNAMICS_PERIOD_NANOS, 0);
    }
    if config.write_csv {
        simulation.add_module("recorder", &mut recorder, DYNAMICS_PERIOD_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        joints: joint_logs,
        guidance: guidance_log,
        csv_path,
    }
}

fn main() {
    let mut cli = Cli::from_env("scenario_flexible_panel [--segments N]");
    let config = Config {
        number_of_segments: cli.value("segments", 5),
        write_csv: true,
    };
    cli.finish();

    let results = run(config);
    println!(
        "wrote {} (final |sigma_BR| = {:.3e})",
        results.csv_path.expect("CSV written").display(),
        results.guidance.last().sigma_br.norm(),
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn upstream_test_run() -> Results {
        // Upstream `test_scenarioFlexiblePanel.py`: `run(False, 5)`.
        run(Config {
            number_of_segments: 5,
            write_csv: false,
        })
    }

    fn peak(log: &MessageLog<HingedRigidBodyMsg>, since_nanos: u64) -> f64 {
        log.times_nanos
            .iter()
            .zip(&log.values)
            .filter(|(time, _)| **time >= since_nanos)
            .fold(0.0_f64, |max, (_, joint)| max.max(joint.theta_rad.abs()))
    }

    /// The slew starts from the inertial frame toward `sigma_R0N`; the
    /// controller (a 50 s decay) removes all but about 1 % of the error,
    /// what is left a slow wobble with the array's first bending mode
    /// (period about 3 minutes, lightly damped).
    #[test]
    fn the_hub_slews_to_the_reference() {
        let results = upstream_test_run();
        assert_eq!(results.joints.len(), 10);
        let first = results.guidance.values[0].sigma_br;
        assert!(
            (first + Vector3::from(SIGMA_R0N)).norm() < 1.0e-12,
            "{first}"
        );
        let late = results
            .guidance
            .since(DURATION_NANOS - 120_000_000_000)
            .fold(0.0_f64, |max, guidance| max.max(guidance.sigma_br.norm()));
        assert!(late < 0.015 * first.norm(), "late |sigma_BR| {late}");
        assert!(results.guidance.last().omega_br_b_radps.norm() < 1.0e-3);
    }

    /// The slew bends and twists every sub-panel, the joints nearest the
    /// hub the most, and the dampers wear the oscillation down.
    #[test]
    fn the_slew_excites_the_array_which_then_decays() {
        let results = upstream_test_run();
        for (index, log) in results.joints.iter().enumerate() {
            let excited = peak(log, 0);
            assert!(
                excited > 1.0e-3_f64.to_radians(),
                "joint {index}: peak {excited} rad"
            );
            let late = peak(log, DURATION_NANOS - 120_000_000_000);
            assert!(
                late < 0.5 * excited,
                "joint {index}: {late} of {excited} rad late"
            );
        }
        for parity in [0, 1] {
            let peaks: Vec<f64> = results
                .joints
                .iter()
                .skip(parity)
                .step_by(2)
                .map(|log| peak(log, 0))
                .collect();
            assert!(peaks[0] > peaks[4], "{peaks:?}");
        }
    }
}
