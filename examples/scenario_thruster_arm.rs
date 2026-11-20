//! A thruster at the tip of a two-joint arm, branched onto the arm or
//! approximated on the hub: port of upstream `examples/scenarioThrusterArm.py`.
//!
//! A 400 kg cylindrical hub (2 m across, 4 m tall), tumbling at
//! `[0.005, -0.005, 0.005] rad/s` on a 7500 km altitude, `e = 0.01` Earth
//! orbit, carries a two-segment, 3 m arm: one
//! [`SpinningBodyTwoDofStateEffector`] of two 20 kg segments, both joints
//! about `x` (`k = 50 N m/rad`, `c = 30 N m s/rad`, reference angle zero).
//! A 4.5 N `MOOG_Monarc_5` thruster at the outer segment's tip, pointing
//! along that segment's `z`, fires for 30 s. RK4, 0.01 s steps, 1 minute.
//!
//! Upstream runs the scenario twice and overlays the results:
//!
//! 1. **branched** (`addToSpacecraftSubcomponent(..., arm, 2)`): the
//!    thruster is attached to the outer segment, so its force and torque
//!    turn with the segment and load the joints;
//! 2. **hub-direct** (`addToSpacecraft`): the thruster is fixed to the hub
//!    at the tip's initial position and direction.
//!
//! For the branched case the thruster is attached to the outer segment
//! ([`SpinningBodyTwoDofStateEffector::add_dynamic_effector`]), which drives
//! it as upstream's branching parent does: its command is read before each
//! step, its firing decided at every integrator stage, and it computes its
//! force and torque in the segment's frame `S2` about that frame's origin
//! (thruster at `[0, 1.5, 0] m`, thrust along `[0, 0, 1]` of `S2`).
//!
//! Each case records the joint angles and rates and the hub velocity, rate
//! and accumulated ΔV every step to
//! `examples/output/scenarios/scenarioThrusterArm_<branched|direct>.csv`.
//!
//! Upstream's `test_scenarioThrusterArm.py` only checks that both cases
//! run. `tests` checks what upstream's figures show: both cases deliver the
//! same thrust, but the branched one loads the arm's joints directly and
//! bends it the other way, so the hub responses differ.

#[path = "support/attitude.rs"]
mod attitude;
#[path = "support/cli.rs"]
mod cli;
#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/thruster_factory.rs"]
mod thruster_factory;

use std::path::PathBuf;

use attitude::scenario_recorder;
use basilisk_rs::dynamics::spinning_body_two_dof_state_effector::{
    SpinningBodyTwoDofBodyConfig, SpinningBodyTwoDofConfig, SpinningBodyTwoDofStateEffector,
};
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::messages::{
    HingedRigidBodyMsg, Output, SpacecraftStateMsg, ThrusterCommandMsg, ThrusterOutputMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::prv_to_dcm;
use basilisk_rs::{connect, schedule};
use cli::Cli;
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::{box_inertia, set_center_of_mass_initial_state};
use nalgebra::{Matrix3, Vector3};
use thruster_factory::{ThrusterFactory, ThrusterModel};

/// Upstream `simulationTimeStep = macros.sec2nano(0.01)`.
const STEP_NANOS: u64 = 10_000_000;
/// Upstream `simDuration = macros.min2nano(1)`.
const DURATION_NANOS: u64 = 60_000_000_000;
/// Upstream `thrusterOnTime` \[s\].
const THRUSTER_ON_TIME_S: f64 = 30.0;
/// Upstream `massSC`, `diameter`, `height`.
const MASS_SC: f64 = 400.0;
const DIAMETER: f64 = 2.0;
const HEIGHT: f64 = 4.0;
/// Upstream `armMass`, `armLength`, `armWidth`, `armThickness`.
const ARM_MASS: f64 = 20.0;
const ARM_LENGTH: f64 = 1.5 * DIAMETER;
const ARM_WIDTH: f64 = 0.1;
const ARM_THICKNESS: f64 = 0.1;

/// Which upstream case to run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Attachment {
    /// `useEffectorBranching = True`.
    Branched,
    /// `useEffectorBranching = False`.
    HubDirect,
}

impl Attachment {
    fn suffix(self) -> &'static str {
        match self {
            Self::Branched => "branched",
            Self::HubDirect => "direct",
        }
    }
}

/// The recorded histories of one case, every step.
#[cfg_attr(not(test), allow(dead_code))]
struct Results {
    theta1: MessageLog<HingedRigidBodyMsg>,
    theta2: MessageLog<HingedRigidBodyMsg>,
    states: MessageLog<SpacecraftStateMsg>,
    thrust: MessageLog<ThrusterOutputMsg>,
    csv_path: Option<PathBuf>,
}

/// Upstream `setupSpinningBody`: two 20 kg, 1.5 m segments, both joints
/// about `x`, the first at the hub's bottom rim.
fn arm() -> SpinningBodyTwoDofStateEffector {
    let segment_length = ARM_LENGTH / 2.0;
    // Upstream's `armI`: `segmentLength` along y, `armWidth` along x.
    let inertia = box_inertia(ARM_MASS, ARM_WIDTH, segment_length, ARM_THICKNESS);
    let body = || SpinningBodyTwoDofBodyConfig {
        mass_kg: ARM_MASS,
        spring_constant_nm_per_rad: 50.0,
        damping_nm_s_per_rad: 30.0,
        theta_init_rad: 0.0,
        theta_dot_init_radps: 0.0,
        center_of_mass_spinner_m: Vector3::new(0.0, ARM_LENGTH / 4.0, 0.0),
        spin_axis_spinner: Vector3::x(),
        inertia_about_com_spinner_kg_m2: inertia,
    };
    SpinningBodyTwoDofStateEffector::new(SpinningBodyTwoDofConfig {
        body1: body(),
        body2: body(),
        spinner1_origin_body_m: Vector3::new(
            0.0,
            DIAMETER / 2.0,
            -HEIGHT / 2.0 + ARM_THICKNESS / 2.0,
        ),
        spinner2_origin_spinner1_m: Vector3::new(0.0, ARM_LENGTH / 2.0, 0.0),
        body_to_spinner1_dcm: Matrix3::identity(),
        spinner1_to_spinner2_dcm: Matrix3::identity(),
        ..SpinningBodyTwoDofConfig::new("arm")
    })
}

/// Upstream `initialTipFrameInB`: the outer segment's tip and its `z` axis
/// in B at the initial joint angles.
fn initial_tip_frame_in_b(arm: &SpinningBodyTwoDofConfig) -> (Vector3<f64>, Vector3<f64>) {
    let dcm_s1s10 = prv_to_dcm(arm.body1.theta_init_rad * arm.body1.spin_axis_spinner);
    let dcm_s2s20 = prv_to_dcm(arm.body2.theta_init_rad * arm.body2.spin_axis_spinner);
    let dcm_s1b = dcm_s1s10 * arm.body_to_spinner1_dcm;
    let dcm_s2b = dcm_s2s20 * arm.spinner1_to_spinner2_dcm * dcm_s1b;
    let r_t2s2_s2 = Vector3::new(0.0, ARM_LENGTH / 2.0, 0.0);
    let r_pcb_b = arm.spinner1_origin_body_m
        + dcm_s1b.transpose() * arm.spinner2_origin_spinner1_m
        + dcm_s2b.transpose() * r_t2s2_s2;
    (r_pcb_b, dcm_s2b.transpose() * Vector3::z())
}

fn run_one_case(attachment: Attachment, write_csv: bool) -> Results {
    let earth = Body::Earth;
    let (r_cn_n, v_cn_n) = elements_to_state(
        earth.mu_m3ps2(),
        &ClassicElements {
            semi_major_axis: earth.radius_equator_m() + 7500e3,
            eccentricity: 0.01,
            inclination_rad: 30.0_f64.to_radians(),
            raan_rad: 60.0_f64.to_radians(),
            arg_periapsis_rad: 15.0_f64.to_radians(),
            true_anomaly_rad: 90.0_f64.to_radians(),
            ..Default::default()
        },
    );
    let transverse = MASS_SC / 16.0 * DIAMETER * DIAMETER + MASS_SC / 12.0 * HEIGHT * HEIGHT;
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: MASS_SC,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(
            transverse,
            transverse,
            MASS_SC / 8.0 * DIAMETER * DIAMETER,
        )),
        integration_step_nanos: STEP_NANOS,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::new(0.005, -0.005, 0.005),
        integrator: None,
    });
    spacecraft
        .add_grav_body(earth.point_mass(true))
        .expect("unique central gravity body");

    let mut arm = arm();
    // Upstream's stand-alone `angle1RefMsg`, `angle2RefMsg`: both zero.
    let references = [
        Output::new(HingedRigidBodyMsg::default()),
        Output::new(HingedRigidBodyMsg::default()),
    ];

    let mut factory = ThrusterFactory::default();
    let (position, direction) = match attachment {
        // In the outer segment's frame.
        Attachment::Branched => (Vector3::new(0.0, ARM_LENGTH / 2.0, 0.0), Vector3::z()),
        Attachment::HubDirect => initial_tip_frame_in_b(&arm.config),
    };
    factory.create(ThrusterModel::MoogMonarc5, position, direction, None);
    let mut thruster_config = factory.thrusters[0].dynamic_effector_config();
    thruster_config.name = "armThruster".to_string();
    let mut thruster = ThrusterDynamicEffector::new(thruster_config);
    // Upstream `THRArrayOnTimeCmdMsgPayload(OnTimeRequest=[30])`, written
    // once before the run.
    let command = Output::new(ThrusterCommandMsg {
        on_time_s: THRUSTER_ON_TIME_S,
        ..Default::default()
    });

    let mut theta1_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut theta2_log = MessageLog::<HingedRigidBodyMsg>::default();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut thrust_log = MessageLog::<ThrusterOutputMsg>::default();
    let (mut recorder, csv_path) = scenario_recorder(
        &format!("scenarioThrusterArm_{}", attachment.suffix()),
        write_csv,
    );

    let mut simulation = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2019, 1, 1), false);
    for (input, reference) in arm.spinning_body_ref_in.iter_mut().zip(&references) {
        connect!(&simulation, reference => input);
    }
    connect!(&simulation,
        &command => &mut thruster.command_in,
        &thruster.thruster_out_msg => &mut thrust_log.input,
        &arm.spinning_body_out[0] => &mut theta1_log.input,
        &arm.spinning_body_out[1] => &mut theta2_log.input,
        &spacecraft.state_out => &mut state_log.input,
        &arm.spinning_body_out[0] => recorder.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
            ("theta_rad", "theta1_rad"),
            ("theta_dot_radps", "theta1Dot_rad_per_s"),
        ])),
        &arm.spinning_body_out[1] => recorder.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
            ("theta_rad", "theta2_rad"),
            ("theta_dot_radps", "theta2Dot_rad_per_s"),
        ])),
        &spacecraft.state_out => recorder.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
            vector_columns("velocity_mps", "v_BN_N_m_per_s")
                .into_iter()
                .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s"))
                .chain(vector_columns("accumulated_dv_cn_n_mps", "TotalAccumDV_CN_N_m_per_s")),
        )),
    );
    match attachment {
        // Upstream `addToSpacecraftSubcomponent(..., arm, 2)`.
        Attachment::Branched => {
            arm.add_dynamic_effector(1, thruster);
            spacecraft.add_state_effector(arm);
        }
        Attachment::HubDirect => {
            spacecraft.add_state_effector(arm);
            spacecraft.add_dynamic_effector(thruster);
        }
    }
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    schedule! { simulation,
        "hub" => &mut spacecraft, period=STEP_NANOS, priority=100;
        "theta1Rec" => &mut theta1_log, period=STEP_NANOS, priority=0;
        "theta2Rec" => &mut theta2_log, period=STEP_NANOS, priority=0;
        "scStateRec" => &mut state_log, period=STEP_NANOS, priority=0;
        "thrustRec" => &mut thrust_log, period=STEP_NANOS, priority=0;
    }
    if write_csv {
        simulation.add_module("recorder", &mut recorder, STEP_NANOS, 0);
    }
    simulation.run_for(DURATION_NANOS);
    drop(simulation);
    drop(recorder);

    Results {
        theta1: theta1_log,
        theta2: theta2_log,
        states: state_log,
        thrust: thrust_log,
        csv_path,
    }
}

/// Upstream `run(show_plots)`: the branched case, then the hub-direct one.
fn run(write_csv: bool) -> [Results; 2] {
    [
        run_one_case(Attachment::Branched, write_csv),
        run_one_case(Attachment::HubDirect, write_csv),
    ]
}

fn main() {
    Cli::from_env("scenario_thruster_arm").finish();
    for results in run(true) {
        println!(
            "wrote {} (peak joint angles {:.2} and {:.2} deg)",
            results.csv_path.expect("CSV written").display(),
            peak_angle(&results.theta1).to_degrees(),
            peak_angle(&results.theta2).to_degrees(),
        );
    }
}

fn peak_angle(log: &MessageLog<HingedRigidBodyMsg>) -> f64 {
    log.values
        .iter()
        .fold(0.0_f64, |max, joint| max.max(joint.theta_rad.abs()))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// `|ΔV_CN|` gained over step `index` (logged at `index + 1`).
    fn step_dv(results: &Results, index: usize) -> f64 {
        let states = &results.states.values;
        (states[index + 1].accumulated_dv_cn_n_mps - states[index].accumulated_dv_cn_n_mps).norm()
    }

    /// Both thrusters fire 4.5 N for upstream's 30 s on-time: every step of
    /// the burn adds `F dt / m` to the center of mass's non-gravitational
    /// ΔV (its direction turning with the hub, which the off-center thrust
    /// spins up), and none is added after it.
    #[test]
    fn both_cases_deliver_the_thrust() {
        let total_mass = MASS_SC + 2.0 * ARM_MASS;
        let per_step = 4.5 * 0.01 / total_mass;
        for results in run(false) {
            // The thrust each step starts with, logged at the step's end.
            let firing = |seconds: f64| {
                results
                    .thrust
                    .at((seconds * 1e9).round() as u64)
                    .thrust_force_n
            };
            assert_eq!(firing(0.01), 4.5);
            assert_eq!(firing(30.0), 4.5);
            assert_eq!(firing(30.02), 0.0);
            for index in 0..3000 {
                let dv = step_dv(&results, index);
                assert!(
                    (dv - per_step).abs() < 1.0e-5 * per_step,
                    "step {index}: {dv}"
                );
            }
            for index in 3001..results.states.values.len() - 1 {
                assert!(step_dv(&results, index) < 1.0e-6 * per_step);
            }
            // The 18 N m thrust torque about the center of mass spins the
            // hub up from 0.005 to about 0.45 rad/s about its x axis.
            let omega = results.states.last().omega_radps;
            assert!(omega.x > 0.3, "{omega}");
        }
    }

    /// What upstream's comparison shows: the branched thrust pushes the
    /// arm's tip along its own `z`, bending both joints positive during the
    /// first seconds of the burn, while the hub-direct thruster leaves the
    /// arm to lag the spinning-up hub, bending it negative; the hub's rate
    /// differs between the cases from the first step on. After the burn the
    /// spin's centrifugal load holds both arms at about the same deflection.
    #[test]
    fn branching_changes_the_arm_and_hub_response() {
        let [branched, direct] = run(false);
        let at = |log: &MessageLog<HingedRigidBodyMsg>, seconds: f64| {
            log.at((seconds * 1e9).round() as u64).theta_rad
        };
        assert!(at(&branched.theta1, 10.0) > 0.05 && at(&branched.theta2, 10.0) > 0.05);
        assert!(at(&direct.theta1, 10.0) < -0.05 && at(&direct.theta2, 10.0) < -0.01);
        let difference = |seconds: f64| {
            let nanos = (seconds * 1e9).round() as u64;
            (branched.states.at(nanos).omega_radps - direct.states.at(nanos).omega_radps).norm()
        };
        assert!(difference(0.01) > 1.0e-4, "{}", difference(0.01));
        assert!(difference(10.0) > 1.0e-2, "{}", difference(10.0));
        let settled = (branched.theta1.last().theta_rad - direct.theta1.last().theta_rad).abs();
        assert!(
            settled < 0.15 * direct.theta1.last().theta_rad.abs(),
            "{settled}"
        );
    }
}
