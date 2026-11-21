//! Port of upstream `examples/scenarioRoboticArm.py`: a two-link robotic arm
//! on a 1000 kg hub in an 8000 km, e = 0.1 Earth orbit, modeled with
//! [`SpinningBodyNDofStateEffector`](basilisk_rs::dynamics::spinning_body_n_dof_state_effector::SpinningBodyNDofStateEffector).
//! Each link (50 kg, 3 m) sits on a two-axis joint made of a massless body
//! spinning about x̂ followed by the link spinning about ẑ; every joint has a
//! spring–damper (`k = 500` N m/rad, `c = 100` N m s/rad) that tracks a
//! reference profiled by its own
//! [`PrescribedRotation1Dof`](basilisk_rs::device_interface::prescribed_rotation_1dof::PrescribedRotation1Dof)
//! (smoothed bang-bang, 10 s smoothing; 0.01 deg/s² for the first link's
//! joints, 0.02 deg/s² for the second's) from 0 to −30°, 45°, 90° and −20°.
//! The run lasts 5 min at a 0.2 s step.
//!
//! The gravity is the point-mass Earth and Sun, with upstream's
//! `createSpiceInterface` (zero base Earth, epoch 2012 MAY 1 00:28:30) as
//! basilisk-rs's ANISE ephemeris on DE440s (`just fetch-anise-assets`).
//! Upstream's `r_CN_NInit` is the center of mass; [`multibody::set_center_of_mass_initial_state`] converts it to
//! the hub state. Upstream's Vizard setup is not ported.
//!
//! `scenarioRoboticArm.csv` holds at every step the hub attitude and rate
//! (`sigma_BN_i`, `omega_BN_B_rad_per_s_i`) and each joint's angle and rate
//! (`theta<k>_rad`, `theta<k>Dot_rad_per_s`).
//! `plot_scenario_robotic_arm.py` draws upstream's four figures.
//!
//! Upstream's check, ported in `tests`: every joint's final angle is within
//! 0.5° of its commanded angle (`test_scenarioRoboticArm.py`).

#[path = "support/common.rs"]
mod common;
#[path = "support/grav_factory.rs"]
mod grav_factory;
#[path = "support/message_log.rs"]
mod message_log;
#[path = "support/multibody.rs"]
mod multibody;
#[path = "support/prescribed.rs"]
mod prescribed;
#[path = "support/recording.rs"]
mod recording;

use basilisk_rs::device_interface::prescribed_rotation_1dof::PrescribedRotation1Dof;
use basilisk_rs::dynamics::spinning_body_n_dof_state_effector::{
    SpinningBody, SpinningBodyNDofStateEffector,
};
use basilisk_rs::messages::{HingedRigidBodyMsg, Output, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use common::vector_columns;
use grav_factory::Body;
use hifitime::Epoch;
use message_log::MessageLog;
use multibody::set_center_of_mass_initial_state;
use nalgebra::{Matrix3, Vector3};
use prescribed::{hub_at_rest, rotation_profiler};
use recording::scenario_csv;

const STEM: &str = "scenarioRoboticArm";
/// Upstream `dynTimeStep = macros.sec2nano(0.2)`.
const STEP_NANOS: u64 = 200_000_000;
/// Upstream `simulationTime = macros.min2nano(5.0)`.
const DURATION_NANOS: u64 = 300_000_000_000;
/// Upstream's commanded joint angles \[deg\] (`test_scenarioRoboticArm.py`'s
/// `commandedDeg`).
pub const COMMANDED_DEG: [f64; 4] = [-30.0, 45.0, 90.0, -20.0];
/// Upstream `setThetaDDotMax` of each joint's profiler \[deg/s^2\].
const THETA_DDOT_MAX_DEG: [f64; 4] = [0.01, 0.01, 0.02, 0.02];

/// Upstream `geometryClass`.
struct Geometry;

impl Geometry {
    const MASS_HUB: f64 = 1000.0;
    const LENGTH_HUB: f64 = 3.0;
    const WIDTH_HUB: f64 = 3.0;
    const HEIGHT_HUB: f64 = 6.0;
    const MASS_LINK: f64 = 50.0;
    const HEIGHT_LINK: f64 = 3.0;
    const DIAMETER_LINK: f64 = 0.6;
}

pub struct Results {
    pub csv_path: std::path::PathBuf,
    pub states: Vec<(u64, SpacecraftStateMsg)>,
    /// Upstream `thetaLog`: each joint's `spinningBodyOutMsgs` recorder.
    pub joints: Vec<Vec<HingedRigidBodyMsg>>,
}

/// A joint body: upstream `createFirstLink` / `createSecondLink`'s
/// `SpinningBody`, massless for the x̂ joint and the link for the ẑ joint.
fn joint_body(link: bool, origin_parent_m: Vector3<f64>) -> SpinningBody {
    let (mass_kg, inertia, spin_axis) = if link {
        let mass = Geometry::MASS_LINK;
        let radius = Geometry::DIAMETER_LINK / 2.0;
        let height = Geometry::HEIGHT_LINK;
        let transverse = mass / 12.0 * (3.0 * radius * radius + height * height);
        (
            mass,
            Matrix3::from_diagonal(&Vector3::new(
                transverse,
                mass / 12.0 * radius * radius,
                transverse,
            )),
            Vector3::z(),
        )
    } else {
        (0.0, Matrix3::zeros(), Vector3::x())
    };
    SpinningBody {
        mass_kg,
        spring_constant_nm_per_rad: 500.0,
        damping_nm_s_per_rad: 100.0,
        theta_init_rad: 0.0,
        theta_dot_init_radps: 0.0,
        origin_parent_m,
        center_of_mass_spinner_m: Vector3::new(0.0, Geometry::HEIGHT_LINK / 2.0, 0.0),
        spin_axis_spinner: spin_axis,
        inertia_about_com_spinner_kg_m2: inertia,
        parent_to_spinner0_dcm: Matrix3::identity(),
    }
}

pub fn run() -> Results {
    let mass = Geometry::MASS_HUB;
    let (length, width, height) = (
        Geometry::LENGTH_HUB,
        Geometry::WIDTH_HUB,
        Geometry::HEIGHT_HUB,
    );
    let mut spacecraft = hub_at_rest(
        mass,
        Matrix3::from_diagonal(&Vector3::new(
            mass / 12.0 * (length * length + height * height),
            mass / 12.0 * (width * width + height * height),
            mass / 12.0 * (length * length + width * width),
        )),
        STEP_NANOS,
    );

    // The arm: the first link's two-axis joint on the hub's top edge, the
    // second link's at the first link's end.
    let mut arm = SpinningBodyNDofStateEffector::new("spinningBodyEffector");
    let first_joint = Vector3::new(
        0.0,
        length / 2.0,
        height / 2.0 - Geometry::DIAMETER_LINK / 2.0,
    );
    let second_joint = Vector3::new(0.0, Geometry::HEIGHT_LINK, 0.0);
    arm.add_spinning_body(joint_body(false, first_joint));
    arm.add_spinning_body(joint_body(true, Vector3::zeros()));
    arm.add_spinning_body(joint_body(false, second_joint));
    arm.add_spinning_body(joint_body(true, Vector3::zeros()));

    let degrees = std::f64::consts::PI / 180.0;
    let mut profilers: Vec<PrescribedRotation1Dof> = Vec::new();
    for (index, (commanded_deg, theta_ddot_max_deg)) in COMMANDED_DEG
        .into_iter()
        .zip(THETA_DDOT_MAX_DEG)
        .enumerate()
    {
        let mut profiler = rotation_profiler(
            "profiler",
            Vector3::x(),
            theta_ddot_max_deg * degrees,
            arm.spinning_body(index).theta_init_rad,
            0.0,
            10.0,
        );
        let reference = Output::new(HingedRigidBodyMsg {
            theta_rad: commanded_deg * degrees,
            theta_dot_radps: 0.0,
        });
        reference.connect_to(&mut profiler.spinning_body_in_msg);
        profiler
            .spinning_body_out_msg
            .connect_to(&mut arm.spinning_body_ref_in[index]);
        profilers.push(profiler);
    }
    let joint_out = arm.spinning_body_out.clone();
    spacecraft.add_state_effector(arm);

    // Upstream `setUpGravity`: Earth (central) and Sun from SPICE.
    spacecraft
        .add_grav_body(Body::Earth.point_mass(true))
        .expect("unique gravity body");
    spacecraft
        .add_grav_body(Body::Sun.point_mass(false))
        .expect("unique gravity body");
    let mut earth_ephemeris = Body::Earth.ephemeris(Body::Earth);
    let mut sun_ephemeris = Body::Sun.ephemeris(Body::Earth);

    // Upstream `defineInitialConditions`.
    let (position_m, velocity_mps) = elements_to_state(
        Body::Earth.mu_m3ps2(),
        &ClassicElements {
            semi_major_axis: 8.0e6,
            eccentricity: 0.1,
            ..ClassicElements::default()
        },
    );
    set_center_of_mass_initial_state(&mut spacecraft, position_m, velocity_mps);

    let mut csv = scenario_csv(STEM);
    let csv_path = csv.config.output_path.clone();
    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut joint_logs: Vec<MessageLog<HingedRigidBodyMsg>> = Vec::new();
    // Upstream `timeInitString = "2012 MAY 1 00:28:30.0"`.
    let epoch = Epoch::from_gregorian_utc(2012, 5, 1, 0, 28, 30, 0);
    {
        let mut simulation = Simulation::new(epoch, false);
        connect!(&simulation,
            &earth_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("earth").expect("earth"),
            &sun_ephemeris.output_planet_msg => spacecraft.grav_body_input_mut("sun").expect("sun"),
            &spacecraft.state_out => csv.add_source::<SpacecraftStateMsg>(CsvSourceConfig::columns(
                vector_columns("sigma_bn", "sigma_BN")
                    .into_iter()
                    .chain(vector_columns("omega_radps", "omega_BN_B_rad_per_s")),
            )),
            &spacecraft.state_out => &mut state_log.input,
        );
        for (index, output) in joint_out.iter().enumerate() {
            let k = index + 1;
            let mut log = MessageLog::default();
            connect!(&simulation,
                output => csv.add_source::<HingedRigidBodyMsg>(CsvSourceConfig::columns([
                    ("theta_rad".to_string(), format!("theta{k}_rad")),
                    ("theta_dot_radps".to_string(), format!("theta{k}Dot_rad_per_s")),
                ])),
                output => &mut log.input,
            );
            joint_logs.push(log);
        }
        schedule! { simulation,
            "earth_ephemeris" => &mut earth_ephemeris, period=STEP_NANOS, priority=110;
            "sun_ephemeris" => &mut sun_ephemeris, period=STEP_NANOS, priority=110;
            "scObject" => &mut spacecraft, period=STEP_NANOS, priority=100;
        }
        for profiler in &mut profilers {
            simulation.add_module("profiler", profiler, STEP_NANOS, 50);
        }
        schedule! { simulation,
            "csv" => &mut csv, period=STEP_NANOS, priority=0;
            "scLog" => &mut state_log, period=STEP_NANOS, priority=0;
        }
        for log in &mut joint_logs {
            simulation.add_module("thetaLog", log, STEP_NANOS, 0);
        }
        simulation.run_for(DURATION_NANOS);
    }
    csv.flush().expect("flush scenario CSV");

    Results {
        csv_path,
        states: state_log.into_samples(),
        joints: joint_logs
            .iter()
            .map(|log| log.values.iter().cloned().collect())
            .collect(),
    }
}

fn main() {
    let results = run();
    println!(
        "wrote {} (final joint angles {:?} deg)",
        results.csv_path.display(),
        results
            .joints
            .iter()
            .map(|joint| joint.last().expect("samples").theta_rad.to_degrees())
            .collect::<Vec<_>>(),
    );
}

#[cfg(test)]
mod tests {
    use super::{COMMANDED_DEG, run};

    /// `test_scenarioRoboticArm.py`: four joint logs, and each joint's final
    /// angle within 0.5° of its commanded angle.
    #[test]
    fn joints_reach_their_commanded_angles() {
        let results = run();
        assert_eq!(results.joints.len(), COMMANDED_DEG.len());
        for (index, (joint, commanded_deg)) in results.joints.iter().zip(COMMANDED_DEG).enumerate()
        {
            let final_deg = joint.last().expect("samples").theta_rad.to_degrees();
            assert!(
                final_deg.is_finite() && (final_deg - commanded_deg).abs() <= 0.5,
                "Joint {} final angle {final_deg:.3} deg did not reach commanded {commanded_deg:.1} deg.",
                index + 1
            );
        }
    }
}
