//! The torqued rigid body of upstream `scenarioCompareTorque.py`, built for
//! each engine, shared by `scenario_compare_torque`, `sweep_torque_artifact`
//! and `sweep_torque_mechanism_checks`.
//!
//! The BSM side is a [`Spacecraft`] hub with an [`ExtForceTorque`] reading a
//! constant body torque command; the MuJoCo side is one free body with an
//! `MJTorqueActuator` at its origin site, whose constant `torque_S` is a
//! body-frame torque, and `highOrderAttitudeIntegration` on. Each case sets
//! its initial position and velocity and whether point-mass Earth gravity
//! acts (upstream's orbiting, at-rest, drifting and quiet cases).

#![allow(dead_code)]

use basilisk_rs::dynamics::ext_force_torque::{ExtForceTorque, ExtForceTorqueConfig};
use basilisk_rs::messages::{BodyTorqueCommandMsg, Output, SpacecraftStateMsg};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::{connect, schedule};
use nalgebra::{Matrix3, Vector3};

use super::dyncomp::{recorder_period_nanos, sec2nano_u64, timed_run};
use super::dyncomp_orbit::{epoch, initial_orbit_state};
use super::grav_factory::custom_point_mass;
use super::message_log::MessageLog;

/// Upstream `MASS` \[kg\].
pub const MASS_KG: f64 = 750.0;
/// Upstream `INERTIA_DIAG` \[kg m^2\], principal, about the center of mass.
pub const INERTIA_DIAG: [f64; 3] = [900.0, 800.0, 600.0];
/// Upstream `TORQUE_B` \[N m\]: constant body-frame torque.
pub const TORQUE_B: [f64; 3] = [0.2, -0.3, 0.4];
/// Upstream `OMEGA0_B` \[rad/s\]: initial body rate, off the principal axes.
pub const OMEGA0_B: [f64; 3] = [0.02, -0.01, 0.03];
/// Upstream `ANALYTIC_AXIS`: the body z principal axis.
pub const ANALYTIC_AXIS: usize = 2;
/// Upstream `ANALYTIC_TORQUE_B` \[N m\].
pub const ANALYTIC_TORQUE_B: [f64; 3] = [0.0, 0.0, 0.4];
/// Upstream `ANALYTIC_OMEGA0_B` \[rad/s\].
pub const ANALYTIC_OMEGA0_B: [f64; 3] = [0.0, 0.0, 0.03];

/// Upstream `mujocoModel(mass, inertiaDiag)`: one free rigid body.
pub fn mujoco_model(mass_kg: f64, inertia: [f64; 3]) -> String {
    let [ix, iy, iz] = inertia;
    format!(
        r#"
<mujoco>
  <option gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{mass_kg}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
    </body>
  </worldbody>
</mujoco>
"#
    )
}

/// One torqued-body propagation.
#[derive(Clone, Copy, Debug)]
pub struct TorqueCase {
    pub mass_kg: f64,
    /// Constant body torque \[N m\]; `None` adds no torque model at all
    /// (upstream's quiet mechanism-control body).
    pub torque_b: Option<Vector3<f64>>,
    pub omega0_b: Vector3<f64>,
    pub position_m: Vector3<f64>,
    pub velocity_mps: Vector3<f64>,
    /// Point-mass Earth gravity of `mu` \[m^3/s^2\].
    pub gravity_mu: Option<f64>,
}

impl TorqueCase {
    /// Upstream `withGravity=True`: on the Keplerian orbit under Earth gravity.
    pub fn orbiting(mu: f64, torque_b: [f64; 3], omega0_b: [f64; 3]) -> Self {
        let (r, v, _) = initial_orbit_state(mu);
        Self {
            mass_kg: MASS_KG,
            torque_b: Some(Vector3::from(torque_b)),
            omega0_b: Vector3::from(omega0_b),
            position_m: r,
            velocity_mps: v,
            gravity_mu: Some(mu),
        }
    }

    /// Upstream `withGravity=False`: at the origin, at rest, no gravity.
    pub fn at_rest(torque_b: [f64; 3], omega0_b: [f64; 3]) -> Self {
        Self::drifting(Vector3::zeros(), torque_b, omega0_b)
    }

    /// Upstream `runBSMDrift`/`runMujocoDrift`: no gravity, a constant
    /// inertial drift velocity from the origin.
    pub fn drifting(velocity_mps: Vector3<f64>, torque_b: [f64; 3], omega0_b: [f64; 3]) -> Self {
        Self {
            mass_kg: MASS_KG,
            torque_b: Some(Vector3::from(torque_b)),
            omega0_b: Vector3::from(omega0_b),
            position_m: Vector3::zeros(),
            velocity_mps,
            gravity_mu: None,
        }
    }
}

/// A recorded attitude history.
#[derive(Clone, Debug, Default)]
pub struct AttitudeHistory {
    pub times_nanos: Vec<u64>,
    pub sigma_bn: Vec<Vector3<f64>>,
    pub omega_radps: Vec<Vector3<f64>>,
}

impl AttitudeHistory {
    pub fn from_log(log: &MessageLog<SpacecraftStateMsg>) -> Self {
        Self {
            times_nanos: log.times_nanos.clone(),
            sigma_bn: log.values.iter().map(|state| state.sigma_bn).collect(),
            omega_radps: log.values.iter().map(|state| state.omega_radps).collect(),
        }
    }
}

/// Upstream `buildBSM` + `runBSM` (and `runBSMDrift`): the hub history
/// (with `record`) and the propagation seconds.
pub fn run_bsm(
    case: &TorqueCase,
    dt_s: f64,
    tf_s: f64,
    record_dt_s: f64,
    record: bool,
) -> (AttitudeHistory, f64) {
    let step = sec2nano_u64(dt_s);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: case.mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::from(INERTIA_DIAG)),
        integration_step_nanos: step,
        initial_position_m: case.position_m,
        initial_velocity_mps: case.velocity_mps,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: case.omega0_b,
        integrator: None,
    });
    if let Some(mu) = case.gravity_mu {
        spacecraft
            .add_grav_body(custom_point_mass("earth", mu, true))
            .expect("one central body");
    }
    let torque_msg = Output::new(BodyTorqueCommandMsg {
        torque_request_body_nm: case.torque_b.unwrap_or_else(Vector3::zeros),
        ..Default::default()
    });
    let mut ext_torque = ExtForceTorque::new(ExtForceTorqueConfig {
        name: "extTorque".to_string(),
        ..Default::default()
    });
    torque_msg.connect_to(&mut ext_torque.cmd_torque_in_msg);
    spacecraft.add_dynamic_effector(ext_torque);

    let mut log = MessageLog::<SpacecraftStateMsg>::default();
    let seconds = {
        let mut simulation = Simulation::new(epoch(), false);
        if record {
            connect!(&simulation, &spacecraft.state_out => &mut log.input);
        }
        schedule! { simulation, "scBSM" => &mut spacecraft, period = step, priority = 10; }
        if record {
            simulation.add_module(
                "recorder",
                &mut log,
                recorder_period_nanos(dt_s, record_dt_s),
                0,
            );
        }
        timed_run(&mut simulation, sec2nano_u64(tf_s))
    };
    (AttitudeHistory::from_log(&log), seconds)
}

#[cfg(feature = "mujoco")]
#[allow(unused_imports)]
pub use mujoco_side::{MujocoTorqueRun, run_mujoco};

#[cfg(feature = "mujoco")]
mod mujoco_side {
    use super::*;
    use crate::dyncomp::{PreparedScene, add_earth_gravity_to_scene};
    use basilisk_rs::integrators::Rk4;
    use basilisk_rs::messages::MjJointReactionsMsg;
    use basilisk_rs::mujoco_dynamics::MjScene;
    use basilisk_rs::mujoco_dynamics::mj_joint_reaction_forces::MjJointReactionForces;

    /// A MuJoCo propagation: the hub-origin history, the propagation
    /// seconds and, when captured, the first recorded `biasForces`.
    pub struct MujocoTorqueRun {
        pub history: AttitudeHistory,
        pub seconds: f64,
        pub initial_bias_forces: Option<Vec<f64>>,
    }

    /// Upstream `buildMujoco` + `runMujoco` (and `runMujocoDrift`,
    /// `quietMujocoRun` with `captureBias` and an `MJJointReactionForces`).
    pub fn run_mujoco(
        case: &TorqueCase,
        dt_s: f64,
        tf_s: f64,
        record_dt_s: f64,
        record: bool,
        capture_bias: bool,
    ) -> MujocoTorqueRun {
        let step = sec2nano_u64(dt_s);
        let mut scene =
            MjScene::new(&mujoco_model(case.mass_kg, INERTIA_DIAG)).expect("torque scene");
        scene.name = "scMujoco".to_string();
        scene.extra_eom_call = true;
        scene.set_high_order_attitude_integration(true);
        scene.set_integrator(Rk4);
        if let Some(mu) = case.gravity_mu {
            add_earth_gravity_to_scene(&mut scene, mu);
        }
        let origin = scene.body("hub").origin().name().to_string();
        let torque_msg = Output::new(BodyTorqueCommandMsg {
            torque_request_body_nm: case.torque_b.unwrap_or_else(Vector3::zeros),
            ..Default::default()
        });
        if case.torque_b.is_some() {
            torque_msg.connect_to(
                &mut scene
                    .add_torque_actuator("bodyTorque", &origin)
                    .torque_in_msg,
            );
        }
        let origin_state = scene.body("hub").origin().state_out_msg.clone();
        let mut reaction_forces = capture_bias.then(|| MjJointReactionForces::new(&scene));
        let case = *case;
        let mut scene = PreparedScene::new(scene, move |scene| {
            let mut hub = scene.body_mut("hub");
            hub.set_position(case.position_m);
            hub.set_velocity(case.velocity_mps);
            hub.set_attitude(Vector3::zeros());
            hub.set_attitude_rate(case.omega0_b);
        });
        let mut log = MessageLog::<SpacecraftStateMsg>::default();
        let mut bias_log = MessageLog::<MjJointReactionsMsg>::default();
        let seconds = {
            let mut simulation = Simulation::new(epoch(), false);
            if record {
                connect!(&simulation, &origin_state => &mut log.input);
            }
            if let Some(reaction_forces) = &reaction_forces {
                connect!(&simulation, &reaction_forces.reaction_forces_out_msg => &mut bias_log.input);
            }
            schedule! { simulation, "scMujoco" => &mut scene, period = step, priority = 1; }
            if let Some(reaction_forces) = reaction_forces.as_mut() {
                simulation.add_module("jointReactionForces", reaction_forces, step, 0);
                simulation.add_module("biasRecorder", &mut bias_log, step, -1);
            }
            if record {
                simulation.add_module(
                    "recorder",
                    &mut log,
                    recorder_period_nanos(dt_s, record_dt_s),
                    0,
                );
            }
            timed_run(&mut simulation, sec2nano_u64(tf_s))
        };
        MujocoTorqueRun {
            history: AttitudeHistory::from_log(&log),
            seconds,
            initial_bias_forces: capture_bias.then(|| {
                bias_log
                    .values
                    .first()
                    .expect("bias sample")
                    .bias_forces
                    .clone()
            }),
        }
    }
}
