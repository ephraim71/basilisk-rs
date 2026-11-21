//! The two-body orbit of upstream `scenarioCompareOrbit.py`, built for each
//! engine, shared by `scenario_compare_orbit` and `sweep_orbit_dt`.
//!
//! The BSM side is a 750 kg [`Spacecraft`] under a point-mass Earth. Upstream
//! sets `pointMassTranslationalOnly = True`, which skips the rotational
//! equations; basilisk-rs's spacecraft has no such switch, but with zero
//! initial rate and no torque its attitude stays exactly at rest, so the
//! translation it integrates is the same. The MuJoCo side is one body on
//! three orthogonal slide joints with upstream's `NBodyGravity` setup.

#![allow(dead_code)]

use basilisk_rs::messages::SpacecraftStateMsg;
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::{connect, schedule};
use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use super::dyncomp::{recorder_period_nanos, sec2nano_u64, timed_run};
use super::grav_factory::custom_point_mass;
use super::message_log::MessageLog;
use super::orbit_tools::kepler_state;

/// Upstream `mass` \[kg\].
pub const MASS_KG: f64 = 750.0;

/// Upstream `ORBIT_SCENE_XML`: one body free to translate along three
/// axes, collisions and constraints off.
pub const ORBIT_SCENE_XML: &str = r#"
<mujoco>
  <option>
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="sat">
      <joint name="sat_x" axis="1 0 0" type="slide"/>
      <joint name="sat_y" axis="0 1 0" type="slide"/>
      <joint name="sat_z" axis="0 0 1" type="slide"/>
      <inertial pos="0 0 0" mass="750" diaginertia="900 800 600"/>
    </body>
  </worldbody>
</mujoco>
"#;

/// Upstream `initialOrbitState`: the elements and their inertial state.
pub fn initial_orbit_state(mu: f64) -> (Vector3<f64>, Vector3<f64>, ClassicElements) {
    let degrees = std::f64::consts::PI / 180.0;
    let elements = ClassicElements {
        semi_major_axis: 7000.0e3,
        eccentricity: 0.01,
        inclination_rad: 33.3 * degrees,
        raan_rad: 48.2 * degrees,
        arg_periapsis_rad: 347.8 * degrees,
        true_anomaly_rad: 85.3 * degrees,
        ..Default::default()
    };
    let (r, v) = elements_to_state(mu, &elements);
    (r, v, elements)
}

/// Upstream `keplerTruth`: the analytic position at each sample time \[m\].
pub fn kepler_truth(mu: f64, elements: &ClassicElements, times_s: &[f64]) -> Vec<Vector3<f64>> {
    times_s
        .iter()
        .map(|time| kepler_state(mu, elements, *time).0)
        .collect()
}

/// A recorded position and velocity history.
#[derive(Clone, Debug, Default)]
pub struct OrbitHistory {
    pub times_nanos: Vec<u64>,
    pub positions_m: Vec<Vector3<f64>>,
    pub velocities_mps: Vec<Vector3<f64>>,
}

impl OrbitHistory {
    fn from_log(log: MessageLog<SpacecraftStateMsg>) -> Self {
        Self {
            positions_m: log.values.iter().map(|state| state.position_m).collect(),
            velocities_mps: log.values.iter().map(|state| state.velocity_mps).collect(),
            times_nanos: log.times_nanos,
        }
    }
}

pub fn epoch() -> Epoch {
    Epoch::from_gregorian_utc_at_midnight(2019, 1, 1)
}

/// Upstream `buildBSM` + `runBSM`: the history (with `record`) and the
/// propagation seconds.
pub fn run_bsm(
    mass_kg: f64,
    mu: f64,
    dt_s: f64,
    tf_s: f64,
    record_dt_s: f64,
    record: bool,
) -> (OrbitHistory, f64) {
    let step = sec2nano_u64(dt_s);
    let (r, v, _) = initial_orbit_state(mu);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
        integration_step_nanos: step,
        initial_position_m: r,
        initial_velocity_mps: v,
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::zeros(),
        integrator: None,
    });
    spacecraft
        .add_grav_body(custom_point_mass("earth", mu, true))
        .expect("one central body");
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
    (OrbitHistory::from_log(log), seconds)
}

#[cfg(feature = "mujoco")]
#[allow(unused_imports)]
pub use mujoco_side::run_mujoco;

#[cfg(feature = "mujoco")]
mod mujoco_side {
    use super::*;
    use crate::dyncomp::{PreparedScene, add_earth_gravity_to_scene};
    use basilisk_rs::integrators::Rk4;
    use basilisk_rs::mujoco_dynamics::MjScene;

    /// Upstream `buildMujoco` + `runMujoco`: the `sat` center-of-mass
    /// history and the propagation seconds.
    pub fn run_mujoco(
        mu: f64,
        dt_s: f64,
        tf_s: f64,
        record_dt_s: f64,
        record: bool,
    ) -> (OrbitHistory, f64) {
        let step = sec2nano_u64(dt_s);
        let mut scene = MjScene::new(ORBIT_SCENE_XML).expect("orbit scene");
        scene.name = "scMujoco".to_string();
        scene.extra_eom_call = true;
        scene.set_integrator(Rk4);
        add_earth_gravity_to_scene(&mut scene, mu);
        let com = scene.body("sat").center_of_mass().state_out_msg.clone();
        let (r, v, _) = initial_orbit_state(mu);
        let mut scene = PreparedScene::new(scene, move |scene| {
            let mut sat = scene.body_mut("sat");
            sat.set_position(r);
            sat.set_velocity(v);
        });
        let mut log = MessageLog::<SpacecraftStateMsg>::default();
        let seconds = {
            let mut simulation = Simulation::new(epoch(), false);
            schedule! { simulation, "scMujoco" => &mut scene, period = step, priority = 1; }
            if record {
                connect!(&simulation, &com => &mut log.input);
                simulation.add_module(
                    "recorder",
                    &mut log,
                    recorder_period_nanos(dt_s, record_dt_s),
                    0,
                );
            }
            timed_run(&mut simulation, sec2nano_u64(tf_s))
        };
        (OrbitHistory::from_log(log), seconds)
    }
}
