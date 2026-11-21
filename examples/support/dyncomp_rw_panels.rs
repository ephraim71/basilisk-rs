//! The hub with three reaction wheels and two hinged solar panels of
//! upstream `scenarioCompareRwPanels.py`, built for each engine; shared by
//! `scenario_compare_rw_panels`, `scenario_compare_orbit_multibody` (in
//! orbit, with or without the panels), `scenario_compare_pareto_rw_panels`
//! (any integrator) and `sweep_rw_bookkeeping_mass` (any MuJoCo wheel mass).
//!
//! * **Wheels.** The BSM balanced wheels add no mass or inertia, so their
//!   inertia tensors are folded into the hub ([`hub_inertia_bsm`]) and the
//!   effector carries only `Js`; the MuJoCo wheels are spinning bodies of a
//!   negligible bookkeeping mass ([`RW_MASS_KG`]) driven by joint motors.
//! * **Panels.** A [`HingedRigidBodyStateEffector`] spring-damper hinge with
//!   the panel center of mass `d` from the hinge is, in MuJoCo, a hinge joint
//!   of the same stiffness and damping with the child's center of mass at
//!   `-d` along its x axis; the positive-x root frame is turned 180° about z
//!   so both panels extend outward.
//!
//! The BSM wheels take one motor-torque input each (basilisk-rs's
//! per-device ports) in place of upstream's `ArrayMotorTorqueMsg`.

#![allow(dead_code)]

use basilisk_rs::connect;
use basilisk_rs::dynamics::hinged_rigid_body_state_effector::{
    HingedRigidBodyConfig, HingedRigidBodyStateEffector,
};
use basilisk_rs::dynamics::reaction_wheel_state_effector::{
    ReactionWheelStateEffector, ReactionWheelStateEffectorConfig,
};
use basilisk_rs::messages::{
    HingedRigidBodyMsg, MotorTorqueMsg, Output, RwSpeedMsg, SpacecraftStateMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use nalgebra::{Matrix3, Vector3};

use super::dyncomp::{IntegratorChoice, sec2nano_u64, timed_run};
use super::dyncomp_orbit::{epoch, initial_orbit_state};
use super::grav_factory::custom_point_mass;
use super::message_log::MessageLog;
use super::multibody::set_center_of_mass_initial_state;

pub const HUB_MASS_KG: f64 = 600.0;
/// Upstream `HUB_CORE_INERTIA` \[kg m^2\]: the bare hub.
pub const HUB_CORE_INERTIA: [f64; 3] = [400.0, 380.0, 360.0];
/// Upstream `RW_JS`, `RW_JT` \[kg m^2\]: wheel axial and transverse inertia.
pub const RW_JS: f64 = 0.08;
pub const RW_JT: f64 = 0.04;
/// Upstream `RW_MASS` \[kg\]: the MuJoCo wheel bookkeeping mass.
pub const RW_MASS_KG: f64 = 1.0e-6;
/// Upstream `RW_TORQUE` \[N m\]: constant motor torque per wheel.
pub const RW_TORQUE: [f64; 3] = [0.010, -0.015, 0.008];
pub const PANEL_MASS_KG: f64 = 50.0;
/// Upstream `PANEL_INERTIA` \[kg m^2\], about the panel center of mass.
pub const PANEL_INERTIA: [f64; 3] = [30.0, 20.0, 12.0];
/// Upstream `PANEL_D` \[m\]: hinge axis to panel center of mass.
pub const PANEL_D_M: f64 = 1.2;
/// Upstream `PANEL_K` \[N m/rad\], `PANEL_C` \[N m s/rad\].
pub const PANEL_K: f64 = 150.0;
pub const PANEL_C: f64 = 8.0;
/// Upstream `PANEL_HINGE_X` \[m\]: hinge offset from the hub origin along x.
pub const PANEL_HINGE_X_M: f64 = 0.8;
/// Upstream `PANEL_THETA0` \[rad\]: initial panel deflection, 12°.
pub const PANEL_THETA0_RAD: f64 = 12.0 * std::f64::consts::PI / 180.0;
/// Upstream `OMEGA0_B` \[rad/s\]: initial hub rate.
pub const OMEGA0_B: [f64; 3] = [0.010, -0.020, 0.015];

/// Upstream `panelRootDcm`: the hinge frame that makes both panels extend
/// away from the hub.
pub fn panel_root_dcm(hinge_x: f64) -> Matrix3<f64> {
    if hinge_x > 0.0 {
        Matrix3::from_diagonal(&Vector3::new(-1.0, -1.0, 1.0))
    } else {
        Matrix3::identity()
    }
}

/// Upstream `hubInertiaBSM`: the bare hub plus the three wheel tensors.
pub fn hub_inertia_bsm() -> Matrix3<f64> {
    let mut inertia = Matrix3::from_diagonal(&Vector3::from(HUB_CORE_INERTIA));
    for axis in 0..3 {
        let mut diagonal = Vector3::repeat(RW_JT);
        diagonal[axis] = RW_JS;
        inertia += Matrix3::from_diagonal(&diagonal);
    }
    inertia
}

/// Upstream `mujocoModel()` (`scenarioCompareRwPanels`,
/// `scenarioCompareParetoRwPanels`) and `mujocoModel(withPanels)`
/// (`scenarioCompareOrbitMultibody`), with the wheel mass `rw_mass_kg`.
pub fn mujoco_model(with_panels: bool, rw_mass_kg: f64) -> String {
    let [ix, iy, iz] = HUB_CORE_INERTIA;
    let [px, py, pz] = PANEL_INERTIA;
    let (js, jt, d, k, c, x) = (RW_JS, RW_JT, PANEL_D_M, PANEL_K, PANEL_C, PANEL_HINGE_X_M);
    let panels = if with_panels {
        format!(
            r#"
      <body name="panelP" pos="{x} 0 0" quat="0 0 0 1">
        <joint name="panelP" type="hinge" axis="0 1 0" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="{} 0 0" mass="{PANEL_MASS_KG}" fullinertia="{px} {py} {pz} 0 0 0"/>
      </body>
      <body name="panelM" pos="{} 0 0">
        <joint name="panelM" type="hinge" axis="0 1 0" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="{} 0 0" mass="{PANEL_MASS_KG}" fullinertia="{px} {py} {pz} 0 0 0"/>
      </body>"#,
            -d, -x, -d
        )
    } else {
        String::new()
    };
    format!(
        r#"
<mujoco>
  <option gravity="0 0 0">
    <flag contact="disable" constraint="disable"/>
  </option>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS_KG}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
      <body name="rw_x" pos="0 0 0">
        <joint name="rw_x" type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 0" mass="{rw_mass_kg:e}" fullinertia="{js} {jt} {jt} 0 0 0"/>
      </body>
      <body name="rw_y" pos="0 0 0">
        <joint name="rw_y" type="hinge" axis="0 1 0"/>
        <inertial pos="0 0 0" mass="{rw_mass_kg:e}" fullinertia="{jt} {js} {jt} 0 0 0"/>
      </body>
      <body name="rw_z" pos="0 0 0">
        <joint name="rw_z" type="hinge" axis="0 0 1"/>
        <inertial pos="0 0 0" mass="{rw_mass_kg:e}" fullinertia="{jt} {jt} {js} 0 0 0"/>
      </body>{panels}
    </body>
  </worldbody>
  <actuator>
    <motor name="rw_x" joint="rw_x"/>
    <motor name="rw_y" joint="rw_y"/>
    <motor name="rw_z" joint="rw_z"/>
  </actuator>
</mujoco>
"#
    )
}

/// Upstream `initialBodyMassDistribution`: body masses \[kg\], their centers
/// and the total center of mass in the hub frame \[m\].
pub fn initial_body_mass_distribution(
    with_panels: bool,
) -> (Vec<f64>, Vec<Vector3<f64>>, Vector3<f64>) {
    let mut masses = vec![HUB_MASS_KG, RW_MASS_KG, RW_MASS_KG, RW_MASS_KG];
    let mut centers = vec![Vector3::zeros(); 4];
    if with_panels {
        for sign in [1.0, -1.0] {
            masses.push(PANEL_MASS_KG);
            centers.push(Vector3::new(
                sign * (PANEL_HINGE_X_M + PANEL_D_M * PANEL_THETA0_RAD.cos()),
                0.0,
                PANEL_D_M * PANEL_THETA0_RAD.sin(),
            ));
        }
    }
    let total: f64 = masses.iter().sum();
    let com = masses
        .iter()
        .zip(&centers)
        .fold(Vector3::zeros(), |sum, (m, c)| sum + *m * c)
        / total;
    (masses, centers, com)
}

/// One hub-wheels-panels propagation.
#[derive(Clone, Copy, Debug)]
pub struct RwPanelsCase {
    pub with_panels: bool,
    /// Point-mass Earth gravity of `mu` on the orbit of
    /// [`initial_orbit_state`] (upstream `scenarioCompareOrbitMultibody`);
    /// free floating at the origin without.
    pub gravity_mu: Option<f64>,
    /// The MuJoCo wheel bookkeeping mass \[kg\].
    pub rw_mass_kg: f64,
    pub integrator: IntegratorChoice,
}

impl RwPanelsCase {
    /// `scenarioCompareRwPanels`: free floating, panels on, RK4.
    pub const FREE: Self = Self {
        with_panels: true,
        gravity_mu: None,
        rw_mass_kg: RW_MASS_KG,
        integrator: IntegratorChoice::RK4,
    };
}

/// The recorded histories; `r_cn_n` and `v_cn_n` are the system center of
/// mass.
#[derive(Clone, Debug, Default)]
pub struct RwPanelsHistory {
    pub times_nanos: Vec<u64>,
    pub sigma_bn: Vec<Vector3<f64>>,
    pub omega_radps: Vec<Vector3<f64>>,
    pub r_bn_n: Vec<Vector3<f64>>,
    pub r_cn_n: Vec<Vector3<f64>>,
    pub v_cn_n: Vec<Vector3<f64>>,
    pub wheel_speeds_radps: Vec<[f64; 3]>,
    pub panel_angles_rad: Vec<[f64; 2]>,
}

fn panel_config(hinge_x: f64) -> HingedRigidBodyConfig {
    let mut config = HingedRigidBodyConfig::new(if hinge_x > 0.0 { "panelP" } else { "panelM" });
    config.mass_kg = PANEL_MASS_KG;
    config.inertia_about_panel_com_panel_kg_m2 =
        Matrix3::from_diagonal(&Vector3::from(PANEL_INERTIA));
    config.center_of_mass_offset_m = PANEL_D_M;
    config.spring_constant_nm_per_rad = PANEL_K;
    config.damping_nm_s_per_rad = PANEL_C;
    config.hinge_position_body_m = Vector3::new(hinge_x, 0.0, 0.0);
    config.body_to_hinge_dcm = panel_root_dcm(hinge_x);
    config.theta_init_rad = PANEL_THETA0_RAD;
    config.theta_dot_init_radps = 0.0;
    config
}

/// Upstream `rwFactory.create("custom", spinAxis, Js=RW_JS, Omega=0.0,
/// rWB_B=[0, 0, 0], useMaxTorque=False, useMinTorque=False)`: a balanced
/// wheel, `Jt = Jg = Js / 2`, no torque, momentum or speed limit.
fn wheel_config(index: usize) -> ReactionWheelStateEffectorConfig {
    let mut spin_axis = Vector3::zeros();
    spin_axis[index] = 1.0;
    let mut config = ReactionWheelStateEffectorConfig::balanced(
        format!("RW{}", index + 1),
        Vector3::zeros(),
        spin_axis,
        -1.0,
        0.0,
    );
    config.js_kg_m2 = RW_JS;
    config.jt_kg_m2 = 0.5 * RW_JS;
    config.jg_kg_m2 = 0.5 * RW_JS;
    config
}

/// Upstream `buildBSM` + `runBSM`: the histories, recorded every
/// `record_period` \[ns\] when given, and the propagation seconds.
pub fn run_bsm(
    case: &RwPanelsCase,
    dt_s: f64,
    tf_s: f64,
    record_period: Option<u64>,
) -> (RwPanelsHistory, f64) {
    let step = sec2nano_u64(dt_s);
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: hub_inertia_bsm(),
        integration_step_nanos: step,
        initial_position_m: Vector3::zeros(),
        initial_velocity_mps: Vector3::zeros(),
        initial_sigma_bn: Vector3::zeros(),
        initial_omega_radps: Vector3::from(OMEGA0_B),
        integrator: Some(case.integrator.integrator()),
    });

    let mut wheels = ReactionWheelStateEffector::new("rw");
    let torque_msgs: Vec<Output<MotorTorqueMsg>> = RW_TORQUE
        .iter()
        .map(|torque| {
            Output::new(MotorTorqueMsg {
                motor_torque_nm: *torque,
            })
        })
        .collect();
    for (index, torque_msg) in torque_msgs.iter().enumerate() {
        let wheel = wheels.add_reaction_wheel(wheel_config(index));
        torque_msg.connect_to(&mut wheel.motor_torque_in_msg);
    }
    let wheel_speeds = wheels.rw_speed_out_msg.clone();
    spacecraft.add_state_effector(wheels);
    if let Some(mu) = case.gravity_mu {
        spacecraft
            .add_grav_body(custom_point_mass("earth", mu, true))
            .expect("one central body");
    }
    let mut panel_outputs = Vec::new();
    if case.with_panels {
        for hinge_x in [PANEL_HINGE_X_M, -PANEL_HINGE_X_M] {
            let panel = HingedRigidBodyStateEffector::new(panel_config(hinge_x));
            panel_outputs.push(panel.hinged_rigid_body_out.clone());
            spacecraft.add_state_effector(panel);
        }
    }
    if let Some(mu) = case.gravity_mu {
        // Upstream `hub.r_CN_NInit`, `hub.v_CN_NInit`: the system center of
        // mass starts on the orbit.
        let (r, v, _) = initial_orbit_state(mu);
        set_center_of_mass_initial_state(&mut spacecraft, r, v);
    }

    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut com_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut wheel_log = MessageLog::<RwSpeedMsg>::default();
    let mut panel_logs: Vec<MessageLog<HingedRigidBodyMsg>> = panel_outputs
        .iter()
        .map(|_| MessageLog::default())
        .collect();
    let seconds = {
        let mut simulation = Simulation::new(epoch(), false);
        if record_period.is_some() {
            connect!(&simulation,
                &spacecraft.state_out => &mut state_log.input,
                &spacecraft.com_state_out => &mut com_log.input,
                &wheel_speeds => &mut wheel_log.input,
            );
            for (output, log) in panel_outputs.iter().zip(panel_logs.iter_mut()) {
                connect!(&simulation, output => &mut log.input);
            }
        }
        simulation.add_module("hub", &mut spacecraft, step, 10);
        if let Some(period) = record_period {
            simulation.add_module("stateRecorder", &mut state_log, period, 0);
            simulation.add_module("comRecorder", &mut com_log, period, 0);
            simulation.add_module("wheelRecorder", &mut wheel_log, period, 0);
            for log in &mut panel_logs {
                simulation.add_module("panelRecorder", log, period, 0);
            }
        }
        timed_run(&mut simulation, sec2nano_u64(tf_s))
    };
    let history = RwPanelsHistory {
        times_nanos: state_log.times_nanos.clone(),
        sigma_bn: state_log.values.iter().map(|s| s.sigma_bn).collect(),
        omega_radps: state_log.values.iter().map(|s| s.omega_radps).collect(),
        r_bn_n: state_log.values.iter().map(|s| s.position_m).collect(),
        r_cn_n: com_log.values.iter().map(|s| s.position_m).collect(),
        v_cn_n: com_log.values.iter().map(|s| s.velocity_mps).collect(),
        wheel_speeds_radps: wheel_log
            .values
            .iter()
            .map(|speeds| [0, 1, 2].map(|i| speeds.wheel_speeds_radps[i]))
            .collect(),
        panel_angles_rad: (0..state_log.values.len())
            .map(|sample| {
                let mut angles = [0.0; 2];
                for (angle, log) in angles.iter_mut().zip(&panel_logs) {
                    *angle = log.values[sample].theta_rad;
                }
                angles
            })
            .collect(),
    };
    (history, seconds)
}

#[cfg(feature = "mujoco")]
#[allow(unused_imports)]
pub use mujoco_side::{initial_mujoco_root_state, run_mujoco};

#[cfg(feature = "mujoco")]
mod mujoco_side {
    use super::*;
    use crate::dyncomp::{PreparedScene, add_earth_gravity_to_scene};
    use basilisk_rs::messages::{ScalarJointStateMsg, SingleActuatorMsg};
    use basilisk_rs::mujoco_dynamics::MjScene;
    use basilisk_rs::mujoco_dynamics::mj_system_com::MjSystemCom;

    const WHEELS: [&str; 3] = ["rw_x", "rw_y", "rw_z"];
    const PANELS: [&str; 2] = ["panelP", "panelM"];

    /// Upstream `initialMujocoRootState`: the hub-origin state that puts the
    /// total center of mass at `r_cn_n` with velocity `v_cn_n` (identity
    /// attitude, hinges at rest, so `v_CN = v_BN + ω × c_B`).
    pub fn initial_mujoco_root_state(
        with_panels: bool,
        r_cn_n: Vector3<f64>,
        v_cn_n: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let (_, _, c_b) = initial_body_mass_distribution(with_panels);
        (r_cn_n - c_b, v_cn_n - Vector3::from(OMEGA0_B).cross(&c_b))
    }

    /// Upstream `buildMujoco` + `runMujoco`: the hub-origin histories (the
    /// center of mass from an `MJSystemCoM` in orbit), recorded every
    /// `record_period` \[ns\] when given, and the propagation seconds.
    pub fn run_mujoco(
        case: &RwPanelsCase,
        dt_s: f64,
        tf_s: f64,
        record_period: Option<u64>,
    ) -> (RwPanelsHistory, f64) {
        let step = sec2nano_u64(dt_s);
        let mut scene =
            MjScene::new(&mujoco_model(case.with_panels, case.rw_mass_kg)).expect("rw scene");
        scene.name = "hubMj".to_string();
        scene.extra_eom_call = true;
        scene.set_high_order_attitude_integration(true);
        scene.set_boxed_integrator(case.integrator.integrator());
        let torque_msgs: Vec<Output<SingleActuatorMsg>> = RW_TORQUE
            .iter()
            .map(|torque| Output::new(SingleActuatorMsg { input: *torque }))
            .collect();
        for (name, msg) in WHEELS.iter().zip(&torque_msgs) {
            msg.connect_to(&mut scene.single_actuator_mut(name).actuator_in_msg);
        }
        if let Some(mu) = case.gravity_mu {
            add_earth_gravity_to_scene(&mut scene, mu);
        }
        let hub_state = scene.body("hub").origin().state_out_msg.clone();
        let wheel_rates: Vec<Output<ScalarJointStateMsg>> = WHEELS
            .iter()
            .map(|name| scene.scalar_joint(name).state_dot_out_msg.clone())
            .collect();
        let panel_angles: Vec<Output<ScalarJointStateMsg>> = if case.with_panels {
            PANELS
                .iter()
                .map(|name| scene.scalar_joint(name).state_out_msg.clone())
                .collect()
        } else {
            Vec::new()
        };
        let mut system_com = case.gravity_mu.map(|_| MjSystemCom::new(&scene));
        let has_com = system_com.is_some();
        let with_panels = case.with_panels;
        let initial_root = case.gravity_mu.map(|mu| {
            let (r, v, _) = initial_orbit_state(mu);
            initial_mujoco_root_state(with_panels, r, v)
        });
        let mut scene = PreparedScene::new(scene, move |scene| {
            if with_panels {
                for name in PANELS {
                    scene
                        .body_mut(name)
                        .set_scalar_joint_position(name, PANEL_THETA0_RAD);
                }
            }
            let mut hub = scene.body_mut("hub");
            if let Some((r, v)) = initial_root {
                hub.set_position(r);
                hub.set_velocity(v);
            }
            hub.set_attitude_rate(Vector3::from(OMEGA0_B));
        });

        let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
        let mut com_log = MessageLog::<SpacecraftStateMsg>::default();
        let mut wheel_logs: Vec<MessageLog<ScalarJointStateMsg>> =
            WHEELS.iter().map(|_| MessageLog::default()).collect();
        let mut panel_logs: Vec<MessageLog<ScalarJointStateMsg>> =
            panel_angles.iter().map(|_| MessageLog::default()).collect();
        let seconds = {
            let mut simulation = Simulation::new(epoch(), false);
            if record_period.is_some() {
                connect!(&simulation, &hub_state => &mut state_log.input);
                if let Some(system_com) = &system_com {
                    connect!(&simulation, &system_com.com_states_out_msg => &mut com_log.input);
                }
                for (output, log) in wheel_rates.iter().zip(wheel_logs.iter_mut()) {
                    connect!(&simulation, output => &mut log.input);
                }
                for (output, log) in panel_angles.iter().zip(panel_logs.iter_mut()) {
                    connect!(&simulation, output => &mut log.input);
                }
            }
            simulation.add_module("hubMj", &mut scene, step, 2);
            if let Some(system_com) = system_com.as_mut() {
                simulation.add_module("systemCoM", system_com, step, 1);
            }
            if let Some(period) = record_period {
                simulation.add_module("stateRecorder", &mut state_log, period, 0);
                if has_com {
                    simulation.add_module("comRecorder", &mut com_log, period, 0);
                }
                for log in wheel_logs.iter_mut().chain(panel_logs.iter_mut()) {
                    simulation.add_module("jointRecorder", log, period, 0);
                }
            }
            timed_run(&mut simulation, sec2nano_u64(tf_s))
        };
        let samples = state_log.values.len();
        let history = RwPanelsHistory {
            times_nanos: state_log.times_nanos.clone(),
            sigma_bn: state_log.values.iter().map(|s| s.sigma_bn).collect(),
            omega_radps: state_log.values.iter().map(|s| s.omega_radps).collect(),
            r_bn_n: state_log.values.iter().map(|s| s.position_m).collect(),
            r_cn_n: com_log.values.iter().map(|s| s.position_m).collect(),
            v_cn_n: com_log.values.iter().map(|s| s.velocity_mps).collect(),
            wheel_speeds_radps: (0..samples)
                .map(|sample| [0, 1, 2].map(|i| wheel_logs[i].values[sample].state))
                .collect(),
            panel_angles_rad: (0..samples)
                .map(|sample| {
                    let mut angles = [0.0; 2];
                    for (angle, log) in angles.iter_mut().zip(&panel_logs) {
                        *angle = log.values[sample].state;
                    }
                    angles
                })
                .collect(),
        };
        (history, seconds)
    }
}
