//! The variable-mass vehicle of upstream `scenarioCompareVariableMass.py`,
//! built for each engine; shared by `scenario_compare_variable_mass`,
//! `sweep_slosh_displacement` and `sweep_variable_mass_pendulum_inertia`.
//!
//! A 1500 kg dry hub carries a spherical hydrazine tank (1500 kg in a
//! 0.75 m sphere, aft of the dry center of mass) drained by a MOOG
//! Monarc-445 main engine (or an equivalent leak). The tank holds the
//! non-sloshing propellant, three orthogonal spring-mass-damper particles
//! (benchmark fixtures) and a two-degree-of-freedom spherical pendulum (the
//! first lateral slosh mode, Dodge 2000), all depleting proportionally.
//!
//! * **BSM**: a [`FuelTank`] (constant volume, update only) that owns three
//!   [`LinearSpringMassDamper`]s and a [`SphericalPendulum`] and feeds a
//!   [`ThrusterDynamicEffector`] (`addThrusterSet`). basilisk-rs's tank owns
//!   its particles, so only the tank is added to the spacecraft; the slosh
//!   states upstream reads from the dynamics manager come from a
//!   [`Hosted`] observer of the tank.
//! * **MuJoCo**: the hub with a welded tank body, three slide-jointed
//!   particles and a two-hinge pendulum bob, each propellant body depleted
//!   by a constant mass rate into its `derivative_mass_properties_in_msg`,
//!   the thrust a site motor, the pendulum damping a [`TwoHingeDamper`] and
//!   the system center of mass an `MJSystemCoM`.

#![allow(dead_code)]

use basilisk_rs::connect;
use basilisk_rs::dynamics::fuel_tank::{FuelTank, FuelTankConfig, FuelTankModel, FuelTankShape};
use basilisk_rs::dynamics::linear_spring_mass_damper::{
    LinearSpringMassDamper, LinearSpringMassDamperConfig,
};
use basilisk_rs::dynamics::spherical_pendulum::{SphericalPendulum, SphericalPendulumConfig};
use basilisk_rs::dynamics::thruster_dynamic_effector::ThrusterDynamicEffector;
use basilisk_rs::messages::{
    Output, SpacecraftMassPropsMsg, SpacecraftStateMsg, ThrusterCommandMsg,
};
use basilisk_rs::simulation::Simulation;
use basilisk_rs::spacecraft::{Spacecraft, SpacecraftConfig};
use basilisk_rs::utilities::orbital_motion::{ClassicElements, elements_to_state};
use basilisk_rs::utilities::rigid_body_kinematics::dcm_to_mrp;
use nalgebra::{Matrix3, Vector3};

use super::dyncomp::{sec2nano_u64, timed_run};
use super::dyncomp_orbit::epoch;
use super::grav_factory::{Body, custom_point_mass};
use super::message_log::MessageLog;
use super::multibody::{Hosted, SharedLog, set_center_of_mass_initial_state};
use super::thruster_factory::{ThrusterFactory, ThrusterModel, ThrusterSpec};

/// Upstream `G0` \[m/s^2\].
pub const G0: f64 = 9.80665;
pub const ORBIT_A_M: f64 = 7000.0e3;
const DEG: f64 = std::f64::consts::PI / 180.0;
pub const ORBIT_I_RAD: f64 = 33.3 * DEG;
pub const ORBIT_RAAN_RAD: f64 = 48.2 * DEG;
pub const ORBIT_ARGLAT_RAD: f64 = 85.3 * DEG;
pub const HUB_MASS_KG: f64 = 1500.0;
/// Upstream `HUB_INERTIA` \[kg m^2\]: the pitch axis (body y) is the major
/// axis of the loaded vehicle.
pub const HUB_INERTIA: [f64; 3] = [1200.0, 2000.0, 1900.0];
pub const TANK_RADIUS_M: f64 = 0.75;
/// Upstream `TANK_R_TB_B` \[m\]: tank center, aft of the dry center of mass.
pub const TANK_R_TB_B: [f64; 3] = [0.0, 0.0, -0.8];
pub const PROPELLANT_MASS_KG: f64 = 1500.0;
pub const PROPELLANT_DENSITY: f64 = 1004.0;
/// Upstream `SLOSH_MASS_FRACTION`, `PEND_LENGTH_RATIO`,
/// `SLOSH_DAMPING_RATIO` (Dodge 2000, at ~80 % fill).
pub const SLOSH_MASS_FRACTION: f64 = 0.335;
pub const PEND_LENGTH_RATIO: f64 = 0.452;
pub const SLOSH_DAMPING_RATIO: f64 = 0.0026;
pub const SMD_MASS_KG: f64 = 25.0;
pub const SMD_LATERAL_RHO0_M: f64 = 0.02;
pub const PEND_PHAT_01: [f64; 3] = [0.0, 0.0, -1.0];
pub const PEND_PHAT_02: [f64; 3] = [1.0, 0.0, 0.0];
pub const PEND_PHAT_03: [f64; 3] = [0.0, -1.0, 0.0];
pub const PEND_RATE0_RADPS: f64 = 0.02;
/// Upstream `PEND_BOB_INERTIA` \[kg m^2\]: the MuJoCo bob's initial
/// centroidal inertia.
pub const PEND_BOB_INERTIA: f64 = 1.0e-4;
pub const THRUST_DIR_B: [f64; 3] = [0.0, 0.0, 1.0];
pub const THRUST_POS_B: [f64; 3] = [0.0, 0.0, -1.5];
/// Upstream `SIM_DURATION` \[s\]: a 15-minute burn.
pub const SIM_DURATION_S: f64 = 900.0;
pub const STEPS_PER_PERIOD: f64 = 150.0;
pub const MAX_TIME_STEP_S: f64 = 0.05;

/// Upstream `thrusterSpec`: the catalog MOOG Monarc-445.
pub fn thruster_spec() -> ThrusterSpec {
    let mut factory = ThrusterFactory::default();
    factory
        .create(
            ThrusterModel::MoogMonarc445,
            Vector3::from(THRUST_POS_B),
            Vector3::from(THRUST_DIR_B),
            None,
        )
        .clone()
}

/// Upstream `nominalMassFlow` \[kg/s\].
pub fn nominal_mass_flow(max_thrust: f64, steady_isp: f64) -> f64 {
    max_thrust / (G0 * steady_isp)
}

/// Upstream `earthMu` \[m^3/s^2\].
pub fn earth_mu() -> f64 {
    Body::Earth.mu_m3ps2()
}

/// Upstream `sloshParameters(nearRigid)`.
#[derive(Clone, Copy, Debug)]
pub struct SloshParameters {
    pub accel: f64,
    pub pend_mass: f64,
    pub pend_length: f64,
    pub omega1: f64,
    pub bulk_mass: f64,
    pub fill_fraction: f64,
    pub smd_mass: f64,
    pub lateral_rho0: f64,
    pub pend_rate0: f64,
    pub near_rigid: bool,
    pub smd_k: f64,
    pub smd_c: f64,
    pub pend_d: f64,
    pub axial_rho0: f64,
}

pub fn slosh_parameters(near_rigid: bool) -> SloshParameters {
    let max_thrust = thruster_spec().max_thrust_n;
    let wet_mass = HUB_MASS_KG + PROPELLANT_MASS_KG;
    let accel = max_thrust / wet_mass;
    let (pend_mass, smd_mass, lateral_rho0, pend_rate0) = if near_rigid {
        (1.0e-6, 1.0e-9, 0.0, 0.0)
    } else {
        (
            SLOSH_MASS_FRACTION * PROPELLANT_MASS_KG,
            SMD_MASS_KG,
            SMD_LATERAL_RHO0_M,
            PEND_RATE0_RADPS,
        )
    };
    let pend_length = PEND_LENGTH_RATIO * TANK_RADIUS_M;
    let omega1 = (accel / pend_length).sqrt();
    let tank_volume = 4.0 / 3.0 * std::f64::consts::PI * TANK_RADIUS_M.powi(3);
    SloshParameters {
        accel,
        pend_mass,
        pend_length,
        omega1,
        bulk_mass: PROPELLANT_MASS_KG - pend_mass - 3.0 * smd_mass,
        fill_fraction: PROPELLANT_MASS_KG / (PROPELLANT_DENSITY * tank_volume),
        smd_mass,
        lateral_rho0,
        pend_rate0,
        near_rigid,
        smd_k: smd_mass * omega1 * omega1,
        smd_c: 2.0 * SLOSH_DAMPING_RATIO * smd_mass * omega1,
        pend_d: 2.0 * SLOSH_DAMPING_RATIO * pend_mass * omega1,
        axial_rho0: -accel / (omega1 * omega1),
    }
}

/// Upstream `timeStep()` \[s\].
pub fn time_step() -> f64 {
    let period = 2.0 * std::f64::consts::PI / slosh_parameters(false).omega1;
    MAX_TIME_STEP_S.min(period / STEPS_PER_PERIOD)
}

/// Upstream `initialThrustTorqueAboutCoM` \[N m\]: the thrust moment about
/// the wet center of mass, offset by the seeded lateral slosh.
pub fn initial_thrust_torque_about_com(max_thrust: f64, near_rigid: bool) -> Vector3<f64> {
    let slosh = slosh_parameters(near_rigid);
    let wet_mass = HUB_MASS_KG + PROPELLANT_MASS_KG;
    let offset = Vector3::new(
        slosh.smd_mass * slosh.lateral_rho0 / wet_mass,
        -slosh.smd_mass * slosh.lateral_rho0 / wet_mass,
        0.0,
    );
    let lever_arm = Vector3::from(THRUST_POS_B) - offset;
    lever_arm.cross(&(max_thrust * Vector3::from(THRUST_DIR_B)))
}

/// Upstream `initialOrbitState`: the circular parking orbit.
pub fn initial_orbit_state(mu: f64) -> (Vector3<f64>, Vector3<f64>) {
    elements_to_state(
        mu,
        &ClassicElements {
            semi_major_axis: ORBIT_A_M,
            eccentricity: 0.0,
            inclination_rad: ORBIT_I_RAD,
            raan_rad: ORBIT_RAAN_RAD,
            arg_periapsis_rad: 0.0,
            true_anomaly_rad: ORBIT_ARGLAT_RAD,
            ..Default::default()
        },
    )
}

/// Upstream `velocityAlignedAttitude`: body z along the velocity, y along
/// the orbit normal, and the orbital pitch rate about y.
pub fn velocity_aligned_attitude(
    r: Vector3<f64>,
    v: Vector3<f64>,
    mu: f64,
) -> (Vector3<f64>, Vector3<f64>, f64) {
    let v_hat = v.normalize();
    let h_hat = r.cross(&v).normalize();
    let x_hat = h_hat.cross(&v_hat);
    let dcm_bn = Matrix3::from_rows(&[x_hat.transpose(), h_hat.transpose(), v_hat.transpose()]);
    let mean_motion = (mu / ORBIT_A_M.powi(3)).sqrt();
    (
        dcm_to_mrp(dcm_bn),
        Vector3::new(0.0, mean_motion, 0.0),
        mean_motion,
    )
}

/// Upstream `gravityGradientRateEstimate` \[rad/s^2\].
pub fn gravity_gradient_rate_estimate(mu: f64) -> f64 {
    let arm = Vector3::from(TANK_R_TB_B).norm();
    let delta_inertia = PROPELLANT_MASS_KG * arm * arm;
    let mean_inertia = HUB_INERTIA.iter().sum::<f64>() / 3.0;
    3.0 * (mu / ORBIT_A_M.powi(3)) * delta_inertia / mean_inertia
}

/// The comparison switches of upstream `run` and `buildBSM`/`buildMujoco`.
#[derive(Clone, Copy, Debug)]
pub struct VariableMassCase {
    /// Upstream `useThruster`: the main engine, or an equivalent leak.
    pub use_thruster: bool,
    /// Upstream `inOrbit`: Earth gravity on the parking orbit, or a
    /// deep-space burn from rest.
    pub in_orbit: bool,
    /// Upstream `simDuration` \[s\]: the engine stays lit for twice it.
    pub sim_duration_s: f64,
    /// Upstream `nearRigid`: negligible slosh masses, no residual slosh.
    pub near_rigid: bool,
    /// Upstream `pendulumBobInertia` \[kg m^2\] (MuJoCo).
    pub pendulum_bob_inertia: f64,
}

impl VariableMassCase {
    pub fn new(use_thruster: bool, in_orbit: bool, sim_duration_s: f64) -> Self {
        Self {
            use_thruster,
            in_orbit,
            sim_duration_s,
            near_rigid: false,
            pendulum_bob_inertia: PEND_BOB_INERTIA,
        }
    }
}

/// Upstream `pullBSM`/`pullMujoco`: the histories both engines report.
#[derive(Clone, Debug, Default)]
pub struct VariableMassHistory {
    pub times_nanos: Vec<u64>,
    pub sigma_bn: Vec<Vector3<f64>>,
    pub omega_bn_b: Vec<Vector3<f64>>,
    pub r_bn_n: Vec<Vector3<f64>>,
    pub v_bn_n: Vec<Vector3<f64>>,
    /// The system center of mass.
    pub r_cn_n: Vec<Vector3<f64>>,
    pub v_cn_n: Vec<Vector3<f64>>,
    /// Vis-viva semi-major axis of the center of mass \[m\].
    pub semi_major_axis: Vec<f64>,
    /// The non-sloshing propellant `m0` \[kg\].
    pub fuel_mass: Vec<f64>,
    /// Dry hub plus every propellant mass \[kg\].
    pub total_mass: Vec<f64>,
    /// The three spring-mass-damper displacements \[m\].
    pub rho: Vec<[f64; 3]>,
    pub phi: Vec<f64>,
    pub theta: Vec<f64>,
}

impl VariableMassHistory {
    fn finish(&mut self, mu: f64) {
        self.semi_major_axis = self
            .r_cn_n
            .iter()
            .zip(&self.v_cn_n)
            .map(|(r, v)| 1.0 / (2.0 / r.norm() - v.norm_squared() / mu))
            .collect();
    }
}

/// The initial hub-origin state the MuJoCo run copies from BSM's first
/// sample.
#[derive(Clone, Copy, Debug)]
pub struct InitialState {
    pub r_bn_n: Vector3<f64>,
    pub v_bn_n: Vector3<f64>,
    pub sigma_bn: Vector3<f64>,
    pub omega_bn_b: Vector3<f64>,
}

impl VariableMassHistory {
    pub fn initial_state(&self) -> InitialState {
        InitialState {
            r_bn_n: self.r_bn_n[0],
            v_bn_n: self.v_bn_n[0],
            sigma_bn: self.sigma_bn[0],
            omega_bn_b: self.omega_bn_b[0],
        }
    }
}

/// `[m0, rho1, rho2, rho3, phi, theta]` of the tank's committed state.
type TankSample = [f64; 6];

/// Upstream `buildBSM` propagated to `case.sim_duration_s`: the histories
/// (with `record`, every step) and the propagation seconds.
pub fn run_bsm(case: &VariableMassCase, dt_s: f64, record: bool) -> (VariableMassHistory, f64) {
    let step = sec2nano_u64(dt_s);
    let mu = earth_mu();
    let (orbit_r, orbit_v) = initial_orbit_state(mu);
    let (sigma_bn, omega_bn_b, _) = velocity_aligned_attitude(orbit_r, orbit_v, mu);
    let (r_cn_n, v_cn_n) = if case.in_orbit {
        (orbit_r, orbit_v)
    } else {
        (Vector3::zeros(), Vector3::zeros())
    };
    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: HUB_MASS_KG,
        hub_center_of_mass_body_m: Vector3::zeros(),
        inertia_kg_m2: Matrix3::from_diagonal(&Vector3::from(HUB_INERTIA)),
        integration_step_nanos: step,
        initial_position_m: r_cn_n,
        initial_velocity_mps: v_cn_n,
        initial_sigma_bn: sigma_bn,
        initial_omega_radps: omega_bn_b,
        integrator: None,
    });
    if case.in_orbit {
        spacecraft
            .add_grav_body(custom_point_mass("earth", mu, true))
            .expect("one central body");
    }
    let slosh = slosh_parameters(case.near_rigid);
    let tank_center = Vector3::from(TANK_R_TB_B);

    let mut model = FuelTankModel::new(
        FuelTankShape::ConstantVolume {
            radius_tank_init_m: TANK_RADIUS_M,
        },
        slosh.bulk_mass,
    );
    model.max_fuel_mass_kg = slosh.bulk_mass;
    model.r_tct_t_init_m = Vector3::zeros();
    let spec = thruster_spec();
    let mut tank = FuelTank::new(FuelTankConfig {
        r_tb_b_m: tank_center,
        update_only: true,
        fuel_leak_rate_kgps: if case.use_thruster {
            0.0
        } else {
            nominal_mass_flow(spec.max_thrust_n, spec.steady_isp_s)
        },
        ..FuelTankConfig::new("propTank", model)
    });
    for (index, (direction, rho0)) in [
        ([1.0, 0.0, 0.0], slosh.lateral_rho0),
        ([0.0, 1.0, 0.0], -slosh.lateral_rho0),
        ([0.0, 0.0, 1.0], slosh.axial_rho0),
    ]
    .into_iter()
    .enumerate()
    {
        tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(LinearSpringMassDamperConfig {
            spring_constant_npm: slosh.smd_k,
            damping_ns_per_m: slosh.smd_c,
            rho_init_m: rho0,
            rho_dot_init_mps: 0.0,
            mass_init_kg: slosh.smd_mass,
            equilibrium_position_body_m: tank_center,
            direction_body: Vector3::from(direction),
            ..LinearSpringMassDamperConfig::new(format!("smd{}", index + 1))
        }));
    }
    tank.push_fuel_slosh_particle(SphericalPendulum::new(SphericalPendulumConfig {
        pendulum_radius_m: slosh.pend_length,
        tank_center_body_m: tank_center,
        damping_matrix_p0_ns_per_m: slosh.pend_d * Matrix3::identity(),
        phi_dot_init_radps: slosh.pend_rate0,
        theta_dot_init_radps: slosh.pend_rate0,
        mass_init_kg: slosh.pend_mass,
        p_hat_01_body: Vector3::from(PEND_PHAT_01),
        p_hat_02_body: Vector3::from(PEND_PHAT_02),
        p_hat_03_body: Vector3::from(PEND_PHAT_03),
        ..SphericalPendulumConfig::new("pendulum")
    }));

    let command = Output::new(ThrusterCommandMsg {
        // Keep the engine lit for the whole burn.
        on_time_s: 2.0 * case.sim_duration_s,
        command_time_nanos: 0,
    });
    if case.use_thruster {
        let mut thruster = ThrusterDynamicEffector::new(spec.dynamic_effector_config());
        tank.add_thruster_set(&mut thruster);
        command.connect_to(&mut thruster.command_in);
        spacecraft.add_dynamic_effector(thruster);
    }

    // Upstream's `pythonVariableLogger` of the tank and slosh states.
    let tank_log = SharedLog::<TankSample>::default();
    let observed = tank_log.clone();
    let tank = Hosted::new(tank).observe(move |tank: &FuelTank, time_nanos, _| {
        let particles = tank.fuel_slosh_particles();
        let rho = |index: usize| {
            particles[index]
                .as_any()
                .downcast_ref::<LinearSpringMassDamper>()
                .expect("spring-mass-damper particle")
                .rho_m
        };
        let pendulum = particles[3]
            .as_any()
            .downcast_ref::<SphericalPendulum>()
            .expect("spherical pendulum");
        observed.push(
            time_nanos,
            [
                tank.fuel_mass_kg,
                rho(0),
                rho(1),
                rho(2),
                pendulum.angles_rad[0],
                pendulum.angles_rad[1],
            ],
        );
    });
    spacecraft.add_state_effector(tank);
    // Upstream `hub.r_CN_NInit`, `hub.v_CN_NInit`.
    set_center_of_mass_initial_state(&mut spacecraft, r_cn_n, v_cn_n);

    let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut com_log = MessageLog::<SpacecraftStateMsg>::default();
    let mut mass_log = MessageLog::<SpacecraftMassPropsMsg>::default();
    let seconds = {
        let mut simulation = Simulation::new(epoch(), false);
        if record {
            connect!(&simulation,
                &spacecraft.state_out => &mut state_log.input,
                &spacecraft.com_state_out => &mut com_log.input,
                &spacecraft.mass_props_out => &mut mass_log.input,
            );
        }
        simulation.add_module("hub", &mut spacecraft, step, 10);
        if record {
            simulation.add_module("stateRecorder", &mut state_log, step, 0);
            simulation.add_module("comRecorder", &mut com_log, step, 0);
            simulation.add_module("massRecorder", &mut mass_log, step, 0);
        }
        timed_run(&mut simulation, sec2nano_u64(case.sim_duration_s))
    };
    let tank_samples: Vec<TankSample> = tank_log
        .samples()
        .into_iter()
        .filter(|(time, _)| state_log.times_nanos.binary_search(time).is_ok())
        .map(|(_, sample)| sample)
        .collect();
    let mut history = VariableMassHistory {
        times_nanos: state_log.times_nanos.clone(),
        sigma_bn: state_log.values.iter().map(|s| s.sigma_bn).collect(),
        omega_bn_b: state_log.values.iter().map(|s| s.omega_radps).collect(),
        r_bn_n: state_log.values.iter().map(|s| s.position_m).collect(),
        v_bn_n: state_log.values.iter().map(|s| s.velocity_mps).collect(),
        r_cn_n: com_log.values.iter().map(|s| s.position_m).collect(),
        v_cn_n: com_log.values.iter().map(|s| s.velocity_mps).collect(),
        total_mass: mass_log.values.iter().map(|m| m.mass_kg).collect(),
        fuel_mass: tank_samples.iter().map(|s| s[0]).collect(),
        rho: tank_samples.iter().map(|s| [s[1], s[2], s[3]]).collect(),
        phi: tank_samples.iter().map(|s| s[4]).collect(),
        theta: tank_samples.iter().map(|s| s[5]).collect(),
        ..VariableMassHistory::default()
    };
    if record {
        assert_eq!(
            tank_samples.len(),
            history.times_nanos.len(),
            "tank observer samples"
        );
    }
    history.finish(mu);
    (history, seconds)
}

#[cfg(feature = "mujoco")]
#[allow(unused_imports)]
pub use mujoco_side::{body_mass_flow_rates, mujoco_model, run_mujoco};

#[cfg(feature = "mujoco")]
mod mujoco_side {
    use super::*;
    use crate::dyncomp::{PreparedScene, add_earth_gravity_to_scene};
    use basilisk_rs::integrators::Rk4;
    use basilisk_rs::messages::{ScalarJointStateMsg, SingleActuatorMsg};
    use basilisk_rs::mujoco_dynamics::MjScene;
    use basilisk_rs::mujoco_dynamics::mj_system_com::MjSystemCom;
    use basilisk_rs::mujoco_dynamics::two_hinge_damper::TwoHingeDamper;

    const PROPELLANT_BODIES: [&str; 5] = ["tank", "sloshX", "sloshY", "sloshZ", "pendulum"];
    const SLOSH_JOINTS: [&str; 3] = ["sloshX", "sloshY", "sloshZ"];

    /// Upstream `mujocoModel(pendulumBobInertia, nearRigid)`.
    pub fn mujoco_model(bob_inertia: f64, near_rigid: bool) -> String {
        assert!(
            bob_inertia > 0.0,
            "The MuJoCo pendulum bob inertia must be positive."
        );
        let [ix, iy, iz] = HUB_INERTIA;
        let [tx, ty, tz] = TANK_R_TB_B;
        let [px, py, pz] = THRUST_POS_B;
        let slosh = slosh_parameters(near_rigid);
        let m0 = slosh.bulk_mass;
        let tank_inertia = 0.4 * m0 * TANK_RADIUS_M * TANK_RADIUS_M;
        let (k, c, length, pend_mass, smd_mass) = (
            slosh.smd_k,
            slosh.smd_c,
            slosh.pend_length,
            slosh.pend_mass,
            slosh.smd_mass,
        );
        let particle = |name: &str, axis: &str| {
            format!(
                r#"
      <body name="{name}" pos="{tx} {ty} {tz}">
        <joint name="{name}" type="slide" axis="{axis}" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{smd_mass:e}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>"#
            )
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
      <site name="thrustPoint" pos="{px} {py} {pz}"/>
      <body name="tank" pos="{tx} {ty} {tz}">
        <inertial pos="0 0 0" mass="{m0}" diaginertia="{tank_inertia} {tank_inertia} {tank_inertia}"/>
      </body>{}{}{}
      <body name="pendulum" pos="{tx} {ty} {tz}">
        <joint name="pendulumPhi" type="hinge" axis="0 -1 0"/>
        <joint name="pendulumTheta" type="hinge" axis="1 0 0"/>
        <inertial pos="0 0 {}" mass="{pend_mass:e}" diaginertia="{bob_inertia:e} {bob_inertia:e} {bob_inertia:e}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="mainEngine" site="thrustPoint" gear="0 0 1 0 0 0"/>
  </actuator>
</mujoco>
"#,
            particle("sloshX", "1 0 0"),
            particle("sloshY", "0 1 0"),
            particle("sloshZ", "0 0 1"),
            -length
        )
    }

    /// Upstream `bodyMassFlowRates`: each propellant body drains at its
    /// initial share of the engine flow \[kg/s\] (negative).
    pub fn body_mass_flow_rates(near_rigid: bool) -> Vec<(&'static str, f64)> {
        let spec = thruster_spec();
        let total = nominal_mass_flow(spec.max_thrust_n, spec.steady_isp_s);
        let slosh = slosh_parameters(near_rigid);
        let share = |mass: f64| -total * mass / PROPELLANT_MASS_KG;
        vec![
            ("tank", share(slosh.bulk_mass)),
            ("sloshX", share(slosh.smd_mass)),
            ("sloshY", share(slosh.smd_mass)),
            ("sloshZ", share(slosh.smd_mass)),
            ("pendulum", share(slosh.pend_mass)),
        ]
    }

    /// Upstream `buildMujoco` propagated to `case.sim_duration_s` from
    /// `initial` (BSM's first sample).
    pub fn run_mujoco(
        case: &VariableMassCase,
        dt_s: f64,
        record: bool,
        initial: InitialState,
    ) -> (VariableMassHistory, f64) {
        let step = sec2nano_u64(dt_s);
        let mu = earth_mu();
        let slosh = slosh_parameters(case.near_rigid);
        let mut scene = MjScene::new(&mujoco_model(case.pendulum_bob_inertia, case.near_rigid))
            .expect("variable-mass scene");
        scene.name = "hubMj".to_string();
        scene.extra_eom_call = true;
        scene.set_high_order_attitude_integration(true);
        scene.set_integrator(Rk4);
        if case.in_orbit {
            add_earth_gravity_to_scene(&mut scene, mu);
        }
        let rate_msgs: Vec<Output<SpacecraftMassPropsMsg>> = body_mass_flow_rates(case.near_rigid)
            .into_iter()
            .map(|(name, rate)| {
                let msg = Output::new(SpacecraftMassPropsMsg {
                    mass_kg: rate,
                    ..SpacecraftMassPropsMsg::default()
                });
                msg.connect_to(&mut scene.body_mut(name).derivative_mass_properties_in_msg);
                msg
            })
            .collect();
        let thrust = Output::new(SingleActuatorMsg {
            input: thruster_spec().max_thrust_n,
        });
        if case.use_thruster {
            thrust.connect_to(&mut scene.single_actuator_mut("mainEngine").actuator_in_msg);
        }
        // `Q_phi = -d L^2 cos(theta)^2 phiDot`, `Q_theta = -d L^2 thetaDot`.
        let mut damper = TwoHingeDamper {
            name: "pendulumDamper".to_string(),
            damping_coeff: slosh.pend_d * slosh.pend_length * slosh.pend_length,
            ..TwoHingeDamper::default()
        };
        damper
            .apply_to(&mut scene, "pendulumPhi", "pendulumTheta")
            .expect("two distinct hinges");
        scene.add_model_to_dynamics_task(damper, -1);

        let hub_state = scene.body("hub").origin().state_out_msg.clone();
        let masses: Vec<Output<SpacecraftMassPropsMsg>> = PROPELLANT_BODIES
            .iter()
            .map(|name| scene.body(name).mass_properties_out_msg.clone())
            .collect();
        let joints: Vec<Output<ScalarJointStateMsg>> = SLOSH_JOINTS
            .iter()
            .chain(["pendulumPhi", "pendulumTheta"].iter())
            .map(|name| scene.scalar_joint(name).state_out_msg.clone())
            .collect();
        let mut system_com = MjSystemCom::new(&scene);
        system_com.name = "systemCoM".to_string();
        let mut scene = PreparedScene::new(scene, move |scene| {
            {
                let mut hub = scene.body_mut("hub");
                hub.set_position(initial.r_bn_n);
                hub.set_velocity(initial.v_bn_n);
                hub.set_attitude(initial.sigma_bn);
                hub.set_attitude_rate(initial.omega_bn_b);
            }
            for (name, rho0) in [
                ("sloshX", slosh.lateral_rho0),
                ("sloshY", -slosh.lateral_rho0),
                ("sloshZ", slosh.axial_rho0),
            ] {
                scene.body_mut(name).set_scalar_joint_position(name, rho0);
            }
            let mut pendulum = scene.body_mut("pendulum");
            pendulum.set_scalar_joint_velocity("pendulumPhi", slosh.pend_rate0);
            pendulum.set_scalar_joint_velocity("pendulumTheta", slosh.pend_rate0);
        });

        let mut state_log = MessageLog::<SpacecraftStateMsg>::default();
        let mut com_log = MessageLog::<SpacecraftStateMsg>::default();
        let mut mass_logs: Vec<MessageLog<SpacecraftMassPropsMsg>> =
            masses.iter().map(|_| MessageLog::default()).collect();
        let mut joint_logs: Vec<MessageLog<ScalarJointStateMsg>> =
            joints.iter().map(|_| MessageLog::default()).collect();
        let seconds = {
            let mut simulation = Simulation::new(epoch(), false);
            if record {
                connect!(&simulation,
                    &hub_state => &mut state_log.input,
                    &system_com.com_states_out_msg => &mut com_log.input,
                );
                for (output, log) in masses.iter().zip(mass_logs.iter_mut()) {
                    connect!(&simulation, output => &mut log.input);
                }
                for (output, log) in joints.iter().zip(joint_logs.iter_mut()) {
                    connect!(&simulation, output => &mut log.input);
                }
            }
            simulation.add_module("hubMj", &mut scene, step, 2);
            simulation.add_module("systemCoM", &mut system_com, step, 1);
            if record {
                simulation.add_module("stateRecorder", &mut state_log, step, 0);
                simulation.add_module("comRecorder", &mut com_log, step, 0);
                for log in mass_logs.iter_mut() {
                    simulation.add_module("massRecorder", log, step, 0);
                }
                for log in joint_logs.iter_mut() {
                    simulation.add_module("jointRecorder", log, step, 0);
                }
            }
            timed_run(&mut simulation, sec2nano_u64(case.sim_duration_s))
        };
        drop(rate_msgs);
        let samples = state_log.values.len();
        let joint = |index: usize| -> Vec<f64> {
            joint_logs[index].values.iter().map(|j| j.state).collect()
        };
        let mut history = VariableMassHistory {
            times_nanos: state_log.times_nanos.clone(),
            sigma_bn: state_log.values.iter().map(|s| s.sigma_bn).collect(),
            omega_bn_b: state_log.values.iter().map(|s| s.omega_radps).collect(),
            r_bn_n: state_log.values.iter().map(|s| s.position_m).collect(),
            v_bn_n: state_log.values.iter().map(|s| s.velocity_mps).collect(),
            r_cn_n: com_log.values.iter().map(|s| s.position_m).collect(),
            v_cn_n: com_log.values.iter().map(|s| s.velocity_mps).collect(),
            fuel_mass: mass_logs[0].values.iter().map(|m| m.mass_kg).collect(),
            total_mass: (0..samples)
                .map(|i| {
                    HUB_MASS_KG
                        + mass_logs
                            .iter()
                            .map(|log| log.values[i].mass_kg)
                            .sum::<f64>()
                })
                .collect(),
            rho: (0..samples)
                .map(|i| [0, 1, 2].map(|j| joint_logs[j].values[i].state))
                .collect(),
            phi: joint(3),
            theta: joint(4),
            ..VariableMassHistory::default()
        };
        history.finish(mu);
        (history, seconds)
    }
}
