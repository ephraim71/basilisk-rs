//! Scaffolding shared by the integrated state-effector tests ported from
//! upstream Basilisk (`test_dualhingedRigidBodyStateEffector.py`,
//! `test_nHingedRigidBodyStateEffector.py`, `test_linearSpringMassDamper.py`,
//! `test_sphericalPendulum.py`, `test_mass_depletion.py`).
//!
//! Those tests configure the hub the way upstream `spacecraft.hub` does, with
//! the initial position and velocity of the spacecraft center of mass C
//! (`r_CN_NInit`, `v_CN_NInit`), log the spacecraft energy and momentum every
//! integration step and compare the first and last samples with
//! `unitTestSupport.isArrayEqualRelative` / `isArrayEqual`.

use hifitime::{Duration, Epoch};
use nalgebra::{Matrix3, Vector3};

use crate::dynamics::fuel_tank::{FuelTank, FuelTankConfig, FuelTankModel, FuelTankShape};
use crate::dynamics::gravity::GravBodyData;
use crate::dynamics::thruster_dynamic_effector::{
    ThrusterDynamicEffector, ThrusterDynamicEffectorConfig,
};
use crate::messages::{Output, SpacecraftDiagnosticsMsg, ThrusterCommandMsg};
use crate::simulation::Simulation;
use crate::spacecraft::mrp::body_to_inertial_dcm_from_sigma_bn;
use crate::spacecraft::{Spacecraft, SpacecraftConfig, StateEffector};
use crate::{Module, SimulationContext};

/// Earth gravitational parameter \[m^3/s^2\] of the upstream tests
/// (`earthGravBody.mu`, `simIncludeGravBody.createEarth().mu`).
pub(crate) const MU_EARTH_M3PS2: f64 = 0.3986004415e15;

/// Upstream `r_CN_NInit` \[m\] of the "Gravity" cases.
pub(crate) const GRAVITY_CASE_R_CN_N_M: [f64; 3] =
    [-4020338.690396649, 7490566.741852513, 5248299.211589362];

/// Upstream `v_CN_NInit` \[m/s\] of the "Gravity" cases.
pub(crate) const GRAVITY_CASE_V_CN_N_MPS: [f64; 3] =
    [-5199.77710904224, -3436.681645356935, 1041.576797498721];

/// Hub parameters as upstream `spacecraft.hub` takes them.
#[derive(Clone, Debug)]
pub(crate) struct HubSetup {
    /// `mHub` \[kg\].
    pub mass_kg: f64,
    /// `r_BcB_B` \[m\].
    pub center_of_mass_body_m: Vector3<f64>,
    /// `IHubPntBc_B` \[kg m^2\].
    pub inertia_kg_m2: Matrix3<f64>,
    /// `r_CN_NInit` \[m\]: initial position of the spacecraft center of mass.
    pub r_cn_n_m: Vector3<f64>,
    /// `v_CN_NInit` \[m/s\]: initial velocity of the spacecraft center of mass.
    pub v_cn_n_mps: Vector3<f64>,
    /// `sigma_BNInit`.
    pub sigma_bn: Vector3<f64>,
    /// `omega_BN_BInit` \[rad/s\].
    pub omega_bn_b_radps: Vector3<f64>,
}

impl HubSetup {
    /// The 750 kg hub of the hinged-body tests, at rest at the origin.
    pub(crate) fn hinged_body_hub() -> Self {
        Self {
            mass_kg: 750.0,
            center_of_mass_body_m: Vector3::zeros(),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            r_cn_n_m: Vector3::zeros(),
            v_cn_n_mps: Vector3::zeros(),
            sigma_bn: Vector3::zeros(),
            omega_bn_b_radps: Vector3::zeros(),
        }
    }

    /// Switches to the "Gravity" case initial orbit.
    pub(crate) fn in_gravity_case_orbit(mut self) -> Self {
        self.r_cn_n_m = Vector3::from(GRAVITY_CASE_R_CN_N_M);
        self.v_cn_n_mps = Vector3::from(GRAVITY_CASE_V_CN_N_MPS);
        self
    }
}

/// A spacecraft carrying `effectors`, with the hub's body-origin state chosen
/// so that the center of mass starts at `hub.r_cn_n_m` with velocity
/// `hub.v_cn_n_mps`, as upstream `Spacecraft::initializeDynamics` does:
/// `r_BN = r_CN − [NB] c` and `v_BN = v_CN − [NB] (c' + ω × c)`.
pub(crate) fn spacecraft_with_effectors(
    hub: &HubSetup,
    integration_step_nanos: u64,
    effectors: Vec<Box<dyn StateEffector>>,
) -> Spacecraft {
    let mut mass_kg = hub.mass_kg;
    let mut first_moment = hub.mass_kg * hub.center_of_mass_body_m;
    let mut first_moment_prime = Vector3::zeros();
    for effector in &effectors {
        let props = effector.mass_properties(&effector.initial_state());
        mass_kg += props.mass_kg;
        first_moment += props.mass_kg * props.center_of_mass_body_m;
        first_moment_prime += props.mass_kg * props.center_of_mass_prime_body_mps;
    }
    let c_b = first_moment / mass_kg;
    let c_dot_b = first_moment_prime / mass_kg + hub.omega_bn_b_radps.cross(&c_b);
    let dcm_nb = body_to_inertial_dcm_from_sigma_bn(hub.sigma_bn);

    let mut spacecraft = Spacecraft::new(SpacecraftConfig {
        mass_kg: hub.mass_kg,
        hub_center_of_mass_body_m: hub.center_of_mass_body_m,
        inertia_kg_m2: hub.inertia_kg_m2,
        integration_step_nanos,
        initial_position_m: hub.r_cn_n_m - dcm_nb * c_b,
        initial_velocity_mps: hub.v_cn_n_mps - dcm_nb * c_dot_b,
        initial_sigma_bn: hub.sigma_bn,
        initial_omega_radps: hub.omega_bn_b_radps,
        integrator: None,
    });
    spacecraft.state_effectors.extend(effectors);
    spacecraft
}

/// Adds the upstream tests' point-mass Earth as the central body.
pub(crate) fn add_point_mass_earth(spacecraft: &mut Spacecraft) {
    spacecraft
        .add_grav_body(
            GravBodyData::point_mass(
                "earth",
                MU_EARTH_M3PS2,
                true,
                Vector3::zeros(),
                Vector3::zeros(),
            )
            .expect("valid Earth gravity body"),
        )
        .expect("unique central gravity body");
}

/// Runs `spacecraft` alone for `steps` integration steps and returns its
/// diagnostics at t = 0 and after every step (upstream's `scObject.logger` on
/// the spacecraft task). `sample` is called at the same instants, so a test can
/// record effector outputs alongside.
pub(crate) fn run_logging_every_step(
    spacecraft: &mut Spacecraft,
    steps: usize,
    mut sample: impl FnMut(),
) -> Vec<SpacecraftDiagnosticsMsg> {
    let step_nanos = spacecraft.config.integration_step_nanos;
    let diagnostics = spacecraft.diagnostics_out.clone();
    let mut log = Vec::with_capacity(steps + 1);
    let mut sim = Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
    sim.add_module("spacecraft", spacecraft, step_nanos, 0);
    sim.run_for(0);
    log.push(diagnostics.read());
    sample();
    for _ in 0..steps {
        sim.run_for(step_nanos);
        log.push(diagnostics.read());
        sample();
    }
    log
}

/// Steps `spacecraft` alone through `steps` integration steps, calling
/// `inspect` at t = 0 and after every step, where it can read the effectors.
pub(crate) fn run_inspecting_every_step(
    spacecraft: &mut Spacecraft,
    steps: usize,
    mut inspect: impl FnMut(&Spacecraft),
) {
    let step_nanos = spacecraft.config.integration_step_nanos;
    let start = Epoch::from_gregorian_utc_at_midnight(2025, 1, 1);
    let context = |nanos: u64| SimulationContext {
        current_sim_nanos: nanos,
        current_epoch: start + Duration::from_total_nanoseconds(nanos as i128),
    };
    spacecraft.init();
    spacecraft.update(&context(0));
    inspect(spacecraft);
    for step in 1..=steps as u64 {
        spacecraft.update(&context(step * step_nanos));
        inspect(spacecraft);
    }
}

/// The 40 kg constant-volume tank of the slosh `MassDepletion` cases
/// (`test_linearSpringMassDamper.py`, `test_sphericalPendulum.py`), fed by
/// upstream `thFactory.create('MOOG_Monarc_445', [1, 0, 0], [0, 1, 0])`
/// commanded on for `on_time_s` at t = 0.
pub(crate) fn slosh_depletion_tank(on_time_s: f64) -> (FuelTank, ThrusterDynamicEffector) {
    let mut model = FuelTankModel::new(
        FuelTankShape::ConstantVolume {
            radius_tank_init_m: 46.0 / 2.0 / 3.2808399 / 12.0,
        },
        40.0,
    );
    model.r_tct_t_init_m = Vector3::zeros();
    let mut tank = FuelTank::new(FuelTankConfig::new("fuelTank", model));
    let mut config = ThrusterDynamicEffectorConfig::simple(
        "TH1",
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        445.0,
    );
    config.steady_isp_s = 234.0;
    config.area_nozzle_m2 = 0.06881;
    let mut thruster = ThrusterDynamicEffector::new(config);
    Output::new(ThrusterCommandMsg {
        on_time_s,
        command_time_nanos: 0,
    })
    .connect_to(&mut thruster.command_in);
    tank.add_thruster_set(&mut thruster);
    (tank, thruster)
}

/// Upstream `unitTestSupport.isArrayEqualRelative`: every component of
/// `result` within `accuracy` of `truth`, relative to `truth`; a zero truth
/// component requires an exactly zero result.
pub(crate) fn assert_relative_equal(result: &[f64], truth: &[f64], accuracy: f64, what: &str) {
    assert_eq!(result.len(), truth.len(), "{what}: dimension mismatch");
    for (index, (result, truth)) in result.iter().zip(truth).enumerate() {
        if *truth == 0.0 {
            assert!(
                *result == 0.0,
                "{what}[{index}]: truth is zero, result is {result:e}"
            );
            continue;
        }
        let relative = ((result - truth) / truth).abs();
        assert!(
            relative <= accuracy,
            "{what}[{index}]: {result:.15e} vs {truth:.15e}, relative error {relative:.3e} > {accuracy:e}"
        );
    }
}

/// Upstream `unitTestSupport.isArrayEqual`: every component of `result` within
/// `accuracy` of `truth`.
pub(crate) fn assert_absolute_equal(result: &[f64], truth: &[f64], accuracy: f64, what: &str) {
    assert_eq!(result.len(), truth.len(), "{what}: dimension mismatch");
    for (index, (result, truth)) in result.iter().zip(truth).enumerate() {
        let error = (result - truth).abs();
        assert!(
            error <= accuracy,
            "{what}[{index}]: {result:.15e} vs {truth:.15e}, error {error:.3e} > {accuracy:e}"
        );
    }
}

/// The energy and momentum checks every conservation test makes, comparing
/// `final_sample` against `initial`. `check_rotational_energy` is false for the
/// damped upstream cases, which only check the other three quantities.
pub(crate) fn assert_conserved(
    initial: &SpacecraftDiagnosticsMsg,
    final_sample: &SpacecraftDiagnosticsMsg,
    accuracy: f64,
    check_rotational_energy: bool,
) {
    assert_relative_equal(
        final_sample
            .orbital_angular_momentum_inertial_kg_m2ps
            .as_slice(),
        initial.orbital_angular_momentum_inertial_kg_m2ps.as_slice(),
        accuracy,
        "orbital angular momentum",
    );
    assert_relative_equal(
        final_sample
            .rotational_angular_momentum_inertial_kg_m2ps
            .as_slice(),
        initial
            .rotational_angular_momentum_inertial_kg_m2ps
            .as_slice(),
        accuracy,
        "rotational angular momentum",
    );
    assert_relative_equal(
        &[final_sample.orbital_energy_j],
        &[initial.orbital_energy_j],
        accuracy,
        "orbital energy",
    );
    if check_rotational_energy {
        assert_relative_equal(
            &[final_sample.rotational_energy_j],
            &[initial.rotational_energy_j],
            accuracy,
            "rotational energy",
        );
    }
}
