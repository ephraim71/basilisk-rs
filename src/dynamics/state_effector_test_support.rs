//! Scaffolding shared by the integrated state-effector tests ported from
//! upstream Basilisk (`test_dualhingedRigidBodyStateEffector.py`,
//! `test_nHingedRigidBodyStateEffector.py`, `test_linearSpringMassDamper.py`,
//! `test_sphericalPendulum.py`).
//!
//! Those tests configure the hub the way upstream `spacecraft.hub` does, with
//! the initial position and velocity of the spacecraft center of mass C
//! (`r_CN_NInit`, `v_CN_NInit`), log the spacecraft energy and momentum every
//! integration step and compare the first and last samples with
//! `unitTestSupport.isArrayEqualRelative` / `isArrayEqual`.

use hifitime::Epoch;
use nalgebra::{Matrix3, Vector3};

use crate::dynamics::gravity::GravBodyData;
use crate::messages::SpacecraftDiagnosticsMsg;
use crate::simulation::Simulation;
use crate::spacecraft::mrp::body_to_inertial_dcm_from_sigma_bn;
use crate::spacecraft::{Spacecraft, SpacecraftConfig, StateEffector};

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
