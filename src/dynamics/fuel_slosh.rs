//! A fuel-slosh particle a fuel tank drives: port of upstream Basilisk
//! `simulation/dynamics/_GeneralModuleFiles/fuelSlosh.h` (`FuelSlosh`).
//!
//! A slosh particle carries part of a tank's propellant. Its mass is one of
//! its integrated states; on its own the mass stays constant, and once pushed
//! onto a [`FuelTank`](super::fuel_tank::FuelTank)
//! (upstream `pushFuelSloshParticle`) the tank drains it in proportion to its
//! share of the tank's propellant.
//!
//! Upstream the tank keeps pointers to particles that are also added to the
//! spacecraft, and sets their public `fuelMassDot` before they compute their
//! mass properties. Here the tank owns its particles: their states follow the
//! tank's mass in the tank's own state, and the tank passes each particle its
//! [`FuelSloshFlow`] with every call.

use nalgebra::Vector3;

use crate::spacecraft::{StateEffector, StateEffectorMassProps};

/// The propellant flow a tank assigns to one particle (upstream
/// `FuelSlosh::fuelMassDot` and `omitMassRateDynamics`, set in
/// `FuelTank::updateEffectorMassProps`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FuelSloshFlow {
    /// Upstream `fuelMassDot` \[kg/s\]: rate of the particle's mass.
    pub fuel_mass_dot_kgps: f64,
    /// Upstream `omitMassRateDynamics`: the tank is update-only, so the
    /// equations of motion take the mass rate as zero.
    pub omit_mass_rate_dynamics: bool,
}

/// Mass properties of a depleting body: a particle under a
/// [`FuelSloshFlow`], or a [`FuelTank`](super::fuel_tank::FuelTank)'s own.
///
/// Upstream `EffectorMassProps` carries both rate sets: the rates of the
/// retained mass properties (`mEffDot`, `rEffPrime_CB_B`, `IEffPrimePntB_B`)
/// and the overrides the equations of motion substitute into their generic
/// rate terms (`mEffDotDynamics`, `rEffPrime_CB_BDynamics`,
/// `IEffPrimePntB_BDynamics`). Both have the same mass, center of mass and
/// inertia.
#[derive(Clone, Debug)]
pub struct DepletionMassProps {
    /// Mass properties with the retained-property rates.
    pub retained: StateEffectorMassProps,
    /// Mass properties with the rates the equations of motion use; what
    /// [`StateEffector::mass_properties`] reports.
    pub dynamics: StateEffectorMassProps,
}

/// A state effector a fuel tank can drive (upstream `FuelSlosh`).
pub trait FuelSlosh: StateEffector {
    /// Index of the particle's mass \[kg\] in its state.
    fn fuel_mass_index(&self) -> usize;

    /// Upstream `retrieveMassValue`: the particle's propellant mass
    /// (`fuelMass`) in `effector_state`, an integration overshoot below zero
    /// read as empty.
    fn fuel_mass_kg(&self, effector_state: &[f64]) -> f64 {
        effector_state[self.fuel_mass_index()].max(0.0)
    }

    /// Mass properties while the tank drains the particle at `flow` (upstream
    /// `updateEffectorMassProps` after the tank set `fuelMassDot`).
    fn mass_properties_with_flow(
        &self,
        effector_state: &[f64],
        flow: FuelSloshFlow,
    ) -> DepletionMassProps;

    /// State derivatives while the tank drains the particle at `flow`; the
    /// mass state's is `flow.fuel_mass_dot_kgps` (upstream
    /// `computeDerivatives`).
    fn compute_derivatives_with_flow(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
        flow: FuelSloshFlow,
    ) -> Vec<f64>;
}

/// Mass properties of a point-mass particle of `mass_kg` at `r_pc_b_m` with
/// body-frame velocity `r_prime_pc_b_mps`, draining at `flow` (the
/// `updateEffectorMassProps` of upstream `LinearSpringMassDamper` and
/// `SphericalPendulum`).
pub(crate) fn point_mass_slosh_props(
    mass_kg: f64,
    r_pc_b_m: Vector3<f64>,
    r_prime_pc_b_mps: Vector3<f64>,
    flow: FuelSloshFlow,
) -> DepletionMassProps {
    let r_tilde = r_pc_b_m.cross_matrix();
    let r_prime_tilde = r_prime_pc_b_mps.cross_matrix();
    let inertia_rate_from_motion = -mass_kg * (r_prime_tilde * r_tilde + r_tilde * r_prime_tilde);
    let retained = StateEffectorMassProps {
        mass_kg,
        mass_dot_kgps: flow.fuel_mass_dot_kgps,
        center_of_mass_body_m: r_pc_b_m,
        center_of_mass_prime_body_mps: r_prime_pc_b_mps,
        inertia_about_point_b_body_kg_m2: mass_kg * r_tilde * r_tilde.transpose(),
        inertia_about_point_b_body_prime_kg_m2ps: -flow.fuel_mass_dot_kgps * r_tilde * r_tilde
            + inertia_rate_from_motion,
    };
    let dynamics = StateEffectorMassProps {
        mass_dot_kgps: if flow.omit_mass_rate_dynamics {
            0.0
        } else {
            flow.fuel_mass_dot_kgps
        },
        inertia_about_point_b_body_prime_kg_m2ps: inertia_rate_from_motion,
        ..retained.clone()
    };
    DepletionMassProps { retained, dynamics }
}
