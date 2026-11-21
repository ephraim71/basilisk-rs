//! A point mass on a linear spring–damper inside the hub: port of upstream
//! Basilisk `simulation/dynamics/LinearSpringMassDamper`
//! (`LinearSpringMassDamper`), the linear fuel-slosh model.
//!
//! The particle moves along the body-fixed unit vector `pHat_B` through its
//! equilibrium point P (`r_PB_B`); its displacement ρ from P is restrained by a
//! spring `k` and a damper `c`. The states `[ρ, ρ̇, m]` are coupled to the hub
//! by back-substitution.
//!
//! The particle mass `m` is constant on its own. Pushed onto a
//! [`FuelTank`](super::fuel_tank::FuelTank) as fuel slosh
//! ([`FuelTank::push_fuel_slosh_particle`](super::fuel_tank::FuelTank::push_fuel_slosh_particle),
//! upstream `pushFuelSloshParticle`), it is a [`FuelSlosh`] particle the tank
//! drains (see [`super::fuel_slosh`]).
//!
//! Not ported: upstream `calcForceTorqueOnBody`, the force and torque the
//! particle exerts on the hub, which upstream only stores for logging.

use nalgebra::Vector3;
use std::any::Any;
use std::cell::RefCell;

use super::fuel_slosh::{DepletionMassProps, FuelSlosh, FuelSloshFlow, point_mass_slosh_props};
use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};

/// User-settable parameters of [`LinearSpringMassDamper`].
#[derive(Clone, Debug)]
pub struct LinearSpringMassDamperConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `k` \[N/m\]: linear spring constant (default 1).
    pub spring_constant_npm: f64,
    /// Upstream `c` \[N s/m\]: linear damping coefficient (default 0).
    pub damping_ns_per_m: f64,
    /// Upstream `rhoInit` \[m\]: initial displacement from equilibrium
    /// (default 0).
    pub rho_init_m: f64,
    /// Upstream `rhoDotInit` \[m/s\] (default 0).
    pub rho_dot_init_mps: f64,
    /// Upstream `massInit` \[kg\]: initial particle mass, the start of its
    /// mass state (default 0, a particle that does not couple to the hub).
    pub mass_init_kg: f64,
    /// Upstream `r_PB_B` \[m\]: equilibrium point relative to the body origin
    /// (default zero).
    pub equilibrium_position_body_m: Vector3<f64>,
    /// Upstream `pHat_B`: unit direction of particle motion in body-frame
    /// components (default x̂, as upstream's `setIdentity()` on a 3-vector).
    pub direction_body: Vector3<f64>,
}

impl LinearSpringMassDamperConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            spring_constant_npm: 1.0,
            damping_ns_per_m: 0.0,
            rho_init_m: 0.0,
            rho_dot_init_mps: 0.0,
            mass_init_kg: 0.0,
            equilibrium_position_body_m: Vector3::zeros(),
            direction_body: Vector3::x(),
        }
    }

    /// Upstream `validateConfiguration`: the initial mass is finite and
    /// non-negative.
    pub fn validate(&self) -> Result<(), String> {
        if !self.mass_init_kg.is_finite() || self.mass_init_kg < 0.0 {
            return Err("mass_init_kg must be finite and non-negative".to_string());
        }
        Ok(())
    }
}

/// Linear spring–mass–damper particle (upstream `LinearSpringMassDamper`).
///
/// State layout: `[ρ, ρ̇, m]`.
#[derive(Debug)]
pub struct LinearSpringMassDamper {
    pub config: LinearSpringMassDamperConfig,
    /// Upstream `rho` \[m\] of the last loaded state.
    pub rho_m: f64,
    /// Upstream `rhoDot` \[m/s\] of the last loaded state.
    pub rho_dot_mps: f64,
    /// Upstream `massSMD` \[kg\] of the last loaded state.
    pub mass_kg: f64,
    back_sub: RefCell<ParticleAccelerationTerms>,
}

impl LinearSpringMassDamper {
    pub fn new(config: LinearSpringMassDamperConfig) -> Self {
        let rho_m = config.rho_init_m;
        let rho_dot_mps = config.rho_dot_init_mps;
        let mass_kg = config.mass_init_kg;
        Self {
            config,
            rho_m,
            rho_dot_mps,
            mass_kg,
            back_sub: RefCell::new(ParticleAccelerationTerms::default()),
        }
    }

    /// Particle position `r_PcB_B` \[m\] and its body-frame derivative
    /// `rPrime_PcB_B` \[m/s\] for a state.
    fn particle_position(&self, effector_state: &[f64]) -> (Vector3<f64>, Vector3<f64>) {
        assert_eq!(
            effector_state.len(),
            3,
            "linear spring mass damper state length mismatch"
        );
        let p_hat = self.config.direction_body;
        (
            effector_state[0] * p_hat + self.config.equilibrium_position_body_m,
            effector_state[1] * p_hat,
        )
    }
}

impl StateEffector for LinearSpringMassDamper {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        3
    }

    /// Validates the configuration (upstream `registerStates`) and returns the
    /// initial `[ρ, ρ̇, m]`.
    ///
    /// # Panics
    ///
    /// If [`LinearSpringMassDamperConfig::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid LinearSpringMassDamperConfig '{}': {message}",
                self.config.name
            );
        }
        vec![
            self.config.rho_init_m,
            self.config.rho_dot_init_mps,
            self.config.mass_init_kg,
        ]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "linear spring mass damper state length mismatch"
        );
        self.rho_m = state[0];
        self.rho_dot_mps = state[1];
        self.mass_kg = state[2];
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}

    /// Upstream `updateContributions`. A massless particle contributes nothing
    /// and has zero acceleration.
    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let body_omega_radps = hub_state.omega_radps;
        let (r_pc, r_prime_pc) = self.particle_position(effector_state);
        let mass = self.fuel_mass_kg(effector_state);
        if mass <= 0.0 {
            self.back_sub.replace(ParticleAccelerationTerms::default());
            return;
        }
        let (rho, rho_dot) = (effector_state[0], effector_state[1]);
        let p_hat = self.config.direction_body;
        let omega = body_omega_radps;

        let a_rho = -p_hat;
        let b_rho = p_hat.cross(&r_pc);
        let c_rho = 1.0 / mass
            * (p_hat.dot(&(mass * gravity_body_mps2))
                - self.config.spring_constant_npm * rho
                - self.config.damping_ns_per_m * rho_dot
                - 2.0 * mass * p_hat.dot(&omega.cross(&r_prime_pc))
                - mass * p_hat.dot(&omega.cross(&omega.cross(&r_pc))));

        back_sub.matrix_a += mass * p_hat * a_rho.transpose();
        back_sub.matrix_b += mass * p_hat * b_rho.transpose();
        back_sub.matrix_c += -mass * b_rho * a_rho.transpose();
        back_sub.matrix_d += -mass * b_rho * b_rho.transpose();
        back_sub.vec_trans += -mass * c_rho * p_hat;
        back_sub.vec_rot += -mass * omega.cross(&r_pc.cross(&r_prime_pc)) + mass * c_rho * b_rho;
        self.back_sub.replace(ParticleAccelerationTerms {
            a_rho,
            b_rho,
            c_rho,
        });
    }

    /// The derivatives at a constant mass: the particle on its own.
    fn compute_derivatives(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
    ) -> Vec<f64> {
        self.compute_derivatives_with_flow(
            effector_state,
            body_trans_accel_mps2,
            body_omega_dot_radps2,
            FuelSloshFlow::default(),
        )
    }

    /// Upstream `updateEffectorMassProps` at a constant mass.
    fn mass_properties(&self, effector_state: &[f64]) -> StateEffectorMassProps {
        self.mass_properties_with_flow(effector_state, FuelSloshFlow::default())
            .dynamics
    }

    /// Upstream `updateEnergyMomContributions` (angular momentum about B).
    fn rotational_angular_momentum_body(
        &self,
        effector_state: &[f64],
        body_omega_radps: Vector3<f64>,
    ) -> Vector3<f64> {
        let (r_pc, r_prime_pc) = self.particle_position(effector_state);
        let r_dot = r_prime_pc + body_omega_radps.cross(&r_pc);
        self.fuel_mass_kg(effector_state) * r_pc.cross(&r_dot)
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B
    /// plus the spring energy.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let (r_pc, r_prime_pc) = self.particle_position(effector_state);
        let r_dot = r_prime_pc + body_omega_radps.cross(&r_pc);
        let rho = effector_state[0];
        0.5 * self.fuel_mass_kg(effector_state) * r_dot.dot(&r_dot)
            + 0.5 * self.config.spring_constant_npm * rho * rho
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl FuelSlosh for LinearSpringMassDamper {
    fn fuel_mass_index(&self) -> usize {
        2
    }

    /// Upstream `updateEffectorMassProps` with the tank's `fuelMassDot`.
    fn mass_properties_with_flow(
        &self,
        effector_state: &[f64],
        flow: FuelSloshFlow,
    ) -> DepletionMassProps {
        let (r_pc, r_prime_pc) = self.particle_position(effector_state);
        point_mass_slosh_props(self.fuel_mass_kg(effector_state), r_pc, r_prime_pc, flow)
    }

    fn compute_derivatives_with_flow(
        &self,
        effector_state: &[f64],
        body_trans_accel_mps2: Vector3<f64>,
        body_omega_dot_radps2: Vector3<f64>,
        flow: FuelSloshFlow,
    ) -> Vec<f64> {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "linear spring mass damper state length mismatch"
        );
        let terms = self.back_sub.borrow();
        vec![
            effector_state[1],
            terms.a_rho.dot(&body_trans_accel_mps2)
                + terms.b_rho.dot(&body_omega_dot_radps2)
                + terms.c_rho,
            flow.fuel_mass_dot_kgps,
        ]
    }
}

/// Upstream `aRho`, `bRho`, `cRho`: `ρ̈ = aRho·r̈_B + bRho·ω̇ + cRho`.
#[derive(Clone, Debug, Default)]
struct ParticleAccelerationTerms {
    a_rho: Vector3<f64>,
    b_rho: Vector3<f64>,
    c_rho: f64,
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_linearSpringMassDamper.py`.

    use super::{LinearSpringMassDamper, LinearSpringMassDamperConfig};
    use crate::dynamics::fuel_tank::FuelTank;
    use crate::dynamics::state_effector_test_support::{
        HubSetup, add_point_mass_earth, assert_conserved, run_inspecting_every_step,
        run_logging_every_step, slosh_depletion_tank, spacecraft_with_effectors,
    };
    use crate::spacecraft::{BackSubMatrices, StateEffector};
    use nalgebra::Vector3;

    fn particle_config(
        name: &str,
        equilibrium: [f64; 3],
        direction: [f64; 3],
        rho_init_m: f64,
        mass_kg: f64,
        damping_ns_per_m: f64,
    ) -> LinearSpringMassDamperConfig {
        let mut config = LinearSpringMassDamperConfig::new(name);
        config.spring_constant_npm = 100.0;
        config.damping_ns_per_m = damping_ns_per_m;
        config.equilibrium_position_body_m = Vector3::from(equilibrium);
        config.direction_body = Vector3::from(direction) * (3.0_f64.sqrt() / 3.0);
        config.rho_init_m = rho_init_m;
        config.mass_init_kg = mass_kg;
        config
    }

    /// The three particles of upstream `fuelSloshTest`.
    fn particle_configs(damping: impl Fn(f64) -> f64) -> [LinearSpringMassDamperConfig; 3] {
        [
            particle_config(
                "particle1",
                [0.1, 0.0, -0.1],
                [1.0, 1.0, 1.0],
                0.05,
                10.0,
                damping(15.0),
            ),
            particle_config(
                "particle2",
                [0.0, 0.0, 0.1],
                [1.0, -1.0, -1.0],
                -0.025,
                20.0,
                damping(17.0),
            ),
            particle_config(
                "particle3",
                [-0.1, 0.0, 0.1],
                [-1.0, -1.0, 1.0],
                -0.015,
                15.0,
                damping(11.0),
            ),
        ]
    }

    /// Upstream `fuelSloshTest`'s hub, at rest at the origin but tumbling.
    fn tumbling_hub() -> HubSetup {
        let mut hub = HubSetup::hinged_body_hub();
        hub.r_cn_n_m = Vector3::new(0.5, 0.4, -0.7);
        hub.v_cn_n_mps = Vector3::new(0.1, -5.0, 0.3);
        hub.omega_bn_b_radps = Vector3::new(0.1, -0.1, 0.1);
        hub
    }

    /// Upstream `fuelSloshTest`: three particles on a tumbling hub for 2.5 s at
    /// a 1 ms step.
    fn fuel_slosh_test(test_case: &str) {
        let damped = test_case == "Damping";
        let damping = |c: f64| if damped { c } else { 0.0 };
        let particles: Vec<Box<dyn StateEffector>> = particle_configs(damping)
            .into_iter()
            .map(|config| Box::new(LinearSpringMassDamper::new(config)) as Box<dyn StateEffector>)
            .collect();
        let mut hub = tumbling_hub();
        let gravity = test_case == "Gravity";
        if gravity {
            hub = hub.in_gravity_case_orbit();
        }
        let mut spacecraft = spacecraft_with_effectors(&hub, 1_000_000, particles);
        if gravity {
            add_point_mass_earth(&mut spacecraft);
        }

        let log = run_logging_every_step(&mut spacecraft, 2_500, || {});

        assert_conserved(&log[0], &log[log.len() - 1], 1e-10, !damped);
    }

    #[test]
    fn linear_spring_mass_damper_conserves_energy_and_momentum_without_gravity() {
        fuel_slosh_test("NoGravity");
    }

    #[test]
    fn linear_spring_mass_damper_conserves_energy_and_momentum_with_gravity() {
        fuel_slosh_test("Gravity");
    }

    #[test]
    fn damped_linear_spring_mass_damper_conserves_momentum_and_orbital_energy() {
        fuel_slosh_test("Damping");
    }

    /// Upstream `fuelSloshTest`'s `MassDepletion` case: the three particles
    /// pushed onto a 40 kg tank that a 445 N thruster drains for 5 s; each
    /// particle drains at its share of the thruster's flow.
    #[test]
    fn particles_drain_at_their_share_of_the_tank_flow() {
        let (mut tank, thruster) = slosh_depletion_tank(5.0);
        for config in particle_configs(|_| 0.0) {
            tank.push_fuel_slosh_particle(LinearSpringMassDamper::new(config));
        }
        let mut spacecraft =
            spacecraft_with_effectors(&tumbling_hub(), 1_000_000, vec![Box::new(tank)]);
        spacecraft.add_dynamic_effector(thruster);

        let mut masses: Vec<[f64; 3]> = Vec::new();
        run_inspecting_every_step(&mut spacecraft, 10_000, |spacecraft| {
            let tank = spacecraft.state_effectors[0]
                .as_any()
                .downcast_ref::<FuelTank>()
                .expect("tank");
            let mass = |index: usize| {
                tank.fuel_slosh_particles()[index]
                    .as_any()
                    .downcast_ref::<LinearSpringMassDamper>()
                    .expect("particle")
                    .mass_kg
            };
            masses.push([mass(0), mass(1), mass(2)]);
        });

        let m_dot_fuel = -0.19392039093;
        let accuracy = 1e-4;
        for (index, share) in [10.0 / 85.0, 20.0 / 85.0, 15.0 / 85.0]
            .into_iter()
            .enumerate()
        {
            let m_dot = (masses[2][index] - masses[1][index]) / 1e-3;
            assert!(
                (m_dot - m_dot_fuel * share).abs() <= accuracy,
                "particle {}: {m_dot} vs {}",
                index + 1,
                m_dot_fuel * share
            );
        }
    }

    /// Upstream's `massSMD <= 0` branch: a massless particle neither couples
    /// to the hub nor accelerates.
    #[test]
    fn massless_particle_contributes_nothing() {
        let mut config = LinearSpringMassDamperConfig::new("empty");
        config.rho_init_m = 0.2;
        config.rho_dot_init_mps = 0.1;
        let particle = LinearSpringMassDamper::new(config);
        let mut back_sub = BackSubMatrices::default();

        particle.update_contributions(
            &[0.2, 0.1, 0.0],
            &crate::messages::SpacecraftStateMsg {
                omega_radps: Vector3::new(0.1, 0.2, 0.3),
                ..Default::default()
            },
            Vector3::new(0.0, 0.0, -9.81),
            &mut back_sub,
        );

        assert_eq!(back_sub.matrix_a.norm(), 0.0);
        assert_eq!(back_sub.matrix_d.norm(), 0.0);
        assert_eq!(back_sub.vec_trans.norm(), 0.0);
        assert_eq!(back_sub.vec_rot.norm(), 0.0);
        let derivatives = particle.compute_derivatives(
            &[0.2, 0.1, 0.0],
            Vector3::new(1.0, 2.0, 3.0),
            Vector3::x(),
        );
        assert_eq!(derivatives, vec![0.1, 0.0, 0.0]);
        assert_eq!(particle.mass_properties(&[0.2, 0.1, 0.0]).mass_kg, 0.0);
    }

    /// A particle at rest on a static hub accelerates as `−k ρ / m` along its
    /// direction.
    #[test]
    fn spring_accelerates_particle_toward_equilibrium() {
        let mut config = LinearSpringMassDamperConfig::new("particle");
        config.spring_constant_npm = 100.0;
        config.mass_init_kg = 10.0;
        config.direction_body = Vector3::y();
        let particle = LinearSpringMassDamper::new(config);
        let mut back_sub = BackSubMatrices::default();

        particle.update_contributions(
            &[0.05, 0.0, 10.0],
            &crate::messages::SpacecraftStateMsg::default(),
            Vector3::zeros(),
            &mut back_sub,
        );
        let derivatives =
            particle.compute_derivatives(&[0.05, 0.0, 10.0], Vector3::zeros(), Vector3::zeros());

        assert_eq!(derivatives[0], 0.0);
        assert!((derivatives[1] + 0.5).abs() < 1e-15);
        assert!((back_sub.vec_trans - Vector3::new(0.0, 5.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn validation_rejects_negative_and_non_finite_mass() {
        let mut config = LinearSpringMassDamperConfig::new("particle");
        assert_eq!(config.validate(), Ok(()));
        for mass_kg in [-1.0, f64::NAN, f64::INFINITY] {
            config.mass_init_kg = mass_kg;
            assert!(config.validate().is_err(), "mass {mass_kg} accepted");
        }
    }

    #[test]
    #[should_panic(expected = "mass_init_kg must be finite and non-negative")]
    fn spacecraft_initialization_rejects_negative_mass() {
        let mut config = LinearSpringMassDamperConfig::new("particle");
        config.mass_init_kg = -1.0;
        let _ = LinearSpringMassDamper::new(config).initial_state();
    }
}
