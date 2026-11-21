//! A point mass on a massless rod pivoting about a tank center: port of
//! upstream Basilisk `simulation/dynamics/sphericalPendulum`
//! (`SphericalPendulum`), a fuel-slosh model.
//!
//! The pendulum mass sits at `l` from the tank center T (`d` from the body
//! origin), with `|l|` the pendulum radius. Its direction is given by two
//! angles in the pendulum frame P0 (axes `p̂01`, `p̂02`, `p̂03`, fixed in the body
//! frame): `l_P0 = R [cos φ cos θ, sin φ cos θ, −sin θ]`. A damping matrix `D`
//! (P0 components) resists the motion of the mass relative to the tank. The
//! states `[φ, θ, φ̇, θ̇, m]` are coupled to the hub by back-substitution.
//!
//! The pendulum mass `m` is constant on its own. Pushed onto a
//! [`FuelTank`](super::fuel_tank::FuelTank) as fuel slosh
//! ([`FuelTank::push_fuel_slosh_particle`](super::fuel_tank::FuelTank::push_fuel_slosh_particle),
//! upstream `pushFuelSloshParticle`), it is a [`FuelSlosh`] particle the tank
//! drains (see [`super::fuel_slosh`]).
//!
//! The angles are singular at `θ = ±90°`. Called after an integration step,
//! [`StateEffector::modify_states`] (upstream `modifyStates`) moves P0 onto the
//! current pendulum direction whenever `|cos θ| < √2/2`, resetting
//! `φ = θ = 0` and rewriting the rates for the new frame; the spacecraft must
//! call it after every step for the angles to stay clear of the singularity.
//! The frame in use is [`SphericalPendulum::pendulum_frame_body`]; like
//! upstream, it is kept when the spacecraft is initialized again.
//!
//! Differences from upstream:
//! - Upstream folds φ and θ back into `[−2π, 2π]` inside
//!   `updateEffectorMassProps`, rewriting the state in the middle of an
//!   integrator stage. Here the fold happens in `modify_states`, after the
//!   step; the dynamics are 2π-periodic in both angles, so only the reported
//!   angles differ.
//! - The pendulum radius must be finite and positive; upstream leaves it
//!   uninitialized and does not check it.

use nalgebra::{Matrix3, Vector3};
use std::any::Any;
use std::cell::RefCell;
use std::f64::consts::{FRAC_1_SQRT_2, PI};

use super::fuel_slosh::{DepletionMassProps, FuelSlosh, FuelSloshFlow, point_mass_slosh_props};
use crate::messages::SpacecraftStateMsg;
use crate::spacecraft::{BackSubMatrices, StateEffector, StateEffectorMassProps};
use crate::utilities::avs_eigen_support::{DEFAULT_TOLERANCE, is_positive_semidefinite};

/// User-settable parameters of [`SphericalPendulum`].
#[derive(Clone, Debug)]
pub struct SphericalPendulumConfig {
    /// Effector name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `pendulumRadius` \[m\]: distance from the tank center to the
    /// pendulum mass (no upstream default; 0 here, which fails validation).
    pub pendulum_radius_m: f64,
    /// Upstream `d` \[m\]: tank center T relative to the body origin (default
    /// zero).
    pub tank_center_body_m: Vector3<f64>,
    /// Upstream `D` \[N s/m\]: symmetric positive-semidefinite damping matrix in
    /// P0 components (default zero).
    pub damping_matrix_p0_ns_per_m: Matrix3<f64>,
    /// Upstream `phiDotInit` \[rad/s\] (default 0). The initial angles are zero,
    /// as upstream keeps `phiInit` and `thetaInit` private: P0 sets the initial
    /// pendulum direction `p̂01`.
    pub phi_dot_init_radps: f64,
    /// Upstream `thetaDotInit` \[rad/s\] (default 0).
    pub theta_dot_init_radps: f64,
    /// Upstream `massInit` \[kg\]: initial pendulum mass, the start of its
    /// mass state (default 1).
    pub mass_init_kg: f64,
    /// Upstream `pHat_01`: first P0 axis in body-frame components (default x̂).
    pub p_hat_01_body: Vector3<f64>,
    /// Upstream `pHat_02` (default ŷ).
    pub p_hat_02_body: Vector3<f64>,
    /// Upstream `pHat_03` (default ẑ).
    pub p_hat_03_body: Vector3<f64>,
}

impl SphericalPendulumConfig {
    /// A configuration with upstream's default values.
    pub fn new(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            pendulum_radius_m: 0.0,
            tank_center_body_m: Vector3::zeros(),
            damping_matrix_p0_ns_per_m: Matrix3::zeros(),
            phi_dot_init_radps: 0.0,
            theta_dot_init_radps: 0.0,
            mass_init_kg: 1.0,
            p_hat_01_body: Vector3::x(),
            p_hat_02_body: Vector3::y(),
            p_hat_03_body: Vector3::z(),
        }
    }

    /// Upstream `validateConfiguration`: the initial mass finite and
    /// non-negative and `D` symmetric positive semidefinite. Also requires a
    /// finite, positive pendulum radius.
    pub fn validate(&self) -> Result<(), String> {
        if !self.mass_init_kg.is_finite() || self.mass_init_kg < 0.0 {
            return Err("mass_init_kg must be finite and non-negative".to_string());
        }
        if !is_positive_semidefinite(&self.damping_matrix_p0_ns_per_m, DEFAULT_TOLERANCE) {
            return Err("D must be symmetric positive semidefinite".to_string());
        }
        if !self.pendulum_radius_m.is_finite() || self.pendulum_radius_m <= 0.0 {
            return Err("pendulum_radius_m must be finite and positive".to_string());
        }
        Ok(())
    }

    /// `[B P0]`, whose columns are the P0 axes (upstream `dcm_B_P0`).
    fn pendulum_frame_body(&self) -> Matrix3<f64> {
        Matrix3::from_columns(&[self.p_hat_01_body, self.p_hat_02_body, self.p_hat_03_body])
    }
}

/// Spherical pendulum fuel-slosh model (upstream `SphericalPendulum`).
///
/// State layout: `[φ, θ, φ̇, θ̇, m]`.
#[derive(Debug)]
pub struct SphericalPendulum {
    pub config: SphericalPendulumConfig,
    /// Upstream `phi`, `theta` \[rad\] of the last loaded state.
    pub angles_rad: [f64; 2],
    /// Upstream `phiDot`, `thetaDot` \[rad/s\] of the last loaded state.
    pub angle_rates_radps: [f64; 2],
    /// Upstream `massFSP` \[kg\] of the last loaded state.
    pub mass_kg: f64,
    /// Upstream `dcm_B_P0` (columns `pHat_01`, `pHat_02`, `pHat_03`), moved by
    /// `modify_states`.
    dcm_b_p0: Matrix3<f64>,
    back_sub: RefCell<PendulumAccelerationTerms>,
}

impl SphericalPendulum {
    pub fn new(config: SphericalPendulumConfig) -> Self {
        let dcm_b_p0 = config.pendulum_frame_body();
        let angle_rates_radps = [config.phi_dot_init_radps, config.theta_dot_init_radps];
        let mass_kg = config.mass_init_kg;
        Self {
            config,
            angles_rad: [0.0; 2],
            angle_rates_radps,
            mass_kg,
            dcm_b_p0,
            back_sub: RefCell::new(PendulumAccelerationTerms::default()),
        }
    }

    /// The P0 frame in use, `[B P0]`: its columns are `p̂01`, `p̂02`, `p̂03`.
    pub fn pendulum_frame_body(&self) -> Matrix3<f64> {
        self.dcm_b_p0
    }

    fn kinematics(&self, effector_state: &[f64]) -> PendulumKinematics {
        assert_eq!(
            effector_state.len(),
            5,
            "spherical pendulum state length mismatch"
        );
        PendulumKinematics::new(
            self.config.pendulum_radius_m,
            self.config.tank_center_body_m,
            self.dcm_b_p0,
            effector_state,
        )
    }

    /// Upstream `updateContributions` for a pendulum with mass.
    fn back_sub_terms(
        &self,
        k: &PendulumKinematics,
        mass: f64,
        body_omega_radps: Vector3<f64>,
        gravity_body_mps2: Vector3<f64>,
    ) -> (BackSubMatrices, PendulumAccelerationTerms) {
        let radius = self.config.pendulum_radius_m;
        let d = self.config.tank_center_body_m;
        let omega = body_omega_radps;
        let (p_hat_01, p_hat_02, p_hat_03) = (
            self.dcm_b_p0.column(0).into_owned(),
            self.dcm_b_p0.column(1).into_owned(),
            self.dcm_b_p0.column(2).into_owned(),
        );
        let (sin_phi, cos_phi) = k.phi.sin_cos();
        let (sin_theta, cos_theta) = k.theta.sin_cos();
        let radius_squared = radius * radius;

        let p_hat_03_cross_l = p_hat_03.cross(&k.l_b);
        let a_phi = -p_hat_03_cross_l / (radius_squared * cos_theta * cos_theta);
        let b_phi = p_hat_03_cross_l.cross(&(k.l_b + d)) / (radius_squared * cos_theta * cos_theta);
        let l_t = k.l_b.cross(
            &(mass * gravity_body_mps2
                - self.dcm_b_p0 * self.config.damping_matrix_p0_ns_per_m * k.l_prime_p0),
        );
        let omega_cross_omega_cross_d = omega.cross(&omega.cross(&d));
        let relative_accel_terms =
            2.0 * omega.cross(&k.l_prime_b) + omega.cross(&omega.cross(&k.l_b));
        let c_phi = 1.0 / (mass * radius_squared * cos_theta * cos_theta)
            * (-mass * p_hat_03.dot(&k.l_b.cross(&omega_cross_omega_cross_d))
                + p_hat_03.dot(&l_t)
                + 2.0 * mass * radius_squared * k.phi_dot * k.theta_dot * cos_theta * sin_theta
                - mass * p_hat_03.dot(&k.l_b.cross(&relative_accel_terms)));

        let p_hat_02_prime = self.dcm_b_p0 * Vector3::new(-sin_phi, cos_phi, 0.0);
        let p_hat_02_prime_cross_l = p_hat_02_prime.cross(&k.l_b);
        let a_theta = -p_hat_02_prime_cross_l / radius_squared;
        let b_theta = p_hat_02_prime_cross_l.cross(&(k.l_b + d)) / radius_squared;
        let c_theta = 1.0 / (mass * radius_squared)
            * (-mass * p_hat_02_prime.dot(&k.l_b.cross(&omega_cross_omega_cross_d))
                + p_hat_02_prime.dot(&l_t)
                - mass * radius_squared * k.phi_dot * k.phi_dot * cos_theta * sin_theta
                - mass * p_hat_02_prime.dot(&k.l_b.cross(&relative_accel_terms)));

        let phi_basis = sin_phi * cos_theta * p_hat_01 - cos_phi * cos_theta * p_hat_02;
        let theta_basis =
            cos_phi * sin_theta * p_hat_01 + sin_phi * sin_theta * p_hat_02 + cos_theta * p_hat_03;
        let scale = -mass * radius;
        let r_cross_phi_basis = k.r_pc_b.cross(&phi_basis);
        let r_cross_theta_basis = k.r_pc_b.cross(&theta_basis);
        let pendulum_acceleration_term = (-cos_phi * cos_theta * p_hat_01
            - sin_phi * cos_theta * p_hat_02)
            * k.phi_dot
            * k.phi_dot
            + (-cos_phi * cos_theta * p_hat_01 - sin_phi * cos_theta * p_hat_02
                + sin_theta * p_hat_03)
                * k.theta_dot
                * k.theta_dot
            + (2.0 * sin_phi * sin_theta * p_hat_01 - 2.0 * cos_phi * sin_theta * p_hat_02)
                * k.phi_dot
                * k.theta_dot
            - phi_basis * c_phi
            - theta_basis * c_theta;

        (
            BackSubMatrices {
                matrix_a: scale
                    * (phi_basis * a_phi.transpose() + theta_basis * a_theta.transpose()),
                matrix_b: scale
                    * (phi_basis * b_phi.transpose() + theta_basis * b_theta.transpose()),
                matrix_c: scale
                    * (r_cross_phi_basis * a_phi.transpose()
                        + r_cross_theta_basis * a_theta.transpose()),
                matrix_d: scale
                    * (r_cross_phi_basis * b_phi.transpose()
                        + r_cross_theta_basis * b_theta.transpose()),
                vec_trans: scale * pendulum_acceleration_term,
                vec_rot: -mass
                    * (omega.cross(&k.r_pc_b.cross(&k.l_prime_b))
                        + radius * k.r_pc_b.cross(&pendulum_acceleration_term)),
            },
            PendulumAccelerationTerms {
                a_phi,
                b_phi,
                c_phi,
                a_theta,
                b_theta,
                c_theta,
            },
        )
    }
}

impl StateEffector for SphericalPendulum {
    fn name(&self) -> &str {
        &self.config.name
    }

    fn state_len(&self) -> usize {
        5
    }

    /// Validates the configuration (upstream `registerStates`) and returns the
    /// initial `[0, 0, φ̇0, θ̇0, m0]`.
    ///
    /// # Panics
    ///
    /// If [`SphericalPendulumConfig::validate`] fails.
    fn initial_state(&self) -> Vec<f64> {
        if let Err(message) = self.config.validate() {
            panic!(
                "invalid SphericalPendulumConfig '{}': {message}",
                self.config.name
            );
        }
        vec![
            0.0,
            0.0,
            self.config.phi_dot_init_radps,
            self.config.theta_dot_init_radps,
            self.config.mass_init_kg,
        ]
    }

    fn load_state(&mut self, state: &[f64]) {
        assert_eq!(
            state.len(),
            self.state_len(),
            "spherical pendulum state length mismatch"
        );
        self.angles_rad = [state[0], state[1]];
        self.angle_rates_radps = [state[2], state[3]];
        self.mass_kg = state[4];
    }

    fn pre_integration(&mut self, _current_sim_nanos: u64, _dt_seconds: f64) {}

    /// Upstream `updateContributions`. A massless pendulum contributes nothing
    /// and has zero angular accelerations.
    fn update_contributions(
        &self,
        effector_state: &[f64],
        hub_state: &SpacecraftStateMsg,
        gravity_body_mps2: Vector3<f64>,
        back_sub: &mut BackSubMatrices,
    ) {
        let body_omega_radps = hub_state.omega_radps;
        let kinematics = self.kinematics(effector_state);
        let mass = self.fuel_mass_kg(effector_state);
        if mass <= 0.0 {
            self.back_sub.replace(PendulumAccelerationTerms::default());
            return;
        }
        let (terms, acceleration) =
            self.back_sub_terms(&kinematics, mass, body_omega_radps, gravity_body_mps2);
        back_sub.matrix_a += terms.matrix_a;
        back_sub.matrix_b += terms.matrix_b;
        back_sub.matrix_c += terms.matrix_c;
        back_sub.matrix_d += terms.matrix_d;
        back_sub.vec_trans += terms.vec_trans;
        back_sub.vec_rot += terms.vec_rot;
        self.back_sub.replace(acceleration);
    }

    /// The derivatives at a constant mass: the pendulum on its own.
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
        let k = self.kinematics(effector_state);
        let r_dot = k.l_prime_b + body_omega_radps.cross(&k.r_pc_b);
        self.fuel_mass_kg(effector_state) * k.r_pc_b.cross(&r_dot)
    }

    /// Upstream `updateEnergyMomContributions`: kinetic energy relative to B.
    fn rotational_energy_j(&self, effector_state: &[f64], body_omega_radps: Vector3<f64>) -> f64 {
        let k = self.kinematics(effector_state);
        let r_dot = k.l_prime_b + body_omega_radps.cross(&k.r_pc_b);
        0.5 * self.fuel_mass_kg(effector_state) * r_dot.dot(&r_dot)
    }

    /// Upstream `modifyStates`, with the angle folding upstream does in
    /// `updateEffectorMassProps`: folds φ and θ into `[−2π, 2π]`, then, when
    /// `|cos θ| < √2/2`, rotates P0 onto the pendulum direction and rewrites
    /// the state as `[0, 0, φ̇', θ̇', m]` in the new frame.
    fn modify_states(&mut self, _current_sim_nanos: u64, effector_state: &mut [f64]) {
        assert_eq!(
            effector_state.len(),
            self.state_len(),
            "spherical pendulum state length mismatch"
        );
        for angle in &mut effector_state[..2] {
            if *angle > 2.0 * PI {
                *angle -= 2.0 * PI;
            }
            if *angle < -2.0 * PI {
                *angle += 2.0 * PI;
            }
        }
        let k = self.kinematics(effector_state);
        if k.theta.cos().abs() < FRAC_1_SQRT_2 {
            let (sin_phi, cos_phi) = k.phi.sin_cos();
            let (sin_theta, cos_theta) = k.theta.sin_cos();
            let dcm_p0_p0new = Matrix3::new(
                cos_phi * cos_theta,
                -sin_phi,
                cos_phi * sin_theta,
                sin_phi * cos_theta,
                cos_phi,
                sin_phi * sin_theta,
                -sin_theta,
                0.0,
                cos_theta,
            );
            self.dcm_b_p0 *= dcm_p0_p0new;
            let l_prime_p0new = self.dcm_b_p0.transpose() * k.l_prime_b;
            let radius = self.config.pendulum_radius_m;
            effector_state[..4].copy_from_slice(&[
                0.0,
                0.0,
                l_prime_p0new.y / radius,
                -l_prime_p0new.z / radius,
            ]);
        }
        self.load_state(effector_state);
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

impl FuelSlosh for SphericalPendulum {
    fn fuel_mass_index(&self) -> usize {
        4
    }

    /// Upstream `updateEffectorMassProps` with the tank's `fuelMassDot`.
    fn mass_properties_with_flow(
        &self,
        effector_state: &[f64],
        flow: FuelSloshFlow,
    ) -> DepletionMassProps {
        let k = self.kinematics(effector_state);
        point_mass_slosh_props(
            self.fuel_mass_kg(effector_state),
            k.r_pc_b,
            k.l_prime_b,
            flow,
        )
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
            "spherical pendulum state length mismatch"
        );
        let terms = self.back_sub.borrow();
        vec![
            effector_state[2],
            effector_state[3],
            terms.a_phi.dot(&body_trans_accel_mps2)
                + terms.b_phi.dot(&body_omega_dot_radps2)
                + terms.c_phi,
            terms.a_theta.dot(&body_trans_accel_mps2)
                + terms.b_theta.dot(&body_omega_dot_radps2)
                + terms.c_theta,
            flow.fuel_mass_dot_kgps,
        ]
    }
}

/// Upstream `aPhi`, `bPhi`, `cPhi`, `aTheta`, `bTheta`, `cTheta`:
/// `φ̈ = aPhi·r̈_B + bPhi·ω̇ + cPhi`, and likewise for θ̈.
#[derive(Clone, Debug, Default)]
struct PendulumAccelerationTerms {
    a_phi: Vector3<f64>,
    b_phi: Vector3<f64>,
    c_phi: f64,
    a_theta: Vector3<f64>,
    b_theta: Vector3<f64>,
    c_theta: f64,
}

/// Pendulum geometry for one state (upstream `updateEffectorMassProps`).
#[derive(Clone, Debug)]
struct PendulumKinematics {
    phi: f64,
    theta: f64,
    phi_dot: f64,
    theta_dot: f64,
    /// Upstream `l_B`.
    l_b: Vector3<f64>,
    /// Upstream `lPrime_P0`.
    l_prime_p0: Vector3<f64>,
    /// Upstream `lPrime_B` (= `rPrime_PcB_B`).
    l_prime_b: Vector3<f64>,
    /// Upstream `r_PcB_B`.
    r_pc_b: Vector3<f64>,
}

impl PendulumKinematics {
    fn new(
        radius: f64,
        tank_center_b: Vector3<f64>,
        dcm_b_p0: Matrix3<f64>,
        state: &[f64],
    ) -> Self {
        let (phi, theta, phi_dot, theta_dot) = (state[0], state[1], state[2], state[3]);
        let (sin_phi, cos_phi) = phi.sin_cos();
        let (sin_theta, cos_theta) = theta.sin_cos();
        let l_p0 = radius * Vector3::new(cos_phi * cos_theta, sin_phi * cos_theta, -sin_theta);
        let l_prime_p0 = radius
            * Vector3::new(
                -phi_dot * sin_phi * cos_theta - theta_dot * cos_phi * sin_theta,
                phi_dot * cos_phi * cos_theta - theta_dot * sin_phi * sin_theta,
                -theta_dot * cos_theta,
            );
        let l_b = dcm_b_p0 * l_p0;
        Self {
            phi,
            theta,
            phi_dot,
            theta_dot,
            l_b,
            l_prime_p0,
            l_prime_b: dcm_b_p0 * l_prime_p0,
            r_pc_b: tank_center_b + l_b,
        }
    }
}

#[cfg(test)]
mod tests {
    //! Ports of upstream `test_sphericalPendulum.py`.
    //!
    //! The 10 ms cases (`testCase` 1, 4 and 5) drive θ past 45° and rely on the
    //! frame switch of [`StateEffector::modify_states`], which `Spacecraft`
    //! calls after every integration step. Without it the angles approach the
    //! θ = ±90° singularity and the momentum drift reaches about 1.04e-8, just
    //! over the 1e-8 tolerance. The 1 ms case passes either way.

    use super::{SphericalPendulum, SphericalPendulumConfig};
    use crate::dynamics::fuel_tank::FuelTank;
    use crate::dynamics::state_effector_test_support::{
        HubSetup, MU_EARTH_M3PS2, add_point_mass_earth, run_inspecting_every_step,
        run_logging_every_step, slosh_depletion_tank, spacecraft_with_effectors,
    };
    use crate::spacecraft::{BackSubMatrices, StateEffector};
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state};
    use nalgebra::{Matrix3, Vector3};
    use std::f64::consts::FRAC_1_SQRT_2;

    fn pendulums(damping: Matrix3<f64>) -> Vec<Box<dyn StateEffector>> {
        pendulum_configs(damping)
            .into_iter()
            .map(|config| Box::new(SphericalPendulum::new(config)) as Box<dyn StateEffector>)
            .collect()
    }

    /// The two pendulums of upstream `sphericalPendulumTest`.
    fn pendulum_configs(damping: Matrix3<f64>) -> [SphericalPendulumConfig; 2] {
        let mut config1 = SphericalPendulumConfig::new("pendulum1");
        config1.pendulum_radius_m = 0.3;
        config1.tank_center_body_m = Vector3::new(0.1, 0.1, 0.1);
        config1.damping_matrix_p0_ns_per_m = damping;
        config1.phi_dot_init_radps = 0.01;
        config1.theta_dot_init_radps = 0.05;
        config1.mass_init_kg = 20.0;
        config1.p_hat_01_body = Vector3::new(FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
        config1.p_hat_02_body = Vector3::new(0.0, 1.0, 0.0);
        config1.p_hat_03_body = Vector3::new(-FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);

        let mut config2 = SphericalPendulumConfig::new("pendulum2");
        config2.pendulum_radius_m = 0.4;
        config2.tank_center_body_m = Vector3::new(0.1, 0.1, 0.1);
        config2.damping_matrix_p0_ns_per_m = damping;
        config2.phi_dot_init_radps = 0.1;
        config2.theta_dot_init_radps = 0.5;
        config2.mass_init_kg = 40.0;

        [config1, config2]
    }

    /// Upstream `sphericalPendulumTest`'s hub, in Earth orbit and spinning
    /// fast.
    fn spinning_hub() -> HubSetup {
        let elements = ClassicElements {
            semi_major_axis: 6700.0 * 1000.0,
            eccentricity: 0.01,
            inclination_rad: 30.0_f64.to_radians(),
            raan_rad: 100.0_f64.to_radians(),
            arg_periapsis_rad: 100.0_f64.to_radians(),
            true_anomaly_rad: 0.0,
            ..ClassicElements::default()
        };
        let (r_n, v_n) = elements_to_state(MU_EARTH_M3PS2, &elements);
        HubSetup {
            mass_kg: 1500.0,
            center_of_mass_body_m: Vector3::new(1.0, 0.5, 0.1),
            inertia_kg_m2: Matrix3::from_diagonal(&Vector3::new(900.0, 800.0, 600.0)),
            r_cn_n_m: r_n,
            v_cn_n_mps: v_n,
            sigma_bn: Vector3::zeros(),
            omega_bn_b_radps: Vector3::new(1.0, 0.5, 0.1),
        }
    }

    /// Upstream `sphericalPendulumTest(testCase = 3)`: the pendulums pushed
    /// onto a 40 kg tank that a 445 N thruster drains for 5 s; each pendulum
    /// drains at its share of the thruster's flow.
    #[test]
    fn spherical_pendulums_drain_at_their_share_of_the_tank_flow() {
        let (mut tank, thruster) = slosh_depletion_tank(5.0);
        for config in pendulum_configs(Matrix3::zeros()) {
            tank.push_fuel_slosh_particle(SphericalPendulum::new(config));
        }
        let mut spacecraft =
            spacecraft_with_effectors(&spinning_hub(), 10_000_000, vec![Box::new(tank)]);
        spacecraft.add_dynamic_effector(thruster);
        add_point_mass_earth(&mut spacecraft);

        let mut masses: Vec<[f64; 2]> = Vec::new();
        run_inspecting_every_step(&mut spacecraft, 1_000, |spacecraft| {
            let tank = spacecraft.state_effectors[0]
                .as_any()
                .downcast_ref::<FuelTank>()
                .expect("tank");
            let mass = |index: usize| {
                tank.fuel_slosh_particles()[index]
                    .as_any()
                    .downcast_ref::<SphericalPendulum>()
                    .expect("pendulum")
                    .mass_kg
            };
            masses.push([mass(0), mass(1)]);
        });

        let m_dot_fuel = -0.19392039093;
        let accuracy = 1e-4;
        for (index, share) in [20.0 / 100.0, 40.0 / 100.0].into_iter().enumerate() {
            let m_dot = (masses[2][index] - masses[1][index]) / 0.01;
            assert!(
                (m_dot - m_dot_fuel * share).abs() <= accuracy,
                "pendulum {}: {m_dot} vs {}",
                index + 1,
                m_dot_fuel * share
            );
        }
    }

    /// Upstream `sphericalPendulumTest`: two pendulums on a fast-spinning hub
    /// in Earth orbit for 10 s.
    fn spherical_pendulum_test(test_case: u32) {
        let time_step_nanos = if test_case == 2 {
            1_000_000
        } else {
            10_000_000
        };
        let damping = match test_case {
            4 => Matrix3::from_diagonal(&Vector3::new(10.0, 10.0, 10.0)),
            5 => Matrix3::from_diagonal(&Vector3::new(50.0, 0.0, 0.0)),
            _ => Matrix3::zeros(),
        };
        let hub = spinning_hub();
        let mut spacecraft = spacecraft_with_effectors(&hub, time_step_nanos, pendulums(damping));
        add_point_mass_earth(&mut spacecraft);

        let steps = (10_000_000_000 / time_step_nanos) as usize;
        let log = run_logging_every_step(&mut spacecraft, steps, || {});

        let accuracy = 1e-8;
        let relative = |value: f64, initial: f64| ((value - initial) / initial).abs();
        let first = &log[0];
        for (k, sample) in log.iter().enumerate() {
            for axis in 0..3 {
                let rot = relative(
                    sample.rotational_angular_momentum_inertial_kg_m2ps[axis],
                    first.rotational_angular_momentum_inertial_kg_m2ps[axis],
                );
                assert!(
                    rot <= accuracy,
                    "sample {k}: rotational angular momentum axis {axis}: {rot:.3e}"
                );
                let orb = relative(
                    sample.orbital_angular_momentum_inertial_kg_m2ps[axis],
                    first.orbital_angular_momentum_inertial_kg_m2ps[axis],
                );
                assert!(
                    orb <= accuracy,
                    "sample {k}: orbital angular momentum axis {axis}: {orb:.3e}"
                );
            }
            // Rotational energy is conserved only without damping.
            if test_case == 1 || test_case == 2 {
                let rot = relative(sample.rotational_energy_j, first.rotational_energy_j);
                assert!(rot <= accuracy, "sample {k}: rotational energy: {rot:.3e}");
            }
            let orb = relative(sample.orbital_energy_j, first.orbital_energy_j);
            assert!(orb <= accuracy, "sample {k}: orbital energy: {orb:.3e}");
        }

        if test_case == 4 || test_case == 5 {
            let energies: Vec<f64> = log
                .iter()
                .map(|sample| sample.rotational_energy_j)
                .collect();
            assert!(
                energies.iter().all(|energy| energy.is_finite()),
                "damping produced non-finite rotational energy"
            );
            let e0 = energies[0];
            let net_loss = (e0 - energies[energies.len() - 1]) / e0;
            assert!(
                net_loss >= 1.0e-3,
                "damping did not dissipate rotational energy (net loss {net_loss:.3e}, expected > 1e-3)"
            );
            let max_step_rise = energies
                .windows(2)
                .map(|pair| pair[1] - pair[0])
                .fold(f64::NEG_INFINITY, f64::max)
                / e0;
            assert!(
                max_step_rise <= 1.0e-10,
                "damping injected rotational energy (maximum normalized step rise {max_step_rise:.3e})"
            );
        }
    }

    #[test]
    fn spherical_pendulum_conserves_energy_and_momentum_at_10_ms() {
        spherical_pendulum_test(1);
    }

    #[test]
    fn spherical_pendulum_conserves_energy_and_momentum_at_1_ms() {
        spherical_pendulum_test(2);
    }

    #[test]
    fn spherical_pendulum_isotropic_damping_dissipates_energy() {
        spherical_pendulum_test(4);
    }

    #[test]
    fn spherical_pendulum_anisotropic_damping_dissipates_energy() {
        spherical_pendulum_test(5);
    }

    fn damped_config(damping: Matrix3<f64>) -> SphericalPendulumConfig {
        let mut config = SphericalPendulumConfig::new("pendulum");
        config.pendulum_radius_m = 0.3;
        config.mass_init_kg = 20.0;
        config.damping_matrix_p0_ns_per_m = damping;
        config
    }

    /// Upstream `test_sphericalPendulumAcceptsValidDampingMatrix` and
    /// `test_sphericalPendulumRejectsInvalidDampingMatrix`.
    #[test]
    fn spherical_pendulum_damping_matrix_validation() {
        let valid = [
            Matrix3::zeros(),
            Matrix3::from_diagonal(&Vector3::new(10.0, 10.0, 10.0)),
            Matrix3::from_diagonal(&Vector3::new(50.0, 0.0, 0.0)),
        ];
        for damping in valid {
            assert_eq!(damped_config(damping).validate(), Ok(()), "{damping:?}");
            let _ = SphericalPendulum::new(damped_config(damping)).initial_state();
        }
        let invalid = [
            Matrix3::new(1.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
            Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, 1.0)),
            Matrix3::new(1.0, f64::NAN, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0),
        ];
        for damping in invalid {
            let result = damped_config(damping).validate();
            assert!(
                result.as_ref().is_err_and(
                    |message| message.contains("D must be symmetric positive semidefinite")
                ),
                "{damping:?}: {result:?}"
            );
        }
    }

    #[test]
    #[should_panic(expected = "D must be symmetric positive semidefinite")]
    fn spacecraft_initialization_rejects_indefinite_damping() {
        let damping = Matrix3::from_diagonal(&Vector3::new(1.0, -1.0, 1.0));
        let _ = SphericalPendulum::new(damped_config(damping)).initial_state();
    }

    #[test]
    fn validation_rejects_bad_mass_and_radius() {
        let mut config = damped_config(Matrix3::zeros());
        config.mass_init_kg = -1.0;
        assert!(config.validate().unwrap_err().contains("mass_init_kg"));
        let mut config = damped_config(Matrix3::zeros());
        config.pendulum_radius_m = 0.0;
        assert!(config.validate().unwrap_err().contains("pendulum_radius_m"));
    }

    /// The frame switch keeps the pendulum's position and body-relative
    /// velocity, and puts the pendulum on the new first axis.
    #[test]
    fn frame_switch_preserves_pendulum_position_and_velocity() {
        let mut config = damped_config(Matrix3::zeros());
        config.tank_center_body_m = Vector3::new(0.1, -0.2, 0.3);
        config.p_hat_01_body = Vector3::new(FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
        config.p_hat_03_body = Vector3::new(-FRAC_1_SQRT_2, 0.0, FRAC_1_SQRT_2);
        let mut pendulum = SphericalPendulum::new(config);
        let mut state = [0.7, 1.0, -0.3, 0.4, 20.0];
        let before = pendulum.mass_properties(&state);

        pendulum.modify_states(0, &mut state);

        assert_eq!(state[..2], [0.0, 0.0]);
        let after = pendulum.mass_properties(&state);
        assert!((after.center_of_mass_body_m - before.center_of_mass_body_m).norm() < 1e-15);
        assert!(
            (after.center_of_mass_prime_body_mps - before.center_of_mass_prime_body_mps).norm()
                < 1e-15
        );
        let frame = pendulum.pendulum_frame_body();
        assert!((frame.transpose() * frame - Matrix3::identity()).norm() < 1e-14);
        assert_eq!(pendulum.angles_rad, [0.0, 0.0]);
        assert_eq!(pendulum.angle_rates_radps, [state[2], state[3]]);
    }

    #[test]
    fn modify_states_folds_angles_and_keeps_frame_below_45_degrees() {
        let mut pendulum = SphericalPendulum::new(damped_config(Matrix3::zeros()));
        let mut state = [7.0, -6.5, 0.1, 0.2, 20.0];

        pendulum.modify_states(0, &mut state);

        let two_pi = 2.0 * std::f64::consts::PI;
        assert_eq!(state, [7.0 - two_pi, -6.5 + two_pi, 0.1, 0.2, 20.0]);
        assert_eq!(pendulum.pendulum_frame_body(), Matrix3::identity());
    }

    /// Upstream's `massFSP <= 0` branch: a massless pendulum neither couples
    /// to the hub nor accelerates.
    #[test]
    fn massless_pendulum_contributes_nothing() {
        let mut config = damped_config(Matrix3::identity());
        config.mass_init_kg = 0.0;
        let pendulum = SphericalPendulum::new(config);
        let state = [0.2, 0.3, 0.1, -0.1, 0.0];
        let mut back_sub = BackSubMatrices::default();

        pendulum.update_contributions(
            &state,
            &crate::messages::SpacecraftStateMsg {
                omega_radps: Vector3::new(0.1, 0.2, 0.3),
                ..Default::default()
            },
            Vector3::z(),
            &mut back_sub,
        );

        assert_eq!(back_sub.matrix_a.norm() + back_sub.matrix_d.norm(), 0.0);
        assert_eq!(back_sub.vec_trans.norm() + back_sub.vec_rot.norm(), 0.0);
        let derivatives = pendulum.compute_derivatives(&state, Vector3::x(), Vector3::y());
        assert_eq!(derivatives, vec![0.1, -0.1, 0.0, 0.0, 0.0]);
    }
}
