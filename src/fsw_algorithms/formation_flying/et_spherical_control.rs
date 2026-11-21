//! Electrostatic tractor relative motion control in spherical coordinates:
//! port of upstream Basilisk `etSphericalControl`.
//!
//! A servicer and a debris object charged to potentials of opposite sign
//! attract each other; the electrostatic tractor uses that force to tug the
//! debris to another orbit. This module computes the servicer thrust that
//! keeps the debris at a reference position relative to the servicer
//! (Hogan & Schaub, Relative Motion Control For Two-Spacecraft Electrostatic
//! Orbit Corrections, <https://doi.org/10.2514/1.56118>).
//!
//! The debris position relative to the servicer is described in the
//! servicer's Hill frame by the spherical coordinates `X = (L, θ, φ)`: the
//! separation, the in-plane angle `θ = atan2(x, -y)` and the out-of-plane
//! angle `φ = asin(-z / L)`. With the Clohessy-Wiltshire dynamics written as
//! `Ẍ = [F](X, Ẋ) + [G] u` the feedback acceleration is
//!
//! `u_S = [G]^-1 (-[P] Ẋ - [K] (X - X_r) - [F])`
//!
//! in spherical-frame components, and the thrust compensates the measured
//! electrostatic force `F_c` on both craft:
//!
//! `T = -m_T (u + F_c (1/m_T + 1/m_D))`.
//!
//! This is what upstream's code computes, and its unit test truth agrees.
//! Upstream's `.rst` writes the corrected Eq. 45 of the paper as
//! `T = m_T [u - F_c (1/m_T + 1/m_D)]`, with the opposite sign on `u`; the
//! port follows the code.
//!
//! The thrust is published in inertial and in servicer body components;
//! connect only one of the two downstream, or it is applied twice.

use nalgebra::{Matrix3, Vector3};

use crate::messages::{
    BodyForceCommandMsg, InertialForceCommandMsg, Input, NavigationAttitudeMsg,
    NavigationTranslationMsg, Output, VehicleConfigMsg,
};
use crate::utilities::linear_algebra::{m33_eigen_values, safe_asin};
use crate::utilities::orbital_motion::{hill_frame, state_to_elements};
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// User-settable parameters of [`EtSphericalControl`]. `Default` is
/// upstream's zero-initialized configuration, which [`Module::reset`]
/// reports as incomplete until `l_r_m` is set.
#[derive(Clone, Debug)]
pub struct EtSphericalControlConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu` \[m^3/s^2\]: gravitational parameter of the central body.
    pub mu_m3ps2: f64,
    /// Upstream `L_r` \[m\]: reference separation; must be positive.
    pub l_r_m: f64,
    /// Upstream `theta_r` \[rad\]: reference in-plane angle.
    pub theta_r_rad: f64,
    /// Upstream `phi_r` \[rad\]: reference out-of-plane angle.
    pub phi_r_rad: f64,
    /// Upstream `K`: symmetric positive definite position gain. Upstream
    /// stores it row-major as `double[9]`; build it from that array with
    /// [`Matrix3::from_row_slice`].
    pub k: Matrix3<f64>,
    /// Upstream `P`: symmetric positive definite rate gain, row-major like
    /// `k`.
    pub p: Matrix3<f64>,
}

impl Default for EtSphericalControlConfig {
    fn default() -> Self {
        Self {
            name: "etSphericalControl".to_string(),
            mu_m3ps2: 0.0,
            l_r_m: 0.0,
            theta_r_rad: 0.0,
            phi_r_rad: 0.0,
            k: Matrix3::zeros(),
            p: Matrix3::zeros(),
        }
    }
}

/// Electrostatic tractor relative motion controller: upstream
/// `etSphericalControl`.
#[derive(Clone, Debug)]
pub struct EtSphericalControl {
    /// User-settable parameters.
    pub config: EtSphericalControlConfig,
    /// Upstream `servicerTransInMsg`: servicer inertial position and velocity.
    pub servicer_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `debrisTransInMsg`: debris inertial position and velocity.
    pub debris_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `servicerAttInMsg`: servicer attitude.
    pub servicer_att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `servicerVehicleConfigInMsg`: servicer mass.
    pub servicer_vehicle_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `debrisVehicleConfigInMsg`: debris mass.
    pub debris_vehicle_config_in_msg: Input<VehicleConfigMsg>,
    /// Upstream `eForceInMsg`: electrostatic force on the servicer, inertial
    /// components.
    pub e_force_in_msg: Input<InertialForceCommandMsg>,
    /// Upstream `forceInertialOutMsg`: servicer thrust, inertial components.
    pub force_inertial_out_msg: Output<InertialForceCommandMsg>,
    /// Upstream `forceBodyOutMsg`: servicer thrust, servicer body components.
    pub force_body_out_msg: Output<BodyForceCommandMsg>,
}

/// Debris position relative to the servicer in spherical coordinates.
struct SphericalState {
    /// `X = (L [m], θ [rad], φ [rad])`.
    x: Vector3<f64>,
    /// `Ẋ`, from the debris velocity relative to the rotating Hill frame.
    x_dot: Vector3<f64>,
    /// `[SN]`: inertial to spherical frame.
    dcm_sn: Matrix3<f64>,
}

impl EtSphericalControl {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: EtSphericalControlConfig) -> Self {
        Self {
            config,
            servicer_trans_in_msg: Input::default(),
            debris_trans_in_msg: Input::default(),
            servicer_att_in_msg: Input::default(),
            servicer_vehicle_config_in_msg: Input::default(),
            debris_vehicle_config_in_msg: Input::default(),
            e_force_in_msg: Input::default(),
            force_inertial_out_msg: Output::default(),
            force_body_out_msg: Output::default(),
        }
    }

    /// Upstream `Reset_etSphericalControl`: the module keeps no state, so a
    /// reset only reports missing inputs and invalid parameters. As upstream
    /// it reads the masses from the vehicle configuration inputs here.
    fn check_configuration(&self) {
        let connections = [
            (
                self.servicer_trans_in_msg.is_connected(),
                "servicer_trans_in_msg",
            ),
            (
                self.debris_trans_in_msg.is_connected(),
                "debris_trans_in_msg",
            ),
            (
                self.servicer_att_in_msg.is_connected(),
                "servicer_att_in_msg",
            ),
            (
                self.servicer_vehicle_config_in_msg.is_connected(),
                "servicer_vehicle_config_in_msg",
            ),
            (
                self.debris_vehicle_config_in_msg.is_connected(),
                "debris_vehicle_config_in_msg",
            ),
            (self.e_force_in_msg.is_connected(), "e_force_in_msg"),
        ];
        for (_, port) in connections.iter().filter(|(connected, _)| !connected) {
            log::error!("etSphericalControl.{port} is not connected");
        }

        if self.config.l_r_m <= 0.0 {
            log::error!("etSphericalControl: l_r_m must be set to a positive value");
        }
        if self.servicer_vehicle_config_in_msg.read().mass_kg <= 0.0 {
            log::error!("etSphericalControl: servicer mass must be set to a positive value");
        }
        if self.debris_vehicle_config_in_msg.read().mass_kg <= 0.0 {
            log::error!("etSphericalControl: debris mass must be set to a positive value");
        }
        if has_negative_eigenvalue(&self.config.k) {
            log::error!("etSphericalControl: k must be a positive definite 3 by 3 matrix");
        }
        if has_negative_eigenvalue(&self.config.p) {
            log::error!("etSphericalControl: p must be a positive definite 3 by 3 matrix");
        }
    }

    /// Upstream `calc_RelativeMotionControl`: servicer thrust \[N\] in
    /// inertial and in servicer body components.
    fn control_force(
        &self,
        servicer: &NavigationTranslationMsg,
        debris: &NavigationTranslationMsg,
        servicer_attitude: &NavigationAttitudeMsg,
        servicer_mass_kg: f64,
        debris_mass_kg: f64,
        electrostatic_force_n: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        let config = &self.config;
        let state = spherical_state(servicer, debris);

        let semi_major_axis = state_to_elements(
            config.mu_m3ps2,
            servicer.position_bn_n_m,
            servicer.velocity_bn_n_mps,
        )
        .semi_major_axis;
        let mean_motion =
            (config.mu_m3ps2 / semi_major_axis / semi_major_axis / semi_major_axis).sqrt();
        let natural = natural_dynamics(mean_motion, &state.x, &state.x_dot);

        let length = state.x[0];
        let cos_phi = state.x[2].cos();
        let g_inverse = Matrix3::from_diagonal(&Vector3::new(1.0, length * cos_phi, -length));
        let x_r = Vector3::new(config.l_r_m, config.theta_r_rad, config.phi_r_rad);
        // Subtracted term by term from zero, as upstream.
        let u_s = g_inverse
            * (Vector3::zeros() - config.p * state.x_dot - config.k * (state.x - x_r) - natural);

        let inverse_reduced_mass = 1.0 / servicer_mass_kg + 1.0 / debris_mass_kg;
        let electrostatic_s = state.dcm_sn * electrostatic_force_n;
        let thrust_s = (u_s + electrostatic_s * inverse_reduced_mass) * -servicer_mass_kg;
        let thrust_n = state.dcm_sn.transpose() * thrust_s;
        let thrust_b = mrp_to_dcm(servicer_attitude.sigma_bn) * thrust_n;
        (thrust_n, thrust_b)
    }
}

/// Spherical coordinates of the debris relative to the servicer, their rate
/// and `[SN]`.
fn spherical_state(
    servicer: &NavigationTranslationMsg,
    debris: &NavigationTranslationMsg,
) -> SphericalState {
    let r_tn_n = servicer.position_bn_n_m;
    let v_tn_n = servicer.velocity_bn_n_mps;
    let dcm_hn = hill_frame(r_tn_n, v_tn_n);

    let rho_n = debris.position_bn_n_m - r_tn_n;
    let rho_h = dcm_hn * rho_n;
    let rho_dot_n = debris.velocity_bn_n_mps - v_tn_n;
    let r_tn = r_tn_n.norm();
    let omega_hn_n = r_tn_n.cross(&v_tn_n) * (1.0 / r_tn / r_tn);
    let rho_prime_h = dcm_hn * (rho_dot_n - omega_hn_n.cross(&rho_n));

    let length = rho_h.norm();
    let theta = rho_h[0].atan2(-rho_h[1]);
    let phi = safe_asin(-rho_h[2] / length);
    let (sin_theta, cos_theta) = (theta.sin(), theta.cos());
    let (sin_phi, cos_phi) = (phi.sin(), phi.cos());

    #[rustfmt::skip]
    let dcm_sh = Matrix3::new(
        cos_phi * sin_theta, -cos_theta * cos_phi, -sin_phi,
        cos_theta, sin_theta, 0.0,
        sin_theta * sin_phi, -cos_theta * sin_phi, cos_phi,
    );
    // Maps Hill-frame rates to (L̇, θ̇, φ̇); written as upstream writes it.
    #[rustfmt::skip]
    let hill_to_spherical_rates = Matrix3::new(
        cos_phi * sin_theta, -cos_theta * cos_phi, -sin_phi,
        cos_theta / cos_phi / length, 1.0 / cos_phi * sin_theta / length, 0.0,
        -sin_theta * sin_phi / length, cos_theta * sin_phi / length, -cos_phi / length,
    );

    SphericalState {
        x: Vector3::new(length, theta, phi),
        x_dot: hill_to_spherical_rates * rho_prime_h,
        dcm_sn: dcm_sh * dcm_hn,
    }
}

/// `[F](X, Ẋ)`: the unforced Clohessy-Wiltshire accelerations in spherical
/// coordinates for a servicer orbit of mean motion `n` \[rad/s\]. The
/// expressions keep upstream's operation order.
fn natural_dynamics(n: f64, x: &Vector3<f64>, x_dot: &Vector3<f64>) -> Vector3<f64> {
    let (length, theta, phi) = (x[0], x[1], x[2]);
    let (length_dot, theta_dot, phi_dot) = (x_dot[0], x_dot[1], x_dot[2]);
    let cos_phi = phi.cos();
    let cos_2theta = (2.0 * theta).cos();
    Vector3::new(
        0.25 * length
            * (n * n * (-6.0 * cos_2theta * cos_phi * cos_phi + 5.0 * (2.0 * phi).cos() + 1.0)
                + 4.0 * theta_dot * cos_phi * cos_phi * (2.0 * n + theta_dot)
                + 4.0 * phi_dot * phi_dot),
        (3.0 * n * n * theta.sin() * theta.cos() + 2.0 * phi_dot * phi.tan() * (n + theta_dot))
            - 2.0 * length_dot / length * (n + theta_dot),
        0.25 * (2.0 * phi).sin()
            * (n * n * (3.0 * cos_2theta - 5.0) - 2.0 * theta_dot * (2.0 * n + theta_dot))
            - 2.0 * length_dot / length * phi_dot,
    )
}

/// Upstream's positive-definiteness check: `vMax(-eig) > 0`, so a zero
/// eigenvalue passes. Upstream `vMax` keeps the first element unless a later
/// one compares greater, which fixes how NaN eigenvalues are treated.
fn has_negative_eigenvalue(matrix: &Matrix3<f64>) -> bool {
    let negated = -m33_eigen_values(matrix);
    let v_max = negated.iter().skip(1).fold(
        negated[0],
        |max, &value| if value > max { value } else { max },
    );
    v_max > 0.0
}

impl Module for EtSphericalControl {
    fn init(&mut self) {
        self.force_inertial_out_msg
            .write(InertialForceCommandMsg::default());
        self.force_body_out_msg
            .write(BodyForceCommandMsg::default());
        self.check_configuration();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_configuration();
    }

    fn update(&mut self, _context: &SimulationContext) {
        let (force_request_inertial_n, force_request_body_n) = self.control_force(
            &self.servicer_trans_in_msg.read(),
            &self.debris_trans_in_msg.read(),
            &self.servicer_att_in_msg.read(),
            self.servicer_vehicle_config_in_msg.read().mass_kg,
            self.debris_vehicle_config_in_msg.read().mass_kg,
            self.e_force_in_msg.read().force_request_inertial_n,
        );
        self.force_inertial_out_msg.write(InertialForceCommandMsg {
            force_request_inertial_n,
        });
        self.force_body_out_msg.write(BodyForceCommandMsg {
            force_request_body_n,
        });
    }
}

#[cfg(test)]
mod tests {
    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3, Vector4};

    use super::{EtSphericalControl, EtSphericalControlConfig, has_negative_eigenvalue};
    use crate::fsw_algorithms::formation_flying::mean_oe_feedback::test_orbits::python_elem2rv;
    use crate::messages::{
        BodyForceCommandMsg, InertialForceCommandMsg, NavigationAttitudeMsg,
        NavigationTranslationMsg, Output, VehicleConfigMsg,
    };
    use crate::simulation::Simulation;
    use crate::test_utils::MessageRecorder;
    use crate::utilities::orbital_motion::{ClassicElements, hill_frame};
    use crate::utilities::rigid_body_kinematics::{ep_to_mrp, mrp_to_dcm};
    use crate::{Module, SimulationContext};

    /// Upstream test task rate, `macros.sec2nano(0.5)`.
    const TASK_RATE_NANOS: u64 = 500_000_000;
    /// The test's `mu` \[m^3/s^2\].
    const MU_M3PS2: f64 = 3.986004418e14;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `isArrayEqual`: absolute tolerance per component, NaN fails.
    fn assert_array_equal(actual: Vector3<f64>, truth: [f64; 3], accuracy: f64, what: &str) {
        for (index, (value, expected)) in actual.iter().zip(truth).enumerate() {
            assert!(
                (value - expected).abs() <= accuracy,
                "{what}[{index}] = {value:.17e}, truth {expected:.17e}"
            );
        }
    }

    /// The inputs of one test case.
    struct Scenario {
        servicer: NavigationTranslationMsg,
        debris: NavigationTranslationMsg,
        sigma_tn: Vector3<f64>,
        servicer_mass_kg: f64,
        debris_mass_kg: f64,
        electrostatic_force_n: Vector3<f64>,
    }

    /// Producers of a scenario's inputs.
    struct Producers {
        servicer_trans: Output<NavigationTranslationMsg>,
        debris_trans: Output<NavigationTranslationMsg>,
        servicer_att: Output<NavigationAttitudeMsg>,
        servicer_config: Output<VehicleConfigMsg>,
        debris_config: Output<VehicleConfigMsg>,
        e_force: Output<InertialForceCommandMsg>,
    }

    impl Producers {
        fn new(scenario: &Scenario) -> Self {
            let mass = |mass_kg| VehicleConfigMsg {
                mass_kg,
                ..Default::default()
            };
            Self {
                servicer_trans: Output::new(scenario.servicer.clone()),
                debris_trans: Output::new(scenario.debris.clone()),
                servicer_att: Output::new(NavigationAttitudeMsg {
                    sigma_bn: scenario.sigma_tn,
                    ..Default::default()
                }),
                servicer_config: Output::new(mass(scenario.servicer_mass_kg)),
                debris_config: Output::new(mass(scenario.debris_mass_kg)),
                e_force: Output::new(InertialForceCommandMsg {
                    force_request_inertial_n: scenario.electrostatic_force_n,
                }),
            }
        }

        fn connect(&self, module: &mut EtSphericalControl) {
            self.servicer_trans
                .connect_to(&mut module.servicer_trans_in_msg);
            self.debris_trans
                .connect_to(&mut module.debris_trans_in_msg);
            self.servicer_att
                .connect_to(&mut module.servicer_att_in_msg);
            self.servicer_config
                .connect_to(&mut module.servicer_vehicle_config_in_msg);
            self.debris_config
                .connect_to(&mut module.debris_vehicle_config_in_msg);
            self.e_force.connect_to(&mut module.e_force_in_msg);
        }
    }

    /// The inputs of `test_etSphericalControl.py`: a servicer on an inclined
    /// GEO orbit (built with upstream's Python `elem2rv`), the debris 20 m
    /// behind it with the same velocity, and the electrostatic force between
    /// two single spheres (R_T = 2 m at 25 kV, R_D = 3 m at -25 kV).
    fn upstream_scenario() -> Scenario {
        let deg = std::f64::consts::PI / 180.0;
        let (r_tn_n, v_tn_n) = python_elem2rv(
            MU_M3PS2,
            &ClassicElements {
                semi_major_axis: 42164.0 * 1e3,
                eccentricity: 0.0,
                inclination_rad: 10.0 * deg,
                raan_rad: 20.0 * deg,
                arg_periapsis_rad: 30.0 * deg,
                true_anomaly_rad: 40.0 * deg,
                ..Default::default()
            },
        );
        let l0 = 20.0;
        let r_dt_n: Vector3<f64> = Vector3::new(2.0, -l0, -3.0);
        let beta_th = Vector4::new(
            0.972960339471760,
            0.107600839071972,
            -0.0289291519077161,
            0.202319898714648,
        );

        let (r_t, r_d, v_t, v_d): (f64, f64, f64, f64) = (2.0, 3.0, 25000.0, -25000.0);
        let kc = 8.9875517923e9;
        let l = r_dt_n.norm();
        let q_t = (l * (l * r_t * v_t - r_t * r_d * v_d)) / (kc * (l.powf(2.0) - r_t * r_d));
        let q_d = (l * (l * r_d * v_d - r_t * r_d * v_t)) / (kc * (l.powf(2.0) - r_t * r_d));
        let fc = kc * q_t * q_d / l.powf(2.0);

        Scenario {
            servicer: NavigationTranslationMsg {
                position_bn_n_m: r_tn_n,
                velocity_bn_n_mps: v_tn_n,
                ..Default::default()
            },
            debris: NavigationTranslationMsg {
                position_bn_n_m: r_tn_n + r_dt_n,
                velocity_bn_n_mps: v_tn_n,
                ..Default::default()
            },
            sigma_tn: ep_to_mrp(beta_th),
            servicer_mass_kg: 500.0,
            debris_mass_kg: 2000.0,
            electrostatic_force_n: (-r_dt_n / r_dt_n.norm()) * fc,
        }
    }

    /// Upstream `test_etSphericalControl.py` (its single `accuracy = 1e-8`
    /// parametrization): one update at t = 0, both outputs against the
    /// Matlab truth upstream hard-codes. The gains and references are set on
    /// `config` after construction, as upstream sets them after creating the
    /// module.
    #[test]
    fn matches_upstream_et_spherical_control_unit_test() {
        let accuracy = 1e-8;
        let scenario = upstream_scenario();
        let producers = Producers::new(&scenario);
        let mut module = EtSphericalControl::new(EtSphericalControlConfig::default());
        let k_i = 4e-7;
        let p_i = 1.85 * f64::powf(k_i, 0.5);
        module.config.k = Matrix3::from_row_slice(&[k_i, 0.0, 0.0, 0.0, k_i, 0.0, 0.0, 0.0, k_i]);
        module.config.p = Matrix3::from_row_slice(&[p_i, 0.0, 0.0, 0.0, p_i, 0.0, 0.0, 0.0, p_i]);
        module.config.l_r_m = 30.0;
        module.config.theta_r_rad = 0.0;
        module.config.phi_r_rad = 0.0;
        module.config.mu_m3ps2 = MU_M3PS2;
        producers.connect(&mut module);

        let mut inertial = MessageRecorder::<InertialForceCommandMsg>::default();
        let mut body = MessageRecorder::<BodyForceCommandMsg>::default();
        {
            let mut simulation =
                Simulation::new(Epoch::from_gregorian_utc_at_midnight(2025, 1, 1), false);
            simulation.connect(&module.force_inertial_out_msg, &mut inertial.input_msg);
            simulation.connect(&module.force_body_out_msg, &mut body.input_msg);
            simulation.add_module("etSphericalControl", &mut module, TASK_RATE_NANOS, 0);
            simulation.add_module("inertialRecorder", &mut inertial, TASK_RATE_NANOS, 0);
            simulation.add_module("bodyRecorder", &mut body, TASK_RATE_NANOS, 0);
            // Upstream `SingleStepProcesses`: the t = 0 tick only.
            simulation.run_for(0);
        }

        assert_eq!(inertial.samples.len(), 1);
        assert_eq!(body.samples.len(), 1);
        assert_array_equal(
            inertial.samples[0].force_request_inertial_n,
            [
                -0.00714223893615245,
                0.00267752848271998,
                0.000883681113161883,
            ],
            accuracy,
            "forceRequestInertial",
        );
        assert_array_equal(
            body.samples[0].force_request_body_n,
            [
                -0.00541988234898216,
                0.00542736415350300,
                0.000360862543207430,
            ],
            accuracy,
            "forceRequestBody",
        );
    }

    /// Not upstream: the debris trails the servicer on a circular orbit,
    /// co-rotating with the Hill frame (`X = (L, 0, 0)`, `Ẋ = 0`). There the
    /// natural dynamics vanish, so `u_S = (-k (L - L_r), 0, 0)` and the thrust
    /// is `m_T (k (L - L_r) - c (1/m_T + 1/m_D))` along the line of sight for
    /// an electrostatic force `c` along it.
    #[test]
    fn leader_follower_thrust_matches_the_hand_derived_law() {
        let (r_tn_n, v_tn_n) = python_elem2rv(
            MU_M3PS2,
            &ClassicElements {
                semi_major_axis: 42164.0 * 1e3,
                eccentricity: 0.0,
                inclination_rad: 0.3,
                raan_rad: 1.1,
                arg_periapsis_rad: 0.0,
                true_anomaly_rad: 2.0,
                ..Default::default()
            },
        );
        let separation_m = 25.0;
        let dcm_hn = hill_frame(r_tn_n, v_tn_n);
        let rho_n = dcm_hn.transpose() * Vector3::new(0.0, -separation_m, 0.0);
        let omega_hn_n = r_tn_n.cross(&v_tn_n) / r_tn_n.norm_squared();
        let rho_hat_n = rho_n / separation_m;
        let electrostatic_n = 3e-3;
        let sigma_tn = Vector3::new(0.1, -0.3, 0.2);
        let scenario = Scenario {
            servicer: NavigationTranslationMsg {
                position_bn_n_m: r_tn_n,
                velocity_bn_n_mps: v_tn_n,
                ..Default::default()
            },
            debris: NavigationTranslationMsg {
                position_bn_n_m: r_tn_n + rho_n,
                velocity_bn_n_mps: v_tn_n + omega_hn_n.cross(&rho_n),
                ..Default::default()
            },
            sigma_tn,
            servicer_mass_kg: 400.0,
            debris_mass_kg: 1500.0,
            electrostatic_force_n: rho_hat_n * electrostatic_n,
        };
        let producers = Producers::new(&scenario);
        let k_gain = 2e-6;
        let mut module = EtSphericalControl::new(EtSphericalControlConfig {
            mu_m3ps2: MU_M3PS2,
            l_r_m: 30.0,
            k: Matrix3::identity() * k_gain,
            p: Matrix3::identity() * 3e-3,
            ..Default::default()
        });
        producers.connect(&mut module);
        module.init();
        module.update(&context(0));

        let thrust_along_rho = 400.0 * k_gain * (separation_m - 30.0)
            - 400.0 * electrostatic_n * (1.0 / 400.0 + 1.0 / 1500.0);
        let expected_n = rho_hat_n * thrust_along_rho;
        let inertial = module
            .force_inertial_out_msg
            .read()
            .force_request_inertial_n;
        let body = module.force_body_out_msg.read().force_request_body_n;
        // The module sees the separation through `(r_T + rho) - r_T`, which
        // rounds rho at the 1e-8 m level of a GEO position: about 1e-11 N here.
        assert!(
            (inertial - expected_n).amax() < 1e-10,
            "{inertial:?} vs {expected_n:?}"
        );
        assert!(
            (body - mrp_to_dcm(sigma_tn) * expected_n).amax() < 1e-10,
            "{body:?}"
        );
    }

    /// Not upstream: an out-of-plane and in-plane offset with relative
    /// velocity; the published body thrust is the inertial thrust rotated by
    /// `[TN]`, and both scale linearly with the gains when the natural
    /// dynamics and the electrostatic force are cancelled out.
    #[test]
    fn body_thrust_is_the_inertial_thrust_in_servicer_axes() {
        let mut scenario = upstream_scenario();
        scenario.debris.position_bn_n_m += Vector3::new(4.0, 1.0, -6.0);
        scenario.debris.velocity_bn_n_mps += Vector3::new(1e-3, -2e-3, 5e-4);
        let producers = Producers::new(&scenario);
        let mut module = EtSphericalControl::new(EtSphericalControlConfig {
            mu_m3ps2: MU_M3PS2,
            l_r_m: 30.0,
            theta_r_rad: 0.2,
            phi_r_rad: -0.1,
            k: Matrix3::identity() * 4e-7,
            p: Matrix3::identity() * 1.2e-3,
            ..Default::default()
        });
        producers.connect(&mut module);
        module.init();
        module.update(&context(0));
        let inertial = module
            .force_inertial_out_msg
            .read()
            .force_request_inertial_n;
        let body = module.force_body_out_msg.read().force_request_body_n;
        assert_eq!(body, mrp_to_dcm(scenario.sigma_tn) * inertial);

        // Thrust minus its zero-gain part is linear in the gains.
        let mut feedback_part = |gain_scale: f64| {
            module.config.k = Matrix3::identity() * 4e-7 * gain_scale;
            module.config.p = Matrix3::identity() * 1.2e-3 * gain_scale;
            module.update(&context(TASK_RATE_NANOS));
            module
                .force_inertial_out_msg
                .read()
                .force_request_inertial_n
        };
        let open_loop = feedback_part(0.0);
        let single = feedback_part(1.0) - open_loop;
        let double = feedback_part(2.0) - open_loop;
        assert!(
            (double - single * 2.0).amax() < 1e-15,
            "{single:?} {double:?}"
        );
    }

    #[test]
    fn positive_definiteness_check_follows_upstream() {
        assert!(!has_negative_eigenvalue(&Matrix3::identity()));
        assert!(!has_negative_eigenvalue(&Matrix3::from_diagonal(
            &Vector3::new(3.0, 2.0, 1.0)
        )));
        assert!(has_negative_eigenvalue(&Matrix3::from_diagonal(
            &Vector3::new(1.0, -1e-3, 2.0)
        )));
        // A zero eigenvalue passes, as upstream's strict comparison lets it.
        assert!(!has_negative_eigenvalue(&Matrix3::zeros()));
        // Symmetric with eigenvalues 3 and -1.
        assert!(has_negative_eigenvalue(&Matrix3::new(
            1.0, 2.0, 0.0, 2.0, 1.0, 0.0, 0.0, 0.0, 1.0
        )));
    }

    #[test]
    fn init_publishes_zero_forces() {
        let mut module = EtSphericalControl::new(EtSphericalControlConfig::default());
        module
            .force_inertial_out_msg
            .write(InertialForceCommandMsg {
                force_request_inertial_n: Vector3::new(1.0, 2.0, 3.0),
            });
        module.force_body_out_msg.write(BodyForceCommandMsg {
            force_request_body_n: Vector3::new(1.0, 2.0, 3.0),
        });
        module.init();
        assert_eq!(
            module.force_inertial_out_msg.read(),
            InertialForceCommandMsg::default()
        );
        assert_eq!(
            module.force_body_out_msg.read(),
            BodyForceCommandMsg::default()
        );
    }
}
