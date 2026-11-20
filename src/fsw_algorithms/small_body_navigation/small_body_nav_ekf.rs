//! Small-body navigation extended Kalman filter: port of upstream Basilisk
//! `smallBodyNavEKF` (`fswAlgorithms/smallBodyNavigation/smallBodyNavEKF`).
//!
//! The twelve states are the spacecraft position `r_BO_O` and velocity
//! relative to the small body in the small body's Hill frame `O`, the small
//! body's MRP attitude `σ_AN` and its angular velocity `ω_AN_A`. The
//! asteroid's heliocentric orbit is taken as known from its ephemeris; it
//! sets the Hill frame `[ON]`, the true-anomaly rate `ḟ` and acceleration
//! `f̈`. The relative motion includes the rotating-frame terms, the small
//! body's point-mass gravity, the solar tidal acceleration, solar radiation
//! pressure and the thruster and commanded forces, rotated from the
//! spacecraft body frame with the navigation attitude. Each update
//! propagates the state and the state transition matrix with RK4, switches
//! the attitude MRPs to the shadow set when `|σ_AN| > 1`, and — when the
//! measurements are new — performs a Joseph-form measurement update with the
//! full state measured directly (`H = I`): the relative position and velocity
//! from the navigation and ephemeris inputs rotated into `O`, and the
//! attitude and rate from the ephemeris.
//!
//! # Measurement write time
//!
//! Upstream processes the measurements when the translational navigation,
//! attitude navigation and small-body ephemeris messages were all written at
//! the current time (`timeWritten` from the message headers). The Rust
//! inputs carry no header, so the measurements count as new at every update
//! when those three inputs are connected — what upstream sees when their
//! producers run before the filter on every tick. The update logic itself
//! takes that decision as an explicit argument, and the tests replay
//! upstream's header timeline. A caller that knows when the producers ran
//! passes that decision through
//! [`SmallBodyNavEkf::update_with_measurement_status`].
//!
//! # Upstream behaviour kept
//!
//! - The state transition matrix is propagated from its previous value and
//!   never restarted, so it maps from the first update, not from the last.
//! - Without new measurements the a priori state is kept but the covariance
//!   is not: the next propagation starts from the last updated covariance,
//!   and the output reports the a priori one.
//! - The SRP acceleration divides the 1 AU pressure `P_0` by the squared
//!   sun distance in metres, not in AU, which makes it negligible.
//! - The attitude-rate block of the dynamics matrix uses `3 σ σᵀ` where the
//!   MRP kinematics have `2 σ σᵀ`.
//! - Reset neither restarts the filter clock nor the transition matrix.

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::messages::{
    BodyForceCommandMsg, EphemerisMsg, Input, NavigationAttitudeMsg, NavigationTranslationMsg,
    Output, SMALL_BODY_NAV_N_STATES, SmallBodyNavMsg, ThrusterOutputMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::orbital_motion::{hill_frame, state_to_elements};
use crate::utilities::rigid_body_kinematics::{d_omega_to_mrp, mrp_to_dcm, tilde};
use crate::{Module, SimulationContext};

/// The filter state (upstream `StateVector`).
pub type SmallBodyNavState = SVector<f64, SMALL_BODY_NAV_N_STATES>;
/// A state-sized square matrix (upstream `StateMatrix`).
pub type SmallBodyNavMatrix = SMatrix<f64, SMALL_BODY_NAV_N_STATES, SMALL_BODY_NAV_N_STATES>;

/// Upstream `mu_sun` [m³/s²]: the sun's gravitational parameter the filter
/// uses for the asteroid orbit and the solar tide.
const MU_SUN_M3PS2: f64 = 1.327124e20;

/// User-settable parameters of [`SmallBodyNavEkf`].
#[derive(Clone, Debug)]
pub struct SmallBodyNavEkfConfig {
    /// Module instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `C_SRP` [-]: SRP scaling coefficient. Default 1.
    pub c_srp: f64,
    /// Upstream `P_0` [N/m²]: solar radiation pressure at 1 AU. Default
    /// 4.56e-6.
    pub p_0_npm2: f64,
    /// Upstream `rho` [-]: surface reflectivity. Default 0.4.
    pub rho: f64,
    /// Upstream `A_sc` \[m²\]: spacecraft area. Upstream leaves it
    /// uninitialized; zero here.
    pub area_sc_m2: f64,
    /// Upstream `M_sc` \[kg\]: spacecraft mass. Upstream leaves it
    /// uninitialized; zero here, which must be replaced (the accelerations
    /// divide by it).
    pub mass_sc_kg: f64,
    /// Upstream `mu_ast` [m³/s²]: the small body's gravitational parameter.
    /// Upstream leaves it uninitialized; zero here.
    pub mu_ast_m3ps2: f64,
    /// Upstream `Q`: process noise covariance.
    pub process_noise: SmallBodyNavMatrix,
    /// Upstream `R`: measurement noise covariance.
    pub measurement_noise: SmallBodyNavMatrix,
}

impl Default for SmallBodyNavEkfConfig {
    fn default() -> Self {
        Self {
            name: "smallBodyNavEKF".to_string(),
            c_srp: 1.0,
            p_0_npm2: 4.56e-6,
            rho: 0.4,
            area_sc_m2: 0.0,
            mass_sc_kg: 0.0,
            mu_ast_m3ps2: 0.0,
            process_noise: SmallBodyNavMatrix::zeros(),
            measurement_noise: SmallBodyNavMatrix::zeros(),
        }
    }
}

/// Small-body relative navigation EKF (upstream `smallBodyNavEKF`).
///
/// Seed `x_hat_k` and `p_k` before `init`, as upstream's Python sets
/// `x_hat_k` and `P_k`; reset leaves them untouched.
#[derive(Clone, Debug)]
pub struct SmallBodyNavEkf {
    /// User-settable parameters.
    pub config: SmallBodyNavEkfConfig,
    /// Upstream `navTransInMsg`: spacecraft inertial position and velocity.
    pub nav_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `navAttInMsg`: spacecraft attitude, which rotates the
    /// thruster and commanded forces into the Hill frame.
    pub nav_att_in_msg: Input<NavigationAttitudeMsg>,
    /// Upstream `asteroidEphemerisInMsg`: the small body's heliocentric
    /// position, velocity, attitude and rate.
    pub asteroid_ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `sunEphemerisInMsg`: the sun's position.
    pub sun_ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `cmdForceBodyInMsg` (optional): commanded force, body
    /// components.
    pub cmd_force_body_in_msg: Input<BodyForceCommandMsg>,
    /// Upstream `thrusterInMsgs`, one per thruster, added with
    /// [`Self::add_thruster_input`]: their body-frame thrust forces add up.
    pub thruster_in_msgs: Vec<Input<ThrusterOutputMsg>>,
    /// Upstream `navTransOutMsg`: the estimated spacecraft inertial position
    /// and velocity.
    pub nav_trans_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `smallBodyNavOutMsg`: state and covariance.
    pub small_body_nav_out_msg: Output<SmallBodyNavMsg>,
    /// Upstream `asteroidEphemerisOutMsg`: the input ephemeris with the
    /// estimated attitude and rate.
    pub asteroid_ephemeris_out_msg: Output<EphemerisMsg>,
    /// Upstream `x_hat_k`: current state estimate.
    pub x_hat_k: SmallBodyNavState,
    /// Upstream `P_k`: current state covariance.
    pub p_k: SmallBodyNavMatrix,
    /// Upstream `prevTime` \[ns\].
    prev_time_nanos: u64,
    /// Upstream `Phi_k`: state transition matrix.
    phi_k: SmallBodyNavMatrix,
    /// Upstream `cmdForce_B` \[N\]: the last commanded force read.
    cmd_force_b_n: Vector3<f64>,
}

/// The inputs of one filter update (upstream's message buffers).
#[derive(Clone, Debug)]
struct Measurements {
    nav_trans: NavigationTranslationMsg,
    nav_att: NavigationAttitudeMsg,
    asteroid: EphemerisMsg,
    sun: EphemerisMsg,
    /// Sum of the thruster forces \[N\], body components (upstream
    /// `thrust_B`).
    thrust_b_n: Vector3<f64>,
}

/// The asteroid-orbit quantities one propagation uses.
#[derive(Clone, Debug)]
struct AsteroidFrame {
    /// Upstream `F_dot` \[rad/s\]: true-anomaly rate.
    f_dot: f64,
    /// Upstream `F_ddot` \[rad/s²\]: true-anomaly acceleration.
    f_ddot: f64,
    /// Upstream `dcm_ON`: inertial to the small body's Hill frame.
    dcm_on: Matrix3<f64>,
    /// Upstream `r_SO_O` \[m\]: small body to sun, Hill components.
    r_so_o: Vector3<f64>,
    /// Spacecraft body to Hill frame.
    dcm_ob: Matrix3<f64>,
}

impl SmallBodyNavEkf {
    /// A filter with unconnected inputs, no thruster input and a zero
    /// estimate; seed `x_hat_k` and `p_k` before `init`.
    pub fn new(config: SmallBodyNavEkfConfig) -> Self {
        Self {
            config,
            nav_trans_in_msg: Input::default(),
            nav_att_in_msg: Input::default(),
            asteroid_ephemeris_in_msg: Input::default(),
            sun_ephemeris_in_msg: Input::default(),
            cmd_force_body_in_msg: Input::default(),
            thruster_in_msgs: Vec::new(),
            nav_trans_out_msg: Output::default(),
            small_body_nav_out_msg: Output::default(),
            asteroid_ephemeris_out_msg: Output::default(),
            x_hat_k: SmallBodyNavState::zeros(),
            p_k: SmallBodyNavMatrix::zeros(),
            prev_time_nanos: 0,
            phi_k: SmallBodyNavMatrix::identity(),
            cmd_force_b_n: Vector3::zeros(),
        }
    }

    /// Upstream `addThrusterToFilter`: adds one thruster input and returns
    /// it, to be connected.
    pub fn add_thruster_input(&mut self) -> &mut Input<ThrusterOutputMsg> {
        self.thruster_in_msgs.push(Input::default());
        self.thruster_in_msgs
            .last_mut()
            .expect("the thruster input was just added")
    }

    /// Upstream `Reset`: checks the required inputs. The state dimensions
    /// upstream validates here are fixed by the types.
    pub fn reset(&mut self) {
        let name = &self.config.name;
        for (connected, input) in [
            (self.nav_trans_in_msg.is_connected(), "nav_trans_in_msg"),
            (self.nav_att_in_msg.is_connected(), "nav_att_in_msg"),
            (
                self.asteroid_ephemeris_in_msg.is_connected(),
                "asteroid_ephemeris_in_msg",
            ),
            (
                self.sun_ephemeris_in_msg.is_connected(),
                "sun_ephemeris_in_msg",
            ),
        ] {
            if !connected {
                log::error!("{name}.{input} is not connected");
            }
        }
    }

    /// Upstream `UpdateState` with the header check made by the caller:
    /// `new_measurements` says whether the translational navigation,
    /// attitude navigation and small-body ephemeris messages were written at
    /// `call_time_nanos`. For a wrapper module that knows when the
    /// measurement producers ran, such as a scenario that disables their
    /// task (upstream `disableTask`), where the connected-inputs stand-in of
    /// [`Module::update`] would take the stale messages as new.
    pub fn update_with_measurement_status(&mut self, call_time_nanos: u64, new_measurements: bool) {
        self.step(call_time_nanos, new_measurements);
    }

    /// Upstream `UpdateState`, with upstream's header check explicit:
    /// `new_measurements` is whether the navigation and ephemeris messages
    /// were written at `call_time_nanos`.
    fn step(&mut self, call_time_nanos: u64, new_measurements: bool) {
        let measurements = self.read_messages();
        let frame = self.asteroid_frame(&measurements);
        let dt_s = diff_nanos_to_seconds(call_time_nanos, self.prev_time_nanos);

        let (mut x_apriori, mut p_apriori) = self.predict(&measurements, &frame, dt_s);
        check_mrp_switching(&mut x_apriori, &mut p_apriori);
        if new_measurements {
            (self.x_hat_k, self.p_k) =
                self.measurement_update(&x_apriori, &p_apriori, &measurements, &frame.dcm_on);
        } else {
            self.x_hat_k = x_apriori;
        }

        let covariance = if new_measurements {
            self.p_k
        } else {
            p_apriori
        };
        self.write_messages(&measurements, &frame.dcm_on, covariance);
        self.prev_time_nanos = call_time_nanos;
    }

    /// Upstream `readMessages`. The commanded force is read only when its
    /// input is connected; otherwise the last one read stays in use, as
    /// upstream.
    fn read_messages(&mut self) -> Measurements {
        if self.thruster_in_msgs.is_empty() {
            log::warn!("{} has no thruster messages to read", self.config.name);
        }
        if self.cmd_force_body_in_msg.is_connected() {
            self.cmd_force_b_n = self.cmd_force_body_in_msg.read().force_request_body_n;
        }
        Measurements {
            nav_trans: self.nav_trans_in_msg.read(),
            nav_att: self.nav_att_in_msg.read(),
            asteroid: self.asteroid_ephemeris_in_msg.read(),
            sun: self.sun_ephemeris_in_msg.read(),
            thrust_b_n: self
                .thruster_in_msgs
                .iter()
                .fold(Vector3::zeros(), |sum, input| {
                    sum + input.read().thrust_force_b_n
                }),
        }
    }

    /// The first part of upstream `predict`: the asteroid's orbit elements
    /// (with the sun's `μ`), Hill frame and sun direction.
    fn asteroid_frame(&self, measurements: &Measurements) -> AsteroidFrame {
        let asteroid = &measurements.asteroid;
        let elements = state_to_elements(
            MU_SUN_M3PS2,
            asteroid.position_bdy_zero_n_m,
            asteroid.velocity_bdy_zero_n_mps,
        );
        let (a, e, f) = (
            elements.semi_major_axis,
            elements.eccentricity,
            elements.true_anomaly_rad,
        );
        let mean_motion_term = (MU_SUN_M3PS2 / (a * (1.0 - e.powi(2))).powi(3)).sqrt();
        let f_dot = mean_motion_term * (1.0 + e * f.cos()).powi(2);
        let f_ddot = -2.0 * e * mean_motion_term * f.sin() * (1.0 + e * f.cos()) * f_dot;

        let dcm_on = hill_frame(
            asteroid.position_bdy_zero_n_m,
            asteroid.velocity_bdy_zero_n_mps,
        );
        // Heliocentric frame centred on the sun, not the barycentre.
        let r_so_o =
            dcm_on * (measurements.sun.position_bdy_zero_n_m - asteroid.position_bdy_zero_n_m);
        let dcm_bn = mrp_to_dcm(measurements.nav_att.sigma_bn);
        AsteroidFrame {
            f_dot,
            f_ddot,
            dcm_on,
            r_so_o,
            dcm_ob: dcm_on * dcm_bn.transpose(),
        }
    }

    /// Upstream `aprioriState` and `aprioriCovar`: RK4 propagation of the
    /// state and the transition matrix over `dt_s`, then
    /// `P⁻ = Φ P Φᵀ + Q` (upstream's noise mapping `L` is the identity).
    fn predict(
        &mut self,
        measurements: &Measurements,
        frame: &AsteroidFrame,
        dt_s: f64,
    ) -> (SmallBodyNavState, SmallBodyNavMatrix) {
        let x = self.x_hat_k;
        let phi = self.phi_k;
        let eom = |x: &SmallBodyNavState, phi: &SmallBodyNavMatrix| {
            self.equations_of_motion(x, phi, measurements, frame)
        };
        let (x_dot, phi_dot) = eom(&x, &phi);
        let (k1, k1_phi) = (dt_s * x_dot, dt_s * phi_dot);
        let (x_dot, phi_dot) = eom(&(x + k1 / 2.0), &(phi + k1_phi / 2.0));
        let (k2, k2_phi) = (dt_s * x_dot, dt_s * phi_dot);
        let (x_dot, phi_dot) = eom(&(x + k2 / 2.0), &(phi + k2_phi / 2.0));
        let (k3, k3_phi) = (dt_s * x_dot, dt_s * phi_dot);
        let (x_dot, phi_dot) = eom(&(x + k3), &(phi + k3_phi));
        let (k4, k4_phi) = (dt_s * x_dot, dt_s * phi_dot);

        let x_apriori = x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0;
        self.phi_k = phi + (k1_phi + 2.0 * k2_phi + 2.0 * k3_phi + k4_phi) / 6.0;
        let p_apriori = self.phi_k * self.p_k * self.phi_k.transpose() + self.config.process_noise;
        (x_apriori, p_apriori)
    }

    /// Upstream `computeEquationsOfMotion`: the state derivative at `x` and
    /// the transition-matrix derivative `A(x) Φ`.
    fn equations_of_motion(
        &self,
        x: &SmallBodyNavState,
        phi: &SmallBodyNavMatrix,
        measurements: &Measurements,
        frame: &AsteroidFrame,
    ) -> (SmallBodyNavState, SmallBodyNavMatrix) {
        let config = &self.config;
        let x_1 = x.fixed_rows::<3>(0).into_owned();
        let x_2 = x.fixed_rows::<3>(3).into_owned();
        let x_3 = x.fixed_rows::<3>(6).into_owned();
        let x_4 = x.fixed_rows::<3>(9).into_owned();
        let o_hat_3_tilde = tilde(Vector3::z());
        let identity = Matrix3::identity();
        let r_so_o = frame.r_so_o;
        let r_so_hat = r_so_o / r_so_o.norm();

        let rotating_frame = -frame.f_ddot * o_hat_3_tilde * x_1
            - 2.0 * frame.f_dot * o_hat_3_tilde * x_2
            - frame.f_dot.powi(2) * o_hat_3_tilde * o_hat_3_tilde * x_1;
        let small_body_gravity = -config.mu_ast_m3ps2 * x_1 / x_1.norm().powi(3);
        let solar_tide = MU_SUN_M3PS2 * (3.0 * r_so_hat * r_so_hat.transpose() - identity) * x_1
            / r_so_o.norm().powi(3);
        let srp = config.c_srp
            * config.p_0_npm2
            * (1.0 + config.rho)
            * (config.area_sc_m2 / config.mass_sc_kg)
            * Vector3::x()
            / r_so_o.norm().powi(2);
        let forces = frame.dcm_ob * measurements.thrust_b_n / config.mass_sc_kg
            + frame.dcm_ob * self.cmd_force_b_n / config.mass_sc_kg;

        let mut x_dot = SmallBodyNavState::zeros();
        x_dot.fixed_rows_mut::<3>(0).copy_from(&x_2);
        x_dot
            .fixed_rows_mut::<3>(3)
            .copy_from(&(rotating_frame + small_body_gravity + solar_tide + srp + forces));
        x_dot
            .fixed_rows_mut::<3>(6)
            .copy_from(&d_omega_to_mrp(x_3, x_4));

        let dynamics = self.dynamics_matrix(x, frame);
        (x_dot, dynamics * phi)
    }

    /// Upstream `computeDynamicsMatrix`: the Jacobian `A` of the equations of
    /// motion at `x`.
    fn dynamics_matrix(&self, x: &SmallBodyNavState, frame: &AsteroidFrame) -> SmallBodyNavMatrix {
        let mu_ast = self.config.mu_ast_m3ps2;
        let x_1 = x.fixed_rows::<3>(0).into_owned();
        let x_3 = x.fixed_rows::<3>(6).into_owned();
        let x_4 = x.fixed_rows::<3>(9).into_owned();
        let o_hat_3_tilde = tilde(Vector3::z());
        let identity = Matrix3::identity();
        let r_so_o = frame.r_so_o;

        let mut a = SmallBodyNavMatrix::zeros();
        a.fixed_view_mut::<3, 3>(0, 3).copy_from(&identity);
        let position_partial = -frame.f_ddot * o_hat_3_tilde
            - frame.f_dot.powi(2) * o_hat_3_tilde * o_hat_3_tilde
            - mu_ast / x_1.norm().powi(3) * identity
            + 3.0 * mu_ast * x_1 * x_1.transpose() / x_1.norm().powi(5)
            + MU_SUN_M3PS2
                * (3.0 * (r_so_o * r_so_o.transpose()) / r_so_o.norm().powi(2) - identity)
                / r_so_o.norm().powi(3);
        a.fixed_view_mut::<3, 3>(3, 0).copy_from(&position_partial);
        a.fixed_view_mut::<3, 3>(3, 3)
            .copy_from(&(-2.0 * frame.f_dot * o_hat_3_tilde));
        let attitude_partial = 0.5
            * (x_3 * x_4.transpose() - x_4 * x_3.transpose() - tilde(x_4)
                + x_4.dot(&x_3) * identity);
        a.fixed_view_mut::<3, 3>(6, 6).copy_from(&attitude_partial);
        // Upstream's `3 σ σᵀ`, kept (see the module docs).
        let rate_partial = 0.25
            * ((1.0 - x_3.norm().powi(2)) * identity
                + 2.0 * tilde(x_3)
                + 3.0 * x_3 * x_3.transpose());
        a.fixed_view_mut::<3, 3>(6, 9).copy_from(&rate_partial);
        a
    }

    /// Upstream `measurementUpdate`: Kalman gain and Joseph-form update with
    /// the full state measured (`H = I`, `M = I`).
    fn measurement_update(
        &self,
        x_apriori: &SmallBodyNavState,
        p_apriori: &SmallBodyNavMatrix,
        measurements: &Measurements,
        dcm_on: &Matrix3<f64>,
    ) -> (SmallBodyNavState, SmallBodyNavMatrix) {
        let noise = &self.config.measurement_noise;
        let innovation_covariance = p_apriori + noise;
        let gain = p_apriori
            * innovation_covariance.try_inverse().unwrap_or_else(|| {
                log::error!("{}: singular innovation covariance", self.config.name);
                SmallBodyNavMatrix::from_element(f64::NAN)
            });

        let asteroid = &measurements.asteroid;
        let nav = &measurements.nav_trans;
        let mut y = SmallBodyNavState::zeros();
        y.fixed_rows_mut::<3>(0)
            .copy_from(&(dcm_on * (nav.position_bn_n_m - asteroid.position_bdy_zero_n_m)));
        y.fixed_rows_mut::<3>(3)
            .copy_from(&(dcm_on * (nav.velocity_bn_n_mps - asteroid.velocity_bdy_zero_n_mps)));
        y.fixed_rows_mut::<3>(6).copy_from(&closest_mrp_set(
            asteroid.sigma_bn,
            &x_apriori.fixed_rows::<3>(6).into_owned(),
        ));
        y.fixed_rows_mut::<3>(9)
            .copy_from(&asteroid.omega_bn_b_radps);

        let x = x_apriori + gain * (y - x_apriori);
        let i_minus_k = SmallBodyNavMatrix::identity() - gain;
        let p = i_minus_k * p_apriori * i_minus_k.transpose() + gain * noise * gain.transpose();
        (x, p)
    }

    /// Upstream `writeMessages`.
    fn write_messages(
        &self,
        measurements: &Measurements,
        dcm_on: &Matrix3<f64>,
        covariance: SmallBodyNavMatrix,
    ) {
        let asteroid = &measurements.asteroid;
        let x = &self.x_hat_k;
        self.nav_trans_out_msg.write(NavigationTranslationMsg {
            time_tag_s: measurements.nav_trans.time_tag_s,
            position_bn_n_m: asteroid.position_bdy_zero_n_m
                + dcm_on.transpose() * x.fixed_rows::<3>(0),
            velocity_bn_n_mps: asteroid.velocity_bdy_zero_n_mps
                + dcm_on.transpose() * x.fixed_rows::<3>(3),
            // Not estimated: passed through.
            accumulated_dv_b_mps: measurements.nav_trans.accumulated_dv_b_mps,
        });
        self.asteroid_ephemeris_out_msg.write(EphemerisMsg {
            position_bdy_zero_n_m: asteroid.position_bdy_zero_n_m,
            velocity_bdy_zero_n_mps: asteroid.velocity_bdy_zero_n_mps,
            sigma_bn: x.fixed_rows::<3>(6).into_owned(),
            omega_bn_b_radps: x.fixed_rows::<3>(9).into_owned(),
            time_tag_s: asteroid.time_tag_s,
        });
        self.small_body_nav_out_msg.write(SmallBodyNavMsg {
            state: *x,
            covar: covariance,
        });
    }
}

impl Module for SmallBodyNavEkf {
    fn init(&mut self) {
        self.nav_trans_out_msg
            .write(NavigationTranslationMsg::default());
        self.small_body_nav_out_msg
            .write(SmallBodyNavMsg::default());
        self.asteroid_ephemeris_out_msg
            .write(EphemerisMsg::default());
        self.prev_time_nanos = 0;
        self.phi_k = SmallBodyNavMatrix::identity();
        self.cmd_force_b_n = Vector3::zeros();
        SmallBodyNavEkf::reset(self);
    }

    fn reset(&mut self, _context: &SimulationContext) {
        SmallBodyNavEkf::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        // The header stand-in: connected inputs count as written now.
        let new_measurements = self.nav_trans_in_msg.is_connected()
            && self.nav_att_in_msg.is_connected()
            && self.asteroid_ephemeris_in_msg.is_connected();
        self.step(context.current_sim_nanos, new_measurements);
    }
}

/// Upstream `checkMRPSwitching`: switches the a priori attitude MRPs to the
/// shadow set when `|σ| > 1` and maps the covariance with the switch's
/// Jacobian `Λ`.
fn check_mrp_switching(x: &mut SmallBodyNavState, p: &mut SmallBodyNavMatrix) {
    let sigma = x.fixed_rows::<3>(6).into_owned();
    let mut lambda = SmallBodyNavMatrix::identity();
    if sigma.norm() > 1.0 {
        x.fixed_rows_mut::<3>(6)
            .copy_from(&(-sigma / sigma.norm().powi(2)));
        let jacobian = 2.0 * sigma * sigma.transpose() / sigma.norm().powi(4)
            - Matrix3::identity() / sigma.norm().powi(2);
        lambda.fixed_view_mut::<3, 3>(6, 6).copy_from(&jacobian);
    }
    *p = lambda * *p * lambda.transpose();
}

/// The measured MRPs, or their shadow set when `|σ| > 1/3` and the shadow
/// set is closer to the estimate (upstream `measurementUpdate`).
fn closest_mrp_set(measured: Vector3<f64>, estimate: &Vector3<f64>) -> Vector3<f64> {
    if measured.norm() > 1.0 / 3.0 {
        let shadow = -measured / measured.norm().powi(2);
        if (shadow - estimate).norm() < (measured - estimate).norm() {
            return shadow;
        }
    }
    measured
}

#[cfg(test)]
mod tests {
    //! Ports `smallBodyNavEKF/_UnitTest/test_smallBodyNavEKF.py`.
    //!
    //! `test_smallBodyNavEKF_rejects_invalid_dimensions` is not ported:
    //! `x_hat_k`, `p_k`, `Q` and `R` are fixed-size here, so a wrongly sized
    //! state or covariance does not compile. The C-wrapped output check is
    //! the same message.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{
        AsteroidFrame, Measurements, SmallBodyNavEkf, SmallBodyNavEkfConfig, SmallBodyNavMatrix,
        SmallBodyNavState, check_mrp_switching, closest_mrp_set,
    };
    use crate::messages::{
        BodyForceCommandMsg, EphemerisMsg, NavigationAttitudeMsg, NavigationTranslationMsg, Output,
        ThrusterOutputMsg,
    };
    use crate::utilities::astro_constants::{AU, MU_SUN};
    use crate::{Module, SimulationContext};

    /// Upstream's 0.5 s task rate.
    const TASK_RATE_NANOS: u64 = 500_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// The upstream test's input messages.
    struct UpstreamInputs {
        nav_trans: Output<NavigationTranslationMsg>,
        nav_att: Output<NavigationAttitudeMsg>,
        asteroid: Output<EphemerisMsg>,
        sun: Output<EphemerisMsg>,
        thruster: Output<ThrusterOutputMsg>,
        cmd_force: Output<BodyForceCommandMsg>,
    }

    /// Upstream `smallBodyNavEKFTestFunction`'s module and messages, wired.
    fn upstream_setup() -> (SmallBodyNavEkf, UpstreamInputs) {
        let mut module = SmallBodyNavEkf::new(SmallBodyNavEkfConfig {
            name: "smallBodyNavEKFTag".to_string(),
            area_sc_m2: 1.0,
            mass_sc_kg: 100.0,
            mu_ast_m3ps2: 5.2,
            process_noise: 0.1 * SmallBodyNavMatrix::identity(),
            measurement_noise: 0.1 * SmallBodyNavMatrix::identity(),
            ..Default::default()
        });
        let bennu_radius = 1.355887692 * AU * 1000.0;
        let bennu_velocity = (MU_SUN * 1000.0_f64.powi(3) / bennu_radius).sqrt();
        module.x_hat_k = SmallBodyNavState::from_column_slice(&[
            2010., 1510., 1010., 0., 2., 0., 0.14, 0., 0., 0., 0., 0.,
        ]);
        module.p_k = 0.1 * SmallBodyNavMatrix::identity();

        let inputs = UpstreamInputs {
            nav_trans: Output::new(NavigationTranslationMsg {
                position_bn_n_m: Vector3::new(bennu_radius + 1000., 1000., 1000.),
                velocity_bn_n_mps: Vector3::new(0., bennu_velocity + 1., 0.),
                ..Default::default()
            }),
            nav_att: Output::new(NavigationAttitudeMsg {
                sigma_bn: Vector3::new(0.1, 0.0, 0.0),
                ..Default::default()
            }),
            asteroid: Output::new(EphemerisMsg {
                position_bdy_zero_n_m: Vector3::new(bennu_radius, 0., 0.),
                velocity_bdy_zero_n_mps: Vector3::new(0., bennu_velocity, 0.),
                sigma_bn: Vector3::new(0.1, 0.0, 0.0),
                ..Default::default()
            }),
            sun: Output::new(EphemerisMsg::default()),
            thruster: Output::new(ThrusterOutputMsg::default()),
            cmd_force: Output::new(BodyForceCommandMsg::default()),
        };
        inputs.nav_trans.connect_to(&mut module.nav_trans_in_msg);
        inputs.nav_att.connect_to(&mut module.nav_att_in_msg);
        inputs
            .asteroid
            .connect_to(&mut module.asteroid_ephemeris_in_msg);
        inputs.sun.connect_to(&mut module.sun_ephemeris_in_msg);
        inputs.thruster.connect_to(module.add_thruster_input());
        inputs
            .cmd_force
            .connect_to(&mut module.cmd_force_body_in_msg);
        (module, inputs)
    }

    /// Upstream `test_smallBodyNavEKF`: the messages are written once, at
    /// t = 0, before a 10 s run at 0.5 s, so upstream's header check makes
    /// only the t = 0 update a measurement update; the estimate then
    /// propagates. Upstream's `compareArray` checks the first three states
    /// to 0.1; all twelve are checked here, against the same truth.
    #[test]
    fn matches_upstream_small_body_nav_ekf_unit_test() {
        let (mut module, _inputs) = upstream_setup();
        module.init();
        let mut state = SmallBodyNavState::zeros();
        for tick in 0..=20 {
            let now = tick * TASK_RATE_NANOS;
            // Upstream's check: the three `timeWritten` (all 0) sum to
            // 3 CurrentSimNanos.
            module.step(now, now == 0);
            state = module.small_body_nav_out_msg.read().state;
        }

        let true_x_hat = [
            1.33666664e+03,
            1.18333330e+03,
            1.00333330e+03,
            -4.77594532e-06,
            1.33332617,
            -6.10976335e-06,
            1.13333333e-01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ];
        for (index, (actual, expected)) in state.iter().zip(true_x_hat).enumerate() {
            assert!(
                (actual - expected).abs() <= 0.1,
                "x_hat[{index}] = {actual}, expected {expected}"
            );
            // Not upstream: the truth's nine printed digits all agree.
            assert!(
                (actual - expected).abs() <= 1e-8 * expected.abs(),
                "x_hat[{index}] = {actual}, expected {expected}"
            );
        }
    }

    /// Not upstream: the t = 0 measurement update is the closed form with
    /// `P⁻ = P + Q` (no propagation over a zero step), and the outputs map
    /// the estimate back to inertial.
    #[test]
    fn first_update_blends_estimate_and_measurement() {
        let (mut module, inputs) = upstream_setup();
        module.init();
        let x_0 = module.x_hat_k;
        module.update(&context(0));

        let asteroid = inputs.asteroid.read();
        let nav = inputs.nav_trans.read();
        // The asteroid's Hill frame is the inertial frame here.
        let mut y = SmallBodyNavState::zeros();
        y.fixed_rows_mut::<3>(0)
            .copy_from(&(nav.position_bn_n_m - asteroid.position_bdy_zero_n_m));
        y.fixed_rows_mut::<3>(3)
            .copy_from(&(nav.velocity_bn_n_mps - asteroid.velocity_bdy_zero_n_mps));
        y.fixed_rows_mut::<3>(6).copy_from(&asteroid.sigma_bn);
        // Gain 0.2 / (0.2 + 0.1) on every state.
        let expected = x_0 + (y - x_0) * (2.0 / 3.0);
        let output = module.small_body_nav_out_msg.read();
        assert!((output.state - expected).norm() < 1e-9);
        // Joseph form: (1 - k)² 0.2 + k² 0.1.
        let variance = (1.0_f64 / 3.0).powi(2) * 0.2 + (2.0_f64 / 3.0).powi(2) * 0.1;
        assert!((output.covar - variance * SmallBodyNavMatrix::identity()).norm() < 1e-12);

        let nav_out = module.nav_trans_out_msg.read();
        assert!(
            (nav_out.position_bn_n_m
                - (asteroid.position_bdy_zero_n_m + expected.fixed_rows::<3>(0)))
            .norm()
                < 1e-6
        );
        let ephemeris_out = module.asteroid_ephemeris_out_msg.read();
        assert_eq!(
            ephemeris_out.position_bdy_zero_n_m,
            asteroid.position_bdy_zero_n_m
        );
        assert!((ephemeris_out.sigma_bn - expected.fixed_rows::<3>(6)).norm() < 1e-12);
    }

    /// Not upstream: with the stand-in every update with connected inputs is
    /// a measurement update, so the estimate converges on the measurement.
    #[test]
    fn connected_inputs_update_every_step() {
        let (mut module, inputs) = upstream_setup();
        module.init();
        for tick in 0..=40 {
            module.update(&context(tick * TASK_RATE_NANOS));
        }
        let relative =
            inputs.nav_trans.read().position_bn_n_m - inputs.asteroid.read().position_bdy_zero_n_m;
        let state = module.small_body_nav_out_msg.read().state;
        assert!((state.fixed_rows::<3>(0) - relative).norm() < 5.0);
    }

    /// Not upstream: the a priori MRPs switch to the shadow set past
    /// `|σ| = 1`, and the covariance maps through the switch's Jacobian.
    #[test]
    fn mrp_switching_maps_state_and_covariance() {
        let sigma = Vector3::new(0.9, 0.5, 0.2);
        let mut x = SmallBodyNavState::zeros();
        x.fixed_rows_mut::<3>(6).copy_from(&sigma);
        let mut p = SmallBodyNavMatrix::identity();
        check_mrp_switching(&mut x, &mut p);
        let s2 = sigma.norm_squared();
        assert!((x.fixed_rows::<3>(6) + sigma / s2).norm() < 1e-15);
        let lambda = 2.0 * sigma * sigma.transpose() / s2.powi(2) - Matrix3::identity() / s2;
        assert!((p.fixed_view::<3, 3>(6, 6) - lambda * lambda.transpose()).norm() < 1e-14);
        assert_eq!(
            p.fixed_view::<6, 6>(0, 0),
            SmallBodyNavMatrix::identity().fixed_view::<6, 6>(0, 0)
        );

        let mut x = SmallBodyNavState::zeros();
        x[6] = 0.5;
        let mut p = SmallBodyNavMatrix::identity();
        check_mrp_switching(&mut x, &mut p);
        assert_eq!(x[6], 0.5);
        assert_eq!(p, SmallBodyNavMatrix::identity());
    }

    /// Not upstream: a measured attitude past `|σ| = 1/3` is replaced by its
    /// shadow set when that is closer to the estimate.
    #[test]
    fn measured_attitude_uses_the_closer_mrp_set() {
        let measured = Vector3::new(0.9, 0.0, 0.0);
        let shadow = -measured / measured.norm_squared();
        assert_eq!(
            closest_mrp_set(measured, &Vector3::new(-0.9, 0.0, 0.0)),
            shadow
        );
        assert_eq!(
            closest_mrp_set(measured, &Vector3::new(0.8, 0.0, 0.0)),
            measured
        );
        let small = Vector3::new(0.3, 0.0, 0.0);
        assert_eq!(closest_mrp_set(small, &Vector3::new(-3.0, 0.0, 0.0)), small);
    }

    /// Not upstream: the translational blocks of the dynamics matrix are the
    /// Jacobian of the equations of motion (by central differences). The
    /// attitude-rate block is upstream's and is not checked.
    #[test]
    fn dynamics_matrix_translation_blocks_match_finite_differences() {
        let (mut module, _inputs) = upstream_setup();
        module.config.mu_ast_m3ps2 = 5.0e6;
        let measurements = Measurements {
            nav_trans: NavigationTranslationMsg::default(),
            nav_att: NavigationAttitudeMsg::default(),
            asteroid: EphemerisMsg::default(),
            sun: EphemerisMsg::default(),
            thrust_b_n: Vector3::zeros(),
        };
        let frame = AsteroidFrame {
            f_dot: 2.0e-7,
            f_ddot: 1.0e-14,
            dcm_on: Matrix3::identity(),
            r_so_o: Vector3::new(-2.0e11, 1.0e9, 0.0),
            dcm_ob: Matrix3::identity(),
        };
        let x = SmallBodyNavState::from_column_slice(&[
            2010., 1510., 1010., 0.1, 2., -0.3, 0.14, 0.1, -0.2, 1e-3, 2e-3, -1e-3,
        ]);
        let phi = SmallBodyNavMatrix::identity();
        let a = module.dynamics_matrix(&x, &frame);
        for column in 0..6 {
            let step = 0.1;
            let mut plus = x;
            let mut minus = x;
            plus[column] += step;
            minus[column] -= step;
            let (f_plus, _) = module.equations_of_motion(&plus, &phi, &measurements, &frame);
            let (f_minus, _) = module.equations_of_motion(&minus, &phi, &measurements, &frame);
            let derivative = (f_plus - f_minus) / (2.0 * step);
            for row in 0..6 {
                let scale = a[(row, column)].abs().max(1e-12);
                assert!(
                    (derivative[row] - a[(row, column)]).abs() <= 1e-6 * scale + 1e-12,
                    "A[{row}, {column}] = {}, finite difference {}",
                    a[(row, column)],
                    derivative[row]
                );
            }
        }
    }
}
