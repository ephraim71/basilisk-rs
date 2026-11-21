//! Small-body navigation unscented Kalman filter: port of upstream Basilisk
//! `smallBodyNavUKF` (`fswAlgorithms/smallBodyNavigation/smallBodyNavUKF`).
//!
//! The nine states are the spacecraft position `r` and velocity `v`
//! relative to the small body and the non-Keplerian acceleration `a`
//! perturbing its motion, all in the small body's fixed frame `A`. Each
//! update draws `2·9 + 1` sigma points from the covariance (Cholesky factor
//! scaled by `√(9 + κ)`), propagates each with one Euler step of the
//! rotating-frame point-mass dynamics
//!
//! `ṙ = v`, `v̇ = -2 ω × v - ω × (ω × r) - μ r / |r|³ + a`, `ȧ = 0`,
//!
//! with the small body's rate `ω_AN_A` from its ephemeris, adds the process
//! noise, redraws sigma points about the a priori estimate and updates with
//! the measured position: the spacecraft minus the small-body position,
//! rotated into `A` with the ephemeris attitude. The measurement noise is
//! rotated into `A` the same way.
//!
//! The weights are upstream's: `w_m,0 = κ / (κ + 9)`,
//! `w_c,0 = w_m,0 + 1 - α² + β`, and `1 / (2 (9 + κ))` for every other point.
//!
//! Every update is a measurement update; upstream reads no message header.
//! If a covariance is not positive definite the update is skipped (with an
//! error logged) and the previous output kept; upstream's Eigen Cholesky
//! would carry on with an invalid factor.

use nalgebra::{Matrix3, SMatrix, SVector, Vector3};

use crate::messages::{
    EphemerisMsg, Input, NavigationTranslationMsg, Output, SMALL_BODY_NAV_UKF_N_STATES,
    SmallBodyNavUkfMsg,
};
use crate::time::diff_nanos_to_seconds;
use crate::utilities::rigid_body_kinematics::mrp_to_dcm;
use crate::{Module, SimulationContext};

/// Upstream `stateSize`.
const N: usize = SMALL_BODY_NAV_UKF_N_STATES;
/// Upstream `sigmaCount`: `2 N + 1` sigma points.
const SIGMA_COUNT: usize = 2 * N + 1;

/// The filter state (upstream `StateVector`).
pub type SmallBodyNavUkfState = SVector<f64, N>;
/// A state-sized square matrix (upstream `StateMatrix`).
pub type SmallBodyNavUkfMatrix = SMatrix<f64, N, N>;
/// One column per sigma point (upstream `StateSigmaMatrix`).
type SigmaPoints = SMatrix<f64, N, SIGMA_COUNT>;
/// Sigma-point weights (upstream `SigmaWeightVector`).
type SigmaWeights = SVector<f64, SIGMA_COUNT>;

/// User-settable parameters of [`SmallBodyNavUkf`].
#[derive(Clone, Debug)]
pub struct SmallBodyNavUkfConfig {
    /// Module instance name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `mu_ast` [m³/s²]: the small body's gravitational parameter.
    /// Upstream leaves it uninitialized; zero here.
    pub mu_ast_m3ps2: f64,
    /// Upstream `P_proc`: process noise covariance.
    pub process_noise: SmallBodyNavUkfMatrix,
    /// Upstream `R_meas`: position measurement noise covariance, in the
    /// measurement's (inertial) frame.
    pub measurement_noise: Matrix3<f64>,
    /// Upstream `alpha` [-]: UKF hyper-parameter (only in `w_c,0`). Default
    /// 0.
    pub alpha: f64,
    /// Upstream `beta` [-]: UKF hyper-parameter (only in `w_c,0`). Default 2.
    pub beta: f64,
    /// Upstream `kappa` [-]: sigma-point spread. Default 1e-3.
    pub kappa: f64,
}

impl Default for SmallBodyNavUkfConfig {
    fn default() -> Self {
        Self {
            name: "smallBodyNavUKF".to_string(),
            mu_ast_m3ps2: 0.0,
            process_noise: SmallBodyNavUkfMatrix::zeros(),
            measurement_noise: Matrix3::zeros(),
            alpha: 0.0,
            beta: 2.0,
            kappa: 1e-3,
        }
    }
}

/// Small-body relative navigation UKF (upstream `smallBodyNavUKF`).
///
/// Seed `x_hat_k` and `p_k` before `init`, as upstream's Python sets
/// `x_hat_k` and `P_k`; reset leaves them untouched.
#[derive(Clone, Debug)]
pub struct SmallBodyNavUkf {
    /// User-settable parameters; the weights are computed from them at reset.
    pub config: SmallBodyNavUkfConfig,
    /// Upstream `navTransInMsg`: spacecraft inertial position.
    pub nav_trans_in_msg: Input<NavigationTranslationMsg>,
    /// Upstream `asteroidEphemerisInMsg`: the small body's position,
    /// attitude and rate.
    pub asteroid_ephemeris_in_msg: Input<EphemerisMsg>,
    /// Upstream `smallBodyNavUKFOutMsg`: state and covariance.
    pub small_body_nav_ukf_out_msg: Output<SmallBodyNavUkfMsg>,
    /// Upstream `x_hat_k`: current state estimate.
    pub x_hat_k: SmallBodyNavUkfState,
    /// Upstream `P_k`: current state covariance.
    pub p_k: SmallBodyNavUkfMatrix,
    /// Upstream `prevTime` \[ns\].
    prev_time_nanos: u64,
    /// Upstream `wm_sigma`: mean weights.
    wm_sigma: SigmaWeights,
    /// Upstream `wc_sigma`: covariance weights.
    wc_sigma: SigmaWeights,
}

/// The unscented transform of the a priori state into the measurement
/// (upstream `measurementUT`'s outputs).
struct MeasurementTransform {
    /// Upstream `y_hat_k1_`: a priori measurement.
    y_hat: Vector3<f64>,
    /// Upstream `R_k1_`: measurement covariance, noise included.
    r: Matrix3<f64>,
    /// Upstream `H`: state-measurement cross-covariance.
    h: SMatrix<f64, N, 3>,
    /// Upstream `dcm_AN`: inertial to small-body frame.
    dcm_an: Matrix3<f64>,
}

impl SmallBodyNavUkf {
    /// A filter with unconnected inputs and a zero estimate; seed `x_hat_k`
    /// and `p_k` before `init`.
    pub fn new(config: SmallBodyNavUkfConfig) -> Self {
        Self {
            config,
            nav_trans_in_msg: Input::default(),
            asteroid_ephemeris_in_msg: Input::default(),
            small_body_nav_ukf_out_msg: Output::default(),
            x_hat_k: SmallBodyNavUkfState::zeros(),
            p_k: SmallBodyNavUkfMatrix::zeros(),
            prev_time_nanos: 0,
            wm_sigma: SigmaWeights::zeros(),
            wc_sigma: SigmaWeights::zeros(),
        }
    }

    /// Upstream `Reset`: checks the inputs and computes the sigma-point
    /// weights.
    pub fn reset(&mut self) {
        let name = &self.config.name;
        if !self.nav_trans_in_msg.is_connected() {
            log::error!("{name}.nav_trans_in_msg is not connected");
        }
        if !self.asteroid_ephemeris_in_msg.is_connected() {
            log::error!("{name}.asteroid_ephemeris_in_msg is not connected");
        }

        let SmallBodyNavUkfConfig {
            alpha, beta, kappa, ..
        } = self.config;
        let n = N as f64;
        self.wm_sigma = SigmaWeights::from_element(1.0 / (2.0 * (n + kappa)));
        self.wc_sigma = self.wm_sigma;
        self.wm_sigma[0] = kappa / (kappa + n);
        self.wc_sigma[0] = self.wm_sigma[0] + 1.0 - alpha.powi(2) + beta;
    }

    /// Upstream `UpdateState`.
    fn step(&mut self, call_time_nanos: u64) {
        let nav = self.nav_trans_in_msg.read();
        let asteroid = self.asteroid_ephemeris_in_msg.read();
        let dt_s = diff_nanos_to_seconds(call_time_nanos, self.prev_time_nanos);
        self.prev_time_nanos = call_time_nanos;

        let Some((x_apriori, p_apriori)) = self.process_ut(&asteroid, dt_s) else {
            log::error!(
                "{}: the covariance is not positive definite",
                self.config.name
            );
            return;
        };
        let Some(transform) = self.measurement_ut(&x_apriori, &p_apriori, &asteroid) else {
            log::error!(
                "{}: the a priori covariance is not positive definite",
                self.config.name
            );
            return;
        };

        // Upstream `kalmanUpdate`.
        let y = transform.dcm_an * (nav.position_bn_n_m - asteroid.position_bdy_zero_n_m);
        let gain = transform.h
            * transform.r.try_inverse().unwrap_or_else(|| {
                log::error!("{}: singular measurement covariance", self.config.name);
                Matrix3::from_element(f64::NAN)
            });
        self.x_hat_k = x_apriori + gain * (y - transform.y_hat);
        self.p_k = p_apriori - gain * transform.r * gain.transpose();

        self.small_body_nav_ukf_out_msg.write(SmallBodyNavUkfMsg {
            state: self.x_hat_k,
            covar: self.p_k,
        });
    }

    /// Upstream `processUT`: the a priori state and covariance from the
    /// sigma points of the current estimate, each propagated by one Euler
    /// step of `dt_s`. `None` when the covariance has no Cholesky factor.
    fn process_ut(
        &self,
        asteroid: &EphemerisMsg,
        dt_s: f64,
    ) -> Option<(SmallBodyNavUkfState, SmallBodyNavUkfMatrix)> {
        let omega = asteroid.omega_bn_b_radps;
        let mu = self.config.mu_ast_m3ps2;
        let sigma_points = self.sigma_points(&self.x_hat_k, &self.p_k)?;

        let mut propagated = SigmaPoints::zeros();
        for (i, point) in sigma_points.column_iter().enumerate() {
            let r = point.fixed_rows::<3>(0).into_owned();
            let v = point.fixed_rows::<3>(3).into_owned();
            let a = point.fixed_rows::<3>(6).into_owned();
            let mut derivative = SmallBodyNavUkfState::zeros();
            derivative.fixed_rows_mut::<3>(0).copy_from(&v);
            derivative.fixed_rows_mut::<3>(3).copy_from(
                &(-2.0 * omega.cross(&v)
                    - omega.cross(&omega.cross(&r))
                    - mu * r / r.norm().powi(3)
                    + a),
            );
            propagated.set_column(i, &(point + derivative * dt_s));
        }

        let x_apriori = self.weighted_mean(&propagated);
        let p_apriori = self.weighted_covariance(&propagated, &x_apriori, &propagated, &x_apriori)
            + self.config.process_noise;
        Some((x_apriori, p_apriori))
    }

    /// Upstream `measurementUT`: redraws the sigma points about the a priori
    /// estimate and transforms them into the measured position. `None` when
    /// the a priori covariance has no Cholesky factor.
    fn measurement_ut(
        &self,
        x_apriori: &SmallBodyNavUkfState,
        p_apriori: &SmallBodyNavUkfMatrix,
        asteroid: &EphemerisMsg,
    ) -> Option<MeasurementTransform> {
        let sigma_points = self.sigma_points(x_apriori, p_apriori)?;
        let measured: SMatrix<f64, 3, SIGMA_COUNT> = sigma_points.fixed_rows::<3>(0).into_owned();
        let y_hat = self.weighted_mean(&measured);
        let dcm_an = mrp_to_dcm(asteroid.sigma_bn);
        let r = self.weighted_covariance(&measured, &y_hat, &measured, &y_hat)
            + dcm_an * self.config.measurement_noise * dcm_an.transpose();
        let h = self.weighted_covariance(&sigma_points, x_apriori, &measured, &y_hat);
        Some(MeasurementTransform {
            y_hat,
            r,
            h,
            dcm_an,
        })
    }

    /// The `2 N + 1` sigma points of `mean` and `covariance`: the mean, then
    /// `mean - √(N + κ) L_i` and `mean + √(N + κ) L_i` for each column of the
    /// Cholesky factor `L`.
    fn sigma_points(
        &self,
        mean: &SmallBodyNavUkfState,
        covariance: &SmallBodyNavUkfMatrix,
    ) -> Option<SigmaPoints> {
        let factor = covariance.cholesky()?.l();
        let spread = (N as f64 + self.config.kappa).sqrt();
        let mut points = SigmaPoints::zeros();
        points.set_column(0, mean);
        for (i, column) in factor.column_iter().enumerate() {
            points.set_column(i + 1, &(mean - spread * column));
            points.set_column(N + i + 1, &(mean + spread * column));
        }
        Some(points)
    }

    /// `Σ w_m,i X_i` over the sigma points.
    fn weighted_mean<const R: usize>(
        &self,
        points: &SMatrix<f64, R, SIGMA_COUNT>,
    ) -> SVector<f64, R> {
        points
            .column_iter()
            .zip(self.wm_sigma.iter())
            .fold(SVector::zeros(), |sum, (point, weight)| {
                sum + *weight * point
            })
    }

    /// `Σ w_c,i (X_i - x̄)(Y_i - ȳ)ᵀ` over the sigma points.
    fn weighted_covariance<const R: usize, const C: usize>(
        &self,
        x: &SMatrix<f64, R, SIGMA_COUNT>,
        x_mean: &SVector<f64, R>,
        y: &SMatrix<f64, C, SIGMA_COUNT>,
        y_mean: &SVector<f64, C>,
    ) -> SMatrix<f64, R, C> {
        x.column_iter()
            .zip(y.column_iter())
            .zip(self.wc_sigma.iter())
            .fold(SMatrix::zeros(), |sum, ((x_i, y_i), weight)| {
                sum + *weight * (x_i - x_mean) * (y_i - y_mean).transpose()
            })
    }
}

impl Module for SmallBodyNavUkf {
    fn init(&mut self) {
        self.small_body_nav_ukf_out_msg
            .write(SmallBodyNavUkfMsg::default());
        self.prev_time_nanos = 0;
        SmallBodyNavUkf::reset(self);
    }

    /// Upstream `Reset`; as upstream, the filter clock is not restarted.
    fn reset(&mut self, _context: &SimulationContext) {
        SmallBodyNavUkf::reset(self);
    }

    fn update(&mut self, context: &SimulationContext) {
        self.step(context.current_sim_nanos);
    }
}

#[cfg(test)]
mod tests {
    //! Ports `smallBodyNavUKF/_UnitTest/test_smallBodyNavUKF.py`. The
    //! C-wrapped output check is the same message.

    use hifitime::Epoch;
    use nalgebra::{Matrix3, Vector3};

    use super::{
        SmallBodyNavUkf, SmallBodyNavUkfConfig, SmallBodyNavUkfMatrix, SmallBodyNavUkfState,
    };
    use crate::messages::{EphemerisMsg, NavigationTranslationMsg, Output};
    use crate::utilities::astro_constants::{AU, MU_SUN};
    use crate::{Module, SimulationContext};

    /// Upstream's 15 s task rate.
    const TASK_RATE_NANOS: u64 = 15_000_000_000;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `smallBodyNavUKFTestFunction`'s module, seeded and wired to
    /// static messages.
    fn upstream_setup() -> (
        SmallBodyNavUkf,
        Output<NavigationTranslationMsg>,
        Output<EphemerisMsg>,
    ) {
        let mut module = SmallBodyNavUkf::new(SmallBodyNavUkfConfig {
            name: "smallBodyNavUKFTag".to_string(),
            alpha: 0.0,
            beta: 2.0,
            kappa: 1e-3,
            mu_ast_m3ps2: 17.2882449693 * 1e9,
            process_noise: 0.1 * SmallBodyNavUkfMatrix::identity(),
            measurement_noise: 0.1 * Matrix3::identity(),
        });
        let vesta_radius = 2.3612 * AU * 1000.0;
        let vesta_velocity = (MU_SUN * 1000.0_f64.powi(3) / vesta_radius).sqrt();
        module.x_hat_k = SmallBodyNavUkfState::from_column_slice(&[
            2010., 1510., 1010., 0., 2., 0., 0.14, 0., 0.,
        ]);
        module.p_k =
            SmallBodyNavUkfMatrix::from_diagonal(&SmallBodyNavUkfState::from_column_slice(&[
                1000., 1000., 1000., 1., 1., 1., 1e-3, 1e-3, 1e-3,
            ]));

        let nav = Output::new(NavigationTranslationMsg {
            position_bn_n_m: Vector3::new(vesta_radius + 600. * 1000., -400. * 1000., 200. * 1000.),
            velocity_bn_n_mps: Vector3::new(0., vesta_velocity, 0.),
            ..Default::default()
        });
        let asteroid = Output::new(EphemerisMsg {
            position_bdy_zero_n_m: Vector3::new(vesta_radius, 0., 0.),
            velocity_bdy_zero_n_mps: Vector3::new(0., vesta_velocity, 0.),
            ..Default::default()
        });
        nav.connect_to(&mut module.nav_trans_in_msg);
        asteroid.connect_to(&mut module.asteroid_ephemeris_in_msg);
        (module, nav, asteroid)
    }

    /// Upstream `test_smallBodyNavUKF`: 600 s at 15 s with a static
    /// measurement of a non-rotating small body. The position converges on
    /// the measurement, the velocity on zero, and the estimated acceleration
    /// on the opposite of the Keplerian gravity there. Upstream's
    /// `compareArrayRelative` checks the first three states to 1 %
    /// (relative); the velocity and acceleration checks are added.
    #[test]
    fn matches_upstream_small_body_nav_ukf_unit_test() {
        let (mut module, _nav, _asteroid) = upstream_setup();
        module.init();
        for tick in 0..=40 {
            module.update(&context(tick * TASK_RATE_NANOS));
        }
        let state = module.small_body_nav_ukf_out_msg.read().state;

        let true_r = Vector3::new(600. * 1000., -400. * 1000., 200. * 1000.);
        let true_a = module.config.mu_ast_m3ps2 * true_r / true_r.norm().powi(3);
        for axis in 0..3 {
            let relative = (state[axis] - true_r[axis]) / true_r[axis];
            assert!(relative.abs() <= 0.01, "x_hat[{axis}] = {}", state[axis]);
        }
        // Not upstream: velocity near zero, acceleration within 1 %.
        assert!(
            state.fixed_rows::<3>(3).norm() < 1.0,
            "velocity {}",
            state.fixed_rows::<3>(3)
        );
        for axis in 0..3 {
            let relative = (state[6 + axis] - true_a[axis]) / true_a[axis];
            assert!(
                relative.abs() <= 0.01,
                "x_hat[{}] = {}",
                6 + axis,
                state[6 + axis]
            );
        }
    }

    /// Not upstream: the weights are upstream's, the mean weights sum to one,
    /// and a zero-step update with an exactly measured state keeps it.
    #[test]
    fn weights_and_zero_step_update() {
        let (mut module, nav, asteroid) = upstream_setup();
        module.config.alpha = 0.5;
        module.init();
        let n = 9.0;
        let kappa = 1e-3;
        assert_eq!(module.wm_sigma[0], kappa / (kappa + n));
        assert_eq!(module.wc_sigma[0], kappa / (kappa + n) + 1.0 - 0.25 + 2.0);
        assert!((module.wm_sigma.sum() - 1.0).abs() < 1e-14);
        for i in 1..19 {
            assert_eq!(module.wm_sigma[i], 1.0 / (2.0 * (n + kappa)));
            assert_eq!(module.wc_sigma[i], module.wm_sigma[i]);
        }

        // Measured position equal to the estimate, no motion over dt = 0:
        // the position estimate stays and its covariance shrinks.
        let relative = nav.read().position_bn_n_m - asteroid.read().position_bdy_zero_n_m;
        module.x_hat_k.fixed_rows_mut::<3>(0).copy_from(&relative);
        let position_variance = module.p_k[(0, 0)];
        module.update(&context(0));
        let output = module.small_body_nav_ukf_out_msg.read();
        assert!((output.state.fixed_rows::<3>(0) - relative).norm() < 1e-6);
        assert!(output.covar[(0, 0)] < position_variance);
        assert_eq!(output.state, module.x_hat_k);
    }

    /// Not upstream: a covariance without a Cholesky factor skips the update
    /// and keeps the estimate.
    #[test]
    fn invalid_covariance_skips_the_update() {
        let (mut module, _nav, _asteroid) = upstream_setup();
        module.p_k = -SmallBodyNavUkfMatrix::identity();
        module.init();
        let before = module.x_hat_k;
        module.update(&context(TASK_RATE_NANOS));
        assert_eq!(module.x_hat_k, before);
        assert_eq!(
            module.small_body_nav_ukf_out_msg.read().state,
            SmallBodyNavUkfState::zeros()
        );
    }
}
