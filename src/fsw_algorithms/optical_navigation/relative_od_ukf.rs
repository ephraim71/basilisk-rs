//! Relative orbit determination from optical-navigation position fixes: port of
//! upstream Basilisk `relativeODuKF` (`fswAlgorithms/opticalNavigation`).
//!
//! A square-root unscented Kalman filter estimates the spacecraft position and
//! velocity relative to the planet being navigated, in inertial components. The
//! measurement is the planet-relative position of an [`OpNavMsg`] (upstream
//! produced by `houghCircles` and `pixelLineConverter`); the dynamics are
//! two-body gravity of that planet (Earth, Mars or Jupiter), integrated with one
//! fixed RK4 step per sigma point. The filter works in kilometres internally;
//! its configuration, inputs and outputs are in metres, except
//! [`RelativeOdUkfConfig::q_noise`].
//!
//! Each update first processes a new valid measurement at its write time (time
//! update to the measurement, then measurement update), then propagates the
//! estimate to the current time.
//!
//! # Deviations from upstream
//!
//! * The crate's message ports carry no write header, so upstream's
//!   `opNavInMsg.isWritten()` / `timeWritten()` are emulated: the input counts as
//!   written when it is connected, at the first update in which its payload
//!   differs from the previous one. This equals upstream
//!   when the producer runs before the filter in the tick it writes.
//! * Upstream propagates with an uninitialized gravitational parameter when the
//!   planet ID is not 1, 2 or 3 (undefined behaviour); the port uses NaN and
//!   logs an error for every unknown ID, not only for upstream's ID 0.
//! * Upstream's clean-up after a rejected update recomputes the weights from
//!   the `alpha` and `beta` currently in the configuration (with the `lambda`
//!   of the last reset). The port keeps the tuning of the last reset, so
//!   changing `alpha`, `beta` or `kappa` takes effect at the next reset only.
//!
//! Upstream quirks kept: the navigation output's time tag is the reset time
//! forever (`timeTagOut` is only set by Reset); `noiseSF` scales the sigma-point
//! measurement deviations rather than the measurement noise; the measurement
//! noise enters the QR step as the rows of its lower Cholesky factor; post-fit
//! residuals are computed from the sigma points of the final propagation of
//! the update; a stale valid measurement is processed a second time when its
//! write time equals the filter time (`>=`); the filter output's `num_obs` and
//! `state_error` stay zero.

pub(super) mod square_root_ukf;

use nalgebra::{SMatrix, SVector, Vector3};

use self::square_root_ukf::{
    NavigationPlanet, SquareRootUkf, UkfTuning, WriteStamp, planet_mu_km3ps2, two_body_rk4,
};
use crate::messages::{
    Input, NavigationTranslationMsg, ODUKF_N_MEAS, ODUKF_N_STATES, OpNavFilterMsg, OpNavMsg, Output,
};
use crate::time::NANO2SEC;
use crate::{Module, SimulationContext};

/// Number of sigma points, `2 ODUKF_N_STATES + 1`.
const NUM_SIGMA_POINTS: usize = 2 * ODUKF_N_STATES + 1;

type RelativeOdFilter = SquareRootUkf<ODUKF_N_STATES, ODUKF_N_MEAS, NUM_SIGMA_POINTS>;
type SigmaPoints = SMatrix<f64, ODUKF_N_STATES, NUM_SIGMA_POINTS>;
type PredictedMeasurements = SMatrix<f64, ODUKF_N_MEAS, NUM_SIGMA_POINTS>;

/// User configuration of [`RelativeOdUkf`]. [`Default`] is upstream's
/// zero-initialized configuration.
#[derive(Clone, Debug)]
pub struct RelativeOdUkfConfig {
    /// Module name.
    pub name: String,
    /// Upstream `planetIdInit` [-]: planet navigated about until a measurement
    /// names one (Earth = 1, Mars = 2, Jupiter = 3).
    pub planet_id_init: i32,
    /// Upstream `alpha` [-]: sigma-point spread parameter.
    pub alpha: f64,
    /// Upstream `beta` [-]: prior-distribution parameter (2 for Gaussian).
    pub beta: f64,
    /// Upstream `kappa` [-]: secondary spread parameter.
    pub kappa: f64,
    /// Upstream `stateInit` \[m, m/s\]: initial planet-relative position and
    /// velocity, inertial components.
    pub state_init: SVector<f64, ODUKF_N_STATES>,
    /// Upstream `covarInit` \[m^2, m^2/s, m^2/s^2\]: initial state covariance.
    pub covar_init: SMatrix<f64, ODUKF_N_STATES, ODUKF_N_STATES>,
    /// Upstream `qNoise` \[km^2, km^2/s, km^2/s^2\]: process noise covariance
    /// added at every time update. Upstream applies no metre-to-kilometre
    /// conversion to it.
    pub q_noise: SMatrix<f64, ODUKF_N_STATES, ODUKF_N_STATES>,
    /// Upstream `noiseSF` [-]: factor applied to the weighted sigma-point
    /// measurement deviations in the measurement update.
    pub noise_sf: f64,
}

impl Default for RelativeOdUkfConfig {
    fn default() -> Self {
        Self {
            name: "relativeODuKF".to_string(),
            planet_id_init: 0,
            alpha: 0.0,
            beta: 0.0,
            kappa: 0.0,
            state_init: SVector::zeros(),
            covar_init: SMatrix::zeros(),
            q_noise: SMatrix::zeros(),
            noise_sf: 0.0,
        }
    }
}

/// Square-root UKF estimating the planet-relative translational state from
/// optical-navigation position fixes (upstream `relativeODuKF`).
#[derive(Clone, Debug)]
pub struct RelativeOdUkf {
    /// User configuration: `noise_sf` is read at every update, the rest at
    /// reset.
    pub config: RelativeOdUkfConfig,
    /// Upstream `opNavInMsg`: position fix to process (required).
    pub op_nav_in_msg: Input<OpNavMsg>,
    /// Upstream `navStateOutMsg`: estimated position and velocity.
    pub nav_state_out_msg: Output<NavigationTranslationMsg>,
    /// Upstream `filtDataOutMsg`: filter state, covariance and residuals.
    pub filt_data_out_msg: Output<OpNavFilterMsg>,
    filter: RelativeOdFilter,
    /// Upstream `planetId`: planet of the last processed measurement.
    planet_id: i32,
    /// Upstream `opNavInBuffer`: last processed measurement, in kilometres.
    op_nav_in_buffer: OpNavMsg,
    /// Upstream `timeTagOut` \[s\]: navigation output time tag.
    time_tag_out_s: f64,
    op_nav_written: WriteStamp<OpNavMsg>,
}

impl RelativeOdUkf {
    /// A filter with unconnected ports; [`Module::init`] resets it.
    pub fn new(config: RelativeOdUkfConfig) -> Self {
        Self {
            config,
            op_nav_in_msg: Input::default(),
            nav_state_out_msg: Output::default(),
            filt_data_out_msg: Output::default(),
            filter: RelativeOdFilter::default(),
            planet_id: 0,
            op_nav_in_buffer: OpNavMsg::default(),
            time_tag_out_s: 0.0,
            op_nav_written: WriteStamp::default(),
        }
    }

    /// Upstream `Reset_relODuKF`: re-initializes the estimate from the
    /// configuration (converted to kilometres) at `call_time_nanos`.
    pub fn reset(&mut self, call_time_nanos: u64) {
        if !self.op_nav_in_msg.is_connected() {
            log::error!("relativeODuKF.op_nav_in_msg is not connected");
        }

        let time_tag_s = call_time_nanos as f64 * NANO2SEC;
        self.planet_id = self.config.planet_id_init;
        let tuning = UkfTuning {
            alpha: self.config.alpha,
            beta: self.config.beta,
            kappa: self.config.kappa,
        };
        let reset = self.filter.reset(
            time_tag_s,
            tuning,
            self.config.state_init * 1.0e-3,
            self.config.covar_init * 1.0e-6,
            &self.config.q_noise,
        );
        self.time_tag_out_s = time_tag_s;
        if reset.is_err() {
            log::warn!("relODuKF: Reset method contained bad update");
        }

        // Upstream checks the connection a second time, with another message.
        if !self.op_nav_in_msg.is_connected() {
            log::error!("relODuKF: required op_nav_in_msg is not linked");
        }
    }

    /// Upstream `relODuKFTimeUpdate`.
    fn time_update(&mut self, update_time_s: f64) {
        if self.planet_id == 0 {
            log::error!("relODuKF: Need a planet to navigate");
        } else if NavigationPlanet::from_id(self.planet_id).is_none() {
            log::error!(
                "relODuKF: unknown planet ID {}; propagating with NaN gravity",
                self.planet_id
            );
        }
        let mu_km3ps2 = planet_mu_km3ps2(self.planet_id);
        // Upstream ignores the status: a rejected update has already restored
        // the previous estimate.
        let _ = self.filter.time_update(update_time_s, |state, dt_s| {
            two_body_rk4(state, mu_km3ps2, dt_s)
        });
    }

    /// Upstream `relODuKFMeasUpdate`, with the buffered measurement.
    fn measurement_update(&mut self) {
        let position_km = self.op_nav_in_buffer.position_bn_n_m;
        let _ = self.filter.measurement_update(
            |sigma_points| measurement_model(&position_km, sigma_points),
            &self.op_nav_in_buffer.covar_n_m2,
            self.config.noise_sf,
            |_| {},
        );
    }

    /// Post-fit residuals \[km\]: the buffered measurement minus the mean
    /// predicted by the current sigma points.
    fn post_fit_residuals(&self) -> Vector3<f64> {
        let (observation, predicted) = measurement_model(
            &self.op_nav_in_buffer.position_bn_n_m,
            &self.filter.sigma_points,
        );
        observation - self.filter.weighted_mean(&predicted)
    }

    fn nav_state(&self) -> NavigationTranslationMsg {
        let state = &self.filter.state;
        NavigationTranslationMsg {
            time_tag_s: self.time_tag_out_s,
            position_bn_n_m: state.fixed_rows::<3>(0) * 1.0e3,
            velocity_bn_n_mps: state.fixed_rows::<3>(3) * 1.0e3,
            ..Default::default()
        }
    }

    fn filter_data(&self, post_fits_km: Vector3<f64>) -> OpNavFilterMsg {
        OpNavFilterMsg {
            time_tag_s: self.filter.time_tag_s,
            covar: self.filter.covar * 1.0e6,
            state: self.filter.state * 1.0e3,
            post_fit_res: post_fits_km * 1.0e3,
            ..Default::default()
        }
    }
}

impl Module for RelativeOdUkf {
    fn init(&mut self) {
        self.nav_state_out_msg
            .write(NavigationTranslationMsg::default());
        self.filt_data_out_msg.write(OpNavFilterMsg::default());
        RelativeOdUkf::reset(self, 0);
    }

    fn reset(&mut self, context: &SimulationContext) {
        RelativeOdUkf::reset(self, context.current_sim_nanos);
    }

    /// Upstream `Update_relODuKF`.
    fn update(&mut self, context: &SimulationContext) {
        let call_time_nanos = context.current_sim_nanos;
        let raw = self.op_nav_in_msg.read();
        let written_nanos = self
            .op_nav_in_msg
            .is_connected()
            .then(|| self.op_nav_written.observe(raw.clone(), call_time_nanos));

        // Upstream converts the fix to kilometres before buffering it.
        let mut measurement = raw;
        measurement.position_bn_n_m *= 1.0e-3;
        measurement.covar_n_m2 *= 1.0e-6;

        let mut processed_measurement = false;
        if let Some(written_nanos) = written_nanos {
            let measurement_time_s = written_nanos as f64 * NANO2SEC;
            if measurement_time_s >= self.filter.time_tag_s && measurement.valid {
                self.planet_id = measurement.planet_id;
                self.op_nav_in_buffer = measurement;
                self.time_update(measurement_time_s);
                self.measurement_update();
                processed_measurement = true;
            }
        }

        let call_time_s = call_time_nanos as f64 * NANO2SEC;
        if call_time_s >= self.filter.time_tag_s {
            self.time_update(call_time_s);
        }

        let post_fits_km = if processed_measurement {
            self.post_fit_residuals()
        } else {
            Vector3::zeros()
        };
        self.nav_state_out_msg.write(self.nav_state());
        self.filt_data_out_msg.write(self.filter_data(post_fits_km));
    }
}

/// Upstream `relODuKFMeasModel`: the observation is the measured position and
/// each sigma point predicts its own position \[km\].
fn measurement_model(
    position_km: &Vector3<f64>,
    sigma_points: &SigmaPoints,
) -> (Vector3<f64>, PredictedMeasurements) {
    (*position_km, sigma_points.fixed_rows::<3>(0).into_owned())
}

#[cfg(test)]
mod tests {
    //! Ports `_UnitTest/test_relativeODuKF.py`: `test_methods_kf`
    //! (`relOD_method_test`), `test_propagation_kf` (`StatePropRelOD` at
    //! `dt = 10 s`) and `test_measurements_kf` (`StateUpdateRelOD`), with the
    //! same truth (the numpy RK4 in metres, re-implemented in
    //! `square_root_ukf::test_support`) and tolerances. The plots of
    //! `relativeODuKF_test_utilities.py` are not ported.
    //!
    //! `StateUpdateRelOD` writes each fix from Python between two ticks, which
    //! upstream stamps with the earlier tick. Without message headers the port
    //! stamps a write at the tick it is first seen (see the module doc), so
    //! here each fix is written in the tick it describes, before the filter
    //! runs, and an invalid message follows on the next tick so that it is
    //! processed once, as upstream. The measurement noise comes from a seeded
    //! generator instead of the unseeded `np.random.normal`.

    use nalgebra::{Matrix3, SMatrix, SVector, Vector3};
    use rand::SeedableRng;
    use rand::rngs::StdRng;
    use rand_distr::{Distribution, Normal};

    use super::square_root_ukf::test_support::{
        context, initial_orbit, orbital_energy, rk4, time_grid,
    };
    use super::square_root_ukf::{planet_mu_km3ps2, two_body_rk4};
    use super::{RelativeOdUkf, RelativeOdUkfConfig, SigmaPoints, measurement_model};
    use crate::Module;
    use crate::messages::{ODUKF_N_STATES, OpNavFilterMsg, OpNavMsg, Output};

    const NANOS_PER_SECOND: u64 = 1_000_000_000;

    type State = SVector<f64, ODUKF_N_STATES>;

    /// Upstream `setupFilterData` (with `noiseSF = 1`, as every test sets).
    fn setup_filter_data() -> RelativeOdUkfConfig {
        let (r, v) = initial_orbit(4000.0 * 1.0e3);
        let mut q_noise = SMatrix::<f64, ODUKF_N_STATES, ODUKF_N_STATES>::identity();
        q_noise
            .fixed_view_mut::<3, 3>(0, 0)
            .scale_mut(0.00001 * 0.00001 * 1.0e-6);
        q_noise
            .fixed_view_mut::<3, 3>(3, 3)
            .scale_mut(0.0001 * 0.0001 * 1.0e-6);
        RelativeOdUkfConfig {
            planet_id_init: 2,
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            state_init: State::from_column_slice(&[r.x, r.y, r.z, v.x, v.y, v.z]),
            covar_init: SMatrix::from_diagonal(&State::from_column_slice(&[
                1000.0 * 1.0e6,
                1000.0 * 1.0e6,
                1000.0 * 1.0e6,
                5.0 * 1.0e6,
                5.0 * 1.0e6,
                5.0 * 1.0e6,
            ])),
            q_noise,
            noise_sf: 1.0,
            ..Default::default()
        }
    }

    /// An initialized filter subscribed to a fresh opnav message holding
    /// `initial`.
    fn connected_filter(
        config: RelativeOdUkfConfig,
        initial: OpNavMsg,
    ) -> (RelativeOdUkf, Output<OpNavMsg>) {
        let mut module = RelativeOdUkf::new(config);
        let op_nav = Output::new(initial);
        op_nav.connect_to(&mut module.op_nav_in_msg);
        module.init();
        (module, op_nav)
    }

    fn position_fix(position_m: Vector3<f64>, time_s: f64) -> OpNavMsg {
        OpNavMsg {
            time_tag_s: time_s,
            valid: true,
            planet_id: 2,
            position_bn_n_m: position_m,
            covar_n_m2: Matrix3::from_diagonal_element(5.0 * 1.0e-2),
            ..Default::default()
        }
    }

    fn nanos(seconds: u64) -> u64 {
        seconds * NANOS_PER_SECOND
    }

    #[test]
    fn matches_upstream_methods_test() {
        let state = State::from_column_slice(&[250.0, 32000.0, 1000.0, 5.0, 3.0, 2.0]);
        let sigma_points = SigmaPoints::from_fn(|row, col| {
            let unit = |index: usize| if row == index { 1.0 } else { 0.0 };
            match col {
                0 => state[row],
                1..=ODUKF_N_STATES => state[row] + unit(col - 1),
                _ => state[row] - unit(col - 1 - ODUKF_N_STATES),
            }
        });

        // Measurement model: every sigma point predicts its own position.
        let (observation, predicted) =
            measurement_model(&Vector3::new(300.0, 200.0, 100.0), &sigma_points);
        assert_eq!(observation, Vector3::new(300.0, 200.0, 100.0));
        assert!((predicted - sigma_points.fixed_rows::<3>(0)).norm() <= 1.0e-15);

        // Dynamics model: one RK4 step about Mars against the numpy truth.
        let dt_s = 10.0;
        let mut propagated = state;
        two_body_rk4(&mut propagated, planet_mu_km3ps2(2), dt_s);
        let truth = rk4(&[0.0, dt_s], state * 1.0e3)[1] * 1.0e-3;
        let relative_error = (propagated - truth).component_div(&truth);
        assert!(relative_error.norm() <= 1.0e-15, "{relative_error}");
    }

    #[test]
    fn matches_upstream_state_propagation_test() {
        let dt_s = 10;
        let config = setup_filter_data();
        let (mut module, _op_nav) = connected_filter(config.clone(), OpNavMsg::default());

        let time_sim_s = 60 * 60;
        let steps = time_sim_s / dt_s;
        let state_log: Vec<State> = (0..=steps)
            .map(|step| {
                module.update(&context(nanos(step * dt_s)));
                module.filt_data_out_msg.read().state
            })
            .collect();

        let time = time_grid(time_sim_s as f64, steps as usize + 1);
        let expected = rk4(&time, config.state_init);
        let energy: Vec<f64> = expected.iter().map(orbital_energy).collect();

        let last = expected.len() - 1;
        let relative_error = (state_log[last] - expected[last]).component_div(&expected[last]);
        assert!(relative_error.norm() <= 1.0e-10, "{relative_error}");
        assert!((energy[0] - energy[last]) / energy[0] <= 1.0e-10);
    }

    #[test]
    fn matches_upstream_state_update_test() {
        let t1 = 250;
        let mult_t1 = 8;
        let end = mult_t1 * t1;
        let config = setup_filter_data();

        // Truth: free flight, then a velocity kick added to the state of
        // `t1 - 1` and flown again from `t1`.
        let time = time_grid(end as f64, end + 1);
        let mut expected = rk4(&time[..t1], config.state_init);
        let kick = State::from_column_slice(&[0.0, 0.0, 0.0, -0.01, 0.01, 0.02]) * 10.0 * 1.0e3;
        expected.extend(rk4(&time[t1..], expected[t1 - 1] + kick));

        let initial = OpNavMsg {
            planet_id: 2,
            position_bn_b_m: expected[0].fixed_rows::<3>(0).into_owned(),
            ..Default::default()
        };
        let (mut module, op_nav) = connected_filter(config, initial);
        let noise = Normal::new(0.0, 5.0 * 1.0e-2).expect("valid distribution");
        let mut rng = StdRng::seed_from_u64(20190620);

        let mut log: Vec<OpNavFilterMsg> = Vec::new();
        for (t, true_state) in expected.iter().enumerate() {
            if t > 0 && t % 50 == 0 && t < end {
                let noise_m = Vector3::from_fn(|_, _| noise.sample(&mut rng));
                let position_m = true_state.fixed_rows::<3>(0) + noise_m;
                op_nav.write(position_fix(position_m, t as f64));
            } else if t > 50 && t % 50 == 1 {
                op_nav.write(OpNavMsg {
                    valid: false,
                    ..op_nav.read()
                });
            }
            module.update(&context(nanos(t as u64)));
            log.push(module.filt_data_out_msg.read());
        }

        for i in 0..ODUKF_N_STATES {
            let initial_variance = log[0].covar[(i, i)];
            assert!(log[t1].covar[(i, i)] <= initial_variance / 100.0, "at {t1}");
            assert!(
                log[end].covar[(i, i)] <= initial_variance / 100.0,
                "at {end}"
            );
        }
        let relative_error = (log[end].state - expected[end]).component_div(&expected[end]);
        assert!(relative_error.norm() <= 1.0e-1, "{relative_error}");
    }

    /// Asserts `actual` equals the upstream C values to 1e-12, relative for
    /// magnitudes above one and absolute below.
    fn assert_matches_c(actual: &[f64], expected: &[f64], what: &str) {
        assert_eq!(actual.len(), expected.len(), "{what}");
        for (i, (a, e)) in actual.iter().zip(expected).enumerate() {
            let tolerance = 1.0e-12 * e.abs().max(1.0);
            assert!((a - e).abs() <= tolerance, "{what}[{i}]: {a} vs {e}");
        }
    }

    #[test]
    fn matches_compiled_upstream_on_a_correlated_fix() {
        // Truth from upstream relativeODuKF.c (gcc -O2 -ffp-contract=off) on
        // the same inputs. It pins the quirks no Python test reaches: noiseSF
        // scaling the measurement deviations, the correlated noise entering
        // as rows of its lower Cholesky factor, and post-fits taken from the
        // zero-length propagation that follows the update.
        let mut covar_init = SMatrix::from_diagonal(&State::from_column_slice(&[
            1.0e9, 1.0e9, 1.0e9, 5.0e6, 5.0e6, 5.0e6,
        ]));
        covar_init[(0, 1)] = 2.0e8;
        covar_init[(1, 0)] = 2.0e8;
        let config = RelativeOdUkfConfig {
            planet_id_init: 2,
            alpha: 0.02,
            beta: 2.0,
            kappa: 0.0,
            noise_sf: 2.5,
            state_init: State::from_column_slice(&[4.0e6, 1.0e6, 3.0e5, -600.0, 3000.0, 200.0]),
            covar_init,
            q_noise: SMatrix::from_diagonal(&State::from_column_slice(&[
                1.0e-16, 1.0e-16, 1.0e-16, 1.0e-14, 1.0e-14, 1.0e-14,
            ])),
            ..Default::default()
        };
        let (mut module, op_nav) = connected_filter(config, OpNavMsg::default());
        module.update(&context(0));
        module.update(&context(nanos(1)));

        let fix = OpNavMsg {
            time_tag_s: 2.0,
            valid: true,
            planet_id: 2,
            position_bn_n_m: Vector3::new(3_998_900.0, 1_005_800.0, 300_500.0),
            covar_n_m2: Matrix3::new(
                2500.0, 300.0, 0.0, 300.0, 1600.0, -200.0, 0.0, -200.0, 900.0,
            ),
            ..Default::default()
        };
        op_nav.write(fix.clone());
        module.update(&context(nanos(2)));
        let output = module.filt_data_out_msg.read();
        assert_matches_c(
            output.state.as_slice(),
            &[
                3998811.927038357,
                1005966.9796253681,
                300415.6943592274,
                -604.6136874184853,
                2998.4263184279635,
                199.79349513840936,
            ],
            "state at 2 s",
        );
        assert_matches_c(
            output.post_fit_res.as_slice(),
            &[88.07295919405078, -166.97962540195022, 84.30564087757375],
            "post-fit residuals at 2 s",
        );
        let covar = |filter: &OpNavFilterMsg| {
            let mut entries: Vec<f64> = filter.covar.diagonal().iter().copied().collect();
            entries.push(filter.covar[(0, 1)]);
            entries
        };
        assert_matches_c(
            &covar(&output),
            &[
                856804058.2462381,
                856798638.3130785,
                856798004.5447264,
                4983701.542349611,
                4983679.68480034,
                4984305.554550836,
                168001648.81409988,
            ],
            "covariance at 2 s",
        );

        op_nav.write(OpNavMsg {
            valid: false,
            ..fix
        });
        module.update(&context(nanos(3)));
        let output = module.filt_data_out_msg.read();
        assert_matches_c(
            output.state.as_slice(),
            &[
                3998206.1017022706,
                1008965.1008154089,
                300615.3968029924,
                -607.0368769155981,
                2997.8157705849803,
                199.61137572704018,
            ],
            "state at 3 s",
        );
        assert_eq!(output.post_fit_res, Vector3::zeros());
        assert_matches_c(
            &covar(&output),
            &[
                878592802.247596,
                878580544.980817,
                878579760.7374424,
                4983725.391803245,
                4983668.838486621,
                4984292.563673855,
                168006929.42343056,
            ],
            "covariance at 3 s",
        );
        assert_eq!(module.nav_state_out_msg.read().time_tag_s, 0.0);
    }

    #[test]
    fn a_stale_valid_fix_is_processed_again_when_its_write_time_equals_the_filter_time() {
        // Upstream accepts a fix written at or after the filter time (`>=`),
        // and processing it leaves the filter time at the write time.
        let config = setup_filter_data();
        let (r, _) = initial_orbit(4000.0 * 1.0e3);
        let (mut stale, stale_op_nav) = connected_filter(config.clone(), OpNavMsg::default());
        let (mut fresh, fresh_op_nav) = connected_filter(config, OpNavMsg::default());

        for t in 0..=3 {
            if t == 1 {
                stale_op_nav.write(position_fix(r, 1.0));
                fresh_op_nav.write(position_fix(r, 1.0));
            }
            if t == 2 {
                fresh_op_nav.write(OpNavMsg::default());
            }
            stale.update(&context(nanos(t)));
            fresh.update(&context(nanos(t)));
            let stale_residuals = stale.filt_data_out_msg.read().post_fit_res;
            let fresh_residuals = fresh.filt_data_out_msg.read().post_fit_res;
            match t {
                1 => {
                    assert_eq!(stale_residuals, fresh_residuals);
                    assert_ne!(stale_residuals, Vector3::zeros());
                }
                2 => {
                    assert_ne!(stale_residuals, Vector3::zeros());
                    assert_eq!(fresh_residuals, Vector3::zeros());
                }
                _ => {
                    assert_eq!(stale_residuals, Vector3::zeros());
                    assert_eq!(fresh_residuals, Vector3::zeros());
                }
            }
        }
    }

    #[test]
    fn a_fix_with_an_indefinite_covariance_restores_the_propagated_estimate() {
        let config = setup_filter_data();
        let (r, _) = initial_orbit(4000.0 * 1.0e3);
        let (mut rejected, op_nav) = connected_filter(config.clone(), OpNavMsg::default());
        let (mut reference, _reference_op_nav) = connected_filter(config, OpNavMsg::default());

        for t in 0..=2 {
            if t == 2 {
                op_nav.write(OpNavMsg {
                    covar_n_m2: -Matrix3::identity(),
                    ..position_fix(r + Vector3::new(500.0, 0.0, 0.0), 2.0)
                });
            }
            rejected.update(&context(nanos(t)));
            reference.update(&context(nanos(t)));
        }
        // The failed Cholesky factorization of the measurement noise restores
        // the time-updated estimate; the zero-length propagation that follows
        // leaves the state itself unchanged.
        assert_eq!(
            rejected.filt_data_out_msg.read().state,
            reference.filt_data_out_msg.read().state
        );
    }

    #[test]
    fn navigation_output_keeps_the_reset_time_tag() {
        let config = setup_filter_data();
        let (mut module, _op_nav) = connected_filter(config.clone(), OpNavMsg::default());
        module.reset(nanos(5));
        for t in 5..=8 {
            module.update(&context(nanos(t)));
            assert_eq!(module.nav_state_out_msg.read().time_tag_s, 5.0);
            assert_eq!(module.filt_data_out_msg.read().time_tag_s, t as f64);
        }

        // Reset restarts the estimate from the configured state.
        module.reset(nanos(8));
        module.update(&context(nanos(8)));
        let state = module.filt_data_out_msg.read().state;
        assert!((state - config.state_init).norm() <= 1.0e-12 * config.state_init.norm());
    }

    #[test]
    fn an_unconnected_input_only_propagates_like_a_never_valid_one() {
        let config = setup_filter_data();
        let mut unconnected = RelativeOdUkf::new(config.clone());
        unconnected.init();
        let (mut connected, _op_nav) = connected_filter(config, OpNavMsg::default());
        for t in 0..=5 {
            unconnected.update(&context(nanos(t * 10)));
            connected.update(&context(nanos(t * 10)));
            assert_eq!(
                unconnected.filt_data_out_msg.read(),
                connected.filt_data_out_msg.read()
            );
        }
    }

    #[test]
    fn an_unknown_planet_propagates_with_nan_gravity() {
        let config = RelativeOdUkfConfig {
            planet_id_init: 7,
            ..setup_filter_data()
        };
        let (mut module, _op_nav) = connected_filter(config, OpNavMsg::default());
        module.update(&context(nanos(10)));
        let velocity = module.nav_state_out_msg.read().velocity_bn_n_mps;
        assert!(velocity.iter().all(|component| component.is_nan()));
    }
}
