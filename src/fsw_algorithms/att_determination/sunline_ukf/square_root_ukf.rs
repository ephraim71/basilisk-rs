//! Square-root unscented Kalman filter core shared by `sunlineUKF`
//! ([`super::SunlineUkf`]) and `sunlineSuKF`
//! ([`crate::fsw_algorithms::att_determination::sunline_sukf::SunlineSukf`]).
//!
//! Upstream `sunlineUKF.c` and `sunlineSuKF.c` each carry a copy of the same
//! six-state square-root UKF (van der Merwe and Wan): sigma points spread along
//! the columns of `γ sBar`, a QR factorization of the weighted deviations
//! stacked on the noise square root, and Cholesky down-dates for the zeroth
//! weight and for the Kalman update. The two filters differ only in their state
//! propagation, their CSS model and what they do when a factorization fails, so
//! this core reports failures (upstream `badUpdate`) and leaves the policy to
//! them.
//!
//! Sums keep upstream's operation order: products of dynamically sized
//! matrices go through [`sequential_product`] rather than nalgebra's
//! `matrixmultiply` kernel.

use nalgebra::{
    DMatrix, DVector, Matrix, Matrix6, RawStorage, SMatrix, SVector, U1, U6, Vector3, Vector6,
};

use crate::SimulationContext;
use crate::messages::{
    CssArraySensorMsg, CssConfigMsg, CssUnitConfigMsg, Input, MAX_N_CSS_MEAS, SKF_N_STATES,
};
use crate::utilities::linear_algebra::sequential_product;
use crate::utilities::ukf_utilities::{
    UkfResultExt, ukf_chol_decomp, ukf_chol_down_date, ukf_l_inv, ukf_qrd_just_r, ukf_u_inv,
};

/// Number of sigma points, `2 L + 1` for the [`SKF_N_STATES`] states.
pub(crate) const NUM_SIGMA_POINTS: usize = 2 * SKF_N_STATES + 1;

/// Sigma points, one per column (upstream `SP`, whose row `i` is column `i`
/// here).
pub(crate) type SigmaPoints = SMatrix<f64, SKF_N_STATES, NUM_SIGMA_POINTS>;

/// One weight per sigma point (upstream `wM`, `wC`).
pub(crate) type SigmaWeights = SVector<f64, NUM_SIGMA_POINTS>;

/// The numerical state of a six-state square-root UKF.
#[derive(Clone, Debug)]
pub(crate) struct SquareRootUkf {
    /// Upstream `state`: the estimate at the filter time tag.
    pub(crate) state: Vector6<f64>,
    /// Upstream `covar`: `sBar sBar^T`.
    pub(crate) covar: Matrix6<f64>,
    /// Upstream `sBar`: lower-triangular square root of the covariance.
    pub(crate) s_bar: Matrix6<f64>,
    /// Upstream `sQnoise`: transposed (upper-triangular) Cholesky factor of
    /// the process noise.
    pub(crate) s_q_noise: Matrix6<f64>,
    /// Upstream `SP`: the sigma points of the last time update, propagated.
    pub(crate) sigma_points: SigmaPoints,
    /// Upstream `xBar`: weighted mean of the propagated sigma points.
    pub(crate) x_bar: Vector6<f64>,
    /// Upstream `wM`: mean weights.
    pub(crate) w_m: SigmaWeights,
    /// Upstream `wC`: covariance weights.
    pub(crate) w_c: SigmaWeights,
    /// Upstream `lambdaVal`: `α² (L + κ) - L`.
    pub(crate) lambda: f64,
    /// Upstream `gamma`: sigma-point spread, `sqrt(L + λ)`.
    pub(crate) gamma: f64,
}

impl SquareRootUkf {
    /// A filter at `state` with covariance `covar`; everything else is zero,
    /// like upstream's zero-initialized configuration structure.
    pub(crate) fn new(state: Vector6<f64>, covar: Matrix6<f64>) -> Self {
        Self {
            state,
            covar,
            s_bar: Matrix6::zeros(),
            s_q_noise: Matrix6::zeros(),
            sigma_points: SigmaPoints::zeros(),
            x_bar: Vector6::zeros(),
            w_m: SigmaWeights::zeros(),
            w_c: SigmaWeights::zeros(),
            lambda: 0.0,
            gamma: 0.0,
        }
    }

    /// The filter part of upstream Reset: λ, γ and the weights from the tuning
    /// parameters, zeroed sigma points, and the Cholesky factors of the
    /// current covariance (`sBar`) and of `q_noise` (`sQnoise`, transposed).
    ///
    /// Returns `true` if a factorization failed; the factor then holds
    /// upstream's partial output.
    pub(crate) fn reset(
        &mut self,
        alpha: f64,
        beta: f64,
        kappa: f64,
        q_noise: &Matrix6<f64>,
    ) -> bool {
        let num_states = SKF_N_STATES as f64;
        self.sigma_points = SigmaPoints::zeros();
        self.lambda = alpha * alpha * (num_states + kappa) - num_states;
        self.gamma = (num_states + self.lambda).sqrt();
        self.set_weights(alpha, beta);

        let mut bad_update = false;
        self.s_bar = to_state_matrix(
            &ukf_chol_decomp(&to_dynamic(&self.covar)).into_output_flagging(&mut bad_update),
        );
        self.s_q_noise = to_state_matrix(
            &ukf_chol_decomp(&to_dynamic(q_noise)).into_output_flagging(&mut bad_update),
        )
        .transpose();
        bad_update
    }

    /// Upstream's standard unscented weights for the stored λ.
    pub(crate) fn set_weights(&mut self, alpha: f64, beta: f64) {
        let num_states = SKF_N_STATES as f64;
        self.w_m = SigmaWeights::repeat(0.5 / (num_states + self.lambda));
        self.w_c = self.w_m;
        self.w_m[0] = self.lambda / (num_states + self.lambda);
        self.w_c[0] = self.lambda / (num_states + self.lambda) + (1.0 - alpha * alpha + beta);
    }

    /// First half of upstream's time update: spreads the sigma points around
    /// the state along the columns of `±γ sBar`, propagates each with
    /// `propagate`, and accumulates their weighted mean `xBar` in upstream's
    /// interleaved `+i`, `-i` order.
    pub(crate) fn propagate_sigma_points(&mut self, mut propagate: impl FnMut(&mut Vector6<f64>)) {
        let mut central = self.state;
        propagate(&mut central);
        self.sigma_points.set_column(0, &central);
        let mut x_bar = central * self.w_m[0];
        for i in 0..SKF_N_STATES {
            for (index, spread) in [(i + 1, self.gamma), (i + 1 + SKF_N_STATES, -self.gamma)] {
                let mut point = self.s_bar.column(i) * spread + self.state;
                propagate(&mut point);
                x_bar += point * self.w_m[index];
                self.sigma_points.set_column(index, &point);
            }
        }
        self.x_bar = x_bar;
    }

    /// Whether a non-central covariance weight is negative, so that its square
    /// root does not exist (`sunlineSuKF` then rejects the update).
    pub(crate) fn has_negative_spread_weight(&self) -> bool {
        self.w_c.iter().skip(1).any(|&weight| weight < 0.0)
    }

    /// Second half of upstream's time update: the QR factorization of the
    /// weighted deviations stacked on `sQnoise` gives `sBar`, a down-date with
    /// the zeroth deviation applies `wC[0]`, and the propagated zeroth sigma
    /// point (not the mean) becomes the state. The process noise is added once
    /// per update, whatever its length.
    ///
    /// Returns `true` if the down-date failed.
    pub(crate) fn update_time_covariance(&mut self) -> bool {
        let points = to_dynamic(&self.sigma_points);
        let mean = DVector::from_column_slice(self.x_bar.as_slice());
        let stacked = stacked_deviations(&points, &mean, &self.w_c, &to_dynamic(&self.s_q_noise));
        let s_bar = ukf_qrd_just_r(&stacked).transpose();

        let mut bad_update = false;
        let s_bar = ukf_chol_down_date(&s_bar, &(points.column(0) - &mean), self.w_c[0])
            .into_output_flagging(&mut bad_update);
        self.s_bar = to_state_matrix(&s_bar);
        self.covar = self.s_bar * self.s_bar.transpose();
        self.state = self.sigma_points.column(0).into_owned();
        bad_update
    }

    /// Weighted mean of per-sigma-point predictions, one column per sigma point
    /// (upstream `yBar`).
    pub(crate) fn predicted_mean(&self, predictions: &DMatrix<f64>) -> DVector<f64> {
        predictions.column_iter().zip(self.w_m.iter()).fold(
            DVector::zeros(predictions.nrows()),
            |mean, (column, &weight)| mean + column * weight,
        )
    }

    /// Upstream's measurement update for `observations` whose predictions at
    /// the sigma points of the last time update are the columns of
    /// `predictions`, with independent measurement noise of variance `q_obs`.
    ///
    /// Returns `true` if a down-date or triangular inversion failed (as
    /// upstream, the Cholesky factor of the measurement noise is not checked).
    pub(crate) fn measurement_update(
        &mut self,
        predictions: &DMatrix<f64>,
        observations: &DVector<f64>,
        q_obs: f64,
    ) -> bool {
        let num_obs = predictions.nrows();
        let mut bad_update = false;
        let y_bar = self.predicted_mean(predictions);

        // Square root of the innovation covariance.
        let noise_root =
            ukf_chol_decomp(&(DMatrix::identity(num_obs, num_obs) * q_obs)).into_output();
        let stacked = stacked_deviations(predictions, &y_bar, &self.w_c, &noise_root);
        let s_y = ukf_qrd_just_r(&stacked).transpose();
        let s_y = ukf_chol_down_date(&s_y, &(predictions.column(0) - &y_bar), self.w_c[0])
            .into_output_flagging(&mut bad_update);

        // State-measurement cross covariance, accumulated sigma point by sigma point.
        let points = to_dynamic(&self.sigma_points);
        let x_bar = DVector::from_column_slice(self.x_bar.as_slice());
        let p_xy = points
            .column_iter()
            .zip(predictions.column_iter())
            .zip(self.w_c.iter())
            .fold(
                DMatrix::zeros(SKF_N_STATES, num_obs),
                |p_xy, ((point, prediction), &weight)| {
                    p_xy + ((point - &x_bar) * weight) * (prediction - &y_bar).transpose()
                },
            );

        // Kalman gain Pxy (Sy Sy^T)^-1, through the two triangular inverses.
        let s_y_t_inverse = ukf_u_inv(&s_y.transpose()).into_output_flagging(&mut bad_update);
        let gain = sequential_product(&p_xy, &s_y_t_inverse);
        let s_y_inverse = ukf_l_inv(&s_y).into_output_flagging(&mut bad_update);
        let gain = sequential_product(&gain, &s_y_inverse);

        let correction = sequential_product(&gain, &(observations - &y_bar));
        self.state += Vector6::from_column_slice(correction.as_slice());

        // Every column of K Sy down-dates sBar.
        let mut s_bar = to_dynamic(&self.s_bar);
        for column in sequential_product(&gain, &s_y).column_iter() {
            s_bar = ukf_chol_down_date(&s_bar, &column.into_owned(), -1.0)
                .into_output_flagging(&mut bad_update);
        }
        self.s_bar = to_state_matrix(&s_bar);
        self.covar = self.s_bar * self.s_bar.transpose();
        bad_update
    }
}

/// The sun-heading part (first three states) of a state or sigma point.
pub(crate) fn heading<S: RawStorage<f64, U6>>(point: &Matrix<f64, U6, U1, S>) -> Vector3<f64> {
    point.fixed_rows::<3>(0).into_owned()
}

/// The sensors of a CSS configuration message (upstream `numCSSTotal`,
/// `cssNHat_B`, `CBias`). Upstream logs an error for more than
/// `MAX_N_CSS_MEAS` sensors and then overruns its arrays; the port keeps the
/// first `MAX_N_CSS_MEAS`.
pub(crate) fn configured_css_units(config: &CssConfigMsg, module: &str) -> Vec<CssUnitConfigMsg> {
    if config.num_css > MAX_N_CSS_MEAS {
        log::error!(
            "{module}.css_config_in_msg.num_css must not be greater than MAX_N_CSS_MEAS ({MAX_N_CSS_MEAS})"
        );
    }
    config.active_units().to_vec()
}

/// Upstream's CSS message header (`isWritten`, `timeWritten`) as far as a port
/// can tell: Rust ports carry no header, so a connected input counts as
/// written at the current update, as upstream sees a CSS producer that runs
/// before the filter on every tick.
pub(crate) fn css_written_nanos(
    input: &Input<CssArraySensorMsg>,
    context: &SimulationContext,
) -> Option<u64> {
    input.is_connected().then_some(context.current_sim_nanos)
}

/// Upstream's `AT` matrix: the rows `sqrt(wC[i]) (column_i - mean)^T` of the
/// non-central sigma points, followed by the rows of `noise_root`.
fn stacked_deviations(
    columns: &DMatrix<f64>,
    mean: &DVector<f64>,
    w_c: &SigmaWeights,
    noise_root: &DMatrix<f64>,
) -> DMatrix<f64> {
    let dimension = mean.len();
    let spread_rows = NUM_SIGMA_POINTS - 1;
    let mut stacked = DMatrix::zeros(spread_rows + dimension, dimension);
    for (index, (column, weight)) in columns.column_iter().zip(w_c.iter()).enumerate().skip(1) {
        stacked
            .row_mut(index - 1)
            .tr_copy_from(&((column - mean) * weight.sqrt()));
    }
    stacked
        .rows_mut(spread_rows, dimension)
        .copy_from(noise_root);
    stacked
}

/// A static matrix as the [`DMatrix`] the `ukf_utilities` routines take.
fn to_dynamic<const R: usize, const C: usize>(m: &SMatrix<f64, R, C>) -> DMatrix<f64> {
    DMatrix::from_column_slice(R, C, m.as_slice())
}

/// A 6×6 `ukf_utilities` result as a state matrix.
fn to_state_matrix(m: &DMatrix<f64>) -> Matrix6<f64> {
    Matrix6::from_column_slice(m.as_slice())
}

#[cfg(test)]
pub(crate) mod test_support {
    //! Replays the message timeline of the upstream sunline-filter pytests.

    use crate::messages::CssArraySensorMsg;

    /// Upstream's 0.5 s task rate (`macros.sec2nano(0.5)`).
    pub(crate) const TASK_RATE_NANOS: u64 = 500_000_000;

    /// The pytests' CSS message between `ExecuteSimulation` calls:
    /// `cssDataInMsg.write(data, CurrentNanos)` stamps it with the time of the
    /// last executed tick, and the next call runs every tick up to its stop
    /// time. (The first call after initialization runs both t = 0 and the
    /// first tick.)
    pub(crate) struct UpstreamTimeline {
        current_nanos: u64,
        next_tick_nanos: u64,
        written: Option<(u64, CssArraySensorMsg)>,
    }

    impl UpstreamTimeline {
        pub(crate) fn new() -> Self {
            Self {
                current_nanos: 0,
                next_tick_nanos: 0,
                written: None,
            }
        }

        /// `cssDataInMsg.write(readings, unitTestSim.TotalSim.CurrentNanos)`.
        pub(crate) fn write(&mut self, readings: CssArraySensorMsg) {
            self.written = Some((self.current_nanos, readings));
        }

        /// `ConfigureStopTime(stop_nanos)` and `ExecuteSimulation()`: calls
        /// `step(call_nanos, readings, written_nanos)` for every tick up to the
        /// stop time, with the message contents and header upstream reads.
        pub(crate) fn execute_until(
            &mut self,
            stop_nanos: u64,
            mut step: impl FnMut(u64, CssArraySensorMsg, Option<u64>),
        ) {
            while self.next_tick_nanos <= stop_nanos {
                self.current_nanos = self.next_tick_nanos;
                let (written_nanos, readings) = match &self.written {
                    Some((nanos, readings)) => (Some(*nanos), readings.clone()),
                    None => (None, CssArraySensorMsg::default()),
                };
                step(self.current_nanos, readings, written_nanos);
                self.next_tick_nanos += TASK_RATE_NANOS;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{DMatrix, DVector, Matrix6, Vector6};

    use super::SquareRootUkf;

    #[test]
    fn weights_follow_the_standard_unscented_definition() {
        let mut filter = SquareRootUkf::new(Vector6::zeros(), Matrix6::identity());
        filter.reset(0.02, 2.0, 0.0, &Matrix6::identity());
        let lambda = 0.0004 * 6.0 - 6.0;
        assert!((filter.lambda - lambda).abs() <= 1.0e-15);
        // L + λ = 0.0024 cancels about 13 digits.
        assert!((filter.gamma - 0.0024_f64.sqrt()).abs() <= 1.0e-13);
        assert!((filter.w_m[0] - lambda / 0.0024).abs() <= 1.0e-8);
        assert!((filter.w_c[0] - (lambda / 0.0024 + 2.9996)).abs() <= 1.0e-8);
        for (w_m, w_c) in filter.w_m.iter().zip(filter.w_c.iter()).skip(1) {
            assert!((w_m - 0.5 / 0.0024).abs() <= 1.0e-10);
            assert_eq!(w_m, w_c);
        }
        assert!((filter.w_m.sum() - 1.0).abs() <= 1.0e-10);
    }

    #[test]
    fn a_static_time_update_adds_the_process_noise_to_the_covariance() {
        // With α = 1, κ = 0 the spread sigma points reproduce the covariance
        // exactly, so propagating nothing leaves P + Q.
        let covar = Matrix6::from_diagonal(&Vector6::new(0.4, 0.3, 0.2, 0.04, 0.03, 0.02));
        let q_noise = Matrix6::from_diagonal(&Vector6::repeat(1.0e-4));
        let state = Vector6::new(1.0, 0.5, -0.25, 0.01, 0.02, 0.03);
        let mut filter = SquareRootUkf::new(state, covar);
        assert!(!filter.reset(1.0, 2.0, 0.0, &q_noise));
        filter.propagate_sigma_points(|_| {});
        assert!(!filter.update_time_covariance());
        assert!((filter.covar - (covar + q_noise)).amax() <= 1.0e-14);
        assert_eq!(filter.state, state);
        assert!((filter.x_bar - state).amax() <= 1.0e-15);
    }

    #[test]
    fn an_update_without_observations_keeps_the_estimate() {
        let covar = Matrix6::from_diagonal(&Vector6::new(0.4, 0.3, 0.2, 0.04, 0.03, 0.02));
        let state = Vector6::new(1.0, 0.5, -0.25, 0.01, 0.02, 0.03);
        let mut filter = SquareRootUkf::new(state, covar);
        filter.reset(0.02, 2.0, 0.0, &(Matrix6::identity() * 1.0e-6));
        filter.propagate_sigma_points(|_| {});
        filter.update_time_covariance();
        let (state, covar) = (filter.state, filter.covar);
        let bad_update = filter.measurement_update(
            &DMatrix::zeros(0, super::NUM_SIGMA_POINTS),
            &DVector::zeros(0),
            1.0e-3,
        );
        assert!(!bad_update);
        assert_eq!(filter.state, state);
        assert_eq!(filter.covar, covar);
    }
}
