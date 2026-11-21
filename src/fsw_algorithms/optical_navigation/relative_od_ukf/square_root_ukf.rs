//! Square-root unscented Kalman filter core shared by the two optical-navigation
//! filters, `relativeODuKF` and `pixelLineBiasUKF`.
//!
//! Upstream carries two near-identical copies of this code (`relODuKFTimeUpdate`
//! / `pixelLineBiasUKFTimeUpdate`, `...MeasUpdate`, `...CleanUpdate`,
//! `relODStateProp`, `...TwoBodyDyn`). The copies differ only in their state and
//! measurement sizes and in a few hooks, which are parameters here: the
//! sigma-point propagation, the measurement model, the measurement noise, the
//! scale applied to the measurement deviations and a post-update state
//! constraint.
//!
//! Every product and sum is evaluated in upstream's order (`mMultM` accumulates
//! each entry from `0.0`, sigma points are summed `0, 1, 1 + N, 2, 2 + N, ...`)
//! so the filters reproduce upstream bit for bit.

use nalgebra::{DMatrix, DVector, Dim, Matrix, RawStorage, SMatrix, SVector, U1, Vector3, Vector6};

use crate::utilities::astro_constants::{
    MU_EARTH, MU_JUPITER, MU_MARS, REQ_EARTH, REQ_JUPITER, REQ_MARS,
};
use crate::utilities::linear_algebra::sequential_product;
use crate::utilities::ukf_utilities::{
    UkfResultExt, ukf_chol_decomp, ukf_chol_down_date, ukf_l_inv, ukf_qrd_just_r, ukf_u_inv,
};

/// The filter rejected an update (upstream status `-1`) and restored the
/// previous estimate where upstream does.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct BadUpdate;

/// Upstream `alpha`, `beta` and `kappa` unscented-transform parameters.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct UkfTuning {
    pub(crate) alpha: f64,
    pub(crate) beta: f64,
    pub(crate) kappa: f64,
}

/// The bodies the optical-navigation filters can navigate about (upstream
/// `planetID`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum NavigationPlanet {
    Earth = 1,
    Mars = 2,
    Jupiter = 3,
}

impl NavigationPlanet {
    /// The planet with upstream ID `id`, if the filters know it.
    pub(crate) fn from_id(id: i32) -> Option<Self> {
        match id {
            1 => Some(Self::Earth),
            2 => Some(Self::Mars),
            3 => Some(Self::Jupiter),
            _ => None,
        }
    }

    /// Gravitational parameter \[km^3/s^2\].
    pub(crate) fn mu_km3ps2(self) -> f64 {
        match self {
            Self::Earth => MU_EARTH,
            Self::Mars => MU_MARS,
            Self::Jupiter => MU_JUPITER,
        }
    }

    /// Equatorial radius \[km\].
    pub(crate) fn equatorial_radius_km(self) -> f64 {
        match self {
            Self::Earth => REQ_EARTH,
            Self::Mars => REQ_MARS,
            Self::Jupiter => REQ_JUPITER,
        }
    }
}

/// Gravitational parameter \[km^3/s^2\] of upstream planet `planet_id`.
///
/// Upstream reads an uninitialized `muPlanet` for any other ID (undefined
/// behaviour); the port returns NaN so that the resulting garbage is explicit.
pub(crate) fn planet_mu_km3ps2(planet_id: i32) -> f64 {
    NavigationPlanet::from_id(planet_id).map_or(f64::NAN, NavigationPlanet::mu_km3ps2)
}

/// Upstream `relODStateProp`: one fixed-step RK4 step of two-body motion for a
/// position/velocity state \[km, km/s\], with upstream's staging of the
/// increments.
pub(crate) fn two_body_rk4(state: &mut Vector6<f64>, mu_km3ps2: f64, dt_s: f64) {
    let half_step_s = dt_s / 2.0;
    let k1 = two_body_derivative(state, mu_km3ps2) * half_step_s;
    let k2 = two_body_derivative(&(*state + k1), mu_km3ps2) * half_step_s;
    let k3 = two_body_derivative(&(*state + k2), mu_km3ps2) * dt_s;
    let k4 = two_body_derivative(&(*state + k3), mu_km3ps2) * dt_s;
    *state += k1 * (1.0 / 3.0);
    *state += k2 * (2.0 / 3.0);
    *state += k3 * (1.0 / 3.0);
    *state += k4 * (1.0 / 6.0);
}

/// Upstream `relODuKFTwoBodyDyn`: point-mass gravity state derivative.
fn two_body_derivative(state: &Vector6<f64>, mu_km3ps2: f64) -> Vector6<f64> {
    let position = state.fixed_rows::<3>(0).into_owned();
    let mut derivative = Vector6::zeros();
    derivative
        .fixed_rows_mut::<3>(0)
        .copy_from(&state.fixed_rows::<3>(3));
    derivative
        .fixed_rows_mut::<3>(3)
        .copy_from(&(position * (-mu_km3ps2 / norm3(&position).powf(3.0))));
    derivative
}

/// Upstream `v3Norm`: the square root of a left-to-right dot product.
pub(crate) fn norm3(v: &Vector3<f64>) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Stand-in for upstream's message-header `isWritten`/`timeWritten`, which the
/// crate's ports do not carry.
///
/// A payload counts as written at the first update in which it differs from the
/// previously seen one (compared through a fingerprint `K`). This equals
/// upstream's write time whenever the producer runs before the filter in the
/// tick it writes, and its payloads change from write to write; a write that
/// repeats the previous payload exactly goes unnoticed.
#[derive(Clone, Debug)]
pub(crate) struct WriteStamp<K> {
    last_seen: Option<K>,
    written_nanos: u64,
}

impl<K> Default for WriteStamp<K> {
    fn default() -> Self {
        Self {
            last_seen: None,
            written_nanos: 0,
        }
    }
}

impl<K: PartialEq> WriteStamp<K> {
    /// Records the payload fingerprint seen at `now_nanos` and returns the
    /// emulated upstream `timeWritten` \[ns\].
    pub(crate) fn observe(&mut self, fingerprint: K, now_nanos: u64) -> u64 {
        if self.last_seen.as_ref() != Some(&fingerprint) {
            self.last_seen = Some(fingerprint);
            self.written_nanos = now_nanos;
        }
        self.written_nanos
    }
}

/// Square-root UKF with `N` states, `M` measurements and `S = 2 N + 1` sigma
/// points. Square-root covariances are lower triangular (`P = S S^T`), as the
/// upstream `sBar`.
#[derive(Clone, Debug)]
pub(crate) struct SquareRootUkf<const N: usize, const M: usize, const S: usize> {
    /// Tuning the weights were last computed from (upstream reads it from the
    /// config struct; it only changes at reset).
    tuning: UkfTuning,
    /// Upstream `lambdaVal`.
    lambda: f64,
    /// Upstream `gamma`: sigma-point spread `sqrt(N + lambda)`.
    gamma: f64,
    /// Upstream `wM`: mean weights.
    w_m: SVector<f64, S>,
    /// Upstream `wC`: covariance weights.
    w_c: SVector<f64, S>,
    /// Upstream `state`.
    pub(crate) state: SVector<f64, N>,
    /// Upstream `statePrev`: estimate restored by a rejected update.
    state_prev: SVector<f64, N>,
    /// Upstream `sBar`: lower-triangular square root of `covar`.
    s_bar: SMatrix<f64, N, N>,
    /// Upstream `sBarPrev`.
    s_bar_prev: SMatrix<f64, N, N>,
    /// Upstream `covar`.
    pub(crate) covar: SMatrix<f64, N, N>,
    /// Upstream `covarPrev`.
    covar_prev: SMatrix<f64, N, N>,
    /// Upstream `sQnoise`: the *transposed* (upper-triangular) Cholesky factor
    /// of the process noise.
    s_q_noise: SMatrix<f64, N, N>,
    /// Upstream `xBar`: weighted mean of the propagated sigma points.
    x_bar: SVector<f64, N>,
    /// Upstream `SP`: column `j` is sigma point `j` (upstream row `j`).
    pub(crate) sigma_points: SMatrix<f64, N, S>,
    /// Upstream `timeTag` \[s\]: time of the estimate.
    pub(crate) time_tag_s: f64,
    /// Upstream `dt` \[s\]: length of the last time update.
    dt_s: f64,
}

impl<const N: usize, const M: usize, const S: usize> Default for SquareRootUkf<N, M, S> {
    fn default() -> Self {
        let () = Self::SIGMA_POINT_COUNT;
        Self {
            tuning: UkfTuning::default(),
            lambda: 0.0,
            gamma: 0.0,
            w_m: SVector::zeros(),
            w_c: SVector::zeros(),
            state: SVector::zeros(),
            state_prev: SVector::zeros(),
            s_bar: SMatrix::zeros(),
            s_bar_prev: SMatrix::zeros(),
            covar: SMatrix::zeros(),
            covar_prev: SMatrix::zeros(),
            s_q_noise: SMatrix::zeros(),
            x_bar: SVector::zeros(),
            sigma_points: SMatrix::zeros(),
            time_tag_s: 0.0,
            dt_s: 0.0,
        }
    }
}

impl<const N: usize, const M: usize, const S: usize> SquareRootUkf<N, M, S> {
    const SIGMA_POINT_COUNT: () = assert!(S == 2 * N + 1, "a UKF has 2 N + 1 sigma points");

    /// The filter part of upstream `Reset_*`: weights, initial estimate and the
    /// Cholesky factors of the initial covariance and of the process noise.
    ///
    /// Fails if either factorization fails; the partial factors are kept, as
    /// upstream does.
    pub(crate) fn reset(
        &mut self,
        time_tag_s: f64,
        tuning: UkfTuning,
        state: SVector<f64, N>,
        covar: SMatrix<f64, N, N>,
        q_noise: &SMatrix<f64, N, N>,
    ) -> Result<(), BadUpdate> {
        let n = N as f64;
        self.time_tag_s = time_tag_s;
        self.dt_s = 0.0;
        self.sigma_points = SMatrix::zeros();
        self.tuning = tuning;
        self.lambda = tuning.alpha * tuning.alpha * (n + tuning.kappa) - n;
        self.gamma = (n + self.lambda).sqrt();
        self.set_standard_weights();

        self.state = state;
        self.covar = covar;
        let mut bad_update = false;
        self.s_bar =
            to_static(&ukf_chol_decomp(&to_dynamic(&covar)).into_output_flagging(&mut bad_update));
        self.s_q_noise = to_static::<N, N>(
            &ukf_chol_decomp(&to_dynamic(q_noise)).into_output_flagging(&mut bad_update),
        )
        .transpose();
        if bad_update { Err(BadUpdate) } else { Ok(()) }
    }

    /// Upstream `*TimeUpdate`: propagates the sigma points to `update_time_s`
    /// with `propagate(point, dt)` and rebuilds the state and square-root
    /// covariance from them.
    ///
    /// On failure the previous estimate is restored and the time tag is not
    /// advanced; the propagated sigma points are kept, as upstream.
    pub(crate) fn time_update(
        &mut self,
        update_time_s: f64,
        propagate: impl Fn(&mut SVector<f64, N>, f64),
    ) -> Result<(), BadUpdate> {
        self.dt_s = update_time_s - self.time_tag_s;
        self.save_previous_estimate();
        self.propagate_sigma_points(&propagate);

        let mut stacked = DMatrix::zeros(2 * N + N, N);
        for i in 0..2 * N {
            // Upstream checks the weight only once the sigma points are propagated.
            let weight = self.w_c[i + 1];
            if weight <= 0.0 {
                self.clean_update();
                return Err(BadUpdate);
            }
            let deviation = (self.sigma_points.column(i + 1) - self.x_bar) * weight.sqrt();
            stacked.row_mut(i).copy_from(&deviation.transpose());
        }
        stacked.rows_mut(2 * N, N).copy_from(&self.s_q_noise);
        let s_bar = to_static::<N, N>(&ukf_qrd_just_r(&stacked)).transpose();

        let mut bad_update = false;
        let center = self.sigma_points.column(0).into_owned();
        let center_error = center - self.x_bar;
        self.s_bar = to_static(
            &ukf_chol_down_date(&to_dynamic(&s_bar), &to_dvector(&center_error), self.w_c[0])
                .into_output_flagging(&mut bad_update),
        );
        self.covar = sequential_product(&self.s_bar, &self.s_bar.transpose());
        self.state = center;

        if bad_update {
            self.clean_update();
            return Err(BadUpdate);
        }
        self.time_tag_s = update_time_s;
        Ok(())
    }

    /// Upstream `*MeasUpdate`.
    ///
    /// * `measurement` maps the sigma points to the observation vector and to
    ///   the predicted measurement of every sigma point (column `j` for sigma
    ///   point `j`), like upstream `*MeasModel`.
    /// * `noise` is the measurement noise covariance. Upstream appends the rows
    ///   of its *lower* Cholesky factor (not of its transpose) to the QR input,
    ///   which is exact only for diagonal noise; this is kept.
    /// * `deviation_scale` multiplies the weighted measurement deviations
    ///   (upstream `noiseSF`; `1.0` leaves them unchanged).
    /// * `constrain` is applied to the updated state before the covariance
    ///   down-dates.
    ///
    /// A negative covariance weight fails without touching the estimate
    /// (upstream returns before cleaning up); a failed factorization restores
    /// the previous estimate.
    pub(crate) fn measurement_update(
        &mut self,
        measurement: impl Fn(&SMatrix<f64, N, S>) -> (SVector<f64, M>, SMatrix<f64, M, S>),
        noise: &SMatrix<f64, M, M>,
        deviation_scale: f64,
        constrain: impl Fn(&mut SVector<f64, N>),
    ) -> Result<(), BadUpdate> {
        self.save_previous_estimate();
        let (observation, predicted) = measurement(&self.sigma_points);
        let predicted_mean = self.weighted_mean(&predicted);

        let mut stacked = DMatrix::zeros(2 * N + M, M);
        for i in 0..2 * N {
            let weight = self.w_c[i + 1];
            if weight < 0.0 {
                return Err(BadUpdate);
            }
            let deviation = (predicted.column(i + 1) - predicted_mean) * weight.sqrt();
            stacked.row_mut(i).copy_from(&deviation.transpose());
        }
        stacked.rows_mut(0, 2 * N).scale_mut(deviation_scale);
        let mut bad_update = false;
        let noise_factor =
            ukf_chol_decomp(&to_dynamic(noise)).into_output_flagging(&mut bad_update);
        stacked.rows_mut(2 * N, M).copy_from(&noise_factor);
        let s_y = to_static::<M, M>(&ukf_qrd_just_r(&stacked)).transpose();

        let center_error = predicted.column(0) - predicted_mean;
        let s_y: SMatrix<f64, M, M> = to_static(
            &ukf_chol_down_date(&to_dynamic(&s_y), &to_dvector(&center_error), self.w_c[0])
                .into_output_flagging(&mut bad_update),
        );

        let mut cross_covariance = SMatrix::<f64, N, M>::zeros();
        for (i, weight) in self.w_c.iter().enumerate() {
            let measurement_error = predicted.column(i) - predicted_mean;
            let state_error = (self.sigma_points.column(i) - self.x_bar) * *weight;
            cross_covariance += sequential_product(&state_error, &measurement_error.transpose());
        }

        // K = Pxy (Sy^T)^-1 Sy^-1 through the triangular inverses; upstream
        // ignores their status.
        let s_y_t_inverse = ukf_u_inv(&to_dynamic(&s_y.transpose())).into_output();
        let gain = sequential_product(&cross_covariance, &to_static::<M, M>(&s_y_t_inverse));
        let s_y_inverse = ukf_l_inv(&to_dynamic(&s_y)).into_output();
        let gain = sequential_product(&gain, &to_static::<M, M>(&s_y_inverse));

        let innovation = observation - predicted_mean;
        self.state += sequential_product(&gain, &innovation);
        constrain(&mut self.state);

        let update = sequential_product(&gain, &s_y);
        for column in update.column_iter() {
            self.s_bar = to_static(
                &ukf_chol_down_date(&to_dynamic(&self.s_bar), &to_dvector(&column), -1.0)
                    .into_output_flagging(&mut bad_update),
            );
        }
        self.covar = sequential_product(&self.s_bar, &self.s_bar.transpose());

        if bad_update {
            self.clean_update();
            return Err(BadUpdate);
        }
        Ok(())
    }

    /// Upstream `yBar`: the `wM`-weighted mean of per-sigma-point vectors.
    pub(crate) fn weighted_mean<const R: usize>(
        &self,
        points: &SMatrix<f64, R, S>,
    ) -> SVector<f64, R> {
        points
            .column_iter()
            .zip(self.w_m.iter())
            .fold(SVector::zeros(), |sum, (point, weight)| {
                sum + point * *weight
            })
    }

    /// Upstream `*CleanUpdate`: restores the estimate saved by the last update
    /// and the standard weights.
    fn clean_update(&mut self) {
        self.state = self.state_prev;
        self.s_bar = self.s_bar_prev;
        self.covar = self.covar_prev;
        self.set_standard_weights();
    }

    fn save_previous_estimate(&mut self) {
        self.state_prev = self.state;
        self.s_bar_prev = self.s_bar;
        self.covar_prev = self.covar;
    }

    /// Standard unscented-transform weights for the stored `lambda`.
    fn set_standard_weights(&mut self) {
        let n = N as f64;
        let UkfTuning { alpha, beta, .. } = self.tuning;
        self.w_m = SVector::repeat(0.5 / (n + self.lambda));
        self.w_c = self.w_m;
        self.w_m[0] = self.lambda / (n + self.lambda);
        self.w_c[0] = self.lambda / (n + self.lambda) + (1.0 - alpha * alpha + beta);
    }

    /// Spreads the sigma points around the state with `gamma` times the
    /// columns of `sBar`, propagates them and accumulates their weighted mean in
    /// upstream's `0, 1, 1 + N, 2, 2 + N, ...` order.
    fn propagate_sigma_points(&mut self, propagate: &impl Fn(&mut SVector<f64, N>, f64)) {
        let mut center = self.state;
        propagate(&mut center, self.dt_s);
        self.sigma_points.set_column(0, &center);
        self.x_bar = center * self.w_m[0];
        for i in 0..N {
            for (index, scale) in [(i + 1, self.gamma), (i + 1 + N, -self.gamma)] {
                let mut point = self.s_bar.column(i) * scale + self.state;
                propagate(&mut point, self.dt_s);
                self.sigma_points.set_column(index, &point);
                self.x_bar = point * self.w_m[index] + self.x_bar;
            }
        }
    }
}

fn to_dynamic<const R: usize, const C: usize>(m: &SMatrix<f64, R, C>) -> DMatrix<f64> {
    DMatrix::from_column_slice(R, C, m.as_slice())
}

fn to_static<const R: usize, const C: usize>(m: &DMatrix<f64>) -> SMatrix<f64, R, C> {
    SMatrix::from_column_slice(m.as_slice())
}

fn to_dvector<D: Dim, St: RawStorage<f64, D>>(v: &Matrix<f64, D, U1, St>) -> DVector<f64> {
    DVector::from_iterator(v.nrows(), v.iter().copied())
}

/// Truth shared by the ports of `test_relativeODuKF.py` and
/// `test_pixelLineBiasUKF.py`, which carry identical copies of `rk4`,
/// `twoBodyGrav` and the initial orbit of `setupFilterData`.
#[cfg(test)]
pub(crate) mod test_support {
    use hifitime::Epoch;
    use nalgebra::{SVector, Vector3};

    use crate::SimulationContext;
    use crate::utilities::orbital_motion::{ClassicElements, elements_to_state, state_to_elements};

    /// The tests' Mars gravitational parameter `42828.314*1E9` \[m^3/s^2\].
    pub(crate) const MU_M3PS2: f64 = 42828.314 * 1.0e9;

    pub(crate) fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream test `twoBodyGrav`, for states whose leading six entries are a
    /// position and velocity \[m, m/s\]; further entries are constant.
    pub(crate) fn two_body_grav<const D: usize>(x: &SVector<f64, D>) -> SVector<f64, D> {
        let position = x.fixed_rows::<3>(0);
        let mut dxdt = SVector::<f64, D>::zeros();
        dxdt.fixed_rows_mut::<3>(0).copy_from(&x.fixed_rows::<3>(3));
        dxdt.fixed_rows_mut::<3>(3)
            .copy_from(&(-MU_M3PS2 / position.norm().powf(3.0) * position));
        dxdt
    }

    /// Upstream test `rk4`: the states at `times`, from `x0` at `times[0]`.
    pub(crate) fn rk4<const D: usize>(times: &[f64], x0: SVector<f64, D>) -> Vec<SVector<f64, D>> {
        let mut states = vec![x0];
        for step in times.windows(2) {
            let h = step[1] - step[0];
            let x = *states.last().expect("rk4 starts from x0");
            let k1 = h * two_body_grav(&x);
            let k2 = h * two_body_grav(&(x + 0.5 * k1));
            let k3 = h * two_body_grav(&(x + 0.5 * k2));
            let k4 = h * two_body_grav(&(x + k3));
            states.push(x + (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0);
        }
        states
    }

    /// `np.linspace(0, end, count)` for integer-spaced grids.
    pub(crate) fn time_grid(end_s: f64, count: usize) -> Vec<f64> {
        let step_s = end_s / (count - 1) as f64;
        (0..count).map(|i| i as f64 * step_s).collect()
    }

    /// `setupFilterData`'s initial orbit about Mars (`e = 0.2`, `i = 10`,
    /// `Omega = 0.001`, `omega = 0.01`, `f = 0.1`) with semi-major axis `a_m`.
    pub(crate) fn initial_orbit(a_m: f64) -> (Vector3<f64>, Vector3<f64>) {
        let elements = ClassicElements {
            semi_major_axis: a_m,
            eccentricity: 0.2,
            inclination_rad: 10.0,
            raan_rad: 0.001,
            arg_periapsis_rad: 0.01,
            true_anomaly_rad: 0.1,
            ..Default::default()
        };
        elements_to_state(MU_M3PS2, &elements)
    }

    /// The tests' orbital energy `-mu / (2 a)` \[m^2/s^2\].
    pub(crate) fn orbital_energy<const D: usize>(state: &SVector<f64, D>) -> f64 {
        let elements = state_to_elements(
            MU_M3PS2,
            state.fixed_rows::<3>(0).into_owned(),
            state.fixed_rows::<3>(3).into_owned(),
        );
        -MU_M3PS2 / (2.0 * elements.semi_major_axis)
    }
}
