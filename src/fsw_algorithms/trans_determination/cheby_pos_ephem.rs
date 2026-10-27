//! Chebyshev-fit position and velocity ephemeris: port of upstream Basilisk
//! `chebyPosEphem`.
//!
//! The user loads up to [`MAX_CHEB_RECORDS`] records, each a Chebyshev series
//! for the inertial position of a body over a time interval. Every update
//! converts the vehicle clock to ephemeris time (TDB) with the clock
//! correlation message, picks the first record whose interval contains that
//! time (record 0 if none does), scales the time to `[-1, 1]` within the
//! interval and evaluates the series. Times outside the interval rail to its
//! boundary. The velocity comes from the derivative series, which reset
//! computes from the position coefficients.
//!
//! Upstream stores each record's coefficients in fixed buffers of at most 40
//! terms per axis together with a count, `nChebCoeff`. Here the count is the
//! length of [`ChebyEphemRecord::pos_cheby_coeff`], which has no fixed limit.
//! The velocity series keeps upstream's flat, persistent buffer layout,
//! because reset leaves one slot per axis unwritten and upstream then reads
//! whatever an earlier reset left there; the port keeps that behaviour.

use nalgebra::Vector3;

use super::ephemeris_utilities::{calculate_cheby_value, ephemeris_time_s, scaled_fit_time};
use crate::messages::{EphemerisMsg, Input, Output, TdbVehicleClockCorrelationMsg};
use crate::time::NANO2SEC;
use crate::{Module, SimulationContext};

/// Upstream `MAX_CHEB_RECORDS`: number of fit records the module holds.
pub const MAX_CHEB_RECORDS: usize = 4;

/// Upstream `MAX_CHEB_COEFF`: terms per axis that upstream's coefficient
/// buffers hold, and the initial size of each velocity buffer.
const MAX_CHEB_COEFF: usize = 40;

/// One Chebyshev fit of a body's position: upstream `ChebyEphemRecord`.
///
/// The default record is upstream's zeroed one: an empty series centred on
/// ephemeris time zero with a zero radius.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ChebyEphemRecord {
    /// Upstream `ephemTimeMid` \[s\]: ephemeris time (TDB) at the middle of the
    /// fit interval.
    pub ephem_time_mid_s: f64,
    /// Upstream `ephemTimeRad` \[s\]: half the length of the fit interval.
    pub ephem_time_rad_s: f64,
    /// Upstream `posChebyCoeff` \[m\]: entry `i` holds the coefficients of `T_i`
    /// for the inertial x, y and z position. Its length is upstream's
    /// `nChebCoeff`.
    pub pos_cheby_coeff: Vec<Vector3<f64>>,
}

/// User-settable parameters of [`ChebyPosEphem`].
#[derive(Clone, Debug)]
pub struct ChebyPosEphemConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `ephArray`: the fit records, searched in order.
    pub eph_array: [ChebyEphemRecord; MAX_CHEB_RECORDS],
}

impl Default for ChebyPosEphemConfig {
    fn default() -> Self {
        Self {
            name: "chebyPosEphem".to_string(),
            eph_array: Default::default(),
        }
    }
}

/// Chebyshev position/velocity ephemeris: upstream `chebyPosEphem`.
#[derive(Clone, Debug)]
pub struct ChebyPosEphem {
    /// User-settable parameters, read at reset (velocity coefficients) and on
    /// every update (records).
    pub config: ChebyPosEphemConfig,
    /// Upstream `clockCorrInMsg`: vehicle clock to TDB correlation.
    pub clock_corr_in_msg: Input<TdbVehicleClockCorrelationMsg>,
    /// Upstream `posFitOutMsg`: fitted position and velocity; the attitude
    /// fields stay zero.
    pub pos_fit_out_msg: Output<EphemerisMsg>,
    /// Upstream `coeffSelector`: index of the record used by the last update.
    coeff_selector: usize,
    /// Upstream `ephArray[i].velChebyCoeff`: derivative series of each record,
    /// already divided by the record's time radius, in upstream's layout: the
    /// x, y and z series in consecutive blocks of `nChebCoeff` terms. Like
    /// upstream's buffer, it starts zeroed and persists across resets.
    vel_cheby_coeff: [Vec<f64>; MAX_CHEB_RECORDS],
}

impl ChebyPosEphem {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: ChebyPosEphemConfig) -> Self {
        Self {
            config,
            clock_corr_in_msg: Input::default(),
            pos_fit_out_msg: Output::default(),
            coeff_selector: 0,
            vel_cheby_coeff: std::array::from_fn(|_| vec![0.0; 3 * MAX_CHEB_COEFF]),
        }
    }

    /// Upstream `Reset_chebyPosEphem`: checks the clock input and derives the
    /// velocity series of every record from its position series.
    fn reset_velocity_coefficients(&mut self) {
        if !self.clock_corr_in_msg.is_connected() {
            log::error!("chebyPosEphem.clock_corr_in_msg is not connected");
        }
        for (velocity, record) in self.vel_cheby_coeff.iter_mut().zip(&self.config.eph_array) {
            write_velocity_coefficients(velocity, &record.pos_cheby_coeff, record.ephem_time_rad_s);
        }
    }

    /// Upstream `Update_chebyPosEphem`: the fitted state at `call_time_nanos`.
    fn fitted_state(&mut self, call_time_nanos: u64) -> EphemerisMsg {
        let correlation = self.clock_corr_in_msg.read();
        let current_eph_time_s = ephemeris_time_s(call_time_nanos, &correlation);

        let selector = self
            .config
            .eph_array
            .iter()
            .position(|record| {
                (current_eph_time_s - record.ephem_time_mid_s).abs() <= record.ephem_time_rad_s
            })
            .unwrap_or(0);
        self.coeff_selector = selector;
        let record = &self.config.eph_array[selector];
        let scaled_time = scaled_fit_time(
            current_eph_time_s,
            record.ephem_time_mid_s,
            record.ephem_time_rad_s,
        );

        // Upstream indexes the velocity buffer with the current count, even
        // if it changed after the last reset; slots never written are zero.
        let n = record.pos_cheby_coeff.len();
        let velocity = &mut self.vel_cheby_coeff[selector];
        if velocity.len() < 3 * n {
            velocity.resize(3 * n, 0.0);
        }
        let velocity_axis =
            |axis: usize| calculate_cheby_value(&velocity[axis * n..(axis + 1) * n], scaled_time);

        EphemerisMsg {
            position_bdy_zero_n_m: calculate_cheby_value(&record.pos_cheby_coeff, scaled_time),
            velocity_bdy_zero_n_mps: Vector3::new(
                velocity_axis(0),
                velocity_axis(1),
                velocity_axis(2),
            ),
            time_tag_s: call_time_nanos as f64 * NANO2SEC,
            ..Default::default()
        }
    }
}

/// Writes the time-derivative series of `position` into `velocity`, upstream's
/// `velChebyCoeff` buffer of the record, with upstream's algorithm and order
/// of operations: the series derivative with respect to the scaled time,
/// multiplied by `1 / time_radius_s`, for each axis in a block of
/// `n = position.len()` slots starting at `axis * n`.
///
/// The derivative of an `n`-term series has `n - 1` terms. For `n >= 3`
/// upstream never writes the last slot of a block, yet still scales it by
/// `1 / time_radius_s`. That slot is zero after the zeroed start, but it holds
/// a stale, rescaled coefficient if an earlier reset ran with a different
/// count. This upstream bug is kept: the buffer persists across resets. For
/// `n < 2` upstream also writes slots 0 and 1 of the last block.
fn write_velocity_coefficients(
    velocity: &mut Vec<f64>,
    position: &[Vector3<f64>],
    time_radius_s: f64,
) {
    let n = position.len();
    let written = (3 * n).max(2 * n + 2);
    if velocity.len() < written {
        velocity.resize(written, 0.0);
    }
    let inverse_radius = 1.0 / time_radius_s;
    for axis in 0..3 {
        let block = &mut velocity[axis * n..];
        // Upstream's scratch buffer, zero beyond the loaded coefficients.
        let mut scratch: Vec<f64> = position.iter().map(|c| c[axis]).collect();
        scratch.resize(n.max(3), 0.0);
        for j in (2..n.saturating_sub(1)).rev() {
            block[j] = (2 * (j + 1)) as f64 * scratch[j + 1];
            scratch[j - 1] += ((j + 1) as f64 * scratch[j + 1]) / (j - 1) as f64;
        }
        block[1] = 4.0 * scratch[2];
        block[0] = scratch[1];
        for coefficient in &mut block[..n] {
            *coefficient *= inverse_radius;
        }
    }
}

impl Module for ChebyPosEphem {
    fn init(&mut self) {
        self.pos_fit_out_msg.write(EphemerisMsg::default());
        self.reset_velocity_coefficients();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.reset_velocity_coefficients();
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.fitted_state(context.current_sim_nanos);
        self.pos_fit_out_msg.write(state);
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_chebyPosEphem.py` has two cases. `sineCosine` is ported
    //! as is. `earthOrbitFit` fits Hubble's orbit read with SPICE from
    //! `hst_edited.bsp`, whose Hubble segments are SPK type 10 (two-line
    //! elements, evaluated with SGP4). Neither ANISE nor this crate evaluates
    //! type 10, so the port fits a two-body orbit with Hubble-like elements
    //! instead, sampled at the same ephemeris times; the fit degree, sample
    //! count, task rate, clock correlation and tolerances are upstream's. The
    //! plots are not ported.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::Vector3;

    use super::{ChebyEphemRecord, ChebyPosEphem, ChebyPosEphemConfig};
    use crate::fsw_algorithms::trans_determination::ephemeris_utilities::test_support::{
        chebfit, linspace,
    };
    use crate::messages::{EphemerisMsg, Output, TdbVehicleClockCorrelationMsg};
    use crate::utilities::astro_constants::{D2R, MU_EARTH};
    use crate::utilities::orbital_motion::{
        ClassicElements, eccentric_to_true_anomaly, elements_to_state, mean_to_eccentric_anomaly,
    };
    use crate::{Module, SimulationContext};

    /// Upstream `orbitPosAccuracy` (in the length unit of each fit).
    const ORBIT_POS_ACCURACY: f64 = 1.0;
    /// Upstream `orbitVelAccuracy`.
    const ORBIT_VEL_ACCURACY: f64 = 0.01;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Upstream `macros.sec2nano`: `int(seconds * 1e9 + 0.5)`.
    fn sec2nano(seconds: f64) -> u64 {
        (seconds * 1.0e9 + 0.5) as u64
    }

    /// Builds the module with `record` in slot 0 and the clock correlation
    /// upstream uses: vehicle time 0 at the start of the fit interval.
    fn fitted_module(
        record: ChebyEphemRecord,
    ) -> (ChebyPosEphem, Output<TdbVehicleClockCorrelationMsg>) {
        let clock = Output::new(TdbVehicleClockCorrelationMsg {
            vehicle_clock_time_s: 0.0,
            ephemeris_time_s: record.ephem_time_mid_s - record.ephem_time_rad_s,
        });
        let mut module = ChebyPosEphem::new(ChebyPosEphemConfig::default());
        module.config.eph_array[0] = record;
        clock.connect_to(&mut module.clock_corr_in_msg);
        (module, clock)
    }

    /// Runs the module at `task_nanos` from 0 through `stop_nanos`, recording
    /// the output after every update, as upstream's recorder does.
    fn run(module: &mut ChebyPosEphem, task_nanos: u64, stop_nanos: u64) -> Vec<EphemerisMsg> {
        module.init();
        (0..)
            .map(|step| step * task_nanos)
            .take_while(|&time| time <= stop_nanos)
            .map(|time| {
                module.update(&context(time));
                module.pos_fit_out_msg.read()
            })
            .collect()
    }

    /// Upstream's error metric `abs(max(fit[:, i] - truth[:, i]))`, the
    /// absolute value of the largest signed difference, maximized over the
    /// axes.
    fn upstream_max_error(fit: &[Vector3<f64>], truth: &[Vector3<f64>]) -> f64 {
        assert_eq!(fit.len(), truth.len());
        (0..3)
            .map(|axis| {
                fit.iter()
                    .zip(truth)
                    .map(|(f, t)| f[axis] - t[axis])
                    .fold(f64::NEG_INFINITY, f64::max)
                    .abs()
            })
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Upstream `sineCosine`: a circle-plus-offset curve over three years,
    /// fitted with 22 coefficients and checked at ten times the fit density.
    #[test]
    fn matches_upstream_sine_cosine_fit() {
        let orbit_radius = 70000.0;
        let num_curve_points = 365 * 3 + 1;
        let curve_duration_days = 365.0 * 3.0;
        let deg_cheb_coeff = 21;
        let curve = |angle: f64| {
            Vector3::new(
                angle.cos() * orbit_radius,
                angle.sin() * orbit_radius,
                angle.sin() + orbit_radius,
            )
        };

        let angle_space = linspace(-3.0 * PI, 3.0 * PI, num_curve_points);
        let values: Vec<Vector3<f64>> = angle_space.iter().map(|&angle| curve(angle)).collect();
        let fit_times = linspace(-1.0, 1.0, num_curve_points);
        // SPICE str2et of "2019 APR 1 12:12:12.0 (UTC)".
        let et_mid = Epoch::from_gregorian_utc_hms(2019, 4, 1, 12, 12, 12).to_et_seconds();

        let (mut module, _clock) = fitted_module(ChebyEphemRecord {
            ephem_time_mid_s: et_mid,
            ephem_time_rad_s: curve_duration_days / 2.0 * 86400.0,
            pos_cheby_coeff: chebfit(&fit_times, &values, deg_cheb_coeff),
        });
        let samples = run(
            &mut module,
            sec2nano(8640.0),
            (curve_duration_days * 86400.0 * 1.0e9) as u64,
        );

        let angle_space_fine = linspace(-3.0 * PI, 3.0 * PI, num_curve_points * 10 - 9);
        assert_eq!(samples.len(), angle_space_fine.len());
        let max_error = samples
            .iter()
            .zip(&angle_space_fine)
            .map(|(sample, &angle)| (sample.position_bdy_zero_n_m - curve(angle)).amax())
            .fold(0.0, f64::max);
        assert!(
            max_error < ORBIT_POS_ACCURACY,
            "sine wave error {max_error}"
        );
    }

    /// Two-body stand-in for Hubble's SPICE ephemeris: position \[km\] and
    /// velocity \[km/s\] relative to the Earth, inertial components, at
    /// `seconds_since_start` past the fit start.
    fn hubble_like_state(seconds_since_start: f64) -> (Vector3<f64>, Vector3<f64>) {
        let semi_major_axis_km: f64 = 6918.0;
        let eccentricity = 2.8e-4;
        let initial_mean_anomaly_rad = 10.0 * D2R;
        let mean_motion = (MU_EARTH / semi_major_axis_km.powi(3)).sqrt();
        let mean_anomaly =
            (initial_mean_anomaly_rad + mean_motion * seconds_since_start).rem_euclid(2.0 * PI);
        let eccentric_anomaly = mean_to_eccentric_anomaly(mean_anomaly, eccentricity);
        let elements = ClassicElements {
            semi_major_axis: semi_major_axis_km,
            eccentricity,
            inclination_rad: 28.47 * D2R,
            raan_rad: 145.0 * D2R,
            arg_periapsis_rad: 80.0 * D2R,
            true_anomaly_rad: eccentric_to_true_anomaly(eccentric_anomaly, eccentricity),
            ..Default::default()
        };
        elements_to_state(MU_EARTH, &elements)
    }

    /// Upstream `earthOrbitFit` on a two-body Hubble-like orbit (see the
    /// module comment): three orbits fitted with 24 coefficients, position
    /// checked to 1 km and the derived velocity to 0.01 km/s.
    #[test]
    fn matches_upstream_earth_orbit_fit_on_a_two_body_hubble_orbit() {
        let num_curve_points = 365 * 3 + 1;
        let curve_duration_seconds = 3.0 * 5950.0;
        let deg_cheb_coeff = 23;
        // SPICE str2et of "2015 February 10, 00:00:00.0 TDB": 5518.5 days
        // after J2000 (2000-01-01 12:00 TDB).
        let et_start = 5518.5 * 86400.0;
        let et_end = et_start + curve_duration_seconds;

        let time_history = linspace(et_start, et_end, num_curve_points);
        let (truth_positions, truth_velocities): (Vec<_>, Vec<_>) = time_history
            .iter()
            .map(|&time| hubble_like_state(time - et_start))
            .unzip();
        let fit_times = linspace(-1.0, 1.0, num_curve_points);

        let (mut module, _clock) = fitted_module(ChebyEphemRecord {
            ephem_time_mid_s: et_start + curve_duration_seconds / 2.0,
            ephem_time_rad_s: curve_duration_seconds / 2.0,
            pos_cheby_coeff: chebfit(&fit_times, &truth_positions, deg_cheb_coeff),
        });
        let samples = run(
            &mut module,
            sec2nano(curve_duration_seconds / (num_curve_points - 1) as f64),
            (curve_duration_seconds * 1.0e9) as u64,
        );

        let positions: Vec<_> = samples.iter().map(|s| s.position_bdy_zero_n_m).collect();
        let velocities: Vec<_> = samples.iter().map(|s| s.velocity_bdy_zero_n_mps).collect();
        let position_error = upstream_max_error(&positions, &truth_positions);
        let velocity_error = upstream_max_error(&velocities, &truth_velocities);
        assert!(
            position_error < ORBIT_POS_ACCURACY,
            "orbit accuracy {position_error}"
        );
        assert!(
            velocity_error < ORBIT_VEL_ACCURACY,
            "velocity accuracy {velocity_error}"
        );
    }

    /// `d/dx T_n(x) = n U_{n-1}(x)`, with the second-kind polynomials from
    /// `U_0 = 1`, `U_1 = 2x`, `U_k = 2x U_{k-1} - U_{k-2}`.
    fn series_derivative(coefficients: &[Vector3<f64>], x: f64) -> Vector3<f64> {
        let mut second_kind = vec![1.0, 2.0 * x];
        for k in 2..coefficients.len() {
            second_kind.push(2.0 * x * second_kind[k - 1] - second_kind[k - 2]);
        }
        coefficients
            .iter()
            .enumerate()
            .skip(1)
            .map(|(n, c)| c * (n as f64 * second_kind[n - 1]))
            .sum()
    }

    // Not upstream: the velocity is the time derivative of the position
    // series for every coefficient count, including upstream's special cases
    // n <= 3.
    #[test]
    fn velocity_is_the_time_derivative_of_the_position_series() {
        let radius_s = 250.0;
        for n in 0..=9 {
            let coefficients: Vec<Vector3<f64>> = (0..n)
                .map(|i| {
                    let i = i as f64;
                    Vector3::new(1.0 + i, -0.5 * i * i, 3.0 / (1.0 + i))
                })
                .collect();
            let (mut module, _clock) = fitted_module(ChebyEphemRecord {
                ephem_time_mid_s: 1000.0,
                ephem_time_rad_s: radius_s,
                pos_cheby_coeff: coefficients.clone(),
            });
            module.init();
            // Vehicle time 0 is the start of the interval (scaled time -1).
            for elapsed_s in [0.0, 70.0, 250.0, 333.3, 500.0] {
                module.update(&context((elapsed_s * 1.0e9) as u64));
                let x = (elapsed_s - radius_s) / radius_s;
                let expected = series_derivative(&coefficients, x) / radius_s;
                let velocity = module.pos_fit_out_msg.read().velocity_bdy_zero_n_mps;
                assert!(
                    (velocity - expected).norm() <= 1e-12 * (1.0 + expected.norm()),
                    "n = {n}, x = {x}: {velocity} != {expected}"
                );
            }
        }
    }

    fn constant_record(mid_s: f64, rad_s: f64, position: Vector3<f64>) -> ChebyEphemRecord {
        ChebyEphemRecord {
            ephem_time_mid_s: mid_s,
            ephem_time_rad_s: rad_s,
            pos_cheby_coeff: vec![position, Vector3::new(1.0, 0.0, 0.0)],
        }
    }

    // Not upstream: the first record containing the time wins, record 0 is
    // used outside every record, and the state rails at the interval ends.
    #[test]
    fn selects_the_first_record_containing_the_ephemeris_time() {
        let clock = Output::new(TdbVehicleClockCorrelationMsg {
            ephemeris_time_s: 1000.0,
            vehicle_clock_time_s: 0.0,
        });
        let mut module = ChebyPosEphem::new(ChebyPosEphemConfig::default());
        module.config.eph_array[0] = constant_record(1000.0, 100.0, Vector3::new(10.0, 0.0, 0.0));
        module.config.eph_array[1] = constant_record(1150.0, 100.0, Vector3::new(20.0, 0.0, 0.0));
        module.config.eph_array[2] = constant_record(1400.0, 100.0, Vector3::new(30.0, 0.0, 0.0));
        clock.connect_to(&mut module.clock_corr_in_msg);
        module.init();

        // (vehicle time [s], selected record, scaled time)
        for (time_s, selector, scaled_time) in [
            (0.0, 0, 0.0),
            (75.0, 0, 0.75),
            (100.0, 0, 1.0),
            (125.0, 1, -0.25),
            (350.0, 2, -0.5),
            (500.0, 2, 1.0),
            // Beyond every interval: record 0, railed to its end.
            (800.0, 0, 1.0),
        ] {
            module.update(&context((time_s * 1.0e9) as u64));
            let expected = module.config.eph_array[selector].pos_cheby_coeff[0].x + scaled_time;
            let output = module.pos_fit_out_msg.read();
            assert_eq!(module.coeff_selector, selector, "t = {time_s}");
            assert!(
                (output.position_bdy_zero_n_m.x - expected).abs() < 1e-12,
                "t = {time_s}: {} != {expected}",
                output.position_bdy_zero_n_m.x
            );
            assert_eq!(output.velocity_bdy_zero_n_mps.x, 1.0 / 100.0);
            assert!((output.time_tag_s - time_s).abs() < 1e-12);
        }
    }

    // Not upstream: reset rebuilds the velocity series from the coefficients
    // in the config at that time.
    #[test]
    fn reset_rebuilds_the_velocity_series_from_the_config() {
        let (mut module, _clock) =
            fitted_module(constant_record(500.0, 50.0, Vector3::new(1.0, 2.0, 3.0)));
        module.init();
        module.update(&context(0));
        assert_eq!(
            module.pos_fit_out_msg.read().velocity_bdy_zero_n_mps,
            Vector3::new(1.0 / 50.0, 0.0, 0.0)
        );

        module.config.eph_array[0].pos_cheby_coeff[1] = Vector3::new(0.0, -5.0, 0.0);
        module.update(&context(0));
        // Until the next reset, upstream keeps the old velocity series.
        assert_eq!(
            module.pos_fit_out_msg.read().velocity_bdy_zero_n_mps,
            Vector3::new(1.0 / 50.0, 0.0, 0.0)
        );
        module.reset(&context(0));
        module.update(&context(0));
        assert_eq!(
            module.pos_fit_out_msg.read().velocity_bdy_zero_n_mps,
            Vector3::new(0.0, -5.0 * (1.0 / 50.0), 0.0)
        );
    }

    // Not upstream: upstream's reset never writes the last velocity slot of
    // each axis for n >= 3. After a reset with n = 4 and x = T_3, the x
    // velocity series is [3, 0, 6, 0] / rad. A second reset with n = 3 and a
    // zero position leaves the stale 6 / rad in slot 2 and divides it by rad
    // again, so the velocity is (6 / rad^2) T_2 although the position is zero.
    #[test]
    fn a_reset_with_fewer_coefficients_keeps_upstreams_stale_velocity_slot() {
        let radius_s = 2.0;
        let mut t3 = vec![Vector3::zeros(); 4];
        t3[3] = Vector3::new(1.0, 0.0, 0.0);
        let (mut module, _clock) = fitted_module(ChebyEphemRecord {
            ephem_time_mid_s: 500.0,
            ephem_time_rad_s: radius_s,
            pos_cheby_coeff: t3,
        });
        module.init();
        // Vehicle time 2 s is the interval mid-point (scaled time 0), where
        // d/dt T_3 = T_3'(0) / rad = -3 / rad.
        module.update(&context(2_000_000_000));
        assert_eq!(
            module.pos_fit_out_msg.read().velocity_bdy_zero_n_mps,
            Vector3::new(-3.0 / radius_s, 0.0, 0.0)
        );

        module.config.eph_array[0].pos_cheby_coeff = vec![Vector3::zeros(); 3];
        module.reset(&context(2_000_000_000));
        for (time_nanos, t2) in [(0, 1.0), (2_000_000_000, -1.0)] {
            module.update(&context(time_nanos));
            let output = module.pos_fit_out_msg.read();
            assert_eq!(output.position_bdy_zero_n_m, Vector3::zeros());
            assert_eq!(
                output.velocity_bdy_zero_n_mps,
                Vector3::new(6.0 / (radius_s * radius_s) * t2, 0.0, 0.0)
            );
        }
    }

    // Not upstream: with no records loaded, upstream evaluates its zeroed
    // buffers at a NaN scaled time, which gives a zero state.
    #[test]
    fn an_unconfigured_module_publishes_a_zero_state() {
        let mut module = ChebyPosEphem::new(ChebyPosEphemConfig::default());
        module.init();
        module.update(&context(2_500_000_000));
        let output = module.pos_fit_out_msg.read();
        assert_eq!(output.position_bdy_zero_n_m, Vector3::zeros());
        assert_eq!(output.velocity_bdy_zero_n_mps, Vector3::zeros());
        assert_eq!(output.time_tag_s, 2.5);
    }
}
