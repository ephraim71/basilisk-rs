//! Orbit-element Chebyshev ephemeris: port of upstream Basilisk
//! `oeStateEphem`.
//!
//! The user loads up to [`MAX_OE_RECORDS`] records, each holding Chebyshev
//! series for the periapsis radius, eccentricity, inclination, argument of
//! periapsis, right ascension of the ascending node and an anomaly angle over
//! a time interval. Every update converts the vehicle clock to ephemeris time
//! (TDB) with the clock correlation message, picks the record whose interval
//! mid-point is nearest to that time, scales the time to `[-1, 1]` within the
//! interval (railing at its ends), evaluates the element series and converts
//! the elements to inertial position and velocity with `elem2rv`.
//!
//! Upstream stores each record's coefficients in fixed buffers of at most 20
//! terms with one shared count, `nChebCoeff`. Here every element series is a
//! `Vec` whose length is its term count; a zeroed upstream buffer slot and a
//! missing `Vec` entry both contribute nothing.

use crate::messages::{EphemerisMsg, Input, Output, TdbVehicleClockCorrelationMsg};
use crate::time::NANO2SEC;
use crate::utilities::orbital_motion::{
    ClassicElements, eccentric_to_true_anomaly, elements_to_state, hyperbolic_to_true_anomaly,
    mean_hyperbolic_to_hyperbolic_anomaly, mean_to_eccentric_anomaly,
};
use crate::{Module, SimulationContext};

use super::ephemeris_utilities::{calculate_cheby_value, ephemeris_time_s, scaled_fit_time};

/// Upstream `MAX_OE_RECORDS`: number of fit records the module holds.
pub const MAX_OE_RECORDS: usize = 10;

/// Upstream eccentricity tolerance below which `|e - 1|` counts as parabolic.
const PARABOLIC_ECCENTRICITY_TOLERANCE: f64 = 1e-12;

/// Which anomaly angle a record's anomaly series fits: upstream `anomalyFlag`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub enum AnomalyType {
    /// True anomaly; upstream `0`, the default.
    #[default]
    True = 0,
    /// Mean anomaly: elliptic `M` for `e < 1`, hyperbolic `N` otherwise.
    /// Upstream treats every nonzero flag this way.
    Mean = 1,
}

/// One Chebyshev fit of a body's orbit elements: upstream `ChebyOERecord`.
///
/// The default record is upstream's zeroed one: empty series centred on
/// ephemeris time zero with a zero radius.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ChebyOeRecord {
    /// Upstream `ephemTimeMid` \[s\]: ephemeris time (TDB) at the middle of the
    /// fit interval.
    pub ephem_time_mid_s: f64,
    /// Upstream `ephemTimeRad` \[s\]: half the length of the fit interval.
    pub ephem_time_rad_s: f64,
    /// Upstream `rPeriapCoeff` \[m\]: periapsis radius series, in the length
    /// unit of `mu_central_m3ps2`.
    pub r_periap_coeff_m: Vec<f64>,
    /// Upstream `eccCoeff` \[-\]: eccentricity series.
    pub ecc_coeff: Vec<f64>,
    /// Upstream `incCoeff` \[rad\]: inclination series.
    pub inc_coeff_rad: Vec<f64>,
    /// Upstream `argPerCoeff` \[rad\]: argument of periapsis series.
    pub arg_per_coeff_rad: Vec<f64>,
    /// Upstream `RAANCoeff` \[rad\]: right ascension of the ascending node
    /// series.
    pub raan_coeff_rad: Vec<f64>,
    /// Upstream `anomCoeff` \[rad\]: anomaly series, of the kind given by
    /// `anomaly_flag`.
    pub anom_coeff_rad: Vec<f64>,
    /// Upstream `anomalyFlag`: which anomaly `anom_coeff_rad` fits.
    pub anomaly_flag: AnomalyType,
}

/// User-settable parameters of [`OeStateEphem`].
#[derive(Clone, Debug)]
pub struct OeStateEphemConfig {
    /// Module name (upstream `ModelTag`).
    pub name: String,
    /// Upstream `muCentral` \[m^3/s^2\]: gravitational parameter of the body
    /// the elements are relative to.
    pub mu_central_m3ps2: f64,
    /// Upstream `ephArray`: the fit records.
    pub eph_array: [ChebyOeRecord; MAX_OE_RECORDS],
}

impl Default for OeStateEphemConfig {
    fn default() -> Self {
        Self {
            name: "oeStateEphem".to_string(),
            mu_central_m3ps2: 0.0,
            eph_array: Default::default(),
        }
    }
}

/// Orbit-element Chebyshev ephemeris: upstream `oeStateEphem`.
#[derive(Clone, Debug)]
pub struct OeStateEphem {
    /// User-settable parameters, read on every update.
    pub config: OeStateEphemConfig,
    /// Upstream `clockCorrInMsg`: vehicle clock to TDB correlation.
    pub clock_corr_in_msg: Input<TdbVehicleClockCorrelationMsg>,
    /// Upstream `stateFitOutMsg`: fitted position and velocity; the attitude
    /// fields stay zero.
    pub state_fit_out_msg: Output<EphemerisMsg>,
    /// Upstream `coeffSelector`: index of the record used by the last update.
    coeff_selector: usize,
}

impl OeStateEphem {
    /// Creates the module with `config` and unconnected ports.
    pub fn new(config: OeStateEphemConfig) -> Self {
        Self {
            config,
            clock_corr_in_msg: Input::default(),
            state_fit_out_msg: Output::default(),
            coeff_selector: 0,
        }
    }

    /// Upstream `Reset_oeStateEphem`: only checks the clock input.
    fn check_connections(&self) {
        if !self.clock_corr_in_msg.is_connected() {
            log::error!("oeStateEphem.clock_corr_in_msg is not connected");
        }
    }

    /// Upstream `Update_oeStateEphem`: the fitted state at `call_time_nanos`.
    fn fitted_state(&mut self, call_time_nanos: u64) -> EphemerisMsg {
        let correlation = self.clock_corr_in_msg.read();
        let current_eph_time_s = ephemeris_time_s(call_time_nanos, &correlation);

        self.coeff_selector = nearest_record(&self.config.eph_array, current_eph_time_s);
        let record = &self.config.eph_array[self.coeff_selector];
        let scaled_time = scaled_fit_time(
            current_eph_time_s,
            record.ephem_time_mid_s,
            record.ephem_time_rad_s,
        );
        let elements = fitted_elements(record, scaled_time);
        let (position, velocity) = elements_to_state(self.config.mu_central_m3ps2, &elements);
        EphemerisMsg {
            position_bdy_zero_n_m: position,
            velocity_bdy_zero_n_mps: velocity,
            time_tag_s: call_time_nanos as f64 * NANO2SEC,
            ..Default::default()
        }
    }
}

/// Index of the record whose mid-point is nearest to `ephemeris_time_s`.
/// Every record takes part, including unused zeroed ones (mid-point 0), and
/// the lowest index wins a tie, as upstream.
fn nearest_record(records: &[ChebyOeRecord], ephemeris_time_s: f64) -> usize {
    let distance = |record: &ChebyOeRecord| (ephemeris_time_s - record.ephem_time_mid_s).abs();
    let mut selector = 0;
    let mut smallest = distance(&records[0]);
    for (index, record) in records.iter().enumerate().skip(1) {
        let time_difference = distance(record);
        if time_difference < smallest {
            selector = index;
            smallest = time_difference;
        }
    }
    selector
}

/// The record's classical elements at `scaled_time`, with the anomaly
/// converted to the true anomaly and the semi-major axis derived from the
/// periapsis radius.
fn fitted_elements(record: &ChebyOeRecord, scaled_time: f64) -> ClassicElements {
    let series = |coefficients: &[f64]| calculate_cheby_value(coefficients, scaled_time);
    let periapsis_radius = series(&record.r_periap_coeff_m);
    let eccentricity = series(&record.ecc_coeff);
    let anomaly = series(&record.anom_coeff_rad);

    let true_anomaly_rad = match record.anomaly_flag {
        AnomalyType::True => anomaly,
        AnomalyType::Mean if eccentricity < 1.0 => eccentric_to_true_anomaly(
            mean_to_eccentric_anomaly(anomaly, eccentricity),
            eccentricity,
        ),
        AnomalyType::Mean => hyperbolic_to_true_anomaly(
            mean_hyperbolic_to_hyperbolic_anomaly(anomaly, eccentricity),
            eccentricity,
        ),
    };
    // elem2rv takes a = 0 as a parabola, sized by the periapsis radius.
    let semi_major_axis = if (eccentricity - 1.0).abs() > PARABOLIC_ECCENTRICITY_TOLERANCE {
        periapsis_radius / (1.0 - eccentricity)
    } else {
        0.0
    };

    ClassicElements {
        semi_major_axis,
        eccentricity,
        inclination_rad: series(&record.inc_coeff_rad),
        raan_rad: series(&record.raan_coeff_rad),
        arg_periapsis_rad: series(&record.arg_per_coeff_rad),
        true_anomaly_rad,
        periapsis_radius,
        ..Default::default()
    }
}

impl Module for OeStateEphem {
    fn init(&mut self) {
        self.state_fit_out_msg.write(EphemerisMsg::default());
        self.check_connections();
    }

    fn reset(&mut self, _context: &SimulationContext) {
        self.check_connections();
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.fitted_state(context.current_sim_nanos);
        self.state_fit_out_msg.write(state);
    }
}

#[cfg(test)]
mod tests {
    //! Upstream `test_oeStateEphem.py` fits the orbit of TDRS-12 read with
    //! SPICE from `TDRSS.bsp`, whose segments are SPK type 10 (two-line
    //! elements, evaluated with SGP4/SDP4). Neither ANISE nor this crate
    //! evaluates type 10, so the port fits a two-body orbit with TDRS-like
    //! (near-geosynchronous) elements instead, sampled at the same ephemeris
    //! times. The rest is upstream's: the states go through `rv2elem`, the
    //! anomaly is unwrapped the same way, every element is fitted with 15
    //! coefficients over four days, and all four parametrizations and
    //! tolerances are kept. The plots and TeX snippets are not ported.

    use std::f64::consts::PI;

    use hifitime::Epoch;
    use nalgebra::{SVector, Vector3};

    use super::{AnomalyType, ChebyOeRecord, MAX_OE_RECORDS, OeStateEphem, OeStateEphemConfig};
    use crate::fsw_algorithms::trans_determination::ephemeris_utilities::test_support::{
        chebfit, linspace,
    };
    use crate::messages::{EphemerisMsg, Output, TdbVehicleClockCorrelationMsg};
    use crate::utilities::astro_constants::D2R;
    use crate::utilities::orbital_motion::{
        ClassicElements, eccentric_to_mean_anomaly, eccentric_to_true_anomaly, elements_to_state,
        mean_to_eccentric_anomaly, state_to_elements, true_to_eccentric_anomaly,
    };
    use crate::{Module, SimulationContext};

    /// Upstream `orbitPosAccuracy` \[m\].
    const ORBIT_POS_ACCURACY: f64 = 10000.0;
    /// Upstream `orbitVelAccuracy` \[m/s\].
    const ORBIT_VEL_ACCURACY: f64 = 1.0;
    /// Upstream `centralBodyMu` \[m^3/s^2\].
    const CENTRAL_BODY_MU: f64 = 3.98574405096e14;

    fn context(current_sim_nanos: u64) -> SimulationContext {
        SimulationContext {
            current_sim_nanos,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2025, 1, 1),
        }
    }

    /// Two-body stand-in for TDRS-12's SPICE ephemeris: position \[m\] and
    /// velocity \[m/s\] relative to the Earth, inertial components, at
    /// `seconds_since_start` past the fit start.
    fn tdrs_like_state(seconds_since_start: f64) -> (Vector3<f64>, Vector3<f64>) {
        let semi_major_axis_m: f64 = 42_164.6e3;
        let eccentricity = 2.5e-4;
        let initial_mean_anomaly_rad = 200.0 * D2R;
        let mean_motion = (CENTRAL_BODY_MU / semi_major_axis_m.powi(3)).sqrt();
        let mean_anomaly =
            (initial_mean_anomaly_rad + mean_motion * seconds_since_start).rem_euclid(2.0 * PI);
        let eccentric_anomaly = mean_to_eccentric_anomaly(mean_anomaly, eccentricity);
        let elements = ClassicElements {
            semi_major_axis: semi_major_axis_m,
            eccentricity,
            inclination_rad: 5.5 * D2R,
            raan_rad: 75.0 * D2R,
            arg_periapsis_rad: 120.0 * D2R,
            true_anomaly_rad: eccentric_to_true_anomaly(eccentric_anomaly, eccentricity),
            ..Default::default()
        };
        elements_to_state(CENTRAL_BODY_MU, &elements)
    }

    /// Upstream `chebyPosFitAllTest(validChebyCurveTime, anomFlag)`, with
    /// `anomFlag = -1` (leave the record's default) as `None`.
    fn upstream_cheby_pos_fit_all_test(
        valid_cheby_curve_time: bool,
        anom_flag: Option<AnomalyType>,
    ) {
        let num_curve_points = 4 * 8640 + 1;
        let curve_duration_seconds: u64 = 4 * 86400;
        let log_period = curve_duration_seconds / (num_curve_points as u64 - 1);
        let deg_cheb_coeff = 14;
        // SPICE str2et of "2015 April 10, 00:00:00.0 TDB": 5577.5 days after
        // J2000 (2000-01-01 12:00 TDB).
        let et_start = 5577.5 * 86400.0;
        let et_end = et_start + curve_duration_seconds as f64;

        let time_history = linspace(et_start, et_end, num_curve_points);
        let mut tdrss_pos_list = Vec::with_capacity(num_curve_points);
        let mut tdrss_vel_list = Vec::with_capacity(num_curve_points);
        // [rPeriap, e, i, Omega, omega, anomaly] per sample.
        let mut element_list: Vec<SVector<f64, 6>> = Vec::with_capacity(num_curve_points);
        let mut anom_prev = 0.0;
        let mut anom_count = 0.0;
        for &time in &time_history {
            let (position, velocity) = tdrs_like_state(time - et_start);
            let orb_el = state_to_elements(CENTRAL_BODY_MU, position, velocity);
            let current_anom = if anom_flag == Some(AnomalyType::Mean) {
                eccentric_to_mean_anomaly(
                    true_to_eccentric_anomaly(orb_el.true_anomaly_rad, orb_el.eccentricity),
                    orb_el.eccentricity,
                )
            } else {
                orb_el.true_anomaly_rad
            };
            if current_anom < anom_prev {
                anom_count += 1.0;
            }
            element_list.push(SVector::from([
                orb_el.periapsis_radius,
                orb_el.eccentricity,
                orb_el.inclination_rad,
                orb_el.raan_rad,
                orb_el.arg_periapsis_rad,
                2.0 * PI * anom_count + current_anom,
            ]));
            anom_prev = current_anom;
            tdrss_pos_list.push(position);
            tdrss_vel_list.push(velocity);
        }

        let fit_times = linspace(-1.0, 1.0, num_curve_points);
        let coefficients = chebfit(&fit_times, &element_list, deg_cheb_coeff);
        let element_series =
            |element: usize| -> Vec<f64> { coefficients.iter().map(|c| c[element]).collect() };

        let mut record = ChebyOeRecord {
            ephem_time_mid_s: et_start + curve_duration_seconds as f64 / 2.0,
            ephem_time_rad_s: curve_duration_seconds as f64 / 2.0,
            r_periap_coeff_m: element_series(0),
            ecc_coeff: element_series(1),
            inc_coeff_rad: element_series(2),
            raan_coeff_rad: element_series(3),
            arg_per_coeff_rad: element_series(4),
            anom_coeff_rad: element_series(5),
            ..Default::default()
        };
        if let Some(flag) = anom_flag {
            record.anomaly_flag = flag;
        }
        let clock = Output::new(TdbVehicleClockCorrelationMsg {
            vehicle_clock_time_s: 0.0,
            ephemeris_time_s: record.ephem_time_mid_s - record.ephem_time_rad_s,
        });
        let mut module = OeStateEphem::new(OeStateEphemConfig::default());
        module.config.mu_central_m3ps2 = CENTRAL_BODY_MU;
        module.config.eph_array[0] = record;
        clock.connect_to(&mut module.clock_corr_in_msg);

        let stop_seconds = if valid_cheby_curve_time {
            curve_duration_seconds
        } else {
            // One logging period past the end of the fit interval.
            curve_duration_seconds + log_period
        };
        let task_nanos = log_period * 1_000_000_000;
        module.init();
        let samples: Vec<EphemerisMsg> = (0..=stop_seconds * 1_000_000_000 / task_nanos)
            .map(|step| {
                module.update(&context(step * task_nanos));
                module.state_fit_out_msg.read()
            })
            .collect();

        if valid_cheby_curve_time {
            let positions: Vec<_> = samples.iter().map(|s| s.position_bdy_zero_n_m).collect();
            let velocities: Vec<_> = samples.iter().map(|s| s.velocity_bdy_zero_n_mps).collect();
            let position_error = upstream_max_error(&positions, &tdrss_pos_list);
            let velocity_error = upstream_max_error(&velocities, &tdrss_vel_list);
            assert!(
                position_error < ORBIT_POS_ACCURACY,
                "{anom_flag:?}: TDRSS orbit accuracy {position_error}"
            );
            assert!(
                velocity_error < ORBIT_VEL_ACCURACY,
                "{anom_flag:?}: TDRSS velocity accuracy {velocity_error}"
            );
        } else {
            // Upstream compares the last two samples less the same truth row
            // with numpy.array_equal: the state must rail exactly.
            let last_log_idx = ((curve_duration_seconds + log_period) / log_period - 1) as usize;
            let last = &samples[last_log_idx];
            let past_end = &samples[last_log_idx + 1];
            assert_eq!(
                past_end.position_bdy_zero_n_m - tdrss_pos_list[last_log_idx],
                last.position_bdy_zero_n_m - tdrss_pos_list[last_log_idx],
                "Chebyshev position must rail high or low"
            );
            assert_eq!(
                past_end.velocity_bdy_zero_n_mps - tdrss_vel_list[last_log_idx],
                last.velocity_bdy_zero_n_mps - tdrss_vel_list[last_log_idx],
                "Chebyshev velocity must rail high or low"
            );
        }
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

    /// Upstream `test_chebyPosFitAllTest` parametrizations
    /// `(validChebyCurveTime, anomFlag)`: `(True, 0)`, `(True, 1)`,
    /// `(True, -1)` and `(False, -1)`.
    #[test]
    fn matches_upstream_cheby_pos_fit_all_test() {
        for (valid_cheby_curve_time, anom_flag) in [
            (true, Some(AnomalyType::True)),
            (true, Some(AnomalyType::Mean)),
            (true, None),
            (false, None),
        ] {
            upstream_cheby_pos_fit_all_test(valid_cheby_curve_time, anom_flag);
        }
    }

    /// A record whose every element is constant (a one-term series).
    fn constant_record(
        mid_s: f64,
        periapsis_radius_m: f64,
        eccentricity: f64,
        anomaly_rad: f64,
        anomaly_flag: AnomalyType,
    ) -> ChebyOeRecord {
        ChebyOeRecord {
            ephem_time_mid_s: mid_s,
            ephem_time_rad_s: 100.0,
            r_periap_coeff_m: vec![periapsis_radius_m],
            ecc_coeff: vec![eccentricity],
            inc_coeff_rad: vec![0.3],
            raan_coeff_rad: vec![1.1],
            arg_per_coeff_rad: vec![0.4],
            anom_coeff_rad: vec![anomaly_rad],
            anomaly_flag,
        }
    }

    fn single_record_output(record: ChebyOeRecord) -> EphemerisMsg {
        let clock = Output::new(TdbVehicleClockCorrelationMsg {
            ephemeris_time_s: record.ephem_time_mid_s,
            vehicle_clock_time_s: 0.0,
        });
        let mut module = OeStateEphem::new(OeStateEphemConfig::default());
        module.config.mu_central_m3ps2 = CENTRAL_BODY_MU;
        // Every other record sits far from the evaluation time.
        for (index, other) in module.config.eph_array.iter_mut().enumerate() {
            other.ephem_time_mid_s = 1.0e9 + index as f64;
        }
        module.config.eph_array[0] = record;
        clock.connect_to(&mut module.clock_corr_in_msg);
        module.init();
        module.update(&context(0));
        module.state_fit_out_msg.read()
    }

    // Not upstream: a mean elliptic anomaly is solved through Kepler's
    // equation. Truth: r = a (1 - e cos E) with E from an independent Newton
    // solve of M = E - e sin E.
    #[test]
    fn converts_a_mean_elliptic_anomaly() {
        let (rp, e, mean_anomaly) = (7.0e6, 0.3, 2.0);
        let output = single_record_output(constant_record(
            5.0e8,
            rp,
            e,
            mean_anomaly,
            AnomalyType::Mean,
        ));
        let mut ecc_anomaly: f64 = mean_anomaly;
        for _ in 0..50 {
            ecc_anomaly -= (ecc_anomaly - e * ecc_anomaly.sin() - mean_anomaly)
                / (1.0 - e * ecc_anomaly.cos());
        }
        let a = rp / (1.0 - e);
        let expected_radius = a * (1.0 - e * ecc_anomaly.cos());
        let radius = output.position_bdy_zero_n_m.norm();
        assert!(
            (radius - expected_radius).abs() < 1e-6,
            "{radius} != {expected_radius}"
        );
        // Vis-viva.
        let expected_speed = (CENTRAL_BODY_MU * (2.0 / expected_radius - 1.0 / a)).sqrt();
        let speed = output.velocity_bdy_zero_n_mps.norm();
        assert!(
            (speed - expected_speed).abs() < 1e-9,
            "{speed} != {expected_speed}"
        );
    }

    // Not upstream: for e >= 1 the mean anomaly is the hyperbolic one,
    // N = e sinh H - H, and r = a (1 - e cosh H) with a < 0.
    #[test]
    fn converts_a_mean_hyperbolic_anomaly() {
        let (rp, e, mean_anomaly) = (7.0e6, 1.8, -1.5);
        let output = single_record_output(constant_record(
            5.0e8,
            rp,
            e,
            mean_anomaly,
            AnomalyType::Mean,
        ));
        let mut hyp_anomaly: f64 = mean_anomaly;
        for _ in 0..50 {
            hyp_anomaly -= (e * hyp_anomaly.sinh() - hyp_anomaly - mean_anomaly)
                / (e * hyp_anomaly.cosh() - 1.0);
        }
        let a = rp / (1.0 - e);
        let expected_radius = a * (1.0 - e * hyp_anomaly.cosh());
        let radius = output.position_bdy_zero_n_m.norm();
        assert!(
            (radius - expected_radius).abs() < 1e-6,
            "{radius} != {expected_radius}"
        );
        let expected_speed = (CENTRAL_BODY_MU * (2.0 / expected_radius - 1.0 / a)).sqrt();
        let speed = output.velocity_bdy_zero_n_mps.norm();
        assert!(
            (speed - expected_speed).abs() < 1e-9,
            "{speed} != {expected_speed}"
        );
    }

    // Not upstream: e = 1 is a parabola sized by the periapsis radius; at
    // true anomaly 0 the body sits at periapsis with the escape speed.
    #[test]
    fn treats_unit_eccentricity_as_a_parabola() {
        let rp = 7.0e6;
        let output = single_record_output(constant_record(5.0e8, rp, 1.0, 0.0, AnomalyType::True));
        let radius = output.position_bdy_zero_n_m.norm();
        assert!((radius - rp).abs() < 1e-6, "{radius} != {rp}");
        let speed = output.velocity_bdy_zero_n_mps.norm();
        let escape_speed = (2.0 * CENTRAL_BODY_MU / rp).sqrt();
        assert!(
            (speed - escape_speed).abs() < 1e-9,
            "{speed} != {escape_speed}"
        );
    }

    // Not upstream: the nearest mid-point wins, the lower index wins a tie,
    // and unused zeroed records (mid-point 0) take part in the search.
    #[test]
    fn selects_the_record_with_the_nearest_mid_point() {
        let clock = Output::new(TdbVehicleClockCorrelationMsg::default());
        let mut module = OeStateEphem::new(OeStateEphemConfig::default());
        module.config.mu_central_m3ps2 = CENTRAL_BODY_MU;
        for (index, mid_s) in [(0, 1000.0), (1, 2000.0), (2, 3000.0), (3, 2000.0)] {
            module.config.eph_array[index] =
                constant_record(mid_s, 7.0e6, 0.1, 0.5, AnomalyType::True);
        }
        clock.connect_to(&mut module.clock_corr_in_msg);
        module.init();

        // (ephemeris time [s], selected record)
        for (time_s, selector) in [
            (1100.0, 0),
            (1600.0, 1),
            // Records 1 and 3 share a mid-point: the first one wins.
            (2100.0, 1),
            (2600.0, 2),
            (9000.0, 2),
            // Records 4 and up are unused, with mid-point 0.
            (400.0, 4),
        ] {
            clock.write(TdbVehicleClockCorrelationMsg {
                ephemeris_time_s: time_s,
                vehicle_clock_time_s: 0.0,
            });
            module.update(&context(0));
            assert_eq!(module.coeff_selector, selector, "t = {time_s}");
        }
        assert_eq!(module.config.eph_array.len(), MAX_OE_RECORDS);
    }

    // Not upstream: config set after construction is read on the first
    // update, and the time tag is the vehicle time.
    #[test]
    fn reads_the_config_set_before_init() {
        let mut module = OeStateEphem::new(OeStateEphemConfig::default());
        let clock = Output::new(TdbVehicleClockCorrelationMsg {
            ephemeris_time_s: 5.0e8,
            vehicle_clock_time_s: 0.0,
        });
        clock.connect_to(&mut module.clock_corr_in_msg);
        module.config.mu_central_m3ps2 = CENTRAL_BODY_MU;
        module.config.eph_array[0] = constant_record(5.0e8, 7.0e6, 0.0, 0.0, AnomalyType::True);
        module.init();
        module.update(&context(0));
        let output = module.state_fit_out_msg.read();
        assert!((output.position_bdy_zero_n_m.norm() - 7.0e6).abs() < 1e-6);
        assert_eq!(output.time_tag_s, 0.0);
    }
}
