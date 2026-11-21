//! Two-line element sets (TLE): port of upstream Basilisk
//! `utilities/tleHandling.py`.
//!
//! A TLE holds SGP4 mean elements in the TEME frame. Reading one
//! ([`sat_tle_to_elements`], upstream `satTle2elem`) propagates those elements
//! with SGP4 to the TLE epoch, rotates the TEME state to the GCRF (the
//! inertial frame of Basilisk and of SPICE's `J2000`) and returns the
//! osculating classical elements about the Earth, with the set's metadata, as a
//! [`TleData`]. Writing one ([`generate_tle`], upstream `generateTle`) runs the
//! inverse: it iterates SGP4 mean elements until SGP4 reproduces the osculating
//! position, and formats them.
//!
//! | upstream | here |
//! |---|---|
//! | `TleData` | [`TleData`] |
//! | `satTle2elem(path, on_error=...)` | [`sat_tle_to_elements`], [`parse_tle_text`] with [`OnError`] |
//! | `generateTle(tleData)` | [`generate_tle`] |
//! | `_teme2j2000`, `nutation_iau1980`, `frame_bias` | [`teme_to_gcrf`] |
//! | `datetime_to_jd` | [`UtcDateTime::julian_date`] |
//! | a `datetime` | [`UtcDateTime`] |
//! | `sgp4.api.Satrec.twoline2rv`, `Satrec.sgp4` | the [`sgp4`] crate (WGS72, improved mode) |
//!
//! Upstream propagates with the Python `sgp4` package (Vallado's C++ SGP4);
//! here the pure-Rust [`sgp4`] crate runs the same theory. Its defaults are
//! WGS84; upstream's `Satrec.twoline2rv` uses WGS72 in the "improved"
//! operation mode, so the propagator is built from [`sgp4::WGS72`] and the IAU
//! sidereal time explicitly. The TLE lines are parsed here, as upstream parses
//! them, and the crate only propagates (its own parser rejects sets upstream
//! accepts, such as a blank ephemeris-type column).
//!
//! Upstream prints warnings for the lines and sets it skips; here they go to
//! [`log::warn!`]. Errors that upstream raises as `ValueError` are
//! [`TleError`]s whose messages keep upstream's wording.

use std::f64::consts::PI;
use std::fmt;
use std::path::Path;

use nalgebra::{Matrix3, Vector3};

use super::astro_constants::{J2_EARTH, MU_EARTH, REQ_EARTH};
use super::orbital_motion::{
    ClassicElements, MeanOscDirection, eccentric_to_mean_anomaly, eccentric_to_true_anomaly,
    elements_to_state, mean_osculating_map, mean_to_eccentric_anomaly, state_to_elements,
    true_to_eccentric_anomaly,
};

/// Year of the first catalogued satellite (Sputnik 1): two-digit TLE years
/// from 57 on are 19xx (upstream `SPUTNIK_LAUNCHDATE.year`).
const SPUTNIK_LAUNCH_YEAR: i32 = 1957;
/// Upstream `ARCSEC2RAD`.
const ARCSEC2RAD: f64 = PI / 180.0 / 3600.0;
/// Upstream `DELTA_AT`: TAI - UTC leap seconds, fixed at 37 s.
const DELTA_AT: f64 = 37.0;
/// Upstream `DAYS_YEAR`, `AVG_DAYS_MONTH` and `EPOCH_ALLIGNMENT_CONST` of the
/// Julian date formula.
const DAYS_YEAR: f64 = 365.25;
const AVG_DAYS_MONTH: f64 = 30.6001;
const EPOCH_ALIGNMENT_CONST: f64 = 1524.5;
/// Upstream `SEC_PER_DAY`.
const SEC_PER_DAY: f64 = 86_400.0;
/// Upstream `_TLE_MAX_NORAD_ID`: the 5-character catalog field.
const TLE_MAX_NORAD_ID: u64 = 99_999;
/// Upstream `_osculating2mean_sgp4` iteration: position tolerance \[m\],
/// iteration limit, stall limit and the TLE quantization floor \[m\].
const MEAN_ELEMENT_TOLERANCE_M: f64 = 0.1;
const MEAN_ELEMENT_MAX_ITERATIONS: usize = 50;
const MEAN_ELEMENT_STALL_LIMIT: usize = 5;
const TLE_PRECISION_FLOOR_M: f64 = 15.0;

/// A UTC calendar date and time to the microsecond: the stand-in for the
/// Python `datetime` upstream's TLE epochs and launch dates are. Leap seconds
/// are not represented, as in `datetime`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct UtcDateTime {
    pub year: i32,
    /// 1 to 12.
    pub month: u32,
    /// 1 to 31.
    pub day: u32,
    pub hour: u32,
    pub minute: u32,
    pub second: u32,
    pub microsecond: u32,
}

impl UtcDateTime {
    pub const fn new(
        year: i32,
        month: u32,
        day: u32,
        hour: u32,
        minute: u32,
        second: u32,
        microsecond: u32,
    ) -> Self {
        Self {
            year,
            month,
            day,
            hour,
            minute,
            second,
            microsecond,
        }
    }

    /// Midnight at the start of the given day.
    pub const fn midnight(year: i32, month: u32, day: u32) -> Self {
        Self::new(year, month, day, 0, 0, 0, 0)
    }

    /// `datetime(year, 1, 1) + timedelta(days=day_of_year - 1)`: day 1.5 is
    /// noon on 1 January. The fraction is rounded to the microsecond as
    /// `timedelta` rounds it.
    pub fn from_day_of_year(year: i32, day_of_year: f64) -> Self {
        let days = day_of_year - 1.0;
        let whole_days = days.trunc();
        let seconds = (days - whole_days) * SEC_PER_DAY;
        let whole_seconds = seconds.trunc();
        let microseconds = ((seconds - whole_seconds) * 1e6).round_ties_even();
        Self::midnight(year, 1, 1).plus_microseconds(
            whole_days as i64 * 86_400_000_000
                + whole_seconds as i64 * 1_000_000
                + microseconds as i64,
        )
    }

    /// This time shifted by `microseconds` (negative for earlier).
    pub fn plus_microseconds(self, microseconds: i64) -> Self {
        let day_us = 86_400_000_000_i64;
        let total = self.microseconds_of_day() as i64 + microseconds;
        let days =
            days_from_civil(self.year as i64, self.month, self.day) + total.div_euclid(day_us);
        let of_day = total.rem_euclid(day_us);
        let (year, month, day) = civil_from_days(days);
        Self::new(
            year as i32,
            month,
            day,
            (of_day / 3_600_000_000) as u32,
            (of_day / 60_000_000 % 60) as u32,
            (of_day / 1_000_000 % 60) as u32,
            (of_day % 1_000_000) as u32,
        )
    }

    /// Day of the year, 1 on 1 January (`%j`).
    pub fn day_of_year(&self) -> u32 {
        (days_from_civil(self.year as i64, self.month, self.day)
            - days_from_civil(self.year as i64, 1, 1)
            + 1) as u32
    }

    /// Seconds since midnight, with the microseconds.
    pub fn seconds_of_day(&self) -> f64 {
        (self.hour * 3600 + self.minute * 60 + self.second) as f64 + self.microsecond as f64 / 1e6
    }

    fn microseconds_of_day(&self) -> u64 {
        (self.hour as u64 * 3600 + self.minute as u64 * 60 + self.second as u64) * 1_000_000
            + self.microsecond as u64
    }

    /// Upstream `datetime_to_jd`: the Julian date of this UTC time (the
    /// Meeus formula with the Gregorian correction).
    pub fn julian_date(&self) -> f64 {
        let mut year = self.year as f64;
        let mut month = self.month as f64;
        let day = self.day as f64
            + (self.hour as f64
                + self.minute as f64 / 60.0
                + self.second as f64 / 3600.0
                + self.microsecond as f64 / 3.6e9)
                / 24.0;
        if month <= 2.0 {
            year -= 1.0;
            month += 12.0;
        }
        let a = (year / 100.0).trunc();
        let b = 2.0 - a + (a / 4.0).trunc();
        (DAYS_YEAR * (year + 4716.0)).trunc() + (AVG_DAYS_MONTH * (month + 1.0)).trunc() + day + b
            - EPOCH_ALIGNMENT_CONST
    }
}

impl fmt::Display for UtcDateTime {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{:04}-{:02}-{:02} {:02}:{:02}:{:02}.{:06} UTC",
            self.year, self.month, self.day, self.hour, self.minute, self.second, self.microsecond
        )
    }
}

/// Days since 1970-01-01 of a proleptic Gregorian date.
fn days_from_civil(year: i64, month: u32, day: u32) -> i64 {
    let year = if month <= 2 { year - 1 } else { year };
    let era = year.div_euclid(400);
    let year_of_era = year - era * 400;
    let month_from_march = (month as i64 + 9) % 12;
    let day_of_year = (153 * month_from_march + 2) / 5 + day as i64 - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

/// The proleptic Gregorian date `days` after 1970-01-01.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let days = days + 719_468;
    let era = days.div_euclid(146_097);
    let day_of_era = days - era * 146_097;
    let year_of_era =
        (day_of_era - day_of_era / 1460 + day_of_era / 36_524 - day_of_era / 146_096) / 365;
    let day_of_year = day_of_era - (365 * year_of_era + year_of_era / 4 - year_of_era / 100);
    let month_from_march = (5 * day_of_year + 2) / 153;
    let day = (day_of_year - (153 * month_from_march + 2) / 5 + 1) as u32;
    let month = if month_from_march < 10 {
        month_from_march + 3
    } else {
        month_from_march - 9
    } as u32;
    let year = year_of_era + era * 400 + i64::from(month <= 2);
    (year, month, day)
}

/// One satellite of a TLE file: its osculating elements and metadata
/// (upstream `TleData`).
#[derive(Clone, Debug, PartialEq)]
pub struct TleData {
    /// Upstream `oe`: osculating elements about the Earth in the GCRF \[m,
    /// rad\] when read; the elements to write otherwise.
    pub oe: ClassicElements,
    /// Upstream `tleEpoch`: when the elements are valid.
    pub tle_epoch: UtcDateTime,
    /// Upstream `satName` (default `"BSK-Sat-00"`; empty for a 2LE record).
    pub sat_name: String,
    /// Upstream `noradID`: catalog number, `0..=99999` (default `"00000"`).
    pub norad_id: String,
    /// Upstream `classification`: `"Unclassified"`, `"Classified"`,
    /// `"Secret"` or `"Unknown"` (default).
    pub classification: String,
    /// Upstream `revAtEpoch`: revolution number at epoch.
    pub rev_at_epoch: f64,
    /// Upstream `propagator`: the ephemeris-type name (default `"0"`).
    pub propagator: String,
    /// Upstream `elemSetNo` (default 999).
    pub elem_set_no: u32,
    /// Upstream `nDot` \[rev/day²\].
    pub n_dot: f64,
    /// Upstream `nDotDot` \[rev/day³\].
    pub n_dot_dot: f64,
    /// Upstream `bStar` \[1/Earth radii\].
    pub b_star: f64,
    /// Upstream `launchDate` (1 January of the launch year when read);
    /// `None` when the international designator is blank.
    pub launch_date: Option<UtcDateTime>,
    /// Upstream `launchNo`: launch number of the year (default 1).
    pub launch_no: u32,
    /// Upstream `pol`: piece of the launch, up to 3 characters (default `"A"`).
    pub pol: String,
}

impl TleData {
    /// Upstream `TleData(oe=..., tleEpoch=...)` with the default metadata.
    pub fn new(oe: ClassicElements, tle_epoch: UtcDateTime) -> Self {
        Self {
            oe,
            tle_epoch,
            sat_name: "BSK-Sat-00".to_string(),
            norad_id: "00000".to_string(),
            classification: "Unknown".to_string(),
            rev_at_epoch: 0.0,
            propagator: "0".to_string(),
            elem_set_no: 999,
            n_dot: 0.0,
            n_dot_dot: 0.0,
            b_star: 0.0,
            launch_date: None,
            launch_no: 1,
            pol: "A".to_string(),
        }
    }
}

/// Upstream `satTle2elem(on_error=...)`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OnError {
    /// Stop at the first record error (upstream `"raise"`, the default).
    #[default]
    Raise,
    /// Warn about and skip each set with a record error, return the others
    /// (upstream `"skip"`); if every set fails, the first error is returned.
    Skip,
}

/// Why a TLE could not be read or written.
#[derive(Debug)]
pub enum TleError {
    /// The file could not be read.
    Io(std::io::Error),
    /// The file's layout is invalid (upstream's file-level `ValueError`s,
    /// raised whatever the [`OnError`] policy).
    Format(String),
    /// One element set is invalid or SGP4 failed on it (upstream's per-set
    /// `ValueError`s, skipped with [`OnError::Skip`]).
    Record(String),
}

impl fmt::Display for TleError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io(error) => write!(formatter, "cannot read the TLE file: {error}"),
            Self::Format(message) | Self::Record(message) => formatter.write_str(message),
        }
    }
}

impl std::error::Error for TleError {}

fn record_error(message: impl Into<String>) -> TleError {
    TleError::Record(message.into())
}

// ------------------------------------------------------------------------- //
// TEME -> GCRF (upstream `_teme2j2000`)

/// Upstream `rot(axis, angle)`: the frame rotation by `angle` about `axis`
/// (1, 2 or 3), `+sin` above the diagonal for axes 1 and 3.
fn rot(axis: u8, angle: f64) -> Matrix3<f64> {
    let (s, c) = angle.sin_cos();
    match axis {
        1 => Matrix3::new(1.0, 0.0, 0.0, 0.0, c, s, 0.0, -s, c),
        2 => Matrix3::new(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c),
        3 => Matrix3::new(c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0),
        _ => unreachable!("rotation axis must be 1, 2 or 3"),
    }
}

/// Upstream `frame_bias`: GCRS to mean equator and equinox of J2000 (IERS
/// Conventions 2010).
fn frame_bias() -> Matrix3<f64> {
    let d_alpha = -0.0146 * ARCSEC2RAD;
    let xi = -0.016617 * ARCSEC2RAD;
    let eta = -0.0068192 * ARCSEC2RAD;
    rot(1, -eta) * rot(2, xi) * rot(3, d_alpha)
}

/// IAU 1980 nutation series (IERS Technical Note 21): the multipliers of
/// `l, l', F, D, Ω` and `ψ₀, ψ_T, ε₀, ε_T` in 0.1 mas.
const NUTATION_TERMS: [([i8; 5], [f64; 4]); 106] = [
    ([0, 0, 0, 0, 1], [-171996.0, -174.2, 92025.0, 8.9]),
    ([0, 0, 0, 0, 2], [2062.0, 0.2, -895.0, 0.5]),
    ([-2, 0, 2, 0, 1], [46.0, 0.0, -24.0, 0.0]),
    ([2, 0, -2, 0, 0], [11.0, 0.0, 0.0, 0.0]),
    ([-2, 0, 2, 0, 2], [-3.0, 0.0, 1.0, 0.0]),
    ([1, -1, 0, -1, 0], [-3.0, 0.0, 0.0, 0.0]),
    ([0, -2, 2, -2, 1], [-2.0, 0.0, 1.0, 0.0]),
    ([2, 0, -2, 0, 1], [1.0, 0.0, 0.0, 0.0]),
    ([0, 0, 2, -2, 2], [-13187.0, -1.6, 5736.0, -3.1]),
    ([0, 1, 0, 0, 0], [1426.0, -3.4, 54.0, -0.1]),
    ([0, 1, 2, -2, 2], [-517.0, 1.2, 224.0, -0.6]),
    ([0, -1, 2, -2, 2], [217.0, -0.5, -95.0, 0.3]),
    ([0, 0, 2, -2, 1], [129.0, 0.1, -70.0, 0.0]),
    ([2, 0, 0, -2, 0], [48.0, 0.0, 1.0, 0.0]),
    ([0, 0, 2, -2, 0], [-22.0, 0.0, 0.0, 0.0]),
    ([0, 2, 0, 0, 0], [17.0, -0.1, 0.0, 0.0]),
    ([0, 1, 0, 0, 1], [-15.0, 0.0, 9.0, 0.0]),
    ([0, 2, 2, -2, 2], [-16.0, 0.1, 7.0, 0.0]),
    ([0, -1, 0, 0, 1], [-12.0, 0.0, 6.0, 0.0]),
    ([-2, 0, 0, 2, 1], [-6.0, 0.0, 3.0, 0.0]),
    ([0, -1, 2, -2, 1], [-5.0, 0.0, 3.0, 0.0]),
    ([2, 0, 0, -2, 1], [4.0, 0.0, -2.0, 0.0]),
    ([0, 1, 2, -2, 1], [4.0, 0.0, -2.0, 0.0]),
    ([1, 0, 0, -1, 0], [-4.0, 0.0, 0.0, 0.0]),
    ([2, 1, 0, -2, 0], [1.0, 0.0, 0.0, 0.0]),
    ([0, 0, -2, 2, 1], [1.0, 0.0, 0.0, 0.0]),
    ([0, 1, -2, 2, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 1, 0, 0, 2], [1.0, 0.0, 0.0, 0.0]),
    ([-1, 0, 0, 1, 1], [1.0, 0.0, 0.0, 0.0]),
    ([0, 1, 2, -2, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 0, 2, 0, 2], [-2274.0, -0.2, 977.0, -0.5]),
    ([1, 0, 0, 0, 0], [712.0, 0.1, -7.0, 0.0]),
    ([0, 0, 2, 0, 1], [-386.0, -0.4, 200.0, 0.0]),
    ([1, 0, 2, 0, 2], [-301.0, 0.0, 129.0, -0.1]),
    ([1, 0, 0, -2, 0], [-158.0, 0.0, -1.0, 0.0]),
    ([-1, 0, 2, 0, 2], [123.0, 0.0, -53.0, 0.0]),
    ([0, 0, 0, 2, 0], [63.0, 0.0, -2.0, 0.0]),
    ([1, 0, 0, 0, 1], [63.0, 0.1, -33.0, 0.0]),
    ([-1, 0, 0, 0, 1], [-58.0, -0.1, 32.0, 0.0]),
    ([-1, 0, 2, 2, 2], [-59.0, 0.0, 26.0, 0.0]),
    ([1, 0, 2, 0, 1], [-51.0, 0.0, 27.0, 0.0]),
    ([0, 0, 2, 2, 2], [-38.0, 0.0, 16.0, 0.0]),
    ([2, 0, 0, 0, 0], [29.0, 0.0, -1.0, 0.0]),
    ([1, 0, 2, -2, 2], [29.0, 0.0, -12.0, 0.0]),
    ([2, 0, 2, 0, 2], [-31.0, 0.0, 13.0, 0.0]),
    ([0, 0, 2, 0, 0], [26.0, 0.0, -1.0, 0.0]),
    ([-1, 0, 2, 0, 1], [21.0, 0.0, -10.0, 0.0]),
    ([-1, 0, 0, 2, 1], [16.0, 0.0, -8.0, 0.0]),
    ([1, 0, 0, -2, 1], [-13.0, 0.0, 7.0, 0.0]),
    ([-1, 0, 2, 2, 1], [-10.0, 0.0, 5.0, 0.0]),
    ([1, 1, 0, -2, 0], [-7.0, 0.0, 0.0, 0.0]),
    ([0, 1, 2, 0, 2], [7.0, 0.0, -3.0, 0.0]),
    ([0, -1, 2, 0, 2], [-7.0, 0.0, 3.0, 0.0]),
    ([1, 0, 2, 2, 2], [-8.0, 0.0, 3.0, 0.0]),
    ([1, 0, 0, 2, 0], [6.0, 0.0, 0.0, 0.0]),
    ([2, 0, 2, -2, 2], [6.0, 0.0, -3.0, 0.0]),
    ([0, 0, 0, 2, 1], [-6.0, 0.0, 3.0, 0.0]),
    ([0, 0, 2, 2, 1], [-7.0, 0.0, 3.0, 0.0]),
    ([1, 0, 2, -2, 1], [6.0, 0.0, -3.0, 0.0]),
    ([0, 0, 0, -2, 1], [-5.0, 0.0, 3.0, 0.0]),
    ([1, -1, 0, 0, 0], [5.0, 0.0, 0.0, 0.0]),
    ([2, 0, 2, 0, 1], [-5.0, 0.0, 3.0, 0.0]),
    ([0, 1, 0, -2, 0], [-4.0, 0.0, 0.0, 0.0]),
    ([1, 0, -2, 0, 0], [4.0, 0.0, 0.0, 0.0]),
    ([0, 0, 0, 1, 0], [-4.0, 0.0, 0.0, 0.0]),
    ([1, 1, 0, 0, 0], [-3.0, 0.0, 0.0, 0.0]),
    ([1, 0, 2, 0, 0], [3.0, 0.0, 0.0, 0.0]),
    ([1, -1, 2, 0, 2], [-3.0, 0.0, 1.0, 0.0]),
    ([-1, -1, 2, 2, 2], [-3.0, 0.0, 1.0, 0.0]),
    ([-2, 0, 0, 0, 1], [-2.0, 0.0, 1.0, 0.0]),
    ([3, 0, 2, 0, 2], [-3.0, 0.0, 1.0, 0.0]),
    ([0, -1, 2, 2, 2], [-3.0, 0.0, 1.0, 0.0]),
    ([1, 1, 2, 0, 2], [2.0, 0.0, -1.0, 0.0]),
    ([-1, 0, 2, -2, 1], [-2.0, 0.0, 1.0, 0.0]),
    ([2, 0, 0, 0, 1], [2.0, 0.0, -1.0, 0.0]),
    ([1, 0, 0, 0, 2], [-2.0, 0.0, 1.0, 0.0]),
    ([3, 0, 0, 0, 0], [2.0, 0.0, 0.0, 0.0]),
    ([0, 0, 2, 1, 2], [2.0, 0.0, -1.0, 0.0]),
    ([-1, 0, 0, 0, 2], [1.0, 0.0, -1.0, 0.0]),
    ([1, 0, 0, -4, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([-2, 0, 2, 2, 2], [1.0, 0.0, -1.0, 0.0]),
    ([-1, 0, 2, 4, 2], [-2.0, 0.0, 1.0, 0.0]),
    ([2, 0, 0, -4, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([1, 1, 2, -2, 2], [1.0, 0.0, -1.0, 0.0]),
    ([1, 0, 2, 2, 1], [-1.0, 0.0, 1.0, 0.0]),
    ([-2, 0, 2, 4, 2], [-1.0, 0.0, 1.0, 0.0]),
    ([-1, 0, 4, 0, 2], [1.0, 0.0, 0.0, 0.0]),
    ([1, -1, 0, -2, 0], [1.0, 0.0, 0.0, 0.0]),
    ([2, 0, 2, -2, 1], [1.0, 0.0, -1.0, 0.0]),
    ([2, 0, 2, 2, 2], [-1.0, 0.0, 0.0, 0.0]),
    ([1, 0, 0, 2, 1], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 0, 4, -2, 2], [1.0, 0.0, 0.0, 0.0]),
    ([3, 0, 2, -2, 2], [1.0, 0.0, 0.0, 0.0]),
    ([1, 0, 2, -2, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 1, 2, 0, 1], [1.0, 0.0, 0.0, 0.0]),
    ([-1, -1, 0, 2, 1], [1.0, 0.0, 0.0, 0.0]),
    ([0, 0, -2, 0, 1], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 0, 2, -1, 2], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 1, 0, 2, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([1, 0, -2, -2, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([0, -1, 2, 0, 1], [-1.0, 0.0, 0.0, 0.0]),
    ([1, 1, 0, -2, 1], [-1.0, 0.0, 0.0, 0.0]),
    ([1, 0, -2, 2, 0], [-1.0, 0.0, 0.0, 0.0]),
    ([2, 0, 0, 2, 0], [1.0, 0.0, 0.0, 0.0]),
    ([0, 0, 2, 4, 2], [-1.0, 0.0, 0.0, 0.0]),
    ([0, 1, 0, 1, 0], [1.0, 0.0, 0.0, 0.0]),
];

/// Upstream `nutation_iau1980(T)`: the nutation matrix, the nutation in
/// longitude and the mean obliquity at `T` Julian centuries (TT) since J2000.
fn nutation_iau1980(t: f64) -> (Matrix3<f64>, f64, f64) {
    let l = (134.96298139 + (1325.0 * 360.0 + 198.8673981) * t).to_radians();
    let lp = (357.52772333 + (99.0 * 360.0 + 359.0503400) * t).to_radians();
    let f = (93.27191028 + (1342.0 * 360.0 + 82.0175381) * t).to_radians();
    let d = (297.85036306 + (1236.0 * 360.0 + 307.1114800) * t).to_radians();
    let om = (125.04452222 - (5.0 * 360.0 + 134.1362608) * t).to_radians();

    let mut dpsi = 0.0;
    let mut deps = 0.0;
    for ([ml, mlp, mf, md, mom], [psi0, psi_t, eps0, eps_t]) in NUTATION_TERMS {
        let arg = f64::from(ml) * l
            + f64::from(mlp) * lp
            + f64::from(mf) * f
            + f64::from(md) * d
            + f64::from(mom) * om;
        dpsi += (psi0 + psi_t * t) * arg.sin();
        deps += (eps0 + eps_t * t) * arg.cos();
    }
    dpsi *= 1e-4 * ARCSEC2RAD;
    deps *= 1e-4 * ARCSEC2RAD;

    let mean_obliquity =
        (84381.448 - 46.8150 * t - 0.00059 * t.powi(2) + 0.001813 * t.powi(3)) * ARCSEC2RAD;
    let true_obliquity = mean_obliquity + deps;
    let nutation = rot(1, -true_obliquity) * rot(3, -dpsi) * rot(1, mean_obliquity);
    (nutation, dpsi, mean_obliquity)
}

/// Upstream `_teme2j2000`: a TEME state \[m, m/s\] at the UTC `epoch` in the
/// GCRF, through the equation of the equinoxes, IAU 1980 nutation, IAU 1976
/// precession and the inverse frame bias. TT is UTC + 37 s + 32.184 s.
pub fn teme_to_gcrf(
    r_teme: Vector3<f64>,
    v_teme: Vector3<f64>,
    epoch: &UtcDateTime,
) -> (Vector3<f64>, Vector3<f64>) {
    let jd_tt = epoch.julian_date() + (DELTA_AT + 32.184) / SEC_PER_DAY;
    let t = (jd_tt - 2_451_545.0) / 36_525.0;

    let zeta = (2306.2181 * t + 0.30188 * t.powi(2) + 0.017998 * t.powi(3)) * ARCSEC2RAD;
    let theta = (2004.3109 * t - 0.42665 * t.powi(2) - 0.041833 * t.powi(3)) * ARCSEC2RAD;
    let z = (2306.2181 * t + 1.09468 * t.powi(2) + 0.018203 * t.powi(3)) * ARCSEC2RAD;
    let precession = rot(3, -z) * rot(2, theta) * rot(3, -zeta);

    let (nutation, dpsi, mean_obliquity) = nutation_iau1980(t);
    let equation_of_equinoxes = rot(3, -(dpsi * mean_obliquity.cos()));

    let rotation = frame_bias().transpose()
        * precession.transpose()
        * nutation.transpose()
        * equation_of_equinoxes;
    (rotation * r_teme, rotation * v_teme)
}

// ------------------------------------------------------------------------- //
// SGP4 (upstream `Satrec.twoline2rv` and `Satrec.sgp4`)

/// Characters `start..end` of `line` (Python `line[start:end]`, clipped).
fn columns(line: &str, start: usize, end: usize) -> String {
    line.chars()
        .skip(start)
        .take(end.saturating_sub(start))
        .collect()
}

fn parse_float(text: &str, what: &str) -> Result<f64, TleError> {
    text.trim()
        .parse::<f64>()
        .map_err(|_| record_error(format!("could not convert {what} '{text}' to float")))
}

fn parse_int(text: &str, what: &str) -> Result<i64, TleError> {
    text.trim()
        .parse::<i64>()
        .map_err(|_| record_error(format!("invalid literal for {what}: '{text}'")))
}

/// Upstream `_tleStr2physVal`: a decimal-point-assumed mantissa with a
/// power-of-ten exponent (`" 56053-3"` is `0.56053e-3`).
fn tle_exponential_field(text: &str) -> Result<f64, TleError> {
    let sign = if text.starts_with('-') { -1.0 } else { 1.0 };
    let exponent_sign = if columns(text, 6, 7).starts_with('-') {
        -1.0
    } else {
        1.0
    };
    let mantissa = parse_float(&format!("0.{}", columns(text, 1, 6)), "mantissa")?;
    let exponent = parse_float(&columns(text, 7, 8), "exponent")?;
    Ok(sign * mantissa * 10_f64.powf(exponent_sign * exponent))
}

/// Upstream `_firstDeriv2ballistCoef`: the first derivative of the mean
/// motion \[rev/day²\].
fn first_derivative_field(text: &str) -> Result<f64, TleError> {
    let sign = if text.starts_with('-') { -1.0 } else { 1.0 };
    Ok(sign * parse_float(&columns(text, 1, text.chars().count()), "nDot")?)
}

/// The SGP4 inputs of a TLE's two lines, as `Satrec.twoline2rv` reads them.
struct Sgp4Fields {
    epoch_year: i32,
    epoch_days: f64,
    b_star: f64,
    inclination_deg: f64,
    raan_deg: f64,
    eccentricity: f64,
    arg_perigee_deg: f64,
    mean_anomaly_deg: f64,
    mean_motion_rev_per_day: f64,
}

impl Sgp4Fields {
    fn from_lines(line1: &str, line2: &str) -> Result<Self, TleError> {
        let year = parse_int(&columns(line1, 18, 20), "the epoch year")? as i32;
        Ok(Self {
            epoch_year: if year < 57 { year + 2000 } else { year + 1900 },
            epoch_days: parse_float(&columns(line1, 20, 32), "the epoch day")?,
            b_star: tle_exponential_field(&columns(line1, 53, 61))?,
            inclination_deg: parse_float(&columns(line2, 8, 16), "the inclination")?,
            raan_deg: parse_float(&columns(line2, 17, 25), "the ascending node")?,
            eccentricity: parse_float(
                &format!("0.{}", columns(line2, 26, 33).trim()),
                "the eccentricity",
            )?,
            arg_perigee_deg: parse_float(&columns(line2, 34, 42), "the argument of perigee")?,
            mean_anomaly_deg: parse_float(&columns(line2, 43, 51), "the mean anomaly")?,
            mean_motion_rev_per_day: parse_float(&columns(line2, 52, 63), "the mean motion")?,
        })
    }
}

/// Vallado's `jday`: the Julian date split into whole and fractional days.
fn jday(year: i32, month: u32, day: u32, hour: u32, minute: u32, second: f64) -> (f64, f64) {
    let (year, month) = (year as f64, month as f64);
    let jd = 367.0 * year - (7.0 * (year + ((month + 9.0) / 12.0).floor()) * 0.25).floor()
        + (275.0 * month / 9.0).floor()
        + day as f64
        + 1_721_013.5;
    let mut fraction = (second + minute as f64 * 60.0 + hour as f64 * 3600.0) / SEC_PER_DAY;
    let mut jd = jd;
    if fraction.abs() > 1.0 {
        let whole = fraction.floor();
        jd += whole;
        fraction -= whole;
    }
    (jd, fraction)
}

/// Vallado's `days2mdhms`: month, day, hour, minute and second of a day of
/// the year.
fn days2mdhms(year: i32, days: f64) -> (u32, u32, u32, u32, f64) {
    let mut month_lengths = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    if year % 4 == 0 {
        month_lengths[1] = 29;
    }
    let day_of_year = days.floor() as u32;
    let mut month = 1;
    let mut elapsed = 0;
    while day_of_year > elapsed + month_lengths[month - 1] && month < 12 {
        elapsed += month_lengths[month - 1];
        month += 1;
    }
    let day = day_of_year - elapsed;
    let hours = (days - days.floor()) * 24.0;
    let hour = hours.floor();
    let minutes = (hours - hour) * 60.0;
    let minute = minutes.floor();
    let second = (minutes - minute) * 60.0;
    (month as u32, day, hour as u32, minute as u32, second)
}

/// The TEME state \[m, m/s\] of the TLE `line1`, `line2` at `epoch`:
/// upstream `Satrec.twoline2rv(line1, line2).sgp4(*jday(epoch))`, WGS72 and
/// the improved operation mode.
fn sgp4_teme_state(
    line1: &str,
    line2: &str,
    epoch: &UtcDateTime,
) -> Result<(Vector3<f64>, Vector3<f64>), String> {
    let fields = Sgp4Fields::from_lines(line1, line2).map_err(|error| error.to_string())?;
    let (month, day, hour, minute, second) = days2mdhms(fields.epoch_year, fields.epoch_days);
    let (jd_sat, fraction_sat) = jday(fields.epoch_year, month, day, hour, minute, second);
    let (jd, fraction) = jday(
        epoch.year,
        epoch.month,
        epoch.day,
        epoch.hour,
        epoch.minute,
        epoch.second as f64 + epoch.microsecond as f64 / 1e6,
    );
    let minutes_since_epoch = (jd - jd_sat) * 1440.0 + (fraction - fraction_sat) * 1440.0;

    // `twoline2rv`: the Kozai mean motion in rad/min (`xpdotp = 1440/2π`).
    let orbit = sgp4::Orbit::from_kozai_elements(
        &sgp4::WGS72,
        fields.inclination_deg.to_radians(),
        fields.raan_deg.to_radians(),
        fields.eccentricity,
        fields.arg_perigee_deg.to_radians(),
        fields.mean_anomaly_deg.to_radians(),
        fields.mean_motion_rev_per_day / (1440.0 / (2.0 * PI)),
    )
    .map_err(|error| format!("{error:?}"))?;
    let epoch_years_since_j2000 = (jd_sat + fraction_sat - 2_451_545.0) / 365.25;
    let constants = sgp4::Constants::new(
        sgp4::WGS72,
        sgp4::iau_epoch_to_sidereal_time,
        epoch_years_since_j2000,
        fields.b_star,
        orbit,
    )
    .map_err(|error| format!("{error:?}"))?;
    let prediction = constants
        .propagate(sgp4::MinutesSinceEpoch(minutes_since_epoch))
        .map_err(|error| format!("{error:?}"))?;
    Ok((
        Vector3::from(prediction.position) * 1e3,
        Vector3::from(prediction.velocity) * 1e3,
    ))
}

/// The SGP4 state of a TLE at `epoch` in the GCRF \[m, m/s\].
fn sgp4_gcrf_state(
    line1: &str,
    line2: &str,
    epoch: &UtcDateTime,
) -> Result<(Vector3<f64>, Vector3<f64>), String> {
    let (r_teme, v_teme) = sgp4_teme_state(line1, line2, epoch)?;
    Ok(teme_to_gcrf(r_teme, v_teme, epoch))
}

fn mu_earth_m3ps2() -> f64 {
    MU_EARTH * 1e9
}

/// Upstream `_convertMean2osculating`: SGP4 at the TLE epoch, TEME to GCRF,
/// then the osculating elements.
fn mean_to_osculating(
    line1: &str,
    line2: &str,
    data: &TleData,
) -> Result<ClassicElements, TleError> {
    let (r, v) = sgp4_gcrf_state(line1, line2, &data.tle_epoch).map_err(|error| {
        record_error(format!(
            "SPG4 propagation failed for satellite {} with NORAD ID {} at epoch {}. Error code: {error}",
            data.sat_name, data.norad_id, data.tle_epoch
        ))
    })?;
    Ok(state_to_elements(mu_earth_m3ps2(), r, v))
}

// ------------------------------------------------------------------------- //
// Parsing (upstream `_parseTle`)

/// Upstream `_check_tle_id_is_5_digit_numeric`.
fn check_id_is_numeric(id_field: &str, line_name: &str) -> Result<(), TleError> {
    let id = id_field.trim();
    if id.is_empty() || !id.chars().all(|character| character.is_ascii_digit()) {
        return Err(record_error(format!(
            "_parseTle() received a non-numeric catalog field in {line_name} ('{id_field}'). \
             This parser only supports standard numeric TLE catalog IDs in the 5-character field."
        )));
    }
    Ok(())
}

/// Upstream `_check_tle_id_is_not_alpha5_like`.
fn check_id_is_not_alpha5(id_field: &str, line_name: &str) -> Result<(), TleError> {
    let id: Vec<char> = id_field.trim().to_uppercase().chars().collect();
    if id.len() == 5 && id[0].is_alphabetic() && id[1..].iter().all(char::is_ascii_digit) {
        return Err(record_error(format!(
            "_parseTle() received an Alpha-5-like catalog field in {line_name} ('{id_field}'). \
             This parser only supports classic numeric TLE IDs up to 99999. Use OMM \
             (CSV/JSON/XML) for newer catalog schemes."
        )));
    }
    Ok(())
}

/// Upstream `_check_tle_id_no_overflow`.
fn check_id_no_overflow(line: &str, line_name: &str) -> Result<(), TleError> {
    if line
        .chars()
        .nth(7)
        .is_some_and(|character| character.is_ascii_digit())
    {
        return Err(record_error(format!(
            "_parseTle() detected a catalog-ID overflow in {line_name}. This is likely a 6+ \
             digit object ID forced into the 5-character TLE field. Use OMM (CSV/JSON/XML) for \
             these objects."
        )));
    }
    Ok(())
}

/// Upstream `_calcTleChecksum`: the digits plus one per minus sign, mod 10.
fn tle_checksum<'a>(parts: impl IntoIterator<Item = &'a str>) -> u32 {
    parts
        .into_iter()
        .flat_map(str::chars)
        .map(|character| match character {
            '-' => 1,
            _ => character.to_digit(10).unwrap_or(0),
        })
        .sum::<u32>()
        % 10
}

/// Upstream `_parseTleYear`.
fn parse_tle_year(text: &str) -> Result<i32, TleError> {
    let year = parse_int(text, "the year")? as i32;
    Ok(if year >= SPUTNIK_LAUNCH_YEAR % 100 {
        1900 + year
    } else {
        2000 + year
    })
}

fn checksum_digit(line: &str, line_name: &str) -> Result<u32, TleError> {
    let text = columns(line, 68, 69);
    text.trim()
        .parse::<u32>()
        .map_err(|_| record_error(format!("invalid checksum '{text}' in {line_name}")))
}

/// Upstream `_parseTle`: one 2LE or 3LE record, its elements still the SGP4
/// mean elements read as two-body elements.
fn parse_tle(record: &[&str]) -> Result<TleData, TleError> {
    let (line0, line1, line2) = match record {
        [line0, line1, line2] => (*line0, *line1, *line2),
        [line1, line2] => ("", *line1, *line2),
        _ if record.len() > 3 => {
            return Err(record_error(format!(
                "_parseTle() received tle with {} lines. Most likely a concatenation of multiple \
                 TLEs.",
                record.len()
            )));
        }
        _ => {
            return Err(record_error(format!(
                "_parseTle() received tle with {} lines. The TLE should have 2 or 3 lines.",
                record.len()
            )));
        }
    };

    let line1_number = columns(line1, 0, 1);
    if line1_number != "1" {
        return Err(record_error(format!(
            "_parseTle() received line1 starting with '{line1_number}'. The first character of \
             line1 should be '1'."
        )));
    }
    check_id_no_overflow(line1, "line1")?;
    let sat_id1 = columns(line1, 2, 7);
    check_id_is_not_alpha5(&sat_id1, "line1")?;
    check_id_is_numeric(&sat_id1, "line1")?;
    let sat_id1_clean = sat_id1.trim().to_string();
    let class = columns(line1, 7, 8);
    let launch_year = columns(line1, 9, 11);
    let launch_no = columns(line1, 11, 14);
    let pol = columns(line1, 14, 17);
    let epoch_year = columns(line1, 18, 20);
    let epoch_day = columns(line1, 20, 32);
    let n_dot = columns(line1, 33, 43);
    let n_dot_dot = columns(line1, 44, 52);
    let b_star = columns(line1, 53, 61);
    let ephemeris_type = columns(line1, 62, 63);
    let elem_set_no = columns(line1, 64, 68);
    let checksum1 = checksum_digit(line1, "line1")?;
    let computed1 = tle_checksum([
        line1_number.as_str(),
        &sat_id1,
        &class,
        &launch_year,
        &launch_no,
        &pol,
        &epoch_year,
        &epoch_day,
        &n_dot,
        &n_dot_dot,
        &b_star,
        &ephemeris_type,
        &elem_set_no,
    ]);
    if checksum1 != computed1 {
        return Err(record_error(format!(
            "_parseTle() checksum mismatch for line1. The calculated checksum is {computed1} \
             while the provided checksum is {checksum1}."
        )));
    }

    let line2_number = columns(line2, 0, 1);
    if line2_number != "2" {
        return Err(record_error(format!(
            "_parseTle() received line2 starting with '{line2_number}'. The first character of \
             line2 should be '2'."
        )));
    }
    check_id_no_overflow(line2, "line2")?;
    let sat_id2_field = columns(line2, 2, 7);
    check_id_is_not_alpha5(&sat_id2_field, "line2")?;
    check_id_is_numeric(&sat_id2_field, "line2")?;
    let sat_id2 = sat_id2_field.trim().to_string();
    if sat_id2 != sat_id1_clean {
        return Err(record_error(format!(
            "_parseTle() received mismatched satellite ID numbers in line1 ('{sat_id1}') and \
             line2 ('{sat_id2}')."
        )));
    }
    let inclination = columns(line2, 8, 16);
    let raan = columns(line2, 17, 25);
    let eccentricity = columns(line2, 26, 33);
    let arg_perigee = columns(line2, 34, 42);
    let mean_anomaly = columns(line2, 43, 51);
    let mean_motion = columns(line2, 52, 63);
    let rev_no = columns(line2, 63, 68);
    let checksum2 = checksum_digit(line2, "line2")?;
    let computed2 = tle_checksum([
        line2_number.as_str(),
        &sat_id2,
        &inclination,
        &raan,
        &eccentricity,
        &arg_perigee,
        &mean_anomaly,
        &mean_motion,
        &rev_no,
    ]);
    if checksum2 != computed2 {
        return Err(record_error(format!(
            "_parseTle() checksum mismatch for line2. The calculated checksum is {computed2} \
             while the provided checksum is {checksum2}."
        )));
    }

    let classification = match class.as_str() {
        "U" => "Unclassified",
        "C" => "Classified",
        "S" => "Secret",
        _ => "Unknown",
    };
    let tle_epoch = UtcDateTime::from_day_of_year(
        parse_tle_year(&epoch_year)?,
        parse_float(&epoch_day, "the epoch day")?,
    );

    // Only SGP4 element sets: types 0 (distributed TLEs) and 2 (SGP4), or a
    // blank column. Type 4 (SGP4-XP) holds other quantities in its drag fields.
    if !matches!(ephemeris_type.trim(), "" | "0" | "2") {
        return Err(record_error(format!(
            "_parseTle() received an unsupported ephemeris type '{ephemeris_type}' for NORAD ID \
             {sat_id1_clean}; expected 0 (default) or 2 (SGP4). Type 4 element sets need SGP4-XP."
        )));
    }
    let propagator = match ephemeris_type.as_str() {
        "1" => "SGP4",
        "2" => "SDP4",
        "3" => "SGP8",
        "4" => "SDP8",
        _ => "Unknown",
    };

    let e = parse_float(&format!("0.{eccentricity}"), "the eccentricity")?;
    let revolutions_per_day = parse_float(&mean_motion, "the mean motion")?;
    let mean_anomaly_rad = parse_float(&mean_anomaly, "the mean anomaly")?.to_radians();
    let oe = ClassicElements {
        semi_major_axis: (MU_EARTH / ((revolutions_per_day * 2.0 * PI) / SEC_PER_DAY).powi(2))
            .powf(1.0 / 3.0)
            * 1e3,
        eccentricity: e,
        inclination_rad: parse_float(&inclination, "the inclination")?.to_radians(),
        raan_rad: parse_float(&raan, "the ascending node")?.to_radians(),
        arg_periapsis_rad: parse_float(&arg_perigee, "the argument of perigee")?.to_radians(),
        true_anomaly_rad: eccentric_to_true_anomaly(
            mean_to_eccentric_anomaly(mean_anomaly_rad, e),
            e,
        ),
        ..Default::default()
    };

    let mut data = TleData::new(oe, tle_epoch);
    data.sat_name = line0.trim().to_string();
    data.norad_id = sat_id1_clean;
    data.classification = classification.to_string();
    data.rev_at_epoch = parse_float(&rev_no, "the revolution number")?;
    data.propagator = propagator.to_string();
    data.elem_set_no = parse_int(&elem_set_no, "the element set number")? as u32;
    data.n_dot = first_derivative_field(&n_dot)?;
    data.n_dot_dot = tle_exponential_field(&n_dot_dot)?;
    data.b_star = tle_exponential_field(&b_star)?;
    // Analyst objects can have a blank international designator: keep the
    // launch defaults.
    if !format!("{launch_year}{launch_no}{pol}").trim().is_empty() {
        data.launch_date = Some(UtcDateTime::midnight(parse_tle_year(&launch_year)?, 1, 1));
        data.launch_no = parse_int(&launch_no, "the launch number")? as u32;
        data.pol = pol;
    }
    Ok(data)
}

// ------------------------------------------------------------------------- //
// File layout (upstream `_isTleElementLine`, `_isTleTitleLine`)

/// Upstream `_isTleElementLine`: the line starts with its number and a blank.
fn is_element_line(line: &str, number: u8) -> bool {
    let mut characters = line.chars();
    characters.next() == Some(char::from(b'0' + number)) && characters.next() == Some(' ')
}

fn is_digit(character: char) -> bool {
    character.is_ascii_digit()
}

fn is_digit_or_blank(character: char) -> bool {
    character == ' ' || character.is_ascii_digit()
}

/// One position of a pattern: a character predicate, `None` for any
/// character (`.`).
type CharClass = Option<fn(char) -> bool>;

/// `pattern` matched at `start` of `line`.
fn matches_at(line: &[char], start: usize, pattern: &[CharClass]) -> bool {
    start + pattern.len() <= line.len()
        && pattern
            .iter()
            .zip(&line[start..])
            .all(|(class, character)| class.is_none_or(|predicate| predicate(*character)))
}

fn repeat(class: CharClass, count: usize) -> Vec<CharClass> {
    vec![class; count]
}

fn literal(character: char) -> CharClass {
    // Only the literals the TLE patterns use.
    match character {
        ' ' => Some(|c| c == ' '),
        '.' => Some(|c| c == '.'),
        '1' => Some(|c| c == '1'),
        '2' => Some(|c| c == '2'),
        _ => unreachable!("no pattern uses {character:?}"),
    }
}

/// `r"[0-9]{5}\.[0-9]{8} [ +-]\.[0-9]{8}(?: |$)"` or
/// `r"[ 0-9]{3}\.[0-9]{4} [0-9]{7} [ 0-9]{3}\.[0-9]{4}(?: |$)"` (line 2)
/// found anywhere in `body`: the intact epoch and derivative fields of line
/// 1, or the node, eccentricity and perigee fields of line 2.
fn contains_element_body(body: &[char], line_number: char) -> bool {
    let digit = Some(is_digit as fn(char) -> bool);
    let digit_or_blank = Some(is_digit_or_blank as fn(char) -> bool);
    let pattern: Vec<CharClass> = if line_number == '1' {
        [
            repeat(digit, 5),
            vec![literal('.')],
            repeat(digit, 8),
            vec![
                literal(' '),
                Some((|c| matches!(c, ' ' | '+' | '-')) as fn(char) -> bool),
                literal('.'),
            ],
            repeat(digit, 8),
        ]
        .concat()
    } else {
        [
            repeat(digit_or_blank, 3),
            vec![literal('.')],
            repeat(digit, 4),
            vec![literal(' ')],
            repeat(digit, 7),
            vec![literal(' ')],
            repeat(digit_or_blank, 3),
            vec![literal('.')],
            repeat(digit, 4),
        ]
        .concat()
    };
    (0..=body.len().saturating_sub(pattern.len())).any(|start| {
        matches_at(body, start, &pattern)
            && body
                .get(start + pattern.len())
                .is_none_or(|next| *next == ' ')
    })
}

/// `r"1 .{15} .{5}\..{8} ."` or `r"2 .{14} .{3}\..{4} ."` matched at the
/// start: the field boundaries of a longer element-line fragment.
fn matches_field_boundaries(line: &[char]) -> bool {
    let line1 = [
        vec![literal('1'), literal(' ')],
        repeat(None, 15),
        vec![literal(' ')],
        repeat(None, 5),
        vec![literal('.')],
        repeat(None, 8),
        vec![literal(' '), None],
    ]
    .concat();
    let line2 = [
        vec![literal('2'), literal(' ')],
        repeat(None, 14),
        vec![literal(' ')],
        repeat(None, 3),
        vec![literal('.')],
        repeat(None, 4),
        vec![literal(' '), None],
    ]
    .concat();
    matches_at(line, 0, &line1) || matches_at(line, 0, &line2)
}

/// The lengths of `(?:[ 0-9]{5}|[A-Za-z][0-9]{4}|[0-9]{6,})` matching at
/// `start`.
fn catalog_field_lengths(line: &[char], start: usize) -> Vec<usize> {
    let rest = line.get(start..).unwrap_or(&[]);
    let mut lengths = Vec::new();
    if rest.len() >= 5 && rest[..5].iter().copied().all(is_digit_or_blank) {
        lengths.push(5);
    }
    if rest.len() >= 5 && rest[0].is_ascii_alphabetic() && rest[1..5].iter().copied().all(is_digit)
    {
        lengths.push(5);
    }
    let digits = rest
        .iter()
        .take_while(|character| is_digit(**character))
        .count();
    lengths.extend((6..=digits).rev());
    lengths
}

/// `"1 " + catalog + r"[A-Z ] (?:[0-9]{5}|.{8} .)"` or
/// `"2 " + catalog + r" (?:[ 0-9]{3}\.|.{8} .)"` matched at the start: a
/// recognizable truncated element-line header.
fn matches_header(line: &[char]) -> bool {
    let digit = Some(is_digit as fn(char) -> bool);
    let digit_or_blank = Some(is_digit_or_blank as fn(char) -> bool);
    let upper_or_blank = Some((|c: char| c == ' ' || c.is_ascii_uppercase()) as fn(char) -> bool);
    let any_then_blank = [repeat(None, 8), vec![literal(' '), None]].concat();
    let line1_rest_a = [vec![upper_or_blank, literal(' ')], repeat(digit, 5)].concat();
    let line1_rest_b = [vec![upper_or_blank, literal(' ')], any_then_blank.clone()].concat();
    let line2_rest_a = [
        vec![literal(' ')],
        repeat(digit_or_blank, 3),
        vec![literal('.')],
    ]
    .concat();
    let line2_rest_b = [vec![literal(' ')], any_then_blank].concat();
    let (rests, number) = match line.first() {
        Some('1') => ([line1_rest_a, line1_rest_b], '1'),
        Some('2') => ([line2_rest_a, line2_rest_b], '2'),
        _ => return false,
    };
    matches_at(line, 0, &[literal(number), literal(' ')])
        && catalog_field_lengths(line, 2)
            .into_iter()
            .any(|length| rests.iter().any(|rest| matches_at(line, 2 + length, rest)))
}

/// Upstream `_isTleTitleLine`: whether a line can be a satellite name or a
/// file title, not an element line or a recognizable fragment of one.
fn is_title_line(line: &str) -> bool {
    let line = line.trim_end();
    if !(is_element_line(line, 1) || is_element_line(line, 2)) {
        return true;
    }
    let characters: Vec<char> = line.chars().collect();
    if characters.len() >= 69 {
        return false;
    }
    if contains_element_body(&characters[2..], characters[0]) {
        return false;
    }
    if matches_field_boundaries(&characters) {
        return false;
    }
    !matches_header(&characters)
}

/// Python `str.splitlines()` for `\n`, `\r\n` and `\r` line breaks.
fn split_lines(text: &str) -> Vec<&str> {
    let mut lines = Vec::new();
    let mut rest = text;
    while !rest.is_empty() {
        match rest.find(['\n', '\r']) {
            Some(index) => {
                lines.push(&rest[..index]);
                let skip = if rest[index..].starts_with("\r\n") {
                    2
                } else {
                    1
                };
                rest = &rest[index + skip..];
            }
            None => {
                lines.push(rest);
                rest = "";
            }
        }
    }
    lines
}

// ------------------------------------------------------------------------- //
// Reading (upstream `satTle2elem`)

/// Upstream `satTle2elem(tle_path, on_error=...)`: every SGP4 element set of
/// a 2LE or 3LE file, with osculating GCRF elements in [`TleData::oe`].
///
/// See [`parse_tle_text`] for the layout rules and the error policy.
pub fn sat_tle_to_elements(
    path: impl AsRef<Path>,
    on_error: OnError,
) -> Result<Vec<TleData>, TleError> {
    let path = path.as_ref();
    let text = std::fs::read_to_string(path).map_err(TleError::Io)?;
    parse_tle_text(&text, &path.display().to_string(), on_error)
}

/// [`sat_tle_to_elements`] of the file contents `text`; `label` names the
/// file in messages.
///
/// The ephemeris type (column 63 of line 1) may be blank, 0 or 2; other types
/// are record errors. A blank international designator leaves
/// [`TleData::launch_date`] `None`. Trailing blank lines are ignored; a 3LE
/// layout (every third line from the second a line-1/line-2 pair) and a 2LE
/// layout with a title line are recognized before counting element-line
/// prefixes, as names may themselves start with `1 ` or `2 `. An element
/// line, or a recognizable fragment of one, in a name or title position is a
/// [`TleError::Format`] error.
///
/// With [`OnError::Raise`] the first record error is returned; with
/// [`OnError::Skip`] each failing set is skipped with a warning, and the
/// first error is returned only when every set fails.
pub fn parse_tle_text(
    text: &str,
    label: &str,
    on_error: OnError,
) -> Result<Vec<TleData>, TleError> {
    let mut lines = split_lines(text);
    while lines.last().is_some_and(|line| line.trim().is_empty()) {
        lines.pop();
    }
    let line_count = lines.len();
    if line_count < 2 {
        return Err(TleError::Format(format!(
            "constTLE2Elem() received a TLE with {line_count} lines. A valid TLE must have at \
             least two lines."
        )));
    }

    let pairs_from = |first: usize, stride: usize| {
        (first..line_count).step_by(stride).all(|index| {
            is_element_line(lines[index], 1)
                && lines
                    .get(index + 1)
                    .is_some_and(|next| is_element_line(next, 2))
        })
    };
    let mut first_record_line = 0;
    let tle_type = if line_count.is_multiple_of(3) && pairs_from(1, 3) {
        for index in (0..line_count).step_by(3) {
            if !is_title_line(lines[index]) {
                return Err(TleError::Format(format!(
                    "satTle2elem() found a possible orphaned TLE element line in a \
                     satellite-name position on line {} in {label}.",
                    index + 1
                )));
            }
        }
        3
    } else if !line_count.is_multiple_of(2) && pairs_from(1, 2) {
        if !is_title_line(lines[0]) {
            return Err(TleError::Format(format!(
                "satTle2elem() found a possible orphaned TLE element line in a file-title \
                 position on line 1 in {label}."
            )));
        }
        first_record_line = 1;
        2
    } else {
        let ones = lines.iter().filter(|line| is_element_line(line, 1)).count();
        let twos = lines.iter().filter(|line| is_element_line(line, 2)).count();
        let others = line_count - ones - twos;
        if ones != twos {
            return Err(TleError::Format(format!(
                "constTLE2Elem() received a TLE with {ones} lines starting with '1' and {twos} \
                 lines starting with '2'. The number of lines starting with '1' and '2' must be \
                 equal."
            )));
        }
        if others >= ones {
            3
        } else if others == 0 {
            2
        } else if others == 1 && line_count > 2 {
            if is_element_line(lines[0], 1) || is_element_line(lines[0], 2) {
                return Err(TleError::Format(
                    "constTLE2Elem() received a TLE with one title line but the first line \
                     starts with '1' or '2'. --- TLE not valid ---."
                        .to_string(),
                ));
            }
            2
        } else {
            return Err(TleError::Format(format!(
                "constTLE2Elem() could not identify the 2LE or 3LE layout of {label}."
            )));
        }
    };

    #[derive(Clone, Copy, PartialEq)]
    enum Expected {
        Title,
        Line1,
        Line2,
    }
    let reading_order: &[Expected] = if tle_type == 2 {
        &[Expected::Line1, Expected::Line2]
    } else {
        &[Expected::Title, Expected::Line1, Expected::Line2]
    };
    let mut record: Vec<&str> = vec![""; tle_type];
    let mut ready = vec![false; tle_type];
    let mut order_index = 0;
    let mut line1 = "";
    let mut line2 = "";
    let mut set_number = 0;
    let mut records = Vec::new();
    let mut skipped = Vec::new();
    let reset = |record: &mut Vec<&str>, ready: &mut Vec<bool>, order_index: &mut usize| {
        record.iter_mut().for_each(|line| *line = "");
        ready.iter_mut().for_each(|flag| *flag = false);
        *order_index = 0;
    };

    for (line_number, line) in lines.iter().enumerate().skip(first_record_line) {
        let expected = reading_order.get(order_index).copied();
        if is_element_line(line, 1) && expected == Some(Expected::Line1) {
            if ready[1] {
                log::warn!(
                    "constTLE2Elem() found a new TLE line1 before completing the previous TLE \
                     (line number {line_number}). The previous TLE will be discarded."
                );
                reset(&mut record, &mut ready, &mut order_index);
            }
            record[tle_type - 2] = line;
            ready[tle_type - 2] = true;
            order_index += 1;
            line1 = line;
        } else if is_element_line(line, 2) && expected == Some(Expected::Line2) {
            if ready[tle_type - 1] {
                log::warn!(
                    "constTLE2Elem() found a new TLE line2 before completing the previous TLE \
                     (line number {line_number}). The previous TLE will be discarded."
                );
                reset(&mut record, &mut ready, &mut order_index);
            }
            record[tle_type - 1] = line;
            ready[tle_type - 1] = true;
            line2 = line;
        } else if expected == Some(Expected::Title) && tle_type == 3 {
            if ready[0] {
                log::warn!(
                    "constTLE2Elem() found a new TLE title line before completing the previous \
                     TLE (line number {line_number}). The previous TLE will be discarded."
                );
                reset(&mut record, &mut ready, &mut order_index);
            }
            record[0] = line;
            ready[0] = true;
            order_index += 1;
        } else {
            log::warn!(
                "constTLE2Elem() found an unexpected line: {}, line number {line_number}. This \
                 line will be ignored. The previous TLE will be discarded.",
                line.trim()
            );
            continue;
        }

        if ready.iter().all(|flag| *flag) {
            set_number += 1;
            let parsed = parse_tle(&record).and_then(|mut data| {
                data.oe = mean_to_osculating(line1, line2, &data)?;
                Ok(data)
            });
            match parsed {
                Ok(data) => records.push(data),
                Err(error) if on_error == OnError::Skip => {
                    log::warn!(
                        "satTle2elem() skipped TLE set {set_number} (ending on line \
                         {line_number}) in {label}: {error}"
                    );
                    skipped.push(error);
                }
                Err(error) => return Err(error),
            }
            reset(&mut record, &mut ready, &mut order_index);
        }
    }
    if records.is_empty() && !skipped.is_empty() {
        return Err(skipped.swap_remove(0));
    }
    Ok(records)
}

// ------------------------------------------------------------------------- //
// Writing (upstream `generateTle`)

/// Upstream `_wrap_deg`: degrees in `[0, 360)`.
fn wrap_deg(angle_rad: f64) -> f64 {
    angle_rad.to_degrees().rem_euclid(360.0)
}

/// Upstream `_norad_id_to_tle_str`: the 5-digit catalog field.
fn norad_id_field(norad_id: &str) -> Result<String, TleError> {
    let id: u64 = norad_id
        .trim()
        .parse()
        .map_err(|_| record_error(format!("invalid NORAD catalog number '{norad_id}'")))?;
    if id > TLE_MAX_NORAD_ID {
        return Err(record_error(format!(
            "TLE format cannot represent catalog number {id}. The TLE 5-character field supports \
             integers 0–{TLE_MAX_NORAD_ID}. Objects with 6+ digit catalog numbers (assigned from \
             ~July 2026) require the OMM format (CSV/JSON/XML) instead. See: \
             https://celestrak.org/NORAD/elements/gp-faq.php"
        )));
    }
    Ok(format!("{id:05}"))
}

/// Upstream `_str2tleFormat`: the second derivative of the mean motion or
/// B* as a decimal-point-assumed mantissa and exponent.
fn tle_exponential_string(value: f64) -> String {
    let value = value * 10.0;
    let sign = if value >= 0.0 { " " } else { "-" };
    let scientific = format!("{:.4e}", value.abs());
    let (mantissa, exponent) = scientific
        .split_once('e')
        .expect("scientific notation has an exponent");
    let mantissa: String = mantissa.replace('.', "").chars().take(5).collect();
    let exponent: i32 = exponent.parse().expect("integer exponent");
    let exponent_sign = if exponent > 0 { "+" } else { "-" };
    format!("{sign}{mantissa}{exponent_sign}{}", exponent.abs())
}

/// The epoch field `YYDDD.DDDDDDDD` (upstream `f"{epoch:%y%j}" + ...`).
fn epoch_field(epoch: &UtcDateTime) -> String {
    let fraction = format!("{:.8}", epoch.seconds_of_day() / SEC_PER_DAY);
    format!(
        "{:02}{:03}{}",
        epoch.year.rem_euclid(100),
        epoch.day_of_year(),
        &fraction[1..]
    )
}

/// The classification letter: the first letter of [`TleData::classification`]
/// when it is `U`, `C` or `S`, else `U`.
fn classification_letter(classification: &str) -> char {
    let letter = classification
        .chars()
        .next()
        .map(|letter| letter.to_ascii_uppercase());
    match letter {
        Some('C') => 'C',
        Some('S') => 'S',
        _ => 'U',
    }
}

fn pol_field(pol: &str) -> String {
    format!("{pol:<3}").chars().take(3).collect()
}

fn launch_date_of(data: &TleData) -> Result<UtcDateTime, TleError> {
    data.launch_date.ok_or_else(|| {
        record_error("generateTle() needs the launch date of the international designator")
    })
}

/// Upstream `_generateTleFromMean`: a TLE of the mean elements `mean`, with
/// zero derivatives of the mean motion.
fn tle_from_mean(mean: &ClassicElements, data: &TleData) -> Result<[String; 3], TleError> {
    let norad = norad_id_field(&data.norad_id)?;
    let launch_date = launch_date_of(data)?;
    let line1 = format!(
        "1 {norad}{} {:02}{:03}{} {}  .00000000  00000-0 {} 0  {:03}",
        classification_letter(&data.classification),
        launch_date.year.rem_euclid(100),
        data.launch_no,
        pol_field(&data.pol),
        epoch_field(&data.tle_epoch),
        tle_exponential_string(data.b_star),
        data.elem_set_no,
    );
    let line1 = format!("{line1}{}", tle_checksum([line1.as_str()]));
    let a_km = mean.semi_major_axis / 1000.0;
    let mean_motion = (MU_EARTH / a_km.powi(3)).sqrt() * SEC_PER_DAY / (2.0 * PI);
    let mean_anomaly = eccentric_to_mean_anomaly(
        true_to_eccentric_anomaly(mean.true_anomaly_rad, mean.eccentricity),
        mean.eccentricity,
    );
    let line2 = format!(
        "2 {norad} {:8.4} {:8.4} {:07} {:8.4} {:8.4} {:11.8}{:05}",
        mean.inclination_rad.to_degrees(),
        wrap_deg(mean.raan_rad),
        (mean.eccentricity * 1e7) as i64,
        wrap_deg(mean.arg_periapsis_rad),
        mean_anomaly.to_degrees().rem_euclid(360.0),
        mean_motion,
        data.rev_at_epoch as i64,
    );
    let line2 = format!("{line2}{}", tle_checksum([line2.as_str()]));
    Ok([data.sat_name.clone(), line1, line2])
}

fn six_elements(oe: &ClassicElements) -> ClassicElements {
    ClassicElements {
        semi_major_axis: oe.semi_major_axis,
        eccentricity: oe.eccentricity,
        inclination_rad: oe.inclination_rad,
        raan_rad: oe.raan_rad,
        arg_periapsis_rad: oe.arg_periapsis_rad,
        true_anomaly_rad: oe.true_anomaly_rad,
        ..Default::default()
    }
}

/// Upstream `_osculating2mean_sgp4`: SGP4 mean elements whose SGP4 position
/// at the epoch, through the quantized TLE, matches the osculating one,
/// starting from the first-order J2 mean elements.
fn osculating_to_mean_sgp4(data: &TleData) -> Result<ClassicElements, TleError> {
    let osculating = data.oe;
    let mut mean = mean_osculating_map(
        REQ_EARTH * 1e3,
        J2_EARTH,
        &osculating,
        MeanOscDirection::OsculatingToMean,
    );
    let (r_target, _) = elements_to_state(mu_earth_m3ps2(), &osculating);

    let mut best_error = f64::INFINITY;
    let mut best = six_elements(&mean);
    let mut stall_count = 0;
    for iteration in 0..MEAN_ELEMENT_MAX_ITERATIONS {
        let [_, line1, line2] = tle_from_mean(&mean, data)?;
        let (r_sgp4, v_sgp4) =
            sgp4_gcrf_state(&line1, &line2, &data.tle_epoch).map_err(|error| {
                record_error(format!(
                    "SGP4 propagation failed (error code {error}) for satellite '{}' (NORAD {}) \
                     at iteration {iteration}.",
                    data.sat_name, data.norad_id
                ))
            })?;
        let position_error = (r_target - r_sgp4).norm();
        if position_error < best_error {
            best_error = position_error;
            best = six_elements(&mean);
            stall_count = 0;
        } else {
            stall_count += 1;
        }
        if position_error < MEAN_ELEMENT_TOLERANCE_M {
            return Ok(mean);
        }
        if stall_count >= MEAN_ELEMENT_STALL_LIMIT && best_error < TLE_PRECISION_FLOOR_M {
            return Ok(best);
        }
        let sgp4 = state_to_elements(mu_earth_m3ps2(), r_sgp4, v_sgp4);
        mean.semi_major_axis += osculating.semi_major_axis - sgp4.semi_major_axis;
        mean.eccentricity += osculating.eccentricity - sgp4.eccentricity;
        mean.inclination_rad += osculating.inclination_rad - sgp4.inclination_rad;
        mean.raan_rad += osculating.raan_rad - sgp4.raan_rad;
        mean.arg_periapsis_rad += osculating.arg_periapsis_rad - sgp4.arg_periapsis_rad;
        mean.true_anomaly_rad += osculating.true_anomaly_rad - sgp4.true_anomaly_rad;
        mean.raan_rad = mean.raan_rad.rem_euclid(2.0 * PI);
        mean.arg_periapsis_rad = mean.arg_periapsis_rad.rem_euclid(2.0 * PI);
        mean.true_anomaly_rad = mean.true_anomaly_rad.rem_euclid(2.0 * PI);
    }
    if best_error < TLE_PRECISION_FLOOR_M {
        return Ok(best);
    }
    Err(record_error(format!(
        "Mean-element iteration did not converge for satellite '{}' (NORAD {}) after \
         {MEAN_ELEMENT_MAX_ITERATIONS} iterations. Final position error: {best_error:.2} m \
         (tolerance: {MEAN_ELEMENT_TOLERANCE_M} m).",
        data.sat_name, data.norad_id
    )))
}

/// Upstream `generateTle(tleData)`: the 3LE (name, line 1, line 2, joined by
/// newlines) of one satellite from its osculating elements and metadata.
///
/// The mean elements are iterated through SGP4 until it reproduces the
/// osculating position to 0.1 m (or the TLE quantization floor, 15 m). The
/// revolution number written is 0 and the ephemeris type 0, as upstream.
/// Upstream fails without a launch date, before reaching its default; here
/// that is a [`TleError::Record`].
pub fn generate_tle(data: &TleData) -> Result<String, TleError> {
    let mean = osculating_to_mean_sgp4(data)?;
    let norad = norad_id_field(&data.norad_id)?;
    let launch_date = launch_date_of(data)?;
    let pol = if data.pol.chars().count() > 3 {
        "A".to_string()
    } else {
        data.pol.clone()
    };

    let launch_year = format!("{:02}", launch_date.year.rem_euclid(100));
    let launch_no = format!("{:03}", data.launch_no);
    let epoch = epoch_field(&data.tle_epoch);
    let n_dot = format!("{:10.8}", data.n_dot).replace("-0", "-");
    let n_dot = if data.n_dot > 0.0 {
        n_dot.replace("0.", " .")
    } else {
        n_dot.replace("0.", "-.")
    };
    let n_dot_dot = tle_exponential_string(data.n_dot_dot);
    let b_star = tle_exponential_string(data.b_star);
    let ephemeris = "0";
    let elem_set_no = format!("{:03}", data.elem_set_no);
    let checksum1 = tle_checksum([
        "1",
        &norad,
        &launch_year,
        &launch_no,
        &epoch,
        &n_dot,
        &n_dot_dot,
        &b_star,
        ephemeris,
        &elem_set_no,
    ]);
    let line1 = format!(
        "1 {norad}{} {launch_year}{launch_no}{} {epoch} {n_dot} {n_dot_dot} {b_star} \
         {ephemeris}  {elem_set_no}{checksum1}",
        classification_letter(&data.classification),
        pol_field(&pol),
    );

    let inclination = format!("{:8.4}", mean.inclination_rad.to_degrees());
    let raan = format!("{:8.4}", wrap_deg(mean.raan_rad));
    let eccentricity = format!("{:07}", (mean.eccentricity * 1e7) as i64);
    let arg_perigee = format!("{:8.4}", wrap_deg(mean.arg_periapsis_rad));
    let mean_anomaly = eccentric_to_mean_anomaly(
        true_to_eccentric_anomaly(mean.true_anomaly_rad, mean.eccentricity),
        mean.eccentricity,
    );
    let mean_anomaly = format!("{:8.4}", mean_anomaly.to_degrees().rem_euclid(360.0));
    let mean_motion = format!(
        "{:11.8}",
        (MU_EARTH / (mean.semi_major_axis / 1000.0).powi(3)).sqrt() * SEC_PER_DAY / (2.0 * PI)
    );
    let revolution = "00000";
    let checksum2 = tle_checksum([
        "2",
        &norad,
        &inclination,
        &raan,
        &eccentricity,
        &arg_perigee,
        &mean_anomaly,
        &mean_motion,
        revolution,
    ]);
    let line2 = format!(
        "2 {norad} {inclination} {raan} {eccentricity} {arg_perigee} {mean_anomaly} \
         {mean_motion}{revolution}{checksum2}"
    );
    Ok(format!("{}\n{line1}\n{line2}", data.sat_name))
}

#[cfg(test)]
mod tests {
    //! Upstream `src/tests/test_readTLE.py`, with its truth values and
    //! tolerances. The TLE files are upstream's
    //! `examples/MultiSatBskSim/tleData`, copied to `examples/data/tle`.

    use super::*;

    const HYPSO1: &str = include_str!("../../examples/data/tle/hypso1.tle");
    const ONE_WEB: &str = include_str!("../../examples/data/tle/oneWeb25.tle");
    const SPACE_STATIONS: &str = include_str!("../../examples/data/tle/spacestations.2le");

    /// Upstream `A_TOL_DIST`, `A_TOL`, `A_TOL_ROUNDTRIP`,
    /// `A_TOL_DEG_ROUNDTRIP` and `A_TOL_ECC_ROUNDTRIP`.
    const A_TOL_DIST: f64 = 0.1;
    const A_TOL: f64 = 1e-5;
    const A_TOL_ROUNDTRIP: f64 = 1e-4;
    const A_TOL_DEG_ROUNDTRIP: f64 = 1.5;
    const A_TOL_ECC_ROUNDTRIP: f64 = 1e3;

    /// `(a, e, i, Omega, omega, f)` per satellite.
    type Expected = [[f64; 6]];

    const EXPECTED_ONE_WEB: [[f64; 6]; 6] = [
        [
            7578544.80840588,
            0.0008817590462989669,
            1.531618126589653,
            4.8831623958720565,
            1.4377465022195557,
            2.1381866215235528,
        ],
        [
            7581725.314320893,
            0.001210093768077448,
            1.5316291459001932,
            4.883186443196075,
            1.2801901375990692,
            5.002606275268809,
        ],
        [
            7581725.090937277,
            0.001258633728769965,
            1.5316258452568352,
            4.883574022425417,
            1.2537640336417852,
            5.029031746586543,
        ],
        [
            7585736.508260437,
            0.0011981313722308415,
            1.5322241554919225,
            5.415984679757847,
            1.2645835093359827,
            5.017006771870916,
        ],
        [
            7585735.9677996775,
            0.001206952489243135,
            1.5322307739504486,
            5.414680762509103,
            1.2744200510794836,
            5.0071733437393355,
        ],
        [
            7585737.120163768,
            0.001239828742668404,
            1.5322225935006697,
            5.415014177551135,
            1.278331704552413,
            5.003259952527291,
        ],
    ];

    const EXPECTED_2LE: [[f64; 6]; 3] = [
        [
            6802505.842859207,
            0.0011264735370325686,
            0.903248948599224,
            2.3886009471482503,
            0.8571434002531804,
            5.428341410047342,
        ],
        [
            6770009.6048538275,
            0.0011530055595853114,
            0.7265482390037918,
            1.6261518262462817,
            0.7478010900864719,
            5.535537401708238,
        ],
        [
            6802528.7123854,
            0.001128835473051786,
            0.9032099423215613,
            2.410867356948434,
            0.8567388460152983,
            5.428797206538743,
        ],
    ];

    /// Upstream `oeHypso1`.
    fn hypso1_elements() -> ClassicElements {
        ClassicElements {
            inclination_rad: 1.6979291907417804,
            eccentricity: 0.0013996087479637449,
            semi_major_axis: 6808434.893655814,
            raan_rad: 6.085804103135117,
            arg_periapsis_rad: 1.43544167146666,
            true_anomaly_rad: 4.845270389715793,
            ..Default::default()
        }
    }

    fn expected_hypso1() -> [[f64; 6]; 1] {
        let oe = hypso1_elements();
        [[
            oe.semi_major_axis,
            oe.eccentricity,
            oe.inclination_rad,
            oe.raan_rad,
            oe.arg_periapsis_rad,
            oe.true_anomaly_rad,
        ]]
    }

    fn read(text: &str, on_error: OnError) -> Result<Vec<TleData>, TleError> {
        parse_tle_text(text, "test.tle", on_error)
    }

    fn lines_of(text: &str) -> Vec<String> {
        split_lines(text).into_iter().map(str::to_string).collect()
    }

    /// Upstream `_with_checksum`.
    fn with_checksum(line: &str) -> String {
        let body = columns(line, 0, 68);
        let total: u32 = body
            .chars()
            .map(|character| match character {
                '-' => 1,
                _ => character.to_digit(10).unwrap_or(0),
            })
            .sum();
        format!("{body}{}", total % 10)
    }

    /// Upstream `_write_modified_hypso_tle` (as text).
    fn modified_hypso(line1: Option<String>, line2: Option<String>) -> String {
        let mut lines = lines_of(HYPSO1);
        if let Some(line1) = line1 {
            lines[1] = line1;
        }
        if let Some(line2) = line2 {
            lines[2] = line2;
        }
        lines.join("\n") + "\n"
    }

    fn six(oe: &ClassicElements) -> [f64; 6] {
        [
            oe.semi_major_axis,
            oe.eccentricity,
            oe.inclination_rad,
            oe.raan_rad,
            oe.arg_periapsis_rad,
            oe.true_anomaly_rad,
        ]
    }

    /// Upstream `test_read_tle`.
    fn check_read(text: &str, expected: &Expected) {
        let records = read(text, OnError::Raise).expect("readable TLE");
        assert_eq!(records.len(), expected.len());
        let names = ["a", "e", "i", "Omega", "omega", "f"];
        let mut failures = Vec::new();
        for (record, truth) in records.iter().zip(expected) {
            for (index, (actual, wanted)) in six(&record.oe).iter().zip(truth).enumerate() {
                let tolerance = if index == 0 { A_TOL_DIST } else { A_TOL };
                if (actual - wanted).abs() >= tolerance {
                    failures.push(format!(
                        "{}_{}: expected {wanted}, got {actual}",
                        record.norad_id, names[index]
                    ));
                }
            }
        }
        assert!(failures.is_empty(), "{failures:#?}");
    }

    #[test]
    fn read_hypso1() {
        check_read(HYPSO1, &expected_hypso1());
    }

    #[test]
    fn read_one_web() {
        check_read(ONE_WEB, &EXPECTED_ONE_WEB);
    }

    #[test]
    fn read_space_stations_2le() {
        check_read(SPACE_STATIONS, &EXPECTED_2LE);
    }

    #[test]
    fn read_from_file() {
        let path =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("examples/data/tle/spacestations.2le");
        let records = sat_tle_to_elements(path, OnError::Raise).expect("readable file");
        assert_eq!(
            records
                .iter()
                .map(|record| record.norad_id.as_str())
                .collect::<Vec<_>>(),
            ["25544", "48274", "49044"]
        );
        assert!(matches!(
            sat_tle_to_elements("no/such/file.tle", OnError::Raise),
            Err(TleError::Io(_))
        ));
    }

    #[test]
    fn read_metadata() {
        let record = &read(HYPSO1, OnError::Raise).unwrap()[0];
        assert_eq!(record.sat_name, "HYPSO 1");
        assert_eq!(record.norad_id, "51053");
        assert_eq!(record.classification, "Unclassified");
        assert_eq!(record.launch_date, Some(UtcDateTime::midnight(2022, 1, 1)));
        assert_eq!(record.launch_no, 2);
        assert_eq!(record.pol, "BX ");
        assert_eq!(record.elem_set_no, 999);
        assert_eq!(record.rev_at_epoch, 20740.0);
        assert_eq!(record.n_dot, 0.00028852);
        assert_eq!(record.n_dot_dot, 0.0);
        assert!((record.b_star - 0.00056053).abs() < 1e-18);
        // Upstream `hypso1tleEpoch`.
        assert_eq!(
            record.tle_epoch,
            UtcDateTime::from_day_of_year(2025, 279.47924866)
        );
    }

    /// Splits a TLE line 2 on whitespace into floats (upstream's field
    /// comparison).
    fn fields(line: &str) -> Vec<f64> {
        line.split_whitespace()
            .map(|field| field.parse().expect("numeric field"))
            .collect()
    }

    /// The line 0, 1 and 2 comparisons of upstream's write tests, line 2's
    /// eccentricity within `eccentricity_tolerance` (in units of 1e-7).
    fn compare_written(generated: &str, expected: &str, eccentricity_tolerance: f64) {
        let generated = lines_of(generated);
        let expected = lines_of(expected);
        assert_eq!(generated[0], expected[0], "line 0");
        assert_eq!(generated[1], expected[1], "line 1");
        let mut failures = Vec::new();
        for (index, (wanted, actual)) in fields(&expected[2])
            .into_iter()
            .zip(fields(&generated[2]))
            .enumerate()
        {
            let tolerance = match index {
                4 => eccentricity_tolerance,
                5 | 6 => A_TOL_DEG_ROUNDTRIP,
                _ => A_TOL_ROUNDTRIP,
            };
            if (wanted - actual).abs() > tolerance {
                failures.push(format!(
                    "line 2 field {index}: expected {wanted}, got {actual}"
                ));
            }
        }
        assert!(
            failures.is_empty(),
            "{failures:#?}\n{}\n{}",
            generated[2],
            expected[2]
        );
    }

    /// Upstream `test_write_tle`.
    #[test]
    fn write_hypso1() {
        let epoch = UtcDateTime::midnight(2025, 1, 1)
            .plus_microseconds(((279.47924866_f64 - 1.0) * 86_400e6).round() as i64);
        let mut data = TleData::new(hypso1_elements(), epoch);
        data.sat_name = "HYPSO 1".to_string();
        data.norad_id = "51053".to_string();
        data.launch_date = Some(UtcDateTime::midnight(2022, 1, 13));
        data.launch_no = 2;
        data.pol = "BX".to_string();
        data.n_dot = 0.00028852;
        data.n_dot_dot = 0.0;
        data.b_star = 0.00056053;
        let generated = generate_tle(&data).expect("writable TLE");
        compare_written(&generated, HYPSO1, A_TOL_ECC_ROUNDTRIP);
    }

    /// Upstream `test_read_write_tle`.
    #[test]
    fn read_write_hypso1() {
        let record = &read(HYPSO1, OnError::Raise).unwrap()[0];
        let generated = generate_tle(record).expect("writable TLE");
        compare_written(&generated, HYPSO1, A_TOL_ROUNDTRIP * 1e7);
    }

    #[test]
    fn write_needs_a_launch_date_and_a_tle_catalog_number() {
        let mut data = TleData::new(hypso1_elements(), UtcDateTime::midnight(2025, 10, 6));
        assert!(generate_tle(&data).is_err());
        data.launch_date = Some(UtcDateTime::midnight(2022, 1, 1));
        data.norad_id = "100000".to_string();
        let error = generate_tle(&data).unwrap_err().to_string();
        assert!(error.contains("cannot represent catalog number"), "{error}");
    }

    #[test]
    fn exponential_fields_round_trip() {
        assert_eq!(tle_exponential_string(0.00056053), " 56053-3");
        assert_eq!(tle_exponential_string(0.0), " 00000-0");
        // Upstream writes a zero exponent as "-0".
        assert_eq!(tle_exponential_string(-0.11697), "-11697-0");
        assert_eq!(tle_exponential_field(" 56053-3").unwrap(), 0.56053e-3);
        assert_eq!(tle_exponential_field("-11697+0").unwrap(), -0.11697);
        assert_eq!(first_derivative_field("-.00044498").unwrap(), -0.00044498);
    }

    fn hypso_lines() -> (String, String) {
        let lines = lines_of(HYPSO1);
        (lines[1].clone(), lines[2].clone())
    }

    fn expect_record_error(text: &str, needle: &str) {
        let error = read(text, OnError::Raise).unwrap_err();
        assert!(matches!(error, TleError::Record(_)), "{error}");
        assert!(error.to_string().contains(needle), "{error}");
    }

    /// Upstream `test_read_tle_rejects_alpha5_like_id`.
    #[test]
    fn rejects_alpha5_like_id() {
        let (line1, line2) = hypso_lines();
        let line1 = format!("{}A1053{}", columns(&line1, 0, 2), columns(&line1, 7, 69));
        let line2 = format!("{}A1053{}", columns(&line2, 0, 2), columns(&line2, 7, 69));
        expect_record_error(&modified_hypso(Some(line1), Some(line2)), "Alpha-5-like");
    }

    /// Upstream `test_read_tle_rejects_non_numeric_id_field`.
    #[test]
    fn rejects_non_numeric_id_field() {
        let (line1, line2) = hypso_lines();
        let line1 = format!("{}12-53{}", columns(&line1, 0, 2), columns(&line1, 7, 69));
        let line2 = format!("{}12-53{}", columns(&line2, 0, 2), columns(&line2, 7, 69));
        expect_record_error(&modified_hypso(Some(line1), Some(line2)), "non-numeric");
    }

    /// Upstream `test_read_tle_rejects_catalog_overflow`.
    #[test]
    fn rejects_catalog_overflow() {
        let (line1, line2) = hypso_lines();
        let overflow = |line: &str| {
            let mut characters: Vec<char> = line.chars().collect();
            characters[7] = '1';
            characters.into_iter().collect::<String>()
        };
        expect_record_error(
            &modified_hypso(Some(overflow(&line1)), Some(overflow(&line2))),
            "overflow",
        );
    }

    /// Upstream `test_read_tle_blank_international_designator`.
    #[test]
    fn blank_international_designator() {
        let (line1, _) = hypso_lines();
        let line1 = with_checksum(&format!(
            "{}{}{}",
            columns(&line1, 0, 9),
            " ".repeat(8),
            columns(&line1, 17, 69)
        ));
        let reference = read(HYPSO1, OnError::Raise).unwrap().remove(0);
        let records = read(&modified_hypso(Some(line1), None), OnError::Raise).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].launch_date, None);
        assert_eq!(records[0].launch_no, 1);
        assert_eq!(records[0].pol, "A");
        assert_eq!(records[0].norad_id, reference.norad_id);
        assert_eq!(six(&records[0].oe), six(&reference.oe));
    }

    fn with_ephemeris_type(ephemeris_type: &str) -> String {
        let (line1, _) = hypso_lines();
        let line1 = with_checksum(&format!(
            "{}{ephemeris_type}{}",
            columns(&line1, 0, 62),
            columns(&line1, 63, 69)
        ));
        modified_hypso(Some(line1), None)
    }

    /// Upstream `test_read_tle_rejects_non_sgp4_ephemeris_type`.
    #[test]
    fn rejects_non_sgp4_ephemeris_types() {
        for ephemeris_type in ["4", "1", "3"] {
            expect_record_error(
                &with_ephemeris_type(ephemeris_type),
                "unsupported ephemeris type",
            );
        }
    }

    /// Upstream `test_read_tle_accepts_sgp4_ephemeris_types`.
    #[test]
    fn accepts_sgp4_ephemeris_types() {
        let reference = read(HYPSO1, OnError::Raise).unwrap().remove(0);
        for ephemeris_type in ["0", "2", " "] {
            let records = read(&with_ephemeris_type(ephemeris_type), OnError::Raise).unwrap();
            assert_eq!(
                six(&records[0].oe),
                six(&reference.oe),
                "type {ephemeris_type:?}"
            );
        }
    }

    const ON_ERROR: [OnError; 2] = [OnError::Raise, OnError::Skip];

    /// Upstream `test_read_tle_name_lines_starting_with_a_digit`: every
    /// record's name and catalog ID survive names with element-line
    /// prefixes (the default policy is `Raise`, covered by its entry).
    #[test]
    fn name_lines_starting_with_a_digit() {
        let names: [&[&str]; 12] = [
            &["1KUNS-PF"],
            &["HYPSO-1", "2026-195B"],
            &["2026-195B", "1KUNS-PF"],
            &["1 TEST"],
            &["2 TEST"],
            &["1 TEST", "2 TEST"],
            &["HYPSO-1", "1 TEST", "2 TEST"],
            &["1 12345U", "2 67890"],
            &["1 OBJECT 12345", "2 PANEL 123.0000"],
            &["1 OBJECT 12345678 PAYLOAD", "2 PANEL 123.0000 SIDE A"],
            &["1 a0001", "2 12-53"],
            &["1 5153U MODEL", "2 5153 CATALOG"],
        ];
        let (line1, line2) = hypso_lines();
        let reference = read(HYPSO1, OnError::Raise).unwrap().remove(0);
        for on_error in ON_ERROR {
            for set in names {
                let catalog_ids: Vec<String> = (0..set.len())
                    .map(|index| format!("{:05}", 90001 + index))
                    .collect();
                let mut lines = Vec::new();
                for (name, catalog_id) in set.iter().zip(&catalog_ids) {
                    lines.push(name.to_string());
                    lines.push(with_checksum(&format!(
                        "{}{catalog_id}{}",
                        columns(&line1, 0, 2),
                        columns(&line1, 7, 69)
                    )));
                    lines.push(with_checksum(&format!(
                        "{}{catalog_id}{}",
                        columns(&line2, 0, 2),
                        columns(&line2, 7, 69)
                    )));
                }
                let records = read(&(lines.join("\n") + "\n"), on_error)
                    .unwrap_or_else(|error| panic!("{set:?}: {error}"));
                assert_eq!(
                    records
                        .iter()
                        .map(|record| record.sat_name.as_str())
                        .collect::<Vec<_>>(),
                    set,
                    "{set:?}"
                );
                assert_eq!(
                    records
                        .iter()
                        .map(|record| record.norad_id.clone())
                        .collect::<Vec<_>>(),
                    catalog_ids
                );
                for record in &records {
                    assert_eq!(six(&record.oe), six(&reference.oe));
                }
            }
        }
    }

    /// Upstream `test_read_tle_2le_optional_title`.
    #[test]
    fn two_line_sets_with_an_optional_title() {
        let titles = [
            None,
            Some("CATALOG"),
            Some("1 TEST"),
            Some("2 TEST"),
            Some("1 SATELLITE GROUP RECENT CATALOG 2026"),
            Some("2 SATELLITE LIST CATALOGS 2026"),
        ];
        let (line1, line2) = hypso_lines();
        for on_error in ON_ERROR {
            for title in titles {
                let mut lines: Vec<String> = title.into_iter().map(str::to_string).collect();
                for catalog_id in ["90001", "90002"] {
                    lines.push(with_checksum(&format!(
                        "{}{catalog_id}{}",
                        columns(&line1, 0, 2),
                        columns(&line1, 7, 69)
                    )));
                    lines.push(with_checksum(&format!(
                        "{}{catalog_id}{}",
                        columns(&line2, 0, 2),
                        columns(&line2, 7, 69)
                    )));
                }
                let records = read(&(lines.join("\n") + "\n"), on_error)
                    .unwrap_or_else(|error| panic!("{title:?}: {error}"));
                assert_eq!(
                    records
                        .iter()
                        .map(|record| record.norad_id.as_str())
                        .collect::<Vec<_>>(),
                    ["90001", "90002"],
                    "{title:?}"
                );
                assert!(records.iter().all(|record| record.sat_name.is_empty()));
            }
        }
    }

    /// Upstream `test_read_tle_incomplete_2le_is_not_a_named_record`.
    #[test]
    fn incomplete_2le_is_not_a_named_record() {
        let (line1, line2) = hypso_lines();
        for on_error in ON_ERROR {
            for missing in 0..4 {
                let mut lines = vec![line1.clone(), line2.clone(), line1.clone(), line2.clone()];
                lines.remove(missing);
                let error = read(&(lines.join("\n") + "\n"), on_error).unwrap_err();
                let message = error.to_string();
                assert!(
                    message.contains("orphan") || message.contains("must be equal"),
                    "missing line {missing}: {message}"
                );
            }
        }
    }

    #[test]
    fn skip_policy_keeps_the_readable_sets() {
        let (line1, line2) = hypso_lines();
        let broken = format!("{}0", columns(&line2, 0, 68));
        let text = format!("GOOD\n{line1}\n{line2}\nBAD\n{line1}\n{broken}\n");
        assert!(read(&text, OnError::Raise).is_err());
        let records = read(&text, OnError::Skip).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].sat_name, "GOOD");
        // Every set failing returns the first error.
        let text = format!("BAD\n{line1}\n{broken}\n");
        let error = read(&text, OnError::Skip).unwrap_err();
        assert!(error.to_string().contains("checksum mismatch"), "{error}");
    }

    #[test]
    fn dates_follow_python_datetime() {
        let epoch = UtcDateTime::from_day_of_year(2024, 366.5);
        assert_eq!(epoch, UtcDateTime::new(2024, 12, 31, 12, 0, 0, 0));
        assert_eq!(epoch.day_of_year(), 366);
        assert_eq!(
            UtcDateTime::new(2025, 1, 1, 0, 0, 0, 0).plus_microseconds(-1),
            UtcDateTime::new(2024, 12, 31, 23, 59, 59, 999_999)
        );
        // J2000: 2000-01-01 12:00 is JD 2451545.0.
        assert_eq!(
            UtcDateTime::new(2000, 1, 1, 12, 0, 0, 0).julian_date(),
            2_451_545.0
        );
        assert_eq!(
            epoch_field(&UtcDateTime::from_day_of_year(2025, 279.47924866)),
            "25279.47924866"
        );
    }

    #[test]
    fn teme_to_gcrf_is_a_small_rotation() {
        let epoch = UtcDateTime::new(2025, 10, 6, 11, 30, 7, 84_224);
        let r = Vector3::new(7000e3, 0.0, 0.0);
        let (r_gcrf, _) = teme_to_gcrf(r, Vector3::zeros(), &epoch);
        assert!((r_gcrf.norm() - r.norm()).abs() < 1e-6);
        // Precession over 25.8 years: about 0.36 deg.
        let angle = r.angle(&r_gcrf).to_degrees();
        assert!((0.3..0.4).contains(&angle), "{angle}");
    }
}
