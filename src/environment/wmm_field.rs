//! World Magnetic Model environment module.
//!
//! The implementation follows the spherical-harmonic formulation used by the
//! current Basilisk C++ WMM module. WMM2025 coefficients are embedded,
//! evaluated in `f64`, and advanced from the 2025.0 model epoch with their
//! secular-variation terms. Dates before 2025 are deliberately supported by
//! negative extrapolation; the Basilisk momentum-management reference scenario
//! starts in June 2019.

use hifitime::{Duration as HiDuration, Epoch};
use nalgebra::{Matrix3, Vector3};
use time::{Date, Duration as CivilDuration, Month, PrimitiveDateTime};

use super::wmm_coefficients::{WMM_COEFFICIENTS, WMM_MAX_DEGREE, WMM_MODEL_EPOCH};
use crate::messages::{Input, MagneticFieldMsg, Output, PlanetStateMsg, SpacecraftStateMsg};
use crate::{Module, SimulationContext};

const WMM_REFERENCE_RADIUS_KM: f64 = 6_371.2;
const TRIANGULAR_TERM_COUNT: usize = (WMM_MAX_DEGREE + 1) * (WMM_MAX_DEGREE + 2) / 2;

#[derive(Clone, Debug)]
pub struct WmmFieldConfig {
    pub name: String,
}

impl Default for WmmFieldConfig {
    fn default() -> Self {
        Self {
            name: "wmm2025".to_string(),
        }
    }
}

/// Evaluates WMM2025 at one spacecraft and publishes the field in inertial
/// frame components, in tesla.
///
/// `input_planet_msg` is optional. With no planet message, or with a planet
/// message whose `orientation` is `None`, the planet is treated as centered at
/// the inertial origin with identity inertial-to-fixed orientation, matching
/// Basilisk's magnetic-field base-class fallback. A connected planet message
/// still supplies its inertial position when its orientation is absent.
#[derive(Clone, Debug)]
pub struct WmmField {
    pub config: WmmFieldConfig,
    pub input_state_msg: Input<SpacecraftStateMsg>,
    pub input_planet_msg: Input<PlanetStateMsg>,
    pub output_magnetic_field_msg: Output<MagneticFieldMsg>,
}

impl WmmField {
    pub fn new(config: WmmFieldConfig) -> Self {
        Self {
            config,
            input_state_msg: Input::default(),
            input_planet_msg: Input::default(),
            output_magnetic_field_msg: Output::default(),
        }
    }

    fn compute_magnetic_field(
        &self,
        state: &SpacecraftStateMsg,
        current_epoch: Epoch,
        current_sim_nanos: u64,
    ) -> Vector3<f64> {
        let planet = self.input_planet_msg.read();
        let inertial_to_fixed = planet
            .orientation
            .as_ref()
            .map(|orientation| orientation.inertial_to_fixed)
            .unwrap_or_else(Matrix3::identity);
        let relative_position_inertial_m = state.position_m - planet.position_inertial_m;
        let relative_position_fixed_m = inertial_to_fixed * relative_position_inertial_m;
        let orbit_radius_m = relative_position_fixed_m.norm();

        if !orbit_radius_m.is_finite() || orbit_radius_m <= f64::EPSILON {
            return Vector3::zeros();
        }

        // Basilisk's current WMM module evaluates the harmonics directly in
        // geocentric spherical coordinates. In particular, it does not pass
        // through WGS-84 geodetic coordinates or rotate the spherical field to
        // a geodetic tangent frame.
        let latitude_rad = (relative_position_fixed_m.z / orbit_radius_m)
            .clamp(-1.0, 1.0)
            .asin();
        let longitude_rad = relative_position_fixed_m
            .y
            .atan2(relative_position_fixed_m.x);
        let decimal_year = decimal_year_from_simulation_time(current_epoch, current_sim_nanos);
        let field_ned_t =
            evaluate_wmm_ned_t(latitude_rad, longitude_rad, orbit_radius_m, decimal_year);
        let field_fixed_t = ned_to_planet_fixed(latitude_rad, longitude_rad, field_ned_t);

        inertial_to_fixed.transpose() * field_fixed_t
    }
}

impl Module for WmmField {
    fn init(&mut self) {
        self.output_magnetic_field_msg
            .write(MagneticFieldMsg::default());
    }

    fn update(&mut self, context: &SimulationContext) {
        let state = self.input_state_msg.read();
        let magnetic_field_inertial_t =
            self.compute_magnetic_field(&state, context.current_epoch, context.current_sim_nanos);
        self.output_magnetic_field_msg.write(MagneticFieldMsg {
            magnetic_field_inertial_t,
        });
    }
}

fn evaluate_wmm_ned_t(
    geocentric_latitude_rad: f64,
    longitude_rad: f64,
    spherical_radius_m: f64,
    decimal_year: f64,
) -> Vector3<f64> {
    evaluate_wmm_ned_nt(
        geocentric_latitude_rad,
        longitude_rad,
        spherical_radius_m * 1.0e-3,
        decimal_year,
    ) * 1.0e-9
}

/// Evaluate the WMM harmonics in geocentric spherical coordinates, returning
/// spherical north/east/down components in nT.
fn evaluate_wmm_ned_nt(
    geocentric_latitude_rad: f64,
    longitude_rad: f64,
    spherical_radius_km: f64,
    decimal_year: f64,
) -> Vector3<f64> {
    let (legendre, legendre_derivative) =
        schmidt_quasi_normalized_legendre(geocentric_latitude_rad.sin());
    let mut relative_radius_power = [0.0; WMM_MAX_DEGREE + 1];
    let radius_ratio = WMM_REFERENCE_RADIUS_KM / spherical_radius_km;
    relative_radius_power[0] = radius_ratio * radius_ratio;
    let mut next_radius_power = relative_radius_power[0];
    for radius_power in relative_radius_power.iter_mut().skip(1) {
        next_radius_power *= radius_ratio;
        *radius_power = next_radius_power;
    }

    let mut cos_m_longitude = [0.0; WMM_MAX_DEGREE + 1];
    let mut sin_m_longitude = [0.0; WMM_MAX_DEGREE + 1];
    cos_m_longitude[0] = 1.0;
    if WMM_MAX_DEGREE > 0 {
        cos_m_longitude[1] = longitude_rad.cos();
        sin_m_longitude[1] = longitude_rad.sin();
    }
    for m in 2..=WMM_MAX_DEGREE {
        cos_m_longitude[m] = cos_m_longitude[m - 1] * cos_m_longitude[1]
            - sin_m_longitude[m - 1] * sin_m_longitude[1];
        sin_m_longitude[m] = cos_m_longitude[m - 1] * sin_m_longitude[1]
            + sin_m_longitude[m - 1] * cos_m_longitude[1];
    }

    let elapsed_model_years = decimal_year - WMM_MODEL_EPOCH;
    let mut timed_g = [0.0; TRIANGULAR_TERM_COUNT];
    let mut timed_h = [0.0; TRIANGULAR_TERM_COUNT];
    for coefficient in WMM_COEFFICIENTS {
        let index = triangular_index(coefficient.n, coefficient.m);
        timed_g[index] = coefficient.g_nt + elapsed_model_years * coefficient.g_dot_nt_per_year;
        timed_h[index] = coefficient.h_nt + elapsed_model_years * coefficient.h_dot_nt_per_year;
    }

    let mut north_spherical_nt = 0.0;
    let mut east_spherical_nt = 0.0;
    let mut down_spherical_nt = 0.0;
    for (n, &radius_power) in relative_radius_power
        .iter()
        .enumerate()
        .take(WMM_MAX_DEGREE + 1)
        .skip(1)
    {
        for m in 0..=n {
            let index = triangular_index(n, m);
            let longitude_coefficient =
                timed_g[index] * cos_m_longitude[m] + timed_h[index] * sin_m_longitude[m];

            down_spherical_nt -=
                radius_power * longitude_coefficient * (n + 1) as f64 * legendre[index];
            east_spherical_nt += radius_power
                * (timed_g[index] * sin_m_longitude[m] - timed_h[index] * cos_m_longitude[m])
                * m as f64
                * legendre[index];
            north_spherical_nt -= radius_power * longitude_coefficient * legendre_derivative[index];
        }
    }

    let cos_geocentric_latitude = geocentric_latitude_rad.cos();
    if cos_geocentric_latitude.abs() > 1.0e-10 {
        east_spherical_nt /= cos_geocentric_latitude;
    } else {
        east_spherical_nt = east_field_at_geographic_pole(
            geocentric_latitude_rad,
            &relative_radius_power,
            &sin_m_longitude,
            &cos_m_longitude,
            &timed_g,
            &timed_h,
        );
    }

    Vector3::new(north_spherical_nt, east_spherical_nt, down_spherical_nt)
}

fn schmidt_quasi_normalized_legendre(
    sin_geocentric_latitude: f64,
) -> ([f64; TRIANGULAR_TERM_COUNT], [f64; TRIANGULAR_TERM_COUNT]) {
    let mut legendre = [0.0; TRIANGULAR_TERM_COUNT];
    let mut derivative = [0.0; TRIANGULAR_TERM_COUNT];
    legendre[0] = 1.0;
    let cos_geocentric_latitude =
        ((1.0 - sin_geocentric_latitude) * (1.0 + sin_geocentric_latitude)).sqrt();

    // First compute Gauss-normalized associated Legendre functions.
    for n in 1..=WMM_MAX_DEGREE {
        for m in 0..=n {
            let index = triangular_index(n, m);
            if n == m {
                let previous = triangular_index(n - 1, m - 1);
                legendre[index] = cos_geocentric_latitude * legendre[previous];
                derivative[index] = cos_geocentric_latitude * derivative[previous]
                    + sin_geocentric_latitude * legendre[previous];
            } else if n == 1 {
                let previous = triangular_index(n - 1, m);
                legendre[index] = sin_geocentric_latitude * legendre[previous];
                derivative[index] = sin_geocentric_latitude * derivative[previous]
                    - cos_geocentric_latitude * legendre[previous];
            } else {
                let previous = triangular_index(n - 1, m);
                if m > n - 2 {
                    legendre[index] = sin_geocentric_latitude * legendre[previous];
                    derivative[index] = sin_geocentric_latitude * derivative[previous]
                        - cos_geocentric_latitude * legendre[previous];
                } else {
                    let two_back = triangular_index(n - 2, m);
                    let k =
                        (((n - 1) * (n - 1) - m * m) as f64) / (((2 * n - 1) * (2 * n - 3)) as f64);
                    legendre[index] =
                        sin_geocentric_latitude * legendre[previous] - k * legendre[two_back];
                    derivative[index] = sin_geocentric_latitude * derivative[previous]
                        - cos_geocentric_latitude * legendre[previous]
                        - k * derivative[two_back];
                }
            }
        }
    }

    // Convert Gauss normalization to Schmidt quasi-normalization. The NOAA
    // derivative convention is with respect to latitude, hence the sign flip.
    let mut schmidt = [0.0; TRIANGULAR_TERM_COUNT];
    schmidt[0] = 1.0;
    for n in 1..=WMM_MAX_DEGREE {
        let zonal_index = triangular_index(n, 0);
        let previous_zonal_index = triangular_index(n - 1, 0);
        schmidt[zonal_index] = schmidt[previous_zonal_index] * (2 * n - 1) as f64 / n as f64;
        for m in 1..=n {
            let index = triangular_index(n, m);
            let previous = triangular_index(n, m - 1);
            let order_factor = if m == 1 { 2 } else { 1 };
            schmidt[index] =
                schmidt[previous] * (((n - m + 1) * order_factor) as f64 / (n + m) as f64).sqrt();
        }
    }
    for n in 1..=WMM_MAX_DEGREE {
        for m in 0..=n {
            let index = triangular_index(n, m);
            legendre[index] *= schmidt[index];
            derivative[index] *= -schmidt[index];
        }
    }

    (legendre, derivative)
}

fn east_field_at_geographic_pole(
    geocentric_latitude_rad: f64,
    relative_radius_power: &[f64; WMM_MAX_DEGREE + 1],
    sin_m_longitude: &[f64; WMM_MAX_DEGREE + 1],
    cos_m_longitude: &[f64; WMM_MAX_DEGREE + 1],
    timed_g: &[f64; TRIANGULAR_TERM_COUNT],
    timed_h: &[f64; TRIANGULAR_TERM_COUNT],
) -> f64 {
    let mut pole_legendre = [0.0; WMM_MAX_DEGREE + 1];
    pole_legendre[0] = 1.0;
    let mut previous_zonal_schmidt = 1.0;
    let sin_latitude = geocentric_latitude_rad.sin();
    let mut east_nt = 0.0;

    for n in 1..=WMM_MAX_DEGREE {
        let zonal_schmidt = previous_zonal_schmidt * (2 * n - 1) as f64 / n as f64;
        let order_one_schmidt = zonal_schmidt * (2.0 * n as f64 / (n + 1) as f64).sqrt();
        previous_zonal_schmidt = zonal_schmidt;
        if n == 1 {
            pole_legendre[n] = pole_legendre[n - 1];
        } else {
            let k = (((n - 1) * (n - 1) - 1) as f64) / (((2 * n - 1) * (2 * n - 3)) as f64);
            pole_legendre[n] = sin_latitude * pole_legendre[n - 1] - k * pole_legendre[n - 2];
        }

        let index = triangular_index(n, 1);
        east_nt += relative_radius_power[n]
            * (timed_g[index] * sin_m_longitude[1] - timed_h[index] * cos_m_longitude[1])
            * pole_legendre[n]
            * order_one_schmidt;
    }

    east_nt
}

fn triangular_index(n: usize, m: usize) -> usize {
    n * (n + 1) / 2 + m
}

/// Reproduce Basilisk C++'s civil-time conversion: round the epoch-message
/// seconds once, round elapsed simulation seconds once, then normalize the
/// resulting Gregorian calendar before computing the fractional year.
fn decimal_year_from_simulation_time(current_epoch: Epoch, current_sim_nanos: u64) -> f64 {
    let start_epoch =
        current_epoch - HiDuration::from_total_nanoseconds(i128::from(current_sim_nanos));
    let (year, month, day, hour, minute, second, nanos) = start_epoch.to_gregorian_utc();
    let date = Date::from_calendar_date(
        year,
        Month::try_from(month).expect("invalid Gregorian month from hifitime"),
        day,
    )
    .expect("invalid Gregorian date from hifitime");
    let mut date_time = date
        .with_hms(hour, minute, second)
        .expect("invalid Gregorian time from hifitime");
    if nanos >= 500_000_000 {
        date_time = date_time
            .checked_add(CivilDuration::SECOND)
            .expect("rounded WMM start epoch is outside the supported calendar");
    }

    let rounded_sim_seconds = (u128::from(current_sim_nanos) + 500_000_000) / 1_000_000_000;
    let rounded_sim_seconds = i64::try_from(rounded_sim_seconds)
        .expect("WMM simulation duration exceeds the supported calendar");
    date_time = date_time
        .checked_add(CivilDuration::seconds(rounded_sim_seconds))
        .expect("WMM epoch is outside the supported calendar");

    decimal_year_from_date_time(date_time)
}

fn decimal_year_from_date_time(date_time: PrimitiveDateTime) -> f64 {
    let year = date_time.year();
    let days_in_year = if is_gregorian_leap_year(year) {
        366.0
    } else {
        365.0
    };
    let elapsed_days = date_time.ordinal() as f64 - 1.0
        + date_time.hour() as f64 / 24.0
        + date_time.minute() as f64 / (24.0 * 60.0)
        + date_time.second() as f64 / (24.0 * 60.0 * 60.0);

    year as f64 + elapsed_days / days_in_year
}

fn is_gregorian_leap_year(year: i32) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

fn ned_to_planet_fixed(
    latitude_rad: f64,
    longitude_rad: f64,
    field_ned_t: Vector3<f64>,
) -> Vector3<f64> {
    let sin_latitude = latitude_rad.sin();
    let cos_latitude = latitude_rad.cos();
    let sin_longitude = longitude_rad.sin();
    let cos_longitude = longitude_rad.cos();

    let north = Vector3::new(
        -sin_latitude * cos_longitude,
        -sin_latitude * sin_longitude,
        cos_latitude,
    );
    let east = Vector3::new(-sin_longitude, cos_longitude, 0.0);
    let down = Vector3::new(
        -cos_latitude * cos_longitude,
        -cos_latitude * sin_longitude,
        -sin_latitude,
    );

    north * field_ned_t.x + east * field_ned_t.y + down * field_ned_t.z
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::messages::{Output, PlanetOrientation};
    use crate::simulation::Simulation;

    const REFERENCE_EPOCH: f64 = 2019.486_116_818_873_6;
    const REFERENCE_RADIUS_KM: f64 = 6_778.14;
    const REFERENCE_LONGITUDE_RAD: f64 = std::f64::consts::PI / 3.0;
    const REFERENCE_FIELD_INERTIAL_T: [f64; 3] = [
        5.776_798_033_004_385e-6,
        6.169_997_614_858_818e-6,
        2.923_781_212_797_075_3e-5,
    ];

    #[test]
    fn matches_basilisk_cpp_wmm2025_spherical_reference() {
        // Basilisk C++ output using WMM2025.COF at the initial state of
        // scenarioMtbMomentumManagement. This is the spherical N/E/D vector
        // before Basilisk rotates it into inertial components.
        let expected_nt = Vector3::new(
            29_237.812_127_970_752,
            -1_917.855_041_684_363_4,
            -8_231.773_692_259_323,
        );
        let actual_nt = evaluate_wmm_ned_nt(
            0.0,
            REFERENCE_LONGITUDE_RAD,
            REFERENCE_RADIUS_KM,
            REFERENCE_EPOCH,
        );

        assert!((actual_nt - expected_nt).norm() < 1.0e-6);
    }

    #[test]
    fn extrapolates_before_the_model_epoch() {
        let before = evaluate_wmm_ned_nt(
            45_f64.to_radians(),
            REFERENCE_LONGITUDE_RAD,
            REFERENCE_RADIUS_KM,
            2024.0,
        );
        let at_epoch = evaluate_wmm_ned_nt(
            45_f64.to_radians(),
            REFERENCE_LONGITUDE_RAD,
            REFERENCE_RADIUS_KM,
            2025.0,
        );
        let after = evaluate_wmm_ned_nt(
            45_f64.to_radians(),
            REFERENCE_LONGITUDE_RAD,
            REFERENCE_RADIUS_KM,
            2026.0,
        );

        // WMM time adjustment is linear in every Gauss coefficient, so each
        // field component must be symmetric around the model epoch.
        assert!((before + after - 2.0 * at_epoch).norm() < 1.0e-9);
        assert!(before.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn decimal_year_matches_basilisk_subday_and_rounding_behavior() {
        let epoch = Epoch::from_gregorian_utc(2020, 7, 2, 12, 0, 0, 0);
        assert!(
            (decimal_year_from_simulation_time(epoch, 0) - 2020.501_366_120_218_6).abs() < 1.0e-12
        );

        let reference_epoch = Epoch::from_gregorian_utc(2019, 6, 27, 10, 23, 0, 0);
        assert!(
            (decimal_year_from_simulation_time(reference_epoch, 0) - REFERENCE_EPOCH).abs()
                < 1.0e-12
        );

        // C++ rounds both the epoch message's seconds and elapsed simulation
        // seconds before normalizing the calendar with mktime().
        let fractional_start = Epoch::from_gregorian_utc(2019, 12, 31, 23, 59, 59, 600_000_000);
        assert_eq!(
            decimal_year_from_simulation_time(fractional_start, 0),
            2020.0
        );
        let half_second = 500_000_000;
        assert_eq!(
            decimal_year_from_simulation_time(
                fractional_start + HiDuration::from_total_nanoseconds(half_second),
                half_second as u64,
            ),
            2020.0 + 1.0 / (366.0 * 86_400.0)
        );
    }

    #[test]
    fn unconnected_planet_input_uses_identity_orientation() {
        let latitude = 0_f64;
        let longitude = REFERENCE_LONGITUDE_RAD;
        let radius = REFERENCE_RADIUS_KM * 1_000.0;
        let state = SpacecraftStateMsg {
            position_m: Vector3::new(
                radius * latitude.cos() * longitude.cos(),
                radius * latitude.cos() * longitude.sin(),
                radius * latitude.sin(),
            ),
            ..SpacecraftStateMsg::default()
        };
        let field = WmmField::new(WmmFieldConfig::default()).compute_magnetic_field(
            &state,
            Epoch::from_gregorian_utc(2019, 6, 27, 10, 23, 0, 0),
            0,
        );
        let expected_inertial_t = Vector3::from(REFERENCE_FIELD_INERTIAL_T);

        assert!((field - expected_inertial_t).norm() < 2.0e-16);
    }

    #[test]
    fn connected_planet_translation_and_rotation_are_applied() {
        let inertial_to_fixed = Matrix3::new(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        let planet_position = Vector3::new(1_000.0, 2_000.0, -1_000.0);
        let radius = REFERENCE_RADIUS_KM * 1_000.0;
        let fixed_position = Vector3::new(
            radius * REFERENCE_LONGITUDE_RAD.cos(),
            radius * REFERENCE_LONGITUDE_RAD.sin(),
            0.0,
        );
        let state_output = Output::new(SpacecraftStateMsg {
            position_m: planet_position + inertial_to_fixed.transpose() * fixed_position,
            ..SpacecraftStateMsg::default()
        });
        let planet_output = Output::new(PlanetStateMsg {
            position_inertial_m: planet_position,
            orientation: Some(PlanetOrientation {
                inertial_to_fixed,
                inertial_to_fixed_dot: Matrix3::zeros(),
            }),
            ..PlanetStateMsg::default()
        });
        let mut field = WmmField::new(WmmFieldConfig::default());
        let epoch = Epoch::from_gregorian_utc(2019, 6, 27, 10, 23, 0, 0);
        let sim = Simulation::new(epoch, false);
        sim.connect(&state_output, &mut field.input_state_msg);
        sim.connect(&planet_output, &mut field.input_planet_msg);
        let context = SimulationContext {
            current_sim_nanos: 0,
            current_epoch: epoch,
        };
        field.update(&context);

        let expected_inertial =
            inertial_to_fixed.transpose() * Vector3::from(REFERENCE_FIELD_INERTIAL_T);
        assert!(
            (field
                .output_magnetic_field_msg
                .read()
                .magnetic_field_inertial_t
                - expected_inertial)
                .norm()
                < 2.0e-16
        );
    }

    #[test]
    fn zero_radius_writes_a_finite_zero_field() {
        let mut field = WmmField::new(WmmFieldConfig::default());
        field.update(&SimulationContext {
            current_sim_nanos: 0,
            current_epoch: Epoch::from_gregorian_utc_at_midnight(2019, 1, 1),
        });
        assert_eq!(
            field
                .output_magnetic_field_msg
                .read()
                .magnetic_field_inertial_t,
            Vector3::zeros()
        );
    }
}
