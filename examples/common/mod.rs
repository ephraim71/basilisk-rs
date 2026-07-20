#![allow(dead_code)]

use std::path::PathBuf;

use basilisk_rs::telemetry::TelemetryField;
use nalgebra::Vector3;

// Value used by the Basilisk build that produced `csvs.zip`.
pub const MU_EARTH_M3PS2: f64 = 3.986_004_36e14;
pub const EARTH_EQUATORIAL_RADIUS_M: f64 = 6_378_136.6;
pub const NANOS_PER_SECOND: u64 = 1_000_000_000;

pub const fn seconds(value: u64) -> u64 {
    value * NANOS_PER_SECOND
}

pub fn rpm_to_radps(rpm: f64) -> f64 {
    rpm * 2.0 * std::f64::consts::PI / 60.0
}

pub fn scenario_output_path(file_name: &str) -> PathBuf {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("examples/output/scenarios")
        .join(file_name);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).expect("failed to create scenario output directory");
    }
    match std::fs::remove_file(&path) {
        Ok(()) => {}
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => panic!("failed to remove previous {}: {error}", path.display()),
    }
    path
}

/// Classical orbital elements to inertial Cartesian state, matching
/// Basilisk's `orbitalMotion.elem2rv` convention.
#[allow(clippy::too_many_arguments)]
pub fn elem2rv(
    mu_m3ps2: f64,
    semimajor_axis_m: f64,
    eccentricity: f64,
    inclination_rad: f64,
    raan_rad: f64,
    argument_of_periapsis_rad: f64,
    true_anomaly_rad: f64,
) -> (Vector3<f64>, Vector3<f64>) {
    let p = semimajor_axis_m * (1.0 - eccentricity * eccentricity);
    let radius_m = p / (1.0 + eccentricity * true_anomaly_rad.cos());

    let (sin_i, cos_i) = inclination_rad.sin_cos();
    let (sin_raan, cos_raan) = raan_rad.sin_cos();
    let (sin_aop, cos_aop) = argument_of_periapsis_rad.sin_cos();
    let p_hat = Vector3::new(
        cos_raan * cos_aop - sin_raan * sin_aop * cos_i,
        sin_raan * cos_aop + cos_raan * sin_aop * cos_i,
        sin_aop * sin_i,
    );
    let q_hat = Vector3::new(
        -cos_raan * sin_aop - sin_raan * cos_aop * cos_i,
        -sin_raan * sin_aop + cos_raan * cos_aop * cos_i,
        cos_aop * sin_i,
    );

    let (sin_f, cos_f) = true_anomaly_rad.sin_cos();
    let position_m = radius_m * (cos_f * p_hat + sin_f * q_hat);
    let velocity_mps = (mu_m3ps2 / p).sqrt() * (-sin_f * p_hat + (eccentricity + cos_f) * q_hat);
    (position_m, velocity_mps)
}

pub fn vector_fields(prefix: &str, vector: Vector3<f64>) -> Vec<TelemetryField> {
    vector
        .iter()
        .enumerate()
        .map(|(index, value)| TelemetryField {
            path: format!("{prefix}_{index}"),
            value: *value,
        })
        .collect()
}

pub fn array_fields(prefix: &str, values: &[f64]) -> Vec<TelemetryField> {
    values
        .iter()
        .enumerate()
        .map(|(index, value)| TelemetryField {
            path: format!("{prefix}_{index}"),
            value: *value,
        })
        .collect()
}
