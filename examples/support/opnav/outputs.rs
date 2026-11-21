//! What the OpNav scenarios record and check: CSV columns for upstream's
//! `log_outputs` recorders, and the truth upstream's `pull_outputs`
//! computes from the spacecraft state (the planet's true circle in the
//! image, the camera-frame position, the off-pointing).
//!
//! The scenario CSV holds the recorded histories; `examples/opnav_plotting.py`
//! computes upstream's derived quantities from them as `pull_outputs` does.
//! The limb points (up to 2000 per image) do not fit that table:
//! [`write_limb_points`] writes every valid limb's size, and the points of
//! the limbs upstream's `plot_limb` draws (every 30th), to
//! `<stem>_limb.csv`.

use std::path::PathBuf;

use basilisk_rs::messages::{OpNavLimbMsg, SpacecraftStateMsg};
use basilisk_rs::telemetry::CsvSourceConfig;
use basilisk_rs::utilities::linear_algebra::safe_acos;
use basilisk_rs::utilities::rigid_body_kinematics::mrp_to_dcm;
use nalgebra::{Matrix3, Vector3};

use super::common::{array_columns, vector_columns};
use super::dynamics::{CAMERA_FOV_DEG, CAMERA_MRP_CB, CAMERA_RESOLUTION_PX};
use super::grav_factory::Body;
use super::message_log::MessageLog;
use super::table_csv::write_table;

/// Priority of the limb-point log: right after the FSW process, so that it
/// samples each image's limb in the tick it is found.
pub const LIMB_LOG_PRIORITY: i32 = super::fsw::FSW_PRIORITY - 10;

/// Upstream `Rmars` \[m\] of `pull_outputs`.
pub const R_MARS_M: f64 = 3396.19e3;

fn named(prefix: &str, columns: Vec<(String, String)>) -> CsvSourceConfig {
    let config = CsvSourceConfig::columns(columns);
    if prefix.is_empty() {
        config
    } else {
        config.with_name(prefix)
    }
}

/// The nine entries of a 3x3 matrix field, row-major: `<output>_0` ...
/// `<output>_8`.
fn matrix3_columns(source: &str, output: &str) -> Vec<(String, String)> {
    matrix_columns(source, output, 3)
}

/// The `n x n` entries of a matrix field, row-major.
fn matrix_columns(source: &str, output: &str, n: usize) -> Vec<(String, String)> {
    let separator = if n > 9 { "_" } else { "" };
    (0..n * n)
        .map(|index| {
            (
                format!("{source}.m{}{separator}{}", index / n + 1, index % n + 1),
                format!("{output}_{index}"),
            )
        })
        .collect()
}

/// An `OpNavMsg` recorder (upstream `opnavMsg.recorder()`): `valid`,
/// `faultDetected`, `r_BN_N`, `r_BN_C` \[m\] and `covar_N` \[m^2\], the
/// fields upstream's figures use (the body-frame and camera-frame
/// covariances would add 21 columns to every FSW step).
pub fn opnav_columns(prefix: &str) -> CsvSourceConfig {
    let mut columns = vec![
        ("valid".to_string(), "valid".to_string()),
        ("fault_detected".to_string(), "faultDetected".to_string()),
    ];
    columns.extend(vector_columns("position_bn_n_m", "r_BN_N_m"));
    columns.extend(vector_columns("position_bn_c_m", "r_BN_C_m"));
    columns.extend(matrix3_columns("covar_n_m2", "covar_N_m2"));
    named(prefix, columns)
}

/// An `OpNavCirclesMsg` recorder (upstream `opnavCirclesMsg.recorder()`):
/// `valid`, the first circle's centre and radius \[pixels\] and the
/// uncertainty diagonal.
pub fn circles_columns() -> CsvSourceConfig {
    named(
        "",
        vec![
            ("valid".to_string(), "circles_valid".to_string()),
            (
                "circles_centers_px.0.x".to_string(),
                "circlesCenters_0".to_string(),
            ),
            (
                "circles_centers_px.0.y".to_string(),
                "circlesCenters_1".to_string(),
            ),
            (
                "circles_radii_px.0".to_string(),
                "circlesRadii_0".to_string(),
            ),
            ("uncertainty.m11".to_string(), "uncertainty_0".to_string()),
            ("uncertainty.m22".to_string(), "uncertainty_4".to_string()),
            ("uncertainty.m33".to_string(), "uncertainty_8".to_string()),
        ],
    )
}

/// An `OpNavFilterMsg` recorder (upstream `relativeOD.filtDataOutMsg`):
/// `state` \[m, m/s\], the covariance diagonal `covar_0`, `covar_7`, ...
/// `covar_35` (row-major indices; upstream's figures use only the
/// variances) and `postFitRes` \[m\].
pub fn od_filter_columns(prefix: &str) -> CsvSourceConfig {
    let mut columns = array_columns("state", "state", 6);
    columns.extend((0..6).map(|index| {
        (
            format!("covar.m{}{}", index + 1, index + 1),
            format!("covar_{}", index * 7),
        )
    }));
    columns.extend(vector_columns("post_fit_res", "postFitRes"));
    named(prefix, columns)
}

/// A `PixelLineFilterMsg` recorder (upstream
/// `pixelLineFilter.filtDataOutMsg`).
pub fn pixel_filter_columns(prefix: &str) -> CsvSourceConfig {
    let mut columns = array_columns("state", "state", 9);
    columns.extend(matrix_columns("covar", "covar", 9));
    columns.extend(array_columns("post_fit_res", "postFitRes", 6));
    named(prefix, columns)
}

/// A `HeadingFilterMsg` recorder (upstream `headingUKF.filtDataOutMsg`).
pub fn heading_filter_columns(prefix: &str) -> CsvSourceConfig {
    let mut columns = array_columns("state", "state", 5);
    columns.extend(matrix_columns("covar", "covar", 5));
    columns.extend(vector_columns("post_fit_res", "postFitRes"));
    named(prefix, columns)
}

/// Upstream `rwStateEffector.rwOutMsgs[i].recorder()`'s `u_current`.
pub fn rw_current_columns(index: usize) -> CsvSourceConfig {
    CsvSourceConfig::columns([(
        "applied_motor_torque_nm".to_string(),
        format!("RW{index}_u_current_Nm"),
    )])
}

/// The truth of `pull_outputs` for one spacecraft state.
#[derive(Clone, Copy, Debug)]
pub struct CameraTruth {
    /// Upstream `trueR_C` \[m\]: the spacecraft relative to Mars, camera
    /// frame.
    pub r_bn_c_m: Vector3<f64>,
    /// Upstream `trueCircles`: Mars's centre and apparent radius in the
    /// image \[pixels\].
    pub center_px: [f64; 2],
    pub radius_px: f64,
    /// Angle between the camera boresight and the direction to Mars \[rad\].
    pub off_pointing_rad: f64,
}

/// `[CB]` of the instrument camera.
pub fn dcm_cb() -> Matrix3<f64> {
    mrp_to_dcm(Vector3::from(CAMERA_MRP_CB))
}

/// Upstream's truth for the spacecraft `state` (Mars at the origin), with
/// the instrument camera of field of view `fov_rad`.
pub fn camera_truth(state: &SpacecraftStateMsg, fov_rad: f64) -> CameraTruth {
    let dcm_cn = dcm_cb() * mrp_to_dcm(state.sigma_bn);
    let r_bn_c_m = dcm_cn * state.position_m;
    let range_m = state.position_m.norm();
    let [width, height] = CAMERA_RESOLUTION_PX;
    // Upstream's `focal / pixelSize`, per axis.
    let d_x =
        f64::from(width) / (2.0 * (fov_rad * f64::from(width) / f64::from(height) / 2.0).tan());
    let d_y = f64::from(height) / (2.0 * (fov_rad / 2.0).tan());
    let center_px = [
        r_bn_c_m.x / r_bn_c_m.z * d_x + f64::from(width) / 2.0 - 0.5,
        r_bn_c_m.y / r_bn_c_m.z * d_y + f64::from(height) / 2.0 - 0.5,
    ];
    let radius_px = (R_MARS_M / range_m).asin().tan() * d_x;
    // The camera looks along +z_C; Mars is along -r_BN.
    let off_pointing_rad = safe_acos(-r_bn_c_m.z / range_m);
    CameraTruth {
        r_bn_c_m,
        center_px,
        radius_px,
        off_pointing_rad,
    }
}

/// [`camera_truth`] with upstream's 55-degree field of view.
pub fn instrument_truth(state: &SpacecraftStateMsg) -> CameraTruth {
    camera_truth(state, CAMERA_FOV_DEG.to_radians())
}

/// The relative position and velocity errors \[-\] of a filter state
/// `[r, v]` against the truth (upstream `nav_percentages`, without the 100).
pub fn relative_nav_errors(state: &[f64], truth: &SpacecraftStateMsg) -> (f64, f64) {
    let position = Vector3::new(state[0], state[1], state[2]);
    let velocity = Vector3::new(state[3], state[4], state[5]);
    (
        (position - truth.position_m).norm() / truth.position_m.norm(),
        (velocity - truth.velocity_mps).norm() / truth.velocity_mps.norm(),
    )
}

/// Mars's gravitational parameter \[m^3/s^2\].
pub fn mu_mars() -> f64 {
    Body::MarsBarycenter.mu_m3ps2()
}

/// Writes the valid limbs of `limbs` to `examples/output/scenarios/<stem>.csv`
/// (upstream's `limbRec` and `plot_limb`): one row per point of every
/// `every`-th valid limb (upstream `plot_limb` draws every 30th), and one row
/// with `NaN` points for each other one, as `time_ns, limb_index,
/// numLimbPoints, limbPoints_x, limbPoints_y`.
///
/// Schedule the limb log after the flight software at the camera rate (see
/// [`LIMB_LOG_PRIORITY`]): a limb is only valid in the tick its image is
/// processed, and recording 2000 points at every FSW step would hold
/// gigabytes.
pub fn write_limb_points(stem: &str, limbs: &MessageLog<OpNavLimbMsg>, every: usize) -> PathBuf {
    let valid: Vec<(u64, Vec<[f64; 2]>)> = limbs
        .times_nanos
        .iter()
        .zip(&limbs.values)
        .filter(|(_, limb)| limb.valid)
        .map(|(time, limb)| {
            let points = limb
                .active_limb_points()
                .iter()
                .map(|point| [point.x, point.y])
                .collect();
            (*time, points)
        })
        .collect();
    write_limbs(stem, &valid, every)
}

/// [`write_limb_points`] of valid limbs given as `(time [ns], points)`.
pub fn write_limbs(stem: &str, limbs: &[(u64, Vec<[f64; 2]>)], every: usize) -> PathBuf {
    let headers = [
        "time_ns",
        "limb_index",
        "numLimbPoints",
        "limbPoints_x",
        "limbPoints_y",
    ]
    .map(String::from)
    .to_vec();
    let mut rows = Vec::new();
    for (index, (time, points)) in limbs.iter().enumerate() {
        let row = |x: f64, y: f64| vec![*time as f64, index as f64, points.len() as f64, x, y];
        if index % every == 0 {
            rows.extend(points.iter().map(|point| row(point[0], point[1])));
        } else {
            rows.push(row(f64::NAN, f64::NAN));
        }
    }
    write_table(stem, &headers, &rows)
}
