#[cfg(feature = "opencv")]
pub mod camera;
pub mod coarse_sun_sensor;
pub mod gps;
pub mod hinged_rigid_body_motor_sensor;
pub mod imu_sensor;
pub mod magnetometer;
pub mod navigation;
pub mod pinhole_camera;
pub mod planet_heading;
pub mod planet_nav;
pub mod simple_mass_props;
pub mod simple_volt_estimator;
pub mod star_tracker;

/// Default random seed of a noisy sensor: a hash of its name, so differently
/// named instances draw different streams and a rerun repeats its own.
pub(crate) fn seed_from_name(name: &str) -> u64 {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};

    let mut hasher = DefaultHasher::new();
    name.hash(&mut hasher);
    hasher.finish()
}
