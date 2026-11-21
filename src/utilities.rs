//! Numerical building blocks shared by the flight-software algorithms, ported
//! from upstream Basilisk's `architecture/utilities`.

pub mod astro_constants;
pub mod avs_eigen_support;
pub mod bisection;
pub mod brightness_temperature_solar_system;
pub mod bspline;
pub mod geodetic_conversion;
pub mod itu_ref_atmosphere;
pub mod lambert;
pub mod linear_algebra;
pub mod newton_raphson;
#[cfg(feature = "opencv")]
pub mod opencv_image;
pub mod orbital_motion;
pub mod rigid_body_kinematics;
pub mod signal_condition;
pub mod svd;
pub mod tle_handling;
pub mod ukf_utilities;
