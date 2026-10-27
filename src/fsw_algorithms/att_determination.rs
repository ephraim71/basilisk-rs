//! Attitude determination: estimate the sun heading or the full attitude from sensors.

pub mod css_wls_est;
pub mod heading_sukf;
pub mod inertial_ukf;
pub mod okeefe_ekf;
pub mod sunline_ekf;
pub mod sunline_ephem;
pub mod sunline_sekf;
pub mod sunline_sukf;
pub mod sunline_ukf;
