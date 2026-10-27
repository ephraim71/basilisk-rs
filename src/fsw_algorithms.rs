//! Flight-software algorithms, grouped into the same families as upstream
//! Basilisk's `src/fswAlgorithms`.

pub mod att_control;
pub mod att_determination;
pub mod att_guidance;
pub mod config_data;
pub mod dv_guidance;
pub mod effector_interfaces;
pub mod formation_flying;
pub mod image_processing;
pub mod optical_navigation;
pub mod sensor_interfaces;
pub mod trans_determination;

// The paths these modules had before they were grouped by family.
pub use att_control::{mrp_feedback, mtb_momentum_management};
pub use att_determination::css_wls_est;
pub use att_guidance::{att_tracking_error, inertial_3d, sun_safe_point};
pub use effector_interfaces::{rw_motor_torque, rw_motor_voltage};
pub use sensor_interfaces::tam_comm;
