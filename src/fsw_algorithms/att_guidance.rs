//! Attitude guidance: build the reference attitude to track and the error from it.

pub mod att_ref_correction;
pub mod att_tracking_error;
pub mod celestial_two_body_point;
pub mod euler_rotation;
pub mod hill_point;
pub mod inertial_3d;
pub mod inertial_3d_spin;
pub mod location_pointing;
pub mod mrp_rotation;
pub mod one_axis_solar_array_point;
pub mod op_nav_point;
pub mod raster_manager;
pub mod simple_deadband;
pub mod sun_safe_point;
pub mod velocity_point;
