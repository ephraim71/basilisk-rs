//! Image processing: turn camera observations into navigation measurements.

#[cfg(feature = "opencv")]
pub mod center_radius_cnn;
pub mod horizon_op_nav;
#[cfg(feature = "opencv")]
pub mod hough_circles;
#[cfg(feature = "opencv")]
pub mod limb_finding;
pub mod pixel_line_converter;
