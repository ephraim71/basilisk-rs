use std::fmt::Debug;

use nalgebra::Vector3;

use super::GravityError;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GravityFrame {
    InertialInvariant,
    BodyFixed,
}

/// Numerical gravity strategy used by a [`super::GravBodyData`].
///
/// Models receive positions relative to the gravitating body. Body-fixed models
/// are rotated by the owning gravity effector before these methods are called.
pub trait GravityModel: Debug + Send {
    fn frame(&self) -> GravityFrame;

    fn potential_frame(&self) -> GravityFrame {
        self.frame()
    }

    fn gravitational_parameter_m3ps2(&self) -> f64;

    fn acceleration_mps2(&mut self, position_m: Vector3<f64>)
    -> Result<Vector3<f64>, GravityError>;

    /// Gravitational potential per unit spacecraft mass in J/kg.
    fn specific_potential_jpkg(&mut self, position_m: Vector3<f64>) -> Result<f64, GravityError>;
}

pub(crate) fn checked_radius(position_m: Vector3<f64>) -> Result<f64, GravityError> {
    let radius = position_m.norm();
    if !radius.is_finite() || radius == 0.0 {
        return Err(GravityError::SingularPosition);
    }
    Ok(radius)
}

pub(crate) fn validate_positive(parameter: &'static str, value: f64) -> Result<f64, GravityError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(GravityError::InvalidParameter { parameter, value });
    }
    Ok(value)
}
