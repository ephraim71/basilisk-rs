use nalgebra::Vector3;

use super::gravity_model::{checked_radius, validate_positive};
use super::{GravityError, GravityFrame, GravityModel};

#[derive(Clone, Debug)]
pub struct PointMassGravityModel {
    mu_m3ps2: f64,
}

impl PointMassGravityModel {
    pub fn new(mu_m3ps2: f64) -> Result<Self, GravityError> {
        Ok(Self {
            mu_m3ps2: validate_positive("mu_m3ps2", mu_m3ps2)?,
        })
    }

    pub fn mu_m3ps2(&self) -> f64 {
        self.mu_m3ps2
    }
}

impl GravityModel for PointMassGravityModel {
    fn frame(&self) -> GravityFrame {
        GravityFrame::InertialInvariant
    }

    fn gravitational_parameter_m3ps2(&self) -> f64 {
        self.mu_m3ps2
    }

    fn acceleration_mps2(
        &mut self,
        position_m: Vector3<f64>,
    ) -> Result<Vector3<f64>, GravityError> {
        let radius = checked_radius(position_m)?;
        Ok(-self.mu_m3ps2 * position_m / radius.powi(3))
    }

    fn specific_potential_jpkg(&mut self, position_m: Vector3<f64>) -> Result<f64, GravityError> {
        Ok(-self.mu_m3ps2 / checked_radius(position_m)?)
    }
}
