use nalgebra::{Matrix3, Vector3};
use serde::{Deserialize, Serialize};

/// Top-level vehicle properties consumed by flight-software controllers.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct VehicleConfigMsg {
    pub inertia_about_point_b_kg_m2: Matrix3<f64>,
    pub center_of_mass_body_m: Vector3<f64>,
    pub mass_kg: f64,
    pub current_adcs_state: u32,
}
