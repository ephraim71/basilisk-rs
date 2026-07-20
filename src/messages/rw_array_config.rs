use nalgebra::Vector3;

use super::MAX_EFF_COUNT;

/// Reaction-wheel array configuration used by flight-software modules.
#[derive(Clone, Debug, PartialEq)]
pub struct RwArrayConfigMsg {
    pub num_rw: usize,
    /// One body-frame spin-axis vector per wheel (`GsMatrix_B` upstream).
    pub spin_axes_body: [Vector3<f64>; MAX_EFF_COUNT],
    pub spin_axis_inertias_kg_m2: [f64; MAX_EFF_COUNT],
    pub max_motor_torques_nm: [f64; MAX_EFF_COUNT],
}

impl Default for RwArrayConfigMsg {
    fn default() -> Self {
        Self {
            num_rw: 0,
            spin_axes_body: [Vector3::zeros(); MAX_EFF_COUNT],
            spin_axis_inertias_kg_m2: [0.0; MAX_EFF_COUNT],
            max_motor_torques_nm: [0.0; MAX_EFF_COUNT],
        }
    }
}
